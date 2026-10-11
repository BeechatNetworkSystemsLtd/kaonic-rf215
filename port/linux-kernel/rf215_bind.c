// SPDX-License-Identifier: GPL-2.0
/*
 * Binds a probed SPI device to a pair of transceivers.
 *
 * Module parameters, the reset and interrupt wiring, and the registration
 * and teardown of both interfaces.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* The radio settings start at the core's defaults (rf215_phy_config_default);
 * the device tree describes only the board, see rf215_read_board(). Without
 * properties the board is a Kaonic 1S: an external +12 dB LNA on sub-GHz and
 * none on 2.4 GHz, both transceivers in use. */
#define RF215_DEFAULT_AGC_MAP 2

/******************************************************************************/
/** Forward Declarations ******************************************************/

/*
 * Probe runs these in order; each one either completes or returns the error
 * that ends the probe. Resources are devres-managed, so a failure half way
 * needs no unwinding here.
 */

/* Allocates the driver state and the bus buffers. */
static struct rf215_spi* rf215_alloc(struct spi_device* spi);
/* Locks, work items, timer, front end and the two interfaces. */
static void rf215_init_state(struct rf215_spi* rf);
/* Default radio settings of one interface. */
static void rf215_init_phy(struct rf215_iface* ifc);
/* Reset line, engine worker, and the teardown that stops them. */
static int rf215_init_worker(struct rf215_spi* rf);
/* SPI mode and the core engine with its port callbacks. */
static int rf215_init_core(struct rf215_spi* rf);
/* Reads what the device tree says about the board around the chip. */
static int rf215_read_board(struct rf215_spi* rf);
/* Receive buffers of the enabled transceivers. */
static int rf215_alloc_rx(struct rf215_spi* rf);
/* The chip's interrupt line, level triggered. */
static int rf215_init_irq(struct rf215_spi* rf);
/* Resets the chip, configures the enabled transceivers and registers their
 * interfaces; returns how many came up. */
static int rf215_bring_up(struct rf215_spi* rf, unsigned int* attached);

/* Stops every pass, timer and recovery, and keeps them stopped. */
static void rf215_halt(struct rf215_spi* rf);
/* Stops anything that could still reach the device, in teardown order. */
static void rf215_quiesce(void* data);
/* Binds a probed SPI device. */
static int rf215_probe(struct spi_device* spi);
/* Unbinds the device. */
static void rf215_remove(struct spi_device* spi);

/******************************************************************************/
/** Data **********************************************************************/

static char* mode = "raw";

module_param(mode, charp, 0444);

MODULE_PARM_DESC(mode,
                 "Upward interface: \"raw\" for plain network devices carrying whole PHY frames "
                 "(up to 2047 bytes), \"wpan\" for 802.15.4 PHYs via mac802154 (123 bytes)");

static const struct of_device_id rf215_of_match[] = {
    {
        .compatible = "beechat,rf215",
    },
    {
        .compatible = "microchip,at86rf215",
    },
    {},
};

/* One id per of_device_id compatible (the part after the comma); module
 * autoloading needs it. */
static const struct spi_device_id rf215_spi_ids[] = {
    {
        "rf215",
        0,
    },
    {
        "at86rf215",
        0,
    },
    {},
};

static struct spi_driver rf215_driver = {
	.driver = {
		.name = "kaonic_rf215",
		.of_match_table = rf215_of_match,
	},
	.id_table = rf215_spi_ids,
	.probe = rf215_probe,
	.remove = rf215_remove,
};

/******************************************************************************/
/** Static Functions **********************************************************/

/******************************************************************************/
/** Probe Stages **************************************************************/

static struct rf215_spi* rf215_alloc(struct spi_device* spi) {
    struct rf215_spi* rf;

    rf = devm_kzalloc(&spi->dev, sizeof(*rf), GFP_KERNEL);
    if (!rf) {
        return NULL;
    }

    rf->spi = spi;

    /* Bus buffers are separate allocations so that a DMA unmap cannot
     * invalidate a cache line shared with another field. */
    rf->hdr = devm_kzalloc(&spi->dev, RF215_SPI_HDR_LEN + RF215_SPI_SMALL_MAX, GFP_KERNEL);
    rf->small = devm_kzalloc(&spi->dev, RF215_SPI_HDR_LEN + RF215_SPI_SMALL_MAX, GFP_KERNEL);
    rf->scratch = devm_kzalloc(&spi->dev, RF215_SCRATCH_LEN, GFP_KERNEL);
    if (!rf->hdr || !rf->small || !rf->scratch) {
        return NULL;
    }

    return rf;
}

static void rf215_init_state(struct rf215_spi* rf) {
    unsigned int i;

    rf->fe = (mode && !strcmp(mode, "wpan")) ? rf215_mac_frontend() : rf215_raw_frontend();

    rt_mutex_init(&rf->lock);
    mutex_init(&rf->cfg_lock);
    kthread_init_work(&rf->work, rf215_work_fn);
    kthread_init_delayed_work(&rf->irq_work, rf215_irq_work_fn);
    INIT_WORK(&rf->recover, rf215_recover_fn);
    hrtimer_init(&rf->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    rf->timer.function = rf215_timer_fn;

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        struct rf215_iface* ifc = &rf->iface[i];

        ifc->rf = rf;
        ifc->band = (enum rf215_band)i;
        mutex_init(&ifc->op_lock);
        init_completion(&ifc->op_done);
        /* Nothing measured yet; 0 dBm is a valid reading, so mark invalid. */
        ifc->rx_rssi = RF215_RSSI_INVALID;
        ifc->ed_dbm = RF215_RSSI_INVALID;
        rf215_init_phy(ifc);
    }
}

static void rf215_init_phy(struct rf215_iface* ifc) {
    rf215_phy_config_default(&ifc->phy, ifc->band);

    /* The 802.15.4 stack relies on the baseband for the frame check sequence. */
    ifc->phy.fcs = rf215_is_mac_fe(ifc->rf);
}

static int rf215_init_worker(struct rf215_spi* rf) {
    struct device* dev = &rf->spi->dev;
    int ret;

    /* Held in reset until the core's reset operation. Must be taken before
     * rf215_quiesce() is registered: devres then frees the line after that
     * action has asserted reset through it. */
    rf->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(rf->reset_gpio)) {
        return dev_err_probe(dev, PTR_ERR(rf->reset_gpio), "failed to get reset gpio\n");
    }

    /* Runs at the interrupt thread's priority: a timer-driven engine pass is
     * as urgent as an interrupt-driven one. */
    rf->worker = kthread_create_worker(0, "rf215/%s", dev_name(dev));
    if (IS_ERR(rf->worker)) {
        return PTR_ERR(rf->worker);
    }

    sched_set_fifo(rf->worker->task);

    /* Must be registered before the interrupt, so devres runs it after the
     * interrupt is released and no timer outlives the device. */
    ret = devm_add_action_or_reset(dev, rf215_quiesce, rf);
    if (ret) {
        return ret;
    }

    return 0;
}

static int rf215_init_core(struct rf215_spi* rf) {
    struct spi_device* spi = rf->spi;
    struct rf215_iq_config iq;
    struct rf215_port port;
    int ret;

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret) {
        return dev_err_probe(&spi->dev, ret, "spi setup failed\n");
    }

    port.ctx = rf;
    port.set_reset = rf215_spi_set_reset;
    port.now = rf215_spi_now;
    port.log = rf215_spi_log;

    ret = rf215_init(&rf->core, &port);
    if (ret) {
        return dev_err_probe(&spi->dev, ret, "core init failed\n");
    }

    /* Two nodes that collide must not back off in step. */
    rf215_seed_backoff(&rf->core, get_random_u32());
    rf215_iq_config_default(&iq);
    rf215_set_iq_config(&rf->core, &iq);

    ret = rf215_read_board(rf);
    if (ret) {
        return ret;
    }

    return rf215_alloc_rx(rf);
}

/* Reads the board description (transceivers in use, pad drive, front ends)
 * from the device tree. Per-band properties list sub-GHz first; an absent
 * property keeps the default. */
static int rf215_read_board(struct rf215_spi* rf) {
    static const char* const disable[RF215_TRX_COUNT] = {
        [RF215_BAND_09] = "rf09-disable",
        [RF215_BAND_24] = "rf24-disable",
    };
    struct device* dev = &rf->spi->dev;
    u32 bypass[RF215_TRX_COUNT] = {
        [RF215_BAND_09] = 0,
        [RF215_BAND_24] = 1,
    };
    u32 map = RF215_DEFAULT_AGC_MAP;
    u32 drive_ma = 8;
    unsigned int i;

    device_property_read_u32_array(dev, "ext-lna-bypass", bypass, ARRAY_SIZE(bypass));
    device_property_read_u32(dev, "agc-gain-map", &map);
    device_property_read_u32(dev, "pad-drive-milliamp", &drive_ma);

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        rf->iface[i].phy.ext_lna_bypass = bypass[i] != 0;
        rf->iface[i].phy.agc_map = (u8)min(map, 2u);
        rf215_set_band_enabled(
            &rf->core, rf->iface[i].band, !device_property_read_bool(dev, disable[i]));
    }

    if (!rf215_iface_enabled(&rf->iface[RF215_BAND_09])
        && !rf215_iface_enabled(&rf->iface[RF215_BAND_24])) {
        return dev_err_probe(dev, -EINVAL, "both transceivers are disabled\n");
    }

    /* RF_CFG.DRV counts in steps of 2 mA from 2 mA. Applied by the reset. */
    rf215_set_pad_drive(&rf->core, (u8)(clamp(drive_ma, 2u, 8u) / 2u - 1u));

    return 0;
}

/* Separate allocations, like the bus buffers; a disabled transceiver gets none. */
static int rf215_alloc_rx(struct rf215_spi* rf) {
    unsigned int i;

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        struct rf215_iface* ifc = &rf->iface[i];

        if (!rf215_iface_enabled(ifc)) {
            continue;
        }

        ifc->rx_buf = devm_kzalloc(&rf->spi->dev, RF215_RX_BUF_LEN, GFP_KERNEL);
        if (!ifc->rx_buf) {
            return -ENOMEM;
        }
    }

    return 0;
}

static int rf215_init_irq(struct rf215_spi* rf) {
    struct spi_device* spi = rf->spi;
    unsigned int irq_flags;
    int ret;

    if (spi->irq <= 0) {
        return dev_err_probe(&spi->dev, -EINVAL, "no interrupt assigned\n");
    }

    /* The line stays asserted until all four status registers are read, so it
     * must be level triggered: an edge trigger loses interrupts raised while
     * it is high. An unset or edge type from the device tree becomes level-high. */
    irq_flags = irq_get_trigger_type(spi->irq);
    if (irq_flags == IRQ_TYPE_NONE || (irq_flags & IRQ_TYPE_EDGE_BOTH)) {
        irq_flags = IRQF_TRIGGER_HIGH;
    }

    ret = devm_request_threaded_irq(&spi->dev,
                                    spi->irq,
                                    NULL,
                                    rf215_irq_thread,
                                    irq_flags | IRQF_ONESHOT,
                                    dev_name(&spi->dev),
                                    rf);
    if (ret) {
        return dev_err_probe(&spi->dev, ret, "failed to request irq\n");
    }

    return 0;
}

/* A disabled transceiver is skipped; one that fails is logged and skipped. The
 * probe fails only when none comes up. */
static int rf215_bring_up(struct rf215_spi* rf, unsigned int* attached) {
    struct rf215_iface* chip = rf215_chip_iface(rf);
    const struct rf215_req reset_req = {
        .kind = RF215_REQ_RESET,
        .band = chip->band,
    };
    struct device* dev = &rf->spi->dev;
    unsigned int i;
    int ret;

    /* One reset covers the whole chip, so it runs on one transceiver. */
    ret = rf215_run_op(chip, &reset_req);
    if (ret) {
        dev_err(dev, "reset failed: %d\n", ret);
        return ret;
    }

    *attached = 0;

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        struct rf215_iface* ifc = &rf->iface[i];
        const char* band = (i == RF215_BAND_24) ? "rf24" : "rf09";

        if (!rf215_iface_enabled(ifc)) {
            dev_info(dev, "%s disabled\n", band);
            continue;
        }

        ret = rf215_configure(ifc);
        if (ret) {
            dev_err(dev, "%s configuration failed: %d\n", band, ret);
            continue;
        }

        /* The kernel numbers the interface; udev gives the stable name. */
        ret = rf->fe->attach(ifc, "rf%d");
        if (ret) {
            dev_err(dev, "failed to register %s: %d\n", band, ret);
            continue;
        }

        dev_info(dev,
                 "%s: %s, %u Hz, MR-OFDM option 1 MCS %u\n",
                 rf215_iface_name(ifc),
                 rf->fe->name,
                 ifc->phy.freq_hz,
                 ifc->phy.ofdm.mcs);
        (*attached)++;
    }

    if (!*attached) {
        dev_err(dev, "no transceiver could be registered\n");
        return -ENODEV;
    }

    return 0;
}

/******************************************************************************/
/** Teardown ******************************************************************/

/* Stops every pass, timer and recovery. The stopping flag must be set under the
 * engine lock before the cancels: a pass arms the timer and the timer queues a
 * pass, so cancelling alone leaves one of them pending. */
static void rf215_halt(struct rf215_spi* rf) {
    rt_mutex_lock(&rf->lock);
    rf->stopping = true;
    rt_mutex_unlock(&rf->lock);

    cancel_work_sync(&rf->recover);
    hrtimer_cancel(&rf->timer);
    kthread_cancel_delayed_work_sync(&rf->irq_work);
    kthread_cancel_work_sync(&rf->work);
}

static void rf215_quiesce(void* data) {
    struct rf215_spi* rf = data;

    rf215_halt(rf);
    kthread_destroy_worker(rf->worker);

    gpiod_set_value_cansleep(rf->reset_gpio, 1);
}

/******************************************************************************/
/** Driver ********************************************************************/

static int rf215_probe(struct spi_device* spi) {
    struct rf215_spi* rf;
    unsigned int attached;
    int ret;

    rf = rf215_alloc(spi);
    if (!rf) {
        return -ENOMEM;
    }

    spi_set_drvdata(spi, rf);
    rf215_init_state(rf);

    ret = rf215_init_worker(rf);
    if (ret) {
        return ret;
    }

    ret = rf215_init_core(rf);
    if (ret) {
        return ret;
    }

    ret = rf215_init_irq(rf);
    if (ret) {
        return ret;
    }

    ret = rf215_bring_up(rf, &attached);
    if (ret) {
        return ret;
    }

    rf215_debugfs_init(rf);

    dev_info(&spi->dev,
             "AT86RF215 part 0x%02x version 0x%02x, %u of %u transceivers\n",
             rf->core.part_number,
             rf->core.version,
             attached,
             RF215_TRX_COUNT);

    return 0;
}

static void rf215_remove(struct spi_device* spi) {
    struct rf215_spi* rf = spi_get_drvdata(spi);
    unsigned int i;

    debugfs_remove_recursive(rf->debugfs);

    /* Order matters: stop the engine and release the interrupt before
     * detaching, because the threaded handler delivers frames straight into
     * the interfaces. */
    rt_mutex_lock(&rf->lock);
    rf->stopping = true;
    rt_mutex_unlock(&rf->lock);

    devm_free_irq(&spi->dev, spi->irq, rf);
    rf215_halt(rf);

    /* Both settings directories first: iq/ of one interface reads the other's device. */
    for (i = 0; i < RF215_TRX_COUNT; i++) {
        rf215_sysfs_detach(&rf->iface[i]);
    }

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        rf->fe->detach(&rf->iface[i]);
    }

    gpiod_set_value_cansleep(rf->reset_gpio, 1);
}

/******************************************************************************/
/** Module ********************************************************************/

MODULE_DEVICE_TABLE(of, rf215_of_match);
MODULE_DEVICE_TABLE(spi, rf215_spi_ids);
module_spi_driver(rf215_driver);
MODULE_DESCRIPTION("Microchip AT86RF215 sub-GHz / 2.4 GHz transceiver");
MODULE_AUTHOR("Beechat Network Systems Ltd.");
MODULE_LICENSE("GPL");
