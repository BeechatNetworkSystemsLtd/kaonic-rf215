// SPDX-License-Identifier: GPL-2.0
/*
 * Diagnostics.
 *
 * A debugfs view under /sys/kernel/debug/<spi device>/ of what the chip and
 * the engine are doing: state, counters and registers read back.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Forward Declarations ******************************************************/

/* debugfs: engine state of the enabled transceivers. */
static int rf215_state_show(struct seq_file* s, void* unused);
/* debugfs: the core counters. */
static int rf215_stats_show(struct seq_file* s, void* unused);
/* debugfs: configuration read back from the chip. */
static int rf215_regs_show(struct seq_file* s, void* unused);

/******************************************************************************/
/** Data **********************************************************************/

DEFINE_SHOW_ATTRIBUTE(rf215_state);

DEFINE_SHOW_ATTRIBUTE(rf215_stats);

DEFINE_SHOW_ATTRIBUTE(rf215_regs);

/******************************************************************************/
/** Public Functions **********************************************************/

void rf215_debugfs_init(struct rf215_spi* rf) {
    rf->debugfs = debugfs_create_dir(dev_name(&rf->spi->dev), NULL);

    debugfs_create_file("state", 0444, rf->debugfs, rf, &rf215_state_fops);
    debugfs_create_file("stats", 0444, rf->debugfs, rf, &rf215_stats_fops);
    debugfs_create_file("regs", 0444, rf->debugfs, rf, &rf215_regs_fops);
}

/******************************************************************************/
/** Static Functions **********************************************************/

static int rf215_state_show(struct seq_file* s, void* unused) {
    struct rf215_spi* rf = s->private;
    unsigned int i;

    (void)unused;

    rt_mutex_lock(&rf->lock);
    seq_printf(s, "part_number 0x%02x\n", rf->core.part_number);
    seq_printf(s, "version     0x%02x\n", rf->core.version);

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        const struct rf215_iface* ifc = &rf->iface[i];

        if (!rf215_iface_enabled(ifc)) {
            continue;
        }

        seq_printf(s, "\n[%s]\n", rf215_iface_name(ifc));
        seq_printf(s, "  band       %s\n", i ? "2.4 GHz" : "sub-GHz");
        seq_printf(s, "  step       %s\n", rf215_step_name(&rf->core, i));
        seq_printf(s, "  frequency  %u Hz\n", ifc->phy.freq_hz);
        seq_printf(s, "  ofdm_mcs   %u\n", ifc->phy.ofdm.mcs);
        seq_printf(s, "  tx_power   %u\n", ifc->phy.tx_power);
        seq_printf(s, "  lna_bypass %d\n", ifc->phy.ext_lna_bypass);
        seq_printf(s, "  receiving  %d\n", rf->core.trx[i].receiving);
        seq_printf(s, "  armed      %d\n", rf->core.trx[i].radio.armed);
    }
    rt_mutex_unlock(&rf->lock);

    return 0;
}

static int rf215_stats_show(struct seq_file* s, void* unused) {
    struct rf215_spi* rf = s->private;
    const struct rf215_stats* st;

    (void)unused;

    rt_mutex_lock(&rf->lock);
    st = rf215_stats(&rf->core);
    seq_printf(s, "xfers      %u\n", st->xfers);
    seq_printf(s, "xfer_errs  %u\n", st->xfer_errs);
    seq_printf(s, "irqs       %u\n", st->irqs);
    seq_printf(s, "timeouts   %u\n", st->timeouts);
    seq_printf(s, "rx_starts  %u\n", st->rx_starts);
    seq_printf(s, "rx_ends    %u\n", st->rx_ends);
    seq_printf(s, "rx_overruns %u\n", st->rx_overruns);
    seq_printf(s, "rx_rearms  %u\n", st->rx_rearms);
    seq_printf(s, "rx_revivals %u\n", st->rx_revivals);
    seq_printf(s, "rx_stale   %u\n", st->rx_stale);
    seq_printf(s, "cca_busy   %u\n", st->cca_busy);
    seq_printf(s, "tx_dropped %u\n", st->tx_dropped);
    seq_printf(s, "events     %u\n", st->events);
    rt_mutex_unlock(&rf->lock);

    return 0;
}

/* Reads the configuration back from the chip; the lock keeps the engine still meanwhile. */
static int rf215_regs_show(struct seq_file* s, void* unused) {
    static const struct {
        const char* name;
        u16 reg;
    } regs[] = {
        {
            "AUXS     ",
            0x0101,
        },
        {
            "STATE    ",
            0x0102,
        },
        {
            "CS       ",
            0x0104,
        },
        {
            "CCF0L    ",
            0x0105,
        },
        {
            "CCF0H    ",
            0x0106,
        },
        {
            "CNL      ",
            0x0107,
        },
        {
            "CNM      ",
            0x0108,
        },
        {
            "RXBWC    ",
            0x0109,
        },
        {
            "RXDFE    ",
            0x010A,
        },
        {
            "AGCC     ",
            0x010B,
        },
        {
            "AGCS     ",
            0x010C,
        },
        {
            "EDD      ",
            0x010F,
        },
        {
            "TXCUTC   ",
            0x0112,
        },
        {
            "TXDFE    ",
            0x0113,
        },
        {
            "PAC      ",
            0x0114,
        },
        {
            "PADFE    ",
            0x0116,
        },
        {
            "PC       ",
            0x0301,
        },
        {
            "OFDMPHRTX",
            0x030C,
        },
        {
            "OFDMC    ",
            0x030E,
        },
        {
            "OFDMSW   ",
            0x030F,
        },
    };
    struct rf215_spi* rf = s->private;
    unsigned int i, t;

    (void)unused;

    rt_mutex_lock(&rf->lock);

    for (t = 0; t < RF215_TRX_COUNT; t++) {
        if (!rf215_iface_enabled(&rf->iface[t])) {
            continue;
        }

        seq_printf(s, "[%s]\n", t ? "rf24" : "rf09");

        for (i = 0; i < ARRAY_SIZE(regs); i++) {
            u16 reg = regs[i].reg + (u16)(t * RF215_TRX_REG_STRIDE);
            struct rf215_xfer xfer = {
                .op = RF215_XFER_READ,
                .reg = reg,
                .buf = rf->scratch,
                .len = 1,
            };

            if (rf215_spi_xfer(rf, &xfer)) {
                seq_printf(s, "  %s 0x%04x  <read failed>\n", regs[i].name, reg);
            } else {
                seq_printf(s, "  %s 0x%04x  0x%02x\n", regs[i].name, reg, rf->scratch[0]);
            }
        }
    }

    rt_mutex_unlock(&rf->lock);

    return 0;
}
