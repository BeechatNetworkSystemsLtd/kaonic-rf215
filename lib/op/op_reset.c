/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Reset: pulse RST, identify the part, then arm interrupts on both
 * transceivers and clear whatever the chip already had pending.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_core.h"
#include "rf215_op.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* RST must be held low for at least 625 ns (datasheet 6.2). */
#define RESET_PULSE_US (50u)

/* No SPI access is valid until the chip reaches TRXOFF, 1 ms after RST release. */
#define RESET_SETTLE_US (1000u)

/* Limit on the wait for WAKEUP; a slow reset line can need tens of milliseconds. */
#define RESET_WAKEUP_TIMEOUT_US (50000u)

#define RESET_WAKEUP_POLL_US (100u)

/* Interrupt pin: mask mode on, active high. The host supplies the pad drive (DRV). */
#define RF_CFG_BASE (RF215_RG_RF_CFG_IRQMM)

/******************************************************************************/
/** Forward Declarations ******************************************************/

static enum rf215_step_result
reset_poll_wakeup(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);

/******************************************************************************/
/** Static Functions **********************************************************/

/* Reports the chip as ready. */
static enum rf215_step_result
reset_ready(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    /* Reset takes both transceivers off air; the engine restores a wanted receiver. */
    for (uint8_t i = 0u; i < RF215_TRX_COUNT; ++i) {
        dev->trx[i].radio.irqs = 0u;
        dev->trx[i].baseband.amedt_valid = false;
        dev->trx[i].baseband.amcs_valid = false;
        rf215_rx_off_air(&dev->trx[i]);
    }

    /* A commanded reset answers any chip reset seen before it. */
    dev->chip_reset = false;
    dev->irq_empty = 0u;

    return rf215_emit(op, RF215_EVT_READY, RF215_OK, NULL);
}

/* Reads all four IRQS registers; the chip holds its interrupt pin until each is read. */
static enum rf215_step_result
reset_clear_irqs(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;

    return rf215_read(op, out, RF215_RG_RF09_IRQS, 4u, reset_ready);
}

/* Arms the claimed baseband interrupts on BBC1. */
static enum rf215_step_result
reset_irqm_bb24(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(&dev->trx[1], RF215_RG_BBCX_IRQM),
                        RF215_BASEBAND_IRQ_USED,
                        reset_clear_irqs);
}

/* Arms the claimed baseband interrupts on BBC0. */
static enum rf215_step_result
reset_irqm_bb09(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(&dev->trx[0], RF215_RG_BBCX_IRQM),
                        RF215_BASEBAND_IRQ_USED,
                        reset_irqm_bb24);
}

/* Arms the claimed radio interrupts on RF24. */
static enum rf215_step_result
reset_irqm_radio24(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return rf215_write8(op,
                        out,
                        rf215_radio_reg(&dev->trx[1], RF215_RG_RFXX_IRQM),
                        RF215_RADIO_IRQ_USED,
                        reset_irqm_bb09);
}

/* Arms the claimed radio interrupts on RF09. */
static enum rf215_step_result
reset_irqm_radio09(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return rf215_write8(op,
                        out,
                        rf215_radio_reg(&dev->trx[0], RF215_RG_RFXX_IRQM),
                        RF215_RADIO_IRQ_USED,
                        reset_irqm_radio24);
}

/* Records the chip version and moves on to the interrupt masks. */
static enum rf215_step_result
reset_have_vn(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    dev->version = op->buf[0];

    const uint8_t cfg = (uint8_t)(RF_CFG_BASE | (dev->pad_drive & RF215_RG_RF_CFG_DRV_MASK));

    return rf215_write8(op, out, RF215_RG_RF_CFG, cfg, reset_irqm_radio09);
}

/* Identifies the part, or fails on an unknown one. */
static enum rf215_step_result
reset_have_pn(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    switch (op->buf[0]) {
        case RF215_PN_AT86RF215:
        case RF215_PN_AT86RF215IQ:
        case RF215_PN_AT86RF215M:
            dev->part_number = (enum rf215_pn)op->buf[0];
            break;
        default:
            RF215_LOGE(dev, "reset: unknown part number");
            return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_NODEV, NULL);
    }

    return rf215_read(op, out, RF215_RG_RF_VN, 1u, reset_have_vn);
}

/* Reads the part number (RF_PN). */
static enum rf215_step_result
reset_read_pn(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;

    return rf215_read(op, out, RF215_RG_RF_PN, 1u, reset_have_pn);
}

/* Reads the part number once awake, or after the poll deadline. */
static enum rf215_step_result
reset_check_wakeup(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_RADIO_IRQ_WAKEUP) != 0u) {
        return reset_read_pn(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "reset: no wake-up reported, reading the chip anyway");
        return reset_read_pn(dev, op, out);
    }

    return rf215_delay(dev, op, RESET_WAKEUP_POLL_US, reset_poll_wakeup);
}

/* Reads RF09_IRQS to see whether the chip reports WAKEUP. */
static enum rf215_step_result
reset_poll_wakeup(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return rf215_read(op, out, dev->trx[0].info->radio_irqs, 1u, reset_check_wakeup);
}

static enum rf215_step_result
reset_release(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    dev->port.set_reset(dev->port.ctx, false);

    /* In reset every register but STATE reads 0xFF, so poll IRQS.WAKEUP (section 4.1.3)
     * before reading the part number; the fixed settle is the floor. */
    rf215_set_deadline(dev, op, RESET_WAKEUP_TIMEOUT_US);

    return rf215_delay(dev, op, RESET_SETTLE_US, reset_poll_wakeup);
}

/******************************************************************************/
/** Public Functions **********************************************************/

enum rf215_step_result
rf215_op_reset_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    dev->part_number = RF215_PN_UNKNOWN;
    dev->version = 0u;

    dev->port.set_reset(dev->port.ctx, true);

    return rf215_delay(dev, op, RESET_PULSE_US, reset_release);
}

/******************************************************************************/
/** Step Table ****************************************************************/

/*
 *   rf215_op_reset_start          assert RST
 *            |
 *            |  50 us
 *            v
 *      reset_release              release RST
 *            |
 *            |  1 ms
 *            v
 *    reset_poll_wakeup <-------+  read RF09_IRQS
 *            |                 |
 *            v                 |  no WAKEUP yet: 100 us
 *    reset_check_wakeup -------+
 *            |
 *            |  WAKEUP seen, or 50 ms gone
 *            v
 *      reset_read_pn
 *            |
 *            v
 *      reset_have_pn ---- unknown part ----> ERROR
 *            |
 *            v
 *      reset_have_vn              write RF_CFG
 *            |
 *            v
 *    reset_irqm_radio09 -> reset_irqm_radio24 -> reset_irqm_bb09 -> reset_irqm_bb24
 *                                                                         |
 *            +------------------------------------------------------------+
 *            v
 *     reset_clear_irqs            read all four IRQS
 *            |
 *            v
 *       reset_ready ----> READY
 */

static const struct rf215_step_name steps[] = {
    {
        RF215_STEP(rf215_op_reset_start),
    },
    {
        RF215_STEP(reset_release),
    },
    {
        RF215_STEP(reset_poll_wakeup),
    },
    {
        RF215_STEP(reset_check_wakeup),
    },
    {
        RF215_STEP(reset_read_pn),
    },
    {
        RF215_STEP(reset_have_pn),
    },
    {
        RF215_STEP(reset_have_vn),
    },
    {
        RF215_STEP(reset_irqm_radio09),
    },
    {
        RF215_STEP(reset_irqm_radio24),
    },
    {
        RF215_STEP(reset_irqm_bb09),
    },
    {
        RF215_STEP(reset_irqm_bb24),
    },
    {
        RF215_STEP(reset_clear_irqs),
    },
    {
        RF215_STEP(reset_ready),
    },
};

const struct rf215_step_name* rf215_op_reset_steps(size_t* count) {
    *count = sizeof(steps) / sizeof(steps[0]);

    return steps;
}
