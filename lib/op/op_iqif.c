/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * I/Q data interface configuration.
 *
 * Two consecutive registers carry every setting (RF_IQIFC0 and RF_IQIFC1),
 * so they go out in one transfer; the three status bits come back in one
 * read of RF_IQIFC0..2 afterwards. The chip mode decides which basebands
 * stay in the frame path: with the interface on, the transceivers it serves
 * stream samples to an external baseband and the SPI frame buffers are out
 * of use for them. Changing the mode is only allowed with the affected
 * transceivers in TRXOFF; the host is expected to have taken them off air.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_core.h"
#include "rf215_op.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Static Functions **********************************************************/

/* Records the interface status and reports completion. */
static enum rf215_step_result
iqif_have_status(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    dev->iq_status.sync_failure = (op->buf[0] & 0x40u) != 0u;
    dev->iq_status.receiver_failsafe = (op->buf[1] & 0x80u) != 0u;
    dev->iq_status.synchronised = (op->buf[2] & 0x80u) != 0u;

    return rf215_emit(op, RF215_EVT_IQIF_DONE, RF215_OK, NULL);
}

static enum rf215_step_result
iqif_read_back(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;

    return rf215_read(op, out, RF215_RG_RF_IQIFC0, 3u, iqif_have_status);
}

/******************************************************************************/
/** Public Functions **********************************************************/

enum rf215_step_result
rf215_op_iqif_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_iq_config* iq = &dev->iq;
    const uint8_t mode = (uint8_t)iq->chip_mode;

    if ((mode != RF215_CHIP_MODE_BBRF) && (mode != RF215_CHIP_MODE_RF)
        && (mode != RF215_CHIP_MODE_BBRF09) && (mode != RF215_CHIP_MODE_BBRF24)) {
        RF215_LOGE(dev, "iqif: unknown chip mode");
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_INVAL, NULL);
    }

    /* Chip-wide change, made with both transceivers off air: drop both receive states. */
    for (uint8_t i = 0u; i < RF215_TRX_COUNT; ++i) {
        rf215_rx_off_air(&dev->trx[i]);
    }

    /* IQIFC0: EXTLB 7, SF 6 (read only), DRV 5:4, CMV 3:2, CMV1V2 1, EEC 0. */
    uint8_t iqifc0 = 0u;

    iqifc0 |= iq->external_loopback ? 0x80u : 0u;
    iqifc0 |= (uint8_t)((iq->drive_ma & 0x03u) << 4);
    iqifc0 |= (uint8_t)((iq->common_mode & 0x03u) << 2);
    iqifc0 |= iq->common_mode_1v2 ? 0x02u : 0u;
    iqifc0 |= iq->embedded_tx_control ? 0x01u : 0u;

    /* IQIFC1: FAILSF 7 (read only), CHPM 6:4, SKEWDRV 1:0. */
    uint8_t iqifc1 = 0u;

    iqifc1 |= (uint8_t)(mode << 4);
    iqifc1 |= iq->skew & 0x03u;

    op->buf[0] = iqifc0;
    op->buf[1] = iqifc1;
    op->next = iqif_read_back;

    out->op = RF215_XFER_WRITE;
    out->reg = RF215_RG_RF_IQIFC0;
    out->buf = op->buf;
    out->len = 2u;

    return RF215_STEP_XFER;
}

/******************************************************************************/
/** Step Table ****************************************************************/

/*
 *   rf215_op_iqif_start ---- unknown chip mode ----> ERROR
 *            |
 *            |  write IQIFC0..1
 *            v
 *     iqif_read_back
 *            |
 *            |  read IQIFC0..2
 *            v
 *    iqif_have_status ----> IQIF_DONE
 */

static const struct rf215_step_name steps[] = {
    {
        RF215_STEP(rf215_op_iqif_start),
    },
    {
        RF215_STEP(iqif_read_back),
    },
    {
        RF215_STEP(iqif_have_status),
    },
};

const struct rf215_step_name* rf215_op_iqif_steps(size_t* count) {
    *count = sizeof(steps) / sizeof(steps[0]);

    return steps;
}
