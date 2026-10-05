/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Receive and energy detect.
 *
 * Arming the receiver is a submitted operation; reading a frame out of it is
 * not. A frame arrives when it arrives, so the engine starts the read-out
 * itself when the baseband reports a frame end and nothing else is running.
 *
 * The receiver is armed the way the userspace driver armed it on the same
 * boards: through TXPREP, with the PLL read back as locked before RX is
 * commanded, and with every commanded state read back rather than assumed.
 *
 * A frame is read out with the receiver already back on air, so the gap per
 * frame is a few short transfers rather than the whole read-out. The price is
 * that the next frame can start writing over the one being read; that is
 * noticed by its frame start and the frame read is dropped rather than
 * delivered corrupt.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_core.h"
#include "rf215_op.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* A single energy measurement takes one ED duration, 960 us as configured. */
#define ED_TIMEOUT_US (20000u)

/*
 * Settle between a confirmed PLL lock and the RX command, in us. Kept under
 * 200 us so a port that waits short delays inline (Linux) keeps the re-arm in
 * the interrupt thread.
 */
#define ARM_SETTLE_US (150u)

/******************************************************************************/
/** Forward Declarations ******************************************************/

static enum rf215_step_result
ed_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
arm_read_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
arm_rx_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
arm_pll_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
arm_txprep_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);

/******************************************************************************/
/** Static Functions **********************************************************/

static enum rf215_step_result
rx_armed(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    return rf215_emit(op, RF215_EVT_RX_ARMED, RF215_OK, NULL);
}

/* Ends an engine-started re-arm. Emits no event: no host wait belongs to it. */
static enum rf215_step_result
rx_rearmed(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    op->next = NULL;

    return RF215_STEP_IDLE;
}

/* Reports the transceiver as off air. */
static enum rf215_step_result
rx_stopped(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    return rf215_emit(op, RF215_EVT_RX_STOPPED, RF215_OK, NULL);
}

/******************************************************************************/
/** Read-Out ******************************************************************/

/* Hands the frame up, or ends quietly when there is nothing to report. */
static enum rf215_step_result
rx_report(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    /* Nothing to report: the frame was dropped or never read. */
    if (op->len == 0u) {
        op->next = NULL;
        return RF215_STEP_IDLE;
    }

    /* rf215_emit leaves len and rssi alone, so both survive the staging. */
    op->evt.len = op->len;

    return rf215_emit(op, RF215_EVT_RX_FRAME, RF215_OK, NULL);
}

/* Records the receiver as on air when a blind command reached RX. */
static enum rf215_step_result
rx_confirm_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_RX) {
        trx->radio.armed = true;
    }

    return rx_report(dev, op, out);
}

/* Drops a frame overwritten during the read-out; confirms the receiver on air. */
static enum rf215_step_result
rx_rearm(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /*
     * RXFS since this frame's end: the next frame began overwriting the
     * buffer. The FCS was checked before that and does not cover it.
     */
    if ((trx->baseband.irqs & RF215_BASEBAND_IRQ_RXFS) != 0u) {
        dev->stats.rx_overruns++;
        op->evt.kind = RF215_EVT_NONE;
        op->len = 0u;

        /* No RXFE: that frame was cut off and no end will follow. Clear RXFS,
         * or it refuses every transmit until it ages out. */
        if ((trx->baseband.irqs & RF215_BASEBAND_IRQ_RXFE) == 0u) {
            trx->baseband.irqs &= (uint8_t)~RF215_BASEBAND_IRQ_RXFS;
        }
    }

    /* A blind RX command issued before the read-out is confirmed with one
     * state read, which saves the engine a full arming sequence. */
    if (trx->receiving && !trx->radio.armed) {
        return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, rx_confirm_state);
    }

    return rx_report(dev, op, out);
}

/* Reads the frame out of the receive buffer. */
static enum rf215_step_result
rx_read_frame(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    op->next = rx_rearm;
    out->op = RF215_XFER_READ;
    out->reg = rf215_frame_buffer_reg(trx, RF215_RG_BBCX_FBRXS);
    out->buf = trx->baseband.rx_buf;
    out->len = op->len;

    return RF215_STEP_XFER;
}

/* Commands RX only if the completed reception left the transceiver off it. */
static enum rf215_step_result
rx_have_edv(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* EDV is the signal strength of the frame just received, in dBm. 127 means
     * no valid measurement (section 6.2.5.8). */
    op->evt.rssi = (op->buf[0] == 0x7Fu) ? RF215_RSSI_INVALID : (int8_t)op->buf[0];

    if (op->state == RF215_STATE_RX) {
        trx->radio.armed = true;

        return rx_read_frame(dev, op, out);
    }

    dev->stats.rx_rearms++;

    /* Written without confirmation; rx_rearm or the engine confirms it. */
    trx->radio.armed = false;

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_RX, rx_read_frame);
}

/*
 * Reads RFn_EDV before the receiver is re-armed: the next frame's measurement
 * replaces it. A separate one-register read on purpose: one block from
 * RFn_STATE to RFn_EDV does not fit a host SPI FIFO.
 */
static enum rf215_step_result
rx_len_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    /* The transfer buffer is reused by the read below. */
    op->state = op->buf[0] & RF215_STATE_MASK;

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDV), 1u, rx_have_edv);
}

/* Checks the frame length and re-arms the receiver before the read-out. */
static enum rf215_step_result
rx_have_len(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint16_t len = (uint16_t)(op->buf[0] | ((uint16_t)op->buf[1] << 8));

    /* Off air since the frame end: the length and the buffer are stale (after
     * a band switch, the previous band's frame). */
    if (!trx->receiving) {
        dev->stats.rx_stale++;
        op->len = 0u;

        return rx_report(dev, op, out);
    }

    /* Zero means no frame; a length past the buffer is not a valid one. */
    if ((len == 0u) || (len > trx->baseband.rx_cap)) {
        RF215_LOGE(dev, "rx: frame does not fit the receive buffer");
        dev->stats.rx_stale++;
        op->len = 0u;

        return rx_report(dev, op, out);
    }

    op->len = len;

    /*
     * Re-arm before the read-out to keep the off-air gap short; rx_rearm
     * catches a frame that starts meanwhile. RX is commanded only if the
     * state has left RX: RX written into a live receiver restarts it.
     */
    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, rx_len_state);
}

/******************************************************************************/
/** Energy Detect *************************************************************/

/* Reports the measurement to the host. */
static enum rf215_step_result
ed_restore(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    return rf215_emit(op, RF215_EVT_ED_DONE, RF215_OK, NULL);
}

/* A transceiver brought on air for the measurement goes back off it. */
static enum rf215_step_result
ed_baseband_on(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (op->attempt == 0u) {
        return ed_restore(dev, op, out);
    }

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXOFF, ed_restore);
}

/* Records the value and re-enables the baseband. */
static enum rf215_step_result
ed_have_value(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    op->evt.rssi = (op->buf[0] == 0x7Fu) ? RF215_RSSI_INVALID : (int8_t)op->buf[0];

    /* The single measurement mode resets itself to automatic when it
     * completes (section 6.2.4), so only the baseband needs putting back. */
    return rf215_write8(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), trx->baseband.pc_value, ed_baseband_on);
}

/* Reads the measured energy from RFn_EDV. */
static enum rf215_step_result
ed_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDV), 1u, ed_have_value);
}

/* Waits for the measurement to complete (EDC). */
static enum rf215_step_result
ed_wait(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (rf215_take_radio_irq(trx, RF215_RADIO_IRQ_EDC)) {
        return ed_read(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "ed: measurement never completed");
        dev->stats.timeouts++;
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
    }

    op->next = ed_wait;

    return RF215_STEP_WAIT;
}

/* Starts one energy measurement and waits for EDC. */
static enum rf215_step_result
ed_trigger(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    trx->radio.irqs &= (uint8_t)~RF215_RADIO_IRQ_EDC;
    rf215_set_deadline(dev, op, ED_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDC), RF215_EDC_SINGLE, ed_wait);
}

/* Disables the baseband, as the datasheet recommends, so a frame arriving
 * during the measurement cannot overwrite the value. */
static enum rf215_step_result
ed_baseband_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t pc = (uint8_t)(trx->baseband.pc_value & (uint8_t)~RF215_PC_BBEN);

    return rf215_write8(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), pc, ed_trigger);
}

/* Re-reads RFn_STATE while the transition to RX completes. */
static enum rf215_step_result
ed_poll(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, ed_state);
}

/* Enters RX for the measurement if the transceiver is not there already. */
static enum rf215_step_result
ed_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_RX) {
        return ed_baseband_off(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "ed: receiver never settled for the measurement");
        dev->stats.timeouts++;
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
    }

    if (op->attempt == 0u) {
        op->attempt = 1u;
        return rf215_write8(
            op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_RX, ed_poll);
    }

    return rf215_delay(dev, op, 50u, ed_poll);
}

/******************************************************************************/
/** Arming the Receiver *******************************************************/

/*
 * Sequence: read the state, go to TXPREP unless already there or in RX,
 * confirm PLL lock (kick when unlocked), command RX, read the state back.
 * Retries are bounded by RF215_PLL_KICKS. A receiver in RX with a locked PLL
 * is left alone: commanding RX into a live receiver restarts it mid-frame.
 */

/*
 * Marks the receiver on air: RFn_STATE was read back as RX with the PLL
 * locked. Every path that leaves RX must clear radio.armed.
 */
static enum rf215_step_result
arm_resume(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /*
     * A commanded transition lost any frame still latched. A sequence that
     * found RX already commanded nothing, so its latches stay and a frame end
     * among them still drives the read-out.
     */
    if (op->arm_moved) {
        rf215_rx_off_air(trx);
    }

    trx->radio.armed = true;

    return op->resume(dev, op, out);
}

/* Arming: second half of the PLL kick, then back to the state dispatch. */
static enum rf215_step_result
arm_kick_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), RF215_PLL_KICK_OFF, arm_read_state);
}

/* Arming: delay between the two halves of the PLL kick. */
static enum rf215_step_result
arm_kick_wait(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    return rf215_delay(dev, op, RF215_PLL_KICK_US, arm_kick_off);
}

/* Arming: first half of the PLL kick. Bit 0 of RFn_PLL is raised briefly to
 * restart the synthesizer, then the reset value is written back. */
static enum rf215_step_result
arm_kick_on(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), RF215_PLL_KICK_ON, arm_kick_wait);
}

/* Kicks the PLL and restarts the sequence; fails after RF215_PLL_KICKS tries. */
static enum rf215_step_result
arm_retry(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out, const char* what) {
    op->tries++;

    if (op->tries >= RF215_PLL_KICKS) {
        RF215_LOGE(dev, what);
        dev->stats.timeouts++;
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
    }

    return arm_kick_on(dev, op, out);
}

/* Arming: resumes the caller in RX, re-issues past the deadline, otherwise polls. */
static enum rf215_step_result
arm_rx_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_RX) {
        return arm_resume(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        return arm_retry(dev, op, out, "rx: transceiver never reached RX");
    }

    return rf215_delay(dev, op, RF215_STATE_POLL_US, arm_rx_read);
}

/* Arming: reads RFn_STATE while RX is awaited. */
static enum rf215_step_result
arm_rx_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, arm_rx_state);
}

/* Arming: commands RX and starts the clock on it. */
static enum rf215_step_result
arm_cmd_rx(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    op->arm_moved = true;

    rf215_set_deadline(dev, op, RF215_STATE_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_RX, arm_rx_read);
}

/* Arming: proceeds when locked; polls, then kicks, when not. */
static enum rf215_step_result
arm_pll_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_PLL_LS) != 0u) {
        op->polls = 0u;
        return rf215_delay(dev, op, ARM_SETTLE_US, arm_cmd_rx);
    }

    op->polls++;

    if (op->polls < RF215_PLL_LOCK_POLLS) {
        return rf215_delay(dev, op, RF215_PLL_LOCK_POLL_US, arm_pll_read);
    }

    op->polls = 0u;

    return arm_retry(dev, op, out, "rx: PLL never locked");
}

/* Arming: reads RFn_PLL. */
static enum rf215_step_result
arm_pll_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), 1u, arm_pll_state);
}

/* Arming: proceeds in TXPREP, re-issues past the deadline, otherwise polls. */
static enum rf215_step_result
arm_txprep_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_TXPREP) {
        return arm_pll_read(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        return arm_retry(dev, op, out, "rx: transceiver never reached TXPREP");
    }

    return rf215_delay(dev, op, RF215_STATE_POLL_US, arm_txprep_read);
}

/* Arming: reads RFn_STATE while TXPREP is awaited. */
static enum rf215_step_result
arm_txprep_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, arm_txprep_state);
}

/* Arming: commands TXPREP and starts the clock on it. */
static enum rf215_step_result
arm_cmd_txprep(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    op->arm_moved = true;

    rf215_set_deadline(dev, op, RF215_STATE_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXPREP, arm_txprep_read);
}

/* Arming: already in RX. An unlocked PLL is relocked through TXPREP (errata #2). */
static enum rf215_step_result
arm_rx_pll_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_PLL_LS) != 0u) {
        return arm_resume(dev, op, out);
    }

    return arm_cmd_txprep(dev, op, out);
}

/* Arming: dispatches on the state read. */
static enum rf215_step_result
arm_have_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t state = op->buf[0] & RF215_STATE_MASK;

    op->polls = 0u;

    if (state == RF215_STATE_RX) {
        return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), 1u, arm_rx_pll_state);
    }

    if (state == RF215_STATE_TXPREP) {
        return arm_pll_read(dev, op, out);
    }

    return arm_cmd_txprep(dev, op, out);
}

/* Arming: reads RFn_STATE to decide where to start from. */
static enum rf215_step_result
arm_read_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, arm_have_state);
}

/*
 * Arms from an unknown chip state. BBCn_PC is rewritten first: a transmit
 * abandoned during CCA leaves the baseband off and the arming sequence does
 * not re-enable it. pc_value == 0 means not configured, nothing to restore.
 */
static enum rf215_step_result arm_from_unknown(struct rf215_dev* dev,
                                               struct rf215_op* op,
                                               struct rf215_xfer* out,
                                               rf215_step_fn resume) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (trx->baseband.pc_value == 0u) {
        return rf215_op_rx_arm(dev, op, out, resume);
    }

    op->resume = resume;
    op->tries = 0u;
    op->polls = 0u;
    op->arm_moved = false;

    return rf215_write8(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), trx->baseband.pc_value, arm_read_state);
}

/******************************************************************************/
/** Public Functions **********************************************************/

enum rf215_step_result
rf215_op_rx_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* Latches left by the last session belong to a frame the chip no longer
     * holds. */
    rf215_rx_off_air(trx);

    trx->receiving = true;

    return arm_from_unknown(dev, op, out, rx_armed);
}

enum rf215_step_result
rf215_op_rx_stop(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* Latches are cleared before the command: otherwise the engine starts a
     * read-out that hands up the buffer of the band just left. */
    rf215_rx_off_air(trx);

    trx->receiving = false;

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXOFF, rx_stopped);
}

/* RXFLL and RXFLH are contiguous and little endian. */
enum rf215_step_result
rf215_op_rx_read_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_RXFLL), 2u, rx_have_len);
}

enum rf215_step_result
rf215_op_rx_rearm_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    return arm_from_unknown(dev, op, out, rx_rearmed);
}

enum rf215_step_result
rf215_op_ed_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* The measurement only runs in RX (section 6.2.4). op->attempt records
     * whether RX was entered for it and must be left after. armed is cleared
     * so the engine re-confirms the receiver afterwards. */
    trx->radio.armed = false;
    op->attempt = 0u;
    rf215_set_deadline(dev, op, ED_TIMEOUT_US);

    /* The transmit path leaves CCATX enabled, and a single measurement
     * triggers it (section 6.15.5): AMCS is cleared first, or an idle channel
     * sends whatever the transmit buffer holds. */
    if (!trx->baseband.amcs_valid || (trx->baseband.amcs != 0u)) {
        trx->baseband.amcs = 0u;
        trx->baseband.amcs_valid = true;

        return rf215_write8(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_AMCS), 0u, ed_poll);
    }

    return ed_poll(dev, op, out);
}

enum rf215_step_result rf215_op_rx_arm(struct rf215_dev* dev,
                                       struct rf215_op* op,
                                       struct rf215_xfer* out,
                                       rf215_step_fn resume) {
    op->resume = resume;
    op->tries = 0u;
    op->polls = 0u;
    op->arm_moved = false;

    return arm_read_state(dev, op, out);
}

/******************************************************************************/
/** Step Table ****************************************************************/

/*
 * Start, stop and the engine's own re-arm:
 *
 *   rf215_op_rx_start -------> arming ----> rx_armed -----> RX_ARMED
 *   rf215_op_rx_rearm_start -> arming ----> rx_rearmed      (nothing reported)
 *   rf215_op_rx_stop --------- write CMD=TRXOFF ----> rx_stopped ----> RX_STOPPED
 *
 * Read-out, started by the engine on a frame end:
 *
 *   rf215_op_rx_read_start        read RXFL
 *            |
 *            v
 *       rx_have_len ---- off air, or length 0 or too long ----> rx_report (nothing)
 *            |
 *            |  read RFn_STATE
 *            v
 *       rx_len_state
 *            |
 *            |  read RFn_EDV
 *            v
 *       rx_have_edv ---- not in RX: write CMD=RX ----+
 *            |                                       |
 *            v                                       |
 *      rx_read_frame <-------------------------------+
 *            |
 *            |  read the frame
 *            v
 *        rx_rearm ---- re-armed blind ----> rx_confirm_state ----+
 *            |                                                   |
 *            v                                                   |
 *        rx_report <---------------------------------------------+
 *            |
 *            v
 *        RX_FRAME                 (nothing if a new frame start overwrote the buffer)
 *
 * Energy detect:
 *
 *   rf215_op_ed_start             write AMCS=0 if assessment was armed
 *            |
 *            v
 *        ed_poll <-------------------+  read RFn_STATE
 *            |                       |
 *            v                       |  not RX: write CMD=RX once, then 50 us
 *        ed_state -------------------+----- 20 ms gone ----> ERROR
 *            |
 *            |  RX
 *            v
 *     ed_baseband_off -> ed_trigger -> ed_wait ---- 20 ms without EDC ----> ERROR
 *                                         |
 *                                         |  EDC interrupt
 *                                         v
 *                  ed_read -> ed_have_value -> ed_baseband_on -> ed_restore ----> ED_DONE
 *
 * Arming (rf215_op_rx_arm), shared with configure:
 *
 *     arm_read_state <-------------------------------------------- arm_kick_off
 *            |                                                          ^
 *            v                                                          |
 *     arm_have_state                                               arm_kick_wait
 *        |    |    |                                                    ^
 *        |    |    +-- RX ------> arm_rx_pll_state -- locked --+        |
 *        |    |                          |                     |   arm_kick_on
 *        |    |                          |  not locked         |        ^
 *        |    |                          v                     |        |
 *        |    +------- other ---> arm_cmd_txprep               |    arm_retry ----> ERROR
 *        |                               |                     |        ^     (after 4 kicks)
 *        |                               v                     |        |
 *        |              arm_txprep_read <-> arm_txprep_state --|--------+  20 ms gone
 *        |                               |  (100 us poll)      |        |
 *        |  TXPREP                       v  TXPREP             |        |
 *        +----------------------> arm_pll_read <-> arm_pll_state -------+  10 polls unlocked
 *                                        |  (20 us poll)       |        |
 *                                        v  locked, 150 us     |        |
 *                                   arm_cmd_rx                 |        |
 *                                        |                     |        |
 *                                        v                     |        |
 *                           arm_rx_read <-> arm_rx_state ------|--------+  20 ms gone
 *                                        |  (100 us poll)      |
 *                                        v  RX                 |
 *                                   arm_resume <---------------+
 *                                        |
 *                                        v
 *                               the caller's next step
 */

static const struct rf215_step_name steps[] = {
    {
        RF215_STEP(rf215_op_rx_start),
    },
    {
        RF215_STEP(rx_armed),
    },
    {
        RF215_STEP(rf215_op_rx_rearm_start),
    },
    {
        RF215_STEP(rx_rearmed),
    },
    {
        RF215_STEP(rf215_op_rx_stop),
    },
    {
        RF215_STEP(rx_stopped),
    },
    {
        RF215_STEP(rf215_op_rx_read_start),
    },
    {
        RF215_STEP(rx_have_len),
    },
    {
        RF215_STEP(rx_len_state),
    },
    {
        RF215_STEP(rx_have_edv),
    },
    {
        RF215_STEP(rx_read_frame),
    },
    {
        RF215_STEP(rx_rearm),
    },
    {
        RF215_STEP(rx_confirm_state),
    },
    {
        RF215_STEP(rx_report),
    },
    {
        RF215_STEP(rf215_op_ed_start),
    },
    {
        RF215_STEP(ed_state),
    },
    {
        RF215_STEP(ed_poll),
    },
    {
        RF215_STEP(ed_baseband_off),
    },
    {
        RF215_STEP(ed_trigger),
    },
    {
        RF215_STEP(ed_wait),
    },
    {
        RF215_STEP(ed_read),
    },
    {
        RF215_STEP(ed_have_value),
    },
    {
        RF215_STEP(ed_baseband_on),
    },
    {
        RF215_STEP(ed_restore),
    },
    {
        RF215_STEP(arm_read_state),
    },
    {
        RF215_STEP(arm_have_state),
    },
    {
        RF215_STEP(arm_rx_pll_state),
    },
    {
        RF215_STEP(arm_cmd_txprep),
    },
    {
        RF215_STEP(arm_txprep_read),
    },
    {
        RF215_STEP(arm_txprep_state),
    },
    {
        RF215_STEP(arm_pll_read),
    },
    {
        RF215_STEP(arm_pll_state),
    },
    {
        RF215_STEP(arm_kick_on),
    },
    {
        RF215_STEP(arm_kick_wait),
    },
    {
        RF215_STEP(arm_kick_off),
    },
    {
        RF215_STEP(arm_cmd_rx),
    },
    {
        RF215_STEP(arm_rx_read),
    },
    {
        RF215_STEP(arm_rx_state),
    },
};

const struct rf215_step_name* rf215_op_rx_steps(size_t* count) {
    *count = sizeof(steps) / sizeof(steps[0]);

    return steps;
}
