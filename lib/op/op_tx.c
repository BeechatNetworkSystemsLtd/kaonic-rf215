/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Transmit, with clear channel assessment and listen-before-talk.
 *
 * The chip assesses the channel itself: with BBCn_AMCS.CCATX armed and the
 * transceiver in receive, triggering a single energy measurement makes it
 * switch to transmit automatically if the channel reads idle, or stay in
 * receive and report TXFE if it does not (section 6.15.5). That keeps the
 * decision and the transmit atomic, which software cannot do across an SPI
 * bus.
 *
 * A busy channel is retried a bounded number of times with a growing wait,
 * which is the listen-before-talk behaviour the 863-870 MHz band requires.
 * When the retries run out the frame is dropped and reported, rather than
 * being sent over whatever is already on the air.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_core.h"
#include "rf215_op.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* Backstop for reaching TXPREP, per attempt at the command. */
#define TXPREP_TIMEOUT_US (5000u)

/* Longest a frame may take on air; sized for the slowest O-QPSK chip rate. */
#define TXFE_TIMEOUT_US (2000000u)

/* Longest the assessment may take to answer, host scheduling delays included. */
#define CCA_TIMEOUT_US (20000u)

/* Appended by the chip, but counted in the length written to TXFL. */
#define FCS_LEN (2u)

/* Capacity of a frame buffer, in octets, check sequence included. */
#define RF215_FRAME_MAX (2047u)

/* BBCn_AMCS.CCATX: assess the channel, then transmit if it is idle. */
#define AMCS_CCATX (0x02u)

/* A frame is arriving when the baseband has reported a start without the matching end. */
/*
 * Longest a reception can still be running: about twice a full-length frame at the slowest
 * modulation. Past it the start is stale; without the cap a missed end blocks all transmits.
 */
#define RX_IN_PROGRESS_MAX_US (200000u)

/******************************************************************************/
/** Forward Declarations ******************************************************/

static rf215_micros_t
backoff_for(struct rf215_dev* dev, const struct rf215_trx* trx, uint8_t attempt);
static enum rf215_step_result
tx_begin_attempt(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
tx_pll_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
tx_txprep_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
tx_cca_read_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);

/******************************************************************************/
/** Static Functions **********************************************************/

/******************************************************************************/
/** Shared Tail ***************************************************************/

/* Ends the attempt. With no event staged this was a backoff: wait here, then assess again. */
static enum rf215_step_result
tx_finish(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    (void)out;

    if (op->evt.kind == RF215_EVT_NONE) {
        /* A received frame is waiting and the next one overwrites it: the read-out needs
         * the operation slot, so the transmit is deferred and the host submits it again. */
        if ((trx->baseband.irqs & RF215_BASEBAND_IRQ_RXFE) != 0u) {
            return rf215_emit(op, RF215_EVT_TX_DEFERRED, RF215_OK, NULL);
        }

        return rf215_delay(dev, op, backoff_for(dev, trx, op->attempt), tx_begin_attempt);
    }

    return rf215_emit(op, op->evt.kind, op->evt.status, NULL);
}

/* Marks the receiver confirmed if it reads RX; if not, the engine re-arms it in full. */
static enum rf215_step_result
tx_rearm_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_RX) {
        trx->radio.armed = true;
    }

    return tx_finish(dev, op, out);
}

/* Reads RFn_STATE once (TXPREP to RX is immediate) to confirm the receiver is on air again. */
static enum rf215_step_result
tx_rearm_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, tx_rearm_state);
}

/* Transmitting leaves the transceiver in TXPREP; puts a receiver that was on air back in RX. */
static enum rf215_step_result
tx_rearm_rx(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (!trx->receiving) {
        return tx_finish(dev, op, out);
    }

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_RX, tx_rearm_read);
}

/* Switches the baseband back on if the assessment left it off; the chip does so itself only
 * on an idle channel (section 6.15.5). */
static enum rf215_step_result
tx_baseband_on(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (!op->bb_off) {
        return tx_rearm_rx(dev, op, out);
    }

    op->bb_off = false;

    return rf215_write8(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), trx->baseband.pc_value, tx_rearm_rx);
}

/* Puts energy detection back into automatic mode after a measurement that never completed. */
static enum rf215_step_result
tx_restore_ed(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    op->ed_pending = false;

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDC), RF215_EDC_AUTO, tx_baseband_on);
}

/*
 * Undoes what the assessment left behind. CCATX stays enabled (see tx_cca_arm) unless a
 * measurement is still outstanding: then it is switched off so a late completion cannot fire
 * the staged buffer, and RFn_EDC is put back to automatic (it resets itself only on
 * completion, section 6.2.4).
 */
static enum rf215_step_result
tx_disarm(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (!op->ed_pending) {
        return tx_baseband_on(dev, op, out);
    }

    trx->baseband.amcs = 0u;
    trx->baseband.amcs_valid = true;

    return rf215_write8(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_AMCS), 0u, tx_restore_ed);
}

/* Waits for the frame end and reports the frame sent. */
static enum rf215_step_result
tx_wait_txfe(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (rf215_take_baseband_irq(trx, RF215_BASEBAND_IRQ_TXFE)) {
        op->evt.kind = RF215_EVT_TX_DONE;

        return tx_disarm(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "tx: frame end never reported");
        dev->stats.timeouts++;
        /* The frame almost certainly went out, so the caller must not resend it. */
        op->evt.kind = RF215_EVT_ERROR;
        op->evt.status = RF215_ERR_TIMEOUT;

        return tx_disarm(dev, op, out);
    }

    op->next = tx_wait_txfe;

    return RF215_STEP_WAIT;
}

/******************************************************************************/
/** The plain path, used when assessment is switched off **********************/

/* Plain path: commands TX. */
static enum rf215_step_result
tx_cmd_tx(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    rf215_set_deadline(dev, op, TXFE_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TX, tx_wait_txfe);
}

/* Second half of the PLL kick, then the lock is read again. */
static enum rf215_step_result
tx_kick_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), RF215_PLL_KICK_OFF, tx_pll_read);
}

/* Waits between the two halves of the PLL kick. */
static enum rf215_step_result
tx_kick_wait(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    return rf215_delay(dev, op, RF215_PLL_KICK_US, tx_kick_off);
}

/* First half of the PLL kick: raises bit 0 of RFn_PLL to restart the synthesizer. */
static enum rf215_step_result
tx_kick_on(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), RF215_PLL_KICK_ON, tx_kick_wait);
}

/* Transmits when locked; polls, then kicks, when not. */
static enum rf215_step_result
tx_pll_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_PLL_LS) != 0u) {
        return tx_cmd_tx(dev, op, out);
    }

    op->polls++;

    if (op->polls < RF215_PLL_LOCK_POLLS) {
        return rf215_delay(dev, op, RF215_PLL_LOCK_POLL_US, tx_pll_read);
    }

    op->polls = 0u;
    op->tries++;

    if (op->tries < RF215_PLL_KICKS) {
        return tx_kick_on(dev, op, out);
    }

    RF215_LOGE(dev, "tx: PLL never locked");
    dev->stats.timeouts++;

    return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
}

/* Reads RFn_PLL: the frame must not go out on an unlocked synthesizer. */
static enum rf215_step_result
tx_pll_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PLL), 1u, tx_pll_state);
}

/* Proceeds in TXPREP, re-issues past the deadline, otherwise polls. */
static enum rf215_step_result
tx_txprep_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_TXPREP) {
        return tx_pll_read(dev, op, out);
    }

    if (!rf215_past_deadline(dev, op)) {
        return rf215_delay(dev, op, RF215_STATE_POLL_US, tx_txprep_read);
    }

    op->tries++;

    if (op->tries < RF215_STATE_ATTEMPTS) {
        rf215_set_deadline(dev, op, TXPREP_TIMEOUT_US);

        return rf215_write8(
            op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXPREP, tx_txprep_read);
    }

    RF215_LOGE(dev, "tx: transmitter never became ready");
    dev->stats.timeouts++;

    return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
}

/* Reads RFn_STATE while TXPREP is awaited. */
static enum rf215_step_result
tx_txprep_read(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, tx_txprep_state);
}

/*
 * Commands TXPREP and polls RFn_STATE instead of awaiting TRXRDY, which only marks the PLL
 * settling from TRXOFF and does not come when leaving RX. The lock is checked afterwards
 * (errata #2: TXPREP can be reached with an unlocked PLL).
 */
static enum rf215_step_result
tx_cmd_txprep(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* Off air from here; the tail reads the receiver back in RX. */
    trx->radio.armed = false;
    op->tries = 0u;
    op->polls = 0u;
    rf215_set_deadline(dev, op, TXPREP_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXPREP, tx_txprep_read);
}

/* Whether a frame is being received right now, with the start latch aged out. */
static bool rx_in_progress(struct rf215_dev* dev, const struct rf215_trx* trx) {
    /* The chip has reported the reception over: frame end, or AGC release. */
    if (!trx->baseband.rx_active) {
        return false;
    }

    if ((trx->baseband.irqs & RF215_BASEBAND_IRQ_RXFS) == 0u) {
        return false;
    }

    if ((trx->baseband.irqs & RF215_BASEBAND_IRQ_RXFE) != 0u) {
        return false;
    }

    const rf215_time_t now = dev->port.now(dev->port.ctx);

    return (rf215_time_t)(now - trx->baseband.rx_start_time) < RX_IN_PROGRESS_MAX_US;
}

/* Each attempt waits a little longer than the last. */
/* Jitter source: two nodes that collide must not back off by the same amount. */
static uint32_t backoff_rand(struct rf215_dev* dev) {
    /* xorshift32: no libc, no division, and good enough to break a tie. */
    uint32_t x = (dev->backoff_rand != 0u) ? dev->backoff_rand : 0x2545f491u;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    dev->backoff_rand = x;

    return x;
}

/*
 * Backoff for the given attempt: growing, capped and jittered. The retries together must
 * outlast one full-length frame from a neighbour, while each wait stays short enough not to
 * sleep through the gap.
 */
static rf215_micros_t
backoff_for(struct rf215_dev* dev, const struct rf215_trx* trx, uint8_t attempt) {
    const uint32_t base =
        (trx->lbt_backoff_us != 0u) ? trx->lbt_backoff_us : RF215_CCA_DEFAULT_BACKOFF_US;
    uint32_t window = base * ((uint32_t)attempt + 1u);

    if (window > RF215_CCA_BACKOFF_US_MAX) {
        window = RF215_CCA_BACKOFF_US_MAX;
    }

    /* Uniform over the upper half of the window: never too early, never past the gap. */
    const uint32_t half = window / 2u;

    return (rf215_micros_t)(half + (backoff_rand(dev) % (half + 1u)));
}

/* Counts the busy channel and either retries after a backoff or drops the frame. */
static enum rf215_step_result
tx_cca_busy(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    dev->stats.cca_busy++;

    if (op->attempt < trx->lbt_retries) {
        /* Disarm first so the backoff is not spent with the baseband off; the wait
         * itself is in tx_finish. */
        op->attempt++;
        op->evt.kind = RF215_EVT_NONE;

        return tx_disarm(dev, op, out);
    }

    dev->stats.tx_dropped++;
    op->evt.kind = RF215_EVT_TX_BUSY;

    return tx_disarm(dev, op, out);
}

/*
 * Decides the outcome from the interrupts alone (section 6.15.5, Figures 6-35 and 6-36):
 * TRXRDY means idle and the chip transmits, TXFE without it means busy. TRXRDY is checked
 * first because a short frame can put both in one status read; EDC alone decides nothing.
 */
static enum rf215_step_result
tx_cca_wait(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (rf215_take_radio_irq(trx, RF215_RADIO_IRQ_TRXRDY)) {
        /* Idle: the chip has the frame and has switched the baseband back on. */
        op->ed_pending = false;
        op->bb_off = false;
        rf215_set_deadline(dev, op, TXFE_TIMEOUT_US);
        op->next = tx_wait_txfe;

        return RF215_STEP_WAIT;
    }

    if (rf215_take_baseband_irq(trx, RF215_BASEBAND_IRQ_TXFE)) {
        /* Busy: the measurement is over, nothing was sent. */
        op->ed_pending = false;

        return tx_cca_busy(dev, op, out);
    }

    /* Synthesizer lost lock (TRXERR, section 6.3): retried like a busy channel. */
    if (rf215_take_radio_irq(trx, RF215_RADIO_IRQ_TRXERR)) {
        return tx_cca_busy(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "tx: channel assessment never answered");
        dev->stats.timeouts++;
        op->evt.kind = RF215_EVT_ERROR;
        op->evt.status = RF215_ERR_TIMEOUT;

        return tx_disarm(dev, op, out);
    }

    op->next = tx_cca_wait;

    return RF215_STEP_WAIT;
}

/* Clears stale interrupts and starts the energy measurement. */
static enum rf215_step_result
tx_cca_trigger(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    /* A stale TRXRDY or TRXERR would answer the wait at once. TXFE is cleared too: a busy
     * assessment raises it without transmitting, and left latched it would cut the next
     * frame off. Receiver interrupts are left alone for the read-out. */
    trx->radio.irqs &=
        (uint8_t)~(RF215_RADIO_IRQ_TRXRDY | RF215_RADIO_IRQ_TRXERR | RF215_RADIO_IRQ_EDC);
    trx->baseband.irqs &= (uint8_t)~RF215_BASEBAND_IRQ_TXFE;

    op->ed_pending = true;
    rf215_set_deadline(dev, op, CCA_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDC), RF215_EDC_SINGLE, tx_cca_wait);
}

/* Switches the baseband off so it cannot start decoding during the measurement, as the
 * datasheet recommends. The chip turns it back on when idle; otherwise the tail does. */
static enum rf215_step_result
tx_cca_baseband_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t pc = (uint8_t)(trx->baseband.pc_value & (uint8_t)~RF215_PC_BBEN);

    op->bb_off = true;

    return rf215_write8(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), pc, tx_cca_trigger);
}

/* Proceeds in RX, keeps polling in transition, gives up past the deadline. */
static enum rf215_step_result
tx_cca_check_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_RX) {
        return tx_cca_baseband_off(dev, op, out);
    }

    if (rf215_past_deadline(dev, op)) {
        RF215_LOGE(dev, "tx: receiver never settled for the assessment");
        dev->stats.timeouts++;
        op->evt.kind = RF215_EVT_ERROR;
        op->evt.status = RF215_ERR_TIMEOUT;

        return tx_disarm(dev, op, out);
    }

    /* A transition takes on the order of a hundred microseconds. */
    return rf215_delay(dev, op, 50u, tx_cca_read_state);
}

/* Reads RFn_STATE: an energy detect triggered while still in transition is ignored. */
static enum rf215_step_result
tx_cca_read_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, tx_cca_check_state);
}

/*
 * Commands RX for the measurement. The write is unconditional on purpose (section 7.2.4):
 * gating it on RFn_STATE already reading RX stopped all transmits. The state is polled only
 * when the receiver was not confirmed on air: from TRXOFF it takes up to 200 us (Table 10-7).
 */
static enum rf215_step_result
tx_cca_listen(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];
    const bool on_air = trx->radio.armed;

    /* An idle assessment leaves the transceiver in TXPREP; the tail confirms RX again. */
    trx->radio.armed = false;

    rf215_set_deadline(dev, op, CCA_TIMEOUT_US);

    return rf215_write8(op,
                        out,
                        rf215_radio_reg(trx, RF215_RG_RFXX_CMD),
                        RF215_CMD_RF_RX,
                        on_air ? tx_cca_baseband_off : tx_cca_read_state);
}

/*
 * Enables CCATX once and leaves it on between attempts and frames: the procedure only starts
 * on a single energy measurement (note to section 6.15.5) and RFn_CMD stays writable until
 * then (section 7.2.4). The energy detect operation must switch it off first.
 */
static enum rf215_step_result
tx_cca_arm(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (trx->baseband.amcs_valid && (trx->baseband.amcs == AMCS_CCATX)) {
        return tx_cca_listen(dev, op, out);
    }

    /* Recorded before the write completes, as the threshold is. */
    trx->baseband.amcs = AMCS_CCATX;
    trx->baseband.amcs_valid = true;

    return rf215_write8(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_AMCS), AMCS_CCATX, tx_cca_listen);
}

/* Starts one attempt: backs off while a frame is arriving, else arms CCA or commands TXPREP. */
static enum rf215_step_result
tx_begin_attempt(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if (!trx->cca_enabled) {
        return tx_cmd_txprep(dev, op, out);
    }

    /*
     * Going ahead would cut off the arriving frame. Nothing is armed yet, so this backs off
     * directly rather than through the recovery tail.
     */
    if (rx_in_progress(dev, trx)) {
        if (op->attempt < trx->lbt_retries) {
            op->attempt++;

            return rf215_delay(dev, op, backoff_for(dev, trx, op->attempt), tx_begin_attempt);
        }

        dev->stats.tx_dropped++;

        return rf215_emit(op, RF215_EVT_TX_BUSY, RF215_OK, NULL);
    }

    return tx_cca_arm(dev, op, out);
}

/* Writes the threshold. It is recorded before the write completes; op_fail() forgets it
 * again if the write fails. */
static enum rf215_step_result
tx_cca_threshold(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    trx->baseband.amedt = (uint8_t)trx->cca_threshold;
    trx->baseband.amedt_valid = true;

    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(trx, RF215_RG_BBCX_AMEDT),
                        trx->baseband.amedt,
                        tx_begin_attempt);
}

/*
 * Loads the frame before any assessment starts: with CCATX the chip transmits the buffer the
 * moment the channel reads idle. The check sequence bytes are not written; the baseband
 * computes them.
 */
static enum rf215_step_result
tx_load_frame(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    /* The threshold only changes through rf215_set_cca(), so it is written only when stale. */
    const bool threshold_stale =
        !trx->baseband.amedt_valid || (trx->baseband.amedt != (uint8_t)trx->cca_threshold);

    op->next = (trx->cca_enabled && threshold_stale) ? tx_cca_threshold : tx_begin_attempt;

    out->op = RF215_XFER_WRITE;
    out->reg = rf215_frame_buffer_reg(trx, RF215_RG_BBCX_FBTXS);
    /* The buffer belongs to the caller for the life of the operation. */
    out->buf = (void*)(uintptr_t)op->tx_data;
    out->len = op->len;

    return RF215_STEP_XFER;
}

/******************************************************************************/
/** Public Functions **********************************************************/

/* Writes the frame length; TXFLL and TXFLH are contiguous and little endian. */
enum rf215_step_result
rf215_op_tx_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];

    if ((op->tx_data == NULL) || (op->len == 0u)) {
        RF215_LOGE(dev, "tx: empty frame");
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_INVAL, NULL);
    }

    /* The check sequence is counted only when the baseband is configured to append it. */
    const uint16_t total = (uint16_t)(op->len + (trx->baseband.fcs ? FCS_LEN : 0u));

    /* The other transceiver's receive buffer follows this one directly; a longer burst
     * would overwrite it. */
    if (total > RF215_FRAME_MAX) {
        RF215_LOGE(dev, "tx: frame longer than the transmit buffer");
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_INVAL, NULL);
    }

    return rf215_write16(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_TXFLL), total, tx_load_frame);
}

/******************************************************************************/
/** Step Table ****************************************************************/

/*
 *   rf215_op_tx_start ---- empty or too long ----> ERROR
 *            |
 *            |  write TXFL
 *            v
 *      tx_load_frame              write the frame buffer
 *            |
 *            v
 *    (tx_cca_threshold)           write AMEDT, only when it changed
 *            |
 *            v
 *     tx_begin_attempt <---------------------------------- backoff, from tx_finish
 *        |        |
 *        |        +---- a frame is arriving: back off and come back,
 *        |              or TX_BUSY when out of retries
 *        |
 *        +---- assessment on ----> the channel assessment path
 *        +---- assessment off ---> the plain path
 *
 * Channel assessment path:
 *
 *       tx_cca_arm                write AMCS=CCATX, once
 *            |
 *            v
 *      tx_cca_listen              write CMD=RX
 *            |        |
 *            |        +-- receiver not confirmed --> tx_cca_read_state <-> tx_cca_check_state
 *            |                                              |  (50 us poll)        |
 *            v                                              |  RX                  |  20 ms gone
 *    tx_cca_baseband_off <----------------------------------+                      v
 *            |                                                               tail, ERROR
 *            v
 *      tx_cca_trigger             write EDC=single
 *            |
 *            v
 *       tx_cca_wait               waits for the interrupt
 *            |
 *            +-- TRXRDY: idle, the chip transmits ---------> tx_wait_txfe
 *            +-- TXFE or TRXERR: busy ---> tx_cca_busy ----> tail, retry or TX_BUSY
 *            +-- 20 ms without an answer ------------------> tail, ERROR
 *
 * Plain path:
 *
 *      tx_cmd_txprep              write CMD=TXPREP
 *            |
 *            v
 *     tx_txprep_read <-> tx_txprep_state      100 us poll; re-issued after 5 ms, ERROR after 3
 *            |
 *            |  TXPREP
 *            v
 *       tx_pll_read <-> tx_pll_state          20 us poll; after 10 polls unlocked:
 *            |                                tx_kick_on -> tx_kick_wait -> tx_kick_off,
 *            |  locked                        ERROR after 4 kicks
 *            v
 *        tx_cmd_tx ----> tx_wait_txfe
 *
 * Shared tail:
 *
 *      tx_wait_txfe ---- TXFE: TX_DONE staged ---- 2 s gone: ERROR staged
 *            |
 *            v
 *        tx_disarm -> (tx_restore_ed) -> tx_baseband_on -> tx_rearm_rx -> tx_rearm_read
 *                                                                              |
 *            +-----------------------------------------------------------------+
 *            v
 *      tx_rearm_state
 *            |
 *            v
 *        tx_finish
 *            |
 *            +-- an event is staged ----------------------> TX_DONE, TX_BUSY or ERROR
 *            +-- a received frame waits to be read out ---> TX_DEFERRED
 *            +-- neither: back off ------------------------> tx_begin_attempt
 */

static const struct rf215_step_name steps[] = {
    {
        RF215_STEP(rf215_op_tx_start),
    },
    {
        RF215_STEP(tx_load_frame),
    },
    {
        RF215_STEP(tx_begin_attempt),
    },
    {
        RF215_STEP(tx_cca_threshold),
    },
    {
        RF215_STEP(tx_cca_arm),
    },
    {
        RF215_STEP(tx_cca_listen),
    },
    {
        RF215_STEP(tx_cca_read_state),
    },
    {
        RF215_STEP(tx_cca_check_state),
    },
    {
        RF215_STEP(tx_cca_baseband_off),
    },
    {
        RF215_STEP(tx_cca_trigger),
    },
    {
        RF215_STEP(tx_cca_wait),
    },
    {
        RF215_STEP(tx_cca_busy),
    },
    {
        RF215_STEP(tx_cmd_txprep),
    },
    {
        RF215_STEP(tx_txprep_read),
    },
    {
        RF215_STEP(tx_txprep_state),
    },
    {
        RF215_STEP(tx_pll_read),
    },
    {
        RF215_STEP(tx_pll_state),
    },
    {
        RF215_STEP(tx_kick_on),
    },
    {
        RF215_STEP(tx_kick_wait),
    },
    {
        RF215_STEP(tx_kick_off),
    },
    {
        RF215_STEP(tx_cmd_tx),
    },
    {
        RF215_STEP(tx_wait_txfe),
    },
    {
        RF215_STEP(tx_disarm),
    },
    {
        RF215_STEP(tx_restore_ed),
    },
    {
        RF215_STEP(tx_baseband_on),
    },
    {
        RF215_STEP(tx_rearm_rx),
    },
    {
        RF215_STEP(tx_rearm_read),
    },
    {
        RF215_STEP(tx_rearm_state),
    },
    {
        RF215_STEP(tx_finish),
    },
};

const struct rf215_step_name* rf215_op_tx_steps(size_t* count) {
    *count = sizeof(steps) / sizeof(steps[0]);

    return steps;
}
