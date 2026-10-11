/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_core.h"
#include "rf215_op.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* The four interrupt status registers are contiguous and must all be read
 * before the chip releases its interrupt line. */
#define IRQS_BASE (RF215_RG_RF09_IRQS)

#define IRQS_COUNT (4u)

/* Stages of the recovery sequence, in order. */
#define RECOVERY_STAGE_IDLE (0u)
#define RECOVERY_STAGE_RESET (1u)
#define RECOVERY_STAGE_IQIF (2u)
#define RECOVERY_STAGE_CFG_09 (3u)
#define RECOVERY_STAGE_CFG_24 (4u)

/******************************************************************************/
/** Forward Declarations ******************************************************/

static void init_trx(struct rf215_trx* trx, enum rf215_band band);
/* Starts a request in a free slot; the caller has checked it is free. */
static void start_request(struct rf215_dev* dev, const struct rf215_req* req, bool recovery_stage);
/* Starts the queued request or frame, if the slot is free and one is waiting. */
static bool start_waiting_work(struct rf215_dev* dev, uint8_t index);
/* Abandons an operation and stages RF215_ERR_ABORTED for whoever waits on it. */
static void abort_and_report(struct rf215_dev* dev, uint8_t index);
/* Begins the recovery sequence; see rf215_submit. */
static int start_recovery(struct rf215_dev* dev, enum rf215_band band);
/* Submits the recovery stage that follows the one that just completed. */
static void recovery_next_stage(struct rf215_dev* dev);
/* Ends the recovery sequence, with an event for the host. */
static void recovery_end(struct rf215_dev* dev, int status, struct rf215_event* out_evt);
/* Counts an unanswered operation; enough in a row ask for a recovery. */
static void count_fault(struct rf215_dev* dev);
/* Keeps the fault count and the last applied configuration up to date. */
static void track_event(struct rf215_dev* dev, const struct rf215_op* op);
/* Starts the read-out of a frame the chip holds, if the slot is free. */
static bool start_pending_receive(struct rf215_dev* dev, uint8_t index);
/* Puts back a receiver the host wants that nothing has confirmed on air.
 * Returns how long until the next attempt when it is too early for one. */
static rf215_micros_t rearm_receive(struct rf215_dev* dev, uint8_t index);
/* Terminal step: hands the staged event to the host and ends the operation. */
static enum rf215_step_result
step_report(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
/* Aborts the running operation and stages an error for the host. */
static void op_fail(struct rf215_dev* dev, int status);
/* Distributes the four status bytes over the two transceivers. */
static void absorb_irqs(struct rf215_dev* dev);
/* Advances one transceiver by a single step. */
static enum rf215_step_result step_one(struct rf215_dev* dev,
                                       uint8_t index,
                                       struct rf215_xfer* out_xfer,
                                       rf215_micros_t* out_delay,
                                       struct rf215_event* out_evt);

/******************************************************************************/
/** Public Functions **********************************************************/

void rf215_log(const struct rf215_dev* dev, enum rf215_log_level lvl, const char* msg) {
    if ((dev != NULL) && (dev->port.log != NULL)) {
        dev->port.log(dev->port.ctx, lvl, msg);
    }
}

int rf215_init(struct rf215_dev* dev, const struct rf215_port* port) {
    if ((dev == NULL) || (port == NULL)) {
        return RF215_ERR_INVAL;
    }

    if ((port->set_reset == NULL) || (port->now == NULL)) {
        return RF215_ERR_INVAL;
    }

    const struct rf215_dev empty = { 0 };
    *dev = empty;

    dev->port = *port;
    dev->part_number = RF215_PN_UNKNOWN;
    dev->pad_drive = RF215_PAD_DRIVE_DEFAULT;

    init_trx(&dev->trx[0], RF215_BAND_09);
    init_trx(&dev->trx[1], RF215_BAND_24);

    return RF215_OK;
}

int rf215_submit(struct rf215_dev* dev, const struct rf215_req* req) {
    if ((dev == NULL) || (req == NULL)) {
        return RF215_ERR_INVAL;
    }

    if (!rf215_band_enabled(dev, req->band)) {
        return RF215_ERR_INVAL;
    }

    switch (req->kind) {
        case RF215_REQ_RESET:
        case RF215_REQ_CONFIGURE:
        case RF215_REQ_TX:
        case RF215_REQ_RX_START:
        case RF215_REQ_RX_STOP:
        case RF215_REQ_ED:
        case RF215_REQ_IQIF:
            break;
        case RF215_REQ_RECOVER:
            return start_recovery(dev, req->band);
        default:
            return RF215_ERR_INVAL;
    }

    if (dev->op[req->band].next == NULL) {
        start_request(dev, req, false);
        return RF215_OK;
    }

    struct rf215_queued_req* queued = &dev->queue[req->band];

    if (queued->valid) {
        return RF215_ERR_BUSY;
    }

    queued->req = *req;

    if (req->phy != NULL) {
        queued->phy = *req->phy;
        queued->req.phy = &queued->phy;
    }

    queued->valid = true;

    return RF215_OK;
}

int rf215_tx_queue(struct rf215_dev* dev, enum rf215_band band, const uint8_t* data, uint16_t len) {
    if (!rf215_band_enabled(dev, band) || (data == NULL) || (len == 0u)) {
        return RF215_ERR_INVAL;
    }

    if (dev->tx[band].data != NULL) {
        return RF215_ERR_BUSY;
    }

    dev->tx[band].data = data;
    dev->tx[band].len = len;

    return RF215_OK;
}

bool rf215_tx_cancel(struct rf215_dev* dev, enum rf215_band band) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT) || (dev->tx[band].data == NULL)) {
        return false;
    }

    dev->tx[band].data = NULL;
    dev->tx[band].len = 0u;

    return true;
}

void rf215_set_cca(struct rf215_dev* dev,
                   enum rf215_band band,
                   bool enabled,
                   int8_t threshold,
                   uint8_t retries,
                   uint16_t backoff_us) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT)) {
        return;
    }

    dev->trx[band].cca_enabled = enabled;
    dev->trx[band].cca_threshold = threshold;
    dev->trx[band].lbt_retries = retries;
    dev->trx[band].lbt_backoff_us = backoff_us;
}

void rf215_set_rx_buffer(struct rf215_dev* dev, enum rf215_band band, uint8_t* buf, uint16_t cap) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT)) {
        return;
    }

    dev->trx[band].baseband.rx_buf = buf;
    dev->trx[band].baseband.rx_cap = cap;
}

void rf215_set_pad_drive(struct rf215_dev* dev, uint8_t drive) {
    if (dev != NULL) {
        dev->pad_drive = drive & RF215_RG_RF_CFG_DRV_MASK;
    }
}

void rf215_set_band_enabled(struct rf215_dev* dev, enum rf215_band band, bool enabled) {
    if ((dev != NULL) && ((unsigned)band < RF215_TRX_COUNT)) {
        dev->trx[band].enabled = enabled;
    }
}

bool rf215_band_enabled(const struct rf215_dev* dev, enum rf215_band band) {
    return (dev != NULL) && ((unsigned)band < RF215_TRX_COUNT) && dev->trx[band].enabled;
}

void rf215_set_iq_config(struct rf215_dev* dev, const struct rf215_iq_config* iq) {
    if ((dev != NULL) && (iq != NULL)) {
        dev->iq = *iq;
    }
}

const struct rf215_iq_status* rf215_iq_status(const struct rf215_dev* dev) {
    return (dev != NULL) ? &dev->iq_status : NULL;
}

void rf215_seed_backoff(struct rf215_dev* dev, uint32_t seed) {
    if (dev != NULL) {
        dev->backoff_rand = seed;
    }
}

bool rf215_busy(const struct rf215_dev* dev, enum rf215_band band) {
    return (dev != NULL) && ((unsigned)band < RF215_TRX_COUNT) && (dev->op[band].next != NULL);
}

bool rf215_transmitting(const struct rf215_dev* dev, enum rf215_band band) {
    return rf215_busy(dev, band) && (dev->op[band].kind == RF215_REQ_TX);
}

bool rf215_irq_unserviced(const struct rf215_dev* dev) {
    return (dev != NULL) && ((dev->irq_raised != 0u) || (dev->irq_empty >= RF215_IRQ_EMPTY_MAX));
}

bool rf215_recovery_needed(struct rf215_dev* dev) {
    if ((dev == NULL) || !(dev->chip_reset || dev->recovery_due)) {
        return false;
    }

    /* Too soon after the last one: the condition is dropped, not deferred, so
     * a chip that keeps failing is not reset over and over. */
    const rf215_time_t now = dev->port.now(dev->port.ctx);
    const bool due = now >= dev->recovery_allowed_at;

    dev->chip_reset = false;
    dev->recovery_due = false;

    if (due) {
        dev->recovery_allowed_at = now + RF215_RECOVER_INTERVAL_US;
    }

    return due;
}

void rf215_phy_config_default(struct rf215_phy_config* phy, enum rf215_band band) {
    if (phy == NULL) {
        return;
    }

    const struct rf215_phy_config empty = { 0 };
    *phy = empty;

    phy->modulation = RF215_MOD_OFDM;
    phy->freq_hz = (band == RF215_BAND_24) ? RF215_DEFAULT_FREQ_24_HZ : RF215_DEFAULT_FREQ_09_HZ;
    phy->channel_spacing_hz = RF215_DEFAULT_CHANNEL_SPACING_HZ;
    phy->channel = RF215_DEFAULT_CHANNEL;
    phy->ofdm.opt = 0u; /* bandwidth option 1 */
    phy->ofdm.mcs = RF215_DEFAULT_OFDM_MCS;
    phy->qpsk.fchip = 0u; /* 100 kchip/s */
    phy->qpsk.mode = 0u;
    /* IEEE 802.15.4g common sub-GHz MR-FSK mode: 2-FSK, 200 ksym/s, modulation
     * index 1, BT 0.5, whitened, 8 octet preamble. */
    phy->fsk.srate = 3u;
    phy->fsk.midx = 3u;
    phy->fsk.mord = 0u;
    phy->fsk.bt = 0u;
    phy->fsk.fec = false;
    phy->fsk.dw = true;
    phy->fsk.preamble = 8u;
    phy->tx_power = RF215_DEFAULT_TX_POWER;
    phy->fcs = false;
    phy->agc_target = RF215_DEFAULT_AGC_TARGET;
    phy->edd_us = 0u;
}

void rf215_iq_config_default(struct rf215_iq_config* iq) {
    if (iq == NULL) {
        return;
    }

    const struct rf215_iq_config reset_values = {
        .chip_mode = RF215_CHIP_MODE_BBRF,
        .drive_ma = 1u,
        .common_mode = 1u,
        .skew = 2u,
    };

    *iq = reset_values;
}

uint8_t rf215_lqi(int8_t rssi) {
    if ((rssi == RF215_RSSI_INVALID) || (rssi <= -100)) {
        return 0u;
    }

    if (rssi >= 0) {
        return 255u;
    }

    return (uint8_t)(((rssi + 100) * 255) / 100);
}

void rf215_abort(struct rf215_dev* dev, enum rf215_band band) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT)) {
        return;
    }

    struct rf215_op* op = &dev->op[band];

    if (op->next == NULL) {
        return;
    }

    RF215_LOGE(dev, "operation abandoned");

    op->next = NULL;
    op->waiting = false;
    op->evt.kind = RF215_EVT_NONE;

    /* Chip state is unknown from here. Off air, a latched frame end cannot
     * start a stale read-out; the engine re-arms a receiver still wanted. */
    rf215_rx_off_air(&dev->trx[band]);

    /* An assessment register write in flight may not have landed. */
    dev->trx[band].baseband.amedt_valid = false;
    dev->trx[band].baseband.amcs_valid = false;

    /* Release the bus, or every later step would wait on this transfer. */
    if ((dev->xfer_owner == RF215_XFER_OWNER_OP) && (dev->active == (uint8_t)band)) {
        dev->xfer_owner = RF215_XFER_OWNER_NONE;
    }
}

void rf215_abort_unanswered(struct rf215_dev* dev, enum rf215_band band) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT) || (dev->op[band].next == NULL)) {
        return;
    }

    rf215_abort(dev, band);
    count_fault(dev);
}

enum rf215_step_result rf215_step(struct rf215_dev* dev,
                                  struct rf215_xfer* out_xfer,
                                  rf215_micros_t* out_delay,
                                  struct rf215_event* out_evt) {
    if ((dev == NULL) || (out_xfer == NULL) || (out_delay == NULL) || (out_evt == NULL)) {
        return RF215_STEP_IDLE;
    }

    *out_delay = 0u;

    /* A transfer is outstanding: the host owes us rf215_xfer_done() first. */
    if (dev->xfer_owner != RF215_XFER_OWNER_NONE) {
        RF215_LOGE(dev, "step called with a transfer outstanding");
        return RF215_STEP_WAIT;
    }

    /* Interrupt status is read first, for both transceivers, so the step that
     * follows sees its bits. */
    if (dev->irq_raised != 0u) {
        /* A failed read waits out its retry time; nothing else runs
         * meanwhile, since every operation uses the same bus. */
        if (dev->irq_errs != 0u) {
            const rf215_time_t now = dev->port.now(dev->port.ctx);

            if (now < dev->irq_retry_at) {
                *out_delay = (rf215_micros_t)(dev->irq_retry_at - now);

                return RF215_STEP_WAIT;
            }
        }

        dev->irq_raised = 0u;
        dev->xfer_owner = RF215_XFER_OWNER_IRQ;

        out_xfer->op = RF215_XFER_READ;
        out_xfer->reg = IRQS_BASE;
        out_xfer->buf = dev->irq_buf;
        out_xfer->len = IRQS_COUNT;

        return RF215_STEP_XFER;
    }

    /* One bus, two independent state machines: advance whichever has work,
     * round-robin from next_trx so neither starves the other. */
    rf215_micros_t soonest = 0u;
    bool waiting = false;

    for (uint8_t n = 0u; n < RF215_TRX_COUNT; ++n) {
        const uint8_t index = (uint8_t)((dev->next_trx + n) % RF215_TRX_COUNT);
        rf215_micros_t delay = 0u;

        const enum rf215_step_result result = step_one(dev, index, out_xfer, &delay, out_evt);

        if (result == RF215_STEP_IDLE) {
            continue;
        }

        if (result == RF215_STEP_WAIT) {
            /* Remember the earliest deadline and try the other transceiver. */
            if (!waiting || ((delay != 0u) && ((soonest == 0u) || (delay < soonest)))) {
                soonest = delay;
            }
            waiting = true;
            continue;
        }

        dev->next_trx = (uint8_t)((index + 1u) % RF215_TRX_COUNT);
        return result;
    }

    if (waiting) {
        *out_delay = soonest;
        return RF215_STEP_WAIT;
    }

    return RF215_STEP_IDLE;
}

void rf215_xfer_done(struct rf215_dev* dev, int status) {
    if (dev == NULL) {
        return;
    }

    const enum rf215_xfer_owner owner = (enum rf215_xfer_owner)dev->xfer_owner;

    if (owner == RF215_XFER_OWNER_NONE) {
        RF215_LOGE(dev, "transfer completion reported with none outstanding");
        return;
    }

    dev->xfer_owner = RF215_XFER_OWNER_NONE;

    if (status < 0) {
        dev->stats.xfer_errs++;
        RF215_LOGE(dev, "bus transfer failed");

        /* A failed interrupt read leaves the line asserted: retry it, after
         * a wait that doubles while the bus keeps failing. */
        if (owner == RF215_XFER_OWNER_IRQ) {
            const uint8_t doublings = (dev->irq_errs < RF215_IRQ_RETRY_DOUBLINGS)
                ? dev->irq_errs
                : RF215_IRQ_RETRY_DOUBLINGS;

            dev->irq_retry_at =
                dev->port.now(dev->port.ctx) + ((rf215_time_t)RF215_IRQ_RETRY_US << doublings);

            if (dev->irq_errs < 0xFFu) {
                dev->irq_errs++;
            }

            dev->irq_raised = 1u;
        } else {
            op_fail(dev, status);
        }

        return;
    }

    dev->stats.xfers++;

    if (owner == RF215_XFER_OWNER_IRQ) {
        dev->irq_errs = 0u;
        absorb_irqs(dev);
    }
}

void rf215_notify_irq(struct rf215_dev* dev) {
    if (dev != NULL) {
        dev->irq_raised = 1u;
    }
}

const char* rf215_step_name(const struct rf215_dev* dev, enum rf215_band band) {
    if ((dev == NULL) || ((unsigned)band >= RF215_TRX_COUNT) || (dev->op[band].next == NULL)) {
        return "idle";
    }

    const rf215_step_fn current = dev->op[band].next;

    if (current == step_report) {
        return "report";
    }

    static const struct rf215_step_table {
        const struct rf215_step_name* (*steps)(size_t* count);
    } tables[] = {
        {
            rf215_op_reset_steps,
        },
        {
            rf215_op_configure_steps,
        },
        {
            rf215_op_tx_steps,
        },
        {
            rf215_op_rx_steps,
        },
        {
            rf215_op_iqif_steps,
        },
    };

    for (size_t t = 0u; t < (sizeof(tables) / sizeof(tables[0])); ++t) {
        size_t count = 0u;
        const struct rf215_step_name* names = tables[t].steps(&count);

        for (size_t i = 0u; i < count; ++i) {
            if (names[i].fn == current) {
                return names[i].name;
            }
        }
    }

    return "unknown";
}

const struct rf215_stats* rf215_stats(const struct rf215_dev* dev) {
    return (dev != NULL) ? &dev->stats : NULL;
}

/******************************************************************************/
/** Static Functions **********************************************************/

static void init_trx(struct rf215_trx* trx, enum rf215_band band) {
    trx->band = band;
    trx->info = rf215_band_info(band);
    trx->enabled = true;
    trx->radio.irqs = 0u;
    trx->baseband.irqs = 0u;
    trx->cca_enabled = RF215_CCA_DEFAULT_ENABLED;
    trx->cca_threshold = RF215_CCA_DEFAULT_THRESHOLD_DBM;
    trx->lbt_retries = RF215_CCA_DEFAULT_RETRIES;
    trx->lbt_backoff_us = RF215_CCA_DEFAULT_BACKOFF_US;
}

static void start_request(struct rf215_dev* dev, const struct rf215_req* req, bool recovery_stage) {
    struct rf215_op* op = &dev->op[req->band];
    const struct rf215_op empty = { 0 };

    *op = empty;
    op->kind = req->kind;
    op->band = req->band;
    op->recovery_stage = recovery_stage;

    if (req->phy != NULL) {
        op->phy_copy = *req->phy;
        op->phy = &op->phy_copy;
    }

    op->tx_data = req->tx_data;
    op->len = req->tx_len;

    switch (req->kind) {
        case RF215_REQ_RESET:
            op->next = rf215_op_reset_start;
            break;
        case RF215_REQ_CONFIGURE:
            op->next = rf215_op_configure_start;
            break;
        case RF215_REQ_TX:
            op->next = rf215_op_tx_start;
            break;
        case RF215_REQ_RX_START:
            op->next = rf215_op_rx_start;
            break;
        case RF215_REQ_RX_STOP:
            op->next = rf215_op_rx_stop;
            break;
        case RF215_REQ_ED:
            op->next = rf215_op_ed_start;
            break;
        case RF215_REQ_IQIF:
        default:
            op->next = rf215_op_iqif_start;
            break;
    }
}

/* A waiting request goes before a waiting frame, so a settings change or a
 * receive start gets in under a standing transmit load; nothing starts while a
 * recovery runs, except the recovery's own stages. */
static bool start_waiting_work(struct rf215_dev* dev, uint8_t index) {
    if (dev->recovery.active) {
        if ((dev->recovery.stage == RECOVERY_STAGE_IDLE)
            && (index == (uint8_t)dev->recovery.band)) {
            recovery_next_stage(dev);
            return dev->op[index].next != NULL;
        }

        return false;
    }

    struct rf215_queued_req* queued = &dev->queue[index];

    if (queued->valid) {
        queued->valid = false;
        start_request(dev, &queued->req, false);
        return true;
    }

    struct rf215_queued_tx* tx = &dev->tx[index];

    if (tx->data != NULL) {
        const struct rf215_req req = {
            .kind = RF215_REQ_TX,
            .band = (enum rf215_band)index,
            .tx_data = tx->data,
            .tx_len = tx->len,
        };

        tx->data = NULL;
        tx->len = 0u;
        start_request(dev, &req, false);
        return true;
    }

    return false;
}

static void abort_and_report(struct rf215_dev* dev, uint8_t index) {
    struct rf215_op* op = &dev->op[index];

    if (op->next == NULL) {
        return;
    }

    const enum rf215_req_kind kind = op->kind;

    rf215_abort(dev, (enum rf215_band)index);

    /* The engine's own read-out has nobody waiting on it. */
    if (kind == RF215_REQ_RX_READ) {
        return;
    }

    op->kind = kind;
    op->evt.kind = RF215_EVT_ERROR;
    op->evt.op = kind;
    op->evt.band = (enum rf215_band)index;
    op->evt.status = RF215_ERR_ABORTED;
    op->next = step_report;
}

/*
 * Every running operation is abandoned with an event, so its waiter is released.
 * The reset itself waits until the slot it runs in has reported that event: the
 * sequence starts from step_one() once the slot is free. Receivers that were on
 * air are put back by the configure stages and the engine's own re-arm.
 */
static int start_recovery(struct rf215_dev* dev, enum rf215_band band) {
    if (dev->recovery.active) {
        return RF215_ERR_BUSY;
    }

    for (uint8_t i = 0u; i < RF215_TRX_COUNT; ++i) {
        abort_and_report(dev, i);
        /* A frame still waiting is the host's to cancel; see rf215_tx_cancel. */
    }

    dev->recovery.active = true;
    dev->recovery.stage = RECOVERY_STAGE_IDLE;
    dev->recovery.band = band;

    return RF215_OK;
}

static void recovery_next_stage(struct rf215_dev* dev) {
    struct rf215_recovery* rec = &dev->recovery;
    struct rf215_req req = {
        .band = rec->band,
    };

    for (;;) {
        rec->stage++;

        switch (rec->stage) {
            case RECOVERY_STAGE_RESET:
                req.kind = RF215_REQ_RESET;
                break;
            case RECOVERY_STAGE_IQIF:
                req.kind = RF215_REQ_IQIF;
                break;
            case RECOVERY_STAGE_CFG_09:
            case RECOVERY_STAGE_CFG_24: {
                const uint8_t i = (rec->stage == RECOVERY_STAGE_CFG_09) ? 0u : 1u;

                /* A transceiver never configured is left alone. */
                if (!dev->trx[i].phy_valid) {
                    continue;
                }

                req.kind = RF215_REQ_CONFIGURE;
                req.band = dev->trx[i].band;
                req.phy = &dev->trx[i].phy;
                break;
            }
            default:
                /* Nothing left: the sequence completes from the stage that got here. */
                return;
        }

        start_request(dev, &req, true);
        return;
    }
}

static void recovery_end(struct rf215_dev* dev, int status, struct rf215_event* out_evt) {
    const struct rf215_event evt = {
        .kind = (status == RF215_OK) ? RF215_EVT_RECOVERED : RF215_EVT_ERROR,
        .op = RF215_REQ_RECOVER,
        .band = dev->recovery.band,
        .status = status,
    };

    dev->recovery.active = false;
    dev->recovery.stage = RECOVERY_STAGE_IDLE;
    dev->faults = 0u;
    *out_evt = evt;
}

static void count_fault(struct rf215_dev* dev) {
    if (dev->recovery.active) {
        return;
    }

    dev->faults++;

    if (dev->faults >= RF215_FAULT_LIMIT) {
        dev->faults = 0u;
        dev->recovery_due = true;
    }
}

static void track_event(struct rf215_dev* dev, const struct rf215_op* op) {
    switch (op->evt.kind) {
        case RF215_EVT_ERROR:
            /* Only timeouts count towards a reset: a refused request or a bus
             * error says nothing about the chip. */
            if (op->evt.status == RF215_ERR_TIMEOUT) {
                count_fault(dev);
            }
            break;

        case RF215_EVT_CONFIGURED:
            dev->trx[op->band].phy = op->phy_copy;
            dev->trx[op->band].phy_valid = true;
            dev->faults = 0u;
            break;

        case RF215_EVT_READY:
        case RF215_EVT_RX_ARMED:
        case RF215_EVT_TX_DONE:
        case RF215_EVT_RX_FRAME:
            dev->faults = 0u;
            break;

        default:
            break;
    }
}

/* Starts the read-out of a frame that arrived while the slot was idle, before
 * the next one overwrites it. */
static bool start_pending_receive(struct rf215_dev* dev, uint8_t index) {
    struct rf215_trx* trx = &dev->trx[index];

    if ((trx->baseband.rx_buf == NULL) || (trx->baseband.rx_cap == 0u)) {
        return false;
    }

    /*
     * Only a receiver confirmed on air holds a fresh frame: otherwise the
     * length register and buffer still hold an earlier one. RXFE stays
     * latched until the receiver is confirmed or a transition drops it.
     */
    if (!trx->receiving || !trx->radio.armed) {
        return false;
    }

    if (!rf215_take_baseband_irq(trx, RF215_BASEBAND_IRQ_RXFE)) {
        return false;
    }

    /* Clear the matching RXFS too, or the transmit path sees a frame arriving
     * forever and refuses to key up. */
    (void)rf215_take_baseband_irq(trx, RF215_BASEBAND_IRQ_RXFS);

    const struct rf215_op empty = { 0 };
    dev->op[index] = empty;
    dev->op[index].kind = RF215_REQ_RX_READ;
    dev->op[index].band = trx->band;
    dev->op[index].next = rf215_op_rx_read_start;

    return true;
}

/*
 * Re-arms a receiver the host wants that is not confirmed on air (after an
 * abort, a failed transfer or a dropped command). Returns the time left when
 * it is too early to retry; retries no faster than RF215_REARM_RETRY_US.
 */
static rf215_micros_t rearm_receive(struct rf215_dev* dev, uint8_t index) {
    struct rf215_trx* trx = &dev->trx[index];

    if (!trx->receiving || trx->radio.armed) {
        trx->rearm_fails = 0u;

        return 0u;
    }

    const rf215_time_t now = dev->port.now(dev->port.ctx);

    if (now < trx->rearm_after) {
        return (rf215_micros_t)(trx->rearm_after - now);
    }

    /* The retry interval doubles with each failed attempt, up to the cap. */
    rf215_time_t retry = (rf215_time_t)RF215_REARM_RETRY_US
        << ((trx->rearm_fails < 7u) ? trx->rearm_fails : 7u);

    if (retry > RF215_REARM_RETRY_MAX_US) {
        retry = RF215_REARM_RETRY_MAX_US;
    }

    if (trx->rearm_fails < 0xFFu) {
        trx->rearm_fails++;
    }

    trx->rearm_after = now + retry;
    dev->stats.rx_revivals++;

    const struct rf215_op empty = { 0 };
    dev->op[index] = empty;
    dev->op[index].kind = RF215_REQ_RX_READ;
    dev->op[index].band = trx->band;
    dev->op[index].next = rf215_op_rx_rearm_start;

    return 0u;
}

/* Terminal step: hands the staged event to the host and ends the operation. */
static enum rf215_step_result
step_report(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    op->next = NULL;

    return RF215_STEP_EVENT;
}

static void op_fail(struct rf215_dev* dev, int status) {
    struct rf215_op* op = &dev->op[dev->active];

    /* The failed transfer may have moved the chip: state is unknown, as in
     * rf215_abort(). */
    rf215_rx_off_air(&dev->trx[dev->active]);
    dev->trx[dev->active].baseband.amedt_valid = false;
    dev->trx[dev->active].baseband.amcs_valid = false;

    op->evt.kind = RF215_EVT_ERROR;
    op->evt.op = op->kind;
    op->evt.band = op->band;
    op->evt.status = status;
    op->waiting = false;
    op->next = step_report;
}

/* Distributes the four status bytes over the two transceivers. Bits are ORed
 * in and cleared only by the step that claims them. */
static void absorb_irqs(struct rf215_dev* dev) {
    /* Counted here, not where claimed, so a start with no end (a decode
     * failure) shows in the stats. */
    for (uint8_t i = 0u; i < RF215_TRX_COUNT; ++i) {
        const uint8_t bb = dev->irq_buf[2u + i];

        /* In mask mode the status registers report masked interrupts too:
         * nothing claims the bits of a transceiver that is not enabled. */
        if (!dev->trx[i].enabled) {
            continue;
        }

        dev->trx[i].radio.irqs |= dev->irq_buf[i] & RF215_RADIO_IRQ_MASK;
        dev->trx[i].baseband.irqs |= bb & RF215_BASEBAND_IRQ_USED;

        if ((bb & RF215_BASEBAND_IRQ_RXFS) != 0u) {
            dev->stats.rx_starts++;
            /* Timed so a start whose end never arrives can be aged out. */
            dev->trx[i].baseband.rx_start_time = dev->port.now(dev->port.ctx);
            dev->trx[i].baseband.rx_active = true;
        } else if ((bb & RF215_BASEBAND_IRQ_AGCR) != 0u) {
            /* AGC release with no start ends the reception. Beside a start it
             * may belong to the frame before, so the start wins and ages out. */
            dev->trx[i].baseband.rx_active = false;
        }

        if ((bb & RF215_BASEBAND_IRQ_RXFE) != 0u) {
            dev->stats.rx_ends++;
            dev->trx[i].baseband.rx_active = false;
        }

        /* Claimed here: nothing waits on the release itself. */
        dev->trx[i].baseband.irqs &= (uint8_t)~RF215_BASEBAND_IRQ_AGCR;
    }

    /* Count empty status reads so the host can stop answering a stuck line. */
    if ((dev->irq_buf[0] | dev->irq_buf[1] | dev->irq_buf[2] | dev->irq_buf[3]) == 0u) {
        if (dev->irq_empty < 0xFFu) {
            dev->irq_empty++;
        }
    } else {
        dev->irq_empty = 0u;
    }

    /* WAKEUP outside a reset operation: the chip reset uncommanded and lost
     * every register. */
    if (((dev->irq_buf[0] | dev->irq_buf[1]) & RF215_RADIO_IRQ_WAKEUP) != 0u) {
        bool resetting = false;

        for (uint8_t i = 0u; i < RF215_TRX_COUNT; ++i) {
            resetting =
                resetting || ((dev->op[i].next != NULL) && (dev->op[i].kind == RF215_REQ_RESET));
            dev->trx[i].radio.irqs &= (uint8_t)~RF215_RADIO_IRQ_WAKEUP;
        }

        if (!resetting) {
            RF215_LOGE(dev, "chip woke up outside a reset");
            dev->chip_reset = true;
        }
    }

    dev->stats.irqs++;
}

/* Advances one transceiver by a single step; RF215_STEP_IDLE when it has
 * nothing to do. */
static enum rf215_step_result step_one(struct rf215_dev* dev,
                                       uint8_t index,
                                       struct rf215_xfer* out_xfer,
                                       rf215_micros_t* out_delay,
                                       struct rf215_event* out_evt) {
    struct rf215_op* op = &dev->op[index];

    /* A frame the chip holds comes first, or the next one overwrites it; then
     * whatever waits for the slot; then a receiver that fell off air. */
    if (op->next == NULL) {
        if (!start_pending_receive(dev, index) && !start_waiting_work(dev, index)) {
            const rf215_micros_t retry = rearm_receive(dev, index);

            if (op->next == NULL) {
                /* A pending re-arm keeps the host coming back; otherwise the
                 * slot is idle. */
                *out_delay = retry;

                return (retry != 0u) ? RF215_STEP_WAIT : RF215_STEP_IDLE;
            }
        }
    }

    /* Hold a plain delay here rather than trusting the host to have waited. */
    if (op->waiting) {
        const rf215_time_t now = dev->port.now(dev->port.ctx);

        if (now < op->wake) {
            *out_delay = (rf215_micros_t)(op->wake - now);
            return RF215_STEP_WAIT;
        }

        op->waiting = false;
    }

    const enum rf215_step_result result = op->next(dev, op, out_xfer);

    switch (result) {
        case RF215_STEP_XFER:
            dev->xfer_owner = RF215_XFER_OWNER_OP;
            dev->active = index;
            break;

        case RF215_STEP_WAIT: {
            /* Reports the time to the wake or the deadline; zero (interrupt
             * only, no deadline) tells the host not to poll. */
            const rf215_time_t now = dev->port.now(dev->port.ctx);
            const rf215_time_t until = op->waiting ? op->wake : op->deadline;

            /* An expired deadline reports 1, not 0, or the host stops calling. */
            *out_delay = (until > now) ? (rf215_micros_t)(until - now) : ((until != 0u) ? 1u : 0u);
            break;
        }

        case RF215_STEP_EVENT:
            track_event(dev, op);

            /* A transmit that gave way to a read-out goes back to the queue; the
             * read-out starts from the now free slot. */
            if (op->evt.kind == RF215_EVT_TX_DEFERRED) {
                op->evt.kind = RF215_EVT_NONE;
                dev->tx[index].data = op->tx_data;
                dev->tx[index].len = op->len;

                return step_one(dev, index, out_xfer, out_delay, out_evt);
            }

            /* A recovery stage advances the sequence instead of reaching the host. */
            if (op->recovery_stage) {
                const int status = op->evt.status;

                op->evt.kind = RF215_EVT_NONE;

                if (status != RF215_OK) {
                    recovery_end(dev, status, out_evt);
                    dev->stats.events++;
                    break;
                }

                recovery_next_stage(dev);

                if (dev->op[dev->recovery.band].next == NULL) {
                    recovery_end(dev, RF215_OK, out_evt);
                    dev->stats.events++;
                    break;
                }

                return step_one(dev, index, out_xfer, out_delay, out_evt);
            }

            *out_evt = op->evt;
            op->evt.kind = RF215_EVT_NONE;
            dev->stats.events++;
            break;

        case RF215_STEP_IDLE:
        default:
            break;
    }

    return result;
}
