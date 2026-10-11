// SPDX-License-Identifier: GPL-2.0
/*
 * Drives the RF215 core against a Linux SPI device.
 *
 * The core never blocks: every transfer it asks for is performed with
 * spi_sync() from a sleepable context, so the engine runs from a threaded
 * interrupt handler and a worker thread rather than a callback chain. Both
 * transceivers share one bus, and rf->lock serialises them.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Forward Declarations ******************************************************/

static void rf215_finish(struct rf215_iface* ifc, int status);
/* Routes a core event to the interface it concerns. */
static void rf215_on_event(struct rf215_spi* rf, const struct rf215_event* evt);
/* Drives the core until it has nothing left to do. Must hold rf->lock. */
static void rf215_run_locked(struct rf215_spi* rf);
/* Queues the frames the stack handed down, then runs the engine. */
static void rf215_service_locked(struct rf215_spi* rf);
/* Idle time the controller must add between bytes at the configured rate. */
static u32 rf215_spi_gap_ns(const struct rf215_spi* rf);
/* Schedules the recovery work. */
static void rf215_request_recovery(struct rf215_spi* rf);
/* Keeps the bus controller from suspending while this interface is up. */
static void rf215_bus_hold(struct rf215_iface* ifc);
/* Lets the bus controller suspend again. */
static void rf215_bus_release(struct rf215_iface* ifc);
/* Unmasks an interrupt line that was masked as stuck. Must hold rf->lock. */
static void rf215_irq_rest_end(struct rf215_spi* rf);

/******************************************************************************/
/** Public Functions **********************************************************/

/******************************************************************************/
/** Platform Callbacks ********************************************************/

void rf215_spi_set_reset(void* ctx, bool asserted) {
    struct rf215_spi* rf = ctx;

    gpiod_set_value_cansleep(rf->reset_gpio, asserted ? 1 : 0);
}

rf215_time_t rf215_spi_now(void* ctx) {
    (void)ctx;

    return ktime_to_us(ktime_get());
}

void rf215_spi_log(void* ctx, enum rf215_log_level lvl, const char* msg) {
    struct rf215_spi* rf = ctx;

    /* The core logs failures only, and a dead chip repeats them: rate limited. */
    (void)lvl;

    dev_err_ratelimited(&rf->spi->dev, "%s\n", msg);
}

/******************************************************************************/
/** Bus ***********************************************************************/

/*
 * Performs one register or frame access. Above 8 MHz the SCK period is below the 125 ns the
 * chip needs between bytes (tSPI_5), so a word delay is added, chosen from the configured rate.
 * A short access is one transfer; a frame is two, using the caller's buffer directly.
 */
int rf215_spi_xfer(struct rf215_spi* rf, const struct rf215_xfer* xfer) {
    const bool write = xfer->op == RF215_XFER_WRITE;
    const bool small = xfer->len <= RF215_SPI_SMALL_MAX;
    const struct spi_delay gap = {
        .value = rf215_spi_gap_ns(rf),
        .unit = SPI_DELAY_UNIT_NSECS,
    };
    struct spi_transfer xfers[2] = {};
    u16 hdr = xfer->reg & RF215_SPI_ADDR_MASK;
    int ret;

    if (!xfer->buf || !xfer->len) {
        return -EINVAL;
    }

    if (write) {
        hdr |= RF215_SPI_WRITE_FLAG;
    }

    rf->hdr[0] = hdr >> 8;
    rf->hdr[1] = hdr & 0xff;

    xfers[0].tx_buf = rf->hdr;
    xfers[0].len = RF215_SPI_HDR_LEN;
    xfers[0].word_delay = gap;
    xfers[1].word_delay = gap;

    if (small) {
        if (write) {
            memcpy(rf->hdr + RF215_SPI_HDR_LEN, xfer->buf, xfer->len);
        } else {
            memset(rf->hdr + RF215_SPI_HDR_LEN, 0, xfer->len);
            xfers[0].rx_buf = rf->small;
        }

        xfers[0].len += xfer->len;
    } else {
        if (write) {
            xfers[1].tx_buf = xfer->buf;
        } else {
            xfers[1].rx_buf = xfer->buf;
        }

        xfers[1].len = xfer->len;
    }

    ret = spi_sync_transfer(rf->spi, xfers, small ? 1 : 2);
    if (ret) {
        dev_err_ratelimited(&rf->spi->dev,
                            "spi %s of %zu bytes at 0x%04x failed: %d\n",
                            write ? "write" : "read",
                            xfer->len,
                            xfer->reg,
                            ret);
        return ret;
    }

    if (small && !write) {
        memcpy(xfer->buf, rf->small + RF215_SPI_HDR_LEN, xfer->len);
    }

    return 0;
}

const char* rf215_iface_name(const struct rf215_iface* ifc) {
    if (ifc->ndev) {
        return netdev_name(ifc->ndev);
    }

    if (ifc->hw) {
        return wpan_phy_name(ifc->hw->phy);
    }

    return "unattached";
}

void rf215_work_fn(struct kthread_work* work) {
    struct rf215_spi* rf = container_of(work, struct rf215_spi, work);

    rt_mutex_lock(&rf->lock);
    rf215_service_locked(rf);

    /* A masked line whose status read now succeeded is unmasked early, not after the rest. */
    if (!rf215_irq_unserviced(&rf->core)) {
        rf215_irq_rest_end(rf);
    }

    rt_mutex_unlock(&rf->lock);
}

enum hrtimer_restart rf215_timer_fn(struct hrtimer* timer) {
    struct rf215_spi* rf = container_of(timer, struct rf215_spi, timer);

    rf215_kick(rf);

    return HRTIMER_NORESTART;
}

void rf215_kick(struct rf215_spi* rf) {
    if (!READ_ONCE(rf->stopping)) {
        kthread_queue_work(rf->worker, &rf->work);
    }
}

/* Ends the rest of a masked interrupt line: reads the chip status first, then unmasks. */
void rf215_irq_work_fn(struct kthread_work* work) {
    struct rf215_spi* rf = container_of(work, struct rf215_spi, irq_work.work);

    rt_mutex_lock(&rf->lock);

    if (rf->irq_off && !rf->stopping) {
        rf215_notify_irq(&rf->core);
        rf215_service_locked(rf);
        rf215_irq_rest_end(rf);
    }

    rt_mutex_unlock(&rf->lock);
}

irqreturn_t rf215_irq_thread(int irq, void* data) {
    struct rf215_spi* rf = data;

    rf215_notify_irq(&rf->core);

    rt_mutex_lock(&rf->lock);
    rf215_service_locked(rf);

    /*
     * Level triggered: a line still asserted here would re-enter at once, forever, at
     * real-time priority. Mask it and retry after RF215_IRQ_REST_MS.
     */
    if (rf215_irq_unserviced(&rf->core) && !rf->irq_off && !rf->stopping) {
        rf->irq_off = true;
        disable_irq_nosync(irq);
        dev_warn_ratelimited(
            &rf->spi->dev, "interrupt line stuck; masked for %u ms\n", RF215_IRQ_REST_MS);
        kthread_queue_delayed_work(rf->worker, &rf->irq_work, msecs_to_jiffies(RF215_IRQ_REST_MS));
    }

    rt_mutex_unlock(&rf->lock);

    /*
     * Always claimed on purpose: IRQ_NONE for a pass that found nothing (another thread read
     * the status first) would let the spurious-interrupt detector disable a working line.
     */
    return IRQ_HANDLED;
}

/* Runs one operation on one transceiver to completion. Not for the transmit
 * path, which reports upward rather than through a completion. */
int rf215_run_op(struct rf215_iface* ifc, const struct rf215_req* req) {
    struct rf215_spi* rf = ifc->rf;
    int ret;

    /* The completion and the status are one per interface, so a second
     * caller would reset them under the first. */
    mutex_lock(&ifc->op_lock);

    rt_mutex_lock(&rf->lock);
    reinit_completion(&ifc->op_done);
    ifc->op_status = -ETIMEDOUT;

    /* The core queues the request behind whatever holds the slot and starts it
     * from this pass or a later one. */
    ret = rf215_submit(&rf->core, req);
    rf215_service_locked(rf);
    rt_mutex_unlock(&rf->lock);

    if (ret) {
        dev_err_ratelimited(&rf->spi->dev, "submit failed: %d\n", ret);
        goto out;
    }

    if (wait_for_completion_timeout(&ifc->op_done, msecs_to_jiffies(RF215_OP_TIMEOUT_MS))) {
        ret = ifc->op_status;
        goto out;
    }

    rt_mutex_lock(&rf->lock);

    /* It may have finished while this was giving up on it. */
    if (try_wait_for_completion(&ifc->op_done)) {
        ret = ifc->op_status;
    } else {
        dev_err_ratelimited(&rf->spi->dev,
                            "%s: operation timed out at step %s\n",
                            rf215_iface_name(ifc),
                            rf215_step_name(&rf->core, ifc->band));

        /* Abort it, or the core refuses every later operation on this transceiver. Unless a
         * transmit holds the slot by now: then recovery already aborted this operation, and
         * aborting the transmit would orphan its frame. */
        if (!rf215_transmitting(&rf->core, ifc->band)) {
            rf215_abort_unanswered(&rf->core, ifc->band);
        }
        ret = -ETIMEDOUT;
    }

    rt_mutex_unlock(&rf->lock);

out:
    mutex_unlock(&ifc->op_lock);

    return ret;
}

/*
 * Drops the frame in flight. tx_skb is detached under rf->lock, which the engine reads and
 * completes it under. The transmit is aborted first: the core points into the frame.
 */
void rf215_drop_tx(struct rf215_iface* ifc) {
    struct rf215_spi* rf = ifc->rf;
    struct sk_buff* skb;

    rt_mutex_lock(&rf->lock);

    if (!rf215_tx_cancel(&rf->core, ifc->band) && rf215_transmitting(&rf->core, ifc->band)) {
        rf215_abort(&rf->core, ifc->band);
    }

    ifc->tx_queued = false;
    skb = ifc->tx_skb;
    ifc->tx_skb = NULL;

    rt_mutex_unlock(&rf->lock);

    if (skb) {
        dev_kfree_skb_any(skb);
    }
}

/*
 * Runs the core's recovery: reset, I/Q interface, the configurations, receivers back. The
 * core abandons every running operation with RF215_ERR_ABORTED, which releases its waiter
 * and fails its frame through the usual event path; a frame still queued is dropped here.
 */
void rf215_recover_fn(struct work_struct* work) {
    struct rf215_spi* rf = container_of(work, struct rf215_spi, recover);
    struct rf215_iface* chip = rf215_chip_iface(rf);
    const struct rf215_req recover_req = {
        .kind = RF215_REQ_RECOVER,
        .band = chip->band,
    };
    unsigned int i;
    int ret;

    if (READ_ONCE(rf->stopping)) {
        return;
    }

    dev_warn(&rf->spi->dev, "chip stopped answering; resetting it\n");

    mutex_lock(&rf->cfg_lock);

    rt_mutex_lock(&rf->lock);

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        struct rf215_iface* ifc = &rf->iface[i];

        if (rf215_tx_cancel(&rf->core, ifc->band)) {
            ifc->tx_queued = false;
            rf->fe->tx_done(ifc, -EIO);
        }
    }

    rt_mutex_unlock(&rf->lock);

    ret = rf215_run_op(chip, &recover_req);
    if (ret) {
        dev_err(&rf->spi->dev, "recovery failed: %d\n", ret);
    }

    mutex_unlock(&rf->cfg_lock);
}

int rf215_start_receive(struct rf215_iface* ifc) {
    const struct rf215_req req = {
        .kind = RF215_REQ_RX_START,
        .band = ifc->band,
    };

    int ret;

    rf215_set_rx_buffer(&ifc->rf->core, ifc->band, ifc->rx_buf, RF215_RX_BUF_LEN);

    rf215_bus_hold(ifc);

    ret = rf215_run_op(ifc, &req);
    if (ret) {
        /* No stop follows a start that failed. */
        rf215_bus_release(ifc);
    }

    return ret;
}

int rf215_stop_receive(struct rf215_iface* ifc) {
    const struct rf215_req req = {
        .kind = RF215_REQ_RX_STOP,
        .band = ifc->band,
    };
    int ret;

    ret = rf215_run_op(ifc, &req);
    rf215_bus_release(ifc);

    return ret;
}

/* Chip-wide, but runs on the interface's slot like any other operation. */
int rf215_apply_iq(struct rf215_iface* ifc) {
    const struct rf215_req req = {
        .kind = RF215_REQ_IQIF,
        .band = ifc->band,
    };

    return rf215_run_op(ifc, &req);
}

int rf215_configure(struct rf215_iface* ifc) {
    const struct rf215_req req = {
        .kind = RF215_REQ_CONFIGURE,
        .band = ifc->band,
        .phy = &ifc->phy,
    };

    return rf215_run_op(ifc, &req);
}

/******************************************************************************/
/** Static Functions **********************************************************/

/******************************************************************************/
/** Engine ********************************************************************/

static u32 rf215_spi_gap_ns(const struct rf215_spi* rf) {
    const u32 period_ns = NSEC_PER_SEC / max_t(u32, rf->spi->max_speed_hz, 1);

    return period_ns < RF215_SPI_INTERBYTE_NS ? RF215_SPI_INTERBYTE_NS : 0;
}

/*
 * Holds a runtime-PM reference on the SPI controller while the interface is up. Without it
 * the controller autosuspends during a frame and the end-of-frame status read pays the resume.
 */
static void rf215_bus_hold(struct rf215_iface* ifc) {
    if (!ifc->bus_held) {
        /* The count is raised even when the resume fails, so the release
         * below always balances it. */
        pm_runtime_get_sync(ifc->rf->spi->controller->dev.parent);
        ifc->bus_held = true;
    }
}

static void rf215_bus_release(struct rf215_iface* ifc) {
    if (ifc->bus_held) {
        struct device* bus = ifc->rf->spi->controller->dev.parent;

        ifc->bus_held = false;
        pm_runtime_mark_last_busy(bus);
        pm_runtime_put_autosuspend(bus);
    }
}

static void rf215_irq_rest_end(struct rf215_spi* rf) {
    if (rf->irq_off && !rf->stopping) {
        rf->irq_off = false;
        enable_irq(rf->spi->irq);
    }
}

static void rf215_request_recovery(struct rf215_spi* rf) {
    if (!READ_ONCE(rf->stopping)) {
        schedule_work(&rf->recover);
    }
}

static void rf215_finish(struct rf215_iface* ifc, int status) {
    ifc->op_status = status;
    complete(&ifc->op_done);
}

static void rf215_on_event(struct rf215_spi* rf, const struct rf215_event* evt) {
    struct rf215_iface* ifc;

    if ((unsigned int)evt->band >= RF215_TRX_COUNT) {
        return;
    }

    ifc = &rf->iface[evt->band];

    switch (evt->kind) {
        case RF215_EVT_READY:
        case RF215_EVT_CONFIGURED:
        case RF215_EVT_RX_ARMED:
        case RF215_EVT_RX_STOPPED:
        case RF215_EVT_IQIF_DONE:
        case RF215_EVT_RECOVERED:
            rf215_finish(ifc, 0);
            break;

        case RF215_EVT_ED_DONE:
            ifc->ed_dbm = evt->rssi;
            ifc->ed_level = rf215_lqi(evt->rssi);
            rf215_finish(ifc, 0);
            break;

        case RF215_EVT_TX_DONE:
            ifc->tx_queued = false;
            rf->fe->tx_done(ifc, 0);
            break;

        case RF215_EVT_TX_BUSY:
            /* Listen-before-talk gave up; the frame is dropped. */
            ifc->tx_queued = false;
            rf->fe->tx_done(ifc, -EBUSY);
            break;

        case RF215_EVT_RX_FRAME:
            ifc->rx_rssi = evt->rssi;
            rf->fe->rx_frame(ifc, ifc->rx_buf, evt->len, evt->rssi);
            break;

        case RF215_EVT_ERROR:
            dev_err_ratelimited(&rf->spi->dev,
                                "%s: operation failed at step %s: %d\n",
                                rf215_iface_name(ifc),
                                rf215_step_name(&rf->core, evt->band),
                                evt->status);

            if (evt->op == RF215_REQ_TX && ifc->tx_skb) {
                ifc->tx_queued = false;
                rf->fe->tx_done(ifc, evt->status);
            } else if (evt->op != RF215_REQ_RX_READ) {
                /* Nobody waits on the engine's own read-out; completing on it would hand
                 * its error to the next interface operation. */
                rf215_finish(ifc, evt->status);
            }
            break;

        case RF215_EVT_NONE:
        default:
            break;
    }
}

static void rf215_run_locked(struct rf215_spi* rf) {
    for (;;) {
        struct rf215_xfer xfer;
        struct rf215_event evt;
        rf215_micros_t delay;
        int ret;

        switch (rf215_step(&rf->core, &xfer, &delay, &evt)) {
            case RF215_STEP_XFER:
                ret = rf215_spi_xfer(rf, &xfer);
                rf215_xfer_done(&rf->core, ret);
                break;

            case RF215_STEP_EVENT:
                rf215_on_event(rf, &evt);
                break;

            case RF215_STEP_WAIT:
                /* No deadline: every transceiver is waiting on an interrupt. */
                if (!delay) {
                    return;
                }

                if (delay < RF215_INLINE_WAIT_US) {
                    usleep_range(delay, delay);
                    break;
                }

                /* Teardown has cancelled the timer; it must not be armed again. */
                if (rf->stopping) {
                    return;
                }

                /* A running timer is moved: the engine tracks its own deadlines, so an
                 * early pass is harmless and a late one is not. */
                hrtimer_start(
                    &rf->timer, ns_to_ktime((u64)delay * NSEC_PER_USEC), HRTIMER_MODE_REL);
                return;

            case RF215_STEP_IDLE:
            default:
                return;
        }
    }
}

/* Hands the core each frame the stack handed down, then runs the engine. */
static void rf215_service_locked(struct rf215_spi* rf) {
    unsigned int i;

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        struct rf215_iface* ifc = &rf->iface[i];

        if (ifc->tx_skb && !ifc->tx_queued) {
            ifc->tx_queued =
                rf215_tx_queue(&rf->core, ifc->band, ifc->tx_skb->data, ifc->tx_skb->len) == 0;
        }
    }

    rf215_run_locked(rf);

    if (rf215_recovery_needed(&rf->core)) {
        rf215_request_recovery(rf);
    }
}
