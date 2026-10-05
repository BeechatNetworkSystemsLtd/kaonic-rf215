/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Internal helpers shared by the core. Not part of the public interface.
 */

#ifndef KAONIC_DRIVERS_RF215_CORE_H__
#define KAONIC_DRIVERS_RF215_CORE_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215/rf215_api.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Defines *******************************************************************/

#define RF215_LOGE(dev, msg) rf215_log((dev), RF215_LOG_ERR, (msg))

/******************************************************************************/
/** Register Fields ***********************************************************/

/* BBCn_PC.BBEN: cleared while an energy measurement runs so no frame is decoded. */
#define RF215_PC_BBEN (0x04u)

/* RFn_EDC.EDM: 1 triggers a single measurement, 0 restores the automatic
 * per-frame one the receive path reads signal strength from. */
#define RF215_EDC_SINGLE (0x01u)
#define RF215_EDC_AUTO (0x00u)

/* RFn_PLL.LS: synthesizer locked. Unlocked leaves the radio off channel
 * (errata #2); the kick restarts the PLL by writing the reset value with bit 0
 * raised for RF215_PLL_KICK_US. */
#define RF215_PLL_LS (0x02u)
#define RF215_PLL_KICK_ON (0x09u)
#define RF215_PLL_KICK_OFF (0x08u)
#define RF215_PLL_KICK_US (20u)
/* Reads of PLL.LS before a kick, the wait between them, and kicks before
 * giving up. */
#define RF215_PLL_LOCK_POLLS (10u)
#define RF215_PLL_LOCK_POLL_US (20u)
#define RF215_PLL_KICKS (4u)
/* State polls: between reads, and how long a commanded transition is given
 * before the command is re-issued (errata #2/#6: a command can be dropped). */
#define RF215_STATE_POLL_US (100u)
#define RF215_STATE_TIMEOUT_US (20000u)
#define RF215_STATE_ATTEMPTS (3u)

/*
 * Interval at which the engine re-arms a receiver the host wants on air but
 * nothing has confirmed there. The chip raises no interrupt when it drops out
 * of RX, so nothing else would put it back.
 */
#define RF215_REARM_RETRY_US (10000u)
/* Each failed attempt doubles the wait, up to this. */
#define RF215_REARM_RETRY_MAX_US (1000000u)

/*
 * A failed status read is retried, because the chip holds its interrupt line
 * until one succeeds: first after RF215_IRQ_RETRY_US, then doubling each time.
 */
#define RF215_IRQ_RETRY_US (1000u)
#define RF215_IRQ_RETRY_DOUBLINGS (6u)

/* Status reads in a row with nothing in them before the line counts as stuck. */
#define RF215_IRQ_EMPTY_MAX (16u)

/******************************************************************************/
/** Types *********************************************************************/

/* Which transfer, if any, the host is currently performing. */
enum rf215_xfer_owner {
    RF215_XFER_OWNER_NONE = 0,
    RF215_XFER_OWNER_IRQ,
    RF215_XFER_OWNER_OP,
};

/******************************************************************************/
/** Public Functions **********************************************************/

void rf215_log(const struct rf215_dev* dev, enum rf215_log_level lvl, const char* msg);

/******************************************************************************/
/** Register addresses ********************************************************/

/* A transceiver's registers are offsets (RF215_RG_RFXX_*, RF215_RG_BBCX_*)
 * placed by its band. */
static inline rf215_reg_t rf215_radio_reg(const struct rf215_trx* trx, rf215_reg_t reg) {
    return (rf215_reg_t)(trx->info->radio_base + reg);
}

static inline rf215_reg_t rf215_baseband_reg(const struct rf215_trx* trx, rf215_reg_t reg) {
    return (rf215_reg_t)(trx->info->baseband_base + reg);
}

static inline rf215_reg_t rf215_frame_buffer_reg(const struct rf215_trx* trx, rf215_reg_t reg) {
    return (rf215_reg_t)(trx->info->frame_buffer_base + reg);
}

/******************************************************************************/
/** Interrupt and deadline helpers ********************************************/

static inline void
rf215_set_deadline(struct rf215_dev* dev, struct rf215_op* op, rf215_micros_t us) {
    op->deadline = dev->port.now(dev->port.ctx) + us;
}

static inline bool rf215_past_deadline(struct rf215_dev* dev, const struct rf215_op* op) {
    return dev->port.now(dev->port.ctx) >= op->deadline;
}

/* Claims the requested interrupt bits if pending; other bits stay latched. */
static inline bool rf215_take_radio_irq(struct rf215_trx* trx, uint8_t mask) {
    if ((trx->radio.irqs & mask) != 0u) {
        trx->radio.irqs &= (uint8_t)~mask;
        return true;
    }

    return false;
}

static inline bool rf215_take_baseband_irq(struct rf215_trx* trx, uint8_t mask) {
    if ((trx->baseband.irqs & mask) != 0u) {
        trx->baseband.irqs &= (uint8_t)~mask;
        return true;
    }

    return false;
}

/*
 * Marks the receiver as not confirmed on air and drops the RXFS/RXFE latches
 * and frame timing: a stale RXFE would read out a stale buffer as a new frame,
 * a stale RXFS would block transmits.
 */
static inline void rf215_rx_off_air(struct rf215_trx* trx) {
    trx->radio.armed = false;
    trx->baseband.irqs &= (uint8_t)~(RF215_BASEBAND_IRQ_RXFS | RF215_BASEBAND_IRQ_RXFE);
    trx->baseband.rx_start_time = 0u;
    trx->baseband.rx_active = false;
}

/******************************************************************************/
/** Step Constructors *********************************************************/

/* Write one register, then continue at `next`. */
static inline enum rf215_step_result rf215_write8(struct rf215_op* op,
                                                  struct rf215_xfer* out,
                                                  rf215_reg_t reg,
                                                  uint8_t value,
                                                  rf215_step_fn next) {
    op->next = next;
    op->buf[0] = value;

    out->op = RF215_XFER_WRITE;
    out->reg = reg;
    out->buf = op->buf;
    out->len = 1u;

    return RF215_STEP_XFER;
}

/* Write a 16-bit value to `reg` and the register above it, then continue at
 * `next`. The chip's 16-bit fields are little endian and contiguous. */
static inline enum rf215_step_result rf215_write16(struct rf215_op* op,
                                                   struct rf215_xfer* out,
                                                   rf215_reg_t reg,
                                                   uint16_t value,
                                                   rf215_step_fn next) {
    op->next = next;
    op->buf[0] = (uint8_t)(value & 0xffu);
    op->buf[1] = (uint8_t)(value >> 8);

    out->op = RF215_XFER_WRITE;
    out->reg = reg;
    out->buf = op->buf;
    out->len = 2u;

    return RF215_STEP_XFER;
}

/* Read `len` bytes into op->buf, then continue at `next`. */
static inline enum rf215_step_result rf215_read(struct rf215_op* op,
                                                struct rf215_xfer* out,
                                                rf215_reg_t reg,
                                                size_t len,
                                                rf215_step_fn next) {
    op->next = next;

    out->op = RF215_XFER_READ;
    out->reg = reg;
    out->buf = op->buf;
    out->len = len;

    return RF215_STEP_XFER;
}

/* Wait `delay`, then continue at `next`; an early rf215_step() cannot run
 * `next` sooner. Uses op->wake, not op->deadline, so a deadline set earlier
 * survives the sleep. */
static inline enum rf215_step_result
rf215_delay(struct rf215_dev* dev, struct rf215_op* op, rf215_micros_t delay, rf215_step_fn next) {
    op->next = next;
    op->wake = dev->port.now(dev->port.ctx) + delay;
    op->waiting = true;

    return RF215_STEP_WAIT;
}

/* Report an event to the host, then continue at `next` (NULL ends the op). */
static inline enum rf215_step_result
rf215_emit(struct rf215_op* op, enum rf215_event_kind kind, int status, rf215_step_fn next) {
    op->next = next;
    op->evt.kind = kind;
    op->evt.op = op->kind;
    op->evt.band = op->band;
    op->evt.status = status;

    return RF215_STEP_EVENT;
}

#endif /* KAONIC_DRIVERS_RF215_CORE_H__ */
