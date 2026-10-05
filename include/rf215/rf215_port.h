/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file
 * @brief The interface a host platform provides to the RF215 core.
 *
 * The core never blocks and never touches the bus itself. It asks for one
 * transfer at a time by returning it from @ref rf215_step; the host performs
 * it, synchronously or asynchronously, and reports completion with
 * @ref rf215_xfer_done. A port therefore supplies only three things: the reset
 * pin, a monotonic clock, and optionally a log sink.
 */

#ifndef KAONIC_DRIVERS_RF215_PORT_H__
#define KAONIC_DRIVERS_RF215_PORT_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_types.h"

/******************************************************************************/
/** Types *********************************************************************/

/** @brief Direction of a bus transfer. */
enum rf215_xfer_op {
    RF215_XFER_READ = 0, /**< Read @ref rf215_xfer::len bytes into the buffer. */
    RF215_XFER_WRITE,    /**< Write @ref rf215_xfer::len bytes from the buffer. */
};

/**
 * @brief A single bus transfer requested by the core.
 *
 * The buffer belongs to the core and stays valid until the host reports the
 * transfer complete. It is suitable for DMA: the core places it in storage that
 * is not on any stack.
 */
struct rf215_xfer {
    enum rf215_xfer_op op; /**< Direction. */
    rf215_reg_t reg;       /**< First register address. */
    void* buf;             /**< Data buffer, owned by the core. */
    size_t len;            /**< Number of bytes to transfer. */
};

/**
 * @brief Platform operations required by the core.
 */
struct rf215_port {
    void* ctx; /**< Opaque host context, passed back to every callback. */

    /**
     * @brief Drive the chip's reset pin.
     *
     * @param ctx       Host context.
     * @param asserted  True holds the chip in reset, false releases it.
     */
    void (*set_reset)(void* ctx, bool asserted);

    /**
     * @brief Read the monotonic clock.
     *
     * Must not jump backwards. Microsecond resolution is expected; coarser
     * clocks only make the core's timeouts more generous.
     *
     * @param ctx  Host context.
     *
     * @return Current time in microseconds.
     */
    rf215_time_t (*now)(void* ctx);

    /**
     * @brief Emit a log message. May be NULL to discard all logging.
     *
     * @param ctx  Host context.
     * @param lvl  Severity.
     * @param msg  Already-formatted, NUL-terminated message.
     */
    void (*log)(void* ctx, enum rf215_log_level lvl, const char* msg);
};

#endif /* KAONIC_DRIVERS_RF215_PORT_H__ */
