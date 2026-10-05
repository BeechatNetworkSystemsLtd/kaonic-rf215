/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file
 * @brief Scalar types and error codes shared by the RF215 core and its ports.
 */

#ifndef KAONIC_DRIVERS_RF215_TYPES_H__
#define KAONIC_DRIVERS_RF215_TYPES_H__

/******************************************************************************/
/** Includes ******************************************************************/

/* The kernel builds with -nostdinc, so the fixed-width types, bool and size_t
 * come from its own headers there. */
#ifdef __KERNEL__
#include <linux/stddef.h>
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#endif

/******************************************************************************/
/** Types *********************************************************************/

/** @brief Absolute monotonic time in microseconds. */
typedef uint64_t rf215_time_t;

/** @brief Duration in microseconds. */
typedef uint32_t rf215_micros_t;

/** @brief Register address (14 significant bits). */
typedef uint16_t rf215_reg_t;

/**
 * @brief Error codes returned by the core.
 *
 * Values match the corresponding Linux errno codes negated, so a kernel port
 * may return them to its caller unchanged.
 */
enum rf215_err {
    RF215_OK = 0,             /**< Success. */
    RF215_ERR_IO = -5,        /**< Bus transfer failed (-EIO). */
    RF215_ERR_NODEV = -19,    /**< No known part number found (-ENODEV). */
    RF215_ERR_INVAL = -22,    /**< Invalid argument (-EINVAL). */
    RF215_ERR_BUSY = -16,     /**< Operation already in progress (-EBUSY). */
    RF215_ERR_TIMEOUT = -110, /**< Chip did not answer in time (-ETIMEDOUT). */
};

/** @brief Severity of a message passed to @ref rf215_port::log. */
enum rf215_log_level {
    RF215_LOG_ERR = 0, /**< Operation failed; the caller is affected. */
};

#endif /* KAONIC_DRIVERS_RF215_TYPES_H__ */
