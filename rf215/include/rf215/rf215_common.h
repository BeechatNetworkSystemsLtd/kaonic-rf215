/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#ifndef KAONIC_DRIVERS_RF215_COMMON_H__
#define KAONIC_DRIVERS_RF215_COMMON_H__

/*****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*****************************************************************************/

#define RF215_REG_VALUE_ARRAY_COUNT(arr) (sizeof(arr) / sizeof(arr[0]))

/*****************************************************************************/

typedef uint32_t rf215_millis_t; /* Time duration in milliseconds */

/*****************************************************************************/

#endif // KAONIC_DRIVERS_RF215_COMMON_H__
