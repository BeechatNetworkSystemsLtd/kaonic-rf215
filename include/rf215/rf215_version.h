/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file
 * @brief Version of the RF215 driver: the core and the ports built from it.
 */

#ifndef KAONIC_DRIVERS_RF215_VERSION_H__
#define KAONIC_DRIVERS_RF215_VERSION_H__

/******************************************************************************/
/** Defines *******************************************************************/

#define RF215_VERSION_MAJOR 0
#define RF215_VERSION_MINOR 1
#define RF215_VERSION_PATCH 0

#define RF215_VERSION_STR_(x) #x
#define RF215_VERSION_STR(x) RF215_VERSION_STR_(x)

/** @brief The version as "major.minor.patch". */
#define RF215_VERSION_STRING               \
    RF215_VERSION_STR(RF215_VERSION_MAJOR) \
    "." RF215_VERSION_STR(RF215_VERSION_MINOR) "." RF215_VERSION_STR(RF215_VERSION_PATCH)

#endif /* KAONIC_DRIVERS_RF215_VERSION_H__ */
