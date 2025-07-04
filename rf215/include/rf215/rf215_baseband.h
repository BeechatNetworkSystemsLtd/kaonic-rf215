/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#ifndef KAONIC_DRIVERS_RF215_BASEBAND_H__
#define KAONIC_DRIVERS_RF215_BASEBAND_H__

/*****************************************************************************/

#include "rf215.h"

/*****************************************************************************/

int rf215_baseband_rx_frame(struct rf215_trx* trx,
                            rf215_millis_t timeout,
                            struct rf215_frame* frame);
int rf215_baseband_rx(struct rf215_trx* trx, rf215_millis_t timeout, uint8_t* data, uint16_t* len);
int rf215_baseband_rx_load(struct rf215_trx* trx, uint8_t* data, uint16_t* len);

int rf215_baseband_cca_tx_frame(struct rf215_trx* trx, struct rf215_frame* frame);
int rf215_baseband_cca_tx(struct rf215_trx* trx, uint8_t* data, uint16_t length);

int rf215_baseband_tx_frame(struct rf215_trx* trx, struct rf215_frame* frame);
int rf215_baseband_tx(struct rf215_trx* trx, uint8_t* data, uint16_t length);
int rf215_baseband_tx_load(struct rf215_trx* trx, uint8_t* data, uint16_t length);

/*****************************************************************************/

#endif // KAONIC_DRIVERS_RF215_BASEBAND_H__
