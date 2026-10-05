/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * MR-O-QPSK frontend tables.
 *
 * Datasheet Table 6-106 (O-QPSK receiver frontend configuration, AGC
 * settings). Unlike MR-OFDM, both bands use the same configuration, so one
 * table serves them.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_phy.h"

/******************************************************************************/
/** Defines *******************************************************************/

/*
 * The energy detect duration is a multiple of the chip period: ten symbols at
 * 100 kchip/s, five at 200, four above.
 */
#define QPSK_SYMBOL_US (128u)

/******************************************************************************/
/** Data **********************************************************************/

static const struct rf215_frontend_cfg qpsk_cfg[4] = {
    /* 100 kchip/s. Direct modulation is required at this chip rate. */
    {
        .tx.sr = RF215_SR_400,
        .tx.rcut = RF215_RCUT_0750,
        .tx.direct_mod = true,
        .tx.lpfcut = RF215_LPF_400,
        .tx.paramp = RF215_PARAMP_32,
        .rx.sr = RF215_SR_400,
        .rx.rcut = RF215_RCUT_0375,
        .rx.bw = RF215_BW_160_250,
        .rx.if_shift = false,
        .agc.enabled = true,
        .agc.avg = RF215_AVGS_32,
        .edd_us = 10u * QPSK_SYMBOL_US,
    },
    /* 200 kchip/s. Direct modulation is required at this chip rate. */
    {
        .tx.sr = RF215_SR_800,
        .tx.rcut = RF215_RCUT_0750,
        .tx.direct_mod = true,
        .tx.lpfcut = RF215_LPF_400,
        .tx.paramp = RF215_PARAMP_16,
        .rx.sr = RF215_SR_800,
        .rx.rcut = RF215_RCUT_0375,
        .rx.bw = RF215_BW_250_250,
        .rx.if_shift = false,
        .agc.enabled = true,
        .agc.avg = RF215_AVGS_32,
        .edd_us = 5u * QPSK_SYMBOL_US,
    },
    /* 1000 kchip/s */
    {
        .tx.sr = RF215_SR_4000,
        .tx.rcut = RF215_RCUT_0750,
        .tx.lpfcut = RF215_LPF_1000,
        .tx.paramp = RF215_PARAMP_4,
        .rx.sr = RF215_SR_4000,
        .rx.rcut = RF215_RCUT_0250,
        .rx.bw = RF215_BW_1000_1000,
        .rx.if_shift = false,
        .agc.enabled = true,
        .agc.avg = RF215_AVGS_8,
        .edd_us = 4u * QPSK_SYMBOL_US,
    },
    /* 2000 kchip/s */
    {
        .tx.sr = RF215_SR_4000,
        .tx.rcut = RF215_RCUT_1000,
        .tx.lpfcut = RF215_LPF_1000,
        .tx.paramp = RF215_PARAMP_4,
        .rx.sr = RF215_SR_4000,
        .rx.rcut = RF215_RCUT_0500,
        .rx.bw = RF215_BW_2000_2000,
        .rx.if_shift = false,
        .agc.enabled = true,
        .agc.avg = RF215_AVGS_8,
        .edd_us = 4u * QPSK_SYMBOL_US,
    },
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_frontend_cfg* rf215_phy_qpsk_frontend(uint8_t fchip) {
    return &qpsk_cfg[fchip & 0x03u];
}

bool rf215_phy_qpsk_direct_mod(uint8_t fchip) {
    return qpsk_cfg[fchip & 0x03u].tx.direct_mod;
}
