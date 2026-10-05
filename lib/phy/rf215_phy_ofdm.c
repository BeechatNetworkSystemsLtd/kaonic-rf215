/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * MR-OFDM frontend tables.
 *
 * Datasheet Table 6-90 (recommended transmitter frontend configuration) and
 * Table 6-93 (recommended PHY receiver and digital frontend configuration).
 * The transmitter settings are the same in both bands; the receiver bandwidth
 * and relative cut-off differ. Table 6-93 lists two 2.4 GHz variants; the
 * entries here are the ones for reception without low frequency offset, which
 * is the operating point the driver commits to.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_phy.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* Energy detect duration recommended for OFDM in both bands. */
#define OFDM_EDD_US (960u)

/* BBCn_OFDMSW.RXO, receiver override. */
#define OFDMSW_RXO (0x10u)

/******************************************************************************/
/** Data **********************************************************************/

static const struct rf215_frontend_cfg ofdm_cfg[2][4] = {
    [RF215_BAND_09] = {
        /* Option 1 */
        { .tx.sr = RF215_SR_1333, .tx.rcut = RF215_RCUT_1000,
          .tx.lpfcut = RF215_LPF_800, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_1333, .rx.rcut = RF215_RCUT_1000,
          .rx.bw = RF215_BW_1250_2000, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 2 */
        { .tx.sr = RF215_SR_1333, .tx.rcut = RF215_RCUT_0750,
          .tx.lpfcut = RF215_LPF_500, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_1333, .rx.rcut = RF215_RCUT_0500,
          .rx.bw = RF215_BW_800_1000, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 3 */
        { .tx.sr = RF215_SR_666, .tx.rcut = RF215_RCUT_0750,
          .tx.lpfcut = RF215_LPF_250, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_666, .rx.rcut = RF215_RCUT_0500,
          .rx.bw = RF215_BW_400_500, .rx.if_shift = false,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 4 */
        { .tx.sr = RF215_SR_666, .tx.rcut = RF215_RCUT_0500,
          .tx.lpfcut = RF215_LPF_160, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_666, .rx.rcut = RF215_RCUT_0375,
          .rx.bw = RF215_BW_250_250, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
    },
    [RF215_BAND_24] = {
        /* Option 1 */
        { .tx.sr = RF215_SR_1333, .tx.rcut = RF215_RCUT_1000,
          .tx.lpfcut = RF215_LPF_800, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_1333, .rx.rcut = RF215_RCUT_1000,
          .rx.bw = RF215_BW_1600_2000, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 2 */
        { .tx.sr = RF215_SR_1333, .tx.rcut = RF215_RCUT_0750,
          .tx.lpfcut = RF215_LPF_500, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_1333, .rx.rcut = RF215_RCUT_0500,
          .rx.bw = RF215_BW_800_1000, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 3 */
        { .tx.sr = RF215_SR_666, .tx.rcut = RF215_RCUT_0750,
          .tx.lpfcut = RF215_LPF_250, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_666, .rx.rcut = RF215_RCUT_0750,
          .rx.bw = RF215_BW_500_500, .rx.if_shift = true,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
        /* Option 4 */
        { .tx.sr = RF215_SR_666, .tx.rcut = RF215_RCUT_0500,
          .tx.lpfcut = RF215_LPF_160, .tx.paramp = RF215_PARAMP_4,
          .rx.sr = RF215_SR_666, .rx.rcut = RF215_RCUT_0375,
          .rx.bw = RF215_BW_320_500, .rx.if_shift = false,
          .agc.enabled = true, .agc.avg = RF215_AVGS_8, .edd_us = OFDM_EDD_US, },
    },
};

/* BBCn_OFDMSW.PDT, the recommended preamble detection threshold per option. */
static const uint8_t ofdm_pdt[4] = {
    5u,
    5u,
    4u,
    3u,
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_frontend_cfg* rf215_phy_ofdm_frontend(enum rf215_band band, uint8_t opt) {
    return &ofdm_cfg[(band == RF215_BAND_24) ? RF215_BAND_24 : RF215_BAND_09][opt & 0x03u];
}

uint8_t rf215_phy_ofdm_switches(uint8_t opt) {
    return (uint8_t)((uint8_t)(ofdm_pdt[opt & 0x03u] << 5) | OFDMSW_RXO);
}
