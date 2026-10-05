/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * MR-FSK frontend tables.
 *
 * Datasheet Table 6-51 fixes the digital frontend sample rates per symbol
 * rate (the transmitter values are those for chip version 3, which the
 * configure operation pairs with direct modulation). Tables 6-53 and 6-54
 * give the transmitter filter and ramp for modulation index 1/2 and 1;
 * Tables 6-60 to 6-63 give the receiver bandwidth, IF shift and cut-off for
 * the sub-GHz and 2.4 GHz bands at the same two indices. The index 1 set is
 * used for any index above 1/2. The energy detect duration is eight symbols,
 * floored at 128 us. Table 6-57 lists the pre-emphasis coefficients.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_phy.h"

/******************************************************************************/
/** Data **********************************************************************/

static const struct rf215_frontend_cfg fsk_cfg[2][2][6] = {
    [RF215_BAND_09] = {
        /* modulation index 1/2 */
        {
            /* 50 ksym/s */
            {
                .tx.sr = RF215_SR_500,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_80,
                .tx.paramp = RF215_PARAMP_32,
                .rx.sr = RF215_SR_400,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_160_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 160u,
            },
            /* 100 ksym/s */
            {
                .tx.sr = RF215_SR_1000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_100,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_800,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_200_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 150 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_160,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_320_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 200 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_200,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_320_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 300 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_315,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_500_500,
                .rx.if_shift = true,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 400 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_400,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_630_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
        },
        /* modulation index 1 */
        {
            /* 50 ksym/s */
            {
                .tx.sr = RF215_SR_500,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_80,
                .tx.paramp = RF215_PARAMP_32,
                .rx.sr = RF215_SR_400,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_160_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 160u,
            },
            /* 100 ksym/s */
            {
                .tx.sr = RF215_SR_1000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_160,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_800,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_320_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 150 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_250,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_400_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 200 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_315,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0500,
                .rx.bw = RF215_BW_500_500,
                .rx.if_shift = true,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 300 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_500,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_630_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 400 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_625,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_1000_1000,
                .rx.if_shift = true,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
        },
    },
    [RF215_BAND_24] = {
        /* modulation index 1/2 */
        {
            /* 50 ksym/s */
            {
                .tx.sr = RF215_SR_500,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_80,
                .tx.paramp = RF215_PARAMP_32,
                .rx.sr = RF215_SR_400,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_160_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 160u,
            },
            /* 100 ksym/s */
            {
                .tx.sr = RF215_SR_1000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_100,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_800,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_200_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 150 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_160,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_320_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 200 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_200,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_400_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 300 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_315,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0250,
                .rx.bw = RF215_BW_630_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 400 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_0250,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_400,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_800_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
        },
        /* modulation index 1 */
        {
            /* 50 ksym/s */
            {
                .tx.sr = RF215_SR_500,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_80,
                .tx.paramp = RF215_PARAMP_32,
                .rx.sr = RF215_SR_400,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_200_250,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 160u,
            },
            /* 100 ksym/s */
            {
                .tx.sr = RF215_SR_1000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_160,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_800,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_400_500,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 150 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_250,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0500,
                .rx.bw = RF215_BW_630_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 200 ksym/s */
            {
                .tx.sr = RF215_SR_2000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_315,
                .tx.paramp = RF215_PARAMP_16,
                .rx.sr = RF215_SR_1000,
                .rx.rcut = RF215_RCUT_0750,
                .rx.bw = RF215_BW_630_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 300 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_500,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0375,
                .rx.bw = RF215_BW_800_1000,
                .rx.if_shift = false,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
            /* 400 ksym/s */
            {
                .tx.sr = RF215_SR_4000,
                .tx.rcut = RF215_RCUT_1000,
                .tx.direct_mod = false,
                .tx.lpfcut = RF215_LPF_625,
                .tx.paramp = RF215_PARAMP_8,
                .rx.sr = RF215_SR_2000,
                .rx.rcut = RF215_RCUT_0500,
                .rx.bw = RF215_BW_1000_1000,
                .rx.if_shift = true,
                .agc.enabled = true,
                .agc.input = false,
                .agc.avg = RF215_AVGS_8,
                .edd_us = 128u,
            },
        },
    },
};

static const uint8_t fsk_preemphasis[6][3] = {
    {
        0x02u,
        0x03u,
        0xFCu,
    },
    {
        0x0Eu,
        0x0Fu,
        0xF0u,
    },
    {
        0x3Cu,
        0x3Fu,
        0xC0u,
    },
    {
        0x74u,
        0x7Fu,
        0x80u,
    },
    {
        0x05u,
        0x3Cu,
        0xC3u,
    },
    {
        0x13u,
        0x29u,
        0xC7u,
    },
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_frontend_cfg*
rf215_phy_fsk_frontend(enum rf215_band band, uint8_t srate, bool high_index) {
    return &fsk_cfg[(band == RF215_BAND_24) ? RF215_BAND_24 : RF215_BAND_09][high_index ? 1u : 0u]
                   [srate < 6u ? srate : 5u];
}

void rf215_phy_fsk_preemphasis(uint8_t srate, uint8_t out[3]) {
    const uint8_t* pe = fsk_preemphasis[srate < 6u ? srate : 5u];

    out[0] = pe[0];
    out[1] = pe[1];
    out[2] = pe[2];
}
