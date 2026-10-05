/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215/rf215_regs.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Data **********************************************************************/

static const struct rf215_band_info bands[] = {
    [RF215_BAND_09] = {
        .radio_base = RF215_RG_RF09_BASE,
        .baseband_base = RF215_RG_BBC0_BASE,
        .frame_buffer_base = RF215_RG_BBC0_FRAME_BUFFER,
        .radio_irqs = RF215_RG_RF09_IRQS,
        .baseband_irqs = RF215_RG_BBC0_IRQS,
        .freq_offset_hz = 0u,
    },
    [RF215_BAND_24] = {
        .radio_base = RF215_RG_RF24_BASE,
        .baseband_base = RF215_RG_BBC1_BASE,
        .frame_buffer_base = RF215_RG_BBC1_FRAME_BUFFER,
        .radio_irqs = RF215_RG_RF24_IRQS,
        .baseband_irqs = RF215_RG_BBC1_IRQS,
        /* RFn_CCF0 counts from 1.5 GHz on the 2.4 GHz transceiver. */
        .freq_offset_hz = 1500000000u,
    },
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_band_info* rf215_band_info(enum rf215_band band) {
    return &bands[(band == RF215_BAND_24) ? RF215_BAND_24 : RF215_BAND_09];
}
