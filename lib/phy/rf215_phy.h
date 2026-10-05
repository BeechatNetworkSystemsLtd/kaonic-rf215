/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Per-modulation frontend configuration.
 *
 * Each modulation contributes a table of recommended frontend settings, taken
 * from the datasheet's recommended-configuration tables. The tables are data
 * only: composing them into register values is the configure operation's
 * job, so adding a modulation means adding a table, not another code path.
 */

#ifndef KAONIC_DRIVERS_RF215_PHY_H__
#define KAONIC_DRIVERS_RF215_PHY_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215/rf215_regs.h"
#include "rf215/rf215_types.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* Sample rates, cut-offs, ramp times and bandwidths, as the datasheet encodes
 * them. Shared by every modulation table. */
#define RF215_SR_4000 0x01u
#define RF215_SR_2000 0x02u
#define RF215_SR_1333 0x03u
#define RF215_SR_1000 0x04u
#define RF215_SR_800 0x05u
#define RF215_SR_666 0x06u
#define RF215_SR_500 0x08u
#define RF215_SR_400 0x0Au

#define RF215_RCUT_0250 0x00u
#define RF215_RCUT_0375 0x01u
#define RF215_RCUT_0500 0x02u
#define RF215_RCUT_0750 0x03u
#define RF215_RCUT_1000 0x04u

#define RF215_LPF_80 0x00u
#define RF215_LPF_100 0x01u
#define RF215_LPF_125 0x02u
#define RF215_LPF_160 0x03u
#define RF215_LPF_200 0x04u
#define RF215_LPF_250 0x05u
#define RF215_LPF_315 0x06u
#define RF215_LPF_400 0x07u
#define RF215_LPF_500 0x08u
#define RF215_LPF_625 0x09u
#define RF215_LPF_800 0x0Au
#define RF215_LPF_1000 0x0Bu

#define RF215_PARAMP_4 0x00u
#define RF215_PARAMP_8 0x01u
#define RF215_PARAMP_16 0x02u
#define RF215_PARAMP_32 0x03u

#define RF215_BW_160_250 0x00u
#define RF215_BW_200_250 0x01u
#define RF215_BW_250_250 0x02u
#define RF215_BW_320_500 0x03u
#define RF215_BW_400_500 0x04u
#define RF215_BW_500_500 0x05u
#define RF215_BW_630_1000 0x06u
#define RF215_BW_800_1000 0x07u
#define RF215_BW_1000_1000 0x08u
#define RF215_BW_1250_2000 0x09u
#define RF215_BW_1600_2000 0x0Au
#define RF215_BW_2000_2000 0x0Bu

#define RF215_AVGS_8 0x00u
#define RF215_AVGS_16 0x01u
#define RF215_AVGS_32 0x02u
#define RF215_AVGS_64 0x03u

/******************************************************************************/
/** Types *********************************************************************/

/*
 * One frontend configuration: transmitter, receiver and gain control, each
 * with fields named after the datasheet register fields rather than the
 * tables, so that composing them reads directly.
 */
/* Transmitter front end. */
struct rf215_radio_tx_cfg {
    uint8_t sr;      /* TXDFE.SR */
    uint8_t rcut;    /* TXDFE.RCUT */
    bool direct_mod; /* TXDFE.DM */
    uint8_t lpfcut;  /* TXCUTC.LPFCUT */
    uint8_t paramp;  /* TXCUTC.PARAMP */
};

/* Receiver front end. */
struct rf215_radio_rx_cfg {
    uint8_t sr;    /* RXDFE.SR */
    uint8_t rcut;  /* RXDFE.RCUT */
    uint8_t bw;    /* RXBWC.BW */
    bool if_shift; /* RXBWC.IFS */
};

/* Automatic gain control. */
struct rf215_agc_cfg {
    bool enabled; /* AGCC.EN */
    bool input;   /* AGCC.AGCI */
    uint8_t avg;  /* AGCC.AVGS */
};

struct rf215_frontend_cfg {
    struct rf215_radio_tx_cfg tx;
    struct rf215_radio_rx_cfg rx;
    struct rf215_agc_cfg agc;
    uint16_t edd_us; /* Energy detect duration, RFn_EDD */
};

/******************************************************************************/
/** MR-OFDM *******************************************************************/

/**
 * @brief Recommended frontend for a bandwidth option in a band.
 *
 * @param band  Which transceiver.
 * @param opt   Bandwidth option, 0 to 3 (option 1 to option 4).
 */
const struct rf215_frontend_cfg* rf215_phy_ofdm_frontend(enum rf215_band band, uint8_t opt);

/**
 * @brief Value for BBCn_OFDMSW: the recommended preamble detection threshold
 *        for the option, with the receiver override enabled.
 */
uint8_t rf215_phy_ofdm_switches(uint8_t opt);

/******************************************************************************/
/** MR-O-QPSK *****************************************************************/

/**
 * @brief Recommended frontend for a chip rate. Both bands share it.
 *
 * @param fchip  Chip rate, 0 to 3 (100, 200, 1000, 2000 kchip/s).
 */
const struct rf215_frontend_cfg* rf215_phy_qpsk_frontend(uint8_t fchip);

/**
 * MR-FSK frontend for a band and symbol rate (0..5 = 50..400 ksym/s). The
 * datasheet gives two receiver configurations, for modulation index 1/2 and
 * for index 1; @p high_index selects the latter, for any index above 1/2.
 */
const struct rf215_frontend_cfg*
rf215_phy_fsk_frontend(enum rf215_band band, uint8_t srate, bool high_index);

/** Pre-emphasis filter for direct modulation at this symbol rate (FSKPE0..2). */
void rf215_phy_fsk_preemphasis(uint8_t srate, uint8_t out[3]);

/**
 * @brief Whether direct modulation is required at this chip rate. It is at
 *        100 and 200 kchip/s, and ignored above.
 */
bool rf215_phy_qpsk_direct_mod(uint8_t fchip);

#endif /* KAONIC_DRIVERS_RF215_PHY_H__ */
