/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef KAONIC_DRIVERS_RF215_REGS_H__
#define KAONIC_DRIVERS_RF215_REGS_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_types.h"

/******************************************************************************/
/** Defines *******************************************************************/

#define RF215_FLAG_WRITE (0x8000u)
#define RF215_FLAG_READ (0x0000u)

#define RF215_FREQ_RESOLUTION_KHZ (25u)

#define RF215_MASK_IRQS_TXFE (0x10u)
#define RF215_MASK_IRQS_TRXRDY (0x02u)
#define RF215_MASK_IRQS_RXFS (0x01u)
#define RF215_MASK_IRQS_RXFE (0x02u)

/**
 * RFn_IRQS – Radio IRQ Status
 */
#define RF215_RADIO_IRQ_WAKEUP (1u << 0u)
#define RF215_RADIO_IRQ_TRXRDY (1u << 1u)
#define RF215_RADIO_IRQ_EDC (1u << 2u)
#define RF215_RADIO_IRQ_BATLOW (1u << 3u)
#define RF215_RADIO_IRQ_TRXERR (1u << 4u)
#define RF215_RADIO_IRQ_IQIFSF (1u << 5u)
#define RF215_RADIO_IRQ_MASK (0x1Fu) /* Every radio IRQ except IQIFSF */
#define RF215_RADIO_IRQ_ALL (0x3Fu)  /* Every radio IRQ the chip defines */
/* The radio interrupts the driver claims. WAKEUP is polled during reset and
 * BATLOW read nowhere, so arming either only wakes the host for nothing. */
#define RF215_RADIO_IRQ_USED (RF215_RADIO_IRQ_TRXRDY | RF215_RADIO_IRQ_EDC | RF215_RADIO_IRQ_TRXERR)

/**
 * BBCn_IRQS – Baseband IRQ Status
 */
#define RF215_BASEBAND_IRQ_RXFS (1u << 0u)
#define RF215_BASEBAND_IRQ_RXFE (1u << 1u)
#define RF215_BASEBAND_IRQ_RXAM (1u << 2u)
#define RF215_BASEBAND_IRQ_RXEM (1u << 3u)
#define RF215_BASEBAND_IRQ_TXFE (1u << 4u)
#define RF215_BASEBAND_IRQ_AGCH (1u << 5u)
#define RF215_BASEBAND_IRQ_AGCR (1u << 6u)
#define RF215_BASEBAND_IRQ_FBLI (1u << 7u)
#define RF215_BASEBAND_IRQ_MASK (0x1Fu) /* RXFS..TXFE only; excludes AGCH, AGCR and FBLI */
#define RF215_BASEBAND_IRQ_ALL (0xFFu)  /* Every baseband IRQ, FBLI included */

/*
 * The ones the driver actually acts on.
 *
 * Enabling more is not free: AGCH fires at the start of every frame and AGCR at
 * the end of each one, and each unused interrupt is a wake-up that takes the
 * bus lock, on a part where both transceivers share it. Anything armed here
 * must be claimed somewhere, or it accumulates in the pending mask forever.
 *
 * AGCR is armed for one reason: it is the only word the chip gives that a
 * reception is over when the frame did not decode. A frame start has no frame
 * end to follow it then, and without the release the transmit path would hold
 * off for as long as a frame could possibly last.
 */
#define RF215_BASEBAND_IRQ_USED                                                  \
    (RF215_BASEBAND_IRQ_RXFS | RF215_BASEBAND_IRQ_RXFE | RF215_BASEBAND_IRQ_TXFE \
     | RF215_BASEBAND_IRQ_AGCR)

/* IRQ Configuration */
#define RF215_RG_RF_CFG_IRQMM (1u << 3u)
#define RF215_RG_RF_CFG_IRQP (1u << 2u)
#define RF215_RG_RF_CFG_DRV_MASK (0x03u)
/* RF_CFG.DRV values: how hard the chip drives MISO and the interrupt line. */
#define RF215_PAD_DRIVE_2MA (0u)
#define RF215_PAD_DRIVE_4MA (1u)
#define RF215_PAD_DRIVE_6MA (2u)
#define RF215_PAD_DRIVE_8MA (3u)
/* The chip's own reset value, and what the driver used before it was tunable. */
/* 8 mA, which is what the userspace driver ran the same boards at. The
 * second chip sits on a longer track; a weaker pad reads back marginally at
 * the clock the bus is run at. */
#define RF215_PAD_DRIVE_DEFAULT RF215_PAD_DRIVE_8MA

/* RFn_STATE carries the state in the low three bits. */
#define RF215_STATE_MASK (0x07u)

/******************************************************************************/
/** Types *********************************************************************/

enum rf215_pn {
    RF215_PN_UNKNOWN = 0x00,
    RF215_PN_AT86RF215 = 0x34,
    RF215_PN_AT86RF215IQ = 0x35,
    RF215_PN_AT86RF215M = 0x36,
};

enum rf215_cmd {
    RF215_CMD_RF_NOP = 0x00,
    RF215_CMD_RF_SLEEP = 0x01,
    RF215_CMD_RF_TRXOFF = 0x02,
    RF215_CMD_RF_TRXPREP = 0x03,
    RF215_CMD_RF_TX = 0x04,
    RF215_CMD_RF_RX = 0x05,
    RF215_CMD_RF_RESET = 0x07,
};

enum rf215_state {
    RF215_STATE_TRXOFF = 0x02,     /* Transceiver off, SPI active */
    RF215_STATE_TXPREP = 0x03,     /* Transmit preparation */
    RF215_STATE_TX = 0x04,         /* Transmit */
    RF215_STATE_RX = 0x05,         /* Receive */
    RF215_STATE_TRANSITION = 0x06, /* State transition in progress */
    RF215_STATE_RESET = 0x07,      /* Transceiver is in state RESET or SLEEP */
};

/* The two transceivers of the chip. The values index per-band tables. */
enum rf215_band {
    RF215_BAND_09 = 0, /* sub-GHz modem */
    RF215_BAND_24 = 1, /* 2.4GHz modem */
};

/**
 * What differs between the two transceivers: where their registers sit and
 * what their synthesizers can tune. Everything else about them is the same,
 * so the rest of the driver is written once against this description.
 */
struct rf215_band_info {
    rf215_reg_t radio_base;        /* RFn_* registers */
    rf215_reg_t baseband_base;     /* BBCn_* registers */
    rf215_reg_t frame_buffer_base; /* BBCn_FBRXS / BBCn_FBTXS */
    rf215_reg_t radio_irqs;        /* RFn_IRQS */
    rf215_reg_t baseband_irqs;     /* BBCn_IRQS */
    uint32_t freq_offset_hz;       /* What RFn_CCF0 counts from */
};

/******************************************************************************/
/** Public Functions **********************************************************/

/**
 * @brief Look up what is specific to one of the two transceivers.
 *
 * @param band  Which transceiver.
 *
 * @return Its register bases and tuning offset. Never NULL: a value that is
 *         not a band is answered with the sub-GHz description.
 */
const struct rf215_band_info* rf215_band_info(enum rf215_band band);

#endif /* KAONIC_DRIVERS_RF215_REGS_H__ */
