/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#ifndef KAONIC_DRIVERS_RF215_REGS_H__
#define KAONIC_DRIVERS_RF215_REGS_H__

/*****************************************************************************/

#include <stddef.h>
#include <stdint.h>

/*****************************************************************************/

typedef uint16_t rf215_reg_t;
typedef uint8_t rf215_irq_t;
typedef uint32_t rf215_irq_data_t;

/*****************************************************************************/

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
#define RF215_RADIO_IRQ_MASK (0x1Fu)

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
#define RF215_BASEBAND_IRQ_MASK (0x1Fu)

// IRQ Configuration
#define RF215_RG_RF_CFG_IRQMM (1u << 3u)
#define RF215_RG_RF_CFG_IRQP (1u << 2u)
#define RF215_RG_RF_CFG_DRV_MASK (0x07u)

/*****************************************************************************/

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

struct rf215_common_regs {
    rf215_reg_t RG_RF_RST;
    rf215_reg_t RG_RF_CFG;
    rf215_reg_t RG_RF_CLKO;
    rf215_reg_t RG_RF_BMDVC;
    rf215_reg_t RG_RF_XOC;
    rf215_reg_t RG_RF_IQIFC0;
    rf215_reg_t RG_RF_IQIFC1;
    rf215_reg_t RG_RF_IQIFC2;
    rf215_reg_t RG_RF_PN;
    rf215_reg_t RG_RF_VN;
};

struct rf215_radio_regs {
    rf215_reg_t RG_IRQS;
    rf215_reg_t RG_AUXS;
    rf215_reg_t RG_STATE;
    rf215_reg_t RG_CMD;
    rf215_reg_t RG_CS;
    rf215_reg_t RG_CCF0L;
    rf215_reg_t RG_CCF0H;
    rf215_reg_t RG_CNL;
    rf215_reg_t RG_CNM;
    rf215_reg_t RG_RXBWC;
    rf215_reg_t RG_RXDFE;
    rf215_reg_t RG_AGCC;
    rf215_reg_t RG_AGCS;
    rf215_reg_t RG_RSSI;
    rf215_reg_t RG_EDC;
    rf215_reg_t RG_EDD;
    rf215_reg_t RG_EDV;
    rf215_reg_t RG_RNDV;
    rf215_reg_t RG_TXCUTC;
    rf215_reg_t RG_TXDFE;
    rf215_reg_t RG_PAC;
    rf215_reg_t RG_IRQM;
    rf215_reg_t RG_PADFE;
    rf215_reg_t RG_PLL;
    rf215_reg_t RG_PLLCF;
    rf215_reg_t RG_TXCI;
    rf215_reg_t RG_TXCQ;
    rf215_reg_t RG_TXDACI;
    rf215_reg_t RG_TXDACQ;
};

struct rf215_baseband_regs {
    rf215_reg_t RG_IRQS;
    rf215_reg_t RG_FBRXS;
    rf215_reg_t RG_FBRXE;
    rf215_reg_t RG_FBTXS;
    rf215_reg_t RG_FBTXE;
    rf215_reg_t RG_IRQM;
    rf215_reg_t RG_PC;
    rf215_reg_t RG_PS;
    rf215_reg_t RG_RXFLL;
    rf215_reg_t RG_RXFLH;
    rf215_reg_t RG_TXFLL;
    rf215_reg_t RG_TXFLH;
    rf215_reg_t RG_FBLL;
    rf215_reg_t RG_FBLH;
    rf215_reg_t RG_FBLIL;
    rf215_reg_t RG_FBLIH;
    rf215_reg_t RG_OFDMPHRTX;
    rf215_reg_t RG_OFDMPHRRX;
    rf215_reg_t RG_OFDMC;
    rf215_reg_t RG_OFDMSW;
    rf215_reg_t RG_OQPSKC0;
    rf215_reg_t RG_OQPSKC1;
    rf215_reg_t RG_OQPSKC2;
    rf215_reg_t RG_OQPSKC3;
    rf215_reg_t RG_OQPSKPHRTX;
    rf215_reg_t RG_OQPSKPHRRX;
    rf215_reg_t RG_AFC0;
    rf215_reg_t RG_AFC1;
    rf215_reg_t RG_AFFTM;
    rf215_reg_t RG_AFFVM;
    rf215_reg_t RG_AFS;
    rf215_reg_t RG_MACEA0;
    rf215_reg_t RG_MACEA1;
    rf215_reg_t RG_MACEA2;
    rf215_reg_t RG_MACEA3;
    rf215_reg_t RG_MACEA4;
    rf215_reg_t RG_MACEA5;
    rf215_reg_t RG_MACEA6;
    rf215_reg_t RG_MACEA7;
    rf215_reg_t RG_MACPID0F0;
    rf215_reg_t RG_MACPID1F0;
    rf215_reg_t RG_MACSHA0F0;
    rf215_reg_t RG_MACSHA1F0;
    rf215_reg_t RG_MACPID0F1;
    rf215_reg_t RG_MACPID1F1;
    rf215_reg_t RG_MACSHA0F1;
    rf215_reg_t RG_MACSHA1F1;
    rf215_reg_t RG_MACPID0F2;
    rf215_reg_t RG_MACPID1F2;
    rf215_reg_t RG_MACSHA0F2;
    rf215_reg_t RG_MACSHA1F2;
    rf215_reg_t RG_MACPID0F3;
    rf215_reg_t RG_MACPID1F3;
    rf215_reg_t RG_MACSHA0F3;
    rf215_reg_t RG_MACSHA1F3;
    rf215_reg_t RG_AMCS;
    rf215_reg_t RG_AMEDT;
    rf215_reg_t RG_AMAACKPD;
    rf215_reg_t RG_AMAACKTL;
    rf215_reg_t RG_AMAACKTH;
    rf215_reg_t RG_FSKC0;
    rf215_reg_t RG_FSKC1;
    rf215_reg_t RG_FSKC2;
    rf215_reg_t RG_FSKC3;
    rf215_reg_t RG_FSKC4;
    rf215_reg_t RG_FSKPLL;
    rf215_reg_t RG_FSKSFD0L;
    rf215_reg_t RG_FSKSFD0H;
    rf215_reg_t RG_FSKSFD1L;
    rf215_reg_t RG_FSKSFD1H;
    rf215_reg_t RG_FSKPHRTX;
    rf215_reg_t RG_FSKPHRRX;
    rf215_reg_t RG_FSKRPC;
    rf215_reg_t RG_FSKRPCONT;
    rf215_reg_t RG_FSKRPCOFFT;
    rf215_reg_t RG_FSKRRXFLL;
    rf215_reg_t RG_FSKRRXFLH;
    rf215_reg_t RG_FSKDM;
    rf215_reg_t RG_FSKPE0;
    rf215_reg_t RG_FSKPE1;
    rf215_reg_t RG_FSKPE2;
    rf215_reg_t RG_PMUC;
    rf215_reg_t RG_PMUVAL;
    rf215_reg_t RG_PMUQF;
    rf215_reg_t RG_PMUI;
    rf215_reg_t RG_PMUQ;
    rf215_reg_t RG_CNTC;
    rf215_reg_t RG_CNT0;
    rf215_reg_t RG_CNT1;
    rf215_reg_t RG_CNT2;
    rf215_reg_t RG_CNT3;
};

enum rf215_trx_type {
    RF215_TRX_TYPE_RF09, /* sub-GHz modem */
    RF215_TRX_TYPE_RF24, /* 2.4GHz modem */
};

struct rf215_reg_value {
    rf215_reg_t reg;
    uint8_t value;
};

struct rf215_reg_set {
    const struct rf215_reg_value* values;
    size_t len;
};

/*****************************************************************************/

const struct rf215_common_regs* rf215_regs_for_common(void);
const struct rf215_radio_regs* rf215_regs_for_radio(enum rf215_trx_type type);
const struct rf215_baseband_regs* rf215_regs_for_baseband(enum rf215_trx_type type);

/*****************************************************************************/

#endif // KAONIC_DRIVERS_RF215_REGS_H__
