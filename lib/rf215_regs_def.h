/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * AT86RF215 register summary.
 *
 * The two transceivers have the same register layout at different addresses,
 * so a transceiver's registers are given once, as offsets, and placed by the
 * base addresses of its band; see struct rf215_band_info.
 */

#ifndef KAONIC_DRIVERS_RF215_REGS_DEF_H__
#define KAONIC_DRIVERS_RF215_REGS_DEF_H__

/******************************************************************************/
/** Address Space *************************************************************/

#define RF215_RG_RF09_BASE 0x0100
#define RF215_RG_RF24_BASE 0x0200
#define RF215_RG_BBC0_BASE 0x0300
#define RF215_RG_BBC1_BASE 0x0400
#define RF215_RG_BBC0_FRAME_BUFFER 0x2000
#define RF215_RG_BBC1_FRAME_BUFFER 0x3000

/******************************************************************************/
/** Interrupt Status and Common Registers (absolute) **************************/

#define RF215_RG_RF09_IRQS 0x0000
#define RF215_RG_RF24_IRQS 0x0001
#define RF215_RG_BBC0_IRQS 0x0002
#define RF215_RG_BBC1_IRQS 0x0003
#define RF215_RG_RF_RST 0x0005
#define RF215_RG_RF_CFG 0x0006
#define RF215_RG_RF_CLKO 0x0007
#define RF215_RG_RF_BMDVC 0x0008
#define RF215_RG_RF_XOC 0x0009
#define RF215_RG_RF_IQIFC0 0x000A
#define RF215_RG_RF_IQIFC1 0x000B
#define RF215_RG_RF_IQIFC2 0x000C
#define RF215_RG_RF_PN 0x000D
#define RF215_RG_RF_VN 0x000E

/******************************************************************************/
/** Radio Registers (offset from the radio base) ******************************/

#define RF215_RG_RFXX_IRQM 0x000
#define RF215_RG_RFXX_AUXS 0x001
#define RF215_RG_RFXX_STATE 0x002
#define RF215_RG_RFXX_CMD 0x003
#define RF215_RG_RFXX_CS 0x004
#define RF215_RG_RFXX_CCF0L 0x005
#define RF215_RG_RFXX_CCF0H 0x006
#define RF215_RG_RFXX_CNL 0x007
#define RF215_RG_RFXX_CNM 0x008
#define RF215_RG_RFXX_RXBWC 0x009
#define RF215_RG_RFXX_RXDFE 0x00A
#define RF215_RG_RFXX_AGCC 0x00B
#define RF215_RG_RFXX_AGCS 0x00C
#define RF215_RG_RFXX_RSSI 0x00D
#define RF215_RG_RFXX_EDC 0x00E
#define RF215_RG_RFXX_EDD 0x00F
#define RF215_RG_RFXX_EDV 0x010
#define RF215_RG_RFXX_RNDV 0x011
#define RF215_RG_RFXX_TXCUTC 0x012
#define RF215_RG_RFXX_TXDFE 0x013
#define RF215_RG_RFXX_PAC 0x014
#define RF215_RG_RFXX_PADFE 0x016
#define RF215_RG_RFXX_PLL 0x021
#define RF215_RG_RFXX_PLLCF 0x022
#define RF215_RG_RFXX_TXCI 0x025
#define RF215_RG_RFXX_TXCQ 0x026
#define RF215_RG_RFXX_TXDACI 0x027
#define RF215_RG_RFXX_TXDACQ 0x028

/******************************************************************************/
/** Baseband Registers (offset from the baseband base) ************************/

#define RF215_RG_BBCX_IRQM 0x000
#define RF215_RG_BBCX_PC 0x001
#define RF215_RG_BBCX_PS 0x002
#define RF215_RG_BBCX_RXFLL 0x004
#define RF215_RG_BBCX_RXFLH 0x005
#define RF215_RG_BBCX_TXFLL 0x006
#define RF215_RG_BBCX_TXFLH 0x007
#define RF215_RG_BBCX_FBLL 0x008
#define RF215_RG_BBCX_FBLH 0x009
#define RF215_RG_BBCX_FBLIL 0x00A
#define RF215_RG_BBCX_FBLIH 0x00B
#define RF215_RG_BBCX_OFDMPHRTX 0x00C
#define RF215_RG_BBCX_OFDMPHRRX 0x00D
#define RF215_RG_BBCX_OFDMC 0x00E
#define RF215_RG_BBCX_OFDMSW 0x00F
#define RF215_RG_BBCX_OQPSKC0 0x010
#define RF215_RG_BBCX_OQPSKC1 0x011
#define RF215_RG_BBCX_OQPSKC2 0x012
#define RF215_RG_BBCX_OQPSKC3 0x013
#define RF215_RG_BBCX_OQPSKPHRTX 0x014
#define RF215_RG_BBCX_OQPSKPHRRX 0x015
#define RF215_RG_BBCX_AFC0 0x020
#define RF215_RG_BBCX_AFC1 0x021
#define RF215_RG_BBCX_AFFTM 0x022
#define RF215_RG_BBCX_AFFVM 0x023
#define RF215_RG_BBCX_AFS 0x024
#define RF215_RG_BBCX_MACEA0 0x025
#define RF215_RG_BBCX_MACEA1 0x026
#define RF215_RG_BBCX_MACEA2 0x027
#define RF215_RG_BBCX_MACEA3 0x028
#define RF215_RG_BBCX_MACEA4 0x029
#define RF215_RG_BBCX_MACEA5 0x02A
#define RF215_RG_BBCX_MACEA6 0x02B
#define RF215_RG_BBCX_MACEA7 0x02C
#define RF215_RG_BBCX_MACPID0F0 0x02D
#define RF215_RG_BBCX_MACPID1F0 0x02E
#define RF215_RG_BBCX_MACSHA0F0 0x02F
#define RF215_RG_BBCX_MACSHA1F0 0x030
#define RF215_RG_BBCX_MACPID0F1 0x031
#define RF215_RG_BBCX_MACPID1F1 0x032
#define RF215_RG_BBCX_MACSHA0F1 0x033
#define RF215_RG_BBCX_MACSHA1F1 0x034
#define RF215_RG_BBCX_MACPID0F2 0x035
#define RF215_RG_BBCX_MACPID1F2 0x036
#define RF215_RG_BBCX_MACSHA0F2 0x037
#define RF215_RG_BBCX_MACSHA1F2 0x038
#define RF215_RG_BBCX_MACPID0F3 0x039
#define RF215_RG_BBCX_MACPID1F3 0x03A
#define RF215_RG_BBCX_MACSHA0F3 0x03B
#define RF215_RG_BBCX_MACSHA1F3 0x03C
#define RF215_RG_BBCX_AMCS 0x040
#define RF215_RG_BBCX_AMEDT 0x041
#define RF215_RG_BBCX_AMAACKPD 0x042
#define RF215_RG_BBCX_AMAACKTL 0x043
#define RF215_RG_BBCX_AMAACKTH 0x044
#define RF215_RG_BBCX_FSKC0 0x060
#define RF215_RG_BBCX_FSKC1 0x061
#define RF215_RG_BBCX_FSKC2 0x062
#define RF215_RG_BBCX_FSKC3 0x063
#define RF215_RG_BBCX_FSKC4 0x064
#define RF215_RG_BBCX_FSKPLL 0x065
#define RF215_RG_BBCX_FSKSFD0L 0x066
#define RF215_RG_BBCX_FSKSFD0H 0x067
#define RF215_RG_BBCX_FSKSFD1L 0x068
#define RF215_RG_BBCX_FSKSFD1H 0x069
#define RF215_RG_BBCX_FSKPHRTX 0x06A
#define RF215_RG_BBCX_FSKPHRRX 0x06B
#define RF215_RG_BBCX_FSKRPC 0x06C
#define RF215_RG_BBCX_FSKRPCONT 0x06D
#define RF215_RG_BBCX_FSKRPCOFFT 0x06E
#define RF215_RG_BBCX_FSKRRXFLL 0x070
#define RF215_RG_BBCX_FSKRRXFLH 0x071
#define RF215_RG_BBCX_FSKDM 0x072
#define RF215_RG_BBCX_FSKPE0 0x073
#define RF215_RG_BBCX_FSKPE1 0x074
#define RF215_RG_BBCX_FSKPE2 0x075
#define RF215_RG_BBCX_PMUC 0x080
#define RF215_RG_BBCX_PMUVAL 0x081
#define RF215_RG_BBCX_PMUQF 0x082
#define RF215_RG_BBCX_PMUI 0x083
#define RF215_RG_BBCX_PMUQ 0x084
#define RF215_RG_BBCX_CNTC 0x090
#define RF215_RG_BBCX_CNT0 0x091
#define RF215_RG_BBCX_CNT1 0x092
#define RF215_RG_BBCX_CNT2 0x093
#define RF215_RG_BBCX_CNT3 0x094

/******************************************************************************/
/** Frame Buffers (offset from the frame buffer base) *************************/

#define RF215_RG_BBCX_FBRXS 0x000
#define RF215_RG_BBCX_FBRXE 0x7FE
#define RF215_RG_BBCX_FBTXS 0x800
#define RF215_RG_BBCX_FBTXE 0xFFE

#endif /* KAONIC_DRIVERS_RF215_REGS_DEF_H__ */
