/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#ifndef KAONIC_DRIVERS_RF215_REGS_DEF_H__
#define KAONIC_DRIVERS_RF215_REGS_DEF_H__

/*****************************************************************************/

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

#define RF215_RG_RF09_IRQM 0x0100
#define RF215_RG_RF09_AUXS 0x0101
#define RF215_RG_RF09_STATE 0x0102
#define RF215_RG_RF09_CMD 0x0103
#define RF215_RG_RF09_CS 0x0104
#define RF215_RG_RF09_CCF0L 0x0105
#define RF215_RG_RF09_CCF0H 0x0106
#define RF215_RG_RF09_CNL 0x0107
#define RF215_RG_RF09_CNM 0x0108
#define RF215_RG_RF09_RXBWC 0x0109
#define RF215_RG_RF09_RXDFE 0x010A
#define RF215_RG_RF09_AGCC 0x010B
#define RF215_RG_RF09_AGCS 0x010C
#define RF215_RG_RF09_RSSI 0x010D
#define RF215_RG_RF09_EDC 0x010E
#define RF215_RG_RF09_EDD 0x010F
#define RF215_RG_RF09_EDV 0x0110
#define RF215_RG_RF09_RNDV 0x0111
#define RF215_RG_RF09_TXCUTC 0x0112
#define RF215_RG_RF09_TXDFE 0x0113
#define RF215_RG_RF09_PAC 0x0114
#define RF215_RG_RF09_PADFE 0x0116
#define RF215_RG_RF09_PLL 0x0121
#define RF215_RG_RF09_PLLCF 0x0122
#define RF215_RG_RF09_TXCI 0x0125
#define RF215_RG_RF09_TXCQ 0x0126
#define RF215_RG_RF09_TXDACI 0x0127
#define RF215_RG_RF09_TXDACQ 0x0128

#define RF215_RG_RF24_IRQM 0x0200
#define RF215_RG_RF24_AUXS 0x0201
#define RF215_RG_RF24_STATE 0x0202
#define RF215_RG_RF24_CMD 0x0203
#define RF215_RG_RF24_CS 0x0204
#define RF215_RG_RF24_CCF0L 0x0205
#define RF215_RG_RF24_CCF0H 0x0206
#define RF215_RG_RF24_CNL 0x0207
#define RF215_RG_RF24_CNM 0x0208
#define RF215_RG_RF24_RXBWC 0x0209
#define RF215_RG_RF24_RXDFE 0x020A
#define RF215_RG_RF24_AGCC 0x020B
#define RF215_RG_RF24_AGCS 0x020C
#define RF215_RG_RF24_RSSI 0x020D
#define RF215_RG_RF24_EDC 0x020E
#define RF215_RG_RF24_EDD 0x020F
#define RF215_RG_RF24_EDV 0x0210
#define RF215_RG_RF24_RNDV 0x0211
#define RF215_RG_RF24_TXCUTC 0x0212
#define RF215_RG_RF24_TXDFE 0x0213
#define RF215_RG_RF24_PAC 0x0214
#define RF215_RG_RF24_PADFE 0x0216
#define RF215_RG_RF24_PLL 0x0221
#define RF215_RG_RF24_PLLCF 0x0222
#define RF215_RG_RF24_TXCI 0x0225
#define RF215_RG_RF24_TXCQ 0x0226
#define RF215_RG_RF24_TXDACI 0x0227
#define RF215_RG_RF24_TXDACQ 0x0228

#define RF215_RG_BBC0_IRQM 0x0300
#define RF215_RG_BBC0_PC 0x0301
#define RF215_RG_BBC0_PS 0x0302
#define RF215_RG_BBC0_RXFLL 0x0304
#define RF215_RG_BBC0_RXFLH 0x0305
#define RF215_RG_BBC0_TXFLL 0x0306
#define RF215_RG_BBC0_TXFLH 0x0307
#define RF215_RG_BBC0_FBLL 0x0308
#define RF215_RG_BBC0_FBLH 0x0309
#define RF215_RG_BBC0_FBLIL 0x030A
#define RF215_RG_BBC0_FBLIH 0x030B
#define RF215_RG_BBC0_OFDMPHRTX 0x030C
#define RF215_RG_BBC0_OFDMPHRRX 0x030D
#define RF215_RG_BBC0_OFDMC 0x030E
#define RF215_RG_BBC0_OFDMSW 0x030F
#define RF215_RG_BBC0_OQPSKC0 0x0310
#define RF215_RG_BBC0_OQPSKC1 0x0311
#define RF215_RG_BBC0_OQPSKC2 0x0312
#define RF215_RG_BBC0_OQPSKC3 0x0313
#define RF215_RG_BBC0_OQPSKPHRTX 0x0314
#define RF215_RG_BBC0_OQPSKPHRRX 0x0315
#define RF215_RG_BBC0_AFC0 0x0320
#define RF215_RG_BBC0_AFC1 0x0321
#define RF215_RG_BBC0_AFFTM 0x0322
#define RF215_RG_BBC0_AFFVM 0x0323
#define RF215_RG_BBC0_AFS 0x0324
#define RF215_RG_BBC0_MACEA0 0x0325
#define RF215_RG_BBC0_MACEA1 0x0326
#define RF215_RG_BBC0_MACEA2 0x0327
#define RF215_RG_BBC0_MACEA3 0x0328
#define RF215_RG_BBC0_MACEA4 0x0329
#define RF215_RG_BBC0_MACEA5 0x032A
#define RF215_RG_BBC0_MACEA6 0x032B
#define RF215_RG_BBC0_MACEA7 0x032C
#define RF215_RG_BBC0_MACPID0F0 0x032D
#define RF215_RG_BBC0_MACPID1F0 0x032E
#define RF215_RG_BBC0_MACSHA0F0 0x032F
#define RF215_RG_BBC0_MACSHA1F0 0x0330
#define RF215_RG_BBC0_MACPID0F1 0x0331
#define RF215_RG_BBC0_MACPID1F1 0x0332
#define RF215_RG_BBC0_MACSHA0F1 0x0333
#define RF215_RG_BBC0_MACSHA1F1 0x0334
#define RF215_RG_BBC0_MACPID0F2 0x0335
#define RF215_RG_BBC0_MACPID1F2 0x0336
#define RF215_RG_BBC0_MACSHA0F2 0x0337
#define RF215_RG_BBC0_MACSHA1F2 0x0338
#define RF215_RG_BBC0_MACPID0F3 0x0339
#define RF215_RG_BBC0_MACPID1F3 0x033A
#define RF215_RG_BBC0_MACSHA0F3 0x033B
#define RF215_RG_BBC0_MACSHA1F3 0x033C
#define RF215_RG_BBC0_AMCS 0x0340
#define RF215_RG_BBC0_AMEDT 0x0341
#define RF215_RG_BBC0_AMAACKPD 0x0342
#define RF215_RG_BBC0_AMAACKTL 0x0343
#define RF215_RG_BBC0_AMAACKTH 0x0344
#define RF215_RG_BBC0_FSKC0 0x0360
#define RF215_RG_BBC0_FSKC1 0x0361
#define RF215_RG_BBC0_FSKC2 0x0362
#define RF215_RG_BBC0_FSKC3 0x0363
#define RF215_RG_BBC0_FSKC4 0x0364
#define RF215_RG_BBC0_FSKPLL 0x0365
#define RF215_RG_BBC0_FSKSFD0L 0x0366
#define RF215_RG_BBC0_FSKSFD0H 0x0367
#define RF215_RG_BBC0_FSKSFD1L 0x0368
#define RF215_RG_BBC0_FSKSFD1H 0x0369
#define RF215_RG_BBC0_FSKPHRTX 0x036A
#define RF215_RG_BBC0_FSKPHRRX 0x036B
#define RF215_RG_BBC0_FSKRPC 0x036C
#define RF215_RG_BBC0_FSKRPCONT 0x036D
#define RF215_RG_BBC0_FSKRPCOFFT 0x036E
#define RF215_RG_BBC0_FSKRRXFLL 0x0370
#define RF215_RG_BBC0_FSKRRXFLH 0x0371
#define RF215_RG_BBC0_FSKDM 0x0372
#define RF215_RG_BBC0_FSKPE0 0x0373
#define RF215_RG_BBC0_FSKPE1 0x0374
#define RF215_RG_BBC0_FSKPE2 0x0375
#define RF215_RG_BBC0_PMUC 0x0380
#define RF215_RG_BBC0_PMUVAL 0x0381
#define RF215_RG_BBC0_PMUQF 0x0382
#define RF215_RG_BBC0_PMUI 0x0383
#define RF215_RG_BBC0_PMUQ 0x0384
#define RF215_RG_BBC0_CNTC 0x0390
#define RF215_RG_BBC0_CNT0 0x0391
#define RF215_RG_BBC0_CNT1 0x0392
#define RF215_RG_BBC0_CNT2 0x0393
#define RF215_RG_BBC0_CNT3 0x0394

#define RF215_RG_BBC1_IRQM 0x0400
#define RF215_RG_BBC1_PC 0x0401
#define RF215_RG_BBC1_PS 0x0402
#define RF215_RG_BBC1_RXFLL 0x0404
#define RF215_RG_BBC1_RXFLH 0x0405
#define RF215_RG_BBC1_TXFLL 0x0406
#define RF215_RG_BBC1_TXFLH 0x0407
#define RF215_RG_BBC1_FBLL 0x0408
#define RF215_RG_BBC1_FBLH 0x0409
#define RF215_RG_BBC1_FBLIL 0x040A
#define RF215_RG_BBC1_FBLIH 0x040B
#define RF215_RG_BBC1_OFDMPHRTX 0x040C
#define RF215_RG_BBC1_OFDMPHRRX 0x040D
#define RF215_RG_BBC1_OFDMC 0x040E
#define RF215_RG_BBC1_OFDMSW 0x040F
#define RF215_RG_BBC1_OQPSKC0 0x0410
#define RF215_RG_BBC1_OQPSKC1 0x0411
#define RF215_RG_BBC1_OQPSKC2 0x0412
#define RF215_RG_BBC1_OQPSKC3 0x0413
#define RF215_RG_BBC1_OQPSKPHRTX 0x0414
#define RF215_RG_BBC1_OQPSKPHRRX 0x0415
#define RF215_RG_BBC1_AFC0 0x0420
#define RF215_RG_BBC1_AFC1 0x0421
#define RF215_RG_BBC1_AFFTM 0x0422
#define RF215_RG_BBC1_AFFVM 0x0423
#define RF215_RG_BBC1_AFS 0x0424
#define RF215_RG_BBC1_MACEA0 0x0425
#define RF215_RG_BBC1_MACEA1 0x0426
#define RF215_RG_BBC1_MACEA2 0x0427
#define RF215_RG_BBC1_MACEA3 0x0428
#define RF215_RG_BBC1_MACEA4 0x0429
#define RF215_RG_BBC1_MACEA5 0x042A
#define RF215_RG_BBC1_MACEA6 0x042B
#define RF215_RG_BBC1_MACEA7 0x042C
#define RF215_RG_BBC1_MACPID0F0 0x042D
#define RF215_RG_BBC1_MACPID1F0 0x042E
#define RF215_RG_BBC1_MACSHA0F0 0x042F
#define RF215_RG_BBC1_MACSHA1F0 0x0430
#define RF215_RG_BBC1_MACPID0F1 0x0431
#define RF215_RG_BBC1_MACPID1F1 0x0432
#define RF215_RG_BBC1_MACSHA0F1 0x0433
#define RF215_RG_BBC1_MACSHA1F1 0x0434
#define RF215_RG_BBC1_MACPID0F2 0x0435
#define RF215_RG_BBC1_MACPID1F2 0x0436
#define RF215_RG_BBC1_MACSHA0F2 0x0437
#define RF215_RG_BBC1_MACSHA1F2 0x0438
#define RF215_RG_BBC1_MACPID0F3 0x0439
#define RF215_RG_BBC1_MACPID1F3 0x043A
#define RF215_RG_BBC1_MACSHA0F3 0x043B
#define RF215_RG_BBC1_MACSHA1F3 0x043C
#define RF215_RG_BBC1_AMCS 0x0440
#define RF215_RG_BBC1_AMEDT 0x0441
#define RF215_RG_BBC1_AMAACKPD 0x0442
#define RF215_RG_BBC1_AMAACKTL 0x0443
#define RF215_RG_BBC1_AMAACKTH 0x0444
#define RF215_RG_BBC1_FSKC0 0x0460
#define RF215_RG_BBC1_FSKC1 0x0461
#define RF215_RG_BBC1_FSKC2 0x0462
#define RF215_RG_BBC1_FSKC3 0x0463
#define RF215_RG_BBC1_FSKC4 0x0464
#define RF215_RG_BBC1_FSKPLL 0x0465
#define RF215_RG_BBC1_FSKSFD0L 0x0466
#define RF215_RG_BBC1_FSKSFD0H 0x0467
#define RF215_RG_BBC1_FSKSFD1L 0x0468
#define RF215_RG_BBC1_FSKSFD1H 0x0469
#define RF215_RG_BBC1_FSKPHRTX 0x046A
#define RF215_RG_BBC1_FSKPHRRX 0x046B
#define RF215_RG_BBC1_FSKRPC 0x046C
#define RF215_RG_BBC1_FSKRPCONT 0x046D
#define RF215_RG_BBC1_FSKRPCOFFT 0x046E
#define RF215_RG_BBC1_FSKRRXFLL 0x0470
#define RF215_RG_BBC1_FSKRRXFLH 0x0471
#define RF215_RG_BBC1_FSKDM 0x0472
#define RF215_RG_BBC1_FSKPE0 0x0473
#define RF215_RG_BBC1_FSKPE1 0x0474
#define RF215_RG_BBC1_FSKPE2 0x0475
#define RF215_RG_BBC1_PMUC 0x0480
#define RF215_RG_BBC1_PMUVAL 0x0481
#define RF215_RG_BBC1_PMUQF 0x0482
#define RF215_RG_BBC1_PMUI 0x0483
#define RF215_RG_BBC1_PMUQ 0x0484
#define RF215_RG_BBC1_CNTC 0x0490
#define RF215_RG_BBC1_CNT0 0x0491
#define RF215_RG_BBC1_CNT1 0x0492
#define RF215_RG_BBC1_CNT2 0x0493
#define RF215_RG_BBC1_CNT3 0x0494

#define RF215_RG_BBC0_FBRXS 0x2000
#define RF215_RG_BBC0_FBRXE 0x27FE
#define RF215_RG_BBC0_FBTXS 0x2800
#define RF215_RG_BBC0_FBTXE 0x2FFE

#define RF215_RG_BBC1_FBRXS 0x3000
#define RF215_RG_BBC1_FBRXE 0x37FE
#define RF215_RG_BBC1_FBTXS 0x3800
#define RF215_RG_BBC1_FBTXE 0x3FFE

/*****************************************************************************/

#endif // KAONIC_DRIVERS_RF215_REGS_DEF_H__
