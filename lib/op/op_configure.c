/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 *
 * Applies a PHY configuration: frequency, radio frontend, AGC and baseband.
 *
 * The recommended settings per modulation live in lib/phy/; this file only
 * composes them into register values and writes them out. Every write is a
 * plain write rather than a read-modify-write, which is safe because a
 * configuration always follows a reset and the bits this would otherwise
 * preserve are at their reset defaults.
 *
 * The sequence follows the userspace driver that ran the same boards: off
 * air, baseband off, frequency and frontend, PHY type and modulation,
 * baseband on, and back to receive through TXPREP with the PLL confirmed.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "phy/rf215_phy.h"
#include "rf215_core.h"
#include "rf215_op.h"
#include "rf215_regs_def.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* RFn_CS and RFn_CCF0 count in 25 kHz steps. */
#define FREQ_RESOLUTION_HZ (25000u)

/*
 * Time allowed to reach TRXOFF per attempt, and the interval between state
 * reads. A backstop for a command the chip dropped (errata #2/#6); the
 * command is re-issued once per deadline, not on every read.
 */
#define TRXOFF_TIMEOUT_US (100000u)

#define TRXOFF_POLL_US (200u)

#define TRXOFF_ATTEMPTS (3u)

/* Synthesizer tuning ranges (section 6.3.1). Outside them the registers are
 * accepted but the PLL never locks. */
#define BAND_09_LOW_MIN_HZ (389500000u)

#define BAND_09_LOW_MAX_HZ (510000000u)

#define BAND_09_HIGH_MIN_HZ (779000000u)

#define BAND_09_HIGH_MAX_HZ (1020000000u)

#define BAND_24_MIN_HZ (2400000000u)

#define BAND_24_MAX_HZ (2483500000u)

/* AGC gain control word, the maximum. */
#define AGCS_GCW (23u)

/* BBCn_PC fields. */
#define PC_PT_FSK (0x01u)

#define PC_PT_OFDM (0x02u)

#define PC_PT_QPSK (0x03u)

#define PC_FCS (0x08u | 0x10u | 0x40u) /* 16-bit, appended and checked */

/* RFn_PADFE front-end pad modes; unset, the external LNA/PA switch is never
 * driven. Sub-GHz: mode 2 (enable pin, TX/RX switch pin). 2.4 GHz: mode 3
 * (TX/RX switch, LNA bypass, enable from the MCU). */
#define PADFE_MODE_09 (0x02u << 6)

#define PADFE_MODE_24 (0x03u << 6)

/* RFn_AUXS: power amplifier supply at 2400 mV; the AGC gain map and external
 * LNA bypass come from the configuration. */
#define AUXS_PAVOL_2400MV (0x02u)

#define AUXS_EXT_LNA_BYPASS (0x80u)

/* Power amplifier current: no reduction of small signal gain. */
#define PAC_PACUR_MAX (0x03u << 5)

/* RFn_CS up to RFn_AGCS, and RFn_TXCUTC up to RFn_PAC, are consecutive and
 * written as one block each. */
#define CHANNEL_RX_BLOCK_LEN (RF215_RG_RFXX_AGCS - RF215_RG_RFXX_CS + 1u)

#define TX_BLOCK_LEN (RF215_RG_RFXX_PAC - RF215_RG_RFXX_TXCUTC + 1u)

/******************************************************************************/
/** Forward Declarations ******************************************************/

static enum rf215_step_result
cfg_poll_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);
static enum rf215_step_result
cfg_cmd_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out);

/******************************************************************************/
/** Static Functions **********************************************************/

/* Picks the recommended frontend table for the modulation and band. */
static const struct rf215_frontend_cfg* frontend_for(const struct rf215_phy_config* phy,
                                                     enum rf215_band band) {
    if (phy->modulation == RF215_MOD_FSK) {
        /* Index 1/2 and below use the narrower receiver, anything above the
         * index 1 set (Tables 6-60 to 6-63). */
        return rf215_phy_fsk_frontend(band, phy->fsk.srate, phy->fsk.midx >= 2u);
    }

    if (phy->modulation == RF215_MOD_QPSK) {
        return rf215_phy_qpsk_frontend(phy->qpsk.fchip);
    }

    return rf215_phy_ofdm_frontend(band, phy->ofdm.opt);
}

/* A run of consecutive registers from the operation's own buffer. */
static enum rf215_step_result write_block(struct rf215_op* op,
                                          struct rf215_xfer* out,
                                          rf215_reg_t reg,
                                          size_t len,
                                          rf215_step_fn next) {
    op->next = next;

    out->op = RF215_XFER_WRITE;
    out->reg = reg;
    out->buf = op->buf;
    out->len = len;

    return RF215_STEP_XFER;
}

/*
 * Encodes a duration for RFn_EDD: a six-bit count of the shortest of four
 * time bases that fits. Rounds up, never down: listening too short is unsafe.
 */
static uint8_t encode_edd(uint16_t micros) {
    static const uint16_t base_us[4] = {
        2u,
        8u,
        32u,
        128u,
    };

    for (uint8_t i = 0u; i < 4u; ++i) {
        const uint16_t count = (uint16_t)((micros + base_us[i] - 1u) / base_us[i]);

        if (count <= 63u) {
            return (uint8_t)((uint8_t)(count << 2) | i);
        }
    }

    /* Longest representable duration. */
    return (uint8_t)((63u << 2) | 3u);
}

/* Reports the configuration as applied. */
static enum rf215_step_result
cfg_finish(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)dev;
    (void)out;

    return rf215_emit(op, RF215_EVT_CONFIGURED, RF215_OK, NULL);
}

/* Puts a receiver that was on air back into RX, after the baseband is on. */
static enum rf215_step_result
cfg_done(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    /* Through TXPREP, with the PLL confirmed locked on the new frequency; the
     * caller need not re-arm. */
    if (trx->receiving) {
        return rf215_op_rx_arm(dev, op, out, cfg_finish);
    }

    return cfg_finish(dev, op, out);
}

/* Switches the baseband back on, with the modulation registers written. */
static enum rf215_step_result
cfg_bb_on(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(
        op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_PC), trx->baseband.pc_value, cfg_done);
}

/*
 * FSK direct modulation, version 3 silicon only: FSKDM.EN pairs with TXDFE.DM
 * from cfg_tx(); pre-emphasis (FSKDM.PE, FSKPE0..2 from Table 6-57) above
 * 100 ksym/s. Version 1 gets the four consecutive registers cleared.
 */
static enum rf215_step_result
cfg_fsk_dm(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const bool direct = (dev->version == RF215_VN_3);
    const bool preemphasis = direct && (op->phy->fsk.srate >= 2u);
    uint8_t pe[3];

    rf215_phy_fsk_preemphasis(op->phy->fsk.srate, pe);

    op->buf[0] = (uint8_t)((direct ? 0x01u : 0u) | (preemphasis ? 0x02u : 0u));
    op->buf[1] = preemphasis ? pe[0] : 0u;
    op->buf[2] = preemphasis ? pe[1] : 0u;
    op->buf[3] = preemphasis ? pe[2] : 0u;

    return write_block(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_FSKDM), 4u, cfg_bb_on);
}

/* Writes the OFDM receiver switches for the bandwidth option. */
static enum rf215_step_result
cfg_ofdm_switches(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(trx, RF215_RG_BBCX_OFDMSW),
                        rf215_phy_ofdm_switches(op->phy->ofdm.opt),
                        cfg_bb_on);
}

/* Second modulation-specific write: FSKPHRTX, OQPSKPHRTX or OFDMPHRTX. */
static enum rf215_step_result
cfg_mod1(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (op->phy->modulation == RF215_MOD_FSK) {
        /* FEC is selected by transmitting SFD1, the coded one (FSKC4.CSFD1). */
        const uint8_t phr =
            (uint8_t)((op->phy->fsk.fec ? 0x08u : 0u) | (op->phy->fsk.dw ? 0x04u : 0u));

        return rf215_write8(
            op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_FSKPHRTX), phr, cfg_fsk_dm);
    }

    if (op->phy->modulation == RF215_MOD_QPSK) {
        return rf215_write8(op,
                            out,
                            rf215_baseband_reg(trx, RF215_RG_BBCX_OQPSKPHRTX),
                            (uint8_t)((op->phy->qpsk.mode & 0x07u) << 1),
                            cfg_bb_on);
    }

    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(trx, RF215_RG_BBCX_OFDMPHRTX),
                        (uint8_t)(op->phy->ofdm.mcs & 0x07u),
                        cfg_ofdm_switches);
}

/* First modulation-specific write: FSKC0..FSKPLL, OQPSKC0 or OFDMC. */
static enum rf215_step_result
cfg_mod0(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (op->phy->modulation == RF215_MOD_FSK) {
        const struct rf215_phy_config* phy = op->phy;

        /* FSKC0..FSKC4 and FSKPLL are consecutive. MIDXS scale 1.0; FSKC2:
         * receiver override 18 dB, interleaving on; FSKC3: default detector
         * thresholds; FSKC4: SFD0 uncoded, SFD1 coded IEEE PPDUs. */
        op->buf[0] = (uint8_t)((uint8_t)(phy->fsk.bt << 6) | 0x10u | (uint8_t)(phy->fsk.midx << 1)
                               | phy->fsk.mord);
        op->buf[1] = (uint8_t)((uint8_t)((phy->fsk.preamble >> 8) & 0x03u) << 6) | phy->fsk.srate;
        op->buf[2] = 0x41u;
        op->buf[3] = 0x85u;
        op->buf[4] = 0x18u;
        op->buf[5] = (uint8_t)(phy->fsk.preamble & 0xFFu);

        return write_block(op, out, rf215_baseband_reg(trx, RF215_RG_BBCX_FSKC0), 6u, cfg_mod1);
    }

    if (op->phy->modulation == RF215_MOD_QPSK) {
        const uint8_t fchip = op->phy->qpsk.fchip & 0x03u;
        const uint8_t direct = rf215_phy_qpsk_direct_mod(fchip) ? 0x10u : 0u;

        return rf215_write8(op,
                            out,
                            rf215_baseband_reg(trx, RF215_RG_BBCX_OQPSKC0),
                            (uint8_t)(fchip | direct),
                            cfg_mod1);
    }

    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(trx, RF215_RG_BBCX_OFDMC),
                        (uint8_t)(op->phy->ofdm.opt & 0x03u),
                        cfg_mod1);
}

/* Writes the energy detect duration. */
static enum rf215_step_result
cfg_edd(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const struct rf215_frontend_cfg* cfg = frontend_for(op->phy, trx->band);
    /* A non-zero caller value overrides the table's recommended duration. */
    const uint16_t edd_us = (op->phy->edd_us != 0u) ? op->phy->edd_us : cfg->edd_us;

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_EDD), encode_edd(edd_us), cfg_mod0);
}

/* Sets the AGC gain map, PA voltage and external LNA bypass. */
static enum rf215_step_result
cfg_auxs(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t value = (uint8_t)(AUXS_PAVOL_2400MV | (uint8_t)((op->phy->agc_map & 0x03u) << 5)
                                    | (op->phy->ext_lna_bypass ? AUXS_EXT_LNA_BYPASS : 0u));

    return rf215_write8(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_AUXS), value, cfg_edd);
}

/* Sets the external frontend pad drive. */
static enum rf215_step_result
cfg_padfe(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t mode = (trx->band == RF215_BAND_24) ? PADFE_MODE_24 : PADFE_MODE_09;

    return rf215_write8(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_PADFE), mode, cfg_auxs);
}

/* Transmitter frontend and power in one transfer: RFn_TXCUTC, RFn_TXDFE and
 * RFn_PAC are consecutive. */
static enum rf215_step_result
cfg_tx(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const struct rf215_frontend_cfg* cfg = frontend_for(op->phy, trx->band);
    /* Direct modulation: per the table, and for all FSK on version 3 silicon
     * (section 6.10.4.2); the matching FSKDM.EN is written in cfg_fsk_dm(). */
    const bool direct = cfg->tx.direct_mod
        || ((op->phy->modulation == RF215_MOD_FSK) && (dev->version == RF215_VN_3));
    /* PAC.TXPWR, 0 to 31, clamped; no per-MCS ceiling is imposed. */
    const uint8_t power = (op->phy->tx_power > 31u) ? 31u : op->phy->tx_power;

    /* RFn_TXCUTC: analog filter cut-off and PA ramp. */
    op->buf[0] = (uint8_t)(cfg->tx.lpfcut | (uint8_t)(cfg->tx.paramp << 6));
    /* RFn_TXDFE: sample rate, cut-off, direct modulation. */
    op->buf[1] = (uint8_t)(cfg->tx.sr | (uint8_t)(cfg->tx.rcut << 5) | (direct ? 0x10u : 0u));
    /* RFn_PAC: transmit power at full PA current. */
    op->buf[2] = (uint8_t)(power | PAC_PACUR_MAX);

    return write_block(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_TXCUTC), TX_BLOCK_LEN, cfg_padfe);
}

/*
 * Channel, receiver frontend and AGC in one transfer: RFn_CS up to RFn_AGCS
 * are consecutive. RFn_CNM latches CS, CCF0 and CNL and must be written after
 * them (section 6.3.2); the ascending block write does that. TRXOFF only.
 */
static enum rf215_step_result
cfg_channel_rx(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];
    const struct rf215_phy_config* phy = op->phy;
    const struct rf215_frontend_cfg* cfg = frontend_for(phy, trx->band);
    const uint16_t freq =
        (uint16_t)((phy->freq_hz - trx->info->freq_offset_hz) / FREQ_RESOLUTION_HZ);

    /* RFn_CS: channel spacing. */
    op->buf[0] = (uint8_t)(phy->channel_spacing_hz / FREQ_RESOLUTION_HZ);
    /* RFn_CCF0L, RFn_CCF0H: centre frequency, little endian. */
    op->buf[1] = (uint8_t)(freq & 0xFFu);
    op->buf[2] = (uint8_t)(freq >> 8);
    /* RFn_CNL, RFn_CNM: CNM.CM = 0 selects the IEEE-compliant channel scheme;
     * the top bits of the channel number live in the same register. */
    op->buf[3] = (uint8_t)(phy->channel & 0xFFu);
    op->buf[4] = (uint8_t)((phy->channel >> 8) & 0x03u);
    /* RFn_RXBWC: receiver bandwidth and IF shift. */
    op->buf[5] = (uint8_t)(cfg->rx.bw | (cfg->rx.if_shift ? 0x10u : 0u));
    /* RFn_RXDFE: receiver sample rate and cut-off. */
    op->buf[6] = (uint8_t)(cfg->rx.sr | (uint8_t)(cfg->rx.rcut << 5));
    /* RFn_AGCC: enable, input, averaging. Reserved bit 7 reads 1 after reset
     * but is written 0, as the recommended configurations give it (Tables
     * 6-93 and 6-106). */
    op->buf[7] = (uint8_t)((cfg->agc.enabled ? 0x01u : 0u) | (cfg->agc.input ? 0x40u : 0u)
                           | (uint8_t)(cfg->agc.avg << 4));
    /* RFn_AGCS: the target level, leaving the gain word at maximum. */
    op->buf[8] = (uint8_t)((uint8_t)((phy->agc_target & 0x07u) << 5) | AGCS_GCW);

    return write_block(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CS), CHANNEL_RX_BLOCK_LEN, cfg_tx);
}

/*
 * Writes the PHY type and FCS setting with the baseband off. It must stay off
 * until every register below is written; cfg_bb_on() switches it back on.
 */
static enum rf215_step_result
cfg_bb_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    struct rf215_trx* trx = &dev->trx[op->band];
    const uint8_t phy_type = (op->phy->modulation == RF215_MOD_QPSK) ? PC_PT_QPSK
        : (op->phy->modulation == RF215_MOD_FSK)                     ? PC_PT_FSK
                                                                     : PC_PT_OFDM;
    const uint8_t value = (uint8_t)(phy_type | RF215_PC_BBEN | (op->phy->fcs ? PC_FCS : 0u));

    /* Kept so transmit can switch the baseband off for a channel assessment
     * and restore it; the FCS flag sets the transmit and receive lengths. */
    trx->baseband.pc_value = value;
    trx->baseband.fcs = op->phy->fcs;

    /* In TRXOFF: a frame end still latched belongs to the old configuration
     * and must not be read out as a frame received on the new one. */
    rf215_rx_off_air(trx);

    return rf215_write8(op,
                        out,
                        rf215_baseband_reg(trx, RF215_RG_BBCX_PC),
                        (uint8_t)(value & (uint8_t)~RF215_PC_BBEN),
                        cfg_channel_rx);
}

/* Proceeds in TRXOFF, re-issues the command past the deadline, otherwise keeps polling. */
static enum rf215_step_result
cfg_off_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_TRXOFF) {
        return cfg_bb_off(dev, op, out);
    }

    if (!rf215_past_deadline(dev, op)) {
        return cfg_poll_off(dev, op, out);
    }

    op->tries++;

    if (op->tries < TRXOFF_ATTEMPTS) {
        return cfg_cmd_off(dev, op, out);
    }

    RF215_LOGE(dev, "configure: transceiver never went off air");
    dev->stats.timeouts++;

    return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_TIMEOUT, NULL);
}

/* Re-reads RFn_STATE while the transition to TRXOFF completes. */
static enum rf215_step_result
cfg_read_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, cfg_off_state);
}

/* Sleeps between reads of RFn_STATE. */
static enum rf215_step_result
cfg_poll_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    (void)out;

    return rf215_delay(dev, op, TRXOFF_POLL_US, cfg_read_off);
}

/*
 * Commands TRXOFF, which every state yields to, and starts the deadline.
 * Written once per deadline, not per poll: only a lost command stalls it.
 */
static enum rf215_step_result
cfg_cmd_off(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_trx* trx = &dev->trx[op->band];

    rf215_set_deadline(dev, op, TRXOFF_TIMEOUT_US);

    return rf215_write8(
        op, out, rf215_radio_reg(trx, RF215_RG_RFXX_CMD), RF215_CMD_RF_TRXOFF, cfg_poll_off);
}

static enum rf215_step_result
cfg_state(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    if ((op->buf[0] & RF215_STATE_MASK) == RF215_STATE_TRXOFF) {
        return cfg_bb_off(dev, op, out);
    }

    return cfg_cmd_off(dev, op, out);
}

/* Whether the transceiver can tune to this centre frequency. */
static bool in_band(enum rf215_band band, uint64_t hz) {
    if (band == RF215_BAND_24) {
        return (hz >= BAND_24_MIN_HZ) && (hz <= BAND_24_MAX_HZ);
    }

    return ((hz >= BAND_09_LOW_MIN_HZ) && (hz <= BAND_09_LOW_MAX_HZ))
        || ((hz >= BAND_09_HIGH_MIN_HZ) && (hz <= BAND_09_HIGH_MAX_HZ));
}

/*
 * The checks below each return NULL for settings the chip can take, or the
 * reason they are refused. One rule per test, so a refusal names its cause.
 */

/* Table 6-88: MCS 0 is not defined for options 3 and 4, nor MCS 1 for
 * option 4. The chip accepts the registers and transmits nothing usable. */
static const char* check_ofdm(const struct rf215_phy_config_ofdm* ofdm) {
    if (ofdm->opt > 3u) {
        return "configure: OFDM option out of range";
    }

    if (ofdm->mcs > 6u) {
        return "configure: OFDM MCS out of range";
    }

    if ((ofdm->opt >= 2u) && (ofdm->mcs == 0u)) {
        return "configure: OFDM MCS 0 needs option 1 or 2";
    }

    if ((ofdm->opt == 3u) && (ofdm->mcs == 1u)) {
        return "configure: OFDM MCS 1 is not defined for option 4";
    }

    return NULL;
}

/* Section 6.10.2.6: 4-FSK needs a modulation index of at least 1 and a
 * bandwidth-time product of 2. The preamble is a ten bit field. */
static const char* check_fsk(const struct rf215_phy_config_fsk* fsk) {
    if (fsk->srate > 5u) {
        return "configure: FSK symbol rate out of range";
    }

    if (fsk->midx > 7u) {
        return "configure: FSK modulation index out of range";
    }

    if (fsk->mord > 1u) {
        return "configure: FSK order out of range";
    }

    if (fsk->bt > 3u) {
        return "configure: FSK bandwidth-time product out of range";
    }

    if ((fsk->preamble == 0u) || (fsk->preamble > 1023u)) {
        return "configure: FSK preamble length out of range";
    }

    if ((fsk->mord == 1u) && (fsk->midx < 3u)) {
        return "configure: 4-FSK needs a modulation index of at least 1";
    }

    if ((fsk->mord == 1u) && (fsk->bt != 3u)) {
        return "configure: 4-FSK needs a bandwidth-time product of 2";
    }

    return NULL;
}

/* Rate mode 4 exists only at 2000 kchip/s; at the other chip rates the chip
 * takes the registers and falls back to mode 0. */
static const char* check_qpsk(const struct rf215_phy_config_qpsk* qpsk) {
    if (qpsk->fchip > 3u) {
        return "configure: O-QPSK chip rate out of range";
    }

    if (qpsk->mode > 4u) {
        return "configure: O-QPSK rate mode out of range";
    }

    if ((qpsk->mode == 4u) && (qpsk->fchip != 3u)) {
        return "configure: O-QPSK rate mode 4 needs 2000 kchip/s";
    }

    return NULL;
}

static const char* check_modulation(const struct rf215_phy_config* phy) {
    switch (phy->modulation) {
        case RF215_MOD_OFDM:
            return check_ofdm(&phy->ofdm);
        case RF215_MOD_FSK:
            return check_fsk(&phy->fsk);
        case RF215_MOD_QPSK:
            return check_qpsk(&phy->qpsk);
        default:
            return "configure: unsupported modulation";
    }
}

/* Frequency, channel number and spacing, and the channel they add up to. */
static const char* check_channel(const struct rf215_trx* trx, const struct rf215_phy_config* phy) {
    const uint32_t offset = trx->info->freq_offset_hz;
    const uint32_t spacing = phy->channel_spacing_hz / FREQ_RESOLUTION_HZ;

    if ((phy->freq_hz < offset) || (((phy->freq_hz - offset) / FREQ_RESOLUTION_HZ) > 0xFFFFu)) {
        return "configure: frequency out of range for this band";
    }

    /* CNM holds the channel mode in bits 7:6 and the top channel bits in 1:0;
     * a channel over ten bits would change the channel scheme. */
    if (phy->channel > RF215_CHANNEL_MAX) {
        return "configure: channel number out of range";
    }

    if (spacing > 0xFFu) {
        return "configure: channel spacing out of range";
    }

    /* The channel actually tuned is channel 0 plus the channel number times
     * the spacing, as the chip rounds it. */
    if (!in_band(trx->band,
                 (uint64_t)phy->freq_hz
                     + ((uint64_t)phy->channel * spacing * FREQ_RESOLUTION_HZ))) {
        return "configure: frequency outside the band's tuning range";
    }

    return NULL;
}

static const char* check_phy(const struct rf215_trx* trx, const struct rf215_phy_config* phy) {
    const char* refused = check_modulation(phy);

    if (refused != NULL) {
        return refused;
    }

    refused = check_channel(trx, phy);

    if (refused != NULL) {
        return refused;
    }

    if ((phy->edd_us != 0u)
        && ((phy->edd_us < RF215_EDD_US_MIN) || (phy->edd_us > RF215_EDD_US_MAX))) {
        return "configure: energy detect duration out of range";
    }

    return NULL;
}

/******************************************************************************/
/** Public Functions **********************************************************/

enum rf215_step_result
rf215_op_configure_start(struct rf215_dev* dev, struct rf215_op* op, struct rf215_xfer* out) {
    const struct rf215_phy_config* phy = op->phy;
    const struct rf215_trx* trx = &dev->trx[op->band];

    if (phy == NULL) {
        RF215_LOGE(dev, "configure: no configuration given");
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_INVAL, NULL);
    }

    const char* refused = check_phy(trx, phy);

    if (refused != NULL) {
        RF215_LOGE(dev, refused);
        return rf215_emit(op, RF215_EVT_ERROR, RF215_ERR_INVAL, NULL);
    }

    /* Channel, PHY and frontend registers and the transmit power are only
     * taken in TRXOFF (sections 5.2.7, 6.3.2, 6.9.1) and silently ignored
     * otherwise, so the transceiver is taken off air first. */
    op->tries = 0u;

    return rf215_read(op, out, rf215_radio_reg(trx, RF215_RG_RFXX_STATE), 1u, cfg_state);
}

/******************************************************************************/
/** Step Table ****************************************************************/

/*
 *   rf215_op_configure_start ---- settings refused ----> ERROR
 *            |
 *            |  read RFn_STATE
 *            v
 *        cfg_state ---- already TRXOFF ----------------------------+
 *            |                                                     |
 *            v                                                     |
 *       cfg_cmd_off <---- 100 ms gone, tries left ----+            |
 *            |                                        |            |
 *            |  write CMD=TRXOFF                      |            |
 *            v                                        |            |
 *       cfg_poll_off <---- not TRXOFF yet ----+       |            |
 *            |                                |       |            |
 *            |  200 us                        |       |            |
 *            v                                |       |            |
 *       cfg_read_off ----> cfg_off_state -----+-------+            |
 *                               |    |                             |
 *                               |    +---- 3 tries used ----> ERROR
 *                               |  TRXOFF                          |
 *                               v                                  |
 *                          cfg_bb_off <----------------------------+
 *                               |
 *                               v
 *         cfg_channel_rx -> cfg_tx -> cfg_padfe -> cfg_auxs -> cfg_edd
 *                                                                 |
 *                               +---------------------------------+
 *                               v
 *                           cfg_mod0 -> cfg_mod1
 *                                          |
 *                               +----------+   OFDM: cfg_ofdm_switches
 *                               |              FSK: cfg_fsk_dm
 *                               v              O-QPSK: straight on
 *                           cfg_bb_on
 *                               |
 *                               v
 *                           cfg_done ---- receiver wanted ----> arming (op_rx.c) ----+
 *                               |                                                    |
 *                               v                                                    |
 *                          cfg_finish <----------------------------------------------+
 *                               |
 *                               v
 *                          CONFIGURED
 */

static const struct rf215_step_name steps[] = {
    {
        RF215_STEP(rf215_op_configure_start),
    },
    {
        RF215_STEP(cfg_state),
    },
    {
        RF215_STEP(cfg_cmd_off),
    },
    {
        RF215_STEP(cfg_poll_off),
    },
    {
        RF215_STEP(cfg_read_off),
    },
    {
        RF215_STEP(cfg_off_state),
    },
    {
        RF215_STEP(cfg_bb_off),
    },
    {
        RF215_STEP(cfg_channel_rx),
    },
    {
        RF215_STEP(cfg_tx),
    },
    {
        RF215_STEP(cfg_padfe),
    },
    {
        RF215_STEP(cfg_auxs),
    },
    {
        RF215_STEP(cfg_edd),
    },
    {
        RF215_STEP(cfg_mod0),
    },
    {
        RF215_STEP(cfg_mod1),
    },
    {
        RF215_STEP(cfg_ofdm_switches),
    },
    {
        RF215_STEP(cfg_fsk_dm),
    },
    {
        RF215_STEP(cfg_bb_on),
    },
    {
        RF215_STEP(cfg_done),
    },
    {
        RF215_STEP(cfg_finish),
    },
};

const struct rf215_step_name* rf215_op_configure_steps(size_t* count) {
    *count = sizeof(steps) / sizeof(steps[0]);

    return steps;
}
