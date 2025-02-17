/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#include "rf215/rf215.h"

/*****************************************************************************/

int rf215_write_reg(const struct rf215_device* dev, rf215_reg_t reg, uint8_t value) {
    return rf215_write_data(dev, reg, &value, sizeof(value));
}

int rf215_read_reg(const struct rf215_device* dev, rf215_reg_t reg, uint8_t* value) {
    return rf215_read_data(dev, reg, value, sizeof(*value));
}

int rf215_write_data(const struct rf215_device* dev, rf215_reg_t reg, void* data, size_t len) {
    return dev->iface.write(dev->iface.ctx, (reg | RF215_FLAG_WRITE), data, len);
}

int rf215_read_data(const struct rf215_device* dev, rf215_reg_t reg, void* data, size_t len) {
    return dev->iface.read(dev->iface.ctx, (reg | RF215_FLAG_READ), data, len);
}

/*****************************************************************************/

static int trx_reset(struct rf215_trx* trx) {
    int rc = 0;

    rc += rf215_cmd(trx, RF215_CMD_RF_TRXOFF);
    rc += rf215_wait_state(trx, RF215_STATE_TRXOFF);
    rc += rf215_write_reg(trx->dev, trx->radio_regs->RG_IRQM, RF215_RADIO_IRQ_MASK);
    rc += rf215_write_reg(trx->dev, trx->baseband_regs->RG_IRQM, RF215_BASEBAND_IRQ_MASK);

    rf215_clear_radio_irq(trx, RF215_RADIO_IRQ_MASK);
    rf215_clear_baseband_irq(trx, RF215_BASEBAND_IRQ_MASK);

    return rc;
}

static void init_trx(struct rf215_device* dev, enum rf215_trx_type type, struct rf215_trx* trx) {
    trx->type = type;
    trx->dev = dev;
    trx->radio_regs = rf215_regs_for_radio(type);
    trx->baseband_regs = rf215_regs_for_baseband(type);
}

void rf215_sleep(const struct rf215_device* dev, rf215_millis_t delay) {
    if ((dev != NULL) && (dev->iface.sleep != NULL)) {
        dev->iface.sleep(dev->iface.ctx, delay);
    }
}

bool rf215_wait_irq(struct rf215_device* dev, rf215_millis_t timeout, rf215_irq_data_t* irq) {

    bool has_irq = false;

    if (irq != NULL) {
        *irq = 0u;
    }

    if (dev->iface.wait_irq != NULL) {

        rf215_irq_data_t irq_data = 0u;
        has_irq = dev->iface.wait_irq(dev->iface.ctx, timeout, &irq_data);

        if (has_irq) {

            dev->rf09.radio_irq |= (irq_data >> (8u * 0u)) & 0xFFu;
            dev->rf09.baseband_irq |= (irq_data >> (8u * 2u)) & 0xFFu;

            dev->rf24.radio_irq |= (irq_data >> (8u * 1u)) & 0xFFu;
            dev->rf24.baseband_irq |= (irq_data >> (8u * 3u)) & 0xFFu;

            if (irq != NULL) {
                *irq = irq_data;
            }
        }
    }

    return has_irq;
}

int rf215_init(struct rf215_device* dev) {

    int rc = 0;

    init_trx(dev, RF215_TRX_TYPE_RF09, &dev->rf09);
    init_trx(dev, RF215_TRX_TYPE_RF24, &dev->rf24);

    dev->common_regs = rf215_regs_for_common();

    // Hardware reset the module
    rf215_hardware_reset(dev);

    // Check for known part number
    const enum rf215_pn pn = rf215_probe(dev);
    if (pn == RF215_PN_UNKNOWN) {
        // TODO: print error
    }

    return rc;
}

enum rf215_pn rf215_probe(const struct rf215_device* dev) {

    enum rf215_pn pn = RF215_PN_UNKNOWN;

    uint8_t value;
    const int rc = rf215_read_reg(dev, 0x0D, &value);

    if (rc == 0) {

        static const uint8_t known_pn[] = {
            RF215_PN_AT86RF215,
            RF215_PN_AT86RF215IQ,
            RF215_PN_AT86RF215M,
        };

        for (size_t i = 0u; i < sizeof(known_pn) / sizeof(known_pn[0]); ++i) {
            if (value == known_pn[i]) {
                pn = (enum rf215_pn)value;
                break;
            }
        }
    }

    return pn;
}

void rf215_hardware_reset(struct rf215_device* dev) {

    if (dev->iface.hardware_reset != NULL) {
        dev->iface.hardware_reset(dev->iface.ctx);
    }

    rf215_wait_irq(dev, 150u, NULL);

    trx_reset(&dev->rf09);
    trx_reset(&dev->rf24);
}

int rf215_wait_state(struct rf215_trx* trx, enum rf215_state state) {

    int rc;
    uint8_t value = 0xFF;

    const size_t max_repeats = 20u;
    size_t repeat = 0u;

    rc = 0;
    while ((rc == 0) && (value != (uint8_t)state)) {

        rc = rf215_read_reg(trx->dev, trx->radio_regs->RG_STATE, &value);

        ++repeat;
        if (repeat >= max_repeats) {
            rc = -1;
        }

        rf215_sleep(trx->dev, 1u);
    }

    if (rc == 0) {
        trx->state = state;
    }

    return rc;
}

int rf215_set_state(struct rf215_trx* trx, enum rf215_state state) {

    enum rf215_cmd cmd = RF215_CMD_RF_NOP;
    switch (state) {
        case RF215_STATE_TRXOFF:
            cmd = RF215_CMD_RF_TRXOFF;
            break;
        case RF215_STATE_TXPREP:
            cmd = RF215_CMD_RF_TRXPREP;
            break;
        case RF215_STATE_TX:
            cmd = RF215_CMD_RF_TX;
            break;
        case RF215_STATE_RX:
            cmd = RF215_CMD_RF_RX;
            break;
        case RF215_STATE_TRANSITION:
            cmd = RF215_CMD_RF_NOP;
            break;
        case RF215_STATE_RESET:
            cmd = RF215_CMD_RF_RESET;
            break;
    }

    return rf215_cmd(trx, cmd);
}

bool rf215_clear_radio_irq(struct rf215_trx* trx, rf215_irq_t irq) {
    const bool has_irq = (irq & trx->radio_irq) == irq;

    trx->radio_irq &= ~(irq);

    return has_irq;
}

bool rf215_clear_baseband_irq(struct rf215_trx* trx, rf215_irq_t irq) {
    const bool has_irq = (irq & trx->baseband_irq) == irq;

    trx->baseband_irq &= ~(irq);

    return has_irq;
}

bool rf215_wait_clear_radio_irq(struct rf215_trx* trx, rf215_millis_t timeout, rf215_irq_t irq) {
    bool has_irq = rf215_clear_radio_irq(trx, irq);

    if (!has_irq) {
        if (rf215_wait_irq(trx->dev, timeout, NULL)) {
            has_irq = rf215_clear_radio_irq(trx, irq);
        }
    }

    return has_irq;
}

bool rf215_wait_clear_baseband_irq(struct rf215_trx* trx, rf215_millis_t timeout, rf215_irq_t irq) {
    bool has_irq = rf215_clear_baseband_irq(trx, irq);

    if (!has_irq) {
        if (rf215_wait_irq(trx->dev, timeout, NULL)) {
            has_irq = rf215_clear_baseband_irq(trx, irq);
        }
    }

    return has_irq;
}

int rf215_cmd(struct rf215_trx* trx, enum rf215_cmd cmd) {
    return rf215_write_reg(trx->dev, trx->radio_regs->RG_CMD, (uint8_t)cmd);
}

int rf215_set_freq(struct rf215_trx* trx, const struct rf215_freq* freq) {

    int rc = -1;

    if (freq != NULL) {
        rc = 0;

        uint32_t frequency = freq->frequency;

        if (trx->type == RF215_TRX_TYPE_RF24) {
            const uint32_t freq_offset = 1500000000u;

            if (frequency > freq_offset) {
                frequency = frequency - freq_offset;
            } else {
                rc = -1;
            }
        }

        uint16_t frequency_word = frequency / RF215_FREQ_RESOLUTION_KHZ;
        uint16_t channel = (freq->channel & 0x1FFu);
        uint8_t cs = freq->channel_spacing / RF215_FREQ_RESOLUTION_KHZ;

        if (rc == 0) {
            rc = rf215_write_data(trx->dev, trx->radio_regs->RG_CS, &cs, sizeof(cs));
        }

        if (rc == 0) {
            rc = rf215_write_data(
                trx->dev, trx->radio_regs->RG_CCF0L, &frequency_word, sizeof(frequency_word));
        }

        if (rc == 0) {
            rc = rf215_write_data(trx->dev, trx->radio_regs->RG_CNL, &channel, sizeof(channel));
        }

        // Refresh frequency settings
        if (rc == 0) {
            rc = rf215_write_reg(trx->dev, trx->radio_regs->RG_CNM, 0x00u);
        }
    }

    return rc;
}

rf215_millis_t rf215_current_time(struct rf215_device* dev) {
    rf215_millis_t time = 0u;

    if (dev->iface.current_time != NULL) {
        time = dev->iface.current_time(dev->iface.ctx);
    }

    return time;
}

struct rf215_trx* rf215_get_trx(struct rf215_device* dev, enum rf215_trx_type type) {
    struct rf215_trx* trx = NULL;
    switch (type) {
        case RF215_TRX_TYPE_RF09:
            trx = &dev->rf09;
            break;
        case RF215_TRX_TYPE_RF24:
            trx = &dev->rf24;
            break;
    }

    return trx;
}

struct rf215_trx* rf215_get_trx_by_freq(struct rf215_device* dev, uint32_t freq) {
    if (freq <= 1500000000u) {
        return &dev->rf09;
    }

    return &dev->rf24;
}

int rf215_write_reg_set(const struct rf215_device* dev, const struct rf215_reg_set* reg_set) {
    int rc = -1;

    if ((dev != NULL) && (reg_set != NULL) && (reg_set->values != NULL)) {
        rc = 0;
        for (size_t i = 0; i < reg_set->len; ++i) {
            rc += rf215_write_reg(dev, reg_set->values[i].reg, reg_set->values[i].value);
        }
    }

    return rc;
}
