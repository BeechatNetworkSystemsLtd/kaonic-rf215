/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#include "rf215/rf215_baseband.h"

/*****************************************************************************/

int rf215_baseband_rx_frame(struct rf215_trx* trx,
                            rf215_millis_t timeout,
                            struct rf215_frame* frame) {

    return rf215_baseband_rx(trx, timeout, frame->data, &frame->len);
}

int rf215_baseband_rx(struct rf215_trx* trx, rf215_millis_t timeout, uint8_t* data, uint16_t* len) {
    int rc = 0;

    rc += rf215_set_state(trx, RF215_STATE_RX);
    rc += rf215_wait_state(trx, RF215_STATE_RX);

    if (rc == 0) {
        rc = -1;

        const rf215_millis_t start_time = rf215_current_time(trx->dev);

        while ((rf215_current_time(trx->dev) - start_time) <= timeout) {
            if (rf215_wait_clear_baseband_irq(trx, timeout, RF215_BASEBAND_IRQ_RXFE)) {
                rc = rf215_baseband_rx_load(trx, data, len);
                break;
            }
        }
    }

    rc += rf215_set_state(trx, RF215_STATE_RX);

    return rc;
}

int rf215_baseband_rx_load(struct rf215_trx* trx, uint8_t* data, uint16_t* len) {
    int rc = 0;

    // Transceiver should be in RX state
    if (trx->state != RF215_STATE_RX) {
        rc = -1;
    }

    const uint16_t max_len = *len;

    if (rc == 0) {
        rc = rf215_read_data(trx->dev, trx->baseband_regs->RG_RXFLL, len, sizeof(*len));

        if (*len > RF215_FRAME_DATA_MAX_LEN) {
            rc = -1;
        }
    }

    if (rc == 0) {

        if ((max_len > 0) && (*len > max_len)) {
            *len = max_len;
        }

        rc = rf215_read_data(trx->dev, trx->baseband_regs->RG_FBRXS, data, *len);
    }

    return rc;
}

int rf215_baseband_cca_tx_frame(struct rf215_trx* trx, struct rf215_frame* frame) {
    return rf215_baseband_cca_tx(trx, frame->data, frame->len);
}

int rf215_baseband_cca_tx(struct rf215_trx* trx, uint8_t* data, uint16_t length) {

    int rc = 0;
    uint8_t reg_data;

    rc += rf215_set_state(trx, RF215_STATE_TRXOFF);
    rc += rf215_wait_state(trx, RF215_STATE_TRXOFF);
    if (rc != 0) {
        return -1;
    }

    // Enable CCATX
    rc += rf215_write_reg(trx->dev, trx->baseband_regs->RG_AMCS, 0x01 << 1);
    if (rc != 0) {
        return -2;
    }

    // Auto Mode Energy Detection Threshold
    rc += rf215_write_reg(trx->dev, trx->baseband_regs->RG_AMEDT, -50);
    if (rc != 0) {
        return -3;
    }

    // Disable baseband
    rc += rf215_read_reg(trx->dev, trx->baseband_regs->RG_PC, &reg_data);
    if (rc != 0) {
        return -4;
    }

    reg_data &= ~(0x01 << 2);

    rc += rf215_write_reg(trx->dev, trx->baseband_regs->RG_PC, reg_data);
    if (rc != 0) {
        return -5;
    }

    rc += rf215_set_state(trx, RF215_STATE_TXPREP);
    rc += rf215_wait_state(trx, RF215_STATE_TXPREP);
    if (rc != 0) {
        return -6;
    }

    rc = -7;
    if (rf215_wait_clear_radio_irq(trx, 500, RF215_RADIO_IRQ_TRXRDY)) {
        rc = 0;

        rc += rf215_set_state(trx, RF215_STATE_RX);
        if (rc != 0) {
            return -8;
        }

        // Start single energy detection measurement
        rc += rf215_write_reg(trx->dev, trx->radio_regs->RG_EDC, 0x01);
        if (rc != 0) {
            return -10;
        }

        rc += rf215_baseband_tx_load(trx, data, length);
        if (rc != 0) {
            return -9;
        }
    }

    if (rc == 0) {
        rc = -11;

        rf215_wait_state(trx, RF215_STATE_TXPREP);

        // Wait for TXFE IRQ
        if (rf215_wait_clear_baseband_irq(trx, 500, RF215_BASEBAND_IRQ_TXFE)) {
            // Read CCAED
            rc = rf215_read_reg(trx->dev, trx->baseband_regs->RG_AMCS, &reg_data);
            if (rc == 0) {
                rc = -12;

                // Channel is clear
                if ((reg_data & (0x01 << 2)) == 0) {
                    rc = 0;
                }
            }
        }
    }

    rf215_set_state(trx, RF215_STATE_RX);

    return rc;
}

int rf215_baseband_tx_frame(struct rf215_trx* trx, struct rf215_frame* frame) {
    return rf215_baseband_tx(trx, frame->data, frame->len);
}

int rf215_baseband_tx(struct rf215_trx* trx, uint8_t* data, uint16_t len) {
    int rc = 0;

    rc += rf215_set_state(trx, RF215_STATE_TXPREP);
    rc += rf215_baseband_tx_load(trx, data, len);

    if (rc == 0) {

        rc = -1;

        if (rf215_wait_clear_radio_irq(trx, 500, RF215_RADIO_IRQ_TRXRDY)) {

            (void)rf215_cmd(trx, RF215_CMD_RF_TX);

            if (rf215_wait_clear_baseband_irq(trx, 500, RF215_BASEBAND_IRQ_TXFE)) {
                rc = 0;
            }
        }
    }

    rc += rf215_set_state(trx, RF215_STATE_RX);

    return rc;
}

int rf215_baseband_tx_load(struct rf215_trx* trx, uint8_t* data, uint16_t len) {

    int rc = -1;

    if ((data != NULL) && (len > 0u) && (len <= RF215_FRAME_DATA_MAX_LEN)) {

        rc = rf215_write_data(trx->dev, trx->baseband_regs->RG_TXFLL, &len, sizeof(len));

        if (rc == 0) {
            rc = rf215_write_data(trx->dev, trx->baseband_regs->RG_FBTXS, data, len);
        }
    }

    return rc;
}

/*****************************************************************************/
