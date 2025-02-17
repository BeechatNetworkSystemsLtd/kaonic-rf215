/*
 * Copyright (c) 2025 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/*****************************************************************************/

#ifndef KAONIC_DRIVERS_RF215_H__
#define KAONIC_DRIVERS_RF215_H__

/*****************************************************************************/

#include "rf215_common.h"
#include "rf215_regs.h"

/*****************************************************************************/

/* Maximum supported data size */
#define RF215_FRAME_DATA_MAX_LEN (2048u)

/*****************************************************************************/

/**
 * @brief Represents frame of data
 */
struct rf215_frame {
    uint16_t len;                           /* Length of the data */
    uint8_t data[RF215_FRAME_DATA_MAX_LEN]; /* Data buffer */
};

struct rf215_freq {
    uint32_t channel_spacing; /* Channel spacing in kHz */
    uint32_t frequency;       /* Frequency in kHz */
    uint8_t channel;          /* Selected channel */
};

/**
 * @brief Represents low-level interface for RF215 module
 */
struct rf215_iface {
    const void* ctx; /* Keeps internal context data */

    /**
     * @brief   Write register data to RF215 module
     *
     * @param   ctx Interface context
     * @param   reg Register address to write
     * @param   data Pointer to buffer where data is stored
     * @param   len Buffer length in bytes
     *
     * @return  error code
     */
    int (*write)(const void* ctx, rf215_reg_t reg, void* data, size_t len);

    /**
     * @brief   Read register data from RF215 module
     *
     * @param   ctx Interface context
     * @param   reg Register address to read
     * @param   data Pointer to buffer where data needs to be stored
     * @param   len Expected length in bytes
     *
     * @return  error code
     */
    int (*read)(const void* ctx, rf215_reg_t reg, void* data, size_t len);

    /**
     * @brief   Invoke hardware reset by pulse on RST pin
     *
     * @param   ctx Interface context
     */
    void (*hardware_reset)(const void* ctx);

    /**
     * @brief   Wait on IRQ event or timeout
     *
     * @param   ctx Interface context
     * @param   timeout Timeout in milliseconds (0 to no wait)
     * @param   irq Read IRQ events from the chip if not null
     *
     * @return  true if IRQ has been occurred otherwise false
     */
    bool (*wait_irq)(const void* ctx, rf215_millis_t timeout, rf215_irq_data_t* irq);

    /**
     * @brief   Sleep for specific delay
     *
     * @param   ctx Interface context
     * @param   delay Sleep delay in milliseconds
     *
     * @return  void
     */
    void (*sleep)(const void* ctx, rf215_millis_t delay);

    /**
     * @brief   Return current time in milliseconds
     *
     * @param   ctx Interface context
     *
     * @return  current time in milliseconds
     */
    rf215_millis_t (*current_time)(const void* ctx);
};

struct rf215_device;

/*
 * @brief The RF215 Transceiver
 */
struct rf215_trx {
    enum rf215_trx_type type;
    struct rf215_device* dev; /* Pointer to parent device */

    const struct rf215_radio_regs* radio_regs;
    const struct rf215_baseband_regs* baseband_regs;
    enum rf215_state state;

    rf215_irq_t radio_irq;
    rf215_irq_t baseband_irq;
};

struct rf215_device {
    struct rf215_iface iface; /* Communication interface */

    struct rf215_trx rf09; /* RF09 sub-GHz Transceiver */
    struct rf215_trx rf24; /* RF24 2.4-GHz Transceiver */

    const struct rf215_common_regs* common_regs; /* Common Registers */
};

/*****************************************************************************/

/**
 * @brief   Initialize RF215 module
 *
 * @param   dev Device to initialize
 *
 * @return  error code
 */
int rf215_init(struct rf215_device* dev);

/**
 * @brief Writes value to register
 *
 * @param dev
 * @param reg
 * @param value
 *
 * @return error code
 */
int rf215_write_reg(const struct rf215_device* dev, rf215_reg_t reg, uint8_t value);
int rf215_read_reg(const struct rf215_device* dev, rf215_reg_t reg, uint8_t* value);

int rf215_write_data(const struct rf215_device* dev, rf215_reg_t reg, void* data, size_t len);
int rf215_read_data(const struct rf215_device* dev, rf215_reg_t reg, void* data, size_t len);

int rf215_write_reg_set(const struct rf215_device* dev, const struct rf215_reg_set* reg_set);

void rf215_sleep(const struct rf215_device* dev, rf215_millis_t delay);

void rf215_hardware_reset(struct rf215_device* dev);

enum rf215_pn rf215_probe(const struct rf215_device* dev);

bool rf215_wait_irq(struct rf215_device* dev, rf215_millis_t timeout, rf215_irq_data_t* irq);
bool rf215_clear_radio_irq(struct rf215_trx* trx, rf215_irq_t irq);
bool rf215_clear_baseband_irq(struct rf215_trx* trx, rf215_irq_t irq);

bool rf215_wait_clear_radio_irq(struct rf215_trx* trx, rf215_millis_t timeout, rf215_irq_t irq);
bool rf215_wait_clear_baseband_irq(struct rf215_trx* trx, rf215_millis_t timeout, rf215_irq_t irq);

int rf215_set_freq(struct rf215_trx* trx, const struct rf215_freq* freq);

int rf215_cmd(struct rf215_trx* trx, enum rf215_cmd cmd);
int rf215_wait_state(struct rf215_trx* trx, enum rf215_state state);
int rf215_set_state(struct rf215_trx* trx, enum rf215_state state);

rf215_millis_t rf215_current_time(struct rf215_device* dev);

struct rf215_trx* rf215_get_trx(struct rf215_device* dev, enum rf215_trx_type type);

struct rf215_trx* rf215_get_trx_by_freq(struct rf215_device* dev, uint32_t freq);

#endif // KAONIC_DRIVERS_RF215_H_

