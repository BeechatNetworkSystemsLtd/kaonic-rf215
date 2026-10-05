/*
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file
 * @brief The RF215 core: a non-blocking driver that owns the chip's state
 *        machine.
 *
 * A host submits an operation, then drives it by calling @ref rf215_step in a
 * loop. Each call either asks for a bus transfer, asks to wait, reports an
 * event, or reports that nothing is outstanding. The host performs transfers
 * and delivers interrupts; all sequencing, timing and error recovery live in
 * the core, so kernel, userspace and bare-metal ports share one implementation.
 *
 * Typical host loop:
 * @code
 * rf215_submit(dev, &(struct rf215_req){ .kind = RF215_REQ_RESET });
 *
 * for (;;) {
 *     struct rf215_xfer xfer;
 *     struct rf215_event evt;
 *     rf215_micros_t delay;
 *
 *     switch (rf215_step(dev, &xfer, &delay, &evt)) {
 *     case RF215_STEP_XFER:  host_start_xfer(&xfer);  break;  <- calls rf215_xfer_done()
 *     case RF215_STEP_WAIT:  host_wait(delay);        break;  <- or wait for an interrupt
 *     case RF215_STEP_EVENT: host_on_event(&evt);     break;
 *     case RF215_STEP_IDLE:  return;
 *     }
 * }
 * @endcode
 *
 * The core is not thread safe. A host must serialise @ref rf215_submit,
 * @ref rf215_step and @ref rf215_xfer_done; only @ref rf215_notify_irq may be
 * called concurrently, from interrupt context.
 */

#ifndef KAONIC_DRIVERS_RF215_API_H__
#define KAONIC_DRIVERS_RF215_API_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_port.h"
#include "rf215_regs.h"
#include "rf215_types.h"

/******************************************************************************/
/** Defines *******************************************************************/

/** @brief Number of transceivers in the device (sub-GHz and 2.4 GHz). */
#define RF215_TRX_COUNT (2u)

/** Shortest and longest energy detect duration a caller may ask for. */
#define RF215_EDD_US_MIN (64u)
#define RF215_EDD_US_MAX (8064u)

/**
 * @name Channel assessment defaults and limits
 *
 * Declared here so a port configures the core without restating its numbers:
 * a second copy of a default is a second thing to keep in step, and one that
 * disagreed once already.
 * @{
 */
/** Assess the channel before transmitting. */
#define RF215_CCA_DEFAULT_ENABLED true
/** Energy above which the channel counts as busy, dBm. */
#define RF215_CCA_DEFAULT_THRESHOLD_DBM (-80)
/**
 * Retries on a busy channel before the frame is dropped.
 *
 * The retries together have to outlast one frame from a neighbour - at the
 * slowest modulation a full-length frame holds the air for over a tenth of a
 * second - or a transmit is dropped whenever someone sends something large.
 */
#define RF215_CCA_DEFAULT_RETRIES (96u)
/** Base wait between attempts, microseconds. */
#define RF215_CCA_DEFAULT_BACKOFF_US (500u)
/**
 * Longest single wait between attempts, microseconds.
 *
 * The budget comes from asking often rather than waiting longer: a wait beyond
 * this sleeps through the gap it was waiting for. Anything larger is capped, so
 * this is the real upper bound and what a port should accept.
 */
#define RF215_CCA_BACKOFF_US_MAX (2000u)
/** @} */

/** Signal strength value meaning "no valid measurement" (the chip reports 127). */
#define RF215_RSSI_INVALID ((int8_t)-128)

/** Chip versions, as RF_VN reports them. */
#define RF215_VN_1 (0x01u)
#define RF215_VN_3 (0x03u)

/** Largest channel number RFn_CNM can carry (CNH 1:0 plus CNL 7:0). */
#define RF215_CHANNEL_MAX (0x3FFu)

/******************************************************************************/
/** Types *********************************************************************/

/** @brief What the core wants to happen next. */
enum rf215_step_result {
    RF215_STEP_IDLE = 0, /**< No operation outstanding. */
    RF215_STEP_XFER,     /**< Run the transfer, then call @ref rf215_xfer_done. */
    RF215_STEP_WAIT,     /**< Nothing to do for the reported duration. */
    RF215_STEP_EVENT,    /**< An event is reported; call @ref rf215_step again. */
};

/** @brief Operation kinds accepted by @ref rf215_submit. */
enum rf215_req_kind {
    RF215_REQ_RESET = 0, /**< Reset the chip, identify it and arm interrupts. */
    RF215_REQ_CONFIGURE, /**< Apply a PHY configuration and frequency. */
    RF215_REQ_TX,        /**< Transmit one frame. */
    RF215_REQ_RX_START,  /**< Put the receiver on air. */
    RF215_REQ_RX_STOP,   /**< Take the transceiver off air. */
    RF215_REQ_ED,        /**< Measure the energy on the channel. */
    RF215_REQ_RX_READ,   /**< Internal: read a frame the chip has received. */
    RF215_REQ_IQIF,      /**< Apply the I/Q data interface configuration (chip-wide). */
};

/** @brief Modulations the driver can configure. */
enum rf215_modulation {
    RF215_MOD_OFDM = 0, /**< MR-OFDM. */
    RF215_MOD_QPSK,     /**< MR-O-QPSK. */
    RF215_MOD_FSK,      /**< MR-FSK (2-FSK or 4-FSK, filtered). */
};

/** @brief MR-OFDM settings. */
struct rf215_phy_config_ofdm {
    uint8_t opt; /**< Bandwidth option, 0 = option 1. */
    uint8_t mcs; /**< Modulation and coding scheme, 0..6. */
};

/** @brief MR-O-QPSK settings. */
struct rf215_phy_config_qpsk {
    uint8_t fchip; /**< Chip rate: 0 = 100, 1 = 200, 2 = 1000, 3 = 2000 kchip/s. */
    uint8_t mode;  /**< Rate mode, 0..4. */
};

/** @brief MR-FSK settings. */
struct rf215_phy_config_fsk {
    uint8_t srate;     /**< Symbol rate: 0 = 50, 1 = 100, 2 = 150, 3 = 200, 4 = 300,
                            5 = 400 ksym/s. */
    uint8_t midx;      /**< Modulation index: 0 = 0.375, 1 = 0.5, then 0.75, 1.0, 1.25,
                            1.5, 1.75, 7 = 2.0. */
    uint8_t mord;      /**< Order: 0 = 2-FSK, 1 = 4-FSK (needs index >= 1.0 and
                            BT = 2.0). */
    uint8_t bt;        /**< Bandwidth-time product: 0 = 0.5, 1 = 1.0, 2 = 1.5, 3 = 2.0. */
    bool fec;          /**< Forward error correction (coded PPDU). */
    bool dw;           /**< PSDU data whitening. */
    uint16_t preamble; /**< Preamble length in octets, 1..1023. */
};

/**
 * @brief PHY configuration.
 *
 * The recommended frontend settings for each modulation are held in lib/phy/;
 * only the choices a caller makes appear here. Each modulation keeps its own
 * settings, so switching @ref modulation and back loses none of them; only the
 * group the modulation names is applied.
 */
struct rf215_phy_config {
    enum rf215_modulation modulation;  /**< Which modulation. */
    uint32_t freq_hz;                  /**< Centre frequency of channel 0. */
    uint32_t channel_spacing_hz;       /**< Channel spacing. */
    uint16_t channel;                  /**< Channel number. */
    struct rf215_phy_config_ofdm ofdm; /**< Used when modulation is MR-OFDM. */
    struct rf215_phy_config_qpsk qpsk; /**< Used when modulation is MR-O-QPSK. */
    struct rf215_phy_config_fsk fsk;   /**< Used when modulation is MR-FSK. */
    uint8_t tx_power;                  /**< PAC.TXPWR, 0 (lowest) to 31. */
    bool fcs;                          /**< Append and check the frame check sequence. */
    bool ext_lna_bypass;               /**< Bypass the external low noise amplifier. */
    uint8_t agc_map;                   /**< AGC gain map: 0 internal, 1 +9 dB, 2 +12 dB. */
    uint8_t agc_target;                /**< AGC target level, 0 (-21 dB) to 7 (-42 dB). */
    uint16_t edd_us;                   /**< Energy detect duration in microseconds, 0 for the
                                            modulation's recommended one. It is how long the
                                            channel is listened to before every transmit, and
                                            the window a frame's signal strength is averaged
                                            over. */
};

/** @brief An operation to perform. */
struct rf215_req {
    enum rf215_req_kind kind;           /**< Which operation. */
    enum rf215_band band;               /**< Transceiver it applies to. */
    const struct rf215_phy_config* phy; /**< Configuration, for CONFIGURE. */
    const uint8_t* tx_data;             /**< Frame to send, for TX. */
    uint16_t tx_len;                    /**< Its length in bytes. */
};

/** @brief Kinds of event reported by @ref rf215_step. */
enum rf215_event_kind {
    RF215_EVT_NONE = 0,    /**< No event. */
    RF215_EVT_READY,       /**< Reset completed; the chip is identified. */
    RF215_EVT_CONFIGURED,  /**< The PHY configuration has been applied. */
    RF215_EVT_TX_DONE,     /**< The frame left the antenna. */
    RF215_EVT_TX_BUSY,     /**< The channel stayed busy; the frame was not sent. */
    RF215_EVT_RX_FRAME,    /**< A frame was received into the receive buffer. */
    RF215_EVT_ED_DONE,     /**< An energy measurement completed. */
    RF215_EVT_RX_ARMED,    /**< The receiver is on air. */
    RF215_EVT_RX_STOPPED,  /**< The transceiver is off air. */
    RF215_EVT_TX_DEFERRED, /**< A frame arrived during listen-before-talk; the transmit
                                was set aside so it can be read out, and must be
                                submitted again. The transmit buffer is untouched. */
    RF215_EVT_IQIF_DONE,   /**< The I/Q interface configuration has been applied. */
    RF215_EVT_ERROR,       /**< The operation failed; see @ref rf215_event::status. */
};

/** @brief An event reported by the core. */
struct rf215_event {
    enum rf215_event_kind kind; /**< What happened. */
    enum rf215_req_kind op;     /**< Operation that produced it. */
    enum rf215_band band;       /**< Transceiver it concerns, where relevant. */
    int status;                 /**< @ref rf215_err, zero unless kind is an error. */
    uint16_t len;               /**< Bytes received, for @ref RF215_EVT_RX_FRAME. */
    int8_t rssi;                /**< dBm, for RX_FRAME and ED_DONE. */
};

/** @brief Counters kept for diagnostics. */
struct rf215_stats {
    uint32_t xfers;       /**< Transfers completed successfully. */
    uint32_t xfer_errs;   /**< Transfers that reported failure. */
    uint32_t irqs;        /**< Interrupts absorbed from the chip. */
    uint32_t timeouts;    /**< Deadlines that expired while waiting. */
    uint32_t rx_starts;   /**< Frame starts seen by the baseband. */
    uint32_t rx_ends;     /**< Frame ends seen, the ones that decoded. */
    uint32_t rx_overruns; /**< Frames overwritten while being read out. */
    uint32_t rx_rearms;   /**< Read-outs that found the receiver off air and put it back. */
    uint32_t rx_revivals; /**< Receivers the host wanted on air that nothing else was
                               going to put back, so the engine did. */
    uint32_t rx_stale;    /**< Read-outs refused because the transceiver was off air:
                               the frame the latch describes is no longer there. */
    uint32_t cca_busy;    /**< Channel assessments that found the channel busy. */
    uint32_t tx_dropped;  /**< Frames given up on after listen-before-talk. */
    uint32_t events;      /**< Events emitted to the host. */
};

struct rf215_dev;
struct rf215_op;

/**
 * @brief One step of an operation.
 *
 * Steps chain by storing their successor in @ref rf215_op::next, so a stuck
 * driver can be diagnosed by resolving that pointer to a symbol. See
 * @ref rf215_step_name.
 */
typedef enum rf215_step_result (*rf215_step_fn)(struct rf215_dev* dev,
                                                struct rf215_op* op,
                                                struct rf215_xfer* out);

/**
 * @brief State of the operation currently in progress.
 *
 * Anything that must survive a step boundary lives here rather than on a
 * stack, because a step returns to the host between bus transfers.
 */
struct rf215_op {
    rf215_step_fn next;                 /**< Next step, NULL when idle. */
    enum rf215_req_kind kind;           /**< Operation being performed. */
    enum rf215_band band;               /**< Transceiver it applies to. */
    rf215_time_t deadline;              /**< Absolute time the step's own wait expires: a
                                             transition, an interrupt, an answer. */
    rf215_time_t wake;                  /**< Absolute time a plain delay ends. Kept apart from
                                             @ref deadline so a poll loop can sleep between
                                             reads without shortening the time it has. */
    bool waiting;                       /**< A plain delay is in progress; see @ref wake. */
    rf215_step_fn resume;               /**< Where a shared sub-sequence (arming the receiver)
                                             returns to when it is done. */
    uint8_t tries;                      /**< Commands re-issued or PLL kicks so far, in a
                                             sub-sequence. */
    uint8_t polls;                      /**< Reads so far of a register being polled. */
    bool arm_moved;                     /**< The arming sequence commanded a transition, so
                                             the receiver was not on air throughout it. */
    bool ed_pending;                    /**< A single energy measurement was started and has
                                             not reported completion, so RFn_EDC may still
                                             hold single mode. */
    bool bb_off;                        /**< The baseband was switched off for a channel
                                             assessment and the chip has not switched it
                                             back on. */
    struct rf215_event evt;             /**< Event staged for the host. */
    uint8_t attempt;                    /**< Listen-before-talk attempts so far. */
    const uint8_t* tx_data;             /**< Frame being transmitted. */
    uint16_t len;                       /**< Length in flight, transmit or receive. */
    const struct rf215_phy_config* phy; /**< Configuration being applied; points at
                                             @ref phy_copy. */
    struct rf215_phy_config phy_copy;   /**< The caller's configuration as it stood at
                                             submit, so one changed while the operation
                                             runs cannot get past the checks made at its
                                             start. */
    uint8_t state;                      /**< RFn_STATE as last read, for a step that needs
                                             it after another transfer has reused the
                                             buffer. */
    uint8_t buf[9];                     /**< Transfer storage, sized for the longest register
                                             block written (RFn_CS..RFn_AGCS); never on a
                                             stack. */
};

/** @brief The radio half of a transceiver: state machine, synthesizer, front end. */
struct rf215_radio {
    uint8_t irqs; /**< Pending radio interrupts. */
    bool armed;   /**< RFn_STATE was read back as RX since the last thing that could
                       have taken it out. */
};

/** @brief The baseband half of a transceiver: framing, frame buffers, assessment. */
struct rf215_baseband {
    uint8_t irqs;               /**< Pending baseband interrupts. */
    uint8_t* rx_buf;            /**< Where received frames are put. */
    uint16_t rx_cap;            /**< Capacity of that buffer. */
    rf215_time_t rx_start_time; /**< When the outstanding frame start was seen. */
    bool rx_active;             /**< A frame start was seen and neither a frame end nor
                                     an AGC release since. */
    bool fcs;                   /**< A frame check sequence is appended and checked. */
    uint8_t pc_value;           /**< Last BBCn_PC written, for toggling the baseband. */
    uint8_t amedt;              /**< BBCn_AMEDT as last written. */
    bool amedt_valid;           /**< @ref amedt is what the chip holds; cleared by a
                                     reset and by any failed or abandoned operation. */
    uint8_t amcs;               /**< BBCn_AMCS as last written. */
    bool amcs_valid;            /**< @ref amcs is what the chip holds; cleared as
                                     @ref amedt_valid is. */
};

/** @brief One transceiver: a radio and a baseband on one band. */
struct rf215_trx {
    enum rf215_band band;               /**< Which band. */
    const struct rf215_band_info* info; /**< Its register bases and tuning offset. */
    struct rf215_radio radio;
    struct rf215_baseband baseband;
    bool receiving;           /**< The host wants this transceiver on air. */
    rf215_time_t rearm_after; /**< Earliest a receiver the host wants, but which nothing
                                   has confirmed on air, is put back. */
    uint8_t rearm_fails;      /**< Attempts at putting the receiver back since it was
                                   last confirmed on air. */
    bool cca_enabled;         /**< Assess the channel before transmitting. */
    int8_t cca_threshold;     /**< Energy above which the channel counts as busy, dBm. */
    uint8_t lbt_retries;      /**< Times to retry a busy channel. */
    uint16_t lbt_backoff_us;  /**< Base wait between retries. */
};

/** @name I/Q data interface (serial LVDS to an external baseband)
 * @{ */
/** RF_IQIFC1.CHPM - which parts of the chip are in use. */
enum rf215_chip_mode {
    RF215_CHIP_MODE_BBRF = 0,   /**< Radios and both basebands, I/Q interface off (normal). */
    RF215_CHIP_MODE_RF = 1,     /**< Radios only, I/Q interface on: an external baseband
                                     drives both transceivers. */
    RF215_CHIP_MODE_BBRF09 = 4, /**< Baseband 1 on, baseband 0 off; sub-GHz over I/Q. */
    RF215_CHIP_MODE_BBRF24 = 5, /**< Baseband 0 on, baseband 1 off; 2.4 GHz over I/Q. */
};

/** RF_IQIFC0 and RF_IQIFC1, as the datasheet lays them out (section 4.5.8). */
struct rf215_iq_config {
    enum rf215_chip_mode chip_mode; /**< Which transceivers hand their I/Q stream out. */
    uint8_t drive_ma;               /**< LVDS driver current: 0 = 1 mA .. 3 = 4 mA. */
    uint8_t common_mode;            /**< Common mode voltage: 0 = 150 mV .. 3 = 300 mV. */
    bool common_mode_1v2;           /**< Use the 1.2 V IEEE 1596 common mode instead. */
    bool embedded_tx_control;       /**< TX start/stop from I_DATA[0] rather than SPI. */
    bool external_loopback;         /**< Feed TXD straight back to RXD (interface test). */
    uint8_t skew;                   /**< RXCLK to RXD alignment: 0 = 1.9 ns .. 3 = 4.9 ns. */
};

/** RF_IQIFC0..2 as last read back after applying the configuration. */
struct rf215_iq_status {
    bool sync_failure;      /**< IQIFC0.SF: the incoming stream is not synchronised. */
    bool receiver_failsafe; /**< IQIFC1.FAILSF: no LVDS driver on the TX link. */
    bool synchronised;      /**< IQIFC2.SYNC: a data word was received in sync. */
};
/** @} */

/**
 * @brief A RF215 device.
 *
 * Allocated by the host and initialised with @ref rf215_init. All fields are
 * private to the core except @ref part_number and @ref version, which are
 * valid once @ref RF215_EVT_READY has been reported.
 */
struct rf215_dev {
    enum rf215_pn part_number; /**< Identified part, valid after reset. */
    uint8_t version;           /**< Chip version, valid after reset. */

    /** @private */
    struct rf215_port port;
    /** @private */
    struct rf215_trx trx[RF215_TRX_COUNT];
    /** @private One independent state machine per transceiver. */
    struct rf215_op op[RF215_TRX_COUNT];
    /** @private Which transceiver owns the outstanding transfer. */
    uint8_t active;
    /** @private Round-robin cursor, so neither transceiver starves the other. */
    uint8_t next_trx;
    /** @private */
    struct rf215_stats stats;
    /** @private Set from interrupt context, cleared by the core. */
    volatile uint8_t irq_raised;
    /** @private Which transfer is outstanding, if any. */
    uint8_t xfer_owner;
    /** @private Burst-read buffer for the four interrupt status registers. */
    uint8_t irq_buf[4];
    /** @private Backoff jitter state; see @ref rf215_seed_backoff. */
    uint32_t backoff_rand;
    /** @private Status reads that failed in a row, and when the next may go. */
    uint8_t irq_errs;
    /** @private */
    rf215_time_t irq_retry_at;
    /** @private Status reads in a row that reported nothing at all. */
    uint8_t irq_empty;
    /** @private The chip reported a wake-up nobody asked for. */
    bool chip_reset;
    /** @private RF_CFG.DRV, the digital pad drive strength. */
    uint8_t pad_drive;
    /** @private I/Q interface configuration, applied by RF215_REQ_IQIF. */
    struct rf215_iq_config iq;
    /** @private What the interface registers reported after the last apply. */
    struct rf215_iq_status iq_status;
};

/******************************************************************************/
/** Public Functions **********************************************************/

/**
 * @brief Initialise a device.
 *
 * Does not touch the chip; submit @ref RF215_REQ_RESET to do that.
 *
 * @param dev   Device to initialise.
 * @param port  Platform operations. Copied into the device.
 *
 * @return @ref RF215_OK, or @ref RF215_ERR_INVAL if a required callback is
 *         missing.
 */
int rf215_init(struct rf215_dev* dev, const struct rf215_port* port);

/**
 * @brief Begin an operation.
 *
 * @param dev  Device.
 * @param req  Operation to perform.
 *
 * @return @ref RF215_OK, @ref RF215_ERR_BUSY if one is already in progress, or
 *         @ref RF215_ERR_INVAL for an unknown request.
 */
int rf215_submit(struct rf215_dev* dev, const struct rf215_req* req);

/**
 * @brief Advance the state machine.
 *
 * @param dev        Device.
 * @param out_xfer   Filled when @ref RF215_STEP_XFER is returned.
 * @param out_delay  Filled when @ref RF215_STEP_WAIT is returned. Zero means
 *                   the core is waiting for an interrupt with no deadline; a
 *                   deadline that has already passed is reported as one
 *                   microsecond, never as zero.
 * @param out_evt    Filled when @ref RF215_STEP_EVENT is returned.
 *
 * @return What the host should do next.
 */
enum rf215_step_result rf215_step(struct rf215_dev* dev,
                                  struct rf215_xfer* out_xfer,
                                  rf215_micros_t* out_delay,
                                  struct rf215_event* out_evt);

/**
 * @brief Report completion of the transfer the core last asked for.
 *
 * @param dev     Device.
 * @param status  @ref RF215_OK, or a negative @ref rf215_err. A failure aborts
 *                the operation and raises @ref RF215_EVT_ERROR.
 */
void rf215_xfer_done(struct rf215_dev* dev, int status);

/**
 * @brief Record that the chip raised its interrupt line.
 *
 * Safe to call from interrupt context. Performs no bus access: the core reads
 * the interrupt status registers on its next step.
 *
 * @param dev  Device.
 */
void rf215_notify_irq(struct rf215_dev* dev);

/**
 * @brief Name of the step the device is currently parked on.
 *
 * Intended for diagnostics, for instance a debugfs file or an error log after
 * a timeout.
 *
 * @param dev   Device.
 * @param band  Transceiver to report on.
 *
 * @return A static string, "idle" when no operation is in progress, or
 *         "unknown" for a step with no registered name.
 */
const char* rf215_step_name(const struct rf215_dev* dev, enum rf215_band band);

/**
 * @brief Read the diagnostic counters.
 *
 * @param dev  Device.
 *
 * @return Pointer to the device's counters, valid for the device's lifetime,
 *         or NULL if @p dev is NULL.
 */
const struct rf215_stats* rf215_stats(const struct rf215_dev* dev);

/**
 * @brief Give the core somewhere to put received frames.
 *
 * A frame that arrives with no operation running is read out by the core on
 * its own and reported as @ref RF215_EVT_RX_FRAME. Without a buffer the frame
 * is dropped, so set one before arming the receiver. The buffer must stay
 * valid and is reused for every frame, so the host must consume one before
 * stepping again.
 *
 * @param dev   Device.
 * @param band  Transceiver the buffer belongs to.
 * @param buf   Destination buffer, owned by the host.
 * @param cap   Its capacity in bytes.
 */
void rf215_set_rx_buffer(struct rf215_dev* dev, enum rf215_band band, uint8_t* buf, uint16_t cap);

/**
 * @brief Abandon whatever a transceiver is doing.
 *
 * For a host that has given up waiting. Without it the core goes on believing
 * the operation is running and refuses every later one for that transceiver -
 * including the receive an interface needs to come back up - so one unanswered
 * operation would cost the transceiver rather than a frame.
 *
 * The chip is not touched: it is left to finish or to time out on its own, and
 * the next operation puts it back into a known state. Any event the abandoned
 * operation would have reported is lost.
 *
 * @param dev   Device.
 * @param band  Transceiver to release.
 */
void rf215_abort(struct rf215_dev* dev, enum rf215_band band);

/**
 * @brief Set the drive strength of the chip's digital output pads.
 *
 * RF_CFG.DRV: 0 = 2 mA, 1 = 4 mA, 2 = 6 mA, 3 = 8 mA. It governs how hard the
 * chip drives MISO and the interrupt line, so it decides how fast a host can
 * clock data out of it: a weak pad into a capacitive track cannot make its
 * edges in time and reads come back corrupt, which looks like a bus that
 * cannot take the clock rather than a pad that needs more current.
 *
 * Takes effect at the next reset, so set it before submitting one.
 *
 * @param dev    Device.
 * @param drive  Field value, clamped to 0..3.
 */
void rf215_set_pad_drive(struct rf215_dev* dev, uint8_t drive);

/**
 * @brief Choose the I/Q data interface settings for the next
 *        @ref RF215_REQ_IQIF.
 *
 * Chip-wide: both transceivers must be off air (TRXOFF) when the chip mode
 * changes, section 4.5.3. Nothing is written to the chip until the request is
 * submitted.
 *
 * @param dev  Device.
 * @param iq   Settings to apply. Copied into the device.
 */
void rf215_set_iq_config(struct rf215_dev* dev, const struct rf215_iq_config* iq);

/**
 * @brief Seed the listen-before-talk backoff jitter.
 *
 * Two nodes that collide have to back off by different amounts, which they
 * only do if their sequences differ. Give each device something of its own.
 *
 * @param dev   Device.
 * @param seed  Any value; zero selects the built-in default.
 */
void rf215_seed_backoff(struct rf215_dev* dev, uint32_t seed);

/**
 * @brief Whether a transceiver has an operation in progress.
 *
 * That includes the operations the core starts for itself: the read-out of a
 * received frame and putting a receiver back on air.
 *
 * @param dev   Device.
 * @param band  Transceiver to ask about.
 *
 * @return True while an operation holds the transceiver's slot, so that
 *         @ref rf215_submit would answer @ref RF215_ERR_BUSY.
 */
bool rf215_busy(const struct rf215_dev* dev, enum rf215_band band);

/**
 * @brief Whether a transceiver is in the middle of a transmit.
 *
 * A transmit holds a pointer to the caller's frame for as long as it runs, so
 * a host must not free the frame while this is true.
 *
 * @param dev   Device.
 * @param band  Transceiver to ask about.
 *
 * @return True while a @ref RF215_REQ_TX operation is in progress.
 */
bool rf215_transmitting(const struct rf215_dev* dev, enum rf215_band band);

/**
 * @brief Whether the interrupt line is likely still asserted after a step.
 *
 * True while a status read is owed - one failed and is waiting to be retried -
 * and while a run of reads has come back with nothing in them, which is a line
 * stuck high rather than a chip with something to say. A host on a level
 * triggered line should mask it for a while instead of returning into it.
 *
 * @param dev  Device.
 *
 * @return True if the line should be treated as still asserted.
 */
bool rf215_irq_unserviced(const struct rf215_dev* dev);

/**
 * @brief Whether the chip reset itself since this was last asked.
 *
 * A wake-up outside a reset operation means the chip lost its supply or was
 * reset behind the driver's back: every register is at its default and it
 * will raise no further interrupt. The host should reset and configure again.
 * Reading the flag clears it.
 *
 * @param dev  Device.
 *
 * @return True once for each unexpected chip reset seen.
 */
bool rf215_take_chip_reset(struct rf215_dev* dev);

/**
 * @brief Interface status read back by the last @ref RF215_REQ_IQIF.
 *
 * @param dev  Device.
 *
 * @return Pointer to the status, valid for the device's lifetime, or NULL if
 *         @p dev is NULL. All flags are false until a request has completed.
 */
const struct rf215_iq_status* rf215_iq_status(const struct rf215_dev* dev);

/**
 * @brief Configure clear channel assessment and listen-before-talk.
 *
 * With assessment enabled the chip measures the channel and only transmits if
 * it reads idle, which is what regulatory listen-before-talk requires in the
 * 863-870 MHz band. A busy channel is retried @p retries times, waiting a
 * little longer after each attempt, before the frame is reported as
 * @ref RF215_EVT_TX_BUSY and dropped.
 *
 * @param dev         Device.
 * @param band        Transceiver these settings apply to.
 * @param enabled     Whether to assess the channel at all.
 * @param threshold   Energy above which the channel counts as busy, in dBm.
 *                    A value near the strongest expected neighbour almost
 *                    never defers; one just above the noise floor shares the
 *                    channel with peers that can actually be heard.
 * @param retries     Attempts after the first before giving up.
 * @param backoff_us  Base wait between attempts, in microseconds. Waits are
 *                    capped at @ref RF215_CCA_BACKOFF_US_MAX; zero selects
 *                    @ref RF215_CCA_DEFAULT_BACKOFF_US.
 */
void rf215_set_cca(struct rf215_dev* dev,
                   enum rf215_band band,
                   bool enabled,
                   int8_t threshold,
                   uint8_t retries,
                   uint16_t backoff_us);

#endif /* KAONIC_DRIVERS_RF215_API_H__ */
