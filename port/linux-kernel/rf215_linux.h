/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared declarations for the RF215 Linux host.
 *
 * The host is split by the direction it faces: rf215_port.c drives the chip,
 * rf215_netdev.c and rf215_mac802154.c present a transceiver upward as a raw
 * network device or an 802.15.4 PHY, rf215_sysfs.c exposes its settings,
 * rf215_debugfs.c shows what it is doing, and rf215_bind.c ties a probed SPI
 * device to the rest.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

#ifndef KAONIC_RF215_LINUX_H__
#define KAONIC_RF215_LINUX_H__

/******************************************************************************/
/** Includes ******************************************************************/

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/ieee802154.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/rtmutex.h>
#include <linux/rtnetlink.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <net/mac802154.h>

#include "rf215/rf215_api.h"
#include "rf215/rf215_version.h"

/******************************************************************************/
/** Defines *******************************************************************/

#define RF215_SPI_ADDR_MASK 0x3fff
#define RF215_SPI_WRITE_FLAG 0x8000
#define RF215_SPI_HDR_LEN 2

/* tSPI_5 (Table 10-29, datasheet rev. E): the least idle time between two bytes.
 * Back to back it caps the clock at 8 MHz; above that the controller inserts
 * the gap as whole clocks (word_delay), four at 25 MHz. The clock period
 * already covers 40 ns of it, but the full figure is kept: a byte corrupted on
 * the bus is not caught by the FCS, which the chip checks before the read-out. */
#define RF215_SPI_INTERBYTE_NS 125

/* Accesses up to this long go out as one transfer, address and data together;
 * covers every access but the frame buffers. */
#define RF215_SPI_SMALL_MAX 16

/* Waits up to this long are slept inline; longer ones go to an hrtimer that
 * queues the next pass. Below the shortest CCA backoff on purpose: an inline
 * sleep holds the engine lock and would stall the other transceiver. No slack
 * on either kind: on this board slack is only added delay. */
#define RF215_INLINE_WAIT_US 200u

/* Backstop for an operation the chip never answers. */
#define RF215_OP_TIMEOUT_MS 500u

/* How long a stuck interrupt line (failing status reads, or asserted with
 * nothing to report) stays masked. */
#define RF215_IRQ_REST_MS 100u

/* Sized for the hardware, so an oversized frame is diagnosed, not truncated. */
#define RF215_RX_BUF_LEN 2048

/* Holds one register value read back for the diagnostics dump. */
#define RF215_SCRATCH_LEN 4

/* Largest frame the baseband can hold, frame check sequence included. The
 * 802.15.4 stack caps this at 123 bytes; the raw interface does not. */
#define RF215_FRAME_LEN_MAX 2047

/* The two transceivers' register blocks sit 0x100 apart. */
#define RF215_TRX_REG_STRIDE 0x0100

/* The 802.15.4 PHY reports its configured channel on this page. */
#define RF215_PAGE 0

/******************************************************************************/
/** Types *********************************************************************/

struct rf215_spi;

/* One transceiver and its interface, whichever front end presents it. One the
 * device tree disables keeps its slot here but gets no interface, no settings
 * directory and no receive buffer; see rf215_iface_enabled(). */
struct rf215_iface {
    struct rf215_spi* rf;
    enum rf215_band band;

    struct net_device* ndev;
    struct ieee802154_hw* hw;
    /* The rf215/ settings directory under the raw interface. */
    struct kobject* sysfs;

    struct rf215_phy_config phy;

    /* Frame handed down by the stack, owned until the transmit completes; queued
     * with the core by the next engine pass. */
    struct sk_buff* tx_skb;
    bool tx_queued;

    /* This interface holds the bus controller awake; see rf215_bus_hold(). */
    bool bus_held;

    /* One interface operation at a time: they share the completion below. */
    struct mutex op_lock;
    struct completion op_done;
    int op_status;
    u8 ed_level;
    s8 ed_dbm;
    /* RSSI of the last received frame; the raw interface has no per-frame metadata. */
    s8 rx_rssi;

    /* A separate allocation: it is DMA-mapped, and unmapping invalidates whole
     * cache lines, which would clobber neighbouring fields of an embedded array. */
    u8* rx_buf;
};

/* How a transceiver is presented upward: a raw network interface or an
 * 802.15.4 PHY. Both drive the same core. */
struct rf215_frontend {
    const char* name;
    int (*attach)(struct rf215_iface* ifc, const char* name);
    void (*detach)(struct rf215_iface* ifc);
    void (*rx_frame)(struct rf215_iface* ifc, const u8* data, u16 len, s8 rssi);
    void (*tx_done)(struct rf215_iface* ifc, int status);
};

struct rf215_spi {
    struct spi_device* spi;
    struct gpio_desc* reset_gpio;

    /* The engine lock: every core call and every engine pass runs under it.
     * Priority inheriting, as the real-time interrupt thread waits on it. */
    struct rt_mutex lock;

    /* Serialises settings changes, so a refused one is put back before the
     * next is applied. Taken before lock; RTNL, where held, before both. */
    struct mutex cfg_lock;

    /* One SCHED_FIFO worker per chip runs the engine passes; the timer fires
     * waits at microsecond resolution instead of the next jiffy. */
    struct kthread_worker* worker;
    struct kthread_work work;
    struct hrtimer timer;

    /* Unmasks the interrupt line after it was masked as stuck. */
    struct kthread_delayed_work irq_work;
    bool irq_off;

    /* Runs the core's recovery of a chip that stopped answering. */
    struct work_struct recover;

    /* Set for teardown: nothing queues a pass or arms the timer after it. */
    bool stopping;

    struct dentry* debugfs;
    const struct rf215_frontend* fe;

    /* Bus buffers, separate allocations like rx_buf: the address and a short
     * access's data go out from hdr, the answer comes back into small. */
    u8* hdr;
    u8* small;
    u8* scratch;

    struct rf215_iface iface[RF215_TRX_COUNT];
    struct rf215_dev core;
};

/******************************************************************************/
/** Rf215_port.c: drives the chip *********************************************/

void rf215_spi_set_reset(void* ctx, bool asserted);
rf215_time_t rf215_spi_now(void* ctx);
void rf215_spi_log(void* ctx, enum rf215_log_level lvl, const char* msg);

/* One transfer on the bus; the register dump uses it directly. */
int rf215_spi_xfer(struct rf215_spi* rf, const struct rf215_xfer* xfer);

const char* rf215_iface_name(const struct rf215_iface* ifc);

void rf215_work_fn(struct kthread_work* work);
/* Unmasks an interrupt line that was masked as stuck. */
void rf215_irq_work_fn(struct kthread_work* work);
/* Resets and configures a chip that stopped answering. */
void rf215_recover_fn(struct work_struct* work);
/* Fires the engine pass a wait was arming. */
enum hrtimer_restart rf215_timer_fn(struct hrtimer* timer);
irqreturn_t rf215_irq_thread(int irq, void* data);
/* Queues an engine pass. */
void rf215_kick(struct rf215_spi* rf);

/* Submits one request and waits for it to finish. */
int rf215_run_op(struct rf215_iface* ifc, const struct rf215_req* req);
int rf215_start_receive(struct rf215_iface* ifc);
int rf215_stop_receive(struct rf215_iface* ifc);
int rf215_configure(struct rf215_iface* ifc);
int rf215_apply_iq(struct rf215_iface* ifc);
/* Gives up the frame in flight, if any, and frees it. */
void rf215_drop_tx(struct rf215_iface* ifc);

/******************************************************************************/
/** Rf215_netdev.c: presents a transceiver as a raw network device ************/

const struct rf215_frontend* rf215_raw_frontend(void);

struct rf215_iface* rf215_from_ndev(struct net_device* ndev);

/* Largest payload: the frame less the FCS, when it is on. */
unsigned int rf215_raw_mtu(const struct rf215_iface* ifc);
/* Makes the MTU limit follow the FCS setting. Caller holds RTNL. */
void rf215_raw_update_mtu(struct rf215_iface* ifc);

/******************************************************************************/
/** Rf215_mac802154.c: presents a transceiver as an 802.15.4 PHY **************/

const struct rf215_frontend* rf215_mac_frontend(void);

/******************************************************************************/
/** Rf215_sysfs.c: settings ***************************************************/

/* Creates rf215/ and its sub-directories under a registered interface. */
int rf215_sysfs_attach(struct rf215_iface* ifc);
/* Removes them; no attribute handler runs after this returns. */
void rf215_sysfs_detach(struct rf215_iface* ifc);

/******************************************************************************/
/** Rf215_debugfs.c: diagnostics **********************************************/

void rf215_debugfs_init(struct rf215_spi* rf);

/******************************************************************************/
/** Helpers *******************************************************************/

/* Whether the chip is presented as 802.15.4 PHYs rather than raw interfaces. */
static inline bool rf215_is_mac_fe(const struct rf215_spi* rf) {
    return rf->fe == rf215_mac_frontend();
}

/* Whether the board uses this transceiver. Valid once the core is initialised. */
static inline bool rf215_iface_enabled(const struct rf215_iface* ifc) {
    return rf215_band_enabled(&ifc->rf->core, ifc->band);
}

/* The interface the chip-wide operations (reset, I/Q interface, recovery) run
 * on: the first enabled one. Probe fails when there is none. */
static inline struct rf215_iface* rf215_chip_iface(struct rf215_spi* rf) {
    return &rf->iface[rf215_band_enabled(&rf->core, RF215_BAND_09) ? RF215_BAND_09 : RF215_BAND_24];
}

#endif /* KAONIC_RF215_LINUX_H__ */
