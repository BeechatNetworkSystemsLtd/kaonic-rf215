// SPDX-License-Identifier: GPL-2.0
/*
 * Presents a transceiver as an IEEE 802.15.4 PHY through mac802154.
 *
 * The stack owns framing, addressing and the 123 byte frame limit; the
 * transceiver only carries frames and reports channel energy. The raw
 * presentation of the same transceiver lives in rf215_netdev.c; both drive
 * the same core.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Forward Declarations ******************************************************/

static struct rf215_iface* rf215_from_hw(struct ieee802154_hw* hw);
/* Brings the receiver on air. */
static int rf215_mac_start(struct ieee802154_hw* hw);
/* Takes the transceiver off air. */
static void rf215_mac_stop(struct ieee802154_hw* hw);
/* Queues a frame for the engine thread. */
static int rf215_mac_xmit_async(struct ieee802154_hw* hw, struct sk_buff* skb);
/* Energy detect for a channel scan. */
static int rf215_mac_ed(struct ieee802154_hw* hw, u8* level);
/* Re-applies the configuration for the (single) channel. */
static int rf215_mac_set_channel(struct ieee802154_hw* hw, u8 page, u8 channel);
/* Maps dBm onto PAC.TXPWR and re-configures. */
static int rf215_mac_set_txpower(struct ieee802154_hw* hw, s32 mbm);
/* Registers the wpan front end for this transceiver. */
static int rf215_mac_attach(struct rf215_iface* ifc, const char* name);
/* Unregisters the wpan front end. */
static void rf215_mac_detach(struct rf215_iface* ifc);
/* Hands a received frame to mac802154. */
static void rf215_mac_rx_frame(struct rf215_iface* ifc, const u8* data, u16 len, s8 rssi);
/* Completes or fails the frame in flight. */
static void rf215_mac_tx_done(struct rf215_iface* ifc, int status);

/******************************************************************************/
/** Data **********************************************************************/

static const struct ieee802154_ops rf215_mac_ops = {
    .owner = THIS_MODULE,
    .start = rf215_mac_start,
    .stop = rf215_mac_stop,
    .xmit_async = rf215_mac_xmit_async,
    .ed = rf215_mac_ed,
    .set_channel = rf215_mac_set_channel,
    .set_txpower = rf215_mac_set_txpower,
};

static const struct rf215_frontend rf215_frontend_wpan = {
    .name = "wpan",
    .attach = rf215_mac_attach,
    .detach = rf215_mac_detach,
    .rx_frame = rf215_mac_rx_frame,
    .tx_done = rf215_mac_tx_done,
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_frontend* rf215_mac_frontend(void) {
    return &rf215_frontend_wpan;
}

/******************************************************************************/
/** Static Functions **********************************************************/

static struct rf215_iface* rf215_from_hw(struct ieee802154_hw* hw) {
    return *(struct rf215_iface**)hw->priv;
}

static int rf215_mac_start(struct ieee802154_hw* hw) {
    return rf215_start_receive(rf215_from_hw(hw));
}

static void rf215_mac_stop(struct ieee802154_hw* hw) {
    rf215_stop_receive(rf215_from_hw(hw));
}

/*
 * Called from soft interrupt context, where the bus cannot be used, so the
 * frame is handed to the work item.
 */
static int rf215_mac_xmit_async(struct ieee802154_hw* hw, struct sk_buff* skb) {
    struct rf215_iface* ifc = rf215_from_hw(hw);

    if (ifc->tx_skb) {
        return -EBUSY;
    }

    ifc->tx_skb = skb;

    rf215_kick(ifc->rf);

    return 0;
}

static int rf215_mac_ed(struct ieee802154_hw* hw, u8* level) {
    struct rf215_iface* ifc = rf215_from_hw(hw);
    const struct rf215_req req = {
        .kind = RF215_REQ_ED,
        .band = ifc->band,
    };
    int ret;

    ret = rf215_run_op(ifc, &req);
    if (ret) {
        return ret;
    }

    *level = ifc->ed_level;

    return 0;
}

static int rf215_mac_set_channel(struct ieee802154_hw* hw, u8 page, u8 channel) {
    struct rf215_iface* ifc = rf215_from_hw(hw);
    int ret;

    if (page != RF215_PAGE || channel != ifc->phy.channel) {
        return -EINVAL;
    }

    /* Configuring takes the transceiver off air and puts a receiver back
     * itself. */
    mutex_lock(&ifc->rf->cfg_lock);
    ret = rf215_configure(ifc);
    mutex_unlock(&ifc->rf->cfg_lock);

    return ret;
}

static int rf215_mac_set_txpower(struct ieee802154_hw* hw, s32 mbm) {
    struct rf215_iface* ifc = rf215_from_hw(hw);
    s32 dbm = mbm / 100;
    int ret;

    /* PAC.TXPWR is a 0..31 setting rather than decibels, spanning roughly
     * -17 dBm at 0 to +14 dBm at 31. */
    dbm = clamp(dbm, -17, 14);

    mutex_lock(&ifc->rf->cfg_lock);
    ifc->phy.tx_power = (u8)(dbm + 17);
    ret = rf215_configure(ifc);
    mutex_unlock(&ifc->rf->cfg_lock);

    return ret;
}

static int rf215_mac_attach(struct rf215_iface* ifc, const char* name) {
    struct ieee802154_hw* hw;
    int ret;

    (void)name;

    hw = ieee802154_alloc_hw(sizeof(struct rf215_iface*), &rf215_mac_ops);
    if (!hw) {
        return -ENOMEM;
    }

    *(struct rf215_iface**)hw->priv = ifc;
    ifc->hw = hw;
    hw->parent = &ifc->rf->spi->dev;

    /* One channel: the one the transceiver is configured on. */
    hw->phy->supported.channels[RF215_PAGE] = BIT(ifc->phy.channel);
    hw->phy->current_page = RF215_PAGE;
    hw->phy->current_channel = (u8)ifc->phy.channel;
    hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

    /* The baseband appends and checks the frame check sequence itself, and
     * address filtering is left to the stack. */
    hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_RX_OMIT_CKSUM;

    ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

    ret = ieee802154_register_hw(hw);
    if (ret) {
        ieee802154_free_hw(hw);
        ifc->hw = NULL;
        return ret;
    }

    return 0;
}

static void rf215_mac_detach(struct rf215_iface* ifc) {
    if (!ifc->hw) {
        return;
    }

    ieee802154_unregister_hw(ifc->hw);
    ieee802154_free_hw(ifc->hw);
    ifc->hw = NULL;

    /* A frame still owned here belongs to nobody now. */
    rf215_drop_tx(ifc);
}

static void rf215_mac_rx_frame(struct rf215_iface* ifc, const u8* data, u16 len, s8 rssi) {
    struct sk_buff* skb;

    /* The engine runs from probe until remove; the interface does not. */
    if (!ifc->hw || len > IEEE802154_MTU) {
        return;
    }

    skb = dev_alloc_skb(len);
    if (!skb) {
        return;
    }

    skb_put_data(skb, data, len);

    /* Bracketed so the stack's tasklet runs here rather than whenever the
     * softirq thread is next scheduled. */
    local_bh_disable();
    ieee802154_rx_irqsafe(ifc->hw, skb, rf215_lqi(rssi));
    local_bh_enable();
}

static void rf215_mac_tx_done(struct rf215_iface* ifc, int status) {
    struct sk_buff* skb = ifc->tx_skb;

    if (!skb) {
        return;
    }

    /* Cleared before completing: the stack can hand down the next frame on another CPU at
     * once, and clearing afterwards would overwrite that frame's pointer. */
    ifc->tx_skb = NULL;

    /* As on receive: the queue this releases is restarted here, so the next
     * frame reaches the driver within the same engine pass. */
    local_bh_disable();

    if (status) {
        ieee802154_xmit_error(ifc->hw, skb, status);
    } else {
        ieee802154_xmit_complete(ifc->hw, skb, false);
    }

    local_bh_enable();
}
