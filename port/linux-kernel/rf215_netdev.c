// SPDX-License-Identifier: GPL-2.0
/*
 * Presents a transceiver as a plain network interface.
 *
 * The interface carries whole PHY frames with no 802.15.4 framing, which is
 * what lets it use the transceiver's real 2047 byte limit instead of the 123
 * the 802.15.4 stack imposes. It behaves as a layer 3 point-to-point device,
 * so IP can be configured directly on it. The 802.15.4 presentation of the
 * same transceiver lives in rf215_mac802154.c; both drive the same core.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Forward Declarations ******************************************************/

static int rf215_raw_open(struct net_device* ndev);
/* Takes the transceiver off air. */
static int rf215_raw_stop(struct net_device* ndev);
/* Queues one frame, stopping the queue until it is done. */
static netdev_tx_t rf215_raw_xmit(struct sk_buff* skb, struct net_device* ndev);
/* net_device defaults. */
static void rf215_raw_setup(struct net_device* ndev);
/* Registers the raw front end for this transceiver. */
static int rf215_raw_attach(struct rf215_iface* ifc, const char* name);
/* Unregisters the raw front end. */
static void rf215_raw_detach(struct rf215_iface* ifc);
/* Hands a received frame to the network stack. */
static void rf215_raw_rx_frame(struct rf215_iface* ifc, const u8* data, u16 len, s8 rssi);
/* Frees the frame in flight and restarts the queue. */
static void rf215_raw_tx_done(struct rf215_iface* ifc, int status);

/******************************************************************************/
/** Data **********************************************************************/

static const struct net_device_ops rf215_raw_netdev_ops = {
    .ndo_open = rf215_raw_open,
    .ndo_stop = rf215_raw_stop,
    .ndo_start_xmit = rf215_raw_xmit,
};

static const struct rf215_frontend rf215_frontend_raw = {
    .name = "raw",
    .attach = rf215_raw_attach,
    .detach = rf215_raw_detach,
    .rx_frame = rf215_raw_rx_frame,
    .tx_done = rf215_raw_tx_done,
};

/******************************************************************************/
/** Public Functions **********************************************************/

const struct rf215_frontend* rf215_raw_frontend(void) {
    return &rf215_frontend_raw;
}

struct rf215_iface* rf215_from_ndev(struct net_device* ndev) {
    return *(struct rf215_iface**)netdev_priv(ndev);
}

unsigned int rf215_raw_mtu(const struct rf215_iface* ifc) {
    return RF215_FRAME_LEN_MAX - (ifc->phy.fcs ? RF215_FCS_LEN : 0);
}

/* An MTU left at the old limit follows the new one; a smaller one the user set is kept. */
void rf215_raw_update_mtu(struct rf215_iface* ifc) {
    struct net_device* ndev = ifc->ndev;
    unsigned int limit = rf215_raw_mtu(ifc);
    bool follow;

    ASSERT_RTNL();

    if (!ndev || (ndev->max_mtu == limit)) {
        return;
    }

    follow = (ndev->mtu == ndev->max_mtu) || (ndev->mtu > limit);
    ndev->max_mtu = limit;

    if (follow) {
        dev_set_mtu(ndev, (int)limit);
    }
}

/******************************************************************************/
/** Static Functions **********************************************************/

static int rf215_raw_open(struct net_device* ndev) {
    struct rf215_iface* ifc = rf215_from_ndev(ndev);
    int ret;

    ret = rf215_start_receive(ifc);
    if (ret) {
        return ret;
    }

    netif_start_queue(ndev);

    return 0;
}

static int rf215_raw_stop(struct net_device* ndev) {
    struct rf215_iface* ifc = rf215_from_ndev(ndev);
    int ret;

    netif_stop_queue(ndev);

    /* Drop the frame in flight first: left behind it refuses every transmit after the next
     * open, and a transmit waiting out a busy channel outlasts the stop below. */
    rf215_drop_tx(ifc);

    ret = rf215_stop_receive(ifc);

    return ret;
}

/* Stops the queue for each frame instead of refusing the skb, so the stack paces itself. */
static netdev_tx_t rf215_raw_xmit(struct sk_buff* skb, struct net_device* ndev) {
    struct rf215_iface* ifc = rf215_from_ndev(ndev);

    netif_stop_queue(ndev);

    if (ifc->tx_skb) {
        return NETDEV_TX_BUSY;
    }

    ifc->tx_skb = skb;

    rf215_kick(ifc->rf);

    return NETDEV_TX_OK;
}

static void rf215_raw_setup(struct net_device* ndev) {
    ndev->netdev_ops = &rf215_raw_netdev_ops;
    ndev->needs_free_netdev = true;

    ndev->type = ARPHRD_NONE;
    ndev->flags = IFF_NOARP | IFF_POINTOPOINT;
    ndev->addr_len = 0;
    ndev->hard_header_len = 0;
    ndev->mtu = RF215_FRAME_LEN_MAX;
    ndev->min_mtu = 1;
    ndev->max_mtu = RF215_FRAME_LEN_MAX;
    ndev->tx_queue_len = 64;
}

static int rf215_raw_attach(struct rf215_iface* ifc, const char* name) {
    struct net_device* ndev;
    int ret;

    ndev = alloc_netdev(sizeof(struct rf215_iface*), name, NET_NAME_ENUM, rf215_raw_setup);
    if (!ndev) {
        return -ENOMEM;
    }

    *(struct rf215_iface**)netdev_priv(ndev) = ifc;
    ndev->mtu = rf215_raw_mtu(ifc);
    ndev->max_mtu = ndev->mtu;
    SET_NETDEV_DEV(ndev, &ifc->rf->spi->dev);

    /* Which of the chip's two transceivers this is: readable as dev_port the moment the
     * interface appears, which is what the udev naming rule matches on. */
    ndev->dev_port = (unsigned short)ifc->band;

    /* Published before registering: a transmit can complete as soon as the interface is
     * visible, and tx_done reads it. */
    ifc->ndev = ndev;

    ret = register_netdev(ndev);
    if (ret) {
        ifc->ndev = NULL;
        free_netdev(ndev);
        return ret;
    }

    ret = rf215_sysfs_attach(ifc);
    if (ret) {
        unregister_netdev(ndev);
        ifc->ndev = NULL;
        return ret;
    }

    return 0;
}

static void rf215_raw_detach(struct rf215_iface* ifc) {
    if (!ifc->ndev) {
        return;
    }

    /* First: the settings handlers use the interface. */
    rf215_sysfs_detach(ifc);

    unregister_netdev(ifc->ndev);
    ifc->ndev = NULL;

    /* Removing the device with a frame in flight would otherwise leak it. */
    rf215_drop_tx(ifc);
}

static void rf215_raw_rx_frame(struct rf215_iface* ifc, const u8* data, u16 len, s8 rssi) {
    struct net_device* ndev = ifc->ndev;
    struct sk_buff* skb;

    (void)rssi;

    /* The engine runs from probe until remove; the interface does not. */
    if (!ndev) {
        return;
    }

    skb = netdev_alloc_skb(ndev, len);
    if (!skb) {
        ndev->stats.rx_dropped++;
        return;
    }

    skb_put_data(skb, data, len);
    skb_reset_mac_header(skb);
    skb->dev = ndev;
    /* The frame check sequence covers the link, when it is on at all; it says
     * nothing about the transport checksums, which the stack must verify. */
    skb->ip_summed = CHECKSUM_NONE;

    /* No link header: the IP version nibble selects the protocol, as on a tunnel device. */
    switch (data[0] >> 4) {
        case 4:
            skb->protocol = htons(ETH_P_IP);
            break;
        case 6:
            skb->protocol = htons(ETH_P_IPV6);
            break;
        default:
            skb->protocol = htons(ETH_P_802_2);
            break;
    }

    ndev->stats.rx_packets++;
    ndev->stats.rx_bytes += len;

    netif_rx(skb);
}

static void rf215_raw_tx_done(struct rf215_iface* ifc, int status) {
    struct net_device* ndev = ifc->ndev;
    struct sk_buff* skb = ifc->tx_skb;

    if (!skb) {
        return;
    }

    if (status == -EBUSY) {
        ndev->stats.tx_dropped++;
    } else if (status) {
        ndev->stats.tx_errors++;
    } else {
        ndev->stats.tx_packets++;
        ndev->stats.tx_bytes += skb->len;
    }

    ifc->tx_skb = NULL;

    /* Bracketed so the transmit softirq runs here: the next frame reaches rf215_raw_xmit()
     * before this returns and is submitted in the same engine pass. */
    local_bh_disable();
    netif_wake_queue(ndev);
    local_bh_enable();

    if (status) {
        dev_kfree_skb_any(skb);
    } else {
        dev_consume_skb_any(skb);
    }
}
