// SPDX-License-Identifier: GPL-2.0
/*
 * Runtime settings.
 *
 * Settings under /sys/class/net/<iface>/rf215/, grouped into ofdm/, qpsk/,
 * fsk/, frontend/, cca/ and iq/. The I/Q interface settings are chip-wide, so
 * iq/ shows the same values under both of a chip's interfaces. The diagnostics
 * view of the chip and the engine lives in rf215_debugfs.c.
 *
 * Copyright (c) 2026 Beechat Network Systems Ltd.
 */

/******************************************************************************/
/** Includes ******************************************************************/

#include "rf215_linux.h"

/******************************************************************************/
/** Defines *******************************************************************/

/* An attribute whose file name is not its C identifier: "mode" exists in qpsk/ and in iq/. */
#define RF215_KOBJ_ATTR(id, fname, perm, show, store) \
    static struct kobj_attribute rf215_attr_##id = __ATTR(fname, perm, show, store)

/* A PHY setting: applied at once by a configure operation, link up or down. */
#define RF215_ATTR_RW(id, fname, member, lo, hi)                                             \
    static ssize_t id##_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) { \
        (void)attr;                                                                          \
        return sysfs_emit(buf, "%u\n", (u32)rf215_from_kobj(kobj)->phy.member);              \
    }                                                                                        \
    static ssize_t id##_store(                                                               \
        struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {    \
        struct rf215_iface* ifc = rf215_from_kobj(kobj);                                     \
        u32 value;                                                                           \
        typeof(ifc->phy.member) previous;                                                    \
        int ret;                                                                             \
                                                                                             \
        (void)attr;                                                                          \
                                                                                             \
        ret = rf215_parse(buf, (lo), (hi), &value);                                          \
        if (ret)                                                                             \
            return ret;                                                                      \
                                                                                             \
        mutex_lock(&ifc->rf->cfg_lock);                                                      \
        previous = ifc->phy.member;                                                          \
        ifc->phy.member = value;                                                             \
                                                                                             \
        ret = rf215_commit(ifc);                                                             \
        if (ret) {                                                                           \
            /* The setting the chip refused must not be readable as                          \
             * though it had taken: a 2.4 GHz frequency on a sub-GHz                         \
             * transceiver is rejected here, and reporting it back                           \
             * makes the interface look retuned when it is not.                              \
             * Configure validates before it writes, so the chip                             \
             * still holds the old setting.                                                  \
             */                                                                              \
            ifc->phy.member = previous;                                                      \
        }                                                                                    \
        mutex_unlock(&ifc->rf->cfg_lock);                                                    \
                                                                                             \
        return ret ? ret : (ssize_t)len;                                                     \
    }                                                                                        \
    RF215_KOBJ_ATTR(id, fname, 0644, id##_show, id##_store)

/* CCA settings live in the core, not the PHY config: written directly, no reconfigure. */
#define RF215_CCA_ATTR(id, fname, member, lo, hi)                                            \
    static ssize_t id##_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) { \
        const struct rf215_iface* ifc = rf215_from_kobj(kobj);                               \
        const struct rf215_trx* t = &ifc->rf->core.trx[ifc->band];                           \
                                                                                             \
        (void)attr;                                                                          \
        return sysfs_emit(buf, "%d\n", (int)(t->member));                                    \
    }                                                                                        \
    static ssize_t id##_store(                                                               \
        struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {    \
        struct rf215_iface* ifc = rf215_from_kobj(kobj);                                     \
        struct rf215_trx* t = &ifc->rf->core.trx[ifc->band];                                 \
        int value;                                                                           \
                                                                                             \
        (void)attr;                                                                          \
                                                                                             \
        if (kstrtoint(buf, 0, &value))                                                       \
            return -EINVAL;                                                                  \
        if (value < (lo) || value > (hi))                                                    \
            return -EINVAL;                                                                  \
                                                                                             \
        rt_mutex_lock(&ifc->rf->lock);                                                       \
        t->member = (typeof(t->member))value;                                                \
        rt_mutex_unlock(&ifc->rf->lock);                                                     \
                                                                                             \
        return (ssize_t)len;                                                                 \
    }                                                                                        \
    RF215_KOBJ_ATTR(id, fname, 0644, id##_show, id##_store)

/* I/Q data interface, chip-wide. Writes are refused while either link is up: the chip mode
 * may only change with both transceivers off air (section 4.5.3). */
#define RF215_IQ_ATTR(id, fname, member, lo, hi)                                             \
    static ssize_t id##_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) { \
        (void)attr;                                                                          \
        return sysfs_emit(buf, "%u\n", (u32)rf215_from_kobj(kobj)->rf->core.iq.member);      \
    }                                                                                        \
    static ssize_t id##_store(                                                               \
        struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {    \
        struct rf215_spi* rf = rf215_from_kobj(kobj)->rf;                                    \
        struct rf215_iq_config iq;                                                           \
        u32 value;                                                                           \
        int ret;                                                                             \
                                                                                             \
        (void)attr;                                                                          \
                                                                                             \
        ret = rf215_parse(buf, (lo), (hi), &value);                                          \
        if (ret)                                                                             \
            return ret;                                                                      \
                                                                                             \
        /* RTNL keeps the link state fixed from the check to the apply. */                   \
        if (!rtnl_trylock())                                                                 \
            return restart_syscall();                                                        \
                                                                                             \
        if (rf215_any_link_up(rf)) {                                                         \
            rtnl_unlock();                                                                   \
            return -EBUSY;                                                                   \
        }                                                                                    \
                                                                                             \
        mutex_lock(&rf->cfg_lock);                                                           \
        rt_mutex_lock(&rf->lock);                                                            \
        iq = rf->core.iq;                                                                    \
        iq.member = value;                                                                   \
        rf215_set_iq_config(&rf->core, &iq);                                                 \
        rt_mutex_unlock(&rf->lock);                                                          \
                                                                                             \
        ret = rf215_apply_iq(rf215_chip_iface(rf));                                          \
        mutex_unlock(&rf->cfg_lock);                                                         \
        rtnl_unlock();                                                                       \
                                                                                             \
        return ret ? ret : (ssize_t)len;                                                     \
    }                                                                                        \
    RF215_KOBJ_ATTR(id, fname, 0644, id##_show, id##_store)

/******************************************************************************/
/** Forward Declarations ******************************************************/

/* The interface a settings directory belongs to. */
static struct rf215_iface* rf215_from_kobj(struct kobject* kobj);
/* Re-applies the configuration; configure restores the receiver itself. */
static int rf215_commit(struct rf215_iface* ifc);
/* Parses an unsigned value within bounds. */
static int rf215_parse(const char* buf, u32 min, u32 max, u32* out);
/* Whether the interface is up, which some settings refuse to change under. */
static bool rf215_link_is_up(const struct rf215_iface* ifc);
/* Reports the modulation as a name. */
static ssize_t modulation_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Selects the modulation by name (ofdm, qpsk, fsk). */
static ssize_t
modulation_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len);
/* Reports whether the frame check sequence is on. */
static ssize_t fcs_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Switches the frame check sequence; the interface's MTU limit follows. */
static ssize_t
fcs_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len);
/* RSSI of the last received frame in dBm; -128 (RF215_RSSI_INVALID) when there is none or the
 * chip had no valid measurement. Always a number: readers parse one. */
static ssize_t rssi_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Reports which band this transceiver serves. */
static ssize_t band_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Reports the step the engine is at for this transceiver. */
static ssize_t state_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Runs a single energy measurement on the channel and reports it in dBm. */
static ssize_t ed_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Whether either transceiver is up; chip-wide settings need both down. */
static bool rf215_any_link_up(const struct rf215_spi* rf);
/* Reports the I/Q chip mode as a name. */
static ssize_t iq_mode_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
/* Selects the I/Q chip mode by name and applies it. */
static ssize_t
iq_mode_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len);
/* Reports the I/Q interface synchronisation status. */
static ssize_t iq_status_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);

/******************************************************************************/
/** Data **********************************************************************/

/* Top level: status, and the settings every modulation shares. */
RF215_KOBJ_ATTR(band, band, 0444, band_show, NULL);

RF215_KOBJ_ATTR(state, state, 0444, state_show, NULL);

RF215_KOBJ_ATTR(rssi, rssi, 0444, rssi_show, NULL);

/* Root only: reading it runs a measurement, which takes the baseband off the
 * channel for its duration. */
RF215_KOBJ_ATTR(ed, ed, 0400, ed_show, NULL);

RF215_KOBJ_ATTR(modulation, modulation, 0644, modulation_show, modulation_store);

RF215_KOBJ_ATTR(fcs, fcs, 0644, fcs_show, fcs_store);

/* Channel and modulation. Changing these under a live link is the caller's
 * business: the far side has to follow, and the frame on air is lost. */
RF215_ATTR_RW(frequency, frequency, freq_hz, 70000000, 3000000000u);

RF215_ATTR_RW(channel, channel, channel, 0, 65535);

RF215_ATTR_RW(channel_spacing, channel_spacing, channel_spacing_hz, 0, 6375000);

/* ofdm/ */
RF215_ATTR_RW(ofdm_opt, opt, ofdm.opt, 0, 3);

RF215_ATTR_RW(ofdm_mcs, mcs, ofdm.mcs, 0, 6);

/* qpsk/ */
RF215_ATTR_RW(qpsk_fchip, fchip, qpsk.fchip, 0, 3);

RF215_ATTR_RW(qpsk_mode, mode, qpsk.mode, 0, 4);

/* fsk/ */
RF215_ATTR_RW(fsk_srate, srate, fsk.srate, 0, 5);

RF215_ATTR_RW(fsk_midx, midx, fsk.midx, 0, 7);

RF215_ATTR_RW(fsk_mord, mord, fsk.mord, 0, 1);

RF215_ATTR_RW(fsk_bt, bt, fsk.bt, 0, 3);

RF215_ATTR_RW(fsk_fec, fec, fsk.fec, 0, 1);

RF215_ATTR_RW(fsk_dw, dw, fsk.dw, 0, 1);

RF215_ATTR_RW(fsk_preamble, preamble, fsk.preamble, 1, 1023);

/* frontend/ */
RF215_ATTR_RW(tx_power, tx_power, tx_power, 0, 31);

RF215_ATTR_RW(lna_bypass, lna_bypass, ext_lna_bypass, 0, 1);

RF215_ATTR_RW(agc_map, agc_map, agc_map, 0, 2);

RF215_ATTR_RW(agc_target, agc_target, agc_target, 0, 7);

/* cca/ */
RF215_CCA_ATTR(cca_enable, enable, cca_enabled, 0, 1);

RF215_CCA_ATTR(cca_threshold, threshold, cca_threshold, -127, 4);

/* Zero selects the modulation's recommended time; the core refuses anything
 * else below its minimum. */
RF215_ATTR_RW(cca_listen_us, listen_us, edd_us, 0, RF215_EDD_US_MAX);

RF215_CCA_ATTR(cca_retries, retries, lbt_retries, 0, 255);

/* The core caps a longer wait, so accepting one would report back a value it
 * does not use. */
RF215_CCA_ATTR(cca_backoff, backoff, lbt_backoff_us, 0, RF215_CCA_BACKOFF_US_MAX);

/* iq/ */
RF215_KOBJ_ATTR(iq_mode, mode, 0644, iq_mode_show, iq_mode_store);

RF215_KOBJ_ATTR(iq_status, status, 0444, iq_status_show, NULL);

RF215_IQ_ATTR(iq_drive, drive, drive_ma, 0, 3);

RF215_IQ_ATTR(iq_cmv, cmv, common_mode, 0, 3);

RF215_IQ_ATTR(iq_cmv_1v2, cmv_1v2, common_mode_1v2, 0, 1);

RF215_IQ_ATTR(iq_embedded_ctrl, embedded_ctrl, embedded_tx_control, 0, 1);

RF215_IQ_ATTR(iq_ext_loopback, ext_loopback, external_loopback, 0, 1);

RF215_IQ_ATTR(iq_skew, skew, skew, 0, 3);

static struct attribute* rf215_top_attrs[] = {
    &rf215_attr_band.attr,       &rf215_attr_state.attr,
    &rf215_attr_rssi.attr,       &rf215_attr_ed.attr,
    &rf215_attr_modulation.attr, &rf215_attr_frequency.attr,
    &rf215_attr_channel.attr,    &rf215_attr_channel_spacing.attr,
    &rf215_attr_fcs.attr,        NULL,
};

static struct attribute* rf215_ofdm_attrs[] = {
    &rf215_attr_ofdm_opt.attr,
    &rf215_attr_ofdm_mcs.attr,
    NULL,
};

static struct attribute* rf215_qpsk_attrs[] = {
    &rf215_attr_qpsk_fchip.attr,
    &rf215_attr_qpsk_mode.attr,
    NULL,
};

static struct attribute* rf215_fsk_attrs[] = {
    &rf215_attr_fsk_srate.attr,    &rf215_attr_fsk_midx.attr,
    &rf215_attr_fsk_mord.attr,     &rf215_attr_fsk_bt.attr,
    &rf215_attr_fsk_fec.attr,      &rf215_attr_fsk_dw.attr,
    &rf215_attr_fsk_preamble.attr, NULL,
};

static struct attribute* rf215_frontend_attrs[] = {
    &rf215_attr_tx_power.attr,
    &rf215_attr_lna_bypass.attr,
    &rf215_attr_agc_map.attr,
    &rf215_attr_agc_target.attr,
    NULL,
};

static struct attribute* rf215_cca_attrs[] = {
    &rf215_attr_cca_enable.attr,  &rf215_attr_cca_threshold.attr, &rf215_attr_cca_listen_us.attr,
    &rf215_attr_cca_retries.attr, &rf215_attr_cca_backoff.attr,   NULL,
};

static struct attribute* rf215_iq_attrs[] = {
    &rf215_attr_iq_mode.attr,
    &rf215_attr_iq_drive.attr,
    &rf215_attr_iq_cmv.attr,
    &rf215_attr_iq_cmv_1v2.attr,
    &rf215_attr_iq_embedded_ctrl.attr,
    &rf215_attr_iq_ext_loopback.attr,
    &rf215_attr_iq_skew.attr,
    &rf215_attr_iq_status.attr,
    NULL,
};

static const struct attribute_group rf215_top_group = {
    .attrs = rf215_top_attrs,
};

static const struct attribute_group rf215_ofdm_group = {
    .name = "ofdm",
    .attrs = rf215_ofdm_attrs,
};

static const struct attribute_group rf215_qpsk_group = {
    .name = "qpsk",
    .attrs = rf215_qpsk_attrs,
};

static const struct attribute_group rf215_fsk_group = {
    .name = "fsk",
    .attrs = rf215_fsk_attrs,
};

static const struct attribute_group rf215_frontend_group = {
    .name = "frontend",
    .attrs = rf215_frontend_attrs,
};

static const struct attribute_group rf215_cca_group = {
    .name = "cca",
    .attrs = rf215_cca_attrs,
};

static const struct attribute_group rf215_iq_group = {
    .name = "iq",
    .attrs = rf215_iq_attrs,
};

static const struct attribute_group* rf215_iface_groups[] = {
    &rf215_top_group,      &rf215_ofdm_group, &rf215_qpsk_group, &rf215_fsk_group,
    &rf215_frontend_group, &rf215_cca_group,  &rf215_iq_group,   NULL,
};

static const char* const rf215_iq_mode_names[] = {
    [RF215_CHIP_MODE_BBRF] = "bbrf",
    [RF215_CHIP_MODE_RF] = "rf",
    [RF215_CHIP_MODE_BBRF09] = "bbrf09",
    [RF215_CHIP_MODE_BBRF24] = "bbrf24",
};

/******************************************************************************/
/** Public Functions **********************************************************/

int rf215_sysfs_attach(struct rf215_iface* ifc) {
    struct kobject* kobj;
    int ret;

    kobj = kobject_create_and_add("rf215", &ifc->ndev->dev.kobj);
    if (!kobj) {
        return -ENOMEM;
    }

    ret = sysfs_create_groups(kobj, rf215_iface_groups);
    if (ret) {
        kobject_put(kobj);
        return ret;
    }

    ifc->sysfs = kobj;

    return 0;
}

void rf215_sysfs_detach(struct rf215_iface* ifc) {
    if (!ifc->sysfs) {
        return;
    }

    /* Returns once no show or store is running, so none outlives the interface. */
    kobject_del(ifc->sysfs);
    kobject_put(ifc->sysfs);
    ifc->sysfs = NULL;
}

/******************************************************************************/
/** Static Functions **********************************************************/

/******************************************************************************/
/** Sysfs Configuration *******************************************************/

/*
 * Per-transceiver settings. Each write is applied at once by a configure operation, link up
 * or down; only the chip-wide I/Q settings need both interfaces down.
 */

static struct rf215_iface* rf215_from_kobj(struct kobject* kobj) {
    return rf215_from_ndev(to_net_dev(kobj_to_dev(kobj->parent)));
}

/* Configuring takes the transceiver off air and puts a receiver back itself. */
static int rf215_commit(struct rf215_iface* ifc) {
    return rf215_configure(ifc);
}

static int rf215_parse(const char* buf, u32 min, u32 max, u32* out) {
    u32 value;
    int ret = kstrtou32(buf, 0, &value);

    if (ret) {
        return ret;
    }

    if (value < min || value > max) {
        return -EINVAL;
    }

    *out = value;

    return 0;
}

/* Settings that would break a link in progress may only change while down. */
static bool rf215_link_is_up(const struct rf215_iface* ifc) {
    return ifc->ndev && (ifc->ndev->flags & IFF_UP);
}

static ssize_t modulation_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    (void)attr;

    const enum rf215_modulation mod = rf215_from_kobj(kobj)->phy.modulation;

    return sysfs_emit(buf,
                      "%s\n",
                      mod == RF215_MOD_QPSK      ? "qpsk"
                          : mod == RF215_MOD_FSK ? "fsk"
                                                 : "ofdm");
}

static ssize_t
modulation_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {
    struct rf215_iface* ifc = rf215_from_kobj(kobj);
    enum rf215_modulation previous;
    enum rf215_modulation value;
    int ret;

    (void)attr;

    if (sysfs_streq(buf, "ofdm")) {
        value = RF215_MOD_OFDM;
    } else if (sysfs_streq(buf, "qpsk")) {
        value = RF215_MOD_QPSK;
    } else if (sysfs_streq(buf, "fsk")) {
        value = RF215_MOD_FSK;
    } else {
        return -EINVAL;
    }

    mutex_lock(&ifc->rf->cfg_lock);
    previous = ifc->phy.modulation;
    ifc->phy.modulation = value;

    ret = rf215_commit(ifc);
    if (ret) {
        ifc->phy.modulation = previous;
    }
    mutex_unlock(&ifc->rf->cfg_lock);

    return ret ? ret : (ssize_t)len;
}

static ssize_t fcs_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    (void)attr;

    return sysfs_emit(buf, "%u\n", (u32)rf215_from_kobj(kobj)->phy.fcs);
}

static ssize_t
fcs_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {
    struct rf215_iface* ifc = rf215_from_kobj(kobj);
    bool previous;
    u32 value;
    int ret;

    (void)attr;

    ret = rf215_parse(buf, 0, 1, &value);
    if (ret) {
        return ret;
    }

    /* The MTU changes under RTNL. Unregistering holds it while it waits for this file to
     * close, so it is only tried here. */
    if (!rtnl_trylock()) {
        return restart_syscall();
    }

    mutex_lock(&ifc->rf->cfg_lock);
    previous = ifc->phy.fcs;
    ifc->phy.fcs = value;

    ret = rf215_commit(ifc);
    if (ret) {
        ifc->phy.fcs = previous;
    }
    mutex_unlock(&ifc->rf->cfg_lock);

    rf215_raw_update_mtu(ifc);
    rtnl_unlock();

    return ret ? ret : (ssize_t)len;
}

static ssize_t rssi_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    (void)attr;

    return sysfs_emit(buf, "%d\n", rf215_from_kobj(kobj)->rx_rssi);
}

static bool rf215_any_link_up(const struct rf215_spi* rf) {
    unsigned int i;

    for (i = 0; i < RF215_TRX_COUNT; i++) {
        if (rf215_link_is_up(&rf->iface[i])) {
            return true;
        }
    }

    return false;
}

static ssize_t iq_mode_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    const unsigned int mode = (unsigned int)rf215_from_kobj(kobj)->rf->core.iq.chip_mode;
    const char* name = (mode < ARRAY_SIZE(rf215_iq_mode_names) && rf215_iq_mode_names[mode])
        ? rf215_iq_mode_names[mode]
        : "?";

    (void)attr;

    return sysfs_emit(buf, "%s\n", name);
}

static ssize_t
iq_mode_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t len) {
    struct rf215_spi* rf = rf215_from_kobj(kobj)->rf;
    struct rf215_iq_config iq;
    unsigned int mode;
    int ret;

    (void)attr;

    for (mode = 0; mode < ARRAY_SIZE(rf215_iq_mode_names); mode++) {
        if (rf215_iq_mode_names[mode] && sysfs_streq(buf, rf215_iq_mode_names[mode])) {
            break;
        }
    }

    if (mode >= ARRAY_SIZE(rf215_iq_mode_names)) {
        return -EINVAL;
    }

    /* RTNL keeps the link state fixed from the check to the apply. */
    if (!rtnl_trylock()) {
        return restart_syscall();
    }

    if (rf215_any_link_up(rf)) {
        rtnl_unlock();
        return -EBUSY;
    }

    mutex_lock(&rf->cfg_lock);
    rt_mutex_lock(&rf->lock);
    iq = rf->core.iq;
    iq.chip_mode = (enum rf215_chip_mode)mode;
    rf215_set_iq_config(&rf->core, &iq);
    rt_mutex_unlock(&rf->lock);

    ret = rf215_apply_iq(rf215_chip_iface(rf));
    mutex_unlock(&rf->cfg_lock);
    rtnl_unlock();

    return ret ? ret : (ssize_t)len;
}

static ssize_t iq_status_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    const struct rf215_iq_status* st = rf215_iq_status(&rf215_from_kobj(kobj)->rf->core);

    (void)attr;

    return sysfs_emit(buf,
                      "sync=%d sync_failure=%d receiver_failsafe=%d\n",
                      st->synchronised,
                      st->sync_failure,
                      st->receiver_failsafe);
}

static ssize_t band_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    (void)attr;

    return sysfs_emit(
        buf, "%s\n", rf215_from_kobj(kobj)->band == RF215_BAND_24 ? "2.4GHz" : "sub-GHz");
}

static ssize_t state_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    struct rf215_iface* ifc = rf215_from_kobj(kobj);
    ssize_t n;

    (void)attr;

    rt_mutex_lock(&ifc->rf->lock);
    n = sysfs_emit(buf, "%s\n", rf215_step_name(&ifc->rf->core, ifc->band));
    rt_mutex_unlock(&ifc->rf->lock);

    return n;
}

/* Runs a single energy measurement on the channel and reports it in dBm. */
static ssize_t ed_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf) {
    struct rf215_iface* ifc = rf215_from_kobj(kobj);
    const struct rf215_req req = {
        .kind = RF215_REQ_ED,
        .band = ifc->band,
    };
    int ret;

    (void)attr;

    /* The measurement only completes while the receiver is on air; asking
     * for one with the transceiver off would block until it timed out. */
    if (!ifc->rf->core.trx[ifc->band].receiving) {
        return -ENETDOWN;
    }

    ret = rf215_run_op(ifc, &req);
    if (ret) {
        return ret;
    }

    return sysfs_emit(buf, "%d\n", ifc->ed_dbm);
}
