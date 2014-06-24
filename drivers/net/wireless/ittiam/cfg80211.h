/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Filename : cfg80211.h
 * Abstract : This file is a definition for cfg80211 subsystem
 *
 * Authors	:
 * Leon Liu <leon.liu@spreadtrum.com>
 * Wenjie.Zhang <Wenjie.Zhang@spreadtrum.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ITM_CFG80211_H__
#define __ITM_CFG80211_H__

#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <net/ieee80211_radiotap.h>
#include <net/cfg80211.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>

#ifdef CONFIG_ITM_WIFI_DIRECT
#include <linux/workqueue.h>
#endif	/* CONFIG_ITM_WIFI_DIRECT */
#include "ittiam.h"

/*FIXME: determine the actual values for the macros below*/
#define SCAN_IE_LEN_MAX			2304
#define MAX_NUM_PMKIDS			4

#define MAX_SITES_FOR_SCAN		12

#define WLAN_MAX_SSID_SIZE		32

#define WLAN_MAX_KEY_INDEX		3

#define ITM_SCAN_TIMER_INTERVAL_MS	8000

/* parise or group key type */
#define GROUP				0
#define PAIRWISE			1

#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
/* emulate a modern version */
#define LINUX_VERSION_CODE KERNEL_VERSION(3, 10, 0)

enum ANDROID_WIFI_CMD {
	ANDROID_WIFI_CMD_START,
	ANDROID_WIFI_CMD_STOP,
	ANDROID_WIFI_CMD_SCAN_ACTIVE,
	ANDROID_WIFI_CMD_SCAN_PASSIVE,
	ANDROID_WIFI_CMD_RSSI,
	ANDROID_WIFI_CMD_LINKSPEED,
	ANDROID_WIFI_CMD_RXFILTER_START,
	ANDROID_WIFI_CMD_RXFILTER_STOP,
	ANDROID_WIFI_CMD_RXFILTER_ADD,
	ANDROID_WIFI_CMD_RXFILTER_REMOVE,
	ANDROID_WIFI_CMD_BTCOEXSCAN_START,
	ANDROID_WIFI_CMD_BTCOEXSCAN_STOP,
	ANDROID_WIFI_CMD_BTCOEXMODE,
	ANDROID_WIFI_CMD_SETSUSPENDOPT,
	ANDROID_WIFI_CMD_P2P_DEV_ADDR,
	ANDROID_WIFI_CMD_SETFWPATH,
	ANDROID_WIFI_CMD_SETBAND,
	ANDROID_WIFI_CMD_GETBAND,
	ANDROID_WIFI_CMD_COUNTRY,
	ANDROID_WIFI_CMD_P2P_SET_NOA,
	ANDROID_WIFI_CMD_P2P_GET_NOA,
	ANDROID_WIFI_CMD_P2P_SET_PS,
	ANDROID_WIFI_CMD_SET_AP_WPS_P2P_IE,
#ifdef PNO_SUPPORT
	ANDROID_WIFI_CMD_PNOSSIDCLR_SET,
	ANDROID_WIFI_CMD_PNOSETUP_SET,
	ANDROID_WIFI_CMD_PNOENABLE_SET,
	ANDROID_WIFI_CMD_PNODEBUG_SET,
#endif
	ANDROID_WIFI_CMD_MACADDR,
	ANDROID_WIFI_CMD_BLOCK,
	ANDROID_WIFI_CMD_WFD_ENABLE,
	ANDROID_WIFI_CMD_WFD_DISABLE,
	ANDROID_WIFI_CMD_WFD_SET_TCPPORT,
	ANDROID_WIFI_CMD_WFD_SET_MAX_TPUT,
	ANDROID_WIFI_CMD_WFD_SET_DEVTYPE,
	ANDROID_WIFI_CMD_MAX
};

enum cp2_state {
	ITM_NOT_READY,
	ITM_READY
};

enum wlan_state {
	ITM_UNKOWN = 0,
	ITM_SCANNING,
	ITM_SCAN_ABORTING,
	ITM_DISCONNECTED,
	ITM_CONNECTING,
	ITM_CONNECTED
};

enum wlan_mode {
	ITM_NONE_MODE,
	ITM_STATION_MODE,
	ITM_AP_MODE,
	ITM_NPI_MODE,
#ifdef CONFIG_ITM_WIFI_DIRECT
	ITM_P2P_CLIENT_MODE,
	ITM_P2P_GO_MODE,
#endif	/* CONFIG_ITM_WIFI_DIRECT */
};

#define HOSTAP_CONF_FILE_NAME "/data/misc/wifi/hostapd.conf"

struct hostap_conf {
	char wpa_psk[128];
	unsigned int len;
};

#ifdef CONFIG_ITM_WIFI_DIRECT
void init_register_frame_param(struct itm_priv *priv);
void itm_cfg80211_p2p_rx_mgmt(struct itm_priv *priv);
void itm_cfg80211_remain_on_channel_expired(struct itm_priv *priv);
void itm_cfg80211_p2p_prob_request(struct itm_priv *priv);
void itm_cfg80211_new_station(struct itm_priv *priv);
void itm_cfg80211_mgmt_deauth(struct itm_priv *priv);
void itm_cfg80211_mgmt_disassoc(struct itm_priv *priv);
extern int itm_wlan_set_tx_mgmt_cmd(struct wlan_sipc *wlan_sipc,
				    struct ieee80211_channel *channel,
				    unsigned int wait, const u8 *mac,
				    size_t mac_len);
extern int itm_wlan_remain_chan_cmd(struct wlan_sipc *wlan_sipc,
				    struct ieee80211_channel *channel,
				    enum nl80211_channel_type channel_type,
				    unsigned int duration, u64 *cookie);
extern int itm_wlan_cancel_remain_chan_cmd(struct wlan_sipc *wlan_sipc,
					   u64 cookie);
#if 0
extern int itm_wlan_set_change_beacon_cmd(struct wlan_sipc *wlan_sipc,
					  u8 *beacon, u16 len);
#endif
extern int itm_wlan_set_p2p_ie_cmd(struct wlan_sipc *wlan_sipc, u8 type,
				   const u8 *ie, u8 len);
void itm_mac_event_report_frame(struct itm_priv *priv);
#endif	/* CONFIG_ITM_WIFI_DIRECT */

int hostap_conf_load(char *filename, u8 *key_val);

typedef struct android_wifi_priv_cmd {
#ifdef CONFIG_COMPAT
	compat_uptr_t buf;
#else
	char *buf;
#endif
	int used_len;
	int total_len;
} android_wifi_priv_cmd;

struct itm_ieee80211_regdomain {
	u32 n_reg_rules;
	char alpha2[2];
	struct ieee80211_reg_rule reg_rules[];
};

void itm_cfg80211_report_connect_result(struct itm_priv *priv);
void itm_cfg80211_report_disconnect_done(struct itm_priv *priv);
void itm_cfg80211_report_scan_done(struct itm_priv *priv, bool aborted);
void itm_cfg80211_report_ready(struct itm_priv *priv);
void itm_cfg80211_report_tx_busy(struct itm_priv *priv);
void itm_cfg80211_report_softap(struct itm_priv *priv);
int itm_cfg80211_android_priv_cmd(struct net_device *dev, struct ifreq *req);
int itm_get_mac_from_cfg(struct itm_priv *priv);
int itm_register_wdev(struct itm_priv *priv, struct device *dev);
void itm_unregister_wdev(struct itm_priv *priv);

#endif/* __ITM_CFG80211_H__ */
