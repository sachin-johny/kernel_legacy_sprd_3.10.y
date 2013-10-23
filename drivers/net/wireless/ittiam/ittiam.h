/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Authors:
 * Keguang Zhang <keguang.zhang@spreadtrum.com>
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

/*
 * Device instance data.
 */

#ifndef __ITTIAM_H__
#define __ITTIAM_H__

#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/ieee80211.h>
#include <linux/if_ether.h>
#include "itm_sipc_types.h"


struct itm_priv {
	struct net_device *ndev;	/* Linux net device */
	struct wireless_dev *wdev;	/* Linux wireless device */
	spinlock_t scan_req_lock;	/* spinlock for scan request */
	atomic_t stopped;		/* sblock indicator */
	int txrcnt;			/* seth tx resend count*/

	struct wlan_sipc *wlan_sipc;	/* hook of sipc command ops */

	int cp2_status;

	/* CFG80211 */
	struct cfg80211_scan_request *scan_request;
	struct timer_list scan_timeout; /* Timer for catch scan event timeout */
	int connect_status;
	int mode;
	int ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];

	/* Encryption stuff */
	u8 cipher_type;
	u8 key_index[2];
	u8 key[2][4][WLAN_MAX_KEY_LEN];
	u8 key_len[2][4];
	u8 key_txrsc[2][WLAN_MAX_KEY_LEN];
};

#endif/*__ITTIAM_H__*/
