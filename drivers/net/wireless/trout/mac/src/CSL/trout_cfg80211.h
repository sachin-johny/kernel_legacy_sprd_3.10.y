//*****************************************************************************/
/*                                                                           */
/*  File Name         : trout_cfg80211.h                                     */
/*	Author			  : leon liu											 */
/*                                                                           */
/*  Description       : CFG80211 support for Trout							 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifndef __TROUT_CFG80211_H__
#define __TROUT_CFG80211_H__

#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/circ_buf.h>
#include <asm/uaccess.h>
#include <asm/byteorder.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <net/ieee80211_radiotap.h>
#include <net/cfg80211.h>
#include <linux/sem.h>
#include <linux/sched.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include <linux/if_arp.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/completion.h>
#include "itypes.h"
#include "csl_linux.h"
#include "trout_trace.h"
#include "fsm.h"
#include "common.h"
#include "itm_wifi_iw.h"
#include "mac_init.h"
#include "management_sta.h"
#include "cglobals_sta.h"

//Trout global variables and routines
extern BOOL_T	g_keep_connection;
extern BOOL_T g_BusyTraffic;
extern int itm_scan_flag;
extern unsigned char *g_itm_config_buf;
extern int is_scanlist_report2ui;

#define IEEE_CRYPT_ALG_NAME_LEN 16

typedef struct ieee_param {
		u32 cmd;
		u8 sta_addr[ETH_ALEN];
		union {
				struct {
						u8 name;
						u32 value;
				} wpa_param;
				struct {
						u32 len;
						u8 reserved[32];
						u8 data[0];
				} wpa_ie;
				struct{
						int command;
						int reason_code;
				} mlme;
				struct {
						u8 alg[IEEE_CRYPT_ALG_NAME_LEN];
						u8 set_tx;
						u32 err;
						u8 idx;
						u8 seq[8]; /* sequence counter (set: RX, get: TX) */
						u16 key_len;
						u8 key[0];
				} crypt;
				//#ifdef CONFIG_AP_MODE
#if 0
				struct {
						u16 aid;
						u16 capability;
						int flags;
						u8 tx_supp_rates[16];		
						struct ieee80211_ht_cap ht_cap;
				} add_sta;
#endif
				struct {
						u8	reserved[2];//for set max_num_sta
						u8	buf[0];
				} bcn_ie;
				//#endif

		} u;	   
}ieee_param;


//FIXME: determine the actual values for the macros below
#define SCAN_IE_LEN_MAX			2304
#define MAX_NUM_PMKIDS			4

struct trout_wdev_priv
{
		struct wireless_dev *trout_wdev;
		struct trout_private *trout_priv; /*Points to struct trout_private*/
		struct cfg80211_scan_request *scan_request;
		spinlock_t scan_req_lock;
};

int trout_cfg80211_report_connect_result(struct net_device *dev, u8 *bssid, u8 *req_ie, u32 req_ie_len
				, u8* resp_ie, u32 resp_ie_len, u16 status);
int trout_cfg80211_report_scan_done(struct net_device *dev, int aborted);
int trout_wdev_alloc(struct trout_private *trout_priv, struct device *dev);
void trout_wdev_free(struct wireless_dev *wdev);
int trout_cfg80211_android_priv_cmd(struct net_device *dev, struct ifreq *req);

int trout_cfg80211_del_prev_bss(struct net_device *dev);

#define TRACE_FUNC()		do{printk("Entered ===>>%s\n", __func__);}while(0)
//#define TRACE_FUNC()		
#define ITM_MAC_FMT			"%02x:%02x:%02x:%02x:%02x:%02x"
#define ITM_MAC_ARG(x) ((u8*)(x))[0],((u8*)(x))[1],((u8*)(x))[2],((u8*)(x))[3],((u8*)(x))[4],((u8*)(x))[5]

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

		ANDROID_WIFI_CMD_SET_MAX_POWER,
		ANDROID_WIFI_CMD_UNSET_MAX_POWER,
		ANDROID_WIFI_CMD_DHCP_STATUS,

		ANDROID_WIFI_CMD_MAX
};

typedef struct android_wifi_priv_cmd {

#ifdef CONFIG_COMPAT
		compat_uptr_t buf;
#else
		char *buf;
#endif

		int used_len;
		int total_len;
} android_wifi_priv_cmd;

#define WLAN_MAX_SSID_SIZE		32
/*leon liu added scan completion on 2013-07-11*/
extern struct completion scan_completion;

#endif /*TROUT_CFG80211_H*/
