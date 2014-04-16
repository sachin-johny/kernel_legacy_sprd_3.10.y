/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : cglobals_sta.h                                       */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      the STA mode.                                        */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef CGLOBALS_STA_H
#define CGLOBALS_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "management_sta.h"
#include "sme_sta.h"

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/
// update authentication and association policy
#define AUTH_MAX_RETRY_CNT 5
#define ASSOC_MAX_RETRY_CNT 5
extern UWORD32 g_auth_retry_cnt ;
extern UWORD32 g_assoc_retry_cnt ;

extern volatile UWORD8   g_link_loss_count;

extern UWORD8     g_bss_dscr_set_index;
extern UWORD8     g_last_network_index;

#ifndef COMBO_SCAN //chenq mod for combo scan 2013-03-12 
extern UWORD8     g_num_channels;
extern UWORD8     g_channel_index;
#else
extern UWORD16     g_num_channels;
extern UWORD16     g_channel_index;
#endif

extern UWORD8     g_scan_source;
extern BOOL_T     g_start_scan_req;
extern UWORD8     g_force_keep_bss_info;
extern UWORD8     g_bss_dscr_first_set_sent;
extern UWORD16    g_asoc_id;
extern UWORD16    g_signal_quality;
extern SITE_SURVEY_T g_site_survey_enabled;
extern BOOL_T     g_keep_connection;
extern bss_dscr_t g_bss_dscr_set[MAX_SITES_FOR_SCAN];
extern WORD8      g_channel_rssi[];
extern UWORD8     g_prefered_bssid[6];
extern UWORD8     g_assoc_req_info[MAX_STRING_LEN];
extern UWORD8     g_assoc_res_info[MAX_STRING_LEN];
extern UWORD8     g_channel_list[];
extern UWORD8     g_ethernet_addr[6];
extern UWORD8     g_join_sta_addr[6];
extern UWORD8     g_link_loss_threshold;
extern UWORD8     g_scan_filter;
extern UWORD16    g_active_scan_time;
extern UWORD16    g_passive_scan_time;
extern UWORD16    g_site_survey_scan_time;
extern UWORD32    g_obss_scan_period;
extern ALARM_HANDLE_T *g_obss_scan_timer;
// 20120709 caisf masked, merged ittiam mac v1.2 code
//extern BOOL_T     g_int_supp_enable;

//chenq add for wpa/rsn ie buf
extern UWORD8 g_wpa_rsn_ie_buf_1[MAX_SITES_FOR_SCAN][512];

//chenq add for check state
extern UWORD32 g_wrong_bssid;
extern UWORD32 g_unable2decode;
extern UWORD32 g_perform_mac_reassembly_err;
extern UWORD32 g_tx_retries;
extern UWORD32 g_rx_misc;
extern UWORD32 g_missed_beacons;

//chenq add 2013-02-06
extern int itm_scan_flag;

//chenq add for BusyTraffic 2013-02-12
extern BOOL_T     g_BusyTraffic;

//chenq add for UI cmd ps 2013-02-20
extern BOOL_T     g_UI_cmd_ps;

#ifdef COMBO_SCAN
//chenq add for scan hide AP 2013-03-12
extern UWORD8     g_ap_combo_scan_list[MAX_AP_COMBO_SCAN][MAX_SSID_LEN];
extern UWORD32    g_ap_combo_scan_cnt;
extern UWORD32    g_ap_combo_scan_index;

extern struct hrtimer * g_ap_combo_scan_timer;
extern ktime_t g_ap_combo_scan_kt;
#endif

//chenq add for ext wpa_wpa2_hs 2013-04-16
extern BOOL_T g_wpa_wpa2_hs_complete;

//chenq add for ext wpa_wpa2_rehs 2013-12-05
extern BOOL_T g_wpa_wpa2_rehs;
extern UWORD8 g_wpa_wpa2_ptk_key[32];
//chenq add do ap list merge logic 2013-08-28
extern bss_link_dscr_t * g_user_getscan_aplist;
extern UWORD8 g_link_list_bss_count;

/*junbinwang add for cr 238822. 20131128*/
extern UWORD32 g_merge_aplist_flag;
//chenq add for auto set tx power 2013-07-29
#ifdef TROUT2_WIFI_IC
extern UWORD8  g_update_rssi_count;
extern UWORD32  g_tbtt_cnt;
extern UWORD8  g_auto_tx_power_cnt;
extern UWORD32 g_pwr_tx_11b;
extern UWORD32 g_pwr_tx_11g;
extern UWORD32 g_pwr_tx_11n;
extern UWORD32 g_pwr_tx_rx_11abgn_reset;
extern WORD32 g_pwr_rx_11abgn;
#endif

extern volatile UWORD8 g_cmcc_test_mode;

//chenq mod scan_limit => g_default_scan_limit 2013-09-11
extern int g_default_scan_limit;

typedef enum
{
	WIFI_NORMAL_POWER_MODE = 0,
	WIFI_LOW_POWER_MODE    = 1,
}WIFI_POWER_MODE;	//define by chwg.
extern WIFI_POWER_MODE g_wifi_power_mode;	//define in trout_sdio modules, add by chwg 2013.12.2
//caisf add 0226
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
typedef enum
{
    wifi_suspend_nosuspend = 0,
    wifi_suspend_early_suspending = 1,
    wifi_suspend_early_suspend = 2,
    wifi_suspend_suspend = 3
} wifi_suspend_status;

extern volatile UWORD32   g_enable_machw_clock_flag;
extern volatile UWORD32   g_last_rmac_pa_con_val;
extern volatile UWORD32   g_enable_ttbt_isr_flag;

extern wifi_suspend_status g_wifi_suspend_status;

#ifdef WIFI_SLEEP_POLICY
extern struct completion wifi_resume_completion;
extern struct completion null_frame_completion;
extern volatile UWORD32 *null_frame_dscr;
extern struct mutex suspend_mutex;
extern volatile int g_wakeup_flag_for_60s;
#ifdef WAKE_LOW_POWER_POLICY
#define FLOW_DETECT_TIME	2000
extern struct mutex low_power_mutex;	//add by chwg 2013.12.2
#endif	
#endif
#endif


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_globals_sta(void);

//chenq add 2012-10-29
extern void init_globals_sta_plus(void);

/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

INLINE void init_assoc_info(void)
{
    /* Reset Association information arrays to -1 */
    mem_set(g_assoc_req_info, 0xFF, MAX_STRING_LEN);
    mem_set(g_assoc_res_info, 0xFF, MAX_STRING_LEN);

    /* Set the length of the association info to zero */
    g_assoc_req_info[0] = 0;
    g_assoc_res_info[0] = 0;
}

#endif /* CGLOBALS_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
