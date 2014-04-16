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
/*  File Name         : cglobals_sta.c                                       */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      the STA mode.                                        */
/*                                                                           */
/*  List of Functions : init_globals_sta                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_sta.h"
#include "mib.h"
#include "cglobals_sta.h"
#include "pm_sta.h"
#include "sta_prot_if.h"
#include "sme_sta.h"
#include "channel_sw.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/* Management globals */
volatile UWORD8  g_link_loss_count        = 0;/* Link loss counter            */
#ifndef COMBO_SCAN //chenq mod for combo scan 2013-03-12 
UWORD8  g_channel_index                   = 0;/* Channel index to be scanned  */
UWORD8  g_num_channels                    = 0;/* Number of channels to scan   */
#else
UWORD16  g_channel_index                   = 0;/* Channel index to be scanned  */
UWORD16  g_num_channels                    = 0;/* Number of channels to scan   */
#endif

UWORD8  g_bss_dscr_set_index              = 0;/* BSS descriptor list index    */
UWORD8  g_last_network_index              = 0xFF;/* Index of the last BSS/IBSS */
UWORD8  g_force_keep_bss_info             = 0xFF;/* Index the STA is forced to keep   */
UWORD16 g_asoc_id                         = 0;/* Association ID for BSS mode  */
UWORD16 g_signal_quality                  = 0;
UWORD8  g_bss_dscr_first_set_sent         = 0;/* Idx used to send rsp to host */
WORD8   g_channel_rssi[MAX_SUPP_CHANNEL]  = {0}; /* RSSI values for each channel */

UWORD8        g_scan_source              = DEFAULT_SCAN;
BOOL_T        g_start_scan_req           = BFALSE;
UWORD8        g_scan_filter              = CH_FILTER_OFF;//libing modify for fix the scan ap buger:211215
UWORD8        g_link_loss_threshold      = DEFAULT_LINK_LOSS_THRESHOLD;
UWORD16       g_active_scan_time         = ACTIVE_SCAN_TIME;
UWORD16       g_passive_scan_time        = PASSIVE_SCAN_TIME;
UWORD16       g_site_survey_scan_time    = MIN_SCAN_TIME;
SITE_SURVEY_T g_site_survey_enabled      = SITE_SURVEY_OFF;
BOOL_T        g_keep_connection          = BFALSE;
UWORD32       g_obss_scan_period         = 0;
ALARM_HANDLE_T *g_obss_scan_timer        = NULL;

volatile UWORD8 g_cmcc_test_mode = 0;

//chenq add 2012-10-29
struct task_struct * itm_scan_task = NULL;

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef SUPP_11I
BOOL_T        g_int_supp_enable          = BTRUE;
#else /* SUPP_11I */
BOOL_T        g_int_supp_enable          = BFALSE;
#endif /* SUPP_11I */
#endif

//chenq add for check state
UWORD32 g_wrong_bssid = 0;
UWORD32 g_unable2decode = 0;
UWORD32 g_perform_mac_reassembly_err = 0;
UWORD32 g_tx_retries = 0;
UWORD32 g_rx_misc = 0;

UWORD32 g_missed_beacons = 0;

//caisf add 0226
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
volatile UWORD32   g_enable_machw_clock_flag = 0;
volatile UWORD32   g_last_rmac_pa_con_val = 0;
volatile UWORD32   g_enable_ttbt_isr_flag = 0; // 1 enable, 2 disable;

wifi_suspend_status g_wifi_suspend_status = wifi_suspend_nosuspend;

#ifdef WIFI_SLEEP_POLICY
volatile int g_wakeup_flag_for_60s = 0;
struct completion wifi_resume_completion = COMPLETION_INITIALIZER(wifi_resume_completion);
struct completion null_frame_completion = COMPLETION_INITIALIZER(null_frame_completion);
volatile UWORD32 *null_frame_dscr = NULL;
struct mutex suspend_mutex = __MUTEX_INITIALIZER(suspend_mutex);

#ifdef WAKE_LOW_POWER_POLICY
struct mutex low_power_mutex = __MUTEX_INITIALIZER(low_power_mutex);	//chwg add.
#endif

#endif
#endif

/* The ethernet address of STA */
UWORD8     g_ethernet_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bss_dscr_t g_bss_dscr_set[MAX_SITES_FOR_SCAN] = {{INFRASTRUCTURE,0,},};

//chenq add for test 2013-02-05
#ifndef COMBO_SCAN
UWORD8     g_channel_list[MAX_SUPP_CHANNEL*5]   = {0}; /* Channel list         */
#else
UWORD8     g_channel_list[MAX_SUPP_CHANNEL* SCAN_IN_ONE_CHANNEL_CNT * (MAX_AP_COMBO_SCAN+1)] = {0};/* Channel list         */
#endif

//chenq add for BusyTraffic 2013-02-12
BOOL_T     g_BusyTraffic = BFALSE;

//chenq add for UI cmd ps 2013-02-20
BOOL_T     g_UI_cmd_ps = BFALSE;

//chenq add a save ap wpa/rsn ie buf
UWORD8 g_wpa_rsn_ie_buf_1[MAX_SITES_FOR_SCAN][512] = {{0},};

/* Global arrays to store the association requenst and response information  */
/* This information is needed to be sent to external supplicant              */
UWORD8     g_assoc_req_info[MAX_STRING_LEN]   = {0};
UWORD8     g_assoc_res_info[MAX_STRING_LEN]   = {0};
UWORD8     g_join_sta_addr[6]                 = {0};
/* Preffered AP/IBSS network BSSID   */
UWORD8     g_prefered_bssid[6]                = {0};
UWORD32    g_time_ref_count                   = 0;/* Time Reference Count*/

#ifdef COMBO_SCAN
//chenq add for scan hide AP 2013-03-12
UWORD8     g_ap_combo_scan_list[MAX_AP_COMBO_SCAN][MAX_SSID_LEN]= {0};
UWORD32    g_ap_combo_scan_cnt = 0;
UWORD32    g_ap_combo_scan_index = 0;

combo_stats g_combo_aplist[MAX_AP_COMBO_SCAN_LIST];
//chenq add 2013-08-23
extern int is_scanlist_report2ui;

struct hrtimer * g_ap_combo_scan_timer = NULL;
ktime_t g_ap_combo_scan_kt;
#endif

//chenq add for ext wpa_wpa2_hs 2013-04-16
BOOL_T g_wpa_wpa2_hs_complete = BFALSE;
BOOL_T g_wpa_wpa2_rehs = BFALSE;
UWORD8 g_wpa_wpa2_ptk_key[32] = {0};

//chenq add do ap list merge logic 2013-08-28
bss_link_dscr_t * g_user_getscan_aplist = NULL;
UWORD8 g_link_list_bss_count            = 0;
UWORD32 g_merge_aplist_flag = 0;

//chenq mod scan_limit => g_default_scan_limit 2013-09-11
int g_default_scan_limit = 0;

//chenq add for auto set tx power 2013-07-29
#ifdef TROUT2_WIFI_IC
UWORD8 g_update_rssi_count = 0;
UWORD8 g_auto_tx_power_cnt = 50;
UWORD32 g_pwr_tx_11b = 0;
UWORD32 g_pwr_tx_11g = 0;
UWORD32 g_pwr_tx_11n = 0;
UWORD32 g_pwr_tx_rx_11abgn_reset = 1;
WORD32 g_pwr_rx_11abgn = -1;


// add for read tx power from nv and cmcc test[zhongli wang 20130911]
UWORD32 g_nv_11b_tx_pwr[4] = {0,0,0,0};
UWORD32 g_nv_11g_tx_pwr[4] = {0,0,0,0};
UWORD32 g_nv_11n_tx_pwr[4] = {0,0,0,0};
UWORD32 g_cmcc_set_max_pwr = 0;
UWORD32 g_cmcc_cfg_tx_rate = 0;
UWORD32 g_cmcc_set_aci = 0;
UWORD32 g_tbtt_cnt = 0;
UWORD32 g_0x57_delta = 0;
UWORD32 g_pwr_tpc_switch = 1;//0:disable tpc 1:enable tpc
UWORD32 g_pwr_tpc_switch_last = 1;//0:disable tpc 1:enable tpc
// add end [zhongli wang 20130911]

// update authentication and association policy in order to adapt to coex situation
UWORD32 g_auth_retry_cnt = 0;
UWORD32 g_assoc_retry_cnt = 0;

#endif




/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_globals_sta                                         */
/*                                                                           */
/*  Description   : This function initializes the globals for STA mode.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_bss_dscr_set_index                                     */
/*                  g_channel_index                                          */
/*                  g_num_channels                                           */
/*                  g_asoc_id                                                */
/*                  g_scan_timeout                                           */
/*                  g_ethernet_addr                                          */
/*                                                                           */
/*  Processing    : This function initializes all the globals used by STA.   */
/*                                                                           */
/*  Outputs       : All globals are initialized.                             */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_globals_sta(void)
{
    UWORD8 i = 0;
	TROUT_FUNC_ENTER;

    g_channel_index           = 0;
    g_num_channels            = 0;
    g_asoc_id                 = 0;
    g_signal_quality          = 0;
    g_link_loss_count         = 0;
    g_time_ref_count          = 0;
    g_bss_dscr_first_set_sent = 0;
    g_keep_connection         = BFALSE;

	//chenq add for check state
	g_wrong_bssid = 0;
	g_unable2decode = 0;
	g_perform_mac_reassembly_err = 0;
	g_tx_retries = 0;
	g_rx_misc = 0;

	g_missed_beacons = 0;

	//chenq add 2013-02-06
	itm_scan_flag = 0;

	//chenq add for BusyTraffic 2013-02-12
	g_BusyTraffic = BFALSE;

	//chenq add for UI cmd ps 2013-02-20
	g_UI_cmd_ps = BFALSE;

#ifdef COMBO_SCAN
	//chenq add for scan hide AP 2013-03-12
	memset(g_ap_combo_scan_list,0x00,sizeof(g_ap_combo_scan_list));
	g_ap_combo_scan_cnt = 0;
	g_ap_combo_scan_index = 0;

	g_ap_combo_scan_timer = NULL;
#endif	

	//chenq add for ext wpa_wpa2_hs 2013-04-16
	g_wpa_wpa2_hs_complete = BFALSE;
	//chenq add for ext wpa_wpa2_rehs 2013-12-05
	g_wpa_wpa2_rehs = BFALSE;
	memset(g_wpa_wpa2_ptk_key,0x00,32);

	//chenq add 2013-08-23
	is_scanlist_report2ui = 0;
			//chenq add for auto set tx power 2013-07-29
#ifdef TROUT2_WIFI_IC	
			g_update_rssi_count    = 0;
			g_tbtt_cnt = 0;
//			g_pwr_tx_11b		   = 0;
//			g_pwr_tx_11g		   = 0;
//			g_pwr_tx_11n		   = 0;			
			g_pwr_tx_rx_11abgn_reset  = 1;
			g_pwr_rx_11abgn 	   = -1;
#endif

	//chenq mod scan_limit => g_default_scan_limit 2013-09-11
    g_default_scan_limit = 0;

    /* Clear all the existing networks in case of a system error */
    if(g_system_error != NO_ERROR)
    {
        g_bss_dscr_set_index = 0;
    }

    /* Reset the scan source  */
    if(g_start_scan_req == BFALSE)
    {
        g_scan_source = DEFAULT_SCAN;
    }

    for(i = 0; i < 6; i++)
    {
        g_join_sta_addr[i] = 0x00;
    }

    for(i = 0; i < MAX_SUPP_CHANNEL; i++)
    {
        g_channel_rssi[i] = (WORD8)-100;
    }

    /* Initialize all power management related variables */
    set_ps_state(STA_ACTIVE);

    /* Protocol related initializations */
    init_prot_globals_sta();

    /* Initialize all power management related variables */
    initialize_psm_globals();

    /* Initialize channel switching related globals */
    init_csw_sta_globals();

#ifdef MEM_STRUCT_SIZES_INIT

    strncpy(g_struct_sizes[g_struct_count].name, "join_rsp_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(join_rsp_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "asoc_rsp_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(asoc_rsp_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "auth_rsp_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(auth_rsp_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "sta_entry_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(sta_entry_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "scan_rsp_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(scan_rsp_t) ;

#ifdef MAC_802_11I
    strncpy(g_struct_sizes[g_struct_count].name, "rsna_supp_persta_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(rsna_supp_persta_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "auth_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(auth_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "supp_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(supp_t) ;
#endif /*MAC_802_11I*/

#endif /*MEM_STRUCT_SIZES_INIT*/

    /* Initialize association information arrays */
    init_assoc_info();
	//chenq add 2012-10-29
	if( itm_scan_task != NULL )
	{
		wake_up_process(itm_scan_task);
	}	
	itm_scan_task = NULL;
	
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
        //Bug#229353
	if(wake_lock_active(&scan_ap_lock)){
		wake_unlock(&scan_ap_lock);
	    printk("@@@ Warning: Unexpected release scan_ap_lock in %s\n", __func__);
	}
#endif
#endif
	
	 TROUT_FUNC_EXIT;
}


//chenq add 2012-10-29
void init_globals_sta_plus(void)
{
	//chenq add 2012-10-29
	if( itm_scan_task != NULL )
	{
		wake_up_process(itm_scan_task);
	}	
	itm_scan_task = NULL;
}

#endif /* IBSS_BSS_STATION_MODE */
