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
/*  File Name         : cglobals.c                                           */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      for AP and STA mode.                                 */
/*                                                                           */
/*  List of Functions : init_globals                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals.h"
#include "management.h"
#include "rf_if.h"
#include "iconfig.h"
#include "qmu_if.h"
#ifdef MAC_802_11I
#include "mib_11i.h"
#endif /* MAC_802_11I */

#ifdef MEM_STRUCT_SIZES_INIT
#include "index_util.h"
#include "host_if.h"
#include "rsna_auth_km.h"
#include "management_11e.h"
#include "buff_desc.h"
#include "management_11n.h"
#include "blockack.h"
#endif /*MEM_STRUCT_SIZES_INIT*/

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8             g_beacon_index       = 0;
UWORD8             g_ack_policy         = 0;
UWORD8             g_preamble_type      = 0;
UWORD8             g_pcf_mode           = 0;
UWORD8             g_bcst_ssid          = 0;
UWORD8             g_addr_sdram         = 0;
UWORD8             g_phy_active_reg     = 0xFF;
UWORD8             g_current_mac_status = 0xFF;
BOOL_T             g_rx_buffer_based_ack= BTRUE;
UWORD16            g_beacon_len         = 0;
UWORD32            g_phy_ver_num        = 0x0;
UWORD32            g_memory_address     = PLD0_ADDR;

//chenq mod 2012-11-02
//BOOL_T             g_mac_reset_done     = BFALSE;
atomic_t g_mac_reset_done = ATOMIC_INIT(BTRUE);
UWORD8             g_disable_rtscts_norsp = 0;
UWORD8             g_user_allow_rifs_tx = 1;
UWORD16            g_user_pref_channel  = 1;	//channel.
UWORD8             g_device_mode        = NOT_CONFIGURED;


#ifdef TX_ABORT_FEATURE
BOOL_T             g_machw_tx_aborted   = BFALSE;
#endif /* TX_ABORT_FEATURE */
UWORD8             *g_beacon_frame[2]   = {0, 0};
ALARM_HANDLE_T     *g_mgmt_alarm_handle = NULL;

INTERRUPT_HANDLE_T *g_mac_isr_handle    = 0;

WORD8             g_rssi               = -100;
//chenq add snr
WORD16             g_snr                = 0;

//chenq add 2012-10-30
ALARM_HANDLE_T     *g_mac_event_timer    = NULL;

UWORD16            g_join_timeout           = JOIN_START_TIMEOUT;
UWORD16            g_11i_protocol_timeout   = HANDSHAKE_TIMEOUT_802_11I;
UWORD16            g_eapol_response_timeout = RESP_TIMEOUT_801_11I;

#ifdef BSS_ACCESS_POINT_MODE
//fix bug by chengwg, because sta_index is start from 1 to 8, 0 is reserved.
UWORD8             g_decr_fail_cnt[MAX_STA_SUPPORTED + 1] = {0};	
//UWORD8             g_decr_fail_cnt[MAX_STA_SUPPORTED] = {0};
#else /* BSS_ACCESS_POINT_MODE */
UWORD8             g_decr_fail_cnt = 0;
#endif /* BSS_ACCESS_POINT_MODE */

BOOL_T             g_short_preamble_enabled         = BTRUE;
BOOL_T             g_short_slot_allowed             = BTRUE;
BOOL_T             g_phy_continuous_tx_mode_enable  = BFALSE;
BOOL_T             g_reset_mac_in_progress          = BFALSE;

#ifdef BURST_TX_MODE
BOOL_T             g_burst_tx_mode_enabled          = BFALSE;
BOOL_T             g_burst_tx_mode_enabled_earlier  = BFALSE;
#endif /* BURST_TX_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
BOOL_T             g_user_control_enabled           = BTRUE;
#else /* MAC_HW_UNIT_TEST_MODE */
BOOL_T             g_user_control_enabled           = BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef MEM_STRUCT_SIZES_INIT
/* The following array stores the names of all structures used in */
/* MAC software and their sizes calculated using sizeof()         */
struct_sizes_t g_struct_sizes[MEM_MAX_NUMBER_STRUCTS];
UWORD16        g_struct_count;
#endif /*MEM_STRUCT_SIZES_INIT*/

#ifdef MAC_HW_UNIT_TEST_MODE
WORD16             g_rssi_ex         = -100;
WORD16             g_adj_rssi_ex     = -100;
UWORD16            g_rssi_ex_count   = 1;
WORD16             g_target_rssi_11b = -100;
WORD16             g_target_rssi_11g = -100;
#endif /* MAC_HW_UNIT_TEST_MODE */
/* TxPower Calibration  */
UWORD8             g_wid_tx_cal          = BFALSE;
UWORD8             g_target_tssi_11a[16] = {0};
UWORD8             g_target_tssi_11b[16] = {0};

UWORD8             g_ht_enable	         = 1;

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef SUPP_11I
BOOL_T        g_int_supp_enable          = BTRUE;
#else /* SUPP_11I */
BOOL_T        g_int_supp_enable          = BFALSE;
#endif /* SUPP_11I */

//chenq add for cur tx rate 2012-10-17
UWORD8           g_cur_tx_rate = 0;

#include <linux/wakelock.h>
struct wake_lock reset_mac_lock; /*Keep awake when resetting MAC, by keguang 20130609*/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
struct wake_lock scan_ap_lock; /*Keep awake when scan ap, caisf add, 20130929*/
struct wake_lock deauth_err_lock; /*Keep awake when system error deauth, caisf add, 20130929*/
struct wake_lock handshake_frame_lock; /*Keep awake when rx handshake frames, caisf add, 20131004*/ 
ALARM_HANDLE_T *g_hs_wake_timer = 0; //add by caisf
struct wake_lock buffer_frame_lock; /*Keep awake when receiving buffered frames, by keguang 20130904*/
extern unsigned int txhw_idle(void);
static void buffer_frame_timerfunc(unsigned long data);
struct timer_list buffer_frame_timer = TIMER_INITIALIZER(buffer_frame_timerfunc, 0, 0);
int buffer_frame_count = 0;
static void buffer_frame_timerfunc(unsigned long data)
{
	if (txhw_idle() || !buffer_frame_count) {
		buffer_frame_count ? : pr_info("no more chance\n", __func__);
		pr_info("Let it go\n", __func__);
		wake_unlock(&buffer_frame_lock);
	}
	else {
		pr_info("Hold on\n", __func__);
		buffer_frame_count--;
		mod_timer(&buffer_frame_timer, jiffies + msecs_to_jiffies(30));
	}
}
#endif
#endif

//chenq add for protect event q 2012-11-01

//spinlock_t g_event_spin_lock;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_globals                                             */
/*                                                                           */
/*  Description   : This function initializes globals for AP and STA mode.   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_isr_handle                                         */
/*                  g_mgmt_alarm_handle                                      */
/*                  g_ack_policy                                             */
/*                  g_pcf_mode                                               */
/*                                                                           */
/*  Processing    : This function initializes common globals for AP and STA  */
/*                                                                           */
/*  Outputs       : All globals are initialized.                             */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_globals(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    UWORD8 count = 0;
    /* Initialize the Decryption Failure count for each Station */
    for(count=0; count < MAX_STA_SUPPORTED + 1; count++)	//modify by chengwg, 2013-03-16.
        g_decr_fail_cnt[count] = 0;

#else /* BSS_ACCESS_POINT_MODE */
    g_decr_fail_cnt = 0;
#endif /* BSS_ACCESS_POINT_MODE */
	TROUT_FUNC_ENTER;

    //g_mac_isr_handle     = 0;	//masked by chengwg.2013-03-15
    //xuan.yang, 2013-10-16, delete timer (g_mgmt_alarm_handle);
    delete_alarm(&g_mgmt_alarm_handle);
   // g_mgmt_alarm_handle  = 0;
    g_ack_policy         = 0;
    g_pcf_mode           = 0;
    g_bcst_ssid          = 0;
    g_addr_sdram         = 0;
    g_rssi               = -100;

	//chenq add snr
	g_snr                = 0;

	//chenq add 2012-10-30
	g_mac_event_timer    = NULL;
	
	//chenq add for cur tx rate 2012-10-17
	g_cur_tx_rate        = 0;

#ifdef MEM_STRUCT_SIZES_INIT

    g_struct_count = 0 ;

    strncpy(g_struct_sizes[g_struct_count].name, "start_rsp_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(start_rsp_t);

    strncpy(g_struct_sizes[g_struct_count].name, "q_entry_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(q_entry_t);

    strncpy(g_struct_sizes[g_struct_count].name, "q_struct_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(q_struct_t);

    strncpy(g_struct_sizes[g_struct_count].name, "table_elmnt_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(table_elmnt_t);

    strncpy(g_struct_sizes[g_struct_count].name, "msdu_desc_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(msdu_desc_t);

#ifdef MAC_802_11I
#ifndef MAC_HW_UNIT_TEST_MODE
    strncpy(g_struct_sizes[g_struct_count].name, "rsna_auth_persta_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(rsna_auth_persta_t);
#endif /* MAC_HW_UNIT_TEST_MODE */
    strncpy(g_struct_sizes[g_struct_count].name, "dot11RSNAStatsEntry_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(dot11RSNAStatsEntry_t);
#endif /*MAC_802_11I*/

#ifdef MAC_802_11N
    strncpy(g_struct_sizes[g_struct_count].name, "bfd_tx_pkt_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(bfd_tx_pkt_t);

    strncpy(g_struct_sizes[g_struct_count].name, "action_req_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(action_req_t);

    strncpy(g_struct_sizes[g_struct_count].name, "bfd_rx_pkt_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(bfd_rx_pkt_t);

    strncpy(g_struct_sizes[g_struct_count].name, "ba_tx_struct_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(ba_tx_struct_t);

    strncpy(g_struct_sizes[g_struct_count].name, "ba_rx_struct_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(ba_rx_struct_t);

    strncpy(g_struct_sizes[g_struct_count].name, "ht_tx_struct_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(ht_tx_struct_t);

    strncpy(g_struct_sizes[g_struct_count].name, "auth_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(auth_t);

    strncpy(g_struct_sizes[g_struct_count].name, "asoc_entry_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(asoc_entry_t);

    strncpy(g_struct_sizes[g_struct_count].name, "pm_buff_element_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(pm_buff_element_t);

    strncpy(g_struct_sizes[g_struct_count].name, "radius_node_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(radius_node_t);

    strncpy(g_struct_sizes[g_struct_count].name, "resp_pending_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(resp_pending_t);

    strncpy(g_struct_sizes[g_struct_count].name, "q_struct_t", MEM_MAX_STRUCT_NAME);
    g_struct_sizes[g_struct_count++].size = sizeof(q_struct_t);

#endif /* MAC_802_11N */
#endif /*MEM_STRUCT_SIZES_INIT*/

#ifdef MAC_HW_UNIT_TEST_MODE
    g_rssi_ex            = -100;
    g_adj_rssi_ex        = -100;
    g_rssi_ex_count      = 1;
#endif /* MAC_HW_UNIT_TEST_MODE */

    init_dev_spec_info();

    g_ht_enable	         = 1;
	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Set the suspend all data qs flag to 0 */
    g_is_suspend_all_data_qs = 0;
    TROUT_DBG4("set_device_mode to %d(not config)!\n", NOT_CONFIGURED);
	set_device_mode(NOT_CONFIGURED);
    
	//chenq add 2012-11-01
	//spin_lock_init(&g_event_spin_lock);
    TROUT_FUNC_EXIT;
}

#ifdef MEM_STRUCT_SIZES_INIT
void print_struct_size(void)
{
    UWORD32 indx = 0;
    TROUT_DBG4("Struct-Name \t Struct-Size\n\r");
    for(indx = 0; indx < g_struct_count; indx++)
    {
        TROUT_DBG4("%s \t %d\n\r",g_struct_sizes[indx].name,
        g_struct_sizes[indx].size);
    }
}
#endif /* MEM_STRUCT_SIZES_INIT */
