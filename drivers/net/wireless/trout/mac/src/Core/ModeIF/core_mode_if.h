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
/*  File Name         : core_mode_if.h                                       */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC mode interface.        */
/*                                                                           */
/*  List of Functions : AP/STA mode interface functions                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CORE_MODE_IF_H
#define CORE_MODE_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "autorate.h"
#include "cglobals.h"
#include "maccontroller.h"
#include "mib.h"
#include "index_util.h"
#include "qif.h"
#include "receive.h"

#ifdef IBSS_BSS_STATION_MODE
#include "autorate_sta.h"
#include "cglobals_sta.h"
#include "frame_sta.h"
#include "fsm_sta.h"
#include "management_sta.h"
#include "receive_sta.h"
#include "sta_prot_if.h"
#include "pm_sta.h"
#include "sme_sta.h"
#include "channel_sw.h"
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
#include "ap_prot_if.h"
#include "autorate_ap.h"
#include "cglobals_ap.h"
#include "frame_ap.h"
#include "fsm_ap.h"
#include "management_ap.h"
#include "pm_ap.h"
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test_utils.h"
#include "mh_test.h"
#include "prot_if.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef TX_ABORT_FEATURE
#ifdef TEST_TX_ABORT_FEATURE
#include "mh_test_txabort.h"
#endif /* TEST_TX_ABORT_FEATURE */
#endif /* TX_ABORT_FEATURE */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function calls the appropriate frame handling routines specific to   */
/* Access Point or Station mode of operation.                                */
INLINE void init_mac_fsm(mac_struct_t* mac)
{
#ifdef BURST_TX_MODE
    /* Start scaning only when burst tx mode is disabled */
    if(g_burst_tx_mode_enabled == BTRUE)
    {
       init_mac_fsm_bt(mac);
       return;
    }
#endif /* BURST_TX_MODE */

#ifdef IBSS_BSS_STATION_MODE
	//xuan yang, 2013-7-05, don't send disconnected state to supplicant in the suspend state.
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	if( g_wifi_suspend_status != wifi_suspend_suspend) 
	{
		//chenq add 2012-10-15
		set_mac_state(DISABLED);
	}
#else
	//chenq add 2012-10-15
	set_mac_state(DISABLED);
#endif
    init_mac_fsm_sta(mac);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    init_mac_fsm_ap(mac);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function calls the appropriate frame handling routines specific to   */
/* Access Point or Station mode of operation.                                */
INLINE void init_mac_globals(void)
{
    init_globals();

#ifdef IBSS_BSS_STATION_MODE
    init_globals_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    init_globals_ap();
#endif /* BSS_ACCESS_POINT_MODE */
    init_sta_table();

#ifdef MEM_STRUCT_SIZES_INIT
    print_struct_size();
#endif /* MEM_STRUCT_SIZES_INIT */
}

INLINE void init_mac_globals_plus(void)
{
#ifdef IBSS_BSS_STATION_MODE
    init_globals_sta_plus();
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function calls the appropriate routines to prepare the probe         */
/* response frame specific to Access Point or Station mode of operation.     */
INLINE UWORD16 prepare_probe_rsp(UWORD8* data, UWORD8* probe_req,
                                 BOOL_T is_p2p)
{
#ifdef IBSS_BSS_STATION_MODE
    return prepare_probe_rsp_sta(data, probe_req, is_p2p);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return prepare_probe_rsp_ap(data, probe_req, is_p2p);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
  return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}
//Begin:modified by wulei 2791 for bug 160423 on 2013-05-04
/* This function deletes all alarms used in MAC specific to Access Point or  */
/* Station mode of operation.                                                */
#if 0
INLINE void delete_mac_alarms(void)
{
#ifdef IBSS_BSS_STATION_MODE
#ifdef COMBO_SCAN
	//chenq add for combo scan 2013-03-13
	if(g_ap_combo_scan_timer)
	{
		hrtimer_cancel(g_ap_combo_scan_timer);
		kfree(g_ap_combo_scan_timer);
		g_ap_combo_scan_timer = NULL;
	}
#endif	
#endif

    delete_alarm(&g_mgmt_alarm_handle);
    delete_alarm(&g_ar_timer);


#ifdef IBSS_BSS_STATION_MODE
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: delete_mac_alarms: OBSS-ScanTimer Deleted\n\r");
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
    delete_alarm(&g_obss_scan_timer);
    delete_alarm(&g_psm_alarm_handle);
    delete_csw_alarms();
    delete_11h_alarms();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    delete_alarm(&g_aging_timer);
    delete_chan_mgmt_alarms_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef TEST_TX_ABORT_FEATURE
    delete_tx_abort_test_alarms();
#endif /* TEST_TX_ABORT_FEATURE */

	//chenq add 2012-10-30
	delete_alarm(&g_mac_event_timer);
}
#endif
extern void delete_alarm(ALARM_HANDLE_T ** handle);
INLINE void delete_mac_alarms(void)
{
#ifdef IBSS_BSS_STATION_MODE
#ifdef COMBO_SCAN
	//chenq add for combo scan 2013-03-13
	if(g_ap_combo_scan_timer)
	{
		hrtimer_cancel(g_ap_combo_scan_timer);
		kfree(g_ap_combo_scan_timer);
		g_ap_combo_scan_timer = NULL;
	}
#endif	
#endif

    delete_alarm(&g_mgmt_alarm_handle);
    delete_alarm(&g_ar_timer);


#ifdef IBSS_BSS_STATION_MODE
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: delete_mac_alarms: OBSS-ScanTimer Deleted\n\r");
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
    delete_alarm(&g_obss_scan_timer);
    delete_alarm(&g_psm_alarm_handle);
	delete_alarm(&g_csw_alarm_wait_for_AP);
    delete_11h_alarms();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    delete_alarm(&g_aging_timer);
    delete_chan_mgmt_alarms_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef TEST_TX_ABORT_FEATURE
    delete_tx_abort_test_alarms();
#endif /* TEST_TX_ABORT_FEATURE */

	//chenq add 2012-10-30
	delete_alarm(&g_mac_event_timer);
}
//End:modified by wulei 2791 for bug 160423 on 2013-05-04
/* This function extracts the source address and checks if the received      */
/* packet is sent and received by same station, which is possible in case of */
/* broadcast packet transmission from BSS STA.                               */
INLINE BOOL_T check_source_address(UWORD8 *sa)
{
#ifdef IBSS_BSS_STATION_MODE
    return check_source_address_sta(sa);
#else /* IBSS_BSS_STATION_MODE */
    return BFALSE;
#endif /* IBSS_BSS_STATION_MODE */

}

/* If the operating mode of the MAC is AP, then when a frame is received, it */
/* has to be filtered if its not associated                                  */
INLINE BOOL_T filter_wlan_rx_frame(wlan_rx_t *wlan_rx)
{
#ifdef IBSS_BSS_STATION_MODE
    return filter_wlan_rx_frame_sta(wlan_rx);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return filter_wlan_rx_frame_ap(wlan_rx);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Check if a packet must be buffered due to the service class policy */
INLINE BOOL_T is_serv_cls_buff_pkt(UWORD8 *entry, UWORD8 q_num, UWORD8 tid,
                                   UWORD8 *tx_dscr)
{
#ifdef IBSS_BSS_STATION_MODE
    return is_serv_cls_buff_pkt_sta((sta_entry_t *)entry, q_num, tid, tx_dscr);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return is_serv_cls_buff_pkt_ap((asoc_entry_t *)entry, q_num, tid, tx_dscr);
#endif /* BSS_ACCESS_POINT_MODE */


#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Check if a packet must be buffered due to the power save policy */
INLINE BOOL_T is_ps_buff_pkt(UWORD8 *entry, UWORD8 *da, UWORD8 *tx_dscr)
{
#ifdef IBSS_BSS_STATION_MODE
    /* Do nothing */
    return BFALSE;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return is_ps_buff_pkt_ap((asoc_entry_t *)entry, da, tx_dscr);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}


/* Returns the appropriate PS queue pointer for the given packet descriptor */
INLINE void *get_ps_q_ptr_mode(void *entry, UWORD8 tid, BOOL_T *ps_del_en_ac)
{
#ifdef BSS_ACCESS_POINT_MODE
    void *qh = NULL;
    *ps_del_en_ac = check_ac_is_del_en((asoc_entry_t *)entry, tid);

    if(BTRUE == *ps_del_en_ac)
        qh = &(((asoc_entry_t *)entry)->ps_q_del_ac);
    else
        qh = &(((asoc_entry_t *)entry)->ps_q_lgcy);

    return qh;
#else /* BSS_ACCESS_POINT_MODE */
    return NULL;
#endif /* BSS_ACCESS_POINT_MODE */

}

/* Update the power save flags */
INLINE void update_ps_flags(void *entry, UWORD16 num_elements_moved,
                            BOOL_T ps_del_en_ac)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_ps_flags_ap((asoc_entry_t *)entry, BFALSE, num_elements_moved,
        ps_del_en_ac);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* In AP/STA mode, check if the packet is from this BSS, once state is ENABLED */
INLINE BOOL_T check_bssid_match(mac_struct_t *mac, wlan_rx_t *wlan_rx)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    if((get_mac_state() == ENABLED) && (wlan_rx->type != CONTROL))
    {
        /* Management frames are processed (probe request may have broadcast */
        /* cast BSSID, coalescing may be required)                           */
        if(wlan_rx->type == MANAGEMENT)
            return BTRUE;

        if(mac_addr_cmp(wlan_rx->bssid, mget_bssid()) == BFALSE)
            return BFALSE;
    }

    return BTRUE;
#else /* MAC_HW_UNIT_TEST_MODE */
    return BTRUE;
#endif /* MAC_HW_UNIT_TEST_MODE */

}

/* This function sets the 802.11I/security information element in beacons   */
/* depending on the protocols in use.                                       */
INLINE UWORD8 set_sec_info_element(UWORD8 *data, UWORD16 index, UWORD8 supp)
{
#ifdef IBSS_BSS_STATION_MODE
    return set_sec_info_element_sta(data, index, supp);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return set_sec_info_element_ap(data, index);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Initializes the security Authentication node */
INLINE BOOL_T init_sec_auth_node(void **auth_ptr, UWORD8 *rsna, UWORD16 aid,
                                 void *auth_ptr_buff)
{
#ifdef IBSS_BSS_STATION_MODE
    return init_sec_auth_node_sta(auth_ptr, rsna, aid, auth_ptr_buff);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return init_sec_auth_node_ap(auth_ptr, rsna, aid, auth_ptr_buff);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Frees the security Authentication node */
INLINE void free_sec_auth_node(void **auth_ptr)
{
#ifdef IBSS_BSS_STATION_MODE
    free_sec_auth_node_sta(auth_ptr);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    free_sec_auth_node_ap(auth_ptr);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Stops the security Authentication node */
INLINE void stop_sec_auth_node(void)
{
#ifdef IBSS_BSS_STATION_MODE
    stop_sec_auth_node_sta();
    stop_sec_supp_node_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    stop_sec_auth_node_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE UWORD8 get_operating_bss_type(void)
{
#ifndef BSS_ACCESS_POINT_MODE
    return mget_DesiredBSSType();
#else /* BSS_ACCESS_POINT_MODE */
    return ANY_BSS;
#endif /* BSS_ACCESS_POINT_MODE */

}

INLINE void wlan_rx_control(UWORD8 *req)
{
#ifdef IBSS_BSS_STATION_MODE
    sta_enabled_rx_control((UWORD8 *)req);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    if(handle_ps_poll(req) == BFALSE)
    {
        /* Handle control frames other than PS Poll */
        ap_enabled_rx_control((UWORD8 *)req);
    }
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE BOOL_T is_txba_session_active(void *entry, UWORD8 tid)
{
    BOOL_T tx_ba_session = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    tx_ba_session = is_txba_session_active_sta((sta_entry_t *)entry, tid);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    tx_ba_session = is_txba_session_active_ap((asoc_entry_t *)entry, tid);
#endif /* BSS_ACCESS_POINT_MODE */

    return tx_ba_session;
}

/* This function updates the pending buffer parameters of the specified */
/* Tx-BA session.                                                       */
INLINE void update_txba_session_params(void *entry, UWORD8 tid,
                                       UWORD16 timeout, UWORD8 max_num)
{
#ifdef IBSS_BSS_STATION_MODE
    update_txba_session_params_sta((sta_entry_t *)entry, tid, timeout, max_num);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    update_txba_session_params_ap((asoc_entry_t *)entry, tid, timeout, max_num);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Functions to handle transmission success or failure for auto rate based   */
/* on the mode of operation.                                                 */
INLINE void ar_tx_failure(void *entry)
{
    if(is_autorate_enabled() == BTRUE)
    {
#ifdef IBSS_BSS_STATION_MODE
        ar_tx_failure_sta((sta_entry_t*)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
        ar_tx_failure_ap((asoc_entry_t*)entry);
#endif /* BSS_ACCESS_POINT_MODE */
    }
}

INLINE void ar_tx_success(void *entry, UWORD8 retry_count)
{
    if(is_autorate_enabled() == BTRUE)
    {
#ifdef IBSS_BSS_STATION_MODE
        ar_tx_success_sta((sta_entry_t*)entry, retry_count);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
        ar_tx_success_ap((asoc_entry_t*)entry, retry_count);
#endif /* BSS_ACCESS_POINT_MODE */
    }
}

INLINE void do_per_entry_ar(void *te)
{
#ifdef AUTORATE_FEATURE
    if(is_autorate_enabled() == BTRUE)
    {
#ifdef IBSS_BSS_STATION_MODE
   sta_entry_t entry;
   sta_entry_t *update_entry;
   
   if (te == NULL){
       return ;
   }

   memcpy(&entry, te, sizeof(sta_entry_t));
   do_sta_entry_ar(&entry);

   /*
    * leon liu added, fix reboot bug, during do_sta_entry_ar,
    * AP send deauth frame and entry is deleted, this will
    * result in NULL pointer access.
   */
   update_entry = find_entry(mget_bssid());
   if (update_entry){
       pr_info("%s: updateing sta_entry\n", __func__);
       update_entry->tx_rate_index = entry.tx_rate_index;
       update_entry->ar_stats = entry.ar_stats;
       update_entry->ht_hdl.tx_sgi = entry.ht_hdl.tx_sgi;
       update_entry->ht_hdl.tx_mcs_index = entry.ht_hdl.tx_mcs_index;
       update_entry->retry_rate_set[0] = entry.retry_rate_set[0];
       update_entry->retry_rate_set[1] = entry.retry_rate_set[1];
   }
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
        do_asoc_entry_ar((asoc_entry_t*)te);
#endif /* BSS_ACCESS_POINT_MODE */
    }
#endif /* AUTORATE_FEATURE */
}

INLINE void intialize_sec_km(void)
{
#ifdef IBSS_BSS_STATION_MODE
    intialize_sec_km_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    intialize_sec_km_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}


INLINE void stop_sec_km(void)
{
#ifdef IBSS_BSS_STATION_MODE
    stop_sec_km_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    stop_sec_km_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE void delete_element(void *element)
{
#ifdef IBSS_BSS_STATION_MODE
    delete_sta_entry(element);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    delete_asoc_entry(element);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Check privacy bit of capability information */
INLINE BOOL_T check_bss_mac_privacy(UWORD16 cap_info)
{
#ifdef IBSS_BSS_STATION_MODE
    return check_bss_mac_privacy_sta(cap_info);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return check_bss_mac_privacy_ap(cap_info);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function checks the Capability Info field for Misc features */
INLINE BOOL_T check_misc_cap_info(UWORD16 cap_info)
{
#ifdef IBSS_BSS_STATION_MODE
    return check_misc_sta_cap_info_prot(cap_info);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return BTRUE;
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BTRUE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Set the QoS control field depending on the mode */
INLINE void set_qos_control(UWORD8 *header, UWORD8 tid, UWORD8 ap)
{
#ifdef IBSS_BSS_STATION_MODE
    set_qos_control_field_sta(header, tid, ap);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    set_qos_control_field_ap(header, tid, ap);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    set_qos_control_field_ut(header, tid, ap);
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Handle the authentication/arp packets */
INLINE BOOL_T check_sec_auth_pkt(UWORD8 *buffer, UWORD32 len)
{
#ifdef IBSS_BSS_STATION_MODE
    return BFALSE;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return check_sec_auth_pkt_ap(buffer, len);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

INLINE void init_sec_auth_policy(void)
{
#ifdef IBSS_BSS_STATION_MODE
#ifdef IBSS_11I
    init_sec_auth_sta();
#endif /* IBSS_11I */
    init_sec_supp_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    init_sec_auth_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE void compute_install_sec_key(void)
{
#ifdef IBSS_BSS_STATION_MODE
    compute_install_sec_key_sta();
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    compute_install_sec_key_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Initialize the DTIM period value in the bitmap for AP mode */
INLINE void init_dtim_period(UWORD8 val)
{
#ifdef IBSS_BSS_STATION_MODE
    /* Do nothing */
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    g_vbmap[DTIM_PERIOD_OFFSET] = mget_DTIMPeriod();
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Get the number of non-ERP stations */
INLINE UWORD8 get_num_non_erp_sta(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    return g_num_sta_non_erp;
#else  /* BSS_ACCESS_POINT_MODE */
    return 0;
#endif /* BSS_ACCESS_POINT_MODE */

}

/* Get the number of long preamble support only stations */
INLINE UWORD8 get_num_sta_no_short_pream(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    return g_num_sta_no_short_pream;
#else/* BSS_ACCESS_POINT_MODE */
    return 0;
#endif /* BSS_ACCESS_POINT_MODE */

}

/* Start MAC daemons for Linux */
INLINE void start_mac_daemons(void)
{
#ifdef OS_LINUX_CSL_TYPE
#ifdef IBSS_BSS_STATION_MODE

#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    start_sec_daemon_ap();
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* OS_LINUX_CSL_TYPE */
}

/* Stop MAC daemons for Linux */
INLINE void stop_mac_daemons(void)
{
#ifdef OS_LINUX_CSL_TYPE
#ifdef IBSS_BSS_STATION_MODE

#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    stop_sec_daemon_ap();
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* OS_LINUX_CSL_TYPE */
}

/* This function sets the 1X Supplicant Username */
INLINE void set_supp_username(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update_sta();
    set_supp_username_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets the 1X Supplicant Username */
INLINE UWORD8 *get_supp_username(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return get_supp_username_sta();
#else /*  IBSS_BSS_STATION_MODE */
    return NULL;
#endif /*  IBSS_BSS_STATION_MODE */

}

/* This function sets the 1X Supplicant Password */
INLINE void set_supp_password(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update_sta();
    set_supp_password_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets the 1X Supplicant Username */
INLINE UWORD8 *get_supp_password(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return get_supp_password_sta();
#else  /* IBSS_BSS_STATION_MODE */
    return NULL;
#endif /* IBSS_BSS_STATION_MODE */

}

/* This function returns the site survey results */
INLINE UWORD8 *get_site_survey_results(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return get_site_survey_results_sta();
#else  /* IBSS_BSS_STATION_MODE */
    return NULL;
#endif /* IBSS_BSS_STATION_MODE */
}

/* Initiate the join */
INLINE void set_join_req(UWORD8 val)
{
#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update_sta();
    set_join_req_sta(val);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return;
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Initiate the join */
INLINE void set_start_scan_req(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update_sta();
    set_start_scan_req_sta(val);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return;
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE UWORD8 get_start_scan_req(void)
{
#ifdef IBSS_BSS_STATION_MODE
        return (g_scan_source & USER_SCAN);
#else
    return 0;
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void set_1x_key(UWORD8* val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_1x_key_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE UWORD8* get_1x_key(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    return get_1x_key_ap();
#else /* BSS_ACCESS_POINT_MODE */
    return NULL;
#endif /* BSS_ACCESS_POINT_MODE */

}

INLINE void set_1x_serv_addr(UWORD32 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_1x_serv_addr_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE UWORD32 get_1x_serv_addr(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    return get_1x_serv_addr_ap();
#else /* BSS_ACCESS_POINT_MODE */
    return 0;
#endif /* BSS_ACCESS_POINT_MODE */

}

/* This function returns the observed power levels of the received signal on */
/* all channels to the host in the format required by the host.              */
INLINE WORD8* get_rx_power_level(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return get_rx_power_level_sta();
#else /* IBSS_BSS_STATION_MODE */
    return NULL;
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets the HT Capabilities Infomation element depending upon */
/* the mode of operation.                                                   */
INLINE void set_ht_capinfo(UWORD8 *data, TYPESUBTYPE_T frame_type)
{
#ifdef IBSS_BSS_STATION_MODE
    set_ht_capinfo_sta(data, frame_type);
#endif /*IBSS_BSS_STATION_MODE*/

#ifdef BSS_ACCESS_POINT_MODE
    set_ht_capinfo_ap(data, frame_type);
#endif /*BSS_ACCESS_POINT_MODE*/
}

INLINE UWORD8 get_sta_index(void *entry)
{
    UWORD8 sta_index = 0;
#ifdef IBSS_BSS_STATION_MODE
    sta_index = ((sta_entry_t *)entry)->sta_index;
#endif /*IBSS_BSS_STATION_MODE*/

#ifdef BSS_ACCESS_POINT_MODE
    sta_index = ((asoc_entry_t *)entry)->sta_index;
#endif /*BSS_ACCESS_POINT_MODE*/

    return sta_index;
}

INLINE void *get_ba_handle_entry(void *entry)
{
    void *ba_entry_handle = NULL;
#ifdef IBSS_BSS_STATION_MODE
    ba_entry_handle = get_ba_handle_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    ba_entry_handle = get_ba_handle_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return ba_entry_handle;
}

INLINE void *get_ht_handle_entry(void *entry)
{
    void *ht_entry_handle = NULL;
#ifdef IBSS_BSS_STATION_MODE
    ht_entry_handle = get_ht_handle_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    ht_entry_handle = get_ht_handle_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return ht_entry_handle;
}

INLINE BOOL_T is_ht_capable(void *entry)
{
    BOOL_T ht_capable = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    ht_capable = is_ht_capable_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    ht_capable = is_ht_capable_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return ht_capable;
}

INLINE BOOL_T is_imm_ba_capable(void *entry)
{
    BOOL_T imm_ba_capable = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    imm_ba_capable = is_imm_ba_capable_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    imm_ba_capable = is_imm_ba_capable_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return imm_ba_capable;
}

INLINE void set_ht_ps_params(UWORD8 *tx_dscr, void *entry, UWORD8 rate)
{
#ifdef IBSS_BSS_STATION_MODE
   set_ht_ps_params_sta(tx_dscr, (sta_entry_t*)entry, rate);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    set_ht_ps_params_ap(tx_dscr, (asoc_entry_t*)entry, rate);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function checks the power saving mode of the station */
INLINE UWORD8 check_ps_mode(UWORD8 priority)
{
    UWORD8 ps_bit = 0;

#ifdef IBSS_BSS_STATION_MODE
    /* Process the event to get the appropriate power save bit to be set */
    ps_bit = psm_handle_tx_packet_sta(priority);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    /* Do nothing */
#endif /* BSS_ACCESS_POINT_MODE */

    return ps_bit;
}

/* This function checks whether the receiver is HT capable */
INLINE BOOL_T is_htc_capable(void *entry)
{
    BOOL_T htc_capable = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    htc_capable = is_htc_capable_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    htc_capable = is_htc_capable_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return htc_capable;
}

/* This fucntion updates the service class parameter of a MSDU */
INLINE void update_serv_class_prot(void *entry, UWORD8 tid,
                                   UWORD8 *serv_class)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_serv_class_prot_ap(entry, tid, serv_class);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    update_serv_class_prot_sta(entry, tid, serv_class);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Sets the SMPS mode for 11n MAC based on the mode of operation */
INLINE void set_11n_smps_mode(UWORD8 val)
{
#ifdef IBSS_BSS_STATION_MODE
    set_11n_smps_mode_sta(val);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    /* Currently SMPS mode for AP is not supported */
    return;
#endif /* BSS_ACCESS_POINT_MODE */
}

//chenq add for cur tx rate 2012-10-17
INLINE void set_cur_tx_rate(UWORD8 val)
{
	g_cur_tx_rate = val;
}

INLINE UWORD8 get_cur_tx_rate(void)
{
	return g_cur_tx_rate;
}


INLINE UWORD8 get_tx_rate_to_sta(void *entry)
{
    UWORD8 tx_rate = get_curr_tx_rate();
#ifdef IBSS_BSS_STATION_MODE
    tx_rate = get_tx_rate_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    tx_rate = get_tx_rate_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return tx_rate;
}

#ifdef AUTORATE_FEATURE
/* This function returns the operational rates table pointer for an entry */
INLINE rate_t *get_op_rates_entry(void *entry)
{
    rate_t *op_rates_entry = NULL;
#ifdef IBSS_BSS_STATION_MODE
    op_rates_entry = get_op_rates_entry_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    op_rates_entry = get_op_rates_entry_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return op_rates_entry;
}

INLINE UWORD8 get_ht_tx_rate_to_sta(void *entry)
{
    UWORD8 tx_rate = 0x87; /* MCS7 */
#ifdef IBSS_BSS_STATION_MODE
    tx_rate = get_ht_tx_rate_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    tx_rate = get_ht_tx_rate_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return tx_rate;
}

/* This function checks if a given MCS is supported by the given STA entry */
INLINE BOOL_T is_mcs_supp(void *entry, UWORD8 mcs_index)
{
    BOOL_T mcs_supported = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    mcs_supported = is_mcs_supp_sta((sta_entry_t *)entry, mcs_index);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    mcs_supported = is_mcs_supp_ap((asoc_entry_t *)entry, mcs_index);
#endif /* BSS_ACCESS_POINT_MODE */

    return mcs_supported;
}

/* This function updates the maximum supported rate index */
INLINE void update_max_rate_idx(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    update_max_rate_idx_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    update_max_rate_idx_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function updates the minimum supported rate index */
INLINE void update_min_rate_idx(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    update_min_rate_idx_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    update_min_rate_idx_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function initializes the transmit rate index */
INLINE void init_tx_rate_idx(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    init_tx_rate_idx_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    init_tx_rate_idx_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function re-initializes the transmit rate index */
INLINE void reinit_tx_rate_idx(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    reinit_tx_rate_idx_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    reinit_tx_rate_idx_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE BOOL_T is_max_rate(void *entry)
{
    BOOL_T ret_val = BFALSE;
#ifdef IBSS_BSS_STATION_MODE
    ret_val = is_max_rate_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    ret_val = is_max_rate_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return ret_val;
}
#endif /* AUTORATE_FEATURE */

/* Update MIB for security failures */
INLINE void update_sec_fail_stats(CIPHER_T ct, UWORD8 status, UWORD8 *sa,
                                  UWORD8 *da)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_sec_fail_stats_ap(ct,status,sa);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    update_sec_fail_stats_sta(ct,status,sa,da);
#endif /* IBSS_BSS_STATION_MODE */

    if((status == ICV_FAILURE) && (mget_PrivacyInvoked() == TV_TRUE)
        && (ct == WEP40 || ct == WEP104))
    {
        /*Increment the MIB Count for dot11WepICVErrorCount */
        mincr_WEPICVErrorCount();
    }
}

/* This function updates the security statistics on a successful reception */
INLINE void update_sec_success_stats(void *entry, CIPHER_T ct, UWORD8 status)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_sec_success_stats_ap((asoc_entry_t *)entry, ct, status);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    update_sec_success_stats_sta((sta_entry_t *)entry, ct, status);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the retry rate set that is saved in the station or  */
/* association entry                                                         */
INLINE UWORD32 *get_retry_rate_set(void *entry)
{
    UWORD32 *retry_rate = 0;
#ifdef IBSS_BSS_STATION_MODE
    retry_rate = ((sta_entry_t *)entry)->retry_rate_set;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    retry_rate = ((asoc_entry_t *)entry)->retry_rate_set;
#endif /* BSS_ACCESS_POINT_MODE */

    return retry_rate;
}

/* This function returns the current rate index of the station */
INLINE UWORD8 get_tx_rate_index(void *entry)
{
    UWORD8 tx_rate_index = 0;
#ifdef IBSS_BSS_STATION_MODE
    tx_rate_index = ((sta_entry_t *)entry)->tx_rate_index;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    tx_rate_index = ((asoc_entry_t *)entry)->tx_rate_index;
#endif /* BSS_ACCESS_POINT_MODE */

    return tx_rate_index;
}

#ifdef AUTORATE_FEATURE
/* This function returns the minimum supported rate index of the station */
INLINE UWORD8 get_min_rate_index(void *entry)
{
    UWORD8 min_support_rate_index = 0;
#ifdef IBSS_BSS_STATION_MODE
    min_support_rate_index = ((sta_entry_t *)entry)->min_rate_index;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    min_support_rate_index = ((asoc_entry_t *)entry)->min_rate_index;
#endif /* BSS_ACCESS_POINT_MODE */

    return min_support_rate_index;
}
#endif /* AUTORATE_FEATURE */

/* Post an event for radar detection */
INLINE void post_radar_detect_event(void)
{

    PRINTD("Radar Detected\n");

#ifdef BSS_ACCESS_POINT_MODE
    post_radar_detect_event_11h_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    post_radar_detect_event_11h_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the signal quality based on mode of operation */
INLINE UWORD16 get_signal_quality(void)
{
    UWORD16 signal_quality = 0;
#ifdef IBSS_BSS_STATION_MODE
    signal_quality = g_signal_quality;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    signal_quality = 0;
#endif /* BSS_ACCESS_POINT_MODE */

    return signal_quality;
}

/* This function performs seurity checks on the received MPDU */
INLINE BOOL_T check_sec_rx_mpdu(UWORD8 *entry, CIPHER_T ct, UWORD8 *msa,
                                UWORD8 *pn_val)
{
#ifdef IBSS_BSS_STATION_MODE
    return check_sec_rx_mpdu_sta(entry, ct, msa, pn_val);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    return check_sec_rx_mpdu_ap(entry, ct, msa, pn_val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function checks whether the Receive Service Class for the specified */
/* entry is Block-Ack.                                                      */
INLINE BOOL_T is_rx_ba_service_class(void *entry, UWORD8 tid)
{
    BOOL_T retval = BFALSE;

#ifdef IBSS_BSS_STATION_MODE
    retval = is_rx_ba_service_class_sta((sta_entry_t *)entry, tid);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    retval = is_rx_ba_service_class_ap((asoc_entry_t *)entry, tid);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function checks whether a MAC reset is allowed to be done under the */
/* current operating state.                                                 */
INLINE BOOL_T allow_mac_reset(void)
{
    BOOL_T retval = BTRUE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = allow_mac_reset_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = allow_mac_reset_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* This function returns the number of TX-AMPDU sessions currently active */
/* with the specified station.                                            */
INLINE UWORD8 get_num_tx_ampdu_sessions(void *entry)
{
    UWORD8 retval = 0;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_num_tx_ampdu_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_num_tx_ampdu_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function increments the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void incr_num_tx_ampdu_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    incr_num_tx_ampdu_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    incr_num_tx_ampdu_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function decrements the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void decr_num_tx_ampdu_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    decr_num_tx_ampdu_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    decr_num_tx_ampdu_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function returns the number of Block-Ack TX sessions setup with */
/* the specified station.                                               */
INLINE UWORD8 get_num_tx_ba_sessions(void *entry)
{
    UWORD8  retval   = 0;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_num_tx_ba_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_num_tx_ba_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_tx_ba_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    incr_num_tx_ba_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    incr_num_tx_ba_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_tx_ba_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    decr_num_tx_ba_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    decr_num_tx_ba_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_rx_ba_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    incr_num_rx_ba_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    incr_num_rx_ba_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_rx_ba_sessions(void *entry)
{
#ifdef IBSS_BSS_STATION_MODE
    decr_num_rx_ba_sessions_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    decr_num_rx_ba_sessions_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */
}

#ifdef AUTORATE_FEATURE
/* This function returns the minimum non-zero TXOP limit configured across   */
/* all ACs. If all TXOP-Limits are zero, then 0xFFFF is returned.            */
INLINE UWORD16 get_min_non_zero_txop(void *entry)
{
    UWORD16 min_txop = 0xFFFF;

#ifdef BSS_ACCESS_POINT_MODE
    min_txop = get_min_non_zero_txop_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    min_txop = get_min_non_zero_txop_sta((sta_entry_t *)entry);
#endif /* IBSS_BSS_STATION_MODE */

    return min_txop;
}

#endif /* AUTORATE_FEATURE */

/* This function updates the receiver address in the incoming action request */
/* automatically as required depending on the mode of operation.             */
INLINE void update_action_req_ra(UWORD8 *ra)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_action_req_ra_ap(ra);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    update_action_req_ra_sta(ra);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the STA index for a given entry */
INLINE UWORD8 get_entry_sta_index(void *te)
{
    UWORD8 retval = 0;

#ifdef IBSS_BSS_STATION_MODE
    retval = (((sta_entry_t*)te)->sta_index);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    retval = (((asoc_entry_t*)te)->sta_index);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function updates the Secondary Channel Setting based upon the current */
/* settings of Primary Channel, Secondary Channel and applicable Regulatory   */
/* Domain.                                                                    */
INLINE void update_operating_channel(void)
{
/* TBD: Implement Support for Non-HUT Mode of operation */
#ifdef MAC_HW_UNIT_TEST_MODE
    update_operating_channel_hut();
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Start a rejoin timer if not running */
INLINE void start_rejoin_timer(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    /* Do nothing */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    start_rejoin_timer_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

/* Initiate a Join with configuration saved in the device */
INLINE void join_config_saved_in_device(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    /* Do nothing */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    join_config_saved_in_device_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

/* Disable any protocols in progress */
INLINE BOOL_T allow_sys_restart_prot(ERROR_CODE_T sys_error)
{
    BOOL_T retval = BTRUE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = allow_sys_restart_prot_ap(sys_error);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = allow_sys_restart_prot_sta(sys_error);
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* This function checks if the received non directed frame is useful for us */
INLINE BOOL_T useful_non_directed(mac_struct_t *mac, TYPESUBTYPE_T frame_st)
{
#ifdef IBSS_BSS_STATION_MODE
    return useful_non_directed_sta(mac, frame_st);
#else /* IBSS_BSS_STATION_MODE */
    return BFALSE;
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the station related information for the specified */
/* address.                                                                */
INLINE UWORD16 get_sta_join_info(UWORD8* sta_addr, UWORD8* ptr)
{
#ifdef BSS_ACCESS_POINT_MODE
      return get_sta_join_info_ap(NULL, sta_addr, ptr);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
      return 0;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
      return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function returns the station related information for all the stations*/
/* associated with the AP.                                                   */
INLINE UWORD16 get_all_sta_join_info(UWORD8* ptr, UWORD16 max_len)
{
#ifdef BSS_ACCESS_POINT_MODE
      return get_all_sta_info_ap(ptr, max_len);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
      return 0;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
      return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Check for consistency of build parameters */
INLINE void check_build_params(void)
{
#ifdef DEBUG_MODE

#ifdef BSS_ACCESS_POINT_MODE
      check_build_params_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
      check_build_params_sta();
#endif /* IBSS_BSS_STATION_MODE */

#endif /* DEBUG_MODE */
}

/* This function checks and returns BTRUE or BFALSE if WID write request is  */
/* to be processed                                                           */
INLINE BOOL_T process_wid_write_prot(UWORD8 *req, UWORD16 msg_len)
{
    BOOL_T retval = BTRUE;
#ifdef BSS_ACCESS_POINT_MODE
      retval = process_wid_write_prot_ap(req, msg_len);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

INLINE void wps_handle_config_update(void)
{
#ifdef BSS_ACCESS_POINT_MODE
      wps_handle_config_update_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE void sys_init_prot(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    sys_init_prot_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

/*****************************************************************************/
/* WPS Related Configuration Functions                                       */
/*****************************************************************************/
/* This function sets the WPS Mode to Enabled/Disabled */
INLINE void set_wps_enable(UWORD8 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_wps_enable_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_wps_enable_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the WPS Enabled status */
INLINE BOOL_T get_wps_enable(void)
{
    BOOL_T retval = BFALSE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_enable_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_enable_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* This function gets the WPS Protocol type configured */
INLINE UWORD8 get_wps_prot(void)
{
    UWORD8 retval = 0;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_prot_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_prot_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* This function sets the WPS Protocol type */
INLINE void set_wps_prot(UWORD8 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_wps_prot_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_wps_prot_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets the WPS Protocol type */
INLINE void set_oob_reset_req(UWORD8 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    handle_oob_reset_req_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function return the WPS Password ID programmed */
INLINE UWORD16 get_wps_pass_id(void)
{
    UWORD16 retval = 0;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_pass_id_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_pass_id_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

INLINE void set_wps_pass_id(UWORD16 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_wps_pass_id_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_wps_pass_id_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE UWORD16 get_wps_config_method(void)
{
    UWORD16 retval = 0;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_config_method_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_config_method_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

INLINE void set_wps_config_method(UWORD16 val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_wps_config_method_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_wps_config_method_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE UWORD8 *get_wps_pin(void)
{
    UWORD8 *retval = NULL;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_pin_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_pin_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

INLINE void set_wps_pin(UWORD8 *val)
{
#ifdef BSS_ACCESS_POINT_MODE
        set_wps_pin_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
        set_wps_pin_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}
/* Functions specific to internal Enrollee */
INLINE UWORD8 get_wps_dev_mode(void)
{
    UWORD8 retval = 0;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_dev_mode_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

#if 0 /* Not Used */
INLINE UWORD8 get_mac_cred_wps_status(void)
{
    UWORD8 retval = 0;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_mac_cred_wps_status_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}
#endif /* 0 */

INLINE void set_wps_dev_mode(UWORD8 val)
{
#ifdef IBSS_BSS_STATION_MODE
    set_wps_dev_mode_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void set_wps_cred_list(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
        set_wps_cred_list_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to get AC parameter values for STA side*/
INLINE UWORD8* get_wps_cred_list(void)
{
    UWORD8 *retval = NULL;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_wps_cred_list_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* Check the device mode */
INLINE BOOL_T is_dev_mode_host_managed(void)
{
    BOOL_T retval = BFALSE;

#ifdef IBSS_BSS_STATION_MODE
    retval = is_dev_mode_host_managed_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* This function handles updates done in device information by the user */
INLINE void update_device_specific_info(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_device_specific_info_ap();
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function handles update to WPS Credential by the user */
INLINE void handle_wps_cred_update(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    handle_wps_cred_update_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

/* Indicate that WID Config Write Status to Protocol */
INLINE void indicate_config_write_prot(BOOL_T val)
{
#ifdef IBSS_BSS_STATION_MODE
    indicate_config_write_prot_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function indicate start/stop of WID restoration to all the Protocols */
INLINE void indicate_wid_restore_prot(BOOL_T val)
{
#ifdef BSS_ACCESS_POINT_MODE
    indicate_wid_restore_prot_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function returns the WPS Capability of the device */
INLINE BOOL_T is_sta_wps_cap(void *entry)
{
    BOOL_T retval = BFALSE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = is_sta_wps_cap_ap((asoc_entry_t *)entry);
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function returns the WPS implemented status */
INLINE BOOL_T get_wps_implemented(void)
{
    BOOL_T retval = BTRUE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = get_wps_implemented_ap();
#endif /* BSS_ACCESS_POINT_MODE */

    return retval;
}

/* This function sets the WPS implemented status to specified input value */
INLINE void set_wps_implemented(BOOL_T val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_wps_implemented_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function sets the WPS mode with specified input value */
INLINE void enable_wps_mode(BOOL_T val)
{
#ifdef BSS_ACCESS_POINT_MODE
    enable_wps_mode_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function calls the function to add the group info P2P attribute      */
/* depending upon the mode of operation. */
INLINE UWORD16 add_p2p_grp_info_attr_mode(UWORD8* ptr, UWORD16 index)
{
    UWORD16 retval = 0;
#ifdef BSS_ACCESS_POINT_MODE
    retval = add_p2p_grp_info_attr_prot(ptr, index);
#endif /* BSS_ACCESS_POINT_MODE */

/* If the device is operating in the client mode then the group info         */
/* attribute is not added. */
    return retval;
}

/* This function calls the function  to start the scan process depending on */
/* the mode of operation                                                    */
INLINE void p2p_start_invit_scan_req_mode(void)
{
#ifdef IBSS_BSS_STATION_MODE
    g_wps_scan_req_from_user = BTRUE;
    set_start_scan_req(USER_SCAN);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function extracts the config method sent in the probe response frame */
/* and checks if it is a valid config method                                 */

INLINE void p2p_process_prob_rsp_enr_mode(UWORD8 *msa, UWORD16 rx_len,
                                     UWORD16 *pref_confg_meth)
{
#ifdef IBSS_BSS_STATION_MODE
    p2p_process_prob_rsp_sta_prot(msa, rx_len, pref_confg_meth);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function calls the function to update the persistent credential */
/* list depending upon the mode of operation                            */
INLINE void update_persist_cred_list_mode(UWORD8 *if_addr)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_persist_cred_list_prot_ap(if_addr);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function sets the P2P NOA schedule in AP mode. In STA mode nothing   */
/* is done since the NOA schedule is adopted from the GO.                    */
INLINE void set_p2p_noa_sched(UWORD8 *val)
{
#ifdef BSS_ACCESS_POINT_MODE
    set_p2p_noa_sched_prot_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE void process_scan_itr_comp_mode(mac_struct_t *mac)
{
#ifdef IBSS_BSS_STATION_MODE
    /* End the scan and call the function to handle the timeout */
    process_scan_itr_comp(mac);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void update_entry_mode(UWORD8 count)
{
#ifdef BSS_ACCESS_POINT_MODE
    update_entry_prot(count);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function initializes the current Regulatory Domain. For IBSS STA and */
/* AP mode, it is set to the default regulatory domain. For BSS STA it is    */
/* set to No regulatory domain. Once it associates with an AP, appropriate   */
/* regulatory domain parameters shall be adapted.                            */
INLINE void init_current_reg_domain(void)
{
#ifdef IBSS_BSS_STATION_MODE
    set_current_reg_domain(RD0); // caisf set to no country regulatory domain 
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    set_current_reg_domain(RD1); // caisf set to "US " regulatory domain
#endif /* BSS_ACCESS_POINT_MODE */

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_HW_UNIT_TEST_MODE
    set_current_reg_domain(DEFAULT_REG_DOMAIN);
#endif /* MAC_HW_UNIT_TEST_MODE */

}


/* This function sets P2P Enable depending on the mode of operation. In STA  */
/* mode this can be set to enable P2P discovery. In P2P device mode (STA i.e.*/
/* P2P Client or AP i.e. P2P GO) this can be disabled to become non-P2P STA. */
INLINE void set_p2p_enable_mode(UWORD8 p2p_en)
{
    /* Input Sanity Check */
    if(p2p_en > 1)
        return;

#ifdef IBSS_BSS_STATION_MODE
    set_p2p_enable_sta(p2p_en);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    set_p2p_enable_ap(p2p_en);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function sets Persistent Capability depending on mode of operation */
INLINE void set_p2p_persist_grp_mode(BOOL_T val)
{
	/* Input Sanity Check */
    if(val > 1)
        return;

#ifdef BSS_ACCESS_POINT_MODE
    set_p2p_persist_grp_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_p2p_persist_grp_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets Device discoverability depending on mode of operation */
INLINE void set_p2p_discoverable_mode(BOOL_T val)
{
	/* Input Sanity Check */
    if(val > 1)
        return;

#ifdef BSS_ACCESS_POINT_MODE
    /* This is a read-only WID in GO mode */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_p2p_persist_grp_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets Intra-BSS Capability depending on mode of operation */
INLINE void set_p2p_intra_bss_mode(BOOL_T val)
{
	/* Input Sanity Check */
    if(val > 1)
        return;

#ifdef BSS_ACCESS_POINT_MODE
    set_p2p_intra_bss_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    set_p2p_intra_bss_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function sets P2P CT Window depending on mode of operation */
INLINE void set_p2p_ct_window(UWORD8 val)
{
	/* Input Sanity Check */
    if(val > 127)
        return;

#ifdef BSS_ACCESS_POINT_MODE
    set_p2p_ct_window_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    /* Do nothing */
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function restores required WID values to required default values at  */
/* the time of switching operating mode.                                     */
INLINE void restore_default_mode_wids(void)
{
#ifdef BSS_ACCESS_POINT_MODE
    restore_default_mode_wids_ap();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    restore_default_mode_wids_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

#endif /* CORE_MODE_IF_H */
