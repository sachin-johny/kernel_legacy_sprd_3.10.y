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
/*  File Name         : management_sta.c                                     */
/*                                                                           */
/*  Description       : This file contains the functions related to the MAC  */
/*                      management routines. The request updating and        */
/*                      mib updating will be done in these functions.        */
/*                                                                           */
/*  List of Functions : update_scan_req_params                               */
/*                      update_join_req_params                               */
/*                      update_auth_req_params                               */
/*                      update_asoc_req_params                               */
/*                      update_start_req_params                              */
/*                      update_scan_response                                 */
/*                      update_bss_dscr_set                                  */
/*                      scan_channel                                         */
/*                      set_ibss_param                                       */
/*                      generate_bssid                                       */
/*                      init_sta_entry                                       */
/*                      delete_sta_entry                                     */
/*                      reset_sta_entry                                      */
/*                      check_bss_capability_mac                             */
/*                      filter_host_rx_frame_sta                             */
/*                      filter_wlan_rx_frame_sta                             */
/*                      get_rx_power_level_sta                               */
/*                      get_site_survey_results_sta                          */
/*                      set_join_req_sta                                     */
/*                      disconnect_sta                                       */
/*                      handle_mlme_rsp_sta                                  */
/*                      is_sta_connected                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_sta.h"
#include "frame_sta.h"
#include "management_sta.h"
#include "mh.h"
#include "rf_if.h"
#include "sta_prot_if.h"
#include "controller_mode_if.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "iconfig.h"
#include "mac_init.h"
#include "mib.h"
#include "itm_wifi_iw.h"
#include "qmu_tx.h"
extern unsigned int check_tpc_switch_from_nv(void);
extern void  MxdRfGetRfMode(unsigned int * pFlagFmBit2BtBit1WfBit0 );
/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_scan_req_params                                   */
/*                                                                           */
/*  Description   : This function updates the required parameters for scan.  */
/*                                                                           */
/*  Inputs        : 1) Scan request structure                                */
/*                                                                           */
/*  Globals       : g_channel_index                                          */
/*                  g_num_channels                                           */
/*                                                                           */
/*  Processing    : The global scan request structure is updated according   */
/*                  to the input scan request structure. Few MIB parameters  */
/*                  are also updated.                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_scan_req_params(scan_req_t *scan_req)
{
	#ifndef COMBO_SCAN //chenq mod for combo scan 2013-03-12
    UWORD8 i = 0;
	#else
	UWORD16 i = 0;
	UWORD16 * ie_len = NULL;
	#endif

	TROUT_FUNC_ENTER;
    /* Update the parameters required for scaning (number of channels,       */
    /* channel list, channel index and timeout values)                       */
    g_num_channels            = scan_req->num_channels;
    g_channel_index           = 0;
    g_bss_dscr_set_index      = 0;
    g_bss_dscr_first_set_sent = 0;
    g_force_keep_bss_info     = 0xFF;

    /* Save the currently connected AP configuration if incase of user scan  */
    if((g_scan_source != DEFAULT_SCAN) && (g_last_network_index != 0xFF))
    {
        if(g_last_network_index != 0)
        {
            memcpy(&g_bss_dscr_set[0], &g_bss_dscr_set[g_last_network_index],
                   sizeof(bss_dscr_t));
			
			ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_1[g_last_network_index]);
			memcpy(g_wpa_rsn_ie_buf_1[0],g_wpa_rsn_ie_buf_1[g_last_network_index],*ie_len + 2);
        }

        g_last_network_index = 0xFF;
        g_bss_dscr_set_index = 1;

        /* Reset BSS descriptior table */
        mem_set(&g_bss_dscr_set[1], 0, (sizeof(bss_dscr_t) * (MAX_SITES_FOR_SCAN - 1)));
    }
    else
    {
        /* Reset BSS descriptior table */
        mem_set(&g_bss_dscr_set, 0, (sizeof(bss_dscr_t) * MAX_SITES_FOR_SCAN));
    }

    for(i = 0; i < scan_req->num_channels; i++)
    {
        g_channel_list[i] = scan_req->channel_list[i];
    }

    update_scan_req_params_prot(scan_req);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_join_req_params                                   */
/*                                                                           */
/*  Description   : This function updates the globals and MIB parameters     */
/*                  according to join request structure.                     */
/*                                                                           */
/*  Inputs        : 1) Join request structure                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The global BSS Descriptor set is initialized according   */
/*                  to join request structure. Few MIB values also are       */
/*                  also updated.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_join_req_params(join_req_t *join_req)
{
    /* Update the MIB parameters using Join request parameters. The BSSID,   */
    /* Beacon period, DTIM period and Channel are updated. Note that the     */
    /* SSID and BSS Type are not updated as these should be same as the      */
    /* configured values.                                                    */
	TROUT_FUNC_ENTER;
    mset_bssid(join_req->bss_dscr.bssid);
    mset_BeaconPeriod(join_req->bss_dscr.beacon_period);
    mset_DTIMPeriod(join_req->bss_dscr.dtim_period);
    set_mac_chnl_num(join_req->bss_dscr.channel);
    update_join_req_params_prot(join_req);

    /* Set corresponding MAC H/w and RF registers with the updated settings. */
    set_machw_bssid(mget_bssid());
    set_machw_beacon_period(mget_BeaconPeriod());
    set_machw_dtim_period(mget_DTIMPeriod());
    select_channel_rf(mget_CurrentChannel(), get_bss_sec_chan_offset());
    update_phy_rate_table(join_req->bss_dscr.supp_rates, NULL);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_auth_req_params                                   */
/*                                                                           */
/*  Description   : This function updates globals and MIB values according   */
/*                  to authentication request structure.                     */
/*                                                                           */
/*  Inputs        : 1) Authentication request structure                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : MIB values are updated according to input authentication */
/*                  structure. Note that this is an overhead since the MIB   */
/*                  has already been set. Currently this function is empty   */
/*                  and provides a framework for future enhancements.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_auth_req_params(auth_req_t *auth_req)
{
    /* Currently do nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_asoc_req_params                                   */
/*                                                                           */
/*  Description   : This function updates globals and MIB values according   */
/*                  to association request structure.                        */
/*                                                                           */
/*  Inputs        : 1) Association request structure                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : MIB values are updated according to input association    */
/*                  structure. Note that this is an overhead since the MIB   */
/*                  has already been set. Currently this function is empty   */
/*                  and provides a framework for future enhancements.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_asoc_req_params(asoc_req_t *asoc_req)
{
    /* Currently do nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_start_req_params                                  */
/*                                                                           */
/*  Description   : This function updates globals and MIB values according   */
/*                  to input start request structure.                        */
/*                                                                           */
/*  Inputs        : 1) Start request structure                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The global start structure and MIB values are updated    */
/*                  according to input start request structure.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_start_req_params(start_req_t *start_req)
{
    /* Update Station Parameters based on the Start Request */
    update_start_req_params_prot(start_req);

    /* Set the channel with the configured channel. All other configurations */
    /* are not modified and need not be reinitialized.                       */
    select_channel_rf(mget_CurrentChannel(), get_bss_sec_chan_offset());
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : scan_channel                                             */
/*                                                                           */
/*  Description   : This function scans a channel at a time.                 */
/*                                                                           */
/*  Inputs        : 1) Scan request structure                                */
/*                                                                           */
/*  Globals       : g_channel_index                                          */
/*                                                                           */
/*  Processing    : None                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void scan_channel(UWORD8 channel)
{
    static UWORD32 s_count = 0;
    UWORD16 scan_timeout   = 0;
    BOOL_T  send_prb_req   = BFALSE;
    UWORD8  freq           = get_current_start_freq();
    UWORD8  tbl_idx        = 0;
    UWORD32 coex_reg = 0; 
    
	TROUT_FUNC_ENTER;
	// get coex state to select scan policy.
	MxdRfGetRfMode(&coex_reg);
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);
	if(reset_mac_trylock() == 0 ) return ;
	
    tbl_idx = get_tbl_idx_from_ch_idx(freq, channel);

    TROUT_DBG5("trout: scan channel %u(total %u)\n", channel+1, ++s_count);
    
    if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
    {
		TROUT_FUNC_EXIT;
		reset_mac_unlock();
        	return;
    }

	itm_scan_flag = 1;
    /*  Clear RX Power level for current channel   */
    g_channel_rssi[tbl_idx] = (WORD8)-100;

    /* Set appropriate VCO frequency for the channel that is scanned */
    /* Scanning is always done in 20MHz mode.                        */
    select_channel_rf(channel, 0);

    /* Active Scanning is done if enabled by the user and allowed by */
    /* Regulatory Restrictions for the current Channel.              */
    if((BFALSE == skip_chnl_for_scan(channel)) &&
       ((mget_scan_type() == ACTIVE_SCAN) || (g_scan_source & USER_SCAN)))
       send_prb_req = BTRUE;
	
    /* Get other protocol requirement for probe request */
    send_prb_req = get_prot_scan_type(send_prb_req);
    if(BTRUE == send_prb_req)
    {
        UWORD16 probe_req_frame_len = 0;
        UWORD8  *probe_req_frame = 0;

        /* Prepare a probe request and add it to the H/w queue with transmit */
        /* descriptor for transmission.                                      */

#ifdef WSC_IE_PROBE_REQ_EN
        probe_req_frame = (UWORD8*)pkt_mem_alloc(MEM_PRI_HPTX);
#else /* WSC_IE_PROBE_REQ_EN */
        probe_req_frame = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                             MANAGEMENT_FRAME_LEN);
#endif /* WSC_IE_PROBE_REQ_EN */

        if(probe_req_frame != NULL)
        {
            probe_req_frame_len = prepare_probe_req(probe_req_frame);

            /* Transmit the management frame */
            tx_mgmt_frame(probe_req_frame, probe_req_frame_len, HIGH_PRI_Q, 0);
            
            if((g_scan_source & USER_SCAN) &&
               (get_site_survey_status() == SITE_SURVEY_1CH ||
                get_site_survey_status() == SITE_SURVEY_ALL_CH))
            {
                /* In Site Survey Scan mode  */
                TROUT_DBG5("%s: survey scan mode!\n", __func__);
                scan_timeout = g_site_survey_scan_time;
			if(0 != (coex_reg & BIT1)){
		   		scan_timeout = 30; //ms
		   	}
            }
            else
            {
                /* In Active Scan mode, Time out can be less  */
                TROUT_DBG5("%s: active scan mode!\n", __func__);

				//chenq mod for combo scan 2013-03-12
				#ifndef COMBO_SCAN
                scan_timeout = g_active_scan_time;
				#else
				if(!g_ap_combo_scan_index)
				{
					scan_timeout = g_active_scan_time;
					if(0 != (coex_reg & BIT1)){
		   				scan_timeout = 80; //ms
		   			}
				}
				else
				{
					scan_timeout = 2;
				}
				#endif
				TROUT_DBG5("wait time %d ms\n",scan_timeout);
            }
        }
        else
        {
        	TROUT_DBG4("%s: no mem for prob req buffer!\n", __func__);
        }
    }
    else /* PASSIVE SCAN */
    {
        /* In Passive Scan mode  */
        TROUT_DBG5("%s: passive scan mode!\n", __func__);
        scan_timeout = g_passive_scan_time;
    }

    /* Overwrite the scan time with scan time dictated by any protocol */
    scan_timeout = get_scan_time_prot(scan_timeout, send_prb_req);

	TROUT_DBG5("%s: scan_timeout = %d ms\n", __FUNCTION__, scan_timeout);


    //chenq mod for combo scan 2013-03-13
    #ifdef COMBO_SCAN
	if(g_ap_combo_scan_timer == NULL)
	{
		//chenq mask g_ap_combo_scan_timer malloc 2013-04-11
		//g_ap_combo_scan_timer = (struct hrtimer *)kmalloc(sizeof(struct hrtimer), GFP_KERNEL);
		if(g_ap_combo_scan_timer == NULL)
		{
			TROUT_DBG5("alloc hrtime failed! will use  mgmt timer\n");
			start_mgmt_timeout_timer(scan_timeout);
		}
		else
		{
			g_ap_combo_scan_kt = ktime_set(0, scan_timeout * 1000 * 1000);//s,ns
			hrtimer_init(g_ap_combo_scan_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			g_ap_combo_scan_timer->function = mgmt_timeout_alarm_fn_combo_scan;
			hrtimer_start(g_ap_combo_scan_timer, g_ap_combo_scan_kt, HRTIMER_MODE_REL);
		}
	}
	else
	{
		g_ap_combo_scan_kt = ktime_set(0, scan_timeout * 1000 * 1000);//s,ns
		hrtimer_start(g_ap_combo_scan_timer, g_ap_combo_scan_kt, HRTIMER_MODE_REL);
	}
	#else
    /* Start the scan timeout timer for maximum channel time */
    start_mgmt_timeout_timer(scan_timeout);
	#endif
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: [%d] : scan_channel: ChNo=%d ChScanTime=%d\n\r",
                get_machw_tsf_timer_lo(), channel, scan_timeout);
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
	reset_mac_unlock();
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ibss_param                                           */
/*                                                                           */
/*  Description   : This function sets the IBSS Parameters element in the    */
/*                  beacon or probe response frame.                          */
/*                                                                           */
/*  Inputs        : 1) BSS Description Set                                   */
/*                  2) Beacon/ProbeResponse Frame                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_ibss_param(bss_dscr_t* bss_dscr, UWORD8* data)
{
    UWORD16 i   = 0;
    UWORD8  len = IE_HDR_LEN + data[1];
    for(i = 0; i < len; i++)
        bss_dscr->ibss_param[i] = data[i];
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_scan_response                                     */
/*                                                                           */
/*  Description   : This function updates the global BSS Descriptor Set      */
/*                  according to the information obtained by the beacon/     */
/*                  probe response frames.                                   */
/*                                                                           */
/*  Inputs        : 1) Beacon/Probe Response frame                           */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                  g_bss_dscr_set_index                                     */
/*                                                                           */
/*  Processing    : The beacon/probe response frames are processed according */
/*                  to their field values and BSS Descriptor set is updated. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

//chenq add a flag "snr" 
//void update_scan_response(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi)
void update_scan_response(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi,WORD8 snr)
{
    BSSTYPE_T bss_type           = INDEPENDENT;
    UWORD8    ssid[MAX_SSID_LEN] = {0};
    UWORD8    bssid[6]           = {0};
    UWORD8    sa[6]              = {0};
    UWORD16   cap_info           = 0;
    UWORD16   i                  = 0;
    UWORD8    index              = 0;
    UWORD8    len                = 0;
    UWORD8    *tim_elm           = 0;
    UWORD8    current_channel    = 0;
    UWORD8    bss_idx            = 0;
    UWORD8    info_keep_flg      = BFALSE;
    UWORD8    freq               = get_current_start_freq();
    UWORD8    tbl_idx            = 0;
    BOOL_T    status             = BFALSE;
    WORD32    add_bss_rssi       = 0;
    WORD32    bss_rssi           = 0;

	TROUT_FUNC_ENTER;
    /* Process the scan response frame for WPS */
    status = process_wps_scan_response((UWORD8 *)&g_mac, msa, rx_len, rssi);

    /* If processing has been completed successfully, no further processing  */
    /* is required to be done. Return. Otherwise do normal processing.       */
    if(BTRUE == status)
    {
    	TROUT_FUNC_EXIT;
        return;
    }
    
    /* Get the cap_info */
    cap_info  = get_cap_info(msa);

    index = MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN;
    len = msa[index+1];

    get_TA(msa, sa);
    get_ssid(msa, ssid);
    get_BSSID(msa, bssid);

    bss_type     = get_bss_type(cap_info);

    /* Get the current channel value */
    current_channel = get_rf_channel_idx();

    tbl_idx = get_tbl_idx_from_ch_idx(freq, current_channel);

    if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Update the current channel RSSI */
    if(g_channel_rssi[tbl_idx] < rssi )
        g_channel_rssi[tbl_idx] = rssi;

    /* Search for an existing entry */
    for(i = 0; i < g_bss_dscr_set_index; i++)
    {
        if(mac_addr_cmp(g_bss_dscr_set[i].bssid, bssid) == BTRUE)
        {
            /* Update the RSSI field of the descriptor with the RSSI observed */
            /* in the correct channel                                         */
            if(g_bss_dscr_set[i].channel == current_channel)
            {
                g_bss_dscr_set[i].rssi = rssi;
            }
            else if (rssi < g_bss_dscr_set[i].rssi)
            {
				TROUT_FUNC_EXIT;
                return;
            }
            break;
        }
    }

    /* If no matching entry is found, add a new entry to the BSS descriptor  */
    /* list                                                                  */
    if(i == g_bss_dscr_set_index)
    {
        UWORD8 scan_filter;
        if(g_force_keep_bss_info == 0xFF)
        {
            if((strlen(mget_DesiredSSID()) == strlen((WORD8 *)ssid)) &&
               (memcmp(mget_DesiredSSID(), ssid, strlen(mget_DesiredSSID())) == 0))
            {
                info_keep_flg = BTRUE;
            }
        }

        scan_filter = get_scan_filter();

        if(((scan_filter & SCAN_FILTER_BIT) == FILTER_AP_ONLY  && bss_type == INDEPENDENT ) ||
           ((scan_filter & SCAN_FILTER_BIT) == FILTER_STA_ONLY && bss_type == INFRASTRUCTURE ))
        {
            /*  The BSS information that is not matched to the BSS filter setting is discarded.  */
			TROUT_FUNC_EXIT;
            return;
        }

        if((( scan_filter & SCAN_CH_BIT ) == CH_FILTER_ON ) &&
           (get_current_channel(msa, rx_len) != (current_channel+1)))
        {
            /* The channel of this BSS not matched to current channel. */
			TROUT_DBG5("ssid: %s, channel unmatch! net channel: %d, current_channel=%d\n", 
				g_bss_dscr_set[bss_idx].ssid,get_current_channel(msa, rx_len),(current_channel+1));
			TROUT_FUNC_EXIT;
            return;
        }

        if(g_bss_dscr_set_index >= MAX_SITES_FOR_SCAN)
        {
            /*  If Scan priority setting is the order of detection.  */
            if((scan_filter & SCAN_PRI_BIT) == PRI_DETECT)
            {
                if(info_keep_flg == BTRUE)
                {
                    bss_idx = g_bss_dscr_set_index - 1;
                    g_force_keep_bss_info = bss_idx;
                }
                else
                {
                    /*  Already BSS could be found until the upper limit.  */
					TROUT_FUNC_EXIT;
                    return;
                }
            }
            /*  If Scan priority setting is the order of high RSSI.  */
            else if((scan_filter & SCAN_PRI_BIT) == PRI_HIGH_RSSI)
            {
                // modify by Ke.Li at 2013-01-07 for rssi must be used with signed
                WORD32 low_rssi = +127;
                add_bss_rssi = (SWORD8)rssi;
                bss_rssi = 0;

                for(i=0; i < g_bss_dscr_set_index; i++)
                {
                    bss_rssi = (SWORD8)g_bss_dscr_set[i].rssi;
                    if((bss_rssi < low_rssi) && (g_force_keep_bss_info != i))
                    {
                        low_rssi = bss_rssi;
                        bss_idx = i;
                    }
                }

                if(info_keep_flg == BTRUE)
                {
                    g_force_keep_bss_info = bss_idx;
                }
                else
                {
                    if(low_rssi > add_bss_rssi)
                    {
                        /*  If current RSSI is less than the RSSI which is listed up in BSS then it discards.  */
						TROUT_FUNC_EXIT;
                        return;
                    }
                }
                // end modify by Ke.Li at 2013-01-07 for rssi must be used with signed
            }
            /*  If Scan priority setting is the order of low RSSI.  */
            else if((scan_filter & SCAN_PRI_BIT) == PRI_LOW_RSSI)
            {
                // modify by Ke.Li at 2013-01-07 for rssi must be used with signed
                WORD32 high_rssi = (SWORD8)-127;
                add_bss_rssi = (SWORD8)rssi;
                bss_rssi = 0;

                for(i=0; i<g_bss_dscr_set_index; i++)
                {
                    bss_rssi = (SWORD8)g_bss_dscr_set[i].rssi;
                    if((bss_rssi > high_rssi) && (g_force_keep_bss_info != i))
                    {
                        high_rssi = bss_rssi;
                        bss_idx = i;
                    }
                }
                if(info_keep_flg == BTRUE)
                {
                    g_force_keep_bss_info = bss_idx;
                }
                else
                {
                    if(high_rssi < add_bss_rssi)
                    {
                        /*  If current RSSI is more than the RSSI which is listed up in BSS then it discards.  */
						TROUT_FUNC_EXIT;
                        return;
                    }
                }
                // end modify by Ke.Li at 2013-01-07 for rssi must be used with signed
            }
        }
        else
        {
            bss_idx = g_bss_dscr_set_index;
            if(info_keep_flg == BTRUE)
            {
                g_force_keep_bss_info = bss_idx;
            }
        }

        mem_set(&g_bss_dscr_set[bss_idx], 0, (sizeof(bss_dscr_t))); 

        /* Get the current channel */
        g_bss_dscr_set[bss_idx].channel = get_current_channel(msa, rx_len);

        /* Update the rssi information field */
        g_bss_dscr_set[bss_idx].rssi = rssi;

		//chenq add for snr information field
		g_bss_dscr_set[bss_idx].snr = snr;

        memcpy(g_bss_dscr_set[bss_idx].ssid,  ssid,  len);

        /* Set the last byte to '\0' so that strcmp may be used */
        g_bss_dscr_set[bss_idx].ssid[len] = '\0';

        memcpy(g_bss_dscr_set[bss_idx].bssid, bssid, 6);
        memcpy(g_bss_dscr_set[bss_idx].sa,  sa,  6);
        g_bss_dscr_set[bss_idx].bss_type = bss_type;
        g_bss_dscr_set[bss_idx].cap_info = cap_info;

        /* Update the beacon period */
        index = (MAC_HDR_LEN + TIME_STAMP_LEN);
        g_bss_dscr_set[bss_idx].beacon_period =
            get_beacon_period(msa + index);

        /* Skip Capability Info, SSID, Supported Rates, FH Parameter fields */
        index += BEACON_INTERVAL_LEN + CAP_INFO_LEN;

        /* Update the Supported and Extendend rates in the Descriptor set */
        update_bss_dscr_rates(msa, rx_len, bss_idx, index);

        /* Search for the CF parameter Field and update the Descriptor set */
        i = index;
        while(i < (rx_len - FCS_LEN))
        {
            if(msa[i] == IIBPARMS)
            {
                set_ibss_param(&g_bss_dscr_set[bss_idx], msa + i);
                break;
            }
            else
            {
                i += (IE_HDR_LEN + msa[i + 1]);
            }
        }

        /* Update the DTIM Period */
        tim_elm = get_tim_elm(msa, rx_len, index);
        if(tim_elm != 0)
            g_bss_dscr_set[bss_idx].dtim_period = tim_elm[3];

        /* Time stamp is not updated since this is taken care by hardware */
        update_scan_response_prot(msa, rx_len, index, bss_idx);

		//chenq add for check link ap info change 2013-06-08
		if( check_cur_link_ap_info_change(&(g_bss_dscr_set[bss_idx])) == BFALSE )
		{
			 TROUT_DBG4("cur link ap_info change!\n");
			 raise_system_error(LINK_LOSS);
		}
		//chenq add end
		
        /* Increment the index */
        if(g_bss_dscr_set_index < MAX_SITES_FOR_SCAN)
            g_bss_dscr_set_index++;

		TROUT_DBG5("max = %d,g_bss_dscr_set_index = %d\n",MAX_SITES_FOR_SCAN,g_bss_dscr_set_index);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_bss_dscr_set                                      */
/*                                                                           */
/*  Description   : This function updates the global BSS Descriptor Set      */
/*                  according to the information obtained by the beacon/     */
/*                  probe response frames at the given index entry.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Beacon/Probe Response frame            */
/*                  2) Length of the received frame                          */
/*                  3) RSSI of the received frame                            */
/*                  4) Index of the BSS descriptor to be updated             */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                                                                           */
/*  Processing    : The beacon/probe response frames are processed according */
/*                  to their field values and BSS Descriptor set is updated. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_bss_dscr_set(UWORD8 *msa, UWORD16 rx_len, SWORD8 rssi,
                         UWORD8 bss_idx)
{
    BSSTYPE_T bss_type           = INDEPENDENT;
    UWORD8    ssid[MAX_SSID_LEN] = {0};
    UWORD8    bssid[6]           = {0};
    UWORD8    sa[6]              = {0};
    UWORD16   cap_info           = 0;
    UWORD16   i                  = 0;
    UWORD16   index              = 0;
    UWORD8    len                = 0;
    UWORD8    *tim_elm           = 0;

    index = MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN;
    len = msa[index+1];

    /* Get the cap_info */
    cap_info = get_cap_info(msa);
    bss_type = get_bss_type(cap_info);

    get_SA(msa, sa);
    get_ssid(msa, ssid);
    get_BSSID(msa, bssid);

    /* Get the current channel */
    g_bss_dscr_set[bss_idx].channel = get_current_channel (msa,rx_len);

    /* Update the rssi information field */
    g_bss_dscr_set[bss_idx].rssi = rssi;

    memcpy(g_bss_dscr_set[bss_idx].ssid,  ssid,  len);

    /* Set the last byte to '\0' so that strcmp may be used */
    g_bss_dscr_set[bss_idx].ssid[len] = '\0';

    memcpy(g_bss_dscr_set[bss_idx].bssid, bssid, 6);
    memcpy(g_bss_dscr_set[bss_idx].sa,  sa,  6);
    g_bss_dscr_set[bss_idx].bss_type = bss_type;
    g_bss_dscr_set[bss_idx].cap_info = cap_info;

    /* Update the beacon period */
    index = (MAC_HDR_LEN + TIME_STAMP_LEN);
    g_bss_dscr_set[bss_idx].beacon_period =
        get_beacon_period(msa + index);

    /* Skip Capability Info, SSID, FH Parameter fields */
    index += BEACON_INTERVAL_LEN + CAP_INFO_LEN;

    /* Update the Supported and Extendend rates in the Descriptor set */
    update_bss_dscr_rates(msa, rx_len, bss_idx, index);

    i = index;

    /* Search for the IBSS parameter Field and update the Descriptor set */
    while(i < (rx_len - FCS_LEN))
    {
        if(msa[i] == IIBPARMS)
        {
            set_ibss_param(&g_bss_dscr_set[bss_idx], msa + i);
            break;
        }
        else
        {
            i += (2 + msa[i + 1]);
        }
    }

    /* Update the DTIM Period */
    tim_elm = get_tim_elm(msa, rx_len, index);
    if(tim_elm != 0)
        g_bss_dscr_set[bss_idx].dtim_period = tim_elm[3];

    /* Time stamp is not updated since this is taken care by hardware */
    update_scan_response_prot(msa, rx_len, index, bss_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_bss_dscr_rates                                    */
/*                                                                           */
/*  Description   : This function updates the supported and extended rate    */
/*                  fields global BSS Descriptor Set according to the        */
/*                  information obtained by the beacon/probe response frames */
/*                  at the given index entry.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Beacon/Probe Response frame            */
/*                  2) Length of the received frame                          */
/*                  3) Index of the BSS descriptor to be updated             */
/*                  4) Index in Beacon/Probe Response frame                  */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                                                                           */
/*  Processing    : The beacon/probe response frames are processed searched  */
/*                  for supported and extended supported rates IEs and       */
/*                  corresponding fields in the BSS Descriptor set is updated*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_bss_dscr_rates(UWORD8 *msa,    UWORD16 len,
                           UWORD8 bss_idx, UWORD16 offset)
{
    UWORD8  i       = 0;
    UWORD8  num_br  = 0;
    UWORD8  num_nbr = 0;
    UWORD16 idx     = 0;
    UWORD8 *supp_rates = g_bss_dscr_set[bss_idx].supp_rates;

	TROUT_FUNC_ENTER;
    /* Index 0 of the array is for storing the number of rates */
    supp_rates[0] = 0;

    /* Initialize the data index to TAG parameter length */
    idx = offset;

    /* Extract Basic rates if applicable */
    while(idx < (len - FCS_LEN))
    {
        if(msa[idx] == ISUPRATES)
        {
            num_br = msa[idx + 1];

            if(num_br > MAX_RATES_SUPPORTED)
                num_br = MAX_RATES_SUPPORTED;

            supp_rates[0] = num_br;

            idx += 2;

            /* Set the rate supported by the STA in the table */
            for(i = 0; i < num_br; i++)
            {
                supp_rates[i + 1] = msa[idx + i];
            }

            break;
        }
        else
        {
            idx += (msa[idx + 1] + 2);
        }
    }

    idx = offset;

    /* Extract Extended rates if present */
    while(idx < (len - FCS_LEN))
    {
        if(msa[idx] == IEXSUPRATES)
        {
            num_nbr = msa[idx + 1];
            idx += 2;

            if(num_nbr > (MAX_RATES_SUPPORTED - num_br))
            {
               num_nbr = (MAX_RATES_SUPPORTED - num_br);
            }

            supp_rates[0] += num_nbr;

            /* Set the rate supported by the STA in the table */
            for(i = 0; i < num_nbr; i++)
            {
                if ( (i + num_br + 1) > MAX_RATES_SUPPORTED)
                    break;
                supp_rates[i + num_br + 1] = msa[idx + i];
            }

            break;
        }
        else
        {
            idx += (msa[idx + 1] + 2);
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : generate_bssid                                           */
/*                                                                           */
/*  Description   : This function generates a random BSSID.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the BSSID                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The BSSID is set using random numbers read from MAC H/w  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void generate_bssid(UWORD8 *addr)
{
    UWORD8 i = 0;

    for(i = 0; i < 6; i++)
    {
        addr[i] = get_random_number();
    }

    /* Reset the universal bit */
    addr[0] &= ~BIT0;

    /* It is the locally adminstered address */
    addr[0] |= BIT1;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_sta_entry                                           */
/*                                                                           */
/*  Description   : This function initializes the structure related to STA   */
/*                  auto rate.                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function is called during initialization of the     */
/*                  MAC station globals.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_rate_low2high(UWORD8* rate_ie)
{
    UWORD8 flag = 1;
    UWORD32 i   = 0;
    UWORD8 tmp  = 0;
    UWORD8 rate_data_len = rate_ie[1];

    rate_ie +=IE_HDR_LEN;
    while(flag)
    {
        flag = 0;
        for( i = 0; i < rate_data_len-1; i++)
        {
            if((rate_ie[i] & 0x7F) > (rate_ie[i + 1] & 0x7F))
            {
                tmp = rate_ie[i+1];
                rate_ie[i+1] = rate_ie[i];
                rate_ie[i]   = tmp;
                flag = 1;
            }
        }
    }

    for(i=0;i<rate_data_len;i++)
        TROUT_DBG4("%02x(%02x) ",rate_ie[i]&0x7F,rate_ie[i]);

    TROUT_DBG4("\n");
}

void init_sta_entry(sta_entry_t *se, UWORD8 *msa, UWORD16 len, UWORD16 offset)
{
    UWORD8  k       = 0;
    UWORD8  j       = 0;
    UWORD8  i       = 0;
    UWORD8  num_br  = 0;
    UWORD8  num_nbr = 0;
    UWORD16 idx     = 0;
    UWORD8 *supp_rates = NULL;
    UWORD8 *extn_rates = NULL;

	TROUT_FUNC_ENTER;
    /* If station entry does not exist, return */
    if(se == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
	}
    /* If initialization is already done, return */
    if(se->init_done == FULLY_DONE)
    {
		TROUT_FUNC_EXIT;
        return;
    }
    else if((se->init_done == PARTLY_DONE) &&
            (msa == NULL))
    {
		TROUT_FUNC_EXIT;
        return;
    }

    if(msa == NULL)
    {
        /* Extract the number of basic and non-basic rates and set the       */
        /* number of supported rates field based on this. Maximum value      */
        /* allowed is 8. All other rates (if any) are set in the extended    */
        /* rates field.                                                      */
        num_nbr = get_num_non_basic_rates();
        num_br  = get_num_basic_rates();

        k = num_nbr + num_br;

        if(k > MAX_RATES_SUPPORTED)
            k = MAX_RATES_SUPPORTED;

        /* Set the supported rates */
        for(j = 0; j < k; j++)
        {
            if(j < num_br)
            {
                se->op_rates.rates[j] = get_mac_basic_rate(j);
            }
            else
            {
                se->op_rates.rates[j] =
                            get_mac_non_basic_rate((UWORD8)(j - num_br));
            }
        }
    }
    else
    {
        /* Initialize the data index to TAG parameter length */
        idx = offset;
        k   = 0;

        /* Extract Basic rates if applicable */
        while(idx < (len - FCS_LEN))
        {
            if(msa[idx] == ISUPRATES)
            {
                set_rate_low2high(&msa[idx]);

                num_br = msa[idx + 1];
                supp_rates = &msa[idx + 1];

                idx += IE_HDR_LEN;

                /* Set the rate supported by the STA in the table */
                for(i = 0; i < num_br; i++)
                    se->op_rates.rates[k + i] = msa[idx + i] & 0x7F;

                break;
            }
            else
            {
                idx += (msa[idx + 1] + IE_HDR_LEN);
            }
        }

        idx = offset;
        k   += num_br;

        /* Extract ERP rates if applicable */
        while(idx < (len - FCS_LEN))
        {
            if(msa[idx] == IEXSUPRATES)
            {
                set_rate_low2high(&msa[idx]);

                num_nbr = msa[idx + 1];
                extn_rates = &msa[idx + 1];

                idx += IE_HDR_LEN;

                if(num_nbr > (MAX_RATES_SUPPORTED - k))
                {
                   num_nbr =  (MAX_RATES_SUPPORTED - k);
                }

                /* Set the rate supported by the STA in the table */
                for(i = 0; i < num_nbr; i++)
                    se->op_rates.rates[k + i] = msa[idx + i] & 0x7F;

                break;
            }
            else
            {
                idx += (msa[idx + 1] + IE_HDR_LEN);
            }
        }

        k += num_nbr;

        //chenq add for workaround single dateRate test 2013-10-21
        if( (num_nbr == 0) && (num_br == 1) && ( (supp_rates[1] & 0x7F) == 0) )
        {
            UWORD8 tmp_date_rate[12] = {0};
            set_sup_rates_element( tmp_date_rate,0);
            *(supp_rates + 1) = tmp_date_rate[2];
            se->op_rates.rates[0] = tmp_date_rate[2] & 0x7F;
        }
        //chenq add end

        /* Update the PHY Rate Table according to AP in BSS-STA Mode */
        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
           update_phy_rate_table(supp_rates,extn_rates);
        }

    }

    se->tx_rate_index = k - 1;

    /* Update the number of rates supported field */
    se->op_rates.num_rates = k;

    /* Sort the operational rate set of the station in ascending order */
    for(i = 0; i < k; i++)
    {
        UWORD8 min = (se->op_rates.rates[i] & 0x7F);
        UWORD8 mid = i;
        UWORD8 tmp = 0;

        for(j = i + 1; j < k; j++)
        {
            if(min > (se->op_rates.rates[j] & 0x7F))
            {
                min = (se->op_rates.rates[j] & 0x7F);
                mid = j;
            }
        }

        /* Swap the rates if they are not in order */
        tmp = se->op_rates.rates[i];

        se->op_rates.rates[i] = se->op_rates.rates[mid];

        se->op_rates.rates[mid] = tmp;
    }

     /* Incase of Ittiam Station in 802.11b mode and The other station or AP */
     /* is in 802.11g mode, Use Ittiam Station Supported Rates for AutoRate  */
     num_nbr = get_num_non_basic_rates();
     num_br  = get_num_basic_rates();
     if(se->op_rates.num_rates > (num_nbr + num_br))
     {
        se->op_rates.num_rates = num_nbr + num_br;
        se->tx_rate_index      = se->op_rates.num_rates - 1;

        /* Set the self rates */
        for(j = 0; j < se->op_rates.num_rates; j++)
        {
            if(j < num_br)
            {
                se->op_rates.rates[j] = get_mac_basic_rate(j);
            }
            else
            {
                se->op_rates.rates[j] =
                            get_mac_non_basic_rate((UWORD8)(j - num_br));
            }
        }

     }

    /* Sort the operational rate set of the 11g station(Operating in         */
    /* compatibility mode) in such a way that all 11b rates will be clubbed  */
    /* and 11a rates will come after the 11b rates                           */
    if(se->op_rates.num_rates == 12) /* 11g_compatibility mode */
    {
        if((se->op_rates.rates[5] & 0x7F) == 0x16) /* 11Mbps */
        {
            UWORD8 temp;
            temp                  = se->op_rates.rates[5];
            se->op_rates.rates[5] = se->op_rates.rates[4];
            se->op_rates.rates[4] = se->op_rates.rates[3];
            se->op_rates.rates[3] = temp;
        }
    }

#ifdef AUTORATE_FEATURE
    /* Update the minimum and maximum rate index in the global auto rate     */
    /* table  for this STA                                                   */
    update_min_rate_idx_sta(se);
    update_max_rate_idx_sta(se);

    /* Update the current transmit rate index to the minimum supported rate  */
    /* index in case of auto rate                                            */
    init_tx_rate_idx(se);
    reinit_tx_rate_idx(se);
#endif /* AUTORATE_FEATURE */

    /* Initialize the auto rate statistics */
    ar_stats_init(&(se->ar_stats));

    /* Update the retry rate set table for this station based on the current */
    /* transmit rate of this station                                         */
    update_entry_retry_rate_set((void *)se, get_phy_rate(get_tx_rate_sta(se)));

    /* Set the flag saying initialization done */
    if(msa == NULL)
        se->init_done = PARTLY_DONE;
    else
        se->init_done = FULLY_DONE;

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_sta_entry                                         */
/*                                                                           */
/*  Description   : This function clears the remote sta entry elements and   */
/*                  deletes the entry                                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the sta entry                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function clears the remote sta entry elements and   */
/*                  deletes the entry                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_sta_entry(void *element)
{
    sta_entry_t *se = (sta_entry_t *)element;

    reset_sta_entry(se);

    /* Delete the security table/LUT and halt all the */
    /* security related FSMs                          */
    delete_sec_entry_sta(se);

    /* Free the association table entry */
    mem_free(g_local_mem_handle, se);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_sta_entry                                       */
/*                                                                           */
/*  Description      : This function resets the fields of STA entry.         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the sta entry of the STA                */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function resets the fields of STA entry.         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void reset_sta_entry(sta_entry_t *se)
{
    if(se != NULL)
    {
        /* Reset the security LUTs and entries */
        reset_prot_entry_sta(se);

        /* Remove the entry of the STA from the STA Index Table   */
        /* Note that the sta index is reset and cannot be re-used */
        del_sta_index((void *)se);

        /* Reset the initialization flag */
        se->init_done = NOT_DONE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_bss_capability_mac                                 */
/*                                                                           */
/*  Description   : This function compares the capabiltiy of the STA and the */
/*                  BSS                                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the BSS Scan Descriptor set                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if all capabilities specific to MAC */
/*                  (like BSS Type, Privacy and Multi-domain) match those    */
/*                  of the STA.                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if all capabilities match                         */
/*                  BFALSE, otherwise.                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_bss_capability_mac(bss_dscr_t *bss_dscr)
{
    BOOL_T ret_val = BTRUE;

    ret_val &= check_bss_capability_info(bss_dscr->cap_info);
    ret_val &= check_bss_sec_info_sta(bss_dscr);
	// 20120830 caisf add, merged ittiam mac v1.3 code
    ret_val &= check_bss_reg_domain_info(bss_dscr);

    return ret_val;
}

//chenq add for check link ap info change 2013-06-08
BOOL_T check_bss_capability_mac_change(bss_dscr_t *bss_dscr)
{
    BOOL_T ret_val = BTRUE;

    ret_val &= check_bss_capability_info_change(bss_dscr->cap_info,bss_dscr);
    ret_val &= check_bss_sec_info_sta_change(bss_dscr);
	// 20120830 caisf add, merged ittiam mac v1.3 code
    ret_val &= check_bss_reg_domain_info(bss_dscr);

    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : filter_host_rx_frame_sta                              */
/*                                                                           */
/*  Description      : This function filters the incoming lan packet and     */
/*                     sends the results of the operation.                   */
/*                                                                           */
/*  Inputs           : 1) mac_addres_t. The destination address              */
/*                     2) UWORD8*. Pointer of the buffer to be released      */
/*                     3) Cipher Type*. Pointer to the cpiher type for STA   */
/*                     4) BOOL_T  Flag indicating if port Status needs to be */
/*                                checked                                    */
/*                     5) UWORD*. Pointer to the sta index                   */
/*                     6) UWORD*. Pointer to the key type                    */
/*                     7) UWORD** Pointer to any info required for Tx        */
/*                     8) UWORD** Pointer to the STA Entry                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function returns 'BTRUE' only if the LAN packet  */
/*                     was discarded. Also the function updates the Cipher   */
/*                     type for the STA and the key index for the STA        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T. The result of lan frame filtering.            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T filter_host_rx_frame_sta(wlan_tx_req_t * tx_req, UWORD8* da,
                                BOOL_T ignore_port, CIPHER_T* ct,
                                UWORD8* sta_index, UWORD8* key_type,
                                UWORD8** info, sta_entry_t **se)
{
    UWORD8 *start_ptr = tx_req->buffer_addr;

	TROUT_FUNC_ENTER;
    /* Reset the parameters to default values */
    *ct        = NO_ENCRYP;
    *sta_index = 0;
    *key_type  = 0;
    *info      = NULL;

    /* For unicast packets, check if the destination STA is associated. */
    if(is_group(da) == BFALSE)
    {
        /* Get association entry for association state */
        *se = (sta_entry_t *)find_entry(da);

        /* If the destination STA is not listed free the data packet and */
        /* return.                                                       */
        if(*se == NULL)
        {
            pkt_mem_free(start_ptr);
			TROUT_FUNC_EXIT;
            return BTRUE;
        }

        /* Set key type as unicast key type */
        *key_type  = UCAST_KEY_TYPE;
        *sta_index = (*se)->sta_index;
    }
    else
    {
        /* Set key type as unicast key type */
        *key_type  = BCAST_KEY_TYPE;
    }

    /* Filter the frame on the basis of the security policies */
    {
        BOOL_T ret_val = BFALSE;

        ret_val = filter_host_rx_sec_sta(tx_req, ct, info, *key_type, *se,
                                         ignore_port);

        if(ret_val == BTRUE)
        {
            pkt_mem_free(start_ptr);
            TROUT_FUNC_EXIT;
            return ret_val;
        }
    }

    /* Extract service class information */
    update_serv_class_prot_sta(*se, tx_req->priority,
                               &(tx_req->service_class));
	TROUT_FUNC_EXIT;
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : filter_wlan_rx_frame_sta                                 */
/*                                                                           */
/*  Description   : This function filters the incoming wlan rx packet and    */
/*                  sends the results of the operation.                      */
/*                                                                           */
/*  Inputs        : 1) MAC header of incoming packet.                        */
/*                  2) Pointer to rx descriptor                              */
/*                  3) Pointer to wlan_rx structure                          */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function returns 'BTRUE' only if the WLAN packet    */
/*                  was discarded.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T. The result of lan frame filtering.               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T filter_wlan_rx_frame_sta(wlan_rx_t *wlan_rx)
{
    UWORD8       sta_type  = mget_DesiredBSSType();
    sta_entry_t *se = (sta_entry_t *)wlan_rx->sa_entry;
    UWORD8 *msa     = wlan_rx->msa;
    UWORD8 *ta;

	TROUT_FUNC_ENTER;
    if(wlan_rx->type == DATA_BASICTYPE)
    {
        if((se == 0) && (sta_type == INFRASTRUCTURE) &&
           (mac_addr_cmp(wlan_rx->da, mget_StationID()) == BTRUE))
        {
            /* Deauthenticate oneself for a invalid frame */
            UWORD16 error = 0;

            /* Get the error code based on the frame type */
            switch(get_frame_class(msa))
            {
                case CLASS2_FRAME_TYPE:
                default:
                {
                    error = (UWORD16)CLASS2_ERR;
                }
                break;
                case CLASS3_FRAME_TYPE:
                {
                    error = (UWORD16)CLASS3_ERR;
                }
            }
            /* Send the De-authentication Frame to the station */
			ta = wlan_rx->ta;
            TROUT_DBG4("%s: send deauth to sta: %02x-%02x-%02x-%02x-%02x-%02x\n", 
            					__func__, ta[0], ta[1], ta[2], ta[3], ta[4], ta[5]);
            send_deauth_frame(wlan_rx->ta, error);

            /* Return TRUE for filter frame, this should free the rx frame */
			TROUT_FUNC_EXIT;
            return BTRUE;
        }

        /* Filter the frame on the basis of the security policies */
        if(se != 0)
        {
            BOOL_T ret_val = BFALSE;

            ret_val = filter_wlan_rx_sec_sta(wlan_rx);
            if(ret_val == BTRUE)
            {
				TROUT_FUNC_EXIT;
                return ret_val;
            }
        }

        /* Filter the frame on the basis of the service class/priorities */
        if(se != 0)
        {
            BOOL_T ret_val = BFALSE;

            ret_val = filter_wlan_serv_cls_sta(wlan_rx);
            if(ret_val == BTRUE)
            {
				TROUT_FUNC_EXIT;
                return ret_val;
            }
        }
    }

	TROUT_FUNC_EXIT;
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_rx_power_level_sta                                   */
/*                                                                           */
/*  Description   : This function updates the configurator message with the  */
/*                  observed RSSI values in all channels at the time of      */
/*                  scanning                                                 */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_cfg_val                                             */
/*                  2) g_channel_rssi                                        */
/*                                                                           */
/*  Processing    : This function returns the channel RSSI string in a       */
/*                  format understood by the configuartor                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Configuartion message                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

WORD8* get_rx_power_level_sta(void)
{
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);

/* MD-TBD : configurator will not understand this now as index does not make sense */
    g_cfg_val[0] = num_sup_chan;
    memcpy((char *)(g_cfg_val + 1), g_channel_rssi, g_cfg_val[0]);

    return (WORD8*) g_cfg_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_site_survey_results_sta                              */
/*                                                                           */
/*  Description   : This function returns the site survey results            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_bss_dscr_set_index                                  */
/*                  2) g_bss_dscr_first_set_sent                             */
/*                                                                           */
/*  Processing    : This function updates a string with the Site survey      */
/*                  results to be sent to the host                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : The result of the Site Survey                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *get_site_survey_results_sta(void)
{
    UWORD8 idx      = 0;
    UWORD8 start_id = 0;
    UWORD8 i        = 0;
    UWORD8 *ptr     = 0;
    UWORD8 tmp      = 0;

	UWORD16 * ie_len = NULL;

    /* g_bss_dscr_first_set_sent: Indicates the fragement number to be sent */
    /* g_bss_dscr_set_index     : Number of elements of BSS DSCR            */
	#if 0
    if((g_bss_dscr_set_index > 5) && (g_bss_dscr_first_set_sent == 1))
    {
        idx = g_bss_dscr_set_index - 5;
        start_id = 5;
    }
    else if((g_bss_dscr_set_index > 5) && (g_bss_dscr_first_set_sent == 0))
    {
        idx      = 5;
        start_id = 0;
    }
    else if((g_bss_dscr_set_index <= 5) && (g_bss_dscr_first_set_sent == 1))
    {
        idx      = 0;
        start_id = 0;
    }
    else /* (g_bss_dscr_set_index <= 5) && (g_bss_dscr_first_set_sent == 0) */
    {
        idx      = g_bss_dscr_set_index;
        start_id = 0;
    }
	#else

	//int tmp_a = g_bss_dscr_set_index/5;
	//int tmp_b = g_bss_dscr_set_index%5;

	if(g_bss_dscr_set_index == 0 )
	{
		idx      = 0;
        start_id = 0;
	}
	else if( (g_bss_dscr_first_set_sent >= 0) && (g_bss_dscr_first_set_sent <= 3) )
	{
		if( ( g_bss_dscr_set_index >= (1 + g_bss_dscr_first_set_sent*5) ) 
		  && ( g_bss_dscr_set_index <= (5 + g_bss_dscr_first_set_sent*5) ) 
		  )
		{
			idx = g_bss_dscr_set_index - (5 * g_bss_dscr_first_set_sent);
			start_id = 5 * g_bss_dscr_first_set_sent;
		}
		else 
		if( (g_bss_dscr_set_index > (5 + g_bss_dscr_first_set_sent*5) )
		   &&(g_bss_dscr_set_index <= 4*5)
		  )
		{
			idx = 5;
			start_id = 5 * g_bss_dscr_first_set_sent;
		}
		else
		{
			idx = 0;
			start_id = 0;
		}
	}			
	else
	{
		idx = 0;
		start_id = 0;
	}
	#endif
	//chenq mod
	#if 0
    g_cfg_val[0] = ((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH) + 2;
    g_cfg_val[1] = ((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH);
    g_cfg_val[2] = g_bss_dscr_first_set_sent;

    ptr          = &g_cfg_val[3];
	#else
	g_cfg_val_for_gscan[0] = (((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH) + 3) & 0xFF;
	g_cfg_val_for_gscan[1] = ((((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH) + 3) >> 8 ) & 0xFF;
	g_cfg_val_for_gscan[2] = ((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH) & 0xFF;
	g_cfg_val_for_gscan[3] = (((idx)* SITE_SURVEY_RESULTS_ELEMENT_LENGTH) >> 8) &0xFF;
    g_cfg_val_for_gscan[4] = g_bss_dscr_first_set_sent;

    ptr          = &g_cfg_val_for_gscan[5];
	#endif

	//printk("chenq_itm get_site_survey_results_sta:len=%d,num=%d\n",
	//		g_cfg_val_for_gscan[2] | (g_cfg_val_for_gscan[3] << 8),g_cfg_val_for_gscan[3]);

    mem_set(ptr, 0x00, SITE_SURVEY_RESULTS_ELEMENT_LENGTH*5 );


    for(i = 0; i < idx; i++)
    {
        /* Setting SSID */
        memcpy(ptr, g_bss_dscr_set[start_id + i].ssid, MAX_SSID_LEN);
        ptr += MAX_SSID_LEN;

        /* Setting BSS Type Information */
#ifdef SUPP_11I
        /* Ittiam Convention */
        *ptr = g_bss_dscr_set[start_id + i].bss_type;
#else /* SUPP_11I */
        /* Customer Convention(SDIO Host driver follows this) */
        *ptr = g_bss_dscr_set[start_id + i].bss_type - 1;
#endif /* SUPP_11I */
        ptr++;

        /* Setting Channel Information */
        *ptr = g_bss_dscr_set[start_id + i].channel;
        ptr++;

        /* Setting Capability Information */
	/*
	 * leon liu modified cap_info retrieving
         * tmp = ((g_bss_dscr_set[start_id + i].cap_info) & PRIVACY)?(BIT0):(0);
	 */
        tmp = g_bss_dscr_set[start_id + i].cap_info;
       

#ifdef  MAC_802_11I
        /* Setting 11i mode information */
        tmp |= g_bss_dscr_set[start_id + i].dot11i_info;
#endif  /* MAC_802_11I */

        *ptr = tmp;
        ptr++;

//chenq add for wpa supplicant need
#ifdef MAC_802_11I
		
		tmp =  g_bss_dscr_set[start_id + i].rsn_cap[0];
		*ptr = tmp;
        ptr++;
		
		tmp =  g_bss_dscr_set[start_id + i].rsn_cap[1];
		*ptr = tmp;
        ptr++;
#endif

        /* Setting BSSID Information */
        memcpy(ptr, g_bss_dscr_set[start_id + i].bssid, 6);
        ptr += 6;

        /* Setting RSSI Information */
        *ptr = g_bss_dscr_set[start_id + i].rssi;
        ptr++;

		//chenq add snr Information
		*ptr = g_bss_dscr_set[start_id + i].snr;
        ptr++;

        //chenq add (MAX_RATES_SUPPORTED) byte for supp rates
        /* Setting supp rates*/
        memset(ptr,0x00,MAX_RATES_SUPPORTED);
        memcpy(ptr,&(g_bss_dscr_set[start_id + i].supp_rates[1]),
                MAX_RATES_SUPPORTED);
        ptr += MAX_RATES_SUPPORTED;

        /* Setting mac_802_11n Information */ 
        *ptr = g_bss_dscr_set[start_id + i].ht_capable;
        ptr++;
        
        *ptr =  g_bss_dscr_set[start_id + i].supp_chwidth;
        ptr++;
            
        *ptr = g_bss_dscr_set[start_id + i].sta_chwidth;
        ptr++;
        
        *ptr = g_bss_dscr_set[start_id + i].sec_ch_offset;
        ptr++;
        
        *ptr = g_bss_dscr_set[start_id + i].coex_mgmt_supp;
        ptr++;

        /* Setting Reserved byte */
        *ptr = (UWORD8)0;
        ptr++;


		//chenq add 2013-02-26
		ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_1[start_id + i]);

		memcpy(g_wpa_rsn_ie_buf_2[start_id + i],g_wpa_rsn_ie_buf_1[start_id + i],
			   *ie_len + 2);
			   //g_wpa_rsn_ie_buf_1[start_id + i][0]+1);
    }

    //g_bss_dscr_first_set_sent = (g_bss_dscr_first_set_sent + 1) % 2;
    g_bss_dscr_first_set_sent = (g_bss_dscr_first_set_sent + 1) % 4;

    return g_cfg_val_for_gscan;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_join_req_sta                                         */
/*                                                                           */
/*  Description   : This function initiates a join req                       */
/*                                                                           */
/*  Inputs        : Index of the station to be joined                        */
/*                                                                           */
/*  Globals       : 1) g_bss_dscr                                            */
/*                  2) g_bss_dscr_set_index                                  */
/*                                                                           */
/*  Processing    : This function initiates a join request                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_join_req_sta(UWORD8 val)
{
    /* Update the scan Response */
    if((WORD8)g_bss_dscr_set_index > (WORD8)val)
    {
        /* Set SSID, BSS type and privacy status MIB parameters              */
        mset_DesiredBSSType(g_bss_dscr_set[val].bss_type);
        mset_DesiredSSID(g_bss_dscr_set[val].ssid);
        memcpy(g_prefered_bssid, g_bss_dscr_set[val].bssid, 6);
        set_mac_chnl_num(g_bss_dscr_set[val].channel);

        if((g_bss_dscr_set[val].cap_info & PRIVACY) != PRIVACY)
        {
            set_802_11I_mode(0);
        }
#ifdef MAC_802_11I
        else  if(g_bss_dscr_set[val].rsn_found == BTRUE)
        {
            set_802_11I_mode(g_bss_dscr_set[val].dot11i_info);
        }
#endif /* MAC_802_11I */
    }
    g_scan_source  = DEFAULT_SCAN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : disconnect_sta                                           */
/*                                                                           */
/*  Description   : This function is used to disconnect.                     */
/*                                                                           */
/*  Inputs        : Index of the station to be disconnected                  */
/*                                                                           */
/*  Globals       : 1) g_mac.                                                */
/*                                                                           */
/*  Processing    : This function is used to disconnect.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void disconnect_sta(UWORD8 val)
{
    UWORD8  null_bssid[6] = {0};

	TROUT_FUNC_ENTER;
    /* Perform appropriate pre-reset tasks releated to host */
    pre_reset_tasks_host(&g_mac);

    /* Perform appropriate pre-reset tasks releated to wireless lan */
    pre_reset_tasks_wlan(&g_mac);

    /* Stop MAC HW and PHY HW first before going for reset */
    disable_machw_phy_and_pa();

    /* Disable all CE features of MAC H/w before disconnecting */
    disable_machw_ce_features();

    /* Deauthenticate for the BSS STA */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        /* Remove the entry for the sta                             */
        /* This does the RSNA Cleanup along with that of Asoc table */
        delete_entry(mget_bssid());
    }

    /* Reset BSS ID of Station */
    mset_bssid(null_bssid);

    /* Change state of the MAC structure to DISABLED */
    set_mac_state(DISABLED);

    /* Stop the Autorate timer as it is going to be started again */
    /* when STA associates with AP after the cntr msr period      */
    stop_ar_timer();

    /* Stop the activity (power save) timer */
    stop_activity_timer();
    
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_free_sta_entry                                     */
/*                                                                           */
/*  Description   : This function frees the Station Entry in the association */
/*                  table based on its activity history.                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_sta_table.                                          */
/*                                                                           */
/*  Processing    : This function traverses through the association table    */
/*                  and compares the connection count of the entry with the  */
/*                  current time reference. The connection count provides    */
/*                  the activity history with the station. If the last       */
/*                  activity was found to be sufficiently back in time, then */
/*                  the entry is deleted from the association table.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void check_free_sta_entry(void)
{
    UWORD8        i        = 0;
    sta_entry_t   *te      = 0;
    table_elmnt_t *tbl_elm = 0;

    /* Traverse entire station/association table and process all non-zero    */
    /* entries                                                               */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = (sta_entry_t *)tbl_elm->element;

            if(te == 0)
                break;

            if((mac_addr_cmp(tbl_elm->key, mget_StationID()) == BFALSE) &&
               (get_uptime_cnt() - te->connection_cnt >
                                             IBSS_STA_LINK_LOSS_THRESHOLD))
            {
                delete_entry(tbl_elm->key);
            }

            tbl_elm = tbl_elm->next_hash_elmnt;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_sta_connected                                         */
/*                                                                           */
/*  Description   : This function checks if a STA is connected (includes any */
/*                  key handshake etc)                                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the MAC state is enabled and any */
/*                  required security handshake has been completed.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T; BTRUE if the STA is connected, BFALSE otherwise  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_sta_connected(void)
{
    BOOL_T status = BFALSE;

    if(get_mac_state() == ENABLED)
    {
        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            sta_entry_t *se = find_entry(mget_bssid());

            if(is_sec_hs_complete(se) == BTRUE)
                status = BTRUE;
        }
        else /* INDEPENDENT */
        {
            status = BTRUE;
        }
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_rsp_sta                                      */
/*                                                                           */
/*  Description   : This function handles MLME response for STA mode         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) MLME Response type                                    */
/*                  3) MLME Response message                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes the MLME response based on the   */
/*                  protocols enabled and if not handled sends the same to   */
/*                  the host.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_rsp_sta(mac_struct_t *mac, UWORD8 rsp_type, UWORD8 *rsp_msg)
{
    if(handle_mlme_rsp_prot(rsp_type, rsp_msg) == BFALSE)
    {
        /* Send the response to host if the same has not been handled */
        send_mlme_rsp_to_host_sta((mac_struct_t *)mac, rsp_type, rsp_msg);
    }
}

//chenq add for check link ap info change 2013-06-08
BOOL_T check_bss_capability_info_change(UWORD16 cap_info,bss_dscr_t * bss_dscr)
{
    BOOL_T ret_val = BTRUE;

    /* BSS Type */
    if(mget_DesiredBSSType() != get_bss_type(cap_info))
    {
    	printk("check_bss_capability_info_change 1\n");
        ret_val = BFALSE;
    }
    else
    {
        ret_val = check_bss_mac_privacy_change(cap_info,bss_dscr);
		printk("check_bss_capability_info_change 2\n");
        if(ret_val == BTRUE)
        {
        	printk("check_bss_capability_info_change 3\n");
            ret_val = check_misc_cap_info(cap_info);
        }
    }

    return ret_val;
}
//chenq add end

//chenq add for auto set tx/rx power 2013-07-29
#ifdef TROUT2_WIFI_IC
extern UWORD32 g_nv_11b_tx_pwr[4];
extern UWORD32 g_nv_11g_tx_pwr[4];
extern UWORD32 g_nv_11n_tx_pwr[4];
extern UWORD32 g_0x57_delta;
extern UWORD32 g_pwr_tpc_switch;
extern UWORD32 g_pwr_tpc_switch_last;

#define TX_PWR_TH1  100
#define TX_PWR_TH2  -25
#define TX_PWR_TH3  -34
#define TX_PWR_TH4  -47
#define TX_PWR_TH_DELTA 2

#define RX_PWR_TH -35
#define RX_PWR_TH_DELTA 3
#define SAME_RSSI_TIME 4
#define TBTT_ISR_CNT_END 5
#define PA_SEL_BIT  0x0E00
#define LNA_BPS_BIT 0x38

#if 1

void set_tx_pwr_pa_sel_from_self_adaptive(UWORD32 new_tx_pwr_11b, UWORD32 new_tx_pwr_11g, UWORD32 pa_value){
	static UWORD32 last_pwr_tx_11b = 0;
	static UWORD32 last_pwr_tx_11g = 0;
	static UWORD32 last_pa_value = 0;
	if(1 == g_pwr_tx_rx_11abgn_reset){
		last_pwr_tx_11b = 0;
		last_pwr_tx_11g = 0;
		last_pa_value = 0;
		g_pwr_tx_rx_11abgn_reset = 0;
	}
	if(last_pwr_tx_11g != new_tx_pwr_11g){  
		last_pwr_tx_11g = new_tx_pwr_11g;
		write_dot11_phy_reg(rTXPOWER11A,(UWORD32)(new_tx_pwr_11g));  			
//	printk("tpc:[%s] set reg 0x70 = %x a_rssi[%d] l_rssi[%d]\n", __FUNCTION__,  new_tx_pwr_11g,a_rssi,l_rssi);
	}  	 		
	if(last_pwr_tx_11b  != new_tx_pwr_11b){ 
		last_pwr_tx_11b = new_tx_pwr_11b;
		write_dot11_phy_reg(rTXPOWER11B,(UWORD32)(new_tx_pwr_11b));  			
//	printk("tpc:[%s] set reg 0x71 = %x a_rssi[%d]\n", __FUNCTION__,  new_tx_pwr_11b,a_rssi,l_rssi);
	}  	

//printk("tpc:[%s][%d] last_pa_value[0x%x] pa_value[0x%x]\n",__FUNCTION__,__LINE__,last_pa_value,pa_value);

	if((last_pa_value & PA_SEL_BIT) != (pa_value & PA_SEL_BIT)){
		UWORD32 reg_data = 0;
 //printk("tpc:last_pa_value[0x%x] pa_value[0x%x] a_rssi[%d] l_rssi[%d]\n",last_pa_value,pa_value,a_rssi,l_rssi);
		reg_data = convert_to_le(host_read_trout_reg(0x160)); //PA_SEL_CFG
		reg_data &= ~(PA_SEL_BIT);
		reg_data |= (pa_value & PA_SEL_BIT);
		host_write_trout_reg(reg_data,convert_to_le(0x160));
		last_pa_value = pa_value;
	}
}

extern int  MxdRfSwtchLnaCfg(UWORD32 flagOnOff); // 0:off  1: On


static UWORD8 in_margin(WORD32 cur_rssi,WORD32 last_rssi,WORD32 margin_l,WORD32 margin_r)
{
    if((margin_l<=cur_rssi)&&(cur_rssi<=margin_r)&&(margin_l<=last_rssi)&&(last_rssi<=margin_r))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void uptate_rssi4_auto_set_tx_rx_power(void)
{
	static WORD32 total_rssi = 0;
	WORD32 avg_rssi = 0;
    static WORD32 avg_rssi_last = 0;
	static UWORD8  update_rssi_cnt_max = 50;
	WORD32 pwr_tx_11b  = 0;
	WORD32 pwr_tx_11g  = 0;
	WORD32 pwr_tx_11n  = 0;
	UWORD32 pa_value = 0;
	WORD32 pwr_rx_11abgn = 0;
	static WORD32 first_set_lna   = 1;
	static WORD32 lna_value_in_nv = 0;
    static WORD32 pa_value_in_nv = 0;
	static WORD32 bt_share = 0;
	static UWORD32 ori_0x57 = 0;
    static UWORD32 ori_0x70= 0;
    static UWORD32 ori_0x71= 0;
    WORD32 temp_rssi = 0;
    UWORD8 pwr_update_flag = 0;
    UWORD8 lna_update_flag = 0;
    UWORD8 rssi_direction = 0;//0:equal 1:up 2:down

	if(first_set_lna == 1)
    {
        UWORD32 reg_0x160 = 0;

        reg_0x160 = convert_to_le(host_read_trout_reg(0x160));
        lna_value_in_nv = reg_0x160 & LNA_BPS_BIT;
        pa_value_in_nv = reg_0x160 & PA_SEL_BIT;
        
        read_dot11_phy_reg(0x57,&ori_0x57);
        read_dot11_phy_reg(0x70,&ori_0x70);
        read_dot11_phy_reg(0x71,&ori_0x71);

        printk("tpc:first # lna[0x%x] pa[0x%x] 57[0x%x] 70[0x%x] 71[0x%x]\n",lna_value_in_nv,pa_value_in_nv,ori_0x57,ori_0x70,ori_0x71);
        //printk("tpc: g_pwr_tpc_switch=%d, g_pwr_tpc_switch_last=%d\n", g_pwr_tpc_switch, g_pwr_tpc_switch_last);

        first_set_lna = 0;
    }
	
    if((g_pwr_tpc_switch==0)&&(g_pwr_tpc_switch_last==1))
    {
        UWORD32 reg_data = 0;
        UWORD32 reg_data_tmp = 0;

        write_dot11_phy_reg(0x70,ori_0x70);  			
        write_dot11_phy_reg(0x71,ori_0x71);  			

        reg_data = convert_to_le(host_read_trout_reg(0x160)); //PA_SEL_CFG

        reg_data_tmp = reg_data;
        reg_data_tmp &= ~(PA_SEL_BIT);
        reg_data_tmp |= pa_value_in_nv;
        reg_data_tmp &= ~(LNA_BPS_BIT);
        reg_data_tmp |= lna_value_in_nv;

        host_write_trout_reg(reg_data_tmp,convert_to_le(0x160));
	
        write_dot11_phy_reg(0x57,ori_0x57);		

        //yangke, 2013-1010, for cmcc 11n signalling test
        write_dot11_phy_reg(0xFF, 0x00);
        write_dot11_phy_reg(0xF0, 0x65);

        //yangke, 2013-10-24, switch off Q_empty check
        disable_rx_buff_based_ack();

        printk("tpc:shutdown restore default config # 160 [0x%x]->[0x%x] 70[0x%x] 71[0x%x] 57[0x%x]\n",reg_data,reg_data_tmp,ori_0x70,ori_0x71,ori_0x57);
    }
	
    if((g_pwr_tpc_switch==1)&&(g_pwr_tpc_switch_last==0))
    {
        //yangke, 2013-1010, for cmcc 11n signalling test
        write_dot11_phy_reg(0xFF, 0x00);
        write_dot11_phy_reg(0xF0, 0x01);

        //yangke, 2013-10-24, switch on Q_empty check
        enable_rx_buff_based_ack();
    
        g_pwr_tx_rx_11abgn_reset = 1;
        g_pwr_rx_11abgn = -1;
        g_update_rssi_count = 0;
        g_tbtt_cnt = 0;
        msleep(10);
        printk("tpc:re-open g_pwr_tx_rx_11agbn_reset[%d] g_pwr_rx_11abgn[%d] a_rssi[%d]\n",g_pwr_tx_rx_11abgn_reset,g_pwr_rx_11abgn,avg_rssi);
    }
	 
    if(g_pwr_tpc_switch_last!=g_pwr_tpc_switch)
    {
        printk("tpc:tpc_switch now[%d] last[%d]\n",g_pwr_tpc_switch,g_pwr_tpc_switch_last);
        g_pwr_tpc_switch_last=g_pwr_tpc_switch;
    }

    if (0 == check_tpc_switch_from_nv())
    {
		//printk("lihua:: shutdown tpc from NV!\r\n");
		return;
    }

    if(!g_pwr_tpc_switch)
    {
        //printk("shutdown tpc\n");
        return;
    }
    else
    {
        //printk("enable tpc\n");
    }
        
#if 1 

	
	if(g_auto_tx_power_cnt != update_rssi_cnt_max)
	{
        //printk("change auto power ptk %d => %d\n",update_rssi_cnt_max,g_auto_tx_power_cnt);		
		update_rssi_cnt_max = g_auto_tx_power_cnt;
		g_update_rssi_count = 0;
		g_tbtt_cnt = 0;
	}

	if( g_update_rssi_count == 0 )
	{
		total_rssi = 0;
	}

	g_update_rssi_count++;
    //total_rssi += get_rssi()-0xFF;

#if 1
    temp_rssi = get_rssi() - 0xFF;
    //printk("tpc:temp_rssi[0x%x][%d]\n",temp_rssi,temp_rssi);

    if(temp_rssi<=-128)
    {
        total_rssi += temp_rssi + 0xFF;
        //printk("tpc:single_rssi 1 [%d][0x%x]\n",(temp_rssi+0xFF),(temp_rssi+0xFF));
    }
    else
    {
        total_rssi += temp_rssi;
        //printk("tpc:single_rssi 2 [%d][0x%x]\n",(temp_rssi),(temp_rssi));
    }
#endif

    pwr_update_flag = 0;
    lna_update_flag = 0;

	if((g_update_rssi_count >= update_rssi_cnt_max) || (g_tbtt_cnt >= TBTT_ISR_CNT_END))
	{
		if(0 != g_update_rssi_count){
			avg_rssi = total_rssi/g_update_rssi_count;

            if(avg_rssi>avg_rssi_last)
            {
                rssi_direction = 1;
            }
            else if(avg_rssi<avg_rssi_last)
            {
                rssi_direction = 2;
            }
            else
            {
                rssi_direction = 0;
            }
		}
		else{
			return;
		}
		
        //     -47 -34 -25  middle
        //       -45 -32 -23  +delta up
        //   -49 -36 -27  -delta down

        if(1==rssi_direction)
        {
            //up direction

            if( avg_rssi < TX_PWR_TH4+TX_PWR_TH_DELTA)        // -45 
            {	
                pwr_tx_11b = g_nv_11b_tx_pwr[3] & 0xFFFF;
                pwr_tx_11g = g_nv_11g_tx_pwr[3] & 0xFFFF;
                pwr_tx_11n = g_nv_11n_tx_pwr[3] & 0xFFFF;
                pa_value = (g_nv_11n_tx_pwr[3] >> 16)  & 0xFFFF;

                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH4-TX_PWR_TH_DELTA,TX_PWR_TH4+TX_PWR_TH_DELTA))
                {
                    //printk("tpc:up margin 3 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }
                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
            }
            else if( avg_rssi < TX_PWR_TH3+TX_PWR_TH_DELTA )   // -32 
            {
                pwr_tx_11b = g_nv_11b_tx_pwr[2] & 0xFFFF;
                pwr_tx_11g = g_nv_11g_tx_pwr[2] & 0xFFFF;
                pwr_tx_11n = g_nv_11n_tx_pwr[2] & 0xFFFF;
                pa_value = (g_nv_11n_tx_pwr[2] >> 16)  & 0xFFFF;
                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH3-TX_PWR_TH_DELTA,TX_PWR_TH3+TX_PWR_TH_DELTA))
                {
                    //printk("tpc:up margin 2 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }
                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
            }
            else if( avg_rssi < TX_PWR_TH2+TX_PWR_TH_DELTA )   // -23; 
            {
                pwr_tx_11b = g_nv_11b_tx_pwr[1] & 0xFFFF;
                pwr_tx_11g = g_nv_11g_tx_pwr[1] & 0xFFFF;
                pwr_tx_11n = g_nv_11n_tx_pwr[1] & 0xFFFF;
                pa_value = (g_nv_11n_tx_pwr[1] >> 16)  & 0xFFFF;
                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH2-TX_PWR_TH_DELTA,TX_PWR_TH2+TX_PWR_TH_DELTA))
                {

                    //printk("tpc:up margin 1 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }
                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
            }
            else if( avg_rssi < TX_PWR_TH1 )  //  100
            {
                pwr_tx_11b = g_nv_11b_tx_pwr[0] & 0xFFFF;
                pwr_tx_11g = g_nv_11g_tx_pwr[0] & 0xFFFF;
                pwr_tx_11n = g_nv_11n_tx_pwr[0] & 0xFFFF;
                pa_value = (g_nv_11n_tx_pwr[0] >> 16)  & 0xFFFF;
                pwr_update_flag = 1;
                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
            }
            else{
                pwr_tx_11b = g_nv_11b_tx_pwr[0] & 0xFFFF;
                pwr_tx_11g = g_nv_11g_tx_pwr[0] & 0xFFFF;
                pwr_tx_11n = g_nv_11n_tx_pwr[0] & 0xFFFF;
                pa_value = (g_nv_11n_tx_pwr[0] >> 16)  & 0xFFFF;
                pwr_update_flag = 0;
                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
            }


        }
        else if(2==rssi_direction)
        {
            //down direction

            if( avg_rssi < TX_PWR_TH4-TX_PWR_TH_DELTA)        // -49 
		{	
			pwr_tx_11b = g_nv_11b_tx_pwr[3] & 0xFFFF;
			pwr_tx_11g = g_nv_11g_tx_pwr[3] & 0xFFFF;
			pwr_tx_11n = g_nv_11n_tx_pwr[3] & 0xFFFF;
			pa_value = (g_nv_11n_tx_pwr[3] >> 16)  & 0xFFFF;
                pwr_update_flag = 1;

                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
		}
                else if( avg_rssi < TX_PWR_TH3-TX_PWR_TH_DELTA )   // -36 
		{
			pwr_tx_11b = g_nv_11b_tx_pwr[2] & 0xFFFF;
			pwr_tx_11g = g_nv_11g_tx_pwr[2] & 0xFFFF;
			pwr_tx_11n = g_nv_11n_tx_pwr[2] & 0xFFFF;
			pa_value = (g_nv_11n_tx_pwr[2] >> 16)  & 0xFFFF;
                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH4-TX_PWR_TH_DELTA,TX_PWR_TH4+TX_PWR_TH_DELTA))
                {
                    //printk("tpc:down margin 3 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }

                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
		}
                else if( avg_rssi < TX_PWR_TH2-TX_PWR_TH_DELTA )   // -27
		{
			pwr_tx_11b = g_nv_11b_tx_pwr[1] & 0xFFFF;
			pwr_tx_11g = g_nv_11g_tx_pwr[1] & 0xFFFF;
			pwr_tx_11n = g_nv_11n_tx_pwr[1] & 0xFFFF;
			pa_value = (g_nv_11n_tx_pwr[1] >> 16)  & 0xFFFF;
                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH3-TX_PWR_TH_DELTA,TX_PWR_TH3+TX_PWR_TH_DELTA))
                {
                    //printk("tpc:down margin 2 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }

                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
		}
		else if( avg_rssi < TX_PWR_TH1 )  //  100
		{
			pwr_tx_11b = g_nv_11b_tx_pwr[0] & 0xFFFF;
			pwr_tx_11g = g_nv_11g_tx_pwr[0] & 0xFFFF;
			pwr_tx_11n = g_nv_11n_tx_pwr[0] & 0xFFFF;
			pa_value = (g_nv_11n_tx_pwr[0] >> 16)  & 0xFFFF;
                if(in_margin(avg_rssi,avg_rssi_last,TX_PWR_TH2-TX_PWR_TH_DELTA,TX_PWR_TH2+TX_PWR_TH_DELTA))
                {
                    //printk("tpc:down margin 1 a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
                    pwr_update_flag = 0;
                }
                else
                {
                    pwr_update_flag = 1;
                }

                //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
		}
		else{
			pwr_tx_11b = g_nv_11b_tx_pwr[0] & 0xFFFF;
			pwr_tx_11g = g_nv_11g_tx_pwr[0] & 0xFFFF;
			pwr_tx_11n = g_nv_11n_tx_pwr[0] & 0xFFFF;
			pa_value = (g_nv_11n_tx_pwr[0] >> 16)  & 0xFFFF;
                        pwr_update_flag = 0;
                        //printk("tpc:[%s] avg_rssi = %d, 11b = %x, 11g = %x, 11n = %x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
                        //printk("tpc:[%s] avg_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, avg_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
		}
        }
        else
        {
            //equal
            //printk("tpc:pwr equal\n");
            pwr_update_flag = 0;
        }

        if( avg_rssi > RX_PWR_TH+RX_PWR_TH_DELTA )	 
		{
			pwr_rx_11abgn = 0;    //bypass LNA
            lna_update_flag = 1;
            //printk("tpc:[%s] avg_rssi = %d > %d, rx power bypass LNA\n", __FUNCTION__, avg_rssi, RX_PWR_TH);
        }
        else if(avg_rssi < RX_PWR_TH-RX_PWR_TH_DELTA)
        {
            pwr_rx_11abgn = 1;    //use LNA
            lna_update_flag = 1;
            //printk("tpc:[%s] avg_rssi = %d  <= %d, rx power enable LNA\n", __FUNCTION__, avg_rssi, RX_PWR_TH);
		}
		else
		{
            //printk("lna margin a_rssi[%d] l_rssi[%d]\n",avg_rssi,avg_rssi_last);
            lna_update_flag = 0;
		}

        if(pwr_update_flag)
        {
		set_tx_pwr_pa_sel_from_self_adaptive(pwr_tx_11b, pwr_tx_11g, pa_value);
        }
        else
        {
            //printk("tpc:tx magin\n");
        }

        if(lna_update_flag)
        {
		if( (pwr_rx_11abgn != g_pwr_rx_11abgn) || (1 == bt_share))
		{		
                //printk("lna update status[%d] a_rssi[%d] l_rssi[%d]\n",pwr_rx_11abgn,avg_rssi,avg_rssi_last);

			if(pwr_rx_11abgn == 0)
			{
				if(0 == MxdRfSwtchLnaCfg(0)){
					bt_share = 0;
                        write_dot11_phy_reg(0x57,(ori_0x57+g_0x57_delta));
				} 
				else{
					bt_share = 1;
				}
				#if 0
				UWORD32 write_tmp = 0;
				WORD32 read_tmp = 0;
				UWORD32 reg_data = 0;
				reg_data = convert_to_le(host_read_trout_reg(0x160)); //M_ANT_SEL_CFG
				if( (reg_data & LNA_BPS_BIT) != 0 ) //000 XXX
				{			
					reg_data &= ~(LNA_BPS_BIT);				
					host_write_trout_reg(reg_data,convert_to_le(0x160));
				}
				printk("[%s] pwr_rx_11abgn bypass LAN, M_ANT_SEL_CFG = %08x\n", __FUNCTION__, reg_data);

				// don't config the 0x57 reg anymore.[zhongli wang 20130925]
				/*write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
				write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
				write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
				printk("[%s] write %x to phy reg 0x57\n", __FUNCTION__, (UWORD32)0x13);*/
				#endif
			}
			else
			{
				if(0 == MxdRfSwtchLnaCfg(1)){
					bt_share = 0;

                        //printk("tpc:%d ori_0x57[0x%x] g_0x57_delta[0x%x]\n",__LINE__,ori_0x57,g_0x57_delta);
                        write_dot11_phy_reg(0x57,ori_0x57);
				} 
				else{
					bt_share = 1;
				}
				#if 0
				UWORD32 read_tmp = 0;
				UWORD32 write_tmp = 0;
				UWORD32 reg_data = 0;

				reg_data = convert_to_le(host_read_trout_reg(0x160)); //M_ANT_SEL_CFG
				if( (reg_data & LNA_BPS_BIT) != lna_value_in_nv ) //010 XXX
				{				
					reg_data &= ~(LNA_BPS_BIT);
					reg_data |= lna_value_in_nv;				
					host_write_trout_reg(reg_data,convert_to_le(0x160));
				}
				printk("[%s] pwr_rx_11abgn use LAN, M_ANT_SEL_CFG = %08x\n",__FUNCTION__,reg_data);
				// don't config the 0x57 reg anymore.[zhongli wang 20130925]
				/*write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
				write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
				write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
				printk("[%s] write %x to phy reg 0x57\n", __FUNCTION__, (UWORD32)0x7);*/
				#endif
			}
		}
        }
        else
        {
            // printk("tpc:lna magin\n");
        }

        if(pwr_update_flag)
        { 
		g_pwr_tx_11b = pwr_tx_11b;
		g_pwr_tx_11g = pwr_tx_11g;
		g_pwr_tx_11n = pwr_tx_11n;		
        }

        if(lna_update_flag)
        { 
		g_pwr_rx_11abgn = pwr_rx_11abgn;	
        }

        avg_rssi_last = avg_rssi;
		g_update_rssi_count = 0;	
		g_tbtt_cnt = 0;		
	}
#endif
}
#else



//if rssi changed return 1, otherwise return 0
WORD32 is_rssi_change(void* cur_rssi, void* last_rssi){
	WORD32 tmp_cur_rssi = 0;
	WORD32 tmp_last_rssi = 0;

	tmp_cur_rssi = *((WORD32*)cur_rssi);
	tmp_last_rssi = *((WORD32*)last_rssi);

	if((tmp_cur_rssi >= TX_PWR_TH1) && (tmp_last_rssi >= TX_PWR_TH1)){
		return 0;
	}
	else if(((tmp_cur_rssi < TX_PWR_TH1) && (tmp_cur_rssi >= TX_PWR_TH2))
			&& ((tmp_last_rssi < TX_PWR_TH1) && (tmp_last_rssi >= TX_PWR_TH2))){
		return 0;
	}
	else if(((tmp_cur_rssi < TX_PWR_TH2) && (tmp_cur_rssi >= TX_PWR_TH3))
			&& ((tmp_last_rssi < TX_PWR_TH2) && (tmp_last_rssi >= TX_PWR_TH3))){
		return 0;
	}
	else if(((tmp_cur_rssi < TX_PWR_TH3) && (tmp_cur_rssi >= TX_PWR_TH4))
			&& ((tmp_last_rssi < TX_PWR_TH3) && (tmp_last_rssi >= TX_PWR_TH4))){
		return 0;
	}
	else if((tmp_cur_rssi < TX_PWR_TH4) && (tmp_last_rssi < TX_PWR_TH4)){
		return 0;
	}
	else{
		return 1;
	}	
}

void uptate_rssi4_auto_set_tx_rx_power(void)
{
	static WORD32 last_beacon_rssi = 0;
	static WORD32 same_rssi_cnt = 0;
	WORD32 cur_beacon_rssi = 0;
	WORD32 pwr_tx_11b  = 0;
	WORD32 pwr_tx_11g  = 0;
	WORD32 pwr_tx_11n  = 0;
	WORD32 pwr_rx_11abgn = 0;

	cur_beacon_rssi = get_rssi()-256;
	if(0 == is_rssi_change(&cur_beacon_rssi, &last_beacon_rssi)){
		same_rssi_cnt++;
		if(SAME_RSSI_TIME == same_rssi_cnt){
			if(cur_beacon_rssi >= TX_PWR_TH1)        //Pwr_th1 = -29; 
			{	
				pwr_tx_11b = 0x8;
				pwr_tx_11g = 0x20;
				pwr_tx_11n = 0x20;
				//printk("[%s] cur_beacon_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, cur_beacon_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
			}
			else if(cur_beacon_rssi >= TX_PWR_TH2)   //Pwr_th2 = -35; 
			{
				pwr_tx_11b = 0x8;
				pwr_tx_11g = 0x23;
				pwr_tx_11n = 0x23;
				//printk("[%s] cur_beacon_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, cur_beacon_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
			}
			else if(cur_beacon_rssi >= TX_PWR_TH3)   //Pwr_th3 = -41; 
			{
				pwr_tx_11b = 0xF;
				pwr_tx_11g = 0x29;
				pwr_tx_11n = 0x29;
				//printk("[%s] cur_beacon_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, cur_beacon_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
			}
			else if(cur_beacon_rssi >= TX_PWR_TH4)  //Pwr_th3 = -47; 
			{
				pwr_tx_11b = 0xF;
				pwr_tx_11g = 0x2F;
				pwr_tx_11n = 0x2C;
				//printk("[%s] cur_beacon_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, cur_beacon_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
			}
			else{
				pwr_tx_11b = 0x10;
				pwr_tx_11g = 0x2F;
				pwr_tx_11n = 0x2C;
				//printk("[%s] cur_beacon_rssi:%d, 11b-0x%x, 11g-0x%x, 11n-0x%x\n", __FUNCTION__, cur_beacon_rssi, pwr_tx_11b, pwr_tx_11g, pwr_tx_11n);
			}
			if(cur_beacon_rssi > RX_PWR_TH)	 
			{
				pwr_rx_11abgn = 0;    //bypass LNA
				//printk("[%s][cur_beacon_rssi:%d] > %d, rx power bypass LNA\n", __FUNCTION__, cur_beacon_rssi, pwr_rx);
			}
			else
			{
				pwr_rx_11abgn = 1;    //use LNA
				//printk("[%s][cur_beacon_rssi:%d] <= %d, rx power enable LNA\n", __FUNCTION__, cur_beacon_rssi, pwr_rx);
			}
			g_pwr_tx_11b = pwr_tx_11b;
			g_pwr_tx_11g = pwr_tx_11g;
			g_pwr_tx_11n = pwr_tx_11n;		
			g_pwr_rx_11abgn = pwr_rx_11abgn;	
		}	
	}
	else{
		same_rssi_cnt = 0;
		last_beacon_rssi = cur_beacon_rssi;
	}
}
#endif
#if 0
extern UWORD32 g_cmcc_set_max_pwr;
void auto_set_tx_rx_power(void)
{
	static WORD32 pwr_tx_11agn  = 0x00112233;
	static WORD32 pwr_tx_11b    = 0x00112233;
	static WORD32 pwr_rx_11abgn = 0x00112233;
	UWORD32 reg_data = 0;
	
	static WORD32 first_set_lna   = 1;
	static WORD32 lna_value_in_nv = 0;

	if(first_set_lna == 1)
	{
		lna_value_in_nv = convert_to_le(host_read_trout_reg(0x160));
		printk("[%s] lna_value_in_nv == %08x\n", __FUNCTION__, lna_value_in_nv);
		lna_value_in_nv &= 0x38;
		first_set_lna = 0;
	}

	if(g_pwr_tx_rx_11abgn_reset == 1)
	{
		//printk("[%s] auto_set_tx_rx_power,after reset all flag init\n", __FUNCTION__);
		g_pwr_tx_rx_11abgn_reset = 0;
		pwr_tx_11agn = 0x00112233;
		pwr_tx_11b   = 0x00112233;
		pwr_rx_11abgn= 0x00112233;
		return;
	}

	if(g_pwr_rx_11abgn == -1)
	{
		//printk("[%s] uptate_rssi4_auto_set_tx_rx_power don't run yet,will not do auto_set_tx_rx_power\n", __FUNCTION__);
		return;
	}

#if 1
	if(0 == TX_POW_SEL_HDR){
		if(1 == g_cmcc_set_max_pwr){
			if(( pwr_tx_11agn != 0x2C) || (pwr_tx_11b != 0x8)){
				pwr_tx_11agn = 0x2C;
				pwr_tx_11b = 0x8;
				write_dot11_phy_reg(rTXPOWER11A,(UWORD32)0x2C);  
				write_dot11_phy_reg(rTXPOWER11B,(UWORD32)0x8);		
			}
		}
		else{		
		 	if( pwr_tx_11agn != g_pwr_tx_11g){  	 		
		 		pwr_tx_11agn = g_pwr_tx_11g;  //zl@@
				write_dot11_phy_reg(rTXPOWER11A,(UWORD32)(pwr_tx_11agn&0xffff));  
				//trout_reg_write(_TROUT_ANT_SEL_REG_,(((UWORD32)pwr_tx_11agn>>16)&0xffff));//sys 58 reg???
		
			}  
		 		
		 	if( pwr_tx_11b  != g_pwr_tx_11b ){  
				pwr_tx_11b   = g_pwr_tx_11b;     //zl@@
				//write_dot11_phy_reg(rTXPOWER11B,(UWORD32)pwr_tx_11b);			
				write_dot11_phy_reg(rTXPOWER11B,(UWORD32)(pwr_tx_11b&0xffff));  
				
				//trout_reg_write(_TROUT_ANT_SEL_REG_,(((UWORD32)pwr_tx_11b>>16)&0xffff)); //sys 58 reg???
			}  
			printk("[%s] set self-adaptive tx power 0x70 = %x, 0x71 = %x\n", __FUNCTION__, g_pwr_tx_11n, g_pwr_tx_11b);
		}
	}
#endif

	if( pwr_rx_11abgn != g_pwr_rx_11abgn )
	{
		pwr_rx_11abgn = g_pwr_rx_11abgn;
		
		if(pwr_rx_11abgn == 0)
		{
			UWORD32 write_tmp = 0;
			WORD32 read_tmp = 0;
			reg_data = convert_to_le(host_read_trout_reg(0x160)); //M_ANT_SEL_CFG
			if( (reg_data & 0x38) != 0 ) //000 XXX
			{			
				reg_data &= ~(0x38);				
				host_write_trout_reg(reg_data,convert_to_le(0x160));
			}
			printk("[%s] pwr_rx_11abgn bypass LAN, M_ANT_SEL_CFG = %08x\n", __FUNCTION__, reg_data);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x13);
			printk("[%s] write %x to phy reg 0x57\n", __FUNCTION__, (UWORD32)0x13);
		}
		else
		{
			UWORD32 read_tmp = 0;
			UWORD32 write_tmp = 0;
			reg_data = convert_to_le(host_read_trout_reg(0x160)); //M_ANT_SEL_CFG
			if( (reg_data & 0x38) != lna_value_in_nv ) //010 XXX
			{				
				reg_data &= ~(0x38);
				reg_data |= lna_value_in_nv;				
				host_write_trout_reg(reg_data,convert_to_le(0x160));
			}
			printk("[%s] pwr_rx_11abgn use LAN, M_ANT_SEL_CFG = %08x\n",__FUNCTION__,reg_data);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
			write_dot11_phy_reg(rPHYCCAINSERTIONVAL, (UWORD32)0x7);
			printk("[%s] write %x to phy reg 0x57\n", __FUNCTION__, (UWORD32)0x7);
		}
	}
}
#endif
#endif
//chenq add end
#endif /* IBSS_BSS_STATION_MODE */
