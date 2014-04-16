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
/*  File Name         : sme_sta.c                                            */
/*                                                                           */
/*  Description       : This file contains the SME functions specific to     */
/*                      STA mode.                                            */
/*                                                                           */
/*  List of Functions : send_mlme_rsp_to_host_sta                            */
/*                      handle_scan_rsp                                      */
/*                      handle_join_rsp                                      */
/*                      handle_start_rsp                                     */
/*                      handle_auth_rsp                                      */
/*                      handle_asoc_rsp                                      */
/*                      prepare_mlme_scan_req                                */
/*                      prepare_mlme_join_req                                */
/*                      prepare_mlme_start_req                               */
/*                      prepare_mlme_auth_req                                */
/*                      prepare_mlme_asoc_req                                */
/*                      check_scan_match                                     */
/*                      prepare_scan_response                                */
/*                      obss_scan_to_fn                                      */
/*                      start_obss_scan_timer                                */
/*                      stop_obss_scan_timer                                 */
/*                      start_obss_scan                                      */
/*                      end_obss_scan                                        */
/*                      set_start_scan_req_sta                               */
/*                      initiate_scan_procedure                              */
/*                      initiate_scan                                        */
/*                      initiate_start                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "iconfig.h"
#include "mib.h"
#include "sme_sta.h"
#include "qmu_if.h"
#include "controller_sta_prot_if.h"
#include "mac_init.h"
#include "qmu_tx.h"
/*leon liu added for powersave timer header 2013-4-1*/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include "ps_timer.h"
#endif
//zhuyg add
#ifdef TROUT_WIFI_NPI
#include "trout_wifi_npi.h"
#endif
/*leon liu added for cfg80211 layer on 2013-07-12*/
#ifdef CONFIG_CFG80211
#include "trout_cfg80211.h"
#endif
// chenq add
int itm_scan_flag = 0;

BOOL_T g_wps_scan_req_from_user         = BFALSE; // caisf add for fix wps scan bug. 1121
extern void  MxdRfGetRfMode(unsigned int * pFlagFmBit2BtBit1WfBit0 );

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void handle_scan_rsp(mac_struct_t *mac, UWORD8 *msg);
static void handle_join_rsp(mac_struct_t *mac, UWORD8 *msg);
static void handle_start_rsp(mac_struct_t *mac, UWORD8 *msg);
static void handle_auth_rsp(mac_struct_t *mac, UWORD8 *msg);
static void handle_asoc_rsp(mac_struct_t *mac, UWORD8 *msg);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
extern struct wake_lock  scan_ap_lock;
#endif
#endif

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

/* The various types of MLME response types are handled in different ways. A */
/* function pointer array is maintained to map the different functions to    */
/* the different MLME response types.                                        */

static void (*g_handle_mlme_rsp_func_sta[MAX_MLME_RSP_TYPE])
            (mac_struct_t *mac, UWORD8 *msg) =
{
    handle_scan_rsp,
    handle_start_rsp,
    handle_join_rsp,
    handle_auth_rsp,
    handle_asoc_rsp
};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_mlme_rsp_to_host_sta                                */
/*                                                                           */
/*  Description   : This function handles the request to send a MLME         */
/*                  response to the host based on the SME mode in use in the */
/*                  STA mode.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Type of MLME Response                                 */
/*                  3) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : g_handle_mlme_rsp_func_sta                               */
/*                                                                           */
/*  Processing    : In case test or auto SME modes are in use, the MLME      */
/*                  response is not sent to the host. Instead the response   */
/*                  is parsed and the next action appropriately initiated.   */
/*                  In case of manual SME mode (currently not supported) the */
/*                  response is sent to the host in appropraite format.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_mlme_rsp_to_host_sta(mac_struct_t *mac, UWORD8 type, UWORD8 *msg)
{
#ifndef ENABLE_STACK_ESTIMATION
    g_handle_mlme_rsp_func_sta[type](mac, msg);
#else /* ENABLE_STACK_ESTIMATION */
    handle_scan_rsp(mac, msg);
    handle_start_rsp(mac, msg);
    handle_join_rsp(mac, msg);
    handle_auth_rsp(mac, msg);
    handle_asoc_rsp(mac, msg);
#endif /* ENABLE_STACK_ESTIMATION */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_scan_rsp                                          */
/*                                                                           */
/*  Description   : This function processes the MLME scan response for auto  */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Scan Response is parsed. In case the status is  */
/*                  successful the BSS descriptor list is processed in order */
/*                  to find a BSS matching with the user requirements. If a  */
/*                  matching BSS is found a join request is created and the  */
/*                  MAC API to join is called.  In case of a failure the     */
/*                  scan request is sent again for INFRASTRUCTURE BSS type   */
/*                  while a start request is created and the MAC API to      */
/*                  start is called for INDEPENDENT type.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_scan_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    UWORD8     bss_index = 0;
    scan_rsp_t *scan_rsp = (scan_rsp_t *)msg;
    //chenq add 2013-10-10
	BOOL_T is_send_scan_cmp = BFALSE;

	TROUT_FUNC_ENTER;
	//chenq add 0730
	itm_scan_flag = 0;
#ifdef CONFIG_CFG80211
	complete(&scan_completion);
#endif

    /* If scan was initiated while in connected mode */
    /* go back to connected mode */
    if(g_keep_connection == BTRUE)
    {
        end_obss_scan(mac);
	//chenq add 2012-11-19
	send_mac_status(MAC_SCAN_CMP);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if(wake_lock_active(&scan_ap_lock))  
            wake_unlock(&scan_ap_lock);  
        printk("@@@: release scan_ap_lock\n");  
#endif
#endif
        TROUT_FUNC_EXIT;
        return;
    }

    /* Call a function to handle the scan response based on the protocol in  */
    /* use. If handled, return. No further processing is required.           */
    if(BTRUE == handle_scan_rsp_prot(mac, msg))
    {
        g_scan_source = DEFAULT_SCAN;
        set_site_survey(SITE_SURVEY_OFF);
        g_keep_connection = BFALSE;
		//chenq add 2012-11-19
		send_mac_status(MAC_SCAN_CMP);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if(wake_lock_active(&scan_ap_lock))  
            wake_unlock(&scan_ap_lock);  
        printk("@@@: release scan_ap_lock\n");  
#endif
#endif
        TROUT_FUNC_EXIT;
        return;
    }

#ifdef DEBUG_MODE
	g_mac_stats.scan_ap_count += scan_rsp->num_dscr;
#endif

    if(get_site_survey_status() == SITE_SURVEY_OFF)
    {
        bss_index = check_scan_match(scan_rsp);

        if((scan_rsp->result_code == SUCCESS_MLMESTATUS) &&
           (bss_index < MAX_NUM_DESCRIPTOR))
        {
			// 20120709 caisf add, merged ittiam mac v1.2 code
            update_non_p2p_join_prot();
            initiate_join(mac, &(scan_rsp->bss_dscr[bss_index]));
            TROUT_DBG4("Status: Scan Successful. Initaiting Join: \n");

            /* ITM_DEBUG */
#ifdef DEBUG_MODE
            {
                bss_dscr_t *dscr = &(scan_rsp->bss_dscr[bss_index]);

                TROUT_DBG4("SSID  = %s\n",dscr->ssid);
                TROUT_DBG4("BSSID = %x:%x:%x:%x:%x:%x\n",dscr->bssid[0],
                dscr->bssid[1], dscr->bssid[2], dscr->bssid[3], dscr->bssid[4], dscr->bssid[5]);
                TROUT_DBG4("Channel = %d\n",dscr->channel);
            }
#endif /* DEBUG_MODE */
        }
        else if(g_scan_source & USER_SCAN)
        {
            /* Wait for the user to give join request */

	    //chenq mod 2012-11-01
	    #if 0
            set_mac_state(WAIT_JOIN);
            start_mgmt_timeout_timer(USER_INPUT_TIMEOUT);
	    #else
	    set_mac_state(WAIT_SCAN);
  	    #endif

	    //chenq add 2012-11-19 & mod 2013-10-10
	    //send_mac_status(MAC_SCAN_CMP);
	    is_send_scan_cmp = BTRUE;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if(wake_lock_active(&scan_ap_lock))  
            wake_unlock(&scan_ap_lock);  
        printk("@@@: release scan_ap_lock\n");  
#endif
#endif
        }
        else
        {
            if(mget_DesiredBSSType() == INFRASTRUCTURE)
            {
	    		// modify by Ke.Li at 2013-01-09 for refresh UI ap list bug
		    	//send_mac_status(MAC_SCAN_CMP);
            	// end by Ke.Li at 2013-01-09 for refresh UI ap list bug
#ifdef MAC_P2P
			if (BTRUE == mget_p2p_enable())
			{
				if (BTRUE == is_p2p_grp_form_in_prog())
				{
					TROUT_DBG4("P2P scan complete, continue P2P scan!!!\n");
					send_mac_status(MAC_SCAN_CMP);
					initiate_scan(mac);
					goto OUT;
				}
			}
#endif /* ifdef MAC_P2P */
				initiate_scan(mac);
				TROUT_DBG3("Status: Scan Failed. Result-Code = %d. Reinitiating Scan\n\r",
				scan_rsp->result_code);
				/*leon liu added, report scan aborted to upper layer*/
				#ifdef CONFIG_CFG80211
				trout_cfg80211_report_scan_done(g_mac_dev, 1);
				#endif
            }
            else /* INDEPENDENT */
            {
                initiate_start(mac);

                /* ITM_DEBUG */
                TROUT_DBG3("Status: Scan Failed. Result-Code = %d. Starting IBSS N/w \n",
                scan_rsp->result_code);
            }
        }
    }
    else
    {
    	

		#if 0
        /* Wait for the user to give join request */
         set_mac_state(WAIT_JOIN);
         start_mgmt_timeout_timer(USER_INPUT_TIMEOUT);
		#else
		//chenq mod 2012-11-01
		if(g_scan_source & USER_SCAN)
        {
		//chenq mod 2012-11-01
		#if 0
                set_mac_state(WAIT_JOIN);
                start_mgmt_timeout_timer(USER_INPUT_TIMEOUT);
		#else
		set_mac_state(WAIT_SCAN);
		#endif

		//chenq add 2012-11-19 & mod 2013-10-10
		//send_mac_status(MAC_SCAN_CMP);
		is_send_scan_cmp = BTRUE;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	      if(wake_lock_active(&scan_ap_lock))  
                  wake_unlock(&scan_ap_lock);  
              printk("@@@: release scan_ap_lock\n");  
#endif
#endif
        }
		else
		{
			/* Wait for the user to give join request */
         	set_mac_state(WAIT_JOIN);
         	start_mgmt_timeout_timer(USER_INPUT_TIMEOUT);
		}
		#endif
    }
#ifdef MAC_P2P
OUT:
#endif
    g_scan_source = DEFAULT_SCAN;
    set_site_survey(SITE_SURVEY_OFF);
    g_keep_connection = BFALSE;
    
    //chenq add 2013-10-10
    if(is_send_scan_cmp == BTRUE)
    {
    	/*leon liu added, start pstimer when not connected*/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	pstimer_start(&pstimer);
#endif
        send_mac_status(MAC_SCAN_CMP);
    }
	
    TROUT_FUNC_EXIT;
}

//zhangzhao add 2013-01-12
unsigned int scan_need_goon()
{
	if(g_default_scan_limit <= MAX_NR_SCAN_TIMES)
		return 1;
	else
		return 0;
}

void init_scan_limit()
{
	g_default_scan_limit = 0;
}

void inc_scan_limit()
{
	g_default_scan_limit++;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_join_rsp                                          */
/*                                                                           */
/*  Description   : This function processes the MLME join response for auto  */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Join Response is parsed. In case the status is  */
/*                  successful the BSS type is checked. In case it is of     */
/*                  INDEPENDENT type no further action is taken. In case it  */
/*                  is of INFRASTRUCTURE type an authentication request is   */
/*                  created and the MAC API to authenticate is called. In    */
/*                  case of a failure the scan request is sent again for     */
/*                  INFRASTRUCTURE BSS type while a start request is created */
/*                  and the MAC API to start is called for INDEPENDENT type. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_join_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    join_rsp_t *join_rsp = (join_rsp_t *)msg;

	TROUT_FUNC_ENTER;
    if(join_rsp->result_code == SUCCESS_MLMESTATUS)
    {
        /* Call a function to handle a successful join response based on the */
        /* protocol. If handled, return. No further processing is required.  */
        if(BTRUE == handle_join_success_prot(mac))
        {
			TROUT_FUNC_EXIT;
            return;
        }

        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            initiate_auth(mac);

            /* ITM_DEBUG */
            TROUT_DBG4("Status: Initiating Authentication.\n\r");
        }
    }
    else
    {
        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            initiate_scan(mac);

            /* ITM_DEBUG */
            TROUT_DBG4("Status: Join Failed. Result-Code = %d. Reinitiating Scan.\n\r",
            join_rsp->result_code);
        }
        else /* INDEPENDENT */
            initiate_start(mac);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_start_rsp                                         */
/*                                                                           */
/*  Description   : This function processes the MLME start response for auto */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Start Response is parsed. In case the status is */
/*                  successful no further action is taken. In case of a      */
/*                  failure the start request is sent again.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_start_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    start_rsp_t *start_rsp = (start_rsp_t *)msg;

	TROUT_FUNC_ENTER;
    if(start_rsp->result_code != SUCCESS_MLMESTATUS)
    {
        pkt_mem_free(g_beacon_frame[0]);
        pkt_mem_free(g_beacon_frame[1]);
        initiate_scan(mac);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_auth_rsp                                          */
/*                                                                           */
/*  Description   : This function processes the MLME auth response for auto  */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Auth Response is parsed. In case the status is  */
/*                  successful an association request is created and the MAC */
/*                  API to associate is called. In case of a failure the     */
/*                  scan request is sent again.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_auth_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    auth_rsp_t *auth_rsp = (auth_rsp_t *)msg;

	TROUT_FUNC_ENTER;
    if(auth_rsp->result_code == SUCCESS_MLMESTATUS)
    {
        initiate_asoc(mac);

        /* ITM_DEBUG */
        TROUT_DBG4("Status: Authentication Successful. Initiating Association.\n\r");

    }
	else if(g_auth_retry_cnt < AUTH_MAX_RETRY_CNT){
		g_auth_retry_cnt++;
		initiate_auth(mac);
		TROUT_DBG4("Status: Authentication Failed. Result-Code = %d. retry time = %d\n", auth_rsp->result_code, g_auth_retry_cnt);
	}
    else
    {
        initiate_scan(mac);

        /* ITM_DEBUG */
        TROUT_DBG4("Status: Authentication Failed. Result-Code = %d. ",
        auth_rsp->result_code);
/* leon liu added, keep MAC_CONNECT_FAILED for cfg80211 (trout2.3.5 on 09-19 removed this) */
        // add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        //send_mac_status(MAC_CONNECT_FAILED);
        // end add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        TROUT_DBG4("Reinitiating Scan.\n\r");
    }

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_asoc_rsp                                          */
/*                                                                           */
/*  Description   : This function processes the MLME asoc response for auto  */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Asoc Response is parsed. In case the status is  */
/*                  successful no further action is taken. In case of a      */
/*                  failure the scan request is sent again.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#include "prot_if.h"
#include "amsdu_aggr.h"
#include "csl_linux.h"	//chwg debug, 2013.12.3


void handle_asoc_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    asoc_rsp_t *asoc_rsp = (asoc_rsp_t *)msg;

	TROUT_FUNC_ENTER;
    if(asoc_rsp->result_code != SUCCESS_MLMESTATUS)
    {
    	 	if(g_assoc_retry_cnt < ASSOC_MAX_RETRY_CNT){
			g_assoc_retry_cnt++;
			initiate_asoc(mac);
			TROUT_DBG4("Status: Association Failed. Result-Code = %d. retry time = %d\n", asoc_rsp->result_code, g_assoc_retry_cnt);
		}
		else{
        initiate_scan(mac);

        /* ITM_DEBUG */
        TROUT_DBG4("Status: Association Failed. Result-Code = %d. ",asoc_rsp->result_code);
        // add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        		//send_mac_status(MAC_CONNECT_FAILED);
        // end add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        TROUT_DBG4("Reinitiating Scan.\n\r");
    }
    }
    else
    {
        /* ITM_DEBUG */
        TROUT_DBG4("Status: Association Successful.\n\r");
		//config_amsdu_func(NULL, BTRUE);	//add by chengwg for sta mode.
		config_802_11n_feature(NULL, BTRUE);	//sta mode.

		#ifdef IBSS_BSS_STATION_MODE
		TROUT_DBG4("after associated, init trout self-cts & ps null data!\n");
		coex_null_data_init();	//add by chengwg for wifi&bt coex init.
		host_notify_arm7_con_ap_mode();
		#endif	/* IBSS_BSS_STATION_MODE */
		
		#ifdef TROUT_WIFI_NPI
		printk("npi: coonnect ok\n");
		g_connect_ok_flag = 1;
		#endif
#ifdef WAKE_LOW_POWER_POLICY
		printk("%s: start flow detect timer!\n", __func__);
		clear_history_flow_record();
		restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
#endif	/* WAKE_LOW_POWER_POLICY */
    }

	#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/*leon liu added, stop pstimer*/
	pstimer_stop(&pstimer);
	#endif
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_scan_req                                    */
/*                                                                           */
/*  Description   : This function prepares the MLME scan request.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the scan request buffer                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given scan request buffer is updated with the user   */
/*                  parameters that are available from the MIB.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_scan_req(scan_req_t *scan_req)
{
    //UWORD8 i = 0;
    //UWORD8 k = 0;
    BOOL_T scan_all_channels = BFALSE;
	UWORD32 coex_reg = 0;

	//chenq add a addr
	UWORD8 addr0[6] = {0};
	MxdRfGetRfMode(&coex_reg);

	TROUT_FUNC_ENTER;
    mem_set(scan_req, 0, sizeof(scan_req_t));

    /* Set the fields of the scan request according to the user settings     */
    /* (MIB has been updated with these).                                    */
    if(g_scan_source != DEFAULT_SCAN)
    {
        scan_req->bss_type = INFRASTRUCTURE;
        mem_set(scan_req->bssid,0,6);
        mem_set(scan_req->ssid,0,MAX_SSID_LEN);
    }
    else
    {
        scan_req->bss_type = mget_DesiredBSSType();
		if(scan_need_goon())
		{ //zhangzhao add if 2013-01-13
        	//chenq mod 2013-10-10
        	//strcpy((WORD8 *)scan_req->bssid, (const WORD8 *)mget_bssid());
        	memcpy((WORD8 *)scan_req->bssid, (const WORD8 *)g_prefered_bssid,6);
        	strcpy((WORD8 *)scan_req->ssid, (const WORD8 *)mget_DesiredSSID());
		}
		else
		{ //zhangzhao add else 2013-01-13
        	mem_set(scan_req->bssid,0,6);
        	mem_set(scan_req->ssid,0,MAX_SSID_LEN);
		}
    }

    /* If user requested to scan all channels do it */
    if((g_scan_source & USER_SCAN) &&
       (get_site_survey_status() == SITE_SURVEY_ALL_CH))
    {
        scan_all_channels = BTRUE;
    }
	//chenq add for ui req scan 1 channel 2013-01-11
	#if 1
	//chenq mod 2013-10-10
	//else if( (strlen(scan_req->ssid) > 0 ) && (memcmp(addr0,mget_bssid(),6)) )
	else if( (strlen(scan_req->ssid) > 0 ) && (memcmp(addr0,g_prefered_bssid,6)) )
	{
		scan_all_channels = BFALSE;
	}
	#endif
    /* In BSS STA mode, do all channel scan */
    else if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        scan_all_channels = BTRUE;
    }
    else
    {
        scan_all_channels = BFALSE;
    }

    if(scan_all_channels == BTRUE)
    {
#if 0 // 1226, caisf mod it for reset the sequence of scan channel list from 0 to 14. 
        UWORD8 list_idx = 0;
        UWORD8 ch_idx   = 0;
        UWORD8 freq     = get_current_start_freq();
        UWORD8 num_ch   = get_max_num_channel(freq);

        /* Prepare the channel list. Scanning is done in the current         */
        /* frequency band set by the user, starting from the channel number  */
        /* configured by the user. The list of all channels in the frequency */
        /* band that are supported by the device and valid for the current   */
        /* regulatory domain are included in the scan request channel list.  */
        k = mget_CurrentChannel();
        for(i = 0; i < num_ch; i++, k++)
        {
            ch_idx = (k % num_ch);

            if(is_ch_idx_supported(freq, ch_idx) == BTRUE)
            {
        		TROUT_DBG4("domain: list_idx = %d, ch_idx = %d",list_idx,ch_idx);
                scan_req->channel_list[list_idx++] = ch_idx;
            }
        }
#else
        UWORD8 list_idx = 0;
        UWORD8 ch_idx   = 0;
        UWORD8 freq     = get_current_start_freq();
        UWORD8 num_ch   = get_max_num_channel(freq);

		// caisf add for config max scan channel number, 20130506
		if(num_ch >= get_scan_req_channel_no() && get_scan_req_channel_no() > 0)
		{
        	TROUT_DBG4("set current max scan channel number: %d (%d)\n",
				get_scan_req_channel_no(), num_ch);
			num_ch = get_scan_req_channel_no();
		}
        
        TROUT_DBG4("domain: num_ch = %d\n",num_ch);
        for(ch_idx = 0; ch_idx < num_ch; ch_idx++)
        {
            if(is_ch_idx_supported(freq, ch_idx) == BTRUE)
            {
        		TROUT_DBG5("domain: list_idx = %d, ch_idx = %d\n",list_idx,ch_idx);
				//chenq add for test 2013-02-05
				#ifndef COMBO_SCAN
				scan_req->channel_list[list_idx*5 + 0] = ch_idx;
				scan_req->channel_list[list_idx*5 + 1] = ch_idx;
				scan_req->channel_list[list_idx*5 + 2] = ch_idx;
				scan_req->channel_list[list_idx*5 + 3] = ch_idx;
				scan_req->channel_list[list_idx*5 + 4] = ch_idx;
				list_idx++;
				#else
				int tmp_i = 0;
				if(0 == (coex_reg & BIT1)){
				for(tmp_i = 0;tmp_i < SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1);tmp_i++)
				{
					scan_req->channel_list[list_idx * SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1)  + tmp_i]
						= ch_idx;
					
					//TROUT_DBG4("will scan channel times %d num %d\n",
						//list_idx*SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1) + tmp_i,
						//scan_req->channel_list[list_idx*SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1) + tmp_i]);	
				}
				}
				else{
					for(tmp_i = 0;tmp_i < COEX_SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1);tmp_i++)
					{
						scan_req->channel_list[list_idx * COEX_SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1)  + tmp_i]
							= ch_idx;
					}
				}
				list_idx++;
				#endif
            }
        }
#endif
		#ifndef COMBO_SCAN
            scan_req->num_channels = list_idx * 5;
		#else
		if(0 == (coex_reg & BIT1)){	
                    scan_req->num_channels = list_idx * SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1);
		}
		else{
		    scan_req->num_channels = list_idx * COEX_SCAN_IN_ONE_CHANNEL_CNT * (g_ap_combo_scan_cnt+1);
		}
		#endif  
		TROUT_DBG4("will scan all channel nums %d\n",scan_req->num_channels);
    }
    else
    {
    	TROUT_DBG4("will scan one channel nums %d\n",mget_CurrentChannel());
        /* If there is no site survey, then fix the channel to one */
        scan_req->num_channels    = 5;
        scan_req->channel_list[0] = mget_CurrentChannel();
		scan_req->channel_list[1] = mget_CurrentChannel();
		scan_req->channel_list[2] = mget_CurrentChannel();
		scan_req->channel_list[3] = mget_CurrentChannel();
		scan_req->channel_list[4] = mget_CurrentChannel();
    }

    scan_req->max_channel_time = MAX_SCAN_TIME;
    scan_req->min_channel_time = MIN_SCAN_TIME;
    scan_req->scan_type        = (SCANTYPE_T ) mget_scan_type();
    scan_req->probe_delay = PROBE_DELAY_TIME;
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_join_req                                    */
/*                                                                           */
/*  Description   : This function prepares the MLME join request.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the join request buffer                    */
/*                  2) Pointer to the BSS descriptor set of BSS to join      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given join request buffer is updated with the BSS    */
/*                  parameters that is given as input.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_join_req(join_req_t *join_req, bss_dscr_t *bss_dscr)
{
	TROUT_FUNC_ENTER;
    mem_set(join_req, 0, sizeof(join_req_t));

    /* Set the fields of the join request according to the given BSS         */
    /* descriptor                                                            */
    memcpy(&join_req->bss_dscr, bss_dscr, sizeof(bss_dscr_t));
    memcpy(g_join_sta_addr, bss_dscr->sa, 6);

    join_req->join_timeout = get_join_start_timeout();
    join_req->probe_delay  = PROBE_DELAY_TIME;
	
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
	join_req->bss_dscr.p2p_manag = bss_dscr->p2p_manag;
#endif /* MAC_P2P */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_start_req                                   */
/*                                                                           */
/*  Description   : This function prepares the MLME start request.           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start request buffer                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given start request buffer is updated with           */
/*                  parameters from MIB.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_start_req(start_req_t *start_req)
{
    /* Set the time out value */
    start_req->start_timeout = get_join_start_timeout();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_auth_req                                    */
/*                                                                           */
/*  Description   : This function prepares the MLME authentication request.  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the auth request buffer                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given authentication request buffer is updated with  */
/*                  parameters from MIB. Note that this is an overhead since */
/*                  the auth request contains only setting of the MIB which  */
/*                  has already been set. Currently this function is empty   */
/*                  and provides a framework for future enhancements.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_auth_req(auth_req_t *auth_req)
{
    /* Set the timeout value */
    auth_req->auth_timeout = mget_AuthenticationResponseTimeOut();
    //TROUT_DBG4("[lym] prepare_mlme_auth_req auth_timeout=%d\n", auth_req->auth_timeout);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_asoc_req                                    */
/*                                                                           */
/*  Description   : This function prepares the MLME association request.     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the asoc request buffer                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given association request buffer is updated with     */
/*                  parameters from MIB. Note that this is an overhead since */
/*                  the auth request contains only setting of the MIB which  */
/*                  has already been set. Currently this function is empty   */
/*                  and provides a framework for future enhancements.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_asoc_req(asoc_req_t *asoc_req)
{
    /* Set the timeout value */
    asoc_req->assoc_timeout = mget_AssociationResponseTimeOut();
    //TROUT_DBG4("[lym] prepare_mlme_asoc_req assoc_timeout=%d\n", asoc_req->assoc_timeout);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_scan_match                                         */
/*                                                                           */
/*  Description   : This function checks the scan list in the scan response  */
/*                  for the specified scan criteria                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the scan response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches the specified field in the scan   */
/*                  list in the scan response and returns the index of the   */
/*                  matching BSS.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Index of the matching BSS descriptor. A value of         */
/*                  MAX_NUM_DESCRIPTOR indicates no matching BSS is found    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 check_scan_match(scan_rsp_t *scan_rsp)
{
    UWORD8     i             = 0;
    UWORD8     num_dscr      = 0;
    UWORD8     matched_dscr  = MAX_NUM_DESCRIPTOR;
    WORD8      last_rssi     = (WORD8)-100;
    BOOL_T     last_ssid_matched = BFALSE;
    bss_dscr_t *bss_dscr_set = 0;

	TROUT_FUNC_ENTER;
    bss_dscr_set = scan_rsp->bss_dscr;
    num_dscr     = scan_rsp->num_dscr;
#ifdef MAC_P2P  //wxb add
	if (BTRUE == mget_p2p_enable())
	{
		if (BTRUE == is_p2p_grp_form_in_prog())
		{
			TROUT_DBG4("P2P scan, return from check_scan_match\n");
			return matched_dscr;
		}
	}
#endif /* ifdef MAC_P2P */

	TROUT_DBG4("%s: scaned ap num: %d, cur_ssid: %s\n", __FUNCTION__, num_dscr, mget_DesiredSSID());
	//chenq add for cur ssid is NULL will return
	if( strlen(mget_DesiredSSID()) == 0 )
	{
		PRINTD("cur ssid is null,so not scan,return!\n");
		return matched_dscr;
	}
	
    /* Search for the BSS descriptor in the list with matching SSID          */
    /* and capabilities.                                                     */
    for(i = 0; i < num_dscr; i++)
    {
        BOOL_T check_for_ssid    = BTRUE;
        BOOL_T curr_ssid_matched = BFALSE;

		TROUT_DBG4("index: %d, ssid: %s\n", i, bss_dscr_set[i].ssid);
		TROUT_DBG6("will %s check ssid\n",get_bcst_ssid() ? "not" : "");
		
        /* Determine whether SSID has to be checked now or not */
        if((strlen(bss_dscr_set[i].ssid) == 0) ||
           (strlen(mget_DesiredSSID())   == 0) ||
           (get_bcst_ssid() == 1))

        {
            check_for_ssid = BFALSE;
        }

        /* Check the SSID of the network if it is to be done */
        if(check_for_ssid == BTRUE)
        {
            /* Ignore the network if SSID is not matching */
            if(strcmp(bss_dscr_set[i].ssid, mget_DesiredSSID()) != 0)
            {
                continue;
            }

            /* Else set the matched variable to TRUE */
            curr_ssid_matched = BTRUE;
        }

        /* Ignore the network if MAC or PHY capabilities did not match */
        if((check_bss_capability_mac(&bss_dscr_set[i]) == BFALSE) ||
           (check_bss_capability_phy(bss_dscr_set[i].cap_info) == BFALSE))
        {
			TROUT_DBG4("MAC or PHY capabilities did not match!\n");
            continue;
        }

        /* If Preffered BSSID is set, check whether there is any     */
        /* network with matching BSSID, if not return the descriptor */
        /* with all other capabilities matched except BSSID          */
        if(memcmp(bss_dscr_set[i].bssid, g_prefered_bssid, 6) == 0)
        {
            matched_dscr = i;
            break;
        }

        /* Give preferrence to this network if it's SSID is matching */
        /* while the last found network was an SSID-hiding network   */
        /* irrespective of the signal strength of both the networks  */
        if((curr_ssid_matched == BTRUE) &&
           (last_ssid_matched == BFALSE))
        {
            last_rssi          = bss_dscr_set[i].rssi;
            last_ssid_matched  = curr_ssid_matched;
            matched_dscr       = i;
            continue;
        }
        /* Do not give preference to this SSID hiding network if we */
        /* already have found a SSID matching network               */
        /* irrespective of the signal strength of both the networks  */
        else if((curr_ssid_matched == BFALSE) &&
                (last_ssid_matched == BTRUE))
        {
            continue;
        }

        #if 0 //chenq mod 20120903
        /* Give preference to this network if it has a better signal strength */
        if(last_rssi < ((WORD8)bss_dscr_set[i].rssi))
        {
            last_rssi          = bss_dscr_set[i].rssi;
            last_ssid_matched  = curr_ssid_matched;
            matched_dscr       = i;
        }
		#else
		else if((curr_ssid_matched == BTRUE) &&
                (last_ssid_matched == BTRUE))
		{
			last_rssi          = bss_dscr_set[i].rssi;
            last_ssid_matched  = curr_ssid_matched;
            matched_dscr       = i;
		}
		#endif
    }

    /* In case there was no matching BSS in the list, MAX_NUM_DESCRIPTOR     */
    /*  will be returned, which indicates no match is found.                 */
    if(matched_dscr  != MAX_NUM_DESCRIPTOR)
    {
        if(strlen(bss_dscr_set[matched_dscr].ssid) != 0)
        {
            mset_DesiredSSID(bss_dscr_set[matched_dscr].ssid);
        }

        g_last_network_index = matched_dscr;
        TROUT_DBG4("%s: matched_index = %d\n", __FUNCTION__, matched_dscr);
    }
	else
		TROUT_DBG4("%s: no matched ap found!\n", __FUNCTION__);

	TROUT_FUNC_EXIT;
    return matched_dscr;
}

//chenq add a func for check current connect ap some info changed 2013-06-08
BOOL_T check_cur_link_ap_info_change(bss_dscr_t * bss_dscr)
{
	TROUT_FUNC_ENTER;

	if(g_keep_connection == BFALSE)
	{
		TROUT_DBG5("%s: cur wifi status is not enable,return!\n",__FUNCTION__);
		return BTRUE;
	}

	if(memcmp(bss_dscr->bssid, mget_bssid(), 6) != 0)
	{
		TROUT_DBG5("%s: bssid not match,return!\n",__FUNCTION__);
		return BTRUE;
	}

	/* check ssid changed */
	//add by chenjun for CR 185042
	if(strlen(bss_dscr->ssid) > 0 )
	{
		if( strcmp(bss_dscr->ssid,mget_DesiredSSID()) != 0 )
		{
			TROUT_DBG4("%s: ssid is changed!\n",__FUNCTION__);
			TROUT_DBG4("%s: ssid len is %d \n ",__FUNCTION__,strlen(bss_dscr->ssid));
			return BFALSE;
		}
    }
	else
	{
		TROUT_DBG4("%s: ssid is NULL! len is %d \n",__FUNCTION__,strlen(bss_dscr->ssid));
	}
	//end by chenjun for CR 185042
	/* check channel */
	#if 0
	if( bss_dscr->channel != mget_CurrentChannel() )
	{
		TROUT_DBG4("%s: channel is changed!\n",__FUNCTION__);
		TROUT_DBG4("bss_dscr->channel = %d,mget_CurrentChannel() = %d\n",
			bss_dscr->channel,mget_CurrentChannel());
		return BFALSE;
	}
	#endif

    /* check MAC and PHY capabilities */
    if(check_bss_capability_mac_change(bss_dscr) == BFALSE)
    {
		TROUT_DBG4("%s: MAC capabilities is changed!\n",__FUNCTION__);
        return BFALSE;
	}

	if(check_bss_capability_phy( bss_dscr->cap_info ) == BFALSE)
    {
		TROUT_DBG4("%s: PHY capabilities is changed!\n",__FUNCTION__);
        return BFALSE;
    }

	TROUT_FUNC_EXIT;
    return BTRUE;
}
//chenq add end

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_scan_response                                    */
/*                                                                           */
/*  Description   : This function prepares the scan response structure       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function copies the scanned network details from    */
/*                  global structure to the mem-alloced scan rsp sctrut      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void prepare_scan_response(scan_rsp_t *scan_rsp)
{
	TROUT_FUNC_ENTER;
    scan_rsp->bss_dscr    = g_bss_dscr_set;
    scan_rsp->result_code = (g_bss_dscr_set_index > 0) ? SUCCESS_MLMESTATUS:
                                                         TIMEOUT;
    scan_rsp->num_dscr    = MIN(g_bss_dscr_set_index, MAX_NUM_DESCRIPTOR);

    p2p_update_scan_resp(scan_rsp);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : obss_scan_to_fn                                       */
/*                                                                           */
/*  Description      : This functions posts an event to start an OBSS scan   */
/*                                                                           */
/*  Inputs           : data pointer                                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions posts an evnet in the MISC queue       */
/*                     to start an periodic OBSS scan process                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void obss_scan_to_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void obss_scan_to_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* Create a MISCELLANEOUS event to start an OBSS scan  */
    misc_event_msg_t *misc = 0;
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc != NULL)
    {
        misc->data = NULL;
        misc->name = MISC_OBSS_TO;
        post_event((UWORD8*)misc, MISC_EVENT_QID);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_obss_scan_timer                                    */
/*                                                                           */
/*  Description   : This function starts the OBSS scan timer                 */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the OBSS scan period from the related */
/*                  prot function, and if it is a valid value, then starts   */
/*                  the timer. If the timer was NULL, it creates the timer   */
/*                  before starting                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_obss_scan_timer(void)
{
    g_obss_scan_period = get_obss_scan_period_prot(g_obss_scan_period);

    if(g_obss_scan_period > 0)
    {
        if(g_obss_scan_timer == NULL)
        {
            g_obss_scan_timer = create_alarm(obss_scan_to_fn, 0, NULL);
        }
        else
        {
            stop_alarm(g_obss_scan_timer);
        }

        start_alarm(g_obss_scan_timer, g_obss_scan_period);
    }
	// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: [%x] : start_obss_scan_timer: ScanTimer=%x  ScanPeriod=%d\n\r",
            get_machw_tsf_timer_lo(), g_obss_scan_timer, g_obss_scan_period);
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : stop_obss_scan_timer                                     */
/*                                                                           */
/*  Description   : This function stops the OBSS scan timer                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the OBSS timer is created already*/
/*                  and if so stops the same.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void stop_obss_scan_timer(void)
{
    if(g_obss_scan_timer != NULL)
    {
        stop_alarm(g_obss_scan_timer);
		// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: stop_obss_scan_timer: ScanTimer Stopped \n\r");
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_obss_scan                                          */
/*                                                                           */
/*  Description   : This function initiates the obss scan procedure          */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function suspends all the data qs, calls the rqured */
/*                  protocol obss scan functions, sends a Doze-NULL frame to */
/*                  the connected AP and then initiates a scan               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

//chenq add 2012-10-20
extern BOOL_T is_all_machw_q_null(void);
extern BOOL_T is_indecate_machw_q_null(int q_num);

extern unsigned int txhw_idle(void);  
extern int send_null_frame_to_AP_trick(UWORD8 psm, BOOL_T is_qos, UWORD8 priority);  
   
int start_obss_scan_prepare(void)  
{  
       unsigned int cnt = 0, v = 0, tnr = 0;  
       UWORD32 *tmp = NULL;  
 
       TROUT_DBG4("start_obss_scan 2 \n");  
       netif_stop_queue(g_mac_dev);  
#ifdef MAC_WMM  
       if(get_wmm_enabled() == BTRUE){  
           force_suspend_softtxq_above(get_txq_num(0));  
       }else{  
           force_suspend_softtxq_above(NORMAL_PRI_Q);  
       }  
#else  
       force_suspend_softtxq_above(NORMAL_PRI_Q);  
#endif  
		/*junbinwang modify 20131027*/
       //while(!txhw_idle()){  
       while(!is_all_machw_q_null()){
           msleep(10);  
           cnt++;  
           if(cnt > 15)  
              goto fail;  
       }  
      
#ifdef MAC_WMM  
       if(get_wmm_enabled() == BTRUE){  
              force_suspend_softtxq_above(AC_BK_Q);  
       }else{  
              force_suspend_softtxq_above(NORMAL_PRI_Q);  
       }  
#else  
       force_suspend_softtxq_above(NORMAL_PRI_Q);  
#endif  
       cnt = 0;  
       start_obss_scan_prot();  
       TROUT_DBG4("start_obss_scan 3 \n");  
       /* Remember that this scan is from a connected mode */  
       g_keep_connection = BTRUE;  
retry:  
       v = send_null_frame_to_AP_trick(STA_DOZE, BTRUE, 0);  
       if(!v && null_frame_dscr != NULL) {  
              tmp = null_frame_dscr;  
              tnr = 0;  
wait:  
              msleep(20);  
              if(null_frame_dscr == tmp){  
                   tnr++;  
                   if(tnr < 50)  
                       goto wait;  
                   printk("@@@: %s no tx_complete is for at least 1s\n", __func__);  
              }  
              if((UWORD32)null_frame_dscr == 0x1){  
                   cnt++;  
                   printk("@@@: %s SUSPEND SEND NULL try %d times\n", cnt, __func__);  
                   if(cnt < 5)  
                       goto retry;  
              }  
              pr_info("%s null frame done null_frame_dscr = %08X\n", __func__, null_frame_dscr);  
         }else{  
              printk("@@@: %s SUSPEND no memory for NULL\n", __func__);  
              goto fail;  
       }  
 
       netif_wake_queue(g_mac_dev);  
       enable_all_txq();  
       TROUT_DBG4("start_obss_scan 4 \n");  
       return 0;  
fail:  
       netif_wake_queue(g_mac_dev);  
       enable_all_txq();  
       return 1;  
}  

void end_obss_scan_prepare(void)  
{  
       unsigned int tnr = 0, cnt = 0, v = 0;  
       UWORD32 *tmp = NULL;  
   
       enable_all_txq();  
retry:  
       v = send_null_frame_to_AP_trick(STA_ACTIVE, BTRUE, 0);  
       if(!v && null_frame_dscr != NULL) {  
            tmp = null_frame_dscr;  
            tnr = 0;  
wait:  
            msleep(20);  
            if(null_frame_dscr == tmp){  
                tnr++;  
                if(tnr < 20)  
                    goto wait;  
                printk("@@@: RESUME -at least 400ms no TX ISR for NULL frame\n");  
            }  
            if((UWORD32)null_frame_dscr == 0x1){  
                cnt++;  
                printk("@@@:RESUME-SEND NULL try %d times\n", cnt);  
                if(cnt < 3)  
                   goto retry;  
            }  
            printk("@@@: %s send null frame done null_frame_dscr = %08x!\n", __func__, null_frame_dscr);  
       }else{  
            printk("@@@: %s RESUME no memory for NULL or other reason\n", __func__);  
       }  
}  



void start_obss_scan(void)
{
	struct trout_private *tp = netdev_priv(g_mac_dev);

    TROUT_DBG4("start_obss_scan 1 \n");

	if(reset_mac_trylock() == 0)
	{
		TROUT_DBG4("%s: WiFi is under reseting, please try again!\n", __func__);
                return;
	}
    
#ifdef WAKE_LOW_POWER_POLICY
	//avoid doing enter_low_power_mode() when start obss scan!
	if(!mutex_trylock(&tp->ps_mutex))
	{
		reset_mac_unlock();
		printk("get ps_mutex fail when obss scan start, skip!\n");
		return 0;
	}
	stop_alarm(g_flow_detect_timer);
	exit_low_power_mode(BFALSE);
	
#endif
	if(start_obss_scan_prepare())
	{

#ifdef WAKE_LOW_POWER_POLICY		
		mutex_unlock(&tp->ps_mutex);
#endif	
		reset_mac_unlock();	
		return;
	}
    TROUT_DBG4("start_obss_scan 5 \n");

    /* Go for scanning */
    initiate_scan(&g_mac);
#ifdef WAKE_LOW_POWER_POLICY
    mutex_unlock(&tp->ps_mutex);
#endif    
	reset_mac_unlock();
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : end_obss_scan                                            */
/*                                                                           */
/*  Description   : This function handles the end of an OBSS scan            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the channel back to home channel      */
/*                  informs the AP about coming back, calls the required     */
/*                  protocol functions, resumes the suspended queues         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void end_obss_scan(mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
    /* enable the required filters of data mode */
    enable_machw_obss_mgmt_filter();
#ifndef NON_FC_MACHW_SUPPORT
    enable_machw_non_directed_mgmt_filter();
#endif /* NON_FC_MACHW_SUPPORT */

    /* Any protocol related tasks after stopping the scan */
    end_obss_scan_prot();

    /* come back to home channel */
    select_channel_rf(mget_CurrentChannel(), get_bss_sec_chan_offset());

    /* Indicate to AP that sta is back */
    end_obss_scan_prepare(); 
    //send_null_frame_to_AP(STA_ACTIVE, BFALSE, 0);

    /* Any protocol related tasks after stopping the scan */
    send_obss_scan_report_prot();

    /* Reset the scan related parameters  */
	//chenq move under set_mac_state(ENABLED);
    //g_keep_connection    = BFALSE;
    set_site_survey(SITE_SURVEY_OFF);

    /* Restart the OBSS Scan timer for next scan */
    if(g_scan_source & OBSS_PERIODIC_SCAN)
    {
        start_obss_scan_timer();
    }

    g_scan_source = DEFAULT_SCAN;

    /* Change the MAC state to data mode */
    set_mac_state(ENABLED);

	//chenq add
	g_keep_connection    = BFALSE;
    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();

    /* Resume the data transmission */
 	//chenq mod 
    //resume_all_data_qs();
	resume_all_data_qs_not_resume_hw();

	//chenq add 2012-12-29 when scan end,load qmu
	trout_load_qmu();
	
#ifdef WAKE_LOW_POWER_POLICY
	printk("after obss scan, restart low power timer again!\n");
	clear_history_flow_record();
	restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
#endif
	
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_start_scan_req_sta                                   */
/*                                                                           */
/*  Description   : This function initiates the scan procedure               */
/*                                                                           */
/*  Inputs        : Value set by user                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if user is initiating the scan from */
/*                  connected state, if so goes for a scan with out resetting*/
/*                  the system, else sets the appropriate variable and resets*/
/*                  the system so that a fresh scan is done                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_start_scan_req_sta(UWORD8 scan_source)
{
	TROUT_FUNC_ENTER;
// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
/*
#define DEFAULT_SCAN        0
#define USER_SCAN           BIT0
#define OBSS_PERIODIC_SCAN  BIT1
#define OBSS_ONETIME_SCAN   BIT2
*/
    PRINTD("SwDb: set_start_scan_req_sta: ScanSource=%d  \n\r", scan_source);
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
    g_scan_source |= scan_source;
	/* If MAC-reset is not in progress then process the start scan request   */
	/* else this function is being called as part of WID restore during MAC  */
	/* reset                                                                 */
	if(g_reset_mac_in_progress != BTRUE)
	{

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	    if(!wake_lock_active(&scan_ap_lock)){
	    	wake_lock(&scan_ap_lock);
	        printk("@@@: acquire scan_ap_lock in %s\n", __func__);
        }
#endif
#endif
	    /* Connected State: Start the OBSS Scan */
	    if(get_mac_state() == ENABLED)
	    {
	        start_obss_scan();
	    }
	    /* WAIT_SCAN State: Already Scan in progress, restart the scan */
	    else if (get_mac_state() == WAIT_SCAN)
	    {
	        /* restart the scanning */
	        initiate_scan(&g_mac);
	    }
	    else
	    {
			/* Indicate scan is requested */
			if(g_scan_source != DEFAULT_SCAN)
				g_start_scan_req = BTRUE;
	    }
       }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initiate_scan_procedure                                  */
/*                                                                           */
/*  Description   : This function initiates the scan procedure               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if there are networks from previous */
/*                  scan available to join, if so joins, else re-scans       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initiate_scan_procedure(mac_struct_t *mac)
{
	//chenq add 2012-10-12
	itm_scan_flag = 0; 

	/* Go for re-scanning if the scan was requested by host    */
    /* Go for re-scanning if last scan didnt find any networks */
    /* Go for re-scanning if it is an independent (IBSS) mode  */

	TROUT_FUNC_ENTER;
    if((USER_SCAN   &  g_scan_source)         ||
       (0           == g_bss_dscr_set_index)  ||
       (INDEPENDENT == mget_DesiredBSSType()))
    {
        initiate_scan(mac);
    }
    /* See if any network is available from last scan */
    else
    {
        UWORD8     bss_index = 0;
        scan_rsp_t scan_rsp  = {0};

        prepare_scan_response(&scan_rsp);

       //20130621 chenjun modify for after resume connect ap  
       // bss_index = check_scan_match(&scan_rsp);
        bss_index = MAX_NUM_DESCRIPTOR;
       //end by chenjun for after resume connetct ap 

        /* Network found, initiate the join process */
        if((scan_rsp.result_code == SUCCESS_MLMESTATUS) &&
           (bss_index < MAX_NUM_DESCRIPTOR))
        {
            set_mac_state(SCAN_COMP);
			// 20120709 caisf add, merged ittiam mac v1.2 code
            update_non_p2p_join_prot();
            initiate_join(mac, &(scan_rsp.bss_dscr[bss_index]));
        }
        /* No matching network found, go for re-scanning */
        else
        {
            initiate_scan(mac);
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initiate_scan                                            */
/*                                                                           */
/*  Description   : This function initiates the scan process                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the scan request message and      */
/*                  calls the required MAC API to start the scan process     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initiate_scan(mac_struct_t *mac)
{
    scan_req_t *scan_req = NULL;

	TROUT_FUNC_ENTER;
	
	/*
	 * leon liu added, alloc scan_req,
	 * since scan_req_t is too large which may corrupt kernel stack
	 */
	scan_req = kmalloc(sizeof(scan_req_t), GFP_KERNEL);

	if (scan_req == NULL){
		TROUT_DBG2("No free memory for scan request\n");
		return;
	}
	//chenq add 0730
	//itm_scan_flag = 1;
	
	//zhangzhao add 2013-01-12
	if( g_scan_source & USER_SCAN 
		|| g_scan_source & OBSS_PERIODIC_SCAN
		||   g_scan_source & OBSS_ONETIME_SCAN)
		init_scan_limit();
	else
	{
#ifdef MAC_P2P //wxb add
		if (BTRUE == mget_p2p_enable())
		{
			if(BTRUE == is_p2p_grp_form_in_prog())
			{
				init_scan_limit(); //p2p scan will loop forever till p2p is closed.
			}
		}
#endif /* ifdef MAC_P2P */
		inc_scan_limit();
	}

	/*junbinwang mask this for bug 225695*/
	/*if(g_default_scan_limit > 1)
	{
		TROUT_DBG4("send_deauth_frame when auth failed \n");
		send_deauth_frame(g_prefered_bssid, (UWORD16)UNSPEC_REASON);
	}*/
	 
	//chenq add 2013-01-12
	if(scan_need_goon() == 0)
	{
        // add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        send_mac_status(MAC_CONNECT_FAILED);
        // end add by Ke.Li at 2013-04-04 for fix refresh UI ap list
		mset_DesiredSSID("");

		//chenq add 2013-10-10
		memset(g_prefered_bssid,0,6);

        //chenq add 2013-05-23
        set_mac_state(WAIT_SCAN);
		//add end
		kfree(scan_req);
		return;
	}
		
		
	prepare_mlme_scan_req(scan_req);
	mlme_scan_req(mac, (UWORD8*)scan_req);

	/*leon liu added, free scan_req.channel_list*/
	kfree(scan_req);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initiate_start                                           */
/*                                                                           */
/*  Description   : This function initiates the start process                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the start request message and     */
/*                  calls the required MAC API to start the start process    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initiate_start(mac_struct_t *mac)
{
    start_req_t start_req = {{0,},};
	TROUT_FUNC_ENTER;
    prepare_mlme_start_req(&start_req);

    mlme_start_req(mac, (UWORD8*)&start_req);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_start_scan_req_sta                                */
/*                                                                           */
/*  Description   : This function handles any user/protocol scan request     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_scan_source                                            */
/*                                                                           */
/*  Processing    : This function handles any user/protocol scan request     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_start_scan_req_sta(void)
{
	TROUT_FUNC_ENTER;

	if((g_start_scan_req == BTRUE) && (g_scan_source != DEFAULT_SCAN))
	{
		/* Intiate scan by restarting the MAC */
		restart_mac(&g_mac,0);

        /* Start a Rejoin timer if required after restarting MAC */
        start_rejoin_timer();
	
		//zhangzhao add 2013-01-12
		init_scan_limit();

        g_start_scan_req = BFALSE;
	}
	TROUT_FUNC_EXIT;
}

#endif /* IBSS_BSS_STATION_MODE */
