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
/*  File Name         : wlan_rx_mgmt_sta.c                                   */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      WLAN_RX (MGMT) events.                               */
/*                                                                           */
/*  List of Functions : sta_wait_scan_rx                                     */
/*                      sta_wait_join_rx                                     */
/*                      sta_wait_auth_seq2_rx                                */
/*                      sta_wait_auth_seq4_rx                                */
/*                      sta_wait_asoc_rx                                     */
/*                      sta_enabled_rx_mgmt                                  */
/*                      coalesce_if_required                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "sta_prot_if.h"
#include "mac_init.h"
#include "index_util.h"
#include "receive.h"
#include "metrics.h"
#include "core_mode_if.h"
#include "iconfig.h"
#include "autorate.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void coalesce_if_required(UWORD8 *msa, UWORD16 rx_len, UWORD8 *bssid);
#ifdef AUTORATE_PING
//ping.jiang add for calculating statistics 2013-12-12
WORD32  g_asoc_rssi_value = 0;
WORD32  g_asoc_rssi_num   = 0;

void asoc_rssi_value_add(void)
{
	WORD32 temp_rssi = 0;
	WORD32 mask_rssi = 0;
	
    	temp_rssi = get_rssi() - 0xFF;
		
    	if(temp_rssi <= -128)
    	{	
		mask_rssi = temp_rssi + 0xFF;
		printk("change rssi value %d => %d\n",temp_rssi,mask_rssi);
    	}
    	else
    	{
       		mask_rssi = temp_rssi;
    	}
	g_asoc_rssi_value += mask_rssi;
	g_asoc_rssi_num ++;
	return;
}


WORD32 get_asoc_avg_rssi(void)
{
	WORD32 cur_avg_rssi = 0;
	if(0 != g_asoc_rssi_num)
	{
		cur_avg_rssi = g_asoc_rssi_value / g_asoc_rssi_num;		
		g_asoc_rssi_value = 0;
		g_asoc_rssi_num   = 0;
	}
	
	return cur_avg_rssi;
}
//ping.jiang add for calculating statistics end
#endif /* AUTORATE_PING */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_scan_rx                                         */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_SCAN state.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Beacons and probe responses are processed to update the  */
/*                  global scan response structure with new BSS descriptors. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_scan_rx(mac_struct_t *mac, UWORD8 *msg)
{
    WORD8     rssi      = 0;
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD16   rx_len    = wlan_rx->rx_len;
    UWORD8    *msa      = wlan_rx->msa;
    UWORD32   *rx_dscr  = wlan_rx->base_dscr;
	//chenq add a flag "snr"
	WORD8     snr       = 0;
	UWORD8    ssid[MAX_SSID_LEN] = {0};
	TROUT_FUNC_ENTER;
    rssi    = get_rx_dscr_rssi_db(rx_dscr);
	snr     = get_rx_dscr_snr(rx_dscr);
	
    if(rssi == INVALID_RSSI_DB)
        rssi = (WORD8)-100;

    switch(wlan_rx->sub_type)
    {
    case BEACON: //libing modify for fix the scan ap buger:211215
    {
		/*junbinwang add this 20131027.if ssid is null, shouldn't handle it.*/
		get_ssid(msa, ssid);
		if(0 == strlen(ssid))/*junbingwang for combo scan.*/
			return;
	}
    case PROBE_RSP:
    {
#ifdef DEBUG_MODE
		if(wlan_rx->sub_type == PROBE_RSP)
			g_mac_stats.rcv_prob_rsp_count++;
		else if(wlan_rx->sub_type == BEACON)
			g_mac_stats.rcv_beacon_fram_count++;
#endif
		// 20120709 caisf add, merged ittiam mac v1.2 code
        if(BFALSE == is_frame_valid(msa))
        {
			TROUT_FUNC_EXIT;
            return;
		}
        /* Send the scan result to host */
        send_network_info_to_host(msa, rx_len, rssi);

        /* Process the scan response for protocol                       */
        /* Normal processing is done if protocol processing indicate so */
        if(BFALSE == handle_scan_resp_prot(mac, msa, rx_len, rssi,
                                           (TYPESUBTYPE_T)(wlan_rx->sub_type)))
        {
            /* Update a global scan response structure with the received BSS */
            /* descriptor set, if it has not been received already.          */
			//chenq add a flag "snr"
            update_scan_response(msa, rx_len, rssi,snr);
        }
    }
    break;
    case PROBE_REQ:
    {
        /* Process probe requests for other protocol */
        sta_wait_scan_handle_prb_req_prot(msa, rx_len, rssi);
    }
    break;
    case ACTION:
    {
        sta_wait_scan_handle_action_prot(mac, msa, rx_len, rssi);
    }
    break;
    default:
    {
        /* Do Nothing */
    }
    } /* end of switch(get_sub_type(msa)) */

    /* Check for P2P dev id match */
    p2p_check_dev_match(mac);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_join_rx                                         */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_JOIN state.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : In the infrastructure mode beacons are checked and the   */
/*                  DTIM count is extracted and updated in the MAC hardware  */
/*                  register for beacons from the same BSS. In any case once */
/*                  a beacon is received from the same BSS the hardware      */
/*                  registers are set to enable join function. This is to    */
/*                  ensure that the DTIM count in case of infrastructure     */
/*                  mode is adopted before hardware join is complete (TBTT   */
/*                  interrupt is received).                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_join_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD16   rx_len    = wlan_rx->rx_len;
    UWORD8    *msa      = wlan_rx->msa;

	TROUT_FUNC_ENTER;
    switch(get_sub_type(msa))
    {
        case BEACON:
        {
            UWORD8 bssid[6] = {0};

            get_BSSID(msa, bssid);

            /* The DTIM count must be adopted only if the BSSID matches */
            if(mac_addr_cmp(bssid, mget_bssid()) == BTRUE)
            {
                if(mget_DesiredBSSType() == INFRASTRUCTURE)
                {
                    UWORD8 *tim_elm = get_tim_elm(msa, rx_len, TAG_PARAM_OFFSET);

                    /* Extract the DTIM count from the beacon and update the MAC */
                    /* H/w register with the value.                              */
                    if(tim_elm != 0)
                    {
                        UWORD8 dtim_cnt = get_dtim_count(tim_elm);
                        mset_DTIMPeriod(tim_elm[3]);
                        set_machw_dtim_period(tim_elm[3]);
                        set_machw_ap_dtim_cnt(dtim_cnt);
                    }
                     
                    //chenq add 2013-08-22
                    update_rssi(wlan_rx->base_dscr);
#ifdef AUTORATE_PING
		     //ping.jiang add for calculating statistics 2013-12-12
                  asoc_rssi_value_add();
#endif /* AUTORATE_PING */
					// 20120830 caisf mod, merged ittiam mac v1.3 code
					#if 0
                    /* Update 11h related elements */
                    update_11h_elemets(msa, rx_len, TAG_PARAM_OFFSET);
					#else
                    /* Update station protocol related elements */
                    update_joinrx_prot_sta_elemets(msa, rx_len, TAG_PARAM_OFFSET);
					#endif

                }

                /* Write required Hardware registers (BSSID, enable              */
                /* synchronization) and wait for TBTT                            */
                set_machw_bssid(mget_bssid());
                set_machw_tsf_join();
            }
        }
        break;

        case ACTION:
        {
            sta_wait_join_handle_action_prot(mac, msa, rx_len);
        }

        default:
        {
            /* Do Nothing */
        }
    } /* end of switch(get_sub_type(msa)) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_auth_seq2_rx                                    */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_AUTH_SEQ2 state.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Authentication frames are processed in this state and    */
/*                  the MAC state is changed appropriately based on this.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_auth_seq2_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8    *msa      = wlan_rx->msa;

	TROUT_FUNC_ENTER;
#ifdef AUTORATE_PING
    //ping.jiang add for calculating statistics 2013-12-12
    update_rssi(wlan_rx->base_dscr);
    asoc_rssi_value_add();
#endif /* AUTORATE_PING */
    switch(get_sub_type(msa))
    {
    case AUTH:
    {
        if((get_auth_seq_num(msa) == 0x0002) &&
           (get_auth_status(msa) == SUCCESSFUL_STATUSCODE))
        {
            UWORD8 auth_type = mget_auth_type();

            if(auth_type & BIT0)
            {
                auth_type = SHARED_KEY;
            }
            else
            {
                auth_type = OPEN_SYSTEM;
            }
#ifdef INT_WPS_SUPP
            if(BTRUE == is_wps_prot_enabled())
            {
                auth_type = OPEN_SYSTEM;
            }
#endif /* INT_WPS_SUPP */
            /* If an Authentication Sequence 2 frame with SUCCESS status is  */
            /* received, cancel the authentication timeout timer             */
            cancel_mgmt_timeout_timer();

            if(auth_type == OPEN_SYSTEM)
            {
                auth_rsp_t auth_rsp = {{0,},};

                /* Prepare the authentication response structure and send it */
                /* to the host. Change state to AUTH_COMP                    */
                auth_rsp.auth_type   = OPEN_SYSTEM;
                auth_rsp.result_code = SUCCESS_MLMESTATUS;

                set_mac_state(AUTH_COMP);

                /* Send the response to host now. The response is freed in   */
                /* the function send_mlme_rsp_to_host.                       */
                send_mlme_rsp_to_host(mac, MLME_AUTH_RSP, (UWORD8 *)(&auth_rsp));
            }
            else if(auth_type == SHARED_KEY)
            {
                UWORD16 auth_frame_len = 0;
                UWORD8  *auth_frame    = 0;

                /* Prepare the Authnetication Frame <Sequence 3> and add it  */
                /* to the H/w queue with required transmit descriptor.       */
                auth_frame = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,
                                                 MANAGEMENT_FRAME_LEN);
                if(auth_frame == NULL)
                {
                    /* Error condition: Reset the whole MAC */
                    raise_system_error(NO_LOCAL_MEM);
                    TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
					g_mac_stats.no_mem_count++;
#endif                    
                    return;
                }

                auth_frame_len = prepare_auth_req_seq3(auth_frame, msa);

                /* Transmit the management frame */
                tx_mgmt_frame(auth_frame, auth_frame_len, HIGH_PRI_Q, 0);

                /* Change state to WAIT_AUTH_SEQ4 and start auth timeout timer */
                set_mac_state(WAIT_AUTH_SEQ4);
                start_mgmt_timeout_timer(mget_AuthenticationResponseTimeOut());
		   //TROUT_DBG4("[lym] sta_wait_auth_seq2_rx auth_timeout=%d\n", mget_AuthenticationResponseTimeOut());
            }
        }
		//chenq add for share key
		else if((get_auth_seq_num(msa) == 0x0002) &&
           (get_auth_status(msa) == UNSUPT_ALG))
		{
			mset_auth_type(1);
		}
		else	
        {
            /* Do nothing. Wait for a response till timeout occurs. */
        }
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;
    }  /* end of switch(get_msg_name(msa)) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_auth_seq4_rx                                    */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_AUTH_SEQ4 state.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Authentication frames are processed in this state and    */
/*                  the MAC state is changed appropriately based on this.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_auth_seq4_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8    *msa      = wlan_rx->msa;

	TROUT_FUNC_ENTER;
#ifdef AUTORATE_PING
    //ping.jiang add for calculating statistics 2013-12-12
    update_rssi(wlan_rx->base_dscr);
    asoc_rssi_value_add();	
#endif /* AUTORATE_PING */
    switch(get_sub_type(msa))
    {
    case AUTH:
    {
        if((get_auth_seq_num(msa) == 0x0004) &&
           (get_auth_status(msa) == SUCCESSFUL_STATUSCODE))
        {
            auth_rsp_t auth_rsp = {{0,},};

            /* If an Authentication Sequence 4 frame with SUCCESS status is  */
            /* received, cancel the authentication timeout timer             */
            cancel_mgmt_timeout_timer();

            /* Prepare the authentication response structure and send it to  */
            /* the host. Change state to AUTH_COMP                           */
            auth_rsp.auth_type   = SHARED_KEY;
            auth_rsp.result_code = SUCCESS_MLMESTATUS;

            set_mac_state(AUTH_COMP);

            /* Send the response to host now. */
            send_mlme_rsp_to_host(mac, MLME_AUTH_RSP, (UWORD8 *)(&auth_rsp));
        }
        else
        {
            /* Do nothing. Wait for a response till timeout occurs. */
        }
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;
    } /* end of switch(get_msg_name(msa)) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_asoc_rx                                         */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_ASOC state.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Association Response frames are processed in this state  */
/*                  and the MAC state is changed appropriately based on this.*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_asoc_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8    sub_type  = 0;
    UWORD16   rx_len    = wlan_rx->rx_len;
    UWORD8    *msa      = wlan_rx->msa;

	TROUT_FUNC_ENTER;
#ifdef AUTORATE_PING
    //ping.jiang add for calculating statistics 2013-12-12
    update_rssi(wlan_rx->base_dscr);
    asoc_rssi_value_add();
#endif /* AUTORATE_PING */
    switch(sub_type = get_sub_type(msa))
    {
    case ASSOC_RSP:
    {
        if(get_asoc_status(msa) == SUCCESSFUL_STATUSCODE)
        {
            sta_entry_t *se      = 0;
            asoc_rsp_t  asoc_rsp = {0};
            UWORD8      sa[6]    = {0};

            get_SA(msa, sa);
            se    = (sta_entry_t *)find_entry(sa);
            if(se == NULL)
            {
                /* Error condition: Reset the whole MAC */
                raise_system_error(NO_LOCAL_MEM);
                TROUT_FUNC_EXIT;
                return;
            }

            /* If an Association response frame with SUCCESS status is       */
            /* received, cancel the association timeout timer                */
            cancel_mgmt_timeout_timer();

            /* Update required globals with the association response fields  */
            g_asoc_id = get_asoc_id(msa);

            /* Update power saving once associated based on protocol in use */
            update_prot_power_save();

            /* Store the last association response information */
            /* This is to be sent to external supplicant(host)*/
            mem_set(g_assoc_res_info, 0xFF, MAX_STRING_LEN);
			//chenq mod 2012-11-05
            //g_assoc_res_info[0] = MIN(MAX_STRING_LEN - 1,(rx_len - MAC_HDR_LEN - FCS_LEN));
            //memcpy(g_assoc_res_info + 1, msa + MAC_HDR_LEN, g_assoc_res_info[0]);
            g_assoc_res_info[0] = MIN(MAX_STRING_LEN - 1,(rx_len - MAC_HDR_LEN - 6 - FCS_LEN));
            memcpy(g_assoc_res_info + 1, msa + MAC_HDR_LEN + 6, g_assoc_res_info[0]);

            /* Prepare the association response structure and send it to     */
            /* the host. Change state to ENABLED                             */
            asoc_rsp.result_code = SUCCESS_MLMESTATUS;

            /* Initialize the security related FSMs */
            if(init_sec_entry_sta(se, sa) != BTRUE)
            {
                /* Some serious error has occured */
                send_deauth_frame(sa, (UWORD16)AUTH_NOT_VALID);

                delete_entry(sa);

                /* The MAC state is changed to Auth complete state. Further */
                /* MLME requests are processed in this state.               */
                set_mac_state(DISABLED);

                asoc_rsp.result_code = INVALID_MLMESTATUS;
            }
            else
            {
                UWORD16 offset = 0;

                set_mac_state(ENABLED);

                /* Kick start the seucrity FSM */
                start_sec_fsm_sta(se);

                /* Initialize the offset to the information elements */
                offset = MAC_HDR_LEN + CAP_INFO_LEN +  STATUS_CODE_LEN + AID_LEN;

                /* Update the QoS protocol table */
                update_qos_table_prot(se->sta_index, msa, rx_len, offset);

                /* Update QoS parameters */
                update_qos_params(msa, rx_len, offset, sub_type);

                /* Update the HT specific entries */
                update_sta_entry_prot(msa, offset, rx_len, se);

                /* Initialize autorate table for this station */
                init_sta_entry(se, msa, rx_len, offset);

                /* Start the autorate alarm */
                start_ar_timer();

                /* Disable MAC Hardware Beacon Filter */
                disable_machw_beacon_filter();

               /* Update the MAC H/w protection parameters based on the */
               /* newly adopted TX rate set.                            */
               set_default_machw_prot_params();
               if(g_short_preamble_enabled == BFALSE)
                   set_machw_prot_pream(1);
               else
                   set_machw_prot_pream(0);

				
                /* Unmask the deauth interrupt and enable the deauth filter */
                unmask_machw_deauth_int();
				
				/*xuan yang, 2013.6.8, disable the deauth filter to receive deauth frame*/
                //enable_machw_deauth_filter();
				disable_machw_deauth_filter();
				
                /* Incase of Security, further handshake   */
                /* is expected so dont start the ps timers */
                /* and do not send MAC status to host now  */
                /* These will be taken care later          */
                if((mget_RSNAEnabled() == TV_FALSE ) ||
                   (BFALSE == g_int_supp_enable))
                {
                    if(is_wps_prot_enabled() == BFALSE)
                    {
                        start_activity_timer();

                        /* Convey to Host that device is ready for Tx/Rx */
						//chenq mask
                        //send_mac_status(MAC_CONNECTED);

#ifdef NO_ACTION_RESET
                        /* Restore all the saved action requests */
                        restore_all_saved_action_req();
#endif /* NO_ACTION_RESET */
                    }
                }

                /* Initialize Coexistence Management Protocol */
                start_2040_coex_sta();

                /* Start the CPU utiltization test */
                start_cpu_util_test();
            }

            /* Handle MLME response */
            handle_mlme_rsp_sta(mac, MLME_ASOC_RSP, (UWORD8 *)(&asoc_rsp));

            /* Enable Radar detection */
            enable_11h_radar_detection();

            /* Enable Long NAV */
            //enable_machw_long_nav_support();
        }
        else
        {
            /* Do nothing. Wait for a response till timeout occurs. */
        }
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(get_sub_type(msa)) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_mgmt                                      */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the ENABLED state.                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : g_link_loss_count, g_rssi                                */
/*                                                                           */
/*  Processing    : The incoming frame type is checked and appropriate       */
/*                  processing is done based on mode of operation.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
extern void rssi_value_add(void);

void sta_enabled_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
    UWORD8    sta_index = 0;
    wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8    *msa      = wlan_rx->msa;
    UWORD16   rx_len    = wlan_rx->rx_len;
    UWORD32   *rx_dscr  = 0;

	TROUT_FUNC_ENTER;
	
    /* Pointer to the Rx-Descriptor. */
    rx_dscr = wlan_rx->base_dscr;

    switch(wlan_rx->sub_type)
    {
    case PROBE_REQ:
    {
        if((mget_DesiredBSSType() == INDEPENDENT) &&
           (probe_req_ssid_cmp(msa, mget_DesiredSSID()) == BTRUE))
        {
            if(get_machw_bcn_tx_stat() == BTRUE)
            {
                /* If the last beacon was sent by the STA, respond to the    */
                /* probe request.                                            */
                send_probe_rsp(msa, BFALSE);
            }
            else
            {
                /* If the received probe request is directed to the STA a    */
                /* probe response must be sent even if the STA did not send  */
                /* the last beacon                                           */
                if(mac_addr_cmp(wlan_rx->addr1, mget_StationID()) == BTRUE)
                {
                    send_probe_rsp(msa, BFALSE);
                }
            }
        }
    }
    break;

    case PROBE_RSP:
    {
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            /* Get the cap_info */
            UWORD16 cap_info = get_cap_info(msa);

            if((check_bss_capability_info(cap_info) == BFALSE) ||
               (check_bss_capability_phy(cap_info) == BFALSE))
            {
				TROUT_FUNC_EXIT;
                return;
            }

            /* Get the STA index for the STA sending probe response */
            sta_index = get_new_sta_index(msa + ADDR2_OFFSET);
            if(sta_index != INVALID_ID)
            {
                update_qos_table_prot(sta_index, msa, rx_len,
                                      TAG_PARAM_OFFSET);

            }
        }
        else
        {
            /* Do nothing */
        }
    }
    break;

    case BEACON:
    {
        UWORD8  start_freq   = get_current_start_freq();
        UWORD8  cur_chan_num = get_current_channel(msa, rx_len);
        UWORD16 index    = 0;
        UWORD16 cap_info = 0;
        BOOL_T  ibss_ht_check = BFALSE;

        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            if(mac_addr_cmp(wlan_rx->bssid, mget_bssid()) == BTRUE)
            {
                sta_entry_t *se    = 0;

                	/* Update rssi inforamation */
              	update_rssi(rx_dscr);
			//chenq add snr
			update_snr(rx_dscr);
							

                /*  Get preamble information   */
                cap_info = get_cap_info(msa);

                if(g_short_preamble_enabled != get_bss_short_preamble_info(cap_info))
                {
                    g_short_preamble_enabled = get_bss_short_preamble_info(cap_info);

                    if(g_short_preamble_enabled == BFALSE)
                        set_machw_prot_pream(1);
                    else
                        set_machw_prot_pream(0);
                }

                /* Find the entry in the STA Table */
                se = (sta_entry_t *)wlan_rx->sa_entry;

                /* If sta index is 0, no entry exists. A new entry is added. */
                /* Otherwise the value of the sta index is refreshed from    */
                /* the table                                                 */
                if(se == NULL)
                {
                    if(check_sec_capability_sta(msa, rx_len))
                    {
                        /* Delete older station entries from the association table */
                        check_free_sta_entry();

                        /* Malloc the memory for the new handle */
                        se = (sta_entry_t *)mem_alloc(g_local_mem_handle,
                                                      sizeof(sta_entry_t));
                        if(se == NULL)
                        {
                            /* If Station Entry could not be created,Leave it */
                            /* for now. When next beacon is received we will  */
                            /* try again and create the entry for this station*/
							TROUT_FUNC_EXIT;
                            return;
                        }
                        mem_set(se, 0, sizeof(sta_entry_t));

                        /* Add the entry in the STA Table */
                        add_entry(se, wlan_rx->ta);

                        se->sta_index = get_new_sta_index(wlan_rx->ta);

                        if(se->sta_index == INVALID_ID)
                        {
                            delete_entry(wlan_rx->ta);
                            break;
                        }
                        else
                        {
                            /* Initialize prot-table entry */
                            init_prot_handle_sta(se);
                            /* Initialize the security related FSMs */
                            if(init_sec_entry_sta(se, wlan_rx->ta) != BTRUE)
                            {
                                delete_entry(wlan_rx->ta);
                                break;
                            }

                            /* Kick start the seucrity FSM */
                            start_sec_fsm_sta(se);
                        }
                        sta_index = se->sta_index;
                    }
                    else
                    {
                        delete_entry(wlan_rx->ta);
                        break;
                    }
                }
                else
                {
                    sta_index = se->sta_index;
                }

                /* In case of 11e update the table with the capability info  */
                /* other stations                                            */
                update_qos_table_prot(sta_index, msa, rx_len,
                                      TAG_PARAM_OFFSET);

                ibss_ht_check = is_ht_capable(se);

                /* Update the HT specific entries */
                update_sta_entry_prot(msa, TAG_PARAM_OFFSET, rx_len, se);

                /* Update retry rate set according to HT capable settings of the STA */
                if(ibss_ht_check != is_ht_capable(se))
                {
                    update_entry_retry_rate_set((void *)se, get_phy_rate(get_tx_rate_sta(se)));

#ifdef NO_ACTION_RESET
                    /* Restore all the saved action requests */
                    restore_all_saved_action_req();
#endif /* NO_ACTION_RESET */
                }

                {
                    UWORD16 offset = TAG_PARAM_OFFSET;

                    /* Skip the SSID Element */
                    if(msa[offset] == 0)
                    {
                        offset += (IE_HDR_LEN + msa[offset + 1]);
                        /* Initialize autorate table for this station */
                        init_sta_entry(se, msa, rx_len, offset);
                    }
                    else
                    {
                        /* Initialize autorate table for this station */
                        init_sta_entry(se, 0, 0, 0);
                    }
                }

                /* Check for ERP protection field in the beacon */
                index = TAG_PARAM_OFFSET;
                while(index < (rx_len - FCS_LEN))
                {
                    if(msa[index] == IERPINFO)
                    {
                        /* Check the protection type. If the bit is set      */
                        /* indicating that protection should be used in the  */
                        /* network, the STA must start using protection (if  */
                        /* it is not) irrespective of current protection     */
                        /* setting. Otherwise protection is not being used   */
                        /* in the network and the STA need not change the    */
                        /* current protection type as set by the user.       */
                        if((msa[index + 2] & BIT1) &&
                           (get_protection() != ERP_PROT) &&
                           (is_autoprot_enabled() == BTRUE))
                        {
                            set_protection(ERP_PROT);
                            set_machw_prot_control();
                            update_beacon_erp_info();
                        }
                        break;
                    }
                    else if((msa[index] == ISUPRATES) &&
                            (get_current_start_freq() == RC_START_FREQ_2))
                    {
                        UWORD8 num_rates = msa[index + 1];

                        /* Check if the station is 11b station and enable ERP    */
                        /* protection.                                           */
                        if(num_rates <= NUM_BR_PHY_802_11G_11B_1)
                        {
                            if(is_autoprot_enabled() == BTRUE)
                            {
                                set_protection(ERP_PROT);
                                set_machw_prot_control();
                                update_beacon_erp_info();
                            }
                        }
                        index += num_rates + IE_HDR_LEN;
                    }
                    else
                    {
                        /* Increment index by length information & tag header */
                        index += msa[index + 1] + IE_HDR_LEN;
                    }
                }

                /* This function updates the connection counter for the */
                /* particular station.                                  */
                update_connection_cnt(se);
            }
            else
            {
                /* In IBSS mode if a beacon is received from a different     */
                /* network coalesce if required.                             */
                coalesce_if_required(msa, rx_len, wlan_rx->bssid);
            }
        }
        else /* INFRASTRUCTURE MODE */
        {
            /* Beacons from other BSS need not be processed, return */
            if(mac_addr_cmp(wlan_rx->bssid, mget_bssid()) == BFALSE)
            {
				TROUT_FUNC_EXIT;
                return;
            }

            /* The Link Loss related statistics are updated only when the */
            /* beacon is received in the current channel. This is to take */
            /* care of the condition where the AP might have shifted to a */
            /* neighbouring channel.                                      */
            if(get_ch_idx_from_num(start_freq, cur_chan_num) ==
               mget_CurrentChannel())
            {
                /* Reset the link loss count on receiving beacon from AP */
                g_link_loss_count = 0;
                //printk("rxmgmt: link_lost = %d\n", g_link_loss_count);

                /* Update the RSSI information */
              update_rssi(rx_dscr);

				//chenq the snr information
		update_snr(rx_dscr);
#ifdef AUTORATE_PING
		//ping.jiang add for calculating statistics 2013-10-31
		ar_rssi_value_add();
#endif /* AUTORATE_PING */
		#ifdef TROUT2_WIFI_IC
			uptate_rssi4_auto_set_tx_rx_power();
		#endif			
		}

            /*  Get preamble information   */
            cap_info = get_cap_info(msa);
			
			//xuan yang, 2013-6-28, check sec type
			if (BFALSE == check_sec_type_sta(msa, rx_len, cap_info))
			{
				/* Process frames from the AP with which the BSS STA is associated */
				if(mget_DesiredBSSType() == INFRASTRUCTURE)
				{
					UWORD8 null_addr[6] = {0};

					printk("drop link loss ... \n");
#ifdef DEBUG_MODE
					g_reset_stats.deauthcnt++;
#endif /* DEBUG_MODE */

					/* Reset the STA Entry */
					delete_entry(mget_bssid());

					/* Reset the BSS ID so that no more deauth is sent */
					mset_bssid(null_addr);

					/* process for any protocol */
					process_deauth_prot();

					/* Reset MAC by raising a system error */
					raise_system_error(DEAUTH_RCD);
				}
			}
            if(g_short_preamble_enabled != get_bss_short_preamble_info(cap_info))
            {
                g_short_preamble_enabled = get_bss_short_preamble_info(cap_info);

                if(g_short_preamble_enabled == BFALSE)
                    set_machw_prot_pream(1);
                else
                    set_machw_prot_pream(0);
            }

            /* Update the signal quality with the latest beacon information */
            {
                UWORD16 sig_qual = get_rx_dscr_signal_quality(rx_dscr);

                /* Only non-zero Signal Quality Numbers are valid */
                if(sig_qual > 0)
                    g_signal_quality = sig_qual;
            }


            /* Process the ERP information field */
            process_erp_info_sta(msa, rx_len);

            /* Process the HT information field */
            process_ht_info_sta(msa, rx_len, (sta_entry_t *)wlan_rx->sa_entry);

			// 20120830 caisf mod, merged ittiam mac v1.3 code
			#if 0
            /* Process 11h related info */
            update_11h_prot_info(msa, rx_len);
			#else
			/* Process protocol related info */
            update_enable_sta_prot_info(msa, rx_len);
			#endif

            /* Update QoS parameters */
            update_qos_params(msa, rx_len, TAG_PARAM_OFFSET, wlan_rx->sub_type);

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
            p2p_handle_noa(msa, rx_len);
#endif /* MAC_P2P */

            /* If the STA is in Power save mode, process the TIM element */
            if(mget_PowerManagementMode() == MIB_POWERSAVE)
            {
                UWORD8 *tim_elm = get_tim_elm(msa, rx_len, TAG_PARAM_OFFSET);

                if(tim_elm != 0)
                    psm_handle_tim_elm(tim_elm);
            }
        }
    }
    break;

    case DEAUTH:
    case DISASOC:
    {
        UWORD8  bssid[6] = {0};

        get_BSSID(msa, bssid);

        /* Process frames from the AP with which the BSS STA is associated */
        if((mget_DesiredBSSType() == INFRASTRUCTURE) &&
           (mac_addr_cmp(bssid, mget_bssid()) == BTRUE))
        {
            UWORD8 null_addr[6] = {0};

#ifdef DEBUG_MODE
            g_reset_stats.deauthcnt++;
#endif /* DEBUG_MODE */

            /* Reset the STA Entry */
            delete_entry(mget_bssid());

            /* Reset the BSS ID so that no more deauth is sent */
            mset_bssid(null_addr);

            /* process for any protocol */
            process_deauth_prot();

            /* Reset MAC by raising a system error */
            raise_system_error(DEAUTH_RCD);

        }
    }
    break;
    case ACTION:
    {
        wlan_rx_t *wlan_rx  = (wlan_rx_t *)msg;
        UWORD8    *msa      = wlan_rx->msa;
        UWORD16   rx_len    = wlan_rx->rx_len;
        UWORD32   *rx_dscr  = wlan_rx->base_dscr;

        /* Process ACTION frame for spectrum management */
        if((INFRASTRUCTURE == mget_DesiredBSSType()) &&
           (TV_TRUE == mget_SpectrumManagementImplemented()))
        {
            /* Handle Spectrum Management Action Frames */
            handle_spec_mgmt_action(msa, rx_len, MAC_HDR_LEN, rx_dscr);
        }

        /* Process other protocol action frames */
        handle_action_prot_sta(msg);
    }
    break;
    default:
    {
        /* None */
    }
    break;

    } /* end of switch(get_sub_type(msa)) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : coalesce_if_required                                     */
/*                                                                           */
/*  Description   : This function performs coalescing if required.           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the beacon                                 */
/*                  2) Length of the beacon                                  */
/*                  3) Extracted BSSID                                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if coalescing should be performed   */
/*                  based on capability information and timestamp. If        */
/*                  required the BSSID and beacon interval is updated.       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void coalesce_if_required(UWORD8 *msa, UWORD16 rx_len, UWORD8 *bssid)
{
    UWORD16 index    = 0;
    UWORD16 cap_info = 0;

    /* Coalescing cannot be done if the capability information does not match */
    cap_info = get_cap_info(msa);

    if((check_bss_capability_info(cap_info) == BFALSE) ||
       (check_bss_capability_phy(cap_info) == BFALSE))
    {
        return;
    }

    index = TAG_PARAM_OFFSET;

    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == ISSID)
        {
            UWORD8  *ssid      = 0;
            UWORD8  ssid_len   = 0;

            ssid     = &msa[index + 2];
            ssid_len = msa[index + 1];

            if((ssid_len == strlen(mget_DesiredSSID())) &&
               (memcmp(mget_DesiredSSID(), ssid, ssid_len) == 0))
            {
                UWORD32 tsf_hi     = 0;
                UWORD32 tsf_lo     = 0;
                UWORD32 bcn_tsf_hi = 0;
                UWORD32 bcn_tsf_lo = 0;

                /* Extract the timestamp from the received beacon */
                index = MAC_HDR_LEN;

                bcn_tsf_lo = *(UWORD32 *)(&msa[index]);
                bcn_tsf_hi = *(UWORD32 *)(&msa[index + 4]);

                /* Extract the MAC H/w TSF timer timestamp */
                get_machw_tsf_timer(&tsf_hi, &tsf_lo);

                /* Compare the timestamps. Coalescing is done only if the    */
                /* received timestamp is higher than the H/w TSF timestamp.  */
                if((bcn_tsf_hi > tsf_hi) ||
                   ((bcn_tsf_hi == tsf_hi) && (bcn_tsf_lo > tsf_lo)))
                {
                    UWORD8  *bcn  = 0;
                    UWORD16 value = 0;

                    /* Adopt the BSSID and beacon interval */
                    mset_BeaconPeriod(get_beacon_period(msa + TIME_STAMP_LEN +
                                                              MAC_HDR_LEN));
                    mset_bssid(bssid);

                    /* Update related H/w registers */
                    set_machw_beacon_period(mget_BeaconPeriod());
                    set_machw_bssid(bssid);

                    /* Update beacon frame with the new BSSID, beacon period */
                    bcn = g_beacon_frame[g_beacon_index];
                    set_address3(bcn, mget_bssid());
                    value = mget_BeaconPeriod();
                    bcn[MAC_HDR_LEN + TIME_STAMP_LEN]     = value & 0xFF;
                    bcn[MAC_HDR_LEN + TIME_STAMP_LEN + 1] = (value >> 8) & 0xFF;

#ifdef DEBUG_MODE
                    g_mac_stats.coalesce++;
#endif /* DEBUG_MODE */

                }
                else
                {
                    /* Do nothing. Coalescing is not required. */
                }
            }
            break;
        }
        else
        {
            /* Increment index by length information and tag element */
            index += msa[index + 1] + IE_HDR_LEN;
        }
    }
}

#endif /* IBSS_BSS_STATION_MODE */
