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
/*  File Name         : wlan_misc_sta.c                                      */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      MISC events.                                         */
/*                                                                           */
/*  List of Functions : sta_wait_scan_misc                                   */
/*                      sta_wait_join_misc                                   */
/*                      sta_wait_start_misc                                  */
/*                      sta_wait_auth_misc                                   */
/*                      sta_wait_asoc_misc                                   */
/*                      sta_enabled_misc                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_sta.h"
#include "cglobals_sta.h"
#include "sta_prot_if.h"
#include "controller_mode_if.h"
#include "mac_init.h"
#include "metrics.h"
#include "qmu_if.h"
#include "pm_sta.h"
#include "iconfig.h"
#include "../../CSL/csl_linux.h"
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include "ps_timer.h"
#endif

//chenq mode auto rate policy 2013-07-24
#include "autorate.h"
#include "management_sta.h"

/*****************************************************************************/
/* Static Function Declaration                                               */
/*****************************************************************************/

static void process_tx_comp_sta(UWORD8 q_num, UWORD8 num_dscr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_scan_misc                                       */
/*                                                                           */
/*  Description   : This function handles the miscellaneous event input as   */
/*                  appropriate in the WAIT_SCAN state.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle the            */
/*                  miscellaneous event input. Any required state change is  */
/*                  done by the core function.                               */
/*                                                                           */
/*  Outputs       : MAC state of the input MAC library structure may change  */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_scan_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

	TROUT_FUNC_ENTER;
	
    switch(misc_event_msg->name)
    {
    case MISC_TIMEOUT:
    {
        SWORD8  chan2scan = -1;

        /* Return if the current MAC state does not match the state saved in */
        /* the miscellaneous timeout event                                   */
        if(get_mac_state() != misc_event_msg->info)
        {
		pr_info("%s: mac_state(%d) and msg->info(%d) not equal\n"
				, __func__, get_mac_state(), misc_event_msg->info);
		itm_scan_flag = 0;
		#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
		pstimer_start(&pstimer);
		#endif

			TROUT_FUNC_EXIT;
            return;
		}
        chan2scan = handle_scan_itr_to(mac);
        if(chan2scan >= 0)
            scan_channel(chan2scan);
    }
    break;

    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        process_tx_comp_sta(q_num, num_dscr);
    }
    break;

    case MISC_ERROR:
    {
            raise_system_error(SYSTEM_ERROR);
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(msg_name) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_join_misc                                       */
/*                                                                           */
/*  Description   : This function handles the miscellaneous event input as   */
/*                  appropriate in the WAIT_JOIN state.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle the            */
/*                  miscellaneous event input. Any required state change is  */
/*                  done by the core function.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_join_misc(mac_struct_t *mac, UWORD8 *msg)
{
    join_rsp_t       join_rsp        = {0};
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

	TROUT_FUNC_ENTER;
    switch(misc_event_msg->name)
    {
    case MISC_TBTT:
    {
        /* Reception of TBTT indicates that Join was successful. Cancel the  */
        /* Join timeout timer and prepare a join response message with       */
        /* SUCCESS status and send it to the host.                           */
        cancel_mgmt_timeout_timer();

        join_rsp.result_code = SUCCESS_MLMESTATUS;

        /* The MAC state is changed to Enabled or Join Complete based on the */
        /* mode of operation.                                                */
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            sta_entry_t *se = 0;

            /* STATION table needs to be added/updated */
            /* Find the entry in the STA Table */
            se = find_entry(g_join_sta_addr);

            /* If sta index is 0, an entry dsn't exist, new entry is added */
            /* else the value of the sta index is refreshed from the table */
            if(se == NULL)
            {
                /* Malloc the memory for the new handle */
                se = (sta_entry_t *)mem_alloc(g_local_mem_handle,
                                              sizeof(sta_entry_t));
                if(se != NULL)
                {

                   mem_set(se, 0, sizeof(sta_entry_t));

                   /* Add the entry in the STA Table */
                   add_entry(se, g_join_sta_addr);

                   se->sta_index = get_new_sta_index(g_join_sta_addr);

                   if(se->sta_index == INVALID_ID)
                   {
                       /* Should never come here */
                       delete_entry(g_join_sta_addr);
                       se = NULL;
                   }
                   else
                   {
                       /* Initialize prot-table entry */
                       init_prot_handle_sta(se);
                   }
                }
            }

            /* Initialize the security related FSMs */
            /* se==NULL check is being done inside this function */
            if(init_sec_entry_sta(se, g_join_sta_addr) != BTRUE)
            {
                /* The MAC state is changed to Auth complete state. Further */
                /* MLME requests are processed in this state.               */
                set_mac_state(DISABLED);

                join_rsp.result_code = TIMEOUT;
            }
            else
            {
                set_mac_state(ENABLED);

                if(is_wps_prot_enabled() == BFALSE)
                {
                    /* Convey the current MAC status to Host */
					//chenq mask
                    //send_mac_status(MAC_CONNECTED);
                }

                disable_machw_beacon_filter();

                /* Kick start the seucrity FSM */
                start_sec_fsm_sta(se);

                /* Initialize the table for autorate */
                init_sta_entry(se, 0, 0, 0);

                /* Start the autorate alarm */
                start_ar_timer();

                /* Start the CPU utiltization test */
                start_cpu_util_test();

                /* Enable Long NAV */
                //enable_machw_long_nav_support();

            }
        }
        else
        {
            set_mac_state(JOIN_COMP);
            TROUT_DBG5("%s: set state to 6(JOIN_COMP)\n", __FUNCTION__);
        }

        /* Send the response to host now. */
        send_mlme_rsp_to_host(mac, MLME_JOIN_RSP, (UWORD8 *)(&join_rsp));
    }
    break;

    case MISC_TIMEOUT:
    {
        /* Return if the current MAC state does not match the state saved in */
        /* the miscellaneous timeout event                                   */
        if(get_mac_state() != misc_event_msg->info)
        {
			TROUT_FUNC_EXIT;
            return;
        }
        /* Timeout indicates that Join was not successful. Prepare a join    */
        /* response message with TIMEOUT status and send it to the host.     */
        join_rsp.result_code = TIMEOUT;

        /* The MAC state is changed to Disabled state. Further MLME requests */
        /* are processed in this state.                                      */
        set_mac_state(DISABLED);

        /* Send the response to host now. */
        handle_mlme_rsp_sta(mac, MLME_JOIN_RSP, (UWORD8 *)(&join_rsp));
    }
    break;

    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        process_tx_comp_sta(q_num, num_dscr);
    }
    break;

    case MISC_ERROR:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(msg_name) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_auth_misc                                       */
/*                                                                           */
/*  Description   : This function handles the miscellaneous event input as   */
/*                  appropriate in the WAIT_AUTH_SEQ2/4 state.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle the            */
/*                  miscellaneous event input. Any required state change is  */
/*                  done by the core function.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_auth_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_TIMEOUT:
    {
        auth_rsp_t auth_rsp = {{0,},};

        /* Return if the current MAC state does not match the state saved in */
        /* the miscellaneous timeout event                                   */
        if(get_mac_state() != misc_event_msg->info)
            return;

        /* Prepare an authentication response message with TIMEOUT status    */
        /* and send it to the host.                                          */
        auth_rsp.result_code = TIMEOUT;

        /* The MAC state is changed to Join complete state. Further MLME     */
        /* requests are processed in this state.                             */
        set_mac_state(JOIN_COMP);
	//	printk("==%s: JOIN_COMP==\n", __FUNCTION__);
        /* Send the response to host now. */
        handle_mlme_rsp_sta(mac, MLME_AUTH_RSP, (UWORD8 *)(&auth_rsp));
    }
    break;

    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        process_tx_comp_sta(q_num, num_dscr);
    }
    break;

    case MISC_ERROR:
    {
            raise_system_error(SYSTEM_ERROR);
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(msg_name) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_asoc_misc                                       */
/*                                                                           */
/*  Description   : This function handles the hardware input as appropriate  */
/*                  in the WAIT_ASOC state.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                  and state change may happen based on the function output.*/
/*                                                                           */
/*  Outputs       : MAC state of the input MAC library structure may change  */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_asoc_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_TIMEOUT:
    {
	if(get_mac_state() == ENABLED)
		return;
        asoc_rsp_t asoc_rsp = {0};
        /* Return if the current MAC state does not match the state saved in */
        /* the miscellaneous timeout event                                   */
        if(get_mac_state() != misc_event_msg->info)
            return;

        /* Prepare an authentication response message with TIMEOUT status    */
        /* and send it to the host.                                          */
        asoc_rsp.result_code = TIMEOUT;
	// re-call sta_wait_auth again until g_assoc_retry_cnt >= 5
	if(g_assoc_retry_cnt < ASSOC_MAX_RETRY_CNT){
		set_mac_state(AUTH_COMP);		
	}
	else{
        /* Send de-auth frame with error code, Previous Authentication no    */
        /* longer valid                                                      */
        send_deauth_frame(mget_bssid(), (UWORD16)AUTH_NOT_VALID);

        /* The MAC state is changed to Auth complete state. Further MLME     */
        /* requests are processed in this state.                             */
        set_mac_state(DISABLED);

        /* Reset the STA Entry */
        delete_entry(mget_bssid());
	}
        /* Send the response to host now. */
        handle_mlme_rsp_sta(mac, MLME_ASOC_RSP, (UWORD8 *)(&asoc_rsp));
    }
    break;

    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        process_tx_comp_sta(q_num, num_dscr);
    }
    break;

    case MISC_ERROR:
    {
            raise_system_error(SYSTEM_ERROR);
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(msg_name) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_start_misc                                      */
/*                                                                           */
/*  Description   : This function handles the hardware input as appropriate  */
/*                  in the WAIT_START state.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                  and state change may happen based on the function output.*/
/*                                                                           */
/*  Outputs       : MAC state of the input MAC library structure may change  */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_start_misc(mac_struct_t *mac, UWORD8 *msg)
{
    start_rsp_t      start_rsp       = {0};
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_TBTT:
    {
        sta_entry_t *se = 0;

        /* Since the station is starting the network, the join address can */
        /* be updated with the station address                             */
        memcpy(g_join_sta_addr, mget_StationID(), 6);

        /* STATION table needs to be added/updated */
        /* Find the entry in the STA Table */
        se = find_entry(g_join_sta_addr);

        /* If sta index is zero, an entry dsn't exist, new entry is added */
        /* else the value of the sta index is refreshed from the table    */
        if(se == NULL)
        {
            /* Malloc the memory for the new handle */
            se = (sta_entry_t *)mem_alloc(g_local_mem_handle,
                                          sizeof(sta_entry_t));
            if(se != NULL)
            {

                mem_set(se, 0, sizeof(sta_entry_t));

                /* Add the entry in the STA Table */
                add_entry(se, g_join_sta_addr);

                se->sta_index = get_new_sta_index(g_join_sta_addr);

                if(se->sta_index == INVALID_ID)
                {
                    /* Should never come here */
                    delete_entry(g_join_sta_addr);
                    se = NULL;
                }
                else
                {
                    /* Initialize prot-table entry */
                    init_prot_handle_sta(se);
                }
            }
        }

        if(se != NULL)
        {

            /* Reception of TBTT indicates that Start was successful. Cancel the */
            /* Start timeout timer and prepare a start response message with     */
            /* SUCCESS status and send it to the host.                           */
            cancel_mgmt_timeout_timer();

            start_rsp.result_code = SUCCESS_MLMESTATUS;

            /* The MAC state is changed to Enabled (note that mode must be IBSS) */
            set_mac_state(ENABLED);

            if(is_wps_prot_enabled() == BFALSE)
            {
                /* Convey the current MAC status to Host */
				//chenq mask	
                //send_mac_status(MAC_CONNECTED);
            }

            disable_machw_beacon_filter();

            /* Initialize the table for autorate */
            init_sta_entry(se, 0, 0, 0);

            /* Start the autorate alarm */
            start_ar_timer();

            /* Enable Long NAV */
            //enable_machw_long_nav_support();

        }
        else
        {
               /* The MAC state is changed to Disabled (note that mode must be IBSS) */
                set_mac_state(DISABLED);
        }
        /* Send the response to host now. */
        send_mlme_rsp_to_host(mac, MLME_START_RSP, (UWORD8 *)(&start_rsp));

        /* Start the CPU utiltization test */
        start_cpu_util_test();
    }
    break;

    case MISC_TIMEOUT:
    {
        /* Return if the current MAC state does not match the state saved in */
        /* the miscellaneous timeout event                                   */
        if(get_mac_state() != misc_event_msg->info)
            return;
        /* Timeout indicates that Start was not successful. Prepare a start  */
        /* response message with TIMEOUT status and send it to the host.     */
        start_rsp.result_code = TIMEOUT;

        /* The MAC state is changed to Disabled state. Further MLME requests */
        /* are processed in this state.                                      */
        set_mac_state(DISABLED);

        /* Send the response to host now. */
        send_mlme_rsp_to_host(mac, MLME_START_RSP, (UWORD8 *)(&start_rsp));
    }
    break;

    case MISC_ERROR:
    {
            raise_system_error(SYSTEM_ERROR);
    }
    break;

    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        process_tx_comp_sta(q_num, num_dscr);
    }
    break;

    default:
    {
        /* Do Nothing */
    }
    break;

    } /* end of switch(msg_name) */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_misc                                         */
/*                                                                           */
/*  Description   : This function handles hardware input in ENABLED state.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : g_link_loss_count                                        */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
extern struct net_device_stats *g_mac_net_stats;
extern WORD32 get_avg_rssi(void);
extern UWORD8  g_ap_last_rate_integer;
extern UWORD8  g_ap_last_rate_decimal;
extern UWORD32 g_cmcc_set_aci;

extern void MxdRfPulse(WORD32 RSSI,WORD32 rate1,WORD32 rate2);

void sta_enabled_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

	//chenq add 2013-02-15
	static unsigned long tx_packets = 0;
	static unsigned long rx_packets = 0;

	TROUT_FUNC_ENTER;

    switch(misc_event_msg->name)
    {
    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *base_dscr = misc_event_msg->data;
        UWORD8 *dscr      = 0;
        UWORD8 q_num      = get_tx_dscr_q_num((UWORD32*)base_dscr);
        BOOL_T sync_found = BFALSE;

		misc_tx_comp_count++;
        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by first         */
        /* processing all the packets till the given frame.                  */
        while(sync_found == BFALSE)
        {
            dscr = get_txq_head_pointer(q_num);
            if(dscr == NULL)
            {
				TROUT_FUNC_EXIT;
                return;
			}
            if(base_dscr == dscr)
            {
                process_tx_comp_sta(q_num, num_dscr);
                sync_found = BTRUE;
            }
            else
            {
                process_tx_comp_sta(q_num, 1);
            }

        }
    }
    break;

    case MISC_TBTT:
    {
    	misc_tbtt_count++;
        /* Update the reference timer of the station */
        update_uptime_cnt();

        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            /* Increment the link loss counter. Compare with the threshold   */
            /* for link loss. Once it crosses, reset MAC and rescan.         */
            /* The link loss counter will be reset every time a beacon is    */
            /* received. In case filter is disabled, no link loss detection  */
            /* is done.                                                      */
            if(is_bcn_filter_on() == BFALSE)
            {
#ifdef TX_ABORT_FEATURE
                if(is_machw_tx_aborted() == BFALSE)
                    g_link_loss_count++;
                else
                    g_link_loss_count = 0;
#else /* TX_ABORT_FEATURE */
                g_link_loss_count++;
                //printk("tbtt: link_lost = %d\n", g_link_loss_count);
#endif /* TX_ABORT_FEATURE */

                /* Spectrum management functions to be done at every TBTT */
                handle_tbtt_spec_mgmt_sta();

                /* Other protocol functions to be done at TBTT */
                handle_tbtt_prot_sta();

                if((get_link_loss_threshold() != 255) &&
                   (g_link_loss_count > get_link_loss_threshold()))
                {
                    if(is_machw_enabled() == BFALSE)
                    {
						TROUT_DBG3("%s: link losss occur, connect will break!\n", __func__);
                        raise_system_error(LINK_LOSS_PA);
#ifdef DEBUG_MODE
                        g_reset_stats.linklosspacnt++;
#endif /* DEBUG_MODE */
                    }
                    else if(BTRUE == is_machw_rx_buff_null(NORMAL_PRI_RXQ))
                    {
						TROUT_DBG3("%s: link losss occur, normal rx buff is empty!\n", __func__);
                        raise_system_error(LINK_LOSS_BUFF);
#ifdef DEBUG_MODE
                        g_reset_stats.linklossbuffcnt++;
#endif /* DEBUG_MODE */
                    }
                    else
                    {
                    	TROUT_DBG3("%s: send link loss error! linkloss_threshold=%d\n", __func__, get_link_loss_threshold());
                        raise_system_error(LINK_LOSS);
#ifdef DEBUG_MODE
                        g_reset_stats.linklosscnt++;
#endif /* DEBUG_MODE */
                    }
                }
            }
            /* Handle the TBTT event for STA mode */
            handle_tbtt_sta();

			#if 1
			//chenq add for BusyTraffic 2013-02-12
			if( (get_uptime_cnt() % 20) == 0 )//20 * TBTT(100ms) == 2000ms == 2s
			{
                                int tx_packets_check = 0;
                                int rx_packets_check = 0;

                          /*Begin of modified by yiming.li, cd bug 475*/
                                //check tx packets, overflow is no need to change
					tx_packets_check = g_mac_net_stats->tx_packets - tx_packets;
					tx_packets = g_mac_net_stats->tx_packets;
                                if(tx_packets_check < 0) tx_packets_check = (-1)*tx_packets_check;


                                //check rx packets, overflow is no need to change
					rx_packets_check = g_mac_net_stats->rx_packets - rx_packets;
					rx_packets = g_mac_net_stats->rx_packets;
                                if(rx_packets_check < 0) rx_packets_check = (-1)*rx_packets_check;
                                /*End of modified by yiming.li, cd bug 475*/
				
				if( (rx_packets_check > 100) || (tx_packets_check > 100) )
				{
					g_BusyTraffic = BTRUE;

					#if 0
					if(g_UI_cmd_ps == BTRUE)
					{
						//go to Maximum_PSPOLL_Power_Save_Mode
						set_PowerManagementMode(4);
					}
					else
					{
						//go to Active_Mode
						set_PowerManagementMode(0);
					}
					#endif
				}
				else
				{
					g_BusyTraffic = BFALSE;

					#if 0
					//go to Maximum_PSPOLL_Power_Save_Mode
					set_PowerManagementMode(4);
					#endif
				}
			}
			#endif
			// for cmcc test, set aci [zhongli wang 20130911]
			if(1 == g_cmcc_set_aci)
			{
				static WORD32 cal_100ms_time = 0;
				WORD32 tmp_rssi = 0;

				cal_100ms_time++;
				if(cal_100ms_time == 2)
				{
					tmp_rssi = get_avg_rssi();
					if(tmp_rssi == 277){
						printk("avg rssi invalid\n");
					}
					else{
						MxdRfPulse(tmp_rssi,g_ap_last_rate_integer,g_ap_last_rate_decimal);
					}
					cal_100ms_time = 0;
				}
			}			
			//for cmcc test, set aci [zhongli wang 20130911] end
        }
        else
        {
            UWORD8  old_bcn_idx = 0;
            UWORD32 *trout_beacon_buf = 0, beacon_len = 0;

            /* Increment the link loss counter. Compare with the threshold   */
            /* for link loss. Once it crosses, reset RSSI Values             */
            if(is_bcn_filter_on() == BFALSE)
            {
                g_link_loss_count++;

                if((get_link_loss_threshold() != 255) &&
                   (g_link_loss_count > get_link_loss_threshold()))
                {
                     g_rssi = -100;
                     g_link_loss_count = 0;
                }
            }

            /* Get the index which is currently being used by H/w */
            old_bcn_idx  = (g_beacon_index + 1)%2;

            /* Update the beacon with latest protocol parameters */
            g_beacon_len = prepare_beacon_sta(g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH);

    		//add by chengwg.
    		beacon_len = g_beacon_len;
    		if(g_beacon_len % 4 != 0)
    			beacon_len += 4 - (g_beacon_len % 4);
    		if(beacon_len > BEACON_MEM_SIZE)
	    	{
				printk("%s: beacon mem is too small(%d : %d)!\n", __func__, beacon_len, BEACON_MEM_SIZE);
				return;
	    	}	
    		trout_beacon_buf = (UWORD32 *)BEACON_MEM_BEGIN;
    		host_write_trout_ram((void *)trout_beacon_buf, g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);

            /* Swap the beacon frame pointer in the MAC H/w */
            set_machw_beacon_pointer((UWORD32)trout_beacon_buf);

            /* S/w will now make updates if required in the old beacon */
            g_beacon_index = old_bcn_idx;

#ifdef IBSS_11I
            handle_tbtt_sec_event();
#endif /* IBSS_11I */
        }
        
//        sprd_nstime_get(&time);
//		etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
//		misc_tbtt_time += etime - stime;
    }
    break;

    case MISC_ERROR:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;

    case MISC_OBSS_TO:
    {
        set_start_scan_req_sta(OBSS_PERIODIC_SCAN);
    }
    break;

#ifdef AUTORATE_FEATURE
    case MISC_SW_AR_CNTL:
    {
        void *te = misc_event_msg->data;
        do_per_entry_ar(te);
    }
    break;
#endif /* AUTORATE_FEATURE */

    default:
    {
        /* Check for sta protocol misc events */
        misc_sta_enabled_prot_if(mac,msg);
    }
    break;

    } /* end of switch(msg_name) */
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_scan_itr_comp                                    */
/*                                                                           */
/*  Description   : This function handles the processing when an iteration of*/
/*                  scan is completed.                                       */
/*                                                                           */
/*  Inputs        : None                      */
/*                                                                           */
/*  Globals       : g_link_loss_count                                        */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void process_scan_itr_comp(mac_struct_t *mac)
{
    scan_rsp_t scan_rsp = {0};
    BOOL_T     status       = BFALSE;

	TROUT_FUNC_ENTER;
    /* Call WPS function to process scan complete event */
    status = process_wps_scan_complete((UWORD8 *)mac);

    /* If processing has been completed successfully, no further processing  */
    /* is required to be done. Return. Otherwise do normal processing.       */
    if(BTRUE == status)
    {
		TROUT_FUNC_EXIT;
        return;
	}
    prepare_scan_response(&scan_rsp);

    set_mac_state(SCAN_COMP);

    /* Send the response to host now. The response is freed in the   */
    /* function send_mlme_rsp_to_host.                               */
    send_mlme_rsp_to_host(mac, MLME_SCAN_RSP, (UWORD8 *)(&scan_rsp));
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_tx_comp_sta                                      */
/*                                                                           */
/*  Description   : This function processes frames from the TX-Q after TX    */
/*                  Complete Interrupt is raised.                            */
/*                                                                           */
/*  Inputs        : 1) Q-Number of the Queue to be processed                 */
/*                  2) Number of Descriptors to be processed                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : TX Complete interrupt is processed by reading out the    */
/*                  descriptors from the TX-Q one at a time.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_tx_comp_sta(UWORD8 q_num, UWORD8 num_dscr)
{
    UWORD8 i      = 0;
    UWORD8 *dscr  = 0;
    UWORD8 ra[6]  = {0};
    UWORD8 *msa   = NULL;
    void   *entry = NULL;

	TROUT_FUNC_ENTER;
    /* Process and remove the given number of frames in order from the   */
    /* head of the transmit queue.                                       */
    for(i = 0; i < num_dscr; i++)
    {
        /* Get the transmit queue head pointer */
        dscr = get_txq_head_pointer(q_num);

        /* If the queue has become empty, this is an exception. Return */
        if(dscr == NULL)
        {
			TROUT_FUNC_EXIT;
            return;
		}
        /* Extract MSA */
		// 20120709 caisf add, merged ittiam mac v1.2 code
        msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);
		
        if(i == 0)
        {

#ifdef ENABLE_MACHW_KLUDGE
            /* Workaround for MAC H/w issue which can cause the num_dscr  */
            /* count to go wrong when a Normal-MPDU frame TX following an */
            /* AMPDU fails.                                               */
            if((num_dscr > 1) && (is_tx_success((UWORD32 *)dscr) == BFALSE))
            {
                if((get_tx_dscr_short_retry_count((UWORD32 *)dscr) >= mget_ShortRetryLimit())
                   && (get_tx_dscr_cts_failure((UWORD32 *)dscr) >= mget_ShortRetryLimit()))
                {
                    num_dscr = 1;
                }
            }
#endif /* ENABLE_MACHW_KLUDGE */

            /* The Station entry is extracted for the transmitted frame. */
            /* Since all MPDUs of an AMPDU have the same RA, this can be */
            /* done for only once.                                       */
			// 20120709 caisf masked, merged ittiam mac v1.2 code
            //msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);
            get_address1(msa, ra);
            entry = find_entry(ra);

            /* Update the MIB counters for this MPDU */
            update_tx_mib(dscr, num_dscr, entry);

            /* Update required MIB for the aggregation of frames transmitted */
            update_aggr_tx_mib(num_dscr);

			//Begin:delete by wulei 2791 for bug 155500 on 2013-04-24
#if 0
	        /* Reset the link loss count on each sucessful tx in Infra STA mode  */
	        if(mget_DesiredBSSType() == INFRASTRUCTURE)
	        {
	            UWORD8 ack_policy = get_tx_dscr_ack_policy((UWORD32*)dscr);

	            /* If the frame has been transmitted to AP successfully */
				/* update the condiction to clear g_link_loss_count by zhao */
	            if(((ack_policy == NORMAL_ACK) || (ack_policy == COMP_BAR)) &&
	               (get_tx_dscr_status((UWORD32*)dscr) != TX_TIMEOUT))// &&
	               //(get_tx_dscr_frag_status_summary((UWORD32*)dscr) == 0xFFFFFFFF))
	            {
	                g_link_loss_count = 0;
	                //printk("txcomp: link_lost = %d\n", g_link_loss_count);
	            }
	        }
#endif
			//End:delete by wulei 2791 for bug 155500 on 2013-04-24
        }

        print_log_debug_level_1("\n[DL1][INFO][Tx] {Transmit complete event}");

        if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE))
        {
            /* Update protocol table at tx complete */
			// 20120709 caisf mod, merged ittiam mac v1.2 code
            //handle_prot_tx_comp_sta(dscr, entry);
			handle_prot_tx_comp_sta(dscr, entry, msa);
        	
            /* Power Management */
            handle_tx_complete_sta((UWORD32 *)dscr);
        }

        /* Call the latency test function */
        latency_test_function(dscr);

#ifdef MEASURE_PROCESSING_DELAY
        g_delay_stats.numsubmsduontxcmp +=
                get_tx_dscr_num_submsdu((UWORD32*)dscr);
        g_delay_stats.numtxdscrontxcmp++;
#endif /* MEASURE_PROCESSING_DELAY */

        /* Delete the packet from the head of the transmit queue */
        if(qmu_del_tx_packet(&g_q_handle.tx_handle, q_num) != QMU_OK)
        {
            print_log_debug_level_1("\n[DL1][EXC][Tx] {Packet not deleted from queue}");
        }
        else
        {
            print_log_debug_level_1("\n[DL1][INFO][Tx] {Packet deleted from queue}");
        }

    }
    TROUT_FUNC_EXIT;
}
#endif /* IBSS_BSS_STATION_MODE */
