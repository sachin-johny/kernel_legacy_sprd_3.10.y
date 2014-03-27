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
/*  File Name         : fsm.c                                                */
/*                                                                           */
/*  Description       : This file contains the functions common to the MAC   */
/*                      FSM implementation for AP and STA modes.             */
/*                                                                           */
/*  List of Functions : misc_null_fn                                         */
/*                      wlan_rx_data_null_fn                                 */
/*                      wlan_tx_data_null_fn                                 */
/*                      set_mac_state                                        */
/*                      get_mac_state                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "fsm.h"
#include "imem_if.h"
#include "qmu_if.h"
#include "receive.h"
#include "transmit.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_null_fn                                             */
/*                                                                           */
/*  Description   : This function handles unexpected inputs in the FSM for   */
/*                  MISC_EVENT type of event.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to message                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : On TX_COMPLETE event, the Tx descriptor address is       */
/*                  received in message. Using this free the buffers used.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#include "iconfig.h"
#include "qmu_tx.h"

#ifdef IBSS_BSS_STATION_MODE //chenq add 0723
//chenq add a flag
extern BOOL_T  g_keep_connection;
#endif

void misc_null_fn(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_TX_COMP:
    {
        UWORD8 i        = 0;
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *dscr    = misc_event_msg->data;
        UWORD8 q_num    = get_tx_dscr_q_num((UWORD32*)dscr);

        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by deleting all  */
        /* the packets till the given frame.                                 */
        qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, dscr);

        /* Process and remove the given number of frames in order from the   */
        /* head of the transmit queue.                                       */
        /* Whenever a packet is not found indicating that the queue is empty */
        /* break and return.                                                 */
        for(i = 0; i < num_dscr; i++)
        {
            if(qmu_del_tx_packet(&g_q_handle.tx_handle, q_num) ==
               QMU_PACKET_NOT_FOUND)
            {
                printk(" QMU_PACKET_NOT_FOUND\n");
                return;
            }
        }
    }
    break;
    default:
    {
        /* Do nothing */
    }
    break;

    } /* end of switch(msg_name) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wlan_rx_data_null_fn                                     */
/*                                                                           */
/*  Description   : This function handles unexpected inputs in the FSM for   */
/*                  WLAN_RX_DATA_FRAME type.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to message                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function frees the buffer allocated for the frame   */
/*                  and the request structure.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wlan_rx_data_null_fn(mac_struct_t *mac, UWORD8 *msg)
{
#ifdef DEBUG_MODE
    g_mac_stats.wrdnull++;
#endif /* DEBUG_MODE */
    /* Do nothing. The buffer will be freed at the end of process_wlan_rx in */
    /* any case and the event will be freed in MAC controller.               */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wlan_tx_data_null_fn                                     */
/*                                                                           */
/*  Description   : This function handles unexpected inputs in the FSM for   */
/*                  WLAN_TX_DATA_FRAME type.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to message                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function frees the buffers allocated for the frame  */
/*                  and the request structure.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wlan_tx_data_null_fn(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_FUNC_ENTER;
#ifdef DEBUG_MODE
        g_mac_stats.wtdnull++;
#endif /* DEBUG_MODE */
	//caisf add 2013-02-15
    if(g_mac_net_stats)
        g_mac_net_stats->tx_dropped++;
    
	TX_PATH_DBG("%s: state=%d\n", __func__, get_mac_state());
    /* Free the memory allocated for the packet */
    pkt_mem_free(((wlan_tx_req_t *)msg)->buffer_addr);
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_mac_state                                            */
/*                                                                           */
/*  Description   : This function sets the MAC state to the new state.       */
/*                                                                           */
/*  Inputs        : 1) State to which MAC should be set                      */
/*                                                                           */
/*  Globals       : g_mac                                                    */
/*                  g_mgmt_alarm_handle                                      */
/*                                                                           */
/*  Processing    : This function checks if a management alarm is already    */
/*                  present. If so, the same is stopped and deleted. A new   */
/*                  management alarm is created with  mgmt_timeout_alarm_fn  */
/*                  as the callback function and the new state as the input  */
/*                  data for this callback function. This is to ensure that  */
/*                  any timer triggered in any MAC state is processed only   */
/*                  in the same state. The timeout event is no longer valid  */
/*                  in the other states. The global MAC structure state is   */
/*                  then updated.                                            */
/*                  Note that this requires the code to be developed         */
/*                  ensuring that the start_mgmt_timeout_timer function is   */
/*                  called appropriately only after the required MAC state   */
/*                  change is complete.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_mac_state(UWORD8 state)
{
	TROUT_DBG4("%s: set state from %d to %d\n", __FUNCTION__, g_mac.state, state);
#ifdef DEBUG_MODE
//	print_symbol(" -> %s\n", (unsigned long)__builtin_return_address(0));
#endif

    /* If the management alarm exists, stop and delete the same */
    if(g_mgmt_alarm_handle != 0)
    {
        stop_alarm(g_mgmt_alarm_handle);
        delete_alarm(&g_mgmt_alarm_handle);
    }

    /* Create the management alarm with the new MAC state as the management  */
    /* timeout function callback data                                        */
    g_mgmt_alarm_handle = create_alarm(mgmt_timeout_alarm_fn, state, NULL);

#ifdef IBSS_BSS_STATION_MODE	
	//chenq add send_mac_status for uper server
	if((g_mac.state == ENABLED) && (state != ENABLED) && (g_keep_connection == BFALSE))
	{
		///chenq mask 2013-01-12 
		//send_mac_status(MAC_DISCONNECTED);
		TROUT_DBG4("send disconnected status to UI!\n");
	}
	else if((g_mac.state == ENABLED) && (state == DISABLED))
	{
		//chenq mask 2013-01-12 
		//send_mac_status(MAC_DISCONNECTED);
		TROUT_DBG4("send disconnected status to UI!\n");
	}
	else if((state == DISABLED) && (g_keep_connection == BTRUE))
	{//chenq add 2012-11-01 
	    //chenq mask 2013-01-12 
		//send_mac_status(MAC_DISCONNECTED);
		TROUT_DBG4("send disconnected status to UI!\n");
	}
	else if((g_mac.state != ENABLED) && (state == ENABLED))
	{
		if(g_keep_connection == BFALSE)
		{
			TROUT_DBG4("send connected status to UI!\n");
			send_mac_status(MAC_CONNECTED);
		//else if(g_keep_connection == BTRUE)
		    //send_mac_status(MAC_SCAN_CMP);
		}	
	} else if((state==DISABLED) && (g_mac.state != DISABLED)) {
		/*xuan yang, 2013.5.30, send disconnected state to ui in the resume assocated stage*/
		send_mac_status(MAC_DISCONNECTED);
	}
	#if 0 //chenq mask 2012-11-19
	else if(state == SCAN_COMP)
	{
		send_mac_status(MAC_SCAN_CMP);
	}
	#endif
#endif	

#ifdef BSS_ACCESS_POINT_MODE
	if( (g_mac.state == ENABLED) && (state != ENABLED) )
	{
		//chenq mask 2013-01-12 
		//send_mac_status(MAC_DISCONNECTED);
	}
	else if( (g_mac.state != ENABLED) && (state == ENABLED) )
	{
		send_mac_status(MAC_CONNECTED);
	}
#endif
    /* Set the MAC state to the new state */
    g_mac.state = state;

// for wifi/bt coex 
#ifdef IBSS_BSS_STATION_MODE
	if(DISABLED == state){
		// clear tx int before wifi disconnect in coexistence situation. wzl
		coex_state_switch(COEX_WIFI_IDLE);
		/*
		if(BTRUE == g_wifi_bt_coex){
			UWORD32 arm2host = 0;
			arm2host = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
			if(arm2host & BIT0)	//tx int.
			{
				printk("[%s]: clear tx int before wifi disconnect\n", __FUNCTION__);
				arm2host &= (~BIT0);		
				host_write_trout_reg(arm2host, (UWORD32)rCOMM_ARM2HOST_INFO3);
			}
		}*/
	}
	host_notify_arm7_wifi_state(state);
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_mac_state                                            */
/*                                                                           */
/*  Description   : This function gets the current MAC state.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac                                                    */
/*                                                                           */
/*  Processing    : This function returns the global MAC structure state.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8; Current MAC state                                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_mac_state(void)
{
    return g_mac.state;
}

