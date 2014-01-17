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
/*  File Name         : fsm_ap.c                                             */
/*                                                                           */
/*  Description       : This file contains the functions related to the      */
/*                      initialization of the MAC FSM implementation for AP  */
/*                      mode.                                                */
/*                                                                           */
/*  List of Functions : init_mac_fsm_ap                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "fsm_ap.h"
#include "receive.h"
#include "ap_prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mac_fsm_ap                                          */
/*                                                                           */
/*  Description   : This function initializes the MAC FSM for AP mode. The   */
/*                  FSM is maintained as a table of function pointers        */
/*                  indexed by the current MAC state and the input message.  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The FSM table of the MAC library structure given as      */
/*                  input is initialized to the required functions. This is  */
/*                  done only once during start up and remains unaltered     */
/*                  thereafter.                                              */
/*                                                                           */
/*  Outputs       : FSM table of the input MAC library structure is set      */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_mac_fsm_ap(mac_struct_t *mac)
{
    UWORD32 i = 0;
	TROUT_FUNC_ENTER;
    /* The MAC FSM table is a 2-d array of function pointers indexed by the  */
    /* current MAC state and the input message.                              */
    /*                                                                       */
    /* All the AP MAC FSM states are,                                        */
    /* DISABLED, WAIT_START, ENABLED                                         */
    /*                                                                       */
    /* All the AP MAC FSM inputs are,                                        */
    /* START_REQ, WLAN_RX, WLAN_RX_MGMT_FRAME, WLAN_RX_DATA_FRAME,           */
    /* WLAN_TX_DATA_FRAME, MISC_EVENT, ACTION_REQ                            */

    /* Initialize the different columns of the table with the required       */
    /* functions based on the MAC state.                                     */

    /* Initialize the elements in the START_REQ input column. The functions  */
    /* for all the states with this input is listed below.                   */
    /*                                                                       */
    /* +------------------+-------------------+                              */
    /* | FSM State        | FSM Function      |                              */
    /* +------------------+-------------------+                              */
    /* | WAIT_START       | null_fn           |                              */
    /* | All other states | ap_wait_start     |                              */
    /* +------------------+-------------------+                              */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][START_REQ] = ap_wait_start;
    }

    mac->fsm[WAIT_START][START_REQ] = null_fn;

    /* Initialize the elements in the WLAN_RX input column. The functions    */
    /* for all the states with this input is listed below.                   */
    /*                                                                       */
    /* +----------------+---------------------+                              */
    /* | FSM State      | FSM Function        |                              */
    /* +----------------+---------------------+                              */
    /* | All states     | process_wlan_rx     |                              */
    /* +----------------+---------------------+                              */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_RX_EVENT] = process_wlan_rx;
    }

    /* Initialize the elements in the WLAN_RX_MGMT_FRAME input column. The   */
    /* functions for all the states with this input is listed below.         */
    /*                                                                       */
    /* +------------------+-----------------------+                          */
    /* | FSM State        | FSM Function          |                          */
    /* +------------------+-----------------------+                          */
    /* | WAIT_START       | ap_wait_start_rx_mgmt |                          */
    /* | ENABLED          | ap_enabled_rx         |                          */
    /* | All other states | wlan_rx_mgmt_null_fn  |                          */
    /* +------------------+-----------------------+                          */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_RX_MGMT_FRAME] = null_fn;
    }
    mac->fsm[WAIT_START][WLAN_RX_MGMT_FRAME] = ap_wait_start_rx_mgmt;
    mac->fsm[ENABLED][WLAN_RX_MGMT_FRAME]    = ap_enabled_rx_mgmt;

    /* Initialize the elements in the WLAN_RX_DATA_FRAME input column. This  */
    /* input is valid only in the ENABLED state.                             */
    /*                                                                       */
    /* +------------------+----------------------+                           */
    /* | FSM State        | FSM Function         |                           */
    /* +------------------+----------------------+                           */
    /* | ENABLED          | ap_rx_data_frame     |                           */
    /* | All other states | wlan_rx_data_null_fn |                           */
    /* +------------------+----------------------+                           */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_RX_DATA_FRAME] = wlan_rx_data_null_fn;
    }

    mac->fsm[ENABLED][WLAN_RX_DATA_FRAME] = ap_enabled_rx_data;

    /* Initialize the elements in the WLAN_TX_DATA_FRAME input column. This  */
    /* input is valid only in the ENABLED state.                             */
    /*                                                                       */
    /* +------------------+----------------------+                           */
    /* | FSM State        | FSM Function         |                           */
    /* +------------------+----------------------+                           */
    /* | ENABLED          | ap_tx_data_frame     |                           */
    /* | All other states | wlan_tx_data_null_fn |                           */
    /* +------------------+----------------------+                           */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_TX_DATA_FRAME] = wlan_tx_data_null_fn;
    }

    mac->fsm[ENABLED][WLAN_TX_DATA_FRAME] = ap_enabled_tx_data;

    /* Initialize the elements in the MISC_EVENT column. The functions       */
    /* for all the states with this input is listed below.                   */
    /*                                                                       */
    /* +----------------+-------------------------+                          */
    /* | FSM State      | FSM Function            |                          */
    /* +----------------+-------------------------+                          */
    /* | DISABLED       | misc_null_fn            |                          */
    /* | WAIT_START     | ap_wait_start_misc      |                          */
    /* | ENABLED        | ap_enabled_misc         |                          */
    /* +----------------+-------------------------+                          */

    mac->fsm[DISABLED][MISC_EVENT]   = misc_null_fn;
    mac->fsm[WAIT_START][MISC_EVENT] = ap_wait_start_misc;
    mac->fsm[ENABLED][MISC_EVENT]    = ap_enabled_misc;

    /* Initialize the elements in the ACTION_REQ input column. This input is */
    /* valid only in the ENABLED state.                                      */
    /*                                                                       */
    /* +------------------+------------------------+                         */
    /* | FSM State        | FSM Function           |                         */
    /* +------------------+------------------------+                         */
    /* | ENABLED          | ap_enabled_action_req  |                         */
    /* | All other states | null_fn                |                         */
    /* +------------------+------------------------+                         */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][ACTION_REQ] = null_fn;
    }

    mac->fsm[ENABLED][ACTION_REQ] = ap_enabled_action_req;

    /* Initialize the MAC state to DISABLED */
    set_mac_state(DISABLED);
    TROUT_FUNC_EXIT;
}

#ifdef ENABLE_STACK_ESTIMATION
void mlme_start_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    ap_wait_start(mac, msg);
}

void mlme_action_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    ap_enabled_action_req(mac, msg);
}

void wlan_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    ap_wait_start_rx_mgmt(mac, msg);
    ap_enabled_rx_mgmt(mac, msg);
}

void wlan_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_null_fn(mac, msg);
    ap_wait_start_misc(mac, msg);
    ap_enabled_misc(mac, msg);
}

void wlan_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_tx_data_null_fn(mac, msg);
    ap_enabled_tx_data(mac, msg);
}

void wlan_rx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_data_null_fn(mac, msg);
    ap_enabled_rx_data(mac, msg);
}

void wlan_rx(mac_struct_t *mac, UWORD8 *msg)
{
    process_wlan_rx(mac, msg);
}
#endif /* ENABLE_STACK_ESTIMATION */

#endif /* BSS_ACCESS_POINT_MODE */
