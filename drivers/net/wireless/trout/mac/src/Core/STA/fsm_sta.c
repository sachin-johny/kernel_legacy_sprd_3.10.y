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
/*  File Name         : fsm_sta.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions related to the      */
/*                      initialization of the MAC FSM implementation for STA */
/*                      mode.                                                */
/*                                                                           */
/*  List of Functions : init_mac_fsm_sta                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "fsm_sta.h"
#include "qmu_if.h"
#include "receive.h"
#include "transmit.h"
#include "sta_prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mac_fsm_sta                                         */
/*                                                                           */
/*  Description   : This function initializes the MAC FSM for STA mode. The  */
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

void init_mac_fsm_sta(mac_struct_t *mac)
{
    UWORD32 i = 0;

	TROUT_FUNC_ENTER;
    /* The MAC FSM table is a 2-d array of function pointers indexed by the  */
    /* current MAC state and the input message.                              */
    /*                                                                       */
    /* All the STA MAC FSM states are,                                       */
    /* DISABLED, WAIT_START, WAIT_SCAN, SCAN_COMP, WAIT_JOIN, JOIN_COMP      */
    /* WAIT_AUTH_SEQ2, WAIT_AUTH_SEQ4, AUTH_COMP, WAIT_ASOC, ENABLED         */
    /*                                                                       */
    /* All the STA MAC FSM inputs are,                                       */
    /* SCAN_REQ, JOIN_REQ, AUTH_REQ, ASOC_REQ, START_REQ, WLAN_RX,           */
    /* WLAN_RX_MGMT_FRAME, MISC_EVENT, WLAN_TX_DATA_FRAME,                   */
    /* WLAN_RX_DATA_FRAME, ACTION_REQ                                        */

    /* Initialize the different columns of the table with the required       */
    /* functions based on the MAC state.                                     */

    /* Initialize the elements in the SCAN_REQ input column. The functions   */
    /* for all the states with this input is listed below.                   */
    /*                                                                       */
    /* +----------------+---------------------+                              */
    /* | FSM State      | FSM Function        |                              */
    /* +----------------+---------------------+                              */
    /* | DISABLED       | sta_wait_scan       |                              */
    /* | WAIT_START     | null_fn             |                              */
    /* | WAIT_SCAN      | null_fn             |                              */
    /* | SCAN_COMP      | sta_wait_scan       |                              */
    /* | WAIT_JOIN      | null_fn             |                              */
    /* | JOIN_COMP      | sta_wait_scan       |                              */
    /* | WAIT_AUTH_SEQ2 | null_fn             |                              */
    /* | WAIT_AUTH_SEQ4 | null_fn             |                              */
    /* | AUTH_COMP      | sta_wait_scan       |                              */
    /* | WAIT_ASOC      | null_fn             |                              */
    /* | ENABLED        | sta_wait_scan       |                              */
    /* +----------------+---------------------+                              */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][SCAN_REQ] = null_fn;
    }

    mac->fsm[DISABLED][SCAN_REQ]  = sta_wait_scan;
	//chenq add 2012-11-01
	mac->fsm[WAIT_SCAN][SCAN_REQ] = sta_wait_scan;
    mac->fsm[SCAN_COMP][SCAN_REQ] = sta_wait_scan;
    mac->fsm[JOIN_COMP][SCAN_REQ] = sta_wait_scan;
    mac->fsm[AUTH_COMP][SCAN_REQ] = sta_wait_scan;
    mac->fsm[ENABLED][SCAN_REQ]   = sta_wait_scan;

    /* Initialize the elements in the JOIN_REQ input column. This input is   */
    /* valid only in the SCAN_COMP state.                                    */
    /*                                                                       */
    /* +------------------+---------------------+                            */
    /* | FSM State        | FSM Function        |                            */
    /* +------------------+---------------------+                            */
    /* | SCAN_COMP        | sta_wait_join       |                            */
    /* | All other states | null_fn             |                            */
    /* +------------------+---------------------+                            */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][JOIN_REQ] = null_fn;
    }

    mac->fsm[SCAN_COMP][JOIN_REQ] = sta_wait_join;

    /* Initialize the elements in the AUTH_REQ input column. This input is   */
    /* valid only in the JOIN_COMP state.                                    */
    /*                                                                       */
    /* +------------------+---------------------+                            */
    /* | FSM State        | FSM Function        |                            */
    /* +------------------+---------------------+                            */
    /* | JOIN_COMP        | sta_wait_auth       |                            */
    /* | All other states | null_fn             |                            */
    /* +------------------+---------------------+                            */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][AUTH_REQ] = null_fn;
    }

    mac->fsm[JOIN_COMP][AUTH_REQ] = sta_wait_auth;

    /* Initialize the elements in the ASOC_REQ input column. This input is   */
    /* valid only in the AUTH_COMP state.                                    */
    /*                                                                       */
    /* +------------------+---------------------+                            */
    /* | FSM State        | FSM Function        |                            */
    /* +------------------+---------------------+                            */
    /* | AUTH_COMP        | sta_wait_asoc       |                            */
    /* | All other states | null_fn             |                            */
    /* +------------------+---------------------+                            */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][ASOC_REQ] = null_fn;
    }

    mac->fsm[AUTH_COMP][ASOC_REQ] = sta_wait_asoc;


    /* Initialize the elements in the START_REQ input column. This input is  */
    /* valid only in the DISABLED state.                                     */
    /*                                                                       */
    /* +------------------+---------------------+                            */
    /* | FSM State        | FSM Function        |                            */
    /* +------------------+---------------------+                            */
    /* | SCAN_COMP        | sta_wait_start      |                            */
    /* | All other states | null_fn             |                            */
    /* +------------------+---------------------+                            */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][START_REQ] = null_fn;
    }

    mac->fsm[SCAN_COMP][START_REQ] = sta_wait_start;

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
    /* +----------------+-----------------------+                            */
    /* | FSM State      | FSM Function          |                            */
    /* +----------------+-----------------------+                            */
    /* | DISABLED       | null_fn               |                            */
    /* | WAIT_START     | null_fn               |                            */
    /* | WAIT_SCAN      | sta_wait_scan_rx      |                            */
    /* | SCAN_COMP      | null_fn               |                            */
    /* | WAIT_JOIN      | sta_wait_join_rx      |                            */
    /* | JOIN_COMP      | null_fn               |                            */
    /* | WAIT_AUTH_SEQ2 | sta_wait_auth_seq2_rx |                            */
    /* | WAIT_AUTH_SEQ4 | sta_wait_auth_seq4_rx |                            */
    /* | AUTH_COMP      | null_fn               |                            */
    /* | WAIT_ASOC      | sta_wait_asoc_rx      |                            */
    /* | ENABLED        | sta_enabled_rx_mgmt   |                            */
    /* +----------------+-----------------------+                            */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_RX_MGMT_FRAME] = null_fn;
    }

    mac->fsm[WAIT_SCAN][WLAN_RX_MGMT_FRAME]      = sta_wait_scan_rx;
    mac->fsm[WAIT_JOIN][WLAN_RX_MGMT_FRAME]      = sta_wait_join_rx;
    mac->fsm[WAIT_AUTH_SEQ2][WLAN_RX_MGMT_FRAME] = sta_wait_auth_seq2_rx;
    mac->fsm[WAIT_AUTH_SEQ4][WLAN_RX_MGMT_FRAME] = sta_wait_auth_seq4_rx;
    mac->fsm[WAIT_ASOC][WLAN_RX_MGMT_FRAME]      = sta_wait_asoc_rx;
    mac->fsm[ENABLED][WLAN_RX_MGMT_FRAME]        = sta_enabled_rx_mgmt;

    /* Initialize the elements in the MISC_EVENT column. The functions       */
    /* for all the states with this input is listed below.                   */
    /*                                                                       */
    /* +----------------+-------------------------+                          */
    /* | FSM State      | FSM Function            |                          */
    /* +----------------+-------------------------+                          */
    /* | DISABLED       | misc_null_fn            |                          */
    /* | WAIT_START     | sta_wait_start_misc     |                          */
    /* | WAIT_SCAN      | sta_wait_scan_misc      |                          */
    /* | SCAN_COMP      | misc_null_fn            |                          */
    /* | WAIT_JOIN      | sta_wait_join_misc      |                          */
    /* | JOIN_COMP      | misc_null_fn            |                          */
    /* | WAIT_AUTH_SEQ2 | sta_wait_auth_misc      |                          */
    /* | WAIT_AUTH_SEQ4 | sta_wait_auth_misc      |                          */
    /* | AUTH_COMP      | misc_null_fn            |                          */
    /* | WAIT_ASOC      | sta_wait_asoc_misc      |                          */
    /* | ENABLED        | sta_enabled_misc        |                          */
    /* +----------------+-------------------------+                          */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][MISC_EVENT] = misc_null_fn;
    }

    mac->fsm[WAIT_SCAN][MISC_EVENT]      = sta_wait_scan_misc;
    mac->fsm[WAIT_START][MISC_EVENT]     = sta_wait_start_misc;
    mac->fsm[WAIT_JOIN][MISC_EVENT]      = sta_wait_join_misc;
    mac->fsm[WAIT_AUTH_SEQ2][MISC_EVENT] = sta_wait_auth_misc;
    mac->fsm[WAIT_AUTH_SEQ4][MISC_EVENT] = sta_wait_auth_misc;
    mac->fsm[WAIT_ASOC][MISC_EVENT]      = sta_wait_asoc_misc;
    mac->fsm[ENABLED][MISC_EVENT]        = sta_enabled_misc;

    /* Initialize the elements in the WLAN_TX_DATA_FRAME input column. This  */
    /* input is valid only in the ENABLED state.                             */
    /*                                                                       */
    /* +------------------+----------------------+                           */
    /* | FSM State        | FSM Function         |                           */
    /* +------------------+----------------------+                           */
    /* | ENABLED          | sta_enabled_tx_data  |                           */
    /* | All other states | wlan_tx_data_null_fn |                           */
    /* +------------------+----------------------+                           */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_TX_DATA_FRAME] = wlan_tx_data_null_fn;
    }

    mac->fsm[WAIT_SCAN][WLAN_TX_DATA_FRAME] = sta_wait_scan_tx_data;
    mac->fsm[ENABLED][WLAN_TX_DATA_FRAME] = sta_enabled_tx_data;

    /* Initialize the elements in the WLAN_RX_DATA_FRAME input column. This  */
    /* input is valid only in the ENABLED state.                             */
    /*                                                                       */
    /* +------------------+----------------------+                           */
    /* | FSM State        | FSM Function         |                           */
    /* +------------------+----------------------+                           */
    /* | ENABLED          | sta_enabled_rx_data  |                           */
    /* | All other states | wlan_rx_data_null_fn |                           */
    /* +------------------+----------------------+                           */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][WLAN_RX_DATA_FRAME] = wlan_rx_data_null_fn;
    }

    mac->fsm[ENABLED][WLAN_RX_DATA_FRAME] = sta_enabled_rx_data;

    /* Initialize the elements in the ACTION_REQ input column. This input is */
    /* valid only in the ENABLED state.                                      */
    /*                                                                       */
    /* +------------------+------------------------+                         */
    /* | FSM State        | FSM Function           |                         */
    /* +------------------+------------------------+                         */
    /* | ENABLED          | sta_enabled_action_req |                         */
    /* | All other states | null_fn                |                         */
    /* +------------------+------------------------+                         */

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][ACTION_REQ] = null_fn;
    }

    mac->fsm[ENABLED][ACTION_REQ] = sta_enabled_action_req;

#ifdef NO_ACTION_RESET
    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        mac->fsm[i][ACTION_REQ] = sta_enabled_action_req;
    }
#endif /* NO_ACTION_RESET */

    /* Initialize the MAC state to DISABLED */
    mac->state = DISABLED;
    TROUT_FUNC_EXIT;
}

#ifdef ENABLE_STACK_ESTIMATION
void mlme_scan_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_scan(mac, msg);
}

void mlme_join_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_join(mac, msg);
}

void mlme_auth_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_auth(mac, msg);
}

void mlme_asoc_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_asoc(mac, msg);
}

void mlme_start_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_start(mac, msg);
}

void mlme_action_req(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_enabled_action_req(mac, msg);
}

void wlan_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
    null_fn(mac, msg);
    sta_wait_scan_rx(mac, msg);
    sta_wait_join_rx(mac, msg);
    sta_wait_auth_seq2_rx(mac, msg);
    sta_wait_auth_seq4_rx(mac, msg);
    sta_wait_asoc_rx(mac, msg);
    sta_enabled_rx_mgmt(mac, msg);
}

void wlan_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_null_fn(mac, msg);
    sta_wait_scan_misc(mac, msg);
    sta_wait_start_misc(mac, msg);
    sta_wait_join_misc(mac, msg);
    sta_wait_auth_misc(mac, msg);
    sta_wait_auth_misc(mac, msg);
    sta_wait_asoc_misc(mac, msg);
    sta_enabled_misc(mac, msg);
}

void wlan_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_tx_data_null_fn(mac, msg);
    sta_wait_scan_tx_data(mac, msg);
    sta_enabled_tx_data(mac, msg);
}

void wlan_rx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_data_null_fn(mac, msg);
    sta_enabled_rx_data(mac, msg);
}

void wlan_rx(mac_struct_t *mac, UWORD8 *msg)
{
    process_wlan_rx(mac, msg);
}

#endif /* ENABLE_STACK_ESTIMATION */

#endif /* IBSS_BSS_STATION_MODE */
