/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : p2p_host_rx_mlme_sta.c                               */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      HOST_RX (MLME) events for P2P mode.                  */
/*                                                                           */
/*  List of Functions : p2p_sta_wait_join                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "fsm_sta.h"
#include "sta_mgmt_p2p.h"
#include "sta_prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_sta_wait_join                                        */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  P2P join (group formation) process.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core functions are called based on   */
/*                  the input join request parameters. The MAC state is      */
/*                  changed to WAIT_JOIN.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_sta_wait_join(mac_struct_t *mac, UWORD8 *msg)
{
    p2p_join_req_t *p2p_join_req = (p2p_join_req_t *)msg;

    /* Set the MAC state to WAIT_JOIN */
    set_mac_state(WAIT_JOIN);

    /* Get the required information of the network or device that the P2P    */
    /* device must form a group with.                                        */
    memcpy(&g_p2p_join_req, p2p_join_req, sizeof(p2p_join_req_t));

    /* Perform required function based on the Join Method */
    switch(g_p2p_join_req.join_method)
    {
    case P2P_INVITE_PERSIST:
    {
		// 20120709 caisf add the "if", merged ittiam mac v1.2 code
        if(BFALSE == mget_p2p_listen_mode())
		{
        	p2p_wait_inv_rsp();
		}
    }
    break;
    case P2P_GO_NEG:
    {
        try_p2p_wait_go_neg_rsp();
    }
    break;
    case P2P_DDR_INVITE_PERSIST:
    case P2P_DDR_GO_NEG:
    {
        p2p_wait_dev_disc_rsp();
    }
    break;
    case P2P_JOIN_GROUP:
    {
        p2p_wait_prov_disc_rsp();
    }
    break;
    default:
    {
        /* Invalid Join method. Do nothing. */
        p2p_join_complete(mac, INVALID_MLMESTATUS);
    }
    break;
    }
}

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */

