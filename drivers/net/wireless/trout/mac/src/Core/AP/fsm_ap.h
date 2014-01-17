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
/*  File Name         : fsm_ap.h                                             */
/*                                                                           */
/*  Description       : This file contains all the declarations related to   */
/*                      MAC FSM implementation for AP mode.                  */
/*                                                                           */
/*  List of Functions : mlme_start_req                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef FSM_AP_H
#define FSM_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "fsm.h"

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void init_mac_fsm_ap(mac_struct_t *mac);
extern void ap_wait_start(mac_struct_t *mac, UWORD8 *msg);
extern void ap_wait_start_rx_mgmt(mac_struct_t *mac, UWORD8 *msg);
extern void ap_enabled_rx_mgmt(mac_struct_t *mac, UWORD8 *msg);
extern void ap_enabled_rx_data(mac_struct_t * mac, UWORD8 * msg);
extern void ap_enabled_tx_data(mac_struct_t * mac, UWORD8 * msg);
extern void ap_wait_start_misc(mac_struct_t *mac, UWORD8 *msg);
extern void ap_enabled_misc(mac_struct_t *mac, UWORD8 *msg);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* MLME API functions - these functions are called upon receiving an MLME    */
/* request from the host.                                                    */
/*****************************************************************************/

#ifndef ENABLE_STACK_ESTIMATION

INLINE void mlme_start_req(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("mlme_start_req, FSM[%d][%d]\n", mac->state, START_REQ);
    mac->fsm[mac->state][START_REQ](mac, msg);
}

INLINE void mlme_action_req(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("mlme_action_req, FSM[%d][%d]\n", mac->state, ACTION_REQ);
    mac->fsm[mac->state][ACTION_REQ](mac, msg);
}
#else /* ENABLE_STACK_ESTIMATION */
extern void mlme_start_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_action_req(mac_struct_t *mac, UWORD8 *msg);
#endif /* ENABLE_STACK_ESTIMATION */

#endif /* FSM_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
