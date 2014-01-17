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
/*  File Name         : fsm_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains all the declarations related to   */
/*                      MAC FSM implementation for STA mode.                 */
/*                                                                           */
/*  List of Functions : mlme_scan_req                                        */
/*                      mlme_join_req                                        */
/*                      mlme_auth_req                                        */
/*                      mlme_asoc_req                                        */
/*                      mlme_start_req                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef FSM_STA_H
#define FSM_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "fsm.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* The MAC states for Station mode */
typedef enum {WAIT_SCAN = MAX_MAC_FSM_STATE,
              SCAN_COMP,
              WAIT_JOIN,
              JOIN_COMP,
              WAIT_AUTH_SEQ2,
              WAIT_AUTH_SEQ4,
              AUTH_COMP,
              WAIT_ASOC,
              MAX_STA_MAC_FSM_STATE
} STA_MAC_FSM_STATE_T;

/* The Input Messages for Station mode */
typedef enum {SCAN_REQ = MAX_MAC_FSM_INPUT,
              JOIN_REQ,
              AUTH_REQ,
              ASOC_REQ,
              MAX_STA_MAC_FSM_INPUT
} STA_MAC_FSM_INPUT_T;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void init_mac_fsm_sta(mac_struct_t *mac);
extern void sta_wait_start(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_start_misc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_scan(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_scan_rx(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_scan_misc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_join(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_join_rx(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_join_misc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_auth_seq2_rx(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_auth_seq4_rx(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_auth(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_auth_misc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_asoc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_asoc_rx(mac_struct_t *mac, UWORD8 *msg);
extern void sta_wait_asoc_misc(mac_struct_t *mac, UWORD8 *msg);
extern void sta_enabled_rx_mgmt(mac_struct_t *mac, UWORD8 *msg);
extern void sta_enabled_tx_data(mac_struct_t * mac, UWORD8 * msg);
extern void sta_wait_scan_tx_data(mac_struct_t * mac, UWORD8 * msg);
extern void sta_enabled_rx_data(mac_struct_t * mac, UWORD8 * msg);
extern void sta_enabled_misc(mac_struct_t *mac, UWORD8 *msg);

extern void process_scan_itr_comp(mac_struct_t *mac);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* MLME API functions - these functions are called upon receiving an MLME    */
/* request from the host.                                                    */
/*****************************************************************************/

#ifndef ENABLE_STACK_ESTIMATION
INLINE void mlme_scan_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][SCAN_REQ](mac, msg);
}

INLINE void mlme_join_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][JOIN_REQ](mac, msg);
}

INLINE void mlme_auth_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][AUTH_REQ](mac, msg);
}

INLINE void mlme_asoc_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][ASOC_REQ](mac, msg);
}

INLINE void mlme_start_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][START_REQ](mac, msg);
}

INLINE void mlme_action_req(mac_struct_t *mac, UWORD8 *msg)
{
    mac->fsm[mac->state][ACTION_REQ](mac, msg);
}
#else /* ENABLE_STACK_ESTIMATION */
extern void mlme_scan_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_join_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_auth_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_asoc_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_start_req(mac_struct_t *mac, UWORD8 *msg);
extern void mlme_action_req(mac_struct_t *mac, UWORD8 *msg);
#endif /* ENABLE_STACK_ESTIMATION */
#endif /* FSM_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
