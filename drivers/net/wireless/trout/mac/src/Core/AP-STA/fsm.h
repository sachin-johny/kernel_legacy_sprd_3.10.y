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
/*  File Name         : fsm.h                                                */
/*                                                                           */
/*  Description       : This file contains inline functions related to MAC   */
/*                      FSM for both the AP and STA modes.                   */
/*                                                                           */
/*  List of Functions : null_fn                                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef FSM_H
#define FSM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "maccontroller.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* The MAC states for Station mode */
typedef enum {DISABLED          = 0,
              ENABLED           = 1,
              WAIT_START        = 2,
              MAX_MAC_FSM_STATE = 3
} MAC_FSM_STATE_T;

/* The Input Messages for Station mode */
typedef enum {START_REQ          = 0,
              WLAN_RX_EVENT      = 1,
              WLAN_RX_MGMT_FRAME = 2,
              WLAN_RX_DATA_FRAME = 3,
              MISC_EVENT         = 4,
              WLAN_TX_DATA_FRAME = 5,
              ACTION_REQ         = 6,
              MAX_MAC_FSM_INPUT  = 7
} MAC_FSM_INPUT_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void misc_null_fn(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_rx_data_null_fn(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_tx_data_null_fn(mac_struct_t *mac, UWORD8 *msg);

extern void   set_mac_state(UWORD8 state);
extern UWORD8 get_mac_state(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Null function which does nothing */
INLINE void null_fn(mac_struct_t *mac, UWORD8 *msg)
{
    /* Do nothing */
}

#ifndef ENABLE_STACK_ESTIMATION
/* WLAN API functions - these functions are called upon receiving a frame    */
/* from WLAN, transmitting to WLAN or a getting a hardware interrupt.        */
INLINE void wlan_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("wlan_rx_mgmt, FSM[%d][%d]\n", mac->state, WLAN_RX_MGMT_FRAME);
    mac->fsm[mac->state][WLAN_RX_MGMT_FRAME](mac, msg);
}

INLINE void wlan_misc(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("wlan_misc, FSM[%d][%d]\n", mac->state, MISC_EVENT);
    mac->fsm[mac->state][MISC_EVENT](mac, msg);
}

INLINE void wlan_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("wlan_tx_data, FSM[%d][%d]\n", mac->state, WLAN_TX_DATA_FRAME);
    mac->fsm[mac->state][WLAN_TX_DATA_FRAME](mac, msg);
}

INLINE void wlan_rx_data(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("wlan_rx_data, FSM[%d][%d]\n", mac->state, WLAN_RX_DATA_FRAME);
    mac->fsm[mac->state][WLAN_RX_DATA_FRAME](mac, msg);
}

INLINE void wlan_rx(mac_struct_t *mac, UWORD8 *msg)
{
	TROUT_DBG6("wlan_rx, FSM[%d][%d]\n", mac->state, WLAN_RX_EVENT);
    mac->fsm[mac->state][WLAN_RX_EVENT](mac, msg);
}
#else /* ENABLE_STACK_ESTIMATION */

/* WLAN API functions - these functions are called upon receiving a frame    */
/* from WLAN, transmitting to WLAN or a getting a hardware interrupt.        */
extern void wlan_rx_mgmt(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_misc(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_tx_data(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_rx_data(mac_struct_t *mac, UWORD8 *msg);
extern void wlan_rx(mac_struct_t *mac, UWORD8 *msg);

#endif /* ENABLE_STACK_ESTIMATION */
#endif /* FSM_H */
