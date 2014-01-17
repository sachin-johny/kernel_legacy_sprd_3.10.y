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
/*  File Name         : p2p_ps_sta.h                                         */
/*                                                                           */
/*  Description       : This file conatins the power save management         */
/*                      functions for P2P protocol                           */
/*                                                                           */
/*  List of Functions : is_ps_poll_rsp_pending                               */
/*                      set_ps_poll_rsp_pending                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

#ifndef P2P_PS_STA_H
#define P2P_PS_STA_H
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* MAC HW Schedule Status */
typedef enum
{
    NOA_STAT_EXPIRED   = 0,
    NOA_STAT_CURRENT   = 1,
    NOA_STAT_NEW       = 2,
    NOA_STAT_CANCELLED = 3
} NOA_STATUS_T;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define MACHW_P2P_RX_BCN_TO          550 /* in micro seconds */
#define AB_PERIOD_START_TIME_OFFSET 1000 /* in micro seconds */


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_ps_poll_pending;
extern WORD16 g_noa_index;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void handle_p2p_go_absence_period_start(void);
extern void handle_p2p_go_absence_period_end(void);
extern void process_p2p_noa_attr(UWORD8 *attr_start_ptr, UWORD8 ie_len);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function checks if a PS Poll Response is pending */
INLINE BOOL_T is_ps_poll_rsp_pending(void)
{
    return g_ps_poll_pending;
}

/* This function sets the PS Poll Response pending flag */
INLINE void set_ps_poll_rsp_pending(BOOL_T val)
{
    g_ps_poll_pending = val;
}

#endif /* P2P_PS_STA_H */

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
