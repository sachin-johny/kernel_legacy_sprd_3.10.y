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
/*  File Name         : controller_sta_prot_if.h                             */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      STA mode for the Controller.                         */
/*                                                                           */
/*  List of Functions : Protocol interface functions for STA Controller      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef CONTROLLER_STA_PROT_IF_H
#define CONTROLLER_STA_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "event_manager.h"
#include "sta_prot_if.h"

#ifdef MAC_P2P
#include "p2p_sme_sta.h"
#include "sta_mgmt_p2p.h"
#endif /* MAC_P2P */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* P2P protocol Interface Functions                                          */
/*****************************************************************************/

INLINE BOOL_T handle_scan_rsp_prot(mac_struct_t *mac, UWORD8 *msg)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        ret_val = p2p_handle_scan_rsp(mac, msg);
    }
#endif /* MAC_P2P */

    return ret_val;
}

/* This function handles the Join Success based on protocols in use */
INLINE BOOL_T handle_join_success_prot(mac_struct_t *mac)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        p2p_end_grp_form(mac);
        ret_val = BTRUE;
    }
#endif /* MAC_P2P */

    return ret_val;
}


// 20120709 caisf add, merged ittiam mac v1.2 code
/* This function resets P2P group formation in progress flag if normal join  */
/* is initiated (and not P2P join)                                           */
INLINE void update_non_p2p_join_prot(void)
{
#ifdef MAC_P2P
    set_p2p_grp_form_in_prog(BFALSE);
#endif /* MAC_P2P */
}


#endif /* CONTROLLER_STA_PROT_IF_H */
#endif /* IBSS_BSS_STATION_MODE */
