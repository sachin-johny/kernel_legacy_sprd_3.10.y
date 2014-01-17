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
/*  File Name         : p2p_sme_sta.h                                        */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to SME functions specific to P2P STA mode.   */
/*                                                                           */
/*  List of Functions : p2p_initiate_join                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

#ifndef P2P_SME_STA_H
#define P2P_SME_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "sta_mgmt_p2p.h"
#include "fsm_sta.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern BOOL_T p2p_handle_scan_rsp(mac_struct_t *mac, UWORD8 *msg);
extern void p2p_prepare_mlme_join_req(p2p_join_req_t *p2p_join_req,
                               p2p_dev_dscr_t *dev_dscr, UWORD8 join_method);


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
/* This function prepares the P2P join request message and calls the MAC API */
/* to start joining.                                                         */
INLINE void p2p_initiate_join(mac_struct_t *mac, p2p_dev_dscr_t *dev_dscr,
                              UWORD8 join_method)
{
    p2p_join_req_t p2p_join_req;

    p2p_prepare_mlme_join_req(&p2p_join_req, dev_dscr, join_method);

    mlme_join_req(mac, (UWORD8*)&p2p_join_req);
}

/* This function checks if Persistent Group Info is available */
INLINE BOOL_T check_persist_grp_info(UWORD8 *target_dev_id)
{
    UWORD8 num = 0;
    BOOL_T status = BFALSE;

    /* Check if we have valid persistent credential */
    if(g_persist_list.num_cl > 0)
    {
        /* Check the role we had in the persistent grp */
        if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
        {

            for(num = 0; num < g_persist_list.num_cl; num++)
            {
                if(BTRUE == mac_addr_cmp(target_dev_id,
                             g_persist_list.cl_list[num]))
                {
                    status = BTRUE;
                    break;
                }
            }
        }
        else
        {
            if(BTRUE == mac_addr_cmp(target_dev_id, g_persist_list.cl_list[0]))
            {
                status = BTRUE;
            }
        }
    }

    if(BTRUE == status)
    {
        g_curr_persist_list = &g_persist_list;
    }
    return status;
}

#endif /* P2P_SME_STA_H */

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
