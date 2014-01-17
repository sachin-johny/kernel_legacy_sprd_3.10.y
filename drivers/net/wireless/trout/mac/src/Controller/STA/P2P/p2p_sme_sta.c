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
/*  File Name         : p2p_sme_sta.c                                        */
/*                                                                           */
/*  Description       : This file contains the SME functions specific to     */
/*                      P2P STA mode.                                        */
/*                                                                           */
/*  List of Functions : p2p_check_scan_match                                 */
/*                      p2p_handle_scan_rsp                                  */
/*                      p2p_prepare_mlme_join_req                            */
/*                      p2p_handle_grp_form_comp                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "event_manager.h"
#include "management_sta.h"
#include "sta_mgmt_p2p.h"
#include "mib_p2p.h"
#include "frame_p2p.h"
#include "p2p_sme_sta.h"
#include "fsm_sta.h"
#include "sme_sta.h"
#include "wps_sta.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_check_scan_match                                     */
/*                                                                           */
/*  Description   : This function matches the device id's to find a match    */
/*                  It also finds if the device is part of an existing       */
/*                  network and returns the index to the bss descriptor      */
/*                                                                           */
/*  Inputs        : scan response                                            */
/*                  pointer to bss descriptor index                          */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The function matches the device id to the target device  */
/*                  id and evaluates the joining method to be used           */
/*                  If the device is part of an existing group, it returns   */
/*                  the index to the bss descriptor set also                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : direct - Join Method                                     */
/*                  indirect - index to the bss descriptor                   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
P2P_JOIN_METHOD_T p2p_check_scan_match(scan_rsp_t *scan_rsp, UWORD8 *index)
{

    P2P_JOIN_METHOD_T join_method = P2P_SCAN_MATCH_FAIL;
    UWORD8 grp_capability         = 0;
    UWORD8 dev_capability         = 0;
    UWORD8 idx                    = 0;

    for(idx = 0; idx < scan_rsp->p2p_num_dscr; idx++)
    {
        if(BTRUE == mac_addr_cmp(mget_p2p_trgt_dev_id(),
                                 scan_rsp->p2p_dev_dscr[idx].dev_addr))
        {
            /* Matching device found */
            join_method = P2P_UNAVAILABLE;
            /* Check if the device is part of an existing group */
            if(BTRUE ==
                       is_mac_addr_null(scan_rsp->p2p_dev_dscr[idx].grp_bssid))
            {
                dev_capability = scan_rsp->p2p_dev_dscr[idx].dev_capability;
                /* Check if the device supports invitation procedure and if  */
                /* persistent grp info is avaliable                          */
                if(CHECK_BIT(dev_capability, P2P_INVIT_PROC) &&
                   (BTRUE == check_persist_grp_info(mget_p2p_trgt_dev_id())))
                {
                    join_method = P2P_INVITE_PERSIST;
                }
                else
                {
                    join_method = P2P_GO_NEG;
                }
            }
            else
            {
                grp_capability = scan_rsp->p2p_dev_dscr[idx].grp_capability;
				/* Check if the target id is a GO */
				if(CHECK_BIT(grp_capability, P2PCAPBIT_GO))
				{
					/* Check if we can join the GO */
                	if(!CHECK_BIT(grp_capability, P2PCAPBIT_GRP_LMT))
                	{
                        join_method = P2P_JOIN_GROUP;
                    }
                	else
                	{   /* Can't join the group :( */
                    	dev_capability =scan_rsp->p2p_dev_dscr[idx].dev_capability;
                        /* Check if the device supports invitation procedure */
                        /* and if persistent grp info is avaliable           */
                        if(CHECK_BIT(dev_capability,P2P_INVIT_PROC) &&
                           (BTRUE == check_persist_grp_info(
                                        mget_p2p_trgt_dev_id())))
                        {
                            join_method = P2P_INVITE_PERSIST;
                        }
                        else
                        {
                            join_method = P2P_GO_NEG;
                        }
                    }
				}
                else
                {
					dev_capability = scan_rsp->p2p_dev_dscr[idx].dev_capability;
                    /* Check if the client supports device discovery  */
                    /* procedure                                      */
                    if(CHECK_BIT(dev_capability, P2P_CLIENT_DISC))
                    {
                        /* Check if the device supports invitation       */
                        /* procedure and if persistent grp info is       */
                        /* avaliable                                     */
                        if(CHECK_BIT(dev_capability, P2P_INVIT_PROC)
                           && (BTRUE == check_persist_grp_info(
                                        mget_p2p_trgt_dev_id())))
                        {
                            join_method = P2P_DDR_INVITE_PERSIST;
                        }
                        else
                        {
                            join_method = P2P_DDR_GO_NEG;
                        }
                    }
                }
            }
            /* Once a match is found set the index and break */
            break;
        }
    }

    *index = idx;
    return join_method;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_handle_scan_rsp                                      */
/*                                                                           */
/*  Description   : This function checks for P2P scan match.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the scan response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T; BTRUE if a P2P scan match was found              */
/*                          BFALSE, otherwise                                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T p2p_handle_scan_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    scan_rsp_t *scan_rsp = (scan_rsp_t *)msg;
    UWORD8     index     = 0;
    P2P_JOIN_METHOD_T status = P2P_SCAN_MATCH_FAIL;

	TROUT_FUNC_ENTER;
    /* Process the scan response and find the matching BSS descriptor for    */
    /* joining an existing group or the matching device descriptor to start  */
    /* group formation with.                                                 */
    status = p2p_check_scan_match(scan_rsp, &index);

    if(P2P_SCAN_MATCH_FAIL == status || P2P_UNAVAILABLE == status)
    {
		TROUT_FUNC_EXIT;
        return BFALSE;
	}
	
    TROUT_DBG4("P2P match found P2P_JOIN_METHOD = %d\n\r", status);
    /* Start Group Formation */

    /* If the result of the processing is to form a group with a device     */
    /* initiate the same using a join request.                              */
    p2p_initiate_join(mac, &(scan_rsp->p2p_dev_dscr[index]), status);

	TROUT_FUNC_EXIT;
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_prepare_mlme_join_req                                */
/*                                                                           */
/*  Description   : This function prepares the P2P MLME join request.        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the join request buffer                    */
/*                  2) Pointer to the descriptor of the device to join       */
/*                  3) The method to form the group with the device          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_prepare_mlme_join_req(p2p_join_req_t *p2p_join_req,
                               p2p_dev_dscr_t *dev_dscr, UWORD8 join_method)
{
    mem_set(p2p_join_req, 0, sizeof(p2p_join_req_t));

    /* Set the fields of the P2P join request */
    memcpy(&p2p_join_req->dev_dscr, dev_dscr, sizeof(p2p_dev_dscr_t));
    p2p_join_req->join_method = (P2P_JOIN_METHOD_T)join_method;
}

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
