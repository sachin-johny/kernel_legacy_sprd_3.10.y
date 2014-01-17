/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : ap_management_mdom.c                                 */
/*                                                                           */
/*  Description       : This file contains MAC management related functions  */
/*                      and definitions for Multi-domain operation.          */
/*                                                                           */
/*  List of Functions : get_rc_info_sec_chan_offset                          */
/*                      select_suitable_reg_class                            */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_MULTIDOMAIN

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "frame.h"
#include "ap_management_mdom.h"
#include "ap_prot_if.h"
#include "common.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_rc_info_sec_chan_offset                              */
/*                                                                           */
/*  Description   : This function is called to get suitable Regulatory Class */
/*                  for given channel index and secondary channel offset     */
/*                                                                           */
/*  Inputs        : 1) Channel Index                                         */
/*                  2) Secondary Channel Offset                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets suitable Regulatory Class for the     */
/*                  given channel index. This function also takes care of    */
/*                  40MHz operation if secondary channel offset is not equal */
/*                  to SCN.                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Regulatory Class                                         */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_rc_info_sec_chan_offset(UWORD8 ch_idx, UWORD8 sec_offset)
{
    UWORD8 num_rc              = 0;
    UWORD8 rc_idx              = 0;
    UWORD8 reg_domain          = get_current_reg_domain();
    BOOL_T flag_sec_offset     = BFALSE;
    reg_domain_info_t *rd_info = NULL;
    reg_class_info_t *rc_info  = NULL;

    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return INVALID_REG_CLASS;
    }

    rd_info = get_reg_domain_info(reg_domain);
    num_rc  = get_num_reg_class(rd_info);

    /* Search suitable supported regulatory classes which supports given */
    /* channel index with 40MHz operation if secondary channel offsaet   */
    /* is not equal to SCN                                               */
    for(rc_idx = 0; rc_idx < num_rc; rc_idx++)
    {
        rc_info = rd_info->reg_class_info[rc_idx];

        if(is_ch_supp_in_rc(rc_info, get_current_start_freq(), ch_idx) == BTRUE)
        {
            flag_sec_offset = BTRUE;
            if(flag_sec_offset == BTRUE)
                return get_reg_class_val(rc_idx, reg_domain);
        }
    }

    return INVALID_REG_CLASS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : select_suitable_reg_class                                */
/*                                                                           */
/*  Description   : This function is called to set the suitable Regulatory   */
/*                  Class before starting BSS.                               */
/*                                                                           */
/*  Inputs        : 1) Channel Index                                         */
/*                  2) Secondary Channel Offset                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets suitable Regulatory Class for given   */
/*                  channel index and secondary offset.Before starting BSS   */
/*                  it disables 40MHz operation if no Regulatory Class is    */
/*                  found.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Secondary Channel offset                                 */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 select_suitable_reg_class(UWORD8 ch_idx, UWORD8 sec_offset)
{
    UWORD8 reg_class = 0;

    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return sec_offset;
    }

    if(get_current_reg_domain() == NO_REG_DOMAIN)
    {
		return sec_offset;
	}

    reg_class = get_rc_info_sec_chan_offset(ch_idx, sec_offset);

    if(reg_class == INVALID_REG_CLASS)
    {
        if(sec_offset != SCN)
        {
            PRINTD("Warning:MultiDomain- 40MHz is not allowed \n");
            /* Disable 40MHz operation If Regulatory Class is not found */
            sec_offset = SCN;
            reg_class  = get_rc_info_sec_chan_offset(ch_idx, sec_offset);
        }

    }

    set_current_reg_class(reg_class);

    return sec_offset;
}

#endif  /* MAC_MULTIDOMAIN */
#endif /* BSS_ACCESS_POINT_MODE */

