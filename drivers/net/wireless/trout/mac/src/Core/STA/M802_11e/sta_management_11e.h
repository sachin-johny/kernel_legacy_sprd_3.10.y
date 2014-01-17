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
/*  File Name         : sta_management_11e.h                                 */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for 802.11e station mode.                */
/*                                                                           */
/*  List of Functions : set_ap_wmm_cap                                       */
/*                      is_ap_wmm_cap                                        */
/*                      get_ac_from_wmm_aci                                  */
/*                      update_ac_param_record_sta                           */
/*                      change_wmm_priority_if                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_WMM

#ifndef STA_MANAGEMENT_11E_H
#define STA_MANAGEMENT_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "cglobals_sta.h"
#include "frame_11e.h"
#include "mh.h"

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_ap_wmm_capable;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    update_scan_response_wmm(UWORD8 *msa, UWORD16 len, UWORD16 idx,
                                        UWORD8 dscr_set_index);
extern void    update_params_wmm_sta(UWORD8 *msa, UWORD16 len, UWORD16 index,
                                     UWORD8 sub_type);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function sets the WMM capability of the AP */
INLINE void set_ap_wmm_cap(BOOL_T val)
{
    g_ap_wmm_capable = val;
}

/* This function checks if the AP is WMM capable */
INLINE BOOL_T is_ap_wmm_cap(void)
{
    return g_ap_wmm_capable;
}

/* This function returns the access category given the access category index */
/* as specified in WMM standard.                                             */
INLINE UWORD8 get_ac_from_wmm_aci(UWORD8 aci)
{
    UWORD8 ac = 0;

    for(ac = 0; ac < NUM_AC; ac++)
    {
        if(aci == mget_EDCATableIndex(ac))
            return ac;
    }

    return NUM_AC;
}

/* This function updates the parameters for the required access category for */
/* the given AC Parameter Record                                             */
INLINE void update_ac_param_record_sta(UWORD8 *ac_params)
{
    UWORD8        ac     = 0;
    UWORD8        aci    = 0;
    UWORD8        aifsn  = 0;
    UWORD8        cw_min = 0;
    UWORD8        cw_max = 0;
    UWORD16       txop   = 0;
    TRUTH_VALUE_T acm    = TV_FALSE;

    /* AC_** Parameter Record field               */
    /* ------------------------------------------ */
    /* | Byte 1    | Byte 2        | Byte 3:4   | */
    /* ------------------------------------------ */
    /* | ACI/AIFSN | ECWmin/ECWmax | TXOP Limit | */
    /* ------------------------------------------ */

    /* ACI/AIFSN Field                    */
    /* ---------------------------------- */
    /* | B0:B3 | B4  | B5:B6 | B7       | */
    /* ---------------------------------- */
    /* | AIFSN | ACM | ACI   | Reserved | */
    /* ---------------------------------- */
    aifsn = ac_params[0] & 0x0F;
    acm   = (ac_params[0] & BIT4)?TV_TRUE:TV_FALSE;
    aci   = (ac_params[0] >> 5) & 0x03;

    /* ECWmin/ECWmax Field */
    /* ------------------- */
    /* | B0:B3  | B4:B7  | */
    /* ------------------- */
    /* | ECWmin | ECWmax | */
    /* ------------------- */
    cw_min = ac_params[1] & 0x0F;
    cw_max = ((ac_params[1] & 0xF0) >> 4);

    /* TXOP Limit. The value saved in MIB is in usec while the value to be   */
    /* set in this element should be in multiple of 32us                     */
    txop = ac_params[2] | ((ac_params[3] & 0x00FF) << 8);
    txop = txop << 5;

    /* Get the access category from the ACI for which the parameters need to */
    /* be updated.                                                           */
    ac = get_ac_from_wmm_aci(aci);

    /* If the AC is not valid return */
    if(ac == NUM_AC)
        return;

    /* Update the MIB for the required AC */
    mset_EDCATableCWmin(cw_min, ac);
    mset_EDCATableCWmax(cw_max, ac);
    mset_EDCATableAIFSN(aifsn, ac);
    mset_EDCATableTXOPLimit(txop, ac);
    mset_EDCATableMandatory(acm, ac);
}

/* Check whether admision control is mandatory for the given priority.If,so  */
/* change priority to a lower value which does not require admission control */
INLINE BOOL_T change_wmm_priority_if(wlan_tx_req_t *msg)
{
    UWORD8 priority = 0;
    UWORD8 ac       = 0;

    priority = msg->priority;

    if(map_priority_to_edca_ac(priority, &ac) == BTRUE)
    {
        if(mget_EDCATableMandatory(ac) == TV_TRUE)
        {
            while(ac !=0)
            {
                ac--;
                if(mget_EDCATableMandatory(ac) == TV_FALSE)
                {
                    msg->priority = map_edca_ac_to_priority(ac);
                    return BTRUE;
                }
            }
            return BFALSE;
        }
    }
    return BTRUE;
}

/* This function returns the minimum non-zero TXOP limit configured across   */
/* all ACs for the STA. If all TXOP-Limits are zero, then 0xFFFF is returned */
INLINE UWORD16 get_min_non_zero_txop_wmm_sta(void)
{
    UWORD8 i = 0;
    UWORD16 min_txop = 0xFFFF;
    UWORD16 ac_txop  = 0xFFFF;

    for(i = 0; i < NUM_AC; i++)
    {
        ac_txop = mget_EDCATableTXOPLimit(i);
        if((ac_txop != 0) && (ac_txop < min_txop))
            min_txop = ac_txop;
    }

    return min_txop;
}

#endif /* STA_MANAGEMENT_11E_H */

#endif /* MAC_WMM */
#endif /* IBSS_BSS_STATION_MODE */
