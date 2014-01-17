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
/*  File Name         : ap_management_11e.h                                  */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for 802.11e access point mode.           */
/*                                                                           */
/*  List of Functions : set_ap_ac_param_record                               */
/*                      get_ap_ac_param_record                               */
/*                      set_sta_ac_param_record                              */
/*                      get_sta_ac_param_record                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_WMM

#ifndef AP_MANAGEMENT_11E_H
#define AP_MANAGEMENT_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mib_11e.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8 ac_asoc_uapsd_status;    /* UAPSD status at association */
    UWORD8 ac_current_uapsd_status; /* Current UAPSD status        */
    UWORD8 max_sp_len;              /* Service period              */
} uapsd_status_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    set_wmm_ap_ac_params_config (UWORD8 *val);
extern UWORD8* get_wmm_ap_ac_params_config(void);
extern void    set_wmm_sta_ac_params_config (UWORD8 *val);
extern void    update_sta_info_wmm(void *asoc_entry, UWORD8 *info);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function sets the parameters for the required access category for    */
/* the given AC Parameter Record to be used by the AP                        */
INLINE void set_ap_ac_param_record(UWORD8 *data, UWORD8 ac)
{
    UWORD16 txop = 0;

    /* Format of each AC_** Param Record                 */
    /* ------------------------------------------------- */
    /* | Byte 0:1 | Byte 2  | Byte 3 | Byte 4 | Byte 5 | */
    /* ------------------------------------------------- */
    /* | TXOP     | CWmax   | CWmin  | AIFSN  | ACM    | */
    /* ------------------------------------------------- */

    /* TXOP */
    txop  = (data[0] & 0x00FF);
    txop |= ((UWORD16)data[1] << 8) & 0xFF00;
    mset_QAPEDCATableTXOPLimit(txop, ac);

    /* CWmax */
    mset_QAPEDCATableCWmax(data[2], ac);

    /* CWmin */
    mset_QAPEDCATableCWmin(data[3], ac);

    /* AIFSN */
    mset_QAPEDCATableAIFSN(data[4], ac);

    /* ACM */
    if(data[5] & BIT0)
        mset_QAPEDCATableMandatory(TV_TRUE, ac);
    else
        mset_QAPEDCATableMandatory(TV_FALSE, ac);
}

/* This function updates the given data pointer with the AC Parameter Record */
/* for the given access category being used by the AP                        */
INLINE void get_ap_ac_param_record(UWORD8 *data, UWORD8 ac)
{
    UWORD16 txop = 0;

    /* Format of each AC_** Param Record                 */
    /* ------------------------------------------------- */
    /* | Byte 0:1 | Byte 2  | Byte 3 | Byte 4 | Byte 5 | */
    /* ------------------------------------------------- */
    /* | TXOP     | CWmax   | CWmin  | AIFSN  | ACM    | */
    /* ------------------------------------------------- */

    /* TXOP */
    txop = mget_QAPEDCATableTXOPLimit(ac);
    data[0] = (txop & 0x00FF);
    data[1] = (txop & 0xFF00) >> 8;

    /* CWmax */
    data[2] = (mget_QAPEDCATableCWmax(ac) & 0x000F);

    /* CWmin */
    data[3] = (mget_QAPEDCATableCWmin(ac) & 0x000F);

    /* AIFSN */
    data[4] = mget_QAPEDCATableAIFSN(ac);

    /* ACM */
    if(mget_QAPEDCATableMandatory(ac) == TV_TRUE)
        data[5] |= BIT0;
}

/* This function sets the parameters for the required access category for    */
/* the given AC Parameter Record to be used by STA associated with the AP    */
INLINE void set_sta_ac_param_record(UWORD8 *data, UWORD8 ac)
{
    UWORD16 txop = 0;

    /* Format of each AC_** Param Record                 */
    /* ------------------------------------------------- */
    /* | Byte 0:1 | Byte 2  | Byte 3 | Byte 4 | Byte 5 | */
    /* ------------------------------------------------- */
    /* | TXOP     | CWmax   | CWmin  | AIFSN  | ACM    | */
    /* ------------------------------------------------- */

    /* TXOP */
    txop  = (data[0] & 0x00FF);
    txop |= ((UWORD16)data[1] << 8) & 0xFF00;
    mset_EDCATableTXOPLimit(txop, ac);

    /* CWmax */
    mset_EDCATableCWmax(data[2], ac);

    /* CWmin */
    mset_EDCATableCWmin(data[3], ac);

    /* AIFSN */
    mset_EDCATableAIFSN(data[4], ac);

    /* ACM */
    if(data[5] & BIT0)
        mset_EDCATableMandatory(TV_TRUE, ac);
    else
        mset_EDCATableMandatory(TV_FALSE, ac);
}

/* This function returns the minimum non-zero TXOP limit configured across   */
/* all ACs for the AP. If all TXOP-Limits are zero, then 0xFFFF is returned. */
INLINE UWORD16 get_min_non_zero_txop_wmm_ap(void)
{
    UWORD8 i = 0;
    UWORD16 min_txop = 0xFFFF;
    UWORD16 ac_txop  = 0xFFFF;

    for(i = 0; i < NUM_AC; i++)
    {
        ac_txop = mget_QAPEDCATableTXOPLimit(i);
        if((ac_txop != 0) && (ac_txop < min_txop))
            min_txop = ac_txop;
    }

    return min_txop;
}
#endif /* AP_MANAGEMENT_11E_H */
#endif /* MAC_WMM */
#endif /* BSS_ACCESS_POINT_MODE */
