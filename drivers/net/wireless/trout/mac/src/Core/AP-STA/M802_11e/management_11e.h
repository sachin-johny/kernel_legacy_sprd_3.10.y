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
/*  File Name         : management_11e.h                                     */
/*                                                                           */
/*  Description       : This file contains MAC management related functions  */
/*                      and definitions for 802.11e protocol.                */
/*                                                                           */
/*  List of Functions : map_priority_to_edca_ac                              */
/*                      map_edca_ac_to_priority                              */
/*                      get_txq_num_11e                                      */
/*                      is_wmm_info_param_elem                               */
/*                      is_wmm_info_elem                                     */
/*                      is_wmm_param_elem                                    */
/*                      is_wmm_supported                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

#ifndef MANAGEMENT_11E_H
#define MANAGEMENT_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mib.h"
#include "mib_11e.h"
#include "management.h"
#include "transmit.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define AC_PARAM_RECORD_CONFIG_LEN 6
#define AC_PARAM_CONFIG_LEN        24

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum{QOS_OPTION   = 0x200,  /* QoS Option Implemented       */
             APSD         = 0x800,  /* APSD Option Implemented      */
             DELAYED_BA   = 0x4000, /* DelayedBAOptionImplemented   */
             IMMEDIATE_BA = 0x8000  /* ImmediateBAOptionImplemented */
} CAPABILITY_11E_T;

/* Status Codes for Authentication and Association Frames in 802.11e */
typedef enum {UNSPEC_QOS_FAIL           = 32,
              QAP_INSUFF_BANDWIDTH_FAIL = 33,
              POOR_CHANNEL_FAIL         = 34,
              REMOTE_STA_NOT_QOS        = 35,
              REQ_DECLINED              = 37,
              INVALID_REQ_PARAMS        = 38,
              RETRY_NEW_TSPEC           = 39,
              RETRY_TS_LATER            = 47,
              DLS_NOT_SUPP              = 48,
              DST_STA_NOT_IN_QBSS       = 49,
              DST_STA_NOT_QSTA          = 50,
} STATUS_CODE_11E_T;

/* Reason Codes for Deauthentication and Disassociation Frames in 802.11e */
typedef enum {UNSPEC_QOS_REASON      = 32,
              QAP_INSUFF_BANDWIDTH   = 33,
              POOR_CHANNEL           = 34,
              STA_TX_AFTER_TXOP      = 35,
              QSTA_LEAVING_NETWORK   = 36,
              QSTA_INVALID_MECHANISM = 37,
              QSTA_SETUP_NOT_DONE    = 38,
              QSTA_TIMEOUT           = 39,
              QSTA_CIPHER_NOT_SUPP   = 45
} REASON_CODE_11E_T;

/* Access categories for EDCA */
typedef enum {AC_BK = 0,
              AC_BE = 1,
              AC_VI = 2,
              AC_VO = 3,
} EDCA_ACI_T;

/* TID Limits */
typedef enum {EDCA_TID_MAX = 7,
              HCCA_TID_MAX = 15,
              INVALID_TID  = 16
} TID_LIMIT_T;

/* Queue numbers mapped to priorities for EDCA */
typedef enum {AC_VO_Q        = 1, /* EDCA AC VO queue */
              AC_VI_Q        = 2, /* EDCA AC VI queue */
              AC_BE_Q        = 3, /* EDCA AC BE queue */
              AC_BK_Q        = 4, /* EDCA AC BK queue */
              NUM_MAX_EDCA_Q = 5
} EDCA_Q_NUM_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_wmm_enabled;
extern UWORD8 g_wmm_capability[MAX_STA_SUPPORTED];
extern const UWORD8 g_txq_pri_to_num[NUM_TIDS + 1];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 classify_msdu(UWORD8 *esa, UWORD16 eth_type);
extern BOOL_T is_dst_wmm_capable(UWORD8 sta_index);
extern void   initialize_wmm(void);
extern void   update_table_wmm(UWORD8 sta_index, UWORD8 *msa, UWORD16 rx_len,
                               UWORD16 ie_offset);
extern void   update_tx_mib_11e(UWORD8 tid, UWORD8 retry_cnt, UWORD8 cts_fail,
                                UWORD8 rts_succ, UWORD8 frag_num,
                                BOOL_T is_succ);
extern UWORD8* get_wmm_sta_ac_params_config(void);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function maps the priority of the packet to the corresponding EDCA   */
/* access category. The mapping status is returned.                          */
INLINE BOOL_T map_priority_to_edca_ac(UWORD8 priority, UWORD8 *ac)
{
    switch(priority)
    {
    case 0x01:
    case 0x02:
    {
        *ac = AC_BK;
    }
    break;
    case 0x00:
    case 0x03:
    {
        *ac = AC_BE;
    }
    break;
    case 0x04:
    case 0x05:
    {
        *ac = AC_VI;
    }
    break;
    case 0x06:
    case 0x07:
    {
        *ac = AC_VO;
    }
    break;
    default:
    {
        return BFALSE;
    }
    }

    return BTRUE;
}

/* This function maps EDCA access category to the corresponding priority */
INLINE UWORD8 map_edca_ac_to_priority(UWORD8 ac)
{
    switch(ac)
    {
    case AC_BK:
        return PRIORITY_0;
    case AC_BE:
        return PRIORITY_1;
    case AC_VI:
        return PRIORITY_2;
    case AC_VO:
        return PRIORITY_3;
    default:
        return PRIORITY_1;
    }
}

/* This function returns the queue number mapped to the given priority */
INLINE UWORD8 get_txq_num_11e(UWORD8 priority)
{
    return g_txq_pri_to_num[priority];
}

/* This function checks if the given frame information element is the WMM    */
/* parameter or WMM information element.                                     */
INLINE BOOL_T is_wmm_info_param_elem(UWORD8 *ie)
{
    /* --------------------------------------------------------------------- */
    /* WMM Information/Parameter Element Format                              */
    /* --------------------------------------------------------------------- */
    /* | OUI | OUIType | OUISubtype | Version | QoSInfo | OUISubtype based | */
    /* --------------------------------------------------------------------- */
    /* |3    | 1       | 1          | 1       | 1       | ---------------- | */
    /* --------------------------------------------------------------------- */
    if((ie[0] == IWMM) && /* WMM Element ID */
       (ie[2] == 0x00) && (ie[3] == 0x50) && (ie[4] == 0xF2) && /* OUI */
       (ie[5] == 0x02) && /* OUI Type     */
       ((ie[6] == 0x00) || (ie[6] == 0x01)) && /* OUI Sub Type */
       (ie[7] == 0x01)) /* Version field */
       return BTRUE;

    return BFALSE;
}

/* This function checks if the given frame information element is the WMM    */
/* information element.                                                      */
INLINE BOOL_T is_wmm_info_elem(UWORD8 *ie)
{
    if((ie[0] == IWMM) && /* WMM Element ID */
       (ie[2] == 0x00) && (ie[3] == 0x50) && (ie[4] == 0xF2) && /* OUI */
       (ie[5] == 0x02) && /* OUI Type     */
       (ie[6] == 0x00) && /* OUI Sub Type */
       (ie[7] == 0x01)) /* Version field */
       return BTRUE;

    return BFALSE;
}

/* This function checks if the given frame information element is the WMM    */
/* parameter element.                                                        */
INLINE BOOL_T is_wmm_param_elem(UWORD8 *ie)
{
    if((ie[0] == IWMM) && /* WMM Element ID */
       (ie[2] == 0x00) && (ie[3] == 0x50) && (ie[4] == 0xF2) && /* OUI */
       (ie[5] == 0x02) && /* OUI Type     */
       (ie[6] == 0x01) && /* OUI Sub Type */
       (ie[7] == 0x01)) /* Version field */
       return BTRUE;

    return BFALSE;
}

/* This function checks if the given frame information element is the WMM    */
/* TSPEC element.                                                        */
INLINE BOOL_T is_wmm_tspec_elem(UWORD8 *ie)
{
    if((ie[0] == IWMM) && /* WMM Element ID */
       (ie[2] == 0x00) && (ie[3] == 0x50) && (ie[4] == 0xF2) && /* OUI */
       (ie[5] == 0x02) && /* OUI Type     */
       (ie[6] == 0x02) && /* OUI Sub Type */
       (ie[7] == 0x01)) /* Version field */
       return BTRUE;

    return BFALSE;
}

/* This function checks if WMM is supported by parsing the given frame to    */
/* check the presence of WMM information/parameter element.                  */
INLINE BOOL_T is_wmm_supported(UWORD8 *msa, UWORD16 rx_len, UWORD16 ie_offset)
{
    /* Check for WMM information/parameter element */
    while(ie_offset < (rx_len - FCS_LEN))
    {
        if(is_wmm_info_param_elem(msa + ie_offset) == BTRUE)
        {
            return BTRUE;
        }

        ie_offset += (2 + msa[ie_offset + 1]);
    }

    return BFALSE;
}

/* This function updates the given data pointer with the AC Parameter Record */
/* for the given access category to be used by STA associated with the AP    */
INLINE void get_sta_ac_param_record(UWORD8 *data, UWORD8 ac)
{
    UWORD16 txop = 0;

    /* Format of each AC_** Param Record                 */
    /* ------------------------------------------------- */
    /* | Byte 0:1 | Byte 2  | Byte 3 | Byte 4 | Byte 5 | */
    /* ------------------------------------------------- */
    /* | TXOP     | CWmax   | CWmin  | AIFSN  | ACM    | */
    /* ------------------------------------------------- */

    /* TXOP */
    txop = mget_EDCATableTXOPLimit(ac);
    data[0] = (txop & 0x00FF);
    data[1] = (txop & 0xFF00) >> 8;

    /* CWmax */
    data[2] = (mget_EDCATableCWmax(ac) & 0x000F);

    /* CWmin */
    data[3] = (mget_EDCATableCWmin(ac) & 0x000F);

    /* AIFSN */
    data[4] = mget_EDCATableAIFSN(ac);

    /* ACM */
    if(mget_EDCATableMandatory(ac) == TV_TRUE)
        data[5] = 1;
    else
        data[5] = 0;
}

#endif  /* MANAGEMENT_11E_H */

#endif  /* MAC_WMM */

