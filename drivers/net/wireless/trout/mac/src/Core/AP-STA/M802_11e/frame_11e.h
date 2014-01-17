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
/*  File Name         : frame_11e.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the MAC frames in 802.11e standard.                  */
/*                                                                           */
/*  List of Functions : is_qos_bit_set                                       */
/*                      get_dialog_token                                     */
/*                      get_tsinfo                                           */
/*                      get_frame_class_11e                                  */
/*                      get_ack_policy_11e                                   */
/*                      get_tid_value                                        */
/*                      get_eosp                                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

#ifndef FRAME_11E_H
#define FRAME_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "frame.h"
#include "itypes.h"
#include "common.h"
#include "mib_11e.h"
#include "management_11e.h"
#include "transmit.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define QOS_CTRL_FIELD_OFFSET 24
#define QOS_MAC_HDR_LEN       26 /* No Address4 - non-ESS */
#define QOS_CTRL_FIELD_LEN    2
#define AC_PARAM_REC_LEN      4
#define QOS_INFO_LEN          1
#define TSPEC_ELEMENT_LEN     63 /* Includes Element ID & Length */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Different types of Directions */
typedef enum{UPLINK   = 0,
             DOWNLINK = 1,
             BIDIR    = 3,
} TS_DIRECTION_T;

/*****************************************************************************/
/* Extern function declarations                                              */
/*****************************************************************************/

extern UWORD8 set_mac_hdr_11e(UWORD8 *mac_hdr, UWORD8 tid, UWORD8 ack_policy);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function checks if QoS bit is set in the given QoS frame */
INLINE BOOL_T is_qos_bit_set(UWORD8* msa)
{
    return (msa[0] & 0x80)?BTRUE:BFALSE;
}

/* This function returns the dialog token from the given action frame */
INLINE UWORD8 get_dialog_token(UWORD8* msa)
{
    return msa[MAC_HDR_LEN + DIALOG_TOKEN_OFFSET];
}

/* This function returns a pointer to the TSINFO field in the given TSPEC     */
/* element.                                                                   */
INLINE UWORD8 *get_tsinfo(UWORD8* tspec)
{
    if(is_wmm_tspec_elem(tspec) == BTRUE)
        return &tspec[8];

    return &tspec[2];
}

/* This function returns the class of the received 802.11e frame */
INLINE UWORD8 get_frame_class_11e(UWORD8* msa)
{
    FRAMECLASS_T ret_val = CLASS1_FRAME_TYPE;

    switch((TYPESUBTYPE_T)(msa[0] & 0xFC))
    {
    case QOS_CFPOLL:
    case QOS_CFPOLL_ACK:
    case BLOCKACK_REQ:
    case BLOCKACK:
    {
        ret_val = CLASS3_FRAME_TYPE;
    }
    break;
    case QOS_DATA:
    case QOS_DATA_ACK:
    case QOS_DATA_POLL:
    case QOS_DATA_POLL_ACK:
    case QOS_NULL_FRAME:
    {
        if((get_to_ds(msa) == 0) && (get_from_ds(msa) == 0))
        {
            ret_val = CLASS1_FRAME_TYPE;
        }
        else
        {
            ret_val = CLASS3_FRAME_TYPE;
        }
    }
    break;
    default:
    {
        ret_val = CLASS1_FRAME_TYPE;
    }
    break;
    }

    return ret_val;
}

/* This function returns the ACK Policy from the given QoS frame */
INLINE UWORD8 get_ack_policy_11e(UWORD8* msa)
{
    return ((msa[QOS_CTRL_FIELD_OFFSET] & 0x60) >> 5); /* B5 - B6 */
}

/* This function returns the TID value from the given QoS frame */
INLINE UWORD8 get_tid_value(UWORD8* msa)
{
    return (msa[QOS_CTRL_FIELD_OFFSET] & 0x07); /* B0 - B2 */
}

/* This function returns the EOSP bit value from the given QoS frame */
INLINE UWORD8 get_eosp(UWORD8 *msa)
{
    return((msa[QOS_CTRL_FIELD_OFFSET] & BIT4) >> 4); /* B4 */
}

#endif /* FRAME_11E_H */

#endif /* MAC_WMM */
