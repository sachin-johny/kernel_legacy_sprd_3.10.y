/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wpse_prot_rx.h                                       */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      the receiver WPS registration protocol functions     */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

#ifndef WPS_PROT_RX_H
#define WPS_PROT_RX_H
#define ALLOW_ZERO_AS_VALID_CONN_TYPE
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_prot.h"
#include "wpse_prot_tx.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern const UWORD8 wps_1x_header[ONE_X_PCK_TYPE_OFFSET];
extern const UWORD8 eap_wps_type_vid_vt[EAP_WPS_TYPE_VID_VT_LEN];
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
#ifdef MAC_P2P
extern void reassemble_wsc_ie_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                  UWORD8 *buffer);
extern BOOL_T is_wsc_ie_reassm_req_enr(UWORD8 *msa, UWORD16 rx_len,
                                       UWORD16 index,
                                       UWORD16 *total_wsc_ie_len,
                                       UWORD8 **buffer);
#endif /* MAC_P2P */
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* This function compares two "Authentication Type Flags" and checks if they */
/* have some common capability or not                                        */
/*****************************************************************************/
INLINE STATUS_T wps_find_common_auth_type(UWORD16 auth_type_flags1,
                                UWORD16 auth_type_flags2)
{
    if(auth_type_flags1 & auth_type_flags2 & WPS_AUTH_TYPE_FLAGS_MASK)
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }
}
/*****************************************************************************/
/* This function compares two "Encryption Type Flags" and checks if they have*/
/* some common capability or not                                             */
/*****************************************************************************/
INLINE STATUS_T wps_find_common_encry_type(UWORD16 enc_type_flags1,
                                UWORD16 enc_type_flags2)
{
    if(enc_type_flags1 & enc_type_flags2 & WPS_ENCR_TYPE_FLAGS_MASK)
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }

}
/*****************************************************************************/
/* This function compares two "Connection Type Flags" and checks if they have*/
/* some common capability or not                                             */
/*****************************************************************************/
INLINE STATUS_T wps_find_common_conn_type(UWORD16 con_type_flag1,
                                UWORD16 con_type_flag2)
{
    if(con_type_flag1 & con_type_flag2 & WPS_CONN_TYPE_FLAGS_MASK)
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }
}

#endif /* WPS_PROT_RX_H */

#endif /* INT_WPS_ENR_SUPP */
