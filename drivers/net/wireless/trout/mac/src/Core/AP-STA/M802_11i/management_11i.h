/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
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
/*  File Name         : management_11i.h                                     */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      definitions in 802.11i mode                          */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef MANAGEMENT_11I_H
#define MANAGEMENT_11I_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib_11i.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Cipher types defined in 802.11i mode */
typedef enum{ TKIP = 1,
              CCMP = 2
}CIPHER_11I_T;

/* OUI Type used by WPA */
typedef enum{OUITYPE_WPA = 0x01 /* OUI Type WPA */
} OUITYPE_11I_T;

/* Reason codes used in Management frames in 802.11i mode */
typedef enum{ INFO_ELMNT_FAIL = 13,
              MIC_FAIL        = 14,
              HS_4W_TIMEOUT   = 15,
              GRP_HS_TIMEOUT  = 16,
              DIFF_INFO_ELEM  = 17,
              GRP_CP_FAIL     = 18,
              PW_CP_FAIL      = 19,
              INVALID_AKMP    = 20,
              RSN_VER_FAIL    = 21,
              RSN_CAP_FAIL    = 22,
              AUTH_1X_FAIL    = 23
} REASON_CODE_11I_T;

/* Status codes used in Management frames in 802.11i mode */
typedef enum{ INVALID_INFO_ELMNT     = 40,
              INVALID_GRP_CIPHER     = 41,
              INVALID_PW_CIPHER      = 42,
              INVALID_AKMP_CIPHER    = 43,
              UNSUP_RSN_INFO_VER     = 44,
              INVALID_RSN_INFO_CAP   = 45,
              CIPHER_REJ             = 46,
} STATUS_CODE_11I_T;

/* Event types exclusive to 801.11i */
typedef enum{ MISC_AUTH_SND_DEAUTH   = 0x45,
              MISC_SUPP_SND_DEAUTH   = 0x46,
              MISC_AUTH_TKIP_CNT_MSR = 0x47,
              MISC_SUPP_TKIP_CNT_MSR = 0x48,
              MISC_STRT_REKEY        = 0x49,
             /* add new event, and use the expanding value */
              MISC_RSNA_TIMEOUT      = 0x90,
} EVENT_TYPESUBTYPE_11I_T;

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function checks if the given frame information element is the WPA    */
/* information element.                                                      */
INLINE BOOL_T is_wpa_ie(UWORD8 *ie)
{
    UWORD8 *oui = mget_WPAOUI();

    if((ie[0] == IWPAELEMENT) && /* Vendor specific tag */
       (ie[2] == oui[0]) && (ie[3] == oui[1]) && (ie[4] == oui[2]) && /* OUI */
       (ie[5] == OUITYPE_WPA)) /* OUI Type */
       return BTRUE;

    return BFALSE;
}


#endif  /* MANAGEMENT_11I_H */
#endif  /* MAC_802_11I */

