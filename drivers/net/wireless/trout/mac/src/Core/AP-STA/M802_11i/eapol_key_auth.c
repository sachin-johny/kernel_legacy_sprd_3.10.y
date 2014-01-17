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
/*  File Name         : eapol_key_auth.c                                     */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various functions related to EAPOL Key frames.       */
/*                                                                           */
/*  List of Functions : prepare_kde                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "mib_11i.h"
#include "eapol_key_auth.h"

#ifdef AUTH_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prepare_kde                                           */
/*                                                                           */
/*  Description      : This function prepares the encapsulated KDE format of */
/*                     the required type. The KDE itself is already prepared */
/*                     and set in the buffer.                                */
/*                                                                           */
/*  Inputs           : 1) KDE Type                                           */
/*                     2) KDE Length                                         */
/*                     3) Encapsulated KDE buffer with the KDE set           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void prepare_kde(KDE_DATA_TYPE kde_type, UWORD8 kde_len, UWORD8* kde)
{
    /* --------------------------------------------------------------------- */
    /* | Type(0xDD) | Length  | OUI      | Data Type | Data                | */
    /* --------------------------------------------------------------------- */
    /* | 1 octet    | 1 octet | 3 octets | 1 octet   | (Length - 4) octets | */
    /* --------------------------------------------------------------------- */

    kde[0] = KDE_TYPE;
    kde[1] = kde_len + 4;

    /* The OUI and Data Type are set */
    memcpy(&kde[2], mget_RSNAOUI(), 3);
    kde[5] = kde_type;
}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */

