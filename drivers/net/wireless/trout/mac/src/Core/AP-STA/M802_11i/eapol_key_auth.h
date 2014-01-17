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
/*  File Name         : eapol_key_auth.h                                     */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      the EAPOL Key frame related functions.               */
/*                                                                           */
/*  List of Functions : is_mic_fail_report                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef EAPOL_KEY_AUTH_H
#define EAPOL_KEY_AUTH_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "eapol_key.h"
#include "rsna_km.h"

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function checks if the received frame is an remote Mic Failure */
/* report frame                                                        */
INLINE BOOL_T is_mic_fail_report(UWORD8* eapol_key_frame)
{
    if((eapol_key_frame[5] & 0x0F) == 0x0F)
    {
        return BTRUE;
    }

    return BFALSE;
}

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   prepare_kde(KDE_DATA_TYPE kde_type, UWORD8 kde_len, UWORD8* kde);

#endif /* EAPOL_KEY_AUTH_H */

#endif /* MAC_802_11I  */
