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
/*  File Name         : auth_frame_11i.h                                     */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      preparation of the various MAC frames specific to    */
/*                      Authentication mode of operation in 802.11i.         */
/*                                                                           */
/*  List of Functions : get_pcip_policy_auth                                 */
/*                      get_auth_policy_auth                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I
#ifndef AUTH_FRAME_11I_H
#define AUTH_FRAME_11I_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "itypes.h"
#include "mib_11i.h"
#include "frame_11i.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern UWORD8 set_rsn_element_auth(UWORD8 *data, UWORD16 index,
                                   UWORD8 mode_802_11i);

/*****************************************************************************/
/* Inline Function Declaration                                               */
/*****************************************************************************/

/* This function extracts the 'Pairwise Cipher Policy type from the frame    */
/* pointer pointing at the beginning of RSN Information Element P/W policy   */
/* Returns the value as contained in the frame popinted by the frame pointer */
/* if the value is valid, returns 0xFF otherwise                             */
INLINE UWORD8 get_pcip_policy_auth(UWORD8* frame, UWORD8 *index, UWORD8 *oui)
{
    UWORD8 idx        = 0;
    UWORD8 pcip_policy = 0;

    *index = frame[0] * 4 + 2;

    if((frame[0] > 1) || (frame[0] == 0))
    {
        pcip_policy = 0xFF;
    }
    else
    {
        idx = 2; /* Skip the number of P/W suites field */

        /* Check the OUI for the P/W cipher policy */
        if(check_oui(&frame[idx], oui) == BFALSE)
        {
            pcip_policy = 0xFF;
        }

        idx += 3; /* Skip the OUI Length */

        pcip_policy = frame[idx];
    }
    return pcip_policy;
}

/* This function extracts the 'Authentication Policy type from the frame     */
/* pointer pointing at the beginning of RSN Information Element's AKM Policy */
/* Returns the value as contained in the frame popinted by the frame pointer */
/* if the value is valid, returns 0xFF otherwise                             */
INLINE UWORD8 get_auth_policy_auth(UWORD8* frame, UWORD8 *index, UWORD8 *oui)
{
    UWORD8 idx        = 0;
    UWORD8 auth_policy = 0;

    *index = frame[0] * 4 + 2;

    if((frame[0] > 1) || (frame[0] == 0))
    {
        auth_policy = 0xFF;
    }
    else
    {
        idx = 2; /* Skip the number of P/W suites field */

        /* Check the OUI for the P/W cipher policy */
        if(check_oui(&frame[idx], oui) == BFALSE)
        {
            auth_policy = 0xFF;
        }

        idx += 3; /* Skip the OUI Length */

        auth_policy = frame[idx];
    }
    return auth_policy;
}

/* Sorts the enabled PCIPs for the requested mode */
INLINE void sort_pcip_4_mode(UWORD8 mode_802_11i, UWORD8 *pcip,
                             UWORD8 pw_count)
{
    UWORD8 temp    = 0;
    UWORD8 i       = 0;

    if(pw_count == 1)
    {
        return;
    }

    switch(mode_802_11i)
    {
        case RSNA_802_11I:
        {
            /* If the default value is not CCMP, swap */
            if(pcip[0] != 0x04)
            {
                for(i = 1; i < pw_count; i++)
                {
                    if(pcip[i] == 0x04)
                    {
                        break;
                    }
                }
                /* CCMP swap */
                if(i != pw_count)
                {
                    temp    = pcip[0];
                    pcip[0] = 0x04;
                    pcip[i] = temp;
                }
            }
        }
        break;
        default:
        case WPA_802_11I:
            /* If the default value is not TKIP, swap */
            if(pcip[0] != 0x02)
            {
                for(i = 1; i < pw_count; i++)
                {
                    if(pcip[i] == 0x02)
                    {
                        break;
                    }
                }
                /* TKIP swap */
                if(i != pw_count)
                {
                    temp    = pcip[0];
                    pcip[0] = 0x02;
                    pcip[i] = temp;
                }
            }
        break;
    }
}


#endif /* AUTH_FRAME_11I_H */

#endif /* AUTH_11I */
#endif /* MAC_802_11I */

