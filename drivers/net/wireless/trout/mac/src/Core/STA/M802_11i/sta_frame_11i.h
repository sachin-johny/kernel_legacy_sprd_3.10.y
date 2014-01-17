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
/*  File Name         : sta_frame_11i.h                                      */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      preparation of the various MAC frames specific to    */
/*                      station mode of operation in 802.11i.                */
/*                                                                           */
/*  List of Functions : get_pcip_policy_sta                                  */
/*                      get_auth_policy_sta                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

#ifndef STA_FRAME_11I_H
#define STA_FRAME_11I_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame_11i.h"


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 set_rsn_element_sta(UWORD8 *data, UWORD16 index,
                                  UWORD8 mode_802_11i);

/*****************************************************************************/
/* Inline Function Declaration                                               */
/*****************************************************************************/


/* This function extracts the 'Pairwise Cipher Policy type from the frame    */
/* pointer pointing at the beginning of RSN Information Element P/W policy   */
/* Returns the value as contained in the frame popinted by the frame pointer */
/* if the value is valid, returns 0 otherwise                                */
INLINE UWORD8 get_pcip_policy_sta(UWORD8 *frame, UWORD8 *index,
                              UWORD8 *pcip_policy, UWORD8 *oui)
{
    UWORD8 i           = 0;
    UWORD8 ret_val     = 0;

    *index  = frame[0] * 4 + 2;

    /* Return a maximum of 3 pairwise policies */
    ret_val = (frame[0] > 3) ? 3 : frame[0];

    for(i = 2; i < *index; i += 4)
    {
        /* Check the OUI for the P/W cipher policy */
        if(check_oui(&frame[i], oui) == BFALSE)
        {
            return 0;
        }

        if(i < 15)
        {
            *pcip_policy++ = frame[i + 3];
        }
    }

    return ret_val;
}

/* This function extracts the 'Authentication Policy type from the frame     */
/* pointer pointing at the beginning of RSN Information Element's AKM Policy */
/* Returns the value as contained in the frame popinted by the frame pointer */
/* if the value is valid, returns 0 otherwise                                */
INLINE UWORD8 get_auth_policy_sta(UWORD8 *frame, UWORD8 *index,
                              UWORD8 *auth_policy, UWORD8 *oui)
{
    UWORD8 i           = 0;
    UWORD8 ret_val     = 0;

    *index  = frame[0] * 4 + 2;

    /* Return a maximum of 3 pairwise policies */
    ret_val = (frame[0] > 3) ? 3 : frame[0];

    for(i = 2; i < *index; i += 4)
    {
        /* Check the OUI for the P/W cipher policy */
        if(check_oui(&frame[i], oui) == BFALSE)
        {
            return 0;
        }

        if(i < 15)
        {
            *auth_policy++ = frame[i + 3];
        }
    }

    return ret_val;
}




#endif /* STA_FRAME_11I_H */

#endif /* MAC_802_11I */

#endif /* IBSS_BSS_STATION_MODE */
