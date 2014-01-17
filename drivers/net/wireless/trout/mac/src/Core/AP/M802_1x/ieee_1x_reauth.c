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
/*  File Name         : ieee_1x_reauth.c                                     */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of Reauthenticator state  */
/*                      machine according to IEEE 802.1x-REV/D9.             */
/*                                                                           */
/*  List of Functions : reauthenticator_fsm                                  */
/*                      reauth_initialize                                    */
/*                      reauth_reauthenticate                                */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
/* Common includes */
#include "itypes.h"

/* 802.1x includes */
#include "ieee_auth_1x.h"
#include "eapol_key.h"
#include "x_timers_auth.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void reauth_initialize(auth_t* auth_1x);
static void reauth_reauthenticate(auth_t* auth_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reauthenticator_fsm                                   */
/*                                                                           */
/*  Description      : This function implements the reauthentication state   */
/*                     machine. In each of the various states, it checks for */
/*                     the appropriate inputs before switching to next state */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void reauthenticator_fsm(auth_t* auth_1x)
{
    /* This state is executed only at start of the state machine */
    if((auth_1x->global.initialize    == BTRUE)        ||
       (auth_1x->reauth.reAuthEnabled == BFALSE))
    {
        reauth_initialize(auth_1x);
        return;
    }

    switch(auth_1x->reauth.state)
    {
    case RA_INITIALIZE:
    {
        /* Change the state to RA_REAUTHENTICATE after the reauthentication  */
        /* timer expires.                                                    */
        if(auth_1x->global.reAuthWhenTimeout == BTRUE)
        {
            reauth_reauthenticate(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case RA_INITIALIZE: */
    break;

    case RA_REAUTHENTICATE:
    {
        /* Change the state unconditionally to RA_INITIALIZE */
        reauth_initialize(auth_1x);
    } /* End of case RA_REAUTHENTICATE: */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(auth_1x->reauth.state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reauth_initialize                                     */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of the  */
/*                     reauthentication state machine.                       */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void reauth_initialize(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.reauth_stats.i++;
#endif /* DEBUG_STATS */

    auth_1x->reauth.reAuthEnabled = BTRUE;

    /* Start reAuthentication timer */
    start_reAuthWhen_timer(auth_1x, auth_1x->reauth.reAuthPeriod);

    /* Set Reauthentication state */
    auth_1x->reauth.state = RA_INITIALIZE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reauth_reauthenticate                                 */
/*                                                                           */
/*  Description      : This function implements the REAUTHENTICATE state of  */
/*                     the reauthentication state machine.                   */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void reauth_reauthenticate(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.reauth_stats.r++;
#endif /* DEBUG_STATS */

    auth_1x->global.reAuthWhenTimeout = BFALSE;

    auth_1x->global.reAuthenticate = BTRUE;

    /* Set Reauthentication state */
    auth_1x->reauth.state = RA_REAUTHENTICATE;
}

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
