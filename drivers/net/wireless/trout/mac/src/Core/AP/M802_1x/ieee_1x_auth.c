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
/*  File Name         : ieee_1x_auth.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of Authenticator state    */
/*                      machine according to IEEE 802.1x-REV/D9.             */
/*                                                                           */
/*  List of Functions : authenticator_fsm                                    */
/*                      auth_initialize                                      */
/*                      auth_disconnected                                    */
/*                      auth_connecting                                      */
/*                      auth_authenticating                                  */
/*                      auth_authenticated                                   */
/*                      auth_aborting                                        */
/*                      auth_held                                            */
/*                      auth_force_auth                                      */
/*                      auth_force_unauth                                    */
/*                      txCannedFail                                         */
/*                      txCannedSuccess                                      */
/*                      auth_force_unauth                                    */
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
#include "rsna_auth_km.h"
#include "eapol_key.h"
#include "x_timers_auth.h"


/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void auth_disconnected(auth_t* auth_1x);
static void auth_connecting(auth_t* auth_1x);
static void auth_authenticating(auth_t* auth_1x);
static void auth_authenticated(auth_t* auth_1x);
static void auth_aborting(auth_t* auth_1x);
static void auth_held(auth_t* auth_1x);
static void auth_force_auth(auth_t* auth_1x);
static void auth_force_unauth(auth_t* auth_1x);
static void txReqId(auth_t* auth_1x);
static void auth_initialize(auth_t* auth_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : authenticator_fsm                                     */
/*                                                                           */
/*  Description      : This function implements the authentication state     */
/*                     machine. In each of the various states, it checks for */
/*                     the appropriate inputs before switching to the next   */
/*                     state.                                                */
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

void authenticator_fsm(auth_t* auth_1x)
{
    /* Change the state to AS_INITAILIZE if portControl is set to AUTO and   */
    /* other variables are appropriately set.                                */
    if(((auth_1x->global.portControl == AUTO)  &&
        (auth_1x->auth_pae.portMode   !=
         auth_1x->global.portControl))          ||
        (auth_1x->global.initialize   == BTRUE) ||
        (auth_1x->global.portEnabled  == BFALSE))
    {
        auth_initialize(auth_1x);
        return;
    }

    /* Transition to AS_FORCE_AUTH and AS_FORCE_UNAUTH may happen from any   */
    /* state. Hence the conditions for these transitions are checked here.   */

    /* Change the state to AS_FORCE_AUTH if portControl is set to            */
    /* FORCE_AUTHORIZED and other variables are appropriately set.           */
    if((auth_1x->global.portControl == FORCE_AUTHORIZED) &&
       (auth_1x->auth_pae.portMode  !=
        auth_1x->global.portControl)                     &&
       ((auth_1x->global.initialize == BFALSE) ||
       (auth_1x->global.portEnabled  == BTRUE)))
    {
        auth_force_auth(auth_1x);
    }
    /* Change the state to AS_FORCE_UNAUTH if portControl is set to          */
    /* FORCE_UNAUTHORIZED and other variables are appropriately set.         */
    else if((auth_1x->global.portControl == FORCE_UNAUTHORIZED) &&
       (auth_1x->auth_pae.portMode  !=
        auth_1x->global.portControl)                       &&
       ((auth_1x->global.initialize == BFALSE) ||
       (auth_1x->global.portEnabled  == BTRUE)))
    {
        auth_force_unauth(auth_1x);
    }


    switch(auth_1x->auth_pae.state)
    {
    case AS_INITIALIZE:
    {
        /* Change the state unconditionally to AS_DISCONNECTED */
        auth_disconnected(auth_1x);
    } /* End of case AS_INITIALIZE: */
    break;

    case AS_DISCONNECTED:
    {
        auth_connecting(auth_1x);
    }
    break;

    case AS_CONNECTING:
    {
        /* Change the state to AS_DISCONNECTED only if reAuthentication      */
        /* count has exceeded limit or an eapolLogoff message is received.   */
        if((auth_1x->auth_pae.eapolLogoff == BTRUE) ||
           ((auth_1x->auth_pae.reAuthCount > auth_1x->auth_pae.reAuthMax) &&
           (auth_1x->global.txWhenTimeout == BTRUE)))
        {
            auth_1x->auth_pae.authEapLogoffsWhileConnecting++;

            auth_disconnected(auth_1x);
        }
        /* If the retransmission limit is not reached and the time out has   */
        /* occured, re transmit the message to the Supplicant                */
        else if((auth_1x->auth_pae.reAuthCount <= auth_1x->auth_pae.reAuthMax)
                && (auth_1x->global.txWhenTimeout  == BTRUE))
        {
            auth_connecting(auth_1x);
        }

        /* Also, In the event of any reauthentication or if the EAPOL Start  */
        /* has been received in this state the ReqId message is resent to    */
        /* the Supplicant state machine                                      */
        else if((auth_1x->auth_pae.reAuthCount <= auth_1x->auth_pae.reAuthMax)
                && ((auth_1x->auth_pae.eapolStart   == BTRUE) ||
                    (auth_1x->global.reAuthenticate == BTRUE)))
        {
            auth_1x->auth_pae.reAuthCount = 0;
            auth_connecting(auth_1x);
        }

        /* Change the state to AS_AUTHENTICATING only if a EAP message is to */
        /* be sent from higher layer and reAuthentication count has not      */
        /* exceeded the limit. Also an indication of Authentication Success/ */
        /* Failure from the higher layers initiates a change of state.       */
        else if((auth_1x->auth_pae.rxRespId == BTRUE)  &&
                (auth_1x->auth_pae.reAuthCount <= auth_1x->auth_pae.reAuthMax))
        {
            auth_1x->auth_pae.authEntersAuthenticating++;

            auth_authenticating(auth_1x);
        }
        /* If a packet was ignored, restart the timer */
        else if((auth_1x->auth_pae.rxRespId   == BTRUE) ||
                (auth_1x->auth_pae.eapolStart == BTRUE))
        {
            auth_1x->auth_pae.authEapLogoffsWhileConnecting++;

            auth_disconnected(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case AS_CONNECTING: */
    break;

    case AS_AUTHENTICATING:
    {
        /* Change the state to AS_AUTHENTICATED only if authentication       */
        /* process successful along with portValid is TRUE (after            */
        /* establishing secure communication channel between authenticator   */
        /* and supplicant - after key exchanges in 802.11).                  */
        if(auth_1x->global.authSuccess == BTRUE &&
           auth_1x->global.portValid   == BTRUE)
        {
            auth_1x->auth_pae.authAuthSuccessesWhileAuthenticating ++;

            auth_authenticated(auth_1x);
        }
        /* Change the state to AS_ABORTING if any of the messages namely,    */
        /* eapolStart, eapolLogoff or authTimeout has arrived.               */
        else if((auth_1x->auth_pae.eapolStart  == BTRUE) ||
                (auth_1x->auth_pae.eapolLogoff == BTRUE) ||
                (auth_1x->global.authTimeout   == BTRUE))
        {
            if(auth_1x->global.authTimeout == BTRUE)
                auth_1x->auth_pae.authAuthTimeoutsWhileAuthenticating++;
            else if(auth_1x->auth_pae.eapolStart == BTRUE)
                auth_1x->auth_pae.authAuthEapStartsWhileAuthenticating++;
            else if(auth_1x->auth_pae.eapolLogoff== BTRUE)
                auth_1x->auth_pae.authAuthEapLogoffWhileAuthenticating++;

            auth_aborting(auth_1x);
        }
        /* Change the state to AS_HELD only if Authentication fails for the  */
        /* port or portValid is FALSE (after key state machine indicates if  */
        /* portValid can be tested.                                          */
        else if((auth_1x->global.authFail   == BTRUE) ||
                ((auth_1x->global.keyDone   == BTRUE) &&
                 (auth_1x->global.portValid == BFALSE)))
        {
            auth_1x->auth_pae.authAuthFailWhileAuthenticating++;

            auth_held(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case AS_AUTHENTICATING: */
    break;

    case AS_AUTHENTICATED:
    {
        /* Change the state to AS_RESTART if eapolStart or reauthenticate    */
        /* message arrives.                                                  */
        if((auth_1x->auth_pae.eapolStart   == BTRUE) ||
           (auth_1x->global.reAuthenticate == BTRUE))
        {
            if(auth_1x->global.reAuthenticate == BTRUE)
                auth_1x->auth_pae.authAuthReauthsWhileAuthenticated++;
            else if(auth_1x->auth_pae.eapolStart == BTRUE)
                auth_1x->auth_pae.authAuthEapStartsWhileAuthenticated++;

            auth_connecting(auth_1x);
        }
        /* Change the state to AS_DISCONNECTED if either eapolLoff message   */
        /* comes or portValid is FALSE.                                      */
        else if((auth_1x->auth_pae.eapolLogoff == BTRUE) ||
                (auth_1x->global.portValid     == BFALSE))
        {
            auth_1x->auth_pae.authAuthEapLogoffWhileAuthenticated++;

            auth_disconnected(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case AS_AUTHENTICATED: */
    break;

    case AS_ABORTING:
    {
        /* Change the state to AS_RESTART if eapolLogoff arrives or          */
        /* authAbort is reset by Authentication backend state machine.       */
        if((auth_1x->auth_pae.eapolLogoff == BFALSE) &&
           (auth_1x->global.authAbort     == BFALSE))
        {
            auth_connecting(auth_1x);
        }
        /* Change the state to AS_DISCONNECTED if either eapolLoff message   */
        /* comes or authAbort is reset by the Authentication backend state   */
        /* machine.                                                          */
        else if((auth_1x->auth_pae.eapolLogoff == BTRUE)  &&
                (auth_1x->global.authAbort     == BFALSE))
        {
            auth_disconnected(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case AS_ABORTING: */
    break;

    case AS_HELD:
    {
        /* Change the state to AS_RESTART after quietWhile timer expires     */
        if(auth_1x->global.quietWhileTimeout == BTRUE)
        {
            auth_connecting(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case AS_HELD: */
    break;

    case AS_FORCE_AUTH:
    {
        /* Remain in AS_FORCE_AUTH state. However, execute the function      */
        /* again only if EAPOL PDU carrying EAPOL-Start is received.         */
        if(auth_1x->auth_pae.eapolStart == BTRUE)
        {
            auth_force_auth(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    }
    break;

    case AS_FORCE_UNAUTH:
    {
        /* Remain in AS_FORCE_UNAUTH state. However, execute the function    */
        /* again only if EAPOL PDU carrying EAPOL-Start is received.         */
        if(auth_1x->auth_pae.eapolStart == BTRUE)
        {
            auth_force_unauth(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    }
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(auth_1x->auth_pae.state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_initialize                                       */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of the  */
/*                     authentication state machine.                         */
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

void auth_initialize(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.i++;
#endif /* DEBUG_STATS */

    auth_1x->auth_pae.portMode = AUTO;
    auth_1x->global.currentId  = 0;

    /* Set authentication state */
    auth_1x->auth_pae.state    = AS_INITIALIZE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_disconnected                                     */
/*                                                                           */
/*  Description      : This function implements the DISCONNECTED state of    */
/*                     the authentication state machine.                     */
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

void auth_disconnected(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.d++;
#endif /* DEBUG_STATS */

    auth_1x->global.reAuthenticate  = BFALSE;
    auth_1x->global.txWhenTimeout   = BFALSE;
    auth_1x->global.authPortStatus  = UNAUTHORIZED;
    auth_1x->auth_pae.reAuthCount   = 0;
    auth_1x->auth_pae.eapolLogoff   = BFALSE;

    /* Set authentication state */
    auth_1x->auth_pae.state = AS_DISCONNECTED;

    /* If this state is not entered after an initialization */
    /* relay a Failure frame                                */
    if(auth_1x->global.currentId != 0)
    {
        /* Set authentication backend state */
        auth_1x->auth_bak.state = ABS_FAIL;
        txCannedFail(auth_1x);
        stop_auth_rsna_fsm((rsna_auth_persta_t*)(auth_1x->rsna_ptr));
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_connecting                                       */
/*                                                                           */
/*  Description      : This function implements the CONNECTING state of      */
/*                     the authentication state machine.                     */
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

void auth_connecting(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.c++;
#endif /* DEBUG_STATS */

    /* Reset the ReAuthentication, EAPOL Start and Timeout flags */
    auth_1x->global.reAuthenticate = BFALSE;
    auth_1x->global.txWhenTimeout  = BFALSE;
    auth_1x->auth_pae.eapolStart   = BFALSE;
    auth_1x->auth_pae.rxRespId     = BFALSE;

    auth_1x->auth_pae.reAuthCount++;

    /* Set authentication state */
    auth_1x->auth_pae.state        = AS_CONNECTING;

    auth_1x->auth_pae.authEntersConnecting++;

    txReqId(auth_1x);

    start_txWhen_timer(auth_1x, auth_1x->auth_pae.txPeriod);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_authenticating                                   */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATING state of  */
/*                     the authentication state machine.                     */
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

void auth_authenticating(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.aing++;
#endif /* DEBUG_STATS */

    auth_1x->auth_pae.eapolStart = BFALSE;
    auth_1x->global.authSuccess  = BFALSE;
    auth_1x->global.authFail     = BFALSE;
    auth_1x->global.authTimeout  = BFALSE;

    /* Indicate to authentication backend state machine about the start of   */
    /* authentication process. NOTE: Authentication backend state machine is */
    /* run after Authentication sate machine.                                */
    auth_1x->global.authStart    = BTRUE;

    auth_1x->global.keyRun       = BFALSE;
    auth_1x->global.keyDone      = BFALSE;

    /* Reset the RespID Flag */
    auth_1x->auth_pae.rxRespId   = BFALSE;

    if(get_eap_type(auth_1x->eapolEap) == EAP_TYPE_IDENTITY)
    {
        update_identity(auth_1x, get_eap_identity(auth_1x->eapolEap),
                        get_eap_identity_len(auth_1x->eapolEap));
    }

    /* Set authentication state */
    auth_1x->auth_pae.state      = AS_AUTHENTICATING;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_authenticated                                    */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATED state of   */
/*                     the authentication state machine.                     */
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

void auth_authenticated(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.a++;
#endif /* DEBUG_STATS */

    auth_1x->global.authPortStatus = AUTHORIZED;
    auth_1x->auth_pae.reAuthCount  = 0;

    /* Set authentication state */
    auth_1x->auth_pae.state = AS_AUTHENTICATED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_aborting                                         */
/*                                                                           */
/*  Description      : This function implements the ABORTING state of        */
/*                     the authentication state machine.                     */
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

void auth_aborting(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.abing++;
#endif /* DEBUG_STATS */

    auth_1x->global.authAbort = BTRUE;
    auth_1x->global.keyRun    = BFALSE;
    auth_1x->global.keyDone   = BFALSE;

    /* Set authentication state */
    auth_1x->auth_pae.state   = AS_ABORTING;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_held                                             */
/*                                                                           */
/*  Description      : This function implements the HELD state of the        */
/*                     authentication state machine.                         */
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

void auth_held(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.h++;
#endif /* DEBUG_STATS */

    auth_1x->global.authPortStatus = UNAUTHORIZED;

    /* Start the quietWhile timer */
    /* Period here is irrelavant  */
    start_quietWhile_timer(auth_1x, auth_1x->auth_pae.quietPeriod);

    auth_1x->auth_pae.eapolLogoff = BFALSE;

    /* Set authentication state */
    auth_1x->auth_pae.state = AS_HELD;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_force_auth                                       */
/*                                                                           */
/*  Description      : This function implements the FORCE_AUTH state of the  */
/*                     authentication state machine.                         */
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

void auth_force_auth(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.fa++;
#endif /* DEBUG_STATS */

    auth_1x->global.authPortStatus = AUTHORIZED;
    auth_1x->auth_pae.portMode     = FORCE_AUTHORIZED;
    auth_1x->auth_pae.eapolStart   = BFALSE;

    txCannedSuccess(auth_1x);

    /* Set authentication state */
    auth_1x->auth_pae.state = AS_FORCE_AUTH;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_force_unauth                                     */
/*                                                                           */
/*  Description      : This function implements the FORCE UNAUTH state of    */
/*                     the authentication state machine.                     */
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

void auth_force_unauth(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_stats.fua++;
#endif /* DEBUG_STATS */

    auth_1x->global.authPortStatus = UNAUTHORIZED;
    auth_1x->auth_pae.portMode     = FORCE_UNAUTHORIZED;
    auth_1x->auth_pae.eapolStart   = BFALSE;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_FAIL;

    txCannedFail(auth_1x);

    /* Set authentication state */
    auth_1x->auth_pae.state = AS_FORCE_UNAUTH;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txCannedFail                                          */
/*                                                                           */
/*  Description      : This function sends an EAP failure message to the     */
/*                     supplicant.                                           */
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

void txCannedFail(auth_t* auth_1x)
{
    UWORD8 *buffer  = 0;
    UWORD8 *eap_ptr = 0;
    UWORD16 len     = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);

    if(buffer == NULL)
    {
        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    /* Set the buffer pointer offseted with the mac and 1x header */
    eap_ptr = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* EAP Packet                                        */
    /*  Code   |   Id   |   Length  |     Data           */
    /*  1 Byte | 1 Byte | 2 bytes   |   Length - 4 Bytes */
    eap_ptr[0] = EAP_FAILURE;
    /* Increment the EAPOL Request ID */
    auth_1x->global.currentId += 1;
    eap_ptr[1] = auth_1x->global.currentId;
    eap_ptr[2] = 0x00;
    eap_ptr[3] = 0x04;

    /* Set the length of the EAPOL Body as 0x04 */
    len       = 0x04;

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAP_PACKET, len,
                   ((rsna_auth_persta_t*)(auth_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(auth_1x->addr, buffer, len, BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txCannedSuccess                                       */
/*                                                                           */
/*  Description      : This function sends an EAP success message to the     */
/*                     supplicant.                                           */
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

void txCannedSuccess(auth_t* auth_1x)
{
    UWORD8 *buffer  = 0;
    UWORD8 *eap_ptr = 0;
    UWORD16 len     = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);

    if(buffer == NULL)
    {
        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    /* Set the buffer pointer offseted with the mac and 1x header */
    eap_ptr = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* EAP Packet                                        */
    /*  Code   |   Id   |   Length  |     Data           */
    /*  1 Byte | 1 Byte | 2 bytes   |   Length - 4 Bytes */
    eap_ptr[0] = EAP_SUCCESS;
    /* Increment the EAPOL Request ID */
    auth_1x->global.currentId += 1;
    eap_ptr[1] = auth_1x->global.currentId;
    eap_ptr[2] = 0x00;
    eap_ptr[3] = 0x04;

    /* Set the length of the EAPOL Body as 0x04 */
    len       = 0x04;

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAP_PACKET, len,
                   ((rsna_auth_persta_t*)(auth_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(auth_1x->addr, buffer, len, BFALSE);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txReqId                                               */
/*                                                                           */
/*  Description      : This function sends an EAP success Request ID to the  */
/*                     supplicant.                                           */
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

void txReqId(auth_t* auth_1x)
{
    UWORD8 *buffer  = 0;
    UWORD8 *eap_ptr = 0;
    UWORD16 len     = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);

    if(buffer == NULL)
    {
        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    /* The EAPOL Buffer is offseted with mac and 1x header length */
    eap_ptr = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* EAP Packet                                        */
    /*  Code   |   Id   |   Length  |     Data           */
    /*  1 Byte | 1 Byte | 2 bytes   |   Length - 4 Bytes */
    eap_ptr[0] = EAP_REQUEST;
    /* Increment the EAPOL Request ID */
    auth_1x->global.currentId += 1;
    eap_ptr[1] = auth_1x->global.currentId;
    eap_ptr[2] = 0x00;
    eap_ptr[3] = 0x05;
    eap_ptr[4] = EAP_TYPE_IDENTITY;
    /* Set the length of the EAPOL Body as 0x04 */
    len       = 0x05;

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAP_PACKET, len,
                   ((rsna_auth_persta_t*)(auth_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(auth_1x->addr, buffer, len, BFALSE);
}

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
