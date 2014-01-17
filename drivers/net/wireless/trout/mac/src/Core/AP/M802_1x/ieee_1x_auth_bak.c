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
/*  File Name         : ieee_1x_auth_bak.c                                   */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of Authenticator Backend  */
/*                      state machine according to IEEE 802.1x-REV/D9.       */
/*                                                                           */
/*  List of Functions : authenticator_backend_fsm                            */
/*                      auth_bak_initialize                                  */
/*                      auth_bak_idle                                        */
/*                      auth_bak_request                                     */
/*                      auth_bak_response                                    */
/*                      auth_bak_timeout                                     */
/*                      auth_bak_fail                                        */
/*                      auth_bak_success                                     */
/*                      txReq                                                */
/*                      abortAuth                                            */
/*                      sendRespToServer                                     */
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
#include "rsna_auth_km.h"
#include "host_if.h"

/* Test includes */
#ifdef DEBUG_STATS
#include "test.h"
#endif /* DEBUG_STATS */

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void auth_bak_initialize(auth_t* auth_1x);
static void auth_bak_idle(auth_t* auth_1x);
static void auth_bak_request(auth_t* auth_1x);
static void auth_bak_response(auth_t* auth_1x);
static void auth_bak_timeout(auth_t* auth_1x);
static void auth_bak_fail(auth_t* auth_1x);
static void auth_bak_success(auth_t* auth_1x);
static void txReq(auth_t* auth_1x);
static void abortAuth(auth_t* auth_1x);
static void sendRespToServer(auth_t* auth_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : authenticator_backend_fsm                             */
/*                                                                           */
/*  Description      : This function implements the authentication backend   */
/*                     state machine. In each of the various states, it      */
/*                     checks for the appropriate inputs before switching to */
/*                     the next state.                                       */
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

void authenticator_backend_fsm(auth_t *auth_1x) //Hugh: may sleep.
{
    if((auth_1x->global.portControl != AUTO)  ||
       (auth_1x->global.initialize  == BTRUE) ||
       (auth_1x->global.authAbort   == BTRUE))
    {
        auth_bak_initialize(auth_1x);
        return;
    }

    switch(auth_1x->auth_bak.state)
    {
    case ABS_REQUEST:
    {
        /* Retain the state if timeout has occured and the re-transmission   */
        /* count is less than the maximum number of permissible transmissions*/
        /* This is equivalent to retransmissions.                            */
        if((auth_1x->global.aWhileTimeout == BTRUE) &&
           (auth_1x->auth_bak.reqCount <= auth_1x->auth_bak.maxReq))
        {
            auth_bak_request(auth_1x);
        }
        /* Change the state to ABS_TIMEOUT if supplicant is not responding   */
        /* to requests. The timeout variable is set by the higher layers.    */
        else if((auth_1x->global.aWhileTimeout == BTRUE) &&
                (auth_1x->auth_bak.reqCount > auth_1x->auth_bak.maxReq))
        {
            auth_bak_timeout(auth_1x);
        }
        /* Change the state to ABS_RESPONSE if EAP-Packet is received as     */
        /* indicated by an external entity (rxResp).                         */
        else if(auth_1x->auth_bak.rxResp == BTRUE)
        {
            /* Stop server timeout timer */
            stop_aWhile_timer(auth_1x);

            auth_bak_response(auth_1x);
        }
        else
        {
            /* No state change is required */
            auth_1x->auth_bak.aReq = BFALSE;
        }
    } /* End of case ABS_REQUEST: */
    break;

    case ABS_RESPONSE:
    {
        /* Change the state to ABS_TIMEOUT if server is not responding    */
        /* to requests. The timeout variable is set by the higher layers. */
        if(auth_1x->global.aWhileTimeout == BTRUE)
        {
            auth_bak_timeout(auth_1x);
        }
        /* Change the state to ABS_FAIL if higher layer indicates failure of */
        /* authentication via the aFail variable.                            */
        else if(auth_1x->auth_bak.aFail == BTRUE)
        {
            auth_1x->auth_bak.backendAuthFails++;

            auth_bak_fail(auth_1x);
        }
        /* Change the state to ABS_SUCCESS if higher layer indicates success */
        /* of authentication via the aSuccess variable.                      */
        else if(auth_1x->auth_bak.aSuccess == BTRUE)
        {
            auth_1x->auth_bak.backendAuthSuccesses++;

            auth_bak_success(auth_1x);
        }
        /* Change the state to ABS_REQUEST if higher layer indicates that a  */
        /* frame needs to be sent to supplicant via the aReq variable        */
        else if(auth_1x->auth_bak.aReq == BTRUE)
        {
            auth_1x->auth_bak.backendAccessChallenges++;

            auth_bak_request(auth_1x);
        }
        else
        {
            /* No state change is required */
            auth_1x->auth_bak.rxResp = BFALSE;
        }
    } /* End of case ABS_RESPONSE: */
    break;

    case ABS_SUCCESS:
    case ABS_FAIL:
    case ABS_TIMEOUT:
    {
        /* Unconditionally change the authentication backend state machine   */
        /* to ABS_IDLE.                                                      */
        auth_bak_idle(auth_1x);
    } /* End of case ABS_TIMEOUT: */
    break;

    case ABS_IDLE:
    {
        /* Change the state to ABS_RESPONSE if Authenticator state machine   */
        /* has indicated start of authentication procedure.                  */
        if(auth_1x->global.authStart  == BTRUE)
        {
            auth_bak_response(auth_1x);
        }
        else
        {
            /* No state change is required */
        }
    } /* End of case ABS_IDLE: */
    break;

    case ABS_INITIALIZE:
    {
        /* Change the state unconditionally to ABS_IDLE */
        auth_bak_idle(auth_1x);
    } /* End of case ABS_INITIALIZE: */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(auth_1x->auth_bak.state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_initialize                                   */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of the  */
/*                     authentication backend state machine.                 */
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

void auth_bak_initialize(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.init++;
#endif /* DEBUG_STATS */

    abortAuth(auth_1x);

    auth_1x->global.authAbort  = BFALSE;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_INITIALIZE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_idle                                         */
/*                                                                           */
/*  Description      : This function implements the IDLE state of the        */
/*                     authentication backend state machine.                 */
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

void auth_bak_idle(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.id++;
#endif /* DEBUG_STATS */

    auth_1x->global.authStart        = BFALSE;
    auth_1x->auth_bak.reqCount       = 0;

    /* Delete the unwanted buffer */
    if(auth_1x->eap != NULL)
    {
        pkt_mem_free(auth_1x->eap);
    }
    /* Set authentication backend state */
    auth_1x->auth_bak.state   = ABS_IDLE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_request                                      */
/*                                                                           */
/*  Description      : This function implements the REQUEST state of the     */
/*                     authentication backend state machine.                 */
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

void auth_bak_request(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.req++;
#endif /* DEBUG_STATS */

    auth_1x->global.aWhileTimeout    = BFALSE;
    auth_1x->global.currentId        = auth_1x->auth_bak.idFromServer;

    /* Transmit the received EAP to the supplicant */
    txReq(auth_1x);

    /* Start the timer for the Supplicant response */
    start_aWhile_timer(auth_1x, auth_1x->auth_bak.suppTimeout);

    /* Increment the request transmit counter */
    auth_1x->auth_bak.reqCount++;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_REQUEST;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_response                                     */
/*                                                                           */
/*  Description      : This function implements the RESPONSE state of the    */
/*                     authentication backend state machine.                 */
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

void auth_bak_response(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.rsp++;
#endif /* DEBUG_STATS */

    auth_1x->auth_bak.aReq      = BFALSE;
    auth_1x->auth_bak.aSuccess  = BFALSE;
    auth_1x->global.authTimeout = BFALSE;
    auth_1x->auth_bak.rxResp    = BFALSE;
    auth_1x->auth_bak.aFail     = BFALSE;
    auth_1x->auth_bak.reqCount  = 0;

    /* Send the message to the server */
    sendRespToServer(auth_1x);

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_RESPONSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_timeout                                      */
/*                                                                           */
/*  Description      : This function implements the TIMEOUT state of the     */
/*                     authentication backend state machine.                 */
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

void auth_bak_timeout(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.to++;
#endif /* DEBUG_STATS */

    auth_1x->global.aWhileTimeout = BFALSE;

    if(auth_1x->global.portStatus == UNAUTHORIZED)
    {
        txCannedFail(auth_1x);
    }

    auth_1x->global.authTimeout = BTRUE;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_TIMEOUT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_fail                                         */
/*                                                                           */
/*  Description      : This function implements the FAIL state of the        */
/*                     authentication backend state machine.                 */
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

void auth_bak_fail(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.fail++;
#endif /* DEBUG_STATS */

    auth_1x->global.currentId = auth_1x->auth_bak.idFromServer;

    txCannedFail(auth_1x);

    auth_1x->global.authFail = BTRUE;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_FAIL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_bak_success                                      */
/*                                                                           */
/*  Description      : This function implements the SUCCESS state of the     */
/*                     authentication backend state machine.                 */
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

void auth_bak_success(auth_t* auth_1x)
{
#ifdef DEBUG_STATS
    g_debug_stats.auth_bak_stats.suc++;
#endif /* DEBUG_STATS */

    auth_1x->global.currentId = auth_1x->auth_bak.idFromServer;

    txCannedSuccess(auth_1x);

    auth_1x->global.authSuccess  = BTRUE;

    auth_1x->global.keyRun       = BTRUE;
    auth_1x->global.keyAvailable = BTRUE;

    /* Set authentication backend state */
    auth_1x->auth_bak.state = ABS_SUCCESS;


    /* TxKey */
    /* If key has been enabled, start RSNA FSM */
    start_auth_rsna_fsm((rsna_auth_persta_t*)(auth_1x->rsna_ptr));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txReq                                                 */
/*                                                                           */
/*  Description      : This function sends an EAPOL packet of the type EAP   */
/*                     PACKET to the supplicant that has come from the       */
/*                     higher layers.                                        */
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

void txReq(auth_t* auth_1x)
{
    UWORD8 *buffer  = 0;
    UWORD16 len     = 0;

    if(auth_1x->eap == NULL)
    {
        return;
    }

    /* Set the length of the EAPOL Body as 0x04 */
    len = auth_1x->eap_frame_len;


    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    auth_1x->eap_buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = auth_1x->eap_buffer;

    if(buffer == NULL)
    {
        return;
    }

    memcpy(buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN,
        auth_1x->eap + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN, len);

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
/*  Function Name    : abortAuth                                             */
/*                                                                           */
/*  Description      : This function aborts the authentication process. This */
/*                     may involve sending messages to higher layers and     */
/*                     appropriate release of buffers.                       */
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

void abortAuth(auth_t* auth_1x)
{
    /* Currently not implemented. Will be clear only after implementation of */
    /* authentication client.                                                */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sendRespToServer                                      */
/*                                                                           */
/*  Description      : This function sends the EAP packets to higher layers  */
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

void sendRespToServer(auth_t* auth_1x)
{
    /* Send the EAP packet to the authentication client. The address of the  */
    /* packet is stored in the eapolEap pointer (auth_1x->eapolEap).     */
    if(send_msg_to_serv(auth_1x) != BTRUE)
    {
        auth_1x->global.aWhileTimeout = BTRUE;
    }
}


#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESSS_POINT_MODE */
