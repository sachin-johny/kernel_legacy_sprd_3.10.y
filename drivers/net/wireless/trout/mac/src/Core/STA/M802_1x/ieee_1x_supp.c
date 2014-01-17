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
/*  File Name         : ieee_1x_supp.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of supplicant PAE state   */
/*                      machine according to IEEE 802.1x-REV/D9.             */
/*                                                                           */
/*  List of Functions : supplicant_fsm                                       */
/*                      supp_logoff                                          */
/*                      supp_disconnected                                    */
/*                      supp_held                                            */
/*                      supp_authenticated                                   */
/*                      supp_connecting                                      */
/*                      supp_acquired                                        */
/*                      supp_authenticating                                  */
/*                      txStart                                              */
/*                      txLogoff                                             */
/*                      txRspId                                              */
/*                      txRspAuth                                            */
/*                                                                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "itypes.h"
#include "cglobals_sta.h"

/* 802.1x includes */
#include "ieee_supp_1x.h"
#include "eapol_key.h"
#include "rsna_supp_km.h"
#include "x_timers_supp.h"
#include "ieee_1x_md5_supp.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void supp_logoff(supp_t* supp_1x);
static void supp_disconnected(supp_t* supp_1x);
static void supp_held(supp_t* supp_1x);
static void supp_authenticated(supp_t* supp_1x);
static void supp_connecting(supp_t* supp_1x);
static void supp_acquired(supp_t* supp_1x);
static void supp_authenticating(supp_t* supp_1x);
static void txStart(supp_t* supp_1x);
static void txLogoff(supp_t* supp_1x);
static void txRspId(supp_t* supp_1x);
static void txRspAuth(supp_t* supp_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supplicant_fsm                                        */
/*                                                                           */
/*  Description      : This function implements the supplicant PAE state     */
/*                     machine. In each of the various states, it checks for */
/*                     the appropriate inputs before switching to the next   */
/*                     state.                                                */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supplicant_fsm(supp_t* supp_1x)
{
    /*  Transition to SS_LOGOFF,SS_DISCONNECTED,SS_HELD,SS_AUTHENTICARED may */
    /*  happen from any state. Hence the conditions for these transitions    */
    /*  are checked here.                                                    */

    if(((supp_1x->supp_pae.userLogoff == BTRUE) &&
        (supp_1x->supp_pae.logoffSent == BFALSE)) &&
        (!((supp_1x->global.initialize == BTRUE) ||
        (supp_1x->global.portEnabled == BFALSE))))
    {
        supp_logoff(supp_1x);
    }

    if((supp_1x->global.initialize == BTRUE) ||
        (supp_1x->global.portEnabled == BFALSE))
    {
        supp_disconnected(supp_1x);
    }

    if((supp_1x->supp_pae.eapFail == BTRUE) &&
        (!((supp_1x->global.initialize == BTRUE) ||
        (supp_1x->global.portEnabled == BFALSE))) &&
        (supp_1x->supp_pae.userLogoff == BFALSE) &&
        (supp_1x->supp_pae.logoffSent == BFALSE))
    {
        supp_held(supp_1x);
    }
    if((supp_1x->supp_pae.eapSuccess == BTRUE) &&
        (!((supp_1x->global.initialize == BTRUE) ||
        (supp_1x->global.portEnabled == BFALSE))) &&
        (supp_1x->supp_pae.userLogoff == BFALSE) &&
        (supp_1x->supp_pae.logoffSent == BFALSE))
    {
 #if 0 /*Enable this if advanced EAP methods are used */
        /* copy the PMK from PSK for EAP-TLS/TTLS/PEAP */
        memcpy(supp_1x->rx_key, mget_RSNAConfigPSKValue(), 32);
 #endif
        supp_1x->global.keyRun       = BTRUE;
        supp_1x->global.keyAvailable = BTRUE;
        supp_authenticated(supp_1x);
#ifdef SUPP_11I
		if(BTRUE == g_int_supp_enable)
    	    start_supp_rsna_fsm((rsna_supp_persta_t *)supp_1x->rsna_ptr);
#endif /* SUPP_11I */
    }

    switch(supp_1x->supp_pae.state)
    {
    case SS_INITIALIZE:
        {
            /* Change the state unconditionally to SS_DISCONNECTED */
            supp_disconnected(supp_1x);
        } /* End of case SS_INITIALIZE: */
        break;
    case SS_LOGOFF:
        {
            if(supp_1x->supp_pae.userLogoff == BFALSE)
            {
                supp_disconnected(supp_1x);
            }
        }

        break;

    case SS_DISCONNECTED:
        {
            supp_connecting(supp_1x);
        }
        break;

    case SS_HELD:
        {
            if(supp_1x->global.heldWhileTimeout == BTRUE)
            {
                supp_connecting(supp_1x);
            }
            else if(supp_1x->supp_pae.reqId == BTRUE)
            {
                supp_acquired(supp_1x);
            }
        }
        break;

    case SSS_CONNECTING:
        {
            if((supp_1x->global.startWhenTimeout == BTRUE) &&
               (supp_1x->supp_pae.startCount >= supp_1x->supp_pae.maxStart))
            {
                supp_authenticated(supp_1x);
            }
            else if(supp_1x->supp_pae.reqId == BTRUE)
            {
                supp_acquired(supp_1x);
            }
            else if((supp_1x->global.startWhenTimeout == BTRUE) &&
                   (supp_1x->supp_pae.startCount < supp_1x->supp_pae.maxStart))
            {
                supp_connecting(supp_1x);
            }
        }
        break;

    case SS_AUTHENTICATED:
        {
            if(supp_1x->supp_pae.reqId == BTRUE)
            {
                supp_acquired(supp_1x);
            }
        }
        break;

    case SS_ACQUIRED:
        {
            if(supp_1x->supp_pae.reqId == BTRUE)
            {
                supp_acquired(supp_1x);
            }
            else if(supp_1x->supp_pae.reqAuth == BTRUE)
            {
                supp_authenticating(supp_1x);
            }
            else if(supp_1x->global.authWhileTimeout == BTRUE)
            {
                supp_connecting(supp_1x);
            }
        }
        break;
    case SS_AUTHENTICATING:
        {
            if(supp_1x->supp_pae.reqAuth == BTRUE)
            {
                supp_authenticating(supp_1x);
            }
            else if(supp_1x->supp_pae.reqId == BTRUE)
            {
                supp_acquired(supp_1x);
            }
            else if(supp_1x->global.authWhileTimeout == BTRUE)
            {
                supp_connecting(supp_1x);
            }
        }
        break;
    default:
        {
            /* Do nothing */
        }

    }

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_initialize                                       */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of the  */
/*                     supplicant state machine.                             */
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

void supp_initialize(supp_t* supp_1x)
{
    supp_1x->global.currentId  = 0;

    /* Set authentication state */
    supp_1x->supp_pae.state    = SS_INITIALIZE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_logoff                                           */
/*                                                                           */
/*  Description      : This function implements the LOGOFF state of the      */
/*                     supplicant PAE state machine.                         */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_logoff(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_LOGOFF;
    txLogoff(supp_1x);

    supp_1x->supp_pae.logoffSent    = BTRUE;
    supp_1x->global.suppPortStatus  = UNAUTHORIZED;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_disconnected                                     */
/*                                                                           */
/*  Description      : This function implements the DISCONNECTED state of    */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_disconnected(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_DISCONNECTED;

    supp_1x->supp_pae.eapSuccess    = BFALSE;
    supp_1x->supp_pae.eapFail       = BFALSE;
    supp_1x->supp_pae.startCount    = 0;
    supp_1x->supp_pae.logoffSent    = BFALSE;
    supp_1x->supp_pae.previousId    = 256;
    supp_1x->global.suppPortStatus  = UNAUTHORIZED;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_held                                             */
/*                                                                           */
/*  Description      : This function implements the HELD state of            */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_held(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_HELD;
    supp_1x->supp_pae.eapFail       = BFALSE;
    supp_1x->supp_pae.eapSuccess    = BFALSE;
    supp_1x->global.suppPortStatus  = UNAUTHORIZED;
    start_heldWhile_timer(supp_1x, supp_1x->supp_pae.heldPeriod);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_connecting                                       */
/*                                                                           */
/*  Description      : This function implements the CONNECTING state of      */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_connecting(supp_t *supp_1x)
{
    /* Set supplicant state */
    supp_1x->supp_pae.state = SSS_CONNECTING;
    supp_1x->supp_pae.startCount++;
    supp_1x->supp_pae.reqId = BFALSE;
    txStart(supp_1x);
    start_startWhen_timer(supp_1x, supp_1x->supp_pae.startPeriod);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_authenticated                                    */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATED state of   */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_authenticated(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_AUTHENTICATED;
    supp_1x->supp_pae.eapSuccess    = BFALSE;
    supp_1x->supp_pae.eapFail       = BFALSE;
    supp_1x->global.suppPortStatus  = AUTHORIZED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_acquired                                         */
/*                                                                           */
/*  Description      : This function implements the ACQUIRED state of        */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_acquired(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_ACQUIRED;
    start_authWhile_timer(supp_1x, supp_1x->supp_pae.authPeriod);
    supp_1x->supp_pae.startCount    = 0;
    supp_1x->supp_pae.reqId         = BFALSE;
    supp_1x->supp_pae.reqAuth       = BFALSE;
    supp_1x->supp_pae.previousId    = supp_1x->global.receivedId;
    txRspId(supp_1x);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_authenticating                                   */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATING state of  */
/*                     the supplicant PAE state machine.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_authenticating(supp_t* supp_1x)
{
    /* Set supplicant PAE state */
    supp_1x->supp_pae.state         = SS_AUTHENTICATING;
    supp_1x->supp_pae.previousId    = supp_1x->global.receivedId;
    supp_1x->supp_pae.reqAuth       = BFALSE;
    start_authWhile_timer(supp_1x, supp_1x->supp_pae.authPeriod);
    txRspAuth(supp_1x);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txStart                                               */
/*                                                                           */
/*  Description      : This function sends an EAPOL frame of type            */
/*                     EAPOL-Start to the Authenticator.                     */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void txStart(supp_t* supp_1x)
{
    UWORD8 *buffer  = 0;
    UWORD16 len     = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);

    if(buffer == NULL)
    {
        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_START, len,
                   ((rsna_supp_persta_t*)(supp_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(supp_1x->addr, buffer, len, BFALSE);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txLogoff                                              */
/*                                                                           */
/*  Description      : This function sends an EAPOL frame of type            */
/*                     EAPOL-Logoff to the Authenticator.                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Supplicant Handle                   */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void txLogoff(supp_t* supp_1x)
{
    UWORD8 *buffer  = 0;
    UWORD16 len     = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);

    if(buffer == NULL)
    {
        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_LOGOFF, len,
                   ((rsna_supp_persta_t*)(supp_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(supp_1x->addr, buffer, len, BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txRspId                                               */
/*                                                                           */
/*  Description      : This function sends an EAPOL frame of type            */
/*                     EAP-packet,containing an EAP Response/Identity packet */
/*                     to the authenticator                                  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Supplicant Handle                   */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void txRspId(supp_t* supp_1x)
{
    UWORD8 *buffer  = 0;
    UWORD8 *eap_ptr = 0;
    UWORD16 len     = 0;
    UWORD16 id_len  = g_supp_username[0];
    UWORD8 *id      = &g_supp_username[1];

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
    eap_ptr[0] = EAP_RESPONSE;
    eap_ptr[1] = supp_1x->global.receivedId;
    eap_ptr[2] = 0x00;
    eap_ptr[3] = 0x05 + id_len;
    eap_ptr[4] = EAP_TYPE_IDENTITY;

    memcpy(&eap_ptr[5], id, id_len);

    /* Set the length of the EAPOL Body as 0x04 */
    len       = 0x05 + id_len;

    /* Prepare the frame 802.1x header    */
    /* This puts the 12 byte eapol header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAP_PACKET, len,
                   ((rsna_supp_persta_t*)(supp_1x->rsna_ptr))->mode_802_11i);
    len   += EAPOL_1X_HDR_LEN;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(supp_1x->addr, buffer, len, BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txRspAuth                                             */
/*                                                                           */
/*  Description      : This function sends an EAPOL frame of type            */
/*                     EAP-packet,containing an EAP Reaponse packet other    */
/*                     than an EAP Response/Identity packet to the           */
/*                     authenticator                                         */
/*                                                                           */
/*  Inputs           :                                                       */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void txRspAuth(supp_t* supp_1x)
{
    UWORD8 eap_type = get_eap_type(supp_1x->eapolEap);
    switch(eap_type)
    {
        case EAP_TYPE_MD5_CHALLENGE:
            /* If we are authenticating for the first time, setup the params */
            if(supp_1x->global.suppPortStatus != AUTHORIZED)
            {
                if(eapmd5_auth_setup(supp_1x) != BTRUE)
                {
                    return;
                }
            }
            break;
        /* All the protocols can be hooked up correspondingly */
        case EAP_TYPE_TLS:
        case EAP_TYPE_ONE_TIME_PWD:
        case EAP_TYPE_GENERIC_TOKEN_CARD:
        case EAP_TYPE_TTLS:
        case EAP_TYPE_MS_CHAP_V2:
        default:
            return;
    }

    {
        UWORD8 *buffer  = 0;
        UWORD8 *eap_ptr = 0;
        UWORD16 len     = 0;

        /* Fetch memory for the EAPOL Packet to be sent to the Remote AP */
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
        eap_ptr[0] = EAP_RESPONSE;
        eap_ptr[1] = supp_1x->global.receivedId;
        eap_ptr[2] = 0x00;

        switch(eap_type)
        {
            default:
            case EAP_TYPE_MD5_CHALLENGE:
                eapmd5_decode_packet(supp_1x->eapolEap,
                                     (eap_ptr + 5), &len,
                                     supp_1x->global.receivedId);
                len += 5;
                eap_ptr[4] = EAP_TYPE_MD5_CHALLENGE;
            break;
        }

        eap_ptr[3] = len;


        /* Prepare the frame 802.1x header    */
        /* This puts the 12 byte eapol header */
        prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAP_PACKET, len,
                       ((rsna_supp_persta_t*)(supp_1x->rsna_ptr))->mode_802_11i);
        len   += EAPOL_1X_HDR_LEN;

        /* Schedule the EAPOL frame for transmission */
        send_eapol(supp_1x->addr, buffer, len, BFALSE);
    }
}

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */

