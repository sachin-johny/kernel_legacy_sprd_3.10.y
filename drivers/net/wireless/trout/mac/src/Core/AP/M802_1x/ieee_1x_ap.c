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
/*  File Name         : ieee_1x_ap.c                                         */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of IEEE 802.1x-REV/D9.    */
/*                                                                           */
/*  List of Functions : init_ap_1x                                           */
/*                      stop_ap_1x                                           */
/*                      init_auth_node_1x                                    */
/*                      reset_auth_node_1x                                   */
/*                      free_auth_node_1x                                    */
/*                      auth_onex_eapol                                      */
/*                      auth_onex_fsm_run                                    */
/*                      start_fsm_ap_1x                                      */
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
#include "common.h"

/* 802.1x includes */
#include "ieee_auth_1x.h"
#include "frame.h"
#include "eapol_key.h"
#include "rsna_km.h"
#include "host_if.h"
#include "imem_if.h"
#include "radius_client.h"
#include "rkmal_auth.h"
#include "receive.h"
#include "x_timers_auth.h"
#include "rsna_auth_km.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/* Convert char to hex */
#define char_2_hex(x) (((x) - '0' <= 9) ? ((x) - '0') : ((x) - 'a' + 10))



/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T (*send_msg_to_serv)(auth_t *auth_1x);/* send_msg_to_serv is used by 1x*/
                                    /* backend state machine to send messages*/
                                    /* to Authenticating server.             */

void (*update_identity)(auth_t *auth_1x, UWORD8* usr_name, UWORD8 len);
                                    /* update_identity is used to update EAP */
                                    /* FSM parameters upon the receipt of    */
                                    /* Identity Response EAP EAPOL Frame     */
UWORD8 g_radius_key[RAD_KEY_MAX_LEN + 1] = {0};
UWORD8 g_radius_serv_addr[6]             = {0};

/* In MWLAN platform, WLAN module is bridged to Ethernet module. During      */
/* transactions between AP and 1x Server, if a packet is received from 		 */
/* 1x server with destination eth address set to WLAN MAC address, bridge    */
/* will not forward the packet to WLAN module. So for transactions between AP*/
/* and 1x server a dummy MAC address differnt from WLAN MAC address is used  */
/* This is also the address retuned in ARP reply too 1x server.				 */
/* As 1x1 FPGA or LSI board does not use any bridge for UDB or SDIO Host i/f,*/
/* This dummy address is not required										 */
UWORD8 g_src_local_eth_addr[6] 	= {0x00, 0x50, 0xc2, 0x5e, 0x10, 0x85};
/* This is the MAC address of the 1x server */
UWORD8 g_1x_server_eth_addr[6]  = {0x00, 0x12, 0x3f, 0x59, 0xca, 0x1b};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void reset_auth_node_1x(auth_t *auth_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_ap_1x                                            */
/*                                                                           */
/*  Description      : This function runs all the initialization for the     */
/*                     Authenticator 802.1x handle                           */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : 1) g_radius_serv_addr                                 */
/*                                                                           */
/*  Processing       : This function runs all the initialization for the     */
/*                     Authenticator 802.1x handle                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void init_ap_1x(void)
{
#ifdef MWLAN
    init_radius_client(mget_StationID(), g_src_local_eth_addr,
                       &g_radius_key[1], g_radius_key[0], g_src_ip_addr,
                       1812, g_radius_serv_addr, 1812, g_1x_server_eth_addr,
                       &send_msg_to_serv, &update_identity);
#else /* MWLAN */
#ifdef ETHERNET_HOST
    init_radius_client(mget_StationID(), get_src_eth_addr(),
#else /* ETHERNET_HOST */
    init_radius_client(mget_StationID(), mget_StationID(),
#endif /* ETHERNET_HOST */
                       &g_radius_key[1], g_radius_key[0], g_src_ip_addr,
                       1812, g_radius_serv_addr, 1812, g_1x_server_eth_addr,
                       &send_msg_to_serv, &update_identity);
#endif /* MWLAN */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_ap_1x                                            */
/*                                                                           */
/*  Description      : This function stops Authenticator 802.1x handles      */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function stops Authenticator 802.1x handles      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_ap_1x(void)
{
     stop_radius_client();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_auth_node_1x                                     */
/*                                                                           */
/*  Description      : This function runs all the initialization for the     */
/*                     Authenticator 802.1x habdle                           */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                     2) Pointer to 11i handle                              */
/*                     3) Association Id for the Remote STA                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T init_auth_node_1x(auth_t **auth_1x, UWORD8* rsna_ptr, UWORD16 aid,
                         auth_t *auth_1x_buff)
{
    if(*auth_1x != 0)
    {
        /* An entry exists for this STA. Clear the entry and redo the inits */
        free_auth_node_1x(auth_1x);
    }

    /* Initialize the handle to the given buffer */
    *auth_1x = auth_1x_buff;

    /* Initialize the authentication node state variables. */
    reset_auth_node_1x(*auth_1x);

    /* Copy the 6 byte MAC address of the supplicant */
    memcpy((*auth_1x)->addr, ((rsna_auth_persta_t *)rsna_ptr)->addr, 6);

    /* Update the RSNA Pointer of the Auth 1x Handle */
    (*auth_1x)->rsna_ptr = rsna_ptr;

    /* Update the AID */
    (*auth_1x)->aid = (UWORD8)aid;

    /* Return status SUCCESS once the new authentication node is     */
    /* created and initialized.                                      */
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_auth_node_1x                                    */
/*                                                                           */
/*  Description      : This function initializes the buffer associated with  */
/*                     the authentication node. This function assumes that   */
/*                     the buffer for the authentication node is already     */
/*                     allocated.                                            */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void reset_auth_node_1x(auth_t *auth_1x)
{
    x_auth_global_t *glb = &(auth_1x->global);
    auth_pae_t      *aut = &(auth_1x->auth_pae);
    key_rx_t        *key = &(auth_1x->key_rx);
    reauth_t        *rea = &(auth_1x->reauth);
    auth_backend_t  *abk = &(auth_1x->auth_bak);
    control_dir_t   *cnt = &(auth_1x->control_dir);

    /* Initialize the members of the Autheticator S/M to null */
    auth_1x->eap        = NULL;
    auth_1x->eap_buffer = NULL;
    auth_1x->eapolEap   = NULL;


    mem_set(auth_1x->addr, 0, 6);

    /* Initialize global authentication variables */
    glb->aWhile_alarm_handle                    = NULL;
    glb->aWhileTimeout                          = BFALSE;
    glb->quietWhile_alarm_handle                = NULL;
    glb->quietWhileTimeout                      = BFALSE;
    glb->reAuthWhen_alarm_handle                = NULL;
    glb->reAuthWhenTimeout                      = BFALSE;
    glb->txWhen_alarm_handle                    = NULL;
    glb->txWhenTimeout                          = BFALSE;
    glb->authAbort                              = BFALSE;
    glb->authFail                               = BFALSE;
    glb->authPortStatus                         = UNAUTHORIZED;
    glb->authStart                              = BFALSE;
    glb->authTimeout                            = BFALSE;
    glb->authSuccess                            = BFALSE;
    glb->currentId                              = 0;
    glb->initialize                             = BFALSE;
    glb->keyAvailable                           = BFALSE;
    glb->keyDone                                = BFALSE;
    glb->keyRun                                 = BFALSE;
    glb->keyTxEnabled                           = BFALSE;
    glb->portControl                            = AUTO;
    glb->portEnabled                            = BTRUE;
    glb->portStatus                             = UNAUTHORIZED;
    glb->portValid                              = BFALSE;
    glb->reAuthenticate                         = BFALSE;
    glb->receivedId                             = 0;

    /* Initialize authentication state machine variables */
    aut->state                                  = AS_INITIALIZE;
    aut->eapolLogoff                            = BFALSE;
    aut->eapolStart                             = BFALSE;
    aut->portMode                               = AUTO;
    aut->reAuthCount                            = 0;
    aut->rxRespId                               = BFALSE;
    aut->quietPeriod                            = 60000;
    aut->reAuthMax                              = 2;
    aut->txPeriod                               = 30000;
    aut->authEntersConnecting                   = 0;
    aut->authEapLogoffsWhileConnecting          = 0;
    aut->authEntersAuthenticating               = 0;
    aut->authAuthSuccessesWhileAuthenticating   = 0;
    aut->authAuthTimeoutsWhileAuthenticating    = 0;
    aut->authAuthFailWhileAuthenticating        = 0;
    aut->authAuthEapStartsWhileAuthenticating   = 0;
    aut->authAuthEapLogoffWhileAuthenticating   = 0;
    aut->authAuthReauthsWhileAuthenticated      = 0;
    aut->authAuthEapStartsWhileAuthenticated    = 0;
    aut->authAuthEapLogoffWhileAuthenticated    = 0;

    /* Initialize Key Receive state machine variables */
    key->state                                  = KR_NO_KEY_RECEIVE;
    key->rxKey                                  = BFALSE;

    /* Initialize Reauthentication Timer state machine */
    rea->state                                  = RA_INITIALIZE;
    rea->reAuthPeriod                           = 3600000; /* ms in a day */
    rea->reAuthEnabled                          = BFALSE;

    /* Initialize authentication backend state machine variables */
    abk->state                                  = ABS_INITIALIZE;
    abk->reqCount                               = 0;
    abk->rxResp                                 = BFALSE;
    abk->aSuccess                               = BFALSE;
    abk->aFail                                  = BFALSE;
    abk->aReq                                   = BFALSE;
    abk->idFromServer                           = 0;
    abk->suppTimeout                            = 30000;
    abk->serverTimeout                          = 30000;
    abk->maxReq                                 = 2;

    abk->backendResponses                       = 0;
    abk->backendAccessChallenges                = 0;
    abk->backendOtherRequestsToSupplicant       = 0;
    abk->backendAuthSuccesses                   = 0;
    abk->backendAuthFails                       = 0;

    /* Initialize Controlled Directions state machine variables */
    cnt->state                                  = CD_FORCE_BOTH;
    cnt->adminControlledDirections              = BOTH;
    cnt->operControlledDirections               = BOTH;
    cnt->operEdge                               = BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : free_auth_node_1x                                     */
/*                                                                           */
/*  Description      : This function releases the buffer associated with the */
/*                     authentication node.                                  */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void free_auth_node_1x(auth_t **auth_1x)
{
    /* Freeing the radius node */
    free_rad_auth_node((*auth_1x)->aid);

    /* stoping all the timers */
    stop_quietWhile_timer(*auth_1x);
    stop_txWhen_timer(*auth_1x);
    stop_reAuthWhen_timer(*auth_1x);
    stop_aWhile_timer(*auth_1x);

    /* Delete the unwanted buffer */
    if((*auth_1x)->eap != NULL)
    {
        pkt_mem_free((*auth_1x)->eap);
    }

    /* Reseting the 1x handle */
    mem_set(*auth_1x, 0, sizeof(auth_t));
    *auth_1x = NULL;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_onex_eapol                                       */
/*                                                                           */
/*  Description      : This function handles messages from MAC which are of  */
/*                     the type EAPOL.                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to EAPOL packet                            */
/*                     2) EAPOL Length                                       */
/*                     3) oneX handle                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void auth_onex_eapol(UWORD8* rx_pkt, UWORD16 data_len, auth_t* onex_ptr)
{
    /* Check the version of the EAPOL Packet. If it is not one, discard */
    /* the received EAPOL Frame                                         */
    if((rx_pkt[0] != EAPOL_VERSION_01) && (rx_pkt[0] != EAPOL_VERSION_02))
    {
        return;
    }

    /* Set the pointer of the packet to be sent to higher layers. It */
    /* is sent by the authentication backend state machine.          */
    onex_ptr->eapolEap           = rx_pkt  ;
    onex_ptr->eapolEap_frame_len = data_len;

    switch(get_eapol_type(rx_pkt))
    {
        case EAP_PACKET:
        {
            /* Send EAPOL EAP_RESPONSE packet to higher layer                */
            /* (Authentication client)                                       */
            if(get_eap_code(onex_ptr->eapolEap) == EAP_RESPONSE)
            {
                /* EAPOL Header is no longer required. Is omitted therefore  */
                onex_ptr->eapolEap          += EAPOL_HDR_LEN;
                onex_ptr->eapolEap_frame_len = get_eap_len(onex_ptr->eapolEap);

                /* Extract the recived Id from the EAP recieved frame */
                onex_ptr->global.receivedId     = get_eapol_id(rx_pkt);

                /* Call the onex State machine only if the Id matches        */
                if(onex_ptr->global.currentId   == onex_ptr->global.receivedId)
                {
                    /* Stop the previously running timer */
                    stop_txWhen_timer(onex_ptr);

                    onex_ptr->auth_pae.rxRespId = BTRUE;
                    onex_ptr->auth_bak.rxResp   = BTRUE;

                    /* Fun the state machine */
                    auth_onex_fsm_run(onex_ptr);
                }
            }
        } /* End of case EAPOL_EAP_PACKET: */
        break;

        case EAPOL_START:
        {
            /* Stop the previously running timer */
            stop_txWhen_timer(onex_ptr);

            onex_ptr->auth_pae.eapolStart = BTRUE;

            /* Fun the state machine */
            auth_onex_fsm_run(onex_ptr);
        } /* End of case EAPOL_START: */
        break;

        case EAPOL_LOGOFF:
        {
            /* Stop the previously running timer */
            stop_txWhen_timer(onex_ptr);

            onex_ptr->auth_pae.eapolLogoff = BTRUE;

            /* Fun the state machine */
            auth_onex_fsm_run(onex_ptr);

        } /* End of case EAPOL_LOGOFF: */
        break;

        default:
        {
        }
        break;
    } /* End of switch(eapol_hdr->pkt_type) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_onex_fsm_run                                     */
/*                                                                           */
/*  Description      : This function runs all the authenticator state        */
/*                     machines till no further state change is possible.    */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void auth_onex_fsm_run(auth_t *auth_1x) //Hugh: may sleep.
{
    UWORD32 ostate      = 0;
    BOOL_T  statechange = BTRUE;

    while(statechange == BTRUE)
    {
        statechange = BFALSE;

        ostate = auth_1x->auth_pae.state;
        authenticator_fsm(auth_1x);
        statechange |= (ostate != (UWORD32)auth_1x->auth_pae.state);

        ostate = auth_1x->auth_bak.state;
        authenticator_backend_fsm(auth_1x);
        statechange |= (ostate != (UWORD32)auth_1x->auth_bak.state);

        ostate = auth_1x->reauth.state;
        reauthenticator_fsm(auth_1x);
        statechange |= (ostate != (UWORD32)auth_1x->reauth.state);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_fsm_ap_1x                                       */
/*                                                                           */
/*  Description      : This function initializes the 802.1x  state machine.  */
/*                     It is called after association request is recevied    */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void start_fsm_ap_1x(auth_t *auth_1x)
{
    /* Start/Restart state machine. Note that we run the state       */
    /* machine(s) twice because the initialize handling will         */
    /* prematurely terminate stepping.                               */

    /* Set the global    variable to BTRUE to initialize all the FSMs   */
    auth_1x->global.initialize = BTRUE;

    /* All FSMs are set to their INITIALIZE states */
    auth_onex_fsm_run(auth_1x);

    /* Reset the global variable to BFALSE once FSMs are initialized */
    auth_1x->global.initialize = BFALSE;

    /* All FSMs run from the INITIALIZE state */
    auth_onex_fsm_run(auth_1x);
}


#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
