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
/*  File Name         : ieee_1x_sta.c                                        */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of IEEE 802.1x-REV/D9.    */
/*                                                                           */
/*  List of Functions : init_sta_1x                                          */
/*                      stop_sta_1x                                          */
/*                      init_supp_node_1x                                    */
/*                      reset_supp_node_1x                                   */
/*                      free_1x_supp_node                                    */
/*                      supp_onex_eapol                                      */
/*                      supp_onex_fsm_run                                    */
/*                      start_fsm_sta_1x                                     */
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
#include "common.h"

/* 802.1x includes */
#include "ieee_1x.h"
#include "frame.h"
#include "eapol_key.h"
#include "rsna_km.h"
#include "host_if.h"
#include "imem_if.h"
#include "rkmal_auth.h"
#include "receive.h"
#include "x_timers_supp.h"
#include "rsna_supp_km.h"
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/* Convert char to hex */
#define char_2_hex(x) (((x) - '0' <= 9) ? ((x) - '0') : ((x) - 'a' + 10))


/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_supp_username[MAX_SUPP_USERNAME_LEN + 1] = {0};
UWORD8 g_supp_password[MAX_SUPP_PASSWORD_LEN + 1] = {0};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void reset_supp_node_1x(supp_t *supp_1x);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_sta_1x                                           */
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

void init_sta_1x(void)
{
    /* INTEGRATETBD   */
    /* Start TLS/PEAP */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_sta_1x                                           */
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

void stop_sta_1x(void)
{
     /* IntegrateTDB  */
     /* Stop TLS/PEAP */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_supp_node_1x                                     */
/*                                                                           */
/*  Description      : This function runs all the initialization for the     */
/*                     Authenticator 802.1x habdle                           */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                     2) Authentication Client Address                      */
/*                     3) Association Id for the Remote STA                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T init_supp_node_1x(supp_t **supp_1x, UWORD8* rsna_ptr, UWORD16 aid,
                         supp_t *supp_1x_buff)
{
    if(*supp_1x != 0)
    {
        /* An entry exists for this STA. Re-allocation is not required */
    }
    else
    {
        /* Initialize the handle to the given buffer */
        *supp_1x = supp_1x_buff;
    }

    /* Initialize the authentication node state variables. */
    reset_supp_node_1x(*supp_1x);

    /* Copy the 6 byte MAC address of the supplicant */
    memcpy((*supp_1x)->addr, ((rsna_supp_persta_t *)rsna_ptr)->addr, 6);

    /* Update the RSNA Pointer of the Auth 1x Handle */
    (*supp_1x)->rsna_ptr = rsna_ptr;

    /* Update the AID */
    (*supp_1x)->aid = aid;

    /* Return status SUCCESS once the new authentication node is     */
    /* created and initialized.                                      */
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_supp_node_1x                                    */
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

void reset_supp_node_1x(supp_t *supp_1x)
{
    x_supp_global_t *glb = &(supp_1x->global);
    key_rx_t        *key = &(supp_1x->key_rx);
    supp_pae_t      *pae = &(supp_1x->supp_pae);

    supp_1x->eap       = NULL;
    supp_1x->eapolEap  = NULL;

    mem_set(supp_1x->addr, 0, 6);

    /* Initialize global authentication variables */
    glb->authWhile_alarm_handle                 = NULL;
    glb->authWhileTimeout                       = BFALSE;
    glb->heldWhile_alarm_handle                 = NULL;
    glb->heldWhileTimeout                       = BFALSE;
    glb->startWhen_alarm_handle                 = NULL;
    glb->startWhenTimeout                       = BFALSE;
    glb->currentId                              = 0;
    glb->initialize                             = BFALSE;
    glb->keyAvailable                           = BFALSE;
    glb->keyDone                                = BFALSE;
    glb->keyRun                                 = BFALSE;
    glb->keyTxEnabled                           = BFALSE;
    glb->portEnabled                            = BTRUE;
    glb->portValid                              = BFALSE;
    glb->receivedId                             = 0;
    glb->suppPortStatus                         = AUTHORIZED;

    key->state = KR_NO_KEY_RECEIVE;
    key->rxKey  = BFALSE;

    /* Initialize authentication state machine variables */
    pae->state                                  = SS_INITIALIZE;
    pae->userLogoff                             = BFALSE;
    pae->logoffSent                             = BFALSE;
    pae->reqId                                  = BFALSE;
    pae->reqAuth                                = BFALSE;
    pae->eapSuccess                             = BFALSE;
    pae->eapFail                                = BFALSE;
    pae->startCount                             = 3;
    pae->previousId                             = 256;
    pae->authPeriod                             = 30000; /* 30sec */
    pae->heldPeriod                             = 60000; /* 60sec */
    pae->startPeriod                            = 30000; /* 30sec */
    pae->maxStart                               = 3;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : free_1x_supp_node                                     */
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

void free_supp_node_1x(supp_t **supp_1x)
{
    /* stoping all the timers */
    stop_authWhile_timer(*supp_1x);
    stop_heldWhile_timer(*supp_1x);
    stop_startWhen_timer(*supp_1x);

    /* Reseting the 1x handle */
    mem_set(*supp_1x, 0, sizeof(supp_t));
    *supp_1x = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_onex_eapol                                       */
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

void supp_onex_eapol(UWORD8* rx_pkt, UWORD16 data_len, supp_t* onex_ptr)
{
    /* Check the version of the EAPOL Packet. If it is not one, discard */
    /* the received EAPOL Frame                                         */
    if((rx_pkt[0] != EAPOL_VERSION_01) && (rx_pkt[0] != EAPOL_VERSION_02))
    {
        return;
    }

    /* Set the pointer of the packet to be sent to higher layers. It */
    /* is sent by the authentication backend state machine.          */
    onex_ptr->eapolEap           = rx_pkt;
    onex_ptr->eapolEap_frame_len = data_len;

    switch(get_eapol_type(rx_pkt))
    {
        case EAP_PACKET:
        {
            UWORD8 eap_code = get_eap_code(onex_ptr->eapolEap);
            /* Send EAPOL EAP_RESPONSE packet to higher layer                */
            /* (Authentication client)                                       */
            if(eap_code == EAP_REQUEST)
            {
                /* EAPOL Header is no longer required. Is omitted therefore  */
                onex_ptr->eapolEap          += EAPOL_HDR_LEN;
                onex_ptr->eapolEap_frame_len = get_eap_len(onex_ptr->eapolEap);

                /* Extract the recived Id from the EAP recieved frame */
                onex_ptr->global.receivedId     = get_eapol_id(rx_pkt);

                /* Stop the previously running timer */
                stop_authWhile_timer(onex_ptr);

                if(get_eap_type(onex_ptr->eapolEap) == EAP_TYPE_IDENTITY)
                {
                    onex_ptr->supp_pae.reqId   = BTRUE;
                    onex_ptr->supp_pae.reqAuth = BFALSE;
                }
                else
                {
                    onex_ptr->supp_pae.reqId   = BFALSE;
                    onex_ptr->supp_pae.reqAuth = BTRUE;
                }

                /* Fun the state machine */
                supp_onex_fsm_run(onex_ptr);
            }
            else if(eap_code == EAP_SUCCESS)
            {
                /* Stop the previously running timer */
                stop_authWhile_timer(onex_ptr);

                onex_ptr->supp_pae.eapSuccess = BTRUE;

                /* Fun the state machine */
                supp_onex_fsm_run(onex_ptr);
            }
            else if(eap_code == EAP_FAILURE)
            {
                /* Stop the previously running timer */
                stop_authWhile_timer(onex_ptr);

                onex_ptr->supp_pae.eapFail = BTRUE;

                /* Fun the state machine */
                supp_onex_fsm_run(onex_ptr);
            }
        } /* End of case EAPOL_EAP_PACKET: */
        break;

        case EAPOL_START:
        {
            /* Stop the previously running timer */
            stop_authWhile_timer(onex_ptr);

            /* Fun the state machine */
            supp_onex_fsm_run(onex_ptr);
        } /* End of case EAPOL_START: */
        break;

        case EAPOL_LOGOFF:
        {
            /* Stop the previously running timer */
            stop_authWhile_timer(onex_ptr);

            /* Fun the state machine */
            supp_onex_fsm_run(onex_ptr);

        } /* End of case EAPOL_LOGOFF: */
        break;

        case EAPOL_KEY:
        {
            /* Stop the previously running timer */
            stop_authWhile_timer(onex_ptr);

            /* Fun the state machine */
            supp_onex_fsm_run(onex_ptr);

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
/*  Function Name    : supp_onex_fsm_run                                     */
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

void supp_onex_fsm_run(supp_t *supp_1x)
{
    UWORD32 ostate      = 0;
    BOOL_T  statechange = BTRUE;

    while(statechange == BTRUE)
    {
        statechange = BFALSE;

        ostate = supp_1x->supp_pae.state;
        supplicant_fsm(supp_1x);
        statechange |= (ostate != (UWORD32)supp_1x->supp_pae.state);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_fsm_sta_1x                                      */
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

void start_fsm_sta_1x(supp_t *supp_1x)
{
    /* Start/Restart state machine. Note that we run the state       */
    /* machine(s) twice because the initialize handling will         */
    /* prematurely terminate stepping.                               */

    /* Set the global    variable to BTRUE to initialize all the FSMs   */
    supp_1x->global.initialize = BTRUE;

    /* All FSMs are set to their INITIALIZE states */
    supp_onex_fsm_run(supp_1x);

    /* Reset the global variable to BFALSE once FSMs are initialized */
    supp_1x->global.initialize = BFALSE;

    /* All FSMs run from the INITIALIZE state */
    supp_onex_fsm_run(supp_1x);
}


#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
