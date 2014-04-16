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
/*  File Name         : rsna_supp_km.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of RSNA Key Management.   */
/*                                                                           */
/*  List of Functions : initialize_supp_rsna                                 */
/*                      stop_supp_rsna                                       */
/*                      initialize_supp_rsna_hdl                             */
/*                      start_supp_rsna_fsm                                  */
/*                      free_supp_rsna_km                                    */
/*                      supp_rsn_fsm_run                                     */
/*                      supp_rsn_eapol                                       */
/*                      check_rply_cnt                                       */
/*                      check_eapol_pkt_supp                                 */
/*                      update_join_req_params_11i                           */
/*                      check_security_cap_sta                               */
/*                      check_rsn_capabilities_sta                           */
/*                      update_scan_response_11i                             */
/*                      handle_tbtt_11i_event                                */
/*                      set_grp_key                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "rsna_supp_km.h"
#include "mib_11i.h"
#include "ieee_1x.h"
#include "utils.h"
#include "eapol_key.h"
#include "receive.h"
#include "md5.h"
#include "transmit.h"
#include "frame_11i.h"
#include "mh.h"
#include "ce_lut.h"
#include "management_sta.h"
#include "rkmal_sta.h"
#include "sta_prot_if.h"
#include "cglobals_sta.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

rsna_supp_t g_rsna_supp = {{{0},}};

/*****************************************************************************/
/* Static Functions                                                          */
/*****************************************************************************/
#ifdef SUPP_11I
static BOOL_T  check_rply_cnt(rsna_supp_persta_t *rsna_ptr, BOOL_T copy_rply);
#endif /* SUPP_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_supp_rsna                                  */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after MAC is enabled.                    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_rsna_supp: The pointer to the Supplicant RSN Handle */
/*                                                                           */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_supp_rsna(void)
{
    UWORD8 i = 0;

    /* Reset Parameters for the GTK State machine */
    mem_set(&g_rsna_supp, 0, sizeof(rsna_supp_t));

    /* Initialize the Nonce and the Nonce Counters */
    for(i = 0 ; i < NONCE_SIZE; i++)
    {
        g_rsna_supp.global.Counter[i] = get_random_byte();
    }

    /* Reset the Rekey Alarm Handle and counter measure Alarm Handle */
    g_rsna_supp.global.cntr_msr_alarm_handle = NULL;
    g_rsna_supp.global.cntr_msr_in_progress  = BFALSE;

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_supp_rsna                                        */
/*                                                                           */
/*  Description      : This function stops the RSNA KM state machine.        */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_rsna_supp: Pointer to the RSNA Supplicant           */
/*                                                                           */
/*  Processing       : This function stops the RSNA KM state machine.        */
/*                     It is called before any reset                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_supp_rsna(void)
{
    UWORD8 i = 0;

	TROUT_FUNC_ENTER;
    /* Disable the RNSA Enabled Parameter in MIB */
    mset_RSNAEnabled(TV_FALSE);

    /* Stop the rekey timer */
    stop_supp_cntr_msr_timer();

    /* Free all the RSNA Handles */
    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        if(g_rsna_supp.rsna_ptr[i] != NULL)
        {
            free_supp_rsna_km(&g_rsna_supp.rsna_ptr[i]);
        }
    }

    /* Enable the RNSA Enabled Parameter in MIB after stopping the RSNA FSM */
    mset_RSNAEnabled(TV_TRUE);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_supp_rsna_hdl                              */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Supplicant handle              */
/*                     2) Pairwise policy in use                             */
/*                     3) Group Cip policy in use                            */
/*                     4) Authentication policy is use                       */
/*                     5) Key index for the remote STA                       */
/*                     6) Pointer to the remote sta Address                  */
/*                                                                           */
/*  Globals          : g_rsna_supp: The pointer to the Supplicant RSN Handle */
/*                                                                           */
/*  Processing       : This function initializes the RSNA KM state machine.  */
/*                     It is called after association request is recevied    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Returns BFALSE on failure, BTRUE otherwise            */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T initialize_supp_rsna_hdl(rsna_supp_persta_t **rsna_per_sta_ptr,
                                UWORD8 pcip_policy, UWORD8 grp_policy,
                                UWORD8 auth_policy, UWORD8 key_index,
                                UWORD8* da,
                                rsna_supp_persta_t *rsna_per_sta_ptr_buff)
{
    rsna_supp_persta_t *rsna_ptr    = NULL;
    UWORD8             mode_802_11i = mget_RSNAModeRequested();
    UWORD8             aid          = key_index;
    supp_t             *sta_1x      = NULL;

    /* Check if the RSNA Handle is already in use */
    if(*rsna_per_sta_ptr != NULL)
    {
        rsna_ptr = *rsna_per_sta_ptr;

        /* If the RSNA Handle is already in use - do nothing*/
        stop_11i_supp_timer(rsna_ptr);

        /* Stop the RSNA FSM if already running */
        stop_supp_rsna();

        sta_1x = rsna_ptr->sta_1x;
    }
    else
    {
        /* Initialize the handle to the given buffer */
        rsna_ptr = rsna_per_sta_ptr_buff;
    }

    if(rsna_ptr == NULL)
    {
         return BFALSE;
    }

    mem_set(rsna_ptr, 0, sizeof(rsna_supp_persta_t));

    /* Copy the address and cipher/AKM policies */
    memcpy(rsna_ptr->addr, da, 6);

    /* Reinitialize the RSNA FSM */
    initialize_supp_rsna();

    if((auth_policy == 0x02) && (g_psk_available == BFALSE))
    {
        return BFALSE;
    }
#ifdef SUPP_11I
    else if((auth_policy == 0x01) &&
            (BTRUE == g_int_supp_enable))
    {
        if(init_sec_supp_node_sta((void **)(&(rsna_ptr->sta_1x)),
                                  (UWORD8 *)rsna_ptr, aid,
                                  (void *)(&(rsna_ptr->supp_1x_buff))) != BTRUE)
        {
            free_supp_rsna_km(&rsna_ptr);
            return BFALSE;
        }
    }
#endif /* SUPP_11I */
    else if((BFALSE == g_int_supp_enable) ||
            ((auth_policy == 0x02) && (g_psk_available == BTRUE)))

    {
        /* The parameters are initialized for the PSK FSM */
        /* Reset the 1X handle and related parameters     */
        if(sta_1x == NULL)
        {
            /* Initialize the handle to the given buffer */
            rsna_ptr->sta_1x = &rsna_ptr->supp_1x_buff;
        }
        else
        {
            rsna_ptr->sta_1x = sta_1x;
        }

        if(rsna_ptr->sta_1x == NULL)
        {
            free_supp_rsna_km(&rsna_ptr);
            return BFALSE;
        }

        mem_set(rsna_ptr->sta_1x, 0, sizeof(supp_t));

        /* Reset the parameters for PSK */
        rsna_ptr->sta_1x->global.keyRun       = BTRUE;
        rsna_ptr->sta_1x->global.keyAvailable = BTRUE;
    }

    /* Setting the key length for the Pairwise cipher suite */
    if(pcip_policy == 0x02)
    {
        /* For TKIP the key length is 256 */
        rsna_ptr->key_length = 32;
    }
    else if(pcip_policy == 0x04)
    {
        /* The Pairwise cipher suite is 128 */
        rsna_ptr->key_length = 16;
    }

    /* Setting the key length for the Group cipher suite */
    if(grp_policy == 0x02)
    {
        /* For TKIP the key length is 256 */
        rsna_ptr->gkey_length = 32;
    }
    else if(grp_policy == 0x04)
    {
        /* The Pairwise cipher suite is 128 */
        rsna_ptr->gkey_length = 16;
    }

    rsna_ptr->pcip_policy  = pcip_policy;
    rsna_ptr->grp_policy   = grp_policy;
    rsna_ptr->auth_policy  = auth_policy;
    rsna_ptr->mode_802_11i = mode_802_11i;

    /* Reset the FSM Related parameters */
    rsna_ptr->Init                    = BFALSE;
    rsna_ptr->TimeoutEvt              = BFALSE;
    rsna_ptr->DeauthenticationRequest = BFALSE;
    rsna_ptr->AuthenticationRequest   = BFALSE;
    rsna_ptr->AuthenticationFailed    = BFALSE;
    rsna_ptr->EAPOLKeyReceived        = BFALSE;
    rsna_ptr->IntegrityFailed         = BFALSE;
    rsna_ptr->MICVerified             = BFALSE;
    rsna_ptr->send_deauth             = BFALSE;
    rsna_ptr->deauth_reason           = UNSPEC_REASON;

    rsna_ptr->keycount                   = 0;
    rsna_ptr->sta_1x->global.portValid   = BFALSE;
    rsna_ptr->sta_1x->global.portEnabled = BFALSE;

    /* Key version is the field which determines type of the encryption used */
    if((rsna_ptr->pcip_policy == 0x02) || (rsna_ptr->pcip_policy == 0x00))
    {
        rsna_ptr->key_version = 1;
    }
    else if(rsna_ptr->pcip_policy == 0x04)
    {
        /* For CCMP/AES, the key type is set to 2 */
        rsna_ptr->key_version = 2;
    }

    /* Set the GTK for the BroadCast packets */
    rsna_ptr->alarm_handle          = NULL;

    /* Copy the key index: 0 if new RSNA Handle or the old value from */
    /* previous handle                                                */
    rsna_ptr->key_index             = key_index;

    if(key_index)
    {
        g_rsna_supp.rsna_ptr[key_index - 1] = rsna_ptr;
    }
    else
    {
        return BFALSE;
    }


    *rsna_per_sta_ptr               = rsna_ptr;

    /* Setting the Init Condition to true */
    rsna_ptr->Init          = BTRUE;
    rsna_ptr->GN            = UNUSED_GTK_ID;
    rsna_ptr->last_key_num  = UNUSED_GTK_ID;

    rsna_ptr->tbtt_count     = 0;
    rsna_ptr->fsm_start_pend = BTRUE;

#ifdef SUPP_11I
    /* Kickstart the Supp FSM so that it reaches the Idle state */
    if(BTRUE == g_int_supp_enable)
        supp_rsn_fsm_run(NULL);
#endif /* SUPP_11I */

    return BTRUE;
}

#ifdef SUPP_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_supp_rsna_fsm                                   */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after association response is recevied   */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Supplicant handle              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function initializes the RSNA KM state machine.  */
/*                     It is called after association response is recevied   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void start_supp_rsna_fsm(rsna_supp_persta_t *rsna_ptr)
{
    /* Setting the Authentication Condition to true */
    rsna_ptr->AuthenticationRequest = BTRUE;

    /* Running the FSM to move the state machine to the initial states */
    supp_rsn_fsm_run(rsna_ptr);

    /* Start the timer for 11i handshake timeout for 5 sec */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        start_mgmt_timeout_timer(g_11i_protocol_timeout);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_supp_rsna_fsm                                    */
/*                                                                           */
/*  Description      : This function halts the RSNA KM state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Supplicant handle              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function halts the RSNA KM state machine.        */
/*                     It is called after dissociation request is recevied   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_supp_rsna_fsm(rsna_supp_persta_t *rsna_ptr)
{
    if(rsna_ptr != NULL)
    {
        /* Stop any associated Timer with this RSNA Suite */
        stop_11i_supp_timer(rsna_ptr);

        /* Setting the Init Condition to true */
        rsna_ptr->DeauthenticationRequest = BTRUE;

        /* Running the FSM to move the state machine to the initial states */
        supp_rsn_fsm_run(rsna_ptr);

        /* Remove the Key index from the CE-LUT */
        machw_ce_del_key(rsna_ptr->key_index);
    }
}
#endif /* SUPP_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : free_supp_rsna_km                                     */
/*                                                                           */
/*  Description      : This function frees the RSNA KM state machine handle  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Supplicant handle              */
/*                                                                           */
/*  Globals          : g_rsna_supp: The pointer to the Supplicant RSN Handle */
/*                                                                           */
/*  Processing       : This function frees the RSNA KM state machine handle  */
/*                     It is called when the connection is dissolved         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void free_supp_rsna_km(rsna_supp_persta_t **rsna_ptr)
{
    UWORD16 index = 0;

    if(*rsna_ptr == NULL)
    {
        return;
    }

    if((*rsna_ptr)->key_index != 0)
    {
        /* Remove the RSNA Stats from the MIB */
        mfree_RSNAStats((*rsna_ptr)->key_index);

        /* Remove the RSNA Entry from the CE LUT */
        machw_ce_del_key((*rsna_ptr)->key_index);

        index = (*rsna_ptr)->key_index;
    }

    /* Stop any associated Timer with this RSNA Suite */
    stop_11i_supp_timer(*rsna_ptr);

    /* Set Deauthentication Request to True */
    (*rsna_ptr)->DeauthenticationRequest = BTRUE;

#ifdef SUPP_11I
    /* Run the FSM over the Deauthenticatin request */
    if(BTRUE == g_int_supp_enable)
        supp_rsn_fsm_run(*rsna_ptr);
#endif /* SUPP_11I */

    /* If PSK is used by the remote STA, free the 1X handle directly */
    /* else invoke the 1X handle freeing                             */
    if((*rsna_ptr)->auth_policy == 0x02)
    {
        /* Free allocated memory from local pool for the 1X Handle */
        if((*rsna_ptr)->sta_1x != NULL)
        {
             (*rsna_ptr)->sta_1x = NULL;
        }
    }
    else
    {
        free_sec_supp_node_sta((void **)(&((*rsna_ptr)->sta_1x)));
    }

    /* Reset the pointers to this State machine handle */
    *rsna_ptr                   = NULL;

    if(index)
    {
        g_rsna_supp.rsna_ptr[index - 1] = NULL;
    }

    return;
}

#ifdef SUPP_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_rsn_fsm_run                                      */
/*                                                                           */
/*  Description      : This function runs all the authenticator state        */
/*                     machines till no further state change is possible.    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Supplicant handle              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function runs all the authenticator state        */
/*                     machines till no further state change is possible.    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_rsn_fsm_run(rsna_supp_persta_t *rsna_ptr)
{
    UWORD32 ostate      = 0;
    BOOL_T  statechange = BTRUE;

    while(statechange == BTRUE)
    {
        statechange = BFALSE;

        if(rsna_ptr !=NULL)
        {
            ostate = rsna_ptr->rsna_supp_4way_state;
            rsna_supp_4way_fsm(rsna_ptr);
            statechange |= (ostate != (UWORD32)rsna_ptr->rsna_supp_4way_state);
        }
    }
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_rsn_eapol                                        */
/*                                                                           */
/*  Description      : This function calls the authenticator state m/c       */
/*                     for the received EAPOL packet                         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the received EAPOL Frame                */
/*                     2) The length of the received frame                   */
/*                     3) Pointer to the Supplicant Handle                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function calls the authenticator state m/c       */
/*                     for the received EAPOL packet                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void supp_rsn_eapol(UWORD8* rx_pkt, UWORD16 len, rsna_supp_persta_t *rsna_ptr)
{
    rsna_ptr->eapol_key_frame   = rx_pkt;
    rsna_ptr->eapol_frame_len   = len;

    /* Check the version of the EAPOL Packet. If it is not one, discard */
    /* the received EAPOL Frame                                         */
    if((rsna_ptr->mode_802_11i == RSNA_802_11I) ||
       (rsna_ptr->mode_802_11i == WPA_802_11I))
    {
        if((rx_pkt[0] != EAPOL_VERSION_02) && (rx_pkt[0] != EAPOL_VERSION_01))
        {
            return;
        }
    }

    switch (get_eapol_type(rx_pkt))
    {
        case EAPOL_START:
        {
            break;
        }
        case EAPOL_KEY:
        {
            /* Check the Descriptor type of the EAPOL Packet. If it does not */
            /* conform to the necessary type drop the packet                 */
            if(rsna_ptr->mode_802_11i == RSNA_802_11I)
            {
                if(rx_pkt[EAPOL_HDR_LEN] != IEE80211_KEY_DESCRIPTOR_TYPE)
                {
                    break;
                }
            }
            else if(rsna_ptr->mode_802_11i == WPA_802_11I)
            {
                if(rx_pkt[EAPOL_HDR_LEN] != WPA_KEY_DESCRIPTOR_TYPE)
                {
                    break;
                }
            }
            rsna_ptr->EAPOLKeyReceived = BTRUE;

            /* The timer is stopped here and restared if the frame is */
            /* erroneous or discarded                                 */
            stop_11i_supp_timer(rsna_ptr);

            supp_rsn_fsm_run(rsna_ptr);

            /* If the deauthentication bit is set, send Deauth frame */
            if(rsna_ptr->send_deauth == BTRUE)
            {
                rsna_send_deauth((UWORD8 *)rsna_ptr, 1);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_rply_cnt                                        */
/*                                                                           */
/*  Description      : This procedure verifies EAPOL REPLAY in the rxed frame*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                     2) Copy flag for updating the local Replay counter    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : Extracts the replay counter from the received packet  */
/*                     and compares it with the transmitted counter          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True if the counters match; False otherwise           */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_rply_cnt(rsna_supp_persta_t *rsna_ptr, BOOL_T copy_rply)
{
    BOOL_T ret_val = BFALSE;
    UWORD8 rply_rcvd[REPLAY_COUNTER_SIZE];
    UWORD8 *eapol_buffer;

    /* Set the buffer and length to appropriate values */
    eapol_buffer = rsna_ptr->eapol_key_frame;

    /* Extract the replay counter from the recevied EAPOL Frame */
    memcpy(rply_rcvd, get_rply_cnt(eapol_buffer), REPLAY_COUNTER_SIZE);

    /* Compare the Replay Counter Values                                */
    if(memcmp(rply_rcvd, rsna_ptr->ReplayCount, REPLAY_COUNTER_SIZE) >= 0)
    {
        ret_val = BTRUE;
    }

    /* Note that this value is updated only after the MIC is verified */
    /* in message-3                                                   */
    if(copy_rply == BTRUE)
    {
        memcpy(rsna_ptr->ReplayCount, rply_rcvd, REPLAY_COUNTER_SIZE);
    }

    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_eapol_pkt_supp                                  */
/*                                                                           */
/*  Description      : This procedure verifies EAPOL received frame.         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The EAPOL Frame is checked for its correctness in     */
/*                     1) Key information                                    */
/*                     2) Replay counter                                     */
/*                     3) RSC Value                                          */
/*                     4) Key IV value                                       */
/*                     4) Key ID value                                       */
/*                     5) Key Length                                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True if all fields check passes; False otherwise      */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_eapol_pkt_supp(rsna_supp_persta_t *rsna_ptr)
{
    UWORD8 *eapol_buffer = 0;
    UWORD8 index         = 0;
    UWORD8 desc_version  = 0;
    UWORD8 key_type      = 0;
    UWORD8 key_index     = 0;
    BOOL_T copy_rply     = BFALSE;

    /* Set the buffer and length to appropriate values */
    eapol_buffer = rsna_ptr->eapol_key_frame;

    index += EAPOL_HDR_LEN; /* Skip the EAPOL Header   */

    index += KEY_DESCRIPTOR_SIZE; /* Skip the desciptor type field */

    /* Verify the Key information field */
    desc_version = eapol_buffer[index + 1] & 0x07;
    if(desc_version != rsna_ptr->key_version)
    {
        /* If the versions mismatch, discard the packet */
        return BFALSE;
    }

    key_type  = get_k(eapol_buffer);
    key_index = (eapol_buffer[index + 1] & 0x30) >> 4;

    /* The ACK bit must be set */
    if((eapol_buffer[index + 1] & 0x80) == 0x00)
    {
        rsna_ptr->AckReq = BFALSE;
        return BFALSE;
    }
    rsna_ptr->AckReq = BTRUE;

    /* Reset the State of the EAPOL procedure */
    rsna_ptr->State = UNKNOWN;

    /* If MIC Bit is set, verify the EAPOL Mic */
    if((eapol_buffer[index] & 0x01) == 0x01)
    {
        if(verifyMIC(eapol_buffer, rsna_ptr->eapol_frame_len,
                    rsna_ptr->PTK, rsna_ptr->key_version) == BFALSE)
        {
            rsna_ptr->MICVerified = BFALSE;
            rsna_ptr->State       = FAILED;
            return BFALSE;
        }
        else
        {
            /* If the MIC Detection passes, set the flag */
            /* This will copy the received EAPOL Replay  */
            /* Count to the locally maintained copy      */
            rsna_ptr->MICVerified = BTRUE;
            rsna_ptr->State       = MICOK;
            copy_rply = BTRUE;
        }
    }
    else
    {
/* Belkin AP sets Non-Zero MIC in the first EAPOL frame. Since the Key MIC Flag */
/* is Zero there is no need to check for non-zero MIC. But as per the standard  */
/* this MIC key field should be zero for the first frame in 4-Way HS            */
#if 0
        /* Even if the MIC is absent the field is verified and if all zeroes */
        /* are found a MIC Verified Flag is set                              */
        /* Otherwise the MIC and State Flags are reset                       */
        for(i = (MIC_OFFSET + EAPOL_HDR_LEN);
            i < (MIC_OFFSET + EAPOL_HDR_LEN + MIC_SIZE); i++)
        {
            if(eapol_buffer[i] != 0x00)
            {
                rsna_ptr->MICVerified = BFALSE;
                rsna_ptr->State       = FAILED;
                return BFALSE;
            }
        }
#endif /* 0 */
        rsna_ptr->MICVerified = BTRUE;
    }

    index += KEY_INFO_SIZE; /* Skip the Key info field */

    /* Verify the Key length of a pairwise key */
    if(key_type == P)
    {
        /* key index of PTK has to be zero for WPA */
        if(rsna_ptr->mode_802_11i == WPA_802_11I)
        {
        if(key_index != 0)
        {
            return BFALSE;
        }
        }

        /* Verfiy the key length */
        if(eapol_buffer[index + 1] != rsna_ptr->key_length)
        {
            return BFALSE;
        }
    }

    /* Verify the Key length of a group key */
    if(key_type == G)
    {
        /* key index of GTK has to be non-zero for WPA */
        if(rsna_ptr->mode_802_11i == WPA_802_11I)
        {
        if(key_index == 0)
        {

            return BFALSE;
        }
        }
    }

    index += KEY_LENGTH_SIZE; /* Skip the Key length field */

    /* Check the replay counter */
    if(check_rply_cnt(rsna_ptr, copy_rply) == BFALSE)
    {
        return BFALSE;
    }

    return BTRUE;
}
#endif /* SUPP_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_join_req_params_11i                            */
/*                                                                           */
/*  Description      : This function 802.11I related MIB parameters          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the BSS Join Descriptor                 */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function 802.11I related MIB parameters from the */
/*                     saved values in BSS descriptor. This also resloves any*/
/*                     protocols that conflict with selected Pairwaise Policy*/
/*                     selected. Example:Disabling of HT if TKIP is selected */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void update_join_req_params_11i(void *ptr)
{
    bss_dscr_t *bss_dscr = (bss_dscr_t *)ptr;

    /* If RSNA is disabled, no need to update 11i parameters */
    if(mget_RSNAEnabled() != TV_TRUE)
        return;

    /* Update other protocol related decisions based on the chosen pairwise  */
    /* Cipher type.                                                          */
    update_pcip_policy_prot_sta(bss_dscr->pcip_policy_match);

    /* Adopt to the authenticators's group policy at STA                     */
    if((mget_RSNAConfigGroupCipher() != bss_dscr->grp_policy_match))
    {
        mset_RSNAConfigGroupCipher(bss_dscr->grp_policy_match);
        mset_RSNAGroupCipherSelected(bss_dscr->grp_policy_match);
    }

    /* Update the 11I MIB parameters from RSN/WPA-IE policy matches */
    mset_RSNAAuthenticationSuiteRequested(bss_dscr->auth_policy_match);
    mset_RSNAPairwiseCipherRequested(bss_dscr->pcip_policy_match);
    mset_RSNAGroupCipherRequested(bss_dscr->grp_policy_match);
    mset_RSNAModeRequested(bss_dscr->mode_802_11i);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_security_cap_sta                                */
/*                                                                           */
/*  Description      : This function checks if the WPA/RSN capabilities      */
/*                     matches with that configured in the device.           */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Scan Descriptor                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This verifies the RSN/WPA capabilities in scan        */
/*                     response with that configured on the device. On match */
/*                     configures saves all the matched policies and settings*/
/*                     in the BSS descriptor and returns BTRUE else BFASLE   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE: Security Capability Match                      */
/*                     BFALSE: Otherwise                                     */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_security_cap_sta(bss_dscr_t *bss_dscr, UWORD8 mode_802_11i)
{
    UWORD8 i = 0;
    UWORD8 pcip_policy_match = 0;
    UWORD8 grp_policy_match  = 0;
    UWORD8 auth_policy_match = 0;
    UWORD8 *pcip_policy = 0;
    UWORD8 *auth_policy = 0;
    UWORD8 grp_policy  = 0;

    if(!(bss_dscr->mode_802_11i & mode_802_11i))
        return BFALSE;

    if((mode_802_11i != RSNA_802_11I) && (mode_802_11i != WPA_802_11I))
        return BFALSE;

    if(mode_802_11i == RSNA_802_11I)
    {
        pcip_policy = bss_dscr->rsn_pcip_policy;
        auth_policy = bss_dscr->rsn_auth_policy;
        grp_policy  = bss_dscr->rsn_grp_policy;
    }
    else
    {
        pcip_policy = bss_dscr->wpa_pcip_policy;
        auth_policy = bss_dscr->wpa_auth_policy;
        grp_policy  = bss_dscr->wpa_grp_policy;
    }

    /* Set invalid value */
    pcip_policy_match = 0xFF;

    /* Verify if atleast one of the authenticator's pairwise policies */
    /* is supported here at STA                                       */
    for(i = 0; i < 3; i++)
    {
        /* If Cipher policy is zero, it is required to use Grp policy */
        if(pcip_policy[i] == 0)
        {
            /* No PW cipher is used, WEP is used for all packets */
            pcip_policy_match = pcip_policy[i];
            break;
        }
        /* If empty then continue the search */
        else if(pcip_policy[i] == 0xFF)
        {
            continue;
        }

        /* If the entry was found, check if it implemented */
        if(check_pcip_policy(pcip_policy[i]) == BTRUE)
        {
            pcip_policy_match = pcip_policy[i];
            /* In case of AES select this policy */
            if(pcip_policy[i] == 0x04)
            {
                break;
            }
        }
    }

    /* If no match was found, return error */
    if(pcip_policy_match == 0xFF)
    {
        return BFALSE;
    }

    /* Verify if atleast one of the authenticator's authentication    */
    /* policies is supported here at STA                              */
    for(i = 0; i < 3; i++)
    {
        /* If empty then continue the search */
        if(auth_policy[i] == 0xFF)
        {
            continue;
        }

        /* If the entry was found, check if it implemented */
        if(check_auth_policy(auth_policy[i]) == BTRUE)
        {
            auth_policy_match = auth_policy[i];
            break;
        }
    }

    /* If no match was found, return error */
    if(auth_policy_match == 0xFF)
    {
        return BFALSE;
    }

    grp_policy_match = grp_policy;

    /* For 802.11I RSNA, RSN Capability field must match */
    if(mode_802_11i == RSNA_802_11I)
    {

        /* RSN Capabilities Information                                      */
        /*********************************************************************/
        /* +----------+---------------+-------------+------------+---------+ */
        /* | B15 - B6 |  B5 - B4      | B3 - B2     |      B1    |    B0   | */
        /* +----------+---------------+-------------+------------+---------+ */
        /* | Reserved |  GTSKA Replay | PTSKA Replay| NoPairwise | Pre-Auth| */
        /* |          |    Counter    |   Counter   |            |         | */
        /* +----------+---------------+-------------+------------+---------+ */
        /*********************************************************************/

        /* Check Pre-Auth status in the capability */
        if((mget_RSNAPreauthenticationImplemented() == TV_TRUE) &&
           (mget_RSNAPreauthenticationEnabled() == TV_TRUE))
        {
            if(!(bss_dscr->rsn_cap[0] & BIT0))
            {
                return BFALSE;
            }
        }

        /* Pairwise field in the Capability field. AP must set this to 0 */
        if(bss_dscr->rsn_cap[0] & BIT1)
        {
            return BFALSE;
        }

        /* Check PTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfPTSKAReplayCounters  */
        if((bss_dscr->rsn_cap[0] & (BIT2 | BIT3)) !=
           ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2)
               & 0x000C))
        {
            /* Since we havent implemented this, just ignore */
        }

        /* Check GTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfGTSKAReplayCounters  */
        if((bss_dscr->rsn_cap[0] & (BIT4 | BIT5)) !=
            ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4)
               & 0x0030))
        {
             /* Since we havent implemented this, just ignore */
        }
    }

    /* Save the matched policies and settings */
    bss_dscr->auth_policy_match = auth_policy_match;
    bss_dscr->pcip_policy_match = pcip_policy_match;
    bss_dscr->grp_policy_match  = grp_policy_match;
    bss_dscr->mode_802_11i      = mode_802_11i;

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_rsn_capabilities_sta                            */
/*                                                                           */
/*  Description      : This procedure verifies the RSNA Capabilites in Scan  */
/*                     response                                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Scan Descriptor                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure verifies the RSNA Capabilites in Scan  */
/*                     response                                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE if capabilities match and BFASLE otherwise      */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_rsn_capabilities_sta(void *ptr)
{
    bss_dscr_t *bss_dscr = (bss_dscr_t *)ptr;

    /* If RSNA is enabled, RSN IE is a must in the Probe Res/Beacon Frames */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* FALSE is returned to AP which is not connectable. */
        if((bss_dscr->mode_802_11i & g_mode_802_11i) == 0)
        {
            return BFALSE;
        }

        /* Dont allow joing during counter measures */
        if(g_rsna_supp.global.cntr_msr_in_progress == BTRUE)
        {
            return BFALSE;
        }

        /* Check whether a valid RSN Element was found in the scan response  */
        if(bss_dscr->rsn_found == BFALSE)
        {
            return BFALSE;
        }

        /* If RSN-IE was present in scan response and RSN is enabled on the  */
        /* device then check if the capabilities match                       */
        if((g_mode_802_11i & RSNA_802_11I) &&
           (bss_dscr->mode_802_11i & RSNA_802_11I))
        {
            if(BTRUE == check_security_cap_sta(bss_dscr, RSNA_802_11I))
                return BTRUE;
        }

        /* If WPA-IE was present in scan response and WPA is enabled on the  */
        /* device then check if the capabilities match                       */
        if((g_mode_802_11i & WPA_802_11I) &&
           (bss_dscr->mode_802_11i & WPA_802_11I))
        {
            if(BTRUE == check_security_cap_sta(bss_dscr, WPA_802_11I))
                return BTRUE;
        }

        return BFALSE;
    }

    return BTRUE;
}

//chenq add for check link ap info change 2013-06-08
BOOL_T check_rsn_capabilities_sta_change(void *ptr)
{
    bss_dscr_t *bss_dscr = (bss_dscr_t *)ptr;

    /* If RSNA is enabled, RSN IE is a must in the Probe Res/Beacon Frames */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* FALSE is returned to AP which is not connectable. */
        if((bss_dscr->mode_802_11i & g_mode_802_11i) == 0)
        {
            return BFALSE;
        }

        /* Check whether a valid RSN Element was found in the scan response  */
        if(bss_dscr->rsn_found == BFALSE)
        {
            return BFALSE;
        }

        /* If RSN-IE was present in scan response and RSN is enabled on the  */
        /* device then check if the capabilities match                       */
        if((g_mode_802_11i & RSNA_802_11I) &&
           (bss_dscr->mode_802_11i & RSNA_802_11I))
        {
            if(BTRUE == check_security_cap_sta(bss_dscr, RSNA_802_11I))
                return BTRUE;
        }

        /* If WPA-IE was present in scan response and WPA is enabled on the  */
        /* device then check if the capabilities match                       */
        if((g_mode_802_11i & WPA_802_11I) &&
           (bss_dscr->mode_802_11i & WPA_802_11I))
        {
            if(BTRUE == check_security_cap_sta(bss_dscr, WPA_802_11I))
                return BTRUE;
        }

        return BFALSE;
    }

    return BTRUE;
}
//chenq add end

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_scan_response_11i                              */
/*                                                                           */
/*  Description      : This procedure extracts the RSNA Capabilites in Probe */
/*                     resp/beacon frames                                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Asoc/Reasoc Frame                   */
/*                     2) Length of the Received frame                       */
/*                     3) Offset from where the seach must be done           */
/*                     4) Index to the BSS-Dscr Table entry                  */
/*                                                                           */
/*  Globals          : g_bss_dscr_set                                        */
/*                                                                           */
/*  Processing       : Updates the AKM and PW Policies with the values in the*/
/*                     scan desriptor set                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void update_scan_response_11i(UWORD8 *msa, UWORD16 rx_len, UWORD16 offset,
                              UWORD8 dscr_set_index)
{
    UWORD16 index  = offset;
    UWORD16 index1 = offset;
    UWORD8  mode_802_11i = 0;

	//chenq add for wpa/rsn ie
	UWORD16  ie_index = 2;

	//chenq add 2013-02-26
	UWORD16 * ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_1[dscr_set_index]);
	
	memset(g_wpa_rsn_ie_buf_1[dscr_set_index],0x00,512);

    /* Reset the RSN Element Parsed condition */
    g_bss_dscr_set[dscr_set_index].rsn_found = BFALSE;
    g_bss_dscr_set[dscr_set_index].dot11i_info = 0;

    while(index < (rx_len - FCS_LEN))
    {
        index1 = index;

        /* Only copy the information element which is supported by this      */
        /* station. If it is a user initiated scan, then anyway it has to be */
        /* copied                                                            */
        if(((g_mode_802_11i & RSNA_802_11I) && (msa[index] == IRSNELEMENT)) ||
           ((g_mode_802_11i & WPA_802_11I)  && (is_wpa_ie(msa + index) == BTRUE)) || 
           ((g_scan_source & USER_SCAN) && (get_mac_state() == WAIT_SCAN)))
        {
            UWORD8 *oui         = 0;
            UWORD8 temp         = 0;
            UWORD8 *pcip_policy = 0;
            UWORD8 *grp_policy  = 0;
            UWORD8 *auth_policy = 0;
            UWORD8 rsn_type     = 0;

            /* Skip the Tag number and the length */
            index += 2;

			//chenq add for save wpa/rsn ie
			//#ifdef EXT_SUPP_11i
			#if 1
			if(    (msa[index - 2] == IRSNELEMENT) 
				|| (msa[index - 2] == IWPAELEMENT) 
				|| (msa[index - 2] == IWAPIELEMENT)
				|| (msa[index - 2] == ISSID)
				|| (msa[index - 2] == ISUPRATES)
				|| (msa[index - 2] == IDSPARMS)
				|| (msa[index - 2] == ICOUNTRY)
				|| (msa[index - 2] == IERPINFO)
				|| (msa[index - 2] == IEXSUPRATES))
		    {
				//chenq add a new buf tp save wpa/rsn ie
				//g_wpa_rsn_ie_buf_1[dscr_set_index][0] += msa[index1 + 1] + 2;
				
				if((ie_index + msa[index1 + 1] + 2) < 512)
				{
				*ie_len += msa[index1 + 1] + 2;
				memcpy(&g_wpa_rsn_ie_buf_1[dscr_set_index][ie_index],&msa[index1],msa[index1 + 1] + 2);
				ie_index += msa[index1 + 1] + 2;
				//printk("1 chenq_itm: id:%d,len:%d\n",msa[index1],msa[index1 + 1]);
				}
				else
				{
					printk("%s[%d]  ie_len:%d over 512 bytes ==== fail\n",
						__FUNCTION__, __LINE__, ie_index + msa[index1 + 1] + 2);
				}
				
			}

			if(both_elements_already_found(&g_bss_dscr_set[dscr_set_index]))
		    {
		         index1 += msa[index1 + 1] + 2;
		         index   = index1;
		         continue;
		    }
			#endif
			
            /* Update the OUI For the RSN mode */
            if(msa[index - 2] == IRSNELEMENT)
            {
                oui = mget_RSNAOUI();
                mode_802_11i |= RSNA_802_11I;
                rsn_type = IRSNELEMENT;
                pcip_policy = g_bss_dscr_set[dscr_set_index].rsn_pcip_policy;
                grp_policy  = &(g_bss_dscr_set[dscr_set_index].rsn_grp_policy);
                auth_policy = g_bss_dscr_set[dscr_set_index].rsn_auth_policy;
            }
            else
            {
                oui = mget_WPAOUI();
                mode_802_11i |= WPA_802_11I;

                rsn_type = IWPAELEMENT;
                /* Skip the WPA OUI and OUI Type for WPA IE */
                index += 4;
                pcip_policy = g_bss_dscr_set[dscr_set_index].wpa_pcip_policy;
                grp_policy  = &(g_bss_dscr_set[dscr_set_index].wpa_grp_policy);
                auth_policy = g_bss_dscr_set[dscr_set_index].wpa_auth_policy;
            }

            /* Verify the verion number for the 802.11i at STA */
            if(msa[index] != mget_RSNAConfigVersion())
            {
                /* On failure status code is set appropriately*/
                index1 += msa[index1 + 1] + 2;
                index   = index1;
                continue;
            }

            /* Skip the version number */
            index += 2;

            /* Get AKM, PW, and GW Cipher policies from the frame */

			//chenq move after check_oui 2013-06-09
			#if 0
            /* Reset the Cipher and Authentication Policies */
            mem_set(pcip_policy, 0xFF, 3);
            mem_set(auth_policy, 0xFF, 3);
			#endif

            /* Check the OUI for the group cipher policy */
            if(check_oui(&msa[index], oui) == BFALSE)
            {
                /* On failure status code is set appropriately*/
                index1 += msa[index1 + 1] + 2;
                index   = index1;
                continue;
            }

			//chenq move here 2013-06-09
			#if 1
            /* Reset the Cipher and Authentication Policies */
            mem_set(pcip_policy, 0xFF, 3);
            mem_set(auth_policy, 0xFF, 3);
			#endif

            index += 3; /* Skip the OUI Length */

            *grp_policy = msa[index];
            index++;

            if(get_pcip_policy_sta(&msa[index], &temp, pcip_policy,
                                   oui) > 0)
            {
                index += temp;
            }
            else
            {
                /* On failure status code is set appropriately*/
                index1 += msa[index1 + 1] + 2;
                index   = index1;
                continue;
            }

            if(get_auth_policy_sta(&msa[index], &temp, auth_policy,
                                   oui) > 0)
            {
                index += temp;
            }
            else
            {
                /* On failure status code is set appropriately*/
                index1 += msa[index1 + 1] + 2;
                index   = index1;
                continue;
            }

            /* Copy the capability field */
            if(rsn_type == IRSNELEMENT)
            {
                g_bss_dscr_set[dscr_set_index].dot11i_info |= BIT4;
                g_bss_dscr_set[dscr_set_index].rsn_type = IRSNELEMENT;
                g_bss_dscr_set[dscr_set_index].rsn_cap[0] = msa[index];
                g_bss_dscr_set[dscr_set_index].rsn_cap[1] = msa[index + 1];
            }

            if(rsn_type == IWPAELEMENT)
            {
                g_bss_dscr_set[dscr_set_index].dot11i_info |= BIT3;
                g_bss_dscr_set[dscr_set_index].rsn_type = IWPAELEMENT;
            }

            /* Set the Mode(WPA or WPA2) & rnsa found flag and exit */
            g_bss_dscr_set[dscr_set_index].mode_802_11i = mode_802_11i;
            g_bss_dscr_set[dscr_set_index].rsn_found    = BTRUE;

            /* Consolidate all the security information into a single byte */
            update_dot11i_info(&g_bss_dscr_set[dscr_set_index], rsn_type);

//#ifndef EXT_SUPP_11i
#if 0
            /* End the search if both WPA/WPA2 elements are already found */
            if(both_elements_already_found(&g_bss_dscr_set[dscr_set_index]))
            {
                 break;
            }
            /* Keep searching till the end */
            else
#endif				
            {
                /* On failure status code is set appropriately*/
                index1 += msa[index1 + 1] + 2;
                index   = index1;
                continue;
            }
        }
        else
        {
			//printk("update_scan_response_11i not in save logic\n");
			//#ifdef EXT_SUPP_11i
			#if 1
			if(    (msa[index] == IRSNELEMENT) 
				|| (msa[index] == IWPAELEMENT) 
				|| (msa[index] == IWAPIELEMENT)
				|| (msa[index] == ISSID)
				|| (msa[index] == ISUPRATES)
				|| (msa[index] == IDSPARMS)
				|| (msa[index] == ICOUNTRY)
				|| (msa[index] == IERPINFO)
				|| (msa[index] == IEXSUPRATES))
		    {
				//chenq add a new buf tp save wpa/rsn ie
				//g_wpa_rsn_ie_buf_1[dscr_set_index][0] += msa[index1 + 1] + 2;
				
				if((ie_index + msa[index1 + 1] + 2) < 512)
				{
				*ie_len += msa[index1 + 1] + 2;
				memcpy(&g_wpa_rsn_ie_buf_1[dscr_set_index][ie_index],&msa[index1],msa[index1 + 1] + 2);
				ie_index += msa[index1 + 1] + 2;
				//printk("2 chenq_itm: id:%d,len:%d\n",msa[index1],msa[index1 + 1]);	
				}
				else
				{
					printk("%s[%d]  ie_len:%d over 512 bytes ==== fail\n",
						__FUNCTION__, __LINE__, ie_index + msa[index1 + 1] + 2);
				}
			}
			#endif

		
            index1 += msa[index1 + 1] + 2;
            index   = index1;
        }
    }
}

#ifdef IBSS_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_tbtt_11i_event                                 */
/*                                                                           */
/*  Description      : This procedure checks if the Authentication FSM is to */
/*                     scheduled for any of the remote stations              */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_rsna_supp                                           */
/*                     g_rsna_auth                                           */
/*                                                                           */
/*  Processing       : If the Beacon Tx counter is greater than 10, the FSM  */
/*                     is started if not already started                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_tbtt_11i_event(void)
{
    UWORD16 i = 0;
    if(get_machw_prev_bcn_tx_stat() == BFALSE)
    {
        return;
    }
    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        if(g_rsna_supp.rsna_ptr[i] != NULL)
        {
            rsna_supp_persta_t *rsna_ptr      = g_rsna_supp.rsna_ptr[i];
            rsna_auth_persta_t *auth_rsna_ptr = g_rsna_auth.rsna_ptr[i];

            rsna_ptr->tbtt_count++;
            if((rsna_ptr->fsm_start_pend == BTRUE) &&
               (rsna_ptr->tbtt_count >= 10))
            {
                rsna_ptr->fsm_start_pend = BFALSE;
                if((g_psk_available == BTRUE) &&
                   (rsna_ptr->auth_policy == 0x02))
                {
                    /* If PSK is enabled, start the RSNA state machine */
                    start_auth_rsna_fsm(auth_rsna_ptr);
                }
                else
                {
                    /* If PSK is disabled for this STA, enabled authentication FSM */
                    /* 802.1X authentication is to be invoked here :IntegrateTBD   */
                }
            }
        }
    }
}
#endif /* IBSS_11I */

#ifdef SUPP_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : set_grp_key                                           */
/*                                                                           */
/*  Description      : The function sets the group ket to MAC HW CE          */
/*                                                                           */
/*  Inputs           : Pointer to RSNA structure                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : determines the cypher type from group policy, reads   */
/*                     the group ket from the respective locations and sets  */
/*                     the same in to MAC HW CE                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void set_grp_key(rsna_supp_persta_t *rsna_ptr)
{
    UWORD8 key[16]       = {0};
    UWORD8 *mic_key      = NULL;
    UWORD8 cipher_type   = 0;
    UWORD8 key_id        = 0;
    UWORD8 mask          = 0;

    switch(rsna_ptr->grp_policy)
    {
        case 4:
        {
            UWORD16 i   = 0;
            cipher_type = CCMP;

            if( rsna_ptr->GN > GTK_MAX_INDEX )
            {
                rsna_ptr->deauth_reason = UNSPEC_REASON;
                rsna_supp_disconnected(rsna_ptr);
                return;
            }
            key_id      = rsna_ptr->GN;
            for(i = 0; i < 16; i ++)
            {
                key[i] = rsna_ptr->GTK[get_GTK_array_index(key_id)][i];
            }
        }
        break;
        case 2:
        {
            UWORD16 i   = 0;
            cipher_type = TKIP;

            if( rsna_ptr->GN > GTK_MAX_INDEX )
            {
                rsna_ptr->deauth_reason = UNSPEC_REASON;
                rsna_supp_disconnected(rsna_ptr);
                return;
            }
            key_id      = rsna_ptr->GN;
            for(i = 0; i < 16; i ++)
            {
                key[i] = rsna_ptr->GTK[get_GTK_array_index(key_id)][i];
            }
            mic_key = &rsna_ptr->GTK[get_GTK_array_index(key_id)][TEMPORAL_KEY_LENGTH];
        }
        break;
        case 1:
        {
            UWORD16 i   = 0;
            cipher_type = WEP40;
            key_id      = mget_WEPDefaultKeyID();
            mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
            for(i = 0; i < 5; i ++)
            {
                key[i] = rsna_ptr->GTK[get_GTK_array_index(key_id)][i];
            }
        }
        break;
        case 5:
        {
            UWORD16 i   = 0;
            cipher_type = WEP104;
            key_id      = mget_WEPDefaultKeyID();
            mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
            for(i = 0; i < 13; i ++)
            {
                key[i] = rsna_ptr->GTK[get_GTK_array_index(key_id)][i];
            }
        }
        break;
        default:
            return;

    }

    /* Set the cipher type in RX GTK Cipher Type register */
    set_machw_ce_grp_key_type(cipher_type);

    /* Rotate the Rx Key num and use it for setting the new key */
    if(!((cipher_type == WEP40) || (cipher_type == WEP104)))
    {
        if(key_id == 2)
        {
            rsna_ptr->last_key_num = RX_GTK1_NUM;
        }
        else
        {
            rsna_ptr->last_key_num = RX_GTK0_NUM;
        }
    }
    else
    {
         rsna_ptr->last_key_num = TX_GTK_NUM;
    }

    /* Set the broadcast receive keys */
    machw_ce_add_key(key_id, (KEY_NUMBER_T)rsna_ptr->last_key_num, rsna_ptr->key_index,
                     mask, cipher_type, key, rsna_ptr->addr,
                     SUPP_KEY, mic_key);

    if(rsna_ptr->last_key_num == RX_GTK0_NUM)
    {
        /* Set the broadcast PN value for this STA index */
        machw_ce_init_rx_bcmc_pn_val(0, 0, rsna_ptr->bcst_rx_pn_val);
    }
    else
    {
        /* Set the broadcast PN value for this STA index */
        machw_ce_init_rx_bcmc_pn_val(0, 1, rsna_ptr->bcst_rx_pn_val);
    }
}
#endif /* SUPP_11I */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
