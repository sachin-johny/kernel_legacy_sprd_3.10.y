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
/*  File Name         : rsna_auth_km.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of RSNA Key Management.   */
/*                                                                           */
/*  List of Functions : initialize_auth_rsna                                 */
/*                      stop_auth_rsna                                       */
/*                      initialize_auth_rsna_hdl                             */
/*                      start_auth_rsna_fsm                                  */
/*                      free_auth_rsna_km                                    */
/*                      auth_rsn_fsm_run                                     */
/*                      auth_rsn_eapol                                       */
/*                      check_rply_cnt                                       */
/*                      check_eapol_pkt_auth                                 */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "rsna_auth_km.h"
#include "rkmal_auth.h"
#include "ieee_1x.h"
#include "mib_11i.h"
#include "utils.h"
#include "eapol_key.h"
#include "receive.h"
#include "md5.h"
#include "transmit.h"
#include "frame_11i.h"
#include "auth_frame_11i.h"
#include "mh.h"
#include "ce_lut.h"
#include "core_mode_if.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

rsna_auth_t g_rsna_auth    = {{AGLK_GTK_INIT, },};

/* The Tx/Rx bit is set according to the cipher suite in use. If a   */
/* pairwise key is used for encryption/integrity, then the STA never */
/* transmits with the GTK; otherwise, STA uses the GTK for transmit. */
/* Currently this is maintained as a global.                         */
BOOL_T  g_use_pairwisekey  = BTRUE;

UWORD8  g_gmk[GMK_SIZE]    = {0};

/*****************************************************************************/
/* Static Functions                                                          */
/*****************************************************************************/

static BOOL_T  check_rply_cnt(rsna_auth_persta_t *rsna_ptr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_auth_rsna                                  */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after MAC is enabled.                    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_auth_rsna(void)
{
    UWORD8 i = 0;

    /* Reset Parameters for the GTK State machine */
    mem_set(&g_rsna_auth.global, 0, sizeof(rsna_auth_global_t));

    /* Set the GInit parameter, this is used to initialize the State machine */
    g_rsna_auth.global.GInit    = BTRUE;
    g_rsna_auth.global.GTKReKey = BFALSE;

    /* Set GTH Authenticator */
    g_rsna_auth.global.GTKAuthenticator = BTRUE;

    /* Count of number of STAs left to have the */
    /* global variable.                         */
    g_rsna_auth.global.GKeyDoneStations = 0;

    /* Initialize the GMK with random values */
    for(i = 0 ; i < GTK_SIZE; i++)
    {
        g_gmk[i] = get_random_byte();
    }

    /* Initialize the Nonce and the Nonce Counters */
    for(i = 0 ; i < NONCE_SIZE; i++)
    {
        g_rsna_auth.global.GNonce[i] = get_random_byte();
    }

    /* The nonce counters are set to be equal to the Nonce */
    memcpy(g_rsna_auth.global.GCounter, g_rsna_auth.global.GNonce, NONCE_SIZE);

    /* Initialize the Nonce and the Nonce Counters */
    for(i = 0 ; i < NONCE_SIZE; i++)
    {
        g_rsna_auth.global.Counter[i] = get_random_byte();
    }

    /* Reset the Rekey Alarm Handle and counter measure Alarm Handle */
    g_rsna_auth.global.alarm_handle          = NULL;
    g_rsna_auth.global.cntr_msr_alarm_handle = NULL;
    g_rsna_auth.global.cntr_msr_in_progress  = BFALSE;

    /* Initialize all the pointers to the RSNA state machines to NULL */
    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        g_rsna_auth.rsna_ptr[i] = NULL;
    }

    /* Set the GTK for the BroadCast packets */
    auth_rsn_fsm_run(NULL);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_auth_rsna                                        */
/*                                                                           */
/*  Description      : This function stops the RSNA KM state machine.        */
/*                     It is called before any reset                         */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_auth_rsna(void)
{
    UWORD8 i = 0;

	TROUT_FUNC_ENTER;
    /* Disable the RNSA Enabled Parameter in MIB */
    mset_RSNAEnabled(TV_FALSE);

    /* Stop and delete the rekey timer */
    delete_rekey_timer();

    /* Stop and delete the counter measure timer */
    stop_auth_cntr_msr_timer();

    /* Free all the RSNA Handles */
    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        if(g_rsna_auth.rsna_ptr[i] != NULL)
        {
            free_auth_rsna_km(&g_rsna_auth.rsna_ptr[i]);
        }
    }

    /* Stop the 1X FSM All together */
    stop_sec_auth_node();

    /* Enable the RNSA Enabled Parameter in MIB after stopping the RSNA FSM */
    mset_RSNAEnabled(TV_TRUE);
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_auth_rsna_hdl                              */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after association request is recevied    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : Returns BFALSE on failure, BTRUE otherwise            */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T initialize_auth_rsna_hdl(rsna_auth_persta_t **rsna_per_sta_ptr,
                                UWORD16 aid, UWORD16 listen_interval,
                                UWORD8 pcip_policy, UWORD8 auth_policy,
                                UWORD8* sa, UWORD8 *msa, UWORD16 rx_len,
                                rsna_auth_persta_t *rsna_per_sta_ptr_buff)
{
    rsna_auth_persta_t *rsna_ptr        = NULL;
    auth_t             *sta_1x          = NULL;
    UWORD8             pmkid[PMKID_LEN] = {0};
    BOOL_T             pmksa            = BFALSE;
    UWORD8             pmk[PMK_SIZE]    = {0};
    UWORD8             mode_802_11i     = mget_RSNAModeRequested();

    /* For WEP Stations in TSN, RSN Handle is not required */
    if((pcip_policy == 0x01) || (pcip_policy == 0x05))
    {
        /* Add WEP Entry for the STA */
        /* Using Default KeyId & Sta Index generated while AUTH_REQ */
       	add_wep_entry(aid, mget_WEPDefaultKeyID(), sa);

        /* Reset the handle */
        *rsna_per_sta_ptr = NULL;

        return BTRUE;
    }

    /* Check if the RSNA Handle is already in use */
    if(*rsna_per_sta_ptr != NULL)
    {
        rsna_ptr = *rsna_per_sta_ptr;

        if(rsna_ptr->mode_802_11i == mode_802_11i)
        {
            if(get_pmkid_asoc(msa, rx_len, rsna_ptr->pmkid) != NULL)
            {
                memcpy(pmkid, rsna_ptr->pmkid, PMKID_LEN);
                memcpy(pmk, rsna_ptr->PMK, PMK_SIZE);
                pmksa = BTRUE;
            }
        }

        /* Copy the old value of the 1x handle */
        sta_1x = rsna_ptr->sta_1x;

        /* If the RSNA Handle is already in use - do nothing*/
        stop_11i_auth_timer(*rsna_per_sta_ptr);
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

    mem_set(rsna_ptr, 0, sizeof(rsna_auth_persta_t));

    /* The aid has the two MSBs set to 1, Reset them for current use */
    aid &= 0x3F;

    memcpy(rsna_ptr->addr, sa, 6);

    /* Set the required parameters of per-Sta StateMachine */
    rsna_ptr->AuthenticationRequest   = BTRUE;
    rsna_ptr->DeauthenticationRequest = BFALSE;
    rsna_ptr->ReAuthenticationRequest = BFALSE;
    rsna_ptr->aid                     = aid;
    rsna_ptr->sta_1x                  = sta_1x;

    /* Set the listen interval. If it is zero, set it to 100ms */
    rsna_ptr->listen_interval         = listen_interval;
    rsna_ptr->pcip_policy             = pcip_policy;
    rsna_ptr->auth_policy             = auth_policy;
    rsna_ptr->mode_802_11i            = mode_802_11i;

    if((rsna_ptr->auth_policy == 0x02) && (g_psk_available == BFALSE))
    {
        //xuan.yang, 2013-10-18, initialize rsna state;
        rsna_ptr->rsna_auth_4way_state = A4_INITIALIZE;
        free_auth_rsna_km(&rsna_ptr);
        return BFALSE;
    }
    else if((rsna_ptr->auth_policy == 0x02) && (g_psk_available == BTRUE))
    {
        /* If the PSK is used for the STA 1X handle will not be instantiated */
        /* By 1X initializations                                             */
        /* Allocate the handle for 1x and initialize the variables           */
        if(sta_1x == NULL)
        {
            /* Initialize the handle to the given buffer */
            rsna_ptr->sta_1x = &(rsna_ptr->auth_1x_buff);
        }
        else
        {
            rsna_ptr->sta_1x = sta_1x;
        }
        if(rsna_ptr->sta_1x == NULL)
        {
            free_auth_rsna_km(&rsna_ptr);
            return BFALSE;
        }

        mem_set(rsna_ptr->sta_1x, 0, sizeof(auth_t));


        ((auth_t *)(rsna_ptr->sta_1x))->global.keyRun       = BTRUE;
        ((auth_t *)(rsna_ptr->sta_1x))->global.keyAvailable = BTRUE;
    }
    else if(rsna_ptr->auth_policy == 0x01)
    {
        if(init_sec_auth_node((void **)(&(rsna_ptr->sta_1x)),
                              (UWORD8 *)rsna_ptr, aid,
                              (void *)(&(rsna_ptr->auth_1x_buff))) != BTRUE)
        {
            //xuan.yang, 2013-10-18, initialize rsna state;
            rsna_ptr->rsna_auth_4way_state = A4_INITIALIZE;
            free_auth_rsna_km(&rsna_ptr);
            return BFALSE;
        }
        if(pmksa == BTRUE)
        {
            ((auth_t *)(rsna_ptr->sta_1x))->global.keyRun       = BTRUE;
            ((auth_t *)(rsna_ptr->sta_1x))->global.keyAvailable = BTRUE;
            memcpy(&(((auth_t *)(rsna_ptr->sta_1x))->rx_key[1]), pmk, PMK_LEN);
            memcpy(rsna_ptr->pmkid, pmkid, PMKID_LEN);
        }
    }

    /* Setting the key length for the Pairwise cipher suite */
    if((pcip_policy == 0x02) || (pcip_policy == 0x00))
    {
        /* For TKIP the key length is 256 */
        rsna_ptr->key_length = 32;
    }
    else if(pcip_policy == 0x04)
    {
        /* The Pairwise cipher suite is 128 */
        rsna_ptr->key_length = 16;
    }

    /* Key version is the field which determines type of the encryption used */
    /* If TKIP is used, then set the key type to 1                           */
    if((rsna_ptr->pcip_policy == 0x02) || (rsna_ptr->pcip_policy == 0x00))
    {
        rsna_ptr->key_version = 1;
    }
    else if(rsna_ptr->pcip_policy == 0x04)
    {
        /* For CCMP/AES, the key type is set to 2 */
        rsna_ptr->key_version = 2;
    }

    rsna_ptr->send_deauth        = BFALSE;
    rsna_ptr->alarm_handle       = NULL;
    rsna_ptr->GUpdateStationKeys = BFALSE;
    ((auth_t *)(rsna_ptr->sta_1x))->global.keyDone = BFALSE;

    /* Copy the key index: 0 if new RSNA Handle or the old value from */
    /* previous handle                                                */
    rsna_ptr->key_index          = aid;

    /* Update the pointer */
    *rsna_per_sta_ptr            = rsna_ptr;

    /* Update the Glohal list with this RSNA handler */
    if(aid)
    {
        g_rsna_auth.rsna_ptr[aid - 1]    = rsna_ptr;
    }
    else
    {
        return BFALSE;
    }

    return BTRUE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_auth_rsna_fsm                                   */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after association request is recevied    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void start_auth_rsna_fsm(rsna_auth_persta_t *rsna_per_sta_ptr)
{
    /* Setting the Init Condition to true */
    rsna_per_sta_ptr->InitA4 = BTRUE;
    rsna_per_sta_ptr->InitGK = BTRUE;
    rsna_per_sta_ptr->AuthenticationRequest = BTRUE;

    /* Running the FSM to move the state machine to the initial states */
    auth_rsn_fsm_run(rsna_per_sta_ptr);

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_auth_rsna_fsm                                    */
/*                                                                           */
/*  Description      : This function halts the RSNA KM state machine.        */
/*                     It is called after dissociation request is recevied   */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_auth_rsna_fsm(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr != NULL)
    {
        /* Stop any associated Timer with this RSNA Suite */
        stop_11i_auth_timer(rsna_ptr);

        /* Setting the Init Condition to true */
        rsna_ptr->DeauthenticationRequest = BTRUE;

        /* Running the FSM to move the state machine to the initial states */
        auth_rsn_fsm_run(rsna_ptr);

        /* Remove the Key index from the CE-LUT */
        machw_ce_del_key(rsna_ptr->key_index);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : free_auth_rsna_km                                     */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after MAC is enabled.                    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void free_auth_rsna_km(rsna_auth_persta_t **rsna_ptr)
{
    UWORD16 aid = 0;
    if(*rsna_ptr == NULL)
    {
        return;
    }

    if((*rsna_ptr)->key_index != 0)
    {
        /* Remove the RSNA Stats from the MIB */
        mfree_RSNAStats((UWORD8)((*rsna_ptr)->key_index));

        /* Remove the RSNA Entry from the CE LUT */
        machw_ce_del_key((*rsna_ptr)->key_index);
    }

    /* Copy the AID from the RSNA handle */
    aid = (*rsna_ptr)->aid;

    /* Set Deauthentication Request to True */
    (*rsna_ptr)->DeauthenticationRequest = BTRUE;

    /* Run the FSM over the Deauthenticatin request */
    auth_rsn_fsm_run(*rsna_ptr);

    /* Stop any associated Timer with this RSNA Suite */
    stop_11i_auth_timer(*rsna_ptr);

    /* Set the group key state to invalid so that GTK update */
    /* wont happen for a deleted station */
    (*rsna_ptr)->rsna_auth_grpkey_state = AGK_INVALID_STATE;

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
        free_sec_auth_node((void **)(&((*rsna_ptr)->sta_1x)));
    }

    /* Reset the pointers to this State machine handle */
    *rsna_ptr                 = NULL;

    /* Update the Glohal list with NULL handler */
    if(aid)
    {
        g_rsna_auth.rsna_ptr[aid - 1]    = NULL;
    }
    else
    {
        return;
    }

    return;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_rsn_fsm_run                                      */
/*                                                                           */
/*  Description      : This function runs all the authenticator state        */
/*                     machines till no further state change is possible.    */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void auth_rsn_fsm_run(rsna_auth_persta_t *rsna_ptr)
{
    UWORD32 ostate      = 0;
    BOOL_T  statechange = BTRUE;

    while(statechange == BTRUE)
    {
        statechange = BFALSE;

        if(rsna_ptr !=NULL)
        {
            ostate = rsna_ptr->rsna_auth_4way_state;
            rsna_auth_4way_fsm(rsna_ptr);
            statechange |= (ostate != (UWORD32)rsna_ptr->rsna_auth_4way_state);

            ostate = rsna_ptr->rsna_auth_grpkey_state;
            rsna_auth_grpkey_fsm(rsna_ptr);
            statechange |=
                (ostate != (UWORD32)rsna_ptr->rsna_auth_grpkey_state);

            /* If the deauthentication bit is set, send Deauth frame */
            if(rsna_ptr->send_deauth == BTRUE)
            {
                rsna_send_deauth((UWORD8 *)rsna_ptr, 0);
                rsna_ptr->send_deauth = BFALSE;
            }
        }

        ostate = g_rsna_auth.global.rsna_auth_glk_state;
        rsna_auth_glk_fsm();
        statechange |=
                (ostate != (UWORD32)g_rsna_auth.global.rsna_auth_glk_state);

    }
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_rsn_eapol                                        */
/*                                                                           */
/*  Description      : This function calls the authenticator state m/c       */
/*                     for the received EAPOL packet                         */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void auth_rsn_eapol(UWORD8* rx_pkt, UWORD16 len, rsna_auth_persta_t *rsna_ptr)
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
            /* Delete the timer and restart the FSM */
            stop_11i_auth_timer(rsna_ptr);
            start_auth_rsna_fsm(rsna_ptr);

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
            stop_11i_auth_timer(rsna_ptr);

            auth_rsn_fsm_run(rsna_ptr);

            break;
        }
        default:
        {
            break;
        }
    }

    /* Reset the buffer addr as this packet is no longer valid  */
    rsna_ptr->eapol_key_frame = NULL;
    rsna_ptr->EAPOLKeyReceived  = BFALSE;

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_rply_cnt                                        */
/*                                                                           */
/*  Description      : This procedure verifies EAPOL REPLAY in the rxed frame*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
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

BOOL_T check_rply_cnt(rsna_auth_persta_t *rsna_ptr)
{
    BOOL_T ret_val = BFALSE;
    UWORD8 rply_rcvd[REPLAY_COUNTER_SIZE];
    UWORD8 *eapol_buffer;

    /* Set the buffer and length to appropriate values */
    eapol_buffer = rsna_ptr->eapol_key_frame;

    /* Extract the replay counter from the recevied EAPOL Frame */
    memcpy(rply_rcvd, get_rply_cnt(eapol_buffer), REPLAY_COUNTER_SIZE);

    /* The replay counter in FSM has been incremented, so increment the */
    /* received counter before comparing                                */
    incr_rply_cnt(rply_rcvd);

    if(memcmp(rply_rcvd, rsna_ptr->ReplayCount, REPLAY_COUNTER_SIZE) == 0)
    {
        ret_val = BTRUE;
    }

    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_eapol_pkt_auth                                  */
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

BOOL_T check_eapol_pkt_auth(rsna_auth_persta_t *rsna_ptr)
{
    UWORD8 *eapol_buffer = 0;
    UWORD8 index         = 0;
    UWORD8 desc_version  = 0;
    UWORD8 key_type      = 0;
    UWORD8 key_index     = 0;

    /* Set the buffer and length to appropriate values */
    eapol_buffer = rsna_ptr->eapol_key_frame;


    if(eapol_buffer == NULL)
    {
        return BFALSE;
    }

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

    /* For a pairwise key type the key index has to be zero */
    if(key_type == P)
    {
        if(key_index != 0)
        {
            return BFALSE;
        }
    }

    /* Install bit and the ACK bit must be reset */
    if((eapol_buffer[index + 1] & 0xC0) != 0x00)
    {
        return BFALSE;
    }

    /* MIC Bit must be set */
    if((eapol_buffer[index] & 0x01) != 0x01)
    {
        return BFALSE;
    }

    /* Error Bit must be reset */
    if((eapol_buffer[index] & 0x04) != 0x00)
    {
        return BFALSE;
    }

    index += KEY_INFO_SIZE; /* Skip the Key info field */

#if 0 /* Cisco Kludge */
    if(eapol_buffer[index + 1] != 0)
    {
        return BFALSE;
    }
#endif /* 0 */

    index += KEY_LENGTH_SIZE; /* Skip the Key length field */

    /* Check the replay counter */
    if(check_rply_cnt(rsna_ptr) == BFALSE)
    {
        return BFALSE;
    }

    /* Checking of the key IV field KEY RSC and Key ID Fields     */
    /* is not required as standard doesnt require them to be zero */

    return BTRUE;
}

void handle_rsna_timeout(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr->alarm_handle != NULL)
    {   
        /* Stop the timer */
        stop_11i_auth_timer(rsna_ptr);

        /* Set timeout event to True */
        rsna_ptr->TimeoutEvt = BTRUE;

        printk("@@@handle_rsna_timeout run.@@@\n");
        /* Run the 802.11i state machine */
        auth_rsn_fsm_run(rsna_ptr);
    }
}
#endif /* AUTH_11I */
#endif /* MAC_802_11I*/
#endif /* MAC_HW_UNIT_TEST_MODE */
