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
/*  File Name         : rsna_supp_4way.c                                     */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of the Per-STA 4-Way Key  */
/*                      Handshake state machine.                             */
/*                                                                           */
/*  List of Functions : rsna_supp_4way_fsm                                   */
/*                      rsna_supp_authentication                             */
/*                      rsna_supp_disconnected                               */
/*                      rsna_supp_initialize                                 */
/*                      rsna_supp_initpmk                                    */
/*                      rsna_supp_initpsk                                    */
/*                      rsna_supp_ptkcalcnegotiating                         */
/*                      rsna_supp_ptkcalcnegotiating2                        */
/*                      rsna_supp_ptkinitnegotiating                         */
/*                      rsna_supp_ptkinitdone                                */
/*                      rsna_supp_ptkstart                                   */
/*                      STADisconnect                                        */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "itypes.h"
#include "rsna_supp_km.h"
#include "eapol_key.h"
#include "prf.h"
#include "utils.h"
#include "rkmal_sta.h"
#include "aes.h"
#include "md5.h"
#include "mib_11i.h"
#include "frame_11i.h"
#include "ce_lut.h"
#include "rc4.h"
#include "iconfig.h"

#ifdef SUPP_11I

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void rsna_supp_authentication(rsna_supp_persta_t *rsna_ptr);
static void rsna_supp_initialize(rsna_supp_persta_t *rsna_ptr);
static void rsna_supp_stakeystart(rsna_supp_persta_t *rsna_ptr);
static void STADisconnect(rsna_supp_persta_t *rsna_ptr);
static void STAProcessEAPOLKey(rsna_supp_persta_t *rsna_ptr);
static BOOL_T check_key_data(rsna_supp_persta_t *rsna_ptr, BOOL_T *is_present);
static UWORD8 DecryptGTK(rsna_supp_persta_t *rsna_ptr);
static BOOL_T verify_RSNIE(UWORD8 *eapol_buffer, BOOL_T first_rsnie,
                           UWORD8 mode_802_11i);
static BOOL_T DecryptSK(rsna_supp_persta_t *rsna_ptr){return BTRUE;}
static BOOL_T send_supp_eapol(rsna_supp_persta_t *rsna_ptr, BOOL_T set_rsn_ie,
                    BOOL_T is_secured);


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_supp_4way_fsm                                    */
/*                                                                           */
/*  Description      : This function implements the Per-STA 4-Way Key        */
/*                     Handshake state machine. In each of the various       */
/*                     states, it checks for the appropriate inputs before   */
/*                     switching to next state. This state machine is        */
/*                     instantiated per supplicant.                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_supp_4way_fsm(rsna_supp_persta_t *rsna_ptr)
{
    if((rsna_ptr->DeauthenticationRequest == BTRUE) ||
       (rsna_ptr->Init == BTRUE))
    {
        rsna_supp_initialize(rsna_ptr);
        return;
    }

    if((rsna_ptr->AuthenticationFailed == BTRUE) ||
       (rsna_ptr->TimeoutEvt == BTRUE))
    {
        rsna_supp_disconnected(rsna_ptr);
        return;
    }


    switch(rsna_ptr->rsna_supp_4way_state)
    {
    case S4_DISCONNECTED:
    {
        rsna_supp_initialize(rsna_ptr);
    } /* End of case S4_DISCONNECTED: */
    break;

    case S4_INITIALIZE:
    {
        if(rsna_ptr->AuthenticationRequest == BTRUE)
        {
            rsna_supp_authentication(rsna_ptr);
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case S4_INITIALIZE: */
    break;

    case S4_AUTHENTICATION:
    case S4_STAKEYSTART:
    {
        if((rsna_ptr->EAPOLKeyReceived     == BTRUE) &&
           (!get_request(rsna_ptr->eapol_key_frame)) &&
           (check_eapol_pkt_supp(rsna_ptr) == BTRUE))
        {
            /* Check the MIC for the received packet */
            if(rsna_ptr->MICVerified == BTRUE)
            {
                rsna_supp_stakeystart(rsna_ptr);
            }
            else
            {
                TROUT_DBG4("P2P: rsna_ptr->MICVerified FALSE");
                rsna_ptr->deauth_reason = MIC_FAIL;
                rsna_supp_disconnected(rsna_ptr);
            }

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Reset MIC Valid flag */
            rsna_ptr->MICVerified = BFALSE;
        }
        else if(rsna_ptr->EAPOLKeyReceived == BTRUE)
        {
            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case S4_AUTHENTICATION/S4_STAKEYSTART */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(rsna_ptr->rsna_supp_4way_state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_supp_authentication                              */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATION state of  */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_supp_authentication(rsna_supp_persta_t *rsna_ptr)
{
    /* Reset the PTK/TPKT/GTK */
    {
        UWORD8 i = 0;
        UWORD8 j = 0;

        for(i = 0; i < PTK_SIZE; i++)
        {
            rsna_ptr->PTK[i]  = 0;
        }

        for(j = 0; j < NUM_GTK; j ++)
        {
            for(i = 0; i < GTK_SIZE; i++)
            {
                rsna_ptr->GTK[j][i] = 0;
            }
        }
    }

    memcpy(rsna_ptr->SNonce, &(g_rsna_supp.global.Counter), NONCE_SIZE);

    /* Key counter is incremented as it is used for the SNnonce */
    incr_cnt(g_rsna_supp.global.Counter, NONCE_SIZE);


    rsna_ptr->sta_1x->global.portEnabled = BTRUE;
    rsna_ptr->sta_1x->global.portValid   = BFALSE;

    rsna_ptr->AuthenticationRequest = BFALSE;

    /* Set State to S4_AUTHENTICATION */
    rsna_ptr->rsna_supp_4way_state  = S4_AUTHENTICATION;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_supp_disconnected                                */
/*                                                                           */
/*  Description      : This function implements the DISCONNECTED state of    */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_supp_disconnected(rsna_supp_persta_t *rsna_ptr)
{
    rsna_ptr->AuthenticationFailed = BFALSE;

    STADisconnect(rsna_ptr);

    /* Set State to S4_DISCONNECTED */
    rsna_ptr->rsna_supp_4way_state  = S4_DISCONNECTED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_supp_initialize                                  */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of      */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_supp_initialize(rsna_supp_persta_t *rsna_ptr)
{
    rsna_ptr->keycount                  = 0;
    rsna_ptr->Init                      = BFALSE;
    rsna_ptr->DeauthenticationRequest   = BFALSE;
    rsna_ptr->sta_1x->global.portValid   = BFALSE;
    rsna_ptr->sta_1x->global.portEnabled = BFALSE;

    /* Set State to S4_INITIALIZE */
    rsna_ptr->rsna_supp_4way_state  = S4_INITIALIZE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_supp_stakeystart                                 */
/*                                                                           */
/*  Description      : This function implements the STAKEYSTART state of     */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_supp_stakeystart(rsna_supp_persta_t *rsna_ptr)
{
    STAProcessEAPOLKey(rsna_ptr);

    if(rsna_ptr->State == FAILED)
    {
        rsna_supp_disconnected(rsna_ptr);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : STADisconnect                                         */
/*                                                                           */
/*  Description      : This procedure deauthenticates the STA.               */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void STADisconnect(rsna_supp_persta_t *rsna_ptr)
{
    /* Set the disconnect flag, which will send the de-auth frame after the */
    /* FSM Run is completed                                                 */
    rsna_ptr->send_deauth = BTRUE;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : STAProcessEAPOLKey                                    */
/*                                                                           */
/*  Description      : This procedure implements the STAProcessEAPOLKey of   */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void STAProcessEAPOLKey(rsna_supp_persta_t *rsna_ptr)
{
    BOOL_T is_key_data = BFALSE;
    BOOL_T set_rsn_ie  = BFALSE;
    BOOL_T is_secured  = BFALSE;
    BOOL_T sta_connected = BFALSE;

    /* Note the current status */
    if((rsna_ptr->sta_1x->global.portValid == BTRUE) &&
       (rsna_ptr->sta_1x->global.keyDone   == BTRUE))
    {
       sta_connected = BTRUE;
    }

    /* The execution point has reached here, implies that the STATE is not */
    /* Failed, and is either MICOK is the MIC has passed or if Message was */
    /* recieved without the MIC, has a UNKNOWN state                       */

    /* Before processing the message, it is ascertained if the received    */
    /* message has a encypted key data and if it is not encrypted then the */
    /* either STA or GTK Should not be the pay loads                       */
    if(check_key_data(rsna_ptr, &is_key_data) == BFALSE)
    {
        rsna_ptr->State = FAILED;
        return;
    }

    /* Process the pairwise key message */
    if(get_k(rsna_ptr->eapol_key_frame) == P)
    {
        /* If PSK Is used, refresh the PMK, */
        /* else obtain the PMK from AAA     */
        if(rsna_ptr->auth_policy == 0x02) //chenq mark psk
        {
            /* PMK <- L(PSK, 0, 256)        */
            memcpy(rsna_ptr->PMK, mget_RSNAConfigPSKValue(), 32);//chenq mark password => pmk 32 byte
        }
        else if(rsna_ptr->auth_policy == 0x01)
        {
            /* Update the PMK from the AAA     */
            /* PMK <- L(AAA Key, 0, 256)       */
            get_pmk(((supp_t *)(rsna_ptr->sta_1x))->rx_key, rsna_ptr->PMK);
        }

        /* Compare the ANonce received and the ANonce stored */
        /* If they are different, TPTK needs to be computed  */
        if(memcmp(get_nonce(rsna_ptr->eapol_key_frame),
                   rsna_ptr->PreANonce, NONCE_SIZE) != 0) //chenq mark rx 1st eapol ptk(only anonce)
        {
            /* 'key_length' specifies the length of the TK to be used. */
            /* The PTK length is initialized according to the cipher   */
            /* suite in use.                                           */
            UWORD16 ptk_size = ((rsna_ptr->key_length) + 32) * 8; /* (Bits) */

            memcpy(rsna_ptr->PreANonce, get_nonce(rsna_ptr->eapol_key_frame),
                    NONCE_SIZE);
            get_ptk(rsna_ptr->PMK, rsna_ptr->PTK, ptk_size, rsna_ptr->addr,
                mget_StationID(), rsna_ptr->PreANonce, rsna_ptr->SNonce);
        }
        /* If the received message was Message # 3 */
        if(rsna_ptr->State == MICOK) //chenq mark rx 3rd eapol ptk(anonce,rsn ie,use mic)
        {
            UWORD8 index = EAPOL_HDR_LEN        /* Skip the EAPOL Header     */
                         + KEY_DESCRIPTOR_SIZE; /* Skip desciptor type field */
            /* If the install bit was set in the received EAPOL Key packet   */
            /* Install the PTK for the remote station                        */
            if(rsna_ptr->eapol_key_frame[index + 1] & 0x70)
            {
                UWORD8 cipher_type = 0;
                UWORD8 key_id      = 0;

                /* Depending on ciphers policies set the key ids & policies */
                key_id      = 0;
                cipher_type = cipsuite_to_ctype(rsna_ptr->pcip_policy);

                if(!((mget_DesiredBSSType() == INDEPENDENT) &&
                     (memcmp(mget_StationID(), rsna_ptr->addr, 6) > 0)))
                {
                    machw_ce_add_key(key_id,
                                     PTK_NUM,
                                     rsna_ptr->key_index,
                                     0,
                                     cipher_type,
                                     (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH +
                                     KEK_LENGTH),
                                     rsna_ptr->addr,
                                     SUPP_KEY,
                                     (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH +
                                     KEK_LENGTH + TEMPORAL_KEY_LENGTH));


                   /* Initiate a new RSNA Stats MIB */
                    mnew_RSNAStats((UWORD8)rsna_ptr->key_index);
                    mset_RSNAStatsSTAAddress(rsna_ptr->addr,
                        (UWORD8)rsna_ptr->key_index);

                   /* Create and set the cipher element in the RSNA Stats */
                   mset_RSNAStatsSelectedPairwiseCipher(rsna_ptr->pcip_policy,
                        (UWORD8)rsna_ptr->key_index);

                   /* Update the MIB with the last selected PCIP and ATUH policies */
                   mset_RSNAPairwiseCipherSelected(rsna_ptr->pcip_policy);
                   mset_RSNAAuthenticationSuiteSelected(rsna_ptr->auth_policy);
                }
            }

            if(is_key_data == BTRUE)
            {
                if(DecryptGTK(rsna_ptr) != BTRUE)
                {
                    TROUT_DBG4("P2P: DecryptGTK FALSE");
                    rsna_ptr->deauth_reason = MIC_FAIL;
                    rsna_ptr->State = FAILED;
                    return;
                }
                else
                {

                    /* Set the group key */
                    set_grp_key(rsna_ptr);
                    /* Set the KeyDone value to True */
                    rsna_ptr->sta_1x->global.keyDone   = BTRUE;
                }
            }
        }
        else
        {
            /* In WPA mode RSN Element must be sent */
            set_rsn_ie = BTRUE;
        }
    }
    /* If the received EAPOL Key is not a 'P' frame */
    else
    {
        if((rsna_ptr->mode_802_11i == WPA_802_11I) || (is_key_data == BTRUE))
        {
            /* The Ack EAPOL Key packet must be encrypted */
            is_secured = BTRUE;
            if(rsna_ptr->State == MICOK)
            {
                /* Try decryption GTK , if GTK is not found search for SK */
                UWORD8 ret_val = DecryptGTK(rsna_ptr);

                if(ret_val == BTRUE)
                {
                    /* Set the group key */
                    set_grp_key(rsna_ptr);

                    /* Set the KeyDone value to True */
                    rsna_ptr->sta_1x->global.keyDone   = BTRUE;
                }
                else if(ret_val == BFALSE)
                {
                    TROUT_DBG4("P2P: DecryptGTK 2 FALSE");
                    /* If the Decryption failed, abort the processing */
                    rsna_ptr->deauth_reason = MIC_FAIL;
                    rsna_ptr->State = FAILED;
                    return;
                }
                else if(ret_val == 0xFF)
                {
                    /* If the Decryption passed but GTK was not found */
                    /* check for SK                                   */
                    if(DecryptSK(rsna_ptr) == BTRUE)
                    {
                        /* IntegrateTBD: Set the Sta Key */
                    }

                    /* For now, exit with an error code */
                    rsna_ptr->deauth_reason = UNSPEC_REASON;
                    rsna_ptr->State = FAILED;
                    return;
                }
            }
        }
        else
        {
            TROUT_DBG4("P2P: is_key_data FALSE");
            rsna_ptr->deauth_reason = MIC_FAIL;
            rsna_ptr->State = FAILED;
            return;
        }
    }

    /* If the ACK bit is set send the EAPOL Response message */
    if(rsna_ptr->AckReq == BTRUE)
    {
        if(BFALSE == send_supp_eapol(rsna_ptr, set_rsn_ie, is_secured))
        {
           return ;
        }
    }

    /* If the Secure bit was set in the received EAPOL Key packet */
    if(
#if 0
       ((rsna_ptr->mode_802_11i == WPA_802_11I) &&
        (rsna_ptr->eapol_key_frame[EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE + 1] & 0x70) &&
        (rsna_ptr->State == MICOK)) ||
       ((rsna_ptr->mode_802_11i == RSNA_802_11I) &&
        ((rsna_ptr->eapol_key_frame[EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE] & 0x02) &&
        (rsna_ptr->State == MICOK)))
#else
        (rsna_ptr->eapol_key_frame[EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE] & 0x01) &&
        (rsna_ptr->State == MICOK))
#endif
    {
#ifdef IBSS_11I
        /* If the Station is an IBSS Station */
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            rsna_ptr->keycount++;
            if(get_auth_key_count(rsna_ptr->key_index) >= 1)
            {
                rsna_ptr->sta_1x->global.portValid = BTRUE;
                auth_finalize_fsm(rsna_ptr->key_index);
            }
        }
        else
#endif /* IBSS_11I */
        {
            /* Set the PortValid value to True */
            rsna_ptr->sta_1x->global.portValid = BTRUE;
        }
    }

    /* If the 11i handshake is complete take the appropriate actions */
   if((rsna_ptr->sta_1x->global.portValid == BTRUE) &&
      (rsna_ptr->sta_1x->global.keyDone   == BTRUE) &&
      (sta_connected == BFALSE))
   {
        /* Cancel the management timer for handshake timeout */
        cancel_mgmt_timeout_timer();

        /* Start the activity timer for power management */
        start_activity_timer();

        /* Convey to the host that device is ready for Tx/Rx */
		//chenq mask 0727
        //send_mac_status(MAC_CONNECTED);
        update_persist_cred_list_prot_sta();

#ifdef NO_ACTION_RESET
        /* Restore all the saved action requests */
        restore_all_saved_action_req();
#endif /* NO_ACTION_RESET */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_key_data                                        */
/*                                                                           */
/*  Description      : This procedure verifies if a valid Key Data field is  */
/*                     present                                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure verifies if a valid Key Data field is  */
/*                     present                                               */
/*                                                                           */
/*  Outputs          : True is the Key Data is present; False Otherwise      */
/*                                                                           */
/*  Returns          : True is the Key Data is valid; False Otherwise        */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_key_data(rsna_supp_persta_t *rsna_ptr, BOOL_T *is_present)
{
    BOOL_T ret_val = BTRUE;
    UWORD8 *eapol_buffer = rsna_ptr->eapol_key_frame;
    UWORD8 key_data_len  = 0;
    UWORD8 encr_data = 0;

    key_data_len = eapol_buffer[EAPOL_HDR_LEN + MIC_SIZE + MIC_OFFSET + 1] +
                  (eapol_buffer[EAPOL_HDR_LEN + MIC_SIZE + MIC_OFFSET] << 8);

    /* Extract Encrypted Data Bit */
    encr_data =
            (eapol_buffer[EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE] & 0x10);

    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
        /* Check if the data is present and if it is GTK KDE */
        if(key_data_len == 0)
        {
            *is_present   = BFALSE;
            if(encr_data != 0)
            {
                ret_val = BFALSE;
                /* Set the Disconect reason */
                rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
            }
        }
        else
        {
            if(encr_data == 0)
            {
                UWORD16 index       = 0;
                BOOL_T  first_rsnie = BTRUE;

                *is_present = BFALSE;

                eapol_buffer += EAPOL_HDR_LEN + KEY_DATA_OFFSET;

                /* Search if STA KEY or GTK KDE or PMKID are present */
                /* If they are , the frame must be ignored           */
                while(index < key_data_len)
                {
                    /* Check the validy of the received packet */
                    if(eapol_buffer[index] == 0xDD)
                    {
                        /* Check the validy of the OUI */
                        if(check_oui(&eapol_buffer[index + 2],
                                                mget_RSNAOUI()) != BTRUE)
                        {
                            /* It may have been a vendor specific element */
                            /* So continue searching for RSNAIE */
                            index += eapol_buffer[index + 1] + 2;
                            continue;
                        }

                        /* Check if none of the fields for which  */
                        /* encrypted encapsulation is a must have */
                        /* been send unencrypted                  */
                        if((eapol_buffer[index + 5] == 1) ||
                           (eapol_buffer[index + 5] == 2))
                        {
                            /* Set the Disconect reason */
                            rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
                            ret_val = BFALSE;
                            break;
                        }
                        /* Extract PMKID */
                        if((eapol_buffer[index + 5] == 4) &&
                           (eapol_buffer[index + 1] != (4 + PMKID_LEN)))
                        {
                            /* Set the Disconect reason */
                            rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
                            ret_val = BFALSE;
                            break;
                        }
                        else
                        {
                            memcpy(rsna_ptr->pmkid, &eapol_buffer[index + 6],
                                   PMKID_LEN);
                        }

                    }
                    else if(eapol_buffer[index + 0] == IRSNELEMENT)
                    {
                        if(verify_RSNIE(&eapol_buffer[index],
                                        first_rsnie, rsna_ptr->mode_802_11i)
                           == BFALSE)
                        {
                            /* Set the Disconect reason */
                            rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
                            ret_val = BFALSE;
                            break;
                        }
                        first_rsnie = BFALSE;
                    }
                    else
                    {
                        ret_val = BFALSE;
                        break;
                    }
                    index += eapol_buffer[index + 1] + 2;
                }
            }
            else
            {
                *is_present = BTRUE;
            }
        }
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        /* Check if the data is present and if it is GTK KDE */
        if(key_data_len == 0)
        {
            *is_present   = BFALSE;
            if(encr_data != 0)
            {
                /* Set the Disconect reason */
                rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
                ret_val = BFALSE;
            }
        }
        else
        {
            if(encr_data == 0)
            {
                eapol_buffer += EAPOL_HDR_LEN + KEY_DATA_OFFSET;

                /* If WPA IE is present, then verify it */
                if(is_wpa_ie(eapol_buffer) == BTRUE)
                {
                    if(verify_RSNIE(eapol_buffer, BTRUE,
                                    rsna_ptr->mode_802_11i) == BFALSE)
                    {
                        /* Set the Disconect reason */
                        rsna_ptr->deauth_reason = INFO_ELMNT_FAIL;
                        ret_val = BFALSE;
                    }
                }
                *is_present = BFALSE;
            }
            else
            {
                *is_present = BTRUE;
            }
        }
    }
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : verify_RSNIE                                          */
/*                                                                           */
/*  Description      : This procedure verfies if the received RSN IE is      */
/*                     correct                                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to the eapol buffer                        */
/*                     2) Is the RSNIE first IE received                     */
/*                     3) 802.11i mode                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure verfies if the received RSN IE is      */
/*                     correct                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True is the RSN IE is correct; False Otherwise        */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T verify_RSNIE(UWORD8 *eapol_buffer, BOOL_T first_rsnie,
                    UWORD8 mode_802_11i)
{
    UWORD8 index       = 0;
    UWORD8 i           = 0;
    BOOL_T match_found = BFALSE;
    UWORD8 *oui        = 0;

    if(mode_802_11i == RSNA_802_11I)
    {
        if(eapol_buffer[0] != IRSNELEMENT)
        {
            return BFALSE;
        }

        oui   = mget_RSNAOUI();

        index = 2;
    }
    else if(mode_802_11i == WPA_802_11I)
    {
        if(is_wpa_ie(eapol_buffer) == BFALSE)
            return BFALSE;

        oui   = mget_WPAOUI();

        index = 6;
    }

    /* Check the RSN Version number */
    if(eapol_buffer[index] != mget_RSNAConfigVersion())
    {
        return BFALSE;
    }

    /* Check the validy of the OUI */
    if(check_oui(&eapol_buffer[index + 2], oui) != BTRUE)
    {
        return BFALSE;
    }

    index += 5;

    /* Check Group cipher policy */
    if(first_rsnie == BTRUE)
    if(eapol_buffer[index++] != mget_RSNAGroupCipherSelected())
    {
        return BFALSE;
    }

    i      = eapol_buffer[index];
    index += 2;

     /* Verify if atleast one of the policies is supported at the sta */
    for(; i > 0; i--)
    {
        /* Check the validy of the OUI */
        if(check_oui(&eapol_buffer[index], oui) != BTRUE)
        {
            return BFALSE;
        }
        index += 3;

        if(first_rsnie == BTRUE)
        {
            if(check_pcip_policy(eapol_buffer[index++]) == BTRUE)
            {
                match_found = BTRUE;
            }
        }
        else
        {
            match_found = BTRUE;
        }
    }

    /* If no match was found, return error */
    if(match_found == BFALSE)
    {
        return BFALSE;
    }

    i      = eapol_buffer[index];
    index += 2;

    /* Verify if atleast one of the policies is supported at the sta */
    for(; i > 0; i--)
    {
        /* Check the validy of the OUI */
        if(check_oui(&eapol_buffer[index], oui) != BTRUE)
        {
            return BFALSE;
        }
        index += 3;

        if(first_rsnie == BTRUE)
        {
            if(check_auth_policy(eapol_buffer[index++]) == BTRUE)
            {
                match_found = BTRUE;
            }
        }
        else
        {
            match_found = BTRUE;
        }
    }

    /* If no match was found, return error */
    if(match_found == BFALSE)
    {
        return BFALSE;
    }

    /* For 802.11I RSNA, RSN Capability field must match */
    if(mode_802_11i == RSNA_802_11I)
    {

/* RSN Capabilities Information                                          */
/*************************************************************************/
/* --------------------------------------------------------------------- */
/* | B15 - B6  |  B5 - B4      | B3 - B2     |       B1    |     B0    | */
/* --------------------------------------------------------------------- */
/* | Reserved  |  GTSKA Replay | PTSKA Replay| No Pairwise | Pre - Auth| */
/* |           |    Counter    |   Counter   |             |           | */
/* --------------------------------------------------------------------- */
/*                                                                       */
/*************************************************************************/

        /* Check Pre-Auth status in the capability */
        if((mget_RSNAPreauthenticationImplemented() == TV_TRUE) &&
           (mget_RSNAPreauthenticationEnabled() == TV_TRUE))
        {
            if(!(eapol_buffer[index] & BIT0))
            {
                return BFALSE;
            }
        }

        /* Pairwise field in the Capability field. AP must set this to 0 */
        if(eapol_buffer[index] & BIT1)
        {
            return BFALSE;
        }

        /* Check PTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfPTSKAReplayCounters  */
        if((eapol_buffer[index] & (BIT2 | BIT3)) !=
           ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2)
               & 0x000C))
        {
            /* Since we havent implemented this, just ignore */
            //return BFALSE;
        }

        /* Check GTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfGTSKAReplayCounters  */
        if((eapol_buffer[index] & (BIT4 | BIT5)) !=
            ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4)
               & 0x0030))
        {
             /* Since we havent implemented this, just ignore */
            //return BFALSE;
        }
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : DecryptGTK                                            */
/*                                                                           */
/*  Description      : This procedure decrpypts the GTK from the received    */
/*                     EAPOL packet                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure decrpypts the GTK from the received    */
/*                     EAPOL packet                                          */
/*                                                                           */
/*  Outputs          : Updates the GTK in the RSNA Handle                    */
/*                                                                           */
/*  Returns          : True is the decryption passes; False Otherwise; 0xFF  */
/*                     if decryption passed but GTK not found                */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 DecryptGTK(rsna_supp_persta_t *rsna_ptr)
{
    UWORD8 ret_val       = BTRUE;
    UWORD8 *eapol_buffer = rsna_ptr->eapol_key_frame;
    UWORD16 key_data_len = 0;
    UWORD8 *kek          = get_kek(rsna_ptr->PTK);
    UWORD8 *key_data     = &eapol_buffer[EAPOL_HDR_LEN + KEY_DATA_OFFSET];

    key_data_len = eapol_buffer[EAPOL_HDR_LEN + MIC_SIZE + MIC_OFFSET + 1] +
                  (eapol_buffer[EAPOL_HDR_LEN + MIC_SIZE + MIC_OFFSET] << 8);

    /* RC4 is used to encrypt/decrypt the Key Data */
    /* This is done in place                       */
    if(rsna_ptr->key_version == 1)
    {
        /* RC4 - is used as Key encryption algorigthm*/
        UWORD8 rc4_ek[32] = {0};
        UWORD8 *key_iv = &eapol_buffer[EAPOL_HDR_LEN + EAPOL_KEY_IV_OFFSET];

            /* RC4 Key Encryption uses IV field and KEK together          */
            /*                 RC4 KEY - 32 Bytes                         */
            /*  16 bytes from Key counter (LSB) | 16 bytes KEK from TK    */
            memcpy(rc4_ek, key_iv, 16);
            memcpy(rc4_ek + 16, kek, 16);

            /* RC4 is used to encrypt the Key Data */
            rc4_skip(rc4_ek, 32, 256, (UWORD8 *)(key_data), key_data_len);
    }
    else if(rsna_ptr->key_version == 2)
    {
        /* Adjust the padding length and unwrap the GTK information */
        key_data_len -= 8;
        if(aes_key_unwrap(key_data, kek, (key_data_len/8)) != BTRUE)
        {
            ret_val = BFALSE;
        }
    }
    else
    {
        ret_val = BFALSE;
    }


    /* Proceed only if the decryption has succeeded       */
    /* RSNA/WPA are have different key data encapsulation */
    /* So are differently decapsulated                    */
    if(ret_val == BTRUE)
    {
        if(rsna_ptr->mode_802_11i == RSNA_802_11I)
        {
            UWORD16 index       = 0;
            BOOL_T  first_rsnie = BTRUE;

            ret_val       = 0xFF;
            eapol_buffer += EAPOL_HDR_LEN + KEY_DATA_OFFSET;


            /* Search if STA KEY or GTK KDE or PMKID are present */
            /* If they are , the frame must be ignored           */
            while(index < key_data_len)
            {
                /* Perform a validy check and copy the GTK if found */
                if(eapol_buffer[index + 0] == 0xDD)
                {
                    /* Verify the OUI for the RSNA */
                    if(check_oui(&eapol_buffer[index + 2],
                                     mget_RSNAOUI()) != BTRUE)
                    {
                        /* It may have been a vendor specific element */
                        /* So continue searching for RSNAIE */
                        index += eapol_buffer[index + 1] + 2;
                        continue;
                    }

                    if(eapol_buffer[index + 5] == 1)
                    {
                        /* Verify the length */
                        if(eapol_buffer[index + 1] < rsna_ptr->gkey_length+6)
                        {
                            ret_val = BFALSE;
                            break;
                        }
                        else
                        {
                            rsna_ptr->GN = eapol_buffer[index + 6] & 0x03;

                            if( rsna_ptr->GN > GTK_MAX_INDEX )
                            {
                                ret_val = BFALSE;
                                break;
                            }
// 20120709 caisf masked, merged ittiam mac v1.2 code
/* Note :  This check has been disabled as association was not happening with */
/* P2P Broadcom AP                                                            */
/* P2P_KLUDGE                                                                 */
#if 0
                            /* The Key number must be correct */
                            /* else return the error message  */
                            if((rsna_ptr->GN == 0) &&
                               ((rsna_ptr->grp_policy != 0x01) &&
                                (rsna_ptr->grp_policy != 0x05)))
                            {
                                ret_val = BFALSE;
                                break;
                            }
#endif /* 0 */

                            /* Copy the GTK as validity check passed */
                            memcpy(rsna_ptr->GTK[get_GTK_array_index(rsna_ptr->GN)],
                              &eapol_buffer[index + 8], rsna_ptr->gkey_length);

                            /* Also update the Bcast PN Value */
                            {
                                UWORD8 i = 0;
                                for(i = 0; i < 6; i++)
                                {
                                    rsna_ptr->bcst_rx_pn_val[i] =
                                        rsna_ptr->eapol_key_frame
                                        [EAPOL_HDR_LEN + RSC_OFFSET + i];
                                }
                            }
                            ret_val = BTRUE;
                            break;
                        }
                    }
                }
                else if(eapol_buffer[index + 0] == IRSNELEMENT)
                {
                    /* Performing the check for the RSNA Information element */
                    /* A flag is maintained to see if this is the first RSN  */
                    /* element. The second RSN element is checked only for   */
                    /* OUIs                                                  */
                    if(verify_RSNIE(&eapol_buffer[index], first_rsnie,
                                    rsna_ptr->mode_802_11i) == BFALSE)
                    {
                        ret_val = BFALSE;
                        break;
                    }
                    first_rsnie = BFALSE;
                }

                    index += eapol_buffer[index + 1] + 2;
                }
            }
        else if(rsna_ptr->mode_802_11i == WPA_802_11I)
        {
            /* WPA if encrpyted only has GTK, STA keys are not supported */
            /* so just copy the GTK                                      */
            UWORD8 index = EAPOL_HDR_LEN + KEY_DATA_OFFSET;

            /* Backup the old index value and obtain the new value       */
            rsna_ptr->GN =
                (eapol_buffer[EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE + 1] & 0x30)
                >> 4;

            if( rsna_ptr->GN > GTK_MAX_INDEX )
            {
                ret_val = BFALSE;
            }
            else
            {
            /* The Key number must be correct, else return the error message */
            if((rsna_ptr->GN == 0) &&
               ((rsna_ptr->grp_policy != 0x01) &&
                (rsna_ptr->grp_policy != 0x05)))
            {
                ret_val = BFALSE;
            }
            else
            {
                    memcpy(rsna_ptr->GTK[get_GTK_array_index(rsna_ptr->GN)], &eapol_buffer[index],
                        rsna_ptr->gkey_length);

                /* Also update the Bcast PN Value */
                    {
                        UWORD8 i = 0;
                        for(i = 0; i < 6; i++)
                        {
                            rsna_ptr->bcst_rx_pn_val[i] =
                                eapol_buffer[EAPOL_HDR_LEN + RSC_OFFSET + i];
                        }
                    }
            }
        }
    }
    else
    {
        ret_val = BFALSE;
    }
    }
    else
    {
        ret_val = BFALSE;
    }

    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_supp_eapol                                       */
/*                                                                           */
/*  Description      : This procedure prepares an EAPOL Frame and sends it   */
/*                     to the remote station                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                     2) Flag if RSNA Element is to be set in the packet    */
/*                     3) Flag if the outgoing packet is secured             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure prepares an EAPOL Frame and sends it   */
/*                     to the remote station                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_supp_eapol(rsna_supp_persta_t *rsna_ptr, BOOL_T set_rsn_ie,
                    BOOL_T is_secured)
{
    UWORD16 key_info      = 0;
    UWORD16 key_data_len  = 0;
    UWORD16 index         = 0;
    UWORD8  *replay_cnt   = 0;
    UWORD16 eapol_size    = 0;
    UWORD8  *buffer       = 0;
    UWORD8  *key_data     = 0;
    UWORD8  *eapol_buffer = 0;
    UWORD16 eapol_length  = 0;
    UWORD16 curr_idx      = 0;

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Reset Timeout Event */
    rsna_ptr->TimeoutEvt = BFALSE;

    /* Send EAPOL(0, 1, 0, 0, K, 0, SNonce, MIC(PTK), RSNIE, 0) */
    /* Allocate buffers for the EAPOL Key Frame, Key Data and MIC.         */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* If general buffers are not available then take from management buffers */
    if(buffer == NULL)
    {
        buffer = mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);
    }

    if(buffer == NULL)
    {
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_supp_disconnected(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    key_data = (UWORD8 *)scratch_mem_alloc(128);
    if(key_data == NULL)
    {
        pkt_mem_free(buffer);
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_supp_disconnected(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    eapol_buffer = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* Prepare the Key Information field with the required parameters.   */
    key_info = (rsna_ptr->eapol_key_frame
                        [EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE] & 0xFF) << 8;
    key_info |= (rsna_ptr->eapol_key_frame
                        [EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE + 1] & 0xFF);

    key_info &= 0x0F0F;
    key_info |= 0x0100;

    if(set_rsn_ie == BTRUE)
    {

        /* Prepare the data for the EAPOL key frame. The EAPOL Key Data */
        /* consists of the WPAIE/RSNIE depending on the mode            */
        index = 0;
        /* Set the RSNIE */
        index += set_rsn_element_sta(key_data, 0, rsna_ptr->mode_802_11i);
        key_data_len += index;
    }

    /* Get the replay count for this station */
    replay_cnt = get_rply_cnt(rsna_ptr->eapol_key_frame);


    /* Prepare the EAPOL Key frame with the Key MIC field being zeroed.      */
    eapol_length = prepare_eapol_key_frame(key_info, 0, NULL,
                            rsna_ptr->SNonce, NULL, key_data_len, key_data,
                            (UWORD8 *)replay_cnt, eapol_buffer,
                            rsna_ptr->mode_802_11i);

    /* Set the EAPOL Key frame size */
    eapol_size = KEY_DATA_OFFSET + key_data_len;

    /* Add SNAP Header and the EAPOL Header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_KEY, eapol_length,
                   rsna_ptr->mode_802_11i);
    eapol_length += EAPOL_1X_HDR_LEN;

    /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC  */
    /* field zeroed. The MIC is computed using the KCK derived from PTK.*/
    add_eapol_mic(eapol_buffer, eapol_size, rsna_ptr->PTK,
            rsna_ptr->key_version);

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(rsna_ptr->addr, buffer, eapol_length, is_secured))
    {
        /* Buffer should not be deleted here as it is already deleted inside */
        /* the above function */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_supp_disconnected(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return BTRUE;
}
#endif /* SUPP_11I */
#endif /* IBSS_BSS_STATION_MODE */


