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
/*  File Name         : rsna_auth_grpkey.c                                   */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of the Per-STA Group Key  */
/*                      Handshake state machine.                             */
/*                                                                           */
/*  List of Functions : rsna_auth_grpkey_fsm                                 */
/*                      rsna_auth_grpkey_idle                                */
/*                      rsna_auth_grpkey_rekeynegotiating                    */
/*                      rsna_auth_grpkey_keyerror                            */
/*                      rsna_auth_grpkey_rekeyestablished                    */
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
#include "eapol_key_auth.h"
#include "rkmal_auth.h"
#include "prf.h"
#include "utils.h"
#include "aes.h"
#include "md5.h"
#include "mib_11i.h"
#include "ce_lut.h"
#include "rc4.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void rsna_auth_grpkey_idle(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_grpkey_rekeynegotiating(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_grpkey_rekeyestablished(rsna_auth_persta_t *rsna_ptr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_grpkey_fsm                                  */
/*                                                                           */
/*  Description      : This function implements the Per-STA Group Key        */
/*                     Handshake state machine. In each of the various       */
/*                     states, it checks for the appropriate inputs before   */
/*                     switching to next state. This state machine is        */
/*                     instantiated per supplicant.                          */
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

void rsna_auth_grpkey_fsm(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr->InitGK == BTRUE)
    {
        rsna_auth_grpkey_idle(rsna_ptr);
        rsna_ptr->InitGK = BFALSE;
        return;
    }

    switch(rsna_ptr->rsna_auth_grpkey_state)
    {
    case AGK_IDLE:
    {
        if(rsna_ptr->mode_802_11i == RSNA_802_11I)
        {
            if(rsna_ptr->GUpdateStationKeys == BTRUE)
            {
                rsna_auth_grpkey_rekeynegotiating(rsna_ptr);
            }
            else
            {
                /* Do Nothing */
            }
        }
        if(rsna_ptr->mode_802_11i == WPA_802_11I)
        {
            if((rsna_ptr->GUpdateStationKeys == BTRUE)
                || ((rsna_ptr->GInitAKeys == BTRUE)
                && (rsna_ptr->PInitAKeys == BTRUE)))
            {
                rsna_auth_grpkey_rekeynegotiating(rsna_ptr);
            }
            else
            {
                /* Do Nothing */
            }
        }
    } /* End of case AGK_IDLE: */
    break;

    case AGK_KEYERROR:
    {
        rsna_auth_grpkey_idle(rsna_ptr);
    } /* End of case AGK_KEYERROR: */
    break;

    case AGK_REKEYESTABLISHED:
    {
        rsna_auth_grpkey_idle(rsna_ptr);
    } /* End of case AGK_REKEYESTABLISHED: */
    break;

    case AGK_REKEYNEGOTIATING:
    {
            if((enough_attempts_grpkey(rsna_ptr) == BTRUE) ||
               (rsna_ptr->GTimeout == BTRUE))
        {
            /* Set the reason and initiate a disconnect */
            rsna_ptr->deauth_reason = GRP_HS_TIMEOUT;
            rsna_auth_grpkey_keyerror(rsna_ptr);
        }
        else if(rsna_ptr->TimeoutEvt == BTRUE)
        {
            rsna_auth_grpkey_rekeynegotiating(rsna_ptr);
        }
        else if((rsna_ptr->EAPOLKeyReceived == BTRUE)     &&
                (!get_request(rsna_ptr->eapol_key_frame)) &&
                (get_k(rsna_ptr->eapol_key_frame) == G)   &&
                (check_eapol_pkt_auth(rsna_ptr)   == BTRUE))
        {
            /* Calculate and verify EAPOL Mic field in the received packet */
            rsna_ptr->MICVerified = verifyMIC(rsna_ptr->eapol_key_frame,
                                        rsna_ptr->eapol_frame_len,
                                        rsna_ptr->PTK,
                                        rsna_ptr->key_version);

            /* Check the MIC for the received packet */
            if(rsna_ptr->MICVerified == BTRUE)
            {
                rsna_auth_grpkey_rekeyestablished(rsna_ptr);
            }
            else
            {
                /* Set the Disconect reason */
                rsna_ptr->deauth_reason = MIC_FAIL;
                /* Initiate a disconnect */
                rsna_auth_grpkey_keyerror(rsna_ptr);
            }

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Reset MIC Valid flag */
            rsna_ptr->MICVerified = BFALSE;
        }
        else if(rsna_ptr->EAPOLKeyReceived == BTRUE)
        {
            BOOL_T timer_started = BFALSE;

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Restart timer */
            timer_started = start_11i_auth_timer(rsna_ptr,
                get_11i_resp_timeout((UWORD8)rsna_ptr->GTimeoutCtr,
                rsna_ptr->listen_interval));

            if(timer_started == BFALSE)
            {
                /* Timer could not be started, so assume that this sta is done with GTK update */
                /* so that whole process of GTK update dont get stuck for this STA alone      */
                rsna_auth_grpkey_rekeyestablished(rsna_ptr);

            }
        }

        else
        {
            /* Do Nothing */
        }
    } /* End of case AGK_REKEYNEGOTIATING: */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(rsna_ptr->rsna_auth_grpkey_state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_grpkey_idle                                 */
/*                                                                           */
/*  Description      : This function implements the IDLE state of the        */
/*                     per-sta group key handshake state machine.            */
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

void rsna_auth_grpkey_idle(rsna_auth_persta_t *rsna_ptr)
{
    rsna_ptr->GTimeout = BFALSE;

    rsna_ptr->GTimeoutCtr = 0;

    /* Set State to AGK_IDLE */
    rsna_ptr->rsna_auth_grpkey_state  = AGK_IDLE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_grpkey_rekeynegotiating                     */
/*                                                                           */
/*  Description      : This function implements the REKEYNEGOTIATING state   */
/*                     of the per-sta group key handshake state machine.     */
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

void rsna_auth_grpkey_rekeynegotiating(rsna_auth_persta_t *rsna_ptr)
{
    UWORD8  *key_rsc      = 0;
    UWORD8  *replay_cnt   = 0;
    UWORD16 key_info      = 0;
    UWORD16 key_data_len  = 0;
    UWORD16 eapol_size    = 0;
    UWORD16 eapol_length  = 0;
    UWORD8  gtk_len       = 0;
    UWORD8  pad           = 0;
    UWORD8  i             = 0;
    UWORD8  *buffer       = 0;
    UWORD8  *eapol_buffer = 0;
    //UWORD8  *kck          = 0;
    UWORD8  *kek          = 0;
    UWORD8  *key_data     = 0;
    UWORD8  *gnonce       = 0;
    UWORD8  *key_iv       = NULL;
    UWORD8  key_rsc_val[RSC_SIZE]= {0};
    UWORD16 curr_idx = 0;
    BOOL_T timer_started = BFALSE;

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Reset Timeout Event */
    rsna_ptr->TimeoutEvt = BFALSE;

    if( g_rsna_auth.global.GN > GTK_MAX_INDEX )
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnected(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    /* Send EAPOL(1, 1, 1, !Pair, G RSC, GNonce, MIC(PTK), GTK[GN])          */
    /* Allocate buffers for the EAPOL Key Frame, Key Data and MIC.           */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* If general buffers are not available then take from management buffers */
    if(buffer == NULL)
    {
        buffer = mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);
    }

    if(buffer == NULL)
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnected(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    key_data     = (UWORD8 *)scratch_mem_alloc(128);
    if(key_data == NULL)
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnected(rsna_ptr);
        pkt_mem_free(buffer);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    eapol_buffer = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* Prepare the Key Information field with the required parameters.       */
    key_info = prepare_key_info(rsna_ptr->key_version, 1, 1, 1, 0, G,
                                g_rsna_auth.global.GN, 1,
                                rsna_ptr->mode_802_11i);

    /* Obtain the KEK from the PTK. This will be used in key data encryption.*/
    kek = get_kek(rsna_ptr->PTK);

    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {

        /* Prepare the data for the EAPOL Key frame. This is encrypted GTK.*/

        /* Prepare the GTK KDE. This is set after the RSNIE in same buffer  .*/
        /* ----------------------------------------------------------------- */
        /* | Key ID  | Tx    | Reserved (0) | Reserved (0) | GTK           | */
        /* ----------------------------------------------------------------- */
        /* | Bit 0-1 | Bit 2 | Bit 3-7      | 1 octet      | Len - 6 octets| */
        /* ----------------------------------------------------------------- */

        /* dot11RSNAConfigGroupCipherSize specifies the length of GTK  */
        /* This is initialized according to the cipher suite in use.   */
        gtk_len = mget_RSNAConfigGroupCipherSize() >> 3; /* Group size / 8 */

        /* Encapsulate the GTK KDE in the required format */
        prepare_kde(GTK_KDE, (UWORD8)(gtk_len + 2), key_data);

        key_data[KDE_OFFSET]     = g_rsna_auth.global.GN; /* Key ID. */

        /* The Tx/Rx bit is set according to the cipher suite in use. If a   */
        /* pairwise key is used for encryption/integrity, then the STA never */
        /* transmits with the GTK; otherwise, STA uses the GTK for transmit. */
        /* Currently this is maintained as a global.                         */
        if(g_use_pairwisekey == BFALSE)
            key_data[KDE_OFFSET] |= BIT2;

        key_data[KDE_OFFSET + 1] = 0;

        memcpy(key_data + KDE_OFFSET + 2,
            g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)], gtk_len);

        key_data_len += KDE_OFFSET + (gtk_len + 2);
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        /* dot11RSNAConfigGroupCipherSize specifies length of GTK to be used */
        /* This is initialized according to the cipher suite in use.         */
        gtk_len = mget_RSNAConfigGroupCipherSize() >> 3; /* Group size / 8   */

        /* Copy GTK to the key_data field */
        memcpy(key_data,
            g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)], gtk_len);

        /* Key data length is equal to the GTK Length */
        key_data_len = gtk_len;
    }

    if(rsna_ptr->key_version == 1)
    {
        /* RC4 - is used as Key encryption algorigthm*/
        UWORD8 *rc4_ek = 0;

        rc4_ek = (UWORD8 *)scratch_mem_alloc(32);
        if(rc4_ek == NULL)
        {
            /* Set the deauth reason and initiate a disconnect */
            rsna_ptr->deauth_reason = UNSPEC_REASON;
            rsna_auth_disconnected(rsna_ptr);
            pkt_mem_free(buffer);
            
            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);
            
            return;
        }

        /* Pad for data less than 64-bits or non-multiple of 64-bits */
        key_iv = g_rsna_auth.global.GCounter + NONCE_SIZE - 16;

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
        /* Key data field is encrypted using encryption algorithm in use.*/
        /* The length of the encrypted data is returned Encryption should*/
        /* be in place.                                                  */
        if(key_data_len < 8) /* Key data should be at least 16 bytes     */
        {
            pad = 16 - key_data_len;
        }
        else if(key_data_len % 8 != 0) /* Key data: multiple of 64-bits */
        {
            pad = 8 - key_data_len % 8;
        }

        /* Pad for data less than 64-bits or non-multiple of 64-bits */
        key_data[key_data_len]       = 0xDD;

        for(i = 0; i < pad; i++)
            key_data[key_data_len++] = 0x00;

        /* Perform AES Key wrap */
        aes_key_wrap(key_data, kek, key_data_len/8);

        /* The length after wrapping increases by 64-bits */
        key_data_len += 8;
    }

    /* Get the replay count for this station */
    replay_cnt = rsna_ptr->ReplayCount;

    if(rsna_ptr->GUpdateStationKeys == BFALSE)
    {
        /* Read the key_rsc value from the MAC HW CE register */
        get_machw_ce_pn_val(key_rsc_val);
    }

    /* The Read value is the last used PN Value */
    /* This is incremented by one to set it to the next value to be used */
    incr_byte_cnt(key_rsc_val, 6);
    key_rsc = key_rsc_val;


	/* Though the 802.11i and WPA standard specifies that the key length should*/
	/* be set to 0 in GK-HS-MSG1, with some supplicants (Eg: WZC), in WPATKIP  */
	/* mode, stations do not respond to GK-HS-MSG1 with key length 0, so the   */
	/* key length field will be set to 32/16 depending on the cipher type.	   */
	/* Also, 802.11 standard is contradicting on setting GNONCE in GK-HS-MSG1. */
	/* Thougn there is no specific use of GNONCE for a STA, we will set GNONCE */
	/* in GK-HS-MSG1 for both WPA and WPA2.  								   */
	/* With the above changes, initial HS and Group key update works in all */
	/* security modes with Odessey and WZC supplicants						   */

	/* Obtain GNonce from the global counter. */
	gnonce = g_rsna_auth.global.GNonce;
	eapol_length = prepare_eapol_key_frame(key_info, gtk_len,
			key_rsc, gnonce, key_iv, key_data_len, key_data,
            (UWORD8 *)replay_cnt, eapol_buffer, rsna_ptr->mode_802_11i);

    incr_rply_cnt(rsna_ptr->ReplayCount);

    /* Set the EAPOL Key frame size */
    eapol_size = KEY_DATA_OFFSET + key_data_len;

    /* Obtain the KCK from the PTK. This will be used in the MIC computation.*/
    //kck = get_kck(rsna_ptr->PTK);

    /* Add SNAP Header and the EAPOL Header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_KEY, eapol_length,
                   rsna_ptr->mode_802_11i);
    eapol_length += EAPOL_1X_HDR_LEN;

    /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC  */
    /* field zeroed. The MIC is computed using the KCK derived from PTK.*/
    add_eapol_mic(eapol_buffer, eapol_size, rsna_ptr->PTK,
            rsna_ptr->key_version);

    if(rsna_ptr->key_version == 1)
    {
        /* Key counter is used as the IV, therefore increment the Key */
        /* counter                                                    */
        incr_cnt(g_rsna_auth.global.GCounter, NONCE_SIZE);
    }

    /* Increment the Timeout Counter */
    rsna_ptr->GTimeoutCtr++;


    /* Reset the flags as required by the state machine */
    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {

    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        rsna_ptr->PInitAKeys = BFALSE;
        rsna_ptr->GInitAKeys = BFALSE;
    }


    /* Set State to AGK_REKEYNEGOTIATING */
    rsna_ptr->rsna_auth_grpkey_state  = AGK_REKEYNEGOTIATING;

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(rsna_ptr->addr, buffer, eapol_length, BTRUE))
    {
        /* EAPOL could not be sent, so assume that this sta is done with GTK update */
        /* so that whole process of GTK update dont get stuck for this STA alone      */
        rsna_auth_grpkey_rekeyestablished(rsna_ptr);
        
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);
                
        return;
    }

    /* Set timer since the response is expected for this packet */
    timer_started = start_11i_auth_timer(rsna_ptr,
            get_11i_resp_timeout((UWORD8)rsna_ptr->GTimeoutCtr,
            rsna_ptr->listen_interval));

    if(timer_started == BFALSE)
    {
        /* Timer could not be started, so assume that this sta is done with GTK update */
        /* so that whole process of GTK update dont get stuck for this STA alone      */
         rsna_auth_grpkey_rekeyestablished(rsna_ptr);
         
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);
                 
         return;
    }
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_grpkey_keyerror                             */
/*                                                                           */
/*  Description      : This function implements the KEYERROR state of the    */
/*                     per-sta group key handshake state machine.            */
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

void rsna_auth_grpkey_keyerror(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr->GUpdateStationKeys == BTRUE)
    {
        g_rsna_auth.global.GKeyDoneStations--;
        rsna_ptr->GUpdateStationKeys = BFALSE;
    }

    rsna_ptr->Disconnect         = BTRUE;

    rsna_ptr->deauth_reason      = GRP_HS_TIMEOUT;

    /* Set State to AGK_KEYERROR */
    rsna_ptr->rsna_auth_grpkey_state  = AGK_KEYERROR;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_grpkey_rekeyestablished                     */
/*                                                                           */
/*  Description      : This function implements the REKEYESTABLISHED state   */
/*                     of the per-sta group key handshake state machine.     */
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

void rsna_auth_grpkey_rekeyestablished(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr->GUpdateStationKeys == BTRUE)
    {
        g_rsna_auth.global.GKeyDoneStations--;
        rsna_ptr->GUpdateStationKeys = BFALSE;
    }

    rsna_ptr->keycount           = 0;

    /* Set State to AGK_REKEYESTABLISHED */
    rsna_ptr->rsna_auth_grpkey_state  = AGK_REKEYESTABLISHED;

#ifdef BSS_ACCESS_POINT_MODE
    /* Set keyDone to TRUE */
    ((auth_t *)(rsna_ptr->sta_1x))->global.keyDone = BTRUE;
#endif /* BSS_ACCESS_POINT_MODE */

}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
