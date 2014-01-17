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
/*  File Name         : ieee_11i_ap.h                                        */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AP mode for 802.11i                                  */
/*                                                                           */
/*  List of Functions : filter_wlan_rx_11i_auth_ap                           */
/*                      filter_host_rx_11i_auth_ap                           */
/*                      reset_11i_entry_auth_ap                              */
/*                      delete_11i_entry_auth_ap                             */
/*                      init_11i_handle_auth_ap                              */
/*                      start_11i_fsm_auth_ap                                */
/*                      check_11i_ucast_wlan_2_wlan_ap                       */
/*                      check_11i_bcast_wlan_2_wlan_ap                       */
/*                      is_11i_handshake_pkt_ap                              */
/*                      ap_check_11i_tx_sta_state                            */
/*                      get_supp_key_count                                   */
/*                      is_wep_allowed_11i                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifdef MAC_802_11I

#ifndef IEEE_11I_AP_H
#define IEEE_11I_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "eapol_key_auth.h"
#include "index_util.h"
#include "mib_11i.h"
#include "management_11i.h"
#include "management_ap.h"
#include "ap_onex_prot_if.h"
#include "rkmal_auth.h"
#include "rsna_auth_km.h"
#include "tkip_auth.h"
#include "utils.h"
#include "wep.h"

/*****************************************************************************/
/* Extern Functions                                                          */
/*****************************************************************************/

extern UWORD16 check_rsn_capabilities_ap(asoc_entry_t *ae, UWORD8 *msa, UWORD16 rx_len);
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
extern void handle_11i_tx_comp_ap(UWORD8 *dscr, asoc_entry_t *ae);
#else
extern void handle_11i_tx_comp_ap(UWORD8 *dscr, asoc_entry_t *ae, UWORD8 *msa);
#endif
extern UWORD8 *get_pmkid_asoc(UWORD8 *msa, UWORD16 rx_len, UWORD8 *pmkid);
extern void update_11i_fail_stats_ap(CIPHER_T ct, UWORD8 status, UWORD8 *sa);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
INLINE BOOL_T is_11i_hs_complete(asoc_entry_t *ae)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        if(ae == NULL)
        {
           return BFALSE;
        }

        if(ae->persta == NULL)
        {
            return BFALSE;
        }

        if(ae->persta->sta_1x == NULL)
        {
            return BFALSE;
        }

        if((((auth_t *)(ae->persta->sta_1x))->global.keyDone != BTRUE) ||
           (((auth_t *)(ae->persta->sta_1x))->global.portValid != BTRUE))
        {
            return BFALSE;
        }
    }

    return BTRUE;
}

/* Function filters the Frame for 802.11i encryption  in Access point mode */
/* using the MIB Exclude paramter. Returns TRUE if the frame is to be      */
/* discarded; False otherwise                                              */
INLINE BOOL_T filter_wlan_rx_11i_auth_ap(wlan_rx_t *wlan_rx)
{
    CIPHER_T ct         = (CIPHER_T)wlan_rx->ct;
    asoc_entry_t *ae    = (asoc_entry_t *) wlan_rx->sa_entry;
    UWORD8 type_subtype = wlan_rx->sub_type;

    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return BFALSE;
    }


    /* Taking care of TSN */
    if(ae->persta == NULL)
    {
        if(ct != g_wep_type)
        {
            return BTRUE;
        }
        return BFALSE;
    }

    if(ae->persta->sta_1x == NULL)
    {
        return BFALSE;
    }

    /* The check whether to allow the frame travel further if the  */
    /* port is disabled is done later                              */

    /* Here the check is done for the encryption enforcement after */
    /* the port is enabled                                         */
    /* 802.11i requires all the frames after the port enable to be */
    /* STRICTLY encripted. If the frame is not encrypted the frame */
    /* is dropped                                                  */
    if((((auth_t *)(ae->persta->sta_1x))->global.portEnabled == BTRUE)
      && (((auth_t *)(ae->persta->sta_1x))->global.portValid == BTRUE))
    {
        /* Check only if Data is present in the frame */
        if(((type_subtype & 0xF0) >> 4) < 4)
        {
            /* The data frames must be discarded during cntr measure */
            if(g_rsna_auth.global.cntr_msr_in_progress == BTRUE)
            {
                return BTRUE;
            }

            if(is_group(wlan_rx->addr1) == BTRUE)
            {
                UWORD8 grp_policy = 0;
                grp_policy = mget_RSNAConfigGroupCipher();
                if(ct != cipsuite_to_ctype(grp_policy))
                {
                    /* Discard the frame */
                    return BTRUE;
                }
            }
            else
            {
                UWORD8 pcip = g_wep_type;

                if(ae->persta != NULL)
                {
                    pcip = cipsuite_to_ctype(ae->persta->pcip_policy);
                }

                /* PCIP NO Encryption implies WEP     */
                /* Else TKIP/CCMP                     */
                /* Check the policies correcpondingly */
                if(pcip == NO_ENCRYP)
                {
                    /* For the wep, if the flag is set, frames without */
                    /* security should be discarded                    */
                    if(mget_ExcludeUnencrypted() == TV_TRUE)
                    {
                        if(ct != g_wep_type)
                        {
                            /* Discard the frame */
                            mincr_WEPExcludedCount();
                            return BTRUE;
                        }
                    }
                    else if((ct != g_wep_type) && (ct != NO_ENCRYP))
                    {
                        return BTRUE;
                    }
                }
                else
                {
                    /* Ignore the packet if the policies donot match */
                    if(ct != pcip)
                    {
                        /* Discard the frame */
                        return BTRUE;
                    }
                }
            }
        }
    }
    return BFALSE;
}


/* Set cipher type in the host Rx Frame filtering process for Access point   */
/* mode for 802.11i encryption. Returns false if the frame must be processed */
/* and true if the frame be dropped without any processing                   */
INLINE BOOL_T filter_host_rx_11i_auth_ap(CIPHER_T *ct, UWORD8 **info,
                                         UWORD8 key_type, asoc_entry_t *ae,
                                         BOOL_T ignore_port)
{
    /* If RSNA is disabled, donot proceed further */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return BFALSE;
    }

    /* The packets must be discarded when coutner measure is in progress */
    if(g_rsna_auth.global.cntr_msr_in_progress == BTRUE)
    {
        return BTRUE;
    }

    /* Filtering for the Unicast Type frame */
    if(key_type == UCAST_KEY_TYPE)
    {
        /* If RSNA is enabled, cipher type and key index are selected for    */
        /* the corresponding STA                                             */
        if(ignore_port == BFALSE)
        {
            /* Drop the data frame if 11i handshake is not yet complete */
            if(BFALSE == is_11i_hs_complete(ae))
            {
                return BTRUE;
            }

            /* Get the cipher type for the sta */
            *ct = cipsuite_to_ctype(ae->persta->pcip_policy);

            /* If Cipher type is TKIP, mic key needs to be specified */
            if(*ct == TKIP)
            {
                /* Get Mic key required for transmission with Key type 1 */
                *info = get_auth_mic_key(ae->persta->PTK, 1);
            }
        }
    }
    else if(key_type == BCAST_KEY_TYPE)
    {
        /* If RSNA is enabled, cipher type and key index are selected for    */
        /* the corresponding broadcast type in use                           */
        if(g_rsna_auth.global.rsna_auth_glk_state != AGLK_GTK_INIT)
        {
            UWORD8 grp_policy = mget_RSNAConfigGroupCipher();

            /* Read the cipher Type value for the cipher suite */
            *ct = cipsuite_to_ctype(grp_policy);

            /* If Cipher type is TKIP, mic key needs to be specified */
            if(*ct == TKIP)
            {
                if( g_rsna_auth.global.GN > GTK_MAX_INDEX )
                {
                    return BFALSE;
                }
                /* Get Mic key required for transmission with Key type 0 */
                *info = get_auth_mic_key
                        (g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)], 0);
            }
        }
    }
    return BFALSE;
}

/* Reset the 802.11i entry by stopping its RSNA FSM */
INLINE void reset_11i_entry_auth_ap(asoc_entry_t *ae)
{
    /* If RSNA is implemented and enabled clear the RSNA Handle  */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If the RSNA has been implemented, free the RSNA KM Suite */
        if(ae->persta != NULL)
        {
            stop_auth_rsna_fsm(ae->persta);
        }
    }
}


/* Reset the 802.11i entry by halting and freeing the RSNA FSM */
INLINE void delete_11i_entry_auth_ap(asoc_entry_t *ae)
{
    /* If RSNA is implemented and enabled clear the RSNA Handle  */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If the RSNA has been implemented, free the RSNA KM Suite */
        if(ae->persta != NULL)
        {
            free_auth_rsna_km(&(ae->persta));
        }
    }
}

/* Initialize the RSNA Entry to NULL. This is done at the reception of the   */
/* authentication frame sequence # 1                                         */
INLINE void init_11i_handle_auth_ap(asoc_entry_t *ae)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        ae->persta = NULL;
    }
}

/* Initialize the RSNA Entry with default parameters. This is done after     */
/* the association request is successfully accepted                          */
INLINE UWORD16 init_11i_entry_ap(asoc_entry_t *ae, UWORD8 *sa, UWORD8 *msa,
                                 UWORD16 rx_len)
{
    /* If RSNA is enabled, install the 11i state machine */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* The association must not be allowed */
        if(g_rsna_auth.global.cntr_msr_in_progress == BTRUE)
        {
            return UNSPEC_FAIL;
        }

        /* If the installation is successful */
        /* proceed, else set failure status  */
        if(initialize_auth_rsna_hdl(&(ae->persta), ae->sta_index,
                ae->listen_interval, mget_RSNAPairwiseCipherRequested(),
                mget_RSNAAuthenticationSuiteRequested(), sa, msa, rx_len,
                &(ae->auth_persta_buff)) != BTRUE)
        {
            return UNSPEC_FAIL;
        }
    }

    return SUCCESSFUL_STATUSCODE;
}

/* If RSNA is enabled, start the 802.11i state machine */
INLINE void start_11i_fsm_auth_ap(asoc_entry_t *ae)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        if(ae == NULL)
        {
            return;
        }

        if(ae->persta == NULL)
        {
            return;
        }
        if((g_psk_available == BTRUE) &&
           (ae->persta->auth_policy == 0x02))
        {
            /* If PSK is enabled, start the RSNA state machine */
            start_auth_rsna_fsm(ae->persta);
        }
        else
        {

            if(ae->persta->sta_1x == NULL)
            {
                return;
            }

            /* If PMLSA is enabled, start the RSNA state machine */
            /* else enable 1x authentication FSM                 */
            if((((auth_t *)(ae->persta->sta_1x))->global.keyRun == BTRUE) &&
               (((auth_t *)(ae->persta->sta_1x))->global.keyAvailable == BTRUE))
            {
                start_auth_rsna_fsm(ae->persta);
            }
            else
            {
                start_sec_auth_fsm_ap((auth_t *)(ae->persta->sta_1x));
            }
        }
    }
}

/* Before forwarding the packet across the WLAN-WLAN interface security      */
/* checks needs to performed on the states of the transmitting and receiving */
/* stations                                                                  */
INLINE BOOL_T check_11i_ucast_wlan_2_wlan_ap(asoc_entry_t *da_ae,
            asoc_entry_t *sa_ae, CIPHER_T *ct, UWORD8 *data_trailer)
{
    *data_trailer = 0;

    /* Check if RSNA is enabled */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        *ct = (CIPHER_T)g_wep_type;

        /* If RSNA is enabled check if keyDone is true        */
        if((sa_ae->persta == NULL) ||
           (BTRUE == is_11i_hs_complete(sa_ae)))
        {
            if(da_ae->persta == NULL)
            {
                return BTRUE;
            }
            /* If portValid is set for SA and DA, the source and    */
            /* destination each have ports configured for Tx and Rx */
            if(BTRUE == is_11i_hs_complete(sa_ae))
            {
                /* Read the cipher type value for the cipher suite */
                *ct = cipsuite_to_ctype(da_ae->persta->pcip_policy);

                if(*ct == TKIP)
                {
                    /* Adding the MIC Key length to the packet length */
                    *data_trailer = TKIP_MIC_LEN;
                 }

                 /* Frame requires WLAN to WLAN transmission */
                 return BTRUE;
            }
        }
    }
    else
    {
        return BTRUE;
    }
    return BFALSE;
}


/* Before forwarding the packet across the WLAN-WLAN interface */
/* security checks needs to performed on the states of the     */
/* transmitting and receiving stations                         */
INLINE UWORD8 check_11i_bcast_wlan_2_wlan_ap(CIPHER_T *ct)
{
    UWORD8 trailer = 0;

    /* Check if RSNA is enabled */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* Read the cipher type value for the cipher suite */
        *ct = cipsuite_to_ctype(mget_RSNAConfigGroupCipher());

        /* Compute MIC is the group encryption is TKIP */
        if((mget_RSNAEnabled() == TV_TRUE) && (*ct == TKIP))
        {
            /* Adding the MIC Key length to the packet length */
            trailer = TKIP_MIC_LEN;
        }
    }

    return trailer;
}

/* The received packet is checked if it is a 802.11i handshake */
/* packet. If it is, it is processed by 11i state machine.     */
/* Returns true if it is a 11i packet                          */
INLINE BOOL_T is_11i_handshake_pkt_ap(asoc_entry_t *ae, UWORD8 *buffer,
                                      UWORD8 offset, UWORD16 rx_len,
                                      CIPHER_T ct)

{
    UWORD8 *snap_hdr_ptr = NULL;
    UWORD8 *rx_data      = NULL;

    /* Check if RSNA is enabled & that the SNAP implies presence of */
    /* 802.1x/802.11i header                                        */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        snap_hdr_ptr = buffer + offset;

        if(BTRUE == check_11i_frame(snap_hdr_ptr))
        {
            rx_data = snap_hdr_ptr + SNAP_HDR_LEN;

            /* Check if the frame is a Remote MIC Failure Frame */
            /* The frame is a remote MIC Failure report frame if the */
            /* frame type is EAPOL Key with cipher type TKIP and     */
            /* MIC Bit, Error Bit, Request Bit and Secure Bit set    */
            if((get_eapol_type(rx_data) == EAPOL_KEY) && (ct == TKIP)
             && (is_mic_fail_report(rx_data) == BTRUE))
            {
                /* Increment the MIB MIC Failure Count */
                mincr_RSNAStatsTKIPRemoteMICFailures(
                                            (UWORD8)ae->persta->key_index);

                /* MIC Failure: Counter Measures invoked */
                start_auth_cntr_msr_timer();
            }
            else
            {
                if(ae == NULL)
                {
                    return BFALSE;
                }

                if(ae->persta == NULL)
                {
                    return BFALSE;
                }

                if(ae->persta->sta_1x == NULL)
                {
                    return BFALSE;
                }
                if(((auth_t *)(ae->persta->sta_1x))->global.keyRun == BFALSE)
                {
                    /* Key run is False and the packet is meant for 1X */
                    /* state machine                                   */
                    /* Key run is False and the packet is meant for 1X */
                    sec_auth_eapol_ap(rx_data,
                                      (UWORD16)(rx_len - SNAP_HDR_LEN),
                                      (auth_t *)(ae->persta->sta_1x));
                }
                else
                {
                    /* Key run is True and the packet is meant for 11i FSM */
                    /* Pass on the message to the 11i authenticator FSM    */
                    auth_rsn_eapol(rx_data,
                                   (UWORD16)(rx_len - SNAP_HDR_LEN),
                                   ae->persta);
                }
            }
            return BTRUE;
        }
    }
    return BFALSE;
}

/* Checks if the transmitting station has invalid 802.11i states for the */
/* Broadcast frame                                                       */
INLINE BOOL_T ap_check_11i_tx_sta_state(asoc_entry_t *ae)
{
    /* Check if RSNA is enabled */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If RSNA is enabled check if keyDone is true              */
        /* If portValid is set for SA, and the packet was decrypted */
        /* successfully implies STA has configured GTK              */
        /* If either of these conditions is false, packet must be   */
        /* discarded                                                */
        if(BFALSE == is_11i_hs_complete(ae))
        {
            return BFALSE;
        }
    }
    return BTRUE;
}

/* Dummy function, meant only for IBSS mode */
INLINE UWORD8 get_supp_key_count(UWORD8 sta_index)
{
    return 0;
}

/* Check if WEP/shared key is ok */
INLINE BOOL_T is_wep_allowed_11i(void)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        UWORD8 grp_policy = mget_RSNAConfigGroupCipher();
        if((grp_policy == 0x01) || (grp_policy == 0x05))
        {
            return BTRUE;
        }
        return BFALSE;
    }
    else
    {
        return BTRUE;
    }
}

/* For all the incoming packets, the security check is performed so as to    */
/* determine if the incoming packets have been tampered                      */
INLINE BOOL_T check_11i_rx_mpdu_ap(asoc_entry_t *ae, CIPHER_T ct, UWORD8 *msa,
                                   UWORD8 *old_pn_val)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* Perform replay detection. Reordering of MSDUs is not supported    */
        /* since only one counter per STA per SA is supported                */
        if((ct == TKIP) || (ct == CCMP))
        {
            UWORD8 rx_pn_val[6] = {0};
            UWORD8 mac_hdr_len  = get_mac_hdr_len(msa);

            if((ae == NULL) || (ae->persta == NULL))
            {
                return BFALSE;
            }

            /* If QOS is Enabled, then the H/w will be offsetting two extra  */
            /* bytes after the header to make it word aligned                */
            mac_hdr_len = (mac_hdr_len == MAC_HDR_LEN)? MAC_HDR_LEN :
                                                        (MAC_HDR_LEN + 4);


            get_pn_val(msa + mac_hdr_len, (UWORD8)ct, rx_pn_val);

            /* Check PN Value wrap around condition for the recevied value */
            if((rx_pn_val[5] == 0xFF) &&
               (rx_pn_val[4] == 0xFF) &&
               (rx_pn_val[3] == 0xFF) &&
               (rx_pn_val[2] == 0xFF) &&
               (rx_pn_val[1] == 0xFF) &&
               (rx_pn_val[0] == 0xF0))
            {
                rsna_send_deauth((UWORD8 *)ae->persta, 0);
            }

            if(cmp_pn_val(old_pn_val, rx_pn_val) != BTRUE)
            {
                /* The PN Sequence as received is out of order. Reply error  */
                /* is logged                                                 */
                if(ct == CCMP)
                {
                    mincr_RSNAStatsCCMPReplays((UWORD8)ae->persta->key_index);
                }
                else if(ct == TKIP)
                {
                    mincr_RSNAStatsTKIPReplays((UWORD8)ae->persta->key_index);
                }

#ifdef DEBUG_MODE
                g_mac_stats.pewrxrply++;
#endif /* DEBUG_MODE */
                return BFALSE;
            }
        }
    }

    return BTRUE;
}

/* This function updates the security statistics on a successful reception */
INLINE void update_11i_success_stats_ap(asoc_entry_t *ae, CIPHER_T ct,
                                        UWORD8 status)
{
    if(ae != NULL)
    {
        if((status == RX_SUCCESS) && ((ct == CCMP) || (ct == TKIP)))
            g_decr_fail_cnt[ae->sta_index] = 0;
    }
}

/* This function checks whether a counter measure operation is currently */
/* in progress.                                                          */
INLINE BOOL_T is_cntr_msr_in_progress_ap(void)
{
    return g_rsna_auth.global.cntr_msr_in_progress;
}

#endif /* IEEE_11I_AP_H */

#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
