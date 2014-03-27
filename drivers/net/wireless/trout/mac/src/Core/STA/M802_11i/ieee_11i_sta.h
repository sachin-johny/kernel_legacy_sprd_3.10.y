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
/*  File Name         : ieee_11i_sta.h                                       */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      STA mode for 802.11i                                 */
/*                                                                           */
/*  List of Functions : is_11i_handshake_pkt_sta                             */
/*                      delete_11i_entry_sta                                 */
/*                      reset_11i_entry_sta                                  */
/*                      init_11i_handle_sta                                  */
/*                      init_11i_entry_sta                                   */
/*                      start_11i_fsm_sta                                    */
/*                      filter_wlan_rx_11i_sta                               */
/*                      filter_host_rx_11i_supp_sta                          */
/*                      sta_check_11i_tx_sta_state                           */
/*                      check_11i_rx_mpdu_sta                                */
/*                      get_auth_key_count                                   */
/*                      get_supp_key_count                                   */
/*                      auth_finalize_fsm                                    */
/*                      get_pmkid_asoc                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

#ifndef IEEE_11I_STA_H
#define IEEE_11I_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "eapol_key.h"
#include "mib_11i.h"
#include "index_util.h"
#include "management_11i.h"
#include "management_sta.h"
#include "prot_if.h"
#include "rsna_supp_km.h"
#include "rkmal_sta.h"
#include "utils.h"
#include "wep.h"
#include "sta_onex_prot_if.h"
#include "receive.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define PMKID_CACHE_SIZE  4

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8 addr[MAC_ADDRESS_LEN];
    UWORD8 pmkid[PMKID_LEN];
} pmkid_info_t;

typedef struct
{
    UWORD8       num_elems;
    pmkid_info_t elem[PMKID_CACHE_SIZE];
} pmkid_cache_t;

/*****************************************************************************/
/* Extern Functions                                                          */
/*****************************************************************************/

extern void misc_11i_event_sta(mac_struct_t *mac, UWORD8 *msg);
// 20120709 caisf mod, merged ittiam mac v1.2 code
//extern void handle_11i_tx_comp_sta(UWORD8 *dscr, sta_entry_t *se);
extern void handle_11i_tx_comp_sta(UWORD8 *dscr, sta_entry_t *se, UWORD8* msa);
extern void update_11i_fail_stats_sta(CIPHER_T ct, UWORD8 status, UWORD8 *sa,
                                      UWORD8 *da);
extern UWORD8 search_pmkid_cache(UWORD8 *bssid, UWORD8 *pmkid);
extern void flush_pmkid_cache(void);
extern UWORD8 *get_pmkid_cache_sta(void);
extern void set_pmkid_cache_sta(UWORD8 *val);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
INLINE BOOL_T is_11i_hs_complete(sta_entry_t *se)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
    	//chenq add for fix bug 2013-02-21
    	#if 1
		#ifdef EXT_SUPP_11i
		//xuan yang, 2013.6.20, 返回wpa密钥协商是否完成
			return g_wpa_wpa2_hs_complete;
			//return BTRUE;
		#endif
		#endif
	
        if(se == NULL)
        {
           return BFALSE;
        }

        if(se->persta == NULL)
        {
            return BFALSE;
        }

        if(se->persta->sta_1x == NULL)
        {
            return BFALSE;
        }

        if((se->persta->sta_1x->global.keyDone != BTRUE) ||
           (se->persta->sta_1x->global.portValid != BTRUE))
        {
            return BFALSE;
        }
    }

    return BTRUE;
}
#ifdef SUPP_11I
/* The received packet is checked if it is a 802.11i handshake */
/* packet. If it is, it is processed by 11i state machine.     */
/* Returns true if it is a 11i packet                          */
INLINE BOOL_T is_11i_handshake_pkt_sta(sta_entry_t *se, UWORD8 *buffer,
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
            if(se == NULL)
                return BTRUE;

            rx_data = snap_hdr_ptr + SNAP_HDR_LEN;

            if(se->persta->sta_1x->global.keyRun == BFALSE)
            {
                /* KeyRun is False. The packet is meant for 1x state machine */
                /* state machine. Run the 1x state machine                   */
                sec_supp_eapol_sta(rx_data, rx_len - SNAP_HDR_LEN,
                                   se->persta->sta_1x);
            }
            else
            {
                if(snap_hdr_ptr[14] & 0x80)
                {
                    /* Key run is True and the packet is meant for 11i FSM */
                    /* Pass on the message to the 11i authenticator FSM    */
                    supp_rsn_eapol(rx_data, rx_len - SNAP_HDR_LEN, se->persta);
                }
#ifdef IBSS_11I
                else
                {
                    auth_rsn_eapol(rx_data, rx_len - SNAP_HDR_LEN, se->auth_persta);
                }
#endif /* IBSS_11I */
            }
            return BTRUE;
        }
    }
    return BFALSE;
}
#endif /* SUPP_11I */
/* Reset the 802.11i entry by halting and freeing the RSNA FSM */
INLINE void delete_11i_entry_sta(sta_entry_t *se)
{
    /* If RSNA is implemented and enabled clear the RSNA Handle  */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If the RSNA has been implemented, free the RSNA KM Suite */
        free_supp_rsna_km(&(se->persta));
#ifdef IBSS_11I
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            free_auth_rsna_km(&(se->auth_persta));
        }
#endif /* IBSS_11I */
    }
}


/* Reset the 802.11i entry by stopping its RSNA FSM */
INLINE void reset_11i_entry_sta(sta_entry_t *se)
{
    /* If RSNA is implemented and enabled clear the RSNA Handle  */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If the RSNA has been implemented, free the RSNA KM Suite */
#ifdef SUPP_11I
		if(BTRUE == g_int_supp_enable)
        	stop_supp_rsna_fsm(se->persta);
#endif /* SUPP_11I */

#ifdef IBSS_11I
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            stop_auth_rsna_fsm(se->auth_persta);
        }
#endif /* IBSS_11I */
    }
}

/* Initialize the RSNA Entry to NULL. This is done at the transmission of */
/* authentication frame sequence # 1                                      */
INLINE void init_11i_handle_sta(sta_entry_t *se)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        se->persta = NULL;
#ifdef IBSS_11I
        se->auth_persta = NULL;
#endif /* IBSS_11I */
    }
}


/* Initialize the RSNA Entry with default parameters. This is done after */
/* the association request is successfully accepted                      */
INLINE BOOL_T init_11i_entry_sta(sta_entry_t *se, UWORD8 *sa)
{
    BOOL_T ret_val = BTRUE;

    /* If RSNA is enabled, install the 11i state machine */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* The association must not be allowed if cntr msr in progress */
        if(g_rsna_supp.global.cntr_msr_in_progress == BTRUE)
        {
            return BFALSE;
        }

        /* If the installation is successful */
        /* proceed, else set failure status  */
        ret_val = initialize_supp_rsna_hdl(&(se->persta),
                mget_RSNAPairwiseCipherRequested(),
                mget_RSNAGroupCipherRequested(),
                mget_RSNAAuthenticationSuiteRequested(), se->sta_index, sa,
                &(se->supp_persta_buff));

        if(ret_val != BTRUE)
        {
            return ret_val;
        }

#ifdef IBSS_11I
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            /* The association must not be allowed if cntr msr in progress */
            if(g_rsna_auth.global.cntr_msr_in_progress == BTRUE)
            {
                return BFALSE;
            }

            ret_val = initialize_auth_rsna_hdl(&(se->auth_persta),
                    se->sta_index, mget_listen_interval(),
                    mget_RSNAPairwiseCipherRequested(),
                    mget_RSNAAuthenticationSuiteRequested(), sa, 0, 0,
                    &(se->auth_persta_buff));
        }
#endif /* IBSS_11I */
    }

    return ret_val;
}


/* If RSNA is enabled, start the 802.11i state machine */
INLINE void start_11i_fsm_sta(sta_entry_t *se)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        if((g_psk_available == BTRUE) &&
                (se->persta->auth_policy == 0x02))
        {
            /* If PSK is enabled, start the RSNA state machine */
            start_supp_rsna_fsm(se->persta);
        }
        else
        {
            /* If PMKSA is enabled, start the RSNA state machine */
            /* else enable 1x authentication FSM                 */
            if((se->persta->sta_1x->global.keyRun == BTRUE) &&
               (se->persta->sta_1x->global.keyAvailable == BTRUE))
            {
                start_supp_rsna_fsm(se->persta);
            }
            else
            {
                start_sec_supp_fsm_sta(se->persta->sta_1x);
            }
        }
    }
}

/* Function filters the Frame for 802.11i encryption in Station mode  */
/* using the MIB Exclude paramter. Returns TRUE if the frame is to be */
/* discarded; False otherwise                                         */
INLINE BOOL_T filter_wlan_rx_11i_sta(wlan_rx_t *wlan_rx)
{
    CIPHER_T ct         = (CIPHER_T) wlan_rx->ct;
    sta_entry_t *se     = (sta_entry_t *) wlan_rx->sa_entry;
    UWORD8 type_subtype = wlan_rx->sub_type;

    /* If RSNA is disabled, donot proceed further */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return BFALSE;
    }

    /*Drop the frame if RSNA ptr is not yet created for this station */
    if(se->persta == NULL)
    {
        return BTRUE;
    }

#ifdef IBSS_11I
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        /* The data frames must be discarded during cntr measure */
        if(g_rsna_auth.global.cntr_msr_in_progress == BTRUE)
        {
            return BTRUE;
        }
    }
#endif /* IBSS_11I */

    /* The data frames must be discarded during cntr measure */
    if(g_rsna_supp.global.cntr_msr_in_progress == BTRUE)
    {
        return BTRUE;
    }

    /* The check whether to allow the frame travel further if the  */
    /* port is disabled is done later                              */

    /* Here the check is done for the encryption enforcement after */
    /* the port is enabled                                         */
    /* 802.11i requires all the frames after the port enable to be */
    /* STRICTLY encripted. If the frame is not encrypted the frame */
    /* is dropped                                                  */
    if((se->persta->sta_1x->global.portEnabled == BTRUE) &&
       (se->persta->sta_1x->global.portValid == BTRUE))
    {
        /* Check only if the Data is present */
        if(((type_subtype & 0xF0) >> 4) < 4)
        {
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
                    pcip = cipsuite_to_ctype(se->persta->pcip_policy);

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


/* Set cipher type in the host Rx Frame filtering process for */
/* Access point mode for 802.11i encryption                   */
/* Returns false if the frame must be processed and true if   */
/* the frame be dropped without any processing                */
INLINE BOOL_T filter_host_rx_11i_supp_sta(UWORD16 eth_type, CIPHER_T *ct,
                                          UWORD8 **info, UWORD8 key_type,
                                          sta_entry_t *se, BOOL_T ignore_port)
{
    /* If RSNA is disabled, donot proceed further */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return BFALSE;
    }

    /* The packets must not be aired during counter measures */
    if(g_rsna_supp.global.cntr_msr_in_progress == BTRUE)
    {
        return BTRUE;
    }
    /* Filtering for the Unicast Type frame */
    if(key_type == UCAST_KEY_TYPE)
    {
        /* If RSNA is enabled, cipher type and key index are selected for */
        /* the corresponding STA                                          */
        if(ignore_port == BFALSE)
        {
            /*Drop the frame if 11i hs is not yet complete */
            if(BFALSE == is_11i_hs_complete(se))
            {
#ifdef SUPP_11I

#ifndef EXT_SUPP_11i
				if(BTRUE == g_int_supp_enable)
                	return BTRUE;
                else
#endif /* EXT_SUPP_11i */

#endif /* SUPP_11I */
				{
					/* Pass only 802.1x type packets for EAPOL handshake */
					if(eth_type == 0x888E)
					{
						return BFALSE;
					}
					else
					{
						return BTRUE;
					}
				}
            }

            /* Get the cipher type for the sta */
            *ct = cipsuite_to_ctype(se->persta->pcip_policy);

            /* If Cipher type is TKIP, mic key needs to be specified */
            if(*ct == TKIP)
            {
                /* Get Mic key required for transmission with Key type 1 */
                *info = get_supp_mic_key(se->persta->PTK, 1);
            }
        }
    }
    return BFALSE;
}

#ifdef IBSS_11I
/* Set cipher type in the host Rx Frame filtering process for Access point   */
/* mode for 802.11i encryption. Returns false if the frame must be processed */
/* and true if the frame be dropped without any processing                   */
INLINE BOOL_T filter_host_rx_11i_auth_sta(CIPHER_T *ct, UWORD8 **info,
                                    UWORD8 key_type, sta_entry_t *se,
                                    BOOL_T ignore_port)
{
    /* If RSNA is disabled, donot proceed further */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return BFALSE;
    }

        /* The packets must not be aired during counter measures */
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
            /* Taking care of TSN */
            if(se->auth_persta == NULL)
            {
                *ct = (CIPHER_T) g_wep_type;
                return BFALSE;
            }

            if((((auth_t *)(se->auth_persta->sta_1x))->global.keyDone != BTRUE)
               || (((auth_t *)(se->auth_persta->sta_1x))->global.portValid
                                                                     != BTRUE))
            {
                return BTRUE;
            }

            /* Get the cipher type for the sta */
            *ct = cipsuite_to_ctype(se->auth_persta->pcip_policy);

            /* If Cipher type is TKIP, mic key needs to be specified */
            if(*ct == TKIP)
            {
                /* Get Mic key required for transmission with Key type 1 */
                *info = get_auth_mic_key(se->auth_persta->PTK, 1);
            }
        }
    }
    else if(key_type == BCAST_KEY_TYPE)
    {
        /* If RSNA is enabled, cipher type and key index are selected for    */
        /* the corresponding broadcast type in use                           */
        if(g_rsna_auth.global.rsna_auth_glk_state == AGLK_SETKEYSDONE)
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
#endif /* IBSS_11I */

/* Checks if the transmitting station has invalid 802.11i states for the */
/* Broadcast frame                                                       */
INLINE BOOL_T sta_check_11i_tx_sta_state(sta_entry_t *se,
                                         msdu_desc_t *frame_desc,
                                         CIPHER_T ct)
{
	/* If 11i Handshake is not complete, Allow only 1x packets */
    if(BFALSE == is_11i_hs_complete(se))
	{
		/* Check if its a 1x handshake packet only if it has snap header*/
		if(BTRUE == is_snap_header_present(frame_desc))
	    {
			UWORD8 *snap_hdr_ptr = frame_desc->buffer_addr +
								   frame_desc->data_offset;

            /* Check if its a 1x handshake packet */
			if(BTRUE == check_11i_frame(snap_hdr_ptr))
			{
				return BTRUE;
			}
        }
		PRINTD("Dropping the non-1x pkt \n\r");
				return BFALSE;
	}
	/* If 11i Handshake is complete, Allow all packets */
	else
	{
		return BTRUE;
	}
}

#ifdef IBSS_11I
/* Gets the Authentication FSM Key count for the IBSS PSK */
INLINE UWORD8 get_auth_key_count(UWORD8 sta_index)
{
    if(sta_index)
    {
        return g_rsna_auth.rsna_ptr[sta_index - 1]->keycount;
    }
    else
    {
        return g_rsna_auth.rsna_ptr[0]->keycount;
    }
}
#endif /* IBSS_11I */

/* Gets the Supplicant FSM Key count for the IBSS PSK */
INLINE UWORD8 get_supp_key_count(UWORD8 sta_index)
{
    if(sta_index)
    {
        if(g_rsna_supp.rsna_ptr[sta_index - 1]->keycount == 1)
        {
            g_rsna_supp.rsna_ptr[sta_index - 1]->sta_1x->global.portValid = BTRUE;
            return 1;
        }
    }
    return 0;
}

#ifdef IBSS_11I
/* Start the Authentication FSM after the group key count is 2 for IBSS mode */
INLINE void auth_finalize_fsm(UWORD8 sta_index)
{
    UWORD8             key_id    = 0;
    rsna_auth_persta_t *rsna_ptr = NULL;
    CIPHER_T         cipher_type = NO_ENCRYP;

    if(sta_index)
    {
        rsna_ptr = g_rsna_auth.rsna_ptr[sta_index - 1];
    }
    else
    {
        return;
    }

    if(rsna_ptr == NULL)
    {
        if(g_rsna_supp.rsna_ptr[sta_index - 1] != NULL)
        {
            /* Delete the correspoding station entry */
            delete_entry(g_rsna_supp.rsna_ptr[sta_index - 1]->addr);
        }
        return;
    }

    ((auth_t *)(rsna_ptr->sta_1x))->global.portValid = BTRUE;

    /* Depending on the ciphers policies set the key ids and policies */
    cipher_type = cipsuite_to_ctype(rsna_ptr->pcip_policy);

    if(rsna_ptr->pcip_policy != 0x00)
    {
        /* Set keys for this STA */
        if(memcmp(mget_StationID(), rsna_ptr->addr, 6) > 0)
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
        }
    }
    /* Set the flags indicating completion of the state machine */
    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        rsna_ptr->GInitAKeys = BTRUE;
        rsna_ptr->PInitAKeys = BTRUE;
    }
    auth_rsn_fsm_run(rsna_ptr);
}
#endif /* IBSS_11I */

/* Dummy function for IBSS mode; return PMKID in association packet */
INLINE UWORD8 *get_pmkid_asoc(UWORD8 *msa, UWORD16 rx_len, UWORD8 *pmkid)
{
    return NULL;
}

/* This function is used to add PTK for STA */
INLINE void add_11i_ptk_sta(UWORD8 *val)
{
    UWORD8 key_length    = 0;
    UWORD8 cipher_type   = 0;
    UWORD8 sta_addr[6]   = {0};
    UWORD8* key          = NULL;
    sta_entry_t *se      = NULL;
    rsna_supp_persta_t *rsna_ptr = NULL;


    /*----------------------------------------*/
    /*    STA Addr  | KeyLength |   Key       */
    /*----------------------------------------*/
    /*       6      |     1     |  KeyLength  */
    /*----------------------------------------*/

    /*---------------------------------------------------------*/
    /*                      key                                */
    /*---------------------------------------------------------*/
    /* Temporal Key    | Rx Micheal Key    |   Tx Micheal Key  */
    /*---------------------------------------------------------*/
    /*    16 bytes     |      8 bytes      |       8 bytes     */
    /*---------------------------------------------------------*/

    if(val == NULL)
    {
        return;
    }

    /* Check for BSSID */
    memcpy(sta_addr, &val[0], 6);
    if(memcmp(mget_bssid(), sta_addr, 6) != 0)
    {
        return;
    }

    /* Check if Station has an association entry */
    se = find_entry(sta_addr);
    if(se == NULL)
    {
        return;
    }

    /* Get the RSNA pointer */
    rsna_ptr = se->persta;

    if(rsna_ptr == NULL)
    {
        return;
    }

    /* Get the key length */
    key_length = *(UWORD8 *)(val + 6);

    /* PTK or GTK should atleast be 16 bytes */
    if(key_length < 16)
    {
        return;
    }

    /* Get the Key */
    key = val + 7;

    if(rsna_ptr->sta_1x != NULL)
    {
        rsna_ptr->sta_1x->global.keyDone   = BTRUE;
        rsna_ptr->sta_1x->global.portValid = BTRUE;
    }

    /* Copy 16 byte PTK */
    memcpy(rsna_ptr->PTK + 32, key, 16);

    /* Copy Rx Micheal Key and Tx Micheal Key */
    if(key_length > 16)
    {
        memcpy(rsna_ptr->PTK + 32 + 16, key + 16, key_length-16);
    }

    cipher_type = cipsuite_to_ctype(rsna_ptr->pcip_policy);

    machw_ce_add_key(0,
        PTK_NUM,
        se->sta_index,
        0,
        cipher_type,
        (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH + KEK_LENGTH),
        sta_addr,
        SUPP_KEY,
        (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH + KEK_LENGTH + TEMPORAL_KEY_LENGTH));


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

/* This function is used to add rx GTK */
INLINE void add_11i_rx_gtk_sta(UWORD8 *val)
{
    UWORD8 mask          = 0;
    UWORD8 count         = 0;
    UWORD8 key_id        = 0;
    UWORD8 key_length    = 0;
    UWORD8 cipher_type   = 0;
    UWORD8 sta_addr[6]   = {0};
    UWORD8 *key          = NULL;
    UWORD8 *keyRSC       = NULL;
    UWORD8 *mic_key      = NULL;
    sta_entry_t *se      = NULL;
    rsna_supp_persta_t *rsna_ptr  =NULL;

    /*---------------------------------------------------------*/
    /*    STA Addr  | KeyRSC | KeyID | KeyLength |   Key       */
    /*---------------------------------------------------------*/
    /*       6      |   8    |   1   |     1     |  KeyLength  */
    /*---------------------------------------------------------*/

    /*-------------------------------------*/
    /*                      key            */
    /*-------------------------------------*/
    /* Temporal Key    | Rx Micheal Key    */
    /*-------------------------------------*/
    /*    16 bytes     |      8 bytes      */
    /*-------------------------------------*/

    if(val == NULL)
    {
        return;
    }

    /* Check for BSSID */
    memcpy(sta_addr, &val[0], 6);
    if(memcmp(mget_bssid(), sta_addr, 6) != 0)
    {
        return;
    }

    /* Check if Station has an association entry */
    se = find_entry(sta_addr);
    if(se == NULL)
    {
        return;
    }

    /* Get the RSNA pointer */
    rsna_ptr = se->persta;

    if(rsna_ptr == NULL)
    {
        return;
    }

    /* Get the pointer to the initial PN value */
    keyRSC = val + 6;

    /* Get the Key Index */
    key_id = *(val + 14);

    if(key_id > GTK_MAX_INDEX)
    {
        return;
    }

    /* Get the key length */
    key_length = *(val + 15);

    /* PTK or GTK should atleast be 16 bytes */
    if(key_length < 16)
    {
        return;
    }

    /* Get the Key */
    key = val + 16;

    /* Copy the keys */
    for(count = 0; count < 16; count++)
    {
        /* Copy Rx GTK */
        rsna_ptr->GTK[get_GTK_array_index(key_id)][count] = key[count];

        /* Copy  Rx Micheal Key */
        if((key_length > 16) && (count < 8))
        {
            rsna_ptr->GTK[get_GTK_array_index(key_id)][count + 16] = key[count + 16];
        }
    }

    /* Copy initial PN value */
    for(count = 0; count < 6; count++)
    {
        rsna_ptr->bcst_rx_pn_val[count] = keyRSC [count];
    }

    /* WEP40 or WEP 104 */
    if(rsna_ptr->grp_policy == 4)
    {
        cipher_type = CCMP;
    }
    else if(rsna_ptr->grp_policy == 2)
    {
        cipher_type = TKIP;
        mic_key = &rsna_ptr->GTK[get_GTK_array_index(key_id)][TEMPORAL_KEY_LENGTH];
    }
    else if(rsna_ptr->grp_policy == 1)
    {
        cipher_type = WEP40;
        mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
    }
    else if(rsna_ptr->grp_policy == 5)
    {
        cipher_type = WEP104;
        mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
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
    machw_ce_add_key(key_id, (KEY_NUMBER_T) (rsna_ptr->last_key_num), se->sta_index,
                     mask, cipher_type, key, sta_addr, SUPP_KEY, mic_key);

    if(rsna_ptr->last_key_num == RX_GTK1_NUM)
    {
        /* Set the broadcast PN value for this STA index */
        machw_ce_init_rx_bcmc_pn_val(0, 1, rsna_ptr->bcst_rx_pn_val);
    }
    else
    {
        /* Set the broadcast PN value for this STA index */
        machw_ce_init_rx_bcmc_pn_val(0, 0, rsna_ptr->bcst_rx_pn_val);
    }
}

/* This function is used to remove key of particular index */
INLINE void remove_11i_key_sta(UWORD8 *val)
{
    UWORD8 sta_addr[6]  = {0};
    sta_entry_t *se     = NULL;

    /* delete TKIP/CCMP Unicast/Broadcast key to MAC HW CE*/
    memcpy(sta_addr, val+9, 6);

    se = find_entry(sta_addr);

    if(se == NULL)
    {
        return;
    }

    machw_ce_del_key(se->sta_index);
}

/* For all the incoming packets, the security check is performed so as to*/
/* determine if the incoming packets have been tampered                  */
INLINE BOOL_T check_11i_rx_mpdu_sta(sta_entry_t *se, CIPHER_T ct, UWORD8 *msa,
                                    UWORD8 *old_pn_val)
{
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* Perform replay detection. Reordering of MSDUs is not supported */
        /* since only one counter per STA per SA is supported             */
        if((ct == TKIP) || (ct == CCMP))
        {
            UWORD8 rx_pn_val[6] = {0};
            UWORD8 da[6]        = {0};
            UWORD8 mac_hdr_len  = get_mac_hdr_len(msa);
            BOOL_T error        = BFALSE;

            /* If QOS is Enabled, then the H/w will be offsetting two extra  */
            /* bytes after the header to make it word aligned                */
            mac_hdr_len = (mac_hdr_len == MAC_HDR_LEN)? MAC_HDR_LEN :
                                                        (MAC_HDR_LEN + 4);

            /* Extract the destination address from the frame */
            get_DA(msa, da);

            if(se == NULL)
            {
                return BFALSE;
            }

            get_pn_val(msa + mac_hdr_len, (UWORD8)ct, rx_pn_val);

            /* Check PN Value wrap around condition for the recevied value */
            if((rx_pn_val[5] == 0xFF) &&
               (rx_pn_val[4] == 0xFF) &&
               (rx_pn_val[3] == 0xFF) &&
               (rx_pn_val[2] == 0xFF) &&
               (rx_pn_val[1] == 0xFF) &&
               (rx_pn_val[0] == 0xF0))
            {
                /* Send Deauth */
#ifdef IBSS_11I
                if(mget_DesiredBSSType() != INFRASTRUCTURE)
                {
                    rsna_send_deauth((UWORD8 *)se->auth_persta, 0);
                }
                else
#endif /* IBSS_11I */
                {
                    rsna_send_deauth((UWORD8 *)se->persta, 1);
                }
            }

            if(is_group(da) == BFALSE)
            {
                if(cmp_pn_val(old_pn_val, rx_pn_val) != BTRUE)
                {
                    error = BTRUE;
                }
            }
            else
            {
                if(get_GTK_ID(msa + mac_hdr_len) != se->persta->GN)
                {
                    /* Frame with Wrong GTK ID: Drop it  */
#ifdef DEBUG_MODE
                    g_mac_stats.pewrxwrongid++;
#endif /* DEBUG_MODE */
                    return BFALSE;

                }

                if(cmp_pn_val(old_pn_val, rx_pn_val) != BTRUE)
                {
                    error = BTRUE;
                }
            }

            /* If replay error was found */
            if(error == BTRUE)
            {
                /* The PN Sequence as received is out of order */
                /* Replay error is logged                      */
                if(ct == CCMP)
                {
                    mincr_RSNAStatsCCMPReplays(se->persta->key_index);
                }
                else if(ct == TKIP)
                {
                    mincr_RSNAStatsTKIPReplays(se->persta->key_index);
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
INLINE void update_11i_success_stats_sta(sta_entry_t *se, CIPHER_T ct,
                                         UWORD8 status)
{
    if(se != NULL)
    {
        if((status == RX_SUCCESS) && ((ct == CCMP) || (ct == TKIP)))
            g_decr_fail_cnt = 0;
    }
}


/* This function checks whether a counter measure operation is currently */
/* in progress.                                                          */
INLINE BOOL_T is_cntr_msr_in_progress_sta(void)
{
    BOOL_T retval = BFALSE;

    retval = g_rsna_supp.global.cntr_msr_in_progress;

#ifdef IBSS_11I
    if(retval == BFALSE)
        retval = g_rsna_auth.global.cntr_msr_in_progress;
#endif /* IBSS_11I */

    return retval;
}

/* This function returns the cached PMKIDs corresponding to the passed BSSID */
INLINE UWORD8 get_cached_pmkids(UWORD8 *bssid, UWORD8 *pmkid)
{
    UWORD8 num_pmkid = 0;

	if(BFALSE == g_int_supp_enable)
    	num_pmkid = search_pmkid_cache(bssid, pmkid);

    return num_pmkid;
}
#endif /* IEEE_11I_STA_H */

#endif /* MAC_802_11I */

#endif /* IBSS_BSS_STATION_MODE */
