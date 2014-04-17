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
/*  File Name         : sta_prot_if.h                                        */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      STA mode.                                            */
/*                                                                           */
/*  List of Functions : Protocol interface functions for STA mode            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef STA_PROT_IF_H
#define STA_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "cglobals_sta.h"
#include "prot_if.h"
#include "wep_sta.h"
#include "pm_sta.h"
#include "management_sta.h"
#include "channel_sw.h"
#include "ch_info.h"

#ifdef MAC_802_11I
#include "ieee_11i_sta.h"
#include "ieee_11i_auth.h"
#include "rsna_supp_km.h"
#include "sta_frame_11i.h"
#include "auth_frame_11i.h"
#endif /* MAC_802_11I */

#ifdef MAC_WMM
#include "mib_11e.h"
#include "sta_frame_11e.h"
#include "sta_management_11e.h"
#include "sta_uapsd.h"
#endif /* MAC_WMM */

#ifdef MAC_802_11H
#include "dfs_sta.h"
#include "tpc_sta.h"
#include "ieee_11h_sta.h"
#include "mib_11h.h"
#endif /* MAC_802_11H */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
#include "frame_mdom.h"
#include "mib_mdom.h"
#include "sta_management_mdom.h"
#endif /* MAC_MULTIDOMAIN */

#ifdef MAC_802_11N
#include "management_11n.h"
#include "sta_frame_11n.h"
#include "sta_management_11n.h"
#endif /* MAC_802_11N */

#ifdef INT_WPS_SUPP
#include "wps_sta.h"
#endif /* INT_WPS_SUPP */

#ifdef MAC_P2P
#include "p2p_wlan_rx_mgmt_sta.h"
#include "sta_mgmt_p2p.h"
#include "mgmt_p2p.h"
#include "p2p_ps_sta.h"
#endif /* MAC_P2P */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
#include "wapi.h"
#endif

/*junbinwang add for wps 20130811*/
extern WORD32 trout_is_probe_req_wps_ie(void);
extern WORD32 trout_get_probe_req_wps_ie_len(void);
extern UWORD8* trout_get_probe_req_wps_ie_addr(void);
extern void trout_clear_probe_req_wps_ie(void);
extern WORD32 trout_is_asoc_req_wps_ie(void);
extern WORD32 trout_get_asoc_req_wps_ie_len(void);
extern UWORD8* trout_get_asoc_req_wps_ie_addr(void);
extern void trout_clear_asoc_req_wps_ie(void);
extern void trout_set_wps_sec_type_flag(WORD32 flag);
extern WORD32 trout_is_wps_sec_type_flag(void);
extern void wps_gen_uuid(UWORD8 * mac_addr, UWORD8 * uuid, UWORD8 * temp);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* 802.11i Related Prot-ifcation functions                                   */
/*****************************************************************************/
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
/* This function returns the internal supplicant info */
INLINE UWORD8 get_int_supp_mode_sta(void)
{
#ifndef SUPP_11I
    return 0;
#endif /* SUPP_11I */

    return g_int_supp_enable;
}

INLINE void set_int_supp_mode_sta(UWORD8 val)
{
#ifdef SUPP_11I
    if(val)
    {
        g_int_supp_enable = BTRUE;
    }
    else
    {
        g_int_supp_enable = BFALSE;
    }
#else /* SUPP_11I */
    g_int_supp_enable = BFALSE;
#endif /* SUPP_11I */
}
#endif

INLINE BOOL_T is_sec_hs_complete(sta_entry_t *se)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
        return BFALSE;
#endif /* INT_WPS_SUPP */

#ifdef SUPP_11I
   if(BTRUE == g_int_supp_enable)
   {
       return is_11i_hs_complete(se);
   }
   else
#endif /* SUPP_11I */
   {
       return BTRUE;
   }
}

/* If the received packet is a security handshake packet, process the */
/* packet in the security FSM                                         */
INLINE BOOL_T is_sec_handshake_pkt_sta(sta_entry_t *se,
                                       UWORD8 *buffer, UWORD8 offset,
                                       UWORD16 rx_len, CIPHER_T ct)
{
    if(trout_is_wps_sec_type_flag() == 1)
    {
        return BFALSE;
    }
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        process_wps_pkt_sta((buffer + offset), rx_len);
        return BTRUE;
    }
#endif /* INT_WPS_SUPP */

#ifdef SUPP_11I
#ifndef EXT_SUPP_11i
   if(BTRUE == g_int_supp_enable)
   {
       return is_11i_handshake_pkt_sta(se, buffer, offset, rx_len, ct);
   }
   else
#endif   	
#endif /* SUPP_11I */
   {
       return BFALSE;
   }
}

/* Filter the frame received from the WLAN based on the security policy */
INLINE BOOL_T filter_wlan_rx_sec_sta(wlan_rx_t *wlan_rx)
{
#ifdef MAC_802_11I
    if(filter_wlan_rx_11i_sta(wlan_rx) == BTRUE)
    {
        return BTRUE;
    }
#endif /* MAC_802_11I */

    return filter_wlan_rx_wep_sta((CIPHER_T)wlan_rx->ct, wlan_rx->sub_type);
}

/* This function sets the 802.11I parameter element in the STA mode frames */
/* protocols in use.                                                       */
INLINE UWORD8 set_sec_info_element_sta(UWORD8 *data, UWORD16 index, UWORD8 supp)
{
#ifdef MAC_802_11I
    if(supp == 1)
    {
        return set_rsn_element_sta(data, index, mget_RSNAModeRequested());
    }
    else
    {
        UWORD16 len = 0;
#ifdef IBSS_11I
        if(g_mode_802_11i & RSNA_802_11I)
        {
            len += set_rsn_element_auth(data, index, RSNA_802_11I);
        }
        if(g_mode_802_11i & WPA_802_11I)
        {
            len += set_rsn_element_auth(data, index + len, WPA_802_11I);
        }
#endif /* IBSS_11I */
        return len;
    }
#else /* MAC_802_11I */
    return 0;
#endif /* MAC_802_11I */
}

/* Initialize security related FSMs */
INLINE void intialize_sec_km_sta(void)
{
#ifdef MAC_802_11I
    initialize_supp_rsna();
#ifdef IBSS_11I
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        initialize_auth_rsna();
    }
#endif /* IBSS_11I */
#endif /* MAC_802_11I */
}

/* Stop security related FSMs */
INLINE void stop_sec_km_sta(void)
{
#ifdef MAC_802_11I
    stop_supp_rsna();
#endif /* MAC_802_11I */
}

/* Delete the security entry by deleting LUT and stopping the FSMs */
INLINE void delete_sec_entry_sta(sta_entry_t *se)
{
#ifdef MAC_802_11I
    delete_11i_entry_sta(se);
#endif /* MAC_802_11I */
    delete_wep_entry_sta(se);
}

/* Initialize the security handle to the default parameters */
INLINE BOOL_T check_bss_sec_info_sta(bss_dscr_t *bss_dscr)
{
#ifdef MAC_802_11I
    return check_rsn_capabilities_sta((void *)bss_dscr);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}

//chenq add for check link ap info change 2013-06-08
INLINE BOOL_T check_bss_sec_info_sta_change(bss_dscr_t *bss_dscr)
{
#ifdef MAC_802_11I
    return check_rsn_capabilities_sta_change((void *)bss_dscr);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}
//chenq add end

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
INLINE BOOL_T check_bss_reg_domain_info(bss_dscr_t *bss_dscr)
{
#ifdef MAC_MULTIDOMAIN
    return check_bss_reg_domain_cap_info((void *)bss_dscr);
#else /* MAC_MULTIDOMAIN */
    return BTRUE;
#endif /* MAC_MULTIDOMAIN */
}

INLINE void adjust_bss_width_in_new_reg_class(UWORD8 reg_class)
{
#ifdef MAC_MULTIDOMAIN
    adjust_bss_width_in_new_rc(reg_class);
#endif /* MAC_MULTIDOMAIN */
}

INLINE void adopt_new_reg_class(UWORD8 reg_class, UWORD8 ch_idx, UWORD8 sec_ch_offset)
{
#ifdef MAC_MULTIDOMAIN
	adopt_new_rc(reg_class, ch_idx, sec_ch_offset);
#endif /* MAC_MULTIDOMAIN */
}
#endif

/*junbinwang add wps 20130812*/
extern WORD32 trout_is_wps_sec_type_flag(void);
//xuan yang, 2013-6-28, check sec type 
INLINE BOOL_T check_sec_type_sta(UWORD8 *msa, UWORD16 rx_len, UWORD16 cap_info)
{
#ifdef INT_WPS_SUPP  //wxb add
	if (WPS_ENABLED== get_wps_mode())
	{
		return BTRUE;
	}
#endif /* ifdef INT_WPS_SUPP */
	UWORD8    sec_mode = 0;
#ifdef MAC_WAPI_SUPP
	UWORD8    i = 0;
#endif

        /*junbinwang modify for wps 20130812*/
       if(trout_is_wps_sec_type_flag() == 1){
             return BTRUE;
       }

	//get current sec type
	sec_mode =  get_802_11I_mode_prot();
	sec_mode &= (BIT4|BIT3);

#ifdef MAC_802_11I
	//get beacon dot11 sec type
	update_scan_response_11i(msa, rx_len, MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN, 0);
	if(mget_RSNAEnabled() == TV_TRUE)
	{
		if((sec_mode & g_bss_dscr_set[0].dot11i_info) == sec_mode) 
		{			
			return BTRUE;
		}
		else 
		{			
			return BFALSE;
		}
	}
#endif
	if(is_wep_enabled()==BTRUE) 
	{
		if(BTRUE == check_bss_capability_info(cap_info)) 
		{
#ifdef MAC_802_11I
			if(0 != g_bss_dscr_set[0].dot11i_info) 
			{			
			  return BFALSE;
			} 
			else 
#endif
			{			
			  return BTRUE;
			}  
		} 
		else 
		{			
			return BFALSE;
		}
	}
#ifdef  MAC_802_11I
	if(0 != g_bss_dscr_set[0].dot11i_info) 
	{			
	  return BFALSE;
	}
#endif

#ifdef MAC_WAPI_SUPP
	//xuanyang, 2013.7.18, handle wapi type
	update_scan_response_wapi(msa, rx_len, MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN, 0);

	if(mget_wapi_enable() == TV_TRUE)
	{
		if(g_bss_dscr_set[0].wapi_found == BFALSE)
		{
			return BFALSE;
		}

		if(g_bss_dscr_set[0].wapi_akm_cnt != mget_wapi_akm_cnt())
		{
			return BFALSE;
		}

		for( i = 0; i < mget_wapi_akm_cnt();i++)
		{
			if( memcmp(mget_wapi_akm(i),g_bss_dscr_set[0].wapi_akm[i],4) != 0 )
			{
				return BFALSE;
			}
		}

		if(g_bss_dscr_set[0].wapi_pcip_cnt != mget_wapi_pcip_cnt())
		{
			return BFALSE;
		}

		for( i = 0; i < mget_wapi_pcip_cnt();i++)
		{
			if( memcmp(mget_wapi_pcip_policy(i),g_bss_dscr_set[0].wapi_pcip_policy[i],4) != 0 )
			{
				return BFALSE;
			}
		}

		if( memcmp(mget_wapi_grp_policy(),g_bss_dscr_set[0].wapi_grp_policy,4) != 0 )
		{
				return BFALSE;
		}
			
		return BTRUE;
	}
	else
	{
		if(g_bss_dscr_set[0].wapi_found == BTRUE)
		{
			return BFALSE;
		}
	}
#endif
	
	if((cap_info & PRIVACY) == PRIVACY)
	{			
	  return BFALSE;
	}
	
	return BTRUE;
}

/* Initialize the security handle to the default parameters */
INLINE BOOL_T check_sec_capability_sta(UWORD8 *msa, UWORD16 rx_len)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        g_bss_dscr_set_index = 0;
        update_scan_response_11i(msa, rx_len, get_mac_hdr_len(msa),
                                 g_bss_dscr_set_index);
        if(g_bss_dscr_set[g_bss_dscr_set_index].rsn_found == BTRUE)
        {
            return check_rsn_capabilities_sta((void *)g_bss_dscr_set);
        }
        else
        {
            return BFALSE;
        }
    }
#endif /* MAC_802_11I */

    if(is_wep_enabled() == BTRUE)
    {
        /* Check the cap_info */
        return check_bss_capability_info(get_cap_info(msa));
    }
    return BTRUE;
}

/* Handles the TBTT Event for security module */
INLINE void handle_tbtt_sec_event(void)
{
#ifdef MAC_802_11I
#ifdef IBSS_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        handle_tbtt_11i_event();
    }
#endif /* IBSS_11I */
#endif /* MAC_802_11I */
}

/* Copy security parameters from the descriptor set to the given descriptor */
INLINE void bss_dscr_copy_sec_info_sta(bss_dscr_t *bss_dscr, UWORD8 i)
{
#ifdef MAC_802_11I
    bss_dscr->rsn_found      = g_bss_dscr_set[i].rsn_found;
    bss_dscr->rsn_type       = g_bss_dscr_set[i].rsn_type;
    bss_dscr->rsn_grp_policy = g_bss_dscr_set[i].rsn_grp_policy;
    bss_dscr->wpa_grp_policy = g_bss_dscr_set[i].wpa_grp_policy;
    bss_dscr->mode_802_11i   = g_bss_dscr_set[i].mode_802_11i;
    memcpy(bss_dscr->rsn_pcip_policy, g_bss_dscr_set[i].rsn_pcip_policy, 3);
    memcpy(bss_dscr->rsn_auth_policy, g_bss_dscr_set[i].rsn_auth_policy, 3);
    memcpy(bss_dscr->rsn_cap, g_bss_dscr_set[i].rsn_cap, 2);
    memcpy(bss_dscr->wpa_pcip_policy, g_bss_dscr_set[i].wpa_pcip_policy, 3);
    memcpy(bss_dscr->wpa_auth_policy, g_bss_dscr_set[i].wpa_auth_policy, 3);
#endif /* MAC_802_11I */
}

/* Initialize the security handle to the default parameters */
INLINE BOOL_T init_sec_entry_sta(sta_entry_t *se, UWORD8 *sa)
{
    if(se == NULL)
    {
        return BFALSE;
    }

#ifdef MAC_802_11I
    if(init_11i_entry_sta(se, sa) != BTRUE)
    {
        return BFALSE;
    }
#endif /* MAC_802_11I */

    if(is_wep_enabled() == BTRUE)
    {
        /* Add WEP Entry for the STA */
        /* Using Default KeyId & Sta Index generated while AUTH_REQ */
        add_wep_entry(se->sta_index, mget_WEPDefaultKeyID(), sa);
    }

    g_decr_fail_cnt = 0;

    return BTRUE;
}

/* Start the security FSM. This is done after reception of a */
/* successful Association Response                           */
INLINE void start_sec_fsm_sta(sta_entry_t *se)
{
#ifdef SUPP_11I
	//chenq add only wps use inter 11i
	#ifdef EXT_SUPP_11i

	#ifdef INT_WPS_SUPP
	if( (BTRUE == g_int_supp_enable) && (WPS_ENABLED == get_wps_mode()) )
	#else
	return;
	#endif/* INT_WPS_SUPP */
	
	#else	
    if(BTRUE == g_int_supp_enable)

	#endif /* EXT_SUPP_11i */
        start_11i_fsm_sta(se);
#endif /* SUPP_11I */
}

/* Filter the frame received from the Host based on the security policy */
INLINE BOOL_T filter_host_rx_sec_sta(wlan_tx_req_t * tx_req, CIPHER_T *ct,
                                     UWORD8 **info,UWORD8 key_type,
                                     sta_entry_t *se,BOOL_T ignore_port)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
        return BFALSE;
#endif /* INT_WPS_SUPP */

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            if(filter_host_rx_11i_supp_sta(tx_req->eth_type, ct, info, key_type, se,
                                           ignore_port) == BTRUE)
            {
                return BTRUE;
            }
        }
#ifdef IBSS_11I
        else
        {
            if(filter_host_rx_11i_auth_sta(ct, info, key_type, se,
                                           ignore_port) == BTRUE)
            {
                return BTRUE;
            }
        }
#endif /* IBSS_11I */
    }
#endif /* MAC_802_11I */

    return filter_host_rx_wep_sta(ct);
}

/* Before forwarding the packet across to the HOST interface */
/* security checks needs to performed on the states of the   */
/* transmitting station                                      */
INLINE BOOL_T sta_check_sec_tx_sta_state(sta_entry_t *se,
                 msdu_desc_t *frame_desc, CIPHER_T ct)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
        return BTRUE;
#endif /* INT_WPS_SUPP */


#ifdef MAC_802_11I
    return sta_check_11i_tx_sta_state(se, frame_desc, ct);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}

/* If 802.11i and 802.1x are enabled, initialize 802.1x related security */
INLINE void init_sec_auth_sta(void)
{
#ifdef MAC_802_11I
    init_sec_auth_sta_1x();
#endif /* MAC_802_11I */
}

/* If 802.11i and 802.1x are enabled, initialize 802.1x related security */
INLINE void init_sec_supp_sta(void)
{
#ifdef MAC_802_11I
    init_sec_supp_sta_1x();
#endif /* MAC_802_11I */
}

INLINE BOOL_T init_sec_auth_node_sta(void **sta_1x, UWORD8 *rsna, UWORD16 aid,
                                     void *sta_1x_buff)
{
#ifdef MAC_802_11I
    return init_sec_auth_node_sta_1x(sta_1x, rsna, aid, sta_1x_buff);
#else /* MAC_802_11I */
    return BFALSE;
#endif /* MAC_802_11I */
}

INLINE void free_sec_auth_node_sta(void **sta_1x)
{
#ifdef MAC_802_11I
    free_sec_auth_node_sta_1x(sta_1x);
#endif /* MAC_802_11I */
}

/* Stop 802.1x related security */
INLINE void stop_sec_auth_node_sta(void)
{
#ifdef MAC_802_11I
    stop_sec_auth_node_sta_1x();
#endif /* MAC_802_11I */
}

/* Initialize 802.1x related security for STA */
INLINE BOOL_T init_sec_supp_node_sta(void **sta_1x, UWORD8 *rsna, UWORD16 aid,
                                     void *supp_1x_buff)
{
#ifdef MAC_802_11I
    return init_sec_supp_node_sta_1x(sta_1x, rsna, aid, supp_1x_buff);
#else /* MAC_802_11I */
    return BFALSE;
#endif /* MAC_802_11I */
}

INLINE void free_sec_supp_node_sta(void **sta_1x)
{
#ifdef MAC_802_11I
    free_sec_supp_node_sta_1x(sta_1x);
#endif /* MAC_802_11I */
}

/* Stop 802.1x related security */
INLINE void stop_sec_supp_node_sta(void)
{
#ifdef MAC_802_11I
    stop_sec_supp_node_sta_1x();
#endif /* MAC_802_11I */
}

/* Set/Get Supplicant Username */
INLINE void set_supp_username_sta(UWORD8 *val)
{
#ifdef MAC_802_11I
    set_supp_username_sta_1x(val);
#endif /* MAC_802_11I */
}

INLINE UWORD8 *get_supp_username_sta(void)
{
#ifdef MAC_802_11I
    return get_supp_username_sta_1x();
#else /* MAC_802_11I */
    return NULL;
#endif /* MAC_802_11I */
}

/* Set/Get Supplicant Password */
INLINE void set_supp_password_sta(UWORD8 *val)
{
#ifdef MAC_802_11I
    set_supp_password_sta_1x(val);
#endif /* MAC_802_11I */
}

INLINE UWORD8 *get_supp_password_sta(void)
{
#ifdef MAC_802_11I
    return get_supp_password_sta_1x();
#else /* MAC_802_11I */
    return NULL;
#endif /* MAC_802_11I */
}

INLINE void compute_install_sec_key_sta(void)
{
#ifdef SUPP_11I
    if(BTRUE == g_int_supp_enable)
    {
        install_psk();
    }
#endif /* SUPP_11I */
}

INLINE void update_sec_fail_stats_sta(CIPHER_T ct, UWORD8 status, UWORD8 *sa,
                                      UWORD8 *da)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        update_11i_fail_stats_sta(ct,status,sa,da);
    }
#endif /* MAC_802_11I */
}

/* This function updates the security statistics on a successful reception */
INLINE void update_sec_success_stats_sta(sta_entry_t *se, CIPHER_T ct,
                                         UWORD8 status)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        update_11i_success_stats_sta(se, ct, status);
    }
#endif /* MAC_802_11I */
}

/*****************************************************************************/
/* WMM Related Prot-ifcation functions                                       */
/*****************************************************************************/

INLINE void init_prot_globals_sta(void)
{
#ifdef MAC_WMM
    set_ap_wmm_cap(BFALSE);
#endif /* MAC_WMM */

#ifdef MAC_802_11H
    /* Initialize all dot11h DFS feature related variables */
    init_dfs_sta_globals();
#endif /* MAC_802_11H */


#ifdef INT_WPS_SUPP
    init_wps_sta_globals();
#endif /* INT_WPS_SUPP */

#ifdef MAC_P2P
    init_p2p_sta_globals();
    init_p2p_globals();
#endif /* MAC_P2P */
}

/* Set the WMM information field association request frame */
INLINE UWORD16 set_wmm_asoc_req_params(UWORD8 *data, UWORD16 index)
{
//chenq add for wapi 2013-02-16
#if 1
#ifdef MAC_WAPI_SUPP	
	if( mget_wapi_enable() == TV_TRUE )
	{
		return 0;
	}
#endif
#endif

#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        if(is_ap_wmm_cap() == BTRUE)
            return set_wmm_info_element_sta(data, index);
    }
#endif /* MAC_WMM */
    return 0;
}

/* Update the protocol dependent parameters */
INLINE void update_qos_params(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                              UWORD8 sub_type)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        update_params_wmm_sta(msa, rx_len, index, sub_type);
    }
#endif /* MAC_WMM */
}

/* Set the WMM parameters field for beacon frames */
INLINE UWORD16 set_wmm_beacon_params(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return set_wmm_info_element_sta(data, index);
#endif /* MAC_WMM */
    return 0;
}

/* Set the WMM parameters field for probe response frame */
INLINE UWORD16 set_wmm_probe_rsp_params(UWORD8 *data, UWORD16 index,
                                        UWORD8 *probe_req)
{
#ifdef MAC_WMM
    /* The WMM Parameter element is added in the probe response frames that  */
    /* are to be sent in response to probe request frames from the same IBSS */
    if(get_wmm_enabled() == BTRUE)
    {
        UWORD8 bssid[6] = {0};

        get_BSSID(probe_req, bssid);

        if((mget_DesiredBSSType() == INDEPENDENT) &&
           (mac_addr_cmp(bssid, mget_bssid()) == BTRUE))
        {
            return set_wmm_param_element_sta(data, index);
        }
    }
#endif /* MAC_WMM */
    return 0;
}

/* If 802.11e is enabled, set the QoS control field */
INLINE void set_qos_control_field_sta(UWORD8 *header, UWORD8 tid, UWORD8 ap)
{
#ifdef MAC_WMM
    set_qos_control_sta(header, tid, ap);
#endif /* MAC_WMM */
}

/* This function initializes power management related globals based on the   */
/* protocol defined                                                          */
INLINE void init_psm_prot_globals(void)
{
#ifdef MAC_WMM
    init_uapsd_globals();
#endif /* MAC_WMM */
}

/* This function is used for setting the frame control field for a QOS NULL  */
/* frame based on the protocol defined                                       */
INLINE UWORD8 set_frame_ctrl_qos_null(UWORD8* header, UWORD8 priority)
{
#ifdef MAC_WMM
    set_frame_control(header, (UWORD16)QOS_NULL_FRAME);
    set_qos_control_sta(header, priority, NORMAL_ACK);
    return QOS_MAC_HDR_LEN;
#else /* MAC_WMM */
    return MAC_HDR_LEN;
#endif /* MAC_WMM */
}

/* Change the priority of the packet, if ACM for that queue is set.          */
/* This function returns BFALSE if priority conversion is not possible.      */
/* In that case packet transmission should be aborted                        */
INLINE BOOL_T change_priority_if(wlan_tx_req_t *msg)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return change_wmm_priority_if(msg);
#endif /* MAC_WMM */
    return BTRUE;
}

/* This functions handles a packet transmission event for STA mode based on  */
/* the protocol defined                                                      */
INLINE PS_STATE_T psm_handle_tx_packet_sta_prot(UWORD8 priority)
{
#ifdef MAC_WMM
    /* If the AP is UAPSD capable and the AC for given priority is trigger   */
    /* enabled the UAPSD service period is started. Note that the frame      */
    /* transmission status needs to be checked before actually starting the  */
    /* service period. This is done during transmit complete processing.     */
    /* The power save state is STA_DOZE so that the AP continues to buffer   */
    /* packets for this STA                                                  */
    if(is_ap_uapsd_capable() == BTRUE)
    {
        if(is_trigger_enabled(priority) == BTRUE)
        {
            wait_uapsd_sp_start();
            return STA_DOZE;
        }

        if(is_delivery_enabled(priority) == BTRUE)
        {
            /* Exception case. Should not occur. If it is an only delivery   */
            /* enabled AC there should be no uplink traffic.                 */
#ifdef DEBUG_MODE
        g_mac_stats.wmmpssta_dacul++;
#endif /* DEBUG_MODE */
        }
    }
#endif /* MAC_WMM */

    /* The power save state is set to STA_ACTIVE if a packet is ready to be  */
    /* transmitted by the STA                                                */
    return STA_ACTIVE;
}

/* This function checks if the AC for the given priority is legacy */
INLINE BOOL_T is_legacy_ac(UWORD8 priority)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        return is_legacy(priority);
    }
#endif /* MAC_WMM */

    return BTRUE;
}

/* This function checks if any legacy AC is present for the STA */
INLINE BOOL_T is_any_legacy_ac_present(void)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        return is_legacy_ac_present();
    }
#endif /* MAC_WMM */

    return BTRUE;
}

/* This function checks if the AC for the given priority is trigger enabled */
INLINE BOOL_T is_trigger_enabled_ac(UWORD8 priority)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        return is_trigger_enabled(priority);
    }
#endif /* MAC_WMM */

    return BFALSE;
}

/* This function checks if the AC for the given priority is delivery enabled */
INLINE BOOL_T is_delivery_enabled_ac(UWORD8 priority)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        return is_delivery_enabled(priority);
    }
#endif /* MAC_WMM */

    return BFALSE;
}

/* This function handles a packet reception event for STA mode based on the  */
/* protocol defined                                                          */
INLINE void psm_handle_rx_packet_sta_prot(UWORD8 *msa, UWORD8 priority)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        uapsd_handle_wlan_rx_sta(msa, priority);
    }
#endif /* MAC_WMM */

#ifdef MAC_P2P
    /* On receiving a frame the PS-Poll response pending flag is reset */
    set_ps_poll_rsp_pending(BFALSE);
#endif /* MAC_P2P */

}

/* This function handles TIM set event for STA mode based on the protocol    */
/* defined                                                                   */
INLINE void psm_handle_tim_set_sta_prot(void)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        uapsd_trigger_sp_sta();
    }
#endif /* MAC_WMM */
}

/* This function handles TIM set event for STA mode based on the protocol    */
/* defined                                                                   */
INLINE void handle_tx_complete_sta_prot(UWORD8 *msa, UWORD8 tx_success)
{
#ifdef MAC_WMM
    if(is_ap_uapsd_capable() == BTRUE)
    {
        uapsd_handle_tx_complete_sta(msa, tx_success);
    }
#endif /* MAC_WMM */
}

/* This function returns the highest priority which is trigger enabled */
INLINE UWORD8 get_highest_trigger_enabled_priority(void)
{
#ifdef MAC_WMM
    UWORD8 pri = INVALID_PRI;

    /* Check for highest priority trigger enabled AC */
    if(is_trigger_enabled(PRIORITY_3) == BTRUE)
    {
        pri = PRIORITY_3;
    }
    else if(is_trigger_enabled(PRIORITY_2) == BTRUE)
    {
        pri = PRIORITY_2;
    }
    else if(is_trigger_enabled(PRIORITY_1) == BTRUE)
    {
        pri = PRIORITY_1;
    }
    else if (is_trigger_enabled(PRIORITY_0) == BTRUE)
    {
        pri = PRIORITY_0;
    }

    return pri;

#else /* MAC_WMM */

    return PRIORITY_3;

#endif /* MAC_WMM */
}

/*****************************************************************************/
/* 802.11n Related Prot-ifcation functions                                   */
/*****************************************************************************/

INLINE void set_ht_capinfo_sta(UWORD8 *data, TYPESUBTYPE_T frame_type)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        set_ht_capinfo_field_sta(data, frame_type);
    }
#endif /* MAC_802_11N */
}

INLINE void *get_ba_handle_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    if(NULL != entry)
        return &(entry->ba_hdl);
#endif /* MAC_802_11N */
    return NULL;
}

INLINE void *get_ht_handle_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    if(NULL != entry)
        return &(entry->ht_hdl);
#endif /* MAC_802_11N */
    return NULL;
}

INLINE BOOL_T is_ht_capable_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (1 == entry->ht_hdl.ht_capable))
        return BTRUE;
#endif /* MAC_802_11N */
    return BFALSE;
}

INLINE BOOL_T is_imm_ba_capable_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    if(1 == entry->ba_hdl.imm_ba_capable)
        return BTRUE;
#endif /* MAC_802_11N */
    return BFALSE;
}

/* This function updates the STA entry params with the protocol specific */
/* parameters  contained in the information elements                     */
INLINE void update_sta_entry_prot(UWORD8 *msa, UWORD16 offset, UWORD16 rx_len,
                                  void *entry)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        update_11n_params_sta(msa, offset, rx_len, (sta_entry_t *)entry);
    }
#endif /* MAC_802_11N */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
        p2p_handle_noa(msa, rx_len);
#endif /* MAC_P2P */
#endif
}

/* This function checks whether the receiver is HT capable */
INLINE BOOL_T is_htc_capable_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (entry != NULL))
    {
        if(mget_HTControlFieldSupported() == TV_TRUE)
        {
            if((1 == entry->ht_hdl.ht_capable) && (1 == entry->ht_hdl.htc_support))
                return BTRUE;
        }
    }
#endif /* MAC_802_11N */
    return BFALSE;
}

/* This function updates the service class parameter */
INLINE void update_serv_class_prot_sta(sta_entry_t *entry, UWORD8 tid,
                                       UWORD8 *serv_class)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        if(entry != NULL)
        {
            update_serv_class_ba(&(entry->ba_hdl), tid, serv_class);
            update_serv_class_11n(&(entry->ht_hdl), tid, serv_class);
        }
    }
#endif /* MAC_802_11N */

}

/* This function locates and processes the HT Information element */
INLINE void process_ht_info_sta(UWORD8 *msa, UWORD16 rx_len, sta_entry_t *se)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        process_11n_ht_info_sta(msa, rx_len, se);
    }
#endif /* MAC_802_11N */
}

/* Set 11n SM power save based on STA mode */
INLINE void set_11n_smps_mode_sta(UWORD8 val)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
#ifndef NMAC_1X1_MODE
        set_sm_power_save_sta(val);
#endif /* NMAC_1X1_MODE */
    }
#endif /* MAC_802_11N */
}

/* Updates power save based on protocol */
INLINE void update_prot_power_save(void)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        update_mimo_power_save_sta();
    }
#endif /* MAC_802_11N */
}

/* Process the control message in station mode */
INLINE void sta_enabled_rx_control(UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        sta_enabled_rx_11n_control(msg);
    }
#endif /* MAC_802_11N */
}


/* Filter the frame received from the WLAN based on the service class */
INLINE BOOL_T filter_wlan_serv_cls_sta(wlan_rx_t *wlan_rx)
{
    BOOL_T retval = BFALSE;

#ifdef MAC_802_11N
    UWORD8 pri_val = wlan_rx->priority_val;

    retval = filter_wlan_ba_serv_cls(wlan_rx,
           &(((sta_entry_t*)wlan_rx->sa_entry)->ba_hdl.ba_rx_ptr[pri_val]));
#endif /* MAC_802_11N */

    return retval;
}

/* Handle action request based on protocol enabled */
INLINE void sta_enabled_action_req(mac_struct_t *mac, UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        sta_enabled_action_req_11n(msg);
    }
#endif /* MAC_802_11N */
}

/* Check if a packet must be buffered due to the service class policy */
INLINE BOOL_T is_serv_cls_buff_pkt_sta(sta_entry_t *entry, UWORD8 q_num,
                                       UWORD8 tid, UWORD8 *tx_dscr)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        if(entry != NULL)
        {
            return is_ba_buff_tx_pkt(&((sta_entry_t *)entry)->ba_hdl,
                                     q_num, tid, tx_dscr);
        }
    }
#endif /* MAC_802_11N */

    return BFALSE;
}

/* This function indicates if a transmit Block ACK session is active for the */
/* given station entry and TID.                                              */
INLINE BOOL_T is_txba_session_active_sta(sta_entry_t *entry, UWORD8 tid)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (NULL != entry))
    {
        ba_tx_struct_t *ba = entry->ba_hdl.ba_tx_ptr[tid];

        if(ba != NULL)
            return (ba->is_ba);
    }
#endif /* MAC_802_11N */

    return BFALSE;
}

/* This function updates the pending buffer parameters of the specified */
/* Tx-BA session.                                                       */
INLINE void update_txba_session_params_sta(sta_entry_t *entry, UWORD8 tid,
                                           UWORD16 timeout, UWORD8 max_num)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (NULL != entry))
    {
        ba_tx_struct_t *ba = entry->ba_hdl.ba_tx_ptr[tid];

        if(ba != NULL)
        {
            /* The pending buffer parameters are set to the minimum  */
            /* of the requirements of the existing B-Ack session and */
            /* the new A-MPDU session.                               */
            ba->max_pend     = MIN(ba->max_pend, max_num);
            ba->buff_timeout = MIN(ba->buff_timeout, timeout);
        }
    }
#endif /* MAC_802_11N */
}


/* This function checks if a given MCS is supported by the given STA entry.  */
/* 11N TBD - Note that only the Rx MCS bitmask is currently checked. The     */
/* Highest Supported Data Rate field is not checked.                         */
INLINE BOOL_T is_mcs_supp_sta(sta_entry_t *entry, UWORD8 mcs_index)
{
#ifdef MAC_802_11N
    if(is_ht_capable_sta(entry) == BTRUE)
    {
        UWORD8 *mcs_bmap   = entry->ht_hdl.rx_mcs_bitmask;
        UWORD8 byte_offset = 0;
        UWORD8 bit_offset  = 0;

        byte_offset = mcs_index >> 3;
        bit_offset  = mcs_index & 0x07;

        if((mcs_bmap[byte_offset] & (1 << bit_offset)) != 0)
        {
            return BTRUE;
        }
    }
#endif /* MAC_802_11N */

    return BFALSE;
}

#ifdef AUTORATE_FEATURE
INLINE void update_tx_mcs_index_ar_sta(sta_entry_t *se)
{
#ifdef MAC_802_11N
    UWORD8 indx = 0;
    UWORD8 rate = 0;
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_sta(se);

    /* Search for any MCS at lower indexes than the current Tx rate index in */
    /* the AR table                                                          */
    for(indx = se->tx_rate_index; indx > 0; indx--)
    {
        rate = get_ar_table_rate(indx);

        if(IS_RATE_MCS(rate) == BTRUE)
        {
            if((is_rate_supp(rate, se) == 1) &&
               (is_rate_allowed(rate, se) == 1))
            {
                ht_hdl->tx_mcs_index = indx;
                return;
            }
        }
    }

    /* If there are no MCS rates at lower indexes of AR table then try to */
    /* search at indexes greater than current TX rate                     */
    for(indx = (se->tx_rate_index + 1); indx < get_ar_table_size(); indx++)
    {
        if(IS_RATE_MCS(get_ar_table_rate(indx)) == BTRUE)
        {
            ht_hdl->tx_mcs_index = indx;
            return;
        }
    }

#ifdef DEBUG_MODE
    /* Exception */
    PRINTD("update_tx_mcs_index_ar: Exception\n\r");
    g_mac_stats.no_mcs_index++;
#endif /* DEBUG_MODE */
#endif /* MAC_802_11N */
}

/* This function checks if SGI transmission is enabled for current receiver */
INLINE BOOL_T is_sgi_possible_sta(sta_entry_t *se)
{
    BOOL_T ret_val      = BFALSE;
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_sta(se);

    /* Receiver supports Short GI for the reception of 20MHz packets and */
    /* Short GI in 20MHz is enabled for transmission or the Receiver     */
    /* supports Short GI for the reception of 40MHz packets and Short GI */
    /* in 40MHz  is enabled for transmission                             */
    if(((ht_hdl->short_gi_20 == 1) &&
        (mget_ShortGIOptionInTwentyEnabled() == TV_TRUE)) ||
       ((ht_hdl->short_gi_40 == 1) &&
        (mget_ShortGIOptionInFortyEnabled() == TV_TRUE)))
    {
        ret_val = BTRUE;
    }

#endif /* MAC_802_11N */
    return ret_val;
}

INLINE BOOL_T get_tx_sgi_sta(sta_entry_t *se)
{
    BOOL_T ret_val = BFALSE;
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_sta(se);

    ret_val = ht_hdl->tx_sgi;
#endif /* MAC_802_11N */

    return ret_val;
}

INLINE void set_tx_sgi_sta(BOOL_T tx_sgi, sta_entry_t *se)
{
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_sta(se);

    ht_hdl->tx_sgi = tx_sgi;
#endif /* MAC_802_11N */
}

#endif /* AUTORATE_FEATURE */

INLINE UWORD8 get_ht_tx_rate_sta(sta_entry_t *se)
{
#ifdef AUTORATE_FEATURE
    if(is_autorate_enabled() == BTRUE)
    {
        UWORD8 curr_tx_mcs = 0;
        ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_sta(se);

        curr_tx_mcs = get_ar_table_rate(ht_hdl->tx_mcs_index);

        /* If the rate is not an MCS return the PHY rate for MCS7 */
        if(IS_RATE_MCS(curr_tx_mcs) != BTRUE)
        {
            curr_tx_mcs = 0x87;
#ifdef DEBUG_MODE
            PRINTD("get_ht_tx_rate_sta() Exception\n\r");
            g_mac_stats.non_mcs_txrate++;
#endif /* DEBUG_MODE */
        }

        /* Return the rate in mbps as set for the associated STA. */
        return curr_tx_mcs;
    }
#endif /* AUTORATE_FEATURE */

#ifdef PHY_802_11n
    if(is_ht_capable_sta(se) == BTRUE)
    {
        UWORD8 curr_tx_mcs = 0;

        curr_tx_mcs  = get_curr_tx_mcs();

        /* If the Current TX MCS is not an valid MCS return the PHY rate */
        /* for MCS7                                                      */
        if(curr_tx_mcs != INVALID_MCS_VALUE)
        {
            curr_tx_mcs |= BIT7;
        }
        else
        {
            curr_tx_mcs = 0x87;
        }

        return curr_tx_mcs;
    }
#endif /* PHY_802_11n */

    /* Else return MCS7 */
    return 0x87;
}

/* Set the HT information field association request frame */
INLINE UWORD8 set_ht_asoc_req_params(UWORD8 *data, UWORD16 index)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        if(is_ap_ht_cap() == BTRUE)
        {
            retval += set_ht_cap_11n(data, index, ASSOC_REQ);
            index  += retval;

            retval += set_ext_cap(data, index);
            index  += retval;
        }
    }
#endif /* MAC_802_11N */

    return retval;
}

/* This function sets the HT-Operation Information element */
INLINE UWORD8 set_ht_opern_element_sta(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        return set_ht_opern_11n_sta(data, index);
    }
#endif /* MAC_802_11N */

    return 0;
}

// 20120709 caisf mod, merged ittiam mac v1.2 code
/* This function checks if all protocol allows sending of Qos NULL frame */
INLINE BOOL_T psm_allow_null_send_prot(sta_entry_t *entry, UWORD8 tid)
{
    BOOL_T ret_val = BTRUE;
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        if((entry != NULL) &&
           (frames_in_ba_pnd_q(&(entry->ba_hdl), tid) == BTRUE))
        {
            ret_val = BFALSE;
        }
    }
#endif /* MAC_802_11N */

    return ret_val;

}

/*****************************************************************************/
/* 11n 20/40 Interface Functions                                             */
/*****************************************************************************/
INLINE UWORD32 get_obss_scan_period_prot(UWORD32 last_scan_period)
{
    UWORD32 scan_period = 0x7FFFFFFF ;


     if(scan_period == 0x7FFFFFFF)
     {
         scan_period = 0;
     }

     return scan_period;
}

/* This function performs all protocol specific tasks before starting an */
/* OBSS Channel Scan.                                                    */
INLINE void start_obss_scan_prot(void)
{
}

/* This function performs all protocol specific tasks after an OBSS Channel */
/* Scan is completed.                                                       */
INLINE void end_obss_scan_prot(void)
{
}

/* This function should be called after completion of OBSS Scanning to send */
/* the Scan report if required.                                             */
INLINE void send_obss_scan_report_prot(void)
{
}

/* This function updates the S/w and H/w settings based on the applicable */
/* protocol settings before starting scanning.                            */
INLINE void update_scan_req_params_prot(scan_req_t *scan_req)
{

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        g_dev_dscr_set_index = 0;
        set_p2p_scan_state(NORMAL_SCAN);
    }
#endif /* MAC_P2P */
}

/* This function updates the S/w and H/w settings based on the applicable */
/* protocol settings before starting an IBSS N/w.                         */
INLINE void update_start_req_params_prot(start_req_t *start_req)
{
}

/* This function handles the secondary Channel Offset field advertised for */
/* the BSS by the AP.                                                      */
/* Inputs: sec_chan_offset - Secondary Channel Offset advertised by AP     */
/*         prog_chan       - BTRUE if RF Channel can be updated by this    */
/*                                 function if required.                   */
/*                           BFALSE if RF Channel should not be updated    */
/*                                 by this function                        */
INLINE void process_sec_chan_offset(UWORD8 sec_chan_offset, BOOL_T prog_chan)
{
}
/*****************************************************************************/
/* 802.11h protocol Interface Functions                                      */
/*****************************************************************************/

/* Set 11h related Elements */
INLINE UWORD8 set_11h_elements(UWORD8 *data, UWORD16 index)
{
    UWORD8 len = 0;
#ifdef MAC_802_11H
    /* If STAUT supports TPC */
    if(mget_enableTPC() == BTRUE)
    {
       /* Set Power Capability of STA  and update index value with the       */
       /* Power Capability length, if AP supports dot11h features.           */
        if(mget_SpectrumManagementRequired() == TV_TRUE)
        {
            len   += set_power_cap_element(data, index);
            index += len;
        }
    }

    /* If STAUT supports DFS */
    if(mget_enableDFS() == BTRUE)
    {
       /* Set Supported channel list and update index value with the supported  */
       /* channel list length, if AP supports dot11h features.                  */
        if(mget_SpectrumManagementRequired() == TV_TRUE)
        {
            len += set_sup_channel_list_element(data, index);
        }
    }

#endif /* MAC_802_11H */
    return len;
}

/* Create 11h related alarms */
INLINE void create_11h_alarms(void)
{
#ifdef MAC_802_11H
    /* Create DFS alarms */
    create_dfs_alarms();
#endif /* MAC_802_11H */
}

/* Delete 11h related alarms */
INLINE void delete_11h_alarms(void)
{
#ifdef MAC_802_11H
    delete_dfs_alarms();
#endif /* MAC_802_11H */
}

/* Handle TBTT for 11h BSS-STA */
INLINE void handle_tbtt_prot_sta(void)
{
#ifdef MAC_802_11H
    handle_tbtt_dfs_sta();
#endif /* MAC_802_11H */

#ifdef MAC_P2P
    /* Decrement the count for GO discoverability request frame */
    if(g_p2p_go_disc_req_rx > 0)
        g_p2p_go_disc_req_rx--;

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifndef MAC_P2P_HW_SUPP
    /* Process TBTT only to support P2P PS without MAC HW support. This      */
    /* support shall be removed in the future.                               */
    p2p_process_tbtt();
#endif /* MAC_P2P_HW_SUPP */
#endif

#endif /*MAC_P2P */
}

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
/* Update 11h related elements */
INLINE void update_11h_elemets(UWORD8 *msa,UWORD16 rx_len,
                                       UWORD16 tag_param_offset)
{
#ifdef MAC_802_11H
    if(mget_enableDFS() == BTRUE)
    {
       /* Update quiet element info */
       update_quiet_element(msa, rx_len, tag_param_offset);
    }

    if(mget_enableTPC() == BTRUE)
    {
       /* Update TPC related variables */
       update_power_constr (msa, rx_len, tag_param_offset);
       update_reg_max_power(msa, rx_len, tag_param_offset);
    }
#endif /* MAC_802_11H */
}

/* Process 11h related info */
INLINE void update_11h_prot_info(UWORD8 *msa, UWORD16 rx_len)
{
#ifdef MAC_802_11H
    update_11h_prot_params(msa, rx_len);

    if(mget_enableTPC() == BTRUE)
    {
        /* Update TPC related variables */
        update_power_constr (msa, rx_len, TAG_PARAM_OFFSET);
        update_reg_max_power(msa, rx_len, TAG_PARAM_OFFSET);
    }
#endif /* MAC_802_11H */

    update_csw_info(msa, rx_len);
}
#else
/* Update station protocol related elements */
INLINE void update_joinrx_prot_sta_elemets(UWORD8 *msa, UWORD16 rx_len,
                                       UWORD16 tag_param_offset)
{
#ifdef MAC_MULTIDOMAIN
	if((mget_MultiDomainOperationImplemented() == TV_TRUE) &&
	   (mget_MultiDomainCapabilityEnabled()    == TV_TRUE))
    {
        update_11d_reg_max_tx_pow(msa, rx_len, tag_param_offset);
    }
#endif /* MAC_MULTIDOMAIN */

#ifdef MAC_802_11H
    if(mget_enableDFS() == BTRUE)
    {
       /* Update quiet element info */
       update_quiet_element(msa, rx_len, tag_param_offset);
    }

    if(mget_enableTPC() == BTRUE)
    {
       /* Update TPC related variables */
       update_power_constr (msa, rx_len, tag_param_offset);
    }
#endif /* MAC_802_11H */
}

/* Process 11h related info */
INLINE void update_enable_sta_prot_info(UWORD8 *msa, UWORD16 rx_len)
{
#ifdef MAC_802_11H
    update_11h_prot_params(msa, rx_len);

    if(mget_enableTPC() == BTRUE)
    {
        /* Update TPC related variables */
        update_power_constr (msa, rx_len, TAG_PARAM_OFFSET);
    }
#endif /* MAC_802_11H */

    update_csw_info(msa, rx_len);
}
#endif

/* This function is used to check whether current channel is in radar band */
INLINE BOOL_T check_radar_ch(void)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_802_11H
    UWORD8 reg_class = 0;
    UWORD8 freq      = get_current_start_freq();
    reg_class_info_t *rc_info  = NULL;

    reg_class = get_reg_class_from_ch_idx(freq, mget_CurrentChannel());
	// 20120830 caisf mod, merged ittiam mac v1.3 code
	#if 0 
    rc_info   = get_reg_class_info(freq, reg_class);
	#else
    rc_info   = get_reg_class_info(freq, reg_class, get_current_reg_domain());
	#endif

    if(rc_info != NULL)
        ret_val = is_rc_dfs_req(rc_info);
#endif /* MAC_802_11H */

    return ret_val;
}

/* Enable Radar detection */
INLINE void enable_11h_radar_detection(void)
{
#ifdef MAC_802_11H
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        if(mget_enableDFS() == BTRUE)
        {
            /* Enable radar detection if following conditions meet  */
            /* DFS feature  is enabled, AP supports dot11h features,  */
            /* present channel may interfere with radar band */
            if(mget_SpectrumManagementRequired() == TV_TRUE)
            {
                if(check_radar_ch() == BTRUE)
                {
                    enable_radar_detection();
                    unmask_machw_radar_det_int();
                }
            }
        }
    }
#endif /* MAC_802_11H */
}

/* If the channel is blocked for transmission, then do only passive scanning */
INLINE BOOL_T skip_chnl_for_scan(UWORD8 ch_idx)
{
    UWORD8 freq = get_current_start_freq();
    BOOL_T retval = BFALSE;

    /* Skip the channel if information is not available for operation */
    if(is_ch_idx_info_available(freq, ch_idx) == BFALSE)
        return BTRUE;

#ifdef MAC_802_11H
    retval = is_this_chnl_blocked(freq, ch_idx);
#endif /* MAC_802_11H */

    return retval;
}

/* Post radar detect event */
INLINE void post_radar_detect_event_11h_sta(void)
{
#ifdef MAC_802_11H
    if(mget_enableDFS() == BTRUE)
    {
        /* Post event for sending autonoumous report */
        /* if this radar detection was unexpected  */
        if(BTRUE == can_dfs_sta_transmit_now())
        {
            misc_event_msg_t *misc = 0;
            misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

            if(misc != NULL)
            {
                misc->data = 0;
                misc->name = MISC_RADAR_DETECTED;

                post_event((UWORD8*)misc, MISC_EVENT_QID);
            }

        }
    }
    else
#endif /* MAC_802_11H */
    {
		PRINTD("Warning: Radar detected when 11H/DFS is OFF\n");

    	/* Disable Radar detection */
    	disable_radar_detection();
    	mask_machw_radar_det_int();
	}
}

/* Check if STA can transmit now */
INLINE BOOL_T can_sta_transmit_now_prot(void)
{
#ifdef MAC_802_11H
    if(can_dfs_sta_transmit_now() == BFALSE)
    {
        return BFALSE;
    }
#endif /* MAC_802_11H */
    return BTRUE;
}

/* Check power limit for 11h protocol */
INLINE BOOL_T check_misc_sta_cap_info_prot(UWORD16 cap_info)
{
#ifdef MAC_802_11H
    if(mget_enableTPC() == BTRUE)
    {
        if(check_tpc_power_limit() == BFALSE)
        {
            /* Check whether Local max power is below the RF min power */
            return BFALSE;
        }
    }
#endif /* MAC_802_11H */
    return BTRUE;
}

INLINE void handle_spec_mgmt_action_prot(UWORD8 *msa, UWORD16 rx_len,
                                         UWORD8 mac_hdr_len, UWORD32 *rx_dscr)
{
#ifdef MAC_802_11H
    identify_measure_req(msa, rx_len, mac_hdr_len, rx_dscr);
#endif /* MAC_802_11H */
}

INLINE void clear_wait_for_csa_prot(void)
{
#ifdef MAC_802_11H
    /* If STA is detected radar itself then it shceduled an alarm */
    /* to resume Tx,  stop alarm, if it is running  */
    if(g_waiting_for_csa == BTRUE)
    {
        stop_alarm(g_dfs_alarm_wait_for_csa);
        g_waiting_for_csa = BFALSE;
    }
#endif /* MAC_802_11H */
}

/*****************************************************************************/
/* P2P protocol Interface Functions                                          */
/*****************************************************************************/
/* This function checks if P2P Group Formation if in progress */
INLINE BOOL_T is_p2p_grp_form_in_prog(void)
{
#ifdef MAC_P2P
    return g_p2p_grp_form_in_prog;
#else /* MAC_P2P */
    return BFALSE;
#endif /* MAC_P2P */
}

INLINE BOOL_T get_prot_scan_type(BOOL_T send_prb_req)
{
#ifdef MAC_P2P
    if((BTRUE == is_p2p_grp_form_in_prog())||
       (SEND_INVITE == g_p2p_invit))
    {
        if(SEARCH_STATE == get_p2p_scan_state())
        {
            send_prb_req = BTRUE;
        }
        else if(LISTEN_STATE == get_p2p_scan_state())
        {
            send_prb_req = BFALSE;
        }
    }
#endif /* MAC_P2P */
    return send_prb_req;

}

INLINE SWORD8 handle_scan_itr_to(mac_struct_t *mac)
{
    SWORD8 chan2scan = -1;
#ifdef MAC_P2P
    if(WAIT_INVITE_RSP == g_p2p_invit)
    {
        /* The dwell timer has expired waiting for an invitation response in */
        /* the current channel */
        handle_inv_req_to(mac);
        return chan2scan;
    }

    chan2scan = handle_scan_itr_to_p2p(mac);
#else /* MAC_P2P */
	//chenq mod 0727,1019 resume
    if(g_channel_index >= g_num_channels)
    //if( (g_channel_index >= g_num_channels) && (g_channel_index != 0) )
    {
        process_scan_itr_comp(mac);
    }
    else
    {
        /* Scan other channels, channel index will be incremented */
        chan2scan = g_channel_list[g_channel_index];
        g_channel_index++;
    }
#endif /* MAC_P2P */
    return chan2scan;
}

INLINE void p2p_check_dev_match(mac_struct_t *mac)
{
#ifdef MAC_P2P
    if(g_p2p_match_idx >= 0)
    {
        /* Check if an invitation request is pending */
        if((SEND_INVITE == g_p2p_invit) &&
           CHECK_BIT(g_p2p_dev_dscr_set[g_p2p_match_idx].dev_capability,
                     P2P_INVIT_PROC))
        {
            p2p_send_inv_req(0, 0, 0, (UWORD8 *)(mget_DesiredSSID()),
                             g_p2p_go_dev_id, mget_CurrentChannel());

            g_p2p_invit = WAIT_INVITE_RSP;
            /* Restart the timeout timer for getting the response */
            start_mgmt_timeout_timer(RSP_TIMEOUT);
        }
        else
        {
            /* if the dev id matched the target dev id then send the */
            /* info to sme to process the same out of turn           */
            process_scan_itr_comp(mac);
        }
    }

    g_p2p_match_idx = -1;
#endif /* MAC_P2P */
}

INLINE void p2p_update_scan_resp(scan_rsp_t *scan_rsp)
{
#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        scan_rsp->p2p_dev_dscr    = g_p2p_dev_dscr_set;
        if(TIMEOUT == scan_rsp->result_code )
        {
            scan_rsp->result_code = (g_dev_dscr_set_index > 0) ?
                                    SUCCESS_MLMESTATUS : TIMEOUT;
        }

        scan_rsp->p2p_num_dscr = MIN(g_dev_dscr_set_index, MAX_DEV_FOR_SCAN);
    }
#endif /* MAC_P2P */
}

/* Handle the incoming MLME Join request based on protocol in use */
INLINE BOOL_T sta_handle_join_req_prot(mac_struct_t *mac, UWORD8 *msg)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        p2p_sta_wait_join(mac, msg);
        ret_val = BTRUE;
    }
#endif /* MAC_P2P */

    return ret_val;
}


/* Handle the protocol dependent action frames in WAIT_JOIN state */
INLINE void sta_wait_join_handle_action_prot(mac_struct_t *mac, UWORD8 *msa,
                                             UWORD16 rx_len)
{
#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        p2p_wait_join_handle_action_rx(mac, msa, rx_len);
    }
#endif /* MAC_P2P */
}

INLINE void process_deauth_prot(void)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        set_p2p_grp_form_in_prog(BTRUE);
        restore_user_settings_p2p();
    }
#endif /* MAC_P2P */
}

/* Set the P2P PS Poll pending flag whenever a PS-Poll frame is sent to AP */
INLINE void send_ps_poll_to_AP_prot(void)
{
#ifdef MAC_P2P
    set_ps_poll_rsp_pending(BTRUE);
#endif /* MAC_P2P */
}


/*****************************************************************************/
/* Scanning related Prot-ifcation functions                                  */
/*****************************************************************************/
// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/* This function is valid in P2P mode. It checks if the probe response or    */
/* beacon received is from the GO which the P2P device is supposed to        */
/* join. If it is not from that GO then frame discarded                      */
/* This  check is done so that the P2P device should do WPS only with the    */
/* device that matches with its target id and not with any other available   */
/* devices                                                                   */
INLINE BOOL_T handle_wps_scan_rsp_sta_prot(UWORD8 *msa, UWORD16 rx_len)
{
	BOOL_T retval = BTRUE;
#ifdef MAC_P2P
    UWORD16 ie_len         = 0;
    UWORD16 attr_len         = 0;
    UWORD8 *attr_start_ptr = NULL;
    UWORD8 *attr_ptr       = NULL;

	if(BTRUE == mget_p2p_enable())
	{
        retval = BFALSE;
        if(BFALSE == mac_addr_cmp(mget_p2p_trgt_dev_id(), get_SA_ptr(msa)))
        {
			/* Get the pointer to the combined P2P attributes */
			attr_start_ptr = get_p2p_attributes(msa, TAG_PARAM_OFFSET, rx_len,
												&ie_len);

			if(attr_start_ptr != NULL)
			{
				/* Check if the frame is sent by the GO which the P2P device */
				/* is supposed to join                                       */
				attr_ptr = p2p_get_attr(P2P_DEVICE_INFO, attr_start_ptr, ie_len,
									    &attr_len);
				if(attr_ptr == NULL)
				{
					attr_ptr = p2p_get_attr(P2P_DEVICE_ID, attr_start_ptr, ie_len,
									        &attr_len);
				}

				if(attr_ptr != NULL)
				{
					retval =  mac_addr_cmp(mget_p2p_trgt_dev_id(), attr_ptr);
				}

				/* Free the local memory buffer allocated for P2P attributes */
				mem_free(g_local_mem_handle, attr_start_ptr);

			}
		}
		else
		{
		  	retval = BTRUE;
	    }
	}
#endif /* MAC_P2P */
    return retval;
}
#endif

INLINE BOOL_T handle_scan_resp_prot(mac_struct_t *mac, UWORD8 *msa,
                                    UWORD16 rx_len, SWORD8 rssi,
                                    TYPESUBTYPE_T sub_type)
{
    /* Scan response frames are beacon and probe response  */
    /* Process them for all protocol and return the status */

    BOOL_T ret_val = BFALSE;

	TROUT_FUNC_ENTER;
#ifdef MAC_P2P
	// 20120830 caisf mod, merged ittiam mac v1.3 code
	#if 0
    if((BTRUE == is_p2p_grp_form_in_prog()) || (SEND_INVITE == g_p2p_invit))
    {
        /* The ret_val should be set to true if the frame is a probe response*/
        /* from a P2P device only. It should not be added to the bss         */
        /* descriptor                                                        */
        ret_val = p2p_get_dev(msa, TAG_PARAM_OFFSET, rx_len);
    }
	#else
    if(BTRUE == mget_p2p_enable())
    {
        if((BTRUE == is_p2p_grp_form_in_prog()) || (SEND_INVITE == g_p2p_invit))
        {
            /* The ret_val should be set to true if the frame is a probe response*/
            /* from a P2P device only. It should not be added to the bss         */
            /* descriptor                                                        */
            ret_val = p2p_get_dev(msa, TAG_PARAM_OFFSET, rx_len);
        }
	 else
	 {
			/* Check if the device sending the probe response or beacon frame    */
			/* should be added to the scan response list                         */
			if(BFALSE == handle_wps_scan_rsp_sta_prot(msa, rx_len))
				return BFALSE;
	 }
    }
	#endif
#endif /* MAC_P2P */

#ifdef INT_WPS_ENR_SUPP
    /* Process the scan response frame for WPS */
    if(WPS_ENABLED == get_wps_mode())
    {
        handle_wps_scan_rsp_sta((UWORD8*)mac, msa, rx_len, rssi);
        ret_val = BTRUE;
    }
#endif /* INT_WPS_ENR_SUPP */

	TROUT_FUNC_EXIT;
    return ret_val;
}

/* This function handles probe request during scanning */
INLINE void sta_wait_scan_handle_prb_req_prot(UWORD8 *msa, UWORD16 rx_len,
                                              SWORD8 rssi)
{
#ifdef MAC_P2P
    if(BTRUE        == is_p2p_grp_form_in_prog() &&
       LISTEN_STATE == get_p2p_scan_state())
    {
        p2p_sta_wait_scan_handle_prb_req(msa, rx_len);
    }
#endif /* MAC_P2P */
}


/* Handle the protocol dependent action frames in WAIT_SCAN state */
INLINE void sta_wait_scan_handle_action_prot(mac_struct_t *mac, UWORD8 *msa,
                                             UWORD16 rx_len, SWORD8 rssi)
{
#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog() || (NO_INVITE != g_p2p_invit))
    {
        p2p_wait_scan_handle_action_rx(mac, msa, rx_len);
    }
#endif /* MAC_P2P */

}

/* This function checks if a WMM service period is in progress. If it is, a  */
/* Trigger frame is queued.                                                  */
INLINE void p2p_go_absence_period_start_sta_prot(void)
{
#ifdef MAC_P2P
#ifdef MAC_WMM
    UWORD8 pri  = INVALID_PRI;

    /* If WMM-PS USP is in process, queue a QoS NULL trigger frame in the    */
    /* highest priority trigger enabled queue                                */
    if(is_uapsd_sp_not_in_progress() == BFALSE)
    {
        pri = get_highest_trigger_enabled_priority();

        if(pri != INVALID_PRI)
        {
            send_null_frame_to_AP(STA_DOZE, BTRUE, pri);
        }
    }
#endif /* MAC_WMM */
#endif /* MAC_P2P */
}
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
/* This funtions calls the funtion to set the P2P IE in the probe reponse    */
/* P2P is enabled                                                            */
INLINE UWORD16 set_p2p_ie_probe_req(UWORD8 *data, UWORD16 index)
{
    UWORD16 ret_val = 0;
#ifdef MAC_P2P
    if(mget_p2p_enable() == BTRUE)
    {
        ret_val = add_p2p_ie_probe_req(data, index);
    }
#endif /* MAC_P2P */
    return ret_val;
}
#endif

/*****************************************************************************/
/* Multiple protocol Related Prot-ifcation functions                         */
/*****************************************************************************/

/* This function gets the scan time required for any protocol */
INLINE UWORD16 get_scan_time_prot(UWORD16 scan_time, UWORD8 send_prb_req)
{
#ifdef MAC_P2P
    /* Over ride all other protocol in P2P Find phase */
    /* This part of the scan is exclusive to P2P      */
    if((BTRUE == is_p2p_grp_form_in_prog()) || (SEND_INVITE == g_p2p_invit))
    {
        if(LISTEN_STATE == get_p2p_scan_state())
        {
            UWORD8 rand_num = get_random_byte();

            /* Get a random 8 bit no. and evenly spread in 3 bins 255/3 = 85 */
            /* 0-85, 86-170, 170 - 255                                       */
            if(rand_num <= 85)
                scan_time = 100;
            else if ((rand_num <= 170))
                scan_time = 200;
            else
                scan_time = 300;
        }
        else if(SEARCH_STATE == get_p2p_scan_state())
        {
            scan_time = g_active_scan_time;
        }
    }
#endif /* MAC_P2P */

    return scan_time;
}


/* This function checks if the STA can doze based on the protocol defined */
INLINE BOOL_T can_sta_doze_prot(void)
{
#ifdef MAC_WMM
    /* If the AP is UAPSD capable and UAPSD service period is in progress    */
    /* or STA is waiting for UAPSD service period to start the STA cannot go */
    /* to doze mode.                                                         */
    if(is_ap_uapsd_capable() == BTRUE)
    {
        if(is_uapsd_sp_not_in_progress() == BFALSE)
            return BFALSE;
    }
#endif /* MAC_WMM */

#ifdef MAC_P2P
    if(g_p2p_go_disc_req_rx > 0)
        return BFALSE;
#endif /* MAC_P2P */

#ifdef MAC_802_11H
    /* Check if STA is waiting for AP response */
    if(can_dfs_sta_doze_now() == BFALSE)
    {
        return BFALSE;
    }
#endif /* MAC_802_11H */

    return BTRUE;
}

/* Handle the protocol dependent action frames */
INLINE void handle_action_prot_sta(UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        sta_enabled_rx_11n_action(msg);
    }
#endif /* MAC_802_11N */
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        sta_enabled_rx_p2p_action(msg);
    }
#endif /* MAC_P2P */
}

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
/* Set the QoS control field if 802.11e is enabled */
INLINE void handle_prot_tx_comp_sta(UWORD8 *dscr, void *entry)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        handle_11i_tx_comp_sta(dscr, entry);
    }
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        handle_ba_tx_comp(dscr, entry);
        handle_11n_tx_comp_sta(dscr);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11H
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        if(mget_enableDFS() == BTRUE)
        {
            /* Handle the tx complete for the DFS feature */
            if(g_sending_aut_report_in_progress == BTRUE)
            {
                handle_tx_complete_dfs_sta((UWORD32 *)dscr);
            }
        }
    }
#endif /* MAC_802_11H */

    return;
}
#else
/* Set the QoS control field if 802.11e is enabled */
INLINE void handle_prot_tx_comp_sta(UWORD8 *dscr, void *entry, UWORD8 *msa)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        handle_11i_tx_comp_sta(dscr, entry, msa);
    }
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        handle_ba_tx_comp(dscr, entry, msa);
        handle_11n_tx_comp_sta(dscr);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11H
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        if(mget_enableDFS() == BTRUE)
        {
            /* Handle the tx complete for the DFS feature */
            if(g_sending_aut_report_in_progress == BTRUE)
            {
                handle_tx_complete_dfs_sta((UWORD32 *)dscr);
            }
        }
    }
#endif /* MAC_802_11H */

    return;
}
#endif

/* Reset the security entry by deleting LUT and stopping the FSMs */
INLINE void reset_prot_entry_sta(sta_entry_t *se)
{
#ifdef MAC_802_11I
    reset_11i_entry_sta(se);
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        reset_ba_handle(&se->ba_hdl);
        reset_11n_entry_sta(se);
    }
#endif /* MAC_802_11N */
    reset_wep_entry_sta(se);
}

/* Initialize the security/QOS handle to the default parameters */
INLINE void init_prot_handle_sta(sta_entry_t *se)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        init_ba_handle(&(se->ba_hdl));
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    init_11i_handle_sta(se);
#endif /* MAC_802_11I */
}

/* Function is used to PTK key */
INLINE void add_ptk_sta(UWORD8 *val)
{
#ifdef MAC_802_11I
#ifndef EXT_SUPP_11i //chenq add for only wps
    if(BFALSE == g_int_supp_enable)
#endif
        add_11i_ptk_sta(val);
#endif /* MAC_802_11I */
}

/* Function is used to add GTK */
INLINE void add_rx_gtk_sta(UWORD8 *val)
{
#ifdef MAC_802_11I
#ifndef EXT_SUPP_11i //chenq add for only wps
    if(BFALSE == g_int_supp_enable)
#endif		
        add_11i_rx_gtk_sta(val);
#endif /* MAC_802_11I */
}


/* Function to remove key */
INLINE void remove_key_sta(UWORD8 *val)
{
#ifdef MAC_802_11I
    if(BFALSE == g_int_supp_enable)
        remove_11i_key_sta(val);
#endif /* MAC_802_11I */
}

/* Function is used to update PMKID Cache */
INLINE void set_pmkid_cache(UWORD8 *val)
{
#ifdef MAC_802_11I
#ifndef EXT_SUPP_11i //chenq add for only wps
    if(BFALSE == g_int_supp_enable)
#endif
        set_pmkid_cache_sta(val);
#endif /* MAC_802_11I */
}

/* Function is used get cached PMKID */
INLINE UWORD8 *get_pmkid_cache(void)
{
	UWORD8 *ret_val = NULL;
#ifdef MAC_802_11I
#ifndef EXT_SUPP_11i //chenq add for only wps
    if(BFALSE == g_int_supp_enable)
#endif
        ret_val = get_pmkid_cache_sta();
#endif /* MAC_802_11I */
    return ret_val;
}

/* Check for misc sta protocol events */
INLINE void misc_sta_enabled_prot_if(mac_struct_t *mac, UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        misc_11n_event(mac, msg);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    /* Misc Security related event */
    misc_11i_event_sta(mac, msg);
#endif /* MAC_802_11I */

#ifdef MAC_802_11H
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        /* Radar detection related event */
        misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

        if(misc_event_msg->name == MISC_RADAR_DETECTED)
        {
            misc_ieee_11h_radar_detected();
        }
    }
#endif /* MAC_802_11H */

#ifdef MAC_P2P
    misc_p2p_event_sta(msg);
#endif /* MAC_P2P */
}

/* Updates the protocol dependent parameters after scaning. */
INLINE void update_scan_response_prot(UWORD8 *msa, UWORD16 len,
                                      UWORD16 offset, UWORD8 dscr_set_index)
{
#ifdef MAC_WMM
    update_scan_response_wmm(msa, len, offset, dscr_set_index);
#endif /* MAC_WMM */

#ifdef MAC_802_11I
    update_scan_response_11i(msa, len, offset, dscr_set_index);
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    update_scan_response_11n(msa, len, offset, dscr_set_index);
#endif /* MAC_802_11N */

#ifdef MAC_P2P
    g_bss_dscr_set[dscr_set_index].p2p_manag = g_p2p_manageability;
#endif /* MAC_P2P */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
    update_scan_response_11d(msa, len, offset, dscr_set_index);
#endif /* MAC_MULTIDOMAIN */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
	update_scan_response_wapi(msa, len, offset, dscr_set_index);
#endif
}

/* Set the capability information fields for STA according to protocol */
INLINE void set_capability_prot_sta(UWORD8* data, UWORD16 index)
{
#ifdef MAC_802_11N
    set_capability_11n(data, index);
#endif /* MAC_802_11N */
}

/* For all the incoming MPDUs, the security check is performed so as to */
/* determine if the incoming packets have been tampered                 */
INLINE BOOL_T check_sec_rx_mpdu_sta(UWORD8 *se, CIPHER_T ct, UWORD8 *msa,
                                    UWORD8 *pn_val)
{
#ifdef MAC_802_11I
    return check_11i_rx_mpdu_sta((sta_entry_t*)se, ct, msa, pn_val);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}

/* This function checks whether the Receive Service Class for the specified */
/* entry is Block-Ack.                                                      */
INLINE BOOL_T is_rx_ba_service_class_sta(sta_entry_t *se, UWORD8 tid)
{
    BOOL_T retval = BFALSE;

#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (se != NULL))
    {
        retval = is_ba_rx_session_setup(&(se->ba_hdl.ba_rx_ptr[tid]));
    }
#endif /* MAC_802_11N */

    return retval;
}

/* This function checks whether a MAC reset is allowed to be done under the */
/* current operating state.                                                 */
INLINE BOOL_T allow_mac_reset_sta(void)
{
    BOOL_T retval = BTRUE;

#ifdef MAC_802_11I
    if(is_cntr_msr_in_progress_sta() == BTRUE)
        retval = BFALSE;
#endif /* MAC_802_11I */

    return retval;
}

/* This function returns the minimum non-zero TXOP limit configured across   */
/* all ACs. If all WMM is disabled or TXOP-Limits are zero, then 0xFFFF is   */
/* returned.                                                                 */
INLINE UWORD16 get_min_non_zero_txop_sta(sta_entry_t *se)
{
    UWORD16 min_txop = 0xFFFF;

#ifdef MAC_WMM
    if((get_wmm_enabled() == BTRUE) &&
       (is_dst_wmm_capable(se->sta_index) == BTRUE))
    {
        min_txop = get_min_non_zero_txop_wmm_sta();
    }
#endif /* MAC_WMM */

    return min_txop;
}

/* This function returns the number of TX-AMPDU sessions currently active */
/* with the specified station.                                            */
INLINE UWORD8 get_num_tx_ampdu_sessions_sta(sta_entry_t *entry)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
    retval = get_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */

    return retval;
}

/* This function increments the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void incr_num_tx_ampdu_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */
}

/* This function increments the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void decr_num_tx_ampdu_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */
}

/* This function returns the number of Block-Ack TX sessions setup with */
/* the specified station.                                               */
INLINE UWORD8 get_num_tx_ba_sessions_sta(sta_entry_t *entry)
{
    UWORD8  retval   = 0;

#ifdef MAC_802_11N
    retval = get_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */

    return retval;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_tx_ba_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_tx_ba_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_rx_ba_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_rx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_rx_ba_sessions_sta(sta_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_rx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function updates protocol settings based on the chosed pairwise */
/* cipher suite.                                                        */
INLINE void update_pcip_policy_prot_sta(UWORD8 pcip_policy)
{
#ifdef MAC_802_11N
    /* If HT is enabled and the chosen pairwise cipher suite is anything but */
    /* CCMP, then proceed by disabling HT.                                   */
    if((get_ht_enable() == 1) && (pcip_policy != 0x04))
        set_ht_enable(0);
#endif /* MAC_802_11N */
}


/* Updates the protocol dependent parameters before joining. */
INLINE void update_join_req_params_prot(join_req_t *join_req)
{
    /* Adapt spectrum management parameters to the BSS settings */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        check_spectrum_mgmt_bit(join_req->bss_dscr.cap_info);
    }

#ifdef MAC_WMM
    /* Update the AP WMM and WMM PS capability for BSS STA */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        set_ap_wmm_cap((BOOL_T)join_req->bss_dscr.wmm_cap);
        set_uapsd_status_qap_cap((BOOL_T)join_req->bss_dscr.uapsd_cap);
    }
#endif /* MAC_WMM */

#ifdef MAC_P2P
    update_join_req_params_p2p(join_req);
#endif /* MAC_P2P */

#ifdef MAC_802_11N
    /* Update the AP HT capability for BSS STA */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        set_ap_ht_cap((BOOL_T)join_req->bss_dscr.ht_capable);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    update_join_req_params_11i(&join_req->bss_dscr);
#endif /* MAC_802_11I */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
	update_join_req_params_wapi(&join_req->bss_dscr);
#endif

}

/* This function initiates 20/40 Operations in STA mode */
INLINE void start_2040_coex_sta(void)
{
}

#ifdef DEBUG_MODE
/* This function checks for consistency of the build parameters used */
/* in STA mode.                                                      */
INLINE void check_build_params_sta(void)
{

}
#endif /* DEBUG_MODE */

/* This function sets the address3 field in the probe response frame */
/* depending upon the protocol used                                  */
INLINE void set_address3_probe_resp_prot(UWORD8 *data)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
        set_address3(data, mget_p2p_dev_addr());
    else
#endif /* MAC_P2P */
        set_address3(data, mget_bssid());

}
// 20120709 caisf mod, merged ittiam mac v1.2 code
/* This function adds the ssid element in the probe response and probe req */
/* sent by the STA depending upon the protocol in use                      */
//INLINE UWORD8 set_ssid_probe_rsp_sta_prot(UWORD8 *data, UWORD8 index)
INLINE UWORD8 set_ssid_probe_sta_prot(UWORD8 *data, UWORD8 index)
{
    UWORD8 retval = 0;

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        retval = set_ssid_element(data, index, P2P_WILDCARD_SSID);
    }
    else
#endif /* MAC_P2P */
    {
	#ifndef COMBO_SCAN
	if(g_keep_connection == BTRUE)
	{
	    retval = set_ssid_element(data, index, "\0");
	}
	else
	{
	    retval = set_ssid_element(data, index, mget_DesiredSSID());
	}
	#else
    	//chenq mod for combo scan 2013-03-12 
        //retval = set_ssid_element(data, index, mget_DesiredSSID());
        if(!g_ap_combo_scan_cnt)
        {
                if(g_keep_connection == BTRUE)
                {
                    retval = set_ssid_element(data, index, "\0");
                }
                else
                {
                    retval = set_ssid_element(data, index, mget_DesiredSSID());
                }

	}
	else
	{
		g_ap_combo_scan_index++;
		if(g_ap_combo_scan_index > g_ap_combo_scan_cnt)
		{
                	if(g_keep_connection == BTRUE)
                	{
                    		retval = set_ssid_element(data, index, "\0");
                	}
                	else
                	{
                    		retval = set_ssid_element(data, index, mget_DesiredSSID());
                	}

			g_ap_combo_scan_index = 0;
		}
		else
		{
			retval = set_ssid_element(data, index, g_ap_combo_scan_list[g_ap_combo_scan_index-1]);
		}
	}
	#endif
    }

    return retval;
}


/* This function extracts the config method sent in the probe response frame */
/* and checks if it is a valid config method                                 */
INLINE void p2p_process_prob_rsp_sta_prot(UWORD8 *msa, UWORD16 rx_len,
                                          UWORD16 *pref_confg_meth)
{
#ifdef INT_WPS_SUPP
#ifdef MAC_P2P
    /* Call the function to extract the config method sent in the probe */
    /* probe response                                                   */
    wps_process_p2p_prob_rsp_enr(msa, rx_len, pref_confg_meth);
    /* Check is the confiduration method is valid */
    if((WPS_CONFIG_METH_DISPLAY != *pref_confg_meth) &&
       (WPS_CONFIG_METH_PUSHBUTTON != *pref_confg_meth) &&
       (WPS_CONFIG_METH_KEYPAD != *pref_confg_meth))
    {
            pref_confg_meth = 0;
    }
#endif /* MAC_P2P */
#endif /* INT_WPS_SUPP */

}

/* Update HT Powersave (SMPS) mode. This is always set to disabledin STA */
INLINE void set_ht_ps_params_sta(UWORD8 *dscr, sta_entry_t *se, UWORD8 rate)
{
#ifdef MAC_802_11N
    set_tx_dscr_smps_mode((UWORD32 *)dscr, MIMO_MODE);;
#endif /* MAC_802_11N */
}

/*****************************************************************************/
/* WPS protocol Interface Functions                                          */
/*****************************************************************************/

/* This function sets the required WPS IE in the given frame */
INLINE UWORD16 set_wps_element(UWORD8 *data, UWORD16 index, UWORD8 frm_type)
{
    UWORD8 ret_val= 0;
    UWORD32 len = 0;
    UWORD8* ptr = NULL;

#ifdef INT_WPS_SUPP
#ifdef MAC_P2P
    if(BTRUE == is_p2p_grp_form_in_prog())
    {
        if(g_wps_enrollee_ptr!=NULL) // caisf add this line "if"
            ret_val = p2p_add_wsc_ie(g_wps_enrollee_ptr, data + index, frm_type);
    }
    else
#endif /* MAC_P2P */
    {
        ret_val = set_wsc_info_element(data, index, frm_type);
	}

#else
        /*junbinwang add for wps 2013-0809*/
        if(trout_is_probe_req_wps_ie() == 1 && frm_type == PROBE_REQ)
        {
                len = trout_get_probe_req_wps_ie_len();
                ptr = trout_get_probe_req_wps_ie_addr();

                printk("[wjb] need prepare_probe_req len %d\n", len);
                memcpy(data+index, ptr, len);
                ret_val = len;
                //trout_clear_probe_req_wps_ie();
	   }
          else if(trout_is_asoc_req_wps_ie() == 1 && frm_type == ASSOC_REQ)
          {
                 len = trout_get_asoc_req_wps_ie_len();
                 ptr = trout_get_asoc_req_wps_ie_addr();
                 trout_set_wps_sec_type_flag(1);
                 printk("[wjb] need prepare_asoc_req len %d\n", len);
                 memcpy(data+index, ptr, len);
                 ret_val = len;
                 //trout_clear_asoc_req_wps_ie();
          }
#endif /* INT_WPS_SUPP */

    return ret_val;
}

/* This function sets security IE as required for association request in WPS */
INLINE UWORD8 set_asoc_req_sec_ie_sta(UWORD8 *data, UWORD16 index, UWORD8 supp)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
        return 0;
#endif /* INT_WPS_SUPP */

    return set_sec_info_element_sta(data, index, supp);
}

/* This function handles MLME response for WPS STA */
INLINE BOOL_T handle_mlme_rsp_prot(UWORD8 rsp_type, UWORD8 *rsp_msg)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        handle_mlme_rsp_wps_sta(rsp_type, rsp_msg);
        return BTRUE;
    }
#endif /* INT_WPS_SUPP */

    return BFALSE;
}

/* This function processes WPS events in STA mode */
INLINE BOOL_T process_wps_event_sta(UWORD32 event)
{
#ifdef INT_WPS_SUPP
    return handle_wps_event_sta(event);
#else /* INT_WPS_SUPP */
    return BFALSE;
#endif /* INT_WPS_SUPP */
}

/* This function processes scan response frames in WPS mode */
INLINE BOOL_T process_wps_scan_response(UWORD8 *mac, UWORD8 *msa,
                                        UWORD16 rx_len, WORD8 rssi)
{

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		/* Check if the device sending the probe response or beacon frame  */
	    /* should be added to the scan response list                       */
		if((BFALSE == is_p2p_grp_form_in_prog()) &&
		   (BFALSE == handle_wps_scan_rsp_sta_prot(msa, rx_len)))
			return BFALSE;
	}
#endif /* MAC_P2P */

#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        handle_wps_scan_rsp_sta(mac, msa, rx_len, rssi);
        return BTRUE;
    }
#endif /* INT_WPS_SUPP */

    return BFALSE;
}

/* This function processes WPS scan complete event in STA mode */
INLINE BOOL_T process_wps_scan_complete(UWORD8 *mac)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        handle_wps_scan_complete_sta(mac);
        return BTRUE;
    }
#endif /* INT_WPS_SUPP */

    return BFALSE;
}

/* Start a rejoin timer if not running */
INLINE void start_rejoin_timer_sta(void)
{
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED != get_wps_mode())
    {
        start_wps_cred_rejoin_timer();
    }
#endif /* INT_WPS_SUPP */
}

/* Initiate a Join with configuration saved in the device */
INLINE void join_config_saved_in_device_sta(void)
{
#ifdef INT_WPS_SUPP
    if(WPS_STANDALONE_DEVICE == get_wps_dev_mode_enr())
    {
        join_wps_cred_from_eeprom();
    }
#endif /* INT_WPS_SUPP */
}

/* Disable protocols in progress for STA mode */
INLINE BOOL_T allow_sys_restart_prot_sta(ERROR_CODE_T sys_error)
{
    BOOL_T ret_val = BTRUE;
#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        ret_val = wps_allow_sys_restart_sta(sys_error);
    }
#endif /* INT_WPS_SUPP */
    return ret_val;
}

/* Check if WPS protocol is enabled for the STA */
INLINE BOOL_T is_wps_prot_enabled(void)
{
    BOOL_T ret_val = BFALSE;

#ifdef INT_WPS_SUPP
    if(WPS_ENABLED == get_wps_mode())
    {
        ret_val = BTRUE;
    }
#endif /* INT_WPS_SUPP */

    return ret_val;
}

/* This function gets the WPS Protocol type configured */
INLINE UWORD8 get_wps_prot_sta(void)
{
    UWORD8 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_prot_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function sets the WPS Protocol type */
INLINE void set_wps_prot_sta(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        BOOL_T b_temp = g_config_write_from_host;
        set_wps_prot_enr(val);

#ifdef MAC_P2P
        /* If P2P Group formation is in progress, do not start WPS now. It   */
        /* will be automatically started once Group formation is complete.   */
        if(BFALSE == is_p2p_grp_form_in_prog())
#endif /* MAC_P2P */
        {
			/* Start WPS protocol (PBC or PIN) */
			sys_start_wps((WPS_PROT_TYPE_T)val);
		}

        g_config_write_from_host = b_temp;
    }
#endif /* INT_WPS_SUPP */
}

/* This function return the WPS Password ID programmed */
INLINE UWORD16 get_wps_pass_id_sta(void)
{
    UWORD16 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_pass_id_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_pass_id_sta(UWORD16 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_pass_id_enr(val);
    }
#endif /* INT_WPS_SUPP */
}

INLINE UWORD16 get_wps_config_method_sta(void)
{
    UWORD16 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_config_method_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_config_method_sta(UWORD16 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_config_method_enr(val);
    }
#endif /* INT_WPS_SUPP */
}

INLINE UWORD8 *get_wps_pin_sta(void)
{
    UWORD8 *retval = NULL;

#ifdef INT_WPS_SUPP
    retval = get_wps_pin_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_pin_sta(UWORD8 *val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_pin_enr(val);
    }
#endif /* INT_WPS_SUPP */
}

/* Functions specific to internal Enrollee */
INLINE UWORD8 get_wps_dev_mode_sta(void)
{
    UWORD8 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_dev_mode_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

#if 0 /* Not Used */
INLINE UWORD8 get_mac_cred_wps_status_sta(void)
{
    UWORD8 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_mac_cred_wps_status_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}
#endif /* 0 */

INLINE void set_wps_dev_mode_sta(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_dev_mode_enr(val);
    }
#endif /* INT_WPS_SUPP */
}

INLINE void set_wps_cred_list_sta(UWORD8 *val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_cred_list_enr(val);
    }
#endif /* INT_WPS_SUPP */
}

/* Function to get AC parameter values for STA side*/
INLINE UWORD8* get_wps_cred_list_sta(void)
{
    UWORD8 *retval = NULL;

#ifdef INT_WPS_SUPP
    retval = get_wps_cred_list_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* Check the device mode */
INLINE BOOL_T is_dev_mode_host_managed_sta(void)
{
    BOOL_T retval = BFALSE;

#ifdef INT_WPS_SUPP
    retval = is_dev_mode_host_managed_enr();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function handles update to WPS Credential by the user */
INLINE void handle_wps_cred_update_sta(void)
{
#ifdef INT_WPS_SUPP
    /* Disable credential read from WPS credential list, as user has directly*/
    /* programmed a credential value                                         */
    handle_host_non_wps_cred_req();
#endif /* INT_WPS_SUPP */
}

/* Indicate that WID Config Write Status to Protocol */
INLINE void indicate_config_write_prot_sta(BOOL_T val)
{
#ifdef INT_WPS_SUPP
    wps_set_config_write_sta(val);
#endif /* INT_WPS_SUPP */
}

/* This function sets the WPS Enrolle Mode to Enabled/Disabled */
INLINE void set_wps_enable_sta(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    if(val == 0)
    {
        disable_wps_mode();
        set_wps_mode(val);
    }
#endif /* INT_WPS_SUPP */
}

/* This function returns the WPS Enabled status */
INLINE BOOL_T get_wps_enable_sta(void)
{
    BOOL_T retval = BFALSE;

#ifdef INT_WPS_SUPP
    retval = (BOOL_T)get_wps_mode();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function calls the function to add P2P IE in assoc req frame */
INLINE UWORD16 set_asoc_req_p2p_ie(UWORD8 *data, UWORD16 index)
{
    UWORD16 retval = 0;
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        retval = add_asoc_req_p2p_ie(data, index);
    }
#endif /* MAC_P2P */
    return retval;

}

/* If P2P is enabled the persistent credential list is stored */
INLINE void update_persist_cred_list_prot_sta(void)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        if(BTRUE == mget_p2p_persist_grp())
        {
            p2p_update_cred_list();
        }
        mac_addr_cpy(g_p2p_go_dev_id, mget_p2p_trgt_dev_id());
    }
#endif /* MAC_P2P */
}

/* If P2P is enabled, ESS and IBSS bits should not be set in Listen state */
INLINE UWORD8 get_sta_bss_type_prot(void)
{
	UWORD8 ret_val = 0;

    /* The ESS  bit is set by STAs in BSS mode in Association Request Frames */
    /* for interoperability with some APs. The IBSS bit is only set by STAs  */
    /* in IBSS mode in Beacon and Probe Response Frames.                     */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
        ret_val = ESS;
    else
        ret_val = IBSS;

#ifdef MAC_P2P
    if((BTRUE == is_p2p_grp_form_in_prog()) &&
       (WAIT_SCAN == get_mac_state()) &&
       (LISTEN_STATE == get_p2p_scan_state()))
       ret_val = 0;
#endif /* MAC_P2P */

    return ret_val;
}

/* This function handles WPS settings conflicts when P2P group formation is  */
/* in progress.                                                              */
INLINE void handle_wps_p2p_grp_form_in_prog(void)
{
#ifdef INT_WPS_SUPP
#ifdef MAC_P2P
    UWORD8 val = get_wps_prot_enr();

    /* If P2P is enabled and P2P group formation is in progress, WPS should  */
	/* not be started now. It shall be started automatically once P2P group  */
	/* formation ends. If P2P group formation is in progress, disable WPS    */
	/* mode now.                                                             */
	set_wps_mode(WPS_DISABLED);
	free_wps_handle_mem();
	if(SUCCESS == allocate_wps_handle_mem())
	{
		wps_init_enrollee(g_wps_enrollee_ptr, g_wps_config_ptr);
		prepare_wps_config(g_wps_config_ptr, WPS_PROT_NONE);
		wps_gen_uuid(g_wps_config_ptr->mac_address, g_wps_enrollee_ptr->enr_uuid,
                 g_wps_enrollee_ptr->pke);
	}

	/* If P2P group formation is in progress and the current P2P_JOIN_STATE  */
	/* is P2P_WAIT_WPS_START, and the WID_WPS_START is set by the user to    */
	/* PIN or PBC, call the function p2p_wait_go_neg_rsp (which will send GO */
	/* Negotiation Request and change state to P2P_WAIT_GO_NEG_RSP)          */
    if(((val == PIN) || (val == PBC)) &&
	   (get_mac_state() == WAIT_JOIN) &&
	   (get_p2p_join_state() == P2P_WAIT_WPS_START))
	{
		p2p_wait_go_neg_rsp();
	}
#endif /* MAC_P2P */
#endif /* INT_WPS_SUPP */
}

/* This function initializes required Random variables based on Protocol in  */
/* use. Note that this needs to be called after MAC HW is initialized since  */
/* get_random_byte is used for these. Otherwise this function may hang.      */
INLINE void init_prot_rand_sta(void)
{
#ifdef MAC_P2P
	if(BFALSE == is_switch_in_progress())
	{
		gen_p2p_ssid();
		g_GO_tie_breaker = get_random_byte() & 1;
		gen_random_pass_phrase();// 20120709 caisf add, merged ittiam mac v1.2 code
	}
#endif /* MAC_P2P */
}

/* If P2P is not enabled, disable P2P discovery and restart as a non-P2P STA */
/* and if P2P is enabled, start P2P discovery.                               */
INLINE void set_p2p_enable_sta(UWORD8 p2p_en)
{
#ifdef MAC_P2P
    mset_p2p_enable((BOOL_T)p2p_en);

    if(BFALSE == mget_p2p_enable())
    {
        set_p2p_grp_form_in_prog(BFALSE);
    }
    else if(BFALSE == g_reset_mac_in_progress)
    {
        set_p2p_grp_form_in_prog(BTRUE);
        init_prot_rand_sta();
    }
#endif /* MAC_P2P */
}


/* This function sets Device discoverability for STA mode */
INLINE void set_p2p_discoverable_sta(BOOL_T val)
{
#ifdef MAC_P2P
    /* This can be set in Client mode only if group formation is in progress */
    /* or the device is in DISABLED state                                    */
    if((BTRUE == is_p2p_grp_form_in_prog()) || (DISABLED == get_mac_state()))
        mset_p2p_discoverable(val);
#endif /* MAC_P2P */
}

/* This function sets Persistent Capability for STA mode */
INLINE void set_p2p_persist_grp_sta(BOOL_T val)
{
#ifdef MAC_P2P
	/* This can be set in Client mode only if group formation is in progress */
    /* or the device is in DISABLED state                                    */
    if((BTRUE == is_p2p_grp_form_in_prog()) || (DISABLED == get_mac_state()))
        mset_p2p_persist_grp(val);
#endif /* MAC_P2P */
}

/* This function sets Intra-BSS Capability for STA mode */
INLINE void set_p2p_intra_bss_sta(BOOL_T val)
{
#ifdef MAC_P2P
	/* This can be set in Client mode only if group formation is in progress */
    /* or the device is in DISABLED state                                    */
    if((BTRUE == is_p2p_grp_form_in_prog()) || (DISABLED == get_mac_state()))
        mset_p2p_intra_bss(val);
#endif /* MAC_P2P */
}

/* This function restores default settings as required before mode switch,   */
/* i.e. STA mode to AP mode.                                                 */
INLINE void restore_default_mode_wids_sta(void)
{
#ifdef MAC_P2P
    /* Switching to AP mode. Auto GO should be always BTRUE to prevent the   */
    /* AP from switching back to STA mode due to this setting.               */
    /* Note that this is required since the Auto GO mode may not be BTRUE in */
    /* case the STA is switching to AP mode as a result of group negotiation */
    /* and not Auto GO enable configuration.                                 */
    mset_p2p_auto_go(BTRUE); /* Switching to AP Mode: GO = BTRUE */
#endif /* MAC_P2P */
}

/* This function tries to send a GO Negotiation Request and change P2P WAIT  */
/* JOIN sub state to P2P_WAIT_GO_NEG_RSP. If WPS has not been started yet by */
/* the user i.e. the value is not PIN or PBC, change to P2P_WAIT_WPS_START   */
/* instead.                                                                  */
INLINE void try_p2p_wait_go_neg_rsp(void)
{
#ifdef MAC_P2P
#ifdef INT_WPS_SUPP
	if((get_wps_prot_enr() != PIN) && (get_wps_prot_enr() != PBC))
	{
		p2p_wait_wps_start();
		return;
	}
#endif /* INT_WPS_SUPP */

    p2p_wait_go_neg_rsp();
#endif /* MAC_P2P */
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE BOOL_T is_frame_valid(UWORD8* msa)
{
#ifdef MAC_P2P
    /* Ignore the validity check during P2P group formation */
    if((BFALSE == is_p2p_grp_form_in_prog()) && (SEND_INVITE != g_p2p_invit))
#endif /* MAC_P2P */
    {
        BSSTYPE_T bss_type = INDEPENDENT;
        UWORD16   cap_info = 0;

        /* Extract the BSS type from the capability information from the     */
        /* received frame. If it is not INDEPENDENT or INFRASTRUCTURE, then  */
        /* do not process this frame further. Issue was seen during P2P      */
        /* inter-op testing when test bed device was setting cap_info = 0    */
        /* incorrectly. Such frames need to be ignored.                      */
        cap_info = get_cap_info(msa);
        bss_type = get_bss_type(cap_info);

        /* Check if the bss type is valid */
        if(is_bss_type_valid(bss_type) == BFALSE)
            return BFALSE;
    }

    return BTRUE;
}


#endif /* STA_PROT_IF_H */
#endif /* IBSS_BSS_STATION_MODE */
