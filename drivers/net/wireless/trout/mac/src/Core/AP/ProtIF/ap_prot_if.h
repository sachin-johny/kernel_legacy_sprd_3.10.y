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
/*  File Name         : ap_prot_if.h                                         */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AP mode.                                             */
/*                                                                           */
/*  List of Functions : Protocol interface functions for AP mode             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef AP_PROT_IF_H
#define AP_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_ap.h"
#include "itypes.h"
#include "common.h"
#include "prot_if.h"
#include "wep_ap.h"
#include "phy_hw_if.h"
#include "ch_info.h"

#ifdef MAC_802_11I
#include "rsna_auth_km.h"
#include "auth_frame_11i.h"
#include "ieee_11i_ap.h"
#include "ieee_11i_auth.h"
#endif /* MAC_802_11I */

#ifdef MAC_WMM
#include "ap_frame_11e.h"
#include "mib_11e.h"
#include "ap_management_11e.h"
#include "ap_uapsd.h"
#endif /* MAC_WMM */

#ifdef MAC_802_11H
#include "mib_11h.h"
#include "dfs_ap.h"
#include "management_11h.h"
#endif /* MAC_802_11H */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
#include "frame_mdom.h"
#include "mib_mdom.h"
#include "ap_management_mdom.h"
#endif /* MAC_MULTIDOMAIN */

#ifdef MAC_802_11N
#include "management_11n.h"
#include "ap_frame_11n.h"
#include "ap_management_11n.h"
#endif /* MAC_802_11N */

#ifdef INT_WPS_SUPP
#include "wps_ap.h"
#include "wps_cred_mgmt_ap.h"
#include "ap_management_wps.h"
#endif /* INT_WPS_SUPP */

#ifdef MAC_P2P
#include "p2p_wlan_rx_mgmt_ap.h"
#include "ap_frame_p2p.h"
#include "ap_mgmt_p2p.h"
#endif /* MAC_P2P */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern UWORD32  g_num_ps_pkt;

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function returns the WPS Capability of the STA */
INLINE BOOL_T is_sta_wps_cap_ap(asoc_entry_t *ae)
{
    BOOL_T retval = BFALSE;

#ifdef INT_WPS_SUPP
    retval = ae->wps_cap_sta;
#endif /* INT_WPS_SUPP */

    return retval;
}

/*****************************************************************************/
/* 802.11i Related Prot-ifcation functions                                   */
/*****************************************************************************/

/* Check sec protocol dependent capability information fields */
INLINE BOOL_T check_bss_mac_privacy_ap(UWORD16 cap_info)
{
#ifdef MAC_802_11I
    /* If RSNA is enabled, ignore the privacy bit */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        return BTRUE;
    }
#endif /* MAC_802_11I */

#ifdef INT_WPS_SUPP
    /* In case of WPS Mode, do not check for Privacy bit. This was done as to*/
    /* support Ittiam Enrollee that does not sets the Privacy Bit based on   */
    /* AP`s Privacy bit                                                      */
    if(BFALSE == get_wps_reg_enabled())
#endif /* INT_WPS_SUPP */
    {
        if(mget_PrivacyInvoked() == TV_FALSE)
        {
            /* This STA doesn't have Privacy invoked but the other STA does */
            if((cap_info & PRIVACY) == PRIVACY)
            {
                return BFALSE;
            }
        }
        else /* mget_PrivacyInvoked() == TV_TRUE */
        {
            /* This STA has Privacy invoked but the other STA doesn't */
            if((cap_info & PRIVACY) != PRIVACY)
            {
                return BFALSE;
            }
        }
    }

    return BTRUE;
}

/* This function sets the 802.11I parameter element in the AP mode beacons   */
/* protocols in use.                                                         */
INLINE UWORD8 set_sec_info_element_ap(UWORD8 *data, UWORD16 index)
{
    UWORD16 len = 0;

#ifdef MAC_802_11I
    if(g_mode_802_11i & RSNA_802_11I)
    {
        len += set_rsn_element_auth(data, index, RSNA_802_11I);
    }
    if(g_mode_802_11i & WPA_802_11I)
    {
        len += set_rsn_element_auth(data, index + len, WPA_802_11I);
    }

#endif /* MAC_802_11I */

    return len;
}

/* Filter the frame received from the WLAN based on the security policy */
INLINE BOOL_T filter_wlan_rx_sec_ap(wlan_rx_t *wlan_rx)
{
    /* If the STA is WPS Capable then do not do any Security related         */
    /* filtering                                                             */
    if(BTRUE == is_sta_wps_cap_ap((asoc_entry_t *)wlan_rx->sa_entry))
        return BFALSE;

#ifdef MAC_802_11I
    if(filter_wlan_rx_11i_auth_ap(wlan_rx) == BTRUE)
        return BTRUE;
#endif /* MAC_802_11I */

    return filter_wlan_rx_wep_ap((CIPHER_T)wlan_rx->ct, wlan_rx->sub_type);
}

/* Filter the frame received from the Host based on the security policy */
INLINE BOOL_T filter_host_rx_sec_ap(CIPHER_T *ct, UWORD8 **info,
            UWORD8 key_type, asoc_entry_t *asoc_entry, BOOL_T ignore_port)
{
#ifdef MAC_802_11I
    if(filter_host_rx_11i_auth_ap(ct, info, key_type, asoc_entry, ignore_port)
       == BTRUE)
        return BTRUE;
#endif /* MAC_802_11I */

    if(ignore_port == BFALSE)
        return filter_host_rx_wep_ap(ct);

    return BFALSE;
}

/* Delete the security entry by deleting LUT and stopping the FSMs */
INLINE void delete_sec_entry_ap(asoc_entry_t *ae)
{
#ifdef MAC_802_11I
    delete_11i_entry_auth_ap(ae);
#endif /* MAC_802_11I */

    delete_wep_entry_ap(ae);
}

/* Check if the Shared Key authentication is permitted */
INLINE BOOL_T is_wep_allowed(void)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
        return is_wep_allowed_11i();
#endif /* MAC_802_11I */

    return is_wep_enabled();
}


/* Check the security capabilities before accepting the association request */
INLINE UWORD16 check_sec_capabilities_ap(asoc_entry_t *ae, UWORD8 *msa,
                                         UWORD16 rx_len)
{
    UWORD16 retval = SUCCESSFUL_STATUSCODE;

#ifdef MAC_802_11I
#ifdef INT_WPS_SUPP
    WPS_ASSOC_CHECK_T wps_check = check_wps_capabilities_ap(ae, msa, rx_len);

    /* Check if the STA is WPS Capable. If it WPS Capable and the registrar */
    /* is not busy then return successful status code else return           */
    /* UNSPEC_FAIL. If the STA is not WPS Capable then further security     */
    /* related checks are performed                                         */
    if(WPS_CHECK_FAILURE == wps_check)
    {
        retval = UNSPEC_FAIL;
    }
    else if(WPS_CHECK_FURTHER == wps_check)
#endif /* INT_WPS_SUPP */
    {
        /* In a RSNA Enabled Network, RSNA IE is present. This field is  */
        /* present at the end of the received frame                      */
        /* Capabilities in this element also need to be validated before */
        /* accepting the association request                             */
        retval = check_rsn_capabilities_ap(ae, msa, rx_len);
    }
#endif /* MAC_802_11I */

    return retval;
}

/* Initialize the security handle to the default parameters */
INLINE UWORD16 init_sec_entry_ap(asoc_entry_t *ae, UWORD8 *sa, UWORD8 *msa,
                                 UWORD16 rx_len)
{
#ifdef MAC_802_11I
    UWORD16 ret_val = SUCCESSFUL_STATUSCODE;
#ifdef INT_WPS_SUPP
    /* For WPS Capable STA, do not do any security related initialization */
    /* and reset the RSNA PerSTA Pointer                                     */
    if(BTRUE == is_sta_wps_cap_ap(ae))
        init_11i_handle_auth_ap(ae);
    else
#endif /* INT_WPS_SUPP */
        ret_val     = init_11i_entry_ap(ae, sa, msa, rx_len);

    if(ret_val != SUCCESSFUL_STATUSCODE)
        return ret_val;
#endif /* MAC_802_11I */

    if(is_wep_enabled() == BTRUE)
    {
        /* Add WEP Entry for the STA */
        /* Using Default KeyId & Sta Index generated while AUTH_REQ */
        add_wep_entry(ae->sta_index, mget_WEPDefaultKeyID(), sa);
    }

    g_decr_fail_cnt[ae->sta_index] = 0;

    return SUCCESSFUL_STATUSCODE;
}

/* Start the security FSM. This is done after transmission of a */
/* successful Association Response                              */
INLINE void start_sec_fsm_ap(asoc_entry_t *ae)
{
#ifdef MAC_802_11I
    start_11i_fsm_auth_ap(ae);
#endif /* MAC_802_11I */
}

/* For all the incoming MSDUs, the security check is performed so as to */
/* Before forwarding the packet across the WLAN-WLAN interface */
/* security checks needs to performed on the states of the     */
/* transmitting and receiving stations                         */
INLINE BOOL_T check_sec_ucast_wlan_2_wlan_ap(asoc_entry_t *da_ae,
            asoc_entry_t *sa_ae, CIPHER_T *ct, UWORD8 *data_trailer)
{
#ifdef MAC_802_11I
    BOOL_T ret_val = BFALSE;

    ret_val = check_11i_ucast_wlan_2_wlan_ap(da_ae, sa_ae, ct,
                                             data_trailer);
    if(ret_val != BTRUE)
        return ret_val;
#endif /* MAC_802_11I */
    return check_wep_ucast_wlan_2_wlan_ap(ct);
}

/* Before forwarding the packet across the WLAN-WLAN interface */
/* security checks needs to performed on the states of the     */
/* transmitting station                                        */
INLINE UWORD8 check_sec_bcast_wlan_2_wlan_ap(CIPHER_T *ct)
{
    UWORD8 ret_val = 0;

#ifdef MAC_802_11I
    ret_val = check_11i_bcast_wlan_2_wlan_ap(ct);
#endif /* MAC_802_11I */

    check_wep_bcast_wlan_2_wlan_ap(ct);

    return ret_val;
}

/* If the received packet is a security handshake packet, process the */
/* packet in the security FSM                                         */
INLINE BOOL_T is_sec_handshake_pkt_ap(asoc_entry_t *ae,
                                      UWORD8 *buffer, UWORD8 offset,
                                      UWORD16 rx_len, CIPHER_T ct)
{
    BOOL_T retval = BFALSE;
#ifdef MAC_802_11I
#ifdef INT_WPS_SUPP
    if(SUCCESS == wps_handle_eap_rx_ap(ae, (buffer + offset), rx_len))
        retval = BTRUE;
    else
#endif /* INT_WPS_SUPP */
        retval = is_11i_handshake_pkt_ap(ae, buffer, offset, rx_len, ct);

#endif /* MAC_802_11I */

    return retval;
}

/* Before forwarding the packet across the WLAN-WLAN/HOST interface */
/* security checks needs to performed on the states of the          */
/* transmitting station                                             */
INLINE BOOL_T ap_check_sec_tx_sta_state(asoc_entry_t *ae)
{
#ifdef MAC_802_11I
    return ap_check_11i_tx_sta_state(ae);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}

/* Initialize security related FSMs */
INLINE void intialize_sec_km_ap(void)
{
#ifdef MAC_802_11I
    initialize_auth_rsna();
#endif /* MAC_802_11I */
}

/* Stop security related FSMs */
INLINE void stop_sec_km_ap(void)
{
#ifdef MAC_802_11I
    stop_auth_rsna();
#endif /* MAC_802_11I */
}

/* If 802.11i and 802.1x are enabled, initialize 802.1x related security */
INLINE void init_sec_auth_ap(void)
{
#ifdef MAC_802_11I
    init_sec_auth_ap_1x();
#endif /* MAC_802_11I */
}

/* Handle the authentication/arp packets */
INLINE BOOL_T check_sec_auth_pkt_ap(UWORD8 *buffer, UWORD32 len)
{
#ifdef MAC_802_11I
    return check_sec_auth_pkt_ap_1x(buffer, len);
#else /* MAC_802_11I */
    return BFALSE;
#endif /* MAC_802_11I */
}

INLINE BOOL_T init_sec_auth_node_ap(void **auth_1x, UWORD8 *rsna, UWORD16 aid,
                                    void *auth_1x_buff)
{
#ifdef MAC_802_11I
    return init_sec_auth_node_ap_1x(auth_1x, rsna, aid, auth_1x_buff);
#else /* MAC_802_11I */
    return BFALSE;
#endif /* MAC_802_11I */
}

INLINE void free_sec_auth_node_ap(void **auth_1x)
{
#ifdef MAC_802_11I
    free_sec_auth_node_ap_1x(auth_1x);
#endif /* MAC_802_11I */
}

/* Initialize 802.1x related security */
INLINE void stop_sec_auth_node_ap(void)
{
#ifdef MAC_802_11I
    stop_sec_auth_node_ap_1x();
#endif /* MAC_802_11I */
}

/* Start Sec Daemon for AP Mode */
INLINE void start_sec_daemon_ap(void)
{
#ifdef MAC_802_11I
    start_sec_daemon_ap_1x();
#endif /* MAC_802_11I */
}

/* Stop Sec Daemon for AP Mode */
INLINE void stop_sec_daemon_ap(void)
{
#ifdef MAC_802_11I
    stop_sec_daemon_ap_1x();
#endif /* MAC_802_11I */
}

/* This function sets the Radius Key used to communicate with radius server */
INLINE void set_1x_key_ap(UWORD8* val)
{
#ifdef MAC_802_11I
    set_1x_radius_key_ap(val);
#endif /* MAC_802_11I */
}

/* This function gets the Radius Key used to communicate with radius server */
INLINE UWORD8* get_1x_key_ap(void)
{
#ifdef MAC_802_11I
    return get_1x_radius_key_ap();
#else /* MAC_802_11I */
    return NULL;
#endif /* MAC_802_11I */
}

/* This function sets the Radius Server Address */
INLINE void set_1x_serv_addr_ap(UWORD32 val)
{
#ifdef MAC_802_11I
    set_1x_radius_serv_addr_ap(val);
#endif /* MAC_802_11I */
}

/* This function sets the Radius Server Address */
INLINE UWORD32 get_1x_serv_addr_ap(void)
{
#ifdef MAC_802_11I
    return get_1x_radius_serv_addr_ap();
#else /* MAC_802_11I */
    return 0;
#endif /* MAC_802_11I */
}

INLINE void compute_install_sec_key_ap(void)
{
#ifdef MAC_802_11I
    /* If PSK is enabled install the PSK/PMK */
    if(g_psk_available == BTRUE)
        install_psk();
#endif /* MAC_802_11I */
}

INLINE void update_sec_fail_stats_ap(CIPHER_T ct, UWORD8 status, UWORD8 *sa)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        update_11i_fail_stats_ap(ct, status, sa);
    }
#endif /* MAC_802_11I */
}

/* This function updates the security statistics on a successful reception */
INLINE void update_sec_success_stats_ap(asoc_entry_t *ae, CIPHER_T ct,
                                        UWORD8 status)
{
#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        update_11i_success_stats_ap(ae, ct, status);
    }
#endif /* MAC_802_11I */
}
// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/*****************************************************************************/
/*              Multi-Domain Related elements                                */
/*****************************************************************************/
/* This function checks for any regulatory class that is available for given */
/* combination of channel and secondary channel offset                       */
INLINE BOOL_T is_reg_class_available(UWORD8 ch_idx, UWORD8 sec_offset)
{
    BOOL_T ret_val = BTRUE;
#ifdef MAC_MULTIDOMAIN
        if(INVALID_REG_CLASS == get_rc_info_sec_chan_offset(ch_idx, sec_offset))
    {
            ret_val = BFALSE;
    }
#endif /* MAC_MULTIDOMAIN */
    return ret_val;
}

/* Select suitable Regulatory Class for the combination of channel  and      */
/* offset, Secondary channel offset is set to SCN If regulatory class is not */
/* found for the given combination of Channel and Secondary Ch off set       */
INLINE UWORD8 select_reg_class(UWORD8 chan_idx, UWORD8 sec_offset)
{
    UWORD8 ret_val = sec_offset;

#ifdef MAC_MULTIDOMAIN
    ret_val = select_suitable_reg_class(chan_idx, sec_offset);
#endif  /* MAC_MULTIDOMAIN */

    return ret_val;
}
#endif

/*****************************************************************************/
/* WMM Related Prot-ifcation functions                                       */
/*****************************************************************************/

/* Set the QoS control field */
INLINE void set_qos_control_field_ap(UWORD8 *header, UWORD8 tid, UWORD8 ap)
{
#ifdef MAC_WMM
    set_qos_control_ap(header, tid, ap);
#endif /* MAC_WMM */
}

/* Set the WMM parameters field for beacon frames */
INLINE UWORD16 set_wmm_beacon_params(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return set_wmm_param_element_ap(data, index);
#endif /* MAC_WMM */

    return 0;
}

/* Set the WMM parameters field for probe response frames */
INLINE UWORD16 set_wmm_probe_rsp_params(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return set_wmm_param_element_ap(data, index);
#endif /* MAC_WMM */

    return 0;
}

/* Set the WMM parameters field for association response frames only if the  */
/* association request frame contains a WMM information element.             */
INLINE UWORD16 set_wmm_asoc_rsp_params(UWORD8 *data, UWORD16 index,
                                       UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8 ret_len = 0;

#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        UWORD16 offset = MAC_HDR_LEN + CAP_INFO_LEN + LISTEN_INT_LEN;
		// 20120709 caisf add, merged ittiam mac v1.2 code
        /* If the frame is Re-association request, add 6 bytes to the offset */
        /* since this frame includes an extra field (Current AP Address)     */
        if(get_sub_type(msa) == REASSOC_REQ)
        {
			offset += MAC_ADDRESS_LEN;
		}

        while(offset < (rx_len - FCS_LEN))
        {
            if(is_wmm_info_elem(msa + offset) == BTRUE)
            {
                ret_len = set_wmm_param_element_ap(data, index);
                break;
            }

            offset += (2 + msa[offset + 1]);
        }
    }
#endif /* MAC_WMM */

    return ret_len;
}

/* This function configures the AC parameters of a AP */
INLINE void set_ap_ac_params_prot_ap(UWORD8 *val)
{
#ifdef MAC_WMM
   if(get_wmm_enabled() == BTRUE)
   {
       set_wmm_ap_ac_params_config(val);
   }
#endif /* MAC_WMM */
}

/* This function gets the current AC parameters configured at the AP */
INLINE UWORD8* get_ap_ac_params_prot_ap(void)
{
#ifdef MAC_WMM
   if(get_wmm_enabled() == BTRUE)
   {
       return (UWORD8 *)get_wmm_ap_ac_params_config();
   }
#endif /* MAC_WMM */
    return 0;
}

/* This function configures the AC parameters of a STA */
INLINE void set_sta_ac_params_prot_ap(UWORD8 *val)
{
#ifdef MAC_WMM
   if(get_wmm_enabled() == BTRUE)
   {
       set_wmm_sta_ac_params_config(val);
   }
#endif /* MAC_WMM */
}

/* Get the queue number for WLAN to WLAN transmission based on QoS setting.  */
/* For WMM add to the AC mapping to the TID in the received frame. If no     */
/* QoS is present add to default AC_VO queue. In case of no WMM add to the   */
/* default NORMAL_PRIORITY queue.                                            */
INLINE UWORD8 get_w2w_txq_num_ap(UWORD8 priority)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        return get_txq_num(priority);
    }

    return AC_VO_Q;
#else /* MAC_WMM */
    return NORMAL_PRI_Q;
#endif /* MAC_WMM */
}

/* return if the pkt is for a delivery enabled queue */
INLINE BOOL_T check_ac_is_del_en_prot(asoc_entry_t *ae, UWORD8 priority)
{
    BOOL_T ret_val = BFALSE;
#ifdef MAC_WMM
    ret_val = check_ac_is_del_en(ae, priority);
#endif /* MAC_WMM */
    return ret_val;
}

/* Returns the appropriate PS queue pointer for the given packet descriptor */
INLINE void *get_ps_q_ptr(asoc_entry_t *ae, BOOL_T ps_del_en_ac)
{
#ifdef MAC_WMM
    void *qh            = NULL;

    if(BTRUE == ps_del_en_ac)
        qh = &(ae->ps_q_del_ac);
    else
        qh = &(ae->ps_q_lgcy);

    return qh;
#else /* MAC_WMM */
    return (&(ae->ps_q_lgcy));
#endif /* MAC_WMM */
}

/* Returns the legacy PS queue pointer by default, if its NULL then WMM-PS Q pointer */
INLINE void *get_any_ps_q_ptr(asoc_entry_t *ae)
{
    list_buff_t *qh = &(ae->ps_q_lgcy);

#ifdef MAC_WMM
    if((qh == NULL) ||
       (qh->head == 0))
    {
        qh = &(ae->ps_q_del_ac);
    }
#endif /* MAC_WMM */

    return qh;
}

/* Check if any AC for the associated STA is delivery enabled */
INLINE BOOL_T check_legacy_flag_prot(asoc_entry_t *ae)
{
#ifdef MAC_WMM
    return is_all_ac_del(ae);
#else /* MAC_WMM */
    return BFALSE;
#endif /* MAC_WMM */
}

/* Update the appropriate PS queue counter after adding packets to Legacy or */
/* Delivery enabled and PS queue. Returns BTRUE if TIM needs to be updated   */
INLINE BOOL_T update_ps_counts(asoc_entry_t *ae, UWORD8 num_buff_added,
                                 BOOL_T ps_add_del_ac)
{
    BOOL_T retval = BFALSE;
#ifdef MAC_WMM
    /* Check if Legacy or Delivery enabled PS Queue */
    if(ps_add_del_ac == BFALSE)
    {
        ae->num_ps_pkt_lgcy += num_buff_added;
        /* Set the TIM bit for this station */
        retval = BTRUE;
    }
    else
    {
        ae->num_ps_pkt_del_ac += num_buff_added;
        /* Set the TIM bit for this station if all AC are delivery enabled */
        if(BTRUE == check_legacy_flag_prot(ae))
        {
            retval = BTRUE;
        }
    }
    g_num_ps_pkt   += num_buff_added;
#else /* MAC_WMM */
    ae->num_ps_pkt_lgcy += num_buff_added;
    g_num_ps_pkt   += num_buff_added;
    /* Set the TIM bit for this station */
    retval = BTRUE;
#endif /* MAC_WMM */

    return retval;
}


/* This function handles power management for delivery enabled ACs */
INLINE void pwr_mgt_handle_prot(asoc_entry_t *ae, UWORD8 ps, UWORD8 priority,
                                UWORD8 *msa)
{
#ifdef MAC_WMM
    if((ps == 1) && (is_ap_uapsd_capable() == BTRUE) &&
       (is_qos_required(ae->sta_index) == BTRUE))
            uapsd_handle_pwr_mgt(ae, priority, msa);
#endif /* MAC_WMM */
}

/* This function sets the end of service period bit */
INLINE void set_qos_prot(UWORD8 *msa)
{
#ifdef MAC_WMM
    set_eosp(msa);
#endif /* MAC_WMM */
}

INLINE UWORD8 set_frame_ctrl_qos_null_ap(UWORD8* mac_hdr, UWORD8 priority,
                                         UWORD8 ack_policy)
{
#ifdef MAC_WMM
    set_frame_control(mac_hdr, QOS_NULL_FRAME);
    set_qos_control_field_ap(mac_hdr,priority, ack_policy);
    set_eosp(mac_hdr);
    return QOS_MAC_HDR_LEN + FCS_LEN;
#else /* MAC_WMM */
    return MAC_HDR_LEN + FCS_LEN;
#endif /* MAC_WMM */
}

INLINE void set_frame_ctrl_prot(UWORD8 *mac_hdr, UWORD8 priority, UWORD8 ack_policy)
{
#ifdef MAC_WMM
    set_frame_control(mac_hdr, QOS_DATA);
    set_qos_control_ap(mac_hdr, priority, ack_policy);
#endif /* MAC_WMM */
}


/* update the appropriate PS queue counters after a Tx complete interrupt */
/* returns BTRUE if TIM needs to be reset                                 */
INLINE BOOL_T update_ps_counts_txcomp(asoc_entry_t *ae, UWORD8 *msa)
{
    BOOL_T retval = BFALSE;

    /* TBD: Device a method avoid all this processing in every Tx comp    */
    /* interrupt                                                          */
#ifdef MAC_WMM
    BOOL_T del_ena = BFALSE;
    if(BTRUE == is_qos_bit_set(msa))
    {
        UWORD8 priority = get_tid_value(msa);

        if(BTRUE == check_ac_is_del_en(ae, priority))
            del_ena = BTRUE;
    }

    if(BTRUE == del_ena)
    {
        /* Check the no. of packets queued for a USP, if an USP is in        */
        /* progress. Clear the flag if all packets queued in an USP are      */
        /* transmitted                                                       */
        /* This is done to ignore multiple trigger frames during an USP      */
        /* "USP_in_progress" flag is cleared when the STA wakes up           */

        if(ae->USP_in_progress == BTRUE)
        {
		// 20120709 caisf add the "if", merged ittiam mac v1.2 code
            if(ae->num_USP_pkts_qed)
            	ae->num_USP_pkts_qed--;

            if(ae->num_USP_pkts_qed == 0)
                ae->USP_in_progress = BFALSE;
        }
    }
    else
#endif /* MAC_WMM */
    {
        /* Non Delivery PS Q Frame are sent only on PS poll */
        /* Clear the Flag for PS poll response              */
        ae->ps_poll_rsp_qed = BFALSE;
    }

    if(ae->num_qd_pkt)
    {
        ae->num_qd_pkt--;
    }

#ifdef MAC_WMM
    if(is_all_ac_del(ae) == BTRUE)
    {
		// 20120709 caisf add, merged ittiam mac v1.2 code
        //if( (ae->num_ps_pkt_lgcy + ae->num_qd_pkt) == 0)
        if((ae->num_ps_pkt_del_ac + ae->num_qd_pkt) == 0)
            retval = BTRUE;
    }
    else
#endif /* MAC_WMM */
    {
        /* If no packets are queued in Sw PSQ or Hw for the */
        /* station, reset TIM bit                           */
        if( (ae->num_ps_pkt_lgcy + ae->num_qd_pkt) == 0)
        {
            retval = BTRUE;
        }
    }

    return retval;
}

/* This function handles the change of state of STA to active state */
INLINE REQUEUE_STATUS_T handle_ps_sta_change_to_active_prot(asoc_entry_t  *ae)
{
#ifdef MAC_WMM
    return uapsd_handle_ps_sta_change_to_active(ae);
#else /* MAC_WMM */
    return 0;
#endif /* MAC_WMM */
}

/* To check if there are any pending packets in apsd power save queue */
INLINE REQUEUE_STATUS_T check_pending_packet_prot(asoc_entry_t *ae)
{
#ifdef MAC_WMM
    return uapsd_handle_ps_sta_change_to_active(ae);
#else /* MAC_WMM */
    return 0;
#endif /* MAC_WMM */
}

INLINE BOOL_T is_sub_type_null_prot(UWORD16 frm_type)
{
#ifdef MAC_WMM
    if(frm_type == QOS_NULL_FRAME)
        return BTRUE;
#endif /* MAC_WMM */

    return BFALSE;
}

INLINE void flush_ps_queues(asoc_entry_t *ae)
{
    UWORD32 *tx_dscr = NULL;

    /* Free the elements in Legacy PS queue */
    while(NULL != (tx_dscr = remove_list_element_head(&(ae->ps_q_lgcy))))
    {
        free_tx_dscr(tx_dscr);
        g_num_ps_pkt--;
    }

    ae->num_ps_pkt_lgcy = 0;

#ifdef MAC_WMM
    /* Free the elements in Delivery enabled PS queue */
    while(NULL != (tx_dscr = remove_list_element_head(&(ae->ps_q_del_ac))))
    {
        /* Free the buffers associated with the transmit descriptor */
        free_tx_dscr(tx_dscr);
        g_num_ps_pkt--;
    }
    ae->num_ps_pkt_del_ac = 0;
    ae->USP_in_progress   = BFALSE;
#endif /* MAC_WMM */
}

/*****************************************************************************/
/* 802.11h Related Prot-ifcation functions                                   */
/*****************************************************************************/
/*  Processing       : This function sets all the SW parameters to 40 MHz    */
/*                     and calls enable** funtion to set HW to 40 MHz        */
INLINE void initiate_switch_to_40MHz_ap(UWORD8 sec_chan_off)
{
}

/* This funtion sets the appropriate channel in MAC SW/HW/RF */
INLINE void select_channel_mac(UWORD8 channel, UWORD8 sec_chan)
{
    /* Set the new channel in RF */
    select_channel_rf(channel, sec_chan);

#ifdef MAC_802_11N
#endif /* MAC_802_11N */
	{
		disable_2040_operation();
	}

    /* change the channel number in MAC MIB */
    mset_CurrentChannel(channel);
}

/*  Processing       : This function sets all the SW parameters to 20 MHz    */
/*                     sets a global so that HW switching to 20 MHz can      */
/*                     happen at the appropriate end of DTIM interval        */
INLINE void initiate_switch_to_20MHz_ap(void)
{
     /* disable 2040 if enabled */
	 select_channel_mac(mget_CurrentChannel(), 0);

	update_beacon_ht_info();
}
/* This function initializes globals for various protocols */
INLINE void init_prot_globals_ap(void)
{
#ifdef MAC_802_11H
    initialize_dfs_ap_globals();
	// 20120830 caisf masked, merged ittiam mac v1.3 code
    //set_default_country_info_element();
#endif /* MAC_802_11H */

#ifdef MAC_802_11N
    g_40MHz_intol_bit_recd    = BFALSE;
    g_num_sta_nonht_asoc      = 0;
    g_num_sta_20mhz_only      = 0;
    g_num_sta_ngf             = 0;
    g_num_sta_no_lsig_txop    = 0;
    g_obss_non_ht_sta_present = 0;
#endif /* MAC_802_11N */

#ifdef MAC_P2P
    g_p2p_GO_role             = BTRUE;
    mset_p2p_GO(BTRUE);
    mset_p2p_dev_lmt(BTRUE);
	// 20120709 caisf add, merged ittiam mac v1.2 code
    init_invi_scan_chan_list();
    init_p2p_globals();
#endif /* MAC_P2P */
}

/* This funtion checks if the current channel is available */
/* for doing scanning and starting network */
INLINE BOOL_T scan_availabilty(ap_ch_info_t channel_info)
{
#ifdef MAC_802_11H
    if(mget_enableDFS() == BFALSE)
    {
		return BTRUE;
	}

    if((channel_info.dfs_ch_status != BLOCKED_DUE_TO_RADAR) &&
       (channel_info.dfs_ch_status != NOT_SUPPORTED))
    {
        g_ap_dfs_radar_detected = BFALSE;
        return BTRUE;
    }
    else
    {
        return BFALSE;
    }

#else /* MAC_802_11H */
    return BTRUE;
#endif /* MAC_802_11H */
}

/* This funtion checks if 11h status of the said channel */
/* is blocked/available. If the status is not well known */
/* then the channel has to be re-scanned */
INLINE BOOL_T need_of_rescan(ap_ch_info_t channel_info)
{
#ifdef MAC_802_11H
    if(mget_enableDFS() == BFALSE)
    {
		return BFALSE;
	}

    if(channel_info.dfs_ch_status == AVAILABLE_TO_SCAN)
    {
        g_ap_dfs_radar_detected = BFALSE;
        return BTRUE;
    }
    else
    {
        return BFALSE;
    }

#else /* MAC_802_11H */
    return BFALSE;
#endif /* MAC_802_11H */
}

/* This funtion updates the status of a said channel interms  */
/* of radar or any other protocol related information         */
INLINE void update_chan_status_prot(UWORD8 ch_idx)
{
#ifdef MAC_802_11H
    update_channel_availability_status(ch_idx);
#endif /* MAC_802_11H */
}

/* This funtion returns the maximum number of scanning iterations */
/* needed by any protocol, returns 1 if no protocol has a choice */
INLINE UWORD8 get_num_scan_itr_prot(void)
{
    UWORD8 num_itr = 1;

#ifdef MAC_802_11H
    num_itr = MAX(num_itr, NUM_ITR_FOR_SCAN);
#endif /* MAC_802_11H */


   return num_itr;
}
/* This funtion returns the Channel scanning time per channel */
INLINE UWORD32 get_ch_scan_time_prot(void)
{
	UWORD32 scan_time = 0;

#ifdef MAC_802_11H
    if(mget_enableDFS() == BTRUE)
    {
     	scan_time = MAX(scan_time, CHANNEL_AVAILABILITY_CHECK_TIME);
	}
#endif /* MAC_802_11H */


    /* If Auto Channel selection is needed */
    if(BTRUE == is_auto_chan_sel_enabled())
    {
     	scan_time = MAX(scan_time, MIN_AUTO_CH_SCAN_TIME);
    }

     return scan_time;
}

/* This function checks if channel availability check is required to be done */
/* before starting a BSS.                                                    */
INLINE BOOL_T is_chnl_scan_needed(UWORD8* start_tbl_idx, UWORD8* end_tbl_idx)
{
    UWORD8 freq         = get_current_start_freq();

    /* If Auto Channel selection is needed */
    if(BTRUE == is_auto_chan_sel_enabled())
    {
        PRINTD("Auto Channel Selection in progress......");

        *start_tbl_idx =  get_tbl_idx_from_ch_idx(freq, mget_CurrentChannel());

        if(*start_tbl_idx > 0)
        {
			*end_tbl_idx = *start_tbl_idx - 1;
		}
		else
		{
			*end_tbl_idx = get_num_supp_channel(freq) - 1;
		}

        return BTRUE;
    }


#ifdef MAC_802_11H
    /* If 11h initial scan is needed */
    if((mget_enableDFS() == BTRUE) &&
       (is_ch_in_radar_band(freq, mget_CurrentChannel()) == BTRUE))
    {
        *start_tbl_idx = get_tbl_idx_from_ch_idx(freq, mget_CurrentChannel());
        *end_tbl_idx   = *start_tbl_idx;

        enable_radar_detection();
        unmask_machw_radar_det_int();

        return BTRUE;
    }
#endif /* MAC_802_11H */

   return BFALSE;
}

/* Returns if 40 MHz network is possible with the current channel scan info */
INLINE BOOL_T get_40MHz_possibility(eval_scan_res_t *chan_scan_res)
{
	BOOL_T fortyMHz_poss = BFALSE;


	return fortyMHz_poss;
}

/* This funtion returns if a 40MHz network can be started using Above Secondary channel  */
INLINE BOOL_T is_40MHz_SCA_allowed (eval_scan_res_t *chan_scan_res,
                                 UWORD8 tbl_idx, UWORD8 user_chan_offset)
{
   return BFALSE;
}


/* This funtion returns if a 40MHz network can be started using Below Secondary channel  */
INLINE BOOL_T is_40MHz_SCB_allowed (eval_scan_res_t *chan_scan_res,
                                 UWORD8 tbl_idx, UWORD8 user_chan_offset)
{
   return BFALSE;
}

/* This function marks the channel which are not blocked due to radar */
INLINE void check_radar(eval_scan_res_t *chan_scan_res, UWORD8 num_sup_chan)
{
    UWORD8 i = 0;

    /* Check the entire channel list till a channel is found with status     */
    /* AVAILABLE_TO_OPERATE or AVAILABLE_ALWAYS. The checking is started     */
    /* from the channel set by  the user.                                    */
    for(i = 0; i < num_sup_chan; i++)
    {
#ifdef MAC_802_11H
		if(mget_enableDFS() == BTRUE)
		{
			if((g_ap_channel_list[i].dfs_ch_status == AVAILABLE_TO_OPERATE) ||
			   (g_ap_channel_list[i].dfs_ch_status == AVAILABLE_ALWAYS))
			{
				chan_scan_res[i].chan_op |= OP_ALLOWED;
			}
		}
		else
#endif /* MAC_802_11H */
		{
	        chan_scan_res[i].chan_op |= OP_ALLOWED;
		}

    }
}


/* This function processes MISC events based on the protocol in use in the   */
/* WAIT_START state.                                                         */
INLINE void wait_start_misc_event_prot_ap(mac_struct_t *mac, UWORD8 *msg)
{
#ifdef MAC_802_11H
    wait_start_misc_event_11h_ap(mac, msg);
#endif /* MAC_802_11H */
}

/* This function handles TBTT event based on the protocol in use */
INLINE void handle_tbtt_prot_ap(void)
{
#ifdef MAC_802_11H
    handle_tbtt_11h_ap();
#endif /* MAC_802_11H */
}

/* This function posts a RADAR DETECT event in AP mode */
INLINE void post_radar_detect_event_11h_ap(void)
{
#ifdef MAC_802_11H
    if(mget_enableDFS() == BTRUE)
    {

        misc_event_msg_t *misc = 0;
        misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

        if(misc == NULL)
        {
            /* Exception */
            raise_system_error(NO_EVENT_MEM);
            return;
        }

        misc->data = 0;
        misc->name = MISC_RADAR_DETECTED;

        post_event((UWORD8*)misc, MISC_EVENT_QID);
    }
    else
#endif /* MAC_802_11H */
    {
		PRINTD("Warning: Radar detected when 11H/DFS is OFF\n");

        /* Disable radar detection and mask the interrupt */
        disable_radar_detection();
        mask_machw_radar_det_int();
    }
}

/* Set the Spectrum management parameters for beacon frames */
INLINE UWORD16 set_spectrum_mgmt_params(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_802_11H
    return set_spectrum_mgmt_params_ap(data, index);
#endif /* MAC_802_11H */

    return 0;
}


/*****************************************************************************/
/* 802.11n Related Prot-ifcation functions                                   */
/*****************************************************************************/

/* Check if a packet must be buffered due to the service class policy */
INLINE BOOL_T is_serv_cls_buff_pkt_ap(asoc_entry_t *entry, UWORD8 q_num,
                                      UWORD8 tid, UWORD8 *tx_dscr)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (NULL != entry))
        return is_ba_buff_tx_pkt(&((asoc_entry_t *)entry)->ba_hdl,
                                 q_num, tid, tx_dscr);
#endif /* MAC_802_11N */

    return BFALSE;
}

/* This function indicates if a transmit Block ACK session is active for the */
/* given station entry and TID.                                              */
INLINE BOOL_T is_txba_session_active_ap(asoc_entry_t *entry, UWORD8 tid)
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

/* This function updates pending buffer parameters of Tx-BA session */
INLINE void update_txba_session_params_ap(asoc_entry_t *entry, UWORD8 tid,
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

/* This function sets the HT Capability Information Field in AP mode */
INLINE void set_ht_capinfo_ap(UWORD8 *data, TYPESUBTYPE_T frame_type)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        set_ht_capinfo_field_ap(data, frame_type);
#endif /*MAC_802_11N*/
}

/* This function sets the OBSS Scan Parameters Info-Element in AP mode */
INLINE UWORD8 set_obss_scan_params(UWORD8 *data, UWORD16 index)
{
    UWORD8 retval = 0;


    return retval;
}

/* This function returns the Block-Ack handle for the specified station */
INLINE void *get_ba_handle_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    if(NULL != entry)
        return &(entry->ba_hdl);
#endif /* MAC_802_11N */

    return NULL;
}

/* This function returns the HT handle for the specified station */
INLINE void *get_ht_handle_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    if(NULL != entry)
        return &(entry->ht_hdl);
#endif /* MAC_802_11N */

    return NULL;
}

/* This function checks the HT capability of the specified station. */
/* Returns BTRUE - If Station HT capable  BFALSE - Otherwise        */
INLINE BOOL_T is_ht_capable_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (1 == entry->ht_hdl.ht_capable))
        return BTRUE;
#endif /* MAC_802_11N */

    return BFALSE;
}

/* This function checks the BlockAck capability of the specified station. */
/* Returns BTRUE - If Station supports BlockAck  BFALSE - Otherwise       */
INLINE BOOL_T is_imm_ba_capable_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    if(1 == entry->ba_hdl.imm_ba_capable)
        return BTRUE;
#endif /* MAC_802_11N */

    return BFALSE;
}

INLINE void handle_ba_active_ap_prot(asoc_entry_t *ae)
{
#ifdef MAC_802_11N
    handle_ba_active_ap(ae);
#endif /* MAC_802_11N */
}

INLINE void set_ht_ps_params_ap(UWORD8 *dscr, asoc_entry_t *ae, UWORD8 rate)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        set_11n_ht_ps_params_ap(dscr, ae, rate);
#endif /* MAC_802_11N */
}

/* This function updates the asctn entry with the HT specific parameters */
INLINE UWORD8 check_ht_capabilities_ap(UWORD8 *msa, UWORD16 offset,
                                       UWORD16 rx_len, void *entry)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        return check_11n_params_ap(msa, offset, rx_len, entry);
#endif /* MAC_802_11N */

    return 0;
}

/* This function checks whether the receiver is HT capable */
INLINE BOOL_T is_htc_capable_ap(asoc_entry_t *entry)
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
INLINE void update_serv_class_prot_ap(asoc_entry_t *entry, UWORD8 tid,
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

/* This function sets the HT-Operation Information element */
INLINE UWORD8 set_ht_opern_element_ap(UWORD8 *data, UWORD16 index)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        return set_ht_opern_11n_ap(data, index);
    }
#endif /* MAC_802_11N */

    return 0;
}

/* This function processes overlapping non-HT BSS */
INLINE void process_obss_beacon(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi)
{
	/* Process the info from 11b APs/STAs */
	process_obss_erp_info(msa, rx_len, rssi);

#ifdef MAC_802_11N
    /* In HT mode */
    if(get_ht_enable() == 1)
    {
        /* Process the info from non-HT APs/STAs */
        process_obss_nonht_sta(msa, rx_len, rssi);

    }
#endif /* MAC_802_11N */
}

/* This function processes overlapping non-HT BSS */
INLINE void process_nonht_obss(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        process_obss_nonht_sta(msa, rx_len, rssi);
    }
#endif /* MAC_802_11N */
}

/* This function handles the specific protocol related tasks when PS modes */
/* of the STA changes from Active to Power-Save                            */
INLINE void handle_ps_sta_change_to_powersave_prot(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        handle_ba_powersave_ap(entry);
    }
#endif /* MAC_802_11N */
}

/* This function updates the specific protocol related contexts on */
/* transmission of power-save buffered frame.                      */
INLINE void update_ps_ctxt_prot(asoc_entry_t *entry, UWORD8 tid)
{
#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (NULL != entry))
        update_halted_ba_ctxt(&(entry->ba_hdl.ba_tx_ptr[tid]));
#endif /* MAC_802_11N */
}

/* Filter the frame received from the WLAN based on the service class */
INLINE BOOL_T filter_wlan_serv_cls_ap(wlan_rx_t *wlan_rx)
{
    BOOL_T retval = BFALSE;
#ifdef MAC_802_11N
    UWORD8 pri_val = wlan_rx->priority_val;

    retval = filter_wlan_ba_serv_cls(wlan_rx,
                 &(((asoc_entry_t*)wlan_rx->sa_entry)->ba_hdl.ba_rx_ptr[pri_val]));
#endif /* MAC_802_11N */

    return retval;
}

/* Handle action request based on protocol enabled */
INLINE void ap_enabled_action_req(mac_struct_t *mac, UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        ap_enabled_action_req_11n(msg);
#endif /* MAC_802_11N */
}

/* Process the control message in access point mode */
INLINE void ap_enabled_rx_control(UWORD8 *msg)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        ap_enabled_rx_11n_control(msg);
#endif /* MAC_802_11N */
}

INLINE void update_chan_info_prot(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi,
                                  UWORD8 tbl_index, UWORD8 ch_offset )
{
}

/*****************************************************************************/
/* Multiple protocol Related Prot-ifcation functions                         */
/*****************************************************************************/
/* Set the capability information fields for AP according to protocol */
INLINE void set_capability_prot_ap(UWORD8* data, UWORD16 index)
{
#ifdef MAC_802_11H
    set_spectrum_mgmt_bit_ap(data,index);
#endif /* MAC_802_11H */

#ifdef MAC_802_11N
    set_capability_11n(data, index);
#endif /* MAC_802_11N */
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAC_P2P
    g_p2p_GO_role             = BTRUE;
    mset_p2p_GO(BTRUE);
    mset_p2p_dev_lmt(BTRUE);
    init_p2p_globals();
#endif /* MAC_P2P */
#endif
}

/* Update the UAPSD details in the association entry */
INLINE void update_asoc_entry_prot(asoc_entry_t *ae, UWORD8 *msa,
                                   UWORD16 rx_len, UWORD16 cap_info,
                                   BOOL_T *is_p2p)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        update_asoc_uapsd_status(ae, msa, rx_len);
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    update_ba_cap(&(ae->ba_hdl), cap_info);
#endif /* MAC_802_11N */
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        UWORD8  *attr_ptr = 0;
        UWORD16 ie_len    = 0;
        UWORD16 scratch_mem_idx = get_scratch_mem_idx();

        attr_ptr = get_p2p_attributes(msa,
                        MAC_HDR_LEN + CAP_INFO_LEN + LISTEN_INT_LEN,
                        rx_len, &ie_len);
        if(NULL != attr_ptr)
        {
            update_p2p_asoc_client_info((void *)ae, attr_ptr, ie_len);
            *is_p2p = BTRUE;

            /* Restore the scratch memory which was used to copy the         */
            /* attributes.                                                   */
            restore_scratch_mem_idx(scratch_mem_idx);
        }
        else
            *is_p2p = BFALSE;

    }
#endif /* MAC_P2P */
#else
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        UWORD8  *attr_ptr = 0;
        UWORD16 ie_len    = 0;
        UWORD16 offset    = MAC_HDR_LEN + CAP_INFO_LEN + LISTEN_INT_LEN;

        /* If the frame is Re-association request, add 6 bytes to the offset */
        /* since this frame includes an extra field (Current AP Address)     */
        if(get_sub_type(msa) == REASSOC_REQ)
        {
			offset += MAC_ADDRESS_LEN;
		}

        attr_ptr = get_p2p_attributes(msa, offset, rx_len, &ie_len);

        if(NULL != attr_ptr)
        {
            update_p2p_asoc_client_info((void *)ae, attr_ptr, ie_len);
            *is_p2p = BTRUE;

            /* Free the local memory buffer allocated for P2P attributes */
			mem_free(g_local_mem_handle, attr_ptr);
        }
        else
            *is_p2p = BFALSE;

    }
#endif /* MAC_P2P */
#endif

}

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
/* Handle Tx Complete event for different protocols in AP mode */
INLINE void handle_prot_tx_comp_ap(UWORD8 *dscr, void *entry)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        handle_ba_tx_comp(dscr, entry);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        handle_11i_tx_comp_ap(dscr, entry);
    }
#endif /* MAC_802_11I */

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        handle_p2p_go_disc_req_com_ap(dscr);
    }
#endif /* MAC_P2P */

    return;
}
#else
/* Handle Tx Complete event for different protocols in AP mode */
INLINE void handle_prot_tx_comp_ap(UWORD8 *dscr, void *entry, UWORD8 *msa)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        handle_ba_tx_comp(dscr, entry, msa);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        handle_11i_tx_comp_ap(dscr, entry, msa);
    }
#endif /* MAC_802_11I */

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        /* Check if there is an ivitation pending and if then channel */
        /* was switched to search for the device                      */
        if((SEND_INVITE == g_p2p_invit) && (1 == g_p2p_switch_chan))
        {
            handle_p2p_prb_req_com_ap(dscr);
		}
		else if(WAIT_INVITE_RSP == g_p2p_invit)
		{
			/* If an inviation request was sent then handle its tx */
			/* complete                                            */
			handle_p2p_inv_req_com_ap(dscr);
		}
		else
		{
        	handle_p2p_go_disc_req_com_ap(dscr);
    	}
    }
#endif /* MAC_P2P */

    return;
}
#endif

/* Initialize the security/QOS handle to the default parameters */
INLINE void init_prot_handle_ap(asoc_entry_t *ae)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        init_ba_handle(&(ae->ba_hdl));
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    init_11i_handle_auth_ap(ae);
#endif /* MAC_802_11I */
}

/* Misc Security related event */
INLINE void ap_enabled_misc_prot(mac_struct_t *mac, UWORD8 *msg)
{

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        misc_11n_event(mac, msg);
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11I
    misc_11i_event_auth(mac, msg);
#endif /* MAC_802_11I */

#ifdef MAC_802_11H
    enabled_misc_event_11h_ap(mac, msg);
#endif /* MAC_802_11H */
}

/* Handle the protocol dependent management packets */
INLINE void handle_mgmt_prot_ap(UWORD8 *msg, UWORD8 sub_type,
                                mac_struct_t *mac)
{
    switch(sub_type)
    {
    case ACTION:
    {
#ifdef MAC_WMM
        if(get_wmm_enabled() == BTRUE)
        {
            ap_enabled_wmm_rx_action(msg);
        }
#endif /* MAC_WMM */

#ifdef MAC_802_11N
        if(get_ht_enable() == 1)
        {
            ap_enabled_rx_11n_action(msg);
        }
#endif /* MAC_802_11N */

#ifdef MAC_802_11H
        ap_enabled_11h_rx_action(msg);
#endif /* MAC_802_11H */

#ifdef MAC_P2P
		// 20120709 caisf add the if, merged ittiam mac v1.2 code
        if(BTRUE == mget_p2p_enable())
        {
        	ap_enabled_rx_p2p_action(msg, mac);
		}
#endif /* MAC_P2P */
    }
    break;
    default:
    {
        /* Do nothing */
    }
    break;
    }
}

/* Reset the security/QOS entry by deleting LUT and stopping the FSMs */
/* Reset the UAPSD sta entries */
INLINE void reset_prot_entry_ap(asoc_entry_t *ae)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        reset_uapsd_entry(ae);
#endif /* MAC_WMM */

#ifdef MAC_802_11I
    reset_11i_entry_auth_ap(ae);
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        reset_ba_handle(&(ae->ba_hdl));
        reset_11n_entry_ap(ae);
    }
#endif /* MAC_802_11N */

    reset_wep_entry_ap(ae);
}
/* This function returns the number of TX-AMPDU sessions currently active */
/* with the specified station.                                            */
INLINE UWORD8 get_num_tx_ampdu_sessions_ap(asoc_entry_t *entry)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
    retval = get_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */

    return retval;
}

/* This function increments the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void incr_num_tx_ampdu_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */
}

/* This function increments the number of TX-AMPDU sessions currently  */
/* active with the specified station.                                  */
INLINE void decr_num_tx_ampdu_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_tx_ampdu_sessions_11n(&(entry->ht_hdl));
#endif /* MAC_802_11N */
}

/* This function returns the number of Block-Ack TX sessions setup with */
/* the specified station.                                               */
INLINE UWORD8 get_num_tx_ba_sessions_ap(asoc_entry_t *entry)
{
    UWORD8  retval   = 0;

#ifdef MAC_802_11N
    get_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */

    return retval;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_tx_ba_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_tx_ba_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_tx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_rx_ba_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    incr_num_rx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_rx_ba_sessions_ap(asoc_entry_t *entry)
{
#ifdef MAC_802_11N
    decr_num_rx_ba_sessions_11n(&(entry->ba_hdl));
#endif /* MAC_802_11N */
}

#ifdef AUTORATE_FEATURE
/* This function checks if a given MCS is supported by the given STA entry.  */
/* 11N TBD - Note that only the Rx MCS bitmask is currently checked. The     */
/* Highest Supported Data Rate field is not checked.                         */
INLINE BOOL_T is_mcs_supp_ap(asoc_entry_t *entry, UWORD8 mcs_index)
{
#ifdef MAC_802_11N
    if(is_ht_capable_ap(entry) == BTRUE)
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

/* This function returns the minimum non-zero TXOP limit configured across   */
/* all ACs. If all WMM is disabled or TXOP-Limits are zero, then 0xFFFF is   */
/* returned.                                                                 */
INLINE UWORD16 get_min_non_zero_txop_ap(asoc_entry_t *ae)
{
    UWORD16 min_txop = 0xFFFF;

#ifdef MAC_WMM
    if((get_wmm_enabled() == BTRUE) &&
       (is_dst_wmm_capable(ae->sta_index) == BTRUE))
    {
        min_txop = get_min_non_zero_txop_wmm_ap();
    }
#endif /* MAC_WMM */

    return min_txop;
}

INLINE void update_tx_mcs_index_ar_ap(asoc_entry_t *ae)
{
#ifdef MAC_802_11N
    UWORD8 indx = 0;
    UWORD8 rate = 0;
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_ap(ae);

    /* Search for any MCS at lower indexes than the current Tx rate index in */
    /* the AR table                                                          */
    for(indx = ae->tx_rate_index; indx > 0; indx--)
    {
        rate = get_ar_table_rate(indx);

        if(IS_RATE_MCS(rate) == BTRUE)
        {
            if((is_rate_supp(rate, ae) == 1) &&
               (is_rate_allowed(rate, ae) == 1))
            {
                ht_hdl->tx_mcs_index = indx;
                return;
            }
        }
    }

    /* If there are no MCS rates at lower indexes of AR table then try to */
    /* search at indexes greater than current TX rate                     */
    for(indx = (ae->tx_rate_index + 1); indx < get_ar_table_size(); indx++)
    {
        if(IS_RATE_MCS(get_ar_table_rate(indx)) == BTRUE)
        {
            ht_hdl->tx_mcs_index = indx;
            return;
        }
    }

#ifdef DEBUG_MODE
    /* Exception */
    PRINTD("update_tx_mcs_index_ar_ap: Exception\n\r");
    g_mac_stats.no_mcs_index++;
#endif /* DEBUG_MODE */
#endif /* MAC_802_11N */
}

/* This function checks if SGI transmission is enabled for current receiver */
INLINE BOOL_T is_sgi_possible_ap(asoc_entry_t *ae)
{
    BOOL_T ret_val      = BFALSE;
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_ap(ae);

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

INLINE BOOL_T get_tx_sgi_ap(asoc_entry_t *ae)
{
    BOOL_T ret_val = BFALSE;
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_ap(ae);

    ret_val = ht_hdl->tx_sgi;
#endif /* MAC_802_11N */

    return ret_val;
}

INLINE void set_tx_sgi_ap(BOOL_T tx_sgi, asoc_entry_t *ae)
{
#ifdef MAC_802_11N
    ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_ap(ae);

    ht_hdl->tx_sgi = tx_sgi;
#endif /* MAC_802_11N */
}

#endif /* AUTORATE_FEATURE */

INLINE UWORD8 get_ht_tx_rate_ap(asoc_entry_t *ae)
{
#ifdef AUTORATE_FEATURE
    if(is_autorate_enabled() == BTRUE)
    {
        UWORD8 curr_tx_mcs = 0;
        ht_struct_t *ht_hdl = (ht_struct_t *)get_ht_handle_ap(ae);

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
    if(is_ht_capable_ap(ae) == BTRUE)
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

/* For all the incoming MPDUs, the security check is performed so as to */
/* determine if the incoming packets have been tampered                 */
INLINE BOOL_T check_sec_rx_mpdu_ap(UWORD8 *ae, CIPHER_T ct, UWORD8 *msa,
                                   UWORD8 *pn_val)
{
#ifdef MAC_802_11I
    return check_11i_rx_mpdu_ap((asoc_entry_t *)ae, ct, msa, pn_val);
#else /* MAC_802_11I */
    return BTRUE;
#endif /* MAC_802_11I */
}

/* This function checks whether the Receive Service Class for the specified */
/* entry is Block-Ack.                                                      */
INLINE BOOL_T is_rx_ba_service_class_ap(asoc_entry_t *ae, UWORD8 tid)
{
    BOOL_T retval = BFALSE;

#ifdef MAC_802_11N
    if((get_ht_enable() == 1) && (ae != NULL))
    {
        retval = is_ba_rx_session_setup(&(ae->ba_hdl.ba_rx_ptr[tid]));
    }
#endif /* MAC_802_11N */

    return retval;
}

/* This function checks whether a MAC reset is allowed to be done under the */
/* current operating state.                                                 */
INLINE BOOL_T allow_mac_reset_ap(void)
{
    BOOL_T retval = BTRUE;

#ifdef MAC_802_11I
    if(is_cntr_msr_in_progress_ap() == BTRUE)
        retval = BFALSE;
#endif /* MAC_802_11I */

    return retval;
}

/*************************************************************************/
/*                'Format of Type of STA'                                */
/*-----------------------------------------------------------------------*/
/*    0: station is not associated with the AP                           */
/*    1: station supports 11a rates only (5 GHz)                         */
/*    2: station supports 11a rates only (2.4 GHz)                       */
/*    3: station supports 11b rates only (2.4 GHz)                       */
/*    4: station supports 11a & 11b rates (2.4 GHz)                      */
/*    5: station supports 11n rates (2.4 GHz)                            */
/*    6: station supports 11n rates (5 GHz)                              */
/*************************************************************************/

INLINE void update_sta_phy_type_info(asoc_entry_t *ae, UWORD8 *ptr)
{
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        if(ae->ht_hdl.ht_capable == 1)
        {
            *ptr = 6;
        }
        else
        {
            *ptr = 1;
        }
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
#ifdef MAC_802_11N
        if(ae->ht_hdl.ht_capable == 1)
        {
            *ptr = 5;
        }
        else
#endif /* MAC_802_11N */
        {
            /* 11b station */
            if((ae->op_rates.num_rates <= 4) && (ae->tx_rate_mbps <= 11))
            {
                 *ptr = 3;
            }
            /* 11g-HP station */
            else if((ae->op_rates.num_rates <= 8) && (ae->tx_rate_mbps >= 6))
            {
                *ptr = 2;
            }
            else
            {
                *ptr = 4;
            }
        }
    }
}

/* This function updates the Station Capability Information fields for */
/* various protocols. The function takes the association entry and the */
/* pointer to the start of the STA JOIN INFO message as the arguments  */
/*************************************************************************/
/* Format of STA JOIN INFO message                                       */
/* ----------------------------------------------------------------------*/
/* |Last byte| MAC address  | Type of STA    |  11g info  | Security     */
/* | of AID  | of STA       | 11 a/b/g/n     |  byte      | byte         */
/* ----------------------------------------------------------------------*/
/* | 1 byte  | 6 bytes      | 1 byte         |  1 byte    |  1 byte      */
/* ----------------------------------------------------------------------*/
/*                                                                       */
/* ----------------------------------------------------------------------*/
/* | WMM/QoS info     | 11n capability     |   Reserved                  */
/* | bytes            | bytes              |   bytes                     */
/* ----------------------------------------------------------------------*/
/* | 2 bytes          | 2 bytes            |   2 bytes                   */
/* ----------------------------------------------------------------------*/

INLINE void update_sta_info_prot(asoc_entry_t *ae, UWORD8 *ptr)
{
/*************************************************************************/
/*                'Format of Security Info Byte'                         */
/*-----------------------------------------------------------------------*/
/* BIT0: 1 -> Security ON              0 -> OFF                          */
/* BIT1: 1 -> WEP40  cipher supported  0 -> Not supported                */
/* BIT2: 1 -> WEP104 cipher supported  0 -> Not supported                */
/* BIT3: 1 -> WPA mode      supported  0 -> Not supported                */
/* BIT4: 1 -> WPA2 (RSN)    supported  0 -> Not supported                */
/* BIT5: 1 -> AES-CCMP cphr supported  0 -> Not supported                */
/* BIT6: 1 -> TKIP   cipher supported  0 -> Not supported                */
/* BIT7: 1 -> TSN           supported  0 -> Not supported                */
/*************************************************************************/
    if(ae->cipher_type != NO_ENCRYP)
     {
         UWORD8 security_info = BIT0;

#ifdef MAC_802_11I
         if(ae->persta)
         {
             /* WPA or WPA2 info */
             security_info |= (ae->persta->mode_802_11i << 3);

             /* AES-CCMP support */
             if(ae->persta->pcip_policy == 0x04)
             {
                 security_info |= BIT5;
             }
             /* TKIP support */
             else if(ae->persta->pcip_policy  == 0x02)
             {
                 security_info |= BIT6;
             }
         }
         else
#endif /* MAC_802_11I */
         /* If WEP40 or WEP104 info */
         if(ae->cipher_type == WEP40)
         {
             security_info = (BIT0 | BIT1);
         }
         else if (ae->cipher_type == WEP104)
         {
             security_info = (BIT0 | BIT1 | BIT2);
         }

         *(ptr + 9) = security_info;
     }

#ifdef MAC_WMM
    update_sta_info_wmm(ae, ptr+10);
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    update_sta_info_11n(ae, ptr+12);
#endif /* MAC_802_11N */
}

#ifdef DEBUG_MODE
/* This function checks for consistency of the build parameters used */
/* in AP mode.                                                       */
INLINE void check_build_params_ap(void)
{

}
#endif /* DEBUG_MODE */

/*****************************************************************************/
/* WPS Related Prot-ifcation functions                                       */
/*****************************************************************************/

/* This function calls appropriate function to insert WSC-IE in the passed   */
/* frames                                                                    */
INLINE UWORD16 set_wps_wsc_info_element(UWORD8 *data, UWORD16 index,
                                        TYPESUBTYPE_T frame_type)
{
    UWORD16 retval = 0;

#ifdef INT_WPS_SUPP
    retval = set_wps_wsc_info_element_ap(data, index, frame_type);
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function call approriate functions to handle reception of PBC        */
/* Enrollee probe request                                                    */
INLINE void wps_handle_probe_req(UWORD8 *msa, UWORD8 *sa, UWORD16 rx_len,
                                 TYPESUBTYPE_T frame_type)
{
#ifdef INT_WPS_SUPP
    wps_handle_probe_req_ap(msa, sa, rx_len, frame_type);
#endif /* INT_WPS_SUPP */
}

/* This function processes WPS events in AP mode */
INLINE BOOL_T process_wps_event_ap(UWORD32 event)
{
    BOOL_T retval = BFALSE;

#ifdef INT_WPS_SUPP
    retval = wps_handle_event_ap(event);
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function checks and returns BTRUE or BFALSE if WID write request is  */
/* to be processed                                                           */
INLINE BOOL_T process_wid_write_prot_ap(UWORD8 *req, UWORD16 msg_len)
{
    BOOL_T retval = BTRUE;

#ifdef INT_WPS_SUPP
      retval = wps_process_wid_write_ap(req, msg_len);
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function handles updating of Credentials in Persistent memory on     */
/* every WID write request                                                   */
INLINE void wps_handle_config_update_ap(void)
{
#ifdef INT_WPS_SUPP
      wps_handle_cred_update_ap();
#endif /* INT_WPS_SUPP */
}

INLINE BOOL_T allow_sys_restart_prot_ap(ERROR_CODE_T sys_error)
{
    BOOL_T retval = BTRUE;

#ifdef INT_WPS_SUPP
    retval = wps_allow_sys_restart_ap(sys_error);
#endif /* INT_WPS_SUPP */
    return retval;
}

INLINE void sys_init_prot_ap(void)
{
#ifdef INT_WPS_SUPP
    sys_init_wps_reg();
#endif /* INT_WPS_SUPP */
}

/* This function sets the WPS Registrar Mode to Enabled/Disabled */
INLINE void set_wps_enable_ap(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    set_wps_reg_enabled((BOOL_T)val);

    /* Set WPS Registrar implemented status also */
    set_wps_implemented_reg((BOOL_T)val);
#endif /* INT_WPS_SUPP */
}

/* This function returns the WPS Enabled status */
INLINE BOOL_T get_wps_enable_ap(void)
{
    BOOL_T retval = BFALSE;

#ifdef INT_WPS_SUPP
    retval = get_wps_implemented_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function gets the WPS Protocol type configured */
INLINE UWORD8 get_wps_prot_ap(void)
{
    UWORD8 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_prot_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function sets the WPS Protocol type */
INLINE void set_wps_prot_ap(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_prot_reg(val);
    }
#endif /* INT_WPS_SUPP */
}

/* This function sets the WPS Protocol type */
INLINE void handle_oob_reset_req_ap(UWORD8 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        if(0 != val)
            wps_handle_oob_reset_req_ap();
    }
#endif /* INT_WPS_SUPP */
}

/* This function return the WPS Password ID programmed */
INLINE UWORD16 get_wps_pass_id_ap(void)
{
    UWORD16 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_pass_id_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_pass_id_ap(UWORD16 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_pass_id_reg(val);
    }
#endif /* INT_WPS_SUPP */
}

INLINE UWORD16 get_wps_config_method_ap(void)
{
    UWORD16 retval = 0;

#ifdef INT_WPS_SUPP
    retval = get_wps_config_method_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_config_method_ap(UWORD16 val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_config_method_reg(val);
    }
#endif /* INT_WPS_SUPP */
}

INLINE UWORD8 *get_wps_pin_ap(void)
{
    UWORD8 *retval = NULL;

#ifdef INT_WPS_SUPP
    retval = get_wps_pin_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

INLINE void set_wps_pin_ap(UWORD8 *val)
{
#ifdef INT_WPS_SUPP
    if(BFALSE == g_reset_mac_in_progress)
    {
        set_wps_pin_reg(val);
    }
#endif /* INT_WPS_SUPP */
}

/* This function handles updates done in device information by the user */
INLINE void update_device_specific_info_ap(void)
{
#ifdef INT_WPS_SUPP
    wps_update_device_info_ap();
#endif /* INT_WPS_SUPP */
}

/* This function handles update to WPS Credential by the user */
INLINE void handle_wps_cred_update_ap(void)
{
#ifdef INT_WPS_SUPP
    set_cred_updated_ap(BTRUE);
#endif /* INT_WPS_SUPP */
}

INLINE void handle_disconnect_req_prot_ap(asoc_entry_t *ae, UWORD8 *sta_addr)
{
#ifdef INT_WPS_SUPP
    wps_handle_disconnect_req_ap(ae, sta_addr);
#endif /* INT_WPS_SUPP */
}


/* This function indicate start/stop of WID restoration to all the Protocols */
/* in BSS Access Point mode                                                  */
INLINE void indicate_wid_restore_prot_ap(BOOL_T val)
{
#ifdef INT_WPS_SUPP
    indicate_wid_restore_wps_ap(val);
#endif /* INT_WPS_SUPP */
}

/* This function checks if any of the protocol requires that authentication  */
/* type checking be skipped.                                                 */
INLINE BOOL_T check_auth_type_prot(void)
{
    BOOL_T retval = BTRUE;

#ifdef INT_WPS_SUPP
    /* If WID Set Configuration is a allowed then there is not WPS Start     */
    /* Request from the user, hence need to check Authentication Type, else  */
    /* disable checking of Authentication Type                               */
    retval = get_wps_allow_config();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This fucntion return BTRUE if the WPS Registrar functionality is          */
/* implemented else returns BFALSE                                           */
INLINE BOOL_T get_wps_implemented_ap(void)
{
    BOOL_T retval = BTRUE;

#ifdef INT_WPS_SUPP
    retval = get_wps_implemented_reg();
#endif /* INT_WPS_SUPP */

    return retval;
}

/* This function sets WPS Registrar implemented flag */
INLINE void set_wps_implemented_ap(BOOL_T val)
{
#ifdef INT_WPS_SUPP
    set_wps_implemented_reg(val);
#endif /* INT_WPS_SUPP */
}

/* This function sets the WPS Registrar mode with specified input value */
INLINE void enable_wps_mode_ap(BOOL_T val)
{
#ifdef INT_WPS_SUPP
    set_wps_reg_enabled(val);
#endif /* INT_WPS_SUPP */
}

/*****************************************************************************/
/* P2P Related Prot-ifcation functions                                       */
/*****************************************************************************/

/* This funtion checks if P2P is enable then it adds the corresponding       */
/* IE for the beacon frame.                                                  */
INLINE UWORD16 set_p2p_beacon_ie(UWORD8* data, UWORD16 index)
{
    UWORD16 ie_len = 0;

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        ie_len = add_p2p_beacon_ie(data, index);
    }
#endif /* MAC_P2P */
    return ie_len;
}


/* The function checks if P2P is enabled and the asoc req was sent by a P2P  */
/* device.                                                                   */
INLINE UWORD16 set_asoc_resp_p2p_ie(UWORD8 *data, UWORD16 index,
                                    UWORD16 status, BOOL_T is_p2p)
{
    UWORD16 ie_len = 0;

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        if(BTRUE == is_p2p)
        {
            ie_len = add_asoc_resp_p2p_ie(data, index, status);
        }
    }
#endif /* MAC_P2P */
    return ie_len;
}

INLINE BOOL_T handle_prob_req_prot_ap(UWORD8 *msa, UWORD16 rx_len,
                                      UWORD8 *sa)
{
    BOOL_T  is_p2p = BFALSE;

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        /* Check if the probe request has a P2P IE */
        UWORD16 index  = is_p2p_frame(msa, MAC_HDR_LEN, rx_len);
        /* Check if the probe request has a P2P IE */
        if(index < (rx_len - FCS_LEN))
        {
            is_p2p = BTRUE;
            /* Check if an ivitation request is pending */
            if((SEND_INVITE == g_p2p_invit) &&
               (BTRUE == mac_addr_cmp(sa, mget_p2p_trgt_dev_id())))
            {
                /* Call the function to handle the pending request */
                p2p_send_invit_ap(msa, rx_len, sa, index);
            }
        }
    }

#endif /* MAC_P2P */

    return is_p2p;
}

/* This function checks if a Probe response needs to be sent by the AP */
INLINE BOOL_T is_probe_rsp_to_be_sent(UWORD8 *msa, UWORD16 len, BOOL_T is_p2p)
{
	BOOL_T send_prb_rsp = BFALSE;

	/* Check for Probe request SSID match */
	if(probe_req_ssid_cmp(msa, mget_DesiredSSID()) == BTRUE)
	{
		send_prb_rsp = BTRUE;
	}

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		/* If the non-P2P check has determined that Probe response should    */
		/* not be sent due to mismatched SSID, do another check for P2P      */
		/* Probe request SSID match. If this check also fails then no        */
		/* further checks are required. Return BFALSE i.e. Probe response    */
		/* need not be sent. If the non-P2P SSID check had passed, no P2P    */
		/* Probe request SSID checks are required.                           */
		// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
		if((BFALSE == send_prb_rsp) &&
		   (probe_req_ssid_cmp(msa, P2P_WILDCARD_SSID) == BFALSE))
		{
			send_prb_rsp = BFALSE;
		}
#else
		if((BFALSE == send_prb_rsp) &&
		   (probe_req_ssid_cmp(msa, P2P_WILDCARD_SSID) == BTRUE))
		{
			send_prb_rsp = BTRUE;
		}
#endif

		/* If SSID checks have passed, check for P2P specific information if */
		/* a P2P IE is present.                                              */
		/* 1) Requested device type attribute matches that of Group Owner or */
		/*    one of the clients.                                            */
		/* 2) Requested device ID attribute matches that of Group Owner or   */
		/*    one of the clients.                                            */
		// 20120709 caisf mod, merged ittiam mac v1.2 code
		if((BTRUE == send_prb_rsp) &&
		   (BTRUE == is_p2p)       &&
		   ((p2p_probe_req_dev_id_cmp(msa, len) == BFALSE) ||
			   (p2p_probe_req_dev_type_cmp(msa, len) == BFALSE)))
		{
			send_prb_rsp = BFALSE;
		}

    }
#endif /* MAC_P2P */

	return send_prb_rsp;
}

/* This function updates the WPS complete flag as required for P2P protocol. */
/* If P2P is enabled and a Target Device Address has been set, it checks if  */
/* WPS has been completed for the Target Device Address and sets the flag to */
/* BTRUE. This flag is used to check if clients other than the target device */
/* can be allowed to associate. Note that this check is valid only for the   */
/* first WPS exchange with the target Client after the GO comes up after a   */
/* GO negotiation. Thereafter no further checks are done i.e. the flag is    */
/* never reset.                                                              */
INLINE void update_wps_complete_flag(UWORD8 *mac_addr)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		/* Target Device WPS has not been done */
		if(g_target_p2p_wps_done == BFALSE)
		{
			UWORD8 *trgt_dev = mget_p2p_trgt_dev_id();

			/* No Target Device is configured or the WPS Enrollee address    */
			/* matches the Target Device configured for P2P.                 */
			if((BTRUE == is_mac_addr_null(trgt_dev)) ||
			   (BTRUE == mac_addr_cmp(trgt_dev, mac_addr)))
			{
				g_target_p2p_wps_done = BTRUE;
			}
		}
    }
#endif /* MAC_P2P */
}

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0 
/* This function checks if a STA may be allowed to authenticate/associate at */
/* this time. A new STA may be allowed only if the Target Device Address is  */
/* set to NULL value or if it is a non-NULL value the following conditions   */
/* are met: 1. The transmitter STA address matches the target, 2. The target */
/* device has already completed WPS.                                         */
INLINE BOOL_T is_new_sta_allowed(UWORD8 *sta_addr)
{
	BOOL_T is_allowed = BTRUE;

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		UWORD8 *trgt_dev = mget_p2p_trgt_dev_id();

		/* Target Device Address is not NULL and the transmitting STA is not */
		/* the Target Device.                                                */
        if((BFALSE == is_mac_addr_null(trgt_dev)) &&
           (BFALSE == mac_addr_cmp(trgt_dev, sta_addr)))
        {
			/* WPS has not been completed with Target Device yet */
			if(BFALSE == g_target_p2p_wps_done)
			{
				is_allowed = BFALSE;
			}
	    }
    }
#endif /* MAC_P2P */

    return is_allowed;
}

/* This function handles the protocol based processing for MISC TIMEOUT in ap*/
/* enabled state                                                             */
INLINE void handle_ap_enabled_misc_to_prot(mac_struct_t *mac)
{
#ifdef MAC_P2P
    if(WAIT_INVITE_RSP == g_p2p_invit)
        handle_inv_req_to(mac);
#endif /* MAC_P2P */
}
#else
/* This function handles the protocol based processing for MISC TIMEOUT in ap*/
/* enabled state                                                             */
INLINE void handle_ap_enabled_misc_to_prot(mac_struct_t *mac)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		if(SEND_INVITE == g_p2p_invit)
		{
			/* Toggle the last bit of g_p2p_switch_chan */
			g_p2p_switch_chan = g_p2p_switch_chan ^ 0x01;

			handle_p2p_invitation_to();
	    }
		else if(WAIT_INVITE_RSP == g_p2p_invit)
		{
            p2p_end_ivitation_process();
		}
	}

#endif /* MAC_P2P */
}
#endif

/* If p2p is enabled then this function calls the function to update the */
/* client list in the persistent credential list                         */
INLINE void update_persist_cred_list_prot_ap(UWORD8 *if_addr)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        if(BTRUE == mget_p2p_persist_grp())
        {
            p2p_update_persist_cl_list(if_addr);
        }
    }
#endif /* MAC_P2P */
}

/* This function sets the P2P NOA schedule in AP mode. In STA mode nothing   */
/* is done since the NOA schedule is adopted from the GO.                    */
INLINE void set_p2p_noa_sched_prot_ap(UWORD8 *val)
{
#ifdef MAC_P2P
    if(BFALSE == g_reset_mac_in_progress)
    set_p2p_noa_sched_ap(val);
#endif /* MAC_P2P */
}

/* This funtion checks if the number of sta associated has reached the max   */
/* number of assocation allowed and updates the P2P group limit in P2P is    */
/* used                                                                      */
INLINE void update_entry_prot(UWORD8 count)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        if(NUM_STA_SUPPORTED > count)
        {
            mset_p2p_grp_lmt(0);
            update_beacon_ap();
        }
        else
        {
            mset_p2p_grp_lmt(1);
            update_beacon_ap();
        }
    }
#endif /* MAC_P2P */
}

INLINE BOOL_T get_p2p_intra_bss_prot(void)
{
    BOOL_T retval = BTRUE;
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
        retval = mget_p2p_intra_bss();
#endif /* MAC_P2P */
    return retval;
}

/* This setting is valid only in AP mode operating as a P2P GO. If the value */
/* is 0, the P2P GO must switch to non-P2P STA mode. The value cannot be set */
/* by a user to 1 (even if it is set it would have no effect).               */
INLINE void set_p2p_enable_ap(UWORD8 p2p_en)
{
#ifdef MAC_P2P
    mset_p2p_enable((BOOL_T)p2p_en);
#endif /* MAC_P2P */
}

/* This function sets Persistent Capability for AP mode */
INLINE void set_p2p_persist_grp_ap(BOOL_T val)
{
#ifdef MAC_P2P
	/* This can be set in GO mode only once when switching from/to Client    */
	/* mode. Otherwise, it is a read-only WID in GO mode.                    */
	if(BTRUE == is_switch_in_progress())
        mset_p2p_persist_grp(val);
#endif /* MAC_P2P */
}

/* This function sets Intra-BSS Capability for AP mode */
INLINE void set_p2p_intra_bss_ap(BOOL_T val)
{
#ifdef MAC_P2P
	/* This can be set in GO mode only once when switching from/to Client    */
	/* mode. Otherwise, it is a read-only WID in GO mode.                    */
	if(BTRUE == is_switch_in_progress())
        mset_p2p_intra_bss(val);
#endif /* MAC_P2P */
}

/* This function sets P2P CT Window depending on mode of operation */
INLINE void set_p2p_ct_window_ap(UWORD8 val)
{
#ifdef MAC_P2P
#ifdef ENABLE_P2P_GO_TEST_SUPPORT
    mset_p2p_CTW(val);

    /* Update the beacon frame with the new CT Window information */
    update_beacon_ap();
#endif /* ENABLE_P2P_GO_TEST_SUPPORT */
#endif /* MAC_P2P */
}

/* This function restores default settings as required before mode switch,   */
/* i.e. AP mode to STA mode.                                                 */
INLINE void restore_default_mode_wids_ap(void)
{
#ifdef MAC_P2P
    /* Switching to STA mode. Auto GO should be always BFALSE to prevent the */
    /* STA from switching back to AP mode due to this setting.               */
    /* Note that this may be required if the switch is a result of setting   */
    /* P2P enable to 0. In that case it would switch to STA mode but if the  */
    /* Auto GO is not set to BFALSE, in STA mode when P2P enable is set to 1 */
    /* it would again switch back to Auto GO mode.                           */
    mset_p2p_auto_go(BFALSE); /* Switching to STA Mode: GO = BFALSE */

    /* Group Capabilities are reset to default when switching back as these  */
    /* may or may not have been user setting (for e.g. a persistent group    */
    /* may have been reinvoked). Note that these WIDs can be set here since  */
    /* a mode switch is in progress when this function is called. Otherwise  */
    /* in GO mode these WIDs cannot be set.                                  */
    set_p2p_persist_grp_ap(BFALSE);
    set_p2p_intra_bss_ap(BFALSE);

    /* Disable WPS */
    set_wps_enable_ap(BFALSE);
#endif /* MAC_P2P */
}

// 20120709 caisf add, merged ittiam mac v1.2 code
#if 1
/* This function process the P2P protocol related processing at TBTT */
/* It checks if there is an invitation pending to be sent and does   */
/* further processing accordingly                                    */
INLINE void process_tbtt_ap_prot(void)
{
#ifdef MAC_P2P

    if((BTRUE == mget_p2p_enable()) && (SEND_INVITE == g_p2p_invit))
    {
        g_p2p_switch_chan = 0;

        /* Start the timer to wait in the operating channel before */
        /* switching to other channel                              */
        start_mgmt_timeout_timer(P2P_INV_OP_CHAN_TO);

	}
#endif /* MAC_P2P */
}

/* This function does the protocol related processing of the probe response */
/* frame received in the AP enabled state                                   */
INLINE void handle_ap_enabled_probe_rsp_prot(UWORD8 *msa, UWORD16 rx_len)
{
#ifdef MAC_P2P
     /* Check if P2P is enabled and an invitation request is pending */
     if((BTRUE == mget_p2p_enable()) && (SEND_INVITE == g_p2p_invit))
     {
		 /* Call the function to do process probe response */
		 p2p_handle_probe_rsp(msa, rx_len);
	 }
#endif /* MAC_P2P */

}

/* This function returns the ssid to set in the frame depending upon the */
/* protocol                                                              */
INLINE WORD8* get_probe_req_ssid_prot(void)
{
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
		return P2P_WILDCARD_SSID;
	}
#endif /* MAC_P2P */

    return mget_DesiredSSID();

}
#endif

#endif /* AP_PROT_IF_H */
#endif /* BSS_ACCESS_POINT_MODE */

