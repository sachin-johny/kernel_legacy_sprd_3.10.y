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
/*  File Name         : prot_if.h                                            */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface.    */
/*                                                                           */
/*  List of Functions : Protocol interface functions for AP/STA mode         */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PROT_IF_H
#define PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ce_lut.h"
#include "itypes.h"
#include "mh.h"
#include "transmit.h"
#include "wep.h"
#include "buff_desc.h"
#include "qif.h"

#ifdef MAC_WMM
#include "frame_11e.h"
#include "mib_11e.h"
#include "management_11e.h"

#ifdef BSS_ACCESS_POINT_MODE
#include "ap_uapsd.h"
#include "fsm_ap.h"
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
#include "sta_management_11e.h"
#include "fsm_sta.h"
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test_config.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

#endif /* MAC_WMM */

#ifdef MAC_802_11H
#include "mib_11h.h"
#endif /* MAC_802_11H */

#ifdef MAC_802_11I
#include "frame_11i.h"
#include "rsna_km.h"
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
#include "frame_11n.h"
#include "amsdu_aggr.h"
#include "amsdu_deaggr.h"
#include "management_11n.h"
#include "blockack.h"
#endif /* MAC_802_11N */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
#include "mib_mdom.h"
#include "frame_mdom.h"
#endif /* MAC_MULTIDOMAIN */

#ifdef INT_WPS_SUPP
#include "wps_sta.h"
#endif /* INT_WPS_SUPP */

#ifdef MAC_P2P
#include "mib_p2p.h"
#include "frame_p2p.h"
#include "p2p_ps.h"
#include "mgmt_p2p.h"

#ifdef IBSS_BSS_STATION_MODE
#include "sta_mgmt_p2p.h"
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
#include "ap_mgmt_p2p.h"
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef INT_WPS_REG_SUPP
#include "wps_ap.h"
#endif /* INT_WPS_REG_SUPP */

extern UWORD16 g_current_len;
extern UWORD8  g_current_settings[];
extern void    restore_wids(void);
#endif /* MAC_P2P */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
#include "wapi.h"
#endif


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#ifdef MAC_WMM

#define NUM_TX_QUEUE          NUM_MAX_EDCA_Q

#define HIGH_PRIORITY         INVALID_TID

#define INVALID_PRI           0xFF

/* Currently a maximum limit is set unifromly across all the queues. This is */
/* taken as an empirical value of 32. May be enhanced in the future to have  */
/* separate queue limits on each queue.                                      */
#define MAX_EDCA_Q_SIZE       32
#define MAX_HCCA_Q_SIZE       20

#else /* MAC_WMM */

#define NUM_TX_QUEUE          3
#define HIGH_PRIORITY         MAX_PRIORITY

/* Based on the maximum number of transmit descriptor buffers available in   */
/* shared memory (currently 29). Indicates that in absence of 802.11e there  */
/* is no queue clamping. As long as there are buffers packets can be added   */
/* to any queue.                                                             */
#define MAX_Q_SIZE            29

#endif /* MAC_WMM */

#ifdef MAC_HW_UNIT_TEST_MODE
#ifndef MAC_WMM
#undef  NUM_TX_QUEUE
#define NUM_TX_QUEUE          5
#endif /* MAC_WMM */
#endif /* MAC_HW_UNIT_TEST_MODE */

#define ICVFAILURE            6
#define MICFAILURE            5
#define RXSUCCESS             1
#define DECR_FAIL_THR         10

#ifdef INT_WPS_SUPP
typedef enum
{
    WPS_MODE_REGISTRAR = 0,
    WPS_MODE_ENROLLEE  = 1
} WPS_MODE_T;
#endif /* INT_WPS_SUPP */

/*****************************************************************************/
/* External Variables                                                        */
/*****************************************************************************/
extern UWORD8 g_ht_enable;
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function sets the Short Guard Interval */
INLINE void set_short_gi_enable(UWORD8 val)
{
#ifdef MAC_802_11N
    /* Input Sanity Check */
    if(val > 1)
        return;

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.shortgi = val;
#else  /* MAC_HW_UNIT_TEST_MODE */
    if(val == 1)
        mset_ShortGIOptionInTwentyEnabled(TV_TRUE);
    else
        mset_ShortGIOptionInTwentyEnabled(TV_FALSE);

#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */
}

/* This function returns the Short Guard Interval */
INLINE UWORD8 get_short_gi_enable(void)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
#ifdef MAC_HW_UNIT_TEST_MODE
    retval = g_test_params.shortgi;
#else  /* MAC_HW_UNIT_TEST_MODE */
    if(mget_ShortGIOptionInTwentyEnabled() == TV_TRUE)
        retval = 1;


#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */

    return retval;
}


/* This function sets the Immediate Block ACK */
INLINE void set_immediate_block_ack_enable(UWORD8 val)
{
#ifdef MAC_802_11N
    /* Input Sanity Check */
    if(val > 1)
        return;

    if(val == 1)
        mset_ImmediateBlockAckOptionImplemented(TV_TRUE);
    else
        mset_ImmediateBlockAckOptionImplemented(TV_FALSE);
#endif /* MAC_802_11N */
}

/* This function returns the Immediate Block ACKl */
INLINE UWORD8 get_immediate_block_ack_enable(void)
{
#ifdef MAC_802_11N
    if(mget_ImmediateBlockAckOptionImplemented() == TV_TRUE)
        return 1;
#endif /* MAC_802_11N */

    return 0;
}

/*****************************************************************************/
/* 802.11e  Related Prot-ifcation functions                                  */
/*****************************************************************************/

/* Return the value of the WMM enabled flag */
INLINE BOOL_T get_wmm_enabled(void)
{
    BOOL_T retval = BFALSE;
#ifdef MAC_WMM
    retval = g_wmm_enabled;
#endif /* MAC_WMM */
    return retval;
}

/* Set the WMM enabled flag */
INLINE void set_wmm_enabled(UWORD8 val)
{
#ifdef MAC_WMM
    if(val == 1)
        g_wmm_enabled = BTRUE;
    else
        g_wmm_enabled = BFALSE;
#endif /* MAC_WMM */
}

/* Returns TRUE if QoS is enabled in MAC */
INLINE UWORD8 get_qos_enable(void)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return 1;
#endif /* MAC_WMM */

    return 0;
}

/* Update MIB related to QoS according to the argument value. Currently only */
/* WMM quality of service is allowed.                                        */
INLINE void set_qos_enable(UWORD8 val)
{
#ifdef MAC_WMM
    if(val == 1) /* Enable WMM */
    {
        set_wmm_enabled(BTRUE);
        mset_QoSOptionImplemented(TV_FALSE);
    }
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    else if(val ==0)  /* Disable QoS & WMM */
    {
        set_wmm_enabled(BFALSE);
        mset_QoSOptionImplemented(TV_FALSE);
    }
    else
    {
		PRINTD("Warning: Invalid value %d entered for Qos\n",val);
	}
#endif /* MAC_WMM */
}

INLINE UWORD8 get_ht_enable(void)
{
    UWORD8 retval = 0;

	/*leon liu modified, softap using 11bg*/
	#ifdef BSS_ACCESS_POINT_MODE //ap not supprot 11n ,by chenq add 2013-01-13
	//#ifndef TROUT2_WIFI_IC  // Trout2 IC support 11n, zhangym 2013-04-11
		return 0;
	//#endif
	#endif

	//chenq add a marco 2013-01-07
	#ifdef TROUT_WIFI_EVB
	   return 0;	//modify by dumy  for b/g test.
	#endif

	//chenq add for wapi 2012-09-26
	#ifdef MAC_WAPI_SUPP
	if( mget_wapi_enable() == TV_TRUE )
	{
		return 0;
	}
	#endif
	
#ifdef MAC_802_11N
    retval = g_ht_enable;
#endif /* MAC_802_11N */

    return retval;
}

INLINE void set_ht_enable(UWORD8 val)
{
#ifdef MAC_802_11N
    g_ht_enable = val;
#endif /* MAC_802_11N */
}

/* Returns TRUE if HT Option is enabled in MAC */
INLINE UWORD8 get_ht_mib_enable(void)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
    if(mget_HighThroughputOptionImplemented() == TV_TRUE)
        retval = 1;
#endif /* MAC_802_11N */

    return retval;
}

/* Update MIB related to HT according to the argument value */
INLINE void set_ht_mib_enable(UWORD8 val)
{
#ifdef MAC_802_11N
    /* Input Sanity Check */
    if(val > 1)
        return;

    if(val == 0)
    {
        set_ht_enable(0);
        mset_HighThroughputOptionImplemented(TV_FALSE);
    }
    else
    {
        set_ht_enable(1);
        mset_HighThroughputOptionImplemented(TV_TRUE);
    }
#endif /* MAC_802_11N */
}

INLINE BOOL_T is_ccmp_enabled(void)
{
    UWORD8 i;
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        for(i = 0; i < PAIRWISE_CIPHER_SUITES; i++)
        {
            /* If the Entry in the MIB is enabled, increment the counter */
            if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
                dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE)
            {
                if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
                    dot11RSNAConfigPairwiseCipher == 0x04)
                {
                    return BTRUE;
                }
            }
        }
    }

    return BFALSE;
}

/* Get the MAC header length depending on the protocol used */
INLINE UWORD8 get_mac_hdr_len(UWORD8 *msa)
{
    UWORD8 mac_hdr_len = MAC_HDR_LEN;

#ifdef MAC_WMM
    /* The MAC Header len is 26 only when in QOD Data frames */
    if((is_qos_bit_set(msa) == BTRUE) && (get_type(msa) == DATA_BASICTYPE))
        mac_hdr_len += QOS_CTRL_FIELD_LEN;
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    if(BTRUE == is_ht_frame(msa))
        mac_hdr_len += HT_CTRL_FIELD_LEN;
#endif /* MAC_802_11N */
    return mac_hdr_len;
}

/* Update the protocol dependent table */
INLINE void update_qos_table_prot(UWORD8 sta_index, UWORD8 *msa,
                                  UWORD16 rx_len, UWORD16 ie_offset)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        update_table_wmm(sta_index, msa, rx_len, ie_offset);
        mset_AssociatedStationCount(mget_AssociatedStationCount() + 1);
    }
#endif /* MAC_WMM */
}

/* Returns the number of Qs based on the protocol */
INLINE UWORD16 get_no_tx_queues(void)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        return NUM_MAX_EDCA_Q;
    }
#endif /* MAC_WMM */

    return 3;
}

/* Returns the maximum queue limit based on the protocol */
INLINE UWORD8 get_max_txq_size(UWORD8 q_num)
{
#ifdef MAC_WMM

    return MAX_EDCA_Q_SIZE;

#else /* MAC_WMM */

    return MAX_Q_SIZE;

#endif /* MAC_WMM */
}

/* For 11e protocol if QoS is enabled, this function returns the logical     */
/* queue number mapped to the given TID. If there is no QoS (or 11e) the     */
/* given TID is mapped accordingly. Note that the STA index argument is      */
/* ignored for Station mode.                                                 */
INLINE UWORD16 get_txq_num(UWORD8 priority)
{
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        return get_txq_num_11e(priority);
#endif /* MAC_WMM */

    if(priority == HIGH_PRIORITY)
        return HIGH_PRI_Q;

    return NORMAL_PRI_Q;
}

/* Initialize the queue interface table based on the mode of operation */
INLINE void init_qif_table(void)
{
	/* Initialize the a/b/g queue related information common to both STA and */
    /* AP mode (high priority and normal priority queue.                     */
    g_qif_table[HIGH_PRI_Q].addr       = MAC_EDCA_PRI_HP_Q_PTR;
    g_qif_table[HIGH_PRI_Q].lock_bit   = REGBIT7;
    g_qif_table[NORMAL_PRI_Q].addr     = MAC_EDCA_PRI_VO_Q_PTR;
    g_qif_table[NORMAL_PRI_Q].lock_bit = REGBIT11;
    g_qif_table[CF_PRI_Q].addr         = MAC_EDCA_PRI_CF_Q_PTR;
    g_qif_table[CF_PRI_Q].lock_bit     = REGBIT21;

#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        /* Depending on the protocol in use the queue interface table is     */
        /* updated based on the mode of operation.                           */
        g_qif_table[AC_VO_Q].addr     = MAC_EDCA_PRI_VO_Q_PTR;
        g_qif_table[AC_VI_Q].addr     = MAC_EDCA_PRI_VI_Q_PTR;
        g_qif_table[AC_BE_Q].addr     = MAC_EDCA_PRI_BE_Q_PTR;
        g_qif_table[AC_BK_Q].addr     = MAC_EDCA_PRI_BK_Q_PTR;

        g_qif_table[AC_VO_Q].lock_bit = REGBIT11;
        g_qif_table[AC_VI_Q].lock_bit = REGBIT10;
        g_qif_table[AC_BE_Q].lock_bit = REGBIT9;
        g_qif_table[AC_BK_Q].lock_bit = REGBIT8;
    }
#endif /* MAC_WMM */
}

/* Initialize the MIBs depending on protocols */
INLINE void init_mib(void)
{
    /* Initialize the common mib */
    initialize_mac_mib();
    initialize_private_mib();

#ifdef MAC_WMM
    initialize_mac_mib_11e();
#endif /* MAC_WMM */

#ifdef MAC_802_11I
    initialize_mac_mib_11i();
#endif /* MAC_802_11I */

#ifdef MAC_802_11H
    initialize_mac_mib_11h();
    initialize_private_mib_11h();
#endif /* MAC_802_11H */

#ifdef MAC_802_11N
    initialize_mac_mib_11n();
#endif /* MAC_802_11N */

#ifdef MAC_P2P
    initialize_private_mib_p2p();
#endif /* MAC_P2P */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
	initialize_mac_mib_11d();
#endif /* MAC_MULTIDOMAIN */

}

/* This function checks the MAC header for QoS control field and returns     */
/* the TID. By default it returns 0                                          */
INLINE UWORD8 get_priority_value(UWORD8 *msa)
{
#ifdef MAC_WMM
    if(is_qos_bit_set(msa) == BTRUE)
    {
        return get_tid_value(msa);
    }
#endif /* MAC_WMM */

    return 0;
}

/* This function returns the ack policy used by the packet. If its for group */
/* address, then it will be broadcast NO_ACK type, other wise NORMAL_ACK in  */
/* case of 11a/b/g. If 11e is enabled, then it searches for QoS Control field*/
/* values                                                                    */
INLINE UWORD8 get_ack_policy(UWORD8 *msa, UWORD8 *da)
{
    if(is_group(da) == BTRUE)
        return BCAST_NO_ACK;

#ifdef MAC_WMM
    if(is_qos_bit_set(msa) == BTRUE)
    {
        return get_ack_policy_11e(msa);
    }
#endif /* MAC_WMM */

    return g_ack_policy;
}

/* This function assigns the priority to packets received from higher layer  */
/* if 11e is enabled. Otherwise it will return AC_VO as default.             */
INLINE UWORD8 get_priority(UWORD8* data, UWORD16 eth_type)
{
    UWORD8 retval = (UWORD8)NORMAL_PRIORITY;
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        retval = classify_msdu(data, eth_type);
    else
        retval = (UWORD8)PRIORITY_3;
#endif /* MAC_WMM */

    return retval;
}

/* The function returns either TRUE or FALSE depending on Destination STAs   */
/* capability to support QoS.                                                */
INLINE BOOL_T is_qos_required(UWORD8 sta_index)
{
	//return BFALSE;	//add by chengwg, for test...
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
    {
        if(is_dst_wmm_capable(sta_index) == BTRUE)
            return BTRUE;
    }
#endif /* MAC_WMM */

    /* In case 802.11e protocol is not defined, QoS is never required */
    return BFALSE;
}


INLINE void init_prot(void)
{
#ifdef MAC_WMM
#ifdef IBSS_BSS_STATION_MODE //chenq add for reconnect debug! 0723
	set_wmm_enabled(BTRUE);
#endif
    if(get_wmm_enabled() == BTRUE)
    {
        initialize_wmm();
    }
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    if(mget_HighThroughputOptionImplemented() == TV_TRUE)
    {
        initialize_11n();
    }
#endif /* MAC_802_11N */

#ifdef MAC_802_11H
    /* Channel management should be turned on if 11h is enabled. */
    if(mget_enableDFS() == BTRUE)
    {
        mset_SpectrumManagementImplemented(TV_TRUE);
    }
#endif /* MAC_802_11H */
}

/* This function extracts the class of received frame as defined in 802.11   */
/* Returns the value of the class of the received frame                      */
INLINE UWORD8 get_prot_frame_class(UWORD8* header)
{
    UWORD8 retval = (UWORD8)CLASS1_FRAME_TYPE;
#ifdef MAC_WMM
    retval = get_frame_class_11e(header);
#endif /* MAC_WMM */
    return retval;
}

/* MIB Updates according to protocol */
INLINE void update_tx_mib_prot(UWORD8 *msa, UWORD8 rc, UWORD8 cf, UWORD8 rs,
                               UWORD8 fn, UWORD16 nb, BOOL_T is_suc)
{
#ifdef MAC_WMM
    /* For non-QoS frames the QoS MIB need not be updated */
    if(is_qos_bit_set(msa) == BFALSE)
        return;

    /* Update the QoS MIB for the appropriate TID */
    update_tx_mib_11e(get_tid_value(msa), rc, cf, rs, fn, is_suc);
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    /* Update the HT MIB parameters */
    update_tx_mib_11n(msa, rc, nb, is_suc);
#endif /* MAC_802_11N */
}

/* Function is called if action request from higher layers is received. */
INLINE void set_action_request(UWORD8 *req)
{
#ifdef MAC_802_11N
#ifndef MAC_HW_UNIT_TEST_MODE
    mlme_action_req(&g_mac, req);
#else /* MAC_HW_UNIT_TEST_MODE */
    hut_action_req(req);
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */
}

/* Function is called if action request from higher layers is received. */
INLINE UWORD8* get_action_request(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_802_11N
#ifndef MAC_HW_UNIT_TEST_MODE
    retval =  handle_action_req_query();
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */

    return retval;
}

/* This function gets the current AC parameters configured at the STA */
INLINE UWORD8* get_sta_ac_params_prot(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_WMM
   if(get_wmm_enabled() == BTRUE)
   {
       retval = (UWORD8 *)get_wmm_sta_ac_params_config();
   }
#endif /* MAC_WMM */

    return retval;
}
/*****************************************************************************/
/* 802.11 i Related Prot-ifcation functions                                  */
/*****************************************************************************/

/* This function checks if the cipher type for a received frame with WEP bit */
/* set is valid and updates relevant debug statistics.                       */
INLINE BOOL_T is_valid_cipher_type(UWORD8 ct)
{
#ifdef DEBUG_MODE
    g_mac_stats.pewrxenc++;
#endif /* DEBUG_MODE */

    switch(ct)
    {
    case WEP40:
    {
#ifdef DEBUG_MODE
        g_mac_stats.pewrxwep40++;
#endif /* DEBUG_MODE */
    }
    break;
    case WEP104:
    {
#ifdef DEBUG_MODE
        g_mac_stats.pewrxwep104++;
#endif /* DEBUG_MODE */
    }
    break;
#ifdef MAC_802_11I
    case TKIP:
    {
#ifdef DEBUG_MODE
        g_mac_stats.pewrxtkip++;
#endif /* DEBUG_MODE */
    }
    break;
    case CCMP:
    {
#ifdef DEBUG_MODE
        g_mac_stats.pewrxccmp++;
#endif /* DEBUG_MODE */
    }
    break;
#endif /* MAC_802_11I */
    default: /* Exception case: Not a valid cypher type */
    {
        return BFALSE;
    }
    }

    return BTRUE;
}

/* This function determines if the WEP is enabled */
INLINE BOOL_T is_wep_enabled(void)
{
#ifdef MAC_802_11I
    if((mget_RSNAEnabled() == TV_FALSE) && (mget_PrivacyInvoked() == TV_TRUE))
    {
        return BTRUE;
    }
#else /* MAC_802_11I */
    if(mget_PrivacyInvoked() == TV_TRUE)
    {
        return BTRUE;
    }
#endif /* MAC_802_11I */

    return BFALSE;
}

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
/* This function determines if the wapi is enabled */
INLINE BOOL_T is_wapi_enabled(void)
{
    if(mget_wapi_enable() == TV_TRUE)
    {
        return BTRUE;
    }
	else 
	{
    	return BFALSE;
	}	
}
#endif

INLINE void append_sec_param(CIPHER_T ct, wlan_tx_req_t *wlan_tx_req,
                UWORD8 *tx_info, BOOL_T is_qos)
{
#ifdef MAC_802_11I
    append_11i_param(ct, wlan_tx_req, tx_info, is_qos);
#endif /* MAC_802_11I */

    return;
}

/* This function returns the length of the security header for each cipher   */
/* type.                                                                     */
INLINE UWORD8 get_sec_header_len(CIPHER_T ct)
{
    switch(ct)
    {
        case WEP40:
        case WEP104:
        {
            return WEP_HDR_LEN;
        }
#ifdef MAC_802_11I
        case CCMP:
        {
            return CCMP_HDR_LEN;
        }
        case TKIP:
        {
            return TKIP_HDR_LEN;
        }
#endif /* MAC_802_11I */
        default:
        {
            return 0;
        }
    }
}

/* This function returns the length of the security trailer for each cipher  */
/* type.                                                                     */
INLINE UWORD8 get_sec_trailer_len(CIPHER_T ct)
{
    UWORD8 retval = 0;

    switch(ct)
    {
    case WEP40:
    case WEP104:
        retval = WEP_IV_LEN;
    break;
#ifdef MAC_802_11I
    case TKIP:
        retval = TKIP_MIC_LEN + TKIP_ICV_LEN;
    break;
    case CCMP:
        retval = CCMP_MIC_LEN;
    break;
#endif /* MAC_802_11I */
    default:
    break;
    }

    return retval;
}

INLINE BOOL_T is_this_sec_hs_pkt(UWORD8 *msa, CIPHER_T  ct)
{
#ifdef MAC_802_11I
    UWORD8 mac_hdr_len = get_mac_hdr_len(msa);
    UWORD8 offset      = get_sec_header_len(ct);
    UWORD8 *data_ptr   = 0;

    /* If QOS is Enabled, then the H/w will be offsetting two extra  */
    /* bytes after the header to make it word aligned                */
    mac_hdr_len = (mac_hdr_len == MAC_HDR_LEN)? MAC_HDR_LEN :
                                                (MAC_HDR_LEN + 4);

    /* Start of data is end of mac header offseted by the security header */
    data_ptr = msa + mac_hdr_len + offset;

    if(mget_RSNAEnabled() == TV_TRUE)
    {
        if((data_ptr[6] == 0x88) && (data_ptr[7] == 0x8E))
        {
            return BTRUE;
        }
    }
#endif /* MAC_802_11I */

    return BFALSE;
}

/* Adjust length and data offset of the 11i protected received packet */
/* Returns the length of the frame header.                            */
INLINE UWORD16 modify_frame_length(CIPHER_T ct, UWORD8 *msa, UWORD16 *rx_len,
                                  UWORD16 *data_len)
{
    UWORD8 offset      = 0;
    UWORD8 mac_hdr_len = get_mac_hdr_len(msa);

    switch(ct)
    {
        case WEP40:
        case WEP104:
        {
            adjust_wep_headers(ct, rx_len, &offset);
            break;
        }
#ifdef MAC_802_11I
        case CCMP:
        case TKIP:
        {
            adjust_11i_headers(msa, ct, rx_len, &offset);
        }
        break;
#endif /* MAC_802_11I */
        default:
        {
            offset = 0;
        }
        break;
    }

    /* Start of data is end of mac header offseted by the security header */
    *data_len = *rx_len - mac_hdr_len - FCS_LEN;

    /* If MAC header length is not a multiple of 4 (QOS is Enabled), then  */
    /* the H/w will be offsetting two extra bytes after the header to make */
    /* it word aligned                                                     */
    if(mac_hdr_len & 3)
        mac_hdr_len += 2;

    return (mac_hdr_len + offset);
}

/* Adjust length and data offset of the 11i protected received packet */
INLINE void copy_sec_trailer(UWORD8 *dst, UWORD8 *src, CIPHER_T ct)
{
    switch(ct)
    {
#ifdef MAC_802_11I
        case TKIP:
        {
            copy_11i_trailer(dst, src);
        }
        break;
#endif /* MAC_802_11I */
        default:
        {
        }
        break;
    }
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE void set_int_supp_mode_prot(UWORD8 val)
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

// 20120709 caisf add, merged ittiam mac v1.2 code
/* This function returns the internal supplicant info */
INLINE UWORD8 get_int_supp_mode_prot(void)
{
#ifndef SUPP_11I
    return 0;
#endif /* SUPP_11I */

    return g_int_supp_enable;
}


/* Enable the security for the wireless entity */
INLINE void enable_security(void)
{

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {

        /* Initialize RSNA */
        initialize_rsna();

        /* If PSK is enabled install the PSK/PMK */
        compute_psk();

        /* Initialize the security authentication policy */
        init_sec_auth();
    }

#endif /* MAC_802_11I */

    if(is_wep_enabled() == BTRUE)
    {
        /* If RSNA is disabled but privacy is invoked, initialize WEP */
        init_wep();
    }

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
    if(is_wapi_enabled() == BTRUE)
    {
		init_wapi();
	}
#endif
}

INLINE void set_RSNAConfigNumberOfPTKSAReplayCounters_prot(void)
{
#ifdef MAC_802_11I
#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
{
        /* Set the number of PTKSA replay counters per association to 16 */
        /* (No.of TID's 16) by setting the value of RSNA Config Number   */
        /* Of PTKSA Replay Counter to 3                                  */

		//chenq mod for test 2 => 0 
        //mset_RSNAConfigNumberOfPTKSAReplayCounters(2);
        mset_RSNAConfigNumberOfPTKSAReplayCounters(0);
        return;
    }
#endif /* MAC_WMM */
    /* Set the number of PTKSA replay counters per association to 1  */
    /* (No.of TID's 1) by setting the value of RSNA Config Number    */
    /* Of PTKSA Replay Counter to 0                                  */
    mset_RSNAConfigNumberOfPTKSAReplayCounters(0);
#endif /* MAC_802_11I */
}

INLINE void set_RSNAConfigNumberOfGTKSAReplayCounters_prot(void)
{
#ifdef MAC_802_11I
    /* Set the number of GTKSA replay counters per association to 1  */
    /* (No.of TID's 1) by setting the value of RSNA Config Number    */
    /* Of PTKSA Replay Counter to 0                                  */
    mset_RSNAConfigNumberOfGTKSAReplayCounters(0);
#endif /* MAC_802_11I */
}

/* Set the authentication type */
INLINE void set_auth_type_prot(UWORD8 val)
{
    if(mget_PrivacyInvoked() == TV_TRUE)
    {
        mset_auth_type((val & (BIT0 | BIT1)) - 1);
    }

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If BIT1 of the value is set, set 802.1x Authentication */
        /* Else set the Authentication policy to PSK              */
        if(val & BIT2)
        {
            mset_RSNAConfigAuthenticationSuite(0x1, 0);
        }
        else
        {
            mset_RSNAConfigAuthenticationSuite(0x2, 0);
        }
        /* If 11i is enabled, override the setting to the open type */
        mset_auth_type(0);
    }
#endif /* MAC_802_11I */
}

/* If WEP is enabled, get the authentication type, otherwise, return 0 */
INLINE UWORD8 get_auth_type_prot(void)
{
    UWORD8 ret_val = mget_auth_type() + 1;

#ifdef MAC_802_11I
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        /* If 11i is enabled, check if 1x is enabled */
        if(check_auth_policy(0x1) == BTRUE)
        {
            ret_val |= 0x04;
        }
    }
#endif /* MAC_802_11I */

    return ret_val;
}

/* Set the RSNA PSK Pass Phrase */
INLINE void set_RSNAConfigPSKPassPhrase_prot(UWORD8* val)
{
#ifdef MAC_802_11I
    UWORD8 len = strlen((const WORD8*)val);

    mset_RSNAConfigPSKPassPhrase(val, len);
#endif /* MAC_802_11I */
}

INLINE UWORD8* get_RSNAConfigPSKPassPhrase_prot(void)
{
#ifdef MAC_802_11I
    return mget_RSNAConfigPSKPassPhrase();
#else /* MAC_802_11I */
    return NULL;
#endif /* MAC_802_11I */

}

// 20120709 caisf add, merged ittiam mac v1.2 code
/* Set the RSNA PSK  */
INLINE void set_RSNAConfigPSKValue(UWORD8* val)
{
#ifdef MAC_802_11I
   mset_RSNAConfigPSKValue(val);
#endif /* MAC_802_11I */
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE UWORD8* get_RSNAConfigPSKValue(void)
{
#ifdef MAC_802_11I
// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    return mget_RSNAConfigPSKValue();
#else
    g_cfg_val[0] = 32;
    memcpy(g_cfg_val + 1, mget_RSNAConfigPSKValue(), g_cfg_val[0]);
    return g_cfg_val;
#endif
#else /* MAC_802_11I */
    return NULL;
#endif /* MAC_802_11I */

}



/* This function returns the Encyrption Configuration */
INLINE UWORD8 get_802_11I_mode_prot(void)
{
    UWORD8 ret_val = 0x00;

    /* If encryption is disabled, return the value zero */
    if(mget_PrivacyInvoked() == TV_FALSE)
    {
        ret_val = 0;
    }
    else
    {
        /* If encyption is enabled, set the BIT0 of the flag */
        ret_val |= BIT0;

#ifdef MAC_802_11I
        /* IF RSNA is enabled, set the RSNA/WPA/AES/TKIP BITS */
        if(mget_RSNAEnabled() == TV_TRUE)
        {
            UWORD8 grp_policy = mget_RSNAConfigGroupCipher();

            /* Set BIT3 for WPA and BIT4 for RSN */
            ret_val |= (g_mode_802_11i << 3);

            /* Set BIT5 for CCMP/AES and BIT6 for TKIP */
            switch(grp_policy)
            {
                case 0x01:
                    ret_val |= (BIT1 | BIT7);
                    if(check_pcip_policy(0x02))
                    {
                        ret_val |= BIT6;
                    }
                    if(check_pcip_policy(0x04))
                    {
                        ret_val |= BIT5;
                    }
                    break;
                case 0x05:
                    ret_val |= (BIT2 | BIT7 | BIT1);
                    if(check_pcip_policy(0x02))
                    {
                        ret_val |= BIT6;
                    }
                    if(check_pcip_policy(0x04))
                    {
                        ret_val |= BIT5;
                    }
                    break;
                default:
                    if(check_pcip_policy(0x02))
                    {
                        ret_val |= BIT6;
                    }
                    if(check_pcip_policy(0x04))
                    {
                        ret_val |= BIT5;
                    }
                    break;
            }
        }
        /* IF RSNA is disable, set the WEP Bits if enabled */
        else
        {
            /* Set BIT1 for WEP40 and BIT1-2 for WEP104 */
            ret_val |= ((g_wep_type == WEP40) ? BIT1 : (BIT1 | BIT2));
        }
#else /* MAC_802_11I */
        /* Set the WEP Bits if enabled */
        {
            /* Set BIT1 for WEP40 and BIT1-2 for WEP104 */
            ret_val |= ((g_wep_type == WEP40) ? BIT1 : (BIT1 | BIT2));
        }
#endif /* MAC_802_11I */
    }
    return ret_val;
}

/* This function sets the protection mode */
INLINE void set_802_11I_mode_prot(UWORD8 val)
{
    /* If encryption is disabled, return the value zero */
    if(val & BIT0)
    {
        /* Check if WEP is enabled */
        if((val & BIT1) && (!(val & BIT7)))
        {
            /* Enable Protection */
            mset_PrivacyInvoked(TV_TRUE);

            /* Set the WEP Type */
            g_wep_type = (val & BIT2) ? WEP104 : WEP40;

#ifdef MAC_802_11I
            /* Disable RSNA if enabled */
            mset_RSNAEnabled(TV_FALSE);
#endif /* MAC_802_11I */
        }
        else
        {
#ifdef MAC_802_11I
#ifdef IBSS_11I
            if(mget_DesiredBSSType() == INDEPENDENT)
            {
                /* Except WEP, other security modes are disabled when 11i    */
                /* IBSS code is removed from SW                              */
                /* Disable Protection */
                mset_PrivacyInvoked(TV_FALSE);

                /* Disable RSNA */
                mset_RSNAEnabled(TV_FALSE);
            }
            else
#endif /* IBSS_11I */
            {
                /* Enable Protection */
                mset_PrivacyInvoked(TV_TRUE);

                /* Disable RSNA */
                mset_RSNAEnabled(TV_TRUE);

                g_mode_802_11i = 0;

                /* Set WPA/RSNA Mode of operation */
                g_mode_802_11i |= (val & BIT3) ? WPA_802_11I  : 0;
                g_mode_802_11i |= (val & BIT4) ? RSNA_802_11I : 0;

                /* Default : Disable both the RSNA parameters */
                disable_rsna_tkip();
                disable_rsna_aes();

                /* Check if the TSN is enabled */
                if(val & BIT7)
                {
                    UWORD8 wep_type = 0;
                    UWORD8 wep_size = 0;
                    /* Set the WEP Type */
                    g_wep_type = (val & BIT2) ? WEP104 : WEP40;
                    wep_type   = (g_wep_type == WEP40) ? 0x01 : 0x05;
                    wep_size   = (g_wep_type == WEP40) ? 40 : 104;

                    /* Set Group cipher as TKIP */
                    mset_RSNAConfigGroupCipher(wep_type);
                    mset_RSNAConfigGroupCipherSize(wep_size);
                    mset_RSNAGroupCipherSelected(wep_type);

                }
                /* Set the AES/TKIP Mode of operation */
                else if(val & BIT6)
                {
                    /* Set Group cipher as TKIP */
                    mset_RSNAConfigGroupCipher(0x02);
                    mset_RSNAConfigGroupCipherSize(256);
                    mset_RSNAGroupCipherSelected(0x02);
                }
                else if(val & BIT5)
                {
                    /* Set Group cipher as CCMP */
                    mset_RSNAConfigGroupCipher(0x04);
                    mset_RSNAConfigGroupCipherSize(128);
                    mset_RSNAGroupCipherSelected(0x04);
                }

                /* We might want to broadcast AES and TKIP */
                if(val & BIT5)
                {
                    enable_rsna_aes();
                }

                if(val & BIT6)
                {
                    enable_rsna_tkip();
                }
            }
#else   /* MAC_802_11I */

            /* Disable Protection is RSNA is not Enabled */
            mset_PrivacyInvoked(TV_FALSE);
#endif /* MAC_802_11I */
        }
    }
    else
    {
        /* Disable WEP */
        mset_PrivacyInvoked(TV_FALSE);

#ifdef MAC_802_11I
        /* Disable RSNA */
        mset_RSNAEnabled(TV_FALSE);
#endif /* MAC_802_11I */
    }

	//chenq add for wapi 2012-09-19
	#ifdef MAC_WAPI_SUPP
	/*
	 * leon liu modified,when using CFG80211,
	 * do not change wapi_enable status to TV_FALSE since 
	 * itm_set_wapimode is called in trout_cfg80211_connect
	 */
	#ifndef CONFIG_CFG80211
	mset_wapi_enable(TV_FALSE);
	#endif/*CONFIG_CFG80211*/
	#endif
}

/* This function is used to add GTK */
INLINE void add_tx_gtk(UWORD8 *val)
{
#ifdef MAC_802_11I
        /*---------------------------------*/
        /* KeyID | KeyLength |   Key       */
        /*---------------------------------*/
        /*   1   |     1     |  KeyLength  */
        /*---------------------------------*/

        /*-------------------------------------*/
        /*                      key            */
        /*-------------------------------------*/
        /* Temporal Key    | Tx Micheal Key    */
        /*-------------------------------------*/
        /*    16 bytes     |      8 bytes      */
        /*-------------------------------------*/
        /* Only for IBSS Mode: TBD */
#endif /* MAC_802_11I */
}

/* Get/Set the Rekey method */
INLINE UWORD8 get_RSNAConfigGroupRekeyMethod(void)
{
#ifdef MAC_802_11I
    return mget_RSNAConfigGroupRekeyMethod();
#else /* MAC_802_11I */
    return 0;
#endif /* MAC_802_11I */
}

INLINE void set_RSNAConfigGroupRekeyMethod(UWORD8 val)
{
#ifdef MAC_802_11I
    mset_RSNAConfigGroupRekeyMethod(val);
#endif /* MAC_802_11I */
}

/* Get/Set the Rekey period */
INLINE UWORD32 get_RSNAConfigGroupRekeyTime(void)
{
#ifdef MAC_802_11I
    return mget_RSNAConfigGroupRekeyTime();
#else /* MAC_802_11I */
    return 0;
#endif /* MAC_802_11I */

}
INLINE void set_RSNAConfigGroupRekeyTime(UWORD32 val)
{
#ifdef MAC_802_11I
    mset_RSNAConfigGroupRekeyTime(val);
#endif /* MAC_802_11I */
}

/* Get/Set the Rekey packet count */
INLINE UWORD32 get_RSNAConfigGroupRekeyPackets(void)
{
#ifdef MAC_802_11I
    return mget_RSNAConfigGroupRekeyPackets();
#else /* MAC_802_11I */
    return 0;
#endif /* MAC_802_11I */

}

INLINE void set_RSNAConfigGroupRekeyPackets(UWORD32 val)
{
#ifdef MAC_802_11I
    mset_RSNAConfigGroupRekeyPackets(val);
#endif /* MAC_802_11I */
}

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/*****************************************************************************/
/*              Multi-Domain Related elements                                */
/*****************************************************************************/
/* Add Supported operating classes element */
INLINE UWORD8 set_sup_opclas_11d_elem(UWORD8 *data, UWORD16 index)
{
    UWORD8 len = 0;

#ifdef MAC_MULTIDOMAIN
    len = set_sup_op_classes_ie(data, index);
#endif /* MAC_MULTIDOMAIN */
    return len;
}

/* This function sets the Country Information Element from the saved data. */
INLINE UWORD8 set_country_info_field(UWORD8 *ie, UWORD16 index)
{
    UWORD8 ret_val = 0;
#ifdef MAC_MULTIDOMAIN
    ret_val = set_country_info_elem_frm_reg_tbl(ie, index);
#endif /* MAC_MULTIDOMAIN */

    return ret_val;
}
#endif

/*****************************************************************************/
/* 802.11n  Related Prot-ifcation functions                                  */
/*****************************************************************************/

/* This function sets the HT-Capabilities Information element */
INLINE UWORD8 set_ht_capabilities(UWORD8 *data, UWORD16 index,
                                  TYPESUBTYPE_T frame_type)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        return set_ht_cap_11n(data, index, frame_type);
    }
#endif /* MAC_802_11N */
    return 0;
}

/* This function sets the Secondary Channel Offset Information Element */
INLINE UWORD8 set_sec_channel_offset(UWORD8 *data, UWORD16 index,
                                     TYPESUBTYPE_T frame_type)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        return set_sec_choff_11n(data, index, frame_type);
    }
#endif /* MAC_802_11N */
    return 0;
}

/* This function checks whether the frame is AMSDU frame */
INLINE UWORD8 is_amsdu_frame(UWORD8 *mac_header)
{
#ifdef MAC_802_11N
    if(is_qos_bit_set(mac_header) == BTRUE)
        return is_amsdu_bit_set(mac_header);
#endif /* MAC_802_11N */

    return 0;
}

/* This functions advertises the amsdu frame by setting the appropriate */
/* field in the MAC header.                                             */
INLINE void advt_amsdu_frame(UWORD8 *mac_header)
{
#ifdef MAC_802_11N
    set_amsdu_present_bit(mac_header);
#endif /* MAC_802_11N */
}

/* Update the MAC header depending on the protocol */
INLINE UWORD8 set_mac_hdr_prot(UWORD8 *mac_hdr, UWORD8 priority,
                               UWORD8 service_class, BOOL_T qos, BOOL_T is_ht,
                               UWORD8 is_amsdu)
{
#ifdef MAC_802_11N
    if(BTRUE == qos)
    {
        UWORD8 mac_hdr_len = 0;

        mac_hdr_len = set_mac_hdr_11e(mac_hdr, priority, service_class);

        if(BTRUE == is_ht)
            mac_hdr_len += set_ht_control(mac_hdr, mac_hdr_len);

        if(1 == is_amsdu)
            advt_amsdu_frame(mac_hdr);

        return mac_hdr_len;
    }
#else /* MAC_802_11N */

#ifdef MAC_WMM
    if(BTRUE == qos)
        return set_mac_hdr_11e(mac_hdr, priority, service_class);
#endif /* MAC_WMM */

#endif /* MAC_802_11N */

    return set_mac_hdr(mac_hdr);
}

/* This function fetches the AMSDU handle for the specified connection */
/* parameters if 11n is enabled.                                     */
INLINE void *get_amsdu_handle(UWORD8 *rx_addr, UWORD8 priority_val,
                              UWORD8 q_num,void *entry, UWORD16 msdu_len,
                              UWORD8 num_frags, UWORD8 curr_tx_rate)
{
    void *retval = NULL;
#ifdef MAC_802_11N
    retval = get_amsdu_ctxt(rx_addr, priority_val, q_num, entry, msdu_len,
                          num_frags, curr_tx_rate);
#endif /* MAC_802_11N */
    return retval;
}

/* This function releases the AMSDU handle */
INLINE void free_amsdu_handle(void *amsdu_ctxt)
{
#ifdef MAC_802_11N
    free_amsdu_ctxt(amsdu_ctxt);
#endif /* MAC_802_11N */
}
/* This function adds the AMSDU frame to the transmission queue */
INLINE BOOL_T qmu_add_tx_amsdu(void *amsdu_ctxt)
{
    BOOL_T retval = BFALSE;
#ifdef MAC_802_11N
    retval = amsdu_tx(amsdu_ctxt);
#endif /* MAC_802_11N */

    return retval;
}

/* This function sets the Tx-descriptor structure within the AMSDU */
/* context structure.                                              */
INLINE void amsdu_set_tx_dscr(void *amsdu_ctxt, void *val)
{
#ifdef MAC_802_11N
        ((amsdu_ctxt_t *)amsdu_ctxt)->tx_dscr = val;
#endif /* MAC_802_11N */
}

/* This function returns the Tx-descriptor structure within the AMSDU */
/* context structure.                                                 */
INLINE void *amsdu_get_tx_dscr(void *amsdu_ctxt)
{
    void *retval = NULL;

#ifdef MAC_802_11N
    retval = ((amsdu_ctxt_t *)amsdu_ctxt)->tx_dscr;
#endif /* MAC_802_11N */
    return retval;
}

/* This function checks whether the AMSDU frame is ready for transmission */
INLINE BOOL_T is_tx_ready(void *amsdu_ctxt)
{
#ifdef MAC_802_11N
    if(NULL != amsdu_ctxt)
        return check_amsdu_ready_status(amsdu_ctxt);
#endif /* MAC_802_11N */
    return BTRUE;
}

/* This function sets the padding bytes of a sub-MSDU */
INLINE UWORD8 set_submsdu_padding(UWORD8 *submsdu_tail, UWORD16 submsdu_len)
{
    UWORD8 retval = 0;
#ifdef MAC_802_11N
    retval = set_submsdu_padding_bytes(submsdu_tail, submsdu_len);
#endif /* MAC_802_11N */
    return retval;
}

/* This function sets the header fields of a sub-MSDU */
INLINE void set_submsdu_header(UWORD8 *sub_msdu_hdr, UWORD8 *sa, UWORD8 *da,
                               UWORD16 data_len)
{
#ifdef MAC_802_11N
    set_submsdu_header_field(sub_msdu_hdr, sa, da, data_len);
#endif /* MAC_802_11N */
}

/* Function is called if action request from higher layers is received. */
INLINE void set_p_action_request(UWORD8 *req)
{
#ifdef MAC_802_11N
#ifndef MAC_HW_UNIT_TEST_MODE
    set_11n_p_action_req(req);
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */
}

/* This function sets the sub-MSDU info table in the Tx-Descriptor */
INLINE void set_submsdu_info(UWORD8 *tx_dscr, buffer_desc_t *buff_list,
                             UWORD16 num_buff, UWORD16 buff_len,
                             UWORD16 mac_hdr_len, UWORD8 *sa,
                             UWORD8 *da, void *amsdu_ctxt)
{
#ifdef MAC_802_11N
    amsdu_set_submsdu_info(tx_dscr, buff_list, num_buff, buff_len,
                           mac_hdr_len, sa, da, amsdu_ctxt);
#else /* MAC_802_11N */
    set_tx_submsdu_info(tx_dscr, buff_list, num_buff, mac_hdr_len);
#endif /* MAC_802_11N */
}

/* This function calls the correct Tx interface depending upon whether */
/* AMSDU is enabled or not.                                            */
INLINE BOOL_T tx_data_packet(UWORD8 *entry, UWORD8 *da, UWORD8 priority,
                             UWORD8 q_num, UWORD8 *tx_dscr, void *amsdu_ctxt)
{
#ifdef MAC_802_11N
    if(NULL != amsdu_ctxt)
    {
    	TX_PATH_DBG("%s: qmu add tx pkt\n", __func__);
        return qmu_add_tx_amsdu(amsdu_ctxt);
    }
#endif /* MAC_802_11N */
	TX_PATH_DBG("%s: tx msdu frame\n", __func__);
    return tx_msdu_frame(entry, da, priority, q_num, tx_dscr);
}


INLINE MSDU_PROC_STATUS_T update_msdu_info(void *wlan_rx, void *msdu,
                                           void *msdu_state)
{
#ifdef MAC_802_11N
    return deaggr_create_msdu_desc(wlan_rx, msdu, msdu_state);
#else  /* MAC_802_11N */
    return create_msdu_desc(wlan_rx, msdu);
#endif /* MAC_802_11N */
}

/* This function updates the length fields in the MSDU descriptor when */
/* the frame is extended its tail side.                                */
INLINE void update_frame_length(msdu_desc_t *frame_desc, UWORD16 len)
{
#ifdef MAC_802_11I
    /* Update the MSDU length field */
    frame_desc->data_len += len;
#endif /* MAC_802_11I */
}

/* Returns the SMPS mode for 11n MAC from the 11n MIB */
INLINE UWORD8 get_11n_smps_mode(void)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        return mget_MIMOPowerSave();
    }
#endif /* MAC_802_11N */

    return 0;
}

INLINE UWORD8 get_bss_mship_sel_val(void)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        if(get_11n_op_type() == NTYPE_MIXED)
        {
            return 0;
        }
        else
        {
            return BSS_MSHIP_SEL_VAL;
        }
    }
#endif /* MAC_802_11N */

    return 0;
}

/* This function returns the ACK policy to be used for the control frame to  */
/* be transmitted. For 11n Compressed Block ACK request this needs to be set */
/* to Compressed BAR type. Otherwise it is set to Normal ACK policy.         */
INLINE UWORD8 get_ctrl_frame_ack_policy(UWORD8 *msa)
{
#ifdef MAC_802_11N
    if(get_sub_type(msa) == BLOCKACK_REQ)
    {
        if(((msa[16] & 0x06) >> 1) == COMPRESSED_BACK)
        {
            return COMP_BAR;
        }
    }
#endif /* MAC_802_11N */

    return NORMAL_ACK;
}

/* This function sets the RA LUT index in the descriptor in 11n mode if the  */
/* current transmit rate to the STA is a HT rate, HT is enabled and an AMPDU */
/* session is present. Otherwise a value of 0 is set.                        */
INLINE void set_ht_ra_lut_index(UWORD8 *dscr, void *ht_entry, UWORD8 tid,
                                UWORD8 curr_tx_rate)
{
    UWORD8 idx = 0;

#ifdef MAC_802_11N
    if((IS_RATE_MCS(curr_tx_rate) == BTRUE) && (ht_entry != NULL))
    {
        ht_tx_struct_t *ht_tx_hdl = ((ht_struct_t *)ht_entry)->ht_tx_ptr[tid];

        if((tid < 16) && (ht_tx_hdl != NULL) && (ht_tx_hdl->ampdu_maxnum > 0))
            idx = ht_tx_hdl->ampdu_lut_idx;
    }
#endif /* MAC_802_11N */

    set_tx_dscr_ra_lut_index((UWORD32 *)dscr, idx);
}

/* This function resets the HT-State on Block-Ack Tx Session teardown */
INLINE void reset_tx_ba_state_prot(void *entry, UWORD8 tid)
{
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        reset_tx_ampdu_session(entry, tid);
    }
#endif /* MAC_802_11N */
}


/* This function deletes all protocol specific state information on MAC */
/* reset.                                                               */
INLINE void delete_mac_prot(void)
{
#ifdef WAKE_LOW_POWER_POLICY
	delete_alarm_self(&g_flow_detect_timer);
	if(g_wifi_power_mode == WIFI_LOW_POWER_MODE)	//chwg debug, 2013.12.4
	{
		printk("%s: before reset, exit low power mode!\n", __func__);
		exit_low_power_mode(BFALSE);
	}
#endif
	
#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        reset_amsdu_aggr();
        delete_all_ba_handles();
        disable_machw_ba();	//add by chengwg, 2013.03.12!
    }
#endif /* MAC_802_11N */
}

/* MIB Updates for aggregated Tx according to protocol */
INLINE void update_aggr_tx_mib(UWORD8 num)
{
#ifdef MAC_802_11N
    /* Update the Tx AMPDU count for num > 1 */
    if(num > 1)
        mset_TransmittedAMPDUCount(mget_TransmittedAMPDUCount() + 1);
#endif /* MAC_802_11N */
}

/* MIB Updates for aggregated Rx according to protocol */
INLINE void update_aggr_rx_mib(UWORD8 num)
{
#ifdef MAC_802_11N
    /* Update the Rx AMPDU count for num > 1 */
    if(num > 1)
        mset_AMPDUReceivedCount(mget_AMPDUReceivedCount() + 1);
#endif /* MAC_802_11N */
}

/* MIB Updates according to protocol */
INLINE void update_rx_mib_prot(UWORD8 *msa, UWORD16 nb)
{
#ifdef MAC_802_11N
    /* Update the HT MIB parameters */
    update_rx_mib_11n(msa, nb);
#endif /* MAC_802_11N */
}

#ifdef NO_ACTION_RESET
/* Initialize the save action request data structures */
INLINE void init_save_action_req_list(void)
{
#ifdef MAC_802_11N
    init_save_action_req_list_11n();
#endif /* MAC_802_11N */
}
#endif /* NO_ACTION_RESET */


/* This function returns the Minimum Length of frame which will be subjected */
/* to TXOP fragmentation                                                     */
INLINE UWORD16 get_txop_frag_min_len(void *entry)
{
    UWORD16 retval = 0;

#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        retval = DEFAULT_MIN_TXOP_FRAG_LENGTH;
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
        retval = get_txop_frag_min_len_11n(entry);
#endif /* MAC_802_11N */

    return retval;
}

/* This function sets the Extended Capabilities Info-Element */
INLINE UWORD8 set_ext_cap(UWORD8 *data, UWORD16 index)
{
    UWORD8 retval = 0;


    return retval;
}

/* This function enables 20/40 operation by the STA with the Secondary       */
/* Channel Offset provided in the current BSS                                */
/*****************************************************************************/
/* Note that these functions update the SW MIB elements controlling Channel  */
/* Bandwidth and also updates corresponding MAC & PHY H/w register settings. */
/* However, the corresponding RF center frequency updation is not done. The  */
/* calling function should do this separately                                */
/*****************************************************************************/
INLINE void enable_2040_operation(UWORD8 sec_chan_offset)
{
}

/* This function disables 20/40 operation by the station in the current BSS */
INLINE void disable_2040_operation(void)
{
    mset_FortyMHzOperationEnabled(TV_FALSE);
}

/* This function returns the default RF Channel Index for the System */
INLINE UWORD8 get_bss_sec_chan_offset(void)
{
    return 0;
}

/*****************************************************************************/
/* P2P related commom prot-if                                                */
/*****************************************************************************/

INLINE UWORD16 set_p2p_ie_probe_rsp(UWORD8 *data, UWORD16 index, BOOL_T is_p2p)
{
    UWORD16 retval = 0;

#ifdef MAC_P2P
    if((mget_p2p_enable() == BTRUE) && (BTRUE == is_p2p))
    {
        retval = add_p2p_ie_probe_rsp(data, index);
    }

#endif /* MAC_P2P */
    return retval;
}

INLINE UWORD16 add_p2p_grp_info_attr_prot(UWORD8 *ptr, UWORD16 index)
{
    UWORD16 retval = 0;
#ifdef MAC_P2P
    retval = add_p2p_grp_info_attr(ptr, index) ;
#endif /* MAC_P2P */
    return retval;
}

/* This funtion handles the MAC isr for start and end of an absent period */
INLINE BOOL_T mac_isr_prot(UWORD32 int_stat)
{
#ifdef MAC_P2P
#ifdef MAC_P2P_HW_SUPP
    if(int_stat & BIT16)
    {
        p2p_absent_period_isr(MISC_ABSENT_PERIOD_START);
        reset_ab_period_start_int();
        int_stat &= ~BIT16;     //Add by Hugh
        return BTRUE;
    }
    else if(int_stat & BIT17)
    {
        p2p_absent_period_isr(MISC_ABSENT_PERIOD_END);
        reset_ab_period_end_int(); 
        int_stat &= ~BIT17;    //Add by Hugh
        return BTRUE;
    }
#endif /* MAC_P2P_HW_SUPP */
#endif /* MAC_P2P */

    return BFALSE;
}

/* This function adds the WSC IE */
INLINE UWORD16 add_wsc_ie(UWORD8* frm_ptr, UWORD8 frm_type)
{
    UWORD16 ret_val = 0;

#ifdef MAC_P2P
#ifdef INT_WPS_ENR_SUPP
	if(BTRUE == mget_p2p_enable())
	{
		if (NULL != g_wps_enrollee_ptr)//wxb add
		{
			ret_val = p2p_add_wsc_ie(g_wps_enrollee_ptr, frm_ptr, frm_type);
		}
	}
#endif /* INT_WPS_ENR_SUPP */
#endif /* MAC_P2P */

    return ret_val;
}

/* This function copies the WID settings if present from MAC HW shared mem */
INLINE void get_wid_settings(void)
{
#ifdef MAC_P2P
#if 0 //caisf add for P2P
    UWORD8 *p2p_saved_settings = (UWORD8 *)SHARED_DSCR_MEM_POOL_BASE;
#else
    WORD32 read_len = 0, p2p_file_pos = 0;
    UWORD8 *tmp_str = "got it then clean it...";
    UWORD8 *p2p_saved_settings = (UWORD8 *)kmalloc(P2P_WID_CONFIG_MEM_SIZE, GFP_KERNEL);
    if (NULL == p2p_saved_settings)
    {
       TROUT_DBG4("no mem\n");
	return;
    }
    memset(p2p_saved_settings, 0, P2P_WID_CONFIG_MEM_SIZE);

    //caisf mod wid store style. 1118
    // read wid config from file
    read_len = drv_read_file(P2P_CONFIG_FILE_PATH, p2p_saved_settings,
            P2P_WID_CONFIG_MEM_SIZE-1, p2p_file_pos);
/*    if(read_len <= 0) // wxb mask
    {
        printk("%s-%d: read file:%s ERROR!\n",__FUNCTION__,__LINE__,
            P2P_CONFIG_FILE_PATH);
        kfree(p2p_saved_settings);
        return;
    }
*/
    // clean
    p2p_file_pos = 0;
    read_len = drv_write_file(P2P_CONFIG_FILE_PATH, tmp_str, 
            strlen(tmp_str)+1, p2p_file_pos);
    if(read_len <= 0)
    {
        printk("%s-%d: write file:%s ERROR!\n",__FUNCTION__,__LINE__,
            P2P_CONFIG_FILE_PATH);
        kfree(p2p_saved_settings);
        return;
    }

#endif

    memset(g_current_settings, 0, 1596);// wxb add
    if(0 == strcmp((WORD8 *)p2p_saved_settings, P2P_MAGIC_STR))
    {
        g_current_len = p2p_saved_settings[P2P_MAGIC_STR_LEN + 1] +
                        (p2p_saved_settings[P2P_MAGIC_STR_LEN + 2] << 8);
        memcpy(g_current_settings, &p2p_saved_settings[P2P_MAGIC_STR_LEN + 3],
           g_current_len);
    }

#if 1 //caisf add for P2P
    TROUT_PRINT("P2P: magic=%d",strcmp((WORD8 *)p2p_saved_settings, P2P_MAGIC_STR));

    kfree(p2p_saved_settings);

    TROUT_DBG4("P2P: p2p en=%d, p2p GO=%d",mget_p2p_enable(),mget_p2p_auto_go());
#endif
#endif /* MAC_P2P */
}

/* This function restores the WID settings during module switch and start    */
/* registrar in AP mode                                                      */
INLINE void start_p2p_prot(void)
{
#ifdef MAC_P2P
	TROUT_FUNC_ENTER;
#ifdef INT_WPS_REG_SUPP
    if(BTRUE == mget_p2p_enable())
    {
        TROUT_DBG4("Starting WPS reg\n");
        set_wps_reg_enabled(BTRUE);
        /* Set WPS Registrar implemented status also */
        set_wps_implemented_reg(BTRUE);
		// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
        if(PASS_ID_PUSHBUTTON == get_wps_pass_id_reg())
        {
            sys_start_wps_reg(PBC);
        }
        else
        {
            sys_start_wps_reg(PIN);
        }
#else
		set_wps_prot_reg(get_wps_prot_reg());
#endif
    }
#endif /* INT_WPS_REG_SUPP */
	TROUT_FUNC_EXIT;
#endif /* MAC_P2P */
}

/* This function sets the flag to indicate that a mode-switch is in progress */
INLINE void set_switch_in_progress(BOOL_T val)
{
#ifdef MAC_P2P
    g_mode_switch_in_prog = val;
#endif /* MAC_P2P */
}

/* This function checks if a mode-switch is in progress */
INLINE BOOL_T is_switch_in_progress(void)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_P2P
    ret_val = g_mode_switch_in_prog;
#endif /* MAC_P2P */

    return ret_val;
}


// 20120709 caisf add, merged ittiam mac v1.2 code
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

#endif /* PROT_IF_H */
