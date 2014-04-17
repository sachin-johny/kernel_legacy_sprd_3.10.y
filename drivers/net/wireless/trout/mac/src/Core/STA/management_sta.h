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
/*  File Name         : management_sta.h                                     */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      structures, like mlme requests and responses, etc.   */
/*                                                                           */
/*  List of Functions : process_erp_info_sta                                 */
/*                      get_op_rates_entry_sta                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef MANAGEMENT_STA_H
#define MANAGEMENT_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "autorate.h"
#include "frame.h"
#include "frame_sta.h"
#include "maccontroller.h"
#include "management.h"
#include "mib.h"
#include "phy_hw_if.h"

#ifdef MAC_802_11I
#include "rsna_supp_km.h"
#include "rsna_auth_km.h"
#endif /* MAC_802_11I */

#ifdef MAC_WMM
#include "management_11e.h"
#endif /* MAC_WMM */

#ifdef MAC_802_11N
#include "management_11n.h"
#include "frame_11n.h"
#include "blockack.h"
#endif /* MAC_802_11N */

#ifdef MAC_P2P
#include "mgmt_p2p.h"
#endif /* MAC_P2P */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_NUM_DESCRIPTOR                     MAX_SITES_FOR_SCAN
//chenq mod to add (MAX_RATES_SUPPORTED + about 11n 5byte) byte for supp rates
//#define  SITE_SURVEY_RESULTS_ELEMENT_LENGTH     44
#define  SITE_SURVEY_RESULTS_ELEMENT_LENGTH (44 + MAX_RATES_SUPPORTED + 5)

#define COEXIST_LINK_LOSS_THRESHOLD            100
#define DEFAULT_LINK_LOSS_THRESHOLD            50	//20	 modify by chengwg.
#define MIN_LINK_LOSS_THRESHOLD                20
#define IBSS_STA_LINK_LOSS_THRESHOLD           50
#ifdef DV_SIM
#define ACTIVE_SCAN_TIME                       (20)
#else
#define ACTIVE_SCAN_TIME                       (20)//(40)//(20) //chenq add active scan time to 40
#endif
#define PASSIVE_SCAN_TIME                      1200


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {MLME_SCAN_RSP   = 0,
              MLME_START_RSP  = 1,
              MLME_JOIN_RSP   = 2,
              MLME_AUTH_RSP   = 3,
              MLME_ASOC_RSP   = 4,
              MAX_MLME_RSP_TYPE
} MLME_RSP_TYPE_T;

/* PCF modes */
typedef enum {NON_CF_POLLABLE             = 0,
              CF_POLLABLE_NOT_REQ_ON_LIST = 1,
              CF_POLLABLE_REQ_ON_LIST     = 2,
              CF_POLLABLE_NEVER_ON_LIST   = 3
} STA_PCF_MODE_T;

typedef enum {NOT_DONE               = 0,
              PARTLY_DONE            = 1,
              FULLY_DONE             = 2
} INIT_DONE_T;

#ifdef MAC_802_11I
/* CIPHER set for RSN or WPA element  */
typedef enum { CIPHER_TYPE_USE_GROUP_SET  = 0,
               CIPHER_TYPE_WEP40          = 1,
               CIPHER_TYPE_TKIP           = 2,
               CIPHER_TYPE_CCMP           = 4,
               CIPHER_TYPE_WEP104         = 5
} CIPHER_TYPE_T;
#endif  /* MAC_802_11I */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* BSS Descriptor Set structure */
typedef struct
{
UWORD8 cur_flag;
UWORD8 cnt;
WORD8 ssid[MAX_SSID_LEN];
size_t ssid_len;
} combo_stats;
typedef struct
{
    BSSTYPE_T bss_type;
    UWORD8    channel;
    UWORD8    dtim_period;
    UWORD16   beacon_period;
    UWORD16   cap_info;
    UWORD32   time_stamp_msb;
    UWORD32   time_stamp_lsb;
    UWORD32   local_time_msb;
    UWORD32   local_time_lsb;
    UWORD8    bssid[6];
    UWORD8    sa[6];
    WORD8     ssid[MAX_SSID_LEN];
    WORD8     ibss_param[IIBPARMS_LEN+IE_HDR_LEN];
    WORD8     rssi;
	//chenq add snr
	WORD8     snr;
	
    UWORD8    supp_rates[MAX_RATES_SUPPORTED + 1];

#ifdef MAC_WMM
    UWORD8    qos_cap;
    UWORD8    wmm_cap;
    UWORD8    uapsd_cap;
    UWORD8    qbss_load[QBSS_LOAD_LEN];
    UWORD8    edca_params[EDCA_PARAMS_LEN];
#endif /* MAC_WMM */

#ifdef MAC_802_11I
    BOOL_T    rsn_found;
    UWORD8    rsn_type;
    UWORD8    mode_802_11i;
    UWORD8    rsn_grp_policy;
    UWORD8    rsn_pcip_policy[3];
    UWORD8    rsn_auth_policy[3];
    UWORD8    rsn_cap[2];
    UWORD8    wpa_grp_policy;
    UWORD8    wpa_pcip_policy[3];
    UWORD8    wpa_auth_policy[3];
    UWORD8    dot11i_info;
    UWORD8    grp_policy_match;
    UWORD8    pcip_policy_match;
    UWORD8    auth_policy_match;
#endif /* MAC_802_11I */
#ifdef MAC_802_11N
    UWORD8    ht_capable;
    UWORD8    supp_chwidth;
    UWORD8    sta_chwidth;
    UWORD8    sec_ch_offset;
    UWORD8    coex_mgmt_supp;
#endif /* MAC_802_11N */

#ifdef MAC_P2P
    UWORD8    p2p_manag;
#endif /* MAC_P2P */

// 20120830 caisf add, merged ittiam mac v1.3 code
#ifdef MAC_MULTIDOMAIN
	UWORD8    reg_domain;
	UWORD8    reg_class;
	UWORD8    max_reg_tx_pow;
#endif /* MAC_MULTIDOMAIN */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

    //chenq add a wapi_found 2013-06-08
	BOOL_T    wapi_found;
	//chenq add end

	UWORD8    wapi_version[2];
	UWORD8    wapi_akm_cnt;
	UWORD8    wapi_akm[3][4];
	UWORD8    wapi_pcip_cnt;
	UWORD8    wapi_pcip_policy[3][4];
	UWORD8    wapi_grp_policy[4];
	UWORD8    wapi_cap[2];
#endif
} bss_dscr_t;

//chenq add do ap list merge logic 2013-08-28
typedef struct
{
	UWORD8      generic_ie[512];
	UWORD32     bss_tstamp;
	UWORD8      bss_actcnt;
	UWORD8      ap_active;
	/*junbinwang modify 20131028*/
	struct bss_link_dscr_t  *bss_next;
	bss_dscr_t        *bss_curr;
}bss_link_dscr_t;


/* MLME Scan.request structure */
typedef struct
{
    UWORD8     bss_type;
    SCANTYPE_T scan_type;
#ifndef COMBO_SCAN //chenq mod for combo scan 2013-03-12
    UWORD8     num_channels;
#else
	UWORD16     num_channels;
#endif
    UWORD16    probe_delay;
    UWORD16    min_channel_time;
    UWORD16    max_channel_time;
    UWORD8     bssid[6];
    WORD8      ssid[MAX_SSID_LEN];

	//chenq add for test 2013-02-05
	#ifndef COMBO_SCAN
    UWORD32    channel_list[MAX_SUPP_CHANNEL*5];
	#else
	UWORD32    channel_list[MAX_SUPP_CHANNEL* SCAN_IN_ONE_CHANNEL_CNT * (MAX_AP_COMBO_SCAN+1)];
	#endif
} scan_req_t;

/* MLME Scan response structure */
typedef struct
{
    UWORD8     num_dscr;
    UWORD8     result_code;
    bss_dscr_t *bss_dscr;
#ifdef MAC_P2P
    p2p_dev_dscr_t *p2p_dev_dscr;
    UWORD8     p2p_num_dscr;
    WORD8      match_index;
#endif /* MAC_P2P */
} scan_rsp_t;

/* MLME Join request structure */
typedef struct
{
    bss_dscr_t bss_dscr;
    UWORD16    join_timeout;
    UWORD16    probe_delay;
    UWORD8     op_rate[MAX_OPRATESET_LEN];
} join_req_t;

/* MLME Join response structure */
typedef struct
{
    UWORD8 result_code;
} join_rsp_t;

/* MLME Authentication request structure */
typedef struct
{
    UWORD8     peer_sta_addr[6];
    AUTHTYPE_T auth_type;
    UWORD16    auth_timeout;
} auth_req_t;

/* MLME Authentication response structure */
typedef struct
{
    UWORD8     peer_sta_addr[6];
    AUTHTYPE_T auth_type;
    UWORD16    result_code;
} auth_rsp_t;

/* MLME Association request structure */
typedef struct
{
    UWORD8  peer_sta_addr[6];
    UWORD16 cap_info;
    UWORD16 listen_interval;
    UWORD16 assoc_timeout;
} asoc_req_t;

/* MLME Association response structure */
typedef struct
{
    UWORD8 result_code;
} asoc_rsp_t;

/* MLME De-authentication structure */
typedef struct
{
    UWORD8      peer_sta_addr[6];
    UWORD16     result_code;
} deauth_t;

/* MLME Disassociation structure */
typedef struct
{
    UWORD8 result_code;
} disasoc_t;

/* Remote Station Entry structure. This is the element structure to which the*/
/* element pointers of the Remote Station table entries point.               */
typedef struct
{
    /* NOTE: Sta index has to be the first member of the structure otherwise */
    /* Sta index search will collapse                                        */
    UWORD8              sta_index;    /* Used for 11e and 11i/wep tables    */
    ar_stats_t          ar_stats;     /* Autorate statistics structure */
    UWORD8              init_done;
    UWORD8              tx_rate_index;
#ifdef AUTORATE_FEATURE
    UWORD8              min_rate_index; /* Minimum index of rate supported */
    UWORD8              max_rate_index; /* Maximum index of rate supported */
#endif /* AUTORATE_FEATURE */
    UWORD32             connection_cnt;    /* Aging cnt for STA                 */
    UWORD32             retry_rate_set[2]; /* Retry rate set */

    rate_t              op_rates;

#ifdef MAC_802_11I
    rsna_supp_persta_t supp_persta_buff; /* Buffer for RSNA Supp Handle */
    rsna_supp_persta_t *persta;          /* Handle to RSNA Supp FSM     */
#endif /* MAC_802_11I */

#ifdef MAC_802_11I
#ifdef IBSS_11I
    rsna_auth_persta_t auth_persta_buff; /* Buffer for RSNA Auth Handle */
    rsna_auth_persta_t *auth_persta;     /* Handle to RSNA Auth FSM     */
#endif /* IBSS_11I */
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    ba_struct_t    ba_hdl;             /* Handle to Block Ack.               */
    ht_struct_t    ht_hdl;            /* Handle to control HT Features       */
#endif /* MAC_802_11N */
} sta_entry_t;

/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/* MLME Re-association request structure */
typedef asoc_req_t reasoc_req_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern combo_stats g_combo_aplist[MAX_AP_COMBO_SCAN_LIST];
extern UWORD32    g_time_ref_count;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   update_scan_req_params(scan_req_t *scan_req);
extern void   update_join_req_params(join_req_t *join_req);
extern void   update_auth_req_params(auth_req_t *auth_req);
extern void   update_asoc_req_params(asoc_req_t *asoc_req);
extern void   update_start_req_params(start_req_t *start_req);
//chenq add a flag "snr" 
//extern void   update_scan_response(UWORD8* data, UWORD16 rx_len, WORD8 rssi);
extern void   update_scan_response(UWORD8* data, UWORD16 rx_len, WORD8 rssi,WORD8 snr);
extern void   update_bss_dscr_set(UWORD8 *msa, UWORD16 rx_len, SWORD8 rssi,
                                  UWORD8 bss_idx);
extern void   scan_channel(UWORD8 channel);
extern void   set_ibss_param(bss_dscr_t* bss_dscr, UWORD8* data);
extern void   generate_bssid(UWORD8 *addr);
extern void   init_sta_entry(sta_entry_t *se, UWORD8 *msa, UWORD16 len,
                             UWORD16 offset);
extern void   delete_sta_entry(void *element);
extern void   reset_sta_entry(sta_entry_t *se);
extern BOOL_T check_bss_capability_mac(bss_dscr_t *bss_dscr);

//chenq add for check link ap info change 2013-06-08
extern BOOL_T check_bss_capability_mac_change(bss_dscr_t *bss_dscr);
//chenq add end

extern BOOL_T filter_host_rx_frame_sta(wlan_tx_req_t *start_ptr, UWORD8* da,
                                       BOOL_T ignore_port, CIPHER_T* ct,
                                       UWORD8* sta_index, UWORD8* key_type,
                                       UWORD8** info, sta_entry_t **se);

extern BOOL_T filter_wlan_rx_frame_sta(wlan_rx_t *wlan_rx);
extern UWORD8 *get_site_survey_results_sta(void);
extern void set_join_req_sta(UWORD8 val);
extern WORD8* get_rx_power_level_sta(void);

#ifndef MAC_HW_UNIT_TEST_MODE
extern void disconnect_sta(UWORD8 val);
#endif /* MAC_HW_UNIT_TEST_MODE */

extern void check_free_sta_entry(void);
extern BOOL_T is_sta_connected(void);

extern void update_bss_dscr_rates(UWORD8 *msa,    UWORD16 len,
                                  UWORD8 bss_idx, UWORD16 offset);
extern void handle_mlme_rsp_sta(mac_struct_t *mac, UWORD8 rsp_type,
                                UWORD8 *rsp_msg);

//chenq add for check link ap info change 2013-06-08
extern BOOL_T    check_bss_capability_info_change(UWORD16 cap_info,bss_dscr_t * bss_dscr);
//chenq add end

//chenq add for auto set tx power 2013-07-29 
extern void uptate_rssi4_auto_set_tx_rx_power(void);
extern void auto_set_tx_rx_power(void);
//chenq add end
/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* This function locates and processes the ERP Information element. If auto  */
/* protection is enabled the ERP protection status is checked and required   */
/* action is taken.                                                          */
INLINE void process_erp_info_sta(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 index = TAG_PARAM_OFFSET;

    if(get_current_start_freq() != RC_START_FREQ_2)
        return;

    if(is_autoprot_enabled() == BTRUE)
    {
        while(index < (rx_len - FCS_LEN))
        {
            if(msa[index] == IERPINFO)
            {
                /* Check the ERP:Use Protection bit */
                if(msa[index + 2] & BIT1)
                {
                    /* If ERP prot is not in use, enable the same */
                    if(get_protection() != ERP_PROT)
                    {
                        set_protection(ERP_PROT);
                        set_machw_prot_control();
                    }
                }
                else
                {
                    /* If ERP prot is in use, disable the same */
                    if(get_protection() == ERP_PROT)
                    {
                        set_protection(NO_PROT);
                        set_machw_prot_control();
                    }
                }

                break;
            }
            else
            {
                /* Increment index by length information and tag header */
                index += msa[index + 1] + IE_HDR_LEN;
            }
        }
    }
}

extern int trout_is_asoc_req_wps_ie(void);
/* Check sec protocol dependent capability information fields */
INLINE BOOL_T check_bss_mac_privacy_sta(UWORD16 cap_info)
{
    /*junbinwang modify for wps 20130810*/
    if(trout_is_asoc_req_wps_ie() == 1)
         return BTRUE;
    if(mget_PrivacyInvoked() == TV_FALSE)
    {
        /* This STA doesn't have Privacy invoked but the other STA does */
        if((cap_info & PRIVACY) == PRIVACY)
        {
        //chenq add for wapi 2012-09-19
        #ifdef MAC_WAPI_SUPP
			if(mget_wapi_enable() == TV_TRUE)
			{
				printk("chenq debug check_bss_mac_privacy_sta 2\n");
				return BTRUE;
			}
			else
			{
				printk("chenq debug check_bss_mac_privacy_sta 3\n");
				return BFALSE;
			}
		#else
            return BFALSE;
		#endif
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

    return BTRUE;
}

//chenq add for check link ap info change 2013-06-08
INLINE BOOL_T check_bss_mac_privacy_sta_change(UWORD16 cap_info,bss_dscr_t * bss_dscr)
{
	int i = 0;

    if(mget_PrivacyInvoked() == TV_FALSE)
    {
        /* This STA doesn't have Privacy invoked but the other STA does */
        if((cap_info & PRIVACY) == PRIVACY)
        {
        #ifdef MAC_WAPI_SUPP
			if(mget_wapi_enable() == TV_TRUE)
			{
				;
			}
			else
			{
				return BFALSE;
			}
		#else
            return BFALSE;
		#endif
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

    #ifdef MAC_WAPI_SUPP
	if(mget_wapi_enable() == TV_TRUE)
	{
		if(bss_dscr->wapi_found == BFALSE)
		{
			return BFALSE;
		}

		if(bss_dscr->wapi_akm_cnt != mget_wapi_akm_cnt())
		{
			return BFALSE;
		}

		for( i = 0; i < mget_wapi_akm_cnt();i++)
		{
			if( memcmp(mget_wapi_akm(i),bss_dscr->wapi_akm[i],4) != 0 )
			{
				return BFALSE;
			}
		}

		if(bss_dscr->wapi_pcip_cnt != mget_wapi_pcip_cnt())
		{
			return BFALSE;
		}

		for( i = 0; i < mget_wapi_pcip_cnt();i++)
		{
			if( memcmp(mget_wapi_pcip_policy(i),bss_dscr->wapi_pcip_policy[i],4) != 0 )
			{
				return BFALSE;
			}
		}

		if( memcmp(mget_wapi_grp_policy(),bss_dscr->wapi_grp_policy,4) != 0 )
		{
				return BFALSE;
		}
			
		return BTRUE;
	}
	else
	{
		if(bss_dscr->wapi_found == BTRUE)
		{
			return BFALSE;
		}
	}
	#endif

    return BTRUE;
}
//chenq add end

//chenq add for check link ap info change 2013-06-08
INLINE BOOL_T check_bss_mac_privacy_change(UWORD16 cap_info,bss_dscr_t * bss_dscr)
{
#ifdef IBSS_BSS_STATION_MODE
    return check_bss_mac_privacy_sta_change(cap_info,bss_dscr);
#endif /* IBSS_BSS_STATION_MODE */
    return BTRUE;
}
//chenq add end

#ifdef AUTORATE_FEATURE
/* This function returns the operational rates table pointer for an entry */
INLINE rate_t *get_op_rates_entry_sta(sta_entry_t *entry)
{
    return &(entry->op_rates);
}
#endif /* AUTORATE_FEATURE */

/* This function updates the receiver address in the incoming action request */
/* automatically as required depending on the mode of operation.             */
INLINE void update_action_req_ra_sta(UWORD8 *ra)
{
    UWORD8 i = 0;

    /* For BSS STA, overwrite the receiver address to the AP address always */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        mac_addr_cpy(ra, mget_bssid());
    }
    else /* INDEPENDENT */
    {
        if(ra[0] == 0xFF)
        {
            /* For IBSS STA, setup session with the first valid Station */
            for(i = 1; i <= MAX_STA_SUPPORTED; i++)
            {
                UWORD8 *sta_addr = NULL;

                sta_addr = get_mac_addr_from_sta_id(i);

                if((sta_addr != NULL) &&
                   (mac_addr_cmp(sta_addr, mget_StationID()) == BFALSE))
                {
                    mac_addr_cpy(ra, sta_addr);

                    break;
                }
            }
        }
    }
}

/* This function updates the current time refernce counter for the station */
INLINE void update_uptime_cnt(void)
{
    g_time_ref_count++;
}

/* This function returns the current time reference counter for the station */
INLINE UWORD32 get_uptime_cnt(void)
{
    return g_time_ref_count;
}

/* This function updates the connection counter for the specified station */
INLINE void update_connection_cnt(sta_entry_t *entry)
{
    if(entry != NULL)
        entry->connection_cnt = get_uptime_cnt();
}

/* This function consolidates all the information about 11i cypher suites   */
/* into a single byte variable called dot11i_info. This will be used later  */
/* for displaying in the site survey results of configurator/supplicant     */
INLINE void update_dot11i_info(bss_dscr_t* bss_dscr, UWORD8 rsn_type)
{
#ifdef MAC_802_11I
    if((bss_dscr->rsn_found == BTRUE) &&
       ((rsn_type == IRSNELEMENT) || (rsn_type == IWPAELEMENT)))
    {
        UWORD8 count;
        UWORD8 *pcip_policy = 0;
        UWORD8 grp_policy  = 0;

        /* Set Security enabled */
        bss_dscr->dot11i_info |= BIT0;

        if(rsn_type == IRSNELEMENT)
        {
            pcip_policy = bss_dscr->rsn_pcip_policy;
            grp_policy  = bss_dscr->rsn_grp_policy;
        }
        else
        {
            pcip_policy = bss_dscr->wpa_pcip_policy;
            grp_policy  = bss_dscr->wpa_grp_policy;
        }

        /* Add Information of Group Cypher Type */
        if(grp_policy == CIPHER_TYPE_WEP40)
        {
            bss_dscr->dot11i_info |= (BIT1 | BIT7);
        }
        else if(grp_policy == CIPHER_TYPE_WEP104)
        {
            bss_dscr->dot11i_info |= (BIT2 | BIT7 | BIT1);
        }
        else if(grp_policy == CIPHER_TYPE_TKIP)
        {
            bss_dscr->dot11i_info |= BIT6;
        }
        else if(grp_policy == CIPHER_TYPE_CCMP)
        {
            bss_dscr->dot11i_info |= BIT5;
        }
        /* Add Information of Pairwise Cypher Types */
        for(count = 0; count < 3; count++)
        {
            if(pcip_policy[count] == CIPHER_TYPE_TKIP)
            {
                bss_dscr->dot11i_info |= BIT6;
            }
            else if(pcip_policy[count] == CIPHER_TYPE_CCMP)
            {
                bss_dscr->dot11i_info |= BIT5;
            }
        }
    }
#endif /* MAC_802_11I */
}

/* This function returns TRUE if both WPA/WPA2 elements are already */
/* found in this network beacon/probe response */
INLINE BOOL_T both_elements_already_found(bss_dscr_t* bss_dscr)
{
#ifdef MAC_802_11I
    if((bss_dscr->dot11i_info & BIT3) &&
       (bss_dscr->dot11i_info & BIT4))
     {
         return BTRUE;
     }
#endif /* MAC_802_11I */

    return BFALSE;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/* This function checks if the BSS type is a valid BSS type or not */
INLINE BOOL_T is_bss_type_valid(BSSTYPE_T bss_type)
{
    if((bss_type == INDEPENDENT) || (bss_type == INFRASTRUCTURE))
        return BTRUE;
    else
        return BFALSE;
}

#endif /* MANAGEMENT_STA_H */
#endif /* IBSS_BSS_STATION_MODE */
