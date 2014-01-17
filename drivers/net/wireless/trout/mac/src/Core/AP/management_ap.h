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
/*  File Name         : management_ap.h                                      */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      structures, for AP mode of operation like hash and   */
/*                      sending deauth and disasoc messages.                 */
/*                                                                           */
/*  List of Functions : send_disasoc_frame                                   */
/*                      get_entry_tx_rate                                    */
/*                      get_op_rates_entry_ap                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef MANAGEMENT_AP_H
#define MANAGEMENT_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate.h"
#include "common.h"
#include "frame_ap.h"
#include "management.h"
#include "mib.h"
#include "queue.h"
#include "transmit.h"
#include "event_manager.h"

#ifdef MAC_802_11I
#include "rsna_auth_km.h"
#endif /* MAC_802_11I */

#ifdef MAC_WMM
#include "management_11e.h"
#include "ap_management_11e.h"
#endif /* MAC_WMM */

#ifdef MAC_802_11N
#include "management_11n.h"
#include "blockack.h"
#endif /* MAC_802_11N */

#ifdef MAC_P2P
#include "ap_mgmt_p2p.h"
#endif /* MAC_P2P */


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_LISTEN_INT          10
#define VBMAP_SIZE              256
#define MIN_TIM_LEN             4

#define PS_PKT_Q_MAX_LEN        16

#define PS_BUFF_TO              500 /* Time out for discarding a packets in  */
                                    /* PS queues in millisecs                */
#define PS_Q_QUOTA              8   /* Maximum no. of packets in 1 queue     */

#define STA_JOIN_INFO_LEN          16

#define AGING_TIMER_PERIOD_IN_SEC  60    /*  60 Seconds */
#define AGING_THR_IN_SEC           60    /*  60 Seconds */
#define ERP_AGING_THRESHOLD        10

/*****************************************************************************/
/* External Global Variable                                                  */
/*****************************************************************************/

extern UWORD8  g_num_sta_non_erp;
extern UWORD8  g_num_sta_no_short_pream;
extern UWORD8  g_num_sta_no_short_slot;
extern UWORD8  g_erp_aging_cnt;
extern ALARM_HANDLE_T *g_aging_timer;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Authentication/Association states of STAs maintained by the Access Point  */
typedef enum {AUTH_COMPLETE      = 1,   /* Authenticated                     */
              AUTH_KEY_SEQ1      = 2,   /* Shared Key Auth (Sequence 1) done */
              ASOC               = 3    /* Associated                        */
} STATIONSTATE_T;

typedef enum {MLME_START_RSP     = 0,
              MAX_MLME_RSP_TYPE  = 1
} MLME_RSP_TYPE_T;

typedef enum {ACTIVE_PS      = 0,
              POWER_SAVE     = 1
} STA_PS_STATE_T;

typedef enum {TYPE_OFFSET        = 0,
              LENGTH_OFFSET      = 1,
              DTIM_CNT_OFFSET    = 2,
              DTIM_PERIOD_OFFSET = 3,
              BMAP_CTRL_OFFSET   = 4,
              TIM_OFFSET         = 5
} OFFSET_T;

typedef enum {AID0_BIT           = 0,
              DTIM_BIT           = 1
} TIMBIT_T;

/* PCF modes */
typedef enum {NO_PC_AT_AP          = 0,
              PC_DELIVERY_ONLY     = 1,
              PC_DELIVERY_AND_POLL = 2,
              PC_MODE_RESERVED     = 3
} AP_PCF_MODE_T;

typedef enum {RE_Q_ERROR       = 0,
              NO_PKT_IN_QUEUE  = 1,
              PKT_REQUEUED     = 2,
              END_OF_QUEUE     = 3,
              PKT_NOT_REQUEUED = 4
} REQUEUE_STATUS_T;

typedef enum {NO_PKT_IN_FLUSH_QUEUE = 1,
              PKT_FLUSHED           = 2,
              END_OF_FLUSH_QUEUE    = 3,
              PKT_NOT_FLUSHED       = 4,
} PS_FLUSH_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Association element structure. This is the element structure to which the */
/* element pointers of the association table entries point.                  */
typedef struct
{
    /* NOTE: Sta index has to be the first member of the structure otherwise */
    /* Sta index search will collapse                                        */
    UWORD8         sta_index;         /* Used for 11e and 11i/wep tables     */
    UWORD8         tx_rate_index;     /* Index of current rate used          */
#ifdef AUTORATE_FEATURE
    UWORD8         min_rate_index;     /* Minimum index of rate supported    */
    UWORD8         max_rate_index;     /* Maximum index of rate supported    */
#endif /* AUTORATE_FEATURE */
    UWORD8         tx_rate_mbps;      /* Rate to communicate with STA        */
    UWORD8         short_preamble;    /* Short preamble supported by STA     */
    UWORD8         preamble_used;     /* Current preamble used for the STA   */
    UWORD8         short_slot;        /* Short Slot supported by STA         */
    STA_PS_STATE_T ps_state;          /* Power save state of the STA         */
    BOOL_T         ps_poll_rsp_qed;   /* Flag to indicate PS poll rsp is     */
                                      /* already queued and pending          */
    STATIONSTATE_T state;             /* Auth/Asoc state of the STA          */
    UWORD16        asoc_id;           /* Association ID of the STA           */
    UWORD16        listen_interval;   /* Listen Interval of the STA          */
    UWORD16        num_ps_pkt_lgcy;   /* Number of UC pkts queued in PSQ     */
    UWORD16        num_qd_pkt;        /* Number of UC pkts queued in HW      */
    UWORD32        aging_cnt;         /* Aging cnt for STA                   */
    ar_stats_t     ar_stats;          /* Auto rate statistics for the STA    */
    rate_t         op_rates;          /* Operational rates of the STA        */
    list_buff_t    ps_q_lgcy;         /* Power save queue for the STA        */
    UWORD8         ch_text[CHTXT_SIZE];/* Challenge text used for Shared     */
    ALARM_HANDLE_T *msg_3_timer;      /* Timer for shared key message # 3    */
    CIPHER_T       cipher_type;       /* Cipher type stores encryption type  */
    UWORD32        retry_rate_set[2]; /* Retry rate set                      */

#ifdef MAC_802_11I
    rsna_auth_persta_t auth_persta_buff; /* Buffer for RSNA Auth handle */
    rsna_auth_persta_t *persta;          /* Handle to RSNA Auth FSM     */
#endif /* MAC_802_11I */

#ifdef MAC_WMM
    BOOL_T         USP_in_progress;   /* Flag to indicate USP in progress    */
    UWORD16        num_USP_pkts_qed;  /* No of packet that are pending for   */
                                      /* the current USP                     */
    UWORD16        num_ps_pkt_del_ac; /* Number of packets queued for STA    */
                                      /* in delivery enabled ac queue        */
    list_buff_t    ps_q_del_ac;       /* Power save queue for delivery       */
    uapsd_status_t uapsd_status;      /* UAPSD Status of STA                 */
    /* enabled ac s of the STA           */
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    ba_struct_t    ba_hdl;             /* Handle to Block Ack.               */
    ht_struct_t    ht_hdl;             /* Handle to control HT Features      */
#endif /* MAC_802_11N */

#ifdef INT_WPS_SUPP
    BOOL_T wps_cap_sta;               /* Flag to indicate WPS Capable STA    */
    BOOL_T ver2_cap_sta;              /* WPS 2.0 Capable STA                 */
#endif /* INT_WPS_SUPP */

#ifdef MAC_P2P
    p2p_client_info_dscr_t p2p_client_dscr;
#endif /* MAC_P2P */
} asoc_entry_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    update_start_req_params(start_req_t *start_req);
extern void    reset_asoc_entry(asoc_entry_t *ae);
extern void    delete_asoc_entry(void *element);
extern UWORD16 update_asoc_entry(asoc_entry_t *ae, UWORD8 *msa,
                                 UWORD16 rx_len, BOOL_T *is_p2p);

extern BOOL_T  filter_host_rx_frame_ap(wlan_tx_req_t *, BOOL_T ignore_port,
                                       CIPHER_T* ct, UWORD8* sta_index,
                                       UWORD8* key_type, UWORD8** info,
                                       asoc_entry_t **ae);

extern BOOL_T filter_wlan_rx_frame_ap(wlan_rx_t *wlan_rx);

#ifndef OS_LINUX_CSL_TYPE
extern void    ap_aging_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void    ap_aging_timeout_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

extern void ap_aging_fn(void);
extern void unjoin_all_stations(UWORD16);
extern void unjoin_station(UWORD16 deauth_reason, UWORD8 assoc_id);
extern UWORD16 get_sta_join_info_ap(asoc_entry_t *ae, UWORD8* sta_addr,
                                    UWORD8* ptr);
extern UWORD16 get_all_sta_info_ap(UWORD8* ptr, UWORD16 max_len);
// 20120709 caisf mod, merged ittiam mac v1.2 code
//extern BOOL_T update_asoc_entry_txci(UWORD8 *tx_dscr, asoc_entry_t *ae);
extern BOOL_T update_asoc_entry_txci(UWORD8 *tx_dscr, asoc_entry_t *ae, UWORD8 *msa);
extern void process_obss_erp_info(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi);
// 20120709 caisf add, merged ittiam mac v1.2 code
extern void send_probe_request_ap(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/


/* Send the disassociation frame to destination */
INLINE void send_disasoc_frame(UWORD8 *da, UWORD16 error)
{
    UWORD8  *disasoc    = 0;
    UWORD16 disasoc_len = 0;

    /* On receiving this message the AP prepares a disassociation frame */
    disasoc = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                 MANAGEMENT_FRAME_LEN);
    if(disasoc == NULL)
    {
        return;
    }

    disasoc_len = prepare_disasoc(disasoc, da, error);

    /* MIB variables related to disassociation are updated */
    mset_DisassociateReason(error);
    mset_DisassociateStation(da);

    /* Buffer this frame in the Memory Queue for transmission */
    tx_mgmt_frame(disasoc, disasoc_len, HIGH_PRI_Q, 0);
}

/* This function returns the maximum speed supported by the destination STA. */
INLINE UWORD8 get_entry_tx_rate(asoc_entry_t *asoc_entry)
{
    /* Return the transmit rate as specified by the STA entry */
    return asoc_entry->tx_rate_mbps;
}

#ifdef AUTORATE_FEATURE
/* This function returns the operational rates table pointer for an entry */
INLINE rate_t *get_op_rates_entry_ap(asoc_entry_t *entry)
{
    return &(entry->op_rates);
}
#endif /* AUTORATE_FEATURE */

/* This function updates the receiver address in the incoming action request */
/* automatically as required depending on the mode of operation.             */
INLINE void update_action_req_ra_ap(UWORD8 *ra)
{
    /* For an AP if the first byte of the address is 0xFF it indicates that  */
    /* the address field contains the STA Index. In such a case get the      */
    /* address corresponding to this STA Index and overwrite the RA with the */
    /* same. Otherwise, leave the address field unchanged.                   */
    if(ra[0] == 0xFF)
    {
        UWORD8 sta_index = ra[1];
        UWORD8 *sta_addr = NULL;

        sta_addr = get_mac_addr_from_sta_id(sta_index);

        if(sta_addr != NULL)
        {
            /* If an entry (and thus the MAC address) corresponding to the   */
            /* given STA index is found, update the incoming address with    */
            /* the same.                                                     */
            mac_addr_cpy(ra, sta_addr);
        }
        else
        {
            /* If no entry (and thus no MAC address) corresponding to the    */
            /* given STA index is found, reset the address with all zeroes   */
            /* and display an error message.                                 */
            mem_set(ra, 0, 6);
#ifdef DEBUG_MODE
            PRINTD("Error: No STA found with given STA Index\n");
            g_mac_stats.nostafound++;
#endif /* DEBUG_MODE */
        }
    }
    else /* (ra[0] != 0xFF) */
    {
        /* Required peer address is configured by the user. Do nothing. */
    }
}

/* This function updates the beacon frame with the latest information */
INLINE void update_beacon_ap(void)
{
    UWORD8 *beacon_frame = 0;

    if((g_beacon_frame[0] != NULL) &&
       (g_beacon_frame[1] != NULL))
    {
    	/* fix bug about no beacon send out after switch sta mode by chwg 2013-02-17 */
        beacon_frame = g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
        g_beacon_len = prepare_beacon_ap(beacon_frame);
    }
}

/* Check if any 11b APs/STAs are found */
INLINE BOOL_T is_olbc_present(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 index = 0;
    UWORD8      num_rates = 0;

	/* Check for ERP protection field in the received beacon or  */
	/* check if it belongs to an 11b BSS.                        */
	index = TAG_PARAM_OFFSET;
	while(index < (rx_len - FCS_LEN))
	{
		if(get_current_start_freq() == RC_START_FREQ_2)
		{
			if(msa[index] == IERPINFO)
			{
				/* Check the protection type */
				if(msa[index + 2] & BIT1)
				{
					return BTRUE;
				}
			}
			else if(msa[index] == ISUPRATES)
			{
				num_rates += msa[index + 1];
			}
			else if(msa[index] == IEXSUPRATES)
			{
				num_rates += msa[index + 1];
			}
		}

		/* Increment index by length information and header */
		index += msa[index + 1] + IE_HDR_LEN;
	}

	/* Check if the beacon is from an 11b BSS */
	if(num_rates <= NUM_BR_PHY_802_11G_11B_1)
	{
		return BTRUE;
	}

	return BFALSE;
}

#endif /* MANAGEMENT_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
