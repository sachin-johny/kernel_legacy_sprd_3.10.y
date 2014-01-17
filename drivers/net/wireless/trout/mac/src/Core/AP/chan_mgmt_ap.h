/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : dfs_ap.h                                             */
/*                                                                           */
/*  Description       : This file contains all structures and definitions    */
/*                      related to 802.11h DFS feature for AP mode.          */
/*                                                                           */
/*  List of Functions : create_alarms_11h_ap                                 */
/*                      delete_alarms_11h_ap                                 */
/*                      start_chnl_av_chk_timer                              */
/*                      start_bss_in_available_channel                       */
/*                      handle_chnl_av_chk_timeout_event                     */
/*                      handle_tbtt_chan_mgmt_ap                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifndef CHAN_MGMT_AP_H
#define CHAN_MGMT_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "cglobals.h"
#include "cglobals_ap.h"
#include "mib.h"
#ifdef MAC_802_11H
#include "mib_11h.h"
#include "frame_11h_ap.h"
#endif /* MAC_802_11H */
#include "management_ap.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define MIN_AUTO_CH_SCAN_TIME           (1*1000)  /*1sec per channel*/

#ifdef DEBUG_MODE
#define CHANNEL_AVAILABILITY_CHECK_TIME (1*1000) /* 1 sec for internal testing purpose */
#else /* DEBUG_MODE */
#define CHANNEL_AVAILABILITY_CHECK_TIME (60*1000) /* 60 sec as per ETSI standard */
#endif /* DEBUG_MODE */

#define CH_SWITCH_SEND_TIME_OUT_COUNT   200
#define CHSWAN_IE_LEN                   5
#define NUM_ITR_FOR_SCAN                1

/* Following constants are used for evaluating chan scan report */
#define OP_ALLOWED                      BIT0
#define SCA_ALLOWED                     BIT1
#define SCB_ALLOWED                     BIT2

#define NUM_NET_SCA                     0
#define NUM_NET_SCB                     1


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {NOT_SUPPORTED        = 0, /* Channel not supported by RF     */
              AVAILABLE_ALWAYS     = 1, /* Channel available always        */
              AVAILABLE_TO_OPERATE = 2, /* Channel available for operation */
              AVAILABLE_TO_SCAN    = 3, /* Channel available for scanning  */
              BLOCKED_DUE_TO_RADAR = 4  /* Channel not available           */
} CHNL_STATUS_T;

typedef enum {CHNL_AV_CHK_NOT_REQ   = 0, /* Availability check isn't needed   */
              CHNL_AV_CHK_IN_PROG   = 1, /* Availability check is in progress */
              CHNL_AV_CHK_COMP      = 2  /* Availability check is complete    */
} CHNL_AV_CHK_STAT_T;

typedef enum {NTWRK_STAT_NO_CHANGE = 0, /* No change in the network status  */
              NTWRK_STAT_PR_CHANGE = 1, /* Change in the primary/both chnls */
              NTWRK_STAT_SC_CHANGE = 2  /* Change in only secondary channel */
} NTWRK_STAT_CHANGE_T;

/* 11h events */
typedef enum {MISC_CH_AVAIL_CHECK_TIMEOUT   = 0x61,
} EVENT_TYPESUBTYPE_CM_T;

/****************************************************************************/
/* The following state machine is used for swicthing channel in an AP       */
/* CH_SWITCH_STG_1: CSA/ECSA are being transmitted in the becons AP is      */
/*                  still in the initial channel                            */
/* CH_SWITCH_STG_2: AP has switched to the new channel but a scan is        */
/*                  required before restarting the newtork. Scanning is in  */
/*                  progress                                                */
/* CH_SWITCH_DONE:  Channel switching is done and network is up             */
/****************************************************************************/
typedef enum {CH_SWITCH_DONE      = 0,
              CH_SWITCH_STG_1     = 1,
              CH_SWITCH_STG_2     = 2
} CH_SWITCH_STATUS_T;


typedef enum {BW_SWITCH_DONE      = 0,
              BW_SWITCH_40_TO_20  = 1,
              BW_SWITCH_20_TO_40  = 2
} BW_SWITCH_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD16           num_networks;
#ifdef MAC_802_11H
    UWORD32           dfs_ch_count;
    CHNL_STATUS_T     dfs_ch_status;
#endif /* MAC_802_11H */


} ap_ch_info_t;

typedef struct
{
	UWORD8 num_ntwrks;
	UWORD8 *bss_id_array;
} bss_id_list_t;

typedef struct {
    UWORD16 cumm_num_ntwrk[2];
    UWORD8  chan_op;
} eval_scan_res_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern UWORD8             g_ap_announced_channel;
extern UWORD8             g_ap_ch_tbl_index;
extern UWORD8             g_ap_ch_switch_cnt;
extern UWORD16            g_csa_ie_index;
extern CH_SWITCH_STATUS_T g_ap_ch_switch_status;
extern BW_SWITCH_STATUS_T g_ap_bw_switch_status;
extern ALARM_HANDLE_T     *g_ap_ch_av_chk_alarm_handle;
extern start_req_t        g_ap_saved_start_req;
extern ap_ch_info_t       g_ap_channel_list[];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void   initialize_ap_globals(void);
extern void   start_channel_availability_check(void *start_req);
extern void   wait_start_misc_event_11h_ap(mac_struct_t *mac, UWORD8 *msg);
extern UWORD8 do_channel_availability_check(BOOL_T);
extern UWORD8 select_channel_for_operation(UWORD8 *sec_chan_offset);
extern UWORD8 select_next_possible_channel(void);
extern void   attempt_new_chan(UWORD8 channel, UWORD8 sec_chan_off);
extern void   start_bss_ap(UWORD8 *msg);
extern NTWRK_STAT_CHANGE_T   eval_chan_status(void);
extern void   init_chan_mgmt_ap_globals(void);
extern void   update_chan_status_ap(UWORD8 ch_idx);
extern void store_network(UWORD8* ntwrk_bssid);
extern void handle_tbtt_chan_mgmt_ap(UWORD8 dtim_count);
extern void start_bss_in_available_channel(void *start_req);
extern UWORD32 get_ch_scan_time_ap(void);

#ifndef OS_LINUX_CSL_TYPE
extern void ch_av_chk_timeout_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void ch_av_chk_timeout_alarm_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/
/* Find the channel index from a network */
INLINE BOOL_T get_chan_idx_of_ntwk(UWORD8 *msa, UWORD16 index, UWORD16 rx_len,
                                   UWORD8* chan_index, UWORD8* ch_offset)
{
	UWORD8 freq = get_current_start_freq();
	UWORD8 ch_idx = INVALID_CHANNEL;

#ifdef MAC_802_11N
    while(index < (rx_len - FCS_LEN))
    {
        if(IHTOPERATION == msa[index])
        {
            /* In case HT-Operation Info-Element is found, then the Primary */
            /* Channel can be directly extracted from it. This will be more */
            /* reliable than the input passed to this function              */
           	ch_idx = get_ch_idx_from_num(freq, msa[index + 2]);

			if((chan_index != NULL) &&
			   (BTRUE == is_ch_valid(ch_idx)))
			 {
			   *chan_index = ch_idx;

				if(ch_offset != NULL)
					*ch_offset  = msa[index + 3] & 0x03;

               return BTRUE;
			 }
        }

        index += (IE_HDR_LEN + msa[index + 1]);
    }
#endif /* MAC_802_11N */

	if(freq == RC_START_FREQ_2)
	{
		while(index < (rx_len - FCS_LEN))
		{
			if(msa[index] == IDSPARMS)
			{
				/* Get the Channel Index corresponding to the Channel */
				/* advertised in the DS Parameter Set.                */
				ch_idx = get_ch_idx_from_num(freq, msa[index + 2]);

				 if((chan_index != NULL) &&
				   (BTRUE == is_ch_valid(ch_idx)))
				  {
					  *chan_index = ch_idx;
					  return BTRUE;
				  }
			}

			/* Increment index by length information and header */
			index += (IE_HDR_LEN + msa[index + 1]);
		}
	}

    return BFALSE;
}

/* Create all Channel Management related alarms */
INLINE void create_chan_mgmt_alarms_ap(void)
{
    g_ap_ch_av_chk_alarm_handle = create_alarm(ch_av_chk_timeout_alarm_fn, 0, NULL);
}

/* Delete all Channel Management related alarms */
INLINE void delete_chan_mgmt_alarms_ap(void)
{
	if(g_ap_ch_av_chk_alarm_handle != NULL)
	{
    	delete_alarm(&g_ap_ch_av_chk_alarm_handle);
	}
}

/* This function starts the channel availability check timer */
INLINE void start_chnl_av_chk_timer(void)
{
	/* If the alarm is not created yet, creat now */
	if(g_ap_ch_av_chk_alarm_handle == NULL)
	{
		create_chan_mgmt_alarms_ap();
	}

    /* If the alarm could not be created then error */
	if(g_ap_ch_av_chk_alarm_handle == NULL)
	{
		raise_system_error(NO_LOCAL_MEM);
    }
    /* Else stop the alarm if it was already running and restart */
    else
    {
	    stop_alarm(g_ap_ch_av_chk_alarm_handle);
    	start_alarm(g_ap_ch_av_chk_alarm_handle, get_ch_scan_time_ap());
	}
}

/* This function updates the Channel Switch Announcement Information Element */
INLINE void set_csa_info_field(UWORD8 *csa, UWORD8 index, UWORD8 new_ch)
{
    /*  Channel Switch Announcement Information Element Format               */
    /* --------------------------------------------------------------------- */
    /* | Element ID | Length | Chnl Switch Mode | New Chnl | Ch Switch Cnt | */
    /* --------------------------------------------------------------------- */
    /* | 1          | 1      | 1                | 1        | 1             | */
    /* --------------------------------------------------------------------- */

    /* Element ID */
    csa[index] = ICHSWANNOUNC;

    /* Length */
    csa[index + 1] = ICHSWANNOUNC_LEN;

    /* Channel Switch mode field */
    csa[index + 2] = mget_chan_switch_mode();

    /* New channel for BSS operation */
    csa[index + 3] = get_ch_num_from_idx(get_current_start_freq(), new_ch);

    /* Channel switch count */
    /* NOTE: This can also be set to 0 always. In that case no updating is   */
    /* required at every TBTT                                                */
    csa[index + 4] = mget_chan_switch_cnt();
}

/* This function adds the Channel Switch Announcement information element in */
/* the beacon frame at the end (i.e. it is added as the last information     */
/* element in the frame).                                                    */
/* NOTE: The assumption that CSA field will be the last in the beacon frame  */
/* is critical. Any deviation from this in the future requires all functions */
/* related to add/remove CSA element from beacon to be modified accordingly  */
INLINE void add_csa_elem_in_bcn(UWORD8 new_ch)
{
    UWORD8 *curr_bcn = g_beacon_frame[g_beacon_index];

	if((curr_bcn == NULL) ||
	   (g_beacon_index > 1))
	{
		return;
	}

    /* Update the CSA info element index and beacon frame length such that   */
    /* it is added to the end of the beacon frame (as the last information   */
    /* element in the frame) after checking that the current beacon does not */
    /* contain the CSA info element                                          */
    if((g_csa_ie_index == 0) || (curr_bcn[g_csa_ie_index] != ICHSWANNOUNC))
    {
        g_csa_ie_index            = g_beacon_len - FCS_LEN;
        g_beacon_len              += (IE_HDR_LEN + ICHSWANNOUNC_LEN);
        g_tim_element_trailer_len += (IE_HDR_LEN + ICHSWANNOUNC_LEN);
    }

    /* Update the CSA information element field at the required index */
    set_csa_info_field(curr_bcn, g_csa_ie_index, new_ch);

    /* Update the beacon length in HW */
    set_machw_beacon_tx_params(g_beacon_len, get_phy_rate(get_beacon_tx_rate()));
}

/* This function updates the Channel Switch Count in the Channel Switch      */
/* Announcement information element in the beacon frame.                     */
INLINE void update_csa_elem_in_bcn(UWORD8 cnt)
{
    UWORD8 *curr_bcn = g_beacon_frame[g_beacon_index];

	if((curr_bcn == NULL) ||
	   (g_beacon_index > 1))
	{
		return;
	}

    /* Check that the current beacon has the CSA info element */
    if((g_csa_ie_index == 0) || (curr_bcn[g_csa_ie_index] != ICHSWANNOUNC))
        return;

    /* Update the channel switch count to the current value */
    curr_bcn[g_csa_ie_index + 4] = cnt;
}

/* This function removes the Channel Switch Announcement information element */
/* in the beacon frame from the end.                                         */
/* NOTE: The assumption that CSA field will be the last in the beacon frame  */
/* is critical. Any deviation from this in the future requires all functions */
/* related to add/remove CSA element from beacon to be modified accordingly  */
INLINE void remove_csa_elem_in_bcn(void)
{
    UWORD8 *curr_bcn = g_beacon_frame[g_beacon_index];

	if((curr_bcn == NULL) ||
	   (g_beacon_index > 1))
	{
		return;
	}

    /* Check that the current beacon has the CSA info element */
    if((g_csa_ie_index == 0) || (curr_bcn[g_csa_ie_index] != ICHSWANNOUNC))
        return;

    /* Reset the contents of the CSA info element to 0 */
    mem_set(&curr_bcn[g_csa_ie_index], 0, (IE_HDR_LEN + ICHSWANNOUNC_LEN));

    /* Reset the CSA info element index */
    g_csa_ie_index = 0;

    /* Update beacon frame length by removing the CSA info element length */
    g_beacon_len              -= (IE_HDR_LEN + ICHSWANNOUNC_LEN);
    g_tim_element_trailer_len -= (IE_HDR_LEN + ICHSWANNOUNC_LEN);
}

INLINE void restart_network_after_switch(void)
{
    /* Set the global to done */
    g_ap_ch_switch_status = CH_SWITCH_DONE;

    /* Start the network again */
    mset_bssid(mget_StationID());

    /* Resume HW Beacon frame transmission in the new channel */
    set_machw_tsf_beacon_tx_suspend_disable();

    /* Resume transmission in the new channel */
    set_machw_tx_resume();
}


/* This function handles the channel availability check timeout event. This  */
/* event happens on expiry of the channel availibility check timer. The      */
/* channel availability status for the channel is updated based on the radar */
/* detection status. The global channel index is then incremented and        */
/* channel availability check is again done. If the status indicates that    */
/* the check is complete, start a BSS in the available channel using the     */
/* saved global start request.                                               */
INLINE void handle_chnl_av_chk_timeout_event(void)
{
    UWORD8 status = 0;

    /* If already switching is in stage one */
    if(g_ap_ch_switch_status == CH_SWITCH_STG_1)
    {
        /* Do nothing, as already switching is initiated */
        return;
    }

    /* The channel avalabity check timer is started when switching to a new  */
    /* channel where scanning is required. Evaluate the channel state        */
    /* after the scan in the new channel and start a network if possible     */
    if(g_ap_ch_switch_status == CH_SWITCH_STG_2)
    {
        /* The channel master record is updated during the scan */
        /* Re-evalaute the channel operating condition again    */
        UWORD8 channel = 0;
        UWORD8 sec_chan_off = 0;

        channel = select_channel_for_operation(&sec_chan_off);

        if(is_ch_valid(channel) == BFALSE)
        {
            PRINTD("DFSErr: No channel available for operation3\n");

            /* No channel was found for operation */
            raise_system_error(DFS_ERROR);
            return;
        }

        /* If the channel remains the same then start the network else       */
        /* repeat the scaning process in the new channel                     */
        if(channel == mget_CurrentChannel())
        {
            restart_network_after_switch();
        }
        else
        {
            /* NOTE: This is an error condition since STA's might lose the   */
            /* link since the announced channel is not used but can't be     */
            /* helped                                                        */
            attempt_new_chan(channel, sec_chan_off);
        }

        return;
    }
    else
    {
		/* Update any protocol(11h) related status */
        update_chan_status_ap(g_ap_ch_tbl_index);

        /* Redo channel availability check */
        status = do_channel_availability_check(BFALSE);

        /* If the channel availability check is complete select an available     */
        /* channel for operation and initiate the BSS start using the saved      */
        /* start request.                                                        */
        if(status == CHNL_AV_CHK_COMP)
        {
            start_bss_in_available_channel((void *)(&g_ap_saved_start_req));
        }
    }
}

/* This funtion returns the total number of beacons present in the affected  */
/* range of channels around this channel  */
INLINE UWORD8 get_cum_networks(UWORD8 ch_idx)
{
    UWORD8 j = 0;
    UWORD8 cum_networks = 0;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_chan     = get_max_num_channel(freq);
    WORD16 affected_chan_lo = INVALID_CHANNEL;
    WORD16 affected_chan_hi = INVALID_CHANNEL;
    UWORD8 tbl_idx = INVALID_CH_TBL_INDEX;
    UWORD8 affected_ch_idx_offset = get_affected_ch_idx_offset(freq);

	/* Get the lower limit of affected channel range  */
	affected_chan_lo = ch_idx - affected_ch_idx_offset;

	if(affected_chan_lo < 0)
		affected_chan_lo = 0;

    /* Get the uppder limit of affected channel range */
	affected_chan_hi = ch_idx + affected_ch_idx_offset;

	if(affected_chan_hi >= num_chan)
		affected_chan_hi = num_chan - 1;

    /* Calculate the cumulative number of beacons in the range */
	cum_networks = 0;
	for(j = affected_chan_lo; j <= affected_chan_hi; j++)
	{
		tbl_idx = get_tbl_idx_from_ch_idx(freq, j);

		if(is_ch_tbl_idx_valid(freq, tbl_idx) == BTRUE)
		{
			cum_networks += g_ap_channel_list[tbl_idx].num_networks;
		}
	}

	return cum_networks;
}

/* This funtion returns if operation is allowed in a a given channel */
INLINE BOOL_T is_ch_op_allowed(eval_scan_res_t *chan_scan_res, UWORD8 tbl_idx)
{
    if(chan_scan_res[tbl_idx].chan_op & OP_ALLOWED)
	{
		return BTRUE;
	}
	else
	{
		return BFALSE;
	}
}
#endif /* CHAN_MGMT_AP_H */
#endif /* BSS_ACCESS_POINT_MODE */
