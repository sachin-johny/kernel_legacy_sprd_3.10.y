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
/*  File Name         : ap_management_11n.h                                  */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      structures, like mlme requests and responses, etc.   */
/*                                                                           */
/*  List of Functions : detect_obss_nonht_sta                                */
/*                      set_obss_non_htsta_present                           */
/*                      get_obss_non_htsta_present                           */
/*                      get_obss_non_htsta_present_bit                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifdef MAC_802_11N

#ifndef AP_MANAGEMENT_11N_H
#define AP_MANAGEMENT_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "management_ap.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "chan_mgmt_ap.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {NO_DETECT             = 0,
              DETECT_ONLY           = 1,
              DETECT_PROTECT        = 2,
              DETECT_PROTECT_REPORT = 3,
              NUM_N_OBSS_DETECTION
} N_OBSS_DETECTION_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_num_sta_nonht_asoc;
extern UWORD8 g_num_sta_20mhz_only;
extern UWORD8 g_num_sta_ngf;
extern UWORD8 g_num_sta_no_lsig_txop;
extern UWORD8 g_obss_non_ht_sta_present;
extern BOOL_T g_40MHz_intol_bit_recd;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 check_11n_params_ap(UWORD8 *msa, UWORD16 offset, UWORD16 rx_len,
                                  asoc_entry_t *entry);
extern void   update_beacon_ht_info(void);
extern void   reset_11n_entry_ap(asoc_entry_t *ae);
extern void   process_obss_nonht_sta(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi);
extern void   ap_enabled_action_req_11n(UWORD8 *msg);
extern void   ap_enabled_rx_11n_action(UWORD8 *msg);
extern void   ap_enabled_rx_11n_control(UWORD8 *msg);
extern void   set_11n_ht_ps_params_ap(UWORD8 *tx_dscr, asoc_entry_t *ae,
                                      UWORD8 rate);
extern void   misc_11e_event_ap(mac_struct_t *mac, UWORD8 *msg);
extern void   update_sta_info_11n(asoc_entry_t *ae, UWORD8 *info);


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function locates the HT Information element and detects OBSS non-HT  */
/* STAs. The function also detects OBSS non-HT STAs if no HT Info or Cap is  */
/* found.                                                                    */
INLINE UWORD8 detect_obss_nonht_sta(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8  obss  = 0;
    UWORD8  ht    = 0;
    UWORD16 index = TAG_PARAM_OFFSET;

    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IHTOPERATION)
        {
            ht = 1;

            /* If HT Info is found set the obss flag to the OBSS non-HT STA  */
            /* present bit value                                             */
            obss = (msa[index + 4] & BIT4) >> 4;
        }
        else if(msa[index] == IHTCAP)
        {
            ht = 1;
        }

        /* Increment index by length information and tag header */
        index += msa[index + 1] + IE_HDR_LEN;
    }

    /* If no HT elements are found, set obss to 1 */
    if(ht == 0)
    {
        obss = 1;
    }

    return obss;
}

/* This function locates the HT Capability element and detects if 40 MHz Intol */
/* bit is set in the received frame */
INLINE BOOL_T is_40MHz_intol_bit_set(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 index = TAG_PARAM_OFFSET;
    UWORD16     info_elem    = 0;

    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IHTCAP)
        {
            index += IE_HDR_LEN;

            /* Parse HT Capabilities Info Field */
            info_elem = MAKE_WORD16(msa[index], msa[index + 1]);

			if(0 != (info_elem & BIT14))
			{
				return BTRUE;
			}
        }

        /* Increment index by length information and tag header */
        index += msa[index + 1] + IE_HDR_LEN;
    }

    return BFALSE;
}

/* This function sets the OBSS non-HT STA present flag */
INLINE void set_obss_non_htsta_present(UWORD8 val)
{
    g_obss_non_ht_sta_present = val;
}

/* This function returns the OBSS non-HT STA present flag */
INLINE UWORD8 get_obss_non_htsta_present(void)
{
    return g_obss_non_ht_sta_present;
}

/* This function returns the value to be set for OBSS non-HT STA present bit */
/* The bit is to be set if there are any non-HT STAs associated with the AP  */
/* or if any non-HT STAs were detected in OBSS and the 11n detection flag is */
/* set to DETECT_PROTECT_REPORT                                              */
INLINE UWORD8 get_obss_non_htsta_present_bit(void)
{
    if(g_num_sta_nonht_asoc != 0)
        return 1;

    if((g_obss_non_ht_sta_present == 1) &&
       (get_11n_obss_detection() == DETECT_PROTECT_REPORT))
       return 1;

    return 0;
}


/* This function handles PS buffering of the frames for the TIDs under */
/* BA agreement when the PS mode is changed from active to Powersave.  */
INLINE void handle_ba_powersave_ap(asoc_entry_t *ae)
{
    UWORD8 idx = 0;

    for(idx = 0; idx < NUM_TIDS; idx++)
    {
        if(ae->ba_hdl.ba_tx_ptr[idx] != NULL)
        {
            ps_buff_ba_frames(ae, &(ae->ba_hdl.ba_tx_ptr[idx]), idx);
        }
    }
}

/* This function restarts block ACK for the TIDs under BA agreement */
/* when the PS mode is changed from powersave to active.            */
INLINE void handle_ba_active_ap(asoc_entry_t *ae)
{
    UWORD8 idx = 0;

    for(idx = 0; idx < NUM_TIDS; idx++)
    {
        if(ae->ba_hdl.ba_tx_ptr[idx] != NULL)
        {
            restart_ba(&(ae->ba_hdl.ba_tx_ptr[idx]));
        }
    }
}

/* This funtion returns the total number of beacons present in the affected  */
/* range of channels around this channel  */
INLINE void get_affected_channels(UWORD8 ch_idx, UWORD8 *low_tbl_idx, UWORD8 *high_tbl_idx)
{
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_chan     = get_max_num_channel(freq);
    WORD16 affected_chan_lo = INVALID_CHANNEL;
    WORD16 affected_chan_hi = INVALID_CHANNEL;
    UWORD8 tbl_idx = INVALID_CH_TBL_INDEX;
    UWORD8 affected_ch_idx_offset = get_affected_ch_idx_offset(freq);

	/* Get the lower limit of affected channel range  */
	affected_chan_lo = ch_idx - affected_ch_idx_offset;

	tbl_idx = get_tbl_idx_from_ch_idx(freq, affected_chan_lo);

	if(is_ch_tbl_idx_valid(freq, tbl_idx) == BTRUE)
	{
		*low_tbl_idx = tbl_idx;
	}
	else
	{
		*low_tbl_idx = 0;
	}


    /* Get the uppder limit of affected channel range */
	affected_chan_hi = ch_idx + affected_ch_idx_offset;

	tbl_idx = get_tbl_idx_from_ch_idx(freq, affected_chan_hi);

	if(is_ch_tbl_idx_valid(freq, tbl_idx) == BTRUE)
	{
		*high_tbl_idx = tbl_idx;
	}
	else
	{
		*high_tbl_idx =num_chan - 1;
	}

	return;
}

#endif /* AP_MANAGEMENT_11N_H */
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */

