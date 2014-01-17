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
/*  List of Functions : is_ch_in_radar_band                                  */
/*                      update_channel_availability_status                   */
/*                      handle_dfs_expiry_timer_trigger                      */
/*                      dfs_timer_expiry_check                               */
/*                      handle_tbtt_11h_ap                                   */
/*                      handle_radar_detection_event                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11H

#ifndef DFS_AP_H
#define DFS_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "itypes.h"
#include "cglobals.h"
#include "mib.h"
#include "mib_11h.h"
#include "frame_11h_ap.h"
#include "management_ap.h"
#include "phy_hw_if.h"
#include "chan_mgmt_ap.h"
#include "spi_interface.h"
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define DFS_EXPIRY_TIMER_TRIGGER_INT    10        /* Expiry timer trigger in sec */
#define CH_SWITCH_SEND_TIME_OUT_COUNT   200

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern UWORD32            g_ap_dfs_exp_trig_cnt;
extern BOOL_T             g_ap_dfs_radar_detected;
extern ap_ch_info_t       g_ap_channel_list[];
extern CH_SWITCH_STATUS_T g_ap_ch_switch_status;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   initialize_dfs_ap_globals(void);
extern void   wait_start_misc_event_11h_ap(mac_struct_t *mac, UWORD8 *msg);
extern void   enabled_misc_event_11h_ap(mac_struct_t *mac, UWORD8 *msg);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/


/* This function updates the channel availibility status for the channel     */
/* given by the user based on the radar detection status. If no radar has    */
/* been detected the status is set to AVAILABLE_TO_OPERATE and the count is  */
/* set to 1. If radar has been detected the status is set to NOT_AVAILABLE   */
/* and the count is set to 0. The counts are used for checking expiry of the */
/* validity period or non-occupancy period for available or non-available    */
/* channels respectively. The values 1 and 0 are set for +/- error in the    */
/* expiry time check (i.e. a value of 1 ensures expiry happens at most by    */
/* the expiry time while 0 ensures it happens at least after some time).     */
INLINE void update_channel_availability_status(UWORD8 ch_idx)
{
    UWORD8 freq    = get_current_start_freq();
    UWORD8 tbl_idx = MAX_SUPP_CHANNEL;

    tbl_idx = get_tbl_idx_from_ch_idx(freq, ch_idx);

    if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
    {
        return;
    }

    if(g_ap_channel_list[tbl_idx].dfs_ch_status != AVAILABLE_ALWAYS)
    {
        if(g_ap_dfs_radar_detected == BFALSE)
        {
            g_ap_channel_list[tbl_idx].dfs_ch_status = AVAILABLE_TO_OPERATE;
            g_ap_channel_list[tbl_idx].dfs_ch_count  = 1;
        }
        else
        {
            g_ap_channel_list[tbl_idx].dfs_ch_status = BLOCKED_DUE_TO_RADAR;
            g_ap_channel_list[tbl_idx].dfs_ch_count  = 0;
        }

    }
    else if(g_ap_dfs_radar_detected == BTRUE)
    {
        PRINTD("11hErr: Radar detected in a Non-Radar Channel\n");

        /* Exception case: Radar is detected in a non-radar channel          */
        raise_system_error(DFS_ERROR);
    }
}


/* This function handles the radar detection event for AP in the ENABLED     */
/* state. This event happens when a radar detect interrupt is given by MAC   */
/* H/w. The status for the current channel is updated as required. The AP    */
/* then attempts to select a new channel for operation. If found it then     */
/* initiates a switch to this channel. In case no channel is available       */
/* exception handling must be done.                                          */
INLINE void handle_radar_detection_event(void)
{
    /* If a switch has been initiated already do nothing */
    if(g_ap_ch_switch_status == CH_SWITCH_STG_1)
        return;

    /* Update the current channel status */
    g_ap_dfs_radar_detected = BTRUE;
    update_channel_availability_status(mget_CurrentChannel());
    /* Reset the radar detected flag after master record is updated */
    g_ap_dfs_radar_detected = BFALSE;

    /* request for a re-evaluation of operating channel */
    eval_chan_status();

}

/* This function handles the DFS expiry timer trigger event. This event can  */
/* be triggered at regular intervals of time in any manner (using S/w or H/w */
/* timers). This trigger is used to check for various expiry events in the   */
/* channel list. It checks NOT_AVAILABLE channels for NonOccupancyPeriod     */
/* expiry and AVAILABLE_TO_OPERATE channels for ValidityPeriod expiry.       */
INLINE void handle_dfs_expiry_timer_trigger(void)
{
    UWORD8 tbl_idx      = 0;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);

    for(tbl_idx = 0; tbl_idx < num_sup_chan; tbl_idx++)
    {
        UWORD32 stat               = g_ap_channel_list[tbl_idx].dfs_ch_status;
        UWORD32 expiry_time_in_sec = 0;
        UWORD32 time_in_sec        = 0;

        if(stat == AVAILABLE_TO_OPERATE)
        {
            expiry_time_in_sec = mget_ValidityPeriod() * 60 * 60;
        }
        else if(stat == BLOCKED_DUE_TO_RADAR)
        {
            expiry_time_in_sec = mget_NonOccupancyPeriod() * 60;
        }
        else /* NOT_SUPPORTED or AVAILABLE_TO_SCAN or AVAILABLE_ALWAYS */
        {
            expiry_time_in_sec = 0; /* Indicates no expiry check required */
        }

        if(expiry_time_in_sec != 0)
        {
            /* Get the current time expired in seconds for this channel */
            time_in_sec = g_ap_channel_list[tbl_idx].dfs_ch_count *
                          DFS_EXPIRY_TIMER_TRIGGER_INT;

            /* If required time has expired, change the status to AVAILABLE_ */
            /* TO_SCAN and reset count to 0. Otherwise increment the count.  */
            if(time_in_sec >= expiry_time_in_sec)
            {
                g_ap_channel_list[tbl_idx].dfs_ch_status = AVAILABLE_TO_SCAN;
                g_ap_channel_list[tbl_idx].dfs_ch_count = 0;
            }
            else
            {
                g_ap_channel_list[tbl_idx].dfs_ch_count++;
            }
        }
    }
}

/* This function performs the DFS timer expiry check. Note that currently    */
/* the TBTT interrupts are used to handle this timing. The same function can */
/* be reused if a S/w timer is used also. Only the required timer interval   */
/* needs to be appropriately modified  in the function.                      */
INLINE void dfs_timer_expiry_check(void)
{
    UWORD32 exp_time_in_sec;

    /* Increment the DFS expiry timer trigger count */
    g_ap_dfs_exp_trig_cnt++;

    /* Update the time expired in seconds based on the beacon interval and   */
    /* the saved count.                                                      */
    exp_time_in_sec = (g_ap_dfs_exp_trig_cnt * mget_BeaconPeriod())/1000;

    /* If the trigger limit is expired, reset the count and call a function  */
    /* to handle the trigger.                                                */
    if(exp_time_in_sec >= DFS_EXPIRY_TIMER_TRIGGER_INT)
    {
        g_ap_dfs_exp_trig_cnt = 0;
        handle_dfs_expiry_timer_trigger();
    }
}

/* This function handles TBTT event for AP in the enabled state. If a        */
/* channel switch is in progress it updates the channel switch count and the */
/* beacon frame as required. In case the count has reached zero it switches  */
/* the channel.                                                              */
INLINE void handle_tbtt_11h_ap(void)
{
    /* Do DFS timer expiry check */
    dfs_timer_expiry_check();
}

INLINE void print_channel_table(void)
{
    UWORD8 tbl_idx      = 0;
    UWORD8 ch_idx       = 0;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);

    PRINTD("Idx\tChNo\tAvlbl?\tCount?\n");

    for(tbl_idx = 0; tbl_idx < num_sup_chan; tbl_idx++)
    {
        ch_idx = get_ch_idx_from_tbl_idx(freq, tbl_idx);
        PRINTK("%d\t%d\t%d\t%d\n", tbl_idx, get_ch_num_from_idx(freq, ch_idx),
                                   g_ap_channel_list[tbl_idx].dfs_ch_status,
                                   g_ap_channel_list[tbl_idx].dfs_ch_count);
    }

    //PRINTD("\nChannel Switching stage: %d\n",g_ap_dfs_ch_switch_status);
    PRINTD("MAC HW Tx Suspend Status: %d\n",is_machw_tx_suspended());

    //chenq mod
    //PRINTD("Beacon Tx Suspend Status: %d\n\n\n",((rMAC_TSF_CON & BIT2) >> 2));
    PRINTD("Beacon Tx Suspend Status: %d\n\n\n",
            ((host_read_trout_reg( (UWORD32)rMAC_TSF_CON) & BIT2) >> 2));
}

#endif /* DFS_AP_H */
#endif /* MAC_802_11H */
#endif /* BSS_ACCESS_POINT_MODE */
