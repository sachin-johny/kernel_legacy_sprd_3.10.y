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
/*  File Name         : dfs_ap.c                                             */
/*                                                                           */
/*  Description       : This file contains all functions related to 802.11h  */
/*                      DFS feature for AP mode.                             */
/*                                                                           */
/*  List of Functions : initialize_channel_list_dfs_ap                       */
/*                      wait_start_misc_event_11h_ap                         */
/*                      enabled_misc_event_11h_ap                            */
/*                      select_next_possible_channel                         */
/*                      attempt_next_channel                                 */
/*                      ch_av_chk_timeout_alarm_fn                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "dfs_ap.h"
#include "management_11h.h"
#include "rf_if.h"
#include "qif.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
UWORD32            g_ap_dfs_exp_trig_cnt                   = 0;
BOOL_T             g_ap_dfs_radar_detected                 = BFALSE;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_dfs_ap_globals                                */
/*                                                                           */
/*  Description   : This function initializes the channel list and other     */
/*                  globals of DFS operation in AP mode                      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : all dfs globals                                          */
/*                                                                           */
/*  Processing    : This function checks the entire channel list for DFS AP  */
/*                  and updates the status to one of the following,          */
/*                  - NOT_SUPPORTED, if the RF does not support the channel  */
/*                  - AVAILABLE_TO_OPERATE, if the channel is not in radar   */
/*                    band and hence is available for operation.             */
/*                  - AVAILABLE_TO_SCAN, if the channel is in radar band     */
/*                    and requires channel availability check before use.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_dfs_ap_globals(void)
{
    UWORD8 i            = 0;
    UWORD8 status       = 0;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);

    if(freq != RC_START_FREQ_5)
    {
		return;
	}

    /* Initialize the channel list */
    for(i = 0; i < num_sup_chan; i++)
    {
        UWORD8 ch_idx = get_ch_idx_from_tbl_idx(freq, i);

        if(is_ch_idx_supported(freq, ch_idx) == BTRUE)
        {
			if(is_ch_in_radar_band(freq, ch_idx) == BTRUE)
				status = AVAILABLE_TO_SCAN;
			else
				status = AVAILABLE_ALWAYS;

			g_ap_channel_list[i].dfs_ch_status = status;
		}
        else
        {
            g_ap_channel_list[i].dfs_ch_status = NOT_SUPPORTED;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wait_start_misc_event_11h_ap                             */
/*                                                                           */
/*  Description   : This function handles miscellaneous events for 11h in AP */
/*                  mode in the WAIT_START state.                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC handle                             */
/*                  2) Pointer to the event message                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The miscellaneous 11h events are processed as required   */
/*                  based on the type and state.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wait_start_misc_event_11h_ap(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    if(MISC_RADAR_DETECTED == misc_event_msg->name)
    {
        /* Update radar detection global if radar is detected in this state */
        g_ap_dfs_radar_detected = BTRUE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : enabled_misc_event_11h_ap                                */
/*                                                                           */
/*  Description   : This function handles miscellaneous events for 11h in AP */
/*                  mode in the ENABLED state.                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC handle                             */
/*                  2) Pointer to the event message                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The miscellaneous 11h events are processed as required   */
/*                  based on the type and state.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void enabled_misc_event_11h_ap(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_DFS_EXPIRY_TIMER_TRIGGER:
    {
        handle_dfs_expiry_timer_trigger();
    }
    break;

    case MISC_RADAR_DETECTED:
    {
        handle_radar_detection_event();
    }

    default:
    {
        /* Do nothing */
    }
    break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : select_next_possible_channel                             */
/*                                                                           */
/*  Description   : This function selects a channel available for scanning   */
/*                  from the global DFS channel list after the current ch    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ap_channel_list                                        */
/*                                                                           */
/*  Processing    : This function checks the entire channel list for DFS AP  */
/*                  for a channel with status not set to blocked/unavailable */
/*                  The checking is started at the current channel set by    */
/*                  the user.                                                */
/*                  In case DFS is disabled the current channel set by the   */
/*                  user is assumed to be available channel.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Channel number available for operation           */
/*                          INVALID_CHANNEL in case no available channel is  */
/*                          found                                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 select_next_possible_channel(void)
{
    UWORD8 ch_idx       = 0;
    UWORD8 tbl_idx      = 0;
    UWORD8 i            = 0;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);

    /* If DFS is disabled return the current channel set by user */
    if(mget_enableDFS() == BFALSE)
    {
        return mget_CurrentChannel();
    }

    /* Set the index to the current channel configured by the user */
    ch_idx = mget_CurrentChannel();

    /* Get the table index corresponding to the current channel */
    tbl_idx = get_tbl_idx_from_ch_idx(freq, ch_idx);

	if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
	{
        return INVALID_CHANNEL;
    }

    /* Check the entire channel list till a channel is found with status     */
    /* AVAILABLE_TO_OPERATE or AVAILABLE_ALWAYS. The checking is started     */
    /* from the channel set by  the user.                                    */
    for(i = 0; i < num_sup_chan; i++)
    {
        if((g_ap_channel_list[tbl_idx].dfs_ch_status == AVAILABLE_TO_OPERATE) ||
           (g_ap_channel_list[tbl_idx].dfs_ch_status == AVAILABLE_TO_SCAN) ||
           (g_ap_channel_list[tbl_idx].dfs_ch_status == AVAILABLE_ALWAYS))
        {
            return get_ch_idx_from_tbl_idx(freq, tbl_idx);
        }

        /* Go to next channel in the list (wrap around if end is reached) */
        tbl_idx = (tbl_idx + 1) % num_sup_chan;
    }

    /* If no channel is found to be available return INVALID_CHANNEL */
    return INVALID_CHANNEL;
}



#endif /* MAC_802_11H */
#endif /* BSS_ACCESS_POINT_MODE */
