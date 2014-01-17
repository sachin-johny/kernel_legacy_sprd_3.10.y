/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : chan_mgmt_ap.c                                       */
/*                                                                           */
/*  Description       : This file contains all functions related to channel  */
/*                      management for AP  mode.                             */
/*                                                                           */
/*  List of Functions : init_chan_mgmt_ap_globals                            */
/*                      start_channel_availability_check                     */
/*                      do_channel_availability_check                        */
/*                      select_channel_for_operation                         */
/*                      initiate_switch_to_new_channel                       */
/*                      attempt_new_chan                                     */
/*                      eval_chan_status                                     */
/*                      send_csa_frame                                       */
/*                      ch_av_chk_timeout_alarm_fn                           */
/*                      update_chan_status_ap                                */
/*                      store_network                                        */
/*                      handle_tbtt_chan_mgmt_ap                             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "chan_mgmt_ap.h"
#include "qif.h"
#include "ap_prot_if.h"
#include "phy_hw_if.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8             g_ap_announced_channel          = 0;
UWORD8             g_ap_announced_sec_chan         = 0;
UWORD8             g_ap_ch_tbl_index               = 0;
UWORD8             g_ap_scan_itr_cnt               = 0;
UWORD8             g_ap_ch_switch_cnt              = 0;
CH_SWITCH_STATUS_T g_ap_ch_switch_status           = CH_SWITCH_DONE;
BW_SWITCH_STATUS_T g_ap_bw_switch_status           = BW_SWITCH_DONE;
UWORD16            g_csa_ie_index                  = 0;
ALARM_HANDLE_T     *g_ap_ch_av_chk_alarm_handle    = 0;
start_req_t        g_ap_saved_start_req            = {{0,},};
ap_ch_info_t       g_ap_channel_list[MAX_SUPP_CHANNEL] = {{0,},};
bss_id_list_t      g_bss_id_list                   = {0,};

/*****************************************************************************/
/* Static functions                                                          */
/*****************************************************************************/
static void send_csa_frame(UWORD8 new_channel);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_chan_mgmt_ap_globals                                */
/*                                                                           */
/*  Description   : This function initializes the channel list and other     */
/*                  globals related to channel management in AP mode         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ap_announced_channel                                   */
/*                  g_ap_ch_tbl_index                                        */
/*                  g_ap_ch_switch_cnt                                       */
/*                  g_ap_ch_switch_status                                    */
/*                  g_ap_channel_list                                        */
/*                                                                           */
/*  Processing    : All globals related to channel management are            */
/*                  initialized to sane values in this function.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_chan_mgmt_ap_globals(void)
{
    UWORD8 i = 0;

    g_ap_announced_channel = mget_CurrentChannel();
    g_ap_ch_tbl_index      = get_tbl_idx_from_ch_idx(get_current_start_freq(),
                             mget_CurrentChannel());
    g_ap_ch_switch_cnt     = 0;
    g_ap_ch_switch_status  = CH_SWITCH_DONE;
    g_ap_scan_itr_cnt      = 0;

    for(i = 0; i < MAX_SUPP_CHANNEL; i++)
    {
		/* Init common parameters */
        g_ap_channel_list[i].num_networks = 0;


       /* DFS related elements are initilized in dfs_ap.c */
    }

    /* Initialize the BSS ID Storage strucure */
    g_bss_id_list.num_ntwrks   = 0;
    g_bss_id_list.bss_id_array = NULL;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_bss_in_available_channel                           */
/*                                                                           */
/*  Description   : This function starts the channel availability check for  */
/*                  an 11h AP.                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start request message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function starts a BSS in the channel available for  */
/*                  operation (giving first preference to the current channel*/
/*                  configured by the user). In case no channel is available */
/*                  exception handling must be done.                         */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void start_bss_in_available_channel(void *start_req)
{
    UWORD8 channel = 0;
    UWORD8 sec_chan_off = 0;

    /* Select an available channel for operation */
    channel = select_channel_for_operation(&sec_chan_off);

    if(is_ch_valid(channel) == BFALSE)
    {
        PRINTK("DFSErr: No channel available for operation2\n");

        /* Exception case. No channel was found available for operation.     */
        /* restart so that channel availability check will be done again     */
        raise_system_error(DFS_ERROR);
    }
    else
    {
        PRINTD("AP: Starting network in %d-%d\n",channel, sec_chan_off);

		// 20120830 caisf mod, merged ittiam mac v1.3 code
		#if 0
        /* Select the right primary secondary channels */
        select_channel_mac(mget_CurrentChannel(), sec_chan_off);
		#else
        /* Select suitable Regulatory Class for the combination of channel  */
        /* and offset 														*/
        sec_chan_off = select_reg_class(channel, sec_chan_off);

        /* Select the right primary secondary channels */
        select_channel_mac(channel, sec_chan_off);
		#endif

        /* Initiate the BSS start with the given start request */
        start_bss_ap(start_req);
    }
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_channel_availability_check                         */
/*                                                                           */
/*  Description   : This function starts the channel availability check for  */
/*                  an 11h AP.                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start request message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function resets the global DFS channel index to 0.  */
/*                  It then performs a channel availability check. In case   */
/*                  the status returned from this check indicates that the   */
/*                  channel availability check is complete a BSS is started  */
/*                  in the available channel. If it indicates that the       */
/*                  check is still in progress the incoming start request    */
/*                  is saved in a global and the function returns.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_channel_availability_check(void *start_req)
{
    UWORD8 status  = 0;

    /* Perform a channel availability check */
    status = do_channel_availability_check(BTRUE);

    /* If the status indicates that the check is complete/not required, start*/
    /* bss in the avalble cha. Else, save the incoming start request and     */
    /* return. The BSS start will be initiated once the channel availability */
    /* check is complete (in the handling of the channel availability timer  */
    /* timeout MISC event)                                                   */
    if((CHNL_AV_CHK_COMP    == status) ||
       (CHNL_AV_CHK_NOT_REQ == status))
    {
        start_bss_in_available_channel(start_req);
    }
    else /* CHNL_AV_CHK_IN_PROG */
    {
        memcpy(&g_ap_saved_start_req, start_req, sizeof(start_req_t));
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : do_channel_availability_check                            */
/*                                                                           */
/*  Description   : This function performs channel availability check for    */
/*                  the channel with status AVAILABLE_TO_SCAN.               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ap_channel_list                                        */
/*                  g_ap_ch_tbl_index                                        */
/*                                                                           */
/*  Processing    : This function checks the entire channel list for DFS AP  */
/*                  and for channels with status set to AVAILABLE_TO_SCAN it */
/*                  does the following. It resets the global for radar       */
/*                  detection and selects the channel. It next starts a      */
/*                  timer for the channel availability check time. It        */
/*                  returns the status to indicate that channel availability */
/*                  check is in progress. In case all channels with the      */
/*                  status AVAILABLE_TO_SCAN have been processed there is no */
/*                  further checking to be done. In such a case a status is  */
/*                  returned to indicate that the channel availability check */
/*                  has been completed.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, CHNL_AV_CHK_COMP if channel availability check   */
/*                          check is complete and CHNL_AV_CHK_IN_PROG if it  */
/*                          is currently in progress                         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 do_channel_availability_check(BOOL_T first_time)
{
    ap_ch_info_t channel_info = {0};
    UWORD8       freq         = get_current_start_freq();
    UWORD8       num_sup_chan = get_num_supp_channel(freq);
    static UWORD8 start_tbl_idx = 0;
    static UWORD8 end_tbl_idx   = 0;
    UWORD8       ch_idx        = 0;


    /* Do channel availability check for the channel in channel list that    */
    /* is not blocked or un-supported                                        */
    while(1)
    {
		/* Increment the index for scanning of next channel, */
		/* if this is not the first channel being scanned */
		if(first_time == BTRUE)
		{
			if(BFALSE == is_chnl_scan_needed(&start_tbl_idx, &end_tbl_idx))
			{
    		    PRINTD("Channel scanning not needed\n");
				return CHNL_AV_CHK_NOT_REQ;
			}
			else
			{
				PRINTD("Scanning from Ch Idx.%d to %d\n",start_tbl_idx, end_tbl_idx);
			}

			/* Reset the global DFS channel index */
			g_ap_ch_tbl_index =  start_tbl_idx;

			/* Reset the number of interations of scanning */
			g_ap_scan_itr_cnt  = 0;

			/* Initialize the num networks found in a channel to zero */
			g_bss_id_list.num_ntwrks   = 0;

			/* Get a buffer to store the BSSIDs from Tx/Rx buffer pool */
			if(g_bss_id_list.bss_id_array == NULL)
			{
				/* This will be deleted once AP goes to data mode
				form scan mode */
				g_bss_id_list.bss_id_array =
				(UWORD8*)pkt_mem_alloc(MEM_PRI_RX);
			}
		}
		else
		{
			BOOL_T scan_itr_over = BFALSE;

			/* Store the number of detected networks in this channel */
			g_ap_channel_list[g_ap_ch_tbl_index].num_networks
			= g_bss_id_list.num_ntwrks;

			/* Reset the num networks found in a chan to 0 for next channel */
			g_bss_id_list.num_ntwrks   = 0;

            /* Iteration is complete if we reached end channel */
			if(g_ap_ch_tbl_index == end_tbl_idx)
			{
				scan_itr_over = BTRUE;
			}

			/* Increment the channel index */
			g_ap_ch_tbl_index++;

			/* Validate the channel index upper side*/
			if(g_ap_ch_tbl_index >= num_sup_chan)
					g_ap_ch_tbl_index = 0;

            /* Iteration is complete if we reached back to home channel */
			if(g_ap_ch_tbl_index == start_tbl_idx)
			{
				scan_itr_over = BTRUE;
			}

            if(BTRUE == scan_itr_over)
			{
				g_ap_scan_itr_cnt++;

				if(g_ap_scan_itr_cnt >= get_num_scan_itr_prot())
				{
				    PRINTD("Scan over\n");
					break;
				}
			}
		}

        /* Get the current channel info from the list and index */
        channel_info = g_ap_channel_list[g_ap_ch_tbl_index];

		ch_idx = get_ch_idx_from_tbl_idx(freq, g_ap_ch_tbl_index);

        /* Select the channel if it is available (no blocking) and start     */
        /* the channel availability check timer. Reset the global variable   */
        /* for radar detection. Break from this loop. Once the timer expires */
        /* this loop needs to be resumed.                                    */
        if(BTRUE == scan_availabilty(channel_info))
        {
            select_channel_rf(ch_idx, 0);
            start_chnl_av_chk_timer();

            PRINTD("Scanning Ch Idx.%d Tbl Tdx.%d\n",ch_idx, g_ap_ch_tbl_index);
            return CHNL_AV_CHK_IN_PROG;
        }
        else
        {
            PRINTD("Skipping Ch Idx.%d Tbl Tdx.%d\n",ch_idx, g_ap_ch_tbl_index);
        }

        /* Mark the variable such that next channel will be
        scanned in while loop */
        first_time = BFALSE;
    }

    /* Free buffer allocated for storing the BSSIDs */
    pkt_mem_free(g_bss_id_list.bss_id_array);
	g_bss_id_list.bss_id_array = NULL;

    return CHNL_AV_CHK_COMP;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : select_channel_for_operation                             */
/*                                                                           */
/*  Description   : This function selects a channel available for operation  */
/*                  from the global DFS channel list.                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ap_channel_list                                        */
/*                                                                           */
/*  Processing    : This function checks the entire channel list for DFS AP  */
/*                  for a channel with status set to AVAILABLE_TO_OPERATE.   */
/*                  In case DFS and 40MHz are disabled the current channel   */
/*                  set by the user is assumed to be available channel, if   */
/*                  Least congested channel selection is not enabled.        */
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

UWORD8 select_channel_for_operation(UWORD8 *sec_chan_offset)
{
    UWORD16  scratch_mem_idx     = get_scratch_mem_idx();
    UWORD8 i = 0;
    UWORD8 least_busy_chan = 0;
    UWORD16 least_cum_beacon = 0xFFFF;
    UWORD8 chan_offset = SCN;
    UWORD8 user_chan_idx = 0xFF, user_chan_offset = 0xFF;
    UWORD8 cum_networks = 0;
    UWORD8 ch_idx  = INVALID_CHANNEL;
    BOOL_T fortyMHz_poss = BFALSE;
    UWORD8 freq         = get_current_start_freq();
    UWORD8 num_sup_chan = get_num_supp_channel(freq);
    eval_scan_res_t *chan_scan_res = (eval_scan_res_t *)
    scratch_mem_alloc(MAX_SUPP_CHANNEL * sizeof(eval_scan_res_t));

    /* Reset the Eval Channel structure */
    mem_set(chan_scan_res,0,MAX_SUPP_CHANNEL * sizeof(eval_scan_res_t));

    /* Check radar and mark channel that can be used */
    check_radar(chan_scan_res, num_sup_chan);

	/* Choose the user selected channel,
	if Auto Channel selection is disabled */
	if(BFALSE == is_auto_chan_sel_enabled())
	{
		user_chan_idx =  get_ch_idx_from_num(freq, get_user_pref_primary_ch());
	   	user_chan_offset = get_user_pref_sec_ch();

		PRINTD("User Preferred channel idx %d-%d\n",user_chan_idx,user_chan_offset);
	}

	fortyMHz_poss = get_40MHz_possibility(chan_scan_res);

	if(BTRUE == fortyMHz_poss)
	{
		PRINTD("Selecting 40MHz ch from One of ...  ");
	}
	else
	{
		PRINTD("Selecting 20MHz ch from One of ...  ");
	}

    /* Now we have the 11h and 11n protocol related information for */
    /* Look for the least busy channel                              */
    for(i = 0; i < num_sup_chan; i++)
    {
        ch_idx = get_ch_idx_from_tbl_idx(freq, i);

        /* Check if operation is allowed as per DFS and 2040 */
        /* else skip this channel */
        if(!(chan_scan_res[i].chan_op & OP_ALLOWED))
        {
			continue;
		}

		/* If 40 MHz is possible and user also wants it */
		if((BTRUE == fortyMHz_poss)  &&
	       (user_chan_offset != SCN))
		{
			/* If Above channel can be used for Secondary */
			if(BTRUE == is_40MHz_SCA_allowed(chan_scan_res,i,user_chan_offset))
			{
				PRINTD("%dA ",i);

				/* Select this if number of other beacons
				are less than last minimum */
				if(chan_scan_res[i].cumm_num_ntwrk[NUM_NET_SCA] <
				   least_cum_beacon)
				{
					least_cum_beacon =
					chan_scan_res[i].cumm_num_ntwrk[NUM_NET_SCA];
					least_busy_chan = ch_idx;
					chan_offset = SCA;
				}
			}

			/* If below channel can be used for Secondary */
			if(BTRUE == is_40MHz_SCB_allowed(chan_scan_res,i,user_chan_offset))
			{
				PRINTD("%dB ",i);

				/* Select this if number of other beacons
				  are less than last minimum */
				if(chan_scan_res[i].cumm_num_ntwrk[NUM_NET_SCB] <
				   least_cum_beacon)
				{
					least_cum_beacon =
					chan_scan_res[i].cumm_num_ntwrk[NUM_NET_SCB];
					least_busy_chan = ch_idx;
					chan_offset = SCB;
				}
			}
		}
		else /* 40 MHz not possible */
		{
		    PRINTD("%d ",i);

			/* Get the cumulative # of other network beacons in the vicinity */
			cum_networks = get_cum_networks(ch_idx);

			/* Choose the channel with minimum cumulative number of beacons */
			if(cum_networks < least_cum_beacon)
			{
				least_cum_beacon = cum_networks;
				least_busy_chan = ch_idx;
			}
		}


		/* User preferred channel index reached */
		if(ch_idx == user_chan_idx)
		{
			least_busy_chan = ch_idx;
			break;
		}
    }

    *sec_chan_offset = chan_offset;

    PRINTD("\n Selected Channel Tbl Idx %d-%d\n",least_busy_chan, chan_offset);

    if((user_chan_idx != 0xFF) &&
       (user_chan_idx != least_busy_chan))
    {
		PRINTD("WARNING: User's preferred channel could not be selected\n");
	}

    /* Restore the scratch memory which was used to copy the attributes*/
    restore_scratch_mem_idx(scratch_mem_idx);

    /* If no channel is found to be available return an Invalid value */
    return least_busy_chan;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initiate_switch_to_new_channel                        */
/*                                                                           */
/*  Description      : This function initiates switch to the new channel     */
/*                     once radar has been detected.                         */
/*                                                                           */
/*  Inputs           : 1) Channel index of the new channel                   */
/*                                                                           */
/*  Globals          : g_ap_announced_channel                                */
/*                     g_ap_ch_switch_cnt                                    */
/*                     g_ap_ch_switch_status                                 */
/*                                                                           */
/*  Processing       : This function performs the required functions to      */
/*                     initiate the switch to the new channel. It saves the  */
/*                     new channel in a global, updates the global channel   */
/*                     switch count and sets a flag to indicate that channel */
/*                     switch has been initiated. It adds the Channel Switch */
/*                     Announcement information element in the beacon frame. */
/*                     Lastly it prepares and sends a Channel Switch         */
/*                     Announcement Frame.                                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

static void initiate_switch_to_new_channel(UWORD8 channel, UWORD8 sec_chan)
{
    UWORD8 cnt = 0;

    /* Update the global variables */
    g_ap_ch_switch_status   = CH_SWITCH_STG_1;
    g_ap_announced_channel  = channel;
    g_ap_ch_switch_cnt      = mget_chan_switch_cnt();
    g_ap_announced_sec_chan = sec_chan;

    /* Add the Channel Switch Announcement information element in the beacon */
    add_csa_elem_in_bcn(g_ap_announced_channel);

    /* Prepare and send a Channel Switch Announcement Frame.                 */
    /* NOTE: This is optional. It can be done to ensure that stations are    */
    /* immediately silenced instead of after reception of next beacon. There */
    /* is a possibility that this frame does not get sent before the channel */
    /* is switched (in case a beacon transmission happens before this and    */
    /* the channel switch count is 0). In such a case after the channel      */
    /* switch this frame may be transmitted. However this should not affect  */
    /* any functionality as the new channel will then be the current channel */
    send_csa_frame(g_ap_announced_channel);

    /* Wait on the transmission of this Channel announcement frame */
    while((is_machw_q_null(HIGH_PRI_Q) == BFALSE) &&
           (cnt < CH_SWITCH_SEND_TIME_OUT_COUNT))
    {
        add_delay(0xFFF);
        cnt++;
    }

    /* Suspend the frame transmissions (except for beacons)  */
    set_machw_tx_suspend();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : attempt_new_chan                                      */
/*                                                                           */
/*  Description      : This function switches to the new channel.            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_ap_announced_channel                                */
/*                                                                           */
/*  Processing       : This function disables MAC H/w (after suspending the  */
/*                     transmissions) and PHY. It then selects the channel   */
/*                     based on the new available channel saved in a global. */
/*                     It then enables MAC H/w and PHY and resumes Tx.       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void attempt_new_chan(UWORD8 channel, UWORD8 sec_chan)
{
	/* Select the 20/40 MHz channel */
	select_channel_mac(channel,sec_chan);

    /* While attempting a new channel check if the new channel need to be    */
    /* scanned again, else start the network right away                      */
    /* Note that already radar blocked check was done on this channel in fn  */
    /* select_channel_for_operation, so no need for that check again  */
    if(BTRUE == need_of_rescan(g_ap_channel_list[channel]))
    {
        /* Set the global to scan in progress */
        g_ap_ch_switch_status = CH_SWITCH_STG_2;

        /* Start the channel scan again */
        start_chnl_av_chk_timer();
    }
    else
    {
        restart_network_after_switch();
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : eval_chan_status                                      */
/*                                                                           */
/*  Description      : This function switches to the new channel.            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_ap_ch_switch_status                                 */
/*                                                                           */
/*  Processing       : This function processes a request for switching       */
/*                     channel. It evaluates the updated scan result and     */
/*                     finds the best possible channel to work in            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Netwrork Status change indicator                      */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

NTWRK_STAT_CHANGE_T eval_chan_status(void)
{
    UWORD8 channel = 0;
    UWORD8 sec_chan_off = 0;
    /* If a switch has been initiated already do nothing */
    if(g_ap_ch_switch_status == CH_SWITCH_STG_1)
        return NTWRK_STAT_NO_CHANGE;

    /* There is a event which needs re-evaluation of operating parameter*/
    /* The current channel and other protocol (20/40) operation is also */
    /* evaluated                                                        */
    channel = select_channel_for_operation(&sec_chan_off);

    if(is_ch_valid(channel) == BFALSE)
    {
        PRINTD("Couldnt start ntwk using the selected channel or options\n");

        /* Exception case. No channel was found available for operation.     */
        /* restart so that channel availability check will be done again     */
        raise_system_error(DFS_ERROR);

        return NTWRK_STAT_PR_CHANGE;
    }

    if(channel != mget_CurrentChannel())
    {
		// 20120830 caisf add, merged ittiam mac v1.3 code
        /* Select suitable Regulatory Class for the combination of channel */
        /* and offset 													   */
        sec_chan_off = select_reg_class(channel, sec_chan_off);

        /* There is a change in the operating condition of the channel  */
        /* and a new channel is selected then initiate a channel switch */
        initiate_switch_to_new_channel(channel, sec_chan_off);

        return NTWRK_STAT_PR_CHANGE;
    }
    else if(sec_chan_off != get_bss_sec_chan_offset())
    {
        /* Select the right primary secondary channels */
        select_channel_mac(mget_CurrentChannel(), sec_chan_off);

        return NTWRK_STAT_SC_CHANGE;
    }


    PRINTD("No channel change after re-evaluation....\n");

    return NTWRK_STAT_NO_CHANGE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_csa_frame                                        */
/*                                                                           */
/*  Description      : This function sends a Channel Switch Announcement     */
/*                     frame.                                                */
/*                                                                           */
/*  Inputs           : 1) New channel to switch to                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function prepares a Channel Switch Announcement  */
/*                     frame and transmits the same.                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void send_csa_frame(UWORD8 new_channel)
{
    UWORD16 index = 0;
    UWORD8  *data = NULL;

    data = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);

    if(data == NULL)
    {
        return;
    }

    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)ACTION);

    set_address1(data, mget_bcst_addr());
    set_address2(data, mget_StationID());
    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /* Channel Switch Announcement - Frame body                              */
    /* --------------------------------------------------------------------- */
    /* | Category | Action | Channel Switch Announcement IE                | */
    /* --------------------------------------------------------------------- */
    /* | 1        | 1      | 6                                             | */
    /* --------------------------------------------------------------------- */

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Category field of action frame */
    data[index++] = SPECMGMT_CATEGORY;

    /* Action field of action frame           */
    data[index++] = CHANNEL_SWITCH_ANNOUNCEMENT_TYPE;

    set_csa_info_field(data, index, new_channel);
    index += (IE_HDR_LEN + ICHSWANNOUNC_LEN);

    /* Transmit the CSA frame */
    tx_mgmt_frame(data, index + FCS_LEN, HIGH_PRI_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : ch_av_chk_timeout_alarm_fn                            */
/*                                                                           */
/*  Description      : This is the channel availability check timeout        */
/*                     alarm function.                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to the timer                               */
/*                     2) Data for the alarm function (unused)               */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function creates and posts a miscellaneous       */
/*                     channel availability check timeout event in the       */
/*                     miscellaneous event queue.                            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void ch_av_chk_timeout_alarm_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void ch_av_chk_timeout_alarm_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc = 0;

    /* Create a MISC_CH_AVAIL_CHECK_TIMEOUT event with no message and post   */
    /* it to the event queue.                                                */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc == NULL)
    {
        /* Exception */
        raise_system_error(NO_EVENT_MEM);
        return;
    }

    misc->data = 0;
    misc->name = MISC_CH_AVAIL_CHECK_TIMEOUT;
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_chan_status_ap                                 */
/*                                                                           */
/*  Description      : This funtion updates the channel status for AP        */
/*                                                                           */
/*  Inputs           : 1) Channel index                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function calls the respective protif function    */
/*                     to update the channel information status              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void update_chan_status_ap(UWORD8 ch_idx)
{
    update_chan_status_prot(ch_idx);
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_ch_scan_time_ap                                   */
/*                                                                           */
/*  Description      : This funtion gets the channel scan time per channel   */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function calls the respective protif function    */
/*                     to get the channel scan time                          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
UWORD32 get_ch_scan_time_ap(void)
{
	return get_ch_scan_time_prot();
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : store_network                                         */
/*                                                                           */
/*  Description      : This funtion stores a received network BSSID          */
/*                                                                           */
/*  Inputs           : BSSID of received a network frame                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the BSSID is already present  */
/*                     in its array of BSSIDs, else it stores & updates cnt  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void store_network(UWORD8* ntwrk_bssid)
{
	UWORD8 bcast_bssid[MAC_ADDRESS_LEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	BOOL_T already_present = BFALSE;
	UWORD8 cnt = 0;

	/* Ignore the broadcast BSSID frames */
	if(BTRUE == mac_addr_cmp(bcast_bssid,ntwrk_bssid))
	{
	   return;
	}

    //chengwg add
	/* Get a buffer to store the BSSIDs from Tx/Rx buffer pool */
	if(g_bss_id_list.bss_id_array == NULL)
	{
		/* This will be deleted once AP goes to data mode form scan mode */
		g_bss_id_list.bss_id_array = (UWORD8*)pkt_mem_alloc(MEM_PRI_RX);
	}
    
    /* Search through the array */
	for(cnt = 0; cnt < g_bss_id_list.num_ntwrks; cnt++)
	{
		/* Check if any BSSID matches the received frame's BSSID */
		if(BTRUE == mac_addr_cmp((g_bss_id_list.bss_id_array +
		                          cnt*MAC_ADDRESS_LEN) ,  ntwrk_bssid))
		{
			already_present = BTRUE;
			break;
		}
	}

    /* If its a new network's frame, then store it */
	if(already_present == BFALSE)
	{
		mac_addr_cpy((g_bss_id_list.bss_id_array +
		              cnt*MAC_ADDRESS_LEN), ntwrk_bssid);
		g_bss_id_list.num_ntwrks++;
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_tbtt_chan_mgmt_ap                              */
/*                                                                           */
/*  Description      : This function handles TBTT event for AP               */
/*                     in the enabled state.                                 */
/*                                                                           */
/*  Inputs           : DTIM count of current TBTT                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : channel switch is in progress it updates the channel  */
/*                     switch count and the beacon frame as required. In case*/
/*                     the count has reached zero it switches the channel    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void handle_tbtt_chan_mgmt_ap(UWORD8 dtim_count)
{
	/* If channel switching is in progress */
    if(g_ap_ch_switch_status == CH_SWITCH_STG_1)
    {
        /* If channel switch is in progress, check the channel switch count  */
        /* global and process further                                        */
        if(g_ap_ch_switch_cnt == 0)
        {
            UWORD8 null_addr[6] = {0};

            /* If the count zero it indicates that the last beacon sent had  */
            /* a Channel Switch Count value of 0. Hence the channel can now  */
            /* be switched. Remove CSA element from the beacon and switch    */
            /* channel. Suspend beacon tx untill the new channel is cleared  */
            set_machw_tsf_beacon_tx_suspend_enable();
            remove_csa_elem_in_bcn();
            mset_bssid(null_addr);
            attempt_new_chan(g_ap_announced_channel, g_ap_announced_sec_chan);
        }
        else
        {
            /* If the count is non-zero, decrement it and update the CSA     */
            /* element in the beacon with the updated Channel Switch Count.  */
            /* NOTE: This is optional. The CSA element channel switch count  */
            /* can be chosen to be set to 0 always. In that case at every    */
            /* TBTT the beacon does not need to be updated.                  */
            g_ap_ch_switch_cnt--;
            update_csa_elem_in_bcn(g_ap_ch_switch_cnt);
        }
    }
    /* Handle Bandwidth switching at the end of a DTIM period */
    else if(dtim_count == 0)
    {
		if(g_ap_bw_switch_status == BW_SWITCH_40_TO_20)
		{
 			g_ap_bw_switch_status = BW_SWITCH_DONE;

	        /* Select the right primary secondary channels */
	        select_channel_mac(mget_CurrentChannel(), SCN);

    		PRINTD("Switched to 20MHz at DTIM 0\n");
		}
    }
}

#endif /* BSS_ACCESS_POINT_MODE */
