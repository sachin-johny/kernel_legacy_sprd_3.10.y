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
/*  File Name         : channel_sw.c                                         */
/*                                                                           */
/*  Description       : This file contains functions to handle Dynamic       */
/*                      Frequency Switching feature through use of Channel   */
/*                      Switch Announcement frames described in the standard.*/
/*                                                                           */
/*  List of Functions : handle_ch_switch_ie                                  */
/*                      update_ch_swt_count                                  */
/*                      handle_tbtt_spec_mgmt_sta                            */
/*                      handle_spec_mgmt_action                              */
/*                      switch_channel_csw                                   */
/*                      on_AP_wait_expiry                                    */
/*                      is_this_channel_supported_by_rf                      */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "frame.h"
#include "channel_sw.h"
#include "cglobals_sta.h"
#include "sta_prot_if.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_current_channel                  = 0;
UWORD8 g_current_sec_chan_offset          = 0;

BOOL_T g_waiting_for_AP                   = BFALSE;
BOOL_T g_waiting_to_shift_channel         = BFALSE;
BOOL_T g_channel_swt_cnt_zero             = BFALSE;

ALARM_HANDLE_T *g_csw_alarm_wait_for_AP   = NULL;

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static UWORD8  g_new_channel         = 0;
static UWORD8  g_channel_swt_cnt     = 0;
static UWORD8  g_new_sec_chan_offset = 0xFF;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void handle_ch_switch_ie(ELEMENTID_T ie_type, UWORD8 *data);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_ch_switch_ie                                      */
/*                                                                           */
/*  Description   : This function is used to parse any (Extended) Channel    */
/*                  Switch Announcement Elements.                            */
/*                                                                           */
/*  Inputs        : 1) Type of Channel Switch Element                        */
/*                     ICHSWANNOUNC - Channel Switch Announcement Element    */
/*                     IEXCHSWANNOUNC - Extended Channel Switch Announcement */
/*                                      Element                              */
/*                  2) Pointer to the Channel Switch Element (excluding      */
/*                     headers)                                              */
/*                                                                           */
/*  Globals       : g_new_channel                                            */
/*                  g_waiting_to_shift_channel                               */
/*                  g_channel_swt_cnt_zero                                   */
/*                                                                           */
/*  Processing    : This function parses the channel switch element and      */
/*                  updates the corresponding channel switching parameters   */
/*                  based on the information contained in it.                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_ch_switch_ie(ELEMENTID_T ie_type, UWORD8 *data)
{
    UWORD8 new_chan   = 0;
    UWORD8 new_reg    = 0;
    UWORD8 ch_sw_mode = 0;
    UWORD8 ch_sw_cnt  = 0;

    /*************************************************************************/
    /*       Channel Switch Announcement element                             */
    /* --------------------------------------------------                    */
    /* |Channel Switch Mode|New Channel| Ch switch count|                    */
    /* --------------------------------------------------                    */
    /* |1                  |1          |1               |                    */
    /* --------------------------------------------------                    */
    /*                                                                       */
    /*                Extended Channel Switch Announcement element           */
    /* ----------------------------------------------------------------      */
    /* |Channel Switch Mode|New Reg Class|New Channel| Ch switch count|      */
    /* ----------------------------------------------------------------      */
    /* |1                  |1            |1          |1               |      */
    /* ----------------------------------------------------------------      */
    /*                                                                       */
    /*************************************************************************/

    if(ie_type == ICHSWANNOUNC)
    {
        /* Channel Switch Announcement element */
        ch_sw_mode = data[0];
        new_chan   = data[1];
        ch_sw_cnt  = data[2];

        /* Parsing of the frame is continued since some APs may also */
        /* include Extended Channel Switch Announcement Element in   */
        /* the frame.                                                */
    }
    else if(ie_type == IEXCHSWANNOUNC)
    {
        /* Extended Channel Switch Announcement element */
        ch_sw_mode = data[0];
        new_reg    = data[1];// 20120830 caisf mod, merged ittiam mac v1.3 code
        new_chan   = data[2];
        ch_sw_cnt  = data[3];
    }
    else
        return;

    /* If channel switch count is already updated then do nothing */
    if((g_waiting_to_shift_channel == BFALSE) &&
       (g_waiting_for_AP == BFALSE))
    {
        BOOL_T check = BFALSE;

        if(ch_sw_mode == 1)
        {
            /* Disable transmission of frames from the STA till the */
            /* channel switch is complete.                          */
            disable_machw_tx();
        }

        /* Update new channel for BSS operation */
        g_new_channel = new_chan;

		// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
        /* Extended Channel Switch element can convey Channel Width */
        /* information through "New Regulatory Class" element       */
        if(ie_type == IEXCHSWANNOUNC)
        {
            /* TBD: Regulatory Class Support to be added                */
            /* Derive the Channel width info using the variable new_reg */
        }

        /* Check whether this channel is supported by STA or not */
        /* if STA doesn't support this channel then return BFALSE */
        check = is_this_channel_supported_by_rf(g_new_channel);

        if(check == BTRUE)
        {
            /* Update channel switch count */
            g_channel_swt_cnt          = ch_sw_cnt;
            g_waiting_to_shift_channel = BTRUE;

            /* If channel switch count is 'zero' then change the channel */
            /* after receiving this frame */
            if(g_channel_swt_cnt == 0)
            {
                 g_channel_swt_cnt_zero = BTRUE;
            }
        }
        else
        {
             PRINTD("DFSErr: New Channel No.%d not supported by RF\n",g_new_channel);

             /* If STA doesn't support new channel then raise system error*/
             raise_system_error(DFS_ERROR);
        }
#else
        /* Extended Channel Switch element can convey Channel Width */
        /* information through "New Regulatory Class" element       */
        if(ie_type == IEXCHSWANNOUNC)
        {
            /* Derive the Channel width info using the variable new_reg */
            g_new_reg_class = new_reg;

	        /* Check whether this channel is supported by STA or not */
	        /* if STA doesn't support this channel then return BFALSE */
	        check = is_ch_supp_in_rc_in_rd(g_new_channel,
	                                       get_current_reg_domain(),
	                                       g_new_reg_class);
            if(check == BTRUE)
            {
                /* set BSS width to SCN if Regulatory class does not allow */
                /* SCA or SCB                                              */
                adjust_bss_width_in_new_reg_class(g_new_reg_class);
        	}
            else
            {
                PRINTD("MultiDomainErr: New Channel No.%d is not supported in New Regulatory Class = %d\n",g_new_channel, g_new_reg_class);
                raise_system_error(MDOM_ERROR);
            }
        }
        else
        {
       	 	/* Check whether this channel is supported by STA or not */
        	/* if STA doesn't support this channel then return BFALSE */
        	check = is_this_channel_supported_by_rf(g_new_channel);

            if(check == BFALSE)
            {
                PRINTD("DFSErr: New Channel No.%d not supported by RF\n",g_new_channel);
                 /* If STA doesn't support new channel then raise system error*/
                 raise_system_error(DFS_ERROR);
            }
        }

        if(check == BTRUE)
        {
            /* Update channel switch count */
            g_channel_swt_cnt          = ch_sw_cnt;
            g_waiting_to_shift_channel = BTRUE;

            /* If channel switch count is 'zero' then change the channel */
            /* after receiving this frame */
            if(g_channel_swt_cnt == 0)
            {
                 g_channel_swt_cnt_zero = BTRUE;
            }
        }
#endif
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_csa_ie                                             */
/*                                                                           */
/*  Description   : This function is used to parse any channel switch        */
/*                  information contained in Beacon Frames.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to Beacon Frame from Associated AP            */
/*                  2) Length of the frame                                   */
/*                  3) Index from where the search should start              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the Beacon Frame for Channel Switch */
/*                  and Extended Channel Switch Announcement Information     */
/*                  Elements. On finding any of these, it processes them to  */
/*                  extract the channel switching information contained in   */
/*                  them.                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_csa_ie(UWORD8 *msa, UWORD16 rx_len, UWORD16 tag_param_offset)
{
    UWORD16 index     = 0;
    UWORD16 ch_index  = 0xFFFF;
    BOOL_T retval     = BFALSE;

    /*************************************************************************/
    /*                    Channel Switch Announcement element                */
    /* --------------------------------------------------------------------- */
    /* |Element ID|Length |Channel switch Mode|New Channel| Ch switch count| */
    /* --------------------------------------------------------------------- */
    /* |1         |1      |1                  |1          |1               | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*                Extended Channel Switch Announcement element           */
    /* --------------------------------------------------------------------- */
    /* |Elem ID|Length|Ch Switch Mode|New Reg Class|New Ch| Ch switch count| */
    /* --------------------------------------------------------------------- */
    /* |1      |1     |1             |1            |1     |1               | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index = tag_param_offset;

    /* Search for Channel Switch Element */
    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == ICHSWANNOUNC)
        {
            /* Channel Switch Announcement element */
            ch_index = index;

            /* Parsing of the frame is continued since some APs may also */
            /* include Extended Channel Switch Announcement Element in   */
            /* the frame.                                                */
        }
        else if(msa[index] == IEXCHSWANNOUNC)
        {
            /* Extended Channel Switch Announcement element */
            ch_index = index;

            break;
        }

        index += (IE_HDR_LEN + msa[index + 1]);
    }

    if(ch_index !=0xFFFF)
    {
        handle_ch_switch_ie((ELEMENTID_T)(msa[ch_index]),
                            (msa + ch_index + IE_HDR_LEN));

        retval = BTRUE;
    }

    return retval;

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_tbtt_spec_mgmt_sta                                */
/*                                                                           */
/*  Description   : This function should be called on every TBTT interrupt.  */
/*                  It handles channel switching at the right time based on  */
/*                  the state variables.                                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : g_channel_swt_cnt_zero                                   */
/*                  g_link_loss_count                                        */
/*                  g_waiting_for_AP                                         */
/*                                                                           */
/*  Processing    : This function triggers channel switching process if      */
/*                  required as indicated by the Channel Switch Count state  */
/*                  variable. It also ensures that unnecessary Link Loss     */
/*                  does not occur during the channel switching process.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_tbtt_spec_mgmt_sta(void)
{
	TROUT_FUNC_ENTER;
    if(mget_SpectrumManagementImplemented() == TV_FALSE)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Switch to the new channel if the AP has indicated immediate    */
    /* channel switching by setting the Channel Switch Count to Zero. */
    if(g_channel_swt_cnt_zero == BTRUE)
    {
        if(g_link_loss_count > LINK_LOSS_OFFSET_11H)
        {
            switch_channel_csw();
            g_link_loss_count = 0;
        }
    }

    /* Channel Switch is done. Now waiting for beacons from AP in the */
    /* new channel.                                                   */
    if(g_waiting_for_AP == BTRUE)
    {
        if(check_radar_ch() == BTRUE)
        {
            /* If the new channel is Radar affected, then avoid link loss */
            /* while the wait continues.                                  */
            g_link_loss_count = 0;
        }
        else
        {
            /* Enable transmissions in the new channel after waiting for */
            /* sometime (even if the AP has not moved yet).              */
            if(g_link_loss_count > LINK_LOSS_OFFSET_11H)
            {
                g_waiting_for_AP = BFALSE;
                stop_alarm(g_csw_alarm_wait_for_AP);
                enable_machw_tx();

                g_link_loss_count = 0;
            }
        }
    }

    /* If Channel switching is co-ordinated through Channel Switch count, */
    /* then the count has to be decremented on every TBTT.                */
    if(g_channel_swt_cnt > 0)
    {
        g_channel_swt_cnt--;

        if(g_channel_swt_cnt == 0)
        {
            /* If the channel switch count was loaded from an incoming */
            /* frame, then switch the channel immediately.             */
            if(g_waiting_to_shift_channel == BTRUE)
            {
                switch_channel_csw();
            }
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_spec_mgmt_action                                  */
/*                                                                           */
/*  Description   : This function handles Spectrum Management related        */
/*                  Action frames                                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to Action Frame                               */
/*                  2) Length of the Frame                                   */
/*                  3) Length of the MAC Header                              */
/*                  4) RX Dscr corresponding to the received Frame           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function handles Channel Switch and Extended        */
/*                  Channel switch Action frames. It extracts the relevant   */
/*                  Channel Switching related information contained in them. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_spec_mgmt_action(UWORD8* msa, UWORD16 rx_len, UWORD8 mac_hdr_len,
                             UWORD32* rx_dscr)
{
    UWORD8  *data = NULL;

	TROUT_FUNC_ENTER;
	
    if(mget_SpectrumManagementImplemented() == TV_FALSE)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    data = msa + mac_hdr_len;

    /* Process the action frame based on the category */
    switch(data[CATEGORY_OFFSET])
    {
    case SPECMGMT_CATEGORY:
    {
        /* Process the QOS action frame based on the action type */
        switch(data[ACTION_OFFSET])
        {
        case CHANNEL_SWITCH_ANNOUNCEMENT_TYPE:
        {
            /* Offset into Data portion of Action Frame */
            UWORD16 index = mac_hdr_len + 2;

            while(index < (rx_len - FCS_LEN))
            {
                if(msa[index] == ICHSWANNOUNC)
                {
                    /* Channel Switch Announcement element */
                    handle_ch_switch_ie(ICHSWANNOUNC,
                                        &msa[index+IE_HDR_LEN]);
                }
                else if(msa[index] == ISECCHOFF)
                {
                    /* TBD: Adopt the Secondary Channel Offset for the new Channel */
                    g_new_sec_chan_offset = msa[index + 2];
                }

                index += (IE_HDR_LEN + msa[index + 1]);
            }
        }
        break;

        default:
        {
            /* Do nothing. Received action type is either not recognized */
            /* or does not need to be handled in STA mode.               */
        }
        break;
        }
    }
    break;

    case PUBLIC_CATEGORY:
    {
        /* Process the QOS action frame based on the action type */
        switch(data[ACTION_OFFSET])
        {
        case EX_CHAN_SWITCH_ANNOUNCE:
        {
            handle_ch_switch_ie(IEXCHSWANNOUNC, data+2);
        }
        break;

        default:
        {
            /* Do nothing. Received action type is either not recognized */
            /* or does not need to be handled in STA mode.               */
        }
        break;
        }
    }
    break;

    default:
    {
        /* Do nothing. Received action type is either not recognized */
        /* or does not need to be handled in STA mode.               */
    }
    break;
    }

    /* All other protocol related spectrum management frames */
    handle_spec_mgmt_action_prot(msa, rx_len, mac_hdr_len, rx_dscr);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : switch_channel_csw                                       */
/*                                                                           */
/*  Description   : This function is called to switch the channel            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_current_channel                                        */
/*                  g_waiting_to_shift_channel                               */
/*                  g_channel_swt_cnt_zero                                   */
/*                  g_waiting_for_AP                                         */
/*                                                                           */
/*  Processing    : This function is used to switch the channel,             */
/*                  when beacons are stopped in the current channel          */
/*                  i.e. if Channel switch count in CSA frame is zero.       */
/*                                                                           */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void switch_channel_csw(void)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = get_ch_idx_from_num(freq, g_new_channel);

	TROUT_FUNC_ENTER;
	
    if(g_new_sec_chan_offset != 0xFF)
        process_sec_chan_offset(g_new_sec_chan_offset, BFALSE);

	/* Disable transmission of frames from the STA till the */
	/* channel switch is complete.                          */
	disable_machw_tx();

    select_channel_rf(ch_idx, get_bss_sec_chan_offset());
    mset_CurrentChannel(ch_idx);
    g_current_channel = ch_idx;
    g_current_sec_chan_offset = get_bss_sec_chan_offset();

    clear_wait_for_csa_prot();

	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Adopt new regulatory class after channel switch */
    adopt_new_reg_class(g_new_reg_class, g_current_channel,
                        g_current_sec_chan_offset);

    /* This global is to avoid link loss in the new channel */
    g_waiting_for_AP = BTRUE;

    /* Start an alarm of one minute. From the standard, the AP may take */
    /* that much of time to Scan a new channel for Radar                */
    start_alarm(g_csw_alarm_wait_for_AP, SCAN_DFS_TIME);

    /* STA switched to new channel so this global is false */
    g_channel_swt_cnt_zero     = BFALSE;
    g_waiting_to_shift_channel = BFALSE;
    g_new_sec_chan_offset      = 0xFF;
    g_new_reg_class            = INVALID_REG_CLASS; // 20120830 caisf add, merged ittiam mac v1.3 code

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_AP_wait_expiry                                        */
/*                                                                           */
/*  Description   : Expiry of Channel Move Time                              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_waiting_for_AP                                         */
/*                                                                           */
/*  Processing    : This function is called after expiry of 60secs           */
/*                  resets the g_waiting_for_AP global.                      */
/*                                                                           */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void on_AP_wait_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_AP_wait_expiry_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
   if( reset_mac_trylock() == 0)
   {
	printk("on_AP_wait work out\n");
	return;
   }
    ALARM_WORK_ENTRY(work);
    /* This global variable is used to avoid link loss */
    /* when STA is switched to the new channel         */
    g_waiting_for_AP = BFALSE;

	//Begin:add by wulei 2791 for bug 160423 on 2013-05-04
	printk("on_AP_wait work in\n");
	//End:add by wulei 2791 for bug 160423 on 2013-05-04
	
    /* Resume Tx */
    set_machw_tx_resume();
    enable_machw_ack_trans();
    enable_machw_cts_trans();
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}


#ifdef OS_LINUX_CSL_TYPE
void on_AP_wait_expiry(ADDRWORD_T data)
{
	//Begin:add by wulei 2791 for bug 160423 on 2013-05-04
	printk("on_AP_wait_expiry scheduling work\n");

	if((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE)
	{
		printk("on_AP_wait_expiry scheduling work out\n");
		return;
	}
	//End:add by wulei 2791 for bug 160423 on 2013-05-04
	alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_this_channel_supported_by_rf                          */
/*                                                                           */
/*  Description   : This function check whether the particular channel is    */
/*                  supported by STA or not.                                 */
/*                                                                           */
/*  Inputs        : Channel Number corresponding to channel being checked    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the channel number is valid and  */
/*                  is supported by the RF being used by the STA.            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : BTRUE - If the Channel is supported                      */
/*                  BFALSE - If the Channel is not supported                 */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_this_channel_supported_by_rf(UWORD8 channel)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 index  = 0;
    BOOL_T retval = BTRUE;

    index = get_ch_idx_from_num(freq, channel);

    if((is_ch_valid(index) == BFALSE) ||
       (is_ch_idx_supported_by_rf(freq, index) == BFALSE))
    {
        retval = BFALSE;
    }

    return retval;
}

#endif /* IBSS_BSS_STATION_MODE */
