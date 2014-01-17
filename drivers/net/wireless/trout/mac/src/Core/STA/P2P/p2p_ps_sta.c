/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : p2p_ps_sta.c                                         */
/*                                                                           */
/*  Description       : This file contains the power save management         */
/*                      functions for P2P protocol in STA mode.              */
/*                                                                           */
/*  List of Functions : handle_p2p_go_absence_period_start                   */
/*                      handle_p2p_go_absence_period_end                     */
/*                      process_p2p_noa_attr                                 */
/*                      update_machw_noa_attr                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "sta_prot_if.h"
#include "mib_p2p.h"
#include "p2p_ps_sta.h"
#include "p2p_ps.h"
#include "mh.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T g_ps_poll_pending = BFALSE;
WORD16 g_noa_index = -1;


static void update_machw_noa_attr(BOOL_T ctwin_update_flag,
                                  BOOL_T noa_update_flag);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_p2p_go_absence_period_start                       */
/*                                                                           */
/*  Description   : This function handles the start of an Absent period.     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : MAC HW TX is aborted (not required with HW support).     */
/*                  Thereafter a Trigger and/or a PS-Poll frame is queued if */
/*                  a WMM Service Period is in progress and/or a PS-Poll     */
/*                  response is pending. This is to enable a Trigger/PS-Poll */
/*                  frame to be transmitted once the Absent period ends.     */
/*                  MAC HW and PHY are disabled and power down.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_p2p_go_absence_period_start(void)
{
    if(get_ps_state() != STA_DOZE)
    {
        /* Protocol dependent processing */
        p2p_go_absence_period_start_sta_prot();

        /* Check if a PS poll response is pending and queue a PS-Poll frame */
        if(BTRUE == is_ps_poll_rsp_pending())
        {
            wait_machw_suspend_state();
            send_ps_poll_to_AP();
        }

        /* Disable MAC HW and PHY */
        disable_machw_phy_and_pa();

        /* Power down the PHY */
        power_down_phy();
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_p2p_go_absence_period_end                         */
/*                                                                           */
/*  Description   : This function handles the end of an Absent period.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC HW and PHY are enabled, MAC HW transmit abort is */
/*                  disabled.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_p2p_go_absence_period_end(void)
{

    if(get_ps_state() != STA_DOZE)
    {
		/* Power up the PHY */
        power_up_phy();

        /* Enable MAC HW and PHY */
        enable_machw_phy_and_pa();
    }

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_noa_attr                                     */
/*                                                                           */
/*  Description   : This function processes the NOA attribute received.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the attributes                */
/*                  2) Length of the information element                     */
/*                                                                           */
/*  Globals       : g_noa_index                                              */
/*                  g_noa_dscr                                               */
/*                                                                           */
/*  Processing    : The NOA attribute is extracted from the given frame and  */
/*                  processed to update the CT Window parameter and the      */
/*                  global NOA schedule descriptors. In case the attribute   */
/*                  is not present or no NOA scheds are present, the global  */
/*                  schedule descriptors are reset. Note that updating is    */
/*                  done only if the instance of the received attribute is   */
/*                  different from the saved instance (checked using         */
/*                  g_noa_index). Also, a maximum of NUM_NOA_DSCR schedules  */
/*                  are only supported. Hence in case more NOA schedules are */
/*                  present in the frame, these are ignored.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_p2p_noa_attr(UWORD8 *attr_start_ptr, UWORD8 ie_len)
{
    UWORD8  num       = 0;
    UWORD8  noa_index = 0;
    UWORD16 index     = 0;
    UWORD16 attr_len  = 0;
    UWORD8  *noa      = 0;
    UWORD8  curr_ct_window = mget_p2p_CTW();
    BOOL_T  noa_update_flag = BFALSE;
    BOOL_T  ctwin_update_flag = BFALSE;

    noa = p2p_get_attr(NOTICE_OF_ABSENCE, attr_start_ptr, ie_len, &attr_len);

    /* If no NOA attribute is present then reset the CT Window and schedules */
    if(NULL == noa)
    {
        if(get_num_noa_sched() != 0)    /* NOA schedule was present earlier */
            noa_update_flag = BTRUE;

        if(curr_ct_window != 0) /* Non-zero CT Window was present earlier */
            ctwin_update_flag = BTRUE;

        mset_p2p_CTW(0);
        reset_noa_dscr();

        update_machw_noa_attr(ctwin_update_flag, noa_update_flag);

        return;
    }

    /* Extract the NOA attrbiute Index - will be checked later. Process      */
    /* CT Window first since the index will not be updated even if CT Window */
    /* is changed.                                                           */
    noa_index = noa[index++];

    /* Update the CT Window if the Opportunistic PS bit is set */
    if((noa[index] & BIT7) == 0)
    {
        mset_p2p_CTW(0); /* No OppPS */
    }
    else
    {
        mset_p2p_CTW(noa[index] & 0x7F);
    }

    if(curr_ct_window != mget_p2p_CTW()) /* CT Window has been modified */
        ctwin_update_flag = BTRUE;

    index++; /* Skip the CTWindow and OppPS Parameters Field */

    /* Check if the NOA attribute has changed by comparing the Index (which  */
    /* indicates the instance of the NOA schedule) with the saved global NOA */
    /* index. If it has not changed, return without doing anything.          */
    if(noa_index != g_noa_index)
    {
        /* Update the global NOA instance index with the received instance value */
        g_noa_index = noa_index;

        /* NOA schedule has been modified */
        noa_update_flag = BTRUE;

	    /* Reset the NOA descriptors */
	    reset_noa_dscr();
	
	    /* Process the attribute till the end is reached */
	    while(index < attr_len)
	    {
	        g_noa_dscr[num].cnt_typ    = noa[index++];
	
	        g_noa_dscr[num].duration   = MAKE_WORD32(MAKE_WORD16(noa[index],
	                                     noa[index + 1]),
	                                     MAKE_WORD16(noa[index + 2],
	                                     noa[index + 3]));
	        index += 4;
	
	        g_noa_dscr[num].interval   = MAKE_WORD32(MAKE_WORD16(noa[index],
	                                     noa[index + 1]),
	                                     MAKE_WORD16(noa[index + 2],
	                                     noa[index + 3]));
	        index += 4;
	
	        g_noa_dscr[num].start_time = MAKE_WORD32(MAKE_WORD16(noa[index],
	                                     noa[index + 1]),
	                                     MAKE_WORD16(noa[index + 2],
	                                     noa[index + 3]));
	        index += 4;
	
	        /* Increment the number of NOA schedules extracted */
	        num++;
	
	        /* NUM_NOA_DSCR NOA schedules are processed, Remaining are ignored */
	        if (num >= NUM_NOA_DSCR)
	            break;
	    }

	    /* Update the number of NOA schedules */
	    set_num_noa_sched(num);
	}

    /* Update MAC HW with the NOA attribute values */
    update_machw_noa_attr(ctwin_update_flag, noa_update_flag);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : program_noa_schedule                                     */
/*                                                                           */
/*  Description   : This function programs the Noa schedule                  */
/*                                                                           */
/*  Inputs        : 1) Index of the Noa attribute                            */
/*                                                                           */
/*  Globals       : g_noa_dscr                                               */
/*                                                                           */
/*  Processing    : This functions programs the HW registrar with the actaul */
/*                  Noa schedule. It finds out the start time, count and     */
/*                  duration for schedules which have already started in the */
/*                  past                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void program_noa_schedule(UWORD8 idx)
{
    UWORD8  count                    = 1;
    UWORD8  ab_period_cnt            = 0;
    UWORD32 start_time_offset        = 0;
    UWORD32 end_time_offset          = 0;
    UWORD32 start_time               = 0;
    UWORD32 tsf_time          = 0;
    UWORD32 end_time                 = 0;
    UWORD32 duration                 = 0;
    UWORD32 absent_period_start_time = g_noa_dscr[idx].start_time;
    while(1)
    {
        /* Compute the start time based on the current TSF value and the NOA */
        /* schedule interval start time, taking care of wrap-around.         */
        tsf_time = get_machw_tsf_timer_lo() + AB_PERIOD_START_TIME_OFFSET;
    	start_time_offset = (tsf_time ^ 0xFFFFFFFF) + 1;
        start_time = (UWORD32)(absent_period_start_time + start_time_offset);

        /* Check if start time is in the past */
        if(start_time >= (1 << 31))
    	{
            /* Check if it is non-periodic NoA schedule */
            if(1 == g_noa_dscr[0].cnt_typ)
        	{
				end_time        = absent_period_start_time + g_noa_dscr[idx].duration;
				end_time_offset = (end_time ^ 0xFFFFFFFF) + 1;
				duration        = end_time + end_time_offset;

				/* Check if the schedule is still is progress */
				if(duration <= g_noa_dscr[idx].duration)
        		{
					/* Program the hardware */
            		set_machw_noa_status(idx, NOA_STAT_NEW);
					set_machw_noa_sched_dscr(idx, duration,
                                     g_noa_dscr[idx].interval,
                                     g_noa_dscr[idx].cnt_typ,
                                     tsf_time);

				}

				break;
    		}
			else
			{
                ab_period_cnt++ ;
                /* Check if absence schedule should continue by checking if the  */
				/* scheduled absent period count is 255 or the number of elapsed */
				/* absent periods are less than scheduled absent period count.   */
				if((0xFF == g_noa_dscr[idx].cnt_typ ) ||
				   (ab_period_cnt < g_noa_dscr[idx].cnt_typ))
        		{
					/* Update the global start time to the next interval start */
					absent_period_start_time += g_noa_dscr[0].interval;
        		}
        		else
        		{
					break;
    			}
			}

    }
    else
    {
		count = g_noa_dscr[idx].cnt_typ;
		if((g_noa_dscr[idx].cnt_typ != 1) && (g_noa_dscr[idx].cnt_typ != 255))
        {
				count -=  ab_period_cnt;
        }
		/* Program the hardware */
		set_machw_noa_status(idx, NOA_STAT_NEW);
		set_machw_noa_sched_dscr(idx,
		                         g_noa_dscr[idx].duration,
								 g_noa_dscr[idx].interval,
								 count,
                                 absent_period_start_time);
            break;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_machw_noa_attr                                    */
/*                                                                           */
/*  Description   : This function updates the required MAC HW registers      */
/*                  with the NOA attributes.                                 */
/*                                                                           */
/*  Inputs        : 1) CT Window update flag                                 */
/*                  2) NOA schedule update flag                              */
/*                                                                           */
/*  Globals       : g_noa_dscr                                               */
/*                                                                           */
/*  Processing    : The required MAC HW registers are updated with the CT    */
/*                  Window value and NOA schedule information.               */
/*                  Currently there is an option to support P2P PS without   */
/*                  HW support. In this case the absent period timer is      */
/*                  started after initializing the absent period count. This */
/*                  support shall be removed in the future.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_machw_noa_attr(BOOL_T ctwin_update_flag, BOOL_T noa_update_flag)
{
    UWORD8  idx               = 0;

    if(ctwin_update_flag == BTRUE)
        set_machw_CTwindow(mget_p2p_CTW());

    if(noa_update_flag == BFALSE)
        return;

    /* If NOA update is required, cancel all existing NOA schedules in MAC HW */
    set_machw_cancel_noa();

    /* Update the MAC HW with all the NOA schedules present */
    for(idx = 0; idx < NUM_NOA_DSCR; idx++)
	{
        if(is_noa_sched_null(idx) == BFALSE)
    	{
            program_noa_schedule(idx);
        }
    }

    if(0 != get_num_noa_sched())
        reset_machw_cancel_noa();
}

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
