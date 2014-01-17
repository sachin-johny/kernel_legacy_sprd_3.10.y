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
/*  File Name         : rkmal_sta.c                                          */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      RSNA Key management for STA mode.                    */
/*                                                                           */
/*  List of Functions : start_11i_supp_timer                                 */
/*                      stop_11i_supp_timer                                  */
/*                      supp_alarm_cb_fn                                     */
/*                      start_supp_cntr_msr_timer                            */
/*                      stop_supp_cntr_msr_timer                             */
/*                      supp_cntr_msr_cb_fn                                  */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "rkmal_sta.h"
#include "rsna_supp_km.h"
#include "ieee_11i_sta.h"
#include "tkip_sta.h"
#include "sme_sta.h"
#include "mac_init.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_11i_supp_timer                                  */
/*                                                                           */
/*  Description      : This function creates and starts a timer              */
/*                     with period equal to the Timeout Period               */
/*                                                                           */
/*  Inputs           : 1) RNSA Ptr                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T start_11i_supp_timer(rsna_supp_persta_t *rsna_ptr, UWORD32 time)
{
    rsna_ptr->alarm_handle = create_alarm(supp_alarm_cb_fn,
                                            (ADDRWORD_T)rsna_ptr, supp_alarm_cb_work);

    if(rsna_ptr->alarm_handle)
    {
    	start_alarm(rsna_ptr->alarm_handle, time);
    	return BTRUE;
	}
	else
	{
		return BFALSE;
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_11i_supp_timer                                   */
/*                                                                           */
/*  Description      : This function stops and deletes the given timer       */
/*                     with period equal to the Timeout Period               */
/*                                                                           */
/*  Inputs           : 1) RNSA Ptr                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T stop_11i_supp_timer(rsna_supp_persta_t *rsna_ptr)
{
    /* Stop the desired Alarm */
    if(rsna_ptr != NULL)
    {
        if(rsna_ptr->alarm_handle != NULL)
        {
            stop_alarm(rsna_ptr->alarm_handle);
            delete_alarm(&(rsna_ptr->alarm_handle));
            rsna_ptr->alarm_handle = NULL;
            return BTRUE;
        }
    }

    return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_alarm_cb_fn                                      */
/*                                                                           */
/*  Description      : This function implements the timer callback function  */
/*                     for 80211i timeout events                             */
/*                                                                           */
/*  Inputs           : 1) RNSA Ptr                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void supp_alarm_cb_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void supp_alarm_cb_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	rsna_supp_persta_t *rsna_ptr = (rsna_supp_persta_t *)(container_of(work, trout_timer_struct, work)->data);
#else
    rsna_supp_persta_t *rsna_ptr = (rsna_supp_persta_t *)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
		return;
    }
    ALARM_WORK_ENTRY(work);

    if(rsna_ptr->alarm_handle != NULL)
    {
        /* Stop the timer */
        stop_11i_supp_timer(rsna_ptr);

        /* Set timeout event to True */
        rsna_ptr->TimeoutEvt = BTRUE;

#ifdef SUPP_11I
    	if(BTRUE == g_int_supp_enable)
    	{
    	    supp_rsn_fsm_run(rsna_ptr);
    	}
#endif /* SUPP_11I */

        /* If the deauthentication bit is set, send Deauth frame */
        if(rsna_ptr->send_deauth == BTRUE)
        {
            rsna_send_deauth((UWORD8 *)rsna_ptr, 1);
        }
    }
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void supp_alarm_cb_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_supp_cntr_msr_timer                             */
/*                                                                           */
/*  Description      : This function creates and starts a timer              */
/*                     with period equal to the Timeout Period               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True if no counter measures invoked; false otherwise  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T start_supp_cntr_msr_timer(rsna_supp_persta_t *rsna_ptr, BOOL_T is_bcast_error)
{
    /* Send the michael failure report */
    send_mic_error_report(rsna_ptr, is_bcast_error);

    if(g_rsna_supp.global.cntr_msr_alarm_handle == NULL)
    {
        g_rsna_supp.global.cntr_msr_alarm_handle = create_alarm(supp_cntr_msr_cb_fn, 0, supp_cntr_msr_cb_work);

        /* Set quiet period */
        start_alarm(g_rsna_supp.global.cntr_msr_alarm_handle, 60 * 1000);

        return BTRUE;
    }
    else
    {
        /* An event of type MISC is loaded in the event Q to start */
        /* the counter measures                                    */
        misc_event_msg_t *misc       = 0;

        /* The Counter measure timer needs to be stopped */
        stop_supp_cntr_msr_timer();

        /* TBD - Verify if isr malloc is req */
        misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

        if(misc == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */

            /* Exception */
            raise_system_error(NO_EVENT_MEM);
            return BFALSE;
        }

        misc->data = NULL;
        misc->name = MISC_SUPP_TKIP_CNT_MSR;
        post_event((UWORD8*)misc, MISC_EVENT_QID);

        return BFALSE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_supp_cntr_msr_timer                              */
/*                                                                           */
/*  Description      : This function stops and deletes the counter measure   */
/*                     timer                                                 */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_supp_cntr_msr_timer(void)
{
    if(g_rsna_supp.global.cntr_msr_alarm_handle != NULL)
    {
        stop_alarm(g_rsna_supp.global.cntr_msr_alarm_handle);
        delete_alarm(&(g_rsna_supp.global.cntr_msr_alarm_handle));
        g_rsna_supp.global.cntr_msr_alarm_handle = NULL;
    }

        if(g_rsna_supp.global.cntr_msr_in_progress == BTRUE)
        {
            g_rsna_supp.global.cntr_msr_in_progress = BFALSE;
            if(mget_DesiredBSSType() == INFRASTRUCTURE)
            {
                initiate_scan(&g_mac);
            }
        }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : supp_cntr_msr_cb_fn                                   */
/*                                                                           */
/*  Description      : This function implements the timer callback function  */
/*                     for 80211i TKIP Counter measure                       */
/*                                                                           */
/*  Inputs           : 1) RNSA Ptr                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void supp_cntr_msr_cb_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void supp_cntr_msr_cb_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* The timeout indicates silently discard the previous MIC Failure */
    if( reset_mac_trylock() == 0 ){
		return;
    }
    ALARM_WORK_ENTRY(work);
    /* Stop the timer */
    stop_supp_cntr_msr_timer();
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE
void supp_cntr_msr_cb_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_supp_cntr_msr_60s_timer                         */
/*                                                                           */
/*  Description      : This function creates and starts a timer              */
/*                     with period equal to the Timeout Period               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True if no counter measures invoked; false otherwise  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T start_supp_cntr_msr_60s_timer(void)
{
    g_rsna_supp.global.cntr_msr_alarm_handle =
                                          create_alarm(supp_cntr_msr_cb_fn, 0, supp_cntr_msr_cb_work);

    if(g_rsna_supp.global.cntr_msr_alarm_handle == NULL)
    {
        return BFALSE;
    }

    /* Set quiet period */
    start_alarm(g_rsna_supp.global.cntr_msr_alarm_handle, 60 * 1000);

    return BTRUE;
}



#endif /* MAC_802_11I */

#endif /* IBSS_BSS_STATION_MODE */
