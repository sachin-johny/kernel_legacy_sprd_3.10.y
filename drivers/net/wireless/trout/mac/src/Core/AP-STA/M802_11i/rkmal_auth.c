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
/*  File Name         : rkmal_auth.c                                         */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various features of eCos used by RSNA Key management.*/
/*                                                                           */
/*  List of Functions : start_11i_auth_timer                                 */
/*                      stop_11i_auth_timer                                  */
/*                      auth_alarm_cb_fn                                     */
/*                      start_rekey_timer                                    */
/*                      stop_rekey_timer                                     */
/*                      rekey_cb_fn                                          */
/*                      start_auth_cntr_msr_timer                            */
/*                      stop_auth_cntr_msr_timer                             */
/*                      auth_cntr_msr_cb_fn                                  */
/*                      get_11i_resp_timeout                                 */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mac_init.h"
#include "ieee_11i_auth.h"
#include "rkmal_auth.h"
#include "rsna_auth_km.h"

#ifdef AUTH_11I

/*****************************************************************************/
/* Static Global variable                                                    */
/*****************************************************************************/
static REKEY_EVENT_TYPE_T g_rekey_event_type = DO_NEXT_REKEY;

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_11i_resp_timeout                                  */
/*                                                                           */
/*  Description      : This function returns 802.11i response timeout value. */
/*                                                                           */
/*  Inputs           : 1) Retry count                                        */
/*                     2) Listen interval                                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks the retry count and based on the */
/*                     value returns the 802.11i response timeout value.     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD32, 11i response timeout value                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 get_11i_resp_timeout(UWORD8 retry_cnt, UWORD16 listen_interval)
{
    UWORD32 ret_val = 0;
    UWORD32 ps_time = 0;

    switch(retry_cnt)
    {
        case 1:
        {
            ret_val = g_eapol_response_timeout;
        }
        break;
        case 2:
        {
            /* First retry must be after half of the listen interval       */
            /* If the listen interval is not set, then the value of 100 ms */
            /* should be used for all the timeouts                         */
            if(listen_interval)
            {
                ps_time = (mget_BeaconPeriod() * listen_interval) >> 1;
                ret_val = MAX(ps_time,g_eapol_response_timeout);
            }
            else
            {
                ret_val = g_eapol_response_timeout;
            }
            break;
        }
        default:
        {
            /* All other retrys must be after the listen interval          */
            /* If the listen interval is not set, then the value of 100 ms */
            /* should be used for all the timeouts                         */
            if(listen_interval)
            {
                ps_time = mget_BeaconPeriod() * listen_interval;
                ret_val = MAX(ps_time,g_eapol_response_timeout);
            }
            else
            {
                ret_val = g_eapol_response_timeout;
            }
            break;
        }
    }
    ret_val = (ret_val > 310)?ret_val : 310; /* WI-FI Kludge min TO = 310ms  */
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_11i_auth_timer                                  */
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

BOOL_T start_11i_auth_timer(rsna_auth_persta_t *rsna_ptr, UWORD32 time)
{
	critical_section_start();
	if(rsna_ptr->alarm_handle)
	{
         stop_alarm(rsna_ptr->alarm_handle);
	}
	else
	{
    	rsna_ptr->alarm_handle = create_alarm(auth_alarm_cb_fn,
                                            (ADDRWORD_T)rsna_ptr, auth_alarm_cb_work);
	}
	critical_section_end();

    if(rsna_ptr->alarm_handle)
    {
    	start_alarm(rsna_ptr->alarm_handle, time);
    	return BTRUE;
	}

	return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_11i_auth_timer                                   */
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

BOOL_T stop_11i_auth_timer(rsna_auth_persta_t *rsna_ptr)
{
    /* Stop the desired Alarm */
    if(rsna_ptr != NULL)
    {
        if(rsna_ptr->alarm_handle != NULL)
        {
    		critical_section_start();
            stop_alarm(rsna_ptr->alarm_handle);
            delete_alarm(&(rsna_ptr->alarm_handle));
            rsna_ptr->alarm_handle = NULL;
    		critical_section_end();
            return BTRUE;
        }
    }

    return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_alarm_cb_fn                                      */
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
void auth_alarm_cb_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void auth_alarm_cb_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	rsna_auth_persta_t *rsna_ptr = (rsna_auth_persta_t *)(container_of(work, trout_timer_struct, work)->data);
#else
    rsna_auth_persta_t *rsna_ptr = (rsna_auth_persta_t *)data;
#endif /* OS_LINUX_CSL_TYPE */

    misc_event_msg_t *misc       = 0;   
    if( reset_mac_trylock() == 0 )
    {
	 return;
    }
    ALARM_WORK_ENTRY(work);
    if(rsna_ptr != NULL)
    {   
        misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

        if(misc == NULL)
        {   
#ifdef DEBUG_MODE
            g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */
	  ALARM_WORK_EXIT(work);
	  reset_mac_unlock();
            return;
        }   

        printk("@@@auth_alarm_cb_work post events!@@@\n");
        misc->data = (UWORD8*)rsna_ptr;
        misc->name = MISC_RSNA_TIMEOUT;
        post_event((UWORD8*)misc, MISC_EVENT_QID);
    }
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void auth_alarm_cb_fn(ADDRWORD_T data)
{
     alarm_fn_work_sched(data);
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_rekey_timer                                     */
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

void start_rekey_timer(UWORD32 time, REKEY_EVENT_TYPE_T event_type)
{
	g_rekey_event_type = event_type;

    if(g_rsna_auth.global.alarm_handle == NULL)
    {
    	g_rsna_auth.global.alarm_handle = create_alarm(rekey_cb_fn_for_timer, 0, rekey_cb_fn_for_timer_work);
    }
    else
    {
        stop_alarm(g_rsna_auth.global.alarm_handle);
    }

    /* Start the alarm */
    if(g_rsna_auth.global.alarm_handle)
	{
    	start_alarm(g_rsna_auth.global.alarm_handle, time);
	}
    else
    {
        raise_system_error(NO_LOCAL_MEM);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_rekey_timer                                      */
/*                                                                           */
/*  Description      : This function stops the given timer                   */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_rekey_timer(void)
{
    if(g_rsna_auth.global.alarm_handle != NULL)
    {
        stop_alarm(g_rsna_auth.global.alarm_handle);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : delete_rekey_timer                                    */
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

void delete_rekey_timer(void)
{
    if(g_rsna_auth.global.alarm_handle != NULL)
    {
        stop_alarm(g_rsna_auth.global.alarm_handle);
        delete_alarm(&(g_rsna_auth.global.alarm_handle));
        g_rsna_auth.global.alarm_handle = NULL;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rekey_cb_fn                                           */
/*                                                                           */
/*  Description      : This function implements the timer callback function  */
/*                     for 80211i REKEY timeout event                        */
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
void rekey_cb_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void rekey_cb_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
	post_rekey_event(g_rekey_event_type);
}

//add by Hugh
#ifdef OS_LINUX_CSL_TYPE
void rekey_cb_fn_for_timer_work(struct work_struct *work)
{
	if( reset_mac_trylock() == 0 ) return;
         ALARM_WORK_ENTRY(work);
	post_rekey_event(g_rekey_event_type);
	ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

void rekey_cb_fn_for_timer(ADDRWORD_T data)
{
     alarm_fn_work_sched(data);
}

#endif


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : post_rekey_event                                      */
/*                                                                           */
/*  Description      : This function implements posting an event in case of  */
/*                     a rekey timer expiry or rekey hs timeout expiry       */
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
void post_rekey_event(REKEY_EVENT_TYPE_T event_type)
{
    /* An event of type MISC is loaded in the event Q to start the rekeying */
    misc_event_msg_t *misc       = 0;

    /* Stop the timer */
    stop_rekey_timer();

	/* Set the REKY time out Flag if the timer was called for it */
    if(event_type == REKEY_TIMEOUT)
    {
        /* Set the REKY Flag */
		g_rsna_auth.global.GTKReKeyTimeOut = BTRUE;
    }
	/* Else Set the REKY Flag */
	else
	{
        g_rsna_auth.global.GTKReKey = BTRUE;
	}

    /* Run the 802.11i state machine */
    auth_rsn_fsm_run(NULL);

    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */

        return;
    }

    misc->data = NULL;
    misc->name = MISC_STRT_REKEY;
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_auth_cntr_msr_timer                             */
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

BOOL_T start_auth_cntr_msr_timer(void)
{
    if(g_rsna_auth.global.cntr_msr_alarm_handle == NULL)
    {
        g_rsna_auth.global.cntr_msr_alarm_handle =
                                        create_alarm(auth_cntr_msr_cb_fn, 0, NULL);

        /* Set quiet period */
        start_alarm(g_rsna_auth.global.cntr_msr_alarm_handle, 60 * 1000);

        return BTRUE;
    }
    else
    {
        /* An event of type MISC is loaded in the event Q to start */
        /* the counter measures                                    */

        misc_event_msg_t *misc       = 0;

        /* The Counter measure timer needs to be stopped */
        stop_auth_cntr_msr_timer();

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
        misc->name = MISC_AUTH_TKIP_CNT_MSR;
        post_event((UWORD8*)misc, MISC_EVENT_QID);

        return BFALSE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_auth_cntr_msr_timer                              */
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

void stop_auth_cntr_msr_timer(void)
{
    if(g_rsna_auth.global.cntr_msr_alarm_handle != NULL)
    {
        stop_alarm(g_rsna_auth.global.cntr_msr_alarm_handle);
        delete_alarm(&(g_rsna_auth.global.cntr_msr_alarm_handle));
        g_rsna_auth.global.cntr_msr_alarm_handle = NULL;
        g_rsna_auth.global.cntr_msr_in_progress = BFALSE;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : auth_cntr_msr_cb_fn                                   */
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
void auth_cntr_msr_cb_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void auth_cntr_msr_cb_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* The timeout indicates silently discard the previous MIC Failure */

    /* Stop the timer */
    if(g_rsna_auth.global.cntr_msr_alarm_handle != NULL)
    {
        stop_alarm_self(g_rsna_auth.global.cntr_msr_alarm_handle);
        delete_alarm_self(&(g_rsna_auth.global.cntr_msr_alarm_handle));
        g_rsna_auth.global.cntr_msr_alarm_handle = NULL;
        g_rsna_auth.global.cntr_msr_in_progress = BFALSE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_auth_cntr_msr_60s_timer                         */
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

BOOL_T start_auth_cntr_msr_60s_timer(void)
{
    g_rsna_auth.global.cntr_msr_alarm_handle =
                                          create_alarm(auth_cntr_msr_cb_fn, 0, NULL);

    if(g_rsna_auth.global.cntr_msr_alarm_handle == NULL)
    {
        return BFALSE;
    }

    /* Set quiet period */
    start_alarm(g_rsna_auth.global.cntr_msr_alarm_handle, 60 * 1000);

    return BTRUE;
}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
