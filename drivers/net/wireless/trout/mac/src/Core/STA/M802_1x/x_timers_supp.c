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
/*  File Name         : x_timers_supp.c                                      */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various timers used by the 802.1x state meachines.   */
/*                                                                           */
/*  List of Functions : start_authWhile_timer                                */
/*                      stop_authWhile_timer                                 */
/*                      authWhile_alarm_fn                                   */
/*                      start_heldWhile_timer                                */
/*                      stop_heldWhile_timer                                 */
/*                      heldWhile_alarm_fn                                   */
/*                      start_startWhen_timer                                */
/*                      stop_startWhen_timer                                 */
/*                      startWhen_alarm_fn                                   */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals.h"
/* Common Includes */
#include "itypes.h"

/* 802.1x Includes */
#include "ieee_supp_1x.h"
#include "x_timers_supp.h"

/* OSAL Includes */
#include "csl_if.h"
#include "mac_init.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_authWhile_timer                                 */
/*                                                                           */
/*  Description      : This function creates and starts a authWhile timer    */
/*                     with period equal to the QUIET PERIOD.                */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void start_authWhile_timer(supp_t* supp_1x , UWORD32 period)
{
    supp_1x->global.authWhile_alarm_handle =
        create_1x_alarm(authWhile_alarm_fn, (UWORD32)supp_1x, authWhile_alarm_work); //Hugh

    /* Set quiet period */
    start_alarm(supp_1x->global.authWhile_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_authWhile_timer                                  */
/*                                                                           */
/*  Description      : This function stops and deletes a authWhile timer     */
/*                     alarm resources.                                      */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_authWhile_timer(supp_t* supp_1x)
{
    stop_alarm(supp_1x->global.authWhile_alarm_handle);
    delete_alarm(&(supp_1x->global.authWhile_alarm_handle));
    supp_1x->global.authWhile_alarm_handle = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : quietWhile_alarm_fn                                   */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     quietWhile timer. Authenication state machine is also */
/*                     run here.                                             */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void authWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void authWhile_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef OS_LINUX_CSL_TYPE
	supp_t *supp_1x = (supp_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    supp_t *supp_1x = (supp_t*)data;
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
    if( reset_mac_trylock() == 0 ){
		return;
    }
    ALARM_WORK_ENTRY(work);
    /* Indicate timer completion by resetting the quietWhile timer variable  */
    supp_1x->global.authWhileTimeout = BTRUE;

    stop_authWhile_timer(supp_1x);

    supp_onex_fsm_run(supp_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void authWhile_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_heldWhile_timer                                 */
/*                                                                           */
/*  Description      : This function creates and starts a heldWhile timer    */
/*                     with period equal to the given period.                */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                     2) The period of the timer                            */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void start_heldWhile_timer(supp_t* supp_1x , UWORD32 period)
{
    supp_1x->global.heldWhile_alarm_handle =
        create_1x_alarm(heldWhile_alarm_fn, (UWORD32)supp_1x, heldWhile_alarm_work);

    /* Set txWhen period */
    start_alarm(supp_1x->global.heldWhile_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_heldWhile_timer                                  */
/*                                                                           */
/*  Description      : This function stops and deletes a heldWhile timer     */
/*                     alarm resources.                                      */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_heldWhile_timer(supp_t* supp_1x)
{
    if(supp_1x->global.heldWhile_alarm_handle != NULL)
    {
        stop_alarm(supp_1x->global.heldWhile_alarm_handle);
        delete_alarm(&(supp_1x->global.heldWhile_alarm_handle));
        supp_1x->global.heldWhile_alarm_handle = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : heldWhile_alarm_fn                                    */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     heldWhile timer. Authenication state machine is also  */
/*                     run here.                                             */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void heldWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void heldWhile_alarm_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	supp_t *supp_1x = (supp_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    supp_t *supp_1x = (supp_t*)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
		return;
    }
    ALARM_WORK_ENTRY(work);
    stop_heldWhile_timer(supp_1x);

    /* Indicate timer completion by resetting the txWhen timer variable  */
    supp_1x->global.heldWhileTimeout = BTRUE;

    supp_onex_fsm_run(supp_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void heldWhile_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_startWhen_timer                                 */
/*                                                                           */
/*  Description      : This function creates and starts a startWhen timer    */
/*                     with period equal to the given period                 */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void start_startWhen_timer(supp_t* supp_1x, UWORD32 period)
{
    supp_1x->global.startWhen_alarm_handle =
        create_1x_alarm(startWhen_alarm_fn, (UWORD32)supp_1x, startWhen_alarm_work);

    /* aWhile is set to serverTimeout prior to calling this function        */
    start_alarm(supp_1x->global.startWhen_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_startWhen_timer                                  */
/*                                                                           */
/*  Description      : This function stops and deletes a startWhen timer     */
/*                     alarm resources.                                      */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_startWhen_timer(supp_t* supp_1x)
{
    if(supp_1x->global.startWhen_alarm_handle != NULL)
    {
        stop_alarm(supp_1x->global.startWhen_alarm_handle);
        delete_alarm(&(supp_1x->global.startWhen_alarm_handle));
        supp_1x->global.startWhen_alarm_handle = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : startWhen_alarm_fn                                    */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     startWhen timer. Authenication state machine is also  */
/*                     run here.                                             */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void startWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void startWhen_alarm_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	supp_t *supp_1x = (supp_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    supp_t *supp_1x = (supp_t*)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
		return;
    }
    ALARM_WORK_ENTRY(work);
    stop_startWhen_timer(supp_1x);

    /* Indicate timer completion by resetting the aWhile timer variable */
    supp_1x->global.startWhenTimeout = BTRUE;

    supp_onex_fsm_run(supp_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void startWhen_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
