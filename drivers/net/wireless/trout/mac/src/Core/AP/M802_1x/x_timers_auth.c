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
/*  File Name         : x_timers_auth.c                                      */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various timers used by the 802.1x state meachines.   */
/*                                                                           */
/*  List of Functions : start_quietWhile_timer                               */
/*                      stop_quietWhile_timer                                */
/*                      quietWhile_alarm_fn                                  */
/*                      start_txWhen_timer                                   */
/*                      stop_txWhen_timer                                    */
/*                      txWhen_alarm_fn                                      */
/*                      start_reAuthWhen_timer                               */
/*                      stop_reAuthWhen_timer                                */
/*                      reAuthWhen_alarm_fn                                  */
/*                      start_aWhile_timer                                   */
/*                      stop_aWhile_timer                                    */
/*                      aWhile_alarm_fn                                      */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifdef MAC_802_11I

#ifdef MAC_802_1X


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "cglobals.h"
/* Common Includes */
#include "itypes.h"

/* 802.1x Includes */
#include "ieee_auth_1x.h"
#include "x_timers_auth.h"

/* OSAL Includes */
#include "csl_if.h"
#include "mac_init.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_quietWhile_timer                                */
/*                                                                           */
/*  Description      : This function creates and starts a quietWhile timer   */
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

void start_quietWhile_timer(auth_t* auth_1x, UWORD32 period)
{
    auth_1x->global.quietWhile_alarm_handle =
        create_1x_alarm(quietWhile_alarm_fn, (UWORD32)auth_1x, quietWhile_alarm_work);

    start_alarm(auth_1x->global.quietWhile_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_quietWhile_timer                                 */
/*                                                                           */
/*  Description      : This function stops and deletes a quietWhile timer    */
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

void stop_quietWhile_timer(auth_t* auth_1x)
{
    if((auth_1x != NULL) && (auth_1x->global.quietWhile_alarm_handle != NULL))
    {
        stop_alarm(auth_1x->global.quietWhile_alarm_handle);
        delete_alarm(&(auth_1x->global.quietWhile_alarm_handle));
        auth_1x->global.quietWhile_alarm_handle = NULL;
    }
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

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
void quietWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
void quietWhile_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	auth_t *auth_1x = (auth_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    auth_t *auth_1x = (auth_t*)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
	    return;
    }
	ALARM_WORK_ENTRY(work);
    /* Indicate timer completion by resetting the quietWhile timer variable  */
    auth_1x->global.quietWhileTimeout = BTRUE;

    stop_quietWhile_timer(auth_1x);

    auth_onex_fsm_run(auth_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef ENABLE_AP_1X_LINUX_PROCESSING    //add by Hugh

void quietWhile_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_txWhen_timer                                    */
/*                                                                           */
/*  Description      : This function creates and starts a txWhen timer       */
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

void start_txWhen_timer(auth_t* auth_1x, UWORD32 period)
{
    auth_1x->global.txWhen_alarm_handle =
        create_1x_alarm(txWhen_alarm_fn, (UWORD32)auth_1x, txWhen_alarm_work);

    /* Set txWhen period */
    start_alarm(auth_1x->global.txWhen_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_txWhen_timer                                     */
/*                                                                           */
/*  Description      : This function stops and deletes a txWhen timer        */
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

void stop_txWhen_timer(auth_t* auth_1x)
{
    if((auth_1x != NULL) && (auth_1x->global.txWhen_alarm_handle != NULL))
    {
        stop_alarm(auth_1x->global.txWhen_alarm_handle);
        delete_alarm(&(auth_1x->global.txWhen_alarm_handle));
        auth_1x->global.txWhen_alarm_handle = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : txWhen_alarm_fn                                       */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     txWhen timer. Authenication state machine is also     */
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

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
void txWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
void txWhen_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	auth_t *auth_1x = (auth_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    auth_t *auth_1x = (auth_t *)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
		return;
    	}
	ALARM_WORK_ENTRY(work);
    /* Indicate timer completion by resetting the txWhen timer variable  */
    auth_1x->global.txWhenTimeout = BTRUE;

    stop_txWhen_timer(auth_1x);

    auth_onex_fsm_run(auth_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef ENABLE_AP_1X_LINUX_PROCESSING    //add by Hugh

void txWhen_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_reAuthWhen_timer                                */
/*                                                                           */
/*  Description      : This function creates and starts a reAuthWhen timer   */
/*                     with period equal to the reauthentication period.     */
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

void start_reAuthWhen_timer(auth_t* auth_1x, UWORD32 period)
{
    auth_1x->global.reAuthWhen_alarm_handle =
        create_1x_alarm(reAuthWhen_alarm_fn, (UWORD32)auth_1x, reAuthWhen_alarm_work); //Hugh

    /* reAuthWhen is set to reAuthPeroid prior to calling this function      */
    start_alarm(auth_1x->global.reAuthWhen_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_reAuthWhen_timer                                 */
/*                                                                           */
/*  Description      : This function stops and deletes a reAuthWhen timer    */
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

void stop_reAuthWhen_timer(auth_t* auth_1x)
{
    if((auth_1x != NULL) && (auth_1x->global.reAuthWhen_alarm_handle != NULL))
    {
        stop_alarm(auth_1x->global.reAuthWhen_alarm_handle);
        delete_alarm(&(auth_1x->global.reAuthWhen_alarm_handle));
        auth_1x->global.reAuthWhen_alarm_handle = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reAuthWhen_alarm_fn                                   */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     reAuthWhen timer. Authenication state machine is also */
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

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
void reAuthWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
void reAuthWhen_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	auth_t *auth_1x = (auth_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    auth_t *auth_1x = (auth_t*)data;
#endif /* OS_LINUX_CSL_TYPE */
    if( reset_mac_trylock() == 0 ){
		return;
    	}
     ALARM_WORK_ENTRY(work);
    /* Indicate timer completion by resetting the reAuthWhen timer variable  */
    auth_1x->global.reAuthWhenTimeout       = BTRUE;

    stop_reAuthWhen_timer(auth_1x);

    auth_onex_fsm_run(auth_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef ENABLE_AP_1X_LINUX_PROCESSING    //add by Hugh

void reAuthWhen_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_aWhile_timer                                    */
/*                                                                           */
/*  Description      : This function creates and starts a aWhile timer       */
/*                     with period equal to the serverTimeout period.        */
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

void start_aWhile_timer(auth_t* auth_1x, UWORD32 period)
{
    auth_1x->global.aWhile_alarm_handle =
        create_1x_alarm(aWhile_alarm_fn, (UWORD32)auth_1x, aWhile_alarm_work); //Hugh

    /* aWhile is set to serverTimeout prior to calling this function        */
    start_alarm(auth_1x->global.aWhile_alarm_handle, period);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_aWhile_timer                                     */
/*                                                                           */
/*  Description      : This function stops and deletes a aWhile timer        */
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

void stop_aWhile_timer(auth_t* auth_1x)
{
    if((auth_1x != NULL) && (auth_1x->global.aWhile_alarm_handle != NULL))
    {
        stop_alarm(auth_1x->global.aWhile_alarm_handle);
        delete_alarm(&(auth_1x->global.aWhile_alarm_handle));
        auth_1x->global.aWhile_alarm_handle = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : aWhile_alarm_fn                                       */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     aWhile timer. Authenication state machine is also     */
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

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
void aWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
void aWhile_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	auth_t *auth_1x = (auth_t *)(container_of(work, trout_timer_struct, work)->data);
#else	
    auth_t *auth_1x = (auth_t*)data;
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
    if( reset_mac_trylock() == 0 ) {
		return;
    }
    ALARM_WORK_ENTRY(work);
    /* Indicate timer completion by resetting the aWhile timer variable     */
    auth_1x->global.aWhileTimeout = BTRUE;

    stop_aWhile_timer(auth_1x);

    auth_onex_fsm_run(auth_1x);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef ENABLE_AP_1X_LINUX_PROCESSING    //add by Hugh

void aWhile_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
