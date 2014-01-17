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
/*  File Name         : x_timers_auth.h                                      */
/*                                                                           */
/*  Description       : This file contains the definitions of constants,     */
/*                      extern declarations related to the various timers    */
/*                      used by the 802.1x state meachines.                  */
/*                                                                           */
/*  List of Functions : create_1x_alarm                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

#ifndef X_TIMERS_H
#define X_TIMERS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "common.h"
#include "csl_if.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void start_quietWhile_timer(auth_t* auth_1x, UWORD32 period);
extern void stop_quietWhile_timer(auth_t* auth_1x);

extern void start_txWhen_timer(auth_t* auth_1x , UWORD32 period);
extern void stop_txWhen_timer(auth_t* auth_1x);


extern void start_reAuthWhen_timer(auth_t* auth_1x, UWORD32 period);
extern void stop_reAuthWhen_timer(auth_t* auth_1x);

extern void start_aWhile_timer(auth_t* auth_1x, UWORD32 period);
extern void stop_aWhile_timer(auth_t* auth_1x);

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
extern void aWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void reAuthWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void txWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void quietWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
extern void aWhile_alarm_fn(ADDRWORD_T data);
extern void aWhile_alarm_work(struct work_struct *work);    //add by Hugh
extern void reAuthWhen_alarm_fn(ADDRWORD_T data);
extern void reAuthWhen_alarm_work(struct work_struct *work); //add by Hugh

extern void txWhen_alarm_fn(ADDRWORD_T data);
extern void txWhen_alarm_work(struct work_struct *work);       //add by Hugh
extern void quietWhile_alarm_fn(ADDRWORD_T data);
extern void quietWhile_alarm_work(struct work_struct *work);   //add by Hugh

#endif /* ENABLE_AP_1X_LINUX_PROCESSING */


/*****************************************************************************/
/* Inline Function                                                           */
/*****************************************************************************/

INLINE ALARM_HANDLE_T *create_1x_alarm(ALARM_FUNC_T *func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func)
{
    return create_alarm(func, data, work_func);
}

#endif /* X_TIMERS_H */

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */

