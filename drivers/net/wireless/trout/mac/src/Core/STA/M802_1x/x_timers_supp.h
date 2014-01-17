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
/*  File Name         : x_timers_supp.h                                      */
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

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

#ifndef X_TIMERS_SUPP_H
#define X_TIMERS_SUPP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "common.h"
#include "csl_if.h"
#include "ieee_supp_1x.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void start_authWhile_timer(supp_t* supp_1x, UWORD32 period);
extern void stop_authWhile_timer(supp_t* supp_1x);

extern void start_heldWhile_timer(supp_t* supp_1x , UWORD32 period);
extern void stop_heldWhile_timer(supp_t* supp_1x);

extern void start_startWhen_timer(supp_t* supp_1x, UWORD32 period);
extern void stop_startWhen_timer(supp_t* supp_1x);

#ifndef OS_LINUX_CSL_TYPE
extern void authWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void heldWhile_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void startWhen_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void authWhile_alarm_fn(ADDRWORD_T data);
extern void authWhile_alarm_work(struct work_struct *work);     //add by Hugh
extern void heldWhile_alarm_fn(ADDRWORD_T data);
extern void heldWhile_alarm_work(struct work_struct *work);     //add by Hugh
extern void startWhen_alarm_fn(ADDRWORD_T data);
extern void startWhen_alarm_work(struct work_struct *work);     //add by Hugh
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Inline Function                                                           */
/*****************************************************************************/

INLINE ALARM_HANDLE_T *create_1x_alarm(ALARM_FUNC_T *func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func)
{
    return create_alarm(func, data, work_func);
}

#endif /* X_TIMERS_SUPP_H */
#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */

