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
/*  File Name         : radius_timers.h                                      */
/*                                                                           */
/*  Description       : This file contains the definitions of constants,     */
/*                      extern declarations related to the various timers    */
/*                      used by the radius state meachines.                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

#ifndef RADIUS_TIMERS_H
#define RADIUS_TIMERS_H



/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "common.h"
#include "csl_if.h"
#include "radius_client.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define RADIUS_RESP_TIMEOUT   2000

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void start_radius_timer(radius_node_t* rad_node, UWORD32 period);
extern void stop_radius_timer (radius_node_t* rad_node);

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
extern void radius_alarm_fn   (ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
extern void radius_alarm_fn   (ADDRWORD_T data);
extern void radius_alarm_work(struct work_struct *work);
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

#endif /* RADIUS_TIMERS_H */
#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
