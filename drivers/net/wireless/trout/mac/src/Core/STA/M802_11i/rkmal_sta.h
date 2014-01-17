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
/*  File Name         : rkmal_sta.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various features of OS used by STA RSN Key management*/
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

#ifndef RKMAL_STA_H
#define RKMAL_STA_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "common.h"
#include "csl_if.h"
#include "imem_if.h"
#include "mib.h"
#include "mib_11i.h"
#include "rsna_supp_km.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   stop_supp_cntr_msr_timer (void);
extern BOOL_T start_supp_cntr_msr_timer(rsna_supp_persta_t *, BOOL_T);
extern BOOL_T start_supp_cntr_msr_60s_timer(void);

#ifndef OS_LINUX_CSL_TYPE
extern void   supp_alarm_cb_fn   (HANDLE_T* alarm, ADDRWORD_T data);
extern void   supp_cntr_msr_cb_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void supp_alarm_cb_fn   (ADDRWORD_T data);
extern void supp_alarm_cb_work(struct work_struct *work);

extern void supp_cntr_msr_cb_fn(ADDRWORD_T data);
extern void supp_cntr_msr_cb_work(struct work_struct *work);

#endif /* OS_LINUX_CSL_TYPE */

#endif /* RKMAL_STA_H */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
