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
/*  File Name         : rkmal_auth.h                                         */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various features of OS used by Auth RSN Key mgmt     */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I
#ifndef RKMAL_AUTH_H
#define RKMAL_AUTH_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

/* Common includes */
#include "common.h"
#include "csl_if.h"
#include "imem_if.h"
#include "mib.h"
#include "mib_11i.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
typedef enum {
              DO_NEXT_REKEY    = 0,
              REKEY_TIMEOUT    = 1,

} REKEY_EVENT_TYPE_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD32 get_11i_resp_timeout(UWORD8 retry_cnt, UWORD16 listen_interval);
extern void   stop_rekey_timer         (void);
extern void   delete_rekey_timer       (void);
extern void   start_rekey_timer        (UWORD32 time, REKEY_EVENT_TYPE_T use);
extern void   stop_auth_cntr_msr_timer (void);
extern BOOL_T start_auth_cntr_msr_timer(void);
extern BOOL_T start_auth_cntr_msr_60s_timer(void);
extern void   rsna_auth_glk_handle_timeout(void);
extern void   post_rekey_event(REKEY_EVENT_TYPE_T event_type);

#ifndef OS_LINUX_CSL_TYPE
extern void auth_cntr_msr_cb_fn(HANDLE_T* alarm, ADDRWORD_T data);
extern void auth_alarm_cb_fn   (HANDLE_T* alarm, ADDRWORD_T data);
extern void rekey_cb_fn        (HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void rekey_cb_fn(ADDRWORD_T data);
extern void auth_cntr_msr_cb_fn(ADDRWORD_T data);
extern void auth_alarm_cb_fn   (ADDRWORD_T data);
extern void auth_alarm_cb_work(struct work_struct *work);
extern void rekey_cb_fn_for_timer(ADDRWORD_T data);
extern void rekey_cb_fn_for_timer_work(struct work_struct * work);

#endif /* OS_LINUX_CSL_TYPE */

#endif /* RKMAL_AUTH_H */

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */

