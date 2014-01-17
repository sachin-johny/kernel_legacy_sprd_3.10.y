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
/*  File Name         : mh_test_txabort.h                                    */
/*                                                                           */
/*  Description       : This file contains the test code for Tx Abort        */
/*                      feature related function definations.                */
/*                                                                           */
/*  List of Functions : None.                                                */
/*                                                                           */
/*                                                                           */
/*  Issues            : None.                                                */
/*                                                                           */
/*****************************************************************************/

#ifdef TEST_TX_ABORT_FEATURE
#ifdef TX_ABORT_FEATURE

#ifndef MH_TEST_TXABORT_H
#define MH_TEST_TXABORT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "itypes.h"
#include "cglobals.h"
#include "mh.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MIN_ABORT_TIME_IN_MS      4
#define MAX_ABORT_TIME_IN_MS      32
#define MIN_INTER_ABORT_DUR_IN_MS 250
#define MAX_INTER_ABORT_DUR_IN_MS 500

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T         g_sw_trig_tx_abort_test;
extern ALARM_HANDLE_T *g_tx_abort_timer ;
extern ALARM_HANDLE_T *g_tx_resume_timer;
extern BOOL_T         g_tx_abort_in_progress;
extern BOOL_T         g_tx_abort_test_on ;
extern UWORD32 g_numtxabrt;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void start_tx_abort_timer(UWORD32 timeinms);
#ifndef OS_LINUX_CSL_TYPE
extern void txabort_alarm_fn(ALARM_HANDLE_T* alarm, UWORD32 data);
#else /* OS_LINUX_CSL_TYPE */
extern void txabort_alarm_fn(UWORD32 data);
extern void txabort_alarm_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

#ifndef OS_LINUX_CSL_TYPE
extern void txresume_alarm_fn(ALARM_HANDLE_T* alarm, UWORD32 data);
#else /* OS_LINUX_CSL_TYPE */
extern void txresume_alarm_fn(UWORD32 data);
extern void txresume_alarm_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

extern void configure_txab_test(void);
extern void create_tx_abort_alarms(void);
extern void delete_tx_abort_test_alarms(void);
#endif /* MH_TEST_TXABORT_H */

#endif /* TX_ABORT_FEATURE */
#endif /* TEST_TX_ABORT_FEATURE */
