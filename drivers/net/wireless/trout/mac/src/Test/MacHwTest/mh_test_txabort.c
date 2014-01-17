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
/*  File Name         : mh_test_txabort.c                                    */
/*                                                                           */
/*  Description       : This file contains the test code for Tx Abort        */
/*                      feature.                                             */
/*                                                                           */
/*  List of Functions : start_tx_abort_timer                                 */
/*                      start_tx_resume_timer                                */
/*                      txabort_alarm_fn                                     */
/*                      txresume_alarm_fn                                    */
/*                      create_tx_abort_alarms                               */
/*                      delete_tx_abort_test_alarms                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef TEST_TX_ABORT_FEATURE
#ifdef TX_ABORT_FEATURE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mh_test_txabort.h"
#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test_config.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T          g_sw_trig_tx_abort_test = BTRUE;
UWORD32         g_numtxabrt             = 0;
ALARM_HANDLE_T *g_tx_abort_timer        = NULL;
ALARM_HANDLE_T *g_tx_resume_timer       = NULL;
BOOL_T          g_tx_abort_in_progress  = BFALSE;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void start_tx_resume_timer(UWORD32);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_txab_test                                      */
/*                                                                           */
/*  Description   : This function configures and initializes the parameters  */
/*                  for the S/w triggered Tx Abort test.                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_tx_abort_timer                                         */
/*                  g_tx_resume_timer                                        */
/*                  g_tx_abort_in_progress                                   */
/*                                                                           */
/*  Processing    : This function enables S/w triggered Tx Abort and Self    */
/*                  CTS transmission on Tx Abort. It initializes the Tx      */
/*                  Abort and Tx Resume timers. It also resets the global    */
/*                  g_tx_abort_in_progress. It then starts the Tx Abort      */
/*                  timer with the minimum inter abort duration to start the */
/*                  test.                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_txab_test(void)
{
    enable_machw_txabort_sw_trig();

    create_tx_abort_alarms();

    g_tx_abort_in_progress = BFALSE;
#ifdef MAC_HW_UNIT_TEST_MODE
	if(g_test_config.sw_trig_tx_abort_test == 1)
		g_sw_trig_tx_abort_test = 1;
	else
		g_sw_trig_tx_abort_test = 0;
#endif /* MAC_HW_UNIT_TEST_MODE */

    start_tx_abort_timer(MIN_INTER_ABORT_DUR_IN_MS);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_tx_abort_timer                                     */
/*                                                                           */
/*  Description   : This function starts the Tx Abort timer.                 */
/*                                                                           */
/*  Inputs        : 1) Time interval                                         */
/*                                                                           */
/*  Globals       : g_tx_abort_timer                                         */
/*                                                                           */
/*  Processing    : This function stops and starts the Tx Abort timer with   */
/*                  the given time interval.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_tx_abort_timer(UWORD32 timeinms)
{
    if(g_sw_trig_tx_abort_test == 0)
    {
        PRINTD("S/w triggered Tx Abort test is disabled\n");
        return;
    }

    stop_alarm(g_tx_abort_timer);
    start_alarm(g_tx_abort_timer, timeinms);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_tx_resume_timer                                    */
/*                                                                           */
/*  Description   : This function starts the Tx Resume timer.                */
/*                                                                           */
/*  Inputs        : 1) Time interval                                         */
/*                                                                           */
/*  Globals       : g_tx_resume_timer                                        */
/*                                                                           */
/*  Processing    : This function stops and starts the Tx Resume timer with  */
/*                  the given time interval.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_tx_resume_timer(UWORD32 timeinms)
{
    stop_alarm(g_tx_resume_timer);
    start_alarm(g_tx_resume_timer, timeinms);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : txabort_alarm_fn                                         */
/*                                                                           */
/*  Description   : This is the completion function called when the Tx Abort */
/*                  timer expires.                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_tx_abort_in_progress                                   */
/*                                                                           */
/*  Processing    : On expiry of the Tx Abort timer the following actions    */
/*                  are taken if the global indicates that Tx Abort is not   */
/*                  in progress currently (g_tx_abort_in_progress). A random */
/*                  time is selected from the constant range for which MAC   */
/*                  H/w will remain in Tx Abort state and thereafter resume  */
/*                  normal operation, next_resume_time. It then enables MAC  */
/*                  H/w Tx Abort, sets the Self-CTS duration for the         */
/*                  next_resume_time and starts the Tx Resume timer for the  */
/*                  same time (next_resume_time). It also sets the global    */
/*                  g_tx_abort_in_progress to BTRUE.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void txabort_alarm_fn(ALARM_HANDLE_T* alarm, UWORD32 data)
#else /* OS_LINUX_CSL_TYPE */
void txabort_alarm_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    ALARM_WORK_ENTRY(work);
    if(g_sw_trig_tx_abort_test == 0)
    {
        PRINTD("S/w triggered Tx Abort test is disabled\n");
        ALARM_WORK_EXIT(work);
        return;
    }

    if(g_tx_abort_in_progress == BFALSE)
    {
        UWORD32 next_resume_time = 0;

        next_resume_time = MIN_ABORT_TIME_IN_MS + (((MAX_ABORT_TIME_IN_MS -
                           MIN_ABORT_TIME_IN_MS) * get_random_number()) >> 8);

        set_machw_tx_abort_dur(next_resume_time * 1000);

        enable_machw_tx_abort();
        PRINTD("TxAb");
        g_tx_abort_in_progress = BTRUE;

        g_numtxabrt++;

        start_tx_resume_timer(next_resume_time);
    }
    else
    {
        PRINTD("Tx Abort Alarm Fn: Tx abort is in progress\n");
    }
    ALARM_WORK_EXIT(work);
}

//add by Hugh
#ifdef OS_LINUX_CSL_TYPE

void txabort_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : txresume_alarm_fn                                        */
/*                                                                           */
/*  Description   : This is the completion function called when the Tx       */
/*                  Resume timer expires.                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_tx_abort_in_progress                                   */
/*                                                                           */
/*  Processing    : On expiry of the Tx Resume timer the following actions   */
/*                  are taken if the global indicates that Tx Abort is in    */
/*                  progress currently (g_tx_abort_in_progress). It disables */
/*                  MAC H/w Tx Abort and resets the global. A random time is */
/*                  selected from the constant range after which MAC H/w     */
/*                  will go into Tx Abort state, next_abort_time. It then    */
/*                  starts the Tx Abort timer for this (next_abort_time).    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void txresume_alarm_fn(ALARM_HANDLE_T* alarm, UWORD32 data)
#else /* OS_LINUX_CSL_TYPE */
void txresume_alarm_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    ALARM_WORK_ENTRY(work);
    if(g_tx_abort_in_progress == BTRUE)
    {
        UWORD32 next_abort_time = 0;


        disable_machw_tx_abort();
        g_tx_abort_in_progress = BFALSE;

        next_abort_time = MIN_INTER_ABORT_DUR_IN_MS + (((MAX_INTER_ABORT_DUR_IN_MS -
                          MIN_INTER_ABORT_DUR_IN_MS) * get_random_number()) >> 8);

        start_tx_abort_timer(next_abort_time);
    }
    else
    {
        PRINTD("Tx Resume Alarm Fn: Tx abort is not in progress\n");
    }
    ALARM_WORK_EXIT(work);
}

//add by Hugh
#ifdef OS_LINUX_CSL_TYPE

void txresume_alarm_fn(ADDRWORD_T data)
{
     alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_tx_abort_alarms                                   */
/*                                                                           */
/*  Description   : This function is used to create Tx abort alarms.         */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : g_tx_abort_timer                                         */
/*                  g_tx_abort_in_progress                                   */
/*                                                                           */
/*  Processing    : This function creates Tx abort alarms.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void create_tx_abort_alarms(void)
{
    if(g_tx_abort_timer == NULL)
        g_tx_abort_timer = create_alarm(txabort_alarm_fn, 0, txabort_alarm_work);
    
    if(g_tx_resume_timer == NULL)
        g_tx_resume_timer = create_alarm(txresume_alarm_fn, 0, txresume_alarm_work);

    g_tx_abort_in_progress = BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_tx_abort_test_alarms                              */
/*                                                                           */
/*  Description   : This function is used to delete Tx abort alarms.         */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : g_tx_abort_timer                                         */
/*                  g_tx_resume_timer                                        */
/*                                                                           */
/*  Processing    : This function deletes Tx abort alarms.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_tx_abort_test_alarms(void)
{
    if(g_tx_abort_timer != NULL)    //modified by Hugh
    {
        flush_work((struct work_struct *)((struct timer_list *)g_tx_abort_timer + 1));
        delete_alarm(&g_tx_abort_timer);
    }
    
    if(g_tx_resume_timer != NULL)    //modified by Hugh
    {
        flush_work((struct work_struct *)((struct timer_list *)g_tx_resume_timer + 1));
        delete_alarm(&g_tx_resume_timer);
    }
    
    g_tx_abort_timer = NULL;
    g_tx_resume_timer = NULL;

    PRINTD("No.of S/w Trigger Tx Aborts = %d\n",g_numtxabrt);

    /* Reset the counter */
    g_numtxabrt  = 0;
}

#endif /* TX_ABORT_FEATURE */

#endif /* TEST_TX_ABORT_FEATURE */
