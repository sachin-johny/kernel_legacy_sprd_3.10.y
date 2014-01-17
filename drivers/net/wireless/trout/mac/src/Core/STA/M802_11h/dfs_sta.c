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
/*  File Name         : dfs_sta.c                                            */
/*                                                                           */
/*  Description       : All BSS STA dynamic frequency selection related      */
/*                      routines are placed in this file.                    */
/*                                                                           */
/*  List of Functions : update_quiet_element                                 */
/*                      on_quiet_offset_expiry                               */
/*                      on_quiet_dur_expiry                                  */
/*                      on_reminding_to_msr_ch                               */
/*                      start_measurement                                    */
/*                      on_measurement_complete                              */
/*                      identify_measure_req                                 */
/*                      store_basic_req_elem_in_buffer                       */
/*                      add_elem_to_pending_q                                */
/*                      insert_elem_at_right_position                        */
/*                      process_meas_req_elements                            */
/*                      process_pending_requests                             */
/*                      set_sup_channel_list_element                         */
/*                      send_measurement_report                              */
/*                      send_autonomous_report                               */
/*                      suspend_tx_dueto_radar.                              */
/*                      on_csa_wait_expiry.                                  */
/*                      reset_ofdmpre_unidentified_bits_phy                  */
/*                      read_ofdmpre_unidentified_bits_phy                   */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11H

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "mac_init.h"
#include "index_util.h"
#include "receive.h"
#include "metrics.h"
#include "iconfig.h"
#include "cglobals_sta.h"
#include "frame.h"
#include "mh.h"
#include "prot_if.h"
#include "dfs_sta.h"
#include "tpc_sta.h"
#include "itypes.h"
#include "frame.h"
#include "transmit.h"
#include "qmu_if.h"
#include "rf_if.h"
#include "imem_if.h"
#include "core_mode_if.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_num_chnls_to_be_scanned = 0;
UWORD8 g_report_byte             = 0;
UWORD8 elem_incap[30]            = {0};

BOOL_T g_silence_mode                     = BFALSE;
BOOL_T g_radar_detected                   = BFALSE;
BOOL_T g_autonomous_basic_report          = BTRUE;
BOOL_T g_channel_shift                    = BFALSE;
BOOL_T g_waiting_for_csa                  = BFALSE;
BOOL_T g_measurement_in_progress          = BFALSE;
BOOL_T g_ch_msr_reminder_running          = BFALSE;
BOOL_T g_sending_aut_report_in_progress   = BFALSE;

ALARM_HANDLE_T *g_dfs_alarm_quiet_offset  = NULL;
ALARM_HANDLE_T *g_dfs_alarm_quiet_dur     = NULL;
ALARM_HANDLE_T *g_dfs_alarm_remind_ch_msr = NULL;
ALARM_HANDLE_T *g_dfs_alarm_ch_msr        = NULL;
ALARM_HANDLE_T *g_dfs_alarm_wait_for_csa  = NULL;
req_meas_elem *g_head_of_req_q            = NULL;

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static UWORD8  quiet_cnt      = 0;
static UWORD16 quiet_duration = 0;
static UWORD16 quiet_offset   = 0;
static UWORD8 num_incapable_report_bytes  = 0;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

BOOL_T insert_elem_at_right_position(req_meas_elem *req_elem);
BOOL_T add_elem_to_pending_q(req_meas_elem *req_elem);
void send_measurement_report(UWORD8 report_status,UWORD8 dia_token);
void process_pending_requests(void);
void process_meas_req_elements(UWORD8 dia_token);
void store_basic_req_elem_in_buffer(UWORD8* msa,UWORD16 index);
void start_measurement(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_quiet_element                                     */
/*                                                                           */
/*  Description   : this function is used to update quiet element.           */
/*                                                                           */
/*  Inputs        : 1) Beacon Frame                                          */
/*                  2) Length of the frame                                   */
/*                  3) Index from where the search should start              */
/*  Globals       : NONE                                                     */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Processing    : This function is used to update quiet element from       */
/*                  beacons.according to this quiet count and quiet          */
/*                  duration STA will creat alarms to suspend Tx.            */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_quiet_element(UWORD8 *msa,UWORD16 rx_len,UWORD16 tag_param_offset)
{
    UWORD16 index     = 0;
    UWORD8 *quiet_elm = 0;

    /*************************************************************************/
    /*                       Beacon Frame - Frame Body                       */
    /* --------------------------------------------------------------------- */
    /* |Timestamp |BeaconInt |CapInfo |SSID |SupRates |DSParSet |TIM elm   | */
    /* --------------------------------------------------------------------- */
    /* |8         |2         |2       |2-34 |3-10     |3        |4-256     | */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* |Country |FH Parameters |FH Pattern table |.... |.... |....|Quiet   | */
    /* --------------------------------------------------------------------- */
    /* |        |              |                 |     |     |    | 8      | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index     = tag_param_offset;
    quiet_elm = 0;

    /* Search for the TIM Element Field and return if the element is found */
    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IQUIET)
        {
            quiet_elm = &msa[index];
            break;
        }
        else
        {
            index += (IE_HDR_LEN + msa[index + 1]);
        }
    }

    if(quiet_elm !=0)
    {
        if(!quiet_cnt)
        {
            /* Update quiet count from beacon quiet element */
            quiet_cnt      = get_quiet_count(quiet_elm); /* in TBTT */
        }

        /* Update quiet duration from beacon quiet element */
        quiet_duration = get_quiet_duration(quiet_elm); /* in TU */

        /* Update quiet off-set from beacon quiet element */
        quiet_offset   = get_quiet_offset(quiet_elm);  /* in TU */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_tbtt_dfs_sta                                      */
/*                                                                           */
/*  Description   : This function is called whenever TBTT interrupt is       */
/*                  occured.                                                 */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : g_waiting_to_shift_channel                               */
/*                  g_waiting_for_csa                                        */
/*                  g_current_channel                                        */
/*                                                                           */
/*  Processing    : This function updates channel switch count and           */
/*                  quiet count whenever TBTT interrupt is occured.          */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_tbtt_dfs_sta(void)
{
    UWORD16 silence_start_time = 0;
    UWORD32 curr_time_hi       = 0;
    UWORD32 curr_time_lo       = 0;

	TROUT_FUNC_ENTER;
    if((g_measurement_in_progress == BTRUE)&&(g_channel_shift == BTRUE))
    {
        /* avoid link loss during channel measurement in the channels */
        /* which are requested by AP */
        g_link_loss_count = 0;
    }


    /* Decrement quiet count when ever TBTT interrupt is occured */
    /* when quiet count is non zero */
    if(quiet_cnt > 0)
    {
        quiet_cnt--;

        if(quiet_cnt == 0)
        {

#ifdef TX_ABORT_FEATURE
                silence_start_time = (quiet_offset*1024/1000);
                start_alarm(g_dfs_alarm_quiet_offset,silence_start_time);
#else /* TX_ABORT_FEATURE */
                silence_start_time = (quiet_offset*1024/1000 - OFFSET_IN_MS);
                start_alarm(g_dfs_alarm_quiet_offset,silence_start_time);
#endif /* TX_ABORT_FEATURE */

        }
    }

    /* Check if any measurement request falls in this TBTT duration */
    /* and start the alarm for measurement start time               */
    if((g_num_chnls_to_be_scanned != 0)        &&
       (g_measurement_in_progress == BFALSE)   &&
       (g_ch_msr_reminder_running == BFALSE))
    {
        UWORD32 time_diff_in_us;

        get_machw_tsf_timer(&curr_time_hi, &curr_time_lo);

        time_diff_in_us = time_diff(g_head_of_req_q->start_time_hi,
                                    g_head_of_req_q->start_time_lo,
                                    curr_time_hi,curr_time_lo);

        /* Check if start time is greater than TBTT duaration */
        if(time_diff_in_us > 1024*mget_BeaconPeriod())
        {
			TROUT_FUNC_EXIT;
            return;
        }
        else
        {
            start_alarm(g_dfs_alarm_remind_ch_msr,(time_diff_in_us/1000));
            g_ch_msr_reminder_running = BTRUE;
            g_num_chnls_to_be_scanned--;
        }

    }
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_quiet_offset_expiry                                   */
/*                                                                           */
/*  Description   : Activity timeout alarm handler.                          */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Suspend mac hardware Tx when ever quiet period starts    */
/*                  and start another alarm for quiet duration and resume    */
/*                  mac h/w  Tx.                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void on_quiet_offset_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_quiet_offset_expiry_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
	 if(reset_mac_trylock() == 0) return; 
    ALARM_WORK_ENTRY(work);
     /* This global variable indicates STA is in quiet duration */
    g_silence_mode = BTRUE;

    /* Suspend Tx and start alarm for quiet duration */
#ifdef TX_ABORT_FEATURE
    enable_machw_tx_abort();
    start_alarm(g_dfs_alarm_quiet_dur, (quiet_duration * 1024 /1000));
#else /* TX_ABORT_FEATURE */
    set_machw_tx_suspend();
    start_alarm(g_dfs_alarm_quiet_dur,
                (quiet_duration * 1024 /1000) + OFFSET_IN_MS);
#endif /* TX_ABORT_FEATURE */

    disable_machw_ack_trans();
    disable_machw_cts_trans();
    reset_ofdmpre_unidentified_bits_phy();
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE
void on_quiet_offset_expiry(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_quiet_dur_expiry                                      */
/*                                                                           */
/*  Description   : Activity timeout alarm handler.                          */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Resumes MAC H/W Tx after expiry of this alarm.           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/


#ifndef OS_LINUX_CSL_TYPE
void on_quiet_dur_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_quiet_dur_expiry_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* These set of statements will be executed when quiet duration */
    /* is over in this case resume Tx if measurement duration is    */
    /* not running */
    if(reset_mac_trylock() == 0) {
		return;
    }
    ALARM_WORK_ENTRY(work);
    if(g_measurement_in_progress == BFALSE)
    {
#ifdef TX_ABORT_FEATURE
    disable_machw_tx_abort();
#else /* TX_ABORT_FEATURE */
    set_machw_tx_resume();
#endif /* TX_ABORT_FEATURE */
        enable_machw_ack_trans();
        enable_machw_cts_trans();
    }

    g_silence_mode = BFALSE;
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE
void on_quiet_dur_expiry(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_reminding_to_msr_ch                                   */
/*                                                                           */
/*  Description   : Activity timeout alarm handler.                          */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function is called at measurement start time        */
/*                  compare present TSF time with actual  sheduled time      */
/*                  if there is any difference run another alarm otherwise   */
/*                  call start_measurement function.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void on_reminding_to_msr_ch(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_reminding_to_msr_ch_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* This function is called when channel measure time is started */
    /*  measure for radar, start an alarm for measurement duration   */
    UWORD32 lo   = 0;
    UWORD32 hi   = 0;
    UWORD32 time = 0;
    if(reset_mac_trylock() == 0){
		return;
    }
    ALARM_WORK_ENTRY(work);
    get_machw_tsf_timer(&hi, &lo);

    time =((g_head_of_req_q->start_time_lo - lo)/1000);

    start_measurement();
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE

void on_reminding_to_msr_ch(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_measurement                                        */
/*                                                                           */
/*  Description   : Measurement start time.                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_radar_detected                                         */
/*                  g_report_byte                                            */
/*                  g_current_channel                                        */
/*                  g_measurement_in_progress                                */
/*                  g_silence_mode                                           */
/*                  g_head_of_req_q                                          */
/*                                                                           */
/*  Processing    : This function is called at measurement start time        */
/*                  resets OFDM and unidentified bits in the register        */
/*                  activates alarm for measurement duration.                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_measurement(void)
{
    UWORD16 dur   = 0;
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = 0;

    dur = (g_head_of_req_q->dur)*(1024/1000);
    g_radar_detected = BFALSE;

    /* This global variable is used to avoid Autonomous report */
    g_measurement_in_progress = BTRUE;
    g_ch_msr_reminder_running = BFALSE;

    ch_idx = get_ch_idx_from_num(freq, g_head_of_req_q->meas_channel);

    /* Check if requested channel for measurement is not a current channel  */
    /* then shift the channel and measure for radar */
    if(g_current_channel != ch_idx)
    {
        g_channel_shift = BTRUE;
        disable_2040_operation();
        select_channel_rf(ch_idx, 0);
    }

#ifdef TX_ABORT_FEATURE
    enable_machw_tx_abort();
#else /* TX_ABORT_FEATURE */
    set_machw_tx_suspend();
#endif /* TX_ABORT_FEATURE */

    disable_machw_ack_trans();
    disable_machw_cts_trans();

    /* Reset ofdm and unidentified signal strength bits in PHY register */
    reset_ofdmpre_unidentified_bits_phy();

    /* Start alarm for channel measure duration */
    start_alarm(g_dfs_alarm_ch_msr,dur);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_measurement_complete                                  */
/*                                                                           */
/*  Description   : Activity timeout alarm handler.                          */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : g_radar_detected                                         */
/*                  g_report_byte                                            */
/*                  g_current_channel                                        */
/*                  g_measurement_in_progress                                */
/*                  g_silence_mode                                           */
/*                  g_head_of_req_q                                          */
/*                  g_local_mem_handle                                       */
/*                                                                           */
/*  Processing    : send measurement report for channel measurement request  */
/*                  with results of scanning and radar detection.            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void on_measurement_complete(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_measurement_complete_work(struct work_struct *data)
#endif /* OS_LINUX_CSL_TYPE */
{
    ALARM_WORK_ENTRY(work);
    req_meas_elem* dummy = NULL;

    /* This function is called when measurement duration is over */
    /* check if radar is detected, if so prepare report*/
    if(g_radar_detected == BTRUE)
    {
        g_report_byte = 0x08;
    }
    else
    {
        g_report_byte = 0x00;
    }

    if(BTRUE == is_unidentified_bit_detected())
    {
        /* Unidentified bit in report byte */
        g_report_byte |= 0x04;
    }

    if(BTRUE == is_ofdm_preamble_detected())
    {
        /* Check for OFDM preamble detected */
        g_report_byte |= 0x02;
    }


    /* Reset the current channel */
    if(g_channel_shift == BTRUE)
    {
       process_sec_chan_offset(g_current_sec_chan_offset, BFALSE);
       select_channel_rf(g_current_channel, get_bss_sec_chan_offset());
       g_channel_shift = BFALSE;
    }

    /* Reset this,measurement duration is over */
    g_measurement_in_progress = BFALSE;

    /* STA shouldn't  resume Tx if quiet duration is running */
    if(g_silence_mode == BFALSE)
    {
#ifdef TX_ABORT_FEATURE
        disable_machw_tx_abort();
#else /* TX_ABORT_FEATURE */
        set_machw_tx_resume();
#endif /* TX_ABORT_FEATURE */

       enable_machw_ack_trans();
       enable_machw_cts_trans();
    }

    /* Send measurement report*/
    send_measurement_report(STA_REPORT,0);

    /* Update head element with next basic request element  */
    dummy = g_head_of_req_q->next;
    mem_free(g_local_mem_handle, g_head_of_req_q);

    /* g_head_of_req_q always contains address of first basic request element */
    g_head_of_req_q = dummy;

    /* Call this function to process next basic request */
    process_pending_requests();
    ALARM_WORK_EXIT(work);
}

#ifdef OS_LINUX_CSL_TYPE
void on_measurement_complete(ADDRWORD_T data)
{
	alarm_fn_work_sched(data);
}

#endif


/*****************************************************************************/
/*                                                                           */
/*  Function Name : identify_measure_req                                     */
/*                                                                           */
/*  Description   : This function is called when ACTION frame is received.   */
/*                                                                           */
/*  Inputs        : 1) Action Frame                                          */
/*                  2) Length of the Frame                                   */
/*                  3) Index from where the search should start              */
/*                                                                           */
/*  Globals       : g_current_channel                                        */
/*                  g_waiting_to_shift_channel                               */
/*                  g_autonomous_basic_report                                */
/*                  g_waiting_for_csa                                        */
/*                                                                           */
/*  Processing    : This function processes received action frame,and        */
/*                  prepares report for the corresponding measurements       */
/*                  elements and takes care of channel switch element.       */
/*                  this function stores all the information elements        */
/*                  of measurement request to the memory.                    */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void identify_measure_req(UWORD8* msa,UWORD16 rx_len,UWORD16 mac_hdr_len,
                          UWORD32* rx_dscr)
{
    UWORD8 len       = 0;
    UWORD8 dia_token = 0;
    UWORD16 index    = 0;

	TROUT_FUNC_ENTER;
	
    index             = mac_hdr_len;
    dia_token         = msa[index+2];
    g_current_channel = mget_CurrentChannel();
    g_current_sec_chan_offset = get_bss_sec_chan_offset();

    num_incapable_report_bytes = 0;

   /* Check whether incoming frame is ACTION frame */
    if(msa[index] == SPECMGMT_CATEGORY)
    {
        /* 3 LSB bits are used to indentify action details remaining bits are  */
        /* reserved */
        switch(msa[index+1]&0x7)
        {
            case MEASUREMENT_REQUEST_TYPE:
            {
                TROUT_DBG4("Measurement Request Recvd, DialogToken: %d\n",dia_token);

               if(mget_enableDFS() == BFALSE)
               {
					TROUT_FUNC_EXIT;
                   return;
               }

                /* Update index value to identify measurement request element */
                index+=3;
                while(index < (rx_len - FCS_LEN))
                {
                     len = msa[index+1];

                    /* If length of the measurement request is three then */
                    /* that request element indicates enabled or disabled  */
                    /* requests and reports. see 7.3.2.21 */
                    if(len == 3)
                    {
                        if((msa[index+4] & 0x3) == BASICREQUEST)
                         {
                             if(msa[index+3] & 0x08)
                               g_autonomous_basic_report = BTRUE;
                             else
                               g_autonomous_basic_report = BFALSE;
                         }

                     }
                     else
                     {
                        /* Identify measurement request type */
                        if((msa[index+4] & 0x3) == BASICREQUEST)
                        {
                            /* Store all basic request elements for future*/
                            /* measurements */
                            store_basic_req_elem_in_buffer(msa, index);
                        }
                        else
                        {
                            /* Measurement request is CCA or RPI then prepare */
                            /* a report as STA is incapable of generating a report*/
                            /* Measurement Report ID */
                            elem_incap[num_incapable_report_bytes++] =
                                                          IMEASUREMENTREPORT;

                            /* Lenght of the Element */
                            elem_incap[num_incapable_report_bytes++] =
                                                    IMEASUREMENTREPORT_LEN_MIN;

                            /* Measurement Token */
                            elem_incap[num_incapable_report_bytes++] =
                                                                msa[index+2];

                            /* Measurement report mode, incapable*/
                            elem_incap[num_incapable_report_bytes++] =
                                                             INCAPABLE_REPORT;

                            /* Measurement Type */
                            elem_incap[num_incapable_report_bytes++] =
                                                                 msa[index+4];


                            TROUT_DBG2("DFS-Err: Not a Basic Request, DiaToken: %d MesToken: %d\n",dia_token, msa[index+2]);

                        }

                     }

                     index += msa[index + 1] + 2;
                } /* End while loop */

                 /* Call this function to send reports for all incapable */
                 /* measurement requests */
                 process_meas_req_elements(dia_token);

            } /* End case */
            break;
            case TPC_REQUEST_TYPE:
            {
                WORD8 rx_power_req = 0;

                TROUT_DBG4("TPC Request Received, DialogToken: %d\n",dia_token);

                if(mget_enableTPC() == BFALSE)
                {
					TROUT_FUNC_EXIT;
                    return;
                }

                /* Get received power of TPC request this value is */
                /* to calculate link margin. this link margin value */
                /* is transmitted in TPC report */
                rx_power_req = get_rx_dscr_rssi_db((UWORD32 *)rx_dscr);

                send_tpc_report(rx_power_req,dia_token);

            }
                    break;
            default:
            {
                /* do nothing */
				TROUT_FUNC_EXIT;
                return;
            }
        }
    }
    TROUT_FUNC_EXIT;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : store_basic_req_elem_in_buffer                           */
/*                                                                           */
/*  Description   : This function is called when Action frame contains       */
/*                  basic requests                                           */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1) Action Frame                                          */
/*                  2) Index from where the search should start              */
/*                  3) starting element of the array to copy the report      */
/*                                                                           */
/*  Globals       : None                                                     */
/* Processing     : This function stores all basic request elements          */
/*                  for future measurements, if channel in measurement       */
/*                  request doesn't supported by STA then prepare report     */
/*                  as incapable.                                            */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void store_basic_req_elem_in_buffer(UWORD8* msa,UWORD16 index)
{
    BOOL_T check            = 0;
    req_meas_elem* req_elem = 0;

    check = is_this_channel_supported_by_rf(msa[index+5]);

    if(check == BFALSE)
    {
        /* Measurement Report ID */
        elem_incap[num_incapable_report_bytes++] = IMEASUREMENTREPORT;

        /* Lenght of the Element */
        elem_incap[num_incapable_report_bytes++] = IMEASUREMENTREPORT_LEN_MIN;

        /* Measurement Token */
        elem_incap[num_incapable_report_bytes++] = msa[index+2];

        /* Measurement report mode, incapable */
        elem_incap[num_incapable_report_bytes++] = INCAPABLE_REPORT;

        /* Measurement Type */
        elem_incap[num_incapable_report_bytes++] = msa[index+4];

        PRINTD2("DFSErr: Requested Channel No.%d Not supported, MesToken: %d\n",msa[index+5],msa[index+2]);
        return;
    }

    req_elem = (req_meas_elem*)mem_alloc(g_local_mem_handle,
                                         sizeof(req_meas_elem));

    if(req_elem == NULL)
    {
        PRINTD2("DFS-Err: No memory for Measurement Report\n");
      /* Exception */
      return;
    }

    req_elem->dia_token    = msa[MAC_HDR_LEN+2];
    req_elem->token        = msa[index+2];

    /* Channel that STA has to measure */
    req_elem->meas_channel = msa[index+5];

    /* Measurement start time total 8bytes */
    req_elem->start_time_hi       = ((UWORD32)msa[index+13])<<24;
    req_elem->start_time_hi      |= ((UWORD32)msa[index+12])<<16;
    req_elem->start_time_hi      |= ((UWORD32)msa[index+11])<<8;
    req_elem->start_time_hi      |= (UWORD32)msa[index+10];

    req_elem->start_time_lo       = ((UWORD32)msa[index+9])<<24;
    req_elem->start_time_lo      |= ((UWORD32)msa[index+8])<<16;
    req_elem->start_time_lo      |= ((UWORD32)msa[index+7])<<8;
    req_elem->start_time_lo      |= (UWORD32)msa[index+6];

    req_elem->dur          =  (UWORD16)msa[index+15]<<8;
    req_elem->dur         |=  (UWORD16)msa[index+14];

    req_elem->next         = NULL;

    /* Call a function insert this element in queue with respective to */
    /* measurement start time */
    if(BFALSE == add_elem_to_pending_q(req_elem))
    {
        mem_free(g_local_mem_handle, req_elem);

        /* Measurement Report ID */
        elem_incap[num_incapable_report_bytes++]= IMEASUREMENTREPORT;

        /* Lenght of the Element */
        elem_incap[num_incapable_report_bytes++] = IMEASUREMENTREPORT_LEN_MIN;

        /* Measurement Token */
        elem_incap[num_incapable_report_bytes++] = msa[index+2];

        /* Measurement report mode, incapable */
        elem_incap[num_incapable_report_bytes++] = REFUSED_REPORT;

        /* Measurement Type */
        elem_incap[num_incapable_report_bytes++] = msa[index+4];

        PRINTD2("DFS-Err: Adding Element to Measurement Q Failed, DiaToken: %d MesToken: %d\n",req_elem->dia_token, msa[index+2]);

        return;
    }
    else
    {
        PRINTD("DFS-Info: Adding Element to Measurement Q Success, DiaToken: %d MesToken: %d\n",req_elem->dia_token, msa[index+2]);
    }

    g_num_chnls_to_be_scanned++;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_elem_to_pending_q                                    */
/*                                                                           */
/*  Description   : This function is called when Action frame contains       */
/*                  basic requests                                           */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1) New Request Element                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/* Processing     : This function is used to insert the element              */
/*                  according to measurement start time.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T add_elem_to_pending_q(req_meas_elem *req_elem)
{
    /* If it is first element store that element address in head element */
    if(g_head_of_req_q == NULL)
    {
        g_head_of_req_q = req_elem;
        return BTRUE;
    }
    /* Call a function to insert at the right place */
    else
    {
        return insert_elem_at_right_position(req_elem);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : insert_elem_at_right_position                            */
/*                                                                           */
/*  Description   : This function is called to arrange basic request         */
/*                  elements with respective to measurement start time.      */
/*                                                                           */
/*                                                                           */
/*  Inputs        : pointer to incoming basic request element.               */
/*                                                                           */
/*  Globals       : g_ch_msr_reminder_running                                */
/*                  g_dfs_alarm_remind_ch_msr                                */
/*                  g_head_of_req_q                                          */
/*                                                                           */
/*                                                                           */
/*  Processing    : None                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T insert_elem_at_right_position(req_meas_elem *new_req_elem)
{
    req_meas_elem *prev_elem = NULL;
    req_meas_elem *next_elem = NULL;
    req_meas_elem *curr_elem = NULL;

    curr_elem = g_head_of_req_q;

    while(curr_elem != NULL)
    {
        if(is_new_element_later(new_req_elem,curr_elem) == BTRUE)
        {
            prev_elem = curr_elem;
            next_elem = curr_elem->next;
        }
        else
        {
            break;
        }
        curr_elem = curr_elem->next;
    }

    if(prev_elem == NULL)
    {
        if(are_they_overlapping(new_req_elem,g_head_of_req_q) == BTRUE)
        {
            PRINTD2("DFSErr: Overlapping Measurement Req\n");
            return BFALSE;
        }

        if((new_req_elem->start_time_hi == 0) &&
          (new_req_elem->start_time_lo == 0))
        {
            if(g_measurement_in_progress == BTRUE)
            {
                PRINTD2("DFSErr: Can't start measurement immediately as another in progress\n");
                return BFALSE;
            }
        }

        /* If incoming element start time is less than all the elements     */
        /* then insert it at the front i.e store this element address in    */
        /* head_elem if head_elem is to  be processed by STA by running its */
        /* measurement start alarm then stop that alarm and insert          */
        /* this element at the front */
        if(g_ch_msr_reminder_running == BTRUE)
        {
            stop_alarm(g_dfs_alarm_remind_ch_msr);
        }

        new_req_elem->next = g_head_of_req_q;
        g_head_of_req_q = new_req_elem;
    }
    else
    {
        if(are_they_overlapping(prev_elem,new_req_elem) == BTRUE)
        {
            PRINTD2("DFSErr: Overlapping Measurement Req with Prev\n");
            return BFALSE;
        }

        if(are_they_overlapping(new_req_elem,next_elem) == BTRUE)
        {
            PRINTD2("DFSErr: Overlapping Measurement Req with Next\n");
            return BFALSE;
        }

        /* Insert the element to the queue */
        prev_elem->next = new_req_elem;
        new_req_elem->next = next_elem;
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_meas_req_elements                                */
/*                                                                           */
/*  Description   : This function is called to process measurement request   */
/*                  elements.                                                */
/*                                                                           */
/*                                                                           */
/*  Inputs        : dialog token of action frame.                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*                                                                           */
/*  Processing    : this function sends report with Incapable if STA gets    */
/*                  request of CCA, RPI and basic request measurements       */
/*                  in the channels which are not supported by STA.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_meas_req_elements(UWORD8 dia_token)
{
    if(num_incapable_report_bytes != 0)
    {
        send_measurement_report(INCAPABLE,dia_token);
    }

    /* Call a function to measure requested channels for radar */
    /* according to specified start times and durations in request */
    if(g_measurement_in_progress == BFALSE)
    {
        process_pending_requests();
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_pending_requests                                 */
/*                                                                           */
/*  Description   : This function is called to process measurement request   */
/*                  elements                                                 */
/*                                                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_num_chnls_to_be_scanned                                */
/*                  g_head_of_req_q                                          */
/*                  g_ch_msr_reminder_running                                */
/*                                                                           */
/*  Processing    : This function scans the channels which are mentioned in  */
/*                  basic request elements and starts the alarm for          */
/*                  measurement start time and sends report with late bit    */
/*                  if it receives after the measurement start time.         */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None.                                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_pending_requests(void)
{
    UWORD32 curr_time_hi   = 0;
    UWORD32 curr_time_lo   = 0;
    UWORD32 time           = 0;
    req_meas_elem* dummy   = 0;

    while(g_num_chnls_to_be_scanned)
    {
        get_machw_tsf_timer(&curr_time_hi, &curr_time_lo);

        /* Check if measurement start time is zero then measure a channel*/
        /* for radar */
        if((g_head_of_req_q->start_time_hi == 0) &&
          (g_head_of_req_q->start_time_lo == 0))
        {
            start_measurement();
            g_num_chnls_to_be_scanned--;
            break;
        }
        else
        {
            /* Compare measurement start time with present time */
            if(curr_time_hi > g_head_of_req_q->start_time_hi)
            {
                /* Send report as Late if If measurement start time is */
                /* less than STA mac tsf time */
                send_measurement_report(REC_LATE,0);

                /* If measurement start is less than present time then */
                /* update head element with next basic request element  */
                g_num_chnls_to_be_scanned--;

                dummy = g_head_of_req_q->next;
                mem_free(g_local_mem_handle, g_head_of_req_q);
                g_head_of_req_q = dummy;
            }
            else if(curr_time_hi == g_head_of_req_q->start_time_hi)
            {
                if(curr_time_lo > g_head_of_req_q->start_time_lo)
                {
                    req_meas_elem* dummy;
                    /* Send report as Late if If measurement start time is */
                    /* less than STA mac tsf time */
                    send_measurement_report(REC_LATE,0);

                    g_num_chnls_to_be_scanned--;

                    /* Update head element with next basic request element  */
                    dummy = g_head_of_req_q->next;
                    mem_free(g_local_mem_handle, g_head_of_req_q);
                    g_head_of_req_q = dummy;
                }
                else
                {
                    time =((g_head_of_req_q->start_time_lo -
                                        curr_time_lo)/1000);

                    if(time > mget_BeaconPeriod())
                    {
                        return;
                    }

                    start_alarm(g_dfs_alarm_remind_ch_msr,time);
                    g_ch_msr_reminder_running = BTRUE;
                    g_num_chnls_to_be_scanned--;
                    break;
                }
            }
            else
            {
                return;
            }
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_sup_channel_list_element                             */
/*                                                                           */
/*  Description   : This function sets the Supported channel list element    */
/*                  (elementID, length, Rates) in the given frame.           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the Supported Rates element.                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The Supported channel list are obtained from the MIB     */
/*                  and the fields of the Supported channel list element and */
/*                  the   given data frame, at the given index.              */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the Supported channel list element.               */
/*                                                                           */
/*  Returns       : Supported channel list element length.                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 set_sup_channel_list_element(UWORD8* data, UWORD16 index)
{
    UWORD8  count     = 0;
    UWORD16 len       = 0;
    UWORD16 index_len = 0;
    UWORD8  freq      = get_current_start_freq();
    UWORD8  max_ch    = get_max_num_channel(freq);

    if(freq == RC_START_FREQ_2)
        return 0;

    /*************************************************************************/
    /*                    Supported Channel list IE Format                   */
    /* --------------------------------------------------------------------- */
    /* |Supchannel Ele ID | length  | first cha| no.of ch | second| s.no.ofch*/
    /* --------------------------------------------------------------------- */
    /* | 1                | 1       |    1     | 1        |  1    |      1   */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/
    /* Length of the element is variable depends up on supported channels    */
    /* see section 7.3.2.19                                                  */
    /* Set the Supported Channel list element ID to the given frame at given */
    /* index                                                                 */
    data[index] = ISUPCHANNEL;
    index_len   = ++index;
    index++;

    while(count < max_ch )
    {
		// 20120830 caisf mod, merged ittiam mac v1.3 code
        //if(is_ch_idx_supported(freq, count) == BTRUE)
        if(is_ch_idx_supported_by_rf(freq, count) == BTRUE)
        {
            data[index++] = get_ch_num_from_idx(freq, count);
            data[index++] = 1;
            len +=2;
        }
        count++;
    }

    /* Set length of the element */
    data[index_len] = len;

    return (IE_HDR_LEN + len);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_measurement_report                                  */
/*                                                                           */
/*  Description   : Prepare and send measurement report frame to AP.         */
/*                                                                           */
/*  Inputs        : 1) dia_token                                             */
/*                  2) type of report                                        */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the measurement report frame      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_measurement_report(UWORD8 report_status,UWORD8 dia_token)
{
    UWORD8      msr_tkn      = 0;
    UWORD8      q_num        = 0;
    UWORD8      mac_hdr_len  = 0;
    UWORD16     index        = 0;
    UWORD8      *msa         = 0;
    sta_entry_t *se          = 0;

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        PRINTD2("DFS-Err:No Memory for sending Measurement report dia_token: %d\n",dia_token);
        return;
    }

    /* Set Action Req frame type */
    set_frame_control(msa, (UWORD16)ACTION);
    q_num = HIGH_PRI_Q;
    mac_hdr_len = MAC_HDR_LEN;
    set_to_ds(msa, 1);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());
    index = MAC_HDR_LEN;
    index = set_category(msa, SPECMGMT_CATEGORY,index);
    index = set_action(msa, MEASUREMENT_REPORT_TYPE,index);
    index = set_dia_token(msa, dia_token,index);
    index =  set_info_elements(msa,index,report_status,
                                num_incapable_report_bytes, &msr_tkn);

    se = (sta_entry_t *)find_entry(mget_bssid());

    if(BFALSE == tx_mgmt_frame(msa, index + FCS_LEN, q_num, 0))
    {
        PRINTD2("DFSErr: DFS Report with DiaTokenNo.%d Could not be sent\n",dia_token);
    }
    else
    {
        PRINTD("DFS Report of status:%d with DiaToken No.%d MsrTknNo.%d sent\n",report_status, msa[MAC_HDR_LEN+2], msr_tkn);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_autonomous_report                                   */
/*                                                                           */
/*  Description   : Prepare and send autonomous report frame to AP.          */
/*                                                                           */
/*  Inputs        : 1) None.                                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the autonomous report frame and   */
/*                  send to the AP with radar detection bit is set           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_autonomous_report(void)
{
    UWORD8      q_num        = 0;
    UWORD8      mac_hdr_len  = 0;
    UWORD16     index        = 0;
    UWORD8      *msa         = 0;

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        PRINTD2("DFSErr:No Mem for sending Autonomous report\n");
        return;
    }

    /* Set the Frame Control field of the NULL frame.                        */
    set_frame_control(msa, (UWORD16)ACTION);
    q_num = HIGH_PRI_Q;
    mac_hdr_len = MAC_HDR_LEN;

    set_to_ds(msa, 1);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());

    index = MAC_HDR_LEN;
    index = set_category(msa, SPECMGMT_CATEGORY,index);
    index = set_action(msa, MEASUREMENT_REPORT_TYPE,index);
    index = set_dia_token(msa, AUTONOMOUS,index);
    index = set_elem_id(msa,IMEASUREMENTREPORT,index);
    index = set_length(msa,15,index);
    index = set_meas_token(msa,AUTONOMOUS,index);
    index = set_meas_mode(msa,AUTONOMOUS,index);
    index = set_meas_type(msa,AUTONOMOUS,index);
    index = set_ch_number(msa, get_ch_num_from_idx(get_current_start_freq(),
                          mget_CurrentChannel()), index);
    index = set_measurement_time_dur(msa,index);
    set_basic_report(msa,index);

    if(BTRUE == g_radar_detected)
    {
        /* Check for OFDM preamble detected */
        msa[index] |= 0x08;
    }

    if(BTRUE == is_unidentified_bit_detected())
    {
         /* Unidentified bit in report byte */
         msa[index] |= 0x04;
    }

    if(BTRUE == is_ofdm_preamble_detected())
    {
        /* Check for OFDM preamble detected */
        msa[index] |= 0x02;
    }
    index +=1;


    if(BFALSE == tx_mgmt_frame(msa, index + FCS_LEN, q_num, 0))
    {
        PRINTD2("DFSErr: Autonomous Report Could not be sent\n");
    }
    else
    {
        PRINTD("Autonomous Report sent\n");
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : suspend_tx_dueto_radar.                                  */
/*                                                                           */
/*  Description   : suspend Tx when STA detects Radar.                       */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*                                                                           */
/*  Globals       : g_waiting_for_csa                                        */
/*                  g_dfs_alarm_wait_for_csa                                 */
/*                                                                           */
/*                                                                           */
/*  Processing    : According to standard if STA detects radar then STA      */
/*                  should suspend Tx and start alarm of 10sec               */
/*                  with in this time if channel switch is occured           */
/*                  STA will resume Tx other wise STA will resume Tx after   */
/*                  10sec.                                                   */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void suspend_tx_dueto_radar(void)
{
    if(g_waiting_for_csa == BFALSE)
    {
        set_machw_tx_suspend();
        disable_machw_ack_trans();
        disable_machw_cts_trans();
        start_alarm(g_dfs_alarm_wait_for_csa,CHANNEL_MOVE_TIME_OUT);
        g_waiting_for_csa = BTRUE;
    }
    else
    {
        /* Do nothing */
        /* already STA has sent one autonomous report,one alarm is running */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : on_csa_wait_expiry.                                      */
/*                                                                           */
/*  Description   : Activity timeout alarm handler.                          */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : g_waiting_for_csa                                        */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Processing    : Check after 10sec time STA may get Channel switch        */
/*                  element and might have changed its operation channel     */
/*                  if STA doesn't get Channel Switch element then resume Tx.*/
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void on_csa_wait_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void on_csa_wait_expiry_work(struct work_struct * work)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* Check after 10sec time STA may get Channel switch element and might  */
    /* have changed its operation channel in that case do nothing           */
    if(reset_mac_trylock() ==0 ) {
		return;
    }
	ALARM_WORK_ENTRY(work);
    if(g_waiting_for_csa == BTRUE)
    {
        set_machw_tx_resume();
        enable_machw_ack_trans();
        enable_machw_cts_trans();
        g_waiting_for_csa = BFALSE;
    }
    ALARM_WORK_EXIT(work);
     reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE

void on_csa_wait_expiry(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

#endif /* IBSS_BSS_STATION_MODE */

#endif /* MAC_802_11H */
