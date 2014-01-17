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
/*  File Name         : dfs_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains all the functions declarations    */
/*                      that are used in dfs_sta.c                           */
/*                      Station mode of operation.                           */
/*                                                                           */
/*  List of Functions :  set_info_elements                                   */
/*                       get_action_field                                    */
/*                       get_dialog_token_field                              */
/*                       get_meas_token_field                                */
/*                       create_dfs_alarms                                   */
/*                       delete_dfs_alarms                                   */
/*                       get_quiet_count                                     */
/*                       get_quiet_duration                                  */
/*                       get_quiet_offset                                    */
/*                       set_category                                        */
/*                       set_action                                          */
/*                       set_dia_token                                       */
/*                       set_elem_id                                         */
/*                       set_length                                          */
/*                       set_meas_token                                      */
/*                       set_meas_mode                                       */
/*                       set_meas_type                                       */
/*                       set_ch_number                                       */
/*                       set_meas_time_dur                                   */
/*                       set_basic_report                                    */
/*                       init_dfs_sta_globals                                */
/*                       time_diff                                           */
/*                       is_time_greater                                     */
/*                       is_new_element_later                                */
/*                       are_they_overlapping                                */
/*                       can_dfs_sta_doze_now                                */
/*                       can_dfs_sta_transmit_now                            */
/*                       update_11h_prot_params                              */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11H

#ifdef IBSS_BSS_STATION_MODE

#ifndef DFS_STA_H
#define DFS_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "frame.h"
#include "mh.h"
#include "csl_if.h"
#include "common.h"
#include "itypes.h"
#include "cglobals.h"
#include "mib_11h.h"
#include "channel_sw.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* Action field in action frame  */
#define BASICREQUEST        0
#define BASICREPORT         0

/* Length of Action Fields */
#define BASICREPORT_LEN     12

/* Incase if TxSuspend is used for stopping transmission, it may take time incase */
/* if 1Mbps frame is progress. So do the suspension 'offset' time before the actual */
/* suspension start time and also increase the suspension duration by that time  */
#define OFFSET_IN_MS        2
#define AUTONOMOUS          0

#define MEASUREMENT_DUR    0x0F /* Measurement duration(in TU) is 16*1.024msec  */
#define CHANNEL_MOVE_TIME_OUT 10000   /* Channel move time is 10sec */

#define INCAPABLE            0
#define REC_LATE             1
#define STA_REPORT           2
#define REFUSED_REQ          3

#define REC_LATE_REPORT      0x01
#define INCAPABLE_REPORT     0x02
#define REFUSED_REPORT       0x04

#define SCAN_DFS_TIME        60000  /* 60sec */
#define LINK_LOSS_OFFSET_11H 5 /* 5 Beacons */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum{MISC_RADAR_DETECTED = 0x60, /* Radar detected event */
} EVENT_TYPESUBTYPE_11H_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Structure to store Measurement request elements */
typedef struct _req_meas_elem
{
    UWORD8 dia_token;
    UWORD8 token;
    UWORD8 meas_channel;
    UWORD16 dur;
    UWORD32 start_time_hi;
    UWORD32 start_time_lo;
    struct _req_meas_elem *next;
}req_meas_elem;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern ALARM_HANDLE_T *g_dfs_alarm_quiet_offset;
extern ALARM_HANDLE_T *g_dfs_alarm_quiet_dur;
extern ALARM_HANDLE_T *g_dfs_alarm_remind_ch_msr;
extern ALARM_HANDLE_T *g_dfs_alarm_ch_msr;
extern ALARM_HANDLE_T *g_dfs_alarm_wait_for_csa;
extern BOOL_T g_silence_mode ;
extern BOOL_T g_radar_detected ;
extern BOOL_T g_autonomous_basic_report;
extern BOOL_T g_waiting_for_csa;
extern BOOL_T g_measurement_in_progress;
extern UWORD8 g_num_chnls_to_be_scanned ;
extern UWORD8 elem_incap[30];
extern req_meas_elem *g_head_of_req_q;
extern UWORD8 g_report_byte;
extern UWORD8 g_current_channel;
extern UWORD8 g_current_sec_chan_offset;
extern BOOL_T g_ch_msr_reminder_running;
extern BOOL_T g_channel_shift;
extern BOOL_T g_sending_aut_report_in_progress;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void update_quiet_element(UWORD8 *msa,UWORD16 rx_len,
                                                     UWORD16 tag_param_offset);
extern void identify_measure_req(UWORD8* msa,UWORD16 rx_len,UWORD16 mac_hdr_len,
                                                               UWORD32 *rx_dscr);
extern void process_pending_requests(void);
extern void send_measurement_report(UWORD8 report_status, UWORD8 dia_token);
extern void send_autonomous_report(void);
extern void handle_dfs_sta(void);
extern void suspend_tx_dueto_radar(void);
extern UWORD16 set_sup_channel_list_element(UWORD8* data, UWORD16 index);

#ifndef OS_LINUX_CSL_TYPE
extern void on_quiet_offset_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_quiet_offset_expiry(ADDRWORD_T data);
extern void on_quiet_offset_expiry_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

#ifndef OS_LINUX_CSL_TYPE
extern void on_quiet_dur_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_quiet_dur_expiry(ADDRWORD_T data);
extern void on_quiet_dur_expiry_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

#ifndef OS_LINUX_CSL_TYPE
extern void on_reminding_to_msr_ch(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_reminding_to_msr_ch(ADDRWORD_T data);
extern void on_reminding_to_msr_ch_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

#ifndef OS_LINUX_CSL_TYPE
extern void on_measurement_complete(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_measurement_complete(ADDRWORD_T data);
extern void on_measurement_complete_work(struct work_struct *data);
#endif /* OS_LINUX_CSL_TYPE */

#ifndef OS_LINUX_CSL_TYPE
extern void on_csa_wait_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_csa_wait_expiry(ADDRWORD_T data);
extern void on_csa_wait_expiry_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function is used to copy prepared measurement report to measurement   */
/* report frame */
INLINE UWORD16 set_info_elements(UWORD8 *msa,UWORD16 index,
                                 UWORD8 report_status,UWORD8 count,UWORD8* msr_tkn)
{
    UWORD8 cnt = 0;

    switch(report_status)
    {
        case INCAPABLE :
        {
            for(;cnt < count;cnt++)
            msa[index++] = elem_incap[cnt];

            *msr_tkn = elem_incap[2];
        }
        break;
        case REC_LATE:
        {
            /* Dialog Token */
            msa[MAC_HDR_LEN+2] = g_head_of_req_q->dia_token;

            /* Measurement Report ID */
            msa[index++] = IMEASUREMENTREPORT;

            /* Lenght of the Element */
            msa[index++] = IMEASUREMENTREPORT_LEN_MIN;

            /* Measurement Token */
            msa[index++] = g_head_of_req_q->token;

            /* Measurement Report Mode field */
            msa[index++] = REC_LATE_REPORT;

            /* Measurement Type */
            msa[index++] = BASICREPORT;

            *msr_tkn = g_head_of_req_q->token;
        }
        break;
        case REFUSED_REQ:
        {
            /* Dialog Token */
            msa[MAC_HDR_LEN+2] = g_head_of_req_q->dia_token;

            /* Measurement Report ID */
            msa[index++] = IMEASUREMENTREPORT;

            /* Lenght of the Element */
            msa[index++] = IMEASUREMENTREPORT_LEN_MIN;

            /* Measurement Token */
            msa[index++] = g_head_of_req_q->token;

            /* Measurement Report Mode field */
            msa[index++] = REFUSED_REPORT;

            /* Measurement Type */
            msa[index++] = BASICREPORT;

            *msr_tkn = g_head_of_req_q->token;
        }
        break;
        case STA_REPORT:
        {
            /* Dialog Token */
            msa[MAC_HDR_LEN+2] = g_head_of_req_q->dia_token;

            /* Measurement Report ID */
            msa[index++]  = IMEASUREMENTREPORT;

            /* Lenght of the Element */
            msa[index++]  = IMEASUREMENTREPORT_LEN_MIN + BASICREPORT_LEN;

            /* Measurement Token */
            msa[index++]  = g_head_of_req_q->token;

            /* Measurement Report Mode field */
            msa[index++]  = 0;

            /* Measurement Type */
            msa[index++]  = BASICREPORT;

            /* Measurement Report */
            /* Channel number */
            msa[index++]  = g_head_of_req_q->meas_channel;

            /* Measurement start time in micro secs (8bytes) */
            msa[index++]  = (g_head_of_req_q->start_time_lo);
            msa[index++]  = (g_head_of_req_q->start_time_lo)>>8;
            msa[index++]  = (g_head_of_req_q->start_time_lo)>>16;
            msa[index++]  = (g_head_of_req_q->start_time_lo)>>24;
            msa[index++]  = (g_head_of_req_q->start_time_hi);
            msa[index++]  = (g_head_of_req_q->start_time_hi)>>8;
            msa[index++]  = (g_head_of_req_q->start_time_hi)>>16;
            msa[index++]  = (g_head_of_req_q->start_time_hi)>>24;

            /* Measurement duration in TUs (2Bytes) */
            msa[index++]  = (g_head_of_req_q->dur);
            msa[index++]  = (g_head_of_req_q->dur)>>8;

            /* Measurement report */
            msa[index++]  = g_report_byte;


            *msr_tkn = g_head_of_req_q->token;
        }
        break;
        default:
        {
        }
    }

    return index;
}

/* Function is used to get action field */
INLINE UWORD8 get_action_field(UWORD8* msa)
{
    UWORD16 index = MAC_HDR_LEN;

    return(msa[index+1]);
}

/* Function is used to get dialog token field */
INLINE UWORD8 get_dialog_token_field(UWORD8* msa)
{
    UWORD16 index = MAC_HDR_LEN;

    return(msa[index+2]);
}

/* Function is used to get measurement token field */
INLINE UWORD8 get_meas_token_field(UWORD8* msa)
{
    UWORD16 index = MAC_HDR_LEN;

    return(msa[index+5]);
}

/* Creates dfs alarms    */
INLINE void create_dfs_alarms(void)
{
    /* This alarm for quieting the channel */
    g_dfs_alarm_quiet_offset = create_alarm(on_quiet_offset_expiry, 0, on_quiet_offset_expiry_work);

    /* This alarm for quieting the channel */
    g_dfs_alarm_quiet_dur = create_alarm(on_quiet_dur_expiry, 0, on_quiet_dur_expiry_work);

    /* This alarm is for measurement start time */
    g_dfs_alarm_remind_ch_msr = create_alarm(on_reminding_to_msr_ch, 0, on_reminding_to_msr_ch_work);

    /* This alarm for measurement end time */
    g_dfs_alarm_ch_msr = create_alarm(on_measurement_complete, 0, on_reminding_to_msr_ch_work);

    /* This alarm for suspend Tx when STA detects radar */
    g_dfs_alarm_wait_for_csa = create_alarm(on_csa_wait_expiry, 0, on_csa_wait_expiry_work);
}

/* Delete dfs alarms */
INLINE void delete_dfs_alarms(void)
{
    if(g_dfs_alarm_quiet_offset != NULL)
    {
        delete_alarm(&g_dfs_alarm_quiet_offset);
    }
    
    if(g_dfs_alarm_quiet_dur != NULL)
    {
        delete_alarm(&g_dfs_alarm_quiet_dur);
    }

    if(g_dfs_alarm_remind_ch_msr != NULL)
    {
        delete_alarm(&g_dfs_alarm_remind_ch_msr);
    }
    
    if(g_dfs_alarm_ch_msr != NULL)
    {
        delete_alarm(&g_dfs_alarm_ch_msr);
    }
    
    if(g_dfs_alarm_wait_for_csa != NULL)
    {
        delete_alarm(&g_dfs_alarm_wait_for_csa);
    }
}

/* This function extracts quiet count from the incoming quiet element and    */
/* returns the same.                                                         */
INLINE UWORD8 get_quiet_count(UWORD8* data)
{
    return data[2];
}

/* This function extracts quiet duration from the incoming quiet element and */
/* returns the same.                                                         */
INLINE UWORD16 get_quiet_duration(UWORD8* data)
{

    UWORD16 quiet_dur = 0;

    quiet_dur  = data[4];
    quiet_dur |= (data[5] << 8) ;

    return quiet_dur;
}

/* This function extracts quiet offset from the incoming quiet element and   */
/* returns the same.                                                         */
INLINE UWORD16 get_quiet_offset(UWORD8* data)
{
    UWORD16 quiet_offs = 0;

    quiet_offs  = data[6];
    quiet_offs |= (data[7] << 8) ;

    return quiet_offs;
}

/* This function is used to set category field in action frame               */
INLINE UWORD16 set_category(UWORD8 *msa,UWORD8 cat,UWORD16 index)
{
    msa[index++] = cat;

    return index;
}

/* This function is used to set action field in action frame                 */
INLINE UWORD16 set_action(UWORD8 *msa, UWORD8 act,UWORD16 index)
{
    msa[index++] = act;

    return index;
}

/* This function is used to set dialog token field in action frame           */
INLINE UWORD16 set_dia_token(UWORD8 *msa, UWORD8 dia_token,UWORD16 index)
{
    msa[index++] = dia_token;

    return index;
}

/* This function is used to set element id field in action frame             */
INLINE UWORD16 set_elem_id(UWORD8 *msa, UWORD8 id,UWORD16 index)
{
    msa[index++] = id;

    return index;
}

/* This function is used to set length field in action frame information     */
/* element */
INLINE UWORD16 set_length(UWORD8 *msa, UWORD8 len,UWORD16 index)
{
    msa[index++] = len;

    return index;
}

/* This function is used to set measurement token in action frame            */
/* information element */
INLINE UWORD16 set_meas_token(UWORD8 *msa, UWORD8 token,UWORD16 index)
{
    msa[index++] = token;

    return index;
}

/* This function is used to set measurement mode field in action frame       */
/* information element */
INLINE UWORD16  set_meas_mode(UWORD8 *msa, UWORD8 mode,UWORD16 index)
{
    msa[index++] = mode;

    return index;
}

/* This function is used to set measurement type field in action frame       */
/* information element */

INLINE UWORD16  set_meas_type(UWORD8 *msa, UWORD8 type,UWORD16 index)
{
    msa[index++] = type;

    return index;
}

/* This function is used to set channel field in basic report in action frame */
INLINE UWORD16  set_ch_number(UWORD8 *msa, UWORD8 ch,UWORD16 index)
{
    msa[index++] = ch;

    return index;
}

/* This function is used to set measurement start time and measurement     */
/* duration in basic report */
INLINE UWORD16  set_measurement_time_dur(UWORD8 *msa,UWORD16 index)
{
    UWORD32 hi = 0;
    UWORD32 lo = 0;

    get_machw_tsf_timer(&hi, &lo);

    /* Measurement start time 8 bytes in microsec */
    msa[index++] = (UWORD8)(lo&0xF0);
    msa[index++] = (UWORD8)(lo>>8);
    msa[index++] = (UWORD8)(lo>>16);
    msa[index++] = (UWORD8)(lo>>24);

    msa[index++] =(UWORD8)hi;
    msa[index++] =(UWORD8)(hi>>8);
    msa[index++] =(UWORD8)(hi>>16);
    msa[index++] =(UWORD8)(hi>>24);

    /* Measurement duration in TU  2 bytes */
    /* Measurement duration(in TU) is 16*1.024msec (LSByte) */
    msa[index++] =MEASUREMENT_DUR;

    /*  MSByte in TU*/
    msa[index++] = 0;

    return index;
}

/* This function is used to set basic report measurement type mode field in */
/* action frame */
INLINE void set_basic_report(UWORD8 *msa,UWORD16 index)
{
    msa[index] = 0x08;
}

/* Initilize DFS globals  */
INLINE void init_dfs_sta_globals(void)
{
    g_num_chnls_to_be_scanned          = 0;
    g_report_byte                      = 0;
    g_head_of_req_q                    = NULL;
    g_silence_mode                     = BFALSE;
    g_waiting_to_shift_channel         = BFALSE;
    g_radar_detected                   = BFALSE;
    g_autonomous_basic_report          = BTRUE;
    g_waiting_for_csa                  = BFALSE;
    g_measurement_in_progress          = BFALSE;
    g_ch_msr_reminder_running          = BFALSE;
    g_waiting_for_AP                   = BFALSE;
    g_channel_shift                    = BFALSE;
    g_sending_aut_report_in_progress   = BFALSE;
}

/* This function is used to calculate the time difference between two        */
/* request Elements */
INLINE UWORD32 time_diff(UWORD32 future_hi, UWORD32 future_lo,
                         UWORD32 past_hi, UWORD32 past_lo)
{
    if(future_hi > past_hi + 1)
    {
        return 0xFFFFFFFF;
    }
    else if(future_hi == (past_hi + 1))
    {
        if(future_lo > past_lo)
            return 0xFFFFFFFF;
        else
            return (0xFFFFFFFF + future_lo - past_lo);
    }
    else if(future_hi == past_hi)
    {
        if(future_lo > past_lo)
            return (future_lo - past_lo);
        else
            return 0;
    }
    else
    {
        return 0;
    }
}

/* This function is used to find  which measurement element starts first */
INLINE BOOL_T is_time_greater(UWORD32 future_hi, UWORD32 future_lo,
                              UWORD32 past_hi, UWORD32 past_lo)
{
    if(future_hi > past_hi)
    {
        return BTRUE;
    }
    else if(future_hi == past_hi)
    {
        if(future_lo > past_lo)
        {
            return BTRUE;
        }
    }
    return BFALSE;
}

/* This function is used to find  which measurement element starts first */
INLINE BOOL_T is_new_element_later(req_meas_elem *new_req_elem,
                                      req_meas_elem *curr_elem)
{
    if((new_req_elem == NULL) || (curr_elem == NULL))
    {
        return BFALSE;
    }

    return is_time_greater(new_req_elem->start_time_hi,
                            new_req_elem->start_time_lo,
                            curr_elem->start_time_hi,
                            curr_elem->start_time_lo);
}

/* This function is used to find whether two measurement request elements */
/* overlap with each other */
INLINE BOOL_T are_they_overlapping(req_meas_elem *past_elem,
                                   req_meas_elem *future_elem)
{
    UWORD32 duration_in_us   = 0;
    UWORD32 diff_time_in_us  = 0;

    if((past_elem == NULL) || (future_elem == NULL))
    {
        return BFALSE;
    }

    duration_in_us = 1024 * past_elem->dur;

    diff_time_in_us = time_diff(future_elem->start_time_hi,
                                 future_elem->start_time_lo,
                                 past_elem->start_time_hi,
                                 past_elem->start_time_lo);

    if(diff_time_in_us < duration_in_us)
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function is used to check whether DFS station can go to Doze state  */
/* in Power save Mode */
INLINE BOOL_T can_dfs_sta_doze_now(void)
{
    if(g_waiting_for_csa == BTRUE)
    {
        return BFALSE;
    }
    else
    {
        return BTRUE;
    }
}

/* This function is used to check whether DFS-STA can transmits */
INLINE BOOL_T can_dfs_sta_transmit_now(void)
{
    if((g_silence_mode    == BTRUE)          ||
       (g_waiting_for_csa == BTRUE)          ||
       (g_measurement_in_progress == BTRUE))
    {
        return BFALSE;
    }
    else
    {
        return BTRUE;
    }
}

/* Check whether this channel is blocked by Radar */
INLINE BOOL_T is_this_chnl_blocked(UWORD8 freq, UWORD8 ch_idx)
{
    BOOL_T           ret_val   = BFALSE;
    UWORD8           reg_class = 0;
    reg_class_info_t *rc_info  = NULL;

    /* If DFS is enabled, check if DFS is required based on the Regulatory   */
    /* Class information for this channel.                                   */
    if(mget_enableDFS() == BTRUE)
    {
        reg_class = get_reg_class_from_ch_idx(freq, ch_idx);
		// 20120830 caisf mod, merged ittiam mac v1.3 code
        //rc_info   = get_reg_class_info(freq, reg_class);
		rc_info   = get_reg_class_info(freq, reg_class, get_current_reg_domain());

        if(rc_info != NULL)
            ret_val = is_rc_dfs_req(rc_info);
    }

    return ret_val;
}


/* Update 11h related protocol parameters */
INLINE void update_11h_prot_params(UWORD8 *msa, UWORD16 rx_len)
{
    if(mget_enableDFS() == BTRUE)
    {
        UWORD8 cap_info = msa[TAG_PARAM_OFFSET- 1];

        /* Check spectrum management bit in the capability info */
        /* of the beacon if it is reset then disable radar detection */
        if(!((cap_info & BIT0) == BIT0))
        {
             disable_radar_detection();
             mask_machw_radar_det_int();
        }
        else
        {
             enable_radar_detection();
             unmask_machw_radar_det_int();
        }

        update_quiet_element(msa, rx_len, TAG_PARAM_OFFSET);
    }
}

#endif /* DFS_STA_H */

#endif /* IBSS_BSS_STATION_MODE */

#endif /* MAC_802_11H */
