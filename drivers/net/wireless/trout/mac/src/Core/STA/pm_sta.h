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
/*  File Name         : pm_sta.h                                             */
/*                                                                           */
/*  Description       : This file contains STA power management related      */
/*                      definitions and inline functions.                    */
/*                                                                           */
/*  List of Functions : get_ps_state                                         */
/*                      set_ps_state                                         */
/*                      create_psm_alarms                                    */
/*                      set_hw_ps_mode                                       */
/*                      reset_hw_ps_mode                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef PSM_STA_H
#define PSM_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"
#include "mh.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define ACTIVITY_TIME_OUT     10000 /* 10 sec */
#define MIN_PS_ACTIVITY_TO    30    /* 30 ms  */


/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* TIM processing result */
typedef enum {TIM_IS_SET  = 1,
              DTIM_IS_SET = 2
} TIM_PROC_RESULT_T;

/* Station power save modes */
typedef enum {MIB_ACTIVE    = 1,
              MIB_POWERSAVE = 2
} PS_MODE_T;

/* Power save modes specified by the user */
typedef enum {NO_POWERSAVE     = 0,
              MIN_FAST_PS      = 1,
              MAX_FAST_PS      = 2,
              MIN_PSPOLL_PS    = 3,
              MAX_PSPOLL_PS    = 4
} USER_PS_MODE_T;

#define USER_PS_MODE_2STR(v)  (((v) == NO_POWERSAVE) ? "NO_POWERSAVE" : \
                              (((v) == MIN_FAST_PS) ? "MIN_FAST_PS" : \
                              (((v) == MAX_FAST_PS) ? "MAX_FAST_PS" : \
                              (((v) == MIN_PSPOLL_PS) ? "MIN_PSPOLL_PS" : \
                              (((v) == MAX_PSPOLL_PS) ? "MAX_PSPOLL_PS" : \
                                                        "UNKNOWN USER PS MODE")))))

/* Station power save states. When the station is in MIB_ACTIVE mode the     */
/* station will be in STA_ACTIVE state always. When it is in MIB_POWERSAVE   */
/* mode, the station will switch between the following three states. Note    */
/* that whenever the station is in STA_ACTIVE state AP will be informed that */
/* it is in MIB_ACTIVE mode.                                                 */
typedef enum {STA_ACTIVE    = 0,
              STA_DOZE      = 1,
              STA_AWAKE     = 2
} PS_STATE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern USER_PS_MODE_T g_user_ps_mode;
extern BOOL_T         g_receive_dtim;
extern UWORD32        g_activity_timeout ;
extern ALARM_HANDLE_T *g_psm_alarm_handle;
extern BOOL_T         g_beacon_frame_wait;
extern BOOL_T         g_active_null_wait;
extern BOOL_T         g_doze_null_wait;
extern BOOL_T         g_more_data_expected;
extern PS_STATE_T     g_ps_state;
extern UWORD32        g_tx_rx_activity_cnt;


// 20130216 caisf add for power management mode policy to take effect
extern USER_PS_MODE_T g_user_ps_mode_wait_apply; 
extern BOOL_T         g_receive_dtim_wait_apply;
extern UWORD8         dot11PowerManagementMode_wait_apply;
extern BOOL_T         g_powermanagement_wait_apply_flag;

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
extern UWORD32 ps_last_int_mask;

typedef enum
{
	SUSPEND_DOZE   = 0,
	LOW_POWER_DOZE = 1,
}DOZE_T;

extern void sta_doze_trick(DOZE_T type,BOOL_T reset_lock);
extern void sta_awake_trick(void);
extern int send_null_frame_to_AP_trick(UWORD8 psm, BOOL_T is_qos, UWORD8 priority);
extern int prepare_null_frame_for_cp(UWORD8 psm, BOOL_T is_qos, UWORD8 priority);
extern void prepare_ps_poll_for_cp(void);
#endif


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   initialize_psm_globals(void);
extern void   psm_handle_rx_packet_sta(UWORD8* msa, UWORD8 priority);
extern UWORD8 psm_handle_tx_packet_sta(UWORD8 priority);
extern void   handle_tx_complete_sta(UWORD32 *tx_dscr);
extern void   handle_tbtt_sta(void);
extern void   psm_handle_tim_elm(UWORD8* tim_elm);
extern void   init_ps_state_change(UWORD8 psm);
extern void   send_null_frame_to_AP(UWORD8 psm, BOOL_T is_qos, UWORD8 pri);
extern void   send_ps_poll_to_AP(void);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE // power sleep mode
extern void   sta_doze(void);
extern void   sta_awake(bool flag);

extern void   sta_sleep(void);
extern void   sta_wakeup(void);
#else   // normal mode
extern void   sta_doze(void);
extern void   sta_awake(void);
#endif

extern void   start_activity_timer(void);
extern void   stop_activity_timer(void);

#ifndef OS_LINUX_CSL_TYPE
extern void   psm_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void   psm_alarm_fn(ADDRWORD_T data);
extern void psm_alarm_work(struct work_struct *work);
#endif /* OS_LINUX_CSL_TYPE */


/*****************************************************************************/
/* Static Inline Functions                                                   */
/*****************************************************************************/
INLINE void set_user_ps_mode(UWORD8 ps_mode)
{
    g_user_ps_mode = (USER_PS_MODE_T)ps_mode;
}

/* This function gets the current station power save state */
INLINE UWORD8 get_user_ps_mode(void)
{
    return (UWORD8)g_user_ps_mode;
}

/* This function gets the current station power save type */
INLINE BOOL_T is_fast_ps_enabled(void)
{
    if((UWORD8)g_user_ps_mode >= (UWORD8)MIN_PSPOLL_PS)
    {
        return BFALSE;
    }
    else
    {
        return BTRUE;
    }
}

/* This function gets the current station power save state */
INLINE UWORD8 get_ps_state(void)
{
    return g_ps_state;
}

/* This function sets the station power save state */
INLINE void set_ps_state(UWORD8 val)
{
    g_ps_state  = (PS_STATE_T)val;
}

/* This function creates the power save alarm (activity timer) */
INLINE void create_psm_alarms(void)
{
    /* If the Power save alarm exists, stop and delete the same */
    if(g_psm_alarm_handle != 0)
    {
        stop_alarm(g_psm_alarm_handle);
        delete_alarm(&g_psm_alarm_handle);
    }
    g_psm_alarm_handle = create_alarm(psm_alarm_fn, 0, psm_alarm_work);
}

/* This functions sets MAC H/w power save mode (doze state, listen interval  */
/* and receive DTIM bit                                                      */
INLINE void set_hw_ps_mode(void)
{
    set_machw_listen_interval(mget_listen_interval());
    set_machw_ps_doze_mode();
    set_machw_ps_pm_tx_bit();

    if(g_receive_dtim == BFALSE)
    {
        set_machw_ps_rdtim_bit();
    }
    else
    {
        reset_machw_ps_rdtim_bit();
    }
}
/*zhq add for powersave*/
INLINE void root_set_hw_ps_mode(void)
{
    root_set_machw_listen_interval(mget_listen_interval());
    root_set_machw_ps_doze_mode();
    root_set_machw_ps_pm_tx_bit();

    if(g_receive_dtim == BFALSE)
    {
        root_set_machw_ps_rdtim_bit();
    }
    else
    {
        root_reset_machw_ps_rdtim_bit();
    }
}
/* This function disables MAC H/w power save mode */
INLINE void reset_hw_ps_mode(void)
{
    set_machw_ps_active_mode();
    reset_machw_ps_pm_tx_bit();
    reset_machw_ps_rdtim_bit();
}
/*zhq add for powersave*/
INLINE void root_reset_hw_ps_mode(void)
{
    root_set_machw_ps_active_mode();
    root_reset_machw_ps_pm_tx_bit();
    root_reset_machw_ps_rdtim_bit();
}

/* This function increments the activity counter by 1 and incase if counter  */
/* is zero activity timer is also started                                    */
INLINE void incr_activity_cnt(void)
{
    g_tx_rx_activity_cnt++;
}

/* This fucntion resets the acivity counter to ZERO */
INLINE void reset_activity_cnt(void)
{
    g_tx_rx_activity_cnt = 0;
}

INLINE BOOL_T is_activity_cnt_zero(void)
{
    BOOL_T ret_val = BFALSE;

    if(0 == g_tx_rx_activity_cnt)
        ret_val = BTRUE;

    return ret_val;
}

#endif /* PSM_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
