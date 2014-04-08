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
/*  File Name         : cglobals.h                                           */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      for AP and STA mode.                                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CGLOBALS_H
#define CGLOBALS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define REG_VAL_DIFF      10

#define JOIN_START_TIMEOUT                10000
#define AUTH_MSG3_TIMEOUT                 10000
#define ASOC_TIMEOUT                      10000
#define HANDSHAKE_TIMEOUT_802_11I         40000
#define RESP_TIMEOUT_801_11I              1000
#define GTKHS_PERSTA_TIMEOUT              (2 * RESP_TIMEOUT_801_11I)
#define GTKHS_MIN_TIMEOUT                 5000

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern UWORD8             g_beacon_index;
extern UWORD8             g_ack_policy;
extern UWORD8             g_preamble_type;
extern UWORD8             g_pcf_mode;
extern UWORD8             g_bcst_ssid;
extern UWORD8             g_addr_sdram;
extern UWORD8             g_phy_active_reg;
extern UWORD8  			  g_current_mac_status;
extern UWORD16            g_beacon_len;
extern UWORD32            g_phy_ver_num;
extern UWORD32 			  g_memory_address;

//chenq mod 2012-11-02
//extern BOOL_T             g_mac_reset_done;
extern atomic_t g_mac_reset_done;

extern BOOL_T             g_rx_buffer_based_ack;
extern UWORD8             g_disable_rtscts_norsp;
extern UWORD8             g_user_allow_rifs_tx;
extern UWORD16            g_user_pref_channel;
extern UWORD8             g_device_mode;

#ifdef TX_ABORT_FEATURE
extern BOOL_T             g_machw_tx_aborted;
#endif /* TX_ABORT_FEATURE */
extern UWORD8             *g_beacon_frame[2];
extern ALARM_HANDLE_T     *g_mgmt_alarm_handle;
extern INTERRUPT_HANDLE_T *g_mac_isr_handle;
extern WORD8             g_rssi;
//chenq add snr
extern WORD16             g_snr;

//chenq add 2012-10-30
extern ALARM_HANDLE_T     *g_mac_event_timer;

extern BOOL_T             g_short_preamble_enabled;
extern BOOL_T             g_short_slot_allowed;
extern BOOL_T             g_phy_continuous_tx_mode_enable;
extern BOOL_T             g_reset_mac_in_progress;

extern UWORD16            g_join_timeout;
extern UWORD16            g_11i_protocol_timeout;
extern UWORD16            g_eapol_response_timeout;

/* Tx-Calibration */
extern UWORD8             g_target_tssi_11a[16];
extern UWORD8             g_target_tssi_11b[16];
extern UWORD8             g_wid_tx_cal;

#ifdef MAC_HW_UNIT_TEST_MODE
extern WORD16             g_rssi_ex;
extern WORD16             g_adj_rssi_ex;
extern UWORD16            g_rssi_ex_count;
extern WORD16             g_target_rssi_11b;
extern WORD16             g_target_rssi_11g;
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef BSS_ACCESS_POINT_MODE
extern UWORD8             g_decr_fail_cnt[];
#else /* BSS_ACCESS_POINT_MODE */
extern UWORD8             g_decr_fail_cnt;
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef BURST_TX_MODE
extern BOOL_T             g_burst_tx_mode_enabled;
extern BOOL_T             g_burst_tx_mode_enabled_earlier;
#endif /* BURST_TX_MODE */

extern BOOL_T             g_user_control_enabled;
// 20120709 caisf add, merged ittiam mac v1.2 code
extern BOOL_T           g_int_supp_enable;

//chenq add for cur tx rate 2012-10-17
extern UWORD8           g_cur_tx_rate;

#include <linux/wakelock.h>
extern struct wake_lock reset_mac_lock; /*Keep awake when resetting MAC, by keguang 20130609*/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
extern struct wake_lock scan_ap_lock; /*Keep awake when scan ap, caisf add, 20130929*/
extern struct wake_lock deauth_err_lock; /*Keep awake when system error deauth, caisf add, 20130929*/
extern struct wake_lock handshake_frame_lock; /*Keep awake when rx handshake frames, caisf add, 20131004*/ 
extern ALARM_HANDLE_T *g_hs_wake_timer;
extern void del_hs_wake_timer(void);
extern struct wake_lock buffer_frame_lock; /*Keep awake when receiving buffered frames, by keguang 20130904*/
extern struct timer_list buffer_frame_timer;
#endif
#endif

//chenq add for protect event q 2012-11-01
//extern spinlock_t g_event_spin_lock;

#ifdef MEM_STRUCT_SIZES_INIT
/* The maximum length of the name of a structure                         */
#define MEM_MAX_STRUCT_NAME      50

/* The maximum number of structures in the MAC s/w code                  */
#define MEM_MAX_NUMBER_STRUCTS   50

typedef struct
{
	UWORD16     size;
	char        name[MEM_MAX_STRUCT_NAME];
} struct_sizes_t;

extern struct_sizes_t g_struct_sizes[MEM_MAX_NUMBER_STRUCTS];
extern UWORD16        g_struct_count;

#endif /*MEM_STRUCT_SIZES_INIT*/

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    init_globals(void);
extern UWORD32 mac_isr(UWORD32 vector, UWORD32 data);


INLINE BOOL_T is_auto_chan_sel_enabled(void)
{
	if(g_user_pref_channel & 0xFF)
	{
		return BFALSE;
	}
	else
	{
		return BTRUE;
	}
}

INLINE UWORD8 get_user_pref_primary_ch(void)
{
	return (g_user_pref_channel & 0xFF);
}

INLINE UWORD8 get_user_pref_sec_ch(void)
{
	return ((g_user_pref_channel & 0x0300) >> 8);
}
#endif /* CGLOBALS_H */
