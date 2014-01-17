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
/*  File Name         : channel_sw.h                                         */
/*                                                                           */
/*  Description       : This file contains all the declarations and inline   */
/*                      functions for handling Channel Switching operation   */
/*                      in STA mode.                                         */
/*                                                                           */
/*  List of Functions : set_spectrum_mgmt_bit_sta                            */
/*                      create_csw_alarms                                    */
/*                      delete_csw_alarms                                    */
/*                      init_csw_sta_globals                                 */
/*                      is_csw_in_prog                                       */
/*                      check_spectrum_mgmt_bit                              */
/*                      update_csw_info                                      */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef CHANNEL_SW_H
#define CHANNEL_SW_H
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "csl_types.h"
#include "csl_if.h"
#include "mib.h"
#include "management.h"
#include "mh.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define SCAN_DFS_TIME        60000  /* 60sec */
#define LINK_LOSS_OFFSET_11H 5 /* 5 Beacons */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_waiting_for_AP;
extern BOOL_T g_waiting_to_shift_channel;
extern BOOL_T g_channel_swt_cnt_zero;

extern ALARM_HANDLE_T *g_csw_alarm_wait_for_AP;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void handle_tbtt_spec_mgmt_sta(void);
extern BOOL_T check_csa_ie (UWORD8 *msa,UWORD16 rx_len,
                                                     UWORD16 tag_param_offset);
extern void handle_spec_mgmt_action(UWORD8* msa,UWORD16 rx_len,
                                    UWORD8 mac_hdr_len, UWORD32* rx_dscr);
extern void switch_channel_csw(void);

#ifndef OS_LINUX_CSL_TYPE
extern void on_AP_wait_expiry(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void on_AP_wait_expiry(ADDRWORD_T data);
extern void on_AP_wait_expiry_work(struct work_struct *work);

#endif /* OS_LINUX_CSL_TYPE */

extern BOOL_T is_this_channel_supported_by_rf(UWORD8 new_channel);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* Set spectrum management bit in the capability information field */
INLINE void set_spectrum_mgmt_bit_sta(UWORD8* data, UWORD16 index)
{
    /* Set spectrum management bit in the capability info of association */
    /* request frame if DFsenbled or TPC enabled and AP supports dot11h  */
    /* features and STA is in Infrastructure mode */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        if(mget_SpectrumManagementRequired() == TV_TRUE)
        {
            data[index+1] |= (SPECTRUM_MGMT>>8);
        }
    }
}

/* Creates dfs alarms    */
INLINE void create_csw_alarms(void)
{
    /* This alarm for suspend Tx when STA detects radar */
    g_csw_alarm_wait_for_AP = create_alarm(on_AP_wait_expiry, 0, on_AP_wait_expiry_work);
}

/* Delete dfs alarms */
INLINE void delete_csw_alarms(void)
{
    if(g_csw_alarm_wait_for_AP != NULL)
    {
        delete_alarm(&g_csw_alarm_wait_for_AP);
    }
}

/* Initilize DFS globals  */
INLINE void init_csw_sta_globals(void)
{
    /* Added all the initialisation for spectrum management */
    g_waiting_for_AP                   = BFALSE;
    g_waiting_to_shift_channel         = BFALSE;
    g_channel_swt_cnt_zero             = BFALSE;
}

/* This function indicates whether the Channel Switching Process is */
/* in progress.                                                     */
INLINE BOOL_T is_csw_in_prog(void)
{
    if((g_waiting_to_shift_channel == BTRUE) ||
       (g_waiting_for_AP == BTRUE))
    {
        return BFALSE;
    }
    else
    {
        return BTRUE;
    }
}

/* Check spectrum management bit in the capability information field */
INLINE void check_spectrum_mgmt_bit(UWORD16 cap_info)
{
    if(TV_TRUE == mget_SpectrumManagementImplemented())
    {
        if((cap_info & SPECTRUM_MGMT) == SPECTRUM_MGMT)
        {
            /* Set spectrum management required bit if AP supports DFS */
            /* in the current channel */
            mset_SpectrumManagementRequired(TV_TRUE);
        }
        else
        {
            /* Reset spectrum management required bit if AP doesn't support DFS */
            /* in the current channel */
            mset_SpectrumManagementRequired(TV_FALSE);
        }
    }
}

/* Update 11h related protocol parameters */
INLINE void update_csw_info(UWORD8 *msa, UWORD16 rx_len)
{
    if(TV_TRUE == mget_SpectrumManagementImplemented())
    {
		BOOL_T csa_ie_present = check_csa_ie(msa, rx_len, TAG_PARAM_OFFSET);

        /* If (Extended) Channel Switch Announcement IE is present then it   */
        /* indicates Beacon is from channel the STA has switched. So no need */
        /* come out of waiting for AP in the new channel                     */
        if(BFALSE == csa_ie_present)
        {
            /* After switching the channel if beacons are received by STA */
            /* in the current channel then resume Tx and stop alarm */
            if(g_waiting_for_AP == BTRUE)
            {
                g_waiting_for_AP = BFALSE;
                stop_alarm(g_csw_alarm_wait_for_AP);
                enable_machw_tx();
            }
		}
    }
}

#endif /* CHANNEL_SW_H */
#endif /* IBSS_BSS_STATION_MODE */
