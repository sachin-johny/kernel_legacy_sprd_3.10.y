/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : sta_uapsd.h                                          */
/*                                                                           */
/*  Description       : This file contains all the functions and definitions */
/*                      related to STA WMM PS (UAPSD) feature.               */
/*                                                                           */
/*  List of Functions : init_uapsd_globals                                   */
/*                      set_uapsd_status_qap_cap                             */
/*                      is_ap_uapsd_capable                                  */
/*                      is_uapsd_sp_in_progress                              */
/*                      start_uapsd_sp                                       */
/*                      stop_uapsd_sp                                        */
/*                      set_uapsd_config_ac                                  */
/*                      get_uapsd_config_ac                                  */
/*                      set_uapsd_config_max_sp_len                          */
/*                      get_uapsd_config_max_sp_len                          */
/*                      is_legacy                                            */
/*                      is_legacy_ac_present                                 */
/*                      is_trigger_enabled                                   */
/*                      is_delivery_enabled                                  */
/*                      is_trigger_frame                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_WMM

#ifndef STA_UAPSD_H
#define STA_UAPSD_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management_11e.h"
#include "frame_11e.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* STA U-APSD configuration */
typedef struct
{
    UWORD8 max_sp_length;
    UWORD8 delivery_enabled[NUM_AC];
    UWORD8 trigger_enabled[NUM_AC];
} sta_uapsd_config_t;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* STA U-APSD Service Period status */
typedef enum {SP_NOT_IN_PROGRESS, /* UAPSD service period is not in progress */
              WAIT_SP_START,      /* Wait for UAPSD service period to start  */
              SP_IN_PROGRESS      /* UAPSD service period is in progress     */
} STA_UAPSD_SP_STAT_T;

// 20120709 caisf add, merged ittiam mac v1.2 code
//#define EOSP_TIMEOUT_CNT   10    /* 10 TBTTS */
#define EOSP_TIMEOUT_CNT   2      /* 2 TBTTS */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T              g_qap_uapsd_capable;
extern STA_UAPSD_SP_STAT_T g_uapsd_sp_status;
extern sta_uapsd_config_t  g_sta_uapsd_config;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void uapsd_handle_wlan_rx_sta(UWORD8 *msa, UWORD8 priority);
extern void uapsd_trigger_sp_sta(void);
extern void uapsd_handle_tx_complete_sta(UWORD8 *msa, UWORD8 tx_success);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function initializes all global variables used for UAPSD in STA mode */
INLINE void init_uapsd_globals(void)
{
    g_uapsd_sp_status   = SP_NOT_IN_PROGRESS;
    g_qap_uapsd_capable = BFALSE;
}

/* This function sets the UAPSD capability of the QAP */
INLINE void set_uapsd_status_qap_cap(BOOL_T val)
{
    g_qap_uapsd_capable = val;
}

/* This function gets the UAPSD capability of the QAP */
INLINE BOOL_T is_ap_uapsd_capable(void)
{
    return g_qap_uapsd_capable;
}

/* This function checks if a UAPSD service period is not in progress */
INLINE BOOL_T is_uapsd_sp_not_in_progress(void)
{
    if(g_uapsd_sp_status == SP_NOT_IN_PROGRESS)
        return BTRUE;
    else
        return BFALSE;
}

/* This function checks if wait for UAPSD service period is in progress */
INLINE BOOL_T is_uapsd_sp_wait_in_progress(void)
{
    if(g_uapsd_sp_status == WAIT_SP_START)
        return BTRUE;
    else
        return BFALSE;
}

/* This function waits for a UAPSD service period to start. Note that the    */
/* service period will be started only after successful transmission of the  */
/* trigger frame. In case the trigger frame is timed out the STA will try to */
/* retransmit a trigger frame. Also if it is already waiting for a service   */
/* period to start or if a service period is in progress no wait is required */
/* and hence the flag is not set.                                            */
INLINE void wait_uapsd_sp_start(void)
{
    if(g_uapsd_sp_status == SP_NOT_IN_PROGRESS)
        g_uapsd_sp_status = WAIT_SP_START;

#ifdef DEBUG_MODE
        g_mac_stats.wmmpssta_spsw++;
#endif /* DEBUG_MODE */
}

/* This function starts a UAPSD service period */
INLINE void start_uapsd_sp(void)
{
    g_uapsd_sp_status = SP_IN_PROGRESS;

#ifdef DEBUG_MODE
        g_mac_stats.wmmpssta_sps++;
#endif /* DEBUG_MODE */
}

/* This function stops a UAPSD service period. */
INLINE void stop_uapsd_sp(void)
{
    g_uapsd_sp_status = SP_NOT_IN_PROGRESS;

#ifdef DEBUG_MODE
        g_mac_stats.wmmpssta_spe++;
#endif /* DEBUG_MODE */
}

/* This function sets the configured WMM PS settings for a given AC. Note    */
/* that currently the trigger and delivery enabled configurations are the    */
/* same. Hence both trigger and delivery values are set to the configured    */
/* value.                                                                    */
INLINE void set_uapsd_config_ac(UWORD8 ac, UWORD8 val)
{
    if(ac < NUM_AC)
    {
        g_sta_uapsd_config.trigger_enabled[ac]  = val;
        g_sta_uapsd_config.delivery_enabled[ac] = val;
    }
}

/* This function gets the configured WMM PS settings for a given AC. Note    */
/* that currently the trigger and delivery enabled configurations are the    */
/* same. Hence any one value (trigger or delivery) can be used to return the */
/* configured value.                                                         */
INLINE UWORD8 get_uapsd_config_ac(UWORD8 ac)
{
    if(ac < NUM_AC)
    {
        return g_sta_uapsd_config.trigger_enabled[ac];
    }

    return 0;
}

/* This function sets the WMM PS maximum service period length configured */
INLINE void set_uapsd_config_max_sp_len(UWORD8 max_sp_len)
{
    g_sta_uapsd_config.max_sp_length = max_sp_len;
}

/* This function gets the configured WMM PS maximum service period length */
INLINE UWORD8 get_uapsd_config_max_sp_len(void)
{
    return g_sta_uapsd_config.max_sp_length;
}

/* This function checks if AC mapped to given priority is legacy */
INLINE BOOL_T is_legacy(UWORD8 priority)
{
    UWORD8 ac = 0;

    if(map_priority_to_edca_ac(priority, &ac) == BTRUE)
    {
        if(ac < NUM_AC)
        {
            if((g_sta_uapsd_config.trigger_enabled[ac] == 0) &&
               (g_sta_uapsd_config.delivery_enabled[ac] == 0))
               return BTRUE;
        }
    }

    return BFALSE;
}

/* This function checks if any legacy AC is present */
INLINE BOOL_T is_legacy_ac_present(void)
{
    UWORD8 ac = 0;

    for(ac = 0; ac < NUM_AC; ac++)
    {
        if((g_sta_uapsd_config.trigger_enabled[ac] == 0) &&
           (g_sta_uapsd_config.delivery_enabled[ac] == 0))
           return BTRUE;
    }

    return BFALSE;
}

/* This function checks if AC mapped to given priority is trigger enabled */
INLINE BOOL_T is_trigger_enabled(UWORD8 priority)
{
    UWORD8 ac = 0;

    if(map_priority_to_edca_ac(priority, &ac) == BTRUE)
    {
        if(ac < NUM_AC)
            return ((BOOL_T)(g_sta_uapsd_config.trigger_enabled[ac]));
    }

    return BFALSE;
}

/* This function checks if AC mapped to given priority is delivery enabled */
INLINE BOOL_T is_delivery_enabled(UWORD8 priority)
{
    UWORD8 ac = 0;

    if(map_priority_to_edca_ac(priority, &ac) == BTRUE)
    {
        if(ac < NUM_AC)
            return ((BOOL_T)(g_sta_uapsd_config.delivery_enabled[ac]));
    }

    return BFALSE;
}

/* This function checks if the given frame is a WMM PS trigger frame */
INLINE BOOL_T is_trigger_frame(UWORD8 *msa)
{
    UWORD8 frm_type = get_sub_type(msa);

    /* A trigger frame is a QoS Data or QoS Null frame from a WMM STA in     */
    /* power save mode associated with an AC the WMM STA has configured to   */
    /* be a trigger-enabled AC.                                              */
    if((frm_type == QOS_NULL_FRAME) || (frm_type == QOS_DATA))
    {
        UWORD8 pri = 0;

        pri = get_tid_value(msa);

        if(is_trigger_enabled(pri) == BTRUE)
        {
            return BTRUE;
        }
    }

    return BFALSE;
}

#endif /* STA_UAPSD_H */

#endif /* MAC_WMM */
#endif /* IBSS_BSS_STATION_MODE */
