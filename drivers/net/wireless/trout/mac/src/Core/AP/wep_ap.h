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
/*  File Name         : wep_ap.h                                             */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AP mode for WEP encryption                           */
/*                                                                           */
/*  List of Functions : create_msg_3_timer                                   */
/*                      start_msg_3_timer                                    */
/*                      cancel_msg_3_timer                                   */
/*                      filter_wlan_rx_wep_ap                                */
/*                      filter_host_rx_wep_ap                                */
/*                      reset_wep_entry_ap                                   */
/*                      delete_wep_entry_ap                                  */
/*                      check_wep_ucast_wlan_2_wlan_ap                       */
/*                      check_wep_bcast_wlan_2_wlan_ap                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef WEP_AP_H
#define WEP_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ce_lut.h"
#include "index_util.h"
#include "itypes.h"
#include "management.h"
#include "prot_if.h"
#include "wep.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
extern void msg_3_timeout(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void msg_3_timeout(ADDRWORD_T data);
extern void msg_3_timeout_work(struct work_struct *work);

#endif /* OS_LINUX_CSL_TYPE */


/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* This function creates the management timeout timer */
INLINE ALARM_HANDLE_T *create_msg_3_timer(ALARM_FUNC_T *func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func)
{
    return create_alarm(func, data, work_func);
}


/* This function starts the management timeout timer with the given time. */
INLINE void start_msg_3_timer(ALARM_HANDLE_T *handle, UWORD16 time)
{
    if(handle != NULL)
    {
        stop_alarm(handle);
        start_alarm(handle, time);
    }
}

/* This function cancels the management timeout timer. */
INLINE void cancel_msg_3_timer(ALARM_HANDLE_T* handle)
{
    if(handle != NULL)
    {
        stop_alarm(handle);
    }
}


/* Function filters the Frame for WEP encryption type in Access point mode */
/* using the MIB Exclude paramter. Returns TRUE if the frame is to be      */
/* discarded; False otherwise                                              */
INLINE BOOL_T filter_wlan_rx_wep_ap(CIPHER_T ct, UWORD8 type_subtype)
{
    if(is_wep_enabled() == BFALSE)
    {
        return BFALSE;
    }

    /* If Data is absent, no check is required */
    if(((type_subtype & 0xF0) >> 4) > 3)
    {
        /* No check is requried */
        return BFALSE;
    }

    if(mget_ExcludeUnencrypted() == TV_TRUE)
    {
        if(ct != g_wep_type)
        {
            /* Discard the frame */
            mincr_WEPExcludedCount();
            return BTRUE;
        }
    }
    return BFALSE;
}

/* Set cipher type in the host Rx Frame filtering process for */
/* Access point mode for WEP encryption                       */
/* Returns false if the frame must be processed and true if   */
/* the frame be dropped without any processing                */
INLINE BOOL_T filter_host_rx_wep_ap(CIPHER_T *ct)
{
    /* If Privacy is invoked, select WEP as the cipher type */
    if(is_wep_enabled() == BTRUE)
    {
        /* Get the WEP Type currenctly in use */
        *ct = get_wep_type(mget_WEPDefaultKeyID());
    }

    return BFALSE;
}

/* Reset the WEP Entry by flushing the CE LUT */
INLINE void reset_wep_entry_ap(asoc_entry_t *ae)
{
    /* If WEP is enabled, remove the STA's entry from LUT */
    if(is_wep_enabled() == BTRUE)
    {
        /* Remove the Key index from the CE-LUT */
        if(ae->sta_index != INVALID_ID)
        {
             machw_ce_del_key(ae->sta_index);
        }
    }
}

/* Delete the WEP Entry by flushing the CE LUT and freeing necessary data */
/* held in the association tables                                         */
INLINE void delete_wep_entry_ap(asoc_entry_t *ae)
{
    /* If WEP is enabled, clear the challenge text if existent */
    /* Also clear the index number and the LUT Entry           */
    if(is_wep_enabled() == BTRUE)
    {
        if(ae->sta_index != INVALID_ID)
        {
            machw_ce_del_key(ae->sta_index);
        }
    }
}

/* Before forwarding the packet across the WLAN-WLAN interface */
/* security checks needs to performed on the states of the     */
/* transmitting and receiving stations                         */
INLINE BOOL_T check_wep_ucast_wlan_2_wlan_ap(CIPHER_T *ct)
{
    /* If WEP is enabled, set cipher type accordingly */
    if(is_wep_enabled() == BTRUE)
    {
        /* Read the cipher type value for the cipher suite */
        *ct = get_wep_type(mget_WEPDefaultKeyID());
    }
    return BTRUE;
}

/* Before forwarding the packet across the WLAN-WLAN interface */
/* security checks needs to performed on the states of the     */
/* transmitting station                                        */
INLINE void check_wep_bcast_wlan_2_wlan_ap(CIPHER_T *ct)
{
    /* If WEP is enabled, set cipher type accordingly */
    if(is_wep_enabled() == BTRUE)
    {
        /* Read the cipher type value for the cipher suite */
        *ct = get_wep_type(mget_WEPDefaultKeyID());
    }
}



#endif /* WEP_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
