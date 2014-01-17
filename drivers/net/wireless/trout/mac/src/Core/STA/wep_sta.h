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
/*  File Name         : wep_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      STA mode for WEP encryption                          */
/*                                                                           */
/*  List of Functions : delete_wep_entry_sta                                 */
/*                      reset_wep_entry_sta                                  */
/*                      filter_host_rx_wep_sta                               */
/*                      filter_wlan_rx_wep_sta                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef WEP_STA_H
#define WEP_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ce_lut.h"
#include "mib.h"
#include "management.h"
#include "index_util.h"
#include "prot_if.h"
#include "wep.h"
#include "management_sta.h"

/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* Delete the WEP Entry by flushing the CE LUT */
INLINE void delete_wep_entry_sta(sta_entry_t *se)
{
    if(is_wep_enabled() == BTRUE)
    {
        if(se->sta_index != INVALID_ID)
        {
            machw_ce_del_key(se->sta_index);
        }
    }
}

/* Reset the WEP Entry by flushing the CE LUT */
INLINE void reset_wep_entry_sta(sta_entry_t *se)
{
    /* If WEP is enabled, remove the STA's entry from LUT */
    if(is_wep_enabled() == BTRUE)
    {
        /* Remove the Key index from the CE-LUT */
        if(se->sta_index != INVALID_ID)
        {
             machw_ce_del_key(se->sta_index);
        }
    }
}

/* Function filters the Frame for WEP encryption type in Station mode */
/* using the MIB Exclude paramter. Returns TRUE if the frame is to be */
/* discarded; False otherwise                                         */
INLINE BOOL_T filter_wlan_rx_wep_sta(CIPHER_T ct, UWORD8 type_subtype)
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
INLINE BOOL_T filter_host_rx_wep_sta(CIPHER_T *ct)
{
    /* If Privacy is invoked, select WEP as the cipher type */
    if(is_wep_enabled() == BTRUE)
    {
        /* Get the WEP Type currenctly in use */
        *ct = get_wep_type(mget_WEPDefaultKeyID());
    }

    return BFALSE;
}

/* Function is used to add wep for STA */
INLINE void add_wep_key_sta(UWORD8 *val)
{
    UWORD8 ct        = 0;
    UWORD8 mask      = LUT_ADDR_PRGM_MASK;
    UWORD8 key_size  = 0;
    UWORD8 key_id    = 0;
    UWORD8* key      = NULL;

    /* Delete existing keys */
    machw_ce_del_key(0);

    if(val == NULL)
    {
        return;
    }

    /*---------------------------------*/
    /* KeyID | KeyLength |   Key       */
    /*---------------------------------*/
    /*   1   |     1     |  KeyLength  */
    /*---------------------------------*/

    key_id   = *(val);
    key_size = *(val + 1);
    key      = (UWORD8*)(val + 2);

    switch(key_size)
    {
    case 5:
    {
        /* If Key size is 40, set cipher type to WEP40 */
        /* and key size is updated to 5                */
        ct       = WEP40;
    }
    break;
    case 13:
    {
        /* If Key size is 104, set cipher type to WEP104 */
        /* and key size is updated to 13                 */
        ct       = WEP104;
    }
    break;
    default:
    {
        /* If Key size is unknown, set cipher type to WEP40 */
        /* and key size is updated to 5                     */
        ct       = WEP40;
        key_size = 5;
    }
    break;
    }

    /* Set the wep type */
    g_wep_type = ct;

    /* Set the values to MIB */
    mset_WEPDefaultKeyID(key_id);
    mset_WEPKeyValue(key_id, key, key_size*8);

    set_machw_ce_grp_key_type(ct);

    /* Add Broadcast WEP Key */
    machw_ce_add_key(key_id, TX_GTK_NUM, 0, mask, ct, key, mget_bcst_addr(), AUTH_KEY, NULL);

    /* Add Unicast WEP Key for BSS STA  */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        sta_entry_t *se = find_entry(mget_bssid());

        if(se != NULL)
        {
            delete_wep_entry_sta(se);
            machw_ce_add_key(key_id, TX_GTK_NUM, se->sta_index, mask, ct, key, mget_bssid(), AUTH_KEY, NULL);
        }
    }
}

/* Function is used to remove wep-key */
INLINE void remove_wep_key_sta(UWORD8* index)
{
    sta_entry_t *se = find_entry(mget_bssid());

    if(se != NULL)
    {
        delete_wep_entry_sta(se);
    }
}

#endif /* WEP_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
