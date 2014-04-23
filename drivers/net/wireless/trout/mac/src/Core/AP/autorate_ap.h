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
/*  File Name         : autorate_ap.h                                        */
/*                                                                           */
/*  Description       : This file contains all definitions related to the    */
/*                      implementation of autorate for AP mode.              */
/*                                                                           */
/*  List of Functions : ar_tx_success_ap                                     */
/*                      ar_tx_failure_ap                                     */
/*                      is_min_rate_ap                                       */
/*                      is_max_rate_ap                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef AUTORATE_AP_H
#define AUTORATE_AP_H

/*****************************************************************************/
/* File includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "autorate.h"
#include "management_ap.h"
#include "index_util.h"
#include "ap_prot_if.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void do_asoc_entry_ar(asoc_entry_t *ae);
extern UWORD8 get_tx_rate_ap(asoc_entry_t *ae);

#ifdef AUTORATE_FEATURE
extern void update_max_rate_idx_ap(asoc_entry_t *ae);
extern void update_min_rate_idx_ap(asoc_entry_t *ae);
extern void reinit_tx_rate_idx_ap(asoc_entry_t *ae);
extern void increment_rate_ap(asoc_entry_t *ae);

#ifdef AUTORATE_PING
/*ping.jiang add for AR algorithm 2013-10-31*/
extern void decrement_rate_cca_ap(asoc_entry_t *ae);
extern void increment_rate_cca_ap(asoc_entry_t *ae);
/*ping.jiang add for AR algorithm end*/
#endif /* AUTORATE_PING */

extern void decrement_rate_ap(asoc_entry_t *ae);
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function updates the auto rate statistics for the required entry in  */
/* the association table on completion of a successful transmission.         */
INLINE void ar_tx_success_ap(asoc_entry_t *ae, UWORD8 retry_count)
{
#ifdef AUTORATE_FEATURE
    if(ae != 0)
    {
        ar_tx_success_update(&(ae->ar_stats), retry_count);
        check_for_ar((void *)ae, &(ae->ar_stats), ae->tx_rate_index);
    }
#endif /* AUTORATE_FEATURE */
}

/* This function updates the auto rate statistics for the required entry in  */
/* the association table on completion of a failed transmission.             */
INLINE void ar_tx_failure_ap(asoc_entry_t *ae)
{
#ifdef AUTORATE_FEATURE
    if(ae != 0)
    {
        ar_tx_failure_update(&(ae->ar_stats));
        check_for_ar((void *)ae, &(ae->ar_stats), ae->tx_rate_index);
    }
#endif /* AUTORATE_FEATURE */    
}

#ifdef AUTORATE_FEATURE
/* This function returns TRUE, if the current transmission rate is minimum   */
/* with respect to auto rate algorithm                                       */
INLINE BOOL_T is_min_rate_ap(asoc_entry_t *ae)
{
    BOOL_T retval = BFALSE;

    if(ae->tx_rate_index == ae->min_rate_index)
        retval = BTRUE;

    return retval;
}

/* This function returns TRUE, if the current transmission rate is minimum   */
/* with respect to auto rate algorithm                                       */
INLINE BOOL_T is_max_rate_ap(asoc_entry_t *ae)
{
    BOOL_T retval = BFALSE;

    if(ae->tx_rate_index == ae->max_rate_index)
        retval = BTRUE;

    return retval;
}

/* This function initializes the transmit rate index to the Maximum Index in */
/* the autorate table.                                                       */
INLINE void init_tx_rate_idx_ap(asoc_entry_t *ae)
{
    /* Set an initial rate index to approximately middle rate */
    ae->tx_rate_index = ((ae->max_rate_index + ae->min_rate_index) >> 1);

    /* Decrement the rate so that, a valid rate index will be used */
    decrement_rate_ap(ae);

    /* Update the TX MCS index */
    update_tx_mcs_index_ar_ap(ae);

    /* Set SGI tx to BFALSE */
    set_tx_sgi_ap(BFALSE, ae);
}

#endif /* AUTORATE_FEATURE */

#endif /* AUTORATE_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
