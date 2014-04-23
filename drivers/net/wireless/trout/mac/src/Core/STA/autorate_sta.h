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
/*  File Name         : autorate_sta.h                                       */
/*                                                                           */
/*  Description       : This file contains all definitions related to the    */
/*                      implementation of autorate for STA mode.             */
/*                                                                           */
/*  List of Functions : ar_tx_success_sta                                    */
/*                      ar_tx_failure_sta                                    */
/*                      is_min_rate_sta                                      */
/*                      is_max_rate_sta                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef AUTORATE_STA_H
#define AUTORATE_STA_H

/*****************************************************************************/
/* File includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "autorate.h"
#include "index_util.h"
#include "management_sta.h"
#include "sta_prot_if.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
//chenq add a check 11b only if for WIFI & BT coexist 2013-09-19
typedef enum {B_ONLY_RATE_STA   = 1,
              G_ONLY_RATE_STA   = 2,
              N_ONLY_RATE_STA   = 4,
              BG_MIX_RATE_STA   = 3,
              GN_MIX_RATE_STA   = 6,
              BGN_MIX_RATE_STA  = 7,
              OTHER_RATE_STA
} LINK_MODE_T;

extern UWORD8 get_tx_rate_sta(sta_entry_t *se);
extern void do_sta_entry_ar(sta_entry_t *se);

#ifdef AUTORATE_FEATURE
extern void update_max_rate_idx_sta(sta_entry_t *se);
extern void update_min_rate_idx_sta(sta_entry_t *se);
extern void reinit_tx_rate_idx_sta(sta_entry_t *se);
extern void increment_rate_sta(sta_entry_t *se);

#ifdef AUTORATE_PING
/*ping.jiang add for AR algorithm 2013-10-31*/
extern void increment_rate_cca_sta(sta_entry_t *se);
extern void decrement_rate_cca_sta(sta_entry_t *se);
/*ping.jiang add forAR algorithm end*/
#endif /* AUTORATE_PING */

extern void decrement_rate_sta(sta_entry_t *se);
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function updates the auto rate statistics for the required entry in  */
/* the station table on completion of a successful transmission.             */
INLINE void ar_tx_success_sta(sta_entry_t *se, UWORD8 retry_count)
{
#ifdef AUTORATE_FEATURE
    if(se != 0)
    {
        ar_tx_success_update(&(se->ar_stats), retry_count);
        check_for_ar((void *)se, &(se->ar_stats), se->tx_rate_index);
    }
 #endif /* AUTORATE_FEATURE */
}

/* This function updates the auto rate statistics for the required entry in  */
/* the station table on completion of a failed transmission.                 */
INLINE void ar_tx_failure_sta(sta_entry_t *se)
{
#ifdef AUTORATE_FEATURE
    if(se != 0)
    {
        ar_tx_failure_update(&(se->ar_stats));
        check_for_ar((void *)se, &(se->ar_stats), se->tx_rate_index);
    }
 #endif /* AUTORATE_FEATURE */    
}

//chenq add a check 11b only if for WIFI & BT coexist 2013-09-19
INLINE LINK_MODE_T cur_rate_mode_sta(void)
{
	int ret          = 0;
    
#ifdef AUTORATE_FEATURE
	sta_entry_t *se  = 0;
	//UWORD8 rate_idx  = 0;
	UWORD8 rate      = 0;
	int i = 0;

	/* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());
	for(i = se->min_rate_index; i<= se->max_rate_index; i++)
	{
		if(i < get_ar_table_size())
		{
			/* Get the rate corresponding to this index */
	    	rate = get_ar_table_rate(i);

			if((is_rate_supp(rate, se) == 1) &&
               (is_rate_allowed(rate, se) == 1))
			{
				if( (rate == 0x04) ||/*  1M */
					(rate == 0x01) ||/*  2M */
					(rate == 0x02) ||/*5.5M */
					(rate == 0x03) ) /* 11M */
				{
					//find b mode
					ret |= BIT0;
				}
				else if(rate & BIT7)
				{
					//find n mode
					ret |= BIT2;
				}
				else
				{
					//find g mode
					ret |= BIT1;
				}
			}
		}
	}	

	if(ret == 0)
		ret = OTHER_RATE_STA;
#endif

    return (LINK_MODE_T)ret;
}

//leon liu add a check 11b only if for WIFI & BT coexist 2013-09-19
INLINE LINK_MODE_T cur_rate_mode_sta_with_entry(sta_entry_t *se)
{
	int ret          = 0;

#ifdef AUTORATE_FEATURE
	//UWORD8 rate_idx  = 0;
	UWORD8 rate      = 0;
	int i = 0;

	/* Get the station entry for the AP */
	for(i = se->min_rate_index; i<= se->max_rate_index; i++)
	{
		if(i < get_ar_table_size())
		{
			/* Get the rate corresponding to this index */
			rate = get_ar_table_rate(i);

			if((is_rate_supp(rate, se) == 1) &&
					(is_rate_allowed(rate, se) == 1))
			{
				if( (rate == 0x04) ||/*  1M */
						(rate == 0x01) ||/*  2M */
						(rate == 0x02) ||/*5.5M */
						(rate == 0x03) ) /* 11M */
				{
					//find b mode
					ret |= BIT0;
				}
				else if(rate & BIT7)
				{
					//find n mode
					ret |= BIT2;
				}
				else
				{
					//find g mode
					ret |= BIT1;
				}
			}
		}
	}

	if(ret == 0)
		ret = OTHER_RATE_STA;
#endif

	return (LINK_MODE_T)ret;
}

#ifdef AUTORATE_FEATURE
/* This function returns TRUE, if the current transmission rate is minimum   */
/* with respect to auto rate algorithm                                       */
INLINE BOOL_T is_min_rate_sta(sta_entry_t *se)
{
    if(se->tx_rate_index == se->min_rate_index)
        return BTRUE;
    else
        return BFALSE;
}

/* This function returns TRUE, if the current transmission rate is minimum   */
/* with respect to auto rate algorithm                                       */
INLINE BOOL_T is_max_rate_sta(sta_entry_t *se)
{
    if(se->tx_rate_index == se->max_rate_index)
        return BTRUE;
    else
        return BFALSE;
}
extern WORD32 get_asoc_avg_rssi(void);
/* This function initializes the transmit rate index to the Maximum Index in */
/* the autorate table.                                                       */
INLINE void init_tx_rate_idx_sta(sta_entry_t *se)
{
#ifdef AUTORATE_PING
    /*ping.jiang modify for AR algorithm 2013-12-12*/
    UWORD8 target_rate = 0;
    UWORD8 target_rate_index = 0;

    g_asoc_rssi = get_asoc_avg_rssi();
    target_rate = get_rate_from_rssi(se, g_asoc_rssi);
    target_rate_index =  get_ar_table_index(target_rate);
    se->tx_rate_index = target_rate_index;
    /* ping.jiang modify for AR algorithm end */
#else
    /* Set an initial rate index to approximately middle rate */
    se->tx_rate_index = ((se->max_rate_index + se->min_rate_index) >> 1);

    /* Decrement the rate so that, a valid rate index will be used */
    decrement_rate_sta(se);
#endif /* AUTORATE_PING */

    /* Update the TX MCS index */
    update_tx_mcs_index_ar_sta(se);

    /* Set SGI tx to BFALSE */
    set_tx_sgi_sta(BFALSE, se);
}

#endif /* AUTORATE_FEATURE */

#endif /* AUTORATE_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
