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
/*  File Name         : autorate.h                                           */
/*                                                                           */
/*  Description       : This file contains all definitions related to the    */
/*                      implementation of autorate.                          */
/*                                                                           */
/*  List of Functions : create_autorate_timer                                */
/*                      is_autorate_enabled                                  */
/*                      enable_autorate                                      */
/*                      disable_autorate                                     */
/*                      start_ar_timer                                       */
/*                      ar_tx_success_update                                 */
/*                      ar_tx_failure_update                                 */
/*                      set_ar_table                                         */
/*                      get_ar_table                                         */
/*                      get_ar_table_size                                    */
/*                      get_ar_table_rate                                    */
/*                      print_ar_log                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef AUTORATE_H
#define AUTORATE_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "itypes.h"
#include "common.h"
#include "phy_prot_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_AR_THRESHOLD 5
#define MIN_AR_THRESHOLD 1
#define AR_INTERVAL      1000 /* approx 1 second */

#ifdef AUTORATE_FEATURE
#ifdef DEBUG_MODE
#define MAX_AR_LOG_LIMIT 10 /* approx 10 second */
#endif /* DEBUG_MODE */
#define AR_INDEX_THRESHOLD_1  ((g_ar_table_size << 1) / 3)
#define AR_INDEX_THRESHOLD_2  (g_ar_table_size / 3)

#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

/* This returns TRUE if less than 10% => 20% of the packet transmissions failed     */
/* during the previous period, FALSE otherwise.                              */
//chen mode auto rate policy 2013-07-24
#define SUCCESS_TX(ar_stats) (ar_stats->ar_retcnt <= ar_stats->ar_pktcnt/5)
//#define SUCCESS_TX(ar_stats) (((ar_stats->ar_retcnt * 100) / ar_stats->ar_pktcnt) <= 20)

/* This returns TRUE if enough packets were transmitted during the previous  */
/* period to get significative statistics. By default, this returns TRUE if  */
/* the transmission of 10 distinct packets was attempted during the previous */
/* period, FALSE otherwise.                                                  */
#define ENOUGH_TX(ar_stats) (ar_stats->ar_pktcnt > 10)

/* This returns TRUE if more than 33% of the packet transmissions failed     */
/* during the previous period, FALSE otherwise.                              */
#define FAIL_TX(ar_stats) (ar_stats->ar_retcnt > ar_stats->ar_pktcnt/2)

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Auto rate statistics structure */
typedef struct
{
    UWORD16 ar_pktcnt;         /* Transmitted packet count, success/failure  */
    UWORD16 ar_retcnt;         /* Retry count                                */
    UWORD16 ar_success;        /* Success count                              */
    UWORD16 ar_recovery;       /* Recovery status                            */
    UWORD16 ar_success_thresh; /* Current success threshold                  */
} ar_stats_t;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {NO_RATE_CHANGE = 0,
              INCREMENT_RATE = 1,
              DECREMENT_RATE = 2
} AR_ACTION_T;

typedef enum {AUTORATE_TYPE_SPEED    = 0x00,
              AUTORATE_TYPE_DISTANCE = 0x01
} AUTORATE_TYPE_T;

#ifdef AUTORATE_FEATURE
typedef enum {MISC_SW_AR_CNTL = 0x59,
} EVENT_TYPESUBTYPE_AR_T;
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern UWORD8         g_autorate_type;
extern ALARM_HANDLE_T *g_ar_timer;
extern BOOL_T         g_ar_enable;
#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_cmcc_cfg_tx_rate;
#endif
#ifdef AUTORATE_FEATURE
extern UWORD8 g_ar_table[MAX_NUM_RATES];
extern UWORD8 g_ar_table_size;
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
extern void ar_timer_fn(void *h, UWORD32 data);
#else /* OS_LINUX_CSL_TYPE */
extern void ar_timer_fn(UWORD32 data);
#endif /* OS_LINUX_CSL_TYPE */

extern void   ar_stats_init(ar_stats_t *ar_stats);
extern UWORD8 ar_rate_ctl(ar_stats_t *ar_stats, UWORD8 is_max, UWORD8 is_min);
extern void   update_per_entry_retry_set_info(void);
extern void   update_entry_retry_rate_set(void *entry, UWORD8 rate);
extern void   update_retry_rate_set(UWORD8 ret_ar_en, UWORD8 rate, void *entry,
                                    UWORD32 *retry_set);

extern void update_retry_rate_set2(UWORD8 rate, void *entry, UWORD32 *retry_set);


#ifdef AUTORATE_FEATURE
extern void update_per_entry_ar_info(void);
extern void update_per_entry_rate_idx(void);
extern void check_for_ar(void* entry, ar_stats_t *ar_stats,
                         UWORD8 tx_rate_index);
#endif /* AUTORATE_FEATURE */


/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* Create the autorate timer */
INLINE void create_autorate_timer(void)
{
#ifdef AUTORATE_FEATURE
    /* If the Auto Rate alarm exists, stop and delete the same */
    if(g_ar_timer != 0)
    {
        stop_alarm(g_ar_timer);
        delete_alarm(&g_ar_timer);
    }
    g_ar_timer = create_alarm(ar_timer_fn, 0, NULL);    //Hugh
 #endif /* AUTORATE_FEATURE */
}

/* This function starts the auto rate timer for the specified interval       */
/* currently set as 10 seconds.                                              */
INLINE void start_ar_timer(void)
{
#ifdef AUTORATE_FEATURE
    if(g_ar_enable == BTRUE)
        start_alarm(g_ar_timer, AR_INTERVAL);

#endif /* AUTORATE_FEATURE */
}

/* This function stops the auto rate timer                                   */
INLINE void stop_ar_timer(void)
{
#ifdef AUTORATE_FEATURE
    stop_alarm(g_ar_timer);
#endif /* AUTORATE_FEATURE */
}

/* This function checks if auto rate is enabled */
INLINE BOOL_T is_autorate_enabled(void)
{
    return g_ar_enable;
}

/* This function enables auto rate feature. It sets the auto rate flag and   */
/* creates the auto rate alarm. The alarm will be started only after MAC is  */
/* in ENABLED state.                                                         */
INLINE void enable_autorate(void)
{
    stop_ar_timer();
    g_ar_enable = BTRUE;
    start_ar_timer();
}

/* This function disables auto rate feature. It resets the auto rate flag    */
/* and deletes the auto rate alarm.                                          */
INLINE void disable_autorate(void)
{
    stop_ar_timer();
    g_ar_enable = BFALSE;
}

/* This function updates the auto rate statistics on successful transmission */
INLINE void ar_tx_success_update(ar_stats_t *ar_stats, UWORD8 retry_count)
{
#ifdef IBSS_BSS_STATION_MODE
    //chenq add if indicate tx rate,do not update 2013-09-18
	if(g_cmcc_cfg_tx_rate !=0)
	    return;
#endif
    /* Update the packet count */
    ar_stats->ar_pktcnt++;

    /* Based on retry count update the retry count */
    if(retry_count > 0)
    {
        ar_stats->ar_retcnt++;
    }
}

/* This function updates the auto rate statistics on transmission failure */
INLINE void ar_tx_failure_update(ar_stats_t *ar_stats)
{
#ifdef IBSS_BSS_STATION_MODE
	//chenq add if indicate tx rate,do not update 2013-09-18
	if(g_cmcc_cfg_tx_rate !=0)
	    return;
#endif
    /* Update the packet count */
    ar_stats->ar_pktcnt++;

    /* Update the retry count and failure count */
    ar_stats->ar_retcnt++;
}

#ifdef AUTORATE_FEATURE
/* This function sets the global auto rate table with the given values */
INLINE void set_ar_table(UWORD8 *val)
{
    UWORD8 i   = 0;
    UWORD8 len = 0;

    /* Extract length (cannot be more than UWORD8) */
    len = val[0];

    /* The table size is limited by the maximum number of rates supported */
    g_ar_table_size = MIN(len, MAX_NUM_RATES);

    /* Update the autp rate table */
    for(i = 0; i < g_ar_table_size; i++)
        g_ar_table[i] = val[i + 2];

    /* Update relevant information for all station entries */
    update_per_entry_ar_info();
}

/* This function returns the global auto rate table size */
INLINE UWORD8 get_ar_table_size(void)
{
    return g_ar_table_size;
}

/* This function returns the MAC rate at index i in global auto rate table */
INLINE UWORD8 get_ar_table_rate(UWORD8 i)
{
    return g_ar_table[i];
}

/* This function returns the index i in global auto rate table that matches  */
/* the given MAC rate                                                        */
INLINE UWORD8 get_ar_table_index(UWORD8 rate)
{
    UWORD8 i = 0;

    for(i = 0; i < g_ar_table_size; i++)
    {
        if(g_ar_table[i] == rate)
            break;
    }

    return i;
}

/* This function checks if there are enough TX stats collected to run the AR  */
/* algorithm                                                                  */
INLINE BOOL_T enough_tx_stats(ar_stats_t *ar_stats, UWORD8 curr_tx_index)
{
    BOOL_T ret_val = BFALSE;

    if(curr_tx_index > AR_INDEX_THRESHOLD_1)
    {
        if(ar_stats->ar_pktcnt > 1000)
            ret_val = BTRUE;
    }
    else if (curr_tx_index > AR_INDEX_THRESHOLD_2)
    {
        if(ar_stats->ar_pktcnt > 100)
            ret_val = BTRUE;
    }
    else
    {
        if(ar_stats->ar_pktcnt > 10)
            ret_val = BTRUE;
    }

    return ret_val;
}

#endif /* AUTORATE_FEATURE */
#endif /* AUTORATE_H */
