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
#ifdef AUTORATE_PING
/* ping.jiang add for AR algorithm 2013-11-10 */
#define STATISTICS_FILTER_TABLE 	8
#define RXPKT_COUNT_THRESHOLD       10
#define TXPKT_COUNT_THRESHOLD       15 //bt&wifi coexist tx_pkt is less
#define CCA_FREQ_THRESHOLD_1	    20
#define CCA_FREQ_THRESHOLD_2        30
#define CCA_FREQ_THRESHOLD_3        40
#define CCA_FREQ_THRESHOLD_4        50
#define CCA_FREQ_THRESHOLD_5 		60
#define CCA_FREQ_THRESHOLD_6 		70
#define RETRY_RATIO_THRESHOLD_1 	80
#define RETRY_RATIO_THRESHOLD_2 	50
#define RETRY_RATIO_THRESHOLD_3     30 //add for reseting CW
#define MAX_SEND_RTS_CNT            8
#define SENSITIVITY_DELTA 			10

#define SENSITIVITY_802_11B_11 		-90
#define SENSITIVITY_802_11B_5   		-94
#define SENSITIVITY_802_11B_2   		-95
#define SENSITIVITY_802_11B_1   		-99

#define SENSITIVITY_802_11G_54 		-74
#define SENSITIVITY_802_11G_48 		-75
#define SENSITIVITY_802_11G_36 		-80
#define SENSITIVITY_802_11G_24 		-83
#define SENSITIVITY_802_11G_18 		-86
#define SENSITIVITY_802_11G_12 		-88
#define SENSITIVITY_802_11G_9   		-90
#define SENSITIVITY_802_11G_6   		-92

#define SENSITIVITY_802_11N_65 		-71
#define SENSITIVITY_802_11N_59 		-73
#define SENSITIVITY_802_11N_52 		-74
#define SENSITIVITY_802_11N_39 		-79
#define SENSITIVITY_802_11N_26 		-82
#define SENSITIVITY_802_11N_19 		-86
#define SENSITIVITY_802_11N_13 		-88
#define SENSITIVITY_802_11N_6   		-91
/* ping.jiang add for AR algorithm end */
#endif /* AUTORATE_PING */

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

#ifdef AUTORATE_PING
/* ping.jiang add for calculating statistics 2013-10-31 */
struct rx_stats 
{
	WORD32 rssi;
	unsigned int cca_freq;
	UWORD16 retry_ratio;
};

struct rx_var_value 
{
	unsigned int rx_end_cnt;
	unsigned int rx_end_error_cnt;
	UWORD16 rx_complete_cnt;
	UWORD16 tx_complete_cnt;
};
/* ping.jiang add for calculating statistics end */
#endif /* AUTORATE_PING */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
#ifdef AUTORATE_PING
/* ping.jiang modify for AR algorithm 2013-10-31 */
typedef enum {NO_RATE_CHANGE = 0,
              INCREMENT_RATE = 1,
              DECREMENT_RATE = 2,
              INCREMENT_RATE_CCA = 3,
              DECREMENT_RATE_CCA = 4
} AR_ACTION_T;
/* ping.jiang modify for AR algorithm end */
#else
typedef enum {NO_RATE_CHANGE = 0,
              INCREMENT_RATE = 1,
              DECREMENT_RATE = 2
} AR_ACTION_T;
#endif /* AUTORATE_PING */

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
#ifdef AUTORATE_PING
/* ping.jiang add for calculating statistics 2013-10-31 */
extern struct rx_stats g_rx_data;
extern struct rx_stats g_cur_rx_data;
extern struct rx_var_value g_rx_var_value;
extern WORD32 g_asoc_rssi;
extern unsigned int g_send_rts_flag;
extern unsigned int g_ar_rr_flag;
extern unsigned int g_ar_rr_index;
extern unsigned int g_ar_stats_flag;
extern unsigned int g_ar_stats_index;
extern WORD32	g_ar_rssi_flag;
extern WORD32	g_ar_rssi_index;
extern WORD32  g_ar_total_rssi_value;
extern WORD32  g_ar_total_rssi_num;
extern UWORD8 g_send_rts_cnt;
extern WORD32 g_total_rssi_num_table[STATISTICS_FILTER_TABLE];
extern WORD32 g_total_rssi_value_table[STATISTICS_FILTER_TABLE];
extern UWORD16 g_retry_cnt_table[STATISTICS_FILTER_TABLE];
extern UWORD16 g_txpkt_cnt_table[STATISTICS_FILTER_TABLE];
extern UWORD16 g_rx_complete_cnt_table[STATISTICS_FILTER_TABLE];
extern unsigned int g_rx_nack_all_cnt_table[STATISTICS_FILTER_TABLE];
/* ping.jiang add for calculating statistics end */
#endif /* AUTORATE_PING */

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
#ifdef AUTORATE_PING
/* ping.jiang add for AR algorithm 2013-11-10 */
extern void get_rx_statistics(ar_stats_t *ar_stats);
extern unsigned int get_rx_complete_cnt_sum(UWORD8 stats_index);
extern unsigned int get_rx_nack_all_cnt_sum(UWORD8 stats_index);
extern unsigned int get_avg_cca_freq(void);
extern UWORD16 get_retry_cnt_sum(UWORD8 stats_index);
extern UWORD16 get_txpkt_cnt_sum(UWORD8 stats_index);
extern UWORD16 get_avg_retry_ratio(void);
extern UWORD8 get_relevant_index(UWORD8 cur_index, UWORD8 sub_index);
extern void ar_reset_cw_cca_threshold(void);
#ifdef IBSS_BSS_STATION_MODE
extern WORD32 get_filter_avg_rssi(void);
extern WORD32 get_ar_avg_rssi(void);
extern void ar_rssi_value_add(void);
extern WORD32 get_ar_total_rssi_value_sum(UWORD8 stats_index);
extern WORD32 get_ar_total_rssi_num_sum(UWORD8 stats_index);
extern UWORD8 get_rate_from_rssi(void *se, WORD32 rx_rssi);
extern void ar_reset_prot(void);
#endif /* IBSS_BSS_STATION_MODE */

/* ping.jiang add for AR algorithm end */
#endif /* AUTORATE_PING */

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
