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
/*  File Name         : autorate.c                                           */
/*                                                                           */
/*  Description       : This file contains the functions for auto rate.      */
/*                      The current algortihm supported for auto rate is the */
/*                      Adaptive Auto Rate Fallback (AARF) algorithm.        */
/*                      In ARF, each sender attempts to use a higher         */
/*                      transmission rate after a fixed number of successful */
/*                      transmissions at a given rate and switches back to a */
/*                      lower rate after 1 or 2 consecutive failures.        */
/*                      For more effective rate adaption the threshold is    */
/*                      continuously changed at runtime.This is done by      */
/*                      increasing the amount of history available, helping  */
/*                      in better decisions. In this algortihm, threshold    */
/*                      is adapted by using Binary Exponential Backoff.      */
/*                                                                           */
/*  List of Functions : ar_stats_init                                        */
/*                      ar_timer_fn                                          */
/*                      ar_rate_ctl                                          */
/*                      update_per_entry_ar_info                             */
/*                      update_per_entry_retry_set_info                      */
/*                      update_entry_retry_rate_set                          */
/*                      update_retry_rate_set                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "core_mode_if.h"
#include "phy_prot_if.h"
#include "mh.h"
#include "autorate.h"
#include "autorate_sta.h"
#include "receive.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8         g_autorate_type = AUTORATE_TYPE_SPEED;
ALARM_HANDLE_T *g_ar_timer     = 0;
#ifdef AUTORATE_FEATURE 
BOOL_T         g_ar_enable     = BTRUE; // caisf, default use auto rate // BFALSE;
#else
BOOL_T         g_ar_enable     = BFALSE;
#endif

#ifdef AUTORATE_PING
/* ping.jiang add for calculating statistics 2013-10-31 */
struct rx_stats g_rx_data = {0,0,0};
struct rx_stats g_cur_rx_data = {0,0,0};
struct rx_var_value g_rx_var_value = {0,0,0,0};
WORD32 g_asoc_rssi = 0;
unsigned int g_send_rts_flag = 0;
unsigned int g_ar_rr_flag = 0;
unsigned int g_ar_rr_index = 0;
unsigned int g_ar_stats_flag = 0;
unsigned int g_ar_stats_index = 0;
WORD32	g_ar_rssi_flag = 0;
WORD32	g_ar_rssi_index = 0;
WORD32  g_ar_total_rssi_value = 0;
WORD32  g_ar_total_rssi_num   = 0;
UWORD8  g_send_rts_cnt = 0;
WORD32 g_total_rssi_num_table[STATISTICS_FILTER_TABLE] = {0};
WORD32 g_total_rssi_value_table[STATISTICS_FILTER_TABLE] = {0};
UWORD16 g_retry_cnt_table[STATISTICS_FILTER_TABLE] = {0};
UWORD16 g_txpkt_cnt_table[STATISTICS_FILTER_TABLE] = {0};
UWORD16 g_rx_complete_cnt_table[STATISTICS_FILTER_TABLE] = {0};
unsigned int g_rx_nack_all_cnt_table[STATISTICS_FILTER_TABLE] = {0};
/* ping.jiang add for calculating statistics end */
#endif /* AUTORATE_PING */

#ifdef AUTORATE_PING
/* ping.jiang add for AR algorithm 2013-11-10 */
UWORD8 g_802_11b_rate_table[4] =
{
     	0x04,   /*   1 Mbps    */
     	0x01,   /*   2 Mbps    */
     	0x02,   /*   5.5 Mbps */
      	0x03,   /*  11 Mbps   */
};
UWORD8 g_802_11g_rate_table[8] =
{
       0x0B, 	/*   6 Mbps      */
       0x0F, 	/*   9 Mbps      */
      0x0A,  	/*  12 Mbps      */
      0x0E,  	/*  18 Mbps      */
      0x09,  	/*  24 Mbps      */
      0x0D,  	/*  36 Mbps      */
      0x08,  	/*  48 Mbps      */
      0x0C,  	/*  54 Mbps      */
};
UWORD8 g_802_11n_rate_table[8] =
{
	0x80,	     /* MCS-0  :   6.5 Mbps */
      0x81,	     /* MCS-1  :  13.0 Mbps */
      0x82,	     /* MCS-2  :  19.5 Mbps */
      0x83,	     /* MCS-3  :  26.0 Mbps */
      0x84,	     /* MCS-4  :  39.0 Mbps */
      0x85,	     /* MCS-5  :  52.0 Mbps */
      0x86,	     /* MCS-6  :  58.5 Mbps */
      0x87,	     /* MCS-7  :  65.0 Mbps */
};
/* ping.jiang add for AR algorithm end */
#endif /* AUTORATE_PING */

#ifdef AUTORATE_FEATURE
UWORD8 g_ar_table[MAX_NUM_RATES] = {0};
UWORD8 g_ar_allow[MAX_NUM_RATES] = {0};
UWORD8 g_ar_table_size = 0;
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ar_stats_init                                            */
/*                                                                           */
/*  Description   : This function initializes the given autorate statistics. */
/*                                                                           */
/*  Inputs        : 1) Pointer to the autorate statistics structure          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets all the statistics to initial value   */
/*                  of 0 and the success threshold to the minimum value.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ar_stats_init(ar_stats_t *ar_stats)
{
    ar_stats->ar_pktcnt   = 0;
    ar_stats->ar_retcnt   = 0;
    ar_stats->ar_success  = 0;
    ar_stats->ar_recovery = 0;
    ar_stats->ar_success_thresh = MIN_AR_THRESHOLD;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ar_timer_fn                                              */
/*                                                                           */
/*  Description   : This is the auto rate alarm function.                    */
/*                                                                           */
/*  Inputs        : Unused                                                   */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : This is the function called when the auto rate timer     */
/*                  ends. It is called every 1 seconds. In this function     */
/*                  the entire station table is traversed and auto rate is   */
/*                  performed on each non-zero entry. Also the auto rate     */
/*                  timer is restarted.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void ar_timer_fn(void *h, UWORD32 data)
#else /* OS_LINUX_CSL_TYPE */
void ar_timer_fn(UWORD32 data)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef AUTORATE_FEATURE
    UWORD8        i        = 0;
    void          *te      = 0;
    table_elmnt_t *tbl_elm = 0;

    /* Traverse entire association table and process all non-zero entries */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        misc_event_msg_t *misc       = 0;

        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = tbl_elm->element;

            if(te == 0)
                break;

#ifdef WAKE_LOW_POWER_POLICY
		if(g_wifi_power_mode)
			return;
#endif

            /* Allocate buffer for the miscellaneous event */
            misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

            if(misc == NULL)
            {
                /* Debug Counter TBD */
                return;
            }

            misc->name = MISC_SW_AR_CNTL;
            misc->info = 0;
            misc->data = (void *)te;

            post_event((UWORD8*)misc, MISC_EVENT_QID);

            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */

    /* Restart the autorate timer */
    start_ar_timer();
#endif /* AUTORATE_FEATURE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_for_ar                                             */
/*                                                                           */
/*  Description   : This function checks if there are enough stats collected */
/*                  for a the given entry and performance autorating for     */
/*                  this entry                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the given entry                            */
/*                  2) Pointer to the autorate statistics structure          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if there are enough stats collected */
/*                  for a the given entry to run AR algorithm. This is done  */
/*                  till max index of AR table is reached.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void check_for_ar(void* entry, ar_stats_t *ar_stats, UWORD8 tx_rate_index)
{
#ifdef AUTORATE_FEATURE
    /* Do speedy stepping till the maximum rate index is reached */
    //chenq mod auto rate policy 2013-07-24
	#if 0
	if((is_max_rate(entry) == BFALSE) &&
       (enough_tx_stats(ar_stats, tx_rate_index) == BTRUE))
    #else
	if(enough_tx_stats(ar_stats, tx_rate_index) == BTRUE)
	#endif
    {
        do_per_entry_ar(entry);
    }
#endif /* AUTORATE_FEATURE */
}

#ifdef AUTORATE_PING
/*ping.jiang add for calculating statistics 2013-10-31*/
unsigned int get_rx_complete_cnt_sum(UWORD8 stats_index)
{
	unsigned int total_value = 0;
	UWORD8 index = 0;
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_rx_complete_cnt_table[index];
	}
	return total_value;
}

unsigned int get_rx_nack_all_cnt_sum(UWORD8 stats_index)
{
	unsigned int total_value = 0;
	UWORD8 index = 0;
	
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_rx_nack_all_cnt_table[index];
	}

	return total_value;
}

unsigned int get_avg_cca_freq(void)
{
	unsigned int avg_value = 0;
	
	if(0 == g_ar_stats_flag)	
	{
		if(g_ar_stats_index <= (STATISTICS_FILTER_TABLE - 1))
		{
			avg_value = 100 - 100 * (get_rx_complete_cnt_sum(g_ar_stats_index)) / get_rx_nack_all_cnt_sum(g_ar_stats_index);
		}
	}
	else 
	{
		avg_value = 100 - 100 * (get_rx_complete_cnt_sum(STATISTICS_FILTER_TABLE - 1)) / get_rx_nack_all_cnt_sum(STATISTICS_FILTER_TABLE - 1);
	}
	return avg_value;
}

UWORD16 get_retry_cnt_sum(UWORD8 stats_index)
{
	unsigned int total_value = 0;
	UWORD8 index = 0;
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_retry_cnt_table[index];
	}
	return total_value;
}

UWORD16 get_txpkt_cnt_sum(UWORD8 stats_index)
{
	unsigned int total_value = 0;
	UWORD8 index = 0;
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_txpkt_cnt_table[index];
	}
	return total_value;
}

UWORD16 get_avg_retry_ratio(void)
{
	UWORD16 avg_value = 0;
	if(0 == g_ar_rr_flag)	
	{
		if(g_ar_rr_index <= (STATISTICS_FILTER_TABLE - 1))
		{
			avg_value = 100 * (get_retry_cnt_sum(g_ar_rr_index)) / get_txpkt_cnt_sum(g_ar_rr_index);
		}			
	}
	else 
	{
		avg_value = 100 * (get_retry_cnt_sum(STATISTICS_FILTER_TABLE - 1)) / get_txpkt_cnt_sum(STATISTICS_FILTER_TABLE - 1);
	}
	return avg_value;
}

#ifdef IBSS_BSS_STATION_MODE
void ar_rssi_value_add(void)
{
	WORD32 temp_rssi = 0;
	WORD32 mask_rssi = 0;
	
    	temp_rssi = get_rssi() - 0xFF;
		
    	if(temp_rssi <= -128)
    	{	
		mask_rssi = temp_rssi + 0xFF;
		printk("change rssi value %d => %d\n",temp_rssi,mask_rssi);
    	}
    	else
    	{
       		mask_rssi = temp_rssi;
    	}
	g_ar_total_rssi_value += mask_rssi;
	g_ar_total_rssi_num ++;
	
	return;
}

WORD32 get_ar_avg_rssi(void)
{
	WORD32 cur_avg_rssi = 0;
	if(0 != g_ar_total_rssi_num)
	{
		cur_avg_rssi = g_ar_total_rssi_value / g_ar_total_rssi_num;	
		g_total_rssi_num_table[g_ar_rssi_index] = g_ar_total_rssi_num;
		g_total_rssi_value_table[g_ar_rssi_index] = g_ar_total_rssi_value;
	
		g_ar_total_rssi_value = 0;
		g_ar_total_rssi_num   = 0;
	}
	
	return cur_avg_rssi;
}

WORD32 get_ar_total_rssi_value_sum(UWORD8 stats_index)
{
	WORD32 total_value = 0;
	UWORD8 index = 0;
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_total_rssi_value_table[index];
	}
	return total_value;
}

WORD32 get_ar_total_rssi_num_sum(UWORD8 stats_index)
{
	WORD32 total_value = 0;
	UWORD8 index = 0;
	for(index = 0; index <= stats_index; index++)
	{
		total_value += g_total_rssi_num_table[index];
	}
	return total_value;
}

WORD32 get_filter_avg_rssi(void)
{
	WORD32 avg_value = 0;
	if(0 == g_ar_rssi_flag)	
	{
		if(g_ar_rssi_index >= 0 && g_ar_rssi_index <= (STATISTICS_FILTER_TABLE - 1))
		{
			avg_value = get_ar_total_rssi_value_sum(g_ar_rssi_index) / get_ar_total_rssi_num_sum(g_ar_rssi_index);
		}
	}
	else 
	{
		avg_value = get_ar_total_rssi_value_sum(STATISTICS_FILTER_TABLE - 1) / get_ar_total_rssi_num_sum(STATISTICS_FILTER_TABLE - 1);
	}
	return avg_value;
}
#endif /* IBSS_BSS_STATION_MODE */

void get_rx_statistics(ar_stats_t *ar_stats)
{ 	
	WORD32 rssi_cur = 0;
	UWORD16  txpkt_cnt = 0, retry_cnt = 0;
	UWORD16 tx_complete_cnt_cur = 0, rx_complete_cnt_cur = 0;
	UWORD16 cal_rx_complete_cnt = 0, cal_tx_complete_cnt = 0;
	unsigned int rx_end_cnt_cur = 0, rx_end_error_cnt_cur = 0;
	unsigned int cal_rx_end_cnt = 0, cal_rx_end_error_cnt = 0; 
	unsigned int rx_nack_all_cnt = 0;
	
	/*calculating per and cca_freq*/
	/* Get the value from relevant register */
	rx_complete_cnt_cur = (convert_to_le(host_read_trout_reg((unsigned int)rMAC_TX_RX_COMPLETE_CNT) >> 16) & 0xFFFF);
	tx_complete_cnt_cur = (convert_to_le(host_read_trout_reg((unsigned int)rMAC_TX_RX_COMPLETE_CNT) & 0xFFFF));
	rx_end_cnt_cur = convert_to_le(host_read_trout_reg((unsigned int)rMAC_RX_END_COUNT));
	rx_end_error_cnt_cur = convert_to_le(host_read_trout_reg((unsigned int)rMAC_RX_ERROR_END_COUNT));

	/* Get the current value */
	if(rx_complete_cnt_cur < g_rx_var_value.rx_complete_cnt)
	{
		cal_rx_complete_cnt = rx_complete_cnt_cur + 0xFFFF - g_rx_var_value.rx_complete_cnt;
	}
	else
	{
		cal_rx_complete_cnt = rx_complete_cnt_cur - g_rx_var_value.rx_complete_cnt;
	}

	if(tx_complete_cnt_cur < g_rx_var_value.tx_complete_cnt)
	{
		cal_tx_complete_cnt = tx_complete_cnt_cur +0xFFFF - g_rx_var_value.tx_complete_cnt;
	}
	else
	{
		cal_tx_complete_cnt = tx_complete_cnt_cur - g_rx_var_value.tx_complete_cnt;
	}

	if(rx_end_cnt_cur < g_rx_var_value.rx_end_cnt)
	{
		cal_rx_end_cnt = rx_end_cnt_cur + 0xFFFFFFFF - g_rx_var_value.rx_end_cnt;
	}
	else
	{
		cal_rx_end_cnt = rx_end_cnt_cur - g_rx_var_value.rx_end_cnt;
	}

	if(rx_end_error_cnt_cur < g_rx_var_value.rx_end_error_cnt)
	{
		cal_rx_end_error_cnt = rx_end_error_cnt_cur + 0xFFFFFFFF - g_rx_var_value.rx_end_error_cnt;
	}
	else
	{
		cal_rx_end_error_cnt = rx_end_error_cnt_cur - g_rx_var_value.rx_end_error_cnt;
	}
    //printk("ori:cal_rx_complete_cnt = %u\n", cal_rx_complete_cnt);

    retry_cnt = ar_stats->ar_retcnt;
	txpkt_cnt = ar_stats->ar_pktcnt;

	if(g_send_rts_flag == 1)
	{
		if(txpkt_cnt > retry_cnt)
		{
			cal_rx_complete_cnt = cal_rx_complete_cnt + (cal_tx_complete_cnt * txpkt_cnt /(txpkt_cnt - retry_cnt));
		}
		else
		{
			cal_rx_complete_cnt = cal_rx_complete_cnt + cal_tx_complete_cnt;
		}
	}
	
	rx_nack_all_cnt = cal_rx_end_cnt + cal_rx_end_error_cnt - cal_tx_complete_cnt;
/*
	printk("cal_rx_complete_cnt = %u\n", cal_rx_complete_cnt);	
	printk("cal_tx_complete_cnt = %u\n", cal_tx_complete_cnt);
	printk("cal_rx_end_cnt = %u\n", cal_rx_end_cnt);
	printk("cal_rx_end_error_cnt = %u\n", cal_rx_end_error_cnt); 
*/
    printk("rx_nack_all_cnt = %u\n", rx_nack_all_cnt);

	if((0 != rx_nack_all_cnt) && (rx_nack_all_cnt >=  RXPKT_COUNT_THRESHOLD))
	{			 
	     if(cal_rx_complete_cnt > rx_nack_all_cnt)
	     {
		  g_cur_rx_data.cca_freq = 0;
		  g_rx_complete_cnt_table[g_ar_stats_index] = cal_rx_complete_cnt;
		  g_rx_nack_all_cnt_table[g_ar_stats_index] = rx_nack_all_cnt;
		  g_rx_data.cca_freq = 0;
	     }
	     else
	     {
		  g_cur_rx_data.cca_freq = 100 - (cal_rx_complete_cnt * 100 / rx_nack_all_cnt);
		  g_rx_complete_cnt_table[g_ar_stats_index] = cal_rx_complete_cnt;
		  g_rx_nack_all_cnt_table[g_ar_stats_index] = rx_nack_all_cnt;
		  g_rx_data.cca_freq = get_avg_cca_freq();
	     }
	}
	else
	{
		g_cur_rx_data.cca_freq = 0;
		g_rx_complete_cnt_table[g_ar_stats_index] = RXPKT_COUNT_THRESHOLD;
		g_rx_nack_all_cnt_table[g_ar_stats_index] = RXPKT_COUNT_THRESHOLD;
		g_rx_data.cca_freq = get_avg_cca_freq();
	}	
	//printk("g_cur_rx_data.cca_freq = %u\n",g_cur_rx_data.cca_freq);
	g_ar_stats_index ++;
    if(g_ar_stats_index >= STATISTICS_FILTER_TABLE)
    {
        g_ar_stats_index = 0;
        g_ar_stats_flag = 1;
    }

	/* Reset the value of struct g_rx_var_value */
	g_rx_var_value.rx_complete_cnt = rx_complete_cnt_cur;
	g_rx_var_value.tx_complete_cnt = tx_complete_cnt_cur;
	g_rx_var_value.rx_end_cnt = rx_end_cnt_cur;
	g_rx_var_value.rx_end_error_cnt = rx_end_error_cnt_cur;

	/*calculating RR*/
	if((0 != txpkt_cnt) && (txpkt_cnt >= TXPKT_COUNT_THRESHOLD))
	{
        g_retry_cnt_table[g_ar_rr_index] = retry_cnt;
        g_txpkt_cnt_table[g_ar_rr_index] = txpkt_cnt;
        g_rx_data.retry_ratio = get_avg_retry_ratio();
    }       
	else
	{
        g_retry_cnt_table[g_ar_rr_index] = retry_cnt;
        g_txpkt_cnt_table[g_ar_rr_index] = TXPKT_COUNT_THRESHOLD;
        g_rx_data.retry_ratio = get_avg_retry_ratio();
    }

	g_ar_rr_index ++;
    if(g_ar_rr_index >= STATISTICS_FILTER_TABLE)
    {
	    g_ar_rr_index = 0;
        g_ar_rr_flag = 1;
    }
	printk("g_rx_data.cca_freq=%u\n", g_rx_data.cca_freq);
	printk("g_rx_data.retry_ratio=%u\n", g_rx_data.retry_ratio);
       
#ifdef IBSS_BSS_STATION_MODE
	/*calculating rx rssi*/
	rssi_cur = get_ar_avg_rssi();
	g_rx_data.rssi = get_filter_avg_rssi();
	g_ar_rssi_index ++;
	if(g_ar_rssi_index >= STATISTICS_FILTER_TABLE)
	{
		g_ar_rssi_index = 0;
		g_ar_rssi_flag = 1;
	}
    	printk("g_rx_data.rssi=%d\n", g_rx_data.rssi);
#endif /* IBSS_BSS_STATION_MODE */


}

UWORD8 get_relevant_index(UWORD8 cur_index, UWORD8 sub_index)
{
	return ((cur_index >= sub_index) ? sub_index : cur_index);	 	
}


#ifdef IBSS_BSS_STATION_MODE
extern BOOL_T g_wifi_bt_coex;
UWORD8 get_rate_from_rssi(void *entry, WORD32 rx_rssi)
{
    UWORD8 rate = 0;
    LINK_MODE_T mode;
	sta_entry_t *se = entry;

    printk("g_wifi_bt_coex = %d\n",g_wifi_bt_coex);
    if(g_wifi_bt_coex)
    {
	    if(rx_rssi >= (SENSITIVITY_802_11G_54 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[7];
	    } 
	    else if(rx_rssi	>= (SENSITIVITY_802_11G_48 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[6];
	    }
	    else if(rx_rssi	>= (SENSITIVITY_802_11G_36+ SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[5];
	    }
	    else if(rx_rssi	>= (SENSITIVITY_802_11G_24 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[4];
	    }
	    else if(rx_rssi	>= (SENSITIVITY_802_11G_18 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[3];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_12 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_11 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[3];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11G_9 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11g_rate_table[1];
		//}
		else if(rx_rssi  >= (SENSITIVITY_802_11G_6 + SENSITIVITY_DELTA))
		{
			rate = g_802_11g_rate_table[0];
		} 
		else if(rx_rssi >= (SENSITIVITY_802_11B_5 + SENSITIVITY_DELTA))
		{
			rate = g_802_11b_rate_table[2];
		}
		else if(rx_rssi >= (SENSITIVITY_802_11B_2 + SENSITIVITY_DELTA))
		{
			rate = g_802_11b_rate_table[1];
		}
		else
		{
			rate = g_802_11b_rate_table[0];
		}

	}
	else
	{
		mode = cur_rate_mode_sta_with_entry(se);
    if(mode == B_ONLY_RATE_STA)
    {
	    if(rx_rssi >= (SENSITIVITY_802_11B_11 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[3];
	    }  
	    else if(rx_rssi >= (SENSITIVITY_802_11B_5 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_2 + SENSITIVITY_DELTA))
	    {
		   rate = g_802_11b_rate_table[1];
	    }
	    else
	    {
		    rate = g_802_11b_rate_table[0];
	    }
    } 
    else if(mode == G_ONLY_RATE_STA || mode == BG_MIX_RATE_STA)
    {
	    if(rx_rssi  >= (SENSITIVITY_802_11G_54 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[7];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_48 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[6];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_36+ SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[5];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_24 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[4];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_18 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[3];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_12 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_11 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[3];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11G_9 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11g_rate_table[1];
	    //}
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_6 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[0];
	    } 
	    else if(rx_rssi >= (SENSITIVITY_802_11B_5 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_2 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[1];
	    }
	    else
	    {
		    rate = g_802_11b_rate_table[0];
	    }
    }
    else if(mode == N_ONLY_RATE_STA || mode == BGN_MIX_RATE_STA)
    {
	    if(rx_rssi  >= (SENSITIVITY_802_11N_65 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11n_rate_table[7];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11N_59 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11n_rate_table[6];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11N_52 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11n_rate_table[5];
	    //}
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_54 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[7];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_48 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[6];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11N_39 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11n_rate_table[4];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_36 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[5];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11N_26 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11n_rate_table[3];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_24 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[4];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11N_19 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11n_rate_table[2];
	    //}
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_18 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[3];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11N_13 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11n_rate_table[1];
	    //}
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_12 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_11 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[3];
	    }
	    //else if(rx_rssi  >= (SENSITIVITY_802_11G_9 + SENSITIVITY_DELTA))
	    //{
	    //	rate = g_802_11g_rate_table[1];
	    //}
	    else if(rx_rssi  >= (SENSITIVITY_802_11N_6 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11n_rate_table[0];
	    }
	    else if(rx_rssi  >= (SENSITIVITY_802_11G_6 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11g_rate_table[0];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_5 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[2];
	    }
	    else if(rx_rssi >= (SENSITIVITY_802_11B_2 + SENSITIVITY_DELTA))
	    {
		    rate = g_802_11b_rate_table[1];
	    }
	    else
	    {
		    rate = g_802_11b_rate_table[0];
	    } 
    }
    }
    printk("select rate = %u\n",rate);
    return rate;
}

void ar_reset_prot(void)
{
    UWORD8 rate = 0;
	rate = get_cur_tx_rate();
	//printk("rate = %d\n",rate);
	if(!g_wifi_bt_coex)
	{
		if(rate >= 12)
		{
			if((g_cur_rx_data.cca_freq >= CCA_FREQ_THRESHOLD_6) || (0 != g_send_rts_cnt))
			{
				printk("Enter the branch of sending RTS\n");
			    g_send_rts_flag = 1;
			    host_write_trout_reg(0x10141, (UWORD32)rMAC_PROT_TX_MODE);
				host_write_trout_reg(0x3403, (UWORD32)rMAC_PROT_RATE);
				host_write_trout_reg(0x12, (UWORD32)rMAC_PROT_CON);
				if(0 == g_send_rts_cnt)
				{
					g_send_rts_cnt = MAX_SEND_RTS_CNT;
				}
			} 
			if((g_rx_data.cca_freq < CCA_FREQ_THRESHOLD_4) && (g_cur_rx_data.cca_freq < CCA_FREQ_THRESHOLD_4) && (0 == g_send_rts_cnt))
			{
				printk("Enter the branch no prot\n");
				g_send_rts_flag = 0;
				host_write_trout_reg(0x02, (UWORD32)rMAC_PROT_CON);
			}
			if(0 != g_send_rts_cnt)
			{
				g_send_rts_cnt--; 
			}
        	//printk("g_send_rts_flag = %d\n",g_send_rts_flag);
		    //printk("g_send_rts_cnt = %d\n",g_send_rts_cnt);
		}
	}
}

#endif /* IBSS_BSS_STATION_MODE */

void ar_reset_cw_cca_threshold(void)
{
    if(g_cur_rx_data.cca_freq >= CCA_FREQ_THRESHOLD_4)
    {
        set_machw_cw_be(10,4);
    }
    else if(g_cur_rx_data.cca_freq >= CCA_FREQ_THRESHOLD_2)
{
        set_machw_cw_be(9,2);
    }
    else if(g_cur_rx_data.cca_freq >= CCA_FREQ_THRESHOLD_1)
	{
		set_machw_cw_be(6,2);
	}
	else
	{
        set_machw_cw_be(2,2);
	}
}

/*ping.jiang add for calculating statistics end*/
#endif /* AUTORATE_PING */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ar_rate_ctl                                              */
/*                                                                           */
/*  Description   : This function performs adaptive rate updation for the    */
/*                  given entry statistics.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the autorate statistics structure          */
/*                  2) Flag indicating if current rate is maximum limit      */
/*                  2) Flag indicating if current rate is minimum limit      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This performs rate adaptation according to the auto rate */
/*                  algortihm.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : AR_ACTION_T, Action to be performed on transmit rate     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 ar_rate_ctl(ar_stats_t *ar_stats, UWORD8 is_max, UWORD8 is_min)
{
    UWORD8 status = NO_RATE_CHANGE;

#ifdef AUTORATE_PING
    /* ping.jiang add for AR algorithm 2013-11-10 */

    get_rx_statistics(ar_stats);

    ar_reset_cw_cca_threshold();
   
#ifdef IBSS_BSS_STATION_MODE
   	ar_reset_prot();
#endif

	if(g_rx_data.cca_freq >= CCA_FREQ_THRESHOLD_1)
    {
        if(g_rx_data.retry_ratio >= RETRY_RATIO_THRESHOLD_1)					
	 {
	      status = DECREMENT_RATE_CCA;
	 }
	 else
	 {
	      status = INCREMENT_RATE_CCA;						
	 }
     }
    else
    {
	if(g_rx_data.retry_ratio >= RETRY_RATIO_THRESHOLD_2)	
	{             
		status = DECREMENT_RATE;			
	}
	else
	{
		status = INCREMENT_RATE;
	}
    }

    /* ping.jiang add for AR algorithm end */
#endif /* AUTORATE_PING */

	if(ENOUGH_TX(ar_stats))
	{
	    if(SUCCESS_TX(ar_stats)) //&& ENOUGH_TX(ar_stats))
	    {
	    	#if 1 //chenq mode auto rate policy 2013-07-24
/* ping.jiang modify for AR algorithm 2013-11-10 */
#ifndef AUTORATE_PING
			status = INCREMENT_RATE;
#endif /* AUTORATE_PING */
			#else	
	        /* Increment the success count */
	        ar_stats->ar_success++;

	        if(ar_stats->ar_recovery == 1)
	            ar_stats->ar_success_thresh = MIN_AR_THRESHOLD;

	        /* Increase the rate if the threshold is crossed and maximum */
	        /* rate is not reached.                                      */
	        if((ar_stats->ar_success >= ar_stats->ar_success_thresh)
	            && (is_max == 0))
	        {
	            ar_stats->ar_recovery = 1;
	            ar_stats->ar_success  = 0;
	            status = INCREMENT_RATE;
	        }
	        else
	        {
	            ar_stats->ar_recovery = 0;
	        }
			#endif
	    }
	}
	else if(FAIL_TX(ar_stats))
	{
		/* Change the success threshold and decrease the transmit    */
		/* rate only if the current transmit rate is not the minimum */
		/* Otherwise do not change the threshold or rate.            */
		/*junbin.wang modify 20131121.start slow down data rate if packet count is 10*/
		//if(is_min == BFALSE && ENOUGH_TX(ar_stats))
		//	if(is_min == BFALSE && (ar_stats->ar_pktcnt > 4))//ENOUGH_TX(ar_stats))
		//modify zenghaiqi to fix bug 816
		if(is_min == BFALSE && ENOUGH_TX(ar_stats))
		{
			#if 0 //chenq mode auto rate policy 2013-07-24
		    if(ar_stats->ar_recovery)
		    {
		        /* Recovery failure - Binary Exponential increase of */
		        /* the success threshold is done.                    */
		        /*  increase till max is reached.  */
		        ar_stats->ar_success_thresh *= 2;
		        ar_stats->ar_success_thresh = (ar_stats->ar_success_thresh <
		                                      MAX_AR_THRESHOLD)?
		                                      ar_stats->ar_success_thresh:
		                                      MAX_AR_THRESHOLD;
		    }
		    else
		    {
		        /* Simple failure - Reset success threshold to min */
		        ar_stats->ar_success_thresh = MIN_AR_THRESHOLD;
		    }
			#endif

/* ping.jiang modify for AR algorithm 2013-11-10 */
#ifndef AUTORATE_PING
		    /* Decrease the rate */
		    status = DECREMENT_RATE;
#endif /* AUTORATE_PING */
		}

		/* Reset the success and recovery count */
		ar_stats->ar_success = 0;
		ar_stats->ar_recovery = 0;
	}

    /* If enough statistics were collected or the rate has been      */
    /* changed, then reset the counters                              */
    if(ENOUGH_TX(ar_stats) || (status != NO_RATE_CHANGE))
    {
        ar_stats->ar_pktcnt = 0;
        ar_stats->ar_retcnt  = 0;
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_per_entry_retry_set_info                          */
/*                                                                           */
/*  Description   : This function updates all the entries in the station or  */
/*                  association table with retry rate set for the current Tx */
/*                  rate or MCS.                                             */
/*                                                                           */
/*  Inputs        : 1) Rate in PHY format                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the station/association table and   */
/*                  updates the retry rate set table for each existing entry */
/*                  Note that this function needs to be called whenever      */
/*                  there is a change in the current Tx rate or MCS.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_per_entry_retry_set_info(void)
{
    UWORD8        i        = 0;
    UWORD8        rate     = 0;
    void          *te      = 0;
    table_elmnt_t *tbl_elm = 0;

    /* Traverse entire station/association table and process all non-zero    */
    /* entries                                                               */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = tbl_elm->element;

            if(te == 0)
                break;

            rate = get_tx_rate_to_sta(te);
            update_entry_retry_rate_set(te, get_phy_rate(rate));

            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_entry_retry_rate_set                              */
/*                                                                           */
/*  Description   : This function updates the station or association entry   */
/*                  with the retry rate set for the current Tx rate.         */
/*                                                                           */
/*  Inputs        : 1) Retransmission auto rate enable flag                  */
/*                  2) Transmission rate                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the retry rate set with the next 3 */
/*                  lower rates from the g_ar_table that are supported by    */
/*                  the station. In case auto rate is disabled the updation  */
/*                  is done only for HT rates for HT capable STA and Non-HT  */
/*                  rates for Non-HT STAs                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_entry_retry_rate_set(void *entry, UWORD8 rate)
{
    UWORD32 *retry_rate_set = 0;
    UWORD8  retry_rate[3]   = {0};

    /* If auto rate is not enabled the entry is updated only for HT rate for */
    /* HT station and Non-HT rate for Non-HT station.                        */
    if(is_autorate_enabled() == BFALSE)
    {
        /* If rate being set is a HT rate and the entry is not HT capable do */
        /* nothing since the HT rate will never be used for transmission to  */
        /* this station                                                      */
        if((IS_RATE_MCS(rate) == BTRUE) && (is_ht_capable(entry) == BFALSE))
            return;
#if 0 /* This check is disabled to allow mixing of Rates & MCS values */
        /* If rate being set is a Non-HT rate and the entry is HT capable do */
        /* nothing since the Non-HT rate will never be used for transmission */
        /* to this station                                                   */
        if((IS_RATE_MCS(rate) == BFALSE) && (is_ht_capable(entry) == BTRUE))
            return;
#endif /* 0 */
    }

    /* Get pointer to the retry rate set of the entry. This will be updated  */
    /* in the following code with appropriate retransmission rates.          */
    retry_rate_set = get_retry_rate_set(entry);

    /* Initialize all retry rates to the current transmit rate */
    retry_rate[0] = retry_rate[1] = retry_rate[2] = rate;

#ifdef AUTORATE_FEATURE
    /* If Auto rate feature is enabled update the retry rate set */
    {
        UWORD8 rate_idx = 0;
        BOOL_T  is_mcs   = BFALSE;
        UWORD8  min_rate = 0;

        is_mcs   = IS_RATE_MCS(rate);
        min_rate = rate;

        /* Get the index of the current transmit rate in the g_ar_table. If  */
        /* auto rate is enabled the transmit rate index in the entry can be  */
        /* directly used. Otherwise the g_ar_table needs to be searched for  */
        /* a matching entry.                                                 */
        if(is_autorate_enabled() == BTRUE)
        {
            rate_idx = get_tx_rate_index(entry);
        }
        else
        {
            rate_idx = get_ar_table_index(rate);
        }

        /* Update the retry rate set from the current transmit rate index */
        if(rate_idx == get_ar_table_size())
        {
            /* Exception case: If no entry is found in the g_ar_table that   */
            /* has rate matching with the current Tx rate, do nothing. Set   */
            /* all rates to the current transmit rate. Do nothing.           */
        }
        else
        {
            UWORD8 i = 0;

            /* Select the next lower 3 rates from g_ar_table (r0, r1, r2)    */
            /* that are supported by the station                             */
            for(i = 0; i < 3; i++)
            {
                while(rate_idx != 0)
                {
                    /* If the minimum supported rate index is reached update */
                    /* the retry rate i with this and break from the loop.   */
                    /* Note that since this is the min rate index, the rate  */
                    /* corresponding must be supported. Thus no explicit     */
                    /* check is done in this case and rate i is directly set */
                    if(rate_idx == get_min_rate_index(entry))
                    {
                        retry_rate[i] = min_rate;
                        break;
                    }

                    /* Decrement rate index */
                    rate_idx--;

                    /* Get rate from g_ar_table corresponding to the index */
                    rate = get_ar_table_rate(rate_idx);

                    /* Legacy and HT-Rates are not intermixed for H/w Autorating */
                    if(is_mcs == IS_RATE_MCS(rate))
                    {
                        /* If the rate is supported update the retry rate i with */
                        /* this and break from the loop.                         */
                        if((is_rate_supp(rate, entry) == 1) &&
                           (is_rate_allowed(rate, entry) == 1))
                        {
                            retry_rate[i] = rate;
                            min_rate      = rate;
                            break;
                        }
                    }
                }

                /* If no valid retry rate could be found for index i, then it */
                /* is set same as that of index i-1.                          */
                if(rate_idx == 0)
                    retry_rate[i] = min_rate;
            }
        }
    }
#endif /* AUTORATE_FEATURE */
    {
        UWORD8 i = 0;

        /* Update Preamble for retry rate set */
        for(i = 0; i < 3; i++)
        {
            UWORD8 dr = retry_rate[i];
            UWORD8 pr = get_preamble(dr);

            if(IS_RATE_11B(dr) == BTRUE)
                dr |= (BIT2 & (pr << 2));

            retry_rate[i] = dr;
        }
    }

    /* Update the entry rate set words with the retry rates 0, 1, 2 */
    retry_rate_set[0] = (retry_rate[1] << 24) | (retry_rate[1] << 16) |
                        (retry_rate[0] <<  8) | (retry_rate[0]);
    retry_rate_set[1] = (retry_rate[2] << 24) | (retry_rate[2] << 16) |
                        (retry_rate[2] <<  8) | (retry_rate[2]);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_retry_rate_set                                    */
/*                                                                           */
/*  Description   : This function updates the retry rate set with the values */
/*                  of the rates to be used for the retransmissions          */
/*                                                                           */
/*  Inputs        : 1) Retransmission auto rate enable flag                  */
/*                  2) Transmission rate                                     */
/*                  3) Station/Association entry                             */
/*                  4) Pointer to the retry rate set words                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the retry rate set words to the    */
/*                  retry rates saved in the entry if retransmission auto    */
/*                  rate is enabled. If not all retry rates are set to the   */
/*                  transmission rate.                                       */
/*                                                                           */
/*  Outputs       : Updates the given retry rate set words                   */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_retry_rate_set(UWORD8 ret_ar_en, UWORD8 rate, void *entry,
                           UWORD32 *retry_set)
{
    UWORD8 phy_rate = 0;
	UWORD8 pr;
/*
//0621, mask zhuyg code
#ifdef TROUT_WIFI_NPI
	return;
#endif
*/

#ifdef AUTORATE_FEATURE
    if(ret_ar_en == 1)
    {
        UWORD32 *retry_rate_set = get_retry_rate_set(entry);

        retry_set[0] = retry_rate_set[0];
        retry_set[1] = retry_rate_set[1];

        return;
    }
#endif /* AUTORATE_FEATURE */
	/*junbin.wang add for long preamble.20131118*/
    phy_rate = get_phy_rate(rate);
	pr = get_preamble(phy_rate);
	
	if(IS_RATE_11B(phy_rate) == BTRUE)
		phy_rate |= (BIT2 & (pr << 2));
    /* If AUTORATE_FEATURE is not defined or retransmission auto rate is not */
    /* enabled set all retry rates to the transmit rate                      */
    retry_set[0] = (phy_rate << 24) | (phy_rate << 16) | (phy_rate <<  8) | (phy_rate);
    retry_set[1] = retry_set[0];
}
#if 1
void update_retry_rate_set2(UWORD8 rate, void *entry, UWORD32 *retry_set)
{
	UWORD8 retry_rate[3] = {0,0,0};
	UWORD8 min_rate = 0;
	
	retry_rate[0] = rate;

	#ifdef AUTORATE_FEATURE
	{
			UWORD8 rate_idx = 0;
			BOOL_T	is_mcs	 = BFALSE;
			UWORD8	min_rate = 0;
	
			is_mcs	 = IS_RATE_MCS(rate);
			min_rate = rate;
	
			/* Get the index of the current transmit rate in the g_ar_table. If  */
			/* auto rate is enabled the transmit rate index in the entry can be  */
			/* directly used. Otherwise the g_ar_table needs to be searched for  */
			/* a matching entry.												 */
			if(is_autorate_enabled() == BTRUE)
			{
				rate_idx = get_tx_rate_index(entry);
			}
			else
			{
				rate_idx = get_ar_table_index(rate);
			}
	
			/* Update the retry rate set from the current transmit rate index */
			if(rate_idx == get_ar_table_size())
			{
				/* Exception case: If no entry is found in the g_ar_table that	 */
				/* has rate matching with the current Tx rate, do nothing. Set	 */
				/* all rates to the current transmit rate. Do nothing.			 */
			}
			else
			{
				UWORD8 i = 0;
	
				/* Select the next lower 3 rates from g_ar_table (r0, r1, r2)	 */
				/* that are supported by the station							 */
				for(i = 1; i < 3; i++)
				{
					while(rate_idx != 0)
					{
						/* If the minimum supported rate index is reached update */
						/* the retry rate i with this and break from the loop.	 */
						/* Note that since this is the min rate index, the rate  */
						/* corresponding must be supported. Thus no explicit	 */
						/* check is done in this case and rate i is directly set */
						if(rate_idx == get_min_rate_index(entry))
						{
							retry_rate[i] = min_rate;
							break;
						}
	
						/* Decrement rate index */
						rate_idx--;
	
						/* Get rate from g_ar_table corresponding to the index */
						rate = get_ar_table_rate(rate_idx);
	
						/* Legacy and HT-Rates are not intermixed for H/w Autorating */
						if(is_mcs == IS_RATE_MCS(rate))
						{
							/* If the rate is supported update the retry rate i with */
							/* this and break from the loop.						 */
							if((is_rate_supp(rate, entry) == 1) &&
							   (is_rate_allowed(rate, entry) == 1))
							{
								retry_rate[i] = rate;
								min_rate	  = rate;
								break;
							}
						}
					}
	
					/* If no valid retry rate could be found for index i, then it */
					/* is set same as that of index i-1.						  */
					if(rate_idx == 0)
						retry_rate[i] = min_rate;
				}
			}
		}
	#endif /* AUTORATE_FEATURE */
		printk("[%s] before %#x, %#x, %#x\n", __FUNCTION__, retry_rate[0], retry_rate[1], retry_rate[2]);
		{
			UWORD8 i = 0;
	
			/* Update Preamble for retry rate set */
			for(i = 0; i < 3; i++)
			{
				UWORD8 dr = retry_rate[i];
				UWORD8 pr = get_preamble(dr);
	
				if(IS_RATE_11B(dr) == BTRUE)
					dr |= (BIT2 & (pr << 2));

				retry_rate[i] = dr;
			}
		}
		printk("[%s] after %#x, %#x, %#x\n", __FUNCTION__, retry_rate[0], retry_rate[1], retry_rate[2]);



    /* Update the entry rate set words with the retry rates 0, 1, 2 */
    retry_set[0] = (retry_rate[1] << 24) | (retry_rate[1] << 16) |
                        (retry_rate[1] <<  8) | (retry_rate[0]);
    retry_set[1] = (0x04 << 16) |(retry_rate[2] <<  8) | (retry_rate[2]);
}
#endif



#ifdef AUTORATE_FEATURE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_per_entry_ar_info                                 */
/*                                                                           */
/*  Description   : This function updates all the entries in the station or  */
/*                  association table with current auto rate table info.     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the station/association table and   */
/*                  updates the maximum and minimum supported rate index     */
/*                  values for each existing entry. Note that this function  */
/*                  needs to be called whenever there is a change in the     */
/*                  auto rate table size.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_per_entry_ar_info(void)
{
    UWORD8        i        = 0;
    void          *te      = 0;
    table_elmnt_t *tbl_elm = 0;

    /* Traverse entire station/association table and process all non-zero    */
    /* entries                                                               */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = tbl_elm->element;

            if(te == 0)
                break;

            update_max_rate_idx(te);
            update_min_rate_idx(te);
            init_tx_rate_idx(te);
            reinit_tx_rate_idx(te);
            update_entry_retry_rate_set((void *)te,
                                        get_phy_rate(get_tx_rate_to_sta(te)));
            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_per_entry_rate_idx                                */
/*                                                                           */
/*  Description   : This function updates the rate index of all the entries  */
/*                  in the station. Note that this function needs to be      */
/*                  called whenever there is a change in the global          */
/*                  parameters affecting the current rate index e.g TXOP     */
/*                  Limit.                                                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the station/association table and   */
/*                  updates the current rate index to match the currently    */
/*                  enabled features (TXOP-Limit, Min-TXOP Frag-Limit etc).  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_per_entry_rate_idx(void)
{
    UWORD8        i        = 0;
    void          *te      = 0;
    table_elmnt_t *tbl_elm = 0;

    if(is_autorate_enabled() == BFALSE)
        return;

    /* Traverse entire station/association table and process all non-zero    */
    /* entries                                                               */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = tbl_elm->element;

            if(te == 0)
                break;

            reinit_tx_rate_idx(te);

            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */
}




#endif /* AUTORATE_FEATURE */
