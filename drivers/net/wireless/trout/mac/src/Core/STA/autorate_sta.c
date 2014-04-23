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
/*  File Name         : autorate_sta.c                                       */
/*                                                                           */
/*  Description       : This file contains all functions required for        */
/*                      autorate implementation in STA.                      */
/*                                                                           */
/*  List of Functions : get_tx_rate_sta                                      */
/*                      do_sta_entry_ar                                      */
/*                      increment_rate_sta                                   */
/*                      decrement_rate_sta                                   */
/*                      update_min_rate_idx_sta                              */
/*                      update_max_rate_idx_sta                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File includes                                                             */
/*****************************************************************************/

#include "autorate.h"
#include "autorate_sta.h"
#include "management_sta.h"
#include "core_mode_if.h"


/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_tx_rate_sta                                          */
/*                                                                           */
/*  Description   : This function returns the transmission rate to be        */
/*                  used for a particluar destination.                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if auto rate is enabled. If so it   */
/*                  gets the current transmit rate index from the entry and  */
/*                  gets the corresponding rate from the global auto rate    */
/*                  table. This rate is in PHY rate format. If it is a MCS   */
/*                  (i.e. Bit 7 is set) the same is returned. If not it is   */
/*                  converted to the user rate in Mbps and then returned.    */
/*                  In case auto rate is not enabled and the destination is  */
/*                  HT capable, the current Tx MCS is returned. If not the   */
/*                  current transmit rate is returned.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Rate in Mbps (bit 7 - 0) or as MCS (bit 6 - 0)   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_tx_rate_sta(sta_entry_t *se)
{
#ifdef AUTORATE_FEATURE
    if(is_autorate_enabled() == BTRUE)
    {
        UWORD8 curr_sta_rate_idx = 0;

        curr_sta_rate_idx = get_ar_table_rate(se->tx_rate_index);

        /* If the rate is an MCS return the same */
        if(IS_RATE_MCS(curr_sta_rate_idx) == BTRUE)
            return curr_sta_rate_idx;

        /* Return the rate in mbps as set for the associated STA. */
        return get_pr_to_ur(curr_sta_rate_idx);
    }
#endif /* AUTORATE_FEATURE */

#ifdef PHY_802_11n
    if(is_ht_capable(se) == BTRUE)
    {
        UWORD8 curr_tx_mcs = 0;

        curr_tx_mcs  = get_curr_tx_mcs();

        if(curr_tx_mcs != INVALID_MCS_VALUE)
        {
            curr_tx_mcs |= BIT7;
            return curr_tx_mcs;
        }
    }
#endif /* PHY_802_11n */

    return get_curr_tx_rate();
}

#ifdef AUTORATE_FEATURE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : do_sta_entry_ar                                          */
/*                                                                           */
/*  Description   : This function performs auto rate for the STA entry.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This performs autorate for the given station entry. It   */
/*                  updates the transmit rate of the entry based on the      */
/*                  output of the algorithm.                                 */
/*                                                                           */
/*  Outputs       : The current transmit rate index of the entry is modified */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void do_sta_entry_ar(sta_entry_t *se)
{
    UWORD8 status = 0;
    BOOL_T is_max = is_max_rate_sta(se);
    BOOL_T is_min = is_min_rate_sta(se);
    UWORD8 rate_index = se->tx_rate_index;

    /* Run the auto rate algorithm */
    status = ar_rate_ctl(&(se->ar_stats), is_max, is_min);

    /* If SGI TX is possible to the current entry and TX GI is LongGI and it  */
    /* is maximum index in the AR table and status is NO_RATE_CHANGE and also */
    /* the current Max rate index is a MCS rate then try to do SGI TX at      */
    /* highest rate in AR table                                               */
    if((is_sgi_possible_sta(se) == BTRUE) &&
       (get_tx_sgi_sta(se) == BFALSE) &&
       (is_max == BTRUE) &&
       (status == NO_RATE_CHANGE) &&
       (IS_RATE_MCS(get_ar_table_rate(rate_index))))
    {
        set_tx_sgi_sta(BTRUE, se);
        return;
    }

    /* Change the rate if required based on the algorithm output */
    if(status == INCREMENT_RATE)
    {
        increment_rate_sta(se);
    }
    else if(status == DECREMENT_RATE)
    {
        decrement_rate_sta(se);
	}
#ifdef AUTORATE_PING
    /*ping.jiang modify for AR algorithm 2013-2013-11-10*/
    else if(status == DECREMENT_RATE_CCA)
    {
        decrement_rate_cca_sta(se);
    }
    else if(status == INCREMENT_RATE_CCA)
    {
        increment_rate_cca_sta(se);
    }
    /*ping.jiang modify for AR algorithm end*/
#endif /* AUTORATE_PING */
}

#ifdef AUTORATE_PING
/*ping.jiang modify for AR algorithm 2013-2013-11-10*/
void increment_rate_sta(sta_entry_t *se)
{
	UWORD8 rate_idx = se->tx_rate_index;
	UWORD8 rate = 0;
	UWORD16 irr_th_0 = 20;
	UWORD16 irr_th_1 = 30;
	UWORD16 irr_th_2 = 40;
	UWORD16 irr_th_3 = 100;

	UWORD16 delta_irr = RETRY_RATIO_THRESHOLD_2 - g_rx_data.retry_ratio;
	UWORD8 target_rate = get_rate_from_rssi(se, g_rx_data.rssi);
	UWORD8 target_rate_index =  get_ar_table_index(target_rate);
	printk("[%s]Enter the function increment_rate_sta\n", __FUNCTION__);
	set_tx_sgi_sta(BFALSE, se);
	
	while(1)
	{
		//printk("in rate_idx:%u\n",rate_idx);
		if(rate_idx >= target_rate_index)
		{
			rate_idx = target_rate_index;
	        se->tx_rate_index = rate_idx;
		    update_tx_mcs_index_ar_sta(se);
			update_entry_retry_rate_set((void *)se, rate);
			break;
		}
		else
		{
			if(delta_irr <= irr_th_0)
			{
				rate_idx = rate_idx;
			}
			else if(delta_irr <= irr_th_1)
			{
				rate_idx = rate_idx + 1;
			}
			else if(delta_irr <= irr_th_2)
			{
				rate_idx = rate_idx + 2;
			}
			else 	if(delta_irr <= irr_th_3)
			{
				rate_idx = rate_idx + 4;
			}

			if(rate_idx >= target_rate_index)
			{
				rate_idx = target_rate_index;
			}
		}

		/* Get the rate corresponding to this index */
		rate = get_ar_table_rate(rate_idx);
		//printk("[%s] rate:%u\n", __FUNCTION__, rate);
		if((is_rate_supp(rate, se) == 1) && (is_rate_allowed(rate, se) == 1))
		{
	            /* If the rate is supported by the STA update the transmit rate  */
	            /* index and break.                                              */
	            se->tx_rate_index = rate_idx;

	            /* Update the TX MCS index */
	            update_tx_mcs_index_ar_sta(se);

	            /* Update the retry rate set for the new transmit rate */
	            update_entry_retry_rate_set((void *)se, rate);

	            break;
		}
	}
}

void decrement_rate_cca_sta(sta_entry_t *se)
{
    	UWORD8 rate_idx = se->tx_rate_index;
    	UWORD8 rate = 0;
       printk("[%s]Enter the function decrement_rate_cca_sta\n", __FUNCTION__);
	/* If the current TX rate is at SGI MCS make the TX at LongGI MCS */
   	 if((IS_RATE_MCS(get_ar_table_rate(se->tx_rate_index)) == BTRUE) &&
	 (get_tx_sgi_sta(se) == BTRUE))
    	{
        	set_tx_sgi_sta(BFALSE, se);
        	return;
    	}

    	set_tx_sgi_sta(BFALSE, se);
   	while(1)
    	{
    	
        	if(0 == rate_idx)
        	{
              		break;
        	}
		else
		{
              		rate_idx--;
		}
	
 		/* Get the rate corresponding to this index */
        	rate = get_ar_table_rate(rate_idx);
		//printk("[%s] rate:%u\n", __FUNCTION__, rate);

       		if((is_rate_supp(rate, se) == 1) && (is_rate_allowed(rate, se) == 1))
        	{
	            	/* If the rate is supported by the STA update the transmit rate  */
	            	/* index and break.                                              */
	            	se->tx_rate_index = rate_idx;

	            	/* Update the TX MCS index */
	            	update_tx_mcs_index_ar_sta(se);

	            	/* Update the retry rate set for the new transmit rate */
	            	update_entry_retry_rate_set((void *)se, rate);

	            	break;
       		}
	}

}

void increment_rate_cca_sta(sta_entry_t *se)
{
	UWORD8 rate_idx = se->tx_rate_index;
	UWORD8 rate = 0;
	unsigned int irc_th_0 = 10;
	unsigned int irc_th_1 = 30;
	unsigned int irc_th_2 = 50;
	unsigned int irc_th_3 = 100;

	unsigned int delta_irc = RETRY_RATIO_THRESHOLD_1 - g_rx_data.retry_ratio;
	UWORD8 target_rate = get_rate_from_rssi(se, g_rx_data.rssi);
	UWORD8 target_rate_index =  get_ar_table_index(target_rate);
       printk("[%s]Enter the function increment_rate_cca_sta\n", __FUNCTION__);
	set_tx_sgi_sta(BFALSE, se);

	while(1)
	{
		if(rate_idx >= target_rate_index)
		{
			rate_idx = target_rate_index;
		    se->tx_rate_index = rate_idx;
		    update_tx_mcs_index_ar_sta(se);
			update_entry_retry_rate_set((void *)se, rate);
			break;
		}
		else
		{
			if(delta_irc <= irc_th_0)
			{
				rate_idx = rate_idx;
			}
			else 	if(delta_irc <= irc_th_1)
			{
				rate_idx = rate_idx + 1;
			}
			else 	if(delta_irc <= irc_th_2)
			{
				rate_idx = rate_idx + 2;
			}
			else 	if(delta_irc <= irc_th_3)
			{
				rate_idx = rate_idx + 4;
			}
			if(rate_idx >= target_rate_index)
			{
				rate_idx = target_rate_index;
			}
		}
	       /* Get the rate corresponding to this index */
		rate = get_ar_table_rate(rate_idx);
		//printk("[%s] rate:%u\n", __FUNCTION__, rate);   
		if((is_rate_supp(rate, se) == 1) && (is_rate_allowed(rate, se) == 1))
		{
	            /* If the rate is supported by the STA update the transmit rate  */
	            /* index and break.                                              */
	            se->tx_rate_index = rate_idx;

	            /* Update the TX MCS index */
	            update_tx_mcs_index_ar_sta(se);

	            /* Update the retry rate set for the new transmit rate */
	            update_entry_retry_rate_set((void *)se, rate);

	            break;
		}
	}
}

void decrement_rate_sta(sta_entry_t *se)
{
   	UWORD8 rate_idx = se->tx_rate_index;
   	UWORD8 rate = 0;
	UWORD16 drr_th_0 = 0;
	UWORD16 drr_th_1 = 10;
	UWORD16 drr_th_2 = 30;
	UWORD16 drr_th_3 = 100;
	
	UWORD16 delta_drr = g_rx_data.retry_ratio - RETRY_RATIO_THRESHOLD_2;
	UWORD8 target_rate = get_rate_from_rssi(se, g_rx_data.rssi);
	UWORD8 target_rate_index =  get_ar_table_index(target_rate);
	if(rate_idx >= target_rate_index)
	{
		rate_idx = target_rate_index;
        }
        printk("[%s]Enter the function decrement_rate_sta\n", __FUNCTION__);
    	/* If the current TX rate is at SGI MCS make the TX at LongGI MCS */
   	 if((IS_RATE_MCS(get_ar_table_rate(se->tx_rate_index)) == BTRUE) &&
	 (get_tx_sgi_sta(se) == BTRUE))
    	{
        	set_tx_sgi_sta(BFALSE, se);
        	return;
    	}

    	set_tx_sgi_sta(BFALSE, se);

	while(1)
	{	
		if(0 == rate_idx)
		{
			break;
		}
		else
		{
			if(delta_drr <= drr_th_0)
			{
				rate_idx = rate_idx;
			}
			else if(delta_drr <= drr_th_1)
			{
				rate_idx = rate_idx - get_relevant_index(rate_idx, 1);
			}
			else if(delta_drr <= drr_th_2)
			{
				rate_idx = rate_idx - get_relevant_index(rate_idx, 2);
			}
			else if(delta_drr <= drr_th_3)
			{
				rate_idx = rate_idx - get_relevant_index(rate_idx, 4);
			}
			else
			{
			    	rate_idx --;
			}
			
		}	
 		/* Get the rate corresponding to this index */
        	rate = get_ar_table_rate(rate_idx);
		//printk("[%s] rate:%u\n", __FUNCTION__, rate);
       		if((is_rate_supp(rate, se) == 1) && (is_rate_allowed(rate, se) == 1))
        	{
	            	/* If the rate is supported by the STA update the transmit rate  */
	            	/* index and break.                                              */
	            	se->tx_rate_index = rate_idx;

	            	/* Update the TX MCS index */
	            	update_tx_mcs_index_ar_sta(se);

	            	/* Update the retry rate set for the new transmit rate */
	            	update_entry_retry_rate_set((void *)se, rate);

	            	break;
       		}
	}

}
/*ping.jiang modify for AR algorithm end*/
#else

/*****************************************************************************/
/*                                                                           */
/*  Function Name : increment_rate_sta                                       */
/*                                                                           */
/*  Description   : This function increments the current transmit rate for   */
/*                  a given entry.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches the global auto rate table for a  */
/*                  rate supported by the STA and having a higher index than */
/*                  the current transmit rate index of the STA. It then sets */
/*                  the current transmit rate index of the entry to this     */
/*                  index.                                                   */
/*                  Note that this function is called only if the current    */
/*                  transmit rate is not the maximum supported by the STA.   */
/*                  Thus a higher supported rate must be found in the table. */
/*                                                                           */
/*  Outputs       : The current transmit rate index of the entry is set to   */
/*                  the next higher value supported by the STA               */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void increment_rate_sta(sta_entry_t *se)
{
    UWORD8 rate_idx = se->tx_rate_index;
    UWORD8 rate     = 0;

    set_tx_sgi_sta(BFALSE, se);

    while(1)
    {
        rate_idx++;

        /* Exception - no higher supported rate is found in the global auto  */
        /* rate table. This condition should not occur since the auto rate   */
        /* algorithm checks that the current rate of the STA is not the      */
        /* maximum before calling this.                                      */
        if(rate_idx >= get_ar_table_size())
            break;

        /* Get the rate corresponding to this index */
        rate = get_ar_table_rate(rate_idx);

        if((is_rate_supp(rate, se) == 1) &&
           (is_rate_allowed(rate, se) == 1))
        {
            /* If the rate is supported by the STA update the transmit rate  */
            /* index and break.                                              */
            se->tx_rate_index = rate_idx;

            /* Update the TX MCS index */
            update_tx_mcs_index_ar_sta(se);

            /* Update the retry rate set for the new transmit rate */
            update_entry_retry_rate_set((void *)se, rate);

            break;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : decrement_rate_sta                                       */
/*                                                                           */
/*  Description   : This function decrements the current transmit rate for   */
/*                  a given entry.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches the global auto rate table for a  */
/*                  rate supported by the STA and having a lower index than  */
/*                  the current transmit rate index of the STA. It then sets */
/*                  the current transmit rate index of the entry to this     */
/*                  index.                                                   */
/*                  Note that this function is called only if the current    */
/*                  transmit rate is not the minimum supported by the STA.   */
/*                  Thus a lower supported rate must be found in the table.  */
/*                                                                           */
/*  Outputs       : The current transmit rate index of the entry is set to   */
/*                  the next lower value supported by the STA                */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void decrement_rate_sta(sta_entry_t *se)
{
    UWORD8 rate_idx = se->tx_rate_index;
    UWORD8 rate     = 0;

    /* If the current TX rate is at SGI MCS make the TX at LongGI MCS */
    if((IS_RATE_MCS(get_ar_table_rate(se->tx_rate_index)) == BTRUE) &&
       (get_tx_sgi_sta(se) == BTRUE))
    {
        set_tx_sgi_sta(BFALSE, se);
        return;
    }

    set_tx_sgi_sta(BFALSE, se);

    while(1)
    {
        /* Exception - no lower supported rate is found in the global auto   */
        /* rate table. This condition should not occur since the auto rate   */
        /* algorithm checks that the current rate of the STA is not the      */
        /* minimum before calling this.                                      */
        if(rate_idx == 0)
            break;

        rate_idx--;

        /* Get the rate corresponding to this index */
        rate = get_ar_table_rate(rate_idx);

        if((is_rate_supp(rate, se) == 1) &&
           (is_rate_allowed(rate, se) == 1))
        {
            /* If the rate is supported by the STA update the transmit rate  */
            /* index and break.                                              */
            se->tx_rate_index = rate_idx;

            /* Update the TX MCS index */
            update_tx_mcs_index_ar_sta(se);

            /* Update the retry rate set for the new transmit rate */
            update_entry_retry_rate_set((void *)se, rate);

            break;
        }
    }
}
#endif /* AUTORATE_PING */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_min_rate_idx_sta                                  */
/*                                                                           */
/*  Description   : This function updates the minimum rate index for the     */
/*                  given entry.                                             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches the global auto rate table for a  */
/*                  rate supported by the STA and having the lowest index.   */
/*                  It then sets the minimum transmit rate index of the      */
/*                  entry to this index.                                     */
/*                                                                           */
/*  Outputs       : The minimum transmit rate index of the entry is set      */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_min_rate_idx_sta(sta_entry_t *se)
{
    UWORD8 rate_idx = 0;

    while(1)
    {
        if(is_rate_supp(get_ar_table_rate(rate_idx), se) == 1)
        {
            /* If the rate is supported by the STA update the transmit rate  */
            /* index and break.                                              */
            se->min_rate_index = rate_idx;
            break;
        }

        rate_idx++;

        /* Exception - No rate is found in the table that is supported by    */
        /* the STA. This is not possible.                                    */
        if(rate_idx >= get_ar_table_size())
            break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_max_rate_idx_sta                                  */
/*                                                                           */
/*  Description   : This function updates the maximum rate index for the     */
/*                  given entry.                                             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches the global auto rate table for a  */
/*                  rate supported by the STA and having the highest index.  */
/*                  It then sets the maximum transmit rate index of the      */
/*                  entry to this index.                                     */
/*                                                                           */
/*  Outputs       : The maximum transmit rate index of the entry is set      */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_max_rate_idx_sta(sta_entry_t *se)
{
    UWORD8 rate_idx = get_ar_table_size();

    while(1)
    {
        rate_idx--;

        if(is_rate_supp(get_ar_table_rate(rate_idx), se) == 1)
        {
            /* If the rate is supported by the STA update the transmit rate  */
            /* index and break.                                              */
            se->max_rate_index = rate_idx;
            break;
        }

        /* Exception - No rate is found in the table that is supported by    */
        /* the STA. This is not possible.                                    */
        if(rate_idx == 0)
            break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reinit_tx_rate_idx_sta                                   */
/*                                                                           */
/*  Description   : This function reinitializes the rate index for the given */
/*                  entry to a allowed rate. This function should be called  */
/*                  when certain features which affect the transmission      */
/*                  characteristics of the MAC H/w are changed. For e.g when */
/*                  the TXOP-Limit is changed.                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the current rate is allowed. If  */
/*                  not, it first decrements the rate if possible or then    */
/*                  increments the rate to the next allowed rate.            */
/*                                                                           */
/*  Outputs       : The nearest allowed rate to the current rate.            */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reinit_tx_rate_idx_sta(sta_entry_t *se)
{
#ifdef AUTORATE_PING
    /*ping.jiang modify for AR algorithm 2013-12-12*/
    UWORD8 target_rate = get_rate_from_rssi(se, g_asoc_rssi);
    UWORD8 target_rate_index =  get_ar_table_index(target_rate);
    se->tx_rate_index = target_rate_index;
    /* ping.jiang modify for AR algorithm end */
#else
    UWORD8 curr_rate_index = se->tx_rate_index;

    /* If the current rate is allowed, then do nothing. */
    if(is_rate_allowed(get_ar_table_rate(curr_rate_index), se) == 1)
        return;

    /* If the current rate is not allowed, then step down to the next */
    /* lower allowed rate.                                            */
    decrement_rate_sta(se);

    if(curr_rate_index != se->tx_rate_index)
        return;

    /* If none of the lower rates are allowed, then step up to the */
    /* next higher allowed rate.                                   */
    increment_rate_sta(se);

#ifdef DEBUG_MODE
    if(curr_rate_index == se->tx_rate_index)
    {
        PRINTD2("SwEr: Tx-Rate Reinit Failed.\n");
        g_mac_stats.txrate_reinit_err++;
    }
#endif /* DEBUG_MODE */
#endif /* AUTORATE_PING */
}
#endif /* AUTORATE_FEATURE */

#endif /* IBSS_BSS_STATION_MODE */
