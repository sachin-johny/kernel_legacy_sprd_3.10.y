//leon liu created 2013-3-27
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include <linux/inetdevice.h>
#include "ps_timer.h"
#include "pm_sta.h"

/* leon liu opened PS_TIMER_DEBUG to remove powersave timer function 2013-4-28 */
#define PS_TIMER_DEBUG //danny deng open powersave functon 

#ifdef TROUT_WIFI_NPI
#undef PS_TIMER_DEBUG
#endif

extern struct net_device_stats *g_mac_net_stats;
extern void sta_sleep_disconnected(void);
pstimer_t pstimer;
static int tx_pkts_last = 0;
static int rx_pkts_last = 0;
//---------Static routines------------
static inline unsigned long get_tx_pkts(void)
{
	if (g_mac_net_stats)
	{
		return g_mac_net_stats->tx_packets;
	}
	else
	{
		return 0;
	}
}

static inline unsigned long get_rx_pkts(void)
{
	if (g_mac_net_stats)
	{
		return g_mac_net_stats->rx_packets;
	}
	else
	{
		return 0;
	}
}

#define CACULATE_THRESHOLD(rx_cnt, tx_cnt, rx_cnt_last, tx_cnt_last)		((rx_cnt + tx_cnt) - (rx_cnt_last + tx_cnt_last))

/*
 * To determine whether to swith
 * power save mode.
 * Author: Keguang
 * Date: 20130524
 */
static inline void should_switch(void)
{
	int mode = 0;
	int switch_flag = 0;
	unsigned long tx_pkts;
	unsigned long rx_pkts;
	unsigned long interval;

	tx_pkts = get_tx_pkts();
	rx_pkts = get_rx_pkts();
	interval = pstimer.timeout_ms / 500;

	/*pr_info("rx_pkts %d tx_pkts %d rx_pkts_last %d tx_pkts_last %d\n", rx_pkts, tx_pkts, rx_pkts_last, tx_pkts_last);*/
	pr_info("---- There are %lu TX & RX packets during %d ms\n", CACULATE_THRESHOLD(rx_pkts, tx_pkts, rx_pkts_last, tx_pkts_last), pstimer.timeout_ms);

	if (CACULATE_THRESHOLD(rx_pkts, tx_pkts, rx_pkts_last, tx_pkts_last) < (POWERSAVE_THRESHOLD * interval))
	{
		switch_flag = 1;
		mode = MIN_FAST_PS;
		pstimer.timeout_ms = POWERSAVE_INTERVAL * 500;
	}
	else if (CACULATE_THRESHOLD(rx_pkts, tx_pkts, rx_pkts_last, tx_pkts_last) > (ACTIVE_THRESHOLD * interval))
	{
		switch_flag = 1;
		mode = NO_POWERSAVE;
		pstimer.timeout_ms = ACTIVE_INTERVAL * 500; /*debounce for bad link*/
	}

	if ((switch_flag) && (mode != get_PowerManagementMode())) {
		pr_info("Enter %s\n", USER_PS_MODE_2STR(mode));
		set_PowerManagementMode(mode);
	}
	/*else
		pr_info("Nothing to do!\n");*/

	tx_pkts_last = tx_pkts;
	rx_pkts_last = rx_pkts;
}

static void pstimer_work_func(struct work_struct *work)
{
	//12345678 is used for debug
	int ret = 12345678;
	PS_STATE_T cur_ps_state;
	struct in_device *my_ip_ptr = g_mac_dev->ip_ptr;
	if( reset_mac_trylock() == 0 ){
		return;
	}
	ALARM_WORK_ENTRY(work);
#ifndef PS_TIMER_DEBUG 
	cur_ps_state = get_ps_state();
	//Check if sta has connected 
	if (get_mac_state() == ENABLED || g_keep_connection == BTRUE)
	{
		/*skip power saving when the station is obtaining IP addr*/
		if(my_ip_ptr != NULL)
		{
			struct in_ifaddr *my_ifa_list = my_ip_ptr->ifa_list;
			if(my_ifa_list == NULL)
				goto out;
		}
		should_switch();
	}
	else
	{
		//leon liu added timer status judgement(del_timer() may fail)
		if (pstimer.status == TIMER_STOPPED)
		{
			printk("Timer stopped, won't call sta_sleep_disconnected\n");
			goto out;
		}
		//STA is not connected and timeout happened, go to sleep
		//call sleep
		switch (cur_ps_state)
		{
			case STA_ACTIVE:
				//printk("Trout wifi going to sleep state!!!!!!!!!!!!!!!!!!!\n");
				sta_sleep_disconnected();
				ALARM_WORK_EXIT(work);
				reset_mac_unlock();
				return; /*no timer anymore*/
			//Cannot be in these states below when not connected
			case STA_DOZE:
			case STA_AWAKE:
				printk("Warning: powersave state is in DOZE or AWAKE when STA is not connected!!!!!!!\n");
				break;
			default:
				printk("Error: get_ps_state() returns unknown mode: %d\n", cur_ps_state);
				ret = -1;
				break;
		}

		//Keep ps timer running
		//pstimer_start(&pstimer);
	}
out:
	//Keep ps timer running
	pstimer_start(&pstimer);
#else
	printk("In powersave timer working function!!!!!!!!!!!!!!!!!!!!!\n");
#endif
	ALARM_WORK_EXIT(work);
	reset_mac_unlock();
	//printk("%s returns %d\n", __func__, ret);
}

static void pstimer_timeout_func(ADDRWORD_T data)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return ;
#endif
	if (pstimer.status == TIMER_STOPPED)
	{
		printk("Timer stopped, schedule_work_on will not be called\n");
		return ;
	}
    alarm_fn_work_sched(data);
}

//---------Public interfaces----------
int pstimer_init(pstimer_t *pstimer, int timeout_ms, int ps_pkts_threshold)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if (pstimer != NULL)
	{	
		pstimer->alarm = create_alarm(pstimer_timeout_func, 0, pstimer_work_func);	

		if (pstimer->alarm == NULL)
		{
			return -ENOMEM;
		}

		//leon liu added timer status initialization 2013-4-17
		pstimer->status = TIMER_STOPPED;
		pstimer->timeout_ms = timeout_ms <= 0 ? DEFAULT_PS_TIMEOUT_MS : timeout_ms;
		pstimer->ps_pkts_threshold = ps_pkts_threshold <= 0 ? DEFAULT_PS_PKTS_THRESHOLD : ps_pkts_threshold;

		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

//Refresh or start timer when STA is not connected
int pstimer_start(pstimer_t *pstimer)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if ((pstimer != NULL) && (pstimer->alarm != NULL))
	{
		pstimer_stop(pstimer);

		//leon liu added timer status 2013-4-17
		pstimer->status = TIMER_RUNNING;
		start_alarm(pstimer->alarm, pstimer->timeout_ms);		
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

//Start timer when late_resume is called and STA is connected
int pstimer_start_late_resume(pstimer_t *pstimer)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if ((pstimer != NULL) && (pstimer->alarm != NULL))
	{
		if (get_mac_state() == ENABLED || g_keep_connection == BTRUE)
		{
			//leon liu added timer status 2013-4-17
			pstimer->status = TIMER_RUNNING;
			start_alarm(pstimer->alarm, pstimer->timeout_ms);
			return 0;
		}

		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

int pstimer_set_timeout(pstimer_t *pstimer, int timeout_ms)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if (pstimer != NULL)
	{
		pstimer->timeout_ms = timeout_ms <= 0 ? DEFAULT_PS_TIMEOUT_MS : timeout_ms;
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

int pstimer_set_pkts_threshold(pstimer_t *pstimer, int ps_pkts_threshold)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if (pstimer != NULL)
	{
		pstimer->ps_pkts_threshold = ps_pkts_threshold <= 0 ? DEFAULT_PS_PKTS_THRESHOLD : ps_pkts_threshold;
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

int pstimer_stop(pstimer_t *pstimer)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if ((pstimer != NULL) && (pstimer->alarm != NULL))
	{
		//leon liu added timer status 2013-4-17
		pstimer->status = TIMER_STOPPED;
		stop_alarm(pstimer->alarm);
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

int pstimer_destroy(pstimer_t *pstimer)
{
/* leon liu added, added macro to remove powersave timer function 2013-4-28 */
#ifdef PS_TIMER_DEBUG
	return 0;
#endif
	if ((pstimer != NULL) && (pstimer->alarm != NULL))
	{
		//leon liu added timer status 2013-4-17
		pstimer->status = TIMER_STOPPED;
		delete_alarm(&pstimer->alarm);
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}
#endif
