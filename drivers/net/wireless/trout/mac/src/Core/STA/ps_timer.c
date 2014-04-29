//leon liu created 2013-3-27
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include <linux/inetdevice.h>
#ifdef CONFIG_CFG80211
#include <net/cfg80211.h>
#endif
#include "ps_timer.h"
#include "pm_sta.h"

/* leon liu opened PS_TIMER_DEBUG to remove powersave timer function 2013-4-28 */
#define PS_TIMER_DEBUG //danny deng open powersave functon 

#ifdef TROUT_WIFI_NPI
#undef PS_TIMER_DEBUG
#endif

#define TROUT_MODULE_FAIL               0
#define TROUT_MODULE_LOADING            1
#define TROUT_MODULE_LOADED             2
#define TROUT_MODULE_UNLOADING          3
#define TROUT_MODULE_UNLOADED           4
#define TROUT_MODULE_IDLE               5

extern void set_trout_module_state(unsigned int st); 
extern unsigned int check_trout_module_state(unsigned int st); 
extern struct net_device_stats *g_mac_net_stats;
extern void sta_sleep_disconnected(void);
pstimer_t pstimer;
static int tx_pkts_last = 0;
static int rx_pkts_last = 0;
//---------Static routines------------
static void pstimer_work_func(struct work_struct *work)
{
#ifdef CONFIG_CFG80211
	struct wireless_dev *wdev = g_mac_dev->ieee80211_ptr;
#endif
	//pr_info("%s: entered\n", __func__);

	if (reset_mac_trylock() == 0){
		pr_info("during reset_mac, nothing to do\n");
		return ;
}

	if( mutex_is_locked(&pstimer.run_lock) || check_trout_module_state(TROUT_MODULE_UNLOADING)){
		pr_info("Nothing to be done\n");
		reset_mac_unlock();
		return;
	}

	//pr_info("%s: mutex_trylock\n", __func__);
	if (mutex_trylock(&pstimer.run_lock) == 0){
		/*If we cann't get run_lock, this means pstimer_stop is called*/
		pr_info("%s: mutex_trylock failed\n", __func__);
		reset_mac_unlock();
		return ;
	}

	if (get_mac_state() == ENABLED || g_keep_connection == BTRUE || (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE 
			||  mutex_is_locked(&suspend_mutex) || g_wifi_suspend_status != wifi_suspend_nosuspend  || itm_scan_flag == 1
#ifdef CONFIG_CFG80211
			|| wdev->sme_state != CFG80211_SME_IDLE
#endif
	   ){
		pr_info("%s: condition is not satisfied, not going to sleep\n", __func__);
		goto out;
}

	/*
	 * sta_sleep_disconnected will judge the current state,
	 * if already in TROUT_SLEEP, nothing happens
	 */
	//pr_info("%s: goint to sleep_disconnected\n", __func__);
				sta_sleep_disconnected();
	//pr_info("%s: mod_timer to next 1s\n", __func__);
	mod_timer(&pstimer.timer, jiffies + 1 * HZ);
	//pr_info("%s: mod_timer ok\n", __func__);
out:
	//pr_info("%s: exited\n", __func__);
	mutex_unlock(&pstimer.run_lock);
	reset_mac_unlock();
}

//Timer expiring function
static void pstimer_expire_func(unsigned long data)
{
	//Check if we should dive into work
	//We're in interrupt context,so never use reset_mac_trylock
	if ((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE || check_trout_module_state(TROUT_MODULE_UNLOADING)
		       	|| g_wifi_suspend_status != wifi_suspend_nosuspend){
		pr_info("%s: no need to shcedule_work\n", __func__);
		return ;
	}

	//pr_info("%s: entered\n", __func__);
	schedule_work(&pstimer.work);
}
//---------Public interfaces----------
int pstimer_init(pstimer_t *pstimer, int timeout_ms, int ps_pkts_threshold)
{
	if (pstimer != NULL)
	{	
		init_timer(&pstimer->timer);
		pstimer->timer.data = (unsigned int)pstimer;
		pstimer->timer.function = pstimer_expire_func;
		INIT_WORK(&pstimer->work, pstimer_work_func);	
		mutex_init(&pstimer->run_lock);	
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
	if (check_trout_module_state(TROUT_MODULE_UNLOADING) ){
		pr_info("%s: module is unloading, nothing to do\n", __func__);
		return -1;

	}

	if ((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE){
		pr_info("%s: during reset, nothing to do\n", __func__);
		return -1;
}

	if (mutex_is_locked(&pstimer->run_lock)){
		pr_info("pstimer: work is running, no need to start again\n");
			return 0;
		}

	mod_timer(&pstimer->timer, jiffies + 1 * HZ);

		return 0;
	}

//Start timer when late_resume is called and STA is connected
int pstimer_start_late_resume(pstimer_t *pstimer)
	{
	return 0;
}

int pstimer_set_timeout(pstimer_t *pstimer, int timeout_ms)
{
	return 0;
}

int pstimer_set_pkts_threshold(pstimer_t *pstimer, int ps_pkts_threshold)
{
		return 0;
	}

int pstimer_stop(pstimer_t *pstimer)
	{
	if (pstimer == NULL){
		return -1;
	}

	pr_info("%s: entered\n", __func__);
	mutex_lock(&pstimer->run_lock);
	/*
	 * run_lock is got:
	 * 1.if work is finished
	 * 2.work is pending(timer expired but work is not scheduled)
	 * 3.work is running(work is scheduled out before mutex_lock)
	 */
	if (work_busy(&pstimer->work)){
		cancel_work_sync(&pstimer->work);
}
	del_timer_sync(&pstimer->timer);		
	mutex_unlock(&pstimer->run_lock);

		return 0;
	}

int pstimer_destroy(pstimer_t *pstimer)
{
	return 0;
}
#endif
