#ifndef __PS_TIMER_H__
#define __PS_TIMER_H__

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE

#include "iconfig.h"
#include "controller_mode_if.h"
#include "core_mode_if.h"
#include "csl_if.h"
#include "fsm.h"
#include "host_if.h"
#include "proc_if.h"
#include "mac_init.h"
#include "itypes.h"
#include "maccontroller.h"
#include "mh.h"
#include "buff_desc.h"
#include "common.h"
#include "test_config.h"
#include "mib.h"
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/err.h>

#include "cglobals.h"
#include "trout_wifi_rx.h"
#include "spi_interface.h"
#include "sta_prot_if.h"
#include "trout_trace.h"
#include "wireless.h"
#include "qmu_if.h"
#include "prot_if.h"
#include "core_mode_if.h"

#define DEFAULT_PS_PKTS_THRESHOLD	5
#define ACTIVE_THRESHOLD		50
#define POWERSAVE_THRESHOLD		5

#define DEFAULT_PS_TIMEOUT_MS		5000
#define ACTIVE_INTERVAL			10
#define POWERSAVE_INTERVAL		1
#define INTERVAL_TIME			500

#define TIMER_STOPPED			0
#define TIMER_RUNNING			1
typedef struct powersave_timer{
	int timeout_ms; 
	int status;	//timer satues 0 - stopped 1 - running
	int ps_pkts_threshold; //If packets transmited during timeout_ms is less than threshold, go to powersave mode
	struct timer_list timer;
	struct work_struct work;
	struct mutex run_lock;
}pstimer_t;

extern pstimer_t pstimer;

int pstimer_init(pstimer_t *pstimer, int timeout_ms, int ps_pkts_threshold);
int pstimer_start(pstimer_t *pstimer);
int pstimer_start_late_resume(pstimer_t *pstimer);
int pstimer_set_timeout(pstimer_t *pstimer, int timeout_ms);
int pstimer_set_pkts_threshold(pstimer_t *pstimer, int ps_pkts_threshold);
int pstimer_stop(pstimer_t *pstimer);
int pstimer_destroy(pstimer_t *pstimer);

#endif/*TROUT_WIFI_POWER_SLEEP_ENABLE*/
#endif /*PS_TIMER_H*/
