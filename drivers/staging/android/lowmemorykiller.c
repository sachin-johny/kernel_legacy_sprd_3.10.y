/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_adj values will get killed. Specify the
 * minimum oom_adj values in /sys/module/lowmemorykiller/parameters/adj and the
 * number of free pages in /sys/module/lowmemorykiller/parameters/minfree. Both
 * files take a comma separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill processes
 * with a oom_adj value of 8 or higher when the free memory drops below 4096 pages
 * and kill processes with a oom_adj value of 0 or higher when the free memory
 * drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#ifdef CONFIG_ZRAM_FOR_ANDROID
#include <linux/swap.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm_inline.h>
#endif /* CONFIG_ZRAM_FOR_ANDROID */
#include <linux/memory.h>
#include <linux/memory_hotplug.h>
#include <linux/fs.h>
#include <linux/swap.h>
#include <linux/string.h>
#include <linux/spinlock_types.h>

#ifdef CONFIG_ANDROID_LMK_ENHANCE
#define LOWMEM_DEATHPENDING_DEPTH 3
#endif

#ifdef CONFIG_ANDROID_LMK_THREAD
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
static DECLARE_WAIT_QUEUE_HEAD(lowmemkiller_wait);
#endif

static uint32_t lowmem_debug_level = 2;
static int lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static size_t lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 4;
static pid_t last_killed_pid = 0;

DEFINE_SPINLOCK(lmk_wl_lock);

#define MAX_LMK_WHITE_LIST  0x10
/*lowmemory killer white list*/
struct lmk_wl{
    char wl_name[0x20];
    struct task_struct *wl_tsk;
};

/*note: task name limit 16 characters, intercept last 16 characters*/
static struct lmk_wl lmk_wl_info[MAX_LMK_WHITE_LIST]={
    {"ndroid.launcher", NULL}, 
    {"thunderst.radio", NULL},
};

#ifdef CONFIG_ZRAM_FOR_ANDROID
static unsigned int  default_interval_time = 2*HZ;
static  unsigned int  swap_interval_time = 2*HZ;
#endif


#ifdef CONFIG_ZRAM_FOR_ANDROID
static unsigned int  kill_home_adj_wmark = 6;
static uint32_t lowmem_swap_app_enable = 1;
static uint32_t lowmem_minfile_check_enable = 0;
static uint32_t lowmem_last_swap_time = 0;


static size_t lowmem_minfile[6] = {
	22 * 1024,	 //88MB
	18 * 1024,	 //72MB
	16 * 1024,	 //64MB
	12 * 1024,	 //48MB
	8 * 1024,	 //32MB
	6 * 1024,	 //24MB
};
static int lowmem_minfile_size = 6;

static size_t swap_reclaim_adj[6] = {
        0,
        0,
        0,
        5,
        7,
        9,
};
static int swap_reclaim_adj_size = 6;

static struct class *lmk_class;
static struct device *lmk_dev;
static int lmk_kill_pid = 0;
static int lmk_kill_ok = 0;

extern atomic_t optimize_comp_on;

int lowmemkiller_reclaim_adj = 1;
int swap_to_zram(int  nr_to_scan,  int  min_adj, int max_adj);

extern int isolate_lru_page_compcache(struct page *page);
extern void putback_lru_page(struct page *page);
extern unsigned int zone_id_shrink_pagelist(struct zone *zone_id,struct list_head *page_list, unsigned int nr_to_reclaim);
#define lru_to_page(_head) (list_entry((_head)->prev, struct page, lru))

/*front app adj & most system process oom_adj definition*/
#define FRONT_APP_ADJ          0
/* enable debug log*/
#define SWAP_PROCESS_DEBUG_LOG 0
/* free RAM 8M(2048 pages) */
#define CHECK_FREE_MEMORY 2048
/* free swap (10240 pages) */
#define CHECK_FREE_SWAPSPACE  10240

static unsigned int check_free_memory = 0;

enum pageout_io {
	PAGEOUT_IO_ASYNC,
	PAGEOUT_IO_SYNC,
};


#endif /* CONFIG_ZRAM_FOR_ANDROID */
#ifdef CONFIG_ANDROID_LMK_ENHANCE
static struct task_struct *lowmem_deathpending[LOWMEM_DEATHPENDING_DEPTH] = {NULL,};
#else
static struct task_struct *lowmem_deathpending;
#endif
static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			printk(x);			\
	} while (0)

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data);

static struct notifier_block task_nb = {
	.notifier_call	= task_notify_func,
};

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data)
{
	struct task_struct *task = data;

#ifdef CONFIG_ANDROID_LMK_ENHANCE
	int i = 0;
	for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++)
		if (task == lowmem_deathpending[i]) {
			lowmem_deathpending[i] = NULL;
		break;
	}
#else
	if (task == lowmem_deathpending)
		lowmem_deathpending = NULL;
#endif
	return NOTIFY_OK;
}

#ifdef  CONFIG_ZRAM_FOR_ANDROID
int getbuddyfreepages(void)
{
	struct zone *zone = NULL;
	int  total = 0;
	for_each_populated_zone(zone) 
	{
		unsigned long  flags, order;
//		spin_lock_irqsave(&zone->lock, flags);
		for (order = 0; order < MAX_ORDER; order++) 
		{
			total += zone->free_area[order].nr_free << order;
		}
//		spin_unlock_irqrestore(&zone->lock, flags);
	}
	return total;
}
#endif


static void lowmem_white_list_init(void)
{
    int count=0;

    spin_lock(&lmk_wl_lock);
    for(count=0; count<sizeof(lmk_wl_info)/sizeof(lmk_wl_info[0]); count++){
            lmk_wl_info[count].wl_tsk = NULL;
    }
    spin_unlock(&lmk_wl_lock);
}

static int lowmem_white_list_chk(struct task_struct *p)
{
    int count;
    
    if(p == NULL)
        return 0;
 
    spin_lock(&lmk_wl_lock);
    for(count=0; count<sizeof(lmk_wl_info)/sizeof(lmk_wl_info[0]); count++){
        if(!strcmp((char*)&lmk_wl_info[count].wl_name[0], p->comm)){
            lmk_wl_info[count].wl_tsk = p;
            lowmem_print(2, "[LMK]: %s find white list process:%s\r\n",__func__, p->comm);
            spin_unlock(&lmk_wl_lock);
            return 1;
        }
    }
    spin_unlock(&lmk_wl_lock);
    return 0;
}

static struct  task_struct *lowmem_white_list_kill(int did_some_progress, int* size, int select_oom_adj)
{
    int count;
    int select_size;
    int tasksize;
    struct task_struct *p;
    struct task_struct *select=NULL;

    lowmem_print(2, "[LMK]: %s did_some_progress=%d\r\n",\
            __func__, did_some_progress);

    if(did_some_progress)
        return 0;
 
    spin_lock(&lmk_wl_lock);
    for(count=0; count<sizeof(lmk_wl_info)/sizeof(lmk_wl_info[0]); count++){
        if(lmk_wl_info[count].wl_tsk != NULL){
            p=lmk_wl_info[count].wl_tsk;
            task_lock(p);
            #ifdef CONFIG_ZRAM
		   tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);
            #else		
		   tasksize = get_mm_rss(p->mm);
            #endif

            if(p->signal->oom_adj<select_oom_adj){
                task_unlock(p);
                continue;
            }

            if(!select){
                select=p;
                *size=select_size=tasksize;
            }else{
                if(p->signal->oom_adj > select->signal->oom_adj){
                    task_unlock(p);
                    continue;
                }
                if((p->signal->oom_adj == select->signal->oom_adj)\
                        && (tasksize <= select_size)){
                    task_unlock(p);
                    continue;
                }
                select=p;
                *size=select_size=tasksize;
            }
            task_unlock(p);
        }
    }
    spin_unlock(&lmk_wl_lock);

    if(select){
        lowmem_print(2, "[LMK]: %s find white list process:%s, task_size=%d\r\n",\
            __func__, select->comm, *size);
    }
    return select;
}

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *p;
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	struct task_struct *selected[LOWMEM_DEATHPENDING_DEPTH] = {NULL,};
#else
	struct task_struct *selected = NULL;
#endif
        struct task_struct *wl_selected=NULL;
	int    wl_selected_tasksize=0;
        int    wl_kill_trigger=0;
        int    wl_make_some_progress=0;
        int rem = 0;
	int tasksize;
	int i;
	int min_adj = OOM_ADJUST_MAX + 1;
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	int selected_tasksize[LOWMEM_DEATHPENDING_DEPTH] = {0,};
	int selected_oom_adj[LOWMEM_DEATHPENDING_DEPTH] = {OOM_ADJUST_MAX,};
	int all_selected_oom = 0;
#else
	int selected_tasksize = 0;
	int selected_oom_adj;
#endif
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM);
#ifdef CONFIG_ZRAM_FOR_ANDROID			
	int  oom_adj_index = 0;
	int to_reclaimed = 0;
#endif  /*CONFIG_ZRAM_FOR_ANDROID*/


#ifdef  CONFIG_ZRAM
        other_free -= totalreserve_pages;
	if(other_free < 0)	
	{
		other_free = 0;
	}

	other_file  -=  total_swapcache_pages;
        if(other_file < 0)
        {
        	other_file = 0;
        }
#endif  /*CONFIG_ZRAM*/


	/*
	 * If we already have a death outstanding, then
	 * bail out right away; indicating to vmscan
	 * that we have nothing further to offer on
	 * this pass.
	 *
	 */
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++) {
		if (lowmem_deathpending[i] &&
			time_before_eq(jiffies, lowmem_deathpending_timeout))
			return 0;
	}
#else
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return 0;
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for (i = 0; i < array_size; i++) {
		if (other_free < lowmem_minfree[i] &&
		    other_file < lowmem_minfree[i]) {
			min_adj = lowmem_adj[i];
#ifdef CONFIG_ZRAM_FOR_ANDROID				    
		       oom_adj_index = i;
#endif
				break;
		}
	}
	
	if (sc->nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %d\n",
			     sc->nr_to_scan, sc->gfp_mask, other_free, other_file,
			     min_adj);
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);


        //under try free page situation, should try harder
        if(!current_is_kswapd()){
                if((sc->gfp_mask & GFP_LMK_TRY_HARDER) == GFP_LMK_TRY_HARDER){
   
                    if(min_adj > lowmem_adj[0]){
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	                for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++)
		            selected_oom_adj[i]=min_adj=lowmem_adj[0];
#else
	                    selected_oom_adj=min_adj=lowmem_adj[0];
#endif
                            lowmem_print(2, "[LMK]alloc page rountine, try harder old: %d, new:%d\r\n",\
                                min_adj, min_adj);
                    }
                }
         }else{
#ifdef CONFIG_ZRAM_FOR_ANDROID
	    if(lowmem_minfile_check_enable && (sc->nr_to_scan > 0))
	    {
		int t = 0;
		unsigned int zram_swap_size = 0;
		struct sysinfo si = {0};
		si_swapinfo(&si);	
		zram_swap_size = (si.totalswap - si.freeswap);

		lowmem_print(2,"\r\n[LMK_swap] init oom_adj_index:%d, other_free:%d, other_file:%d\r\n", oom_adj_index, other_free, other_file);
                if( min_adj == OOM_ADJUST_MAX + 1)
		{
			lowmem_print(2,"\r\n[LMK_swap] Cache value high: other_free:%d, other_file:%d, zram_swap_size:%d\r\n",
					    other_free,other_file,  zram_swap_size);
			oom_adj_index = ARRAY_SIZE(lowmem_adj) - 1;
		}


		//Recalculate min_adj value according to swapped size
		for(t = oom_adj_index; t  >= 0; t--)
		{
			if(zram_swap_size < lowmem_minfile[t])
			{
				min_adj = lowmem_adj[t];
				break;
			}
		}


		if( zram_swap_size  >=  lowmem_minfile[0]) 
		{
			if ((min_adj == lowmem_adj[0]) && ((other_file + other_free) < (totalreserve_pages << 1)))
			{
				lowmem_print(2, "\r\n[LMK_swap] WARN No Memory: other_free:%d, other_file:%d,\
                                        lowmem_minfile[0]:%d, min_adj:%d, totalreserve_pages:%d, zram_swap_size:%d\r\n",\
                                            other_free,other_file, lowmem_minfile[0], min_adj, \
                                                totalreserve_pages << 1,zram_swap_size);
				min_adj = 0;
			}
			else
			{
				min_adj = lowmem_adj[0];
			}
		}		
		lowmem_print(2, "\r\n[LMK_swap] zram_swap_size:%d, min_adj:%d, reserve_pages=%d \r\n", \
					    zram_swap_size, min_adj, totalreserve_pages);

		
		if(min_adj  != lowmem_adj[oom_adj_index])
		{
			lowmem_print(2, "[LMK_swap]adjudge adj, old: %d, new:%d\r\n", lowmem_adj[oom_adj_index], min_adj);
		}
            }
		
	 }
#endif

	if(sc->nr_to_scan <= 0 || min_adj == OOM_ADJUST_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     sc->nr_to_scan, sc->gfp_mask, rem);
#ifndef CONFIG_ANDROID_LMK_THREAD
		return rem;
#endif
	}
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++)
		selected_oom_adj[i] = min_adj;
#else
	selected_oom_adj = min_adj;
#endif

#ifdef  CONFIG_ZRAM_FOR_ANDROID
	if(current_is_kswapd()){
		lowmem_print(2,"[LMK_swap] other_free:%d, other_file:%d, min_free[%d]:%d, min_adj:%d\r\n",
						   other_free,other_file, oom_adj_index, lowmem_minfree[oom_adj_index], min_adj);

                if(lowmem_swap_app_enable && ((jiffies -lowmem_last_swap_time) >= swap_interval_time) \
                        && (min_adj>FRONT_APP_ADJ)/*for performance consideration, avoid swapin front app& system process*/\
                            && ((min_adj >= lowmem_adj[0]) && (min_adj < lowmem_adj[5]))/*avoid swap in empty process*/){
			int times = 0;
			struct sysinfo si = {0};
                        int  count=0;
			int  buddy_free = getbuddyfreepages()  >>  1;   //buddy pages /2
                        int  start_adj=0;
                        const static int  swap_thresh[15]={8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 102, 110, 118};
                        const static int  swap_to_scan[15]={1024, 1024, 768, 768, 768, 512, 512, 512, 256, 256, 64, 64, 32, 16, 0};
                        const static int  scan_num=1024;
                        const static int  scan_max_times=2;
                        
                        if(buddy_free > scan_num){
				buddy_free = scan_num;
			}

                        count=jiffies;
                        /*Only run at buddy pages enough*/
                        if(buddy_free > (SWAP_CLUSTER_MAX  << 1) ){
                              for(start_adj=1; start_adj<min_adj; start_adj++){
                                 /*swap policy:
                                  * (a) if current mem pressure not high, swap harder(not easily exist)
                                  * (b) if process has low oom_adj, this process will swap harder, for consideration this process
                                  *     will not easily been killed*/
                                 to_reclaimed += swap_to_zram((buddy_free*swap_to_scan[start_adj])/swap_to_scan[0], start_adj, start_adj);
			
			         lowmem_print(2,"[LMK_swap]buddy_free:%d, swap_to_scan[%d]:%d, to_reclaimed:%d,  min_adj:%d, time:%d ms\r\n", \
                                         buddy_free, start_adj, swap_to_scan[start_adj], to_reclaimed,  min_adj, jiffies_to_msecs(jiffies-count));
			
			         if(to_reclaimed >= swap_thresh[min_adj]){
			            swap_interval_time = default_interval_time;
                                    return rem - to_reclaimed;
			         }
                             }
                        }

			if(0 == to_reclaimed){
			       times = scan_max_times;
			}
			else{
				times = buddy_free*(swap_to_scan[min_adj]/swap_to_scan[0])/to_reclaimed;
				if(times > scan_max_times){
					times = scan_max_times;
				}
			}
			
			swap_interval_time = default_interval_time * times;
			lowmem_last_swap_time =  jiffies;	   
			si_swapinfo(&si);
                        lowmem_print(2,"[LMK_swap] buddy_free:%d, si.totalswap:%d, si.freeswap:%d, minfile:%d\r\n", \
                                buddy_free, si.totalswap, si.freeswap, lowmem_minfile[oom_adj_index]);

			if( (si.totalswap - si.freeswap) < lowmem_minfile[oom_adj_index]){
                                return rem - to_reclaimed;
			}
                 }
#endif
            }

#ifdef  CONFIG_ZRAM_FOR_ANDROID
            lowmem_print(2,"[LMK] kill process start ,other_free:%d, other_file:%d, min_free[%d]:%d, min_adj:%d\r\n", 
						   other_free,other_file, oom_adj_index, lowmem_minfree[oom_adj_index], min_adj);
#endif
            lowmem_white_list_init();
            read_lock(&tasklist_lock);
	    for_each_process(p) {
		struct mm_struct *mm;
		struct signal_struct *sig;
		int oom_adj;

		task_lock(p);
		mm = p->mm;
		sig = p->signal;
		if (!mm || !sig) {
			task_unlock(p);
			continue;
		}
		oom_adj = sig->oom_adj;
		lowmem_print(6, "[[LMK] [%d:%s] loop, current adj %d\n",p->pid, p->comm, oom_adj);

                if(p->state == TASK_UNINTERRUPTIBLE){
                    lowmem_print(6, " [LMK] [%d, %s] uninterruptible skip, adj %d\n",
				p->pid, p->comm, oom_adj);
                    task_unlock(p);
                    continue;
                }
		if (oom_adj < min_adj) {
			lowmem_print(6, " [LMK] [%d:%s] current adj %d\n",
				p->pid, p->comm, oom_adj);
                        task_unlock(p);
			continue;
		}
                if(lowmem_white_list_chk(p)){
                	lowmem_print(2, " [LMK] [%d:%s] whilt lisk check skip, current adj %d\n",
				p->pid, p->comm, oom_adj);
                        
                        if(min_adj<=lowmem_adj[0]){
                            wl_kill_trigger=1;
                        }
                        task_unlock(p);
			continue;
                }
#ifdef CONFIG_ZRAM
		tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);
#else		
		tasksize = get_mm_rss(mm);
#endif
		task_unlock(p);
		if (tasksize <= 0){
		    lowmem_print(6, " [LMK] [%d:%s] no task size skip, adj %d, %d\n",
				p->pid, p->comm, oom_adj, tasksize);
                    continue;
                }

#ifdef CONFIG_ANDROID_LMK_ENHANCE
		for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++) {
			if (all_selected_oom >= LOWMEM_DEATHPENDING_DEPTH) {
				if (oom_adj < selected_oom_adj[i])
					continue;
			if (oom_adj == selected_oom_adj[i] &&
				tasksize <= selected_tasksize[i])
				continue;
			} else if (selected[i])
				continue;

			selected[i] = p;
			selected_tasksize[i] = tasksize;
			selected_oom_adj[i] = oom_adj;

			if (all_selected_oom < LOWMEM_DEATHPENDING_DEPTH)
				all_selected_oom++;

			lowmem_print(2, "[LMK] select %d (%s), adj %d, size %d, to kill\n",
				p->pid, p->comm, oom_adj, tasksize);

			break;
		}
#else
		if (selected) {
			if (oom_adj < selected_oom_adj)
				continue;
			if (oom_adj == selected_oom_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_adj = oom_adj;
		lowmem_print(2, "[LMK] select %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_adj, tasksize);
#endif
	    }
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	    for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++) {
		if (selected[i]) {
			lowmem_print(1, "[LMK] send sigkill to %d (%s), adj %d, size %d\n",
				selected[i]->pid, selected[i]->comm,
				selected_oom_adj[i], selected_tasksize[i]);
			lowmem_deathpending[i] = selected[i];
			lowmem_deathpending_timeout = jiffies + HZ;
			force_sig(SIGKILL, selected[i]);
 
                        if(selected[i]->pid == last_killed_pid && selected[i]->signal->oom_adj > 2) {
                            selected[i]->signal->oom_adj--;
                         } else
                         last_killed_pid = selected[i]->pid;
                         rem -= selected_tasksize[i];
	                 wl_make_some_progress=1;   
                }
	    }
#else
	    if (selected) {
		lowmem_print(1, "[LMK] send sigkill to %d (%s), adj %d, size %d\n",
			     selected->pid, selected->comm,
			     selected_oom_adj, selected_tasksize);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		force_sig(SIGKILL, selected);
                if(selected->pid == last_killed_pid && selected->signal->oom_adj > 2) {
                       selected->signal->oom_adj--;
                } else
                       last_killed_pid = selected->pid;
                
                wl_make_some_progress=1;
		rem -= selected_tasksize;
	    }
#endif

            if(wl_kill_trigger){
                 wl_selected=lowmem_white_list_kill(wl_make_some_progress, (int*)&wl_selected_tasksize, min_adj);
                 if(wl_selected){
#ifdef CONFIG_ANDROID_LMK_ENHANCE
                    lowmem_deathpending[0] = wl_selected;
		    lowmem_deathpending_timeout[0] = jiffies + HZ;
#else
                    lowmem_deathpending = wl_selected;
		    lowmem_deathpending_timeout = jiffies + HZ;
#endif
                    force_sig(SIGKILL, wl_selected);
                    if(wl_selected->pid == last_killed_pid && wl_selected->signal->oom_adj > 2) {
                       wl_selected->signal->oom_adj--;
                    } else
                       last_killed_pid = wl_selected->pid;

		    rem -= wl_selected_tasksize;
                    lowmem_print(2, "[LMK] lowmem_shrink: wl kill tasksize=%d\n", wl_selected_tasksize);
                 }     
             }
	read_unlock(&tasklist_lock);
        lowmem_print(2, "lowmem_shrink %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);

        return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

#ifdef CONFIG_ANDROID_LMK_THREAD
static struct shrink_control sc;

static void lowmem_killer(void) 
{
	set_user_nice(current, 5);
        set_freezable();
        sc.nr_to_scan = 1;
        sc.gfp_mask = 0;

        do {
		int other_free;
		int other_file;

		lowmem_shrink(NULL, &sc);

		wait_event_freezable_timeout(lowmemkiller_wait, false, msecs_to_jiffies(1000));

        } while (1);

        pr_debug("lowmemorykiller exiting\n");
        return 0;
}
#endif

#ifdef CONFIG_ZRAM_FOR_ANDROID
/*
 * zone_id_shrink_pagelist() clear page flags,
 * update the memory zone status, and swap pagelist
 */

static unsigned int shrink_pages(struct mm_struct *mm,
				 struct list_head *zone0_page_list,
				 struct list_head *zone1_page_list,
				 unsigned int num_to_scan)
{
	unsigned long addr;
	unsigned int isolate_pages_countter = 0;

	struct vm_area_struct *vma = mm->mmap;
	while (vma != NULL) {

		for (addr = vma->vm_start; addr < vma->vm_end;
		     addr += PAGE_SIZE) {
			struct page *page;
			/*get the page address from virtual memory address */
			page = follow_page(vma, addr, FOLL_GET);

			if (page && !IS_ERR(page)) {

				put_page(page);
				/* only moveable, anonymous and not dirty pages can be swapped  */
				if ((!PageUnevictable(page))
				    && (!PageDirty(page)) && ((PageAnon(page)))
				    && (0 == page_is_file_cache(page))) {
					switch (page_zone_id(page)) {
					case 0:
						if (!isolate_lru_page_compcache(page)) {
							/* isolate page from LRU and add to temp list  */
							/*create new page list, it will be used in shrink_page_list */
							list_add_tail(&page->lru, zone0_page_list);
							isolate_pages_countter++;
						}
						break;
					case 1:
						if (!isolate_lru_page_compcache(page)) {
							/* isolate page from LRU and add to temp list  */
							/*create new page list, it will be used in shrink_page_list */
							list_add_tail(&page->lru, zone1_page_list);
							isolate_pages_countter++;
						}
						break;
					default:
						break;
					}
				}
			}

			if (isolate_pages_countter >= num_to_scan) {
				return isolate_pages_countter;
			}
		}

		vma = vma->vm_next;
	}

	return isolate_pages_countter;
}

/*
 * swap_application_pages() will search the
 * pages which can be swapped, then call
 * zone_id_shrink_pagelist to update zone
 * status
 */
static unsigned int swap_pages(struct list_head *zone0_page_list,
			       struct list_head *zone1_page_list, unsigned int  nr_to_reclaim)
{
	struct zone *zone_id_0 = &NODE_DATA(0)->node_zones[0];
	struct zone *zone_id_1 = &NODE_DATA(0)->node_zones[1];
	unsigned int pages_counter = 0;

	/*if the page list is not empty, call zone_id_shrink_pagelist to update zone status */
	if ((zone_id_0) && (!list_empty(zone0_page_list))) {
		pages_counter +=
		    zone_id_shrink_pagelist(zone_id_0, zone0_page_list, nr_to_reclaim);
	}
	if ((zone_id_1) && (!list_empty(zone1_page_list))) {
		pages_counter +=
		    zone_id_shrink_pagelist(zone_id_1, zone1_page_list, nr_to_reclaim);
	}
	return pages_counter;
}


int swap_to_zram(int  nr_to_scan,  int  min_adj, int   max_adj)
{
	struct task_struct *p = NULL;
	int pages_tofree = 0, pages_freed = 0;
	LIST_HEAD(zone0_page_list);
	LIST_HEAD(zone1_page_list);
	struct sysinfo ramzswap_info = { 0 };
	int  shrink_to_scan = nr_to_scan ;
    
	si_swapinfo(&ramzswap_info);
	si_meminfo(&ramzswap_info);
	
	if(ramzswap_info.freeswap  <  CHECK_FREE_SWAPSPACE)
	{
		return 0;
	}

        if(nr_to_scan<=0){
                return 0;
        }
	pages_tofree = 0;
		
	read_lock(&tasklist_lock);
	for_each_process(p) 
	{
		struct mm_struct *mm;
		struct signal_struct *sig;
		int oom_adj;

		task_lock(p);
		mm = p->mm;
		sig = p->signal;
		if (!mm || !sig) 
		{
			task_unlock(p);
			continue;
		}
			
                oom_adj = sig->oom_adj;
                if ( (oom_adj < min_adj) || (oom_adj > max_adj) ||\
			(__task_cred(p)->uid  <= 10000) || (p->flags & PF_KTHREAD) )
		{
			task_unlock(p);
			continue;
		}

         
                printk("%s, name:%s, adj:%d, policy:%u, uid:%u\r\n", 
			__func__, p->comm, oom_adj,p->policy, __task_cred(p)->uid );


		atomic_inc(&mm->mm_users);
		task_unlock(p);
                read_unlock(&tasklist_lock);
		down_read(&mm->mmap_sem);
		pages_tofree += shrink_pages(mm, &zone0_page_list, &zone1_page_list, shrink_to_scan);
		up_read(&mm->mmap_sem);
		mmput(mm);
                read_lock(&tasklist_lock);
		pr_debug("%s, name:%s, adj:%d, policy:%u, uid:%u\r\n", 
							__func__, p->comm, oom_adj,p->policy, __task_cred(p)->uid );
	 }
	read_unlock(&tasklist_lock);

	if(pages_tofree)
	{
		pages_freed += swap_pages(&zone0_page_list, &zone1_page_list, pages_tofree);
	}

	return pages_freed;
}


static ssize_t lmk_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d,%d\n", lmk_kill_pid, lmk_kill_ok);
}

/*
 * lmk_state_store() will called by framework,
 * the framework will send the pid of process that need to be swapped
 */
static ssize_t lmk_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	sscanf(buf, "%d,%d", &lmk_kill_pid, &lmk_kill_ok);

	/* if the screen on, the optimized compcache will stop */
	if (atomic_read(&optimize_comp_on) != 1)
		return size;

	if (lmk_kill_ok == 1) {
		struct task_struct *p;
		struct task_struct *selected = NULL;
		struct sysinfo ramzswap_info = { 0 };
		struct mm_struct *mm_scan = NULL;

		/*
		 * check the free RAM and swap area,
		 * stop the optimized compcache in cpu idle case;
		 * leave some swap area for using in low memory case
		 */
		si_swapinfo(&ramzswap_info);
		si_meminfo(&ramzswap_info);

		if ((ramzswap_info.freeswap < CHECK_FREE_SWAPSPACE) ||
		    (ramzswap_info.freeram < check_free_memory)) {
#if SWAP_PROCESS_DEBUG_LOG > 0
			printk(KERN_INFO "idletime compcache is ignored : free RAM %lu, free swap %lu\n",
			ramzswap_info.freeram, ramzswap_info.freeswap);
#endif
			lmk_kill_ok = 0;
			return size;
		}

		read_lock(&tasklist_lock);
		for_each_process(p) {
			if ((p->pid == lmk_kill_pid) &&
			    (__task_cred(p)->uid > 10000)) {
				task_lock(p);
				selected = p;
				if (!selected->mm || !selected->signal) {
					task_unlock(p);
					selected = NULL;
					break;
				}
				mm_scan = selected->mm;
				if (mm_scan) {
					if (selected->flags & PF_KTHREAD)
						mm_scan = NULL;
					else
						atomic_inc(&mm_scan->mm_users);
				}
				task_unlock(selected);

#if SWAP_PROCESS_DEBUG_LOG > 0
				printk(KERN_INFO "idle time compcache: swap process pid %d, name %s, oom %d, task size %ld\n",
					p->pid, p->comm,
					p->signal->oom_adj,
					get_mm_rss(p->mm));
#endif
				break;
			}
		}
		read_unlock(&tasklist_lock);

		if (mm_scan) {
			LIST_HEAD(zone0_page_list);
			LIST_HEAD(zone1_page_list);
			int pages_tofree = 0, pages_freed = 0;

			down_read(&mm_scan->mmap_sem);
			pages_tofree =
			shrink_pages(mm_scan, &zone0_page_list,
					&zone1_page_list, 0x7FFFFFFF);
			up_read(&mm_scan->mmap_sem);
			mmput(mm_scan);
			pages_freed =
			    swap_pages(&zone0_page_list,
				       &zone1_page_list, pages_tofree);
			lmk_kill_ok = 0;

		}
	}

	return size;
}

static DEVICE_ATTR(lmk_state, 0664, lmk_state_show, lmk_state_store);

#endif /* CONFIG_ZRAM_FOR_ANDROID */
static int __init lowmem_init(void)
{
#ifdef CONFIG_ANDROID_LMK_THREAD
        kthread_run(lowmem_killer, NULL, "lowmemorykiller");
#else
#ifdef CONFIG_ZRAM_FOR_ANDROID
	struct zone *zone;
	unsigned int high_wmark = 0;
	lowmem_last_swap_time =  jiffies;
#endif
	task_free_register(&task_nb);
	register_shrinker(&lowmem_shrinker);
#ifdef CONFIG_ZRAM_FOR_ANDROID
	for_each_zone(zone) {
		if (high_wmark < zone->watermark[WMARK_HIGH])
			high_wmark = zone->watermark[WMARK_HIGH];
	}
	check_free_memory = (high_wmark != 0) ? high_wmark : CHECK_FREE_MEMORY;

	lmk_class = class_create(THIS_MODULE, "lmk");
	if (IS_ERR(lmk_class)) {
		printk(KERN_ERR "Failed to create class(lmk)\n");
		return 0;
	}
	lmk_dev = device_create(lmk_class, NULL, 0, NULL, "lowmemorykiller");
	if (IS_ERR(lmk_dev)) {
		printk(KERN_ERR
		       "Failed to create device(lowmemorykiller)!= %ld\n",
		       IS_ERR(lmk_dev));
		return 0;
	}
	if (device_create_file(lmk_dev, &dev_attr_lmk_state) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
		       dev_attr_lmk_state.attr.name);
#endif /* CONFIG_ZRAM_FOR_ANDROID */
#endif
	return 0;
}

static void __exit lowmem_exit(void)
{
#ifndef CONFIG_ANDROID_LMK_THREAD
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);
#endif
}

#ifdef CONFIG_ZRAM_FOR_ANDROID
module_param_named(lowmem_swap_app_enable, lowmem_swap_app_enable, int, S_IRUGO | S_IWUSR);
module_param_named(lowmem_minfile_check_enable, lowmem_minfile_check_enable, int, S_IRUGO | S_IWUSR);
module_param_named(kill_home_adj_wmark, kill_home_adj_wmark, int, S_IRUGO | S_IWUSR);
module_param_array_named(lowmem_minfile, lowmem_minfile, int, &lowmem_minfile_size,
			 S_IRUGO | S_IWUSR);

module_param_array_named(swap_reclaim_adj, swap_reclaim_adj, int, &swap_reclaim_adj_size,

                         S_IRUGO | S_IWUSR);
module_param_named(default_interval_time, default_interval_time, int, S_IRUGO | S_IWUSR);

#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

