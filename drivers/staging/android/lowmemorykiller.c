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

#ifdef CONFIG_ZRAM_FOR_ANDROID
#include <linux/fs.h>
#include <linux/swap.h>
#endif


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

#ifdef CONFIG_ZRAM_FOR_ANDROID
static int fudgeswap = 512;
#endif


#ifdef CONFIG_ZRAM_FOR_ANDROID
static unsigned int  kill_home_adj_wmark = 6;
static uint32_t lowmem_check_filepages = 1;

static size_t lowmem_minfile[6] = {
	2 * 1024,	
	3 * 1024,	
	4 * 1024,	
	5 * 1024,	
	8 * 1024,	
	10 * 1024,		
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
extern unsigned int zone_id_shrink_pagelist(struct zone *zone_id,struct list_head *page_list);
#define lru_to_page(_head) (list_entry((_head)->prev, struct page, lru))

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

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *p;
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	struct task_struct *selected[LOWMEM_DEATHPENDING_DEPTH] = {NULL,};
#else
	struct task_struct *selected = NULL;
#endif
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
	int lru_file = 0;
	int to_reclaimed = 0;
#endif /*CONFIG_ZRAM_FOR_ANDROID*/

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

#ifdef CONFIG_ZRAM_FOR_ANDROID
	if(fudgeswap != 0){
		struct sysinfo si;
		si_swapinfo(&si);

		if(si.freeswap > 0){
			if(fudgeswap > si.freeswap)
				other_file += si.freeswap;
			else
				other_file += fudgeswap;
		}
	}
#endif


	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

#ifndef CONFIG_ZRAM_FOR_ANDROID	
	for (i = 0; i < array_size; i++) {
		if (other_free < lowmem_minfree[i] &&
		    other_file < lowmem_minfree[i]) {
			min_adj = lowmem_adj[i];
				break;
		}
	}
#else
	lru_file =  global_page_state(NR_ACTIVE_FILE) + global_page_state(NR_INACTIVE_FILE);
	for (i = 0; i < array_size; i++) {
		if ( ((other_free + other_file) < lowmem_minfree[i])  &&
			(lowmem_check_filepages ? (lru_file < lowmem_minfile[i]) : 1 )){
			        min_adj = lowmem_adj[i];
				break;
			}

	}
#endif
	
	if (sc->nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %d\n",
			     sc->nr_to_scan, sc->gfp_mask, other_free, other_file,
			     min_adj);
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (sc->nr_to_scan <= 0 || min_adj == OOM_ADJUST_MAX + 1) {
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
	if(selected_oom_adj >=swap_reclaim_adj[5])
	{
		lowmemkiller_reclaim_adj =	1;
	}
	else if (selected_oom_adj >=swap_reclaim_adj[4])
	{
		lowmemkiller_reclaim_adj = 2;
	}
	else if (selected_oom_adj >=swap_reclaim_adj[3])
	{
		lowmemkiller_reclaim_adj = 4;
	}
	else
	{
		lowmemkiller_reclaim_adj = 8;
	}

	pr_debug("%s:selected_oom_adj:%d, lowmemkiller_reclaim_adj:%d\r\n", __func__, selected_oom_adj, lowmemkiller_reclaim_adj);

	to_reclaimed = swap_to_zram(sc->nr_to_scan, min_adj, 1);
	pr_debug("%s: to_reclaimed:%d, sc->nr_to_scan:%u\r\n", __func__, to_reclaimed, sc->nr_to_scan);
	if(to_reclaimed >= sc->nr_to_scan)
	{
		return rem - to_reclaimed;
	}
#endif

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
		
#ifdef CONFIG_ZRAM_FOR_ANDROID	
		if((oom_adj == 6) && (min_adj >= kill_home_adj_wmark))
		{
		     task_unlock(p);
		    continue;	 
		}
#endif	 /*CONFIG_ZRAM_FOR_ANDROID*/	
		if (oom_adj < min_adj) {
			task_unlock(p);
			continue;
		}
		tasksize = get_mm_rss(mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;

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

			lowmem_print(2, "select %d (%s), adj %d, size %d, to kill\n",
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
		lowmem_print(2, "select %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_adj, tasksize);
#endif
	}
#ifdef CONFIG_ANDROID_LMK_ENHANCE
	for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++) {
		if (selected[i]) {
			lowmem_print(1, "send sigkill to %d (%s), adj %d, size %d\n",
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
		}
	}
#else
	if (selected) {
		lowmem_print(1, "send sigkill to %d (%s), adj %d, size %d\n",
			     selected->pid, selected->comm,
			     selected_oom_adj, selected_tasksize);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		force_sig(SIGKILL, selected);
                if(selected->pid == last_killed_pid && selected->signal->oom_adj > 2) {
                       selected->signal->oom_adj--;
                } else
                       last_killed_pid = selected->pid;

		rem -= selected_tasksize;
	}
#endif
	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	read_unlock(&tasklist_lock);
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
			       struct list_head *zone1_page_list)
{
	struct zone *zone_id_0 = &NODE_DATA(0)->node_zones[0];
	struct zone *zone_id_1 = &NODE_DATA(0)->node_zones[1];
	unsigned int pages_counter = 0;

	/*if the page list is not empty, call zone_id_shrink_pagelist to update zone status */
	if ((zone_id_0) && (!list_empty(zone0_page_list))) {
		pages_counter +=
		    zone_id_shrink_pagelist(zone_id_0, zone0_page_list);
	}
	if ((zone_id_1) && (!list_empty(zone1_page_list))) {
		pages_counter +=
		    zone_id_shrink_pagelist(zone_id_1, zone1_page_list);
	}
	return pages_counter;
}


int swap_to_zram(int  nr_to_scan,  int  min_adj, int   max_adj)
{
	struct task_struct *p = NULL;
	int pages_tofree = 0, pages_freed = 0;
	int  oom_adj_wmark = 0;
	LIST_HEAD(zone0_page_list);
	LIST_HEAD(zone1_page_list);
	struct sysinfo ramzswap_info = { 0 };
	int  shrink_to_scan = (nr_to_scan > 128) ?  nr_to_scan : 128;
    
	si_swapinfo(&ramzswap_info);
	si_meminfo(&ramzswap_info);
	
	if(ramzswap_info.freeswap  <  CHECK_FREE_SWAPSPACE)
	{
		return 0;
	}

	for(oom_adj_wmark = min_adj;  oom_adj_wmark >= max_adj;  oom_adj_wmark--)
	{
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
			if ( oom_adj < oom_adj_wmark) 
			{
				if(__task_cred(p)->uid > 10000)
				{
					pr_debug("%s, name:%s, adj:%d, policy:%u, pri:%u\r\n", 
						__func__, p->comm, oom_adj,p->policy, p->rt_priority);
				}
				task_unlock(p);
				continue;
			}
			
			pages_tofree += shrink_pages(mm, &zone0_page_list, &zone1_page_list, shrink_to_scan);
			
			task_unlock(p);
		}
		read_unlock(&tasklist_lock);

		if(pages_tofree)
		{
			pages_freed += swap_pages(&zone0_page_list, &zone1_page_list);
		}

		if(pages_freed >= shrink_to_scan)
		{
			break;
		}
	}

	pr_debug("%s: pages_tofree:%d, pages_freed:%d\r\n", __func__, pages_tofree, pages_freed);

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
				       &zone1_page_list);
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
module_param_named(lowmem_check_filepages, lowmem_check_filepages, int, S_IRUGO | S_IWUSR);
module_param_named(kill_home_adj_wmark, kill_home_adj_wmark, int, S_IRUGO | S_IWUSR);
module_param_array_named(lowmem_minfile, lowmem_minfile, int, &lowmem_minfile_size,
			 S_IRUGO | S_IWUSR);

module_param_array_named(swap_reclaim_adj, swap_reclaim_adj, int, &swap_reclaim_adj_size,
                         S_IRUGO | S_IWUSR);

#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

#ifdef CONFIG_ZRAM_FOR_ANDROID
module_param_named(fudgeswap, fudgeswap, int, S_IRUGO | S_IWUSR);
#endif

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

