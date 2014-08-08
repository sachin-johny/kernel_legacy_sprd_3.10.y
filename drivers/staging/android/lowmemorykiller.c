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
#include <linux/fs.h>
#include <linux/swap.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/cpuset.h>

#define B2G_FG_OOMADJ 0

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

static uint32_t lowmem_minfreeswap_check = 1;
static int lowmem_minfreeswap_size = 6;

/* the value of free swap , total swap >> x*/
static size_t lowmem_minfreeswap[6] = {
     7,
     6,
     5,
     4,
     3,
     2,
};

static size_t lowmem_minfree_notif_trigger;
static size_t lowmem_minswapfree_notif_trigger = 1024;//page

static struct task_struct *lowmem_deathpending;
static unsigned long lowmem_deathpending_timeout;

static struct kobject *lowmem_kobj;

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

static void dump_header(struct task_struct *p, gfp_t gfp_mask, int order,
			struct mem_cgroup *mem, const nodemask_t *nodemask);

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data)
{
	struct task_struct *task = data;

	if (task == lowmem_deathpending)
		lowmem_deathpending = NULL;

	return NOTIFY_OK;
}

static void lowmem_notify_killzone_approach(void);

static inline void get_free_ram(int *p_other_free, int *p_other_file)
{
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM);	
#ifdef CONFIG_ZRAM
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

	*p_other_free = other_free;
	*p_other_file = other_file;
}

#ifdef CONFIG_ANDROID_LMK_DEBUG
extern int user_process_meminfo_show(void);
#endif
#ifdef CONFIG_ZRAM
extern void zram_printlog(void);
#endif
extern void dump_tasks(const struct mem_cgroup *mem, const nodemask_t *nodemask);
extern void boost_dying_task_prio(struct task_struct *p);

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *p;
	struct task_struct *selected = NULL;
	int rem = 0;
	int tasksize;
	int i;
	int min_adj = OOM_ADJUST_MAX + 1;
	int selected_tasksize = 0;
	int selected_oom_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = 0;
	int other_file = 0;
 	int mm_rss = 0;
	int mm_counter = 0;
	struct sysinfo si = {0};

	lowmem_print(3, "%s invoked lowmemorykiller \n", current->comm);

	/*
	 * If we already have a death outstanding, then
	 * bail out right away; indicating to vmscan
	 * that we have nothing further to offer on
	 * this pass.
	 *
	 */
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return 0;

	get_free_ram(&other_free, &other_file);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		if (other_free < lowmem_minfree[i] &&
		    other_file < lowmem_minfree[i]) {
			min_adj = lowmem_adj[i];
			break;
		}
	}

	si_swapinfo(&si);
	if (lowmem_minfreeswap_check && si.totalswap && (min_adj > B2G_FG_OOMADJ)) {
		unsigned int minfreeswap = 0 ;
		int temp_min_adj = OOM_ADJUST_MAX + 1;

		/* Reduce min_adj according to the predefined freeswap levels
		 */
		if (lowmem_minfreeswap_size < array_size)
			array_size = lowmem_minfreeswap_size;

		for (i = 0; i < array_size; i++) {
			minfreeswap = si.freeswap << lowmem_minfreeswap[i];
			if (minfreeswap < si.totalswap) {
				temp_min_adj = lowmem_adj[i];
				break;
			}
		}
		lowmem_print(4, "lowmem_shrink swapfree %u, min_swapfree %u, min_adj %d \n",
				si.freeswap, minfreeswap, temp_min_adj);

		/* don't kill foreground apps when free of swap disk low */
                if(temp_min_adj <= B2G_FG_OOMADJ)
                        temp_min_adj = B2G_FG_OOMADJ + 1;

		if(temp_min_adj < min_adj)
			min_adj = temp_min_adj;
	}

	if (sc->nr_to_scan > 0)
		lowmem_print(4, "lowmem_shrink %lu, %x, ofree %d %d, ma %d\n",
			     sc->nr_to_scan, sc->gfp_mask, other_free, other_file,
			     min_adj);

	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (sc->nr_to_scan <= 0 || min_adj == OOM_ADJUST_MAX + 1) {
		lowmem_print(3, "lowmem_shrink %lu, %x, return %d\n",
			     sc->nr_to_scan, sc->gfp_mask, rem);
		return rem;
	}
	/*
	 add si.freeswap trigger check
	 swap off check:
		lowmem_minfreeswap_check && si.totalswap
	*/
	if (sc->nr_to_scan > 0 &&
		((other_free < lowmem_minfree_notif_trigger &&
		other_file < lowmem_minfree_notif_trigger) ||
		(si.freeswap < lowmem_minswapfree_notif_trigger &&
		lowmem_minfreeswap_check && si.totalswap))) {
			lowmem_notify_killzone_approach();
	}

	selected_oom_adj = min_adj;

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
		if (oom_adj < min_adj) {
			task_unlock(p);
			continue;
		}

 		mm_rss = get_mm_rss(mm);
		mm_counter = get_mm_counter(mm, MM_SWAPENTS);		
		lowmem_print(4, "lowmem_shrink mm_rss %d , mm_counter %d\n", mm_rss, mm_counter);
 
#ifdef CONFIG_ZRAM
		tasksize = mm_rss + mm_counter;
#else		
		tasksize = get_mm_rss(mm);
#endif

		task_unlock(p);
		if (tasksize <= 0)
			continue;
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
		lowmem_print(2, "lowmem_shrink select %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_adj, tasksize);
	}
	if (selected) {
		lowmem_print(1, "lowmem_shrink send sigkill to %d (%s), adj %d, size %d\n",
			     selected->pid, selected->comm,
			     selected_oom_adj, selected_tasksize);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
#ifdef CONFIG_ANDROID_LMK_DEBUG
		if (selected_oom_adj <= 16) {
			dump_header(current, sc->gfp_mask, -1, 0, 0);
			//user_process_meminfo_show();
#ifdef CONFIG_ZRAM
			zram_printlog();
#endif
		}
#endif
		//Improve the priority of killed process can accelerate the process to die,
		//and the process memory would be released quickly
		boost_dying_task_prio(selected);
		force_sig(SIGKILL, selected);
		rem -= selected_tasksize;
	}
	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	read_unlock(&tasklist_lock);
	return rem;
}

static void dump_header(struct task_struct *p, gfp_t gfp_mask, int order,
			struct mem_cgroup *mem, const nodemask_t *nodemask)
{
	task_lock(p);
	pr_warning("%s invoked lowmemorykiller: gfp_mask=0x%x, oom_adj=%d, oom_score_adj=%d\n",
		p->comm, gfp_mask, p->signal->oom_adj, p->signal->oom_score_adj);
	cpuset_print_task_mems_allowed(p);
	task_unlock(p);
	dump_stack();
	//mem_cgroup_print_oom_info(mem, p);
	show_mem(SHOW_MEM_FILTER_NODES);
	dump_tasks(mem, nodemask);
}


static void lowmem_notify_killzone_approach(void)
{
	lowmem_print(5, "lowmem_shrink notify_killzone_approach\n");
	sysfs_notify(lowmem_kobj, NULL, "notify_trigger_active");
}

static ssize_t lowmem_notify_trigger_active_show(struct kobject *k,
		struct kobj_attribute *attr, char *buf)
{
	int other_free, other_file;
	struct sysinfo si = {0};

	lowmem_print(5, "lowmem_shrink notify_trigger_active_show\n");
	get_free_ram(&other_free, &other_file);
	si_swapinfo(&si);
	if ((other_free < lowmem_minfree_notif_trigger &&
		other_file < lowmem_minfree_notif_trigger) ||
		(si.freeswap < lowmem_minswapfree_notif_trigger &&
			lowmem_minfreeswap_check && si.totalswap))
		return snprintf(buf, 3, "1\n");
	else
		return snprintf(buf, 3, "0\n");
}

static struct kobj_attribute lowmem_notify_trigger_active_attr =
	__ATTR(notify_trigger_active, S_IRUGO,
			lowmem_notify_trigger_active_show, NULL);

static struct attribute *lowmem_default_attrs[] = {
	&lowmem_notify_trigger_active_attr.attr,
	NULL,
};

static ssize_t lowmem_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	return kobj_attr->show(k, kobj_attr, buf);
}

static const struct sysfs_ops lowmem_ops = {
	.show = lowmem_show,
};

static void lowmem_kobj_release(struct kobject *kobj)
{
	/* Nothing to be done here */
}

static struct kobj_type lowmem_kobj_type = {
	.release = lowmem_kobj_release,
	.sysfs_ops = &lowmem_ops,
	.default_attrs = lowmem_default_attrs,
};


static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS
};

static int __init lowmem_init(void)
{
	int rc;
	task_free_register(&task_nb);
	register_shrinker(&lowmem_shrinker);

	lowmem_kobj = kzalloc(sizeof(*lowmem_kobj), GFP_KERNEL);
	if (!lowmem_kobj) {
		rc = -ENOMEM;
		goto err;
	}

	rc = kobject_init_and_add(lowmem_kobj, &lowmem_kobj_type,
			mm_kobj, "lowmemkiller");
	if (rc)
		goto err_kobj;

	return 0;

err_kobj:
	kfree(lowmem_kobj);

err:
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);

	return rc;
}

static void __exit lowmem_exit(void)
{
	kobject_put(lowmem_kobj);
	kfree(lowmem_kobj);
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);
}

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(lowmem_minfreeswap_check, lowmem_minfreeswap_check, uint, S_IRUGO | S_IWUSR);
module_param_array_named(lowmem_minfreeswap, lowmem_minfreeswap, uint,
			 &lowmem_minfreeswap_size, S_IRUGO | S_IWUSR);
module_param_named(notify_trigger, lowmem_minfree_notif_trigger, uint,
			 S_IRUGO | S_IWUSR);
module_param_named(notify_trigger_swap, lowmem_minswapfree_notif_trigger, uint,
			 S_IRUGO | S_IWUSR);
module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

