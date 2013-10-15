/* linux/arch/arm/mach-sc8810/sprd_mem_pool.c
 *
 * Copyright (C) 2010 Spreadtrum
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/pid.h>
#include <linux/mutex.h>
#include <asm/current.h>
#include <asm/uaccess.h>
#include <linux/pagemap.h>
#include <linux/pfn.h>
#include <linux/memory.h>

/*extern functions*/
extern void msleep(unsigned int msecs);
extern void dump_stack(void);

/*used for debug*/
#define DEBUG_PRINT	0

/*magic numbers*/
#define SYS_PAGE_THRESHOLD_8K 2
#define SYS_PAGE_THRESHOLD_16K 2
#define MAX_ALLOC_COUNT	0xffffffff

/*alloc orders*/
typedef enum {
	ENTRY_4K,
	ENTRY_8K,
	ENTRY_16K,
	ENTRY_COUNT
}page_order;

/*node status*/
typedef enum {
	NODE_USED,
	NODE_FREE
}node_status;

/*node counts*/
typedef enum {
	LIST_8K = 10,
	LIST_16K = 30
}node_count;

/*stats record*/
struct status {
	unsigned long used_8k;
	unsigned long free_8k;
	unsigned long alloc_peak_8k;
	unsigned long alloc_total_8k;

	unsigned long used_16k;
	unsigned long free_16k;
	unsigned long alloc_peak_16k;
	unsigned long alloc_total_16k;

	unsigned long orders[MAX_ORDER];
};

/*global variables*/
static unsigned long alloc_flag = 1;
static unsigned long node_list_8k[LIST_8K];
static unsigned long node_list_16k[LIST_16K];
static struct workqueue_struct *sprd_alloc_wq = NULL;
static struct work_struct sprd_alloc_work_16k;
static struct work_struct sprd_alloc_work_8k;
static struct mutex list_lock_8k;
static struct mutex list_lock_16k;
static struct status stats;

/*function start*/
static void dumpstack(void)
{
	printk("Process Name: %s, Process Pid: %d, Parent Name: %s, Parent Pid: %d\n", 
			current->comm, current->pid, current->parent->comm, current->parent->pid);
	dump_stack();
}

static struct page *address_to_pages(unsigned long address)
{
	if(!address) return NULL;

#if defined(WANT_PAGE_VIRTUAL)
	return container_of((void *)(address), struct page, virtual);
#else
	return virt_to_page(address);
#endif
}

static unsigned long get_first_node(int sprdcondition, unsigned int order)
{
	unsigned long i = 0;
	unsigned long end = 0;
	unsigned long *node_list = NULL;

	if(ENTRY_8K == order) {
		end = LIST_8K;
		node_list = node_list_8k;
	} else if(ENTRY_16K == order) {
		end = LIST_16K;
		node_list = node_list_16k;
	}

	for(i = 0; i < end; i++) {
		if((NODE_FREE == sprdcondition) && node_list[i]) {
			break;
		} else if((NODE_USED == sprdcondition) && (!node_list[i])) {
			break;
		}
	}

	return i;
}

static int sprd_node_list_init(void)
{
	gfp_t mask = GFP_KERNEL;
	int i = 0;

	memset(node_list_8k, 0, sizeof(node_list_8k));
	memset(node_list_16k, 0, sizeof(node_list_16k));
	for(i = 0; i < LIST_8K; i++) {
		node_list_8k[i] = __get_free_pages(mask, ENTRY_8K);

		/*stats record*/
		node_list_8k[i] ? stats.free_8k++ : stats.used_8k++;
	}

	for(i = 0; i < LIST_16K; i++) {
		node_list_16k[i] = __get_free_pages(mask, ENTRY_16K);

		/*stats record*/
		node_list_16k[i] ? stats.free_16k++ : stats.used_16k++;
	}

	printk("__SPRD__NODE_LIST__INIT__: Init Succeed!!!\n");
	return 0;
}

static unsigned long get_page_count(unsigned int entry_order)
{
	struct zone *zone;
	unsigned int order = 0;
	unsigned long page_count = 0;

	/*normal zone only*/
	for_each_zone(zone) {
		if(is_normal(zone)) break;
	}

	/*get page counts*/
	for(order = 0; order < MAX_ORDER; order++) {
		/*stats record*/
		stats.orders[order] = zone->free_area[order].nr_free;

		if(order < entry_order) continue;
		page_count += (zone->free_area[order].nr_free << (order - entry_order));
	}

	return page_count;
}

static void sprd_alloc_delay_8k(struct work_struct *work)
{
	gfp_t mask = GFP_KERNEL;
	unsigned long p_sub = 0;

	if(SYS_PAGE_THRESHOLD_8K > get_page_count(ENTRY_8K)) goto queuework;

#if DEBUG_PRINT
	printk("__SPRD__ALLOC__DELAY__8K__: work active!!! \
			free node 8k / used node 8k / 8k in sys: %lu / %lu / %lu\n", stats.free_8k, stats.used_8k, get_page_count(ENTRY_8K));
#endif

	p_sub = get_first_node(NODE_USED, ENTRY_8K);
	if(LIST_8K == p_sub) return;

	node_list_8k[p_sub] = __get_free_pages(mask, ENTRY_8K);

	/*stats record*/
	if(node_list_8k[p_sub]) {
		stats.used_8k--;
		stats.free_8k++;
	}

queuework:
	if(LIST_8K == get_first_node(NODE_USED, ENTRY_8K)) return;
	msleep(1000);
	queue_work(sprd_alloc_wq, &sprd_alloc_work_8k);
}

static void sprd_alloc_delay_16k(struct work_struct *work)
{
	gfp_t mask = GFP_KERNEL;
	unsigned long p_sub = 0;

	if(SYS_PAGE_THRESHOLD_16K > get_page_count(ENTRY_16K)) goto queuework;

#if DEBUG_PRINT
	printk("__SPRD__ALLOC__DELAY__16K__: work active!!! \
			free node 16k / used node 16k / 16k in sys: %lu / %lu / %lu\n", stats.free_16k, stats.used_16k, get_page_count(ENTRY_16K));
#endif

	p_sub = get_first_node(NODE_USED, ENTRY_16K);
	if(LIST_16K == p_sub) return;

	node_list_16k[p_sub] = __get_free_pages(mask, ENTRY_16K);

	/*stats record*/
	if(node_list_16k[p_sub]) {
		stats.used_16k--;
		stats.free_16k++;
	}

queuework:
	if(LIST_16K == get_first_node(NODE_USED, ENTRY_16K)) return;
	msleep(1000);
	queue_work(sprd_alloc_wq, &sprd_alloc_work_16k);
}

static unsigned long sprd_one_node_alloc(unsigned int order)
{
	unsigned long address = 0;
	unsigned long p_sub = 0;
	unsigned long end = 0;
	unsigned long *node_list = NULL;
	struct work_struct *sprd_alloc_work = NULL;
	struct mutex *lock = NULL;

#if DEBUG_PRINT
	printk("__SPRD__: lock addr 16k = %p, lock addr 8k = %p\n", &list_lock_16k, &list_lock_8k);
#endif

	if(ENTRY_8K == order) {
		end = LIST_8K;
		lock = &list_lock_8k;
		node_list = node_list_8k;
		sprd_alloc_work = &sprd_alloc_work_8k;
	} else if(ENTRY_16K == order) {
		end = LIST_16K;
		lock = &list_lock_16k;
		node_list = node_list_16k;
		sprd_alloc_work = &sprd_alloc_work_16k;
	}

	if(!lock || !node_list) return address;

	mutex_lock(lock); /*lock*/

	p_sub = get_first_node(NODE_FREE, order);
	if(end == p_sub) {
		mutex_unlock(lock); /*unlock*/

		printk("__SPRD__ALLOC__: node list empty!!!\n");

#if DEBUG_PRINT
		if(ENTRY_8K == order) {
			printk("__SPRD__ALLOC__: free_8k / used_8k / sys // peak_8k / total_8k: %lu / %lu / %lu // %lu / %lu\n", 
					stats.free_8k, stats.used_8k, get_page_count(ENTRY_8K), stats.alloc_peak_8k, stats.alloc_total_8k);
		} else if(ENTRY_16K == order) {
			printk("__SPRD__ALLOC__: free_16k / used_16k / sys // peak_16k / total_16k: %lu / %lu / %lu // %lu / %lu\n", 
					stats.free_16k, stats.used_16k, get_page_count(ENTRY_16K), stats.alloc_peak_16k, stats.alloc_total_16k);
		}
#endif

		dumpstack();

		return address;
	}

	address = node_list[p_sub];
	node_list[p_sub] = 0;

	mutex_unlock(lock); /*unlock*/

	/*stats record*/
	if(ENTRY_8K == order) {
		stats.free_8k--;
		stats.used_8k++;
	} else if(ENTRY_16K == order) {
		stats.free_16k--;
		stats.used_16k++;
	}

	/*feather-weight : reduce "self-work aquire self-pool" counts*/
	if(strncmp(current->comm, "sprd-page-alloc", 13)) {
		queue_work(sprd_alloc_wq, sprd_alloc_work);

		/*stats record*/
		if(ENTRY_8K == order) {
			if(stats.alloc_peak_8k < stats.used_8k) stats.alloc_peak_8k = stats.used_8k;
			if(stats.alloc_total_8k < MAX_ALLOC_COUNT) stats.alloc_total_8k++;
		} else if(ENTRY_16K == order) {
			if(stats.alloc_peak_16k < stats.used_16k) stats.alloc_peak_16k = stats.used_16k;
			if(stats.alloc_total_16k < MAX_ALLOC_COUNT) stats.alloc_total_16k++;
		}

#if DEBUG_PRINT
		printk("__SPRD__ALLOC__: allocated from node list!!!\n");
		if(ENTRY_8K == order) {
			printk("__SPRD__ALLOC__: free_8k / used_8k / sys // peak_8k / total_8k: %lu / %lu / %lu // %lu / %lu\n", 
					stats.free_8k, stats.used_8k, get_page_count(ENTRY_8K), stats.alloc_peak_8k, stats.alloc_total_8k);
		} else if(ENTRY_16K == order) {
			printk("__SPRD__ALLOC__: free_16k / used_16k / sys // peak_16k / total_16k: %lu / %lu / %lu // %lu / %lu\n", 
					stats.free_16k, stats.used_16k, get_page_count(ENTRY_16K), stats.alloc_peak_16k, stats.alloc_total_16k);
		}
		dumpstack();
#endif

	}

	return address;
}

struct page *sprd_page_alloc(gfp_t gfp_mask, unsigned int order, unsigned long zoneidx)
{
	unsigned long address = 0;
	struct page *page = NULL;

#if DEBUG_PRINT
	printk("__SPRD__INNNNN: alloc_flag = %lu, zoneidx = %lu, order = %u\n", alloc_flag, zoneidx, order);
#endif

	if((!alloc_flag) || (GFP_KERNEL != gfp_mask) || (ZONE_NORMAL != zoneidx) || ((ENTRY_8K != order) && (ENTRY_16K != order))) goto Failed;

	address = sprd_one_node_alloc(order);
	if(!address) goto Failed;

	page = address_to_pages(address);
	printk("__SPRD_PAGE_ALLOC: Succeed alloc page = %p\n", page);

	return page;

Failed:
	return NULL;
}

static int sprd_show_pages_info(struct seq_file *m, void *v)
{
	unsigned long pageinfo_16k = get_page_count(ENTRY_16K);
	unsigned long pageinfo_8k = get_page_count(ENTRY_8K);
	unsigned long i = 0;

	seq_printf(m,
		"sprd pool summery:\n"
		"    status: %s\n"
		"    free node 8k / used node 8k: %lu / %lu\n"
		"    free node 16k / used node 16k: %lu / %lu\n"
		"    system memory 8k count: %lu\n"
		"    system memory 16k count: %lu\n"
		"    alloc peak 8k / alloc total 8k: %lu / %lu\n"
		"    alloc peak 16k / alloc total 16k: %lu / %lu\n",
		(alloc_flag ? "open" : "closed"), stats.free_8k, stats.used_8k, stats.free_16k, stats.used_16k,
		pageinfo_8k, pageinfo_16k, stats.alloc_peak_8k, stats.alloc_total_8k, stats.alloc_peak_16k, stats.alloc_total_16k);

	seq_printf(m, "buddy info:\n");
	for(; i < MAX_ORDER; i++) {
		seq_printf(m, "    orders / number: %lu / %lu\n", i, stats.orders[i]);
	}

	seq_printf(m, "node_list_8k info:\n");
	for(i = 0; i < LIST_8K; i++) {
		struct page *page = NULL;
		page = address_to_pages(node_list_8k[i]);
		seq_printf(m, "    subscript / value / page / page->flags: %lu / %p / %p / %lx\n",
				i, (void *)(node_list_8k[i]), page, (page ? page->flags : 0x0));
	}
	seq_printf(m, "node_list_16k info:\n");
	for(i = 0; i < LIST_16K; i++) {
		struct page *page = NULL;
		page = address_to_pages(node_list_16k[i]);
		seq_printf(m, "    subscript / value / page / page->flags: %lu / %p / %p / %lx\n",
				i, (void *)(node_list_16k[i]), page, (page ? page->flags : 0x0));
	}

	return 0;
}

static int sprd_pages_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_show_pages_info, NULL);
}

static int sprd_pages_info_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
	char flag;

	if (copy_from_user(&flag, buf, 1)) return -EFAULT;
	
	switch (flag) {
		case '0': alloc_flag = 0; break;
		case '1': alloc_flag = 1; break;
		default: break;
	}

	return 1;
}

static const struct file_operations sprd_page_info_fops = {
	.open		= sprd_pages_info_open,
	.read		= seq_read,
	.write		= sprd_pages_info_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init sprd_pages_init(void)
{
	/*init stats record*/
	stats.used_8k = 0;
	stats.free_8k = 0;
	stats.used_16k = 0;
	stats.free_16k = 0;
	memset(stats.orders, 0, sizeof(stats.orders));
	stats.alloc_peak_8k = 0;
	stats.alloc_total_8k = 0;
	stats.alloc_peak_16k = 0;
	stats.alloc_total_16k = 0;

	/*init mutex*/
	mutex_init(&list_lock_8k);
	mutex_init(&list_lock_16k);

	/*init pages node*/
	if(sprd_node_list_init()) {
		printk("__SPRD__NODE_LIST__INIT__: Init Failed!!! \n");
		return -1;
	}

	/*create self workqueue & init work*/
	sprd_alloc_wq = create_workqueue("sprd-page-alloc");
	INIT_WORK(&sprd_alloc_work_8k, sprd_alloc_delay_8k); 
	INIT_WORK(&sprd_alloc_work_16k, sprd_alloc_delay_16k); 
	
	/*create proc file*/
	proc_create("sprd_pages", 0, NULL, &sprd_page_info_fops);
	return 0;
}

module_init(sprd_pages_init);

