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


/*used for debug*/
#define DEBUG_PRINT	0


#define ENTRY_ORDER	2					/* order==2, 16KB */
#define NODE_LIST_NODE_COUNT	15		/*node counts in node list*/
#define SYS_PAGE_COUNT_THRESHOLD	2	/*sys mem threshold*/
#define NODE_FREE	1
#define NODE_USED	0
#define MAX_ALLOC_COUNT	4294967295
extern void msleep(unsigned int msecs);
extern void dump_stack(void);

/*stats start*/
struct status {
	unsigned long used;
	unsigned long free;
	unsigned long orders[MAX_ORDER];

	unsigned long alloc_peak;
	unsigned long alloc_total_count;
};
/*stats end*/


static unsigned long alloc_flag = 0;
static unsigned long buf_node[NODE_LIST_NODE_COUNT];
static struct workqueue_struct *sprd_16k_alloc_wq = NULL;
static struct work_struct sprd_16k_alloc_work;
static struct mutex pool_lock;
static struct status stats;

static void dumpstack(void)
{
	printk("Process Name: %s, Process Pid: %lu, Parent Name: %s, Parent Pid: %lu\n", 
			current->comm, current->pid, current->parent->comm, current->parent->pid);
	dump_stack();
}

static unsigned long get_first_node(int sprdcondition)
{
	unsigned long i = 0;

	for(; i < NODE_LIST_NODE_COUNT; i++) {
		if((NODE_FREE == sprdcondition) && buf_node[i]) {
			break;
		} else if((NODE_USED == sprdcondition) && (!buf_node[i])) {
			break;
		}
	}

	return i;
}

static int sprd_16k_buffer_init(void)
{
	gfp_t mask = GFP_KERNEL;
	int i = 0;

	memset(buf_node, 0, sizeof(buf_node));
	for(; i < NODE_LIST_NODE_COUNT; i++) {
		buf_node[i] = __get_free_pages(mask, ENTRY_ORDER);

		/*stats start*/
		if(!buf_node[i]) {
			stats.used++;
			printk("__16K__BUFFER__INIT__: Alloc sprd 16k pool failed, no 16k memory in system memory!!!\n");
		} else {
			stats.free++;
		}
		/*stats end*/
	}

	return 0;
}

static unsigned long _16k_get_pageinfo(void)
{
	struct zone *zone;
	int order = ENTRY_ORDER;
	unsigned long page_count_over_16k = 0;

	/*normal zone only*/
	for_each_zone(zone) {
		if(is_normal(zone)) break;
	}

	/*page counts over 16k*/
	for(; order < MAX_ORDER; order++) {
		page_count_over_16k += (zone->free_area[order].nr_free << (order - 2));
		stats.orders[order] = zone->free_area[order].nr_free;
	}

	return page_count_over_16k;
}

static void sprd_16k_alloc_delay(struct work_struct *work)
{
	gfp_t mask = GFP_KERNEL;
	unsigned long p_sub = 0;

	if(SYS_PAGE_COUNT_THRESHOLD > _16k_get_pageinfo()) goto queuework;
#if DEBUG_PRINT
	printk("__16K__ALLOC__DELAY__: work active!!! \
			free node / used node / node16k: %lu / %lu / %lu\n", stats.free, stats.used, _16k_get_pageinfo());
#endif

	p_sub = get_first_node(NODE_USED);
	if(NODE_LIST_NODE_COUNT == p_sub) return;

	buf_node[p_sub] = __get_free_pages(mask, ENTRY_ORDER);

	/*stats start*/
	if(buf_node[p_sub]) {
		stats.used--;
		stats.free++;
	}
	/*stats end*/

queuework:
	if(NODE_LIST_NODE_COUNT == get_first_node(NODE_USED)) return;
	msleep(1000);
	queue_work(sprd_16k_alloc_wq, &sprd_16k_alloc_work);
}

static unsigned long sprd_16k_alloc(void)
{
	unsigned long address = 0;
	unsigned long p_sub = 0;

	mutex_lock(&pool_lock); /*lock*/

	p_sub = get_first_node(NODE_FREE);
	if(NODE_LIST_NODE_COUNT == p_sub) {
		mutex_unlock(&pool_lock); /*unlock*/

		printk("__16K__ALLOC__: 16k buffer empty!!!  free / used / sys // peak / total: %lu / %lu / %lu // %lu / %lu\n", 
				stats.free, stats.used, _16k_get_pageinfo(), stats.alloc_peak, stats.alloc_total_count);
		dumpstack();
		return address;
	}

	address = buf_node[p_sub];
	buf_node[p_sub] = 0;

	mutex_unlock(&pool_lock); /*unlock*/

	/*stats start*/
	stats.free--;
	stats.used++;
	/*stats end*/

	/*feather-weight : reduce "self-work aquire self-pool" counts*/
	if(strncmp(current->comm, "sprd-page-alloc", 13)) {
		queue_work(sprd_16k_alloc_wq, &sprd_16k_alloc_work);

		/*stats start*/
		if(stats.alloc_peak < stats.used) stats.alloc_peak = stats.used;
		(stats.alloc_total_count < MAX_ALLOC_COUNT) ? (stats.alloc_total_count++) : (stats.alloc_total_count = MAX_ALLOC_COUNT);
		/*stats end*/

#if DEBUG_PRINT
		printk("__16K__ALLOC__: allocated from 16k buffer!!! free / used / sys // peak / total: %lu / %lu / %lu // %lu / %lu\n",
				stats.free, stats.used, _16k_get_pageinfo(), stats.alloc_peak, stats.alloc_total_count);
		dumpstack();
#endif
	}

	return address;
}

struct page *sprd_page_alloc(gfp_t gfp_mask, unsigned int order, unsigned long zoneidx)
{
	unsigned long address = 0;
	struct page *page = NULL;

	if((!alloc_flag) || (ENTRY_ORDER != order) || (ZONE_NORMAL != zoneidx)) goto Failed;

	address = sprd_16k_alloc();
	if(!address) goto Failed;

#if defined(WANT_PAGE_VIRTUAL)
	page = container_of((void *)(address), struct page, virtual);
#else
	page = pfn_to_page(PFN_DOWN(__pa(address)));
#endif
	return page;

Failed:
	return NULL;
}

static int sprd_16k_info_show(struct seq_file *m, void *v)
{
	unsigned long pageinfo = _16k_get_pageinfo();
	unsigned long i = 0;

	seq_printf(m,
		"16k pool summery:\n"
		"    status: %s\n"
		"    free node / used node: %lu / %lu\n"
		"    system memory 16k count: %lu\n"
		"    alloc peak count: %lu\n"
		"    alloc total count: %lu\n",
		(alloc_flag ? "open" : "closed"), stats.free, stats.used, pageinfo, stats.alloc_peak, stats.alloc_total_count);

	seq_printf(m, "buddy info:\n");
	for(; i < MAX_ORDER; i++) {
		seq_printf(m, "    orders / number: %lu / %lu\n", i, stats.orders[i]);
	}

	seq_printf(m, "buf_node info:\n");
	for(i = 0; i < NODE_LIST_NODE_COUNT; i++) {
		seq_printf(m, "    subscript / value : %lu / %p\n", i, buf_node[i]);
	}

	return 0;
}

static int sprd_16k_pool_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_16k_info_show, NULL);
}

static int sprd_16k_pool_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
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

static const struct file_operations sprd_16k_info_pool_fops = {
	.open		= sprd_16k_pool_open,
	.read		= seq_read,
	.write		= sprd_16k_pool_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init sprd_16k_pool_init(void)
{
	/*stats start*/
	stats.used = 0;
	stats.free = 0;
	memset(stats.orders, 0, sizeof(stats.orders));
	stats.alloc_peak = 0;
	stats.alloc_total_count = 0;
	/*stats end*/

	/*init mutex*/
	mutex_init(&pool_lock);

	/*init 16k buffer*/
	if(sprd_16k_buffer_init()) {
		printk("__16K__POOL__INIT__: Init Failed!!! \n");
		return -1;
	}

	/*create self workqueue & init work*/
	sprd_16k_alloc_wq = create_workqueue("sprd-page-alloc");
	INIT_WORK(&sprd_16k_alloc_work, sprd_16k_alloc_delay); 
	
	/*create proc file*/
	proc_create("sprd_pages", 0, NULL, &sprd_16k_info_pool_fops);
	return 0;
}

module_init(sprd_16k_pool_init);

