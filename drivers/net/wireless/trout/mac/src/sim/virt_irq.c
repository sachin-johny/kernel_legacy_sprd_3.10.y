#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/module.h>
#include "virt_irq.h"

static struct list_head *get_irq_list(void)
{
	static LIST_HEAD(irq_list);
	return &irq_list;
}

static DEFINE_SPINLOCK(list_lock);
static inline void irq_list_lock(void)
{
	spin_lock(&list_lock);
}

static inline void irq_list_unlock(void)
{
	spin_unlock(&list_lock);
}

static inline void __irq_add(struct virt_irq *irq)
{
	list_add_tail(&irq->node, get_irq_list());
}

static inline void __irq_del(struct virt_irq *irq)
{
	list_del(&irq->node);
}

static inline void irq_lock(struct virt_irq *irq)
{
	if (irqs_disabled()) {
		spin_lock(&irq->lock);
	} else {
		spin_lock_bh(&irq->lock);
	}
}

static inline void irq_unlock(struct virt_irq *irq)
{
	if (irqs_disabled()) {
		spin_unlock(&irq->lock);
	} else {
		spin_unlock_bh(&irq->lock);
	}
}

static inline void get_virt_irq(struct virt_irq *irq)
{
	if (irq) {
		kref_get(&irq->ref_cnt);
	}
}

static void virt_irq_release(struct kref *ref)
{
	struct virt_irq *irq = container_of(ref, struct virt_irq, ref_cnt);
	kfree(irq);
}

static inline void put_virt_irq(struct virt_irq *irq)
{
	if (irq) {
		kref_put(&irq->ref_cnt, virt_irq_release);
	}
}

static int is_irq_prepared(struct virt_irq *irq)
{
	if (irq->irq_prepare) {
		return irq->irq_prepare(irq->param);
	} else {
		/* just considered as no need preparation */
		return 1;
	}
}

static int is_irq_could_schedule(struct virt_irq *irq)
{
	return !irq->scheduled && !irq->disabled;
}

static void schedule_irq(struct virt_irq *irq)
{
	/* hold @irq until irq_work_fun() excute done */
	get_virt_irq(irq);
	tasklet_schedule(&irq->work);
	irq->scheduled = 1;
}

static inline void __flush_pending_irq(struct virt_irq *irq)
{
	if (irq->pending && is_irq_could_schedule(irq)) {
		if (is_irq_prepared(irq)) {
			irq->pending = 0;
			schedule_irq(irq);
		}
	}
}

// TODO: clean debug info
#include "normal_reg.h"
#include "trout_reg_def.h"
static void irq_work_fun(unsigned long __irq)
{
	struct virt_irq *irq = (struct virt_irq *)__irq;
printk(KERN_EMERG "irq done: %d, status: %#08x\n", irq->irqno, dv_read_reg(rCOMM_INT_STAT));
	if (irq->handler) {
		irq->handler(irq->irqno, irq->dev_id);
	}

	irq_lock(irq);
	irq->scheduled = 0;
	__flush_pending_irq(irq);
	irq_unlock(irq);

	put_virt_irq(irq);
}

static void init_virt_irq(struct virt_irq *irq, unsigned int irqno,
		irq_prepare_t irq_prepare, void *param)
{
	irq->irqno = irqno;
	irq->handler = NULL;
	irq->name = NULL;
	irq->flags = 0;
	irq->dev_id = NULL;

	irq->irq_prepare = irq_prepare;
	irq->param = param;

	irq->pending = 0;
	irq->scheduled = 0;
	irq->disabled = 0;

	spin_lock_init(&irq->lock);
	tasklet_init(&irq->work, irq_work_fun, (unsigned long)irq);
	INIT_LIST_HEAD(&irq->node);
	kref_init(&irq->ref_cnt);
}

static struct virt_irq *__find_irq_by_irqno(unsigned int irqno)
{
	struct virt_irq *entry;

	list_for_each_entry(entry, get_irq_list(), node) {
		if (entry->irqno == irqno) {
			return entry;
		}
	}
	return NULL;
}

/**
 * to prevent race condition, this interface find the virtual irq
 * and get it (to prevent unintended release) simultaneously
 */
static struct virt_irq *find_and_get_irq_by_irqno(unsigned int irqno)
{
	struct virt_irq *irq;

	irq_list_lock();
	irq = __find_irq_by_irqno(irqno);
	get_virt_irq(irq);
	irq_list_unlock();

	return irq;
}

int alloc_virt_irq(unsigned int irqno, irq_prepare_t irq_prepare, void *param)
{
	int ret;
	struct virt_irq *irq;

	irq = kmalloc(sizeof(struct virt_irq), GFP_KERNEL);

	if (!irq) {
		return -ENOMEM;
	}

	init_virt_irq(irq, irqno, irq_prepare, param);

	irq_list_lock();
	if (__find_irq_by_irqno(irqno)) {
		printk(KERN_DEBUG "conflict irq, irqno %u\n", irqno);
		ret = -EINVAL;
		kfree(irq);
	} else {
		ret = 0;
		__irq_add(irq);
	}
	irq_list_unlock();

	return ret;
}
EXPORT_SYMBOL(alloc_virt_irq);

int register_virt_irq(unsigned int irqno, irq_handler_t handler,
		unsigned long flags, const char *name, void *dev_id)
{
	int ret;
	struct virt_irq *irq = find_and_get_irq_by_irqno(irqno);

	if (!irq) {
		printk(KERN_DEBUG "irq didn't allocated, irqno %u\n", irqno);
		ret = -ENODEV;
	} else if (irq->handler) {
		/* for now does not support IRQ sharing */
		printk(KERN_DEBUG "irq already registered, irqno %u\n", irqno);
		ret = -EEXIST;
	} else {
		irq_lock(irq);

		irq->handler = handler;
		irq->name = name;
		irq->flags = flags;
		irq->dev_id = dev_id;

		irq_unlock(irq);
		ret = 0;
	}

	put_virt_irq(irq);

	return ret;
}
EXPORT_SYMBOL(register_virt_irq);

void __unregister_virt_irq(struct virt_irq *irq)
{
	irq->handler = NULL;
}

static inline void report_no_exist_irq(unsigned int irqno)
{
	printk(KERN_DEBUG "irq didn't exist, irqno %u\n", irqno);
}

void unregister_virt_irq(unsigned int irqno, void *dev_id)
{
	struct virt_irq *irq;

	irq_list_lock();
	irq = __find_irq_by_irqno(irqno);

	if (irq) {
		__unregister_virt_irq(irq);
	} else {
		report_no_exist_irq(irqno);
	}

	irq_list_unlock();
}
EXPORT_SYMBOL(unregister_virt_irq);

static int is_irq_register(struct virt_irq *irq)
{
	return irq && irq->handler;
}

/**
 * request virtual irq system _SCHEDULE_ the irq; if this irq
 * already be scheduled, this irq set as _PENDING_ to wait for
 * being scheduled in the future
 */
void raise_virt_irq(unsigned int irqno)
{
	struct virt_irq *irq = find_and_get_irq_by_irqno(irqno);

	if (is_irq_register(irq)) {
// TODO: need clear
printk(KERN_EMERG "request to raise irq: %d, mask: %#08x, status: %#08x\n", 
	irq->irqno, dv_read_reg(rCOMM_INT_MASK), dv_read_reg(rCOMM_INT_STAT));
		irq_lock(irq);
		irq->pending = 1;
		__flush_pending_irq(irq);
		irq_unlock(irq);
	} else {
		report_no_exist_irq(irqno);	
	}

	put_virt_irq(irq);
}
EXPORT_SYMBOL(raise_virt_irq);

void flush_pending_irq(unsigned int irqno)
{
	struct virt_irq *irq = find_and_get_irq_by_irqno(irqno);

	if (is_irq_register(irq)) {
		irq_lock(irq);
		__flush_pending_irq(irq);
		irq_unlock(irq);
	} else {
		report_no_exist_irq(irqno);
	}

	put_virt_irq(irq);
}

int disable_virt_irq(unsigned int irqno)
{
	int ret;
	struct virt_irq *irq = find_and_get_irq_by_irqno(irqno);

	if (is_irq_register(irq)) {
		irq_lock(irq);
		irq->disabled = 1;
		irq_unlock(irq);
		ret = 0;
	} else {
		report_no_exist_irq(irqno);
		ret = -EINVAL;
	}

	put_virt_irq(irq);

	return ret;
}
EXPORT_SYMBOL(disable_virt_irq);

int enable_virt_irq(unsigned int irqno)
{
	int ret = 0;
	struct virt_irq *irq = find_and_get_irq_by_irqno(irqno);

	if (is_irq_register(irq)) {
		irq_lock(irq);
		irq->disabled = 0;
		__flush_pending_irq(irq);
		irq_unlock(irq);
	} else {
		report_no_exist_irq(irqno);
		ret = -EINVAL;
	}
	
	put_virt_irq(irq);

	return ret;
}
EXPORT_SYMBOL(enable_virt_irq);

static void __free_virt_irq(struct virt_irq *irq)
{
	__irq_del(irq);
	put_virt_irq(irq);
}

void virt_irq_cleanup(void)
{
	struct virt_irq *entry, *tmp;

	irq_list_lock();
	list_for_each_entry_safe (entry, tmp, get_irq_list(), node) {
		__free_virt_irq(entry);
	}
	irq_list_unlock();
}
EXPORT_SYMBOL(virt_irq_cleanup);