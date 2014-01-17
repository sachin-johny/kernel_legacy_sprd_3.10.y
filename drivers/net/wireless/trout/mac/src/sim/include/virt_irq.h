#ifndef _INCLUDE_VIRT_IRQ_H
#define _INCLUDE_VIRT_IRQ_H
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kref.h>

typedef int (*irq_prepare_t)(void *);

struct virt_irq {
	/* ISR related parameter */
	unsigned int irqno;
	irq_handler_t handler;
	unsigned long flags;
	const char *name;
	void *dev_id;

	spinlock_t lock;

	/* be callback before ISR excute */
	irq_prepare_t irq_prepare;
	void *param;

	/* internal used member, don't touch directed */
	struct tasklet_struct work;
	struct list_head node;
	struct kref ref_cnt;
	u32 pending : 1;
	u32 scheduled : 1;
	u32 disabled : 1;
};

extern int alloc_virt_irq(unsigned int irqno, irq_prepare_t irq_prepare,
		void *param);

extern int register_virt_irq(unsigned int irqno, irq_handler_t handler, 
		unsigned long flags, const char *name, void *dev_id);

extern void unregister_virt_irq(unsigned int irqno, void *dev_id);

extern void raise_virt_irq(unsigned int irqno);

extern void flush_pending_irq(unsigned int irqno);

extern int disable_virt_irq(unsigned int irqno);

extern int enable_virt_irq(unsigned int irqno);

extern void virt_irq_cleanup(void);

#endif
