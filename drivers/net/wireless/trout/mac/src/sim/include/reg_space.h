#ifndef _SIM_INCLUDE_REG_SPACE_H
#define _SIM_INCLUDE_REG_SPACE_H
#include <linux/types.h>
#include <linux/spinlock.h>

enum reg_event_type {
	REG_READ = (1U<<0),
	REG_WRITE = (1U<<1),
	REG_RDWR = REG_READ | REG_WRITE,
};

struct virt_reg;

typedef u32 (*read_fun_t)(struct virt_reg *);
typedef void (*write_fun_t)(u32, struct virt_reg *);
typedef void (*event_fun_t)(struct virt_reg *, enum reg_event_type, u32);
typedef void (*reset_fun_t)(struct virt_reg *);
typedef void (*log_fun_t)(struct virt_reg *, int);

/**
 * ATTENTION: all the operations be called with virt_reg's @lock held
 */
struct virt_reg_ops {
	read_fun_t read;
	write_fun_t write;
	event_fun_t event;
	reset_fun_t reset;
	log_fun_t log;
};

struct virt_reg {
	u32 addr;
	struct list_head node;
	spinlock_t lock;
	struct virt_reg_ops ops;
};

/**
 * ATTENTION: the implementation of virtual irq used the tasklet, to prevent
 * the ISR operate virtual register and cause deadlock, we need diable BH
 */
static inline void reg_lock(struct virt_reg *reg)
{
	if (irqs_disabled()) {
		spin_lock(&reg->lock);
	} else {
		spin_lock_bh(&reg->lock);
	}
}

static inline void reg_unlock(struct virt_reg *reg)
{
	if (irqs_disabled()) {
		spin_unlock(&reg->lock);
	} else {
		spin_unlock_bh(&reg->lock);
	}
}

static inline void report_invalid_reg(u32 addr)
{
	//printk(KERN_DEBUG "invalid register address: %#.08x\n", addr);
}

extern struct virt_reg *reg_init(struct virt_reg *reg, u32 addr);
extern int reg_register(struct virt_reg *reg);
extern void reg_unregister(struct virt_reg *reg);

extern u32 de_read_reg(const volatile void *addr);
extern void de_write_reg(u32 data, volatile void *addr);

extern struct virt_reg *find_reg_by_addr(const volatile void *addr);

extern void reg_space_reset(void);
extern void reg_space_cleanup(void);

#endif

