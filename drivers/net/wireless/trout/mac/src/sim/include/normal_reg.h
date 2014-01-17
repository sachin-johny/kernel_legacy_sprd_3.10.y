#ifndef _INCLUDE_NORMAL_REG_E_H
#define _INCLUDE_NORMAL_REG_E_H
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "reg_space.h"

enum {
	READ_ONLY = 0U,
	ALL_BIT_WRITEABLE = ~(u32)0U,
};

struct normal_reg;

typedef void (*normal_reg_event_fun_t)(struct normal_reg *reg,
		enum reg_event_type event_type, u32 old_val);

struct normal_reg {
	struct virt_reg reg;

	/* asynchronous event related */
	struct work_struct work;
	u32 old_val;
	normal_reg_event_fun_t event;

	/* register content modified relatied */
	u32 val;
	u32 reset_val;
	u32 write_mask;
	u32 write_clear_mask;

	/* miscellaneous flags */
	u32 cur_event : 2;
	u32 event_mask : 2;
	u32 need_loged : 1;
};

extern void normal_reg_init(struct normal_reg *reg, u32 addr, u32 reset_val);

extern void set_async_event_handler(struct normal_reg *reg, 
		work_func_t event_handler, enum reg_event_type event_mask);

static inline  void set_sync_event_handler(struct normal_reg *reg,
		normal_reg_event_fun_t event, enum reg_event_type event_mask)
{
	reg->event = event;
	reg->event_mask = event_mask;
}

static inline void normal_reg_lock(struct normal_reg *reg)
{
	reg_lock(&reg->reg);
}

static inline void normal_reg_unlock(struct normal_reg *reg)
{
	reg_unlock(&reg->reg);
}

static inline struct normal_reg *virt_reg_to_normal(struct virt_reg *virt_reg)
{
	return container_of(virt_reg, struct normal_reg, reg);
}

static inline
struct normal_reg *find_normal_reg_by_addr(u32 addr)
{
	struct virt_reg *virt_reg = find_reg_by_addr((volatile void *)addr);

	return (virt_reg) ? virt_reg_to_normal(virt_reg) : NULL;
}

extern u32 dv_read_reg(u32 addr);
extern void dv_write_reg(u32 data, u32 addr);
extern u32 dv_set_reg_bit(u32 flags, u32 addr);
extern u32 dv_clear_reg_bit(u32 flags, u32 addr);


static inline int register_normal_reg(struct normal_reg *reg)
{
	return reg_register(&reg->reg);
}

static inline void unregister_normal_reg(struct normal_reg *reg)
{
	reg_unregister(&reg->reg);
}

/* convenient register function with various parameter */
static inline void reg_default_register(struct normal_reg *reg, u32 reg_addr)
{
	normal_reg_init(reg, reg_addr, 0);
	register_normal_reg(reg);
}

static inline void reg_with_reset_val_register(struct normal_reg *reg,
		u32 reg_addr, u32 reset_val)
{
	normal_reg_init(reg, reg_addr, reset_val);
	register_normal_reg(reg);
}

static inline void reg_with_write_mask_register(struct normal_reg *reg,
		u32 reg_addr, u32 write_mask)
{
	normal_reg_init(reg, reg_addr, 0);
	reg->write_mask = write_mask;
	register_normal_reg(reg);
}

static inline void reg_with_sync_event_register(struct normal_reg *reg,
		u32 reg_addr, normal_reg_event_fun_t event_handler,
		u32 write_clear_mask)
{
	normal_reg_init(reg, reg_addr, 0);
	reg->write_clear_mask = write_clear_mask;

	/* in most case we only interested the REG_WRITE event */
	set_sync_event_handler(reg, event_handler, REG_WRITE);
	register_normal_reg(reg);
}

static inline void reg_with_async_event_register(struct normal_reg *reg,
		u32 reg_addr, work_func_t event_handler,
		u32 write_clear_mask)
{
	normal_reg_init(reg, reg_addr, 0);
	reg->write_clear_mask = write_clear_mask;

	/* in most case we only interested the REG_WRITE event */
	set_async_event_handler(reg, event_handler, REG_WRITE);
	register_normal_reg(reg);
}

#endif
