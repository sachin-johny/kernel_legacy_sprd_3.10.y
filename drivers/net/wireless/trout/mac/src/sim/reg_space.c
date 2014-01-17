#include <linux/list.h>
#include <linux/module.h>
#include "types.h"
#include "reg_space.h"
#include "dv_log.h"

static inline struct list_head *get_reg_list(void)
{
	static LIST_HEAD(reg_list);
	return &reg_list;
}

static DEFINE_SPINLOCK(list_lock);
static inline void reg_list_lock(void)
{
	spin_lock(&list_lock);
}

static inline void reg_list_unlock(void)
{
	spin_unlock(&list_lock);
}

static void reg_ops_init(struct virt_reg_ops *ops)
{
	ops->read = NULL;
	ops->write = NULL;
	ops->event = NULL;
	ops->reset = NULL;
	ops->log = NULL;
}

struct virt_reg *reg_init(struct virt_reg *reg, u32 addr)
{
	reg->addr = addr;
	INIT_LIST_HEAD(&reg->node);
	spin_lock_init(&reg->lock);
	reg_ops_init(&reg->ops);

	return reg;
}
EXPORT_SYMBOL(reg_init);

static struct virt_reg *__find_reg_by_addr(u32 addr)
{
	struct virt_reg *entry;

	list_for_each_entry(entry, get_reg_list(), node) {
		if (entry->addr == addr) {
			return entry;
		}
	}

	return NULL;
}

int reg_register(struct virt_reg *reg)
{
	int ret;

	reg_list_lock();
	if (!__find_reg_by_addr(reg->addr)) {
		list_add_tail(&reg->node, get_reg_list());
		ret = 0;
	} else {
		printk(KERN_DEBUG "conflict register, addr %#.08x\n", reg->addr);
		ret = -EEXIST;
	}
	reg_list_unlock();

	return ret;
}
EXPORT_SYMBOL(reg_register);

static inline void __reg_unregister(struct virt_reg *reg)
{
	list_del(&reg->node);
}

void reg_unregister(struct virt_reg *reg)
{
	reg_list_lock();
	__reg_unregister(reg);
	reg_list_unlock();
}
EXPORT_SYMBOL(reg_unregister);

static u32 __read_reg(struct virt_reg *reg)
{
	BUG_ON(!reg->ops.read);
	return reg->ops.read(reg);
}

static void __write_reg(u32 data, struct virt_reg *reg)
{
	if (reg->ops.write) {
		reg->ops.write(data, reg);
	}
}

static void __log_reg_event(struct virt_reg *reg, int log_type)
{
	if (reg->ops.log) {
		reg->ops.log(reg, log_type);
	}
}

static void __reg_event(struct virt_reg *reg,
		enum reg_event_type event_type, u32 old_val)
{
	if (reg->ops.event) {
		reg->ops.event(reg, event_type, old_val);
	}
}

static u32 do_de_read_reg(struct virt_reg *reg)
{
	u32 ret;

	reg_lock(reg);
	ret = __read_reg(reg);
	__log_reg_event(reg, DV_LOG_TYPE_REG_RD);
	__reg_event(reg, REG_READ, ret);
	reg_unlock(reg);

	return ret;
}

u32 de_read_reg(const volatile void *addr)
{
	struct virt_reg *reg = find_reg_by_addr(addr);

	if (reg) {
		return do_de_read_reg(reg);
	} else {
		report_invalid_reg((u32)addr);
		return 0;
	}
}
EXPORT_SYMBOL(de_read_reg);

static void do_de_write_reg(u32 data, struct virt_reg *reg)
{
	u32 old_val;

	reg_lock(reg);
	old_val = __read_reg(reg);
	__write_reg(data, reg);
	__log_reg_event(reg, DV_LOG_TYPE_REG_WT);
	__reg_event(reg, REG_WRITE, old_val);
	reg_unlock(reg);
}

void de_write_reg(u32 data, volatile void *addr)
{
	struct virt_reg *reg = find_reg_by_addr(addr);

	if (reg) {
		do_de_write_reg(data, reg);
	} else {
		report_invalid_reg((u32)addr);
	}
}
EXPORT_SYMBOL(de_write_reg);

struct virt_reg *find_reg_by_addr(const volatile void *addr)
{
	struct virt_reg *reg;

	reg_list_lock();
	reg = __find_reg_by_addr((u32)addr);
	reg_list_unlock();

	return reg;
}
EXPORT_SYMBOL(find_reg_by_addr);

static void reg_reset(struct virt_reg *reg)
{
	reg_lock(reg);
	if (reg->ops.reset) {
		reg->ops.reset(reg);
	}
	reg_unlock(reg);
}

void reg_space_reset(void)
{
	struct virt_reg *entry;

	reg_list_lock();
	list_for_each_entry(entry, get_reg_list(), node) {
		reg_reset(entry);
	}
	reg_list_unlock();

	printk(KERN_DEBUG "reg space reset complete\n");
}
EXPORT_SYMBOL(reg_space_reset);

void reg_space_cleanup(void)
{
	struct virt_reg *entry, *tmp;

	reg_list_lock();
	list_for_each_entry_safe(entry, tmp, get_reg_list(), node) {
		__reg_unregister(entry);
	}
	reg_list_unlock();
}
EXPORT_SYMBOL(reg_space_cleanup);

