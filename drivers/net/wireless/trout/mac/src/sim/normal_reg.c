#include <linux/module.h>
#include "types.h"
#include "normal_reg.h"
#include "dv_log.h"

static u32 normal_reg_read(struct virt_reg *virt_reg)
{
	struct normal_reg *reg = virt_reg_to_normal(virt_reg);
	return reg->val;
}

static void normal_reg_write(u32 data, struct virt_reg *virt_reg)
{
	struct normal_reg *reg = virt_reg_to_normal(virt_reg);

	const u32 a = reg->write_clear_mask;
	const u32 b = reg->write_mask;
	const u32 c = data;
	const u32 d = reg->val;

	/*
	 * according to arithmetic of the Boolean algebra, the Karnaugh map is
	 *     ab 00  01  11 10
	 *    cd
	 *    00   0   0   0  0
	 *    01   1   0   1  1
	 *    11   1   1   0  0
	 *    10   0   1   0  0
	 */
	reg->val = (~a & ~b & d) | (~a & b & c) | (a & ~c & d);
}

static void normal_reg_event(struct virt_reg *virt_reg,
		enum reg_event_type type, u32 old_val)
{
	struct normal_reg *reg = virt_reg_to_normal(virt_reg);

	if (reg->event && (type & reg->event_mask)) {
		reg->event(reg, type, old_val);
	}
}

static void normal_reg_reset(struct virt_reg *virt_reg)
{
	struct normal_reg *reg = virt_reg_to_normal(virt_reg);
	reg->val = reg->reset_val;
}

static void normal_reg_log(struct virt_reg *virt_reg, int log_type)
{
	struct normal_reg *reg = virt_reg_to_normal(virt_reg);

	if (reg->need_loged) {
		u32 log_data[2] = {
			[0] = reg->reg.addr,
			[1] = reg->val,
		};

		DV_LOG_CELL_T log = {
			.type = log_type,
			.dataLen = sizeof(log_data),
			.timeStamp = jiffies,
			.valid = DV_TRUE,
		};

		dvLog(&log, log_data);
	}
}

static void normal_reg_ops_init(struct normal_reg *reg)
{
	struct virt_reg_ops *ops = &reg->reg.ops;

	ops->read = normal_reg_read;
	ops->write = normal_reg_write;
	ops->event = normal_reg_event;
	ops->reset = normal_reg_reset;
	ops->log = normal_reg_log;
}

void normal_reg_init(struct normal_reg *reg, u32 addr, u32 reset_val)
{
	reg_init(&reg->reg, addr);

	normal_reg_ops_init(reg);

	reg->cur_event = 0;
	reg->event = NULL;
	reg->event_mask = 0;

	reg->val = reset_val;
	reg->reset_val = reset_val;
	reg->old_val = reset_val;
	reg->write_mask = ALL_BIT_WRITEABLE;
	reg->write_clear_mask = 0;

	reg->need_loged = 0;
}
EXPORT_SYMBOL(normal_reg_init);

static void async_event_handler_wrap(struct normal_reg *reg,
		enum reg_event_type event_type, u32 old_val)
{
	if (schedule_work_on(0, &reg->work)) {
		reg->old_val = old_val;
		reg->cur_event = event_type;
	}
}

void set_async_event_handler(struct normal_reg *reg, work_func_t event_handler,
		enum reg_event_type event_mask)
{
	reg->event = async_event_handler_wrap;
	reg->event_mask = event_mask;
	INIT_WORK(&reg->work, event_handler);
}
EXPORT_SYMBOL(set_async_event_handler);

u32 dv_read_reg(u32 addr)
{
	u32 ret;
	struct normal_reg *reg = find_normal_reg_by_addr(addr);

	if (reg) {
		normal_reg_lock(reg);
		ret = reg->val;
		normal_reg_unlock(reg);
	} else {
		report_invalid_reg(addr);
		BUG();
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(dv_read_reg);

void dv_write_reg(u32 data, u32 addr)
{
	struct normal_reg *reg = find_normal_reg_by_addr(addr);

	if (reg) {
		normal_reg_lock(reg);
		reg->val = data;
		normal_reg_unlock(reg);
	} else {
		report_invalid_reg(addr);
		BUG();
	}
}
EXPORT_SYMBOL(dv_write_reg);

u32 dv_set_reg_bit(u32 flags, u32 addr)
{
	u32 ret;
	struct normal_reg *reg = find_normal_reg_by_addr(addr);

	if (reg) {
		normal_reg_lock(reg);
		reg->val |= flags;
		ret = reg->val;
		normal_reg_unlock(reg);
	} else {
		report_invalid_reg(addr);
		BUG();
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(dv_set_reg_bit);

u32 dv_clear_reg_bit(u32 flags, u32 addr)
{
	u32 ret;
	struct normal_reg *reg = find_normal_reg_by_addr(addr);

	if (reg) {
		normal_reg_lock(reg);
		reg->val &= ~flags;
		ret = reg->val;
		normal_reg_unlock(reg);
	} else {
		report_invalid_reg(addr);
		BUG();
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(dv_clear_reg_bit);
