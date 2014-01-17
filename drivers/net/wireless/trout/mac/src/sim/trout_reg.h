#ifndef _TROUT_REG_H
#define _TROUT_REG_H
#include "trout.h"
#include "trout_reg_def.h"

static inline struct normal_reg *get_intr_status_reg(void)
{
	return &get_trout()->trout_reg.intr_status;
}

static inline u32 get_intr_status(void)
{
	u32 ret;
	struct normal_reg *intr_status = get_intr_status_reg();

	normal_reg_lock(intr_status);
	ret = intr_status->val;
	normal_reg_unlock(intr_status);

	return ret;
}

static inline void append_intr_status(u32 flag)
{
	struct normal_reg *intr_status = get_intr_status_reg();

	normal_reg_lock(intr_status);
	intr_status->val |= flag;
	normal_reg_unlock(intr_status);
}

static inline u32 get_intr_mask(void)
{
	u32 ret;
	struct normal_reg *intr_mask = &get_trout()->trout_reg.intr_mask;

	normal_reg_lock(intr_mask);
	ret = intr_mask->val;
	normal_reg_unlock(intr_mask);

	return ret;
}

extern int trout_reg_init(struct trout_reg *trout_reg);

#endif
