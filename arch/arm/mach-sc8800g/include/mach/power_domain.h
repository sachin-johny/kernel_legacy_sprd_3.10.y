/*
  *  Copyright (C) 2010 Spreadtrum Inc.
  *
  *   Wang Liwei.  <levee.wang@spreadtrum.com>
  *
  *
  */
#ifndef __ARCH_ARM_SC8800G2_POWER_DOMAIN_H
#define __ARCH_ARM_SC8800G2_POWER_DOMAIN_H

#include <linux/list.h>

#define ENABLE_ON_INIT	(0x1UL << 0)

struct powerdomain {

	const char *name;
	u32 flags;
	struct list_head node;
};

void pwrdm_init(struct powerdomain **pwrdm_list);
int pwrdm_register(struct powerdomain *pwrdm);

#endif

