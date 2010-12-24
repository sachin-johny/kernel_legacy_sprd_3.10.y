/*
 *    Copyright (C)  2010 Spreadtrum Inc.
 *
 *    power domain framework for SC8800G2.
 *
 *    Wang Liwei.   <levee.wang@spreadtrum.com>
 *
 *
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>


#include <mach/power_domain.h>


static LIST_HEAD(pwrdm_list);
static DEFINE_RWLOCK(pwrdm_rwlock);

static __init void _pwrdm_setup(struct powerdomain *pwrdm)
{

}

static struct powerdomain *_pwrdm_lookup(const char *name)
{
	struct powerdomain *pwrdm, *temp_pwrdm;

	pwrdm = NULL;

	list_for_each_entry(temp_pwrdm, &pwrdm_list, node) {
		if (!strcmp(name, temp_pwrdm->name)) {
			pwrdm = temp_pwrdm;
			break;
		}
	}
	return pwrdm;
}

void pwrdm_init(struct powerdomain **pwrdm_list)
{
	struct powerdomain **p = NULL;

	if (pwrdm_list) {
		for (p = pwrdm_list; *p; p++) {
			pwrdm_register(*p);
			_pwrdm_setup(*p);
		}
	}
}

int pwrdm_register(struct powerdomain *pwrdm)
{
	unsigned long flags;
	int ret = -EINVAL;

	if (!pwrdm)
		return -EINVAL;

	write_lock_irqsave(&pwrdm_rwlock, flags);
	if (_pwrdm_lookup(pwrdm->name)) {
		ret = -EEXIST;
		goto unlock;
	}
	list_add(&pwrdm->node, &pwrdm_list);
	ret = 0;

unlock:
	write_unlock_irqrestore(&pwrdm_rwlock, flags);

	return ret;
}

int pwrdm_unregister(struct powerdomain *pwrdm)
{
	unsigned long flags;

	if (!pwrdm)
		return -EINVAL;

	write_lock_irqsave(&pwrdm_rwlock, flags);
	list_del(&pwrdm->node);
	write_unlock_irqrestore(&pwrdm_rwlock, flags);

	return 0;
}


