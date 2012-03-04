/* kernel/arch/arm/mach_sc../pm_wakesource.c
 *
 * Copyright (C) 2001-2011 Spreadtrum, Inc.
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

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <mach/pm_wakesource.h>

static DEFINE_SPINLOCK(wake_source_lock);
static LIST_HEAD(wake_source_handlers);

void register_wake_source(struct wake_source *handler)
{
	unsigned long flags;
	spin_lock_irqsave(&wake_source_lock,flags);
	list_add_tail(&handler->link, &wake_source_handlers);
	spin_unlock_irqrestore(&wake_source_lock,flags);
}
EXPORT_SYMBOL(register_wake_source);

void unregister_wake_source(struct wake_source *handler)
{
	unsigned long flags;
	spin_lock_irqsave(&wake_source_lock,flags);
	list_del(&handler->link);
	spin_unlock_irqrestore(&wake_source_lock,flags);
}
EXPORT_SYMBOL(unregister_wake_source);

void wake_source_set(void)
{
	unsigned long flags;
	struct wake_source *pos;

	spin_lock_irqsave(&wake_source_lock,flags);

	list_for_each_entry(pos, &wake_source_handlers, link) {
		if (pos->set != NULL)
			pos->set(pos);
	}
	spin_unlock_irqrestore(&wake_source_lock,flags);
}
EXPORT_SYMBOL(wake_source_set);

void wake_source_clr(void)
{
	unsigned long flags;
	struct wake_source *pos;

	spin_lock_irqsave(&wake_source_lock,flags);

	list_for_each_entry_reverse(pos, &wake_source_handlers, link){
		if (pos->clear != NULL)
			pos->clear(pos);
       }
	spin_unlock_irqrestore(&wake_source_lock,flags);
}
EXPORT_SYMBOL(wake_source_clr);
