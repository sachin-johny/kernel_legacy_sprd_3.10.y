#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/pm_devices.h>
#include <linux/wakelock.h>

static DEFINE_MUTEX(sprd_pm_suspend_lock);
static LIST_HEAD(sprd_pm_suspend_handlers);

long has_wake_lock_for_suspend(int type);


void register_sprd_pm_suspend(struct sprd_pm_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&sprd_pm_suspend_lock);
	list_for_each(pos, &sprd_pm_suspend_handlers) {
		struct sprd_pm_suspend *e;
		e = list_entry(pos, struct sprd_pm_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&sprd_pm_suspend_lock);
}
EXPORT_SYMBOL(register_sprd_pm_suspend);

void unregister_sprd_pm_suspend(struct sprd_pm_suspend *handler)
{
	mutex_lock(&sprd_pm_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&sprd_pm_suspend_lock);
}
EXPORT_SYMBOL(unregister_sprd_pm_suspend);

int sprd_pm_suspend(void)
{
	int error = 0;
	struct sprd_pm_suspend *pos;

	list_for_each_entry(pos, &sprd_pm_suspend_handlers, link) {
		if (pos->suspend != NULL)
			error |= pos->suspend(pos);
	}

	return error;
}

int sprd_pm_resume(void)
{
	struct sprd_pm_suspend *pos;

	list_for_each_entry_reverse(pos, &sprd_pm_suspend_handlers, link)
		if (pos->resume != NULL)
			pos->resume(pos);
	return 0;
}

int sprd_pm_suspend_check_enter(void)
{
	if (!has_wake_lock_for_suspend(WAKE_LOCK_SUSPEND)) {
		sprd_pm_suspend();
	}

	return 0;
}


