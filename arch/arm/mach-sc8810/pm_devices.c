#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/pm_devices.h>
#include <linux/wakelock.h>
#include <linux/pm.h>

static DEFINE_MUTEX(sprd_pm_suspend_lock);
static LIST_HEAD(sprd_pm_suspend_handlers);

int suspend_status = SUSPEND_NONE;
long has_wake_lock_for_suspend(int type);


void register_sprd_pm_suspend_func(struct sprd_pm_suspend *handler)
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

EXPORT_SYMBOL(register_sprd_pm_suspend_func);

void unregister_sprd_pm_suspend_func(struct sprd_pm_suspend *handler)
{
	mutex_lock(&sprd_pm_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&sprd_pm_suspend_lock);
}
EXPORT_SYMBOL(unregister_sprd_pm_suspend_func);

int sprd_pm_suspend(void)
{
	int error = 0;
	int ret_val = 0;
	struct sprd_pm_suspend *pos;

	list_for_each_entry(pos, &sprd_pm_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			ret_val = pos->suspend(pos->pdev, PMSG_SUSPEND);
			if (ret_val) {
				printk("##: suspend is blocked by %s.\n", pos->file);
			}
			error |= ret_val;
		}
	}

	return error;
}

int sprd_pm_resume(void)
{
	struct sprd_pm_suspend *pos;

	list_for_each_entry_reverse(pos, &sprd_pm_suspend_handlers, link)
		if (pos->resume != NULL)
			pos->resume(pos->pdev);
	return 0;
}

int sprd_pm_suspend_check_enter(void)
{
	int error = 0;
	suspend_status = SUSPEND_NONE;
	if (!has_wake_lock_for_suspend(WAKE_LOCK_SUSPEND)) {
		error = sprd_pm_suspend();
		suspend_status = SUSPEND_ENTER;
		if (error) {
			suspend_status = SUSPEND_CANCEL;
		}
		else {
			suspend_status = SUSPEND_DONE;
		}
	}

	return error;
}

int sprd_pm_resume_check(void)
{
	switch(suspend_status) {
		case SUSPEND_NONE:
			break;
		case SUSPEND_ENTER:
		case SUSPEND_CANCEL:
		case SUSPEND_DONE:
			sprd_pm_resume();
			break;
		default:
			printk("##: unknown suspend_status value!\n");
			break;
	}
	return 0;
}

int sprd_pm_suspend_canceled(void)
{
	sprd_pm_suspend_check_enter();
	return (SUSPEND_CANCEL == suspend_status);
}

