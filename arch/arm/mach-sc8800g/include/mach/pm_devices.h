
#ifndef _PM_DEVICES_H
#define _PM_DEVICES_H

#include <linux/list.h>

/* The sprd_pm_suspend structure defines suspend and resume hooks to be called
 * when system enter and exit deep sleep, and a level to control the order. 
 * Suspend handlers are called in low to high level order, resume handlers are
 * called in the opposite order. 
 */
enum {
	SPRD_PM_SUSPEND_LEVEL0 = 50,
	SPRD_PM_SUSPEND_LEVEL1 = 100,
	SPRD_PM_SUSPEND_LEVEL2 = 150,
};
struct sprd_pm_suspend {
	struct list_head link;
	int level;
	int (*suspend)(struct sprd_pm_suspend *h);
	int (*resume)(struct sprd_pm_suspend *h);
};

void register_sprd_pm_suspend(struct sprd_pm_suspend *handler);
void unregister_sprd_pm_suspend(struct sprd_pm_suspend *handler);

#endif

