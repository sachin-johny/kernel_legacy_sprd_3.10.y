
#ifndef _PM_DEVICES_H
#define _PM_DEVICES_H

#include <linux/list.h>
#include <linux/platform_device.h>


#define SUSPEND_NONE 0
#define SUSPEND_ENTER 1
#define SUSPEND_CANCEL 2
#define SUSPEND_DONE 3


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
	struct platform_device *pdev;
	int (*suspend)(struct platform_device *pdev, pm_message_t state);
	int (*resume)(struct platform_device *pdev);
};

extern void register_sprd_pm_suspend(struct sprd_pm_suspend *handler);
extern void unregister_sprd_pm_suspend(struct sprd_pm_suspend *handler);
extern int sprd_pm_suspend_canceled(void);

#endif

