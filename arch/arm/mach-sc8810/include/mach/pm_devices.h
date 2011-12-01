
#ifndef _PM_DEVICES_H
#define _PM_DEVICES_H

#include <linux/list.h>
#include <linux/device.h>


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
	struct device *pdev;
	int (*suspend)(struct device *pdev, pm_message_t state);
	int (*resume)(struct device *pdev);
	/* for debugging. */
	char *file;
	int line;
	
};


#define register_sprd_pm_suspend(handler)	do {		\
		struct sprd_pm_suspend *h = handler;	\
		h->file = __FILE__;		\
		h->line = __LINE__;	\
		register_sprd_pm_suspend_func(h);     \
} while (0)

#define unregister_sprd_pm_suspend(handler)	do {		\
		unregister_sprd_pm_suspend_func(handler);     \
} while (0)

extern void register_sprd_pm_suspend_func(struct sprd_pm_suspend *handler);
extern void unregister_sprd_pm_suspend_func(struct sprd_pm_suspend *handler);
extern int sprd_pm_suspend_canceled(void);

#endif

