#include <linux/module.h>
#include "reg_space.h"
#include "virt_irq.h"
#include "virt_bus.h"
#include "trout.h"
#include "share_mem.h"

int hwSimInit(void)
{
	printk(KERN_NOTICE "hw sim init\n");
	struct_checker();
	trout_init();
	return 0;
}

void hwSimExit(void)
{
	trout_cleanup();
	reg_space_cleanup();
	virt_irq_cleanup();
	virt_bus_cleanup();
	printk(KERN_NOTICE "hw sim exit\n");
}
