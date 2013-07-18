#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/busmonitor.h>
#include <mach/sci_glb_regs.h>

static void bm_callback(void)
{
	static int count = 0;
	printk("%s %d %x\n", __func__, count++, readl(REG_AP_AHB_AHB_EB));
}

static __init int bm_test_init(void)
{
	struct sci_bm_cfg bm_cfg;
	int ret;

	bm_cfg.addr_min = 0x20d00000;
	bm_cfg.addr_max = 0x20d00000;
	bm_cfg.data_min = 0x0;
	bm_cfg.data_max = 0x1ffff;
	bm_cfg.bm_mode = W_MODE;
	
	ret = sci_bm_set_point(AHB_BM0, CHN0, &bm_cfg, bm_callback);

	printk("ret is %d\n", ret);

	return 0;
}

static __exit void bm_test_exit(void)
{
	sci_bm_unset_point(AXI_BM3);
}

module_init(bm_test_init);
module_exit(bm_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.jiang<jack.jiang@spreadtrum.com>");
MODULE_DESCRIPTION("bm module test");
