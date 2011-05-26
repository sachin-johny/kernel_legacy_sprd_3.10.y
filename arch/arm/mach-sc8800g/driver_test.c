

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/pm_devices.h>
#include <mach/test.h>

static int __devinit omap_wdt_probe(struct platform_device *pdev)
{
	printk("######: omap_wdt_probe()\n");

	return 0;
}

static void omap_wdt_shutdown(struct platform_device *pdev)
{

}

static int __devexit omap_wdt_remove(struct platform_device *pdev)
{


	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int omap_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{

	printk("######: pm: omap_wdt_suspend()\n");

	return 0;
}

static int omap_wdt_resume(struct platform_device *pdev)
{
	printk("######: pm: omap_wdt_resumes()\n");

	return 0;
}

#else
#define	omap_wdt_suspend	NULL
#define	omap_wdt_resume		NULL
#endif

static struct platform_driver omap_wdt_driver = {
	.probe		= omap_wdt_probe,
	.remove		= __devexit_p(omap_wdt_remove),
	.shutdown	= omap_wdt_shutdown,
	.suspend	= omap_wdt_suspend,
	.resume		= omap_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "omap_wdt",
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND

    struct early_suspend early_suspend;
static void sc8800g2_wdt_suspend (struct early_suspend* es)
{
    printk("###: early: sc8800g2_wdt_suspend()!\n");
    printk("###: early: sc8800g2_wdt_suspend()!\n");
    printk("###: early: sc8800g2_wdt_suspend()!\n");
}

static void sc8800g2_wdt_resume (struct early_suspend* es)
{
    printk("###: early: sc8800g2_wdt_resume()!\n");
    printk("###: early: sc8800g2_wdt_resume()!\n");
    printk("###: early: sc8800g2_wdt_resume()!\n");
}
#endif


struct sprd_pm_suspend sprd_suspend;

static int sc8800g2_sprd_suspend (struct platform_device *pdev, pm_message_t state)
{
   // printk("###: sprd: sc8800g2_wdt_suspend()!\n");
	/*
	if (get_sys_cnt() > 300000)	return -1;
	else return 0;
	*/
	return 0;
}

static int sc8800g2_sprd_resume (struct platform_device *pdev)
{
    //printk("###: sprd: sc8800g2_wdt_resume()!\n");
	return 0;
}



static int __init omap_wdt_init(void)
{
	struct clk *clk_usb;

	printk("######: omap_wdt_init()\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.suspend = sc8800g2_wdt_suspend;
	early_suspend.resume  = sc8800g2_wdt_resume;
	early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&early_suspend);
#endif

	sprd_suspend.suspend = sc8800g2_sprd_suspend;
	sprd_suspend.resume  = sc8800g2_sprd_resume;
	/* !!!!!!! please set this parameter correctly. */
	sprd_suspend.pdev = NULL;
	sprd_suspend.level   = SPRD_PM_SUSPEND_LEVEL0;
	register_sprd_pm_suspend(&sprd_suspend);
/*
	clk_usb = clk_get(NULL, "clk_usb_ref");
	if (IS_ERR(clk_usb)) {
		printk("##: can't get clk[usb]!\n");
		printk("##: can't get clk[usb]!\n");
		printk("##: can't get clk[usb]!\n");
	}
	else {
		printk("##: get clk[usb] successfully!\n");
		printk("##: get clk[usb] successfully!\n");
		printk("##: get clk[usb] successfully!\n");
	}

	clk_enable(clk_usb);

*/
	return platform_driver_register(&omap_wdt_driver);
}

static void __exit omap_wdt_exit(void)
{
	platform_driver_unregister(&omap_wdt_driver);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);




