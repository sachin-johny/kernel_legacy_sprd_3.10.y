/*
 *
 * Support for shutdown
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/regs_rtc.h>
#include <mach/hardware.h>
#include <mach/board.h>



///sys/devices/platform/hwrst.0/hwrst
 //cat hwrst
//echo 1 hwrst
static struct platform_device *hwrst_device = NULL;

/* sprd hwrst abnormal shutdown */
struct sprd_hwrst {
	struct platform_device *pdev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
//	struct hwrst_classdev cdev;
	int enabled;
	int suspend;
	struct early_suspend sprd_early_suspend_desc;
};
//ANA_HWRST_RTC

#define HWRST_STATUS_SHUTDOWN (0x10)

#define HWRST_STATUS_WDOG      (0x81)

static ssize_t hwrst_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	unsigned hw_rtc;
	hw_rtc = ANA_REG_GET(ANA_HWRST_RTC);
   hw_rtc |= BIT_0;
   ANA_REG_SET(ANA_HWRST_RTC, hw_rtc);   /*normal shutdown*/

	hw_rtc = ANA_REG_GET(ANA_RST_STATUS);
	hw_rtc |= BIT_9;
//	hw_rtc |= BIT_4;
//	hw_rtc |= BIT_7;
	ANA_REG_SET(ANA_RST_STATUS, hw_rtc);
   
	return size;
}

static ssize_t hwrst_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t ret = 0;

	if(in_abnormal_mode())
		ret = sprintf(buf, "%d\n", 1);
	else
      ret = sprintf(buf, "%d\n", 2);
	
	return ret;
}

static DEVICE_ATTR(hwrst, 0x666, hwrst_show, hwrst_store);

static ssize_t hwrst_status_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	unsigned hw_rst;
	
	hw_rst = ANA_REG_GET(ANA_RST_STATUS);
	hw_rst &= ~BIT_9;
	ANA_REG_SET(ANA_RST_STATUS, hw_rst);
	printk("%s---0x%x\n",__func__,ANA_REG_GET(ANA_RST_STATUS));
	return size;
}

/*
static ssize_t hwrst_status_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
}
*/
static DEVICE_ATTR(hwrst_status, 0x666, NULL/*hwrst_status_show*/, hwrst_status_store);


static int sprd_creat_hwrst_sysfs(struct device *dev)
{
	 int err = 0;

    err = device_create_file(dev, &dev_attr_hwrst);
    if (err){
         goto err_out;
     }

	 err = device_create_file(dev, &dev_attr_hwrst_status);
    if (err){
			device_remove_file(dev, &dev_attr_hwrst);
         goto err_out;
     }
    
	return 0;

err_out:
      return err;
}
static int sprd_remove_hwrst_attr(struct device *dev)
{

    device_remove_file(dev, &dev_attr_hwrst);

    return 0;
}

#ifdef CONFIG_EARLYSUSPEND
static void sprd_hwrst_earlysuspend(struct early_suspend *h)
{
	struct sprd_hwrst *hwrst = container_of(h, struct sprd_hwrst, sprd_early_suspend_desc);
	unsigned hw_rst;
	
	hw_rst = ANA_REG_GET(ANA_RST_STATUS);
	hw_rst &= ~BIT_8;
	ANA_REG_SET(ANA_RST_STATUS, hw_rst);
	printk("\n\033[31m%s()\033[0m\n", __func__);
/*
	mutex_lock(&hwrst->mutex);
	hwrst->suspend = 1;
	mutex_unlock(&hwrst->mutex);
*/
}
static void sprd_hwrst_lateresume(struct early_suspend *h)
{
	struct sprd_hwrst *hwrst = container_of(h, struct sprd_hwrst, sprd_early_suspend_desc);
	unsigned hw_rst;

	hw_rst = ANA_REG_GET(ANA_RST_STATUS);
	hw_rst |= BIT_8;
	ANA_REG_SET(ANA_RST_STATUS, hw_rst);
	printk("\n\033[31m%s()\033[0m\n", __func__);
/*
	mutex_lock(&hwrst->mutex);
	hwrst->suspend = 0;
	mutex_unlock(&hwrst->mutex);
*/
}
#endif

static int hwrst_probe(struct platform_device *pdev)
{
		struct sprd_hwrst *hwrst;
      int ret;

		hwrst = kzalloc(sizeof(*hwrst), GFP_KERNEL);
		hwrst->enabled = 0;
	   hwrst->suspend = 0;

		platform_set_drvdata(pdev, hwrst);
#ifdef CONFIG_EARLYSUSPEND
	   hwrst->sprd_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		hwrst->sprd_early_suspend_desc.suspend = sprd_hwrst_earlysuspend;
		hwrst->sprd_early_suspend_desc.resume = sprd_hwrst_lateresume;
		register_early_suspend(&hwrst->sprd_early_suspend_desc);
#endif

		ret = sprd_creat_hwrst_sysfs(&(pdev->dev));
      if (ret) 
			printk (KERN_ERR "cannot create hwrst_sysfs\n");

		return ret;
}

static int hwrst_remove(struct platform_device *pdev)
{
	sprd_remove_hwrst_attr(&(pdev->dev));
	return 0;
}

static struct platform_driver hwrst_driver = {
	.probe  = hwrst_probe,
	.remove = hwrst_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name  = "hwrst",
	},
};

/*----------------------------------------------------------------------------*/

static int __init hwsrt_set_init(void)
{
	  int err;

	  if ((err = platform_driver_register(&hwrst_driver))){
			printk("platform hwrst device register Failed \n");
	   	return err;
		}

	  if ((hwrst_device = platform_device_alloc("hwrst", 0)) == NULL) {
		  err = -ENOMEM;
		  goto exit_driver_unregister;
	   }

	  if ((err = platform_device_add(hwrst_device)))
		  goto exit_free_hwrst_device;

 	return 0;

exit_free_hwrst_device:
	platform_device_put(hwrst_device);

exit_driver_unregister:
	platform_driver_unregister(&hwrst_driver);
	return err;
}

static void __exit hwrst_set_exit(void)
{
	platform_driver_unregister(&hwrst_driver);
}

module_init(hwsrt_set_init);
module_exit(hwrst_set_exit);

MODULE_AUTHOR("this module for set HWRST");
MODULE_DESCRIPTION("set for HWRST");
MODULE_LICENSE("GPL");
