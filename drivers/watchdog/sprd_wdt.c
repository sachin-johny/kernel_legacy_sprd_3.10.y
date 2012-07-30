/*
 *	Watchdog driver for the SC8810
 *
 * Copyright (C) 2012 Spreadtrum
 *
 * based on sa1100_wdt.c and sc8810 arch reset implementation.
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
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/timex.h>
#include <linux/sysdev.h>

#include <mach/system.h>

/* #define WDT_DEBUG */
#define WDT "SPRD_WDT "
#ifdef WDT_DEBUG
#define DEBUG_PRINT printk
#else
#define DEBUG_PRINT(...)
#endif

#define SPRD_WATCHDOG_MINOR (WATCHDOG_MINOR - 1)

#define WDT_FREQ	32000	/* 32KHz clock */
static int margin = 20;		/* (secs) Default is 20 sec */
static int feed_period = 3;	/* (secs) Default is 3 sec */

static unsigned long wdt_users;
static int boot_status;

static void watchdog_feeder(unsigned long);
static struct timer_list feeder_timer =
	TIMER_INITIALIZER(watchdog_feeder, 0, 0);

/*
 * This is a hw watchdog starts after init, and fed by a timer started
 * by the watchdog driver itself. It can also be fed from userland.
 * The watchdog will be stopped in system suspend, and restarted in
 * system resume.
 */
static void watchdog_feeder(unsigned long data)
{
	DEBUG_PRINT( WDT "%s, margin=%d, feed_period=%d\n",
			__FUNCTION__, margin, feed_period);
	WDG_LOAD_TIMER_VALUE(margin * WDT_FREQ);
	mod_timer(&feeder_timer, jiffies + (feed_period * HZ));
}

static int sci_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, &wdt_users))
		return -EBUSY;

	/* we have started the watchdog in the init phase */

	return nonseekable_open(inode, file);
}

static int sci_release(struct inode *inode, struct file *file)
{
	clear_bit(1, &wdt_users);
	return 0;
}

static ssize_t sci_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	/* TODO: we should have a new reboot reason */
	/*
	ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_NORMAL);
	*/

	if (len)
		WDG_LOAD_TIMER_VALUE(margin * WDT_FREQ);

	return len;
}

static const struct watchdog_info ident = {
	.options		= WDIOF_CARDRESET | WDIOF_SETTIMEOUT
				| WDIOF_KEEPALIVEPING,
	.identity		= "SC8810 Watchdog",
	.firmware_version	= 1,
};

static long sci_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = -ENOTTY;
	int time;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, p);
		break;

	case WDIOC_KEEPALIVE:
		WDG_LOAD_TIMER_VALUE(margin * WDT_FREQ);
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, p);
		if (ret)
			break;

		if (time <= 0 || (WDT_FREQ * (long long)time >= 0xffffffff)) {
			ret = -EINVAL;
			break;
		}

		WDG_LOAD_TIMER_VALUE(time * WDT_FREQ);
		margin = time;

		/*fall through*/

	case WDIOC_GETTIMEOUT:
		ret = put_user(margin, p);
		break;
	}
	return ret;
}

static int sprd_wdt_resume(struct sys_device *dev)
{
	DEBUG_PRINT( WDT "%s\n", __FUNCTION__);
	ANA_REG_OR (WDG_CTRL, WDG_CNT_EN_BIT);
	return 0;
}

static int sprd_wdt_suspend(struct sys_device *dev, pm_message_t state)
{
	DEBUG_PRINT( WDT "%s\n", __FUNCTION__);
	ANA_REG_AND (WDG_CTRL, ~WDG_CNT_EN_BIT);
	return 0;
}

static struct sysdev_class sprd_wdt_sysclass = {
	.name		= "sprd_wdt",
	.resume		= sprd_wdt_resume,
	.suspend	= sprd_wdt_suspend,
};

static struct sys_device device_wdt = {
	.id		= 0,
	.cls		= &sprd_wdt_sysclass,
};

static int __init wdt_sysdev_init(void)
{
	int error = sysdev_class_register(&sprd_wdt_sysclass);
	if (!error)
		error = sysdev_register(&device_wdt);
	return error;
}

static const struct file_operations sci_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= sci_write,
	.unlocked_ioctl	= sci_ioctl,
	.open		= sci_open,
	.release	= sci_release,
};

static struct miscdevice sci_miscdev = {
	.minor		= SPRD_WATCHDOG_MINOR,
	.name		= "sprd-watchdog",
	.fops		= &sci_fops,
};

extern int in_calibration(void);

static int __init sci_wdt_init(void)
{
	int ret;

	boot_status = 0;

	if (in_calibration()) {
		printk(WDT "calibration mode, quit...\n");
		return 0;
	}

	/* TODO: we should have a new reboot reason */
	/*
	ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_NORMAL);
	*/

	/* start the watchdog */
	ANA_REG_OR(ANA_AGEN, AGEN_WDG_EN | AGEN_RTC_ARCH_EN | AGEN_RTC_WDG_EN);
	ANA_REG_SET (WDG_LOCK, WDG_UNLOCK_KEY);
	ANA_REG_AND (WDG_CTRL, (~WDG_INT_EN_BIT));
	WDG_LOAD_TIMER_VALUE(margin * WDT_FREQ);
	ANA_REG_OR (WDG_CTRL, WDG_CNT_EN_BIT);

	ret = misc_register(&sci_miscdev);
	if (ret != 0)
		goto _out;

	ret = wdt_sysdev_init();
	if (ret != 0)
		goto _out;

	mod_timer(&feeder_timer, jiffies + (feed_period * HZ));

	printk(KERN_INFO "SC8810 Watchdog: timer margin %d sec\n", margin);
_out:
	return ret;
}

static void __exit sci_wdt_exit(void)
{
	misc_deregister(&sci_miscdev);
}

module_init(sci_wdt_init);
module_exit(sci_wdt_exit);

MODULE_DESCRIPTION("SPRD Watchdog");

module_param(margin, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(margin, "Watchdog margin in seconds (default 20s)");

module_param(feed_period, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(feed_period, "Watchdog feed period in seconds (default 3s)");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(SPRD_WATCHDOG_MINOR);
