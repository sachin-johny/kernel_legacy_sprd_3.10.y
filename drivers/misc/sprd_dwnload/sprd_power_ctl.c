#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <linux/clk.h>
#include <linux/delay.h>

#define	POWER_CTL_NAME	"power_ctl"

#define DOWNLOAD_IOCTL_BASE	'z'
#define DOWNLOAD_POWER_ON	_IO(DOWNLOAD_IOCTL_BASE, 0x01)
#define DOWNLOAD_POWER_OFF	_IO(DOWNLOAD_IOCTL_BASE, 0x02)
#define DOWNLOAD_POWER_RST	_IO(DOWNLOAD_IOCTL_BASE, 0x03)

#define DOWNLOAD_GPIO_RST	(122)


static int sprd_power_ctl_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int sprd_power_ctl_release(struct inode *inode,struct file *filp)
{
	return 0;
}


static void download_clk_init(bool enable)
{
	static struct clk *download_clk = NULL;

	if (download_clk == NULL) {
		download_clk = clk_get(NULL, "clk_aux0");
		if (IS_ERR(download_clk)) {
			printk("failed to get parent clk_aux0\n");
			return;
		}
		clk_set_rate(download_clk, 32000);
	}

	#ifdef CONFIG_OF
	if (enable) {
		clk_prepare_enable(download_clk);
	}else{
		clk_disable_unprepare(download_clk);
	}
	#else
	if (enable) {
		clk_enable(download_clk);
	}else{
		clk_disable(download_clk);
	}
	#endif
}

static void sprd_download_poweron(bool enable)
{
	int ret;
	static struct regulator *download_vdd = NULL;
	static struct regulator *vddwifipa = NULL;

	if (download_vdd == NULL) {
		download_vdd = regulator_get(NULL, "vddcon");

		if (IS_ERR(download_vdd)) {
			printk("Get regulator of vddcon  error!\n");
			return;
		}
	}

	if (vddwifipa == NULL) {
		vddwifipa = regulator_get(NULL, "vddwifipa");
		if (IS_ERR(vddwifipa)) {
			printk("Get regulator of vddwifipa  error!\n");
			return;
		}
	}

	if (enable) {
		regulator_set_voltage(download_vdd, 1600000, 1600000);
		ret = regulator_enable(download_vdd);

		regulator_set_voltage(vddwifipa, 3300000, 3300000);
		ret = regulator_enable(vddwifipa);
	}
	else if (regulator_is_enabled(download_vdd)) {
		ret = regulator_disable(download_vdd);
		ret = regulator_disable(vddwifipa);
	}
}

static void sprd_download_rst(void)
{
	int ret;
	static bool request = 0;

	if(0 == request)
	{
		ret = gpio_request (DOWNLOAD_GPIO_RST, "download");
		if (ret){
			printk ("DOWNLOAD_GPIO_RST request err: %d\n", DOWNLOAD_GPIO_RST);
		}
		else{
			request = 1;
		}
	}
	gpio_direction_output(DOWNLOAD_GPIO_RST,1);
	msleep(1);
	gpio_direction_output(DOWNLOAD_GPIO_RST,0);
	msleep(1);
	gpio_direction_output(DOWNLOAD_GPIO_RST,1);
	msleep(100);
}

static long sprd_power_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	printk("DOWNLOAD IOCTL: 0x%x.\n", cmd);

	switch (cmd) {
	case DOWNLOAD_POWER_ON:
		sprd_download_poweron(1);
		download_clk_init(1);
		break;
	case DOWNLOAD_POWER_OFF:
		sprd_download_poweron(0);
		break;
	case DOWNLOAD_POWER_RST:
		sprd_download_rst();
		break;
	}
	return 0;
}

static struct file_operations sprd_power_ctl__fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl	= sprd_power_ctl_ioctl,
	.open  = sprd_power_ctl_open,
	.release = sprd_power_ctl_release,
};

static struct miscdevice sprd_power_ctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = POWER_CTL_NAME,
	.fops = &sprd_power_ctl__fops,
};

static int __init sprd_power_ctl_init(void)
{
	int err=0;
	printk("sprd_power_ctl_init\n");
	err = misc_register(&sprd_power_ctl_device);

	if(err){
		printk(KERN_INFO "download power control dev add failed!!!\n");
	}
	return err;
}

static void __exit sprd_power_ctl_cleanup(void)
{
	misc_deregister(&sprd_power_ctl_device);
}
module_init(sprd_power_ctl_init);
module_exit(sprd_power_ctl_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sprd download img driver");

