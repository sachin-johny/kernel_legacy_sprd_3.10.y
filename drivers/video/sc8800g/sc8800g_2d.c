/*
 * drivers/video/sc8800g/sc8800g_2d.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/sched.h>
#include <mach/sc8800g_2d.h>

#include "sc8800g_copybit_lcdc.h"

#define SC8800G_2D_MINOR MISC_DYNAMIC_MINOR

static struct mutex *lock;

int sc8800g_2d_open(struct inode *inode, struct file *file)
{
	struct s2d_blit_req *params;
	params = (struct s2d_blit_req *)kmalloc(
			sizeof(struct s2d_blit_req), GFP_KERNEL);

	if(params == NULL) {
		printk(KERN_ERR "Instance memory allocation was failed\n");
		return -1;
	}

	memset(params, 0, sizeof(struct s2d_blit_req));

	file->private_data = (struct s2d_blit_req *)params;

	printk("[pid:%d] sc8800g_2d_open()\n", current->pid);

	return 0;
}


int sc8800g_2d_release(struct inode *inode, struct file *file)
{
	struct s2d_blit_req *params;

	params = (struct s2d_blit_req *)file->private_data;
	if (params == NULL) {
		printk(KERN_ERR "Can't release sc8800g_2d !!\n");
		return -1;
	}

	kfree(params);

	printk("[pid:%d] sc8800g_2d_release()\n", current->pid);

	return 0;
}

static int sc8800g_2d_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct s2d_blit_req *params;

	params = (struct s2d_blit_req*)file->private_data;
	if (copy_from_user(params, 
		(struct s2d_blit_req*)arg, sizeof(struct s2d_blit_req)))
		return -EFAULT;

	//printk("[pid:%d] sc8800g_2d_ioctl()\n", current->pid);
	mutex_lock(lock);

	if (cmd != SC8800G_2D_BLIT) {
		mutex_unlock(lock);
		printk(KERN_ERR "sc8800g_2d: unknown ioctl cmd\n");
		return -EFAULT;
	}

	if (do_copybit_lcdc(params)) {
		mutex_unlock(lock);
		return -EFAULT;
	}

	mutex_unlock(lock);

	return 0;

}

struct file_operations sc8800g_2d_fops = {
	.owner    = THIS_MODULE,
	.open    = sc8800g_2d_open,
	.release = sc8800g_2d_release,
	.ioctl   = sc8800g_2d_ioctl,
};


static struct miscdevice sc8800g_2d_dev = {
	.minor   = SC8800G_2D_MINOR,
	.name   = "sc8800g_2d",
	.fops   = &sc8800g_2d_fops,
};


int sc8800g_2d_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_ALERT"sc8800g_2d_probe called\n");

	ret = misc_register(&sc8800g_2d_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				SC8800G_2D_MINOR, ret);
		return ret;
	}

	lock = (struct mutex *)kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL)
		return -1;

	mutex_init(lock);

	printk(KERN_ALERT" sc8800g_2d_probe Success\n");

	return 0;
}


static int sc8800g_2d_remove(struct platform_device *dev)
{
	printk(KERN_INFO "sc8800g_2d_remove called !\n");

	misc_deregister(&sc8800g_2d_dev);

	printk(KERN_INFO "sc8800g_2d_remove Success !\n");
	return 0;
}



static struct platform_driver sc8800g_2d_driver = {
	.probe    = sc8800g_2d_probe,
	.remove   = sc8800g_2d_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sc8800g_2d",
	},
};


int __init sc8800g_2d_init(void)
{
	if(platform_driver_register(&sc8800g_2d_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}

	return 0;
}

void sc8800g_2d_exit(void)
{
	platform_driver_unregister(&sc8800g_2d_driver);
	mutex_destroy(lock);
}

module_init(sc8800g_2d_init);
module_exit(sc8800g_2d_exit);

MODULE_DESCRIPTION("SC8800G 2D Driver");
MODULE_LICENSE("GPL");
