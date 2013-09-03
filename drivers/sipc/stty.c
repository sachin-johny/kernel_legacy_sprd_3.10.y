/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/sipc.h>
#include <linux/stty.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/vt_kern.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>


#include <linux/compat.h>
#include <linux/tty_flip.h>
#include <linux/kthread.h>

#define STTY_DEV_MAX_NR 	1
#define STTY_MAX_DATA_LEN 	4096

struct stty_device {
	struct stty_init_data	*pdata;
	struct tty_struct 		*tty;
	struct tty_driver 		*driver;
	struct task_struct		*thread;
};

static struct stty_device *stty_ipc;

static int stty_thread(void *data)
{
	struct stty_device *stty = data;
	int i, cnt = 0;
	unsigned char buf[STTY_MAX_DATA_LEN] = {0};

	while(!kthread_should_stop()) {

		cnt = sbuf_read(stty->pdata->dst, stty->pdata->channel,
						stty->pdata->bufid,(void *)buf, STTY_MAX_DATA_LEN, -1);
		if (cnt > 0) {
			for(i = 0; i < cnt; i++) {
				tty_insert_flip_char(stty->tty, buf[i], TTY_NORMAL);
			}
			tty_schedule_flip(stty->tty);
		} else if (cnt == -ENODEV) {
			msleep(2000);
		} else {
			msleep(40);
		}
	}

	return 0;
}

static int stty_open(struct tty_struct *tty, struct file * filp)
{
	struct stty_device *stty = stty_ipc;
	tty->driver_data = NULL;

	if(stty == NULL) {
		printk(KERN_INFO "stty open err NULL!\n");
		return -ENOMEM;
	}
	if (sbuf_status(stty->pdata->dst, stty->pdata->channel) != 0) {
		printk(KERN_ERR "stty_open sbuf not ready to open!dst=%d,channel=%d\n"
			,stty->pdata->dst,stty->pdata->channel);
		return -ENODEV;
	}
	stty->tty = tty;

	tty->driver_data = stty;

	return 0;
}

static void stty_close(struct tty_struct *tty, struct file * filp)
{
	return;
}

static int stty_write(struct tty_struct * tty,
	      const unsigned char *buf, int count)
{
	struct stty_device *stty = tty->driver_data;
	int cnt;

	cnt = sbuf_write(stty->pdata->dst, stty->pdata->channel,
					stty->pdata->bufid, (void *)buf, count, -1);
	return cnt;
}


static void stty_flush_chars(struct tty_struct *tty)
{
	return;
}

static int stty_write_room(struct tty_struct *tty)
{
	return INT_MAX;
}

static const struct tty_operations stty_ops = {
	.open  = stty_open,
	.close = stty_close,
	.write = stty_write,
	.flush_chars = stty_flush_chars,
	.write_room  = stty_write_room,
};

static int stty_driver_init(struct stty_device *device)
{
	struct tty_driver *driver;
	int ret = 0;

	driver = alloc_tty_driver(STTY_DEV_MAX_NR);
	if (!driver)
		return -ENOMEM;
	/*
	 * Initialize the tty_driver structure
	 * Entries in stty_driver that are NOT initialized:
	 * proc_entry, set_termios, flush_buffer, set_ldisc, write_proc
	 */
	driver->owner = THIS_MODULE;
	driver->driver_name = device->pdata->name;
	driver->name = device->pdata->name;
	driver->major = 0;
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SERIAL_TYPE_NORMAL;
	driver->init_termios = tty_std_termios;
	driver->flags = TTY_DRIVER_INSTALLED | TTY_DRIVER_REAL_RAW;
	driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	driver->driver_state = (void*)device;
	 /* initialize the tty driver */
	tty_set_operations(driver, &stty_ops);
	ret = tty_register_driver(driver);
	if (ret) {
		put_tty_driver(driver);
		return ret;
	}
	device->driver = driver;
	return ret;
}

static void stty_driver_exit(struct stty_device *device)
{
	struct tty_driver *driver = device->driver;
	tty_unregister_driver(driver);
}

static int __devinit stty_probe(struct platform_device *pdev)
{
	struct stty_init_data *pdata = (struct stty_init_data*)pdev->dev.platform_data;
	struct stty_device *stty;
	int rval= 0;

	stty = kzalloc(sizeof(struct stty_device),GFP_KERNEL);
	if (stty == NULL) {
		printk(KERN_ERR "stty Failed to allocate device!\n");
		return -ENOMEM;
	}

	stty->pdata = pdata;
	rval = stty_driver_init(stty);
	if (rval) {
		kfree(stty);
		printk(KERN_ERR "stty driver init error!\n");
		return -EINVAL;
	}

	stty->thread = kthread_create(stty_thread, stty,
			"stty-%d-%d", pdata->dst, pdata->channel);
	if (IS_ERR(stty->thread)) {
		stty_driver_exit(stty);
		kfree(stty);
		printk(KERN_ERR "stty kthread_create err!\n");
		rval = PTR_ERR(stty->thread);
		return rval;
	}

	platform_set_drvdata(pdev, stty);
	stty_ipc = stty;
	wake_up_process(stty->thread);

	return 0;
}

static int __devexit stty_remove(struct platform_device *pdev)
{
	struct stty_device *stty = platform_get_drvdata(pdev);

	stty_driver_exit(stty);
	kthread_stop(stty->thread);
	kfree(stty);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver stty_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sttybt",
	},
	.probe = stty_probe,
	.remove = __devexit_p(stty_remove),
};

static int __init stty_init(void)
{
	return platform_driver_register(&stty_driver);
}

static void __exit stty_exit(void)
{
	platform_driver_unregister(&stty_driver);
}

module_init(stty_init);
module_exit(stty_exit);

MODULE_DESCRIPTION("SIPC/stty driver");
MODULE_LICENSE("GPL");
