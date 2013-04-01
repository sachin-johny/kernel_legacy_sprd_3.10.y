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
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/sprd_cproc.h>

#define CPROC_WDT_TRUE   1
#define CPROC_WDT_FLASE  0

struct cproc_device {
	struct miscdevice		miscdev;
	struct cproc_init_data		*initdata;
	void				*vbase;
	int 				wdtirq;
	int				wdtcnt;
	wait_queue_head_t	wdtwait;
};

static int sprd_cproc_open(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = container_of(filp->private_data,
			struct cproc_device, miscdev);

	filp->private_data = cproc;

	pr_debug("cproc %s opened!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_release (struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = filp->private_data;

	pr_debug("cproc %s closed!\n", cproc->initdata->devname);

	return 0;
}

static long sprd_cproc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct cproc_device *cproc = filp->private_data;

	/* TODO: for general modem download&control */

	return 0;
}

static int sprd_cproc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct cproc_device *cproc = filp->private_data;

	/* TODO: for general modem download&control */

	return 0;
}

static const struct file_operations sprd_cproc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cproc_open,
	.release = sprd_cproc_release,
	.unlocked_ioctl = sprd_cproc_ioctl,
	.mmap = sprd_cproc_mmap,
};

/* NK interface is just for compatible user interface in SC8825,
 * it's not used for future native modem start */
#if defined(CONFIG_PROC_FS) && defined(CONFIG_SPRD_CPROC_NKIF)
static void sprd_cproc_nkif_init(struct cproc_device *cproc);
static void sprd_cproc_nkif_exit(struct cproc_device *cproc);
#else /* !CONFIG_SPRD_CPROC_NKIF */
static inline void sprd_cproc_nkif_init(struct cproc_device *cproc) {}
static inline void sprd_cproc_nkif_exit(struct cproc_device *cproc) {}
#endif

static irqreturn_t sprd_cproc_irq_handler(int irq, void *dev_id)
{
	struct cproc_device *cproc = (struct cproc_device *)dev_id;

	printk("sprd_cproc_irq_handler cp watchdog enable !\n");
	cproc->wdtcnt = CPROC_WDT_TRUE;
	wake_up_interruptible_all(&(cproc->wdtwait));
	return IRQ_HANDLED;
}

static int sprd_cproc_probe(struct platform_device *pdev)
{
	struct cproc_device *cproc;
	int rval;

	cproc = kzalloc(sizeof(struct cproc_device), GFP_KERNEL);
	if (!cproc) {
		printk(KERN_ERR "failed to allocate cproc device!\n");
		return -ENOMEM;
	}

	cproc->initdata = pdev->dev.platform_data;

	cproc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cproc->miscdev.name = cproc->initdata->devname;
	cproc->miscdev.fops = &sprd_cproc_fops;
	cproc->miscdev.parent = NULL;
	rval = misc_register(&cproc->miscdev);
	if (rval) {
		kfree(cproc);
		printk(KERN_ERR "failed to register sprd_cproc miscdev!\n");
		return rval;
	}

	cproc->vbase = ioremap(cproc->initdata->base, cproc->initdata->maxsz);
	if (!cproc->vbase) {
		misc_deregister(&cproc->miscdev);
		kfree(cproc);
		printk(KERN_ERR "Unable to map cproc base: 0x%08x\n", cproc->initdata->base);
		return -ENOMEM;
	}

	cproc->wdtcnt = CPROC_WDT_FLASE;
	init_waitqueue_head(&(cproc->wdtwait));

	/* register IPI irq */
	rval = request_irq(cproc->initdata->wdtirq, sprd_cproc_irq_handler,
			0, cproc->initdata->devname, cproc);
	if (rval != 0) {
		printk(KERN_ERR "Cproc failed to request irq %s: %d\n",
				cproc->initdata->devname, cproc->initdata->wdtirq);
		return rval;
	}

	sprd_cproc_nkif_init(cproc);

	platform_set_drvdata(pdev, cproc);

	printk(KERN_INFO "cproc %s probed!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_remove(struct platform_device *pdev)
{
	struct cproc_device *cproc = platform_get_drvdata(pdev);

	sprd_cproc_nkif_exit(cproc);
	iounmap(cproc->vbase);
	misc_deregister(&cproc->miscdev);
	kfree(cproc);

	printk(KERN_INFO "cproc %s removed!\n", cproc->initdata->devname);

	return 0;
}

static struct platform_driver sprd_cproc_driver = {
	.probe    = sprd_cproc_probe,
	.remove   = sprd_cproc_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sprd_cproc",
	},
};

static int __init sprd_cproc_init(void)
{
	if (platform_driver_register(&sprd_cproc_driver) != 0) {
		printk(KERN_ERR "sprd_cproc platform drv register Failed \n");
		return -1;
	}
	return 0;
}

static void __exit sprd_cproc_exit(void)
{
	platform_driver_unregister(&sprd_cproc_driver);
}

module_init(sprd_cproc_init);
module_exit(sprd_cproc_exit);

MODULE_DESCRIPTION("SPRD Communication Processor Driver");
MODULE_LICENSE("GPL");


#if defined(CONFIG_PROC_FS) && defined(CONFIG_SPRD_CPROC_NKIF)
struct nk_proc_fs {
	struct proc_dir_entry	*nk;
	struct proc_dir_entry	*restart;
	struct proc_dir_entry	*guest;
	struct proc_dir_entry	*modem;
	struct proc_dir_entry	*dsp;
	struct proc_dir_entry	*status;
	struct proc_dir_entry	*mem;
	struct proc_dir_entry	*wdtirq;
	struct cproc_device		*cproc;
};

static struct nk_proc_fs nk_entries;

#define MSG "not started\n"

static int nk_proc_open(struct inode *inode, struct file *filp)
{
	filp->private_data = PDE(inode)->data;
	return 0;
}

static int nk_proc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t nk_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = nk_entries.cproc;
	unsigned int len;
	void *vmem;
	int rval;

	pr_debug("nk proc read type: %s ppos %d\n", type, *ppos);

	if (strcmp(type, "status") == 0) {
		len = strlen(MSG);
		count = (len > count) ? count : len;

		if (copy_to_user(buf, MSG, count))
			return -EFAULT;
		else
			return count;
	} else if (strcmp(type, "mem") == 0) {
		if (cproc->initdata->maxsz < *ppos) {
			return -EINVAL;
		} else if (cproc->initdata->maxsz == *ppos) {
			return 0;
		}

		if ((*ppos + count) > cproc->initdata->maxsz) {
			count = cproc->initdata->maxsz - *ppos;
		}
		vmem = cproc->vbase + *ppos;
		if (copy_to_user(buf, vmem, count))	{
			return -EFAULT;
		}
		*ppos += count;
		return count;
	} else if (strcmp(type, "wdtirq") == 0) {
		/* wait forever */
		rval = wait_event_interruptible(cproc->wdtwait, cproc->wdtcnt  != CPROC_WDT_FLASE);
		if (rval < 0) {
			printk(KERN_ERR "nk_proc_read wait interrupted error !\n");
		}
		len = strlen(MSG);
		count = (len > count) ? count : len;
		if (copy_to_user(buf, MSG, count)) {
			printk(KERN_ERR "nk_proc_read copy data to user error !\n");
			return -EFAULT;
		} else {
			printk(KERN_INFO "nk proc read wdtirq data !\n");
			return count;
		}
	} else {
		return -EINVAL;
	}
}

static ssize_t nk_proc_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = nk_entries.cproc;
	uint32_t base, size, offset;
	void *vmem;

	pr_debug("nk proc write type: %s\n!", type);

	if (strcmp(type, "start") == 0) {
		printk(KERN_INFO "nk_proc_write to map cproc base start\n");
		cproc->initdata->start(NULL);
		cproc->wdtcnt = CPROC_WDT_FLASE;
		return count;
	}

	if (strcmp(type, "modem") == 0) {
		base = cproc->initdata->segs[0].base;
		size = cproc->initdata->segs[0].maxsz;
		offset = *ppos - 0x1000;
	} else if (strcmp(type, "dsp") == 0) {
		base = cproc->initdata->segs[1].base;
		size = cproc->initdata->segs[1].maxsz;
		offset = *ppos - 0x20000;
	} else {
		return -EINVAL;
	}

	pr_debug("nk proc write: 0x%08x, 0x%08x\n!", base + offset, count);
	vmem = cproc->vbase + (base - cproc->initdata->base) + offset;

	if (copy_from_user(vmem, buf, count)) {
		return -EFAULT;
	}
	*ppos += count;
	return count;
}

static loff_t nk_proc_lseek(struct file* filp, loff_t off, int whence )
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = nk_entries.cproc;
	loff_t new;

	switch (whence) {
		case 0:
		new = off;
		filp->f_pos = new;
		break;
		case 1:
		new = filp->f_pos + off;
		filp->f_pos = new;
		break;
		case 2:
		if (strcmp(type, "status") == 0) {
			new = sizeof(MSG) - 1 + off;
			filp->f_pos = new;
		} else if (strcmp(type, "mem") == 0) {
			new = cproc->initdata->maxsz + off;
			filp->f_pos = new;
		} else {
			return -EINVAL;
		}
		break;
		default:
		return -EINVAL;
	}
	return (new);
}

struct file_operations proc_fops = {
	.open		= nk_proc_open,
	.release	= nk_proc_release,
	.llseek  	= nk_proc_lseek,
	.read		= nk_proc_read,
	.write		= nk_proc_write,
};

static inline void sprd_cproc_nkif_init(struct cproc_device *cproc)
{
	nk_entries.nk = proc_mkdir("nk", NULL);

	nk_entries.restart = proc_create_data("restart", S_IWUSR, nk_entries.nk, &proc_fops, "start");
	nk_entries.guest = proc_mkdir("guest-02", nk_entries.nk);

	nk_entries.modem = proc_create_data("guestOS_2_bank", S_IWUSR, nk_entries.guest, &proc_fops, "modem");
	nk_entries.dsp = proc_create_data("dsp_bank", S_IWUSR, nk_entries.guest, &proc_fops, "dsp");
	nk_entries.status = proc_create_data("status", S_IRUSR, nk_entries.guest, &proc_fops, "status");
	nk_entries.mem = proc_create_data("mem", S_IRUSR, nk_entries.guest, &proc_fops, "mem");
	nk_entries.wdtirq = proc_create_data("wdtirq", S_IRUSR, nk_entries.guest, &proc_fops, "wdtirq");
	nk_entries.cproc = cproc;
}

static inline void sprd_cproc_nkif_exit(struct cproc_device *cproc)
{
	remove_proc_entry("guestOS_2_bank", nk_entries.guest);
	remove_proc_entry("dsp_bank", nk_entries.guest);
	remove_proc_entry("status", nk_entries.guest);

	remove_proc_entry("guest-02", nk_entries.nk);
	remove_proc_entry("restart", nk_entries.nk);
	remove_proc_entry("mem", nk_entries.nk);
	remove_proc_entry("wdtirq", nk_entries.nk);

	remove_proc_entry("nk", NULL);
}
#endif /* CONFIG_SPRD_CPROC_NKIF */
