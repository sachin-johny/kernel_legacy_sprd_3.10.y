/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: simdrv.c
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include "dev_interf.h"

#define TROUT_DBG_DEV_NAME "iwlan_dbg"

struct kdv_dev
{
	struct cdev cdev;
	struct semaphore sem;
    int rx_timeout_second;
    int tx_timeout_second;
};

static int major = 0;  /*0: dynamic alloc major number*/
static struct kdv_dev dv_dev;

extern int trout_b2b_pkt_to_host(__user char *buf, int buf_len, int timeout);
extern int trout_b2b_pkt_from_host(const __user char *buf, int buf_len);
extern int b2b_list_reinit(void);
extern void get_txrx_count(u32 *tx_ok, u32*tx_fail, u32 *rx_ok, u32 *rx_fail);

int devIntfOpen(struct inode *inode, struct file *filp)
{
	struct kdv_dev *dev;

	printk("%s \n", __FUNCTION__);
	dev = container_of(inode->i_cdev, struct kdv_dev, cdev);
	filp->private_data = dev;
    
	printk("b2b_list_reinit...\n");
    b2b_list_reinit();
	printk("done\n");

	return 0;
}

int devIntfRelease(struct inode *inode, struct file *filp)
{
	printk("%s \n",__FUNCTION__);
	filp->private_data = NULL;
    
	return 0;
}

ssize_t devIntfRead(struct file *filp, char __user *buf,
		size_t count, loff_t*f_pos)
{
	ssize_t ret;
	struct kdv_dev *dev = filp->private_data;

    ret = trout_b2b_pkt_to_host(buf, count, dev->rx_timeout_second);

	return ret;
}

ssize_t devIntfWrite(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t ret;
//	struct kdv_dev *dev = filp->private_data;

    ret = trout_b2b_pkt_from_host(buf, count);
	return ret;
}

int devIntfIoctl(struct inode *inode, struct file *filp,
		u32 cmd, unsigned long args)
{
    int ret = 0;
	struct kdv_dev *dev = filp->private_data;
    
	switch (cmd)
	{
        case TROUT_IOC_SRXTIMEOUT:
        {
            get_user(dev->rx_timeout_second, (int *)args);
            printk("set rx timeout: %ds\n", dev->rx_timeout_second);
            break;
        }
        case TROUT_IOC_GTXRXCOUNT:
        {
            unsigned int txrx_count[4];
            get_txrx_count(&txrx_count[0], &txrx_count[1], &txrx_count[2], &txrx_count[3]);
            copy_to_user((unsigned int *)args, txrx_count, sizeof(txrx_count));
            printk("ioctl: tx_ok: %u, tx_fail: %u, rx_ok: %u, rx_fail: %u.\n", 
                    txrx_count[0], txrx_count[1], txrx_count[2], txrx_count[3]);
            break;
        }
	    default:
    		printk("Invalid ioctl:0x%x", cmd);
    		ret = -EINVAL;
	}

	return ret;
}

struct file_operations sim_fops = 
{
	.owner = THIS_MODULE,
	.open = devIntfOpen,
	.read = devIntfRead,
	.write = devIntfWrite,
	.ioctl = devIntfIoctl,
	.release = devIntfRelease,
};

#include <linux/cdev.h>
#include <linux/device.h>

static struct class *sim_class;

static void sim_setup_cdev(struct kdv_dev *dev,int index)
{
	//create the specific char device
    
	int ret;
	int devno = MKDEV(major, index);
	struct device *sysfs_node;

    dev->rx_timeout_second = -1;   //wait forever.
	sema_init(&dev->sem, 10);
	cdev_init(&dev->cdev, &sim_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &sim_fops;
	ret = cdev_add(&dev->cdev, devno, 1);
	if(ret)
	{
		printk("cdev_add Error %d adding %d", ret, index);
		return;
	}
    
	sysfs_node = device_create(sim_class, NULL, devno, NULL, "trout_dbg");
	if (IS_ERR(sysfs_node)) {
		printk("WARN: device node can't create automatically,"
			    "errno=%ld\n", PTR_ERR(sysfs_node));
		cdev_del(&dev->cdev);
	}
}

static int __init devIntfInit(void)
{
	int ret;
	dev_t devno = MKDEV(major, 0);

	sim_class = class_create(THIS_MODULE, "trout_dbg_class");
	if (IS_ERR(sim_class)) {
		printk("trout class register failed, errno:%ld\n",
			    PTR_ERR(sim_class));
		return PTR_ERR(sim_class);
	}

	if (major)  //user define the major number
	{
		ret = register_chrdev_region(devno, 1, TROUT_DBG_DEV_NAME);
	}
	else    // system alloc the major number
	{
		ret = alloc_chrdev_region(&devno, 0, 1, TROUT_DBG_DEV_NAME);
		major = MAJOR(devno);
	}

	if (ret < 0)
	{
		printk("can't get major %d) \n", major);
		goto devt_out;
	}

	sim_setup_cdev(&dv_dev, 0);

	return 0;

devt_out:
	class_destroy(sim_class);
    
	return ret;
}

static void __exit devIntfExit(void)
{   
	device_destroy(sim_class, MKDEV(major, 0));
	cdev_del(&dv_dev.cdev);
	unregister_chrdev_region(MKDEV(major,0),1);
	class_destroy(sim_class);
}

module_init(devIntfInit);
module_exit(devIntfExit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Hugh & Tanxy");   
MODULE_DESCRIPTION("A Linux Driver for Trout WiFi Device");   
MODULE_VERSION("V1.00"); 


/*
 *$Log$
 */

