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
#include "types.h"
#include "hw_dv.h"
#include "dv_ioctl.h"
#include "debug.h"
#include "error.h"

#define SIM_DEV_NAME "SIM"
#define SIM_LOG_POOL_SIZE 0x10000  //64k

struct kdv_log_s
{
	char *logPool;
	int   end;
	int   rp;
	int   wp;
};

struct kdv_dev
{
	struct kdv_log_s log;
	struct semaphore sem;
	struct cdev cdev;
};

static int major = 0;  /*0: dynamic alloc major number*/
static struct kdv_dev dv_dev;
unsigned int dbgGate = 0;

void dvDbgMaskSet(u32 mask)
{
	dbgGate = mask;
}

u32 dvDbgMaskGet(void)
{
	return dbgGate;
}

int dvLog(DV_LOG_CELL_T *logCell,u32 *data)
{
	int headLen,tmpLen;
	struct kdv_log_s *log;

	headLen =  sizeof(DV_LOG_CELL_T);
	log = (struct kdv_log_s *)&dv_dev.log;
	//DV_DBG("logCell->type =%d \n",logCell->type);

	if (log->logPool == NULL)
	{
		DV_DBG("logPool is NULL \n");
		return DV_ERROR;
	}
	// free memory is enough for the log
	if (((log->rp+log->end-log->wp-1)%log->end) >(headLen+logCell->dataLen)) 
	{
		if (log->rp>log->wp) 
		{
			memcpy(log->logPool + log->wp, logCell, headLen);
			memcpy(log->logPool + log->wp+headLen, data, logCell->dataLen);
			log->wp += headLen + logCell->dataLen;
		}
	else
	{

		tmpLen = log->end - log->wp;
		if (headLen  > tmpLen)
		{
			memcpy(log->logPool + log->wp, logCell, tmpLen);
			memcpy(log->logPool, logCell+tmpLen, headLen-tmpLen);
			log->wp = headLen-tmpLen;
			memcpy(log->logPool + log->wp, data, logCell->dataLen);
			log->wp += logCell->dataLen;
		}
		else
		{
			memcpy(log->logPool+log->wp, logCell, headLen);
			log->wp = (log->wp+headLen) % log->end;
			if ((log->end - log->wp) < logCell->dataLen)
			{
				tmpLen = log->end - log->wp;
				memcpy(log->logPool + log->wp, data, tmpLen);
				memcpy(log->logPool, data+tmpLen, logCell->dataLen-tmpLen);
				log->wp = logCell->dataLen-tmpLen;
			}
			else
			{
				memcpy(log->logPool+log->wp, data, logCell->dataLen);
				log->wp += logCell->dataLen;
			}

		}

	}
		up(&dv_dev.sem);
		return DV_OK;
	}
	else
	{
		DV_DBG("%s line %d: no space \n", __FUNCTION__,__LINE__);
		return DV_ERROR;
	}
}

EXPORT_SYMBOL(dvLog);

int simDrvOpen(struct inode *inode, struct file *filp)
{
	struct kdv_dev *dev;

	DV_DBG("%s \n",__FUNCTION__);
	dev = container_of(inode->i_cdev, struct kdv_dev, cdev);
	filp->private_data = dev;


	return 0;
}

int simDrvRelease(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

ssize_t doSimDrvRead(struct file *filp, char __user *buf,
		size_t count, loff_t*f_pos)
{
	struct kdv_dev *dev = filp->private_data;
	struct kdv_log_s *log = &dev->log;
	size_t wrapCount;

	if (log->rp == log->wp) 
	{ 
		/* nothing to read */
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		//if (down_interruptible(&dev->sem)) //going to sleep
		//	return -ERESTARTSYS;
		return -EAGAIN;
	}

	/* data here */

	if (log->wp > log->rp)
	{    
		count = count > (log->wp - log->rp) ? log->wp - log->rp : count; 
		wrapCount = 0;

	}
	else /* the write pointer has wrapped */
	{
		if (count > ( log->end - log->rp)) /*read wrapped data  */
		{
			wrapCount = (count -(log->end - log->rp))> log->wp 
				? log->wp : (count -(log->end - log->rp));
			count = log->end - log->rp;
		}
		else  
		{
			wrapCount = 0;
		}
	}

	if (copy_to_user(buf, log->logPool+log->rp, count)) 
	{
		return -EFAULT;

	}
	if (wrapCount>0) //copy wrapped data
	{
		if (copy_to_user(buf+count, log->logPool, wrapCount)) 
		{
			return -EFAULT;

		}
	}

	log->rp += count;
	if (wrapCount > 0)
	log->rp = wrapCount;

	return count + wrapCount;
}

ssize_t simDrvRead(struct file *filp, char __user *buf,
		size_t count, loff_t*f_pos)
{
	ssize_t ret;
	struct kdv_dev *dev = filp->private_data;

	if (down_interruptible(&dev->sem))
	{
		return -ERESTARTSYS;
	}

	ret = doSimDrvRead(filp, buf, count, f_pos);

	up(&dev->sem);

	return ret;

}

ssize_t simDrvWrite(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	return 0;
}

int simDrvIoctl(struct inode *inode, struct file *filp,
		u32 cmd, unsigned long args)
{
	DV_IOCTL_T ioctl;
	char *pBuf;
	int ret = 0;
	int count;

	if (copy_from_user(&ioctl, (char *)args, sizeof(DV_IOCTL_T)))
		return -EFAULT;

	switch (cmd)
	{
	case DV_IOCGETREG8 :  //read 8bit register
		ioctl.data = hwDvRegRead8(ioctl.addr);
		break;

	case DV_IOCGETREG16 : //read 16bit register
		ioctl.data = hwDvRegRead16(ioctl.addr);
		break;

	case DV_IOCGETREG32:  //read 32bit register
		ioctl.data = hwDvRegRead32(ioctl.addr);
		break;

	case DV_IOCSETREG8:   //set 8bit register
		hwDvRegWrite8((u8)ioctl.data, ioctl.addr);
		break;

	case DV_IOCSETREG16:  //set 16bit register
		hwDvRegWrite16((u16)ioctl.data, ioctl.addr);
		break;

	case DV_IOCSETREG32:  //set 32bit register
		hwDvRegWrite32((u32)ioctl.data, ioctl.addr);
		break;

	case DV_IOCGETREG_ALL:  // read all registers
		pBuf = kmalloc(ioctl.dataLen,GFP_KERNEL);
		if (pBuf == NULL)
		{
			ret = -ENOMEM;
			break;
		}
		if ((count = hwDvRegReadAll(pBuf, ioctl.dataLen, DV_TRUE)) < 0)
		{
			kfree(pBuf);
			ret = -EINVAL;
			break;
		}
		if (copy_to_user((char *)ioctl.data, pBuf, count))
			ret = -EINVAL;
		else
		{
			ioctl.dataLen = count;
			ret = 0;
		} 

		kfree(pBuf);
		break;

	case DV_IOCGETDATA_TXHOOK:  // get data from tx stream at hook pointer

		pBuf = kmalloc(ioctl.dataLen, GFP_KERNEL);
		if (pBuf == NULL)
		{
			ret = -ENOMEM;
			break;
		}
		if ((count = hwDvDataHookRead(pBuf, ioctl.dataLen, DV_TRUE)) < 0)
		{
			kfree(pBuf);
			ret = -EINVAL;
			break;
		}
		if (copy_to_user((char *)ioctl.data, pBuf, count))
			ret= -EINVAL;
		else
		{
			ioctl.dataLen = count;
			ret = 0;
		}
		kfree(pBuf);
		break;

	case DV_IOCGETDATA_TXOUT:  // get device output data 
		/*DV_DBG("%s DV_IOCGETDATA_TXOUT len=%d\n",
				__FUNCTION__,ioctl.dataLen);*/
		pBuf = kmalloc(ioctl.dataLen, GFP_KERNEL);
		if (pBuf == NULL)
		{
			DV_DBG("DV_IOCGETDATA_TXOUT malloc err\n");
			ret = -ENOMEM;
			break;
		}
		if ((count = hwDvDataTxoutRead(filp, &ioctl, pBuf)) < 0)
		{
			kfree(pBuf);
			//DV_DBG("DV_IOCGETDATA_TXOUT count=%d\n",count);
			ret = count;
			break;
		}
		if (copy_to_user((char *)ioctl.data, pBuf, count))
			ret = -EINVAL;
		else
		{
			ioctl.dataLen = count;
			ret = 0;
		}
		kfree(pBuf);
		//DV_DBG("DV_IOCGETDATA_TXOUT ret=%d count=%d\n",ret,count);
		break;

	case DV_IOCGETDATA_RXHOOK: //get data from rx stream at hook pointer
		pBuf = kmalloc(ioctl.dataLen, GFP_KERNEL);
		if (pBuf == NULL)
		{
			ret = -ENOMEM;
			break;
		}
		if ((count = hwDvDataHookRead(pBuf, ioctl.dataLen, DV_TRUE))<0)
		{
			kfree(pBuf);
			ret = -EINVAL;
			break;
		}
		if (copy_to_user((char *)ioctl.data, pBuf, count))
			ret = -EINVAL;
		else
		{
			ioctl.dataLen = count;
			ret = 0;
		}
		kfree(pBuf);
		break;

	case DV_IOCSETDATA:  //set  data which used as  device input

		DV_DBG("set dataLen = %d \n",ioctl.dataLen);
		pBuf = kmalloc(ioctl.dataLen, GFP_KERNEL);
		if (pBuf == NULL)
		{
			ret = -ENOMEM;
			break;
		}
		if (copy_from_user(pBuf, (char *)ioctl.data, ioctl.dataLen))
		{
			ret = -EINVAL;
			break;
		}
		ret = hwDvDataRxInWrite(&ioctl, pBuf);
		kfree(pBuf);
		break;

	//indicate the process is completed and the user can get the result 
	case DV_IOCGET_SIMISDONE: 
		ret = hwDvSimIsDone(ioctl.data);
		break;

	case DV_IOCSETREG_RESET: // reset all registers to default value
		ret = hwDvRegReset();
		break;

	case DV_IOCOP_USERDEF:
		ret = hwDvUsrDefineOp(&ioctl);
		break;
	case DV_GET_KERN_ADDR:
		ret = get_kern_addr(&ioctl);
		break;
	case DV_GET_KERN_DATA:
		ret = get_kern_data(&ioctl);
		break;
	case DV_SET_KERN_DATA:
		ret = set_kern_data(&ioctl);
		break;
	default:
		DV_DBG("Invalid ioctl:%d",cmd);
		ret = -EINVAL;
	}

	//copy the response to user space
	if (copy_to_user((char *)args,(char *)&ioctl,sizeof(DV_IOCTL_T)))  
		ret = -EFAULT;
	//DV_DBG("simIoctl ret=%d \n",ret);
	return ret;
}

struct file_operations sim_fops = 
{
	.owner = THIS_MODULE,
	.open = simDrvOpen,
	.read = simDrvRead,
	.write = simDrvWrite,
	.ioctl = simDrvIoctl,
	.release = simDrvRelease,
};

#include <linux/cdev.h>
#include <linux/device.h>

static struct class *sim_class;

static void sim_setup_cdev(struct kdv_dev *dev,int index)
{
	//create the specific char device
    
	int ret;
	int devno = MKDEV(major, index);
	struct kdv_log_s  *log;
	struct device *sysfs_node;

	sema_init(&dev->sem, 10);
	log = &dev->log;
	log->end = SIM_LOG_POOL_SIZE;
	log->logPool = kmalloc(SIM_LOG_POOL_SIZE, GFP_KERNEL);
	if (log->logPool == NULL)
	{
		DV_DBG("alloc log pool failed \n");
		return;
	}
	log->rp = 0;
	log->wp = 0;

	cdev_init(&dev->cdev,&sim_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &sim_fops;
	ret = cdev_add(&dev->cdev, devno, 1);
	if (ret)
	{
		DV_DBG("cdev_add Error %d adding %d", ret, index);
		return;
	}
	sysfs_node = device_create(sim_class, NULL, devno, NULL, "sim");

	if (IS_ERR(sysfs_node)) {
		DV_DBG("WARN: device node can't create automatically,"
			    "errno=%ld\n", PTR_ERR(sysfs_node));
		cdev_del(&dev->cdev);
	}
}

static int __init simDrvInit(void)
{
	int ret;
	dev_t devno = MKDEV(major, 0);

	sim_class = class_create(THIS_MODULE, "sim_class");
	if (IS_ERR(sim_class)) {
		DV_DBG("sim class register failed, errno:%ld\n",
			    PTR_ERR(sim_class));
		return PTR_ERR(sim_class);
	}

	if(hwDvSimInit() != DV_OK)
	{
		DV_DBG("%s: hwDvSimInit failed.\n", __FUNCTION__);
		ret = DV_ERROR;
		goto class_out;
	}

	

	if (major)  //user define the major number
	{
		ret = register_chrdev_region(devno, 1, SIM_DEV_NAME);
	}
	else    // system alloc the major number
	{
		ret = alloc_chrdev_region(&devno,0, 1, SIM_DEV_NAME);
		major = MAJOR(devno);
	}

	if (ret < 0)
	{
		DV_DBG("can't get major %d) \n", major);
		goto devt_out;
	}

	sim_setup_cdev(&dv_dev, 0);

	return DV_OK;

devt_out:
	hwDvSimExit();
class_out:
	class_destroy(sim_class);
	return ret;
}

static void __exit simDrvExit(void)
{   
	device_destroy(sim_class, MKDEV(major, 0));
	cdev_del(&dv_dev.cdev);
	kfree(dv_dev.log.logPool);
	unregister_chrdev_region(MKDEV(major,0),1);
	hwDvSimExit();
	class_destroy(sim_class);
}

module_init(simDrvInit);
module_exit(simDrvExit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Hugh & Tanxy");   
MODULE_DESCRIPTION("A Linux Driver DV Platform");   
MODULE_ALIAS("DRV DV"); 
MODULE_VERSION("V1.00"); 


/*
 *$Log$
 */

