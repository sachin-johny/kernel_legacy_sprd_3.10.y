/* drivers/meida/video/sprd_vsp/sc8800g2_vsp.c
 *
 * SC8800G2 VSP driver.
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>


#include <mach/hardware.h>
#include <mach/irqs.h>

#define VSP_MINOR MISC_DYNAMIC_MINOR
#define VSP_TIMEOUT_MS 1000

#define VSP_SC8800G2
//#define VSP_SC8800H5 
//#define  VSP_DEBUG
#ifdef VSP_DEBUG
#define VSP_PRINT printk
#else
#define VSP_PRINT(...)
#endif

#define BIT_13 (1<<13)
#define BIT_20 (1<<20)
#define BIT_21 (1<<21)
#define BIT_22 (1<<22)
#define BIT_23 (1<<23)
#define BIT_24 (1<<24)
#define BIT_25 (1<<25)
#define BIT_26 (1<<26)
#define DEFAULT_FREQ_DIV 0x4


#define AHB_CTL0                (SPRD_AHB_BASE + 0x200)
#define AHB_SOFT_RST            (SPRD_AHB_BASE + 0x210)

#define  SPRD_GREG_PLL_SCR     (SPRD_GREG_BASE + 0x70)
#define  VSP_96MHz      0x0
#define  VSP_64MHz      0x1
#define  VSP_48MHz      0x2
#define  VSP_26MHz      0x3

#define SPRD_VSP_BASE SPRD_MEA_BASE
#define SPRD_VSP_PHYS SPRD_MEA_PHYS
#define SPRD_VSP_SIZE 0x13000   //76k


#define SC8800G_VSP_IOCTL_MAGIC 'm'
#define VSP_CONFIG_FREQ _IOW(SC8800G_VSP_IOCTL_MAGIC, 1, unsigned int)
#define VSP_GET_FREQ    _IOR(SC8800G_VSP_IOCTL_MAGIC, 2, unsigned int)
#define VSP_ENABLE      _IO(SC8800G_VSP_IOCTL_MAGIC, 3)
#define VSP_DISABLE     _IO(SC8800G_VSP_IOCTL_MAGIC, 4)
#define VSP_ACQUAIRE    _IO(SC8800G_VSP_IOCTL_MAGIC, 5)
#define VSP_RELEASE     _IO(SC8800G_VSP_IOCTL_MAGIC, 6)


struct vsp_dev
{
    unsigned int freq_div;
    wait_queue_head_t	wait_queue;
    int  condition;  
    struct semaphore sem;
};

static struct vsp_dev dev;

static int vsp_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32_t reg_value; 
        int ret;
	switch(cmd)
	{
	    case VSP_CONFIG_FREQ:
	        get_user(dev.freq_div,(int __user *)arg);
		VSP_PRINT("vsp ioctl VSP_CONFIG_FREQ %d\n",dev.freq_div);
	    break;
	    case VSP_GET_FREQ:
#ifdef VSP_SC8800H5
		reg_value = __raw_readl(AHB_CTL0);
		reg_value = (reg_value>>20)&0xf;
#endif
#ifdef VSP_SC8800G2
                reg_value = __raw_readl(SPRD_GREG_PLL_SCR);
		reg_value = (reg_value>>2)&0x3;
#endif
		put_user(reg_value,(int __user *)arg);
		VSP_PRINT("vsp ioctl VSP_GET_FREQ %d\n",reg_value);
            break;
	    case VSP_ENABLE:
		VSP_PRINT("vsp ioctl VSP_ENABLE\n");
#ifdef VSP_SC8800H5
		reg_value = __raw_readl(AHB_CTL0);
		reg_value &= ~(BIT_20|BIT_21|BIT_22|BIT_23|BIT_24|BIT_25);
		reg_value |= (BIT_26|BIT_13);
		reg_value |= (dev.freq_div<<20); 
		__raw_writel(reg_value,AHB_CTL0);
		VSP_PRINT("vsp ioctl VSP_ENABLE m\n");
		reg_value = __raw_readl(AHB_SOFT_RST);
		__raw_writel(reg_value|(1<<15),AHB_SOFT_RST);
		__raw_writel(reg_value|(0<<15),AHB_SOFT_RST);
		VSP_PRINT("vsp ioctl VSP_ENABLE e\n");
#endif
#ifdef VSP_SC8800G2
		reg_value = __raw_readl(AHB_CTL0);
		reg_value |= (1<<13);
		__raw_writel(reg_value,AHB_CTL0);
		
		reg_value = __raw_readl(AHB_SOFT_RST);
		__raw_writel(reg_value|(1<<15),AHB_SOFT_RST);
		__raw_writel(reg_value|(0<<15),AHB_SOFT_RST);	

	        reg_value = __raw_readl(SPRD_GREG_PLL_SCR);
		reg_value &= ~0xc;
		reg_value |= (dev.freq_div<<2);
		__raw_writel(reg_value,SPRD_GREG_PLL_SCR);
#endif

#ifdef VSP_DEBUG
	__raw_writel(4,SPRD_VSP_BASE+0x40);
#endif
	    break;
	    case VSP_DISABLE:
		//todo
		VSP_PRINT("vsp ioctl VSP_DISABLE\n");
	    break;	
	    case VSP_ACQUAIRE:
		VSP_PRINT("vsp ioctl VSP_ACQUAIRE begin\n");
		ret = wait_event_interruptible_timeout(dev.wait_queue, dev.condition,msecs_to_jiffies(VSP_TIMEOUT_MS));
		if (ret == 0){
		    printk("KERN_ERR vsp error timeout\n");
		    dev.condition = 1;
		    return -ETIMEDOUT;
		}
		if(ret == -ERESTARTSYS){
		    printk("KERN_ERR vsp error -ERESTARTSYS\n");
		    return -ERESTARTSYS;
		}
		dev.condition = 0;
		VSP_PRINT("vsp ioctl VSP_ACQUAIRE end\n");
	    break;	
	    case VSP_RELEASE:
		VSP_PRINT("vsp ioctl VSP_RELEASE\n");
		dev.condition = 1;
		wake_up_interruptible(&dev.wait_queue);
	    break;	    
	    default:
		return -EINVAL;
	}
	return 0;
}

static int vsp_nocache_mmap(struct file *filp, struct vm_area_struct *vma)
{
	VSP_PRINT("@vsp[%s]\n", __FUNCTION__);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_pgoff     = (SPRD_VSP_PHYS>>PAGE_SHIFT);
	if(remap_pfn_range(vma,vma->vm_start,vma->vm_pgoff,vma->vm_end-vma->vm_start,vma->vm_page_prot))
	    return -EAGAIN;
	VSP_PRINT("@vsp mmap %x,%x,%x\n",(int)PAGE_SHIFT,(int)vma->vm_start,(int)vma->vm_end-vma->vm_start);
	return 0;
}

static const struct file_operations vsp_fops = 
{
	.owner = THIS_MODULE,
	.ioctl = vsp_ioctl,
	.mmap  = vsp_nocache_mmap,
};

static struct miscdevice vsp_dev = {
	.minor   = VSP_MINOR,
	.name   = "sc8800g_vsp",
	.fops   = &vsp_fops,
};


static int vsp_probe(struct platform_device *pdev)
{
	int ret = misc_register(&vsp_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				VSP_MINOR, ret);
		return ret;
	}

	init_waitqueue_head(&dev.wait_queue);
	dev.condition = 1;
#ifdef VSP_SC8800H5
	dev.freq_div = DEFAULT_FREQ_DIV;
#endif
#ifdef VSP_SC8800G2
	dev.freq_div = VSP_96MHz;
#endif
	return 0;
}

static int vsp_remove(struct platform_device *dev)
{
	printk(KERN_INFO "vsp_remove called !\n");

	misc_deregister(&vsp_dev);

	printk(KERN_INFO "vsp_remove Success !\n");
	return 0;
}


static struct platform_driver vsp_driver = {
	.probe    = vsp_probe,
	.remove   = vsp_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sc8800g_vsp",
	},
};

static int __init vsp_init(void)
{
	printk(KERN_INFO "vsp_init called !\n");
	if(platform_driver_register(&vsp_driver) != 0) {
		printk("platform device vsp drv register Failed \n");
		return -1;
	}
	return 0;
}

static void vsp_exit(void)
{
	printk(KERN_INFO "vsp_exit called !\n");
	platform_driver_unregister(&vsp_driver);
}

module_init(vsp_init);
module_exit(vsp_exit);

MODULE_DESCRIPTION("VSP Driver");
MODULE_LICENSE("GPL");
