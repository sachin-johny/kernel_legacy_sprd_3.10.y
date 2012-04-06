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
#include <linux/sched.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

#include <mach/clock_common.h>

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

#define  VSP_154MHz      0x0
#define  VSP_128MHz      0x1
#define  VSP_64MHz      0x2
#define  VSP_48MHz      0x3

#define SPRD_VSP_BASE SPRD_MEA_BASE
#define SPRD_VSP_PHYS SPRD_MEA_PHYS
#define SPRD_VSP_SIZE 0x13000   //76k

#define DCAM_CFG_OFF						0x0
#define DCAM_SRC_SIZE_OFF					0xC
#define DCAM_ISP_DIS_SIZE_OFF				0x10
#define DCAM_VSP_TIME_OUT_OFF				0x14
#define DCAM_INT_STS_OFF					0x20
#define DCAM_INT_MASK_OFF					0x24
#define DCAM_INT_CLR_OFF					0x28
#define DCAM_INT_RAW_OFF					0x2C



#define SC8800G_VSP_IOCTL_MAGIC 'm'
#define VSP_CONFIG_FREQ _IOW(SC8800G_VSP_IOCTL_MAGIC, 1, unsigned int)
#define VSP_GET_FREQ    _IOR(SC8800G_VSP_IOCTL_MAGIC, 2, unsigned int)
#define VSP_ENABLE      _IO(SC8800G_VSP_IOCTL_MAGIC, 3)
#define VSP_DISABLE     _IO(SC8800G_VSP_IOCTL_MAGIC, 4)
#define VSP_ACQUAIRE    _IO(SC8800G_VSP_IOCTL_MAGIC, 5)
#define VSP_RELEASE     _IO(SC8800G_VSP_IOCTL_MAGIC, 6)
#define VSP_START     _IO(SC8800G_VSP_IOCTL_MAGIC, 7)

struct vsp_dev{
    unsigned int freq_div;
    wait_queue_head_t	wait_queue;
    int  condition;  
    wait_queue_head_t	wait_queue_work;
    int  condition_work;  	
    int  vsp_int_status;	
    struct semaphore sem;
    struct clk *vsp_clk; 
};

static struct vsp_dev dev;

static char * vsp_get_clk_src_name(unsigned int clk_src)
{
        char *src_name;
	switch(clk_src){
		case VSP_154MHz:
			src_name =  "l3_153m600k";
			break;
		case VSP_128MHz:
			src_name =  "clk_128m";			
			break;
		case VSP_64MHz:
			src_name =  "clk_64m";			
			break;
		default:
			src_name =  "clk_48m";				
			break;
	}	
	return src_name;
}

static inline int rt_policy(int policy)
{
	if (unlikely(policy == SCHED_FIFO) || unlikely(policy == SCHED_RR))
		return 1;
	return 0;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}


#define __wait_event_interruptible_timeout_exclusive(wq, condition, ret)		\
do {									\
	DEFINE_WAIT(__wait);						\
									\
	for (;;) {							\
		prepare_to_wait_exclusive(&wq, &__wait, TASK_INTERRUPTIBLE);	\
		if (condition)						\
			break;						\
		if (!signal_pending(current)) {				\
			ret = schedule_timeout(ret);			\
			if (!ret)					\
				break;					\
			continue;					\
		}							\
		ret = -ERESTARTSYS;					\
		break;							\
	}								\
	finish_wait(&wq, &__wait);					\
} while (0)

#define wait_event_interruptible_timeout_exclusive(wq, condition, timeout)	\
({									\
	long __ret = timeout;						\
	if (!(condition))						\
		__wait_event_interruptible_timeout_exclusive(wq, condition, __ret); \
	__ret;								\
})


	
static int vsp_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32_t reg_value; 
        int ret;
	struct clk *clk_parent;
	char *name_parent;
	int ret_val;	
	
	switch(cmd){
	    case VSP_CONFIG_FREQ:
	        get_user(dev.freq_div,(int __user *)arg);
		//force disable clk
		clk_disable_force(dev.vsp_clk);
		//select new parent clk	
		name_parent =vsp_get_clk_src_name(dev.freq_div);
		clk_parent = clk_get(NULL, name_parent);
		if (!clk_parent) {
			printk("clock[%s]: failed to get parent [%s] by clk_get()!\n",
				dev.vsp_clk->name, name_parent);
			return -EINVAL;
		}
		ret_val = clk_set_parent(dev.vsp_clk, clk_parent);
		if (ret_val) {
			printk("clock[%s]: clk_set_parent() failed!", dev.vsp_clk->name);
			return -EINVAL;
		}			
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
#if 0
		reg_value = __raw_readl(AHB_CTL0);
		reg_value |= (1<<13);
		__raw_writel(reg_value,AHB_CTL0);

	        reg_value = __raw_readl(SPRD_GREG_PLL_SCR);
		reg_value &= ~0xc;
		reg_value |= (dev.freq_div<<2);
		__raw_writel(reg_value,SPRD_GREG_PLL_SCR);
#endif
		ret = clk_enable(dev.vsp_clk);
		if(ret){
			printk("clock[%s]:clk_enable() failed!\n",dev.vsp_clk->name);
		}
//#ifdef CONFIG_ARCH_SC8810
//		reg_value = __raw_readl(AHB_CTL0);
//		reg_value |= ((1<<28) | (1<<27) | (1<<13));
//		__raw_writel(reg_value,AHB_CTL0);
//#endif		
		reg_value = __raw_readl(AHB_SOFT_RST);
		__raw_writel(reg_value|(1<<15),AHB_SOFT_RST);
		__raw_writel(reg_value|(0<<15),AHB_SOFT_RST);	
#endif

#ifdef VSP_DEBUG
	__raw_writel(4,SPRD_VSP_BASE+0x40);
#endif
	    break;
	    case VSP_DISABLE:
	       //clk_disable(dev.vsp_clk);
	       clk_disable_force(dev.vsp_clk);
		VSP_PRINT("vsp ioctl VSP_DISABLE\n");
	    break;	
	    case VSP_ACQUAIRE:
		VSP_PRINT("vsp ioctl VSP_ACQUAIRE begin\n");
		ret = wait_event_interruptible_timeout_exclusive(dev.wait_queue, dev.condition,msecs_to_jiffies(VSP_TIMEOUT_MS));
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
/*
		if (!task_has_rt_policy(current)) {
		    struct sched_param schedpar;
		    int ret;
		    struct cred *new = prepare_creds();
		    cap_raise(new->cap_effective, CAP_SYS_NICE);
		    commit_creds(new);
		    schedpar.sched_priority = 1;
		    ret = sched_setscheduler(current,SCHED_RR,&schedpar);
		    if(ret!=0)
		        printk("vsp change pri fail a\n");
		}
*/
		VSP_PRINT("vsp ioctl VSP_ACQUAIRE end\n");
	    break;	
	    case VSP_RELEASE:
		VSP_PRINT("vsp ioctl VSP_RELEASE\n");
		dev.condition = 1;
		wake_up_interruptible_nr(&dev.wait_queue,1);
	    break;
	    case VSP_START:
		VSP_PRINT("vsp ioctl VSP_START\n");
		ret = wait_event_interruptible_timeout(dev.wait_queue_work, dev.condition_work, msecs_to_jiffies(VSP_TIMEOUT_MS));
		if(ret == -ERESTARTSYS){
		    printk("KERN_ERR vsp error start -ERESTARTSYS\n");
		    dev.vsp_int_status |= 1<<30;
		    ret = -EINVAL;	
		}else if(ret == 0){
		    printk("KERN_ERR vsp error start  timeout\n");
		    dev.vsp_int_status |= 1<<31;
		    ret = -ETIMEDOUT;
		}else{
		    ret = 0;
		}
		if(ret){
		//clear vsp int
			__raw_writel((1<<10)|(1<<12)|(1<<15),SPRD_VSP_BASE+DCAM_INT_CLR_OFF);
		}
		put_user(dev.vsp_int_status,(int __user *)arg);
		dev.vsp_int_status = 0;		
		dev.condition_work = 0;
		VSP_PRINT("vsp ioctl VSP_START end\n");	       
	       return ret;
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

static irqreturn_t vsp_isr(int irq, void *data)
{
	dev.vsp_int_status = __raw_readl(SPRD_VSP_BASE+DCAM_INT_STS_OFF);
	__raw_writel((1<<10)|(1<<12)|(1<<15),SPRD_VSP_BASE+DCAM_INT_CLR_OFF);
	dev.condition_work = 1;
	wake_up_interruptible(&dev.wait_queue_work);
	return IRQ_HANDLED;
}

static int vsp_probe(struct platform_device *pdev)
{
	struct clk *clk_parent;
	char *name_parent;
	int ret_val;
	
	int ret = misc_register(&vsp_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				VSP_MINOR, ret);
		return ret;
	}

	init_waitqueue_head(&dev.wait_queue);
	dev.condition = 1;	

	init_waitqueue_head(&dev.wait_queue_work);
	dev.vsp_int_status = 0;		
	dev.condition_work = 0;
	
#ifdef VSP_SC8800H5
	dev.freq_div = DEFAULT_FREQ_DIV;
#endif
#ifdef VSP_SC8800G2
	dev.freq_div = VSP_154MHz;
#endif
	dev.vsp_clk = clk_get(NULL,"clk_vsp");
	if (IS_ERR(dev.vsp_clk)) {
		printk("###: Failed : Can't get clock [%s}!\n","clk_vsp");
		printk("###: vsp_clk =  %p\n",dev.vsp_clk);
		return -EINVAL;
	}
	name_parent =vsp_get_clk_src_name(dev.freq_div);
	clk_parent = clk_get(NULL, name_parent);
	if (!clk_parent) {
		printk("clock[%s]: failed to get parent in probe[%s] by clk_get()!\n",
				dev.vsp_clk->name, name_parent);
		return -EINVAL;
	}

	ret_val = clk_set_parent(dev.vsp_clk, clk_parent);
	if (ret_val) {
		printk("clock[%s]: clk_set_parent() failed in probe!", dev.vsp_clk->name);
		return -EINVAL;
	}	

#if 1
	/* register isr */
	ret = request_irq(IRQ_VSP_INT, vsp_isr, 0, "VSP", &dev);
	if (ret) {
		printk(KERN_ERR "vsp: failed to request irq!\n");
		return -EINVAL;
	}
#endif	
	return 0;
}

static int vsp_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "vsp_remove called !\n");

	misc_deregister(&vsp_dev);

	clk_put(dev.vsp_clk);
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
