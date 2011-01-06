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
#include <asm/cacheflush.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/sched.h>
#include <mach/sc8800g_2d.h>
#include <mach/dma.h>
#include <linux/delay.h>

#include "sc8800g_copybit_scale.h"
#include "sc8800g_copybit_rotation.h"
#include "sc8800g_copybit_lcdc.h"

#define SC8800G_2D_MINOR MISC_DYNAMIC_MINOR

static struct mutex *lock;
static wait_queue_head_t	wait_queue;
static  int  condition; 

#define SOFT_RGBA2ARGB
#ifdef SOFT_RGBA2ARGB
#define PMEM_BASE_PHY_ADDR 0x0f000000
#define PMEM_SIZE  0x800000*2
#define LCD_WIDTH  320
#define LCD_HEIGHT 480
/* FIXME */
unsigned int buf_ptr;
unsigned int buf_ptr_pa;
unsigned int pmem_ptr;

#define GET_VA(base) (base-PMEM_BASE_PHY_ADDR+pmem_ptr)
#endif

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

static void sprd_2d_dma_irq(int dma_ch, void *dev_id)
{
       // printk("[] sprd_2d_dma_irq()\n");
        condition = 1;
	wake_up_interruptible(&wait_queue);
}
static int do_copybit_dma_copy(struct s2d_blit_req * req,uint32_t byte_per_pixel)
{
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;
	uint32_t src_img_postm = (req->src.width-req->src_rect.w)*byte_per_pixel;
	uint32_t dst_img_postm =( req->dst.width-req->dst_rect.w)*byte_per_pixel;
	uint32_t src_addr = req->src.base + (req->src_rect.y*req->src.width + req->src_rect.x)*byte_per_pixel;
	uint32_t dst_addr = req->dst.base + (req->dst_rect.y*req->dst.width + req->dst_rect.x)*byte_per_pixel;
	uint32_t block_len = req->dst_rect.w*byte_per_pixel;
	uint32_t total_len = req->dst_rect.w*req->dst_rect.h*byte_per_pixel;
        uint32_t ret = 0;

       //printk("[pid:%d] do_copybit_dma_copy() %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);

	ret  = sprd_request_dma(DMA_SOFT0, sprd_2d_dma_irq, req);
        if(ret){
        	return -EFAULT;
        }
	condition = 0;
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;
	sprd_dma_setup_cfg(&ctrl,
                DMA_SOFT0,
                DMA_NORMAL,
                TRANS_DONE_EN,
                DMA_INCREASE, DMA_INCREASE,
                SRC_BURST_MODE_8|src_img_postm, SRC_BURST_MODE_8|dst_img_postm,
                block_len,
                byte_per_pixel*8, byte_per_pixel*8,
                src_addr, dst_addr, total_len);
	 sprd_dma_setup(&ctrl);
	 sprd_dma_start(DMA_SOFT0);
	__raw_bits_or(1 << DMA_SOFT0, DMA_SOFT_REQ);	 
	 if(wait_event_interruptible(wait_queue, condition)){
	 	ret =  -EFAULT;
	 }
	 //mdelay(100);
	 sprd_dma_stop(DMA_SOFT0);
	 sprd_free_dma(DMA_SOFT0);
	 return ret;
}

static int sc8800g_2d_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct s2d_blit_req *params;
	int dma_copy_ret = -1;
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
	
#ifdef SOFT_RGBA2ARGB
	/* we assume RGBA8888 format data is from pmem */
	if ((params->src.format == S2D_BGRA_8888) &&(params->dst.format != S2D_BGRA_8888)){
		/* it's RGBA actually, so we need to change it first */
		int i, j;
		unsigned int *src, *src_base, *dst;

		src_base = (unsigned int *)GET_VA(params->src.base);
		src_base += params->src_rect.y * params->src.width + 
				params->src_rect.x;
		dst = (unsigned int *)buf_ptr;

		for (i = params->src_rect.h; i!=0; i--) {
			src = src_base;
			for (j = params->src_rect.w; j!=0; j--) {
				*dst++ = ((*src)<<8) | ((*src)>>24);
				src++;
			}
			src_base += params->src.width;
		}

		clean_dcache_area((unsigned int *)buf_ptr, 
			(unsigned int)dst - buf_ptr);

		params->src.width = params->src_rect.w;
		params->src.height = params->src_rect.h;
		//params->src.format = S2D_ARGB_8888;
		params->src.base = __pa(buf_ptr);
		params->src_rect.x = 0;
		params->src_rect.y = 0;
	}
#endif

        if((params->alpha==0xff)&&(params->src.format==S2D_RGB_565)&&(params->dst.format==S2D_RGB_565)\
		&&(params->src_rect.w==params->dst_rect.w)&&(params->src_rect.h==params->dst_rect.h))
        {
        	dma_copy_ret = do_copybit_dma_copy(params,2);
        }

	if(dma_copy_ret){
		if (do_copybit_scale(params)) {
			mutex_unlock(lock);
			return -EFAULT;
		}
	
		if (do_copybit_rotation(params)) {
			mutex_unlock(lock);
			return -EFAULT;
		}
		
		if (do_copybit_lcdc(params)) {
			mutex_unlock(lock);
			return -EFAULT;
		}
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
	init_waitqueue_head(&wait_queue);
	
#ifdef SOFT_RGBA2ARGB
	/* FIXME: hard-coded size and address */
	buf_ptr = __get_free_pages(GFP_ATOMIC, get_order(LCD_WIDTH*LCD_HEIGHT*4));
	pmem_ptr = (unsigned int)ioremap_cached(PMEM_BASE_PHY_ADDR, PMEM_SIZE);
	printk("sc8800g_2d alloc buf @ 0x%x\n", buf_ptr);
	buf_ptr_pa = __pa(buf_ptr);
#endif

	printk(KERN_ALERT" sc8800g_2d_probe Success\n");

	return 0;
}


static int sc8800g_2d_remove(struct platform_device *dev)
{
	printk(KERN_INFO "sc8800g_2d_remove called !\n");

	misc_deregister(&sc8800g_2d_dev);

#ifdef SOFT_RGBA2ARGB
	free_pages(buf_ptr, get_order(LCD_WIDTH*LCD_HEIGHT*4));
	iounmap((void*)PMEM_BASE_PHY_ADDR);
#endif

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
