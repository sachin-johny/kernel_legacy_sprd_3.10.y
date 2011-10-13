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
#include <mach/board.h>
#include <linux/delay.h>

#include "sc8800g_copybit_scale.h"
#include "sc8800g_copybit_rotation.h"
#include "sc8800g_copybit_lcdc.h"

#define CROP_OUTPUT_BUF SPRD_SCALE_MEM_BASE
#define SC8800G_2D_MINOR MISC_DYNAMIC_MINOR

static struct mutex *lock;
static wait_queue_head_t	wait_queue;
static  int  condition; 
static wait_queue_head_t	wait_queue_crop;
static  int  condition_crop; 

#define SOFT_RGBA2ARGB
#ifdef SOFT_RGBA2ARGB
#define PMEM_BASE_PHY_ADDR SPRD_PMEM_BASE 
#define PMEM_SIZE  SPRD_IO_MEM_SIZE
#define LCD_WIDTH  320
#define LCD_HEIGHT 480
/* FIXME */
unsigned int buf_ptr;
unsigned int buf_ptr_cached;
unsigned int buf_ptr_pa;
unsigned int pmem_ptr;
unsigned int pmem_ptr_cached;


//#define COPYBIT_2D_DEBUG

#ifdef COPYBIT_2D_DEBUG
#define C2D_PRINT printk
#else
#define C2D_PRINT(...)
#endif

#define GET_VA(base) (base-PMEM_BASE_PHY_ADDR+pmem_ptr)
#endif

int sc8800g_2d_open(struct inode *inode, struct file *file)
{
	struct s2d_blit_req_list *params;
    //printk("VMALLOC_START=0x%x,VMALLOC_END=0x%x,PAGE_OFFSET=0x%x\n",VMALLOC_START,VMALLOC_END,PAGE_OFFSET);
	params = (struct s2d_blit_req_list *)kmalloc(
			sizeof(struct s2d_blit_req_list), GFP_KERNEL);

	if(params == NULL) {
		printk(KERN_ERR "Instance memory allocation was failed\n");
		return -1;
	}

	memset(params, 0, sizeof(struct s2d_blit_req_list));

	file->private_data = (struct s2d_blit_req_list *)params;

	printk("[pid:%d] sc8800g_2d_open()\n", current->pid);

	return 0;
}


int sc8800g_2d_release(struct inode *inode, struct file *file)
{
	struct s2d_blit_req_list *params;

	params = (struct s2d_blit_req_list *)file->private_data;
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

	//C2D_PRINT("[pid:%d] do_copybit_dma_copy() %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);

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
	//C2D_PRINT("[pid:%d] do_copybit_dma_copy() before %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);
	if(wait_event_interruptible(wait_queue, condition)){
		ret =  -EFAULT;
	}
	//C2D_PRINT("[pid:%d] do_copybit_dma_copy() after %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);
	//mdelay(100);
	sprd_dma_stop(DMA_SOFT0);
	sprd_free_dma(DMA_SOFT0);
	return ret;
}
#if 0
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
		if(params->do_flags & (BIT_0 | BIT_1)) {
			if (do_copybit_scale(params)) {
				mutex_unlock(lock);
				return -EFAULT;
			}			
		}		
		if(params->do_flags & BIT_2) {
			if (do_copybit_rotation(params)) {
				mutex_unlock(lock);
				return -EFAULT;
			}			
		}		
		if(params->do_flags & BIT_3) {			
			if (do_copybit_lcdc(params)) {
				mutex_unlock(lock);
				return -EFAULT;
			}			
		}
	}
	mutex_unlock(lock);

	return 0;
}
#endif

static void sprd_2d_crop_dma_irq(int dma_ch, void *dev_id)
{
       // printk("[] sprd_2d_dma_irq()\n");
        condition_crop = 1;
	wake_up_interruptible(&wait_queue_crop);
}

static int crop_by_dma(struct s2d_blit_req * req,uint32_t byte_per_pixel)
{
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;
	uint32_t src_img_postm = 0;//(req->src.width-req->src_rect.w)*byte_per_pixel;
	uint32_t dst_img_postm = 0;//( req->dst.width-req->dst_rect.w)*byte_per_pixel;
	uint32_t src_addr = 0;//req->src.base + (req->src_rect.y*req->src.width + req->src_rect.x)*byte_per_pixel;
	uint32_t dst_addr = CROP_OUTPUT_BUF;//req->dst.base + (req->dst_rect.y*req->dst.width + req->dst_rect.x)*byte_per_pixel;
	uint32_t block_len =0;// req->dst_rect.w*byte_per_pixel;
	uint32_t total_len = 0;//req->dst_rect.w*req->dst_rect.h*byte_per_pixel;
	uint32_t ret = 0, temp_w, temp_h;

	temp_w = req->src_rect.w;
	temp_h = req->src_rect.h;
	if(S2D_RGB_565 == req->src.format){
		if(req->src_rect.w & 0x1)
			temp_w = req->src_rect.w + 1;
		if(req->src_rect.h & 0x1)
			temp_h = req->src_rect.h + 1;
	}
	src_img_postm = (req->src.width-temp_w)*byte_per_pixel;
	src_addr = req->src.base + (req->src_rect.y* req->src.width+ req->src_rect.x)*byte_per_pixel;
	block_len = temp_w*byte_per_pixel;
	total_len = temp_w * temp_h * byte_per_pixel;

	C2D_PRINT("[pid:%d] crop_by_dma() %d,%d,%d,%d\n", current->pid,req->src_rect.x,req->src_rect.y,temp_w,temp_h);
	C2D_PRINT("crop_by_dma() %d,%d,%d,%d\n",src_img_postm,src_addr,block_len,total_len);

	/*ret  = sprd_request_dma(DMA_SOFT0, sprd_2d_crop_dma_irq, req);
	if(ret){
		return -EFAULT;
	}*/
	while(1){		
		ret  = sprd_request_dma(DMA_SOFT0, sprd_2d_crop_dma_irq, req);
        	if(ret){
        		printk("copybit crop_by_dma: request dma fail.ret : %d.\n", ret);
        		msleep(5); 
        	}
		else{
			break;
		}			
	}
	condition_crop = 0;
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
	//C2D_PRINT("[pid:%d] do_copybit_dma_copy() before %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);
	if(wait_event_interruptible(wait_queue_crop, condition_crop)){
		ret =  -EFAULT;
        	printk("copybit crop_by_dma: wait_event_interruptible fail.ret : %d.\n", ret);
	}
	//C2D_PRINT("[pid:%d] do_copybit_dma_copy() after %d,%d,%d,%d\n", current->pid,req->dst_rect.x,req->dst_rect.y,req->dst_rect.w,req->dst_rect.h);
	//mdelay(100);
	sprd_dma_stop(DMA_SOFT0);
	sprd_free_dma(DMA_SOFT0);

	req->src.base = CROP_OUTPUT_BUF;
	req->src.width = temp_w;
	req->src.height = temp_h;
	req->src_rect.x = 0;
	req->src_rect.y = 0;

	return ret;
}

//wxz20110607: crop the rect data from scr image for the rotation.
static int do_copybit_crop_by_dma(struct s2d_blit_req * req)
{
	uint32_t byte_per_pixel = 2;
	
	switch(req->src.format){
		case S2D_RGB_565:
			byte_per_pixel = 2;
			break;
		case S2D_BGRA_8888:
			byte_per_pixel = 4;
			break;
		default:
			C2D_PRINT("do_copybit_crop_by_dma: not support the color format. format: %d.\n", req->src.format);
			return -1;
	}

	return crop_by_dma(req, byte_per_pixel);	
}

static inline void clean_cache_all(void)
{
	__asm__ __volatile__(
"1:		mrc	p15, 0, r15, c7, c10, 3 \n\t"
		"bne	1b\n\t"
	::);
}

extern void format_convert(unsigned int *src,unsigned int *dst,unsigned int num);
extern void format_convert_with_crop(unsigned int *src,unsigned int *dst,unsigned int *parm);
extern void format_convert_with_crop_non_8w(unsigned int *src,unsigned int *dst,unsigned int *parm);

static int sc8800g_2d_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg){
	struct s2d_blit_req_list *parameters;
	int dma_copy_ret = -1, num = 0;
	struct s2d_blit_req *params;
	int i, j;
	unsigned int *src, *src_base, *dst;
	unsigned int param[4] ={0};
	parameters = (struct s2d_blit_req_list*)file->private_data;
	if (copy_from_user(parameters, 
				(struct s2d_blit_req_list*)arg, sizeof(struct s2d_blit_req_list)))
		return -2;
	C2D_PRINT("sc8800g_2d_ioctl: count: %d.\n", parameters->count);

	//printk("[pid:%d] sc8800g_2d_ioctl()\n", current->pid);
	mutex_lock(lock);

	if (cmd != SC8800G_2D_BLIT) {
		mutex_unlock(lock);
		printk(KERN_ERR "sc8800g_2d: unknown ioctl cmd\n");
		return -3;
	}

	/* FIXME: work around for pmem flush failure */
	if (parameters->req[0].src.base >= PMEM_BASE_PHY_ADDR)
		clean_cache_all();

	for(num = 0; num < parameters->count; num++){
		
		params = &parameters->req[num];

	C2D_PRINT("sc8800g_2d_ioctl num: %d, do_flags: %d,alpha:%d, flags: %d, dst:{%d, %d,%d,0x%x}, src:{%d,%d,%d,0x%x}, dst_rect:{%d, %d, %d,%d}, src_rect:{%d,%d,%d,%d}\n",
		num,params->do_flags,params->alpha, params->flags,
		params->dst.width, params->dst.height,params->dst.format,params->dst.base,
		params->src.width, params->src.height,params->src.format,params->src.base,
		params->dst_rect.x, params->dst_rect.y, params->dst_rect.w, params->dst_rect.h,
		params->src_rect.x, params->src_rect.y, params->src_rect.w, params->src_rect.h);
				
#ifdef SOFT_RGBA2ARGB
		/* we assume RGBA8888 format data is from pmem */
		if ((params->src.format == S2D_BGRA_8888) &&(params->dst.format != S2D_BGRA_8888)){
			/* it's RGBA actually, so we need to change it first */

			/* FIXME: work around for pmem flush failure */
			flush_cache_all();


			src_base = (unsigned int *)GET_VA(params->src.base);
			src_base += params->src_rect.y * params->src.width + 
				params->src_rect.x;
			dst = (unsigned int *)buf_ptr_cached;

			if(params->src_rect.w&0x7)
			{						
				if(params->src_rect.w>8)
				{
					param[0] = (params->src.width-params->src_rect.w)<<2;
					param[1] = params->src_rect.w-(params->src_rect.w&0x7);
					param[2] = params->src_rect.w&0x7;
					param[3] = params->src_rect.h;
					src = src_base;					
					format_convert_with_crop_non_8w(src,dst,&param[0]);
					dst += params->src_rect.w*params->src_rect.h;
				}
				else
				{
					for (i = params->src_rect.h; i!=0; i--) {
					src = src_base;
					for (j = params->src_rect.w; j!=0; j--) {
						*dst++ = ((*src)<<8) | ((*src)>>24);
						src++;
					}
					src_base += params->src.width;
					}
				}

			}
			else if(params->src.width==params->src_rect.w)
			{					
				src = src_base;	
			         format_convert(src,dst,params->src_rect.w*params->src_rect.h);
				dst += params->src_rect.w*params->src_rect.h;
			}
			else
			{				
				param[0] = params->src.width;
				param[1] = params->src_rect.w;
				param[2] = params->src_rect.h;
				src = src_base;	
				format_convert_with_crop(src,dst,&param[0]);
				dst += params->src_rect.w*params->src_rect.h;
			}
			clean_dcache_area((unsigned int *)buf_ptr_cached, 
					(unsigned int)dst - buf_ptr_cached);
			params->src.width = params->src_rect.w;
			params->src.height = params->src_rect.h;
			//params->src.format = S2D_ARGB_8888;
			params->src.base = __pa(buf_ptr_cached);
			params->src_rect.x = 0;
			params->src_rect.y = 0;			
		}
#endif	
		if((params->alpha==0xff)&&(params->src.format==S2D_RGB_565)&&(params->dst.format==S2D_RGB_565)\
				&&(params->src_rect.w==params->dst_rect.w)&&(params->src_rect.h==params->dst_rect.h)\
				&&(0 == (params->do_flags & BIT_1)))
		{
			dma_copy_ret = do_copybit_dma_copy(params,2);
		}
		
		if(dma_copy_ret){			
			//if(params->do_flags & (BIT_0 | BIT_1)) {
			if(params->do_flags & BIT_0) {
				if (do_copybit_scale(params)) {
					mutex_unlock(lock);
					return -4;
				}			
			}
			else if(params->do_flags & BIT_3){
				if (do_copybit_crop_by_dma(params)) {
					mutex_unlock(lock);
					return -5;
				}				
			}
			//if(params->do_flags & BIT_2) {
			if(params->do_flags & BIT_1) {
				if (do_copybit_rotation(params)) {
					mutex_unlock(lock);
					return -6;
				}			
			}		
			//if(params->do_flags & BIT_3) {
			if(params->do_flags & BIT_2) {
				if (do_copybit_lcdc(params)) {
					mutex_unlock(lock);
					return -7;
				}			
			}
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


extern unsigned int fb_pa;
extern unsigned int fb_va;
extern unsigned int fb_va_cached;

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
	init_waitqueue_head(&wait_queue_crop);

#ifdef SOFT_RGBA2ARGB
	/* FIXME: hard-coded size and address */
	buf_ptr_cached = __get_free_pages(GFP_ATOMIC, get_order(LCD_WIDTH*LCD_HEIGHT*4));
	buf_ptr_pa = __pa(buf_ptr_cached);
	buf_ptr = (unsigned int)ioremap(buf_ptr_pa, LCD_WIDTH*LCD_HEIGHT*4);
	pmem_ptr = (unsigned int)ioremap(PMEM_BASE_PHY_ADDR, PMEM_SIZE);
	pmem_ptr_cached = (unsigned int)ioremap_cached(PMEM_BASE_PHY_ADDR, PMEM_SIZE);
	printk("sc8800g_2d alloc buf @ 0x%x\n", buf_ptr);
#endif

	printk(KERN_ALERT" sc8800g_2d_probe Success\n");

	return 0;
}


static int sc8800g_2d_remove(struct platform_device *dev)
{
	printk(KERN_INFO "sc8800g_2d_remove called !\n");

	misc_deregister(&sc8800g_2d_dev);

#ifdef SOFT_RGBA2ARGB
	free_pages(buf_ptr_cached, get_order(LCD_WIDTH*LCD_HEIGHT*4));
	iounmap((void*)buf_ptr);
	iounmap((void*)pmem_ptr);
	iounmap((void*)pmem_ptr_cached);
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
