/*
* drivers/media/video/sprd_rotation/rotation_sc8800g2.c
 * Rotation driver based on sc8800g
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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
#include <mach/dma.h>
#include <linux/sched.h>
#include "rotation_sc8800g2.h"
#include "rotation_reg_sc8800g2.h"
#include <linux/slab.h>

//#define ROTATION_DEBUG
#ifdef ROTATION_DEBUG
#define RTT_PRINT printk
#else
#define RTT_PRINT(...)
#endif

#define BOOLEAN char
#define ROTATION_TRUE 1
#define ROTATION_FALSE 0
#define DISABLE_AHB_SLEEP 0
#define ENABLE_AHB_SLEEP 1

typedef struct _dma_rotation_tag
{
    ROTATION_SIZE_T         img_size;    
    ROTATION_DATA_FORMAT_E   data_format;
    ROTATION_DIR_E         rotation_dir; 
    ROTATION_DATA_ADDR_T    src_addr;
    ROTATION_DATA_ADDR_T    dst_addr;  
    uint32_t                 s_addr;
    uint32_t                 d_addr;    
    ROTATION_PIXEL_FORMAT_E pixel_format;
    ROTATION_UV_MODE_E      uv_mode; 
    BOOLEAN                 is_end;
}ROTATION_CFG_T, *ROTATION_CFG_T_PTR;

static ROTATION_CFG_T s_rotation_cfg;

#define ALGIN_FOUR      0x03
#define DECLARE_ROTATION_PARAM_ENTRY(s) ROTATION_CFG_T *s=&s_rotation_cfg
#define ROTATION_MINOR MISC_DYNAMIC_MINOR

typedef void (*ROTATION_IRQ_FUNC) (uint32_t);
static struct mutex *lock;
static wait_queue_head_t	wait_queue;
static  int  condition; 
struct semaphore g_sem_rot;

static int rotation_check_param(ROTATION_PARAM_T* param_ptr)
{
    if(NULL== param_ptr) 
    {
	RTT_PRINT("Rotation: the param ptr is null.\n");
	return -1;
    }   

    if((param_ptr->src_addr.y_addr&ALGIN_FOUR)
        ||(param_ptr->src_addr.uv_addr&ALGIN_FOUR)
        ||(param_ptr->src_addr.v_addr&ALGIN_FOUR)
        ||(param_ptr->dst_addr.y_addr&ALGIN_FOUR)
        ||(param_ptr->dst_addr.uv_addr&ALGIN_FOUR)
        ||(param_ptr->dst_addr.v_addr&ALGIN_FOUR))
    {
        RTT_PRINT("Rotation: the addr not algin.\n");
	return -1;
    }

    if(ROTATION_RGB565<param_ptr->data_format)
    {
        RTT_PRINT("Rotation: data for err : %d.\n", param_ptr->data_format);
	return -1;
    }
   
    return 0;
}

static ROTATION_PIXEL_FORMAT_E rotation_get_pixel_format(void)
{
    DECLARE_ROTATION_PARAM_ENTRY(s);
    
    switch(s->data_format)
    {
        case ROTATION_YUV422:
        case ROTATION_YUV420:
        case ROTATION_YUV400:  
            s->pixel_format=ROTATION_ONE_BYTE;
            break ;

        case ROTATION_RGB565:
            s->pixel_format=ROTATION_TWO_BYTES;
            break ;            
            
        case ROTATION_RGB888:
        case ROTATION_RGB666:
            s->pixel_format=ROTATION_FOUR_BYTES;
            break ;

        default :
            break;
    }

    return s->pixel_format;    
}
static BOOLEAN rotation_get_isend(void)
{
    DECLARE_ROTATION_PARAM_ENTRY(s);
    
    switch(s->data_format)
    {
        case ROTATION_YUV422:
        case ROTATION_YUV420:
            s->is_end=ROTATION_FALSE;
            break ;

        case ROTATION_YUV400:  
        case ROTATION_RGB888:
        case ROTATION_RGB565:            
        case ROTATION_RGB666:
            s->is_end=ROTATION_TRUE;
            break ;

        default :
            break;
    }   

    return s->is_end;    
}

static int rotation_set_y_param(ROTATION_PARAM_T* param_ptr)
{
    DECLARE_ROTATION_PARAM_ENTRY(s);

    memcpy((void*)&(s->img_size),(void*)&(param_ptr->img_size),sizeof(ROTATION_SIZE_T)); 
    memcpy((void*)&(s->src_addr),(void*)&(param_ptr->src_addr),sizeof(ROTATION_DATA_ADDR_T)); 
    memcpy((void*)&(s->dst_addr),(void*)&(param_ptr->dst_addr),sizeof(ROTATION_DATA_ADDR_T));  

    s->s_addr=param_ptr->src_addr.y_addr;
    s->d_addr=param_ptr->dst_addr.y_addr;    
    s->data_format=param_ptr->data_format;
    s->rotation_dir=param_ptr->rotation_dir;  

    s->pixel_format=rotation_get_pixel_format();
    s->is_end=rotation_get_isend();
 
    s->uv_mode=ROTATION_NORMAL;    

    return 0;
    
}

static void rotation_cfg (void)
{
    // rot eb
    _paod(AHB_GLOBAL_REG_CTL0, BIT_14); //ROTATION_DRV_ONE

    // rot soft reset
    _paod(AHB_GLOBAL_REG_SOFTRST, BIT_10);
    _paad(AHB_GLOBAL_REG_SOFTRST, ~BIT_10);
}


static void rotation_disable(void)
{
	  // rot eb
    _paad(AHB_GLOBAL_REG_CTL0, ~BIT_14); //ROTATION_DRV_ONE
}


static void rotation_software_reset(void)
{
    // rot soft reset
    _paod(AHB_GLOBAL_REG_SOFTRST, BIT_10);   
    _paad(AHB_GLOBAL_REG_SOFTRST, ~BIT_10);
}

static void rotation_set_src_addr(uint32_t src_addr)
{
    _pawd(REG_ROTATION_SRC_ADDR, src_addr);

    return;
}
static void rotation_set_dst_addr(uint32_t dst_addr)
{
    _pawd(REG_ROTATION_DST_ADDR, dst_addr);

    return;
}
static void rotation_set_img_size (ROTATION_SIZE_T *size)
{
    _paad(REG_ROTATION_IMG_SIZE, 0xFF000000);
    _paod(REG_ROTATION_IMG_SIZE, (size->h & 0xFFF) | ((size->w & 0xFFF) << 12));

     _pawd(REG_ROTATION_ORIGWIDTH, size->w & 0xFFF);

    return ;
}
static void rotation_set_pixel_mode (ROTATION_PIXEL_FORMAT_E pixel_format)
{
    _paad(REG_ROTATION_IMG_SIZE, ~(0x3 << 24));
    _paod(REG_ROTATION_IMG_SIZE, pixel_format << 24);

    return ;
}
static void rotation_set_dir (ROTATION_DIR_E rotation_dir)
{
    _paad(REG_ROTATION_CTRL, ~(0x3 << 1));
    _paod(REG_ROTATION_CTRL, (rotation_dir & 0x3) << 1);

    return ;
}
static void rotation_set_UV_mode (ROTATION_UV_MODE_E uv_mode)
{
    _paad(REG_ROTATION_CTRL, ~BIT_0);
    _paod(REG_ROTATION_CTRL, uv_mode & 0x1);

    return ;
}
static void rotation_enable (void)
{
    _paod(REG_ROTATION_CTRL, BIT_3);

    return ;
}
#ifdef ROTATION_DEBUG //for debug
static void get_rotation_reg(void)
{
  uint32_t i, value;
  RTT_PRINT("###############get_rotation_reg##########################\n");
  for(i = 0; i < 12; i++)
  {
    value = _pard(REG_ROTATION_SRC_ADDR + i * 4);
    RTT_PRINT("ROT reg:0x%x, 0x%x.\n", REG_ROTATION_SRC_ADDR + i * 4, value);
   }
  RTT_PRINT("###############get_DMA_reg##########################\n");
  for(i = 0; i < 49; i++)
  {
    value = _pard(SPRD_DMA_BASE + i * 4);
    RTT_PRINT("DMA reg:0x%x, 0x%x.\n", SPRD_DMA_BASE + i * 4, value);
   }
    for(i = 0; i < 8; i++)
  {
    value = _pard(SPRD_DMA_BASE + 0x6A0 + i * 4);
    RTT_PRINT("DMA chn 21 reg:0x%x, 0x%x.\n", SPRD_DMA_BASE + 0x6A0 + i * 4, value);
   }
}
#endif
static void rotation_done(void)
{
    DECLARE_ROTATION_PARAM_ENTRY(s);
  
    rotation_software_reset();
    rotation_set_src_addr(s->s_addr);
    rotation_set_dst_addr(s->d_addr);
    rotation_set_img_size(&(s->img_size));
    rotation_set_pixel_mode(s->pixel_format);
    rotation_set_dir(s->rotation_dir);
    rotation_set_UV_mode(s->uv_mode);	
    rotation_enable();
    RTT_PRINT("ok to rotation_done.\n");	
#ifdef ROTATION_DEBUG	
    get_rotation_reg();
#endif
}
static int rotation_set_UV_param(void)
{
    DECLARE_ROTATION_PARAM_ENTRY(s);

    s->s_addr=s->src_addr.uv_addr;
    s->d_addr=s->dst_addr.uv_addr;  

    s->img_size.w>>=0x01;
    s->pixel_format=ROTATION_TWO_BYTES;  

    if((ROTATION_YUV422==s->data_format)
        &&((ROTATION_90==s->rotation_dir)||(ROTATION_270==s->rotation_dir)))
    {
        s->uv_mode=ROTATION_UV422;
        s->img_size.h>>=0x01;
    }
    else if(ROTATION_YUV420==s->data_format)
    {
        s->img_size.h>>=0x01;
    }

    return 0;
}
static void rotation_dma_irq(int dma_ch, void *dev_id)
{ 
        condition = 1;
	wake_up_interruptible(&wait_queue);
	RTT_PRINT("rotation_dma_irq X .\n");
}

int rotation_dma_start(ROTATION_PARAM_T* param_ptr)
{
	int ret = 0;
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;	
	
	RTT_PRINT("rotation_dma_start E .\n");
	ret  = sprd_request_dma(DMA_ROT, rotation_dma_irq, &dma_desc);
        if(ret){
		RTT_PRINT("fail to sprd_request_dma.\n");
        	return -EFAULT;
        }
		
	condition = 0;
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;
#ifdef CONFIG_ARCH_SC8810
	ctrl.dma_desc_phy = (dma_addr_t)0x20800420;
#else
	ctrl.dma_desc_phy = (dma_addr_t)0x20800210;
#endif
	
	sprd_dma_setup_cfg(&ctrl,
                DMA_ROT,
                DMA_LINKLIST,
                LLIST_DONE_EN, //TRANS_DONE_EN,
                DMA_INCREASE, DMA_INCREASE,
                0, 0,
                0,
                0, 0,
                param_ptr->src_addr.y_addr, 0, 0);		
	 sprd_dma_setup(&ctrl);
	 sprd_dma_start(DMA_ROT); 
         RTT_PRINT("rotation_dma_start X .\n");
	 return 0;
}

int rotation_dma_wait_stop(void)
{
	int ret = 0;
	RTT_PRINT("rotation_dma_wait_stop E .\n");
	 if(wait_event_interruptible(wait_queue, condition)){
	 	ret =  -EFAULT;
	 }	 
	 sprd_dma_stop(DMA_ROT);
	 sprd_free_dma(DMA_ROT);	
         RTT_PRINT("ok to rotation_dma_wait_stop.\n");
	 return ret;
}

int rotation_start(ROTATION_PARAM_T* param_ptr)
{   
    DECLARE_ROTATION_PARAM_ENTRY(s);
    
    rotation_check_param(param_ptr);
    rotation_set_y_param(param_ptr);
    rotation_cfg(); 
    rotation_dma_start(param_ptr);       
    rotation_done();  
    
     if(ROTATION_FALSE==s->is_end)
     {
    	  RTT_PRINT("ok to UV plane.\n");
	  rotation_dma_wait_stop();
	  rotation_dma_start(param_ptr);
          rotation_set_UV_param();
          rotation_done(); 
     }

    rotation_dma_wait_stop();
	
    return 0;    
}
int rotation_IOinit(void)
{
	down(&g_sem_rot);
	return 0;
}
int rotation_IOdeinit(void)
{
	rotation_disable();
	up(&g_sem_rot);
	return 0;
}

int rotation_open (struct inode *node, struct file *file)
{
	ROTATION_PARAM_T *params;
	
	rotation_IOinit();

	params = (ROTATION_PARAM_T *)kmalloc(
			sizeof(ROTATION_PARAM_T), GFP_KERNEL);

	if(params == NULL) {
		printk(KERN_ERR "Instance memory allocation was failed\n");
		return -1;
	}

	memset(params, 0, sizeof(ROTATION_PARAM_T));

	file->private_data = (ROTATION_PARAM_T *)params;

	RTT_PRINT("[pid:%d] sc8800g_rotation_open() called.\n", current->pid);
	
	return 0;
}
int rotation_release (struct inode *node, struct file *file)
{
	ROTATION_PARAM_T *params;

	params = (ROTATION_PARAM_T *)file->private_data;
	if (params == NULL) {
		printk(KERN_ERR "Can't release rotation_release !!\n");
		return -1;
	}

	kfree(params);

	RTT_PRINT("[pid:%d] rotation_release()\n", current->pid);

	rotation_IOdeinit();
	
	return 0;
}

static int rotation_ioctl(struct inode *node, struct file *file, unsigned int cmd, unsigned long arg)
{
	ROTATION_PARAM_T *params;

	params = (ROTATION_PARAM_T *)file->private_data;
	if (copy_from_user(params, 
		(ROTATION_PARAM_T *)arg, sizeof(ROTATION_PARAM_T)))
		return -EFAULT;
	
	mutex_lock(lock);

	if (cmd != SC8800G_ROTATION_DONE) {
		mutex_unlock(lock);
		printk(KERN_ERR "sc8800g_rotation: unknown ioctl cmd\n");
		return -EFAULT;
	}
	
	if (rotation_start(params)) {
		mutex_unlock(lock);
		return -EFAULT;
	}
	
	mutex_unlock(lock);

	return 0;
}

static struct file_operations rotation_fops = {
	.owner		= THIS_MODULE,
	.open           = rotation_open,
	.ioctl          = rotation_ioctl,
	.release	= rotation_release,
};


static struct miscdevice rotation_dev = {
	.minor   = ROTATION_MINOR,
	.name   = "sc8800g_rotation",
	.fops   = &rotation_fops,
};

int rotation_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_ALERT"rotation_probe called\n");

	ret = misc_register(&rotation_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				ROTATION_MINOR, ret);
		return ret;
	}

	lock = (struct mutex *)kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL)
		return -1;

	mutex_init(lock);
	
	init_waitqueue_head(&wait_queue);

	printk(KERN_ALERT" rotation_probe Success\n");

	return 0;
}


static int rotation_remove(struct platform_device *dev)
{
	printk(KERN_INFO "rotation_remove called !\n");

	misc_deregister(&rotation_dev);

	printk(KERN_INFO "rotation_remove Success !\n");
	return 0;
}


static struct platform_driver rotation_driver = {
	.probe    = rotation_probe,
	.remove   = rotation_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sc8800g_rotation",
	},
};
int __init rotation_init(void)
{
	printk(KERN_INFO "rotation_init called !\n");
	if(platform_driver_register(&rotation_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	init_MUTEX(&g_sem_rot);	

	return 0;
}

void rotation_exit(void)
{
	printk(KERN_INFO "rotation_exit called !\n");
	platform_driver_unregister(&rotation_driver);
	mutex_destroy(lock);
	kfree(lock);//wxz20120118: free the mutex lock
	lock = NULL;
}

module_init(rotation_init);
module_exit(rotation_exit);

MODULE_DESCRIPTION("rotation Driver");
MODULE_LICENSE("GPL");
