
/*
 * innofidei if2xx demod logic channel driver
 * 
 * Copyright (C) 2010 Innofidei Corporation
 * Author:      sean <zhaoguangyu@innofidei.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * 
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <linux/poll.h>
#include <linux/idr.h>
#include "inno_lgx.h"
#include "inno_uam.h"
#include "inno_demod.h"
#include "inno_comm.h"

#include <linux/dma-mapping.h>


static int inno_lgx_major = -1;

static DEFINE_IDR(lgx_idr);
static DEFINE_MUTEX(minor_lock);

/**
 * lgx_context - 
 * @id :logic channel id
 * @wait:used by inno_lgx_read if data comming
 * @valid:valid data len
 * @ready:1 if data valid; 0 no data
 * @buf_page:buf use to cache data
 * @buf: buf pointer use to kfree
 */
struct lgx_context { 
        struct module           *owner;
        struct lgx_device       *lgx_dev; 
        int                     minor;
        int                     id;
        wait_queue_head_t       wait;
        atomic_t                valid;
        atomic_t                ready;
        unsigned char           *buf_page;
        unsigned long           vm_start;
        unsigned long           vm_end;
        void                    *buf;
        unsigned char           active;
};


static int innolgx_release(struct inode *inode, struct file *filep)
{
        struct lgx_context *context = filep->private_data;
        pr_debug("%s",__FUNCTION__);
        context->active = 0;
        inno_demod_dec_ref();
        return 0;
}

static unsigned int innolgx_poll(struct file *filep, poll_table *wait)
{
        struct lgx_context *context = filep->private_data;

        poll_wait(filep, &context->wait, wait);
        if (atomic_read(&context->ready)) {
                atomic_set(&context->ready, 0);
                return POLLIN | POLLRDNORM;
        }
        return 0;
}

static int innolgx_mmap(struct file *filep, struct vm_area_struct *vma)
{
        struct lgx_context *context = filep->private_data;
        int ret;  
        unsigned long off;
        long len;
        len = vma->vm_end - vma->vm_start;  
        off = vma->vm_pgoff << PAGE_SHIFT;
        
        pr_debug("%s\n", __func__);
        /* check length - do not allow larger mappings than the number of 
              pages allocated */  
        if (off + len > MAX_LGX_BUF_LEN)  
                return -EINVAL;  
        
        vma->vm_flags |= VM_IO | VM_RESERVED;

        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        /* map the whole physically contiguous area in one piece */  
        if ((ret = remap_pfn_range(vma,  
                                   vma->vm_start,  
                                   virt_to_phys((void *)context->buf_page) >> PAGE_SHIFT,  
                                   len,  
                                   vma->vm_page_prot)) < 0 ) {  
                return ret;  
        } 
        context->vm_start = vma->vm_start;
        context->vm_end = vma->vm_end;
 
        return 0;
}

static ssize_t innolgx_read(struct file *filep, char __user *buf,
                        size_t count, loff_t *ppos)
{
        struct lgx_context *context = filep->private_data;
        DECLARE_WAITQUEUE(wait, current);
        ssize_t retval;
        unsigned int valid;

        pr_debug("%s\n", __func__);
        if (count > MAX_LGX_BUF_LEN)
                return -EINVAL;

        add_wait_queue(&context->wait, &wait);

        do {
                set_current_state(TASK_INTERRUPTIBLE);
                valid = atomic_read(&context->valid);
                if (valid) {
                        atomic_set(&context->valid, 0);
                        atomic_set(&context->ready, 0);
                        
                        if (copy_to_user(buf, context->buf_page, valid))
                                retval = -EFAULT;
                        else 
                                retval = valid;
                        break;
                }

                if (filep->f_flags & O_NONBLOCK) {
                        retval = -EAGAIN;
                        break;
                }

                if (signal_pending(current)) {
                        retval = -ERESTARTSYS;
                        break;
                }
                schedule();
        } while (1);

        __set_current_state(TASK_RUNNING);
        remove_wait_queue(&context->wait, &wait);

        return retval;
}

static int innolgx_open(struct inode* inode, struct file* filep)
{
        struct  lgx_context *context;
        int ret = 0;

        pr_debug("%s\n", __func__);
        pr_debug("%s:minor=%d\n", __func__, iminor(inode));
        mutex_lock(&minor_lock);
        context = idr_find(&lgx_idr, iminor(inode));
        mutex_unlock(&minor_lock);
        if (!context) {
                ret = -ENODEV;
                goto out;
        }

        filep->private_data = context;
        
        ret = inno_demod_inc_ref();
        if (ret < 0)
                goto out;

        context->active = 1;
out:
        return ret;
}

static struct file_operations innolgx_fops = {
        .owner          = THIS_MODULE,
        .open           = innolgx_open,
        .read           = innolgx_read,
        .mmap           = innolgx_mmap,
        .poll           = innolgx_poll,
        .release        = innolgx_release,
};

/**
 * lgx_get_minor - bind lgx_context to char device minor number, 
 *                 so inno_lgx_open can get lgx_context pointer from minor number
 */
static int lgx_get_minor(struct lgx_context *context)
{
        int retval = -ENOMEM;
        int id;

        mutex_lock(&minor_lock);

        if (idr_pre_get(&lgx_idr, GFP_KERNEL) == 0)
                goto exit;

        retval = idr_get_new_above(&lgx_idr, context, context->id, &id);
        if (retval == 0 && context->id != id) {
                retval = -EBUSY;
                goto exit;
        }
        if (retval < 0) {
                if (retval == -EAGAIN)
                        retval = -ENOMEM;
                goto exit;
        }
        context->minor = id & MAX_ID_MASK;
exit:
        mutex_unlock(&minor_lock);
        return retval;
}


static int inno_lgx_probe(struct lgx_device *lgx)
{
        int ret = 0;
        int i;
        struct lgx_context *context  =NULL;

        pr_debug("%s\n", __func__);
        pr_debug("%s:lg id = %d\n", __func__, lgx->id);
                        
        context = kzalloc(sizeof(*context), GFP_KERNEL);
        if (context == NULL) {
                ret = -ENOMEM;
                goto err;
        }

        context->buf = kzalloc(MAX_LGX_BUF_LEN + 2*PAGE_SIZE, GFP_DMA | GFP_KERNEL);
        if (context->buf == NULL) {
                ret = -ENOMEM;
                goto err;
        }

        context->owner      = THIS_MODULE; 
        context->id         = lgx->id;
        context->active     = 0;
        ret = lgx_get_minor(context);
        if (ret) 
                goto err;
        
                

        if (context->id != context->minor) {
                pr_err("%s:ch id != minor, id = %d, minor = %d\n",
                        __FUNCTION__,
                        context->id,
                        context->minor);
                goto err; 
        }
        
        atomic_set(&context->valid, 0);
        atomic_set(&context->ready, 0);

        context->buf_page = (unsigned char *)((unsigned long)context->buf & PAGE_MASK);
        
        /* mark the pages reserved */  
        for (i = 0; i < MAX_LGX_BUF_LEN; i+= PAGE_SIZE) {  
                SetPageReserved(virt_to_page(((unsigned long)context->buf_page) + i));  
        }
        init_waitqueue_head(&context->wait);
        
        lgx->buf        = context->buf_page;
        lgx->buf_len    = MAX_LGX_BUF_LEN;
        dev_set_drvdata(&lgx->dev, context);
                
        goto exit;
err:
        if (context)
                kfree(context);
exit:
        return ret;
}

static int __devexit inno_lgx_remove(struct lgx_device *lgx)
{
        struct lgx_context *context  = dev_get_drvdata(&lgx->dev);
        int i;

        pr_debug("%s\n", __func__);
         
        mutex_lock(&minor_lock);
        idr_remove(&lgx_idr, lgx->id);
        mutex_unlock(&minor_lock);

        if (context) {
                if (context->buf) {
                        for (i = 0; i < MAX_LGX_BUF_LEN; i+= PAGE_SIZE) 
                                ClearPageReserved(virt_to_page(((unsigned long)context->buf_page) + i));  
                        kfree(context->buf);
                }
                kfree(context);
        }
                
        return 0;
}

/**
 * innolgx_data_notify - called by inno_irq_req_handler when channel data is valid
 */
static void innolgx_data_notify(struct lgx_device *lgx)
{
        struct lgx_context *context = (struct lgx_context *)dev_get_drvdata(&lgx->dev);
        int ret; 

        pr_debug("%s\n", __func__);
        if (context->active == 0)
                return;

        ret = inno_lgx_fetch_data(lgx);
        if (ret < 0)
                return ;

        /* need flush cache if PIO mode is used by spi driver */
        dma_cache_maint(lgx->buf, lgx->valid, DMA_TO_DEVICE);

        atomic_set(&context->valid, lgx->valid);
        atomic_set(&context->ready, 1);

        /* used by mmap, user can get valid len here */
        *((unsigned int *)&(((unsigned char*)lgx->buf)[MAX_LGX_BUF_LEN - 4])) = lgx->valid;

        pr_debug("%s:valid = %x\n", __func__, lgx->valid);
        wake_up_interruptible(&context->wait); 
        return ;
}

static struct lgx_driver inno_lgx_driver = {
        .probe          = inno_lgx_probe,
        .remove         = __devexit_p(inno_lgx_remove),
        .data_notify    = innolgx_data_notify,
        .driver         = {
                .name   = LG_PREFIX,
                        .owner  = THIS_MODULE,
        },
};
#define LGX_DEV_NUM     4
static struct lgx_device *lgx_dev[LGX_DEV_NUM];

static int inno_lgx_init(void)
{
        int ret;
        int i;

        pr_debug("%s\n", __func__);                
        inno_lgx_major = register_chrdev(0, "innolgx", &innolgx_fops);
        if (inno_lgx_major < 0){ 
                pr_err("failed to register character device.\n");     
                return inno_lgx_major; 
        }
        pr_debug("%s:lgx major = %d\n", __func__, inno_lgx_major);                

        inno_lgx_driver.major = inno_lgx_major;
        ret = inno_lgx_driver_register(&inno_lgx_driver);
        if (ret < 0) {
                pr_err("register inno lgx driver failed! (%d)\n", ret);
                goto err;
        }

        for (i = 0; i < LGX_DEV_NUM; i++) 
                lgx_dev[i] = inno_lgx_new_device(LG_PREFIX, inno_lgx_major, i, NULL);

        ret = inno_uam_init(); 
        if (ret < 0)
                goto err1;

        return 0;
err1:
        inno_lgx_driver_unregister(&inno_lgx_driver);
        for (i = 0; i < LGX_DEV_NUM; i++) {
                if (lgx_dev[i])
                        inno_lgx_device_unregister(lgx_dev[i]);
        }
err:

        unregister_chrdev(inno_lgx_major, "innolgx");
        return -EIO;
}

static void inno_lgx_exit(void)
{
        int i;

        pr_debug("%s\n", __func__);                
        
        inno_uam_exit();
           
        inno_lgx_driver_unregister(&inno_lgx_driver);

        unregister_chrdev(inno_lgx_major, "innolgx");

        for (i = 0; i < LGX_DEV_NUM; i++) {
                if (lgx_dev[i])
                        inno_lgx_device_unregister(lgx_dev[i]);
        }
}

module_init(inno_lgx_init);
module_exit(inno_lgx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean.zhao <zhaoguangyu@innofidei.com>");
MODULE_DESCRIPTION("innofidei cmmb lgx");

