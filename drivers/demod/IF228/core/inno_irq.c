
/*
 * innofidei if2xx demod irq driver
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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <asm/bitops.h>
#include "inno_core.h"
#include <linux/gpio.h>
#include "inno_irq.h"
#include "inno_comm.h"
#include "inno_demod.h"
//#define IRQ_DEBUG


#ifdef  IRQ_DEBUG
#include <linux/timer.h>
#endif


#define UAM_BIT_CNT     0x1
#define UAM_BIT_SHIFT   0x8
#define LGX_BIT_CNT     UAM_BIT_CNT + UAM_BIT_SHIFT

extern struct bus_type inno_lgx_bus_type;

#define MAX_REQ_NUM             10
static struct inno_req          irq_req[MAX_REQ_NUM];
static struct list_head        irq_req_free_list;
static  spinlock_t                irq_req_lock;
/**
 * inno_lgx_match -find which lgx_device this interrupt belong to
 * @dev:device
 * @data:channel id 
 */
static int inno_lgx_match(struct device *dev, void *data)
{
        char name[10];
        int id = *(int *)data;
       
        if (id < UAM_BIT_SHIFT) 
                sprintf(name, LG_PREFIX"%d", id);
        else
                sprintf(name, UAM_PREFIX"%d", id - UAM_BIT_SHIFT);
        
        return (strcmp(name, dev_name(dev)) == 0);
}

/**
 * inno_irq_req_handler -called by demod req workqueue
 * @data:unused
 *
 * identify which channel this interrupt belong to and notify correct lgx_device
 */
static int inno_irq_req_handler(void *data)
{
        unsigned long   lgx_ready = 0;
        int             bit;
        struct device * dev;
        struct lgx_device *lgx_dev;
        struct lgx_driver *lgx_drv;
        struct inno_req *req = (struct inno_req *)data;

        pr_debug("%s\n", __func__);

        inno_get_intr_ch(&lgx_ready);
        // lgx_ready |= 0x1;
        pr_debug("%s:lgx_ready = 0x%lx\n", __func__, lgx_ready);
#ifdef IRQ_DEBUG
        lgx_ready |= 0x1;
#endif
        if (!lgx_ready)
                goto out;
      
        bit = find_first_bit(&lgx_ready, LGX_BIT_CNT);
        while (bit < LGX_BIT_CNT) {
                dev = bus_find_device(&inno_lgx_bus_type, NULL, &bit, inno_lgx_match);
                if (dev && dev->driver) {
                        lgx_dev = container_of(dev, struct lgx_device, dev);
                        lgx_drv = container_of(dev->driver, struct lgx_driver, driver);
                        if (lgx_drv->data_notify)
                                        lgx_drv->data_notify(lgx_dev); 
                }
                
                bit = find_next_bit(&lgx_ready, LGX_BIT_CNT, bit + 1);
        }

out:
	pr_debug("%s:req = %p\n", __func__, req);
	spin_lock_irq(&irq_req_lock);
        list_add_tail(&req->queue, &irq_req_free_list);
	spin_unlock_irq(&irq_req_lock);
        return 0; 
}

/**
 * queue req to demod req queue only 
 */
irqreturn_t inno_demod_irq_handler(int irqnr, void *devid)
{
        struct inno_req *req;
	int ret= 0;
		
        pr_debug("%s\n", __func__);
	spin_lock_irq(&irq_req_lock);
        if (!list_empty(&irq_req_free_list)) {
                req = container_of(irq_req_free_list.next, struct inno_req, queue);
                pr_debug("%s:req = %p\n", __func__, req);
                list_del_init(&req->queue);
                req->context = req;
                ret = inno_req_async(req);
		  if (ret < 0)
		  	 list_add_tail(&req->queue, &irq_req_free_list);
        } else {
                pr_err("%s:no free req node\n", __func__);
        } 
	spin_unlock_irq(&irq_req_lock);
        return IRQ_HANDLED;
}

#ifdef  IRQ_DEBUG
struct timer_list irq_timer;
int     timer_stop = 0;
static void irq_timer_handler(unsigned long data)
{
        pr_debug("\n\n%s++++++++++++++++++++++++++++++++++++++++++++++++\n", __func__);
        inno_req_async(&irq_req);
        if (!timer_stop)
                mod_timer(&irq_timer, jiffies + 3*HZ);
}
#endif

int inno_irq_init(struct inno_demod *inno_demod)
{
        int ret = 0;
        int i;

        pr_debug("%s\n", __func__);

	 spin_lock_init(&irq_req_lock);
        INIT_LIST_HEAD(&irq_req_free_list);
        for (i = 0; i < MAX_REQ_NUM; i++)
        {
                irq_req[i].handler = inno_irq_req_handler;
                irq_req[i].context = NULL;
                irq_req[i].complete = NULL; 
                INIT_LIST_HEAD(&irq_req[i].queue); 
                list_add_tail(&irq_req[i].queue, &irq_req_free_list);
        }        

#ifdef  IRQ_DEBUG
        init_timer(&irq_timer);
        irq_timer.function = irq_timer_handler;
        //mod_timer(&irq_timer, jiffies + 3*HZ);
#endif 
        return ret;
}

void inno_irq_exit(void)
{
        pr_debug("%s\n", __func__);
#ifdef  IRQ_DEBUG
        timer_stop = 1;
        del_timer_sync(&irq_timer);
#endif
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean.zhao <zhaoguangyu@innofidei.com>");
MODULE_DESCRIPTION("innofidei cmmb irq data flow");

