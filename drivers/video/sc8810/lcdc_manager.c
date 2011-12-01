/* drivers/video/sc8800g/sc8800g_lcdc_manager.c
 *
 * SC8800g LCDC Manager
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Geng Ren <geng.ren@spreadtrum.com>
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

/*
 * LCDC manager is the module to manage:
 * 	- per-layer status: disabled/enabled
 * 	- per-layer mode:   display/capture
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mm.h>

#include "lcdc_manager.h"

//#define  LM_DEBUG
#ifdef LM_DEBUG
#define LM_PRINT printk
#else
#define LM_PRINT(...)
#endif

/* TEMP
static struct lcdc_manager lm;
*/
struct lcdc_manager lm;

/* should be called with lm.lock held */
static int enter_mode(int mode)
{
	if (lm.ref_cnt == 0 && lm.mode != mode) {
		/* need to do something to switch to the new mode */
		int i;
		for (i=0 ; i< lm.layer_num; i++) {
			if (!(lm.linfo[i].status & L_REGISTERED)){
LM_PRINT("@fool2, entermode layer[%d] not registered\n", i);
				continue;
			}
			if (lm.linfo[i].mode == mode &&
				(lm.linfo[i].status & L_ENABLED)) {
LM_PRINT("@fool2, entermode enable layer[%d]\n", i);
				if(lm.linfo[i].enable)
					lm.linfo[i].enable(i);
			} else {
LM_PRINT("@fool2, entermode disable layer[%d]\n", i);
				if(lm.linfo[i].disable)
					lm.linfo[i].disable(i);
			}
		}
		lm.mode = mode;
	}

	return 0;
}

int lm_acquire(int id)
{
	unsigned long flags;
 
LM_PRINT("@fool2, lm_acquire, id = %d\n", id);
	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (!(lm.linfo[id].status & L_REGISTERED)) {
		spin_unlock_irqrestore(&lm.lock, flags);
		printk(KERN_ERR "LM: layer not registered!\n");
		return -1;
	}

	if (unlikely(lm.ref_cnt != 0 && lm.mode != lm.linfo[id].mode)) {
		/* lcdc is busy working in another mode */
__again:
LM_PRINT("@fool2(line %d) lm.ref_cnt = %d, lm.mode = %d\n",
__LINE__, lm.ref_cnt, lm.mode);
		lm.linfo[id].status |= L_WAIT;
		spin_unlock_irqrestore(&lm.lock, flags);
		down(&lm.linfo[id].wait);
		spin_lock_irqsave(&lm.lock, flags);

		/* double check */
		if (unlikely(lm.ref_cnt != 0 && lm.mode != lm.linfo[id].mode))
			goto __again;
	}

	enter_mode(lm.linfo[id].mode);

	lm.ref_cnt++;

LM_PRINT("@fool2(line %d) lm.ref_cnt = %d, lm.mode = %d\n",
__LINE__, lm.ref_cnt, lm.mode);
	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

int lm_release(int id)
{
	unsigned long flags;

LM_PRINT("@fool2, lm_release, id = %d\n", id);
	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (lm.linfo[id].mode == lm.mode) {
		lm.ref_cnt--;
	} else {
		printk(KERN_ERR "LM: FATAL ERROR!\n");
		spin_unlock_irqrestore(&lm.lock, flags);
		return -1;
	}

LM_PRINT("@fool2(line %d) lm.ref_cnt = %d, lm.mode = %d\n",
__LINE__, lm.ref_cnt, lm.mode);

	if (lm.ref_cnt == 0) {
		int i;
		for (i=0 ; i< lm.layer_num; i++) {
			/* assume there's one user per layer at most */
			if ((lm.linfo[i].status & L_WAIT)) {
				up(&lm.linfo[i].wait);
				lm.linfo[i].status &= ~L_WAIT;
LM_PRINT("@fool2, wakeup layer[%d]\n", i);
			}
		}
	}

	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

int lm_register_layer(int id, int mode, 
		int (*enable)(int id), int (*disable)(int id))
{
	unsigned long flags;

	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (lm.linfo[id].status & L_REGISTERED) {
		spin_unlock_irqrestore(&lm.lock, flags);
		printk(KERN_ERR "LM: layer has already been registered!\n");
		return -1;
	}

	lm.linfo[id].status |= L_REGISTERED;
	lm.linfo[id].mode = mode;
	lm.linfo[id].enable = enable;
	lm.linfo[id].disable = disable;
	sema_init(&lm.linfo[id].wait, 0);

	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

int lm_unregister_layer(int id)
{
	unsigned long flags;

	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (!(lm.linfo[id].status & L_REGISTERED)) {
		spin_unlock_irqrestore(&lm.lock, flags);
		printk(KERN_ERR "LM: layer not registered!\n");
		return -1;
	}

	lm.linfo[id].status = 0;
	
	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

struct lcdc_manager * lm_init(int num_of_layers)
{
	lm.mode = LMODE_INVALID;
	lm.ref_cnt = 0;
	lm.layer_num = num_of_layers;
	lm.linfo = (struct layer_info*)kzalloc(sizeof(struct layer_info) * num_of_layers, GFP_KERNEL);
	spin_lock_init(&lm.lock);
	LM_PRINT("lcdc manager initialized!\n");

	return &lm;
}

int lm_exit(void)
{
	kfree(lm.linfo);
	return 0;
}

int lm_enable_layer(int id)
{
	unsigned long flags;

	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (!(lm.linfo[id].status & L_REGISTERED)) {
		spin_unlock_irqrestore(&lm.lock, flags);
		printk(KERN_ERR "LM: layer not registered!\n");
		return -1;
	}

	lm.linfo[id].status |= L_ENABLED;

	if(lm.linfo[id].enable)
		lm.linfo[id].enable(id);

	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

int lm_disable_layer(int id)
{
	unsigned long flags;

	if (id >= lm.layer_num || id < 0) {
		printk(KERN_ERR "LM: invalid layer id!\n");
		return -1;
	}

	spin_lock_irqsave(&lm.lock, flags);

	if (!(lm.linfo[id].status & L_REGISTERED)) {
		spin_unlock_irqrestore(&lm.lock, flags);
		printk(KERN_ERR "LM: layer not registered!\n");
		return -1;
	}

	lm.linfo[id].status &= ~L_ENABLED;

	if(lm.linfo[id].disable)
		lm.linfo[id].disable(id);

	spin_unlock_irqrestore(&lm.lock, flags);

	return 0;
}

/* TEMP helper functions */
#include "mach/hardware.h"

unsigned int lreg_addr[LID_MAX] =
{
	(SPRD_LCDC_BASE+0x20),
	(SPRD_LCDC_BASE+0x50), //(SPRD_LCDC_BASE+0x40),
	(SPRD_LCDC_BASE+0x80), //(SPRD_LCDC_BASE+0x70),
	(SPRD_LCDC_BASE+0xb0), //(SPRD_LCDC_BASE+0x90),
	(SPRD_LCDC_BASE+0xe0), //(SPRD_LCDC_BASE+0xb0),
	(SPRD_LCDC_BASE+0x110), //(SPRD_LCDC_BASE+0xd0),
};

int enable_layer(int id)
{
	LM_PRINT("@fool2, enable_layer[%d] %x\n", 
		id, *(unsigned int*)lreg_addr[id]);
	__raw_bits_or(1<<0, lreg_addr[id]);
	LM_PRINT("@fool2, enable_layer[%d] %x\n", 
		id, *(unsigned int*)lreg_addr[id]);
	return 0;
}

int disable_layer(int id)
{
	LM_PRINT("@fool2, disable_layer[%d] %x\n", 
		id, *(unsigned int*)lreg_addr[id]);
	__raw_bits_and(~(1<<0), lreg_addr[id]);
	LM_PRINT("@fool2, disable_layer[%d] %x\n", 
		id, *(unsigned int*)lreg_addr[id]);
	return 0;
	
}
