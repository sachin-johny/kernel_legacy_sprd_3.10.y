/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <mach/irqs.h>

#include "sprdfb.h"

struct sprd_lcd_controller {
	/* only one device can work one time */
	struct sprdfb_device  *dev;

	struct clk		*clk_lcdc;

	wait_queue_head_t       vsync_queue;
	uint32_t	        vsync_done;

	#if 0
	spinlock_t 		my_lock;
	#endif
};

static struct sprd_lcd_controller lcdc;

/* lcdc soft reset */
static void sprd_lcdc_reset(void)
{
	#define REG_AHB_SOFT_RST (AHB_SOFT_RST + SPRD_AHB_BASE)
	__raw_writel(__raw_readl(REG_AHB_SOFT_RST) | (1<<3), REG_AHB_SOFT_RST);
	udelay(10);
	__raw_writel(__raw_readl(REG_AHB_SOFT_RST) | (~(1<<3)), REG_AHB_SOFT_RST);

}

static irqreturn_t lcdc_isr(int irq, void *data)
{
	uint32_t val;
        struct sprd_lcd_controller *lcdc = (struct sprd_lcd_controller *)data;
	struct sprdfb_device *dev = lcdc->dev;

	val = lcdc_read(LCDC_IRQ_STATUS);

	if (val & 1) { /* lcdc done isr */
		lcdc_write(1, LCDC_IRQ_CLR);

		if (dev->ctrl->set_timing) {
			dev->ctrl->set_timing(dev,LCD_REGISTER_TIMING);
		}

		lcdc->vsync_done = 1;
		if (dev->vsync_waiter) {
			wake_up_interruptible(&(lcdc->vsync_queue));
		}
		pr_debug(KERN_INFO "lcdc_done_isr !\n");

	}

	return IRQ_HANDLED;
}

static inline int32_t set_lcdsize(struct fb_var_screeninfo *var)
{
	uint32_t reg_val;

	reg_val = (var->xres & 0xfff) | ((var->yres & 0xfff ) << 16);
	lcdc_write(reg_val, LCDC_DISP_SIZE);

	return 0;
}

static inline int32_t set_lcmrect(struct fb_var_screeninfo *var)
{
	uint32_t reg_val;

	lcdc_write(0, LCDC_LCM_START);

	reg_val = (var->xres & 0xfff) | ((var->yres & 0xfff ) << 16);
	lcdc_write(reg_val, LCDC_LCM_SIZE);

	return 0;
}

static void lcdc_layer_init(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	/******************* OSD1 layer setting **********************/

	lcdc_set_bits((1<<0),LCDC_IMG_CTRL);
	lcdc_set_bits((1<<0),LCDC_OSD2_CTRL);
	lcdc_set_bits((1<<0),LCDC_OSD3_CTRL);
	lcdc_set_bits((1<<0),LCDC_OSD4_CTRL);
	lcdc_set_bits((1<<0),LCDC_OSD5_CTRL);
	/*enable OSD1 layer*/
	reg_val |= (1 << 0);

	/* color key */

	/* alpha mode select */
	reg_val |= (1 << 2);

	/* data format */
	if (var->bits_per_pixel == 32) {
		/* ABGR */
		reg_val |= (3 << 3);
		/* rb switch */
	 	reg_val |= (1 << 9);

	} else {
		/* RGB565 */
		reg_val |= (5 << 3);
		/* B2B3B0B1 */
		reg_val |= (2 << 7);
	}

	lcdc_write(reg_val, LCDC_OSD1_CTRL);

	/* OSD1 layer alpha value */
	lcdc_write(0xff, LCDC_OSD1_ALPHA);

	/* alpha base addr */

	/* OSD1 layer size */
	reg_val = ( var->xres & 0xfff) | (( var->yres & 0xfff ) << 16);

	lcdc_write(reg_val, LCDC_OSD1_SIZE_XY);

	/* OSD1 layer start position */
	lcdc_write(0, LCDC_OSD1_DISP_XY);

	/* OSD1 layer pitch */
	reg_val = ( var->xres & 0xfff) ;
	lcdc_write(reg_val, LCDC_OSD1_PITCH);

	/* OSD1 color_key value */

	/* OSD1 grey RGB */

	/* LCDC workplane size */
	set_lcdsize(var);

	/*LCDC LCM rect size */
	set_lcmrect(var);
}

static void lcdc_hw_init(void)
{
	uint32_t reg_val;

	/* LCDC module enable */
	reg_val = (1<<0);

	/* FMARK mode */
	#if 0
	reg_val |= (1<<1);
	#endif

	/* FMARK pol */
	reg_val |= (1<<2);

	lcdc_write(reg_val, LCDC_CTRL);

	/* set background */
	lcdc_write(0xFFFFFF, LCDC_BG_COLOR);

	/* clear lcdc IRQ */
	lcdc_set_bits((1<<0), LCDC_IRQ_CLR);
}

static int32_t sprd_lcdc_early_init(void)
{
	int ret = 0;
	lcdc.clk_lcdc = clk_get(NULL, "clk_lcdc");
	clk_enable(lcdc.clk_lcdc);

	sprd_lcdc_reset();
	lcdc_hw_init();

	lcdc.vsync_done = 1;
	init_waitqueue_head(&(lcdc.vsync_queue));
	ret = request_irq(IRQ_LCDC_INT, lcdc_isr, IRQF_DISABLED, "LCDC", &lcdc);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		return -1;
	}
	return 0;
}

static int32_t sprd_lcdc_init(struct sprdfb_device *dev)
{
	lcdc_layer_init(&(dev->fb->var));

	/* enable lcdc IRQ */
	lcdc_write((1<<0), LCDC_IRQ_EN);
	dev->enable = 1;
	return 0;
}

static int32_t sprd_lcdc_cleanup(struct sprdfb_device *dev)
{
	return 0;
}

static int32_t sprd_lcdc_enable_plane(struct sprdfb_device *dev, int enable)
{
	if (lcdc.dev == NULL ) {
		lcdc.dev = dev;
	}
	if (enable) {
		if (dev->enable !=0) {
			return 0;
		} else {
			dev->init_panel(dev);
			dev->enable = 1;
		}
	} else {
		if (dev->enable !=0) {
			dev->enable = 0;
		}
	}
	return 0;
}

static int32_t sprd_lcdc_refresh (struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;

	uint32_t base = fb->fix.smem_start + fb->fix.line_length * fb->var.yoffset;

	lcdc.dev = dev;

	pr_debug("fb->var.yoffset: 0x%x\n", fb->var.yoffset);

#ifdef LCD_UPDATE_PARTLY
	if (fb->var.reserved[0] == 0x6f766572) {
		uint32_t x,y, width, height;

		x = fb->var.reserved[1] & 0xffff;
		y = fb->var.reserved[1] >> 16;
   		width  = fb->var.reserved[2] &  0xffff;
		height = fb->var.reserved[2] >> 16;

		base += ((x + y * fb->var.xres) * fb->var.bits_per_pixel / 8);
		lcdc_write(base, LCDC_OSD1_BASE_ADDR);
		lcdc_write(fb->var.reserved[2], LCDC_OSD1_SIZE_XY);

		lcdc_write(fb->var.reserved[2], LCDC_LCM_SIZE);
		lcdc_write(fb->var.reserved[2], LCDC_DISP_SIZE);

		dev->panel->ops->panel_invalidate_rect(dev->panel,
					left, top, left+width-1, top+height-1);
	} else
#endif
	{
		uint32_t size = (fb->var.xres & 0xffff) | ((fb->var.yres) << 16);

		lcdc_write(base, LCDC_OSD1_BASE_ADDR);
		lcdc_write(0, LCDC_OSD1_DISP_XY);
		lcdc_write(size,LCDC_OSD1_SIZE_XY);
		lcdc_write(fb->var.xres, LCDC_OSD1_PITCH);

		lcdc_write(size, LCDC_DISP_SIZE);

		dev->panel->ops->panel_invalidate(dev->panel);
	}

	if (dev->ctrl->set_timing) {
		dev->ctrl->set_timing(dev,LCD_GRAM_TIMING);
	}

	/* start refresh */
	lcdc_set_bits((1 << 3), LCDC_CTRL);

	pr_debug("LCDC_CTRL: 0x%x\n", lcdc_read(LCDC_CTRL));
	pr_debug("LCDC_DISP_SIZE: 0x%x\n", lcdc_read(LCDC_DISP_SIZE));
	pr_debug("LCDC_LCM_START: 0x%x\n", lcdc_read(LCDC_LCM_START));
	pr_debug("LCDC_LCM_SIZE: 0x%x\n", lcdc_read(LCDC_LCM_SIZE));
	pr_debug("LCDC_BG_COLOR: 0x%x\n", lcdc_read(LCDC_BG_COLOR));
	pr_debug("LCDC_FIFO_STATUS: 0x%x\n", lcdc_read(LCDC_FIFO_STATUS));

	pr_debug("LCM_CTRL: 0x%x\n", lcdc_read(LCM_CTRL));
	pr_debug("LCM_TIMING0: 0x%x\n", lcdc_read(LCM_TIMING0));
	pr_debug("LCM_RDDATA: 0x%x\n", lcdc_read(LCM_RDDATA));

	pr_debug("LCDC_IRQ_EN: 0x%x\n", lcdc_read(LCDC_IRQ_EN));

	pr_debug("LCDC_OSD1_CTRL: 0x%x\n", lcdc_read(LCDC_OSD1_CTRL));
	pr_debug("LCDC_OSD1_BASE_ADDR: 0x%x\n", lcdc_read(LCDC_OSD1_BASE_ADDR));
	pr_debug("LCDC_OSD1_ALPHA_BASE_ADDR: 0x%x\n", lcdc_read(LCDC_OSD1_ALPHA_BASE_ADDR));
	pr_debug("LCDC_OSD1_SIZE_XY: 0x%x\n", lcdc_read(LCDC_OSD1_SIZE_XY));
	pr_debug("LCDC_OSD1_PITCH: 0x%x\n", lcdc_read(LCDC_OSD1_PITCH));
	pr_debug("LCDC_OSD1_DISP_XY: 0x%x\n", lcdc_read(LCDC_OSD1_DISP_XY));
	pr_debug("LCDC_OSD1_ALPHA	: 0x%x\n", lcdc_read(LCDC_OSD1_ALPHA));

	return 0;
}

static int32_t sprd_lcdc_sync(struct sprdfb_device *dev)
{
	int ret;

	if (dev->enable == 0) {
		return -1;
	}
	ret = wait_event_interruptible_timeout(lcdc.vsync_queue,
			          lcdc.vsync_done, msecs_to_jiffies(100));
	lcdc.vsync_done = 0;
	if (!ret) { /* time out */
		printk(KERN_ERR "lcdc: sprd_lcdc_sync time out!!!!!\n");
		return -1;
	}
	return 0;
}

static int32_t sprd_lcdc_suspend(struct sprdfb_device *dev)
{
	return 0;
	if (dev->enable != 0) {
		dev->enable = 0;

		/* must wait ,sprd_lcdc_sync() */
		dev->ctrl->sync(dev);

		/* let lcdc sleep in */
		if (dev->panel->ops->panel_enter_sleep != NULL) {
			dev->panel->ops->panel_enter_sleep(dev->panel,1);
		}
		clk_disable(lcdc.clk_lcdc);
	}
	return 0;
}

static int32_t sprd_lcdc_resume(struct sprdfb_device *dev)
{
	return 0;

	if (dev->enable == 0) {
		clk_enable(lcdc.clk_lcdc);

		if (lcdc_read(LCDC_CTRL) == 0) { /* resume from deep sleep */
			sprd_lcdc_reset();
			lcdc_hw_init();
			lcdc_layer_init(&(dev->fb->var));

			dev->panel->ops->panel_reset(dev->panel);
			dev->init_panel(dev);

		} else {
			/* let lcd sleep out */
			dev->panel->ops->panel_enter_sleep(dev->panel,0);
		}

		dev->enable = 1;
	}
	return 0;
}

struct panel_ctrl sprd_lcdc_ctrl = {
	.name			= "lcdc",
	.early_init		= sprd_lcdc_early_init,
	.init		 	= sprd_lcdc_init,
	.cleanup		= sprd_lcdc_cleanup,
	.enable_plane           = sprd_lcdc_enable_plane,
	.refresh	        = sprd_lcdc_refresh,
	.sync                   = sprd_lcdc_sync,
	.suspend		= sprd_lcdc_suspend,
	.resume			= sprd_lcdc_resume,
};



