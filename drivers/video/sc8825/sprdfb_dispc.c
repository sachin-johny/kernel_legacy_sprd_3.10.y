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

#include "sprdfb_dispc_reg.h"
#include "sprdfb.h"

#define DISPC_SOFT_RST (20)
#define DISPC_CLOCK_PARENT ("clk_256m")
#define DISPC_CLOCK (256*1000000)
#define DISPC_DBI_CLOCK_PARENT ("clk_256m")
#define DISPC_DBI_CLOCK (256*1000000)
#define DISPC_DPI_CLOCK_PARENT ("clk_384m")
#define DISPC_DPI_CLOCK (384*1000000)

#define DISPMTX_CLK_EN (11)
#define DISPC_CORE_CLK_EN (9)

struct sprdfb_dispc_context {
	struct clk		*clk_dispc;
	struct clk 		*clk_dispc_dpi;
	struct clk 		*clk_dispc_dbi;
	bool			is_inited;
	bool			is_first_frame;

	struct sprdfb_device	*dev;

	uint32_t	 	vsync_waiter;
	wait_queue_head_t		vsync_queue;
	uint32_t	        vsync_done;
};

static struct sprdfb_dispc_context dispc_ctx = {0};

extern void sprdfb_panel_suspend(struct sprdfb_device *dev);
extern void sprdfb_panel_resume(struct sprdfb_device *dev, bool from_deep_sleep);
extern void sprdfb_panel_before_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_after_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_invalidate(struct panel_spec *self);
extern void sprdfb_panel_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);


static irqreturn_t dispc_isr(int irq, void *data)
{
	struct sprdfb_dispc_context *dispc_ctx = (struct sprdfb_dispc_context *)data;
	uint32_t reg_val;
	struct sprdfb_device *dev = dispc_ctx->dev;
	bool done = false;

	reg_val = dispc_read(DISPC_INT_STATUS);

	pr_debug("dispc_isr (0x%x)\n",reg_val );

	if((SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type) && (reg_val & 0x10)){/*dispc update done isr*/
#if 0
		if(dispc_ctx->is_first_frame){
			/*dpi register update with SW and VSync*/
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);

			/* start refresh */
			dispc_set_bits((1 << 4), DISPC_CTRL);

			dispc_ctx->is_first_frame = false;
		}
#endif
		dispc_write(0x10, DISPC_INT_CLR);
		done = true;
	}else if ((SPRDFB_PANEL_IF_DPI !=  dev->panel_if_type) && (reg_val & 0x1)){ /* dispc done isr */
			dispc_write(1, DISPC_INT_CLR);
			dispc_ctx->is_first_frame = false;
			done = true;
	}

	if(done){
		dispc_ctx->vsync_done = 1;
		if (dispc_ctx->vsync_waiter) {
			wake_up_interruptible_all(&(dispc_ctx->vsync_queue));
			dispc_ctx->vsync_waiter = 0;
		}
		sprdfb_panel_after_refresh(dev);
		pr_debug(KERN_INFO "sprdfb: [%s]: Done INT, reg_val = %d !\n", __FUNCTION__, reg_val);
	}

	return IRQ_HANDLED;
}


/* dispc soft reset */
static void dispc_reset(void)
{
	#define REG_AHB_SOFT_RST (AHB_SOFT_RST + SPRD_AHB_BASE)
	__raw_writel(__raw_readl(REG_AHB_SOFT_RST) | (1<<DISPC_SOFT_RST), REG_AHB_SOFT_RST);
	udelay(10);
	__raw_writel(__raw_readl(REG_AHB_SOFT_RST) & (~(1<<DISPC_SOFT_RST)), REG_AHB_SOFT_RST);
}

static inline void dispc_set_bg_color(uint32_t bg_color)
{
	dispc_write(bg_color, DISPC_BG_COLOR);
}

static inline void dispc_set_osd_ck(uint32_t ck_color)
{
	dispc_write(ck_color, DISPC_OSD_CK);
}

static void dispc_dithering_enable(bool enable)
{
	if(enable){
		dispc_set_bits(BIT(6), DISPC_CTRL);
	}else{
		dispc_clear_bits(BIT(6), DISPC_CTRL);
	}
}

static void dispc_set_exp_mode(uint16_t exp_mode)
{
	uint32_t reg_val = dispc_read(DISPC_CTRL);

	reg_val &= ~(0x3 << 16);
	reg_val |= (exp_mode << 16);
	dispc_write(reg_val, DISPC_CTRL);
}

static void dispc_module_enable(void)
{
	/*dispc module enable */
	dispc_write((1<<0), DISPC_CTRL);

	/*disable dispc INT*/
	dispc_write(0x0, DISPC_INT_EN);

	/* clear dispc INT */
	dispc_write(0x1F, DISPC_INT_CLR);
}

static inline int32_t  dispc_set_disp_size(struct fb_var_screeninfo *var)
{
	uint32_t reg_val;

	reg_val = (var->xres & 0xfff) | ((var->yres & 0xfff ) << 16);
	dispc_write(reg_val, DISPC_SIZE_XY);

	return 0;
}

static void dispc_layer_init(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

//	dispc_clear_bits((1<<0),DISPC_IMG_CTRL);
	dispc_write(0x0, DISPC_IMG_CTRL);
	dispc_clear_bits((1<<0),DISPC_OSD_CTRL);

	/******************* OSD layer setting **********************/

	/*enable OSD layer*/
	reg_val |= (1 << 0);

	/*disable  color key */

	/* alpha mode select  - block alpha*/
	reg_val |= (1 << 2);

	/* data format */
	if (var->bits_per_pixel == 32) {
		/* ABGR */
		reg_val |= (3 << 4);
		/* rb switch */
		reg_val |= (1 << 15);
	} else {
		/* RGB565 */
		reg_val |= (5 << 4);
		/* B2B3B0B1 */
		reg_val |= (2 << 8);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);

	/* OSD layer alpha value */
	dispc_write(0xff, DISPC_OSD_ALPHA);

	/* OSD layer size */
	reg_val = ( var->xres & 0xfff) | (( var->yres & 0xfff ) << 16);
	dispc_write(reg_val, DISPC_OSD_SIZE_XY);

	/* OSD layer start position */
	dispc_write(0, DISPC_OSD_DISP_XY);

	/* OSD layer pitch */
	reg_val = ( var->xres & 0xfff) ;
	dispc_write(reg_val, DISPC_OSD_PITCH);

	/* OSD color_key value */
	dispc_set_osd_ck(0x0);

	/* DISPC workplane size */
	dispc_set_disp_size(var);
}

static void dispc_layer_update(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	/******************* OSD layer setting **********************/

	/*enable OSD layer*/
	reg_val |= (1 << 0);

	/*disable  color key */

	/* alpha mode select  - block alpha*/
	reg_val |= (1 << 2);

	/* data format */
	if (var->bits_per_pixel == 32) {
		/* ABGR */
		reg_val |= (3 << 4);
		/* rb switch */
		reg_val |= (1 << 15);
	} else {
		/* RGB565 */
		reg_val |= (5 << 4);
		/* B2B3B0B1 */
		reg_val |= (2 << 8);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);
}

static int32_t dispc_sync(struct sprdfb_device *dev)
{
	int ret;

	if (dev->enable == 0) {
		printk("sprdfb: dispc_sync fb suspeneded already!!\n");
		return -1;
	}

	ret = wait_event_interruptible_timeout(dispc_ctx.vsync_queue,
			          dispc_ctx.vsync_done, msecs_to_jiffies(100));

	if (!ret) { /* time out */
		dispc_ctx.vsync_done = 1; /*error recovery */
		printk(KERN_ERR "sprdfb: dispc_sync time out!!!!!\n");
		return -1;
	}
	return 0;
}

static void dispc_run(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		/*dpi register update*/
		dispc_set_bits(BIT(5), DISPC_DPI_CTRL);

		udelay(30);

		if(dispc_ctx.is_first_frame){
			/*dpi register update with SW and VSync*/
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);

			/* start refresh */
			dispc_set_bits((1 << 4), DISPC_CTRL);

			dispc_ctx.is_first_frame = false;
		}
	}else{
		/* start refresh */
		dispc_set_bits((1 << 4), DISPC_CTRL);
	}
}

static void dispc_stop(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		/*dpi register update with SW only*/
		dispc_set_bits(BIT(4), DISPC_DPI_CTRL);

		/* stop refresh */
		dispc_clear_bits((1 << 4), DISPC_CTRL);

		dispc_ctx.is_first_frame = true;
	}
}

static int32_t sprdfb_dispc_early_init(struct sprdfb_device *dev)
{
	int ret = 0;
	struct clk *clk_parent1, *clk_parent2, *clk_parent3;

	pr_debug(KERN_INFO "sprdfb:[%s]\n", __FUNCTION__);

	if(dispc_ctx.is_inited){
		printk(KERN_WARNING "sprdfb: dispc early init warning!(has been inited)");
		return 0;
	}

	/*usesd to open dipsc matix clock*/
	__raw_writel((__raw_readl(REG_AHB_MATRIX_CLOCK)) | (1<<DISPC_CORE_CLK_EN) | (1<<DISPMTX_CLK_EN), 
			REG_AHB_MATRIX_CLOCK);

	clk_parent1 = clk_get(NULL, DISPC_CLOCK_PARENT);
	if (IS_ERR(clk_parent1)) {
		printk(KERN_WARNING "sprdfb: get clk_parent1 fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent1 ok!\n");
	}

	clk_parent2 = clk_get(NULL, DISPC_DBI_CLOCK_PARENT);
	if (IS_ERR(clk_parent2)) {
		printk(KERN_WARNING "sprdfb: get clk_parent2 fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent2 ok!\n");
	}

	clk_parent3 = clk_get(NULL, DISPC_DPI_CLOCK_PARENT);
	if (IS_ERR(clk_parent3)) {
		printk(KERN_WARNING "sprdfb: get clk_parent3 fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent3 ok!\n");
	}

	dispc_ctx.clk_dispc = clk_get(NULL, "clk_dispc");
	if (IS_ERR(dispc_ctx.clk_dispc)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc ok!\n");
	}

	dispc_ctx.clk_dispc_dbi = clk_get(NULL, "clk_dispc_dbi");
	if (IS_ERR(dispc_ctx.clk_dispc_dbi)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc_dbi fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc_dbi ok!\n");
	}

	dispc_ctx.clk_dispc_dpi = clk_get(NULL, "clk_dispc_dpi");
	if (IS_ERR(dispc_ctx.clk_dispc_dpi)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc_dpi fail!\n");
		return 0;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc_dpi ok!\n");
	}

	if(!dev->panel_ready){
		ret = clk_set_parent(dispc_ctx.clk_dispc, clk_parent1);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set clk parent fail\n");
		}
		ret = clk_set_rate(dispc_ctx.clk_dispc, DISPC_CLOCK);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set clk parent fail\n");
		}

		ret = clk_set_parent(dispc_ctx.clk_dispc_dbi, clk_parent2);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dbi clk parent fail\n");
		}
		ret = clk_set_rate(dispc_ctx.clk_dispc_dbi, DISPC_DBI_CLOCK);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dbi clk parent fail\n");
		}

		ret = clk_set_parent(dispc_ctx.clk_dispc_dpi, clk_parent3);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dpi clk parent fail\n");
		}
		ret = clk_set_rate(dispc_ctx.clk_dispc_dpi, DISPC_DPI_CLOCK);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dpi clk parent fail\n");
		}

		ret = clk_enable(dispc_ctx.clk_dispc);
		if (ret) {
			printk(KERN_WARNING "sprdfb: enable clk_dispc fail!\n");
			return 0;
		} else {
			pr_debug(KERN_INFO "sprdfb: get clk_dispc ok!\n");
		}

		ret = clk_enable(dispc_ctx.clk_dispc_dbi);
		if (ret) {
			printk(KERN_WARNING "sprdfb: enable clk_dispc_dbi fail!\n");
			clk_disable(dispc_ctx.clk_dispc);
			return 0;
		} else {
			pr_debug(KERN_INFO "sprdfb: get clk_dispc_dbi ok!\n");
		}

		ret = clk_enable(dispc_ctx.clk_dispc_dpi);
		if (ret) {
			printk(KERN_WARNING "sprdfb: enable clk_dispc_dpi fail!\n");
			clk_disable(dispc_ctx.clk_dispc);
			clk_disable(dispc_ctx.clk_dispc_dbi);
			return 0;
		} else {
			pr_debug(KERN_INFO "sprdfb: get clk_dispc_dpi ok!\n");
		}

		dispc_reset();
		dispc_module_enable();
		dispc_ctx.is_first_frame = true;
	}else{
		dispc_ctx.is_first_frame = false;
	}

	dispc_ctx.vsync_done = 1;
	dispc_ctx.vsync_waiter = 0;
	init_waitqueue_head(&(dispc_ctx.vsync_queue));

	dispc_ctx.is_inited = true;

	ret = request_irq(IRQ_DISPC_INT, dispc_isr, IRQF_DISABLED, "DISPC", &dispc_ctx);
	if (ret) {
		printk(KERN_ERR "sprdfb: dispcfailed to request irq!\n");
		clk_disable(dispc_ctx.clk_dispc);
		clk_disable(dispc_ctx.clk_dispc_dbi);
		clk_disable(dispc_ctx.clk_dispc_dpi);
		dispc_ctx.is_inited = false;
		return -1;
	}

	return 0;
}


static int32_t sprdfb_dispc_init(struct sprdfb_device *dev)
{
	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);

	/*set bg color*/
	dispc_set_bg_color(0xFFFFFFFF);
	/*enable dithering*/
	dispc_dithering_enable(true);
	/*use MSBs as img exp mode*/
	dispc_set_exp_mode(0x0);

	if(dispc_ctx.is_first_frame){
		dispc_layer_init(&(dev->fb->var));
	}else{
		dispc_layer_update(&(dev->fb->var));
	}

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(dispc_ctx.is_first_frame){
			/*set dpi register update only with SW*/
			dispc_set_bits(BIT(4), DISPC_DPI_CTRL);
		}else{
			/*set dpi register update with SW & VSYNC*/
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);
		}
		/*enable dispc update done INT*/
		dispc_write((1<<4), DISPC_INT_EN);
	}else{
		/* enable dispc DONE  INT*/
		dispc_write((1<<0), DISPC_INT_EN);
	}
	dev->enable = 1;
	return 0;
}

static int32_t sprdfb_dispc_uninit(struct sprdfb_device *dev)
{
	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);

	dev->enable = 0;
	if(dispc_ctx.is_inited){
		clk_disable(dispc_ctx.clk_dispc);
		dispc_ctx.is_inited = false;
	}
	return 0;
}

static int32_t sprdfb_dispc_refresh (struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;

	uint32_t base = fb->fix.smem_start + fb->fix.line_length * fb->var.yoffset;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);

	dispc_ctx.vsync_waiter ++;
	dispc_sync(dev);
	pr_debug(KERN_INFO "srpdfb: [%s] got sync\n", __FUNCTION__);

	dispc_ctx.dev = dev;
	dispc_ctx.vsync_done = 0;

#ifdef LCD_UPDATE_PARTLY
	if ((fb->var.reserved[0] == 0x6f766572) &&(SPRDFB_PANEL_IF_DPI != dev->panel_if_type)) {
		uint32_t x,y, width, height;

		x = fb->var.reserved[1] & 0xffff;
		y = fb->var.reserved[1] >> 16;
		width  = fb->var.reserved[2] &  0xffff;
		height = fb->var.reserved[2] >> 16;

		base += ((x + y * fb->var.xres) * fb->var.bits_per_pixel / 8);
		dispc_write(base, DISPC_OSD_BASE_ADDR);
		dispc_write(0, DISPC_OSD_DISP_XY);
		dispc_write(fb->var.reserved[2], DISPC_OSD_SIZE_XY);
		dispc_write(fb->var.xres, DISPC_OSD_PITCH);

		dispc_write(fb->var.reserved[2], DISPC_SIZE_XY);

		sprdfb_panel_invalidate_rect(dev->panel,
					x, y, x+width-1, y+height-1);
	} else
#endif
	{
		uint32_t size = (fb->var.xres & 0xffff) | ((fb->var.yres) << 16);

		dispc_write(base, DISPC_OSD_BASE_ADDR);
		dispc_write(0, DISPC_OSD_DISP_XY);
		dispc_write(size, DISPC_OSD_SIZE_XY);
		dispc_write(fb->var.xres, DISPC_OSD_PITCH);

		dispc_write(size, DISPC_SIZE_XY);

		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			sprdfb_panel_invalidate(dev->panel);
		}
	}

	sprdfb_panel_before_refresh(dev);

	dispc_run(dev);

	pr_debug("DISPC_CTRL: 0x%x\n", dispc_read(DISPC_CTRL));
	pr_debug("DISPC_SIZE_XY: 0x%x\n", dispc_read(DISPC_SIZE_XY));

	pr_debug("DISPC_BG_COLOR: 0x%x\n", dispc_read(DISPC_BG_COLOR));

	pr_debug("DISPC_INT_EN: 0x%x\n", dispc_read(DISPC_INT_EN));

	pr_debug("DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("DISPC_OSD_BASE_ADDR: 0x%x\n", dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("DISPC_OSD_SIZE_XY: 0x%x\n", dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("DISPC_OSD_PITCH: 0x%x\n", dispc_read(DISPC_OSD_PITCH));
	pr_debug("DISPC_OSD_DISP_XY: 0x%x\n", dispc_read(DISPC_OSD_DISP_XY));
	pr_debug("DISPC_OSD_ALPHA	: 0x%x\n", dispc_read(DISPC_OSD_ALPHA));
	return 0;
}

static int32_t sprdfb_dispc_suspend(struct sprdfb_device *dev)
{
	printk(KERN_INFO "sprdfb:[%s], dev->enable = %d\n",__FUNCTION__, dev->enable);

	if (0 != dev->enable){
		/* must wait ,dispc_sync() */
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
		printk(KERN_INFO "sprdfb:[%s] got sync\n",__FUNCTION__);

		dispc_stop(dev);

		sprdfb_panel_suspend(dev);

		dev->enable = 0;
		clk_disable(dispc_ctx.clk_dispc);
	}else{
		printk(KERN_ERR "sprdfb: [%s]: Invalid device status %d\n", __FUNCTION__, dev->enable);
	}
	return 0;
}

static int32_t sprdfb_dispc_resume(struct sprdfb_device *dev)
{
	printk(KERN_INFO "sprdfb:[%s], dev->enable= %d\n",__FUNCTION__, dev->enable);

	if (dev->enable == 0) {
		clk_enable(dispc_ctx.clk_dispc);
		dispc_ctx.vsync_done = 1;
		if (0 == dispc_read(DISPC_CTRL)) { /* resume from deep sleep */
			dispc_reset();
			dispc_module_enable();
			dispc_ctx.is_first_frame = true;
			sprdfb_dispc_init(dev);

			sprdfb_panel_resume(dev, true);
		} else {
			sprdfb_panel_resume(dev, false);
		}

		dev->enable = 1;
	}
	return 0;
}


struct display_ctrl sprdfb_dispc_ctrl = {
	.name		= "dispc",
	.early_init		= sprdfb_dispc_early_init,
	.init		 	= sprdfb_dispc_init,
	.uninit		= sprdfb_dispc_uninit,
	.refresh		= sprdfb_dispc_refresh,
	.suspend		= sprdfb_dispc_suspend,
	.resume		= sprdfb_dispc_resume,
};


