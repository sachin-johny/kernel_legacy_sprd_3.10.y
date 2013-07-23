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
//#include <mach/hardware.h>
//#include <mach/globalregs.h>
//#include <mach/irqs.h>
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"
#include "sprdfb.h"
#include "sprdfb_chip_common.h"


#define DISPC_CLOCK_PARENT ("clk_256m")
#define DISPC_CLOCK (256*1000000)
#define DISPC_DBI_CLOCK_PARENT ("clk_256m")
#define DISPC_DBI_CLOCK (256*1000000)
#define DISPC_DPI_CLOCK_PARENT ("clk_384m")

#define SPRDFB_DPI_CLOCK_SRC (384000000)

#define SPRDFB_CONTRAST (74)
#define SPRDFB_SATURATION (73)
#define SPRDFB_BRIGHTNESS (2)

typedef enum
{
   SPRDFB_DYNAMIC_CLK_FORCE,		//force enable/disable
   SPRDFB_DYNAMIC_CLK_REFRESH,		//enable for refresh/display_overlay
   SPRDFB_DYNAMIC_CLK_COUNT,		//enable disable in pairs
   SPRDFB_DYNAMIC_CLK_MAX,
} SPRDFB_DYNAMIC_CLK_SWITCH_E;

struct sprdfb_dispc_context {
	struct clk		*clk_dispc;
	struct clk 		*clk_dispc_dpi;
	struct clk 		*clk_dispc_dbi;
	bool			is_inited;
	bool			is_first_frame;
	bool			is_resume;
	bool			is_wait_for_suspend;

	bool			clk_is_open;
	bool			clk_is_refreshing;
	int				clk_open_count;
	spinlock_t clk_spinlock;

	struct sprdfb_device	*dev;

	uint32_t	 	vsync_waiter;
	wait_queue_head_t		vsync_queue;
	uint32_t	        vsync_done;

#ifdef  CONFIG_FB_LCD_OVERLAY_SUPPORT
	/* overlay */
	uint32_t  overlay_state;  /*0-closed, 1-configed, 2-started*/
//	struct semaphore   overlay_lock;
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	uint32_t	 	waitfor_vsync_waiter;
	wait_queue_head_t		waitfor_vsync_queue;
	uint32_t	        waitfor_vsync_done;
#endif
};

static struct sprdfb_dispc_context dispc_ctx = {0};

void clk_force_disable(struct clk *clk);
extern void sprdfb_panel_suspend(struct sprdfb_device *dev);
extern void sprdfb_panel_resume(struct sprdfb_device *dev, bool from_deep_sleep);
extern void sprdfb_panel_before_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_after_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_invalidate(struct panel_spec *self);
extern void sprdfb_panel_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);

#ifdef CONFIG_FB_ESD_SUPPORT
extern uint32_t sprdfb_panel_ESD_check(struct sprdfb_device *dev);
#endif

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int overlay_start(struct sprdfb_device *dev, uint32_t layer_index);
static int overlay_close(struct sprdfb_device *dev);
#endif

static int sprdfb_dispc_clk_disable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type);
static int sprdfb_dispc_clk_enable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type);
static void dispc_clk_clear_status(struct sprdfb_dispc_context *dispc_ctx_ptr);
static int32_t sprdfb_dispc_init(struct sprdfb_device *dev);
static void dispc_reset(void);
static void dispc_module_enable(void);
static void dispc_stop_for_feature(struct sprdfb_device *dev);
static void dispc_run_for_feature(struct sprdfb_device *dev);


static irqreturn_t dispc_isr(int irq, void *data)
{
	struct sprdfb_dispc_context *dispc_ctx = (struct sprdfb_dispc_context *)data;
	uint32_t reg_val;
	struct sprdfb_device *dev = dispc_ctx->dev;
	bool done = false;
#ifdef CONFIG_FB_VSYNC_SUPPORT
	bool vsync =  false;
#endif

	reg_val = dispc_read(DISPC_INT_STATUS);

	pr_debug("dispc_isr (0x%x)\n",reg_val );

	if(reg_val & 0x04){
		printk("Warning: dispc underflow (0x%x)!\n",reg_val);
		dispc_write(0x04, DISPC_INT_CLR);
	}

	if(NULL == dev){
		return IRQ_HANDLED;
	}

	if((reg_val & 0x10) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){/*dispc update done isr*/
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
	}else if ((reg_val & 0x1) && (SPRDFB_PANEL_IF_DPI !=  dev->panel_if_type)){ /* dispc done isr */
			dispc_write(1, DISPC_INT_CLR);
			dispc_ctx->is_first_frame = false;
			done = true;
	}
#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	if((reg_val & 0x2) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){ /*dispc external TE isr*/
		dispc_write(0x2, DISPC_INT_CLR);
		if(0 != dev->esd_te_waiter){
			printk("sprdfb:dispc_isr esd_te_done!");
			dev->esd_te_done =1;
			wake_up_interruptible_all(&(dev->esd_te_queue));
			dev->esd_te_waiter = 0;
		}
	}
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	if((reg_val & 0x1) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){/*dispc done isr*/
		dispc_write(1, DISPC_INT_CLR);
		vsync = true;
	}else if((reg_val & 0x2) && (SPRDFB_PANEL_IF_EDPI ==  dev->panel_if_type)){ /*dispc te isr*/
		dispc_write(2, DISPC_INT_CLR);
		vsync = true;
	}
	if(vsync){
		//printk("sprdfb: got vsync!\n");
		dispc_ctx->waitfor_vsync_done = 1;
		if(dispc_ctx->waitfor_vsync_waiter){
			//printk("sprdfb: wake!\n");
			wake_up_interruptible_all(&(dispc_ctx->waitfor_vsync_queue));
		}
	}
#endif

	if(done){
		dispc_ctx->vsync_done = 1;

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(SPRDFB_PANEL_IF_DPI !=  dev->panel_if_type){
			sprdfb_dispc_clk_disable(dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH);
		}
#endif

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
	printk("REG_AHB_SOFT_RST:%x ,BIT_DISPC_SOFT_RST:%lx \n",REG_AHB_SOFT_RST,BIT_DISPC_SOFT_RST);
	printk("REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
	sci_glb_set(REG_AHB_SOFT_RST, (BIT_DISPC_SOFT_RST) );
	printk("REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
 	udelay(10);
	sci_glb_clr(REG_AHB_SOFT_RST, (BIT_DISPC_SOFT_RST) );
	printk("REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
}

static inline void dispc_set_bg_color(uint32_t bg_color)
{
	dispc_write(bg_color, DISPC_BG_COLOR);
}

static inline void dispc_set_osd_ck(uint32_t ck_color)
{
	dispc_write(ck_color, DISPC_OSD_CK);
}

static inline void dispc_osd_enable(bool is_enable)
{
	uint32_t reg_val;

	reg_val = dispc_read(DISPC_OSD_CTRL);
	if(is_enable){
		reg_val = reg_val | (BIT(0));
	}
	else{
		reg_val = reg_val & (~(BIT(0)));
	}
	dispc_write(reg_val, DISPC_OSD_CTRL);
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
		{/*for debug*/
			int32_t i;
			for(i=0;i<256;i+=16){
				printk("sprdfb: %x: 0x%x, 0x%x, 0x%x, 0x%x\n", i, dispc_read(i), dispc_read(i+4), dispc_read(i+8), dispc_read(i+12));
			}
			printk("**************************************\n");
		}

		return -1;
	}
	return 0;
}


static void dispc_run(struct sprdfb_device *dev)
{
	if(0 == dev->enable){
		return;
	}

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(!dispc_ctx.is_first_frame){
			dispc_ctx.vsync_done = 0;
			dispc_ctx.vsync_waiter ++;
		}

		/*dpi register update*/
		dispc_set_bits(BIT(5), DISPC_DPI_CTRL);

		udelay(30);

		if(dispc_ctx.is_first_frame){
			/*dpi register update with SW and VSync*/
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);

			/* start refresh */
			dispc_set_bits((1 << 4), DISPC_CTRL);

			dispc_ctx.is_first_frame = false;
		}else{
			dispc_sync(dev);
		}
	}else{
		dispc_ctx.vsync_done = 0;
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

static void dispc_update_clock(struct sprdfb_device *dev)
{
	uint32_t hpixels, vlines, need_clock,  dividor;
	int ret = 0;

	struct panel_spec* panel = dev->panel;
	struct info_mipi * mipi = panel->info.mipi;
	struct info_rgb* rgb = panel->info.rgb;

	pr_debug("sprdfb:[%s]\n", __FUNCTION__);

	if(0 == panel->fps){
		printk("sprdfb: No panel->fps specified!\n");
		return;
	}


	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(LCD_MODE_DSI == dev->panel->type ){
			hpixels = panel->width + mipi->timing->hsync + mipi->timing->hbp + mipi->timing->hfp;
			vlines = panel->height + mipi->timing->vsync + mipi->timing->vbp + mipi->timing->vfp;
		}else if(LCD_MODE_RGB == dev->panel->type ){
			hpixels = panel->width + rgb->timing->hsync + rgb->timing->hbp + rgb->timing->hfp;
			vlines = panel->height + rgb->timing->vsync + rgb->timing->vbp + rgb->timing->vfp;
		}else{
			printk("sprdfb:[%s] unexpected panel type!(%d)\n", __FUNCTION__, dev->panel->type);
			return;
		}

		need_clock = hpixels * vlines * panel->fps;
		dividor  = SPRDFB_DPI_CLOCK_SRC/need_clock;
		if(SPRDFB_DPI_CLOCK_SRC - dividor*need_clock > (need_clock/2) ) {
			dividor += 1;
		}

		if((dividor < 1) || (dividor > 0x100)){
			printk("sprdfb:[%s]: Invliad dividor(%d)!Not update dpi clock!\n", __FUNCTION__, dividor);
			return;
		}

		dev->dpi_clock = SPRDFB_DPI_CLOCK_SRC/dividor;

		ret = clk_set_rate(dispc_ctx.clk_dispc_dpi, dev->dpi_clock);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dpi clk parent fail\n");
		}

		printk("sprdfb:[%s] need_clock = %d, dividor = %d, dpi_clock = %d\n", __FUNCTION__, need_clock, dividor, dev->dpi_clock);
	}

}

static int32_t sprdfb_dispc_uninit(struct sprdfb_device *dev)
{
	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);

	dev->enable = 0;
	sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);

	return 0;
}

static int32_t dispc_clk_init(struct sprdfb_device *dev)
{
	int ret = 0;
	struct clk *clk_parent1, *clk_parent2, *clk_parent3;

	pr_debug(KERN_INFO "sprdfb:[%s]\n", __FUNCTION__);

	dispc_print_clk();
	printk("zcf:BIT_DISPC_CORE_EN:%lx,DISPC_CORE_EN:%x\n",BIT_DISPC_CORE_EN,DISPC_CORE_EN);
	printk("zcf:BIT_DISPC_EMC_EN:%lx,DISPC_EMC_EN:%x\n",BIT_DISPC_EMC_EN,DISPC_EMC_EN);
	printk("zcf:DISPC_CORE_EN:%x\n",dispc_glb_read(DISPC_CORE_EN));
	printk("zcf:DISPC_EMC_EN:%x\n",dispc_glb_read(DISPC_EMC_EN));
	sci_glb_set(DISPC_CORE_EN, BIT_DISPC_CORE_EN);
	sci_glb_set(DISPC_EMC_EN, BIT_DISPC_EMC_EN);

	printk("zcf:DISPC_CORE_EN:%x\n",dispc_glb_read(DISPC_CORE_EN));
	printk("zcf:DISPC_EMC_EN:%x\n",dispc_glb_read(DISPC_EMC_EN));

	clk_parent1 = clk_get(NULL, DISPC_CLOCK_PARENT);
	if (IS_ERR(clk_parent1)) {
		printk(KERN_WARNING "sprdfb: get clk_parent1 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent1 ok!\n");
	}

	clk_parent2 = clk_get(NULL, DISPC_DBI_CLOCK_PARENT);
	if (IS_ERR(clk_parent2)) {
		printk(KERN_WARNING "sprdfb: get clk_parent2 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent2 ok!\n");
	}

	clk_parent3 = clk_get(NULL, DISPC_DPI_CLOCK_PARENT);
	if (IS_ERR(clk_parent3)) {
		printk(KERN_WARNING "sprdfb: get clk_parent3 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_parent3 ok!\n");
	}

	dispc_ctx.clk_dispc = clk_get(NULL, DISPC_PLL_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc ok!\n");
	}

	dispc_ctx.clk_dispc_dbi = clk_get(NULL, DISPC_DBI_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc_dbi)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc_dbi fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc_dbi ok!\n");
	}

	dispc_ctx.clk_dispc_dpi = clk_get(NULL, DISPC_DPI_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc_dpi)) {
		printk(KERN_WARNING "sprdfb: get clk_dispc_dpi fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb: get clk_dispc_dpi ok!\n");
	}

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

	if(0 != dev->panel->fps){
		dispc_update_clock(dev);
	}else{
		dev->dpi_clock = DISPC_DPI_CLOCK;
		ret = clk_set_rate(dispc_ctx.clk_dispc_dpi, DISPC_DPI_CLOCK);
		if(ret){
			printk(KERN_ERR "sprdfb: dispc set dpi clk parent fail\n");
		}
	}

	ret = sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);
	if (ret) {
		printk(KERN_WARNING "sprdfb:[%s] enable dispc_clk fail!\n",__FUNCTION__);
		return -1;
	} else {
		pr_debug(KERN_INFO "sprdfb:[%s] enable dispc_clk ok!\n",__FUNCTION__);
	}

	dispc_print_clk();

	return 0;
}

static int32_t sprdfb_dispc_module_init(struct sprdfb_device *dev)
{
	int ret = 0;

	if(dispc_ctx.is_inited){
		printk(KERN_WARNING "sprdfb: dispc_module has already initialized! warning!!");
		return 0;
	}
	else{
		printk(KERN_INFO "sprdfb: dispc_module_init. call only once!");
	}

	spin_lock_init(&dispc_ctx.clk_spinlock);
	dispc_clk_clear_status(&dispc_ctx);
	dispc_ctx.is_resume=false;

	dispc_ctx.vsync_done = 1;
	dispc_ctx.vsync_waiter = 0;
	init_waitqueue_head(&(dispc_ctx.vsync_queue));

#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	init_waitqueue_head(&(dev->esd_te_queue));
	dev->esd_te_waiter = 0;
	dev->esd_te_done = 0;
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	dispc_ctx.waitfor_vsync_done = 0;
	dispc_ctx.waitfor_vsync_waiter = 0;
	init_waitqueue_head(&(dispc_ctx.waitfor_vsync_queue));
#endif
	sema_init(&dev->refresh_lock, 1);

	ret = request_irq(IRQ_DISPC_INT, dispc_isr, IRQF_DISABLED, "DISPC", &dispc_ctx);
	if (ret) {
		printk(KERN_ERR "sprdfb: dispcfailed to request irq!\n");
		sprdfb_dispc_uninit(dev);
		return -1;
	}

	dispc_ctx.is_inited = true;
	return 0;

}

static int32_t sprdfb_dispc_early_init(struct sprdfb_device *dev)
{
	int ret = 0;

	ret = dispc_clk_init(dev);
	if(ret){
		printk(KERN_WARNING "sprdfb: dispc_clk_init fail!\n");
		return -1;
	}

	if(!dispc_ctx.is_inited){
		//init
		if(dev->panel_ready){
			//panel ready
			printk(KERN_INFO "sprdfb:[%s]: dispc has alread initialized\n", __FUNCTION__);
			dispc_ctx.is_first_frame = false;
		}else{
			//panel not ready
			printk(KERN_INFO "sprdfb:[%s]: dispc is not initialized\n", __FUNCTION__);
			dispc_reset();
			dispc_module_enable();
			dispc_ctx.is_first_frame = true;
		}
		ret = sprdfb_dispc_module_init(dev);
	}else{
		//resume
		printk(KERN_INFO "sprdfb:[%s]: sprdfb_dispc_early_init resume\n", __FUNCTION__);
		dispc_reset();
		dispc_module_enable();
		dispc_ctx.is_first_frame = true;
	}

	return ret;
}


static int sprdfb_dispc_clk_disable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type)
{
	bool is_need_disable=false;
	unsigned long irqflags;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);
	if(!dispc_ctx_ptr){
		return 0;
	}

	spin_lock_irqsave(&dispc_ctx.clk_spinlock, irqflags);
	switch(clock_switch_type){
		case SPRDFB_DYNAMIC_CLK_FORCE:
			is_need_disable=true;
			break;
		case SPRDFB_DYNAMIC_CLK_REFRESH:
			dispc_ctx_ptr->clk_is_refreshing=false;
			if(dispc_ctx_ptr->clk_open_count<=0){
				is_need_disable=true;
			}
			break;
		case SPRDFB_DYNAMIC_CLK_COUNT:
			if(dispc_ctx_ptr->clk_open_count>0){
				dispc_ctx_ptr->clk_open_count--;
				if(dispc_ctx_ptr->clk_open_count==0){
					if(!dispc_ctx_ptr->clk_is_refreshing){
						is_need_disable=true;
					}
				}
			}
			break;
		default:
			break;
	}

	if(dispc_ctx_ptr->clk_is_open && is_need_disable){
		pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_disable real\n");
		clk_disable(dispc_ctx_ptr->clk_dispc);
		clk_disable(dispc_ctx_ptr->clk_dispc_dpi);
		clk_disable(dispc_ctx_ptr->clk_dispc_dbi);
		dispc_ctx_ptr->clk_is_open=false;
		dispc_ctx_ptr->clk_is_refreshing=false;
		dispc_ctx_ptr->clk_open_count=0;
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_disable type=%d refresh=%d,count=%d\n",clock_switch_type,dispc_ctx_ptr->clk_is_refreshing,dispc_ctx_ptr->clk_open_count);
	return 0;
}

static int sprdfb_dispc_clk_enable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type)
{
	int ret = 0;
	bool is_dispc_enable=false;
	bool is_dispc_dpi_enable=false;
	unsigned long irqflags;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);
	if(!dispc_ctx_ptr){
		return -1;
	}

	spin_lock_irqsave(&dispc_ctx.clk_spinlock, irqflags);

	if(!dispc_ctx_ptr->clk_is_open){
		pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_enable real\n");
		ret = clk_enable(dispc_ctx_ptr->clk_dispc);
		if(ret){
			printk("sprdfb:enable clk_dispc error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_enable=true;
		ret = clk_enable(dispc_ctx_ptr->clk_dispc_dpi);
		if(ret){
			printk("sprdfb:enable clk_dispc_dpi error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_dpi_enable=true;
		ret = clk_enable(dispc_ctx_ptr->clk_dispc_dbi);
		if(ret){
			printk("sprdfb:enable clk_dispc_dbi error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		dispc_ctx_ptr->clk_is_open=true;
	}

	switch(clock_switch_type){
		case SPRDFB_DYNAMIC_CLK_FORCE:
			break;
		case SPRDFB_DYNAMIC_CLK_REFRESH:
			dispc_ctx_ptr->clk_is_refreshing=true;
			break;
		case SPRDFB_DYNAMIC_CLK_COUNT:
			dispc_ctx_ptr->clk_open_count++;
			break;
		default:
			break;
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_enable type=%d refresh=%d,count=%d,ret=%d\n",clock_switch_type,dispc_ctx_ptr->clk_is_refreshing,dispc_ctx_ptr->clk_open_count,ret);
	return ret;

ERROR_CLK_ENABLE:
	if(is_dispc_enable){
		clk_disable(dispc_ctx_ptr->clk_dispc);
	}
	if(is_dispc_dpi_enable){
		clk_disable(dispc_ctx_ptr->clk_dispc_dpi);
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	printk("sprdfb:sprdfb_dispc_clk_enable error!!!!!!\n");
	return ret;
}

static void dispc_clk_clear_status(struct sprdfb_dispc_context *dispc_ctx_ptr)
{
	printk("sprdfb:[%s]\n",__FUNCTION__);
	dispc_ctx_ptr->clk_is_open=false;
	dispc_ctx_ptr->clk_is_refreshing=false;
	dispc_ctx_ptr->clk_open_count=0;
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

//	dispc_update_clock(dev);

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
	dispc_set_bits(BIT(2), DISPC_INT_EN);
	dev->enable = 1;
	return 0;
}

static int32_t sprdfb_dispc_refresh (struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;

	uint32_t base = fb->fix.smem_start + fb->fix.line_length * fb->var.yoffset;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);

	down(&dev->refresh_lock);

	if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
//		dispc_ctx.vsync_done = 0;
		if(dispc_ctx.is_wait_for_suspend){
			printk("sprdfb: [%s]: do not refresh in suspend!!!\n", __FUNCTION__);
			goto ERROR_REFRESH;
		}
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH)){
			printk(KERN_WARNING "sprdfb: enable dispc_clk fail in refresh!\n");
			goto ERROR_REFRESH;
		}
#endif
	}

	pr_debug(KERN_INFO "srpdfb: [%s] got sync\n", __FUNCTION__);

	dispc_ctx.dev = dev;
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if(SPRD_OVERLAY_STATUS_STARTED == dispc_ctx.overlay_state){
		overlay_close(dev);
	}
#endif

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

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	dispc_set_bits(BIT(0), DISPC_OSD_CTRL);
	if(SPRD_OVERLAY_STATUS_ON == dispc_ctx.overlay_state){
		overlay_start(dev, (SPRD_LAYER_IMG));
	}
#endif

	dispc_run(dev);

#ifdef CONFIG_FB_ESD_SUPPORT
	if(!dev->ESD_work_start){
		printk("sprdfb: schedule ESD work queue!\n");
		schedule_delayed_work(&dev->ESD_work, msecs_to_jiffies(dev->ESD_timeout_val));
		dev->ESD_work_start = true;
	}
#endif

ERROR_REFRESH:
	up(&dev->refresh_lock);
    if(dev->panel->is_clean_lcd){
		if(dispc_ctx.is_resume){
			dispc_osd_enable(true);
			dispc_ctx.is_resume =false;
		}
    }

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

		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			dispc_ctx.is_wait_for_suspend=true;
			/* must wait ,dispc_sync() */
			dispc_ctx.vsync_waiter ++;
			dispc_sync(dev);
			dispc_ctx.is_wait_for_suspend=false;
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
			printk("sprdfb: open clk in suspend\n");
			if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
				printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
				//return 0;
			}
#endif
			printk(KERN_INFO "sprdfb:[%s] got sync\n",__FUNCTION__);
		}

		dev->enable = 0;

#ifdef CONFIG_FB_ESD_SUPPORT
		if(dev->ESD_work_start == true){
			printk("sprdfb: cancel ESD work queue\n");
			cancel_delayed_work_sync(&dev->ESD_work);
			dev->ESD_work_start = false;
		}

#endif

		sprdfb_panel_suspend(dev);

		dispc_stop(dev);

		mdelay(50); /*fps>20*/

		sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);
		sci_glb_clr(DISPC_EMC_EN, BIT_DISPC_EMC_EN);
	}else{
		printk(KERN_ERR "sprdfb: [%s]: Invalid device status %d\n", __FUNCTION__, dev->enable);
	}
	return 0;
}

static int32_t sprdfb_dispc_resume(struct sprdfb_device *dev)
{
	printk(KERN_INFO "sprdfb:[%s], dev->enable= %d\n",__FUNCTION__, dev->enable);

	if (dev->enable == 0) {
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!\n",__FUNCTION__);
			//return 0;
		}
		sci_glb_set(DISPC_EMC_EN, BIT_DISPC_EMC_EN);

		dispc_ctx.vsync_done = 1;
		if (dispc_read(DISPC_SIZE_XY) == 0 ) { /* resume from deep sleep */
			printk(KERN_INFO "sprdfb:[%s] from deep sleep\n",__FUNCTION__);
			dev->is_deepsleep=true;
			sprdfb_dispc_early_init(dev);
			sprdfb_dispc_init(dev);
			sprdfb_panel_resume(dev, true);
			dev->is_deepsleep=false;
		}else {
			printk(KERN_INFO "sprdfb:[%s]  not from deep sleep\n",__FUNCTION__);

			sprdfb_panel_resume(dev, true);
		}

		dev->enable = 1;
		if(dev->panel->is_clean_lcd){
			dispc_osd_enable(false);
			dispc_set_bg_color(0x00);
			sprdfb_dispc_refresh(dev);
			mdelay(30);
			dispc_ctx.is_resume=true;
		}

	}
	printk(KERN_INFO "sprdfb:[%s], leave dev->enable= %d\n",__FUNCTION__, dev->enable);

	return 0;
}


#ifdef CONFIG_FB_ESD_SUPPORT
static int32_t sprdfb_disc_check_esd(struct sprdfb_device *dev)
{
	uint32_t ret = 0;
	bool	is_clk_enable=false;
	bool	is_need_run=false;
	bool	is_refresh_lock_down=false;

	pr_debug("sprdfb: [%s] \n", __FUNCTION__);

	/*Jessica TODO: need add other mode support*/
	/*only support command mode now*/
	if(SPRDFB_PANEL_IF_DBI == dev->panel_if_type){
		pr_debug("sprdfb: [%s] leave (not support dbi mode now)!\n", __FUNCTION__);
		ret = -1;
		goto ERROR_CHECK_ESD;
	}
	down(&dev->refresh_lock);
	is_refresh_lock_down=true;
	if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
		if(dispc_ctx.is_wait_for_suspend){
			printk("sprdfb: [%s]: do not check esd in suspend!!!\n", __FUNCTION__);
			goto ERROR_CHECK_ESD;
		}
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
			ret = -1;
			goto ERROR_CHECK_ESD;
		}
		is_clk_enable=true;
#endif
	}

	if(0 == dev->enable){
		printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		ret=-1;
		goto ERROR_CHECK_ESD;
	}

#ifndef FB_CHECK_ESD_BY_TE_SUPPORT
	//for video esd check
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		dispc_stop_for_feature(dev);
		is_need_run=true;
	}
#endif

	ret = sprdfb_panel_ESD_check(dev);

	if(0 == dev->enable){
		printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		ret=-1;
		goto ERROR_CHECK_ESD;
	}

	if(0 !=ret || is_need_run){
		dispc_run_for_feature(dev);
		is_need_run=false;
	}

ERROR_CHECK_ESD:
	if(is_clk_enable){
		sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT);
	}
	if(is_need_run){
		dispc_run_for_feature(dev);
	}
	if(is_refresh_lock_down){
		up(&dev->refresh_lock);
	}

	return ret;
}
#endif


#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int overlay_open(void)
{
	pr_debug("sprdfb: [%s] : %d\n", __FUNCTION__,dispc_ctx.overlay_state);

/*
	if(SPRD_OVERLAY_STATUS_OFF  != dispc_ctx.overlay_state){
		printk(KERN_ERR "sprdfb: Overlay open fail (has been opened)");
		return -1;
	}
*/

	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_ON;
	return 0;
}

static int overlay_start(struct sprdfb_device *dev, uint32_t layer_index)
{
	pr_debug("sprdfb: [%s] : %d, %d\n", __FUNCTION__,dispc_ctx.overlay_state, layer_index);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "sprdfb: overlay start fail. (not opened)");
		return -1;
	}

	if((0 == dispc_read(DISPC_IMG_Y_BASE_ADDR)) && (0 == dispc_read(DISPC_OSD_BASE_ADDR))){
		printk(KERN_ERR "sprdfb: overlay start fail. (not configged)");
		return -1;
	}

/*
	if(0 != dispc_sync(dev)){
		printk(KERN_ERR "sprdfb: overlay start fail. (wait done fail)");
		return -1;
	}
*/
	dispc_set_bg_color(0x0);
	dispc_clear_bits(BIT(2), DISPC_OSD_CTRL); /*use pixel alpha*/
	dispc_write(0x80, DISPC_OSD_ALPHA);

	if((layer_index & SPRD_LAYER_IMG) && (0 != dispc_read(DISPC_IMG_Y_BASE_ADDR))){
		dispc_set_bits(BIT(0), DISPC_IMG_CTRL);/* enable the image layer */
	}
	if((layer_index & SPRD_LAYER_OSD) && (0 != dispc_read(DISPC_OSD_BASE_ADDR))){
		dispc_set_bits(BIT(0), DISPC_OSD_CTRL);/* enable the osd layer */
	}
	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_STARTED;
	return 0;
}

static int overlay_img_configure(struct sprdfb_device *dev, int type, overlay_rect *rect, unsigned char *buffer, int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;

	pr_debug("sprdfb: [%s] : %d, (%d, %d,%d,%d), 0x%x\n", __FUNCTION__, type, rect->x, rect->y, rect->h, rect->w, (unsigned int)buffer);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "sprdfb: Overlay config fail (not opened)");
		return -1;
	}

	if (type >= SPRD_DATA_TYPE_LIMIT) {
		printk(KERN_ERR "sprdfb: Overlay config fail (type error)");
		return -1;
	}

	if((y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT) || (uv_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT)){
		printk(KERN_ERR "sprdfb: Overlay config fail (y, uv endian error)");
		return -1;
	}

/*	lcdc_write(((type << 3) | (1 << 0)), LCDC_IMG_CTRL); */
	/*lcdc_write((type << 3) , LCDC_IMG_CTRL);*/
	reg_value = (y_endian << 8)|(uv_endian<< 10)|(type << 4);
	if(rb_switch){
		reg_value |= (1 << 15);
	}
	dispc_write(reg_value, DISPC_IMG_CTRL);

	dispc_write((uint32_t)buffer, DISPC_IMG_Y_BASE_ADDR);
	if (type < SPRD_DATA_TYPE_RGB888) {
		uint32_t size = rect->w * rect->h;
		dispc_write((uint32_t)(buffer + size), DISPC_IMG_UV_BASE_ADDR);
	}

	reg_value = (rect->h << 16) | (rect->w);
	dispc_write(reg_value, DISPC_IMG_SIZE_XY);

	dispc_write(rect->w, DISPC_IMG_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_IMG_DISP_XY);

	if(type < SPRD_DATA_TYPE_RGB888) {
		dispc_write(1, DISPC_Y2R_CTRL);
		dispc_write(SPRDFB_CONTRAST, DISPC_Y2R_CONTRAST);
		dispc_write(SPRDFB_SATURATION, DISPC_Y2R_SATURATION);
		dispc_write(SPRDFB_BRIGHTNESS, DISPC_Y2R_BRIGHTNESS);
	}

	pr_debug("DISPC_IMG_CTRL: 0x%x\n", dispc_read(DISPC_IMG_CTRL));
	pr_debug("DISPC_IMG_Y_BASE_ADDR: 0x%x\n", dispc_read(DISPC_IMG_Y_BASE_ADDR));
	pr_debug("DISPC_IMG_UV_BASE_ADDR: 0x%x\n", dispc_read(DISPC_IMG_UV_BASE_ADDR));
	pr_debug("DISPC_IMG_SIZE_XY: 0x%x\n", dispc_read(DISPC_IMG_SIZE_XY));
	pr_debug("DISPC_IMG_PITCH: 0x%x\n", dispc_read(DISPC_IMG_PITCH));
	pr_debug("DISPC_IMG_DISP_XY: 0x%x\n", dispc_read(DISPC_IMG_DISP_XY));
	pr_debug("DISPC_Y2R_CTRL: 0x%x\n", dispc_read(DISPC_Y2R_CTRL));
	pr_debug("DISPC_Y2R_CONTRAST: 0x%x\n", dispc_read(DISPC_Y2R_CONTRAST));
	pr_debug("DISPC_Y2R_SATURATION: 0x%x\n", dispc_read(DISPC_Y2R_SATURATION));
	pr_debug("DISPC_Y2R_BRIGHTNESS: 0x%x\n", dispc_read(DISPC_Y2R_BRIGHTNESS));

	return 0;
}

static int overlay_osd_configure(struct sprdfb_device *dev, int type, overlay_rect *rect, unsigned char *buffer, int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;

	pr_debug("sprdfb: [%s] : %d, (%d, %d,%d,%d), 0x%x\n", __FUNCTION__, type, rect->x, rect->y, rect->h, rect->w, (unsigned int)buffer);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "sprdfb: Overlay config fail (not opened)");
		return -1;
	}

	if ((type >= SPRD_DATA_TYPE_LIMIT) || (type <= SPRD_DATA_TYPE_YUV400)) {
		printk(KERN_ERR "sprdfb: Overlay config fail (type error)");
		return -1;
	}

	if(y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT ){
		printk(KERN_ERR "sprdfb: Overlay config fail (rgb endian error)");
		return -1;
	}

/*	lcdc_write(((type << 3) | (1 << 0)), LCDC_IMG_CTRL); */
	/*lcdc_write((type << 3) , LCDC_IMG_CTRL);*/

	/*use premultiply pixel alpha*/
	reg_value = (y_endian<<8)|(type << 4|(1<<2))|(2<<16);
	if(rb_switch){
		reg_value |= (1 << 15);
	}
	dispc_write(reg_value, DISPC_OSD_CTRL);

	dispc_write((uint32_t)buffer, DISPC_OSD_BASE_ADDR);

	reg_value = (rect->h << 16) | (rect->w);
	dispc_write(reg_value, DISPC_OSD_SIZE_XY);

	dispc_write(rect->w, DISPC_OSD_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_OSD_DISP_XY);


	pr_debug("DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("DISPC_OSD_BASE_ADDR: 0x%x\n", dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("DISPC_OSD_SIZE_XY: 0x%x\n", dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("DISPC_OSD_PITCH: 0x%x\n", dispc_read(DISPC_OSD_PITCH));
	pr_debug("DISPC_OSD_DISP_XY: 0x%x\n", dispc_read(DISPC_OSD_DISP_XY));

	return 0;
}

static int overlay_close(struct sprdfb_device *dev)
{
	if(SPRD_OVERLAY_STATUS_OFF  == dispc_ctx.overlay_state){
		printk(KERN_ERR "sprdfb: overlay close fail. (has been closed)");
		return 0;
	}

/*
	if (0 != sprd_lcdc_sync(dev)) {
		printk(KERN_ERR "sprdfb: overlay close fail. (wait done fail)\n");
		return -1;
	}
*/
	dispc_set_bg_color(0xFFFFFFFF);
	dispc_set_bits(BIT(2), DISPC_OSD_CTRL);/*use block alpha*/
	dispc_write(0xff, DISPC_OSD_ALPHA);
	dispc_clear_bits(BIT(0), DISPC_IMG_CTRL);	/* disable the image layer */
	dispc_write(0, DISPC_IMG_Y_BASE_ADDR);
	dispc_write(0, DISPC_OSD_BASE_ADDR);
	dispc_layer_init(&(dev->fb->var));
	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_OFF;

	return 0;
}

/*TO DO: need mutext with suspend, resume*/
static int32_t sprdfb_dispc_enable_overlay(struct sprdfb_device *dev, struct overlay_info* info, int enable)
{
	int result = -1;
	bool	is_refresh_lock_down=false;
	bool	is_clk_enable=false;

	if(0 == dev->enable){
		printk(KERN_ERR "sprdfb: sprdfb_dispc_enable_overlay fail. (dev not enable)\n");
		goto ERROR_ENABLE_OVERLAY;
	}

	pr_debug("sprdfb: [%s]: %d, %d\n", __FUNCTION__, enable,  dev->enable);

	if(enable){  /*enable*/
		if(NULL == info){
			printk(KERN_ERR "sprdfb: sprdfb_dispc_enable_overlay fail (Invalid parameter)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

		down(&dev->refresh_lock);
		is_refresh_lock_down=true;

		if(0 != dispc_sync(dev)){
			printk(KERN_ERR "sprdfb: sprdfb_dispc_enable_overlay fail. (wait done fail)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
				printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
				goto ERROR_ENABLE_OVERLAY;
			}
			is_clk_enable=true;
		}
#endif
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
		if(SPRD_OVERLAY_STATUS_STARTED == dispc_ctx.overlay_state){
			overlay_close(dev);
		}
#endif
		result = overlay_open();
		if(0 != result){
			result=-1;
			goto ERROR_ENABLE_OVERLAY;
		}

		if(SPRD_LAYER_IMG == info->layer_index){
			result = overlay_img_configure(dev, info->data_type, &(info->rect), info->buffer, info->y_endian, info->uv_endian, info->rb_switch);
		}else if(SPRD_LAYER_OSD == info->layer_index){
			result = overlay_osd_configure(dev, info->data_type, &(info->rect), info->buffer, info->y_endian, info->uv_endian, info->rb_switch);
		}else{
			printk(KERN_ERR "sprdfb: sprdfb_dispc_enable_overlay fail. (invalid layer index)\n");
		}
		if(0 != result){
			result=-1;
			goto ERROR_ENABLE_OVERLAY;
		}
		/*result = overlay_start(dev);*/
	}else{   /*disable*/
		/*result = overlay_close(dev);*/
	}
ERROR_ENABLE_OVERLAY:
	if(is_clk_enable){
		sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT);
	}
	if(is_refresh_lock_down){
		up(&dev->refresh_lock);
	}

	pr_debug("sprdfb: [%s] return %d\n", __FUNCTION__, result);
	return result;
}


static int32_t sprdfb_dispc_display_overlay(struct sprdfb_device *dev, struct overlay_display* setting)
{
	struct overlay_rect* rect = &(setting->rect);
	uint32_t size =( (rect->h << 16) | (rect->w & 0xffff));

	dispc_ctx.dev = dev;

	pr_debug("sprdfb: sprdfb_dispc_display_overlay: layer:%d, (%d, %d,%d,%d)\n",
		setting->layer_index, setting->rect.x, setting->rect.y, setting->rect.h, setting->rect.w);

	down(&dev->refresh_lock);

	if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
		//dispc_ctx.vsync_done = 0;
		if(dispc_ctx.is_wait_for_suspend){
			printk("sprdfb: [%s]: do not display overlay in suspend!!!\n", __FUNCTION__);
			goto ERROR_DISPLAY_OVERLAY;
		}
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
			goto ERROR_DISPLAY_OVERLAY;
		}
#endif

	}

	pr_debug(KERN_INFO "srpdfb: [%s] got sync\n", __FUNCTION__);

	dispc_ctx.dev = dev;

#ifdef LCD_UPDATE_PARTLY
	if ((setting->rect->h < dev->panel->height) ||
		(setting->rect->w < dev->panel->width)){
		dispc_write(size, DISPC_SIZE_XY);

		sprdfb_panel_invalidate_rect(dev->panel,
					rect->x, rect->y, rect->x + rect->w-1, rect->y + rect->h-1);
	} else
#endif
	{
		dispc_write(size, DISPC_SIZE_XY);

		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			sprdfb_panel_invalidate(dev->panel);
		}
	}

	sprdfb_panel_before_refresh(dev);

	dispc_clear_bits(BIT(0), DISPC_OSD_CTRL);
	if(SPRD_OVERLAY_STATUS_ON == dispc_ctx.overlay_state){
		overlay_start(dev, setting->layer_index);
	}


	dispc_run(dev);

	if((SPRD_OVERLAY_DISPLAY_SYNC == setting->display_mode) && (SPRDFB_PANEL_IF_DPI != dev->panel_if_type)){
		dispc_ctx.vsync_waiter ++;
		if (dispc_sync(dev) != 0) {/* time out??? disable ?? */
			printk("sprdfb  do sprd_lcdc_display_overlay  time out!\n");
		}
		//dispc_ctx.vsync_done = 0;
	}

ERROR_DISPLAY_OVERLAY:
	up(&dev->refresh_lock);

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

#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
static int32_t spdfb_dispc_wait_for_vsync(struct sprdfb_device *dev)
{
	pr_debug("sprdfb: [%s]\n", __FUNCTION__);
	int ret;

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(!dispc_ctx.is_first_frame){
			dispc_ctx.waitfor_vsync_done = 0;
			dispc_set_bits(BIT(0), DISPC_INT_EN);
			dispc_ctx.waitfor_vsync_waiter++;
			ret  = wait_event_interruptible_timeout(dispc_ctx.waitfor_vsync_queue,
					dispc_ctx.waitfor_vsync_done, msecs_to_jiffies(100));
			dispc_ctx.waitfor_vsync_waiter = 0;
		}
	}else{
		dispc_ctx.waitfor_vsync_done = 0;
		dispc_set_bits(BIT(1), DISPC_INT_EN);
		dispc_ctx.waitfor_vsync_waiter++;
		ret  = wait_event_interruptible_timeout(dispc_ctx.waitfor_vsync_queue,
				dispc_ctx.waitfor_vsync_done, msecs_to_jiffies(100));
		dispc_ctx.waitfor_vsync_waiter = 0;
	}
	pr_debug("sprdfb:[%s] (%d)\n", __FUNCTION__, ret);
	return 0;
}
#endif

static void dispc_stop_for_feature(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		dispc_stop(dev);
		while(dispc_read(DISPC_DPI_STS1) & BIT(16));
		udelay(25);
	}
}

static void dispc_run_for_feature(struct sprdfb_device *dev)
{
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if(SPRD_OVERLAY_STATUS_ON != dispc_ctx.overlay_state)
#endif
	{
		dispc_run(dev);
	}
}


struct display_ctrl sprdfb_dispc_ctrl = {
	.name		= "dispc",
	.early_init		= sprdfb_dispc_early_init,
	.init		 	= sprdfb_dispc_init,
	.uninit		= sprdfb_dispc_uninit,
	.refresh		= sprdfb_dispc_refresh,
	.suspend		= sprdfb_dispc_suspend,
	.resume		= sprdfb_dispc_resume,
	.update_clk	= dispc_update_clock,
#ifdef CONFIG_FB_ESD_SUPPORT
	.ESD_check	= sprdfb_disc_check_esd,
#endif
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	.enable_overlay = sprdfb_dispc_enable_overlay,
	.display_overlay = sprdfb_dispc_display_overlay,
#endif
#ifdef CONFIG_FB_VSYNC_SUPPORT
	.wait_for_vsync = spdfb_dispc_wait_for_vsync,
#endif
};


