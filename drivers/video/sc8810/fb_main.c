/* drivers/video/sc8810/fb_main.c
 *
 * SC8810 LCM0 framebuffer driver.
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
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock_common.h>
#include <mach/lcd.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#endif

#include "lcdc_reg.h"

#include "fb_rrm.h"
#include "lcdc_manager.h" 

//#define  FB_DEBUG 
#ifdef FB_DEBUG
#define FB_PRINT printk
#else
#define FB_PRINT(...)
#endif

#define MAX_LCDC_TIMING_VALUE 15
#define FB_NORMAL 0
#define FB_NO_REFRESH 1

struct sc8810fb_info {
	struct fb_info   *fb;
	struct ops_mcu   *ops;
	struct lcd_spec  *panel;
	struct rrmanager *rrm;
	uint32_t fb_state;
	uint32_t bits_per_pixel;
	uint32_t need_reinit;

	uint32_t write_register_timing;
	uint32_t write_gram_timing;

	struct clk *clk_lcdc; 
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

#if CONFIG_CPU_FREQ
	struct notifier_block freq_transition;
	struct notifier_block freq_policy;
#endif
};

static uint32_t lcdc_calculate_lcm_timing(struct timing_mcu *timing);
static void lcdc_update_lcm_timing(uint32_t reg_value);
static void lcdc_reset(void);
static void hw_early_init(struct sc8810fb_info *info);
static void hw_init(struct sc8810fb_info *info);
static void hw_later_init(struct sc8810fb_info *info);

static int32_t lcm_send_cmd (uint32_t cmd)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_CTRL) & BIT20);

	__raw_writel(cmd, LCM_CD0);

	return 0;
}

static int32_t lcm_send_cmd_data (uint32_t cmd, uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_CTRL) & BIT20);
	
	__raw_writel(cmd, LCM_CD0);

	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_CTRL) & BIT20);

	__raw_writel(data, LCM_DATA0);

	return 0;
}

static int32_t lcm_send_data (uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	 while(__raw_readl(LCM_CTRL) & BIT20);

	__raw_writel(data, LCM_DATA0);

	return 0;
}

static uint32_t lcm_read_data (void)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_CTRL) & BIT20);
	__raw_writel(1 << 24, LCM_DATA0);
	udelay(50);
	return __raw_readl(LCM_RDDATA);
}

static struct ops_mcu lcm_mcu_ops = {
	.send_cmd = lcm_send_cmd,
	.send_cmd_data = lcm_send_cmd_data,
	.send_data = lcm_send_data,
	.read_data = lcm_read_data,
};

extern struct lcdc_manager lm; 
static irqreturn_t lcdc_isr(int irq, void *data)
{
	uint32_t val ;
	struct sc8810fb_info *info = (struct sc8810fb_info *)data;

	val = __raw_readl(LCDC_IRQ_STATUS);
	if (val & (1<<0)) {      /* lcdc done */
		FB_PRINT("--> lcdc_isr lm.mode=%d\n", lm.mode);
		__raw_bits_or((1<<0), LCDC_IRQ_CLR);
		if(lm.mode == LMODE_DISPLAY) {
			lcdc_update_lcm_timing(info->write_register_timing);
			rrm_interrupt(info->rrm);
		}
	}

	return IRQ_HANDLED;
}

#define  LCDC_CLK_NAME   "clk_lcdc"
#define  LCDC_96MHz      0x0
#define  LCDC_64MHz      0x1
#define  LCDC_48MHz      0x2
#define  LCDC_26MHz      0x3

static char *lcdc_get_clk_src_name(unsigned int clk_src)
{
        char *src_name;
	switch(clk_src) {
		case LCDC_96MHz:
			src_name =  "clk_96m";
			break;
		case LCDC_64MHz:
			src_name =  "clk_64m";			
			break;
		case LCDC_48MHz:
			src_name =  "clk_48m";			
			break;
		default:
			src_name =  "clk_26m";				
			break;
	}	
	return src_name;
}

static struct clk *lcdc_init_mclk (unsigned int clk_src)
{
    	char *name_parent = NULL;
    	struct clk *clk_parent = NULL;
	int ret;
	struct clk *clk_lcdc = clk_get(NULL, LCDC_CLK_NAME);

	name_parent = lcdc_get_clk_src_name(clk_src);
	clk_parent = clk_get_parent(clk_lcdc);

	FB_PRINT("###lcdc:clock[%s]: parent_name: %s.\n", clk_lcdc->name, clk_parent->name);
	if (!clk_parent || strcmp(name_parent, clk_parent->name)) {//need to wait the parent
		clk_parent = clk_get(NULL, name_parent);
		if (!clk_parent) {
			printk("###lcdc:clock[%s]: failed to get parent [%s] by clk_get()!\n", clk_lcdc->name, name_parent);
			return NULL;
		}

		ret = clk_set_parent(clk_lcdc, clk_parent);
		if (ret) {
			printk("###lcdc:clock[%s]: clk_set_parent() failed!parent: %s, usecount: %d.\n", clk_lcdc->name, clk_parent->name, clk_lcdc->usecount);
			return NULL;
		}		
	}

	return clk_lcdc;	
}

static int32_t panel_reset(struct lcd_spec *self)
{
	//panel reset
	__raw_writel(0x1, LCM_RSTN);	
	msleep(0x1);
	__raw_writel(0x0, LCM_RSTN);
	msleep(0x1);
	__raw_writel(0x1, LCM_RSTN);
	msleep(0x1);

	return 0;
}

static void lcdc_mcu_init(void)
{
	uint32_t reg_val;
	
	//LCDC module enable
	reg_val = (1<<0);

	/*FMARK mode*/
	//reg_val |= (1<<1);

	/*FMARK pol*/
	reg_val |= (1<<2);

	__raw_writel(reg_val, LCDC_CTRL); 
	
	FB_PRINT("[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));
	
	/* set background*/
	__raw_writel(0x0, LCDC_BG_COLOR);   //red

	FB_PRINT("[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_BG_COLOR));

	/* dithering enable*/
	//__raw_bits_or(1<<4, LCDC_CTRL); 	
}

static int mount_panel(struct sc8810fb_info *info, struct lcd_spec *panel)
{
	/* TODO: check whether the mode/res are supported */
	uint32_t bus_width;
	info->panel = panel;

	panel->info.mcu->ops = info->ops;

	panel->ops->lcd_reset = panel_reset;

	bus_width = info->panel->info.mcu->bus_width;
	if (bus_width == 9 || bus_width == 18 || bus_width == 24) {
		info->bits_per_pixel = 32;
	} else {
		info->bits_per_pixel = 16;
	}

	{
		struct timing_mcu *timing = panel->info.mcu->timing;
		info->write_register_timing = lcdc_calculate_lcm_timing(timing);
		timing++;
		info->write_gram_timing = lcdc_calculate_lcm_timing(timing);  
	}

	FB_PRINT("[%s] bits_per_pixel:%d;write_register_timing:0x%x;write_register_timing:0x%x\n",
		 __FUNCTION__, info->bits_per_pixel,info->write_register_timing,info->write_gram_timing);
	return 0;
}

static int setup_fbmem(struct sc8810fb_info *info, struct platform_device *pdev)
{
	uint32_t len, addr;
	len = info->panel->width * info->panel->height * (info->bits_per_pixel/8) * 2;

	addr = __get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len));
	if (!addr)
		return -ENOMEM;

	FB_PRINT("sc8810fb got %d bytes mem at 0x%x\n", len, addr);
	info->fb->fix.smem_start = __pa(addr);
	info->fb->fix.smem_len = len;
	info->fb->screen_base = (char*)addr;
       
	FB_PRINT("sc8810fb->fb->fix.smem_start=0x%x\n",(uint32_t)info->fb->fix.smem_start);

	return 0;
}

static int sc8810fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	if ((var->xres != fb->var.xres) ||
		(var->yres != fb->var.yres) ||
		(var->xres_virtual != fb->var.xres_virtual) ||
		(var->yres_virtual != fb->var.yres_virtual) ||
		(var->xoffset != fb->var.xoffset) ||
		(var->bits_per_pixel != fb->var.bits_per_pixel) ||
		(var->grayscale != fb->var.grayscale))
			return -EINVAL;
	return 0;
}

static void real_set_layer(void *data)
{
	struct fb_info *fb = (struct fb_info *)data;

	uint32_t reg_val = (fb->var.yoffset == 0)?fb->fix.smem_start:
		              (fb->fix.smem_start + fb->fix.smem_len/2);
#ifdef LCD_UPDATE_PARTLY
	if (fb->var.reserved[0] == 0x6f766572) {
		uint32_t x,y;	
		x = fb->var.reserved[1] & 0xffff;
		y = fb->var.reserved[1] >> 16;

		reg_val += ((x + y * fb->var.xres) * fb->var.bits_per_pixel / 8);
	}
#endif		
	__raw_writel(reg_val, LCDC_OSD1_BASE_ADDR);
}

static void real_refresh(void *para)
{
	struct sc8810fb_info *info = (struct sc8810fb_info *)para;

#ifdef LCD_UPDATE_PARTLY
	struct fb_info *fb = info->fb;	
	if (fb->var.reserved[0] == 0x6f766572) {	
		uint16_t left,top,width,height;

		__raw_writel(fb->var.reserved[2], LCDC_LCM_SIZE);

		__raw_writel(fb->var.reserved[2], LCDC_OSD1_SIZE_XY);

		__raw_writel(fb->var.reserved[2], LCDC_DISP_SIZE);
		
		left   = fb->var.reserved[1] &  0xffff;
		top    = fb->var.reserved[1] >> 16;
		width  = fb->var.reserved[2] &  0xffff;
		height = fb->var.reserved[2] >> 16;

		info->panel->ops->lcd_invalidate_rect(info->panel,
					left, top, left+width-1, top+height-1);
	} else 
#endif	
	{

		info->panel->ops->lcd_invalidate(info->panel);

	}
	lcdc_update_lcm_timing(info->write_gram_timing);
	__raw_bits_or((1<<3), LCDC_CTRL); /* start refresh */

}

static int real_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct sc8810fb_info *info = fb->par;

	if (info->fb_state != FB_NORMAL) {
		printk(KERN_ERR " sc8810fb can not do pan_display!!\n");
		return 0;
	}

	if (rrm_refresh(LID_OSD1, NULL, fb) != 0) {
		printk(KERN_ERR " sc8810fb refresh time out \n");
		if (__raw_readl(LCDC_CTRL) == 0) {
			printk(KERN_ERR " sc8810fb resume from time out \n");
			if (info->clk_lcdc->usecount == 0) {// ref count 
				clk_enable(info->clk_lcdc);
			}
			info->need_reinit = 1;
			lcdc_reset();
			hw_early_init(info);
			hw_init(info);
			hw_later_init(info);
			info->need_reinit = 0;
		}
	}

	FB_PRINT("@fool2[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));
	FB_PRINT("@fool2[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_SIZE));
	FB_PRINT("@fool2[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_START));
	FB_PRINT("@fool2[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_SIZE));
	FB_PRINT("@fool2[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_BG_COLOR));
	
	FB_PRINT("@fool2[%s] LCDC_OSD1_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_CTRL));
	FB_PRINT("@fool2[%s] LCDC_OSD1_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_BASE_ADDR));
	FB_PRINT("@fool2[%s] LCDC_OSD1_ALPHA_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_ALPHA_BASE_ADDR));
	FB_PRINT("@fool2[%s] LCDC_OSD1_SIZE_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_SIZE_XY));
	FB_PRINT("@fool2[%s] LCDC_OSD1_PITCH: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_PITCH));
	FB_PRINT("@fool2[%s] LCDC_OSD1_DISP_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_DISP_XY));
	FB_PRINT("@fool2[%s] LCDC_OSD1_ALPHA: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_ALPHA));
	FB_PRINT("@fool2[%s] LCDC_OSD1_GREY_RGB: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_GREY_RGB));
	FB_PRINT("@fool2[%s] LCDC_OSD1_CK: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_CK));

	FB_PRINT("@fool2[%s] LCM_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCM_CTRL));
	FB_PRINT("@fool2[%s] LCM_PARAMETER0: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER0));

	return 0;
}

static struct fb_ops sc8810fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = sc8810fb_check_var,
	.fb_pan_display = real_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static unsigned PP[16];
static void setup_fb_info(struct sc8810fb_info *info)
{
	struct fb_info *fb = info->fb;
	struct lcd_spec *panel = info->panel;
	int r;

	fb->fbops = &sc8810fb_ops;
	fb->flags = FBINFO_DEFAULT;
	
	/* finish setting up the fb_info struct */
	strncpy(fb->fix.id, "sc8810fb", 16);
	fb->fix.ypanstep = 1;
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = panel->width * info->bits_per_pixel / 8;

#ifdef LCD_UPDATE_PARTLY
	if (panel->ops->lcd_invalidate_rect != NULL) {
		fb->fix.reserved[0] = 0x6f76;
		fb->fix.reserved[1] = 0x6572;
	}
#endif

	fb->var.xres = panel->width;
	fb->var.yres = panel->height;
	fb->var.width = panel->width;
	fb->var.height = panel->height;
	fb->var.xres_virtual = panel->width;
	fb->var.yres_virtual = panel->height * 2;
	fb->var.bits_per_pixel = info->bits_per_pixel;
	fb->var.pixclock = 45000; // fake pixel clock to avoid divide 0
	fb->var.accel_flags = 0;

	fb->var.yoffset = 0;

	// only support two pixel format
	if (info->bits_per_pixel == 32) {
		fb->var.red.offset     = 24;
		fb->var.red.length     = 8;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 16;
		fb->var.green.length   = 8;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 8;
		fb->var.blue.length    = 8;
		fb->var.blue.msb_right = 0;
	} else {
		fb->var.red.offset     = 11;
		fb->var.red.length     = 5;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 5;
		fb->var.green.length   = 6;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 0;
		fb->var.blue.length    = 5;
		fb->var.blue.msb_right = 0;
	}
	r = fb_alloc_cmap(&fb->cmap, 16, 0);
	fb->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}

static void fb_free_resources(struct sc8810fb_info *info)
{

	if (info == NULL)
		return;

	if (info->panel != NULL && info->panel->ops->lcd_close != NULL) {
		info->panel->ops->lcd_close(info->panel);
	}

	__raw_writel(0, LCDC_CTRL); // disable LCDC

	rrm_exit();

	if (&info->fb->cmap != NULL) {
		fb_dealloc_cmap(&info->fb->cmap);
	}
	if (info->fb->screen_base) {
		free_pages ((unsigned long)info->fb->screen_base, 
				get_order(info->fb->fix.smem_len));		
	}		
	unregister_framebuffer(info->fb);
	framebuffer_release(info->fb);
}

static void lcdc_lcm_configure(struct lcd_spec *panel)
{
	uint32_t reg_val = 0;

	/* CS0 bus mode [BIT0]: 8080/6800 */
	switch (panel->info.mcu->bus_mode) {
	case LCD_BUS_8080:

		break;
	case LCD_BUS_6800:
		reg_val  |= 1;			
		break;
	default:
		break;
	}
	/* CS0 bus width [BIT1:0] */
	switch (panel->info.mcu->bus_width) {
	case 8:
		break;
	case 9:
		reg_val  |= ((1 << 1) | (1 << 4));
		break;
	case 16:
		reg_val  |= (2 << 1);
		break;
	case 18:
		reg_val  |= ((3 << 1) | (1 << 4));
		break;
	case 24:
		reg_val  |= ((4 << 1) | (2 << 4));
		break;
	default:
		break;

	}
	__raw_writel(reg_val, LCM_CTRL);
	
	FB_PRINT("@fool2[%s] LCM_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCM_CTRL));
}

static uint32_t lcdc_calculate_lcm_timing(struct timing_mcu *timing)
{
	uint32_t  ahb_clk;   
	uint32_t  rcss, rlpw, rhpw, wcss, wlpw, whpw;
	
	struct clk * clk = clk_get(NULL,"clk_ahb");
	ahb_clk = clk_get_rate(clk) / 1000000; 
	
	FB_PRINT("[%s] ahb_clk: 0x%x\n", __FUNCTION__, ahb_clk);

	/************************************************
	* we assume : t = ? ns, AHB = ? MHz   so
        *      1ns  cycle  :  AHB /1000 
	*      tns  cycles :  t * AHB / 1000
	*
	*****************************************/ 

	rcss = (timing->rcss * ahb_clk + 1000 - 1) / 1000; //ceiling
	if (rcss > MAX_LCDC_TIMING_VALUE) {
		rcss = MAX_LCDC_TIMING_VALUE ; // max 15 cycles
	}

	rlpw = (timing->rlpw * ahb_clk + 1000 - 1) / 1000;
	if (rlpw > MAX_LCDC_TIMING_VALUE) {
		rlpw = MAX_LCDC_TIMING_VALUE ; 
	}

	rhpw = (timing->rhpw * ahb_clk + 1000 - 1) / 1000; 
	if (rhpw > MAX_LCDC_TIMING_VALUE) {
		rhpw = MAX_LCDC_TIMING_VALUE ; 
	}

	wcss = (timing->wcss * ahb_clk + 1000 - 1) / 1000; 
	if (wcss > MAX_LCDC_TIMING_VALUE) {
		wcss = MAX_LCDC_TIMING_VALUE ; 
	}

	wlpw = (timing->wlpw * ahb_clk + 1000 - 1) / 1000; 
	if (wlpw > MAX_LCDC_TIMING_VALUE) {
		wlpw = MAX_LCDC_TIMING_VALUE ; 
	}

	whpw = (timing->whpw * ahb_clk + 1000 - 1) / 1000 - 1; 
	if (whpw > MAX_LCDC_TIMING_VALUE) {
		whpw = MAX_LCDC_TIMING_VALUE ; 
	}

	return (whpw | (wlpw << 4) | (wcss << 8)
					| (rhpw << 16) |(rlpw << 20) | (rcss << 24));

}

static void lcdc_update_lcm_timing(uint32_t reg_value)
{
	__raw_writel(reg_value,LCM_PARAMETER0); /* FIXME: hardcoded for !CS0 */

	FB_PRINT("[%s] LCM_PARAMETER0: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER0));
}

static inline int set_lcdsize( struct fb_info *info)
{
	uint32_t reg_val;
	
	reg_val = ( info->var.xres & 0xfff) | (( info->var.yres & 0xfff ) << 16);
	__raw_writel(reg_val, LCDC_DISP_SIZE);
	
	FB_PRINT("[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_SIZE));
	return 0;
}

static inline int set_lcmrect( struct fb_info *fb)
{
	uint32_t reg_val;
	
	__raw_writel(0, LCDC_LCM_START);
	
	reg_val = ( fb->var.xres & 0xfff) | (( fb->var.yres & 0xfff ) << 16);
	__raw_writel(reg_val, LCDC_LCM_SIZE);
	
	FB_PRINT("[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_START));
	FB_PRINT("[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_SIZE));

	return 0;
}

static int set_lcdc_layers(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	uint32_t reg_val = 0;

	/******************* OSD1 layer setting **********************/
	/* we assume that
	 * 1. there's only one fbdev, and only block0 is used
	 * 2. the pan operation is a sync one
	 */
	__raw_bits_and(~(1<<0),LCDC_IMG_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD2_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD3_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD4_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD5_CTRL);  
	/*enable OSD1 layer*/
	//__raw_bits_or((1<<0),LCDC_OSD1_CTRL);  
	reg_val |= (1 << 0);	

	/*color key */
	//__raw_bits_and(~(1<<1),LCDC_OSD1_CTRL);  //disable

	/*alpha mode select*/
	//__raw_bits_or((1<<2),LCDC_OSD1_CTRL);  //block alpha
	reg_val |= (1 << 2);		

	/*data format*/
	if (var->bits_per_pixel == 32) {
		reg_val |= (3 << 3); // ABGR
		/*rb switch*/	   
	 	reg_val |= (1 << 9); 	
	} else {
		reg_val |= (5 << 3); //RGB565
		reg_val |= (2 << 7); //B2B3B0B1
	}

	/*data endian*/
	//__raw_bits_or(1<<8,LCDC_OSD1_CTRL);  //little endian
	//__raw_bits_and(~(1<<7),LCDC_OSD1_CTRL);  
	//reg_val |= (1 << 7); //Big endian	   

	__raw_writel(reg_val, LCDC_OSD1_CTRL);

	FB_PRINT("@fool2[%s] LCDC_OSD1_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_CTRL));

	/* OSD1 layer base */
	//reg_val = (info->var.yoffset)?info->fix.smem_start: (info->fix.smem_start+ info->fix.smem_len/2);
	//__raw_writel(reg_val, LCDC_OSD1_BASE_ADDR);

	//FB_PRINT("@fool2[%s] LCDC_OSD1_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_BASE_ADDR));

	/*OSD1 layer alpha value*/
	__raw_writel(0xff, LCDC_OSD1_ALPHA);

	FB_PRINT("[%s] LCDC_OSD1_ALPHA: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_ALPHA));

	/*alpha base addr*/
	//__raw_writel(reg_val, LCDC_OSD1_ALPHA_BASE_ADDR); 

	/*OSD1 layer size*/
	reg_val = ( fb->var.xres & 0xfff) | (( fb->var.yres & 0xfff ) << 16);
	__raw_writel(reg_val, LCDC_OSD1_SIZE_XY);

	FB_PRINT("@[%s] LCDC_OSD1_SIZE_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_SIZE_XY));

	/*OSD1 layer start position*/
	__raw_writel(0, LCDC_OSD1_DISP_XY);

	FB_PRINT("@[%s] LCDC_OSD1_DISP_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_DISP_XY));

	/*OSD1 layer pitch*/
	reg_val = ( fb->var.xres & 0xfff) ;
	__raw_writel(reg_val, LCDC_OSD1_PITCH);

	FB_PRINT("@[%s] LCDC_OSD1_PITCH: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_PITCH));

	/*OSD1 color_key value*/
	//Fix me
	/*OSD1 grey RGB*/
	//Fix me

	/*LCDC workplane size*/
	set_lcdsize(fb);
	
	/*LCDC LCM rect size*/
	set_lcmrect(fb);

	return 0;
}

static void lcdc_reset(void)
{
	//LCD soft reset
	__raw_bits_or(1<<3, AHB_SOFT_RST);
	udelay(10);	
	__raw_bits_and(~(1<<3), AHB_SOFT_RST); 
}

static void hw_early_init(struct sc8810fb_info *info)
{
	//int ret;
	__raw_bits_and(~(1<<0), LCDC_IRQ_EN);
	__raw_bits_or((1<<0), LCDC_IRQ_CLR);

	/* init lcdc mcu mode using default configuration */
	lcdc_mcu_init();
}

static void hw_init(struct sc8810fb_info *info)
{
	/* only MCU mode is supported currently */
	if (LCD_MODE_RGB == info->panel->mode)
		return;

	//panel reset
	if (info->need_reinit) {
	    panel_reset(NULL);
	}
	
	/* set lcdc-lcd interface parameters */
	lcdc_lcm_configure(info->panel);

	/* set timing parameters for LCD */
	lcdc_update_lcm_timing(info->write_register_timing);
}

static void hw_later_init(struct sc8810fb_info *info)
{
	/* init mounted lcd panel */
	if(info->need_reinit) {
	    info->panel->ops->lcd_init(info->panel);
	}

	set_lcdc_layers(&info->fb->var, info->fb); 
	__raw_bits_or((1<<0), LCDC_IRQ_EN);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sc8810fb_early_suspend (struct early_suspend* es)
{
	struct sc8810fb_info *info = container_of(es, struct sc8810fb_info, early_suspend);

	info->fb_state = FB_NO_REFRESH;
	rrm_wait_for_idle();
	if(info->panel->ops->lcd_enter_sleep != NULL){
		info->panel->ops->lcd_enter_sleep(info->panel,1);
	}
	if (info->clk_lcdc) {
		FB_PRINT("clk_disable\n");
		clk_disable(info->clk_lcdc);
	}
	FB_PRINT("lcdc: [%s]\n", __FUNCTION__);
}

static void sc8810fb_late_resume (struct early_suspend* es)
{
	struct sc8810fb_info *info = container_of(es, struct sc8810fb_info, early_suspend);

	if (info->clk_lcdc) {
		FB_PRINT("clk_enable\n");
		clk_enable(info->clk_lcdc);
	}
	if (__raw_readl(LCDC_CTRL) == 0) {
		FB_PRINT("sc8810fb resume from deep sleep \n");
		info->need_reinit = 1;
		lcdc_reset();
		hw_early_init(info);
		hw_init(info);
		hw_later_init(info);
		info->need_reinit = 0;
	} else {	
		FB_PRINT("sc8810fb resume from sleep \n");
		if(info->panel->ops->lcd_enter_sleep != NULL) {
			info->panel->ops->lcd_enter_sleep(info->panel,0);
		}
	}
	info->fb_state = FB_NORMAL;
	FB_PRINT("lcdc: [%s]\n", __FUNCTION__);
}
#endif

#if CONFIG_CPU_FREQ
/*
*  @nb, structure "notifier_ block" defined in your drivers
*  @val,CPUFREQ_PRECHANGE or CPUFREQ_POSTCHANGE
*  @data, pointer which point to  strcuture "cpufreq_freqs"
*/
static int
lcdfb_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *f = data;
	struct sc8810fb_info *info = container_of(f, struct sc8810fb_info, freq_transition);

	switch(val){
	case CPUFREQ_PRECHANGE:
		info->fb_state = FB_NO_REFRESH;
		rrm_wait_for_idle();
		FB_PRINT("lcdfb cpufreq notify: CPUFREQ_PRECHANGE\n");
	        FB_PRINT("lcdfb cpufreq notify: old_freq:%u, new_freq:%u\n", f->old, f->new);   
		break;
	case CPUFREQ_POSTCHANGE:
		info->fb_state = FB_NORMAL;
		FB_PRINT("lcdfb cpufreq notify: CPUFREQ_POSTCHANGE\n");
		FB_PRINT("lcdfb cpufreq notify: old_freq:%u, new_freq:%u\n", f->old, f->new);   
		break;
	}
	return 0;
}

/*
*  @nb, structure "notifier_ block" defined in your drivers
*  @val,CPUFREQ_ADJUST, CPUFREQ_INCOMPATIBLE, CPUFREQ_NOTIFY
*  @data, pointer which point to  strcuture "cpufreq_policy"
*/
static int
lcdfb_freq_policy(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
		FB_PRINT("lcdfb cpufreq nofity: CPUFREQ_ADJUST\n");
		break;
	case CPUFREQ_INCOMPATIBLE:
		FB_PRINT("lcdfb cpufreq nofity: CPUFREQ_INCOMPATIBLE\n");
		break;
	case CPUFREQ_NOTIFY:
		FB_PRINT("lcdfb cpufreq nofity: CPUFREQ_NOTIFY\n");
		break;
	}
	return 0;
}
#endif


static uint32_t lcd_id_from_uboot = 0;

static int __init calibration_start(char *str)
{
	if ((str != NULL) && (str[0] == 'I') && (str[1] == 'D')) {
		sscanf(&str[2], "%x", &lcd_id_from_uboot);
	}
	FB_PRINT("sc8810fb lcd_id_from_uboot 0x%x\n", lcd_id_from_uboot);
	return 1;
}
__setup("lcd_id=", calibration_start);

static int32_t find_adapt_from_uboot(struct  sprd_lcd_platform_data* platform_data)
{
	int32_t i;
	struct lcd_panel_cfg  *lcd_panel = platform_data->lcd_panel_ptr;
	uint32_t lcd_panel_size = platform_data->lcd_panel_size;

	if(lcd_id_from_uboot != 0) {
		for(i = 0;i < lcd_panel_size; i++) {
			if(lcd_id_from_uboot == lcd_panel[i].lcd_id) {
				return i;
			}
		}
	}
	printk(KERN_ERR " sc8810fb can not get lcd id from uboot!!\n");
	return -1;		
}

static uint32_t lcd_readid_default(struct lcd_spec *self)
{
	uint32_t id = 0;
	//default id reg is 0
	self->info.mcu->ops->send_cmd(0x0);

	if(self->info.mcu->bus_width == 8) {
		id = (self->info.mcu->ops->read_data())&0xff;
		id <<= 8;
		id |= (self->info.mcu->ops->read_data())&0xff;
	} else {
		id = self->info.mcu->ops->read_data();
	}

	return id;
}

static int32_t find_adapt_from_readid(struct sc8810fb_info *info,struct  sprd_lcd_platform_data*platform_data)
{
	int32_t i;
	uint32_t id;
	struct lcd_panel_cfg  *lcd_panel = platform_data->lcd_panel_ptr;
	uint32_t lcd_panel_size = platform_data->lcd_panel_size;
	for(i = 0; i < lcd_panel_size; i++) {
		//first ,try mount
		mount_panel(info,lcd_panel[i].panel);
		//hw init to every panel
		hw_init(info);
		//readid
		if(info->panel->ops->lcd_readid) {
			id = info->panel->ops->lcd_readid(info->panel);
		} else {
			id = lcd_readid_default(info->panel);
		}
		//if the id is right?
		if(id == lcd_panel[i].lcd_id) {
			FB_PRINT ("sc8810fb : this lcd id is %d\n", id);
			return i;
		}
	}
	printk(KERN_ERR "sc8810fb may be wrong , can't get lcd id %x!!\n", id);
	return -1;		
}

static int sc8810fb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct sc8810fb_info *info;
	int32_t ret;
	int32_t lcd_adapt = 0;
	struct sprd_lcd_platform_data* platform_data = pdev->dev.platform_data;
	struct lcd_panel_cfg* lcd_panel = platform_data->lcd_panel_ptr;

	FB_PRINT("sc8810fb initialize!\n");

	lm_init(4); /* TEMP */
	
	fb = framebuffer_alloc(sizeof(struct sc8810fb_info), &pdev->dev);
	if (!fb) {
		ret = -ENOMEM;
		goto err0;
	}

	info = fb->par;
	info->fb = fb;
	info->fb_state = FB_NORMAL;
	info->ops = &lcm_mcu_ops;
	info->rrm = rrm_init(real_refresh, (void*)info);
	rrm_layer_init(LID_OSD1, 2, real_set_layer);

	info->clk_lcdc = lcdc_init_mclk(LCDC_96MHz);
	clk_enable(info->clk_lcdc); 

	/* register isr */
	ret = request_irq(IRQ_LCDC_INT, lcdc_isr, IRQF_DISABLED, "LCDC", info);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		goto cleanup;
	}
	//we maybe readid ,so hardware should be init
	
	hw_early_init(info);

	lcd_adapt = find_adapt_from_uboot(platform_data);
	if (lcd_adapt == -1) {
		info->need_reinit = 1;
		lcd_adapt = find_adapt_from_readid(info,platform_data);
	}
	if (lcd_adapt < 0) { // invalid index
		printk(KERN_ERR " can not read device id, and we will not refresh!\n");
		info->fb_state = FB_NO_REFRESH;
		lcd_adapt = 0;
	}

	mount_panel(info, lcd_panel[lcd_adapt].panel);

	ret = setup_fbmem(info, pdev);
	if (ret) {
		goto cleanup;
	}

	setup_fb_info(info);

	ret = register_framebuffer(fb);
	if (ret) {
		goto cleanup;
	}

	if (info->fb_state == FB_NORMAL) {
		hw_init(info);
		hw_later_init(info);
	}

	// after init hardware
	info->need_reinit = 0;
	
	platform_set_drvdata(pdev, info);

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.suspend = sc8810fb_early_suspend;
	info->early_suspend.resume  = sc8810fb_late_resume;
	info->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&info->early_suspend);
#endif

#if CONFIG_CPU_FREQ
	info->freq_transition.notifier_call = lcdfb_freq_transition;
	cpufreq_register_notifier(&info->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);

	info->freq_policy.notifier_call = lcdfb_freq_policy;
	cpufreq_register_notifier(&info->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	return 0;

cleanup:
	fb_free_resources(info);
err0:
	dev_err(&pdev->dev, "failed to probe sc8810fb\n");
	return ret;
}

#ifndef CONFIG_HAS_EARLYSUSPEND
static int sc8810fb_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct sc8810fb_info *info = platform_get_drvdata(pdev);
	if (info->panel->ops->lcd_enter_sleep != NULL) {
		info->panel->ops->lcd_enter_sleep(info->panel,1);
	}
	if (info->clk_lcdc) {
		FB_PRINT("clk_disable(info->clk_lcdc)\n");
		clk_disable(info->clk_lcdc);
	}
	FB_PRINT("deep sleep: [%s]\n", __FUNCTION__);
	return 0;
}

static int sc8810fb_resume(struct platform_device *pdev)
{
	struct sc8810fb_info *info = platform_get_drvdata(pdev);

	if (info->clk_lcdc) {
		FB_PRINT("clk_enable(info->clk_lcdc)\n");
		clk_enable(info->clk_lcdc);
	}

	if (__raw_readl(LCDC_CTRL) == 0) { // resume from deep sleep
		lcdc_reset();
		hw_early_init(info);
		hw_init(info);
		hw_later_init(info);
	} 

	if(info->panel->ops->lcd_enter_sleep != NULL){
		info->panel->ops->lcd_enter_sleep(info->panel,0);
	}

	FB_PRINT("deep sleep: [%s]\n", __FUNCTION__);
	return 0;
}
#endif

static struct platform_driver sc8810fb_driver = {
	.probe = sc8810fb_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = sc8810fb_suspend,
	.resume = sc8810fb_resume,
#endif
	.driver = {
		.name = "sc8810fb",
		.owner = THIS_MODULE,
	},
};

static int __init sc8810fb_init(void)
{
	FB_PRINT("sc8810fb [%s]\n", __FUNCTION__);
	return platform_driver_register(&sc8810fb_driver);
}

module_init(sc8810fb_init);


