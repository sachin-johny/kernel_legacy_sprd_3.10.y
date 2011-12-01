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

#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/mfp.h>
#include <linux/gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#include "lcdc_reg.h"
#include "lcd.h"
#include "fb_rrm.h"
#include "lcdc_manager.h" /* TEMP */

//#define  FB_DEBUG 
#ifdef FB_DEBUG
#define FB_PRINT printk
#else
#define FB_PRINT(...)
#endif

/* TEMP, software make-up for lcdc's 4-byte-align only limitation */
unsigned int fb_len;
unsigned int fb_pa;
unsigned int fb_va;
unsigned int fb_va_cached;
/* TEMP, end */

struct sc8810fb_info {
	struct fb_info   *fb;
	struct ops_mcu   *ops;
	struct lcd_spec  *panel;
	struct rrmanager *rrm;
	uint32_t cap;
	uint32_t bits_per_pixel;
	uint32_t need_reinit;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_MACH_SC8810OPENPHONE

extern struct lcd_spec lcd_panel_hx8357;
static struct lcd_cfg lcd_panel[] = {
	[0]={
		.lcd_id = 0x57,
		.panel = &lcd_panel_hx8357,
		},
};

#else

extern struct lcd_spec lcd_panel_hx8369;
static struct lcd_cfg lcd_panel[] = {
	[0]={
		.lcd_id = 0x69,
		.panel = &lcd_panel_hx8369,
		},
};


#endif


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

static int32_t lcm_read_data (void)
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

extern struct lcdc_manager lm; /* TEMP */
//extern struct semaphore copybit_wait; /* TEMP */
static irqreturn_t lcdc_isr(int irq, void *data)
{
	uint32_t val ;
	struct sc8810fb_info *fb = (struct sc8810fb_info *)data;

	val = __raw_readl(LCDC_IRQ_STATUS);
	if (val & (1<<0)){      /* lcdc done */
		FB_PRINT("--> lcdc_isr lm.mode=%d\n", lm.mode);
		__raw_bits_or((1<<0), LCDC_IRQ_CLR);
		if(lm.mode == LMODE_DISPLAY) /* TEMP */
			rrm_interrupt(fb->rrm);
	}

	return IRQ_HANDLED;
}

static int32_t panel_reset(struct lcd_spec *self)
{
	//panel reset
	__raw_writel(0x1, LCM_RSTN);	
	mdelay(0x10);
	__raw_writel(0x0, LCM_RSTN);
	mdelay(0x10);
	__raw_writel(0x1, LCM_RSTN);
	mdelay(0x10);

	return 0;
}

static void lcdc_mcu_init(void)
{
	uint32_t reg_val = 0;
	//panel reset
	//panel_reset(NULL);
	
	//LCDC module enable
	reg_val |= (1<<0);


	/*FMARK mode*/
	//reg_val | = (1<<1);

	/*FMARK pol*/
	__raw_writel(reg_val, LCDC_CTRL); 
	
	FB_PRINT("@fool2[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));
	
	/* set background*/
	__raw_writel(0x0, LCDC_BG_COLOR);   //red

	FB_PRINT("@fool2[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_BG_COLOR));

	/* dithering enable*/
	//__raw_bits_or(1<<4, LCDC_CTRL);   
	
}

static int mount_panel(struct sc8810fb_info *fb, struct lcd_spec *panel)
{
	/* TODO: check whether the mode/res are supported */
	uint32_t bus_width;
	fb->panel = panel;

	panel->info.mcu->ops = fb->ops;

	panel->ops->lcd_reset = panel_reset;

	bus_width = fb->panel->info.mcu->bus_width;
	if (bus_width == 9 || bus_width == 18 || bus_width == 24) {
		fb->bits_per_pixel = 32;
	} else {
		fb->bits_per_pixel = 16;
	}
	FB_PRINT("allen: fb->bits_per_pixel = %d;\n", fb->bits_per_pixel);
	return 0;
}

static int setup_fbmem(struct sc8810fb_info *info, struct platform_device *pdev)
{
	uint32_t len, addr;
#if 1
	len = info->panel->width * info->panel->height * (info->bits_per_pixel/8) * 2;

	/* the addr should be 8 byte align */
	addr = __get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len));
	if (!addr)
		return -ENOMEM;

	FB_PRINT("sc8810fb got %d bytes mem at 0x%x\n", len, addr);
	info->fb->fix.smem_start = __pa(addr);
	info->fb->fix.smem_len = len;
	info->fb->screen_base = (char*)addr;

	/* TEMP, software make-up for lcdc's 4-byte-align only limitation */
	fb_len = len;
	fb_pa = info->fb->fix.smem_start;
	fb_va_cached = (uint32_t)info->fb->screen_base;
	fb_va = (uint32_t)ioremap(info->fb->fix.smem_start, 
		info->fb->fix.smem_len);
       
	printk("sc8810fb->fb->fix.smem_start=0x%x\n",(uint32_t)info->fb->fix.smem_start);

	printk("sc8810fb fb_va=0x%x, size=%d\n", fb_va, info->fb->fix.smem_len);
	/* TEMP, end */
#else /*for 8810 fpga gpu test*/
        len = info->panel->width * info->panel->height * (info->bits_per_pixel/8) * 2;
                
        info->fb->fix.smem_start =  63*1024*1024;
        info->fb->fix.smem_len = len;

        fb_va = (uint32_t)ioremap(info->fb->fix.smem_start, 
        	info->fb->fix.smem_len);
		
        info->fb->screen_base = (char*)fb_va;
        
        /* TEMP, software make-up for lcdc's 4-byte-align only limitation */
        fb_len = len;
        fb_pa = info->fb->fix.smem_start;
        fb_va_cached = (uint32_t)info->fb->screen_base;
        fb_va = (uint32_t)ioremap(info->fb->fix.smem_start, 
        	info->fb->fix.smem_len);
        printk("sc8810_fb fb_va=0x%x, size=%d\n", fb_va, info->fb->fix.smem_len);
        /* TEMP, end */
#endif
	return 0;
}

static int sc8810fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if ((var->xres != info->var.xres) ||
	    (var->yres != info->var.yres) ||
	    (var->xres_virtual != info->var.xres_virtual) ||
	    (var->yres_virtual != info->var.yres_virtual) ||
	    (var->xoffset != info->var.xoffset) ||
	    (var->bits_per_pixel != info->var.bits_per_pixel) ||
	    (var->grayscale != info->var.grayscale))
		 return -EINVAL;
	return 0;
}

static void real_set_layer(void *data)
{
	struct fb_info *fb = (struct fb_info *)data;
	uint32_t reg_val;
	uint32_t x,y;

	/* image layer base */
	reg_val = (fb->var.yoffset == 0)?fb->fix.smem_start:
		              (fb->fix.smem_start + fb->fix.smem_len/2);

	//this is for further Optimization
	if(0 /* fb->var.reserved[0] == 0x6f766572*/) {	
		x = fb->var.reserved[1] & 0xffff;
		y = fb->var.reserved[1] >> 16;

		reg_val += ((x + y * fb->var.xres) * fb->var.bits_per_pixel / 8);
	}
		
	__raw_writel(reg_val, LCDC_OSD1_BASE_ADDR);
}

static void real_refresh(void *para)
{
	struct sc8810fb_info *info = (struct sc8810fb_info *)para;
	struct fb_info *fb = info->fb;

	if(0 /*fb->var.reserved[0] == 0x6f7665728*/) {	
		uint16_t left,top,width,height;
		//never use for more further Optimization
		__raw_writel(fb->var.reserved[2], LCDC_LCM_SIZE);

		//this is for further Optimization
		//never use for more further Optimization
		__raw_writel(fb->var.reserved[2], LCDC_OSD1_SIZE_XY);
		//this is for further Optimization

		//this is for more further Optimization
		__raw_writel(fb->var.reserved[2], LCDC_DISP_SIZE);
		//this is for more further Optimization
		
		left   = fb->var.reserved[1] &  0xffff;
		top    = fb->var.reserved[1] >> 16;
		width  = fb->var.reserved[2] &  0xffff;
		height = fb->var.reserved[2] >> 16;

		info->panel->ops->lcd_invalidate_rect(info->panel,left,top,left+width-1,top+height-1);
	} else {
		info->panel->ops->lcd_invalidate(info->panel);
	}

	__raw_bits_or((1<<3), LCDC_CTRL); /* start refresh */

}

static int real_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	rrm_refresh(LID_OSD1, NULL, info);

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
	FB_PRINT("@fool2[%s] LCM_PARAMETER1: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER1));
	//FB_PRINT("@fool2[%s] LCM_IFMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_IFMODE));
	//FB_PRINT("@fool2[%s] LCM_RGBMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_RGBMODE));
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

	fb->var.xres = panel->width;
	fb->var.yres = panel->height;
	fb->var.width = panel->width;
	fb->var.height = panel->height;
	fb->var.xres_virtual = panel->width;
	fb->var.yres_virtual = panel->height * 2;
	fb->var.bits_per_pixel = info->bits_per_pixel;
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

uint32_t CHIP_GetMcuClk (void)
{
	uint32_t clk_mcu_sel;
    	uint32_t clk_mcu = 0;
    	
	clk_mcu_sel = __raw_readl(AHB_ARM_CLK);
	clk_mcu_sel = (clk_mcu_sel >>  23)  &  0x3;
	switch (clk_mcu_sel)
	{
	case 0:
		clk_mcu = 400000000;
            	break;
	case 1:
		clk_mcu = 153600000;
		break;
	case 2:
		clk_mcu = 64000000;
            	break;
	case 3:
            	clk_mcu = 26000000;
            	break;
	default:
		// can't go there
		break;
	}       
	return clk_mcu;
}

static void lcdc_update_lcm_timing(struct sc8810fb_info *info)
{
	uint32_t  reg_value;
	uint32_t  ahb_div,ahb_clk;   
	uint32_t  ahb_clk_cycle;
	uint32_t  rcss, rlpw, rhpw, wcss, wlpw, whpw;
	struct timing_mcu *timing;

	reg_value = __raw_readl(AHB_ARM_CLK);
	
	ahb_div = ( reg_value>>4 ) & 0x7;
	
	ahb_div = ahb_div + 1;

	if(__raw_readl (AHB_ARM_CLK) & (1<<30))
    	{
		ahb_div = ahb_div << 1;
    	}
    	if(__raw_readl (AHB_ARM_CLK) & (1<<31))
    	{
        	ahb_div=ahb_div<<1;
    	}	
	//ahb_clk = CHIP_GetMcuClk()/ahb_div;
	
	// for sc8810 fpga
	ahb_clk = 32*1000*1000;
	
	FB_PRINT("[%s] ahb_clk: 0x%x\n", __FUNCTION__, ahb_clk);

	/* LCD_UpdateTiming() */
	ahb_clk_cycle = (100000000 >> 17)/(ahb_clk >> 20 );

	timing = info->panel->info.mcu->timing;
	rcss = ((timing->rcss/ahb_clk_cycle + 1) < 14) ? (timing->rcss/ahb_clk_cycle+1):14;
	rlpw = ((timing->rlpw/ahb_clk_cycle + 1) < 14) ? (timing->rlpw/ahb_clk_cycle+1):14;
	rhpw = ((timing->rhpw/ahb_clk_cycle + 1) < 14) ? (timing->rhpw/ahb_clk_cycle+1):14;
	wcss = ((timing->wcss/ahb_clk_cycle + 1) < 14) ? (timing->wcss/ahb_clk_cycle+1):14;
	wlpw = ((timing->wlpw/ahb_clk_cycle + 1) < 14) ? (timing->wlpw/ahb_clk_cycle+1):14;
	whpw = ((timing->whpw/ahb_clk_cycle + 1) < 14) ? (timing->whpw/ahb_clk_cycle+1):14;
	
	/*   LCDC_ChangePulseWidth() */
    	reg_value = whpw | (wlpw << 4) | (wcss << 8)
                        | (rhpw << 16) |(rlpw << 20) | (rcss << 24);
	__raw_writel(reg_value,LCM_PARAMETER0); /* FIXME: hardcoded for !CS0 */

	FB_PRINT("[%s] LCM_PARAMETER0: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER0));
}

static inline int set_lcdsize( struct fb_info *info)
{
	uint32_t reg_val;
	
	reg_val = ( info->var.xres & 0xfff) | (( info->var.yres & 0xfff )<<16);
	__raw_writel(reg_val, LCDC_DISP_SIZE);
	
	FB_PRINT("[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_SIZE));
	return 0;
}

static inline int set_lcmrect( struct fb_info *info)
{
	uint32_t reg_val;
	
	__raw_writel(0, LCDC_LCM_START);
	
	reg_val = ( info->var.xres & 0xfff) | (( info->var.yres & 0xfff )<<16);
	__raw_writel(reg_val, LCDC_LCM_SIZE);
	
	FB_PRINT("[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_START));
	FB_PRINT("[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_SIZE));

	return 0;
}

int set_lcdc_layers(struct fb_var_screeninfo *var, struct fb_info *info)
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
	reg_val = ( info->var.xres & 0xfff) | (( info->var.yres & 0xfff )<<16);
	__raw_writel(reg_val, LCDC_OSD1_SIZE_XY);

	FB_PRINT("@[%s] LCDC_OSD1_SIZE_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_SIZE_XY));

	/*OSD1 layer start position*/
	__raw_writel(0, LCDC_OSD1_DISP_XY);

	FB_PRINT("@[%s] LCDC_OSD1_DISP_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_DISP_XY));

	/*OSD1 layer pitch*/
	reg_val = ( info->var.xres & 0xfff) ;
	__raw_writel(reg_val, LCDC_OSD1_PITCH);

	FB_PRINT("@[%s] LCDC_OSD1_PITCH: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_OSD1_PITCH));

	/*OSD1 color_key value*/
	//Fix me
	/*OSD1 grey RGB*/
	//Fix me

	/*LCDC workplane size*/
	set_lcdsize(info);
	
	/*LCDC LCM rect size*/
	set_lcmrect(info);

	return 0;
}

static void hw_early_init(struct sc8810fb_info *info)
{
	int ret;

	//select LCD clock source	
	//__raw_bits_and(~(1<<6), GR_PLL_SRC);    //pll_src=96M
	//__raw_bits_and(~(1<<7), GR_PLL_SRC);

	//set LCD divdior
	//__raw_bits_and(~(1<<0), GR_GEN4);  //div=0
	//__raw_bits_and(~(1<<1), GR_GEN4); 
	//__raw_bits_and(~(1<<2), GR_GEN4);  

	//enable LCD clock
	//__raw_bits_or(1<<3, AHB_CTL0); 

	//LCD soft reset
	//__raw_bits_or(1<<3, AHB_SOFT_RST);
	//mdelay(10);	
	//__raw_bits_and(~(1<<3), AHB_SOFT_RST); 

	__raw_bits_and(~(1<<0), LCDC_IRQ_EN);
	__raw_bits_or((1<<0), LCDC_IRQ_CLR);

	/* register isr */
	ret = request_irq(IRQ_LCDC_INT, lcdc_isr, IRQF_DISABLED, "LCDC", info);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		return;
	}

	/* init lcdc mcu mode using default configuration */
	lcdc_mcu_init();
}

static void hw_init(struct sc8810fb_info *info)
{
	/* only MCU mode is supported currently */
	if (LCD_MODE_RGB == info->panel->mode)
		return;

	//panel reset
	if(info->need_reinit) {
	    panel_reset(NULL);
	}
	
	/* set lcdc-lcd interface parameters */
	lcdc_lcm_configure(info->panel);

	/* set timing parameters for LCD */
	lcdc_update_lcm_timing(info);

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

static int32_t set_backlight(uint32_t value)
{
#ifdef CONFIG_MACH_SC8810OPENPHONE
	FB_PRINT("sc8810fb : openphone\n");
	ANA_REG_AND(WHTLED_CTL, ~(WHTLED_PD_SET | WHTLED_PD_RST));
	ANA_REG_OR(WHTLED_CTL,  WHTLED_PD_RST);
	ANA_REG_MSK_OR (WHTLED_CTL, ( (value << WHTLED_V_SHIFT) &WHTLED_V_MSK), WHTLED_V_MSK);
#else
	if (gpio_request(143, "LCD_BL")) {
		printk(KERN_ERR "Failed ro request LCD_BL GPIO_%d \n",
			143);
		return -ENODEV;
	}
	gpio_direction_output(143, 1);
	gpio_set_value(143, 1);
#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sc8810fb_early_suspend (struct early_suspend* es)
{
	struct sc8810fb_info *info = container_of(es, struct sc8810fb_info, early_suspend);
	if(info->panel->ops->lcd_enter_sleep != NULL){
		info->panel->ops->lcd_enter_sleep(info->panel,1);
	}

}

static void sc8810fb_early_resume (struct early_suspend* es)
{
	struct sc8810fb_info *info = container_of(es, struct sc8810fb_info, early_suspend);
	if(info->panel->ops->lcd_enter_sleep != NULL){
		info->panel->ops->lcd_enter_sleep(info->panel,0);
	}
}
#endif

static uint32_t lcd_id_from_uboot = 0;

static int __init calibration_start(char *str)
{
	if((str[0]=='I')&&(str[1]=='D'))
	{
		sscanf(&str[2], "%x", &lcd_id_from_uboot);
	}
	else
	{
		lcd_id_from_uboot = 0;
	}
	FB_PRINT("sc8810fb lcd_id_from_uboot 0x%x\n", lcd_id_from_uboot);
	return 1;
}
__setup("lcd_id=", calibration_start);

static int32_t find_adapt_from_uboot(void)
{
	int32_t i;
	if(lcd_id_from_uboot != 0)
	{
		for(i = 0;i<(sizeof(lcd_panel))/(sizeof(lcd_panel[0]));i++)
		{
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

static int32_t find_adapt_from_readid(struct sc8810fb_info *info)
{
	int32_t i;
	uint32_t id;
	for(i = 0;i<(sizeof(lcd_panel))/(sizeof(lcd_panel[0]));i++) {
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
	uint32_t lcd_adapt = 0;

	FB_PRINT("sc8810fb initialize!\n");

	lm_init(4); /* TEMP */
	
	fb = framebuffer_alloc(sizeof(struct sc8810fb_info), &pdev->dev);
	if (!fb)
		return -ENOMEM;
	info = fb->par;
	info->fb = fb;
	info->ops = &lcm_mcu_ops;
	info->rrm = rrm_init(real_refresh, (void*)info);
	rrm_layer_init(LID_OSD1, 2, real_set_layer);

	//we maybe readid ,so hardware should be init
	info->need_reinit = 1;
	hw_early_init(info);


	lcd_adapt = find_adapt_from_uboot();
	if(lcd_adapt == -1)
	{
		lcd_adapt = find_adapt_from_readid(info);
	}
	else
	{
		info->need_reinit = 0;
	}

	ret = mount_panel(info, lcd_panel[lcd_adapt].panel);
	if (ret) {
		printk(KERN_ERR "unsupported panel!!\n");
		return -EFAULT;
	}

	ret = setup_fbmem(info, pdev);
	if (ret)
		return ret;

	setup_fb_info(info);

	ret = register_framebuffer(fb);
	if (ret) {
		framebuffer_release(fb);
		return ret;
	}

	hw_init(info);
	hw_later_init(info);

	/* FIXME: put the BL stuff to where it belongs. */
	//set_backlight(50);
	platform_set_drvdata(pdev, info);

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.suspend = sc8810fb_early_suspend;
	info->early_suspend.resume  = sc8810fb_early_resume;
	info->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&info->early_suspend);
#endif
	return 0;
}

static int sc8810fb_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct sc8810fb_info *info=platform_get_drvdata(pdev);
	info->panel->ops->lcd_enter_sleep(info->panel,1);
	return 0;
}

static int sc8810fb_resume(struct platform_device *pdev)
{
	struct sc8810fb_info *info=platform_get_drvdata(pdev);
	info->panel->ops->lcd_enter_sleep(info->panel,0);
	return 0;
}

static struct platform_driver sc8810fb_driver = {
	.probe = sc8810fb_probe,
	.suspend = sc8810fb_suspend,
	.resume = sc8810fb_resume,
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


