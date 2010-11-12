/* drivers/video/sc8800s/sc8800s_fb.c
 *
 * SC8800S LCM0 framebuffer driver.
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
#include "sc8800g_lcd.h"

//#define  FB_DEBUG 
#ifdef FB_DEBUG
#define FB_PRINT printk
#else
#define FB_PRINT(...)
#endif

#define AHB_CTL0                (SPRD_AHB_BASE + 0x200)
#define AHB_SOFT_RST            (SPRD_AHB_BASE + 0x210)
#define AHB_ARM_CLK         (SPRD_AHB_BASE + 0x224)

#define GR_PLL_SRC              (SPRD_GREG_BASE + 0x70)
#define GR_GEN4                 (SPRD_GREG_BASE + 0x60)


#define LCDC_CTRL                   (SPRD_LCDC_BASE + 0x0000)
#define LCDC_DISP_SIZE              (SPRD_LCDC_BASE + 0x0004)
#define LCDC_LCM_START              (SPRD_LCDC_BASE + 0x0008)
#define LCDC_LCM_SIZE               (SPRD_LCDC_BASE + 0x000c)
#define LCDC_BG_COLOR               (SPRD_LCDC_BASE + 0x0010)
#define LCDC_FIFO_STATUS            (SPRD_LCDC_BASE + 0x0014)

#define LCDC_IMG_CTRL                    (SPRD_LCDC_BASE + 0x0020)
#define LCDC_IMG_Y_BASE_ADDR         (SPRD_LCDC_BASE + 0x0024)
#define LCDC_IMG_UV_BASE_ADDR            (SPRD_LCDC_BASE + 0x0028)
#define LCDC_IMG_SIZE_XY             (SPRD_LCDC_BASE + 0x002c)
#define LCDC_IMG_PITCH                   (SPRD_LCDC_BASE + 0x0030)
#define LCDC_IMG_DISP_XY             (SPRD_LCDC_BASE + 0x0034)

#define LCDC_OSD1_CTRL                    (SPRD_LCDC_BASE + 0x0040)
#define LCDC_OSD2_CTRL                    (SPRD_LCDC_BASE + 0x0070)
#define LCDC_OSD3_CTRL                    (SPRD_LCDC_BASE + 0x0090)
#define LCDC_OSD4_CTRL                    (SPRD_LCDC_BASE + 0x00b0)
#define LCDC_OSD5_CTRL                    (SPRD_LCDC_BASE + 0x00d0)

#define LCDC_IRQ_EN             (SPRD_LCDC_BASE + 0x0120)
#define LCDC_IRQ_CLR                (SPRD_LCDC_BASE + 0x0124)
#define LCDC_IRQ_STATUS         (SPRD_LCDC_BASE + 0x0128)
#define LCDC_IRQ_RAW                (SPRD_LCDC_BASE + 0x012c)

#define LCM_CTRL                (SPRD_LCDC_BASE + 0x140)
#define LCM_PARAMETER0                (SPRD_LCDC_BASE + 0x144)
#define LCM_PARAMETER1                (SPRD_LCDC_BASE + 0x148)
#define LCM_IFMODE                (SPRD_LCDC_BASE + 0x14c)
#define LCM_RGBMODE              (SPRD_LCDC_BASE + 0x150)
#define LCM_RDDATA               (SPRD_LCDC_BASE + 0x154)
#define LCM_STATUS               (SPRD_LCDC_BASE + 0x158)
#define LCM_RSTN                 (SPRD_LCDC_BASE + 0x15c)
#define LCM_CD0                  (SPRD_LCDC_BASE + 0x180)
#define LCM_DATA0                (SPRD_LCDC_BASE + 0x184)
#define LCM_CD1                  (SPRD_LCDC_BASE + 0x190)
#define LCM_DATA1                (SPRD_LCDC_BASE + 0x194)

#define ANA_AGEN	SPRD_ANA_BASE+0x00
#define AGEN_PINREG_EN	(1<<3)

#define WHTLED_CTL	SPRD_ANA_BASE +0x40
#define WHTLED_PD_SET  	(1<<0)
#define WHTLED_PD_RST  	(1<<1)
#define WHTLED_V_SHIFT  2
#define WHTLED_V_MSK  	(0x1F << WHTLED_V_SHIFT)

#define BITS_PER_PIXEL 16

struct sc8800fb_info {
	struct fb_info *fb;
	uint32_t cap;
	struct ops_mcu *ops;
	struct lcd_spec *panel;
};
static int32_t lcm_send_cmd (uint32_t cmd)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_STATUS) & 0x1);

	__raw_writel(cmd, LCM_CD0);

	return 0;
}

static int32_t lcm_send_cmd_data (uint32_t cmd, uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_STATUS) & 0x1);

	__raw_writel(cmd, LCM_CD0);

	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_STATUS) & 0x1);

	__raw_writel(data, LCM_DATA0);

	return 0;
}

static int32_t lcm_send_data (uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCM_STATUS) & 0x1);

	__raw_writel(data, LCM_DATA0);

	return 0;
}
static struct ops_mcu lcm_mcu_ops = {
	.send_cmd = lcm_send_cmd,
	.send_cmd_data = lcm_send_cmd_data,
	.send_data = lcm_send_data,
};
static irqreturn_t lcdc_isr(int irq, void *data)
{
	uint32_t val ;
	val = __raw_readl(LCDC_IRQ_STATUS);
	if (val & (1<<0))      /* lcdc done */
			__raw_bits_or((1<<0), LCDC_IRQ_CLR);

	return IRQ_HANDLED;
}
#if 0
/* 
 * there may be some LCD related pins need to be set... 
 * PM_Init/PM_GPIOInit
 */
 /* some temporary code for GPIO */
static void gpio_set_val(int32_t id, int32_t level)
{
	int32_t page, bit_num, gpio_pg_base;

	page = id >>4;
	bit_num = id&0xf;
	if (id >=176)
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE+0x100;
	else
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE;

	if(level)
		__raw_bits_or((1<<bit_num), gpio_pg_base);
	else
		__raw_bits_and(~(1<<bit_num), gpio_pg_base);
}

static void gpio_enable(int32_t id)
{
	int32_t page, bit_num, gpio_pg_base;

	page = id >>4;
	bit_num = id&0xf;
	if (id >=176)
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE+0x100;
	else
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE;

	__raw_bits_or((1<<bit_num), gpio_pg_base+4);
}

/* direction: 1-output, 0-input */
static void gpio_set_direction(int32_t id, uint32_t direction)
{
	int32_t page, bit_num, gpio_pg_base;

	page = id >>4;
	bit_num = id&0xf;
	if (id >=176)
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE+0x100;
	else
		gpio_pg_base = page*0x80 + SPRD_GPIO_BASE;

	FB_PRINT("@fool2[%s] gpio[%d] dir reg val.before: 0x%x\n", __FUNCTION__, id, __raw_readl(gpio_pg_base+8));

	if(direction)
		__raw_bits_or((1<<bit_num), gpio_pg_base+8);
	else
		__raw_bits_and(~(1<<bit_num), gpio_pg_base+8);

	FB_PRINT("@fool2[%s] gpio[%d] dir reg val.after: 0x%x\n", __FUNCTION__, id, __raw_readl(gpio_pg_base+8));
}
#endif
static void set_pins(void)
{	

	__raw_writel(0x1fff00, SPRD_CPC_BASE);
	/* pins connect to LCM/lcd */
	__raw_writel((1<<8), (SPRD_CPC_BASE+0x2AC)); //LCD_CSN1
	__raw_writel(((1<<8) |(1<<3)), (SPRD_CPC_BASE+0x2B0)); //LCD_RSTN
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2B4)); //LCD_CD
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2B8)); //LCD_D[0]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2BC)); //LCD_D[1]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2C0)); //LCD_D[2]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2C4)); //LCD_D[3]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2C8)); //LCD_D[4]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2CC)); //LCD_D[5]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2D0)); //LCD_D[6]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2D4)); //LCD_D[7]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2D8)); //LCD_D[8]
	__raw_writel(((1<<8) |(1<<3)), (SPRD_CPC_BASE+0x2DC)); //LCD_WRN
	__raw_writel(((1<<8) |(1<<3)), (SPRD_CPC_BASE+0x2E0)); //LCD_RDN
	__raw_writel(((1<<8) |(1<<3)), (SPRD_CPC_BASE+0x2E4)); //LCD_CSN0	
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2E8)); //LCD_D[9]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2EC)); //LCD_D[10]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2F0)); //LCD_D[11]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2F4)); //LCD_D[12]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2F8)); //LCD_D[13]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x2FC)); //LCD_D[14]
	__raw_writel(((1<<8) |(1<<2)), (SPRD_CPC_BASE+0x300)); //LCD_D[15]
	__raw_writel((1<<8), (SPRD_CPC_BASE+0x304)); //LCD_D[16]
	__raw_writel((1<<8), (SPRD_CPC_BASE+0x308)); //LCD_D[17]
	__raw_writel((1<<8), (SPRD_CPC_BASE+0x30C)); //LCD_FMARK

	
}
static void panel_reset(void)
{
	//panel reset
	__raw_writel(0x1, LCM_RSTN);	
	mdelay(0x10);
	__raw_writel(0x0, LCM_RSTN);
	mdelay(0x10);
	__raw_writel(0x1, LCM_RSTN);
	mdelay(0x10);
}
static void lcdc_mcu_init(void)
{
	//panel reset
	panel_reset();
	
	//LCDC module enable
	__raw_bits_or(1<<0, LCDC_CTRL);

	/*FMARK mode*/
	__raw_bits_or(1<<1, LCDC_CTRL);

	/*FMARK pol*/
	__raw_bits_and(~(1<<2), LCDC_CTRL);
	
	FB_PRINT("@fool2[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));
	
	/* set background*/
	__raw_writel((0xff<<16), LCDC_BG_COLOR);   //red

	FB_PRINT("@fool2[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_BG_COLOR));

	/* dithering enable*/
	//__raw_bits_or(1<<4, LCDC_CTRL);   
	
}
static int mount_panel(struct sc8800fb_info *sc8800fb, struct lcd_spec *panel)
{
	/* TODO: check whether the mode/res are supported */

	sc8800fb->panel = panel;

	panel->info.mcu->ops = sc8800fb->ops;

	panel->ops->lcd_reset = panel_reset;
	
	return 0;
}
static int setup_fbmem(struct sc8800fb_info *sc8800fb, struct platform_device *pdev)
{
	uint32_t len, addr;
	
	len = sc8800fb->panel->width * sc8800fb->panel->height * (BITS_PER_PIXEL/8) * 2;

	/* the addr should be 8 byte align */
	addr = __get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len));
	if (!addr)
		return -ENOMEM;

	printk("sc8800g_fb got %d bytes mem at 0x%x\n", len, addr);
	sc8800fb->fb->fix.smem_start = __pa(addr);
	sc8800fb->fb->fix.smem_len = len;
	sc8800fb->fb->screen_base = (char*)addr;

	return 0;
}
static int sc8800fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
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
int real_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct sc8800fb_info *sc8800fb = info->par;
	uint32_t reg_val;

	/* image layer base */
	reg_val = (info->var.yoffset == 0)?info->fix.smem_start:
		(info->fix.smem_start+ info->fix.smem_len/2);
	reg_val = (reg_val>>2) & 0x3fffffff;
	__raw_writel(reg_val, LCDC_IMG_Y_BASE_ADDR);

	/* set brush direction */
	/* FIXME: hardcoded direction */
	
	sc8800fb->panel->ops->lcd_invalidate(sc8800fb->panel);

	reg_val = info->var.xres * info->var.yres;
	reg_val |= (1<<20); /* for device 0 */
	reg_val &=~ ((1<<26)|(1<<27) | (1<<28)); /* FIXME: hardcoded cs 1 */
	__raw_writel(reg_val, LCM_CTRL);
	//FB_PRINT("@fool2[%s] LCM_CTRL: 0x%x, reg_val=0x%x\n", __FUNCTION__, __raw_readl(LCM_CTRL), reg_val);


	FB_PRINT("@fool2[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));
	FB_PRINT("@fool2[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_SIZE));
	FB_PRINT("@fool2[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_START));
	FB_PRINT("@fool2[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_SIZE));
	FB_PRINT("@fool2[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_BG_COLOR));
	FB_PRINT("@fool2[%s] LCDC_IMG_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_CTRL));
	FB_PRINT("@fool2[%s] LCDC_IMG_Y_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_Y_BASE_ADDR));
	FB_PRINT("@fool2[%s] LCDC_IMG_UV_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_UV_BASE_ADDR));
	FB_PRINT("@fool2[%s] LCDC_IMG_SIZE_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_SIZE_XY));
	FB_PRINT("@fool2[%s] LCDC_IMG_PITCH: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_PITCH));
	FB_PRINT("@fool2[%s] LCDC_IMG_DISP_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_DISP_XY));

	FB_PRINT("@fool2[%s] LCM_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCM_CTRL));
	FB_PRINT("@fool2[%s] LCM_PARAMETER0: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER0));
	FB_PRINT("@fool2[%s] LCM_PARAMETER1: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER1));
	FB_PRINT("@fool2[%s] LCM_IFMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_IFMODE));
	FB_PRINT("@fool2[%s] LCM_RGBMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_RGBMODE));

	__raw_bits_or((1<<3), LCDC_CTRL); /* start refresh */
	FB_PRINT("@fool2[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_CTRL));

	/* busy wait for image refresh Done */
	while((__raw_readl(LCDC_IRQ_RAW) & (1<<0)) == 0);
	/* clear bit */
	__raw_bits_or((1<<0), LCDC_IRQ_CLR);
	FB_PRINT("@fool2[%s] LCDC_IRQ_RAW: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IRQ_RAW));
	
}
static struct fb_ops sc8800fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = sc8800fb_check_var,
	.fb_pan_display = real_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};
static unsigned PP[16];
static void setup_fb_info(struct sc8800fb_info *sc8800fb)
{
	struct fb_info *fb= sc8800fb->fb;
	int r;

	fb->fbops = &sc8800fb_ops;
	fb->flags = FBINFO_DEFAULT;
	
	/* finish setting up the fb_info struct */
	strncpy(fb->fix.id, "sc8800fb", 16);
	fb->fix.ypanstep = 1;
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = sc8800fb->panel->width * 16/8;

	fb->var.xres = sc8800fb->panel->width;
	fb->var.yres = sc8800fb->panel->height;
	fb->var.width = sc8800fb->panel->width;
	fb->var.height = sc8800fb->panel->height;
	fb->var.xres_virtual = sc8800fb->panel->width;
	fb->var.yres_virtual = sc8800fb->panel->height * 2;
	fb->var.bits_per_pixel = BITS_PER_PIXEL;
	fb->var.accel_flags = 0;

	fb->var.yoffset = 0;

	fb->var.red.offset = 11;
	fb->var.red.length = 5;
	fb->var.red.msb_right = 0;
	fb->var.green.offset = 5;
	fb->var.green.length = 6;
	fb->var.green.msb_right = 0;
	fb->var.blue.offset = 0;
	fb->var.blue.length = 5;
	fb->var.blue.msb_right = 0;

	r = fb_alloc_cmap(&fb->cmap, 16, 0);
	fb->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}
static void lcdc_lcm_configure(struct sc8800fb_info *sc8800fb)
{
	uint32_t bits;
	
	//wait  until AHB FIFO is empty
	while(!(__raw_readl(LCM_STATUS) & (1<<2)));
	
	//__raw_writel(0, LCDC_LCM0PARAMETER0); /* LCM_PARAMETER0 */

	/* CS0 bus mode [BIT0]: 8080/6800 */
	switch (sc8800fb->panel->info.mcu->bus_mode) {
	case LCD_BUS_8080:
		bits = 0;
		break;
	case LCD_BUS_6800:
		bits = 1;
		break;
	default:
		bits = 0;
		break;
	}
	__raw_writel((bits&0x1), LCM_IFMODE);
	FB_PRINT("@fool2[%s] LCM_IFMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_IFMODE));

	/* CS0 bus width [BIT1:0] */
	switch (sc8800fb->panel->info.mcu->bus_width) {
	case 16:
		bits = 0;
		break;
	case 18:
		bits = 1;
		break;
	case 8:
		bits = 2;
		break;
	case 9:
		bits = 3;
		break;
	default:
		bits = 0;
		break;
	}
	__raw_writel((bits&0x3), LCM_RGBMODE);
	FB_PRINT("@fool2[%s] LCM_RGBMODE: 0x%x\n", __FUNCTION__, __raw_readl(LCM_RGBMODE));

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
static void lcdc_update_lcm_timing(struct sc8800fb_info *sc8800fb)
{
	uint32_t  reg_value;
	uint32_t  ahb_div,ahb_clk;   
	uint32_t  ahb_clk_cycle;
	uint32_t rcss, rlpw, rhpw, wcss, wlpw, whpw;

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
	ahb_clk = CHIP_GetMcuClk()/ahb_div;

	FB_PRINT("@fool2[%s] ahb_clk: 0x%x\n", __FUNCTION__, ahb_clk);

	/* LCD_UpdateTiming() */
	ahb_clk_cycle = (100000000 >> 17)/(ahb_clk >> 20 );

	rcss = ((sc8800fb->panel->info.mcu->timing->rcss/ahb_clk_cycle+1)<3)?
	       (sc8800fb->panel->info.mcu->timing->rcss/ahb_clk_cycle+1):3;
	rlpw = ((sc8800fb->panel->info.mcu->timing->rlpw/ahb_clk_cycle+1)<14)?
	       (sc8800fb->panel->info.mcu->timing->rlpw/ahb_clk_cycle+1):14;
	rhpw = ((sc8800fb->panel->info.mcu->timing->rhpw/ahb_clk_cycle+1)<14)?
	       (sc8800fb->panel->info.mcu->timing->rhpw/ahb_clk_cycle+1):14;
	wcss = ((sc8800fb->panel->info.mcu->timing->wcss/ahb_clk_cycle+1)<3)?
	       (sc8800fb->panel->info.mcu->timing->wcss/ahb_clk_cycle+1):3;
	wlpw = ((sc8800fb->panel->info.mcu->timing->wlpw/ahb_clk_cycle+1)<14)?
	       (sc8800fb->panel->info.mcu->timing->wlpw/ahb_clk_cycle+1):14;
	whpw = ((sc8800fb->panel->info.mcu->timing->whpw/ahb_clk_cycle+1)<14)?
	       (sc8800fb->panel->info.mcu->timing->whpw/ahb_clk_cycle+1):14;
	
	//wait  until AHB FIFO if empty
	while(!(__raw_readl(LCM_STATUS) & (1<<2)));
	
	/*   LCDC_ChangePulseWidth() */
	__raw_writel(whpw |(wlpw<<4) |(wcss<<8) | (rhpw<<10) |	(rlpw<<14) |(rcss<<18),LCM_PARAMETER0); /* FIXME: hardcoded for !CS0 */

	//__raw_writel( 0x77555, LCDC_LCMPARAMETER0); /* @fool2, tmp */
	FB_PRINT("@fool2[%s] LCM_PARAMETER0: 0x%x\n", __FUNCTION__, __raw_readl(LCM_PARAMETER0));
}
static inline int set_lcdsize( struct fb_info *info)
{
	uint32_t reg_val;
	
	//reg_val = ( info->var.xres & 0x3ff) | (( info->var.yres & 0x3ff )<<16);
	reg_val = ( 640 & 0x3ff) | (( 640 & 0x3ff )<<16);
	__raw_writel(reg_val, LCDC_DISP_SIZE);
	
	FB_PRINT("@fool2[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_SIZE));

	return 0;
}
static inline int set_lcmrect( struct fb_info *info)
{
	uint32_t reg_val;
	
	__raw_writel(0, LCDC_LCM_START);
	
	reg_val = ( info->var.xres & 0x3ff) | (( info->var.yres & 0x3ff )<<16);
	__raw_writel(reg_val, LCDC_LCM_SIZE);
	
	FB_PRINT("@fool2[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_START));
	FB_PRINT("@fool2[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_SIZE));

	return 0;
}
int set_lcdc_layers(struct fb_var_screeninfo *var, struct fb_info *info)
{
	uint32_t reg_val;

	/* we assume that
	 * 1. there's only one fbdev, and only block0 is used
	 * 2. the pan operation is a sync one
	 */

	__raw_bits_and(~(1<<0),LCDC_OSD1_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD2_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD3_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD4_CTRL);  
	__raw_bits_and(~(1<<0),LCDC_OSD5_CTRL);  
	/*enable imge layer*/
	__raw_bits_or((1<<0),LCDC_IMG_CTRL);  
#if 1
	/*little endian*/
	__raw_bits_or(1<<5,LCDC_IMG_CTRL);  
	__raw_bits_and(~(1<<6),LCDC_IMG_CTRL);  
	__raw_bits_or(1<<7,LCDC_IMG_CTRL);  
	__raw_bits_and(~(1<<8),LCDC_IMG_CTRL);  
	
	/*data format*/
	__raw_bits_or(1<<1,LCDC_IMG_CTRL);  //RGB565
	__raw_bits_and(~(1<<2),LCDC_IMG_CTRL);  
	__raw_bits_or(1<<3,LCDC_IMG_CTRL);  
	__raw_bits_and(~(1<<4),LCDC_IMG_CTRL);  	

	FB_PRINT("@fool2[%s] LCDC_IMG_CTRL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_CTRL));
	
	/* image layer base */
	reg_val = (info->var.yoffset)?info->fix.smem_start:	(info->fix.smem_start+ info->fix.smem_len/2);
	reg_val = (reg_val >>2)& 0x3fffffff;
	__raw_writel(reg_val, LCDC_IMG_Y_BASE_ADDR);

	FB_PRINT("@fool2[%s] LCDC_IMG_Y_BASE_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_Y_BASE_ADDR));
	
	/*image layer size*/
	reg_val = ( info->var.xres & 0x3ff) | (( info->var.yres & 0x3ff )<<16);
	__raw_writel(reg_val, LCDC_IMG_SIZE_XY);

	FB_PRINT("@fool2[%s] LCDC_IMG_SIZE_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_SIZE_XY));
	
	/*image layer start position*/
	__raw_writel(0, LCDC_IMG_DISP_XY);
	
	/*image layer pitch*/
	reg_val = ( info->var.xres & 0x3ff) ;
	__raw_writel(reg_val, LCDC_IMG_PITCH);
	
	FB_PRINT("@fool2[%s] LCDC_IMG_DISP_XY: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_IMG_DISP_XY));
#endif
	/*LCDC workplane size*/
	set_lcdsize(info);
	
	/*LCDC LCM rect size*/
	set_lcmrect(info);

	return 0;
}
static void hw_init(struct sc8800fb_info *sc8800fb)
{
	int ret;
	uint32_t reg_val=0;

	/* only MCU mode is supported currently */
	if (LCD_MODE_RGB == sc8800fb->panel->mode)
		return;

	set_pins();
	
	//misc_setup();
	
	//select LCD clock source	
	__raw_bits_or(1<<7, GR_PLL_SRC);    //pll_src=12M
	__raw_bits_and(~(1<<6), GR_PLL_SRC);  
	
	//set LCD divdior
	__raw_bits_and(~(1<<0), GR_GEN4);  //div=0
	__raw_bits_and(~(1<<1), GR_GEN4); 
	__raw_bits_and(~(1<<2), GR_GEN4);  
	
	//enable LCD clock
	__raw_bits_or(1<<3, AHB_CTL0); 

	//LCD soft reset
	__raw_bits_or(1<<3, AHB_SOFT_RST);
	mdelay(10);	
	__raw_bits_and(~(1<<3), AHB_SOFT_RST); 

	/* register isr */
	ret = request_irq(IRQ_LCDC_INT, lcdc_isr, IRQF_DISABLED, "LCDC", sc8800fb);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		return;
	}
	
	/* init lcdc mcu mode using default configuration */
	lcdc_mcu_init();
	
	//__raw_bits_or((1<<0), LCDC_DAC_CONTROL_REG); /*close tv_out */

	/* set lcdc-lcd interface parameters */
	lcdc_lcm_configure(sc8800fb);

	/* set timing parameters for LCD */
	lcdc_update_lcm_timing(sc8800fb);

	/* init mounted lcd panel */
	sc8800fb->panel->ops->lcd_init(sc8800fb->panel);

	set_lcdc_layers(&sc8800fb->fb->var, sc8800fb->fb); 
	
}
static void set_backlight(uint32_t value)
{
	
	ANA_REG_AND(WHTLED_CTL, ~(WHTLED_PD_SET | WHTLED_PD_RST));
	ANA_REG_OR(WHTLED_CTL,  WHTLED_PD_RST);
	ANA_REG_MSK_OR (WHTLED_CTL, ( (value << WHTLED_V_SHIFT) &WHTLED_V_MSK), WHTLED_V_MSK);
}
static int sc8800fb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct sc8800fb_info *sc8800fb;
	int32_t ret;

	FB_PRINT("@fool2[%s]\n", __FUNCTION__);
	printk("sc8800g_fb initialize!\n");
	
	fb = framebuffer_alloc(sizeof(struct sc8800fb_info), &pdev->dev);
	if (!fb)
		return -ENOMEM;
	sc8800fb = fb->par;
	sc8800fb->fb = fb;
	sc8800fb->ops = &lcm_mcu_ops;

	ret = mount_panel(sc8800fb, &lcd_panel);
	if (ret) {
		printk(KERN_ERR "unsupported panel!!");
		return -EFAULT;
	}

	ret = setup_fbmem(sc8800fb, pdev);
	if (ret)
		return ret;

	setup_fb_info(sc8800fb);

	ret = register_framebuffer(fb);
	if (ret) {
		framebuffer_release(fb);
		return ret;
	}

	hw_init(sc8800fb);

	/* FIXME: put the BL stuff to where it belongs. */
	set_backlight(50);

	
if(1){ /* in-kernel test code */
	struct fb_info test_info;
	int size = sc8800fb->fb->var.xres * sc8800fb->fb->var.yres *2;
	
	/* set color */
	memset(sc8800fb->fb->screen_base, 0x55, size);
	memset(sc8800fb->fb->screen_base+size, 0x55, size);

	/* pan display */
	test_info = *sc8800fb->fb;

//	while(1) {
		if (test_info.var.yoffset == 0)
			test_info.var.yoffset = sc8800fb->fb->var.yres;
		else
			test_info.var.yoffset = 0;

		real_pan_display(&sc8800fb->fb->var, &test_info);
		msleep(500);
	//}
}

	return 0;
}

static struct platform_driver sc8800fb_driver = {
	.probe = sc8800fb_probe,
	.driver = {
		.name = "sc8800fb",
		.owner = THIS_MODULE,
	},
};

static int __init sc8800fb_init(void)
{
	FB_PRINT("@fool2[%s]\n", __FUNCTION__);
	return platform_driver_register(&sc8800fb_driver);
}

module_init(sc8800fb_init);


