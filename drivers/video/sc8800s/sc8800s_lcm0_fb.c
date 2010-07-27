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
#include "sc8800s_lcd.h"

/* #define  FB_DEBUG */
#ifdef FB_DEBUG
#define FB_PRINT printk
#else
#define FB_PRINT(...)
#endif

/* sc8800s reg offset macros */
#define AHB_CTL0                (SPRD_AHB_BASE + 0x200)
#define AHB_SOFT_RST            (SPRD_AHB_BASE + 0x210)
#define AHB_AHB_ARM_CLK         (SPRD_AHB_BASE + 0x224)

/* LCM0 regs */
#define LCDC_LCM0PARAMETER0     (SPRD_NAND_BASE + 0x1c40)
#define LCDC_LCM0PARAMETER1     (SPRD_NAND_BASE + 0x1c4c)
#define LCDC_LCM0RSTN           (SPRD_NAND_BASE + 0x1c44)
#define LCDC_LCM0DATANUMBER     (SPRD_NAND_BASE + 0x1c68)
#define LCDC_LCM0STATUS         (SPRD_NAND_BASE + 0x1c58)
#define LCDC_LCM0MODESEL        (SPRD_NAND_BASE + 0x1c50)
#define LCDC_LCM0_RGB_MODE      (SPRD_NAND_BASE + 0x1c5c)
#define LCDC_LCM0_SELPIN        (SPRD_NAND_BASE + 0x1c60)
#define LCDC_LCM0_INT_STATUS    (SPRD_NAND_BASE + 0x1c84)
#define LCDC_LCM0_INT_EN        (SPRD_NAND_BASE + 0x1c88)
#define LCDC_LCM0_INT_VECTOR    (SPRD_NAND_BASE + 0x1c8c)
#define LCDC_LCM0_RW_MD         (SPRD_NAND_BASE + 0x1c64)
#define LCDC_LCM0_CS1_CD0_CMD   (SPRD_NAND_BASE + 0x1500)
#define LCDC_LCM0_CS1_CD0_DATA  (SPRD_NAND_BASE + 0x1504)

/* LCM1 regs */
#define LCDC_LCMPARAMETER0      (SPRD_LCD_LCM1_BASE + 0x00)
#define LCDC_LCMRSTN            (SPRD_LCD_LCM1_BASE + 0x08)
#define LCDC_LCMDATANUMBER      (SPRD_LCD_LCM1_BASE + 0x0c)
#define LCDC_LCMSTATUS          (SPRD_LCD_LCM1_BASE + 0x14)
#define LCDC_LCMMODESEL         (SPRD_LCD_LCM1_BASE + 0x18)
#define LCDC_LCM_RGB_MODE       (SPRD_LCD_LCM1_BASE + 0x1c)
#define LCDC_LCM_SELPIN         (SPRD_LCD_LCM1_BASE + 0x20)
#define LCDC_LCM_INT_STATUS     (SPRD_LCD_LCM1_BASE + 0x24)
#define LCDC_LCM_INT_EN         (SPRD_LCD_LCM1_BASE + 0x28)
#define LCDC_LCM_INT_VECTOR     (SPRD_LCD_LCM1_BASE + 0x2c)
#define LCDC_LCM_CS0_CD0_CMD    (SPRD_LCD_LCM1_BASE + 0x1400)
#define LCDC_LCM_CS0_CD0_DATA   (SPRD_LCD_LCM1_BASE + 0x1404)

#define LCDC_LCD_MODE           (SPRD_LCD_MCU_BASE + 0x00)
#define LCDC_LCD_BACKGROUND     (SPRD_LCD_MCU_BASE + 0x0c)
#define LCDC_LCD_BLOCK0ADDR     (SPRD_LCD_MCU_BASE + 0x10)
#define LCDC_LCD_BLOCK0START    (SPRD_LCD_MCU_BASE + 0x14)
#define LCDC_LCD_BLOCK0END      (SPRD_LCD_MCU_BASE + 0x18)
#define LCDC_LCD_BLOCK0CONFIG   (SPRD_LCD_MCU_BASE + 0x1c)
#define LCDC_LCD_BLOCK2CONFIG   (SPRD_LCD_MCU_BASE + 0x3c)
#define LCDC_COLOR_COEFF_A1     (SPRD_LCD_MCU_BASE + 0x70)
#define LCDC_COLOR_COEFF_A2     (SPRD_LCD_MCU_BASE + 0x74)
#define LCDC_COLOR_COEFF_A3     (SPRD_LCD_MCU_BASE + 0x78)
#define LCDC_COLOR_COEFF_B1     (SPRD_LCD_MCU_BASE + 0x7c)
#define LCDC_COLOR_COEFF_B2     (SPRD_LCD_MCU_BASE + 0x80)
#define LCDC_COLOR_COEFF_B3     (SPRD_LCD_MCU_BASE + 0x84)
#define LCDC_COLOR_COEFF_C1     (SPRD_LCD_MCU_BASE + 0x88)
#define LCDC_COLOR_COEFF_C2     (SPRD_LCD_MCU_BASE + 0x8c)
#define LCDC_COLOR_COEFF_C3     (SPRD_LCD_MCU_BASE + 0x90)
#define LCDC_LCD_INT_ENABLE     (SPRD_LCD_MCU_BASE + 0x94)
#define LCDC_LCD_INT_STATUS     (SPRD_LCD_MCU_BASE + 0x98)
#define LCDC_LCD_INT_CLEAR      (SPRD_LCD_MCU_BASE + 0x9c)
#define LCDC_DISP_WIN_START_ADDR (SPRD_LCD_MCU_BASE + 0xac)
#define LCDC_DISP_WIN_END_ADDR   (SPRD_LCD_MCU_BASE + 0xb0)
#define LCDC_DAC_CONTROL_REG    (SPRD_LCD_MCU_BASE + 0x110)

#define GR_GEN1                 (SPRD_GREG_BASE + 0x18)
#define GR_MPLL_MN              (SPRD_GREG_BASE + 0x24)
#define GR_GEN2                 (SPRD_GREG_BASE + 0x2c)
#define GR_BUSCLK_ALM           (SPRD_GREG_BASE + 0x44)
#define GR_GEN4                 (SPRD_GREG_BASE + 0x60)
#define GR_VPLL_MN              (SPRD_GREG_BASE + 0x68)

/* gpio id */
#define GPIO_PROD_LCD_BL_BRIGHTNESS_ID 103
#define GPIO_PROD_LCD_RST              83

/* macros for sc8800s fb */
#define BITS_PER_PIXEL 16
#define LCDC_CS 0
#define LCDC_CD 0

struct sc8800sfb_info {
	struct fb_info *fb;
	uint32_t cap;
	struct ops_mcu *ops;
	struct lcd_spec *panel;
};

#ifndef __ARMEB__
static uint32_t swapbuf; /* endian swap buffer ptr */
#endif

static void set_pins(void);
static void misc_setup(void);

static int sc8800sfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
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

#ifndef __ARMEB__
#define WORD_SWAP(word) ((word<<16)|(word>>16))
static void convert_format(uint32_t *src, uint32_t *dst, uint32_t w, uint32_t h)
{
	uint32_t i = w * h /2;

	while (i--) {
		*(dst++) = WORD_SWAP(*src);
		src++;
	}
}

#endif
int set_lcdc_block(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct sc8800sfb_info *sc8800sfb = info->par;
	uint32_t reg_val;

	uint32_t tmp;

	/* we assume that
	 * 1. there's only one fbdev, and only block0 is used
	 * 2. the pan operation is a sync one
	 */

	/* switch buffer base */
#ifndef __ARMEB__
	reg_val = swapbuf;
#else
	reg_val = (info->var.yoffset)?info->fix.smem_start:
		(info->fix.smem_start+ info->fix.smem_len/2);
#endif
	__raw_writel(reg_val, LCDC_LCD_BLOCK0ADDR);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0ADDR));

	reg_val = (1<<28) | (1<<31); /* startx/y=0, prio=1, enable */
	__raw_writel(reg_val, LCDC_LCD_BLOCK0START);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0START));

	reg_val = (info->var.xres-1) |
		((info->var.yres-1)<<12) |
		(((info->var.width>>3))<<24); /* endx/y and width */
	__raw_writel(reg_val, LCDC_LCD_BLOCK0END);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0END: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0END));

	/* FIXME: hardcoded color format */
	reg_val = 1 | (2<<24) | (1<<26) | (1<<28);
	__raw_writel(reg_val, LCDC_LCD_BLOCK0CONFIG);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0CONFIG: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0CONFIG));

	/* FIXME: strange code... copied from lcd_sc8800h.c*/
	__raw_bits_or((1<<27)|(1<<28), LCDC_LCD_BLOCK2CONFIG);
	__raw_bits_or(0xff, LCDC_LCD_BLOCK2CONFIG);

	/* LCDC_SetDisplayWindows */
	__raw_writel(0, LCDC_DISP_WIN_START_ADDR);
	reg_val = (info->var.xres-1) | ((info->var.yres-1)<<16);
	__raw_writel(reg_val, LCDC_DISP_WIN_END_ADDR);
	FB_PRINT("@fool2[%s] LCDC_DISP_WIN_START_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_WIN_START_ADDR));
	FB_PRINT("@fool2[%s] LCDC_DISP_WIN_END_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_WIN_END_ADDR));

	return 0;
}

int real_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct sc8800sfb_info *sc8800sfb = info->par;
	uint32_t reg_val;

#ifndef __ARMEB__
	reg_val = (info->var.yoffset == 0)?info->screen_base:
		(info->screen_base + info->fix.smem_len/2);
	convert_format((uint32_t*)reg_val, (uint32_t*)swapbuf, 
			info->var.xres, info->var.yres);
#else
	reg_val = (info->var.yoffset == 0)?info->fix.smem_start:
		(info->fix.smem_start+ info->fix.smem_len/2);
	__raw_writel(reg_val, LCDC_LCD_BLOCK0ADDR);
#endif

	/* set brush direction */
	/* FIXME: hardcoded direction */
	//sc8800sfb->panel->ops->lcd_set_direction(sc8800sfb->panel, 0);
	sc8800sfb->panel->ops->lcd_invalidate(sc8800sfb->panel);

	/* write data to LCD GRAM - lcdc_mcu_transcmd() */
	__raw_bits_or((1<<3),  LCDC_LCD_MODE); /* enable shadow update */
	FB_PRINT("@fool2[%s] LCDC_LCD_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_MODE));

	reg_val = info->var.xres * info->var.yres;
	reg_val |= (1<<20); /* for device 0 */
	reg_val |= (1<<26); /* FIXME: hardcoded cs 1 */
	__raw_writel(reg_val, LCDC_LCM0DATANUMBER);
	FB_PRINT("@fool2[%s] LCDC_LCMDATANUMBER: 0x%x, reg_val=0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCMDATANUMBER), reg_val);

	__raw_bits_or((1<<5), LCDC_LCD_MODE); /* start refresh */
	FB_PRINT("@fool2[%s] LCDC_LCD_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_MODE));

	/* busy wait for OSD refresh Done */
	while((__raw_readl(LCDC_LCM0_INT_STATUS) & (1<<1)) == 0);
	/* clear bit */
	__raw_bits_or((1<<1), LCDC_LCM0_INT_STATUS);
	FB_PRINT("@fool2[%s] LCDC_LCM_INT_STATUS: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_INT_STATUS));

	/* FIXME: hardcoded direction, and why? */
	//sc8800sfb->panel->ops->lcd_set_direction(sc8800sfb->panel, 0);
}

int sc8800sfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct sc8800sfb_info *sc8800sfb = info->par;
	uint32_t reg_val;

	uint32_t tmp;

	/* we assume that
	 * 1. there's only one fbdev, and only block0 is used
	 * 2. the pan operation is a sync one
	 */

	/* switch buffer base */
#ifndef __ARMEB__
	reg_val = (info->var.yoffset)?info->screen_base:
		(info->screen_base + info->fix.smem_len/2);
	convert_format((uint32_t*)reg_val, (uint32_t*)swapbuf, 
			info->var.xres, info->var.yres);
	reg_val = swapbuf;
#else
	reg_val = (info->var.yoffset)?info->fix.smem_start:
		(info->fix.smem_start+ info->fix.smem_len/2);
#endif
	__raw_writel(reg_val, LCDC_LCD_BLOCK0ADDR);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0ADDR));

	tmp = (info->var.yoffset)?info->screen_base:
		(info->screen_base+ info->fix.smem_len/2);

	FB_PRINT("@fool2[%s] fb@pa0x%08x->va0x%08x (0x%08x, ..., 0x%08x)\n", __FUNCTION__, reg_val,tmp, *((uint32_t*)tmp), *((uint32_t*)(tmp+info->fix.smem_len/2-4)));

	reg_val = (1<<28) | (1<<31); /* startx/y=0, prio=1, enable */
	__raw_writel(reg_val, LCDC_LCD_BLOCK0START);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0START: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0START));

	reg_val = (info->var.xres-1) |
		((info->var.yres-1)<<12) |
		(((info->var.width>>3))<<24); /* endx/y and width */
	__raw_writel(reg_val, LCDC_LCD_BLOCK0END);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0END: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0END));

	/* FIXME: hardcoded color format */
	reg_val = 1 | (2<<24) | (1<<26) | (1<<28);
	__raw_writel(reg_val, LCDC_LCD_BLOCK0CONFIG);
	FB_PRINT("@fool2[%s] LCDC_LCD_BLOCK0CONFIG: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_BLOCK0CONFIG));

	/* FIXME: strange code... copied from lcd_sc8800h.c*/
	/*
	reg_val = (1<<27) | (1<<28);
	__raw_writel(reg_val, LCDC_LCD_BLOCK2CONFIG);
	reg_val = (1<<27) | (1<<28) | 0xff;
	__raw_writel(reg_val, LCDC_LCD_BLOCK2CONFIG);
	*/
	__raw_bits_or((1<<27)|(1<<28), LCDC_LCD_BLOCK2CONFIG);
	__raw_bits_or(0xff, LCDC_LCD_BLOCK2CONFIG);

	/* LCDC_SetDisplayWindows */
	__raw_writel(0, LCDC_DISP_WIN_START_ADDR);
	reg_val = (info->var.xres-1) | ((info->var.yres-1)<<16);
	__raw_writel(reg_val, LCDC_DISP_WIN_END_ADDR);
	FB_PRINT("@fool2[%s] LCDC_DISP_WIN_START_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_WIN_START_ADDR));
	FB_PRINT("@fool2[%s] LCDC_DISP_WIN_END_ADDR: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_DISP_WIN_END_ADDR));

	/* set brush direction */
	/* FIXME: hardcoded direction */
	sc8800sfb->panel->ops->lcd_set_direction(sc8800sfb->panel, 0);
	sc8800sfb->panel->ops->lcd_invalidate(sc8800sfb->panel);

	/* write data to LCD GRAM - lcdc_mcu_transcmd() */
	__raw_bits_or((1<<3),  LCDC_LCD_MODE); /* enable shadow update */
	FB_PRINT("@fool2[%s] LCDC_LCD_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_MODE));

	reg_val = info->var.xres * info->var.yres;
	reg_val |= (1<<20); /* for device 0 */
	reg_val |= (1<<26); /* FIXME: hardcoded cs 1 */
	__raw_writel(reg_val, LCDC_LCM0DATANUMBER);
	FB_PRINT("@fool2[%s] LCDC_LCMDATANUMBER: 0x%x, reg_val=0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCMDATANUMBER), reg_val);

	__raw_bits_or((1<<5), LCDC_LCD_MODE); /* start refresh */
	FB_PRINT("@fool2[%s] LCDC_LCD_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_MODE));

	/* busy wait for OSD refresh Done */
	while((__raw_readl(LCDC_LCM0_INT_STATUS) & (1<<1)) == 0);
	/* clear bit */
	__raw_bits_or((1<<1), LCDC_LCM0_INT_STATUS);
	FB_PRINT("@fool2[%s] LCDC_LCM_INT_STATUS: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM_INT_STATUS));

	/* FIXME: hardcoded direction, and why? */
	sc8800sfb->panel->ops->lcd_set_direction(sc8800sfb->panel, 0);

	return 0;
}

static struct fb_ops sc8800sfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = sc8800sfb_check_var,
	//.fb_pan_display = sc8800sfb_pan_display,
	.fb_pan_display = real_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int32_t lcm_send_cmd (uint32_t cmd)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCDC_LCM0STATUS) & 0x1);

	__raw_writel(cmd, LCDC_LCM0_CS1_CD0_CMD);

	return 0;
}

static int32_t lcm_send_cmd_data (uint32_t cmd, uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCDC_LCM0STATUS) & 0x1);

	__raw_writel(cmd, LCDC_LCM0_CS1_CD0_CMD);

	__raw_writel(data, LCDC_LCM0_CS1_CD0_DATA);

	return 0;
}

static int32_t lcm_send_data (uint32_t data)
{
	/* busy wait for ahb fifo full sign's disappearance */
	while(__raw_readl(LCDC_LCM0STATUS) & 0x1);

	__raw_writel(data, LCDC_LCM0_CS1_CD0_DATA);

	return 0;
}

static struct ops_mcu lcm_mcu_ops = {
	.send_cmd = lcm_send_cmd,
	.send_cmd_data = lcm_send_cmd_data,
	.send_data = lcm_send_data,
};

static irqreturn_t lcdc_isr(int irq, void *data)
{

	/* FIXME: can't find tv bits in spec */
	if((__raw_readl(LCDC_LCD_MODE) & (1<<1)) != 1) { /* !TV int */
		/* TODO: add rgb mode stuff */
		uint32_t val = __raw_readl(LCDC_LCM0_INT_VECTOR);

		if (val & (1<<3)) { /* fifo overflow */
			__raw_bits_or((1<<3), LCDC_LCM0_INT_STATUS);
	FB_PRINT("@fool2[%s] fifo overflow\n", __FUNCTION__);
		}
		if (val & (1<<1)) { /* OSD frame done */
			/* TODO: notify the rest of the system */
			__raw_bits_or((1<<1), LCDC_LCM0_INT_STATUS);
	FB_PRINT("@fool2[%s] osd frame done\n", __FUNCTION__);
		}

		val = __raw_readl(LCDC_LCD_INT_STATUS);
		if (val & (1<<0)) { /* Async FIFO underflow */
			__raw_bits_or((1<<0), LCDC_LCD_INT_CLEAR);
	FB_PRINT("@fool2[%s] fifo underflow\n", __FUNCTION__);
		}
	} else {
		/* TODO: add TV stuff */
	FB_PRINT("@fool2[%s] tv?\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

/* directly copied from chip.c */
uint32_t CHIP_GetVPllClk (void)
{
	uint32_t ext_clk_26M;
	uint32_t pll_freq;
	uint32_t reg_value, M, N;

	ext_clk_26M = (__raw_readl(GR_GEN1) >> 15) & 0x1;
	FB_PRINT("@fool2[%s] GR_GEN1:0x%x\n", __FUNCTION__, __raw_readl(GR_GEN1));

	reg_value = __raw_readl(GR_VPLL_MN);
	M = reg_value & 0x0fff;
	N = (reg_value & 0x0fff0000)>>16;

	if(ext_clk_26M)
		pll_freq = 26*N/M;
	else
		pll_freq = 13*N/M;

	FB_PRINT("@fool2[%s] GR_VPLL_MN:0x%x, M:%d, N:%d, pll_freq: %d\n", __FUNCTION__, reg_value, M, N, pll_freq);

	return (pll_freq*1000000);
}
/* directly copied from power_manager_sc8800h.c.c */
static void PWRMNG_SetVpll(uint32_t vpll_clk)
{
	unsigned long flags;

	raw_local_irq_save(flags);

	__raw_bits_or((1<<20), GR_GEN1);

	if (__raw_readl(GR_GEN1) & (1<<15)) {
		__raw_writel(((vpll_clk/1000000) << 16) | 0x1a, GR_VPLL_MN);
	} else {
		__raw_writel(((vpll_clk/1000000) << 16) | 0x0d, GR_VPLL_MN);
	}

	__raw_bits_and(~(1<<20), GR_GEN1);

	raw_local_irq_restore(flags);
}

static void PWRMNG_ForcePowerOnVPll(void)
{
	uint32_t i;
	unsigned long flags;

	raw_local_irq_save(flags);

	if (0 == ((__raw_readl(GR_BUSCLK_ALM) & (1<<12)) >>12))
		__raw_bits_and( ~(1<<14), GR_BUSCLK_ALM);
	else
		printk(KERN_ERR "VPLL pd src ctl error!\n");

	raw_local_irq_restore(flags);

	for(i=0; i<5000; i++);
}
static void PWRMNG_LCDC_ClkSwitch(uint32_t b_clk_on)
{
	if(b_clk_on)
		__raw_bits_or((1<<3), AHB_CTL0);
	else
		__raw_bits_and(~(1<<3), AHB_CTL0);

}

static void hw_set_pll(struct sc8800sfb_info *sc8800sfb)
{
	uint32_t pll_clk, clk_val, clk_div;
	uint32_t reg_val;

	pll_clk = CHIP_GetVPllClk();

	pll_clk /= 1000000;
	clk_val = 12;
	clk_div = pll_clk/clk_val -1;

	reg_val = __raw_readl(GR_GEN4);
	reg_val &= ~0xff;
	reg_val |= (0xff&clk_div);
	__raw_writel(reg_val, GR_GEN4);

	FB_PRINT("@fool2[%s] GR_GEN4: 0x%x\n",__FUNCTION__, __raw_readl(GR_GEN4));

	PWRMNG_SetVpll(192000000);
	PWRMNG_ForcePowerOnVPll();
	PWRMNG_LCDC_ClkSwitch(1);
}

static void lcdc_mcu_init(void)
{
	/* clear block register */
	{
		uint32_t i;
		for (i=0; i<((0x2070006c-0x20700010)/4) + 1; i++) {
			__raw_writel(0, SPRD_LCD_MCU_BASE + 10 + 4*i);
		}
	}

	/* set color correction coefficient */
	__raw_writel(0x100, LCDC_COLOR_COEFF_A1);
	__raw_writel(0,     LCDC_COLOR_COEFF_A2);
	__raw_writel(0,     LCDC_COLOR_COEFF_A3);
	__raw_writel(0,     LCDC_COLOR_COEFF_B1);
	__raw_writel(0x100, LCDC_COLOR_COEFF_B2);
	__raw_writel(0,     LCDC_COLOR_COEFF_B3);
	__raw_writel(0,     LCDC_COLOR_COEFF_C1);
	__raw_writel(0,     LCDC_COLOR_COEFF_C2);
	__raw_writel(0x100, LCDC_COLOR_COEFF_C3);

	/* set LCDC mode */
	__raw_bits_or((1<<4),  LCDC_LCD_MODE); /* refresh sigle frame */
	/* CONSOLE KILLER!! */
	__raw_bits_or((1<<0),  LCDC_LCD_MODE); /* set lcdc mode */
	/* CONSOLE KILLER!! */
	
	__raw_bits_or((1<<9), AHB_CTL0);       /* enable LCM0 */
	__raw_bits_and(~(1<<11), LCDC_LCD_MODE); /* de-select LCM1 */
	__raw_bits_and(~(1<<10), LCDC_LCD_MODE); /* disable LCM1 */
	__raw_writel(0, LCDC_LCM0_RW_MD);      /* lcm wirte mode */

	mdelay(1);
	__raw_writel(1, LCDC_LCM0RSTN);
	__raw_bits_or((1<<2),  LCDC_LCD_MODE); /* enable lcdc */

	__raw_writel(0xff, LCDC_LCD_BACKGROUND); /* set background */

	__raw_bits_or((1<<3), LCDC_LCM0_INT_EN); /* enable overflow int */

	__raw_bits_or((1<<3),  LCDC_LCD_MODE); /* enable shadowing */

	__raw_bits_or((1<<0), LCDC_LCD_INT_ENABLE); /* enable underflow int */

	FB_PRINT("@fool2[%s] LCDC_LCD_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCD_MODE));
}

static void lcdc_mcu_configure(struct sc8800sfb_info *sc8800sfb)
{
	uint32_t bits;

	__raw_writel(0, LCDC_LCM0PARAMETER0); /* LCM_PARAMETER0 */

	/* CS0 bus mode [BIT0]: 8080/6800 */
	switch (sc8800sfb->panel->info.mcu->bus_mode) {
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
	__raw_writel((bits&0x1), LCDC_LCM0MODESEL);
	FB_PRINT("@fool2[%s] LCDC_LCM0MODESEL: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM0MODESEL));

	/* CS0 bus width [BIT1:0] */
	switch (sc8800sfb->panel->info.mcu->bus_width) {
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
	__raw_writel((bits&0x3), LCDC_LCM0_RGB_MODE);
	FB_PRINT("@fool2[%s] LCDC_LCM0_RGB_MODE: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM0_RGB_MODE));

	__raw_writel(0, LCDC_LCM0_SELPIN);
	FB_PRINT("@fool2[%s] LCDC_LCM0_SELPIN: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM0_SELPIN));
}

static void lcd_update_timing(struct sc8800sfb_info *sc8800sfb)
{
	uint32_t pll_freq, reg_value, M, N;
	uint32_t clk_ahb_div, ahb_clk, ahb_clk_cycle;
	uint32_t rcss, rlpw, rhpw, wcss, wlpw, whpw;

	/* CHIP_GetAhbClk() */
	reg_value = __raw_readl(GR_MPLL_MN);
	M = reg_value & 0x0fff;
	N = (reg_value & 0x0fff0000) >> 16;
	pll_freq = 26*N/M;

	clk_ahb_div = (__raw_readl(AHB_AHB_ARM_CLK) >> 5) & 0x1f;

	ahb_clk = pll_freq*1000000 / (clk_ahb_div + 1);

	FB_PRINT("@fool2[%s] ahb_clk: 0x%x\n", __FUNCTION__, ahb_clk);

	/* LCD_UpdateTiming() */
	ahb_clk_cycle = 1000000000/ahb_clk;

	rcss = ((sc8800sfb->panel->info.mcu->timing->rcss/ahb_clk_cycle+1)<3)?
	       (sc8800sfb->panel->info.mcu->timing->rcss/ahb_clk_cycle+1):3;
	rlpw = ((sc8800sfb->panel->info.mcu->timing->rlpw/ahb_clk_cycle+1)<15)?
	       (sc8800sfb->panel->info.mcu->timing->rlpw/ahb_clk_cycle+1):15;
	rhpw = ((sc8800sfb->panel->info.mcu->timing->rhpw/ahb_clk_cycle+1)<15)?
	       (sc8800sfb->panel->info.mcu->timing->rhpw/ahb_clk_cycle+1):15;
	wcss = ((sc8800sfb->panel->info.mcu->timing->wcss/ahb_clk_cycle+1)<3)?
	       (sc8800sfb->panel->info.mcu->timing->wcss/ahb_clk_cycle+1):3;
	wlpw = ((sc8800sfb->panel->info.mcu->timing->wlpw/ahb_clk_cycle+1)<15)?
	       (sc8800sfb->panel->info.mcu->timing->wlpw/ahb_clk_cycle+1):15;
	whpw = ((sc8800sfb->panel->info.mcu->timing->whpw/ahb_clk_cycle+1)<15)?
	       (sc8800sfb->panel->info.mcu->timing->whpw/ahb_clk_cycle+1):15;

	/*   LCDC_ChangePulseWidth() */
	__raw_writel(whpw |
	            (wlpw<<4) |
		    (wcss<<8) |
		    (rhpw<<10) |
		    (rlpw<<14) |
		    (rcss<<18),
		    LCDC_LCM0PARAMETER1); /* FIXME: hardcoded for !CS0 */

	//__raw_writel( 0x77555, LCDC_LCMPARAMETER0); /* @fool2, tmp */
	FB_PRINT("@fool2[%s] LCDC_LCM0PARAMETER1: 0x%x\n", __FUNCTION__, __raw_readl(LCDC_LCM0PARAMETER1));
}

static void hw_init(struct sc8800sfb_info *sc8800sfb)
{
	int ret;

	/* only MCU mode is supported currently */
	if (LCD_MODE_RGB == sc8800sfb->panel->mode)
		return;

	set_pins();

	misc_setup();

	FB_PRINT("@fool2[%s] AHB_SOFT_RST: 0x%x\n", __FUNCTION__, __raw_readl(AHB_SOFT_RST));
	/* LCDC_Reset */
	__raw_bits_or(1<<3, AHB_SOFT_RST);
	__raw_bits_or(1<<4, AHB_SOFT_RST); /* FIXME: spec says Don't do this */
	FB_PRINT("@fool2[%s] AHB_SOFT_RST: 0x%x\n", __FUNCTION__, __raw_readl(AHB_SOFT_RST));
	mdelay(1);
	__raw_bits_and(~(1<<3), AHB_SOFT_RST);
	__raw_bits_and(~(1<<4), AHB_SOFT_RST);
	FB_PRINT("@fool2[%s] AHB_SOFT_RST: 0x%x\n", __FUNCTION__, __raw_readl(AHB_SOFT_RST));

	/*
	 * select LCM0 - LCDC_LCM_Sel()
	 */
	PWRMNG_LCDC_ClkSwitch(1);
	__raw_bits_and(~(1<<11), LCDC_LCD_MODE);


	/* 
	 * init LCDC
	 */

	/* register isr */
	ret = request_irq(IRQ_NLC_INT, lcdc_isr, IRQF_DISABLED, 
		"LCDC", sc8800sfb);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		return;
	}

	/* set clock */
	hw_set_pll(sc8800sfb);

	/* init lcdc mcu mode using default configuration */
	lcdc_mcu_init();
	
	__raw_bits_or((1<<0), LCDC_DAC_CONTROL_REG); /*close tv_out */

	/* set lcdc-lcd interface parameters */
	lcdc_mcu_configure(sc8800sfb);

	/* set timing parameters for LCD */
	lcd_update_timing(sc8800sfb);

	/* set block regs */
	/* lcd_config_block(sc8800sfb); */

	/* init mounted lcd panel */
	sc8800sfb->panel->ops->lcd_init(sc8800sfb->panel);

	set_lcdc_block(&sc8800sfb->fb->var, sc8800sfb->fb); /* TEMP */
}

static unsigned PP[16];

static void setup_fb_info(struct sc8800sfb_info *sc8800sfb)
{
	struct fb_info *fb_info = sc8800sfb->fb;
	int r;

	/* finish setting up the fb_info struct */
	strncpy(fb_info->fix.id, "sc8800sfb", 16);
	fb_info->fix.ypanstep = 1;

	fb_info->fbops = &sc8800sfb_ops;
	fb_info->flags = FBINFO_DEFAULT;

	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR;
	fb_info->fix.line_length = sc8800sfb->panel->width * 2;

	fb_info->var.xres = sc8800sfb->panel->width;
	fb_info->var.yres = sc8800sfb->panel->height;
	fb_info->var.width = sc8800sfb->panel->width;
	fb_info->var.height = sc8800sfb->panel->height;
	fb_info->var.xres_virtual = sc8800sfb->panel->width;
	fb_info->var.yres_virtual = sc8800sfb->panel->height * 2;
	fb_info->var.bits_per_pixel = BITS_PER_PIXEL;
	fb_info->var.accel_flags = 0;

	fb_info->var.yoffset = 0;

	fb_info->var.red.offset = 11;
	fb_info->var.red.length = 5;
	fb_info->var.red.msb_right = 0;
	fb_info->var.green.offset = 5;
	fb_info->var.green.length = 6;
	fb_info->var.green.msb_right = 0;
	fb_info->var.blue.offset = 0;
	fb_info->var.blue.length = 5;
	fb_info->var.blue.msb_right = 0;

	r = fb_alloc_cmap(&fb_info->cmap, 16, 0);
	fb_info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}

static int setup_fbmem(struct sc8800sfb_info *fb, struct platform_device *pdev)
{
	uint32_t len, addr;
	
	len = fb->panel->width * fb->panel->height * (BITS_PER_PIXEL/8) * 2;

	/* the addr should be 8 byte align */
	addr = __get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len));
	if (!addr)
		return -ENOMEM;

#ifndef __ARMEB__
	swapbuf = __get_free_pages(GFP_ATOMIC | __GFP_ZERO, get_order(len/2));
	if (!swapbuf) {
		free_pages(addr, get_order(len));
		return -ENOMEM;
	}
#endif
	printk("sc8800s_fb got %d bytes mem at 0x%x\n", len, addr);
	fb->fb->fix.smem_start = __pa(addr);
	fb->fb->fix.smem_len = len;
	fb->fb->screen_base = (char*)addr;

	return 0;
}

/* some temporary code for GPIO */
static void gpio_set_val(int32_t id, int32_t level)
{
	int32_t page, bit_num, gpio_pg_base;

	page = id >>4;
	bit_num = id&0xf;
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
	gpio_pg_base = page*0x80 + SPRD_GPIO_BASE;

	__raw_bits_or((1<<bit_num), gpio_pg_base+4);
}

/* direction: 1-output, 0-input */
static void gpio_set_direction(int32_t id, uint32_t direction)
{
	int32_t page, bit_num, gpio_pg_base;

	page = id >>4;
	bit_num = id&0xf;
	gpio_pg_base = page*0x80 + SPRD_GPIO_BASE;

	FB_PRINT("@fool2[%s] gpio[%d] dir reg val.before: 0x%x\n", __FUNCTION__, id, __raw_readl(gpio_pg_base+8));

	if(direction)
		__raw_bits_or((1<<bit_num), gpio_pg_base+8);
	else
		__raw_bits_and(~(1<<bit_num), gpio_pg_base+8);

	FB_PRINT("@fool2[%s] gpio[%d] dir reg val.after: 0x%x\n", __FUNCTION__, id, __raw_readl(gpio_pg_base+8));
}

static int32_t gpio_reset_lcd(struct lcd_spec *self)
{
	/* select  pin function */
	__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x244));

	gpio_enable(GPIO_PROD_LCD_RST);

	gpio_set_direction(GPIO_PROD_LCD_RST, 1);

	/* gpio# GPIO_PROD_LCD_RST, high-low-high */
	gpio_set_val(GPIO_PROD_LCD_RST, 1);
	mdelay(5);
	gpio_set_val(GPIO_PROD_LCD_RST, 0);
	mdelay(5);
	gpio_set_val(GPIO_PROD_LCD_RST, 1);

	return 0;
}


/* 
 * there may be some LCD related pins need to be set... 
 * PM_Init/PM_GPIOInit
 */
static void set_pins(void)
{
	/* pins connect to LCM/lcd */
	__raw_writel(0x30|0x80, (SPRD_CPC_BASE+0x8c));

	__raw_writel(1|0x30, (SPRD_CPC_BASE+0x1f8));
	__raw_writel(1|0x30|0x40, (SPRD_CPC_BASE+0x200));

	__raw_writel(1|0x10|0x40, (SPRD_CPC_BASE+0x23c));
	__raw_writel(0x10, (SPRD_CPC_BASE+0x240));
	__raw_writel(1|0xc0, (SPRD_CPC_BASE+0x244)); /* GPIO102 */
	__raw_writel(1|0xc0, (SPRD_CPC_BASE+0x248));   /* GPIO103 */
	
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x338));   /* GPIO116 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x33c));   /* GPIO117 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x340));   /* GPIO118 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x344));   /* GPIO119 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x348));   /* GPIO120 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x34c));   /* GPIO121 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x350));   /* GPIO122 */
	__raw_writel(0x10|0x40, (SPRD_CPC_BASE+0x354));   /* GPIO123 */
	__raw_writel(2|0x10|0xc0, (SPRD_CPC_BASE+0x358));   /* GPIO124 */

	/* gpio 141/143/144 - MTV_STANDBY/MTV_RESET/MTV_LCDBYPASS  */
	__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x2cc));
	__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x2d4));
	__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x2d8));

	/*
	 * PM_GPIOInit()
	 */

	/* enable gpio blocks */
	__raw_bits_or(0x7ff, GR_GEN2);

	/* init GPIO pins */

	/* MTV_STANDBY */
	gpio_enable(141);
	gpio_set_direction(141, 1);
	gpio_set_val(141, 0);

	/* MTV_RESET */
	gpio_enable(143);
	gpio_set_direction(143, 1);
	gpio_set_val(143, 1);

	/* MTV_LCDBYPASS */
	gpio_enable(144);
	gpio_set_direction(144, 1);
	gpio_set_val(144, 0);

	/* LCD_RST */
	gpio_enable(83);
	gpio_set_direction(83, 1);
	gpio_set_val(83, 1);

	/* MTV_SOFTSWITCH */
	gpio_enable(111);
	gpio_set_direction(111, 1);
	gpio_set_val(111, 1);

	/* LCD_BL_EN_ */
	gpio_enable(103);
	gpio_set_direction(103, 1);
	gpio_set_val(103, 0);
}

static void misc_setup(void)
{
	uint32_t reg;

	/* CHIP_ChangeCoreVoltage() */
	reg = __raw_readl(SPRD_GREG_BASE+0x38) & 0xfffffff0;
	reg |= (1<<2) | (1<<0);
	__raw_writel(reg, SPRD_GREG_BASE+0x38);

}

static int mount_panel(struct sc8800sfb_info *fb, struct lcd_spec *panel)
{
	/* TODO: check whether the mode/res are supported */

	fb->panel = panel;

	panel->info.mcu->ops = fb->ops;

	panel->ops->lcd_reset = gpio_reset_lcd;
	
	return 0;
}

/* copied from  _LCD_SetBackLightBrightness_AAT3155() */
static void set_backlight(uint32_t level)
{
	uint32_t i;
	unsigned long flags;

	FB_PRINT("@fool2[%s] start\n", __FUNCTION__);

	/* select pin function */
	//__raw_bits_or((3<<6), (SPRD_CPC_BASE+0x1f8));
	//__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x1f8));
	__raw_writel(1|(3<<6), (SPRD_CPC_BASE+0x248)); /* gpio103 */

	gpio_enable(GPIO_PROD_LCD_BL_BRIGHTNESS_ID);

	gpio_set_direction(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 1);

	if(0 != level) {
		if (level >= 100)
			level = 96;
		if (level >= 6)
			level = level/6;
		else
			level = 1;

		level = 16 - level;

		/* gpio# GPIO_PROD_LCD_BL_BRIGHTNESS_ID */
		gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 0);
		mdelay(2);

		raw_local_irq_save(flags);

		gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 1);
		udelay(30);
		gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 0);
		udelay(30);

		for (i = 1; i< level; i++) {
			gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 1);
			udelay(30);
			gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 0);
			udelay(30);
		}
		gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 1);

		raw_local_irq_restore(flags);

	} else {
		gpio_set_val(GPIO_PROD_LCD_BL_BRIGHTNESS_ID, 0);
	}

	mdelay(1);
}

static int sc8800sfb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct sc8800sfb_info *sc8800sfb;
	int32_t ret;

	FB_PRINT("@fool2[%s]\n", __FUNCTION__);
	printk("initializing sc8800 lcm0 fb\n");
	
	fb = framebuffer_alloc(sizeof(struct sc8800sfb_info), &pdev->dev);
	if (!fb)
		return -ENOMEM;
	sc8800sfb = fb->par;
	sc8800sfb->fb = fb;
	sc8800sfb->ops = &lcm_mcu_ops;

	ret = mount_panel(sc8800sfb, &lcd_panel);
	if (ret) {
		printk(KERN_ERR "unsupported panel!!");
		return -EFAULT;
	}

	ret = setup_fbmem(sc8800sfb, pdev);
	if (ret)
		return ret;

	setup_fb_info(sc8800sfb);

	ret = register_framebuffer(fb);
	if (ret) {
		framebuffer_release(sc8800sfb->fb);
		return ret;
	}

	hw_init(sc8800sfb);

	//set_lcdc_block(&sc8800sfb->fb->var, sc8800sfb->fb); /* TEMP */

	/* FIXME: put the BL stuff to where it belongs. */
	set_backlight(50);

if(0){ /* in-kernel test code */
	struct fb_info test_info;
	int size = sc8800sfb->fb->var.xres * sc8800sfb->fb->var.yres *2;
	
	/* set color */
	memset(sc8800sfb->fb->screen_base, 0x1234, size);
	memset(sc8800sfb->fb->screen_base+size, 0x4321, size);

	/* pan display */
	test_info = *sc8800sfb->fb;

	while(1) {
		if (test_info.var.yoffset == 0)
			test_info.var.yoffset = sc8800sfb->fb->var.yres;
		else
			test_info.var.yoffset = 0;

		//sc8800sfb_pan_display(&sc8800sfb->fb->var, &test_info);
		real_pan_display(&sc8800sfb->fb->var, &test_info);
		msleep(500);
	}
}

	return 0;
}

static struct platform_driver sc8800sfb_driver = {
	.probe = sc8800sfb_probe,
	.driver = {
		.name = "sc8800sfb",
		.owner = THIS_MODULE,
	},
};

static int __init sc8800sfb_init(void)
{
	FB_PRINT("@fool2[%s]\n", __FUNCTION__);
	return platform_driver_register(&sc8800sfb_driver);
}

module_init(sc8800sfb_init);
