/* drivers/video/sc8800g/sc8800g_copybit_lcdc.c
 *
 * copybit alpha blending/blit driver based on sc8800g lcdc
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

#include "sc8800g_lcdc_manager.h"
#include "sc8800g_copybit_lcdc.h"
#include <mach/hardware.h>

#define LCDC_OSD2_CTRL          (SPRD_LCDC_BASE + 0x0070)
#define LCDC_OSD2_BASE_ADDR     (SPRD_LCDC_BASE + 0x0074)
#define LCDC_OSD2_SIZE_XY       (SPRD_LCDC_BASE + 0x0078)
#define LCDC_OSD2_PITCH         (SPRD_LCDC_BASE + 0x007c)
#define LCDC_OSD2_DISP_XY       (SPRD_LCDC_BASE + 0x0080)
#define LCDC_OSD2_ALPHA         (SPRD_LCDC_BASE + 0x0084)
#define LCDC_OSD2_GREY_RGB      (SPRD_LCDC_BASE + 0x0088)
#define LCDC_OSD2_CK            (SPRD_LCDC_BASE + 0x008c)

#define LCDC_OSD3_CTRL          (SPRD_LCDC_BASE + 0x0090)
#define LCDC_OSD3_BASE_ADDR     (SPRD_LCDC_BASE + 0x0094)
#define LCDC_OSD3_SIZE_XY       (SPRD_LCDC_BASE + 0x0098)
#define LCDC_OSD3_PITCH         (SPRD_LCDC_BASE + 0x009c)
#define LCDC_OSD3_DISP_XY       (SPRD_LCDC_BASE + 0x00a0)
#define LCDC_OSD3_ALPHA         (SPRD_LCDC_BASE + 0x00a4)
#define LCDC_OSD3_GREY_RGB      (SPRD_LCDC_BASE + 0x00a8)
#define LCDC_OSD3_CK            (SPRD_LCDC_BASE + 0x00ac)

/* offset defines for OSDx (x>1) */
#define CTRL_OFFSET          (0x00)
#define BASE_ADDR_OFFSET     (0x04)
#define SIZE_XY_OFFSET       (0x08)
#define PITCH_OFFSET         (0x0c)
#define DISP_XY_OFFSET       (0x10)
#define ALPHA_OFFSET         (0x14)
#define GREY_RGB_OFFSET      (0x18)
#define CK_OFFSET            (0x1c)

#define OSD_DEF_CONFIG          (1) /* layer enabled; color key disabled */ 
#define OSD_BLK_ALPHA           (1<<2) 
#define OSD_PIXEL_ALPHA         (0<<2) 
#define OSD_FMT_RGB888          (3<<3) 
#define OSD_FMT_RGB666          (4<<3) 
#define OSD_FMT_RGB565          (5<<3) 
#define OSD_FMT_RGB555          (6<<3) 
#define OSD_FMT_GREY            (7<<3) 
#define OSD_BSWT_0123           (0<<7) 
#define OSD_BSWT_3210           (1<<7) 
#define OSD_BSWT_2301           (2<<7) 

/* capture */
#define LCDC_CAP_CTRL           (SPRD_LCDC_BASE + 0x00f0)
#define LCDC_CAP_BASE_ADDR      (SPRD_LCDC_BASE + 0x00f4)
#define LCDC_CAP_START_XY       (SPRD_LCDC_BASE + 0x00f8)
#define LCDC_CAP_SIZE_XY        (SPRD_LCDC_BASE + 0x00fc)
#define LCDC_CAP_PITCH          (SPRD_LCDC_BASE + 0x0100)

#define CAP_ENABLE              (1)
#define CAP_FMT_RGB888          (0<<1) 
#define CAP_FMT_RGB666          (1<<1) 
#define CAP_FMT_RGB565          (2<<1) 
#define CAP_BSWT_0123           (0<<3) 
#define CAP_BSWT_3210           (1<<3) 
#define CAP_BSWT_2301           (2<<3) 

#define LCDC_CTRL               (SPRD_LCDC_BASE + 0x0000)
#define LCDC_DISP_SIZE          (SPRD_LCDC_BASE + 0x0004)
#define LCDC_LCM_START          (SPRD_LCDC_BASE + 0x0008)
#define LCDC_LCM_SIZE           (SPRD_LCDC_BASE + 0x000c)
#define LCDC_BG_COLOR           (SPRD_LCDC_BASE + 0x0010)
#define LCDC_FIFO_STATUS        (SPRD_LCDC_BASE + 0x0014)

#define LCDC_IRQ_EN             (SPRD_LCDC_BASE + 0x0120)
#define LCDC_IRQ_CLR            (SPRD_LCDC_BASE + 0x0124)
#define LCDC_IRQ_STATUS         (SPRD_LCDC_BASE + 0x0128)
#define LCDC_IRQ_RAW            (SPRD_LCDC_BASE + 0x012c)

//#define COPYBIT_LCDC_DEBUG
#ifdef COPYBIT_LCDC_DEBUG
#define CL_PRINT printk
#else
#define CL_PRINT(...)
#endif

#define SYNC_BLIT
struct semaphore copybit_wait; /* TEMP */

static inline void do_capture(void)
{
CL_PRINT("===do_capture===\n");
CL_PRINT("  LCDC_OSD2_CTRL:      0x%x\n", *(unsigned int *)LCDC_OSD2_CTRL);
CL_PRINT("  LCDC_OSD2_BASE_ADDR: 0x%x\n", *(unsigned int *)LCDC_OSD2_BASE_ADDR);
CL_PRINT("  LCDC_OSD2_SIZE_XY:   0x%x\n", *(unsigned int *)LCDC_OSD2_SIZE_XY);
CL_PRINT("  LCDC_OSD2_PITCH:     0x%x\n", *(unsigned int *)LCDC_OSD2_PITCH);
CL_PRINT("  LCDC_OSD2_DISP_XY:   0x%x\n", *(unsigned int *)LCDC_OSD2_DISP_XY);
CL_PRINT("  LCDC_OSD2_ALPHA:     0x%x\n", *(unsigned int *)LCDC_OSD2_ALPHA);
CL_PRINT("  LCDC_OSD2_GREY_RGB:  0x%x\n", *(unsigned int *)LCDC_OSD2_GREY_RGB);
CL_PRINT("  LCDC_OSD2_CK:        0x%x\n", *(unsigned int *)LCDC_OSD2_CK);

CL_PRINT("  LCDC_OSD3_CTRL:      0x%x\n", *(unsigned int *)LCDC_OSD3_CTRL);
CL_PRINT("  LCDC_OSD3_BASE_ADDR: 0x%x\n", *(unsigned int *)LCDC_OSD3_BASE_ADDR);
CL_PRINT("  LCDC_OSD3_SIZE_XY:   0x%x\n", *(unsigned int *)LCDC_OSD3_SIZE_XY);
CL_PRINT("  LCDC_OSD3_PITCH:     0x%x\n", *(unsigned int *)LCDC_OSD3_PITCH);
CL_PRINT("  LCDC_OSD3_DISP_XY:   0x%x\n", *(unsigned int *)LCDC_OSD3_DISP_XY);
CL_PRINT("  LCDC_OSD3_ALPHA:     0x%x\n", *(unsigned int *)LCDC_OSD3_ALPHA);
CL_PRINT("  LCDC_OSD3_GREY_RGB:  0x%x\n", *(unsigned int *)LCDC_OSD3_GREY_RGB);
CL_PRINT("  LCDC_OSD3_CK:        0x%x\n", *(unsigned int *)LCDC_OSD3_CK);

CL_PRINT("  LCDC_CAP_CTRL:       0x%x\n", *(unsigned int *)LCDC_CAP_CTRL);
CL_PRINT("  LCDC_CAP_BASE_ADDR:  0x%x\n", *(unsigned int *)LCDC_CAP_BASE_ADDR);
CL_PRINT("  LCDC_CAP_START_XY:   0x%x\n", *(unsigned int *)LCDC_CAP_START_XY);
CL_PRINT("  LCDC_CAP_SIZE_XY:    0x%x\n", *(unsigned int *)LCDC_CAP_SIZE_XY);
CL_PRINT("  LCDC_CAP_PITCH:      0x%x\n", *(unsigned int *)LCDC_CAP_PITCH);

CL_PRINT("  LCDC_CTRL:           0x%x\n", *(unsigned int *)LCDC_CTRL);
CL_PRINT("  LCDC_DISP_SIZE:      0x%x\n", *(unsigned int *)LCDC_DISP_SIZE);
CL_PRINT("  LCDC_LCM_START:      0x%x\n", *(unsigned int *)LCDC_LCM_START);
CL_PRINT("  LCDC_LCM_SIZE:       0x%x\n", *(unsigned int *)LCDC_LCM_SIZE);
CL_PRINT("  LCDC_FIFO_STATUS:    0x%x\n", *(unsigned int *)LCDC_FIFO_STATUS);

	__raw_bits_or((1<<3), LCDC_CTRL);

#ifdef SYNC_BLIT
//printk("copybit: wait...");
	while((__raw_readl(LCDC_IRQ_RAW) & (1<<0)) == 0);
	__raw_bits_or((1<<0), LCDC_IRQ_CLR);
//printk("done\n");
#else
	down(&copybit_wait); /* TEMP */
#endif

	/* disable capture */
CL_PRINT("%s disable capture (1): %x\n", 
__FUNCTION__, *(unsigned int*)LCDC_CAP_CTRL);
	__raw_writel(0, LCDC_CAP_CTRL);
CL_PRINT("%s disable capture (2): %x\n", 
__FUNCTION__, *(unsigned int*)LCDC_CAP_CTRL);
}

static inline int set_capture(struct s2d_img *img, struct s2d_rect *rect)
{
	int byte_per_pixel;
	unsigned int reg_val;

	CL_PRINT("===set_capture===\n"
	       "img->format %d\n"
	       "img->width  %d\n"
	       "img->height %d\n"
	       "img->base   0x%x\n"
	       "rect->x     %d\n"
	       "rect->y     %d\n"
	       "rect->w     %d\n"
	       "rect->h     %d\n",
	       img->format, img->width, img->height, img->base,
	       rect->x, rect->y, rect->w, rect->h
	       );
	switch (img->format) {
		case S2D_ARGB_8888:
			byte_per_pixel = 4;
			reg_val = (CAP_ENABLE | CAP_FMT_RGB888); 
			break;
		case S2D_BGRA_8888:
			byte_per_pixel = 4;
			reg_val = (CAP_ENABLE | CAP_FMT_RGB888 |
				CAP_BSWT_3210); 
			break;
		case S2D_RGB_565:
			byte_per_pixel = 2;
			reg_val = (CAP_ENABLE | CAP_FMT_RGB565 |
				 CAP_BSWT_2301); 
			break;
		default:
			printk(KERN_ERR "S2D: Unsupported Format(%d)!!\n",
			img->format);
			return -1;
			break;
	}

	/* set control register */
	__raw_writel(reg_val, LCDC_CAP_CTRL);

	/* base */
	reg_val = img->base + byte_per_pixel * (img->width*rect->y + rect->x);
	__raw_writel(reg_val>>2, LCDC_CAP_BASE_ADDR);

	/* size*/
	reg_val = ( rect->w & 0x3ff) | (( rect->h & 0x3ff )<<16);
	__raw_writel(reg_val, LCDC_CAP_SIZE_XY);

	/* pitch*/
	reg_val = ( img->width & 0x3ff) ;
	__raw_writel(reg_val, LCDC_CAP_PITCH);

	return 0;
}

/* not working for image/OSD1 layer */
static inline int set_fetch(struct s2d_img *img, struct s2d_rect *rect, 
			unsigned int reg)
{
	int byte_per_pixel;
	unsigned int reg_val;

	CL_PRINT("===set_fetch===\n"
	       "img->format %d\n"
	       "img->width  %d\n"
	       "img->height %d\n"
	       "img->base   0x%x\n"
	       "rect->x     %d\n"
	       "rect->y     %d\n"
	       "rect->w     %d\n"
	       "rect->h     %d\n",
	       img->format, img->width, img->height, img->base,
	       rect->x, rect->y, rect->w, rect->h
	       );
	switch (img->format) {
		case S2D_ARGB_8888:
			byte_per_pixel = 4;
			reg_val = (OSD_DEF_CONFIG | OSD_FMT_RGB888); 
			break;
		case S2D_BGRA_8888:
			byte_per_pixel = 4;
			reg_val = (OSD_DEF_CONFIG | OSD_FMT_RGB888 |
				OSD_BSWT_3210); 
			break;
		case S2D_RGB_565:
			byte_per_pixel = 2;
			reg_val = (OSD_DEF_CONFIG | OSD_FMT_RGB565 |
				 OSD_BLK_ALPHA | OSD_BSWT_2301); 
			break;
		default:
			printk(KERN_ERR "S2D: Unsupported Format(%d)!!\n",
			img->format);
			return -1;
			break;
	}

	/* set control register */
	__raw_writel(reg_val, (reg + CTRL_OFFSET));

	/* base */
	reg_val = img->base + byte_per_pixel * (img->width*rect->y + rect->x);
	__raw_writel(reg_val>>2, (reg + BASE_ADDR_OFFSET));

	/* layer size*/
	reg_val = ( rect->w & 0x3ff) | (( rect->h & 0x3ff )<<16);
	__raw_writel(reg_val, (reg + SIZE_XY_OFFSET));

	/* layer pitch*/
	reg_val = ( img->width & 0x3ff) ;
	__raw_writel(reg_val, (reg + PITCH_OFFSET));

	return 0;
}

static inline int para_check(struct s2d_blit_req * req)
{
	if (req->src.format == S2D_RGB_565)
		req->src_rect.w = (req->src_rect.w+1) & ~1;
	if (req->dst.format == S2D_RGB_565)
		req->dst_rect.w = (req->dst_rect.w+1) & ~1;
	return 0;
}

int do_copybit_lcdc(struct s2d_blit_req * req)
{
	int err = 0;

	CL_PRINT("src.format %d\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n"
	       "dst_rect   x %d, y %d, w %d, h %d\n",
	       req->src.format, req->src.width, req->src.height, req->src.base,
	       req->src_rect.x, req->src_rect.y, 
	       req->src_rect.w, req->src_rect.h, 
	       req->dst.format, req->dst.width, req->dst.height, req->dst.base,
	       req->dst_rect.x, req->dst_rect.y, 
	       req->dst_rect.w, req->dst_rect.h 
	       );

	para_check(req); /* to fulfill the alignment restriction */

	if (req->alpha == 255 && req->src.format == S2D_RGB_565) {
		CL_PRINT("%s [%d]\n", __FILE__, __LINE__);
		/* we use OSD2 for this memcpy like blit */
		lm_acquire(LID_OSD2);

		/* disable lcdc done irq */
		__raw_bits_and(~(1<<0), LCDC_IRQ_EN);

		/* set layer */
		if(set_fetch(&req->src, &req->src_rect, LCDC_OSD2_CTRL)) {
			err = -1;
			goto __out1;
		}
		

		/* set capture */
		if(set_capture(&req->dst, &req->dst_rect)) {
			err = -1;
			goto __out1;
		}

		/* just do it... */
		do_capture();

		/* enable lcdc done irq */
		__raw_bits_or((1<<0), LCDC_IRQ_EN);

__out1:
		lm_release(LID_OSD2);
		CL_PRINT("%s [%d]\n", __FILE__, __LINE__);

	} else {
		CL_PRINT("%s [%d]\n", __FILE__, __LINE__);
		lm_acquire(LID_OSD2);
		lm_acquire(LID_OSD3);
		lm_enable_layer(LID_OSD3);

#ifdef SYNC_BLIT
		/* disable lcdc done irq */
		__raw_bits_and(~(1<<0), LCDC_IRQ_EN);
#endif

		/* set top layer */
		if (set_fetch(&req->src, &req->src_rect, LCDC_OSD3_CTRL)) {
			err = -1;
			goto __out2;
		}

		/* set bottom layer */
		if (set_fetch(&req->dst, &req->dst_rect, LCDC_OSD2_CTRL)) {
			err = -1;
			goto __out2;
		}

		if (req->src.format == S2D_RGB_565) {
			/* set alpha value*/
			__raw_writel(req->alpha, LCDC_OSD3_ALPHA);
		}

		/* set capture */
		if (set_capture(&req->dst, &req->dst_rect)) {
			err = -1;
			goto __out2;
		}
		CL_PRINT("%s [%d]\n", __FILE__, __LINE__);

		/* just do it... */
		do_capture();

#ifdef SYNC_BLIT
		/* enable lcdc done irq */
		__raw_bits_or((1<<0), LCDC_IRQ_EN);
#endif

__out2:
		lm_disable_layer(LID_OSD3);
		lm_release(LID_OSD2);
		lm_release(LID_OSD3);
		CL_PRINT("%s [%d]\n", __FILE__, __LINE__);
	}
	return err;
}

/* TEMP */
extern int enable_layer(int id);
extern int disable_layer(int id);

int copybit_lcdc_init(void)
{
	/* FIXME: hardcoded layer num */
	lm_register_layer(LID_OSD2, LMODE_CAPTURE, enable_layer, disable_layer);
	lm_register_layer(LID_OSD3, LMODE_CAPTURE, enable_layer, disable_layer);
	lm_enable_layer(LID_OSD2);

	/* disable color key */
	__raw_bits_and(~(1<<0), LCDC_OSD2_CTRL);
	__raw_bits_and(~(1<<0), LCDC_OSD3_CTRL);

	/* layer start position */
	__raw_writel(0, LCDC_OSD2_DISP_XY);
	__raw_writel(0, LCDC_OSD3_DISP_XY);
	__raw_writel(0, LCDC_CAP_START_XY);

	/* OSD2 alpha */
	__raw_writel(0xff, LCDC_OSD2_ALPHA);
#ifndef SYNC_BLIT
	sema_init(&copybit_wait, 0);
#endif
	return 0;
}

int copybit_lcdc_exit(void)
{
	/* FIXME: hardcoded layer num */
	lm_disable_layer(LID_OSD2);
	lm_disable_layer(LID_OSD3);
	lm_unregister_layer(LID_OSD2);
	lm_unregister_layer(LID_OSD3);
	return 0;
}
