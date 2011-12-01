/* drivers/video/sc8800g/sc8800g_lcd.h
 *
 * Spreadtrum LCD abstraction
 *
 * Copyright (C) 2010 Spreadtrum.com
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

#ifndef _LCDC_REG_H_
#define _LCDC_REG_H_

#define BIT20 (1<<20)

#define AHB_CTL0                (SPRD_AHB_BASE + 0x200)
#define AHB_SOFT_RST            (SPRD_AHB_BASE + 0x210)
#define AHB_ARM_CLK             (SPRD_AHB_BASE + 0x224)

#define GR_PLL_SRC              (SPRD_GREG_BASE + 0x70)
#define GR_GEN4                 (SPRD_GREG_BASE + 0x60)

#define LCDC_CTRL               (SPRD_LCDC_BASE + 0x0000)
#define LCDC_DISP_SIZE          (SPRD_LCDC_BASE + 0x0004)
#define LCDC_LCM_START          (SPRD_LCDC_BASE + 0x0008)
#define LCDC_LCM_SIZE           (SPRD_LCDC_BASE + 0x000c)
#define LCDC_BG_COLOR           (SPRD_LCDC_BASE + 0x0010)
#define LCDC_FIFO_STATUS        (SPRD_LCDC_BASE + 0x0014)

#define LCDC_IMG_CTRL           (SPRD_LCDC_BASE + 0x0020)
#define LCDC_IMG_Y_BASE_ADDR    (SPRD_LCDC_BASE + 0x0024)
#define LCDC_IMG_UV_BASE_ADDR   (SPRD_LCDC_BASE + 0x0028)
#define LCDC_IMG_SIZE_XY        (SPRD_LCDC_BASE + 0x002c)
#define LCDC_IMG_PITCH          (SPRD_LCDC_BASE + 0x0030)
#define LCDC_IMG_DISP_XY        (SPRD_LCDC_BASE + 0x0034)

#define LCDC_OSD1_CTRL          (SPRD_LCDC_BASE + 0x0050)  
#define LCDC_OSD2_CTRL          (SPRD_LCDC_BASE + 0x0080)  
#define LCDC_OSD3_CTRL          (SPRD_LCDC_BASE + 0x00b0)  
#define LCDC_OSD4_CTRL          (SPRD_LCDC_BASE + 0x00e0)  
#define LCDC_OSD5_CTRL          (SPRD_LCDC_BASE + 0x0110)  

#define LCDC_OSD1_BASE_ADDR       (SPRD_LCDC_BASE + 0x0054) 
#define LCDC_OSD1_ALPHA_BASE_ADDR (SPRD_LCDC_BASE + 0x0058) 
#define LCDC_OSD1_SIZE_XY         (SPRD_LCDC_BASE + 0x005c) 
#define LCDC_OSD1_PITCH           (SPRD_LCDC_BASE + 0x0060) 
#define LCDC_OSD1_DISP_XY         (SPRD_LCDC_BASE + 0x0064)  
#define LCDC_OSD1_ALPHA           (SPRD_LCDC_BASE + 0x0068) 
#define LCDC_OSD1_GREY_RGB        (SPRD_LCDC_BASE + 0x006c) 
#define LCDC_OSD1_CK              (SPRD_LCDC_BASE + 0x0070) 

#define LCDC_IRQ_EN             (SPRD_LCDC_BASE + 0x0170)  
#define LCDC_IRQ_CLR            (SPRD_LCDC_BASE + 0x0174)  
#define LCDC_IRQ_STATUS         (SPRD_LCDC_BASE + 0x0178)  
#define LCDC_IRQ_RAW            (SPRD_LCDC_BASE + 0x017c)  

#define LCM_CTRL                (SPRD_LCDC_BASE + 0x0180) 
#define LCM_PARAMETER0          (SPRD_LCDC_BASE + 0x0184) 
#define LCM_PARAMETER1          (SPRD_LCDC_BASE + 0x0188) 
#define LCM_RDDATA              (SPRD_LCDC_BASE + 0x018c) 
#define LCM_RSTN                (SPRD_LCDC_BASE + 0x0190) 
#define LCM_CD0                 (SPRD_LCDC_BASE + 0x01A0) 
#define LCM_DATA0               (SPRD_LCDC_BASE + 0x01A4) 

#endif
