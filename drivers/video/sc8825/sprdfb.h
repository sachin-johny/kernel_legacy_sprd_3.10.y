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

#ifndef _SPRDFB_H_
#define _SPRDFB_H_

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

enum{
	SPRDFB_PANEL_IF_DBI = 0,
	SPRDFB_PANEL_IF_DPI,
	SPRDFB_PANEL_IF_EDPI,
	SPRDFB_PANEL_IF_LIMIT
};


enum{
	MCU_LCD_REGISTER_TIMING = 0,
	MCU_LCD_GRAM_TIMING,
	MCU_LCD_TIMING_KIND_MAX
};

enum{
	RGB_LCD_H_TIMING = 0,
	RGB_LCD_V_TIMING,
	RGB_LCD_TIMING_KIND_MAX
};

struct sprdfb_device {
	struct fb_info	*fb;

	uint16_t		enable;
	uint16_t	 	dev_id; /*main_lcd, sub_lcd*/

	uint32_t	 	bpp;  /*input bit per pixel*/

	uint16_t		panel_ready; /*panel has been inited by uboot*/
	uint16_t		panel_if_type; /*panel IF*/

	union{
		uint32_t	mcu_timing[MCU_LCD_TIMING_KIND_MAX];
		uint32_t	rgb_timing[RGB_LCD_TIMING_KIND_MAX];
	}panel_timing;

	struct panel_spec	*panel;
	struct display_ctrl	*ctrl;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

struct display_ctrl {
	const char	*name;

	int32_t	(*early_init)	  (struct sprdfb_device *dev);
	int32_t	(*init)		  (struct sprdfb_device *dev);
	int32_t	(*uninit)		  (struct sprdfb_device *dev);

	int32_t 	(*refresh)	  (struct sprdfb_device *dev);

	int32_t	(*suspend)	  (struct sprdfb_device *dev);
	int32_t 	(*resume)	  (struct sprdfb_device *dev);
};


#endif
