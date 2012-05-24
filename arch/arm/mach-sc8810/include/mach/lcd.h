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

#ifndef _LCD_H_
#define _LCD_H_

#include <linux/types.h>
#include <linux/delay.h>

#define LCD_DelayMS  msleep

/* LCD mode */
#define LCD_MODE_RGB 0
#define LCD_MODE_MCU 1

/* bus mode */
#define LCD_BUS_8080 0
#define LCD_BUS_6800 1
#define LCD_BUS_SPI  2

/* lcd directions */
#define LCD_DIRECT_NORMAL  0
#define LCD_DIRECT_ROT_90  1
#define LCD_DIRECT_ROT_180 2
#define LCD_DIRECT_ROT_270 3
#define LCD_DIRECT_MIR_H   4
#define LCD_DIRECT_MIR_V   5
#define LCD_DIRECT_MIR_HV  6

/* lcdc refresh, TE on, double timing, partial update*/
#define LCD_CAP_NORMAL               0x0

/* only not support partial update*/
#define LCD_CAP_NOT_PARTIAL_UPDATE   0x1

/* write register, grame have the same timing */
#define LCD_CAP_UNIQUE_TIMING        0x2

/* lcd not support TE */
#define LCD_CAP_NOT_TEAR_SYNC        0x4

/* only do command/data register, such as some mono oled display device */
#define LCD_CAP_MANUAL_REFRESH       0x8




enum LCD_TIMING {
	LCD_REGISTER_TIMING = 0,
	LCD_GRAM_TIMING,
};

struct lcd_spec;

/* LCD operations */
struct lcd_operations {
	int32_t (*lcd_init)(struct lcd_spec *self);
	int32_t (*lcd_close)(struct lcd_spec *self);
	int32_t (*lcd_reset)(struct lcd_spec *self);
	int32_t (*lcd_enter_sleep)(struct lcd_spec *self, uint8_t is_sleep);
	int32_t (*lcd_set_contrast)(struct lcd_spec *self, uint16_t contrast);
	int32_t (*lcd_set_brightness)(struct lcd_spec *self,
				uint16_t brightness);
	int32_t (*lcd_set_window)(struct lcd_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);
	int32_t (*lcd_invalidate)(struct lcd_spec *self);
	int32_t (*lcd_invalidate_rect)(struct lcd_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);
	int32_t (*lcd_rotate_invalidate_rect)(struct lcd_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom,
				uint16_t angle);
	int32_t (*lcd_set_direction)(struct lcd_spec *self, uint16_t direction);
	uint32_t (*lcd_readid)(struct lcd_spec *self);
};

/* RGB LCD specific properties */
struct timing_rgb {
	uint16_t hfp;
	uint16_t hbp;
	uint16_t hsync;
	uint16_t vfp;
	uint16_t vbp;
	uint16_t vsync;
};

struct ops_rgb {
	int32_t (*send_cmd)(uint32_t cmd);
	int32_t (*send_cmd_data)(uint32_t cmd, uint32_t data);
};

struct info_rgb {
	/* under construction... */
	struct timing_rgb timing;
	struct ops_rgb *ops;
};

/* MCU LCD specific properties */
struct timing_mcu {
	uint16_t rcss;
	uint16_t rlpw;
	uint16_t rhpw;
	uint16_t wcss;
	uint16_t wlpw;
	uint16_t whpw;
};

typedef int32_t (*Send_cmd)(uint32_t data);
typedef int32_t (*Send_data)(uint32_t data);
typedef int32_t (*Send_cmd_data)(uint32_t cmd, uint32_t data);
typedef uint32_t (*Read_data)(void);

struct ops_mcu {
	int32_t (*send_cmd)(uint32_t cmd);
	int32_t (*send_cmd_data)(uint32_t cmd, uint32_t data);
	int32_t (*send_data)(uint32_t data);
	uint32_t (*read_data)(void);
};

struct info_mcu {
	uint16_t bus_mode;
	uint16_t bus_width;
	struct timing_mcu *timing;
	struct ops_mcu *ops;
};

/* LCD abstraction */
struct lcd_spec {
	uint32_t cap;
	uint16_t width;
	uint16_t height;
	uint16_t mode;
	uint16_t direction;

	/* for some panel, such as mono oled ,the driver may read 
the frame buffer's content and do not use the lcdc refresh */
	char*    screen_base;
	uint32_t bpp;
	uint32_t mem_len;

	union {
		struct info_rgb *rgb;
		struct info_mcu *mcu;
	} info;
	struct lcd_operations *ops;
};

struct lcd_panel_cfg{ 
	uint32_t lcd_cs;
	uint32_t lcd_id;
	struct   lcd_spec* panel;
};

struct  sprd_lcd_platform_data {
	struct lcd_panel_cfg  *lcd_panel_ptr;
	uint32_t         lcd_panel_size;
};
#endif
