/* Copyright (C) 2010 Spreadtrum
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

#include <linux/kernel.h>
#include <mach/lcd.h>

#define  LCD_DEBUG

#ifdef LCD_DEBUG

#define LCD_PRINT printk

#else

#define LCD_PRINT(...)

#endif

static int32_t dummy_init(struct lcd_spec *self)
{

	return 0;
}

static int32_t dummy_invalidate(struct lcd_spec *self)
{

	return 0;
}

static int32_t dummy_enter_sleep(struct lcd_spec *self, uint8_t is_sleep)
{

	return ;
}

static int32_t dummy_read_id(struct lcd_spec *self)
{
	return 0; /* 'sprd' */
}

static struct lcd_operations lcd_dummy_operations = {
	.lcd_init            = dummy_init,
	.lcd_enter_sleep     = dummy_enter_sleep,
	.lcd_readid          = dummy_read_id,
	.lcd_invalidate      = dummy_invalidate,	
};

static struct timing_mcu lcd_dummy_timing = {
		.rcss = 10,  
		.rlpw = 120,
		.rhpw = 60,
		.wcss = 20,
		.wlpw = 100,
		.whpw = 100,
};


static struct info_mcu lcd_dummy_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 1,
	.timing = &lcd_dummy_timing,
	.ops = NULL,
};

struct lcd_spec lcd_panel_dummy = {
	.cap   = LCD_CAP_MANUAL_REFRESH | LCD_CAP_NOT_TEAR_SYNC 
			| LCD_CAP_UNIQUE_TIMING | LCD_CAP_NOT_PARTIAL_UPDATE,
	.width = 128,
	.height = 36,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_dummy_info},
	.ops = &lcd_dummy_operations,
};



