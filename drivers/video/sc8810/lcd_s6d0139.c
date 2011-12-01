/* drivers/video/sc8800g/sc8800g_lcd_s6d0139.c
 *
 * Support for s6d0139 LCD device
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

#include <linux/kernel.h>
#include <linux/delay.h>

#include "lcd.h"

//#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

static int32_t s6d0139_init(struct lcd_spec *self)
{
	LCD_PRINT("s6d0139_init\n");

	/* reset the lcd */
	self->ops->lcd_reset(self);
	mdelay(100);
	/* start init sequence */
	self->info.mcu->ops->send_cmd_data(0x0000, 0x0001);
	mdelay(10);
	
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0011, 0x2604);
	self->info.mcu->ops->send_cmd_data(0x0014, 0x1212);
	self->info.mcu->ops->send_cmd_data(0x0010, 0x3C00);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x0040);
	mdelay(10);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x0060);
	mdelay(50);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x0070);
	mdelay(40);
	self->info.mcu->ops->send_cmd_data(0x0001, 0x0127);	
    self->info.mcu->ops->send_cmd_data(0x0002, 0x0700);
	self->info.mcu->ops->send_cmd_data(0x0003, 0x1030);
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0008, 0x0404);
	self->info.mcu->ops->send_cmd_data(0x000B, 0x0200);
	self->info.mcu->ops->send_cmd_data(0x000C, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0015, 0x0000);

   //gamma setting
	self->info.mcu->ops->send_cmd_data(0x0030, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0031, 0x0606);
	self->info.mcu->ops->send_cmd_data(0x0032, 0x0006);
	self->info.mcu->ops->send_cmd_data(0x0033, 0x0403);
	self->info.mcu->ops->send_cmd_data(0x0034, 0x0107);
	self->info.mcu->ops->send_cmd_data(0x0035, 0x0101);
	self->info.mcu->ops->send_cmd_data(0x0036, 0x0707);
	self->info.mcu->ops->send_cmd_data(0x0037, 0x0304);
	self->info.mcu->ops->send_cmd_data(0x0038, 0x0A00);
	self->info.mcu->ops->send_cmd_data(0x0039, 0x0706);
	self->info.mcu->ops->send_cmd_data(0x0040, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0041, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0042, 0x013F);
	self->info.mcu->ops->send_cmd_data(0x0043, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0044, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0045, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0046, 0xEF00);
	self->info.mcu->ops->send_cmd_data(0x0047, 0x013F);
	self->info.mcu->ops->send_cmd_data(0x0048, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0011);
    mdelay(40);
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0017);
	self->info.mcu->ops->send_cmd_data(0x0020, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0021, 0x0000);
	mdelay(40);

	self->info.mcu->ops->send_cmd(0x0022);
	
	
  if(0){
	int i;
	for (i=0; i<320*240/3; i++)
		self->info.mcu->ops->send_data(0xf800);
	for (i=0; i< 320*240/3; i++)
		self->info.mcu->ops->send_data(0x07e0);
       	for (i=0; i< 320*240/3; i++)
		self->info.mcu->ops->send_data(0x001f);
    }

	return 0;
}

static int32_t s6d0139_set_window(struct lcd_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	LCD_PRINT("s6d0139_set_window\n");

	self->info.mcu->ops->send_cmd_data(0x46, ((right << 8) & 0xff00)|(left&0xff));
	self->info.mcu->ops->send_cmd_data(0x47, bottom);
	self->info.mcu->ops->send_cmd_data(0x48, top);

	switch (self->direction) {
	case LCD_DIRECT_NORMAL:
	case LCD_DIRECT_ROT_270:
		self->info.mcu->ops->send_cmd_data(0x0020, left);
		self->info.mcu->ops->send_cmd_data(0x0021, top);
		break;
	case LCD_DIRECT_ROT_90:
	case LCD_DIRECT_MIR_H:
		self->info.mcu->ops->send_cmd_data(0x0020, right);
		self->info.mcu->ops->send_cmd_data(0x0021, top);
		break;
	case LCD_DIRECT_ROT_180:
	case LCD_DIRECT_MIR_HV:
		self->info.mcu->ops->send_cmd_data(0x0020, right);
		self->info.mcu->ops->send_cmd_data(0x0021, bottom);
		break;
	case LCD_DIRECT_MIR_V:
		self->info.mcu->ops->send_cmd_data(0x0020, left);
		self->info.mcu->ops->send_cmd_data(0x0021, bottom);
		break;
	default:
		LCD_PRINT("unknown lcd direction!\n");
		break;
	}
	
	self->info.mcu->ops->send_cmd(0x0022);

	return 0;
}

static int32_t s6d0139_invalidate(struct lcd_spec *self)
{
	LCD_PRINT("s6d0139_invalidate\n");

	return self->ops->lcd_set_window(self, 0, 0, 
			self->width-1, self->height-1);
	
}

static int32_t s6d0139_set_direction(struct lcd_spec *self, uint16_t direction)
{
	LCD_PRINT("s6d0139_set_direction\n");

	switch (direction) {
	case LCD_DIRECT_NORMAL:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1030);
		break;
	case LCD_DIRECT_ROT_90:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1028);
		break;
	case LCD_DIRECT_ROT_180:
	case LCD_DIRECT_MIR_HV:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1000);
		break;
	case LCD_DIRECT_ROT_270:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1018);
		break;
	case LCD_DIRECT_MIR_H:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1020);
		break;
	case LCD_DIRECT_MIR_V:
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1010);
		break;
	default:
		LCD_PRINT("unknown lcd direction!\n");
		self->info.mcu->ops->send_cmd_data(0x0003, 0x1030);
		direction = LCD_DIRECT_NORMAL;
		break;
	}

	self->direction = direction;
	
	return 0;
}

static struct lcd_operations lcd_s6d0139_operations = {
	.lcd_init = s6d0139_init,
	.lcd_set_window = s6d0139_set_window,
	.lcd_invalidate = s6d0139_invalidate,
	.lcd_set_direction = s6d0139_set_direction,
};

static struct timing_mcu lcd_s6d0139_timing = {
	.rcss = 15,
	.rlpw = 250,
	.rhpw = 250,
	.wcss = 10,
	.wlpw = 28,
	.whpw = 28,
};

static struct info_mcu lcd_s6d0139_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 16,
	.timing = &lcd_s6d0139_timing,
	.ops = NULL,
};

struct lcd_spec lcd_panel = {
	.width = 240,
	.height = 320,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_s6d0139_info},
	.ops = &lcd_s6d0139_operations,
};

