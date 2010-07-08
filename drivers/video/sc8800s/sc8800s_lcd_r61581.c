/* drivers/video/msm_fb/mddi_client_dummy.c
 *
 * Support for R61581 LCD device
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

#include "sc8800s_lcd.h"

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

static int32_t r61581_init(struct lcd_spec *self)
{
	LCD_PRINT("r61581_init\n");

	/* reset the lcd */
	/*
	self->ops->lcd_reset(self);
	mdelay(50);
	*/

	mdelay(10);

	self->info.mcu->ops->send_cmd(0xb0);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0xb3);
	self->info.mcu->ops->send_data(0x02);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0xb4);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0xc0);
	self->info.mcu->ops->send_data(0x03);
	self->info.mcu->ops->send_data(0x3b);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x02);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x01);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x43);

	self->info.mcu->ops->send_cmd(0xc1);
	self->info.mcu->ops->send_data(0x08);
	self->info.mcu->ops->send_data(0x17);
	self->info.mcu->ops->send_data(0x08);
	self->info.mcu->ops->send_data(0x08);

	self->info.mcu->ops->send_cmd(0xc4);
	self->info.mcu->ops->send_data(0x22);
	self->info.mcu->ops->send_data(0x02);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0xc8);
	self->info.mcu->ops->send_data(0x09);
	self->info.mcu->ops->send_data(0x08);
	self->info.mcu->ops->send_data(0x10);
	self->info.mcu->ops->send_data(0x85);
	self->info.mcu->ops->send_data(0x07);
	self->info.mcu->ops->send_data(0x08);
	self->info.mcu->ops->send_data(0x16);
	self->info.mcu->ops->send_data(0x05);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x32);
	self->info.mcu->ops->send_data(0x05);
	self->info.mcu->ops->send_data(0x16);
	self->info.mcu->ops->send_data(0x08);
	self->info.mcu->ops->send_data(0x88);
	self->info.mcu->ops->send_data(0x09);
	self->info.mcu->ops->send_data(0x10);
	self->info.mcu->ops->send_data(0x09);
	self->info.mcu->ops->send_data(0x04);
	self->info.mcu->ops->send_data(0x32);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0x2a);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x01);
	self->info.mcu->ops->send_data(0x3f); /* 320 */

	self->info.mcu->ops->send_cmd(0x2b);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x01);
	self->info.mcu->ops->send_data(0xdf); /* 480 */

	self->info.mcu->ops->send_cmd(0x35);
	self->info.mcu->ops->send_data(0x00);

	self->info.mcu->ops->send_cmd(0x3a);
	self->info.mcu->ops->send_data(0x05);

	self->info.mcu->ops->send_cmd(0x44);
	self->info.mcu->ops->send_data(0x00);
	self->info.mcu->ops->send_data(0x01);

	self->info.mcu->ops->send_cmd(0x2c);
	self->info.mcu->ops->send_cmd(0x11);
	mdelay(150);

	self->info.mcu->ops->send_cmd(0xd0);
	self->info.mcu->ops->send_data(0x07);
	self->info.mcu->ops->send_data(0x07);
	self->info.mcu->ops->send_data(0x16);
	self->info.mcu->ops->send_data(0x72);

	self->info.mcu->ops->send_cmd(0xd1);
	self->info.mcu->ops->send_data(0x03);
	self->info.mcu->ops->send_data(0x3a);
	self->info.mcu->ops->send_data(0x0a);

	self->info.mcu->ops->send_cmd(0xd2);
	self->info.mcu->ops->send_data(0x02);
	self->info.mcu->ops->send_data(0x44);
	self->info.mcu->ops->send_data(0x04);

	self->info.mcu->ops->send_cmd(0x29);
	mdelay(10);
if(1){
	uint16_t top = 0;
	uint16_t left = 0;
	uint16_t bottom = 479;
	uint16_t right = 319;
	uint16_t color = 0xf800;

	int i, j;

	self->info.mcu->ops->send_cmd(0x2a);
	self->info.mcu->ops->send_data(left>>8);
	self->info.mcu->ops->send_data(left & 0xff);
	self->info.mcu->ops->send_data(right>>8);
	self->info.mcu->ops->send_data(right&0xff); /* 320 */

	self->info.mcu->ops->send_cmd(0x2b);
	self->info.mcu->ops->send_data(top>>8);
	self->info.mcu->ops->send_data(top&0xff);
	self->info.mcu->ops->send_data(bottom>>8);
	self->info.mcu->ops->send_data(bottom&0xff); /* 480 */
	
	self->info.mcu->ops->send_cmd(0x2c);

	for (j=0; j< (bottom - top + 1); j++)
		for (i=0; i< (right - left + 1); i++)
			self->info.mcu->ops->send_data(color);
}

	return 0;
}

static int32_t r61581_invalidate(struct lcd_spec *self)
{
	uint16_t left = 0;
	uint16_t right = self->width-1;
	uint16_t top = 0;
	uint16_t bottom = self->height-1;

	LCD_PRINT("r61581_invalidate\n");

	self->info.mcu->ops->send_cmd(0x2a);
	self->info.mcu->ops->send_data(left>>8);
	self->info.mcu->ops->send_data(left&0xff);
	self->info.mcu->ops->send_data(right>>8);
	self->info.mcu->ops->send_data(right&0xff);

	self->info.mcu->ops->send_cmd(0x2b);
	self->info.mcu->ops->send_data(top>>8);
	self->info.mcu->ops->send_data(top&0xff);
	self->info.mcu->ops->send_data(bottom>>8);
	self->info.mcu->ops->send_data(bottom&0xff);

	self->info.mcu->ops->send_cmd(0x2c);

	return 0;
}

static int32_t r61581_set_direction(struct lcd_spec *self, uint16_t direction)
{
	LCD_PRINT("r61581_set_direction\n");

	switch (direction) {
	case LCD_DIRECT_NORMAL:
		self->info.mcu->ops->send_cmd(0xc0);
		self->info.mcu->ops->send_data(0x03);
		self->info.mcu->ops->send_data(0x3b);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x02);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x01);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x43);
		break;
	case LCD_DIRECT_ROT_90:
		self->info.mcu->ops->send_cmd(0xc0);
		self->info.mcu->ops->send_data(0x0b);
		self->info.mcu->ops->send_data(0x3b);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x02);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x01);
		self->info.mcu->ops->send_data(0x00);
		self->info.mcu->ops->send_data(0x43);
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

static struct lcd_operations lcd_r61581_operations = {
	.lcd_init = r61581_init,
	.lcd_invalidate = r61581_invalidate,
	.lcd_set_direction = r61581_set_direction,
};

static struct timing_mcu lcd_r61581_timing = {
	.rcss = 170,
	.rlpw = 170,
	.rhpw = 250,
	.wcss = 30,
	.wlpw = 30,
	.whpw = 30,
};

static struct info_mcu lcd_r61581_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 16,
	.timing = &lcd_r61581_timing,
	.ops = NULL,
};

struct lcd_spec lcd_panel = {
	.width = 320,
	.height = 480,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_r61581_info},
	.ops = &lcd_r61581_operations,
};

