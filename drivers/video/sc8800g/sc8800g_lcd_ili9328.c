/* drivers/video/sc8800g/sc8800g_lcd_ili9328.c
 *
 * Support for ILI9328 LCD device
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

#include "sc8800g_lcd.h"

//#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

static int32_t ili9328_init(struct lcd_spec *self)
{
	LCD_PRINT("ili9328_init\n");

	/* reset the lcd */
	self->ops->lcd_reset(self);
	mdelay(50);

	/* start init sequence */
	self->info.mcu->ops->send_cmd_data(0x00e3, 0x3008);
	self->info.mcu->ops->send_cmd_data(0x00e7, 0x0012);
	self->info.mcu->ops->send_cmd_data(0x00ef, 0x1231);
	self->info.mcu->ops->send_cmd_data(0x0001, 0x0100);
	self->info.mcu->ops->send_cmd_data(0x0002, 0x0700);
	self->info.mcu->ops->send_cmd_data(0x0003, 0x1030);
	self->info.mcu->ops->send_cmd_data(0x0004, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0008, 0x0207);
	self->info.mcu->ops->send_cmd_data(0x0009, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x000a, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x000c, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x000d, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x000f, 0x0000);

	/* start power on sequence */
	self->info.mcu->ops->send_cmd_data(0x0010, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0011, 0x0007);
	self->info.mcu->ops->send_cmd_data(0x0012, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0001);
	mdelay(200);
	self->info.mcu->ops->send_cmd_data(0x0010, 0x1590);
	self->info.mcu->ops->send_cmd_data(0x0011, 0x0221);
	mdelay(50);
	self->info.mcu->ops->send_cmd_data(0x0012, 0x009a);
	mdelay(50);
	self->info.mcu->ops->send_cmd_data(0x0013, 0x1b00);
	self->info.mcu->ops->send_cmd_data(0x0029, 0x0013);
	self->info.mcu->ops->send_cmd_data(0x002b, 0x000e);
	mdelay(50);
	self->info.mcu->ops->send_cmd_data(0x0020, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0021, 0x0000);

	/* adjust the gamma curve */
	self->info.mcu->ops->send_cmd_data(0x0030, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0031, 0x0406);
	self->info.mcu->ops->send_cmd_data(0x0032, 0x0004);
	self->info.mcu->ops->send_cmd_data(0x0035, 0x0305);
	self->info.mcu->ops->send_cmd_data(0x0036, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0037, 0x0207);
	self->info.mcu->ops->send_cmd_data(0x0038, 0x0103);
	self->info.mcu->ops->send_cmd_data(0x0039, 0x0707);
	self->info.mcu->ops->send_cmd_data(0x003c, 0x0503);
	self->info.mcu->ops->send_cmd_data(0x003d, 0x0004);

	/* set gram area */
	self->info.mcu->ops->send_cmd_data(0x0050, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0051, 0x00ef);
	self->info.mcu->ops->send_cmd_data(0x0052, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0053, 0x013f);
	self->info.mcu->ops->send_cmd_data(0x0060, 0xa700);
	self->info.mcu->ops->send_cmd_data(0x0061, 0x0001);
	self->info.mcu->ops->send_cmd_data(0x006a, 0x0000);

	/* partial display control */
	self->info.mcu->ops->send_cmd_data(0x0080, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0081, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0082, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0083, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0084, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0085, 0x0000);

	/* panel control */
	self->info.mcu->ops->send_cmd_data(0x0090, 0x0010);
	self->info.mcu->ops->send_cmd_data(0x0092, 0x0600);

	self->info.mcu->ops->send_cmd_data(0x0007, 0x0133);

	/* dislay on */
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0022);
	self->info.mcu->ops->send_cmd_data(0x0007, 0x0133);
	mdelay(30);

	/* write to gram */
	self->info.mcu->ops->send_cmd_data(0x0020, 0x0000);
	self->info.mcu->ops->send_cmd_data(0x0021, 0x0000);

	self->info.mcu->ops->send_cmd(0x0022);

if(0){
	int i;
	for (i=0; i<320*240/2; i++)
		self->info.mcu->ops->send_data(0x1234);
	for (i=0; i< 320*240/2; i++)
		self->info.mcu->ops->send_data(0x4321);
}

	return 0;
}

static int32_t ili9328_set_window(struct lcd_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	LCD_PRINT("ili9328_set_window\n");

	self->info.mcu->ops->send_cmd_data(0x0050, left);
	self->info.mcu->ops->send_cmd_data(0x0051, right);
	self->info.mcu->ops->send_cmd_data(0x0052, top);
	self->info.mcu->ops->send_cmd_data(0x0053, bottom);

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

static int32_t ili9328_invalidate(struct lcd_spec *self)
{
	LCD_PRINT("ili9328_invalidate\n");

	return self->ops->lcd_set_window(self, 0, 0, 
			self->width-1, self->height-1);
	
}

static int32_t ili9328_set_direction(struct lcd_spec *self, uint16_t direction)
{
	LCD_PRINT("ili9328_set_direction\n");

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

static struct lcd_operations lcd_ili9328_operations = {
	.lcd_init = ili9328_init,
	.lcd_set_window = ili9328_set_window,
	.lcd_invalidate = ili9328_invalidate,
	.lcd_set_direction = ili9328_set_direction,
};

static struct timing_mcu lcd_ili9328_timing = {
	.rcss = 5,
	.rlpw = 150,
	.rhpw = 150,
	.wcss = 10,
	.wlpw = 50,
	.whpw = 50,
};

static struct info_mcu lcd_ili9328_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 16,
	.timing = &lcd_ili9328_timing,
	.ops = NULL,
};

struct lcd_spec lcd_panel = {
	.width = 240,
	.height = 320,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_ili9328_info},
	.ops = &lcd_ili9328_operations,
};

