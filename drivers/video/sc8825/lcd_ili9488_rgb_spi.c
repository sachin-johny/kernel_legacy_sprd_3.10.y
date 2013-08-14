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
#include <linux/kernel.h>
#include <linux/delay.h>
#include "sprdfb_panel.h"
#include <asm/io.h>

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define ILI9488_SpiWriteCmd(cmd) \ 
{ \
	spi_send_cmd((cmd& 0xFF));\
}

#define  ILI9488_SpiWriteData(data)\
{ \
	spi_send_data((data& 0xFF));\
}

static int32_t ili9488_init(struct panel_spec *self)
{
	uint32_t data = 0;
	spi_send_cmd_t spi_send_cmd = self->info.rgb->bus_info.spi->ops->spi_send_cmd;
	spi_send_data_t spi_send_data = self->info.rgb->bus_info.spi->ops->spi_send_data;
	spi_read_t spi_read = self->info.rgb->bus_info.spi->ops->spi_read;

	LCD_PRINT("ili9488_init\n");

	//************* Start Initial Sequence **********//
	ILI9488_SpiWriteCmd(0xE0);
	ILI9488_SpiWriteData(0x00);
	ILI9488_SpiWriteData(0x06);
	ILI9488_SpiWriteData(0x12);
	ILI9488_SpiWriteData(0x09);
	ILI9488_SpiWriteData(0x18);
	ILI9488_SpiWriteData(0x0A);
	ILI9488_SpiWriteData(0x42);
	ILI9488_SpiWriteData(0x87);
	ILI9488_SpiWriteData(0x4D);
	ILI9488_SpiWriteData(0x0B);
	ILI9488_SpiWriteData(0x0F);
	ILI9488_SpiWriteData(0x0A);
	ILI9488_SpiWriteData(0x18);
	ILI9488_SpiWriteData(0x1B);
	ILI9488_SpiWriteData(0x0F);

	ILI9488_SpiWriteCmd(0xE1);
	ILI9488_SpiWriteData(0x00);
	ILI9488_SpiWriteData(0x15);
	ILI9488_SpiWriteData(0x1B);
	ILI9488_SpiWriteData(0x03);
	ILI9488_SpiWriteData(0x0F);
	ILI9488_SpiWriteData(0x05);
	ILI9488_SpiWriteData(0x32);
	ILI9488_SpiWriteData(0x24);
	ILI9488_SpiWriteData(0x45);
	ILI9488_SpiWriteData(0x02);
	ILI9488_SpiWriteData(0x0A);
	ILI9488_SpiWriteData(0x09);
	ILI9488_SpiWriteData(0x33);
	ILI9488_SpiWriteData(0x39);
	ILI9488_SpiWriteData(0x0F);

	ILI9488_SpiWriteCmd(0xB0);
	ILI9488_SpiWriteData(0x80);  //3//0x80 3-wires, 4-wires

	ILI9488_SpiWriteCmd(0xB1); 
	ILI9488_SpiWriteData(0xA0);

	ILI9488_SpiWriteCmd(0xB4);
	ILI9488_SpiWriteData(0x02);

	ILI9488_SpiWriteCmd(0xB5);
	ILI9488_SpiWriteData(0x04);
	ILI9488_SpiWriteData(0x04);
	ILI9488_SpiWriteData(0x0A);
	ILI9488_SpiWriteData(0x04);

	ILI9488_SpiWriteCmd(0xB6);
	ILI9488_SpiWriteData(0xB2);//by pass
	ILI9488_SpiWriteData(0x22);

	ILI9488_SpiWriteCmd(0xC0);
	ILI9488_SpiWriteData(0x1B);
	ILI9488_SpiWriteData(0x1B);

	ILI9488_SpiWriteCmd(0xC1);
	ILI9488_SpiWriteData(0x41);

	ILI9488_SpiWriteCmd(0xC5);
	ILI9488_SpiWriteData(0x00);
	ILI9488_SpiWriteData(0x25); //0A-05  0x1f
	ILI9488_SpiWriteData(0x80);

	ILI9488_SpiWriteCmd(0x36);
	ILI9488_SpiWriteData(0x08);   

	ILI9488_SpiWriteCmd(0x3a);    
	ILI9488_SpiWriteData(0x77);//24bits

	ILI9488_SpiWriteCmd(0x35);    
	ILI9488_SpiWriteData(0x00);

	ILI9488_SpiWriteCmd(0x44);    
	ILI9488_SpiWriteData(0x00);
	ILI9488_SpiWriteData(0x00);

	ILI9488_SpiWriteCmd(0xE9);
	ILI9488_SpiWriteData(0x01);  //0x00

	ILI9488_SpiWriteCmd(0XF7);
	ILI9488_SpiWriteData(0xA9);
	ILI9488_SpiWriteData(0x51);
	ILI9488_SpiWriteData(0x2C);
	ILI9488_SpiWriteData(0x82);

	ILI9488_SpiWriteCmd(0x11);
	mdelay(120);
	ILI9488_SpiWriteCmd(0x29);
	mdelay(20);
	ILI9488_SpiWriteCmd(0x2C);
}

static int32_t ili9488_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	spi_send_cmd_t spi_send_cmd = self->info.rgb->bus_info.spi->ops->spi_send_cmd;
	spi_send_data_t spi_send_data = self->info.rgb->bus_info.spi->ops->spi_send_data;
	
	if(is_sleep==1){
		//Sleep In
		ILI9488_SpiWriteCmd(0x28);
		mdelay(120);
		ILI9488_SpiWriteCmd(0x10);
		mdelay(10);
	}else{
		//Sleep Out
		ILI9488_SpiWriteCmd(0x11);
		mdelay(120);
		ILI9488_SpiWriteCmd(0x29);
		mdelay(10);
	}

	return 0;
}




static int32_t ili9488_set_window(struct panel_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	spi_send_cmd_t spi_send_cmd = self->info.rgb->bus_info.spi->ops->spi_send_cmd;
	spi_send_data_t spi_send_data = self->info.rgb->bus_info.spi->ops->spi_send_data;

#if 0
	LCD_PRINT("ili9488_set_window: %d, %d, %d, %d\n",left, top, right, bottom);

	ILI9488_SpiWriteCmd(0x2A);
	ILI9488_SpiWriteData((left>>8));// set left address
	ILI9488_SpiWriteData((left&0xff));
	ILI9488_SpiWriteData((right>>8));// set right address
	ILI9488_SpiWriteData((right&0xff));

	ILI9488_SpiWriteCmd(0x2B);
	ILI9488_SpiWriteData((top>>8));// set left address
	ILI9488_SpiWriteData((top&0xff));
	ILI9488_SpiWriteData((bottom>>8));// set bottom address
	ILI9488_SpiWriteData((bottom&0xff));

	ILI9488_SpiWriteCmd(0x2C);
#endif
	return 0;
}

static int32_t ili9488_invalidate(struct panel_spec *self)
{
	LCD_PRINT("ili9488_invalidate\n");

	return self->ops->panel_set_window(self, 0, 0,
		self->width - 1, self->height - 1);
}

static int32_t ili9488_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	LCD_PRINT("ili9488_invalidate_rect \n");

	return self->ops->panel_set_window(self, left, top,
			right, bottom);
}

static int32_t ili9488_read_id(struct panel_spec *self)
{
	return 0x9488;
}

static struct panel_operations lcd_ili9488_rgb_spi_operations = {
	.panel_init = ili9488_init,
	.panel_set_window = ili9488_set_window,
	.panel_invalidate_rect= ili9488_invalidate_rect,
	.panel_invalidate = ili9488_invalidate,
	.panel_enter_sleep = ili9488_enter_sleep,
	.panel_readid          = ili9488_read_id
};

static struct timing_rgb lcd_ili9488_rgb_spi_timing = {
	.hfp = 180,  /* unit: pixel */
	.hbp = 50,
	.hsync = 10,
	.vfp = 20, /*unit: line*/
	.vbp = 14,
	.vsync = 2,
};

static struct spi_info lcd_ili9488_rgb_spi_info = {
	.ops = NULL,
};

static struct info_rgb lcd_ili9488_rgb_info = {
	.cmd_bus_mode  = SPRDFB_RGB_BUS_TYPE_SPI,
	.video_bus_width = 24, /*18,16*/
	.h_sync_pol = SPRDFB_POLARITY_NEG,
	.v_sync_pol = SPRDFB_POLARITY_NEG,
	.de_pol = SPRDFB_POLARITY_POS,
	.timing = &lcd_ili9488_rgb_spi_timing,
	.bus_info = {
		.spi = &lcd_ili9488_rgb_spi_info,
	}
};

struct panel_spec lcd_ili9488_rgb_spi_spec = {
	.width = 320,
	.height = 480,
	.fps = 58,
	.type = LCD_MODE_RGB,
	.direction = LCD_DIRECT_NORMAL,
	.info = {
		.rgb = &lcd_ili9488_rgb_info
	},
	.ops = &lcd_ili9488_rgb_spi_operations,
};

struct panel_cfg lcd_ili9488_rgb_spi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x9488,
	.lcd_name = "lcd_ili9488_rgb_spi",
	.panel = &lcd_ili9488_rgb_spi_spec,
};

static int __init lcd_ili9488_rgb_spi_init(void)
{
	return sprdfb_panel_register(&lcd_ili9488_rgb_spi);
}

subsys_initcall(lcd_ili9488_rgb_spi_init);

