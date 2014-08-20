/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include "../sprdfb_panel.h"

static int32_t ili9488_init(struct panel_spec *self)
{
	send_data_t send_cmd = self->info.mcu->ops->send_cmd;
	send_data_t send_data = self->info.mcu->ops->send_data;

	printk("ili9488_init\n");

	//************* Start Initial Sequence **********//	
	send_cmd(0XF7);
	send_data(0xA9);
	send_data(0x51);
	send_data(0x2C);
	send_data(0x82);
		//Power Control 1
	send_cmd(0xC0);
	send_data(0x11);	//Verg1out =4.52
	send_data(0x11);	//Vreg2out = -4.12

		//Power Control 2
	send_cmd(0xC1);	//VGH=14.17,VGL =-9.42
	send_data(0x41);

	send_cmd(0XC5);
	send_data(0x00);
	send_data(0x17);
	send_data(0x80);

	send_cmd(0xB1);	//Frame rate
	send_data(0xB0);//70Hz
	send_data(0x11);

	send_cmd(0xB4);	//Display Inversion Control
	send_data(0x02);	//2-dot

	send_cmd(0xB6);	// Interface Mode Control
	send_data(0x02); //RGB/MCU Interface Control			//RGB
	send_data(0x02);//42µ¹ÆÁ

	send_cmd(0x55);
	send_data(0x00);

	send_cmd(0xE9);
	send_data(0x01);			//00=18BIT,01=24BIT

	send_cmd(0x36);
	send_data(0x48);//08µ¹ÆÁ

	send_cmd(0x3A);	//Interface Pixel Format
	send_data(0x77);//77-FOR 24BIT

	send_cmd(0xE0);
	send_data(0x00);
	send_data(0x06);
	send_data(0x06);
	send_data(0x08);
	send_data(0x18);
	send_data(0x0c);
	send_data(0x41);
	send_data(0x9b);
	send_data(0x4f);
	send_data(0x07);
	send_data(0x0e);
	send_data(0x0c);
	send_data(0x1c);
	send_data(0x1c);
	send_data(0x0F);

	send_cmd(0XE1);
	send_data(0x00);
	send_data(0x1b);
	send_data(0x1e);
	send_data(0x03);
	send_data(0x0e);
	send_data(0x04);
	send_data(0x35);
	send_data(0x24);
	send_data(0x49);
	send_data(0x04);
	send_data(0x0f);
	send_data(0x0e);
	send_data(0x37);
	send_data(0x3a);
	send_data(0x0F);

	send_cmd(0x35);
	send_data(0x00);//TE ON

	send_cmd(0x11);
	LCD_DelayMS(120);
	send_cmd(0x29);

	return 0;
}

static int32_t ili9488_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	send_data_t send_cmd = self->info.mcu->ops->send_cmd;
	send_data_t send_data = self->info.mcu->ops->send_data;

	if(is_sleep==1){
		//Sleep In
		send_cmd(0x28);
		LCD_DelayMS(120); 
		send_cmd(0x10);
		LCD_DelayMS(120); 
	}else{
		//Sleep Out
		send_cmd(0x11);
		LCD_DelayMS(120); 
		send_cmd(0x29);
		LCD_DelayMS(120); 
	}

	return 0;
}




static int32_t ili9488_set_window(struct panel_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	send_data_t send_cmd = self->info.mcu->ops->send_cmd;
	send_data_t send_data = self->info.mcu->ops->send_data;

	pr_debug("ili9488_set_window\n");

	send_cmd(0x2A); // col
	send_data((left >> 8));
	send_data((left & 0xFF));
	send_data((right >> 8));
	send_data((right & 0xFF));

	send_cmd(0x2B); // row
	send_data((top >> 8));
	send_data((top & 0xFF));
	send_data((bottom >> 8));
	send_data((bottom & 0xFF));

	send_cmd(0x2C); //Write data

	return 0;
}
static int32_t ili9488_invalidate(struct panel_spec *self)
{
	pr_debug("ili9488_invalidate\n");

	return self->ops->panel_set_window(self, 0, 0,
		self->width - 1, self->height - 1);
}



static int32_t ili9488_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	pr_debug("ili9488_invalidate_rect \n");

	return self->ops->panel_set_window(self, left, top,
			right, bottom);
}

static int32_t ili9488_read_id(struct panel_spec *self)
{
	int32_t id  = 0;
	send_data_t send_cmd = self->info.mcu->ops->send_cmd;
	read_data_t read_data = self->info.mcu->ops->read_data;
	send_data_t send_data = self->info.mcu->ops->send_data;

	//Get ID

	//to be added here


	return 0x84; // id;
}

static struct panel_operations lcd_ili9488_mcu_operations = {
	.panel_init = ili9488_init,
	.panel_set_window = ili9488_set_window,
	.panel_invalidate_rect= ili9488_invalidate_rect,
	.panel_invalidate = ili9488_invalidate,
	.panel_enter_sleep = ili9488_enter_sleep,
	.panel_readid          = ili9488_read_id
};

static struct timing_mcu lcd_ili9488_mcu_timing[] = {
[MCU_LCD_REGISTER_TIMING] = {                    // read/write register timing
		.rcss = 15,  // 15ns
		.rlpw = 60,
		.rhpw = 60,
		.wcss = 10,
		.wlpw = 35,
		.whpw = 35,
	},
[MCU_LCD_GRAM_TIMING] = {                    // read/write gram timing
		.rcss = 15,  // 15ns
		.rlpw = 60,
		.rhpw = 60,
		.wcss = 10,
		.wlpw = 16,
		.whpw = 16,
	},
};

static struct info_mcu lcd_ili9488_mcu_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 24,
	.bpp = 24,
	.timing =lcd_ili9488_mcu_timing,
	.ops = NULL,
};

struct panel_spec lcd_ili9488_mcu_spec = {
	.width = 320,
	.height = 480,
	.is_clean_lcd = true,
	.type = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_ili9488_mcu_info},
	.ops = &lcd_ili9488_mcu_operations,
};

struct panel_cfg lcd_ili9488_mcu = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x88,
	.lcd_name = "lcd_ili9488_mcu",
	.panel = &lcd_ili9488_mcu_spec,
};

static int __init lcd_ili9488_mcu_init(void)
{
	return sprdfb_panel_register(&lcd_ili9488_mcu);
}

subsys_initcall(lcd_ili9488_mcu_init);


