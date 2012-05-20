//overlord for lcd adapt
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>

#include <mach/lcd.h>



#define  LCD_PANEL_ID_HX8369			0x69

// External LCD public Varible
/*****************************************************/
extern  struct lcd_spec lcd_panel_hx8369;

//Customer need to configure the tables below according to the special product
static struct lcd_panel_cfg lcd_panel[] = {
	[0]={
		.lcd_id = LCD_PANEL_ID_HX8369,
		.panel = &lcd_panel_hx8369,
		},
};



struct  sprd_lcd_platform_data  lcd_data = {
		.lcd_panel_ptr  = lcd_panel,
		.lcd_panel_size = ARRAY_SIZE(lcd_panel),
};


