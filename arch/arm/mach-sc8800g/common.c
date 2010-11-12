/* linux/arch/arm/mach-sc8800g/common.c
 *
 * Common setup code for sc8800g Boards
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
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/mach/flash.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/usb/android_composite.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/irqs.h>

#include <mach/mfp.h>
#include <mach/regs_ahb.h>

#include <linux/input.h>
#include <mach/sprd_key.h>



struct flash_platform_data sprd_nand_data = {
	.parts		= 0,
	.nr_parts	= 0,
};

static struct resource sprd_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device sprd_nand_device = {
	.name		= "sprd_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sprd_nand_resources),
	.resource	= sprd_nand_resources,
	.dev		= {
		.platform_data	= &sprd_nand_data,
	},
};

static struct platform_device sprd_smd_device = {
	.name	= "sprd_smd",
	.id	= -1,
};

static struct resource sprd_i2c_resources[] = {
	{
		.start	= SPRD_I2C_BASE,
		.end	= SPRD_I2C_BASE + SPRD_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_I2C_INT,
		.end	= IRQ_I2C_INT,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sprd_i2c_device = {
	.name		= "sc8800-i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_i2c_resources),
	.resource	= sprd_i2c_resources,
};


static struct platform_device sprd_fb_device = {
	.name	= "sc8800fb",
	.id	= -1,
};

#if defined(CONFIG_KEYBOARD_SPRD) || defined(CONFIG_KEYBOARD_SPRD_MODULE)
static const unsigned int sprd_keymap[] = {
        // 0 col
	KEYVAL(0, 0, KEY_BACK),
        KEYVAL(0, 1, KEY_UP),
        KEYVAL(0, 2, KEY_CAMERA),
        KEYVAL(0, 3, KEY_7),
        KEYVAL(0, 4, KEY_VOLUMEDOWN),
        KEYVAL(0, 5, KEY_PAUSECD),
        // 1 col
        KEYVAL(1, 0, KEY_RIGHT),
        //KEYVAL(1, 1, KEY_OK),
	KEYVAL(1, 1, KEY_ENTER),
        KEYVAL(1, 2, KEY_LEFT),
        //KEYVAL(1, 3, KEY_HELP), //KEY_HELP is not known
	KEYVAL(1, 3, KEY_ENTER),
        KEYVAL(1, 4, KEY_VOLUMEUP),
        KEYVAL(1, 5, KEY_BACK),
        // 2 col
        KEYVAL(2, 0, KEY_HELP), //KEY_HELP is not known
        KEYVAL(2, 1, KEY_3),
        KEYVAL(2, 2, KEY_6),
        KEYVAL(2, 3, KEY_9),
        KEYVAL(2, 4, KEY_KPDOT), // is #
        KEYVAL(2, 5, KEY_FORWARD),
        // 3 col
        KEYVAL(3, 0, KEY_SEND),
        KEYVAL(3, 1, KEY_1),
        KEYVAL(3, 2, KEY_4),
        //KEYVAL(3, 3, KEY_1),
        KEYVAL(3, 4, KEY_KPASTERISK), //is *
        KEYVAL(3, 5, KEY_MENU),
        // 4 col
        KEYVAL(4, 0, KEY_DOWN),
        KEYVAL(4, 1, KEY_2),
        KEYVAL(4, 2, KEY_5),
        KEYVAL(4, 3, KEY_8),
        KEYVAL(4, 4, KEY_0),
};

static struct sprd_kpad_platform_data sprd_kpad_data = {
        .rows                   = 6,
        .cols                   = 5,
        .keymap                 = sprd_keymap,
        .keymapsize             = ARRAY_SIZE(sprd_keymap),
        .repeat                 = 0,
        .debounce_time          = 5000, /* ns (5ms) */
        .coldrive_time          = 1000, /* ns (1ms) */
        .keyup_test_interval    = 50, /* 50 ms (50ms) */
};


static struct resource sprd_kpad_resources[] = {
        {
                .start = IRQ_KPD_INT,
                .end = IRQ_KPD_INT,
                .flags = IORESOURCE_IRQ,
        },
};

static struct platform_device sprd_kpad_device = {
        .name           = "sprd-keys",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(sprd_kpad_resources),
        .resource       = sprd_kpad_resources,
        .dev            = {
                .platform_data = &sprd_kpad_data,
        },
};
#endif

static struct resource sprd_battery_resources[] = {
        [0] = {
                .start = 0,
                .end = 0,
                .flags = IORESOURCE_MEM,
        }
};

static struct platform_device sprd_battery_device = {
        .name           = "sprd-battery",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_battery_resources),
        .resource       = sprd_battery_resources,
};

/* keypad backlight */
static struct platform_device sprd_kp_bl_device = {
	.name           = "keyboard-backlight",
        .id             =  -1,
};

/* lcd backlight */
static struct platform_device sprd_lcd_bl_device = {
	.name           = "lcd-backlight",
        .id             =  -1,
};

static struct resource sprd_serial_resources[] = {
        [0] = {
		.start = SPRD_SERIAL0_BASE,
                .end = SPRD_SERIAL0_BASE + SPRD_SERIAL0_SIZE-1,
		.name = "serial_res0",
                .flags = IORESOURCE_MEM,
        },
	[1] = {
		.start = SPRD_SERIAL1_BASE,
                .end = SPRD_SERIAL1_BASE + SPRD_SERIAL1_SIZE-1,
		.name = "serial_res1",
                .flags = IORESOURCE_MEM,
        },
	[2] = {		
                .start = SPRD_SERIAL2_BASE,
                .end = SPRD_SERIAL2_BASE + SPRD_SERIAL2_SIZE-1,
		.name = "serial_res2",
                .flags = IORESOURCE_MEM,
        },
	[3] = {		
                .start = IRQ_SER0_INT,
                .end = IRQ_SER0_INT,
		.name = "serial_res3",
                .flags = IORESOURCE_IRQ,
        },
	[4] = {		
                .start = IRQ_SER1_INT,
                .end = IRQ_SER1_INT,
		.name = "serial_res4",
                .flags = IORESOURCE_IRQ,
        },
	[5] = {		
                .start = IRQ_SER2_INT,
                .end = IRQ_SER2_INT,
		.name = "serial_res5",
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device sprd_serial_device = {
        .name           = "serial_sp",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_serial_resources),
        .resource       = sprd_serial_resources,
};

static struct platform_device *devices[] __initdata = {
	//&sprd_kpad_device,
	&sprd_nand_device,
	&sprd_i2c_device,
	&sprd_fb_device,
	&sprd_battery_device,
	&sprd_kp_bl_device,
	&sprd_lcd_bl_device,
	&sprd_serial_device,
};

void __init sprd_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

#define SPRD_SDIO_SLOT0_BASE SPRD_SDIO_BASE
static struct resource sprd_sdio_resource[] = {
	[0] = {
		.start = SPRD_SDIO_SLOT0_BASE,
		.end   = SPRD_SDIO_SLOT0_BASE + SPRD_SDIO_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO_INT,
		.end   = IRQ_SDIO_INT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device sprd_sdio_device = {
	.name		= "sprd-sdhci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_sdio_resource),
	.resource	= sprd_sdio_resource,
};

static unsigned long sdio_func_cfg[] __initdata = {
	MFP_CFG_X(SD0_CLK, AF0, DS3, PULL_UP, IO_OE),
	MFP_CFG_X(SD0_CMD, AF0, DS3, PULL_UP, IO_Z),
	MFP_CFG_X(SD0_D0, AF0, DS3, PULL_UP, IO_Z),
	MFP_CFG_X(SD0_D1, AF0, DS3, PULL_UP, IO_Z),
	MFP_CFG_X(SD0_D2, AF0, DS3, PULL_UP, IO_Z),
	MFP_CFG_X(SD0_D3, AF0, DS3, PULL_UP, IO_Z),		
};

static void sprd_config_sdio_pins(void)
{
//	sprd_mfp_config(sdio_func_cfg, ARRAY_SIZE(sdio_func_cfg));
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xA8) = 0x38a;	//SD0 CLK output pullup drv3
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xAC) = 0x38a;	//SD0 CLK output pullup drv3
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xB0) = 0x38a;	//SD0 CLK output pullup drv3
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xB4) = 0x38a;	//SD0 CLK output pullup drv3
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xB8) = 0x38a;	//SD0 CLK output pullup drv3
	*(volatile unsigned int*)(SPRD_CPC_BASE + 0xA4) = 0x38a;	//SD0 CLK output pullup drv3
}
void __init sprd_add_sdio_device(void)
{
	/* Enable SDIO Module */
	__raw_bits_or(BIT_4, AHB_CTL0);
	__raw_bits_or(BIT_12, AHB_SOFT_RST);
	__raw_bits_and(~BIT_12, AHB_SOFT_RST);

	sprd_config_sdio_pins();
	platform_device_register(&sprd_sdio_device);
}

/*Android USB Function */
#define SPRD_VENDOR_ID		0x22B8
#define SPRD_PRODUCT_ID		0x41D9
#define SPRD_ADB_PRODUCT_ID		0x41DB
#define SPRD_RNDIS_PRODUCT_ID		0x41E4
#define SPRD_RNDIS_ADB_PRODUCT_ID		0x41E5

//#define SPRD_PRODUCT_ADB
//#define SPRD_PRODUCT_UMS

static char device_serial[] = "19761202";

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
//	#ifdef SPRD_PRODUCT_ADB
	{
		.product_id	= SPRD_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
//	#endif
//	#ifdef SPRD_PRODUCT_UMS
	{
		.product_id	= SPRD_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
//	#endif

	{
		.product_id	= SPRD_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= SPRD_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= SPRD_RNDIS_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id			= SPRD_VENDOR_ID,
	.product_id			= SPRD_PRODUCT_ID,
	.manufacturer_name	= "Spreadtrum",
	.product_name		= "Spreadtrum Bigphone",
	.serial_number		= device_serial,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor			= "Spreadtrum",
	.product		= "Bigphone",
	.release		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usbms_plat,
	},
};

void __init sprd_gadget_init(void)
{
	#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	platform_device_register(&usb_mass_storage_device);
	#endif
	platform_device_register(&androidusb_device);
}
