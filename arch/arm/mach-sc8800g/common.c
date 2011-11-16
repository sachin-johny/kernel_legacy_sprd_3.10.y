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

#include <linux/delay.h>
#include <asm/mach/flash.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/usb/android_composite.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/bootmem.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/irqs.h>

#include <mach/mfp.h>
#include <mach/regs_ahb.h>
#include <mach/regs_global.h>
#include <mach/ldo.h>

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
};

static struct resource sprd_dcam_resources[] = {
	{
		.start	= SPRD_ISP_BASE,
		.end	= SPRD_ISP_BASE + SPRD_ISP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_ISP_INT,
		.end	= IRQ_ISP_INT,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct platform_device sprd_dcam_device = {
	.name		= "sc8800g_dcam",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_dcam_resources),
	.resource	= sprd_dcam_resources,
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
static struct resource sprd_tp_resources[] = {
	{
		.start	= (SPRD_MISC_BASE +0x280),
		.end	= (SPRD_MISC_BASE + 0x280+0x44),
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_ANA_TPC_INT,
		.end	= IRQ_ANA_TPC_INT,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct platform_device sprd_tp_device = {
	.name		= "sprd-tp",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_tp_resources),
	.resource	= sprd_tp_resources,
};



static struct platform_device sprd_fb_device = {
	.name	= "sc8800fb",
	.id	= -1,
};

static struct resource sprd_kpad_resources[] = {
        {
                .start = IRQ_KPD_INT,
                .end = IRQ_KPD_INT,
                .flags = IORESOURCE_IRQ,
        },
};

static struct platform_device sprd_kpad_device = {
#ifdef CONFIG_MACH_OPENPHONE
        .name           = "sprd-keypad",
#elif defined(CONFIG_MACH_SP6810A)
        .name           = "sprd-keypad6810",
#elif defined(CONFIG_MACH_SP8805GA)
        .name           = "sprd-keypad8805ga",
#endif
        .id             = -1,
        .num_resources  = ARRAY_SIZE(sprd_kpad_resources),
        .resource       = sprd_kpad_resources,
};

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

/* Flashled backlight */
static struct platform_device sprd_FlashLed_device = {
	    .name           = "flash-led",
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
        },
	[6] = {		
                .start = IRQ_SLEEP_INT,
                .end = IRQ_SLEEP_INT,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device sprd_serial_device = {
        .name           = "serial_sp",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_serial_resources),
        .resource       = sprd_serial_resources,
};

static struct platform_device sprd_2d_device = {
	.name	= "sc8800g_2d",
	.id	= -1,
};

static struct platform_device sprd_scale_device = {
	.name	= "sc8800g_scale",
	.id	= -1,
};

static struct platform_device sprd_rotation_device = {
	.name	= "sc8800g_rotation",
	.id	= -1,
};

static struct platform_device sprd_vsp_device = {
	.name	= "sc8800g_vsp",
	.id	= -1,
};

#ifdef CONFIG_CMMB_INNOFIDEI
static struct platform_device inno_demod_device = {
	.name   = "inno-demod",
	.id     = -1,
};
#endif

static struct platform_device *devices[] __initdata = {
	&sprd_kpad_device,
	&sprd_nand_device,
	&sprd_i2c_device,
	&sprd_fb_device,
	&sprd_battery_device,
	&sprd_kp_bl_device,
	&sprd_lcd_bl_device,
	&sprd_FlashLed_device,
	&sprd_serial_device, 
	&sprd_tp_device,
	&sprd_2d_device,
	&sprd_scale_device,
	&sprd_rotation_device,
	&sprd_vsp_device
#ifdef CONFIG_CMMB_INNOFIDEI
	,&inno_demod_device
#endif	
};

void __init sprd_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

#define SPRD_SDIO_SLOT0_BASE SPRD_SDIO_BASE

#define SD_DETECT_GPIO	101
static struct resource sprd_sdio_resource[] = {
	[0] = {
		.start = SPRD_SDIO_SLOT0_BASE,
		.end   = SPRD_SDIO_SLOT0_BASE + SPRD_SDIO_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SD_DETECT_GPIO,
		.end   = SD_DETECT_GPIO,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
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

void __init sprd_add_sdio_device(void)
{
	int err;
	/* Enable SDIO Module */
	__raw_bits_or(BIT_4, AHB_CTL0);
	/* reset sdio module*/
	__raw_bits_or(BIT_12, AHB_SOFT_RST);
	__raw_bits_and(~BIT_12, AHB_SOFT_RST);

        //Initialize sdio pin in file pin_map_sc8805.c
	//sprd_config_sdio_pins();
	//
	err = gpio_request(SD_DETECT_GPIO, "sdcard detect");
	if (err) {
		pr_warning("cannot alloc gpio for sdcard detect\r\n");
		return;
	}
	gpio_direction_input(SD_DETECT_GPIO);

	platform_device_register(&sprd_sdio_device);
}

static struct resource sprd_otg_resource[] = {
	[0] = {
		.start = SPRD_USB_BASE,
		.end   = SPRD_USB_BASE + SPRD_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBD_INT,
		.end   = IRQ_USBD_INT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device sprd_otg_device = {
	.name		= "dwc_otg",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_otg_resource),
	.resource	= sprd_otg_resource,
};

static inline
void    usb_ldo_switch(int flag)
{
        if(flag){
            LDO_TurnOnLDO(LDO_LDO_USB);
        } else {
            LDO_TurnOffLDO(LDO_LDO_USB);
        }
}

struct clk *usb_clk;
static void usb_enable_module(int en)
{
	if (en){
		__raw_bits_or(BIT_6, AHB_CTL3);
		__raw_bits_and(~BIT_9, GR_CLK_GEN5);
		//	__raw_bits_or(BIT_5, AHB_CTL0);
	}else {
		__raw_bits_and(~BIT_6, AHB_CTL3);
		__raw_bits_or(BIT_9, GR_CLK_GEN5);
		clk_disable(usb_clk);
	}
}
static void usb_startup(void)
{
	usb_enable_module(1);
	mdelay(10);
	usb_ldo_switch(0);
	__raw_bits_and(~BIT_1, AHB_CTL3);
	__raw_bits_and(~BIT_2, AHB_CTL3);
	usb_ldo_switch(1);
	__raw_bits_or(BIT_6, AHB_CTL3);


	//__raw_bits_or(BIT_6|BIT_7, AHB_SOFT_RST);
	//__raw_bits_or(BIT_7, AHB_SOFT_RST);
	//mdelay(5);
	//__raw_bits_and(~(BIT_6 | BIT_7), AHB_SOFT_RST);
	//__raw_bits_and(~(BIT_7), AHB_SOFT_RST);
	//   __raw_bits_or(BIT_5, AHB_CTL0);
	clk_enable(usb_clk);
	mdelay(5);
}

void udc_enable(void)
{
	usb_startup();
}
EXPORT_SYMBOL(udc_enable);

void udc_disable(void)
{
	clk_disable(usb_clk);
	usb_ldo_switch(0);
}
EXPORT_SYMBOL(udc_disable);

static int calibration_mode = false;
static int __init calibration_start(char *str)
{
        if(str)
                pr_info("modem calibartion:%s\n", str);
        calibration_mode = true;
        return 1;
}
__setup("calibration=", calibration_start);

int in_calibration(void)
{
     return (calibration_mode == true);
}
EXPORT_SYMBOL(in_calibration);

static int factory_mode = false;
static int __init factory_start(char *str)
{
        if(str)
                pr_info("factory mode!\n");
        factory_mode = true;
        return 1;
}
__setup("factory", factory_start);

int in_factory_mode(void)
{
	return (factory_mode == true);
}
EXPORT_SYMBOL(in_factory_mode);

void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	if (calibration_mode)
		return;
	/*
	 * config usb phy controller
	 */
	__raw_bits_or(BIT_8, USB_PHY_CTRL);
	__raw_bits_or(BIT_17, USB_PHY_CTRL);
	__raw_bits_and(~BIT_16, USB_PHY_CTRL);
	__raw_bits_and(~(BIT_13 | BIT_12), USB_PHY_CTRL);
	__raw_bits_or(BIT_15 | BIT_14, USB_PHY_CTRL);

	__raw_bits_and(~BIT_1, AHB_CTL3);
	__raw_bits_and(~BIT_2, AHB_CTL3);

	usb_clk = clk_get(NULL, "clk_usb_ref");
	if (IS_ERR(usb_clk)) {
		pr_warning("cannot get clock for usb\n");
		return;
	}

	platform_device_register(&sprd_otg_device);
}

/*Android USB Function */
#define SPRD_VENDOR_ID		0x1782
#define SPRD_PRODUCT_ID		0x5D00
#define SPRD_RNDIS_PRODUCT_ID		0x5D20
#define SPRD_RNDIS_ADB_PRODUCT_ID		0x5D21

static char device_serial[] = "19761202";

static char *usb_functions_ums[] = {
	"usb_mass_storage",
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
static char *usb_functions_ums_vser_gser[] = {
	"usb_mass_storage",
	"vser",
	"gser",
};

static char *usb_functions_ums_adb_vser_gser[] = {
	"usb_mass_storage",
	"adb",
	"vser",
	"gser",
};
static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_VSERIAL
	"vser",
#endif
#ifdef CONFIG_USB_ANDROID_GSERIAL
	"gser",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= SPRD_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x5D01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x5D03,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_vser_gser),
		.functions	= usb_functions_ums_vser_gser,
	},
	{
		.product_id	= 0x5D04,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb_vser_gser),
		.functions	= usb_functions_ums_adb_vser_gser,
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
	.product_name		= "Spreadtrum phone",
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

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor			= "Spreadtrum",
	.product		= "phone",
	.nluns 			= 1,
	.release		= 0x0200,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usbms_plat,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
       .ethaddr   = {0x02,0x89,0xfb,0xab,0x1b,0xcd},
       .vendorID  = SPRD_VENDOR_ID,
       .vendorDescr  = "Spreadtrum",
};

static struct platform_device rndis_device = {
       .name = "rndis",
       .id   = -1,
       .dev  = {
               .platform_data = &rndis_pdata,
       },
};
#endif

void __init sprd_gadget_init(void)
{
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	platform_device_register(&usb_mass_storage_device);
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif

	platform_device_register(&androidusb_device);
}

void __init sprd_add_dcam_device(void)
{
	// Enable DCAM Module 
	__raw_bits_or(BIT_26, AHB_CTL0);//wxz: H5:0x20900200[26]

	//sprd_config_dcam_pins();
	platform_device_register(&sprd_dcam_device);
}
void __init sprd_charger_init(void)
{
	int irq;
	gpio_request(CHARGER_DETECT_GPIO, "charger detect");
	gpio_request(USB_DP_GPIO, "charger detect");
	gpio_request(USB_DM_GPIO, "charger detect");
	gpio_direction_input(CHARGER_DETECT_GPIO);
	gpio_direction_input(USB_DP_GPIO);
	gpio_direction_input(USB_DM_GPIO);

	irq = sprd_alloc_gpio_irq(CHARGER_DETECT_GPIO);
	if (irq < 0){
		pr_warning("cant alloc gpio irq %d\n", CHARGER_DETECT_GPIO);
		return;
	}
	set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
}

/* because nkernel knows the size of "SDRAM_SIZE", and the high end memory
 * will be assigned to linux normally, so we use nk_highest_addr (the highest
 * vaddr assigned to linux) to make a guess of the SDRAM_SIZE */
extern unsigned long nk_highest_addr;
unsigned long get_sdram_plimit(void)
{

	if ((nk_highest_addr & ~(0xc0000000)) >= 0x8000000)
		return 0x10000000; /* 256MB */
	else
		return 0x8000000;  /* 128MB */
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE

static struct platform_device *ram_console;

int __init sprd_ramconsole_init(void)
{
	struct resource res = { .flags = IORESOURCE_MEM };
	int err;

	if (!(ram_console = platform_device_alloc("ram_console", 0))) {
		pr_err("ram console Failed to allocate device \n");
		err = -ENOMEM;
		goto exit;
	}

	res.start = RAM_CONSOLE_START;
	res.end = (RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1);
	pr_info("alloc resouce for ramconsole: start:%x, size:%d\n",
		res.start, RAM_CONSOLE_SIZE);
	if ((err = platform_device_add_resources(ram_console, &res, 1))) {
		pr_err("ram console:Failed to add device resource "
				"(err = %d).\n", err);
		goto exit_device_put;
	}

	if ((err = platform_device_add(ram_console))) {
		pr_err("ram console: Failed to add device (err = %d).\n",
				err);
		goto exit_device_put;
	}

	return 0;
exit_device_put:
	platform_device_put(ram_console);
	ram_console= NULL;
exit:
	return err;
}

void sprd_ramconsole_reserve_sdram(void)
{
        reserve_bootmem(RAM_CONSOLE_START, RAM_CONSOLE_SIZE, 0);
}

void sprd_pmem_reserve_sdram(void)
{
        reserve_bootmem(SPRD_PMEM_BASE, SPRD_IO_MEM_SIZE, 0);
}
#endif

