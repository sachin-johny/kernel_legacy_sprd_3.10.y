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
#include <mach/eic.h>
#include <mach/usb.h>

static struct resource sprd_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.start	= SPRD_NAND_BASE,
		.end	= SPRD_NAND_BASE + SPRD_NAND_SIZE - 1,
		.flags	= IORESOURCE_MEM,
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


extern struct sprd_lcd_platform_data lcd_data;
static struct platform_device sprd_fb_device = {
	.name	= "sc8810fb",
	.id	= -1,
	.dev.platform_data = &lcd_data,
};

static struct resource sprd_kpad_resources[] = {
        {
                .start = IRQ_KPD_INT,
                .end = IRQ_KPD_INT,
                .flags = IORESOURCE_IRQ,
        },
};

extern struct sprd_kpad_platform_data sprd_kpad_data;
static struct platform_device sprd_kpad_device = {
        .name           = "sprd-keypad",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(sprd_kpad_resources),
        .resource       = sprd_kpad_resources,
        .dev.platform_data = &sprd_kpad_data,
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

static struct platform_device inno_demod_device = {
	.name   = "inno-demod",
	.id     = -1,
};

static struct platform_device *devices[] __initdata = {
	&sprd_kpad_device,
	&sprd_nand_device,
	&sprd_fb_device,
	&sprd_battery_device,
	&sprd_kp_bl_device,
	&sprd_lcd_bl_device,
	&sprd_serial_device,
	&sprd_tp_device,
	&sprd_scale_device,
	&sprd_rotation_device,
	&sprd_vsp_device,
	&inno_demod_device,
};

void __init sprd_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

#define SPRD_SDIO_SLOT0_BASE SPRD_SDIO_BASE

#define SD0_DETECT_GPIO        101
#define SD1_DETECT_GPIO        100

#define SDIO_RESOURCE_BUILDER(base, size, detect_gpio, irq)                    \
       {       \
               {       \
                       .start = base,  \
                       .end   = base + size - 1,       \
                       .flags = IORESOURCE_MEM,        \
               },      \
               {       \
                       .start = detect_gpio,   \
                       .end   = detect_gpio,   \
                       .flags = IORESOURCE_MEM,        \
               },      \
               {       \
                       .start = irq,   \
                       .end   = irq,   \
                       .flags = IORESOURCE_IRQ,        \
               },      \
        }

static struct resource sprd_sdio_resource[][3] = {
       /*sdio0*/
       SDIO_RESOURCE_BUILDER(SPRD_SDIO0_BASE, SPRD_SDIO0_SIZE, SD0_DETECT_GPIO, IRQ_SDIO0_INT),
       /*sdio1*/
       SDIO_RESOURCE_BUILDER(SPRD_SDIO1_BASE, SPRD_SDIO1_SIZE, SD1_DETECT_GPIO, IRQ_SDIO1_INT)
 };


#define SDIO_DEV_BUILDER(bus_id, res)          \
       {       \
               .name           = "sprd-sdhci", \
               .id     = (bus_id),     \
               .num_resources  = ARRAY_SIZE(res),      \
               .resource       = (res),        \
       }

struct platform_device sprd_sdio_device[] = {
       SDIO_DEV_BUILDER(0, sprd_sdio_resource[0]),
       SDIO_DEV_BUILDER(1, sprd_sdio_resource[1])
 };


static unsigned long sdio_func_cfg[] __initdata = {
	MFP_CFG_X(SD0_CLK, AF0, DS3, F_PULL_NONE, S_PULL_NONE, IO_Z),
	MFP_CFG_X(SD_CMD, AF0, DS1, F_PULL_UP,  S_PULL_NONE, IO_Z),
	MFP_CFG_X(SD_D0, AF0, DS1, F_PULL_UP, S_PULL_NONE, IO_Z),
	MFP_CFG_X(SD_D1, AF0, DS1, F_PULL_UP, S_PULL_NONE, IO_Z),
	MFP_CFG_X(SD_D2, AF0, DS1, F_PULL_UP, S_PULL_NONE, IO_Z),
	MFP_CFG_X(SD_D3, AF0, DS1, F_PULL_UP, S_PULL_NONE, IO_Z),
};

static unsigned long sdcard_detect_gpio_cfg =
        MFP_CFG_X(RFCTL11, AF3, DS1, F_PULL_UP,S_PULL_NONE, IO_Z);

static void sprd_config_sdio_pins(void)
{
	sprd_mfp_config(sdio_func_cfg, ARRAY_SIZE(sdio_func_cfg));
	sprd_mfp_config(&sdcard_detect_gpio_cfg, 1);
}

void __init sprd_add_sdio_device(void)
{
	int err;
	/* Enable SDIO0 Module */
	__raw_bits_or(BIT_4, AHB_CTL0);
	/* reset sdio0 module*/
	__raw_bits_or(BIT_12, AHB_SOFT_RST);
	__raw_bits_and(~BIT_12, AHB_SOFT_RST);

	/* Enable SDIO1 Module */
	__raw_bits_or(BIT_19, AHB_CTL0);
	/* reset sdio1 module*/
	__raw_bits_or(BIT_16, AHB_SOFT_RST);
	__raw_bits_and(~BIT_16, AHB_SOFT_RST);

	sprd_config_sdio_pins();
	err = gpio_request(SD0_DETECT_GPIO, "sdcard detect");
	if (err) {
		pr_warning("cannot alloc gpio for sdcard detect\r\n");
		return;
	}
	gpio_direction_input(SD0_DETECT_GPIO);
	platform_device_register(&sprd_sdio_device[0]);
	platform_device_register(&sprd_sdio_device[1]);
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
		//__raw_bits_or(BIT_5, AHB_CTL0);
	}else {
		__raw_bits_and(~BIT_6, AHB_CTL3);
		__raw_bits_or(BIT_9, GR_CLK_GEN5);
		__raw_bits_and(~BIT_5, AHB_CTL0);
		//clk_disable(usb_clk);
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


	__raw_bits_or(BIT_6|BIT_7, AHB_SOFT_RST);
	mdelay(5);
	__raw_bits_and(~(BIT_6|BIT_7), AHB_SOFT_RST);
	//__raw_bits_and(~BIT_5, AHB_CTL0);
	__raw_bits_or(BIT_5, AHB_CTL0);
	//clk_enable(usb_clk);
	mdelay(5);
}

void udc_enable(void)
{
	usb_startup();
}
EXPORT_SYMBOL(udc_enable);

void udc_disable(void)
{
	usb_enable_module(0);
	usb_ldo_switch(0);
}
EXPORT_SYMBOL(udc_disable);

static int usb_vbus_irq = 0;
int usb_alloc_vbus_irq(void)
{
	int irq;

	irq = sprd_alloc_eic_irq(EIC_ID_10);//chg_int
	if (irq < 0) {
		return -1;
	}

	set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
	usb_vbus_irq = irq;
	return irq;
}

void usb_free_vbus_irq(int irq)
{
	return sprd_free_eic_irq(irq);
}

int usb_get_vbus_irq(void)
{
	return usb_vbus_irq;
}

int usb_get_vbus_state(void)
{
	int value;

	value = sprd_get_eic_data(EIC_ID_10);
	return !!value;
}

void usb_set_vbus_irq_type(int irq, int irq_type)
{
	if (irq_type == VBUS_PLUG_IN)
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else if (irq_type == VBUS_PLUG_OUT)
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}

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
        if(readl(CHIP_ID) == 0x88100001){
                /*SMIC chip id == 0x88100001*/
        	__raw_bits_or(BIT_9, USB_PHY_CTRL);
		__raw_bits_and(~(BIT_15 | BIT_14), USB_PHY_CTRL);
		__raw_bits_or(BIT_13 | BIT_12, USB_PHY_CTRL);
                writel(0x28,USB_SPR_REG);
	}else{
		/*
		 * config usb phy controller
		 */
		__raw_bits_or(BIT_8, USB_PHY_CTRL);
		__raw_bits_or(BIT_17, USB_PHY_CTRL);
		__raw_bits_and(~BIT_16, USB_PHY_CTRL);
		__raw_bits_and(~(BIT_13 | BIT_12), USB_PHY_CTRL);
		__raw_bits_or(BIT_15 | BIT_14, USB_PHY_CTRL);
	}
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
	.product		= "openphone",
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

	platform_device_register(&sprd_dcam_device);
}
static unsigned long charger_detect_cfg =
    MFP_ANA_CFG_X(CHIP_RSTN, AF0, DS1, F_PULL_UP,S_PULL_UP, IO_IE);
void __init sprd_charger_init(void)
{
	gpio_request(USB_DP_GPIO, "charger detect");
	gpio_request(USB_DM_GPIO, "charger detect");
	gpio_direction_input(USB_DP_GPIO);
	gpio_direction_input(USB_DM_GPIO);
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

#endif

void sprd_pmem_reserve_sdram(void)
{
        reserve_bootmem(SPRD_PMEM_BASE, SPRD_IO_MEM_SIZE, 0);
}
