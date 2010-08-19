/* linux/arch/arm/mach-sc8800s/common.c
 *
 * Common setup code for sc8800s Boards
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

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/mfp.h>
#include <mach/regs_ahb.h>

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
	.name		= "sprd_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_i2c_resources),
	.resource	= sprd_i2c_resources,
};

static struct platform_device sprd_fb_device = {
	.name	= "sc8800sfb",
	.id	= -1,
};

static struct platform_device *devices[] __initdata = {
	&sprd_kpad_device,
	&sprd_nand_device,
	&sprd_i2c_device,
	&sprd_fb_device,
	&sprd_battery_device,
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
	sprd_mfp_config(sdio_func_cfg, ARRAY_SIZE(sdio_func_cfg));
	
}
void __init sprd_add_sdio_device(void)
{
	/* Enable SDIO Module */
	__raw_bits_or(BIT_10, AHB_MISC);

	sprd_config_sdio_pins();
	platform_device_register(&sprd_sdio_device);
}
