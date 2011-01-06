/* linux/arch/arm/mach-sc8800g/board-openhone.c
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
#include <linux/input.h>
#include <linux/initrd.h>
#include <linux/android_pmem.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/board.h>
#include <mach/hardware.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mach/gpio.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/regs_cpc.h>

#include <linux/clk.h>
#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>


/* pmem area definition */
#define SPRD_PMEM_BASE          ((256-8-8)*1024*1024)
#define SPRD_PMEM_SIZE          (8*1024*1024)
#define SPRD_PMEM_ADSP_BASE     (SPRD_PMEM_BASE+SPRD_PMEM_SIZE)
#define SPRD_PMEM_ADSP_SIZE     (8*1024*1024)

static struct resource example_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C004400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 44,
		.end	= 44,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device example_device = {
	.name           = "example",
	.id	            = 0,
	.num_resources  = ARRAY_SIZE(example_resources),
	.resource       = example_resources,
};

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
       .name = "pmem",
       .start = SPRD_PMEM_BASE,
       .size = SPRD_PMEM_SIZE,
       .no_allocator = 0,
       .cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .start = SPRD_PMEM_ADSP_BASE,
       .size = SPRD_PMEM_ADSP_SIZE,
       .no_allocator = 0,
       .cached = 1,
};

struct platform_device android_pmem_device = {
       .name = "android_pmem",
       .id = 0,
       .dev = { .platform_data = &android_pmem_pdata },
};

struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};
#endif

#if defined(CONFIG_SPI_SC88XX) || defined(CONFIG_SPI_SC88XX_MODULE)
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/mfp.h>
static int spi_cs_gpio[] = {
    [0] = 32, // cs = 0 , GPIO32 == SPI_CSN0
    [1] = 33, // cs = 1 , GPIO33 == SPI_CSN1
    [2] = 32, // to cs0
    [3] = 33, // to cs1
};

static struct spi_board_info openhone_spi_devices[] = {
    {
        .modalias       = "spidev", // "spidev" --> spidev_spi
        .chip_select    = 0,
        .max_speed_hz   = 8 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
    {
        .modalias       = "spidev", // "spidev" --> spidev_spi
        .chip_select    = 1,
        .max_speed_hz   = 0.5 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
    {
        .modalias       = "spi_slot0", // "spidev" --> spidev_spi
        .chip_select    = 2,
        .max_speed_hz   = 8 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
    {
        .modalias       = "spi_slot1", // "spidev" --> spidev_spi
        .chip_select    = 3,
        .max_speed_hz   = 0.5 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
};

static u64 spi_dmamask = DMA_BIT_MASK(32);
static struct resource spi_resources[] = {
    [0] = {
        .start  = SPRD_SPI_PHYS,
        .end    = SPRD_SPI_PHYS + SZ_4K - 1,
        .flags  = IORESOURCE_MEM,
	},
    [1] = {
        .start  = IRQ_SPI_INT,
        .end    = IRQ_SPI_INT,
        .flags  = IORESOURCE_IRQ,
    },
};
static struct platform_device sprd_spi_controller_device = {
    .name   = "sprd_spi",
    .id     = 0,
	.dev    = {
        .dma_mask           = &spi_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
	},
	.resource	= spi_resources,
	.num_resources	= ARRAY_SIZE(spi_resources),
};

#define GPIO_OUTPUT_DEFAUT_VALUE_HIGH   (1 << 31)
struct gpio_desc {
    unsigned long mfp;
    int io;
    const char *desc;
};

static struct gpio_desc gpio_func_cfg[] = {
    {
        MFP_CFG_X(RFCTL9    , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE), // wifi_power_io
        99 | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi power"
    },
    {
        MFP_CFG_X(GPIO139, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        139 | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi wake"
    },
    {
        MFP_CFG_X(GPIO140, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        140 | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi reset"
    },
    {
        MFP_CFG_X(RFCTL0    , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE), // BT_RESET
        90 | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "BT reset"
    },
};

static unsigned long spi_func_cfg[] = {
	MFP_CFG_X(SPI_CLK   , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_DI    , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_DO    , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
#if 1
    /* configure cs pin to normal gpio */
	MFP_CFG_X(SPI_CSN0  , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
	MFP_CFG_X(SPI_CSN1  , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
#else
    /* configure cs pin to spi csx */ 
    MFP_CFG_X(SPI_CSN0  , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_CSN1  , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
#endif
};

static void sprd_spi_init(void)
{
    int gpio, value;
    struct gpio_desc *gd;
    int i, nr_chip = ARRAY_SIZE(openhone_spi_devices);
    struct spi_board_info *chip = openhone_spi_devices;

    for (i = 0; i < ARRAY_SIZE(gpio_func_cfg); i++) {
        gd = &gpio_func_cfg[i];
        sprd_mfp_config(&gd->mfp, 1);
        gpio = gd->io & ~GPIO_OUTPUT_DEFAUT_VALUE_HIGH;
        value = !!(gd->io & GPIO_OUTPUT_DEFAUT_VALUE_HIGH);
        if (gpio_request(gpio, gd->desc))
            printk(KERN_WARNING "%s : [%s] gpio %d request failed!\n", __func__, gd->desc, gpio);
        if (gd->mfp & MFP_IO_OE) {
            gpio_direction_output(gpio, value);
        } else if (gd->mfp & MFP_IO_IE) {
            gpio_direction_input(gpio);
        } else {
            printk(KERN_WARNING "%s : not support gpio mode!\n", __func__);
        }
    }

    sprd_mfp_config(spi_func_cfg, ARRAY_SIZE(spi_func_cfg));

    for (i = 0; i < nr_chip; i++) {
        gpio = spi_cs_gpio[chip[i].chip_select];
#if 0
        // we do it in sprd_spi_setup func
        gpio_request(gpio, chip[i].modalias);
        gpio_direction_output(gpio, !(chip[i].mode & SPI_CS_HIGH));
#endif
        chip[i].controller_data = (void*)gpio;
    }

    spi_register_board_info(chip, nr_chip);
    platform_device_register(&sprd_spi_controller_device);
}
#else
static void sprd_spi_init(void) {}
#endif

static struct platform_device *devices[] __initdata = {
	&example_device,
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
#endif
};


extern struct sys_timer sprd_timer;

static void __init openphone_init_irq(void)
{
	sc8800g2_clock_init();
	sprd_init_irq();
}

int __init LDO_Init();
static void __init chip_init(void)
{
    ANA_REG_SET(ANA_ADIE_CHIP_ID,0);
    /* setup pins configration when LDO shutdown*/
    __raw_writel(0x1fff00, PIN_CTL_REG);
}

static void __init openphone_init(void)
{
	chip_init();
	ADI_init();
	LDO_Init();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sprd_add_devices();
	sprd_gpio_init();
	sprd_add_sdio_device();
	sprd_add_otg_device();
	sprd_gadget_init();
	sprd_add_dcam_device();
    sprd_spi_init();
}

static void __init openphone_map_io(void)
{
	sprd_map_common_io();
}

extern unsigned long phys_initrd_start;
extern unsigned long phys_initrd_size;

static void __init
openphone_fixup(struct machine_desc *desc, struct tag *tag,

	    char **cmdline, struct meminfo *mi)
{
#ifdef CONFIG_BLK_DEV_INITRD

	/*
	phys_initrd_start = 0x3000000;
 	phys_initrd_size = 4*1024*1024;
	*/
    
#endif
}

MACHINE_START(OPENPHONE, "SPRDOP")
/* UART for LL DEBUG */
	.phys_io        = SPRD_SERIAL1_PHYS,
	.io_pg_offst    = ((SPRD_SERIAL1_BASE) >> 18) & 0xfffc,

	.map_io         = openphone_map_io,
	.init_irq       = openphone_init_irq,
	.init_machine   = openphone_init,
	.timer          = &sprd_timer,
	.fixup          = openphone_fixup,
MACHINE_END
