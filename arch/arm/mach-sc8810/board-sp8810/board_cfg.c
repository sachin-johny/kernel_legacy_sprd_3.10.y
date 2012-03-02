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
#include <mach/cache-l3x0.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mach/gpio.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/regs_cpc.h>
#include <mach/mfp.h>
#include <mach/regs_ahb.h>

#include <linux/clk.h>
#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>
#include <mach/common.h>
#include <linux/i2c.h>
#include <mach/ldo.h>

#ifdef CONFIG_SENSORS_MMC31XX
#include <linux/mmc31xx.h>
#endif
#ifdef CONFIG_SENSORS_MXC622X
#include <linux/mxc622x.h>
#endif
#include <linux/dcam_sensor.h>

static struct resource example_resources[] = {
	[0] = {
	       .start = 0x9C004300,
	       .end = 0x9C004400,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = 44,
	       .end = 44,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device example_device = {
	.name = "example",
	.id = 0,
	.num_resources = ARRAY_SIZE(example_resources),
	.resource = example_resources,
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
	.dev = {.platform_data = &android_pmem_pdata},
};

struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = {.platform_data = &android_pmem_adsp_pdata},
};
#endif

#if defined(CONFIG_SPI_SC8810) || defined(CONFIG_SPI_SC8810_MODULE)
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/mfp.h>

#define SPRD_3RDPARTY_SPI_MASTER_BUS_NUM    0
#define SPRD_3RDPARTY_SPI_MASTER_CS0_GPIO   33//32
#define SPRD_3RDPARTY_SPI_MASTER_CS1_GPIO   32//33
#define SPRD_3RDPARTY_SPI_MASTER_CS2_GPIO   33//32
#define SPRD_3RDPARTY_SPI_MASTER_CS3_GPIO   32//33

#define SPRD_3RDPARTY_SPI_WIFI_CS   2
#define SPRD_3RDPARTY_SPI_CMMB_CS   3

static int spi_cs_gpio[] = {
	[0] = SPRD_3RDPARTY_SPI_MASTER_CS0_GPIO,	// cs = 0 , GPIO32 == SPI_CSN0
	[1] = SPRD_3RDPARTY_SPI_MASTER_CS1_GPIO,	// cs = 1 , GPIO33 == SPI_CSN1
	[2] = SPRD_3RDPARTY_SPI_MASTER_CS2_GPIO,	// to cs0
	[3] = SPRD_3RDPARTY_SPI_MASTER_CS3_GPIO,	// to cs1
};

static struct spi_board_info openhone_spi_devices[] = {
	{
	 .modalias = "spidev",	// "spidev" --> spidev_spi
	 .chip_select = 0,
	 .max_speed_hz = 1000 * 1000,
	 .mode = SPI_CPOL | SPI_CPHA,
	 },
	{
	 .modalias = "spidev",	// "spidev" --> spidev_spi
	 .chip_select = 1,
	 .max_speed_hz = 1000 * 1000,
	 .mode = SPI_CPOL | SPI_CPHA,
	 },
};

static struct spi_board_info openhone_spi_devices4wifi[] = {
	{
	 .modalias = "spi_slot0",	// "spidev" --> spidev_spi
	 .chip_select = SPRD_3RDPARTY_SPI_WIFI_CS,
	 .max_speed_hz = 24 * 1000 * 1000,
	 .mode = SPI_CPOL | SPI_CPHA,
	 },
};

static struct spi_board_info openhone_spi_devices4cmmb[] = {
	{
	 .modalias = "cmmb-dev",	// "spidev" --> spidev_spi
	 .chip_select = SPRD_3RDPARTY_SPI_CMMB_CS,
	 .max_speed_hz = 8 * 1000 * 1000,
	 .mode = SPI_CPOL | SPI_CPHA,
	 },
};

static u64 spi_dmamask = DMA_BIT_MASK(32);
static struct resource spi_resources[][2] = {
               {
                       {
                               .start = SPRD_SPI0_PHYS,
                               .end = SPRD_SPI0_PHYS + SZ_4K - 1,
                               .flags = IORESOURCE_MEM,
                       },
                       {
                               .start = IRQ_SPI0_INT,
                               .end = IRQ_SPI0_INT,
                               .flags = IORESOURCE_IRQ,
                       },
               },
               {
                       {
                               .start = SPRD_SPI1_PHYS,
                               .end = SPRD_SPI1_PHYS + SZ_4K - 1,
                               .flags = IORESOURCE_MEM,
                       },

                       {
                               .start = IRQ_SPI1_INT,
                               .end = IRQ_SPI1_INT,
                               .flags = IORESOURCE_IRQ,
                       },
               },
};

static struct platform_device sprd_spi_controller_device[] = {
       {
               .name = "sprd_spi",
               .id = 0,
               .dev = {
                       .dma_mask = &spi_dmamask,
                       .coherent_dma_mask = DMA_BIT_MASK(32),
                       },
               .resource = spi_resources[0],
               .num_resources = ARRAY_SIZE(spi_resources[0]),
       },
       {
               .name = "sprd_spi",
               .id = 1,
               .dev = {
                       .dma_mask = &spi_dmamask,
                       .coherent_dma_mask = DMA_BIT_MASK(32),
                       },
               .resource = spi_resources[1],
               .num_resources = ARRAY_SIZE(spi_resources[1]),
       },

};


static unsigned long spi_func_cfg[] = {
	MFP_CFG_X(SPI_CLK, AF0, DS3, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_DI, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_DO, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),

        /* spi1 pin config*/
        MFP_CFG_X(TRACECTRL, AF2, DS3, F_PULL_UP, S_PULL_UP, IO_NONE),
        MFP_CFG_X(TRACECLK, AF2, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
        MFP_CFG_X(TRACEDAT0, AF2, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),

#if 1
	/* configure cs pin to normal gpio */
	MFP_CFG_X(SPI_CSN0, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
	MFP_CFG_X(SPI_CSN1, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        MFP_CFG_X(TRACEDAT1, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        MFP_CFG_X(TRACEDAT2, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
#else
	/* configure cs pin to spi csx */
	MFP_CFG_X(SPI_CSN0, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(SPI_CSN1, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
        MFP_CFG_X(TRACEDAT1, AF2, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        MFP_CFG_X(TRACEDAT2, AF2, DS1, F_PULL_UP, S_PULL_UP, IO_OE),

#endif
};

#if 0
static unsigned long bt_func_cfg[] = {
	MFP_CFG_X(U0RTS, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
	MFP_CFG_X(U0CTS, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
};
#endif


static struct spi_device *sprd_spi_device_register(int master_bus_num,
						   struct spi_board_info *chip,
						   int type)
{
	int i, gpio;

	if (master_bus_num < 0)
		master_bus_num = SPRD_3RDPARTY_SPI_MASTER_BUS_NUM;

	if (!spi_busnum_to_master(master_bus_num)) {
		printk(KERN_WARNING "%s: no [ %d ] spi master\n", __func__,
		       master_bus_num);
		return NULL;
	}

	if (chip == NULL) {
		switch (type) {
		case SPRD_3RDPARTY_SPI_WIFI_CS:
			chip = openhone_spi_devices4wifi;
			break;
		case SPRD_3RDPARTY_SPI_CMMB_CS:
			chip = openhone_spi_devices4cmmb;
			break;
		}
	}

	for (i = 0; i < 1; i++) {
		if (chip[i].chip_select == -1 || chip[i].chip_select > 64) {
			switch (type) {
			case SPRD_3RDPARTY_SPI_WIFI_CS:
				chip[i].chip_select = SPRD_3RDPARTY_SPI_WIFI_CS;
				break;
			case SPRD_3RDPARTY_SPI_CMMB_CS:
				chip[i].chip_select = SPRD_3RDPARTY_SPI_CMMB_CS;
				break;
			}
		}
		chip[i].irq = IRQ_SPI0_INT;

		gpio = spi_cs_gpio[chip[i].chip_select];
		chip[i].controller_data = (void *)gpio;
	}

	return spi_new_device(spi_busnum_to_master(master_bus_num), chip);
}

struct spi_device *sprd_spi_wifi_device_register(int master_bus_num,
						 struct spi_board_info *chip)
{
	return sprd_spi_device_register(master_bus_num, chip,
					SPRD_3RDPARTY_SPI_WIFI_CS);
}

EXPORT_SYMBOL_GPL(sprd_spi_wifi_device_register);

struct spi_device *sprd_spi_cmmb_device_register(int master_bus_num,
						 struct spi_board_info *chip)
{
	return sprd_spi_device_register(master_bus_num, chip,
					SPRD_3RDPARTY_SPI_CMMB_CS);
}

EXPORT_SYMBOL_GPL(sprd_spi_cmmb_device_register);

int sprd_spi_cs_hook(int cs_gpio, int dir)
{
	return cs_gpio;
}

EXPORT_SYMBOL_GPL(sprd_spi_cs_hook);

static void sprd_spi_init(void)
{
	int i;
	int gpio;
	struct spi_board_info *chip = openhone_spi_devices;
	sprd_mfp_config(spi_func_cfg, ARRAY_SIZE(spi_func_cfg));
	for (i = 0; i < ARRAY_SIZE(openhone_spi_devices); i++) {
		gpio = spi_cs_gpio[chip[i].chip_select];
		chip[i].controller_data = (void *)gpio;
	}

        platform_device_register(&sprd_spi_controller_device[0]);
        platform_device_register(&sprd_spi_controller_device[1]);

        spi_register_board_info(chip, ARRAY_SIZE(openhone_spi_devices));

}
#else
static void sprd_spi_init(void)
{
}
#endif


static void sprd_wifildo_init(void)
{
//---VDD-WIFI1  1.8V

   LDO_TurnOnLDO(LDO_LDO_WIF1);
   LDO_SetVoltLevel(LDO_LDO_WIF1,LDO_VOLT_LEVEL2);

//---VDD-SDIO1  1.8V

   LDO_TurnOnLDO(LDO_LDO_SDIO1);
   LDO_SetVoltLevel(LDO_LDO_SDIO1,LDO_VOLT_LEVEL3);


}
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

int __init LDO_Init(void);
static void __init chip_init(void)
{
//    ANA_REG_SET(ANA_ADIE_CHIP_ID,0);
	/* setup pins configration when LDO shutdown */
	__raw_writel(0x1fff00, PIN_CTL_REG);

	__raw_bits_and(~(BIT_29 | BIT_30), AHB_REG_BASE);//disable axi bus monitor clock
	__raw_bits_and(~(BIT_11 | BIT_7), AHB_REG_BASE);//disable bus monitor clock
}
void __init eic_init(void);

static unsigned long clk_aux0_pin_cfg_on[] = {
            MFP_CFG_X(CLK_AUX0, AF0, DS3, F_PULL_NONE, S_PULL_NONE, IO_OE),
};

static unsigned long clk_aux0_pin_cfg_off[] = {
            MFP_CFG_X(CLK_AUX0, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_Z),
};


void clk_32k_config(int is_on)
{
	//config 32k clk_aux0
    if (is_on){
        sprd_mfp_config(clk_aux0_pin_cfg_on, ARRAY_SIZE(clk_aux0_pin_cfg_on));

        __raw_bits_and(~(BIT_10|BIT_11),SPRD_GREG_BASE+0x0070);
    	__raw_bits_or(BIT_11,SPRD_GREG_BASE+0x70);

    	__raw_bits_and(~(0XFF),SPRD_GREG_BASE+0x0018);
    	__raw_bits_or(BIT_10,SPRD_GREG_BASE+0x0018);

    }
    else{
        __raw_bits_and(~BIT_10,SPRD_GREG_BASE+0x0018);
        sprd_mfp_config(clk_aux0_pin_cfg_off, ARRAY_SIZE(clk_aux0_pin_cfg_off));
        
    }
}
EXPORT_SYMBOL_GPL(clk_32k_config);

void gps_power_ctl(int is_on)
{
    printk("%s is_on=%d\n",__func__,is_on);
    if (is_on){
    	LDO_TurnOnLDO(LDO_LDO_WIF0);
    	LDO_SetVoltLevel(LDO_LDO_WIF0,LDO_VOLT_LEVEL2);
    }
    else {
    	LDO_TurnOffLDO(LDO_LDO_WIF0);
    }
}
EXPORT_SYMBOL_GPL(gps_power_ctl);

extern void sc8810_pin_map_init(void);
static void __init openphone_init(void)
{
	chip_init();
	ADI_init();
	LDO_Init();
	sprd_pin_map_init();
	sprd_gpio_init();
	sprd_i2c_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sprd_add_devices();
	eic_init();
	sprd_add_sdio_device();
	sprd_add_otg_device();
	sprd_gadget_init();
	sprd_add_dcam_device();
	sprd_spi_init();
        sprd_wifildo_init();  //WIFI  vreg  ana  and digital
	sprd_charger_init();
	sprd_ramconsole_init();
}

static void __init openphone_map_io(void)
{
	sprd_map_common_io();
	sprd_ramconsole_reserve_sdram();	
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

MACHINE_START(SC8810OPENPHONE, "SP8810")
/* UART for LL DEBUG */
	.phys_io = SPRD_SERIAL1_PHYS,
	.io_pg_offst =((SPRD_SERIAL1_BASE) >> 18) & 0xfffc,
	.map_io = openphone_map_io,
	.init_irq = openphone_init_irq,
	.init_machine = openphone_init,
	.timer = &sprd_timer,
	.fixup = openphone_fixup, 
MACHINE_END
