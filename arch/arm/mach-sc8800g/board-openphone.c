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

#include <linux/i2c.h>

 #include <linux/dcam_sensor.h>

//move pin configuration to pin_map_openphone.c
#define GPIO_DIRECTION_OUTPUT (1<<30)
#define GPIO_DIRECTION_INPUT  (1<<29)
#define GPIO_DESC_MFP_OBSOLETE (0x0)
#define GPIO_INDEX_MAX (0xffff)

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
       .size = SPRD_PMEM_SIZE,
       .no_allocator = 0,
       .cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
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

//i2c pad:  the high two bit of the addr is the pad control bit
static struct i2c_board_info __initdata openphone_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO(SENSOR_MAIN_I2C_NAME,SENSOR_MAIN_I2C_ADDR|0x8000),
    },
   {
        I2C_BOARD_INFO(SENSOR_SUB_I2C_NAME,SENSOR_SUB_I2C_ADDR|0x8000),
    },
    
#ifdef CONFIG_SENSORS_MMC31XX
#define MMC31XX_I2C_NAME		"mmc31xx"
#define MMC31XX_I2C_ADDR		0x30
    {
        I2C_BOARD_INFO(MMC31XX_I2C_NAME, MMC31XX_I2C_ADDR | 0x4000),
    },
#endif

#ifdef CONFIG_SENSORS_MXC622X
#define MXC622X_I2C_NAME		"mxc622x"
#define MXC622X_I2C_ADDR		0x15
    {
        I2C_BOARD_INFO(MXC622X_I2C_NAME,MXC622X_I2C_ADDR|0x4000),
    },
#endif
};

#if defined(CONFIG_SPI_SC88XX) || defined(CONFIG_SPI_SC88XX_MODULE)
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/mfp.h>

#define SPRD_3RDPARTY_CLOCK_WIFI_FREQ_SPEED_NORMAL      32 * 1000 * 1000
#define SPRD_3RDPARTY_CLOCK_WIFI_FREQ_SPEED_HIGH        32 * 1000 * 1000

#define SPRD_3RDPARTY_SPI_MASTER_BUS_NUM    0
#define SPRD_3RDPARTY_SPI_MASTER_CS0_GPIO   32
#define SPRD_3RDPARTY_SPI_MASTER_CS1_GPIO   33
#define SPRD_3RDPARTY_SPI_MASTER_CS2_GPIO   32
#define SPRD_3RDPARTY_SPI_MASTER_CS3_GPIO   33

#define SPRD_3RDPARTY_SPI_WIFI_CS   2
#define SPRD_3RDPARTY_SPI_CMMB_CS   3

static int spi_cs_gpio[] = {
    [0] = SPRD_3RDPARTY_SPI_MASTER_CS0_GPIO, // cs = 0 , GPIO32 == SPI_CSN0
    [1] = SPRD_3RDPARTY_SPI_MASTER_CS1_GPIO, // cs = 1 , GPIO33 == SPI_CSN1
    [2] = SPRD_3RDPARTY_SPI_MASTER_CS2_GPIO, // to cs0
    [3] = SPRD_3RDPARTY_SPI_MASTER_CS3_GPIO, // to cs1
};

static struct spi_board_info openhone_spi_devices[] = {
    {
        .modalias       = "spidev", // "spidev" --> spidev_spi
        .chip_select    = 0,
        .max_speed_hz   = 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
    {
        .modalias       = "spidev", // "spidev" --> spidev_spi
        .chip_select    = 1,
        .max_speed_hz   = 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
};

static struct spi_board_info openhone_spi_devices4wifi[] = {
    {
        .modalias       = "spi_slot0", // "spidev" --> spidev_spi
        .chip_select    = SPRD_3RDPARTY_SPI_WIFI_CS,
        .max_speed_hz   = SPRD_3RDPARTY_CLOCK_WIFI_FREQ_SPEED_HIGH,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
};

static struct spi_board_info openhone_spi_devices4cmmb[] = {
#ifdef CONFIG_CMMB_INNOFIDEI
    {
        .modalias       = "cmmb-dev", // "spidev" --> spidev_spi
        .chip_select    = SPRD_3RDPARTY_SPI_CMMB_CS,
        .max_speed_hz   = 8 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
#else  /* siano or other.. */
    {
        .modalias       = "cmmb-dev", // "spidev" --> spidev_spi
        .chip_select    = SPRD_3RDPARTY_SPI_CMMB_CS,
        .max_speed_hz   = 10 * 1000 * 1000,
        .mode           = SPI_CPOL | SPI_CPHA,
    },
#endif
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

#define SPRD_3RDPARTY_GPIO_WIFI_POWER       106
#define SPRD_3RDPARTY_GPIO_WIFI_RESET       140
#define SPRD_3RDPARTY_GPIO_WIFI_PWD         99
#define SPRD_3RDPARTY_GPIO_WIFI_WAKE        139
#define SPRD_3RDPARTY_GPIO_WIFI_IRQ         141
#define SPRD_3RDPARTY_GPIO_BT_POWER         -1
#define SPRD_3RDPARTY_GPIO_BT_RESET         90
#define SPRD_3RDPARTY_GPIO_BT_RTS           42
#ifdef CONFIG_CMMB_INNOFIDEI
#define SPRD_3RDPARTY_GPIO_CMMB_POWER       135
#define SPRD_3RDPARTY_GPIO_CMMB_RESET       94
#define SPRD_3RDPARTY_GPIO_CMMB_IRQ         93
#else
#define SPRD_3RDPARTY_GPIO_CMMB_POWER       -1
#define SPRD_3RDPARTY_GPIO_CMMB_RESET       -1
#define SPRD_3RDPARTY_GPIO_CMMB_IRQ         -1
#endif
#define SPRD_3RDPARTY_GPIO_TP_PWR             59   //not used
#define SPRD_3RDPARTY_GPIO_TP_RST              144
#define SPRD_3RDPARTY_GPIO_TP_IRQ              143

int sprd_3rdparty_gpio_wifi_power = SPRD_3RDPARTY_GPIO_WIFI_POWER;
int sprd_3rdparty_gpio_wifi_reset = SPRD_3RDPARTY_GPIO_WIFI_RESET;
int sprd_3rdparty_gpio_wifi_pwd   = SPRD_3RDPARTY_GPIO_WIFI_PWD;
int sprd_3rdparty_gpio_wifi_wake  = SPRD_3RDPARTY_GPIO_WIFI_WAKE;
int sprd_3rdparty_gpio_wifi_irq   = SPRD_3RDPARTY_GPIO_WIFI_IRQ;
int sprd_3rdparty_gpio_bt_power   = SPRD_3RDPARTY_GPIO_BT_POWER;
int sprd_3rdparty_gpio_bt_reset   = SPRD_3RDPARTY_GPIO_BT_RESET;
int sprd_3rdparty_gpio_bt_rts     = SPRD_3RDPARTY_GPIO_BT_RTS;
int sprd_3rdparty_gpio_cmmb_power = SPRD_3RDPARTY_GPIO_CMMB_POWER;
int sprd_3rdparty_gpio_cmmb_reset = SPRD_3RDPARTY_GPIO_CMMB_RESET;
int sprd_3rdparty_gpio_cmmb_irq   = SPRD_3RDPARTY_GPIO_CMMB_IRQ;
int sprd_3rdparty_gpio_tp_pwr   = SPRD_3RDPARTY_GPIO_TP_PWR;
int sprd_3rdparty_gpio_tp_rst   = SPRD_3RDPARTY_GPIO_TP_RST;
int sprd_3rdparty_gpio_tp_irq   = SPRD_3RDPARTY_GPIO_TP_IRQ;

int sprd_3rdparty_clock_wifi_freq_speed_normal = SPRD_3RDPARTY_CLOCK_WIFI_FREQ_SPEED_NORMAL;
int sprd_3rdparty_clock_wifi_freq_speed_high = SPRD_3RDPARTY_CLOCK_WIFI_FREQ_SPEED_HIGH;

EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_wifi_power);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_wifi_reset);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_wifi_pwd);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_wifi_wake);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_wifi_irq);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_bt_power);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_bt_reset);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_bt_rts);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_cmmb_power);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_cmmb_reset);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_cmmb_irq);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_tp_pwr);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_tp_rst);
EXPORT_SYMBOL_GPL(sprd_3rdparty_gpio_tp_irq);

EXPORT_SYMBOL_GPL(sprd_3rdparty_clock_wifi_freq_speed_normal);
EXPORT_SYMBOL_GPL(sprd_3rdparty_clock_wifi_freq_speed_high);

static struct gpio_desc gpio_func_cfg[] = {
    {
       //MFP_CFG_X(XTL_EN, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE), // wifi_power_io - also Bluetooth power
       //SPRD_3RDPARTY_GPIO_WIFI_POWER | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
       
        GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_WIFI_POWER | GPIO_OUTPUT_DEFAUT_VALUE_HIGH | GPIO_DIRECTION_OUTPUT,
	"wifi power"
    },
    {
       // MFP_CFG_X(RFCTL9    , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE), // wifi_pwd_io
        //SPRD_3RDPARTY_GPIO_WIFI_PWD, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_WIFI_PWD | GPIO_DIRECTION_OUTPUT, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi pwd"
    },
    {
        //MFP_CFG_X(GPIO139, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        //SPRD_3RDPARTY_GPIO_WIFI_WAKE, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_WIFI_WAKE | GPIO_DIRECTION_OUTPUT, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi wake"
    },
    {
        //MFP_CFG_X(GPIO140, AF0, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
        //SPRD_3RDPARTY_GPIO_WIFI_RESET, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_WIFI_RESET | GPIO_DIRECTION_OUTPUT, // | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
        "wifi reset"
    },
    {
        //MFP_CFG_X(RFCTL0 , AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE), // BT_RESET
        //SPRD_3RDPARTY_GPIO_BT_RESET | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_BT_RESET | GPIO_OUTPUT_DEFAUT_VALUE_HIGH | GPIO_DIRECTION_OUTPUT,
        "BT reset"
    },
    {
        //MFP_CFG_X(U0RTS  , AF3, DS1, F_PULL_DOWN, S_PULL_UP, IO_OE), // BT_RTS
        //SPRD_3RDPARTY_GPIO_BT_RTS,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_BT_RTS | GPIO_DIRECTION_OUTPUT,
        "BT RTS"
    },
    {
       //MFP_CFG_X(GPIO135    , AF0, DS1, F_PULL_NONE, S_PULL_UP, IO_OE), // cmmb power
       //135,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_CMMB_POWER | GPIO_DIRECTION_OUTPUT,
        "demod power"
    },
    {
        //MFP_CFG_X(RFCTL4    , AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE), // cmmb reset
        //94,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_CMMB_RESET | GPIO_DIRECTION_OUTPUT,
        "demod reset"
    },
    {
        //MFP_CFG_X(RFCTL3    , AF3, DS1, F_PULL_NONE, S_PULL_DOWN, IO_IE), // cmmb interrupt
        //93 | GPIO_OUTPUT_DEFAUT_VALUE_HIGH,
	
	GPIO_DESC_MFP_OBSOLETE,
        SPRD_3RDPARTY_GPIO_CMMB_IRQ | GPIO_DIRECTION_INPUT,
        "demod int"
   }
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

#if 0
static unsigned long bt_func_cfg[] = {
    MFP_CFG_X(U0RTS     , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
    MFP_CFG_X(U0CTS     , AF0, DS1, F_PULL_UP, S_PULL_UP, IO_NONE),
};
#endif

static struct spi_device *sprd_spi_device_register(int master_bus_num, struct spi_board_info *chip, int type)
{
    int i, gpio;

    if (master_bus_num < 0)
        master_bus_num = SPRD_3RDPARTY_SPI_MASTER_BUS_NUM;

    if (!spi_busnum_to_master(master_bus_num)) {
        printk(KERN_WARNING "%s: no [ %d ] spi master\n", __func__, master_bus_num);
        return NULL;
    }

    if (chip == NULL) {
        switch (type) {
            case SPRD_3RDPARTY_SPI_WIFI_CS: chip = openhone_spi_devices4wifi; break;
            case SPRD_3RDPARTY_SPI_CMMB_CS: chip = openhone_spi_devices4cmmb; break;
        }
    }

    for (i = 0; i < 1; i++) {
        if (chip[i].chip_select == -1 || chip[i].chip_select > 64) {
            switch (type) {
                case SPRD_3RDPARTY_SPI_WIFI_CS: chip[i].chip_select = SPRD_3RDPARTY_SPI_WIFI_CS; break;
                case SPRD_3RDPARTY_SPI_CMMB_CS: chip[i].chip_select = SPRD_3RDPARTY_SPI_CMMB_CS; break;
            }
        }

        // if (chip[i].irq < 0 || chip[i].irq > INT_MAX) {
            chip[i].irq = IRQ_SPI_INT;
        // }

        gpio = spi_cs_gpio[chip[i].chip_select];
        chip[i].controller_data = (void*)gpio;
    }

    return spi_new_device(spi_busnum_to_master(master_bus_num), chip);
}

struct spi_device *sprd_spi_wifi_device_register(int master_bus_num, struct spi_board_info *chip)
{
    return sprd_spi_device_register(master_bus_num, chip, SPRD_3RDPARTY_SPI_WIFI_CS);
}
EXPORT_SYMBOL_GPL(sprd_spi_wifi_device_register);

struct spi_device *sprd_spi_cmmb_device_register(int master_bus_num, struct spi_board_info *chip)
{
    return sprd_spi_device_register(master_bus_num, chip, SPRD_3RDPARTY_SPI_CMMB_CS);
}
EXPORT_SYMBOL_GPL(sprd_spi_cmmb_device_register);

int sprd_spi_cs_hook(int cs_gpio, int dir)
{
    return cs_gpio;
}
EXPORT_SYMBOL_GPL(sprd_spi_cs_hook);

static void sprd_spi_init(void)
{
    int gpio, value;
    struct gpio_desc *gd;
    int i, nr_chip = ARRAY_SIZE(openhone_spi_devices);
    struct spi_board_info *chip = openhone_spi_devices;

    for (i = 0; i < ARRAY_SIZE(gpio_func_cfg); i++) {
        gd = &gpio_func_cfg[i];
        /*
        sprd_mfp_config(&gd->mfp, 1);
        */
        gpio = (gd->io & ~GPIO_OUTPUT_DEFAUT_VALUE_HIGH) & GPIO_INDEX_MAX;
        value = !!(gd->io & GPIO_OUTPUT_DEFAUT_VALUE_HIGH);
        if (gpio_request(gpio, gd->desc))
            printk(KERN_WARNING "%s : [%s] gpio %d request failed!\n", __func__, gd->desc, gpio);
        if (gd->io & GPIO_DIRECTION_OUTPUT){
            gpio_direction_output(gpio, value);
        } else if (gd->io & GPIO_DIRECTION_INPUT) {
            gpio_direction_input(gpio);
        } else {
            //printk(KERN_WARNING "%s : not support gpio mode!\n", __func__);
            printk(KERN_WARNING "%s : not support gpio direction!\n", __func__);
        }
    }

    //initialize SPI pin_map to pin_map_openphone.c
    //sprd_mfp_config(spi_func_cfg, ARRAY_SIZE(spi_func_cfg));
    // sprd_mfp_config(bt_func_cfg, ARRAY_SIZE(bt_func_cfg));
    ANA_REG_OR (ANA_LED_CTL, BIT_14); // also enable 26MHz clock for bt when RF chip dsp code sleep

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

int __init LDO_Init(void);
static void __init chip_init(void)
{
    ANA_REG_SET(ANA_ADIE_CHIP_ID,0);
    /* setup pins configration when LDO shutdown*/
    __raw_writel(0x1fff00, PIN_CTL_REG);
}

unsigned long sdram_plimit;
extern void sc8800g_pin_map_init(void);
static void __init openphone_init(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_pdata.start = SPRD_PMEM_BASE;
	android_pmem_adsp_pdata.start = SPRD_PMEM_ADSP_BASE;
#endif
	pr_info("sdram_plimit: 0x%x\n", sdram_plimit);
	chip_init();
	//	ADI_init();
	LDO_Init();
	sc8800g_pin_map_init();
	i2c_register_board_info(1,openphone_i2c_boardinfo,ARRAY_SIZE(openphone_i2c_boardinfo));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sprd_add_devices();
	sprd_gpio_init();
	sprd_add_sdio_device();
	sprd_add_otg_device();
	sprd_gadget_init();
	sprd_add_dcam_device();
	sprd_spi_init();
	sprd_charger_init();
	sprd_ramconsole_init();
}

static void __init openphone_map_io(void)
{
	sdram_plimit = get_sdram_plimit();
	sprd_map_common_io();
	sprd_ramconsole_reserve_sdram();
	sprd_pmem_reserve_sdram();
}

extern unsigned long phys_initrd_start;
extern unsigned long phys_initrd_size;

static void __init
openphone_fixup(struct machine_desc *desc, struct tag *tag,

	    char **cmdline, struct meminfo *mi)
{
#ifdef CONFIG_BLK_DEV_INITRD

/*	
	phys_initrd_start = 0x04b00000;
 	phys_initrd_size = 2*1024*1024;
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
