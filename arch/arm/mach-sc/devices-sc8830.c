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
#include <linux/platform_device.h>
#include <linux/ion.h>
#include <linux/input.h>
#include <linux/mmc/sdhci.h>
#include <linux/gpio.h>

#include <asm/pmu.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/pstore_ram.h>
#include <linux/sprd_iommu.h>
#include <linux/headset_sprd.h>

#include "devices.h"

static struct resource sprd_memnand_system_resources[] = {
	[0] = {
		.start = SPRD_MEMNAND_SYSTEM_BASE,
		.end = SPRD_MEMNAND_SYSTEM_BASE + SPRD_MEMNAND_SYSTEM_SIZE-1,
		.name = "goldfish_memnand",
		.flags = IORESOURCE_MEM,
	},
};
struct platform_device sprd_memnand_system_device  = {
	.name           = "goldfish_memnand",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_memnand_system_resources),
	.resource       = sprd_memnand_system_resources,
	.dev        = {.platform_data="system"},
};
static struct resource sprd_memnand_userdata_resources[] = {
	[0] = {
		.start = SPRD_MEMNAND_USERDATA_BASE,
		.end = SPRD_MEMNAND_USERDATA_BASE + SPRD_MEMNAND_USERDATA_SIZE-1,
		.name = "goldfish_memnand",
		.flags = IORESOURCE_MEM,
	},
};
struct platform_device sprd_memnand_userdata_device  = {
	.name           = "goldfish_memnand",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_memnand_userdata_resources),
	.resource       = sprd_memnand_userdata_resources,
	.dev        = {.platform_data="userdata"},
};



static struct resource sprd_memnand_cache_resources[] = {
	[0] = {
		.start = SPRD_MEMNAND_CACHE_BASE,
		.end = SPRD_MEMNAND_CACHE_BASE + SPRD_MEMNAND_CACHE_SIZE-1,
		.name = "goldfish_memnand",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device sprd_memnand_cache_device  = {
	.name           = "goldfish_memnand",
	.id             =  2,
	.num_resources  = ARRAY_SIZE(sprd_memnand_cache_resources),
	.resource       = sprd_memnand_cache_resources,
	.dev        = {.platform_data="cache"},
};

static struct resource sprd_serial_resources0[] = {
	[0] = {
		.start = SPRD_UART0_BASE,
		.end = SPRD_UART0_BASE + SPRD_UART0_SIZE-1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER0_INT,
		.end = IRQ_SER0_INT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_serial_device0 = {
	.name           = "serial_sprd",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources0),
	.resource       = sprd_serial_resources0,
};

static struct resource sprd_serial_resources1[] = {
	[0] = {
		.start = SPRD_UART1_BASE,
		.end = SPRD_UART1_BASE + SPRD_UART1_SIZE-1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER1_INT,
		.end = IRQ_SER1_INT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_serial_device1 = {
	.name           = "serial_sprd",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources1),
	.resource       = sprd_serial_resources1,
};

static struct resource sprd_serial_resources2[] = {
	[0] = {
		.start = SPRD_UART2_BASE,
		.end = SPRD_UART2_BASE + SPRD_UART2_SIZE - 1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER2_INT,
		.end = IRQ_SER2_INT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device sprd_serial_device2 = {
	.name           = "serial_sprd",
	.id             =  2,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources2),
	.resource       = sprd_serial_resources2,
};

static struct resource sprd_serial_resources3[] = {
	[0] = {
		.start = SPRD_UART3_BASE,
		.end = SPRD_UART3_BASE + SPRD_UART3_SIZE - 1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER3_INT,
		.end = IRQ_SER3_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_serial_device3 = {
	.name           = "serial_sprd",
	.id             =  3,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources3),
	.resource       = sprd_serial_resources3,
};

static struct resource resources_rtc[] = {
	[0] = {
		.start	= IRQ_ANA_RTC_INT,
		.end	= IRQ_ANA_RTC_INT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device sprd_device_rtc= {
	.name	= "sprd_rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_rtc),
	.resource	= resources_rtc,
};

static struct eic_gpio_resource sprd_gpio_resource[] = {
       [ENUM_ID_D_GPIO] = {
               .irq            = IRQ_GPIO_INT,
               .base_addr      = CTL_GPIO_BASE,
               .chip_base      = D_GPIO_START,
               .chip_ngpio     = D_GPIO_NR,
       },
       [ENUM_ID_D_EIC] = {
               .irq            = IRQ_EIC_INT,
               .base_addr      = CTL_EIC_BASE,
               .chip_base      = D_EIC_START,
               .chip_ngpio     = D_EIC_NR,
       },
       [ENUM_ID_A_GPIO] = {
               .irq            = IRQ_ANA_GPIO_INT,
               .base_addr      = ANA_CTL_GPIO_BASE,
               .chip_base      = A_GPIO_START,
               .chip_ngpio     = A_GPIO_NR,
       },
       [ENUM_ID_A_EIC] = {
               .irq            = IRQ_ANA_EIC_INT,
               .base_addr      = ANA_CTL_EIC_BASE,
               .chip_base      = A_EIC_START,
               .chip_ngpio     = A_EIC_NR,
       },
};

struct platform_device sprd_eic_gpio_device = {
       .name = "eic-gpio",
       .id = -1,
       .dev = { .platform_data = sprd_gpio_resource },
};

static struct resource sprd_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end = 7,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.start	= SPRD_NFC_BASE,
		.end = SPRD_NFC_BASE + SPRD_NFC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device sprd_nand_device = {
	.name		= "sprd-nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sprd_nand_resources),
	.resource	= sprd_nand_resources,
};

static struct resource sprd_hwspinlock_resources0[] = {
	[0] = {
		.start	= SPRD_HWLOCK1_BASE,
		.end = SPRD_HWLOCK1_BASE + SPRD_HWLOCK1_SIZE- 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sprd_hwspinlock_resources1[] = {
	[0] = {
		.start	= SPRD_HWLOCK0_BASE,
		.end = SPRD_HWLOCK0_BASE + SPRD_HWLOCK0_SIZE- 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device sprd_hwspinlock_device0 = {
	.name		= "sci_hwspinlock",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_hwspinlock_resources0),
	.resource	= sprd_hwspinlock_resources0,
};

struct platform_device sprd_hwspinlock_device1 = {
	.name		= "sci_hwspinlock",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(sprd_hwspinlock_resources1),
	.resource	= sprd_hwspinlock_resources1,
};

static struct resource sprd_lcd_resources[] = {
	[0] = {
		.start = SPRD_LCDC_BASE,
		.end = SPRD_LCDC_BASE + SPRD_LCDC_SIZE - 1,
		.name = "lcd_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_DISPC0_INT,
		.end = IRQ_DISPC0_INT,
		.flags = IORESOURCE_IRQ,
	},
	
	[2] = {
		.start = IRQ_DISPC1_INT,
		.end = IRQ_DISPC1_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_lcd_device0 = {
	.name           = "sprd_fb",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_lcd_resources),
	.resource       = sprd_lcd_resources,
};
struct platform_device sprd_lcd_device1 = {
	.name           = "sprd_fb",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_lcd_resources),
	.resource       = sprd_lcd_resources,
};
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

/*if the backlight is driven by pwm, please config the pwm info
 *if the backlight is driven by PWMD, the pwm index is 3 as following
 */
struct resource sprd_bl_resource[] = {
	[0] = {
		.start	= 3,
		.end	= 3,
		.flags	= IORESOURCE_IO,
	},
};

struct platform_device sprd_backlight_device = {
	.name           = "sprd_backlight",
	.id             =  -1,
	.num_resources	= ARRAY_SIZE(sprd_bl_resource),
	.resource	= sprd_bl_resource,
};

static struct resource sprd_i2c_resources0[] = {
	[0] = {
		.start = SPRD_I2C0_BASE,
		.end   = SPRD_I2C0_BASE + SPRD_I2C0_SIZE -1,
		.name  = "i2c0_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C0_INT,
		.end   = IRQ_I2C0_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device0 = {
	.name           = "sprd-i2c",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources0),
	.resource       = sprd_i2c_resources0,
};


static struct resource sprd_i2c_resources1[] = {
	[0] = {
		.start = SPRD_I2C1_BASE,
		.end   = SPRD_I2C1_BASE + SZ_4K -1,
		.name  = "i2c1_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C1_INT,
		.end   = IRQ_I2C1_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device1 = {
	.name           = "sprd-i2c",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources1),
	.resource       = sprd_i2c_resources1,
};


static struct resource sprd_i2c_resources2[] = {
	[0] = {
		.start = SPRD_I2C2_BASE,
		.end   = SPRD_I2C2_BASE + SZ_4K -1,
		.name  = "i2c2_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C2_INT,
		.end   = IRQ_I2C2_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device2 = {
	.name           = "sprd-i2c",
	.id             =  2,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources2),
	.resource       = sprd_i2c_resources2,
};


static struct resource sprd_i2c_resources3[] = {
	[0] = {
		.start = SPRD_I2C3_BASE,
		.end   = SPRD_I2C3_BASE + SZ_4K -1,
		.name  = "i2c3_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C3_INT,
		.end   = IRQ_I2C3_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device3 = {
	.name           = "sprd-i2c",
	.id             =  3,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources3),
	.resource       = sprd_i2c_resources3,
};

/* 8810 SPI devices.  */
static struct resource spi0_resources[] = {
    [0] = {
        .start = SPRD_SPI0_BASE,
        .end = SPRD_SPI0_BASE + SPRD_SPI0_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_SPI0_INT,
        .end = IRQ_SPI0_INT,
        .flags = IORESOURCE_IRQ,
    },
};


static struct resource spi1_resources[] = {
    [0] = {
        .start = SPRD_SPI1_BASE,
        .end = SPRD_SPI1_BASE + SPRD_SPI1_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_SPI1_INT,
        .end = IRQ_SPI1_INT,
        .flags = IORESOURCE_IRQ,
    },
};

static struct resource spi2_resources[] = {
	[0] = {
	       .start = SPRD_SPI2_BASE,
	       .end = SPRD_SPI2_BASE + SPRD_SPI2_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SPI2_INT,
	       .end = IRQ_SPI2_INT,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device sprd_spi0_device = {
    .name = "sprd_spi",
    .id = 0,
    .resource = spi0_resources,
    .num_resources = ARRAY_SIZE(spi0_resources),
};

struct platform_device sprd_spi1_device = {
	.name = "sprd_spi",
	.id = 1,
	.resource = spi1_resources,
	.num_resources = ARRAY_SIZE(spi1_resources),
};

struct platform_device sprd_spi2_device = {
	.name = "sprd_spi",
	.id = 2,
	.resource = spi2_resources,
	.num_resources = ARRAY_SIZE(spi2_resources),
};

struct platform_device sprd_ahb_bm_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 0,
};
struct platform_device sprd_axi_bm0_device = {
	.name = "sprd_axi_busmonitor",
	.id = 0,
};
static struct resource sci_keypad_resources[] = {
	{
	        .start = IRQ_KPD_INT,
	        .end = IRQ_KPD_INT,
	        .flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_keypad_device = {
	.name = "sci-keypad",
	.id             = -1,
	.num_resources = ARRAY_SIZE(sci_keypad_resources),
	.resource = sci_keypad_resources,
};

static struct headset_button sprd_headset_button[] = {
	{
		.adc_min			= 0x0000,
		.adc_max			= 0x00C8,
		.code			= KEY_MEDIA,
	},
	{
		.adc_min			= 0x00C9,
		.adc_max			= 0x02BC,
		.code			= KEY_VOLUMEUP,
	},
	{
		.adc_min			= 0x02BD,
		.adc_max			= 0x0514,
		.code			= KEY_VOLUMEDOWN,
	},
};
static struct sprd_headset_buttons_platform_data sprd_headset_button_data = {
	.headset_button	= sprd_headset_button,
	.nbuttons	= ARRAY_SIZE(sprd_headset_button),
};
static struct sprd_headset_detect_platform_data sprd_headset_detect_data = {
	.switch_gpio	= HEADSET_SWITCH_GPIO,
	.detect_gpio	= HEADSET_DETECT_GPIO,
	.button_gpio	= HEADSET_BUTTON_GPIO,
	.detect_active_low	= 1,
	.button_active_low	= 1,
};

struct platform_device sprd_headset_button_device = {
	.name	= "headset-buttons",
	.id	= -1,
	.dev	= {
		.platform_data	= &sprd_headset_button_data,
	},
};

struct platform_device sprd_headset_detect_device = {
	.name	= "headset-detect",
	.id	= -1,
	.dev	= {
		.platform_data	= &sprd_headset_detect_data,
	},
};

static struct resource sprd_battery_resources[] = {
        [0] = {
                .start = EIC_CHARGER_DETECT,
                .end = EIC_CHARGER_DETECT,
                .name = "charger_detect",
                .flags = IORESOURCE_IO,
        },
        [1] = {
                .start = EIC_CHG_CV_STATE,
                .end = EIC_CHG_CV_STATE,
                .name = "chg_cv_state",
                .flags = IORESOURCE_IO,
        },
        [2] = {
                .start = EIC_VCHG_OVI,
                .end = EIC_VCHG_OVI,
                .name = "vchg_ovi",
                .flags = IORESOURCE_IO,
        }

};

struct platform_device sprd_battery_device = {
        .name           = "sprd-battery",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_battery_resources),
        .resource       = sprd_battery_resources,
};


#if defined(CONFIG_ARCH_SCX15)
static struct sprd_iommu_init_data sprd_iommu_gsp_data = {
	.id=0,
	.name="sprd_iommu_gsp",
	.iova_base=0x10000000,
	.iova_size=0x1000000,
	.pgt_base=SPRD_GSPMMU_BASE,
	.pgt_size=0x4000,
};

struct platform_device sprd_iommu_gsp_device = {
	.name = "sprd_iommu",
	.id = 0,
	.dev = {.platform_data = &sprd_iommu_gsp_data },
};

static struct sprd_iommu_init_data sprd_iommu_mm_data = {
	.id=1,
	.name="sprd_iommu_mm",
	.iova_base=0x20000000,
	.iova_size=0x4000000,
	.pgt_base=SPRD_MMMMU_BASE,
	.pgt_size=0x10000,
};

struct platform_device sprd_iommu_mm_device = {
	.name = "sprd_iommu",
	.id = 1,
	.dev = {.platform_data = &sprd_iommu_mm_data },
};
#endif

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
struct platform_device sprd_dcam_device = {
	.name		= "sprd_dcam",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_dcam_resources),
	.resource	= sprd_dcam_resources,
};
struct platform_device sprd_scale_device = {
	.name	= "sprd_scale",
	.id	= -1,
};

struct platform_device sprd_gsp_device =
{
    .name	= "sprd_gsp",
    .id	= -1,
};
struct platform_device sprd_rotation_device = {
	.name	= "sprd_rotation",
	.id	= -1,
};

struct platform_device sprd_sensor_device = {
	.name	= "sprd_sensor",
	.id	= -1,
};
struct platform_device sprd_isp_device = {
	.name = "sprd_isp",
	.id = -1,
};

struct platform_device sprd_dma_copy_device = {
	.name	= "sprd_dma_copy",
	.id	= -1,
};
static struct resource sprd_sdio0_resources[] = {
	[0] = {
		.start = SPRD_SDIO0_BASE,
		.end = SPRD_SDIO0_BASE + SPRD_SDIO0_SIZE-1,
		.name = "sdio0_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO0_INT,
		.end = IRQ_SDIO0_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct sprd_host_platdata sprd_sdio0_pdata = {
	.hw_name = "sprd-sdcard",
	.detect_gpio = 71,
	.vdd_name = "vddsd",
	.clk_name = "clk_sdio0",
	.clk_parent = "clk_192m",
	.max_clock = 192000000,
	.enb_bit = BIT_SDIO0_EB,
	.rst_bit = BIT_SDIO0_SOFT_RST,//FIXME:
};

struct platform_device sprd_sdio0_device = {
	.name           = "sprd-sdhci",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_sdio0_resources),
	.resource       = sprd_sdio0_resources,
	.dev = { .platform_data = &sprd_sdio0_pdata },
};

static struct resource sprd_sdio1_resources[] = {
	[0] = {
		.start = SPRD_SDIO1_BASE,
		.end = SPRD_SDIO1_BASE + SPRD_SDIO1_SIZE-1,
		.name = "sdio1_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO1_INT,
		.end = IRQ_SDIO1_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct sprd_host_platdata sprd_sdio1_pdata = {
	.hw_name = "sprd-sdio1",
	.clk_name = "clk_sdio1",
	.clk_parent = "clk_96m",
	.max_clock = 96000000,
	.enb_bit = BIT_SDIO1_EB,
	.rst_bit = BIT_SDIO1_SOFT_RST,
	.regs.is_valid = 1,
};

struct platform_device sprd_sdio1_device = {
	.name           = "sprd-sdhci",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_sdio1_resources),
	.resource       = sprd_sdio1_resources,
	.dev = { .platform_data = &sprd_sdio1_pdata },
};

struct platform_device sprd_vsp_device = {
	.name	= "sprd_vsp",
	.id	= -1,
};

struct platform_device sprd_jpg_device = {
	.name	= "sprd_jpg",
	.id	= -1,
};

#ifdef CONFIG_ION
#ifdef CONFIG_CMA
static struct ion_platform_data ion_pdata = {
        .nr = 2,
        .heaps = {
                {
                        .id     = ION_HEAP_TYPE_CARVEOUT,
                        .type   = ION_HEAP_TYPE_CUSTOM,
                        .name   = "ion_cma_heap",
                        .base   = SPRD_ION_BASE,
                        .size   = SPRD_ION_SIZE,
                },
                {
                        .id     = ION_HEAP_TYPE_CARVEOUT + 1,
                        .type   = ION_HEAP_TYPE_CUSTOM,
                        .name   = "ion_cma_heap_overlay",
                        .base   = SPRD_ION_OVERLAY_BASE,
                        .size   = SPRD_ION_OVERLAY_SIZE,
                },
        }
};
#else
static struct ion_platform_heap ion_pheaps[] = {
                {
                        .id     = ION_HEAP_TYPE_CARVEOUT,
                        .type   = ION_HEAP_TYPE_CARVEOUT,
                        .name   = "ion_carveout_heap",
                        .base   = SPRD_ION_BASE,
                        .size   = SPRD_ION_SIZE,
                },
#if CONFIG_SPRD_ION_OVERLAY_SIZE
                {
                        .id     = ION_HEAP_TYPE_CARVEOUT + 1,
                        .type   = ION_HEAP_TYPE_CARVEOUT,
                        .name   = "ion_carveout_heap_overlay",
                        .base   = SPRD_ION_OVERLAY_BASE,
                        .size   = SPRD_ION_OVERLAY_SIZE,
                },
#endif
};
static struct ion_platform_data ion_pdata = {
        .nr = sizeof(ion_pheaps)/sizeof(ion_pheaps[0]),
        .heaps = &ion_pheaps[0],
};
#endif

struct platform_device sprd_ion_dev = {
        .name = "ion-sprd",
        .id = -1,
        .dev = { .platform_data = &ion_pdata },
};
#endif

static struct resource sprd_sdio2_resources[] = {
	[0] = {
	       .start = SPRD_SDIO2_BASE,
	       .end = SPRD_SDIO2_BASE + SPRD_SDIO2_SIZE - 1,
	       .name = "sdio2_res",
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SDIO2_INT,
	       .end = IRQ_SDIO2_INT,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct sprd_host_platdata sprd_sdio2_pdata = {
	.hw_name = "sprd-sdio2",
	.clk_name = "clk_sdio2",
	.clk_parent = "clk_96m",
	.max_clock = 96000000,
	.enb_bit = BIT_SDIO2_EB,
	.rst_bit = BIT_SDIO2_SOFT_RST,
};

struct platform_device sprd_sdio2_device = {
	.name = "sprd-sdhci",
	.id = 2,
	.num_resources = ARRAY_SIZE(sprd_sdio2_resources),
	.resource = sprd_sdio2_resources,
	.dev = { .platform_data = &sprd_sdio2_pdata },
};
static struct resource sprd_emmc_resources[] = {
	[0] = {
	       .start = SPRD_EMMC_BASE,
	       .end = SPRD_EMMC_BASE + SPRD_EMMC_SIZE - 1,
	       .name = "sdio3_res",
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_EMMC_INT,
	       .end = IRQ_EMMC_INT,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct sprd_host_platdata sprd_emmc_pdata = {
	.hw_name = "sprd-emmc",
	.vdd_name = "vddemmcio",
	.vdd_ext_name = "vddemmccore",
	.clk_name = "clk_emmc",
	.clk_parent = "clk_192m",
	.max_clock = 192000000,
	.enb_bit = BIT_EMMC_EB,
	.rst_bit = BIT_EMMC_SOFT_RST,
	.regs.is_valid = 1,
};

struct platform_device sprd_emmc_device = {
	.name = "sprd-sdhci",
	.id = 3,
	.num_resources = ARRAY_SIZE(sprd_emmc_resources),
	.resource = sprd_emmc_resources,
	.dev = { .platform_data = &sprd_emmc_pdata },
};

struct sysdump_mem sprd_dump_mem[] = {
	{
		.paddr		= CONFIG_PHYS_OFFSET,
		.vaddr		= PAGE_OFFSET,
		.soff		= 0xffffffff,
		.size		= CPT_START_ADDR - CONFIG_PHYS_OFFSET,
		.type	 	= SYSDUMP_RAM,
	},
	{
		.paddr		= CPT_START_ADDR,
		.vaddr		= PAGE_OFFSET +
					(CPT_START_ADDR - CONFIG_PHYS_OFFSET),
		.soff		= 0xffffffff,
		.size		= CPT_TOTAL_SIZE,
#ifdef CONFIG_SIPC_TD
		.type		= SYSDUMP_MODEM,
#else
		.type		= SYSDUMP_RAM,
#endif
	},
	{
		.paddr		= CPT_START_ADDR + CPT_TOTAL_SIZE,
		.vaddr		= PAGE_OFFSET +
					(CPT_START_ADDR + CPT_TOTAL_SIZE - CONFIG_PHYS_OFFSET),
		.soff		= 0xffffffff,
		.size		= CPW_START_ADDR - (CPT_START_ADDR + CPT_TOTAL_SIZE),
		.type		= SYSDUMP_RAM,
	},
	{
		.paddr		= CPW_START_ADDR,
		.vaddr		= PAGE_OFFSET +
					(CPW_START_ADDR - CONFIG_PHYS_OFFSET),
		.soff		= 0xffffffff,
		.size		= CPW_TOTAL_SIZE,
#ifdef CONFIG_SIPC_WCDMA
		.type		= SYSDUMP_MODEM,
#else
		.type		= SYSDUMP_RAM,
#endif
	},
	{
		.paddr		= WCN_START_ADDR,
		.vaddr		= PAGE_OFFSET +
					(WCN_START_ADDR - CONFIG_PHYS_OFFSET),
		.soff		= 0xffffffff,
		.size		= WCN_TOTAL_SIZE,
//#ifdef CONFIG_SIPC_WCN
		.type		= SYSDUMP_MODEM,
//#else
//		.type		= SYSDUMP_RAM,
//#endif
	},
	{
		.paddr		= WCN_START_ADDR + WCN_TOTAL_SIZE,
		.vaddr		= PAGE_OFFSET +
					(WCN_START_ADDR + WCN_TOTAL_SIZE - CONFIG_PHYS_OFFSET),
		.soff		= 0xffffffff,
		.size		= 0, /* fill this dynamically according to real ram size */
		.type		= SYSDUMP_RAM,
	},
	{
		.paddr		= SPRD_AHB_PHYS,
		.vaddr		= SPRD_AHB_BASE,
		.soff		= 0x0,
		.size		= SPRD_AHB_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_INTC0_PHYS,
		.vaddr		= SPRD_INTC0_BASE,
		.soff		= 0x0,
		.size		= SPRD_INTC0_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GPTIMER0_PHYS,
		.vaddr		= SPRD_GPTIMER0_BASE,
		.soff		= 0x0,
		.size		= SPRD_GPTIMER0_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_ADI_PHYS,
		.vaddr		= SPRD_ADI_BASE,
		.soff		= 0x0,
		.size		= SPRD_ADI_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GPIO_PHYS,
		.vaddr		= SPRD_GPIO_BASE,
		.soff		= 0x0,
		.size		= SPRD_GPIO_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_EIC_PHYS,
		.vaddr		= SPRD_EIC_BASE,
		.soff		= 0x0,
		.size		= SPRD_EIC_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GREG_PHYS,
		.vaddr		= SPRD_GREG_BASE,
		.soff		= 0x0,
		.size		= SPRD_GREG_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
};
int sprd_dump_mem_num = ARRAY_SIZE(sprd_dump_mem);

static struct resource sprd_pmu_resource[] = {
	[0] = {
		.start		= IRQ_NPMUIRQ0_INT,
		.end		= IRQ_NPMUIRQ0_INT,
		.flags		= IORESOURCE_IRQ,
	},
	[1] = {
		.start		= IRQ_NPMUIRQ1_INT,
		.end		= IRQ_NPMUIRQ1_INT,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= IRQ_NPMUIRQ2_INT,
		.end		= IRQ_NPMUIRQ2_INT,
		.flags		= IORESOURCE_IRQ,
	},
	[3] = {
		.start		= IRQ_NPMUIRQ3_INT,
		.end		= IRQ_NPMUIRQ3_INT,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device sprd_a7_pmu_device = {
	.name		= "arm-pmu",
	.id		= -1,
	.resource = sprd_pmu_resource,
	.num_resources	= ARRAY_SIZE(sprd_pmu_resource),
};

#ifdef CONFIG_PSTORE_RAM
static struct ramoops_platform_data ramoops_data = {
	.mem_size               = SPRD_RAM_CONSOLE_SIZE,
	.mem_address            = SPRD_RAM_CONSOLE_START,
	.console_size			= SPRD_RAM_CONSOLE_SIZE,
	.dump_oops              = 0,
	.ecc_info               = {
								.ecc_size = 16
	},
};

struct platform_device sprd_ramoops_device = {
	.name = "ramoops",
	.dev = {
		.platform_data = &ramoops_data,
	},
};
#endif

struct platform_device sprd_audio_vbc_r2p0_sprd_codec_v3_device = {
	.name		= "vbc-r2p0-sprd-codec-v3",
	.id		= -1,
};

struct platform_device sprd_audio_i2s_null_codec_device = {
	.name		= "i2s-null-codec",
	.id		= -1,
};

struct platform_device sprd_audio_platform_pcm_device = {
	.name		= "sprd-pcm-audio",
	.id		= -1,
};

struct platform_device sprd_audio_vaudio_device = {
	.name		= "vaudio",
	.id		= -1,
};

struct platform_device sprd_audio_vbc_r2p0_device = {
	.name		= "vbc-r2p0",
	.id		= -1,
};

static struct resource sprd_i2s0_resources[] = {
        [0] = {
                .start = SPRD_IIS0_BASE,
                .end   = SPRD_IIS0_BASE + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = SPRD_IIS0_PHYS,
                .end   = SPRD_IIS0_PHYS + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [2] = {
                .start = DMA_IIS_TX,
                .end   = DMA_IIS_RX,
                .flags = IORESOURCE_DMA,
        }
};

static struct resource sprd_i2s1_resources[] = {
        [0] = {
                .start = SPRD_IIS1_BASE,
                .end   = SPRD_IIS1_BASE + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = SPRD_IIS1_PHYS,
                .end   = SPRD_IIS1_PHYS + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [2] = {
                .start = DMA_IIS1_TX,
                .end   = DMA_IIS1_RX,
                .flags = IORESOURCE_DMA,
        }
};

static struct resource sprd_i2s2_resources[] = {
        [0] = {
                .start = SPRD_IIS2_BASE,
                .end   = SPRD_IIS2_BASE + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = SPRD_IIS2_PHYS,
                .end   = SPRD_IIS2_PHYS + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [2] = {
                .start = DMA_IIS2_TX,
                .end   = DMA_IIS2_RX,
                .flags = IORESOURCE_DMA,
        }
};

static struct resource sprd_i2s3_resources[] = {
        [0] = {
                .start = SPRD_IIS3_BASE,
                .end   = SPRD_IIS3_BASE + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = SPRD_IIS3_PHYS,
                .end   = SPRD_IIS3_PHYS + SZ_4K -1,
                .flags = IORESOURCE_MEM,
        },
        [2] = {
                .start = DMA_IIS3_TX,
                .end   = DMA_IIS3_RX,
                .flags = IORESOURCE_DMA,
        }
};

struct platform_device sprd_audio_i2s0_device = {
	.name		= "i2s",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(sprd_i2s0_resources),
	.resource       = sprd_i2s0_resources,
};

struct platform_device sprd_audio_i2s1_device = {
	.name		= "i2s",
	.id		= 1,
	.num_resources  = ARRAY_SIZE(sprd_i2s1_resources),
	.resource       = sprd_i2s1_resources,
};

struct platform_device sprd_audio_i2s2_device = {
	.name		= "i2s",
	.id		= 2,
	.num_resources  = ARRAY_SIZE(sprd_i2s2_resources),
	.resource       = sprd_i2s2_resources,
};

struct platform_device sprd_audio_i2s3_device = {
	.name		= "i2s",
	.id		= 3,
	.num_resources  = ARRAY_SIZE(sprd_i2s3_resources),
	.resource       = sprd_i2s3_resources,
};

struct platform_device sprd_audio_sprd_codec_v3_device = {
	.name		= "sprd-codec-v3",
	.id		= -1,
};

struct platform_device sprd_audio_null_codec_device = {
	.name		= "null-codec",
	.id		= -1,
};
