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
#include <linux/persistent_ram.h>

#include <linux/sprd_cproc.h>
#include <linux/sipc.h>
#include <linux/spipe.h>
#include <linux/spool.h>
#include <linux/seth.h>
#include <sound/saudio.h>
#include <asm/pmu.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include <linux/sprd_thm.h>


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

struct persistent_ram_descriptor sprd_console_desc = {
	.name = "ram_console.0",
	.size = SPRD_RAM_CONSOLE_SIZE,
};

struct persistent_ram sprd_console_ram = {
	.start = SPRD_RAM_CONSOLE_START,
	.size = SPRD_RAM_CONSOLE_SIZE,
	.num_descs = 1,
	.descs = &sprd_console_desc,
};

struct platform_device sprd_ram_console = {
	.name           = "ram_console",
	.id             =  -1,
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

/*if the backlight is driven by pwm, please config the pwm info*/
struct resource sprd_bl_resource[] = {
	[0] = {
		.start	= 0,
		.end	= 0,
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

static struct resource sprd_thm_resources[] = {
    [0] = {
        .start = SPRD_THM_BASE,
        .end = SPRD_THM_BASE + SPRD_THM_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_THM_INT,
        .end = IRQ_THM_INT,
        .flags = IORESOURCE_IRQ,
    },
};

static struct sprd_thm_platform_data sprd_thm_data = {
	.trip_points[0] = {
		.temp = 80,
		.type = THERMAL_TRIP_ACTIVE,
		.cdev_name = {
			[0] = "thermal-cpufreq-0",
		},
	},
	.trip_points[1] = {
		.temp = 160,
		.type = THERMAL_TRIP_CRITICAL,
	},
	.num_trips = 2,
};

struct platform_device sprd_thm_device = {
	.name           = "sprd-thermal",
      .id		= 0,
	.resource       = sprd_thm_resources,
	.num_resources  = ARRAY_SIZE(sprd_thm_resources),
	.dev	= {
		.platform_data	= &sprd_thm_data,
	},
};
static struct resource sprd_thm_a_resources[] = {
    [0] = {
        .start = ANA_THM_BASE,
        .end = ANA_THM_BASE + SPRD_THM_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_ANA_THM_OTP_INT,
        .end = IRQ_ANA_THM_OTP_INT,
        .flags = IORESOURCE_IRQ,
    },
};

static struct sprd_thm_platform_data sprd_thm_a_data = {
	.trip_points[0] = {
		.temp = 160,
		.type = THERMAL_TRIP_CRITICAL,
	},
	.num_trips = 1,
};

struct platform_device sprd_thm_a_device = {
	.name           = "sprd-thermal",
      .id		= 1,
	.resource       = sprd_thm_a_resources,
	.num_resources  = ARRAY_SIZE(sprd_thm_a_resources),
	.dev	= {
		.platform_data	= &sprd_thm_a_data,
	},
};


struct platform_device sprd_audio_platform_pcm_device = {
	.name           = "sprd-pcm-audio",
	.id             =  -1,
};

struct platform_device sprd_audio_cpu_dai_vaudio_device = {
	.name           = "vaudio",
	.id             =  -1,
};

struct platform_device sprd_audio_cpu_dai_vbc_device = {
	.name           = "vbc",
	.id             =  -1,
};

struct platform_device sprd_audio_codec_sprd_codec_device = {
	.name           = "sprd-codec",
	.id             =  -1,
};

static struct resource sprd_i2s_resources0[] = {
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

struct platform_device sprd_audio_cpu_dai_i2s_device = {
	.name           = "i2s",
	.id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_i2s_resources0),
        .resource       = sprd_i2s_resources0,
};

static struct resource sprd_i2s_resources1[] = {
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

struct platform_device sprd_audio_cpu_dai_i2s_device1 = {
	.name           = "i2s",
	.id             =  1,
        .num_resources  = ARRAY_SIZE(sprd_i2s_resources1),
        .resource       = sprd_i2s_resources1,
};

static struct resource sprd_i2s_resources2[] = {
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

struct platform_device sprd_audio_cpu_dai_i2s_device2 = {
	.name           = "i2s",
	.id             =  2,
        .num_resources  = ARRAY_SIZE(sprd_i2s_resources2),
        .resource       = sprd_i2s_resources2,
};

static struct resource sprd_i2s_resources3[] = {
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

struct platform_device sprd_audio_cpu_dai_i2s_device3 = {
	.name           = "i2s",
	.id             =  3,
        .num_resources  = ARRAY_SIZE(sprd_i2s_resources3),
        .resource       = sprd_i2s_resources3,
};

struct platform_device sprd_audio_codec_null_codec_device = {
	.name           = "null-codec",
	.id             =  -1,
};

static struct resource sprd_battery_resources[] = {
        [0] = {
                .start = EIC_CHARGER_DETECT,
                .end = EIC_CHARGER_DETECT,
                .flags = IORESOURCE_IO,
        }
};

struct platform_device sprd_battery_device = {
        .name           = "sprd-battery",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_battery_resources),
        .resource       = sprd_battery_resources,
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
static struct ion_platform_data ion_pdata = {
#if CONFIG_SPRD_ION_OVERLAY_SIZE
        .nr = 2,
#else
        .nr = 1,
#endif
        .heaps = {
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
        }
};
#endif

struct platform_device sprd_ion_dev = {
	.name = "ion-sprd",
	.id = -1,
	.dev = { .platform_data = &ion_pdata },
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
	.detect_gpio = 140,
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

#ifdef CONFIG_SIPC_TD

#define TD_REG_CLK_ADDR				(SPRD_PMU_BASE + 0x50)
#define TD_REG_RESET_ADDR			(SPRD_PMU_BASE + 0xA8)
#define TD_REG_STATUS_ADDR			(SPRD_PMU_BASE + 0xBC)
static int native_tdmodem_start(void *arg)
{
	u32 state;
	u32 value;
	u32 cp1data[3] = {0xe59f0000, 0xe12fff10, CPT_START_ADDR + 0x500000};
	memcpy(SPRD_IRAM1_BASE + 0x1800, cp1data, sizeof(cp1data));

	/* clear cp1 force shutdown */
	value = ((__raw_readl(TD_REG_CLK_ADDR) & ~0x02000000));
	__raw_writel(value, TD_REG_CLK_ADDR);

	while(1)
	{
		state = __raw_readl(TD_REG_STATUS_ADDR);
		if (!(state & (0xf<<16)))
			break;
	}

	/* clear cp1 force deep sleep */
	value = ((__raw_readl(TD_REG_CLK_ADDR) & ~0x10000000));
	__raw_writel(value, TD_REG_CLK_ADDR);

	/* clear reset cp1 */
	value = ((__raw_readl(TD_REG_RESET_ADDR) & ~0x00000002));
	__raw_writel(value, TD_REG_RESET_ADDR);
	return 0;
}
static int native_tdmodem_stop(void *arg)
{
	u32 value;
	/* reset cp1 */
	value = ((__raw_readl(TD_REG_RESET_ADDR) | 0x00000002));
	__raw_writel(value, TD_REG_RESET_ADDR);

	/* cp1 force deep sleep */
	value = ((__raw_readl(TD_REG_CLK_ADDR) | 0x10000000));
	__raw_writel(value, TD_REG_CLK_ADDR);

	/* cp1 force shutdown */
	value = ((__raw_readl(TD_REG_CLK_ADDR) | 0x02000000));
	__raw_writel(value, TD_REG_CLK_ADDR);
	return 0;
}
static struct cproc_init_data sprd_cproc_td_pdata = {
	.devname	= "cpt",
	.base		= CPT_START_ADDR,
	.maxsz		= CPT_TOTAL_SIZE,
	.start		= native_tdmodem_start,
	.stop 		= native_tdmodem_stop,
	.wdtirq		= IRQ_CP1_WDG_INT,
	.segnr		= 2,
	.segs		= {
		{
			.name  = "modem",
			.base  = CPT_START_ADDR + 0x500000,
			.maxsz = 0x00800000,
		},
		{
			.name  = "dsp",
			.base  = CPT_START_ADDR + 0x20000,
			.maxsz = 0x003E0000,
		},
	},
};
struct platform_device sprd_cproc_td_device = {
	.name           = "sprd_cproc",
	.id             = 0,
	.dev		= {.platform_data = &sprd_cproc_td_pdata},
};

static struct spipe_init_data sprd_spipe_td_pdata = {
	.name		= "spipe_td",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_PIPE,
	.ringnr		= 8,
	.txbuf_size	= 4096,
	.rxbuf_size	= 4096,
};
struct platform_device sprd_spipe_td_device = {
	.name           = "spipe",
	.id             = 0,
	.dev		= {.platform_data = &sprd_spipe_td_pdata},
};

static struct spipe_init_data sprd_slog_td_pdata = {
	.name		= "slog_td",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_PLOG,
	.ringnr		= 1,
	.txbuf_size	= 32*1024,
	.rxbuf_size	= 256 * 1024,
};
struct platform_device sprd_slog_td_device = {
	.name           = "spipe",
	.id             = 1,
	.dev		= {.platform_data = &sprd_slog_td_pdata},
};

static struct spipe_init_data sprd_stty_td_pdata = {
	.name		= "stty_td",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_TTY,
	.ringnr		= 32,
	.txbuf_size	= 1024,
	.rxbuf_size	= 1024,
};
struct platform_device sprd_stty_td_device = {
	.name           = "spipe",
	.id             = 2,
	.dev		= {.platform_data = &sprd_stty_td_pdata},
};

static struct seth_init_data sprd_seth0_td_pdata = {
	.name		= "seth_td0",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA0,
};
struct platform_device sprd_seth0_td_device = {
	.name           = "seth",
	.id             =  0,
	.dev		= {.platform_data = &sprd_seth0_td_pdata},
};

static struct seth_init_data sprd_seth1_td_pdata = {
	.name		= "seth_td1",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA1,
};
struct platform_device sprd_seth1_td_device = {
	.name           = "seth",
	.id             =  1,
	.dev		= {.platform_data = &sprd_seth1_td_pdata},
};

static struct seth_init_data sprd_seth2_td_pdata = {
	.name		= "seth_td2",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA2,
};
struct platform_device sprd_seth2_td_device = {
	.name           = "seth",
	.id             =  2,
	.dev		= {.platform_data = &sprd_seth2_td_pdata},
};

static struct spool_init_data sprd_spool_td_pdata = {
	.name 		= "spool_td",
	.dst 		= SIPC_ID_CPT,
	.channel 	= SMSG_CH_CTRL,
	.txblocknum 	= 64,
	.txblocksize 	= 1516,
	.rxblocknum 	= 64,
	.rxblocksize 	= 1516,
};
struct platform_device sprd_spool_td_device = {
	.name 		= "spool",
	.id 		= 0,
	.dev 		= {.platform_data = &sprd_spool_td_pdata},
};

static struct saudio_init_data sprd_saudio_td={
	"VIRTUAL AUDIO",
	SIPC_ID_CPT,
	SMSG_CH_VBC_T,
	SMSG_CH_PLAYBACK_T,
	SMSG_CH_CAPTURE_T,
};

struct platform_device sprd_saudio_td_device = {
	.name       = "saudio",
	.id         = 0,
	.dev        = {.platform_data=&sprd_saudio_td},
};
#endif

#ifdef CONFIG_SIPC_WCDMA

#define WCDMA_REG_CLK_ADDR				(SPRD_PMU_BASE + 0x3C)
#define WCDMA_REG_RESET_ADDR			(SPRD_PMU_BASE + 0xA8)
#define WCDMA_REG_STATUS_ADDR			(SPRD_PMU_BASE + 0xB8)

static int native_wcdmamodem_start(void *arg)
{
	u32 state;
	u32 value;

	u32 cp0data[3] = {0xe59f0000, 0xe12fff10, CPW_START_ADDR + 0x500000};
	memcpy(SPRD_IRAM1_BASE, cp0data, sizeof(cp0data));

	/* clear cp0 force shutdown */
	value = ((__raw_readl(WCDMA_REG_CLK_ADDR) & ~0x02000000));
	__raw_writel(value, WCDMA_REG_CLK_ADDR);

	while(1)
	{
		state = __raw_readl(WCDMA_REG_STATUS_ADDR);
		if (!(state & (0xf<<28)))
			break;
	}

	/* clear cp0 force deep sleep */
	value = ((__raw_readl(WCDMA_REG_CLK_ADDR) & ~0x10000000));
	__raw_writel(value, WCDMA_REG_CLK_ADDR);

	/* clear reset cp0 cp1 */
	value = ((__raw_readl(WCDMA_REG_RESET_ADDR) & ~0x00000001));
	__raw_writel(value, WCDMA_REG_RESET_ADDR);
	return 0;
}
static int native_wcdmamodem_stop(void *arg)
{
	u32 value;
	/* reset cp0 */
	value = ((__raw_readl(WCDMA_REG_RESET_ADDR) | 0x00000001));
	__raw_writel(value, WCDMA_REG_RESET_ADDR);

	/* cp0 force deep sleep */
	value = ((__raw_readl(WCDMA_REG_CLK_ADDR) | ~0x10000000));
	__raw_writel(value, WCDMA_REG_CLK_ADDR);

	/* clear cp0 force shutdown */
	value = ((__raw_readl(WCDMA_REG_CLK_ADDR) | ~0x02000000));
	__raw_writel(value, WCDMA_REG_CLK_ADDR);
	return 0;
}

static struct cproc_init_data sprd_cproc_wcdma_pdata = {
	.devname	= "cpw",
	.base		= CPW_START_ADDR,
	.maxsz		= CPW_TOTAL_SIZE,
	.start		= native_wcdmamodem_start,
	.stop 		= native_wcdmamodem_stop,
	.wdtirq		= IRQ_CP0_WDG_INT,
	.segnr		= 2,
	.segs		= {
		{
			.name  = "modem",
			.base  = CPW_START_ADDR + 0x500000,
			.maxsz = 0x00800000,
		},
		{
			.name  = "dsp",
			.base  = CPW_START_ADDR + 0x20000,
			.maxsz = 0x003E0000,
		},
	},
};
struct platform_device sprd_cproc_wcdma_device = {
	.name           = "sprd_cproc",
	.id             = 1,
	.dev		= {.platform_data = &sprd_cproc_wcdma_pdata},
};


static struct spipe_init_data sprd_spipe_wcdma_pdata = {
	.name		= "spipe_w",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_PIPE,
	.ringnr		= 8,
	.txbuf_size	= 4096,
	.rxbuf_size	= 4096,
};
struct platform_device sprd_spipe_wcdma_device = {
	.name           = "spipe",
	.id             = 3,
	.dev		= {.platform_data = &sprd_spipe_wcdma_pdata},
};

static struct spipe_init_data sprd_slog_wcdma_pdata = {
	.name		= "slog_w",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_PLOG,
	.ringnr		= 1,
	.txbuf_size	= 32 * 1024,
	.rxbuf_size	= 256 * 1024,
};
struct platform_device sprd_slog_wcdma_device = {
	.name           = "spipe",
	.id             = 4,
	.dev		= {.platform_data = &sprd_slog_wcdma_pdata},
};

static struct spipe_init_data sprd_stty_wcdma_pdata = {
	.name		= "stty_w",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_TTY,
	.ringnr		= 32,
	.txbuf_size	= 1024,
	.rxbuf_size	= 1024,
};
struct platform_device sprd_stty_wcdma_device = {
	.name           = "spipe",
	.id             = 5,
	.dev		= {.platform_data = &sprd_stty_wcdma_pdata},
};

static struct seth_init_data sprd_seth0_wcdma_pdata = {
	.name		= "seth_w0",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_DATA0,
};
struct platform_device sprd_seth0_wcdma_device = {
	.name           = "seth",
	.id             =  3,
	.dev		= {.platform_data = &sprd_seth0_wcdma_pdata},
};

static struct seth_init_data sprd_seth1_wcdma_pdata = {
	.name		= "seth_w1",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_DATA1,
};
struct platform_device sprd_seth1_wcdma_device = {
	.name           = "seth",
	.id             =  4,
	.dev		= {.platform_data = &sprd_seth1_wcdma_pdata},
};

static struct seth_init_data sprd_seth2_wcdma_pdata = {
	.name		= "seth_w2",
	.dst		= SIPC_ID_CPW,
	.channel	= SMSG_CH_DATA2,
};
struct platform_device sprd_seth2_wcdma_device = {
	.name           = "seth",
	.id             =  5,
	.dev		= {.platform_data = &sprd_seth2_wcdma_pdata},
};

static struct spool_init_data sprd_spool_wcdma_pdata = {
	.name 		= "spool_w",
	.dst 		= SIPC_ID_CPW,
	.channel 	= SMSG_CH_CTRL,
	.txblocknum 	= 64,
	.txblocksize	= 1516,
	.rxblocknum 	= 64,
	.rxblocksize 	= 1516,
};
struct platform_device sprd_spool_wcdma_device = {
	.name 		= "spool",
	.id 		= 1,
	.dev 		= {.platform_data = &sprd_spool_wcdma_pdata},
};


static struct saudio_init_data sprd_saudio_wcdma={
	"VIRTUAL AUDIO W",
	SIPC_ID_CPW,
	SMSG_CH_VBC_W,
	SMSG_CH_PLAYBACK_W,
	SMSG_CH_CAPTURE_W,
};

struct platform_device sprd_saudio_wcdma_device = {
	.name       = "saudio",
	.id         = 1,
	.dev        = {.platform_data=&sprd_saudio_wcdma},
};
static struct saudio_init_data sprd_saudio_voip={
	"saudiovoip",
	SIPC_ID_CPW,
	SMSG_CH_CTRL_VOIP,
	SMSG_CH_PLAYBACK_VOIP,
	SMSG_CH_CAPTURE_VOIP,
};

struct platform_device sprd_saudio_voip_device = {
	.name       = "saudio",
	.id         = 2,
	.dev        = {.platform_data=&sprd_saudio_voip},
};
#endif

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
		.size		= SPRD_IO_MEM_BASE - (CPT_START_ADDR + CPT_TOTAL_SIZE),
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
	.id		= ARM_PMU_DEVICE_CPU,
	.resource = sprd_pmu_resource,
	.num_resources	= ARRAY_SIZE(sprd_pmu_resource),
};



#define AP2CP_INT_CTRL		(SPRD_IPI_BASE + 0x00)
#define CP2AP_INT_CTRL		(SPRD_IPI_BASE + 0x04)


#define AP2CPW_IRQ0_TRIG	0x01
#define CPW2AP_IRQ0_CLR		0x01
uint32_t cpw_rxirq_status(void)
{
	return 1;
}

void cpw_rxirq_clear(void)
{
	__raw_writel(CPW2AP_IRQ0_CLR, CP2AP_INT_CTRL);
}

void cpw_txirq_trigger(void)
{
	__raw_writel(AP2CPW_IRQ0_TRIG, AP2CP_INT_CTRL);
}


#define AP2CPT_IRQ0_TRIG	0x10
#define CPT2AP_IRQ0_CLR		0x10
uint32_t cpt_rxirq_status(void)
{
	return 1;
}

void cpt_rxirq_clear(void)
{
	__raw_writel(CPT2AP_IRQ0_CLR, CP2AP_INT_CTRL);
}

void cpt_txirq_trigger(void)
{
	__raw_writel(AP2CPT_IRQ0_TRIG, AP2CP_INT_CTRL);
}

#define AP2WCN_IRQ0_TRIG	0x100
#define WCN2AP_IRQ0_CLR		0x100
uint32_t wcn_rxirq_status(void)
{
	return 1;
}

void wcn_rxirq_clear(void)
{
	__raw_writel(WCN2AP_IRQ0_CLR, CP2AP_INT_CTRL);
}

void wcn_txirq_trigger(void)
{
	__raw_writel(AP2WCN_IRQ0_TRIG, AP2CP_INT_CTRL);
}
