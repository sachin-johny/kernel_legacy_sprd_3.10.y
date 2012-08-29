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
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/localtimer.h>

#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/i2c/pixcir_i2c_ts.h>
#include <linux/spi/spi.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include <mach/serial_sprd.h>
#include "devices.h"


extern void __init tiger_reserve(void);
extern void __init sci_map_io(void);
extern void __init tiger_init_irq(void);
extern void __init tiger_timer_init(void);
extern void __init regulator_add_devices(void);
extern void __init tiger_clock_init(void);

static struct platform_device *devices[] __initdata = {
	&sprd_hwspinlock_device0,
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device0,
	&sprd_backlight_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_spi0_device,
	&sprd_spi1_device,
	&sprd_keypad_device,
	&sprd_audio_soc_device,
	&sprd_audio_soc_vbc_device,
	&sprd_audio_vbc_device,
	&sprd_battery_device,
	&sprd_pmem_device,
	&sprd_pmem_adsp_device,
	&sprd_sdio0_device,
	&sprd_sdio1_device,
	&sprd_vsp_device,
	&sprd_dcam_device,
	&sprd_scale_device,
	&sprd_rotation_device,
};


static struct sys_timer tiger_timer = {
	.init = tiger_timer_init,
};

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

static void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	if (calibration_mode)
		return;
	platform_device_register(&sprd_otg_device);
}

static struct serial_data plat_data0 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 48000000,
};
static struct serial_data plat_data1 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};
static struct serial_data plat_data2 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};

static struct pixcir_ts_platform_data pixcir_ts_info = {
	.irq_gpio_number	= GPIO_TOUCH_IRQ,
	.reset_gpio_number	= GPIO_TOUCH_RESET,
};

static struct i2c_board_info i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO(PIXICR_DEVICE_NAME, 0x5C),
		.platform_data = &pixcir_ts_info,
	},
};

static struct i2c_board_info i2c1_boardinfo[] = {
	{I2C_BOARD_INFO("sensor_main",0x3C),},
	{I2C_BOARD_INFO("sensor_sub",0x21),},
};

/* config I2C2 SDA/SCL to SIM2 pads */
static void sprd8810_i2c2sel_config(void)
{
	sprd_greg_set_bits(REG_TYPE_GLOBAL, PINCTRL_I2C2_SEL, GR_PIN_CTL);
}

static int sc8810_add_i2c_devices(void)
{
	sprd8810_i2c2sel_config();
	i2c_register_board_info(2, i2c2_boardinfo, ARRAY_SIZE(i2c2_boardinfo));
	i2c_register_board_info(1, i2c1_boardinfo, ARRAY_SIZE(i2c1_boardinfo));
	return 0;
}

struct platform_device audio_pa_amplifier_device = {
	.name = "speaker-pa",
	.id = -1,
};

static int audio_pa_amplifier_l(u32 cmd, void *data)
{
	int ret = 0;
	if (cmd < 0) {
		/* get speaker amplifier status : enabled or disabled */
		ret = 0;
	} else {
		/* set speaker amplifier */
	}
	return ret;
}

static int spi_cs_gpio_map[][2] = {
    {SPI0_CMMB_CS_GPIO,  0},
} ;

static struct spi_board_info spi_boardinfo[] = {
	{
	.modalias = "cmmb-dev",
	.bus_num = 0,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	}
};

static void sprd_spi_init(void)
{
	int busnum, cs, gpio;
	int i;

	struct spi_board_info *info = spi_boardinfo;

	for (i = 0; i < ARRAY_SIZE(spi_boardinfo); i++) {
		busnum = info[i].bus_num;
		cs = info[i].chip_select;
		gpio   = spi_cs_gpio_map[busnum][cs];

		info[i].controller_data = (void *)gpio;
	}

        spi_register_board_info(info, ARRAY_SIZE(spi_boardinfo));
}

static int sc8810_add_misc_devices(void)
{
	if (0) {
		platform_set_drvdata(&audio_pa_amplifier_device, audio_pa_amplifier_l);
		if (platform_device_register(&audio_pa_amplifier_device))
			pr_err("faile to install audio_pa_amplifier_device\n");
	}
	return 0;
}

static void __init tiger_init_machine(void)
{
	regulator_add_devices();
	sprd_add_otg_device();
	platform_device_add_data(&sprd_serial_device0,(const void*)&plat_data0,sizeof(plat_data0));
	platform_device_add_data(&sprd_serial_device1,(const void*)&plat_data1,sizeof(plat_data1));
	platform_device_add_data(&sprd_serial_device2,(const void*)&plat_data2,sizeof(plat_data2));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sc8810_add_i2c_devices();
	sc8810_add_misc_devices();
	sprd_spi_init();
}

extern void tiger_enable_timer_early(void);
static void __init tiger_init_early(void)
{
	/* earlier init request than irq and timer */
	tiger_clock_init();
	tiger_enable_timer_early();
}

/*
 * Setup the memory banks.
 */
static void tiger_fixup(struct tag *tags, char **from, struct meminfo *meminfo)
{
#if 0//WRONG, NEED TODO
/*
 * Most Tiger platforms have 256MB contiguous RAM at 0x80000000.
 * Half of this is at 0.
 */
#ifdef CONFIG_SMP
	meminfo->bank[0].start = 0x80000000;
	meminfo->bank[0].size = SZ_256M;
	meminfo->nr_banks = 1;
#else
	meminfo->bank[0].start = 0;
	meminfo->bank[0].size = SZ_256M;
	meminfo->nr_banks = 1;
#endif
#endif
}

#ifdef CONFIG_SMP
static void __init gic_init_irq(void)
{
	gic_init(0, 29, TIGER_VA_GIC_DIS, TIGER_VA_GIC_CPU);	
}

MACHINE_START(SC8825OPENPHONE, "tiger")	
	.reserve	= tiger_reserve,
	.map_io		= sci_map_io,
	.fixup		= tiger_fixup,
	.init_early	= tiger_init_early,
	.init_irq	= gic_init_irq,
	.timer		= &tiger_timer,
	.init_machine	= tiger_init_machine,
MACHINE_END

#else

MACHINE_START(SC8825OPENPHONE, "tiger")
	.reserve	= tiger_reserve,
	.init_irq	= tiger_init_irq,
	.fixup		= tiger_fixup,
	.map_io		= sci_map_io,
	.init_early	= tiger_init_early,
	.init_machine	= tiger_init_machine,
	.timer		=  &tiger_timer,
MACHINE_END

#endif


