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

#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/i2c/pixcir_i2c_ts.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include "devices.h"

extern void __init sc8810_reserve(void);
extern void __init sc8810_map_io(void);
extern void __init sc8810_init_irq(void);
extern void __init sc8810_timer_init(void);
extern void __init regulator_add_devices(void);
extern void __init sc8810_clock_init(void);

static struct platform_device *devices[] __initdata = {
	&sprd_device_example,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_serial_device3,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device1,
	&sprd_backlight_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_keypad_device,
	&sprd_audio_soc_device,
	&sprd_audio_soc_vbc_device,
	&sprd_audio_vbc_device,
	&sprd_battery_device,
	&sprd_pmem_device,
	&sprd_pmem_adsp_device,
	&sprd_sdio1_device,
	&sprd_sdio0_device,
	&sprd_vsp_device,
};

static struct sys_timer sc8810_timer = {
	.init = sc8810_timer_init,
};

static int pdata_notifier_call(struct pdata_notifier *pdn, u32 sub_cmd, void *data);
static struct pdata_notifier sc8810_pdata_notifier [] = {
{
    .pd = { .name = "speaker-pa", },
    .cmd = PLATFORM_PDATA_TYTE_AUDIO_SPEAKER,
    .notifier_call = pdata_notifier_call,
},
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

/*============================TOUCH SCREEN================================*/

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

/*============================TOUCH SCREEN END================================*/
/* sprd8810_i2c2sel_config
 * config I2C2 SDA/SCL to SIM2 pads
 */
static void sprd8810_i2c2sel_config(void)
{
	sprd_greg_set_bits(REG_TYPE_GLOBAL, PINCTRL_I2C2_SEL, GR_PIN_CTL);
}

static int sc8810_add_i2c_devices(void)
{
	sprd8810_i2c2sel_config();
	i2c_register_board_info(2, i2c2_boardinfo, ARRAY_SIZE(i2c2_boardinfo));

}

static int sc8810_add_pdata_notifier_devices(void)
{
    int i, ignored;
    for (i = 0; i < ARRAY_SIZE(sc8810_pdata_notifier); i++) {
        ignored = 1;
        switch (sc8810_pdata_notifier[i].cmd) {
        // board version 0.1 & 0.2 ...
        case PLATFORM_PDATA_TYTE_AUDIO_SPEAKER:
            break;
        default:
            ignored = 0;
            break;
        }
        if (!ignored && sc8810_pdata_notifier[i].pd.name) {
            int ret;
            sc8810_pdata_notifier[i].pd.id = -1;
            platform_set_drvdata(&sc8810_pdata_notifier[i].pd, &sc8810_pdata_notifier[i]);
            ret = platform_device_register(&sc8810_pdata_notifier[i].pd);
            if (ret)
                pr_err("register pdata_notifier failed --> [ %s ].%d < %d >\n",
                        sc8810_pdata_notifier[i].pd.name, sc8810_pdata_notifier[i].cmd, ret);
        } else pr_warn("this board ignore this pdata_notifier --> [ %s ].%d\n",
                sc8810_pdata_notifier[i].pd.name, sc8810_pdata_notifier[i].cmd);
    }
    return 0;
}

static void __init sc8810_init_machine(void)
{
	regulator_add_devices();
	sprd_add_otg_device();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sc8810_add_i2c_devices();
    sc8810_add_pdata_notifier_devices();
}

static int pdata_notifier_call(struct pdata_notifier *pdn, u32 sub_cmd, void *data)
{
    int ret = 0;
    switch (pdn->cmd) {
    case PLATFORM_PDATA_TYTE_AUDIO_SPEAKER:
        if (sub_cmd < 0) {
            // get speaker amplifier status : enabled or disabled
            ret = 0;
        } else {
            // set speaker amplifier
        }
        break;
    default:
        ret = -1;
        pr_warn("[ %s ] cmd %d does not have notifier_call function\n", pdn->pd.name, pdn->cmd);
    }
    return ret;
}

static void __init sc8810_fixup(struct machine_desc *desc, struct tag *tag,
		char **cmdline, struct meminfo *mi)
{
}

static void __init sc8810_init_early(void)
{
	/* earlier init request than irq and timer */
	sc8810_clock_init();

}

MACHINE_START(SC8810OPENPHONE, "SP8810")
	.reserve	= sc8810_reserve,
	.map_io		= sc8810_map_io,
	.init_irq	= sc8810_init_irq,
	.timer		= &sc8810_timer,
	.init_machine	= sc8810_init_machine,
	.fixup		= sc8810_fixup,
	.init_early	= sc8810_init_early,
MACHINE_END
