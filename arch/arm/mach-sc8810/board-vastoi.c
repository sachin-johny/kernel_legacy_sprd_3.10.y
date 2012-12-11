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
#include <linux/delay.h>
#include <linux/input.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include <mach/gpio-vastoi.h>//bst_zhaoxueyan for vastoi 20120914
#include <sound/audio_pa.h>
#include "devices.h"
#include <linux/ktd253b_bl.h>
#include <mach/gpio.h>
#include <mach/sys_debug.h>
#include <linux/spi/mxd_cmmb_026x.h>

#include <mach/regulator.h>//bst_zhaoxueyan for vastoi 20120914
#include <linux/regulator/consumer.h> //bst_zhaoxueyan for vastoI 20120919
#if defined(CONFIG_SS6000_CHARGER)
#include <linux/ss6000_charger.h>
#endif
#if defined(CONFIG_BQ24157_CHARGER)
#include <mach/bq24157_charger.h>
#endif
#if defined(CONFIG_STC3115_FUELGAUGE)
#include <linux/stc3115_battery.h>
#endif

#include <gps/gpsctl.h>
#include <mach/adc.h>
#include <linux/headset.h>

#if defined(CONFIG_SPA)
#include <linux/power/spa.h>
#endif 

#ifdef CONFIG_INPUT_YAS_SENSORS
#include <linux/yas.h>
#endif

#if defined(CONFIG_SENSORS_GP2A)
#include <linux/gp2a_dev.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_MMS134S_TS)
#include <linux/input/mms134s_ts.h>
#endif

#include <mach/serial_sprd.h>
#include <mach/pinmap.h>
extern void __init sc8810_reserve(void);
extern void __init sc8810_map_io(void);
extern void __init sc8810_init_irq(void);
extern void __init sc8810_timer_init(void);
extern void __init regulator_add_devices(void);
extern void __init sc8810_clock_init(void);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern int __init sprd_ramconsole_init(void);
#endif
extern int sci_adc_get_value(unsigned chan, int scale);
static struct platform_device rfkill_device;
static struct platform_device brcm_bluesleep_device;

static unsigned int sd_detect_gpio = GPIO_SDIO_DETECT;

/* Control ldo for maxscend cmmb chip according to HW design */
static struct regulator *cmmb_regulator_1v8 = NULL;

static struct platform_gpsctl_data pdata_gpsctl = {
	.reset_pin = GPIO_GPS_RESET,
	.onoff_pin = GPIO_GPS_ONOFF,
	.clk_type  = "clk_aux0",
};

static struct platform_device  gpsctl_dev = {
	.name               = "gpsctl",
	.dev.platform_data  = &pdata_gpsctl,
};

#if defined  (CONFIG_SS_VIBRATOR)
struct platform_device vibrator_device = {
   .name = "vibrator",
   .id = 0,
   .dev = {
      .platform_data="VDDWIF0",
   },
};
#endif
#if defined(CONFIG_SS6000_CHARGER)
static struct ss6000_platform_data ss6000_info = {
	.en_set = CHARGERIC_CHG_EN_GPIO,
	.pgb = CHARGERIC_TA_INT_GPIO,
	.chgsb = CHARGERIC_CHG_INT_GPIO,
}; 
static struct platform_device ss6000_charger = {
	.name		= "ss6000_charger",
	.id		= -1, 
	.dev		= {
		.platform_data = &ss6000_info,
	},		
};
#endif
#if defined(CONFIG_BQ24157_CHARGER)
static struct bq24157_platform_data bq24157_charger_info = {
	.cd = CHARGERIC_CD_GPIO,
};
#endif

#if defined(CONFIG_STC3115_FUELGAUGE)
int null_fn(void)
{
        return 0;                // for discharging status
}

int Temperature_fn(void)
{
	return (25);
}
static struct stc311x_platform_data stc3115_data = {
                   .battery_online = NULL,
                .charger_online = null_fn, 		// used in stc311x_get_status()
                .charger_enable = null_fn,		// used in stc311x_get_status()
                .power_supply_register = NULL,
                .power_supply_unregister = NULL,

		.Vmode= 0,       /*REG_MODE, BIT_VMODE 1=Voltage mode, 0=mixed mode */
  		.Alm_SOC = 10,      /* SOC alm level %*/
  		.Alm_Vbat = 3600,   /* Vbat alm level mV*/
  		.CC_cnf = 303,      /* nominal CC_cnf, coming from battery characterisation*/
  		.VM_cnf = 307,      /* nominal VM cnf , coming from battery characterisation*/
  		.Cnom = 1500,       /* nominal capacity in mAh, coming from battery characterisation*/
  		.Rsense = 10,       /* sense resistor mOhms*/
  		.RelaxCurrent = 75, /* current for relaxation in mA (< C/20) */
  		.Adaptive = 1,     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

		.CapDerating[6] = 0,   /* capacity derating in 0.1%, for temp = -20C */
  		.CapDerating[5] = 0,   /* capacity derating in 0.1%, for temp = -10C */
		.CapDerating[4] = 0,   /* capacity derating in 0.1%, for temp = 0C */
		.CapDerating[3] = 0,   /* capacity derating in 0.1%, for temp = 10C */
		.CapDerating[2] = 0,   /* capacity derating in 0.1%, for temp = 25C */
		.CapDerating[1] = 0,   /* capacity derating in 0.1%, for temp = 40C */
		.CapDerating[0] = 0,   /* capacity derating in 0.1%, for temp = 60C */

  		.OCVOffset[15] = 25,    /* OCV curve adjustment */
		.OCVOffset[14] = 17,   /* OCV curve adjustment */
		.OCVOffset[13] = 19,    /* OCV curve adjustment */
		.OCVOffset[12] = 11,    /* OCV curve adjustment */
		.OCVOffset[11] = 0,    /* OCV curve adjustment */
		.OCVOffset[10] = 11,    /* OCV curve adjustment */
		.OCVOffset[9] = 16,     /* OCV curve adjustment */
		.OCVOffset[8] = 4,      /* OCV curve adjustment */
		.OCVOffset[7] = 1,      /* OCV curve adjustment */
		.OCVOffset[6] = 3,    /* OCV curve adjustment */
		.OCVOffset[5] = 14,    /* OCV curve adjustment */
		.OCVOffset[4] = 38,     /* OCV curve adjustment */
		.OCVOffset[3] = 11,    /* OCV curve adjustment */
		.OCVOffset[2] = 35,     /* OCV curve adjustment */
		.OCVOffset[1] = 127,    /* OCV curve adjustment */
		.OCVOffset[0] = 30,     /* OCV curve adjustment */

			/*if the application temperature data is preferred than the STC3115 temperature*/
  		.ExternalTemperature = Temperature_fn, /*External temperature fonction, return C*/
  		.ForceExternalTemperature = 0, /* 1=External temperature, 0=STC3115 temperature */             
};
#endif

#if defined(CONFIG_SPA)
static struct spa_platform_data spa_info = {
	.use_fuelgauge = 0,
	.battery_capacity = 1200,
	.VF_low	= 100,
	.VF_high = 600,
}; 
static struct platform_device Sec_BattMonitor = {
	.name		= "Sec_BattMonitor",
	.id		= -1,
	.dev		= {
		.platform_data = &spa_info,
	},
};
#endif



#if defined(CONFIG_INPUT_YAS_SENSORS)
static struct platform_device yas532_orient_device = {
	.name			= "orientation",
};
#endif

static struct platform_device *devices[] __initdata = {
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device0,
	&sprd_backlight_device,
#if defined(CONFIG_SPA)
	&Sec_BattMonitor,
#else
	&sprd_battery_device,
#endif	
#ifndef CONFIG_MUSB_FSA880
	&sprd_battery_device,
#endif
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_i2c_device3,
	&sprd_spi0_device,
	&sprd_spi1_device,
	&sprd_keypad_device,
	&sprd_audio_soc_device,
	&sprd_audio_soc_vbc_device,
	&sprd_audio_vbc_device,
	&sprd_pmem_device,
	&sprd_pmem_adsp_device,
	&sprd_sdio1_device,
	&sprd_sdio0_device,
	&sprd_vsp_device,
	&sprd_dcam_device,
	&sprd_scale_device,
	&sprd_rotation_device,
#if defined (CONFIG_BT_BCM4330)	
	&rfkill_device,
	&brcm_bluesleep_device,
#endif	
	&gpsctl_dev,
#if defined  (CONFIG_SS_VIBRATOR)	
	&vibrator_device,
#endif
#if defined(CONFIG_SS6000_CHARGER)
	&ss6000_charger,
#endif
#if defined(CONFIG_INPUT_YAS_SENSORS)
	&yas532_orient_device,
#endif
#if defined(CONFIG_SPRD_PEER_STATE)
	&sprd_peer_state_device,
#endif
};


/* Start Bluetooth Releated Device Drivers */
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= GPIO_BT2AP_WAKE,
		.end	= GPIO_BT2AP_WAKE,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= GPIO_AP2BT_WAKE,
		.end	= GPIO_AP2BT_WAKE,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device brcm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct resource rfkill_resources[] = {
	{
		.name   = "bt_power",
		.start  = GPIO_BT_POWER,
		.end    = GPIO_BT_POWER,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bt_reset",
		.start  = GPIO_BT_RESET,
		.end    = GPIO_BT_RESET,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device rfkill_device = {
	.name = "rfkill",
	.id = -1,
	.num_resources	= ARRAY_SIZE(rfkill_resources),
	.resource	= rfkill_resources,
};

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
/* End Bluetooth Releated Device Drivers */

static struct platform_ktd253b_backlight_data ktd253b_data = {
	.max_brightness = 255,
	.dft_brightness = 50,
	.ctrl_pin       = GPIO_BK,
};

static struct sys_timer sc8810_timer = {
	.init = sc8810_timer_init,
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

static struct i2c_gpio_platform_data pdata_gpio_i2c_p4 = {
	.sda_pin                = GPIO_I2C_SDA,             /*  SIMCLK3 */
	.sda_is_open_drain      = 0,
	.scl_pin                = GPIO_I2C_SCL,             /*  SIMCLK2*/
	.scl_is_open_drain      = 0,
	.udelay                 = 2,            /* ~100 kHz */
};

static struct platform_device  device_data_gpio_i2c_p4 = {
	.name               = "i2c-gpio",
	.id                 = 4,
	.dev.platform_data  = &pdata_gpio_i2c_p4,
};

#if defined  (CONFIG_SENSORS_GP2A)
#define PROXI_INT_GPIO_PIN      (94)
#define PROXI_POWER_GPIO_PIN      (92)
static struct gp2a_prox_platform_data gp2a_prox_platform_data = {
	.irq_gpio = PROXI_INT_GPIO_PIN,
	//.irq = gpio_to_irq(PROXI_INT_GPIO_PIN),
	.power = PROXI_POWER_GPIO_PIN,
};
#endif

static struct i2c_board_info i2c0_boardinfo[] = {
	
#if defined  (CONFIG_SENSORS_K3DH)
	{
		I2C_BOARD_INFO("k3dh", 0x19),
	},
#endif

#if defined  (CONFIG_SENSORS_HSCD)
	{
		I2C_BOARD_INFO("hscd_i2c", 0x0c),
	},
 #endif
	
#ifdef CONFIG_INPUT_YAS_SENSORS
	#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
	{
		I2C_BOARD_INFO("accelerometer", 0x19),
	},
	#elif CONFIG_YAS_ACC_DRIVER_BMA222E
	{
		I2C_BOARD_INFO("accelerometer", 0x18),
	},
	#endif
	{
		I2C_BOARD_INFO("geomagnetic", 0x2e),
	},
#endif
	
#if defined  (CONFIG_SENSORS_GP2A)
	{
		I2C_BOARD_INFO("gp2a_prox", 0x44),
		.platform_data = &gp2a_prox_platform_data,    
	},
#endif

};

static struct i2c_board_info __initdata i2c3_boardinfo[] = {
#if defined(CONFIG_MUSB_FSA880)
	 {
		 I2C_BOARD_INFO("fsa880", 0x25),
	 },
#endif
#if defined(CONFIG_BQ27425_FUELGAUGE)
	{
		I2C_BOARD_INFO("bq27425_firmware", 0xB),
	},
	{
		I2C_BOARD_INFO("bq27425_fuelgauge", 0x55),
		.irq = GPIO_BQ27425_LOW_BAT,
	},
#endif
#if defined(CONFIG_BQ24157_CHARGER)
	{
		I2C_BOARD_INFO("bq24157_6A", 0x6A),
		.platform_data	= &bq24157_charger_info,
	},
	{
		I2C_BOARD_INFO("bq24157_6B", 0x6B),
		.platform_data	= &bq24157_charger_info,
	},
#endif
#if defined(CONFIG_STC3115_FUELGAUGE)
	{
		I2C_BOARD_INFO("stc3115", 0x70),
		.platform_data	= &stc3115_data,
	},
#endif
};

static struct i2c_board_info i2c1_boardinfo[] = {
	{I2C_BOARD_INFO("sensor_main",0x3C),},
	{I2C_BOARD_INFO("sensor_sub",0x21),},
};

#if defined  (CONFIG_TOUCHSCREEN_TMA140)
static struct i2c_board_info __initdata i2c_boardinfo_p4[] = {
	{I2C_BOARD_INFO("synaptics-rmi-ts", 0x20),},
};
#elif defined(CONFIG_TOUCHSCREEN_MMS134S_TS)
#define GPIO_I2C_INT 60
static struct i2c_board_info __initdata i2c_boardinfo_p4[] = {
	{
		I2C_BOARD_INFO("mms_ts", 0x48),
		.irq = GPIO_I2C_INT,
	},
};
#else
static struct i2c_board_info __initdata i2c_boardinfo_p4[] = {
	{I2C_BOARD_INFO("zinitix_isp", 0x50),},
	{I2C_BOARD_INFO("Zinitix_tsp", 0x20),},
};
#endif

static int sc8810_add_i2c_devices(void)
{
	platform_device_register(&device_data_gpio_i2c_p4);
	i2c_register_board_info(4, i2c_boardinfo_p4,ARRAY_SIZE(i2c_boardinfo_p4));
	i2c_register_board_info(1, i2c1_boardinfo, ARRAY_SIZE(i2c1_boardinfo));
	i2c_register_board_info(0, i2c0_boardinfo, ARRAY_SIZE(i2c0_boardinfo));
	i2c_register_board_info(3, i2c3_boardinfo,ARRAY_SIZE(i2c3_boardinfo));

	return 0;
}

struct platform_device audio_pa_amplifier_device = {
	.name = "speaker-pa",
	.id = -1,
};

static int audio_pa_amplifier_speaker(u32 cmd, void *data)
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

static int audio_pa_amplifier_headset_init(void)
{
	if (gpio_request(HEADSET_PA_CTL_GPIO, "headset outside pa")) {
		pr_err("failed to alloc gpio %d\n", HEADSET_PA_CTL_GPIO);
		return -1;
	}
	gpio_direction_output(HEADSET_PA_CTL_GPIO, 0);
	return 0;
}

static int audio_pa_amplifier_headset(u32 cmd, void *data)
{
	gpio_direction_output(HEADSET_PA_CTL_GPIO, cmd);
	return 0;
}

static _audio_pa_control audio_pa_control = {
	.speaker = {
		.init = NULL,
		.control = NULL,
	},
	.earpiece = {
		.init = NULL,
		.control = NULL,
	},
	.headset = {
		.init = audio_pa_amplifier_headset_init,
		.control = audio_pa_amplifier_headset,
	},
};

static unsigned int headset_get_button_code_board_method(int v)
{
	static struct headset_adc_range {
		int min;
		int max;
		int code;
	} adc_range[] = {
		{ 0x0000, 0x0030, KEY_MEDIA},
		{ 0x0031, 0x0065, KEY_VOLUMEUP},
		{ 0x0066, 0x00cf, KEY_VOLUMEDOWN },
	};

	int adc_value;

	adc_value = sci_adc_get_value(ADC_CHANNEL_7, ADC_SCALE_3V);
	pr_info("[headset] ********* adc value7 SCALE 0~3V : 0x%x ******* \n", adc_value);			

	int i;
	for (i = 0; i < ARRY_SIZE(adc_range); i++)
		if (adc_value >= adc_range[i].min && adc_value < adc_range[i].max)
			return adc_range[i].code;
	return KEY_RESERVED;
}

static unsigned int headset_map_code2push_code_board_method(unsigned int code, int push_type)
{
	return code;
}

struct platform_device headset_get_button_code_board_method_device = {
	.name = "headset-button",
	.id = -1,
};

struct _headset_button headset_button = {
	.cap = {
		{ EV_KEY, KEY_MEDIA },
		{ EV_KEY, KEY_VOLUMEUP},
		{ EV_KEY, KEY_VOLUMEDOWN },
	},
	.headset_get_button_code_board_method = headset_get_button_code_board_method,
	.headset_map_code2push_code_board_method = headset_map_code2push_code_board_method,
};
#if 0//bst_zhaoxueyan for vastoi 20120914
static void mxd_cmmb_poweron()
{
       gpio_direction_output(GPIO_CMMB_EN, 0);
       msleep(3);
       gpio_direction_output(GPIO_CMMB_EN, 1);
}
static void mxd_cmmb_poweroff()
{
       gpio_direction_output(GPIO_CMMB_EN, 0);
}
static int mxd_cmmb_init()
{
	int ret=0;
	ret = gpio_request(GPIO_CMMB_EN,   "MXD_CMMB_EN");
	if (ret)
	{
		pr_debug("mxd spi req gpio en err!\n");
		goto err_gpio_init;
	}
        gpio_direction_output(GPIO_CMMB_EN, 0);
	gpio_set_value(GPIO_CMMB_EN, 0);
	return 0;
err_gpio_init:
        gpio_free(GPIO_CMMB_EN);
	return ret;
}
#else

#define SPI_PIN_FUNC_MASK  (0x3<<4) 
#define SPI_PIN_FUNC_DEF   (0x2<<4) 
#define SPI_PIN_FUNC_GPIO  (0x3<<4) 

struct spi_pin_desc { 
        const char   *name; 
        unsigned int pin_func; 
        unsigned int reg; 
        unsigned int gpio; 
}; 

static struct spi_pin_desc spi_pin_group[] = { 
        {"SPI_DI",  SPI_PIN_FUNC_DEF,  REG_PIN_TRACECLK  + CTL_PIN_BASE, 41}, 
        {"SPI_CLK", SPI_PIN_FUNC_DEF,  REG_PIN_TRACECTRL + CTL_PIN_BASE, 42}, 
        {"SPI_DO",  SPI_PIN_FUNC_DEF,  REG_PIN_TRACEDAT0 + CTL_PIN_BASE, 43}, 
        {"SPI_CS",  SPI_PIN_FUNC_GPIO, REG_PIN_TRACEDAT1 + CTL_PIN_BASE, 44} 
}; 


static void sprd_restore_spi_pin_cfg(void)
{
	unsigned int reg;
	unsigned int  gpio;
	unsigned int  pin_func;
	unsigned int value;
	unsigned long flags;
	int i = 0;
	int regs_count = sizeof(spi_pin_group)/sizeof(struct spi_pin_desc);

	for (; i < regs_count; i++) {
	    pin_func = spi_pin_group[i].pin_func;
	    gpio = spi_pin_group[i].gpio;
	    if (pin_func == SPI_PIN_FUNC_DEF) {
		 reg = spi_pin_group[i].reg;
		 /* free the gpios that have request */
		 gpio_free(gpio);
		 local_irq_save(flags);
		 /* config pin default spi function */
		 value = ((__raw_readl(reg) & ~SPI_PIN_FUNC_MASK) | SPI_PIN_FUNC_DEF);
		 __raw_writel(value, reg);
		 local_irq_restore(flags);
	    }
	    else {
		 /* CS should config output */
		 gpio_direction_output(gpio, 1);
	    }
	}

}


static void sprd_set_spi_pin_input(void)
{
	unsigned int reg;
	unsigned int value;
	unsigned int  gpio;
	unsigned int  pin_func;
	const char    *name;
	unsigned long flags;
	int i = 0;

	int regs_count = sizeof(spi_pin_group)/sizeof(struct spi_pin_desc);

	for (; i < regs_count; i++) {
	    pin_func = spi_pin_group[i].pin_func;
	    gpio = spi_pin_group[i].gpio;
	    name = spi_pin_group[i].name;

	    /* config pin GPIO function */
	    if (pin_func == SPI_PIN_FUNC_DEF) {
		 reg = spi_pin_group[i].reg;

		 local_irq_save(flags);
		 value = ((__raw_readl(reg) & ~SPI_PIN_FUNC_MASK) | SPI_PIN_FUNC_GPIO);
		 __raw_writel(value, reg);
		 local_irq_restore(flags);
		 if (gpio_request(gpio, name)) {
		     printk("smsspi: request gpio %d failed, pin %s\n", gpio, name);
		 }

	    }

	    gpio_direction_input(gpio);
	}

}

static void mxd_cmmb_poweron(void)
{
        regulator_set_voltage(cmmb_regulator_1v8, 1700000, 1800000);
        regulator_disable(cmmb_regulator_1v8);
        msleep(3);
        regulator_enable(cmmb_regulator_1v8);
        msleep(5);

        /* enable 26M external clock */
        gpio_direction_output(GPIO_CMMB_EN, 1);
}

static void mxd_cmmb_poweroff(void)
{
        regulator_disable(cmmb_regulator_1v8);
        gpio_direction_output(GPIO_CMMB_EN, 0);
}

static int mxd_cmmb_init(void)
{
         int ret=0;

	 cmmb_regulator_1v8 = regulator_get(NULL, REGU_NAME_CMMBIO);
	 regulator_enable(cmmb_regulator_1v8);
	 msleep(5);
	 regulator_disable(cmmb_regulator_1v8);

         ret = gpio_request(GPIO_CMMB_EN,   "MXD_CMMB_CLKEN");
         if (ret)
         {
         	pr_debug("mxd spi req gpio clk en err!\n");
                goto err_gpio_init;
         }
         gpio_direction_output(GPIO_CMMB_EN, 0);

         return 0;

err_gpio_init:
	 gpio_free(GPIO_CMMB_EN);
         return ret;
}

#endif


static struct mxd_cmmb_026x_platform_data mxd_plat_data = {
	.poweron  = mxd_cmmb_poweron,
	.poweroff = mxd_cmmb_poweroff,
	.init     = mxd_cmmb_init,
	.set_spi_pin_input   = sprd_set_spi_pin_input,
	.restore_spi_pin_cfg = sprd_restore_spi_pin_cfg,
};

static int spi_cs_gpio_map[][2] = {
    {SPI0_WIFI_CS_GPIO,  0},
    {SPI1_CMMB_CS_GPIO,  0},
} ;


static struct spi_board_info spi_boardinfo[] = {
	{
		.modalias = "wlan_spi",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 48 * 1000 * 1000,
		.mode = SPI_CPOL | SPI_CPHA,
	},
	{
		.modalias = "cmmb-dev",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 10 * 1000 * 1000,
		.mode = SPI_CPOL | SPI_CPHA,
		.platform_data = &mxd_plat_data,
	},
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

		pr_err("sc8810_add_misc_devices \n");
		
	if (audio_pa_control.speaker.control || audio_pa_control.earpiece.control || \
		audio_pa_control.headset.control) {
		platform_set_drvdata(&audio_pa_amplifier_device, &audio_pa_control);
		if (platform_device_register(&audio_pa_amplifier_device))
			pr_err("faile to install audio_pa_amplifier_device\n");
	}

	platform_set_drvdata(&headset_get_button_code_board_method_device, &headset_button);
	if (platform_device_register(&headset_get_button_code_board_method_device))
		pr_err("faile to install headset_get_button_code_board_method_device\n");

	return 0;
}

static void __init sc8810_init_machine(void)
{
	int clk;

	pr_err("sc8810_init_machine \n");
			
	regulator_add_devices();
	sprd_add_otg_device();
	platform_device_add_data(&sprd_sdio0_device, &sd_detect_gpio, sizeof(sd_detect_gpio));
	platform_device_add_data(&sprd_backlight_device,&ktd253b_data,sizeof(ktd253b_data));
	clk=48000000;
	platform_device_add_data(&sprd_serial_device0,(const void*)&clk,sizeof(int));
	clk=26000000;
	platform_device_add_data(&sprd_serial_device1,(const void*)&clk,sizeof(int));
	platform_device_add_data(&sprd_serial_device2,(const void*)&clk,sizeof(int));     
    platform_device_add_data(&sprd_sdio0_device, &sd_detect_gpio, sizeof(sd_detect_gpio));//saenghee_testcode : need to ask
    platform_device_add_data(&sprd_backlight_device,&ktd253b_data,sizeof(ktd253b_data));//saenghee_testcode : need to ask
	platform_device_add_data(&sprd_serial_device0,(const void*)&plat_data0,sizeof(plat_data0));
	platform_device_add_data(&sprd_serial_device1,(const void*)&plat_data1,sizeof(plat_data1));
	platform_device_add_data(&sprd_serial_device2,(const void*)&plat_data2,sizeof(plat_data2));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sc8810_add_i2c_devices();
	sc8810_add_misc_devices();
	sprd_spi_init();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	sprd_ramconsole_init();
#endif
	sys_debug_init();
}

static void __init sc8810_fixup(struct machine_desc *desc, struct tag *tag,
		char **cmdline, struct meminfo *mi)
{
	struct tag *t = tag;

	/* manipulate cmdline if not in calibration mode, for mfserial */
	for (; t->hdr.size; t = (struct tag*)((__u32*)(t) + (t)->hdr.size)) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			char *p, *c;
			c = (char*)t->u.cmdline.cmdline;
			if(strstr(c, "calibration=") == NULL) {
				p = strstr(c, "console=");
				/* break it, if exists */
				if (p)
					*p = 'O';
				/* add our kernel parameters */
				strcat(c, "console=ttyS1,115200n8 loglevel=0");
			}
			break;
		}
	}
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
