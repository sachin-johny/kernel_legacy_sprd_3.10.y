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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/earlysuspend.h>

#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/adi.h>

//#define SPRD_BACKLIGHT_DBG
#ifdef SPRD_BACKLIGHT_DBG
#define ENTER printk(KERN_INFO "[SPRD_BACKLIGHT_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_BACKLIGHT_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[SPRD_BACKLIGHT_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_BACKLIGHT_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

/* register definitions */
#define        PWM_PRESCALE    (0x0000)
#define        PWM_CNT         (0x0004)
#define        PWM_TONE_DIV    (0x0008)
#define        PWM_PAT_LOW     (0x000C)
#define        PWM_PAT_HIG     (0x0010)

#define        PWM_ENABLE      (1 << 8)
#define        PWM_SCALE       1
#define        PWM_REG_MSK     0xffff
#define        PWM_MOD_MAX     0xff

/*****************************************
*  level              mV            step
*  0 ~  63      0  mV ~ 189 mV      3 mV
* 64 ~  95     192 mV ~ 378 mV      6 mV
* 96 ~ 127     384 mV ~ 756 mV     12 mV
******************************************/
#define        MAX_VOLTAGE_LEVEL    70
#define        MIN_VOLTAGE_LEVEL    4

/* sprdtrum backlight have two driven mode:
 * 1) pwm mode  //you need to define SPRD_BACKLIGHT_PWM
 * 2) white led mode //you need to deifine SPRD_BACKLIGHT_WHITELED
 *	2.1) series mode //whiteled driven by dimming PWM,
	you need to define the DIMMING_PWD_BASE
 *	2.2) parallel mode //whiteled driven by pd PWM,
 *	you need to define the PD_PWD_BASE
 */

#ifdef CONFIG_ARCH_SCX35

#if 0
	/*if the backlight is driven by pwm, use this MACRO */
	#define SPRD_BACKLIGHT_PWM
#else
	/*the backlight is driven by whiteled default */
	#define SPRD_BACKLIGHT_WHITELED
	#define SPRD_DIM_PWM_MODE
	#define DIMMING_PWD_BASE	(SPRD_MISC_BASE + 0x8020)
	#define PD_PWM_BASE		DIMMING_PWD_BASE
#endif

#endif

enum bl_pwm_mode {
	normal_pwm,
	dim_pwm,
	pd_pwd,
};

struct sprd_bl_devdata {
	enum bl_pwm_mode	pwm_mode;
	u32		pwm_index;
	struct backlight_device *bldev;
	int             suspend;
	struct clk      *clk;
	struct early_suspend sprd_early_suspend_desc;
};

static struct sprd_bl_devdata sprdbl;

#if defined(SPRD_BACKLIGHT_PWM)
static inline uint32_t pwm_read(int index, uint32_t reg)
{
       return __raw_readl(SPRD_PWM_BASE + index * 0x20 + reg);
}

static void pwm_write(int index, uint32_t value, uint32_t reg)
{
       __raw_writel(value, SPRD_PWM_BASE + index * 0x20 + reg);
}

static int sprd_bl_pwm_update_status(struct backlight_device *bldev)
{
	u32 bl_brightness;

	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
			bldev->props.power != FB_BLANK_UNBLANK ||
			sprdbl.suspend ||
			bldev->props.brightness == 0) {
		/* disable backlight */
		pwm_write(sprdbl.pwm_index, 0, PWM_PRESCALE);
		clk_disable(sprdbl.clk);
	} else {
		bl_brightness = bldev->props.brightness & PWM_MOD_MAX;
		#if 0
		/*why??, the min brightness is 0x20?*/
		if (bl_brightness < 0x20)
			bl_brightness = 0x20;
		#endif

		clk_enable(sprdbl.clk);

		pwm_write(sprdbl.pwm_index, PWM_SCALE, PWM_PRESCALE);
		pwm_write(sprdbl.pwm_index, (bl_brightness << 8) | PWM_MOD_MAX, PWM_CNT);
		pwm_write(sprdbl.pwm_index, PWM_REG_MSK, PWM_PAT_LOW);
		pwm_write(sprdbl.pwm_index, PWM_REG_MSK, PWM_PAT_HIG);
		pwm_write(sprdbl.pwm_index, PWM_SCALE | PWM_ENABLE, PWM_PRESCALE);
	}

	return 0;
}

static int sprd_bl_pwm_get_brightness(struct backlight_device *bldev)
{
	return (pwm_read(sprdbl.pwm_index, PWM_CNT) >> 8) & PWM_MOD_MAX;
}

static const struct backlight_ops sprd_backlight_ops = {
	.update_status = sprd_bl_pwm_update_status,
	.get_brightness = sprd_bl_pwm_get_brightness,
};

#elif defined(SPRD_BACKLIGHT_WHITELED)

static int sprd_bl_whiteled_update_status(struct backlight_device *bldev)
{
	u32 bl_brightness, led_level, pwm_level;
	u32 reg_val;

	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
			bldev->props.power != FB_BLANK_UNBLANK ||
			sprdbl.suspend ||
			bldev->props.brightness == 0) {
		/* disable backlight */
		if (sprdbl.pwm_mode == dim_pwm) {
			sci_adi_clr(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_BOOST_EN);
		} else {
			/*yes, 1 is disbale and 0 is enable*/
			sci_adi_set(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_PD);
		}
	} else {
		bl_brightness = bldev->props.brightness & PWM_MOD_MAX;
		pwm_level = bl_brightness & 0x3;
		/*duty ratio = 25% 50% 75% or 100%*/
		pwm_level = (PWM_MOD_MAX >> 2) * (pwm_level + 1) & PWM_MOD_MAX;

		if (sprdbl.pwm_mode == dim_pwm) {
			/*series mode*/
			/*whiteled config*/
			led_level = (((MAX_VOLTAGE_LEVEL - MIN_VOLTAGE_LEVEL + 1) * bl_brightness) / (PWM_MOD_MAX + 1)) + MIN_VOLTAGE_LEVEL;
			PRINT_DBG("user requested brightness = %d, caculated led_level = %d\n", bldev->props.brightness, led_level);
			if ((int)led_level < 0) {
				return led_level;
			}
			reg_val = sci_adi_read(ANA_REG_GLB_WHTLED_CTRL1);
			reg_val &= ~(0x7f << 5);
			reg_val |= led_level << 5;
			sci_adi_raw_write(ANA_REG_GLB_WHTLED_CTRL1, reg_val);
			#if 0
			/*dimming pwm config*/
			sci_adi_raw_write(DIMMING_PWD_BASE + PWM_SCALE, PWM_PRESCALE);
			sci_adi_raw_write(DIMMING_PWD_BASE + PWM_CNT,
					(pwm_level << 8) | PWM_MOD_MAX);
			sci_adi_raw_write(DIMMING_PWD_BASE + PWM_SCALE,
					PWM_SCALE | PWM_ENABLE);
			#endif
			/*enable the whiteled bootst output*/
			sci_adi_set(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_SERIES_EN);
			sci_adi_set(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_BOOST_EN);
		} else {
			/*parallel mode*/
			led_level = (bl_brightness >> 2) & 0x3f;
			reg_val = sci_adi_read(ANA_REG_GLB_WHTLED_CTRL0);
			reg_val &= ~(0x3f << 1);
			reg_val |= led_level << 1;
			sci_adi_raw_write(ANA_REG_GLB_WHTLED_CTRL0, reg_val);

			sci_adi_raw_write(PD_PWM_BASE + PWM_SCALE, PWM_PRESCALE);
			sci_adi_raw_write(PD_PWM_BASE + PWM_CNT,
					(pwm_level << 8) | PWM_MOD_MAX);

			sci_adi_raw_write(PD_PWM_BASE + PWM_SCALE,
					PWM_SCALE | PWM_ENABLE);

			sci_adi_clr(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_SERIES_EN);
			sci_adi_clr(ANA_REG_GLB_WHTLED_CTRL0, BIT_WHTLED_PD);
		}
	}

	return 0;
}

static int sprd_bl_whiteled_get_brightness(struct backlight_device *bldev)
{
	/*fixme*/
	return 0;
}

static const struct backlight_ops sprd_backlight_ops = {
	.update_status = sprd_bl_whiteled_update_status,
	.get_brightness = sprd_bl_whiteled_get_brightness,
};
#else
#error "please define the backlight control mode"
#endif

#ifdef CONFIG_EARLYSUSPEND
static void sprd_backlight_earlysuspend(struct early_suspend *h)
{
	sprdbl.suspend = 1;
}

static void sprd_backlight_lateresume(struct early_suspend *h)
{
	sprdbl.suspend = 0;
	sprd_backlight_ops.update_status(sprdbl.bldev);
}
#endif

static int sprd_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;

#ifdef SPRD_BACKLIGHT_PWM
	struct resource *pwm_res;
	char pwm_clk_name[32];

	pwm_res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (IS_ERR(pwm_res)) {
		printk("Can't get pwm resource");
		return -ENODEV;
	}

	sprdbl.pwm_index = pwm_res->start;
	/*fixme, the pwm's clk name must like this:clk_pwmx*/
	sprintf(pwm_clk_name, "%s%d", "clk_pwm", sprdbl.pwm_index);
	sprdbl.clk = clk_get(&pdev->dev, pwm_clk_name);
	if (IS_ERR(sprdbl.clk)) {
		printk("Can't get pwm's clk");
		return -ENODEV;
	}
	sprdbl.pwm_mode = normal_pwm;
#else

#ifdef SPRD_DIM_PWM_MODE
	sprdbl.pwm_mode = dim_pwm;
#else
	sprdbl.pwm_mode = pd_pwd;
#endif

#endif
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = PWM_MOD_MAX;
	props.type = BACKLIGHT_RAW;
	/*the default brightness = 1/2 max brightness */
	props.brightness = PWM_MOD_MAX >> 1;
	props.power = FB_BLANK_UNBLANK;

	bldev = backlight_device_register(
			pdev->name, &pdev->dev,
			&sprdbl, &sprd_backlight_ops, &props);
	if (IS_ERR(bldev)) {
		printk(KERN_ERR "Failed to register backlight device\n");
		return -ENOMEM;
	}
	sprdbl.bldev = bldev;
	platform_set_drvdata(pdev, bldev);

#ifdef CONFIG_EARLYSUSPEND
	sprdbl.sprd_early_suspend_desc.level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	sprdbl.sprd_early_suspend_desc.suspend	= sprd_backlight_earlysuspend;
	sprdbl.sprd_early_suspend_desc.resume	= sprd_backlight_lateresume;
	register_early_suspend(&sprdbl.sprd_early_suspend_desc);
#endif

	return 0;
}

static int sprd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = PWM_MOD_MAX;

	backlight_update_status(bldev);

	backlight_device_unregister(bldev);

	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&sprdbl.sprd_early_suspend_desc);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int sprd_backlight_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int sprd_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define sprd_backlight_suspend NULL
#define sprd_backlight_resume NULL
#endif

static struct platform_driver sprd_backlight_driver = {
	.probe = sprd_backlight_probe,
	.remove = sprd_backlight_remove,
	.suspend = sprd_backlight_suspend,
	.resume = sprd_backlight_resume,
	.driver = {
		.name = "sprd_backlight",
		.owner = THIS_MODULE,
	},
};

static int __init sprd_backlight_init(void)
{
	return platform_driver_register(&sprd_backlight_driver);
}

static void __exit sprd_backlight_exit(void)
{
	platform_driver_unregister(&sprd_backlight_driver);
}

module_init(sprd_backlight_init);
module_exit(sprd_backlight_exit);

MODULE_DESCRIPTION("Spreadtrum backlight Driver");
