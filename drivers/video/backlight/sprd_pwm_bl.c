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
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/globalregs.h>
#include <mach/hardware.h>
/* register definitions */
#define	BK_MOD_MAX	0xff

#define GR_CLK_EN_PWM			(SPRD_GREG_BASE + 0x0074)
#define PIN_MOD_PWMA			(SPRD_PIN_BASE  + 0x0138)
#define SPRD_PWM0_PRESCALE		(SPRD_PWM_BASE + 0x0000 + 0x40)
#define SPRD_PWM0_CNT			(SPRD_PWM_BASE + 0x0004 + 0x40)
#define SPRD_PWM0_PAT_LOW		(SPRD_PWM_BASE + 0x000C + 0x40)
#define SPRD_PWM0_PAT_HIG		(SPRD_PWM_BASE + 0x0010 + 0x40)

#define PIN_PWM0_MOD_VALUE   0x20

/* register definitions */
#define        PWM_PRESCALE    (0x0000)
#define        PWM_CNT         (0x0004)
#define        PWM_TONE_DIV    (0x0008)
#define        PWM_PAT_LOW     (0x000C)
#define        PWM_PAT_HIG     (0x0010)


#define        PWM_ENABLE      (1 << 8)
#define        PWM_SCALE       8
#define        PWM_REG_MSK     0xffff
#define        PWM_MOD_MAX     0xff

static inline void __raw_bits_or(unsigned int v, unsigned int a)
{
	__raw_writel((__raw_readl(a) | v), a);
}

static void BK_SetPwmRatio(unsigned short value)
{

	__raw_bits_or((0X1 << 23), GR_CLK_EN_PWM);
	__raw_bits_or((0X1 << 27), GR_CLK_EN_PWM);
	__raw_bits_or(0x20, PIN_MOD_PWMA);
	__raw_writel(PWM_SCALE, SPRD_PWM0_PRESCALE);
	__raw_writel(value, SPRD_PWM0_CNT);
	__raw_writel(PWM_REG_MSK, SPRD_PWM0_PAT_LOW);
	__raw_writel(PWM_REG_MSK, SPRD_PWM0_PAT_HIG);

	__raw_bits_or(PWM_ENABLE|PWM_SCALE, SPRD_PWM0_PRESCALE);

}

static int sprd_pwm_backlight_update_status(struct backlight_device *bldev)
{
	uint32_t value;
	unsigned long duty_mod= 0;
	struct mutex  mutex;

	//mutex_lock(&mutex);
	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
			bldev->props.power != FB_BLANK_UNBLANK ||
			bldev->props.brightness == 0) {
		/* disable backlight */
		BK_SetPwmRatio(0);
	} else {
		value = bldev->props.brightness & BK_MOD_MAX;
		if(value > BK_MOD_MAX)
			value = BK_MOD_MAX;

		if(value < 0)
			value = 0;

		duty_mod = (value << 8) | BK_MOD_MAX;
		BK_SetPwmRatio(duty_mod);

	}
	//mutex_unlock(&mutex);
	return 0;
}
static int sprd_pwm_backlight_get_brightness(struct backlight_device *bldev)
{
	return 0;
}

static const struct backlight_ops sprd_pwm_backlight_ops = {
	.update_status = sprd_pwm_backlight_update_status,
	.get_brightness = sprd_pwm_backlight_get_brightness,
};

static int __devinit sprd_pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = BK_MOD_MAX;
	props.type = BACKLIGHT_RAW;
	props.brightness = BK_MOD_MAX;
	props.power = FB_BLANK_UNBLANK;


	bldev = backlight_device_register(
			dev_name(&pdev->dev), &pdev->dev,
			NULL, &sprd_pwm_backlight_ops, &props);
	if (IS_ERR(bldev)) {
		printk(KERN_ERR "Failed to register backlight device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, bldev);
	BK_SetPwmRatio(50<<8);

	return 0;

}

static int __devexit sprd_pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = BK_MOD_MAX;
	backlight_update_status(bldev);
	backlight_device_unregister(bldev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int sprd_pwm_backlight_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int sprd_pwm_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define sprd_pwm_backlight_suspend NULL
#define sprd_pwm_backlight_resume NULL
#endif

static struct platform_driver sprd_pwm_backlight_driver = {
	.probe = sprd_pwm_backlight_probe,
	.remove = __devexit_p(sprd_pwm_backlight_remove),
	.suspend = sprd_pwm_backlight_suspend,
	.resume = sprd_pwm_backlight_resume,
	.driver = {
		.name = "sprd_backlight",
		.owner = THIS_MODULE,
	},
};

static int __init sprd_pwm_backlight_init(void)
{
	return platform_driver_register(&sprd_pwm_backlight_driver);
}

static void __exit sprd_pwm_backlight_exit(void)
{
	platform_driver_unregister(&sprd_pwm_backlight_driver);
}

module_init(sprd_pwm_backlight_init);
module_exit(sprd_pwm_backlight_exit);

MODULE_DESCRIPTION("sprd_pwm Backlight Driver");
MODULE_LICENSE("GPL v2");
