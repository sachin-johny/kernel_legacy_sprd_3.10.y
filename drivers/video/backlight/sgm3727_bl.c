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
#include <mach/hardware.h>

#include <linux/delay.h>
#include <linux/gpio.h>


/* register definitions */
#define	BK_DATA		(0x0000)
#define	BK_MSK		(0x0004)
#define	BK_DIR		(0x0008)

#define	BK_ENABLE	(1 << 15)
#define	BK_INDEX	7
#define	BK_MOD_MAX	0xff
#define	BK_GPIO		143
#define	BK_LOW		0
#define	BK_HIGH		1

struct sc8810bl {
	int		pwm;
	struct clk	*clk;
};

static struct sc8810bl sc8810bl;


static void pulse_write(int index, uint32_t value, uint32_t reg)
{
	__raw_writel(value, SPRD_GPIO_BASE + index * 0x80 + reg);
}


static unsigned long step_bak = 0;

static void LCD_setSinglePulse(unsigned long value)
{
	unsigned long value_ret;
	unsigned long flags;
	int step_res;

	if(value > 255)
		value = 255;
	if(value < 0)
		value = 0;

	value_ret = 32 - value/8;

	if(32 == value_ret) {
		gpio_set_value(BK_GPIO, BK_LOW);
		mdelay(3);
		step_bak = value_ret;
		return;
	}

	step_res = 0;

	if(value_ret < step_bak) {
		step_res = 32 - step_bak + value_ret;
		while(step_res-- > 0) {
			local_irq_save(flags);
			gpio_set_value(BK_GPIO, BK_LOW);
			udelay(30);
			gpio_set_value(BK_GPIO, BK_HIGH);
			local_irq_restore(flags);
			udelay(30);
		}
	} else {
		step_res = value_ret - step_bak;
		while(step_res-- > 0) {
			local_irq_save(flags);
			gpio_set_value(BK_GPIO, BK_LOW);
			udelay(30);
			gpio_set_value(BK_GPIO, BK_HIGH);
			local_irq_restore(flags);
			udelay(30);
		}
	}
	step_bak = value_ret;

}

static int sc8810_backlight_update_status(struct backlight_device *bldev)
{
	uint32_t value;

	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
			bldev->props.power != FB_BLANK_UNBLANK ||
			bldev->props.brightness == 0) {
		/* disable backlight */
		LCD_setSinglePulse(0);
	} else {

		value = bldev->props.brightness & BK_MOD_MAX;
		LCD_setSinglePulse(value);
	}

	return 0;
}

static int sc8810_backlight_get_brightness(struct backlight_device *bldev)
{
	return 0;
}

static const struct backlight_ops sc8810_backlight_ops = {
	.update_status = sc8810_backlight_update_status,
	.get_brightness = sc8810_backlight_get_brightness,
};

static int __devinit sc8810_backlight_probe(struct platform_device *pdev)
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
			&sc8810bl, &sc8810_backlight_ops, &props);
	if (IS_ERR(bldev)) {
		printk(KERN_ERR "Failed to register backlight device\n");
		return -ENOMEM;
	}

	/* Enable GPIO: Output - 1 */
	pulse_write(BK_INDEX, BK_ENABLE, BK_MSK);
	pulse_write(BK_INDEX, BK_ENABLE, BK_DIR);
	pulse_write(BK_INDEX, BK_ENABLE, BK_DATA);

	platform_set_drvdata(pdev, bldev);

	return 0;
}

static int __devexit sc8810_backlight_remove(struct platform_device *pdev)
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
static int sc8810_backlight_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int sc8810_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define sc8810_backlight_suspend NULL
#define sc8810_backlight_resume NULL
#endif

static struct platform_driver sc8810_backlight_driver = {
	.probe = sc8810_backlight_probe,
	.remove = __devexit_p(sc8810_backlight_remove),
	.suspend = sc8810_backlight_suspend,
	.resume = sc8810_backlight_resume,
	.driver = {
		.name = "sprd_backlight",
		.owner = THIS_MODULE,
	},
};

static int __init sc8810_backlight_init(void)
{
	return platform_driver_register(&sc8810_backlight_driver);
}

static void __exit sc8810_backlight_exit(void)
{
	platform_driver_unregister(&sc8810_backlight_driver);
}

module_init(sc8810_backlight_init);
module_exit(sc8810_backlight_exit);

MODULE_DESCRIPTION("SC8810 Backlight Driver");
MODULE_LICENSE("GPL v2");
