/*
 * LED driver for Sprd lcd driven LEDS.
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
 *
 *  usage :
	  echo 255 > /sys/class/leds/lcd-backlight/brightness
	  cat /sys/class/leds/lcd-backlight/brightness
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/earlysuspend.h>
#include <mach/io.h>

#include <mach/adi.h>
#include <mach/globalregs.h>
#include <mach/hardware.h>

#define PIN_MOD_PWMA            (SPRD_CPC_BASE + 0x03e0)
#define SPRD_PWM_REG(off) 		(SPRD_PWM_BASE + (off))
#define SPRD_PWM0_PRESCALE   	SPRD_PWM_REG(0x0000)
#define SPRD_PWM0_CNT 			SPRD_PWM_REG(0x0004)
#define SPRD_PWM0_TONE_DIV 	SPRD_PWM_REG(0x0008)
#define SPRD_PWM0_PAT_LOW 	SPRD_PWM_REG(0x000C)
#define SPRD_PWM0_PAT_HIG 	SPRD_PWM_REG(0x0010)

#define LCD_PWM_PRESCALE_VALUE 	0x01
#define LCD_PWM_MOD_VALUE 		0xFF
#define PWM_REG_MSK_VALUE			0xFFFF

#define PIN_PWM0_MOD_VALUE   0x20

#define BIT_5               0x00000020
#define BIT_6               0x00000040
#define BIT_8               0x00000100

#define LCD_PWM0_EN BIT_8

#if defined (CONFIG_MACH_SP6821A)
#define SPRD_ANA_BASE 	        (SPRD_MISC_BASE + 0x600)
#define ANA_REG_BASE            SPRD_ANA_BASE
#define ANA_LED_CTL           (ANA_REG_BASE + 0X68)
#define WHTLED_CTL              ANA_LED_CTL
#define WHTLED_PD_SET           BIT_5
#define WHTLED_PD_RST           BIT_6
#define WHTLED_V_SHIFT          0
#define WHTLED_V_MSK            (0x1F << WHTLED_V_SHIFT)
#else
#error not surpport
#endif

#define ANA_REG_OR(reg_addr, value)     \
    do{\
        unsigned short adi_tmp_val = sci_adi_read(reg_addr); \
        adi_tmp_val |= (unsigned short)(value); \
        sci_adi_raw_write(reg_addr, adi_tmp_val); \
    }while(0)
#define ANA_REG_MSK_OR(reg_addr, value, msk)        \
    do{\
        unsigned short adi_tmp_val = sci_adi_read(reg_addr); \
        adi_tmp_val &= (unsigned short)(~(msk)); \
        adi_tmp_val |= (unsigned short)((value)&(msk)); \
        sci_adi_raw_write(reg_addr, adi_tmp_val); \
    }while(0)
#define ANA_REG_AND(reg_addr, value)    \
    do{\
        unsigned short adi_tmp_val = sci_adi_read(reg_addr); \
        adi_tmp_val &= (unsigned short)(value); \
        sci_adi_raw_write(reg_addr, adi_tmp_val); \
    }while(0)

/* sprd keypad backlight */
struct sprd_lcd_led {
	struct platform_device *pdev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	int suspend;
	struct early_suspend sprd_early_suspend_desc;
};

#define to_sprd_led(led_cdev) \
	container_of(led_cdev, struct sprd_lcd_led, cdev)

static void led_work(struct work_struct *);

#ifdef CONFIG_EARLYSUSPEND
static void sprd_lcd_led_earlysuspend(struct early_suspend *h)
{
	struct sprd_lcd_led *led = container_of(h, struct sprd_lcd_led, sprd_early_suspend_desc);

	printk("\n\033[31m%s()\033[0m\n", __func__);
	mutex_lock(&led->mutex);
	led->suspend = 1;
	mutex_unlock(&led->mutex);
}
static void sprd_lcd_led_lateresume(struct early_suspend *h)
{
	struct sprd_lcd_led *led = container_of(h, struct sprd_lcd_led, sprd_early_suspend_desc);

	printk("\n\033[31m%s()\033[0m\n", __func__);
	mutex_lock(&led->mutex);
	led->suspend = 0;
	mutex_unlock(&led->mutex);
	led_work(&led->work);
}
#endif

static void LCD_SetBackLightBrightness( unsigned long  value)
{
#if defined (CONFIG_MACH_SP6821A)
	if(value > 255)
		value = 255;

    if(value > 8)
	    value = value/8;
    else
        value = 0;

    ANA_REG_MSK_OR (WHTLED_CTL, ( (value << WHTLED_V_SHIFT) &WHTLED_V_MSK), WHTLED_V_MSK);
#else
#error not surpport
#endif
}


static void sprd_led_enable(struct sprd_lcd_led *led)
{
	//open lcm backlight
#if defined (CONFIG_MACH_SP6821A)
    ANA_REG_AND (WHTLED_CTL, ~ (WHTLED_PD_SET|WHTLED_PD_RST));
    ANA_REG_OR (WHTLED_CTL, WHTLED_PD_RST);
#else
#error not surpport
#endif

	LCD_SetBackLightBrightness(led->value);
}

static void sprd_led_disable(struct sprd_lcd_led *led)
{
	//close lcm backlight
#if defined (CONFIG_MACH_SP6821A)
    ANA_REG_AND (WHTLED_CTL, ~ (WHTLED_PD_SET|WHTLED_PD_RST));
    ANA_REG_OR (WHTLED_CTL, WHTLED_PD_SET);
#else
#error not surpport
#endif

	LCD_SetBackLightBrightness(0);
}

static void led_work(struct work_struct *work)
{
	struct sprd_lcd_led *led = container_of(work, struct sprd_lcd_led, work);
	unsigned long flags;

	mutex_lock(&led->mutex);
	spin_lock_irqsave(&led->value_lock, flags);
	if (led->value == LED_OFF || led->suspend) {
		spin_unlock_irqrestore(&led->value_lock, flags);
		sprd_led_disable(led);
		goto out;
	}
	spin_unlock_irqrestore(&led->value_lock, flags);
	sprd_led_enable(led);
out:
	mutex_unlock(&led->mutex);
}

static void sprd_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct sprd_lcd_led *led = to_sprd_led(led_cdev);
	unsigned long flags;

	spin_lock_irqsave(&led->value_lock, flags);
	led->value = value;
	spin_unlock_irqrestore(&led->value_lock, flags);
	schedule_work(&led->work);
}

static void sprd_lcd_led_shutdown(struct platform_device *pdev)
{
	struct sprd_lcd_led *led = platform_get_drvdata(pdev);

	mutex_lock(&led->mutex);
	led->value = LED_OFF;
	sprd_led_disable(led);
	mutex_unlock(&led->mutex);
}

static int sprd_lcd_led_probe(struct platform_device *pdev)
{
	struct sprd_lcd_led *led;
	int ret;

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		ret = -ENOMEM;
		goto err_led;
	}

	led->cdev.brightness_set = sprd_led_set;
	led->cdev.default_trigger = "heartbeat";
	led->cdev.name = "lcd-backlight";
	led->cdev.brightness_get = NULL;
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;
	led->suspend = 0;

	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, led_work);
	led->value = LED_OFF;
	platform_set_drvdata(pdev, led);

	ret = led_classdev_register(&pdev->dev, &led->cdev);

	if (ret < 0)
		goto err_led;

#ifdef CONFIG_EARLYSUSPEND
	led->sprd_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	led->sprd_early_suspend_desc.suspend = sprd_lcd_led_earlysuspend;
	led->sprd_early_suspend_desc.resume = sprd_lcd_led_lateresume;
	register_early_suspend(&led->sprd_early_suspend_desc);
#endif

	return 0;

 err_led:
	kfree(led);
	return ret;
}

static int sprd_lcd_led_remove(struct platform_device *pdev)
{
	struct sprd_lcd_led *led = platform_get_drvdata(pdev);

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&led->sprd_early_suspend_desc);
#endif
	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->value = LED_OFF;
	sprd_led_disable(led);
	kfree(led);

	return 0;
}

static struct platform_driver sprd_lcd_led_driver = {
	.driver = {
			.name = "lcd-backlight",
			.owner = THIS_MODULE,
		   },
	.probe = sprd_lcd_led_probe,
	.remove = sprd_lcd_led_remove,
	.shutdown = sprd_lcd_led_shutdown,
};

static int __devinit sprd_lcd_led_init(void)
{
	return platform_driver_register(&sprd_lcd_led_driver);
}

static void sprd_lcd_led_exit(void)
{
	platform_driver_unregister(&sprd_lcd_led_driver);
}
module_init(sprd_lcd_led_init);
module_exit(sprd_lcd_led_exit);

MODULE_DESCRIPTION("Sprd SC8800G lcd backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lcd-backlight");

