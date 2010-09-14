/*
 * LED driver for Sprd lcd driven LEDS.
 *
 * Copyright(C) 2007, 2008 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/* 
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
#include <mach/mfp.h>
#include <mach/regs_cpc.h>
#include <mach/regs_gpio.h>

#define REG_CPC_LCD_EN				(*((volatile unsigned int *)(CPC_LCD_EN_REG)))
#define REG_GPIO6_DATA				(*((volatile unsigned int *)(GPIO_PG6_BASE + GPIO_DATA)))
#define REG_GPIO6_DMSK				(*((volatile unsigned int *)(GPIO_PG6_BASE + GPIO_DMSK)))
#define REG_GPIO6_DIR				(*((volatile unsigned int *)(GPIO_PG6_BASE + GPIO_DIR)))
#define GPIO_LCDEN				(103)				
#define GPIO_OFFSET				(GPIO_LCDEN % 16)

/* sprd keypad backlight */
struct sprd_lcd_led {
	struct platform_device *pdev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	int enabled;
};

static unsigned long lcd_bl_cfg;

#define to_sprd_led(led_cdev) \
	container_of(led_cdev, struct sprd_lcd_led, cdev)

static void LCD_SetBackLightBrightness( unsigned long  brightness)
{
	unsigned long i = 0;
	unsigned long flags;

	if (0 != brightness) {
		if (brightness >= 100) {
	        	brightness = 96;
	        }
        
		if (brightness >= 6) {
			brightness = brightness / 6;
		} else {
			brightness = 1;
		}
		
		brightness = 16 - brightness;
		REG_GPIO6_DATA = REG_GPIO6_DATA & ~(1 << GPIO_OFFSET);	
		mdelay(2);
		REG_GPIO6_DATA = REG_GPIO6_DATA | (1 << GPIO_OFFSET);	
		udelay(30);
		REG_GPIO6_DATA = REG_GPIO6_DATA & ~(1 << GPIO_OFFSET);	
		udelay(30);

		for (i = 1 ; i < brightness ; i++) {
			REG_GPIO6_DATA = REG_GPIO6_DATA | (1 << GPIO_OFFSET);	
			udelay(30);
			REG_GPIO6_DATA = REG_GPIO6_DATA & ~(1 << GPIO_OFFSET);	
    			udelay(30);
		}

		REG_GPIO6_DATA = REG_GPIO6_DATA | (1 << GPIO_OFFSET);	
	} else {
		REG_GPIO6_DATA = REG_GPIO6_DATA & ~(1 << GPIO_OFFSET);	
	}
	mdelay(1);		
}


static void sprd_led_enable(struct sprd_lcd_led *led)
{
/*
	if (led->enabled)
		return;
*/
	/* backlight on */
	REG_GPIO6_DATA = REG_GPIO6_DATA | (1 << GPIO_OFFSET);
	LCD_SetBackLightBrightness(led->value);

	led->enabled = 1;
}

static void sprd_led_disable(struct sprd_lcd_led *led)
{
/*
	if (!led->enabled)
		return;
*/
	/* backlight off */
	REG_GPIO6_DATA = REG_GPIO6_DATA & ~(1 << GPIO_OFFSET);
	LCD_SetBackLightBrightness(led->value);

	led->enabled = 0;
}

static void led_work(struct work_struct *work)
{
	struct sprd_lcd_led *led = container_of(work, struct sprd_lcd_led, work);
	unsigned long flags;

	mutex_lock(&led->mutex);
	spin_lock_irqsave(&led->value_lock, flags);
	if (led->value == LED_OFF) {
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
	led->enabled = 1;
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
	led->enabled = 0;

	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, led_work);
	led->value = LED_OFF;
	platform_set_drvdata(pdev, led);
	
	ret = led_classdev_register(&pdev->dev, &led->cdev);
	
	if (ret < 0)
		goto err_led;
		
	/* backlight on */
	lcd_bl_cfg = MFP_CFG_X(LCD_EN, GPIO, DS0, PULL_NONE, IO_OE);
	sprd_mfp_config(&lcd_bl_cfg, 1);
	REG_GPIO6_DMSK = REG_GPIO6_DMSK | (1 << GPIO_OFFSET);
	REG_GPIO6_DIR = REG_GPIO6_DIR | (1 << GPIO_OFFSET);

	led->value = LED_FULL;
	led->enabled = 0;
	schedule_work(&led->work);

	return 0;

 err_led:
	kfree(led);
	return ret;

}

static int sprd_lcd_led_remove(struct platform_device *pdev)
{
	struct sprd_lcd_led *led = platform_get_drvdata(pdev);

	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->value = LED_OFF;
	led->enabled = 1;
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
module_init(sprd_lcd_led_init);

static void sprd_lcd_led_exit(void)
{
	platform_driver_unregister(&sprd_lcd_led_driver);
}
module_exit(sprd_lcd_led_exit);

MODULE_AUTHOR("Richard Feng");
MODULE_DESCRIPTION("Sprd LED lcd backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lcd-backlight");
