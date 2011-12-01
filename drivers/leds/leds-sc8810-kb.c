/*
 * LED driver for Sprd kb driven LEDS.
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
 *  usage :
      echo 255 > /sys/class/leds/keyboard-backlight/brightness
          cat /sys/class/leds/keyboard-backlight/brightness
 * 
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>

/* sprd keypad backlight */
struct sprd_kb_led {
	struct platform_device *pdev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	int enabled;
};

#define to_sprd_led(led_cdev) \
	container_of(led_cdev, struct sprd_kb_led, cdev)
	
static void Kb_SetBackLightBrightness( unsigned long  value)
{
	if(value > 255)
		value = 255;
	
    if(value > 32)
	    value = value/32;
    else
        value = 0;
	
    // Set Output Current
    ANA_REG_MSK_OR (KPLED_CTL, ( (value << KPLED_V_SHIFT) &KPLED_V_MSK), KPLED_V_MSK);
}
static void sprd_led_enable(struct sprd_kb_led *led)
{
/*
     if (led->enabled)
		return;
*/
	/* backlight on */
    ANA_REG_AND (KPLED_CTL, ~ (KPLED_PD_SET|KPLED_PD_RST));
    ANA_REG_OR (KPLED_CTL, KPLED_PD_RST);
	   
	Kb_SetBackLightBrightness(led->value);
	
	led->enabled = 1;
}

static void sprd_led_disable(struct sprd_kb_led *led)
{
/*
    if (!led->enabled)
		return;
*/
	/* backlight off */
	ANA_REG_AND (KPLED_CTL, ~ (KPLED_PD_SET|KPLED_PD_RST));
    ANA_REG_OR (KPLED_CTL, KPLED_PD_SET);
	   
	Kb_SetBackLightBrightness(led->value);
	
	led->enabled = 0;
}

static void led_work(struct work_struct *work)
{
	struct sprd_kb_led *led = container_of(work, struct sprd_kb_led, work);
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
	struct sprd_kb_led *led = to_sprd_led(led_cdev);
	unsigned long flags;
	
	spin_lock_irqsave(&led->value_lock, flags);
	led->value = value;
	spin_unlock_irqrestore(&led->value_lock, flags);
	schedule_work(&led->work);	
}

static void sprd_kb_led_shutdown(struct platform_device *pdev)
{
	struct sprd_kb_led *led = platform_get_drvdata(pdev);

	mutex_lock(&led->mutex);
	led->value = LED_OFF;
	led->enabled = 1;
	sprd_led_disable(led);
	mutex_unlock(&led->mutex);
}

static int sprd_kb_led_probe(struct platform_device *pdev)
{
	struct sprd_kb_led *led;
	int ret;

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		ret = -ENOMEM;
		goto err_led;
	}

	led->cdev.brightness_set = sprd_led_set;
	led->cdev.default_trigger = "heartbeat";
	led->cdev.name = "keyboard-backlight";
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
	//FIXME
	
	led->value = LED_FULL;
	led->enabled = 0;
	schedule_work(&led->work);

	return 0;

 err_led:
	kfree(led);
	return ret;
}

static int sprd_kb_led_remove(struct platform_device *pdev)
{
	struct sprd_kb_led *led = platform_get_drvdata(pdev);

	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->value = LED_OFF;
	led->enabled = 1;
	sprd_led_disable(led);
	kfree(led);

	return 0;
}

static struct platform_driver sprd_kb_led_driver = {
	.driver = {
			.name = "keyboard-backlight",
			.owner = THIS_MODULE,
		   },
	.probe = sprd_kb_led_probe,
	.remove = sprd_kb_led_remove,
	.shutdown = sprd_kb_led_shutdown,
};

static int __devinit sprd_kb_led_init(void)
{
	return platform_driver_register(&sprd_kb_led_driver);
}

static void sprd_kb_led_exit(void)
{
	platform_driver_unregister(&sprd_kb_led_driver);
}

module_init(sprd_kb_led_init);
module_exit(sprd_kb_led_exit);

MODULE_DESCRIPTION("Sprd SC8800G Keyboadr backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:keyboard-backlight");

