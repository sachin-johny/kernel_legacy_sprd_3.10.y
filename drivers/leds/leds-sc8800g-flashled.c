/*
 * LED driver for Sprd flash led driven LEDS.
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
	  echo 100 > /sys/class/leds/flash-led/brightness
	  cat /sys/class/leds/flash-led/brightness
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <asm/io.h>
#include <mach/board.h>
#include <mach/gpio.h>

/* sprd keypad backlight */
struct sprd_flash_led {
	struct platform_device *pdev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	int enabled;
};

#define to_sprd_led(led_cdev) \
	container_of(led_cdev, struct sprd_flash_led, cdev)

static void Flash_SetBrightness( unsigned long  value)
{
 	if(value==0)
	__gpio_set_value(93,0);
	else
	__gpio_set_value(93,1);
	
}

static void sprd_led_enable(struct sprd_flash_led *led)
{
	printk("##################################################sprd_led_enable\n");
	Flash_SetBrightness(led->value);
	led->enabled = 1;
}

static void sprd_led_disable(struct sprd_flash_led *led)
{
	printk("##################################################sprd_led_disable\n");	
Flash_SetBrightness(led->value);
	led->enabled = 0;
}

static void led_work(struct work_struct *work)
{
	struct sprd_flash_led *led = container_of(work, struct sprd_flash_led, work);
	unsigned long flags;
printk("##################################################sled_work=%d\n",led->value);
	mutex_lock(&led->mutex);
	spin_lock_irqsave(&led->value_lock, flags);
	if (led->value == 0) {
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
	struct sprd_flash_led *led = to_sprd_led(led_cdev);
	unsigned long flags;
printk("##################################################sprd_led_set=%d\n",value);
	spin_lock_irqsave(&led->value_lock, flags);
	led->value = value;
	spin_unlock_irqrestore(&led->value_lock, flags);
	schedule_work(&led->work);	
}

static void sprd_flash_led_shutdown(struct platform_device *pdev)
{
	struct sprd_flash_led *led = platform_get_drvdata(pdev);
	mutex_lock(&led->mutex);
	led->value =0;
	led->enabled = 1;
	sprd_led_disable(led);
	mutex_unlock(&led->mutex);
}

static int sprd_flash_led_probe(struct platform_device *pdev)
{
	struct sprd_flash_led *led;
	int ret;
	


	
	printk("##################################################sprd_flash_led_probe111\n");
	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		ret = -ENOMEM;
		goto err_led;
	}

	led->cdev.brightness_set = sprd_led_set;
	led->cdev.default_trigger = "heartbeat";
	led->cdev.name = "flash-led";
	led->cdev.brightness_get = NULL;
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;
	led->enabled = 0;
	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, led_work);
	led->value = 0;
	platform_set_drvdata(pdev, led);
	printk("##################################################sprd_flash_led_probe22222\n");
	ret = led_classdev_register(&pdev->dev, &led->cdev);
	
	if (ret < 0)
		goto err_led;
		
	
	led->enabled = 0;
	schedule_work(&led->work);
	return 0;

 err_led:
	kfree(led);
	return ret;

}

static int sprd_flash_led_remove(struct platform_device *pdev)
{
	struct sprd_flash_led *led = platform_get_drvdata(pdev);
	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->value = 0;
	led->enabled = 1;
	sprd_led_disable(led);
	kfree(led);

	return 0;
}

static struct platform_driver sprd_flash_led_driver = {
	.driver = {
			.name = "flash-led",
			.owner = THIS_MODULE,
		   },
	.probe = sprd_flash_led_probe,
	.remove = sprd_flash_led_remove,
	.shutdown = sprd_flash_led_shutdown,
};

static int __devinit sprd_flash_led_init(void)
{
	printk("##################################################sprd_flash_led_driver\n");
	return platform_driver_register(&sprd_flash_led_driver);
}

static void sprd_flash_led_exit(void)
{
	platform_driver_unregister(&sprd_flash_led_driver);
}
module_init(sprd_flash_led_init);
module_exit(sprd_flash_led_exit);

MODULE_DESCRIPTION("Sprd SC8800G flash led driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:flashled");



