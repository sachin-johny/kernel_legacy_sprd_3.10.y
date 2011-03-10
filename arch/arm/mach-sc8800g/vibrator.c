/* include/asm/mach-sprd/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>

#define VIBRATOR_LEVEL	(3)

static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;

static void set_vibrator(int on)
{
    if(on == 0){
        ANA_REG_AND(VIBR_CTL, ~(VIBR_PD_SET | VIBR_PD_RST));
        ANA_REG_OR(VIBR_CTL, VIBR_PD_SET);
    }else{
        ANA_REG_AND(VIBR_CTL, ~(VIBR_PD_SET | VIBR_PD_RST));
        ANA_REG_OR(VIBR_CTL, (VIBRATOR_LEVEL << VIBR_V_SHIFT) & VIBR_V_MSK);
        ANA_REG_OR(VIBR_CTL, VIBR_PD_RST);
    }
}

static void update_vibrator(struct work_struct *work)
{
	set_vibrator(vibe_state);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long	flags;

	spin_lock_irqsave(&vibe_lock, flags);
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		vibe_state = 0;
	else {
		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);

	schedule_work(&vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	schedule_work(&vibrator_work);
	return HRTIMER_NORESTART;
}

static irqreturn_t
share_irq_handler(int irq, void *dev)
{
     pr_info("share irq, nothing to do\n");
     return IRQ_HANDLED;
}

void share_irq_test(void)
{
     int err;
     err = request_irq(26, share_irq_handler, 0, "share irq", NULL);
}
static int creat_vibrator_sysfs_file(void)
{
	int err;


	err = class_register(&output_class);
	if (err)
	{
		printk(KERN_ERR "timed_output: unable to register timed_output class\n");
		return err;
	}

	vibrator_dev = device_create(&output_class, NULL, 0, NULL, "%s", "vibrator");
	err = device_create_file(vibrator_dev, &dev_attr_enable);
	if (err) {
		device_unregister(vibrator_dev);
		return err;
	}
#if 0
	sprd_mfp_config(&vib_gpio_cfg, 1);
	vib_gpio = mfp_to_gpio(MFP_CFG_TO_PIN(vib_gpio_cfg));
	pr_info("vibrator gpio is:%d\r\n", vib_gpio);

	err = gpio_request(vib_gpio, "vibrator");
	if (err) {
		pr_warning("cannot alloc gpio for vibrator\r\n");
		return err;
	}
	gpio_direction_output(vib_gpio, 0);
	//bl_gpio_test();
#endif
	pwr_gpio_int_test();
	spics1_gpio_test();
    share_irq_test();
//	sdcard_gpio_int_test();
	return 0;
}

void __init sprd_init_vibrator(void)
{
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&sprd_vibrator);
}

module_init(sprd_init_vibrator);
MODULE_DESCRIPTION("sprd timed output vibrator device");
MODULE_LICENSE("GPL");

