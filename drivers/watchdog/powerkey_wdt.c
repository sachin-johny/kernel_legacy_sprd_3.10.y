/*
 * Copyright (C) 2012 Spreadtrum
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/jiffies.h>

#define POWERKEY_RETRY_TIMES 1

static int powerkey_wdt_enable = 0;	/* default disabled */
static int powerkey_wdt_triggered = 0;	/* default not trigger */
static int powerkey_wdt_margin = 3;	/* seconds */
static int powerkey_wdt_click = 1;	/* 0.5 seconds */
static int powerkey_retry = 0;

static void powerkey_wdt_check(unsigned long data)
{
	printk(KERN_INFO "%s(): %d, %d\n",
		__func__, powerkey_wdt_enable, powerkey_wdt_triggered);

	/* can't get powerkey up event in "powerkey_wdt_click" seconds,
	 * omit this watchdog trigger */
	powerkey_wdt_triggered = 0;

	return;
}

static void powerkey_wdt_fire(unsigned long data)
{
	printk(KERN_INFO "%s(): %d, %d, %d\n",
		__func__, powerkey_wdt_enable, powerkey_wdt_triggered, powerkey_retry);

	/* we expect to see more info by doing panic */
	if(powerkey_wdt_triggered) {
		if(powerkey_retry++ >= POWERKEY_RETRY_TIMES)
			panic("PowerKey SoftDog: Timeout!!!\n");
	}

	return;
}

static struct timer_list wdt_watchdog_timer =
		TIMER_INITIALIZER(powerkey_wdt_fire, 0, 0);

static struct timer_list wdt_verify_timer =
		TIMER_INITIALIZER(powerkey_wdt_check, 0, 0);

int powerkey_wdt_verify(void)
{
	printk(KERN_INFO "%s(): %d\n", __func__, powerkey_wdt_enable);

	if (powerkey_wdt_enable)
		del_timer(&wdt_verify_timer);

	return 0;
}

int powerkey_wdt_start(void)
{
	printk(KERN_INFO "%s(): %d\n", __func__, powerkey_wdt_enable);

	/* trigger a watchdog.
	 * if has triggered, return directly, otherwise, watchdog reboot will denied by repeat click */
	if (powerkey_wdt_enable && !powerkey_wdt_triggered) {
		powerkey_wdt_triggered = 1;
		mod_timer(&wdt_verify_timer, jiffies + (powerkey_wdt_click * HZ / 2));
		mod_timer(&wdt_watchdog_timer, jiffies + (powerkey_wdt_margin * HZ));
	}

	return 0;
}

int powerkey_wdt_stop(void)
{
	printk(KERN_INFO "%s(): %d\n", __func__, powerkey_wdt_enable);

	/* In factory test mode, watchdog disabled because we never get here */
	powerkey_wdt_enable = 1;

	/* disable trigger */
	powerkey_retry = 0;
	powerkey_wdt_triggered = 0;
	del_timer(&wdt_watchdog_timer);
	del_timer(&wdt_verify_timer);

	return 0;
}

static int __init powerkey_wdt_init(void)
{
	return 0;
}

static void __exit powerkey_wdt_exit(void)
{
}

module_init(powerkey_wdt_init);
module_exit(powerkey_wdt_exit);

MODULE_LICENSE("GPL");
