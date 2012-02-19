/*
 * =====================================================================================
 *
 *       Filename:  wifi_dummy.c
 *
 *    Description:  wifi dummy
 *
 *        Version:  1.0
 *        Created:  02/19/2012 04:39:30 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Binary Yang <Binary.Yang@spreadtrum.com.cn>
 *        Company:  © Copyright 2010 Spreadtrum Communications Inc.
 *
 * =====================================================================================
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>

extern int sprd_3rdparty_gpio_wifi_reset;
extern int sprd_3rdparty_gpio_wifi_pwd;

static int __init wifi_dummy_init(void)
{
	msleep(10000);

	gpio_direction_output(sprd_3rdparty_gpio_wifi_reset, 0);
        gpio_set_value(sprd_3rdparty_gpio_wifi_reset,0);
        gpio_direction_output(sprd_3rdparty_gpio_wifi_pwd,0);
        gpio_set_value(sprd_3rdparty_gpio_wifi_pwd,0);

	printk("=================wifi dummy========================\n");

	return 0;
}

late_initcall(wifi_dummy_init); /* try to load after dma driver when built-in */
