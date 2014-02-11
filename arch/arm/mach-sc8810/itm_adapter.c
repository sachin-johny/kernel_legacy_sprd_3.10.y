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

#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>

#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/mmc/host.h>

/*Since trout must be initialized as SDIO mode, PIN configuration is needed*/
#include <mach/pinmap.h>

#define SPRD_GPIO_REG_WRITEL(val, reg)	do{__raw_writel(val, CTL_PIN_BASE + (reg));}while(0)
/*
 * GPIO_XXX_REG are defined for sp6825gd
 * when porting driver to another board,
 * these macros must be taken care of
 */

#define GPIO_WIFI_RESET_REG	REG_PIN_GPIO140
#define GPIO_WIFI_POWERON_REG	REG_PIN_XTL_EN
struct regulator *wlan_regulator_18;

/*trout_chip_reset is used to RESET trout and initialize trout's interface as SDIO mode*/
static void trout_chip_reset(void)
{
	int ret;
	unsigned long value_gpio_reset = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_SLP_OE);
	unsigned long value_gpio_wifi_d3 = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_WPU | BIT_PIN_SLP_WPU | BIT_PIN_SLP_OE);
	unsigned long value_gpio_sd3 = (BITS_PIN_DS(1) | BITS_PIN_AF(0) | BIT_PIN_WPU);


	/*Config GPIO_SD1_D3 as gpio*/
	SPRD_GPIO_REG_WRITEL(value_gpio_wifi_d3, REG_PIN_SD1_D3);
	pr_info("Entered %s\n", __func__);

	ret = gpio_request(GPIO_SD1_D3, "hello");

	if (ret)
	{
		pr_err("Requesting for GPIO function of PIN SD1_D3 failed!!!!");
	}

	gpio_direction_output(GPIO_SD1_D3, 1);
	mdelay(1);

	/*Config pin function to gpio for Trout Reset Pin*/
	SPRD_GPIO_REG_WRITEL(value_gpio_reset, GPIO_WIFI_RESET_REG);
	ret = gpio_request(GPIO_WIFI_RESET, "trout_reset");

	if (ret)
	{
		pr_err("Requesting for GPIO83(SD1_D3)failed!!!!");
	}
	gpio_direction_output(GPIO_WIFI_RESET, 1);
	gpio_set_value(GPIO_WIFI_RESET, 0);
	mdelay(100);
	gpio_set_value(GPIO_WIFI_RESET, 1);

	gpio_free(GPIO_SD1_D3);
	gpio_free(GPIO_WIFI_RESET);
	/*Config pin function to SD1_D3*/
	SPRD_GPIO_REG_WRITEL(value_gpio_sd3, REG_PIN_SD1_D3);
	mdelay(1);

}

static int wlan_ldo_enable(void)
{
	int err;

	wlan_regulator_18 = regulator_get(NULL, REGU_NAME_SDHOST1);

	if (IS_ERR(wlan_regulator_18)) {
        printk("can't get wlan 1.8V regulator in itm \n");
		pr_err("can't get wlan 1.8V regulator\n");
		return -1;
	}

	pr_info("%s: Setting wlan voltage to 1.8V\n", __func__);
	err = regulator_set_voltage(wlan_regulator_18,1800000,1800000);
	if (err){
		pr_err("can't set wlan to 1.8V.\n");
		return -1;
	}
	regulator_set_mode(wlan_regulator_18, REGULATOR_MODE_STANDBY);
	regulator_enable(wlan_regulator_18);
    printk("[wlan_ldo_enable in itm ] result %d \n",regulator_is_enabled(wlan_regulator_18));
}

static void wlan_clk_init(void)
{
	struct clk *wlan_clk;
	struct clk *clk_parent;

	wlan_clk = clk_get(NULL, "clk_aux0");
	if (IS_ERR(wlan_clk)) {
		pr_info("clock: failed to get clk_aux0\n");
	}
	clk_parent = clk_get(NULL, "ext_32k");
	if (IS_ERR(clk_parent)) {
		pr_info("failed to get parent ext_32k\n");
	}

	clk_set_parent(wlan_clk, clk_parent);
	clk_set_rate(wlan_clk, 32000);
	clk_enable(wlan_clk);
	pr_info("%s:done!!", __func__);
}


static void wlan_pdn_enable(void)
{
	int ret;
	unsigned int value_gpio_pdn = (BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_SLP_OE);

	/*Set gpio function to TRACEDAT3*/
	SPRD_GPIO_REG_WRITEL(value_gpio_pdn, GPIO_WIFI_POWERON_REG);

	ret = gpio_request(GPIO_WIFI_POWERON, "Trout powerdown");

	if (ret)
	{
		pr_err("Requet for gpio pin (GPIO_WIFI_POWERON)failed\n");
	}
	else
	{
		pr_info("Request for gpio pin(GPIO_WIFI_POWERON) ok\n");
	}

	gpio_direction_output(GPIO_WIFI_POWERON,  1);
	gpio_set_value(GPIO_WIFI_POWERON, 1);
	gpio_free(GPIO_WIFI_POWERON);
}

static int __init wlan_device_init(void)
{
	int ret = 0;
	wlan_pdn_enable();
	wlan_ldo_enable();
	wlan_clk_init();

	return ret;
}

late_initcall(wlan_device_init);

MODULE_DESCRIPTION("Ittiam wlan driver");
MODULE_LICENSE("GPL");

