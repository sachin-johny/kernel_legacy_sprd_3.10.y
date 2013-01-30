/*
 * Copyright (C) 2011 Google, Inc.
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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <linux/clk.h>
#include <linux/err.h>

static struct rfkill *bt_rfk;
static const char bt_name[] = "bluetooth";

static  unsigned long bt_setpower;
struct clk *bt_clk;

static void getIoResource(struct platform_device  *pdev)
{
    struct resource *res;

    printk("rfkill get gpio\n");

    res = platform_get_resource_byname(pdev, IORESOURCE_IO,"bt_reset");
    if (!res) {
        printk("couldn't find bt_reset gpio\n");
    }

    bt_setpower = res->start;

    printk("bt_reset = %ld\n", bt_setpower);
}

static void bt_clk_init(bool clk_on )
{
    if(clk_on)
    {
        struct clk *clk_parent;
        bt_clk = clk_get(NULL, "clk_aux0");
        if (IS_ERR(bt_clk)) {
            printk("clock: failed to get clk_aux0\n");
        }

        clk_parent = clk_get(NULL, "ext_32k");
        if (IS_ERR(clk_parent)) {
            printk("failed to get parent ext_32k\n");
        }

        clk_set_parent(bt_clk, clk_parent);
        clk_set_rate(bt_clk, 32000);

        clk_enable(bt_clk);
    }
    else
        clk_disable(bt_clk);

}

static int bluetooth_set_power(void *data, bool blocked)
{
    printk(KERN_ERR "bluetooth_set_power blocked = %d\n", blocked);
    if (!blocked) {
        bt_clk_init(true);
        msleep(100);
        gpio_direction_output(bt_setpower, 0);
        msleep(100);
        gpio_direction_output(bt_setpower, 1);
        msleep(200);
    } else {
        gpio_direction_output(bt_setpower, 0);
        msleep(100);
        bt_clk_init(false);
    }

    return 0;
}

static struct rfkill_ops rfkill_bluetooth_ops = {
    .set_block = bluetooth_set_power,
};

static void rfkill_gpio_init(void)
{
    if(gpio_request(bt_setpower,"bt_reset")){
        printk("request bt reset fail\n");
    }
}

static void rfkill_gpio_deinit(void)
{
    gpio_free(bt_setpower);
}
static int rfkill_bluetooth_probe(struct platform_device *pdev)
{
    int rc = 0;
    bool default_state = true;

    printk(KERN_INFO "-->%s\n", __func__);
    getIoResource(pdev);

    bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
            &rfkill_bluetooth_ops, NULL);
    if (!bt_rfk) {
        rc = -ENOMEM;
        goto err_rfkill_alloc;
    }
    rfkill_gpio_init();
    /* userspace cannot take exclusive control */
    rfkill_init_sw_state( bt_rfk, 0 );
    rc = rfkill_register(bt_rfk);
    if (rc)
        goto err_rfkill_reg;

    rfkill_set_sw_state( bt_rfk, 1 );
    bluetooth_set_power(NULL, default_state);

    printk(KERN_INFO "<--%s\n", __func__);
    return 0;

err_rfkill_reg:
    rfkill_destroy(bt_rfk);
err_rfkill_alloc:
err_gpio_shutdown:
err_gpio_reset:
    return rc;
}

static int rfkill_bluetooth_remove(struct platform_device *dev)
{
    printk(KERN_INFO "-->%s\n", __func__);
    rfkill_gpio_deinit();
    rfkill_unregister(bt_rfk);
    rfkill_destroy(bt_rfk);
    printk(KERN_INFO "<--%s\n", __func__);
    return 0;
}

static struct platform_driver rfkill_bluetooth_driver = {
    .probe  = rfkill_bluetooth_probe,
    .remove = rfkill_bluetooth_remove,
    .driver = {
        .name = "rfkill",
        .owner = THIS_MODULE,
    },
};

static int __init rfkill_bluetooth_init(void)
{
    printk(KERN_INFO "-->%s\n", __func__);
    return platform_driver_register(&rfkill_bluetooth_driver);
}

static void __exit rfkill_bluetooth_exit(void)
{
    printk(KERN_INFO "-->%s\n", __func__);
    platform_driver_unregister(&rfkill_bluetooth_driver);
}

late_initcall(rfkill_bluetooth_init);
module_exit(rfkill_bluetooth_exit);
MODULE_DESCRIPTION("bluetooth rfkill");
MODULE_AUTHOR("Yale Wu <ye.wu@spreadtrum.com>");
MODULE_LICENSE("GPL");
