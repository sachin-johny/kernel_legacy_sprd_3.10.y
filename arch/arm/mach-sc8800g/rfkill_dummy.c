#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>

static int rfkill_set_power(void *data, bool blocked);
static struct platform_device *rfkill_pdev;
static struct rfkill *rfkill;  /* for driver only */
static struct rfkill_ops rfkill_ops = {
    .set_block = rfkill_set_power,
};

static int rfkill_set_power(void *data, bool blocked)
{
//    printk("rfkill to %d\n", blocked);
	return 0;
}

static int __init rfkill_init(void)
{
    int ret = 0;

    rfkill_pdev = platform_device_alloc("rfkill-dummy", -1);
    if (unlikely(!rfkill_pdev))
        return -ENOMEM;
    ret = platform_device_add(rfkill_pdev);
    if (ret) {
        platform_device_put(rfkill_pdev);
        return ret;
    }

    rfkill = rfkill_alloc(rfkill_pdev->name, &rfkill_pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &rfkill_ops, NULL);

    if (unlikely(!rfkill))
        return -ENOMEM;

	ret = rfkill_register(rfkill);

	if (unlikely(ret)) {
		rfkill_destroy(rfkill);
    }

    return ret;
}

static void __exit rfkill_exit(void)
{
    if (likely(rfkill_pdev))
        platform_device_unregister(rfkill_pdev);

    if (likely(rfkill)) {
        rfkill_unregister(rfkill);
        rfkill_destroy(rfkill);
    }
}

late_initcall(rfkill_init);
module_exit(rfkill_exit);

MODULE_DESCRIPTION("rfkill-dummy driver");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
