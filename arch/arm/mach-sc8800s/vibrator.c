#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <mach/mfp.h>

#define GPIO_VIBRATOR	9

#ifdef 	CONFIG_SYSFS 

#define	VIBON 	 -1
#define	VIBOFF 	 0

struct device *vibrator_dev;
unsigned long vib_gpio;

static inline void vibrator_set_status(long value)
{
	if (value == VIBOFF)
	      gpio_set_value(vib_gpio, 0);
	else if (value == VIBON)
	      gpio_set_value(vib_gpio, 1);
	else 
	      printk("Error getting data");
}

static ssize_t vibrator_status_store(struct device *dev,
			struct device_attribute *attr, 
			const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	vibrator_set_status(value);
	return size;
}

static DEVICE_ATTR(enable, 0644, NULL, vibrator_status_store);

struct class output_class = {
	.name		= "timed_output",
};

static unsigned long vib_gpio_cfg = MFP_CFG_X(SD1_CLK, GPIO, DS0, PULL_NONE,
					 IO_Z);

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
	sprd_mfp_config(&vib_gpio_cfg, 1);
	vib_gpio = mfp_to_gpio(MFP_CFG_TO_PIN(vib_gpio_cfg));
	pr_info("vibrator gpio is:%d\r\n", vib_gpio);
	
	err = gpio_request(vib_gpio, "vibrator");
	if (err) {
		pr_warning("cannot alloc gpio for vibrator\r\n");
		return err;
	}
	gpio_direction_output(vib_gpio, 0);
	return 0;
}

static void remove_vibrator_sysfs_file(void)
{
	device_remove_file(vibrator_dev, &dev_attr_enable);
	device_unregister(vibrator_dev);
	class_unregister(&output_class);
}

#endif

static int __init vibrator_sprd_init(void)
{
	return creat_vibrator_sysfs_file();
}

static void __exit vibrator_sprd_exit(void)
{
	remove_vibrator_sysfs_file();
}

module_init(vibrator_sprd_init);
module_exit(vibrator_sprd_exit);
