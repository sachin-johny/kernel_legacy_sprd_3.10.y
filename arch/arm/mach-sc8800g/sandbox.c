#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <mach/mfp.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#define GPIO_VIBRATOR	9

#ifdef 	CONFIG_SYSFS

#define	VIBON 	 -1
#define	VIBOFF 	 0

struct device *vibrator_dev;
unsigned long vib_gpio;

#define BACKLIGHT_GPIO 103
unsigned long bl_gpio;

#include <linux/delay.h>

static unsigned spi_cs1_gpio;
static inline void vibrator_set_status(long value)
{
	if (value == VIBOFF) {
		//gpio_set_value(vib_gpio, 0);
		pr_info("clear spi_cs1");
		gpio_set_value(spi_cs1_gpio, 0);
	}
	else if (value == VIBON) {
		//gpio_set_value(vib_gpio, 1);
		pr_info("set spi_cs1");
		gpio_set_value(spi_cs1_gpio, 1);
	}
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

#if 0
//static unsigned long vib_gpio_cfg = MFP_CFG_X(SD1_CLK, GPIO, DS0, PULL_NONE, IO_NONE);

//static unsigned long bl_gpio_cfg	= MFP_CFG_X(LCD_EN, GPIO, DS0, PULL_NONE, IO_OE);



static int bl_gpio_test(void)
{
	int err;

	sprd_mfp_config(&bl_gpio_cfg, 1);
	bl_gpio = mfp_to_gpio(MFP_CFG_TO_PIN(bl_gpio_cfg));
	pr_info("backlight gpio is:%d\r\n", bl_gpio);

	err = gpio_request(bl_gpio, "backlight");
	if (err) {
		pr_warning("cannot alloc gpio for backlight\r\n");
		return err;
	}
	gpio_direction_output(bl_gpio, 0);
	return 0;
}
#endif
static unsigned long spi_cs1_gpio_cfg = MFP_CFG_X(SPI_CSN0, GPIO, DS1, F_PULL_NONE, S_PULL_DOWN, IO_NONE);
static int spics1_gpio_test(void)
{
	int err;

	sprd_mfp_config(&spi_cs1_gpio_cfg, 1);

	spi_cs1_gpio = 32;
	pr_info("spi_cs1_gpio gpio is:%d\r\n", spi_cs1_gpio);

	err = gpio_request(spi_cs1_gpio, "spi_cs1");
	if (err) {
		pr_warning("cannot alloc gpio for spi cs1\r\n");
		return err;
	}
	gpio_direction_output(spi_cs1_gpio, 0);
	return 0;
}

static unsigned long pwr_gpio_cfg =
	MFP_ANA_CFG_X(PBINT, AF0, DS1, F_PULL_UP,S_PULL_UP, IO_IE);

static unsigned pwr_gpio = 163;

static irqreturn_t
power_button_handler(int irq, void *dev)
{
	int value;

//	sprd_ack_gpio_irq(irq);
//	sprd_mask_gpio_irq(irq);

//	disable_irq_nosync(irq);
	value = __gpio_get_value(pwr_gpio);
	pr_info("power button irq value: %x\n", value);
/*
	if (value)  {
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	} else {
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	}
*/
      msleep(1000);
//	enable_irq(irq);

	return IRQ_HANDLED;
}

static int pwr_gpio_int_test(void)
{
	int err;
	int irq;

	sprd_mfp_config(&pwr_gpio_cfg, 1);
	pwr_gpio = 163;//mfp_to_gpio(MFP_CFG_TO_PIN(bl_gpio_cfg));
	pr_info("power button gpio is:%d\r\n", pwr_gpio);

	err = gpio_request(pwr_gpio, "power button");
	if (err) {
		pr_warning("cannot alloc gpio for power button\r\n");
		return err;
	}
	gpio_direction_input(pwr_gpio);

	irq = sprd_alloc_gpio_irq(pwr_gpio);
	if (irq < 0)
		return -1;

	err = request_threaded_irq(irq,NULL, power_button_handler, IRQF_TRIGGER_LOW
		 | IRQF_ONESHOT, "power button irq", NULL);
		//| IRQF_NOAUTOEN | IRQF_ONESHOT, "headset irq", NULL);
	if (err) {
		pr_warning("cannot alloc irq for headset, err %d\r\n", err);
		return err;
	}
	#if 0
	msleep(5000);
	pr_info("re-enable headset irq again\n");
	enable_irq(IRQ_HEADSET);
	#endif
	return 0;
}

static unsigned long sdcard_detect_gpio_cfg =
        MFP_CFG_X(RFCTL11, AF3, DS1, F_PULL_UP,S_PULL_NONE, IO_Z);

static	int sd_gpio;
static irqreturn_t
sdcard_detect_handler(int irq, void *dev)
{
        int value;
	
	if(gpio_get_value(sd_gpio)){
		pr_info("sdcard plug out\r\n");
		set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		pr_info("sdcard plug in\r\n");
		set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}
	return IRQ_HANDLED;
}

static int sdcard_gpio_int_test(void)
{
        int err;
        int irq;

        sprd_mfp_config(&sdcard_detect_gpio_cfg, 1);
        sd_gpio = 101;//mfp_to_gpio(MFP_CFG_TO_PIN(bl_gpio_cfg));

        err = gpio_request(sd_gpio, "sdcard detect");
        if (err) {
                pr_warning("cannot alloc gpio for power button\r\n");
                return err;
        }
        gpio_direction_input(sd_gpio);
        pr_info("sdcard button gpio is:%d state%d\r\n", sd_gpio, gpio_get_value(sd_gpio));

        irq = sprd_alloc_gpio_irq(sd_gpio);
        if (irq < 0)
                return -1;

        err = request_threaded_irq(irq,NULL, sdcard_detect_handler, IRQF_TRIGGER_LOW
                 | IRQF_ONESHOT, "sdcard detect irq", NULL);
	if (err) {
		pr_warning("cannot alloc irq for sdcard detect, err %d\r\n", err);
		return err;
	}
	return 0;
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
