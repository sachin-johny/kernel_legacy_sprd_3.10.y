/*
 * File:         ltr_558als.c
 * Based on:
 * Author:       Liuxd <liuxiaodong@cellroam.com>
 *
 * Created:      2011-11-08
 * Description:  LTR-558ALS Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>
#include <linux/i2c/ltr_558als.h>
#include <linux/slab.h>

//#define LTR558_DBG
#ifdef LTR558_DBG
#define ENTER printk(KERN_INFO "[LTR558_DBG] func: %s  line: %04d  ", __func__, __LINE__)
#define PRINT_DBG(x...)  printk(KERN_INFO "[LTR558_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[LTR558_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[LTR558_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[LTR558_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[LTR558_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[LTR558_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[LTR558_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

typedef struct tag_ltr558 {
	struct input_dev *input;
	struct i2c_client *client;
	struct work_struct work;
	struct workqueue_struct *ltr_work_queue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ltr_early_suspend;
#endif
} ltr558_t, *ltr558_p;

static int p_flag;
static int l_flag;
static int p_gainrange;
static int l_gainrange;

static struct i2c_client *this_client = NULL;

static int ltr558_i2c_read_reg(u8 regnum)
{
	int readdata;

	if (!this_client)
		return -1;

	readdata = i2c_smbus_read_byte_data(this_client, regnum);
	return readdata;
}

static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
	int writeerror;

	if (!this_client)
		return -1;

	writeerror = i2c_smbus_write_byte_data(this_client, regnum, value);
	if (writeerror < 0)
		return writeerror;
	else
		return 0;
}

static int ltr558_ps_enable(int gainrange)
{
	int error;
	int setgain;

	PRINT_INFO("ltr558_ps_enable %d\n", gainrange);
	switch (gainrange) {
	case PS_RANGE1:
		setgain = MODE_PS_ON_Gain1;
		break;

	case PS_RANGE2:
		setgain = MODE_PS_ON_Gain4;
		break;

	case PS_RANGE4:
		setgain = MODE_PS_ON_Gain8;
		break;

	case PS_RANGE8:
		setgain = MODE_PS_ON_Gain16;
		break;

	default:
		setgain = MODE_PS_ON_Gain8;
		break;
	}

	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain);
	mdelay(WAKEUP_DELAY);

	/* ===============
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
	 * Not set and kept as device default for now.
	 */
	ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);

	return error;
}

// Put PS into Standby mode
static int ltr558_ps_disable(void)
{
	int error;

	PRINT_INFO("ltr558_ps_disable\n");
	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy);
	return error;
}

static int ltr558_ps_read(void)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0) {
		psdata = psval_lo;
		goto out;
	}

	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0) {
		psdata = psval_hi;
		goto out;
	}

	psdata = ((psval_hi & 0x07) << 8) | psval_lo;

out:
	return psdata;
}

static int ltr558_als_enable(int gainrange)
{
	int error;
	int setgain;

	PRINT_INFO("ltr558_als_enable %d\n", gainrange);

	switch (gainrange) {
	case ALS_RANGE1_320:
		setgain = MODE_ALS_ON_Range1;
		break;

	case ALS_RANGE2_64K:
		setgain = MODE_ALS_ON_Range2;
		break;

	default:
		setgain = MODE_ALS_ON_Range1;
		break;
	}

	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, setgain);
	mdelay(WAKEUP_DELAY);

	/* ===============
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
	 * Not set and kept as device default for now.
	 */
	ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);

	return error;
}

// Put ALS into Standby mode
static int ltr558_als_disable(void)
{
	int error;

	PRINT_INFO("ltr558_als_disable\n");
	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);
	return error;
}

static int ltr558_als_read(int gainrange)
{
	int alsval_ch0_lo, alsval_ch0_hi;
	int alsval_ch1_lo, alsval_ch1_hi;
	int luxdata_int;
	int luxdata_flt, ratio;
	int alsval_ch0, alsval_ch1;

	alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	PRINT_DBG("alsval_ch0[%d],  alsval_ch1[%d]\n", alsval_ch0, alsval_ch1);

	if (0 == alsval_ch0)
		ratio = 100;
	else
		ratio = (alsval_ch1 * 100) / alsval_ch0;

	// Compute Lux data from ALS data (ch0 and ch1)
	// For Ratio < 0.69:
	// 1.3618*CH0 – 1.5*CH1
	// For 0.69 <= Ratio < 1:
	// 0.57*CH0 – 0.345*CH1
	// For high gain, divide the calculated lux by 150.
	if (ratio < 69) {
		luxdata_flt = (13618 * alsval_ch0) - (15000 * alsval_ch1);
		luxdata_flt = luxdata_flt / 10000;
	} else if ((ratio >= 69) && (ratio < 100)) {
		luxdata_flt = (5700 * alsval_ch0) - (3450 * alsval_ch1);
		luxdata_flt = luxdata_flt / 10000;
	} else {
		luxdata_flt = 0;
	}

	// For Range1
	if (gainrange == ALS_RANGE1_320)
		luxdata_flt = luxdata_flt / 150;

	luxdata_int = luxdata_flt * 50;

	return luxdata_int;
}

static int ltr558_open(struct inode *inode, struct file *file)
{
	ENTER;
	return 0;
}

static int ltr558_release(struct inode *inode, struct file *file)
{
	ENTER;
	return 0;
}

static long ltr558_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;

	PRINT_INFO("cmd = %d,%d\n", _IOC_NR(cmd), cmd);
	switch (cmd) {
	case LTR_IOCTL_SET_PFLAG:
		{
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			PRINT_DBG("flag = %d\n", flag);
			if (1 == flag) {
				if (ltr558_ps_enable(p_gainrange))
					return -EIO;
			} else if (0 == flag) {
				if (ltr558_ps_disable())
					return -EIO;
			} else {
				return -EINVAL;
			}

			p_flag = flag;
		}
		break;

	case LTR_IOCTL_SET_LFLAG:
		{
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			PRINT_DBG("flag = %d\n", flag);
			if (1 == flag) {
				if (ltr558_als_enable(l_gainrange))
					return -EIO;
			} else if (0 == flag) {
				if (ltr558_als_disable())
					return -EIO;
			} else {
				return -EINVAL;
			}

			l_flag = flag;
		}
		break;

	case LTR_IOCTL_GET_PFLAG:
		{
			flag = p_flag;
			PRINT_DBG("LTR_IOCTL_GET_PFLAG, p_flag = %d\n", flag);
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
		}
		break;

	case LTR_IOCTL_GET_LFLAG:
		{
			flag = l_flag;
			PRINT_DBG("LTR_IOCTL_GET_LFLAG, l_flag = %d\n", flag);
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static struct file_operations ltr558_fops = {
	.owner = THIS_MODULE,
	.open = ltr558_open,
	.release = ltr558_release,
	.unlocked_ioctl = ltr558_ioctl,
};

static struct miscdevice ltr558_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = LTR558_I2C_NAME,
	.fops = &ltr558_fops,
};

static void ltr558_work(struct work_struct *work)
{
	int als_ps_status, val;
	ltr558_t *pls = container_of(work, ltr558_t, work);

	ENTER;
	als_ps_status = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	PRINT_DBG("als_ps_status=0x%02x\n", als_ps_status);

	if (0x03 == (als_ps_status & 0x03)) {
		val = ltr558_ps_read();
		PRINT_DBG("p -> val=0x%04x\n", val);

		if (val >= 0x49d) {	// 3cm
			ltr558_i2c_write_reg(0x90, 0xff);
			ltr558_i2c_write_reg(0x91, 0x07);
			ltr558_i2c_write_reg(0x92, 0x6d);
			ltr558_i2c_write_reg(0x93, 0x05);

			input_report_abs(pls->input, ABS_DISTANCE, 0);
			input_sync(pls->input);
		} else if (val <= 0x56d) {	// 5cm
			ltr558_i2c_write_reg(0x90, 0x9d);
			ltr558_i2c_write_reg(0x91, 0x04);
			ltr558_i2c_write_reg(0x92, 0x00);
			ltr558_i2c_write_reg(0x93, 0x00);

			input_report_abs(pls->input, ABS_DISTANCE, 1);
			input_sync(pls->input);
		}
	}

	if (0x0c == (als_ps_status & 0x0c)) {
		val = ltr558_als_read(l_gainrange);
		PRINT_DBG("l -> val=0x%04x\n", val);
		input_report_abs(pls->input, ABS_MISC, val);
		input_sync(pls->input);
	}

	enable_irq(pls->client->irq);
}

static irqreturn_t ltr558_irq_handler(int irq, void *dev_id)
{
	ltr558_t *pls = (ltr558_t *) dev_id;

	disable_irq_nosync(pls->client->irq);
	queue_work(pls->ltr_work_queue, &pls->work);

	return IRQ_HANDLED;
}

static int ltr558_sw_reset(void)
{
	ENTER;
	return ltr558_i2c_write_reg(LTR558_ALS_CONTR, 0x04);
}

static int ltr558_reg_init(void)
{
	int ret = 0;

	ENTER;
	//ltr558_sw_reset();
	//mdelay(PON_DELAY);

	ret += ltr558_i2c_write_reg(0x82, 0x7b);
	ret += ltr558_i2c_write_reg(0x83, 0x0f);
	ret += ltr558_i2c_write_reg(0x84, 0x00);
	ret += ltr558_i2c_write_reg(0x85, 0x03);
	ret += ltr558_i2c_write_reg(0x8f, 0x0B);
	ret += ltr558_i2c_write_reg(0x9e, 0x02);

	// ps
	ret += ltr558_i2c_write_reg(0x90, 0x01);
	ret += ltr558_i2c_write_reg(0x91, 0x00);
	ret += ltr558_i2c_write_reg(0x92, 0x00);
	ret += ltr558_i2c_write_reg(0x93, 0x00);

	// als
	ret += ltr558_i2c_write_reg(0x97, 0x00);
	ret += ltr558_i2c_write_reg(0x98, 0x00);
	ret += ltr558_i2c_write_reg(0x99, 0x01);
	ret += ltr558_i2c_write_reg(0x9a, 0x00);
	mdelay(WAKEUP_DELAY);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ltr558_early_suspend(struct early_suspend *handler)
{
	PRINT_INFO("early suspend, do nothing\n");
}

static void ltr558_late_resume(struct early_suspend *handler)
{
	PRINT_INFO("late resume, do nothing\n");
}
#endif

static int ltr558_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	ltr558_t *ltr_558als;
	struct ltr558_pls_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int id_revision = 0;
	int id_manufacturer = 0;

	ENTER;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PRINT_ERR("i2c_check_functionality error\n");
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ltr_558als = kzalloc(sizeof(ltr558_t), GFP_KERNEL);
	if (!ltr_558als) {
		PRINT_ERR("kzalloc error\n");
		ret = -ENOMEM;
		goto exit_request_memory_failed;
	}

	i2c_set_clientdata(client, ltr_558als);
	ltr_558als->client = client;
	this_client = client;

	id_revision = ltr558_i2c_read_reg(LTR558_REVISION_ID);
	id_manufacturer = ltr558_i2c_read_reg(LTR558_MANUFACTURER_ID);
	if(id_revision == 0x80 && id_manufacturer == 0x05)
		PRINT_INFO("I'm LTR558, and I'm working now\n");
	else if(id_revision < 0 || id_manufacturer < 0){
		PRINT_ERR("can't read who am I\n");
		goto exit_device_init_failed;
	} else {
		PRINT_ERR("I'm working, but I'm NOT LTR558\n");
		goto exit_device_init_failed;
	}

	p_gainrange = PS_RANGE4;
	l_gainrange = ALS_RANGE2_64K;
	if (ltr558_reg_init() < 0) {
		PRINT_ERR("init error\n");
		ret = -1;
		goto exit_device_init_failed;
	}

	ret = ltr558_ps_disable();
	if(ret<0)
		PRINT_ERR("disable ps error\n");
	ret = ltr558_als_disable();
	if(ret<0)
		PRINT_ERR("disable als error\n");
	p_flag = 0;
	l_flag = 0;

	ret = misc_register(&ltr558_device);
	if (ret) {
		PRINT_ERR("misc_register error ret = %d\n", ret);
		goto exit_misc_register_failed;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		PRINT_ERR("input_allocate_device error, ret = %d\n", ret);
		ret = -ENOMEM;
		goto exit_input_allocate_failed;
	}

	ltr_558als->input = input_dev;
	input_dev->name = LTR558_INPUT_DEV;
	input_dev->phys = LTR558_INPUT_DEV;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 100001, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		PRINT_ERR("input_register_device error, ret = %d\n", ret);
		goto exit_input_register_failed;
	}

	INIT_WORK(&ltr_558als->work, ltr558_work);
	ltr_558als->ltr_work_queue =
	    create_singlethread_workqueue(LTR558_I2C_NAME);
	if (!ltr_558als->ltr_work_queue) {
		PRINT_ERR("create_singlethread_workqueue error, ret = %d\n",  ret);
		goto exit_create_workqueue_failed;
	}

	gpio_request(pdata->irq_gpio_number, LTR558_PLS_IRQ_PIN);
	gpio_direction_input(pdata->irq_gpio_number);

	client->irq = gpio_to_irq(pdata->irq_gpio_number);
	PRINT_DBG("client->irq = %d\n", client->irq);

	if (client->irq > 0) {
		ret = request_irq(client->irq, ltr558_irq_handler,
				IRQ_TYPE_LEVEL_LOW, client->name, ltr_558als);
		if (ret < 0) {
			free_irq(client->irq, ltr_558als);
			client->irq = 0;
			PRINT_ERR("request_irq error, ret = %d\n", ret);
			goto exit_irq_request_err;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ltr_558als->ltr_early_suspend.level =
	    EARLY_SUSPEND_LEVEL_DISABLE_FB + 25;
	ltr_558als->ltr_early_suspend.suspend = ltr558_early_suspend;
	ltr_558als->ltr_early_suspend.resume = ltr558_late_resume;
	register_early_suspend(&ltr_558als->ltr_early_suspend);
#endif

	PRINT_INFO("probe success\n");
#ifdef LTR558_DBG
	ltr558_ps_enable(p_gainrange);
	ltr558_als_enable(l_gainrange);
	p_flag = 1;
	l_flag = 1;
#endif
	return 0;

exit_irq_request_err:
	destroy_workqueue(ltr_558als->ltr_work_queue);
	ltr_558als->ltr_work_queue = NULL;
exit_create_workqueue_failed:
	input_unregister_device(input_dev);
exit_input_register_failed:
	input_free_device(input_dev);
	input_dev = NULL;
exit_input_allocate_failed:
	misc_deregister(&ltr558_device);
exit_misc_register_failed:
exit_device_init_failed:
	kfree(ltr_558als);
	ltr_558als = NULL;
exit_request_memory_failed:
exit_check_functionality_failed:

	PRINT_ERR("probe failed, ret = %d\n", ret);
	return ret;
}

static int ltr558_remove(struct i2c_client *client)
{
	ltr558_t *ltr_558als = i2c_get_clientdata(client);

	ENTER;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ltr_558als->ltr_early_suspend);
#endif

	flush_workqueue(ltr_558als->ltr_work_queue);
	destroy_workqueue(ltr_558als->ltr_work_queue);
	ltr_558als->ltr_work_queue = NULL;
	input_unregister_device(ltr_558als->input);
	input_free_device(ltr_558als->input);
	ltr_558als->input = NULL;
	misc_deregister(&ltr558_device);
	free_irq(ltr_558als->client->irq, ltr_558als);
	kfree(ltr_558als);
	ltr_558als = NULL;
	this_client = NULL;

	return 0;
}

static const struct i2c_device_id ltr558_id[] = {
	{LTR558_I2C_NAME, 0},
	{}
};

static struct i2c_driver ltr558_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LTR558_I2C_NAME,
		   },
	.probe = ltr558_probe,
	.remove = ltr558_remove,
	.id_table = ltr558_id,
};

static int __init ltr558_init(void)
{
	ENTER;
	int ret = 0;
	ret = i2c_add_driver(&ltr558_driver);
	return ret;
}

static void __exit ltr558_exit(void)
{
	ENTER;
	ltr558_sw_reset();
	i2c_del_driver(&ltr558_driver);
}

late_initcall(ltr558_init);
module_exit(ltr558_exit);

MODULE_AUTHOR("Liuxd <liuxiaodong@cellroam.com>");
MODULE_DESCRIPTION("Proximity&Light Sensor LTR558ALS DRIVER");
MODULE_LICENSE("GPL");
