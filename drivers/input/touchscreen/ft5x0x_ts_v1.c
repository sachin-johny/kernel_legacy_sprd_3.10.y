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

 
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/ft5x0x_ts.h>
#include <mach/regulator.h>

#include <mach/hardware.h>
#include<mach/globalregs.h>

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/clk.h>
/*********************************Bee-0928-TOP****************************************/

static unsigned char debug_level=PIXCIR_DEBUG;

#define CONFIG_FT5X0X_MULTITOUCH

#define PIXCIR_DBG(format, ...)	\
	if(debug_level == 1)	\
		printk(KERN_INFO "PIXCIR " format "\n", ## __VA_ARGS__)
		
static struct i2c_client *this_client;
static struct ft5x0x_ts_struct *g_ft5x0x_ts;
static unsigned char status_reg = 0;
static struct point_node_t point_slot[MAX_FINGER_NUM*2];
static struct point_node_t point_slot_back[MAX_FINGER_NUM*2];
static int distance[5]={0};
static int touch_flage[5]={0};
static struct i2c_driver ft5x0x_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);


static ssize_t ft5x0x_set_calibrate(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t ft5x0x_show_suspend(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);


static int ft5x0x_read_data(struct ft5x0x_ts_struct *data);
static void ft5x0x_report_value(struct ft5x0x_ts_struct *data);

static void ft5x0x_reset(int reset_pin);
static void ft5x0x_ts_suspend(struct early_suspend *handler);
static void ft5x0x_ts_resume(struct early_suspend *handler);
static void ft5x0x_ts_pwron(struct regulator *reg_vdd);
static void ft5x0x_ts_pwroff(struct regulator *reg_vdd);
static int ft5x0x_tx_config(void);
//static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR, NULL, pixcir_set_calibrate);
//static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, pixcir_show_suspend, pixcir_store_suspend);
//static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, pixcir_show_debug, pixcir_store_debug);



/* pixcir_i2c_rxdata --  read data from i2c
 * @rxdata: read buffer, first output variable
 * @length: read length, second input variable
 * @return: eror value, 0 means successful
 */
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
        int ret;
        struct i2c_msg msgs[] = {
                {
                        .addr   = g_ft5x0x_ts->client->addr,
                        .flags  = 0,
                        .len    = 1,
                        .buf    = rxdata,
                },
                {
                        .addr   = g_ft5x0x_ts->client->addr,
                        .flags  = I2C_M_RD,
                        .len    = length,
                        .buf    = rxdata,
                },
        };

        ret = i2c_transfer(g_ft5x0x_ts->client->adapter, msgs,2);
        if (ret < 0)
                pr_err("%s i2c read error: %d\n", __func__, ret);

        return ret;
}

/* pixcir_i2c_txdata --  write data to i2c
 * @rxdata: write buffer, first input variable
 * @length: length length, second input variable
 * @return: eror value, 0 means successful
 */
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
		int ret;
		struct i2c_msg msg[] = {
			{
				.addr	= g_ft5x0x_ts->client->addr,
				.flags	= 0,
				.len		= length,
				.buf		= txdata,
			},
		};

		ret = i2c_transfer(g_ft5x0x_ts->client->adapter, msg, 1);
		if (ret < 0)
			pr_err("%s i2c write error: %d\n", __func__, ret);

		return ret;
}


/* pixcir_i2c_write_data --  write one byte to i2c
 * @rxdata: the register address of the i2c device , first input variable
 * @length: the data to write , second input variable
 * @return: eror value, 0 means successful
 */
static int ft5x0x_i2c_write_data(unsigned char addr, unsigned char data)
{
	unsigned char buf[2];
	buf[0]=addr;
	buf[1]=data;
	return ft5x0x_i2c_txdata(buf, 2);
}

static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR, NULL, ft5x0x_set_calibrate);
static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, ft5x0x_show_suspend, ft5x0x_store_suspend);

static ssize_t ft5x0x_set_calibrate(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);

	if(on_off==1)
	{
		PIXCIR_DBG("%s: PIXCIR calibrate\n",__func__);
		ft5x0x_i2c_write_data(0x3a , 0x03);
		msleep(5*1000);
	}

	return len;
}


/* pixcir_show_suspend --  for suspend/resume debug
 *                         show current status
 * params:	no care
 * @return: len
 */
static ssize_t ft5x0x_show_suspend(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(g_ft5x0x_ts->suspend_flag==1)
		sprintf(buf, "Pixcir Suspend\n");
	else
		sprintf(buf, "Pixcir Resume\n");

	ret = strlen(buf) + 1;

	return ret;
}


/* pixcir_store_suspend -- for suspend/resume debug
 *                         set suspend/resume
 * params:	no care
 * @return: len
 */
static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	g_ft5x0x_ts->suspend_flag= on_off;

	if(on_off==1)
	{
		PIXCIR_DBG(KERN_INFO "Pixcir Entry Suspend\n");
		ft5x0x_ts_suspend(NULL);
	}
	else
	{
		PIXCIR_DBG(KERN_INFO "Pixcir Entry Resume\n");
		ft5x0x_ts_resume(NULL);
	}

	return len;
}


/* pixcir_create_sysfs --  create sysfs attribute
 * client:	i2c client
 * @return: len
 */
static int ft5x0x_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	PIXCIR_DBG("%s", __func__);

	err = device_create_file(dev, &dev_attr_calibrate);
	err = device_create_file(dev, &dev_attr_suspend);

	return err;
}

/* pixcir_ts_sleep --  set pixicir into sleep mode
 * @return: none
 */
static void ft5x0x_ts_sleep(void)
{
	PIXCIR_DBG(KERN_INFO "==%s==\n", __func__);
    //pixcir_i2c_write_data(PIXCIR_PWR_MODE_REG , PIXCIR_PWR_SLEEP_MODE);
}

/* pixcir_ts_suspend --  set pixicr into suspend
 * @return: none
 */
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
   	ft5x0x_ts_pwroff(g_ft5x0x_ts->reg_vdd);
	disable_irq_nosync(g_ft5x0x_ts->ft5x0x_irq);
}

/* pixcir_ts_resume --  set pixicr to resume
 * @return: none
 */
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	ft5x0x_ts_pwron(g_ft5x0x_ts->reg_vdd);
	gpio_direction_input(g_ft5x0x_ts->platform_data->irq_gpio_number);
	ft5x0x_reset(g_ft5x0x_ts->platform_data->reset_gpio_number);
	msleep(100);
	//pixcir_tx_config();
	enable_irq(g_ft5x0x_ts->ft5x0x_irq);
}


#ifdef TOUCH_VIRTUAL_KEYS
#define SC8810_KEY_HOME	102
#define SC8810_KEY_MENU	30
#define SC8810_KEY_BACK	17
#define SC8810_KEY_SEARCH  217

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	     __stringify(EV_KEY) ":" __stringify(KEY_HOME)     ":120:500:50:100"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":40:500:50:100"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":200:500:50:100"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":280:500:50:100"
	 "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.ft5x0x_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};


/* pixcir_ts_virtual_keys_init --  register virutal keys to system
 * @return: none
 */
static void ft5x0x_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;
	
    PIXCIR_DBG("%s\n",__func__);
	
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}


#endif


/* pxicir_ts_pininit --  gpio request
 * irq_pin:	irq gpio number
 * rst_pin: reset gpio number
 * @return: none
 */
static void pxicir_ts_pininit(int irq_pin, int rst_pin)
{
	gpio_request(irq_pin, TS_IRQ_PIN);
	gpio_request(rst_pin, TS_RESET_PIN);
	gpio_direction_input(irq_pin);
}

/* pixcir_ts_pwron --  power on pixcir chip
 * reg_vdd:	regulator
 * @return: none
 */
struct regulator *tsp_regulator_33 = NULL;
static void ft5x0x_ts_pwroff(struct regulator *reg_vdd)
{
	PIXCIR_DBG(KERN_INFO "%s\n",__func__);
	regulator_disable(tsp_regulator_33);
	msleep(20);
}
static void ft5x0x_ts_pwron(struct regulator *reg_vdd)
{
	int err = 0;
	PIXCIR_DBG(KERN_INFO "%s\n",__func__);
	tsp_regulator_33 = regulator_get(NULL, REGU_NAME_TP);
	if (IS_ERR(tsp_regulator_33)) {
		pr_err("zinitix:could not get 3.3v regulator\n");
		return -1;
	}
	err =regulator_set_voltage(tsp_regulator_33,3000000,3000000);
	if (err)
		pr_err("zinitix:could not set to 3300mv.\n");
	//regulator_set_voltage(reg_vdd, 3000000, 3000000);
	regulator_enable(tsp_regulator_33);
	//__raw_writel(((__raw_readl(0x82000628) &0xfffff0ff)|(0x00000900)), 0x82000628);
	
	msleep(20);
}

/* attb_read_val --  read the interrupt pin level
 * gpio_pin: pin number
 * @return: 1 or 0
 */
static int attb_read_val(int gpio_pin)
{
	return gpio_get_value(gpio_pin);
}

/* pixcir_reset --  set pixcir reset
 * reset_pin: pin number
 * @return: none
 */
static void ft5x0x_reset(int reset_pin)
{
	PIXCIR_DBG(KERN_INFO "%s\n",__func__);
	#if 0
	gpio_direction_output(reset_pin, 0);
	msleep(3);
	gpio_set_value(reset_pin, 1);
	msleep(10);
	gpio_set_value(reset_pin,0);
	msleep(10);
	#endif
}

/* pixcir_tx_config --  initialize the pixcir register
 * @return: error code
 */
static int ft5x0x_tx_config(void)
{
	int error;
	unsigned char buf;
#if 0
	error=pixcir_i2c_write_data(PIXCIR_INT_MODE_REG, PIXCIR_INT_MODE);
	buf =  PIXCIR_INT_MODE_REG;
	error=pixcir_i2c_rxdata(&buf, 1);
	PIXCIR_DBG("%s: buf=0x%x",__func__, buf);
	return error;
#endif
	return 0;
}


static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
	found: spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "%s: i2c-dev: Out of device minors\n",__func__);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);

	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}
/*********************************Bee-0928-bottom**************************************/


/* pixcir_ts_poscheck -- check it finger on TP and it's position,
 *                       report event to input system
 * pixcir_ts_struct: pixicir data struct
 * @return: none
 */
static void ft5x0x_ts_poscheck(struct ft5x0x_ts_struct *data)
{
	int ret = 0;
	struct ft5x0x_ts_struct *tsdata = data;
	PIXCIR_DBG("%s\n", __func__);
	ret = ft5x0x_read_data(tsdata);	
	if (ret == 0) {	
	        ft5x0x_report_value(tsdata);
		}
}
static void ft5x0x_ts_release(struct ft5x0x_ts_struct *data)
{
#ifdef CONFIG_FT5X0X_MULTITOUCH	
	input_report_key(data->input, BTN_TOUCH, 0);
	input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(data->input, ABS_PRESSURE, 0);  
	input_report_key(data->input, BTN_TOUCH, 0);
#endif
	input_sync(data->input);
}
static int ft5x0x_read_data(struct ft5x0x_ts_struct *data)
{
	struct ts_event *event = &data->event;
	u8 buf[32] = {0};
	int ret = -1;
	int touch_point = 0;

	/*printk("==read data=\n");	*/
#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(buf, 31);
#else
    ret = ft5x0x_i2c_rxdata(buf, 7);
#endif
    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
#if 0
	event->touch_point = buf[2] & 0x03;/* 0000 0011*/
#endif
	touch_point = buf[2] & 0x07;// 000 0111
     
    if (touch_point == 0) {
        ft5x0x_ts_release(data);
        return 1; 
    }
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = touch_point;
#ifdef CONFIG_FT5X0X_MULTITOUCH
    switch (event->touch_point) {
#ifdef CONFIG_TOUCHSCREEN_FT5206_WVGA
//jixwei_add for Q100 ft5x06 convert from WVGA to HVGA
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			event->x5 = event->x5*2/3;
			event->y5 = event->y5*3/5;
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			event->x4 = event->x4*2/3;
			event->y4 = event->y4*3/5;
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			event->x3 = event->x3*2/3;
			event->y3 = event->y3*3/5;
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			event->x2 = event->x2*2/3;
			event->y2 = event->y2*3/5;
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			event->x1 = event->x1*2/3;
			event->y1 = event->y1*3/5;
#else
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
#endif
			break;
		default:
		    return -1;
	}
#else
    if (event->touch_point == 1) {
    	event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
    }
#endif
    event->pressure = 200;

	PIXCIR_DBG("%d (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);

    return 0;
}
static void ft5x0x_report_value(struct ft5x0x_ts_struct *data)
{
	struct ts_event *event = &data->event;
        /*lilonghui delet the log for debug2011-12-26*/
	PIXCIR_DBG("==ft5x0x_report_value =\n");
#ifdef CONFIG_FT5X0X_MULTITOUCH
			input_report_key(data->input, BTN_TOUCH, 1);	
	switch(event->touch_point) {
		case 5:
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input);
			PIXCIR_DBG("===x5 = %d,y5 = %d ====\n",event->x5,event->y5);
		case 4:
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input);
			PIXCIR_DBG("===x4 = %d,y4 = %d ====\n",event->x4,event->y4);
		case 3:
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input);
			PIXCIR_DBG("===x3 = %d,y3 = %d ====\n",event->x3,event->y3);
		case 2:
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input);
			PIXCIR_DBG("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
		case 1:
			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, event->pressure);
	        input_report_abs(data->input, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input);
			PIXCIR_DBG("===x1 = %d,y1 = %d ====\n",event->x1,event->y1);
		default:
			PIXCIR_DBG("==touch_point default =\n");
			break;
	}
    
	input_report_abs(data->input, ABS_PRESSURE, event->pressure);
#if 0
	input_report_key(data->input, BTN_TOUCH, 1);
#endif
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input, ABS_X, event->x1);
		input_report_abs(data->input, ABS_Y, event->y1);
		input_report_abs(data->input, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input, BTN_TOUCH, 1);
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_sync(data->input);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}
/* pixcir_ts_isr -- disable irq, and schedule
 * irq: irq number
 * dev_id: param when request_irq
 * @return: IRQ status
 */
static irqreturn_t ft5x0x_ts_isr(int irq, void *dev_id)
{
	int ret = 0;
	struct ft5x0x_ts_struct *tsdata = (struct ft5x0x_ts_struct *)dev_id;
	PIXCIR_DBG("%s\n", __func__);
	/*disable irq*/
	disable_irq_nosync(irq);
#if 1
	if (!work_pending(&tsdata->pen_event_work)) {
		queue_work(tsdata->ts_workqueue, &tsdata->pen_event_work);
	}
#else
    {
		ret = ft5x0x_read_data(tsdata);	
		if (ret == 0) {	
			ft5x0x_report_value(tsdata);
		}
#if 0
		if (attb_read_val()) {
			PIXCIR_DBG("%s: release\n",__func__);
			input_report_key(tsdata->input, BTN_TOUCH, 0);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
			input_sync(tsdata->input);
			break;
		}
#endif
	}
	enable_irq(irq);
#endif	
	return IRQ_HANDLED;

}


/* pixcir_ts_irq_work -- read tp register and report event

 * @return: none
 */
 #if 1
static void ft5x0x_ts_irq_work(struct work_struct *work)
{
	struct ft5x0x_ts_struct *tsdata = g_ft5x0x_ts;
	ft5x0x_ts_poscheck(tsdata);
    enable_irq(tsdata->client->irq);
}
#endif
#ifdef CONFIG_PM_SLEEP
static int ft5x0x_i2c_ts_suspend(struct device *dev)
{
#if 1
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char wrbuf[2] = { 0 };
	int ret;

	//wrbuf[0] = 0x33;
	//wrbuf[1] = 0x03;	//enter into freeze mode;
	/**************************************************************
	wrbuf[1]:	0x00: Active mode
			0x01: Sleep mode
			0xA4: Sleep mode automatically switch
			0x03: Freeze mode
	More details see application note 710 power manangement section
	****************************************************************/
/*
	ret = i2c_master_send(client, wrbuf, 2);
	if(ret!=2) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed(), ret=%d\n",
			__func__, ret);
	}
*/
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
#endif
	return 0;
}

static int ft5x0x_i2c_ts_resume(struct device *dev)
{
#if 1
	struct i2c_client *client = to_i2c_client(dev);

	PIXCIR_DBG(KERN_INFO "%s\n",__func__);
	ft5x0x_reset(g_ft5x0x_ts->platform_data->reset_gpio_number);
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
#endif
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x0x_dev_pm_ops,
			 ft5x0x_i2c_ts_suspend, ft5x0x_i2c_ts_resume);


/* pixcir_i2c_ts_probe -- pixcir probe function
 * 						  initalize PIXCIR hardware configuration
 *                        register input system, and request irq
 * @return: error code; 0: successful
 */
static int __devinit ft5x0x_i2c_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	struct ft5x0x_ts_struct *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int i, error;

	PIXCIR_DBG(KERN_INFO "%s: probe\n",__func__);

	for(i=0; i<MAX_FINGER_NUM*2; i++) {
		point_slot[i].active = 0;
	}
	this_client = client;
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!tsdata || !input) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
	g_ft5x0x_ts = tsdata;

	PIXCIR_DBG("%s: irq_pin=%d; reset_pin=%d", \
		__func__, pdata->irq_gpio_number,pdata->reset_gpio_number);

	tsdata->platform_data = pdata;

	/*pin init*/
	pxicir_ts_pininit(pdata->irq_gpio_number,pdata->reset_gpio_number);
#if 0
	/*get regulator*/
	tsdata->reg_vdd = regulator_get(&client->dev, REGU_NAME_TP);
#endif
	/*enable VDD*/
	ft5x0x_ts_pwron(tsdata->reg_vdd);

	/*reset TP chip*/
	ft5x0x_reset(pdata->reset_gpio_number);
	msleep(100);

	//get irq number
	client->irq = gpio_to_irq(pdata->irq_gpio_number);
	tsdata->ft5x0x_irq = client->irq;
	PIXCIR_DBG("%s: irq=%d",__func__, client->irq);

	//register virtual keys
#ifdef TOUCH_VIRTUAL_KEYS
	ft5x0x_ts_virtual_keys_init();
#endif

	tsdata->client = client;
	tsdata->input = input;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	__set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	__set_bit(ABS_MT_POSITION_X, input->absbit);
	__set_bit(ABS_MT_POSITION_Y, input->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input->absbit);

	__set_bit(KEY_MENU,  input->keybit);
	__set_bit(KEY_BACK,  input->keybit);
	__set_bit(KEY_HOME,  input->keybit);
	__set_bit(KEY_SEARCH,  input->keybit);
	
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);


	input_set_drvdata(input, tsdata);
#if 1
	INIT_WORK(&tsdata->pen_event_work, ft5x0x_ts_irq_work);
	tsdata->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
#endif
	error = request_irq(client->irq, ft5x0x_ts_isr, IRQF_TRIGGER_FALLING, client->name, tsdata);
	if (error) {
		printk(KERN_ERR "%s:Unable to request touchscreen IRQ.\n",__func__);
		goto err_free_mem;
	}

	disable_irq_nosync(client->irq);

	error = input_register_device(input);
	if (error)
		goto err_free_irq;
	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);
	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
			client->adapter->nr), NULL, "ft5x0x_i2c_ts%d", 0);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************Bee-0928-BOTTOM****************************************/
	tsdata->ft5x0x_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tsdata->ft5x0x_early_suspend.suspend = ft5x0x_ts_suspend;
	tsdata->ft5x0x_early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&tsdata->ft5x0x_early_suspend);

	if((error=ft5x0x_tx_config())<0) {
		printk(KERN_ERR "%s: I2C error\n",__func__);
		goto err_i2c;
	}
	ft5x0x_create_sysfs(client);

	PIXCIR_DBG(KERN_INFO "%s:insmod successfully!\n",__func__);

	enable_irq(client->irq);
	return 0;

	printk(KERN_ERR "%s:insmod Fail!\n",__func__);

err_i2c:
	unregister_early_suspend(&tsdata->ft5x0x_early_suspend);
err_free_irq:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);
	return error;
}


/* pixcir_i2c_ts_remove -- remove pixcir from device list
 * @return: error code; 0: successful
 */
static int __devexit ft5x0x_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct ft5x0x_ts_struct *tsdata = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();
	free_irq(client->irq, tsdata);

	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************Bee-0928-BOTTOM****************************************/
	unregister_early_suspend(&tsdata->ft5x0x_early_suspend);
	//sprd_free_gpio_irq(pixcir_irq);
	input_unregister_device(tsdata->input);
	kfree(tsdata);

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_open                                     */
/*************************************Bee-0928****************************************/
static int ft5x0x_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
	PIXCIR_DBG("enter ft5x0x_open function\n");

	subminor = iminor(inode);

	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev) {
		printk(KERN_ERR "error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter) {
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}

	snprintf(client->name, I2C_NAME_SIZE, "ft5x0x_i2c_ts%d", adapter->nr);
	client->driver = &ft5x0x_i2c_ts_driver;
	client->adapter = adapter;

	file->private_data = client;

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_ioctl                                    */
/*************************************Bee-0928****************************************/
static long ft5x0x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *) file->private_data;

	PIXCIR_DBG("ft5x0x_ioctl(),cmd = %d,arg = %ld\n", cmd, arg);


	switch (cmd)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG = 1
		client->addr = SLAVE_ADDR;
		status_reg = CALIBRATION_FLAG;
		break;

	case BOOTLOADER:	//BOOTLOADER = 7
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER;

		ft5x0x_reset(g_ft5x0x_ts->platform_data->reset_gpio_number);
		mdelay(5);
		break;

	case RESET_TP:		//RESET_TP = 9
		ft5x0x_reset(g_ft5x0x_ts->platform_data->reset_gpio_number);
		break;

	case ENABLE_IRQ:	//ENABLE_IRQ = 10
		status_reg = 0;
		enable_irq(g_ft5x0x_ts->client->irq);
		break;

	case DISABLE_IRQ:	//DISABLE_IRQ = 11
		disable_irq_nosync(g_ft5x0x_ts->client->irq);
		break;

	case BOOTLOADER_STU:	//BOOTLOADER_STU = 12
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER_STU;

		ft5x0x_reset(g_ft5x0x_ts->platform_data->reset_gpio_number);
		mdelay(5);

	case ATTB_VALUE:	//ATTB_VALUE = 13
		client->addr = SLAVE_ADDR;
		status_reg = ATTB_VALUE;
		break;

	default:
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		break;
	}
	return 0;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_read                                      */
/***********************************Bee-0928****************************************/
static ssize_t ft5x0x_read (struct file *file, char __user *buf, size_t count,loff_t *offset)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	unsigned char *tmp, bootloader_stu[4], attb_value[1];
	int ret = 0;

	switch(status_reg)
	{
	case BOOTLOADER_STU:
		i2c_master_recv(client, bootloader_stu, sizeof(bootloader_stu));
		if (ret!=sizeof(bootloader_stu)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		ret = copy_to_user(buf, bootloader_stu, sizeof(bootloader_stu));
		if(ret)	{
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: copy_to_user() failed.\n",	__func__);
			return -EFAULT;
		}else {
			ret = 4;
		}
		break;

	case ATTB_VALUE:
		attb_value[0] = attb_read_val(g_ft5x0x_ts->platform_data->irq_gpio_number);
		if(copy_to_user(buf, attb_value, sizeof(attb_value))) {
			dev_err(&client->dev,
				"%s: ATTB_VALUE: copy_to_user() failed.\n", __func__);
			return -EFAULT;
		}else {
			ret = 1;
		}
		break;

	default:
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		ret = i2c_master_recv(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: default: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		if(copy_to_user(buf, tmp, count)) {
			dev_err(&client->dev,
				"%s: default: copy_to_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  ft5x0x_write                                     */
/***********************************Bee-0928****************************************/
static ssize_t ft5x0x_write(struct file *file,const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	unsigned char *tmp, bootload_data[143];
	int ret=0, i=0;

	client = file->private_data;

	switch(status_reg)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG=1
#if 0
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		if (copy_from_user(tmp,buf,count)) {
			dev_err(&client->dev,
				"%s: CALIBRATION_FLAG: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client,tmp,count);
		if (ret!=count ) {
			dev_err(&client->dev,
				"%s: CALIBRATION: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}

		while(!attb_read_val(g_pixcir_ts->platform_data->irq_gpio_number)) {
			msleep(100);
			i++;
			if(i>99)
				break;  //10s no high aatb break
		}	//waiting to finish the calibration.(pixcir application_note_710_v3 p43)

		kfree(tmp);
#endif
		break;

	case BOOTLOADER:
#if 0
		memset(bootload_data, 0, sizeof(bootload_data));

		if (copy_from_user(bootload_data, buf, count)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: copy_from_user() failed.\n", __func__);
			return -EFAULT;
		}

		ret = i2c_master_send(client, bootload_data, count);
		if(ret!=count) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: i2c_master_send() failed, ret = %d\n",
				__func__, ret);
			return -EFAULT;
		}
#endif
		break;

	default:
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		if (copy_from_user(tmp,buf,count)) {
			dev_err(&client->dev,
				"%s: default: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client,tmp,count);
		if (ret!=count ) {
			dev_err(&client->dev,
				"%s: default: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_release                                   */
/***********************************Bee-0928****************************************/
static int ft5x0x_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;

	PIXCIR_DBG("enter ft5x0x_release funtion\n");

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************Bee-0928-TOP****************************************/
static const struct file_operations ft5x0x_i2c_ts_fops =
{	.owner		= THIS_MODULE,
	.read		= ft5x0x_read,
	.write		= ft5x0x_write,
	.open		= ft5x0x_open,
	.unlocked_ioctl = ft5x0x_ioctl,
	.release	= ft5x0x_release,
};
/*********************************Bee-0928-BOTTOM****************************************/


static const struct i2c_device_id ft5x0x_i2c_ts_id[] = {
	{ FT5X0X_DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_i2c_ts_id);

static struct i2c_driver ft5x0x_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ft5x0x_i2c_ts_v3.2.0A",
	},
	.probe		= ft5x0x_i2c_ts_probe,
	.remove		= __devexit_p(ft5x0x_i2c_ts_remove),
	.id_table	= ft5x0x_i2c_ts_id,
};

static int __init ft5x0x_i2c_ts_init(void)
{
	int ret;
	PIXCIR_DBG("%s",__func__);
	/*********************************Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR,"ft5x0x_i2c_ts",&ft5x0x_i2c_ts_fops);
	if (ret) {
		printk(KERN_ERR "%s:register chrdev failed\n",__func__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "ft5x0x_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************Bee-0928-BOTTOM******************************************/
	return i2c_add_driver(&ft5x0x_i2c_ts_driver);
}

static void __exit ft5x0x_i2c_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_i2c_ts_driver);
	/********************************Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"ft5x0x_i2c_ts");
	/********************************Bee-0928-BOTTOM******************************************/
}

module_init(ft5x0x_i2c_ts_init);
module_exit(ft5x0x_i2c_ts_exit);

MODULE_AUTHOR("Yunlong wang <yunlong.wang@spreadtrum.com>");
MODULE_DESCRIPTION("Pixcir I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
