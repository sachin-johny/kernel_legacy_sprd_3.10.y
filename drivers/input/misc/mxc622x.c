/*
 * Copyright (C) 2012 MEMSIC, Inc.
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include 	<linux/module.h>
#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include	<linux/slab.h>

#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include        <linux/earlysuspend.h>
#endif

#include        <linux/i2c/mxc622x.h>


#define	G_MAX		16000	/** Maximum polled-device-reported g value */
#define WHOAMI_MXC622X_ACC	0x05	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x08	/*	WhoAmI register		*/

#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */

#define	RESUME_ENTRIES		20
#define DEVICE_INFO         "Memsic, MXC622X"
#define DEVICE_INFO_LEN     32

/* end RESUME STATE INDICES */

#define DEBUG
#define MXC622X_DEBUG

#define	MAX_INTERVAL	50

#ifdef __KERNEL__
static struct mxc622x_acc_platform_data mxc622x_plat_data = {
    .poll_interval = 20,
    .min_interval = 10,
};
#endif

#ifdef I2C_BUS_NUM_STATIC_ALLOC
static struct i2c_board_info  mxc622x_i2c_boardinfo = {
        I2C_BOARD_INFO(MXC622X_ACC_I2C_NAME, MXC622X_ACC_I2C_ADDR),
#ifdef __KERNEL__
        .platform_data = &mxc622x_plat_data
#endif
};
#endif

struct mxc622x_acc_data {
	struct i2c_client *client;
	struct mxc622x_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct mxc622x_acc_data *mxc622x_acc_misc_data;
struct i2c_client      *mxc622x_i2c_client;

static int mxc622x_acc_i2c_read(struct mxc622x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc622x_acc_i2c_write(struct mxc622x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = { { .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf, }, };
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc622x_acc_hw_init(struct mxc622x_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	printk( "%s: hw init start\n", MXC622X_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = mxc622x_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if ((buf[0] & 0x3F) != WHOAMI_MXC622X_ACC) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	acc->hw_initialized = 1;
	printk( "%s: hw init done\n", MXC622X_ACC_DEV_NAME);
	return 0;

error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_MXC622X_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void mxc622x_acc_device_power_off(struct mxc622x_acc_data *acc)
{
	int err;
	u8 buf[2] = { MXC622X_REG_CTRL, MXC622X_CTRL_PWRDN };

	err = mxc622x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
}

static int mxc622x_acc_device_power_on(struct mxc622x_acc_data *acc)
{
	int err = -1;
	u8 buf[2] = { MXC622X_REG_CTRL, MXC622X_CTRL_PWRON };

	err = mxc622x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power on failed: %d\n", err);
	
	if (!acc->hw_initialized) {
		err = mxc622x_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			mxc622x_acc_device_power_off(acc);
			return err;
		}
	}

	return 0;
}


/* */

static int mxc622x_acc_register_write(struct mxc622x_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	if (atomic_read(&acc->enabled)) {
		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = mxc622x_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}
	return err;
}

static int mxc622x_acc_register_read(struct mxc622x_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = mxc622x_acc_i2c_read(acc, buf, 1);
	return err;
}

static int mxc622x_acc_register_update(struct mxc622x_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = mxc622x_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = mxc622x_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}

/* */

static int mxc622x_acc_get_acceleration_data(struct mxc622x_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware x, y */
	u8 acc_data[2];

	acc_data[0] = MXC622X_REG_DATA;
	err = mxc622x_acc_i2c_read(acc, acc_data, 2);

	if (err < 0)
        {
                #ifdef DEBUG
                printk( "%s I2C read error %d\n", MXC622X_ACC_I2C_NAME, err);
                #endif
		return err;
        }

	xyz[0] = -((signed char)acc_data[1]);
	xyz[1] = -((signed char)acc_data[0]);
	xyz[2] = 32;

      #ifdef MXC622X_DEBUG
      printk("x = %d, y = %d\n", xyz[0], xyz[1]);
      #endif

	#ifdef MXC622X_DEBUG

		printk( "%s read x=%d, y=%d, z=%d\n",
			MXC622X_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
		printk( "%s poll interval %d\n", MXC622X_ACC_DEV_NAME, acc->pdata->poll_interval);

	#endif
	return err;
}

static void mxc622x_acc_report_values(struct mxc622x_acc_data *acc, int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
	printk( "%s acc_report_values read x=%d, y=%d, z=%d\n",
			MXC622X_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
}

static int mxc622x_acc_enable(struct mxc622x_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = mxc622x_acc_device_power_on(acc);
		msleep(300);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}

		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				acc->pdata->poll_interval));
	}

	return 0;
}

static int mxc622x_acc_disable(struct mxc622x_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		mxc622x_acc_device_power_off(acc);
	}

	return 0;
}

static int mxc622x_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = mxc622x_acc_misc_data;

	return 0;
}

static int mxc622x_acc_misc_ioctl(/*struct inode *inode,*/ struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	u8 mask;
	u8 reg_address;
	u8 bit_values;
	int err;
	int interval;
    int xyz[3] = {0};
	struct mxc622x_acc_data *acc = file->private_data;

//	printk( "%s: %s call with cmd 0x%x and arg 0x%x\n",
//			MXC622X_ACC_DEV_NAME, __func__, cmd, (unsigned int)arg);

	switch (cmd) {
	case MXC622X_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case MXC622X_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1000)
			return -EINVAL;
		if(interval > MAX_INTERVAL)
			interval = MAX_INTERVAL;
		acc->pdata->poll_interval = max(interval,
				acc->pdata->min_interval);
		break;

	case MXC622X_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 1)
			return -EINVAL;
		if (interval)
			err = mxc622x_acc_enable(acc);
		else
			err = mxc622x_acc_disable(acc);
		return err;
		break;

	case MXC622X_ACC_IOCTL_GET_ENABLE:
		interval = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;
	case MXC622X_ACC_IOCTL_GET_COOR_XYZ:	       
		err = mxc622x_acc_get_acceleration_data(acc, xyz);                
		if (err < 0)                    
			return err;
		#ifdef DEBUG
		//                printk(KERN_ALERT "%s Get coordinate xyz:[%d, %d, %d]\n",
		//                        __func__, xyz[0], xyz[1], xyz[2]);
		#endif                
		if (copy_to_user(argp, xyz, sizeof(xyz))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);			
			return -EINVAL;                
			}                
		break;
	case MXC622X_ACC_IOCTL_GET_CHIP_ID:
	{
		u8 devid = 0;
		u8 devinfo[DEVICE_INFO_LEN] = {0};
		err = mxc622x_acc_register_read(acc, &devid, WHO_AM_I);
		if (err < 0) {
			printk("%s, error read register WHO_AM_I\n", __func__);
			return -EAGAIN;
		}
		sprintf(devinfo, "%s, %#x", DEVICE_INFO, devid);

		if (copy_to_user(argp, devinfo, sizeof(devinfo))) {
			printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
			return -EINVAL;
		}
	}
            break;


	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations mxc622x_acc_misc_fops = {
		.owner = THIS_MODULE,
		.open = mxc622x_acc_misc_open,
		.unlocked_ioctl = mxc622x_acc_misc_ioctl,
};

static struct miscdevice mxc622x_acc_misc_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = MXC622X_ACC_DEV_NAME,
		.fops = &mxc622x_acc_misc_fops,
};

static void mxc622x_acc_input_work_func(struct work_struct *work)
{
	struct mxc622x_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct mxc622x_acc_data,	input_work);

	mutex_lock(&acc->lock);
	err = mxc622x_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		mxc622x_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef MXC622X_OPEN_ENABLE
int mxc622x_acc_input_open(struct input_dev *input)
{
	struct mxc622x_acc_data *acc = input_get_drvdata(input);

	return mxc622x_acc_enable(acc);
}

void mxc622x_acc_input_close(struct input_dev *dev)
{
	struct mxc622x_acc_data *acc = input_get_drvdata(dev);

	mxc622x_acc_disable(acc);
}
#endif

static int mxc622x_acc_validate_pdata(struct mxc622x_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int mxc622x_acc_input_init(struct mxc622x_acc_data *acc)
{
	int err;
    // Polling rx data when the interrupt is not used.
    if (1/*acc->irq1 == 0 && acc->irq1 == 0*/) {
    	INIT_DELAYED_WORK(&acc->input_work, mxc622x_acc_input_work_func);
    }

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef MXC622X_ACC_OPEN_ENABLE
	acc->input_dev->open = mxc622x_acc_input_open;
	acc->input_dev->close = mxc622x_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	acc->input_dev->name = MXC622X_ACC_INPUT_NAME;

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input polled device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void mxc622x_acc_input_cleanup(struct mxc622x_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc622x_early_suspend (struct early_suspend* es);
static void mxc622x_early_resume (struct early_suspend* es);
#endif


static int mxc622x_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct mxc622x_acc_data *acc;

	int err = -1;
	int tempvalue;

	printk("%s: probe start.\n", MXC622X_ACC_DEV_NAME);

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}


	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	acc = kzalloc(sizeof(struct mxc622x_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
    mxc622x_i2c_client = client;
	i2c_set_clientdata(client, acc);

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);

	if ((tempvalue & 0x003F) == WHOAMI_MXC622X_ACC) {
		printk( "%s I2C driver registered!\n",
							MXC622X_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		printk( "I2C driver not registered!"
				" Device unknown 0x%x\n", tempvalue);
		goto err_mutexunlockfreedata;
	}
	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto exit_kfree_pdata;
	}

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = mxc622x_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	i2c_set_clientdata(client, acc);


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	err = mxc622x_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&acc->enabled, 1);

	err = mxc622x_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}
	mxc622x_acc_misc_data = acc;

	err = misc_register(&mxc622x_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev,
				"misc MXC622X_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}

	mxc622x_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

    acc->on_before_suspend = 0;

 #ifdef CONFIG_HAS_EARLYSUSPEND
    acc->early_suspend.suspend = mxc622x_early_suspend;
    acc->early_suspend.resume  = mxc622x_early_resume;
    acc->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    register_early_suspend(&acc->early_suspend);
#endif

	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed\n", MXC622X_ACC_DEV_NAME);

	return 0;

err_input_cleanup:
	mxc622x_acc_input_cleanup(acc);
err_power_off:
	mxc622x_acc_device_power_off(acc);
err2:
	if (acc->pdata->exit) acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlockfreedata:
	kfree(acc);
 	mutex_unlock(&acc->lock);
    i2c_set_clientdata(client, NULL);
    mxc622x_acc_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", MXC622X_ACC_DEV_NAME);
	return err;
}

static int mxc622x_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);
	
    	misc_deregister(&mxc622x_acc_misc_device);
    	mxc622x_acc_input_cleanup(acc);
    	mxc622x_acc_device_power_off(acc);
    	if (acc->pdata->exit)
    		acc->pdata->exit();
    	kfree(acc->pdata);
    	kfree(acc);
    
	return 0;
}

static int mxc622x_acc_resume(struct i2c_client *client)
{
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);
#ifdef MXC622X_DEBUG
    printk("%s.\n", __func__);
#endif

	if (acc != NULL && acc->on_before_suspend) {
        acc->on_before_suspend = 0;
		return mxc622x_acc_enable(acc);
    }

	return 0;
}

static int mxc622x_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxc622x_acc_data *acc = i2c_get_clientdata(client);
#ifdef MXC622X_DEBUG
    printk("%s.\n", __func__);
#endif
    if (acc != NULL) {
        if (atomic_read(&acc->enabled)) {
            acc->on_before_suspend = 1;
            return mxc622x_acc_disable(acc);
        }
    }
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void mxc622x_early_suspend (struct early_suspend* es)
{
#ifdef MXC622X_DEBUG
    printk("%s.\n", __func__);
#endif
    mxc622x_acc_suspend(mxc622x_i2c_client,
         (pm_message_t){.event=0});
}

static void mxc622x_early_resume (struct early_suspend* es)
{
#ifdef MXC622X_DEBUG
    printk("%s.\n", __func__);
#endif
    mxc622x_acc_resume(mxc622x_i2c_client);
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id mxc622x_acc_id[]
				= { { MXC622X_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, mxc622x_acc_id);

static struct i2c_driver mxc622x_acc_driver = {
	.driver = {
			.name = MXC622X_ACC_I2C_NAME,
		  },
	.probe = mxc622x_acc_probe,
	.remove = mxc622x_acc_remove,
	.resume = mxc622x_acc_resume,
	.suspend = mxc622x_acc_suspend,
	.id_table = mxc622x_acc_id,
};

/*

#ifdef I2C_BUS_NUM_STATIC_ALLOC

int i2c_static_add_device(struct i2c_board_info *info)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int err;

	adapter = i2c_get_adapter(I2C_STATIC_BUS_NUM);
	if (!adapter) {
		pr_err("%s: can't get i2c adapter\n", __FUNCTION__);
		err = -ENODEV;
		goto i2c_err;
	}

	client = i2c_new_device(adapter, info);
	if (!client) {
		pr_err("%s:  can't add i2c device at 0x%x\n",
			__FUNCTION__, (unsigned int)info->addr);
		err = -ENODEV;
		goto i2c_err;
	}

	i2c_put_adapter(adapter);

	return 0;

i2c_err:
	return err;
}

#endif


static int __init mxc622x_acc_init(void)
{
        int  ret = 0;

    	printk( "%s accelerometer driver: init\n",
						MXC622X_ACC_I2C_NAME);
#ifdef I2C_BUS_NUM_STATIC_ALLOC
        ret = i2c_static_add_device(&mxc622x_i2c_boardinfo);
        if (ret < 0) {
            pr_err("%s: add i2c device error %d\n", __FUNCTION__, ret);
            goto init_err;
        }
#endif

        return i2c_add_driver(&mxc622x_acc_driver);

init_err:
        return ret;
}

static void __exit mxc622x_acc_exit(void)
{
	printk( "%s accelerometer driver exit\n", MXC622X_ACC_DEV_NAME);

	#ifdef I2C_BUS_NUM_STATIC_ALLOC
    i2c_unregister_device(mxc622x_i2c_client);
	#endif

	i2c_del_driver(&mxc622x_acc_driver);
	return;
}
*/

static int __init mxc622x_acc_init(void)
{
	return i2c_add_driver(&mxc622x_acc_driver);
}

static void __exit mxc622x_acc_exit(void)
{
	i2c_del_driver(&mxc622x_acc_driver);
	return;
}


late_initcall(mxc622x_acc_init);
module_exit(mxc622x_acc_exit);

MODULE_AUTHOR("Memsic");
MODULE_DESCRIPTION("mxc622x accelerometer misc driver");
MODULE_LICENSE("GPL");
