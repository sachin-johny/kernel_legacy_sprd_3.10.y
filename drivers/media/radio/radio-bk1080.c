/* radio-bk1080.c
 * A driver for the BK1080 FM receiver
 *
 * Copyright (C) 2013 Mozilla Foundation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#define DRIVER_NAME "radio-bk1080"
MODULE_AUTHOR("Michael Wu <mwu@mozilla.com>");
MODULE_DESCRIPTION("BK1080 FM Receiver Driver");
MODULE_LICENSE("GPL");

#define BKINFO(format, ...) \
	printk(KERN_INFO KBUILD_MODNAME ": " format "\n", \
		## __VA_ARGS__)
#define BKWARN(format, ...) \
	printk(KERN_WARNING KBUILD_MODNAME ": " format "\n", \
		## __VA_ARGS__)


struct beken_device {
	struct i2c_client *i2c;
	struct video_device *radio;
	struct v4l2_ctrl_handler ctrl_handler;

	unsigned int open;

	/* In V4L2 freq units */
	uint32_t rangelow, rangehigh;
	uint32_t channelspacing;

#ifdef CONFIG_ARCH_SC7710
	struct clk *clk;
#endif
};

#define BK1080_REG_CHIPID 	0x01
#define BK1080_REG_POWER_CONFIG	0x02
#define BK1080_REG_POWER_CONFIG_SOFTMUTE_DISABLE	(1 << 15)
#define BK1080_REG_POWER_CONFIG_MUTE			(1 << 14)
#define BK1080_REG_POWER_CONFIG_FORCE_MONO		(1 << 13)
#define BK1080_REG_POWER_CONFIG_CLOCK_SEL		(1 << 12)
#define BK1080_REG_POWER_CONFIG_SEEK_MODE		(1 << 10)
#define BK1080_REG_POWER_CONFIG_SEEK_UP			(1 <<  9)
#define BK1080_REG_POWER_CONFIG_SEEK			(1 <<  8)
#define BK1080_REG_POWER_CONFIG_DISABLE			(1 <<  6)
#define BK1080_REG_POWER_CONFIG_ENABLE			(1 <<  0)

#define BK1080_REG_CHANNEL	0x03
#define BK1080_REG_CHANNEL_TUNE				(1 << 15)

#define BK1080_REG_SYS_CONFIG_1	0x04
#define BK1080_REG_SYS_CONFIG_1_ENABLE_STC_INTERRUPT	(1 << 14)
#define BK1080_REG_SYS_CONFIG_1_BYPASS_DEEMPHASIS	(1 << 13)
#define BK1080_REG_SYS_CONFIG_1_DEEMPHASIS_MODE		(1 << 11)
#define BK1080_REG_SYS_CONFIG_1_AGC_DISABLE		(1 << 10)

#define BK1080_REG_SYS_CONFIG_2	0x05
#define BK1080_REG_SYS_CONFIG_3	0x06
#define BK1080_REG_TEST_1	0x07
#define BK1080_REG_TEST_2	0x08
#define BK1080_REG_BOOT_CONFIG	0x09
#define BK1080_REG_RSSI_STATUS	0x0A
#define BK1080_REG_RSSI_STATUS_SEEKTUNE_COMPLETE	(1 << 14)
#define BK1080_REG_RSSI_STATUS_SEEKFAIL_BANDLIMIT	(1 << 13)
#define BK1080_REG_RSSI_STATUS_STEREO_DECODER		(1 <<  9)
#define BK1080_REG_RSSI_STATUS_STEREO			(1 <<  8)

#define BK1080_REG_READ_CHANNEL	0x0B

/* Register init table from beken_fm_ctrl.c driver */
static const uint16_t BK1080_Digital_Reg[] = {
	0x0008, // REG0
	0x1080, // REG1
	0x4201, // REG2
	0x0000, // REG3
	0x40C0, // REG4
	0x0800, // REG5
	0x002E, // REG6
	0x02FF, // REG7
	0x5B11, // REG8
	0x0000, // REG9
	0x411E, // REG10
	0x0000, // REG11
	0xCE00, // REG12
	0x0000, // REG13
	0x0000, // REG14
	0x1000, // REG15
	0x0010, // REG16
	0x0000, // REG17
	0x13FF, // REG18
	0x9852, // REG19
	0x0000, // REG20
	0x0000, // REG21
	0x0008, // REG22
	0x0000, // REG23
	0x51E1, // REG24
	0x38BC, // REG25
	0x2645, // REG26
	0x00E4, // REG27
	0x1CD8, // REG28
	0x3A50, // REG29
	0xEAF0, // REG30
	0x3000, // REG31
	0x00B8, // REG32
	0x0000, // REG33
};

static int beken_write_registers(struct beken_device *dev,
				 uint8_t addr,
				 const uint16_t *data,
				 int len)
{
	uint8_t buf[128];

	int bytelen = sizeof(*data) * len;
	if (bytelen >= sizeof(buf))
		return -EINVAL;

	buf[0] = addr;
	int i;
	for (i = 0; i < len; i++)
		((__be16 *)(buf + 1))[i] = cpu_to_be16(data[i]);

	struct i2c_msg msgs[] = {
		{
			.addr = dev->i2c->addr,
			.flags = 0,
			.len = bytelen + 1,
			.buf = buf,
		},
	};

	if (i2c_transfer(dev->i2c->adapter, msgs, ARRAY_SIZE(msgs)) !=
	    ARRAY_SIZE(msgs)) {
		BKWARN("Register write (addr %d) failed", addr);
		return -EIO;
	}

	return 0;
}

static int beken_write_register(struct beken_device *dev,
				uint8_t addr,
				uint16_t data)
{
	return beken_write_registers(dev, addr, &data, 1);
}

static int beken_read_register(struct beken_device *dev,
			       uint8_t addr,
			       uint16_t *data)
{
	uint8_t addrbuf[3];

	addrbuf[0] = 0x7E;
	addrbuf[1] = 0;
	addrbuf[2] = addr;

	__be16 reg;

	struct i2c_msg msgs[] = {
		{
			.addr = dev->i2c->addr,
			.flags = 0,
			.len = sizeof(addrbuf),
			.buf = addrbuf,
		},
		{
			.addr = dev->i2c->addr,
			.flags = I2C_M_RD,
			.len = sizeof(reg),
			.buf = &reg,
		},
	};

	if (i2c_transfer(dev->i2c->adapter, msgs, ARRAY_SIZE(msgs)) !=
	    ARRAY_SIZE(msgs)) {
		BKWARN("Register read (addr %d) failed", addr);
		return -EIO;
	}

	*data = be16_to_cpu(reg);
	return 0;
}

static int beken_open(struct file *file)
{
	struct beken_device *dev = video_drvdata(file);
	dev->open++;
	if (dev->open > 1)
		return 0;

	/* XXX Magic initialization sequence */
	beken_write_registers(dev, 0,
			      BK1080_Digital_Reg,
			      ARRAY_SIZE(BK1080_Digital_Reg));
	msleep(300);

	beken_write_register(dev, 25, BK1080_Digital_Reg[25] & ~0x80);
	beken_write_register(dev, 25, BK1080_Digital_Reg[25]);
	msleep(80);

	v4l2_ctrl_handler_setup(&dev->ctrl_handler);

#ifdef CONFIG_ARCH_SC7710
	clk_enable(dev->clk);
#endif

	dev->rangelow = 1400000;
	dev->rangehigh = 1728000;
	dev->channelspacing = 3200; /* 200kHz / 62.5Hz */

	return 0;
}

static int beken_close(struct file *file)
{
	struct beken_device *dev = video_drvdata(file);
	dev->open--;
	if (dev->open)
		return 0;

	int err;
	uint16_t power_conf = 0;
	beken_read_register(dev, BK1080_REG_POWER_CONFIG, &power_conf);
	power_conf |= BK1080_REG_POWER_CONFIG_DISABLE |
		      BK1080_REG_POWER_CONFIG_ENABLE;
	beken_write_register(dev, BK1080_REG_POWER_CONFIG, power_conf);

#ifdef CONFIG_ARCH_SC7710
	clk_disable(dev->clk);
#endif
	return 0;
}

static const struct v4l2_file_operations beken_fops = {
	.owner = THIS_MODULE,
	.open = beken_open,
	.release = beken_close,
	.unlocked_ioctl = video_ioctl2,
};

static int beken_querycap(struct file *file,
			  void *priv,
			  struct v4l2_capability *cap)
{
	struct beken_device *dev = video_drvdata(file);
	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "Beken BK1080 FM Receiver", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "I2C:%s", dev_name(&dev->radio->dev));
	cap->version = KERNEL_VERSION(0, 1, 0);
	cap->capabilities = V4L2_CAP_RADIO |
			    V4L2_CAP_TUNER |
			    V4L2_CAP_HW_FREQ_SEEK;
	return 0;
}

static int beken_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct beken_device *dev =
		container_of(ctrl->handler, struct beken_device, ctrl_handler);
	int err = -EINVAL;
	uint16_t reg;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		err = beken_read_register(dev, BK1080_REG_SYS_CONFIG_2, &reg);
		if (err)
			return err;

		reg &= ~0x000F;
		reg |= ctrl->val;

		err = beken_write_register(dev, BK1080_REG_SYS_CONFIG_2, reg);
		break;
	case V4L2_CID_AUDIO_MUTE:
		err = beken_read_register(dev, BK1080_REG_POWER_CONFIG, &reg);
		if (err)
			return err;

		if (ctrl->val)
			reg |= BK1080_REG_POWER_CONFIG_MUTE;
		else
			reg &= ~BK1080_REG_POWER_CONFIG_MUTE;

		err = beken_write_register(dev, BK1080_REG_POWER_CONFIG, reg);
		break;
	case V4L2_CID_TUNE_PREEMPHASIS:
		err = beken_read_register(dev, BK1080_REG_SYS_CONFIG_1, &reg);
		if (err)
			return err;

		reg &= ~(BK1080_REG_SYS_CONFIG_1_BYPASS_DEEMPHASIS |
			 BK1080_REG_SYS_CONFIG_1_DEEMPHASIS_MODE);
		switch (ctrl->val) {
		case V4L2_PREEMPHASIS_DISABLED:
			reg |= BK1080_REG_SYS_CONFIG_1_BYPASS_DEEMPHASIS;
			break;
		case V4L2_PREEMPHASIS_50_uS:
			reg |= BK1080_REG_SYS_CONFIG_1_DEEMPHASIS_MODE;
			break;
		case V4L2_PREEMPHASIS_75_uS:
			break;
		}
		err = beken_write_register(dev, BK1080_REG_SYS_CONFIG_1, reg);
		break;
	}

	return err;
}

static const struct v4l2_ctrl_ops beken_ctrl_ops = {
	.s_ctrl = beken_set_ctrl,
};

static int beken_get_tuner(struct file *file,
			   void *priv,
			   struct v4l2_tuner *tuner)
{
	if (tuner->index > 0)
		return -EINVAL;

	struct beken_device *dev = video_drvdata(file);
	uint16_t rssi_status, power_conf;
	int err = beken_read_register(dev, BK1080_REG_POWER_CONFIG, &power_conf);
	if (err)
		return err;

	err = beken_read_register(dev, BK1080_REG_RSSI_STATUS, &rssi_status);
	if (err)
		return err;

	strlcpy(tuner->name, "FM", sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
#ifndef V4L2_TUNER_CAP_HWSEEK_BOUNDED
	tuner->capability = V4L2_TUNER_CAP_LOW;
#else
	tuner->capability = V4L2_TUNER_CAP_LOW |
			    V4L2_TUNER_CAP_HWSEEK_BOUNDED |
			    V4L2_TUNER_CAP_HWSEEK_WRAP;
#endif
	tuner->rangelow = dev->rangelow;
	tuner->rangehigh = dev->rangehigh;

	tuner->rxsubchans = (rssi_status & BK1080_REG_RSSI_STATUS_STEREO) ?
			    V4L2_TUNER_SUB_STEREO : V4L2_TUNER_SUB_MONO;
	tuner->audmode = (power_conf & BK1080_REG_POWER_CONFIG_FORCE_MONO) ?
			 V4L2_TUNER_MODE_MONO : V4L2_TUNER_MODE_STEREO;
	tuner->signal = (rssi_status & 0xFF) << 8;
	tuner->afc = 0;
	memset(tuner->reserved, 0, sizeof(tuner->reserved));
	return 0;
}

static int beken_set_tuner(struct file *file,
			   void *priv,
			   struct v4l2_tuner *tuner)
{
	if (tuner->index > 0 || tuner->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	struct beken_device *dev = video_drvdata(file);
	int err;

	uint16_t power_conf;
	err = beken_read_register(dev, BK1080_REG_POWER_CONFIG, &power_conf);
	if (err)
		return err;

	uint16_t sysconf2;
	err = beken_read_register(dev, BK1080_REG_SYS_CONFIG_2, &sysconf2);
	if (err)
		return err;

	uint8_t type;
	if (tuner->rangelow == 1400000 && tuner->rangehigh == 1728000)
		type = 0;
	else if (tuner->rangelow == 1216000 && tuner->rangehigh == 1728000)
		type = 1;
	else if (tuner->rangelow == 1216000 && tuner->rangehigh == 1440000)
		type = 2;
	else if (tuner->rangelow == 1024000 && tuner->rangehigh == 1216000)
		type = 3;
	else
		return -EINVAL;

	dev->rangelow = tuner->rangelow;
	dev->rangehigh = tuner->rangehigh;

	sysconf2 &= ~(0x3 << 6);
	sysconf2 |= type << 6;

	if (tuner->audmode == V4L2_TUNER_MODE_MONO)
		power_conf |= BK1080_REG_POWER_CONFIG_FORCE_MONO;
	else
		power_conf &= ~BK1080_REG_POWER_CONFIG_FORCE_MONO;

	beken_write_register(dev, BK1080_REG_POWER_CONFIG, power_conf);
	beken_write_register(dev, BK1080_REG_SYS_CONFIG_2, sysconf2);

	return 0;
}

static int beken_get_freq(struct file *file,
			  void *priv,
			  struct v4l2_frequency *freq)
{
	struct beken_device *dev = video_drvdata(file);

	uint16_t chan;
	int err = beken_read_register(dev, BK1080_REG_READ_CHANNEL, &chan);
	if (err)
		return err;

	freq->type = V4L2_TUNER_RADIO;
	freq->frequency = dev->rangelow + (chan & 0x3FF) * dev->channelspacing;
	memset(freq->reserved, 0, sizeof(freq->reserved));
	return 0;
}

static int beken_update_channel_spacing(struct beken_device *dev,
					uint32_t spacing)
{
	uint16_t sysconf2;
	int err;
	err = beken_read_register(dev, BK1080_REG_SYS_CONFIG_2, &sysconf2);
	if (err)
		return err;

	sysconf2 &= ~(0x3 << 4);
	switch (spacing) {
	case 3200:
		break;
	case 1600:
		sysconf2 |= (1 << 4);
		break;
	case 800:
		sysconf2 |= (2 << 4);
		break;
	default:
		return -EINVAL;
	}

	err = beken_write_register(dev, BK1080_REG_SYS_CONFIG_2, sysconf2);
	if (err)
		return err;

	dev->channelspacing = spacing;
	return 0;
}

static int beken_set_freq(struct file *file,
			  void *priv,
			  struct v4l2_frequency *freq)
{
	if (freq->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	struct beken_device *dev = video_drvdata(file);
	if (freq->frequency < dev->rangelow ||
	    freq->frequency > dev->rangehigh)
		return -EINVAL;

	int err;
	uint32_t convertedfreq = freq->frequency - dev->rangelow;
	if (convertedfreq % dev->channelspacing) {
		uint32_t spacing = 0;
		if (!(convertedfreq % 1600))
			spacing = 1600;
		else if (!(convertedfreq % 800))
			spacing = 800;
		else
			return -EINVAL;

		err = beken_update_channel_spacing(dev, spacing);
		if (err)
			return err;
	}

	convertedfreq /= dev->channelspacing;
	err = beken_write_register(dev, BK1080_REG_CHANNEL,
				   convertedfreq | BK1080_REG_CHANNEL_TUNE);
	if (err)
		return err;

	int i;
	uint16_t rssi_status;
	for (i = 0; i < 20; i++) {
		err = beken_read_register(dev, BK1080_REG_RSSI_STATUS,
					  &rssi_status);
		if (err ||
		    rssi_status & BK1080_REG_RSSI_STATUS_SEEKTUNE_COMPLETE)
			goto out;
		msleep(100);
	}

out:
	beken_write_register(dev, BK1080_REG_CHANNEL, convertedfreq);
	return err;
}

static int beken_seek(struct file *file,
		      void *priv,
		      struct v4l2_hw_freq_seek *seek)
{
	if (seek->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	struct beken_device *dev = video_drvdata(file);
	uint32_t new_spacing = 0;

	switch (seek->spacing) {
	case 200000:
		if (dev->channelspacing != 3200)
			new_spacing = 3200;
		break;
	case 100000:
		if (dev->channelspacing != 1600)
			new_spacing = 1600;
		break;
	case 50000:
		if (dev->channelspacing != 800)
			new_spacing = 800;
		break;
	case 0:
		break;
	default:
		return -EINVAL;
	}

	int err;
	if (new_spacing) {
		struct v4l2_frequency freq;
		err = beken_get_freq(file, priv, &freq);
		if (err) {
			BKWARN("Couldn't get current channel to reconfigure "
			       "channel spacing for seeking");
			return err;
		}

		uint32_t rounded_freq = freq.frequency;

		rounded_freq -= dev->rangelow;
		rounded_freq /= new_spacing;
		rounded_freq *= new_spacing;
		rounded_freq += dev->rangelow;

		err = beken_update_channel_spacing(dev, new_spacing);
		if (err) {
			BKWARN("Couldn't switch to new channel spacing "
			       "before seeking");
			return err;
		}

		err = beken_set_freq(file, priv, &freq);
		if (err) {
			BKWARN("Couldn't switch to new frequency "
			       "before seeking");
			return err;
		}
	}

	uint16_t power_conf;
	err = beken_read_register(dev, BK1080_REG_POWER_CONFIG, &power_conf);
	if (err)
		return err;

	if (seek->seek_upward)
		power_conf |= BK1080_REG_POWER_CONFIG_SEEK_UP;
	else
		power_conf &= ~BK1080_REG_POWER_CONFIG_SEEK_UP;

	if (seek->wrap_around)
		power_conf &= ~BK1080_REG_POWER_CONFIG_SEEK_MODE;
	else
		power_conf |= BK1080_REG_POWER_CONFIG_SEEK_MODE;

	power_conf &= ~BK1080_REG_POWER_CONFIG_SEEK;

	err = beken_write_register(dev, BK1080_REG_POWER_CONFIG,
				   power_conf |
				   BK1080_REG_POWER_CONFIG_SEEK |
				   BK1080_REG_POWER_CONFIG_MUTE);
	if (err)
		return err;

	int i;
	uint16_t rssi_status;
	for (i = 0; i < 50; i++) {
		err = beken_read_register(dev, BK1080_REG_RSSI_STATUS,
					  &rssi_status);
		if (err ||
		    rssi_status & BK1080_REG_RSSI_STATUS_SEEKTUNE_COMPLETE)
			goto out;
		msleep(100);
	}

out:
	beken_write_register(dev, BK1080_REG_POWER_CONFIG, power_conf);
	return err;
}

static const struct v4l2_ioctl_ops beken_ioctl_ops = {
	.vidioc_querycap = beken_querycap,
	.vidioc_g_tuner = beken_get_tuner,
	.vidioc_s_tuner = beken_set_tuner,
	.vidioc_g_frequency = beken_get_freq,
	.vidioc_s_frequency = beken_set_freq,
	.vidioc_s_hw_freq_seek = beken_seek,
};

static int __devinit beken_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct beken_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		BKWARN("Could not allocate beken_device");
		return -ENOMEM;
	}

	struct video_device *radio = dev->radio = video_device_alloc();
	if (!radio) {
		BKWARN("Could not allocate video_device");
		goto alloc_fail;
	}

	if (IS_ERR(v4l2_ctrl_handler_init(&dev->ctrl_handler, 3))) {
		BKWARN("Could not initialize ctrl_handler");
		goto ctrl_alloc_fail;
	}

#ifdef CONFIG_ARCH_SC7710
	dev->clk = clk_get(NULL, "clk_aux1");
	if (IS_ERR(dev->clk)) {
		goto ctrl_alloc_fail;
	}

	struct clk *clk_parent = clk_get(NULL, "ext_32k");
	if (IS_ERR(clk_parent)) {
		goto ctrl_alloc_fail;
	}

	clk_set_parent(dev->clk, clk_parent);
	clk_set_rate(dev->clk, 32000);
#endif

	dev->i2c = client;
	i2c_set_clientdata(client, dev);
	video_set_drvdata(radio, dev);
	radio->fops = &beken_fops;
	strlcpy(radio->name, DRIVER_NAME, sizeof(radio->name));
	radio->release = video_device_release;
	radio->ioctl_ops = &beken_ioctl_ops;

	uint16_t chipid = 0;
	beken_read_register(dev, BK1080_REG_CHIPID, &chipid);
	if (chipid != 0x1080) {
		BKWARN("Chip ID doesn't match (Got 0x%04x)", chipid);
		goto ctrl_alloc_fail;
	}

	v4l2_ctrl_new_std(&dev->ctrl_handler, &beken_ctrl_ops,
			  V4L2_CID_AUDIO_VOLUME, 0, 15, 1, 0);

	v4l2_ctrl_new_std(&dev->ctrl_handler, &beken_ctrl_ops,
			  V4L2_CID_AUDIO_MUTE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu(&dev->ctrl_handler, &beken_ctrl_ops,
			       V4L2_CID_TUNE_PREEMPHASIS,
			       V4L2_PREEMPHASIS_75_uS, 0x7,
			       V4L2_PREEMPHASIS_75_uS);

	radio->ctrl_handler = &dev->ctrl_handler;

	if (video_register_device(radio, VFL_TYPE_RADIO, -1)) {
		BKWARN("Could not register video_device");
		goto register_fail;
	}

	BKINFO("Registered BK1080 FM Receiver.");

	return 0;

register_fail:
	video_device_release(radio);

ctrl_alloc_fail:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);

alloc_fail:
	kfree(dev);
	return -EINVAL;
}

static int __devexit beken_i2c_remove(struct i2c_client *client)
{
	struct beken_device *dev = i2c_get_clientdata(client);
	if (!dev)
		return 0;

	video_device_release(dev->radio);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	kfree(dev);
	return 0;
}

static const struct i2c_device_id beken_id[] = {
	{ "BEKEN_FM", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, beken_id);

static struct i2c_driver beken_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = beken_i2c_probe,
	.remove = __devexit_p(beken_i2c_remove),
	.id_table = beken_id,
};

static int __init beken_init(void)
{
	return i2c_add_driver(&beken_i2c_driver);
}

static void __exit beken_exit(void)
{
	i2c_del_driver(&beken_i2c_driver);
}

module_init(beken_init);
module_exit(beken_exit);
