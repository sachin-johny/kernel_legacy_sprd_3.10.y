#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include  <linux/module.h>
#include <linux/platform_device.h>

#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>


#include "trout_fm_ctrl.h"
#include "trout_rf_common.h"
#include "trout_interface.h"

#define TROUT_FM_VERSION "V0.1"
#define DRIVER_NAME "radio-trout"
#define CARD_NAME "Trout FM Receiver"
#define FM_FREQ_CONVERSION (100*16)
#define V4L2_CID_PRIVATE_FM_AUDIO (V4L2_CID_BASE + 10)

struct fmdev {
	struct v4l2_ctrl_handler ctrl_handler; /* V4L2 ctrl framwork handler*/
};

atomic_t  is_fm_open;
static struct video_device *s_radio = NULL;

struct trout_interface *p_trout_interface;

static int g_volume = 0;

int trout_fm_set_volume(u8 iarg)
{
	TROUT_PRINT("FM set volume : %i.", iarg);

	g_volume = (int)iarg;
	if(0 == iarg) {
		trout_fm_mute();
	}
	else {
		trout_fm_unmute();
	}

	return 0;
}

int trout_fm_get_volume(void)
{
	TROUT_PRINT("FM get volume.");
	return g_volume;
}

static int _trout_fm_open(struct file *file)
{
	int ret = -EINVAL;
    int status;

	TROUT_PRINT("start open fm module...");

	if (atomic_read(&is_fm_open)) {
		TROUT_PRINT("trout_fm has been opened!");
		return -1;
	}
#if 0
	if (get_suspend_state() != PM_SUSPEND_ON)
	{
		TROUT_PRINT("The system is suspending!");
		return -2;
	}

	/*ret = nonseekable_open(inode, filep);
	if (ret < 0)
	{
		TROUT_PRINT("open misc device failed.");
		return ret;
	}*/

	Set_Trout_PowerOn(2/*FM_MODE*/);
#endif
	ret = trout_fm_init();
	if(ret < 0) {
		TROUT_PRINT("trout_fm_init failed!");
		return ret;
	}

	ret = trout_fm_get_status(&status);
	if(ret < 0) {
		TROUT_PRINT("trout_read_fm_en failed!");
		return ret;
	}

	if(status) {
		TROUT_PRINT("trout fm have been opened.");
	}
	else {
		ret = trout_fm_en();
		if(ret < 0) {
			TROUT_PRINT("trout_fm_en failed!");
			return ret;
		}
#if 0
		ret = trout_fm_iis_pin_cfg();
		if(ret < 0)
		{
			TROUT_PRINT("trout_fm_iis_pin_cfg failed!");
			return ret;
		}
#endif
	}

#if 0
	/* set trout wifi goto sleep */
	#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	wifimac_sleep();
	#endif
#endif
	atomic_cmpxchg(&is_fm_open, 0, 1);

	TROUT_PRINT("Open fm module success.");

	trout_fm_set_volume(7);

	return 0;
}


static int _trout_fm_release(struct file *file)
{
	TROUT_PRINT("trout_fm_release");

	trout_fm_deinit();

    atomic_cmpxchg(&is_fm_open, 1, 0);

	return 0;
}

static int _trout_fm_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, CARD_NAME, sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 1, 0);
	cap->capabilities = V4L2_CAP_RADIO | V4L2_CAP_TUNER | V4L2_CAP_HW_FREQ_SEEK;
	return 0;
}

static int _trout_fm_get_tuner(struct file *file, void *priv, struct v4l2_tuner *tuner)
{
	strlcpy(tuner->name, "FM", sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
	tuner->rangelow = 1400000;
	tuner->rangehigh = 1728000;
	memset(tuner->reserved, 0, sizeof(tuner->reserved));
	return -EINVAL;
}

static int _trout_fm_set_tuner(struct file *file, void *priv, const struct v4l2_tuner *tuner)
{
	return -EINVAL;
}

static int _trout_fm_get_freq(struct file *file, void *priv, struct v4l2_frequency *freq)
{
	u16 ifreq = 0;
	int err = trout_fm_get_frequency(&ifreq);
	if(err) {
		TROUT_PRINT("trout_fm_get_frequency error.");
		return -EINVAL;
	}

	freq->frequency = ((u32)ifreq) * FM_FREQ_CONVERSION;
	TROUT_PRINT("_trout_fm_get_freq %d",freq->frequency);
	freq->type = V4L2_TUNER_RADIO;
	memset(freq->reserved, 0, sizeof(freq->reserved));
	return err;
}

static int _trout_fm_set_freq(struct file *file, void *priv, const struct v4l2_frequency *freq)
{
	int ret;
	u16 ifreq = (freq->frequency) / FM_FREQ_CONVERSION;
	TROUT_PRINT("_trout_fm_set_freq %d",ifreq);
	ret = trout_fm_set_tune(ifreq);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	wifimac_sleep();//hugh:add for power 20130425
#endif
	return ret;
}
static int _trout_fm_seek(struct file *file, void *priv, struct v4l2_hw_freq_seek *seek)
{
	u16 freq = 0;
	u16 reserved = 0;
	u8 direction = seek->seek_upward;
	int err;

	TROUT_PRINT("_trout_fm_seek Start !");

	err = trout_fm_get_frequency((u16*)(&freq));
	if(err) {
		TROUT_PRINT("trout_fm_seek error due to get_frequency error.");
		return -EINVAL;
	}

	err = trout_fm_seek(freq, /* start frequency */
		direction, /* seek direction*/
		3000, /* time out */
		&reserved);

	if(err){
		return -EINVAL;
		TROUT_PRINT("_trout_fm_seek Error !");
	}
	seek->reserved[0] = reserved * FM_FREQ_CONVERSION;

	trout_fm_get_frequency(&freq);
	trout_fm_set_tune(freq);

	TROUT_PRINT("_trout_fm_seek Finish !");

	return 0;
}

static const struct v4l2_file_operations trout_fops = {
	.owner = THIS_MODULE,
	.open = _trout_fm_open,
	.release = _trout_fm_release,
	.unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops trout_ioctl_ops = {
	.vidioc_querycap = _trout_fm_querycap,
	.vidioc_g_tuner = _trout_fm_get_tuner,
	.vidioc_s_tuner = _trout_fm_set_tuner,
	.vidioc_g_frequency = _trout_fm_get_freq,
	.vidioc_s_frequency = _trout_fm_set_freq,
	.vidioc_s_hw_freq_seek = _trout_fm_seek,
};

static const struct v4l2_ctrl_ops trout_ctrl_ops = {
 // .s_ctrl = _trout_fm_control,
};

static int register_v4l2_device(void)
{
	int ret;
	struct fmdev *fmdev = NULL;
	struct video_device *radio = video_device_alloc();
	if (!radio) {
		TROUT_PRINT("Could not allocate video_device");
		return -EINVAL;
	}
	strlcpy(radio->name, DRIVER_NAME, sizeof(radio->name));
	radio->fops = &trout_fops;
	radio->release = video_device_release;
	radio->ioctl_ops = &trout_ioctl_ops;
	if (video_register_device(radio, VFL_TYPE_RADIO, -1)) {
		TROUT_PRINT("Could not register video_device");
		video_device_release(radio);
		return -EINVAL;
	}

	s_radio = radio;

	fmdev = (struct fmdev *)kzalloc(sizeof(struct fmdev), GFP_KERNEL);
	if (!fmdev) {
		TROUT_PRINT("Could not allocate fmdev");
		return -EINVAL;
	}

	video_set_drvdata(radio, fmdev);
	radio->ctrl_handler = &fmdev->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&fmdev->ctrl_handler, 1);
	if (ret < 0) {
		TROUT_PRINT("Failed to int v4l2_ctrl_handler");
		v4l2_ctrl_handler_free(&fmdev->ctrl_handler);
		return -EINVAL;
	}

	v4l2_ctrl_new_std(&fmdev->ctrl_handler, &trout_ctrl_ops, V4L2_CID_PRIVATE_FM_AUDIO, 0, 1, 1, 0);

	TROUT_PRINT("Registered Trout FM Receiver.");
	return 0;
}

static int  trout_fm_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	char *ver_str = TROUT_FM_VERSION;

	p_trout_interface = NULL;

	TROUT_PRINT("**********************************************");
	TROUT_PRINT(" Trout FM driver ");
	TROUT_PRINT(" Version: %s", ver_str);
	TROUT_PRINT(" Build date: %s %s", __DATE__, __TIME__);
	TROUT_PRINT("**********************************************");

	trout_onchip_init(&p_trout_interface);

	if(p_trout_interface == NULL) {
		TROUT_PRINT("none interface used!");
		return ret;
	}

	TROUT_PRINT("use %s interface.", p_trout_interface->name);

	ret = p_trout_interface->init();
	if(ret < 0) {
		TROUT_PRINT("interface init failed!");
		return ret;
	}

	ret = register_v4l2_device();
	if(ret < 0) {
		TROUT_PRINT("register_v4l2_device failed!");
		return ret;
	}

	TROUT_PRINT("trout_fm_init success.\n");

	return 0;
}

static int trout_fm_remove(struct platform_device *pdev)
{
	struct fmdev *fmdev;
	TROUT_PRINT("exit_fm_driver!\n");
	if(s_radio) {
		fmdev = video_get_drvdata(s_radio);
		if (fmdev) {
			v4l2_ctrl_handler_free(&fmdev->ctrl_handler);
			kfree(fmdev);
		}
		video_unregister_device(s_radio);
		s_radio = NULL;
	}

	if(p_trout_interface) {
		p_trout_interface->exit();
		p_trout_interface = NULL;
	}

    return 0;
}

#ifdef CONFIG_PM
static int trout_fm_suspend(struct platform_device *dev, pm_message_t state)
{
	trout_fm_enter_sleep();
	return 0;
}

static int trout_fm_resume(struct platform_device *dev)
{
	trout_fm_exit_sleep();
	return 0;
}
#else
#define trout_fmt_suspend NULL
#define trout_fm_resume NULL
#endif

static struct platform_driver trout_fm_driver = {
		.driver = {
		.name = "trout_fm",
		.owner = THIS_MODULE,
	},
	.probe = trout_fm_probe,
	.remove = trout_fm_remove,
	.suspend = trout_fm_suspend,
	.resume = trout_fm_resume,
};

int __init init_fm_driver(void)
{
    int ret;

    ret = platform_driver_register(&trout_fm_driver);
    if (ret)
        TROUT_PRINT("trout_fm: register driver failed: %d\n", ret);
    else
        TROUT_PRINT("trout_fm: register driver success !\n");

    return ret;
}

void __exit exit_fm_driver(void)
{
    platform_driver_unregister(&trout_fm_driver);
}


module_init(init_fm_driver);
module_exit(exit_fm_driver);

MODULE_DESCRIPTION("TROUT FM radio driver");
MODULE_AUTHOR("Spreadtrum Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(TROUT_FM_VERSION);

