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
#define V4L2_CID_PRIVATE_FM_MUTE (V4L2_CID_PRIVATE_BASE + 1)


static atomic_t  s_opened;
static int s_mute = 0;
static struct video_device *s_radio = NULL;
struct trout_interface *p_trout_interface = NULL;

static int _trout_fm_mute(int mute)
{
    s_mute = mute;
    if (mute == 1) {
        trout_fm_mute();
    } else {
        trout_fm_unmute();
    }
    return 0;
}

static int _trout_fm_open(struct file *file)
{
	int ret = -EINVAL;
  int status;

	TROUT_PRINT("start open fm module...");

	if (atomic_inc_return(&s_opened) > 1) {
		TROUT_PRINT("trout_fm has been opened, ignore this operation.");
		return 0;
	}
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

	if (status) {
		TROUT_PRINT("trout fm have been opened.");
	} else {
		ret = trout_fm_en();
		if(ret < 0) {
			TROUT_PRINT("trout_fm_en failed!");
			return ret;
		}
	}

	TROUT_PRINT("Open fm module success.");

	return 0;
}


static int _trout_fm_release(struct file *file)
{
	TROUT_PRINT("trout_fm_release");

	if (atomic_dec_return(&s_opened) > 0) {
		TROUT_PRINT("trout_fm has been opened by others.");
		return 0;
	}

	trout_fm_deinit();

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
	/** TODO **/
	/*strlcpy(tuner->name, "FM", sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
	tuner->rangelow = 1400000;
	tuner->rangehigh = 1728000;
	memset(tuner->reserved, 0, sizeof(tuner->reserved));*/
	return -EINVAL;
}

static int _trout_fm_set_tuner(struct file *file, void *priv, const struct v4l2_tuner *tuner)
{
	/** TODO **/
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
	ret = trout_fm_set_tune(ifreq);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	wifimac_sleep();//hugh:add for power 20130425
#endif
	return ret;
}

static int _trout_fm_seek(struct file *file, void *priv, const struct v4l2_hw_freq_seek *seek)
{
	u16 freq = 0;
	u16 reserved = 0;
	u8 direction = seek->seek_upward;
	int err;

	err = trout_fm_get_frequency((u16*)(&freq));
	if(err) {
		TROUT_PRINT("trout_fm_seek error due to get_frequency error.");
		return -EINVAL;
	}

	int mute = s_mute;
	if (mute != 1) {
		_trout_fm_mute(1);
	}
	err = trout_fm_seek(freq, /* start frequency */
		direction, /* seek direction*/
		3000, /* time out */
		&reserved);
	if (err) {
		TROUT_PRINT("_trout_fm_seek Error !");
	}
	if (mute != 1) {
		_trout_fm_mute(mute);
	}

	return err ? -EINVAL : 0;
}

static int _trout_fm_control(struct file *file, void *priv, struct v4l2_control *ctrl) {
    switch (ctrl->id) {
		    case V4L2_CID_PRIVATE_FM_MUTE:
		        return _trout_fm_mute(ctrl->value);
		    default:
		        TROUT_PRINT("Unknown fm control.");
		        break;
		}
    return -1;
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
	.vidioc_s_ctrl = _trout_fm_control,
};

static int register_v4l2_device(void)
{
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
	TROUT_PRINT("exit_fm_driver!\n");
	if(s_radio) {
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

