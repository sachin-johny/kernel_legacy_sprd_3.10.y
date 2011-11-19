/*
* drivers/media/video/sprd_dcam/dcam_v4l2_sc8800g2.c
 * Dcam driver based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/highmem.h>
#include <linux/freezer.h>
#include <media/videobuf-vmalloc.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

//for read 
#include <linux/io.h>

#include "dcam_sc8800g2.h"
#include "sensor_drv.h"

//for debug+++++start
#define DCAM_V4L2_DEBUG 1
#if DCAM_V4L2_DEBUG
uint32_t g_address = 0xc0000000;
#endif
#if DCAM_V4L2_DEBUG
#define DCAM_V4L2_PRINT printk 
#else
#define DCAM_V4L2_PRINT(...)
#endif
uint32_t g_buffer_addr = 0;
//for debug+++++end

#define DCAM_MINOR MISC_DYNAMIC_MINOR
static struct mutex *lock;

typedef struct dcam_info
{
	//Sensor
	SENSOR_MODE_E preview_m;
	SENSOR_MODE_E snapshot_m;
	//Dcam
	DCAM_SIZE_T0 input_size;
	DCAM_MODE_TYPE_E mode;
	uint32_t orientation; //wxz20110815: 0: default, landscape; 1: portrait
}DCAM_INFO_T;

uint32_t g_cur_buf_addr = 0; //store the buffer address which is hte next buffer DQbufed.
uint32_t g_first_buf_addr = 0; //store the first buffer address
uint32_t g_last_buf = 0xFFFFFFFF;//record the last buffer for dcam driver
struct dcam_fh *g_fh = NULL; //store the fh pointer for ISR callback function
static uint32_t g_is_first_frame = 1; //store the flag for the first frame
//struct dcam_buffer *g_dcam_buf_ptr = NULL;
DCAM_INFO_T g_dcam_info; //store the dcam and sensor config info
uint32_t g_zoom_level = 0; //zoom level: 0: 1x, 1: 2x, 2: 3x, 3: 4x
uint32_t g_is_first_irq = 1; 

#define DCAM_MODULE_NAME "dcam"
#define WAKE_NUMERATOR 30
#define WAKE_DENOMINATOR 1001

#define DCAM_MAJOR_VERSION 0
#define DCAM_MINOR_VERSION 6
#define DCAM_RELEASE 0
#define DCAM_VERSION \
	KERNEL_VERSION(DCAM_MAJOR_VERSION, DCAM_MINOR_VERSION, DCAM_RELEASE)
#ifdef CONFIG_CAMERA_2M
#define norm_maxw() 1600
#define norm_maxh() 1200
#else
#define norm_maxw() 2048
#define norm_maxh() 1536
#endif

MODULE_DESCRIPTION("Video Technology Magazine Virtual Video Capture Board");
MODULE_AUTHOR("Mauro Carvalho Chehab, Ted Walther and John Sokol");
MODULE_LICENSE("Dual BSD/GPL");

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

static unsigned n_devs = 1;
module_param(n_devs, uint, 0644);
MODULE_PARM_DESC(n_devs, "number of video devices to create");

static unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

static unsigned int vid_limit = 16;
module_param(vid_limit, uint, 0644);
MODULE_PARM_DESC(vid_limit, "capture memory limit in megabytes");

/* R   G   B */
#define COLOR_WHITE	{204, 204, 204}
#define COLOR_AMBAR	{208, 208,   0}
#define COLOR_CIAN	{  0, 206, 206}
#define COLOR_GREEN	{  0, 239,   0}
#define COLOR_MAGENTA	{239,   0, 239}
#define COLOR_RED	{205,   0,   0}
#define COLOR_BLUE	{  0,   0, 255}
#define COLOR_BLACK	{  0,   0,   0}

struct bar_std {
	u8 bar[8][3];
};

/* Maximum number of bars are 10 - otherwise, the input print code
   should be modified */
static struct bar_std bars[] = {
	{	/* Standard ITU-R color bar sequence */
		{
			COLOR_WHITE,
			COLOR_AMBAR,
			COLOR_CIAN,
			COLOR_GREEN,
			COLOR_MAGENTA,
			COLOR_RED,
			COLOR_BLUE,
			COLOR_BLACK,
		}
	}, {
		{
			COLOR_WHITE,
			COLOR_AMBAR,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_AMBAR,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_AMBAR,
		}
	}, {
		{
			COLOR_WHITE,
			COLOR_CIAN,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_CIAN,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_CIAN,
		}
	}, {
		{
			COLOR_WHITE,
			COLOR_GREEN,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_GREEN,
			COLOR_BLACK,
			COLOR_WHITE,
			COLOR_GREEN,
		}
	},
};

#define NUM_INPUTS ARRAY_SIZE(bars)

#define TO_Y(r, g, b) \
	(((16829 * r + 33039 * g + 6416 * b  + 32768) >> 16) + 16)
/* RGB to  V(Cr) Color transform */
#define TO_V(r, g, b) \
	(((28784 * r - 24103 * g - 4681 * b  + 32768) >> 16) + 128)
/* RGB to  U(Cb) Color transform */
#define TO_U(r, g, b) \
	(((-9714 * r - 19070 * g + 28784 * b + 32768) >> 16) + 128)
	
/* supported controls */
static struct v4l2_queryctrl dcam_qctrl[] = {
	{
		.id            = V4L2_CID_AUDIO_VOLUME,
		.name          = "Volume",
		.minimum       = 0,
		.maximum       = 65535,
		.step          = 65535/100,
		.default_value = 65535,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	}, {
		.id            = V4L2_CID_BRIGHTNESS,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Brightness",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 127,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_CONTRAST,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Contrast",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0x10,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_SATURATION,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Saturation",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 127,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_HUE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Hue",
		.minimum       = -128,
		.maximum       = 127,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_DO_WHITE_BALANCE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "whitebalance",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_COLORFX,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "coloreffect",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_COLOR_KILLER,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "scenemode",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_ZOOM_ABSOLUTE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "zoom",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_HFLIP,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "hmirror",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_VFLIP,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "vmirror",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}
};

#define dprintk(dev, level, fmt, arg...)  v4l2_printk(KERN_DEBUG, &dev->v4l2_dev, fmt , ## arg)
	//v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

/* ------------------------------------------------------------------
	Basic structures
   ------------------------------------------------------------------*/

struct dcam_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	int   depth;
	int flag; // 0:qbuf; 1: set driver 
};

static struct dcam_fmt formats[] = {
	{
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, YUV422",
		.fourcc   = V4L2_PIX_FMT_YUV422P,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, UYVY",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.depth    = 16,
	},
	{
		.name     = "4:2:0, packed, YUV",
		.fourcc   = V4L2_PIX_FMT_YUV420,
		.depth    = 16,
	},
	{
		.name     = "RGB565 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
		.depth    = 16,
	},
	{
		.name     = "RGB565 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
		.depth    = 16,
	},
	{
		.name     = "RGB555 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB555, /* gggbbbbb arrrrrgg */
		.depth    = 16,
	},
	{
		.name     = "RGB555 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB555X, /* arrrrrgg gggbbbbb */
		.depth    = 16,
	},
	{
		.name     = "RGB8888",
		.fourcc   = V4L2_PIX_FMT_RGB32, /* aaaarrrrrgg gggbbbbb */
		.depth    = 32,
	},
};

/* buffer for one video frame */
struct dcam_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	struct dcam_fmt        *fmt;
};

struct dcam_dmaqueue {
	struct list_head       active;

	/* thread for generating video stream*/
	struct task_struct         *kthread;
	wait_queue_head_t          wq;
	/* Counters to control fps rate */
	int                        frame;
	int                        ini_jiffies;
};

static LIST_HEAD(dcam_devlist);

struct dcam_dev {
	struct list_head           dcam_devlist;
	struct v4l2_device 	   v4l2_dev;

	spinlock_t                 slock;
	struct mutex		   mutex;

	int                        users;

	/* various device info */
	struct video_device        *vfd;

	struct dcam_dmaqueue       vidq;

	/* Several counters */
	int                        h, m, s, ms;
	unsigned long              jiffies;
	char                       timestr[13];

	int			   mv_count;	/* Controls bars movement */

	/* Input Number */
	int			   input;

	/* Control 'registers' */
	int 			   qctl_regs[ARRAY_SIZE(dcam_qctrl)];

	struct v4l2_streamparm streamparm;
};

struct dcam_fh {
	struct dcam_dev            *dev;

	/* video capture */
	struct dcam_fmt            *fmt;
	unsigned int               width, height;
	struct videobuf_queue      vb_vidq;

	enum v4l2_buf_type         type;
	unsigned char              bars[8][3];
	int			   input; 	/* Input Number on bars */
};


static int init_sensor_parameters(void)
{
	uint32_t i,width;
	SENSOR_EXP_INFO_T *sensor_info_ptr = NULL;
	DCAM_V4L2_PRINT("###V4L2: init sensor parameters E.\n");
	sensor_info_ptr = Sensor_GetInfo();

	for(i = SENSOR_MODE_PREVIEW_ONE; i < SENSOR_MODE_MAX; i++)
	{
		width = sensor_info_ptr->sensor_mode_info[i].width;
		if(g_dcam_info.input_size.w <= width)
		{
			g_dcam_info.snapshot_m = sensor_info_ptr->sensor_mode_info[i].mode;
			if(g_dcam_info.snapshot_m < SENSOR_MODE_PREVIEW_TWO)
				g_dcam_info.preview_m = SENSOR_MODE_PREVIEW_ONE;
			else
				g_dcam_info.preview_m = SENSOR_MODE_PREVIEW_TWO;
			//height = sensor_info_ptr->sensor_mode_info[i].height;
			break;				
		}
	}
	DCAM_V4L2_PRINT("###V4L2: snapshot_m: %d, preview_m: %d.\n", g_dcam_info.snapshot_m, g_dcam_info.preview_m);
	if(g_dcam_info.preview_m != g_dcam_info.snapshot_m){//wxz20111110: for capture mode
		Sensor_Ioctl(SENSOR_IOCTL_BEFORE_SNAPSHOT, (uint32_t)g_dcam_info.snapshot_m);
	}
	else if(g_dcam_info.snapshot_m < SENSOR_MODE_SNAPSHOT_ONE_FIRST){ //for preview mode
		Sensor_Ioctl(SENSOR_IOCTL_AFTER_SNAPSHOT, (uint32_t)g_dcam_info.preview_m);
	}
	return 0;
}


static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct dcam_fh  *fh  = priv;
	struct dcam_dev *dev = fh->dev;

	strcpy(cap->driver, "dcam");
	strcpy(cap->card, "dcam");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = DCAM_VERSION;
	cap->capabilities =	V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_STREAMING     |
				V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_cropcap(struct file *file, void  *priv,
					struct v4l2_cropcap *cc)
{
	//struct dcam_fh *fh = priv;
	//struct dcam_dev *dev = fh->dev;

	if (cc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	cc->bounds.left = 0;
	cc->bounds.top = 0;
	cc->bounds.width = 2560;
	cc->bounds.height = 2048;

	cc->defrect = cc->bounds;

	cc->pixelaspect.numerator = 54;
	cc->pixelaspect.denominator = 59;

	return 0;
}

static int vidioc_s_crop(struct file *file, void  *priv,
					struct v4l2_crop *crop)
{
	//struct dcam_fh *fh = priv;
	//struct dcam_dev *dev = fh->dev;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	DCAM_V4L2_PRINT("###V4L2: vidioc_s_crop left: %d, top: %d, width: %d,height: %d.\n", crop->c.left, crop->c.top, crop->c.width, crop->c.height);

	return 0;
}
static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct dcam_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dcam_fh *fh = priv;

	f->fmt.pix.width        = fh->width;
	f->fmt.pix.height       = fh->height;
	f->fmt.pix.field        = fh->vb_vidq.field;
	f->fmt.pix.pixelformat  = fh->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fh->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return (0);
}

static struct dcam_fmt *get_format(struct v4l2_format *f)
{
	struct dcam_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == ARRAY_SIZE(formats))
		return NULL;

	return &formats[k];
}


static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct dcam_fh  *fh  = priv;
	struct dcam_dev *dev = fh->dev;
	struct dcam_fmt *fmt;
	enum v4l2_field field;
	unsigned int maxw, maxh;

	fmt = get_format(f);
	if (!fmt) {
		dprintk(dev, 1, "Fourcc format (0x%08x) invalid.\n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY) {
		field = V4L2_FIELD_INTERLACED;
	} else if (V4L2_FIELD_INTERLACED != field) {
		dprintk(dev, 1, "Field type invalid.\n");
		return -EINVAL;
	}

	maxw  = norm_maxw();
	maxh  = norm_maxh();

	f->fmt.pix.field = field;
	v4l_bound_align_image(&f->fmt.pix.width, 48, maxw, 2,
			      &f->fmt.pix.height, 32, maxh, 0, 0);
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

/*FIXME: This seems to be generic enough to be at videodev2 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dcam_fh *fh = priv;
	struct videobuf_queue *q = &fh->vb_vidq;

	int ret = vidioc_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
		return ret;

	mutex_lock(&q->vb_lock);
	if (videobuf_queue_is_busy(&fh->vb_vidq)) {
		dprintk(fh->dev, 1, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	fh->fmt           = get_format(f);
	fh->width         = f->fmt.pix.width;
	fh->height        = f->fmt.pix.height;
	fh->vb_vidq.field = f->fmt.pix.field;
	fh->type          = f->type;

	ret = 0;

out:
	mutex_unlock(&q->vb_lock);

	return ret;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id *i)
{
	return 0;
}

/* only one input in this sample driver */
static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index >= NUM_INPUTS)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = V4L2_STD_525_60;
	sprintf(inp->name, "Camera %u", inp->index);

	return (0);
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;

	*i = dev->input;

	return (0);
}

/* precalculate color bar values to speed up rendering */
static void precalculate_bars(struct dcam_fh *fh)
{
	struct dcam_dev *dev = fh->dev;
	unsigned char r, g, b;
	int k, is_yuv;

	fh->input = dev->input;

	for (k = 0; k < 8; k++) {
		r = bars[fh->input].bar[k][0];
		g = bars[fh->input].bar[k][1];
		b = bars[fh->input].bar[k][2];
		is_yuv = 0;

		switch (fh->fmt->fourcc) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
			is_yuv = 1;
			break;
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
			r >>= 3;
			g >>= 2;
			b >>= 3;
			break;
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB555X:
			r >>= 3;
			g >>= 3;
			b >>= 3;
			break;
		}

		if (is_yuv) {
			fh->bars[k][0] = TO_Y(r, g, b);	/* Luma */
			fh->bars[k][1] = TO_U(r, g, b);	/* Cb */
			fh->bars[k][2] = TO_V(r, g, b);	/* Cr */
		} else {
			fh->bars[k][0] = r;
			fh->bars[k][1] = g;
			fh->bars[k][2] = b;
		}
	}

}
static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;

	if (i >= NUM_INPUTS)
		return -EINVAL;

	dev->input = i;
	precalculate_bars(fh);

	return (0);
}

/* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dcam_qctrl); i++)
		if (qc->id && qc->id == dcam_qctrl[i].id) {
			memcpy(qc, &(dcam_qctrl[i]),
				sizeof(*qc));
			return (0);
		}

	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(dcam_qctrl); i++)
		if (ctrl->id == dcam_qctrl[i].id) {
			ctrl->value = dev->qctl_regs[i];
			return 0;
		}

	return -EINVAL;
}
static int vidioc_handle_ctrl(struct v4l2_control *ctrl)	
{
	DCAM_V4L2_PRINT("vidioc_handle_ctrl, id: %d, value: %d.\n", ctrl->id, ctrl->value);
	
	switch(ctrl->id)
	{
		case V4L2_CID_DO_WHITE_BALANCE:
			Sensor_Ioctl(SENSOR_IOCTL_SET_WB_MODE, (uint32_t)ctrl->value);
			break;
		case V4L2_CID_COLORFX:
			Sensor_Ioctl(SENSOR_IOCTL_IMAGE_EFFECT, (uint32_t)ctrl->value);
			break;
		case V4L2_CID_COLOR_KILLER:  //for scene mode			
			Sensor_Ioctl(SENSOR_IOCTL_PREVIEWMODE, (uint32_t)ctrl->value);
			break;	
		case V4L2_CID_BRIGHTNESS:  		
			Sensor_Ioctl(SENSOR_IOCTL_BRIGHTNESS, (uint32_t)ctrl->value);
			break;		
		case V4L2_CID_CONTRAST:  		
			Sensor_Ioctl(SENSOR_IOCTL_CONTRAST, (uint32_t)ctrl->value);
			break;				
		case V4L2_CID_ZOOM_ABSOLUTE:
			g_zoom_level = (uint32_t)ctrl->value;
			DCAM_V4L2_PRINT("g_zoom_level=%d.\n", g_zoom_level);
			break;			
		case V4L2_CID_HFLIP:  		
			Sensor_Ioctl(SENSOR_IOCTL_HMIRROR_ENABLE, (uint32_t)ctrl->value);	
			break;
		case V4L2_CID_VFLIP:  		
			Sensor_Ioctl(SENSOR_IOCTL_VMIRROR_ENABLE, (uint32_t)ctrl->value);	
			break;
		default:
			break;
	}
	
	return 0;
}
static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(dcam_qctrl); i++)
		if (ctrl->id == dcam_qctrl[i].id) {
			if (ctrl->value < dcam_qctrl[i].minimum ||
			    ctrl->value > dcam_qctrl[i].maximum) {
				return -ERANGE;
			}
			dev->qctl_regs[i] = ctrl->value;
			vidioc_handle_ctrl(ctrl);
			return 0;
		}
	return -EINVAL;
}

static int vidioc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *streamparm)
{	
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;
	int i;
	DCAM_V4L2_PRINT("###V4L2: vidioc_g_parm E.\n");
	streamparm->type = dev->streamparm.type;
	for(i = 0; i < 7; i++){
		streamparm->parm.raw_data[i] = dev->streamparm.parm.raw_data[i];
	}
	/*streamparm->parm.capture.capability = dev->streamparm.parm.capture.capability;
	streamparm->parm.capture.capturemode = dev->streamparm.parm.capture.capturemode;
	streamparm->parm.capture.timeperframe.numerator = dev->streamparm.parm.capture.timeperframe.numerator;
	streamparm->parm.capture.timeperframe.denominator = dev->streamparm.parm.capture.timeperframe.denominator;
	streamparm->parm.capture.extendedmode = dev->streamparm.parm.capture.extendedmode;	
	streamparm->parm.capture.readbuffers = dev->streamparm.parm.capture.readbuffers;
	for(i = 0; i < 4; i++)
		streamparm->parm.capture.reserved[i] = dev->streamparm.parm.capture.reserved[i];
	streamparm->parm.raw_data[0] = dev->streamparm.parm.raw_data[0];
	streamparm->parm.raw_data[1] = dev->streamparm.parm.raw_data[1];
	streamparm->parm.raw_data[2] = dev->streamparm.parm.raw_data[2];//wxz20110815: the orientation info
	*/
	DCAM_V4L2_PRINT("###V4L2: vidioc_g_parm X.\n");
	return 0;
}
static int v4l2_sensor_init(SENSOR_CONFIG_T *sensor_config)
{
	//init sensor
	if(SENSOR_TRUE != Sensor_IsInit()){		
		if(SENSOR_SUCCESS != Sensor_Init(sensor_config)){
			if(1 == sensor_config->set_sensor_num){//wxz20111021: HAL set sensor number, but fail. So try again.
				sensor_config->set_sensor_num = 2;
				sensor_config->sensor_num = 0xEE;				
				DCAM_V4L2_PRINT("###V4L2: HAL set sensor number, but fail. So try again.\n");
				if(SENSOR_SUCCESS != Sensor_Init(sensor_config)){
					DCAM_V4L2_PRINT("###DCAM: Fail to init sensor.\n");
					return -1;
				}
				else{
					DCAM_V4L2_PRINT("###DCAM: OK to init sensor; driver find the sensor number: %d.\n", sensor_config->sensor_num);
				}
			}
			else{
				DCAM_V4L2_PRINT("###DCAM: Fail to init sensor.\n");
				return -1;
			}
		}
		//init_sensor_parameters();
	}
	DCAM_V4L2_PRINT("###DCAM: OK to init sensor.\n");	

	return 0;
}
static int vidioc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *streamparm)
{	
	struct dcam_fh *fh = priv;
	struct dcam_dev *dev = fh->dev;
	int i;
	//uint32_t sensor_id = 0;
	SENSOR_CONFIG_T sensor_config;
	
	DCAM_V4L2_PRINT("###V4L2: vidioc_s_parm E.\n");
	dev->streamparm.type = streamparm->type;
	/*dev->streamparm.parm.capture.capability = streamparm->parm.capture.capability;
	dev->streamparm.parm.capture.capturemode = streamparm->parm.capture.capturemode;
	dev->streamparm.parm.capture.timeperframe.numerator = streamparm->parm.capture.timeperframe.numerator;
	dev->streamparm.parm.capture.timeperframe.denominator = streamparm->parm.capture.timeperframe.denominator;
	dev->streamparm.parm.capture.extendedmode = streamparm->parm.capture.extendedmode;	
	dev->streamparm.parm.capture.readbuffers = streamparm->parm.capture.readbuffers;
	for(i = 0; i < 4; i++)
		dev->streamparm.parm.capture.reserved[i] = streamparm->parm.capture.reserved[i];
	*/

	//wxz20111021: raw_data
	//BYTE0: 0: HAL doesn't set sensor_id; 1: HAL sets sensor_id;
	//BYTE1: 0: back camera if BYTE0 is 1; 1: front camera if BYTE0 is 1;
	//BYTE2: 0: no crop and rotation; 1: crop and rotation;
	//BYTE3: 0: HAL doesn't set sensor number; 1: HAL sets sensor number; 2: need to update the sensor number;
	//BYTE4: back sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	//BYTE5: front sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;	     //BYTE6: 0: preview mode; 1: capture mode;
	if(1 == streamparm->parm.raw_data[0])
	{
		if(0 == streamparm->parm.raw_data[1])
		{
			//Sensor_SetCurId(SENSOR_MAIN);
			sensor_config.sensor_id = 0;
		}
		else if(1 == streamparm->parm.raw_data[1])
		{
			//Sensor_SetCurId(SENSOR_SUB);
			sensor_config.sensor_id = 1;
		}			
	}
	else{
		//Sensor_SetCurId(SENSOR_MAIN);
		sensor_config.sensor_id = 0;
	}
	if(1 == streamparm->parm.raw_data[2]){
		g_dcam_info.orientation = 1;	
	}
	else{
		g_dcam_info.orientation = 0;	
	}
	//wxz20111021: handle the sensor number
	if(1 == streamparm->parm.raw_data[3]){
		sensor_config.set_sensor_num = 1;
		if(0 == sensor_config.sensor_id){
			sensor_config.sensor_num = streamparm->parm.raw_data[4];			
		}
		else if(1 == sensor_config.sensor_id){
			sensor_config.sensor_num = streamparm->parm.raw_data[5];			
		}
	}
	else{
		sensor_config.set_sensor_num = 0;
		sensor_config.sensor_num = 0;	
	}	
	//sensor_config.set_sensor_num = 0;
	DCAM_V4L2_PRINT("###V4L2: before v4l2_sensor_init. set: %d, num: %d, id: %d.\n", sensor_config.set_sensor_num, sensor_config.sensor_num, sensor_config.sensor_id);
	if(0 != v4l2_sensor_init(&sensor_config)){
		DCAM_V4L2_PRINT("###V4L2: fail to sensor_init.\n");
		DCAM_V4L2_PRINT("###V4L2: fail to sensor_init, sensor id: %d; to init another sensor.\n", sensor_config.sensor_id);
		//the sensor is bad for init.  set the number 0xFF 
		if(0 == sensor_config.sensor_id){
			streamparm->parm.raw_data[4] = 0xFC;			
		}
		else if(1 == sensor_config.sensor_id){
			streamparm->parm.raw_data[5] = 0xFD;				
		}
		sensor_config.sensor_id = sensor_config.sensor_id == 0 ? 1 : 0;
		//update the sensor number for another sensor.
		if(0 == sensor_config.sensor_id){
			sensor_config.sensor_num = streamparm->parm.raw_data[4];			
		}
		else if(1 == sensor_config.sensor_id){
			sensor_config.sensor_num = streamparm->parm.raw_data[5];			
		}
		if(0 != v4l2_sensor_init(&sensor_config)){
			//the sensor is bad for init.  set the number 0xFF 
			if(0 == sensor_config.sensor_id){
				streamparm->parm.raw_data[4] = 0xFA;			
			}
			else if(1 == sensor_config.sensor_id){
				streamparm->parm.raw_data[5] = 0xFB;				
			}
			DCAM_V4L2_PRINT("###V4L2: fail to sensor_init, sensor id: %d; two sensor are fail.\n", sensor_config.sensor_id);
			return -1;
		}
		if(1 == streamparm->parm.raw_data[0]){
			streamparm->parm.raw_data[1] = sensor_config.sensor_id;			
			DCAM_V4L2_PRINT("###V4L2: the old sensor fail, to change another sensor: %d.\n", sensor_config.sensor_id);
		}		
	}
	if(2 == sensor_config.set_sensor_num){//need to update the sensor number
		streamparm->parm.raw_data[3] = 2;
		DCAM_V4L2_PRINT("###V4L2: update new sensor number, id: %d, num: %d.", sensor_config.sensor_id, sensor_config.sensor_num);
		if(0 == sensor_config.sensor_id){
			streamparm->parm.raw_data[4] = sensor_config.sensor_num;			
		}
		else if(1 == sensor_config.sensor_id){
			streamparm->parm.raw_data[5] = sensor_config.sensor_num;				
		}
	}
	for(i = 0; i < 7; i++){
		dev->streamparm.parm.raw_data[i] = streamparm->parm.raw_data[i];
	}

	DCAM_V4L2_PRINT("###V4L2: vidioc_s_parm X.\n");
	return 0;
}
static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct dcam_fh  *fh = priv;

	//p->memory = V4L2_MEMORY_USERPTR;

	return (videobuf_reqbufs(&fh->vb_vidq, p));
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dcam_fh  *fh = priv;
	
	return (videobuf_querybuf(&fh->vb_vidq, p));
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dcam_fh *fh = priv;

	if(1 == g_is_first_frame)
	{
		g_first_buf_addr = p->m.userptr;
		g_is_first_frame = 0;
		DCAM_V4L2_PRINT("###V4L2: g_first_buf_addr: %x.\n", g_first_buf_addr);
	}
	//printk("###V4L2: vidioc_qbuf addr: %x.\n", p->m.userptr);
	return (videobuf_qbuf(&fh->vb_vidq, p));
}
/*
static inline int rt_policy(int policy)
{
	if (unlikely(policy == SCHED_FIFO) || unlikely(policy == SCHED_RR))
		return 1;
	return 0;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}
*/
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dcam_fh  *fh = priv;
	/*
	//wxz20110725: adjust the priority of the thread.
	static int flag = 0; 
	if(flag == 0){
		flag = 1;
		printk("###wxz: adjust the priority to 2.\n");
	if (!task_has_rt_policy(current)) {
		    struct sched_param schedpar;
		    int ret;
		    struct cred *new = prepare_creds();
		    cap_raise(new->cap_effective, CAP_SYS_NICE);
		    commit_creds(new);
		    schedpar.sched_priority = 22;
		    ret = sched_setscheduler(current,SCHED_RR,&schedpar);
		    if(ret!=0)
		        printk("vsp change pri fail a\n");
	}
	}*/
	DCAM_V4L2_PRINT("###v4l2: vidioc_dqbuf: file->f_flags: %x,  O_NONBLOCK: %x, g_dcam_info.mode: %d.\n", file->f_flags, O_NONBLOCK, g_dcam_info.mode);
	return (videobuf_dqbuf(&fh->vb_vidq, p, file->f_flags & O_NONBLOCK));
	//videobuf_dqbuf(&fh->vb_vidq, p, file->f_flags & O_NONBLOCK);
	//printk("###V4L2: vidioc_ddddqbuf addr: %x.\n", p->m.userptr);
	//return 0;
}
#if 0


static void init_dcam_parameters(void *priv)
{
	struct dcam_fh  *fh = priv;
	struct dcam_dev *dev = fh->dev;
	DCAM_TRIM_RECT_T trim_rect;
	DCAM_INIT_PARAM_T init_param;
	printk("wwwww.init_dcam_parameters.\n");
	camera_zoom_picture_size(fh->width, fh->height, &trim_rect, g_zoom_level);
		
	 if(1 == dev->streamparm.parm.capture.capturemode)
	 {
	 	 init_param.mode = 3; // 0:DCAM_MODE 1:PREVIEW; 2:mpeg; 3:jpeg	
	 }
	 else 
	 {
	 	 init_param.mode = 1;
	 } 	 
  	 init_param.format = DCAM_DATA_YUV422; //for output format of sensor.
    	init_param.yuv_pattern = YUV_YUYV;    
    	init_param.display_rgb_type = RGB_565;
	init_param.input_size.w = trim_rect.w;
	init_param.input_size.h = trim_rect.h;
	init_param.input_rect.x = trim_rect.x;
         init_param.input_rect.y = trim_rect.y;
	init_param.input_rect.w = trim_rect.w;
	init_param.input_rect.h = trim_rect.h;			
   	 init_param.display_rect.x = 0;
   	 init_param.display_rect.y = 0;
   	 init_param.display_rect.w = trim_rect.w;
   	 init_param.display_rect.h = trim_rect.h; 
    	init_param.encoder_rect.x = 0;
    	init_param.encoder_rect.y = 0;
   	 init_param.encoder_rect.w = trim_rect.w;
   	init_param.encoder_rect.h = trim_rect.h;
    	init_param.polarity.hsync = 1;
    	init_param.polarity.vsync = 0;// //1
    	init_param.polarity.pclk = 0;	
   	 init_param.skip_frame = 0;
   	 init_param.rotation = DCAM_ROTATION_0;
   	 init_param.first_buf_addr = g_first_buf_addr;
	g_dcam_info.input_size.w =fh->width;
	g_dcam_info.mode = init_param.mode;
	
	dcam_parameter_init(&init_param);
}

static void init_dcam_parameters(void *priv)
{
	struct dcam_fh  *fh = priv;
	struct dcam_dev *dev = fh->dev;
	 //init dcam
	 {
	 DCAM_INIT_PARAM_T init_param;
	 if(1 == dev->streamparm.parm.capture.capturemode)
	 {
	 	 init_param.mode = 3;//1//:DCAM_MODE 1:PREVIEW; 2:mpeg; 3:jpeg	 	
	 }
	 else 
	 {
	 	 init_param.mode = 1;//1	 	 	 	 
	 }   
	 DCAM_V4L2_PRINT("###v4l2: fh->fmt->fourcc: %d, init_param.mode: %d.\n", fh->fmt->fourcc, init_param.mode);
    init_param.format = DCAM_DATA_YUV422; //for output format of sensor.
    init_param.yuv_pattern = YUV_YUYV;    
    init_param.display_rgb_type = RGB_565;
    /*if((fh->width < 640) && (fh->height < 480))
    	{
	    	init_param.input_size.w = 640;
		init_param.input_size.h = 480;
    	}
	else*/
	{
		init_param.input_size.w = fh->width;
		init_param.input_size.h = fh->height;
	}
		
    init_param.polarity.hsync = 1;
    init_param.polarity.vsync = 0;// //1
    init_param.polarity.pclk = 0;
    init_param.input_rect.x = 0;
    init_param.input_rect.y = 0;
   /*if((fh->width < 640) && (fh->height < 480))
    	{
	    	init_param.input_rect.w = 640;
		init_param.input_rect.h = 480;
    	}
	else*/
	{
		init_param.input_rect.w = fh->width;
		init_param.input_rect.h = fh->height;
	}    
    init_param.display_rect.x = 0;
    init_param.display_rect.y = 0;
    init_param.display_rect.w = fh->width;
    init_param.display_rect.h = fh->height; 
    init_param.encoder_rect.x = 0;
    init_param.encoder_rect.y = 0;
    init_param.encoder_rect.w = fh->width;
   init_param.encoder_rect.h = fh->height;
    init_param.skip_frame = 0;
    init_param.rotation = DCAM_ROTATION_0;
    init_param.first_buf_addr = g_first_buf_addr;

//	g_dcam_info.input_size.w = fh->width;
//	g_dcam_info.input_size.h = fh->height;
	g_dcam_info.input_size.w =init_param.input_size.w;
	g_dcam_info.input_size.h = init_param.input_size.h;
	g_dcam_info.mode = init_param.mode;
	 	dcam_parameter_init(&init_param);
	 }
}

#endif

#define DCAM_PIXEL_ALIGNED 16
#define DCAM_W_H_ALIGNED(x) (((x) + DCAM_PIXEL_ALIGNED - 1) & ~(DCAM_PIXEL_ALIGNED - 1))

typedef struct dcam_trim_rect{
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
}DCAM_TRIM_RECT_T;

void zoom_picture_size(uint32_t in_w, uint32_t in_h, DCAM_TRIM_RECT_T *trim_rect, uint32_t zoom_level)
{
	uint32_t trim_w, trim_h;

	switch(zoom_level)
	{
		case 0:
			trim_w = 0;
			trim_h = 0;
			break;
		case 1:
			trim_w = in_w >> 2;  // 1/4
			trim_h = in_h >> 2; // 1/4
			break;
		case 2:
			trim_w = in_w / 3; // 1/3
			trim_h = in_h / 3; // 1/3
			break;			
		case 3:
			trim_w = in_w * 3 >> 3;  // 3/8
			trim_h = in_h * 3 >> 3; // 3/8
			break;
		default:
			trim_w = 0;
			trim_h = 0;
			break;			
	}

	trim_rect->x = (trim_w + 3) & ~3;
	trim_rect->y = (trim_h + 3) & ~3;	
	trim_rect->w = DCAM_W_H_ALIGNED(in_w - (trim_rect->x << 1) );
	trim_rect->h = DCAM_W_H_ALIGNED(in_h - (trim_rect->y << 1));
	//wxz20110728: the sccale range must be in [4, 1/4]. There need to handle the scale factor when the zoom level is 3.
        if(3 == zoom_level){
                if(trim_rect->w < (in_w >> 2)){
                        trim_rect->w = DCAM_W_H_ALIGNED(in_w >> 2);
                }
                if(trim_rect->h < (in_h >> 2)){
                        trim_rect->h = DCAM_W_H_ALIGNED(in_h >> 2);
                }
        }
	DCAM_V4L2_PRINT("v4l2 trim_rect{x,y,w,h} --{%d, %d, %d, %d}, in_w: %d, in_h: %d, zoom_level: %d.\n", trim_rect->x, trim_rect->y, trim_rect->w, trim_rect->h,in_w, in_h, zoom_level);
}

static void init_dcam_parameters(void *priv)
{
	struct dcam_fh  *fh = priv;
	struct dcam_dev *dev = fh->dev;
	 //init dcam
	 {
	 DCAM_INIT_PARAM_T init_param;
	 //if(1 == dev->streamparm.parm.capture.capturemode)
	 if(1 == dev->streamparm.parm.raw_data[6])
	 {
	 	 init_param.mode = 3;//1//:DCAM_MODE 1:PREVIEW; 2:mpeg; 3:jpeg	 	
	 }
	 else 
	 {
	 	 init_param.mode = 1;//1	 	 	 	 
	 }   
	 DCAM_V4L2_PRINT("###v4l2: fh->fmt->fourcc: %d, init_param.mode: %d.\n", fh->fmt->fourcc, init_param.mode);
    init_param.format = DCAM_DATA_YUV422; //for output format of sensor.
    init_param.yuv_pattern = YUV_YUYV;    
    init_param.display_rgb_type = RGB_565;
    if((fh->width < 640) && (fh->height < 480))
    	{
	    	init_param.input_size.w = 640;
		init_param.input_size.h = 480;
    	}
	else
	{
		init_param.input_size.w = fh->width;
		init_param.input_size.h = fh->height;
	}
		
    init_param.polarity.hsync = 1;
    init_param.polarity.vsync = 0;// //1
    init_param.polarity.pclk = 0;
   
   /*if((fh->width < 640) && (fh->height < 480))
    	{    	
		init_param.input_rect.x = (640 - fh->width) >> 1 ;
		init_param.input_rect.y = (480 - fh->height) >> 1;
    	}
	else
	{
		init_param.input_rect.x = 0;
		init_param.input_rect.y = 0;
	}
	init_param.input_rect.w = fh->width;
	init_param.input_rect.h = fh->height;*/	
	if(1 == g_dcam_info.orientation){
		uint32_t tmp = init_param.input_size.w;
		init_param.input_size.w = (((init_param.input_size.h * 3 >> 2) + 3) >> 2) << 2; //wxz20110815: the w = h*3/4;
		init_param.input_rect.x = ((((tmp - init_param.input_size.w) >> 1) + 3) >> 2) << 2;
		init_param.input_rect.y = 0;
		init_param.input_rect.w = init_param.input_size.w;
		init_param.input_rect.h = init_param.input_size.h;
    		init_param.display_rect.w = fh->height;
		init_param.display_rect.h = fh->width; 
    		init_param.encoder_rect.w = fh->height;
   		init_param.encoder_rect.h = fh->width;
	}
	else{
		init_param.input_rect.x = 0;
		init_param.input_rect.y = 0;
		init_param.input_rect.w = init_param.input_size.w;
		init_param.input_rect.h = init_param.input_size.h;
    		init_param.display_rect.w = fh->width;
    		init_param.display_rect.h = fh->height; 
    		init_param.encoder_rect.w = fh->width;
   		init_param.encoder_rect.h = fh->height;
	}
    init_param.display_rect.x = 0;
    init_param.display_rect.y = 0;
    init_param.encoder_rect.x = 0;
    init_param.encoder_rect.y = 0;
	//if do zoom, need to modify the input_rect and encoder_rect
	if((0 != g_zoom_level) && (3 == init_param.mode))
	{
		DCAM_TRIM_RECT_T trim_rect;
		zoom_picture_size(init_param.input_size.w, init_param.input_size.h,  &trim_rect, g_zoom_level);
		init_param.input_rect.x += trim_rect.x;
		init_param.input_rect.y += trim_rect.y;
		init_param.input_rect.w = trim_rect.w;
		init_param.input_rect.h = trim_rect.h;	
		init_param.encoder_rect.x = 0;
    		init_param.encoder_rect.y = 0;
		if(1 == g_dcam_info.orientation){
			init_param.input_size.w = fh->width;
                }
	    	init_param.encoder_rect.w = trim_rect.w;
   		init_param.encoder_rect.h = trim_rect.h;	
		
		DCAM_V4L2_PRINT("#########wxz: size: w: %d, h: %d, rect: x: %d, y: %d, w: %d, h: %d, enc: w: %d, h: %d.\n", 
			init_param.input_size.w, init_param.input_size.h, init_param.input_rect.x, init_param.input_rect.y, init_param.input_rect.w, init_param.input_rect.h, init_param.encoder_rect.w, init_param.encoder_rect.h);
	}
    init_param.skip_frame = 0;
    init_param.rotation = DCAM_ROTATION_0;
    init_param.first_buf_addr = g_first_buf_addr;

//	g_dcam_info.input_size.w = fh->width;
//	g_dcam_info.input_size.h = fh->height;
	g_dcam_info.input_size.w =init_param.input_size.w;
	g_dcam_info.input_size.h = init_param.input_size.h;
	g_dcam_info.mode = init_param.mode;
	 	dcam_parameter_init(&init_param);
	 }
}
static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct dcam_fh  *fh = priv;
	int ret = 0;//, k;

	DCAM_V4L2_PRINT("###DCAM_V4L2: videobuf_streamon start.\n");
	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;
	init_dcam_parameters(priv);//wxz:???
	init_sensor_parameters();

	if(0 != (ret = videobuf_streamon(&fh->vb_vidq)))
	{
		DCAM_V4L2_PRINT("###DCAM_V4L2: Fail to videobuf_streamon.\n");
		return ret;
	}
	g_is_first_irq = 1;
	g_last_buf = 0xFFFFFFFF;
	 dcam_start();

#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM_V4L2: OK to vidioc_streamon.\n");
#endif	
	return 0;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct dcam_fh  *fh = priv;
	int ret = 0;
	int k;

	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;

	if(0 != (ret = videobuf_streamoff(&fh->vb_vidq)))
	{
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM_V4L2: Fail to videobuf_streamoff.\n");
#endif
		return ret;
	}
	g_is_first_frame = 1; //store the nex first frame.
	
       //stop dcam
      dcam_stop();

	   for(k = 0; k < VIDEO_MAX_FRAME; k++)
		if((NULL != fh->vb_vidq.bufs[k]) && (VIDEOBUF_IDLE != fh->vb_vidq.bufs[k]->state))
		{
			fh->vb_vidq.bufs[k]->state = VIDEOBUF_IDLE;
		}
       	
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM_V4L2: OK to vidioc_streamoff.\n");
#endif	
	return ret;
}

#define frames_to_ms(frames)					\
	((frames * WAKE_NUMERATOR * 1000) / WAKE_DENOMINATOR)


/* ------------------------------------------------------------------
	callback function
   ------------------------------------------------------------------*/
static void set_next_buffer(struct dcam_fh *fh)
{
	struct dcam_buffer *buf;
	struct dcam_dev *dev = fh->dev;
	struct dcam_dmaqueue *dma_q = &dev->vidq;

	unsigned long flags = 0;

	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dma_q->active)) {
		printk("###V4L2: set_next_buffer:No active queue to serve\n");
		goto unlock;
	}
	
	if(NULL == dma_q->active.next){
		printk("###V4L2: set_next_buffer: the dma_q->active.next is NULL.\n");
		goto unlock;
	}
	buf = list_entry(dma_q->active.next,
			 struct dcam_buffer, vb.queue);
	if(0xFFFFFFFF == g_last_buf){ //the first buffer set to driver already.
		buf->fmt->flag = 1;
		g_last_buf = 0;
	}	
	if((1 == buf->fmt->flag) || (g_last_buf == buf->vb.baddr)){		
		if(NULL == dma_q->active.next->next){
			printk("###V4L2: set_next_buffer: the dma_q->active.next->next is NULL.\n");
			goto unlock;
		}
		buf = list_entry(dma_q->active.next->next,
				 struct dcam_buffer, vb.queue);
	}

	/* Nobody is waiting on this buffer, return */
	//if (!waitqueue_active(&buf->vb.done))
	//	goto unlock;

	//list_del(&buf->vb.queue);

	//do_gettimeofday(&buf->vb.ts);

	/* Fill buffer */
	if(0 != buf->vb.baddr){
		buf->fmt->flag = 1;
		g_last_buf = buf->vb.baddr;		
		dcam_set_buffer_address(buf->vb.baddr);		
	}
	else
	{
		printk("###V4L2: fail: set_next_buffer filled buffer is 0.\n");
		goto unlock;
	}
	
	DCAM_V4L2_PRINT("###V4L2: set_next_buffer filled buffer %x.\n", (uint32_t)buf->vb.baddr);

	//wake_up(&buf->vb.done);
	//g_dcam_buf_ptr = buf;
	
unlock:
	spin_unlock_irqrestore(&dev->slock, flags);
	return;
}
#if 0
static void set_layer_cb(uint32_t base)
{
#define LCDC_Y2R_CONTRAST           (SPRD_LCDC_BASE + 0x0114)
#define LCDC_Y2R_SATURATION         (SPRD_LCDC_BASE + 0x0118)
#define LCDC_OSD1_CTRL                    (SPRD_LCDC_BASE + 0x0040)
#define LCDC_OSD1_ALPHA                     (SPRD_LCDC_BASE + 0x0058)
#define LCDC_IMG_CTRL                    (SPRD_LCDC_BASE + 0x0020)
#define LCDC_IMG_Y_BASE_ADDR         (SPRD_LCDC_BASE + 0x0024)
#define LCDC_IMG_UV_BASE_ADDR            (SPRD_LCDC_BASE + 0x0028)
#define LCDC_IMG_SIZE_XY             (SPRD_LCDC_BASE + 0x002c)
#define LCDC_IMG_PITCH                   (SPRD_LCDC_BASE + 0x0030)
#define LCDC_IMG_DISP_XY             (SPRD_LCDC_BASE + 0x0034)

       __raw_bits_or((1<<2), LCDC_OSD1_CTRL);//block alpha
        __raw_writel(0xF,LCDC_OSD1_ALPHA);

        //__raw_writel(0x23,LCDC_IMG_CTRL);
        //__raw_writel(0x14B,LCDC_IMG_CTRL);
        //__raw_writel(0xA1,LCDC_IMG_CTRL);
        __raw_writel(0x141,LCDC_IMG_CTRL);

        __raw_writel(64,LCDC_Y2R_CONTRAST);
        __raw_writel(64,LCDC_Y2R_SATURATION);

        __raw_writel(base >> 2,LCDC_IMG_Y_BASE_ADDR);
        __raw_writel((base+640*480)>>2,LCDC_IMG_UV_BASE_ADDR);
        __raw_writel((480<<16)|640,LCDC_IMG_SIZE_XY);
        __raw_writel(640,LCDC_IMG_PITCH);
        __raw_writel(0x0,LCDC_IMG_DISP_XY);
	printk("set_layer_cb: A1base : 0x%x.\n", base);
}
#endif
static void path1_done_buffer(struct dcam_fh *fh)
{
	struct dcam_buffer *buf;
	struct dcam_dev *dev = fh->dev;
	struct dcam_dmaqueue *dma_q = &dev->vidq;
	

	unsigned long flags = 0;
	DCAM_V4L2_PRINT("###V4L2: path1_done_buffer.\n");	
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dma_q->active)) {
		printk("###V4L2: path1_done_buffer: No active queue to serve\n");
		goto unlock;
	}

	if(NULL == dma_q->active.next){
		printk("###V4L2: path1_done_buffer: the active.next is NULL.\n");
		goto unlock;
	}
	buf = list_entry(dma_q->active.next,
			 struct dcam_buffer, vb.queue);
	if(1 != g_is_first_irq){
		if((g_first_buf_addr != (uint32_t)buf->vb.baddr) ||
			(g_first_buf_addr == g_last_buf)){ //wxz20110727: if the next buf is last buffer, do not wake up the buffer.
			printk("###V4L2: path1_done_buffer: need another buffer. last addr: %x, buf addr: %x, g_last_buf: %x\n", g_first_buf_addr, (uint32_t)buf->vb.baddr, g_last_buf);
			goto unlock;
		}		
	}
	else{
		g_is_first_irq = 0;		
	}

	/* Nobody is waiting on this buffer, return */
	/*if (!waitqueue_active(&buf->vb.done))
	{
		DCAM_V4L2_PRINT("###V4L2: path1_done_buffer: Nobody is waiting on this buffer\n");	
		if(3 == g_dcam_info.mode)
		{			
			list_del(&buf->vb.queue);
			buf->vb.field_count++;
			do_gettimeofday(&buf->vb.ts);
			buf->vb.state = VIDEOBUF_DONE;
			wake_up(&buf->vb.done); 
			DCAM_V4L2_PRINT("###V4L2:g_dcam_info wake_up.\n");
		}
		goto unlock;
	}*/
	//printk("###V4L2: before set_next_buffer :filled buffer %x, addr: %x.\n", (uint32_t)buf->vb.baddr, _pard(DCAM_ADDR_7));
#if 0
	set_layer_cb(buf->vb.baddr);
#endif
	//set_next_buffer(g_fh);//wxz:???

	list_del(&buf->vb.queue);

	//do_gettimeofday(&buf->vb.ts);

	/* Fill buffer */

	/* Advice that buffer was filled */
	buf->vb.field_count++;
	do_gettimeofday(&buf->vb.ts);
	buf->vb.state = VIDEOBUF_DONE;
	#if 0 //wxz20111029: for test the error in preview and capture mode
	{
		//for capture
		if(fh->width > 1000){
			buf->vb.state = VIDEOBUF_ERROR;
		}
		//for preview
		static uint32_t num = 0;
		if(100 == num++){
			buf->vb.state = VIDEOBUF_ERROR;
			num = 0;
		}
	}
	#endif
	
	DCAM_V4L2_PRINT("###V4L2: path1_done_buffer:filled buffer %x, addr: %x.\n", (uint32_t)buf->vb.baddr, _pard(DCAM_ADDR_7));
        //is_first_frame = 0;
        //g_dcam_buf_ptr = buf;
	wake_up(&buf->vb.done);
	g_first_buf_addr = g_last_buf; 
unlock:
	spin_unlock_irqrestore(&dev->slock, flags);
	//g_first_buf_addr = g_last_buf;
	return;
}

static void dcam_error_handle(struct dcam_fh *fh)
{
	struct dcam_buffer *buf;
	struct dcam_dev *dev = fh->dev;
	struct dcam_dmaqueue *dma_q = &dev->vidq;
	

	unsigned long flags = 0;
	DCAM_V4L2_PRINT("###V4L2: dcam_error_handle.\n");	
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dma_q->active)) {
		printk("###V4L2: dcam_error_handle: No active queue to serve\n");
		goto unlock;
	}

	if(NULL == dma_q->active.next){
		printk("###V4L2: dcam_error_handle: the active.next is NULL.\n");
		goto unlock;
	}
	buf = list_entry(dma_q->active.next,
			 struct dcam_buffer, vb.queue);
	if(1 != g_is_first_irq){
		if((g_first_buf_addr != (uint32_t)buf->vb.baddr) ||
			(g_first_buf_addr == g_last_buf)){ //wxz20110727: if the next buf is last buffer, do not wake up the buffer.
			printk("###V4L2: dcam_error_handle: need another buffer. last addr: %x, buf addr: %x, g_last_buf: %x\n", g_first_buf_addr, (uint32_t)buf->vb.baddr, g_last_buf);
			goto unlock;
		}		
	}
	else{
		g_is_first_irq = 0;		
	}

	list_del(&buf->vb.queue);
	// Advice that buffer was filled
	buf->vb.field_count++;
	do_gettimeofday(&buf->vb.ts);
	buf->vb.state = VIDEOBUF_ERROR;
	
	DCAM_V4L2_PRINT("###V4L2: dcam_error_handle:filled buffer %x, addr: %x.\n", (uint32_t)buf->vb.baddr, _pard(DCAM_ADDR_7));
      
	wake_up(&buf->vb.done);
	g_first_buf_addr = g_last_buf; 
unlock:
	spin_unlock_irqrestore(&dev->slock, flags);	
	return;
}

void dcam_cb_ISRSensorSOF(void)
{
}
void dcam_cb_ISRCapEOF(void)
{
	set_next_buffer(g_fh);
}
void dcam_cb_ISRPath1Done(void)
{	
	path1_done_buffer(g_fh);
}
void dcam_cb_ISRCapFifoOF(void)
{	
	dcam_error_handle(g_fh);
}
void dcam_cb_ISRSensorLineErr(void)
{
	//dcam_error_handle(g_fh);
}
void dcam_cb_ISRSensorFrameErr(void)
{	
	dcam_error_handle(g_fh);
}
void dcam_cb_ISRJpegBufOF(void)
{	
	//nothing
}
/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/
static int
buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct dcam_fh  *fh = vq->priv_data;
	//struct dcam_dev *dev  = fh->dev;

	if(V4L2_PIX_FMT_RGB32 == fh->fmt->fourcc)
		*size = fh->width*fh->height*4;
	else if(V4L2_PIX_FMT_RGB565X == fh->fmt->fourcc)
		*size = fh->width*fh->height*2;
	else if(V4L2_PIX_FMT_YUV420 == fh->fmt->fourcc)
		*size = fh->width*fh->height * 3 / 2;	
	else if(V4L2_PIX_FMT_YUV422P == fh->fmt->fourcc)
		*size = fh->width*fh->height * 2;
	else	
		*size = fh->width*fh->height * 2;

	if (0 == *count)
		*count = 32;

	while (*size * *count > vid_limit * 1024 * 1024)
		(*count)--;

	//dprintk(dev, 1, "%s, count=%d, size=%d\n", __func__,	*count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct dcam_buffer *buf)
{
	//struct dcam_fh  *fh = vq->priv_data;
	//struct dcam_dev *dev  = fh->dev;

	//dprintk(dev, 1, "%s, state: %i\n", __func__, buf->vb.state);

	if (in_interrupt())
		BUG();
        
	videobuf_vmalloc_free(&buf->vb);
	//dprintk(dev, 1, "free_buffer: freed\n");
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}


static int
buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct dcam_fh     *fh  = vq->priv_data;
	//struct dcam_dev    *dev = fh->dev;
	struct dcam_buffer *buf = container_of(vb, struct dcam_buffer, vb);
//	int rc = 0;
	//unsigned int addr;
	DCAM_V4L2_PRINT("###DCAM_V4L2:buffer_prepare  w: %d, h: %d, baddr: %lx, bsize: %d.\n ",fh->width,fh->height, buf->vb.baddr, buf->vb.bsize);
        DCAM_V4L2_PRINT("###DCAM_V4L2:buffer_prepare  start.  fh->fmt:%d\n", (int) fh->fmt);
	//dprintk(dev, 1, "%s, field=%d\n", __func__, field);

	BUG_ON(NULL == fh->fmt);

	

	if (fh->width  < 48 || fh->width  > norm_maxw() ||
	    fh->height < 32 || fh->height > norm_maxh())
		return -EINVAL;

	//buf->vb.size = fh->width*fh->height*2;
	if(V4L2_PIX_FMT_RGB32 == fh->fmt->fourcc)
		buf->vb.size = fh->width*fh->height*4;
	else if(V4L2_PIX_FMT_RGB565X == fh->fmt->fourcc)
		buf->vb.size = fh->width*fh->height*2;
	else if(V4L2_PIX_FMT_YUV420 == fh->fmt->fourcc)
		buf->vb.size = fh->width*fh->height * 3 / 2;	
	else if(V4L2_PIX_FMT_YUV422P == fh->fmt->fourcc)
		buf->vb.size = fh->width*fh->height * 2;
	else	
		buf->vb.size = fh->width*fh->height * 2;

	if (0 != buf->vb.baddr  &&  buf->vb.bsize < buf->vb.size)
		return -EINVAL;

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = fh->fmt;
	buf->vb.width  = fh->width;
	buf->vb.height = fh->height;
	buf->vb.field  = field;	

	precalculate_bars(fh);

        //wxz: delete on 20101026 error: no support USERPTR
	/*if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0)
			goto fail;
	}*/

	buf->vb.state = VIDEOBUF_PREPARED;

       //wxz: insert the buffer into dcam driver buffer queue
       /*addr = vb->baddr;
	if(0 != dcam_qbuf(addr))
	{
	  goto fail;
	}*/

	return 0;

//fail:
//	free_buffer(vq, buf);
//	return rc;
}

static void
buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct dcam_buffer    *buf  = container_of(vb, struct dcam_buffer, vb);
	struct dcam_fh        *fh   = vq->priv_data;
	struct dcam_dev       *dev  = fh->dev;
	struct dcam_dmaqueue *vidq = &dev->vidq;

	//dprintk(dev, 1, "%s\n", __func__);

	buf->vb.state = VIDEOBUF_QUEUED;
	buf->fmt->flag = 0;
	list_add_tail(&buf->vb.queue, &vidq->active);
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct dcam_buffer   *buf  = container_of(vb, struct dcam_buffer, vb);
	//struct dcam_fh       *fh   = vq->priv_data;
	//struct dcam_dev      *dev  = (struct dcam_dev *)fh->dev;

	//dprintk(dev, 1, "%s\n", __func__);

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops dcam_video_qops = {
	.buf_setup      = buffer_setup,
	.buf_prepare    = buffer_prepare,
	.buf_queue      = buffer_queue,
	.buf_release    = buffer_release,
};


/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/
static int open(struct file *file)
{
	struct dcam_dev *dev = video_drvdata(file);
	struct dcam_fh *fh = NULL;
	int retval = 0;
	
	mutex_lock(&dev->mutex);
	dev->users++;

	if (dev->users > 1) {
		dev->users--;
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	dprintk(dev, 1, "open /dev/video%d type=%s users=%d\n", dev->vfd->num,
		v4l2_type_names[V4L2_BUF_TYPE_VIDEO_CAPTURE], dev->users);

	/* allocate + initialize per filehandle data */
	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		dev->users--;
		retval = -ENOMEM;
	}
	mutex_unlock(&dev->mutex);

	if (retval)
		return retval;

	file->private_data = fh;
	fh->dev      = dev;

	fh->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->fmt      = &formats[0];
	fh->width    = 640;
	fh->height   = 480;

	/* Resets frame counters */
	dev->h = 0;
	dev->m = 0;
	dev->s = 0;
	dev->ms = 0;
	dev->mv_count = 0;
	dev->jiffies = jiffies;
	sprintf(dev->timestr, "%02d:%02d:%02d:%03d",
			dev->h, dev->m, dev->s, dev->ms);

       videobuf_queue_vmalloc_init(&fh->vb_vidq, &dcam_video_qops,
			NULL, &dev->slock, fh->type, V4L2_FIELD_INTERLACED,
			sizeof(struct dcam_buffer), fh);

	g_fh = fh;

/*
	Sensor_SetSensorType(SENSOR_TYPE_IMG_SENSOR);
	Sensor_SetCurId(SENSOR_MAIN);
	if(0 == Sensor_IsInit()){		
		if(1 != Sensor_Init()){
			DCAM_V4L2_PRINT("###DCAM: Fail to init sensor.\n");
			return -1;
		}
	}
	DCAM_V4L2_PRINT("###DCAM: OK to init sensor.\n");
	//open sensor
        if(0 != Sensor_Open()){
		DCAM_V4L2_PRINT("###DCAM: fail to open sensor.\n");
		return -1;
        }
		
	DCAM_V4L2_PRINT("###DCAM: OK to open sensor.\n");	
	*/
	//open dcam
        dcam_open();
	DCAM_V4L2_PRINT("###DCAM: OK to open dcam.\n");
	dcam_callback_fun_register(DCAM_CB_SENSOR_SOF ,dcam_cb_ISRSensorSOF);
	dcam_callback_fun_register(DCAM_CB_CAP_EOF ,dcam_cb_ISRCapEOF);	
	dcam_callback_fun_register(DCAM_CB_PATH1_DONE,dcam_cb_ISRPath1Done);
	dcam_callback_fun_register(DCAM_CB_CAP_FIFO_OF,dcam_cb_ISRCapFifoOF);	
	dcam_callback_fun_register(DCAM_CB_SENSOR_LINE_ERR,dcam_cb_ISRSensorLineErr);	
	dcam_callback_fun_register(DCAM_CB_SENSOR_FRAME_ERR,dcam_cb_ISRSensorFrameErr);	
	dcam_callback_fun_register(DCAM_CB_JPEG_BUF_OF,dcam_cb_ISRJpegBufOF);

	return 0;
}

static int close(struct file *file)
{
	struct dcam_fh         *fh = file->private_data;
	struct dcam_dev *dev       = fh->dev;
	//struct dcam_dmaqueue *vidq = &dev->vidq;

	int minor = video_devdata(file)->minor;

	//stop_thread(vidq);
	videobuf_stop(&fh->vb_vidq);
	videobuf_mmap_free(&fh->vb_vidq);

	kfree(fh);

	mutex_lock(&dev->mutex);
	dev->users--;
	mutex_unlock(&dev->mutex);

	dprintk(dev, 1, "close called (minor=%d, users=%d)\n",
		minor, dev->users);
	
	//close sensor       
        Sensor_Close();
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM: OK to close sensor.\n");
#endif

	//close dcam
        dcam_close();
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM: OK to close dcam.\n");
#endif

	return 0;
}

/**************************************************************************/

static const struct v4l2_file_operations dcam_fops = {
	.owner		= THIS_MODULE,
	.open           = open,
	.release        = close,
//	.read           = read,
//	.poll		= dcam_poll,
	.ioctl          = video_ioctl2, /* V4L2 ioctl handler */
//	.mmap           = dcam_mmap,
};

static const struct v4l2_ioctl_ops dcam_ioctl_ops = {
	.vidioc_g_parm        = vidioc_g_parm,
	.vidioc_s_parm        = vidioc_s_parm,	
	.vidioc_querycap      = vidioc_querycap,
	.vidioc_cropcap  = vidioc_cropcap,
	.vidioc_s_crop = vidioc_s_crop,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs       = vidioc_reqbufs,
	.vidioc_querybuf      = vidioc_querybuf,
	.vidioc_qbuf          = vidioc_qbuf,
	.vidioc_dqbuf         = vidioc_dqbuf,
	.vidioc_s_std         = vidioc_s_std,
	.vidioc_enum_input    = vidioc_enum_input,
	.vidioc_g_input       = vidioc_g_input,
	.vidioc_s_input       = vidioc_s_input,
	.vidioc_queryctrl     = vidioc_queryctrl,
	.vidioc_g_ctrl        = vidioc_g_ctrl,
	.vidioc_s_ctrl        = vidioc_s_ctrl,
	.vidioc_streamon      = vidioc_streamon,
	.vidioc_streamoff     = vidioc_streamoff,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
//	.vidiocgmbuf          = vidiocgmbuf,
#endif
};

static struct video_device dcam_template = {
	.name		= "dcam",
	.fops           = &dcam_fops,
	.ioctl_ops 	= &dcam_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,

	.tvnorms              = V4L2_STD_525_60,
	.current_norm         = V4L2_STD_NTSC_M,
};

/* -----------------------------------------------------------------
	Initialization and module stuff
   ------------------------------------------------------------------*/

static int release(void)
{
	struct dcam_dev *dev;
	struct list_head *list;

	while (!list_empty(&dcam_devlist)) {
		list = dcam_devlist.next;
		list_del(list);
		dev = list_entry(list, struct dcam_dev, dcam_devlist);

		v4l2_info(&dev->v4l2_dev, "unregistering /dev/video%d\n",
			dev->vfd->num);
		video_unregister_device(dev->vfd);
		v4l2_device_unregister(&dev->v4l2_dev);
		kfree(dev);
	}

	return 0;
}

static int __init create_instance(int inst)
{
	struct dcam_dev *dev;
	struct video_device *vfd;
	int ret, i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
			"%s-%03d", DCAM_MODULE_NAME, inst);
	ret = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (ret)
		goto free_dev;

	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	init_waitqueue_head(&dev->vidq.wq);

	/* initialize locks */
	spin_lock_init(&dev->slock);
	mutex_init(&dev->mutex);

	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;

	*vfd = dcam_template;
	vfd->debug = debug;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		goto rel_vdev;

	video_set_drvdata(vfd, dev);

	/* Set all controls to their default value. */
	for (i = 0; i < ARRAY_SIZE(dcam_qctrl); i++)
		dev->qctl_regs[i] = dcam_qctrl[i].default_value;

	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->dcam_devlist, &dcam_devlist);

	snprintf(vfd->name, sizeof(vfd->name), "%s (%i)",
			dcam_template.name, vfd->num);

	if (video_nr >= 0)
		video_nr++;

	dev->vfd = vfd;
	v4l2_info(&dev->v4l2_dev, "V4L2 device registered as /dev/video%d\n",
			vfd->num);
	return 0;

rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
	kfree(dev);
	return ret;
}

/* This routine allocates from 1 to n_devs virtual drivers.

   The real maximum number of virtual drivers will depend on how many drivers
   will succeed. This is limited to the maximum number of devices that
   videodev supports, which is equal to VIDEO_NUM_DEVICES.
 */

/**************************************************************************/

/* -----------------------------------------------------------------
	Initialization and module stuff
   ------------------------------------------------------------------*/
static struct miscdevice dcam_v4l2_dev = {
	.minor   = DCAM_MINOR,
	.name   = "sc8800g_dcam",
	.fops   = &dcam_fops,
};

int dcam_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_ALERT"dcam_probe called\n");

	ret = misc_register(&dcam_v4l2_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				DCAM_MINOR, ret);
		return ret;
	}

	lock = (struct mutex *)kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL)
		return -1;

	mutex_init(lock);

	printk(KERN_ALERT "dcam_probe Success.\n");

	return 0;
}


static int dcam_remove(struct platform_device *dev)
{
	printk(KERN_INFO "dcam_remove called !\n");

	misc_deregister(&dcam_v4l2_dev);

	printk(KERN_INFO "dcam_remove Success !\n");
	return 0;
}


static struct platform_driver dcam_driver = {
	.probe    = dcam_probe,
	.remove   = dcam_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sc8800g_dcam",
	},
};
int __init dcam_v4l2_init(void)
{
	int ret = 0, i;
	
	if(platform_driver_register(&dcam_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	
	if (n_devs <= 0)
		n_devs = 1;

	for (i = 0; i < n_devs; i++) {
		ret = create_instance(i);
		if (ret) {
			/* If some instantiations succeeded, keep driver */
			if (i)
				ret = 0;
			break;
		}
	}

	if (ret < 0) {
		printk(KERN_INFO "Error %d while loading dcam driver\n", ret);
		return ret;
	}

	printk(KERN_INFO "Video Technology Magazine Virtual Video "
			"Capture Board ver %u.%u.%u successfully loaded.\n",
			(DCAM_VERSION >> 16) & 0xFF, (DCAM_VERSION >> 8) & 0xFF,
			DCAM_VERSION & 0xFF);

	/* n_devs will reflect the actual number of allocated devices */
	n_devs = i;

	//init the dcam and sensor
	//dcam_init();
#if 0	
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM: OK to init dcam.\n");
#endif
	Sensor_SetSensorType(SENSOR_TYPE_IMG_SENSOR);
	if(0 == Sensor_IsInit())
		Sensor_Init();
#if DCAM_V4L2_DEBUG
	DCAM_V4L2_PRINT("###DCAM: OK to init sensor.\n");
#endif
#endif
	return ret;
}

void dcam_v4l2_exit(void)
{
	platform_driver_unregister(&dcam_driver);
	mutex_destroy(lock);
	release();
}

module_init(dcam_v4l2_init);
module_exit(dcam_v4l2_exit);

MODULE_DESCRIPTION("Dcam Driver");
MODULE_LICENSE("GPL");
