/*
 * Sprd Video Overlay  driver 
 *
 * Copyright (c) 2010 by:
 *      http://spreadtrum.com.cn/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the BSD Licence, GNU General Public License
 * as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
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
#include <mach/hardware.h>

#define LID_VIDEO 0
#define LID_FB    1
#define LID_MAX   2
int rrm_refresh(int id, void (*callback)(void* data), void *data);
int rrm_layer_init(int id, int buf_num, void (*set_layer)(void *data));

#define VIDEO_LAYER_BUF_NUM 2
#define V4L2_CID_ROTATE 0

#define DEFAULT_IMAGE_W 320
#define DEFAULT_IMAGE_H 480
#define IAMGE_LAYER_PAGE_ORDER 6 //480x320*1.5

#define norm_maxw() 768
#define norm_maxh() 1024

#define LCDC_CTRL                   (SPRD_LCDC_BASE + 0x0000)
#define LCDC_DISP_SIZE              (SPRD_LCDC_BASE + 0x0004)
#define LCDC_LCM_START              (SPRD_LCDC_BASE + 0x0008)
#define LCDC_LCM_SIZE               (SPRD_LCDC_BASE + 0x000c)
#define LCDC_BG_COLOR               (SPRD_LCDC_BASE + 0x0010)
#define LCDC_FIFO_STATUS            (SPRD_LCDC_BASE + 0x0014)

#define LCDC_IMG_CTRL               (SPRD_LCDC_BASE + 0x0020)
#define LCDC_IMG_Y_BASE_ADDR        (SPRD_LCDC_BASE + 0x0024)
#define LCDC_IMG_UV_BASE_ADDR       (SPRD_LCDC_BASE + 0x0028)
#define LCDC_IMG_SIZE_XY            (SPRD_LCDC_BASE + 0x002c)
#define LCDC_IMG_PITCH              (SPRD_LCDC_BASE + 0x0030)
#define LCDC_IMG_DISP_XY            (SPRD_LCDC_BASE + 0x0034)

#define LCDC_OSD1_CTRL              (SPRD_LCDC_BASE + 0x0040)
#define LCDC_OSD1_ALPHA             (SPRD_LCDC_BASE + 0x0058)

#define LCDC_Y2R_CTRL               (SPRD_LCDC_BASE + 0x0110)
#define LCDC_Y2R_CONTRAST           (SPRD_LCDC_BASE + 0x0114)
#define LCDC_Y2R_SATURATION         (SPRD_LCDC_BASE + 0x0118)
#define LCDC_Y2R_BRIGHTNESS         (SPRD_LCDC_BASE + 0x011C)

#define VIVI_MODULE_NAME "sprd_overlay"

/* Wake up at about 30 fps */
#define WAKE_NUMERATOR 30
#define WAKE_DENOMINATOR 1001
#define BUFFER_TIMEOUT     msecs_to_jiffies(500)  /* 0.5 seconds */

#define VIVI_MAJOR_VERSION 0
#define VIVI_MINOR_VERSION 6
#define VIVI_RELEASE 0
#define VIVI_VERSION \
	KERNEL_VERSION(VIVI_MAJOR_VERSION, VIVI_MINOR_VERSION, VIVI_RELEASE)

MODULE_DESCRIPTION("Sprd Video Overlay");
MODULE_AUTHOR("Jianguo Du");
MODULE_LICENSE("Dual BSD/GPL");

static unsigned video_nr = 1;//-1;
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


/* supported controls */
static struct v4l2_queryctrl vivi_qctrl[] = {
	{
		.id            = V4L2_CID_ROTATE,
		.name          = "rotation",
		.minimum       = 0,
		.maximum       = 270,
		.step          = 90,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	}, 
};

#define dprintk(dev, level, fmt, arg...) \
	v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

/* ------------------------------------------------------------------
	Basic structures
   ------------------------------------------------------------------*/

struct vivi_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	int   depth;
};

static struct vivi_fmt formats[] = {
	{
		.name    = "4:2:0, packed, YUV",
		.fourcc   = V4L2_PIX_FMT_YUV420,
		.depth    = 8,
	},
};

static struct vivi_fmt *get_format(struct v4l2_format *f)
{
	struct vivi_fmt *fmt;
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

/* buffer for one video frame */
struct vivi_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	struct vivi_fmt        *fmt;

	struct v4l2_rect   crop;
};

struct vivi_dmaqueue {
	struct list_head       active;

	/* thread for generating video stream*/
	struct task_struct         *kthread;
	wait_queue_head_t          wq;
	/* Counters to control fps rate */
	int                        frame;
	int                        ini_jiffies;
};

static LIST_HEAD(vivi_devlist);

struct disp_buffer_t{
	unsigned int src_Y_addr;
	unsigned int src_width;
	unsigned int src_height;	
	unsigned int crop_left;
	unsigned int crop_top;
	unsigned int crop_width;
	unsigned int crop_height;
	
	unsigned int Y_addr;
	unsigned int UV_addr;
	
	unsigned int  img_disp_x,img_disp_y, img_size_y, img_size_x;
	unsigned int  img_pitch;
	unsigned int  overlay_degree;

	struct semaphore *sem;
};
struct vivi_dev {
	struct list_head           vivi_devlist;
	struct v4l2_device 	   v4l2_dev;

	spinlock_t                 slock;
	struct mutex		   mutex;

	int                        users;

	/* various device info */
	struct video_device        *vfd;

	struct vivi_dmaqueue       vidq;

	/* Several counters */
	int                        h, m, s, ms;
	unsigned long              jiffies;
	char                       timestr[13];

	int			   mv_count;	/* Controls bars movement */

	/* Input Number */
	int			   input;

	/* Control 'registers' */
	int 			   qctl_regs[ARRAY_SIZE(vivi_qctrl)];

	unsigned int	 overlay_left,overlay_top, overlay_width, overlay_height;
	unsigned int     overlay_degree;
	unsigned int     is_first_pic;       
	struct semaphore disp_done_sem;
	wait_queue_head_t          dma_wq;
	struct disp_buffer_t overlay_buffer[2];
	struct disp_buffer_t *overlay_buffer_ptr[2];
	
};

struct vivi_fh {
	struct vivi_dev            *dev;

	/* video capture */
	struct vivi_fmt            *fmt;
	unsigned int               width, height;
	struct videobuf_queue      vb_vidq;
	struct v4l2_rect   crop;
		
	enum v4l2_buf_type         type;
	unsigned char              bars[8][3];
	int			   input; 	/* Input Number on bars */
};

/* ------------------------------------------------------------------
	DMA and thread functions
   ------------------------------------------------------------------*/
static void rrm_set_layer_cb(void *data)
{
	struct disp_buffer_t *overlay_buffer_ptr = (struct disp_buffer_t *)data;
	printk(KERN_INFO "[Overlay] Disp CB %x,%x\n",overlay_buffer_ptr->src_Y_addr,overlay_buffer_ptr->src_Y_addr+overlay_buffer_ptr->src_width*overlay_buffer_ptr->src_height);	

	__raw_bits_or((1<<2), LCDC_OSD1_CTRL);//block alpha
	__raw_writel(0x7F,LCDC_OSD1_ALPHA);

	__raw_writel(0x23,LCDC_IMG_CTRL);

	__raw_writel(64,LCDC_Y2R_CONTRAST);
	__raw_writel(64,LCDC_Y2R_SATURATION);

	__raw_writel(overlay_buffer_ptr->src_Y_addr>>2,LCDC_IMG_Y_BASE_ADDR);
	__raw_writel((overlay_buffer_ptr->src_Y_addr+overlay_buffer_ptr->src_width*overlay_buffer_ptr->src_height)>>2,LCDC_IMG_UV_BASE_ADDR);
	__raw_writel((overlay_buffer_ptr->src_height<<16)|overlay_buffer_ptr->src_width,LCDC_IMG_SIZE_XY);
	__raw_writel(overlay_buffer_ptr->src_width,LCDC_IMG_PITCH);
	__raw_writel(0x0,LCDC_IMG_DISP_XY);	
}

static void display_done_cb(void *data)
{
	struct disp_buffer_t  *buf = (struct disp_buffer_t  *)data;
	up(buf->sem);
}

static int has_more_than_2_buffers(struct list_head *pList)
{
	struct vivi_buffer *buf;
	int i =0 ;
	list_for_each_entry(buf, pList, vb.queue)
	{
		i++;
	}
	return (i>=2);
}

static void prepare_disp_buffer(struct disp_buffer_t *overlay_buffer_ptr,struct vivi_buffer *buf,struct vivi_dev *dev)
{
	overlay_buffer_ptr->src_Y_addr = buf->vb.baddr;
	overlay_buffer_ptr->src_width    = buf->vb.width; 
	overlay_buffer_ptr->src_height   =  buf->vb.height; 
	overlay_buffer_ptr->crop_left    = buf->crop.left;
	overlay_buffer_ptr->crop_top   =  buf->crop.top;
	overlay_buffer_ptr->crop_width = buf->crop.width;
	overlay_buffer_ptr->crop_height = buf->crop.height;

	overlay_buffer_ptr->img_disp_x = dev->overlay_left;
	overlay_buffer_ptr->img_disp_y = dev->overlay_top;
	overlay_buffer_ptr->img_size_x = dev->overlay_width;
	overlay_buffer_ptr->img_size_y = dev->overlay_height;
	overlay_buffer_ptr->overlay_degree = dev->overlay_degree;
}

static void transform_disp_buffer(struct disp_buffer_t *overlay_buffer_ptr)
{

}

static void send_disp_buffer(struct disp_buffer_t *overlay_buffer_ptr)
{
	int ret = rrm_refresh(LID_VIDEO, display_done_cb, (void *)overlay_buffer_ptr);
	if(ret){
		printk(KERN_INFO "overlay rrm_refresh error\n");
	}
}

static void display_picture(struct vivi_fh *fh)
{
	struct vivi_dev *dev = fh->dev;
	struct vivi_dmaqueue *dma_q = &dev->vidq;
	unsigned long flags = 0;
	struct vivi_buffer *buf_first,*buf_second;
	struct disp_buffer_t *tmp;
	
        dprintk(dev, 1, "%s in\n", __func__);	
	
	spin_lock_irqsave(&dev->slock, flags);
	while(( !has_more_than_2_buffers(&dma_q->active))&&(!kthread_should_stop())){
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(dev->dma_wq,(has_more_than_2_buffers(&dma_q->active)||kthread_should_stop()));
		spin_lock_irqsave(&dev->slock, flags);
	}

        if(kthread_should_stop()){
       	        spin_unlock_irqrestore(&dev->slock, flags);
		return;
        }

 	if(dev->is_first_pic){
 		buf_first = list_entry(dma_q->active.next,
			 struct vivi_buffer, vb.queue);
 		dev->is_first_pic = 0;
		prepare_disp_buffer(dev->overlay_buffer_ptr[1],buf_first,dev);
		spin_unlock_irqrestore(&dev->slock, flags);
		transform_disp_buffer(dev->overlay_buffer_ptr[1]);
 	}else{
  		buf_first = list_entry(dma_q->active.next,
			 struct vivi_buffer, vb.queue);
		list_del(&buf_first->vb.queue);
		buf_first->vb.state = VIDEOBUF_DONE;
		wake_up(&buf_first->vb.done);
  		buf_second= list_entry(dma_q->active.next,
			 struct vivi_buffer, vb.queue);
		
		tmp = dev->overlay_buffer_ptr[1];
		dev->overlay_buffer_ptr[1] = dev->overlay_buffer_ptr[0];
		dev->overlay_buffer_ptr[0] = tmp;

		prepare_disp_buffer(dev->overlay_buffer_ptr[1],buf_second,dev);	
 		spin_unlock_irqrestore(&dev->slock, flags);
		
		down(&dev->disp_done_sem);
		send_disp_buffer(dev->overlay_buffer_ptr[0]);
		transform_disp_buffer(dev->overlay_buffer_ptr[1]);
 	}

       dprintk(dev, 1, "%s out\n", __func__);		
}

static int vivi_thread(void *data)
{
	struct vivi_fh  *fh = data;
	struct vivi_dev *dev = fh->dev;

	dprintk(dev, 1, "thread started\n");	
	//set_freezable();

	for (;;) {
		display_picture(fh);
	
		if (kthread_should_stop())
			break;
	} 

	dprintk(dev, 1, "thread: exit\n");
	return 0;
}

static int vivi_start_thread(struct vivi_fh *fh)
{
	struct vivi_dev *dev = fh->dev;
	struct vivi_dmaqueue *dma_q = &dev->vidq;

//	dma_q->frame = 0;
//	dma_q->ini_jiffies = jiffies;

	dprintk(dev, 1, "%s\n", __func__);

	dma_q->kthread = kthread_run(vivi_thread, fh, "vivi");

	if (IS_ERR(dma_q->kthread)) {
		v4l2_err(&dev->v4l2_dev, "kernel_thread() failed\n");
		return PTR_ERR(dma_q->kthread);
	}
	/* Wakes thread */
	//wake_up_interruptible(&dma_q->wq);

	dprintk(dev, 1, "returning from %s\n", __func__);
	return 0;
}

static void vivi_stop_thread(struct vivi_dmaqueue  *dma_q)
{
	struct vivi_dev *dev = container_of(dma_q, struct vivi_dev, vidq);

	dprintk(dev, 1, "%s\n", __func__);
	
	/* shutdown control thread */
	if (dma_q->kthread) {
		kthread_stop(dma_q->kthread);
		dma_q->kthread = NULL;
	}
	wake_up(&dev->dma_wq);
}

/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/
static int
buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct vivi_fh  *fh = vq->priv_data;
	struct vivi_dev *dev  = fh->dev;

	*size =PAGE_ALIGN( fh->width*fh->height*3/2);

	if (0 == *count)
		*count = 32;

	while (*size * *count > vid_limit * 1024 * 1024)
		(*count)--;

	dprintk(dev, 1, "%s, count=%d, size=%d\n", __func__,
		*count, *size);
	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct vivi_buffer *buf)
{
	struct vivi_fh  *fh = vq->priv_data;
	struct vivi_dev *dev  = fh->dev;

	dprintk(dev, 1, "%s, state: %i\n", __func__, buf->vb.state);
	
	if (in_interrupt())
		BUG();

	videobuf_vmalloc_free(&buf->vb);
	dprintk(dev, 1, "free_buffer: freed\n");
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}


static int
buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct vivi_fh     *fh  = vq->priv_data;
	struct vivi_dev    *dev = fh->dev;
	struct vivi_buffer *buf = container_of(vb, struct vivi_buffer, vb);

	dprintk(dev, 1, "%s, field=%d\n", __func__, field);
	
	BUG_ON(NULL == fh->fmt);

	if (fh->width  < 48 || fh->width  > norm_maxw() ||
	    fh->height < 32 || fh->height > norm_maxh())
		return -EINVAL;

	buf->vb.size =PAGE_ALIGN(  fh->width*fh->height*3/2);
	if (0 != buf->vb.baddr  &&  buf->vb.bsize < buf->vb.size)
	{
		printk(KERN_INFO  "%s:%x,%x,%x\n", __func__, buf->vb.baddr,buf->vb.bsize,buf->vb.size);
		return -EINVAL;
	}	

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = fh->fmt;
	buf->vb.width  = fh->width;
	buf->vb.height = fh->height;
	buf->vb.field  = field;

	buf->crop.left = fh->crop.left;
	buf->crop.top = fh->crop.top;
	buf->crop.width= fh->crop.width;
	buf->crop.height= fh->crop.height;

	//precalculate_bars(fh);
       /*
	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0)
			goto fail;
	}
       */
	buf->vb.state = VIDEOBUF_PREPARED;

	return 0;
}

static void
buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct vivi_buffer    *buf  = container_of(vb, struct vivi_buffer, vb);
	struct vivi_fh        *fh   = vq->priv_data;
	struct vivi_dev       *dev  = fh->dev;
	struct vivi_dmaqueue *vidq = &dev->vidq;

	dprintk(dev, 1, "%s\n", __func__);

	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);

	wake_up(&dev->dma_wq);
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct vivi_buffer   *buf  = container_of(vb, struct vivi_buffer, vb);
	struct vivi_fh       *fh   = vq->priv_data;
	struct vivi_dev      *dev  = (struct vivi_dev *)fh->dev;

	dprintk(dev, 1, "%s\n", __func__);

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops vivi_video_qops = {
	.buf_setup      = buffer_setup,
	.buf_prepare    = buffer_prepare,
	.buf_queue      = buffer_queue,
	.buf_release    = buffer_release,
};

/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct vivi_fh  *fh  = priv;
	struct vivi_dev *dev = fh->dev;

	strcpy(cap->driver, "sprd_overlay");
	strcpy(cap->card, "sprd_overlay");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = VIVI_VERSION;
	cap->capabilities =	V4L2_CAP_VIDEO_OUTPUT |V4L2_CAP_VIDEO_OVERLAY|
				V4L2_CAP_STREAMING  ;
	return 0;
}

#if 0
static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct vivi_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}
#endif 
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct vivi_fh  *fh  = priv;
	struct vivi_dev *dev = fh->dev;
	struct vivi_fmt *fmt;
	enum v4l2_field field;
	unsigned int maxw, maxh;
        dprintk(dev, 1,"%s\n", __func__);
	fmt = get_format(f);
	if (!fmt) {
		dprintk(dev, 1, "Fourcc format (0x%08x) invalid.\n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;

/*
	if (field == V4L2_FIELD_ANY) {
		field = V4L2_FIELD_INTERLACED;
	} else if (V4L2_FIELD_INTERLACED != field) {
		dprintk(dev, 1, "Field type invalid.\n");
		return -EINVAL;
	}
*/
	 if (V4L2_FIELD_NONE != field) {
		dprintk(dev, 1, "Field type invalid.\n");
		return -EINVAL;
	}
	 
	maxw  = norm_maxw();
	maxh  = norm_maxh();

	f->fmt.pix.field = field;
	//v4l_bound_align_image(&f->fmt.pix.width, 48, maxw, 2,
	//		      &f->fmt.pix.height, 32, maxh, 0, 0);
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		PAGE_ALIGN(f->fmt.pix.height * f->fmt.pix.bytesperline*3/2);

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct vivi_fh *fh = priv;

	f->fmt.pix.width        = fh->width;
	f->fmt.pix.height       = fh->height;
	f->fmt.pix.field        = fh->vb_vidq.field;
	f->fmt.pix.pixelformat  = fh->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fh->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		PAGE_ALIGN(f->fmt.pix.height * f->fmt.pix.bytesperline*3/2);

	return (0);
}



/*FIXME: This seems to be generic enough to be at videodev2 */
static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct vivi_fh *fh = priv;
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

	fh->crop.left = 0;
	fh->crop.top = 0;
	fh->crop.width = fh->width;
	fh->crop.height = fh->height;
	
	ret = 0;
out:
	mutex_unlock(&q->vb_lock);

	return ret;
}


static int vidioc_g_fmt_vid_overlay(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct vivi_fh *fh = priv;

	f->fmt.win.w.left           = fh->dev->overlay_left;
	f->fmt.win.w.top           = fh->dev->overlay_top;
	f->fmt.win.w.width        = fh->dev->overlay_width;
	f->fmt.win.w.height       = fh->dev->overlay_height;
	
	return (0);
}



/*FIXME: This seems to be generic enough to be at videodev2 */
static int vidioc_s_fmt_vid_overlay(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct vivi_fh *fh = priv;
	struct vivi_dev *dev = fh->dev;
	unsigned long flags = 0;
	
	spin_lock_irqsave(&dev->slock, flags);
	
	fh->dev->overlay_left = f->fmt.win.w.left;
	fh->dev->overlay_top = f->fmt.win.w.top;
	fh->dev->overlay_width = f->fmt.win.w.width;
	fh->dev->overlay_height = f->fmt.win.w.height;

	spin_unlock_irqrestore(&dev->slock, flags);

	return 0;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct vivi_fh *fh = priv;
        dprintk(fh->dev, 1, "%s\n", __func__);
	return (videobuf_qbuf(&fh->vb_vidq, p));
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct vivi_fh  *fh = priv;
        dprintk(fh->dev, 1, "%s\n", __func__);
	return (videobuf_dqbuf(&fh->vb_vidq, p,
				file->f_flags & O_NONBLOCK));
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct vivi_fh  *fh = priv;

	return (videobuf_reqbufs(&fh->vb_vidq, p));
}


static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct vivi_fh  *fh = priv;

	return (videobuf_querybuf(&fh->vb_vidq, p));
}

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct vivi_fh  *fh = priv;
        dprintk(fh->dev, 1,"%s\n", __func__);
	if (fh->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;
	
        if(!fh->vb_vidq.streaming)
		fh->dev->is_first_pic = 1;

	return videobuf_streamon(&fh->vb_vidq);
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct vivi_fh  *fh = priv;
        dprintk(fh->dev, 1,"%s\n", __func__);
	if (fh->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;

	return videobuf_streamoff(&fh->vb_vidq);
}


/* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vivi_qctrl); i++)
		if (qc->id && qc->id == vivi_qctrl[i].id) {
			memcpy(qc, &(vivi_qctrl[i]),
				sizeof(*qc));
			return (0);
		}

	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct vivi_fh *fh = priv;
	struct vivi_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(vivi_qctrl); i++)
		if (ctrl->id == vivi_qctrl[i].id) {
			ctrl->value = dev->qctl_regs[i];
			return 0;
		}

	return -EINVAL;
}
static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct vivi_fh *fh = priv;
	struct vivi_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(vivi_qctrl); i++)
		if (ctrl->id == vivi_qctrl[i].id) {
			if (ctrl->value < vivi_qctrl[i].minimum ||
			    ctrl->value > vivi_qctrl[i].maximum) {
				return -ERANGE;
			}
			dev->qctl_regs[i] = ctrl->value;
			if(ctrl->id ==V4L2_CID_ROTATE )
			{
				dev->overlay_degree = ctrl->value;
			}
			return 0;
		}
	return -EINVAL;
}


/* cropping (sub-frame capture) */
static int vidioc_cropcap(struct file *file, void *priv,
					struct v4l2_cropcap *cropcap)
{
	return 0;
}

static int vidioc_g_crop (struct file *file, void *priv,
					struct v4l2_crop *a)
{
	struct vivi_fh *fh = priv;

	a->c.left = fh->crop.left;
	a->c.top = fh->crop.top;
	a->c.width = fh->crop.width;
	a->c.height = fh->crop.height;

	return (0);
}
static int vidioc_s_crop (struct file *file, void *priv,
					struct v4l2_crop *a)
{
	struct vivi_fh *fh = priv;
	struct videobuf_queue *q = &fh->vb_vidq;
	int ret;
	mutex_lock(&q->vb_lock);

	if (videobuf_queue_is_busy(&fh->vb_vidq)) {
		dprintk(fh->dev, 1, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}

	fh->crop.left = a->c.left;
	fh->crop.top = a->c.top;
	fh->crop.width =a->c.width;
	fh->crop.height = a->c.height;
	
	ret = 0;
out:
	mutex_unlock(&q->vb_lock);

	return ret;
}
	
/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/

static int vivi_open(struct file *file)
{
	struct vivi_dev *dev = video_drvdata(file);
	struct vivi_fh *fh = NULL;
	int retval = 0;

	mutex_lock(&dev->mutex);
	dev->users++;

	if (dev->users > 1) {
		dev->users--;
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	dprintk(dev, 1, "open /dev/video%d type=%s users=%d\n", dev->vfd->num,
		v4l2_type_names[V4L2_BUF_TYPE_VIDEO_OVERLAY], dev->users);

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

	fh->type     = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	fh->fmt      = &formats[0];
	fh->width    = DEFAULT_IMAGE_W;
	fh->height   = DEFAULT_IMAGE_H;

	fh->crop.left = 0;
	fh->crop.top = 0;
	fh->crop.width = fh->width;
	fh->crop.height = fh->height;
	
	dev->overlay_left    = 0;
	dev->overlay_top    = 0;
	dev->overlay_width = DEFAULT_IMAGE_W;
	dev->overlay_height = DEFAULT_IMAGE_H;
	dev->overlay_degree = 0;
	dev->is_first_pic = 1;

	/* Resets frame counters */
/*	
	dev->h = 0;
	dev->m = 0;
	dev->s = 0;
	dev->ms = 0;
	dev->mv_count = 0;
	dev->jiffies = jiffies;
	sprintf(dev->timestr, "%02d:%02d:%02d:%03d",
			dev->h, dev->m, dev->s, dev->ms);
*/
	videobuf_queue_vmalloc_init(&fh->vb_vidq, &vivi_video_qops,
			NULL, &dev->slock, fh->type, V4L2_FIELD_NONE,
			sizeof(struct vivi_buffer), fh);

	vivi_start_thread(fh);

	return 0;
}




static int vivi_close(struct file *file)
{
	struct vivi_fh         *fh = file->private_data;
	struct vivi_dev *dev       = fh->dev;
	struct vivi_dmaqueue *vidq = &dev->vidq;

	int minor = video_devdata(file)->minor;

	vivi_stop_thread(vidq);
	videobuf_stop(&fh->vb_vidq);
	videobuf_mmap_free(&fh->vb_vidq);

	kfree(fh);

	mutex_lock(&dev->mutex);
	dev->users--;
	mutex_unlock(&dev->mutex);

	dprintk(dev, 1, "close called (minor=%d, users=%d)\n",
		minor, dev->users);

	return 0;
}


static const struct v4l2_file_operations vivi_fops = {
	.owner		= THIS_MODULE,
	.open           = vivi_open,
	.release        = vivi_close,
	//.read           = vivi_read,
	//.poll		= vivi_poll,
	.ioctl          = video_ioctl2, /* V4L2 ioctl handler */
	//.mmap           = vivi_mmap,
};

static const struct v4l2_ioctl_ops vivi_ioctl_ops = {
	.vidioc_querycap      = vidioc_querycap,
	//.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	//.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	//.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	//.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs       = vidioc_reqbufs,
	.vidioc_querybuf      = vidioc_querybuf,
	.vidioc_qbuf          = vidioc_qbuf,
	.vidioc_dqbuf         = vidioc_dqbuf,
	//.vidioc_s_std         = vidioc_s_std,
	//.vidioc_enum_input    = vidioc_enum_input,
	//.vidioc_g_input       = vidioc_g_input,
	//.vidioc_s_input       = vidioc_s_input,	
	.vidioc_queryctrl     = vidioc_queryctrl,
	.vidioc_g_ctrl        = vidioc_g_ctrl,
	.vidioc_s_ctrl        = vidioc_s_ctrl,
	.vidioc_streamon      = vidioc_streamon,
	.vidioc_streamoff     = vidioc_streamoff,
	
	.vidioc_cropcap = vidioc_cropcap,	
	.vidioc_g_crop = vidioc_g_crop,
	.vidioc_s_crop = vidioc_s_crop,	

	.vidioc_g_fmt_vid_out     = vidioc_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out     = vidioc_s_fmt_vid_out,

	.vidioc_g_fmt_vid_overlay     = vidioc_g_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay     = vidioc_s_fmt_vid_overlay,	
};

static struct video_device vivi_template = {
	.name		= "sprd_overlay",
	.fops           = &vivi_fops,
	.ioctl_ops 	= &vivi_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,

	.tvnorms              = V4L2_STD_525_60,
	.current_norm         = V4L2_STD_NTSC_M,
};

/* -----------------------------------------------------------------
	Initialization and module stuff
   ------------------------------------------------------------------*/

static int vivi_release(void)
{
	struct vivi_dev *dev;
	struct list_head *list;

	while (!list_empty(&vivi_devlist)) {
		list = vivi_devlist.next;
		list_del(list);
		dev = list_entry(list, struct vivi_dev, vivi_devlist);

		v4l2_info(&dev->v4l2_dev, "unregistering /dev/video%d\n",
			dev->vfd->num);
		video_unregister_device(dev->vfd);
		v4l2_device_unregister(&dev->v4l2_dev);
		kfree(dev);
	}

	return 0;
}

static int __init vivi_create_instance(int inst)
{
	struct vivi_dev *dev;
	struct video_device *vfd;
	int ret, i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
			"%s-%03d", VIVI_MODULE_NAME, inst);
	ret = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (ret)
		goto free_dev;

	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	init_waitqueue_head(&dev->vidq.wq);

	/* initialize locks */
	spin_lock_init(&dev->slock);
	mutex_init(&dev->mutex);

	sema_init(&dev->disp_done_sem,1);
	init_waitqueue_head(&dev->dma_wq);


	dev->overlay_buffer[0].sem = &dev->disp_done_sem;
	dev->overlay_buffer[1].sem = &dev->disp_done_sem;
	dev->overlay_buffer_ptr[0] = &dev->overlay_buffer[0];
	dev->overlay_buffer_ptr[1] = &dev->overlay_buffer[1];

        ret = rrm_layer_init(LID_VIDEO,VIDEO_LAYER_BUF_NUM,rrm_set_layer_cb);
	if (ret){
		printk(KERN_INFO "Error %d while overlay rrm_layer_init\n", ret);
		goto free_dev;
	}else{
		printk(KERN_INFO "overlay rrm_layer_init ok\n");
	}

	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;

	*vfd = vivi_template;
	vfd->debug = debug;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		goto rel_vdev;

	video_set_drvdata(vfd, dev);

	/* Set all controls to their default value. */
	for (i = 0; i < ARRAY_SIZE(vivi_qctrl); i++)
		dev->qctl_regs[i] = vivi_qctrl[i].default_value;

	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->vivi_devlist, &vivi_devlist);

	snprintf(vfd->name, sizeof(vfd->name), "%s (%i)",
			vivi_template.name, vfd->num);

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
static int __init vivi_init(void)
{
	int ret = 0, i;

	if (n_devs <= 0)
		n_devs = 1;

	for (i = 0; i < n_devs; i++) {
		ret = vivi_create_instance(i);
		if (ret) {
			/* If some instantiations succeeded, keep driver */
			if (i)
				ret = 0;
			break;
		}
	}

	if (ret < 0) {
		printk(KERN_INFO "Error %d while loading sprd overlay driver\n", ret);
		return ret;
	}

	printk(KERN_INFO "SPRD Overlay"
			"SPRD Overlay  ver %u.%u.%u successfully loaded.\n",
			(VIVI_VERSION >> 16) & 0xFF, (VIVI_VERSION >> 8) & 0xFF,
			VIVI_VERSION & 0xFF);

	/* n_devs will reflect the actual number of allocated devices */
	n_devs = i;

	return ret;
}

static void __exit vivi_exit(void)
{
	vivi_release();
}

module_init(vivi_init);
module_exit(vivi_exit);
