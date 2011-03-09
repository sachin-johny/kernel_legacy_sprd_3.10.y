/*
 * Gadget Driver for Android ADB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 #define DEBUG 
 #define VERBOSE_DEBUG 

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/android_composite.h>

#include <linux/sched.h>
#define BULK_BUFFER_SIZE           4096

/* number of tx requests to allocate */
#define TX_REQ_MAX 4

static const char shortname[] = "android_modem";

struct modem_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	int error;

	struct usb_request *rx_req;
	int rx_done;
};

static struct usb_interface_descriptor modem_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	/*DYNAMIC*/
	//.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = 0,
	.bInterfaceProtocol     = 0,
};

static struct usb_endpoint_descriptor modem_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor modem_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor modem_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor modem_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_modem_descs[] = {
	(struct usb_descriptor_header *) &modem_interface_desc,
	(struct usb_descriptor_header *) &modem_fullspeed_in_desc,
	(struct usb_descriptor_header *) &modem_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_modem_descs[] = {
	(struct usb_descriptor_header *) &modem_interface_desc,
	(struct usb_descriptor_header *) &modem_highspeed_in_desc,
	(struct usb_descriptor_header *) &modem_highspeed_out_desc,
	NULL,
};


/* temporary variable used between modem_open() and modem_gadget_bind() */
static struct modem_dev *_modem_dev;

static inline struct modem_dev *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct modem_dev, function);
}


static struct usb_request *modem_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void modem_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}


static void modem_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct modem_dev *dev = _modem_dev;

	if (req->status != 0)
		dev->error = 1;

	//req_put(dev, &dev->tx_idle, req);

//	wake_up(&dev->write_wq);
}

static void modem_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct modem_dev *dev = _modem_dev;

	dev->rx_done = 1;
	if (req->status != 0)
		dev->error = 1;

//	wake_up(&dev->read_wq);
}

static int __init create_bulk_endpoints(struct modem_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for modem ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

#if 0
	/* now allocate requests for our endpoints */
	req = modem_request_new(dev->ep_out, BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = modem_complete_out;
	dev->rx_req = req;

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = modem_request_new(dev->ep_in, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = modem_complete_in;
		req_put(dev, &dev->tx_idle, req);
	}
#endif
	return 0;

fail:
	printk(KERN_ERR "modem_bind() could not allocate requests\n");
	return -1;
}

static int modem_function_setup(struct usb_function *f,
					const struct usb_ctrlrequest *ctrl)
{

	struct modem_dev	*modem = func_to_dev(f);
	struct usb_composite_dev *cdev = modem->cdev;

	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	DBG(cdev, "modem_function_setup\n");
	/* Handle Bulk-only class-specific requests */
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
	DBG(cdev, "USB_TYPE_CLASS\n");
		switch (ctrl->bRequest) {
		default:
			VDBG(cdev, "modem setup\n");
			value = -EOPNOTSUPP;
			break;
		}
	}

		/* respond with data transfer or status phase? */
		if (value >= 0) {
			int rc;
			cdev->req->zero = value < w_length;
			cdev->req->length = value;
			rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
			if (rc < 0)
				printk("%s setup response queue error\n", __func__);
		}

	if (value == -EOPNOTSUPP)
		VDBG(cdev,
			"unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			le16_to_cpu(ctrl->wValue), w_index, w_length);
	return value;
}

static int
modem_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct modem_dev	*dev = func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "modem_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	modem_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(dev, &modem_fullspeed_in_desc,
			&modem_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		modem_highspeed_in_desc.bEndpointAddress =
			modem_fullspeed_in_desc.bEndpointAddress;
		modem_highspeed_out_desc.bEndpointAddress =
			modem_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
modem_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct modem_dev	*dev = func_to_dev(f);
	struct usb_request *req;

	spin_lock_irq(&dev->lock);

/*
	modem_request_free(dev->rx_req, dev->ep_out);
	while ((req = req_get(dev, &dev->tx_idle)))
		modem_request_free(req, dev->ep_in);
*/
	dev->online = 0;
	dev->error = 1;
	spin_unlock_irq(&dev->lock);
/*
	misc_deregister(&modem_device);
	misc_deregister(&modem_enable_device);
*/
	kfree(_modem_dev);
	_modem_dev = NULL;
}

static int modem_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct modem_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "modem_function_set_alt intf: %d alt: %d\n", intf, alt);
	DBG(cdev, "choose speed: %d\n",  cdev->gadget->speed);
	ret = usb_ep_enable(dev->ep_in,
			ep_choose(cdev->gadget,
				&modem_highspeed_in_desc,
				&modem_fullspeed_in_desc));

	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out,
			ep_choose(cdev->gadget,
				&modem_highspeed_out_desc,
				&modem_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->online = 1;

	return 0;
}

static void modem_function_disable(struct usb_function *f)
{
	struct modem_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "modem_function_disable\n");
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

#include <nk/nkern.h>

static NkXIrqId		modem_sysconf_id;
static NkXIrq backend_irq;
static NkOsId server_id;
struct function_log_state {
	int state;
};
struct function_log_state *logel_state;

static int modem_bind_config(struct usb_configuration *c)
{
	struct modem_dev *dev;
	int ret;

	printk(KERN_INFO "modem_bind_config\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	dev->cdev = c->cdev;
	dev->function.name = "modem";
	dev->function.descriptors = fs_modem_descs;
	dev->function.hs_descriptors = hs_modem_descs;
	dev->function.bind = modem_function_bind;
	dev->function.unbind = modem_function_unbind;
	dev->function.set_alt = modem_function_set_alt;
	dev->function.disable = modem_function_disable;
	dev->function.setup = modem_function_setup;

	/* _modem_dev must be set before calling usb_gadget_register_driver */
	_modem_dev = dev;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err1;

	logel_state->state = 0x55aa;
	pr_info("notify mocor\n");
	nkops.nk_xirq_trigger(backend_irq, server_id);
	return 0;

err1:
	kfree(dev);
	printk(KERN_ERR "modem gadget driver failed to initialize\n");
	return ret;
}

static struct android_usb_function modem_function = {
	.name = "modem",
	.bind_config = modem_bind_config,
};



static void vlog_sysconf_hdl(void* cookie, NkXIrq xirq)
{
	pr_info("%s \n", __func__);
}
static int __init init(void)
{
	int myid;
	NkPhAddr plink;
    	NkDevVlink* vlink;
	NkPhAddr paddr;
	
	printk(KERN_INFO "f_modem init\n");
	android_register_function(&modem_function);
	modem_sysconf_id = nkops.nk_xirq_attach(NK_XIRQ_SYSCONF, vlog_sysconf_hdl, 0);
	if (modem_sysconf_id == 0) {
		pr_info( "cannot attach sysconf id\n");
	}

	myid = nkops.nk_id_get();
	 while ((plink = nkops.nk_vlink_lookup("vlog", plink))) {
        	vlink = nkops.nk_ptov(plink);
		server_id = vlink->s_id;
		printnk("vlog s_id %d, c_id : %d\n", vlink->s_id, vlink->c_id);
		if (vlink->link == 0) {
			if (vlink->c_id != myid) {
				pr_info("error vlink\n");
				return -1;
			}
			break;
		}
	 }

	paddr = nkops.nk_pdev_alloc(plink, 0, sizeof (struct function_log_state));
	if (!paddr) {
		pr_info("cannot alloc mem for vlink\n");
		return -1;
	}
	logel_state = (struct function_log_state *) nkops.nk_ptov(paddr);
	
	 backend_irq = nkops.nk_pxirq_alloc(plink, 1, vlink->s_id, 1);
	printk("backend irq for log is:%d\n", backend_irq);
	return 0;
}
module_init(init);
//late_initcall(init);

