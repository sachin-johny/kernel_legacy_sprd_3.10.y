#ifndef _MACH_USB_H
#define _MACH_USB_H

struct usb_hotplug_callback {
	int (*plugin)(int usb_cable, void *data);
	int (*plugout)(int usb_cable, void *data);
	void *data;
};

extern int usb_register_hotplug_callback(struct usb_hotplug_callback *cb);
#endif
