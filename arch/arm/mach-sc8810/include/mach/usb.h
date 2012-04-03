#ifndef _MACH_USB_H
#define _MACH_USB_H

extern void udc_enable(void);
extern void udc_disable(void);
extern void udc_phy_down(void);
extern void udc_phy_up(void);

struct usb_hotplug_callback {
	int (*plugin)(int usb_cable, void *data);
	int (*plugout)(int usb_cable, void *data);
	void *data;
};

extern int usb_register_hotplug_callback(struct usb_hotplug_callback *cb);
extern int usb_alloc_vbus_irq(void);
extern void usb_free_vbus_irq(int irq);
extern int usb_get_vbus_irq(void);
extern int usb_get_vbus_state(void);

enum vbus_irq_type {
	VBUS_PLUG_IN,
	VBUS_PLUG_OUT
};

extern void usb_set_vbus_irq_type(int irq, int irq_type);
#endif
