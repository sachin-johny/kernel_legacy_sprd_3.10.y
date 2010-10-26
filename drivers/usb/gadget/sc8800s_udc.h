/*
 * linux/drivers/usb/gadget/sc8800_udc.h
 * SpreadTrum on-chip full speed USB device controller
 *
 * Copyright (C) 2010 Yingchun Li yingchun.li@spreadtrum.com
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef	_LINUX_USB_GADGET_SC8800_
#define	_LINUX_USB_GADGET_SC8800_

#define		MAX_FIFO_SIZE				(64)
#define		SC8800_UDC_NUM_EPS	3
#define		EP0_PACKETSIZE					MAX_FIFO_SIZE
#define		BULK_PACKETSIZE				MAX_FIFO_SIZE

typedef enum ep_type {
	ep_control		= 0, 
	ep_bulk_in		= 1, 
	ep_bulk_out		= 2, 
	ep_interrupt	= 3
}ep_type_t;

typedef enum ep0_state { 
	EP0_IDLE			= 0,
	EP0_IN_DATA_PHASE	= 1,
	EP0_OUT_DATA_PHASE	= 2,
	EP0_END_XFER		= 3,
	EP0_STALL			= 4,
}ep0_state_t;


struct	sc8800_udc;

struct	sc8800_request
{
	struct usb_request	req;
	struct list_head	queue;
	unsigned 	mapped:1;
};

struct	sc8800_ep
{
	struct	usb_ep		ep;
	struct	sc8800_udc	*dev;
	
	const struct	usb_endpoint_descriptor	*desc;
	struct	list_head	queue;
	
	unsigned 				dma:1,
						is_in:1,
						stopped:1;
	u8				bEndpointAddress;
	u8 				bmAttributes;
	unsigned 			fifo_in, fifo_out;
	/* physical addres for dma */
	unsigned 			pfifo_in, pfifo_out;
	u8				delayed;
};

struct	sc8800_udc
{
	struct	usb_gadget			gadget;
	struct	usb_gadget_driver	*driver;

	struct	platform_device	*pdev;

	int			connected;
	spinlock_t	lock;
	ep0_state_t		ep0state;
	int			pending;
	int			configured;

	struct	sc8800_ep			ep[SC8800_UDC_NUM_EPS];
};

static inline struct sc8800_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct sc8800_udc, gadget);
}

#endif

