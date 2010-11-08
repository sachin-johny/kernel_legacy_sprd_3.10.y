/*
 * linux/drivers/usb/gadget/sc8800_udc.c
 * SpreadTrum on-chip full speed USB device controllers
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
 *
 */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#define PLATFORM_SC8800H
#include <mach/dma.h>
#include <mach/regs_int.h>
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/ldo.h>
#include <mach/power_manager.h>

#include "regs_usb.h"
#include "sc8800s_udc.h"



#define	DRIVER_VERSION		"Aug 6, 2010"
#define	DRIVER_DESC		"SC8800 USB Device Controller driver"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

static const char	driver_name[]	= "sc8800_udc";
static const char	ep0name[]		= "ep0";

#define TRUE		1
#define FALSE	0

#define USB_12M_CLK 		0x00
#define USB_48M_CLK 		0x01
#define USB_MAX_CLK 		0x02

#define USB_LDO_ENABLE    				(0x2)
#define USB_LDO_DISABLE   				(0x1)

/* used for debug */
static int	g_use_dma;
static unsigned int	log_buf[1024]		= {0};

#undef DEBUG
/*
#define DEBUG(fmt,args...) \
	pr_debug(fmt, ## args)
*/
//#define DEBUG(fmt, args...) trace_printk(fmt, ## args)

#define DEBUG(fmt, args...) 

/* The current configuration of sc8800_udc */
static struct sc8800_udc	the_controller;

#define ep_is_in(EP)		(((EP)->bEndpointAddress & USB_DIR_IN) == USB_DIR_IN)
#define ep_index(EP) 		((EP)->bEndpointAddress & 0x7F)

static inline
unsigned int	usb_read(unsigned int port)
{
	return __raw_readl(port);
}

static inline
void	usb_write(unsigned int val, unsigned int port)
{
	__raw_writel(val, port);
}

static inline
void	usb_set(unsigned int val, unsigned int port)
{
	__raw_bits_or(val, port);
}

static inline
void	usb_clear(unsigned int val, unsigned int port)
{
	__raw_bits_and((~val), port);
}

static void dump_regs(void)
{
	unsigned	dev_sts;
	
	dev_sts = usb_read(USB_DEV_STS);

	#define	PRE	"udc regs:"

	DEBUG(PRE"GR_CLK_GEN5:0x%x\n", usb_read(GR_CLK_GEN5));
	DEBUG(PRE"AHB_CTL0:0x%x\n", usb_read(AHB_CTL0));
	DEBUG(PRE"GR_GEN0:0x%x\n", usb_read(GR_GEN0));
	DEBUG(PRE"GR_CLK_EN:0x%x\n", usb_read(GR_CLK_EN));
	DEBUG(PRE"GR_BUSCLK_ALM:0x%x\n", usb_read(GR_BUSCLK_ALM));

	DEBUG(PRE"USB_DEV_CTRL:0x%x\n", usb_read(USB_DEV_CTRL));
	DEBUG(PRE"USB_DEV_STS:0x%x\n", usb_read(USB_DEV_STS));
	DEBUG(PRE"USB_INT_CTRL:0x%x\n", usb_read(USB_INT_CTRL));
	DEBUG(PRE"USB_INT_STS:0x%x\n", usb_read(USB_INT_STS));
	DEBUG(PRE"USB_INT_RAW:0x%x\n", usb_read(USB_INT_RAW));
	DEBUG(PRE"USB_FRAME_NUM:0x%x\n", usb_read(USB_FRAME_NUM));
	DEBUG(PRE"USB_CONFIG:0x%x\n", usb_read(USB_CONFIG));
	DEBUG(PRE"USB_ALT_SETTING:0x%x\n", usb_read(USB_SETTING));
	DEBUG(PRE"USB_TIME_SET:0x%x\n", usb_read(USB_TIME_SET));
	
//	DEBUG(PRE"\n");
	DEBUG(PRE"USB_TRANSF_SIZE_EP0_IN:0x%x\n", usb_read(USB_TRANSF_SIZE_EP0_IN));
	DEBUG(PRE"USB_TRANSF_SIZE_EP0_OUT:0x%x\n", usb_read(USB_TRANSF_SIZE_EP0_OUT));
	DEBUG(PRE"USB_SEND_DATA_NUM_EP0:0x%x\n", usb_read(USB_SEND_DATA_NUM_EP0));
		

	DEBUG(PRE"USB_EP0_CTRL:0x%x\n", usb_read(USB_EP0_CTRL));
	DEBUG(PRE"USB_EP0_INT_CTRL:0x%x\n", usb_read(USB_EP0_INT_CTRL));
	DEBUG(PRE"USB_EP0_INT_STS:0x%x\n", usb_read(USB_EP0_INT_STS));
	DEBUG(PRE"USB_EP0_INT_RAW:0x%x\n", usb_read(USB_EP0_INT_RAW));
	DEBUG(PRE"USB_RECEIVED_DATA_NUM_EP0:0x%x\n", usb_read(USB_RECEIVED_DATA_NUM_EP0));

	DEBUG(PRE"USB_EP1_CTRL:0x%x\n", usb_read(USB_EPx_CTRL(1)));
	DEBUG(PRE"USB_EP1_INT_CTRL:0x%x\n", usb_read(USB_EPx_INT_CTRL(1)));
	DEBUG(PRE"USB_EP1_SEND_DATA:0x%x\n", usb_read(USB_EPx_XFER_DATA(1)));
	DEBUG(PRE"USB_EP1_TRANSFER_SIZE:0x%x\n",usb_read(USB_EPx_XFER_SIZE(1)));
	DEBUG(PRE"USB_EP1_INT_STS:0x%x\n", usb_read(USB_EPx_INT_STS(1)));
	DEBUG(PRE"USB_EP1_INT_RAW:0x%x\n", usb_read(USB_EPx_INT_RAW(1)));

	DEBUG(PRE"USB_EP2_CTRL:0x%x\n", usb_read(USB_EPx_CTRL(2)));
	DEBUG(PRE"USB_EP2_INT_CTRL:0x%x\n", usb_read(USB_EPx_INT_CTRL(2)));
	DEBUG(PRE"USB_EP2_RECEIVED_DATA:0x%x\n", usb_read(USB_EPx_XFER_DATA(2)));
	DEBUG(PRE"USB_EP2_TRANSFER_SIZE:0x%x\n",usb_read(USB_EPx_XFER_SIZE(2)));
	DEBUG(PRE"USB_EP2_INT_STS:0x%x\n", usb_read(USB_EPx_INT_STS(2)));
	DEBUG(PRE"USB_EP2_INT_RAW:0x%x\n", usb_read(USB_EPx_INT_RAW(2)));

	DEBUG("Device Statement: addr=%u; state=%u; interface=%u; ep_num=%u\n",
		dev_sts & 0x7f,
		(dev_sts >> 7) & 0x07,
		(dev_sts >> 10) & 0x1f,
		(dev_sts >> 15) & 0x0f);
		
	switch ((dev_sts >> 7) & 0x07) {
		case 0:
			DEBUG("$$$$$$$  ATTACHED_STATE  $$$$$$$$$$$$$$$\n");
			break;			
		case 1:
			DEBUG("$$$$$$$  DEFAULT_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 2:
			DEBUG("$$$$$$$  ADDRESS_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 3:
			DEBUG("$$$$$$$  CONFIGURE_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 4:
			DEBUG("$$$$$$$  S_From_ATTACHED_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 5:
			DEBUG("$$$$$$$  S_From_DEFAULT_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 6:
			DEBUG("$$$$$$$  S_From_ADDRESS_STATE  $$$$$$$$$$$$$$$\n");
			break;
		case 7:
			DEBUG("$$$$$$$  S_From_CONFIGURE_STATE  $$$$$$$$$$$$$$$\n");
			break;
		default:
			DEBUG("########   #unknown statement ############\n");
			break;
		}
}

static void dump_req(struct usb_request *req)
{
	DEBUG("REQ buf %p len %d dma 0x%08x noi=%d zp=%d snok=%d\n",
			req->buf, req->length, req->dma,
			req->no_interrupt, req->zero, req->short_not_ok);
}

enum {
	RECEIVED_DATA,
	SENT_DATA
};
static void dump_log(char * buf, int len, int direction)
{ 
	int i = 0;
	
	DEBUG("**log_buf :%s data...\r\n", (direction ? "sent" : "received"));
	for (i = 0; i < len; i++)	{
		DEBUG("%2x\r\n", *((unsigned char*)buf+i) );
	}
}
static void USB_EnableClk(unsigned int clk_val, unsigned int pll_clk)
{
    unsigned int  clk_divider;

    BUG_ON(clk_val >= USB_MAX_CLK);

    if (USB_12M_CLK == clk_val) {
	BUG_ON (!(0 == (pll_clk % USB_CLK_12M)));

	clk_12M_divider_set(pll_clk);
    } else if (USB_48M_CLK == clk_val) {

        BUG_ON(!(0 == (pll_clk % USB_CLK_48M)));

        //Set 48m clk divider;
        //48M_div[4:0]   
        clk_divider = __raw_readl(GR_CLK_GEN5);
        clk_divider   &=  ~(0x1f);
        clk_divider   |= (pll_clk / USB_CLK_48M - 1);
        __raw_writel(clk_divider, GR_CLK_GEN5);
    }	
}


static void USB_SetPllDividor(void)
{
    unsigned int  pll_clk ;
  
    pll_clk = CHIP_GetVPllClk();

    /*
    * add code here, to check whether the pll clock is valid
    * according to usb 12Mclk,48Mclk
    */
    USB_EnableClk(USB_12M_CLK,pll_clk);
    USB_EnableClk(USB_48M_CLK,pll_clk);
}

/*
	reset the usb module, include 12M clock reset and 48M clock reset
	NOTE: this function will expend 30 ms so it can't be used in 
                  interrupt handler ;
*/
static void usb_soft_reset (void)
{
	#define USB_SOFT_RESET_BIT				(BIT_6|BIT_7)
	
    //*(volatile unsigned int*)AHB_SOFT_RST &= ~USB_SOFT_RESET_BIT;
    	usb_clear(USB_SOFT_RESET_BIT, AHB_SOFT_RST);
	msleep(10);

    //*(volatile unsigned int*)AHB_SOFT_RST |= USB_SOFT_RESET_BIT;
	usb_set(USB_SOFT_RESET_BIT, AHB_SOFT_RST);
	msleep(10);

    //*(volatile unsigned int*)AHB_SOFT_RST &= ~USB_SOFT_RESET_BIT;
	usb_clear(USB_SOFT_RESET_BIT, AHB_SOFT_RST);
   	msleep(10);
}

static inline
void	usb_ldo_switch(int flag)
{
	if(flag) { 
	    LDO_TurnOnLDO(LDO_LDO_USB);
	} else { 
	    LDO_TurnOffLDO(LDO_LDO_USB);
	}
}

static  void usb_connect (void)
{
    /*pll init*/
    USB_SetPllDividor();
    //Enable USB LDO
    usb_ldo_switch(TRUE);
	
    pr_info("USB LDO enable  ! ");
}

/*
	enable/disable the clock for usb module
*/
static inline
void	usb_mainclock_enable(int flag)
{
	if (flag)
		usb_set(BIT_5, AHB_CTL0);
	else
		usb_clear(BIT_5, AHB_CTL0);
}

static inline
void	enable_usb_device(int flag)
{
	if (flag)
		usb_set(USB_DEV_EN, USB_DEV_CTRL);
	else
		usb_clear(USB_DEV_EN, USB_DEV_CTRL);
}

static inline
void	enable_usb_irq(int flag)
{

	if (flag)
		enable_irq(IRQ_USBD_INT);
	else
		disable_irq(IRQ_USBD_INT);
	
}

/*
	enable or disable endpoint, except for ep0;ep0 is always ready;
*/
static inline
void	__enable_udc_ep(struct sc8800_ep *ep, int en)
{	
	int index = ep_index(ep);
	
	if ((index <= 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	
	if (en)
		usb_set(USB_CTL_EPx_ENABLE, USB_EPx_CTRL(index));
	else
		usb_clear(USB_CTL_EPx_ENABLE, USB_EPx_CTRL(index));
}

static 
void	__set_ep_irqs(struct sc8800_ep *ep, unsigned int_bits)
{
	int index = ep_index(ep);
	unsigned int_ctrl;
	
	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	
	switch (index) {
	case 0:
		int_ctrl = USB_EP0_INT_CTRL;
		break;
	default:
		int_ctrl = USB_EPx_INT_CTRL(index);
		break;
	}
	usb_set(int_bits, int_ctrl);		
}

static 
void	__mask_ep_irqs(struct sc8800_ep *ep, unsigned int_bits)
{
	int index = ep_index(ep);
	unsigned int_ctrl;
	
	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	
	switch (index) {
	case 0:
		int_ctrl = USB_EP0_INT_CTRL;
		break;
	default:
		int_ctrl = USB_EPx_INT_CTRL(index);
		break;
	}
	usb_clear(int_bits, int_ctrl);	
}

/*
	Clear endpoint's irqs status
*/
static 
void	__clear_ep_irqsts(struct sc8800_ep *ep, unsigned int_bits)
{
	int index = ep_index(ep);
	unsigned int_clr;
	
	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	
	switch (index) {
	case 0:
		int_clr = USB_EP0_INT_CLR;
		break;
	default:
		int_clr = USB_EPx_INT_CLR(index);
		break;
	}
	usb_write(int_bits, int_clr);	
}

static int __set_ep_max_payload(struct sc8800_ep *ep, unsigned payload)
{
	unsigned  value;
	unsigned long flags;
	unsigned ctrl_reg = 0;
	int index = ep_index(ep);
	
#define EP_MAX_PKT_SIZE_OFFSET  12
#define EP_MAX_PKT_SIZE_MASK	   (0x7ff << EP_MAX_PKT_SIZE_OFFSET)

	if (payload > MAX_FIFO_SIZE) {
		pr_warning("too large payload for ep\r\n");
		return -1;
	}

	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return -1;
	}
	
	switch(index) {
	case 0:
		ctrl_reg = USB_EP0_CTRL;
		break;
	//endpoint 1~4
	default:		
		ctrl_reg = USB_EPx_CTRL(index);
		break;		
	}
	local_irq_save(flags);
	value = usb_read(ctrl_reg);
	value &= ~EP_MAX_PKT_SIZE_MASK;
	value |= (payload & 0x7ff) << EP_MAX_PKT_SIZE_OFFSET;
	usb_write(value, ctrl_reg);

	local_irq_restore(flags);
	//DEBUG("ctrl_reg :%x, value: %x\r\n", ctrl_reg, usb_read(ctrl_reg));
	return 0;
}

static int __set_ep_transfer_size(struct sc8800_ep *ep, unsigned int transfer_size)
{
	unsigned reg = 0;
	int index= ep_index(ep);
	
	if (transfer_size > 0x1ffff) {
		pr_warning("too large transfer size for ep\r\n");
		return -1;
	}

	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return -1;
	}
	
	switch(index) {	
	case 0:
		if (ep->is_in)
			reg = USB_TRANSF_SIZE_EP0_IN;
		else
			reg = USB_TRANSF_SIZE_EP0_OUT;
		break;
	//endpoint 1~4
	default:
		reg = USB_EPx_XFER_SIZE(index);			
		break;
	}
	usb_write(transfer_size, reg);
	//DEBUG("reg :%x, transfer size: %x\r\n", usb_read(reg), transfer_size);
	return 0;
}
/*
	Enable/Disable endpoint ready for IO.
*/
static void __set_ep_ready(struct sc8800_ep *ep, int en)
{
	int index = ep_index(ep);

	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	DEBUG("%s endpoint ready:%s\r\n", en ? "set":"unset",ep->ep.name);
	switch(index) {
	case 0:
		if (en) {
			if (ep->is_in)
				usb_set(USB_CTL_EPx_DATA_READY, USB_EP0_CTRL);
			else
				usb_set(USB_CTL_EPx_BUF_READY, USB_EP0_CTRL);
		} else {
			if (ep->is_in)
				usb_clear(USB_CTL_EPx_DATA_READY, USB_EP0_CTRL);
			else
				usb_clear(USB_CTL_EPx_BUF_READY, USB_EP0_CTRL);
		}
		break;
	default:
		if (en) {
			if (ep_is_in(ep)) //in endpoint
				usb_set(USB_CTL_EPx_DATA_READY, USB_EPx_CTRL(index));
			else //out endpoint
				usb_set(USB_CTL_EPx_BUF_READY, USB_EPx_CTRL(index));
		} else {
			if (ep_is_in(ep)) //in endpoint
				usb_clear(USB_CTL_EPx_DATA_READY, USB_EPx_CTRL(index));
			else //out endpoint
				usb_clear(USB_CTL_EPx_BUF_READY, USB_EPx_CTRL(index));
		}
		break;
	}
	return;
}

/*
 * Configure endpoints
 */
static
void	ep0_config(struct sc8800_ep	*ep)
{
	unsigned max_packet_size = ep->ep.maxpacket;
	unsigned long flags;

	DEBUG("%s, %s", __func__, ep->ep.name);

	local_irq_save(flags);
 	__set_ep_max_payload(ep, max_packet_size);
	usb_set(USB_CTL_EP0_SETUP_INT_INTERVENE, USB_EP0_CTRL);
	/* Clear interrupt on endpoint0*/
	__clear_ep_irqsts(ep, USB_INT_EPx_MASK);

	//mask all irq
	__mask_ep_irqs(ep, USB_INT_EPx_MASK);
	//open setup end irq
	__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);

	local_irq_restore(flags);
}

static
void	ep1_config(struct sc8800_ep *ep)
{
	unsigned int value;
       unsigned long flags;
	   
	DEBUG("%s, %s\n", __func__, ep->ep.name);

	
	/* Enable endpoint1*/

	local_irq_save(flags);
	__enable_udc_ep(ep, TRUE);

	/* Configue endpoint1 payload size and type */
	value = usb_read(USB_EPx_CTRL(1));
	//ctrl->mBits.type		= 0;
	value &= ~TRANS_TYPE_MASK;
	value |= TRANS_TYPE_BULK;
	usb_write(value, USB_EPx_CTRL(1));

	__set_ep_max_payload(ep, ep->ep.maxpacket);
	/* Configue endpoint1's DMA wait time */
	value = 400; // default is 0x40
	usb_write(value, USB_EPx_DMA_WAITE_TIME(1));
	
	/* Configue the total size in a transfer, this value is equale to payload */
	__set_ep_transfer_size(ep, ep->ep.maxpacket);
	
	/* Set endpoint1's data not ready */
	__set_ep_ready(ep, 0);

	local_irq_restore(flags);
	//dump_regs();
}

static	
void	ep2_config(struct sc8800_ep *ep)
{
	unsigned int			value;
	unsigned long flags;
	
	DEBUG("%s , %s \n", __func__, ep->ep.name);

	local_irq_save(flags);
	/* Enable endpoint2 */
	__enable_udc_ep(ep, TRUE);

	/* Configue endpoint2 payload size and type */
	value	= usb_read(USB_EPx_CTRL(2));
	value &= ~TRANS_TYPE_MASK;
	value |= TRANS_TYPE_BULK;
	usb_write(value, USB_EPx_CTRL(2));
	__set_ep_max_payload(ep, ep->ep.maxpacket);
	/* Configue endpoint2's DMA wait time */
	value = 400;
	usb_write(value, USB_EPx_DMA_WAITE_TIME(2));
	
	/* Configue the total size in a transfer, this value is equale to payload */
	__set_ep_transfer_size(ep, ep->ep.maxpacket);
	//usb_write(USB_INT_EPx_MASK, USB_EPx_INT_CLR(2));
	__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
	/* Disable buf_not_ready interrupt. */
	__mask_ep_irqs(ep, USB_INT_EP_BUF_NREADY);

	/* Enable transfer_end interrupt */

	if (g_use_dma) {
		__mask_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
	} else {
		__set_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
	}

	/* Set endpoint2's buffer not ready */
	__set_ep_ready(ep, 0);

	local_irq_restore(flags);
	//dump_regs();
}

/*
 * 	udc_reinit - initialize software state
 */
static
void	udc_reinit(struct sc8800_udc *udc)
{
	int	i	= 0;
	
	DEBUG("%s, %p\n", __func__, udc);

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	udc->ep0state = EP0_IDLE;
	/*
	 * Link each endpoint into gadget, except endpoint zero. Then the gadget
	 * driver can enumerate each endpoint in the gadget with macro gadget_for_each_ep.
	 */
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++) {
		struct sc8800_ep	*ep	= &udc->ep[i];

		if (0 != i)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);

		ep->stopped	= 0;
		ep->desc	= NULL;
		
		/* Initialize the request queue.*/
		INIT_LIST_HEAD(&ep->queue);
	}
}

/*
 * Reset the fifo. 
 */
static
void	flush(struct sc8800_ep *ep)
{
	int index = ep_index(ep);
	unsigned rst_bit;
	
	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return;
	}
	DEBUG("%s, %s\n", __func__, ep->ep.name);
		
	switch (index) {
	case 0 :
		if (ep->is_in)
			usb_set(USB_RST_FIFO_EP0IN, USB_EPx_RESET);
		else
			usb_set(USB_RST_FIFO_EP0OUT,USB_EPx_RESET);
		break;
	default :
		rst_bit = USB_RST_FIFO_1 << index;
		usb_set(rst_bit, USB_EPx_RESET);
	}
}
/*
 * Configure the DMA for IN endpoint 1 and 3. 
 */
static
void	dma_in_config(struct sc8800_ep *ep, struct sc8800_request *req)
{
#if USB_DMA
    	DMA_CTL_CFG_T	dma_ctl_ep1;
	unsigned 			length;
	struct sc8800_udc * udc = ep->dev;
	struct device *dev = &udc->gadget.dev;
	
	length	= req->req.length - req->req.actual;
	DEBUG("%s: req length=%u\n", __func__, length);
	if (length == 0)
		return;

	if (req->req.dma == DMA_ADDR_INVALID) {
		 req->req.dma = dma_map_single(dev,
					req->req.buf,
					length,
					DMA_TO_DEVICE);
		 req->mapped = 1;
	}
	/* 1. Configure the DMA parameters */
	dma_ctl_ep1.dma_workmode	= DMA_MODE_NORMAL;
	dma_ctl_ep1.src_address		= req->req.dma;
	dma_ctl_ep1.des_address		= ep->pfifo_in;
	dma_ctl_ep1.total_size		= length;
	//dma_ctl_ep1.burst_size		= 0x40;
	dma_ctl_ep1.burst_size		= 0x20;
	dma_ctl_ep1.priority		= 0;
	dma_ctl_ep1.src_burst_step	= 0;
	dma_ctl_ep1.des_burst_step	= 0;
	dma_ctl_ep1.src_burst_mode	= DMA_BURSTMODE_INCR4;
	dma_ctl_ep1.des_burst_mode	= 0;
	dma_ctl_ep1.flag			= CHN_SRC_SIZE_WORD |CHN_DES_SIZE_WORD 
				| CHN_SRC_INCREAMENT| CHN_DES_NOCHANGE;
    DMA_ChannelCfg(DMA_USB_EP1,DMA_TYPE_HARD,DMA_INT_SEL_ALL_TRANSFER,
					&dma_ctl_ep1);

	/* 2. Reset endpoint1 FIFO */
	flush(ep);
	
	/* 3. Enable DMA channel */
    DMA_ChannelEnable(DMA_USB_EP1,TRUE);

	/* 4. Set maxpayload */
	if (likely(length <= MAX_FIFO_SIZE)) {
		__set_ep_max_payload(ep, length);
	} else {
		__set_ep_max_payload(ep, MAX_FIFO_SIZE);
	}

	/* 5. Set transfer_size */
	__set_ep_transfer_size(ep, length);

	/* 6. Ready to send datas to host */
	__set_ep_ready(ep, 1);

	__set_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
#endif
}

/*
 * Configure the DMA for OUT endpoint 2 and 4. 
 */
static
void	dma_out_config(struct sc8800_ep *ep, struct sc8800_request *req)
{
#if USB_DMA
	DMA_CTL_CFG_T	dma_ctl_ep2;
	unsigned int		 length;
	struct sc8800_udc * udc = ep->dev;
	struct device *dev = &udc->gadget.dev;

	length	= req->req.length - req->req.actual;

	DEBUG("%s: req length=%u\n", __func__, length);
	if (0 == length)
		return;

	if (req->req.dma == DMA_ADDR_INVALID) {
		 req->req.dma = dma_map_single(dev,
					req->req.buf,
					length,
					DMA_FROM_DEVICE);
		 req->mapped = 1;
	}

	/* 1. Configure the DMA parameters.*/
	dma_ctl_ep2.dma_workmode	= DMA_MODE_NORMAL;
	dma_ctl_ep2.src_address		= ep->pfifo_out;
	dma_ctl_ep2.des_address		= req->req.dma;
	dma_ctl_ep2.total_size		= length;
	dma_ctl_ep2.burst_size		= 0x40;
	dma_ctl_ep2.priority		= 0;
	dma_ctl_ep2.src_burst_step	= 0;
	dma_ctl_ep2.des_burst_step	= 0;
	dma_ctl_ep2.src_burst_mode	= 0;
	dma_ctl_ep2.des_burst_mode	= DMA_BURSTMODE_INCR4;
	dma_ctl_ep2.flag = CHN_SRC_SIZE_WORD|CHN_DES_SIZE_WORD 
					|CHN_DES_INCREAMENT |CHN_SRC_NOCHANGE;
	DMA_ChannelCfg(DMA_USB_EP2, DMA_TYPE_HARD, 
				DMA_INT_SEL_ALL_TRANSFER,&dma_ctl_ep2);

	/* 2. Enable DMA channel */
    	DMA_ChannelEnable(DMA_USB_EP2,TRUE);

	/* 3. Set maxpayload of endpoint2 */
	if (likely(length <= MAX_FIFO_SIZE)) {
		__set_ep_max_payload(ep, length);
	} else {
		__set_ep_max_payload(ep, MAX_FIFO_SIZE);
	}
	
	/* 4. Set the size of this transfer */
	__set_ep_transfer_size(ep, length);

	flush(ep);
		
	/* 5. Ready to read from the host */
	__set_ep_ready(ep, 1);
#endif
}

/*
 * Complete the request with the status
 */
static 
void done(struct sc8800_ep *ep, struct sc8800_request *req, int status)
{
	unsigned		stopped = ep->stopped;
	struct sc8800_udc * udc = ep->dev;
	struct device *dev = &udc->gadget.dev;
	
	DEBUG("%s - req.status = %d; status = %d\n", __func__, req->req.status, status);

	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (req->mapped) {
		dma_unmap_single(dev, req->req.dma, req->req.length,
			ep_is_in(ep) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}
	//sword, for debug only
	if(g_use_dma) {
		memcpy(log_buf, req->req.buf,  req->req.actual);
	}
	if (status && status != -ESHUTDOWN)
		DEBUG("complete %s req %p stat %d len %u/%u\n",
				ep->ep.name, &req->req, status,
				req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	DEBUG("Req is completed! status=%d; length=%u; actual=%u\n",
		req->req.status, req->req.length, req->req.actual);
	req->req.complete(&ep->ep, &req->req);
	ep->stopped	= stopped;
}

/*
 * Complete all the request in the endpoint.
 */
static
void nuke(struct sc8800_ep *ep, int status)
{
	struct sc8800_request	*req;

	DEBUG("%s, %s\n", __func__, ep->ep.name);

	/* Flush FIFO */
	flush(ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct sc8800_request, queue);
		done(ep, req, status);
	}
}

static 
void clear_ep_state (struct sc8800_udc *udc)
{
	int i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++)
		nuke(&udc->ep[i], -ECONNABORTED);
}

static 
void	udc_disable(struct sc8800_udc *udc)
{
	int	i	= 0;
	/* Disable USB LDO */
	usb_ldo_switch(FALSE);
	/* Disable USB clock */
	usb_mainclock_enable(FALSE);
	/* Disable USB device */
	enable_usb_device(FALSE);
	/* Disable USB Interrupt */
	enable_usb_irq(FALSE);
	/* Disable the interrupt response on endpoints. */
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++) {
		struct sc8800_ep	*ep	= &udc->ep[i];
		
		__mask_ep_irqs(ep, USB_INT_EPx_MASK);
		/* Disable ep1, ep2, ep3, ep4...*/
		if (i != 0)
			__enable_udc_ep(ep, FALSE);
	}

	udc->gadget.speed = USB_SPEED_UNKNOWN;
}

static 
void	udc_enable(struct sc8800_udc *udc)
{
	int	i	= 0;

	DEBUG("%s, %p\n", __func__, udc);

	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/* The USB device should be disabled before endpoint configued */
	enable_usb_irq(FALSE);

	/* Confige endpoint0, endpoint1 and endpoint2 */
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++)	{
		struct sc8800_ep	*ep	= &udc->ep[i];

		switch (ep->bEndpointAddress) {
		case 0:
			ep0_config(ep);
			break;
		default :
			if ((USB_DIR_IN | 1) == ep->bEndpointAddress)
				ep1_config(ep);
			else if ((USB_DIR_OUT | 2) == ep->bEndpointAddress)
				ep2_config(ep);
			else
				printk("Unknown endpoint address: 0x%x", ep->bEndpointAddress);
			break;
		}

		flush(ep);
	}

	/* SET selfpowered */
	usb_set(0x2, USB_DEV_CTRL);
	/* Enable inerrupt on reset message */
	usb_set(USB_INT_DEV_MASK, USB_INT_CLR);
	
	//usb_set(USB_INT_DEV_MASK, USB_INT_CTRL);

	usb_set(0x3E, USB_INT_CTRL);
	usb_ldo_switch(1);
	enable_usb_irq(TRUE);
}

/**
 *	Description:	This function can fill the datas in the irq buffer into 
 *					the FIFO.
 *					As the FIFO can be accessed with words aligned, the format
 *					must be translated if the datas' length is not 4 bytes aligned.
 *	Parameter:
 *		ep		endpoint object
 *		req		IRQ message
 *		length	the size of datas written into the FIFO
 *	Return:
 *		the size of datas written into the FIFO
 */
static 
int	fill_fifo(struct sc8800_ep *ep, struct sc8800_request *req, int length)
{
	unsigned char	 *buf;
	int	tmpLength, i;
	unsigned int fifo = ep->fifo_in;
	unsigned int tmp_buf[MAX_FIFO_SIZE / 4] = {0};
	
	buf	= req->req.buf + req->req.actual;
	req->req.actual += length;

	if (((unsigned int)buf & 0x3) != 0) {
		pr_warning("req buf address wrong, should be align for word\r\n");
		BUG_ON(1);
	}
	tmpLength = (length + 3) /4;

	//DEBUG("length = %d, tmpLength = %d\n", length, tmpLength);

	for (i = 0; i < tmpLength; i++)	{
		tmp_buf[i]	= *(unsigned int*)buf;
#ifndef CONFIG_CPU_BIG_ENDIAN	
		__swab32s(&tmp_buf[i]);
#endif
		buf	+= 4;
		__raw_writel( tmp_buf[i], fifo);
	}
	//dump_log(tmp_buf, length, SENT_DATA);
	return length;
}


/**
 *	Description:	This function can exact the datas into the irq buffer from 
 *					the FIFO.
 *	Parameter:
 *		ep		endpoint object
 *		req		IRQ message
 *		length	the size of datas written into the FIFO
 *	Return:
 *		the size of datas read from the FIFO
 */
static
unsigned int	read_fifo(struct sc8800_ep *ep, struct sc8800_request *req, unsigned int length)
{
	unsigned char	 *buf;
	unsigned int tmp_buf[MAX_FIFO_SIZE / 4]	= {0};
	int	tmpLength, i;
	unsigned fifo = ep->fifo_out;

	unsigned char	 *recv_buf = req->req.buf + req->req.actual;
	
	buf	= req->req.buf + req->req.actual;
	req->req.actual += length;

	tmpLength	= (length + 3) /4;
	
	for (i = 0; i < tmpLength; i++)	{
		tmp_buf[i]	= __raw_readl(fifo);
#ifndef CONFIG_CPU_BIG_ENDIAN	
		__swab32s(&tmp_buf[i]);
#endif
	}
	memcpy(buf, tmp_buf, length);	
//	dump_log(recv_buf, length, RECEIVED_DATA);	
	return length;
}

static
int	write_ep0_fifo(struct sc8800_ep *ep, struct sc8800_request *req)
{
	unsigned int length;
	
	//DEBUG("%s\n", __func__);

	length = req->req.length - req->req.actual;

	/* 1st, Set tranfer size */
	__set_ep_transfer_size(ep, length);
	if (likely(length <= MAX_FIFO_SIZE)) {
		DEBUG("max packet length %d\r\n",length);

		__set_ep_max_payload(ep, length);
		/* 3rd, Reset ep0 in FIFO */
		usb_set(USB_RST_FIFO_EP0IN, USB_EPx_RESET);

		/* 4th, Fill FIFO with datas to be transfered */
		fill_fifo(ep, req, length);

		__set_ep_ready(ep, 1);

		//dump_regs();
	} else {
		int segment;
		int timeout = 0;
		unsigned int left;
		int segs;

		segs = length / MAX_FIFO_SIZE;
		for (segment = 0; segment < segs; segment++)  {
			__set_ep_max_payload(ep, MAX_FIFO_SIZE);

			/* 3rd, Reset ep0 in FIFO */
			usb_set(USB_RST_FIFO_EP0IN, USB_EPx_RESET);
			
			fill_fifo(ep, req, MAX_FIFO_SIZE);
			__set_ep_ready(ep, 1);			
			
			__clear_ep_irqsts(ep, USB_INT_EPx_MASK);

			while (!(usb_read(USB_EP0_INT_RAW) & USB_INT_EP_TRANSACT_END))	{
				mdelay(1);
				timeout++;
				if (timeout == 10) {
					DEBUG("udc send data timeout\r\n");
					dump_regs();
					break;
				}
			}
		}
		
		if (length % MAX_FIFO_SIZE) {
			left = length % MAX_FIFO_SIZE;
			DEBUG("fill fifo left %d, continue...\r\n", left);
			__set_ep_max_payload(ep, left);
			usb_set(USB_RST_FIFO_EP0IN, USB_EPx_RESET);
			fill_fifo(ep, req, left);
			__set_ep_ready(ep, 1);
		}
		//dump_regs();
	}
		
	__set_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
	/*
	DEBUG("%s wrote  %d bytes,  %d left\n", ep->ep.name, length,
		  	(req->req.length - req->req.actual));
	*/
	return length;
}

static
unsigned int send_data(struct sc8800_ep *ep, struct sc8800_request *req)
{
	unsigned int length;
	unsigned int sent_len = 0;
	unsigned char	 *sent_buf	= req->req.buf + req->req.actual;

	length = req->req.length - req->req.actual;

	if (req->req.zero) {
		DEBUG("requset send a zero length packt to host\r\n");
	}
	/* 1st, Set tranfer size */
	__set_ep_transfer_size(ep, length);
	DEBUG("sending length %d\r\n",length);
	if (likely(length <= MAX_FIFO_SIZE)) {
		__set_ep_max_payload(ep, length);	
		/* 3rd, Reset ep1 in FIFO */
		flush(ep);
		fill_fifo(ep, req, length);
		sent_len = length;
	} else {	
		__set_ep_max_payload(ep, MAX_FIFO_SIZE);
		/* 3rd, Reset ep1 in FIFO */
		flush(ep);
		fill_fifo(ep, req, MAX_FIFO_SIZE);
		sent_len = MAX_FIFO_SIZE;
	}
	
	__set_ep_ready(ep, 1);
	__set_ep_irqs(ep, USB_INT_EP_TRANSACT_END);
	
	
	//	dump_log(sent_buf, sent_len, SENT_DATA);
	//dump_regs();
	return length;
}

static
void	prepare_recv_data(struct sc8800_ep *ep, struct sc8800_request *req)
{
	unsigned int length;

	length = req->req.length - req->req.actual;


	DEBUG("%s, length is :%d\n", __func__, length);
	/* 1st, Set max payload */
	if (likely(length <= MAX_FIFO_SIZE)) {
		__set_ep_max_payload(ep, length);
	} else {
		__set_ep_max_payload(ep, MAX_FIFO_SIZE);
	}
	
	/* 2nd, Set tranfer size */
	 __set_ep_transfer_size(ep, length);
	/* 3rd, Reset ep2 FIFO */
	flush(ep);

	/* 5. Ready to read from the host */
	__set_ep_ready(ep, 1);
	//dump_regs();
}


static
void stop_activity(struct sc8800_udc *dev, 
					struct usb_gadget_driver *driver)
{
	int i;

	DEBUG("%s \n", __func__);
	
	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++) {
		struct sc8800_ep *ep = &dev->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		driver->disconnect(&dev->gadget);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}


static
void	sc8800_dma_ep1(unsigned int param)
{	
	DEBUG("%s in\n", __func__);
}

static
void	sc8800_dma_ep2(unsigned int param)
{
	struct sc8800_udc		*udc	= &the_controller;
	struct sc8800_ep		*ep		= &udc->ep[2];
	struct sc8800_request	*req;

	DEBUG("%s out,\r\n", __func__);

	//usb_clear(USB_CTL_EPx_BUF_READY, USB_EPx_CTRL(2));
	__set_ep_ready(ep, 0);
	/* 1. Disable DMA channel */
	//DMA_ChannelEnable(DMA_USB_EP2,FALSE);

	/* 3. Complete the IRQ waiting in the queue.*/
	if (list_empty(&ep->queue))	{
		DEBUG("No request in queue\n");
	} else {
		req = list_entry(ep->queue.next, struct sc8800_request, queue);
		if (req) {
			unsigned length, receive_num;			

			receive_num	= usb_read(USB_EPx_XFER_DATA(2));
			length = min(receive_num, req->req.length);
			
			DEBUG("%s: dma receive end, received:%x \n", __func__, receive_num);
			req->req.actual += length;
			
			done(ep, req, 0);
			DEBUG("%s ,dump received log:\r\n", __func__);
			//dump_log((char *)log_buf, length, RECEIVED_DATA);
			if (!list_empty(&ep->queue)) {
				DEBUG("%s has req to be done\n", ep->ep.name);
				__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
				__set_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
			}
		}
	}
	
	
}


static
void	sc8800_udc_ep0_setup(struct sc8800_udc *udc)
{
	unsigned		setup_low;
	unsigned		setup_high;
	struct usb_ctrlrequest	ctrlreq;
	struct sc8800_ep	*ep	= &udc->ep[0];
	int ret;

	setup_low	= usb_read(USB_SETUP_LOW);
	setup_high	= usb_read(USB_SETUP_HIGH);
	ctrlreq.bRequestType	= setup_low & 0xff;
	ctrlreq.bRequest		= (setup_low & 0x0000ff00) >> 8;
	ctrlreq.wValue			= cpu_to_le16((setup_low & 0xffff0000) >> 16);
	ctrlreq.wIndex			= cpu_to_le16(setup_high & 0x0000ffff);
	ctrlreq.wLength		= cpu_to_le16((setup_high & 0xffff0000) >> 16);

	DEBUG( "Setup end with control req :\r\n"
		"bRequestType=%x\t bRequest=%x\t value=%x\t length=%x\t index=%x\r\n",
		ctrlreq.bRequestType,
		ctrlreq.bRequest,
		le16_to_cpu(ctrlreq.wValue),
		le16_to_cpu(ctrlreq.wLength),
		le16_to_cpu(ctrlreq.wIndex)
		);

	switch (ctrlreq.bRequest) {
		case USB_REQ_SET_CONFIGURATION :
			DEBUG("USB_REQ_SET_CONFIGURATION\r\n");
			break;
		case USB_REQ_SET_INTERFACE :
			DEBUG("USB_REQ_SET_INTERFACE\r\n");
			return;
		case USB_REQ_SET_ADDRESS : //the udc can handle this req
			DEBUG("USB_REQ_SET_ADDRESS\r\n");
			return;
		default :
			DEBUG("Other bRequest: 0x%x\r\n", ctrlreq.bRequest);
			break;
	}

	if (ctrlreq.bRequestType & USB_DIR_IN) {
		udc->ep0state = EP0_IN_DATA_PHASE;
		ep->is_in = 1;
	} else {
		udc->ep0state = EP0_OUT_DATA_PHASE;
		ep->is_in = 0;
	}
	__mask_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
	DEBUG("%s: driver->setup()\r\n", __func__);
	ret = udc->driver->setup(&udc->gadget, &ctrlreq);

	if (ret < 0) {
		__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);
		DEBUG("%s setup failed :%d \r\n", __func__, ret);
		udc->ep0state	= EP0_IDLE;
	}
}


static
int sc8800_udc_ep0_in(struct sc8800_udc *dev, struct sc8800_request *req)
{
	struct sc8800_ep	*ep	= &dev->ep[0];
	int 	ret;

	if (!req) {
		DEBUG("%s: NULL REQ\n", __func__);
		dev->ep0state	= EP0_IDLE;
		return 0;
	}

	if (0 == req->req.length) {
		DEBUG("Request length is %d\n", req->req.length);

		done(ep, req, 0);
		dev->ep0state	= EP0_IDLE;
		ret	= 1;

		goto finish;
	}

	ret = write_ep0_fifo(ep, req);
	
finish:
	return ret;	
}


void sc8800_udc_connect(int state)
{
	struct sc8800_udc		*dev	= &the_controller;
		
	DEBUG("%s state=%d\n", __func__, state);

	if (state == 0) {
		dev->connected	= FALSE;
		dev->configured	= FALSE;
		stop_activity(dev, dev->driver);
	} else if (state == 1) {
		dev->connected	= TRUE;
	}
}

static
void	sc8800_udc_reset(struct sc8800_udc *udc)
{
	int i;
	DEBUG("%s \n", __func__);

	/* prevent new request submissions, kill any outstanding requests  */
	/*
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++) {
		struct sc8800_ep *ep = &udc->ep[i];
		//sc8800_ep_disable(ep);
	}
	*/
	clear_ep_state(udc);
	
	udc->gadget.speed	= USB_SPEED_FULL;
	udc->ep0state		= EP0_IDLE;

}

static
void	sc8800_udc_handle_ep0(struct sc8800_udc *udc)
{
	struct sc8800_ep		*ep		= &udc->ep[0];
	struct sc8800_request	*req;
	int ep0_sts;

	
	ep0_sts = usb_read(USB_EP0_INT_STS);
	DEBUG("%s: ep0_sts:%x, raw_ep0_sts:%x\r\n", __func__, ep0_sts,
		usb_read(USB_EP0_INT_RAW));
	/* Get the first reqest from the request queue of the endpoint0 */
	if (list_empty(&ep->queue))	{
		DEBUG("No request in queue\n");
		req = NULL;
	} else {
		req = list_entry(ep->queue.next, struct sc8800_request, queue);
	}

	__mask_ep_irqs(ep,USB_INT_EP_TRANSFER_END);
	
	switch (udc->ep0state) {
	case EP0_IDLE :
		if (ep0_sts & USB_INT_EP0_SETUP_END) { 
			__clear_ep_irqsts(ep, USB_INT_EP0_SETUP_END);

			//DEBUG("Setup end IRQ\n");		
			sc8800_udc_ep0_setup(udc);
		} else {
			pr_info("Unsupported interrupt in EP0_IDLE.\n");
		}
		break;
	case EP0_IN_DATA_PHASE :
		if (ep0_sts & USB_INT_EP_TRANSFER_END) {
			__clear_ep_irqsts(ep, USB_INT_EP_TRANSFER_END);
			DEBUG("In Transfer end IRQ\n");
			/* All datas has been written into FIFO, so begin to send datas to the host.*/
			/* Complete the request */
			__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);
			done(ep, req, 0);
			udc->ep0state	= EP0_IDLE;
			req = NULL;		
		} else {
			pr_info("Unsupported interrupt in EP0_IN_DATA_PHASE\n");
		}
		break;
	case EP0_OUT_DATA_PHASE :
		if (ep0_sts & USB_INT_EP_BUF_NREADY) {
			/* clear the irq status */
			__clear_ep_irqsts(ep, USB_INT_EP_BUF_NREADY);
			/* Disable buffer not ready interrupt */
			__mask_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
			DEBUG("%s: buf_not_ready\n", __func__);
			if (!req) {
				DEBUG("no request");
				return;
			}
		
			prepare_recv_data(ep, req);
			
			/* Enable transfer end and transact end interrupt */
			__clear_ep_irqsts(ep, USB_INT_EP_TRANSACT_END);
			__set_ep_irqs(ep, USB_INT_EP_TRANSACT_END);
					
		} else if (ep0_sts & USB_INT_EP_TRANSACT_END) {
			DEBUG("transact_end\r\n");
			__clear_ep_irqsts(ep, USB_INT_EP_TRANSACT_END);
			if (req) {
				unsigned int	receive_num, length;
				
				__set_ep_ready(ep, 0);
				
				receive_num	= usb_read(USB_RECEIVED_DATA_NUM_EP0);
				DEBUG("received :%x\r\n", receive_num);
				length = min(receive_num, req->req.length);
				if (length <= MAX_FIFO_SIZE) {
					read_fifo(ep, req, length);
				} 
				
				DEBUG("transact_end, left :%d\r\n", req->req.length - req->req.actual);

				if ((req->req.actual == req->req.length) ||
					(length < MAX_FIFO_SIZE)){
					DEBUG("recevied all data \r\n");
					__mask_ep_irqs(ep, USB_INT_EP_TRANSACT_END);
					done(ep, req, 0);
					
					if (!list_empty(&ep->queue)) {
						DEBUG("%s has req to be done\n", ep->ep.name);
						__set_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
						return;
					}
					__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);
					udc->ep0state	= EP0_IDLE;
					return;
				}
				__set_ep_ready(ep, 1);
			} else {
				DEBUG("%s no request to be done\n", ep->ep.name);
			}
		}else if (ep0_sts & USB_INT_EP_TRANSFER_END) {	
				__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);
				done(ep, req, 0);
				udc->ep0state	= EP0_IDLE;
				DEBUG("%s: received all data %d left\r\n", ep->ep.name,
	  					req->req.length - req->req.actual);
		}else {
			pr_info("Unsupported interrupt in EP0_OUT_DATA_PHASE\n");
		}
		break;
	default :
		pr_info("Unknowned ep0 statement.\n");
		break;
	}
}

static
void	sc8800_udc_handle_ep1(struct sc8800_udc *udc)
{
	struct sc8800_ep		*ep		= &udc->ep[1];
	struct sc8800_request	*req;
	int ep1_sts;

	ep1_sts = usb_read(USB_EPx_INT_STS(1));

	
//	DEBUG("%s: ep1_sts = 0x%x, frame_num:%x \n", __func__, ep1_sts,
//		usb_read(USB_FRAME_NUM));
	
	/* Get the first reqest from the request queue of the endpoint1 */
	if (unlikely(list_empty(&ep->queue)))	{
		DEBUG("No request in queue\n");
		req = NULL;
		__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
		return;
	} else {
		req = list_entry(ep->queue.next, struct sc8800_request, queue);
	}

	if (ep1_sts & USB_INT_EP_TRANSACT_END) {
		//DEBUG("%s transact end\n", ep->ep.name);
		__clear_ep_irqsts(ep, USB_INT_EP_TRANSACT_END);
		__set_ep_ready(ep, 0);
		
		if (req->req.length == req->req.actual) {
			DEBUG("%s all datas have been sent\n", ep->ep.name);
			done(ep, req, 0);
			__mask_ep_irqs(ep, USB_INT_EP_TRANSFER_END);

			if (!list_empty(&ep->queue)) {
				DEBUG("%s has req to be done\n", ep->ep.name);
				req	= list_entry(ep->queue.next, struct sc8800_request, queue);
				send_data(ep, req);
			}
		} else if (req->req.length > req->req.actual) {
//				DEBUG("%s go on sending data\n", ep->ep.name);
			send_data(ep, req);
		} else {
			DEBUG("%s data overflow\n", ep->ep.name);
		}
			
	}else if (ep1_sts & USB_INT_EP_TRANSFER_END) {
		__clear_ep_irqsts(ep, USB_INT_EP_TRANSFER_END);
		DEBUG("%s: transfer end\n", __func__);
		/* Clear data ready bit */
		__set_ep_ready(ep, 0);
		__mask_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
		req->req.actual += req->req.length;
		DEBUG("%s all datas have been sent\n", ep->ep.name);
		done(ep, req, 0);
	
		if (!list_empty(&ep->queue)) {
			DEBUG("%s has req to be done\n", ep->ep.name);
			req	= list_entry(ep->queue.next, struct sc8800_request, queue);
			__mask_ep_irqs(ep, USB_INT_EPx_MASK);
			if (g_use_dma)
				dma_in_config(ep, req);
			else
				send_data(ep, req);
			}
	}	 else {
		pr_info("Unsupported interrupt with ep1_sts=0x%x\n", ep1_sts);
	}
}

static
void	sc8800_udc_handle_ep2(struct sc8800_udc *udc)
{
	struct sc8800_ep		*ep		= &udc->ep[2];
	struct sc8800_request	*req;
	int  ep2_sts;
	
	ep2_sts	= usb_read(USB_EPx_INT_STS(2));
//	DEBUG("%s: ep2_sts = 0x%x, raw_ep2_sts=%x frame_num:0x%x \r\n", __func__, 
//		ep2_sts, usb_read(USB_EPx_INT_RAW(2)), usb_read(USB_FRAME_NUM));
	
	/* Get the first reqest from the request queue of the endpoint2 */
	if (unlikely(list_empty(&ep->queue))) {
		DEBUG("No request in queue\n");
		req = NULL;
		__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
		return;
	} else {
		req = list_entry(ep->queue.next, struct sc8800_request, queue);
	}

	if (ep2_sts & USB_INT_EP_BUF_NREADY) {
		/* clear the irq status */
		__clear_ep_irqsts(ep, USB_INT_EP_BUF_NREADY);
		/* Disable buffer not ready interrupt */
		//__mask_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
		__mask_ep_irqs(ep, USB_INT_EPx_MASK);
		DEBUG("%s: buf_not_ready\n", __func__);

		if (g_use_dma) {
			dma_out_config(ep, req);
			/* Enable transfer end and transact end interrupt */
			//__set_ep_irqs(ep, USB_INT_EP_TRANSFER_END);
		} else {
			prepare_recv_data(ep, req);
			__set_ep_irqs(ep, USB_INT_EP_TRANSACT_END);
		}
		//dump_regs();	
	} else if (ep2_sts & USB_INT_EP_TRANSACT_END) {
		unsigned int	receive_num, length;

		__clear_ep_irqsts(ep, USB_INT_EP_TRANSACT_END);
		__set_ep_ready(ep, 0);
		receive_num	= usb_read(USB_EPx_XFER_DATA(2));
		DEBUG("received :%x\r\n", receive_num);
		length = min(receive_num, req->req.length);
		if (length <= MAX_FIFO_SIZE) {
			read_fifo(ep, req, length);
		} 
			
		DEBUG("transact_end, left :%d\r\n", req->req.length - req->req.actual);

		if ((req->req.actual == req->req.length) ||
			(length < MAX_FIFO_SIZE)) {
			DEBUG("recevied all data \r\n");
			done(ep, req, 0);

			if (!list_empty(&ep->queue)) {
				DEBUG("%s has req to be done\n", ep->ep.name);
				__set_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
			}
	
			return;
		}
		__set_ep_ready(ep, 1);
	} else if (ep2_sts & USB_INT_EP_TRANSFER_END) {
		__set_ep_ready(ep, 0);
		//clear transfer end int
		__clear_ep_irqsts(ep, USB_INT_EP_TRANSFER_END);
		//disable transfer end int
		__mask_ep_irqs(ep, USB_INT_EP_TRANSFER_END);


		#if 0
		if (req) {
			unsigned length, receive_num;			

			receive_num	= usb_read(USB_EPx_XFER_DATA(2));
			length = min(receive_num, req->req.length);
			
			DEBUG("%s: transfer_end, received:%x \n", __func__, receive_num);
			req->req.actual += length;
			
			DEBUG("%s totale received %d over\n", ep->ep.name, req->req.actual);
			done(ep, req, 0);			
			if (!list_empty(&ep->queue)) {
				DEBUG("%s has req to be done\n", ep->ep.name);
				usb_set(USB_INT_EP_BUF_NREADY, USB_EPx_INT_CTRL(2));
			}
			DEBUG("%s ,dump received log:\r\n", __func__);
			dump_log(log_buf, length, RECEIVED_DATA);
		}
		#endif
	}else {
		DEBUG("Unsupported interrupt in ep2_sts=0x%x\n", ep2_sts);
	}
}

static
irqreturn_t sc8800_udc_irq(int irq, void *_udc)
{
	struct sc8800_udc	*udc		= (struct sc8800_udc*)_udc;
	int raw;
	int udc_int_status;
	
	/* read  the interrupt source */
	raw	= usb_read(INT_IRQ_RAW_STS);
	udc_int_status = usb_read(USB_INT_STS);

	if (raw & INTCTL_USBD_IRQ) {
		if (udc_int_status & USB_INT_RST) {
			sc8800_udc_reset(udc);
			//dump_regs();
		}

		if (udc_int_status & USB_INT_SPEND) {
			DEBUG("USB suspend\n");
/*
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
					&& udc->driver && udc->driver->suspend)
				udc->driver->suspend(&udc->gadget);
*/
		}

		if (udc_int_status & USB_INT_RESUM) {
			DEBUG("USB resume\n");
/*
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
					&& udc->driver && udc->driver->resume)
				udc->driver->resume(&udc->gadget);
*/
		}

	
		if (udc_int_status & USB_INT_UNPID) {
			DEBUG("USB unsuppored PID\r\n");
		}

		if (udc_int_status & USB_INT_EPID) {
			DEBUG("USB error PID\r\n");
		}

		if (udc_int_status & USB_INT_CHANGE) {
			DEBUG("USB state change\r\n");
		}
		if (udc_int_status & USB_INT_EP0) {
			DEBUG("USB Endpont0 irq\r\n");
			sc8800_udc_handle_ep0(udc);
		}

		if (udc_int_status & USB_INT_EP1) {
			DEBUG("USB Endpont1 irq\r\n");
			sc8800_udc_handle_ep1(udc);
		}

		if (udc_int_status & USB_INT_EP2) {
			DEBUG("USB Endpoint2 irq\r\n");
			sc8800_udc_handle_ep2(udc);
		}
		
	} else {
		DEBUG("NOT USB interrupt!! INT_IRQ_RAW_STS = 0x%x, USB_INT_STS: %x, INT_IRQ_STS :%x\n", 
			raw, udc_int_status, __raw_readl(INT_IRQ_STS));
	}
	usb_write(USB_INT_DEV_MASK, USB_INT_CLR);
	return IRQ_HANDLED;
}

static 
int sc8800_ep_enable (struct usb_ep *_ep,
						const struct usb_endpoint_descriptor *desc)
{
	struct sc8800_ep        *ep;
	struct sc8800_udc       *dev;
	unsigned long flags;
	
	DEBUG("%s \n", __func__);

	ep = container_of (_ep, struct sc8800_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| ep->bEndpointAddress != desc->bEndpointAddress) {
		printk("%s, bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
			&& ep->bmAttributes != USB_ENDPOINT_XFER_BULK
			&& desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		printk("%s, %s type mismatch\n", __func__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
				&& le16_to_cpu (desc->wMaxPacketSize)
						!= MAX_FIFO_SIZE)
			|| !desc->wMaxPacketSize) {
		printk("%s, bad %s maxpacket\n", __func__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		printk("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}
	local_irq_save(flags);
	ep->desc = desc;
	ep->stopped = 0;
	ep->ep.maxpacket = le16_to_cpu (desc->wMaxPacketSize);
	
	__enable_udc_ep(ep, TRUE);
	/* flush fifo (mostly for OUT buffers) */
	flush (ep);
	local_irq_restore(flags);
	
	DEBUG("enabled %s\n", _ep->name);
	return 0;
}

static 
int sc8800_ep_disable (struct usb_ep *_ep)
{
	struct sc8800_ep        *ep;
	unsigned long 			flags;

	DEBUG("%s \n", __func__);

	ep = container_of (_ep, struct sc8800_ep, ep);
	if (!_ep || !ep->desc) {
		DEBUG("%s, %s not enabled\n", __func__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	__enable_udc_ep(ep, FALSE);
	ep->desc = 0;
	ep->stopped = 1;
	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

static 
struct usb_request *
sc8800_alloc_request (struct usb_ep *_ep, unsigned gfp_flags)
{
	struct sc8800_request *req;

	DEBUG("%s \n", __func__);

	req = kzalloc (sizeof *req, gfp_flags);
	if (!req)
		return NULL;
	req->req.dma = DMA_ADDR_INVALID;
	req->mapped = 0;
	INIT_LIST_HEAD (&req->queue);
	return &req->req;
}

static 
void	sc8800_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
	struct sc8800_request	*req;

	DEBUG("%s \n", __func__);

	req = container_of (_req, struct sc8800_request, req);
	WARN_ON (!list_empty (&req->queue));
	kfree(req);
}

static int	sc8800_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct sc8800_ep	*ep;
	int	 index;
	unsigned long		flags;

	ep = container_of(_ep, struct sc8800_ep, ep);
	if (!_ep || (!ep->desc && ep->ep.name != ep0name)) {
		printk("%s, bad ep: %s\n", __func__, ep->ep.name);
		return -EINVAL;
	}

	pr_info("%s, ep %d, val %d\n", __func__, ep_index(ep), value);
	index = ep_index(ep);

	if ((index < 0) ||(index > (SC8800_UDC_NUM_EPS -1))) {
		WARN(1, "wrong endpoint :%s\r\n", ep->ep.name);
		return -EINVAL;
	}
	spin_lock_irqsave(&ep->dev->lock, flags);

	switch (index) {
	case 0 :
		if (value)
			usb_set(USB_CTL_EPx_STALL, USB_EP0_CTRL);
		else
			usb_clear(USB_CTL_EPx_STALL, USB_EP0_CTRL);
		break;
	default:
		if (value)
			usb_set(USB_CTL_EPx_STALL, USB_EPx_CTRL(index));
		else
			usb_clear(USB_CTL_EPx_STALL, USB_EPx_CTRL(index));
		break;
	}
	
	ep->stopped = value ? 1 : 0;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	pr_info("%s %s halted\n", _ep->name, value == 0 ? "NOT" : "IS");

	return 0;
}

static 
int sc8800_ep_queue(
				struct usb_ep		*_ep, 
				struct usb_request	*_req,
				unsigned 			gfp_flags)
{
	struct sc8800_request	*req;
	struct sc8800_ep		*ep;
	struct sc8800_udc		*dev;
	unsigned long			flags;

	
	
	/* Verify the request parameter. */
	req = container_of(_req, struct sc8800_request, req);
	if (!_req || !_req->complete || !_req->buf	|| !list_empty(&req->queue)) {
		pr_warning("%s, bad params\n", __func__);
		DEBUG("req queue is not empty?%d\r\n", list_empty(&req->queue));
		return -EINVAL;
	}
	
	DEBUG("ep:%s req length=%d \r\n",  _ep->name, _req->length);
	dump_req(_req);
	/* The endpoint must have been enabled */
	ep = container_of(_ep, struct sc8800_ep, ep);
	if (!_ep || (!ep->desc && ep->ep.name != ep0name)) {
		pr_warning("%s, bad ep\n", __func__);
		return -EINVAL;
	}

	/* The device must have been configured */
	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		pr_warning("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (ep->bmAttributes == USB_ENDPOINT_XFER_ISOC
			&& req->req.length > le16_to_cpu(ep->desc->wMaxPacketSize))
		return -EMSGSIZE;
/*
	DEBUG("%s queue req %p, len %d buf %p\n",
	     		_ep->name, _req, _req->length, _req->buf);
*/
	/* Begin to deal with request with interrupt masked.*/
	local_irq_save(flags);

	_req->status	= -EINPROGRESS;
	_req->actual	= 0;

	/* if there's no requests waiting in the queue and the endpoint is enabled,
	 * the request will be dealt with immediately.
	 */
	if (list_empty(&ep->queue) && !ep->stopped) {
		DEBUG("%s list is empty\n", ep->ep.name);
		
		if (0 == ep_index(ep)) {
			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE :
				DEBUG("ep0 statement : EP0_IN_DATA_PHASE\n");
				sc8800_udc_ep0_in(dev, req);
				break;
			case EP0_OUT_DATA_PHASE :
				DEBUG("ep0 statement : EP0_OUT_DATA_PHASE\n");

				/* Prepare to receive datas from the host.*/
				if (!req || (0 == req->req.length)){
					DEBUG("Request length is %d\r\n", req->req.length);
			//		done(ep, req, 0);
					dev->ep0state	= EP0_IDLE;
					//1 does here lose memory?
					req = NULL;
					usb_clear(USB_CTL_EPx_BUF_READY, USB_EP0_CTRL);
					__set_ep_irqs(ep, USB_INT_EP0_SETUP_END);
					goto req_wrong;
				}
				__set_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
			req_wrong:
				break;
			default :
				DEBUG("ep0 statement : ep0_idle\n");
				DEBUG("ep0 i/o, odd state %d\n", dev->ep0state);
				dev->ep0state	= EP0_IDLE;
				local_irq_restore (flags);
				return -EL2HLT;
			}
		} else if (1 == ep_index(ep)) {
			DEBUG("endpoint1 in\n");
			/* The request should be done in IRQ handler */
			if (0) {
				__mask_ep_irqs(ep, USB_INT_EPx_MASK);
				dma_in_config(ep, req);
			} else {
				send_data(ep, req);
			}
			//req	= NULL;	
		} else if (2 == ep_index(ep)) {
			DEBUG("endpoint2 out\n");
			/* The request should be done in IRQ handler */
			//usb_write(USB_INT_EPx_MASK, USB_EPx_INT_CLR(2));
			__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
			/* Generate a interrupt when OUT token arrived. */
			__set_ep_irqs(ep, USB_INT_EP_BUF_NREADY);
		} else {
			DEBUG("Unknown endpoint %s received\n", ep->ep.name);
			local_irq_restore(flags);
			return -EL2HLT;
		}
	}

	if (NULL != req) {
		DEBUG("%s, request queued\n", ep->ep.name);
		list_add_tail(&req->queue, &ep->queue);
	}

	/* Restore interrupt. */
	local_irq_restore(flags);

	return 0;
}

static 
int sc8800_ep_dequeue(
				struct usb_ep		*_ep, 
				struct usb_request	*_req)
{
	struct sc8800_ep		*ep;
	struct sc8800_request	*req;
	unsigned long			flags;

	DEBUG("%s: ep name=%s\n", __func__, _ep->name);

	ep = container_of(_ep, struct sc8800_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/*--------------------------------------------------------------------------*
**                         Interface Function Definition                    *
**-------------------------------------------------------------------------*/

static const struct usb_ep_ops	sc8800_ep_ops	=
{
	.enable		= sc8800_ep_enable,
	.disable		= sc8800_ep_disable,
	.alloc_request	= sc8800_alloc_request,
	.free_request	= sc8800_free_request,
	.queue		= sc8800_ep_queue,
	.dequeue		= sc8800_ep_dequeue,
	.set_halt		= sc8800_ep_set_halt,
	.fifo_status	= NULL,
	.fifo_flush		= NULL
};

static int sc8800_get_frame(struct usb_gadget *gadget)
{
	pr_info("%s\r\n", __func__);
	return __raw_readl(USB_FRAME_NUM);
}
static int sc8800_set_selfpowered(struct usb_gadget *gadget, int is_selfpowered)
{
	unsigned long	flags;

	pr_info("%s\r\n", __func__);
	local_irq_save(flags);
	usb_set(BIT_1, USB_DEV_CTRL);
	local_irq_restore(flags);
	return 0;
}
static int sc8800_pullup(struct usb_gadget *gadget, int is_on)
{
	unsigned long	flags;

	pr_info("%s\r\n", __func__);
	local_irq_save(flags);
	//udc->enabled = is_on = !!is_on;
	usb_ldo_switch(is_on);
	local_irq_restore(flags);
	return 0;
}

static const struct usb_gadget_ops	sc8800_udc_ops	=
{
	.get_frame			= sc8800_get_frame,
	.wakeup				= NULL,
	.set_selfpowered	= sc8800_set_selfpowered,
	.vbus_session		= NULL,
	.vbus_draw			= NULL,
	.pullup				= sc8800_pullup,
	.ioctl				= NULL
};

/* The default configuration of sc8800_udc */
static struct sc8800_udc	the_controller =
{
	.gadget	= {
			.ops	= &sc8800_udc_ops,
			.ep0	= &the_controller.ep[0].ep,
			.name	= driver_name,
			.dev	= 
				{
					.init_name		= "gadget",
				}
		},
		
	.ep[0]	= {
			.ep		= 
				{
					.name		= ep0name,
					.ops		= &sc8800_ep_ops,
					.maxpacket	= EP0_PACKETSIZE,
				},
			.dev	= &the_controller,
			
			.bEndpointAddress	= 0,
			.bmAttributes		= USB_ENDPOINT_XFER_CONTROL,
			.fifo_in			= USB_EP0_IN_FIFO,
			.fifo_out			= USB_EP0_OUT_FIFO,
			.pfifo_in			= USB_EP0_IN_PFIFO,
			.pfifo_out			= USB_EP0_OUT_PFIFO
		},

	.ep[1]	= {
			.ep		= 
				{
					.name		= "ep1in-bulk",
					.ops		= &sc8800_ep_ops,
					.maxpacket	= BULK_PACKETSIZE
				},
			.dev	= &the_controller,

			.is_in			= 1,
			.bEndpointAddress	= USB_DIR_IN | 1,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
			.fifo_in			= USB_EP1_FIFO,
			.fifo_out			= 0,
			.pfifo_in			= USB_EP1_PFIFO
		},

	.ep[2]	= {
			.ep		= 
				{
					.name		= "ep2out-bulk",
					.ops		= &sc8800_ep_ops,
					.maxpacket	= BULK_PACKETSIZE
				},
			.dev	= &the_controller,

			.is_in			= 0,
			.bEndpointAddress	= USB_DIR_OUT | 2,
			.bmAttributes		= USB_ENDPOINT_XFER_BULK,
			.fifo_in			= 0,
			.fifo_out			= USB_EP2_FIFO,
			.pfifo_out			= USB_EP2_PFIFO
		}	
};


static void usb_hw_reset (void)
{
	unsigned long flags;
	struct sc8800_udc *udc= &the_controller;
	int i;
	
	/*pll init*/
	USB_SetPllDividor();

	usb_write(USB_INT_DEV_MASK, USB_INT_CLR);
	
	for (i = 0; i < SC8800_UDC_NUM_EPS; i++)	{
		struct sc8800_ep	*ep	= &udc->ep[i];
		__clear_ep_irqsts(ep, USB_INT_EPx_MASK);
	}
	local_irq_save(flags);
	//Disable USB CLK ;
	usb_mainclock_enable(0);																
	//Disable USB device ;
	usb_clear(USB_DEV_EN, USB_DEV_CTRL);

	local_irq_restore(flags);
	//Reset 48M and 12M clk ;
	usb_soft_reset();
	
	local_irq_save(flags);
	//Enable USB CLK ;
	usb_mainclock_enable(1);
	
	usb_set(USB_SELF_POWER_EN, USB_DEV_CTRL);																										
	
	local_irq_restore(flags);

	usb_set(USB_DEV_EN, USB_DEV_CTRL);																									    
	//Enable USB device ;

	usb_write(15, USB_TIME_SET);;
}


int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct sc8800_udc	 *udc = &the_controller;
	int	retval	= 0;

	DEBUG("%s: %s\n", __func__, driver->driver.name);

	//LDO_TurnOffLDO(LDO_LDO_USB);
	/* Verify the driver to be configured correctly */
	if (!driver
	    || driver->speed == USB_SPEED_UNKNOWN
	    || !driver->bind
	    || !driver->disconnect || !driver->setup) {
		pr_warning("invalid gadget driver %s\r\n", driver->driver.name);
		return -EINVAL;
	}
	if (!udc) {
		pr_warning("invalid device for gadget driver\r\n");
		return -ENODEV;
	}
	if (udc->driver) {
		pr_warning("device gadget driver already installed\r\n");
		return -EBUSY;
	}

	/* Register driver into sc8800_udc device */
	udc->driver	= driver;
	udc->gadget.dev.driver	= &driver->driver;

	usb_hw_reset();
	//according to usb2.0, after detecting device plug in, 100ms is used for reset
	msleep(200);

	DEBUG("%s: binding gadget into driver\n", __func__);
	/* Callback driver to bind the gadget device with driver */
	retval = driver->bind(&udc->gadget);
	if (retval)	{
		pr_debug("%s: bind to driver %s --> error %d\n", udc->gadget.name,
		       driver->driver.name, retval);
		udc->driver = NULL;
		udc->gadget.dev.driver	= NULL;
		return retval;
	}

	DEBUG("%s: registered gadget driver '%s'\n", udc->gadget.name,
	       driver->driver.name);
	/* Start up device */
	udc_enable(udc);
	//LDO_TurnOnLDO(LDO_LDO_USB);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sc8800_udc	*udc= &the_controller;

	if (!udc)
		return -ENODEV;
	if (!driver || driver != udc->driver)
		return -EINVAL;

	raw_local_irq_disable();
	udc_disable(udc);
	stop_activity(udc, driver);
	raw_local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->driver = NULL;

	udc->connected	= FALSE;
	udc->configured	= FALSE;


	DEBUG("unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

static int sc8800_udc_hw_init(void)
{
	unsigned long flags;

	//Disable USB LDO
	usb_ldo_switch(0);
	//Disable USB CLK ;
	//PWRMNG_USBD_ClkSwtch(FALSE);
	usb_mainclock_enable(0);

	//Disable USB device ;
	//ctl_usb->ctrl.mBits.en 		= FALSE; 
	usb_set(USB_DEV_EN, USB_DEV_CTRL);

	USB_SetPllDividor();

	//Reset 48M and 12M clk ;
	usb_soft_reset();

	local_irq_save(flags);

	//Clk_usb_en
		//REG32(GR_CLK_EN) |= BIT_11;
	usb_set(BIT_11, GR_CLK_EN);

	//Enable USB CLK ;
	//PWRMNG_USBD_ClkSwtch(TRUE);
	usb_mainclock_enable(1);
	//Self power;
	//USB_PowerSet(1);
	usb_set(USB_SELF_POWER_EN, USB_DEV_CTRL);
	//*(unsigned int *) USB_TIME_SET    = 15; 
	usb_write(15, USB_TIME_SET);;

	/* Enable all udc level interrupts */
	usb_write(USB_INT_DEV_MASK, USB_INT_CTRL);
	/* Enable ep0 interrupt */
	//usb_write(USB_INT_EPx_MASK, USB_EP0_INT_CTRL);
	local_irq_restore(flags);

	usb_set(USB_DEV_EN, USB_DEV_CTRL);
	return 0;
}
static
int sc8800_udc_probe(struct platform_device *pdev)
{
	struct sc8800_udc	 *udc	= &the_controller;
	struct device *dev = &pdev->dev;
	int	retval	= 0;

	/* Link the USB device controller object and platform device object*/
	udc->pdev = pdev;
	udc->gadget.dev.parent = dev;
	/* Initialize the device struct. */
	retval = device_register(&udc->gadget.dev);
	if (retval < 0)
		goto fail0;
	platform_set_drvdata(pdev, udc);

	/* Disable the usb device controller. */
	udc_disable(udc);

	/* Initialize the software statment. */
	udc_reinit(udc);
	
	/* Initialize other field. */
	udc->connected	= FALSE;
	udc->configured	= FALSE;
	spin_lock_init(&udc->lock);

	/* Register the IRQ of USB device. */
	retval	= request_irq(IRQ_USBD_INT, sc8800_udc_irq, 0, driver_name, udc);	
	if (0 != retval)	{
		printk(KERN_ERR "%s: can't request irq\n", driver_name);
		goto fail1;
	}

	if (g_use_dma) {
		#ifdef USB_DMA
		DMA_RegCallBack(DMA_USB_EP1,	sc8800_dma_ep1);
		DMA_RegCallBack(DMA_USB_EP2,    sc8800_dma_ep2);
		#endif
	}
	return 0;
fail1:
	device_unregister(&udc->gadget.dev);
fail0:
	return retval;	
}

static
int	sc8800_udc_remove(struct platform_device *pdev)
{
	struct sc8800_udc *udc = platform_get_drvdata(pdev);

	//printk("%s: %p\n", __func__, dev);

	udc_disable(udc);

	free_irq(IRQ_USBD_INT, udc);

	platform_set_drvdata(pdev, 0);
	
	device_unregister(&udc->gadget.dev);
	udc->connected	= FALSE;
	udc->configured	= FALSE;
	
	return 0;
}
 
static struct platform_driver sc8800_udc_driver = {
	.driver = {
		.name		= "sc8800_udc",
		.owner	= THIS_MODULE,
		},
	.probe		= sc8800_udc_probe,
	.suspend    = NULL,
	.resume		= NULL,
	.remove		= sc8800_udc_remove
};

static struct platform_device sc8800_device = {
	.name	= "sc8800_udc",
	.id	= 0,
};

//static unsigned long chg_gpio_cfg = MFP_CFG_X(CLK_LCD, GPIO, DS1, PULL_NONE, IO_IE);

#define IRQ_CHARGER 33
#define CHARGER_DECTECT_GPIO 162

static irqreturn_t
charger_dectct_handler(int irq, void *dev)
{

	disable_irq_nosync(irq);
/*
	enable_irq(irq);
	pr_info("charger  irq \n");
*/
//	usb_connect();
	return IRQ_HANDLED;
}

static __init int	setup_charge_dectect(void )
{
	int chg_gpio;
	int err;
	
	//sprd_mfp_config(&chg_gpio_cfg, 1);
	chg_gpio = CHARGER_DECTECT_GPIO;//mfp_to_gpio(MFP_CFG_TO_PIN(bl_gpio_cfg));
	pr_info("charger dectect gpio is:%d\r\n", chg_gpio);
	
	err = gpio_request(chg_gpio, "charger detect");
	if (err) {
		pr_warning("cannot alloc gpio for backlight\r\n");
		return err;
	}
	gpio_direction_input(chg_gpio);

	sprd_gpio_irq_register(chg_gpio, IRQ_CHARGER);

	err = request_threaded_irq(IRQ_CHARGER,NULL,  charger_dectct_handler, 
		 IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "charger dectct irq", NULL);
		//| IRQF_NOAUTOEN | IRQF_ONESHOT, "headset irq", NULL);
	if (err) {
		pr_warning("cannot alloc irq for charger, err %d\r\n", err);
		return err;
	}
	return 0;
}

static __init int sc8800_udc_init(void)
{    
	int ret;

	pr_info("%s\n", __func__);

	g_use_dma	= 0;

	PWRMNG_SetUsbClkSrc();

	ret = platform_driver_register(&sc8800_udc_driver);
	//ret=0;
	if (ret) {
		goto out;
	}
	ret = platform_device_register(&sc8800_device);
	if (ret) {
		pr_info("%s failed to register device with %d\n", __func__, ret);
		platform_driver_unregister(&sc8800_udc_driver);
		goto out;
	}
	
	sc8800_udc_hw_init();
	setup_charge_dectect();
	return 0;
out:
	return ret;
}


static __exit void sc8800_udc_cleanup(void)
{
	platform_device_unregister(&sc8800_device);
	platform_driver_unregister(&sc8800_udc_driver);
}

module_init(sc8800_udc_init);
module_exit(sc8800_udc_cleanup);

