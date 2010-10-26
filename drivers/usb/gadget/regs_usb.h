/*
 * linux/drivers/usb/gadget/regs_usb.h
 * SpreadTrum on-chip full speed USB device controllers
 *
 * Copyright (C) 2010 Yingchun Li yingchunli@spreadtrum.com
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
#ifndef _SC8800H_REG_USB_
#define _SC8800H_REG_USB_

#include <mach/hardware.h>
#include <mach/bits.h>

/*----------USB Ctronl Registers----------*/
//#define USB_REG_BASE            		0x20300000
#define USB_REG_BASE			SPRD_USB_BASE
#define USB_REG_PHY				SPRD_USB_PHYS

#define USB_DEV_CTRL                	(USB_REG_BASE + 0x0000)
# define USB_DEV_EN			(BIT_0)
# define USB_SELF_POWER_EN		(BIT_1)
#define USB_DEV_STS                	(USB_REG_BASE + 0x0004)
#define USB_DEV_RESET              	(USB_REG_BASE + 0x0008)
#define USB_EPx_RESET              	(USB_REG_BASE + 0x000C)
# define USB_RST_FIFO_EP0IN		(BIT_0)
# define USB_RST_FIFO_EP0OUT		(BIT_1)
# define USB_RST_FIFO_1			(BIT_2)
# define USB_RST_FIFO_2			(BIT_3)
# define USB_RST_FIFO_3			(BIT_4)
# define USB_RST_FIFO_4			(BIT_5)
# define USB_RST_TOGGLE_0		(BIT_6)
# define USB_RST_TOGGLE_1		(BIT_7)
# define USB_RST_TOGGLE_2		(BIT_8)
# define USB_RST_TOGGLE_3		(BIT_9)
# define USB_RST_TOGGLE_4		(BIT_10)

#define USB_SIE_CTRL               	(USB_REG_BASE + 0x0010)
#define USB_INT_CTRL               	(USB_REG_BASE + 0x0014)
#define USB_INT_STS               	(USB_REG_BASE + 0x0018)
#define USB_INT_CLR               	(USB_REG_BASE + 0x001C)
#define USB_INT_RAW               	(USB_REG_BASE + 0x0020)
#define USB_FRAME_NUM              	(USB_REG_BASE + 0x0024)
#define USB_TIME_SET              	(USB_REG_BASE + 0x0028)

//Ep0
#define USB_EP0_BASE			(USB_REG_BASE + 0x40)
#define USB_TRANSF_SIZE_EP0_IN     	(USB_REG_BASE + 0x0040)
#define USB_TRANSF_SIZE_EP0_OUT    	(USB_REG_BASE + 0x0044)
#define USB_SEND_DATA_NUM_EP0	   	(USB_REG_BASE + 0x0048)
#define USB_RECEIVED_DATA_NUM_EP0  	(USB_REG_BASE + 0x004C)
#define USB_REMOTE			(USB_REG_BASE + 0x0050)
#define USB_CONFIG			(USB_REG_BASE + 0x0054)
#define USB_SETTING			(USB_REG_BASE + 0x0058)
#define USB_SETUP_LOW			(USB_REG_BASE + 0x005C)
#define USB_SETUP_HIGH			(USB_REG_BASE + 0x0060)
#define USB_EP0_CTRL			(USB_REG_BASE + 0x0064)
#define USB_EP0_INT_CTRL	 	(USB_REG_BASE + 0x0068)
#define USB_EP0_INT_STS			(USB_REG_BASE + 0x006C)
#define USB_EP0_INT_CLR			(USB_REG_BASE + 0x0070)
#define USB_EP0_INT_RAW			(USB_REG_BASE + 0x0074)
#define USB_EP0_DMA_IN_WAITE_TIME  	(USB_REG_BASE + 0x0078)
#define USB_EP0_DMA_OUT_WAITE_TIME 	(USB_REG_BASE + 0x007C)

//Epx
#define USB_EP_CTRL			(0x0000) // 0xc0
//received or sent data; readonly
#define USB_EP_XFER_DATA	    	(0x0004) // 0xc4
#define USB_EP_XFER_SIZE	  	(0x0008) // 0xc8
#define USB_EP_INT_CTRL		 	(0x000C) // 0xcc
#define USB_EP_INT_STS			(0x0010) // 0xd0
#define USB_EP_INT_CLR			(0x0014) // 0xd4
#define USB_EP_INT_RAW			(0x0018) // 0xd8
#define USB_EP_DMA_WAITE_TIME	  	(0x001C) // 0xdc

#define USB_EPx_BASE			(USB_REG_BASE + 0xc0)

#define USB_EP1_BASE			(USB_EPx_BASE + 0x40*0)
#define USB_EP2_BASE			(USB_EPx_BASE + 0x40*1)
#define USB_EP3_BASE			(USB_EPx_BASE + 0x40*2)
#define USB_EP4_BASE			(USB_EPx_BASE + 0x40*3)

#define USB_EPx_CTRL(x)	\
	(USB_EPx_BASE + 0x40*(x-1) + USB_EP_CTRL ) // 0xc0
#define USB_EPx_XFER_DATA(x)	\
	(USB_EPx_BASE + 0x40* (x-1) + USB_EP_XFER_DATA) // 0xc4
#define USB_EPx_XFER_SIZE(x)	\
	(USB_EPx_BASE + 0x40*(x-1) + USB_EP_XFER_SIZE) // 0xc8
#define USB_EPx_INT_CTRL(x)		\
	(USB_EPx_BASE + 0x40*(x-1)+ USB_EP_INT_CTRL) // 0xcc
#define USB_EPx_INT_STS(x)		\
	(USB_EPx_BASE + 0x40*(x-1)+ USB_EP_INT_STS) // 0xd0
#define USB_EPx_INT_CLR(x)		\
	(USB_EPx_BASE + 0x40*(x-1)+ USB_EP_INT_CLR) // 0xd4
#define USB_EPx_INT_RAW(x)		\
	(USB_EPx_BASE + 0x40*(x-1) + USB_EP_INT_RAW) // 0xd8
#define USB_EPx_DMA_WAITE_TIME(x)	  \
	(USB_EPx_BASE + 0x40*(x-1) + USB_EP_DMA_WAITE_TIME) // 0xdc

/* Endpoint's FIFO address, virtual and physical*/
#define USB_EP0_IN_FIFO			(USB_REG_BASE + 0x80000)
#define USB_EP1_FIFO			(USB_REG_BASE + 0x80004)
#define USB_EP3_FIFO			(USB_REG_BASE + 0x80008)
#define USB_EP0_OUT_FIFO		(USB_REG_BASE + 0x8000c)
#define USB_EP2_FIFO			(USB_REG_BASE + 0x80010)
#define USB_EP4_FIFO			(USB_REG_BASE + 0x80014)

#define USB_EP0_IN_PFIFO		(USB_REG_PHY + 0x80000)
#define USB_EP1_PFIFO			(USB_REG_PHY + 0x80004)
#define USB_EP3_PFIFO			(USB_REG_PHY + 0x80008)
#define USB_EP0_OUT_PFIFO		(USB_REG_PHY + 0x8000c)
#define USB_EP2_PFIFO			(USB_REG_PHY + 0x80010)
#define USB_EP4_PFIFO			(USB_REG_PHY + 0x80014)

#define USB_MAX_TRANSFER_SIZE		(64*1024)

// USB EPx ID 
#define USB_EP0_IN			0
#define USB_EP0_OUT			1
#define USB_EP1				2
#define USB_EP2				3
#define USB_EP3				4
#define USB_EP4				5
#define USB_EPx_NUMBER			6

/*
 USB device controller's interrupt control bits
*/
#define USB_INT_SOF			(BIT_0)
#define USB_INT_SPEND			(BIT_1)
#define USB_INT_RESUM			(BIT_2)
#define USB_INT_RST			(BIT_3)
#define USB_INT_UNPID			(BIT_4)
#define USB_INT_EPID			(BIT_5)
#define USB_INT_CHANGE			(BIT_6)
#define USB_INT_DEVICE			(BIT_7)
#define USB_INT_EP0			(BIT_8)
#define USB_INT_EP1			(BIT_9)
#define USB_INT_EP2			(BIT_10)
#define USB_INT_EP3			(BIT_11)
#define USB_INT_EP4			(BIT_12)
#define USB_INT_DEV_MASK		0x7F

/* Endpoint's control bit */
#define USB_CTL_EPx_HALT			(BIT_11)
/*endpoint's transfer type BIT_23 and BIT_24*/
#define TRANS_TYPE_MASK				(0x3 << 23)
# define TRANS_TYPE_BULK			(0x0 << 23)
# define TRANS_TYPE_INTERRUPT			(0x1 << 23)
# define TRANS_TYPE_ISO				(0x2 << 23)
# define TRANS_TYPE_RESERVED			(0x3 << 23)
#define USB_CTL_EPx_ENABLE			(BIT_25)
#define USB_CTL_EPx_DATA_READY			(BIT_27)
#define USB_CTL_EPx_BUF_READY			(BIT_28)
#define USB_CTL_EPx_STALL			(BIT_29)

/*  Endpoint 0's specific control bit */
#define USB_CTL_EP0_SETUP_INT_INTERVENE		(BIT_11)
/*
	Endpoint Interupt control bits 
*/
#define USB_INT_EP_TRANSACT_END			(BIT_0)
/* for ep0 out, ep2, ep4 out endpoint's data toggle*/
#define USB_INT_EP_DATA_TOGGLE			(BIT_1) 
/* for ep0 in, ep1 and ep3  in endpoint's timeout */
#define USB_INT_EP_IN_TMOUT			(BIT_2)
/* for ep0 out, ep2 and ep4 out endpint's timeout */
#define USB_INT_EP_OUT_TMOUT			(BIT_3)
/* for only ep0's setup end timeout */
#define USB_INT_EP0_SETUP_TMOUT			(BIT_4)
/* for  ep0 in, ep2 and ep4 out crc16 error*/
#define USB_INT_EP_CRC16_ERR			(BIT_5)
/* for ep0 in, ep1 and ep3 in buffer empty */
#define USB_INT_EP_BUF_EMPTY			(BIT_6)
/* for ep0 out, ep2 and ep4 out buffer full */
#define USB_INT_EP_BUF_FULL			(BIT_7)
/* for only ep0's setup end */
#define USB_INT_EP0_SETUP_END			(BIT_8)
/* for all endpoint's transfer end */
#define USB_INT_EP_TRANSFER_END			(BIT_9)
/* for ep0 in, ep1 and ep3's DMA wait ack timeout */
#define USB_INT_EP_DMAIN_TMOUT			(BIT_10)
/* for ep0 out, ep2 and ep4's DMA wait ack timeout */
#define USB_INT_EP_DMAOUT_TMOUT			(BIT_11)
/* for ep0 out, ep2 and ep4's buffer not ready */
#define USB_INT_EP_BUF_NREADY			(BIT_12)
/* for ep0 in, ep1 and ep3's data not ready */
#define USB_INT_EP_DATA_NREADY			(BIT_13)
#define USB_INT_EPx_MASK			0x3FFF

#define USB_CLK_12M				12000000
#define USB_CLK_48M				48000000

#endif

