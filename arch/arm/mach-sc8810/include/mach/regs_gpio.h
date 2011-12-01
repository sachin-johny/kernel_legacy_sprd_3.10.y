/*
 * arch/arm/mach-sc8810/include/mach/regs_gpio.h
 *
 * Chip Gpio Config registers Definitions
 *
 * Copyright (C) 2011 Spreadtrum International Ltd.
 *
 * 2010-03-05: yingchun li <yingchun.li@spreadtrum.com>
 *            initial version
 * 2011-11-17: steve.zhan <steve.zhan@spreadtrum.com>
 * 	      modify for sc8810 project
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _SC8810_REG_GPIO_H_
#define _SC8810_REG_GPIO_H_

#include <mach/hardware.h>

#define GPIO_BASE               		SPRD_GPIO_BASE


/*----------GPIO ARM Control Register----------*/
#define GPIO_PG_BASE                    GPIO_BASE
#define GPI_PG0_BASE                GPIO_BASE

//GPIO_OFFSET
#define GPIO_DATA                               0x0000    //GPIO data register
#define GPIO_DMSK                               0x0004    //GPIO data mask register, GPIO pin can be read and write if the mask bit is "1"
#define GPIO_DIR                                0x0008    //"1" configure pin to be output"0" configure pin to be input
#define GPIO_IS                                 0x000C    //Interrupt sense register. "1" detect levels, "0" detect edges
#define GPIO_IBE                                0x0010    //Interrupt both edges register. "1" both edges trigger an interrupt, "0" interrupt generation event is controlled by GPIOIEN
#define GPIO_IEV                                0x0014    //Interrupt event register, "1" rising edges or high levels trigger interrupts, "0" falling edges or low levels trigger interrupts.
#define GPIO_IE                                 0x0018    //Interrupt mask register, "1" corresponding pin is not masked. "0" corresponding pin interrupt is masked
#define GPIO_RIS                                0x001C    //Row interrupt status, reflect the status of interrupts trigger conditions detection on pins (prior to masking). "1" interrupt condition met "0" condition not met
#define GPIO_MIS                                0x0020    //Masked interrupt status, "1" Interrupt active "0" interrupt not active
#define GPIO_IC                                 0x0024    //Interrupt clear, "1" clears edge detection interrupt. "0" has no effect.
#define GPIO_INEN			                    0x0028

//GPI_OFFSET
#define GPI_DATA                        0x0000    //GPI data register, original input signal, not through de-bounce path.
#define GPI_DMSK                        0x0004    //GPI data mask register. GPIDATA register can be read if the mask bit is "1"
#define GPI_IEV                         0x0014    //Interrupt event register, "1" high levels trigger interrupts, "0" low levels trigger interrupts.
#define GPI_IE                          0x0018    //Interrupt mask register, "1" corresponding pin is not masked. "0" corresponding pin interrupt is masked
#define GPI_RIS                         0x001C    //Row interrupt status, reflect the status of interrupts trigger conditions detection on pins (prior to masking). "1" interrupt condition met "0" condition not met
#define GPI_MIS                         0x0020    //Masked interrupt status, "1" Interrupt active "0" interrupt not active
#define GPI_IC                          0x0024    //Interrupt clear, "1" clears level detection interrupt. "0" has no effect.
#define GPI_0CTRL                       0x0028    //GPI0:...
#define GPI_1CTRL                       0x002C    //GPI1:...
#define GPI_2CTRL                       0x0030    //GPI2:...
#define GPI_3CTRL                       0x0034    //GPI3:...
#define GPI_4CTRL                       0x0038    //GPI4:...
#define GPI_5CTRL                       0x003C    //GPI5:...
#define GPI_6CTRL                       0x0040    //GPI5:...
#define GPI_7CTRL                       0x0044    //GPI5:...
#define GPI_TRIG                                0x0048

#define GPI_DEBOUNCE_BIT                 BIT_8
#define GPI_DEBOUNCE_PERIED                255


#endif

