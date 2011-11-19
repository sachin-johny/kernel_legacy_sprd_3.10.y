/*
* drivers/media/video/sprd_dcam/dcam_common.h
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

#ifndef _DCAM_COMMON_H_
#define _DCAM_COMMON_H_
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/io.h>
#include <mach/io.h>
#include <mach/bits.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define __pad(a) __raw_readl(a)
#define _pard(a) __raw_readl(a)
#define _pawd(a,v) __raw_writel(v,a)
#define _paad(a,v) __raw_bits_and(v,a)
#define _paod(a,v) __raw_bits_or(v,a)
#define _pacd(a,v) __raw_bits_xor(v,a)
#define _pamd(a,m,v) do{uint32 _tmp=__raw_readl(a); _tmp &= ~(m); __raw_writel(_tmp|((m)&(v)), (a));}while(0)

//#define DCAM_DEBUG
#ifdef DCAM_DEBUG
#define DCAM_TRACE printk
#else
#define DCAM_TRACE(...)
#endif
#define DCAM_TRACE_ERR printk

//#define SENSOR_DEBUG
#ifdef SENSOR_DEBUG
#define SENSOR_TRACE printk
#else
#define SENSOR_TRACE(...)
#endif

#define BOOLEAN char
#define PNULL  ((void *)0)
#define PUBLIC 
#define LOCAL static
#define DCAM_NULL 0
#define DCAM_Sleep msleep


#define DCAM_SUCCESS 0
#define DCAM_FAIL 1
#define DCAM_FALSE 0
#define DCAM_TRUE 1
#ifndef DCAM_ASSERT
#define DCAM_ASSERT(a) do{}while(!(a));
#endif
#ifndef DCAM_MEMCPY
#define DCAM_MEMCPY memcpy
#endif

typedef void *DCAM_MUTEX_PTR;
typedef void *DCAM_SEMAPHORE_PTR;
typedef void *DCAM_TIMER_PTR;

//for sensor
#define SENSOR_SUCCESS 0
#define SENSOR_FAIL 1
#define SENSOR_FALSE 0
#define SENSOR_TRUE 1
#define SENSOR_ASSERT(a) do{}while(!(a));
#define SENSOR_Sleep(m) msleep(m)
#define SENSOR_MEMSET memset
#define SENSOR_MALLOC kmalloc
typedef void *SENSOR_MUTEX_PTR;
#define SENSOR_PASSERT(m, n) if(!m) printk n 

#endif  // _DCAM_COMMON_H_
