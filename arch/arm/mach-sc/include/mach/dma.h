/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef __MACH_DMA_H
#define __MACH_DMA_H


#if defined(CONFIG_ARCH_SC8825) || defined(CONFIG_ARCH_SC7710)
#define DMA_VER_R1P0
#include <mach/dma_r1p0.h>
#endif


#ifdef CONFIG_ARCH_SC8830
#define DMA_VER_R4P0
#include <mach/dma_r4p0.h>
#endif

#endif
