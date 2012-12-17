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

#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H

#include <asm/sizes.h>

#include <mach/gpio.h>

#if defined (CONFIG_MACH_SP8830EA) || defined (CONFIG_MACH_SP8830FPGA)
#include "__gpio-sp8830ea.h"
#elif defined (CONFIG_MACH_SP8825EA)
#include "__gpio-sp8825ea.h"
#endif

#include "memory.h"

/*
 * pmem area definition
 */
#ifdef CONFIG_ANDROID_PMEM
#define SPRD_PMEM_SIZE		(CONFIG_SPRD_PMEM_SIZE*SZ_1M)
#define SPRD_PMEM_ADSP_SIZE	(CONFIG_SPRD_PMEM_ADSP_SIZE*SZ_1M)
#else
#define SPRD_PMEM_SIZE		(0*SZ_1M)
#define SPRD_PMEM_ADSP_SIZE	(0*SZ_1M)
#endif
#define SPRD_ROT_MEM_SIZE	(0)
#define SPRD_SCALE_MEM_SIZE	(0)

#ifdef CONFIG_ION
//#define SPRD_ION_SIZE           (CONFIG_SPRD_ION_SIZE*SZ_1M)

	#define SPRD_ION_SIZE   (24*1024*1024) // 1280x720 H264 HW Decoding needs 24M
/*
    #if defined(CONFIG_CAMERA_8M)
    #define SPRD_ION_SIZE   (23*1024*1024)
    #elif defined(CONFIG_CAMERA_5M)
    #define SPRD_ION_SIZE   (19*1024*1024)
    #elif defined(CONFIG_CAMERA_3M)
    #define SPRD_ION_SIZE   (19*1024*1024) // 13
    #elif defined(CONFIG_CAMERA_2M)
        #ifdef CONFIG_CAMERA_ROTATION
        #define SPRD_ION_SIZE   (19*1024*1024) // 13
        #else
        #define SPRD_ION_SIZE   (19*1024*1024) // 8
        #endif
    #else
    #define SPRD_ION_SIZE   (19*1024*1024)
    #endif
*/

    #define SPRD_ION_OVERLAY_SIZE   (CONFIG_SPRD_ION_OVERLAY_SIZE*SZ_1M)
#else
#define SPRD_ION_SIZE           (0*SZ_1M)
#define SPRD_ION_OVERLAY_SIZE   (0*SZ_1M)
#endif

#define SPRD_IO_MEM_SIZE	(SPRD_PMEM_SIZE+SPRD_PMEM_ADSP_SIZE+ \
				SPRD_ROT_MEM_SIZE+SPRD_SCALE_MEM_SIZE+ \
				SPRD_ION_SIZE+SPRD_ION_OVERLAY_SIZE)

#define SPRD_PMEM_BASE		(PLAT_PHYS_OFFSET + (256*SZ_1M)-SPRD_IO_MEM_SIZE)
#define SPRD_PMEM_ADSP_BASE	(SPRD_PMEM_BASE+SPRD_PMEM_SIZE)
#define SPRD_ROT_MEM_BASE	(SPRD_PMEM_ADSP_BASE+SPRD_PMEM_ADSP_SIZE)
#define SPRD_SCALE_MEM_BASE	(SPRD_ROT_MEM_BASE+SPRD_ROT_MEM_SIZE)
#define SPRD_ION_BASE           (SPRD_SCALE_MEM_BASE+SPRD_SCALE_MEM_SIZE)
#define SPRD_ION_OVERLAY_BASE   (SPRD_ION_BASE+SPRD_ION_SIZE)

#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SPRD_RAM_CONSOLE_SIZE	0x20000
#define SPRD_RAM_CONSOLE_START	(SPRD_PMEM_BASE - SPRD_RAM_CONSOLE_SIZE)
#endif

