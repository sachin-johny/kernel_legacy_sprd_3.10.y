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

#ifdef	CONFIG_MACH_SP8825EA
#include "__board-sp8825ea.h"
#endif

#ifdef	CONFIG_MACH_SP8830FPGA
#include "__board-sp8830fpga.h"
#endif

#include <asm/sizes.h>

#ifdef CONFIG_ION

    #if defined(CONFIG_CAMERA_8M)
    #define SPRD_ION_SIZE	(23*1024*1024)
    #elif defined(CONFIG_CAMERA_5M)
    #define SPRD_ION_SIZE	(19*1024*1024)
    #elif defined(CONFIG_CAMERA_3M)
    #define SPRD_ION_SIZE	(13*1024*1024)
    #elif defined(CONFIG_CAMERA_2M)
        #ifdef CONFIG_CAMERA_ROTATION
        #define SPRD_ION_SIZE	(13*1024*1024)
        #else
        #define SPRD_ION_SIZE	(8*1024*1024)
        #endif
    #else
    #define SPRD_ION_SIZE	(CONFIG_SPRD_ION_SIZE * SZ_1M)
    #endif

#define SPRD_ION_OVERLAY_SIZE   (CONFIG_SPRD_ION_OVERLAY_SIZE * SZ_1M)

#else /* !ION */
#define SPRD_ION_SIZE           (0 * SZ_1M)
#define SPRD_ION_OVERLAY_SIZE   (0 * SZ_1M)
#endif

#define SPRD_IO_MEM_SIZE	(SPRD_ION_SIZE + SPRD_ION_OVERLAY_SIZE)
#define SPRD_IO_MEM_BASE	\
	((CONFIG_PHYS_OFFSET & (~(SZ_256M - 1))) + SZ_256M - SPRD_IO_MEM_SIZE)

#define SPRD_ION_BASE		(SPRD_IO_MEM_BASE)
#define SPRD_ION_OVERLAY_BASE   (SPRD_ION_BASE + SPRD_ION_SIZE)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SPRD_RAM_CONSOLE_SIZE	0x20000
#define SPRD_RAM_CONSOLE_START	(SPRD_IO_MEM_BASE - SPRD_RAM_CONSOLE_SIZE)
#endif

struct sysdump_mem {
	unsigned long paddr;
	unsigned long vaddr;
	unsigned long soff;
	size_t size;
	int type;
};

extern int sprd_dump_mem_num;
extern struct sysdump_mem sprd_dump_mem[];

#endif
