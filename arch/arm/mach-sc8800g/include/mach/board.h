/* arch/arm/mach-sc8800g/include/mach/board.h
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_SPRD_BOARD_H
#define __ASM_ARCH_SPRD_BOARD_H

#include <linux/types.h>

/* platform device data structures */

struct sprd_platform_data
{
	void (*panel_power)(int on);
	unsigned has_vsync_irq:1;
};

/* common init routines for use by arch/arm/mach-sprd/board-*.c */
void __init sprd_add_devices(void);
void __init sprd_map_common_io(void);
void __init sprd_init_irq(void);
void __init sprd_add_sdio_device(void);
void __init sprd_add_otg_device(void);
void __init sprd_gadget_init(void);
void __init sprd_add_dcam_device(void);
void __init sprd_charger_init(void);

/* pmem area definition */
/*
 *  8M - 2D
 *  6M - Camara/video codec
 *  1M - Rotation
 *  1M - scaling
 */
extern unsigned long sdram_plimit;
unsigned long get_sdram_plimit(void);
#define SPRD_PMEM_SIZE          (8*1024*1024)
#ifdef CONFIG_CAMERA_2M
#define SPRD_PMEM_ADSP_SIZE   (8*1024*1024)//  for 2M
#else
#define SPRD_PMEM_ADSP_SIZE   (14*1024*1024)// for 3M
#endif
#define SPRD_ROT_MEM_SIZE       (1024*512)
#define SPRD_SCALE_MEM_SIZE    (1024*512)
#define SPRD_IO_MEM_SIZE        (SPRD_PMEM_SIZE+SPRD_PMEM_ADSP_SIZE+ \
                                SPRD_ROT_MEM_SIZE+SPRD_SCALE_MEM_SIZE)

#define SPRD_PMEM_END           (128*1024*1024) /* alway 128M */
#define SPRD_PMEM_BASE          (SPRD_PMEM_END-SPRD_IO_MEM_SIZE)
#define SPRD_PMEM_ADSP_BASE     (SPRD_PMEM_BASE+SPRD_PMEM_SIZE)
#define SPRD_ROT_MEM_BASE       (SPRD_PMEM_ADSP_BASE+SPRD_PMEM_ADSP_SIZE)
#define SPRD_SCALE_MEM_BASE     (SPRD_ROT_MEM_BASE+SPRD_ROT_MEM_SIZE)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
# define RAM_CONSOLE_SIZE    0x20000
# define RAM_CONSOLE_START   (SPRD_PMEM_BASE - RAM_CONSOLE_SIZE)
#endif

extern void udc_enable(void);
extern void udc_disable(void);
extern int in_factory_mode(void);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern int __init sprd_ramconsole_init(void);
extern void sprd_ramconsole_reserve_sdram(void);
#else
static inline int sprd_ramconsole_init(void) {return 0;}
static inline void sprd_ramconsole_reserve_sdram(void){}
#endif
extern void sprd_pmem_reserve_sdram(void);

#endif

