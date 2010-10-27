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
#endif
