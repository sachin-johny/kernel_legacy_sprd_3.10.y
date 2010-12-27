/* drivers/video/sc8800g/sc8800g_copybit_lcdc.h
 *
 * copybit alpha blending/blit driver based on sc8800g lcdc
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Geng Ren <geng.ren@spreadtrum.com>
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

#ifndef __SC8800G_COPYBIT_LCDC_H
#define __SC8800G_COPYBIT_LCDC_H

#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <mach/sc8800g_2d.h>


/**
 * do_copyblit_lcdc 
 *
 */
int do_copybit_lcdc(struct s2d_blit_req * req);

int copybit_lcdc_init(void);

int copybit_lcdc_exit(void);

#endif
