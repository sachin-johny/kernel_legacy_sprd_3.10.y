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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/ctl_efuse.h>
#include "adi_internal.h"

#define CTL_EFUSE_BASE				( efuse_iobase )

static u32 efuse_iobase = 0;
void sci_efuse_poweron(void)
{
	//BUGBUG: how about ioremap and power?
}

void sci_efuse_poweroff(void)
{
}

int sci_efuse_read(unsigned blk)
{
	int busy = 0;
	BUG_ON(blk >= (MASK_READ_INDEX >> SHIFT_READ_INDEX));

	SCI_D(REG_EFUSE_BLOCK_INDEX) = BITS_READ_INDEX(blk);
	SCI_D(REG_EFUSE_MODE_CTRL) |= BIT_RD_START;

	do {
		//BUGBUG: how about timeout?
		busy = SCI_D(REG_EFUSE_STATUS) & BIT_READ_BUSY;
	} while (busy);

	return SCI_D(REG_EFUSE_DATA_RD);
}

int sci_efuse_program(unsigned blk, int data)
{
	return 0;
}

int sci_efuse_is_locked(unsigned blk)
{
	return 0;
}

/* low level */
int sci_efuse_raw_write(unsigned blk, int data, u32 magic)
{
	return 0;
}

int sci_efuse_lock(unsigned blk)
{
	return 0;
}
