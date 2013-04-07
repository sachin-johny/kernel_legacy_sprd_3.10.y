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

#include <mach/hardware.h>
#include <mach/board.h>
#include <asm/memory.h>
#include <linux/memblock.h>

static int __init __iomem_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_IO_MEM_BASE, SPRD_IO_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_IO_MEM_BASE, SPRD_IO_MEM_SIZE))
		return -ENOMEM;
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
int __init __ramconsole_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -ENOMEM;
	return 0;
}
#endif

#if defined(CONFIG_SIPC) && !defined(CONFIG_ARCH_SC8825)
int __init __sipc_reserve_memblock(void)
{
	uint32_t smem_size = 0;

#ifdef CONFIG_SIPC_TD
	if (memblock_reserve(CPT_START_ADDR, CPT_TOTAL_SIZE))
		return -ENOMEM;
	smem_size += CPT_SMEM_SIZE;
#endif

#ifdef CONFIG_SIPC_WCDMA
	if (memblock_reserve(CPW_START_ADDR, CPW_TOTAL_SIZE))
		return -ENOMEM;
	smem_size += CPW_SMEM_SIZE;
#endif

#ifdef CONFIG_SIPC_WCN
	if (memblock_reserve(WCN_START_ADDR, WCN_TOTAL_SIZE))
		return -ENOMEM;
	smem_size += WCN_SMEM_SIZE;
#endif

	if (memblock_reserve(SIPC_SMEM_ADDR, smem_size))
		return -ENOMEM;

	return 0;
}
#endif

void __init sci_reserve(void)
{
	int ret = __iomem_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for iomem. errno=%d\n", ret);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ret = __ramconsole_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for ram_console. errno=%d\n", ret);
#endif

#if defined(CONFIG_SIPC) && !defined(CONFIG_ARCH_SC8825)
	ret = __sipc_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for sipc. errno=%d\n", ret);
#endif
}
