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
#include <linux/dma-contiguous.h>
#include <linux/platform_device.h>
#include "devices.h"

#include <asm/setup.h>

static int __init __iomem_reserve_memblock(void)
{
	int ret;

#ifndef CONFIG_CMA
	if (memblock_is_region_reserved(SPRD_ION_MEM_BASE, SPRD_ION_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_ION_MEM_BASE, SPRD_ION_MEM_SIZE))
		return -ENOMEM;
#else
#ifndef CONFIG_OF
	ret = dma_declare_contiguous_reserved(&sprd_ion_dev.dev, SPRD_ION_MEM_SIZE, SPRD_ION_MEM_BASE, 0, CMA_RESERVE, CMA_THRESHOLD);
	if (unlikely(ret))
	{
		pr_err("reserve CMA area(base:%x size:%x) for ION failed!!!\n", SPRD_ION_MEM_BASE, SPRD_ION_MEM_SIZE);
		return -ENOMEM;
	}
	pr_info("reserve CMA area(base:%x size:%x) for ION\n", SPRD_ION_MEM_BASE, SPRD_ION_MEM_SIZE);
#endif
#endif
	return 0;
}

#ifdef CONFIG_PSTORE_RAM
int __init __ramconsole_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -ENOMEM;
	return 0;
}
#endif

#ifdef CONFIG_FB_LCD_RESERVE_MEM
static int __init __fbmem_reserve_memblock(void)
{
	pr_err("__fbmem_reserve_memblock,SPRD_FB_MEM_BASE:%x,SPRD_FB_MEM_SIZE:%x\n",SPRD_FB_MEM_BASE,SPRD_FB_MEM_SIZE);
	if (memblock_is_region_reserved(SPRD_FB_MEM_BASE, SPRD_FB_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_FB_MEM_BASE, SPRD_FB_MEM_SIZE))
		return -ENOMEM;
	pr_err("__fbmem_reserve_memblock-end,\n");
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

#ifdef CONFIG_SPRD_IQ
static phys_addr_t s_iq_addr = 0xffffffff;
int in_iqmode(void);

int __init __sprd_iq_memblock(void)
{
	int i, j;
	struct membank bank;
	bool bfound = false;
	if(!in_iqmode())
		return -EINVAL;
	for(i = meminfo.nr_banks; i > 0; i--) {
		printk("high: %d, start %d, size %d \n", meminfo.bank[i-1].highmem, meminfo.bank[i-1].start,
			meminfo.bank[i-1].size);
		if(meminfo.bank[i-1].highmem || meminfo.bank[i-1].size < SPRD_IQ_SIZE)
			continue;
		bank.start = meminfo.bank[i-1].start;
		bank.size = meminfo.bank[i-1].size;
		while(bank.size - SPRD_IQ_SIZE > 0) {
			if(memblock_is_region_reserved(bank.start + bank.size - SPRD_IQ_SIZE, SPRD_IQ_SIZE)) {
				bank.size -= SZ_1M;
			} else {
				bfound = true;
				break;
			}
		}
		if(bfound)
			break;
	}
	printk("found mem %d \n", bank.size);
	if(bfound) {
		if(memblock_reserve(bank.start + bank.size - SPRD_IQ_SIZE, SPRD_IQ_SIZE))
			return -ENOMEM;
		else {
			s_iq_addr = bank.start + bank.size - SPRD_IQ_SIZE;
			return 0;
		}
	} else
		return -ENOMEM;

}

phys_addr_t sprd_iq_addr(void)
{
	return s_iq_addr;
}

#endif


void __init sci_reserve(void)
{
#ifndef CONFIG_OF
	int ret;
	ret = __iomem_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for iomem. errno=%d\n", ret);

#if defined(CONFIG_SIPC) && !defined(CONFIG_ARCH_SC8825)
	ret = __sipc_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for sipc. errno=%d\n", ret);
#endif

#ifdef CONFIG_PSTORE_RAM
	ret = __ramconsole_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for ram_console. errno=%d\n", ret);
#endif

#ifdef CONFIG_FB_LCD_RESERVE_MEM
	ret = __fbmem_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for framebuffer . errno=%d\n", ret);
#endif
#endif

#ifdef CONFIG_SPRD_IQ
	ret = __sprd_iq_memblock();
	if (ret != 0)
		printk("Fail to reserve mem for sprd iq. errno=%d\n", ret);

#endif
}
