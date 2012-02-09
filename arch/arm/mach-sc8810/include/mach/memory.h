/* arch/arm/mach-sc8800g/include/mach/memory.h
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#define PHYS_OFFSET		UL(0x0)

/* bus address and physical addresses are identical */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#define __pfn_to_bus(x) __pfn_to_phys(x)
#define __bus_to_pfn(x)	__phys_to_pfn(x)


#ifdef CONFIG_DISCONTIGMEM

/*<= because these is bug on mmu.c if (__va(bank->start + bank->size) > VMALLOC_MIN ||*/
#define __virt_to_phys(x)	 (((x) >= 0xd0000000  && (x) <= 0xe0000000) ? ((x)+0x10000000 ): ((x) - PAGE_OFFSET + PHYS_OFFSET))
#define __phys_to_virt(x)	 (((x) >= 0xe0000000  && (x) <= 0xf0000000 ) ? ((x)-0x10000000) : ((x) - PHYS_OFFSET + PAGE_OFFSET))

#define arch_local_page_offset(pfn, nid) ((pfn) - NODE_DATA(nid)->node_start_pfn)
#define virt_to_page(x)  pfn_to_page((unsigned long)(__virt_to_phys(   (unsigned long)(x)  ) )>> PAGE_SHIFT)

/*
 * Given a kernel address, find the home node of the underlying memory.
 */

 #define KVADDR_TO_NID(addr) \
	((addr < 0xd0000000) ?( 0) : (1))
/*
 * Given a page frame number, convert it to a node id.
 */

#define PFN_TO_NID(pfn) \
	((pfn > 0x10000) ? (1) :  (0))

/*
 * Given a kaddr, LOCAL_MEM_MAP finds the owning node of the memory
 * and returns the index corresponding to the appropriate page in the
 * node's mem_map.
 */

#define LOCAL_MAP_NR(addr) \
	(((unsigned long)(addr) & 0x0fffffff) >> PAGE_SHIFT)



#endif

#if 0
/*
 * Sparsemem version of the above
 */
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28
#endif

#endif

