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

#ifndef	_ION_SPRD_H
#define _ION_SPRD_H

#define ION_HEAP_CARVEOUT_ID0   2
#define ION_HEAP_CARVEOUT_ID1   3
#define ION_HEAP_CARVEOUT_ID2   4
#define ION_HEAP_CARVEOUT_ID3   5

#define ION_HEAP_CMA_ID0        6
#define ION_HEAP_CMA_ID1   	7
#define ION_HEAP_CMA_ID2   	8
#define ION_HEAP_CMA_ID3   	9

#define ION_HEAP_IOMMU_ID0      10
#define ION_HEAP_IOMMU_ID1   	11
#define ION_HEAP_IOMMU_ID2   	12
#define ION_HEAP_IOMMU_ID3   	13


#define ION_HEAP_CARVEOUT_MASK		(1 << ION_HEAP_CARVEOUT_ID0)

#define ION_DRIVER_VERSION 0

struct ion_phys_data {
	int fd_buffer;
	unsigned long phys;
	size_t size;
};

struct ion_msync_data {
	int fd_buffer;
	void *vaddr;
	void *paddr;
	size_t size;
};

enum ION_SPRD_CUSTOM_CMD {
	ION_SPRD_CUSTOM_PHYS,
	ION_SPRD_CUSTOM_MSYNC
};

#endif /* _ION_SPRD_H */
