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
#ifndef _SPRD_DMA_COPY_K_H_
#define _SPRD_DMA_COPY_K_H_

typedef struct _dma_copy_cfg_tag {
	uint32_t len;
	uint32_t src_addr;
	uint32_t dst_addr;
}DMA_COPY_CFG_T, *DMA_COPY_CFG_T_PTR;

#endif
