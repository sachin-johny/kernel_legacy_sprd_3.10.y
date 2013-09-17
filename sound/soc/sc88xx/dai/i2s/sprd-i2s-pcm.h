/*
 * sound/soc/sprd/dai/i2s/sprd-i2s-pcm.h
 *
 * SpreadTrum I2S for the pcm stream.
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#ifndef __SPRD_I2S_PCM_H
#define __SPRD_I2S_PCM_H

#include <mach/dma.h>


struct sprd_pcm_dma_params {
	char *name;		/* stream identifier */
	int channels;		/* channel id */
	int workmode;		/* dma work type */
	int irq_type;		/* dma interrupt type */
	struct sprd_dma_channel_desc desc;	/* dma description struct */
	u32 dev_paddr;		/* device physical address for DMA */
};

extern struct snd_soc_platform sprd_i2s_soc_platform;

#endif /* __SPRD_I2S_PCM_H */
