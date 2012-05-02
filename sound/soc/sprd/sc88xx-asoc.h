/*
 * sound/soc/sprd/sc88xx-asoc.h
 *
 * all shared obj
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
#ifndef __SC88XX_ASOC_H
#define __SC88XX_ASOC_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/slab.h>
#include <sound/initval.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/dma.h>
#include <sound/tlv.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include "vbc-codec.h"

#define VBC_PCM_FORMATS		SNDRV_PCM_FMTBIT_S16_LE
extern void start_cpu_dma(struct snd_pcm_substream *substream);
extern void stop_cpu_dma(struct snd_pcm_substream *substream);
extern int cpu_codec_dma_chain_operate_ready(struct snd_pcm_substream *substream);
extern void flush_vbc_cache(struct snd_pcm_substream *substream);
extern int audio_playback_capture_channel(struct snd_pcm_substream *substream);

struct sc88xx_pcm_dma_params {
	char *name;		/* stream identifier */
	int channel;	/* channel id */
	int workmode;	/* dma work type */
	int irq_type;	/* dma interrupt type */
	struct sprd_dma_channel_desc desc; /* dma description struct */
	u32 dev_paddr;	/* device physical address for DMA */
	u32 dev_vaddr;	/* device virtual address for DMA */
	u32 dev_paddr1;	/* device physical address for DMA */
	u32 dev_vaddr1;	/* device virtual address for DMA */
};

typedef struct sprd_dma_desc {
	volatile u32 cfg;
	volatile u32 tlen;	/* Total transfer length, in bytes */
	volatile u32 dsrc;	/* Source address. This address value should align to the SRC_DATA_WIDTH */
	volatile u32 ddst;	/* Destination address. This address value should align to the DES_DATA_WIDTH. */
	volatile u32 llptr;	/* Linked list pointer to the next node address */
	volatile u32 pmod;	/* POST MODE */
	volatile u32 sbm;	/* src burst mode */
	volatile u32 dbm;	/* dst burst mode */
} sprd_dma_desc;

typedef struct sprd_dma_ctrl {
	int interrupt_type;
	int modes;
	int ch_id;
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_desc_phy;
} sprd_dma_ctrl;

struct sc88xx_runtime_data {
	int pcm_1channel_data_width;
	int dma_da_ad_1_offset;
	struct sc88xx_pcm_dma_params *params;
	int uid_cid_map[2];
	sprd_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
	sprd_dma_desc *dma_desc_array1;
	dma_addr_t dma_desc_array_phys1;
	sprd_dma_desc *dma_desc_array_dummy_pcm[2];
};

#define enter_critical()	\
	unsigned long flags;	\
	local_irq_save(flags)

#define exit_critical()		\
	local_irq_restore(flags)

#define not_in_adi_range(addr)							\
	(addr < ANA_REG_ADDR_START || addr > ANA_REG_ADDR_END)

#define check_range(addr)							\
	if (not_in_adi_range(addr)) {						\
		pr_err("AnalogDie addr 0x%08x not in [0x%08x, 0x%08x]\n",	\
				addr, ANA_REG_ADDR_START, ANA_REG_ADDR_END);	\
		return 0; \
	}

#endif
