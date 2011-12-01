/* 
 * sound/soc/sc88xx/sc88xx-asoc.h
 *
 * all share obj
 *
 * Copyright (C) 2010 SpreadTrum Ltd.
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

#include <linux/kernel.h>
#include <mach/dma.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/soc-dapm.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/mfp.h>
#include "sc88xx-pcm.h"
#include "vbc-codec.h"
#include "sc88xx-vbc.h"

#define lprintf(msg...) printk(KERN_EMERG "glx : %s() --> ", __func__);printk(msg)

#define SC88XX_PCM_DMA_SG_CIRCLE    1
#define SC88XX_VBC_DMA_COMBINE      1 
#define VBC_PCM_FORMATS             (SNDRV_PCM_FMTBIT_S16_LE) // | SNDRV_PCM_FMTBIT_S8)
extern void start_cpu_dma(struct snd_pcm_substream *substream);
extern void stop_cpu_dma(struct snd_pcm_substream *substream);
extern int cpu_codec_dma_chain_operate_ready(struct snd_pcm_substream *substream);
extern void flush_vbc_cache(struct snd_pcm_substream *substream);
extern int audio_playback_capture_channel(struct snd_pcm_substream *substream);
#define pause_cpu_dma() __raw_bits_or(1<<8, DMA_CFG)
#define resume_cpu_dma() __raw_bits_and(~(1<<8), DMA_CFG)

#define AUDIO_VBDA0    (1 << 0)
#define AUDIO_VBDA1    (1 << 1)
#define AUDIO_VBAD0    (1 << 2)
#define AUDIO_VBAD1    (1 << 3)
struct sprd_pcm_dma_params {
	char *name;			/* stream identifier */
    u32 aaf;  // audio codec addr choice flag
    u32 cfg;
    u32 tlen;
    u32 pmod;
    u32 sbm;
    u32 dbm;
};

struct sc88xx_runtime_data {
    int dma_channel;
    int pcm_1channel_data_width;
    int dma_da_ad_1_offset;
	struct sprd_pcm_dma_params *params;
	sprd_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
    sprd_dma_desc *dma_desc_array1;
	dma_addr_t dma_desc_array_phys1;
#if !SC88XX_PCM_DMA_SG_CIRCLE
    int ch_max;
    int ch0_idx;
    int ch1_idx;
#endif
};

#endif
