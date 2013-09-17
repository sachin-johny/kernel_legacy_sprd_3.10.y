/*
 * sound/soc/sprd/dai/i2s/sprd-i2s-pcm.c
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
#define pr_fmt(fmt) "[audio:pcm] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <mach/dma.h>

#include "sprd-i2s-pcm.h"
#include "i2s.h"

#ifdef CONFIG_SPRD_IIS_DEBUG
#define sprd_pcm_dbg pr_debug
#else
#define sprd_pcm_dbg(...)
#endif

struct sprd_runtime_data {
	struct sprd_pcm_dma_params *params;
	int uid_cid_map;
	sprd_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
	int burst_len;
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	int buffer_in_iram;
#endif
};

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
#define SPRD_I2S_DMA_NODE_SIZE (1024)
#endif

static const struct snd_pcm_hardware sprd_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FORMAT_S16_LE,
	/* 16bits, stereo-2-channels */
	.period_bytes_min = 8 * 2,
	/* non limit */
	.period_bytes_max = 32 * 2 * 100,
	.periods_min = 1,
	/* non limit */
	.periods_max = PAGE_SIZE / sizeof(sprd_dma_desc),
	.buffer_bytes_max = 64 * 1024,
};

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
static char *s_mem_for_iram = 0;
static char *s_iram_remap_base = 0;

static int sprd_buffer_iram_backup(void)
{
	void __iomem *iram_start;
	sprd_pcm_dbg("Entering %s 0x%x\n", __func__, (int)s_mem_for_iram);
	if (!s_iram_remap_base) {
		s_iram_remap_base =
		    ioremap_nocache(SPRD_IRAM_ALL_PHYS, SPRD_IRAM_ALL_SIZE);
	}
	if (!s_mem_for_iram) {
		s_mem_for_iram = kzalloc(SPRD_IRAM_ALL_SIZE, GFP_KERNEL);
	} else {
		sprd_pcm_dbg("iram is backup, be careful use iram!\n");
		return 0;
	}
	if (!s_mem_for_iram) {
		pr_err("iram backup error\n");
		return -ENOMEM;
	}
	iram_start = (void __iomem *)(s_iram_remap_base);
	memcpy_fromio(s_mem_for_iram, iram_start, SPRD_IRAM_ALL_SIZE);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sprd_buffer_iram_restore(void)
{
	void __iomem *iram_start;
	sprd_pcm_dbg("Entering %s 0x%x\n", __func__, (int)s_mem_for_iram);
	if (!s_mem_for_iram) {
		pr_err("iram not backup\n");
		return 0;
	}
	iram_start = (void __iomem *)(s_iram_remap_base);
	memcpy_toio(iram_start, s_mem_for_iram, SPRD_IRAM_ALL_SIZE);
	kfree(s_mem_for_iram);
	s_mem_for_iram = 0;
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return 0;
}
#endif

static int sprd_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct i2s_private *i2s_private = srtd->dai->cpu_dai->private_data;
	struct i2s_config *config = i2s_private->config;
	int burst_len;
	int ret;

	pr_info("Entering %s %d\n", __func__, substream->stream);

	snd_soc_set_runtime_hwparams(substream, &sprd_pcm_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		burst_len = I2S_FIFO_DEPTH-config->tx_watermark;
	} else {
		burst_len = config->rx_watermark;
	}

        ret = snd_pcm_hw_constraint_step(runtime, 0,
                                         SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
                                         burst_len);
        if (ret)
                goto out;

        ret = snd_pcm_hw_constraint_step(runtime, 0,
                                         SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
                                         burst_len);
        if (ret)
                goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	ret = -ENOMEM;
	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		goto out;

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (!((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	      && 0 == sprd_buffer_iram_backup())) {
#endif
		rtd->dma_desc_array =
		    dma_alloc_writecombine(substream->pcm->card->dev,
					   PAGE_SIZE,
					   &rtd->dma_desc_array_phys,
					   GFP_KERNEL);
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	} else {
		runtime->hw.periods_max =
		    SPRD_I2S_DMA_NODE_SIZE / sizeof(sprd_dma_desc),
		    runtime->hw.buffer_bytes_max =
		    SPRD_IRAM_ALL_SIZE - (2 * SPRD_I2S_DMA_NODE_SIZE),
		    rtd->dma_desc_array =
		    (void *)(s_iram_remap_base + runtime->hw.buffer_bytes_max);
		rtd->dma_desc_array_phys =
		    SPRD_IRAM_ALL_PHYS + runtime->hw.buffer_bytes_max;
		rtd->buffer_in_iram = 1;
	}
#endif
	if (!rtd->dma_desc_array)
		goto err1;
	rtd->uid_cid_map = -1;
	runtime->private_data = rtd;
	ret = 0;
	goto out;

err1:
	kfree(rtd);
out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;

	pr_info("Entering %s %d\n", __func__, substream->stream);

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (rtd->buffer_in_iram)
		sprd_buffer_iram_restore();
	else
#endif
		dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
				      rtd->dma_desc_array,
				      rtd->dma_desc_array_phys);
	kfree(rtd);

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static irqreturn_t sprd_pcm_dma_irq_ch(int dma_ch, void *dev_id)
{
	snd_pcm_period_elapsed(dev_id);
	return IRQ_HANDLED;
}

static int sprd_pcm_dma_config(struct snd_pcm_substream *substream)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	struct sprd_pcm_dma_params *dma;
	struct sprd_dma_channel_desc dma_cfg = { 0 };
	dma_addr_t next_desc_phys;

	sprd_pcm_dbg("Entering %s\n", __func__);

	if (!rtd || !rtd->params)
		return 0;

	dma = rtd->params;
	dma_cfg = dma->desc;

	next_desc_phys = rtd->dma_desc_array_phys;
	if (rtd->uid_cid_map >= 0) {
		dma_cfg.llist_ptr = next_desc_phys;
		sprd_dma_channel_config(rtd->uid_cid_map,
				dma->workmode, &dma_cfg);
	}

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct i2s_private *i2s_private = srtd->dai->cpu_dai->private_data;
	struct i2s_config *config = i2s_private->config;
	struct sprd_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;
	int ret = 0;
	int used_chan_count;

	sprd_pcm_dbg("Entering %s\n", __func__);

	dma = snd_soc_dai_get_dma_data(srtd->dai->cpu_dai, substream);

	if (!dma)
		goto no_dma;

	used_chan_count = params_channels(params);
	pr_info("chan=%d totsize=%d period=%d\n", used_chan_count, totsize,
		period);

	/* this may get called several times by oss emulation
	 * with different params */
	if (rtd->params == NULL) {
		rtd->params = dma;
		ret = sprd_dma_request(dma->channels,
				sprd_pcm_dma_irq_ch, substream);
		if (ret < 0) {
			pr_err("sprd-i2s-pcm request dma error %d\n",
					dma->channels);
			goto hw_param_err;
		}
		sprd_pcm_dbg("dma id=%d\n", ret);
		rtd->uid_cid_map = ret;
		sprd_dma_set_irq_type(rtd->uid_cid_map,
				rtd->params->irq_type, ON);
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totsize;

	dma_desc = rtd->dma_desc_array;
	next_desc_phys = rtd->dma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->burst_len = I2S_FIFO_DEPTH-config->tx_watermark;
	} else {
		rtd->burst_len = config->rx_watermark;
	}
	sprd_pcm_dbg("burst lenght=%d\n", rtd->burst_len);

	do {
		next_desc_phys += sizeof(sprd_dma_desc);
		if (rtd->params->workmode == DMA_LINKLIST) {
			dma_desc->llptr = next_desc_phys;
		}

		dma_desc->cfg = dma->desc.cfg_req_mode_sel |
			dma->desc.cfg_src_data_width |
			dma->desc.cfg_dst_data_width |
			rtd->burst_len;
		dma_desc->sbm = dma->desc.src_burst_mode |
			dma->desc.src_blk_postm;
		dma_desc->dbm = dma->desc.dst_burst_mode |
			dma->desc.dst_blk_postm;
		dma_desc->tlen = period;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			dma_desc->dsrc = dma_buff_phys;
			dma_desc->ddst = dma->dev_paddr;
		} else {
			dma_desc->dsrc = dma->dev_paddr;
			dma_desc->ddst = dma_buff_phys;
		}

		dma_buff_phys += dma_desc->tlen;
		dma_desc++;

		if (period > totsize)
			period = totsize;

	} while (totsize -= period);

	if (rtd->params->workmode == DMA_LINKLIST) {
		dma_desc[-1].llptr = rtd->dma_desc_array_phys;
	}

	sprd_pcm_dma_config(substream);

	goto ok_go_out;

no_dma:
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;
hw_param_err:
ok_go_out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	struct sprd_pcm_dma_params *dma = rtd->params;

	sprd_pcm_dbg("Entering %s\n", __func__);

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (dma) {
		if (rtd->uid_cid_map >= 0) {
			sprd_dma_free(rtd->uid_cid_map);
			rtd->uid_cid_map = -1;
		}
		rtd->params = NULL;
	}

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_prepare(struct snd_pcm_substream *substream)
{
	sprd_pcm_dbg("Entering %s\n", __func__);

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (rtd->uid_cid_map >= 0) {
			sprd_dma_start(rtd->uid_cid_map);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (rtd->uid_cid_map >= 0) {
			sprd_dma_stop(rtd->uid_cid_map);
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

#ifndef dma_get_reg
static inline u32 dma_get_reg(u32 reg){
    return __raw_readl(reg);
}
#endif

static inline int sprd_pcm_dma_get_addr(int dma_ch,
					struct snd_pcm_substream *substream)
{
	int offset;
	offset = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
	    DMA_CH_SRC_ADDR : DMA_CH_DEST_ADDR;
	return dma_get_reg(DMA_CHx_BASE(dma_ch) + offset);
}

static snd_pcm_uframes_t sprd_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	snd_pcm_uframes_t x;
	int bytes_of_pointer = 0;

	bytes_of_pointer = sprd_pcm_dma_get_addr(rtd->uid_cid_map,
			substream) -
		runtime->dma_addr;

	x = bytes_to_frames(runtime, bytes_of_pointer);

	if (x == runtime->buffer_size)
		x = 0;

#if 0
	sprd_pcm_dbg("p=%d f=%d\n", bytes_of_pointer, x);
#endif

	return x;
}

static int sprd_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

#ifndef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr, runtime->dma_bytes);
#else
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       runtime->dma_addr,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
#endif
}

static struct snd_pcm_ops sprd_pcm_ops = {
	.open = sprd_pcm_open,
	.close = sprd_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = sprd_pcm_hw_params,
	.hw_free = sprd_pcm_hw_free,
	.prepare = sprd_pcm_prepare,
	.trigger = sprd_pcm_trigger,
	.pointer = sprd_pcm_pointer,
	.mmap = sprd_pcm_mmap,
};

static int sprd_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = sprd_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (!((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	      && 0 == sprd_buffer_iram_backup())) {
#endif
		buf->private_data = NULL;
		buf->area = dma_alloc_writecombine(pcm->card->dev, size,
						   &buf->addr, GFP_KERNEL);
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	} else {
		buf->private_data = buf;
		buf->area = (void *)(s_iram_remap_base);
		buf->addr = SPRD_IRAM_ALL_PHYS;
		size = SPRD_IRAM_ALL_SIZE - (2 * SPRD_I2S_DMA_NODE_SIZE);
	}
#endif
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static u64 sprd_pcm_dmamask = DMA_BIT_MASK(32);
static struct snd_dma_buffer *save_p_buf = 0;
static struct snd_dma_buffer *save_c_buf = 0;

static int sprd_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
			struct snd_pcm *pcm)
{
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_pcm_substream *substream;
	int ret = 0;

	sprd_pcm_dbg("Entering %s\n", __func__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sprd_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (substream) {
		struct snd_dma_buffer *buf = &substream->dma_buffer;
		if (!save_p_buf) {
			ret = sprd_pcm_preallocate_dma_buffer(pcm,
							      SNDRV_PCM_STREAM_PLAYBACK);
			if (ret)
				goto out;
			save_p_buf = buf;
			sprd_pcm_dbg("playback alloc memery\n");
		} else {
			memcpy(buf, save_p_buf, sizeof(*buf));
			sprd_pcm_dbg("playback share memery\n");
		}
	}

	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (substream) {
		struct snd_dma_buffer *buf = &substream->dma_buffer;
		if (!save_c_buf) {
			ret = sprd_pcm_preallocate_dma_buffer(pcm,
							      SNDRV_PCM_STREAM_CAPTURE);
			if (ret)
				goto out;
			save_c_buf = buf;
			sprd_pcm_dbg("capture alloc memery\n");
		} else {
			memcpy(buf, save_c_buf, sizeof(*buf));
			sprd_pcm_dbg("capture share memery\n");
		}
	}
out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}

static void sprd_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	sprd_pcm_dbg("Entering %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
		if (buf->private_data)
			sprd_buffer_iram_restore();
		else
#endif
			dma_free_writecombine(pcm->card->dev, buf->bytes,
					      buf->area, buf->addr);
		buf->area = NULL;
		if (buf == save_p_buf) {
			save_p_buf = 0;
		}
		if (buf == save_c_buf) {
			save_c_buf = 0;
		}
	}
	sprd_pcm_dbg("Leaving %s\n", __func__);
}

struct snd_soc_platform sprd_i2s_soc_platform = {
	.name = "sprd-i2s-pcm-dma",
	.pcm_ops = &sprd_pcm_ops,
	.pcm_new = sprd_pcm_new,
	.pcm_free = sprd_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(sprd_i2s_soc_platform);

static int __init snd_sprd_pcm_init(void)
{
	sprd_pcm_dbg("Entering %s\n", __func__);
	return snd_soc_register_platform(&sprd_i2s_soc_platform);
}

static void __exit snd_sprd_pcm_exit(void)
{
	sprd_pcm_dbg("Entering %s\n", __func__);
	snd_soc_unregister_platform(&sprd_i2s_soc_platform);
}

module_init(snd_sprd_pcm_init);
module_exit(snd_sprd_pcm_exit);

MODULE_DESCRIPTION("SPRD ASoC PCM I2S DMA");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd-i2s-audio");
