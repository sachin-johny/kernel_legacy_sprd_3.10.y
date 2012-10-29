/*
 * sound/soc/sprd/sc88xx-pcm.c
 *
 * sc88xx SpreadTrum VBC Dolphin codec intergrated chip.
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
#include "sc88xx-asoc.h"

static const struct snd_pcm_hardware sc88xx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_NONINTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME,
/*
 * We can start playback and recording program the same time,
 * but the second program, will be forced to use a running program's sample rate,
 * this should be an alsa bug [luther.ge]
 */
	.formats = VBC_PCM_FORMATS,
	/* 16bits, stereo-2-channels */
	.period_bytes_min = VBC_FIFO_FRAME_NUM * 2 * 2,
	.period_bytes_max = VBC_FIFO_FRAME_NUM * 2 * 2,
	.periods_min = 64,
	/* DA0, DA1 sg are combined */
	.periods_max = 4 * PAGE_SIZE / (2 * sizeof(sprd_dma_desc)),
	.buffer_bytes_max = 256 * 1024,
	.fifo_size = VBC_FIFO_FRAME_NUM * 2,
};

#define VBC_MEM_OPTIMIZATION 1
#ifdef VBC_MEM_OPTIMIZATION
#define VBC_PAGEDIR_BUF_NUM 5
static int pagedir_buf_index[SNDRV_PCM_STREAM_LAST + 1] = {0, 0}, pagedir_buf_after[SNDRV_PCM_STREAM_LAST + 1] = {0, 0};
static unsigned long dma_phy_addr[SNDRV_PCM_STREAM_LAST + 1][VBC_PAGEDIR_BUF_NUM] ={{0, 0, 0, 0 ,0}, {0, 0, 0, 0 ,0}};
static unsigned long dma_virtual_addr[SNDRV_PCM_STREAM_LAST + 1][VBC_PAGEDIR_BUF_NUM] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0 ,0}};
#endif



extern int vbc_resume_late(struct snd_pcm_substream *substream, const char *prefix);

int sc88xx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd;
	int ret;
#ifdef VBC_MEM_OPTIMIZATION
	int buf_index, buf_after;
#endif

	runtime->hw = sc88xx_pcm_hardware;
	/*
	 * Because VBC only support mono capture and caputer DMA buffer size must be 160*2 bytes,
	 * so we must force half size sc88xx_pcm_hardware.period_bytes_min and period_bytes_max
	 */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		runtime->hw.period_bytes_min >>= 1;
		runtime->hw.period_bytes_max >>= 1;
	}
	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
						SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
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

#ifdef VBC_MEM_OPTIMIZATION
	local_irq_disable();
	buf_index = pagedir_buf_index[substream->stream];
	buf_after =  pagedir_buf_after[substream->stream];
	if(buf_index < VBC_PAGEDIR_BUF_NUM)
	{
		pagedir_buf_index[substream->stream]++;
	} else {
		pagedir_buf_after[substream->stream]++;
		if(pagedir_buf_after[substream->stream] == VBC_PAGEDIR_BUF_NUM)
			pagedir_buf_after[substream->stream] = 0;
	}
	local_irq_enable();

	if(buf_index < VBC_PAGEDIR_BUF_NUM){
		rtd->dma_desc_array = dma_alloc_writecombine(substream->pcm->card->dev, 4 * PAGE_SIZE,
				&rtd->dma_desc_array_phys, GFP_KERNEL);
		dma_phy_addr[substream->stream][buf_index] = rtd->dma_desc_array_phys;
		dma_virtual_addr[substream->stream][buf_index] = rtd->dma_desc_array;
		printk("vbc dma memory allocated: phy=0x%x, virt=0x%x, index=%d\n",
                    dma_phy_addr[substream->stream][buf_index], dma_virtual_addr[substream->stream][buf_index], buf_index);
    }else {
		rtd->dma_desc_array_phys = dma_phy_addr[substream->stream][buf_after];
		rtd->dma_desc_array = dma_virtual_addr[substream->stream][buf_after];
		printk("vbc dma memory using allocated index=%d\n",buf_after);
    }
#else
	rtd->dma_desc_array =
		dma_alloc_writecombine(substream->pcm->card->dev, 4 * PAGE_SIZE,
					&rtd->dma_desc_array_phys, GFP_KERNEL);
#endif
	if (!rtd->dma_desc_array)
		goto err1;
	rtd->uid_cid_map[0] = rtd->uid_cid_map[1] = -1;
	runtime->private_data = rtd;
	vbc_resume_late(substream, "snd_pcm_open");
	return 0;

err1:
	kfree(rtd);
out:
	return ret;
}

int sc88xx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd = runtime->private_data;

#ifndef VBC_MEM_OPTIMIZATION
	dma_free_writecombine(substream->pcm->card->dev, 4 * PAGE_SIZE,
				rtd->dma_desc_array, rtd->dma_desc_array_phys);
#endif
	kfree(rtd);

	return 0;
}

void sc88xx_pcm_dma_irq(int dma_ch, void *dev_id)
{
	snd_pcm_period_elapsed(dev_id);
}

u32 sc88xx_get_dma_channel(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct sc88xx_pcm_dma_params *dma_data;
	dma_data = snd_soc_dai_get_dma_data(srtd->cpu_dai, substream);
	return dma_data->channel;
}
EXPORT_SYMBOL_GPL(sc88xx_get_dma_channel);

int cpu_codec_dma_chain_operate_ready(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
	u32 chs = sc88xx_get_dma_channel(substream);
	if ((chs & (DMA_VB_DA0_BIT | DMA_VB_AD0_BIT)) && rtd->uid_cid_map[0] < 0)
		return 0;
	if ((chs & (DMA_VB_DA1_BIT | DMA_VB_AD1_BIT)) && rtd->uid_cid_map[1] < 0)
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(cpu_codec_dma_chain_operate_ready);

void start_cpu_dma(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
#if 1
	u32 chs = sc88xx_get_dma_channel(substream);
	if (chs == (DMA_VB_DA0_BIT | DMA_VB_DA1_BIT) ||
		chs == (DMA_VB_AD0_BIT | DMA_VB_AD1_BIT))
		sprd_dma_start2(rtd->uid_cid_map[0], rtd->uid_cid_map[1]);
	else sprd_dma_start(rtd->uid_cid_map[0]);
#else
	sprd_dma_channel_start(rtd->uid_cid_map[0]);
	sprd_dma_channel_start(rtd->uid_cid_map[1]);
#endif
}
EXPORT_SYMBOL_GPL(start_cpu_dma);

void stop_cpu_dma(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
#if 1
	u32 chs = sc88xx_get_dma_channel(substream);
	if (chs == (DMA_VB_DA0_BIT | DMA_VB_DA1_BIT) ||
		chs == (DMA_VB_AD0_BIT | DMA_VB_AD1_BIT))
		sprd_dma_stop2(rtd->uid_cid_map[0], rtd->uid_cid_map[1]);
	else sprd_dma_stop(rtd->uid_cid_map[0]);
#else
	sprd_dma_channel_stop(rtd->uid_cid_map[0]);
	sprd_dma_channel_stop(rtd->uid_cid_map[1]);
#endif
}
EXPORT_SYMBOL_GPL(stop_cpu_dma);

static int sc88xx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd = runtime->private_data;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct sc88xx_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	sprd_dma_desc *dma_desc, *dma_desc1;
	dma_addr_t dma_buff_phys, dma_buff_phys1, next_desc_phys, next_desc_phys1;
	int period_offset;
	u32 dma_channel;
	int burst_size = sc88xx_pcm_hardware.fifo_size;

	dma = snd_soc_dai_get_dma_data(srtd->cpu_dai, substream);

	if (!dma)
		return 0;

	dma_channel = dma->channel;
	/*
	 * when madplay plays mp3, this func will be called several times by oss emulation
	 * with different params
	 * Similarly in aplay plays multi files
	 */
	if (1 || rtd->params == NULL) {
		rtd->params = dma;
		if (dma_channel & DMA_VB_DA0_BIT) {
			rtd->uid_cid_map[0] = sprd_dma_request(DMA_VB_DA0, sc88xx_pcm_dma_irq, substream);
			sprd_dma_set_irq_type(rtd->uid_cid_map[0], rtd->params->irq_type, ON);
		}
		if (dma_channel & DMA_VB_DA1_BIT) {
			rtd->uid_cid_map[1] = sprd_dma_request(DMA_VB_DA1, sc88xx_pcm_dma_irq, substream);
			sprd_dma_set_irq_type(rtd->uid_cid_map[1], rtd->params->irq_type, ON);
		}
		if (dma_channel & DMA_VB_AD0_BIT) {
			rtd->uid_cid_map[0] = sprd_dma_request(DMA_VB_AD0, sc88xx_pcm_dma_irq, substream);
			sprd_dma_set_irq_type(rtd->uid_cid_map[0], rtd->params->irq_type, ON);
		}
		if (dma_channel & DMA_VB_AD1_BIT) {
			rtd->uid_cid_map[1] = sprd_dma_request(DMA_VB_AD1, sc88xx_pcm_dma_irq, substream);
			sprd_dma_set_irq_type(rtd->uid_cid_map[1], rtd->params->irq_type, ON);
		}
	} else {
		pr_warn("multi called\n");
		rtd->params = dma;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	period_offset = sc88xx_pcm_hardware.periods_max;
	dma_desc = rtd->dma_desc_array;
	dma_desc1= rtd->dma_desc_array1 = rtd->dma_desc_array + period_offset;
	next_desc_phys = rtd->dma_desc_array_phys;
	next_desc_phys1= rtd->dma_desc_array_phys1 = rtd->dma_desc_array_phys + period_offset * sizeof(sprd_dma_desc);

	rtd->dma_da_ad_1_offset = (totsize / params_channels(params)) * (params_channels(params) - 1);

	/* channel 1 dma start addr */
	dma_buff_phys = runtime->dma_addr;
	/* channel 2 dma start addr */
	dma_buff_phys1= dma_buff_phys + rtd->dma_da_ad_1_offset;
	/* one channel occupied bytes in one period */
	rtd->pcm_1channel_data_width = period / params_channels(params);

	do {
		next_desc_phys += sizeof(sprd_dma_desc);
		next_desc_phys1+= sizeof(sprd_dma_desc);

		if (rtd->params->workmode == DMA_LINKLIST) {
			dma_desc->llptr = next_desc_phys;
			dma_desc1->llptr = next_desc_phys1;
		}

		dma_desc->cfg = rtd->params->desc.cfg_req_mode_sel | rtd->params->desc.cfg_src_data_width | \
						rtd->params->desc.cfg_dst_data_width | burst_size;
		dma_desc->sbm = rtd->params->desc.src_burst_mode | rtd->params->desc.src_blk_postm;
		dma_desc->dbm = rtd->params->desc.dst_burst_mode | rtd->params->desc.dst_blk_postm;
		dma_desc->tlen = rtd->pcm_1channel_data_width;

		dma_desc1->cfg = dma_desc->cfg;
		dma_desc1->sbm = dma_desc->sbm;
		dma_desc1->dbm = dma_desc->dbm;
		dma_desc1->tlen = dma_desc->tlen;

		if (dma_channel & DMA_VB_DA0_BIT) {
			dma_desc->dsrc = dma_buff_phys;
			dma_desc->ddst = rtd->params->dev_paddr;
		}

		if (dma_channel & DMA_VB_DA1_BIT) {
			dma_desc1->dsrc = dma_buff_phys1;
			dma_desc1->ddst = rtd->params->dev_paddr1;
		}

		if (dma_channel & DMA_VB_AD0_BIT) {
			dma_desc->dsrc = rtd->params->dev_paddr;
			dma_desc->ddst = dma_buff_phys;
		}

		if (dma_channel & DMA_VB_AD1_BIT) {
			dma_desc1->dsrc = rtd->params->dev_paddr1;
			dma_desc1->ddst = dma_buff_phys1;
		}

		if (period > totsize)
			period = totsize;

		dma_desc++;
		dma_desc1++;
		dma_buff_phys += rtd->pcm_1channel_data_width;
		dma_buff_phys1+= rtd->pcm_1channel_data_width;
	} while (totsize -= period);

	if (rtd->params->workmode == DMA_LINKLIST) {
		dma_desc[-1].llptr = rtd->dma_desc_array_phys;
		dma_desc1[-1].llptr = rtd->dma_desc_array_phys1;
	}

	pr_debug("---- vbc ch_max=%d ----\n", dma_desc - rtd->dma_desc_array - 1);

	return 0;
}

static int sc88xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
	struct sc88xx_pcm_dma_params *dma = rtd->params;

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (dma) {
		u32 dma_channel = dma->channel;
		if ((dma_channel & (DMA_VB_DA0_BIT | DMA_VB_AD0_BIT)) &&
			rtd->uid_cid_map[0] >= 0)
			sprd_dma_free(rtd->uid_cid_map[0]);
		if ((dma_channel & (DMA_VB_DA1_BIT | DMA_VB_AD1_BIT)) &&
			rtd->uid_cid_map[1] >= 0)
			sprd_dma_free(rtd->uid_cid_map[1]);
		rtd->uid_cid_map[0] = rtd->uid_cid_map[1] = -1;
	}
	return 0;
}

int sc88xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd);
int sc88xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	sc88xx_pcm_trigger(substream, SNDRV_PCM_TRIGGER_STOP);
	return 0;
}

static void sc88xx_update_pcm_dma(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
	struct sc88xx_pcm_dma_params *dma = rtd->params;
	u32 dma_channel = dma->channel;
	int ch_id;
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_desc_phy;
	struct sprd_dma_channel_desc dma_cfg = dma->desc;

	for (; dma_channel;) {
		ch_id = -1;
		if (dma_channel & (DMA_VB_DA0_BIT | DMA_VB_AD0_BIT)) {
			ch_id = rtd->uid_cid_map[0];
			dma_desc = rtd->dma_desc_array;
			dma_desc_phy = rtd->dma_desc_array_phys;
			dma_channel &= ~(DMA_VB_DA0_BIT | DMA_VB_AD0_BIT);
			goto __do_cal;
		}
		if (dma_channel & (DMA_VB_DA1_BIT | DMA_VB_AD1_BIT)) {
			ch_id = rtd->uid_cid_map[1];
			dma_desc = rtd->dma_desc_array1;
			dma_desc_phy = rtd->dma_desc_array_phys1;
			dma_channel &= ~(DMA_VB_DA1_BIT | DMA_VB_AD1_BIT);
			goto __do_cal;
		}
		pr_err("%s : not support this channel %08x\n", __func__, dma_channel);
		dma_channel = 0;
__do_cal:
		if (ch_id >= 0) {
			dma_cfg.cfg_blk_len = dma_desc->cfg & CFG_BLK_LEN_MASK;
			dma_cfg.total_len = dma_desc->tlen;
			dma_cfg.src_addr = dma_desc->dsrc;
			dma_cfg.dst_addr = dma_desc->ddst;
			dma_cfg.llist_ptr = dma_desc_phy;
			sprd_dma_channel_config(ch_id, dma->workmode, &dma_cfg);
		}
	}
}

#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
extern void vbc_dma_start(struct snd_pcm_substream *substream);
#endif
int sc88xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			sc88xx_update_pcm_dma(substream);
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
			/*
			 * between start_cpu_dma & vbc_dma_start,
			 * absolutely not permit to have any time gap,
			 * otherwise noise current sound will appear.[luther.ge-2010.10.03]
			 */
			printk("vbc & cpu dma both start here!\n");
			vbc_dma_start(substream);
			/* msleep(2000); */
			start_cpu_dma(substream); /* Start DMA transfer */
#else
			start_cpu_dma(substream); /* Start DMA transfer */
#endif
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			stop_cpu_dma(substream); /* Stop DMA transfer */
		/* The following action will achieve in vbc_codec module */
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

snd_pcm_uframes_t
sc88xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd = runtime->private_data;
	struct sc88xx_pcm_dma_params *dma = rtd->params;
	dma_addr_t ptr = 0;

	snd_pcm_uframes_t x;
	int free_data_height = 0;
	u32 data_base = 0;
	int channels = runtime->channels;

	if (dma) {
		int ch_id;
		int offset;
		u32 dma_channel = dma->channel;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			offset = DMA_CH_SRC_ADDR;
		else offset = DMA_CH_DEST_ADDR;

		/*
		 *  We only support 2 channel,
		 *  but i think sometimes may only use AD1 or DA1, skip AD0 and DA0
		 */
		data_base = runtime->dma_addr; /* use channel 1 dma addr */
		if (likely(dma_channel & (DMA_VB_AD0_BIT | DMA_VB_DA0_BIT))) {
			ch_id = rtd->uid_cid_map[0];
		} else {
			ch_id = rtd->uid_cid_map[1];
			data_base += rtd->dma_da_ad_1_offset; /* skip to channle 2 dma addr */
		}

		ptr = dma_get_reg(DMA_CHx_BASE(ch_id) + offset); /* read data pointer register */

		free_data_height = (ptr - data_base) * channels; /* Each channel data transfer is symmetrical */
	}

	x = bytes_to_frames(runtime, free_data_height);

	if (x == runtime->buffer_size)
		x = 0;

	return x;
}

int sc88xx_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					runtime->dma_area,
					runtime->dma_addr,
					runtime->dma_bytes);
}

struct snd_pcm_ops sc88xx_pcm_ops = {
	.open		= sc88xx_pcm_open,
	.close		= sc88xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sc88xx_pcm_hw_params,
	.hw_free	= sc88xx_pcm_hw_free,
	.prepare	= sc88xx_pcm_prepare,
	.trigger	= sc88xx_pcm_trigger,
	.pointer	= sc88xx_pcm_pointer,
	.mmap		= sc88xx_pcm_mmap,
};

int sc88xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = sc88xx_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
						&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static u64 sc88xx_pcm_dmamask = DMA_BIT_MASK(32);

int sc88xx_pcm_new(struct snd_card *card, struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sc88xx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = sc88xx_pcm_preallocate_dma_buffer(pcm,
							SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = sc88xx_pcm_preallocate_dma_buffer(pcm,
							SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
out:
	return ret;
}

static void sc88xx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}

struct snd_soc_platform_driver sc88xx_soc_platform = {
	.ops		= &sc88xx_pcm_ops,
	.pcm_new	= sc88xx_pcm_new,
	.pcm_free	= sc88xx_pcm_free_dma_buffers,
};

static int __devinit sc88xx_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &sc88xx_soc_platform);
}

static int __devexit sc88xx_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver sc88xx_pcm_driver = {
	.driver = {
		.name = "sc88xx-pcm-audio",
		.owner = THIS_MODULE,
	},

	.probe = sc88xx_soc_platform_probe,
	.remove = __devexit_p(sc88xx_soc_platform_remove),
};

static int __init snd_sc88xx_pcm_init(void)
{
	return platform_driver_register(&sc88xx_pcm_driver);
}

static void __exit snd_sc88xx_pcm_exit(void)
{
	platform_driver_unregister(&sc88xx_pcm_driver);
}

module_init(snd_sc88xx_pcm_init);
module_exit(snd_sc88xx_pcm_exit);

MODULE_DESCRIPTION("ASoC SC88XX PCM DMA");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
