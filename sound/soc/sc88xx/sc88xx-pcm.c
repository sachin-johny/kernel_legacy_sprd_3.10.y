/* 
 * sound/soc/sc88xx/sc88xx-pcm.c
 *
 * sc88xx SpreadTrum VBC Dolphin codec intergrated chip.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/proc_fs.h>

#include "sc88xx-asoc.h"

static const struct snd_pcm_hardware sc88xx_pcm_hardware = {
    .info           = SNDRV_PCM_INFO_MMAP           |
                      SNDRV_PCM_INFO_MMAP_VALID     |
                      SNDRV_PCM_INFO_NONINTERLEAVED |
                      SNDRV_PCM_INFO_PAUSE          |
                      SNDRV_PCM_INFO_RESUME,
// We can start playback and recording program the same time, 
// but the second program, will be forced to use a running program's sample rate, 
// this should be an alsa bug [luther.ge]
    .formats        = VBC_PCM_FORMATS,
#if !SC88XX_PCM_DMA_SG_CIRCLE
#define PCM_DMA_SG_DESC_RESERVED   (2*4)
    .period_bytes_min	= VBC_FIFO_FRAME_NUM*2*2, // 16bits, stereo-2-channels
    .period_bytes_max	= VBC_FIFO_FRAME_NUM*2*2,
#else
#define PCM_DMA_SG_DESC_RESERVED   (0)
    .period_bytes_min	= VBC_FIFO_FRAME_NUM*2*2, // 16bits, stereo-2-channels
    .period_bytes_max	= VBC_FIFO_FRAME_NUM*2*2,
#endif
    .periods_min        = 64,
    .periods_max        = /*18*/4*PAGE_SIZE/(2*sizeof(sprd_dma_desc)) - PCM_DMA_SG_DESC_RESERVED, // DA0, DA1 sg are combined
#define PCM_DUMMY_DATA_RESERVED_BYTES (0)// PAGE_SIZE)
    .buffer_bytes_max	= /*6 **/256 * 1024 - PCM_DUMMY_DATA_RESERVED_BYTES,
    .fifo_size          = VBC_FIFO_FRAME_NUM*2,
};

#define VBC_MEM_OPTIMIZATION 1
#ifdef VBC_MEM_OPTIMIZATION
#define VBC_PAGEDIR_BUF_NUM 5
static int pagedir_buf_index = 0, pagedir_buf_after = 0;
static unsigned long dma_phy_addr[VBC_PAGEDIR_BUF_NUM] = {0, 0, 0, 0 ,0};
static unsigned long dma_virtual_addr[VBC_PAGEDIR_BUF_NUM] = {0, 0, 0, 0, 0};
#endif

extern int vbc_set_sleep_mode(int on);
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
    // Because VBC only support mono capture and caputer DMA buffer size must be 160*2 bytes,
    // so we must force half size sc88xx_pcm_hardware.period_bytes_min and period_bytes_max
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
        buf_index = pagedir_buf_index;
        buf_after =  pagedir_buf_after;
        if(buf_index < VBC_PAGEDIR_BUF_NUM)
          {
            pagedir_buf_index += 1;
          }
        else
          {
            pagedir_buf_after +=1;
             if(pagedir_buf_after == VBC_PAGEDIR_BUF_NUM)
             pagedir_buf_after = 0;
          }
        local_irq_enable();

        if(buf_index < VBC_PAGEDIR_BUF_NUM){
          rtd->dma_desc_array =
                dma_alloc_writecombine(substream->pcm->card->dev, 4 * PAGE_SIZE,
                                        &rtd->dma_desc_array_phys, GFP_KERNEL);
          dma_phy_addr[buf_index] = rtd->dma_desc_array_phys;
          dma_virtual_addr[buf_index] = rtd->dma_desc_array;
          printk("vbc dma memory allocated: phy=0x%x, virt=0x%x, index=%d\n",
                    dma_phy_addr[buf_index], dma_virtual_addr[buf_index],buf_index);
        }
        else{
          rtd->dma_desc_array_phys = dma_phy_addr[buf_after];
          rtd->dma_desc_array = dma_virtual_addr[buf_after];
          printk("vbc dma memory using allocated index=%d\n",buf_after);
        }
#else
		rtd->dma_desc_array =
			dma_alloc_writecombine(substream->pcm->card->dev, 4*PAGE_SIZE,
					       &rtd->dma_desc_array_phys, GFP_KERNEL);
#endif
	if (!rtd->dma_desc_array)
		goto err1;

	runtime->private_data = rtd;
	vbc_resume_late(substream, "snd_pcm_open");
	vbc_set_sleep_mode(0);
	return 0;

err1:
	kfree(rtd);
out:
	return ret;
}

int sc88xx_pcm_close(struct snd_pcm_substream *substream)
{
#ifndef VBC_MEM_OPTIMIZATION
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd = runtime->private_data;
	dma_free_writecombine(substream->pcm->card->dev, 4*PAGE_SIZE,
			      rtd->dma_desc_array, rtd->dma_desc_array_phys);

	kfree(rtd);
#endif
	// vbc_set_sleep_mode(1);
    return 0;
}

#if !SC88XX_PCM_DMA_SG_CIRCLE
#define CHECK_PCM_DUMMY_DATA_EMPTY 0
#if CHECK_PCM_DUMMY_DATA_EMPTY
static inline int check_pcm_dummy_data_empty(unsigned char *ptr, size_t len)
{
    int count = 0;
    // unsigned char *bptr = ptr;
    while (len--) {
        if (*ptr++ != GAP_DATA_CHAR) count++;// return ptr - bptr;
    }
    return count;
}
#endif

static void grab_next_sg_data(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct sc88xx_runtime_data *rtd = runtime->private_data;
    sprd_dma_desc *dma_desc0 = NULL, *dma_desc1 = NULL;
    int chs = rtd->dma_channel;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#if !CHECK_PCM_DUMMY_DATA_EMPTY
        if (likely(!snd_pcm_playback_empty(substream))) {
            if (chs & DMA_VB_DA0_BIT) {
                if (++rtd->ch0_idx > rtd->ch_max)
                    rtd->ch0_idx = 0;
                dma_desc0 = rtd->dma_desc_array + rtd->ch0_idx;
            }
            if (chs & DMA_VB_DA1_BIT) {
                if (++rtd->ch1_idx > rtd->ch_max)
                    rtd->ch1_idx = 0;
                dma_desc1 = rtd->dma_desc_array1 + rtd->ch1_idx;
            }
        } else
#endif
        {
            if (chs & DMA_VB_DA0_BIT)
                dma_desc0 = rtd->dma_desc_array_dummy_pcm[0]; // rtd->dma_desc_array + rtd->ch0_idx;
            if (chs & DMA_VB_DA1_BIT)
                dma_desc1 = rtd->dma_desc_array_dummy_pcm[1]; // rtd->dma_desc_array + rtd->ch1_idx;
#if 1
            printk("$$$$$$$$$$$$\n"
                   "dma_len=%d\n"
                   "gap_data_width=%d\n"
                   "vbc pcm data empty --> %ld:%ld so flush 0 pcm data to hardware\n"
#if CHECK_PCM_DUMMY_DATA_EMPTY
                    "DA0=[%08x]%d\n" //[%08x][%08x]
                    "DA1=[%08x]%d\n" //[%08x][%08x]
#endif
                    , dma_desc0->tlen, rtd->gap_data_width
                    , snd_pcm_playback_avail(runtime), runtime->buffer_size
#if CHECK_PCM_DUMMY_DATA_EMPTY
                    , dma_desc0->dsrc, /*(u32)rtd->dma_cpu_dummy_pcm[0], (u32)bus_to_virt(dma_desc0->dsrc),*/ check_pcm_dummy_data_empty(rtd->dma_cpu_dummy_pcm[0], dma_desc0->tlen)
                    , dma_desc1->dsrc, /*(u32)rtd->dma_cpu_dummy_pcm[1], (u32)bus_to_virt(dma_desc1->dsrc),*/ check_pcm_dummy_data_empty(rtd->dma_cpu_dummy_pcm[1], dma_desc0->tlen)
#endif
                   );
#endif
        }

        if (dma_desc0) {
            __raw_writel(dma_desc0->tlen , DMA_VB_DA0_BASE + 0x04);
            __raw_writel(dma_desc0->dsrc , DMA_VB_DA0_BASE + 0x08);
        }
        if (dma_desc1) {
            __raw_writel(dma_desc1->tlen , DMA_VB_DA1_BASE + 0x04);
            __raw_writel(dma_desc1->dsrc , DMA_VB_DA1_BASE + 0x08);
        }
    } else {
        if (chs & DMA_VB_AD0_BIT) {
            if (++rtd->ch0_idx > rtd->ch_max)
                rtd->ch0_idx = 0;
            dma_desc0 = rtd->dma_desc_array + rtd->ch0_idx;
        }
        if (chs & DMA_VB_AD1_BIT) {
            if (++rtd->ch1_idx > rtd->ch_max)
                rtd->ch1_idx = 0;
            dma_desc1 = rtd->dma_desc_array1 + rtd->ch1_idx;
        }
        if (dma_desc0) {
            __raw_writel(dma_desc0->tlen , DMA_VB_AD0_BASE + 0x04);
            __raw_writel(dma_desc0->ddst , DMA_VB_AD0_BASE + 0x0C);
        }
        if (dma_desc1) {
            __raw_writel(dma_desc1->tlen , DMA_VB_AD1_BASE + 0x04);
            __raw_writel(dma_desc1->ddst , DMA_VB_AD1_BASE + 0x0C);
        }
    }
}
#endif

void sc88xx_pcm_dma_irq(int dma_ch, void *dev_id)
{
    struct snd_pcm_substream *substream = dev_id;
#if 0
    lprintf("dma=%d, en=0x%08x, tran=0x%08x, burst=0x%08x, link=0x%08x\n", 
            dma_ch,
            __raw_readl(DMA_CHx_EN),
            __raw_readl(DMA_TRANSF_INT_STS),
            __raw_readl(DMA_BURST_INT_STS),
            __raw_readl(DMA_LISTDONE_INT_STS));
#endif
    snd_pcm_period_elapsed(substream);
    // printk("va=%d\n", snd_pcm_playback_avail(substream->runtime));
#if !SC88XX_PCM_DMA_SG_CIRCLE
{
    struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
    // stop_cpu_dma(substream); // hardware will auto clear DMA_CHx_EN
    __raw_bits_or(rtd->dma_channel, DMA_TRANSF_INT_CLR);
    grab_next_sg_data(substream);
    start_cpu_dma(substream);
}
#endif
}

int cpu_codec_dma_chain_operate_ready(struct snd_pcm_substream *substream)
{
    struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
    u32 chs = rtd->dma_channel;
    int ch_id;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        ch_id = DMA_VB_DA0;
        if ((chs & (1 << ch_id)) && 
            !sprd_irq_handler_ready(ch_id))
            return 0;
#if !SC88XX_VBC_DMA_COMBINE
        ch_id = DMA_VB_DA1;
        if ((chs & (1 << ch_id)) && 
            !sprd_irq_handler_ready(ch_id))
            return 0;
#endif
    } else {
        ch_id = DMA_VB_AD0;
        if ((chs & (1 << ch_id)) && 
            !sprd_irq_handler_ready(ch_id))
            return 0;
#if !SC88XX_VBC_DMA_COMBINE
        ch_id = DMA_VB_AD1;
        if ((chs & (1 << ch_id)) && 
            !sprd_irq_handler_ready(ch_id))
            return 0;
#endif
    }
    return 1;
}
EXPORT_SYMBOL_GPL(cpu_codec_dma_chain_operate_ready);

void start_cpu_dma(struct snd_pcm_substream *substream)
{
    struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
    __raw_bits_or(rtd->dma_channel, DMA_CHx_EN);
}
EXPORT_SYMBOL_GPL(start_cpu_dma);

void stop_cpu_dma(struct snd_pcm_substream *substream)
{
    struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
#ifdef CONFIG_ARCH_SC8800S
    __raw_bits_and(~rtd->dma_channel, DMA_CHx_EN);
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    __raw_bits_or(rtd->dma_channel, DMA_CHx_DIS);
#endif
}
EXPORT_SYMBOL_GPL(stop_cpu_dma);

int audio_playback_capture_channel(struct snd_pcm_substream *substream)
{
    struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
    int ch_codec = 0;
    if (rtd->dma_channel & DMA_VB_DA0_BIT)
        ch_codec |= AUDIO_VBDA0;
    if (rtd->dma_channel & DMA_VB_DA1_BIT)
        ch_codec |= AUDIO_VBDA1;
    if (rtd->dma_channel & DMA_VB_AD0_BIT)
        ch_codec |= AUDIO_VBAD0;
    if (rtd->dma_channel & DMA_VB_AD1_BIT)
        ch_codec |= AUDIO_VBAD1;
    return ch_codec;
}
EXPORT_SYMBOL_GPL(audio_playback_capture_channel);

static int sc88xx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *rtd = runtime->private_data;
    struct snd_soc_pcm_runtime *srtd = substream->private_data;
    struct sprd_pcm_dma_params *dma = srtd->dai->cpu_dai->dma_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	sprd_dma_desc *dma_desc, *dma_desc1;
	dma_addr_t dma_buff_phys, dma_buff_phys1, dma_buff_phys_base, dma_buff_phys1_base, next_desc_phys, next_desc_phys1;
    unsigned char *dma_buff_cpu_base, *dma_buff_cpu1_base;
    int ret;
    int period_offset;
    int burst_size = sc88xx_pcm_hardware.fifo_size; // sc88xx_pcm_hardware.period_bytes_min / 2; // VBC_FIFO_FRAME_NUM / 2;

	if (!dma)
		return 0;

	/* when madplay plays mp3, this func will be called several times by oss emulation
	 * with different params 
     * Similarly in aplay plays multi files */
	if (1 || rtd->params == NULL) {
		rtd->params = dma;
        ret = 1;
        if (ret && rtd->dma_channel & DMA_VB_DA0_BIT)
            ret = sprd_request_dma(DMA_VB_DA0, sc88xx_pcm_dma_irq, substream);
#if !SC88XX_VBC_DMA_COMBINE
        ret = 1;
#endif
        if (ret && rtd->dma_channel & DMA_VB_DA1_BIT)
            ret = sprd_request_dma(DMA_VB_DA1, sc88xx_pcm_dma_irq, substream);
        ret = 1;
        if (ret && rtd->dma_channel & DMA_VB_AD0_BIT)
            ret = sprd_request_dma(DMA_VB_AD0, sc88xx_pcm_dma_irq, substream);
#if !SC88XX_VBC_DMA_COMBINE
        ret = 1;
#endif
        if (ret && rtd->dma_channel & DMA_VB_AD1_BIT)
            ret = sprd_request_dma(DMA_VB_AD1, sc88xx_pcm_dma_irq, substream);
        if (ret < 0) return ret;
	} else {
        lprintf("multi called\n");
        rtd->params = dma;
        // return 0;
    }

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

    period_offset = sc88xx_pcm_hardware.periods_max + PCM_DMA_SG_DESC_RESERVED / 2;
	dma_desc = rtd->dma_desc_array;
    dma_desc1= rtd->dma_desc_array1 = rtd->dma_desc_array + period_offset;
	next_desc_phys = rtd->dma_desc_array_phys;
    next_desc_phys1= rtd->dma_desc_array_phys1 = rtd->dma_desc_array_phys + period_offset * sizeof(sprd_dma_desc);

    rtd->dma_da_ad_1_offset = (totsize / params_channels(params)) * (params_channels(params)-1);

    /* channel 1 dma start addr */
	dma_buff_phys_base = dma_buff_phys = runtime->dma_addr;
    dma_buff_cpu_base = runtime->dma_area;
    /* channel 2 dma start addr */
    dma_buff_phys1_base = dma_buff_phys1= dma_buff_phys + rtd->dma_da_ad_1_offset;
    dma_buff_cpu1_base= dma_buff_cpu_base + rtd->dma_da_ad_1_offset;
    /* one channel occupied bytes in one period */
    rtd->pcm_1channel_data_width = period / params_channels(params);

#if 0
    lprintf("runtime->hw.info=0x%08x\n"
            "periods_min=%d\n"
            "periods_max=%d\n"
            "period_bytes=%d\n"
            "period_size=%d\n"
            "periods=%d\n"
            "buffer_size=%d\n"
            "buffer_bytes=%d\n"
            "pcm_1channel_data_width=%d\n"
            "channels=%d\n"
            "format=%d\n"
            "sample_bits=[%dbits]\n"
            "rate=%d\n"
            "dma_channel=0x%08x\n"
            "dma_channel_first_bit=%d\n"
            ,runtime->hw.info
            ,sc88xx_pcm_hardware.periods_min
            ,sc88xx_pcm_hardware.periods_max
            ,period
            ,params_period_size(params)
            ,params_periods(params)
            ,params_buffer_size(params)
            ,totsize
            ,rtd->pcm_1channel_data_width
            ,params_channels(params)
            ,params_format(params)
            ,snd_pcm_format_physical_width(params_format(params))
            ,params_rate(params)
            ,rtd->dma_channel
            ,__builtin_ctz(rtd->dma_channel)
            );
#endif

	do {
		next_desc_phys += sizeof(sprd_dma_desc);
        next_desc_phys1+= sizeof(sprd_dma_desc);

		dma_desc->llptr = next_desc_phys;
        dma_desc1->llptr = next_desc_phys1;

        if (rtd->dma_channel & DMA_VB_DA0_BIT) {
            dma_desc->cfg  = rtd->params->cfg | burst_size;
#if !SC88XX_PCM_DMA_SG_CIRCLE
            // dma_desc->cfg |= DMA_LLEND;
#endif
            dma_desc->tlen = rtd->pcm_1channel_data_width;
            dma_desc->dsrc = dma_buff_phys;
            dma_desc->ddst = VBDA0 - SPRD_VB_BASE + SPRD_VB_PHYS;
            dma_desc->pmod = rtd->params->pmod;
            dma_desc->sbm  = rtd->params->sbm;
            dma_desc->dbm  = rtd->params->dbm;
        }

        if (rtd->dma_channel & DMA_VB_DA1_BIT) {
            dma_desc1->cfg  = rtd->params->cfg | burst_size;
#if !SC88XX_PCM_DMA_SG_CIRCLE
            // dma_desc1->cfg |= DMA_LLEND;
#endif
            dma_desc1->tlen = rtd->pcm_1channel_data_width;
            dma_desc1->dsrc = dma_buff_phys1;
            dma_desc1->ddst = VBDA1 - SPRD_VB_BASE + SPRD_VB_PHYS;
            dma_desc1->pmod = rtd->params->pmod;
            dma_desc1->sbm  = rtd->params->sbm;
            dma_desc1->dbm  = rtd->params->dbm;
        }

        if (rtd->dma_channel & DMA_VB_AD0_BIT) {
            dma_desc->cfg  = rtd->params->cfg | burst_size;
#if !SC88XX_PCM_DMA_SG_CIRCLE
            // dma_desc->cfg |= DMA_LLEND;
#endif
            dma_desc->tlen = rtd->pcm_1channel_data_width;
            dma_desc->dsrc = VBAD0 - SPRD_VB_BASE + SPRD_VB_PHYS;
            dma_desc->ddst = dma_buff_phys;
            dma_desc->pmod = rtd->params->pmod;
            dma_desc->sbm  = rtd->params->sbm;
            dma_desc->dbm  = rtd->params->dbm;
        }

        if (rtd->dma_channel & DMA_VB_AD1_BIT) {
            dma_desc1->cfg  = rtd->params->cfg | burst_size;
#if !SC88XX_PCM_DMA_SG_CIRCLE
            // dma_desc1->cfg |= DMA_LLEND;
#endif
            dma_desc1->tlen = rtd->pcm_1channel_data_width;
            dma_desc1->dsrc = VBAD1 - SPRD_VB_BASE + SPRD_VB_PHYS;
            dma_desc1->ddst = dma_buff_phys1; 
            dma_desc1->pmod = rtd->params->pmod;
            dma_desc1->sbm  = rtd->params->sbm;
            dma_desc1->dbm  = rtd->params->dbm;
        }
#if 0
        printk(KERN_EMERG "------------[%d][0x%08x][0x%08x],sg[0x%08x][0x%08x]------------\n"
               "0cfg =0x%08x\n"
               "0tlen=0x%08x\n"
               "0dsrc=0x%08x\n"
               "0ddst=0x%08x\n"
               "0lptr=0x%08x\n"
               "0pmod=0x%08x\n"
               "0sbm =0x%08x\n"
               "0dbm =0x%08x\n"
               "\n"
               "1cfg =0x%08x\n"
               "1tlen=0x%08x\n"
               "1dsrc=0x%08x\n"
               "1ddst=0x%08x\n"
               "1lptr=0x%08x\n"
               "1pmod=0x%08x\n"
               "1sbm =0x%08x\n"
               "1dbm =0x%08x\n"
               ,dma_desc - rtd->dma_desc_array
               ,dma_buff_phys
               ,dma_buff_phys1
               ,next_desc_phys - sizeof(sprd_dma_desc)
               ,next_desc_phys1 - sizeof(sprd_dma_desc)
               ,dma_desc->cfg 
               ,dma_desc->tlen
               ,dma_desc->dsrc
               ,dma_desc->ddst
               ,dma_desc->llptr
               ,dma_desc->pmod
               ,dma_desc->sbm 
               ,dma_desc1->dbm 
               ,dma_desc1->cfg 
               ,dma_desc1->tlen
               ,dma_desc1->dsrc
               ,dma_desc1->ddst
               ,dma_desc1->llptr
               ,dma_desc1->pmod
               ,dma_desc1->sbm 
               ,dma_desc1->dbm 
        );
#endif
#if !SC88XX_PCM_DMA_SG_CIRCLE
        if (totsize == 0) {
            sprd_dma_desc *tdma_desc, *tdma_desc1;
            dma_addr_t tdma_buff_phys, tdma_buff_phys1;
            // we need a gap to skip overlapped dirty pcm data
            #if 0
            tdma_desc = dma_desc;
            tdma_desc1= dma_desc1;
            rtd->gap_data_width = 0;
            tdma_buff_phys = dma_buff_phys;
            tdma_buff_phys1= dma_buff_phys1;
            #else
            tdma_desc = dma_desc;
            tdma_desc1= dma_desc1;
            rtd->gap_data_width = 2*rtd->pcm_1channel_data_width;
            tdma_buff_phys = dma_buff_phys1 + rtd->gap_data_width;
            tdma_buff_phys1= dma_buff_phys1 + rtd->gap_data_width + rtd->pcm_1channel_data_width;
            #endif
            if (rtd->dma_channel & DMA_VB_DA0_BIT)
                tdma_desc->dsrc = tdma_buff_phys;
            if (rtd->dma_channel & DMA_VB_DA1_BIT)
                tdma_desc1->dsrc= tdma_buff_phys1;
            if (rtd->dma_channel & DMA_VB_AD0_BIT)
                tdma_desc->ddst = tdma_buff_phys;
            if (rtd->dma_channel & DMA_VB_AD1_BIT)
                tdma_desc1->ddst= tdma_buff_phys1;
            rtd->dma_desc_array_dummy_pcm[0] = tdma_desc;
            rtd->dma_desc_array_dummy_pcm[1] = tdma_desc1;
            rtd->dma_cpu_dummy_pcm[0] = dma_buff_cpu1_base + tdma_buff_phys - dma_buff_phys1_base;
            rtd->dma_cpu_dummy_pcm[1] = dma_buff_cpu1_base + tdma_buff_phys1- dma_buff_phys1_base;
            memset(rtd->dma_cpu_dummy_pcm[0], GAP_DATA_CHAR, rtd->pcm_1channel_data_width);
            memset(rtd->dma_cpu_dummy_pcm[1], GAP_DATA_CHAR, rtd->pcm_1channel_data_width);
            rtd->dma_phys_dummy_pcm[0] = tdma_buff_phys;
            rtd->dma_phys_dummy_pcm[1] = tdma_buff_phys1;
            break;
        }
#endif
		if (period > totsize)
			period = totsize;

		dma_desc++;
        dma_desc1++;
		dma_buff_phys += rtd->pcm_1channel_data_width;
        dma_buff_phys1+= rtd->pcm_1channel_data_width;
#if !SC88XX_PCM_DMA_SG_CIRCLE
        totsize -= period;
        if (!totsize && substream->stream == SNDRV_PCM_STREAM_CAPTURE) break;
	} while (1/*totsize -= period*/);
#else
    } while (totsize -= period);
#endif
	dma_desc[-1].llptr = rtd->dma_desc_array_phys;
    dma_desc1[-1].llptr = rtd->dma_desc_array_phys1;
#if !SC88XX_PCM_DMA_SG_CIRCLE
    rtd->ch_max = dma_desc - rtd->dma_desc_array - 1;
    lprintf("ch_max = %d\n", rtd->ch_max);
#else
    printk("---- vbc ch_max=%d ----\n", dma_desc - rtd->dma_desc_array - 1);
#endif
	return 0;
}

static int sc88xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;

	snd_pcm_set_runtime_buffer(substream, NULL);
    
    if (rtd->params) {
        if (rtd->dma_channel & DMA_VB_DA0_BIT)
            sprd_free_dma(DMA_VB_DA0);
        if (rtd->dma_channel & DMA_VB_DA1_BIT)
            sprd_free_dma(DMA_VB_DA1);
        if (rtd->dma_channel & DMA_VB_AD0_BIT)
            sprd_free_dma(DMA_VB_AD0);
        if (rtd->dma_channel & DMA_VB_AD1_BIT)
            sprd_free_dma(DMA_VB_AD1);
	}
	return 0;
}

int sc88xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd);
int sc88xx_pcm_prepare(struct snd_pcm_substream *substream)
{
    // From __pxa2xx_pcm_prepare clear DMA or power down
    sc88xx_pcm_trigger(substream, SNDRV_PCM_TRIGGER_STOP); // stop DMA
    return 0;
}

#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
extern inline void vbc_dma_start(struct snd_pcm_substream *substream);
#endif
int sc88xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct sc88xx_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;
    sprd_dma_ctrl ctrl;

	switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ctrl.modes = DMA_NORMAL;
#else
            // ctrl.interrupt_type = TRANS_DONE_EN;
            ctrl.modes = DMA_LINKLIST;
#endif
            ret = 1;
            if (rtd->dma_channel & DMA_VB_DA0_BIT) {
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ctrl.interrupt_type = ret ? /* LLIST_DONE_EN */ TRANS_DONE_EN : INT_NONE;
#else
            ctrl.interrupt_type = ret ? TRANS_DONE_EN : INT_NONE;
#endif
                ctrl.ch_id = DMA_VB_DA0;
                ctrl.dma_desc = rtd->dma_desc_array;
                ctrl.dma_desc_phy = rtd->dma_desc_array_phys;
                sprd_dma_setup(&ctrl);
                ret = 0;
            }
#if !SC88XX_VBC_DMA_COMBINE
            ret = 1;
#endif
            if (rtd->dma_channel & DMA_VB_DA1_BIT) {
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ctrl.interrupt_type = ret ? /* LLIST_DONE_EN */ TRANS_DONE_EN : INT_NONE;
#else
            ctrl.interrupt_type = ret ? TRANS_DONE_EN : INT_NONE;
#endif
                ctrl.ch_id = DMA_VB_DA1;
                ctrl.dma_desc = rtd->dma_desc_array1;
                ctrl.dma_desc_phy = rtd->dma_desc_array_phys1;
                sprd_dma_setup(&ctrl);
                ret = 0;
            }
            ret = 1;
            if (rtd->dma_channel & DMA_VB_AD0_BIT) {
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ctrl.interrupt_type = ret ? /* LLIST_DONE_EN */ TRANS_DONE_EN : INT_NONE;
#else
            ctrl.interrupt_type = ret ? TRANS_DONE_EN : INT_NONE;
#endif
                ctrl.ch_id = DMA_VB_AD0;
                ctrl.dma_desc = rtd->dma_desc_array;
                ctrl.dma_desc_phy = rtd->dma_desc_array_phys;
                sprd_dma_setup(&ctrl);
                ret = 0;
            }
#if !SC88XX_VBC_DMA_COMBINE
            ret = 1;
#endif
            if (rtd->dma_channel & DMA_VB_AD1_BIT) {
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ctrl.interrupt_type = ret ? /* LLIST_DONE_EN */ TRANS_DONE_EN : INT_NONE;
#else
            ctrl.interrupt_type = ret ? TRANS_DONE_EN : INT_NONE;
#endif
                ctrl.ch_id = DMA_VB_AD1;
                ctrl.dma_desc = rtd->dma_desc_array1;
                ctrl.dma_desc_phy = rtd->dma_desc_array_phys1;
                sprd_dma_setup(&ctrl);
                ret = 0;
            }
            ret = 0;
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
            // between start_cpu_dma & vbc_dma_start,
            // absolutely not permit to have any time gap,
            // otherwise noise current sound will appear.[luther.ge-2010.10.03]
            printk("vbc & cpu dma both start here!\n");
            vbc_dma_start(substream);
            // msleep(2000);
            start_cpu_dma(substream); // Start DMA transfer
#else
            start_cpu_dma(substream); // Start DMA transfer
#endif
            break;
        case SNDRV_PCM_TRIGGER_STOP:
            stop_cpu_dma(substream); // Stop DMA transfer
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
    dma_addr_t ptr = 0;
#if !SC88XX_PCM_DMA_SG_CIRCLE
    dma_addr_t ptr_top;
#endif
    snd_pcm_uframes_t x;
    int free_data_height = 0;
    u32 data_base = 0;
    int channels = runtime->channels;

    if (rtd->params) {
        u32 ch_base;
        int offset;

        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
            offset = 0x08;
        else offset = 0x0c;

        // We only support 2 channel, but i think sometimes may only use AD1 or DA1, skip AD0 and DA0
        data_base = runtime->dma_addr; // use channel 1 dma addr
        if (!(rtd->dma_channel & (DMA_VB_AD0_BIT | DMA_VB_DA0_BIT))) {
            data_base += rtd->dma_da_ad_1_offset; // skip to channle 2 dma addr
#if !SC88XX_PCM_DMA_SG_CIRCLE
            ptr_top = rtd->dma_phys_dummy_pcm[1];
        } else {
            ptr_top = rtd->dma_phys_dummy_pcm[0];
#endif
        }
        ch_base = DMA_CHx_CTL_BASE + (__builtin_ctz(rtd->dma_channel) * 0x20);

        ptr = __raw_readl(ch_base + offset); // read data pointer register

#if !SC88XX_PCM_DMA_SG_CIRCLE
        if (ptr <= ptr_top || substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
            rtd->free_data_height = free_data_height = (ptr - data_base) * channels; // Each channel data transfer is symmetrical
        } else {
            // printk("audio dma pointer over highest value, so keep the pointer\n");
            free_data_height = rtd->free_data_height;
        }
#else
        free_data_height = (ptr - data_base) * channels; // Each channel data transfer is symmetrical
#endif
    }
    // From sound/soc/s3c24xx/s3c24xx-pcm.c
	/* we seem to be getting the odd error from the pcm library due
	 * to out-of-bounds pointers. this is maybe due to the dma engine
	 * not having loaded the new values for the channel before being
	 * callled... (todo - fix )
	 */

	// if (free_data_height >= snd_pcm_lib_buffer_bytes(substream)) {
	// 	if (free_data_height == snd_pcm_lib_buffer_bytes(substream)) {
	// 		rtd->free_data_height = free_data_height = 0;
    //     }
	// }

	x = bytes_to_frames(runtime, free_data_height);

	if (x == runtime->buffer_size)
		x = 0;
#if 0
lprintf("dsrc=0x%08x, data_base=0x%08x\n", ptr, data_base);
{
    int i, j;
    static char buf[1024];
    char *p = buf;
    u8 *dat;
    for (j = 0; j < channels; j++) {
        dat = (char *)runtime->dma_area + j * rtd->dma_da_ad_1_offset;
        p = buf;
        for (i = 0; i < 32; i++)
            p += sprintf(p, "%02x ", dat[i]);
        printk("%s%d: %s\n", (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "DA":"AD", j, buf);
    }
}
lprintf("exit  ch=%d, x=%ld\n", channels, x);
#endif
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
	buf->area = dma_alloc_writecombine(pcm->card->dev, size + PCM_DUMMY_DATA_RESERVED_BYTES,
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

	if (dai->playback.channels_min) {
		ret = sc88xx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
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
		dma_free_writecombine(pcm->card->dev, buf->bytes + PCM_DUMMY_DATA_RESERVED_BYTES,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

struct snd_soc_platform sc88xx_soc_platform = {
    .name       = "sc88xx-audio-dma",
    .pcm_ops    = &sc88xx_pcm_ops,
    .pcm_new    = sc88xx_pcm_new,
    .pcm_free   = sc88xx_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(sc88xx_soc_platform);

static int sc88xx_soc_platform_init(void)
{
    return snd_soc_register_platform(&sc88xx_soc_platform);
}

static void sc88xx_soc_platform_exit(void)
{
    snd_soc_unregister_platform(&sc88xx_soc_platform);
}

module_init(sc88xx_soc_platform_init);
module_exit(sc88xx_soc_platform_exit);

MODULE_DESCRIPTION("ASoC SC88XX PCM DMA");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
