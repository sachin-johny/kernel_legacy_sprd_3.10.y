/* 
 * sound/soc/sc88xx/sc88xx-vbc.c
 *
 * SC88XX SoC & VBC CODEC DAI-LINK -- Link SpreadTrum sc88xx chip and VBC Dolphin codec.
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
#include <sound/initval.h>
#include <sound/soc-dapm.h>

#include "sc88xx-asoc.h"

static int sc88xx_vbc_probe(struct platform_device *pdev,
			     struct snd_soc_dai *dai)
{
    return 0;
}

static int sc88xx_vbc_suspend(struct snd_soc_dai *cpu_dai)
{
    return 0;
}

static int sc88xx_vbc_resume(struct snd_soc_dai *cpu_dai)
{
    return 0;
}

static int sc88xx_vbc_set_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
    return 0;
}

static int sc88xx_vbc_set_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
    return 0;
}

static int sc88xx_vbc_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
    return 0;
}

struct vbc_extra {
    int width;
};

static void sc88xx_vbc_dma_params(struct sprd_pcm_dma_params *dma, struct vbc_extra *extra)
{
    int autodma_burst_mod_src = SRC_BURST_MODE_4;
    int autodma_burst_mod_dst = SRC_BURST_MODE_4;
    int autodma_burst_step_src = 0;
    int autodma_burst_step_dst = 0;
    int tlen = 0;
    int pmod = 0;
    int width = extra->width;

    // VB_CR2
    // Only 16-bit word length audio data is supported in SC8800H5, either ADC or DAC
    if (dma->aaf & (AUDIO_VBDA0 | AUDIO_VBDA1)) {
        // play
        autodma_burst_mod_dst = SRC_BURST_MODE_SINGLE;
        if (width == 32) width = DMA_SDATA_WIDTH32 | DMA_DDATA_WIDTH16;
        else if (width == 16) width = DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH16;
        else width = /*DMA_SDATA_WIDTH8*/DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH16;
    } else if (dma->aaf & (AUDIO_VBAD0 | AUDIO_VBAD1)) {
        // capture
        autodma_burst_mod_src = SRC_BURST_MODE_SINGLE;
        if (width == 32) width = DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH32;
        else if (width == 16) width = DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH16;
        else width = /*DMA_SDATA_WIDTH8*/DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH16;
    } else {
        // other codec
        if (width == 32) width = DMA_SDATA_WIDTH32 | DMA_DDATA_WIDTH32;
        else if (width == 16) width = DMA_SDATA_WIDTH16 | DMA_DDATA_WIDTH16;
        else width = DMA_SDATA_WIDTH8 | DMA_DDATA_WIDTH8;
    }

    dma->name = "";
#if !SC88XX_PCM_DMA_SG_CIRCLE
    dma->cfg = DMA_LIT_ENDIAN | width | DMA_REQMODE_TRANS;
#else
    dma->cfg = DMA_LIT_ENDIAN | width | DMA_REQMODE_TRANS;
#endif
    dma->tlen = tlen;
    dma->pmod = pmod;
    dma->sbm = autodma_burst_step_src | autodma_burst_mod_src;
    dma->dbm = autodma_burst_step_dst | autodma_burst_mod_dst;
}

static int sc88xx_vbc_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
    struct snd_pcm_runtime *runtime = substream->runtime;
	struct sc88xx_runtime_data *prtd = runtime->private_data;
    struct sprd_pcm_dma_params *dma;
	int width = snd_pcm_format_physical_width(params_format(params));

	/* generate correct DMA params */
//8810 TODO	if (cpu_dai->dma_data)
//8810 TODO		kfree(cpu_dai->dma_data);
//8810 TODO    cpu_dai->dma_data = dma = kzalloc(sizeof(struct sprd_pcm_dma_params), GFP_KERNEL);

	//by johnnywang
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK || cpu_dai->playback.dma_data)
	{
		kfree(cpu_dai->playback.dma_data);
    	cpu_dai->playback.dma_data = dma = kzalloc(sizeof(struct sprd_pcm_dma_params), GFP_KERNEL);
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE|| cpu_dai->capture.dma_data)
	{
		kfree(cpu_dai->capture.dma_data);
    	cpu_dai->capture.dma_data = dma = kzalloc(sizeof(struct sprd_pcm_dma_params), GFP_KERNEL);
	}

	
    if (dma) {
        struct vbc_extra extra;
        extra.width = width;
        prtd->dma_channel = 0;
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            prtd->dma_channel |= DMA_VB_DA0_BIT;
            dma->aaf = AUDIO_VBDA0;//  | AUDIO_VBDA1;
            // if (params_channels(params) > 1) {
                prtd->dma_channel |= DMA_VB_DA1_BIT;
                dma->aaf |= AUDIO_VBDA1; // Even mono data we also let DA0 and DA1 work for stereo output
            // }
        } else {
            prtd->dma_channel |= DMA_VB_AD0_BIT;
            dma->aaf = AUDIO_VBAD0;
        }
        sc88xx_vbc_dma_params(dma, &extra);
    } else return -ENOMEM;

    return 0;
}

static int sc88xx_vbc_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

//8810 TODO    if (cpu_dai->dma_data) {
//8810 TODO		kfree(cpu_dai->dma_data);
//8810 TODO		cpu_dai->dma_data = NULL;
//8810 TODO	}

	//by johnnywang
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK || cpu_dai->playback.dma_data)
	{
		kfree(cpu_dai->playback.dma_data);
    	cpu_dai->playback.dma_data = NULL;
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE|| cpu_dai->capture.dma_data)
	{
		kfree(cpu_dai->capture.dma_data);
    	cpu_dai->capture.dma_data = NULL;
	}

    return 0;
}

static void sc88xx_vbc_shutdown(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

//    if (cpu_dai->dma_data) {
//		kfree(cpu_dai->dma_data);
//8810 TODO		cpu_dai->dma_data = NULL;
//	}

	//by johnnywang
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK || cpu_dai->playback.dma_data)
	{
		kfree(cpu_dai->playback.dma_data);
    	cpu_dai->playback.dma_data = NULL;
	}
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE|| cpu_dai->capture.dma_data)
	{
		kfree(cpu_dai->capture.dma_data);
    	cpu_dai->capture.dma_data = NULL;
	}
}

static int sc88xx_vbc_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
    return 0;
}

static struct snd_soc_dai_ops sc88xx_vbc_dai_ops = {
    .startup    = sc88xx_vbc_startup,
	.trigger	= sc88xx_vbc_trigger,
    .shutdown   = sc88xx_vbc_shutdown,
	.hw_params	= sc88xx_vbc_hw_params,
	.set_fmt	= sc88xx_vbc_set_fmt,
	.set_clkdiv	= sc88xx_vbc_set_clkdiv,
	.set_sysclk	= sc88xx_vbc_set_sysclk,
};

#define SC88XX_VBC_RATES (SNDRV_PCM_RATE_8000  |	\
			  SNDRV_PCM_RATE_11025 |	\
			  SNDRV_PCM_RATE_16000 |	\
			  SNDRV_PCM_RATE_22050 |	\
              SNDRV_PCM_RATE_32000 |    \
			  SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000 |    \
              SNDRV_PCM_RATE_96000)

struct snd_soc_dai sc88xx_vbc_dai[] = {
{
	.name = "sc88xx-vbc",
	.id = 0,
	.probe = sc88xx_vbc_probe,
	.suspend = sc88xx_vbc_suspend,
	.resume = sc88xx_vbc_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SC88XX_VBC_RATES,
		.formats = VBC_PCM_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SC88XX_VBC_RATES,
		.formats = VBC_PCM_FORMATS,},
	.ops = &sc88xx_vbc_dai_ops,
},
};
EXPORT_SYMBOL_GPL(sc88xx_vbc_dai);

static int sc88xx_vbc_init(void)
{
    return snd_soc_register_dais(sc88xx_vbc_dai, ARRAY_SIZE(sc88xx_vbc_dai));
}

static void sc88xx_vbc_exit(void)
{
    snd_soc_unregister_dais(sc88xx_vbc_dai, ARRAY_SIZE(sc88xx_vbc_dai));
}

module_init(sc88xx_vbc_init);
module_exit(sc88xx_vbc_exit);

MODULE_DESCRIPTION("SC88XX SoC & VBC CODEC DAI-LINK");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
