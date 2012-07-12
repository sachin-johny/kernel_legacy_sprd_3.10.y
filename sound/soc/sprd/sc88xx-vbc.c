/*
 * sound/soc/sprd/sc88xx-vbc.c
 *
 * SC88XX SoC & VBC CODEC DAI-LINK -- Link SpreadTrum sc88xx chip and VBC Dolphin codec.
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

static struct sc88xx_pcm_dma_params sc88xx_vbc_pcm_stereo_out = {
	.name			= "VBC PCM Stereo out",
	.channel		= DMA_VB_DA0_BIT | DMA_VB_DA1_BIT,
	.workmode		= DMA_LINKLIST,
	.irq_type		= TRANSACTION_DONE,
	.desc			= {
		.cfg_req_mode_sel = DMA_REQMODE_NORMAL,
		.cfg_src_data_width = DMA_SDATA_WIDTH16,
		.cfg_dst_data_width = DMA_DDATA_WIDTH16,
		.src_burst_mode = SRC_BURST_MODE_4,
		.dst_burst_mode = SRC_BURST_MODE_SINGLE,
	},
	.dev_paddr		= PHYS_VBDA0,
	.dev_vaddr		= VIRT_VBDA0,
	.dev_paddr1		= PHYS_VBDA1,
	.dev_vaddr1		= VIRT_VBDA1,
};

static struct sc88xx_pcm_dma_params sc88xx_vbc_pcm_stereo_in = {
	.name			= "VBC PCM Stereo in",
	.channel		= DMA_VB_AD0_BIT, /* | DMA_VB_AD1_BIT, */
	.workmode		= DMA_LINKLIST,
	.irq_type		= TRANSACTION_DONE,
	.desc			= {
		.cfg_req_mode_sel = DMA_REQMODE_NORMAL,
		.cfg_src_data_width = DMA_SDATA_WIDTH16,
		.cfg_dst_data_width = DMA_DDATA_WIDTH16,
		.src_burst_mode = SRC_BURST_MODE_SINGLE,
		.dst_burst_mode = SRC_BURST_MODE_4,
	},
	.dev_paddr		= PHYS_VBAD0,
	.dev_vaddr		= VIRT_VBAD0,
	.dev_paddr1		= PHYS_VBAD1,
	.dev_vaddr1		= VIRT_VBAD1,
};

static int sc88xx_vbc_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct sc88xx_pcm_dma_params *dma_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &sc88xx_vbc_pcm_stereo_out;
	else
		dma_data = &sc88xx_vbc_pcm_stereo_in;

	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	return 0;
}

static struct snd_soc_dai_ops sc88xx_vbc_dai_ops = {
	.hw_params	= sc88xx_vbc_hw_params,
};

#define SC88XX_VBC_RATES (SNDRV_PCM_RATE_8000 | \
				SNDRV_PCM_RATE_11025 | \
				SNDRV_PCM_RATE_16000 | \
				SNDRV_PCM_RATE_22050 | \
				SNDRV_PCM_RATE_32000 | \
				SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000 | \
				SNDRV_PCM_RATE_96000)

struct snd_soc_dai_driver sc88xx_vbc_dai = {
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SC88XX_VBC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SC88XX_VBC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &sc88xx_vbc_dai_ops,
};

static int sc88xx_vbc_drv_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &sc88xx_vbc_dai);
}

static int __devexit sc88xx_vbc_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver sc88xx_vbc_driver = {
	.probe = sc88xx_vbc_drv_probe,
	.remove = __devexit_p(sc88xx_vbc_drv_remove),

	.driver = {
		.name = "sc88xx-vbc",
		.owner = THIS_MODULE,
	},
};

static int __init sc88xx_vbc_init(void)
{
	return platform_driver_register(&sc88xx_vbc_driver);
}

static void __exit sc88xx_vbc_exit(void)
{
	platform_driver_unregister(&sc88xx_vbc_driver);
}

module_init(sc88xx_vbc_init);
module_exit(sc88xx_vbc_exit);

MODULE_DESCRIPTION("SC88XX SoC & VBC CODEC DAI-LINK");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
