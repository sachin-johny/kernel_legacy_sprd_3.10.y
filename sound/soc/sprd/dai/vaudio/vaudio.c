/*
 * sound/soc/sprd/dai/vaudio/vaudio.c
 *
 * SpreadTrum Vaudio for the dsp stream.
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
#define pr_fmt(fmt) "[audio:dsp] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include "vaudio.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define sprd_vaudio_dbg pr_debug
#else
#define sprd_vaudio_dbg(...)
#endif

static int sprd_vaudio_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = dai->card;

	sprd_vaudio_dbg("Entering %s\n", __func__);

	kfree(snd_soc_dai_get_dma_data(dai, substream));
	snd_soc_dai_set_dma_data(dai, substream, NULL);

	snd_soc_dapm_force_enable_pin(&card->dapm, "DAC");
	snd_soc_dapm_force_enable_pin(&card->dapm, "ADC");

	/* cancel any delayed stream shutdown that is pending */
	if (codec_dai->pop_wait) {
		codec_dai->pop_wait = 0;
		cancel_delayed_work(&rtd->delayed_work);
	}

	snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->playback.stream_name,
			SND_SOC_DAPM_STREAM_START);
	snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->capture.stream_name,
			SND_SOC_DAPM_STREAM_START);

	snd_soc_dai_digital_mute(codec_dai, 0);

	sprd_vaudio_dbg("Leaving %s\n", __func__);
	return 0;
}

static void sprd_vaudio_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = dai->card;

	sprd_vaudio_dbg("Entering %s\n", __func__);

	snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->playback.stream_name,
			SND_SOC_DAPM_STREAM_STOP);
	snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->capture.stream_name,
			SND_SOC_DAPM_STREAM_STOP);

	snd_soc_dai_digital_mute(codec_dai, 1);

	snd_soc_dapm_disable_pin(&card->dapm, "DAC");
	snd_soc_dapm_disable_pin(&card->dapm, "ADC");

	sprd_vaudio_dbg("Leaving %s\n", __func__);
}

static int sprd_vaudio_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	sprd_vaudio_dbg("Entering %s\n", __func__);

	sprd_vaudio_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sprd_vaudio_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;
#if 0
	sprd_vaudio_dbg("Entering %s\n", __func__);
#endif

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
		break;
	}

#if 0
	sprd_vaudio_dbg("Leaving %s\n", __func__);
#endif
	return ret;
}

static struct snd_soc_dai_ops sprd_vaudio_dai_ops = {
	.startup = sprd_vaudio_startup,
	.shutdown = sprd_vaudio_shutdown,
	.hw_params = sprd_vaudio_hw_params,
	.trigger = sprd_vaudio_trigger,
};

struct snd_soc_dai_driver sprd_vaudio_dai = {
	.id = VAUDIO_MAGIC_ID,
	.playback = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_CONTINUOUS,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = SNDRV_PCM_RATE_CONTINUOUS,
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &sprd_vaudio_dai_ops,
};

static int sprd_vaudio_drv_probe(struct platform_device *pdev)
{
	int ret;

	sprd_vaudio_dbg("Entering %s\n", __func__);

	ret = snd_soc_register_dai(&pdev->dev, &sprd_vaudio_dai);

	sprd_vaudio_dbg("return %i\n", ret);
	sprd_vaudio_dbg("Leaving %s\n", __func__);

	return ret;
}

static int __devexit sprd_vaudio_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver sprd_vaudio_driver = {
	.driver = {
		   .name = "vaudio",
		   .owner = THIS_MODULE,
		   },

	.probe = sprd_vaudio_drv_probe,
	.remove = __devexit_p(sprd_vaudio_drv_remove),
};

static int __init snd_sprd_vaudio_init(void)
{
	return platform_driver_register(&sprd_vaudio_driver);
}

static void __exit snd_sprd_vaudio_exit(void)
{
	platform_driver_unregister(&sprd_vaudio_driver);
}

module_init(snd_sprd_vaudio_init);
module_exit(snd_sprd_vaudio_exit);

MODULE_DESCRIPTION("SPRD ASoC Vaudio CUP-DAI driver for the DSP");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cpu-dai:sprd-vaudio");
