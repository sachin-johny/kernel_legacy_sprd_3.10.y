/*
 * sound/soc/sprd/codec/null-codec/null-codec.c
 *
 * NULL-CODEC -- SpreadTrum just for codec code.
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
#define pr_fmt(fmt) "[audio:null-codec] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/regulator/consumer.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>

#include "null-codec.h"

#ifdef CONFIG_SPRD_IIS_DEBUG
#define null_codec_dbg pr_debug
#else
#define null_codec_dbg(...)
#endif

struct null_codec_data {
	struct snd_soc_codec codec;
};

/* PCM Playing and Recording default in full duplex mode */
struct snd_soc_dai null_codec_dai[] = {
	{
	 .name = "null-codec-dai",
	 .playback = {
		      .stream_name = "Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_32000,
		      .formats = SNDRV_PCM_FMTBIT_S16_LE,
		      },
	 .capture = {
		     .stream_name = "Capture",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_32000,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,
		     },
	 },
};
EXPORT_SYMBOL_GPL(null_codec_dai);

static struct snd_soc_codec *null_codec_codec;
static int null_codec_soc_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	null_codec_dbg("Entering %s\n", __func__);

	BUG_ON(!null_codec_codec);

	socdev->card->codec = null_codec_codec;
	codec = socdev->card->codec;

	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("failed to create pcms\n");
		return ret;
	}

	return 0;
}

static int null_codec_soc_remove(struct platform_device *pdev)
{
	null_codec_dbg("Entering %s\n", __func__);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_null_codec = {
	.probe = null_codec_soc_probe,
	.remove = null_codec_soc_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_null_codec);

static __devinit int null_codec_codec_probe(struct platform_device *pdev)
{
	struct null_codec_data *priv;
	struct snd_soc_codec *codec;
	int ret;

	null_codec_dbg("Entering %s\n", __func__);

	priv = kzalloc(sizeof(struct null_codec_data), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	codec = &priv->codec;

	null_codec_dai[0].dev = &pdev->dev;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->dev = &pdev->dev;
	codec->name = "NULL-CODEC";
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->dai = &null_codec_dai[0];
	codec->num_dai = ARRAY_SIZE(null_codec_dai);
	snd_soc_codec_set_drvdata(codec, priv);
	platform_set_drvdata(pdev, codec);

	ret = snd_soc_register_codec(codec);
	if (ret != 0)
		goto err_priv;

	null_codec_codec = codec;

	ret = snd_soc_register_dais(&null_codec_dai[0], ARRAY_SIZE(null_codec_dai));
	if (ret != 0)
		goto err_codec;
	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err_priv:
	kfree(priv);
	null_codec_codec = NULL;
	return ret;
}

static int __devexit null_codec_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	struct null_codec_data *priv = snd_soc_codec_get_drvdata(codec);

	null_codec_dbg("Entering %s\n", __func__);
	snd_soc_unregister_dais(&null_codec_dai[0], ARRAY_SIZE(null_codec_dai));
	snd_soc_unregister_codec(codec);
	kfree(priv);
	null_codec_codec = NULL;
	return 0;
}

static struct platform_driver null_codec_codec_driver = {
	.driver = {
		   .name = "null-codec-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = null_codec_codec_probe,
	.remove = __devexit_p(null_codec_codec_remove),
};

static void null_codec_device_add(void)
{
	struct platform_device *pdev;
	int ret;
	const char * name = null_codec_codec_driver.driver.name;

	pdev = platform_device_alloc(name, -1);
	if (pdev == NULL) {
		pr_err("Failed to allocate %s\n", name);
		return;
	}

	ret = platform_device_add(pdev);
	if (ret != 0) {
		pr_err("Failed to register %s: %d\n", name, ret);
		platform_device_put(pdev);
		pdev = NULL;
	}
}

static __init int null_codec_init(void)
{
	null_codec_dbg("Entering %s\n", __func__);
	null_codec_device_add();
	return platform_driver_register(&null_codec_codec_driver);
}
module_init(null_codec_init);

static __exit void null_codec_exit(void)
{
	null_codec_dbg("Entering %s\n", __func__);
	platform_driver_unregister(&null_codec_codec_driver);
}
module_exit(null_codec_exit);

MODULE_DESCRIPTION("NULL-CODEC ALSA SoC codec driver");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("codec:null-codec");
