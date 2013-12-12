/*
 * sound/soc/sprd/i2s-null-codec.c
 *
 * Copyright (C) 2013 SpreadTrum Ltd.
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
#include "sprd-asoc-debug.h"
#define pr_fmt(fmt) pr_sprd_fmt("I2SNC") fmt

#include <linux/module.h>
#include <sound/soc.h>
#include "sprd-asoc-common.h"
#include <mach/i2s.h>

static struct snd_soc_dai_link all_i2s_dai[] = {
	{
	 .name = "all-i2s",
	 .stream_name = "i2s",

	 .codec_name = "null-codec",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "i2s.0",
	 .codec_dai_name = "null-codec-dai",
	 },
};

static struct snd_soc_card all_i2s_card = {
	.name = "all-i2s",
	.dai_link = all_i2s_dai,
	.num_links = ARRAY_SIZE(all_i2s_dai),
	.owner = THIS_MODULE,
};

static int sprd_asoc_i2s_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &all_i2s_card;
	card->dev = &pdev->dev;
	return snd_soc_register_card(card);
}

static int sprd_asoc_i2s_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static struct platform_driver sprd_asoc_i2s_driver = {
	.driver = {
		   .name = "i2s-null-codec",
		   .owner = THIS_MODULE,
		   .pm = &snd_soc_pm_ops,
		   },
	.probe = sprd_asoc_i2s_probe,
	.remove = sprd_asoc_i2s_remove,
};

module_platform_driver(sprd_asoc_i2s_driver);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum I2S");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:i2s");
