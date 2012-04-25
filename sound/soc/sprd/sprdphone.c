/*
 * sound/soc/sprd/sprdphone.c
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
#include <sound/soc.h>
#include <sound/soc-dapm.h>

static const struct snd_soc_dapm_widget sprdphone_dapm_widgets[] = {
	
};

/* sprdphone supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {
	
};

static int sprdphone_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, sprdphone_dapm_widgets,
							ARRAY_SIZE(sprdphone_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}

static struct snd_soc_dai_link sprdphone_dai[] = {
{
	.name = "sprdphone-vbc",
	.stream_name = "SPRDPHONE",
	.codec_name = "vbc-codec",
	.platform_name = "sc88xx-pcm-audio",
	.cpu_dai_name = "sc88xx-vbc",
	.codec_dai_name = "vbc-codec-embeded",
	.init = sprdphone_init,
},
};

static struct snd_soc_card sprdphone = {
	.name = "sprdphone",
	.dai_link = sprdphone_dai,
	.num_links = ARRAY_SIZE(sprdphone_dai),
	.owner = THIS_MODULE,
};

static struct platform_device *sprdphone_snd_device;

static int __init sprdphone_modinit(void)
{
	int ret;

	sprdphone_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sprdphone_snd_device)
		return -ENOMEM;

	platform_set_drvdata(sprdphone_snd_device, &sprdphone);
	ret = platform_device_add(sprdphone_snd_device);

	if (ret)
		platform_device_put(sprdphone_snd_device);

	return ret;
}

static void __exit sprdphone_modexit(void)
{
	platform_device_unregister(sprdphone_snd_device);
}

module_init(sprdphone_modinit);
module_exit(sprdphone_modexit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum VBC sprdphone");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
