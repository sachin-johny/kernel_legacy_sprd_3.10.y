/* 
 * sound/soc/sc88xx/sprdphone.c
 *
 * sc88xx -- SpreadTrum VBC codec intergrated chip.
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
#include <sound/soc-dapm.h>

#include "sc88xx-asoc.h"

static struct platform_device *sprdphone_snd_device;

static const struct snd_soc_dapm_widget sprdphone_dapm_widgets[] = {
    
};

/* sprdphone supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {
    
};

static ulong gpio_amplifier = MFP_CFG_X(LCD_RSTN, GPIO, DS0, PULL_NONE/* PULL_UP */, IO_OE);
static u32 speaker_gpio = 102; // mfp_to_gpio(MFP_CFG_TO_PIN(gpio_amplifier));
static inline void vbc_gpio_amplifier_enable(int enable)
{
    gpio_direction_output(speaker_gpio, !!enable);
}

static int sprdphone_vbc_init(struct snd_soc_codec *codec)
{
    sprd_mfp_config(&gpio_amplifier, 1);
	if (gpio_request(speaker_gpio, "speaker amplifier")) {
	    printk(KERN_ERR "speaker amplifier gpio request fail!\n");
    }
    vbc_gpio_amplifier_enable(false);

    snd_soc_dapm_new_controls(codec, sprdphone_dapm_widgets,
				  ARRAY_SIZE(sprdphone_dapm_widgets));

    snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

    // snd_soc_init_card will do this.
    // snd_soc_dapm_new_widgets(codec); 

    return 0;
}

static int sprdphone_startup(struct snd_pcm_substream *substream)
{
    return 0;
}

static void sprdphone_shutdown(struct snd_pcm_substream *substream)
{
    vbc_gpio_amplifier_enable(false);
}

static int sprdphone_prepare(struct snd_pcm_substream *substream)
{
    return 0;
}

static int sprdphone_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
    vbc_gpio_amplifier_enable(true);
    return 0;
}

static int sprdphone_hw_free(struct snd_pcm_substream *substream)
{
    flush_vbc_cache(substream);
    return 0;
}

static struct snd_soc_ops sprdphone_ops = {
	.startup = sprdphone_startup,
    .prepare = sprdphone_prepare,
	.shutdown = sprdphone_shutdown,
	.hw_params = sprdphone_hw_params,
    .hw_free = sprdphone_hw_free,
};

static struct snd_soc_dai_link sprdphone_dai[] = {
{
    .name = "sprdphone",
    .stream_name = "sprdphone",
    .codec_dai = &vbc_dai[0],
    .cpu_dai = &sc88xx_vbc_dai[0],
    .ops = &sprdphone_ops,
    .init = sprdphone_vbc_init,
},
};

static struct snd_soc_card snd_soc_card_sprdphone = {
	.name		= "sprdphone",
	.platform	= &sc88xx_soc_platform,
	.dai_link	= sprdphone_dai,
	.num_links	= ARRAY_SIZE(sprdphone_dai),
};

static struct snd_soc_device sprdphone_snd_devdata = {
	.card		= &snd_soc_card_sprdphone,
	.codec_dev	= &vbc_codec,
};

static int __init sprdphone_init(void)
{
    int ret;

    sprdphone_snd_device = platform_device_alloc("soc-audio", -1);
    if (!sprdphone_snd_device)
        return -ENOMEM;

    platform_set_drvdata(sprdphone_snd_device, &sprdphone_snd_devdata);
    sprdphone_snd_devdata.dev = &sprdphone_snd_device->dev;
    ret = platform_device_add(sprdphone_snd_device);

    if (ret)
        platform_device_put(sprdphone_snd_device);

    return ret;
}

static void __exit sprdphone_exit(void)
{
    platform_device_unregister(sprdphone_snd_device);     
}

module_init(sprdphone_init);
module_exit(sprdphone_exit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum SC88XX VBC sprdphone");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
