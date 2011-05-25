/* 
 * sound/soc/sc88xx/sprdphone.c
 *
 * sc88xx -- SpreadTrum VBC Dolphin codec intergrated chip.
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

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
static void learly_suspend(struct early_suspend *es)
{
    // printk("audio %s\n", __func__);
    // vbc_amplifier_enable(false, "learly_suspend");
}

static void learly_resume(struct early_suspend *es)
{
    // printk("audio %s\n", __func__);
    // vbc_amplifier_enable(true, "learly_resume");
}

static void android_pm_init(void)
{
    early_suspend.suspend = learly_suspend;
    early_suspend.resume = learly_resume;
    early_suspend.level = INT_MAX;
    register_early_suspend(&early_suspend);
}

static void android_pm_exit(void)
{
    unregister_early_suspend(&early_suspend);
}
#else
static void android_pm_init(void) {}
static void android_pm_exit(void) {}
#endif

#if 1
#include <mach/pm_devices.h>
static struct sprd_pm_suspend sprd_suspend;
static int lsprd_suspend(struct device *pdev, pm_message_t state)
{
//    printk("audio %s\n", __func__);
//    vbc_amplifier_enable(false, "lsprd_suspend");
    return 0;
}

static int lsprd_resume(struct device *pdev)
{
//    printk("audio %s\n", __func__);
//    vbc_amplifier_enable(true, "lsprd_resume");
	return 0;
}

static void android_sprd_pm_init(void)
{
    sprd_suspend.suspend = lsprd_suspend;
    sprd_suspend.resume  = lsprd_resume;
    sprd_suspend.level   = INT_MAX;
    register_sprd_pm_suspend(&sprd_suspend);
}

static void android_sprd_pm_exit(void)
{
    unregister_sprd_pm_suspend(&sprd_suspend);
}
#else
static void android_sprd_pm_init(void) {}
static void android_sprd_pm_exit(void) {}
#endif

#ifdef CONFIG_PM
int sndcard_suspend(struct platform_device *pdev, pm_message_t state)
{
//    printk("audio ==> %s\n", __func__);
//    vbc_amplifier_enable(false, "sndcard_suspend");
    return 0;
}

int sndcard_resume(struct platform_device *pdev)
{
//    printk("audio ==> %s\n", __func__);
//    vbc_amplifier_enable(true, "sndcard_resume");
    return 0;
}
#else
#define sndcard_suspend NULL
#define sndcard_resume  NULL
#endif

static int sprdphone_vbc_init(struct snd_soc_codec *codec)
{
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
{ }

static int sprdphone_prepare(struct snd_pcm_substream *substream)
{
    return 0;
}

static int sprdphone_trigger(struct snd_pcm_substream *substream, int cmd)
{
    return 0;
}

static int sprdphone_hw_params(struct snd_pcm_substream *substream,
                               struct snd_pcm_hw_params *params)
{
    return 0;
}

static int sprdphone_hw_free(struct snd_pcm_substream *substream)
{
    return 0;
}

static struct snd_soc_ops sprdphone_ops = {
    .startup = sprdphone_startup,
    .prepare = sprdphone_prepare,
    .trigger = sprdphone_trigger,
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
    .name        = "sprdphone",
    .platform    = &sc88xx_soc_platform,
    .dai_link    = sprdphone_dai,
    .num_links   = ARRAY_SIZE(sprdphone_dai),
    .suspend_pre = sndcard_suspend,
    .resume_post = sndcard_resume,
};

static struct snd_soc_device sprdphone_snd_devdata = {
    .card        = &snd_soc_card_sprdphone,
    .codec_dev   = &vbc_codec,
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
    else {
        android_pm_init();
        android_sprd_pm_init();
    }

    return ret;
}

static void __exit sprdphone_exit(void)
{
    platform_device_unregister(sprdphone_snd_device);
    android_pm_exit();
    android_sprd_pm_exit();
}

module_init(sprdphone_init);
module_exit(sprdphone_exit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum SC88XX VBC sprdphone");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
