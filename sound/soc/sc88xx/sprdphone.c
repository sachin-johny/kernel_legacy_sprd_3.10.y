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

#if     defined(CONFIG_ARCH_SC8800S)             || \
        defined(CONFIG_MACH_SP6810A)
#if     defined(CONFIG_ARCH_SC8800S)
static ulong gpio_amplifier = MFP_CFG_X(LCD_RSTN, GPIO, DS0, PULL_NONE/* PULL_UP */, IO_OE);
static u32 speaker_gpio = 102; // mfp_to_gpio(MFP_CFG_TO_PIN(gpio_amplifier));
#elif   defined(CONFIG_MACH_SP6810A)
static ulong gpio_amplifier = MFP_CFG_X(RFCTL6, AF3, DS2, F_PULL_DOWN, S_PULL_DOWN, IO_OE);
static u32 speaker_gpio = 96;  // GPIO_PROD_SPEAKER_PA_EN_ID
#endif
static inline void local_amplifier_init(void)
{
    sprd_mfp_config(&gpio_amplifier, 1);
    if (gpio_request(speaker_gpio, "speaker amplifier")) {
        printk(KERN_ERR "speaker amplifier gpio request fail!\n");
    }
}

static inline void local_amplifier_enable(int enable)
{
    gpio_direction_output(speaker_gpio, !!enable);
}

static inline int local_amplifier_enabled(void)
{
    if (gpio_get_value(speaker_gpio)) {
        return 1;
    } else {
        return 0;
    }
}
#elif   defined(CONFIG_MACH_SP8805GA)           || \
        defined(CONFIG_MACH_OPENPHONE)
static inline void local_amplifier_init(void)
{

}

static inline void local_amplifier_enable(int enable)
{
    if (enable) {
     // ADI_Analogdie_reg_write(ANA_PA_CTL, 0x1aa9); //classAb
        ADI_Analogdie_reg_write(ANA_PA_CTL, 0x5A5A); //classD
    } else {
        ADI_Analogdie_reg_write(ANA_PA_CTL, 0x1555);
    }
}

static inline int local_amplifier_enabled(void)
{
    u32 value = ADI_Analogdie_reg_read(ANA_PA_CTL);
    switch (value) {
        case 0x5A5A: return 1;
        default : return 0;
    }
}
#else
#error "not define this CONFIG_MACH_xxxxx"
#endif
inline void vbc_amplifier_enable(int enable, const char *prename)
{
    printk("audio %s ==> trun %s PA\n", prename, enable ? "on":"off");
    local_amplifier_enable(enable);
}
EXPORT_SYMBOL_GPL(vbc_amplifier_enable);
inline int vbc_amplifier_enabled(void)
{
    return local_amplifier_enabled();
}
EXPORT_SYMBOL_GPL(vbc_amplifier_enabled);

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
static int lsprd_suspend(struct platform_device *pdev, pm_message_t state)
{
//    printk("audio %s\n", __func__);
//    vbc_amplifier_enable(false, "lsprd_suspend");
    return 0;
}

static int lsprd_resume(struct platform_device *pdev)
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
    local_amplifier_init();
    vbc_amplifier_enable(false, "sprdphone_vbc_init");

    snd_soc_dapm_new_controls(codec, sprdphone_dapm_widgets,
                              ARRAY_SIZE(sprdphone_dapm_widgets));

    snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

    // snd_soc_init_card will do this.
    // snd_soc_dapm_new_widgets(codec); 

    return 0;
}

static int sprdphone_startup(struct snd_pcm_substream *substream)
{
//    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
//        vbc_amplifier_enable(true, "sprdphone_startup");
    return 0;
}

static void sprdphone_shutdown(struct snd_pcm_substream *substream)
{
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        vbc_amplifier_enable(false, "sprdphone_shutdown");
}

static int sprdphone_prepare(struct snd_pcm_substream *substream)
{
    return 0;
}

static int sprdphone_trigger(struct snd_pcm_substream *substream, int cmd)
{
#if 0
    switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
            // if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                vbc_amplifier_enable(true, "sprdphone_trigger");
            break;
        case SNDRV_PCM_TRIGGER_STOP:
            // if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                vbc_amplifier_enable(false, "sprdphone_trigger");
            break;
	}
#endif
    return 0;
}

static int sprdphone_hw_params(struct snd_pcm_substream *substream,
                               struct snd_pcm_hw_params *params)
{
    return 0;
}

static int sprdphone_hw_free(struct snd_pcm_substream *substream)
{
    flush_vbc_cache(substream);
//    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
//        vbc_amplifier_enable(false, "sprdphone_hw_free");
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
