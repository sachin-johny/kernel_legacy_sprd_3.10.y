/* 
 * sound/soc/sc88xx/vbc-codec.c
 *
 * VBC -- SpreadTrum sc88xx intergrated codec.
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
#include <linux/delay.h>
#include <sound/tlv.h>

#include "sc88xx-asoc.h"

#define POWER_OFF_ON_STANDBY 0
/*
  ALSA SOC usually puts the device in standby mode when it's not used
  for sometime. If you define POWER_OFF_ON_STANDBY the driver will
  turn off the ADC/DAC when this callback is invoked and turn it back
  on when needed. Unfortunately this will result in a very light bump
  (it can be audible only with good earphones). If this bothers you
  just comment this line, you will have slightly higher power
  consumption . 
 */
static const unsigned int dac_tlv[] = {
    TLV_DB_RANGE_HEAD(1),
        0, 0xf, TLV_DB_LINEAR_ITEM(5, 450),
};

static const char *vbc_mic_sel[] = {
    "mic1",
    "mic2",
};

static const struct soc_enum vbc_mic12_enum = 
    SOC_ENUM_SINGLE(VBCR2 & 0xffff,
        MICSEL,
        2,
        vbc_mic_sel);

#define VBC_PCM_CTRL(name) \
    SOC_SINGLE(name" Playback Switch", VBCR1, DAC_MUTE, 1, 1), \
    SOC_DOUBLE_TLV(name" Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv), \
    SOC_SINGLE_TLV(name" Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv), \
    SOC_SINGLE_TLV(name" Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv)

static const struct snd_kcontrol_new vbc_snd_controls[] = {
    SOC_ENUM("Micphone12 Mux", vbc_mic12_enum),
    VBC_PCM_CTRL("PCM"),
    VBC_PCM_CTRL("Speaker"),
    VBC_PCM_CTRL("Earpiece"),
};

static const struct snd_soc_dapm_widget vbc_dapm_widgets[] = {
    
};

/* vbc supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {
    
};

static int vbc_add_widgets(struct snd_soc_codec *codec)
{
    snd_soc_dapm_new_controls(codec, vbc_dapm_widgets,
				  ARRAY_SIZE(vbc_dapm_widgets));

    snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

    // snd_soc_init_card will do this.
    // snd_soc_dapm_new_widgets(codec); 

    return 0;
}

static u32 CHIP_GetVPllClk(void)
{
	u32 ext_clk_26M;
	u32 pll_freq;
	u32 reg_value, M, N;

	ext_clk_26M = (__raw_readl(GR_GEN1) >> 15) & 0x1;

	reg_value = __raw_readl(GR_VPLL_MN);
	M = reg_value & 0x0fff;
	N = (reg_value & 0x0fff0000)>>16;

	if(ext_clk_26M)
		pll_freq = 26*N/M;
	else
		pll_freq = 13*N/M;

	return (pll_freq*1000000);
}

static inline void vbc_reg_VBAICR_set(u8 mode)
{
    vbc_reg_write(VBAICR, 0, mode, 0x0F);
}

static inline void vbc_reg_VBCR1_set(u32 type, u32 val)
{
    vbc_reg_write(VBCR1, type, val, 1);
}

static void vbc_reg_VBCR2_set(u32 type, u32 val)
{
    u32 mask;

    switch (type) {
        case ADC_ADWL:
        case DAC_ADWL:
            mask = 0x03; break;
        default: 
            mask = 1; break;
    }
    vbc_reg_write(VBCR2, type, val, mask);
}

static inline void vbc_reg_VBPMR2_set(u32 type, u32 val)
{
    vbc_reg_write(VBPMR2, type, val, 1);
}

static inline void vbc_reg_VBPMR1_set(u32 type, u32 val)
{
    vbc_reg_write(VBPMR1, type, val, 1);
}

static inline void vbc_set_AD_DA_fifo_frame_num(u8 da_fifo_fnum, u8 ad_fifo_fnum)
{
    vbc_reg_write(VBBUFFSIZE, 0, ((da_fifo_fnum-1)<<8) | ((ad_fifo_fnum-1)), 0xffff);
}

static inline void vbc_reg_VBDABUFFDTA_set(u32 type, u32 val)
{
    vbc_reg_write(VBDABUFFDTA, type, val, 1);
}

static inline void vbc_set_VBADBUFFDTA_set(u32 type, u32 val)
{
    vbc_reg_write(VBADBUFFDTA, type, val, 1);
}

static inline void vbc_access_buf(int enable)
{
    // Software access ping-pong buffer enable when VBENABE bit low
    vbc_reg_VBDABUFFDTA_set(RAMSW_EN, enable ? 1:0);
}

static void vbc_buffer_clear(int id)
{
    int i;
    vbc_reg_VBDABUFFDTA_set(RAMSW_NUMB, id ? 1:0);
    for (i = 0; i < VBC_FIFO_FRAME_NUM; i++) {
        __raw_writel(0, VBDA0);
        __raw_writel(0, VBDA1);
    }
}

static void vbc_buffer_clear_all(void)
{
    vbc_access_buf(true);
    vbc_buffer_clear(1); // clear data buffer 1
    vbc_buffer_clear(0); // clear data buffer 0
    vbc_access_buf(false);
}

/* static void vbc_codec_mute(void)
{
    vbc_reg_VBCR1_set(DAC_MUTE, 1); // mute
} */

static void vbc_codec_unmute(void)
{
    vbc_reg_VBCR1_set(DAC_MUTE, 0); // don't mute
}

static void vbc_set_ctrl2arm(void)
{
    // Enable VB DAC0/DAC1 (ARM-side)
    __raw_bits_or(ARM_VB_MCLKON|ARM_VB_ACC|ARM_VB_DA0ON|ARM_VB_ANAON|ARM_VB_DA1ON|ARM_VB_ADCON, AHB_MISC);
    msleep(2);
    __raw_bits_or(1<<17,SPRD_GREG_BASE + 0x28); // LDO_VB_PO
    msleep(10);
}

static void vbc_set_mainclk_to12M(void)
{
    u32 vpll_clk = CHIP_GetVPllClk();
    u8 vb_div = vpll_clk / 12000000;
    unsigned long flags;
    u32 tmp;

    vb_div =  1; // Through various numerical experiments, found that the value must be 1 [luther.ge]

    // Set vb_div value for the frequncy of 12MHz
    raw_local_irq_save(flags);
    tmp = __raw_readl(GR_GEN0);
    tmp &= ~(0x1F<<25);
    tmp |= ((vb_div - 1)&0x1F)<<25;
    __raw_writel(tmp, GR_GEN0);
    raw_local_irq_restore(flags);

    msleep(2); // must have a dealy
}

static inline void vbc_ready2go(void)
{
    // Enable this bit then VBC interface starts working and software 
    // can receive voice band interrupt. 
    // Better set this bit after all other register bits are programmed. 
    vbc_reg_VBDABUFFDTA_set(VBENABLE, 1);
    msleep(2);
}

static int vbc_reset(struct snd_soc_codec *codec)
{
    vbc_set_mainclk_to12M();
    vbc_set_ctrl2arm();
    vbc_ready2go();

    vbc_set_AD_DA_fifo_frame_num(VBC_FIFO_FRAME_NUM, VBC_FIFO_FRAME_NUM);

    vbc_buffer_clear_all(); // must have this func, or first play will have noise

    // IIS timeing : High(right channel) for both DA1/AD1, Low(left channel) for both DA0/AD0
    // Active level of left/right channel for both ADC and DAC channel 
    vbc_set_VBADBUFFDTA_set(VBIIS_LRCK, 0);

    vbc_reg_VBAICR_set(VBCAICR_MODE_ADC_I2S    |
                       VBCAICR_MODE_DAC_I2S    |
                       VBCAICR_MODE_ADC_SERIAL |
                       VBCAICR_MODE_DAC_SERIAL);

    vbc_reg_VBCR2_set(DAC_ADWL, DAC_DATA_WIDTH_16_bit); // DAC data sample depth 16bits
    vbc_reg_VBCR2_set(ADC_ADWL, ADC_DATA_WIDTH_16_bit); // ADC data sample depth 16bits
    vbc_reg_VBCR2_set(MICSEL, MICROPHONE1); // route microphone 1 to ADC module

    vbc_reg_write(VBCCR2, 4, VBC_RATE_8000, 0xf); // 8K sample DAC
    vbc_reg_write(VBCCR2, 0, VBC_RATE_8000, 0xf); // 8K sample ADC

    vbc_reg_write(VBCGR1, 0, 0x88, 0xff); // DAC Gain
    vbc_reg_write(VBCGR8, 0, 0x00, 0x1f);
    vbc_reg_write(VBCGR9, 0, 0x00, 0x1f);
    msleep(1);

    vbc_reg_VBPMR2_set(SB, 0); // Power on sb
    // Deleay between SB and SB_SLEEP, for stablizing the speaker output wave
    msleep(1);

    vbc_reg_VBPMR2_set(SB_SLEEP, 0); // SB quit sleep mode
    msleep(1);

    vbc_reg_VBCR1_set(SB_MICBIAS, 0); // power on mic
    vbc_reg_VBPMR2_set(GIM, 1); // 20db gain mic amplifier

    // vbc_reg_write(VBPMR1, 1, 0x02, 0x7f); // power on all units, except SB_BTL
    vbc_reg_VBPMR1_set(SB_DAC, 0); // Power on DAC
    vbc_reg_VBPMR1_set(SB_ADC, 0); // Power on ADC
    vbc_reg_VBPMR1_set(SB_MIX, 0);
    vbc_reg_VBPMR1_set(SB_LOUT, 0);
    vbc_reg_VBPMR1_set(SB_OUT, 0); // Power on DAC OUT
    msleep(5);

    // mono use DA0 left channel
    vbc_reg_VBCR1_set(MONO, 0); // stereo DAC left & right channel
    vbc_reg_VBCR1_set(HP_DIS, 1); // not route mixer audio data to headphone outputs
    vbc_reg_VBCR1_set(BYPASS, 0); // Analog bypass not route to mixer
    vbc_reg_VBCR1_set(DACSEL, 1); // route DAC to mixer
    vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
    vbc_codec_unmute(); // don't mute

    return 0;
}

static unsigned int vbc_read(struct snd_soc_codec *codec, unsigned int reg)
{
    // Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert
    reg |= ARM_VB_BASE;
    return  __raw_readl(reg);;
}

static int vbc_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
    // Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert
    reg |= ARM_VB_BASE;
    __raw_writel(val, reg);
    return 0;
}

static void vbc_dma_control(int chs, bool on)
{
    if (chs & AUDIO_VBDA0)
        vbc_reg_VBDABUFFDTA_set(VBDA0DMA_EN, on); // DMA write DAC0 data buffer enable/disable
    if (chs & AUDIO_VBDA1)
        vbc_reg_VBDABUFFDTA_set(VBDA1DMA_EN, on); // DMA write DAC1 data buffer enable/disable
    if (chs & AUDIO_VBAD0)
        vbc_reg_VBDABUFFDTA_set(VBAD0DMA_EN, on); // DMA read ADC0 data buffer enable/disable
    if (chs & AUDIO_VBAD1)
        vbc_reg_VBDABUFFDTA_set(VBAD1DMA_EN, on); // DAM read ADC1 data buffer enable/disable
}

static inline void vbc_dma_start(struct snd_pcm_substream *substream)
{
    vbc_dma_control(audio_playback_capture_channel(substream), 1);
}

static inline void vbc_dma_stop(struct snd_pcm_substream *substream)
{
    vbc_dma_control(audio_playback_capture_channel(substream), 0);
}

void flush_vbc_cache(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    // we could not stop vbc_dma_buffer immediately, because audio data still in cache
    if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
        return;
//  vbc_codec_mute();
    /* clear dma cache buffer */
    memset((void*)runtime->dma_area, 0, runtime->dma_bytes);
    if (cpu_codec_dma_chain_operate_ready(substream)) {
        vbc_dma_start(substream); // we must restart dma
        start_cpu_dma(substream);
        /* must wait all dma cache chain filled by 0 data */
//      lprintf("Filling all dma chain cache audio data to 0\n");
        msleep(100);
//      lprintf("done!\n");
        stop_cpu_dma(substream);
        vbc_dma_stop(substream);
    }
}
EXPORT_SYMBOL_GPL(flush_vbc_cache);

static int vbc_startup(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        vbc_buffer_clear_all();
//  vbc_codec_unmute();
    return 0;
}

static int vbc_prepare(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    return 0;
}

static void vbc_shutdown(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    
}

#if POWER_OFF_ON_STANDBY
static int vbc_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
    return 0;
}
#endif

static int vbc_set_dai_tristate(struct snd_soc_dai *codec_dai, int tristate)
{
    return 0;
}

static int vbc_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    return 0;
}

static int vbc_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
    return 0;
}

static int vbc_set_dai_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div)
{
    return 0;
}

static int vbc_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
            vbc_dma_start(substream);
            break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            vbc_dma_stop(substream); // Stop DMA transfer
            break;
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            vbc_dma_start(substream);
            break;
        default:
            ret = -EINVAL;
	}

	return ret;
}

static int vbc_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
    int idx;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        idx = 1;
    else idx = 0;

    switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
        case SNDRV_PCM_FORMAT_S16_BE:
        case SNDRV_PCM_FORMAT_U16_LE:
        case SNDRV_PCM_FORMAT_U16_BE: break;
        default: 
            printk(KERN_EMERG "VBC codec only supports format 16bits"); 
            break;
    }

    switch (params_rate(params)) {
        case  8000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_8000 , 0xf); break;
        case 11025: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_11025, 0xf); break;
        case 16000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_16000, 0xf); break;
        case 22050: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_22050, 0xf); break;
        case 32000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_32000, 0xf); break;
        case 44100: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_44100, 0xf); break;
        case 48000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_48000, 0xf); break;
        case 96000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_96000, 0xf); break;
        default:
            printk(KERN_EMERG "VBC codec not supports rate %d\n", params_rate(params));
            break;
    }

    // lprintf("Sample Rate is [%d]\n", params_rate(params));

    return 0;
}

static struct snd_soc_dai_ops vbc_dai_ops = {
    .startup    = vbc_startup,
    .prepare    = vbc_prepare,
    .trigger    = vbc_trigger,
	.hw_params  = vbc_pcm_hw_params,
    .shutdown   = vbc_shutdown,
	.set_clkdiv = vbc_set_dai_clkdiv,
	.set_pll    = vbc_set_dai_pll,
	.set_fmt    = vbc_set_dai_fmt,
	.set_tristate = vbc_set_dai_tristate,
};

#define VBC_PCM_RATES (SNDRV_PCM_RATE_8000  |	\
			  SNDRV_PCM_RATE_11025 |	\
			  SNDRV_PCM_RATE_16000 |	\
			  SNDRV_PCM_RATE_22050 |	\
              SNDRV_PCM_RATE_32000 |    \
			  SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000 |    \
              SNDRV_PCM_RATE_96000)

// PCM Playing and Recording default in full duplex mode
struct snd_soc_dai vbc_dai[] = {
{
    .name = "VBC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2, // now we only want to support stereo mode [luther.ge]
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1, // now we only support mono capture
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.ops = &vbc_dai_ops,
	.symmetric_rates = 1,
},
};
EXPORT_SYMBOL_GPL(vbc_dai);

static int vbc_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
    int ret = 0;

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (socdev->card->codec == NULL)
		return -ENOMEM;
	codec = socdev->card->codec;
	mutex_init(&codec->mutex);

	codec->name = "VBC";
	codec->owner = THIS_MODULE;
	codec->dai = vbc_dai;
	codec->num_dai = ARRAY_SIZE(vbc_dai);
	codec->write = vbc_write;
	codec->read = vbc_read;
#if POWER_OFF_ON_STANDBY
	codec->set_bias_level = vbc_set_bias_level;
#endif
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;

	vbc_reset(codec);
    /* vbc_reset() must be initialized twice, or the noise when playing audio */
    vbc_reset(codec);

#if POWER_OFF_ON_STANDBY
	vbc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
#endif
	snd_soc_add_controls(codec, vbc_snd_controls,
				ARRAY_SIZE(vbc_snd_controls));
	vbc_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0)
		goto card_err;
	return 0;

card_err:
	snd_soc_free_pcms(socdev);

pcm_err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int vbc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec == NULL)
		return 0;
#if POWER_OFF_ON_STANDBY
    vbc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	vbc_set_bias_level(codec, SND_SOC_BIAS_OFF);
#endif
	snd_soc_dapm_free(socdev);
	snd_soc_free_pcms(socdev);
	kfree(codec);
	return 0;
}

static int vbc_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int vbc_resume(struct platform_device *pdev)
{
    return 0;
}

struct snd_soc_codec_device vbc_codec= {
    .probe   =  vbc_probe,
    .remove  =  vbc_remove,
    .suspend =  vbc_suspend,
    .resume  =  vbc_resume,
};
EXPORT_SYMBOL_GPL(vbc_codec);

static int vbc_init(void)
{
    return snd_soc_register_dais(vbc_dai, ARRAY_SIZE(vbc_dai));
}

static void vbc_exit(void)
{
    snd_soc_unregister_dais(vbc_dai, ARRAY_SIZE(vbc_dai));
}

module_init(vbc_init);
module_exit(vbc_exit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum VBC codec");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
