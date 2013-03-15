/*
 * sound/soc/sprd/sc8830.c
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
#define pr_fmt(fmt) "[audio:sc8830] " fmt

#include <linux/module.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <sound/audio_pa.h>
#include <mach/sprd-audio.h>

int vbc_add_controls(struct snd_soc_codec *codec);

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define sc883x_dbg pr_debug
#else
#define sc883x_dbg(...)
#endif

#define FUN_REG(f) ((unsigned short)(-(f+1)))

#define SWITCH_FUN_ON    1
#define SWITCH_FUN_OFF   0

enum {
	SC883X_FUNC_SPKL = 0,
	SC883X_FUNC_SPKR,
	SC883X_FUNC_EAR,
	SC883X_FUNC_HP,
	SC883X_FUNC_LINE,
	SC883X_FUNC_MIC,
	SC883X_FUNC_AUXMIC,
	SC883X_FUNC_HP_MIC,
#ifdef CONFIG_SPRD_CODEC_DMIC
	SC883X_FUNC_DMIC,
#endif
	SC883X_FUNC_MAX
};

static struct sc883x_priv {
	int func[SC883X_FUNC_MAX];
} sc883x;

static const char *func_name[SC883X_FUNC_MAX] = {
	"Ext Spk",
	"Ext Spk2",
	"Ext Ear",
	"HeadPhone Jack",
	"Line Jack",
	"Mic Jack",
	"Aux Mic Jack",
	"HP Mic Jack",
#ifdef CONFIG_SPRD_CODEC_DMIC
	"DMic Jack",
#endif

};

static void sc883x_ext_control(struct snd_soc_dapm_context *dapm, int s, int e)
{
	int i;
	BUG_ON(e > SC883X_FUNC_MAX);
	for (i = s; i < e; i++) {
		if (sc883x.func[i] == SWITCH_FUN_ON)
			snd_soc_dapm_enable_pin(dapm, func_name[i]);
		else
			snd_soc_dapm_disable_pin(dapm, func_name[i]);
	}

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);
}

int sprd_inter_speaker_pa(int on);
static inline void local_cpu_pa_control(bool enable)
{
	int ret = 0;
	ret = sprd_inter_speaker_pa(enable);
	if (ret < 0)
		pr_err("sc883x audio inter pa control error: %d\n", enable);
}

static void audio_speaker_enable(int enable)
{
	if (audio_pa_amplifier && audio_pa_amplifier->speaker.control)
		audio_pa_amplifier->speaker.control(enable, NULL);
	else
		local_cpu_pa_control(enable);
}

static int sc883x_hp_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	sc883x_dbg("Entering %s switch %s\n", __func__,
		   SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");
	if (audio_pa_amplifier && audio_pa_amplifier->headset.control)
		audio_pa_amplifier->headset.control(! !SND_SOC_DAPM_EVENT_ON(event), NULL);
	sc883x_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sc883x_ear_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *k, int event)
{
	sc883x_dbg("Entering %s switch %s\n", __func__,
		   SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");
	if (audio_pa_amplifier && audio_pa_amplifier->earpiece.control)
		audio_pa_amplifier->earpiece.control(! !SND_SOC_DAPM_EVENT_ON(event), NULL);
	sc883x_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sc883x_amp_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *k, int event)
{
	sc883x_dbg("Entering %s switch %s\n", __func__,
		   SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");
	audio_speaker_enable(! !SND_SOC_DAPM_EVENT_ON(event));
	sc883x_dbg("Leaving %s\n", __func__);
	return 0;
}

static const struct snd_soc_dapm_widget sprd_codec_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Aux Mic Jack", NULL),
	SND_SOC_DAPM_MIC("HP Mic Jack", NULL),
#ifdef CONFIG_SPRD_CODEC_DMIC
	SND_SOC_DAPM_MIC("DMic Jack", NULL),
#endif
	SND_SOC_DAPM_SPK("Ext Spk", sc883x_amp_event),
	SND_SOC_DAPM_SPK("Ext Spk2", sc883x_amp_event),
	SND_SOC_DAPM_SPK("Ext Ear", sc883x_ear_event),
	SND_SOC_DAPM_LINE("Line Jack", NULL),
	SND_SOC_DAPM_HP("HeadPhone Jack", sc883x_hp_event),
};

/* sc883x supported audio map */
static const struct snd_soc_dapm_route sc883x_audio_map[] = {
	{"HeadPhone Jack", NULL, "HEAD_P_L"},
	{"HeadPhone Jack", NULL, "HEAD_P_R"},
	{"Ext Spk", NULL, "AOL"},
	{"Ext Spk2", NULL, "AOR"},
	{"Ext Ear", NULL, "EAR"},
	{"MIC", NULL, "Mic Jack"},
	{"AUXMIC", NULL, "Aux Mic Jack"},
	{"HPMIC", NULL, "HP Mic Jack"},
	{"AIL", NULL, "Line Jack"},
	{"AIR", NULL, "Line Jack"},
#ifdef CONFIG_SPRD_CODEC_DMIC
	{"DMIC", NULL, "DMic Jack"},
#endif
};

static int sc883x_func_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = sc883x.func[id];
	return 0;
}

static int sc883x_func_set(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	int id = FUN_REG(mc->reg);

	pr_info("%s switch %s\n", func_name[id],
		ucontrol->value.integer.value[0] ? "ON" : "OFF");

	if (sc883x.func[id] == ucontrol->value.integer.value[0])
		return 0;

	sc883x.func[id] = ucontrol->value.integer.value[0];
	sc883x_ext_control(&card->dapm, id, id + 1);
	sc883x_dbg("Leaving %s\n", __func__);
	return 1;
}

#define SC883X_CODEC_FUNC(xname, xreg) \
	SOC_SINGLE_EXT(xname, FUN_REG(xreg), 0, 1, 0, sc883x_func_get, sc883x_func_set)

static const struct snd_kcontrol_new sprd_codec_sc883x_controls[] = {
	SC883X_CODEC_FUNC("Speaker Function", SC883X_FUNC_SPKL),
	SC883X_CODEC_FUNC("Speaker2 Function", SC883X_FUNC_SPKR),
	SC883X_CODEC_FUNC("Earpiece Function", SC883X_FUNC_EAR),
	SC883X_CODEC_FUNC("HeadPhone Function", SC883X_FUNC_HP),
	SC883X_CODEC_FUNC("Line Function", SC883X_FUNC_LINE),
	SC883X_CODEC_FUNC("Mic Function", SC883X_FUNC_MIC),
	SC883X_CODEC_FUNC("Aux Mic Function", SC883X_FUNC_AUXMIC),
	SC883X_CODEC_FUNC("HP Mic Function", SC883X_FUNC_HP_MIC),
#ifdef CONFIG_SPRD_CODEC_DMIC
	SC883X_CODEC_FUNC("DMic Function", SC883X_FUNC_DMIC),
#endif

};

static int sc883x_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = list_first_entry(&card->codec_dev_list,
						       struct snd_soc_codec,
						       card_list);
	vbc_add_controls(codec);

	sc883x_ext_control(&card->dapm, 0, SC883X_FUNC_MAX);
	snd_soc_dapm_ignore_suspend(&card->dapm, "Mic Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HP Mic Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Aux Mic Jack");

	snd_soc_dapm_ignore_suspend(&card->dapm, "Line Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Ear");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HeadPhone Jack");
	return 0;
}

static struct snd_soc_dai_link sc883x_dai[] = {
	{
	 .name = "sc883x-vbc",
	 .stream_name = "vbc-dac",

	 .codec_name = "sprd-codec",
	 .platform_name = "sprd-vbc-pcm-audio",
	 .cpu_dai_name = "vbc",
	 .codec_dai_name = "sprd-codec-i2s",
	 },
#ifdef CONFIG_SPRD_CODEC_DMIC
	{
	.name = "sc883x-vbc-ad23",
	.stream_name = "vbc-ext",

	.codec_name = "sprd-codec",
	.platform_name = "sprd-vbc-pcm-audio",
	.cpu_dai_name = "vbc-ad23",
	.codec_dai_name = "sprd-codec-i2s-1",
	},
#endif
#ifdef CONFIG_SND_SPRD_SOC_VAUDIO
	{
	 .name = "sc883x-dsp",
	 .stream_name = "vbc-dsp",

	 .codec_name = "sprd-codec",
	 .platform_name = "sprd-vbc-pcm-audio",
	 .cpu_dai_name = "vaudio",
	 .codec_dai_name = "sprd-codec-i2s",
	 },
#ifdef CONFIG_SPRD_CODEC_DMIC
	 {
	  .name = "sc883x-dsp-ad23",
	  .stream_name = "vbc-dsp-ext",

	  .codec_name = "sprd-codec",
	  .platform_name = "sprd-vbc-pcm-audio",
	  .cpu_dai_name = "vaudio-ad23",
	  .codec_dai_name = "sprd-codec-i2s-1",
	  },
#endif
#endif

};

static struct snd_soc_card sc883x_card = {
	.name = "sprdphone",
	.dai_link = sc883x_dai,
	.num_links = ARRAY_SIZE(sc883x_dai),
	.owner = THIS_MODULE,

	.controls = sprd_codec_sc883x_controls,
	.num_controls = ARRAY_SIZE(sprd_codec_sc883x_controls),
	.dapm_widgets = sprd_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sprd_codec_dapm_widgets),
	.dapm_routes = sc883x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(sc883x_audio_map),
	.late_probe = sc883x_late_probe,
};

static struct platform_device *sc883x_snd_device;

static int __init sc883x_modinit(void)
{
	int ret;

	sc883x_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sc883x_snd_device)
		return -ENOMEM;

	platform_set_drvdata(sc883x_snd_device, &sc883x_card);
	ret = platform_device_add(sc883x_snd_device);

	if (ret)
		platform_device_put(sc883x_snd_device);

	return ret;
}

static void __exit sc883x_modexit(void)
{
	platform_device_unregister(sc883x_snd_device);
}

module_init(sc883x_modinit);
module_exit(sc883x_modexit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum VBC+sprd-codec sc8830");
MODULE_AUTHOR("Zhenfang Wang <zhenfang.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:vbc+sprd-codec");
