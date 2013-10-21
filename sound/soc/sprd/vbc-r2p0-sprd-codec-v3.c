/*
 * sound/soc/sprd/vbc-r2p0-sprd-codec-v3.c
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
#define pr_fmt(fmt) pr_sprd_fmt("SHARK") fmt

#include <linux/module.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/sprd-audio-hook.h>

#include <mach/sprd-audio.h>
#include "sprd-asoc-common.h"

int dig_fm_event(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *k, int event);

#define FUN_REG(f) ((unsigned short)(-(f+1)))

#define SWITCH_FUN_ON    1
#define SWITCH_FUN_OFF   0

enum {
	SHARK_FUNC_SPKL = 0,
	SHARK_FUNC_SPKR,
	SHARK_FUNC_EAR,
	SHARK_FUNC_HP,
	SHARK_FUNC_MUTE_MAX,
	SHARK_FUNC_LINE = SHARK_FUNC_MUTE_MAX,
	SHARK_FUNC_MIC,
	SHARK_FUNC_AUXMIC,
	SHARK_FUNC_HP_MIC,
	SHARK_FUNC_DMIC,
	SHARK_FUNC_DMIC1,
	SHARK_FUNC_DFM,
	SHARK_FUNC_MAX
};

struct shark_mute {
	int need_mute;
	int is_on;
	int (*mute_func) (int);
};

static struct shark_priv {
	int func[SHARK_FUNC_MAX];
	struct shark_mute m[SHARK_FUNC_MUTE_MAX];
} shark;

static const char *func_name[SHARK_FUNC_MAX] = {
	"Ext Spk",
	"Ext Spk2",
	"Ext Ear",
	"HeadPhone Jack",
	"Line Jack",
	"Mic Jack",
	"Aux Mic Jack",
	"HP Mic Jack",
	"DMic Jack",
	"DMic1 Jack",
	"Dig FM Jack",
};

static void shark_ext_control(struct snd_soc_dapm_context *dapm, int s, int e)
{
	int i;
	BUG_ON(e > SHARK_FUNC_MAX);
	for (i = s; i < e; i++) {
		if (shark.func[i] == SWITCH_FUN_ON)
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
		pr_err("ERR:Call audio internal speaker failed %d\n", enable);
}

static int audio_speaker_enable_inter(int enable)
{
	int ret;
	if (enable && shark.m[SHARK_FUNC_SPKL].need_mute) {
		enable = 0;
	}
	ret = sprd_ext_speaker_ctrl(SPRD_AUDIO_ID_SPEAKER, enable);
	if (ret < 0) {
		pr_err("ERR:Call external speaker control failed %d!\n", ret);
		return ret;
	}
	if (HOOK_BPY & ret)
		local_cpu_pa_control(enable);
	return ret;
}

static void audio_speaker_enable(int enable)
{
	shark.m[SHARK_FUNC_SPKL].is_on = enable;
	shark.m[SHARK_FUNC_SPKL].mute_func(enable);
}

static int audio_speaker2_enable_inter(int enable)
{
	int ret;
	if (enable && shark.m[SHARK_FUNC_SPKR].need_mute) {
		enable = 0;
	}
	ret = sprd_ext_speaker_ctrl(SPRD_AUDIO_ID_SPEAKER2, enable);
	if (ret < 0) {
		pr_err("ERR:Call external speaker2 control failed %d!\n", ret);
		return ret;
	}
	if (HOOK_OK != ret)
		pr_err("ERR:Speaker2 have not internal PA!\n");
	return ret;
}

static void audio_speaker2_enable(int enable)
{
	shark.m[SHARK_FUNC_SPKR].is_on = enable;
	shark.m[SHARK_FUNC_SPKR].mute_func(enable);
}

#ifdef CONFIG_SND_SOC_SPRD_AUDIO_USE_INTER_HP_PA
int sprd_inter_headphone_pa(int on);
static inline void local_cpu_hp_pa_control(bool enable)
{
	int ret = 0;
	ret = sprd_inter_headphone_pa(enable);
	if (ret < 0)
		pr_err("ERR:Call audio internal headphone control failed %d\n",
		       enable);
}
#else
static inline void local_cpu_hp_pa_control(bool enable)
{
}
#endif

static int audio_headphone_enable_inter(int enable)
{
	int ret;
	if (enable && shark.m[SHARK_FUNC_HP].need_mute) {
		enable = 0;
	}
	ret = sprd_ext_headphone_ctrl(SPRD_AUDIO_ID_HEADPHONE, enable);
	if (ret < 0) {
		pr_err("ERR:Call external headphone control failed %d!\n", ret);
		return ret;
	}
	if (HOOK_BPY & ret)
		local_cpu_hp_pa_control(enable);
	return ret;
}

static void audio_headphone_enable(int enable)
{
	shark.m[SHARK_FUNC_HP].is_on = enable;
	shark.m[SHARK_FUNC_HP].mute_func(enable);
}

static int audio_earpiece_enable_inter(int enable)
{
	int ret;
	if (enable && shark.m[SHARK_FUNC_EAR].need_mute) {
		enable = 0;
	}
	ret = sprd_ext_earpiece_ctrl(SPRD_AUDIO_ID_EARPIECE, enable);
	if (ret < 0) {
		pr_err("ERR:Call external earpiece control failed %d!\n", ret);
	}
	return ret;
}

static void audio_earpiece_enable(int enable)
{
	shark.m[SHARK_FUNC_EAR].is_on = enable;
	shark.m[SHARK_FUNC_EAR].mute_func(enable);
}

static int shark_headphone_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Headphone Switch %s\n", STR_ON_OFF(on));
	audio_headphone_enable(on);
	return 0;
}

static int shark_earpiece_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Earpiece Switch %s\n", STR_ON_OFF(on));
	audio_earpiece_enable(on);
	return 0;
}

static int shark_speaker_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Speaker Switch %s\n", STR_ON_OFF(on));
	audio_speaker_enable(on);
	return 0;
}

static int shark_speaker2_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Speaker2 Switch %s\n", STR_ON_OFF(on));
	audio_speaker2_enable(on);
	return 0;
}

static int shark_main_mic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Main MIC Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_MAIN_MIC, on);
	return 0;
}

static int shark_sub_mic_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Sub MIC Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_SUB_MIC, on);
	return 0;
}

static int shark_head_mic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Head MIC Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_HEAD_MIC, on);
	return 0;
}

static int shark_dig0_mic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Digtial0 MIC Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_DIG0_MIC, on);
	return 0;
}

static int shark_dig1_mic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Digtial1 MIC Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_DIG1_MIC, on);
	return 0;
}

static int shark_line_in_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("LINE IN Switch %s\n", STR_ON_OFF(on));
	sprd_ext_mic_ctrl(SPRD_AUDIO_ID_LINE_IN, on);
	return 0;
}

static int shark_dig_fm_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int event)
{
	int on = ! !SND_SOC_DAPM_EVENT_ON(event);
	sp_asoc_pr_dbg("Digtial FM Switch %s\n", STR_ON_OFF(on));
	sprd_ext_fm_ctrl(SPRD_AUDIO_ID_DIG_FM, on);
	return 0;
}

static const struct snd_soc_dapm_widget sprd_codec_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", shark_main_mic_event),
	SND_SOC_DAPM_MIC("Aux Mic Jack", shark_sub_mic_event),
	SND_SOC_DAPM_MIC("HP Mic Jack", shark_head_mic_event),
	SND_SOC_DAPM_MIC("DMic Jack", shark_dig0_mic_event),
	SND_SOC_DAPM_MIC("DMic1 Jack", shark_dig1_mic_event),
	/*digital fm input */
	SND_SOC_DAPM_LINE("Dig FM Jack", shark_dig_fm_event),

	SND_SOC_DAPM_SPK("Ext Spk", shark_speaker_event),
	SND_SOC_DAPM_SPK("Ext Spk2", shark_speaker2_event),
	SND_SOC_DAPM_SPK("Ext Ear", shark_earpiece_event),
	SND_SOC_DAPM_LINE("Line Jack", shark_line_in_event),
	SND_SOC_DAPM_HP("HeadPhone Jack", shark_headphone_event),
};

/* shark supported audio map */
static const struct snd_soc_dapm_route shark_audio_map[] = {
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
	{"DMIC", NULL, "DMic Jack"},
	{"DMIC1", NULL, "DMic1 Jack"},

	/* VBC -- SPRD-CODEC */
	{"Aud input", NULL, "AD Clk"},
	{"Aud1 input", NULL, "AD Clk"},
	{"DFM", NULL, "DA Clk"},

	{"Aud input", NULL, "Digital ADCL Switch"},
	{"Aud input", NULL, "Digital ADCR Switch"},

	{"Aud1 input", NULL, "Digital ADC1L Switch"},
	{"Aud1 input", NULL, "Digital ADC1R Switch"},

	{"Digital DACL Switch", NULL, "DFM"},
	{"Digital DACR Switch", NULL, "DFM"},
};

static int shark_func_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = shark.func[id];
	return 0;
}

static int shark_func_set(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	int id = FUN_REG(mc->reg);

	sp_asoc_pr_info("%s Switch %s\n", func_name[id],
			STR_ON_OFF(ucontrol->value.integer.value[0]));

	if (shark.func[id] == ucontrol->value.integer.value[0])
		return 0;

	shark.func[id] = ucontrol->value.integer.value[0];
	shark_ext_control(&card->dapm, id, id + 1);
	return 1;
}

static int shark_mute_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = shark.m[id].need_mute;
	return 0;
}

static int shark_mute_set(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	sp_asoc_pr_info("%s Switch %s\n", func_name[id],
			ucontrol->value.integer.value[0] ? "Mute" : "Unmute");

	if (shark.m[id].need_mute == ucontrol->value.integer.value[0])
		return 0;

	shark.m[id].need_mute = ucontrol->value.integer.value[0];
	shark.m[id].mute_func(shark.m[id].is_on);
	return 1;
}

#define SHARK_CODEC_FUNC(xname, xreg) \
	SOC_SINGLE_EXT(xname, FUN_REG(xreg), 0, 1, 0, shark_func_get, shark_func_set)

#define SHARK_CODEC_MUTE(xname, xreg) \
	SOC_SINGLE_EXT(xname, FUN_REG(xreg), 0, 1, 0, shark_mute_get, shark_mute_set)

static const struct snd_kcontrol_new sprd_codec_shark_controls[] = {
	SHARK_CODEC_FUNC("Speaker Function", SHARK_FUNC_SPKL),
	SHARK_CODEC_FUNC("Speaker2 Function", SHARK_FUNC_SPKR),
	SHARK_CODEC_FUNC("Earpiece Function", SHARK_FUNC_EAR),
	SHARK_CODEC_FUNC("HeadPhone Function", SHARK_FUNC_HP),
	SHARK_CODEC_FUNC("Line Function", SHARK_FUNC_LINE),
	SHARK_CODEC_FUNC("Mic Function", SHARK_FUNC_MIC),
	SHARK_CODEC_FUNC("Aux Mic Function", SHARK_FUNC_AUXMIC),
	SHARK_CODEC_FUNC("HP Mic Function", SHARK_FUNC_HP_MIC),
	SHARK_CODEC_FUNC("DMic Function", SHARK_FUNC_DMIC),
	SHARK_CODEC_FUNC("DMic1 Function", SHARK_FUNC_DMIC1),
	SHARK_CODEC_FUNC("Digital FM Function", SHARK_FUNC_DFM),

	SHARK_CODEC_MUTE("Speaker Mute", SHARK_FUNC_SPKL),
	SHARK_CODEC_MUTE("Speaker2 Mute", SHARK_FUNC_SPKR),
	SHARK_CODEC_MUTE("Earpiece Mute", SHARK_FUNC_EAR),
	SHARK_CODEC_MUTE("HeadPhone Mute", SHARK_FUNC_HP),
};

static int shark_late_probe(struct snd_soc_card *card)
{
	sprd_audio_debug_init(card->snd_card);

	shark_ext_control(&card->dapm, 0, SHARK_FUNC_MAX);
	snd_soc_dapm_ignore_suspend(&card->dapm, "Mic Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HP Mic Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Aux Mic Jack");

	snd_soc_dapm_ignore_suspend(&card->dapm, "Line Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Ear");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HeadPhone Jack");

	snd_soc_dapm_ignore_suspend(&card->dapm, "Dig FM Jack");
	return 0;
}

static struct snd_soc_dai_link shark_dai[] = {
	{
	 .name = "shark-vbc",
	 .stream_name = "vbc-dac",

	 .codec_name = "sprd-codec-v3",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "vbc-r2p0",
	 .codec_dai_name = "sprd-codec-v3-i2s",
	 },
#ifdef CONFIG_SND_SOC_SPRD_VAUDIO
	{
	 .name = "shark-dsp",
	 .stream_name = "vbc-dsp",

	 .codec_name = "sprd-codec-v3",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "vaudio",
	 .codec_dai_name = "sprd-codec-v3-i2s",
	 },
#endif
	{
	 .name = "shark-vbc-ad23",
	 .stream_name = "vbc-ext",

	 .codec_name = "sprd-codec-v3",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "vbc-r2p0-ad23",
	 .codec_dai_name = "codec-i2s-ext",
	 },
#ifdef CONFIG_SND_SOC_SPRD_VAUDIO
	{
	 .name = "shark-dsp-ad23",
	 .stream_name = "vbc-dsp-ext",

	 .codec_name = "sprd-codec-v3",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "vaudio-ad23",
	 .codec_dai_name = "codec-i2s-ext",
	 },
#endif
};

static struct snd_soc_card shark_card = {
	.name = "sprdphone",
	.dai_link = shark_dai,
	.num_links = ARRAY_SIZE(shark_dai),
	.owner = THIS_MODULE,

	.controls = sprd_codec_shark_controls,
	.num_controls = ARRAY_SIZE(sprd_codec_shark_controls),
	.dapm_widgets = sprd_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sprd_codec_dapm_widgets),
	.dapm_routes = shark_audio_map,
	.num_dapm_routes = ARRAY_SIZE(shark_audio_map),
	.late_probe = shark_late_probe,
};

static void shark_mute_init(void)
{
	shark.m[SHARK_FUNC_SPKL].mute_func = audio_speaker_enable_inter;
	shark.m[SHARK_FUNC_SPKR].mute_func = audio_speaker2_enable_inter;
	shark.m[SHARK_FUNC_HP].mute_func = audio_headphone_enable_inter;
	shark.m[SHARK_FUNC_EAR].mute_func = audio_earpiece_enable_inter;
}

static int sprd_asoc_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &shark_card;
	card->dev = &pdev->dev;
	shark_mute_init();
	return snd_soc_register_card(card);
}

static int sprd_asoc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static struct platform_driver sprd_asoc_driver = {
	.driver = {
		   .name = "vbc-r2p0-sprd-codec-v3",
		   .owner = THIS_MODULE,
		   .pm = &snd_soc_pm_ops,
		   },
	.probe = sprd_asoc_probe,
	.remove = sprd_asoc_remove,
};

module_platform_driver(sprd_asoc_driver);

MODULE_DESCRIPTION("ALSA ASoC SpreadTrum VBC(R2P0) SPRD-CODEC(V3)");
MODULE_AUTHOR("Zhenfang Wang <zhenfang.wang@spreadtrum.com>");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:vbc(r2p0)+sprd-codec(v3)");
