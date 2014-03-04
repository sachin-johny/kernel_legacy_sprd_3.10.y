/*
 * sound/soc/sprd/general-i2s.c
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
#define pr_fmt(fmt) "[audio:generali2s] " fmt

#include <sound/soc.h>
#include "dai/i2s/i2s.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define general_i2s_dbg pr_debug
#else
#define general_i2s_dbg (...)
#endif

static struct snd_soc_dai_link general_i2s_dai[] = {
	{
	 .name = "bt-i2s",
	 .stream_name = "bt-i2s",

	 .codec_name = "null-codec",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "i2s.0",
	 .codec_dai_name = "null-codec-dai",
	 },
	{
	 .name = "fm-i2s",
	 .stream_name = "fm-i2s-rx",

	 .codec_name = "null-codec",
	 .platform_name = "sprd-pcm-audio",
	 .cpu_dai_name = "i2s.0",
	 .codec_dai_name = "null-codec-dai",
	 },
};

static struct i2s_config bt_i2s_config = {
	.fs = 8000,
	.slave_timeout = 0xF11,
	.bus_type = PCM_BUS,
	.byte_per_chan = I2S_BPCH_16,
	.mode = I2S_MASTER,
	.lsb = I2S_LSB,
	.rtx_mode = I2S_RTX_MODE,
	.sync_mode = I2S_LRCK,
	.lrck_inv = I2S_L_LEFT,
	.clk_inv = I2S_CLK_N,
	.pcm_bus_mode = I2S_SHORT_FRAME,
	.pcm_slot = 0x1,
	.pcm_cycle = 1,
	.tx_watermark = 8,
	.rx_watermark = 24,
};

static struct i2s_config fm_i2s_config = {
	.fs = 32000,
	.slave_timeout = 0xF11,
	.bus_type = I2S_BUS,
	.byte_per_chan = I2S_BPCH_16,
	.mode = I2S_SLAVER,
	.lsb = I2S_MSB,
	.rtx_mode = I2S_RX_MODE,
	.sync_mode = I2S_LRCK,
	.lrck_inv = I2S_L_LEFT,
	.clk_inv = I2S_CLK_N,
	.i2s_bus_mode = I2S_MSBJUSTFIED,
	.rx_watermark = 20,
};

struct i2s_private i2s_priv[] = { {0}, {0}, };

static int general_i2s_late_probe(struct snd_soc_card *card)
{
	int i;
	general_i2s_dbg("Entering %s\n", __func__);
	i2s_priv[0].config = &bt_i2s_config;
	i2s_priv[1].config = &fm_i2s_config;
	for (i = 0; i < card->num_links; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;
		cpu_dai->ac97_pdata = &i2s_priv[i];
	}
	return 0;
}

static struct snd_soc_card general_i2s_card = {
	.name = "general-i2s",
	.dai_link = general_i2s_dai,
	.num_links = ARRAY_SIZE(general_i2s_dai),
	.owner = THIS_MODULE,
	.late_probe = general_i2s_late_probe,
};

static struct platform_device *general_i2s_snd_device;

static int __init general_i2s_modinit(void)
{
	int ret;

	general_i2s_dbg("Entering %s\n", __func__);

	general_i2s_snd_device = platform_device_alloc("soc-audio", 1);
	if (!general_i2s_snd_device)
		return -ENOMEM;

	platform_set_drvdata(general_i2s_snd_device, &general_i2s_card);
	ret = platform_device_add(general_i2s_snd_device);

	if (ret)
		platform_device_put(general_i2s_snd_device);

	return ret;
}

static void __exit general_i2s_modexit(void)
{
	platform_device_unregister(general_i2s_snd_device);
}

module_init(general_i2s_modinit);
module_exit(general_i2s_modexit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum General IIS");
MODULE_AUTHOR("Zhenfang Wang<zhenfang.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:general i2s");
