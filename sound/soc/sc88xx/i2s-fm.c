/*
 * sound/soc/sprd/i2s-fm.c
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
#define pr_fmt(fmt) "[audio:i2s-fm] " fmt

#include <sound/soc.h>

#include "codec/null-codec/null-codec.h"
#include "dai/i2s/i2s.h"
#include "dai/i2s/sprd-i2s-pcm.h"

#ifdef CONFIG_SPRD_IIS_DEBUG
#define i2s_fm_dbg pr_debug
#else
#define i2s_fm_dbg(...)
#endif


static struct snd_soc_dai_link i2s_fm_dai[] = {
	{
	 .name = "digital-fm-i2s",
	 .stream_name = "i2s-rx",

	 .codec_dai = &null_codec_dai[0],
	 .cpu_dai = &sprd_i2s_dai,
	 },
};

static struct snd_soc_card i2s_fm_card = {
	.name = "digital-fm",
	.platform = &sprd_i2s_soc_platform,
	.dai_link = i2s_fm_dai,
	.num_links = ARRAY_SIZE(i2s_fm_dai),
};

static struct snd_soc_device digital_fm_snd_devdata = {
    .card        = &i2s_fm_card,
    .codec_dev   = &soc_codec_dev_null_codec,
};

static struct platform_device *i2s_fm_snd_device;

#ifndef CONFIG_FM_I2S_PORT
#define CONFIG_FM_I2S_PORT 0
#endif

static struct i2s_config fm_i2s_config =
{
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

struct i2s_private fm_i2s_priv = { 0 };

static int __init i2s_fm_modinit(void)
{
	int ret;

	i2s_fm_snd_device = platform_device_alloc("soc-audio", 1);
	if (!i2s_fm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(i2s_fm_snd_device, &digital_fm_snd_devdata);
	digital_fm_snd_devdata.dev = &i2s_fm_snd_device->dev;
	ret = platform_device_add(i2s_fm_snd_device);

	if (ret) {
		pr_err("i2s fm card register err %d\n", ret);
		platform_device_put(i2s_fm_snd_device);
	} else {
		sprd_i2s_dai.id = CONFIG_FM_I2S_PORT;
		fm_i2s_priv.config = &fm_i2s_config;
		sprd_i2s_dai.private_data = &fm_i2s_priv;
	}

	i2s_fm_dbg("Leaving %s\n", __func__);

	return ret;
}

static void __exit i2s_fm_modexit(void)
{
	platform_device_unregister(i2s_fm_snd_device);
	i2s_fm_dbg("Leaving %s\n", __func__);
}

module_init(i2s_fm_modinit);
module_exit(i2s_fm_modexit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum I2S+Digital FM i2s-fm");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("machine:i2s+digital external FM");
