/*
 * sound/soc/sprd/dai/i2s/i2s.c
 *
 * SPRD SoC CPU-DAI -- SpreadTrum SOC DAI i2s.
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
#define pr_fmt(fmt) "[audio:i2s] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include <mach/hardware.h>
#include <mach/regs_global.h>

#include "sprd-i2s-pcm.h"
#include "i2s.h"

#ifdef CONFIG_SPRD_IIS_DEBUG
#define i2s_dbg pr_debug
#else
#define i2s_dbg(...)
#endif

/* register offset */
#define IIS_TXD			(0x0000)
#define IIS_CLKD		(0x0004)
#define IIS_CTRL0		(0x0008)
#define IIS_CTRL1		(0x000C)
#define IIS_CTRL2		(0x0010)
#define IIS_CTRL3		(0x0014)
#define IIS_INT_IEN		(0x0018)
#define IIS_INT_CLR		(0x001C)
#define IIS_INT_RAW		(0x0020)
#define IIS_INT_STS		(0x0024)
#define IIS_STS1		(0x0028)
#define IIS_STS2		(0x002C)
#define IIS_STS3		(0x0030)
#define IIS_DSPWAIT		(0x0034)
#define IIS_CTRL4		(0x0038)
#define IIS_STS4		(0x003C)

#define I2S_REG(i2s, offset) ((unsigned int)((i2s)->membase + (offset)))
#define I2S_PHY_REG(i2s, offset) (((unsigned int)(i2s)->memphys + (offset)))

struct i2s_rtx {
	int dma_no;
};

struct i2s_priv {
	int id;
	struct device *dev;
	struct list_head list;
	struct i2s_rtx rx;
	struct i2s_rtx tx;
	atomic_t open_cnt;

	int irq_no;
	void __iomem *membase;
	unsigned int *memphys;
	struct i2s_config config;
};

static DEFINE_MUTEX(i2s_list_mutex);
static LIST_HEAD(i2s_list);

static struct sprd_pcm_dma_params i2s_pcm_stereo_out = {
	.name = "I2S PCM Stereo out",
	.workmode = DMA_LINKLIST,
	.irq_type = TRANSACTION_DONE,
	.desc = {
		 .cfg_req_mode_sel = DMA_REQMODE_NORMAL,
		 .cfg_src_data_width = DMA_SDATA_WIDTH32,
		 .cfg_dst_data_width = DMA_DDATA_WIDTH32,
		 .src_burst_mode = SRC_BURST_MODE_4,
		 .dst_burst_mode = SRC_BURST_MODE_SINGLE,
		 },
};

static struct sprd_pcm_dma_params i2s_pcm_stereo_in = {
	.name = "I2S PCM Stereo in",
	.workmode = DMA_LINKLIST,
	.irq_type = TRANSACTION_DONE,
	.desc = {
		 .cfg_req_mode_sel = DMA_REQMODE_NORMAL,
		 .cfg_src_data_width = DMA_SDATA_WIDTH32,
		 .cfg_dst_data_width = DMA_DDATA_WIDTH32,
		 .src_burst_mode = SRC_BURST_MODE_SINGLE,
		 .dst_burst_mode = SRC_BURST_MODE_4,
		 },
};

/* local register setting */
static int i2s_reg_write(unsigned int reg, int val, int mask)
{
	int tmp, ret;
	tmp = __raw_readl(reg);
	i2s_dbg("r reg 0x%x val 0x%x\n", reg, tmp);
	ret = tmp;
	tmp &= ~(mask);
	tmp |= val & mask;
	__raw_writel(tmp, reg);
	i2s_dbg("w reg 0x%x val 0x%x\n", reg, tmp);
	return ret & (mask);
}

#if 0
static inline int i2s_reg_read(unsigned int reg)
{
	int tmp;
	tmp = __raw_readl(reg);
	return tmp;
}
#endif

static int i2s_global_disable(struct i2s_priv *i2s)
{
	int bit;
	i2s_dbg("Entering %s\n", __func__);
	bit = (i2s->id == 1) ? GEN0_I2S1_EN : GEN0_I2S0_EN;
	__raw_bits_and(~bit, GR_GEN0);
	return 0;
}

static int i2s_global_enable(struct i2s_priv *i2s)
{
	int bit;
	i2s_dbg("Entering %s\n", __func__);
	bit = (i2s->id == 1) ? GEN0_I2S1_EN : GEN0_I2S0_EN;
	__raw_bits_or(bit, GR_GEN0);
	bit = (i2s->id == 1) ? BIT(30) : BIT(31);
	__raw_bits_and(~bit, GR_PCTL);
	return 0;
}

static int i2s_soft_reset(struct i2s_priv *i2s)
{
	int bit;
	i2s_dbg("Entering %s\n", __func__);
	bit = (i2s->id == 1) ? SWRST_IIS1_RST : SWRST_IIS_RST;
	__raw_bits_or(bit, GR_SOFT_RST);
	mdelay(1);
	__raw_bits_and(~bit, GR_SOFT_RST);
	return 0;
}

static void i2s_set_bus_type(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(15);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (PCM_BUS == config->bus_type) ? mask : 0, mask);
}

static void i2s_set_byte_per_channal(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 4;
	int mask = 0x3 << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	val = (config->byte_per_chan == I2S_BPCH_32) ? 2 : config->byte_per_chan;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_mode(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(3);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s mode %d\n", __func__, config->mode);
	i2s_reg_write(reg, (I2S_SLAVER == config->mode) ? mask : 0, mask);
}

static void i2s_set_lsb(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(2);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_LSB == config->lsb) ? mask : 0, mask);
}

static void i2s_set_rtx_mode(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 6;
	int mask = 0x3 << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	val = config->rtx_mode;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_sync_mode(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(9);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_SYNC == config->sync_mode) ? mask : 0, mask);
}

static void i2s_set_lrck_invert(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(10);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_L_RIGTH == config->lrck_inv) ? mask : 0, mask);
}

static void i2s_set_clk_invert(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(11);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_CLK_R == config->clk_inv) ? mask : 0, mask);
}

static void i2s_set_i2s_bus_mode(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(8);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_COMPATIBLE == config->i2s_bus_mode) ? mask : 0, mask);
}

static void i2s_set_pcm_bus_mode(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int mask = BIT(8);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s\n", __func__);
	i2s_reg_write(reg, (I2S_SHORT_FRAME == config->pcm_bus_mode) ? mask : 0, mask);
}

static void i2s_set_pcm_slot(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 0;
	int mask = 0x7 << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL2);
	i2s_dbg("Entering %s\n", __func__);
	val = config->pcm_slot;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_pcm_cycle(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 3;
	int mask = 0x7F << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL2);
	i2s_dbg("Entering %s\n", __func__);
	val = config->pcm_cycle;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_rx_watermark(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 0;
	int mask = 0x1F1F << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL3);
	i2s_dbg("Entering %s\n", __func__);
	/* full watermark */
	val = config->rx_watermark;
	/* empty watermark */
	val |= (I2S_FIFO_DEPTH - config->rx_watermark) << 8;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_tx_watermark(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 0;
	int mask = 0x1F1F << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL4);
	i2s_dbg("Entering %s\n", __func__);
	/* empty watermark */
	val = config->tx_watermark << 8;
	/* full watermark */
	val |= I2S_FIFO_DEPTH - config->tx_watermark;
	i2s_reg_write(reg, val<< shift, mask);
}

static void i2s_set_slave_timeout(struct i2s_priv *i2s)
{
	struct i2s_config *config = &i2s->config;
	int shift = 0;
	int mask = 0xFFF << shift;
	int val = 0;
	unsigned int reg = I2S_REG(i2s, IIS_CTRL1);
	i2s_dbg("Entering %s\n", __func__);
	val = config->slave_timeout;
	i2s_reg_write(reg, val<< shift, mask);
}

static int i2s_config(struct i2s_priv *i2s)
{
	int ret = 0;
	struct i2s_config *config = &i2s->config;
	i2s_dbg("Entering %s\n", __func__);

	i2s_set_bus_type(i2s);
	i2s_set_mode(i2s);
	i2s_set_byte_per_channal(i2s);
	i2s_set_rtx_mode(i2s);
	i2s_set_sync_mode(i2s);
	i2s_set_lsb(i2s);
	i2s_set_lrck_invert(i2s);
	i2s_set_clk_invert(i2s);
	i2s_set_rx_watermark(i2s);
	i2s_set_tx_watermark(i2s);
	if (I2S_SLAVER == config->mode) {
		i2s_set_slave_timeout(i2s);
	}
	if (I2S_BUS == config->bus_type) {
		i2s_set_i2s_bus_mode(i2s);
		/* FIXME temporary code start */
		if (I2S_BPCH_8 == config->byte_per_chan) {
			pr_err("the I2S can't support 8byte mode in this code\n");
			ret = -ENODEV;
		}
		/* FIXME temporary code end */
	}
	if (PCM_BUS == config->bus_type) {
		i2s_set_pcm_bus_mode(i2s);
		i2s_set_pcm_slot(i2s);
		i2s_set_pcm_cycle(i2s);
		/* FIXME temporary code start */
		pr_err("the I2S PCM mode can't support in this code\n");
		ret = -ENODEV;
		/* FIXME temporary code end */
	}
	/* FIXME temporary code start */
	if (I2S_MASTER == config->mode) {
		pr_err("the I2S MASTER mode can't support in this code\n");
		ret = -ENODEV;
	}
	/* FIXME temporary code end */

	return ret;
}

static void i2s_dma_ctrl(struct i2s_priv *i2s, int enable)
{
	int mask = BIT(14);
	unsigned int reg = I2S_REG(i2s, IIS_CTRL0);
	i2s_dbg("Entering %s enable = %d\n", __func__, enable);
	i2s_reg_write(reg, enable ? mask : 0, mask);
}

static int i2s_close(struct i2s_priv *i2s)
{
	i2s_dbg("Entering %s %d\n", __func__, atomic_read(&i2s->open_cnt));
	if (atomic_dec_and_test(&i2s->open_cnt)) {
		i2s_soft_reset(i2s);
		i2s_global_disable(i2s);
	}
	return 0;
}

static int i2s_open(struct i2s_priv *i2s)
{
	int ret = 0;

	i2s_dbg("Entering %s %d\n", __func__, atomic_read(&i2s->open_cnt));

	atomic_inc(&i2s->open_cnt);
	if (atomic_read(&i2s->open_cnt) == 1) {
		i2s_global_enable(i2s);
		i2s_soft_reset(i2s);
		i2s_dma_ctrl(i2s, 0);
		ret = i2s_config(i2s);
	}

	return ret;
}

static int i2s_startup(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	int ret;
	struct i2s_priv *i2s;
	int port = dai->id;
	struct i2s_private *i2s_private = dai->private_data;
	struct i2s_config *config = i2s_private->config;

	i2s_dbg("Entering %s port %d\n", __func__, port);
	mutex_lock(&i2s_list_mutex);
	list_for_each_entry(i2s, &i2s_list, list) {
		if (i2s->id == port)
			break;
	}
	mutex_unlock(&i2s_list_mutex);
	if (i2s->id != port) {
		pr_err("i2s port(%d) can't find the driver\n", port);
		return -ENODEV;
	}

	i2s_private->i2s = i2s;
	i2s->config = *config;

	ret = i2s_open(i2s);
	if (ret < 0) {
		pr_err("i2s open error!\n");
		return ret;
	}

	i2s_dbg("Leaving %s\n", __func__);
	return 0;
}

static void i2s_shutdown(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct i2s_private *i2s_private = dai->private_data;
	struct i2s_priv *i2s = i2s_private->i2s;

	i2s_dbg("Entering %s\n", __func__);

	i2s_close(i2s);

	i2s_dbg("Leaving %s\n", __func__);
}

static int i2s_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct sprd_pcm_dma_params *dma_data;
	struct i2s_private *i2s_private = dai->private_data;
	struct i2s_priv *i2s = i2s_private->i2s;

	i2s_dbg("Entering %s port %d\n", __func__, i2s->id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dma_data = &i2s_pcm_stereo_out;
		dma_data->channels = i2s->tx.dma_no;
	} else {
		dma_data = &i2s_pcm_stereo_in;
		dma_data->channels = i2s->rx.dma_no;
	}

	dma_data->dev_paddr = I2S_PHY_REG(i2s, IIS_TXD);

	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	default:
		pr_err("i2s only supports format SNDRV_PCM_FORMAT_S16_LE now!\n");
		break;
	}

	if (params_channels(params) > 2) {
		pr_err("i2s can not supports grate 2 channels\n");
	}

	i2s_dbg("Leaving %s\n", __func__);
	return 0;
}

static int i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai)
{
	struct i2s_private *i2s_private = dai->private_data;
	struct i2s_priv *i2s = i2s_private->i2s;
	int ret = 0;

#if 0
	i2s_dbg("Entering %s\n", __func__);
#endif

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		i2s_dma_ctrl(i2s, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		i2s_dma_ctrl(i2s, 0);
		break;
	default:
		ret = -EINVAL;
		break;
	}

#if 0
	i2s_dbg("Leaving %s\n", __func__);
#endif
	return ret;
}

static struct snd_soc_dai_ops sprd_i2s_dai_ops = {
	.startup = i2s_startup,
	.shutdown = i2s_shutdown,
	.hw_params = i2s_hw_params,
	.trigger = i2s_trigger,
};

struct snd_soc_dai sprd_i2s_dai = {
	.name = "sprd-i2s",
	.playback = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_CONTINUOUS,
		     .rate_max = 96000,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,
		     },
	.capture = {
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = SNDRV_PCM_RATE_CONTINUOUS,
		    .rate_max = 96000,
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
		    },
	.ops = &sprd_i2s_dai_ops,
};
EXPORT_SYMBOL_GPL(sprd_i2s_dai);

static int i2s_drv_probe(struct platform_device *pdev)
{
	struct i2s_priv *i2s;
	struct resource *res;

	i2s_dbg("Entering %s\n", __func__);

	i2s = devm_kzalloc(&pdev->dev, sizeof(struct i2s_priv), GFP_KERNEL);
	if (!i2s) {
		pr_err("no memery!\n");
		return -ENOMEM;
	}

	i2s->dev = &pdev->dev;
	i2s->id = pdev->id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->membase = (void __iomem *)res->start;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	i2s->memphys = (unsigned int *)res->start;
	i2s_dbg("membase = 0x%x memphys = 0x%x\n", (int)i2s->membase, (int)i2s->memphys);

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	i2s->tx.dma_no = res->start;
	i2s->rx.dma_no = res->end;
	i2s_dbg("dma number tx = %d rx = %d\n", i2s->tx.dma_no, i2s->rx.dma_no);

	platform_set_drvdata(pdev, i2s);
	mutex_lock(&i2s_list_mutex);
	list_add(&i2s->list, &i2s_list);
	mutex_unlock(&i2s_list_mutex);

	atomic_set(&i2s->open_cnt, 0);

	i2s_dbg("Leaving %s\n", __func__);

	return 0;
}

static int __devexit i2s_drv_remove(struct platform_device *pdev)
{
	struct i2s_priv *i2s;
	i2s = platform_get_drvdata(pdev);
	mutex_lock(&i2s_list_mutex);
	list_del(&i2s->list);
	mutex_unlock(&i2s_list_mutex);
	return 0;
}

static struct platform_driver i2s_driver = {
	.probe = i2s_drv_probe,
	.remove = __devexit_p(i2s_drv_remove),

	.driver = {
		   .name = "i2s",
		   .owner = THIS_MODULE,
		   },
};

static int __init i2s_init(void)
{
	i2s_dbg("Entering %s\n", __func__);
	platform_driver_register(&i2s_driver);
	return snd_soc_register_dais(&sprd_i2s_dai, 1);
}

static void __exit i2s_exit(void)
{
	i2s_dbg("Entering %s\n", __func__);
	platform_driver_unregister(&i2s_driver);
	snd_soc_unregister_dais(&sprd_i2s_dai, 1);
}

module_init(i2s_init);
module_exit(i2s_exit);

MODULE_DESCRIPTION("SPRD ASoC I2S CUP-DAI driver");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cpu-dai:i2s");
