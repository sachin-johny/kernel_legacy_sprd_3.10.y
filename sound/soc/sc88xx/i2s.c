/*
 * sound/soc/sc88xx/i2s.c
 *
 * SPRD SoC CPU-DAI -- SpreadTrum SOC pure DAI.
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
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/regs_global.h>
#include <mach/dma.h>

#include "i2s.h"

#if 1 /* def CONFIG_SPRD_AUDIO_DEBUG */
#define i2s_dbg pr_info
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
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
};

struct i2s_priv {
	int id;
	struct device *dev;
	struct list_head list;
	struct i2s_rtx rx;
	struct i2s_rtx tx;

	int irq_no;
	void __iomem *membase;
	unsigned int *memphys;
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
	struct i2s_config config;
	struct i2s_from_fm_to_vbc_config *vbc_config;
	struct mutex use_mutex;
};

static DEFINE_MUTEX(i2s_list_mutex);
static LIST_HEAD(i2s_list);

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
	i2s_dbg("Entering %s\n", __func__);
	i2s_soft_reset(i2s);
	i2s_global_disable(i2s);
	return 0;
}

static int i2s_open(struct i2s_priv *i2s)
{
	int ret;

	i2s_dbg("Entering %s\n", __func__);

	i2s_global_enable(i2s);
	i2s_soft_reset(i2s);
	i2s_dma_ctrl(i2s, 0);
	ret = i2s_config(i2s);

	return ret;
}

void print_dma_config(sprd_dma_desc *dma_desc)
{
	i2s_dbg("cfg 0x%x\n", dma_desc->cfg);
	i2s_dbg("tlen 0x%x\n", dma_desc->tlen);
	i2s_dbg("dsrc 0x%x\n", dma_desc->dsrc);
	i2s_dbg("ddst 0x%x\n", dma_desc->ddst);
	i2s_dbg("llptr 0x%x\n", dma_desc->llptr);
	i2s_dbg("pmod 0x%x\n", dma_desc->pmod);
	i2s_dbg("sbm 0x%x\n", dma_desc->sbm);
	i2s_dbg("dbm 0x%x\n", dma_desc->dbm);
}
EXPORT_SYMBOL_GPL(print_dma_config);

static void i2s_vbc_rx_dma_config(struct i2s_priv *i2s)
{
	sprd_dma_ctrl ctrl;
	struct i2s_config *config = &i2s->config;
	struct i2s_from_fm_to_vbc_config *vbc_config = i2s->vbc_config;
	int total_len = vbc_config->bytes >> 1;
	int burst_size = config->rx_watermark << 2;
	sprd_dma_desc *dma_desc = i2s->rx.dma_desc;

	i2s_dbg("Entering %s total_len %d burst_size %d\n", __func__, total_len, burst_size);


	BUG_ON(0 != (total_len % burst_size));

	ctrl.dma_desc = dma_desc;
	ctrl.dma_desc_phy = i2s->rx.dma_desc_phys;
	sprd_dma_setup_cfg(&ctrl,
			i2s->rx.dma_no, /* chan id */
			DMA_LINKLIST, /* dma mode */
			TRANS_DONE_EN, /* interrupt type */
			DMA_NOCHANGE, /* src */
			DMA_INCREASE, /* dst */
			SRC_BURST_MODE_SINGLE, /* src */
			SRC_BURST_MODE_4, /* dst */
			burst_size, /* burst size */
			32, 32, /* data width */
			I2S_PHY_REG(i2s, IIS_TXD), /* source address */
			vbc_config->addr, /* destination address */
			total_len); /* total length */
	dma_desc[0].cfg &= ~DMA_REQMODE_INFIINITE;
	dma_desc[0].cfg |= DMA_REQMODE_NORMAL;
	dma_desc[1] = dma_desc[0];
	dma_desc[1].ddst = vbc_config->addr + (vbc_config->bytes >> 1);
	dma_desc[0].llptr = i2s->rx.dma_desc_phys + sizeof(sprd_dma_desc);
	dma_desc[1].llptr = i2s->rx.dma_desc_phys;
	print_dma_config(ctrl.dma_desc);
	print_dma_config(&dma_desc[1]);
	sprd_dma_setup(&ctrl);
	sprd_dma_start(i2s->rx.dma_no);
}

static irqreturn_t i2s_vbc_rx_dma(int dma_ch, void *dev_id)
{
	struct i2s_priv *i2s = dev_id;
	struct i2s_from_fm_to_vbc_config *vbc_config = i2s->vbc_config;
	sprd_dma_set_irq_type(i2s->rx.dma_no, TRANSACTION_DONE, OFF);
	vbc_config->vbc_transfer_enable(1);
	i2s_dbg("start vbc\n");
	return IRQ_HANDLED;
}

/* 160 * 32 = 5K   20 * 256 = 5K */
/* pingpang buffer so * 2 */
/* frame is work so * 4 */
#define TROUT_FM_I2S_BUFFER_SIZE (1024 * 5 * 2 * 4)
static int i2s_dma_buffer_init(struct i2s_priv *i2s)
{
	int dma_desc_cnt;
	struct i2s_from_fm_to_vbc_config *vbc_config = i2s->vbc_config;
	i2s->dma_desc = dma_alloc_writecombine(i2s->dev, PAGE_SIZE,
			&i2s->dma_desc_phys, GFP_KERNEL);
	if (!i2s->dma_desc) {
		pr_err("no DMA memery!\n");
		return -ENOMEM;
	}

	vbc_config->bytes = TROUT_FM_I2S_BUFFER_SIZE;
	vbc_config->area = dma_alloc_writecombine(i2s->dev, vbc_config->bytes,
					   &vbc_config->addr, GFP_KERNEL);
	if (!vbc_config->area) {
		pr_err("no DMA memery!\n");
		return -ENOMEM;
	}
	dma_desc_cnt = (PAGE_SIZE / sizeof(sprd_dma_desc)) >> 1;

	i2s->rx.dma_desc = i2s->dma_desc;
	i2s->rx.dma_desc_phys = i2s->dma_desc_phys;
	vbc_config->dma_desc = i2s->dma_desc + dma_desc_cnt;
	vbc_config->dma_desc_phys = i2s->dma_desc_phys + (PAGE_SIZE >> 1);
	return 0;
}

static int i2s_dma_buffer_deinit(struct i2s_priv *i2s)
{
	struct i2s_from_fm_to_vbc_config *vbc_config = i2s->vbc_config;
	dma_free_writecombine(i2s->dev, PAGE_SIZE,
			i2s->dma_desc, i2s->dma_desc_phys);
	dma_free_writecombine(i2s->dev, vbc_config->bytes,
			vbc_config->area, vbc_config->addr);
}

int i2s_from_fm_to_vbc_config(int port, struct i2s_from_fm_to_vbc_config *cfg)
{
	int ret = 0;
	struct i2s_priv *i2s;
	i2s_dbg("Entering %s\n", __func__);
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

	mutex_lock(&i2s->use_mutex);
	i2s->config = *cfg->config;
	i2s->vbc_config = cfg;
	ret = i2s_open(i2s);
	if (ret < 0) {
		pr_err("i2s open error!\n");
		return ret;
	}
	ret = i2s_dma_buffer_init(i2s);
	if (ret < 0) {
		pr_err("i2s dma buffer init error!\n");
		goto __dma_buf_err;
	}
	sprd_request_dma(i2s->rx.dma_no, i2s_vbc_rx_dma, i2s);
	i2s_vbc_rx_dma_config(i2s);
	goto __ok_return;
__dma_buf_err:
	i2s_close(i2s);
__ok_return:
	return ret;
}
EXPORT_SYMBOL_GPL(i2s_from_fm_to_vbc_config);

int i2s_from_fm_to_vbc_enable(int port)
{
	struct i2s_priv *i2s;
	i2s_dbg("Entering %s\n", __func__);
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
	i2s_dma_ctrl(i2s, 1);
	return 0;
}
EXPORT_SYMBOL_GPL(i2s_from_fm_to_vbc_enable);

int i2s_from_fm_to_vbc_disable(int port)
{
	int ret;
	struct i2s_priv *i2s;
	i2s_dbg("Entering %s\n", __func__);
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
	i2s->vbc_config->vbc_transfer_enable(0);
	i2s_dma_ctrl(i2s, 0);
	sprd_free_dma(i2s->rx.dma_no);
	ret = i2s_close(i2s);
	i2s_dma_buffer_deinit(i2s);
	mutex_unlock(&i2s->use_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(i2s_from_fm_to_vbc_disable);

static u64 i2s_dmamask = DMA_BIT_MASK(32);

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

	mutex_init(&i2s->use_mutex);
	i2s->dev = &pdev->dev;

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &i2s_dmamask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

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

	i2s_dbg("Leaving %s\n", __func__);

	return 0;
}

static int __devexit i2s_drv_remove(struct platform_device *pdev)
{
	struct i2s_priv *i2s;
	mutex_lock(&i2s->use_mutex);
	i2s = platform_get_drvdata(pdev);
	mutex_lock(&i2s_list_mutex);
	list_del(&i2s->list);
	mutex_unlock(&i2s_list_mutex);
	mutex_unlock(&i2s->use_mutex);
	mutex_destroy(&i2s->use_mutex);
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
	return platform_driver_register(&i2s_driver);
}

static void __exit i2s_exit(void)
{
	platform_driver_unregister(&i2s_driver);
}

module_init(i2s_init);
module_exit(i2s_exit);

MODULE_DESCRIPTION("SPRD ASoC I2S CUP-DAI driver");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cpu-dai:i2s");
