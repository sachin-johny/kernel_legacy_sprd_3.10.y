/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/dma_reg.h>

struct sci_dma_desc {
	const char *dev_name;
	void (*irq_handler) (int, void *);
	void *data;
};

static struct sci_dma_desc *dma_chns;

static DEFINE_SPINLOCK(dma_lock);

static void __inline __dma_clk_enable(void)
{
	sci_glb_set(REG_AP_AHB_AHB_EB, BIT_DMA_EB);
}

static void __inline __dma_clk_disable(void)
{
	sci_glb_clr(REG_AP_AHB_AHB_EB, BIT_DMA_EB);
}

static void __dma_set_prio(u32 dma_chn, dma_pri_level chn_prio)
{
	u32 reg_val;

	reg_val = __raw_readl(DMA_CHN_CFG(dma_chn));
	reg_val &= ~(0x3 << 12);
	reg_val |= chn_prio << 12;

	__raw_writel(reg_val, DMA_CHN_CFG(dma_chn));
}

static int __dma_set_request_mode(u32 dma_chn, dma_request_mode mode)
{
	u32 reg_val = __raw_readl(DMA_CHN_FRAG_LEN(dma_chn));
	u32 req_mod = 0;

	switch (mode)
	{
	case FRAG_REQ_MODE:
		req_mod = 0x0;
		break;

	case BLOCK_REQ_MODE:
		req_mod = 0x1;
		break;

	case TRANS_REQ_MODE:
		if (dma_chn < FULL_CHN_START ||dma_chn > FULL_CHN_END)
			return -EINVAL;
		req_mod = 0x2;
		break;

	case LIST_REQ_MODE:
		if (dma_chn < FULL_CHN_START ||dma_chn > FULL_CHN_END)
			return -EINVAL;
		req_mod = 0x3;
		break;

	default:
		return -EINVAL;
	}

	reg_val &= ~(REQ_MODE_MASK << REQ_MODE_OFFSET);
	reg_val |= mode << REQ_MODE_OFFSET;

	__raw_writel(reg_val, DMA_CHN_FRAG_LEN(dma_chn));

	return 0;
}

static int __dma_set_int_type(u32 dma_chn, dma_int_type int_type)
{
	u32 reg_val;

	reg_val = __raw_readl(DMA_CHN_INT(dma_chn));
	reg_val &= ~0x1f;

	switch (int_type) {
	case INT_NONE:
		break;

	case FRAG_DONE:
		reg_val |= 0x1;
		break;

	case BLOCK_DONE:
		reg_val |= 0x2;
		break;

	case TRANS_DONE:
		reg_val |= 0x4;
		break;

	case LIST_DONE:
		reg_val |= 0x8;
		break;

	case CONFIG_ERR:
		reg_val |= 0x10;
		break;

	default:
		return -EINVAL;
	}

	__raw_writel(reg_val, DMA_CHN_INT(dma_chn));

	return 0;
}

/*convert struct sci_dma_cfg to struct sci_dma_reg*/
static int __dma_cfg_check_and_convert(const struct sci_dma_cfg *cfg,
				       struct sci_dma_reg *dma_reg)
{
	u32 datawidth = 0,req_mode = 0, is_end = 0, addr_fix_mod = 0;
	u32 llist_en = 0;

	switch (cfg->datawidth) {
	case 1:
		datawidth = 0;
		break;
	case 2:
		datawidth = 1;
		break;
	case 4:
		datawidth = 2;
		break;
	default:
		if (cfg->linklist_ptr)
			break;
		printk("DMA config datawidth error!\n");
		return -EINVAL;
	}

	/*check step, the step must be Integer multiple of the data width */
	if (!IS_ALIGNED(cfg->src_step, cfg->datawidth))
		return -EINVAL;

	if (!IS_ALIGNED(cfg->des_step, cfg->datawidth))
		return -EINVAL;
	/*linklist ptr must aligned with 8 bytes*/
	if (cfg->linklist_ptr) {
		if (!PTR_ALIGN(cfg->linklist_ptr, 8))
			return -EINVAL;
		req_mode = LIST_REQ_MODE;
	} else {
		if (cfg->block_len && (cfg->block_len != cfg->fragmens_len)) {
			req_mode = BLOCK_REQ_MODE;
		} else {
			req_mode = FRAG_REQ_MODE;
		}
	}

	if (cfg->src_step != 0 && cfg->des_step != 0) {
		addr_fix_mod = 0x0;
	} else {
		if (cfg->src_step == 0 && cfg->des_step == 0) {
			/*only full chn support data copy from fifo to fifo*/
			addr_fix_mod = 0;
		} else {
			if (cfg->src_step) {
				addr_fix_mod = 0x3;
			} else {
				addr_fix_mod = 0x1;
			}
		}
	}

	/*only full chn support data copy form fifo to fifo */
	if (cfg->transcation_len || cfg->linklist_ptr ||
	    ((cfg->src_step | cfg->des_step) == 0))
			goto full_chn_convert;

	dma_reg->cfg = DMA_PRI_1 << CHN_PRIORITY_OFFSET;
	/*src and des addr */
	dma_reg->src_addr = cfg->src_addr;
	dma_reg->des_addr = cfg->src_addr;
	/*frag len */
	dma_reg->frg_len =
		(datawidth << SRC_DATAWIDTH_OFFSET) |
		(datawidth << DES_DATAWIDTH_OFFSET) |
		(0x0 << SWT_MODE_OFFSET) |
		(req_mode << REQ_MODE_OFFSET) |
		(addr_fix_mod << ADDR_FIX_SEL_EN) |
		(cfg->fragmens_len & FRG_LEN_MASK);
	/*blk len */
	dma_reg->blk_len = cfg->block_len & BLK_LEN_MASK;

	return 0;

 full_chn_convert:
	if (cfg->is_end)
		is_end = 0x1;

	if (cfg->linklist_ptr)
		llist_en = 0x1;

	 dma_reg->cfg = DMA_PRI_1 << CHN_PRIORITY_OFFSET |
		llist_en << LLIST_EN_OFFSET;
	/*src and des addr */
	dma_reg->src_addr = cfg->src_addr;
	dma_reg->des_addr = cfg->src_addr;
	/*frag len */
	dma_reg->frg_len =
		(datawidth << SRC_DATAWIDTH_OFFSET) |
		 (datawidth << DES_DATAWIDTH_OFFSET) |
		(0x0 << SWT_MODE_OFFSET) |
		(req_mode << REQ_MODE_OFFSET) |
		(is_end << LLIST_END_OFFSET) |
		(addr_fix_mod << ADDR_FIX_SEL_EN) |
		(cfg->fragmens_len & FRG_LEN_MASK);
	/*blk len */
	dma_reg->blk_len = cfg->block_len & BLK_LEN_MASK;
	/*trac len */
	dma_reg->trsc_len = cfg->transcation_len & TRSC_LEN_MASK;
	/*fixme, the frag_step == datawidth*/
	dma_reg->trsf_step =
		(datawidth& TRSF_STEP_MASK) << DEST_TRSF_STEP_OFFSET |
		(datawidth & TRSF_STEP_MASK) << SRC_TRSF_STEP_OFFSET;
	dma_reg->llist_ptr = cfg->linklist_ptr;
	dma_reg->frg_step = (cfg->des_step<< DEST_FRAG_STEP_OFFSET) |
		cfg->src_step;
	dma_reg->src_blk_step = cfg->block_len;
	dma_reg->des_blk_step = cfg->block_len;

	return 0;
}

static void __inline __dma_int_clr(u32 dma_chn)
{
	__raw_writel(0x1f << 24, DMA_CHN_INT(dma_chn));
}

static void __init __dma_reg_init(void)
{
	int i = 0x100;;

	/*reset the DMA */
	sci_glb_set(REG_AP_AHB_AHB_RST, 0x1 << 8);
	while (i--) ;
	sci_glb_clr(REG_AP_AHB_AHB_RST, 0x1 << 8);

#ifdef DMA_DEBUG
	__raw_writel(16, DMA_FRAG_WAIT);
#else
	__raw_writel(0x0, DMA_FRAG_WAIT);
#endif
}

static irqreturn_t __dma_irq(int irq, void *dev_id)
{
	int i;
	u32 irq_status;
	u32 reg_addr;

	spin_lock(&dma_lock);

	irq_status = readl(DMA_INT_MSK_STS);

	if (unlikely(0 == irq_status)) {
		spin_unlock(&dma_lock);
		return IRQ_NONE;
	}

	while (irq_status) {
		i = __ffs(irq_status);
		irq_status &= (irq_status - 1);

		reg_addr = DMA_CHN_INT(i);
		/*clean all type interrupt */
		writel(readl(reg_addr) | (0xf << 24), reg_addr);

		if (dma_chns[i].irq_handler)
			dma_chns[i].irq_handler(irq, dma_chns[i].data);
	}

	spin_unlock(&dma_lock);

	return IRQ_HANDLED;
}

static void __inline __dma_set_uid(u32 dma_chn, u32 dev_id)
{
	if (DMA_UID_SOFTWARE != dev_id) {
		__raw_writel(dma_chn, DMA_REQ_CID(dev_id));
	}
}

static void __inline __dma_enable(u32 dma_chn)
{
	writel(readl(DMA_CHN_CFG(dma_chn)) | 0x1, DMA_CHN_CFG(dma_chn));
}

static void __inline __dma_soft_request(u32 dma_chn)
{
	writel(readl(DMA_CHN_REQ(dma_chn)) | 0x1, DMA_CHN_REQ(dma_chn));
}

static void __dma_chn_disable(u32 dma_chn)
{
	writel(readl(DMA_CHN_PAUSE(dma_chn)) | 0x1, DMA_CHN_PAUSE(dma_chn));
	/*fixme, need to set timeout */
	while (!(readl(DMA_CHN_PAUSE(dma_chn)) & (0x1 << 16))) ;

	writel(readl(DMA_CHN_CFG(dma_chn)) & ~0x1, DMA_CHN_CFG(dma_chn));

	writel(readl(DMA_CHN_PAUSE(dma_chn)) & ~0x1, DMA_CHN_PAUSE(dma_chn));
}

/*HAL layer function*/
int sci_dma_start(u32 dma_chn, u32 dev_id)
{
	if (dma_chn > DMA_CHN_MAX)
		return -EINVAL;

	/*fixme, need to check dev_id */
	__dma_set_uid(dma_chn, dev_id);

	__dma_enable(dma_chn);

	if (DMA_UID_SOFTWARE == dev_id)
		__dma_soft_request(dma_chn);

	return 0;
}

int sci_dma_stop(u32 dma_chn)
{
	if (dma_chn > DMA_CHN_MAX)
		return -EINVAL;

	__dma_int_clr(dma_chn);

	__dma_chn_disable(dma_chn);

	return 0;
}

int sci_dma_register_irqhandle(u32 dma_chn, dma_int_type int_type,
			       void (*irq_handle) (int, void *), void *data)
{
	int ret;

	if (dma_chn > DMA_CHN_MAX)
		return -EINVAL;

	if (NULL == irq_handle)
		return -EINVAL;

	ret = __dma_set_int_type(dma_chn, int_type);
	if (ret < 0)
		return ret;

	dma_chns[dma_chn].irq_handler = irq_handle;
	dma_chns[dma_chn].data = data;

	return 0;
}

int sci_dma_config(u32 dma_chn, struct sci_dma_cfg *cfg_list,
		   u32 node_size, struct reg_cfg_addr *cfg_addr)
{
	int ret, i;
	struct sci_dma_reg dma_reg;
	struct sci_dma_reg *dma_reg_list;
	volatile void *reg_addr;

	if (dma_chn > DMA_CHN_MAX)
		return -EINVAL;

	/*must init the dma_reg */
	memset(&dma_reg, 0x0, sizeof(dma_reg));

	if (node_size > 1)
		goto linklist_config;

	ret = __dma_cfg_check_and_convert(cfg_list, &dma_reg);
	if (ret < 0) {
		return -EINVAL;
	}

	goto fill_dma_reg;

 linklist_config:
	if (NULL == cfg_addr)
		return -EINVAL;

	dma_reg_list = (struct sci_dma_reg *)cfg_addr->virt_addr;

	for (i = 0; i < node_size; i++) {
		cfg_list[i].linklist_ptr = cfg_addr->phys_addr +
		    ((i + 1) % node_size) * sizeof(struct sci_dma_reg);
		ret =
		    __dma_cfg_check_and_convert(cfg_list + i, dma_reg_list + i);
		if (ret < 0)
			return -EINVAL;
	}

	dma_reg.llist_ptr = cfg_addr->phys_addr;

 fill_dma_reg:
	__dma_clk_enable();

	reg_addr = (void *)DMA_CHx_BASE(dma_chn);

	memcpy_toio(reg_addr, &dma_reg, sizeof(dma_reg));

	return 0;
}

int sci_dma_request(const char *dev_name, dma_chn_type chn_type)
{
	int i;
	u32 dma_chn;
	u32 dma_chn_start, dma_chn_end;
	ulong flags;

	if (!dev_name)
		return -EINVAL;

	dma_chn = -EBUSY;

	if (FULL_DMA_CHN == chn_type) {
		dma_chn_start = FULL_CHN_START;
		dma_chn_end = FULL_CHN_END;
	} else {
		dma_chn_start = DMA_CHN_MIN;
		dma_chn_end = DMA_CHN_MAX;
	}

	spin_lock_irqsave(&dma_lock, flags);

	for (i = dma_chn_start; i <= dma_chn_end; i++) {
		if (!dma_chns[i].dev_name) {
			dma_chns[i].dev_name = dev_name;
			dma_chn = i;
			break;
		}
	}

	spin_unlock_irqrestore(&dma_lock, flags);

	return dma_chn;
}

int sci_dma_free(u32 dma_chn)
{
	int i;
	ulong flags;

	if (dma_chn > DMA_CHN_MAX)
		return -EINVAL;

	__dma_int_clr(dma_chn);

	__dma_chn_disable(dma_chn);

	spin_lock_irqsave(&dma_lock, flags);

	memset(dma_chns + dma_chn, 0x0, sizeof(*dma_chns));

	/*if all dma chn be free, disable the DMA clk */
	for (i = DMA_CHN_MIN; i <= DMA_CHN_MAX; i++) {
		if (dma_chns[i].dev_name) {
			break;
		}
	}
	if (DMA_CHN_MAX + 1 == i)
		__dma_clk_disable();

	spin_unlock_irqrestore(&dma_lock, flags);

	return 0;
}

int sci_dma_ioctl(u32 dma_chn, dma_cmd cmd, void *arg)
{
	switch (cmd) {
	case SET_IRQ_TYPE:
		break;
	case SET_WRAP_MODE:
		break;

	default:
		break;
	}
	return 0;
}

static int __init sci_init_dma(void)
{
	int ret;

	dma_chns = kzalloc(sizeof(*dma_chns) * DMA_CHN_NUM, GFP_KERNEL);
	if (dma_chns == NULL)
		return -ENOMEM;

	__dma_clk_enable();

	__dma_reg_init();

	__dma_clk_disable();

	ret = request_irq(IRQ_DMA_INT, __dma_irq, 0, "sci-dma", NULL);
	if (ret) {
		printk(KERN_ERR "request dma irq failed %d\n", ret);
		goto request_irq_err;
	}

	return ret;

 request_irq_err:
	kfree(dma_chns);

	return ret;
}

arch_initcall(sci_init_dma);

EXPORT_SYMBOL_GPL(sci_dma_request);
EXPORT_SYMBOL_GPL(sci_dma_free);
EXPORT_SYMBOL_GPL(sci_dma_config);
EXPORT_SYMBOL_GPL(sci_dma_register_irqhandle);
EXPORT_SYMBOL_GPL(sci_dma_start);
EXPORT_SYMBOL_GPL(sci_dma_stop);
EXPORT_SYMBOL_GPL(sci_dma_ioctl);


#ifdef TMP_VERSION
/* those spreadtrum DMA interface must be implemented */
int sprd_dma_request(u32 uid, void (*handle) (int, void *), void *data)
{
	return 0;
}

void sprd_dma_free(u32 uid)
{
}

void sprd_dma_channel_config(u32 chn, dma_work_mode work_mode,
			     const struct sprd_dma_channel_desc *dma_cfg)
{
}

void sprd_dma_default_channel_setting(struct sprd_dma_channel_desc *dma_cfg)
{
}

void sprd_dma_default_linklist_setting(struct sprd_dma_linklist_desc *chn_cfg)
{
}

void sprd_dma_linklist_config(u32 chn_id, u32 dma_cfg)
{
}

void sprd_dma_wrap_addr_config(const struct sprd_dma_wrap_addr *wrap_addr)
{
}

void sprd_dma_set_irq_type(u32 chn, dma_done_type irq_type, u32 on_off)
{
}

void sprd_dma_set_chn_pri(u32 chn, u32 pri)
{
}

void sprd_dma_channel_start(u32 chn)
{
}

void sprd_dma_channel_stop(u32 chn)
{
}

/* ONLY FOR DEBUG */
void sprd_dma_check_channel(void)
{
}

void sprd_dma_dump_regs(void)
{
}

#endif
