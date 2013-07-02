/*copyright (C) 2013 Spreadtrum Communications Inc.
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

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/hardware.h>
#include <mach/__regs_sc8830_pub_apb.h>
#include <mach/bm_sc8830.h>

/*NOTE: depends on ioremap in io.c */
#define AXI_BM_BASE_OFFSET(chn)		(0x2000*chn)
#define AXI_BM_INTC_REG(chn)		(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x0)
#define AXI_BM_CFG_REG(chn)		(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x4)
#define AXI_BM_ADDR_MIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x8)
#define AXI_BM_ADDR_MAX_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0xc)
#define AXI_BM_ADDR_MSK_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x10)
#define AXI_BM_DATA_MIN_L_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x14)
#define AXI_BM_DATA_MIN_H_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x18)
#define AXI_BM_DATA_MAX_L_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x1c)
#define AXI_BM_DATA_MAX_H_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x20)
#define AXI_BM_DATA_MSK_L_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x24)
#define AXI_BM_DATA_MSK_H_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x28)
#define AXI_BM_CNT_WIN_LEN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x2c)
#define AXI_BM_PEAK_WIN_LEN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x30)
#define AXI_BM_MATCH_ADDR_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x34)
#define AXI_BM_MATCH_CMD_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x38)
#define AXI_BM_MATCH_DATA_L_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x3c)
#define AXI_BM_MATCH_DATA_H_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x40)
#define AXI_BM_RTRANS_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x44)
#define AXI_BM_RBW_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x48)
#define AXI_BM_RLATENCY_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x4c)
#define AXI_BM_WTRANS_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x50)
#define AXI_BM_WBW_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x54)
#define AXI_BM_WLATENCY_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x58)
#define AXI_BM_PEAKBW_IN_WIN_REG(chn)	(SPRD_AXIBM0_BASE + AXI_BM_BASE_OFFSET(chn) + 0x5c)

#define AXI_BM_EN			BIT(0)
#define AXI_BM_CNT_EN			BIT(1)
#define AXI_BM_CNT_START		BIT(3)
#define AXI_BM_CNT_CLR			BIT(4)
#define AXI_BM_INT_CLR			BIT(29)


static inline void __sci_axi_bm_chn_en(int chn)
{
	u32 val;

	val = __raw_readl(AXI_BM_INTC_REG(chn));
	val |= (AXI_BM_EN);
	__raw_writel(val, AXI_BM_INTC_REG(chn));

	return;
}

static inline u32 __sci_axi_bm_chn_cnt_bw(int chn)
{
	u32 rbw, wbw;

	rbw = __raw_readl(AXI_BM_RBW_IN_WIN_REG(chn));
	wbw = __raw_readl(AXI_BM_WBW_IN_WIN_REG(chn));
	pr_debug(" chn:%d, rbw:%u, wbw:%u \n", chn, rbw, wbw );

	return (rbw+wbw);
}

static void __sci_axi_bm_cnt_start(void)
{
	int bm_index;
	u32 val;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		val = __raw_readl(AXI_BM_INTC_REG(bm_index));
		val |= (AXI_BM_EN | AXI_BM_CNT_EN | AXI_BM_CNT_START);
		__raw_writel(val, AXI_BM_INTC_REG(bm_index));
	}
	return;
}

static void __sci_axi_bm_cnt_stop(void)
{
	int bm_index;
	u32 val;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		val = __raw_readl(AXI_BM_INTC_REG(bm_index));
		val &= ~(AXI_BM_CNT_START);
		__raw_writel(val, AXI_BM_INTC_REG(bm_index));
	}
	return;
}

/* performance count clear */
static void __sci_axi_bm_cnt_clr(void)
{
	int bm_index;
	u32 val;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		val = __raw_readl(AXI_BM_INTC_REG(bm_index));
		val |= (AXI_BM_CNT_EN | AXI_BM_EN);
		__raw_writel(val, AXI_BM_INTC_REG(bm_index));
		val &= ~(AXI_BM_CNT_START);
		__raw_writel(val, AXI_BM_INTC_REG(bm_index));
		val |= (AXI_BM_CNT_CLR);
		__raw_writel(val, AXI_BM_INTC_REG(bm_index));
		pr_debug(" chn:%d, val:0x%x, reg:0x%x \n", bm_index, val, AXI_BM_INTC_REG(bm_index));
		__sci_axi_bm_chn_cnt_bw(bm_index);
	}
	return;
}

static void __sci_bm_glb_count_enable(bool is_enable)
{
	u32 reg_val = 0;

	reg_val = sci_glb_read(REG_PUB_APB_BUSMON_CNT_START, 0x1);
	if (is_enable) {
		if (!reg_val)
			sci_glb_set(REG_PUB_APB_BUSMON_CNT_START, BIT(0));
	} else {
		sci_glb_clr(REG_PUB_APB_BUSMON_CNT_START, BIT(0));
	}
}

static int __sci_bm_glb_reset_and_enable(u32 bm_index, bool is_enable)
{
	u32 reg_en, reg_rst, bit_en, bit_rst;

	switch (bm_index) {
	case AXI_BM0:
	case AXI_BM1:
	case AXI_BM2:
	case AXI_BM3:
	case AXI_BM4:
	case AXI_BM5:
	case AXI_BM6:
	case AXI_BM7:
	case AXI_BM8:
	case AXI_BM9:
		reg_en = REG_PUB_APB_BUSMON_CFG;
		reg_rst = REG_PUB_APB_BUSMON_CFG;
		bit_en = BIT(16 + bm_index);
		bit_rst = BIT(bm_index);
		break;
	case AHB_BM0:
	case AHB_BM1:
	case AHB_BM2:
		reg_en = REG_AP_AHB_AHB_EB;
		reg_rst = REG_AP_AHB_AHB_RST;
		bit_en = BIT(14 + bm_index - AHB_BM0);
		bit_rst = BIT(17 + bm_index - AHB_BM0);
		break;

	default:
		return -1;
	}

	sci_glb_set(reg_rst, bit_rst);
	sci_glb_clr(reg_rst, bit_rst);

	if (is_enable) {
		sci_glb_set(reg_en, bit_en);
	} else {
		sci_glb_clr(reg_en, bit_en);
	}

	return 0;
}

unsigned int dmc_mon_cnt_bw(void)
{
	int chn;
	u32 cnt;

	cnt = 0;
	for (chn = AXI_BM0; chn <= AXI_BM9; chn++) {
		cnt += __sci_axi_bm_chn_cnt_bw(chn);
	}

	return cnt;
}
EXPORT_SYMBOL_GPL(dmc_mon_cnt_bw);

void dmc_mon_cnt_clr(void)
{
	__sci_axi_bm_cnt_clr();
	return;
}
EXPORT_SYMBOL_GPL(dmc_mon_cnt_clr);

void dmc_mon_cnt_start(void)
{
	__sci_bm_glb_count_enable(true);
	__sci_axi_bm_cnt_start();
	return;
}
EXPORT_SYMBOL_GPL(dmc_mon_cnt_start);

void dmc_mon_cnt_stop(void)
{
	__sci_bm_glb_count_enable(false);
	__sci_axi_bm_cnt_stop();
	return;
}
EXPORT_SYMBOL_GPL(dmc_mon_cnt_stop);

static int __init sci_bm_init(void)
{
	int bm_index;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		__sci_bm_glb_reset_and_enable(bm_index, true);
	}
	__sci_axi_bm_cnt_clr();

	for (bm_index = AHB_BM0; bm_index <= AHB_BM2; bm_index++) {
		__sci_bm_glb_reset_and_enable(bm_index, true);
	}

	return 0;
}

static void __exit sci_bm_exit(void)
{
	int bm_index;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		__sci_bm_glb_reset_and_enable(bm_index, false);
	}

	for (bm_index = AHB_BM0; bm_index <= AHB_BM2; bm_index++) {
		__sci_bm_glb_reset_and_enable(bm_index, false);
	}

	__sci_bm_glb_count_enable(false);
}

module_init(sci_bm_init);
module_exit(sci_bm_exit);
