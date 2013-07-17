#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/busmonitor.h>


struct sci_axi_bm_reg {
	u32 intc;
	u32 cfg;
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u32 data_min_l;
	u32 data_min_h;
	u32 data_max_l;
	u32 data_max_h;
	u32 data_msk_l;
	u32 data_msk_h;
	u32 cnt_win_len;
	u32 peak_win_len;
	u32 match_addr;
	u32 match_cmd;
	u32 match_data_l;
	u32 match_data_h;
	u32 rtrans_in_win;
	u32 rbw_in_win;
	u32 rlatency_in_win;
	u32 wtrans_in_win;
	u32 wbw_in_win;
	u32 wlatency_in_win;
	u32 peakbw_in_win;
};

struct sci_ahb_bm_reg {
	u32 intc;
	u32 cfg;
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u32 data_min;
	u32 data_max;
	u32 data_msk;
	u32 match_addr;
	u32 match_data;
	u32 bus_cnt;
};

static u32 __sci_get_bm_base(u32 bm_index)
{
	switch (bm_index) {
	case AXI_BM0:
		return SPRD_AXIBM0_BASE;
	case AXI_BM1:
		return SPRD_AXIBM1_BASE;
	case AXI_BM2:
		return SPRD_AXIBM2_BASE;
	case AXI_BM3:
		return SPRD_AXIBM3_BASE;
	case AXI_BM4:
		return SPRD_AXIBM4_BASE;
	case AXI_BM5:
		return SPRD_AXIBM5_BASE;
	case AXI_BM6:
		return SPRD_AXIBM6_BASE;
	case AXI_BM7:
		return SPRD_AXIBM7_BASE;
	case AXI_BM8:
		return SPRD_AXIBM8_BASE;
	case AXI_BM9:
		return SPRD_AXIBM9_BASE;
	case AHB_BM0:
		return SPRD_BM0_BASE;
	case AHB_BM1:
		return SPRD_BM1_BASE;
	case AHB_BM2:
		return SPRD_BM2_BASE;
	default:
		break;
	}

	return -EINVAL;
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
		return -EINVAL;
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

static int __sci_bm_chn_sel(u32 bm_index, u32 chn_id)
{
	u32 reg_val;

	reg_val = sci_glb_read(REG_AP_AHB_MISC_CFG, 0xffffffff);

	switch (bm_index) {
	case AHB_BM0:
		reg_val &= ~(0x1 << 4);
		reg_val |= (chn_id & 0x1) << 4;
		break;
	case AHB_BM1:
		reg_val &= ~(0x3 << 8);
		reg_val |= (chn_id & 0x3) << 8;
		break;
	case AHB_BM2:
		reg_val &= ~(0x3 << 10);
		reg_val |= (chn_id & 0x3) << 10;
		break;
	default:
		return -EINVAL;
	}

	sci_glb_write(REG_AP_AHB_MISC_CFG, reg_val, 0xff0);

	return 0;
}

static DEFINE_SPINLOCK(bm_lock);

struct bm_callback_desc {
	void (*fun)(void);
//	void *data;
};

static struct bm_callback_desc bm_callback_set[BM_SIZE];

#define BM_INT_MSK_STS	BIT(31)
#define BM_INT_CLR	BIT(29)
#define BM_INT_EN	BIT(28)
#define BM_CNT_CLR	BIT(4)
#define BM_CNT_EN	BIT(1)
#define BM_CHN_EN	BIT(0)

static irqreturn_t __sci_bm_isr(int irq_num, void *dev)
{
	u32 bm_index;
	u32 reg_addr;
	u32 int_ctl;
	void (*fun)(void);

	spin_lock(&bm_lock);

	bm_index = AXI_BM0;
	while (bm_index < BM_SIZE)
	{
		reg_addr = __sci_get_bm_base(bm_index);
		int_ctl = __raw_readl(reg_addr);
		if (int_ctl & BM_INT_MSK_STS) {
			fun = bm_callback_set[bm_index].fun;
			if (fun) {
				fun();
			}
			int_ctl |= BM_INT_CLR;
			/*clean irq status*/
			writel(int_ctl, reg_addr);
		}

		bm_index++;
	}

	spin_unlock(&bm_lock);

	return IRQ_HANDLED;
}

#if 0
static int chn_int_set(void *data, u64 val)
{
}

static int chn_int_get(void *data, u64 *p)
{
}

DEFINE_SIMPLE_ATTRIBUTE(chn_int_ops, chn_int_get, chn_get_set, "%x\n");
#endif

struct dentry *bm_root_dentry;

static __devinit struct dentry *__sci_bm_create_debug_file(const char *dev_name, u32 bm_index)
{
	struct dentry *bm_dentry;
	umode_t rw_mode, r_mode;
	u32 reg_base;

	bm_dentry = debugfs_create_dir(dev_name, bm_root_dentry);
	if (IS_ERR_OR_NULL(bm_dentry)) {
		return bm_dentry;
	}

	reg_base = __sci_get_bm_base(bm_index);

	/*read and write mode*/
	rw_mode = S_IRUSR | S_IWUSR;
	/*only read mode*/
	r_mode = S_IRUSR;

	/*fixme, need to check return value*/
	debugfs_create_x32("0x0000", rw_mode, bm_dentry, (u32 *)(reg_base + 0x00));
	debugfs_create_x32("0x0004", rw_mode, bm_dentry, (u32 *)(reg_base + 0x04));
	debugfs_create_x32("0x0008", rw_mode, bm_dentry, (u32 *)(reg_base + 0x08));
	debugfs_create_x32("0x000c", rw_mode, bm_dentry, (u32 *)(reg_base + 0x0c));
	debugfs_create_x32("0x0010", rw_mode, bm_dentry, (u32 *)(reg_base + 0x10));
	debugfs_create_x32("0x0014", rw_mode, bm_dentry, (u32 *)(reg_base + 0x14));
	debugfs_create_x32("0x0018", rw_mode, bm_dentry, (u32 *)(reg_base + 0x18));
	debugfs_create_x32("0x001c", rw_mode, bm_dentry, (u32 *)(reg_base + 0x1c));
	debugfs_create_x32("0x0020", rw_mode, bm_dentry, (u32 *)(reg_base + 0x20));
	debugfs_create_x32("0x0024", rw_mode, bm_dentry, (u32 *)(reg_base + 0x24));
	debugfs_create_x32("0x0028", rw_mode, bm_dentry, (u32 *)(reg_base + 0x28));

	/*the ahb bm use old version in 8830 AP side*/
	if (bm_index >= AHB_BM0)
		return bm_dentry;

	debugfs_create_x32("0x002c", rw_mode, bm_dentry, (u32 *)(reg_base + 0x2c));
	debugfs_create_x32("0x0030", r_mode, bm_dentry, (u32 *)(reg_base + 0x30));
	debugfs_create_x32("0x0034", r_mode, bm_dentry,	(u32 *)(reg_base + 0x34));
	debugfs_create_x32("0x0038", r_mode, bm_dentry, (u32 *)(reg_base + 0x38));
	debugfs_create_x32("0x003c", r_mode, bm_dentry, (u32 *)(reg_base + 0x3c));
	debugfs_create_x32("0x0040", r_mode, bm_dentry, (u32 *)(reg_base + 0x40));
	debugfs_create_x32("0x0044", r_mode, bm_dentry, (u32 *)(reg_base + 0x44));
	debugfs_create_x32("0x0048", r_mode, bm_dentry, (u32 *)(reg_base + 0x48));
	debugfs_create_x32("0x004c", r_mode, bm_dentry, (u32 *)(reg_base + 0x4c));
	debugfs_create_x32("0x0050", r_mode, bm_dentry, (u32 *)(reg_base + 0x50));
	debugfs_create_x32("0x0054", r_mode, bm_dentry, (u32 *)(reg_base + 0x54));
	debugfs_create_x32("0x0058", r_mode, bm_dentry, (u32 *)(reg_base + 0x58));
	debugfs_create_x32("0x005c", r_mode, bm_dentry, (u32 *)(reg_base + 0x5c));

	return bm_dentry;
}

int sci_bm_set_point(enum sci_bm_index bm_index, enum sci_bm_chn chn,
	const struct sci_bm_cfg *cfg, void(*call_back)(void))
{
	int ret;
	volatile struct sci_axi_bm_reg *axi_reg;
	volatile struct sci_ahb_bm_reg *ahb_reg;

	if (bm_index < AHB_BM0) {
		axi_reg = (struct sci_axi_bm_reg *)__sci_get_bm_base(bm_index);

		/*clean the irq status*/
		if(axi_reg->intc & BM_CHN_EN)
			axi_reg->intc |= BM_INT_CLR;

		axi_reg->intc = 0x0;

		memset((void *)axi_reg, 0x0, sizeof(*axi_reg));

		axi_reg->addr_min = cfg->addr_min;
		axi_reg->addr_max = cfg->addr_max;

		/* the interrupt just trigger by addr range for axi
		 * busmonitor, so set the data range difficult to
		 * access.
		 */
		axi_reg->data_min_l = 0x0fffffff;
		axi_reg->data_min_h = 0x0fffffff;
		axi_reg->data_max_l = 0x0;
		axi_reg->data_max_h = 0x0;

		axi_reg->data_msk_l = 0x0;
		axi_reg->data_msk_h = 0x0;

		switch (cfg->bm_mode) {
		case R_MODE:
			axi_reg->cfg = 0x1;
			break;
		case W_MODE:
			axi_reg->cfg = 0x3;
			break;
		case RW_MODE:
			axi_reg->cfg = 0x0;
			break;
		default:
			return -EINVAL;
		}

		axi_reg->intc |= BM_INT_EN | BM_CHN_EN;

	} else {
		ahb_reg = (struct sci_ahb_bm_reg *)__sci_get_bm_base(bm_index);

		if(ahb_reg->intc & BM_CNT_EN)
			ahb_reg->intc |= BM_INT_CLR;

		ahb_reg->intc = 0x0;

		memset((void *)ahb_reg, 0x0, sizeof(*ahb_reg));

		ahb_reg->addr_min = cfg->addr_min;
		ahb_reg->addr_max = cfg->addr_max;
		ahb_reg->addr_msk = 0x0;
		ahb_reg->data_min = cfg->data_min & 0xffffffff;
		ahb_reg->data_max = cfg->data_max & 0xffffffff;
		ahb_reg->data_msk = 0x0;

		switch (cfg->bm_mode) {
		case R_MODE:
			ahb_reg->cfg = 0x1;
			break;
		case W_MODE:
			ahb_reg->cfg = 0x3;
			break;
		case RW_MODE:
			ahb_reg->cfg = 0x0;
			break;
		default:
			return -EINVAL;
		}

		ret = __sci_bm_chn_sel(bm_index, chn);
		if (ret < 0)
			return ret;

		ahb_reg->intc |= BM_INT_EN | BM_CHN_EN;
	}

	bm_callback_set[bm_index].fun = call_back;

	return 0;
}

void sci_bm_unset_point(enum sci_bm_index bm_index)
{
	u32 reg_addr;

	reg_addr = __sci_get_bm_base(bm_index);

	writel(BM_INT_CLR | BM_CHN_EN, reg_addr);

	writel(0x0, reg_addr);

	bm_callback_set[bm_index].fun = NULL;
}

EXPORT_SYMBOL_GPL(sci_bm_set_point);
EXPORT_SYMBOL_GPL(sci_bm_unset_point);

static int __devinit sci_bm_probe(struct platform_device *pdev)
{
	int ret;
	u32 bm_index;
	struct dentry *bm_dentry;
	char file_name[32];

	ret = 0;

	if (0 == strcmp(SPRD_AXI_BM_NAME, pdev->name)) {
		ret = request_irq(IRQ_AXI_BM_PUB_INT, __sci_bm_isr, IRQF_TRIGGER_NONE, pdev->name, pdev);
		if (ret)
			return ret;

		for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
			sprintf(file_name, "%s%d", "axi_bm", bm_index);
			bm_dentry = __sci_bm_create_debug_file(file_name, bm_index);
			if (IS_ERR_OR_NULL(bm_dentry))
				return -ENOENT;
			/*enable busmonitor default*/
			__sci_bm_glb_reset_and_enable(bm_index, true);
		}

		__sci_bm_glb_count_enable(true);
	}

	if (0 == strcmp(SPRD_AHB_BM_NAME, pdev->name)) {
		ret = request_irq(IRQ_BM0_INT, __sci_bm_isr, IRQF_SHARED, pdev->name, pdev);
		if (ret)
			return ret;
		ret = request_irq(IRQ_BM1_INT, __sci_bm_isr, IRQF_SHARED, pdev->name, pdev);
		if (ret)
			return ret;
		ret = request_irq(IRQ_BM2_INT, __sci_bm_isr, IRQF_SHARED, pdev->name, pdev);
		if (ret)
			return ret;

		for (bm_index = AHB_BM0; bm_index < BM_SIZE; bm_index++) {
			sprintf(file_name, "%s%d", "ahb_bm", bm_index - AHB_BM0);
			bm_dentry = __sci_bm_create_debug_file(file_name, bm_index);
			if (IS_ERR_OR_NULL(bm_dentry))
				return -ENOENT;
			__sci_bm_glb_reset_and_enable(bm_index, true);
		}
	}

	return 0;
}

static int __devexit sci_bm_remove(struct platform_device *pdev)
{
	u32 bm_index;

	for (bm_index = AXI_BM0; bm_index < BM_SIZE; bm_index++) {
		__sci_bm_glb_reset_and_enable(bm_index, false);
	}

	__sci_bm_glb_count_enable(false);

	free_irq(IRQ_AXI_BM_PUB_INT, pdev);
	free_irq(IRQ_BM0_INT, pdev);
	free_irq(IRQ_BM1_INT, pdev);
	free_irq(IRQ_BM2_INT, pdev);

	return 0;
}

static const struct platform_device_id sci_bm_ids[] = {
	[0] = {
		.name = SPRD_AXI_BM_NAME,
	},
	[1] = {
		.name = SPRD_AHB_BM_NAME,
	},
	[2] = {
	},
};

static struct platform_driver sprd_bm_driver = {
	.probe    = sci_bm_probe,
	.remove   = sci_bm_remove,
	.id_table = sci_bm_ids,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_busmonitor",
	},
};

static struct platform_device sprd_bm_axi_device = {
	.name = "sprd_axi_busmonitor",
	.id = 0,
};

static struct platform_device sprd_bm_ahb_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 0,
};

static int __init sci_bm_init(void)
{
	bm_root_dentry = debugfs_create_dir("sprd_bm", NULL);
	if (IS_ERR_OR_NULL(bm_root_dentry))
		return -ENOENT;
	
	platform_device_register(&sprd_bm_axi_device);
	platform_device_register(&sprd_bm_ahb_device);

	return platform_driver_register(&sprd_bm_driver);
}

static void __exit sci_bm_exit(void)
{
	platform_device_unregister(&sprd_bm_axi_device);
	platform_device_unregister(&sprd_bm_ahb_device);

	platform_driver_unregister(&sprd_bm_driver);
	debugfs_remove_recursive(bm_root_dentry);
}

module_init(sci_bm_init);
module_exit(sci_bm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.jiang<jack.jiang@apreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform busmonitor driver");
