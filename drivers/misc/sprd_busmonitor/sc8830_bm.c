#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/regs_sc8830_ap_ahb.h>
#include <mach/regs_sc8830_pub_apb.h>

#include "sc8830_bm.h"

/*the debugfs file name*/
#define INTC	"intc"
#define CFG	"cfg"
#define ADDR_MIN	"addr_min"
#define ADDR_MAX	"addr_max"
#define ADDR_MSK	"addr_msk"
#define DATA_MIN_L	"data_min_l"
#define DATA_MIN_H	"data_min_h"
#define DATA_MAX_L	"data_max_l"
#define DATA_MAX_H 	"data_max_h"
#define DATA_MSK_L	"data_msk_l"
#define DATA_MSK_H	"data_msk_h"
#define CNT_WIN_LEN	"cnt_win_len"
#define PEAK_WIN_LEN	"peak_win_len"
#define MATCH_ADDR	"match_addr"
#define MATCH_CMD	"match_cmd"
#define MATCH_DATA_L	"match_data_l"
#define MATCH_DATA_H	"match_data_h"
#define RTRANS_IN_WIN	"rtrans_in_win"
#define RBW_IN_WIN	"rbw_in_win"
#define RLATENCY_IN_WIN	"rlatency_in_win"
#define WTRANS_IN_WIN	"wtrans_in_win"
#define WBW_IN_WIN	"wbw_in_win"
#define WLATENCY_IN_WIN	"wlatency_in_win"
#define PEAKBW_IN_WIN	"peakbw_in_win"
#define GLB_CTL		"glb_ctl"

/*the command will be write into debugfs*/
#define BM_GLB_ENABLE "bm_glb_enable"
#define BM_GLB_DISABLE	"bm_glb_disable"
#define BM_GLB_COUNT_START "bm_glb_count_start"
#define BM_GLB_COUNT_STOP "bm_glb_count_stop"

struct sci_bm_reg {
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

struct sci_bm_chip {
	void *__iomem reg_base;
	u32 bm_id;
	unsigned int irq_num;
	struct dentry *bm_dentry;
	spinlock_t lock;
	void (*call_back)(void *);
	void *call_back_arg;
	void *prvdata;
};

static struct sci_bm_chip *chip_set;

static int __bm_glb_enable(const struct sci_bm_chip *chip, bool is_accessible)
{
	u32 reg_addr, bit_shift;

	if (chip->bm_id >= AXI_BM0 && chip->bm_id <= AXI_BM9) {
		reg_addr = REG_PUB_APB_BUSMON_CFG;
		bit_shift = 0x1 << (16 + chip->bm_id);
	} else {
		if (chip->bm_id >= AHB_BM0 && chip->bm_id <= AHB_BM2) {
			reg_addr = REG_AP_AHB_AHB_EB;
			bit_shift = 0x1 << (14 + chip->bm_id - AHB_BM0);
		} else {
			return -EINVAL;
		}
	}

	if (is_accessible) {
		sci_glb_set(reg_addr, bit_shift);
	} else {
		sci_glb_clr(reg_addr, bit_shift);
	}

	return 0;
}

static void __bm_glb_count_enable(bool is_enable)
{
	u32 reg_val = 0;

	reg_val = sci_glb_read(REG_PUB_APB_BUSMON_CNT_START, 0x1);
	if (is_enable) {
		if (!reg_val)
			sci_glb_set(REG_PUB_APB_BUSMON_CNT_START, 0x1);
	} else {
		sci_glb_clr(REG_PUB_APB_BUSMON_CNT_START, 0x1);
	}
}

static void __bm_enable(const struct sci_bm_chip *chip, bool is_enable)
{
	volatile struct sci_bm_reg *reg;

	reg = (struct sci_bm_reg *)chip->reg_base;

	/*clean all interrupt*/
	reg->intc = 0x20000001;

	if (is_enable) {
		reg->intc = 0x10000001;
	} else {
		reg->intc = 0x0;
	}
}

static int __bm_chn_sel(const struct sci_bm_chip *chip, u32 chn_id)
{
	u32 reg_val;

	reg_val = sci_glb_read(REG_AP_AHB_MISC_CFG, 0xffffffff);

	switch (chip->bm_id) {
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

	sci_glb_write(REG_AP_AHB_MISC_CFG, reg_val, 0xffffffff);

	return 0;
}

static irqreturn_t __bm_isr(int irq_num, void *dev)
{
	struct sci_bm_chip *chip;
	volatile struct sci_bm_reg *reg;

	chip = (struct sci_bm_chip *)dev;
	reg = (struct sci_bm_reg *)chip->reg_base;

	if (unlikely(!(reg->intc & 0x40000000)))
		return IRQ_NONE;

	if (chip->call_back)
		chip->call_back(chip->call_back_arg);

	/*clean interrupt*/
	reg->intc |= 0x20000000;

	return IRQ_HANDLED;
}


static int __bm_glb_open(struct inode *i_node, struct file *file)
{
	/*get the sci_bm_chip point*/
	file->private_data = i_node->i_private;

	return 0;
}

static ssize_t __bm_glb_read(struct file *file, char __user *buf,size_t cnt, loff_t *ppos)
{
	return 0;
}


static ssize_t __bm_glb_write(struct file *file, const char __user *buf,size_t cnt, loff_t *ppos)
{
	int ret;
	char cmd[32];
	struct sci_bm_chip *chip = (struct sci_bm_chip *)file->private_data;

	ret = copy_from_user(cmd, buf, cnt);

	if (0 == strncmp(BM_GLB_ENABLE, cmd, cnt - 1)) {
		__bm_glb_enable(chip, true);
		goto exit;
	}
	if (0 == strncmp(BM_GLB_ENABLE, cmd, cnt - 1)) {
		__bm_glb_enable(chip, false);
		goto exit;
	}
	if (0 == strncmp(BM_GLB_COUNT_START, cmd, cnt - 1)) {
		__bm_glb_count_enable(true);
		goto exit;
	}
	if (0 == strncmp(BM_GLB_COUNT_STOP, cmd, cnt -1 )) {
		__bm_glb_count_enable(false);
		goto exit;
	}
exit:
	return ret;
}

static const struct file_operations bm_glb_ops = {
	.owner = THIS_MODULE,
	.write = __bm_glb_write,
	.read = __bm_glb_read,
	.open = __bm_glb_open,
};

static __devinit struct dentry *__bm_create_debugfs(const char *dev_name, const struct sci_bm_chip *chip)
{
	struct dentry *bm_dentry;
	umode_t rw_mode, r_mode;
	volatile struct sci_bm_reg *reg_;
	struct sci_bm_reg *reg;

	reg_ = (struct sci_bm_reg *)chip->reg_base;
	/*just to remove the compile warning message*/
	reg = (struct sci_bm_reg *)reg_;

	bm_dentry = debugfs_create_dir(dev_name, NULL);
	if ((u32)bm_dentry <= 0) {
		return NULL;
	}

	/*read and write mode*/
	rw_mode = S_IRUSR | S_IWUSR;
	/*only read mode*/
	r_mode = S_IRUSR;

	/*the global register file*/
	debugfs_create_file(GLB_CTL, rw_mode, bm_dentry, (void *)chip, &bm_glb_ops);

	/*the bm register file*/
	debugfs_create_x32(INTC, rw_mode, bm_dentry, &reg->intc);
	debugfs_create_x32(CFG, rw_mode, bm_dentry, &reg->cfg);
	debugfs_create_x32(ADDR_MIN, rw_mode, bm_dentry, &reg->addr_min);
	debugfs_create_x32(ADDR_MAX, rw_mode, bm_dentry, &reg->addr_max);
	debugfs_create_x32(ADDR_MSK, rw_mode, bm_dentry, &reg->addr_msk);
	debugfs_create_x32(DATA_MIN_L, rw_mode, bm_dentry, &reg->data_min_l);
	debugfs_create_x32(DATA_MIN_H, rw_mode, bm_dentry, &reg->data_min_h);
	debugfs_create_x32(DATA_MAX_L, rw_mode, bm_dentry, &reg->data_max_l);
	debugfs_create_x32(DATA_MAX_H, rw_mode, bm_dentry, &reg->data_max_h);
	debugfs_create_x32(DATA_MSK_L, rw_mode, bm_dentry, &reg->data_msk_l);
	debugfs_create_x32(DATA_MSK_H, rw_mode, bm_dentry, &reg->data_msk_h);
	debugfs_create_x32(CNT_WIN_LEN, rw_mode, bm_dentry,&reg->cnt_win_len);
	debugfs_create_x32(PEAK_WIN_LEN, r_mode, bm_dentry,&reg->peak_win_len);
	debugfs_create_x32(MATCH_ADDR, r_mode, bm_dentry, &reg->match_addr);
	debugfs_create_x32(MATCH_CMD, r_mode, bm_dentry, &reg->match_cmd);
	debugfs_create_x32(MATCH_DATA_L, r_mode, bm_dentry, &reg->match_data_l);
	debugfs_create_x32(MATCH_DATA_H, r_mode, bm_dentry, &reg->match_data_h);
	debugfs_create_x32(RTRANS_IN_WIN, r_mode, bm_dentry, &reg->rtrans_in_win);
	debugfs_create_x32(RBW_IN_WIN, r_mode, bm_dentry, &reg->rbw_in_win);
	debugfs_create_x32(RLATENCY_IN_WIN, r_mode, bm_dentry, &reg->rlatency_in_win);
	debugfs_create_x32(WTRANS_IN_WIN, r_mode, bm_dentry, &reg->wtrans_in_win);
	debugfs_create_x32(WBW_IN_WIN, r_mode, bm_dentry, &reg->wbw_in_win);
	debugfs_create_x32(WLATENCY_IN_WIN, r_mode, bm_dentry, &reg->wlatency_in_win);
	debugfs_create_x32(PEAKBW_IN_WIN, r_mode, bm_dentry, &reg->peakbw_in_win);

	return bm_dentry;
}

static int __devinit sci_bm_probe(struct platform_device *pdev)
{
	int ret;
	u32 irq_num;
	void *__iomem reg_base;
	char debug_file[32];
	struct resource *res;
	struct sci_bm_chip *chip;
	struct dentry *bm_dentry;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -ENODEV;
	}
	reg_base = (void *__iomem)res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		return -ENODEV;
	}
	irq_num = (u32)res->start;

	chip = chip_set + pdev->id;

	chip->bm_id = pdev->id;
	chip->reg_base = reg_base;

	ret = request_irq(irq_num, __bm_isr, IRQF_SHARED, pdev->name, chip);
	if (ret < 0) {
		/*fixme, print debug info*/
		goto err;
	}

	sprintf(debug_file, "%s%d", pdev->name, pdev->id);

	bm_dentry = __bm_create_debugfs(debug_file, chip);
	if ((u32)bm_dentry <= 0) {
		goto err;
	}
	chip->bm_dentry = bm_dentry;

	platform_set_drvdata(pdev, chip);

	return 0;
err:
	return ret;
}

static int __devexit sci_bm_remove(struct platform_device *pdev)
{
	struct sci_bm_chip *chip;

	chip = platform_get_drvdata(pdev);

	debugfs_remove_recursive(chip->bm_dentry);

	return 0;
}

static struct platform_driver sprd_bm_driver = {
	.probe = sci_bm_probe,
	.remove = sci_bm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_busmonitor",
	},
};

static int __init sci_bm_init(void)
{
	chip_set = kzalloc(sizeof(*chip_set) * BM_SIZE, GFP_KERNEL);
	if (NULL == chip_set) {
		printk("bm driver alloc memory failed!\n");
		return -ENOMEM;
	}

	return platform_driver_register(&sprd_bm_driver);
}

static void __exit sci_bm_exit(void)
{
	platform_driver_unregister(&sprd_bm_driver);
	kfree(chip_set);
}

module_init(sci_bm_init);
module_exit(sci_bm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.jiang<jack.jiang@apreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform busmonitor driver");
