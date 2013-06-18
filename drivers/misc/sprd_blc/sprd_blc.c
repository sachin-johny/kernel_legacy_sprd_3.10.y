#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/sci.h>
#include <mach/adi.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>

#define SCI_BLTC_BASE (ANA_BTLC_INT_BASE)

#define PRESCALE_OFFSET 0x00
#define DUTY_OFFSET	0x04
#define CUREV0_OFFSET	0x08
#define CUREV1_OFFSET 	0x0c

#define STS_OFFSET	0x34

#define SCI_BLC_CUREV_MSK 0x3f

/*blc working clock is 2kHz*/
#define SCI_DEFAULT_PRESCL	0X10
/*default duty ratio is 50%*/
#define SCI_DEFAULT_DUTY	0x408
/*1s rais 1s high 1s fall and 1s low*/
#define SCI_DEFAULT_CUREV	0x08080808


typedef enum {
	blc_red,
	blc_green,
	blc_blue,
} blc_color;

struct sci_blc_cfg {
	u16 prescale;
	u16 duty_ratio;
	/* raise_time | high_time | fall_time | low_time
	 * unit is 1/8 second 
	 */
	u32 work_curve;
};

#if 0
/*mov those to blc head file*/
int sci_blc_config(blc_color color, const struct sci_blc_cfg *cfg);
int sci_blc_onoff(blc_color color, bool on);
#endif
static struct dentry *blc_debug_dir;
static struct sci_blc_cfg red_cfg, green_cfg, blue_cfg;
static u32 red_cmd, green_cmd, blue_cmd;

static void __sci_blc_set_default_value(struct sci_blc_cfg *cfg)
{
	cfg->prescale	= SCI_DEFAULT_PRESCL;
	cfg->duty_ratio	= SCI_DEFAULT_DUTY;
	cfg->work_curve	= SCI_DEFAULT_CUREV;
}

static void __sci_blc_glb_enable(bool on)
{
	int i;

	if (on) {
		sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_BLTC_EN);
		sci_adi_set(ANA_REG_GLB_RTC_CLK_EN, BIT_RTC_BLTC_EN);
		sci_adi_set(ANA_REG_GLB_ANA_DRV_CTRL, BIT_SLP_RGB_PD_EN |
					BIT_RGB_PD_HW_EN );
		sci_adi_set(ANA_REG_GLB_ARM_RST, BIT_ANA_BLTC_SOFT_RST);
		for (i = 0; i < 0xf00; i++);
		sci_adi_clr(ANA_REG_GLB_ARM_RST, BIT_ANA_BLTC_SOFT_RST);
	} else {
		sci_adi_clr(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_BLTC_EN);
		sci_adi_clr(ANA_REG_GLB_RTC_CLK_EN, BIT_RTC_BLTC_EN);
		sci_adi_clr(ANA_REG_GLB_ANA_DRV_CTRL, BIT_SLP_RGB_PD_EN |
					BIT_RGB_PD_HW_EN );
	}
}

static int sci_blc_config(blc_color color, const struct sci_blc_cfg *cfg)
{
	u32 bltc_reg_base;
	u32 bltc_glb_ctl;

	/*fixme, need get the blc's busy status*/
	bltc_glb_ctl = sci_adi_read(SCI_BLTC_BASE);

	switch (color) {
	case blc_red:
		bltc_glb_ctl &= ~0xf;
		bltc_reg_base = SCI_BLTC_BASE + 0x4;
		break;
	case blc_green:
		bltc_glb_ctl &= ~(0xf << 4);
		bltc_reg_base = SCI_BLTC_BASE + 0x14;
		break;
	case blc_blue:
		bltc_glb_ctl &= ~(0xf << 8);
		bltc_reg_base = SCI_BLTC_BASE + 0x24;		
		break;
	default:
		return -EINVAL;
	}

	sci_adi_raw_write(SCI_BLTC_BASE, bltc_glb_ctl);

	sci_adi_raw_write(bltc_reg_base + PRESCALE_OFFSET, cfg->prescale);
	sci_adi_raw_write(bltc_reg_base + DUTY_OFFSET, cfg->duty_ratio);
	sci_adi_raw_write(bltc_reg_base + CUREV0_OFFSET, 
		((cfg->work_curve >> 8) & SCI_BLC_CUREV_MSK) << 8 |
		((cfg->work_curve >> 24) & SCI_BLC_CUREV_MSK));
	sci_adi_raw_write(bltc_reg_base + CUREV1_OFFSET,
		(cfg->work_curve & SCI_BLC_CUREV_MSK) << 8 |
		((cfg->work_curve >> 16) & SCI_BLC_CUREV_MSK) );

	return 0;
}
EXPORT_SYMBOL_GPL(sci_blc_config);

static int sci_blc_onoff(blc_color color, bool on)
{
	u32 operation_bit;
	
	switch (color) {
	case blc_red:
		operation_bit = BIT(0);
		break;
	case blc_green:
		operation_bit = BIT(4);
		break;			
	case blc_blue:
		operation_bit = BIT(8);
		break;			
	default:
		return -EINVAL;
	}

	if (on) {
		sci_adi_set(SCI_BLTC_BASE, operation_bit);
	} else {
		sci_adi_clr(SCI_BLTC_BASE, operation_bit);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sci_blc_onoff);


static int red_enable_get(void *data, u64 *val)
{
	*val = !!(sci_adi_read(SCI_BLTC_BASE + STS_OFFSET) & BIT(0));

	return 0;
}

static int green_enable_get(void *data, u64 *val)
{
	*val = !!(sci_adi_read(SCI_BLTC_BASE + STS_OFFSET) & BIT(1));

	return 0;
}

static int blue_enable_get(void *data, u64 *val)
{
	*val = !!(sci_adi_read(SCI_BLTC_BASE + STS_OFFSET) & BIT(2));

	return 0;
}

static int red_enable_set(void *data, u64 val)
{
	if (val) {
		sci_blc_config(blc_red, &red_cfg);
		sci_blc_onoff(blc_red, true);
	} else {
		sci_blc_onoff(blc_red, false);
	}

	return 0;
}

static int green_enable_set(void *data, u64 val)
{
	if (val) {
		sci_blc_config(blc_green, &green_cfg);
		sci_blc_onoff(blc_green, true);
	} else {
		sci_blc_onoff(blc_green, false);
	}

	return 0;
}

static int blue_enable_set(void *data, u64 val)
{
	if (val) {
		sci_blc_config(blc_blue, &blue_cfg);
		sci_blc_onoff(blc_blue, true);
	} else {
		sci_blc_onoff(blc_blue, false);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(red_ops, red_enable_get, red_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(green_ops, green_enable_get, green_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(blue_ops, blue_enable_get, blue_enable_set, "%llu\n");

#define blc_debug()\
	printk("%s %d\n", __func__, __LINE__)

static int __init sci_blc_init(void)
{
	blc_debug_dir = debugfs_create_dir("bltc", NULL);
	if (IS_ERR_OR_NULL(blc_debug_dir)) {
		return -ENOMEM;
	}

	if (IS_ERR_OR_NULL(debugfs_create_x16("red_clk_div", S_IRUSR | S_IWUSR, blc_debug_dir,
			&red_cfg.prescale)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x16("green_clk_div", S_IRUSR | S_IWUSR,  blc_debug_dir,
			&green_cfg.prescale)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x16("blue_clk_div", S_IRUSR | S_IWUSR, blc_debug_dir,
			&blue_cfg.prescale)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x16("red_duty_ratio", S_IRUSR | S_IWUSR, blc_debug_dir,
			&red_cfg.duty_ratio)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x16("green_duty_ratio", S_IRUSR | S_IWUSR, blc_debug_dir,
			&green_cfg.duty_ratio)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x16("blue_duty_ratio", S_IRUSR | S_IWUSR, blc_debug_dir,
			&blue_cfg.duty_ratio)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x32("red_curve", S_IRUSR | S_IWUSR, blc_debug_dir,
			&red_cfg.work_curve)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x32("green_curve", S_IRUSR | S_IWUSR, blc_debug_dir,
			&green_cfg.work_curve)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_x32("blue_curve", S_IRUSR | S_IWUSR, blc_debug_dir,
			&blue_cfg.work_curve)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_file("red_turnon", S_IRUGO | S_IWUSR, blc_debug_dir,
			&red_cmd, &red_ops)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_file("green_turnon", S_IRUGO | S_IWUSR, blc_debug_dir,
			&green_cmd, &green_ops)))
		goto err;

	if (IS_ERR_OR_NULL(debugfs_create_file("blue_turnon", S_IRUGO | S_IWUSR, blc_debug_dir,
			&blue_cmd, &blue_ops)))
		goto err;
	
	__sci_blc_set_default_value(&red_cfg);
	__sci_blc_set_default_value(&green_cfg);
	__sci_blc_set_default_value(&blue_cfg);

	__sci_blc_glb_enable(true);

	return 0;
err:
	debugfs_remove_recursive(blc_debug_dir);
	blc_debug_dir = NULL;

	return -ENOMEM;
}

static void __exit sci_blc_exit(void)
{
	__sci_blc_glb_enable(false);

	if (blc_debug_dir)
		debugfs_remove_recursive(blc_debug_dir);
}

module_init(sci_blc_init);
module_exit(sci_blc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jack.jiang<jack.jiang@apreadtrum.com>");
MODULE_DESCRIPTION("breathing light driver");
