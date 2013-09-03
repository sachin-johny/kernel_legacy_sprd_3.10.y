#include <linux/io.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <mach/hardware.h>

#ifdef CONFIG_DEBUG_FS
struct sci_pin_switch {
	const char *dirname;
	const char *filename;
	u32 reg;
	u32 bit_offset;
	u32 bit_width;
	u32 func;
};
static struct sci_pin_switch sci_pin_switch_array[] = {
	{"", "vbc_dac_iislrck_pin_in_sel", 0x4, 24, 1, 0},
	{"", "vbc_dac_iisclk_pin_in_sel", 0x4, 23, 1, 0},
	{"", "vbc_adc_iislrck_pin_in_sel", 0x4, 22, 1, 0},
	{"", "vbc_adc_iisdi_pin_in_sel", 0x4, 21, 1, 0},
	{"", "vbc_adc_iisclk_pin_in_sel", 0x4, 20, 1, 0},
	{"", "iis3mck_pin_in_sel", 0x4, 19, 1, 0},
	{"", "iis3lrck_pin_in_sel", 0x4, 18, 1, 0},
	{"", "iis3di_pin_in_sel", 0x4, 17, 1, 0},
	{"", "iis3clk_pin_in_sel", 0x4, 16, 1, 0},
	{"", "iis2mck_pin_in_sel", 0x4, 15, 1, 0},
	{"", "iis2lrck_pin_in_sel", 0x4, 14, 1, 0},
	{"", "iis2di_pin_in_sel", 0x4, 13, 1, 0},
	{"", "iis2clk_pin_in_sel", 0x4, 12, 1, 0},
	{"", "iis23_loop_sel", 0xc, 5, 1, 0},
	{"", "iis13_loop_sel", 0xc, 4, 1, 0},
	{"", "iis12_loop_sel", 0xc, 3, 1, 0},
	{"", "iis03_loop_sel", 0xc, 2, 1, 0},
	{"", "iis02_loop_sel", 0xc, 1, 1, 0},
	{"", "iis01_loop_sel", 0xc, 0, 1, 0},
};

static struct sci_pin_switch bt_iis_sys_sel_array[] = {
	{"bt_iis_sys_sel", "cp0_iis0", 0x8, 28, 4, 0},
	{"bt_iis_sys_sel", "cp0_iis1", 0x8, 28, 4, 1},
	{"bt_iis_sys_sel", "cp0_iis2", 0x8, 28, 4, 2},
	{"bt_iis_sys_sel", "cp0_iis3", 0x8, 28, 4, 3},
	{"bt_iis_sys_sel", "cp1_iis0", 0x8, 28, 4, 4},
	{"bt_iis_sys_sel", "cp1_iis1", 0x8, 28, 4, 5},
	{"bt_iis_sys_sel", "cp1_iis2", 0x8, 28, 4, 6},
	{"bt_iis_sys_sel", "cp1_iis3", 0x8, 28, 4, 7},
	{"bt_iis_sys_sel", "cp2_iis0", 0x8, 28, 4, 8},
	{"bt_iis_sys_sel", "cp2_iis1", 0x8, 28, 4, 9},
	{"bt_iis_sys_sel", "cp2_iis2", 0x8, 28, 4, 10},
	{"bt_iis_sys_sel", "cp2_iis3", 0x8, 28, 4, 11},
};

static struct sci_pin_switch iis_0_array[] = {
	{"iis0_sys_sel", "ap_iis0", 0xc, 6, 2, 0},
	{"iis0_sys_sel", "cp0_iis0", 0xc, 6, 2, 1},
	{"iis0_sys_sel", "cp1_iis0", 0xc, 6, 2, 2},
	{"iis0_sys_sel", "cp2_iis0", 0xc, 6, 2, 3},
};

static struct sci_pin_switch iis_1_array[] = {
	{"iis1_sys_sel", "ap_iis1", 0xc, 9, 2, 0},
	{"iis1_sys_sel", "cp0_iis1", 0xc, 9, 2, 1},
	{"iis1_sys_sel", "cp1_iis1", 0xc, 9, 2, 2},
	{"iis1_sys_sel", "cp2_iis1", 0xc, 9, 2, 3},
};

static struct sci_pin_switch iis_2_array[] = {
	{"iis2_sys_sel", "ap_iis2", 0xc, 12, 2, 0},
	{"iis2_sys_sel", "cp0_iis2", 0xc, 12, 2, 1},
	{"iis2_sys_sel", "cp1_iis2", 0xc, 12, 2, 2},
	{"iis2_sys_sel", "cp2_iis2", 0xc, 12, 2, 3},
};

static struct sci_pin_switch iis_3_array[] = {
	{"iis3_sys_sel", "ap_iis3", 0xc, 15, 2, 0},
	{"iis3_sys_sel", "cp0_iis3", 0xc, 15, 2, 1},
	{"iis3_sys_sel", "cp1_iis3", 0xc, 15, 2, 2},
	{"iis3_sys_sel", "cp2_iis3", 0xc, 15, 2, 3},
};

static struct sci_pin_switch_dir {
	struct sci_pin_switch *sci_pin_switch;
	u32 array_size;
};

static struct sci_pin_switch_dir sci_pin_switch_dir_array[] = {
	{bt_iis_sys_sel_array, ARRAY_SIZE(bt_iis_sys_sel_array)},
	{iis_0_array, ARRAY_SIZE(iis_0_array)},
	{iis_1_array, ARRAY_SIZE(iis_1_array)},
	{iis_2_array, ARRAY_SIZE(iis_1_array)},
	{iis_3_array, ARRAY_SIZE(iis_1_array)},
};

/*
*	#define IIS_TO_AP		(0)
*	#define IIS_TO_CP0		(1)
*	#define IIS_TO_CP1		(2)
*	#define IIS_TO_CP2		(3)
*	#define PIN_CTL_REG3 (SPRD_PIN_BASE + 0xc)
*/
static int read_write_pin_switch(int is_read, int v, struct sci_pin_switch *p)
{
	u32 shift = p->bit_offset;
	u32 mask = (1 << (p->bit_width)) - 1;
	u32 pin_ctl_reg = SPRD_PIN_BASE + p->reg;
	int val = 0;
	if ((shift > 31))
		BUG_ON(1);
	if (v > mask)
		printk("v:0x%x overflow bitwidth:%d, mask:0x%x it\n",
		       v, p->bit_width, mask);
	val = __raw_readl(pin_ctl_reg);
	if (is_read) {
		val >>= shift;
		val &= mask;
		return val;
	} else {
		val &= ~(mask << shift);
		val |= (v & mask) << shift;
		__raw_writel(val, pin_ctl_reg);
	}
	return val;
}

static int pin_switch_debug_set(void *data, u64 val)
{
	struct sci_pin_switch *p = data;
	read_write_pin_switch(0, val, p);
	return 0;
}

static int pin_switch_debug_get(void *data, u64 * val)
{
	struct sci_pin_switch *p = data;
	*val = read_write_pin_switch(1, (int)val, p);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pin_switch_enable_fops, pin_switch_debug_get,
			pin_switch_debug_set, "%llu\n");

static int pin_switch_dir_debug_set(void *data, u64 val)
{
	struct sci_pin_switch *p = data;
	if (val == 1)
		val = p->func;
	read_write_pin_switch(0, val, p);
	return 0;
}

static int pin_switch_dir_debug_get(void *data, u64 * val)
{
	struct sci_pin_switch *p = data;
	int func_tmp;
	func_tmp = read_write_pin_switch(1, (int)val, p);
	if (p->func == func_tmp) {
		*val = 1;
	} else {
		*val = 0;
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pin_switch_dir_enable_fops, pin_switch_dir_debug_get,
			pin_switch_dir_debug_set, "%llu\n");

static struct dentry *pin_switch_debugfs_base;
static int __init pin_switch_debug_add(struct sci_pin_switch *pin_switch)
{
	if (!debugfs_create_file
	    (pin_switch->filename, S_IRUGO | S_IWUSR, pin_switch_debugfs_base,
	     pin_switch, &pin_switch_enable_fops))
		return -ENOMEM;
	return 0;
}

static int __init pin_switch_debug_add_dir(struct sci_pin_switch_dir
					   *pin_switch_dir)
{
	static struct dentry *tmp_dir;
	int i;

	if (pin_switch_dir->sci_pin_switch->dirname == NULL)
		return EINVAL;
	/* has dir name, first create parent dir */
	tmp_dir =
	    debugfs_create_dir(pin_switch_dir->sci_pin_switch->dirname,
			       pin_switch_debugfs_base);
	if (!tmp_dir)
		return -ENOMEM;

	for (i = 0; i < pin_switch_dir->array_size; ++i) {
		if (pin_switch_dir->sci_pin_switch[i].filename == NULL)
			return EINVAL;
		if (!debugfs_create_file
		    (pin_switch_dir->sci_pin_switch[i].filename,
		     S_IRUGO | S_IWUSR, tmp_dir,
		     &pin_switch_dir->sci_pin_switch[i],
		     &pin_switch_dir_enable_fops))
			return -EINVAL;
	}
	return 0;
}

int __init pin_switch_debug_init(void)
{
	int i;
	pin_switch_debugfs_base = debugfs_create_dir("pin_switch", NULL);
	if (!pin_switch_debugfs_base)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(sci_pin_switch_array); ++i) {
		pin_switch_debug_add(&sci_pin_switch_array[i]);
	}
	for (i = 0; i < ARRAY_SIZE(sci_pin_switch_dir_array); ++i) {
		pin_switch_debug_add_dir(&sci_pin_switch_dir_array[i]);
	}
	return 0;
}

late_initcall(pin_switch_debug_init);
#else
#error "CONFIG_DEBUG_FS needed by mach-sc/pin_switch"
#endif /* CONFIG_DEBUG_FS */
