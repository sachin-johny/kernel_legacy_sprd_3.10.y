#include <linux/io.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <mach/hardware.h>

#ifdef CONFIG_DEBUG_FS
struct sci_pin_switch {
	const char *name;
	u32 reg;
	u32 bit_offset;
	u32 bit_width;
};
static struct sci_pin_switch sci_pin_switch_array[] = {
	{"iis-0", 0xc, 6, 2},
	{"iis-1", 0xc, 9, 2},
	{"iis-2", 0xc, 12, 2},
	{"iis-3", 0xc, 15, 2},
};

/*
*	#define IIS_TO_AP		(0)
*	#define IIS_TO_CP0		(1)
*	#define IIS_TO_CP1		(2)
*	#define IIS_TO_CP2		(3)
*	#define PIN_CTL_REG3 (SPRD_PIN_BASE + 0xc)
*/
static int read_write_pin_switch(int is_read, int v,
				struct sci_pin_switch *p)
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
static struct dentry *pin_switch_debugfs_base;
static int __init pin_switch_debug_add(struct sci_pin_switch *pin_switch)
{
	if (!debugfs_create_file
		(pin_switch->name, S_IRUGO | S_IWUSR, pin_switch_debugfs_base,
			pin_switch, &pin_switch_enable_fops))
		return -ENOMEM;
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
	return 0;
}

late_initcall(pin_switch_debug_init);
#else
#error "CONFIG_DEBUG_FS needed by mach-sc/pin_switch"
#endif	/* CONFIG_DEBUG_FS */
