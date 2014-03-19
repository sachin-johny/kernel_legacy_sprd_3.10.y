/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Fixes:
 *		0.2
 *		ARM: sc: add parent pll clock alias
 *		and enable mpll fedback divider config
 *		Change-Id: Ic2e5d78a058d3b017ea17b82e3a920c3efefcf49
 *		0.1
 *		shark dcam: update dcam and mm clocks
 *		Change-Id: Id85d58178aca40fdf13b996853711e92e1171801
 *
 * To Fix:
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>

#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/hardware.h>

#include "clock_dt.h"
#include "mach/__clock_tree.h"

extern int local_mm_enable(struct sci_clk *c, int enable);

const u32 __clkinit0 __clkinit_begin = 0xeeeebbbb;
const u32 __clkinit2 __clkinit_end = 0xddddeeee;
static struct device_node *local_device_node;
#define to_sci_clk(_hw) container_of(_hw, struct sci_clk, hw)

/**
 * clk_force_disable - force disable clock output
 * @clk: clock source
 *
 * Forcibly disable the clock output.
 * NOTE: this *will* disable the clock output even if other consumer
 * devices have it enabled. This should be used for situations when device
 * suspend or damage will likely occur if the devices is not disabled.
 */
extern bool __clk_is_enabled(struct clk *clk);
extern int __clk_enable(struct clk *clk);
extern int __clk_prepare(struct clk *clk);
extern void __clk_disable(struct clk *clk);
void clk_force_disable(struct clk *clk)
{
	do{
		if(__clk_is_enabled(clk))
			clk_disable(clk);
		else
			break;
	}while(1);
}

EXPORT_SYMBOL(clk_force_disable);

static int local_clk_enable(struct sci_clk *c, int enable)
{
	debug("clk %p (%s) %s %08x[%d]\n", c, c->regs->name,
	      enable ? "enb" : "dis", c->regs->enb.reg,
	      __ffs(c->regs->enb.mask));

	BUG_ON(!c->regs->enb.reg);
	if (c->regs->enb.reg & 1)
		enable = !enable;

	if (!c->regs->enb.mask) {	/* enable matrix clock */
		if (enable){
			__clk_prepare(((struct sci_clk *)c->regs->enb.reg)->hw.clk);
			__clk_enable(((struct sci_clk *)c->regs->enb.reg)->hw.clk);
		}
		else{
			__clk_disable(((struct sci_clk *)c->regs->enb.reg)->hw.clk);
			__clk_unprepare(((struct sci_clk *)c->regs->enb.reg)->hw.clk);
		}
	} else {
		if (enable)
			sci_glb_set(c->regs->enb.reg & ~1, c->regs->enb.mask);
		else
			sci_glb_clr(c->regs->enb.reg & ~1, c->regs->enb.mask);
	}
	return 0;
}
static int sci_clk_enable(struct clk_hw *hw)
{
	struct sci_clk *c = to_sci_clk(hw);
	local_clk_enable(c, 1);
}
static int sci_clk_disable(struct clk_hw *hw)
{
	struct sci_clk *c = to_sci_clk(hw);
	local_clk_enable(c, 0);
}
static int sci_mm_clk_enable(struct clk_hw *hw)
{
	struct sci_clk *c = to_sci_clk(hw);
	local_mm_enable(c, 1);
}
static int sci_mm_clk_disable(struct clk_hw *hw)
{
	struct sci_clk *c = to_sci_clk(hw);
	local_mm_enable(c, 0);
}

static int sci_clk_is_enable(struct clk_hw *hw)
{
	int enable;
	struct sci_clk *c = to_sci_clk(hw);

	debug0("clk %p (%s) enb %08x\n", c, c->regs->name, c->regs->enb.reg);

	BUG_ON(!c->regs->enb.reg);
	if (!c->regs->enb.mask) {	/* check matrix clock */
		enable = ! !__clk_is_enabled(((struct sci_clk *)c->regs->enb.reg)->hw.clk);
	} else {
		enable =
		    ! !sci_glb_read(c->regs->enb.reg & ~1, c->regs->enb.mask);
	}

	if (c->regs->enb.reg & 1)
		enable = !enable;
	return enable;
}

static int sci_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long prate)
{
	int div;
	u32 div_shift;
	struct sci_clk *c = to_sci_clk(hw);

	debug2("clk %p (%s) set rate %lu\n", c, c->regs->name, rate);
	div = prate / rate - 1;	//FIXME:
	if (div < 0)
		return -EINVAL;
	div_shift = __ffs(c->regs->div.mask);
	debug("clk %p (%s) pll div reg %08x, val %08x mask %08x\n", c,
	      c->regs->name, c->regs->div.reg, div << div_shift,
	      c->regs->div.mask);

	if (c->regs->div.reg)
		sci_glb_write(c->regs->div.reg, div << div_shift,
			      c->regs->div.mask);

	return 0;
}

/* recalc rate */
static unsigned long sci_clk_get_rate(struct clk_hw *hw, unsigned long prate)
{
	u32 div = 0, div_shift;
	unsigned long rate;
	struct sci_clk *c = to_sci_clk(hw);
	div_shift = __ffs(c->regs->div.mask);
	debug0("clk %p (%s) div reg %08x, shift %u msk %08x\n", c,
	       c->regs->name, c->regs->div.reg, div_shift, c->regs->div.mask);

	if (c->regs->div.reg)
		div = sci_glb_read(c->regs->div.reg,
				   c->regs->div.mask) >> div_shift;
	debug0("clk %p (%s) parent rate %lu, div %u\n", c, c->regs->name, prate,
	       div + 1);
	rate = prate / (div + 1);	//FIXME:
	debug0("clk %p (%s) get real rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static unsigned long sci_pll_get_refin_rate(struct sci_clk *c)
{
	const unsigned long refin[4] = { 2000000, 4000000, 13000000, 26000000 };
	u32 i, msk = BITS_MPLL_REFIN(-1);
	i = sci_glb_read(c->regs->div.reg, msk) >> __ffs(msk);
	debug0("pll %p (%s) refin %d\n", c, c->regs->name, i);
	BUG_ON(i >= ARRAY_SIZE(refin));
	return refin[i];
}

static unsigned long sci_pll_get_rate(struct clk_hw *hw, unsigned long prate)
{
	u32 mn = 1, mn_shift;
	unsigned long rate;
	struct sci_clk *c = to_sci_clk(hw);
	debug0("pll %p (%s) hw %p regs %p div %p\n", c, c->regs->name, hw, c->regs, c->regs->div);
	mn_shift = __ffs(c->regs->div.mask);
	debug0("pll %p (%s) mn reg %08x, shift %u msk %08x\n", c, c->regs->name,
	       c->regs->div.reg, mn_shift, c->regs->div.mask);

	/* get parent rate */
	rate = prate;

	if (0 == c->regs->div.reg) {
		if (c->rate)
			rate = c->rate;	/* fixed rate */
	} else if (c->regs->div.reg < MAX_DIV) {
		mn = c->regs->div.reg;
		if (mn)
			rate = rate / mn;
	} else {
		rate = sci_pll_get_refin_rate(c);
		mn = sci_glb_read(c->regs->div.reg,
				  c->regs->div.mask) >> mn_shift;
		if (mn)
			rate = rate * mn;
	}
	debug0("pll %p (%s) get real rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static int __pll_enable_time(struct sci_clk *c, unsigned long rate, unsigned long old_rate)
{
	/* FIXME: for mpll, each step (100MHz) takes 50us */
	unsigned long diff = abs(rate - old_rate);
	int dly;
	diff = diff/1000000;
	dly = diff * 50 / 100;
	WARN_ON(dly > 1000);
	udelay(dly);
	return 0;
}

static int sci_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long prate)
{
	u32 mn = 1, mn_shift;
	struct sci_clk *c = to_sci_clk(hw);
	mn_shift = __ffs(c->regs->div.mask);
	debug("pll %p (%s) rate %lu, mn reg %08x, shift %u msk %08x\n", c,
	      c->regs->name, rate, c->regs->div.reg, mn_shift,
	      c->regs->div.mask);

	if (0 == c->regs->div.reg || c->regs->div.reg < MAX_DIV) {
/*
		WARN(1, "warning: clock (%s) not support set\n", c->regs->name);
 */
	} else {
		unsigned long old_rate = prate/sci_pll_get_refin_rate(c);
		mn = rate / sci_pll_get_refin_rate(c);
		sci_glb_write(c->regs->div.reg, mn << mn_shift,
			      c->regs->div.mask);
		__pll_enable_time(c, rate, old_rate);
	}

	debug2("pll %p (%s) set rate %lu\n", c, c->regs->name, rate);
	return 0;
}

static unsigned long sci_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct sci_clk *c = to_sci_clk(hw);
	debug0("clk %p (%s) round rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static int sci_clk_set_parent(struct clk_hw *hw, u8 i)
{
	struct sci_clk *c = to_sci_clk(hw);
	u32 sel_shift = __ffs(c->regs->sel.mask);
	debug0("clk %s set parent to %d\n", c->regs->name, i);

	debug0("clk sel reg %08x, val %08x, msk %08x\n",
			c->regs->sel.reg, i << sel_shift,
			c->regs->sel.mask);
	if (c->regs->sel.reg)
		sci_glb_write(c->regs->sel.reg, i << sel_shift,
				c->regs->sel.mask);
	return 0;
}

static u8 sci_clk_get_parent(struct clk_hw *hw)
{
	u8 i = 0;
	struct sci_clk *c = to_sci_clk(hw);
	u32 sel_shift = __ffs(c->regs->sel.mask);
	debug0("pll sel reg %08x, val %08x, msk %08x\n",
	       c->regs->sel.reg, i << sel_shift, c->regs->sel.mask);
	if (c->regs->sel.reg) {
		i = sci_glb_read(c->regs->sel.reg,
				 c->regs->sel.mask) >> sel_shift;
	}
	return i;
}

static struct clk_ops generic_clk_ops = {
	.set_rate = sci_clk_set_rate,
	.recalc_rate = sci_clk_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
	.enable = sci_clk_enable,
	.disable = sci_clk_disable,
};
static struct clk_ops generic_mm_clk_ops = {
	.set_rate = sci_clk_set_rate,
	.recalc_rate = sci_clk_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
	.enable = sci_mm_clk_enable,
	.disable = sci_mm_clk_disable,
};
static struct clk_ops generic_no_enable_clk_ops = {
	.set_rate = sci_clk_set_rate,
	.recalc_rate = sci_clk_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
};
static struct clk_ops generic_simp_clk_ops = {
	.enable = sci_clk_enable,
	.disable = sci_clk_disable,
};

static struct clk_ops generic_pll_ops = {
	.set_rate = sci_pll_set_rate,
	.recalc_rate = sci_pll_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
	.enable = sci_clk_enable,
	.disable = sci_clk_disable,
};
static struct clk_ops generic_mm_pll_ops = {
	.set_rate = sci_pll_set_rate,
	.recalc_rate = sci_pll_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
	.enable = sci_mm_clk_enable,
	.disable = sci_mm_clk_disable,
};
static struct clk_ops generic_no_enable_pll_ops = {
	.set_rate = sci_pll_set_rate,
	.recalc_rate = sci_pll_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
	.get_parent = sci_clk_get_parent,
};
static struct clk_ops mm_simp_clk_ops = {
	.enable = sci_mm_clk_enable,
	.disable = sci_mm_clk_disable,
};
static struct clk_ops empty_clk_ops = {
};

/* debugfs support to trace clock tree hierarchy and attributes */

static __init int __clk_is_dummy_pll(struct sci_clk *c)
{
	return (c->regs->enb.reg & 1) || strstr(c->regs->name, "pll");
}

/*
static __init int __clk_is_dummy_internal(struct clk *c)
{
	int i = strlen(c->regs->name);
	return c->regs->name[i - 2] == '_' && c->regs->name[i - 1] == 'i';
}
*/
static __init int __clk_add_alias(struct sci_clk *c)
{
	int i = strlen(c->regs->name);
	const char *p = &c->regs->name[i - 3];
	if (isdigit(p[0]) && p[1] == 'm' && isdigit(p[2])) {
		char alias[16];
		struct clk_lookup *l;
		strcpy(alias, c->regs->name);
		strcat(alias, "00k");
		l = clkdev_alloc(c, alias, 0);
		BUG_ON(!l);
		clkdev_add(l);
		debug("%s <--- %s\n", c->regs->name, alias);
	}
	return 0;
}
int __init sci_clk_register(struct clk_lookup *cl)
{
	struct sci_clk *c = (struct sci_clk *)cl->clk;
	struct clk_init_data init = {0};
	struct clk *clk;
	int ops_i = 1, ops_j = 0, handle = 0;

	init.name = c->regs->name;
	if (c->rate && !c->regs->nr_sources){	/* fixed OSC */
		clk = clk_register_fixed_rate(NULL, c->regs->name, NULL, 0, c->rate);
		goto cl_add;
	}
	else if ((c->regs->div.reg >= 0 && c->regs->div.reg < MAX_DIV)
			|| c->rate || strstr(c->regs->name, "pll")) {
		ops_i = 2;
	}

	debug0
	    ("clk %p (%s) rate %lu enb %08x sel %08x div %08x nr_sources %u\n",
	     c, c->regs->name, c->rate, c->regs->enb.reg,
	     c->regs->sel.reg, c->regs->div.reg, c->regs->nr_sources);
	debug0(" ops_i %d\n", ops_i);

	if (c->regs->nr_sources) {	/* FIXME: dummy update clock parent and rate */
		init.parent_names = c->regs->sources;
		init.num_parents = c->regs->nr_sources;
	}
#if defined(CONFIG_ARCH_SCX35)
	if (strcmp(c->regs->name, "clk_mm_i") == 0) {
		ops_j = 1;
		if(ops_i == 1)
			init.ops = &generic_mm_clk_ops;
		else
			init.ops = &generic_mm_pll_ops;
	}
#endif

	/* enable == NULL */
	if (ops_j == 0 ){
		if(c->regs->enb.reg) {
			if(ops_i == 1)
				init.ops = &generic_clk_ops;
			else
				init.ops = &generic_pll_ops;
			handle = 1;
		}else{
			if(ops_i == 1)
				init.ops = &generic_no_enable_clk_ops;
			else if(ops_i == 2)
				init.ops = &generic_no_enable_pll_ops;
		}
	}

	c->hw.init = &init;
	clk  = clk_register(NULL, &c->hw);
	if(IS_ERR(clk)){
		BUG();
	}

cl_add:
	cl->clk = clk;
	clkdev_add(cl);
	__clk_add_alias(c);

	__clk_prepare(clk);
	if (handle) {
		/* FIXME: dummy update some pll clocks usage */
		if (__clk_is_dummy_pll(c) && sci_clk_is_enable(c)) {
			__clk_enable(clk);
		}
	}

	return 0;
}

static int __init sci_clock_dump(void)
{
#if defined(CONFIG_ARCH_SCX15)
#else
	struct clk *clk;
	clk = clk_get_sys(NULL, "clk_mm_i");
	clk_prepare_enable(clk);
#if 0
	struct clk_lookup *cl = (struct clk_lookup *)(&__clkinit_begin + 1);
	clk_enable(&clk_gpu_i);
	while (cl < (struct clk_lookup *)&__clkinit_end) {
		struct clk *c = cl->clk;
		struct clk *p = clk_get_parent(c);
		if (!__clk_is_dummy_internal(c))
			printk
			    ("@@@clock[%s] is %sactive, usage %d, rate %lu, parent[%s]\n",
			     c->regs->name,
			     (c->enable == NULL
			      || sci_clk_is_enable(c)) ? "" : "in", c->usage,
			     clk_get_rate(c), p ? p->regs->name : "none");
		cl++;
	}
	debug("okay\n");
	clk_disable(&clk_gpu_i);
#endif
	clk_disable_unprepare(clk);
#endif
	return 0;
}

static int
__clk_cpufreq_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
#if 0				/*!defined(CONFIG_ARCH_SCX35) */
	struct cpufreq_freqs *freq = data;
	printk("%s (%u) dump cpu freq (%u %u %u %u)\n",
	       __func__, (unsigned int)val,
	       freq->cpu, freq->old, freq->new, (unsigned int)freq->flags);
#endif
	return 0;
}

static struct notifier_block __clk_cpufreq_notifier_block = {
	.notifier_call = __clk_cpufreq_notifier
};

int __init sci_clock_init(void)
{
	__raw_writel(__raw_readl(REG_PMU_APB_PD_MM_TOP_CFG)
		     & ~(BIT_PD_MM_TOP_FORCE_SHUTDOWN),
		     REG_PMU_APB_PD_MM_TOP_CFG);

	__raw_writel(__raw_readl(REG_PMU_APB_PD_GPU_TOP_CFG)
		     & ~(BIT_PD_GPU_TOP_FORCE_SHUTDOWN),
		     REG_PMU_APB_PD_GPU_TOP_CFG);

	__raw_writel(__raw_readl(REG_AON_APB_APB_EB0) | BIT_MM_EB |
		     BIT_GPU_EB, REG_AON_APB_APB_EB0);

	__raw_writel(__raw_readl(REG_MM_AHB_AHB_EB) | BIT_MM_CKG_EB,
		     REG_MM_AHB_AHB_EB);

	__raw_writel(__raw_readl(REG_MM_AHB_GEN_CKG_CFG)
		     | BIT_MM_MTX_AXI_CKG_EN | BIT_MM_AXI_CKG_EN,
		     REG_MM_AHB_GEN_CKG_CFG);

	__raw_writel(__raw_readl(REG_MM_CLK_MM_AHB_CFG) | 0x3,
		     REG_MM_CLK_MM_AHB_CFG);
#if 1//ndef CONFIG_MACH_SPX15FPGA
	/* register all clock sources */
	{
		struct clk_lookup *cl =
		    (struct clk_lookup *)(&__clkinit_begin + 1);
		struct clk_lookup *cl_end = (struct clk_lookup *)&__clkinit_end;
		int seq = (0 == strcmp(cl->con_id, "ext_26m"));

		debug0("%p (%x) -- %p -- %p (%x)\n",
		       &__clkinit_begin, __clkinit_begin, cl,
		       &__clkinit_end, __clkinit_end);

		if (seq) {
			while (cl < cl_end)
				sci_clk_register(cl++);
		} else {
			while (--cl_end >= cl)
				sci_clk_register(cl_end);
		}
	}

	/* keep track of cpu frequency transitions */
	cpufreq_register_notifier(&__clk_cpufreq_notifier_block,
				  CPUFREQ_TRANSITION_NOTIFIER);
#endif
	return 0;
}

#ifndef CONFIG_OF
//arch_initcall(sci_clock_init);
#else
#include <linux/of.h>
#define clk_info(format, arg...) pr_info("clk: " "@@@%s: " format, __func__, ## arg)
static struct clk **data_clks;
struct sci_clk_onecell_data {
	struct clk **clks;
	unsigned int clk_num;
};
static struct sci_clk_onecell_data clk_data;
static void __init file_clk_data(struct device_node *node)
{
	int idx = 0;
	struct device_node *child;
	data_clks = kzalloc(sizeof(struct clk *) * of_get_child_count(node), GFP_KERNEL);
	if (!data_clks)
		return;

	for_each_child_of_node(node, child) {
		const char *clk_name;
		if (0 == of_property_read_string(child, "clock-output-names", &clk_name)) {
			struct clk * clk = clk_get_sys(NULL, clk_name);
			if (!IS_ERR(clk)) {
				clk_info("[%d]%s\t%p\n", idx, clk_name, child);
				data_clks[idx++] = clk;
				of_clk_add_provider(child, of_clk_src_simple_get, clk);
			}
		}
	}
	clk_data.clks = data_clks;
	clk_data.clk_num = idx;
}
static void __init clock_init(struct device_node *node)
{
	sci_clock_init();
	file_clk_data(node);
	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);
}

#if defined(CONFIG_ARCH_SCX15)
//CLK_OF_DECLARE(scx15_clock, "sprd,scx15-clocks", clock_init);
#else
//CLK_OF_DECLARE(scx35_clock, "sprd,scx35-clocks", clock_init);
#endif
#endif

/* FIXME: clock dump fail when gpu/mm domain power off
*/
//late_initcall_sync(sci_clock_dump);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum Clock Driver");
MODULE_AUTHOR("robot <zhulin.lian@spreadtrum.com>");
MODULE_VERSION("0.2");
