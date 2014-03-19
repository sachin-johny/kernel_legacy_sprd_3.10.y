/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
 */

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <mach/hardware.h>

//#define CONFIG_CLK_DEBUG

#ifdef CONFIG_OF

#define clk_debug(format, arg...) pr_debug("clk: " "@@@%s: " format, __func__, ## arg)
#define clk_info(format, arg...) pr_info("clk: " "@@@%s: " format, __func__, ## arg)

struct cfg_reg {
	void __iomem *reg;
	u32 msk;
};

struct clk_sprd {
	struct clk_hw hw;
	struct cfg_reg enb;
	u8 flags;
	union {
		unsigned long fixed_rate;
		u32 c_mul;
		struct cfg_reg mul, sel;
		struct clk_hw *mux_hw;
	} m;
	union {
		u32 c_div;
		struct cfg_reg div, pre;
		struct clk_hw *div_hw;
	} d;
};

#define to_clk_sprd(_hw) container_of(_hw, struct clk_sprd, hw)

#define in_range(b, first, len)	((b) >= (first) && (b) <= (first) + (len) - 1)
#define to_range(b, first, base) ( (b) - (first) + (base) )

static inline u32 cfg_reg_p2v(const u32 regp)
{
	if (0) {
	} else if (in_range(regp, SPRD_AHB_PHYS, SPRD_AHB_SIZE)) {
		return to_range(regp, SPRD_AHB_PHYS, SPRD_AHB_BASE);
	} else if (in_range(regp, SPRD_PMU_PHYS, SPRD_PMU_SIZE)) {
		return to_range(regp, SPRD_PMU_PHYS, SPRD_PMU_BASE);
	}
	if (in_range(regp, SPRD_AONAPB_PHYS, SPRD_AONAPB_SIZE)) {
		return to_range(regp, SPRD_AONAPB_PHYS, SPRD_AONAPB_BASE);
	} else if (in_range(regp, SPRD_AONCKG_PHYS, SPRD_AONCKG_SIZE)) {
		return to_range(regp, SPRD_AONCKG_PHYS, SPRD_AONCKG_BASE);
	} else if (in_range(regp, SPRD_GPUCKG_PHYS, SPRD_GPUCKG_SIZE)) {
		return to_range(regp, SPRD_GPUCKG_PHYS, SPRD_GPUCKG_BASE);
	} else if (in_range(regp, SPRD_GPUAPB_PHYS, SPRD_GPUAPB_SIZE)) {
		return to_range(regp, SPRD_GPUAPB_PHYS, SPRD_GPUAPB_BASE);
	} else if (in_range(regp, SPRD_MMCKG_PHYS, SPRD_MMCKG_SIZE)) {
		return to_range(regp, SPRD_MMCKG_PHYS, SPRD_MMCKG_BASE);
	} else if (in_range(regp, SPRD_MMAHB_PHYS, SPRD_MMAHB_SIZE)) {
		return to_range(regp, SPRD_MMAHB_PHYS, SPRD_MMAHB_BASE);
	} else if (in_range(regp, SPRD_APBREG_PHYS, SPRD_APBREG_SIZE)) {
		return to_range(regp, SPRD_APBREG_PHYS, SPRD_APBREG_BASE);
	} else if (in_range(regp, SPRD_APBCKG_PHYS, SPRD_APBCKG_SIZE)) {
		return to_range(regp, SPRD_APBCKG_PHYS, SPRD_APBCKG_BASE);
	}

	WARN(1, "regp = %u\n", regp);
	return 0;
}

static inline void of_read_reg(struct cfg_reg *cfg, const __be32 * cell)
{
	if (!cell)
		return;
	cfg->reg = (void *)cfg_reg_p2v(be32_to_cpu(*(cell++)));
	cfg->msk = be32_to_cpu(*(cell++));
}

static inline void __glbreg_setclr(struct clk_hw *hw, void *reg, u32 msk,
				   int is_set)
{
	if (!reg)
		return;

	clk_debug("%s %s %p[%x]\n", __clk_get_name(hw->clk),
		  (is_set) ? "SET" : "CLR", reg, (u32) msk);

	if (is_set)
		__raw_writel(msk, (void *)((u32) (reg) + 0x1000));
	else
		__raw_writel(msk, (void *)((u32) (reg) + 0x2000));
}

static int sprd_clk_prepare(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	int set = ! !(c->flags & CLK_GATE_SET_TO_DISABLE);

	__glbreg_setclr(hw, c->d.pre.reg, (u32) c->d.pre.msk, set ^ 1);
	return 0;
}

static void sprd_clk_unprepare(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	int set = ! !(c->flags & CLK_GATE_SET_TO_DISABLE);
	__glbreg_setclr(hw, c->d.pre.reg, (u32) c->d.pre.msk, set ^ 0);
}

static int sprd_clk_is_prepared(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	int ret, set = ! !(c->flags & CLK_GATE_SET_TO_DISABLE);

	if (!c->d.pre.reg)
		return 0;

	/* if a set bit prepare this gate, flip it before masking */
	ret = ! !(__raw_readl(c->d.pre.reg) & BIT(c->d.pre.msk));
	return set ^ ret;
}

static int sprd_clk_enable(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	__glbreg_setclr(hw, c->enb.reg, (u32) c->enb.msk, 1);
	return 0;
}

static void sprd_clk_disable(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	__glbreg_setclr(hw, c->enb.reg, (u32) c->enb.msk, 0);
}

static int sprd_clk_is_enable(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	int ret = ! !(__raw_readl(c->enb.reg) & BIT(c->enb.msk));
	return ret;
}

static unsigned long sprd_clk_fixed_pll_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	return to_clk_sprd(hw)->m.fixed_rate;
}

#define BITS_MPLL_REFIN(_X_)                              ( (_X_) << 24 & (BIT(24)|BIT(25)) )
static inline unsigned int __pll_get_refin_rate(void *reg)
{
	const unsigned long refin[4] = { 2000000, 4000000, 13000000, 26000000 };
	u32 i, msk = BITS_MPLL_REFIN(-1);
	i = (__raw_readl(reg) & msk) >> __ffs(msk);
	return refin[i];
}

static unsigned long sprd_clk_adjustable_pll_recalc_rate(struct clk_hw *hw,
							 unsigned long
							 parent_rate)
{
	struct clk_sprd *pll = to_clk_sprd(hw);
	unsigned int refin, mn, rate;

	refin = __pll_get_refin_rate(pll->m.mul.reg);
	mn = (__raw_readl(pll->m.mul.reg) & pll->m.mul.msk) >> __ffs(pll->m.mul.
								     msk);

	rate = refin * mn;
	clk_debug("rate %u, refin %u, mn %u\n", rate, refin, mn);
	return (unsigned long)rate;
}

static long sprd_clk_adjustable_pll_round_rate(struct clk_hw *hw,
					       unsigned long rate,
					       unsigned long *prate)
{
	//struct clk_sprd *pll = to_clk_sprd(hw);
	clk_debug("rate %lu, %lu\n", rate, *prate);
	return rate;
}

int sci_glb_write(u32 reg, u32 val, u32 msk);

static int sprd_clk_adjustable_pll_set_rate(struct clk_hw *hw,
					    unsigned long rate,
					    unsigned long parent_rate)
{
	struct clk_sprd *pll = to_clk_sprd(hw);
	u32 refin, mn;
	refin = __pll_get_refin_rate(pll->m.mul.reg);
	mn = rate / refin;
	clk_debug("rate %u, refin %u, mn %u\n", (u32) rate, refin, mn);
	if (mn <= pll->m.mul.msk >> __ffs(pll->m.mul.msk)) {
		sci_glb_write((u32) pll->m.mul.reg, mn << __ffs(pll->m.mul.msk),
			      pll->m.mul.msk);
	}
	return 0;
}

/* FIXME:
 * Inherit from clk-mux.c
 */
static u8 sprd_clk_mux_get_parent(struct clk_hw *hw)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	if (!c->m.mux_hw->clk)
		c->m.mux_hw->clk = c->hw.clk;
	clk_debug("%s\n", __clk_get_name(hw->clk));
	return clk_mux_ops.get_parent(c->m.mux_hw);
}

static int sprd_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	if (!c->m.mux_hw->clk)
		c->m.mux_hw->clk = c->hw.clk;
	clk_debug("%s %d\n", __clk_get_name(hw->clk), (u32) index);
	return clk_mux_ops.set_parent(c->m.mux_hw, index);
}

/* FIXME:
 * Inherit from clk-divider.c
 */

static unsigned long sprd_clk_divider_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	if (!c->d.div_hw->clk)
		c->d.div_hw->clk = c->hw.clk;
	clk_debug("%s %lu\n", __clk_get_name(hw->clk), parent_rate);
	return clk_divider_ops.recalc_rate(c->d.div_hw, parent_rate);
}

static long sprd_clk_divider_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	if (!c->d.div_hw->clk)
		c->d.div_hw->clk = c->hw.clk;
	clk_debug("%s rate %lu %lu\n", __clk_get_name(hw->clk), rate,
		  (prate) ? *prate : 0);
	return clk_divider_ops.round_rate(c->d.div_hw, rate, prate);
}

static int sprd_clk_divider_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct clk_sprd *c = to_clk_sprd(hw);
	if (!c->d.div_hw->clk)
		c->d.div_hw->clk = c->hw.clk;
	clk_debug("%s %lu %lu\n", __clk_get_name(hw->clk), rate, parent_rate);
	return clk_divider_ops.set_rate(c->d.div_hw, rate, parent_rate);
}

const struct clk_ops sprd_clk_fixed_pll_ops = {
	.prepare = sprd_clk_prepare,
	.unprepare = sprd_clk_unprepare,
	.is_prepared = sprd_clk_is_prepared,
	.recalc_rate = sprd_clk_fixed_pll_recalc_rate,
};

const struct clk_ops sprd_clk_adjustable_pll_ops = {
	.prepare = sprd_clk_prepare,
	.unprepare = sprd_clk_unprepare,
	.round_rate = sprd_clk_adjustable_pll_round_rate,
	.set_rate = sprd_clk_adjustable_pll_set_rate,
	.recalc_rate = sprd_clk_adjustable_pll_recalc_rate,
};

const struct clk_ops sprd_clk_gate_ops = {
	.prepare = sprd_clk_prepare,
	.unprepare = sprd_clk_unprepare,
	.enable = sprd_clk_enable,
	.disable = sprd_clk_disable,
	.is_enabled = sprd_clk_is_enable,
};

const struct clk_ops sprd_clk_mux_ops = {
	.enable = sprd_clk_enable,
	.disable = sprd_clk_disable,
	.get_parent = sprd_clk_mux_get_parent,
	.set_parent = sprd_clk_mux_set_parent,
};

const struct clk_ops sprd_clk_divider_ops = {
	.enable = sprd_clk_enable,
	.disable = sprd_clk_disable,
	.recalc_rate = sprd_clk_divider_recalc_rate,
	.round_rate = sprd_clk_divider_round_rate,
	.set_rate = sprd_clk_divider_set_rate,
};

const struct clk_ops sprd_clk_composite_ops = {
	.enable = sprd_clk_enable,
	.disable = sprd_clk_disable,
	.get_parent = sprd_clk_mux_get_parent,
	.set_parent = sprd_clk_mux_set_parent,
	.recalc_rate = sprd_clk_divider_recalc_rate,
	.round_rate = sprd_clk_divider_round_rate,
	.set_rate = sprd_clk_divider_set_rate,
};

static void __init file_clk_data(struct clk *clk, const char *clk_name);
static void __init sprd_clk_register(struct device *dev,
				     struct device_node *node,
				     struct clk_sprd *c,
				     struct clk_init_data *init);

/**
 * of_sprd_fixed_clk_setup() - Setup function for simple sprd fixed rate clock
 */
void of_sprd_fixed_clk_setup(struct device_node *node)
{
	struct clk *clk = NULL;
	const char *clk_name = node->name;
	u32 rate;

	if (of_property_read_u32(node, "clock-frequency", &rate))
		return;

	of_property_read_string(node, "clock-output-names", &clk_name);

#ifndef CONFIG_CLK_DEBUG
	clk = clk_register_fixed_rate(NULL, clk_name, NULL, CLK_IS_ROOT, rate);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, 0);
		file_clk_data(clk, clk_name);
	}
#endif
	clk_debug("[%p]%s fixed-rate %d\n", clk, clk_name, rate);
}

/**
 * of_sprd_fixed_factor_clk_setup() - Setup function for simple sprd fixed factor clock
 */
void __init of_sprd_fixed_factor_clk_setup(struct device_node *node)
{
	struct clk *clk = NULL;
	const char *clk_name = node->name;
	const char *parent_name;
	u32 div, mult;

	if (of_property_read_u32(node, "clock-div", &div)) {
		pr_err
		    ("%s Fixed factor clock <%s> must have a clock-div property\n",
		     __func__, node->name);
		return;
	}

	if (of_property_read_u32(node, "clock-mult", &mult)) {
		pr_err
		    ("%s Fixed factor clock <%s> must have a clokc-mult property\n",
		     __func__, node->name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);

#ifndef CONFIG_CLK_DEBUG
	clk = clk_register_fixed_factor(NULL, clk_name, parent_name, 0,
					mult, div);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, 0);
		file_clk_data(clk, clk_name);
	}
#endif
	clk_debug("[%p]%s parent %s mult %d div %d\n", clk, clk_name,
		  parent_name, mult, div);
}

/**
 * of_sprd_fixed_pll_clk_setup() - Setup function for simple sprd fixed-pll clock
 */
static void __init of_sprd_fixed_pll_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	struct clk *clk = NULL;
	const char *clk_name = node->name;
	struct clk_init_data init = {
		.name = clk_name,
		.ops = &sprd_clk_fixed_pll_ops,
		.flags = CLK_IS_ROOT,
		.num_parents = 0,
	};
	u32 rate;
	const __be32 *prereg;

	if (of_property_read_u32(node, "clock-frequency", &rate))
		return;

	prereg = of_get_address(node, 0, NULL, NULL);
	if (!prereg) {
		pr_err
		    ("%s Fixed pll clock <%s> must have a prepare reg property\n",
		     __func__, node->name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	/* allocate fixed-pll clock */
	c = kzalloc(sizeof(struct clk_sprd), GFP_KERNEL);
	if (!c) {
		pr_err("%s: could not allocate sprd fixed-pll clk\n", __func__);
		return;
	}

	/* struct clk_fixed_rate assignments */
	c->m.fixed_rate = rate;

	/* set fixed pll regs */
	of_read_reg(&c->d.pre, prereg);

	sprd_clk_register(NULL, node, c, &init);

	clk_debug("[%p]%s fixed-pll-rate %d, prepare %p[%x]\n", clk, clk_name,
		  rate, c->d.pre.reg, c->d.pre.msk);
}

/**
 * of_sprd_adjustable_pll_clk_setup() - Setup function for simple sprd adjustable-pll clock
 */
static void __init of_sprd_adjustable_pll_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	struct clk *clk = NULL;
	const char *clk_name = node->name;
	struct clk_init_data init = {
		.name = clk_name,
		.ops = &sprd_clk_adjustable_pll_ops,
		.flags = CLK_IS_ROOT,
		.num_parents = 0,
	};
	const __be32 *mulreg, *prereg;

	mulreg = of_get_address(node, 0, NULL, NULL);
	if (!mulreg) {
		pr_err
		    ("%s adjustable clock <%s> must have a mul reg property\n",
		     __func__, node->name);
		return;
	}

	prereg = of_get_address(node, 1, NULL, NULL);
	if (!prereg) {
		pr_err
		    ("%s adjustable pll clock <%s> must have a prepare reg property\n",
		     __func__, node->name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	/* allocate adjustable-pll clock */
	c = kzalloc(sizeof(struct clk_sprd), GFP_KERNEL);
	if (!c) {
		pr_err("%s: could not allocate adjustable-pll clk\n", __func__);
		return;
	}

	/* struct clk_adjustable_pll assignments */
	of_read_reg(&c->m.mul, mulreg);
	of_read_reg(&c->d.pre, prereg);

	sprd_clk_register(NULL, node, c, &init);

	if (prereg)
		clk_debug("[%p]%s mul %p[%x], prepare %p[%x]\n", clk, clk_name,
			  c->m.mul.reg, c->m.mul.msk, c->d.pre.reg,
			  c->d.pre.msk);
	else
		clk_debug("[%p]%s mul %p[%x]\n", clk, clk_name,
			  c->m.mul.reg, c->m.mul.msk);

}

/**
 * of_sprd_gate_clk_setup() - Setup function for simple gate rate clock
 */
static void __init of_sprd_gate_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	struct clk *clk = NULL;
	const char *clk_name = node->name;
	const char *parent_name;
	struct clk_init_data init = {
		.name = clk_name,
		.ops = &sprd_clk_gate_ops,
	};
	const __be32 *enbreg, *prereg;

	enbreg = of_get_address(node, 0, NULL, NULL);
	if (!enbreg) {
		pr_err
		    ("%s gate clock <%s> must have a reg-enb property\n",
		     __func__, node->name);
		return;
	}

	prereg = of_get_address(node, 1, NULL, NULL);
	if (!prereg) {
		//prepare reg is optional
	}

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);

	/* allocate the gate */
	c = kzalloc(sizeof(struct clk_sprd), GFP_KERNEL);
	if (!c) {
		pr_err("%s: could not allocate gated clk\n", __func__);
		return;
	}

	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_gate assignments */
	of_read_reg(&c->enb, enbreg);
	of_read_reg(&c->d.pre, prereg);
	/* Flags:
	 * CLK_GATE_SET_TO_DISABLE - by default this clock sets the bit at bit_idx to
	 *  enable the clock.  Setting this flag does the opposite: setting the bit
	 *  disable the clock and clearing it enables the clock
	 */
	if ((u32) c->d.pre.reg & 1) {
		*(u32 *) & c->d.pre.reg &= ~3;
		c->flags |= CLK_GATE_SET_TO_DISABLE;
	}

	sprd_clk_register(NULL, node, c, &init);

	if (prereg)
		clk_debug("[%p]%s enable %p[%x] prepare %p[%x]\n", clk,
			  clk_name, c->enb.reg, c->enb.msk, c->d.pre.reg,
			  c->d.pre.msk);
	else
		clk_debug("[%p]%s enable %p[%x]\n", clk, clk_name,
			  c->enb.reg, c->enb.msk);
}

/**
 * of_sprd_composite_clk_setup() - Setup function for simple composite clock
 */
static struct clk_sprd *__init __of_sprd_composite_clk_setup(struct device_node
							     *node, int has_mux,
							     int has_div)
{
	struct clk_sprd *c;
	const char *clk_name = node->name;
	struct clk_init_data init = {
		.name = clk_name,
	};
	const __be32 *selreg = NULL, *divreg = NULL, *enbreg;
	int idx = 0;

	if (has_mux)
		selreg = of_get_address(node, idx++, NULL, NULL);

	if (has_div)
		divreg = of_get_address(node, idx++, NULL, NULL);

	enbreg = of_get_address(node, idx++, NULL, NULL);

	of_property_read_string(node, "clock-output-names", &clk_name);

	/* allocate the clock */
	c = kzalloc(sizeof(struct clk_sprd), GFP_KERNEL);
	if (!c) {
		pr_err("%s: could not allocate sprd clk\n", __func__);
		return NULL;
	}

	/* struct clk_sprd assignments */
	of_read_reg(&c->enb, enbreg);

	if (selreg) {
		int i, num_parents;
		struct clk_mux *mux;

		of_read_reg(&c->m.sel, selreg);

		init.ops = &sprd_clk_mux_ops,
		    /* FIXME: Retrieve the phandle list property */
		    of_get_property(node, "clocks", &num_parents);
		init.num_parents = (u8) num_parents / 4;
		mux =
		    kzalloc(sizeof(struct clk_mux) +
			    sizeof(const char *) * init.num_parents,
			    GFP_KERNEL);
		init.parent_names = (const char **)&mux[1];
		clk_debug("parents : ");
		for (i = 0; i < init.num_parents; i++) {
			init.parent_names[i] = of_clk_get_parent_name(node, i);
			pr_debug("[%d]%s ", i, init.parent_names[i]);
		}
		pr_debug("\n");

		/* struct clk_mux assignments */
		mux->reg = c->m.sel.reg;
		mux->shift = __ffs(c->m.sel.msk);
		mux->mask = c->m.sel.msk >> mux->shift;
		mux->flags = 0;
		mux->lock = 0;
		mux->table = 0;
		c->m.mux_hw = &mux->hw;	/* FIXME: should not use m.sel.reg at now */
		clk_debug("mux %u, %x\n", (u32) mux->shift, mux->mask);
	}

	if (divreg) {
		const char *parent_name;
		struct clk_divider *div;

		of_read_reg(&c->d.div, divreg);

		div = kzalloc(sizeof(struct clk_divider), GFP_KERNEL);
		init.ops = &sprd_clk_divider_ops;
		if (!init.num_parents) {
			parent_name = of_clk_get_parent_name(node, 0);;
			init.parent_names = (parent_name ? &parent_name : NULL);
			init.num_parents = (parent_name ? 1 : 0);
			clk_debug("parent %s\n", parent_name);
		}

		/* struct clk_divider assignments */
		div->reg = c->d.div.reg;
		div->shift = __ffs(c->d.div.msk);
		div->width = __ffs(~(c->d.div.msk >> div->shift));
		div->flags = 0;
		div->lock = 0;
		div->table = 0;
		c->d.div_hw = &div->hw;	/* FIXME: should not use d.div.reg at now */
		clk_debug("div %u, %u\n", (u32) div->shift, (u32) div->width);
	}

	if (divreg && selreg) {
		init.ops = &sprd_clk_composite_ops;
	}

	sprd_clk_register(NULL, node, c, &init);
	return c;

}

static void __init of_sprd_muxed_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	c = __of_sprd_composite_clk_setup(node, 1, 0);
	if (!c)
		return;
	clk_debug("[%p]%s select %p[%x] enable %p[%x]\n", c->hw.clk,
		  __clk_get_name(c->hw.clk), c->m.sel.reg, c->m.sel.msk,
		  c->enb.reg, c->enb.msk);
}

static void __init of_sprd_divider_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	c = __of_sprd_composite_clk_setup(node, 0, 1);
	if (!c)
		return;
	clk_debug("[%p]%s divider %p[%x] enable %p[%x]\n", c->hw.clk,
		  __clk_get_name(c->hw.clk), c->d.div.reg,
		  c->d.div.msk, c->enb.reg, c->enb.msk);
}

static void __init of_sprd_composite_clk_setup(struct device_node *node)
{
	struct clk_sprd *c;
	c = __of_sprd_composite_clk_setup(node, 1, 1);
	if (!c)
		return;
	clk_debug("[%p]%s select %p[%x] divider %p[%x] enable %p[%x]\n",
		  c->hw.clk, __clk_get_name(c->hw.clk), c->m.sel.reg,
		  c->m.sel.msk, c->d.div.reg, c->d.div.msk, c->enb.reg,
		  c->enb.msk);
}

/* register the clock */
static struct clk_onecell_data clk_data;
static void __init init_clk_data(struct device_node *node)
{
	struct clk **clks;
	int num = of_get_child_count(node);
	clks = kzalloc(sizeof(struct clk *) * num, GFP_KERNEL);
	if (!clks)
		return;

	clk_data.clks = clks;
	clk_data.clk_num = num;
}

static void __init file_clk_data(struct clk *clk, const char *clk_name)
{
	static int clk_idx = 0;
	/* FIXME: Add oncell clock provider, be careful not to mistake the clock index */
	if (clk_data.clks) {
		clk = clk_get_sys(NULL, clk_name);
		if (!IS_ERR(clk)) {
			clk_info("[%d]%s\n", clk_idx, clk_name);
			clk_data.clks[clk_idx++] = clk;
		}
	}
}

static void __init sprd_clk_register(struct device *dev,
				     struct device_node *node,
				     struct clk_sprd *c,
				     struct clk_init_data *init)
{
#ifndef CONFIG_CLK_DEBUG
	struct clk *clk;
	const char *clk_name = init->name;

	c->hw.init = init;
	clk = clk_register(dev, &c->hw);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, 0);
		file_clk_data(clk, clk_name);
	}
#endif
}

static void __init sprd_clocks_init(struct device_node *node)
{
	init_clk_data(node);
	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(scx15_clock, "sprd,scx15-clocks", sprd_clocks_init);
CLK_OF_DECLARE(scx35_clock, "sprd,scx35-clocks", sprd_clocks_init);
CLK_OF_DECLARE(fixed_clock, "sprd,fixed-clock", of_sprd_fixed_clk_setup);
CLK_OF_DECLARE(fixed_factor_clock, "sprd,fixed-factor-clock",
	       of_sprd_fixed_factor_clk_setup);
CLK_OF_DECLARE(fixed_pll_clock, "sprd,fixed-pll-clock",
	       of_sprd_fixed_pll_clk_setup);
CLK_OF_DECLARE(adjustable_pll_clock, "sprd,adjustable-pll-clock",
	       of_sprd_adjustable_pll_clk_setup);
CLK_OF_DECLARE(gate_clock, "sprd,gate-clock", of_sprd_gate_clk_setup);
CLK_OF_DECLARE(muxed_clock, "sprd,muxed-clock", of_sprd_muxed_clk_setup);
CLK_OF_DECLARE(divider_clock, "sprd,divider-clock", of_sprd_divider_clk_setup);
CLK_OF_DECLARE(composite_clock, "sprd,composite-dev-clock",
	       of_sprd_composite_clk_setup);
#endif
