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
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/regs_glb.h>
#include <mach/regs_ahb.h>

#include "clock.h"
#include "mach/__clock_tree.h"

const u32 __clkinit2 __clkinit_end = 0xddddeeee;

/* We originally used an mutex here, but some contexts (see resume)
 * are calling functions such as clk_set_parent() with IRQs disabled
 * causing an BUG to be triggered.
 */
DEFINE_SPINLOCK(clocks_lock);

int clk_enable(struct clk *clk)
{
	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;

	clk_enable(clk->parent);

	spin_lock(&clocks_lock);
	if ((clk->usage++) == 0 && clk->enable)
		(clk->enable) (clk, 1);
	spin_unlock(&clocks_lock);
	debug0("clk %p, usage %d\n", clk, clk->usage);
	return 0;
}

EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	if (IS_ERR_OR_NULL(clk))
		return;

	spin_lock(&clocks_lock);
	if ((--clk->usage) == 0 && clk->enable)
		(clk->enable) (clk, 0);
	if (clk->usage < 0)
		dump_stack();
	spin_unlock(&clocks_lock);
	debug0("clk %p, usage %d\n", clk, clk->usage);
	clk_disable(clk->parent);
}

EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	debug0("clk %p, rate %lu\n", clk, IS_ERR_OR_NULL(clk) ? -1 : clk->rate);
	if (IS_ERR_OR_NULL(clk))
		return 0;

	if (clk->rate != 0)
		return clk->rate;

	if (clk->ops != NULL && clk->ops->get_rate != NULL)
		return (clk->ops->get_rate) (clk);

	if (clk->parent != NULL)
		return clk_get_rate(clk->parent);

	return clk->rate;
}

EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (!IS_ERR_OR_NULL(clk) && clk->ops && clk->ops->round_rate)
		return (clk->ops->round_rate) (clk, rate);

	return rate;
}

EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	debug0("clk %p, rate %lu\n", clk, rate);
	if (IS_ERR_OR_NULL(clk) || rate == 0)
		return -EINVAL;

	/* We do not default just do a clk->rate = rate as
	 * the clock may have been made this way by choice.
	 */

	//WARN_ON(clk->ops == NULL);
	//WARN_ON(clk->ops && clk->ops->set_rate == NULL);

	if (clk->ops == NULL || clk->ops->set_rate == NULL)
		return -EINVAL;

	spin_lock(&clocks_lock);
	ret = (clk->ops->set_rate) (clk, rate);
	spin_unlock(&clocks_lock);
	return ret;
}

EXPORT_SYMBOL(clk_set_rate);

struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}

EXPORT_SYMBOL(clk_get_parent);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = 0;
	debug0("clk %p, parent %p\n", clk, parent);
	if (IS_ERR_OR_NULL(clk) || IS_ERR(parent))
		return -EINVAL;

	spin_lock(&clocks_lock);
	if (clk->ops && clk->ops->set_parent)
		ret = (clk->ops->set_parent) (clk, parent);
	spin_unlock(&clocks_lock);
	return ret;
}

EXPORT_SYMBOL(clk_set_parent);

static int sci_clk_enable(struct clk *c, int enable)
{
	debug("clk %p (%s) enb %08x, %s\n", c, c->regs->name,
	      c->regs->enb.reg, enable ? "enable" : "disable");
	if (c->regs->enb.reg & 1)
		enable = !enable;
	(enable) ? sci_glb_set(c->regs->enb.reg & ~1, c->regs->enb.mask)
	    : sci_glb_clr(c->regs->enb.reg & ~1, c->regs->enb.mask);
	return 0;
}

static int sci_clk_is_enable(struct clk *c)
{
	int enable;
	enable = ! !sci_glb_read(c->regs->enb.reg & ~1, c->regs->enb.mask);
	if (c->regs->enb.reg & 1)
		enable = !enable;
	return enable;
}

static int sci_clk_set_rate(struct clk *c, unsigned long rate)
{
	u32 div, div_shift;
	debug("clk %p (%s) set rate %lu\n", c, c->regs->name, rate);
	rate = clk_round_rate(c, rate);
	div = clk_get_rate(c->parent) / rate - 1;	//FIXME:
	div_shift = __ffs(c->regs->div.mask);
	debug0("clk %p (%s) pll div reg %08x, val %08x mask %08x\n", c,
	       c->regs->name, c->regs->div.reg, div << div_shift,
	       c->regs->div.mask);
	sci_glb_write(c->regs->div.reg, div << div_shift, c->regs->div.mask);

	c->rate = 0;		/* FIXME: auto update all children after new rate if need */
	return 0;
}

static unsigned long sci_clk_get_rate(struct clk *c)
{
	u32 div = 0, div_shift;
	unsigned long rate;
	div_shift = __ffs(c->regs->div.mask);
	debug0("clk %p (%s) div reg %08x, shift %u msk %08x\n", c,
	       c->regs->name, c->regs->div.reg, div_shift, c->regs->div.mask);
	rate = clk_get_rate(c->parent);

	if (c->regs->div.reg)
		div = sci_glb_read(c->regs->div.reg,
				   c->regs->div.mask) >> div_shift;
	debug0("clk %p (%s) parent rate %lu, div %u\n", c, c->regs->name, rate,
	       div + 1);
	c->rate = rate = rate / (div + 1);	//FIXME:
	debug("clk %p (%s) get real rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static unsigned long sci_pll_get_refin_rate(struct clk *c)
{
	const unsigned long refin[4] = { 2, 4, 4, 13 };	/* default refin 4M */
	int i = sci_glb_read(c->regs->div.reg, BIT(16) | BIT(17)) >> 16;
	debug0("pll %p (%s) refin %d\n", c, c->regs->name, i);
	return refin[i] * 1000000;
}

static unsigned long sci_pll_get_rate(struct clk *c)
{
	u32 mn = 1, mn_shift;
	unsigned long rate;
	mn_shift = __ffs(c->regs->div.mask);
	debug0("pll %p (%s) mn reg %08x, shift %u msk %08x\n", c, c->regs->name,
	       c->regs->div.reg, mn_shift, c->regs->div.mask);
	rate = clk_get_rate(c->parent);
	if (0 == c->regs->div.reg) ;
	else if (c->regs->div.reg < MAX_DIV) {
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
	c->rate = rate;
	debug("pll %p (%s) get real rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static unsigned long sci_clk_round_rate(struct clk *c, unsigned long rate)
{
	debug0("clk %p (%s) round rate %lu\n", c, c->regs->name, rate);
	return rate;
}

static int sci_clk_set_parent(struct clk *c, struct clk *parent)
{
	int i;
	debug("clk %p (%s) parent %p (%s)\n", c, c->regs->name,
	      parent, parent ? parent->regs->name : 0);

	for (i = 0; i < c->regs->nr_sources; i++) {
		if (c->regs->sources[i] == parent) {
			u32 sel_shift = __ffs(c->regs->sel.mask);
			debug0("pll sel reg %08x, val %08x, msk %08x\n",
			       c->regs->sel.reg, i << sel_shift,
			       c->regs->sel.mask);
			if (c->regs->sel.reg)
				sci_glb_write(c->regs->sel.reg, i << sel_shift,
					      c->regs->sel.mask);
#if defined(CONFIG_DEBUG_FS)
			if (c->parent && c->parent->dent && c->dent
			    && parent->dent) {
				debug0("directory dentry move %s to %s\n",
				       c->parent->regs->name,
				       parent->regs->name);
				debugfs_rename(c->parent->dent, c->dent,
					       parent->dent, c->regs->name);
			}
#endif
			c->parent = parent;
			if (c->ops)
				c->rate = 0;	/* FIXME: auto update clock rate after new parent */
			return 0;
		}
	}

	return -EINVAL;
}

static int sci_clk_get_parent(struct clk *c)
{
	int i = 0;
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
	.get_rate = sci_clk_get_rate,
	.round_rate = sci_clk_round_rate,
	.set_parent = sci_clk_set_parent,
};

static struct clk_ops generic_pll_ops = {
	.set_rate = 0,
	.get_rate = sci_pll_get_rate,
	.round_rate = 0,
	.set_parent = sci_clk_set_parent,
};

/* debugfs support to trace clock tree hierarchy and attributes */
#if defined(CONFIG_DEBUG_FS)
static struct dentry *clk_debugfs_root;
static int __init clk_debugfs_register(struct clk *c)
{
	char name[NAME_MAX], *p = name;
	p += sprintf(p, "%s", c->regs->name);

	if (IS_ERR_OR_NULL((c->dent =
			    debugfs_create_dir(name,
					       c->parent ? c->parent->dent :
					       clk_debugfs_root))))
		goto err_exit;
	if (IS_ERR_OR_NULL(debugfs_create_u8
			   ("usecount", S_IRUGO, c->dent, (u8 *) & c->usage)))
		goto err_exit;
	if (IS_ERR_OR_NULL(debugfs_create_u32
			   ("rate", S_IRUGO, c->dent, (u32 *) & c->rate)))
		goto err_exit;
	return 0;
err_exit:
	if (c->dent)
		debugfs_remove_recursive(c->dent);
	return -ENOMEM;
}
#endif

int __init sci_clk_register(struct clk_lookup *cl)
{
	struct clk *p;
	struct clk *c = cl->clk;

	if (c->ops == NULL) {
		c->ops = &generic_clk_ops;
		if (c->rate)	/* fixed OSC */
			c->ops = NULL;
		else if ((c->regs->div.reg > 0 && c->regs->div.reg < MAX_DIV) ||
			 strstr(c->regs->name, "pll")) {
			c->ops = &generic_pll_ops;
		}
	}

	debug
	    ("clk %p (%s) rate %lu ops %p enb %08x sel %08x div %08x nr_sources %u\n",
	     c, c->regs->name, c->rate, c->ops, c->regs->enb.reg,
	     c->regs->sel.reg, c->regs->div.reg, c->regs->nr_sources);

	if (c->enable == NULL && c->regs->enb.reg) {
		c->enable = sci_clk_enable;
		/* FIXME: dummy update clock usage */
		if (sci_clk_is_enable(c))
			clk_enable(c);
	}

	if (!c->rate) {		/* FIXME: dummy update parent and rate */
		clk_set_parent(c, c->regs->sources[sci_clk_get_parent(c)]);
		//clk_set_rate(c, clk_get_rate(c->parent));
	}

	printk("register clock (%s) rate %lu, parent (%s)\n",
	       c->regs->name, clk_get_rate(c),
	       (p = clk_get_parent(c)) ? p->regs->name : "null");

	spin_lock(&clocks_lock);
	clkdev_add(cl);
	spin_unlock(&clocks_lock);

#if defined(CONFIG_DEBUG_FS)
	clk_debugfs_register(c);
#endif
	return 0;
}

int __init sci_clock_init(void)
{

#if defined(CONFIG_DEBUG_FS)
	clk_debugfs_root = debugfs_create_dir("sprd-clock", NULL);
	if (IS_ERR_OR_NULL(clk_debugfs_root))
		return -ENOMEM;
#endif

	/* register all clock sources */
	{
		static const u32 __clkinit0 __clkinit_begin = 0xeeeebbbb;
		struct clk_lookup *cl =
		    (struct clk_lookup *)(&__clkinit_begin + 1);
		debug0("%p (%x) -- %p -- %p (%x)\n",
		       &__clkinit_begin, __clkinit_begin, cl, &__clkinit_end,
		       __clkinit_end);
		while (cl < (struct clk_lookup *)&__clkinit_end) {
			sci_clk_register(cl);
			cl++;
		}
	}
	return 0;
}

arch_initcall(sci_clock_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum Clock Driver");
MODULE_AUTHOR("robot <zhulin.lian@spreadtrum.com>");
