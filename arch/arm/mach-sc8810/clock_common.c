/*
 *  Copyright (C) Spreadtrum Inc.
 *
 *  Common clock control routine for SC88xxX.
 *
 *  Wang Liwei.   levee.wang@spreadtrum.com
 *
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>



#include <mach/clock_common.h>
#include <mach/spinlock_hw_vlx.h>

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);
static LIST_HEAD(root_clks);


static struct clk_functions *arch_clock;



/*---------------------------------------------------------
 *
 *
 * platform-dependent functions.
 *
 *
 *
 *
 *
 ---------------------------------------------------------*/
void clk_print_all(void)
{
	struct clk *p;

	list_for_each_entry(p, &clocks, node) {
		printk("clock: [%s], parent = [%s], usecount = %d, rate = %ld.\n",
				p->pstub->name, p->parent ? (const char *)p->parent->pstub->name  : "NULL",
				p->pstub->usecount, p->rate);
	}
}


void clk_reparent(struct clk *child, struct clk *parent)
{
	list_del_init(&child->sibling);
	if (parent)
		list_add(&child->sibling, &parent->children);
	child->parent = parent;
}

void sc88xx_init_clksel_parent(struct clk *clk)
{
	const struct clksel *clks;
	u32 r, found = 0;

	if (!clk->clksel_reg || !clk->clksel_mask) {
		printk("clock[%s]: parent can't be changed!\n", clk->pstub->name);
		return;
	}

	r = __raw_readl(clk->clksel_reg) & clk->clksel_mask;
	r >>= __ffs(clk->clksel_mask);

	for (clks = clk->clksel; clks->parent && !found; clks++) {
		if (clks->val == r) {
			if (clk->parent != clks->parent) {
				printk("clock: set [%s]'s parent from [%s] to [%s].\n",
					clk->pstub->name, clk->parent->pstub->name, clks->parent->pstub->name);
				clk_reparent(clk, clks->parent);
				if (clk->recalc)
					clk->rate = clk->recalc(clk);
				propagate_rate(clk);
			}
			found = 1;
		}
	}
	if (!found)
		printk("clock: Can find parent for clock [%s].\n", clk->pstub->name);
}


void clk_enable_init_clocks(void)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &clocks, node) {
		if (clkp->pstub->flags & ENABLE_ON_INIT)
			clk_enable(clkp);
	}
}
EXPORT_SYMBOL(clk_enable_init_clocks);

int clk_register(struct clk *clk)
{
	if ((NULL == clk) || IS_ERR(clk))
		return -EINVAL;

	if (clk->node.next || clk->node.prev)
		return 0;

	mutex_lock(&clocks_mutex);

	/* handle sibling. */
	if (clk->parent)
		list_add(&clk->sibling, &clk->parent->children);
	else
		list_add(&clk->sibling, &root_clks);

	/* clock list. */
	// printk("####: clk_register(): register clock [%s].", clk->name);
	list_add(&clk->node, &clocks);

	if (clk->init)
		clk->init(clk);

	mutex_unlock(&clocks_mutex);

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	if ((NULL == clk) || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	list_del(&clk->sibling);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

/* propagate rate to children. */
void propagate_rate(struct clk *tclk)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &tclk->children, sibling) {
		if (clkp->recalc)
			clkp->rate = clkp->recalc(clkp);
		propagate_rate(clkp);
	}
}

void recalculate_root_clocks(void)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &root_clks, sibling) {
		if (clkp->recalc)
			clkp->rate = clkp->recalc(clkp);
		propagate_rate(clkp);
	}
}

void clk_preinit(struct clk *clk)
{
	INIT_LIST_HEAD(&clk->children);
}
int __init clk_init(struct clk_functions *custom_clocks)
{
	if (!custom_clocks) {
		printk("Not valid custom clock\n");
		BUG();
	}
	arch_clock = custom_clocks;
	return 0;
}

/*---------------------------------------------------------
 *
 *
 *  API of clock framework.
 *
 *
 *
 *
 ---------------------------------------------------------*/
unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long flags;
	unsigned long ret = 0;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	ret = clk->rate;
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	unsigned long ret = 0;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_round_rate)
			ret = arch_clock->clk_round_rate(clk, rate);
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	unsigned long ret = -EINVAL;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_set_rate)
			ret = arch_clock->clk_set_rate(clk, rate);
	if (0 == ret) {
		if (clk->recalc)
			clk->rate = clk->recalc(clk);
		propagate_rate(clk);
	}

	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;

}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_divisor(struct clk *clk, int divisor)
{
	unsigned long flags;
	unsigned long ret = -EINVAL;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_set_divisor)
			ret = arch_clock->clk_set_divisor(clk, divisor);
	if (0 == ret) {
		if (clk->recalc)
			clk->rate = clk->recalc(clk);
		propagate_rate(clk);
	}

	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;

}
EXPORT_SYMBOL(clk_set_divisor);

int clk_get_divisor(struct clk *clk)
{
	unsigned long flags;
	unsigned long ret = -EINVAL;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_get_divisor)
			ret = arch_clock->clk_get_divisor(clk);
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;

}
EXPORT_SYMBOL(clk_get_divisor);


int clk_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long flags;
	unsigned long ret = -EINVAL;

	if ((NULL == clk) || IS_ERR(clk))
		return ret;

	if ((NULL == parent) || IS_ERR(parent))
		return ret;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (0 == clk->pstub->usecount) {
		if (arch_clock->clk_set_parent)
			ret = arch_clock->clk_set_parent(clk, parent);
		if (0 == ret) {
			if (clk->recalc)
				clk->rate = clk->recalc(clk);
			propagate_rate(clk);
		}
	}
	else
		ret = -EBUSY;

	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;

}
EXPORT_SYMBOL(clk_set_parent);


struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);


/* 
     only clk_enable() & clk_disable() need to 
     really disable interrupt, other ones only
     need to disable vpic.
*/

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;

	if ((NULL == clk) || IS_ERR(clk))
		return -EINVAL;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_enable)
		ret = arch_clock->clk_enable(clk);
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_enable);


void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if ((NULL == clk) || IS_ERR(clk))
		return;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (clk->pstub->usecount == 0) {
		/*
		printk("Trying to disable clock [%s] with 0 usecount\n",
				clk->pstub->name);
		WARN_ON(1);
		*/
		goto out;
	}
	if (arch_clock->clk_disable)
		arch_clock->clk_disable(clk);
out:
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_disable);
void clk_disable_force(struct clk *clk)
{
	unsigned long flags;

	if ((NULL == clk) || IS_ERR(clk))
		return;

	hw_spin_lock_irqsave(&clockfw_lock, flags);
	if (clk->pstub->usecount == 0) {
        /*
		printk("Trying to disable clock [%s] with 0 usecount\n",
				clk->pstub->name);
		WARN_ON(1);
        */
		goto out;
	}

	clk->pstub->usecount = 1; 
	if (arch_clock->clk_disable)
		arch_clock->clk_disable(clk);
out:
	hw_spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_disable_force);


