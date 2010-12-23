/*
 *    Copyright (C)  2010 Spreadtrum Inc. 
 *    
 *    Clock framework for SC8800G2.
 *
 *    Wang Liwei.   <levee.wang@spreadtrum.com>
 *
 *
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/bitops.h>

#include <asm/clkdev.h>

#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>



struct sc88xx_clk {
	u32 cpu;
	struct clk_lookup lk;
};

#define	CLK(dev, con, ck , cp)	\
	{	\
		.cpu = cp,	\
		.lk = {	\
			.dev_id = dev,	\
			.con_id = con,	\
			.clk = ck,	\
		},	\
	}

#define 	CK_SC8800G2	(0x1UL << 0)


static int clkll_enable_null(struct clk *clk)
{
	return 0;
}

static void clkll_disable_null(struct clk *clk)
{
}



const struct clkops clkops_null = {
	.enable = clkll_enable_null,
	.disable = clkll_disable_null,

};


static int sc88xx_clk_enable_generic(struct clk *clk)
{
	u32 v;

	if (unlikely(clk->enable_reg == NULL)) {
		printk("clock: clock [%s]'s enable_reg is NULL\n", clk->name);
		return -EINVAL;
	}

	v = __raw_readl(clk->enable_reg);
	if (clk->flags & INVERT_ENABLE)
		v &= ~(1 << clk->enable_bit);
	else
		v |= (1 << clk->enable_bit);

	__raw_writel(v, clk->enable_reg);
	v = __raw_readl(clk->enable_reg);

	return 0;
}

static void sc88xx_clk_disable_generic(struct clk *clk)
{
	u32 v;

	if (unlikely(clk->enable_reg == NULL)) {
		printk("clock: clock [%s]'s enable_reg is NULL\n", clk->name);
		return;
	}

	v = __raw_readl(clk->enable_reg);
	if (clk->flags & INVERT_ENABLE)
		v |= (1 << clk->enable_bit);
	else
		v &= ~(1 << clk->enable_bit);

	__raw_writel(v, clk->enable_reg);
	v = __raw_readl(clk->enable_reg);
}


const struct clkops sc88xx_clk_ops_generic = {
	.enable = sc88xx_clk_enable_generic,
	.disable = sc88xx_clk_disable_generic,
};

static struct clk *sclk;

/* first level. */
static struct clk ext_32k = {
	.name = "ext_32k",
	.ops = &clkops_null,
	.rate = 32000,
	.flags = RATE_FIXED,
	.clkdm_name = "ext_clkdm",
};

static struct clk ext_26m = {
	.name = "ext_26m",
	.ops = &clkops_null,
	.rate = 26000000,
	.flags = RATE_FIXED,
	.clkdm_name = "ext_clkdm",
};

/* second level. */
static struct clk mpll_ck = {
	.name = "mpll_ck",
	.ops = &clkops_null,
	.parent = &ext_26m,
	.clkdm_name = "pll_clkdm",
	.recalc = &sc8800g2_mpllcore_recalc,
	.set_rate = &sc8800g2_reprogram_mpllcore,
};

static struct clk tdpll_ck = {
	.name = "tdpll_ck",
	.rate = 768000000,
	.flags = RATE_FIXED,
	.ops = &clkops_null,
	.parent = &ext_26m,
	.clkdm_name = "pll_clkdm",
};

/* third level.*/
static struct clk l3_256m = {
	.name = "l3_256m",
	.flags = RATE_FIXED,
	.rate = 256000000,
	.ops = &clkops_null,
	.parent = &tdpll_ck,
	.clkdm_name = "l3_clkdm",
};

static struct clk l3_192m = {
	.name = "l3_192m",
	.flags = RATE_FIXED,
	.rate = 192000000,
	.ops = &clkops_null,
	.parent = &tdpll_ck,
	.clkdm_name = "l3_clkdm",
};

static struct clk l3_153m600k = {
	.name = "l3_153m600k",
	.flags = RATE_FIXED,
	.rate = 153600000,
	.ops = &clkops_null,
	.parent = &tdpll_ck,
	.clkdm_name = "l3_clkdm",
};

/* derived from l3_256m clock. */
static struct clk clk_128m = {
	.name = "clk_128m",
	.flags = RATE_FIXED,
	.rate = 128000000,
	.ops = &clkops_null,
	.parent = &l3_256m,
	.clkdm_name = "from_l3_256m",
};

static struct clk clk_64m = {
	.name = "clk_64m",
	.flags = RATE_FIXED,
	.rate = 64000000,
	.ops = &clkops_null,
	.parent = &clk_128m,
	.clkdm_name = "from_l3_256m",
};

/* derived from l3_192m clock. */
static struct clk clk_96m = {
	.name = "clk_96m",
	.flags = RATE_FIXED,
	.rate = 96000000,
	.ops = &clkops_null,
	.parent = &l3_192m,
	.clkdm_name = "from_l3_192m",
};

static struct clk clk_48m = {
	.name = "clk_48m",
	.flags = RATE_FIXED,
	.rate = 48000000,
	.ops = &clkops_null,
	.parent = &clk_96m,
	.clkdm_name = "from_l3_192m",
};

static struct clk clk_24m = {
	.name = "clk_24m",
	.flags = RATE_FIXED,
	.rate = 24000000,
	.ops = &clkops_null,
	.parent = &clk_48m,
	.clkdm_name = "from_l3_192m",
};

static struct clk clk_12m = {
	.name = "clk_12m",
	.flags = RATE_FIXED,
	.rate = 12000000,
	.ops = &clkops_null,
	.parent = &clk_24m,
	.clkdm_name = "from_l3_192m",
};

/* derived from l3_153m600k clock. */
static struct clk clk_76m800k = {
	.name = "clk_76m800k",
	.flags = RATE_FIXED,
	.rate = 76800000,
	.ops = &clkops_null,
	.parent = &l3_153m600k,
	.clkdm_name = "from_l3_153m600k",
};

static struct clk clk_51m200k = {
	.name = "clk_51m200k",
	.flags = RATE_FIXED,
	.rate = 51200000,
	.ops = &clkops_null,
	.parent = &l3_153m600k,
	.clkdm_name = "from_l3_153m600k",
};

static struct clk clk_10m240k = {
	.name = "clk_10m240k",
	.flags = RATE_FIXED,
	.rate = 10240000,
	.ops = &clkops_null,
	.parent = &clk_51m200k,
	.clkdm_name = "from_l3_153m600k",
};

static struct clk clk_5m120k = {
	.name = "clk_5m120k",
	.flags = RATE_FIXED,
	.rate = 5120000,
	.ops = &clkops_null,
	.parent = &clk_10m240k,
	.clkdm_name = "from_l3_153m600k",
};


/* derived from ext_26m clock. */
static struct clk clk_13m = {
	.name = "clk_13m",
	.flags = RATE_FIXED,
	.rate = 13000000,
	.ops = &clkops_null,
	.parent = &ext_26m,
	.clkdm_name = "from_ext_26m",
};

static struct clk clk_6m500k = {
	.name = "clk_6m500k",
	.flags = RATE_FIXED,
	.rate = 65000000,
	.ops = &clkops_null,
	.parent = &clk_13m,
	.clkdm_name = "from_ext_26m",
};


/* for source-selectable clock. */
static const struct clksel_rate ccir_mclk_rates_clk_48m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 2, .val = 1, .flags = RATE_IN_SC8800G2},
		{.div = 3, .val = 2, .flags = RATE_IN_SC8800G2},
		{.div = 4, .val = 3, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel_rate ccir_mclk_rates_clk_76m800k[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 2, .val = 1, .flags = RATE_IN_SC8800G2},
		{.div = 3, .val = 2, .flags = RATE_IN_SC8800G2},
		{.div = 4, .val = 3, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel_rate ccir_mclk_rates_ext_26m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 2, .val = 1, .flags = RATE_IN_SC8800G2},
		{.div = 3, .val = 2, .flags = RATE_IN_SC8800G2},
		{.div = 4, .val = 3, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel ccir_mclk_clksel[] = {
		{.parent = &clk_48m,		.val = 0,	.rates = ccir_mclk_rates_clk_48m},
		{.parent = &clk_76m800k,	.val = 1,	.rates = ccir_mclk_rates_clk_76m800k},
		{.parent = &ext_26m,		.val = 2,	.rates = ccir_mclk_rates_ext_26m},
		{.parent = &ext_26m,		.val = 3,	.rates = ccir_mclk_rates_ext_26m},
		{.parent = NULL}
};

static struct clk ccir_mclk = {
	.name = "ccir_mclk",
	.ops = &sc88xx_clk_ops_generic,
	.parent = &clk_48m,
	.clkdm_name = "top_module",

	.recalc = &sc88xx_recalc_generic,
	.set_rate = &sc88xx_set_rate_generic,

	.init = &sc88xx_init_clksel_parent,

	.round_rate = &sc88xx_clksel_round_rate,

	.clksel = ccir_mclk_clksel,
	.clksel_reg = IOMEM(PLL_SCR),
	.clksel_mask = CCIR_MCLK_CLKSEL_MASK,

	.enable_reg = IOMEM(GEN0),
	.enable_bit = CCIR_MCLK_EN_SHIFT,

	.clkdiv_reg = IOMEM(GEN3),
	.clkdiv_mask = CCIR_MCLK_CLKDIV_MASK,
};

static struct clk clk_ccir = {
	.name = "clk_ccir",
	.ops = &sc88xx_clk_ops_generic,
	.parent = NULL,
	.clkdm_name = "top_module",
};


static const struct clksel_rate clk_dcam_rates_clk_96m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel_rate clk_dcam_rates_clk_64m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel_rate clk_dcam_rates_clk_48m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel_rate clk_dcam_rates_clk_26m[] = {
		{.div = 1, .val = 0, .flags = RATE_IN_SC8800G2},
		{.div = 0},
};

static const struct clksel clk_dcam_clksel[] = {
		{.parent = &clk_96m,		.val = 0,	.rates = clk_dcam_rates_clk_96m},
		{.parent = &clk_64m,		.val = 1,	.rates = clk_dcam_rates_clk_64m},
		{.parent = &clk_48m,		.val = 2,	.rates = clk_dcam_rates_clk_48m},
		{.parent = &ext_26m,		.val = 3,	.rates = clk_dcam_rates_clk_26m},
		{.parent = NULL}
};

static struct clk clk_dcam = {
	.name = "clk_dcam",
	.ops = &sc88xx_clk_ops_generic,
	.parent = &clk_96m,
	.clkdm_name = "peripheral",

	.recalc = &sc88xx_recalc_generic,
	/*
	.set_rate = &sc88xx_set_rate_generic,
	*/
	.init = &sc88xx_init_clksel_parent,

	.round_rate = &sc88xx_clksel_round_rate,

	.clksel = clk_dcam_clksel,
	.clksel_reg = IOMEM(PLL_SCR),
	.clksel_mask = CLK_DCAM_CLKSEL_MASK,

	.enable_reg = IOMEM(AHB_CTL0),
	.enable_bit = CLK_DCAM_EN_SHIFT,
	/*
	.clkdiv_reg = IOMEM(GEN3),
	.clkdiv_mask = CCIR_MCLK_CLKDIV_MASK,
	*/
};

static const struct clksel clk_vsp_clksel[] = {
		{.parent = &clk_96m,		.val = 0,	.rates = clk_dcam_rates_clk_96m},
		{.parent = &clk_64m,		.val = 1,	.rates = clk_dcam_rates_clk_64m},
		{.parent = &clk_48m,		.val = 2,	.rates = clk_dcam_rates_clk_48m},
		{.parent = &ext_26m,		.val = 3,	.rates = clk_dcam_rates_clk_26m},
		{.parent = NULL}
};

static struct clk clk_vsp = {
	.name = "clk_vsp",
	.ops = &sc88xx_clk_ops_generic,
	.parent = &clk_96m,
	.clkdm_name = "peripheral",

	.recalc = &sc88xx_recalc_generic,
	/*
	.set_rate = &sc88xx_set_rate_generic,
	*/
	.init = &sc88xx_init_clksel_parent,

	.round_rate = &sc88xx_clksel_round_rate,

	.clksel = clk_dcam_clksel,
	.clksel_reg = IOMEM(PLL_SCR),
	.clksel_mask = CLK_VSP_CLKSEL_MASK,

	.enable_reg = IOMEM(AHB_CTL0),
	.enable_bit = CLK_VSP_EN_SHIFT,
	/*
	.clkdiv_reg = IOMEM(GEN3),
	.clkdiv_mask = CCIR_MCLK_CLKDIV_MASK,
	*/
};

static struct sc88xx_clk sc8800g2_clks[] = {
	/* 1. first level: external input clock. */
	CLK(NULL, "ext_32k", &ext_32k, CK_SC8800G2),
	CLK(NULL, "ext_26m", &ext_26m, CK_SC8800G2),

	/* 2. second level: PLL output clock. */
	CLK(NULL, "mpll_ck", &mpll_ck, CK_SC8800G2),
	CLK(NULL, "tdpll_ck", &tdpll_ck, CK_SC8800G2),

	/* third level: clock derived from top module. */
	CLK(NULL, "l3_256m", &l3_256m, CK_SC8800G2),
	CLK(NULL, "l3_192m", &l3_192m, CK_SC8800G2),
	CLK(NULL, "l3_153m600k", &l3_153m600k, CK_SC8800G2),

	/* 3. clocks from top module. */
	/* 3.1 from l3_256m */
	CLK(NULL, "clk_128m", &clk_128m, CK_SC8800G2),
	CLK(NULL, "clk_64m", &clk_64m, CK_SC8800G2),

	/* 3.2 from l3_192m */
	CLK(NULL, "clk_96m", &clk_96m, CK_SC8800G2),
	CLK(NULL, "clk_48m", &clk_48m, CK_SC8800G2),
	CLK(NULL, "clk_24m", &clk_24m, CK_SC8800G2),
	CLK(NULL, "clk_12m", &clk_12m, CK_SC8800G2),

	/* 3.3 from l3_153m600k */
	CLK(NULL, "clk_76m800k", &clk_76m800k, CK_SC8800G2),
	CLK(NULL, "clk_51m200k", &clk_51m200k, CK_SC8800G2),
	CLK(NULL, "clk_10m240k", &clk_10m240k, CK_SC8800G2),
	CLK(NULL, "clk_5m120k", &clk_5m120k, CK_SC8800G2),

	/* 3.4 from ext26m */
	CLK(NULL, "clk_13m", &clk_13m, CK_SC8800G2),
	CLK(NULL, "clk_6m500k", &clk_6m500k, CK_SC8800G2),

	/* 4. other clocks, source for peripherals. */
	CLK(NULL, "ccir_mclk", &ccir_mclk, CK_SC8800G2),
	CLK(NULL, "clk_ccir", &clk_ccir, CK_SC8800G2),
	CLK(NULL, "clk_dcam", &clk_dcam, CK_SC8800G2),
	CLK(NULL, "clk_vsp", &clk_vsp, CK_SC8800G2),
};

static void _sc88xx_clk_commit(struct clk *clk)
{
	/* nothing for now. */
}

static unsigned long sc8800g2_mpllcore_recalc(struct clk *clk)
{

	return 0;
}

static int sc8800g2_reprogram_mpllcore(struct clk *clk, unsigned long rate)
{

	return 0;
}

static int _sc88xx_clk_enable(struct clk *clk)
{
	return clk->ops->enable(clk);
}

static void _sc88xx_clk_disable(struct clk *clk)
{
	clk->ops->disable(clk);
}
static void sc88xx_clk_disable(struct clk *clk)
{
	if ((clk->usecount > 0) && !(--clk->usecount)) {
		_sc88xx_clk_disable(clk);
		if (clk->parent)
			sc88xx_clk_disable(clk->parent);
	}
}

static int sc88xx_clk_enable(struct clk *clk)
{
	int ret = 0;

	if (clk->usecount++ == 0) {
		if (clk->parent) {
			ret = sc88xx_clk_enable(clk->parent);
			if (ret)
				goto err;
		}
		ret = _sc88xx_clk_enable(clk);
		if (ret) {
			if (clk->parent)
				sc88xx_clk_disable(clk->parent);
			goto err;
		}
	}
	return ret;
err:
	clk->usecount--;
	return ret;
}


long sc88xx_clksel_round_rate(struct clk *clk, unsigned long target_rate)
{
	u32 valid_rate;
	const struct clksel_rate *clkr;
	int ret = -EINVAL;

	ret = sc88xx_clksel_rournd_rate_clkr(clk, target_rate, &clkr, &valid_rate);
	if (ret)
		return ret;

	return valid_rate;
}

static long	sc88xx_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk->round_rate)
		return clk->round_rate(clk, rate);

	if (clk->flags & RATE_FIXED)
		printk(KERN_ERR "clock: generic omap2_clk_round_rate called "
		       "on fixed-rate clock %s\n", clk->name);

	return clk->rate;

}

static int sc88xx_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	if (clk->flags & CONFIG_PARTICIPANT)
		return -EINVAL;

	if (clk->set_rate)
		ret = clk->set_rate(clk, rate);
	return ret;
}

static const struct clksel *sc88xx_get_clksel_by_parent(struct clk *clk,
					struct clk *src_clk)
{
	const struct clksel *clks;

	if (!clk->clksel)
		return NULL;

	for (clks = clk->clksel; clks->parent; clks++) {
		if (clks->parent == src_clk);
		break;
	}

	if (!clks->parent) {
		printk("clock: Can't find parent clock [%s] for clock [%s].\n",
				src_clk->name, clk->name);
		return NULL;
	}
	return clks;
}

static int sc88xx_clk_set_parent(struct clk *clk, struct clk *new_parent)
{
	u32 field_val, v;
	const struct clksel *clks;


	if (clk->flags & CONFIG_PARTICIPANT)
		return -EINVAL;

	if (!clk->clksel)
		return -EINVAL;


	clks = sc88xx_get_clksel_by_parent(clk, new_parent);
	if (!clks)
		return -EPERM;

	field_val = clks->val;

	WARN_ON(clk->clksel_mask == 0);

	v = __raw_readl(clk->clksel_reg);
	v &= (~clk->clksel_mask);
	v |= (field_val << __ffs(clk->clksel_mask));
	__raw_writel(v, clk->clksel_reg);
	v = __raw_readl(clk->clksel_reg);

	_sc88xx_clk_commit(clk);
	clk_reparent(clk, new_parent);

	if (clk->recalc)
		clk->rate = clk->recalc(clk);
	propagate_rate(clk);

	return 0;
}

static void sc88xx_clk_disable_unused(struct clk *clk)
{

}

#ifdef CONFIG_CPU_FREQ
static void	sc88xx_clk_init_cpufreq_table(struct cpufreq_frequency_table **)
{

}
#endif

static unsigned long sc88xx_recalc_generic(struct clk *clk)
{
	const struct clksel_rate *clkr;
	const struct clksel *clks;

	u32 v;

	if ((!clk->clksel_reg) || (!clk->clksel_mask))
		return 0;


	clks = sc88xx_get_clksel_by_parent(clk, clk->parent);
	if (!clks) {
		printk("clock: can't find parent for clock [%s].\n", clk->name);
		return 0;
	}

	if ((!clk->clkdiv_reg) || (!clk->clkdiv_mask)) {
		clkr = clks->rates;
		return clk->parent->rate / clkr->div;
	}
	else {
		v = __raw_readl(clk->clkdiv_reg);
		v &= (~clk->clkdiv_mask);
		v >>= __ffs(clk->clkdiv_mask);


		for (clkr = clks->rates; clkr->div; clkr++) {
			if (clkr->val == v)
				break;
		}

		if (!clkr->div) {
			pr_err("clock: can't find divisor for clock [%s].\n",
					clk->name);
			return 0;
		}
	}

	return (clk->parent->rate / clkr->div);
}

int sc88xx_clksel_rournd_rate_clkr(struct clk *clk, unsigned long target_rate,
					const struct clksel_rate **clkrp, u32 *valid_rate)
{
	const struct clksel_rate *clkr;
	const struct clksel *clks;
	u32 last_div = 0;
	unsigned long test_rate;

	if ((!clk->clksel_reg) || (!clk->clksel_mask))
		return -EINVAL;

	clks = sc88xx_get_clksel_by_parent(clk, clk->parent);
	if (!clks)
		return -EINVAL;

	for (clkr = clks->rates; clkr->div; clkr++) {
		if (clkr->div <= last_div) {
			pr_err("clock: clksel_rate table doesn't include item for clock [%s].\n",
					clk->name);
			return -EINVAL;
		}

		last_div = clkr->div;
		test_rate = clk->parent->rate / clkr->div;

		if (test_rate <= target_rate)
			break;
	}

	if (!clkr->div) {
		pr_err("clock: can't find divisor for clock [%s], rate = %ld\n",
				clk->name, target_rate);
		return -EINVAL;
	}
	*clkrp = clkr;
	*valid_rate = test_rate;
	return 0;
}

static int sc88xx_set_rate_generic(struct clk *clk, unsigned long rate)
{
	u32 valid_rate, v;
	const struct clksel_rate *clkr;
	int ret = -EINVAL;

	if ((!clk->clkdiv_reg) || (!clk->clkdiv_mask))
		return -EINVAL;

	ret = sc88xx_clksel_rournd_rate_clkr(clk, rate, &clkr, &valid_rate);
	if (ret)
		return ret;
	if (valid_rate != rate)
		return -EINVAL;

	v = __raw_readl(clk->clkdiv_reg);
	v &= (~clk->clkdiv_mask);
	v |= (clkr->val << __ffs(clk->clkdiv_mask));
	__raw_writel(v, clk->clkdiv_reg);
	v = __raw_readl(clk->clkdiv_reg);

	clk->rate = clk->parent->rate / clkr->div;
	propagate_rate(clk);

	_sc88xx_clk_commit(clk);

	return 0;
}


static struct clk_functions sc8800g2_clk_functions = {
	.clk_enable 		= sc88xx_clk_enable,
	.clk_disable 		= sc88xx_clk_disable,
	.clk_round_rate 	= sc88xx_clk_round_rate,
	.clk_set_rate 		= sc88xx_clk_set_rate,
	.clk_set_parent		= sc88xx_clk_set_parent,
	.clk_disable_unused	= sc88xx_clk_disable_unused,
#ifdef CONFIG_CPU_FREQ
	.clk_init_cpufreq_table	= sc88xx_clk_init_cpufreq_freq,
#endif
};

 int __init sc8800g2_clock_init(void)
 {
	struct sc88xx_clk *c;
	
	clk_init(&sc8800g2_clk_functions);
	for (c = sc8800g2_clks; c < (sc8800g2_clks + ARRAY_SIZE(sc8800g2_clks)); c++)
		clk_preinit(c->lk.clk);
		
	for (c = sc8800g2_clks; c < (sc8800g2_clks + ARRAY_SIZE(sc8800g2_clks)); c++) {
		clkdev_add(&c->lk);
		clk_register(c->lk.clk);
	}

	recalculate_root_clocks();

	clk_enable_init_clocks();

	sclk = clk_get(NULL, "sys_ck");
	clk_get_rate(sclk);
	clk_set_rate(sclk, 400000);
	clk_round_rate(sclk, 400000);
	clk_get_sys(NULL, "ext_26m");
	clk_add_alias("ext_26m_alias", NULL, "ext_26m", NULL);
	clk_set_parent(sclk, sclk);
	clk_get_parent(sclk);
	clk_enable(sclk);
	clk_disable(sclk);
	clk_put(sclk);
	return 0;
 }
arch_initcall(sc8800g2_clock_init);	
