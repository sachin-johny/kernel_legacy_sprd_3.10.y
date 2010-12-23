/*
 *
 *
 *
 *
 */

#ifndef __ARCH_ARM_SC8800G2_CLOCK_SC8800G2_H
#define __ARCH_ARM_SC8800G2_CLOCK_SC8800G2_H

#define IOMEM(a)	(void __iomem *)(a)
extern const struct clkops clkops_null;

static unsigned long sc8800g2_mpllcore_recalc(struct clk *clk);
static int sc8800g2_reprogram_mpllcore(struct clk *clk, unsigned long rate);

static unsigned long sc88xx_recalc_generic(struct clk *);

static int sc88xx_set_rate_generic(struct clk *, unsigned long);
int sc88xx_clksel_rournd_rate_clkr(struct clk *clk, unsigned long target_rate,
					const struct clksel_rate **clkrp, u32 *valid_rate);
long sc88xx_clksel_round_rate(struct clk *clk, unsigned long target_rate);
#endif




