/*
  *  Copyright (C) 2010 Spreadtrum Inc.
  *
  *   Wang Liwei.  <levee.wang@spreadtrum.com>
  *
  *
  */

#ifndef __ARCH_ARM_SC8800G2_CLOCK_SC8800G2_H
#define __ARCH_ARM_SC8800G2_CLOCK_SC8800G2_H

#define IOMEM(a)	(void __iomem *)(a)
extern const struct clkops clkops_null;

int sc88xx_clksel_rournd_rate_clkr(struct clk *clk, unsigned long target_rate,
					const struct clksel_rate **clkrp, u32 *valid_rate);
long sc88xx_clksel_round_rate(struct clk *clk, unsigned long target_rate);
int __init sc8800g2_clock_init(void);
int sc88xx_set_rate_generic(struct clk *clk, unsigned long rate);
unsigned long sc88xx_recalc_generic(struct clk *clk);
int sc8800g_get_clock_status(void);
int sc8800g_get_clock_info(void);

#endif
