/*
  *  Copyright (C) 2010 Spreadtrum Inc.
  * 
  *   Wang Liwei.  <levee.wang@spreadtrum.com>
  *
  *
  */
#ifndef __ARCH_ARM_SC8800G2_CLOCK_COMMON_H
#define __ARCH_ARM_SC8800G2_CLOCK_COMMON_H



/* clock flags. */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define CONFIG_PARTICIPANT	(1 << 10)	/* Fundamental clock */
#define ENABLE_ON_INIT	(0x1UL << 11)	/* enable on framework init. */
#define INVERT_ENABLE           (1 << 12)       /* 0 enables, 1 disables */

/* clksel flags. */
#define RATE_IN_SC8800G2	(0x1UL << 0)



/* control registers. */
#define 	PLL_SCR		GR_PLL_SCR
#define 	GEN0		GR_GEN0
#define 	GEN1		GR_GEN1
#define 	GEN2		GR_GEN2
#define 	GEN3		GR_GEN3
#define 	GEN4		GR_GEN4
#define 	GEN5		GR_CLK_GEN5
#define		PLLMN_CTRL	GR_MPLL_MN
#define		CLK_DLY		GR_CLK_DLY
#define		PCTL		GR_PCTL
#define		CLK_EN		GR_CLK_EN
//#define		AHB_CTL0	AHB_CTL0


/* clock enable bit. */
#define		CCIR_MCLK_EN_SHIFT	14
#define		CLK_DCAM_EN_SHIFT	1
#define		CLK_VSP_EN_SHIFT	0

/* clock source mask. */
#define		CCIR_MCLK_CLKSEL_MASK 	(0x3UL << 18)
#define		CLK_DCAM_CLKSEL_MASK 	(0x3UL << 4)
#define		CLK_VSP_CLKSEL_MASK 	(0x3UL << 2)




/* clock divisor mask */
#define		CCIR_MCLK_CLKDIV_MASK	(0x3UL << 24)



struct clk;

struct clksel_rate {
	u32		val;
	u32		div;
	u32		flags;
};

struct clksel {
	struct clk 					*parent;
	u32							val;
	const struct clksel_rate	*rates;
};
struct clkops {
	int			(*enable)(struct clk *);
	void		(*disable)(struct clk *);
	void		(*find_idlest)(struct clk *, void __iomem **, u8 *);
	void		(*find_companion)(struct clk *, void __iomem **, u8 *);
};

struct clk {
	struct list_head 		node;
	const struct clkops 	*ops;
	const char  			*name;
	int 					id;
	struct clk				*parent;
	struct list_head		children;
	struct list_head		sibling;
	unsigned long 			rate;
	__u32				flags;
	long			(*round_rate)(struct clk *, unsigned long);
	void				(*init)(struct clk *);
	u32					usecount;
	unsigned long		(*recalc)(struct clk *);
	int 				(*set_rate)(struct clk *, unsigned long);
	const char			*clkdm_name;


	u8					fixed_div;
	void __iomem		*clksel_reg;
	u32					clksel_mask;
	const struct clksel	*clksel;

	void __iomem		*enable_reg;
	u32					enable_bit;

	void __iomem		*clkdiv_reg;
	u32					clkdiv_mask;
};


struct clk_functions {
	int		(*clk_enable)(struct clk *clk);
	void	(*clk_disable)(struct clk *clk);
	int		(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_parent)(struct clk *clk, struct clk *parent);
	long	(*clk_round_rate)(struct clk *clk, unsigned long rate);
	void	(*clk_allow_idle)(struct clk *clk);
	void	(*clk_deny_idle)(struct clk *clk);
	void	(*clk_disable_unused)(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
	void	(*clk_init_cpufreq_table)(struct cpufreq_frequency_table **);
#endif
};


int __init clk_init(struct clk_functions *custom_clocks);
void clk_preinit(struct clk *clk);
int clk_register(struct clk *clk);
void propagate_rate(struct clk *tclk);
void clk_enable_init_clocks(void);
void recalculate_root_clocks(void);
void sc88xx_init_clksel_parent(struct clk *clk);
void clk_reparent(struct clk *child, struct clk *parent);

#endif

