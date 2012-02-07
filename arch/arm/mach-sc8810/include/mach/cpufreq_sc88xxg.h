#ifndef __CLOCK_DVFS_H__
#define __CLOCK_DVFS_H__

#include <linux/mutex.h>

#ifdef  CONFIG_FREQ_DEBUG
#define DBG(f, x...) \
	printk("[%s()]: "f, __func__, ## x)
#else
#define DBG(f, x...)
#endif
#define CLK_MCU_SEL_SHIFT     23
#define CLK_AHB_DIV_SHIFT      4
#define CLK_EMC_DIV_SHIFT      8
#define CLK_EMC_SEL_SHIFT     12
#define GR_MPLL_REFIN_SHIFT   16
#define GR_DPLL_REFIN_SHIFT   16
#define CLK_AXI_SHIFT         11

#define CLK_AXI_MASK       (0x3)
#define CLK_MCU_DIV_MASK    (0x7)
#define CLK_AHB_DIV_MASK    (0x7)
#define CLK_EMC_DIV_MASK    (0x7)
#define CLK_EMC_SEL_MASK    (0x3)
#define CLK_MCU_SEL_MASK    (0x3)
#define GR_MPLL_N_MASK      (0x7ff)
#define GR_MPLL_REFIN_MASK  (0x3) 
#define GR_DPLL_N_MASK      (0x7ff)
#define GR_DPLL_REFIN_MASK  (0x3)

#define DCDCARM_CTL_MASK            (0x7)


#define MHz 1000000
#define GR_MPLL_REFIN_2M    (2*MHz) 
#define GR_MPLL_REFIN_4M    (4*MHz)
#define GR_MPLL_REFIN_13M    (13*MHz)
#define GR_DPLL_REFIN_2M    (2*MHz) 
#define GR_DPLL_REFIN_4M    (4*MHz)
#define GR_DPLL_REFIN_13M    (13*MHz)



static unsigned long cpu_main_freq = 0;
#define REG_AHB_ARM_CLK					(*((volatile unsigned int *)(AHB_ARM_CLK)))
#define REG_AHB_CA5_CFG					(*((volatile unsigned int *)(AHB_CA5_CFG)))
#define REG_CHIP_ID					(*((volatile unsigned int *)(CHIP_ID)))
#define REG_GR_GEN1					(*((volatile unsigned int *)(GR_GEN1)))
#define REG_GR_MPLL_MN					(*((volatile unsigned int *)(GR_MPLL_MN)))
#define REG_GR_DPLL_MN					(*((volatile unsigned int *)(GR_DPLL_CTRL)))

#define REG_GR_TDPLL_CTRL					(*((volatile unsigned int *)(GR_TDPLL_CTRL)))
#define REG_GR_PCTL					(*((volatile unsigned int *)(GR_PCTL)))
#define REG_GR_DPLL_CTRL					(*((volatile unsigned int *)(GR_DPLL_CTRL)))


#ifdef CONFIG_ARCH_SC8810
#define ARM_MIN_VDD                 (650)
#define ARM_MAX_VDD                 (1300)
#define ARM_VDD_STEP                (100)
#define ARM_MAX_VDD_USED            (1200)


#define MIN_FREQ					(26000)
#define MAX_FREQ					(1000000)
#endif

#ifdef CONFIG_ARCH_SC8805
#define MIN_FREQ					(26000)
#define MAX_FREQ					(400000)
#endif

#define FREQ_TABLE_ENTRY			(5)


#define LOOPS_PER_JIFFY ( )//should figure out


struct sprd_dvfs_table {
	unsigned long  clk_mcu_mhz;
	unsigned long  vdd_mcu_mv;
};


struct cpu_freq_desc{
	unsigned int	cpu_clk_khz;
	unsigned int	cpu_src_sel;
	unsigned int	cpu_src_div;
	unsigned int	axi_clk_khz;
	unsigned int	vdd_mv;
	unsigned long	lpj; /* loops_per_jiffy */
};
/*
struct clock_state {
	struct cpu_freq_desc	*current_freq;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			vdd_switch_time_us;
	unsigned long			wait_for_irq_khz;
	int				wfi_ramp_down;
	int				pwrc_ramp_down;
};
*/

#endif
