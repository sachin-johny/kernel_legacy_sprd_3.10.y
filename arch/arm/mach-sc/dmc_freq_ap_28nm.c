/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input/matrix_keypad.h>
#include <mach/hardware.h>
#include <mach/__sc8830_dmc_dfs.h>

#include "mach/chip_x30g/dram_phy_28nm.h"

#define REG32(x)                           (*((volatile uint32 *)(x)))

#define UMCTL_REG_BASE (0x30000000)
#define PUBL_REG_BASE  (0x30010000)
#define UART1_PHYS_ADDR (0x70100000)

#define DPLL_CLK			800
#define DPLL_REFIN		26
#define NINT(FREQ,REFIN)	(FREQ/REFIN)
#define KINT(FREQ,REFIN)	((FREQ-(FREQ/REFIN)*REFIN)*1048576/REFIN)

typedef unsigned long int	uint32;

//DMC_UMCTL_REG_INFO_PTR_T gp_umctl_reg;
//DMC_PUBL_REG_INFO_PTR_T gp_publ_reg;

static inline void uart_putch(uint32 c)
{
	REG32(UART1_PHYS_ADDR) = c;
}__attribute__((always_inline))

static inline uint32 reg_bits_set(uint32 addr,uint32 start_bitpos,uint32 bit_num,uint32 value)
{
	/*create bit mask according to input param*/
	uint32 bit_mask = (1 << bit_num) - 1;
	uint32 reg_data = REG32(addr);

	reg_data &= ~(bit_mask<<start_bitpos);
	reg_data |= ((value&bit_mask)<<start_bitpos);

	REG32(addr) = reg_data;

	return 0;
}__attribute__((always_inline))

static inline  void move_upctl_state_to_self_refresh(void)
{
	volatile uint32 val;
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;

	p_umctl_reg->umctl_pwrctl = BIT_PWRCTL_SELFREF_SW;

	/* wait umctl2 core is in self-refresh mode */
	val = p_umctl_reg->umctl_stat;
	while ((val & (BIT_STAT_SELFREF_TYPE | BIT_STAT_OP_MODE)) != 0x23) {
		val = p_umctl_reg->umctl_stat;
	}
}__attribute__((always_inline))

static inline  void move_upctl_state_exit_self_refresh(void )
{
	volatile uint32 val;
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;

	p_umctl_reg->umctl_pwrctl = 0x00;

	/* wait umctl2 core is in self-refresh mode */
	val = p_umctl_reg->umctl_stat;
	while ((val &  BIT_STAT_OP_MODE) != 0x1) {
		val = p_umctl_reg->umctl_stat;
	}

}__attribute__((always_inline))

#if 0
static inline void disable_ddrphy_pll(void) {
        DMC_PUBL_REG_INFO_PTR_T p_publ_reg = (DMC_PUBL_REG_INFO_PTR_T) PUBL_REG_BASE;
	p_publ_reg->publ_pir |= BIT_PIR_PLLBYP;
}__attribute__((always_inline))

static inline void enable_ddrphy_pll(void) {
        DMC_PUBL_REG_INFO_PTR_T p_publ_reg = (DMC_PUBL_REG_INFO_PTR_T) PUBL_REG_BASE;
	p_publ_reg->publ_pir &= ~BIT_PIR_PLLBYP;
}__attribute__((always_inline))

static inline  void disable_cam_command_deque(void)
{
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	p_umctl_reg->umctl_dbg[1] |= BIT_DBG1_DIS_DQ;
}__attribute__((always_inline))

static inline  void enable_cam_command_deque(void)
{
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	p_umctl_reg->umctl_dbg[1] &= ~BIT_DBG1_DIS_DQ;
}__attribute__((always_inline))
#endif
static inline  void ddr_timing_update(ddr_dfs_val_t *timing);

static inline  void wait_queue_complete(void)
{
	volatile uint32 value_temp;
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;

	while(1) {
		value_temp = p_umctl_reg->umctl_dbgcam;
		if (((value_temp & BIT_DBGCAM_WR_PP_EPTY) != 0) &&
			((value_temp & BIT_DBGCAM_RD_PP_EPTY) != 0)) {

			return;
		}
	}
//}__attribute__((always_inline))
}

static inline void exit_lowpower_mode(uint32 *reg_store)
{
	volatile uint32 val;
        DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
#if 0
	p_umctl_reg->umctl_pwrctl = BIT_PWRCTL_DFICLK_DIS;
	val = p_umctl_reg->umctl_stat;
	/* wait umctl2 core is in self-refresh mode */
	while((val & 0x7) != 1) {
		val = p_umctl_reg->umctl_stat;
	}
#endif
	/* disable lp interface. */
	reg_store[0] = p_umctl_reg->umctl_hwlpctl;
	p_umctl_reg->umctl_hwlpctl &= ~0x03;
	//p_umctl_reg->umctl_dfilpcfg[0] = 0x0700f000;

	/* disable_cgm */
	REG32(SPRD_PMU_PHYS + 0xF8) &= ~((1 << 30) | 0x3FF);

	/* disable_powerdown */
	reg_store[1] = p_umctl_reg->umctl_pwrctl;
	p_umctl_reg->umctl_pwrctl = 0;
	/* wait status to normal mode */
	val = p_umctl_reg->umctl_stat;
	while ((val &  BIT_STAT_OP_MODE) != 0x1) {
		val = p_umctl_reg->umctl_stat;
	}

	/* disable_sleep */
	p_umctl_reg->umctl_dfilpcfg[0] &= ~(1 << 8);
}__attribute__((always_inline))

static inline void enable_lowpower_mode(uint32 *reg_store)
{
        DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;

	/* enable_cgm */
	REG32(SPRD_PMU_PHYS + 0xF8) |= ((1 << 30) | 0x3FF);

	/* enable_sleep */
	p_umctl_reg->umctl_dfilpcfg[0] |= (1 << 8);

	/* enable lp interface. */
	p_umctl_reg->umctl_hwlpctl = reg_store[0];

	/* enable_powerdown */
	p_umctl_reg->umctl_pwrctl = reg_store[1];
}__attribute__((always_inline))

static inline void ddr_cam_command_dequeue(uint32 isEnable)
{
        DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;

	if(isEnable) {
		p_umctl_reg->umctl_dbg[1] &= ~BIT_DBG1_DIS_DQ;
	}
	else {
		p_umctl_reg->umctl_dbg[1] |= BIT_DBG1_DIS_DQ;
	}
}__attribute__((always_inline))


static inline void ddr_timing_update_ex(ddr_dfs_val_t *timing_param)
{
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	/* minimum time from refresh to refresh or active */
	//toggle this signel indicate refresh register has been update
	p_umctl_reg->umctl_rfshtmg = timing_param->umctl2_rfshtmg;
	p_umctl_reg->umctl_rfshctl3 ^= (1<<1);

	//update umctl & publ timing
	p_umctl_reg->umctl_dramtmg[0] = timing_param->umctl2_dramtmg0;
	p_umctl_reg->umctl_dramtmg[1] = timing_param->umctl2_dramtmg1;
	p_umctl_reg->umctl_dramtmg[2] = timing_param->umctl2_dramtmg2;
	p_umctl_reg->umctl_dramtmg[3] = timing_param->umctl2_dramtmg3;
	p_umctl_reg->umctl_dramtmg[4] = timing_param->umctl2_dramtmg4;
	p_umctl_reg->umctl_dramtmg[5] = timing_param->umctl2_dramtmg5;
	p_umctl_reg->umctl_dramtmg[6] = timing_param->umctl2_dramtmg6;
	//p_umctl_reg->umctl_dramtmg[7] = timing_param->umctl2_dramtmg7;
	//p_umctl_reg->umctl_dramtmg[8] = timing_param->umctl2_dramtmg8;

}__attribute__((always_inline))

static inline void ddr_clk_set(uint32 new_clk, uint32 sene,ddr_dfs_val_t *timing)
{
	volatile uint32 i;
	volatile uint32 reg_store[3];
	DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	DMC_PUBL_REG_INFO_PTR_T p_publ_reg = (DMC_PUBL_REG_INFO_PTR_T) PUBL_REG_BASE;

	uart_putch('d');
	exit_lowpower_mode(&(reg_store[0]));

//	uart_putch('2');
	/* step 1: assert PWRCTL.selfref_sw. */
	/* step 2: wait until STAT.operating_mode[1:0] = 11, STAT.selfref_type[1:0] = 10 */
	move_upctl_state_to_self_refresh();

//	uart_putch('3');
	/* step 3: hold bus : set DBG1.dis_dq = 1 */
	ddr_cam_command_dequeue(0);
	wait_queue_complete();

//	uart_putch('4');
	/* step4 : DFIMISC.dfi_init_complete_en =0.hold not trigger sdram initialization */
	p_umctl_reg->umctl_dfimisc &= ~BIT_DFIMISC_DFI_COMP_EN;

//	uart_putch('5');
	/* step 5: change the clock frequency to the DWC_ddr_umctl2 and ensure no glitchs. */
	/* phy clock close : DDR_PHY_AUTO_GATE_EN */
	reg_store[2] = REG32(SPRD_PMU_PHYS+0x00D0);
	REG32(SPRD_PMU_PHYS+0x00D0) &= ~((1 << 6) | 0x07);
	for(i = 0; i < 0x2; i++);

	switch(new_clk) {
		case 200:
		{
		#if 0
			if ((sene == EMC_FREQ_NORMAL_SWITCH_SENE)
				|| (sene == EMC_FREQ_RESUME_SENE)) {

				/* set tdpll clock divider */
				/* reg[0x402D0024] :*/
				/* 			[9:8]  : clk_emc_div(clk_div= clk_src/(div+1)) */
				/* 			[1:0]  : clk_emc_sel (0:pub 26m, 1: CPLL, 2:TDPLL, 3:DPLL)*/
				reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x8, 2, 0x1);
				for(i = 0; i < 0x2; i++);

				/* switch to tdpll source 768Mhz */
				reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x0, 2, 0x2);
				for(i = 0; i < 0x2; i++);

				/* 192M */
			}

			if(sene == EMC_FREQ_DEEP_SLEEP_SENE)
		#endif
			{
			#if 0
				/* set  dpll clock to 201M : set DPLL_REFIN to 26M * 31 = 806M*/
				reg_val = REG32(REG_AON_APB_DPLL_CFG1);
				reg_val |= 1 << 10; 		// fractional divider
				reg_val &= ~(0xFFFFF << 12 | 0x3f);
				reg_val |= (KINT(DPLL_CLK, DPLL_REFIN) & 0xFFFFF) << 12;
				reg_val |= (NINT(DPLL_CLK, DPLL_REFIN)) & 0x3f;
				REG32(REG_AON_APB_DPLL_CFG1)= reg_val;

				reg_val = REG32(REG_AON_APB_DPLL_CFG); //DPLL reference clock  0-2M 1-4M 2-13M 3-26M
				reg_val &= ~(3 << 24); 
				reg_val |= (3 << 24); 
				REG32(REG_AON_APB_DPLL_CFG) = reg_val;
			#endif
				/* switch to dpll source */
				reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x8, 2, 0x1);
				for(i = 0; i < 0x2; i++);

				reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x0, 2, 0x3);
				for(i = 0; i < 0x2; i++);
			}

			break;
			}

		case 384:
		{
			/* set  dpll clock to 192M : set DPLL_REFIN to 26M * 26 = 676*/
			//reg_bits_set((SPRD_AONAPB_PHYS + 0x3004), 24, 2, 0x03);
			//reg_bits_set((SPRD_AONAPB_PHYS + 0x3074), 0, 6, 26);
			
			/* switch to dpll source */
			/* reg[0x402D0024] :*/
			/* 			[9:8]  : clk_emc_div(clk_div= clk_src/(div+1)) */
			/* 			[1:0]  : clk_emc_sel (0:pub 26m, 1: CPLL, 2:TDPLL, 3:DPLL)*/
			reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x8, 2, 0x0);
			for(i = 0; i < 0x2; i++);

			reg_bits_set((SPRD_AONCKG_PHYS + 0x0024), 0x0, 2, 0x2);
			for(i = 0; i < 0x2; i++);

			break;
		}

		case 400:
		{
			/* set  dpll clock to 400M : set DPLL_REFIN to 4M*/
			/* reg[0x402E3004] :*/
			/* 			[25:24]  : DPLL_REFIN(00: 2M, 01 :4M, 10:13M, 11:26M) */
			/* 			[10:0]  : DPLL_N( dpll = dpll_refin * dpll_n)*/
			/* reg[0x402E3074] :*/
			/* 			[31:12]  : DPLL_KINT */
			/* 			[10]  : DPLL_DIV_S */
			/* 			[7]  : DPLL_MOD_EN */
			/* 			[6]  : DPLL_SDM_EN */
			/* 			[5:0]  : DPLL_NINT */
		#if 0
			/* set  dpll clock to 201M : set DPLL_REFIN to 26M * 31 = 806M*/
			reg_val = REG32(REG_AON_APB_DPLL_CFG1);
			reg_val |= 1 << 10; 		// fractional divider
			reg_val &= ~(0xFFFFF << 12 | 0x3f);
			reg_val |= (KINT(DPLL_CLK, DPLL_REFIN) & 0xFFFFF) << 12;
			reg_val |= (NINT(DPLL_CLK, DPLL_REFIN)) & 0x3f;
			REG32(REG_AON_APB_DPLL_CFG1)= reg_val;

			reg_val = REG32(REG_AON_APB_DPLL_CFG);
			reg_val &= ~(3 << 24); 
			reg_val |= (3 << 24); 
			REG32(REG_AON_APB_DPLL_CFG) = reg_val;
		#endif

			/* config dpll divider */
			reg_bits_set((SPRD_AONCKG_PHYS+0x0024), 0x8, 2, 0x0);
			for(i = 0; i < 0x2; i++);

			/*switch to dpll source */
			reg_bits_set((SPRD_AONCKG_PHYS+0x0024), 0x0, 2, 0x3);
			for(i = 0; i < 0x2; i++);

			break;
		}
		default:
			break;
	}

	/* phy clock open */
	REG32(SPRD_PMU_PHYS+0x00D0) = reg_store[2];
	//REG32(SPRD_PMU_PHYS+0x00D0) |= (1 << 6);
	for(i = 0; i < 0x2; i++);

	/* PIR phy_init */
	p_publ_reg->publ_pir |= ( BIT_PIR_DCAL | BIT_PIR_INIT);
	while ((p_publ_reg->publ_pgsr[0] & 0x01) != 1);

//	uart_putch('6');
	/* step 6: update phy timing register. static and dynamic register */
	ddr_timing_update_ex(timing);

//	uart_putch('7');
	/* step 7: trigger the initization PHY */
	p_umctl_reg->umctl_dfimisc |= BIT_DFIMISC_DFI_COMP_EN;

	uart_putch('f');
	/* step 8: require to exit sel_refresh by PWRCTL.selfref_sw. */
	move_upctl_state_exit_self_refresh();
//	uart_putch('9');
	/* step 9: Update MR register setting of DRAM */
	//p_umctl_reg->umctl_hwlpctl |= 0x03;

	/* step 10: FBG1.dis_dq = 0 */
	ddr_cam_command_dequeue(1);
	uart_putch('s');

	/* enable lowpower */
	enable_lowpower_mode(&(reg_store[0]));
}__attribute__((always_inline))

#define DFS_PARAM_ADDR	(0x1C00)

static inline  ddr_dfs_val_t *get_clk_timing( uint32 clk )
{
	volatile uint32 i;
	ddr_dfs_val_t *timing;

	timing = (ddr_dfs_val_t *)(DFS_PARAM_ADDR);

	for (i = 0; i < 5; i++) {
		if (clk == timing->ddr_clk) {
			return timing;
		}
		timing++;
	}

	return (ddr_dfs_val_t *)0;
}__attribute__((always_inline))

inline void dev_freq_set(unsigned long req)
{
	u32 clk;
	u32 sene;
	ddr_dfs_val_t *timing;

	/* initialize umctl, phy base address */
	//gp_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	//gp_publ_reg = (DMC_PUBL_REG_INFO_PTR_T) PUBL_REG_BASE;

	//ddr_type = (req & EMC_DDR_TYPE_MASK) >> EMC_DDR_TYPE_OFFSET;
	clk = (req & EMC_CLK_FREQ_MASK) >> EMC_CLK_FREQ_OFFSET;
	//dll_mode = (req & EMC_DLL_MODE_MASK);
	sene = (req & EMC_FREQ_SENE_MASK) >> EMC_FREQ_SENE_OFFSET;

	timing = get_clk_timing(clk);
	if(timing->ddr_clk != clk) {
		uart_putch('d');
		uart_putch('f');
		uart_putch('s');
		uart_putch('e');
		uart_putch('r');
		uart_putch('r');
		uart_putch('o');
		uart_putch('r');
		uart_putch('\n');
		while(1);
	}

	ddr_clk_set(clk, sene, timing);
} __attribute__((always_inline))

void emc_dfs_main(unsigned long flag)
{
	volatile uint32 reg, val;
	/* disable mmu, cache */
	/* SP				: 0x1B00 ~ 0x1BFF */
	/* ddr timing params	: 0x1C00 ~ 0x1CFF */
	asm volatile (
			"mrc p15, 0, %0, c1, c0, 0 \n"
			"ldr %1, =0x1005 \n"
			"bic %0, %0, %1 \n"
			"mcr p15, 0, %0, c1, c0, 0\n"
			"ldr sp, =0x1B00 \n"
			"ldr r11, =0x1F00 \n"
			: "=r"(reg), "=r"(val)
			);

	dev_freq_set(flag); /* call dfs freq set function */

	/* jump back */
	asm volatile (
			"ldr r0, =cpu_resume \n"
			"sub r0, r0, #0xc0000000 \n"
			"add r0, r0, #0x80000000 \n"
			"mov pc, r0 \n"
			);
}
