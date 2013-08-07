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
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <asm/irqflags.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>
#include <mach/system.h>
#include <mach/pm_debug.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/sci.h>
#include "emc_repower.h"
#include <linux/clockchips.h>
#include <linux/wakelock.h>
#include <mach/adi.h>
#include <mach/arch_misc.h>
#if defined(CONFIG_SPRD_DEBUG)
/* For saving Fault status */
#include <mach/sprd_debug.h>
#endif

extern int sc8830_get_clock_status(void);
extern void secondary_startup(void);
extern int sp_pm_collapse(unsigned int cpu, unsigned int save_state);
extern void sp_pm_collapse_exit(void);
extern void sc8830_standby_iram(void);
extern void sc8830_standby_iram_end(void);
extern void sc8830_standby_exit_iram(void);
extern int sc_cpuidle_init(void);
void pm_ana_ldo_config(void);


static void setup_autopd_mode(void)
{
	if (soc_is_scx35_v0())
		sci_glb_write(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG, 0x3a, -1UL);
	else
		sci_glb_write(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG, 0x3B, -1UL);
	sci_glb_write(REG_PMU_APB_AP_WAKEUP_POR_CFG, 0x1, -1UL);//AP_WAKEUP_POR_CFG

	// INTC0_EB, INTC1_EB, INTC2_EB, INTC3_EB
	sci_glb_set(REG_AP_APB_APB_EB, 0xf<<19);

	//XTL_EN function mode 0
	sci_glb_clr(SPRD_PIN_BASE + 0x138, BIT(4) | BIT(5));

	//set PD_CA7_C0_AUTO_SHUTDOWN_EN
	sci_glb_set(REG_PMU_APB_PD_CA7_TOP_CFG, BIT_PD_CA7_TOP_AUTO_SHUTDOWN_EN);

	//set PD_CA7_C0_AUTO_SHUTDOWN_EN
	sci_glb_set(REG_PMU_APB_PD_CA7_C0_CFG, BIT_PD_CA7_C0_AUTO_SHUTDOWN_EN);

	sci_glb_set(SPRD_INT_BASE + 0x8, BIT(14) | BIT(4)); //ana & eic

	sci_glb_set(SPRD_INTC1_BASE + 0x8, BIT(6) | BIT(4)); //ana & kpd
	//sci_glb_set(SPRD_INT_BASE + 0x8, BIT(2)); //ana & eic
	//sci_glb_set(SPRD_INTC0_BASE + 0x8, BIT(30) | BIT(31)); //syst & aon_syst

	sci_adi_set(ANA_REG_GLB_LDO_SLP_CTRL0, BIT_SLP_IO_EN | BIT_SLP_LDORF0_PD_EN | BIT_SLP_LDORF1_PD_EN | BIT_SLP_LDORF2_PD_EN);
	//sci_adi_clr(ANA_REG_GLB_LDO_SLP_CTRL0, BIT_SLP_IO_EN | BIT_SLP_LDORF0_PD_EN | BIT_SLP_LDORF1_PD_EN | BIT_SLP_LDORF2_PD_EN);

	/* don't power down emmccore & emmcio */
	sci_adi_clr(ANA_REG_GLB_LDO_SLP_CTRL0, BIT_SLP_LDOEMMCCORE_PD_EN | BIT_SLP_LDOEMMCIO_PD_EN);

	sci_adi_set(ANA_REG_GLB_LDO_SLP_CTRL1, BIT_SLP_LDO_PD_EN);
	sci_adi_clr(ANA_REG_GLB_XTL_WAIT_CTRL, BIT_SLP_XTLBUF_PD_EN);
	//sci_adi_set(ANA_REG_GLB_XTL_WAIT_CTRL, BIT_SLP_XTLBUF_PD_EN);
	//sci_adi_set(ANA_REG_GLB_LDO_SLP_CTRL2, BIT_SLP_LDORF2_LP_EN | BIT_SLP_LDORF1_LP_EN | BIT_SLP_LDORF0_LP_EN);

	//sci_adi_clr(ANA_REG_GLB_LDO_SLP_CTRL2, BIT_SLP_LDORF2_LP_EN | BIT_SLP_LDORF1_LP_EN | BIT_SLP_LDORF0_LP_EN);

	//sci_glb_set(REG_PMU_APB_DDR_SLEEP_CTRL, 7<<4);
	//sci_glb_set(REG_PMU_APB_DDR_SLEEP_CTRL, 5<<4);
	sci_glb_clr(REG_PMU_APB_PD_PUB_SYS_CFG, BIT_PD_PUB_SYS_AUTO_SHUTDOWN_EN);
	sci_glb_clr(REG_PMU_APB_PD_MM_TOP_CFG,BIT_PD_MM_TOP_AUTO_SHUTDOWN_EN);
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_SLEEP_FOLLOW_CA7_EN);
	sci_glb_write(REG_PMU_APB_AP_WAKEUP_POR_CFG, 0x1, -1UL);
	/* KEEP eMMC/SD power */
	sci_adi_clr(ANA_REG_GLB_LDO_SLP_CTRL0, BIT_SLP_LDOEMMCCORE_PD_EN | BIT_SLP_LDOEMMCIO_PD_EN);
	sci_adi_clr(ANA_REG_GLB_LDO_SLP_CTRL1, BIT_SLP_LDOSD_PD_EN );

	return;
}
void disable_mm(void)
{
	sci_glb_clr(REG_MM_AHB_GEN_CKG_CFG, BIT_JPG_AXI_CKG_EN | BIT_VSP_AXI_CKG_EN |\
		BIT_ISP_AXI_CKG_EN | BIT_DCAM_AXI_CKG_EN | BIT_SENSOR_CKG_EN | BIT_MIPI_CSI_CKG_EN | BIT_CPHY_CFG_CKG_EN);
	sci_glb_clr(REG_MM_AHB_AHB_EB, BIT_JPG_EB | BIT_CSI_EB | BIT_VSP_EB | BIT_ISP_EB | BIT_CCIR_EB | BIT_DCAM_EB);
	sci_glb_clr(REG_MM_AHB_GEN_CKG_CFG, BIT_MM_AXI_CKG_EN);
	sci_glb_clr(REG_MM_AHB_GEN_CKG_CFG, BIT_MM_MTX_AXI_CKG_EN);
	sci_glb_clr(REG_MM_AHB_AHB_EB, BIT_MM_CKG_EB);
	sci_glb_clr(REG_PMU_APB_PD_MM_TOP_CFG,BIT_PD_MM_TOP_AUTO_SHUTDOWN_EN);
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_MM_EB);
	sci_glb_set(REG_PMU_APB_PD_MM_TOP_CFG,BIT_PD_MM_TOP_AUTO_SHUTDOWN_EN);
	return;
}
void bak_restore_mm(int bak)
{
	static uint32_t gen_ckg, ahb_eb, mm_top, aon_eb0;
	static uint32_t gen_ckg_en_mask = BIT_JPG_AXI_CKG_EN | BIT_VSP_AXI_CKG_EN |\
		BIT_ISP_AXI_CKG_EN | BIT_DCAM_AXI_CKG_EN | BIT_SENSOR_CKG_EN | BIT_MIPI_CSI_CKG_EN | BIT_CPHY_CFG_CKG_EN;
	static uint32_t ahb_eb_mask = BIT_JPG_EB | BIT_CSI_EB | BIT_VSP_EB | BIT_ISP_EB | BIT_CCIR_EB | BIT_DCAM_EB;
	if(bak){
		gen_ckg = sci_glb_read(REG_MM_AHB_GEN_CKG_CFG, -1UL);
		ahb_eb = sci_glb_read(REG_MM_AHB_AHB_EB, -1UL);
		mm_top = sci_glb_read(REG_PMU_APB_PD_MM_TOP_CFG, -1UL);
		aon_eb0 = sci_glb_read(REG_AON_APB_APB_EB0, -1UL);
	}else{
		sci_glb_clr(REG_PMU_APB_PD_MM_TOP_CFG,BIT_PD_MM_TOP_AUTO_SHUTDOWN_EN);
		if(aon_eb0 & BIT_MM_EB)
			sci_glb_set(REG_AON_APB_APB_EB0, BIT_MM_EB);
		if(ahb_eb & BIT_MM_CKG_EB)
			sci_glb_set(REG_MM_AHB_AHB_EB, BIT_MM_CKG_EB);
		if(gen_ckg & BIT_MM_MTX_AXI_CKG_EN)
			sci_glb_set(REG_MM_AHB_GEN_CKG_CFG, BIT_MM_MTX_AXI_CKG_EN);
		if(gen_ckg & BIT_MM_AXI_CKG_EN)
			sci_glb_set(REG_MM_AHB_GEN_CKG_CFG, BIT_MM_AXI_CKG_EN);
		if(ahb_eb & ahb_eb_mask)
			sci_glb_write(REG_MM_AHB_AHB_EB, ahb_eb, ahb_eb_mask);
		if(gen_ckg & gen_ckg_en_mask)
			sci_glb_write(REG_MM_AHB_GEN_CKG_CFG, gen_ckg, gen_ckg_en_mask);
	}
	return;
}
void disable_dma(void)
{
	int reg = sci_glb_read(SPRD_DMA0_BASE, BIT(16));
	int cnt = 30;
	if(reg){
		sci_glb_set((SPRD_DMA0_BASE), BIT(0)); // pause dma
		while(cnt-- && reg){
			sci_glb_read(SPRD_DMA0_BASE, BIT(16));
			udelay(100);
			reg = sci_glb_read(SPRD_DMA0_BASE, BIT(16));
		}
		if(reg)
			printk("disable dma failed\n");
	}
	sci_glb_clr(REG_AP_AHB_AHB_EB, BIT_DMA_EB);
}
void disable_ahb_module(void)
{
	//sci_glb_write(REG_AP_AHB_AHB_EB, 0, -1UL);
	sci_glb_clr(REG_AP_AHB_AHB_EB, 0x1fff);

	// AP_PERI_FORCE_SLP

	sci_glb_set(REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG, BIT_AP_PERI_FORCE_SLP);
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_SLEEP_FOLLOW_CA7_EN);
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_DEEP_SLEEP_EN);

	//AP_SYS_AUTO_SLEEP_CFG
	sci_glb_set(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG, 0x3B);

	return;
}
void bak_restore_ahb(int bak)
{
	static uint32_t ahb_eb;
	if(bak){
		ahb_eb = sci_glb_read(REG_AP_AHB_AHB_EB, -1UL);
	}else{
		sci_glb_clr(REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG, BIT_AP_PERI_FORCE_SLP);
		sci_glb_write(REG_AP_AHB_AHB_EB, ahb_eb, -1UL);
	}
	return;
}

void disable_apb_module(void)
{
	int stat;
	stat = sci_glb_read(REG_AP_APB_APB_EB, -1UL);
	stat &=(0xf<<19 | BIT_UART1_EB | BIT_UART0_EB); // leave intc on
	sci_glb_write(REG_AP_APB_APB_EB, stat, -1UL);
	return;
}
void bak_restore_apb(int bak)
{
	static uint32_t apb_eb;
	if(bak){
		apb_eb = sci_glb_read(REG_AP_APB_APB_EB, -1UL);
	}else{
		sci_glb_write(REG_AP_APB_APB_EB, apb_eb, -1UL);
	}
	return;
}
void disable_aon_module(void)
{
	//sci_glb_clr(REG_AON_APB_APB_EB0, BIT_CA7_DAP_EB | BIT_GPU_EB);
	//sci_glb_clr(REG_AON_APB_APB_EB0, BIT_CA7_DAP_EB);

	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_AON_TMR_EB | BIT_AP_TMR0_EB);
	sci_glb_clr(REG_AON_APB_APB_EB1, BIT_AP_TMR2_EB | BIT_AP_TMR1_EB);
	sci_glb_clr(REG_AON_APB_APB_EB1, BIT_DISP_EMC_EB);
	sci_glb_clr(REG_AON_APB_PWR_CTRL, BIT_USB_PHY_PD);
}
void bak_restore_aon(int bak)
{
	static uint32_t apb_eb1, pwr_ctl, apb_eb0;
	if(bak){
		apb_eb0 = sci_glb_read(REG_AON_APB_APB_EB0, -1UL);
		apb_eb1 = sci_glb_read(REG_AON_APB_APB_EB1, -1UL);
		pwr_ctl = sci_glb_read(REG_AON_APB_PWR_CTRL, -1UL);
	}else{
		if(apb_eb1 & BIT_DISP_EMC_EB)
			sci_glb_set(REG_AON_APB_APB_EB1, BIT_DISP_EMC_EB);
		if(pwr_ctl & BIT_USB_PHY_PD)
			sci_glb_set(REG_AON_APB_PWR_CTRL, BIT_USB_PHY_PD);
		sci_glb_write(REG_AON_APB_APB_EB0, apb_eb0, -1UL);
		sci_glb_write(REG_AON_APB_APB_EB1, apb_eb1, -1UL);
	}
	return;
}
void disable_ana_module(void)
{
	sci_adi_set(ANA_REG_GLB_LDO_PD_CTRL, BIT_LDO_SD_PD | BIT_LDO_SIM0_PD | BIT_LDO_SIM1_PD | BIT_LDO_SIM2_PD | BIT_LDO_CAMA_PD |\
		BIT_LDO_CAMD_PD | BIT_LDO_CAMIO_PD | BIT_LDO_CAMMOT_PD | BIT_LDO_USB_PD | BIT_DCDC_WPA_PD);
}
void bak_restore_ana(int bak)
{
	static uint32_t ldo_pd_ctrl;
	static uint32_t mask;
	mask = BIT_LDO_SD_PD | BIT_LDO_SIM0_PD | BIT_LDO_SIM1_PD | BIT_LDO_SIM2_PD | BIT_LDO_CAMA_PD |\
		BIT_LDO_CAMD_PD | BIT_LDO_CAMIO_PD | BIT_LDO_CAMMOT_PD | BIT_LDO_USB_PD | BIT_DCDC_WPA_PD;
	if(bak){
		ldo_pd_ctrl = sci_adi_read(ANA_REG_GLB_LDO_PD_CTRL);
	}else{
		sci_adi_write(ANA_REG_GLB_LDO_PD_CTRL, ldo_pd_ctrl, mask);
	}
	return;
}
#define REG_PIN_XTLEN                   ( SPRD_PIN_BASE + 0x0138 )
#define REG_PIN_CHIP_SLEEP              ( SPRD_PIN_BASE + 0x0208 )
#define REG_PIN_XTL_BUF_EN0             ( SPRD_PIN_BASE + 0x020C )
#define REG_PIN_XTL_BUF_EN1             ( SPRD_PIN_BASE + 0x0210 )
#define REG_PIN_XTL_BUF_EN2             ( SPRD_PIN_BASE + 0x0214 )
void show_pin_reg(void)
{
	sci_glb_set(SPRD_INT_BASE + 0x8, BIT(14) | BIT(4)); //ana & eic
	sci_glb_clr(SPRD_PIN_BASE + 0x138, BIT(4) | BIT(5));
	printk("REG_PIN_XTLEN   0x%08x\n", sci_glb_read(REG_PIN_XTLEN, -1UL));
	printk("REG_PIN_XTL_BUF_EN0   0x%08x\n", sci_glb_read(REG_PIN_XTL_BUF_EN0, -1UL));
	printk("REG_PIN_XTL_BUF_EN1   0x%08x\n", sci_glb_read(REG_PIN_XTL_BUF_EN1, -1UL));
	printk("REG_PIN_XTL_BUF_EN2   0x%08x\n", sci_glb_read(REG_PIN_XTL_BUF_EN2, -1UL));
	printk("REG_PIN_CHIP_SLEEP    0x%08x\n", sci_glb_read(REG_PIN_CHIP_SLEEP, -1UL));
	printk("### uart1 ckd 0x%08x\n", sci_glb_read(SPRD_UART1_BASE + 0X24, -1UL));
	printk("### uart1 ctl 0x%08x\n", sci_glb_read(SPRD_UART1_BASE + 0X18, -1UL));
	sci_glb_set(REG_PMU_APB_XTL0_REL_CFG, 0x4);
	sci_glb_set(REG_PMU_APB_XTLBUF0_REL_CFG, 0x4);
	sci_glb_clr(REG_PMU_APB_PD_CP1_TD_CFG, BIT(24));
	printk("REG_PMU_APB_XTL0_REL_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_XTL0_REL_CFG, -1UL));
	printk("REG_PMU_APB_XTL1_REL_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_XTL1_REL_CFG, -1UL));
	printk("REG_PMU_APB_XTL2_REL_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_XTL2_REL_CFG, -1UL));
	printk("REG_PMU_APB_XTLBUF0_REL_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_XTLBUF0_REL_CFG, -1UL));
	printk("REG_PMU_APB_XTLBUF1_REL_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_XTLBUF1_REL_CFG, -1UL));
	printk("REG_PMU_APB_PD_CP1_TD_CFG 0x%08x\n", sci_glb_read(REG_PMU_APB_PD_CP1_TD_CFG, -1Ul));
}
void force_mcu_core_sleep(void)
{
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_CORE_SLEEP);
}
void enable_mcu_deep_sleep(void)
{
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_DEEP_SLEEP_EN);
}
void disable_mcu_deep_sleep(void)
{
	sci_glb_clr(REG_AP_AHB_MCU_PAUSE, BIT_MCU_DEEP_SLEEP_EN);
}
#define BITS_SET_CHECK(__reg, __bits, __string) do{\
	uint32_t mid =sci_glb_read(__reg, __bits); \
	if(mid != __bits) \
		printk(__string " not 1" "\n"); \
	}while(0)
#define BITS_CLEAR_CHECK(__reg, __bits, __string) do{\
	uint32_t mid = sci_glb_read(__reg, __bits); \
	if(mid & __bits) \
		printk(__string " not 0" "\n"); \
	}while(0)
#define PRINT_REG(__reg) do{\
	uint32_t mid = sci_glb_read(__reg, -1UL); \
	printk(#__reg "value 0x%x\n", mid); \
	}while(0)
void show_reg_status(void)
{
	BITS_CLEAR_CHECK(REG_AON_APB_APB_EB0, BIT_CA7_DAP_EB, "ca7_dap_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_DMA_EB, "dma_eb");
	BITS_CLEAR_CHECK((SPRD_DMA0_BASE + 0x1c), BIT(20), "dma_busy");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_USB_EB, "usb_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_SDIO0_EB, "sdio0_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_SDIO1_EB, "sdio1_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_SDIO2_EB, "sdio2_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_EMMC_EB, "emmc_eb");
	BITS_CLEAR_CHECK(REG_AP_AHB_AHB_EB, BIT_NFC_EB, "nfc_eb");
	PRINT_REG(REG_PMU_APB_SLEEP_STATUS);
}
uint32_t pwr_stat0=0, pwr_stat1=0, pwr_stat2=0, pwr_stat3=0, apb_eb0=0, apb_eb1=0, ahb_eb=0, apb_eb=0, mcu_pause=0, sys_force_sleep=0, sys_auto_sleep_cfg=0;
uint32_t ldo_slp_ctrl0, ldo_slp_ctrl1, ldo_slp_ctrl2, xtl_wait_ctrl, ldo_slp_ctrl3, ldo_aud_ctrl4;
uint32_t mm_apb, ldo_pd_ctrl, arm_module_en;
uint32_t cp_slp_status_dbg0, cp_slp_status_dbg1, sleep_ctrl, ddr_sleep_ctrl, sleep_status, pwr_ctrl, ca7_standby_status;
uint32_t pd_pub_sys;
void bak_last_reg(void)
{
	pd_pub_sys = __raw_readl(REG_PMU_APB_PD_PUB_SYS_CFG);
	cp_slp_status_dbg0 = __raw_readl(REG_PMU_APB_CP_SLP_STATUS_DBG0);
	cp_slp_status_dbg1 = __raw_readl(REG_PMU_APB_CP_SLP_STATUS_DBG1);
	pwr_stat0 = __raw_readl(REG_PMU_APB_PWR_STATUS0_DBG);
	pwr_stat1 = __raw_readl(REG_PMU_APB_PWR_STATUS1_DBG);
	pwr_stat2 = __raw_readl(REG_PMU_APB_PWR_STATUS2_DBG);
	pwr_stat3 = __raw_readl(REG_PMU_APB_PWR_STATUS3_DBG);
	sleep_ctrl = __raw_readl(REG_PMU_APB_SLEEP_CTRL);
	ddr_sleep_ctrl = __raw_readl(REG_PMU_APB_DDR_SLEEP_CTRL);
	sleep_status = __raw_readl(REG_PMU_APB_SLEEP_STATUS);

	apb_eb0 = __raw_readl(REG_AON_APB_APB_EB0);
	apb_eb1 = __raw_readl(REG_AON_APB_APB_EB1);
	pwr_ctrl = __raw_readl(REG_AON_APB_PWR_CTRL);

	ahb_eb = __raw_readl(REG_AP_AHB_AHB_EB);

	apb_eb = __raw_readl(REG_AP_APB_APB_EB);
	mcu_pause = __raw_readl(REG_AP_AHB_MCU_PAUSE);
	sys_force_sleep = __raw_readl(REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG);
	sys_auto_sleep_cfg = __raw_readl(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG);
	ca7_standby_status = __raw_readl(REG_AP_AHB_CA7_STANDBY_STATUS);

	ldo_slp_ctrl0 = sci_adi_read(ANA_REG_GLB_LDO_SLP_CTRL0);
	ldo_slp_ctrl1 = sci_adi_read(ANA_REG_GLB_LDO_SLP_CTRL1);
	ldo_slp_ctrl2 = sci_adi_read(ANA_REG_GLB_LDO_SLP_CTRL2);
	ldo_slp_ctrl3 = sci_adi_read(ANA_REG_GLB_LDO_SLP_CTRL3);
	ldo_aud_ctrl4 = sci_adi_read(ANA_REG_GLB_AUD_SLP_CTRL4);
	xtl_wait_ctrl = sci_adi_read(ANA_REG_GLB_XTL_WAIT_CTRL);

	ldo_pd_ctrl = sci_adi_read(ANA_REG_GLB_LDO_PD_CTRL);
	arm_module_en = sci_adi_read(ANA_REG_GLB_ARM_MODULE_EN);
}
void print_last_reg(void)
{
	printk("aon pmu status reg\n");
	printk("REG_PMU_APB_PD_PUB_SYS_CFG ------ 0x%08x\n", pd_pub_sys);
	printk("REG_PMU_APB_CP_SLP_STATUS_DBG0 ----- 0x%08x\n", cp_slp_status_dbg0);
	printk("REG_PMU_APB_CP_SLP_STATUS_DBG1 ----- 0x%08x\n", cp_slp_status_dbg1);
	printk("REG_PMU_APB_PWR_STATUS0_DBG ----- 0x%08x\n", pwr_stat0);
	printk("REG_PMU_APB_PWR_STATUS1_DBG ----- 0x%08x\n", pwr_stat1);
	printk("REG_PMU_APB_PWR_STATUS2_DBG ----- 0x%08x\n", pwr_stat2);
	printk("REG_PMU_APB_PWR_STATUS3_DBG ----- 0x%08x\n", pwr_stat3);
	printk("REG_PMU_APB_SLEEP_CTRL ----- 0x%08x\n", sleep_ctrl);
	printk("REG_PMU_APB_DDR_SLEEP_CTRL ----- 0x%08x\n", ddr_sleep_ctrl);
	printk("REG_PMU_APB_SLEEP_STATUS ----- 0x%08x\n", sleep_status);

	printk("aon apb reg\n");
	printk("REG_AON_APB_APB_EB0  ----- 0x%08x\n", apb_eb0);
	printk("REG_AON_APB_APB_EB1  ----- 0x%08x\n", apb_eb1);
	printk("REG_AON_APB_PWR_CTRL ----- 0x%08x\n", pwr_ctrl);

	printk("ap ahb reg \n");
	printk("REG_AP_AHB_AHB_EB ----- 0x%08x\n", ahb_eb);

	printk("ap apb reg\n");
	printk("REG_AP_APB_APB_EB ---- 0x%08x\n", apb_eb);
	printk("REG_AP_AHB_MCU_PAUSE ---   0x%08x\n", mcu_pause);
	printk("REG_AP_AHB_AP_SYS_FORCE_SLEEP_CFG --- 0x%08x\n", sys_force_sleep);
	printk("REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG ---- 0x%08x\n", sys_auto_sleep_cfg);
	printk("REG_AP_AHB_CA7_STANDBY_STATUS ---- 0x%08x\n", ca7_standby_status);

	printk("ana reg \n");
	printk("ANA_REG_GLB_LDO_SLP_CTRL0 --- 0x%08x\n", ldo_slp_ctrl0);
	printk("ANA_REG_GLB_LDO_SLP_CTRL1 --- 0x%08x\n", ldo_slp_ctrl1);
	printk("ANA_REG_GLB_LDO_SLP_CTRL2 --- 0x%08x\n", ldo_slp_ctrl2);
	printk("ANA_REG_GLB_LDO_SLP_CTRL3 --- 0x%08x\n", ldo_slp_ctrl3);
	printk("ANA_REG_GLB_AUD_SLP_CTRL4 --- 0x%08x\n", ldo_aud_ctrl4);
	printk("ANA_REG_GLB_XTL_WAIT_CTRL --- 0x%08x\n", xtl_wait_ctrl);

	printk("ANA_REG_GLB_PWR_XTL_EN0 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN0));
	printk("ANA_REG_GLB_PWR_XTL_EN1 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN1));
	printk("ANA_REG_GLB_PWR_XTL_EN2 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN2));
	printk("ANA_REG_GLB_PWR_XTL_EN3 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN3));
	printk("ANA_REG_GLB_PWR_XTL_EN4 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN4));
	printk("ANA_REG_GLB_PWR_XTL_EN5 -- 0x%08x\n", sci_adi_read(ANA_REG_GLB_PWR_XTL_EN5));

	printk("mm reg\n");
	printk("REG_MM_AHB_AHB_EB ---- 0x%08x\n", mm_apb);

	printk("ana reg\n");
	printk("ANA_REG_GLB_LDO_PD_CTRL --- 0x%08x\n", ldo_pd_ctrl);
	printk("ANA_REG_GLB_ARM_MODULE_EN --- 0x%08x\n", arm_module_en);
}
#if 0
static void test_setup_timer(int load)
{
	//AP_TMR0_EB
	sci_glb_set(REG_AON_APB_APB_EB0, BIT(12));

	//AP_TMR0_RTC_EB
	sci_glb_set(REG_AON_APB_APB_RTC_EB, BIT(5));

	//load value
	sci_glb_write(SPRD_APTIMER0_BASE, load, -1UL);

	//int en
	sci_glb_set(SPRD_APTIMER0_BASE + 0xc, BIT(0));


	//AP_TMR0_RTC_EB --- TIMER_INT_EN   timer0 INTC0 en
	sci_glb_set(SPRD_INT_BASE + 0x8, BIT(2) | BIT(29));

	//int en
	sci_glb_set(SPRD_APTIMER0_BASE + 0x8, BIT(7));

	sci_glb_write(SPRD_INT_BASE + 0x8, 0xffffffff, -1UL);
	return;
}
#endif
void check_ldo(void)
{
}

void check_pd(void)
{
}

unsigned int sprd_irq_pending(void)
{
	uint32_t bits = 0;
	bits = sci_glb_read(SPRD_INT_BASE, -1UL);
	if(bits)
		return 1;
	else
		return 0;
}

/*copy code for deepsleep return */
#define SAVED_VECTOR_SIZE 64
static uint32_t *sp_pm_reset_vector = NULL;
static uint32_t saved_vector[SAVED_VECTOR_SIZE];
void __iomem *iram_start;
struct emc_repower_param *repower_param;


#define SPRD_RESET_VECTORS 0X00000000
#define IRAM_START_PHY 	SPRD_IRAM_PHYS
#define SLEEP_CODE_SIZE 4096
#define EMC_REINIT_CODE_SIZE 4096

static int init_reset_vector(void)
{
	sp_pm_reset_vector = (u32 *)SPRD_IRAM0_BASE;

	iram_start = (void __iomem *)(SPRD_IRAM_BASE);
	/* copy sleep code to (IRAM). */
	if ((sc8830_standby_iram_end - sc8830_standby_iram) > SLEEP_CODE_SIZE) {
		panic("##: code size is larger than expected, need more memory!\n");
	}
	memcpy_toio(iram_start, sc8830_standby_iram, SLEEP_CODE_SIZE);


	/* just make sure*/
	flush_cache_all();
	outer_flush_all();
	return 0;
}
void save_reset_vector(void)
{
	int i = 0;
	for (i = 0; i < SAVED_VECTOR_SIZE; i++)
		saved_vector[i] = sp_pm_reset_vector[i];
}

void set_reset_vector(void)
{
	int i = 0;
	printk("SAVED_VECTOR_SIZE 0x%x\n", SAVED_VECTOR_SIZE);
	for (i = 0; i < SAVED_VECTOR_SIZE; i++)
		sp_pm_reset_vector[i] = 0xe320f000; /* nop*/

	sp_pm_reset_vector[SAVED_VECTOR_SIZE - 2] = 0xE51FF004; /* ldr pc, 4 */

	sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] = (sc8830_standby_exit_iram -
		sc8830_standby_iram + IRAM_START_PHY); /* place v7_standby_iram here */
}

void restore_reset_vector(void)
{
	int i;
	for (i = 0; i < SAVED_VECTOR_SIZE; i++)
		sp_pm_reset_vector[i] = saved_vector[i];
}
/* irq functions */
#define hw_raw_irqs_disabled_flags(flags)	\
({						\
	(int)((flags) & PSR_I_BIT);		\
})

#define hw_irqs_disabled()			\
({						\
	unsigned long _flags;			\
	local_irq_save(_flags);			\
	hw_raw_irqs_disabled_flags(_flags);	\
})
u32 __attribute__ ((naked)) read_cpsr(void)
{
	__asm__ __volatile__("mrs r0, cpsr\nbx lr");
}
/* make sure printk is end, if not maybe some messy code  in SERIAL1 output */
#define UART_TRANSFER_REALLY_OVER (0x1UL << 15)
#define UART_STS0 (SPRD_UART1_BASE + 0x08)
#define UART_STS1 (SPRD_UART1_BASE + 0x0c)

static __used void wait_until_uart1_tx_done(void)
{
	u32 tx_fifo_val;
	u32 really_done = 0;
	u32 timeout = 2000;

	tx_fifo_val = __raw_readl(UART_STS1);
	tx_fifo_val >>= 8;
	tx_fifo_val &= 0xff;
	while(tx_fifo_val != 0) {
		if (timeout <= 0) break;
		udelay(100);
		tx_fifo_val = __raw_readl(UART_STS1);
		tx_fifo_val >>= 8;
		tx_fifo_val &= 0xff;
		timeout--;
	}

	timeout = 30;
	really_done = __raw_readl(UART_STS0);
	while(!(really_done & UART_TRANSFER_REALLY_OVER)) {
		if (timeout <= 0) break;
		udelay(100);
		really_done = __raw_readl(UART_STS0);
		timeout--;
	}
}
#define SAVE_GLOBAL_REG do{ \
	bak_restore_ahb(1); \
	bak_restore_aon(1); \
	bak_restore_apb(1); \
	}while(0)
#define RESTORE_GLOBAL_REG do{ \
	bak_restore_apb(0); \
	bak_restore_aon(0); \
	bak_restore_ahb(0); \
	}while(0)

/* arm core sleep*/
static void arm_sleep(void)
{
	cpu_do_idle();
	hard_irq_set();
}

/* arm core + arm sys */
static void mcu_sleep(void)
{
	SAVE_GLOBAL_REG;
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_SYS_SLEEP_EN);
	cpu_do_idle();
	sci_glb_clr(REG_AP_AHB_MCU_PAUSE, BIT_MCU_SYS_SLEEP_EN);
	hard_irq_set();
	RESTORE_GLOBAL_REG;
}

static void light_sleep(void)
{
	sci_glb_set(REG_AP_AHB_MCU_PAUSE, BIT_MCU_LIGHT_SLEEP_EN);
	cpu_do_idle();
	sci_glb_clr(REG_AP_AHB_MCU_PAUSE, BIT_MCU_LIGHT_SLEEP_EN);
}
void int_work_round(void)
{
	sci_glb_set(SPRD_AONAPB_BASE + 0x8, (0x1<<30 | 1<<8 | 1<<1));
	//sci_glb_clr(SPRD_LPDDR2_PHY_BASE + 0x2c, BIT(4));
	//sci_glb_set(SPRD_LPDDR2_PHY_BASE + 0x2c, BIT(4));
	//sci_glb_write(SPRD_LPDDR2_BASE + 0x30, 0, -1UL);
	//sci_glb_set(SPRD_PMU_BASE + 0xa8, 1<<1);
	sci_adi_clr(ANA_EIC_BASE + 0x18, 0x20);
}
void show_deep_reg_status(void)
{
	printk("PWR_STATUS0_DBG	0x%08x\n", sci_glb_read(SPRD_PMU_BASE+0x00B4, -1UL));
	printk("PWR_STATUS1_DBG	0x%08x\n", sci_glb_read(SPRD_PMU_BASE+0x00B8, -1UL));
	printk("PWR_STATUS2_DBG	0x%08x\n", sci_glb_read(SPRD_PMU_BASE+0x00BC, -1UL));
	printk("PWR_STATUS3_DBG	0x%08x\n", sci_glb_read(SPRD_PMU_BASE+0x00C0, -1UL));

}


extern void pm_debug_set_wakeup_timer(void);
extern void pm_debug_set_apwdt(void);
int deep_sleep(int from_idle)
{
	int ret = 0;
	static unsigned int cnt = 0;

	if(!from_idle){
		SAVE_GLOBAL_REG;
		/* some prepare here */
		show_pin_reg();
		enable_mcu_deep_sleep();
		disable_ahb_module();
	    //disable_pmu_ddr_module();
	    
		sci_glb_set(SPRD_PMU_BASE+0x00F4, 0x3FF);
		
		disable_dma();
		//disable_mm();
		//disable_ana_module();
		disable_aon_module();
		show_reg_status();
		bak_last_reg();
		print_last_reg();
		print_int_status();
		//wait_until_uart1_tx_done();
		int_work_round();
		//pm_debug_set_apwdt();
		disable_apb_module();
		//pm_debug_set_wakeup_timer();
		//force_mcu_core_sleep();
		//bak_last_reg();

		__raw_writel(0x0, REG_PMU_APB_CA7_C0_CFG);
	}
	show_deep_reg_status();

	ret = sp_pm_collapse(0, 1);

	udelay(50);
	if(!from_idle){
		__raw_writel(0x1, REG_PMU_APB_CA7_C0_CFG);
		printk("ret %d not from idle\n", ret);
		if(ret){
			printk("deep sleep %u times\n", cnt);
			cnt++;
		}
		sci_glb_set(REG_AP_APB_APB_EB, 0xf<<19);
		hard_irq_set();
		sci_glb_clr(REG_AP_APB_APB_EB, 0xf<<19);
		disable_mcu_deep_sleep();
		sci_glb_clr(SPRD_PMU_BASE+0x00F4, 0x3FF);
		RESTORE_GLOBAL_REG;
	}

	udelay(5);
	if (ret) cpu_init();

	if (soc_is_scx35_v0())
		sci_glb_clr(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,BIT_CA7_CORE_AUTO_GATE_EN);

	return ret;
}

#define DEVICE_AHB              (0x1UL << 20)
#define DEVICE_APB              (0x1UL << 21)
#define DEVICE_VIR              (0x1UL << 22)
#define DEVICE_AWAKE            (0x1UL << 23)
#define LIGHT_SLEEP            (0x1UL << 24)
#define DEVICE_TEYP_MASK        (DEVICE_AHB | DEVICE_APB | DEVICE_VIR | DEVICE_AWAKE | LIGHT_SLEEP)
int sprd_cpu_deep_sleep(unsigned int cpu)
{
	int status, ret = 0;
	unsigned long flags, time;

#ifdef CONFIG_SPRD_PM_DEBUG
	__raw_writel(0xfdffbfff, SPRD_INTC0_BASE + 0xc);//intc0
	__raw_writel(0x02004000, SPRD_INTC0_BASE + 0x8);//intc0
	__raw_writel(0xffffffff, SPRD_INTC0_BASE + 0x100c);//intc1
#endif	


	time = get_sys_cnt();
	if (!hw_irqs_disabled())  {
		flags = read_cpsr();
		printk("##: Error(%s): IRQ is enabled(%08lx)!\n",
			 "wakelock_suspend", flags);
	}
	/*TODO:
	* we need to known clock status in modem side
	*/
#ifdef FORCE_DISABLE_DSP
	status = 0;
#else
	/*
	* TODO: get clock status in native version, force deep sleep now
	*/
	status = 0;
#endif
	if (status & DEVICE_AHB)  {
		printk("###### %s,  DEVICE_AHB ###\n", __func__ );
		set_sleep_mode(SLP_MODE_ARM);
		arm_sleep();
	} else if (status & DEVICE_APB) {
		printk("###### %s,	DEVICE_APB ###\n", __func__ );
		set_sleep_mode(SLP_MODE_MCU);
		mcu_sleep();
	} else if (status & LIGHT_SLEEP) {
		printk("###### %s,	DEVICE_APB ###\n", __func__ );
		set_sleep_mode(SLP_MODE_LIT);
		light_sleep();
	} else {
		/*printk("###### %s,	DEEP ###\n", __func__ );*/
		set_sleep_mode(SLP_MODE_DEP);
#if 1

		ret = deep_sleep(0);
#else
		//pm_debug_set_wakeup_timer();
		ret = 0;
		arm_sleep();
#endif
		flush_cache_all();
	}
	
	time_add(get_sys_cnt() - time, ret);
	print_hard_irq_inloop(ret);

	return ret;
}
static void init_gr(void){}
void sc_default_idle(void)
{
	cpu_do_idle();
	local_irq_enable();
	return;
}
void pm_ana_ldo_config(void)
{
	/*set vddcore deep sleep voltage to 0.9v*/
	sci_adi_set(ANA_REG_GLB_DCDC_SLP_CTRL, BITS_DCDC_CORE_CTL_DS(3));
	/*open vddcore lp mode*/
	sci_adi_set(ANA_REG_GLB_LDO_SLP_CTRL2, BIT_SLP_DCDCCORE_LP_EN);
}
static void init_led(void){}
static struct timespec persistent_ts;
static u64 persistent_ms, last_persistent_ms;
void read_persistent_clock(struct timespec *ts)
{
	u64 delta;
	struct timespec *tsp = &persistent_ts;

	last_persistent_ms = persistent_ms;
	persistent_ms = get_sys_cnt();
	delta = persistent_ms - last_persistent_ms;

	timespec_add_ns(tsp, delta * NSEC_PER_MSEC);
	*ts = *tsp;
}

static void sc8830_power_off(void)
{
	/*turn off all modules's ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL, 0x1fff);

	/*turn off system core's ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_DCDC_PD_RTCCLR, 0x0);
	sci_adi_raw_write(ANA_REG_GLB_LDO_DCDC_PD_RTCSET, 0X7fff);
}

static void sc8830_machine_restart(char mode, const char *cmd)
{
	local_irq_disable();
	local_fiq_disable();

	arch_reset(mode, cmd);

	mdelay(1000);

	printk("reboot failed!\n");

	while (1);
}

void sc_pm_init(void)
{
	
	init_reset_vector();
	pm_power_off   = sc8830_power_off;
	arm_pm_restart = sc8830_machine_restart;
	pr_info("power off %pf, restart %pf\n", pm_power_off, arm_pm_restart);
	init_gr();
	setup_autopd_mode();
	pm_ana_ldo_config();
	init_led();
	/* disable all sleep mode */
	sci_glb_clr(REG_AP_AHB_MCU_PAUSE, BIT_MCU_DEEP_SLEEP_EN | BIT_MCU_LIGHT_SLEEP_EN | \
		BIT_MCU_SYS_SLEEP_EN | BIT_MCU_CORE_SLEEP);
	set_reset_vector();
#ifndef CONFIG_SPRD_PM_DEBUG
	pm_debug_init();
#endif

	/* enable arm clock auto gating*/
	//sci_glb_set(REG_AHB_AHB_CTL1, BIT_ARM_AUTO_GATE_EN);
	pm_idle = sc_default_idle;
#ifdef CONFIG_CPU_IDLE
	sc_cpuidle_init();
#endif
/*
	wake_lock_init(&pm_debug_lock, WAKE_LOCK_SUSPEND, "pm_not_ready");
	wake_lock(&pm_debug_lock);
*/
#if defined(CONFIG_SPRD_DEBUG)
	sprd_debug_init();
#endif
}
