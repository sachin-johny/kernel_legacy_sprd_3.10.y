#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/sci.h>
#include <linux/earlysuspend.h>
#include <mach/sci_glb_regs.h>
#include <linux/random.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>
#include <mach/sc8830_emc_freq_data.h>
#include <asm/suspend.h>
#include <linux/vmalloc.h>
#include <linux/printk.h>
#include <mach/__sc8830_dmc_dfs.h>
#include "mach/chip_x30g/dram_phy_28nm.h"

static volatile u32 mutex_flg = 0;

/* other PLL clock srv, t-shark is used 384 for tdpll */
#define PLAT_CLK				384

#define DDR_TIMING_REG_VAL_ADDR	(SPRD_IRAM0H_BASE + 0xc00)
#define UMCTL_REG_BASE (0x30000000)
#define PUBL_REG_BASE  (0x30010000)

#define NINT(FREQ,REFIN)	(FREQ/REFIN)
#define KINT(FREQ,REFIN)	((FREQ-(FREQ/REFIN)*REFIN)*1048576/REFIN)

#define REG32(x)                           (*((volatile u32 *)(x)))

#ifdef CONFIG_SCX35_DMC_FREQ_AP
static void emc_dfs_code_copy(u8 * dest);
static int emc_dfs_call(unsigned long flag);
void emc_dfs_main(unsigned long flag);
#endif
#ifdef EMC_FREQ_AUTO_TEST
static u32 get_sys_cnt(void)
{
	return __raw_readl(SPRD_GPTIMER_BASE + 0x44);
}
#endif
static u32 dpll_clk_get(void)
{
	u32 clk = 0;
	u32 nint, kint, refin, pnt = 0 ;
	u32 reg, div_s, sdm_en, n;

	reg = sci_glb_read(REG_AON_APB_DPLL_CFG1, -1);
	kint = ((reg >> 12) & 0xFFFFF);
	nint = (reg & 0x3F);
	div_s = (reg >> 10) & 0x01;
	sdm_en = (reg >> 6) & 0x01;

	reg = sci_glb_read(REG_AON_APB_DPLL_CFG, -1);
	n = reg & 0x3F;
	if((reg & 0x03000000) == 0x00000000) {
		refin = 2;
	}
	if((reg & 0x03000000) == 0x01000000) {
		refin = 4;
	}
	if((reg & 0x03000000) == 0x02000000) {
		refin = 13;
	}
	if((reg & 0x03000000) == 0x03000000) {
		refin = 26;
	}

	if ((div_s != 0) && (sdm_en != 0)) {
		if (((kint * refin ) & 0xFFFFF) >= 0x80000) {
			pnt = 1;
		}
		else {
			pnt = 0;
		}
		kint = ((kint * refin) >> 20) + pnt;
		clk = nint * refin + kint;
	}
	else if ((div_s != 0) && (sdm_en == 0)) {
		clk = nint * refin;
	}
	else/* if (div_s == 0) */ {
		clk = refin * n;
	}

	return clk;
}

u32 emc_clk_get(void)
{
	u32 pll_clk;
	u32 div;
	u32 reg_val;
	u32 sel;
	u32 clk;
	reg_val = sci_glb_read(REG_AON_CLK_EMC_CFG, -1);
	sel = reg_val & 0x3;
	div = (reg_val >> 8) & 0x3;
	switch(sel) {
		case 0:
			pll_clk = 26;
			break;
		case 1:
			pll_clk = 624;
			break;
		case 2:
			pll_clk = 768;
			break;
		case 3:
			pll_clk = dpll_clk_get();
			break;
		default:
			break;
	}

	clk = (pll_clk / (div + 1)) >> 1;
	return clk;
}
EXPORT_SYMBOL(emc_clk_get);

static void dpll_clk_set(u32 clk)
{
	volatile u32 reg = 0;
	u32 refin = 0;

	if(dpll_clk_get() == clk) {
		return;
	}

	/* get refin value */
	refin = 26;

	/* set kint, nint */
	reg = sci_glb_read(REG_AON_APB_DPLL_CFG1, -1);
	reg |= 1 << 10;			/* set fractional divider */
	reg &=~(0xfffff<<12 | 0x3f);
	reg |= (KINT(clk, refin) & 0xfffff) << 12;
	reg |= (NINT(clk, refin)) & 0x3f;

	if (0 == (reg & 0xfffff000)) {
		reg &= ~(1<<6);			/* sdm_en disable */
	} else {
		reg |= 1<<6;				/* sdm_en */
	}

	sci_glb_write(REG_AON_APB_DPLL_CFG1, reg, -1);

	/* Set REFIN */
	reg = sci_glb_read(REG_AON_APB_DPLL_CFG,-1);
	reg &= ~(BITS_DPLL_REFIN(0x03));
	if (refin == 26) {
		reg |= BITS_DPLL_REFIN(0x03);
	}
	else if (refin == 13){
		reg |= BITS_DPLL_REFIN(0x02);
	}
/*
	else if (refin = 4){
		reg |= BITS_DPLL_REFIN(0x02);
	}
	else if (refin = 2){
	}
*/
	sci_glb_write(REG_AON_APB_DPLL_CFG, reg, -1);

	udelay(100);
}

static u32 __emc_clk_set(u32 clk)
{
	u32 flag = 0;

	/* check timing params */
	flag = (clk << EMC_CLK_FREQ_OFFSET);

#ifdef CONFIG_SCX35_DMC_FREQ_AP
	flush_cache_all();
	cpu_suspend(flag, emc_dfs_call);

#endif
	if(emc_clk_get() != clk) {
		printk("[__emc_clk_set] error, set clk = %d, get clk = %d\n\r", clk, emc_clk_get());
	}
	return 0;
}

static u32 get_dpll_from_clk (u32 new_clk)
{
	u32 dpll = 0;

	switch (new_clk){
		case 200:
		case 400:
			dpll = 800;
			break;
		case 384:
			/* this clock from tdpll */
			break;
		case 233:
		case 466:
			dpll = 932;
			break;
		case 533:
			dpll = 1066;
			break;
		default :
			break;
	}

	return dpll;
}
/* How To look PLL Setting
	reg name		val				bit
	MPLL_CFG	reFin			[25:24]
	MPLL_CFG	N				[5:0]

	MPLL_CFG1	div_s			[10]
	MPLL_CFG1	sdm_en			[6]
	MPLL_CFG1	Nint				[5:0]
	MPLL_CFG1	Kint				[31:12]
	div_s = 1  	sdm_en = 1		Fout = Fin * ( Nint + Kint/1048576)
	div_s = 1  	sdm_en = 0		Fout = Fin * Nint
	div_s = 0  	sdm_en = x		Fout = Fin * N  */
/* reg[0x402D0024] :*/
/* [9:8]  : clk_emc_div(clk_div= clk_src/(div+1)) */
/* [1:0]  : clk_emc_sel (0:pub 26m, 1: CPLL, 2:TDPLL, 3:DPLL)*/
static u32 check_need_plat_clk(u32 new_clk)
{
	u32 sel, cur_dpll, nd_dpll = 0;

	if (PLAT_CLK == new_clk) {
		return 0;
	}

	sel = sci_glb_read(REG_AON_CLK_EMC_CFG, -1);
	sel &= 0x3;
	/* if current not used dpll return 0 don't  need platform clock */
	if (sel != 0x03) {
		return 0;
	}

	/* get dpll clock to check */
	cur_dpll = dpll_clk_get();
	nd_dpll = get_dpll_from_clk(new_clk);
	if (nd_dpll == 0){
		/* must be not go to this, if running, new_clk error! */
		return 1;
	}

	if (cur_dpll == nd_dpll) {
		return 0;
	}

	return 1;
}

u32 emc_clk_set(u32 new_clk, u32 sene)
{
	u32 old_clk, nd_dpll;
	volatile u32 reg = 0, reg_val = 0;

/* used to get dfs hold times for debug */
#ifdef EMC_FREQ_AUTO_TEST
	u32 start_t1, end_t1;
	static u32 max_u_time = 0;
	u32 current_u_time;

	start_t1 = get_sys_cnt();
#endif

	/* check new_clk */
	if ((new_clk != 200) && (new_clk != 384) && (new_clk != 400)
			&& (new_clk != 233) && (new_clk != 466) && (new_clk != 533)) {
		printk("[emc_clk_set] : new_clk[%d] error!\n\r", new_clk);
		return 0;
	}

	/* make sure no other thread running this code */
	if(mutex_flg >= 1) {
		panic("now other thread set dmc clk\n");
		goto out;
	}
	mutex_flg ++;

	/* step 1: save irq and lock it */
#if defined (EMC_FREQ_AUTO_TEST) || defined (CONFIG_SCX35_DMC_FREQ_AP)
	unsigned long irq_flags = 0;
	local_irq_save(irq_flags);
#endif

	/* check current clock  */
	old_clk = emc_clk_get();
	if(old_clk == new_clk) {
	        goto out;
	}

	/* step 2: check new clock need alter dpll( paltform clock ) or not */
	if (check_need_plat_clk(new_clk)) {
		__emc_clk_set(PLAT_CLK);

		/* set dpll to need */
		nd_dpll = get_dpll_from_clk(new_clk);
		if (nd_dpll == 0){
			/* must be not go to this, if running, new_clk error! */
			goto out;
		}
		dpll_clk_set(nd_dpll);
	}

#ifdef CONFIG_SCX35_DMC_FREQ_AP
	__emc_clk_set(new_clk);
#endif

out:
#if defined (EMC_FREQ_AUTO_TEST) || defined(CONFIG_SCX35_DMC_FREQ_AP)
	local_irq_restore(irq_flags);
#endif
	mutex_flg --;
#ifdef EMC_FREQ_AUTO_TEST
	end_t1 = get_sys_cnt();

	current_u_time = (start_t1 - end_t1)/128;
	if(max_u_time < current_u_time) {
		max_u_time = current_u_time;
	}
	printk("emc dfs use  current = %08u max %08u\n", current_u_time, max_u_time);
#endif
	return 0;
}
EXPORT_SYMBOL(emc_clk_set);

#ifdef CONFIG_SCX35_DMC_FREQ_AP
static int emc_dfs_call(unsigned long flag)
{
	cpu_switch_mm(init_mm.pgd, &init_mm);
	((int (*)(unsigned long))SPRD_IRAM0H_PHYS)(flag); //iram0h must be the first function of dfs
	return 0;
}
static void emc_dfs_code_copy(u8 * dest)
{
	memcpy_toio((void *)dest, (void *)emc_dfs_main, 0xc00);
}
#endif


static int __init emc_early_suspend_init(void)
{
#ifdef CONFIG_SCX35_DMC_FREQ_AP
	int ret;
	emc_dfs_code_copy((u8 *)SPRD_IRAM0H_BASE);
	ret = ioremap_page_range(SPRD_IRAM0H_PHYS,
				SPRD_IRAM0H_PHYS+SZ_4K, SPRD_IRAM0H_PHYS, PAGE_KERNEL_EXEC);
	if (ret) {
		printk("ioremap_page_range err %d\n", ret);
		BUG();
	}
#endif

	return 0;
}

static void  __exit emc_early_suspend_exit(void)
{
}

module_init(emc_early_suspend_init);
module_exit(emc_early_suspend_exit);
