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

static u32 max_clk = 0;
static volatile u32 cp_code_addr = 0x0;
static DEFINE_MUTEX(emc_mutex);
#define CP_DEBUG_ADDR	(cp_code_addr + 0xFF8)
#define CP_FLAGS_ADDR	(cp_code_addr + 0xFFC)
#define CP_PARAM_ADDR	(cp_code_addr + 0xF00)
#define DDR_TIMING_REG_VAL_ADDR	(SPRD_IRAM0H_BASE + 0xc00)

#define UMCTL_REG_BASE (0x30000000)
#define PUBL_REG_BASE  (0x30010000)

#define NINT(FREQ,REFIN)	(FREQ/REFIN)
#define KINT(FREQ,REFIN)	((FREQ-(FREQ/REFIN)*REFIN)*1048576/REFIN)

#define REG32(x)                           (*((volatile u32 *)(x)))

//extern DMC_UMCTL_REG_INFO_PTR_T p_umctl_reg;
//extern DMC_PUBL_REG_INFO_PTR_T gp_publ_reg;

#define debug(format, arg...) pr_debug("emc_freq" "" format, ## arg)
#define info(format, arg...) pr_info("emc_freq: " "" format, ## arg)

static u32 emc_freq = 0;
static u32 emc_delay = 20;
static u32 chip_id = 0;
static ddr_dfs_val_t __emc_param_configs[5];
static void __timing_reg_dump(ddr_dfs_val_t * dfs_val_ptr);
u32 emc_clk_get(void);
static u32 get_dpll_clk(void);
#ifdef CONFIG_SCX35_DMC_FREQ_AP
static void emc_dfs_code_copy(u8 * dest);
static int emc_dfs_call(unsigned long flag);
void emc_dfs_main(unsigned long flag);
#endif
#define MAX_DLL_DISABLE_CLK 200
#define MIN_DPLL_CLK    0x3F
enum{
	CLK_EMC_SELECT_26M = 0,
	CLK_EMC_SELECT_TDPLL = 1,
	CLK_EMC_SELECT_DPLL = 2,
};
//#define EMC_FREQ_AUTO_TEST

#ifndef CONFIG_SCX35_DMC_FREQ_AP
static void close_cp(void)
{
	u32 value;
	u32 times;

	value = __raw_readl(CP_FLAGS_ADDR);
	times = 0;
	while((value & EMC_FREQ_SWITCH_STATUS_MASK) != (EMC_FREQ_SWITCH_COMPLETE << EMC_FREQ_SWITCH_STATUS_OFFSET)) {
		value = __raw_readl(CP_FLAGS_ADDR);
		mdelay(2);
		if(times >= 100) {
			break;
		}
		times ++;
	}
	info("__emc_clk_set flag =  0x%08x , version flag = 0x%08x phy register = 0x%08x\n", __raw_readl(CP_FLAGS_ADDR),__raw_readl(CP_FLAGS_ADDR - 4), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x4));
	udelay(200);
}

#endif
#ifdef EMC_FREQ_AUTO_TEST
static u32 get_sys_cnt(void)
{
	return __raw_readl(SPRD_GPTIMER_BASE + 0x44);
}
#endif

static void __set_dpll_clk(u32 clk)
{
	volatile u32 reg = 0;
	u32 refin = 0;

	if(get_dpll_clk() == clk) {
		return;
	}

	/* get refin value */
	if (clk/13 <= 0x3F) {
		refin = 13;
	}
	else {
		refin = 26;
	}

	/* set kint, nint */
	reg = sci_glb_read(REG_AON_APB_DPLL_CFG1, -1);
	reg |= 1 << 10;			/* set fractional divider */
	reg &=~(0xfffff<<12 | 0x3f);
	reg |= (KINT(clk, refin) & 0xfffff) << 12;
	reg |= (NINT(clk, refin)) & 0x3f;
	sci_glb_write(REG_AON_APB_DPLL_CFG1, reg, -1);

	/* Set REFIN */
	reg = sci_glb_read(REG_AON_APB_DPLL_CFG,-1);
	reg &= ~(BITS_DPLL_REFIN(0x03));
	if (refin = 26) {
		reg |= BITS_DPLL_REFIN(0x03);
	}
	else if (refin = 13){
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

static u32 is_current_set = 0;
static u32 __emc_clk_set(u32 clk, u32 sene, u32 dll_enable, u32 bps_200)
{
	u32 flag = 0;
	volatile u32 i;

	/* check timing params */
#ifdef CONFIG_SCX35_DMC_FREQ_DDR3
	flag = (EMC_DDR_TYPE_DDR3 << EMC_DDR_TYPE_OFFSET) | (clk << EMC_CLK_FREQ_OFFSET);
	flag |= EMC_FREQ_NORMAL_SCENE << EMC_FREQ_SENE_OFFSET;
	flag |= dll_enable;
	flag |= 0x0 << EMC_CLK_DIV_OFFSET;
#else
	flag = (EMC_DDR_TYPE_LPDDR2 << EMC_DDR_TYPE_OFFSET) | (clk << EMC_CLK_FREQ_OFFSET);
	flag |= sene << EMC_FREQ_SENE_OFFSET;
	flag |= dll_enable;
	flag |= bps_200 << EMC_BSP_BPS_200_OFFSET;
#endif

#ifdef CONFIG_SCX35_DMC_FREQ_AP
	if((sene < 2) || (sene > 4)) {
		panic("invalid scene for ap dfs, valid number is only 2,3,4!\n");
		return -1;
	}

	if((sene != EMC_FREQ_NORMAL_SWITCH_SENE) && (clk != 200)) {
		panic("invalid scene clk combination, sleep and resume must be tdpll 200M!\n");
		return -1;
	}

	if(sene == EMC_FREQ_DEEP_SLEEP_SENE)
	{
		__set_dpll_clk(800);
	}

	flush_cache_all();
	printk(" %s, cpu_suspend!\n", __func__);
	cpu_suspend(flag, emc_dfs_call);
#if 1
	printk("ypxie : DDR PUBL REG[0x30010000]\n\r");
	printk("ZQnDR: [0x%x], [0x%x], [0x%x], [0x%x] \n\r", 
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x248), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x258), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x268), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x278), -1));
	printk("ACMDLR: [0x%x]\n\r",  sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x38), -1));
	printk("DXnMDLR: [0x%x], [0x%x], [0x%x], [0x%x] \n\r", 
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x2C4), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x344), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x3C4), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x444), -1));
	printk("ZQnDR: [0x%x], [0x%x], [0x%x], [0x%x] \n\r", 
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x2BC), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x33C), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x3BC), -1),
		sci_glb_read((SPRD_LPDDR2_PHY_BASE + 0x43C), -1));
#endif
	//printk("\n\r %s, over!!!\n", __func__);

	if(sene == EMC_FREQ_RESUME_SENE) {
		__set_dpll_clk(800);
	}
#else
	__raw_writel(flag, (u32)CP_FLAGS_ADDR);

	#ifdef CONFIG_SCX35_DMC_FREQ_CP0
		sci_glb_set(SPRD_IPI_BASE,1 << 0);//send ipi interrupt to cp0
	#endif

	#ifdef CONFIG_SCX35_DMC_FREQ_CP1
		sci_glb_set(SPRD_IPI_BASE,1 << 4);//send ipi interrupt to cp1
	#endif

	#ifdef CONFIG_SCX35_DMC_FREQ_CP2
		sci_glb_set(SPRD_IPI_BASE,1 << 8);//send ipi interrupt to cp2
	#endif
	close_cp();
#endif
	if(emc_clk_get() != clk) {
		info("clk set error, set clk = %d, get clk = %d, sence = %d\n", clk, emc_clk_get(), sene);
	}
	else{
		info("clk set success, set clk = %d, get clk = %d, sence = %d\n", clk, emc_clk_get(), sene);
	}
	return 0;
}

static u32 get_emc_clk_select(u32 clk)
{
	u32 select;
	switch(clk) {
		case 26:
			select = CLK_EMC_SELECT_26M;
			break;
		case 192:
		case 384:
		case 256:
			select = CLK_EMC_SELECT_TDPLL;
			break;
		default:
			select = CLK_EMC_SELECT_DPLL;
			break;
	 }
	return select;
}

u32 emc_clk_set(u32 new_clk, u32 sene)
{
	u32 dll_enable = EMC_DLL_SWITCH_ENABLE_MODE;
	u32 old_clk,old_select,new_select,div = 0x0;

#ifdef EMC_FREQ_AUTO_TEST
	u32 start_t1, end_t1;
	static u32 max_u_time = 0;
	u32 current_u_time;

	start_t1 = get_sys_cnt();
#endif

#if defined (EMC_FREQ_AUTO_TEST) || defined (CONFIG_SCX35_DMC_FREQ_AP)
	unsigned long irq_flags;
	local_irq_save(irq_flags);
#endif

    /*info("emc clk going on %d	#########################################################\n",new_clk);*/
	//mutex_lock(&emc_mutex);
	if(new_clk > max_clk) {
		new_clk = max_clk;
	}
	if(emc_clk_get() == new_clk) {
	//	mutex_unlock(&emc_mutex);
	        goto out;
	}

	if(new_clk <= 200) {
		dll_enable = 0;
	}

	old_clk = emc_clk_get();
	if(is_current_set == 1) {
		panic("now other thread set dmc clk\n");
		goto out;
	}
	is_current_set ++;
	//info("REG_AON_CLK_PUB_AHB_CFG = %x\n", __raw_readl(REG_AON_CLK_PUB_AHB_CFG));
	//info("emc_clk_set old = %d, new = %d\n", old_clk, new_clk);
	//info("emc_clk_set clk = %d, sene = %d, dll_enable = %d, emc_delay = %x\n", new_clk, sene,dll_enable, emc_delay);
#ifdef CONFIG_SCX35_DMC_FREQ_AP
	if (new_clk <= 250) {
		new_clk = 200;
	}
	else if ((new_clk > 200) && (new_clk < 400)) {
		new_clk = 384;
	}
	else/* if (new_clk >= 400)*/ {
		new_clk = 400;
	}

	if(new_clk == old_clk) {
		if(sene == EMC_FREQ_NORMAL_SWITCH_SENE) {
			is_current_set --;
			goto out;
		}
	}
	__emc_clk_set(new_clk,
		sene, EMC_DLL_SWITCH_DISABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
#else
	if( (new_clk >= 0) && (new_clk <= 200) )
	{
		new_clk = 200;
	}
	else if((new_clk > 200) && (new_clk <=400))
	{
		new_clk = 384;
	}

	if(emc_clk_get() == new_clk)
	{
		is_current_set--;
		goto out;
		//return 0;
	}

	if(new_clk <= MAX_DLL_DISABLE_CLK){
		dll_enable = EMC_DLL_SWITCH_DISABLE_MODE;
	}
	old_select = get_emc_clk_select(old_clk);
	new_select = get_emc_clk_select(new_clk);

	switch(new_select)
	{
		case CLK_EMC_SELECT_26M:
			break;
		case CLK_EMC_SELECT_TDPLL:
			__emc_clk_set(new_clk, 0, dll_enable, 0);
			break;
		case CLK_EMC_SELECT_DPLL:
			if(old_select == new_select){
				__emc_clk_set(384, 0, EMC_DLL_SWITCH_ENABLE_MODE,0);
			}
			__set_dpll_clk(new_clk * (div + 1));
			__emc_clk_set(new_clk, 0, dll_enable,0);
			break;
		default:
			break;
	}

#endif
	//mutex_unlock(&emc_mutex);
	is_current_set --;

out:
#ifdef EMC_FREQ_AUTO_TEST
	end_t1 = get_sys_cnt();

	current_u_time = (start_t1 - end_t1)/128;
	if(max_u_time < current_u_time) {
		max_u_time = current_u_time;
	}
	info("**************emc dfs use  current = %08u max %08u\n", current_u_time, max_u_time);
#endif

#if defined (EMC_FREQ_AUTO_TEST) || defined(CONFIG_SCX35_DMC_FREQ_AP)
	local_irq_restore(irq_flags);
#endif
	return 0;
}
EXPORT_SYMBOL(emc_clk_set);

static u32 get_dpll_clk(void)
{
	u32 clk;
	u32 kint, refin, pnt = 0 ;
	u32 reg;

	reg = sci_glb_read(REG_AON_APB_DPLL_CFG1, -1);
	kint = ((reg>>12) & 0xfffff);
	clk = (reg & 0x3F);

	reg = sci_glb_read(REG_AON_APB_DPLL_CFG, -1);
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

	if (((kint * refin ) & 0xFFFFF) >= 0x80000) {
		pnt = 1;
	}
	else {
		pnt = 0;
	}
	kint = ((kint * refin) >> 20) + pnt;
	clk = clk * refin + kint;
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
			pll_clk = get_dpll_clk();
			break;
		default:
			break;
	}

	clk = (pll_clk / (div + 1)) >> 1;
	return clk;
}
EXPORT_SYMBOL(emc_clk_get);

static int debugfs_emc_freq_get(void *data, u64 * val)
{
	u32 freq;
	freq = emc_clk_get();
	info("debugfs_emc_freq_get %d\n",freq);
	*(u32 *) data = *val = freq;
	return 0;
}
static int debugfs_emc_freq_set(void *data, u64 val)
{
	int new_freq = *(u32 *) data = val;
	emc_clk_set(new_freq, 0);
	return 0;
}

static int debugfs_emc_delay_get(void *data, u64 * val)
{
	*(u32 *) data = *val = emc_delay;
	return 0;
}
static int debugfs_emc_delay_set(void *data, u64 val)
{
	emc_delay = *(u32 *) data = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_emc_freq,
			debugfs_emc_freq_get, debugfs_emc_freq_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_emc_delay,
			debugfs_emc_delay_get, debugfs_emc_delay_set, "%llu\n");
static u32 get_spl_emc_clk_set(void)
{
	return emc_clk_get();
}
static void emc_debugfs_creat(void)
{
	static struct dentry *debug_root = NULL;
	debug_root = debugfs_create_dir("emc", NULL);
	if (IS_ERR_OR_NULL(debug_root)) {
		printk("%s return %p\n", __FUNCTION__, debug_root);
	}
	debugfs_create_file("freq", S_IRUGO | S_IWUGO,
			    debug_root, &emc_freq, &fops_emc_freq);
	debugfs_create_file("delay", S_IRUGO | S_IWUGO,
			    debug_root, &emc_delay, &fops_emc_delay);
}

#ifdef EMC_FREQ_AUTO_TEST
#ifdef CONFIG_SCX35_DMC_FREQ_DDR3
static u32 emc_freq_valid_array[] = {
	200,
	384,
	400
};
#else
static u32 emc_freq_valid_array[] = {
	//100,
	200,
	332,
	384,
	532,
};
#endif //CONFIG_SCX35_DMC_FREQ_DDR3
static struct wake_lock emc_freq_test_wakelock;
static int emc_freq_test_thread(void * data)
{
	u32 i = 0;
	wake_lock(&emc_freq_test_wakelock);
	msleep(20000);
	//p_umctl_reg = (DMC_UMCTL_REG_INFO_PTR_T)UMCTL_REG_BASE;
	//gp_publ_reg = (DMC_PUBL_REG_INFO_PTR_T) PUBL_REG_BASE;
	while(1){
		set_current_state(TASK_INTERRUPTIBLE);
		i = get_random_int();
		i = i % (sizeof(emc_freq_valid_array)/ sizeof(emc_freq_valid_array[0]));
		printk("emc_freq_test_thread i = %x\n", i);
		emc_clk_set(emc_freq_valid_array[i], EMC_FREQ_NORMAL_SWITCH_SENE);
		schedule_timeout(10);
	}
	return 0;
}

static void __emc_freq_test(void)
{
	struct task_struct * task;
	wake_lock_init(&emc_freq_test_wakelock, WAKE_LOCK_SUSPEND,
			"emc_freq_test_wakelock");
	task = kthread_create(emc_freq_test_thread, NULL, "emc freq test test");
	if (task == 0) {
		printk("Can't crate emc freq test thread!\n");
	}else {
		wake_up_process(task);
	}
}
#endif

static void __timing_reg_dump(ddr_dfs_val_t * dfs_val_ptr)
{
	debug("umctl2_rfshtmg %x\n", dfs_val_ptr->ddr_clk);
	debug("umctl2_rfshtmg %x\n", dfs_val_ptr->umctl2_rfshtmg);
	debug("umctl2_dramtmg0 %x\n", dfs_val_ptr->umctl2_dramtmg0);
	debug("umctl2_dramtmg1 %x\n", dfs_val_ptr->umctl2_dramtmg1);
	debug("umctl2_dramtmg2 %x\n", dfs_val_ptr->umctl2_dramtmg2);
	debug("umctl2_dramtmg3 %x\n", dfs_val_ptr->umctl2_dramtmg3);
	debug("umctl2_dramtmg4 %x\n", dfs_val_ptr->umctl2_dramtmg4);
	debug("umctl2_dramtmg5 %x\n", dfs_val_ptr->umctl2_dramtmg5);
	debug("umctl2_dramtmg6 %x\n", dfs_val_ptr->umctl2_dramtmg6);
	debug("umctl2_dramtmg7 %x\n", dfs_val_ptr->umctl2_dramtmg7);
	debug("umctl2_dramtmg8 %x\n", dfs_val_ptr->umctl2_dramtmg8);
	debug("publ_dx0gcr %x\n", dfs_val_ptr->publ_dx0gcr);
	debug("publ_dx1gcr %x\n", dfs_val_ptr->publ_dx1gcr);
	debug("publ_dx2gcr %x\n", dfs_val_ptr->publ_dx2gcr);
	debug("publ_dx3gcr %x\n", dfs_val_ptr->publ_dx3gcr);
	debug("publ_dx0dqstr %x\n", dfs_val_ptr->publ_dx0dqstr);
	debug("publ_dx1dqstr %x\n", dfs_val_ptr->publ_dx1dqstr);
	debug("publ_dx2dqstr %x\n", dfs_val_ptr->publ_dx2dqstr);
	debug("publ_dx3dqstr %x\n", dfs_val_ptr->publ_dx3dqstr);
}
static void __emc_timing_reg_init(void)
{
	ddr_dfs_val_t * dfs_val_ptr;
	ddr_dfs_val_t *dmc_timing_ptr;
	u32 i;
	memset(__emc_param_configs, 0, sizeof(__emc_param_configs));
	for(i = 0; i < 5; i++) {
		dfs_val_ptr = (ddr_dfs_val_t *)(DDR_TIMING_REG_VAL_ADDR + i * sizeof(ddr_dfs_val_t));
		dmc_timing_ptr = 0;
		if((dfs_val_ptr->ddr_clk >= 100) && (dfs_val_ptr->ddr_clk <= 533)) {
			dmc_timing_ptr = &__emc_param_configs[i];
		}
		if(dfs_val_ptr->ddr_clk == 200) {
			dfs_val_ptr->ddr_clk = 200;
		}
//		if(dfs_val_ptr->ddr_clk == 533) {
//			dfs_val_ptr->ddr_clk = 532;
//		}
		if(dfs_val_ptr->ddr_clk == 400) {
			dfs_val_ptr->ddr_clk = 400;
		}
		if(dmc_timing_ptr) {
			memcpy(dmc_timing_ptr, dfs_val_ptr, sizeof(*dfs_val_ptr));
		}
	}
	for(i = 0; i < 5; i++) {
		__timing_reg_dump(&__emc_param_configs[i]);
	}
}
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

static int dmcfreq_pm_notifier_do(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	printk("*** %s, event:0x%lx ***\n", __func__, event );

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/*
		* DMC must be set 200MHz before deep sleep in ES chips
		*/
		emc_clk_set(200, EMC_FREQ_NORMAL_SWITCH_SENE);        //nomarl switch to tdpll   192
		emc_clk_set(200, EMC_FREQ_DEEP_SLEEP_SENE);        //deep sleep tdpll192 to dpll   192
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		/* Reactivate */
		emc_clk_set(200, EMC_FREQ_RESUME_SENE);       //resume dpll192 -- tdpll192
		emc_clk_set(max_clk, EMC_FREQ_NORMAL_SWITCH_SENE);   //nomarl tdpll192 -- dpll332
		return (int)NOTIFY_OK;
	}
	printk("*** %s, event:0x%lx done***\n", __func__, event );

	return (int)NOTIFY_DONE;
}

static struct notifier_block dmcfreq_pm_notifier = {
	.notifier_call = dmcfreq_pm_notifier_do,
};


static int __init emc_early_suspend_init(void)
{
	max_clk = get_spl_emc_clk_set();
//	chip_id = __raw_readl(REG_AON_APB_CHIP_ID);
	__emc_timing_reg_init();
#ifdef CONFIG_SCX35_DMC_FREQ_AP
	int ret;
	emc_dfs_code_copy((u8 *)SPRD_IRAM0H_BASE);
	ret = ioremap_page_range(SPRD_IRAM0H_PHYS, SPRD_IRAM0H_PHYS+SZ_4K, SPRD_IRAM0H_PHYS, PAGE_KERNEL_EXEC);
	if (ret) {
		printk("ioremap_page_range err %d\n", ret);
		BUG();
	}
#else
//	cp_init();
#endif

	/* if DFS is not configurated, we keep ddr 200MHz when screen off */
#ifndef CONFIG_SPRD_SCX35_DMC_FREQ
	register_pm_notifier(&dmcfreq_pm_notifier);
#endif
	emc_debugfs_creat();
#ifdef EMC_FREQ_AUTO_TEST
	__emc_freq_test();
#endif
	return 0;
}
static void  __exit emc_early_suspend_exit(void)
{
#ifndef CONFIG_SPRD_SCX35_DMC_FREQ
	unregister_pm_notifier(&dmcfreq_pm_notifier);
#endif
}

module_init(emc_early_suspend_init);
module_exit(emc_early_suspend_exit);
