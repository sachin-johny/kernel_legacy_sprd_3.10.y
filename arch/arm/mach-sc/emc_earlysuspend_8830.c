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
static u32 max_clk = 0;
static volatile u32 cp_code_init_ref = 0x0;
static volatile u32 cp_code_addr = 0x0;
static volatile u32 switch_cnt =0x0;
static DEFINE_MUTEX(emc_mutex);
#define CP_DEBUG_ADDR	(cp_code_addr + 0xFF8)
#define CP_FLAGS_ADDR	(cp_code_addr + 0xFFC)
#define CP_PARAM_ADDR	(cp_code_addr + 0xF00)
#define DDR_TIMING_REG_VAL_ADDR	(SPRD_IRAM0H_BASE + 0xc00)

#define debug(format, arg...) pr_debug("emc_freq" "" format, ## arg)
#define info(format, arg...) pr_info("emc_freq: " "" format, ## arg)

static u32 emc_freq = 0;
static u32 emc_delay = 20;
static u32 chip_id = 0;
static ddr_dfs_val_t __emc_param_configs[5];
static void __timing_reg_dump(ddr_dfs_val_t * dfs_val_ptr);
u32 emc_clk_get(void);
#ifdef CONFIG_SCXX30_AP_DFS
static void emc_dfs_code_copy(u8 * dest);
static int emc_dfs_call(unsigned long flag);
void emc_dfs_main(unsigned long flag);
#endif
//#define EMC_FREQ_AUTO_TEST
static ddr_dfs_val_t *__dmc_param_config(u32 clk)
{
	u32 i;
	ddr_dfs_val_t * ret_timing = 0;
	if(clk == 533) {
		clk = 532;
	}
	if(clk == 333) {
		clk = 332;
	}
	for(i = 0; i < (sizeof(__emc_param_configs) / sizeof(__emc_param_configs[0])); i++) {
		if(__emc_param_configs[i].ddr_clk >= clk) {
			ret_timing = &__emc_param_configs[i];
			break;
		}
	}
	//__timing_reg_dump(ret_timing);
	if(ret_timing) {
#ifdef CONFIG_SCXX30_AP_DFS
		memcpy((void *)DDR_TIMING_REG_VAL_ADDR, ret_timing, sizeof(ddr_dfs_val_t));
#else
		memcpy((void *)CP_PARAM_ADDR, ret_timing, sizeof(ddr_dfs_val_t));
#endif
	}
	return ret_timing;
}
#ifndef CONFIG_SCXX30_AP_DFS
static void cp_code_init(void)
{
	u32 *copy_data;
	u32 copy_size;
	u32 code_phy_addr;
#ifdef CONFIG_DFS_AT_CP0
	copy_data = cp0_dfs_code_data;
	copy_size = sizeof(cp0_dfs_code_data);
	code_phy_addr = 0x50000000;
#else
	copy_data = cp2_dfs_code_data;
	copy_size = sizeof(cp2_dfs_code_data);
	code_phy_addr = 0x50003000;
#endif
	cp_code_addr = (volatile u32)ioremap(code_phy_addr,0x1000);
	memcpy((void *)cp_code_addr, (void *)copy_data, copy_size);
}
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
	info("__emc_clk_set flag =  0x%08x ,- 4 = 0x%08x phy register = 0x%08x\n", __raw_readl(CP_FLAGS_ADDR),__raw_readl(CP_FLAGS_ADDR - 4), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x4));
	udelay(200);
}
static void wait_cp_run(void)
{
	u32 val;
	u32 times = 0;
	val = __raw_readl(CP_FLAGS_ADDR - 4);
	while(val != 0x11223344/*cp2 run flag*/) {
		mdelay(2);
		val = __raw_readl(CP_FLAGS_ADDR - 4);
		times ++;
		if(times >= 10) {
			panic("wait_cp2_run timeout\n");
		}
	}
	__raw_writel(0x44332211, CP_FLAGS_ADDR - 4)/*for watchdog, so clear it*/;
}
#endif
#ifdef EMC_FREQ_AUTO_TEST
static u32 get_sys_cnt(void)
{
	return __raw_readl(SPRD_GPTIMER_BASE + 0x44);
}
#endif
static u32 is_current_set = 0;
static u32 __emc_clk_set(u32 clk, u32 sene, u32 dll_enable, u32 bps_200)
{
	u32 flag = 0;
	ddr_dfs_val_t * ret_timing;
	ret_timing = __dmc_param_config(clk);
	if(!ret_timing) {
		return -1;
	}
	clk = ret_timing->ddr_clk;
	if(clk == 533) {
		clk = 532;
	}
	if(clk == 333) {
		clk = 332;
	}
	flag = (EMC_DDR_TYPE_LPDDR2 << EMC_DDR_TYPE_OFFSET) | (clk << EMC_CLK_FREQ_OFFSET);
	flag |= EMC_FREQ_NORMAL_SCENE << EMC_FREQ_SENE_OFFSET;
	flag |= dll_enable;
	flag |= bps_200 << EMC_BSP_BPS_200_OFFSET;
#ifdef CONFIG_SCXX30_AP_DFS
	flush_cache_all();
	cpu_suspend(flag, emc_dfs_call);
#else
	__raw_writel(flag, CP_FLAGS_ADDR);
#ifdef CONFIG_DFS_AT_CP0
	sci_glb_set(SPRD_IPI_BASE,1 << 0);//send ipi interrupt to cp0
#else
	sci_glb_set(SPRD_IPI_BASE,1 << 8);//send ipi interrupt to cp2
#endif
	close_cp();
	info("__emc_clk_set clk = %d REG_AON_APB_DPLL_CFG = %x, PUBL_DLLGCR = %x\n",clk, sci_glb_read(REG_AON_APB_DPLL_CFG, -1), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x04));
#endif
	if(emc_clk_get() != clk) {
		info("clk set error, set clk = %d, get clk = %d\n", clk, emc_clk_get());
	}
	return 0;
}
u32 emc_clk_set(u32 new_clk, u32 sene)
{
	u32 dll_enable = 1;
	u32 old_clk;
#ifdef EMC_FREQ_AUTO_TEST
	u32 start_t1, end_t1;
	unsigned long irq_flags;
	static u32 max_u_time = 0;
	u32 current_u_time;
	old_clk = emc_clk_get();
	local_irq_save(irq_flags);
	local_irq_disable();
	start_t1 = get_sys_cnt();
	local_fiq_disable();
#endif
	//mutex_lock(&emc_mutex);
	if(new_clk > max_clk) {
		new_clk = max_clk;
	}
	if(emc_clk_get() == new_clk) {
	//	mutex_unlock(&emc_mutex);
		return 0;
	}
	if(new_clk <= 200) {
		dll_enable = 0;
	}
	old_clk = emc_clk_get();
	if(is_current_set == 1) {
		panic("now other thread set dmc clk\n");
		return 0;
	}
	is_current_set ++;
	//info("REG_AON_CLK_PUB_AHB_CFG = %x\n", __raw_readl(REG_AON_CLK_PUB_AHB_CFG));
	//info("emc_clk_set old = %d, new = %d\n", old_clk, new_clk);
	//info("emc_clk_set clk = %d, sene = %d, dll_enable = %d, emc_delay = %x\n", new_clk, sene,dll_enable, emc_delay);
	if((old_clk > 200) && (new_clk == 200)) {
		if(old_clk > 332) {
			__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
		__emc_clk_set(200, 0, EMC_DLL_SWITCH_DISABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
	}
	else if((old_clk == 200) && (new_clk > 200)) {
#ifdef CONFIG_SCXX30_AP_DFS
		__emc_clk_set(322, 0, EMC_DLL_SWITCH_ENABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
#else
		__emc_clk_set(200, 0, EMC_DLL_SWITCH_ENABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
#endif
		if(new_clk > 332) {
			__emc_clk_set(new_clk, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
	}
	else {
		__emc_clk_set(new_clk, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
	}
	//mutex_unlock(&emc_mutex);
	is_current_set --;
#ifdef EMC_FREQ_AUTO_TEST
	local_fiq_enable();
	end_t1 = get_sys_cnt();
	local_irq_enable();
	local_irq_restore(irq_flags);

	current_u_time = (start_t1 - end_t1)/128;
	if(max_u_time < current_u_time) {
		max_u_time = current_u_time;
	}
	info("**************emc dfs use  current = %08u max %08u\n", current_u_time, max_u_time);
#endif
	info("__emc_clk_set REG_AON_APB_DPLL_CFG = %x, PUBL_DLLGCR = %x\n",sci_glb_read(REG_AON_APB_DPLL_CFG, -1), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x04));
	return 0;
}
EXPORT_SYMBOL(emc_clk_set);

static u32 get_dpll_clk(void)
{
	u32 clk;
	u32 reg;
	reg = sci_glb_read(REG_AON_APB_DPLL_CFG, -1);
	clk = reg & 0x7ff;
	if((reg & 0x03000000) == 0x00000000) {
		clk *= 2;
	}
	if((reg & 0x03000000) == 0x01000000) {
		clk *= 4;
	}
	if((reg & 0x03000000) == 0x02000000) {
		clk *= 13;
	}
	if((reg & 0x03000000) == 0x03000000) {
		clk *= 26;
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
#if defined(CONFIG_ARCH_SCX15)
#else
	reg_val = sci_glb_read(REG_AON_CLK_EMC_CFG, -1);
	sel = reg_val & 0x3;
	div = (reg_val >> 8) & 0x3;
	switch(sel) {
	case 0:
		pll_clk = 26;
		break;
	case 1:
		pll_clk = 256;
		break;
	case 2:
		pll_clk = 384;
		break;
	case 3:
		pll_clk = get_dpll_clk();
		break;
	default:
		break;
	}
	clk = pll_clk / (div + 1);
#endif
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
static void emc_earlysuspend(struct early_suspend *h)
{
#ifndef EMC_FREQ_AUTO_TEST
	emc_clk_set(200, EMC_FREQ_NORMAL_SCENE);
#endif
}
static void emc_late_resume(struct early_suspend *h)
{
#ifndef EMC_FREQ_AUTO_TEST
	emc_clk_set(max_clk, EMC_FREQ_NORMAL_SCENE);
#endif
}
static struct early_suspend emc_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100,
	.suspend = emc_earlysuspend,
	.resume = emc_late_resume,
};
#ifdef EMC_FREQ_AUTO_TEST
static u32 emc_freq_valid_array[] = {
	//100,
	200,
	332,
	400,
	532,
};
static struct wake_lock emc_freq_test_wakelock;
static int emc_freq_test_thread(void * data)
{
	u32 i = 0;
	wake_lock(&emc_freq_test_wakelock);
	msleep(20000);
	while(1){
		set_current_state(TASK_INTERRUPTIBLE);
		i = get_random_int();
		i = i % (sizeof(emc_freq_valid_array)/ sizeof(emc_freq_valid_array[0]));
		printk("emc_freq_test_thread i = %x\n", i);
		emc_clk_set(emc_freq_valid_array[i], 0);
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
//#define DDR_TIMING_REG_VAL_ADDR	(SPRD_IRAM0_BASE + 0x1c00)
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
		if(dfs_val_ptr->ddr_clk == 333) {
			dfs_val_ptr->ddr_clk = 332;
		}
		if(dfs_val_ptr->ddr_clk == 533) {
			dfs_val_ptr->ddr_clk = 532;
		}
		if(dmc_timing_ptr) {
			memcpy(dmc_timing_ptr, dfs_val_ptr, sizeof(*dfs_val_ptr));
		}
	}
	for(i = 0; i < 5; i++) {
		__timing_reg_dump(&__emc_param_configs[i]);
	}
}
#ifdef CONFIG_SCXX30_AP_DFS
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

#else
static void cp_init(void)
{
#if defined(CONFIG_ARCH_SCX15)
#else
	cp_code_init();
#ifdef CONFIG_DFS_AT_CP0 //dfs is at cp0
	sci_glb_set(REG_PMU_APB_CP_SOFT_RST, 1 << 0);//reset cp0
	udelay(200);
	sci_glb_clr(REG_PMU_APB_PD_CP0_SYS_CFG, 1 << 25);//power on cp0
	mdelay(4);
	sci_glb_clr(REG_PMU_APB_PD_CP0_SYS_CFG, 1 << 28);//close cp0 force sleep
	mdelay(2);
	sci_glb_clr(REG_PMU_APB_CP_SOFT_RST, 1 << 0);//release cp0
#else
	sci_glb_set(REG_PMU_APB_CP_SOFT_RST, 1 << 2);//reset cp2
	udelay(200);
	sci_glb_clr(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 25);//power on cp2
	mdelay(4);
	sci_glb_clr(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 28);//close cp2 force sleep
	mdelay(2);
	sci_glb_clr(REG_PMU_APB_CP_SOFT_RST, 1 << 2);//reset cp2
#endif
	wait_cp_run();
#endif
}
#endif
static int __init emc_early_suspend_init(void)
{
#if defined(CONFIG_ARCH_SCX15)
#else
	//u32 val;
	//__raw_writel(1, REG_AON_CLK_PUB_AHB_CFG);
	//__raw_writel(3, REG_AON_CLK_AON_APB_CFG);
	//val = __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x02c);
	//val &= ~(1 << 4);
	//__raw_writel(val,SPRD_LPDDR2_PHY_BASE + 0x02c);

	max_clk = get_spl_emc_clk_set();
	chip_id = __raw_readl(REG_AON_APB_CHIP_ID);
	//cp_code_init();
	__emc_timing_reg_init();
#ifndef CONFIG_SCXX30_AP_DFS
	cp_init();
#else
	int ret;
	emc_dfs_code_copy((u8 *)SPRD_IRAM0H_BASE);
	ret = ioremap_page_range(SPRD_IRAM0H_PHYS, SPRD_IRAM0H_PHYS+SZ_4K, SPRD_IRAM0H_PHYS, PAGE_KERNEL_EXEC);
	if(ret){
		printk("ioremap_page_range err %d\n", ret);
		BUG();
	}
#endif
	/*
	* if DFS is not configurated, we keep ddr 200MHz when screen off
	*/
#ifndef CONFIG_SPRD_SCX35_DMC_FREQ
	register_early_suspend(&emc_early_suspend_desc);
#endif
	emc_debugfs_creat();
#ifdef EMC_FREQ_AUTO_TEST
	__emc_freq_test();
#endif
#endif
	return 0;
}
static void  __exit emc_early_suspend_exit(void)
{
#ifndef CONFIG_SPRD_SCX35_DMC_FREQ
	unregister_early_suspend(&emc_early_suspend_desc);
#endif
}

module_init(emc_early_suspend_init);
module_exit(emc_early_suspend_exit);
