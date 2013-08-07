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
#define EMC_DLL_SWITCH_DISABLE_MODE		0x0
#define EMC_DLL_SWITCH_ENABLE_MODE		0x1
#define EMC_DLL_NOT_SWITCH_MODE			0x2
#define EMC_DLL_MODE_MASK			(0xf)

#define EMC_FREQ_SWITCH_STATUS_OFFSET		(4)
#define EMC_FREQ_SWITCH_COMPLETE			(1)
#define EMC_FREQ_SWITCH_STATUS_MASK			(0xf << EMC_FREQ_SWITCH_STATUS_OFFSET)

#define EMC_DDR_TYPE_LPDDR1			0
#define EMC_DDR_TYPE_LPDDR2			1
#define EMC_DDR_TYPE_LPDDR3			2
#define EMC_DDR_TYPE_DDR2			3
#define EMC_DDR_TYPE_DDR3			4
#define EMC_DDR_TYPE_OFFSET			8
#define EMC_DDR_TYPE_MASK			(0xf << EMC_DDR_TYPE_OFFSET)

#define EMC_CLK_FREQ_OFFSET			(12)
#define EMC_CLK_FREQ_MASK			(0xfff << EMC_CLK_FREQ_OFFSET)

#define EMC_FREQ_NORMAL_SCENE			0x0 //normal lcd power off
#define EMC_FREQ_MP4_SENE			0x1 //play mp4 mode
#define EMC_FREQ_SENE_OFFSET			24
#define EMC_FREQ_SENE_MASK			(0xf << EMC_FREQ_SENE_OFFSET)

#define EMC_BSP_BPS_200_CLR			0x0
#define EMC_BSP_BPS_200_SET			0x1
#define EMC_BSP_BPS_200_NOT_CHANGE		0x2
#define EMC_BSP_BPS_200_OFFSET			28
#define EMC_BSP_BPS_200_MASK			(0xf << EMC_BSP_BPS_200_OFFSET)

#define DMC_CHANGE_FREQ_WAIT_TIMEOUT		100
static u32 max_clk = 0;
static volatile u32 cp_code_init_ref = 0x0;
static volatile u32 cp_code_addr = 0x0;
static volatile u32 switch_cnt =0x0;
static DEFINE_MUTEX(emc_mutex);
u32 emc_clk_get(void);
#if 0
#define CP2_FLAGS_ADDR	(SPRD_IRAM1_BASE + 0x3FFC)
#define CP2_PARAM_ADDR	(SPRD_IRAM1_BASE + 0x3F00)
#else
#define CP2_DEBUG_ADDR	(cp_code_addr + 0xFF8)
#define CP2_FLAGS_ADDR	(cp_code_addr + 0xFFC)
#define CP2_PARAM_ADDR	(cp_code_addr + 0xF00)
#endif
#define uint32 u32

#define debug(format, arg...) pr_debug("emc_freq" "" format, ## arg)
#define info(format, arg...) pr_info("emc_freq: " "" format, ## arg)
typedef struct
{
    uint32 ddr_clk;
    //umctl reg
    uint32 umctl2_rfshtmg;
    uint32 umctl2_init0;
    uint32 umctl2_init1;
    uint32 umctl2_init2;
    uint32 umctl2_init3;
    uint32 umctl2_init4;
    uint32 umctl2_init5;
    uint32 umctl2_dramtmg0;
    uint32 umctl2_dramtmg1;
    uint32 umctl2_dramtmg2;
    uint32 umctl2_dramtmg3;
    uint32 umctl2_dramtmg4;
    uint32 umctl2_dramtmg5;
    uint32 umctl2_dramtmg6;
    uint32 umctl2_dramtmg7;
    uint32 umctl2_dramtmg8;
    uint32 umctl2_dfitmg0;
    uint32 umctl2_dfitmg1;
    //publ reg
    uint32 publ_ptr0;
    uint32 publ_ptr1;
    uint32 publ_dtpr0;
    uint32 publ_dtpr1;
    uint32 publ_dtpr2;
    uint32 publ_mr0;
    uint32 publ_mr1;
    uint32 publ_mr2;
    uint32 publ_mr3;
    uint32 publ_dx0gcr;
    uint32 publ_dx1gcr;
    uint32 publ_dx2gcr;
    uint32 publ_dx3gcr;
    uint32 publ_dx0dqstr;
    uint32 publ_dx1dqstr;
    uint32 publ_dx2dqstr;
    uint32 publ_dx3dqstr;
}ddr_dfs_val_t;
static u32 emc_freq = 0;
static u32 emc_delay = 20;
static u32 chip_id = 0;
static ddr_dfs_val_t __emc_param_configs[5];
static void __timing_reg_dump(ddr_dfs_val_t * dfs_val_ptr);
u32 emc_clk_get(void);
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
		memcpy((void *)CP2_PARAM_ADDR, ret_timing, sizeof(ddr_dfs_val_t));
	}
	return ret_timing;
}
static void cp_code_init(void)
{
	if(!cp_code_init_ref) {
		cp_code_addr = (volatile u32)ioremap(0x50003000,0x1000);
		if(chip_id == 0) {
			memcpy((void *)cp_code_addr, cp_code_data_cs, sizeof(cp_code_data_cs));
		}
		else {
			//memcpy((void *)SPRD_IRAM1_BASE + (12 * 1024), cp_code_data_cs, sizeof(cp_code_data_cs));
			memcpy((void *)cp_code_addr, cp_code_data_cs, sizeof(cp_code_data_cs));
		}
	}
}
static void close_cp(void)
{
	u32 value;
	u32 times;

	value = __raw_readl(CP2_FLAGS_ADDR);
	times = 0;
	while((value & EMC_FREQ_SWITCH_STATUS_MASK) != (EMC_FREQ_SWITCH_COMPLETE << EMC_FREQ_SWITCH_STATUS_OFFSET)) {
		value = __raw_readl(CP2_FLAGS_ADDR);
		mdelay(2);
		if(times >= 100) {
			break;
		}
		times ++;
	}
	info("__emc_clk_set flag =  0x%08x ,- 4 = 0x%08x phy register = 0x%08x\n", __raw_readl(CP2_FLAGS_ADDR),__raw_readl(CP2_FLAGS_ADDR - 4), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x4));
	info("__emc_clk_set REG_AON_APB_DPLL_CFG = %x, PUBL_DLLGCR = %x\n",sci_glb_read(REG_AON_APB_DPLL_CFG, -1), __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x10));
	//info("__emc_clk_set flag REG_AON_CLK_EMC_CFG = 0x%08x\n", __raw_readl(REG_AON_CLK_EMC_CFG));
	//sci_glb_set(REG_PMU_APB_CP_SOFT_RST, 1 << 2);//reset cp2
	udelay(200);
	//sci_glb_set(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 28);//cp2 force sleep
	//for(i = 0; i < 0x1000; i++);
	//sci_glb_set(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 25);//power off cp2
}
static void wait_cp2_run(void)
{
	u32 val;
	u32 times = 0;
	val = __raw_readl(CP2_FLAGS_ADDR - 4);
	while(val != 0x11223344/*cp2 run flag*/) {
		mdelay(2);
		val = __raw_readl(CP2_FLAGS_ADDR - 4);
		times ++;
		if(times >= 10) {
			panic("wait_cp2_run timeout\n");
		}
	}
	__raw_writel(0x44332211, CP2_FLAGS_ADDR - 4)/*for watchdog, so clear it*/;
}
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
	cp_code_init();
	info("__emc_clk_set clk = %d,  dll_enable = %d, bps_200 = %x\n", clk, dll_enable, bps_200);
	flag = (EMC_DDR_TYPE_LPDDR2 << EMC_DDR_TYPE_OFFSET) | (clk << EMC_CLK_FREQ_OFFSET);
	flag |= EMC_FREQ_NORMAL_SCENE << EMC_FREQ_SENE_OFFSET;
	flag |= dll_enable;
	flag |= bps_200 << EMC_BSP_BPS_200_OFFSET;
	if(!cp_code_init_ref) {
		sci_glb_set(REG_PMU_APB_CP_SOFT_RST, 1 << 2);//reset cp2
		udelay(200);
		sci_glb_clr(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 25);//power on cp2
		mdelay(4);
		sci_glb_clr(REG_PMU_APB_PD_CP2_SYS_CFG, 1 << 28);//close cp2 force sleep
		mdelay(2);
		__raw_writel(flag, CP2_FLAGS_ADDR);
		udelay(200);
		sci_glb_clr(REG_PMU_APB_CP_SOFT_RST, 1 << 2);//reset cp2
		udelay(200);
		wait_cp2_run();
		cp_code_init_ref++;
	}
	__raw_writel(flag, CP2_FLAGS_ADDR);
	sci_glb_set(SPRD_IPI_BASE,1 << 8);//send ipi interrupt to cp2
	close_cp();
	if(emc_clk_get() != clk) {
		info("clk set error, set clk = %d, get clk = %d\n", emc_clk_get());
	}
	return 0;
}
u32 emc_clk_set(u32 new_clk, u32 sene)
{
	u32 dll_enable = 1;
	u32 old_clk;
	mutex_lock(&emc_mutex);
	if(new_clk > max_clk) {
		new_clk = max_clk;
	}
	if(emc_clk_get() == new_clk) {
		mutex_unlock(&emc_mutex);
		return 0;
	}
	if(new_clk <= 200) {
		dll_enable = 0;
	}
	old_clk = emc_clk_get();
	//info("REG_AON_CLK_PUB_AHB_CFG = %x\n", __raw_readl(REG_AON_CLK_PUB_AHB_CFG));
	//info("emc_clk_set old = %d, new = %d\n", old_clk, new_clk);
	//info("emc_clk_set clk = %d, sene = %d, dll_enable = %d, emc_delay = %x\n", new_clk, sene,dll_enable, emc_delay);
	if(new_clk == 100) {
		if(old_clk > 332) {
			__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
		if(old_clk > 200) {
			__emc_clk_set(200, 0, EMC_DLL_SWITCH_DISABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
		__emc_clk_set(100, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_CLR);
	}
	else if(old_clk == 100) {
		if(new_clk == 200) {
			__emc_clk_set(200, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_SET);
		}
		else {
			__emc_clk_set(200, 0, EMC_DLL_SWITCH_ENABLE_MODE, EMC_BSP_BPS_200_SET);
		}
		if(new_clk > 200) {
			__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
		if(new_clk > 332) {
			__emc_clk_set(new_clk, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
	}
	else if((old_clk > 200) && (new_clk == 200)) {
		if(old_clk > 332) {
			__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
		__emc_clk_set(new_clk, 0, EMC_DLL_SWITCH_DISABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
	}
	else if((old_clk == 200) && (new_clk > 200)) {
		__emc_clk_set(200, 0, EMC_DLL_SWITCH_ENABLE_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		__emc_clk_set(332, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		if(new_clk > 332) {
			__emc_clk_set(new_clk, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
		}
	}
	else {
		__emc_clk_set(new_clk, 0, EMC_DLL_NOT_SWITCH_MODE, EMC_BSP_BPS_200_NOT_CHANGE);
	}
	mutex_unlock(&emc_mutex);
	return 0;
}
EXPORT_SYMBOL(emc_clk_set);

static u32 get_dpll_clk(void)
{
	u32 clk;
	clk = sci_glb_read(REG_AON_APB_DPLL_CFG, -1);
	clk &= 0x7ff;
	clk *= 4;
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
//#define EMC_FREQ_AUTO_TEST
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
	msleep(2000);
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
#define DDR_TIMING_REG_VAL_ADDR	(SPRD_IRAM0_BASE + 0x1c00)
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
		if(dmc_timing_ptr) {
			memcpy(dmc_timing_ptr, dfs_val_ptr, sizeof(*dfs_val_ptr));
		}
	}
	for(i = 0; i < 5; i++) {
		__timing_reg_dump(&__emc_param_configs[i]);
	}
}
static int __init emc_early_suspend_init(void)
{
	//u32 val;
	//__raw_writel(1, REG_AON_CLK_PUB_AHB_CFG);
	//__raw_writel(3, REG_AON_CLK_AON_APB_CFG);
	//val = __raw_readl(SPRD_LPDDR2_PHY_BASE + 0x02c);
	//val &= ~(1 << 4);
	//__raw_writel(val,SPRD_LPDDR2_PHY_BASE + 0x02c);

	max_clk = get_spl_emc_clk_set();
	chip_id = __raw_readl(REG_AON_APB_CHIP_ID);
	cp_code_init();
	__emc_timing_reg_init();
	/*
	* move this early_suspend to dfs governor(governor_ondemand.c)
	* TODO: clean code
	register_early_suspend(&emc_early_suspend_desc);
	*/
	emc_debugfs_creat();
#ifdef EMC_FREQ_AUTO_TEST
	__emc_freq_test();
#endif
	return 0;
}
static void  __exit emc_early_suspend_exit(void)
{
	unregister_early_suspend(&emc_early_suspend_desc);
}

module_init(emc_early_suspend_init);
module_exit(emc_early_suspend_exit);
