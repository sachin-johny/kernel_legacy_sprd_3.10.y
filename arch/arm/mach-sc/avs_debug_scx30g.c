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
#include <mach/adi.h>
#include <linux/random.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <linux/vmalloc.h>
#include <linux/printk.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

struct avs_hw_ctrl
{
	volatile u32 avs_tune_lmt_cfg;
	volatile u32 avs_tune_status;
	volatile u32 avs_cfg;
	volatile u32 avs_tune_step_cfg;
	volatile u32 avs_wait_cfg;
	volatile u32 avs_int_cfg;
	volatile u32 avs_start_single_act;
	volatile u32 avs_start_repeat_act;
	volatile u32 avs_hpm_en;
	volatile u32 avs_hpm_rpt_anls;
	volatile u32 avs_hpm_rpt_vld_status[4];
	volatile u32 avs_hpm_dly_wind_sel;
	volatile u32 avs_fsm_sts;
};

struct sprd_avs_info
{
	struct avs_hw_ctrl *chip;
	struct timer_list avs_timer;
	struct dentry *debug_avs_dir;
	volatile u32 printfreq;
	u32 window;
};

static struct sprd_avs_info sai;

static void avs_init(void)
{
	/* avs enable */
	sci_glb_set(REG_AON_APB_APB_EB1,1 << 6);
}

static unsigned int get_gen_pll_freq(unsigned int pll_cfg0,unsigned int pll_cfg1)
{
	unsigned int pll_n,pll_ref,pll_kint,ret;
	unsigned int ref[4]={2,4,13,26};

	pll_ref    = (pll_cfg0 >> 24)&0x3;

	if(pll_cfg1 & (1 << 10)){
		pll_n   = pll_cfg1 & 0x3f;
		pll_kint= (pll_cfg1 >> 12) & 0xfffff;
		ret     = ref[pll_ref] * pll_n + ((ref[pll_ref] * pll_kint) >> 0x14);
	} else {
		pll_n   = pll_cfg0 & 0x7ff;
		ret     = pll_n * ref[pll_ref];
	}

	return ret;
}

static unsigned int get_ca7_freq(void)
{
#if 0
	unsigned int ca7_clk_cfg = sci_glb_read(REG_AP_AHB_CA7_CKG_CFG,-1UL);
	unsigned int ca7_clk_src,ca7_clk_div,pll_cfg[2],ret;
	ca7_clk_src = ca7_clk_cfg & 0x7;
	ca7_clk_div = (ca7_clk_cfg >> 4) & 0x7;
	switch(ca7_clk_src)
	{
		case 0:
			ret = 26;
			goto calc_finish;
			break;
		case 1:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_DPLL_CFG,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_DPLL_CFG1,-1UL);
			break;
		case 2:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_CPLL_CFG,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_CPLL_CFG1,-1UL);
			break;
		case 3:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_TDPLL_CFG,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_TDPLL_CFG1,-1UL);
			break;
		case 4:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_WIFIPLL1_CFG,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_WIFIPLL1_CFG1,-1UL);
			break;
		case 5:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_WPLL_CFG0,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_WPLL_CFG1,-1UL);
			break;
		case 6:
			pll_cfg[0] = sci_glb_read(REG_AON_APB_MPLL_CFG,-1UL);
			pll_cfg[1] = sci_glb_read(REG_AON_APB_MPLL_CFG1,-1UL);
			break;
		default:
			ret = 0x0;
			goto calc_finish;
			break;
	}

	ret = get_gen_pll_freq(pll_cfg[0],pll_cfg[1]);

calc_finish:
	return ret/(ca7_clk_div+1);
#else
	return 0x0;
#endif
}

static void avs_trigger(void)
{
	volatile unsigned int i;
	/*first soft reset avs module*/
	sci_glb_set(REG_AON_APB_APB_RST1,(1 << 5));
	for(i=0;i<100;i++);
	sci_glb_clr(REG_AON_APB_APB_RST1,(1 << 5));
	/* avs hpm enable */
	sai.chip->avs_hpm_en |= (1 << 1);
	/* avs tune limit cfg */
	sai.chip->avs_tune_lmt_cfg = 0xff80;
	/* avs hpm rpt anls */
	sai.chip->avs_hpm_rpt_anls = 0x3FF94;
	/* start avs */
	sai.chip->avs_start_repeat_act = 0x1;
	/* delay for something */
	udelay(5);
}

unsigned int g_avs_log_flag = 0;

static void avs_timer_func(unsigned long data)
{
	static u32 arm_voltage_int[]={1100,700,800,900,1000,650,1200,1300};
	u32 reg_cal = (sci_adi_read(ANA_REG_GLB_DCDC_ARM_ADI) & 0x1F);
	u32 reg_int = ((sci_adi_read(ANA_REG_GLB_DCDC_ARM_ADI) >> 0x5) & 0x1F) ;
	u32 int_part,frc_part;
	int_part = arm_voltage_int[reg_int & 0x7];

	frc_part = (reg_cal & 0x1F)*3;

	avs_trigger();
	if(g_avs_log_flag)
	{
		printk("****************************avs debug**********************************\r\n");
		sai.window = sai.chip->avs_hpm_rpt_vld_status[0];
		printk("avs window 0x%x arm voltage is %d mv ca7 freq is %d Mhz\r\n",sai.window,(int_part + frc_part),get_ca7_freq());
		printk("****************************avs debug**********************************\r\n");
	}
	mod_timer(&sai.avs_timer,jiffies + msecs_to_jiffies(sai.printfreq + 1000));
}

static int debugfs_avs_printfreq_get(void *data, u64 * val)
{
	*(u32 *) data = *val = sai.printfreq;
	return 0;
}
static int debugfs_avs_printfreq__set(void *data, u64 val)
{
	sai.printfreq = *(u32 *) data = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_avs_printfreq,
			debugfs_avs_printfreq_get, debugfs_avs_printfreq__set, "%llu\n");

static void avs_debugfs_create(void)
{
	sai.debug_avs_dir = debugfs_create_dir("avs_dir", NULL);
	if (IS_ERR_OR_NULL(sai.debug_avs_dir)) {
		pr_err("%s return %p\n", __FUNCTION__, sai.debug_avs_dir);
	}
	debugfs_create_file("avs", S_IRUGO | S_IWUGO,
			    sai.debug_avs_dir, &sai.printfreq, &fops_avs_printfreq);
}

static int __init avs_debug_init(void)
{
	sai.chip = (struct avs_hw_ctrl*)SPRD_AVSCA7_BASE;

	init_timer(&sai.avs_timer);
	sai.avs_timer.function = avs_timer_func;
	sai.avs_timer.expires  = jiffies + msecs_to_jiffies(2000);
	sai.avs_timer.data     = (u32)&sai;
	
	avs_init();

	avs_debugfs_create();

	add_timer(&sai.avs_timer);
	return 0;
}

static void __exit avs_debug_exit(void)
{
	del_timer_sync(&sai.avs_timer);
}
module_init(avs_debug_init);
module_exit(avs_debug_exit);
