/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/regs_global.h>
#include <mach/adi_hal_internal.h>
#include <mach/cpufreq_sc88xxg.h>

#define FREQ_TABLE_ENTRY				(5)
#define TDPLL_FREQ                      (786*MHz)
#define XTL_FREQ                        (26*MHz)

static struct proc_dir_entry *sprd_cpufreq_proc_entry;
static DEFINE_MUTEX(sprd_cpufreq_mutex);
#ifdef CONFIG_MACH_SP6820A
int cpufreq_bypass = 1;
#endif
#ifdef CONFIG_MACH_SP8810
int cpufreq_bypass = 0;
#endif
struct task_struct *cpufreq_thread;


static struct sprd_dvfs_table sc8810g_dvfs_table[] = {
	[0] = { 1000 , 1200 }, /* 1000MHz,  1200mv */
	[1] = { 800 , 1200 },  /* 800MHz,  1200mv */
	[2] = { 600 , 1200 },  /* 600MHz,  1200mv */
	[3] = { 400 , 1100 },  /* 400MHz,  1200mv */
};

static struct cpufreq_frequency_table sc8810g_freq_table[FREQ_TABLE_ENTRY]; 

struct sprd_dvfs_table current_cfg;
struct clock_state {
	struct sprd_dvfs_table  current_para;
	struct mutex			lock;
}drv_state;

static unsigned long get_xtl_freq(void){

	return XTL_FREQ;
}

static unsigned long get_tdpll_freq(void){

	return TDPLL_FREQ;
}

static unsigned long get_dpll_freq(void){
   u32 dpll_refin, dpll_n, dpll_freq;

   dpll_refin = (REG_GR_DPLL_MN>>GR_DPLL_REFIN_SHIFT) & GR_DPLL_REFIN_MASK;
   switch(dpll_refin){
   	   case 0:
	   	     dpll_refin = GR_DPLL_REFIN_2M;
		     break;
	   case 1:
	   case 2:
			 dpll_refin = GR_DPLL_REFIN_4M;
			 break;
	   case 3:
		     dpll_refin = GR_DPLL_REFIN_13M;
	         break;
       default:
	   	     printk("%s ERROR mpll_refin:%d\n", __func__, dpll_refin);
   }
   dpll_n = REG_GR_DPLL_MN & GR_DPLL_N_MASK;
   dpll_freq = dpll_refin * dpll_n;
   DBG("dpll_freq:%u, dpll_n:%u, dpll_refin:u\n", dpll_freq, dpll_n, dpll_refin);

   return dpll_freq;
}


//return Hz
static unsigned long get_mpll_freq(void){
   u32 mpll_refin, mpll_n, mpll_freq;

   mpll_refin = (REG_GR_MPLL_MN>>GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
   switch(mpll_refin){
   	   case 0:
	   	     mpll_refin = GR_MPLL_REFIN_2M;
		     break;
	   case 1:
	   case 2:
			 mpll_refin = GR_MPLL_REFIN_4M;
			 break;
	   case 3:
		     mpll_refin = GR_MPLL_REFIN_13M;
	         break;
       default:
	   	     printk("%s ERROR mpll_refin:%d\n", __func__, mpll_refin);
   }
   mpll_n = REG_GR_MPLL_MN & GR_MPLL_N_MASK;
   mpll_freq = mpll_refin * mpll_n;
   DBG("mpll_freq:%u, mpll_n:%u, mpll_refin:%u\n", mpll_freq, mpll_n, mpll_refin);

   return mpll_freq;
}


static unsigned long get_emc_src(void){
   u32 emc_sel, emc_src;

   emc_sel = (REG_AHB_ARM_CLK>>CLK_EMC_SEL_SHIFT) & CLK_EMC_SEL_MASK;
   DBG("mcu_sel:%u\n", emc_sel);
   switch(emc_sel){
      case 0:
	  	   emc_src = get_mpll_freq( ) / 2;
		   break;
	  case 1:
	  	   emc_src = get_dpll_freq( );
		   break;
	  case 2:
	  	   emc_src = get_tdpll_freq( ) / 3;
		   break;
	  case 3:
	  	   emc_src = get_xtl_freq( );
		   break;
	  default:
	  	   printk("%s ERROR emc_src:%d\n", __func__, emc_src);	      
   }

   return emc_src;
}

//return Hz
static unsigned long get_mcu_src(void){
   u32 mcu_sel, mcu_src;

   mcu_sel = (REG_AHB_ARM_CLK>>CLK_MCU_SEL_SHIFT) & CLK_MCU_SEL_MASK;
   DBG("mcu_sel:%u\n", mcu_sel);
   switch(mcu_sel){
      case 0:
	  	   mcu_src = get_mpll_freq( );
		   break;
	  case 1:
	  	   mcu_src = get_tdpll_freq( ) / 2;
		   break;
	  case 2:
	  	   mcu_src = get_tdpll_freq( ) / 3;
		   break;
	  case 3:
	  	   mcu_src = get_xtl_freq( );
		   break;
	  default:
	  	   printk("%s ERROR mcu_sel:%d\n", __func__, mcu_sel);	      
   }

   return mcu_src;
}


/* return is : Hz */
static unsigned long get_mcu_clk(void)
{
	int clk_mcu_src, clk_mcu, clk_mcu_div;
	
	clk_mcu_div = REG_AHB_ARM_CLK  & CLK_MCU_DIV_MASK;
	clk_mcu_src = get_mcu_src( );
	clk_mcu = clk_mcu_src / (clk_mcu_div+1);
	
    DBG("clk_mcu:%u, clk_mcu_src:%u, clk_mcu_div:%u\n", clk_mcu, clk_mcu_src, clk_mcu_div);
	return clk_mcu;

}

/* return is : Hz */
static unsigned long get_ahb_clk(void)
{

	u32 ahb_div, ahb_src, ahb_clk;

    ahb_src = get_mcu_clk();    	
	ahb_div = (REG_AHB_ARM_CLK >> CLK_AHB_DIV_SHIFT) & CLK_AHB_DIV_MASK;
    ahb_clk = ahb_src / (ahb_div+1);
	
    DBG("ahb_clk:%u, ahb_src:%u, ahb_div:u\n", ahb_clk, ahb_src, ahb_div);
    return ahb_clk;

}

static unsigned long get_emc_clk(void){
    u32 emc_div, emc_src, emc_clk;

	emc_src = get_emc_src();
	emc_div = 0; //suppose 0, can not get emc_div from spec
	emc_clk = emc_src / (emc_div+1);

	
    DBG("emc_clk:%u, emc_src:%u, emc_div:u\n", emc_clk, emc_src, emc_div);
    return emc_clk;
}

static u32 get_mcu_vdd(void){
    u32 mcu_vdd_sel, mcu_vdd;

	mcu_vdd = 0;
	mcu_vdd_sel = ANA_REG_GET(ANA_DCDCARM_CTRL) & DCDCARM_CTL_MASK;
	switch(mcu_vdd_sel){
        case 0:
			 mcu_vdd = 650;
			 break;
		case 1:
			 mcu_vdd = 700;
			 break;
		case 2:
			 mcu_vdd = 800;
			 break;
	    case 3:
			 mcu_vdd = 900;
			 break;
		case 4:
			 mcu_vdd = 1000;
			 break;
		case 5:
			 mcu_vdd = 1100;
			 break;
		case 6:
			 mcu_vdd = 1200;
			 break;
		case 7:
			 mcu_vdd = 1300;
			 break;
		default:
		     printk("incorrect mcu_vdd_sel:%u\n", mcu_vdd_sel);
	}

	return mcu_vdd;
}


/* return is arm clock : Hz */
static unsigned long clk_get_rate(void)
{

	unsigned long arm_clk;
	
	arm_clk = get_mcu_clk();
	return arm_clk;

}


static void set_mcu_vdd(unsigned long vdd_mcu_mv){

    unsigned long vdd_mcu_sel, vdd_mcu_cfg;
    if(vdd_mcu_mv == current_cfg.vdd_mcu_mv){
       return;
    }
#if 0
	if(vdd_mcu_mv == ARM_MIN_VDD){
       vdd_mcu_sel = 0;
	}else{
       vdd_mcu_sel = 7 - ((ARM_MAX_VDD-vdd_mcu_mv)%ARM_VDD_STEP);
	}
    vdd_mcu_cfg = ANA_REG_GET(ANA_DCDCARM_CTRL);
	vdd_mcu_cfg &= ~MCU_VDD_SEL_MASK;
	vdd_mcu_cfg |= vdd_mcu_sel;
	ANA_REG_SET(ANA_DCDCARM_CTRL, vdd_mcu_cfg);
#endif

    return;
}


/*
*  rate is MHz
*/
static void set_dpll_freq(unsigned long rate){
   u32 dpll_refin, dpll_n, dpll_cfg;

   dpll_cfg = REG_GR_DPLL_MN;
   dpll_refin = (REG_GR_DPLL_MN>>GR_DPLL_REFIN_SHIFT) & GR_DPLL_REFIN_MASK;
   switch(dpll_refin){
   	   case 0:
	   	     dpll_refin = GR_DPLL_REFIN_2M;
		     break;
	   case 1:
	   case 2:
			 dpll_refin = GR_DPLL_REFIN_4M;
			 break;
	   case 3:
		     dpll_refin = GR_DPLL_REFIN_13M;
	         break;
       default:
	   	     printk("%s ERROR dpll_refin:%d\n", __func__, dpll_refin);
   }
   dpll_refin /= MHz;
   dpll_n = rate / dpll_refin;
   dpll_cfg &= ~GR_DPLL_N_MASK;
   dpll_cfg |= dpll_n;
 
   DBG("before, dpll_cfg:%u\n", REG_GR_DPLL_MN);
//   REG_GR_GEN1 |= BIT_9;
   REG_GR_DPLL_MN = dpll_cfg;
//   REG_GR_GEN1 &= ~BIT_9;
   DBG("after, mpll_cfg:%u, mpll_n:%u, mpll_refin:%u\n", REG_GR_DPLL_MN, dpll_n, dpll_refin);
   return;
}

/*
*  rate is MHz
*/
static void set_mpll_freq(unsigned long rate){
   u32 mpll_refin, mpll_n, mpll_cfg;

   mpll_cfg = REG_GR_MPLL_MN;
   mpll_refin = (REG_GR_MPLL_MN>>GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
   switch(mpll_refin){
   	   case 0:
	   	     mpll_refin = GR_MPLL_REFIN_2M;
		     break;
	   case 1:
	   case 2:
			 mpll_refin = GR_MPLL_REFIN_4M;
			 break;
	   case 3:
		     mpll_refin = GR_MPLL_REFIN_13M;
	         break;
       default:
	   	     printk("%s ERROR mpll_refin:%d\n", __func__, mpll_refin);
   }
   mpll_refin /= MHz;
   mpll_n = rate / mpll_refin;
   mpll_cfg &= ~GR_MPLL_N_MASK;
   mpll_cfg |= mpll_n;
 
   DBG("before, mpll_cfg:%u\n", REG_GR_MPLL_MN);
   REG_GR_GEN1 |= BIT_9;
   REG_GR_MPLL_MN = mpll_cfg;
   REG_GR_GEN1 &= ~BIT_9;
   DBG("after, mpll_cfg:%u, mpll_n:%u, mpll_refin:%u\n", REG_GR_MPLL_MN, mpll_n, mpll_refin);
   return;
}

/* rate is arm clock : Hz */
static int clk_set_rate(unsigned long mcu_clk, unsigned long mcu_vdd)
{
        int i;
        mcu_clk /= 1000; //MHz
   
	
	if(mcu_clk > current_cfg.clk_mcu_mhz){
           set_mcu_vdd(mcu_vdd);
	   set_mpll_freq(mcu_clk);
	}
   	if(mcu_clk < current_cfg.clk_mcu_mhz){
	   set_mpll_freq(mcu_clk);
	   set_mcu_vdd(mcu_vdd);
	}
	
	for (i = 0; i < 100; i++);

	current_cfg.clk_mcu_mhz = mcu_clk;
	current_cfg.vdd_mcu_mv  = mcu_vdd;
         
	return 0; 
}

/*static void set_armcore_voltage(armcore_voltage_e core_voltage)
{
	unsigned long pwr;	

	pwr = REG_GR_DCDC_CTL & 0xfffffffc;
	pwr |= (BIT_8 | core_voltage);
	REG_GR_DCDC_CTL = pwr;
	REG_GR_DCDC_CTL &= ~BIT_8;
}*/

static void cpufreq_table_init(int cnt){

	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt++) {
		sc8810g_freq_table[cnt].index = 0;
		sc8810g_freq_table[cnt].frequency = CPUFREQ_TABLE_END;
	}
	
    for (cnt = 0; cnt < FREQ_TABLE_ENTRY-1; cnt++) {
     sc8810g_freq_table[cnt].index = cnt;
	 sc8810g_freq_table[cnt].frequency = sc8810g_dvfs_table[cnt].clk_mcu_mhz * 1000;
    }

#if 0    
	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt ++){
		DBG("sc8810g_freq_table[cnt].index = %d\n", cnt, sc8810g_freq_table[cnt].index);
		DBG("sc8810g_freq_table[%d].frequency = %d\n", cnt, sc8810g_freq_table[cnt].frequency);
	}
#endif

	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt ++)
		printk("sc8810g_freq_table[%d].frequency = %d\n", cnt, sc8810g_freq_table[cnt].frequency);

	return;
}

static void cpufreq_state_init(struct clock_state *state){
	state->current_para.clk_mcu_mhz = get_mcu_clk();
    state->current_para.vdd_mcu_mv  = get_mcu_vdd();
	mutex_init(&drv_state.lock);

    
	DBG("current_cfg.clk_mcu_mhz:%dMHz, current_cfg.vdd_mcu_mv:%dmv", current_cfg.clk_mcu_mhz, current_cfg.vdd_mcu_mv);
	return;
}

static int sprd_cpufreq_keep_max(void){
	DBG("%s, get input event\n", __func__);
#if 0
	if(cpufreq_bypass){
			DBG("cpufreq_bypass:%d, return without setting cpufreq\n", cpufreq_bypass);
			return -1;
    }else{
	       mutex_trylock(&drv_state.lock);		   
		   DBG("%s, call clk_set_rate, and set cpufreq_bypass\n", __func__);
		   cpufreq_bypass = 1;
           clk_set_rate(MAX_FREQ, ARM_MAX_VDD_USED);
		   msleep(100);
           mutex_unlock(&drv_state.lock);
           return 0;
    }
#endif
    if(!cpufreq_bypass && (cpufreq_thread->state!=TASK_RUNNING)){
	   DBG("%s, wake_up_process cpufreq_thread\n", __func__);	
       wake_up_process(cpufreq_thread);
	}else{
	   DBG("%s, cpufreq_thread is running\n", __func__);         
	}
	return 0;
}

static void sprd_cpufreq_bypass(int bypass){
        if(bypass){
		   printk("stop cpufreq, and keep cpufreq 1GHz\n");
           sprd_cpufreq_keep_max( );
		   cpufreq_bypass = 1;
        }
		else
		   cpufreq_bypass = 0;

		return;
}


static int sprd_cpufreq_open(struct inode* inode, struct file* file){
        return 0;
}

static int sprd_cpufreq_release(struct inode* inode, struct file* file){
        return 0;
}

static loff_t sprd_cpufreq_llseek(struct file* file, loff_t off, int whence){
        return 0;
}

static ssize_t sprd_cpufreq_read(struct file* file, char* buf, size_t count, loff_t* ppos){
        return 0;
}

static ssize_t sprd_cpufreq_write(struct file* file, const char* buf, size_t size, loff_t* ppos){
        
	char cmd[3];
    unsigned long cpufreq = 0;
        
	if(*ppos){
	     printk("============= *ppos=%d ================\n", *ppos);
	     return -EINVAL;
        }
        
	if(copy_from_user(cmd, buf, size)){
	     printk("=========== copy_from_user failed ==========\n");
             return -EFAULT;
        }
	cmd[size] = '\0';
	printk("========== cmd[0]:%c ================\n", cmd[0]);
	mutex_lock(&sprd_cpufreq_mutex);
        //switch(cmd[0]){
        switch(*cmd){
	     case '0':
		 	 sprd_cpufreq_bypass(1);  
	         break;
		 case '1':
		 	 sprd_cpufreq_bypass(0);
			 //printk("touch event, keep max freq \n");
		 	 //sprd_cpufreq_keep_max( );
	         break;
		 case 'd':
			  cpufreq = get_mcu_clk();
			  printk("current cpu frequency:%uHz\n", cpufreq);
			  break;
	     default :
	          printk("!!!! invalid command !!!!!");
        }		   

	mutex_unlock(&sprd_cpufreq_mutex);
	return size;
}

static struct file_operations sprd_cpufreq_fops = {
    open : sprd_cpufreq_open,
	release : sprd_cpufreq_release,
	llseek : sprd_cpufreq_llseek,
	read : sprd_cpufreq_read,
	write : sprd_cpufreq_write,
};

static struct proc_dir_entry*  
sprd_cpufreq_proc_create(struct proc_dir_entry* parent, char* name, struct file_operations* fops){
    struct proc_dir_entry *proc;
	proc = create_proc_entry(name, S_IFREG|S_IRUGO|S_IWUSR, parent);
	if(!proc){
	   printk("!!!! create_proc_entry failed !!!!\n");
	   return NULL;
        }

	proc->proc_fops = fops;
	return proc;
}

static void sprd_cpufreq_proc_mkdir(unsigned long data){
    DBG("************************************\n");
	printk("**** create current proc file *******\n");
	sprd_cpufreq_proc_entry = proc_mkdir("sprd_cpufreq", NULL);
        if(!sprd_cpufreq_proc_entry){
            printk("!!!!!! proc_mkdir sprd_cpufreq failed !!!!\n");
            return;            
        }
	
        sprd_cpufreq_proc_create(sprd_cpufreq_proc_entry, "state", &sprd_cpufreq_fops);
	return;
}


static int cpufreq_catch_tp(void *d){
  
	while(1){	
	  if(cpufreq_bypass){
		DBG("cpufreq_bypass:%d, return without setting cpufreq\n", cpufreq_bypass);
	  }else{
	        mutex_lock(&drv_state.lock);		   
		cpufreq_bypass = 1;
                clk_set_rate(MAX_FREQ, ARM_MAX_VDD_USED);
		msleep(100);
		cpufreq_bypass = 0;
                mutex_unlock(&drv_state.lock);
		printk("%s, call set cpufreq max, and set cpufreq_bypass\n", __func__);
				
	  }
          set_current_state(TASK_INTERRUPTIBLE);
          schedule();
	}

	return 0;
}

static int sc8800g_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, sc8810g_freq_table);
}

static unsigned int sc8800g_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;
	return clk_get_rate() / 1000;
}

static int sc8800g_cpufreq_set_target_max(struct cpufreq_policy *policy,
				                                      unsigned int target_freq){

    sprd_cpufreq_keep_max();

	return 0;
} 


static int sc8800g_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	unsigned long ret;
	unsigned int i;
	struct cpufreq_freqs freqs;
	struct sprd_dvfs_table *target;
	ret = cpufreq_frequency_table_target(policy, sc8810g_freq_table,
					     target_freq, relation, &i);
	if (ret != 0)
		return ret;
	DBG("target_freq = %d  i = %d\n", target_freq, i);
	freqs.cpu = 0;
	freqs.old = clk_get_rate() / 1000; //KHz
	freqs.new = sc8810g_freq_table[i].frequency;
	freqs.flags = 0;
//	target = &sc8810g_dvfs_table[sc8810g_freq_table[i].index];
	target = &sc8810g_dvfs_table[i];
	if (freqs.old == freqs.new)
		return 0;

	/* perhaps, we need change lcd and emc pll etc. */
	DBG("current_cfg.clk_mcu_mhz:%dMHz, current_cfg.vdd_mcu_mv:%dmv\n", current_cfg.clk_mcu_mhz, current_cfg.vdd_mcu_mv);
        if( !mutex_trylock(&drv_state.lock) ){
            printk("drv_state.lock is already locked\n");
	}else{
	   if(!cpufreq_bypass){		   	  
		  printk("cpufreq: Transition %d-%dkHz\n", freqs.old, freqs.new);
		  cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		  DBG("target->clk_mcu_mhz:%uMHz, target->vdd_mcu_mv:%umv.\n", target->clk_mcu_mhz, target->vdd_mcu_mv );
		  DBG("freqs.new:%uKHz.\n", freqs.new); 	 
		  ret = clk_set_rate(freqs.new, target->vdd_mcu_mv);	  
		  cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);		  
		  printk("cpufreq: Set actual frequency %lukHz\n", clk_get_rate() / 1000);
	   }
	   	mutex_unlock(&drv_state.lock);
	}
	
	DBG("current_cfg.clk_mcu_mhz:%dMHz, current_cfg.vdd_mcu_mv:%dmv\n", current_cfg.clk_mcu_mhz, current_cfg.vdd_mcu_mv);

	return 0;
}

static int sc8800g_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0)
		return -EINVAL;

	if (sc8810g_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}

	policy->cur = clk_get_rate() / 1000; /* current cpu frequency : KHz*/
	policy->cpuinfo.transition_latency = 1 * 1000 * 1000;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(sc8810g_freq_table, policy->cpu);
#endif

	ret = cpufreq_frequency_table_cpuinfo(policy, sc8810g_freq_table);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}
	
	return ret;
}

static struct freq_attr *sc8800g_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sc8800g_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= sc8800g_cpufreq_verify_speed,
	.target		= sc8800g_cpufreq_set_target,
	.target_max		= sc8800g_cpufreq_set_target_max,
	.get		= sc8800g_cpufreq_get_speed,
	.init		= sc8800g_cpufreq_driver_init,
	.name		= "sc8800g",
	.attr		= sc8800g_cpufreq_attr,
};

static int __init sc8800g_cpufreq_init(void)
{
	unsigned int  clk, arm_clk;

	printk("\nREG_CHIP_ID = 0x%08x\n", REG_CHIP_ID);
	printk("REG_AHB_ARM_CLK = 0x%08x\n", REG_AHB_ARM_CLK);
	printk("REG_GR_MPLL_MN = 0x%08x\n", REG_GR_MPLL_MN);
	printk("REG_GR_GEN1 = 0x%08x\n", REG_GR_GEN1);
	printk("ANA_DCDC_CTL = 0x%08x\n", ANA_REG_GET(ANA_DCDC_CTRL));

        get_mcu_clk();
	get_emc_clk();
	get_ahb_clk();
	
	arm_clk = clk_get_rate() / 1000; //KHz
	if (arm_clk < MIN_FREQ) {
		printk("arm clock is too low, cpufreq is not needed.\n");
		return 0;
	}
	if (arm_clk > MAX_FREQ) {
		printk("arm clock is too big beyond frequency table, please add frequency table entry.\n");
		return 0;
	}

        cpufreq_table_init(clk);
	cpufreq_state_init(&drv_state);

        cpufreq_thread = kthread_run(cpufreq_catch_tp, NULL, "cpufreq_catch_input");
        current_cfg = drv_state.current_para;
/*  
        current_cfg.clk_mcu_mhz = get_mcu_clk();
	current_cfg.vdd_mcu_mv  = get_mcu_vdd();
*/	

        sprd_cpufreq_proc_mkdir(1);

	return cpufreq_register_driver(&sc8800g_cpufreq_driver);
}
module_init(sc8800g_cpufreq_init);
