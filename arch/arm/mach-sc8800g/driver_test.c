

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/pm_devices.h>
#include <mach/test.h>

#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/bits.h>


#define DEEP_SLEEP_INTERVAL (6 * HZ)
#define DEEP_SLEEP_INTERVAL2 (8 * HZ)

//#define CURRENT_PROC

static void deep_sleep_timeout(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer, deep_sleep_timeout, 0, 0);
static void deep_sleep_timeout2(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer2, deep_sleep_timeout2, 0, 0);

//#ifdef CURRENT_PROC
static struct proc_dir_entry *current_proc_entry;
static DEFINE_MUTEX(current_mutex);
//#endif

static u32 sleep_time_local1 = 0, sleep_time_local2 = 0;
extern u32 sleep_time;
static void deep_sleep_timeout(unsigned long data)
{
	struct timespec now;

	getnstimeofday(&now);


	sleep_time_local1 = sleep_time_local2;
	sleep_time_local2 = sleep_time;

	printk("## [deep_sleep_timeout : %lld] ## : jiffies = %lu, sleep_time = %u.\n", 
		ktime_to_ns(ktime_get()), jiffies, (sleep_time_local2 - sleep_time_local1));
	printk("[## [deep_sleep_timeout : gettimeofday() = %lld ].\n", 
		timespec_to_ns(&now));
	printk("##: wall_to_monotonic = %lld.\n", timespec_to_ns(&wall_to_monotonic));

	mod_timer(&deep_sleep_timer, jiffies + DEEP_SLEEP_INTERVAL);
}


static void deep_sleep_timeout2(unsigned long data)
{
	struct timespec now;

	getnstimeofday(&now);


	sleep_time_local1 = sleep_time_local2;
	sleep_time_local2 = sleep_time;

	printk("## [deep_sleep_timeout2 : %lld] ## : jiffies = %lu, sleep_time = %u.\n", 
		ktime_to_ns(ktime_get()), jiffies, (sleep_time_local2 - sleep_time_local1));
	printk("[## [deep_sleep_timeout2 : gettimeofday() = %lld ].\n", 
		timespec_to_ns(&now));
	printk("##: wall_to_monotonic = %lld.\n", timespec_to_ns(&wall_to_monotonic));

	mod_timer(&deep_sleep_timer2, jiffies + DEEP_SLEEP_INTERVAL2);
}

/***
 * * added by wong, 02/07/11
 * * print the value of relevent registers when calling
***/
static void test_current_thread(void *pdata){
    while(1){
        printk("*********** AHB_CTL0:0X%x ***********\n", __raw_readl(AHB_CTL0) );
	printk("*********** GEN0:0X%x ***********\n", __raw_readl(GR_GEN0) );
	printk("*********** BUSCLK:0X%x ***********\n", __raw_readl(GR_BUSCLK) );
	printk("*********** CLK_EN0:0X%x ***********\n", __raw_readl(GR_CLK_EN) );
	printk("*********** ANA_LDO_PD_SET:0X%x ***********\n", __raw_readl(ANA_LDO_PD_SET) );
	printk("*********** ANA_LDO_PD_CTL:0X%x ***********\n", __raw_readl(ANA_LDO_PD_CTL) );
        printk("*********** ANA_ANA_CTL0:0X%x ***********\n", __raw_readl(ANA_ANA_CTL0) );
	printk("*********** ANA_PA_CTL:0X%x ***********\n", __raw_readl(ANA_PA_CTL) );
	printk("=== === === === ==== ==== ==== === === ===\n");
	printk("=== === === === ==== ==== ==== === === ===\n");


	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(5*HZ);
    }
}

/***********************
 * added by wong, 12/07/11 
 * disable some clk
 * ********************/
static void display_clks(void){
        printk("**** display various clocks ***********");
	printk("=== === === === ==== ==== ==== === === ===\n");
	printk("=== === === === ==== ==== ==== === === ===\n");
        printk("*********** AHB_CTL0:0X%x ***********\n", __raw_readl(AHB_CTL0) );
	printk("*********** GEN0:0X%x ***********\n", __raw_readl(GR_GEN0) );
	printk("*********** BUSCLK:0X%x ***********\n", __raw_readl(GR_BUSCLK) );
	printk("*********** CLK_EN0:0X%x ***********\n", __raw_readl(GR_CLK_EN) );
	printk("*********** ANA_LDO_PD_SET:0X%x ***********\n", __raw_readl(ANA_LDO_PD_SET) );
	printk("*********** ANA_LDO_PD_CTL:0X%x ***********\n", __raw_readl(ANA_LDO_PD_CTL) );
        printk("*********** ANA_ANA_CTL0:0X%x ***********\n", __raw_readl(ANA_ANA_CTL0) );
	printk("*********** ANA_PA_CTL:0X%x ***********\n", __raw_readl(ANA_PA_CTL) );
	printk("*********** GR_STC_STATE:0X%x ***********\n", __raw_readl(GR_STC_STATE) );
	printk("*********** GR_CLK_DLY:0X%x ***********\n", __raw_readl(GR_CLK_DLY) );
	printk("=== === === === ==== ==== ==== === === ===\n");
	printk("=== === === === ==== ==== ==== === === ===\n");
}
static void disable_usb_clk(void){
        printk("**** disable_usb_clk ****\n");
	__raw_bits_and(~BIT_5, AHB_CTL0);
        __raw_bits_and(~BIT_9, GR_CLK_GEN5);//disable USB LDO 
	printk( "**** after disable_usb_clk, AHB_CTL0:%08x ****\n", __raw_readl(AHB_CTL0) );

	return;
}

static void disable_sdio_clk(void){
        printk("**** disable_sdio_clk ****\n");
	__raw_bits_and(~BIT_4, AHB_CTL0);
	__raw_bits_and(~BIT_11, AHB_CTL0);//disable bus monitor1 clk
	__raw_bits_and(~BIT_7, AHB_CTL0);//disable bus monitor0 clk
       // ANA_REG_SET(ANA_LDO_PD_SET, BIT_2);//power down SDIO LDO
	printk( "**** after disable sdio clk, AHB_CTL0:%08x ****\n", __raw_readl(AHB_CTL0) );

	return;
}

static void disable_rotation_clk(void){
        printk("**** disable_rotation_clk ****\n");
	__raw_bits_and(~BIT_14, AHB_CTL0);
	printk( "**** after disable rotaion clk, AHB_CTL0:%08x ****\n", __raw_readl(AHB_CTL0) );

	return;
}

static void disable_vibrator_clk(void){
        printk("**** disable_vibrator_clk ****\n");
        ANA_REG_SET(ANA_ANA_CTL0, BIT_10);//power down vibrator
	printk( "**** after disable vibrator clk, ANA_ANA_CTL0:%08x ****\n", ANA_REG_GET(ANA_ANA_CTL0) );

	return;
}

static void disable_dcam_ldo(void){
        printk("**** disable_dcam_ldo ****\n");
        ANA_REG_SET(ANA_LDO_PD_SET, BIT_8);//power down CAMD0 LDO
        ANA_REG_SET(ANA_LDO_PD_SET, BIT_10);//power down CAMD1 LDO
        ANA_REG_SET(ANA_LDO_PD_SET, BIT_12);//power down CAMA LDO
        return;
}

static void enable_all_clk(void){
        printk("**** enable_all_clk ****\n");
	__raw_bits_and(BIT_4, AHB_CTL0);//sdio
	__raw_bits_and(BIT_5, AHB_CTL0);//usb
	__raw_bits_and(BIT_14, AHB_CTL0);//rotation
        ANA_REG_SET(ANA_ANA_CTL0, ~BIT_10);//power on vibrator
         
}

static void disable_td_subsystem(void){
        printk("**** disable_td_subsystem ****\n");
        __raw_bits_or(BIT_0, GR_CLK_EN);//power down TD subsystem
}

static void disable_pwm_clk(void){
        printk("**** disable_pwm_clk ****\n");
        __raw_bits_and(~BIT_21, GR_CLK_EN);//pwm0
        __raw_bits_and(~BIT_22, GR_CLK_EN);//pwm1
        __raw_bits_and(~BIT_23, GR_CLK_EN);//pwm2
        __raw_bits_and(~BIT_24, GR_CLK_EN);//pwm3
}

static void disable_uart_clk(void){
	printk("***** disable uart clk ****\n");
        //__raw_bits_and(~BIT_4, GR_GEN0);//I2c
        __raw_bits_and(~BIT_14, GR_GEN0);//ccir
        __raw_bits_and(~BIT_15, GR_GEN0);//EPT
        __raw_bits_and(~BIT_20, GR_GEN0);//UART0
        __raw_bits_and(~BIT_21, GR_GEN0);//UART1
        __raw_bits_and(~BIT_22, GR_GEN0);//UART2

        return;
}
static void disable_lcdc_clk(void){
       printk("**** disable_lcdc_clk ****\n");
       __raw_bits_and(~BIT_3, AHB_CTL0);//lcd controller

       return;
}

static void disable_monitors_clk(void){
       printk("**** disable_monitors_clk ****\n");
       __raw_bits_and(~BIT_7, AHB_CTL0);//bus moniter0
       __raw_bits_and(~BIT_11, AHB_CTL0);//bus moniter1

       return;
}

static int current_info_open(struct inode* inode, struct file* file){
        return 0;
}

static int current_info_release(struct inode* inode, struct file* file){
        return 0;
}

static loff_t current_info_llseek(struct file* file, loff_t off, int whence){
        return 0;
}

static ssize_t current_info_read(struct file* file, char* buf, size_t count, loff_t* ppos){
        return 0;
}

static ssize_t current_info_write(struct file* file, const char* buf, size_t size, loff_t* ppos){
        
	char cmd[3];
        
        
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
	mutex_lock(&current_mutex);
        //switch(cmd[0]){
        switch(*cmd){
	     case 'u':
	          disable_usb_clk();
		  break;
	     case 's':
	          disable_sdio_clk();
		  break;
	     case 'r':
	          disable_rotation_clk();
		  break;
	     case 'v':
	          disable_vibrator_clk();
		  break;
	     case 'a':
	          enable_all_clk();
		  break;
	     case 'c':
	          disable_dcam_ldo();
		  break;
	     case 't':
	          disable_td_subsystem();
		  break;
	     case 'p':
	          disable_pwm_clk();
		  break;
	     case 'e':
	          disable_uart_clk();
		  break;
	     case 'd':
	          display_clks();
		  break;
	     case 'l':
	          disable_lcdc_clk();
		  break;
	     case 'm':
	          disable_monitors_clk();
		  break;
	     default :
	          printk("!!!! invalid command !!!!!");
        }		   

	mutex_unlock(&current_mutex);
	return size;
}
#if 0
static struct file_operations current_info_fops = {
	.open = current_info_open,
	.release = current_info_release,
	.llseek = current_info_llseek,
	.read = current_info_read,
	.write = current_info_write,
};
#endif
static struct file_operations current_info_fops = {
        open : current_info_open,
	release : current_info_release,
	llseek : current_info_llseek,
	read : current_info_read,
	write : current_info_write,
};


static struct proc_dir_entry*  
current_proc_create(struct proc_dir_entry* parent, char* name, struct file_operations* fops){
        struct proc_dir_entry *proc;
	proc = create_proc_entry(name, S_IFREG|S_IRUGO|S_IWUSR, parent);
	if(!proc){
	   printk("!!!! create_proc_entry failed !!!!\n");
	   return NULL;
        }

	proc->proc_fops = fops;
	return proc;
}

static void current_proc_mkdir(unsigned long data){
        printk("************************************\n");
	printk("**** create current proc file *******\n");
	current_proc_entry = proc_mkdir("current_info", NULL);
        if(!current_proc_entry){
            printk("!!!!!! proc_mkdir current_info failed !!!!\n");
            return;            
        }
	
        current_proc_create(current_proc_entry, "current_info", &current_info_fops);
	return;


}

static int __devinit omap_wdt_probe(struct platform_device *pdev)
{
	printk("######: omap_wdt_probe()\n");
	printk("######: omap_wdt_probe()\n");
	printk("######: omap_wdt_probe()\n");
	printk("######: omap_wdt_probe()\n");
	printk("######: omap_wdt_probe()\n");

	return 0;

        
}

static void omap_wdt_shutdown(struct platform_device *pdev)
{

}

static int __devexit omap_wdt_remove(struct platform_device *pdev)
{


	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int omap_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{

	printk("######: pm: omap_wdt_suspend()\n");

	return 0;
}

static int omap_wdt_resume(struct platform_device *pdev)
{
	printk("######: pm: omap_wdt_resumes()\n");

	return 0;
}

#else
#define	omap_wdt_suspend	NULL
#define	omap_wdt_resume		NULL
#endif

static struct platform_driver omap_wdt_driver = {
	.probe		= omap_wdt_probe,
	.remove		= __devexit_p(omap_wdt_remove),
	.shutdown	= omap_wdt_shutdown,
	.suspend	= omap_wdt_suspend,
	.resume		= omap_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "omap_wdt",
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND

    struct early_suspend early_suspend;
static void sc8800g2_wdt_suspend (struct early_suspend* es)
{
    printk("###: early: sc8800g2_wdt_suspend()!\n");
    printk("###: early: sc8800g2_wdt_suspend()!\n");
    printk("###: early: sc8800g2_wdt_suspend()!\n");
}

static void sc8800g2_wdt_resume (struct early_suspend* es)
{
    printk("###: early: sc8800g2_wdt_resume()!\n");
    printk("###: early: sc8800g2_wdt_resume()!\n");
    printk("###: early: sc8800g2_wdt_resume()!\n");
}
#endif


struct sprd_pm_suspend sprd_suspend;

static int sc8800g2_sprd_suspend (struct device *pdev, pm_message_t state)
{
   // printk("###: sprd: sc8800g2_wdt_suspend()!\n");
	/*
	if (get_sys_cnt() > 300000)	return -1;
	else return 0;
	*/
	return 0;
}

static int sc8800g2_sprd_resume (struct device *pdev)
{
    //printk("###: sprd: sc8800g2_wdt_resume()!\n");
	return 0;
}



static int __init omap_wdt_init(void)
{
	/*
	struct clk *clk_usb;
	*/
	printk("######: omap_wdt_init()\n");

/*
	mod_timer(&deep_sleep_timer, jiffies + DEEP_SLEEP_INTERVAL);
	mod_timer(&deep_sleep_timer2, jiffies + DEEP_SLEEP_INTERVAL2);
*/

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.suspend = sc8800g2_wdt_suspend;
	early_suspend.resume  = sc8800g2_wdt_resume;
	early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&early_suspend);
#endif

	sprd_suspend.suspend = sc8800g2_sprd_suspend;
	sprd_suspend.resume  = sc8800g2_sprd_resume;
	/* !!!!!!! please set this parameter correctly. */
	sprd_suspend.pdev = NULL;
	sprd_suspend.level   = SPRD_PM_SUSPEND_LEVEL0;
	register_sprd_pm_suspend(&sprd_suspend);
/*
	clk_usb = clk_get(NULL, "clk_usb_ref");
	if (IS_ERR(clk_usb)) {
		printk("##: can't get clk[usb]!\n");
		printk("##: can't get clk[usb]!\n");
		printk("##: can't get clk[usb]!\n");
	}
	else {
		printk("##: get clk[usb] successfully!\n");
		printk("##: get clk[usb] successfully!\n");
		printk("##: get clk[usb] successfully!\n");
	}

	clk_enable(clk_usb);

*/
/***********************
 * added by wong, 02/07/11 
 *
 * ********************/
#if 0
        pid_t current_test_thread;
        current_test_thread = kernel_thread(test_current_thread, NULL, 0);
        if(current_test_thread<0)
	    printk("!!!!! test_current_thread isn't allowed to be created !!!\n");
#endif

	current_proc_mkdir(1);
	return platform_driver_register(&omap_wdt_driver);
}

static void __exit omap_wdt_exit(void)
{
	platform_driver_unregister(&omap_wdt_driver);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);
