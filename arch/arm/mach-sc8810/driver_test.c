

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include<linux/miscdevice.h>
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
#include <linux/memory.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/pm_devices.h>
#include <mach/test.h>
#include <mach/dma.h>
#include <asm/cacheflush.h>


#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/bits.h>
#include <mach/irqs.h>
//#include <asm/dma-mapping.h>//something wrong I don't known


#define DEEP_SLEEP_INTERVAL (6 * HZ)
#define DEEP_SLEEP_INTERVAL2 (8 * HZ)

#define MAX_NODES 8 //WHY 16??
#define LEN_PER_NODE 256 // WHY 256 ??
#define LINKLIST_TOTAL_LEN (LEN_PER_NODE*MAX_NODES)
#define BURST_LEN 32 // should be equal to size of device FIFO
#define BUF_LEN 1024
//defined in <mach/dma.h>
#if 0
#define ON 1
#define OFF 0
typedef enum{
	LINKLIST_DONE,
	BLOCK_DONE,
	TRANSACTION_DONE,
}dma_done_type;
#endif

typedef enum{
	NORMAL_MODE,
	LINKLIST_MODE,
	SOFTLIST_MODE,
}dma_mode_type;

typedef u32 uid;

struct dma_linklist_descriptor{
     volatile u32 cfg0;
     volatile u32 cfg1;
     volatile u32 src_addr;
     volatile u32 dst_addr;
     volatile u32 ll_ptr;
     volatile u32 sdep;
     volatile u32 sbp;
     volatile u32 dbp;
};
	
volatile int dma_done = 0;
struct dma_linklist_descriptor  dma_linklist_nodes[MAX_NODES];
static unsigned char src_buf[BUF_LEN];
static unsigned char dst_buf[BUF_LEN];
static unsigned char src_data[LINKLIST_TOTAL_LEN];
static unsigned char dst_data[LINKLIST_TOTAL_LEN];
static struct proc_dir_entry *dma_test_proc_entry;
static DEFINE_MUTEX(dma_test_mutex);
static void dma_linklist_set_nodes(u32 req_mode, u32 src_dat_width, u32 dst_dat_width);
static void dma_channel_linklist_config(int dma_chn);
//DMA test
#define SPRD_SERIAL1_TXD SPRD_SERIAL1_BASE+0x0000
#define SPRD_SERIAL1_RXD SPRD_SERIAL1_BASE+0x0004
#define SPRD_SERIAL1_STS0 SPRD_SERIAL1_BASE+0x0008
#define SPRD_SERIAL1_STS1 SPRD_SERIAL1_BASE+0x000c
#define SPRD_SERIAL1_IEN SPRD_SERIAL1_BASE+0x0010
#define SPRD_SERIAL1_ICLR SPRD_SERIAL1_BASE+0x0014
#define SPRD_SERIAL1_CTL0 SPRD_SERIAL1_BASE+0x0018
#define SPRD_SERIAL1_CTL1 SPRD_SERIAL1_BASE+0x001c
#define SPRD_SERIAL1_CTL2 SPRD_SERIAL1_BASE+0x0020
#define SPRD_SERIAL1_CLKD SPRD_SERIAL1_BASE+0x0024
#define SPRD_SERIAL1_STS2 SPRD_SERIAL1_BASE+0x002c
#define SPRD_SERIAL1_DSPWAIT SPRD_SERIAL1_BASE+0x0030
//UART1 test(using DMA)


static void deep_sleep_timeout(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer, deep_sleep_timeout, 0, 0);
static void deep_sleep_timeout2(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer2, deep_sleep_timeout2, 0, 0);

static struct proc_dir_entry *current_proc_entry;
static DEFINE_MUTEX(current_mutex);





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
#if 0
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
#endif
}

/***********************
 * added by wong, 12/07/11 
 * disable some clk
 * ********************/
static void display_clks(void){
#if 0
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
#endif
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
#if 0
        printk("**** disable_vibrator_clk ****\n");
        ANA_REG_SET(ANA_ANA_CTL0, BIT_10);//power down vibrator
	printk( "**** after disable vibrator clk, ANA_ANA_CTL0:%08x ****\n", ANA_REG_GET(ANA_ANA_CTL0) );
#endif
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
#if 0	
        printk("**** enable_all_clk ****\n");
	__raw_bits_and(BIT_4, AHB_CTL0);//sdio
	__raw_bits_and(BIT_5, AHB_CTL0);//usb
	__raw_bits_and(BIT_14, AHB_CTL0);//rotation
        ANA_REG_SET(ANA_ANA_CTL0, ~BIT_10);//power on vibrator
#endif         
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

static void dma_channel_config(int dma_chn, u32 chn_req_mode, u32 src_data_width, u32 dst_data_width){
       unsigned int cfg_val = 0;
       
       switch(chn_req_mode){
	       case DMA_REQMODE_NORMAL:
	       case DMA_REQMODE_TRANS:
                    cfg_val = BURST_LEN |
                    //DMA_REQMODE_TRANS | //Transaction Mode
                    //DMA_REQMODE_NORMAL| //Normal Mode
                    chn_req_mode |//Normal Mode
		    src_data_width |
		    dst_data_width;

                    __raw_writel(cfg_val, DMA_CHx_CFG0(dma_chn) );		 
                    __raw_writel(BUF_LEN, DMA_CHx_CFG1(dma_chn) );		 
                    //__raw_bits_or(BIT_28, DMA_CHx_SBP(dma_chn) );		 
                    //__raw_bits_or(BIT_29, DMA_CHx_SBP(dma_chn) );		 
                    //__raw_writel(0x00010001, DMA_CHx_SDEP(dma_chn) );//memory to memory		 
                    __raw_writel(0x00010000, DMA_CHx_SDEP(dma_chn) );//for UART test		 
                    //__raw_writel(__virt_to_bus(src_buf), DMA_CHx_SRC_ADDR(dma_chn) );		 
                    //__raw_writel(__virt_to_bus(dst_buf), DMA_CHx_DEST_ADDR(dma_chn) );//memory to memory
                    __raw_writel(virt_to_phys(src_buf), DMA_CHx_SRC_ADDR(dma_chn) );		 
                    //__raw_writel(virt_to_phys(dst_buf), DMA_CHx_DEST_ADDR(dma_chn) );//memory to memory
	            __raw_writel(0x84000000, DMA_CHx_DEST_ADDR(dma_chn) );//for UART test, How to translate virtual address to physical address 0x84000000???
		    break;

	       case DMA_REQMODE_LIST:
	            dma_linklist_set_nodes(chn_req_mode, src_data_width, dst_data_width); 
	            //__raw_writel(__virt_to_bus(dma_linklist_nodes), DMA_CHx_LLPTR(dma_chn) );//incorrectconcept, must figure it out
	            __raw_writel(virt_to_phys(&dma_linklist_nodes[0]), DMA_CHx_LLPTR(dma_chn) );
		    break;
		    
	       case DMA_REQMODE_INFIINITE://not done yet
	            break;
		    
	       default:
	            printk("!!!! REQUEST MODE?? please check channel request mode agian !!!!\n"); 
       } 
}

	//must modify later, channel 25 for linklist mode test
static void dma_channel_set_uid(int dma_chn, uid dma_uid){
       if(dma_chn > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }
       
       u32 dma_chn_uid_reg = DMA_CHN_UID_BASE + 4*(dma_chn/4);
       int chn_uid_shift = 8*(dma_chn%4);
       
       __raw_writel( (~(0x1f<<chn_uid_shift))&__raw_readl(dma_chn_uid_reg), dma_chn_uid_reg);
       dma_uid = dma_uid << chn_uid_shift; 
       dma_uid |= __raw_readl(dma_chn_uid_reg);
       __raw_writel(dma_uid, dma_chn_uid_reg);
       printk("**** dma_chn_uid_reg:0x%x, 0x%x ****\n", dma_chn_uid_reg, __raw_readl(dma_chn_uid_reg) );

}

static void linklist_test_buffer_init(void){
#if 0      
       int i;
       for(i=0; i<LINKLIST_TOTAL_LEN; i++){
          src_data[i] = 'a';
       }   
       for(i=0; i<LINKLIST_TOTAL_LEN; i++){
          dst_data[i] = 'x';
       }
#endif
//memory to memory 
     
       unsigned int i, j;
       int k;
       unsigned int cnt = LINKLIST_TOTAL_LEN>>7 ;
       k = 0;
       dma_done = 0;
       for(i=0; i<cnt; i++){
	   for(j=0; j<128; j++){
	     src_data[k] = 'a'+i;
	     k++;
	   }
       }

       for(i=0; i<LINKLIST_TOTAL_LEN; i++){
	   dst_data[i] = '0';
       }
#if 0     
       for(i=0; i<LINKLIST_TOTAL_LEN; ){
	  for(j=0; j<BURST_LEN; j++){
	     src_data[i] = '!' + j;
             i++;
	  }
       }
#endif
//       printk("**** cnt:%d, k:%d****\n", cnt, k); 
       //for(k=0; k<LINKLIST_TOTAL_LEN; k++){
       //	    printk("**** dst_data[%d]:%c ****\n", k, dst_data[k] );
       //}
       //mdelay(2000); 
       return;
}

static void test_buffer_init(void){
       printk("=== test_buffer_init, dma_done:%d ====\n", dma_done);
       unsigned char i, j;
       int k;
       //unsigned int cnt = (BUF_LEN>>8) + 1;
       unsigned int cnt = BUF_LEN>>7 ;
       k = 0;
       dma_done = 0;
     
       for(i=0; i<cnt; i++){
	   for(j=0; j<128; j++){
	     src_buf[k] = 'a'+i;
	     k++;
	   }
       }
       printk("**** cnt:%d, k:%d****\n", cnt, k); 
       
       for(k=0; k<BUF_LEN; k++){
	    printk("**** src_buf[%d]:%c ****\n", k, src_buf[k] );
       }
#if 0       
       for(i=0; i<BUF_LEN; i++){
	    dst_buf[i] = '0';
       }
#endif
/*
       if(dma_done){
	  for(i=0; i<BUF_LEN; i++){
	    dst_buf[i] = '0';
          }
       }
*/
}

static void dma_channel_start(int dma_chn, int on_off){
       switch(on_off){
	     case ON:
	         //__raw_bits_and(~(1<<dma_chn), DMA_CHx_DIS);
	         __raw_bits_or(1<<dma_chn, DMA_CHx_EN);
                 break;
	     case OFF:
	         __raw_bits_and(~(1<<dma_chn), DMA_CHx_EN);
	         //__raw_bits_or((1<<dma_chn), DMA_CHx_DIS);
                 break;
	     default:
	         printk("??? dma_channel_start??? what you mean?\n");
       }

       return;
}

static void dma_channel_int_all_clr(int dma_chn){
     __raw_bits_or(1<<dma_chn, DMA_LISTDONE_INT_CLR);
     __raw_bits_or(1<<dma_chn, DMA_BURST_INT_CLR);
     __raw_bits_or(1<<dma_chn, DMA_TRANSF_INT_CLR);
}

static void dma_channel_set_int_type(int dma_chn, dma_done_type mode, int on_off){
       if(dma_chn > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }

       switch(mode){
	  case LINKLIST_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_LISTDONE_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_LISTDONE_INT_EN);
		      break;

		      default:
		         printk(" LLD_MODE, INT_EN ON OR OFF???\n");
		       
	      }	      
	  break;
	  	         
	  case BLOCK_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_BURST_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_BURST_INT_EN);
		      break;

		      default:
		         printk(" BURST_MODE, INT_EN ON OR OFF???\n");
		       
	      }	      
	  break;	         
	  
	  case TRANSACTION_DONE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_TRANSF_INT_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_TRANSF_INT_EN);
		      break;

		      default:
		         printk(" TRANSACTION_MODE, INT_EN ON OR OFF???\n");
		       
	      }	      
	  break;
	  
	  default:
	      printk("??? WHAT MODE YOU SELECT ???\n");	         

       }

}

static void dma_channel_set_mode(int dma_chn, dma_mode_type mode, int on_off){
       if(dma_chn > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }

       switch(mode){
	  case LINKLIST_MODE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_LINKLIST_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_LINKLIST_EN);
		      break;

		      default:
		         printk(" LINKLIST_MODE, INT_EN ON OR OFF???\n"); 
	      }	      
	  break;
	  	         
	  case SOFTLIST_MODE:
	      switch(on_off){
		      case ON:
		         __raw_bits_or(1<<dma_chn, DMA_SOFTLINK_EN);
		      break;

		      case OFF:
		         __raw_bits_and(~(1<<dma_chn), DMA_SOFTLINK_EN);
		      break;

		      default:
		         printk(" SOFTLIST_MODE, INT_EN ON OR OFF???\n");
	      }	      
	  break;	         
	  
	  default:
	      printk("??? WHICH MODE DID YOU SELECT ???\n");	         
       }
}
       
static void dma_channel_set_software_req(int dma_chn, int on_off){
       if(dma_chn > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }

       switch(on_off){
	  case ON:
	    __raw_bits_or(1<<dma_chn, DMA_SOFT_REQ);
            break;
	  case OFF:
	    __raw_bits_and(~(1<<dma_chn), DMA_SOFT_REQ);
	    break;
	  default:
	    printk("??? channel:%d, DMA_SOFT_REQ, ON or OFF \n", dma_chn);
       }

       return;
}

static void dma_channel_regs_clr(int dma_chn){
       if(dma_chn > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn);
	  return;
       }
     
       __raw_writel(0x0, DMA_CHx_CFG0(dma_chn));  
       __raw_writel(0x0, DMA_CHx_CFG1(dma_chn));  
       __raw_writel(0x0, DMA_CHx_SRC_ADDR(dma_chn));  
       __raw_writel(0x0, DMA_CHx_DEST_ADDR(dma_chn));  
       __raw_writel(0x0, DMA_CHx_LLPTR(dma_chn));  
       __raw_writel(0x0, DMA_CHx_SDEP(dma_chn));  
       __raw_writel(0x0, DMA_CHx_SBP(dma_chn));  
       __raw_writel(0x0, DMA_CHx_DBP(dma_chn));  
       
}
static void dma_channel_init(int dma_chn_index){
       if(dma_chn_index > DMA_CH_NUM){
          printk("!!!! Invalid DMA Channel: %d !!!!\n", dma_chn_index);
	  return;
       }

       dma_channel_start(dma_chn_index, OFF);
       dma_channel_int_all_clr(dma_chn_index);

       dma_channel_set_int_type(dma_chn_index, LINKLIST_DONE, OFF);
       dma_channel_set_int_type(dma_chn_index, BLOCK_DONE, OFF);
       dma_channel_set_int_type(dma_chn_index, TRANSACTION_DONE, OFF);
       
       dma_channel_set_mode(dma_chn_index, LINKLIST_MODE, OFF);
       dma_channel_set_mode(dma_chn_index, SOFTLIST_MODE, OFF);
       
       dma_channel_set_software_req(dma_chn_index, OFF);
       
       dma_channel_regs_clr(dma_chn_index); 
 
       return;
}

static void dma_test_irq(int dma_chn_index, void* data){
	printk("==== dma_test_irq ====\n");
        dma_done = 1;
}


static void dma_print_dest(void){
      int i = 0;
      for(; i<BUF_LEN; i++){
	  printk("**** dst_buf[%d]=%c ****\n", i, dst_buf[i]);
       } 	
      
      printk("**** dma_print_dest DONE ****\n");
}

static void dma_is_correct(u32 req_mode){
       int i = 0;
#if 0       
       while(dma_done != 1){
           printk("!!! dma not done !!!\n");
	   return;
       }
#endif
/*
       for(; i<BUF_LEN; i++){
            printk("!!!! src_buf[%d]=%c !!!\n", i, src_buf[i]);
       }
*/     
       switch(req_mode){
	  case DMA_REQMODE_NORMAL:      	
	  case DMA_REQMODE_TRANS:      	
               for(; i<BUF_LEN; i++){
                   //if(dst_buf[i] != '1')
                   printk("!!!! dst_buf[%d]=%c !!!\n", i, dst_buf[i]);
               }
	       break;
	  case DMA_REQMODE_LIST:
               for(; i<LINKLIST_TOTAL_LEN+20; i++){
                   //if(dst_buf[i] != '1')
                   printk("!!!! dst_data[%d]=%c !!!\n", i, dst_data[i]);
               }
	       break;
	  case DMA_REQMODE_INFIINITE:
	       break;
	  default:
	       printk("???? request mode?, check again please\n");
       }

       return;
}

static void dma_dump_channel_regs(int dma_chn){
      printk("==== DMA_CH%d_CFG0:%x ===\n", dma_chn, __raw_readl(DMA_CHx_CFG0(dma_chn)) ); 
      printk("==== DMA_CH%d_CFG1:%x ===\n", dma_chn, __raw_readl(DMA_CHx_CFG1(dma_chn)) ); 
      printk("==== DMA_CH%d_SRC_ADDR:%x ===\n", dma_chn, __raw_readl(DMA_CHx_SRC_ADDR(dma_chn)) ); 
      printk("==== DMA_CH%d_DEST_ADDR:%x ===\n", dma_chn, __raw_readl(DMA_CHx_DEST_ADDR(dma_chn)) ); 
      printk("==== DMA_CH%d_LLPTR:%x ===\n", dma_chn, __raw_readl(DMA_CHx_LLPTR(dma_chn)) ); 
      printk("==== DMA_CH%d_SDEP:%x ===\n", dma_chn, __raw_readl(DMA_CHx_SDEP(dma_chn)) ); 
      printk("==== DMA_CH%d_SBP:%x ===\n", dma_chn, __raw_readl(DMA_CHx_SBP(dma_chn)) ); 
      printk("==== DMA_CH%d_DBP:%x ===\n", dma_chn, __raw_readl(DMA_CHx_DBP(dma_chn)) ); 

}

static void dma_dump_regs(void){
      printk("==== DMA_CFG:%x ===\n", __raw_readl(DMA_CFG) );
      printk("==== DMA_CHx_EN_STATUS:%x ===\n", __raw_readl(DMA_CFG+0x4) );
      printk("==== DMA_CHN_UID0:%x ===\n", __raw_readl(DMA_CHN_UID0) );
      printk("==== DMA_CHN_UID1:%x ===\n", __raw_readl(DMA_CHN_UID1) );
      printk("==== DMA_CHN_UID2:%x ===\n", __raw_readl(DMA_CHN_UID2) );
      printk("==== DMA_CHN_UID3:%x ===\n", __raw_readl(DMA_CHN_UID3) );
      printk("==== DMA_CHN_UID4:%x ===\n", __raw_readl(DMA_CHN_UID4) );
      printk("==== DMA_CHN_UID5:%x ===\n", __raw_readl(DMA_CHN_UID5) );
      printk("==== DMA_CHN_UID6:%x ===\n", __raw_readl(DMA_CHN_UID6) );
      printk("==== DMA_CHN_UID7:%x ===\n", __raw_readl(DMA_CHN_UID7) );
      
      printk("==== DMA_LINKLIST_EN:%x ===\n", __raw_readl(DMA_LINKLIST_EN) );
      printk("==== DMA_SOFTLINK_EN:%x ===\n", __raw_readl(DMA_SOFTLINK_EN) );
      
      printk("==== DMA_INT_STS:%x ===\n", __raw_readl(DMA_INT_STS) );
      printk("==== DMA_INT_RAW:%x ===\n", __raw_readl(DMA_INT_RAW) );
      
      printk("==== DMA_LISTDONE_INT_EN:%x ===\n", __raw_readl(DMA_LISTDONE_INT_EN) );
      printk("==== DMA_BURST_INT_EN:%x ===\n", __raw_readl(DMA_BURST_INT_EN) );
      printk("==== DMA_TRANSF_INT_EN:%x ===\n", __raw_readl(DMA_TRANSF_INT_EN) );
     
      printk("==== DMA_LISTDONE_INT_STS:%x ===\n", __raw_readl(DMA_LISTDONE_INT_STS) );
      printk("==== DMA_BURST_INT_STS:%x ===\n", __raw_readl(DMA_BURST_INT_STS) );
      printk("==== DMA_TRANSF_INT_STS:%x ===\n", __raw_readl(DMA_TRANSF_INT_STS) );
      
      printk("==== DMA_LISTDONE_INT_RAW:%x ===\n", __raw_readl(DMA_LISTDONE_INT_RAW) );
      printk("==== DMA_BURST_INT_RAW:%x ===\n", __raw_readl(DMA_BURST_INT_RAW) );
      printk("==== DMA_TRANSF_INT_RAW:%x ===\n", __raw_readl(DMA_TRANSF_INT_RAW) );
      
      printk("==== DMA_TRANS_STS:%x ===\n", __raw_readl(DMA_TRANS_STS) );
      printk("==== DMA_REQ_PEND:%x ===\n", __raw_readl(DMA_REQ_PEND) );
      
      
}

//softlist work mode, not done yet
static void dma_softlist_work_mode(void){

}


static void dump_uart2_regs(void){
      printk("==== SERIAL1_STS0:0x%x ===\n", __raw_readl(SPRD_SERIAL1_STS0) );
      printk("==== SERIAL1_STS1:0x%x ===\n", __raw_readl(SPRD_SERIAL1_STS1) );
      printk("==== SERIAL1_IEN:0x%x ===\n", __raw_readl(SPRD_SERIAL1_IEN) );
      printk("==== SERIAL1_ICLR:0x%x ===\n", __raw_readl(SPRD_SERIAL1_ICLR) );
      printk("==== SERIAL1_CTL0:0x%x ===\n", __raw_readl(SPRD_SERIAL1_CTL0) );
      printk("==== SERIAL1_CTL1:0x%x ===\n", __raw_readl(SPRD_SERIAL1_CTL1) );
      printk("==== SERIAL1_CTL2:0x%x ===\n", __raw_readl(SPRD_SERIAL1_CTL2) );
      printk("==== SERIAL1_CLKD:0x%x ===\n", __raw_readl(SPRD_SERIAL1_CLKD) );
      printk("==== SERIAL1_STS2:0x%x ===\n", __raw_readl(SPRD_SERIAL1_STS2) );
      printk("==== SERIAL1_DSPWAIT:0x%x ===\n", __raw_readl(SPRD_SERIAL1_DSPWAIT) );
}

static irqreturn_t serial1_irq(int irq, void *dev_id){
      unsigned int status = __raw_readl(SPRD_SERIAL1_STS0);
      
      //printk("**** UART1 interrupt status:%x ****\n", status);
     
      __raw_bits_and(~BIT_1, SPRD_SERIAL1_IEN);//enable tX fifo empty interrupt
      __raw_writel(0xffff, SPRD_SERIAL1_ICLR);//clear all interrupt
      
      return IRQ_HANDLED;      

}

static void dma_test_init_uart2(int uart){//not done
       int ret = 0;
       ret = request_irq(IRQ_SER1_INT, serial1_irq, 0, "serial1_irq", NULL);
       if(ret != 0){
          printk("!!!! requet irq for UART1 failed:%d !!!!\n", ret);
	  return;
       }
       __raw_writel(0xffff, SPRD_SERIAL1_ICLR);//clear all interrupt
       __raw_bits_or(BIT_15, SPRD_SERIAL1_CTL1);//enable DMA access
       //__raw_bits_or(BIT_8, SPRD_SERIAL1_CTL1);//enable TX hardware flow control
       __raw_bits_and(~BIT_8, SPRD_SERIAL1_CTL1);//disable TX hardware flow control
       //__raw_bits_or(BIT_1, SPRD_SERIAL1_IEN);//enable TX fifo empty interrupt
       dump_uart2_regs( );
       printk("==== UART1 Initialize Done ====\n");
              
}

static void dma_default_work_mode(void){
       int chn_index = 25;
       int uart_num = 2;
       int cnt = 0;
       dma_test_init_uart2(uart_num);//for UART1 test
       test_buffer_init();
       while(1){
          mdelay(5000);
	  dma_channel_init(chn_index);
	  //dma_channel_set_uid(chn_index, DMA_SOFT0);
	  dma_channel_set_uid(chn_index, DMA_UART1_TX);// for UART1 test
          dma_channel_set_int_type(chn_index, TRANSACTION_DONE, ON);
	  dma_channel_config(chn_index, DMA_REQMODE_NORMAL, DMA_SDATA_WIDTH8, DMA_DDATA_WIDTH8);
	  sprd_free_dma(chn_index);
	  sprd_request_dma(chn_index, dma_test_irq, NULL);

	  dma_channel_start(chn_index, ON);
          //dma_channel_set_software_req(chn_index, ON);//memory to memory test
          __raw_bits_or(BIT_1, SPRD_SERIAL1_IEN);//enable TX fifo empty interrupt
	  
	  //dma_dump_regs();
	  //dma_dump_channel_regs(chn_index);
	  //dump_uart2_regs();

          cnt++;
	  if(cnt==100)
            return; 
	  //return;

	  //dma_is_correct(); 
       }

}

static void dma_print_each_node(void){
       int i;

       for(i=0; i<MAX_NODES; i++){
             printk("**** dma_linklist_nodes[%d].cfg0:%x ****\n", i, dma_linklist_nodes[i].cfg0);
             printk("**** dma_linklist_nodes[%d].cfg1:%x ****\n", i, dma_linklist_nodes[i].cfg1);
             printk("**** dma_linklist_nodes[%d].src_addr:%x ****\n", i, dma_linklist_nodes[i].src_addr);
             printk("**** dma_linklist_nodes[%d].dst_addr:%x ****\n", i, dma_linklist_nodes[i].dst_addr);
             printk("**** dma_linklist_nodes[%d].ll_ptr:%x ****\n", i, dma_linklist_nodes[i].ll_ptr);
             printk("**** dma_linklist_nodes[%d].sdep:%x ****\n", i, dma_linklist_nodes[i].sdep);
             printk("**** dma_linklist_nodes[%d].sbp:%x ****\n", i, dma_linklist_nodes[i].sbp);
             printk("**** dma_linklist_nodes[%d].dbp:%x ****\n", i, dma_linklist_nodes[i].dbp);
          
       }
       printk("**** src_data:%p ****\n", virt_to_phys(src_data) );
       printk("**** dst_data:%p ****\n", virt_to_phys(dst_data) );
       printk("**** dma_linklist_nodes:%p ****\n", virt_to_phys(dma_linklist_nodes) );
}

static void dma_linklist_set_nodes(u32 req_mode, u32 src_dat_width, u32 dst_dat_width){
       int i; 
       u32 cfg0;
       u32 sdep;

       //cfg0 = BURST_LEN | DMA_REQMODE_NORMAL | src_dat_width | dst_dat_width;//Memory To Devices 
       cfg0 = BURST_LEN | DMA_REQMODE_LIST | src_dat_width | dst_dat_width; //Memory To Memory
       switch(src_dat_width){
	  case DMA_SDATA_WIDTH8:
               sdep = 0x00010001;
	       break;
	  case DMA_SDATA_WIDTH16:
               sdep = 0x00020002;
	       break;
	  case DMA_SDATA_WIDTH32:
               sdep = 0x00040004;
	       break;
       }
       for(i=0; i<MAX_NODES; i++){
          dma_linklist_nodes[i].cfg0 = cfg0;
	  dma_linklist_nodes[i].cfg1 = LEN_PER_NODE;
          //dma_linklist_nodes[i].src_addr = virt_to_dma(NULL, (src_data + i*LEN_PER_NODE) );//incorrect
          //dma_linklist_nodes[i].dst_addr = virt_to_dma(NULL, (dst_data + i*LEN_PER_NODE) );//incorrect
          dma_linklist_nodes[i].src_addr = virt_to_phys(src_data + i*LEN_PER_NODE);
          dma_linklist_nodes[i].dst_addr = virt_to_phys(dst_data + i*LEN_PER_NODE);//memory to memory
          //dma_linklist_nodes[i].dst_addr = (u32)0x84000000; //for UART1 test
          dma_linklist_nodes[i].ll_ptr = virt_to_phys(&dma_linklist_nodes[i+1]);
          dma_linklist_nodes[i].sdep = sdep;//memory to memory
          //dma_linklist_nodes[i].sdep = (u32)0x00010000;// memory to UART1, TX
          dma_linklist_nodes[i].sbp = (u32)0x0;
          dma_linklist_nodes[i].dbp = (u32)0x0;
	  // find out how to set
       }
       //dma_linklist_nodes[MAX_NODES-2].cfg0 |= DMA_LLEND;
       //dma_linklist_nodes[MAX_NODES/2 - 1].cfg0 |= DMA_LLEND;
       dma_linklist_nodes[MAX_NODES-1].cfg0 |= DMA_LLEND;
       flush_cache_all();//flush cache

       dma_print_each_node();

       return;
}

static void dma_linklist_nodes_init(void){
       int i;
       char *p_node = (char*)(dma_linklist_nodes);

       for(i=0; i<sizeof(dma_linklist_nodes); i++){
          p_node[i] = 0x00;
       }       
}

static void dma_channel_linklist_config(int dma_chn){
       dma_linklist_nodes_init( );
       dma_channel_config(dma_chn, DMA_REQMODE_LIST, DMA_SDATA_WIDTH8, DMA_DDATA_WIDTH8);
}

static void dma_linklist_work_mode(void){
       int chn_index = 25;
       int uart_num = 1;
       int cnt = 0;

       linklist_test_buffer_init( );
       dma_test_init_uart2(uart_num);//for UART1 test, must modify later
       while(1){
         mdelay(5000);
         dma_channel_init(chn_index);
	 dma_channel_set_uid(chn_index, DMA_SOFT0);
	 //dma_channel_set_uid(chn_index, DMA_UART1_TX);// for UART1 test
         dma_channel_set_mode(chn_index, LINKLIST_MODE, ON);
         dma_channel_set_int_type(chn_index, LINKLIST_DONE, ON);
         //dma_channel_set_int_type(chn_index, TRANSACTION_DONE, ON);
         //dma_channel_set_int_type(chn_index, BLOCK_DONE, ON);
         dma_channel_linklist_config(chn_index);
	 sprd_free_dma(chn_index);
	 sprd_request_dma(chn_index, dma_test_irq, NULL);
	 
         
        // __raw_bits_or(BIT_1, SPRD_SERIAL1_IEN);//enable TX fifo empty interrupt
	 dma_channel_start(chn_index, ON);
	 dma_channel_set_software_req(chn_index, ON);//memory to memory
         
	 dma_is_correct(DMA_REQMODE_LIST); 
	 //dma_dump_regs();
	 //dma_dump_channel_regs(chn_index);
          
	 //cnt++; 
	 //if(cnt==100){
         //  return;
	 //}
           

	return;
       }
}

/*********************  dma test using new interface *********************************/
/**
 * two cases passed in this test:
 * 1. normal mode: (1)memory to memory, (2)memory to devices, as UART1 for example 
 * 2. linklist mode: (1)memory to memory, (2)memory to devices, as UART1 for example 
 *
 * DMA users should configurate dma channel parameter by thenselves
 **/

/**
 * 1. normal mode
 *   normal mode is the default work mode after boot up
 * 
 **/
struct sprd_dma_channel_desc dma_cfg;
void dma_test_normal_mode_new_set_config(struct sprd_dma_channel_desc *cfg){
     cfg->cfg_swt_mode_sel = DMA_UN_SWT_MODE;//switch mode
     cfg->cfg_src_data_width = DMA_SDATA_WIDTH8;
     cfg->cfg_dst_data_width = DMA_DDATA_WIDTH8;
     //cfg->cfg_req_mode_sel = DMA_REQMODE_NORMAL;//memory to devices
     cfg->cfg_req_mode_sel = DMA_REQMODE_TRANS;//memory to memory
     cfg->cfg_src_wrap_en = 0;
     cfg->cfg_dst_wrap_en = 0;
     cfg->cfg_blk_len = BURST_LEN;//block length
     cfg->total_len = BUF_LEN;
     cfg->src_addr = virt_to_phys(src_buf);
     cfg->dst_addr = virt_to_phys(dst_buf);//memory to memory
     //cfg->dst_addr = 0x84000000;// UART1 TX FIFO
     cfg->llist_ptr = 0;
     //cfg->llist_ptr = virt_to_phys(&dma_linklist_nodes[0]);//linklist mode
     cfg->src_elem_postm = 1;
     cfg->dst_elem_postm = 1;
     //cfg->dst_elem_postm = 0;//should be 0 when memory to devices
     cfg->src_burst_mode = 0;//0 as default
     cfg->src_blk_postm = 0;//0 as default
     cfg->dst_burst_mode = 0;//0 as default
     cfg->dst_blk_postm = 0;//0 as default
}

void dma_test_normal_mode_new(void){
     u32 chn;
     test_buffer_init( );
     //dma_test_init_uart2(1);//memory to uart1
     dma_test_normal_mode_new_set_config(&dma_cfg);
     //chn = sprd_dma_request(DMA_UART1_TX, dma_test_irq, NULL);
     chn = sprd_dma_request(DMA_SOFT0, dma_test_irq, NULL);
     if(chn >= 0){
         sprd_dma_channel_config(chn, DMA_NORMAL, &dma_cfg);
	 //sprd_dma_set_irq_type(chn, BLOCK_DONE, ON);
	 sprd_dma_set_irq_type(chn, TRANSACTION_DONE, ON);
	 sprd_dma_set_chn_pri(chn, 3);
         //__raw_bits_or(BIT_1, SPRD_SERIAL1_IEN);//enable TX fifo empty interrupt
         sprd_dma_channel_start(chn);
	 mdelay(5000);
     }else{
         printk("!!!!! dma channel:%d is invalid !!!!!\n");
     }

     //dma_dump_channel_regs(chn); //for debug
     sprd_dma_free(chn);
     //dma_is_correct(DMA_REQMODE_NORMAL);//for debug
     //sprd_dma_check_channel( );//for debug
}


/**
 * 2. linklist work mode
 *   dma request mode must be set DMA_REQMODE_NORMAL(normal request mode) in condition "memory to devices"
 *   dma request mode must be set DMA_REQMODE_LIST(normal request mode) in condition "memory to memory"
 **/

// we have configurated dma channel parameters in linklist nodes, so just set llist_ptr here
//
void dma_test_linklist_mode_new_set_config(struct sprd_dma_channel_desc *cfg){
     cfg->cfg_swt_mode_sel = 0;//switch mode
     cfg->cfg_src_data_width = 0;
     cfg->cfg_dst_data_width = 0;
     cfg->cfg_req_mode_sel = 0;
     cfg->cfg_src_wrap_en = 0;
     cfg->cfg_dst_wrap_en = 0;
     cfg->cfg_blk_len = 0;//block length
     cfg->total_len = 0;
     cfg->src_addr = 0;
     cfg->dst_addr = 0;
     cfg->llist_ptr = virt_to_phys(&dma_linklist_nodes[0]);//linklist mode
     cfg->src_elem_postm = 0;
     cfg->dst_elem_postm = 0;
     cfg->src_burst_mode = 0;
     cfg->src_blk_postm = 0;
     cfg->dst_burst_mode = 0;
     cfg->dst_blk_postm = 0;
}
void dma_test_linklist_mode_new(void){
     u32 chn;
     linklist_test_buffer_init( );
     //dma_test_init_uart2(1);
     /****  set the linklist nodes  ****/
     //dma_linklist_set_nodes(DMA_REQMODE_NORMAL, DMA_SDATA_WIDTH8, DMA_DDATA_WIDTH8);//Memory To Devices
     dma_linklist_set_nodes(DMA_REQMODE_LIST, DMA_SDATA_WIDTH8, DMA_DDATA_WIDTH8);//Memory to Memory
     dma_test_linklist_mode_new_set_config(&dma_cfg);//set linklist nodes
     //chn = sprd_dma_request(DMA_UART1_TX, dma_test_irq, NULL);
     chn = sprd_dma_request(DMA_UID_SOFTWARE, dma_test_irq, NULL);
     if(chn >= 0){
	 printk("==== channel number:%d ====\n", chn);//wong
         sprd_dma_channel_config(chn, DMA_LINKLIST, &dma_cfg);
	 //sprd_dma_set_irq_type(chn, BLOCK_DONE, ON);
	 //sprd_dma_set_irq_type(chn, TRANSACTION_DONE, ON);
	 sprd_dma_set_irq_type(chn, BLOCK_DONE, OFF);
	 sprd_dma_set_irq_type(chn, TRANSACTION_DONE, OFF);
	 sprd_dma_set_irq_type(chn, LINKLIST_DONE, ON);
         //__raw_bits_or(BIT_1, SPRD_SERIAL1_IEN);//enable TX fifo empty interrupt
         sprd_dma_channel_start(chn);
     }else{
         printk("!!!!! dma channel:%d is invalid !!!!!\n");
     }
     mdelay(500);
     //sprd_dma_free(chn);
     dma_dump_channel_regs(chn);// for debug
     //dma_is_correct(DMA_REQMODE_LIST);//for debug
}

/*********************  dma test using new interface *********************************/



static int dma_test_open(struct inode* inode, struct file* file){
        return 0;
}
static int dma_test_release(struct inode* inode, struct file* file){
        return 0;
}
static loff_t dma_test_llseek(struct file* file, loff_t off, int whence){
        return 0;
}
static ssize_t dma_test_read(struct file* file, char* buf, size_t count, loff_t* ppos){
        return 0;
}
static ssize_t dma_test_write(struct file* file, const char* buf, size_t size, loff_t* ppos){
        char cmd[3];
	
	if(*ppos){
	     printk("============= *ppos=%d ================\n", *ppos);
	     return -EINVAL;
        }
	if(copy_from_user(cmd, buf, size)){
	     printk("=========== copy_from_user failed ==========\n");
             return -EFAULT;
        }

	mutex_lock(&dma_test_mutex);
        switch(*cmd){
	     case 't':
	       //dma_default_work_mode();//ok case
	       dma_test_normal_mode_new();
	       break;
	     case 'l':
	       dma_linklist_work_mode();
	       //dma_test_linklist_mode_new();
	       break;
	     case 's':
	       dma_test_linklist_mode_new();
	       //dma_softlist_work_mode();//not done yet
	       break;
	     case 'd':
	       dma_dump_regs();
               //sprd_dma_check_channel( );//for debug
	       break;
	     case 'p':
	       dma_print_dest();
	       break;
	     default:
	       printk("!!!! invalid argument, Not Surpported Mode !!!!\n");
        }
	mutex_unlock(&dma_test_mutex);
	
	return size;
}

static struct file_operations dma_test_fops = {
        open : dma_test_open,
	release : dma_test_release,
	llseek : dma_test_llseek,
	read : dma_test_read,
	write : dma_test_write,
};

static struct proc_dir_entry*  
dma_test_proc_create(struct proc_dir_entry* parent, char* name, struct file_operations* fops){
        struct proc_dir_entry *proc;
	proc = create_proc_entry(name, S_IFREG|S_IRUGO|S_IWUSR, parent);
	if(!proc){
	   printk("!!!! dma_test_proc_entry failed !!!!\n");
	   return NULL;
        }

	proc->proc_fops = fops;
	return proc;
}

static void dma_test_proc_mkdir(unsigned long data){
        printk("************************************\n");
	printk("**** create dma_test proc file *******\n");
	dma_test_proc_entry = proc_mkdir("dma_test", NULL);
        if(!dma_test_proc_entry){
            printk("!!!!!! proc_mkdir dma_test failed !!!!\n");
            return;            
        }
	
        dma_test_proc_create(dma_test_proc_entry, "dma_test", &dma_test_fops);
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
	dma_test_proc_mkdir(1);
	return platform_driver_register(&omap_wdt_driver);
}

static void __exit omap_wdt_exit(void)
{
	platform_driver_unregister(&omap_wdt_driver);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);
