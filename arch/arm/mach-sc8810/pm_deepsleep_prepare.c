/*
  *     Deep sleep routine for SC8800G.
  *
  *     Wangliwei.   levee.wang@spreadtrum.com		
  *
  */


/*
check points for botton current.

1. pin map.
2. wifi <--> UNIFIxxxx.
3. bt(gpio_42, gpio_90),  BCxxxx.
4. gps(allen) , /system/lib/libgsd4t.so, on/off <--> uart2.rxd.
5. fm(aijun, 2011-05-24),  -20mA(2011-05-31).
6. ldo control.
7. atv(remove, hw).
8. lcdc(ok).
9. g-sensor(ok).
10. m-sensor(ok).
11. ctp -- mtp(ok).
12. proximity(ok).
 */

/*
1. wifi, bt.
2. fm.
3. dsp, enable deep sleep in nvitem.
4. gps.
5. m-sensor, g-sensor; for now, disable them.
6. audio PA. GPIO96, low.
*/

/*
1. camera, TP, WIFI, BT, FM, audio PA,  LCD are totally ok.
2. GPS, G-sensor, M-sensor need to be checked, totally 1.4mA.
*/
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
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/tlbflush.h>

#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/regs_gpio.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_emc.h>
#include <mach/regs_int.h>
#include <mach/dma.h>


#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>
#include <mach/spinlock_hw_vlx.h>
#include <linux/suspend.h>
#include <mach/pm_devices.h>
#include <mach/test.h>

typedef unsigned int uint32;

#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
#endif
extern void printascii_phy(char *);

#define INT_REG(off) (SPRD_INTCV_BASE + (off))

#define INT_IRQ_STS            INT_REG(0x0000)
#define INT_IRQ_RAW            INT_REG(0x0004)
#define INT_IRQ_ENB            INT_REG(0x0008)
#define INT_IRQ_DIS            INT_REG(0x000c)
#define INT_FIQ_STS            INT_REG(0x0020)

extern long has_wake_lock_info(int type);
extern long has_wake_lock_for_suspend(int type);


#define IRAM_BASE_PHY   0xFFFF0000
#define IRAM_START_PHY 	0xFFFF4000
#define IRAM_SIZE 0x4000

#define SLEEP_CODE_SIZE 4096
#define SC8800G2_DEFAULT_PLL_N (0xd5)
static u32 pll_n = SC8800G2_DEFAULT_PLL_N;
/*********************************/
/*********************************/
/* for debug only. */
u16 val_short;
/* for saving global registers. */
u32 ahb_ctl0, gen0, gen_clk_en, busclk_alm, irq_flags;
u32 val;
u32 sleep_mode = 0;
u32 sleep_counter = 0;
u32 uptime = 0;
u32 sleep_time = 0;
u32 wake_time = 0;
u32 sleep_time_inidle = 0;
u32 idle_time = 0;
int thread_loops = 0;
int idle_loops = 0;
u32 timer_int_counter = 0;
u32	tick_sched_timer_counter = 0;
u32 interrupt_counter = 0;
u32 schedu_counter = 0;
u32 gr_stc_state = 0;
u32 ahb_sts = 0;
u32 gr_clk_dly = 0;
u32 dma_sts = 0;
u32 irq_sts = 0;
u32 fiq_sts = 0;
u32 irq_enable = 0;
struct timespec now_ts_pm;

/* for /proc/xxx interfaces. */
static struct proc_dir_entry* sprd_proc_entry;
static DEFINE_MUTEX(sprd_proc_info_mutex);



/* interrupt statistic. */
u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM]= {0, };
u32 sprd_irqs[SPRD_IRQ_NUM] = {0, };
void irq_statistic_reset(void)
{
	int i;
	for(i = 0; i < SPRD_HARD_INTERRUPT_NUM; i++) sprd_hard_irq[i] = 0;
	for(i = 0; i < SPRD_IRQ_NUM; i++) sprd_irqs[i] = 0;
}

void parse_sprd_hard_irq(unsigned long val)
{
	int i;
	for (i = 0; i < SPRD_HARD_INTERRUPT_NUM; i++) {
		if (test_and_clear_bit(i, &val)) sprd_hard_irq[i]++;
	}
}


int sprd_irq_for_arm(u32 val)
{
	return (val & (~IRQ_FOR_DSP_MASK));
}

void inc_sprd_irq(int irq)
{
	if (irq >= SPRD_IRQ_NUM) {
		printk("## bad irq number %d.\n", irq);
	}
	else {
		sprd_irqs[irq]++;
	}
}

void show_sprd_irq_info(void)
{
	int i;
	printk("#### irq info. ######\n");

	for(i = 0; i < SPRD_HARD_INTERRUPT_NUM; i++)  {
		if (sprd_hard_irq[i] > 0) 
			printk("##: sprd_hard_irq[%d] = %d.\n", 
				i, sprd_hard_irq[i]);
	}
	for(i = 0; i < SPRD_IRQ_NUM; i++)  {
		if (sprd_irqs[i] > 0)
			printk("##: sprd_irqs[%d] = %d.\n", 
				i, sprd_irqs[i]);
	}
}


/* thead statistic. */
u32 sprd_threads[SPRD_THREADS_ARRAY_SIZE]= {0, };
void threads_statistic_reset(void)
{
	int i;
	for(i = 0; i < SPRD_THREADS_ARRAY_SIZE; i++) sprd_threads[i] = 0;
}


void inc_sprd_thread_counts(int thread)
{
	if (thread >= SPRD_THREADS_ARRAY_SIZE) {
		/*
		printk("## Thread index [%d] is larger than array size [%d]!\n", 
			thread, SPRD_THREADS_ARRAY_SIZE);
		*/
	}
	else {
		sprd_threads[thread]++;
	}
}

void show_sprd_thread_info(void)
{
	int i;
	printk("#### thread info. ######\n");

	for(i = 0; i < SPRD_THREADS_ARRAY_SIZE; i++)  {
		if (sprd_threads[i] > 0) 
			printk("##: sprd_threads[%d] = %d.\n", 
				i, sprd_threads[i]);
	}
}


#define SPRD_PM_MESSAGE 1

#define MESSAGE_MAX 2048

struct pm_message_sc8800g2 {
    u32 time_stamp;
    char *msg;
    u32 val0;
    u32 val1;
    u32 val2;
    u64 val64;
};

static struct pm_message_sc8800g2 messages[MESSAGE_MAX];
static int pm_message_pos = 0;
static volatile int pm_message_stop = 0;
static void init_pm_message(void)
{
#ifdef SPRD_PM_MESSAGE
    int i;
    for (i = 0; i < MESSAGE_MAX; i++) {
            messages[i].time_stamp = 0;
            messages[i].msg = NULL;
            messages[i].val0 = 0;
            messages[i].val1 = 0;
            messages[i].val2 = 0;
            messages[i].val64 = 0;
    }
#endif
}
void add_pm_message_legacy(char *msg, uint32 val)
{
#ifdef SPRD_PM_MESSAGE
    if (pm_message_pos >= MESSAGE_MAX) pm_message_pos = 0;
    messages[pm_message_pos].msg = msg;
    messages[pm_message_pos].val0 = val;
    pm_message_pos++;
#endif

}
void print_pm_message_legacy(void)
{
#ifdef SPRD_PM_MESSAGE

    int i;
    for (i = 0; i < MESSAGE_MAX; i++) {
            if ( messages[i].msg != NULL)  {
                printk("######: [%d] %s = %08x. ######\n", 
                    i, messages[i].msg, messages[i].val0);
            }
    }
#endif

}


void add_pm_message(u32 when, char *msg, u32 val0, 
                                              u32 val1, u32 val2)
{
#ifdef SPRD_PM_MESSAGE

    if (pm_message_stop || (sleep_counter <= 0)) return;
    if (pm_message_pos >= MESSAGE_MAX) pm_message_pos = 0;
    messages[pm_message_pos].time_stamp = when;
    messages[pm_message_pos].msg = msg;
    messages[pm_message_pos].val0 = val0;
    messages[pm_message_pos].val1 = val1;
    messages[pm_message_pos].val2 = val2;
    messages[pm_message_pos].val64 = 0;
    pm_message_pos++;
/*
	printk("**********: add_pm_message: pm_message_pos = %d, msg = %s\n", 
		pm_message_pos, msg);
*/
#endif

}

void add_pm_message_val64(u32 when, char *msg, u32 val0, 
                                              u32 val1, u32 val2, u64 val64)
{
#ifdef SPRD_PM_MESSAGE

    if (pm_message_stop || (sleep_counter <= 0)) return;
    if (pm_message_pos >= MESSAGE_MAX) pm_message_pos = 0;
    messages[pm_message_pos].time_stamp = when;
    messages[pm_message_pos].msg = msg;
    messages[pm_message_pos].val0 = val0;
    messages[pm_message_pos].val1 = val1;
    messages[pm_message_pos].val2 = val2;
    messages[pm_message_pos].val64 = val64;

    pm_message_pos++;
#endif

}

void print_pm_message(void)
{
#ifdef SPRD_PM_MESSAGE

    int i;
    for (i = 0; i < MESSAGE_MAX; i++) {
            if ( messages[i].msg != NULL)  {
                printk("##: [%d] [%d] %s: %u, %u, %u, %Lu. \n", 
                    i, messages[i].time_stamp, messages[i].msg, messages[i].val0, 
                      messages[i].val1, messages[i].val2, messages[i].val64);
            }
    }
#endif

}

void stop_pm_message(void)
{
#ifdef SPRD_PM_MESSAGE
	pm_message_stop= 1;
#endif

}

void start_pm_message(void)
{
#ifdef SPRD_PM_MESSAGE
	pm_message_stop= 0;
#endif

}

#define hw_raw_irqs_disabled_flags(flags)	\
({					\
	(int)((flags) & PSR_I_BIT);	\
})



#define hw_irqs_disabled()						\
({								\
	unsigned long _flags;					\
								\
	hw_local_save_flags(_flags);				\
	hw_raw_irqs_disabled_flags(_flags);			\
})

#if 0
static DEFINE_SPINLOCK(deepsleep_lock);
#endif



void sc8800g2_delay(void);
/*********************************/
/*********************************/



u32 reg_gen0_val, reg_busclk_alm, reg_ahb_ctl0_val, reg_gen_clk_en, reg_gen_clk_gen5, reg_ahb_ctl3, reg_intc_en;


#define GEN0_MASK ( GEN0_SIM0_EN | GEN0_I2C_EN | GEN0_GPIO_EN | \
			   GEN0_I2C0_EN|GEN0_I2C1_EN|GEN0_I2C2_EN|GEN0_I2C3_EN | \
			   GEN0_SPI0_EN|GEN0_SPI1_EN| GEN0_I2S0_EN | GEN0_I2S1_EN| \
	                GEN0_EFUSE_EN | GEN0_I2S_EN | GEN0_PIN_EN | \
	                GEN0_EPT_EN | GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | \
	                GEN0_UART1_EN | GEN0_UART2_EN)

#define CLK_EN_MASK (CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN)
#define BUSCLK_ALM_MASK (ARM_VB_MCLKON|ARM_VB_DA0ON|ARM_VB_DA1ON|ARM_VB_ADCON|ARM_VB_ANAON|ARM_VB_ACC)
#define AHB_CTL0_MASK   (AHB_CTL0_DCAM_EN|AHB_CTL0_CCIR_EN|AHB_CTL0_LCDC_EN|    \
                         AHB_CTL0_SDIO0_EN|AHB_CTL0_SDIO1_EN|AHB_CTL0_DMA_EN|     \
                         AHB_CTL0_BM0_EN |AHB_CTL0_NFC_EN|AHB_CTL0_BM1_EN|       \
                         AHB_CTL0_G2D_EN|AHB_CTL0_G3D_EN|	\
                         AHB_CTL0_AXIBUSMON0_EN|AHB_CTL0_AXIBUSMON1_EN|	\
                         AHB_CTL0_VSP_EN|AHB_CTL0_ROT_EN | AHB_CTL0_USBD_EN)

#define GR_CLK_EN_MASK CLK_EN_MASK
#define GR_GEN0_MASK GEN0_MASK

#define INT_IRQ_MASK	(BIT_3)/*UART1*///|(BIT_5)/*TIME0*/

//register save
#define SAVE_REG(_reg_save, _reg_addr, _reg_mask)  {_reg_save = (*(volatile uint32*)(_reg_addr) & ((uint32)_reg_mask));}
#define SAVE_ANA_REG(_reg_save, _reg_addr, _reg_mask) {_reg_save = (ANA_REG_GET(_reg_addr) & ((uint32)_reg_mask));}

#define SAVE_GLOBAL_REG  do{ \
        SAVE_REG(reg_gen_clk_en, GR_CLK_EN, GR_CLK_EN_MASK); \
        SAVE_REG(reg_gen0_val, GR_GEN0, GR_GEN0_MASK);   \
        SAVE_REG(reg_busclk_alm, GR_BUSCLK_ALM, BUSCLK_ALM_MASK);    \
        SAVE_REG(reg_ahb_ctl0_val, AHB_CTL0, AHB_CTL0_MASK);\
    }while(0)

//register restore
#define REG_LOCAL_VALUE_DEF   uint32 reg_val;
#define RESTORE_REG(_reg_addr, _reg_mask, _reg_save)    do{reg_val = *(volatile uint32*)(_reg_addr); \
        reg_val &= ~((uint32)_reg_mask);    \
        reg_val |= ((_reg_save) & ((uint32)_reg_mask));    \
        *(volatile uint32*)(_reg_addr) = reg_val; \
    }while(0)

#define RESTORE_ANA_REG(_reg_addr, _reg_mask, _reg_save)    do{reg_val = ANA_REG_GET(_reg_addr); \
        reg_val &= ~((uint32)_reg_mask);    \
        reg_val |= ((_reg_save) & ((uint32)_reg_mask));    \
        ANA_REG_SET(_reg_addr, reg_val); \
    }while(0)

#define RESTORE_GLOBAL_REG   do{  \
        RESTORE_REG(GR_CLK_EN, GR_CLK_EN_MASK, reg_gen_clk_en);   \
        RESTORE_REG(GR_BUSCLK_ALM, BUSCLK_ALM_MASK, reg_busclk_alm);   \
        RESTORE_REG(GR_GEN0, GR_GEN0_MASK, reg_gen0_val);\
        RESTORE_REG(AHB_CTL0, AHB_CTL0_MASK, reg_ahb_ctl0_val); \
    }while(0)


extern void sp_arch_idle(void);
extern void trace(void);
extern int sp_pm_collapse(void);
extern void sp_pm_collapse_exit(void);
extern void sc8810_standby_iram(void);
extern void sc8810_standby_iram_end(void);
extern void sc8810_standby_exit_iram(void);


#define SAVED_VECTOR_SIZE 64
static uint32_t *sp_pm_reset_vector = NULL;
static uint32_t saved_vector[SAVED_VECTOR_SIZE];

u32 __attribute__ ((naked)) sc8800g_read_cpsr(void)
{
	__asm__ __volatile__("mrs r0, cpsr\nbx lr");
}

#define UART_STS0 (SPRD_SERIAL1_BASE + 0x08)
#define UART_STS1 (SPRD_SERIAL1_BASE + 0x0c)

#define UART_TRANSFER_REALLY_OVER (0x1UL << 15)

static void wait_until_uart1_tx_done(void)
{
    u32 tx_fifo_val;
	u32 really_done = 0;

	if (__raw_readl(GR_PCTL) & BIT_8/* UART1_SEL */) return ;//uart1 owner dsp sel

	/* fifo depth = 128. */
	u32 timeout = 200;	
	    tx_fifo_val = __raw_readl(UART_STS1);
	    tx_fifo_val >>= 8;
	    tx_fifo_val &= 0xff;
	while(tx_fifo_val != 0) {
		/*
		WARN_ON(0 >= timeout);
		*/
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



static int is_dsp_sleep(void)
{
	u32 val;
	int ret_val = 0;

	/*
	printk("####: check register: GR_STC_STATE for DSP\n");
	*/
	val = __raw_readl(GR_STC_STATE);
	/*
	printk("######: GR_STC_STATE =%08x\n", val);
	*/
	if (GR_DSP_STOP & val) {
		/*
		printk("#####: GR_STC_STATE[DSP_STOP] is set!\n");
		*/
	}
	else {
		//printk("#####: GR_STC_STATE[DSP_STOP] is NOT set!\n");
		ret_val = -1;
	}
#if 0
	/*
	printk("####: check register: GR_CLK_DLY for DSP\n");
	*/
	val = __raw_readl(GR_CLK_DLY);
	/*
	printk("######: GR_CLK_DLY =%08x\n", val);
	*/
	if (DSP_DEEP_STOP & val) {
		//printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is set!\n");
	}
	else {
		//printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is NOT set!\n");
		ret_val = -2;
	}
#endif
	return ret_val;
}

static int verify_dsp_deep_sleep_by_value(u32 val1, u32 val2)
{
	int ret_val = 0;

	printk("####: check register: GR_STC_STATE for DSP\n");
	printk("######: GR_STC_STATE =%08x\n", val1);
	if (GR_DSP_STOP & val1) {
		printk("#####: GR_STC_STATE[DSP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_STC_STATE[DSP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	printk("####: check register: GR_CLK_DLY for DSP\n");
	printk("######: GR_CLK_DLY =%08x\n", val2);

	if (DSP_DEEP_STOP & val2) {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_SYS_STOP & val2) {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_AHB_STOP & val2) {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (DSP_MTX_STOP & val2) {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_CORE_STOP & val2) {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (GR_EMC_STOP_CH5 & val2) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH4 & val2) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH3 & val2) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is NOT set!\n");
		ret_val = -EINVAL;
	}

	return ret_val;
}


static int verify_ahb_sts_by_value(u32 val)
{
	int ret_val = 0;

	printk("####: check register: AHB_STS for ARM\n");
	printk("######: AHB_STS =%08x\n", val);

	if (EMC_STOP_CH0 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH0] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH0] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (EMC_STOP_CH1 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH1] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH1] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH2 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH2] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH2] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH3 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH3] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH3] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH4 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH4] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH4] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH5 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH5] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH5] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH6 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH6] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH6] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH7 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH7] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH7] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH8 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH8] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH8] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (ARMMTX_STOP_CH0 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH0] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH0] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (ARMMTX_STOP_CH1 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH1] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH1] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (ARMMTX_STOP_CH2 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH2] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH2] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (AHB_STS_EMC_STOP & val) {
		printk("#####: AHB_STS[AHB_STS_EMC_STOP] is set!\n");
	}
	else {
		printk("#####: AHB_STS[AHB_STS_EMC_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (AHB_STS_EMC_SLEEP & val) {
		printk("#####: AHB_STS[AHB_STS_EMC_SLEEP] is set!\n");
	}
	else {
		printk("#####: AHB_STS[AHB_STS_EMC_SLEEP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DMA_BUSY & val) {
		printk("#####: AHB_STS[DMA_BUSY] is set!\n");
	}
	else {
		printk("#####: AHB_STS[DMA_BUSY] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_MAHB_SLEEP_EN & val) {
		printk("#####: AHB_STS[DSP_MAHB_SLEEP_EN] is set!\n");
	}
	else {
		printk("#####: AHB_STS[DSP_MAHB_SLEEP_EN] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (APB_PRI_EN & val) {
		printk("#####: AHB_STS[APB_PRI_EN] is set!\n");
	}
	else {
		printk("#####: AHB_STS[APB_PRI_EN] is NOT set!\n");
		ret_val = -EINVAL;
	}

	return ret_val;
}

void sc8800g_set_pll(void)
{
#if 0
	u32 val1, val2;

	printk("##: set new pll value!\n");
	printk("##: set new pll value!\n");
	printk("##: set new pll value!\n");
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	val2 &= 0xfffff800;
	val2 |= SC8800G2_DEFAULT_PLL_N;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
#endif
}
EXPORT_SYMBOL(sc8800g_set_pll);


void sc8800g_save_pll(void)
{
#if 0
	u32 val1, val2;
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	pll_n = val2 & 0x07ff;
	val2 &= 0xfffff800;
	val2 |= SC8800G2_DEFAULT_PLL_N;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
#endif
}

void sc8800g_restore_pll(void)
{
#if 0
	u32 val1, val2;
	if (pll_n > 0x07ff) pll_n = SC8800G2_DEFAULT_PLL_N;
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	val2 &= 0xfffff800;
	val2 |= pll_n;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
#endif
}

void sc8800g_pll_default(void)
{
#if 0
	u32 val1, val2;
	if (pll_n > 0x07ff) pll_n = SC8800G2_DEFAULT_PLL_N;
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	val2 &= 0xffff800;
	val2 |= pll_n;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
#endif
}


void __iomem *iram_start;

#if 0
#define PGD_ENTRY 4096
pmd_t pmd_saved0, pmd_saved1;
pmd_t *pmd_saved = NULL;
static DEFINE_SPINLOCK(map_lock);

/* create 1:1 maping for IRAM. */
static void map_iram_identically(void)
{
	unsigned long base_pmdval;
	pgd_t *pgd, *pgd_k;	
	unsigned long pmdval;
	struct mm_struct *current_mm ;
        int i;
	int iram_adr = IRAM_BASE_PHY;
       u32 * pint = (u32 *)IRAM_START_PHY;
       pmd_t *pmd, *pmd_k;

    
	flush_tlb_all();
    flush_cache_all();


	if (current->mm && current->mm->pgd)
		current_mm = current->mm;
	else
		current_mm = &init_mm;

/************************/

       printk("###############################\n");
       printk("###############################\n");

       printk("###: iram_start = %p\n", iram_start);
       pgd = pgd_offset(current_mm, (u32)iram_start);
	pmd = pmd_offset(pgd, iram_adr);
	printk("##: pmd at: %p.\n", pmd);
	printk("##: pmd[0] =%08x, pmd[1] = %08x.\n", pmd[0], pmd[1]);
       printk("###############################\n");
       printk("###############################\n");
 

/************************/



       printk("###: swapper_pg_dir at: %p.\n", swapper_pg_dir);  
      printk("###: pgd[0] at: %p.\n", current_mm->pgd);  
       printk("###: pgd_k[0] at: %p.\n", init_mm.pgd);  

	pint = current_mm->pgd;
/*
       for (i = 0; i < PGD_ENTRY; i++) {
		printk("pgd[%d] = %08x\n", i, pint[i]);
	}
*/

       pgd = pgd_offset(current_mm, iram_adr);
	pgd_k =pgd_offset_k(iram_adr);

       printk("###: pgd[%08x] at: %p.\n", iram_adr, pgd);  
       printk("###: pgd_k[%08x] at: %p.\n", iram_adr, pgd_k);  

	base_pmdval = PMD_SECT_AP_READ | PMD_TYPE_SECT;
	if (cpu_architecture() <= CPU_ARCH_ARMv5TEJ && !cpu_is_xscale())
		base_pmdval |= PMD_BIT4;


		pmd = pmd_offset(pgd, iram_adr);
		printk("##: pmd at: %p.\n", pmd);
		pmd_k = pmd_offset(pgd_k, iram_adr);
		printk("##: pmd at: %p.\n", pmd_k);
               pmd_saved = pmd;
               pmd_saved0 = pmd[0];
               pmd_saved1 = pmd[1];

		pmdval = ((IRAM_BASE_PHY & SECTION_MASK)  | base_pmdval);
		pmd[0] = __pmd(pmdval);
		pmdval = (((IRAM_BASE_PHY + SZ_1M) & SECTION_MASK)  | base_pmdval);
		pmd[1] = __pmd(pmdval);

		flush_pmd_entry(pmd);


	pint = current_mm->pgd;
       for (i = 0; i < 3072; i++) {
               if (pint[i] != 0x0) {
			printk("!!!!!!:  pgd[%d] = %08x\n", i, pint[i]);
		}
	}


		printk("##: pmd[0] =%08x, pmd[1] = %08x.\n", pmd[0], pmd[1]);
		printk("##: pmd_k[0] =%08x, pmd_k[1] = %08x.\n", pmd_k[0], pmd_k[1]);

	printk("###: c1 = %08x.\n", sc8800g_read_cp15_c1());
	printk("###: c3 = %08x.\n", sc8800g_read_cp15_c3());

		pint = (u32 *)IRAM_START_PHY;
		for (i = 0; i <8; i++) printk("pint[%p] = %08x\n", &pint[i], pint[i]);

}
static void map_restore(void)
{
    flush_tlb_all();

    if (pmd_saved) { 
        pmd_saved[0] = pmd_saved0;
        pmd_saved[1] = pmd_saved1;
    }
    flush_pmd_entry(pmd_saved);
}

void sc8800g_enter_deepsleep_internal(void)
{
	unsigned long flags;
	hw_spin_lock_irqsave(&deepsleep_lock, flags);
	sc8800g_cpu_standby_prefetch();
	hw_spin_unlock_irqrestore(&deepsleep_lock, flags);

}
#endif

#if 0
static int enable_mcu_sleep(void)
{
    u32 val;
    /* AHB_PAUSE */
    val = __raw_readl(AHB_PAUSE);
    val |= (MCU_DEEP_SLEEP_EN | MCU_SYS_SLEEP_EN);
    __raw_writel(val, AHB_PAUSE);
    return 0;
}
#endif

void timer_stats_reset(void);
void timer_stats_print(void);


static struct wake_lock messages_wakelock;
static struct wake_lock idle_wakelock;


u32 timer_int_counter0 = 0, timer_int_counter1 = 0;
u32 tick_sched_timer_counter0 = 0, tick_sched_timer_counter1 = 0;
u32 sleep_counter0 = 0, sleep_counter1 = 0;
u32 interrupt_counter0 = 0, interrupt_counter1 = 0;
u32 schedu_counter0 = 0, schedu_counter1 = 0;

#define THREAD_INTERVAL 30

int sprd_irq_info_enable = 0;
int sprd_thread_info_enable = 0;
int sprd_statistic_info_enable = 0;
int sprd_clock_info_enable = 1;
int sprd_timer_info_enable = 0;
int sprd_check_dsp_enable = 0;
int sprd_check_gpio_enable = 0;
int sprd_dump_gpio_registers = 0;
int sprd_wait_until_uart_tx_fifo_empty = 1;
int sprd_sleep_mode_info = 0;
int sprd_sc8810_deepsleep_enable = 1;

int sprd_no_deep_sleep_enable = 0;
int sprd_idle_test_enable = 0;
int sprd_idle_deep_enable = 0;

int sprd_pm_message_enable = 0;
static u32 time0 = 0, time1 = 0, time_duration;
static u32 sleep_time_local1 = 0, sleep_time_local2 = 0, sleep_duration;
static u32 sleep_time_inidle_local1 = 0, sleep_time_inidle_local2 = 0, sleep_duration_inidle;

static u32 arch_idle_local1 = 0, arch_idle_local2 = 0, arch_idle_total = 0, arch_idle_delta = 0;


/* data for pm_suspend. */
int sprd_suspend_enable = 1;
u32 sprd_suspend_interval =  0;

static int print_thread(void *pdata)
{
	u32 val;
    while(1) {
	wake_lock(&messages_wakelock);
	add_pm_message(get_sys_cnt(), "************* print_thread start. *************", 0, 0, 0); 
	stop_pm_message();
	if (0 != sprd_suspend_interval) printk("@@@@: sprd_suspend_interval = %u @@@@.\n", 
			sprd_suspend_interval);

	if (!sprd_suspend_enable) printk("@@@@:  suspend is disabled! @@@@.\n");

	sleep_time_local1 = sleep_time_local2;
	sleep_time_local2 = sleep_time;
	sleep_duration = sleep_time_local2 - sleep_time_local1;

	sleep_time_inidle_local1 = sleep_time_inidle_local2;
	sleep_time_inidle_local2 = sleep_time_inidle;
	sleep_duration_inidle = sleep_time_inidle_local2 - sleep_time_inidle_local1;

 
	arch_idle_local1 = arch_idle_local2;
	arch_idle_local2 = arch_idle_total;
	arch_idle_delta = arch_idle_local2 - arch_idle_local1;


	time0 = time1;
	time1 = get_sys_cnt();
	time_duration = (time1 - time0);
	printk("\n####: thread period: [%d + %d + [arch_idle = %d]]/[%d]\n", 
		sleep_duration, sleep_duration_inidle, arch_idle_delta, time_duration);
	time_duration /= 1000;

	uptime = get_sys_cnt();

	if (sprd_clock_info_enable) {
		printk("\n===========================\n");
	    val = __raw_readl(GR_GEN0);
		printk("##: GR_GEN0 = %08x.\n", val);
		if (val & GEN0_SIM0_EN) printk("GEN0_SIM0_EN =1.\n");
		if (val & GEN0_I2C_EN) printk("GEN0_I2C_EN =1.\n");
		if (val & GEN0_GPIO_EN) printk("GEN0_GPIO_EN =1.\n");
		if (val & GEN0_I2C0_EN) printk("GEN0_I2C0_EN =1.\n");
		if (val & GEN0_I2C1_EN) printk("GEN0_I2C1_EN =1.\n");
		if (val & GEN0_I2C2_EN) printk("GEN0_I2C2_EN =1.\n");
		if (val & GEN0_I2C3_EN) printk("GEN0_I2C3_EN =1.\n");
		if (val & GEN0_SPI0_EN) printk("GEN0_SPI0_EN =1.\n");
		if (val & GEN0_SPI1_EN) printk("GEN0_SPI1_EN =1.\n");
		if (val & GEN0_I2S0_EN) printk("GEN0_I2S0_EN =1.\n");
		if (val & GEN0_I2S1_EN) printk("GEN0_I2S1_EN =1.\n");
		if (val & GEN0_EFUSE_EN) printk("GEN0_EFUSE_EN =1.\n");
		if (val & GEN0_I2S_EN) printk("GEN0_I2S_EN =1.\n");
		if (val & GEN0_PIN_EN) printk("GEN0_PIN_EN =1.\n");
		if (val & GEN0_EPT_EN) printk("GEN0_EPT_EN =1.\n");
		if (val & GEN0_SIM1_EN) printk("GEN0_SIM1_EN =1.\n");
		if (val & GEN0_SPI_EN) printk("GEN0_SPI_EN =1.\n");
		if (val & GEN0_UART0_EN) printk("GEN0_UART0_EN =1.\n");
		if (val & GEN0_UART1_EN) printk("GEN0_UART1_EN =1.\n");
		if (val & GEN0_UART2_EN) printk("GEN0_UART2_EN =1.\n");

	    val = __raw_readl(GR_CLK_EN);
		printk("##: GR_CLK_EN = %08x.\n", val);
		if (val & CLK_PWM0_EN) printk("CLK_PWM0_EN =1.\n");
		if (val & CLK_PWM1_EN) printk("CLK_PWM1_EN = 1.\n");
		if (val & CLK_PWM2_EN) printk("CLK_PWM2_EN = 1.\n");
		if (val & CLK_PWM3_EN) printk("CLK_PWM3_EN = 1.\n");

	    u32 val = __raw_readl(GR_BUSCLK_ALM);
		printk("##: GR_BUSCLK_ALM = %08x.\n", val);
		if (val & ARM_VB_MCLKON) printk("ARM_VB_MCLKON =1.\n");
		if (val & ARM_VB_DA0ON) printk("ARM_VB_DA0ON = 1.\n");
		if (val & ARM_VB_DA1ON) printk("ARM_VB_DA1ON = 1.\n");
		if (val & ARM_VB_ADCON) printk("ARM_VB_ADCON = 1.\n");
		if (val & ARM_VB_ANAON) printk("ARM_VB_ANAON = 1.\n");
		if (val & ARM_VB_ACC) printk("ARM_VB_ACC = 1.\n");

	    val = __raw_readl(AHB_CTL0);
		printk("##: AHB_CTL0 = %08x.\n", val);
		if (val & AHB_CTL0_DCAM_EN) printk("AHB_CTL0_DCAM_EN =1.\n");
		if (val & AHB_CTL0_CCIR_EN) printk("AHB_CTL0_CCIR_EN =1.\n");
		if (val & AHB_CTL0_LCDC_EN) printk("AHB_CTL0_LCDC_EN =1.\n");
		if (val & AHB_CTL0_SDIO0_EN) printk("AHB_CTL0_SDIO0_EN =1.\n");
		if (val & AHB_CTL0_SDIO1_EN) printk("AHB_CTL0_SDIO1_EN =1.\n");
		if (val & AHB_CTL0_DMA_EN) printk("AHB_CTL0_DMA_EN =1.\n");
		if (val & AHB_CTL0_BM0_EN) printk("AHB_CTL0_BM0_EN =1.\n");
		if (val & AHB_CTL0_NFC_EN) printk("AHB_CTL0_NFC_EN =1.\n");
		if (val & AHB_CTL0_BM1_EN) printk("AHB_CTL0_BM1_EN =1.\n");
		if (val & AHB_CTL0_G2D_EN) printk("AHB_CTL0_G2D_EN =1.\n");
		if (val & AHB_CTL0_G3D_EN) printk("AHB_CTL0_G3D_EN =1.\n");
		if (val & AHB_CTL0_AXIBUSMON0_EN) printk("AHB_CTL0_AXIBUSMON0_EN =1.\n");
		if (val & AHB_CTL0_AXIBUSMON1_EN) printk("AHB_CTL0_AXIBUSMON1_EN =1.\n");
		if (val & AHB_CTL0_VSP_EN) printk("AHB_CTL0_VSP_EN =1.\n");
		if (val & AHB_CTL0_ROT_EN) printk("AHB_CTL0_ROT_EN =1.\n");
		if (val & AHB_CTL0_USBD_EN) printk("AHB_CTL0_USBD_EN =1.\n");

		printk("\n===========================\n");

		val = ANA_REG_GET(ANA_LDO_PD_CTL0);
		printk("##: ANA_LDO_PD_CTL0 = %04x.\n", val);
		if ((val & LDO_USB_CTL)) printk("##: LDO_USB_CTL is on.\n");
		else if(!(val & (LDO_USB_CTL >> 1))) printk("##: LDO_USB_CTL is not off.\n");

		if ((val & LDO_SDIO0_CTL)) printk("##: LDO_SDIO0_CTL is on.\n");
		else if(!(val & (LDO_SDIO0_CTL >> 1))) printk("##: LDO_SDIO0_CTL is not off.\n");

		if ((val & LDO_SIM0_CTL)) printk("##: LDO_SIM0_CTL is on.\n");
		else if(!(val & (LDO_SIM0_CTL >> 1))) printk("##: LDO_SIM0_CTL is not off.\n");

		if ((val & LDO_SIM1_CTL)) printk("##: LDO_SIM1_CTL is on.\n");
		else if(!(val & (LDO_SIM1_CTL >> 1))) printk("##: LDO_SIM1_CTL is not off.\n");

		if ((val & LDO_BPCAMD0_CTL)) printk("##: LDO_BPCAMD0_CTL is on.\n");
		else if(!(val & (LDO_BPCAMD0_CTL >> 1))) printk("##: LDO_BPCAMD0_CTL is not off.\n");

		if ((val & LDO_BPCAMD1_CTL)) printk("##: LDO_BPCAMD1_CTL is on.\n");
		else if(!(val & (LDO_BPCAMD1_CTL >> 1))) printk("##: LDO_BPCAMD1_CTL is not off.\n");

		if ((val & LDO_BPCAMA_CTL)) printk("##: LDO_BPCAMA_CTL is on.\n");
		else if(!(val & (LDO_BPCAMA_CTL >> 1))) printk("##: LDO_BPCAMA_CTL is not off.\n");

		if ((val & LDO_BPVB_CTL)) printk("##: LDO_BPVB_CTL is on.\n");
		else if(!(val & (LDO_BPVB_CTL >> 1))) printk("##: LDO_BPVB_CTL is not off.\n");


		val = ANA_REG_GET(ANA_LDO_PD_CTL1);
		printk("##: ANA_LDO_PD_CTL1 = %04x.\n", val);
		if ((val & LDO_SDIO1_CTL)) printk("##: LDO_SDIO1_CTL is on.\n");
		else if(!(val & (LDO_SDIO1_CTL >> 1))) printk("##: LDO_SDIO1_CTL is not off.\n");

		if ((val & LDO_BPWIF0_CTL)) printk("##: LDO_BPWIF0_CTL is on.\n");
		else if(!(val & (LDO_BPWIF0_CTL >> 1))) printk("##: LDO_BPWIF0_CTL is not off.\n");

		if ((val & LDO_BPWIF1_CTL)) printk("##: LDO_BPWIF1_CTL is on.\n");
		else if(!(val & (LDO_BPWIF1_CTL >> 1))) printk("##: LDO_BPWIF1_CTL is not off.\n");

		if ((val & LDO_SIM2_CTL)) printk("##: LDO_SIM2_CTL is on.\n");
		else if(!(val & (LDO_SIM2_CTL >> 1))) printk("##: LDO_SIM2_CTL is not off.\n");

		if ((val & LDO_SIM3_CTL)) printk("##: LDO_SIM3_CTL is on.\n");
		else if(!(val & (LDO_SIM3_CTL >> 1))) printk("##: LDO_SIM3_CTL is not off.\n");


		printk("\n===========================\n");
		val = ANA_REG_GET(ANA_AUDIO_PA_CTRL0);
		printk("##: ANA_AUDIO_PA_CTRL0 = %04x.\n", val);
		if (val & AUDIO_PA_ENABLE)	 printk("##: Audo PA is enabled.\n");	
		else if (!(val & AUDIO_PA_ENABLE_RST)) printk("##: Audo PA is not stopped.\n");

		val = ANA_REG_GET(ANA_AUDIO_PA_CTRL1);
		printk("##: ANA_AUDIO_PA_CTRL1 = %04x.\n", val);
		if (val & AUDIO_PA_LDO_ENABLE)	 printk("##: Audo PA_LDO is enabled.\n");	
		else if (!(val & AUDIO_PA_LDO_ENABLE_RST)) printk("##: Audo PA_LDO is not stopped.\n");
		printk("\n===========================\n");

	}
#ifdef SPRD_COPROCESSOR_INFO
	printk("###: c1 = %08x.\n", sc8800g_read_cp15_c1());
	printk("###: c2 = %08x.\n", sc8800g_read_cp15_c2());
	printk("###: c3 = %08x.\n", sc8800g_read_cp15_c3());
#endif

	if (sprd_statistic_info_enable) {
		sleep_counter0 = sleep_counter1;
		sleep_counter1 = sleep_counter;
	           printk("##: thread_loops = %d, idle_loops = %d\n", ++thread_loops, idle_loops);           
		    printk("##: mode = %d, sleep_counter = %d freq = %d/s.\n", 	sleep_mode, 
			sleep_counter, (sleep_counter1 - sleep_counter0) / time_duration);
		    printk("##[%d]: uptime = %d, sleep_time = %d, idle_time = %d.\n", 
	                (sleep_time * 100) /uptime, uptime, sleep_time, idle_time);

		timer_int_counter0 = timer_int_counter1;
		timer_int_counter1 = timer_int_counter;
		printk("****: timer_interrupt = %d events/s\n", 
			(timer_int_counter1 - timer_int_counter0) / time_duration);

		tick_sched_timer_counter0 = tick_sched_timer_counter1;
		tick_sched_timer_counter1 = tick_sched_timer_counter;
		printk("****: tick_sched_timer_counter = %d events/s\n", 
			(tick_sched_timer_counter1 - tick_sched_timer_counter0) / time_duration);

		interrupt_counter0 = interrupt_counter1;
		interrupt_counter1 = interrupt_counter;
		printk("****: interrupt_counter = %d events/s\n", 
			(interrupt_counter1 - interrupt_counter0) / time_duration);

		schedu_counter0 = schedu_counter1;
		schedu_counter1 = schedu_counter;
		printk("****: schedu_counter = %d events/s\n", 
			(schedu_counter1 - schedu_counter0) / time_duration);

	}
	if (sprd_irq_info_enable) {
		show_sprd_irq_info();
		irq_statistic_reset();
	}

	if (sprd_thread_info_enable) {
		show_sprd_thread_info();
		threads_statistic_reset();
	}
  
#if 0       
	printk("#######  print_thread() ##########\n");
	printk("##: xtime = %lld.\n", timespec_to_ns(&xtime));
	printk("##: wall_to_monotonic = %lld.\n", timespec_to_ns(&wall_to_monotonic));
	getboottime(&now_ts_pm);
	printk("##: getboottime() = %lld.\n", timespec_to_ns(&now_ts_pm));
	getrawmonotonic(&now_ts_pm);
	printk("##: getrawmonotonic() = %lld.\n", timespec_to_ns(&now_ts_pm));
	printk("##: ktime_get() = %lld.\n", ktime_to_ns(ktime_get()));
	getnstimeofday(&now_ts_pm);
	printk("##: getnstimeofday() = %lld.\n", timespec_to_ns(&now_ts_pm));
#endif

	if (sprd_pm_message_enable) print_pm_message();

	if (sprd_clock_info_enable) {
		printk("##: show clock info:\n");
		sc8800g_get_clock_info();
	}
	if (sprd_timer_info_enable){
		timer_stats_print();
		timer_stats_reset();
	}

#if 0
	verify_dsp_deep_sleep_by_value(gr_stc_state);
	verify_ahb_sts_by_value(ahb_sts);
#endif

	/* chcecking DSP. */
	if (sprd_check_dsp_enable) {

	u32 start_time;
	u32 stop_time;
	int dsp_status;
	u32 checking_counter = 0;

		if (sleep_counter >= 0) {
			start_time = get_sys_cnt();
			stop_time = get_sys_cnt();
			while((stop_time - start_time) < (1000 * 60 * 1)) {
				checking_counter++;
				val = __raw_readl(GR_STC_STATE);
				/*
				printk("##: checking DSP status[%d] GR_STC_STATE = %08x\n", 
						checking_counter, val);
				*/
				dsp_status = is_dsp_sleep();
				if (0 == dsp_status) {
					printk("##: DSP is in sleep status.\n");
					printk("##: DSP is in sleep status.\n");
					printk("##: DSP is in sleep status.\n");
					printk("##: DSP is in sleep status.\n");
					break;
				}
				else {
					printk("@@: DSP is NOT in sleep status.\n");
					printk("@@: DSP is NOT in sleep status.\n");
					printk("@@: DSP is NOT in sleep status.\n");
					printk("@@: DSP is NOT in sleep status.\n");
				}
				udelay(300);
				stop_time = get_sys_cnt();
			}
		}
	}

	/* detect GPIO. */
	/*
	for (i = 0; i < 10; i++) {
		val = __raw_readl(SPRD_GPIO_BASE + GPIO_IE + 0x80 * i);
		printk("##: GPIO_IE = %08x\n", val);
		__raw_writel(0x0, SPRD_GPIO_BASE + GPIO_IE + 0x80 * i);
	}
	*/
	

	    if (has_wake_lock_info(WAKE_LOCK_SUSPEND)) {
			printk("##: Some locks are being holded.\n");
	    }
           msleep(100);
	start_pm_message();
	add_pm_message(get_sys_cnt(), "**** print_thread stop. *****", 0, 0, 0); 
           wake_unlock(&messages_wakelock);
           set_current_state(TASK_INTERRUPTIBLE);
           schedule_timeout(THREAD_INTERVAL * HZ);
    }
    return 0;
}


int disable_audio_module(void)
{
    
    u32 val = __raw_readl(GR_BUSCLK_ALM);
    val &= ~BUSCLK_ALM_MASK;
    __raw_writel(val, GR_BUSCLK_ALM);
    return 0;
}

int disable_apb_module(void)
{
    u32 val = __raw_readl(GR_GEN0);
    val &= ~GEN0_MASK;
    __raw_writel(val, GR_GEN0);

    val = __raw_readl(GR_CLK_EN);
    val &= ~CLK_EN_MASK;
    __raw_writel(val, GR_CLK_EN);

    return 0;
}

int sleep_wait_emc_sleep(void)
{
    int i ;
    u32 emc_wait_channel_mask = (BIT_6 | BIT_7 | BIT_8);
    u32 val;

    for (i =  0; i < 0x200; i++) {
        val = __raw_readl(EXT_MEM_STS3);
        if ( (val & emc_wait_channel_mask) == emc_wait_channel_mask) {
            return 1;
        }
    }

    return 0;
}

static void disable_ahb_module (void)
{
    u32 val;
#if 0
    if (!sleep_wait_emc_sleep()) {
        printk("###: EMC channel[6,7,8] is NOT idle!\n");
        printk("###: EMC channel[6,7,8] is NOT idle!\n");
        printk("###: EMC channel[6,7,8] is NOT idle!\n");    
        return;
    }
#endif
    val = __raw_readl(AHB_CTL0);
    val &= ~AHB_CTL0_MASK;
    __raw_writel(val, AHB_CTL0);
}


int supsend_ldo_turnoff(void)
{
#if 0
	u32 val = 0;
 
	val = ANA_REG_GET(ANA_LDO_SLP);
	if ( val != 0xa7fb) {
		printk("##: ANA_LDO_SLP: wrong vaule[%08x].\n", val);
	}

	val = ANA_REG_GET(ANA_LDO_PD_CTL);
	if ((val & 0x03) != 0x01) printk("##: USB LDO was wrong!\n");

	val = ANA_REG_GET(ANA_ANA_CTL0);
	if (!(val & FSM_AFCPD_EN)) printk("##: FSM_AFCPD_EN was not enabled!\n");

	is_dsp_sleep();
#endif
	return 0;
}

int supsend_ldo_turnon(void)
{

	return 0;
}


int supsend_gpio_save(void)
{
	u32 val = 0;
#ifdef CONFIG_MACH_SP6810A
	/* GPIO 96, shutdown audio PA. */
	val = __raw_readl(SPRD_GPIO_BASE + 0x0300);
	val &= ~BIT_0;
	__raw_writel(val, SPRD_GPIO_BASE + 0x0300);
#endif

	return 0;
}

int supsend_gpio_restore(void)
{

	return 0;
}

#define PD_AUTO_EN     BIT_22
int sc8810_setup_pd_automode(void)
{
	//__raw_writel(0x06000320|PD_AUTO_EN, GR_GPU_PWR_CTRL);//reserved
	__raw_writel(0x06000320|PD_AUTO_EN, GR_MM_PWR_CTRL);
	__raw_writel(0x06000320|PD_AUTO_EN, GR_G3D_PWR_CTRL);//GPU
	__raw_writel(0x04000720/*|PD_AUTO_EN*/, GR_CEVA_RAM_TH_PWR_CTRL);
	__raw_writel(0x05000520/*|PD_AUTO_EN*/, GR_GSM_PWR_CTRL);
	__raw_writel(0x05000520/*|PD_AUTO_EN*/, GR_TD_PWR_CTRL);
	__raw_writel(0x04000720/*|PD_AUTO_EN*/, GR_CEVA_RAM_BH_PWR_CTRL);
	__raw_writel(0x03000920/*|PD_AUTO_EN*/, GR_PERI_PWR_CTRL);
	if (__raw_readl(CHIP_ID) == CHIP_ID_VER_0) {//original version
		__raw_writel(0x02000a20|PD_AUTO_EN, GR_ARM_SYS_PWR_CTRL);
		__raw_writel(0x07000f20|BIT_23, GR_POWCTL0);  //ARM Core auto poweroff
	}
	else {
		__raw_writel(0x02000f20|PD_AUTO_EN, GR_ARM_SYS_PWR_CTRL);
		__raw_writel(0x07000a20|BIT_23, GR_POWCTL0);  //ARM Core auto poweroff
	}
//	__raw_writel((0x07<<29)|(30<<21)|(30<<13)|(60<<3), GR_GEN4);	//xtl pll wait
}

int sc8810_setup_ldo_slpmode(void)
{
//	 ANA_REG_SET(ANA_LDO_PD_CTL0, 0x5555);
//	 ANA_REG_SET(ANA_LDO_PD_CTL1, 0x0155);
#if defined(CONFIG_MACH_SP6820A)
	 ANA_REG_SET(ANA_LDO_SLP0, 0x27f3);//except v18/28, SIM0,1
#elif defined(CONFIG_MACH_SP8810)
	ANA_REG_SET(ANA_LDO_SLP0, 0x27f3);//except v18/28, SIM0,1
#else
	ANA_REG_SET(ANA_LDO_SLP0, 0xa7fb);//except v18/28, SIM0
#endif
	 ANA_REG_SET(ANA_LDO_SLP1, 0x801d|(1<<12));//ARMDCDC_PWR_ON_DLY = 1, Not Hold ARMDCDC
	 ANA_REG_SET(ANA_LDO_SLP2, 0x0f20);//a-die armdcdc iso
//	 ANA_REG_SET(ANA_DCDC_CTRL, 0x0025);
//	 ANA_REG_SET(ANA_DCDC_CTRL_DS, 0x0f43);//dcdc lvl dly, and hold vcccore 1.1v
	 ANA_REG_SET(ANA_LED_CTRL, 0x801f);//all led off
}

struct workqueue_struct *deep_sleep_work_queue;
static void deep_sleep_suspend(struct work_struct *work);

static DECLARE_WORK(deep_sleep_wrok, deep_sleep_suspend);
static void deep_sleep_suspend(struct work_struct *work)
{
	verify_dsp_deep_sleep_by_value(gr_stc_state, gr_clk_dly);
	verify_ahb_sts_by_value(ahb_sts);
}

#define IRQ_GPIO (0x1UL << 8)
/*
int in_calibration(void);
*/

extern int outer_cache_poweron(void);
extern int outer_cache_poweroff(void);

#ifdef CONFIG_PM
int sc8800g_enter_deepsleep(int inidle)
{
    int status;
    u32 t0, t1, delta;
    int ret = 0;
	int i;
/*
    int calibration_enable = 0;
*/
	unsigned long flags;

    REG_LOCAL_VALUE_DEF;

	if (!hw_irqs_disabled())  {
		flags = sc8800g_read_cpsr();
		printk("##: Error(%s): IRQ is enabled(%08lx)!\n", 
			inidle ? "idle" : "wakelock_suspend", flags);
	}

	/*	
	__raw_writel(IRQ_GPIO, INT_IRQ_DISABLE);
	__raw_writel(IRQ_GPIO, INT_FIQ_DISABLE);
	*/ 
    status = sc8800g_get_clock_status();
/*
    calibration_enable = in_calibration();
    if (sprd_sleep_mode_info) {
        if (calibration_enable) printk("##: In Calibration mode!\n");
    }
*/
/*
    t0 = get_sys_cnt();
    sc8800g_cpu_standby();
    t1 = get_sys_cnt();
    delta = t1 - t0;
    sleep_time += delta;
*/

#if 0
	val = __raw_readl(EMC_CFG0);
	if (!(val & RF_AUTO_SLEEP_ENABLE)) 
            printk("#####: EMC_CFG0 doesn't set RF_AUTO_SLEEP_ENABLE!\ns");

	for (i = 0; i < 6; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		if (!(val & RF_AUTO_SLEEP_ENABLE_CHX)) {
                      printk("######: channel[%d] dosen't set RF_AUTO_SLEEP_ENABLE_CHX!\n", i);
               }
	}

	for (i = 6; i < 9; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		if (val & RF_AUTO_SLEEP_ENABLE_CHX) {
                      printk("######: channel[%d] does set RF_AUTO_SLEEP_ENABLE_CHX!\n", i);
               }
	}


	val = __raw_readl(EMC_DCFG2);
	if (val & DRF_AUTO_SLEEP_MODE) printk("#####: EMC_DCFG2 does set DRF_AUTO_SLEEP_MODE!\n");
	if (!(val & DRF_REF_CNT_RST)) printk("####: EMC_DCFG2 doesn't set DRF_REF_CNT_RST!\n");
#endif
/*
    if (calibration_enable && (status & DEVICE_TEYP_MASK)) {
	 if (sprd_sleep_mode_info) {
	        printk("## sleep[Calibration].\n");
	 }
        return 0;
    }
*/
	sc8800g_save_pll();

    if (status & DEVICE_AHB)  {
		sleep_mode = SLEEP_MODE_ARM_CORE;
		if (sprd_sleep_mode_info) printk("## sleep[ARM_CORE].\n");
		if (sprd_wait_until_uart_tx_fifo_empty) wait_until_uart1_tx_done();

		t0 = get_sys_cnt();
		//sc8800g_cpu_standby();
		sp_arch_idle();

		t1 = get_sys_cnt();
		delta = t1 - t0;
		idle_time += delta;
		sc8800g_restore_pll();
    }
    else if (status & DEVICE_APB) {
        sleep_mode = SLEEP_MODE_MCU;
	if (sprd_sleep_mode_info) printk("## sleep[MCU].\n");
	if (sprd_wait_until_uart_tx_fifo_empty) wait_until_uart1_tx_done();

        SAVE_GLOBAL_REG;
        disable_audio_module();
        disable_ahb_module();
        //enable_mcu_sleep();
		t0 = get_sys_cnt();
		//ret =  sc8800g_cpu_standby_prefetch();
		sp_arch_idle();
        RESTORE_GLOBAL_REG;
        t1 = get_sys_cnt();
        delta = t1 - t0;
        sleep_time += delta;
        /*
        udelay(20);
        */
        sc8800g_restore_pll();
    }
    else {
		sleep_mode = SLEEP_MODE_DEEP;
		if (sprd_sleep_mode_info) printk("## sleep[DEEP] %d\n", inidle);
		if (sprd_wait_until_uart_tx_fifo_empty) wait_until_uart1_tx_done();

		sleep_counter++;

		supsend_ldo_turnoff();
		supsend_gpio_save();
		add_pm_message(get_sys_cnt(), "deepsleep_enter: inidle = ", inidle, 0, 0);
		/*
		printk("IRQ_STS = %08x, %s\n", 
		__raw_readl(INT_IRQ_STS), inidle ? "idle" : "pm_suspend");
		*/

		SAVE_GLOBAL_REG;
		disable_audio_module();
		disable_apb_module();
		disable_ahb_module();

		__raw_writel(INT_IRQ_MASK, INT_IRQ_DIS);//prevent uart1, time0 int while sleep

		/*
		gr_stc_state = __raw_readl(GR_STC_STATE);
		ahb_sts = __raw_readl(AHB_STS);
		gr_clk_dly = __raw_readl(GR_CLK_DLY);
		*/
	
		dma_sts = __raw_readl(DMA_TRANS_STS);
		irq_sts = __raw_readl(INT_IRQ_STS);
		fiq_sts = __raw_readl(INT_FIQ_STS);

		/* Following code may cause 	CPU to die. */
		/*
		irq_enable = __raw_readl(INT_IRQ_ENB);
		printk("irq_enable = %08x.", irq_enable);
		*/
		
		if (0 != irq_sts) 
			add_pm_message(get_sys_cnt(), "Going to deep sleep with pending irq: INT_IRQ_STS = ", 
					irq_sts, 0, 0);
		if (0 != fiq_sts) 
			add_pm_message(get_sys_cnt(), "Going to deep sleep with pending fiq: INT_FIQ_STS = ", 
					fiq_sts, 0, 0);
#if 0
        add_pm_message(get_sys_cnt(), "AHB_CTL0", __raw_readl(AHB_CTL0), 0, 0);
        add_pm_message(get_sys_cnt(), "AHB_CTL1", __raw_readl(AHB_CTL1), 0, 0);
        add_pm_message(get_sys_cnt(), "AHB_PAUSE", __raw_readl(AHB_PAUSE), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_GEN0", __raw_readl(GR_GEN0), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_PCTL", __raw_readl(GR_PCTL), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_BUSCLK", __raw_readl(GR_BUSCLK), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_POWCTL0", __raw_readl(GR_POWCTL0), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_POWCTL1", __raw_readl(GR_POWCTL1), 0, 0);
        add_pm_message(get_sys_cnt(), "GR_CLK_EN", __raw_readl(GR_CLK_EN), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_ANA_CTL0", ANA_REG_GET(ANA_ANA_CTL0), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_LDO_SLP", ANA_REG_GET(ANA_LDO_SLP), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_LDO_PD_SET", ANA_REG_GET(ANA_LDO_PD_SET), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_LDO_PD_CTL", ANA_REG_GET(ANA_LDO_PD_CTL), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_PLLWAIT", ANA_REG_GET(ANA_PLLWAIT), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_LDO_PD_RST", ANA_REG_GET(ANA_LDO_PD_RST), 0, 0);
        add_pm_message(get_sys_cnt(), "ANA_DCDC_CTRL_DS", ANA_REG_GET(ANA_DCDC_CTRL_DS), 0, 0);
#endif

		t0 = get_sys_cnt();
		//enable_mcu_sleep();
#ifdef CONFIG_CACHE_L2X0_310
//	__raw_writel(0xc5acce55, SPRD_A5_DEBUG_BASE+0x0fb0);//a5 debug lock access, cleared
//	__raw_writel(0, SPRD_A5_DEBUG_BASE+0x310);//a5 debug device power-down and reset control, not assert
	__raw_writel(1, SPRD_CACHE310_BASE+0xF80/*L2X0_POWER_CTRL*/);//l2cache power control, standby mode enable
#endif

		/* AHB_PAUSE */
		val = __raw_readl(AHB_PAUSE);
		val &= ~(MCU_CORE_SLEEP | MCU_DEEP_SLEEP_EN | APB_SLEEP);
		val |= (MCU_SYS_SLEEP_EN);
		
		/* enable MCU deep sleep, wangliwei, 2012-01-06. */
		val |= (MCU_DEEP_SLEEP_EN);//go deepsleep when all PD auto poweroff en
		__raw_writel(val, AHB_PAUSE);

		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			saved_vector[i] = sp_pm_reset_vector[i];
		}
//		saved_vector[0] = sp_pm_reset_vector[0];
//		saved_vector[1] = sp_pm_reset_vector[1];
		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = 0xe320f000; /* nop*/
		}
		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 2] = 0xE51FF004; /* ldr pc, 4 */

		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] = (sc8810_standby_exit_iram - 
			sc8810_standby_iram + IRAM_START_PHY);
		//sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] = virt_to_phys(sc8800g_cpu_standby_end);
	
//		sp_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
//		sp_pm_reset_vector[1] = virt_to_phys(sc8800g_cpu_standby_end);
//		trace();
		ret = sp_pm_collapse();
//		trace();
		wake_time = get_sys_cnt();

		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = saved_vector[i];
		}


//		sp_pm_reset_vector[0] = saved_vector[0];
//		sp_pm_reset_vector[1] = saved_vector[1];

		RESTORE_GLOBAL_REG;
		//printascii_phy("##: We are here!\n");
		if (ret) {
			cpu_init();
			//printascii_phy("@@@\n");
			// Remove following code when power domains don't auto power down.
			// wangliwei, 2010-01-06
//			sp_init_l3x0();
		}
		else {
		}
		//printk(KERN_INFO "@@@@@@@@wakeup(%d)\n", ret);
		outer_cache_poweron();
		//printk("##: 111.\n");
		//mdelay(100);
		//udelay(100);
		//printk("##: 222.\n");
		//mdelay(100);
		t1 = get_sys_cnt();
		delta = t1 - t0;
		if (!inidle) sleep_time += delta;
		else sleep_time_inidle += delta;

		udelay(20);
		//printk("##: 333.\n");
		//mdelay(100);

		supsend_ldo_turnon();
		supsend_gpio_restore();
		/*
		if (ret) {
			printk("##: bad return value.\n");
			printk("##: bad return value.\n");
			printk("##: bad return value.\n");
		}
		*/
			//printk("##: 444.\n");
			//mdelay(100);

		sc8800g_restore_pll();
		irq_sts = __raw_readl(INT_IRQ_STS);
		fiq_sts = __raw_readl(INT_FIQ_STS);
		parse_sprd_hard_irq(irq_sts);
		//printk("##: INT_IRQ_STS = %08x.\n", irq_sts);
		//printk("##: INT_FIQ_STS = %08x.\n", fiq_sts);
		//printk("##: 555.\n");
		//mdelay(100);
	
		if (0 == irq_sts) {
			add_pm_message(get_sys_cnt(), "deepsleep_exit: ### WITHOUT TRIGGER IRQ ###: ", 
					irq_sts, 0, 0);
		}
		else {
			add_pm_message(get_sys_cnt(), "deepsleep_exit: INT_IRQ_STS = ", 
					irq_sts, 0, 0);
		}
	
		if (0 != fiq_sts) {
			add_pm_message(get_sys_cnt(), "deepsleep_exit: INT_FIQ_STS = ", 
					fiq_sts, 0, 0);
		}
		wake_time = get_sys_cnt() - wake_time;
   }
    return ret;
}
EXPORT_SYMBOL(sc8800g_enter_deepsleep);
#endif

/*
Disable deep sleep, system hold a wakelock 
and nerver goes into deep sleep. 
*/
/*
#define CONFIG_SC8810_NO_DEEP_SLEEP 1
*/

/* Enable deep idle, in idle thrad, system may goes into deep sleep mode. */
/*
#define CONFIG_SC8810_IDLE_DEEP 1
*/

/* Close some clock forcely to do some experiments. */
/*
#define CONFIG_SC8810_DEEP_IDLE_TEST 1
*/

#if 0
int sprd_pm_suspend_check_enter(void);
int sprd_pm_resume_check(void);

int sprd_deep_idle_min_time = 2;

int sc8810_idle_sleep(int inidle)
{
    int status;
    u32 t0, t1, delta;
    int ret = 0;
	int i;
	unsigned long flags;
	u32 timer_expiration_ms = 0;
	u32 val = 0;

#ifdef CONFIG_SC8810_IDLE_DEEP
    status = sc8800g_get_clock_status();

	timer_expiration_ms = jiffies_to_msecs(get_next_timer_interrupt(jiffies) -jiffies);
	if ((timer_expiration_ms < sprd_deep_idle_min_time) || 
			has_wake_lock(WAKE_LOCK_IDLE)) {

		t0 = get_sys_cnt();
		sp_arch_idle();
		t1 = get_sys_cnt();
		delta = t1 - t0;
		arch_idle_total += delta;
	}
	else {

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
    	REG_LOCAL_VALUE_DEF;
		SAVE_GLOBAL_REG;
		disable_audio_module();
		disable_apb_module();
		disable_ahb_module();
#endif

#ifdef CONFIG_CACHE_L2X0_310
		__raw_writel(1, SPRD_CACHE310_BASE+0xF80/*L2X0_POWER_CTRL*/);//l2cache power control, standby mode enable
#endif
		/* AHB_PAUSE */
		val = __raw_readl(AHB_PAUSE);
		val &= ~(MCU_CORE_SLEEP | MCU_DEEP_SLEEP_EN | APB_SLEEP);
		val |= (MCU_SYS_SLEEP_EN);
		
		/* enable MCU deep sleep, wangliwei, 2012-01-06. */
		val |= (MCU_DEEP_SLEEP_EN);
		__raw_writel(val, AHB_PAUSE);

		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			saved_vector[i] = sp_pm_reset_vector[i];
		}
		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = 0xe320f000; /* nop*/
		}
		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 2] = 0xE51FF004; /* ldr pc, 4 */
		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] = (sc8810_standby_exit_iram - 
			sc8810_standby_iram + IRAM_START_PHY);
		t0 = get_sys_cnt();
		ret = sp_pm_collapse();
		t1 = get_sys_cnt();
		delta = t1 - t0;
		arch_idle_total += delta;

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
		RESTORE_GLOBAL_REG;
#endif
		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = saved_vector[i];
		}

		if (ret) {
			cpu_init();
		}
		else {
		}
		outer_cache_poweron();
	}
#else /* !CONFIG_SC8810_IDLE_DEEP */
#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	if (get_sys_cnt() > 150000) {
   		REG_LOCAL_VALUE_DEF;
		SAVE_GLOBAL_REG;
		/*
		disable_audio_module();
		disable_apb_module();
		disable_ahb_module();
		*/
    val = __raw_readl(AHB_CTL0);
    val &= ~(AHB_CTL0_SDIO1_EN);
    __raw_writel(val, AHB_CTL0);

	val = ANA_REG_GET(ANA_LDO_PD_CTL0);
	/*
	val &= ~(LDO_BPVB_CTL);
	val |= (LDO_BPVB_CTL >> 1);
	*/

	val &= ~(LDO_USB_CTL);
	val |= (LDO_USB_CTL >> 1);
	ANA_REG_SET(ANA_LDO_PD_CTL0, val);


	val = ANA_REG_GET(ANA_LDO_PD_CTL1);
	val &= ~(LDO_SDIO1_CTL);
	val |= (LDO_SDIO1_CTL >> 1);

	val &= ~(LDO_BPWIF0_CTL);
	val |= (LDO_BPWIF0_CTL >> 1);

	val &= ~(LDO_BPWIF1_CTL);
	val |= (LDO_BPWIF1_CTL >> 1);

	ANA_REG_SET(ANA_LDO_PD_CTL1, val);

#endif

#ifdef CONFIG_CACHE_L2X0_310
	__raw_writel(1, SPRD_CACHE310_BASE+0xF80/*L2X0_POWER_CTRL*/);//l2cache power control, standby mode enable
#endif
	outer_cache_poweroff();
	t0 = get_sys_cnt();
	sp_arch_idle();
	t1 = get_sys_cnt();
	delta = t1 - t0;
	arch_idle_total += delta;

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	RESTORE_GLOBAL_REG;
#endif

	outer_cache_poweron();
#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	}
#endif

#endif
    return ret;
}
EXPORT_SYMBOL(sc8810_idle_sleep);
#endif

int sc8810_idle_sleep(int inidle)
{
    int status;
    u32 t0, t1, delta;
    int ret = 0;
	int i;
	unsigned long flags;
	u32 timer_expiration_ms = 0;
	u32 val = 0;

#ifdef CONFIG_SC8810_IDLE_DEEP
    status = sc8800g_get_clock_status();

	timer_expiration_ms = jiffies_to_msecs(get_next_timer_interrupt(jiffies) -jiffies);
	if ((timer_expiration_ms < sprd_deep_idle_min_time) || 
			has_wake_lock(WAKE_LOCK_IDLE)) {

		t0 = get_sys_cnt();
		sp_arch_idle();
		t1 = get_sys_cnt();
		delta = t1 - t0;
		arch_idle_total += delta;
	}
	else {

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
    	REG_LOCAL_VALUE_DEF;
		SAVE_GLOBAL_REG;
		disable_audio_module();
		disable_apb_module();
		disable_ahb_module();
#endif

#ifdef CONFIG_CACHE_L2X0_310
		__raw_writel(1, SPRD_CACHE310_BASE+0xF80/*L2X0_POWER_CTRL*/);//l2cache power control, standby mode enable
#endif
		/* AHB_PAUSE */
		val = __raw_readl(AHB_PAUSE);
		val &= ~(MCU_CORE_SLEEP | MCU_DEEP_SLEEP_EN | APB_SLEEP);
		val |= (MCU_SYS_SLEEP_EN);
		
		/* enable MCU deep sleep, wangliwei, 2012-01-06. */
		val |= (MCU_DEEP_SLEEP_EN);
		__raw_writel(val, AHB_PAUSE);

		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			saved_vector[i] = sp_pm_reset_vector[i];
		}
		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = 0xe320f000; /* nop*/
		}
		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 2] = 0xE51FF004; /* ldr pc, 4 */
		sp_pm_reset_vector[SAVED_VECTOR_SIZE - 1] = (sc8810_standby_exit_iram - 
			sc8810_standby_iram + IRAM_START_PHY);
		t0 = get_sys_cnt();
		ret = sp_pm_collapse();
		t1 = get_sys_cnt();
		delta = t1 - t0;
		arch_idle_total += delta;

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
		RESTORE_GLOBAL_REG;
#endif
		for (i = 0; i < SAVED_VECTOR_SIZE; i++) {
			sp_pm_reset_vector[i] = saved_vector[i];
		}

		if (ret) {
			cpu_init();
		}
		else {
		}
		outer_cache_poweron();
	}
#else /* !CONFIG_SC8810_IDLE_DEEP */
#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	if (get_sys_cnt() > 150000) {
   		REG_LOCAL_VALUE_DEF;
		SAVE_GLOBAL_REG;
		/*
		disable_audio_module();
		disable_apb_module();
		disable_ahb_module();
		*/
    val = __raw_readl(AHB_CTL0);
    val &= ~(AHB_CTL0_SDIO1_EN);
    __raw_writel(val, AHB_CTL0);

	val = ANA_REG_GET(ANA_LDO_PD_CTL0);
	/*
	val &= ~(LDO_BPVB_CTL);
	val |= (LDO_BPVB_CTL >> 1);
	*/

	val &= ~(LDO_USB_CTL);
	val |= (LDO_USB_CTL >> 1);
	ANA_REG_SET(ANA_LDO_PD_CTL0, val);


	val = ANA_REG_GET(ANA_LDO_PD_CTL1);
	val &= ~(LDO_SDIO1_CTL);
	val |= (LDO_SDIO1_CTL >> 1);

	val &= ~(LDO_BPWIF0_CTL);
	val |= (LDO_BPWIF0_CTL >> 1);

	val &= ~(LDO_BPWIF1_CTL);
	val |= (LDO_BPWIF1_CTL >> 1);

	ANA_REG_SET(ANA_LDO_PD_CTL1, val);

#endif

#ifdef CONFIG_CACHE_L2X0_310
	__raw_writel(1, SPRD_CACHE310_BASE+0xF80/*L2X0_POWER_CTRL*/);//l2cache power control, standby mode enable
#endif
	outer_cache_poweroff();
	t0 = get_sys_cnt();
	sp_arch_idle();
	t1 = get_sys_cnt();
	delta = t1 - t0;
	arch_idle_total += delta;

#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	RESTORE_GLOBAL_REG;
#endif

	outer_cache_poweron();
#ifdef CONFIG_SC8810_DEEP_IDLE_TEST
	}
#endif

#endif
    return ret;
}
EXPORT_SYMBOL(sc8810_idle_sleep);


#ifdef CONFIG_NKERNEL
static void nkidle(void)
{
	int val;
	u32 t0, t1, delta;
	if (!need_resched()) {
		hw_local_irq_disable();
		if (!raw_local_irq_pending()) {
			val = os_ctx->idle(os_ctx);
			if (0 == val) {
#ifdef CONFIG_PM
				sc8810_idle_sleep(1);
#endif
			}
		}

		hw_local_irq_enable();
	}
	local_irq_enable();
}

void nkidle_original(void)
{
	int val;
	u32 t0, t1, delta;

	if (!need_resched()) {
		hw_local_irq_disable();
		if (!raw_local_irq_pending()) {
			val = os_ctx->idle(os_ctx);
			if (0 == val) {
				    idle_loops++;
				    t0 = get_sys_cnt();
				    sp_arch_idle();
				    t1 = get_sys_cnt();
				    delta = t1 - t0;
				    idle_time += delta;
			}
		}
		hw_local_irq_enable();
	}

	local_irq_enable();
	add_pm_message(get_sys_cnt(), "nkidle: 1111--leave.", 0, 0, 0);
}

#endif

#define DEEP_SLEEP_INTERVAL (HZ)

static void deep_sleep_timeout(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer, deep_sleep_timeout, 0, 0);

static int sleep_mode_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sleep_mode_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sleep_mode_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sleep_mode_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sleep_mode_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{

	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_sleep_mode_info = 0;
		break;
	case '1':
		sprd_sleep_mode_info = 1;
		/* to display message completely. */
		sprd_wait_until_uart_tx_fifo_empty = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _sleep_mode_info_proc_fops = {
    open:    sleep_mode_info_open,
    release: sleep_mode_info_release,
    llseek:  sleep_mode_info_lseek,
    read:    sleep_mode_info_read,
    write:   sleep_mode_info_write,
};






static int wait_uart_done_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int wait_uart_done_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t wait_uart_done_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t wait_uart_done_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t wait_uart_done_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{

	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_wait_until_uart_tx_fifo_empty = 0;
		break;
	case '1':
		sprd_wait_until_uart_tx_fifo_empty = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _wait_uart_done_proc_fops = {
    open:    wait_uart_done_open,
    release: wait_uart_done_release,
    llseek:  wait_uart_done_lseek,
    read:    wait_uart_done_read,
    write:   wait_uart_done_write,
};




static int sprd_dump_gpio_registers_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_dump_gpio_registers_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_dump_gpio_registers_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_dump_gpio_registers_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_dump_gpio_registers_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{

	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_dump_gpio_registers = 0;
		break;
	case '1':
		sprd_dump_gpio_registers = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _dump_gpio_registers_proc_fops = {
    open:    sprd_dump_gpio_registers_open,
    release: sprd_dump_gpio_registers_release,
    llseek:  sprd_dump_gpio_registers_lseek,
    read:    sprd_dump_gpio_registers_read,
    write:   sprd_dump_gpio_registers_write,
};



static int sprd_gpio_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_gpio_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_gpio_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_gpio_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_gpio_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{

	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_check_gpio_enable = 0;
		break;
	case '1':
		sprd_check_gpio_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _gpio_info_proc_fops = {
    open:    sprd_gpio_info_open,
    release: sprd_gpio_info_release,
    llseek:  sprd_gpio_info_lseek,
    read:    sprd_gpio_info_read,
    write:   sprd_gpio_info_write,
};


static int sprd_irq_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_irq_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_irq_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_irq_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_irq_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{

	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_irq_info_enable = 0;
		break;
	case '1':
		sprd_irq_info_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _irq_info_proc_fops = {
    open:    sprd_irq_info_open,
    release: sprd_irq_info_release,
    llseek:  sprd_irq_info_lseek,
    read:    sprd_irq_info_read,
    write:   sprd_irq_info_write,
};

static int sprd_thread_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_thread_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_thread_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_thread_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_thread_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_thread_info_enable = 0;
		break;
	case '1':
		sprd_thread_info_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _thread_info_proc_fops = {
    open:    sprd_thread_info_open,
    release: sprd_thread_info_release,
    llseek:  sprd_thread_info_lseek,
    read:    sprd_thread_info_read,
    write:   sprd_thread_info_write,
};


static int sprd_statistic_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_statistic_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_statistic_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_statistic_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_statistic_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_statistic_info_enable = 0;
		break;
	case '1':
		sprd_statistic_info_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _statistic_info_proc_fops = {
    open:    sprd_statistic_info_open,
    release: sprd_statistic_info_release,
    llseek:  sprd_statistic_info_lseek,
    read:    sprd_statistic_info_read,
    write:   sprd_statistic_info_write,
};


static int sprd_timer_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_timer_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_timer_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_timer_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_timer_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_timer_info_enable = 0;
		break;
	case '1':
		sprd_timer_info_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _timer_info_proc_fops = {
    open:    sprd_timer_info_open,
    release: sprd_timer_info_release,
    llseek:  sprd_timer_info_lseek,
    read:    sprd_timer_info_read,
    write:   sprd_timer_info_write,
};


static int sprd_dsp_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_dsp_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_dsp_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_dsp_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_dsp_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_check_dsp_enable = 0;
		break;
	case '1':
		sprd_check_dsp_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _dsp_info_proc_fops = {
    open:    sprd_dsp_info_open,
    release: sprd_dsp_info_release,
    llseek:  sprd_dsp_info_lseek,
    read:    sprd_dsp_info_read,
    write:   sprd_dsp_info_write,
};

static int sprd_clock_info_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_clock_info_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_clock_info_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_clock_info_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_clock_info_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_clock_info_enable = 0;
		break;
	case '1':
		sprd_clock_info_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _clock_info_proc_fops = {
    open:    sprd_clock_info_open,
    release: sprd_clock_info_release,
    llseek:  sprd_clock_info_lseek,
    read:    sprd_clock_info_read,
    write:   sprd_clock_info_write,
};




static int sprd_suspend_enable_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_suspend_enable_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_suspend_enable_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_suspend_enable_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_suspend_enable_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_suspend_enable = 0;
		break;
	case '1':
		sprd_suspend_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _suspend_enable_proc_fops = {
    open:    sprd_suspend_enable_open,
    release: sprd_suspend_enable_release,
    llseek:  sprd_suspend_enable_lseek,
    read:    sprd_suspend_enable_read,
    write:   sprd_suspend_enable_write,
};


static int sprd_sc8810_deepsleep_enable_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_sc8810_deepsleep_enable_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_sc8810_deepsleep_enable_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_sc8810_deepsleep_enable_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_sc8810_deepsleep_enable_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_sc8810_deepsleep_enable = 0;
		break;
	case '1':
		sprd_sc8810_deepsleep_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _suspend_sc8810_deepsleep_proc_fops = {
    open:    sprd_sc8810_deepsleep_enable_open,
    release: sprd_sc8810_deepsleep_enable_release,
    llseek:  sprd_sc8810_deepsleep_enable_lseek,
    read:    sprd_sc8810_deepsleep_enable_read,
    write:   sprd_sc8810_deepsleep_enable_write,
};



static int sprd_suspend_interval_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_suspend_interval_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_suspend_interval_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_suspend_interval_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_suspend_interval_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[12];
	char *end;
	unsigned long val = 0xff;
	
	if (size > 11) {
		printk("##: Wrong Value!\n");
		return -EINVAL;
	}
	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	ctl[size] = '\0';
	val = simple_strtoul(ctl, &end, 10);
	if (end == ctl) {
		printk("##: Conversion Failed!\n");
		size = -EINVAL;
	}
	else {
		sprd_suspend_interval = val;
		printk("##: sprd_suspend_interval = %d.\n", sprd_suspend_interval);
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _suspend_interval_proc_fops = {
    open:    sprd_suspend_interval_open,
    release: sprd_suspend_interval_release,
    llseek:  sprd_suspend_interval_lseek,
    read:    sprd_suspend_interval_read,
    write:   sprd_suspend_interval_write,
};

static int sprd_sc8810_no_deep_sleep_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_sc8810_no_deep_sleep_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_sc8810_no_deep_sleep_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_sc8810_no_deep_sleep_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_sc8810_no_deep_sleep_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_no_deep_sleep_enable = 0;
		break;
	case '1':
		sprd_no_deep_sleep_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _no_deep_sleep_fops = {
    open:    sprd_sc8810_no_deep_sleep_open,
    release: sprd_sc8810_no_deep_sleep_release,
    llseek:  sprd_sc8810_no_deep_sleep_lseek,
    read:    sprd_sc8810_no_deep_sleep_read,
    write:   sprd_sc8810_no_deep_sleep_write,
};


static int sprd_sc8810_idle_test_fops_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_sc8810_idle_test_fops_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_sc8810_idle_test_fops_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_sc8810_idle_test_fops_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_sc8810_idle_test_fops_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_idle_test_enable = 0;
		break;
	case '1':
		sprd_idle_test_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _idle_test_fops = {
    open:    sprd_sc8810_idle_test_fops_open,
    release: sprd_sc8810_idle_test_fops_release,
    llseek:  sprd_sc8810_idle_test_fops_lseek,
    read:    sprd_sc8810_idle_test_fops_read,
    write:   sprd_sc8810_idle_test_fops_write,
};



static int sprd_sc8810_idle_deep_open (struct inode* inode, struct file*  file)
{
    return 0;
}

static int sprd_sc8810_idle_deep_release (struct inode* inode, struct file*  file)
{
    return 0;
}
static loff_t sprd_sc8810_idle_deep_lseek (struct file* file, loff_t off, int whence)
{
	return 0;
}
static ssize_t sprd_sc8810_idle_deep_read (struct file* file, char* buf, size_t count, loff_t* ppos)
{
    return 0;
}

static ssize_t sprd_sc8810_idle_deep_write (struct file* file, const char* ubuf, size_t size, loff_t* ppos)
{
	char ctl[2];

	if (size != 2 || *ppos)
		return -EINVAL;

	if (copy_from_user(ctl, ubuf, size))
		return -EFAULT;

	mutex_lock(&sprd_proc_info_mutex);
	switch (ctl[0]) {
	case '0':
		sprd_idle_deep_enable = 0;
		break;
	case '1':
		sprd_idle_deep_enable = 1;
		break;
	default:
		size = -EINVAL;
	}
	mutex_unlock(&sprd_proc_info_mutex);

	return size;
}

static struct file_operations _idle_deep_fops = {
    open:    sprd_sc8810_idle_deep_open,
    release: sprd_sc8810_idle_deep_release,
    llseek:  sprd_sc8810_idle_deep_lseek,
    read:    sprd_sc8810_idle_deep_read,
    write:   sprd_sc8810_idle_deep_write,
};


static void deep_sleep_timeout(unsigned long data)
{
	printk("###: deep_sleep_timeout()!\n");
	queue_work(deep_sleep_work_queue, &deep_sleep_wrok);
	mod_timer(&deep_sleep_timer, jiffies + DEEP_SLEEP_INTERVAL);
}

static  struct proc_dir_entry*
sprd_proc_create (struct proc_dir_entry*  parent,
                 const char*             name,
                 struct file_operations* fops)
{
    struct proc_dir_entry* file;
    file = create_proc_entry(name, (S_IFREG|S_IRUGO|S_IWUSR), parent);
    if (!file) {
        printk("##: Error: create_proc_entry(%s) failed\n", name);
        return NULL;
    }

    file->proc_fops = fops;

    return file;
}

#ifdef CONFIG_PM
int sc8800g_prepare_deep_sleep(void)
{
    u32 val = 0;
    int i;
    pid_t pid_number;
    u32 * pint;

	if (!sp_pm_reset_vector) {
		sp_pm_reset_vector = ioremap(0xffff0000, PAGE_SIZE);
		if (sp_pm_reset_vector == NULL) {
			printk(KERN_ERR "sp_pm_init: failed to map reset vector\n");
			return 0;
		}
	}



	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));
	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));
	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));
	/*
	sc8800g_pll_default();
	*/
	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));
	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));
	printk("##: GR_MPLL_MN = %08x.\n", __raw_readl(GR_MPLL_MN));



#if 0
	val = __raw_readl(EMC_CFG0);
	val |= RF_AUTO_SLEEP_ENABLE;
	__raw_writel(val, EMC_CFG0);

	for (i = 0; i < 6; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		val |= RF_AUTO_SLEEP_ENABLE_CHX;
		__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}

	for (i = 6; i < 9; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		val &= ~RF_AUTO_SLEEP_ENABLE_CHX;
		__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}


	val = __raw_readl(EMC_DCFG2);
	val &= ~DRF_AUTO_SLEEP_MODE;
	val |= DRF_REF_CNT_RST;
	__raw_writel(val, EMC_DCFG2);
#endif

	printk("####: check EMC channel's setting.\n");
	printk("####: check EMC channel's setting.\n");
	printk("####: check EMC channel's setting.\n");
	printk("####: check EMC channel's setting.\n");
	val = __raw_readl(EMC_CFG0);
	if (!(val & RF_AUTO_SLEEP_ENABLE)) 
            printk("#####: EMC_CFG0 doesn't set RF_AUTO_SLEEP_ENABLE!\ns");

	for (i = 0; i < 6; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		if (!(val & RF_AUTO_SLEEP_ENABLE_CHX)) {
                      printk("######: channel[%d] dosen't set RF_AUTO_SLEEP_ENABLE_CHX!\n", i);
               }
	}

	for (i = 6; i < 9; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		if (val & RF_AUTO_SLEEP_ENABLE_CHX) {
                      printk("######: channel[%d] does set RF_AUTO_SLEEP_ENABLE_CHX!\n", i);
               }
	}
	val = __raw_readl(EMC_DCFG2);
	if (val & DRF_AUTO_SLEEP_MODE) printk("#####: EMC_DCFG2 does set DRF_AUTO_SLEEP_MODE!\n");
	if (!(val & DRF_REF_CNT_RST)) printk("####: EMC_DCFG2 doesn't set DRF_REF_CNT_RST!\n");


    /* AHB_PAUSE */
    val = __raw_readl(AHB_PAUSE);
    val &= ~(MCU_CORE_SLEEP | MCU_DEEP_SLEEP_EN | APB_SLEEP);
    val |= (MCU_SYS_SLEEP_EN);
    __raw_writel(val, AHB_PAUSE);


    /* GR_PCTL */
    val = __raw_readl(GR_PCTL);
    val |= (MCU_MPLL_EN);
    __raw_writel(val, GR_PCTL);

	/* AHB_CTL0 */
	val = __raw_readl(AHB_CTL0);
	val &= ~AHB_CTL0_ROT_EN;
	__raw_writel(val, AHB_CTL0);

    /* AHB_CTL1 */
	val = __raw_readl(AHB_CTL1);
	val |= (AHB_CTRL1_EMC_CH_AUTO_GATE_EN | AHB_CTRL1_EMC_AUTO_GATE_EN | 
		AHB_CTRL1_ARM_AUTO_GATE_EN|
		AHB_CTRL1_AHB_AUTO_GATE_EN|
//		AHB_CTRL1_MCU_AUTO_GATE_EN|	//BUGBUG
		AHB_CTRL1_ARM_DAHB_SLEEP_EN|
	        AHB_CTRL1_ARMMTX_AUTO_GATE_EN | AHB_CTRL1_MSTMTX_AUTO_GATE_EN);
	val &= ~AHB_CTRL1_MCU_AUTO_GATE_EN;
	__raw_writel(val, AHB_CTL1);


    /* GR_CLK_EN */
    /* enable XTL auto power down. */
    val = __raw_readl(GR_CLK_EN);
    val |= MCU_XTLEN_AUTOPD_EN;
	/*
    val &= ~BIT_17;		//must do, I don't know reason.
	*/
   __raw_writel(val, GR_CLK_EN);

	/* Power domain auto power-down, wangliwei, 2012-01-06. */
	if (sprd_sc8810_deepsleep_enable) {
		sc8810_setup_pd_automode();
	}
	sc8810_setup_ldo_slpmode();
#if 0
	printk("####### checking some registers. ##########\n");
	printk("####### checking some registers. ##########\n");
	printk("####### checking some registers. ##########\n");
	val = ANA_REG_GET(ANA_DCDC_CTRL_DS);
	printk("## before ##: ANA_DCDC_CTRL_DS = %08x.\n", val);
	val &= 0xffff00ff;
	val |= (0x0f << 8);
	ANA_REG_SET(ANA_DCDC_CTRL_DS, val);
	val = ANA_REG_GET(ANA_DCDC_CTRL_DS);
	printk("## after ##: ANA_DCDC_CTRL_DS = %08x.\n", val);

	val = __raw_readl(GR_GEN4);
	printk("## before ##: GR_GEN4 = %08x.\n", val);
	val |= (BIT_31 | BIT_30 | BIT_29);
	__raw_writel(val, GR_GEN4);
	val = __raw_readl(GR_GEN4);
	printk("## after ##: GR_GEN4 = %08x.\n", val);
#endif

    //ANA_REG_OR(ANA_LDO_SLP, (FSM_RF0_BP_EN | FSM_RF1_BP_EN));
    //ANA_REG_SET(ANA_LDO_SLP, 0xa4f3);


    /* ANA_ANA_CTL0 */
    /* vibrate power down. */
    /* 
    ANA_REG_OR(ANA_ANA_CTL0, VIBR_PD);
    */
    /* enable LDOs auto power down. */
    //ANA_REG_OR(ANA_ANA_CTL0, FSM_AFCPD_EN);

    /* enable OTP. */
    /*
    ANA_REG_BIC(ANA_ANA_CTL0, OTP_ENABLE);
    */

    /* AHB_CTL1 */
    val = POWCTL1_CONFIG;
    //__raw_writel(val, GR_POWCTL1);


    printk("####: ioremap space for share IRAM ......\n");
    iram_start = ioremap(IRAM_START_PHY,  IRAM_SIZE);
    if (!iram_start) {
        printk("####: Can't ioremap for IRAM!\n");
        return -ENOMEM;
    }
    else {
        printk("###: iram_start = %p\n", iram_start);
    }

    /* copy sleep code to IRAM. */
    printk("###: sc8800g_cpu_standby = %p, sc8800g_cpu_standby_end = %p\n", 
        sc8810_standby_iram, sc8810_standby_iram_end);
    if ((sc8810_standby_iram_end - sc8810_standby_iram + 128) > SLEEP_CODE_SIZE) {
          panic("##: code size is larger than expected, need more memory!\n");
    }

    memcpy_toio(iram_start, sc8810_standby_iram, SLEEP_CODE_SIZE);

    /* we will get these code free different virtual address, so flush it 
        to make it visible, mayde not necessary.
    */
 	flush_cache_all();
	/*
	map_iram_identically();
	*/
	pint = (u32 *)iram_start;

	pid_number = kernel_thread(print_thread, NULL, 0);
	if (pid_number < 0) {
		printk("Can't crate test thread!\n");
	}

	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock");
	wake_lock_init(&idle_wakelock, WAKE_LOCK_SUSPEND,
			"idle_wakelock");
#ifdef CONFIG_SC8810_NO_DEEP_SLEEP
	wake_lock(&idle_wakelock);
#endif
	init_pm_message();

#if defined(CONFIG_NKERNEL)
	pm_idle = nkidle;
#endif
	/*
	mod_timer(&deep_sleep_timer, jiffies + DEEP_SLEEP_INTERVAL);
	*/
	deep_sleep_work_queue = create_singlethread_workqueue("deep_sleep");
	if (deep_sleep_work_queue == NULL) {
		printk("##: Can't create workqueue!\n");
	}
	/*
	queue_work(deep_sleep_work_queue, &deep_sleep_wrok);
	*/

    sprd_proc_entry = proc_mkdir("sprd_sleep_info", NULL);
    if (!sprd_proc_entry) {
        printk("##: proc_mkdir(/proc/sprd_sleep_info) failed\n");
        return 0;
    }

	sprd_proc_create(sprd_proc_entry, "sleep_mode_info", &_sleep_mode_info_proc_fops);

	sprd_proc_create(sprd_proc_entry, "wait_uart_done", &_wait_uart_done_proc_fops);
	sprd_proc_create(sprd_proc_entry, "dump_gpio_registers", &_dump_gpio_registers_proc_fops);
	sprd_proc_create(sprd_proc_entry, "gpio_info", &_gpio_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "irq_info", &_irq_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "thread_info", &_thread_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "statistic_info", &_statistic_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "timer_info", &_timer_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "check_dsp_status", &_dsp_info_proc_fops);
	sprd_proc_create(sprd_proc_entry, "clock_info", &_clock_info_proc_fops);
	/*
    sprd_proc_create(sprd_proc_entry, "pm_message", &_thread_info_proc_fops);
	*/
	sprd_proc_create(sprd_proc_entry, "sprd_suspend_enable", &_suspend_enable_proc_fops);
	sprd_proc_create(sprd_proc_entry, "sprd_suspend_interval", &_suspend_interval_proc_fops);
	sprd_proc_create(sprd_proc_entry, "sprd_sc8810_deepsleep_enable", &_suspend_sc8810_deepsleep_proc_fops);

	sprd_proc_create(sprd_proc_entry, "sprd_no_deep_sleep", &_no_deep_sleep_fops);
	sprd_proc_create(sprd_proc_entry, "sprd_idle_test", &_idle_test_fops);
	sprd_proc_create(sprd_proc_entry, "sprd_idle_deep", &_idle_deep_fops);
    return 0;
}
EXPORT_SYMBOL(sc8800g_prepare_deep_sleep);
#endif
