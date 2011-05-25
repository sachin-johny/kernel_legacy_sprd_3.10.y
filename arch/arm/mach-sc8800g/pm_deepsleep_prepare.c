/*
  *     Deep sleep testing mode.
  *
  *     Download small piece of code into DSP and make it sleep for ever.
  *
  */


/*
modules:

pin map,


wifi,  UNIFIxxxx
bt(gpio_42, gpio_90),  BCxxxx
gps(allen) , /system/lib/libgsd4t.so, on/off <--> uart2.rxd,


fm(aijun, 2011-05-24),


ldo control,
atv(remove, hw),
lcdc(ok),
g-sensor(ok),
m-sensor(ok),
ctp -- mtp(ok),
proximity(ok),
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



extern long has_wake_lock_info(int type);
extern long has_wake_lock_for_suspend(int type);


/*Analog Die interrupt register*/
#define ANA_INT_STATUS             (SPRD_MISC_BASE +0x380+ 0x00)
#define ANA_INT_RAW                  (SPRD_MISC_BASE + 0x380 + 0x04)
#define ANA_INT_EN                	(SPRD_MISC_BASE + 0x380 + 0x08)
#define ANA_INT_STATUS_SYNC      (SPRD_MISC_BASE + 0x380 + 0x0C)



#include "dsp_data_no_wakeup_share_memory_0x0EC0_0000.h"

/* bit definitions. */
#define RUN_TEST         0
#define STOP_TEST        1
#define SYS_SLEEP_TEST   2
#define FORCE_SLEEP_TEST 3
#define PERIOD_TEST      4
#define GSM_PWR_DOMAIN_CLOSE (0x1UL << 16)
#define TD_PWR_DOMAIN_CLOSE  (0x1UL << 17)
#define CEVA_MEM_CLOSE  (0x1UL << 17)

#define 	SHARE_MEMORY_PHY_ADDR 	0x50000000
#define 	SHARE_MEMORY_SIZE 		0x00000100

#define 	DSP_MEMORY_PHY_ADDR 	0x0ec00000
#define 	DSP_MEMORY_SIZE 		0x00400000

#define IRAM_BASE_PHY   0x40000000
#define IRAM_START_PHY 0x40004000
#define IRAM_SIZE 0x4000

#define SLEEP_CODE_SIZE 4096
static u32 pll_n = 200;
#define SC8800G2_DEFAULT_PLL_N 200
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
struct timespec now_ts_pm;


/*
#define SPRD_PM_MESSAGE 1
*/
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
                printk("##: [%d] [%d] %s: %ld, %ld, %ld, %Lu. \n", 
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


static DEFINE_SPINLOCK(deepsleep_lock);




void sc8800g2_delay(void);
/*********************************/
/*********************************/



u32 reg_gen0_val, reg_busclk_alm, reg_ahb_ctl0_val, reg_gen_clk_en, reg_gen_clk_gen5, reg_ahb_ctl3;


#define GEN0_MASK ( GEN0_SIM0_EN | GEN0_I2C_EN | GEN0_GPIO_EN | \
	                GEN0_EFUSE_EN | GEN0_I2S_EN | GEN0_PIN_EN | \
	                GEN0_EPT_EN | GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | \
	                GEN0_UART1_EN | GEN0_UART2_EN)

#define CLK_EN_MASK (CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN)
#define BUSCLK_ALM_MASK (ARM_VB_MCLKON|ARM_VB_DA0ON|ARM_VB_DA1ON|ARM_VB_ADCON|ARM_VB_ANAON|ARM_VB_ACC)
#define AHB_CTL0_MASK   (AHB_CTL0_DCAM_EN|AHB_CTL0_CCIR_EN|AHB_CTL0_LCDC_EN|    \
                         AHB_CTL0_SDIO_EN|AHB_CTL0_DMA_EN|     \
                         AHB_CTL0_BM0_EN |AHB_CTL0_NFC_EN|AHB_CTL0_BM1_EN|       \
                         AHB_CTL0_VSP_EN|AHB_CTL0_ROT_EN)

#define GR_CLK_EN_MASK CLK_EN_MASK
#define GR_GEN0_MASK GEN0_MASK

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

extern void sc8800g_cpu_standby(void);
extern void sc8800g_cpu_standby_end(void);
extern int sc8800g_cpu_standby_prefetch(void);


#define UART_STS0 (SPRD_SERIAL1_BASE + 0x08)
#define UART_STS1 (SPRD_SERIAL1_BASE + 0x0c)

#define UART_TRANSFER_REALLY_OVER (0x1UL << 15)
static void wait_until_uart1_tx_done(void)
{
    u32 tx_fifo_val;
    u32 really_over = 0;

	/* fifo depth = 128. */
	u32 timeout = 150;	
	    tx_fifo_val = __raw_readl(UART_STS1);
	    tx_fifo_val >>= 8;
	    tx_fifo_val &= 0xff;
	while(tx_fifo_val != 0) {
		WARN_ON(0 >= timeout);
		if (timeout <= 0) break;
		udelay(90);
	    tx_fifo_val = __raw_readl(UART_STS1);
	    tx_fifo_val >>= 8;
	    tx_fifo_val &= 0xff;
		timeout--;
	}
	udelay(90);
/*
    do {
        really_over = __raw_readl(UART_STS0);
    }while (!(UART_TRANSFER_REALLY_OVER * really_over));
*/
out:
	return;
}

static int verify_dsp_deep_sleep(void)
{
	u32 val;
	int ret_val = 0;

	printk("####: check register: GR_STC_STATE for DSP\n");
	val = __raw_readl(GR_STC_STATE);
	printk("######: GR_STC_STATE =%08x\n", val);
	if (GR_DSP_STOP & val) {
		printk("#####: GR_STC_STATE[DSP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_STC_STATE[DSP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	printk("####: check register: GR_CLK_DLY for DSP\n");
	val = __raw_readl(GR_CLK_DLY);
	printk("######: GR_CLK_DLY =%08x\n", val);

	if (DSP_DEEP_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_SYS_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_AHB_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (DSP_MTX_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_CORE_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (GR_EMC_STOP_CH5 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH4 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH3 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is NOT set!\n");
		ret_val = -EINVAL;
	}

	return ret_val;
}


static int verify_ahb_sts(void)
{
	u32 val;
	int ret_val = 0;

	printk("####: check register: AHB_STS for ARM\n");
	val = __raw_readl(AHB_STS);
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




int force_dsp_sleep(void)
{
	u32 i; 
	u32 temp;
	int times_try = 10;

	void __iomem *share_memory_base;
	void __iomem *dsp_memory_base;

	u32 val;

	verify_dsp_deep_sleep();

	printk("####: ioremap space for share memroy & dsp memory......\n");
	share_memory_base = ioremap(SHARE_MEMORY_PHY_ADDR,  SHARE_MEMORY_SIZE);
	if (!share_memory_base) {
		printk("####: Can't ioremap for share memroy!\n");
		return -ENOMEM;
	}


	dsp_memory_base = ioremap(DSP_MEMORY_PHY_ADDR,  DSP_MEMORY_SIZE);
	if (!dsp_memory_base) {
		printk("####: Can't ioremap for dsp memroy!\n");
		return -ENOMEM;
	}
	
	printk("#####: Let DSP enter deep sleep.....\n");

	/*    switch the ownership of UART1. */
	// *(volatile uint32 *)(GR_PCTL) |= BIT_8; 
	
    printk("For chip [SC8800G2]......\r\n");
	val = (SYS_SLEEP_TEST | TD_PWR_DOMAIN_CLOSE | TD_PWR_DOMAIN_CLOSE);
	__raw_writel(val, share_memory_base);
 
	printk("####: Downloading DSP firmware.......\r\n");

	for(i = 0; i < td_dsp_total_len; i+=4)
	{
    
    	temp = 0;
    	temp |= td_dsp_all_vec[i + 3];
    	temp |= td_dsp_all_vec[i + 2] << 8;
    	temp |= td_dsp_all_vec[i + 1] << 16;
    	temp |= td_dsp_all_vec[i + 0] << 24;
    	
    	__raw_writel(temp, dsp_memory_base + i);
    }
	
	for (i = 0; i < 32; i++) {
		printk("data[%d] = %02x\n", i, ((u8 *)dsp_memory_base)[i]);
	}
     /* reboot DSP */

	val = __raw_readl(DSP_BOOT_EN);
	val &= ~DSP_BOOT_ENABLE;
	__raw_writel(val, DSP_BOOT_EN);

	val = (DSP_MEMORY_PHY_ADDR & 0x0fffffff) | 0xc0000000;
	__raw_writel(val, DSP_BOOT_VEC);


	val = __raw_readl(DSP_BOOT_EN);
	val |= DSP_BOOT_ENABLE;
	__raw_writel(val, DSP_BOOT_EN);

	val = __raw_readl(DSP_RST);
	val |= DSP_RESET;
	__raw_writel(val, DSP_RST);

	while (verify_dsp_deep_sleep()) {
		if (!times_try) break;
		printk("#####: waiting DSP to enter deep sleep mode.....\n");
		mdelay(300);
	}
	if (!times_try) {
		printk("#####: timeout when waiting DSP to enter deep sleep mode!\n");
		return -EINVAL;
	}
	else {
		printk("#######: DSP is in deep sleep mode Now!\n");
	}
	return 0;
}

EXPORT_SYMBOL(force_dsp_sleep);
/*
 * configure pins for deep sleep, for example,
 * input & output enable in sleep mode.
 *
 */

static void pins_config(void)
{


}

static void adi_init(void)
{



}

#ifdef CONFIG_PM
int prepare_deep_sleep(void)
{
	u32 val;
	int i;
	u32 ahb_sts, gr_stc_sts, gr_clk_dly;
	u16 val_short;

	/* for saving global registers. */
	u32 ahb_ctl0, gen0, irq_flags;
	u16 backlight;

	/*
	irq_flags = __raw_readl(INT_IRQ_EN);
	*/

	/*
	__raw_writel(0xffffffff, INT_IRQ_DISABLE);
	__raw_writel(0xffffffff, INT_FIQ_DISABLE);
	*/

	/* disable all a-die interrupts. */
	/*
	ANA_REG_SET(ANA_INT_EN, 0x0);
	*/

	pins_config();
	adi_init();

	printk("#######: prepare system for deep sleep.......\n");
	/* disable voice band. */
	__raw_writel(0x0, GR_BUSCLK);
    /* ########################################## */

	//ANA_REG_OR(ANA_LDO_SLP, (FSM_RF0_BP_EN | FSM_RF1_BP_EN));
	ANA_REG_SET(ANA_LDO_SLP, 0xa7ff);

	/* vibrate power down. */
	ANA_REG_OR(ANA_ANA_CTL0, VIBR_PD);

	/* enable LDOs auto power down. */
	ANA_REG_OR(ANA_ANA_CTL0, FSM_AFCPD_EN);

	/* enable OTP. */
	ANA_REG_BIC(ANA_ANA_CTL0, OTP_ENABLE);

	/* shut down LDOs. */
	/*
	ANA_REG_SET(ANA_LDO_PD_CTL, 0x5555);
	*/

    /* ########################################## */

	backlight = ANA_REG_GET(ANA_LED_CTL);

	ANA_REG_SET(ANA_LED_CTL, 0xA081);//LED_CTRL

	ANA_REG_SET(ANA_PA_CTL, 0xC055); //PA_CTRL

	mdelay(100);

	val_short = ANA_REG_GET(ANA_LDO_SLP);
	printk("#####: ANA_LDO_SLP = %04x\n", val_short);

	val_short = ANA_REG_GET(ANA_ANA_CTL0);
	printk("#####: ANA_ANA_CTL0 = %04x\n", val_short);

/* #################################### */
	/* close all AHB devices. */
	val = __raw_readl(AHB_CTL0);
	ahb_ctl0 = val;
	val &= ~(AHB_CTL0_DCAM_EN | AHB_CTL0_CCIR_EN | AHB_CTL0_LCDC_EN |
			AHB_CTL0_SDIO_EN | AHB_CTL0_USBD_EN | AHB_CTL0_DMA_EN |
			AHB_CTL0_BM0_EN | AHB_CTL0_NFC_EN | AHB_CTL0_BM1_EN |
			AHB_CTL0_VSP_EN | AHB_CTL0_ROT_EN);
	__raw_writel(val, AHB_CTL0);
/* ####################################### */

	/* setup EMC. */
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


	/* enable AHB devices's clock AUTO-GATE. */
	/* ################################################################## */
	val = __raw_readl(AHB_CTL1);
	val |= (AHB_CTRL1_ARM_AUTO_GATE_EN | AHB_CTRL1_AHB_AUTO_GATE_EN |
			AHB_CTRL1_MCU_AUTO_GATE_EN | AHB_CTRL1_EMC_CH_AUTO_GATE_EN |
			AHB_CTRL1_EMC_AUTO_GATE_EN | AHB_CTRL1_ARMMTX_AUTO_GATE_EN |
			AHB_CTRL1_MSTMTX_AUTO_GATE_EN | AHB_CTRL1_ARM_DAHB_SLEEP_EN);
	__raw_writel(val, AHB_CTL1);


	/* enable XTL auto power down. */
	val = __raw_readl(GR_CLK_EN);
	val |= MCU_XTLEN_AUTOPD_EN;
	val &= ~BIT_17;		//must do, I don't know reason.
	__raw_writel(val, GR_CLK_EN);

	/* disable pwm[3:0]*/
	val = __raw_readl(GR_CLK_EN);
	val &= ~(CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN);
	__raw_writel(val, GR_CLK_EN);
	/* ################################################################## */

	/*
	printk("####: check everything before disabling APB devices!\n");
	verify_dsp_deep_sleep();
	verify_ahb_sts();
	*/

	/*
	for (i = 0;i < 10; i++) {
		printk("####[%d]: IRQ_STS = %08x, FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_STS), __raw_readl(INT_FIQ_STS));

		printk("####[%d]: RAW_IRQ_STS = %08x, RAW_FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_RAW_STS), __raw_readl(INT_FIQ_RAW_STS));

		mdelay(500);
		val_short = ANA_REG_GET(ANA_INT_RAW);

		printk("#####: Analog die interrupt status: %04x\n", val_short);

	}
	mdelay(1000);
	*/
/* ####################################### */
	/* disable all APB devices. */
	val = __raw_readl(GR_GEN0);
	gen0 = val;
	val &= ~(GEN0_TIMER_EN | GEN0_SIM0_EN | GEN0_I2C_EN  | GEN0_EFUSE_EN |
			GEN0_I2S_EN | GEN0_PIN_EN | GEN0_CCIR_MCLK_EN | GEN0_EPT_EN |
			GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | GEN0_UART1_EN  |
			GEN0_UART2_EN | GEN0_VB_EN);
	__raw_writel(val, GR_GEN0);
/* ####################################### */

/*
	mdelay(10);
	ahb_sts = __raw_readl(AHB_STS);
	gr_stc_sts = __raw_readl(GR_STC_STATE);
	gr_clk_dly = __raw_readl(GR_CLK_DLY);

	val = __raw_readl(GR_GEN0);
	val |= GEN0_UART1_EN;
	__raw_writel(val, GR_GEN0);

	printk("####: after enabling UART1: ahb_sts = %08x, stc_sts = %08x, clk_dly = %08x\n",
				ahb_sts, gr_stc_sts, gr_clk_dly);
	mdelay(1000);


	val = __raw_readl(GR_GEN0);
	val &= ~GEN0_UART1_EN;
	__raw_writel(val, GR_GEN0);
	mdelay(100);
*/

#if 0
	/* set keypad as wakeup source. */
	val = __raw_readl(GR_GEN0);
	val |= (GEN0_KPD_EN | GEN0_KPD_RTC_EN);
	__raw_writel(val, GR_GEN0);

	/* enable interrupt of keypad. */
	val = BIT_10;
	__raw_writel(val, INT_IRQ_EN);
#endif
    /* ###################################### */
	val = __raw_readl(AHB_PAUSE);
	val |= (MCU_SYS_SLEEP_EN | MCU_DEEP_SLEEP_EN);
	__raw_writel(val, AHB_PAUSE);
    /* ###################################### */


	sc8800g_cpu_standby();

	mdelay(100);

	__raw_writel(ahb_ctl0, AHB_CTL0);
	__raw_writel(gen0, GR_GEN0);
//	__raw_writel(irq_flags, INT_IRQ_EN);

	mdelay(200);

	ANA_REG_SET(ANA_LED_CTL, backlight);


	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");

	/*
	for (i = 0;i < 10; i++) {
		printk("####[%d]: IRQ_STS = %08x, FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_STS), __raw_readl(INT_FIQ_STS));

		printk("####[%d]: RAW_IRQ_STS = %08x, RAW_FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_RAW_STS), __raw_readl(INT_FIQ_RAW_STS));

		val_short = ANA_REG_GET(ANA_INT_RAW);

		printk("#####: Analog die interrupt status: %04x\n", val_short);

		mdelay(500);

	}
	*/
	/*

	val = __raw_readl(AHB_CTL0);
	val = (AHB_CTL0_DCAM_EN | AHB_CTL0_CCIR_EN | AHB_CTL0_LCDC_EN |
			AHB_CTL0_SDIO_EN | AHB_CTL0_USBD_EN | AHB_CTL0_DMA_EN |
			AHB_CTL0_BM0_EN | AHB_CTL0_NFC_EN | AHB_CTL0_BM1_EN |
			AHB_CTL0_VSP_EN | AHB_CTL0_ROT_EN);
	__raw_writel(val, AHB_CTL0);


	val = __raw_readl(GR_GEN0);
	val |= (GEN0_TIMER_EN | GEN0_SIM0_EN | GEN0_I2C_EN  | GEN0_EFUSE_EN |
			GEN0_I2S_EN | GEN0_PIN_EN | GEN0_CCIR_MCLK_EN | GEN0_EPT_EN |
			GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | GEN0_UART1_EN  |
			GEN0_UART2_EN | GEN0_VB_EN);
	__raw_writel(val, GR_GEN0);

	*/
/*
	val = __raw_readl(GR_GEN0);
	val |= (GEN0_TIMER_EN | GEN0_UART0_EN | GEN0_UART1_EN  | GEN0_UART2_EN);
	__raw_writel(val, GR_GEN0);

	printk("####: after waking up: ahb_sts = %08x, stc_sts = %08x, clk_dly = %08x\n",
				ahb_sts, gr_stc_sts, gr_clk_dly);
*/

	return 0;
}

EXPORT_SYMBOL(prepare_deep_sleep);
#endif



void sc8800g_set_pll(void)
{
	u32 val1, val2;

	printk("##: set new pll value!\n");
	printk("##: set new pll value!\n");
	printk("##: set new pll value!\n");
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	val2 &= 0xffff000;
	val2 |= 0xc8;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
}
EXPORT_SYMBOL(sc8800g_set_pll);


void sc8800g_save_pll(void)
{
	u32 val1, val2;

	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	pll_n = val2 & 0x0fff;
	val2 &= 0xffff000;
	val2 |= SC8800G2_DEFAULT_PLL_N;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
	//printk("##: save_pll: val2 = %08x\n", val2);
}

void sc8800g_restore_pll(void)
{
	u32 val1, val2;
	if (pll_n > 0x0fff) pll_n = 200;
	val1 = __raw_readl(GR_GEN1);
	val1 |= BIT_9;
	__raw_writel(val1, GR_GEN1);

	val2 = __raw_readl(GR_MPLL_MN);
	val2 &= 0xffff000;
	val2 |= pll_n;
	__raw_writel(val2, GR_MPLL_MN);

	val1 &= ~BIT_9;
	__raw_writel(val1, GR_GEN1);
	//printk("##: restore_pll: val2 = %08x\n", val2);

}



#define PGD_ENTRY 4096
pmd_t pmd_saved0, pmd_saved1;
pmd_t *pmd_saved = NULL;
void __iomem *iram_start;
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

#ifdef CONFIG_PM
void sc8800g_enter_deepsleep_internal(void)
{
	unsigned long flags;
	hw_spin_lock_irqsave(&deepsleep_lock, flags);
	sc8800g_cpu_standby_prefetch();
	hw_spin_unlock_irqrestore(&deepsleep_lock, flags);

}
#endif


static int enable_mcu_sleep(void)
{
    u32 val;
    /* AHB_PAUSE */
    val = __raw_readl(AHB_PAUSE);
    val |= (MCU_DEEP_SLEEP_EN | MCU_SYS_SLEEP_EN);
    __raw_writel(val, AHB_PAUSE);
    return 0;
}


static struct wake_lock messages_wakelock;

u32 timer_int_counter0 = 0, timer_int_counter1 = 0;
u32 tick_sched_timer_counter0 = 0, tick_sched_timer_counter1 = 0;
u32 sleep_counter0 = 0, sleep_counter1 = 0;
u32 interrupt_counter0 = 0, interrupt_counter1 = 0;
u32 schedu_counter0 = 0, schedu_counter1 = 0;

#define THREAD_INTERVAL 30

static int print_thread(void *pdata)
{
	int i;
	u32 val;
    while(1) {
           wake_lock(&messages_wakelock);
	add_pm_message(get_sys_cnt(), "************* print_thread start. *************", 0, 0, 0); 
	stop_pm_message();

           uptime = get_sys_cnt();
/*
	printk("###: c1 = %08x.\n", sc8800g_read_cp15_c1());
	printk("###: c2 = %08x.\n", sc8800g_read_cp15_c2());
	printk("###: c3 = %08x.\n", sc8800g_read_cp15_c3());
*/
	sleep_counter0 = sleep_counter1;
	sleep_counter1 = sleep_counter;

           printk("##: thread_loops = %d, idle_loops = %d\n", ++thread_loops, idle_loops);           
	    printk("##: mode = %d, sleep_counter = %d freq = %d/s.\n", 	sleep_mode, 
		sleep_counter, (sleep_counter1 - sleep_counter0) / THREAD_INTERVAL);
	    printk("##[%d]: uptime = %d, sleep_time = %d, idle_time = %d.\n", 
                (sleep_time * 100) /uptime, uptime, sleep_time, idle_time);

	timer_int_counter0 = timer_int_counter1;
	timer_int_counter1 = timer_int_counter;
	printk("****: timer_interrupt = %d events/s\n", 
		(timer_int_counter1 - timer_int_counter0) / THREAD_INTERVAL);

	tick_sched_timer_counter0 = tick_sched_timer_counter1;
	tick_sched_timer_counter1 = tick_sched_timer_counter;
	printk("****: tick_sched_timer_counter = %d events/s\n", 
		(tick_sched_timer_counter1 - tick_sched_timer_counter0) / THREAD_INTERVAL);

	interrupt_counter0 = interrupt_counter1;
	interrupt_counter1 = interrupt_counter;
	printk("****: interrupt_counter = %d events/s\n", 
		(interrupt_counter1 - interrupt_counter0) / THREAD_INTERVAL);

	schedu_counter0 = schedu_counter1;
	schedu_counter1 = schedu_counter;
	printk("****: schedu_counter = %d events/s\n", 
		(schedu_counter1 - schedu_counter0) / THREAD_INTERVAL);

	//sc8800g_enter_deepsleep_internal();
  
/*       
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
*/
 	//print_pm_message();
	printk("##: show clock info:\n");
	sc8800g_get_clock_status();
	
/*
	verify_dsp_deep_sleep_by_value(gr_stc_state);
	verify_ahb_sts_by_value(ahb_sts);
*/
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
           mdelay(100);
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
    if (!sleep_wait_emc_sleep()) {
        printk("###: EMC channel[6,7,8] is NOT idle!\n");
        printk("###: EMC channel[6,7,8] is NOT idle!\n");
        printk("###: EMC channel[6,7,8] is NOT idle!\n");    
        return;
    }
    val = __raw_readl(AHB_CTL0);
    val &= ~AHB_CTL0_MASK;
    __raw_writel(val, AHB_CTL0);
}


int supsend_ldo_turnoff(void)
{
	u32 val = 0;
/*
    ANA_REG_SET(ANA_LDO_PD_SET, 
		BIT_5 | BIT_9);
*/
	val = ANA_REG_GET(ANA_LDO_SLP);
	if ( val != 0xa7fb) {
		printk("##: ANA_LDO_SLP: wrong vaule[%08x].\n", val);
	}

    ANA_REG_SET(ANA_LDO_PD_CTL, 
		BIT_0 | BIT_2 | BIT_6 | BIT_8 | BIT_10 | BIT_12 | BIT_14);

	return 0;
}

int supsend_ldo_turnon(void)
{

	return 0;
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

int sc8800g_enter_deepsleep(int inidle)
{
    int status;
    u32 t0, t1, delta;
    int ret = 0;
    u32 val;
    int i;
	unsigned long flags;



    REG_LOCAL_VALUE_DEF;

    sleep_counter++;

/*
	if (sprd_pm_suspend()) {
		printk("##: sprd_pm_suspend() doesn't allow deep sleep!\n");
		ret = -1;
		goto pm_resume;
	}
*/
	if (!hw_irqs_disabled())  {
		flags = sc8800g_read_cpsr();
		printk("##: Error(%s): IRQ is enabled(%08x)!\n", 
			inidle ? "idle" : "wakelock_suspend", flags);
	}

	/*	
	__raw_writel(IRQ_GPIO, INT_IRQ_DISABLE);
	__raw_writel(IRQ_GPIO, INT_FIQ_DISABLE);
	*/ 
    status = sc8800g_get_clock_status();

/*
    t0 = get_sys_cnt();
    sc8800g_cpu_standby();
    t1 = get_sys_cnt();
    delta = t1 - t0;
    sleep_time += delta;
*/

/*
    t0 = get_sys_cnt();
    sc8800g_enter_deepsleep_internal();
    t1 = get_sys_cnt();
    delta = t1 - t0;
    sleep_time += delta;
*/
/*
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
*/
	sc8800g_save_pll();

    if (status & DEVICE_AHB)  {
        sleep_mode = SLEEP_MODE_ARM_CORE;
        sc8800g_cpu_standby();
	sc8800g_restore_pll();
    }
    else if (status & DEVICE_APB) {
        wait_until_uart1_tx_done();
        sleep_mode = SLEEP_MODE_MCU;
        SAVE_GLOBAL_REG;
        disable_audio_module();
        disable_ahb_module();
        //enable_mcu_sleep();
       ret =  sc8800g_cpu_standby_prefetch();
        RESTORE_GLOBAL_REG;
        udelay(20);
	sc8800g_restore_pll();
    }
    else {
	//__raw_writel(0, TIMER1_CONTROL);
	supsend_ldo_turnoff();

	add_pm_message(get_sys_cnt(), "deepsleep_enter: inidle = ", inidle, 0, 0);
/*
        printk("IRQ_STS = %08x, %s\n", 
		__raw_readl(INT_IRQ_STS), inidle ? "idle" : "pm_suspend");
*/

        wait_until_uart1_tx_done();
        sleep_mode = SLEEP_MODE_DEEP;
        SAVE_GLOBAL_REG;
        disable_audio_module();
        disable_apb_module();
        disable_ahb_module();

/* 
        add_pm_message("AHB_CTL0", __raw_readl(AHB_CTL0));
        add_pm_message("AHB_CTL1", __raw_readl(AHB_CTL1));
        add_pm_message("AHB_PAUSE", __raw_readl(AHB_PAUSE));
        add_pm_message("GR_GEN0", __raw_readl(GR_GEN0));
        add_pm_message("GR_PCTL", __raw_readl(GR_PCTL));
        add_pm_message("GR_BUSCLK", __raw_readl(GR_BUSCLK));
        add_pm_message("GR_POWCTL0", __raw_readl(GR_POWCTL0));
        add_pm_message("GR_POWCTL1", __raw_readl(GR_POWCTL1));
        add_pm_message("GR_CLK_EN", __raw_readl(GR_CLK_EN));
        add_pm_message("ANA_ANA_CTL0", ANA_REG_GET(ANA_ANA_CTL0));
        add_pm_message("ANA_LDO_SLP", ANA_REG_GET(ANA_LDO_SLP));
*/

	gr_stc_state = __raw_readl(GR_STC_STATE);
	ahb_sts = __raw_readl(AHB_STS);
	gr_clk_dly = __raw_readl(GR_CLK_DLY);
	dma_sts = __raw_readl(DMA_TRANS_STS);
	irq_sts = __raw_readl(INT_IRQ_STS);
	fiq_sts = __raw_readl(INT_FIQ_STS);

	if (0 != irq_sts) 
		add_pm_message(get_sys_cnt(), "Going to deep sleep with pending irq: INT_IRQ_STS = ", 
				irq_sts, 0, 0);
		
	if (0 != fiq_sts) 
		add_pm_message(get_sys_cnt(), "Going to deep sleep with pending fiq: INT_FIQ_STS = ", 
				fiq_sts, 0, 0);
		

	//queue_work(deep_sleep_work_queue, &deep_sleep_wrok);


        t0 = get_sys_cnt();
        //enable_mcu_sleep();
       ret = sc8800g_cpu_standby_prefetch();
        RESTORE_GLOBAL_REG;
        t1 = get_sys_cnt();
        delta = t1 - t0;
        sleep_time += delta;
        udelay(20);
	supsend_ldo_turnon();

	if (ret) {
		printk("##: bad return value.\n");
		printk("##: bad return value.\n");
		printk("##: bad return value.\n");
	}

	sc8800g_restore_pll();
	irq_sts = __raw_readl(INT_IRQ_STS);
	fiq_sts = __raw_readl(INT_FIQ_STS);

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

/*		
	add_pm_message(dma_sts,  " dma_sts, ahb_sts, gr_stc_state, gr_clk_dly", 
			ahb_sts, gr_stc_state, gr_clk_dly);
*/
/*
	printk("ahb_sts = %08x\n", ahb_sts);
	printk("gr_stc_state = %08x\n", gr_stc_state);
	printk("gr_clk_dly = %08x\n", gr_clk_dly);
	printk("dma_sts = %08x\n", dma_sts);
*/
/*
        printk("IRQ_STS = %08x, %s\n", 
		__raw_readl(INT_IRQ_STS), inidle ? "idle" : "pm_suspend");
*/
   }
/*
pm_resume:
	sprd_pm_resume();
*/
    return ret;
}
EXPORT_SYMBOL(sc8800g_enter_deepsleep);
#endif


#ifdef CONFIG_NKERNEL
static void nkidle(void)
{
      int val;
    u32 t0, t1, delta;
	add_pm_message(get_sys_cnt(), "nkidle: 1111--enter.", 0, 0, 0);

	if (!need_resched()) {
		hw_local_irq_disable();
		if (!raw_local_irq_pending()) {
			val = os_ctx->idle(os_ctx);
			if (0 == val) {
				if (!has_wake_lock_for_suspend(WAKE_LOCK_SUSPEND) && 
					(!sprd_pm_suspend_canceled())) {
					sc8800g_enter_deepsleep(1);
				}
				else {
				    idle_loops++;
				    t0 = get_sys_cnt();
				    sc8800g_cpu_standby();
				    t1 = get_sys_cnt();
				    delta = t1 - t0;
				    idle_time += delta;
				}
			}
		}
		hw_local_irq_enable();
	}
	local_irq_enable();
	add_pm_message(get_sys_cnt(), "nkidle: 1111--leave.", 0, 0, 0);
}

#endif

#ifdef CONFIG_PM
#define DEEP_SLEEP_INTERVAL (HZ)

static void deep_sleep_timeout(unsigned long data);
static DEFINE_TIMER(deep_sleep_timer, deep_sleep_timeout, 0, 0);


static void deep_sleep_timeout(unsigned long data)
{
	printk("###: deep_sleep_timeout()!\n");
	queue_work(deep_sleep_work_queue, &deep_sleep_wrok);
	mod_timer(&deep_sleep_timer, jiffies + DEEP_SLEEP_INTERVAL);
}




int sc8800g_prepare_deep_sleep(void)
{
    u32 val = 0;
    int i;
    pid_t pid_number;
    u32 * pint = (u32 *)sc8800g_cpu_standby;
    /*
    for (i = 0; i <64; i++) printk("pint[%d] = %08x\n", i, pint[i]);
	*/
/*
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
*/

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
		//__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}

	for (i = 6; i < 9; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		if (val & RF_AUTO_SLEEP_ENABLE_CHX) {
                      printk("######: channel[%d] does set RF_AUTO_SLEEP_ENABLE_CHX!\n", i);
               }
		//__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}


	val = __raw_readl(EMC_DCFG2);
	if (val & DRF_AUTO_SLEEP_MODE) printk("#####: EMC_DCFG2 does set DRF_AUTO_SLEEP_MODE!\n");
	if (!(val & DRF_REF_CNT_RST)) printk("####: EMC_DCFG2 doesn't set DRF_REF_CNT_RST!\n");
	//__raw_writel(val, EMC_DCFG2);


    /* AHB_PAUSE */
    val = __raw_readl(AHB_PAUSE);
    val &= ~(MCU_CORE_SLEEP | MCU_DEEP_SLEEP_EN | APB_SLEEP);
    val |= (MCU_SYS_SLEEP_EN);
    __raw_writel(val, AHB_PAUSE);


    /* GR_PCTL */
    val = __raw_readl(GR_PCTL);
    val |= (MCU_MPLL_EN);
    __raw_writel(val, GR_PCTL);

    /* AHB_CTL1 */
	val = __raw_readl(AHB_CTL1);
	val |= (AHB_CTRL1_EMC_CH_AUTO_GATE_EN | AHB_CTRL1_EMC_AUTO_GATE_EN | 
	        AHB_CTRL1_ARMMTX_AUTO_GATE_EN | AHB_CTRL1_MSTMTX_AUTO_GATE_EN);
	__raw_writel(val, AHB_CTL1);


    /* GR_CLK_EN */
    /* enable XTL auto power down. */
    val = __raw_readl(GR_CLK_EN);
    val |= MCU_XTLEN_AUTOPD_EN;
/*
    val &= ~BIT_17;		//must do, I don't know reason.
*/
   __raw_writel(val, GR_CLK_EN);

    /* disable pwm[3:0]*/
    /*
    val = __raw_readl(GR_CLK_EN);
    val &= ~(CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN);
    __raw_writel(val, GR_CLK_EN);
    */

    //ANA_REG_OR(ANA_LDO_SLP, (FSM_RF0_BP_EN | FSM_RF1_BP_EN));
    ANA_REG_SET(ANA_LDO_SLP, 0xa7fb);


    /* ANA_ANA_CTL0 */
    /* vibrate power down. */
    /* 
    ANA_REG_OR(ANA_ANA_CTL0, VIBR_PD);
    */
    /* enable LDOs auto power down. */
    ANA_REG_OR(ANA_ANA_CTL0, FSM_AFCPD_EN);

    /* enable OTP. */
    /*
    ANA_REG_BIC(ANA_ANA_CTL0, OTP_ENABLE);
    */

    /* AHB_CTL1 */
    val = POWCTL1_CONFIG;
    __raw_writel(val, GR_POWCTL1);


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
        sc8800g_cpu_standby, sc8800g_cpu_standby_end);
    if ((sc8800g_cpu_standby_end - sc8800g_cpu_standby + 128) > SLEEP_CODE_SIZE) {
          panic("##: code size is larger than expected, need more memory!\n");
    }

    memcpy_toio(iram_start, sc8800g_cpu_standby, SLEEP_CODE_SIZE);

    /* we will get these code free different virtual address, so flush it 
        to make it visible, mayde not necessary.
    */
    flush_cache_all();
/*
map_iram_identically();
*/
    pint = (u32 *)iram_start;
    /*
    for (i = 0; i <64; i++) printk("pint[%d] = %08x\n", i, pint[i]);
    */
	pid_number = kernel_thread(print_thread, NULL, 0);
	if (pid_number < 0) {
		printk("Can't crate test thread!\n");
	}
	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock");
       init_pm_message();


    pm_idle = nkidle;
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
    return 0;
}
EXPORT_SYMBOL(sc8800g_prepare_deep_sleep);
#endif

