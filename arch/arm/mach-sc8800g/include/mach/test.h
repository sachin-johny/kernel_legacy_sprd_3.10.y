
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_emc.h>
#include <mach/regs_int.h>



#define TIMER_REG(off) (SPRD_TIMER_BASE + (off))
#define TIMER0_LOAD     TIMER_REG(0x0000)
#define TIMER0_VALUE    TIMER_REG(0x0004)
#define TIMER0_CONTROL  TIMER_REG(0x0008)
#define TIMER0_CLEAR    TIMER_REG(0x000C)
#define TIMER1_LOAD     TIMER_REG(0x0020)
#define TIMER1_VALUE    TIMER_REG(0x0024)
#define TIMER1_CONTROL  TIMER_REG(0x0028)
#define TIMER1_CLEAR    TIMER_REG(0x002C)

#define SYSCNT_REG(off) (SPRD_SYSCNT_BASE + (off))
#define SYSCNT_COUNT    SYSCNT_REG(0x0004)
#define SYSCNT_CTL      SYSCNT_REG(0x0008)

#define GREG_REG(off)   (SPRD_GREG_BASE + (off))
#define GREG_GEN0       GREG_REG(0x0008)
#define GREG_GEN1       GREG_REG(0x0018)

#define GPTIMER_FREQ    32768
#define SYSCNT_FREQ	1000


#define SLEEP_MODE_ARM_CORE 0
#define SLEEP_MODE_MCU 1
#define SLEEP_MODE_DEEP 2

#define 	SPRD_HARD_INTERRUPT_NUM 32
#define	SPRD_IRQ_NUM			1024

#define	SPRD_THREADS_ARRAY_SIZE	2048


#define IRQ_FOR_DSP_MASK (BIT_5 | BIT_15 | BIT_16)
#define IRQ_FOR_DSP 1
#define IRQ_FOR_ARM 2


/*********************************/
/* for debug only. */
extern u16 val_short;
/* for saving global registers. */
extern u32 ahb_ctl0, gen0, gen_clk_en, busclk_alm, irq_flags;
extern u32 val;
extern u32 sleep_mode;
extern u32 idle_time;
extern int idle_loops;
extern u32 timer_int_counter;
extern u32 tick_sched_timer_counter;
extern u32 interrupt_counter;
extern u32 schedu_counter;
extern u32 sleep_time_inidle;

extern struct timespec now_ts_pm;
extern int sprd_suspend_enable;
extern u32 sprd_suspend_interval;

/*********************************/


#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
extern void printch(char);
extern void printhex8(int);
#endif

extern u32 sc8800g_read_cp15_c1(void);
extern u32 sc8800g_read_cp15_c2(void);
extern u32 sc8800g_read_cp15_c3(void);
extern u32 sc8800g_read_cpsr(void);
extern void add_pm_message(u32 when, char *msg, u32 val0, 
                                              u32 val1, u32 val2);

extern void add_pm_message_val64(u32 when, char *msg, u32 val0, 
                                              u32 val1, u32 val2, u64 val64);
extern void parse_sprd_hard_irq(unsigned long val);
extern void inc_sprd_irq(int irq);
extern void show_sprd_irq_info(void);
extern void inc_sprd_thread_counts(int thread);
extern int sprd_irq_for_arm(u32 val);

static u32 inline get_sys_cnt(void)
{
	u32 val1, val2;
	val1 = __raw_readl(SYSCNT_COUNT);
	val2 = __raw_readl(SYSCNT_COUNT);
	while(val2 != val1) {
		val1 = val2;
		val2 = __raw_readl(SYSCNT_COUNT);
	}
	return val2;
}

