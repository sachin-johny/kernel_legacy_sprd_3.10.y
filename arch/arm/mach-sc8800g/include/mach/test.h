
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


#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
extern void printch(char);
extern void printhex8(int);
#endif



static u32 inline get_sys_cnt(void)
{
	return __raw_readl(SYSCNT_COUNT);
}

