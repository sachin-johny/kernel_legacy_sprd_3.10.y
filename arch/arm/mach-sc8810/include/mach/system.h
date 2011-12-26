/* arch/arm/mach-sc8810/include/mach/system.h
 *
 * Copyright (C) 2011 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/proc-fns.h>
#include <mach/hardware.h>
#include <mach/regs_wdg.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>


void cpu_idle_asm(void);

void arch_idle(void)
{
#ifdef CONFIG_SC8810_IDLE
	cpu_idle_asm();
	cpu_do_idle();
#endif
}

#define ANA_WDG_LOAD_TIMEOUT_NUM    (10000)

#define WDG_LOAD_TIMER_VALUE(value) \
        do{\
                    uint32_t   cnt          =  0;\
                    ANA_REG_SET( WDG_LOAD_HIGH, (uint16_t)(((value) >> 16 ) & 0xffff));\
                    ANA_REG_SET( WDG_LOAD_LOW , (uint16_t)((value)  & 0xffff) );\
                    while((ANA_REG_GET(WDG_INT_RAW) & WDG_LD_BUSY_BIT) && ( cnt < ANA_WDG_LOAD_TIMEOUT_NUM )) cnt++;\
                }while(0)

#define HWRST_STATUS_RECOVERY (0x20)
#define HWRST_STATUS_NORMAL (0X40)
#define HWRST_STATUS_ALARM (0X50)
#define HWRST_STATUS_SLEEP (0X60)

static inline void arch_reset(char mode, const char *cmd)
{
	/* our chip reset code */
    volatile int i;
    for(i=0xffff; i>0;i--);
    if(!(strncmp(cmd, "recovery", 8))){
       ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_RECOVERY);
	}else if(!strncmp(cmd, "alarm", 5)){
       ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_ALARM);
	}else if(!strncmp(cmd, "fastsleep", 9)){
       ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_SLEEP);
    }else{
        ANA_REG_SET(ANA_RST_STATUS, HWRST_STATUS_NORMAL);
    }
    // turn on watch dog clock
    ANA_REG_OR(ANA_AGEN, AGEN_WDG_EN | AGEN_RTC_ARCH_EN | AGEN_RTC_WDG_EN);
    ANA_REG_SET (WDG_LOCK, WDG_UNLOCK_KEY);
    ANA_REG_AND (WDG_CTRL, (~WDG_INT_EN_BIT));
    WDG_LOAD_TIMER_VALUE(0x50);
    ANA_REG_OR (WDG_CTRL, WDG_CNT_EN_BIT);
    ANA_REG_SET (WDG_LOCK, (~WDG_UNLOCK_KEY));
}
