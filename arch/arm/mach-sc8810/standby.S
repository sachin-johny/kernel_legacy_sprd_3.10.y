/*
 * sc8800g standby mode
 *
 * Author: Wang Liwei 
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>

		.text

#ifdef CONFIG_ARCH_SC8810
ENTRY(sc8800g_cpu_standby)
    mov r0, #0
    mcr p15, 0, r0, c7, c0, 4
    nop
    nop
    nop
    mov pc, lr

#endif
