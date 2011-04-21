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

#ifdef CONFIG_ARCH_SC8800G
ENTRY(sc8800g_cpu_standby)

   MOV  R1, #0x20
Wait0:
    SUB  R1, R1, #0x1
    CMP  R1, #0x0
    BNE  Wait0
    
    mov r0, #0
    mcr p15, 0, r0, c7, c0, 4

    MOV  R1, #0x20
Wait1:
    SUB  R1, R1, #0x1
    CMP  R1, #0x0
    BNE  Wait1
   
    mov pc, lr

#endif
