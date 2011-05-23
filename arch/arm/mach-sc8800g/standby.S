/*
 * sc8800g standby mode
 *
 * Author: Wang Liwei 
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/regs_ahb.h>

		.text

#ifdef CONFIG_ARCH_SC8800G

	.macro	addruart,rx
	@ see if the MMU is enabled and select appropriate base address
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1
	ldreq	\rx, =SPRD_SERIAL1_PHYS
	ldrne	\rx, =SPRD_SERIAL1_BASE
	.endm

	.macro	senduart,rd,rx
	and	\rd,\rd,#0xFF
	str	\rd, [\rx, #0x00]
	.endm

	.macro	waituart,rd,rx
	@ wait for TX_READY
1:	ldr	\rd, [\rx, #0x0C]
	mov	\rd,\rd,lsr #8
	and	\rd,\rd,#0xFF
	teq	\rd, #0x00
	bne	1b
	.endm

	.macro	busyuart,rd,rx
	@ wait for TX_READY
2:	ldr	\rd, [\rx, #0x0C]
	mov	\rd,\rd,lsr #8
	and	\rd,\rd,#0xFF
	teq	\rd, #0x00
	bne	2b
	.endm

.align 12
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


 return_address: 

/****************************/
/* make marks here for testing. */
/****************************/
    mov r1, #0x55
    mov r2, r1
    mov r3, r1
    mov r4, r1
    mov r5, r1
    mov r6, r1
    mov r7, r1
    mov r8, r1
    mov r9, r1
    mov r10, r1
    mov r11, r1
    mov r12, r1

 
     ldmfd sp!, {r4-r12, r14}
     mov  pc, lr 

ENTRY(sc8800g_cpu_standby_prefetch)

    stmfd sp!, {r4-r12, r14}
    adr	lr, return_address
    
    mov  r2, # phy_addr_sleep - sc8800g_cpu_standby
    ldr  r1, =0x40004000
    add r2, r1, r2

/****************************/    
 /*
    stmfd sp!, {r0-r3,ip, lr}
     mov r0, #'*'
     bl printch
     ldmfd sp!, {r0-r3,ip,lr}
*/
/*********************** ****/

    MRC     p15, 0, r0, c1, c0, 0     
    BIC     r0, r0, #1
    MCR     p15, 0, r0, c1, c0, 0
    mov pc, r2
    nop
    nop
    nop

phy_addr_sleep:

/******************************/
/*
loop1:
    mov r0, #'?'
    bl printch
    b loop1
*/
/*****************************/
    nop
    nop

    MRC     p15, 0, r0, c1, c0, 0   //;Disable the ICache, DCache, write buffer, 
                                               //;alignment fault checking
    BIC     r0, r0, #0x1000              // ;for Norflash, need No alignment access.
    BIC     r0, r0, #0x6
    MCR     p15, 0, r0, c1, c0, 0

tci_loop0:
    MRC p15, 0, r15, c7, c14, 3       //;test clean and invalidate
    BNE tci_loop0

    MOV     r0, #0
    MCR     p15, 0, r0, c7, c7, 0     //;Invalidate the ICache and DCache
    MCR     p15, 0, r0, c7, c10, 4    //;Drain the write buffer
    MCR     p15, 0, r0, c8, c7, 0    // ;Invalidate the TLB
    
    NOP
    NOP
    NOP
    NOP
    NOP

    MRC     p15, 0, r4, c1, c0, 0
    MRC     p15, 0, r5, c3, c0, 0


    LDR  R1, =0x20900214
    LDR  R2, [R1]
    ORR  R2, R2, #0x6
    STR  R2, [R1, #0]  
  
   MOV  R1, #0x20
Wait2:
    SUB  R1, R1, #0x1
    CMP  R1, #0x0
    BNE  Wait2

    mov r0, #0
    mcr p15, 0, r0, c7, c0, 4
	nop
	nop
	nop
	nop
	nop
	

      ldr R1, =0xa0
Wait3:
    SUB  R1, R1, #0x1
    CMP  R1, #0x0
    BNE  Wait3
    

    MOV     r1, #0                      //;   Invalidate the ICache and DCache
    MCR     p15, 0, r1, c7, c7, 0    
    MCR     p15, 0, r1, c7, c10, 4     // ;   Drain the write buffer
    MCR     p15, 0, r1, c8, c7, 0      // ;   Invalidate the TLB
    
    MOV     R2, #0xdf
    MOV     R0, #0
    //;   Enable the ICache, DCache, write buffer, alignment fault checking, align faule
    MRC     p15, 0, r1, c1, c0, 0
    CMP     r4, r1
    MOVNE r1, r4
    MOVNE r0, #1
    
    ORR     r1, r1, # 0x7e
    ORR     r1, r1, #0x1000			
    MCR     p15, 0, r1, c1, c0, 0    
	nop
	nop
	nop
	nop
	nop

	/* check domain access. */
     mrc p15, 0, r1, c3, c0, 0
     cmp r5, r1
     movne r1, r5
     movne r0, #3
     mcrne p15, 0, r1, c3, c0, 0

/****************************/
/* make marks here for testing, don't change r0 which store return value.*/

    mov r1, #0xaa
    mov r2, r1
    mov r3, r1
    mov r4, r1
    mov r5, r1
    mov r6, r1
    mov r7, r1
    mov r8, r1
    mov r9, r1
    mov r10, r1
    mov r11, r1
    mov r12, r1
    
/****************************/
     
     b enable_mmu
 	
.align	5
enable_mmu:
    MRC     p15, 0, r1, c1, c0, 0
    ORR     r1, r1, #1
    MCR     p15, 0, r1, c1, c0, 0    
    mov pc, lr
	nop
	nop
	nop
	nop
	nop


ENTRY(printascii_phy)
		addruart r3
		b	2f
1:		waituart r2, r3
		senduart r1, r3
		busyuart r2, r3
		teq	r1, #'\n'
		moveq	r1, #'\r'
		beq	1b
2:		teq	r0, #0
		ldrneb	r1, [r0], #1
		teqne	r1, #0
		bne	1b
		mov	pc, lr
ENDPROC(printascii_phy)

printch:
		addruart r3
		mov	r1, r0
		mov	r0, #0
		b	1b

ENTRY(sc8800g_read_cp15_c1)
     mrc p15, 0, r0, c1, c0, 0
     mov pc, lr

ENTRY(sc8800g_read_cp15_c2)
     mrc p15, 0, r0, c2, c0, 0
     mov pc, lr


ENTRY(sc8800g_read_cp15_c3)
     mrc p15, 0, r0, c3, c0, 0
     mov pc, lr

ENTRY(sc8800g_read_cpsr)
     mrs r0, cpsr
     mov pc, lr


ENTRY(sc8800g_cpu_standby_end)
    
#endif
