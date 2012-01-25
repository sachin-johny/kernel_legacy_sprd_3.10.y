/*
 * sc8810 standby mode
 *
 * Author: Wang Liwei 
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/regs_ahb.h>

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

	.text
	.align 12

#ifdef CONFIG_ARCH_SC8810
ENTRY(sc8810_standby_iram)
#if 0
	stmfd sp!, {r4-r12, lr}
	
	ldr		r1, =0xe0033008
	ldr		r2, [r1, #0]
	orr		r2, r2, #(0x1 << 21)
	str		r2, [r1, #0]

	
     mov r0, #'@'
     bl printch

     mov r0, #'>'
     bl printch

	ldr r0, =0x2ff
	bl my_delay
	
	ldr		r1, =0xe0033008
	ldr		r2, [r1, #0]
	bic		r2, r2, #(0x1 << 21)
	str		r2, [r1, #0]
	ldmfd sp!, {r4-r12, lr}
#endif

sprd_pm_mapped_va:
	ldr r0, =sprd_pm_va_to_iram
	ldr r1, =sc8810_standby_iram
	ldr r2, =iram_start
	ldr r3, [r2, #0]
	sub r0, r0, r1
	add r0, r0, r3
	mov pc, r0
sprd_pm_va_to_iram:
	/* Run inside IRAM now. */
	nop
	nop
	nop
	
	dsb
	isb
/*
	ldr r1, =AHB_PAUSE
	ldr r2, [r1]
	orr r2, r2, #0x06
	str r2, [r1, #0]

	mov r0, #0x20
1:  sub	r0, r0, #0x1
    cmp	r0, #0x0
    bne  1b
*/	

	wfi
	nop
	nop
	nop
    mov pc, lr
    nop
    nop

ENDPROC(sc8810_standby_iram)


ENTRY(sc8810_standby_exit_iram)
	mrs r0, cpsr
	mov r6, r0
	ldr r1, =0x01d3
	cmp r0, r1
	blne dead
/*
	adr r0, sleep_asm_counter
	ldr r1, [r0]
	add r1, r1, #1
	str r1, [r0]
	cmp r1, #32
	bleq dead
*/	
	
	ldr r0, =0x0020
	bl my_delay
#if 0	
	/* enable uart1 clock to print some messages. */
	ldr		r1, =0x8b000008	/*physical address. */
	ldr		r2, [r1, #0]
	orr		r2, r2, #(0x1 << 21)
	str		r2, [r1, #0]
	
	ldr r0, =0x20
	bl my_delay

     mov r0, #'#'
     bl printch

     mov r0, #'<'
     bl printch

     mov r0, #'\r'
     bl printch

     mov r0, #'\n'
     bl printch
#endif

	ldr	r0, =sp_pm_collapse_exit
	/* 
	convert to physical address, 
	need to find a more elegent way. 
	*/
	ldr	r1, =0xc0000000
	sub	r0, r0, r1
	mov pc, r0
ENDPROC(sc8810_standby_exit_iram)


ENTRY(printascii_phy2)
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
ENDPROC(printascii_phy2)

printch:
		addruart r3
		mov	r1, r0
		mov	r0, #0
		b	1b

ENTRY(my_delay)
1:  sub	r0, r0, #0x1
    cmp	r0, #0x0
    bne  1b
    mov pc, lr
ENDPROC(my_delay)

ENTRY(dead)
	ldr		r1, =0x8b000008	/*physical address. */
	ldr		r2, [r1, #0]
	orr		r2, r2, #(0x1 << 21)
	str		r2, [r1, #0]
	
	ldr r0, =0x20
	bl my_delay

1:	adr	r0, str_a1
	bl	printascii_phy2

	ldr r0, =0x000fffff
	bl my_delay
	
	b 1b
ENDPROC(dead)

	.globl sleep_asm_counter
sleep_asm_counter:
	.long	0x0


str_a1:	.asciz	"dead!"

ENTRY(sc8810_standby_iram_end)
#endif
