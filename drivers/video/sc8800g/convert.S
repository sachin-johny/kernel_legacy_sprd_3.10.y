@input parameter
@r0:src addr
@r1:dst addr
@r2:w*h
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(format_convert)

     STMDB SP!, {r4-r10,lr}    @push 	
  
loop:
     LDMIA r0!,{r3-r10}
    @ PLD [r0,#0]
     ror r3,r3,#24
	  ror r4,r4,#24
	  ror r5,r5,#24
	  ror r6,r6,#24
	  ror r7,r7,#24
	  ror r8,r8,#24
	  ror r9,r9,#24
	  ror r10,r10,#24
  
     STMIA r1!,{r3-r10}

     @loop condition	  
     SUB r2,r2,#8 
     CMP r2,#0
     BNE loop             
             
     LDMIA SP!,{r4-r10,pc}    @pop


@input parameter
@r0:src addr
@r1:dst addr
@r2:src_w,rect_w,rect_h
ENTRY(format_convert_with_crop)

     STMDB SP!, {r4-r12,lr}    @push 
	  
	  LDR	lr,[r2]					 @src_w->lr
     LDR r3,[r2,#8]			    @rect_h->r3
	  LDR r2,[r2,#4]				 @rect_w->r2
     SUB lr,lr,r2					 @(src_w-rect_w)->lr
	  LSL lr,lr,#2 
     MOV r12,r2                @save inner loop varaible to r12

outer:
	  MOV r2,r12
inner:
     LDMIA r0!,{r4-r11}
    @ PLD [r0,#0]   
	  ror r4,r4,#24
	  ror r5,r5,#24
	  ror r6,r6,#24
	  ror r7,r7,#24
	  ror r8,r8,#24
	  ror r9,r9,#24
	  ror r10,r10,#24
	  ror r11,r11,#24
  
     STMIA r1!,{r4-r11}

     @inner condition	  
     SUB r2,r2,#8 
     CMP r2,#0
     BNE inner

     ADD r0,r0,lr            @pointer to next line start address

     @outer condition             
     SUB r3,r3,#1
	  CMP r3,#0
     BNE outer
             
     LDMIA SP!,{r4-r12,pc}    @pop


@input parameter
@r0:src addr
@r1:dst addr
@r2:(src_w-rect_w)*4,rect_w',rect_w'',rect_h,
ENTRY(format_convert_with_crop_non_8w)

     STMDB SP!, {r4-r12,lr}    @push 
	  
	  LDR	lr,[r2]					 @(src_w-rect_w)*4->lr
     LDR r3,[r2,#12]			    @rect_h->r3
     LDR r12,[r2,#8]           @rect_w''->r12	 
	  LDR r2,[r2,#4]				 @rect_w'->r2   
     SUB SP,SP,#4
     STR r2,[SP,#0]

outer1:
     LDR r2,[SP]
inner1:
     LDMIA r0!,{r4-r11}
    @ PLD [r0,#0]   
	  ror r4,r4,#24
	  ror r5,r5,#24
	  ror r6,r6,#24
	  ror r7,r7,#24
	  ror r8,r8,#24
	  ror r9,r9,#24
	  ror r10,r10,#24
	  ror r11,r11,#24
  
     STMIA r1!,{r4-r11}

     @inner1 condition	  
     SUB r2,r2,#8 
     CMP r2,#0
     BNE inner1
              @1
     CMP r12,#1
     BNE REM_2
     LDR r4,[r0]
     ROR r4,r4,#24
     STR r4,[r1]
     ADD r0,r0,#4
     ADD r1,r1,#4
     B inner_end
REM_2:
             @2
	  CMP r12,#2
     BNE REM_3
     LDMIA r0!,{r4,r5}
     ROR r4,r4,#24
     ROR r5,r5,#24
     STMIA r1!,{r4,r5}
     B inner_end
REM_3:
       	     @3
	  CMP r12,#3
     BNE REM_4
     LDMIA r0!,{r4-r6}
     ROR r4,r4,#24
     ROR r5,r5,#24
     ROR r6,r6,#24
     STMIA r1!,{r4-r6}
     B inner_end  
REM_4:
       	     @4
	  CMP r12,#4
     BNE REM_5
     LDMIA r0!,{r4-r7}
     ROR r4,r4,#24
     ROR r5,r5,#24
     ROR r6,r6,#24
     ROR r7,r7,#24
     STMIA r1!,{r4-r7}
     B inner_end  
REM_5:
       	     @5
	  CMP r12,#5
     BNE REM_6
     LDMIA r0!,{r4-r8}
     ROR r4,r4,#24
     ROR r5,r5,#24
     ROR r6,r6,#24
     ROR r7,r7,#24
     ROR r8,r8,#24
     STMIA r1!,{r4-r8}
     B inner_end    
REM_6:
       	     @6
	  CMP r12,#6
     BNE REM_7
     LDMIA r0!,{r4-r9}
     ROR r4,r4,#24
     ROR r5,r5,#24
     ROR r6,r6,#24
     ROR r7,r7,#24
     ROR r8,r8,#24
     ROR r9,r9,#24
     STMIA r1!,{r4-r9}
     B inner_end  
REM_7:
       	     @7
	  CMP r12,#7
     BNE inner_end  
     LDMIA r0!,{r4-r10}
     ROR r4,r4,#24
     ROR r5,r5,#24
     ROR r6,r6,#24
     ROR r7,r7,#24
     ROR r8,r8,#24
     ROR r9,r9,#24
     ROR r10,r10,#24
     STMIA r1!,{r4-r10}       

inner_end:
     ADD r0,r0,lr            @pointer to next line start address

     @outer1 condition             
     SUB r3,r3,#1
	  CMP r3,#0
     BNE outer1
     ADD SP,SP,#4
     LDMIA SP!,{r4-r12,pc}    @pop

