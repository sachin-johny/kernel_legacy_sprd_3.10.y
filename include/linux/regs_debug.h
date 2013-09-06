#ifndef _REGS_DEBUG_
#define _REGS_DEBUG_

#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/jiffies.h>

struct sprd_debug_regs_access{
	unsigned int vaddr;
	unsigned int value;
	unsigned int pc;
	unsigned long time;
	unsigned int status;
};

#define sprd_debug_regs_read_start(a)	({u32 cpu_id, lr;		\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr)			\
			:						\
			: "memory");					\
		sprd_debug_last_regs_access[cpu_id].value = 0;		\
		sprd_debug_last_regs_access[cpu_id].vaddr = a;		\
		sprd_debug_last_regs_access[cpu_id].pc = lr;		\
		sprd_debug_last_regs_access[cpu_id].time = jiffies;	\
		sprd_debug_last_regs_access[cpu_id].status = 0;		\
		})

#define sprd_debug_regs_write_start(v, a)	({u32 cpu_id, lr;	\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr)			\
			:						\
			: "memory");					\
		sprd_debug_last_regs_access[cpu_id].value = (v);	\
		sprd_debug_last_regs_access[cpu_id].vaddr = (a);	\
		sprd_debug_last_regs_access[cpu_id].pc = lr;		\
		sprd_debug_last_regs_access[cpu_id].time = jiffies;	\
		sprd_debug_last_regs_access[cpu_id].status = 0;		\
		})

#define sprd_debug_regs_access_done()	({u32 cpu_id, lr;		\
		asm volatile(						\
			"	mrc	p15, 0, %0, c0, c0, 5\n"	\
			"	ands %0, %0, #0xf\n"			\
			"	mov %1, lr\n"				\
			: "=&r" (cpu_id), "=&r" (lr)			\
			:						\
			: "memory");					\
		sprd_debug_last_regs_access[cpu_id].time = jiffies;	\
		sprd_debug_last_regs_access[cpu_id].status = 1;		\
		})

#endif
