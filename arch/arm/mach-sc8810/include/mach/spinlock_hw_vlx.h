#ifndef __LINUX_SPINLOCK_HW_VLX_H
#define __LINUX_SPINLOCK_HW_VLX_H

#include <linux/typecheck.h>
#include <linux/preempt.h>
#include <linux/linkage.h>
#include <linux/compiler.h>
#include <linux/thread_info.h>
#include <linux/kernel.h>
#include <linux/stringify.h>
#include <linux/bottom_half.h>

#include <asm/system.h>


#define __HW_LOCK_IRQSAVE(lock, flags) \
  do { hw_local_irq_save(flags); __LOCK(lock); } while (0)

#define _hw_spin_lock_irqsave(lock, flags)		__HW_LOCK_IRQSAVE(lock, flags)

#define hw_spin_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		_hw_spin_lock_irqsave(lock, flags);	\
	} while (0)




#define __HW_UNLOCK_IRQRESTORE(lock, flags) \
  do { hw_local_irq_restore(flags); __UNLOCK(lock); } while (0)

#define _hw_spin_unlock_irqrestore(lock, flags)	__HW_UNLOCK_IRQRESTORE(lock, flags)


#define hw_spin_unlock_irqrestore(lock, flags)		\
	do {						\
		typecheck(unsigned long, flags);	\
		_hw_spin_unlock_irqrestore(lock, flags);	\
	} while (0)


#endif
