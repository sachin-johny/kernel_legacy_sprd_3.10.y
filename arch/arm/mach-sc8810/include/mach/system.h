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

void cpu_idle_asm(void);

void arch_idle(void)
{
#ifdef CONFIG_SC8810_IDLE
	cpu_idle_asm();
	cpu_do_idle();
#endif
}


static inline void arch_reset(char mode, const char *cmd)
{
	/* our chip reset code */
}
