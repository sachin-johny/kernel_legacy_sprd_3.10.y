/* arch/arm/mach-sc8800g/include/mach/system.h
 *
 * Copyright (C) 2010 Spreadtrum
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

#include <mach/hardware.h>
#include <mach/test.h>
static inline void arch_idle(void)
{
/*
    u32 t0, t1, delta;
    idle_loops++;
    t0 = get_sys_cnt();
*/
    cpu_do_idle();
/*
    t1 = get_sys_cnt();
    delta = t1 - t0;
    idle_time += delta;
*/
}

static inline void arch_reset(char mode, const char *cmd)
{
	/* our chip reset code */
}
