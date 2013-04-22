/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/pm.h>
#include <linux/delay.h>
#include <asm/system_misc.h>

#include <mach/sci.h>
#include <mach/adi.h>
#include <mach/system.h>
#include <mach/sci_glb_regs.h>

void *iram_start = 0;

void check_ldo(void)
{
}

void check_pd(void)
{
}

unsigned int sprd_irq_pending(void)
{
	return 0;
}

int sprd_cpu_deep_sleep(unsigned int cpu)
{
	return 0;
}

static void sc8830_power_off(void)
{
	/*turn off all modules's ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL, 0x1fff);

	/*turn off system core's ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_DCDC_PD_RTCCLR, 0x0);
	sci_adi_raw_write(ANA_REG_GLB_LDO_DCDC_PD_RTCSET, 0X7fff);
}

static void sc8830_machine_restart(char mode, const char *cmd)
{
	local_irq_disable();
	local_fiq_disable();

	arch_reset(mode, cmd);

	mdelay(1000);

	printk("reboot failed!\n");

	while (1);
}

void sc_pm_init(void)
{
	pm_power_off   = sc8830_power_off;
	arm_pm_restart = sc8830_machine_restart;
}
