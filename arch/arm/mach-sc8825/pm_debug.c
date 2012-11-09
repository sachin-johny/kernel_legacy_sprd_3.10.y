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

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <asm/irqflags.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <mach/pm_debug.h>
#include <linux/debugfs.h>

void inc_irq(int irq)
{

}

#include <mach/system.h>
#include <mach/adi.h>
#include <mach/regs_ana_glb.h>
static void sc8825_power_off(void)
{
	/*ture off all modules ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL1,	0x5555);
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL0,	0x5555);

	/*ture off all system cores ldo*/
	sci_adi_clr(ANA_REG_GLB_LDO_PD_RST, 0x3ff);
	sci_adi_set(ANA_REG_GLB_LDO_PD_SET, 0x3ff);
}

static void sc8825_machine_restart(char mode, const char *cmd)
{
	/* Flush the console to make sure all the relevant messages make it
	 * out to the console drivers */
	mdelay(500);

	/* Disable interrupts first */
	local_irq_disable();
	local_fiq_disable();

	/*
	  * FIXME: Do not turn off cache before ldrex/strex!
	  */

	/*
	 * Now call the architecture specific reboot code.
	 */
	arch_reset(mode, cmd);

	/*
	 * Whoops - the architecture was unable to reboot.
	 * Tell the user!
	 */
	mdelay(1000);
	printk("Reboot failed -- System halted\n");
	while (1);
}

static int __init sc8825_power_init(void)
{
	pm_power_off = sc8825_power_off;
	arm_pm_restart = sc8825_machine_restart;
	pr_info("power off %p, restart %p\n", pm_power_off, arm_pm_restart);
	return 0;
}
arch_initcall(sc8825_power_init);

#include <linux/regulator/consumer.h>
int __init sc8825_ldo_slp_init(void)
{
	int i;

	static struct {
		const char *vdd_name;
		/*
		  * 0: slp pd disable, very light loads when in STANDBY mode
		  * 1: slp pd enable, this is chip default config for most of LDOs
		  */
		bool pd_en;
	} ldo_slp_config[] __initdata = {
		{"vddsim0",		0},
		{"vddsim1",		0},
		{"avddvb",		0},
	};

	for (i = 0; i < ARRAY_SIZE(ldo_slp_config); i++) {
		if (0 == ldo_slp_config[i].pd_en) {
			struct regulator *ldo =
			    regulator_get(NULL, ldo_slp_config[i].vdd_name);
			if (!WARN_ON(IS_ERR(ldo))) {
				regulator_set_mode(ldo, REGULATOR_MODE_STANDBY);
				regulator_put(ldo);
			}
			pr_info("%s slp pd disable\n", ldo_slp_config[i].vdd_name);
		}
	}
	return 0;
}

late_initcall(sc8825_ldo_slp_init);
