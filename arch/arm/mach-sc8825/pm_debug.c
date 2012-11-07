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
#include <mach/globalregs.h>
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
