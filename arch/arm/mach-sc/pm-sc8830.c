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

void sc_pm_init(void)
{

}
