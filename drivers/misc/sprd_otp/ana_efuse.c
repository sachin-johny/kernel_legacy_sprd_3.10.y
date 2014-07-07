/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* IMPORTANT:
 *
 * TODO:
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <mach/arch_lock.h>

#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>

#include "sprd_otp.h"
#include "__regs_ana_efuse.h"

#define ANA_REGS_EFUSE_BASE             ( SPRD_ADISLAVE_BASE + 0x200 )

#define EFUSE_BLOCK_MAX                 ( 32 )
#define EFUSE_BLOCK_WIDTH               ( 8 )	/* bit counts */

static DEFINE_MUTEX(adie_efuse_mtx);
static void adie_efuse_lock(void)
{
	mutex_lock(&adie_efuse_mtx);
}

static void adie_efuse_unlock(void)
{
	mutex_unlock(&adie_efuse_mtx);
}

static void adie_efuse_reset(void)
{
}

static void __adie_efuse_power_on(void)
{
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_EFUSE_EB);

	/* FIXME: sclk always on or not ? */
}

static void __adie_efuse_power_off(void)
{
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_EFUSE_EB);
}

static __inline int __adie_efuse_wait_clear(u32 bits)
{
	int ret = 0;
	unsigned long timeout;

	pr_debug("wait %x\n", sci_adi_read(ANA_REG_EFUSE_STATUS));

	/* wait for maximum of 3000 msec */
	timeout = jiffies + msecs_to_jiffies(3000);
	while (sci_adi_read(ANA_REG_EFUSE_STATUS) & bits) {
		if (time_after(jiffies, timeout)) {
			WARN_ON(1);
			ret = -ETIMEDOUT;
			break;
		}
		cpu_relax();
	}
	return ret;
}

static u32 adie_efuse_read(int blk_index)
{
	u32 val = 0;

	pr_debug("adie efuse read %d\n", blk_index);
	adie_efuse_lock();
	__adie_efuse_power_on();
	/* enable adie_efuse module clk and power before */

	/* FIXME: set read timing, why 0x20? */
	sci_adi_raw_write(ANA_REG_EFUSE_RD_TIMING_CTRL,
			  BITS_EFUSE_RD_TIMING(0x20));

	sci_adi_raw_write(ANA_REG_EFUSE_BLOCK_INDEX,
			  BITS_READ_WRITE_INDEX(blk_index));
	sci_adi_raw_write(ANA_REG_EFUSE_MODE_CTRL, BIT_RD_START);

	if (IS_ERR_VALUE(__adie_efuse_wait_clear(BIT_READ_BUSY)))
		goto out;

	val = sci_adi_read(ANA_REG_EFUSE_DATA_RD);

out:
	__adie_efuse_power_off();
	adie_efuse_unlock();
	return val;
}

static struct sprd_otp_operations adie_efuse_ops = {
	.reset = adie_efuse_reset,
	.read = adie_efuse_read,
};

u32 __adie_efuse_read(int blk_index)
{
	return adie_efuse_ops.read(blk_index);
}

EXPORT_SYMBOL_GPL(__adie_efuse_read);

static ssize_t adie_efuse_block_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "adie efuse blocks: %uX%ubits\n",
		       EFUSE_BLOCK_MAX, EFUSE_BLOCK_WIDTH);
}

static ssize_t adie_efuse_block_dump(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int idx;
	char *p = buf;

	p += sprintf(p, "adie efuse blocks dump:\n");
	for (idx = 0; idx < EFUSE_BLOCK_MAX; idx++) {
		p += sprintf(p, "[%02d] %08x\n", idx, __adie_efuse_read(idx));
	}
	return p - buf;
}

static DEVICE_ATTR(block, S_IRUGO, adie_efuse_block_show, NULL);
static DEVICE_ATTR(dump, S_IRUGO, adie_efuse_block_dump, NULL);

int __init sprd_adie_efuse_init(void)
{
	int ret;
	void *dev;
	dev =
	    sprd_otp_register("sprd_adie_efuse_otp", &adie_efuse_ops,
			      EFUSE_BLOCK_MAX, EFUSE_BLOCK_WIDTH / 8);
	if (IS_ERR_OR_NULL(dev))
		return PTR_ERR(dev);

	ret = device_create_file(dev, &dev_attr_block);
	ret |= device_create_file(dev, &dev_attr_dump);
	if (ret)
		return ret;

	return 0;
}
