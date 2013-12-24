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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <mach/regulator.h>
#include "sdhci-sprd-regulator.h"

#define SPRD_SDHCI_REGULATOR_SUPPLY(id) { \
		REGULATOR_SUPPLY("vmmc", "sprd-sdhci."#id), \
		REGULATOR_SUPPLY("vqmmc", "sprd-sdhci."#id), \
	}

#define SPRD_SDHCI_REGULATOR_INIT_DATA(_id, _supply, _min_uV, _max_uV) { \
		.supply_regulator = #_supply, \
		.constraints = { \
			.min_uV = (_min_uV), \
			.max_uV = (_max_uV), \
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY, \
			.valid_ops_mask =	REGULATOR_CHANGE_STATUS |REGULATOR_CHANGE_VOLTAGE |REGULATOR_CHANGE_MODE, \
		}, \
		.consumer_supplies	= sprd_consumer_supplies[_id], \
		.num_consumer_supplies = ARRAY_SIZE(sprd_consumer_supplies[_id]), \
	}

#define SPRD_SDHCI_REGULATOR_DESC(_id, _name) { \
		.name = #_name, \
		.id = (_id), \
		.ops = &sprd_sdhci_regulator_ops, \
		.type = REGULATOR_VOLTAGE, \
		.n_voltages = 4, \
		.owner = THIS_MODULE, \
	}

#define SPRD_SDHCI_REGULATOR_INFO() \
	struct regulator_consumer_supply sprd_consumer_supplies[4][2] = { \
		SPRD_SDHCI_REGULATOR_SUPPLY(0), \
		SPRD_SDHCI_REGULATOR_SUPPLY(1), \
		SPRD_SDHCI_REGULATOR_SUPPLY(2), \
		SPRD_SDHCI_REGULATOR_SUPPLY(3), \
	}; \
	struct regulator_init_data sprd_init_data[] =  { \
		SPRD_SDHCI_REGULATOR_INIT_DATA(0, vddsd, 1800 * 1000, 3000 * 1000), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(1, NULL, 0, 0), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(2, NULL, 0, 0), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(3, vddemmcio, 1200 * 1000, 1800 * 1000), \
	}; \
	static struct regulator_desc sprd_regulator_desc[] = { \
		SPRD_SDHCI_REGULATOR_DESC(0, vmmc), \
		SPRD_SDHCI_REGULATOR_DESC(1, vmmc), \
		SPRD_SDHCI_REGULATOR_DESC(2, vmmc), \
		SPRD_SDHCI_REGULATOR_DESC(3, vmmc), \
	};

static const int sprd_sdhci_regulator_voltage_level[4][4] = {
	{1800 * 1000, 2500 * 1000, 2800 * 1000, 3000 * 1000}, // vddsd
	{},
	{},
	{1200 * 1000, 1300 * 1000, 1500 * 1000, 1800 * 1000}, // vddemmcio
};

static int sprd_sdhci_regulator_enable(struct regulator_dev *rdev) {
	struct regulator *regulator = rdev_get_drvdata(rdev);
	return regulator ? regulator_enable(regulator) : 0;
}

static int sprd_sdhci_regulator_disable(struct regulator_dev *rdev) {
	struct regulator *regulator = rdev_get_drvdata(rdev);
	return regulator ? regulator_disable(regulator) : 0;
}

static int sprd_sdhci_regulator_is_enabled(struct regulator_dev *rdev) {
	if(!rdev->supply)
		return -EINVAL;
	return rdev->use_count > 0;
}

static int sprd_sdhci_regulator_set_voltage(struct regulator_dev *rdev, int min_uV,
			   int max_uV, unsigned *selector) {
	int uV;
	int i;
	int start;
	int retval;
	int id = rdev_get_id(rdev);
	struct regulator *regulator = rdev_get_drvdata(rdev);
	if(!rdev->supply)
		return -EINVAL;
	if (regulator) {
		uV = 3000 * 1000;
		retval = regulator_set_voltage(regulator, uV, uV);
		if(retval < 0)
			return retval;
	}
	start = sprd_sdhci_regulator_voltage_level[id][0];
	for(i = 0; i < ARRAY_SIZE(sprd_sdhci_regulator_voltage_level[0]); i++) {
		if(min_uV <= sprd_sdhci_regulator_voltage_level[id][i] || min_uV <= (sprd_sdhci_regulator_voltage_level[id][i] + start) /2)
			break;
		start = sprd_sdhci_regulator_voltage_level[id][i];
	}
	if(i == ARRAY_SIZE(sprd_sdhci_regulator_voltage_level[0]))
		i--;
	uV = sprd_sdhci_regulator_voltage_level[id][i];
	if(selector)
		*selector = i;
	return regulator_set_voltage(rdev->supply, uV, uV);
}

static int sprd_sdhci_regulator_get_voltage(struct regulator_dev *rdev) {
	if(!rdev->supply)
		return -EINVAL;
	return regulator_get_voltage(rdev->supply);
}

static int sprd_sdhci_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode) {
	int retval;
	struct regulator *regulator = rdev_get_drvdata(rdev);
	if(!rdev->supply)
		return -EINVAL;
	if (regulator) {
		retval = regulator_set_mode(regulator, mode);
		if(retval < 0)
			return retval;
	}
	return regulator_set_mode(rdev->supply, mode);
}

static int sprd_sdhci_regulator_list_voltage(struct regulator_dev *rdev,
				      unsigned selector) {
	int id = rdev_get_id(rdev);
	if(selector >= ARRAY_SIZE(sprd_sdhci_regulator_voltage_level[0]))
		return -EINVAL;
	return sprd_sdhci_regulator_voltage_level[id][selector];
}

static struct regulator_ops sprd_sdhci_regulator_ops = {
	.is_enabled = sprd_sdhci_regulator_is_enabled,
	.enable = sprd_sdhci_regulator_enable,
	.disable = sprd_sdhci_regulator_disable,
	.set_voltage = sprd_sdhci_regulator_set_voltage,
	.get_voltage = sprd_sdhci_regulator_get_voltage,
	.set_mode = sprd_sdhci_regulator_set_mode,
	.list_voltage = sprd_sdhci_regulator_list_voltage,
};

void sprd_sdhci_regulator_init(struct platform_device *pdev, const char *ext_vdd_name) {
	struct regulator_dev *regulator;
	struct regulator_desc *desc;
	struct regulator_config config;
	struct regulator_init_data *init_data;
	SPRD_SDHCI_REGULATOR_INFO();
	memset(&config, 0, sizeof(struct regulator_config));
	desc = &sprd_regulator_desc[pdev->id];
	init_data = &sprd_init_data[pdev->id];
	config.init_data = init_data;
	config.dev = &pdev->dev;
	if(ext_vdd_name) {
		config.driver_data = regulator_get(&pdev->dev, ext_vdd_name);
		if (IS_ERR(config.driver_data)) {
			config.driver_data = NULL;
		}
	}
	regulator = regulator_register(desc, &config);
	if(IS_ERR_OR_NULL(regulator))
		printk("regulator_register failed\n");
}

