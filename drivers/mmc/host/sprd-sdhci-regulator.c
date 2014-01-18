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
#include "sprd-sdhci-regulator.h"

#define SPRD_SDHCI_REGULATOR_SUPPLY(id, v) \
		REGULATOR_SUPPLY(#v, "sprd-sdhci."#id)

#define SPRD_SDHCI_REGULATOR_INIT_DATA(_id, _v, _supply, _min_uV, _max_uV) { \
		.supply_regulator = _supply, \
		.constraints = { \
			.name = #_v#_id, \
			.min_uV = (_min_uV), \
			.max_uV = (_max_uV), \
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY, \
			.valid_ops_mask =	REGULATOR_CHANGE_STATUS |REGULATOR_CHANGE_VOLTAGE |REGULATOR_CHANGE_MODE, \
		}, \
		.consumer_supplies	= &sprd_consumer_supplies_##_v[_id], \
		.num_consumer_supplies = 1, \
	}

#define SPRD_SDHCI_REGULATOR_DESC(_id, _name) { \
		.name = #_name#_id, \
		.id = (_id), \
		.ops = &sprd_sdhci_regulator_ops, \
		.type = REGULATOR_VOLTAGE, \
		.n_voltages = 4, \
		.owner = THIS_MODULE, \
	}

#define SPRD_SDHCI_REGULATOR_DEFINE_INFO(v) \
	static struct regulator_consumer_supply sprd_consumer_supplies_##v[] = { \
		SPRD_SDHCI_REGULATOR_SUPPLY(0, v), \
		SPRD_SDHCI_REGULATOR_SUPPLY(1, v), \
		SPRD_SDHCI_REGULATOR_SUPPLY(2, v), \
		SPRD_SDHCI_REGULATOR_SUPPLY(3, v), \
	}; \
	static struct regulator_init_data sprd_init_data_##v[] =  { \
		SPRD_SDHCI_REGULATOR_INIT_DATA(0, v, NULL, 1800 * 1000, 3000 * 1000), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(1, v, NULL, 0, 0), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(2, v, NULL, 0, 0), \
		SPRD_SDHCI_REGULATOR_INIT_DATA(3, v, "vddemmccore", 1200 * 1000, 1800 * 1000), \
	}; \
	static struct regulator_desc sprd_regulator_desc_##v[] = { \
		SPRD_SDHCI_REGULATOR_DESC(0, v), \
		SPRD_SDHCI_REGULATOR_DESC(1, v), \
		SPRD_SDHCI_REGULATOR_DESC(2, v), \
		SPRD_SDHCI_REGULATOR_DESC(3, v), \
	};

#define SPRD_SDHCI_REGULATOR_DECLARE_INFO(id, v) do { \
		memset(&config_##v, 0, sizeof(struct regulator_config)); \
		desc_##v = &sprd_regulator_desc_##v[(id)]; \
		config_##v.init_data = &sprd_init_data_##v[(id)]; \
	} while(0)

static const int sprd_sdhci_regulator_voltage_level[4][4] = {
	{1800 * 1000, 2500 * 1000, 2800 * 1000, 3000 * 1000}, // vddsd
	{},
	{},
	{1200 * 1000, 1300 * 1000, 1500 * 1000, 1800 * 1000}, // vddemmcio
};

static int sprd_sdhci_regulator_enable(struct regulator_dev *rdev) {
	int retval;
	struct regulator *regulator = rdev_get_drvdata(rdev);
	regulator_notifier_call_chain(rdev, REGULATOR_EVENT_ENABLE, rdev);
	retval = regulator ? regulator_enable(regulator) : 0;
	if(!retval)
		return retval;
	regulator_notifier_call_chain(rdev, REGULATOR_EVENT_DISABLE, rdev);
	return retval;
}

static int sprd_sdhci_regulator_disable(struct regulator_dev *rdev) {
	struct regulator *regulator = rdev_get_drvdata(rdev);
	return regulator ? regulator_disable(regulator) : 0;
}

static int sprd_sdhci_regulator_is_enabled(struct regulator_dev *rdev) {
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
	if(rdev->supply) {
		uV = 3000 * 1000;
		retval = regulator_set_voltage(rdev->supply, uV, uV);
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
	return regulator_set_voltage(regulator, uV, uV);
}

static int sprd_sdhci_regulator_get_voltage(struct regulator_dev *rdev) {
	struct regulator *regulator = rdev_get_drvdata(rdev);
	return regulator_get_voltage(regulator);
}

static int sprd_sdhci_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode) {
	struct regulator *regulator = rdev_get_drvdata(rdev);
	return regulator_set_mode(regulator, mode);
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

struct regulator_dev *sprd_sdhci_regulator_init(struct platform_device *pdev, struct notifier_block *nb, const char *ext_vdd_name) {
	struct regulator *regulator_vmmc = NULL, *regulator_vqmmc = NULL;
	struct regulator_dev *rdev_vmmc, *rdev_vqmmc;
	struct regulator_desc *desc_vmmc, *desc_vqmmc;
	struct regulator_config config_vmmc, config_vqmmc;
	SPRD_SDHCI_REGULATOR_DEFINE_INFO(vmmc);
	SPRD_SDHCI_REGULATOR_DEFINE_INFO(vqmmc);
	SPRD_SDHCI_REGULATOR_DECLARE_INFO(pdev->id, vmmc);
	SPRD_SDHCI_REGULATOR_DECLARE_INFO(pdev->id, vqmmc);
	if(!ext_vdd_name)
		return NULL;
	config_vmmc.dev = config_vqmmc.dev = &pdev->dev;
	regulator_vqmmc = regulator_get(&pdev->dev, ext_vdd_name);
	if(IS_ERR_OR_NULL(regulator_vqmmc))
		return regulator_vqmmc;
	config_vmmc.driver_data =  config_vqmmc.driver_data = regulator_vqmmc;
	rdev_vmmc = regulator_register(desc_vmmc, &config_vmmc);
	if(IS_ERR_OR_NULL(rdev_vmmc))
		goto ERR_EXIT_VMMC;
	rdev_vqmmc = regulator_register(desc_vqmmc, &config_vqmmc);
	if(IS_ERR_OR_NULL(rdev_vqmmc))
		goto ERR_EXIT_VQMMC;
	regulator_vmmc = regulator_get(&pdev->dev, "vmmc");
	if(IS_ERR_OR_NULL(regulator_vmmc))
		goto ERR_EXIT_GET_VMMC;
	if(regulator_register_notifier(regulator_vmmc, nb) < 0)
		goto ERR_EXIT_NOTIFIER;
	regulator_put(regulator_vmmc);
	return rdev_vmmc;
ERR_EXIT_NOTIFIER:
	printk("regulator_register_notifier vmmc failed\n");
	regulator_put(regulator_vmmc);
ERR_EXIT_GET_VMMC:
	printk("regulator_get vqmmc failed\n");
	regulator_unregister(rdev_vqmmc);
ERR_EXIT_VQMMC:
	printk("regulator_register vqmmc failed\n");
	regulator_unregister(rdev_vmmc);
ERR_EXIT_VMMC:
	printk("regulator_register vmmc failed\n");
	if(regulator_vqmmc)
		regulator_put(regulator_vqmmc);
	return NULL;
}

