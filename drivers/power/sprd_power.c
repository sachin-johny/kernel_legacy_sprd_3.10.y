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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/hrtimer.h>
#ifdef	CONFIG_SPRD_8825_POWER
#include "sprd_8825_charge.h"
#else
#include "sprd_8810_charge.h"
#endif
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/adc.h>
#include <mach/gpio.h>
#include <linux/device.h>

#include <linux/slab.h>
#include <linux/jiffies.h>

extern int sci_adc_get_value(unsigned chan, int scale);

static struct sprd_battery_data *battery_data;

#ifdef CONFIG_NOTIFY_BY_USB
#include <mach/usb.h>
static int plugin_callback(int usb_cable, void *data);
static int plugout_callback(int usb_cable, void *data);
/*
 * we need usb module to detect
 * 1, plug in/out
 * 2, whether it is usb cable
 */
static struct usb_hotplug_callback power_cb = {
	.plugin = plugin_callback,
	.plugout = plugout_callback,
	.data = NULL,
};
#endif

int sprd_get_adc_cal_type(void)
{
	return battery_data ? battery_data->adc_cal_updated : 0;
}

uint16_t sprd_get_adc_to_vol(uint16_t data)
{
	return sprd_bat_adc_to_vol(battery_data, data);
}

uint32_t sprd_get_vbat_voltage(void)
{
	return battery_data ? battery_data->voltage : 0;
}
EXPORT_SYMBOL(sprd_get_vbat_voltage);

static int sprd_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
						      struct sprd_battery_data,
						      ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (likely(psy->type == POWER_SUPPLY_TYPE_MAINS)) {
			val->intval = data->ac_online ? 1 : 0;
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sprd_usb_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
						      struct sprd_battery_data,
						      usb);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->usb_online ? 1 : 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sprd_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
						      struct sprd_battery_data,
						      battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/* suppose battery always online */
		if (data->charging) {
			if (data->capacity >= 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = (data->capacity > 100) ? 100 : data->capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = data->voltage * 1000;
		break;
#ifdef CONFIG_BATTERY_TEMP_DECT
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = data->temp;
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property sprd_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
#ifdef CONFIG_BATTERY_TEMP_DECT
	POWER_SUPPLY_PROP_TEMP,
#endif
};

static enum power_supply_property sprd_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property sprd_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static ssize_t sprd_set_caliberate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);
static ssize_t sprd_show_caliberate(struct device *dev,
				    struct device_attribute *attr, char *buf);

#define SPRD_CALIBERATE_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR | S_IWGRP, },  \
	.show = sprd_show_caliberate,                  \
	.store = sprd_set_caliberate,                              \
}
#define SPRD_CALIBERATE_ATTR_RO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, },  \
	.show = sprd_show_caliberate,                  \
}
#define SPRD_CALIBERATE_ATTR_WO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP, },  \
	.store = sprd_set_caliberate,                              \
}
static struct device_attribute sprd_caliberate[] = {
	SPRD_CALIBERATE_ATTR_RO(real_time_voltage),
	SPRD_CALIBERATE_ATTR_WO(stop_charge),
	SPRD_CALIBERATE_ATTR_RO(real_time_current),
	SPRD_CALIBERATE_ATTR_WO(battery_0),
	SPRD_CALIBERATE_ATTR_WO(battery_1),
	SPRD_CALIBERATE_ATTR(hw_switch_point),
	SPRD_CALIBERATE_ATTR_RO(charger_voltage),
};

enum sprd_charge_prop {
	BATTERY_VOLTAGE = 0,
	STOP_CHARGE,
	BATTERY_NOW_CURRENT,
	BATTERY_0,
	BATTERY_1,
	HW_SWITCH_POINT,
	CHARGER_VOLTAGE,
};
extern uint16_t adc_voltage_table[2][2];
static ssize_t sprd_set_caliberate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long flag;
	unsigned long set_value;
	const ptrdiff_t off = attr - sprd_caliberate;

	set_value = simple_strtoul(buf, NULL, 10);
	pr_info("battery calibrate value %d %lu\n", off, set_value);

	switch (off) {
	case STOP_CHARGE:
		battery_data->usb_online = 0;
		battery_data->ac_online = 0;
		break;
	case BATTERY_0:
		spin_lock_irqsave(&battery_data->lock, flag);
		if (battery_data->adc_cal_updated != ADC_CAL_TYPE_NV) {
			adc_voltage_table[0][1] = set_value & 0xffff;
			adc_voltage_table[0][0] = (set_value >> 16) & 0xffff;
		}
		spin_unlock_irqrestore(&battery_data->lock, flag);
		break;
	case BATTERY_1:
		spin_lock_irqsave(&battery_data->lock, flag);
		if (battery_data->adc_cal_updated != ADC_CAL_TYPE_NV) {
			adc_voltage_table[1][1] = set_value & 0xffff;
			adc_voltage_table[1][0] = (set_value >> 16) & 0xffff;
			sprd_vol_to_percent(battery_data, 0, 1);
			battery_data->adc_cal_updated = ADC_CAL_TYPE_NV;
		}
		spin_unlock_irqrestore(&battery_data->lock, flag);
		break;
	case HW_SWITCH_POINT:
		battery_data->hw_switch_point = set_value;
		sprd_set_sw(battery_data, battery_data->hw_switch_point);
		break;
	default:
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t sprd_show_caliberate(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - sprd_caliberate;
	int adc_value;
	int voltage;
	uint32_t now_current;

	switch (off) {
	case BATTERY_VOLTAGE:
		adc_value = sci_adc_get_value(ADC_CHANNEL_VBAT, false);
		if (adc_value < 0)
			voltage = 0;
		else
			voltage = sprd_bat_adc_to_vol(battery_data, adc_value);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", voltage);
		break;
	case BATTERY_NOW_CURRENT:
		if (battery_data->charging) {
			now_current = get_vprog_value(battery_data);
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       now_current);
		} else {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
				       "discharging");
		}
		break;
	case HW_SWITCH_POINT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       battery_data->hw_switch_point);
		break;
	case CHARGER_VOLTAGE:
		if (battery_data->charging) {
            #ifdef CONFIG_ARCH_SC7710
            adc_value = sci_adc_get_value(ADC_CHANNEL_VCHGBG, false);
            #else
			adc_value = sci_adc_get_value(ADC_CHANNEL_VCHG, false);
            #endif
            if (adc_value < 0)
				voltage = 0;
			else
				voltage =
				    sprd_charger_adc_to_vol(battery_data,
							    adc_value);
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", voltage);
		} else {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
				       "discharging");
		}
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

static int sprd_creat_caliberate_attr(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sprd_caliberate); i++) {
		rc = device_create_file(dev, &sprd_caliberate[i]);
		if (rc)
			goto sprd_attrs_failed;
	}
	goto sprd_attrs_succeed;

sprd_attrs_failed:
	while (i--)
		device_remove_file(dev, &sprd_caliberate[i]);

sprd_attrs_succeed:
	return rc;
}

static int sprd_remove_caliberate_attr(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sprd_caliberate); i++) {
		device_remove_file(dev, &sprd_caliberate[i]);
	}
	return 0;
}

static inline int usb_connected(void)
{
	return gpio_get_value(battery_data->gpio);
}

static irqreturn_t sprd_battery_interrupt(int irq, void *dev_id)
{
	struct sprd_battery_data *data = dev_id;
	uint32_t charger_status;

	charger_status = usb_connected();
	if (charger_status) {
		if (sprd_charger_is_adapter(data)) {
			data->ac_online = 1;
			data->usb_online = 0;
		} else {
			data->usb_online = 1;
			data->ac_online = 0;
		}
		data->timer_freq = HZ / 10;
	} else {
		data->ac_online = 0;
		data->usb_online = 0;
		data->timer_freq = HZ;
		wake_lock_timeout(&(data->charger_plug_out_lock),
				  CONFIG_PLUG_WAKELOCK_TIME_SEC * HZ);
	}
	pr_info("charger interrupt: usb:%d, ac:%d\n", data->usb_online,
		data->ac_online);

	irq_set_irq_type(irq,
			 charger_status ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	mod_timer(&battery_data->battery_timer, jiffies + data->timer_freq);
	gpio_direction_input(data->gpio);
	return IRQ_HANDLED;
}

#ifdef CONFIG_NOTIFY_BY_USB
static int plugin_callback(int usb_cable, void *data)
{
	struct sprd_battery_data *d = battery_data;

	if (!d) {
		pr_warning("batttery_data is NULL!!\n");
		return 1;
	}

	d->ac_online = 0;
	d->usb_online = 0;

	if (usb_cable) {
		d->usb_online = 1;
	} else {
		if (sprd_charger_is_adapter(d)) {
			d->ac_online = 1;
		} else {
			d->usb_online = 1;
			pr_warning("unknown charger!\n");
		}
	}

	pr_info("charger plug in interrupt: usb:%d, ac:%d\n", d->usb_online,
		d->ac_online);

	return 0;
}

static int plugout_callback(int usb_cable, void *data)
{
	struct sprd_battery_data *d = battery_data;

	if (!d) {
		pr_warning("batttery_data is NULL!!\n");
		return 1;
	}

	d->ac_online = 0;
	d->usb_online = 0;

	pr_info("charger plug out interrupt happen\n");
	wake_lock_timeout(&(d->charger_plug_out_lock),
			  CONFIG_PLUG_WAKELOCK_TIME_SEC * HZ);

	return 0;
}
#endif

static int pluse_charging = 0;
static int pluse_charge_cnt = CHGMNG_PLUSE_TIMES;
static int hw_switch_update_cnt = CONFIG_AVERAGE_CNT;
static int stop_left_time = CHARGE_BEFORE_STOP;
static int32_t vprog_current = 0;
static int vchg_vol;
static int pre_usb_online = 0;
static int pre_ac_online = 0;

void enable_usb_charge(struct sprd_battery_data *battery_data)
{
	pluse_charge_cnt = CHGMNG_PLUSE_TIMES;
	hw_switch_update_cnt = CONFIG_AVERAGE_CNT;
	battery_data->charge_start_jiffies = get_jiffies_64();
	battery_data->charging = 1;
	pluse_charging = 0;
	sprd_set_chg_cur(SPRD_USB_CHG_CUR);
	battery_data->cur_type = SPRD_USB_CHG_CUR;
	sprd_set_sw(battery_data, battery_data->hw_switch_point);
	sprd_start_charge(battery_data);
	battery_data->in_precharge = 0;
}

void enable_ac_charge(struct sprd_battery_data *battery_data)
{
	pluse_charge_cnt = CHGMNG_PLUSE_TIMES;
	hw_switch_update_cnt = CONFIG_AVERAGE_CNT;
	battery_data->charge_start_jiffies = get_jiffies_64();
	sprd_set_chg_cur(SPRD_AC_CHG_CUR);
	battery_data->cur_type = SPRD_AC_CHG_CUR;
	battery_data->charging = 1;
	pluse_charging = 0;
	sprd_set_sw(battery_data, battery_data->hw_switch_point);
	sprd_start_charge(battery_data);
	battery_data->in_precharge = 0;
}

void charge_stop(struct sprd_battery_data *battery_data)
{
	sprd_stop_charge(battery_data);
	sprd_stop_recharge(battery_data);
	battery_data->charging = 0;
	pluse_charging = 0;
	battery_data->in_precharge = 0;
}

#ifdef	CONFIG_SPRD_8825_POWER
#define REG_SYST_VALUE                  (SPRD_SYSCNT_BASE + 0x0004)
static u32 sci_syst_read(void)
{
	u32 t = __raw_readl(REG_SYST_VALUE);
	while (t != __raw_readl(REG_SYST_VALUE))
		t = __raw_readl(REG_SYST_VALUE);
	return t;
}

u32 plus_charge_start_time = 0;

#endif

uint32_t vbat_capacity_loop_cnt = 0;
static void charge_handler(struct sprd_battery_data *battery_data, int in_sleep)
{
	uint32_t voltage = 0;
	uint32_t capacity = 0;
	int32_t adc_value = 0;
	int32_t vprog_value = 0;
	int usb_online = 0;
	uint32_t ac_online = 0;
	uint32_t ac_notify = 0;
	int usb_notify = 0;
	int battery_notify = 0;
	int32_t vchg_value;
#ifdef CONFIG_BATTERY_TEMP_DECT
	int32_t temp_value;
	int temp;
#endif
	uint64_t now_jiffies = 0;

	usb_online = battery_data->usb_online;
	ac_online = battery_data->ac_online;

	if (pre_ac_online != ac_online) {
		pre_ac_online = ac_online;
		ac_notify = 1;
		if (ac_online) {
			enable_ac_charge(battery_data);
		} else {
			charge_stop(battery_data);
		}
		battery_notify = 1;
	}

	if (pre_usb_online != usb_online) {
		pre_usb_online = usb_online;
		usb_notify = 1;
		if (usb_online) {
			enable_usb_charge(battery_data);
		} else {
			charge_stop(battery_data);
		}
		battery_notify = 1;
	}

	{
		adc_value = sci_adc_get_value(ADC_CHANNEL_VBAT, false);

		if (adc_value >= 0)
			put_vbat_value(battery_data, adc_value);

		if (adc_value < 0)
			goto out;
		adc_value = get_vbat_value(battery_data);

		vbat_capacity_loop_cnt++;	//10S update vbat capacity buffer
		vbat_capacity_loop_cnt %= CONFIG_AVERAGE_CNT;
		if (0 == vbat_capacity_loop_cnt) {
			put_vbat_capacity_value(adc_value);
		}

		voltage = sprd_bat_adc_to_vol(battery_data, adc_value);

		if (!battery_data->charging && (battery_data->in_precharge == 1)
		    && usb_online && (voltage < battery_data->precharge_start)) {
			enable_usb_charge(battery_data);
			battery_notify = 1;
		}

		if (!battery_data->charging && (battery_data->in_precharge == 1)
		    && ac_online && (voltage < battery_data->precharge_start)) {
			enable_ac_charge(battery_data);
			battery_notify = 1;
		}
	}

	if (usb_online || ac_online) {
		if (battery_data->charging) {
#ifdef CONFIG_SPRD_POWER
			vprog_value = sprd_get_vprog(battery_data);
			if (vprog_value < 0)
				goto out;
			vprog_current =
			    sprd_bat_adc_to_vol(battery_data, vprog_value);

			vprog_current =
			    sprd_adc_to_cur(battery_data, vprog_current);
#else
			vprog_current = sprd_get_chg_current(battery_data);
#endif
			put_vprog_value(battery_data, vprog_current);
			vprog_current = get_vprog_value(battery_data);

		}
        #ifdef  CONFIG_ARCH_SC7710
        vchg_value = sci_adc_get_value(ADC_CHANNEL_VCHGBG, false);
        #else
		vchg_value = sci_adc_get_value(ADC_CHANNEL_VCHG, false);
        #endif
        if (vchg_value < 0)
			goto out;
		vchg_vol = sprd_charger_adc_to_vol(battery_data, vchg_value);
		put_vchg_value(vchg_vol);
		vchg_vol = get_vchg_value();
		if (vchg_vol > battery_data->over_voltage) {
			printk(KERN_ERR "charger voltage too high\n");
			charge_stop(battery_data);
			battery_data->over_voltage_flag = 1;
			battery_notify = 1;
		}

		if (voltage > CHGMNG_OVER_CHARGE) {
			battery_notify = 1;
			charge_stop(battery_data);
			battery_data->in_precharge = 1;
			printk(KERN_ERR "vbat over %d \n", voltage);
		}
	}
	if (!battery_data->charging && !battery_data->in_precharge &&
	    (usb_online || ac_online) && battery_data->over_voltage_flag) {
		if (vchg_vol < battery_data->over_voltage_recovery) {
			printk(KERN_ERR "vbat OVP recovery %d \n", voltage);
			battery_notify = 1;
			battery_data->over_voltage_flag = 0;
			if (ac_online)
				enable_ac_charge(battery_data);
			else
				enable_usb_charge(battery_data);
		}
	}
#ifdef CONFIG_BATTERY_TEMP_DECT
	temp_value = sci_adc_get_value(ADC_CHANNEL_TEMP, false);
	if (temp_value < 0)
		goto out;
	put_temp_value(battery_data, temp_value);
	temp_value = get_temp_value(battery_data);

	temp = sprd_adc_to_temp(battery_data, temp_value);

	if (abs(battery_data->temp - temp) > 2) {
		battery_data->temp = temp;
		battery_notify = 1;
	}

	if (temp > OTP_OVER_HIGH || temp < OTP_OVER_LOW) {
		battery_data->over_temp_flag = 1;
		battery_notify = 1;
		printk(KERN_ERR "battery temperature out temp:%d\n", temp);
		charge_stop(battery_data);
	}
	if (!battery_data->charging && !battery_data->in_precharge &&
	    (usb_online || ac_online) && battery_data->over_temp_flag) {
		if (temp > OTP_RESUME_LOW || temp < OTP_RESUME_HIGH) {
			printk(KERN_ERR
			       "battery recovery temperature temp:%d\n", temp);
			battery_notify = 1;
			battery_data->over_temp_flag = 0;
			if (ac_online)
				enable_ac_charge(battery_data);
			else
				enable_usb_charge(battery_data);
		}
	}
#endif

	if (battery_data->charging) {
		if (!pluse_charging) {
			hw_switch_update_cnt--;
			if (hw_switch_update_cnt <= 0) {
				hw_switch_update_cnt = CONFIG_AVERAGE_CNT;

				if (voltage <= battery_data->precharge_end) {
					if (vprog_current < CC_CV_SWITCH_POINT) {
						battery_data->hw_switch_point =
						    sprd_adjust_sw(battery_data,
								   true);
					}
				} else {
					if (vprog_current <= CV_STOP_CURRENT) {
						pluse_charging = 1;
						stop_left_time =
						    CHARGE_BEFORE_STOP;
#ifdef	CONFIG_SPRD_8825_POWER
						plus_charge_start_time =
						    sci_syst_read();
						printk(KERN_ERR
						       "plus_charge_start_time...%d\n",
						       plus_charge_start_time);
#endif
					}
					if (voltage >
					    (battery_data->precharge_end +
					     10)) {
						battery_data->hw_switch_point =
						    sprd_adjust_sw(battery_data,
								   false);
					}
				}
			}
		} else {
#ifdef	CONFIG_SPRD_8825_POWER
			if ((sci_syst_read() - plus_charge_start_time) >=
			    (stop_left_time * 1000)) {
				printk(KERN_ERR "end time...%d: delta,%d\n",
				       sci_syst_read(),
				       (sci_syst_read() -
					plus_charge_start_time));
#else
			stop_left_time--;
			if (stop_left_time <= 0) {
#endif
				battery_notify = 1;
				charge_stop(battery_data);
				battery_data->in_precharge = 1;
				stop_left_time = CHARGE_BEFORE_STOP;
				pluse_charging = 0;
			}
		}

		sprd_set_recharge(battery_data);
		now_jiffies = get_jiffies_64();
		if ((now_jiffies - battery_data->charge_start_jiffies) >
		    CHARGE_OVER_TIME * HZ) {
			battery_notify = 1;
			charge_stop(battery_data);
			battery_data->in_precharge = 1;
			printk(KERN_ERR
			       "charge last over %d seconds, stop charge\n",
			       CHARGE_OVER_TIME);
		}
	}

out:
	if (!in_sleep) {

		capacity =
		    sprd_vol_to_percent(battery_data,
					sprd_bat_adc_to_vol(battery_data,
							    get_vbat_capacity_value
							    ()), 0);
		voltage = (voltage / 10) * 10;

		if (battery_data->capacity != capacity) {
			battery_data->capacity = capacity;
			battery_notify = 1;
		}

		if (battery_data->voltage != voltage) {
			battery_data->voltage = voltage;
			battery_notify = 1;
		}

		if (battery_notify) {
			power_supply_changed(&battery_data->battery);
		}
		if (usb_notify) {
			power_supply_changed(&battery_data->usb);
		}
		if (ac_notify) {
			power_supply_changed(&battery_data->ac);
		}
		if (battery_notify || usb_notify || ac_notify) {
			pr_debug("voltage %d\n", battery_data->voltage);
			pr_debug("capacity %d\n", battery_data->capacity);
			pr_debug("usb %d ac %d\n", battery_data->usb_online,
				 battery_data->ac_online);
			pr_debug("charge %d precharge %d\n",
				 battery_data->charging,
				 battery_data->in_precharge);
		}
		mod_timer(&battery_data->battery_timer,
			  jiffies + battery_data->timer_freq);
	}
	return;
}

static void battery_handler(unsigned long data)
{
	charge_handler((struct sprd_battery_data *)data, 0);
}

void battery_sleep(void)
{
	charge_handler(battery_data, 1);
}

/* used to detect battery capacity status
 * return 1: need update
 *        0: don't need
 */
#define VBAT_BUFF_NUM	7
int battery_updata(void)
{
	int32_t adc_value;
	int32_t voltage;
	uint32_t capacity;
	int32_t i, j, temp;
	int32_t vbat_result[VBAT_BUFF_NUM];
	static uint32_t pre_capacity = 0xffffffff;

	{
		for (i = 0; i < VBAT_BUFF_NUM; i++) {
			vbat_result[i] =
			    sci_adc_get_value(ADC_CHANNEL_VBAT, false);
		}

		for (j = 1; j <= VBAT_BUFF_NUM - 1; j++) {
			for (i = 0; i < VBAT_BUFF_NUM - j; i++) {
				if (vbat_result[i] > vbat_result[i + 1]) {
					temp = vbat_result[i];
					vbat_result[i] = vbat_result[i + 1];
					vbat_result[i + 1] = temp;
				}
			}
		}
		adc_value = vbat_result[VBAT_BUFF_NUM / 2];
	}
	if (adc_value < 0)
		return 0;
	voltage = sprd_bat_adc_to_vol(battery_data, adc_value);
	capacity = sprd_vol_to_percent(battery_data, voltage, 0);
	pr_info("battery_update: capacity %d,voltage:%d\n", capacity, voltage);
	if (pre_capacity == 0xffffffff) {
		voltage =
		    sprd_bat_adc_to_vol(battery_data,
					get_vbat_capacity_value());
		pre_capacity = sprd_vol_to_percent(battery_data, voltage, 0);
	}

	if (pre_capacity != capacity) {
		pre_capacity = capacity;
		update_vbat_value(battery_data, adc_value);

		for (i = 0; i < VBAT_CAPACITY_BUFF_CNT; i++) {	//init capacity vbat buffer for cal batttery capacity
			put_vbat_capacity_value(adc_value);
		}
	}
	if (capacity < 5) {
		return 1;
	} else {
		return 0;
	}
}

static char *supply_list[] = {
	"battery",
};

int __weak usb_register_hotplug_callback(struct usb_hotplug_callback *cb)
{
	return -ENODEV;
}

static unsigned int adc_data[2]={0};
static int __init adc_cal_start(char *str)
{
        char *cali_data = &str[1];
        if(str){
                pr_info("adc_cal%s!\n",str);
                sscanf(cali_data, "%d,%d", &adc_data[0],&adc_data[1]);
                pr_info("adc_data: 0x%x 0x%x!\n",adc_data[0],adc_data[1]);
        }
        return 1;
}
__setup("adc_cal", adc_cal_start);

extern int sci_efuse_calibration_get(unsigned int *p_cal_data);
static int sprd_battery_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct sprd_battery_data *data;
	int adc_value;
	int voltage_value;
	int i;
	struct resource *res = NULL;
	unsigned int efuse_cal_data[2] = { 0 };

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	platform_set_drvdata(pdev, data);
	battery_data = data;

#ifndef CONFIG_MUSB_FSA880
	spin_lock_init(&data->lock);

	data->charging = 0;
	data->cur_type = 400;
	data->adc_cal_updated = ADC_CAL_TYPE_NO;
	data->hw_switch_point = CHGMNG_DEFAULT_SWITPOINT;

	data->over_voltage = OVP_VOL_VALUE;
	data->over_voltage_recovery = OVP_VOL_RECV_VALUE;
	data->over_voltage_flag = 0;
	data->over_temp_flag = 0;
	data->over_current = CHARGE_OVER_CURRENT;
	data->precharge_start = PREVRECHARGE;
	data->precharge_end = PREVCHGEND;
	data->charge_stop_point = CHGMNG_STOP_VPROG;

	data->battery.properties = sprd_battery_props;
	data->battery.num_properties = ARRAY_SIZE(sprd_battery_props);
	data->battery.get_property = sprd_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	data->ac.properties = sprd_ac_props;
	data->ac.num_properties = ARRAY_SIZE(sprd_ac_props);
	data->ac.get_property = sprd_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.supplied_to = supply_list;
	data->ac.num_supplicants = ARRAY_SIZE(supply_list);

	data->usb.properties = sprd_usb_props;
	data->usb.num_properties = ARRAY_SIZE(sprd_usb_props);
	data->usb.get_property = sprd_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
	data->usb.supplied_to = supply_list;
	data->usb.num_supplicants = ARRAY_SIZE(supply_list);

	init_timer(&data->battery_timer);
	data->battery_timer.function = battery_handler;
	data->battery_timer.data = (unsigned long)data;

	sprd_chg_init();

	printk("probe adc4200: %d,adc3600:%d\n", adc_voltage_table[0][0],
	       adc_voltage_table[1][0]);

	if (sci_efuse_calibration_get(efuse_cal_data)) {
		adc_voltage_table[0][1] = efuse_cal_data[0] & 0xffff;
		adc_voltage_table[0][0] = (efuse_cal_data[0] >> 16) & 0xffff;
		adc_voltage_table[1][1] = efuse_cal_data[1] & 0xffff;
		adc_voltage_table[1][0] = (efuse_cal_data[1] >> 16) & 0xffff;
		data->adc_cal_updated = ADC_CAL_TYPE_EFUSE;
		printk("probe efuse ok!!! adc4200: %d,adc3600:%d\n",
		       adc_voltage_table[0][0], adc_voltage_table[1][0]);
	}
	if(((adc_data[2]&0xffff) < 4500 )&&((adc_data[2]&0xffff) > 3000)&&
	   ((adc_data[3]&0xffff) < 4500 )&&((adc_data[3]&0xffff) > 3000)){
                adc_voltage_table[0][1]=adc_data[0]&0xffff;
                adc_voltage_table[0][0]=(adc_data[0]>>16)&0xffff;
                adc_voltage_table[1][1]=adc_data[1]&0xffff;
                adc_voltage_table[1][0]=(adc_data[1]>>16)&0xffff;
                data->adc_cal_updated = ADC_CAL_TYPE_NV;
		printk("probe cmdline ok!!! adc4200: %d,adc3600:%d\n",
		       adc_voltage_table[0][0], adc_voltage_table[1][0]);
        }

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (unlikely(!res)) {
		dev_err(&pdev->dev, "not io resource\n");
		goto err_io_resource;
	}

	data->gpio = res->start;
#ifndef CONFIG_NOTIFY_BY_USB
	ret = gpio_request(data->gpio, "charger");
	if (ret) {
		dev_err(&pdev->dev, "failed to request gpio: %d\n", ret);
		goto err_io_request;
	}
	gpio_direction_input(data->gpio);
	ret = gpio_to_irq(data->gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get irq form gpio: %d\n", ret);
		goto err_io_to_irq;
	}
	data->irq = ret;
#endif

	for (i = 0; i < CONFIG_AVERAGE_CNT; i++) {
retry_adc:
		adc_value = sci_adc_get_value(ADC_CHANNEL_VBAT, false);
		if (adc_value < 0) {
			pr_err("ADC read error\n");
			msleep(100);
			goto retry_adc;
		} else {
			put_vbat_value(data, adc_value);	//vbat drop by large current.
		}
	}
	adc_value = get_vbat_value(data);
	for (i = 0; i < VBAT_CAPACITY_BUFF_CNT; i++) {	//init capacity vbat buffer for cal batttery capacity
		put_vbat_capacity_value(adc_value);
	}
	voltage_value = sprd_bat_adc_to_vol(data, get_vbat_capacity_value());
	data->capacity = sprd_vol_to_percent(battery_data, voltage_value, 0);
	dev_dbg(&pdev->dev, "charger present: %d capacity %d\n",
		usb_connected(), data->capacity);
	update_vbat_value(data, adc_value);
	update_vprog_value(data, 0);
#ifdef CONFIG_BATTERY_TEMP_DECT
	{
		int32_t temp_value;
		for (i = 0; i < CONFIG_AVERAGE_CNT; i++) {
retry_temp_adc:
			temp_value = sci_adc_get_value(ADC_CHANNEL_TEMP, false);
			if (temp_value < 0) {
				pr_err("temp ADC read error\n");
				msleep(100);
				goto retry_temp_adc;
			} else {
				put_temp_value(data, temp_value);
			}
		}
	}
#endif

	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;
#endif
	wake_lock_init(&(data->charger_plug_out_lock), WAKE_LOCK_SUSPEND,
		       "charger_plug_out_lock");
	data->usb_online = 0;
	data->ac_online = 0;
	data->charge_start_jiffies = 0;

#if defined(CONFIG_NOTIFY_BY_USB)
	data->timer_freq = HZ;
	ret = usb_register_hotplug_callback(&power_cb);
	//data->ac_online = usb_connected();
#else
	if (usb_connected()) {
		if (sprd_charger_is_adapter(data)) {
			data->ac_online = 1;
			data->usb_online = 0;
		} else {
			data->usb_online = 1;
			data->ac_online = 0;
		}
		data->timer_freq = HZ / 10;
	} else {
		data->ac_online = 0;
		data->usb_online = 0;
		data->timer_freq = HZ;
	}
	ret = request_irq(data->irq, sprd_battery_interrupt, IRQF_SHARED |
			  IRQF_TRIGGER_HIGH, pdev->name, data);
	if (ret)
		goto err_request_irq_failed;
#endif

#ifndef CONFIG_MUSB_FSA880
	sprd_creat_caliberate_attr(data->battery.dev);
	mod_timer(&data->battery_timer, jiffies + data->timer_freq);
#endif

	return 0;

err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	if (data->irq) {
		free_irq(data->irq, data);
	}
err_io_to_irq:
err_io_request:
	if (data->gpio) {
		gpio_free(data->gpio);
	}
err_io_resource:
err_request_irq_failed:
	kfree(data);
err_data_alloc_failed:
	battery_data = NULL;
	return ret;
}

static int sprd_battery_remove(struct platform_device *pdev)
{
	struct sprd_battery_data *data = platform_get_drvdata(pdev);

	sprd_remove_caliberate_attr(data->battery.dev);
	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);
	power_supply_unregister(&data->usb);

	del_timer_sync(&data->battery_timer);
#ifndef CONFIG_NOTIFY_BY_USB
	free_irq(data->irq, data);
#endif
	gpio_free(data->gpio);
	kfree(data);
	battery_data = NULL;
	return 0;
}

static int sprd_battery_resume(struct platform_device *pdev)
{
	uint32_t voltage = 0;
	struct sprd_battery_data *data = platform_get_drvdata(pdev);

	voltage = sprd_bat_adc_to_vol(data, get_vbat_capacity_value());
	data->capacity = sprd_vol_to_percent(battery_data, voltage, 0);
	power_supply_changed(&battery_data->battery);
	return 0;
}

static struct platform_driver sprd_battery_device = {
	.probe = sprd_battery_probe,
	.remove = sprd_battery_remove,
	.resume = sprd_battery_resume,
	.driver = {
		   .name = "sprd-battery"}
};

static int __init sprd_battery_init(void)
{
	return platform_driver_register(&sprd_battery_device);
}

static void __exit sprd_battery_exit(void)
{
	platform_driver_unregister(&sprd_battery_device);
}

module_init(sprd_battery_init);
module_exit(sprd_battery_exit);

MODULE_AUTHOR("Mark Yang markyang@spreadtrum.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Batter and charger driver for SC8800G");
