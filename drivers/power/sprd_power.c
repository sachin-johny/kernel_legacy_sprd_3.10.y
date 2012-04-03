/* drivers/power/sprd_battery.c
 *
 * Power supply driver for the sprd emulator
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/hrtimer.h>
#include <mach/adc_drvapi.h>
#include <mach/chg_drvapi.h>
#include <mach/mfp.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <mach/bits.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <mach/usb.h>
#include <mach/hardware.h>
#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>
#if defined(CONFIG_ARCH_SC8810)
#include <mach/eic.h>
#endif

#include <linux/slab.h>
#include <linux/jiffies.h>

//#define CHG_DEBUG
//#define BATTERY_USE_WAKE_LOCK
#define BATTERY_WAKE_LOCK_LENGTH (10*HZ)
#define CHARGER_PLUG_OUT_LENGTH (3*HZ)
#ifdef CHG_DEBUG
#define DEBUG(_fmt...) printk(_fmt)
#else
#define DEBUG(_fmt...) 
#endif

#ifndef abs
#define abs(_x) (_x>=0? _x: -_x)
#endif

//#define BATTERY_HAVE_TEMP

struct sprd_battery_data {
	uint32_t reg_base;
	int irq;
	spinlock_t lock;

    struct timer_list battery_timer;
    int timer_freq;
    int in_precharge;
    int adc_cal_updated;

    uint32_t capacity;
    uint32_t voltage;
#if BATTERY_HAVE_TEMP 
    int temp;
#endif
    uint32_t charging;
    uint32_t ac_online;
    uint32_t usb_online;

    uint32_t precharge_start;
    uint32_t precharge_end;
    uint32_t over_voltage;
    uint32_t over_voltage_recovery;
    uint32_t over_voltage_flag;
    uint32_t over_current;
    uint32_t charge_stop_point;
    uint32_t cur_type;
	uint32_t hw_switch_point;
    uint64_t charge_start_jiffies;

	struct power_supply battery;
	struct power_supply ac;
	struct power_supply usb;
    struct wake_lock charger_plug_out_lock;
#ifdef BATTERY_USE_WAKE_LOCK
    struct wake_lock charge_wake_lock;
    struct wake_lock update_wake_lock;
#endif
};

/* temporary variable used between sprd_battery_probe() and sprd_battery_open() */
static struct sprd_battery_data *battery_data;
uint32_t get_vprog_value(void);

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


static int sprd_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
		struct sprd_battery_data, ac);
	int ret = 0;
    unsigned long flag;

    spin_lock_irqsave(&data->lock, flag);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
        if(likely(psy->type == POWER_SUPPLY_TYPE_MAINS)){
            val->intval = data->ac_online? 1:0;
        }else{
            ret = -EINVAL;
        }
		break;
	default:
		ret = -EINVAL;
		break;
	}
    spin_unlock_irqrestore(&data->lock, flag);
	return ret;
}
static int sprd_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
		struct sprd_battery_data, usb);
	int ret = 0;
    unsigned long flag;

    spin_lock_irqsave(&data->lock, flag);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
        val->intval = data->usb_online? 1:0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
    spin_unlock_irqrestore(&data->lock, flag);
	return ret;
}

static int sprd_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct sprd_battery_data *data = container_of(psy,
		struct sprd_battery_data, battery);
	int ret = 0;
    unsigned long flag;

    spin_lock_irqsave(&data->lock, flag);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		//suppose battery always online
        if(data->charging){
            if(data->capacity >= 100)
              val->intval = POWER_SUPPLY_STATUS_FULL;
            else
              val->intval = POWER_SUPPLY_STATUS_CHARGING;
        }else{
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
		val->intval = (data->capacity > 100)? 100:data->capacity;
		break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = data->voltage*1000;
        break;
#if BATTERY_HAVE_TEMP 
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = data->temp;
        break;
#endif
	default:
		ret = -EINVAL;
		break;
	}
    spin_unlock_irqrestore(&data->lock, flag);
	return ret;
}

static enum power_supply_property sprd_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
#if BATTERY_HAVE_TEMP
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
                                    struct device_attribute *attr,
                                    char *buf);

#define SPRD_CALIBERATE_ATTR(_name)                         \
{                                       \
        .attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO, .owner = THIS_MODULE },  \
        .show = sprd_show_caliberate,                  \
        .store = sprd_set_caliberate,                              \
}
#define SPRD_CALIBERATE_ATTR_RO(_name)                         \
{                                       \
        .attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },  \
        .show = sprd_show_caliberate,                  \
}
#define SPRD_CALIBERATE_ATTR_WO(_name)                         \
{                                       \
        .attr = { .name = #_name, .mode = S_IWUGO, .owner = THIS_MODULE },  \
        .store = sprd_set_caliberate,                              \
}
static struct device_attribute sprd_caliberate[]={
    SPRD_CALIBERATE_ATTR_RO(real_time_voltage),
    SPRD_CALIBERATE_ATTR_WO(stop_charge),
    SPRD_CALIBERATE_ATTR_RO(real_time_current),
    SPRD_CALIBERATE_ATTR_WO(battery_0),
    SPRD_CALIBERATE_ATTR_WO(battery_1),
    SPRD_CALIBERATE_ATTR(hw_switch_point),
    SPRD_CALIBERATE_ATTR_RO(charger_voltage),
};
static enum {
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
    DEBUG("set value %d\n", set_value);

//use the global battery_data
    spin_lock_irqsave(&battery_data->lock, flag);
    switch(off){
    case STOP_CHARGE:
        battery_data->usb_online = 0;
        battery_data->ac_online = 0;
        break;
    case BATTERY_0:
        if(battery_data->adc_cal_updated == 0){
            adc_voltage_table[0][1]=set_value&0xffff;
            adc_voltage_table[0][0]=(set_value>>16)&0xffff;
        }
        break;
    case BATTERY_1:
        if(battery_data->adc_cal_updated == 0){
            adc_voltage_table[1][1]=set_value&0xffff;
            adc_voltage_table[1][0]=(set_value>>16)&0xffff;
            CHGMNG_VoltageToPercentum(0, 0, 1, 0);
            battery_data->adc_cal_updated = 1;
        }
        break;
	case HW_SWITCH_POINT:
		battery_data->hw_switch_point = set_value;
		CHG_SetSwitchoverPoint ((CHG_SWITPOINT_E)(battery_data->hw_switch_point));
		break;
    default:
        count = -EINVAL;
        break;
    }
    spin_unlock_irqrestore(&battery_data->lock, flag);
    return count;
}

static ssize_t sprd_show_caliberate(struct device *dev, 
                                    struct device_attribute *attr,
                                    char *buf)
{
    int i = 0;
    const ptrdiff_t off = attr - sprd_caliberate;
    unsigned long flag;
    int adc_value;
    int voltage;
    uint32_t now_current;
//use the global battery_data
    spin_lock_irqsave(&battery_data->lock, flag);
    switch(off){
    case BATTERY_VOLTAGE:
        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);        
        if(adc_value < 0)
          voltage = 0;
        else
          voltage = CHGMNG_AdcvalueToVoltage(adc_value);
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage);
        break;
    case BATTERY_NOW_CURRENT:
        if(battery_data->charging){
            now_current = get_vprog_value();        
            i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                        now_current);
        }else{
            i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
                        "discharging");
        }
        break;
	case HW_SWITCH_POINT:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->hw_switch_point);
		break;
    case CHARGER_VOLTAGE:
        if(battery_data->charging){
            adc_value = ADC_GetValue(ADC_CHANNEL_VCHG, false); 
            if(adc_value < 0)
              voltage = 0;
            else
              voltage = CHGMNG_ChargerAdcvalueToVoltage(adc_value);
            i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                        voltage);
        }else{
            i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
                        "discharging");
        }
        break;
    default:
        i = -EINVAL;
        break;
    }
    spin_unlock_irqrestore(&battery_data->lock, flag);
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
    goto succeed;

sprd_attrs_failed:
    while (i--)
          device_remove_file(dev, &sprd_caliberate[i]);

succeed:
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
#if defined(CONFIG_ARCH_SC8810)
    return sprd_get_eic_data(EIC_ID_10);
#else
    return gpio_get_value(CHARGER_DETECT_GPIO)&BIT_2? 1:0;
#endif
}
static irqreturn_t sprd_battery_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct sprd_battery_data *data = dev_id;
    uint32_t charger_status;
    DEBUG("charger plug interrupt happen\n");

	spin_lock_irqsave(&data->lock, irq_flags);

    charger_status = usb_connected();
    if(charger_status){
        if(charger_is_adapter()){
            data->ac_online = 1;
            data->usb_online = 0;
        }
        else{
            data->usb_online = 1;
            data->ac_online = 0;
        }
        //mod_timer(&data->battery_timer, jiffies);
#ifdef BATTERY_USE_WAKE_LOCK
        wake_lock(&(data->charge_wake_lock));
#endif
    }else{
        data->ac_online = 0;
        data->usb_online = 0;
#ifdef BATTERY_USE_WAKE_LOCK
        wake_unlock(&(data->charge_wake_lock));
#endif
    }
    DEBUG("interrupt happen: usb:%d, ac:%d\n", data->usb_online, data->ac_online);
    set_irq_type(irq, charger_status? IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH);
	spin_unlock_irqrestore(&data->lock, irq_flags);
	return IRQ_HANDLED;
}

static int plugin_callback(int usb_cable, void *data)
{
	struct sprd_battery_data *d = battery_data;
	unsigned long irq_flags;
	int ac_adapter = 0;

	DEBUG("charger plugin interrupt happen\n");
	if ( !d ) {
		pr_warning("batttery_data is NULL!!\n");
		return 1;
	}

	ac_adapter = charger_is_adapter();
	spin_lock_irqsave(&d->lock, irq_flags);

	d->ac_online = 0;
	d->usb_online = 0;

	if (ac_adapter) {
		d->ac_online = 1;
	} else {
		d->usb_online = 1;
	}

	spin_unlock_irqrestore(&d->lock, irq_flags);
	DEBUG("interrupt happen: usb:%d, ac:%d\n", d->usb_online, d->ac_online);

	return 0;
}

static int plugout_callback(int usb_cable, void *data)
{
	struct sprd_battery_data *d = battery_data;
	unsigned long irq_flags;

	DEBUG("charger plugout interrupt happen\n");
	if ( !d ) {
		pr_warning("batttery_data is NULL!!\n");
		return 1;
	}

	spin_lock_irqsave(&d->lock, irq_flags);

	d->ac_online = 0;
	d->usb_online = 0;

	spin_unlock_irqrestore(&d->lock, irq_flags);
	DEBUG("interrupt happen: usb:%d, ac:%d\n", d->usb_online, d->ac_online);
	wake_lock_timeout(&(d->charger_plug_out_lock), CHARGER_PLUG_OUT_LENGTH);

	return 0;
}

#define _BUF_SIZE 10
#define _VPROG_BUF_SIZE 8
uint32_t temp_buf[_BUF_SIZE];
uint32_t vprog_buf[_VPROG_BUF_SIZE];
uint32_t vbat_buf[_BUF_SIZE];
void put_temp_value(uint32_t temp)
{
    int i;
    for(i=0;i<_BUF_SIZE -1;i++){
        temp_buf[i] = temp_buf[i+1];
    }

    temp_buf[_BUF_SIZE-1] = temp;
}

uint32_t get_temp_value(void)
{
    unsigned long sum=0;
    int i;
    for(i=0; i < _BUF_SIZE; i++)
      sum += temp_buf[i];

    return sum/_BUF_SIZE;
}
void put_vprog_value(uint32_t vprog)
{
    int i;
    uint32_t temp;
    for(i=0;i<_VPROG_BUF_SIZE -1;i++){
        vprog_buf[i] = vprog_buf[i+1];
    }

    vprog_buf[i] = vprog;
}

uint32_t get_vprog_value(void)
{
    int i,sum = 0;
    for(i=0;i<_VPROG_BUF_SIZE;i++){
        sum = sum + vprog_buf[i];
    }
    return sum/_VPROG_BUF_SIZE;
}
void update_vprog_value(uint32_t vprog)
{
    int i;
    for(i=0;i<_VPROG_BUF_SIZE;i++){
        vprog_buf[i] = vprog;
    }
}
void put_vbat_value(uint32_t vbat)
{
    int i;
    for(i=0;i<_BUF_SIZE -1;i++){
        vbat_buf[i] = vbat_buf[i+1];
    }

    vbat_buf[_BUF_SIZE-1] = vbat;
}

uint32_t get_vbat_value(void)
{
    unsigned long sum=0;
    int i;
    for(i=0; i < _BUF_SIZE; i++)
      sum += vbat_buf[i];

    return sum/_BUF_SIZE;
}

void update_vbat_value(uint32_t vbat)
{
    int i;
    for(i=0; i < _BUF_SIZE; i++)
      vbat_buf[i] = vbat;
}

static int pluse_charging = 0;
static int pluse_charge_cnt = CHGMNG_PLUST_TIMES;
static int hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
static int stop_left_time = CHARGE_BEFORE_STOP;
static int32_t vprog_current = 0;
static int pre_usb_online = 0;
static int pre_ac_online = 0;

#define CHARGE_LOOP_CNT 10
void enable_usb_charge(struct sprd_battery_data * battery_data)
{
    pluse_charge_cnt = CHGMNG_PLUST_TIMES;
    hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
    battery_data->charge_start_jiffies = get_jiffies_64();
    battery_data->charging = 1;
    pluse_charging = 0;
    CHG_SetAdapterMode(CHG_USB_ADAPTER);
    CHG_SetUSBChargeCurrent(CHG_USB_400MA);
    battery_data->cur_type = 400;
    CHG_SetSwitchoverPoint ((CHG_SWITPOINT_E)(battery_data->hw_switch_point));
    CHG_TurnOn();
    battery_data->in_precharge = 0;
}
void enable_ac_charge(struct sprd_battery_data * battery_data)
{
    pluse_charge_cnt = CHGMNG_PLUST_TIMES;
    hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
    battery_data->charge_start_jiffies = get_jiffies_64();
    CHG_SetAdapterMode(CHG_NORMAL_ADAPTER);
    CHG_SetNormalChargeCurrent(CHG_NOR_600MA);
    battery_data->cur_type = 600;
    battery_data->charging = 1;
    pluse_charging = 0;
    CHG_SetSwitchoverPoint ((CHG_SWITPOINT_E)(battery_data->hw_switch_point));
    CHG_TurnOn();
    battery_data->in_precharge = 0;
}
void charge_stop(struct sprd_battery_data * battery_data)
{
    CHG_ShutDown();
    CHG_StopRecharge();
    battery_data->charging = 0;
    pluse_charging = 0;
//	battery_data->hw_switch_point = CHG_UpdateSwitchoverPoint(false);
    battery_data->in_precharge = 0;
}
static void charge_handler(struct sprd_battery_data * battery_data, int in_sleep)
{
    uint32_t voltage=0;
    uint32_t capacity=0;
    int32_t adc_value=0;
    int32_t vprog_value=0;
    int usb_online= 0;
    int ac_online = 0;
    int ac_notify = 0;
    int usb_notify = 0;
    int battery_notify = 0;
    uint32_t flag = 0;
    int timer_freq;
    int32_t current_ref = 0;
    int32_t vchg_value;
#if BATTERY_HAVE_TEMP
    int32_t temp_value;
    int temp;
#endif
    static int loop_cnt = 0; 
    uint64_t now_jiffies = 0;
    
    loop_cnt++;
    loop_cnt %= CHARGE_LOOP_CNT;

    if(!in_sleep){
        spin_lock_irqsave(&battery_data->lock, flag);
        timer_freq = battery_data->timer_freq;
    }else{
        DEBUG("charge in sleep\n");
    }

    usb_online = battery_data->usb_online;
    ac_online = battery_data->ac_online;

    if(pre_ac_online != ac_online){
        pre_ac_online = ac_online;
        ac_notify = 1;
        if(ac_online){
            enable_ac_charge(battery_data);
        }else{
            charge_stop(battery_data);
        }
        battery_notify = 1;
    }

    if(pre_usb_online != usb_online){
        pre_usb_online = usb_online;
        usb_notify = 1;
        if(usb_online){
            enable_usb_charge(battery_data);
        }else{
            charge_stop(battery_data);
        }
        battery_notify = 1;
    }

    if(battery_data->charging || loop_cnt == 0){

        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
        if(adc_value >= 0)
          put_vbat_value(adc_value);

        if(adc_value < 0)
          goto out;
        adc_value = get_vbat_value();
        DEBUG("vbat %d\n", adc_value);

        voltage = CHGMNG_AdcvalueToVoltage(adc_value);
        DEBUG("voltage %d\n", voltage);

        DEBUG("charging %d in_precharge %d, usb %d, ac %d, start %d\n", battery_data->charging, battery_data->in_precharge,\
                    usb_online, ac_online, battery_data->precharge_start); 													   

        if(!battery_data->charging && (battery_data->in_precharge == 1) && \
                    usb_online && (voltage < battery_data->precharge_start)){
            enable_usb_charge(battery_data);
            battery_notify = 1;
        }

        if(!battery_data->charging && (battery_data->in_precharge == 1) && \
                    ac_online && (voltage < battery_data->precharge_start)){
            enable_ac_charge(battery_data);
            battery_notify = 1;
        }
    }

    if(battery_data->charging){
        vprog_value = CHG_GetVirtualVprog();
        if(vprog_value < 0)
          goto out;
        DEBUG("raw vprog adc %d\n", vprog_value);
        vprog_current = CHGMNG_AdcvalueToVoltage(vprog_value);
        DEBUG("vporg adc to voltage %d \n", vprog_current);

        vprog_current = CHGMNG_AdcvalueToCurrent(vprog_current, battery_data->cur_type);
        DEBUG("raw vporg current %d\n", vprog_current);
        put_vprog_value(vprog_current);
        vprog_current= get_vprog_value();
        DEBUG("average vprog current %d\n", vprog_current);

        vchg_value = ADC_GetValue(ADC_CHANNEL_VCHG, false);
        if(vchg_value < 0)
          goto out;
        //DEBUG("%s: vchg %d\n", __func__, vchg_value);

        if(vchg_value> battery_data->over_voltage){
            printk("charger voltage too high\n");
            charge_stop(battery_data);
            battery_data->over_voltage_flag = 1;
        }

        if(voltage > 4300){
            battery_notify = 1;
            charge_stop(battery_data);
            battery_data->in_precharge = 1;
            printk("vbat over %d \n", voltage );
        }
    }
    if(!battery_data->charging && !battery_data->in_precharge && \
                (usb_online || ac_online) && battery_data->over_voltage_flag){
        vchg_value = ADC_GetValue(ADC_CHANNEL_VCHG, false);
        if(vchg_value < 0)
          goto out;
        if(vchg_value < battery_data->over_voltage_recovery){
            battery_data->over_voltage_flag = 0;
            if(ac_online)
              enable_ac_charge(battery_data);
            else
              enable_usb_charge(battery_data);
        }
    }

#if BATTERY_HAVE_TEMP
    if(loop_cnt == 0){
        temp_value = ADC_GetValue(ADC_CHANNEL_TEMP, false);
        if(temp_value < 0)
          goto out;
        //put_temp_value(temp_value);
        //temp_value = get_temp_value();
        DEBUG("temp_value 0x%x\n", temp_value);

        temp = CHGMNG_AdcvalueToTemp(temp_value);

        DEBUG("temp: %d\n", temp);
        if(abs(battery_data->temp - temp)>2){
            battery_data->temp = temp;
            battery_notify =1;
        }

        if(temp > 450 || temp < 0){
            printk("battery temperature out of 45~0\n");
            charge_stop(battery_data);
        }
    }
#endif

    if(battery_data->charging && loop_cnt == 0){
        if(!pluse_charging ){
            hw_switch_update_cnt --;
            if(hw_switch_update_cnt <= 0){
                hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
                
                if(voltage <= battery_data->precharge_end) {
                    if(vprog_current < CC_CV_SWITCH_POINT){
                        battery_data->hw_switch_point = CHG_UpdateSwitchoverPoint(true);
                    }
                }else{
                    if(vprog_current <= CV_STOP_CURRENT){
                        pluse_charging = 1;
                        stop_left_time = CHARGE_BEFORE_STOP;
                    }
                    if(voltage > CHGMNG_OVER_CHARGE){
                        battery_data->hw_switch_point = CHG_GetSwitchoverPoint();
                        CHG_UpdateSwitchoverPoint(false);
                    }
               }
            }
        }else {
            stop_left_time --;
            if(stop_left_time <=0){
                battery_notify = 1;
                charge_stop(battery_data);
                battery_data->in_precharge = 1;
                stop_left_time = CHARGE_BEFORE_STOP;
                pluse_charging = 0;
            }
        }
        
        CHG_SetRecharge();
        now_jiffies = get_jiffies_64();
        if((now_jiffies - battery_data->charge_start_jiffies) > CHARGE_OVER_TIME * HZ){
            battery_notify = 1;
            charge_stop(battery_data);
            battery_data->in_precharge = 1;
            printk("charge last over %d seconds, stop charge\n", CHARGE_OVER_TIME);
        }
        DEBUG("usb online %d, ac online %d\n", usb_online, ac_online);
        DEBUG("hw_switch_point %d\n", battery_data->hw_switch_point);
        {
            DEBUG("ANA_CHG_CTL0 0x%x\n", ANA_REG_GET(ANA_CHGR_CTL0));
            DEBUG("ANA_CHG_CTL1 0x%x\n", ANA_REG_GET(ANA_CHGR_CTL1));
            DEBUG("*******************\n");
        }
    }

out:
    if(!in_sleep){
        spin_unlock_irqrestore(&battery_data->lock, flag);

        if(loop_cnt == 0){
            capacity = CHGMNG_VoltageToPercentum(voltage, battery_data->charging, 0, usb_online);
            voltage = (voltage /10)*10;
            //DEBUG("capacity %d\n", capacity);

            if(battery_data->capacity != capacity){
                battery_data->capacity = capacity;
                battery_notify = 1;
            }

            if(battery_data->voltage != voltage){
                battery_data->voltage = voltage;
                battery_notify = 1;
            }
        }

        if(battery_notify){
            power_supply_changed(&battery_data->battery);
        }
        if(usb_notify){
            power_supply_changed(&battery_data->usb);
        }
        if(ac_notify){
            power_supply_changed(&battery_data->ac);
        }
        if(battery_notify || usb_notify || ac_notify){
#ifdef BATTERY_USE_WAKE_LOCK
            wake_lock_timeout(&(battery_data->update_wake_lock), BATTERY_WAKE_LOCK_LENGTH);
#endif
        }
        mod_timer(&battery_data->battery_timer, jiffies + timer_freq);
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
int battery_updata(void)
{
    int32_t adc_value;
    int32_t voltage;
    uint32_t capacity;
    static uint32_t pre_capacity = 0xffffffff;
    adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
    if(adc_value < 0)
      return 0;
    voltage = CHGMNG_AdcvalueToVoltage(adc_value);
    capacity = CHGMNG_VoltageToPercentum(voltage, 0, 0, 0);
    DEBUG("battery_update: capacity %d\n", capacity);
    if(pre_capacity == 0xffffffff){
        adc_value = get_vbat_value();
        voltage = CHGMNG_AdcvalueToVoltage(adc_value);
        pre_capacity = CHGMNG_VoltageToPercentum(voltage, 0, 0, 0);
    }

    if(pre_capacity != capacity){
        pre_capacity = capacity;
        update_vbat_value(adc_value);
    }
    if(capacity <5){
        return 1;
    }else{
        return 0;
    }
}
static char * supply_list[]={
    "battery",
};

extern int sci_efuse_calibration_get(unsigned int* p_cal_data);
static int sprd_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct sprd_battery_data *data;
	int adc_value;
    int voltage_value;
    int i;
    int charger_present;
    unsigned int efuse_cal_data[2] = {0};

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

    data->charging = 0;
    data->cur_type = 400;
    data->adc_cal_updated = 0;

    data->over_voltage = OVP_ADC_VALUE;
    data->over_voltage_recovery = OVP_ADC_RECV_VALUE;
    data->over_voltage_flag = 0;
    data->over_current= CHARGE_OVER_CURRENT;
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

    printk("probe adc4200: %d,adc3600:%d\n", adc_voltage_table[0][0],adc_voltage_table[1][0]);

 	if(sci_efuse_calibration_get(efuse_cal_data))
 	{
		adc_voltage_table[0][1]=efuse_cal_data[0]&0xffff;
		adc_voltage_table[0][0]=(efuse_cal_data[0]>>16)&0xffff;
		adc_voltage_table[1][1]=efuse_cal_data[1]&0xffff;
		adc_voltage_table[1][0]=(efuse_cal_data[1]>>16)&0xffff;			
 	}

    ADC_Init();

    for(i=0;i<_BUF_SIZE;i++){
retry_adc:
        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
        if(adc_value < 0){
            DEBUG("ADC read error\n");
            msleep(100);
            goto retry_adc;
            //adc_value = 0;
        }else{
            put_vbat_value(adc_value);
        }
    }
    adc_value = get_vbat_value();
    printk("charge: adc value in probe 0x%x\n", adc_value);
    voltage_value = CHGMNG_AdcvalueToVoltage(adc_value);
    charger_present = usb_connected();
    printk("charge: charger present %d\n", charger_present);
    data->capacity = CHGMNG_VoltageToPercentum(voltage_value, charger_present, 0, 1);
    printk("charge: capacity value in probe %d\n", data->capacity);
	//update_vbat_value(adc_value);
	update_vprog_value(0);
	memset(vprog_buf, 0, sizeof(vprog_buf));
	memset(temp_buf, 0, sizeof(temp_buf));

	wake_lock_init(&(data->charger_plug_out_lock), WAKE_LOCK_SUSPEND, "charger_plug_out_lock");
#ifdef BATTERY_USE_WAKE_LOCK
	wake_lock_init(&(data->charge_wake_lock), WAKE_LOCK_SUSPEND, "charge_wake_lock");
	wake_lock_init(&(data->update_wake_lock), WAKE_LOCK_SUSPEND, "update_wake_lock");
#endif

	data->timer_freq = HZ/10;
	data->usb_online = 0;
	data->ac_online = 0;
    data->charge_start_jiffies = 0;

#if defined(CONFIG_USB_SUPPORT)
	ret = usb_register_hotplug_callback(&power_cb);
	if (ret)
		goto err_request_irq_failed;
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

	platform_set_drvdata(pdev, data);
	battery_data = data;
	data->hw_switch_point = CHGMNG_DEFAULT_SWITPOINT;

	sprd_creat_caliberate_attr(data->battery.dev);

	mod_timer(&data->battery_timer, jiffies + data->timer_freq);

	return 0;

err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	free_irq(data->irq, data);
err_request_irq_failed:
	kfree(data);
err_data_alloc_failed:
	return ret;
}

static int sprd_battery_remove(struct platform_device *pdev)
{
	struct sprd_battery_data *data = platform_get_drvdata(pdev);

    sprd_remove_caliberate_attr(data->battery.dev);
	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);
	power_supply_unregister(&data->usb);
    gpio_free(CHARGER_DETECT_GPIO);

    del_timer_sync(&data->battery_timer);
	free_irq(data->irq, data);
#ifdef BATTERY_USE_WAKE_LOCK
    wake_lock_destroy(&(data->charge_wake_lock));
    wake_lock_destroy(&(data->update_wake_lock));
#endif
	kfree(data);
	battery_data = NULL;
	return 0;
}
static int sprd_battery_resume(struct platform_device *pdev)
{
    struct sprd_battery_data *data = platform_get_drvdata(pdev);
    int32_t voltage_value;
    int32_t adc_value;
    uint32_t capacity;

    adc_value = get_vbat_value();
    if(adc_value < 0)
      return 0;
    voltage_value = CHGMNG_AdcvalueToVoltage(adc_value);
    capacity = CHGMNG_VoltageToPercentum(voltage_value, battery_data->charging, 0, battery_data->usb_online);
    DEBUG("%s capacity %d pre_capacity %d\n", __func__, capacity, data->capacity);
    capacity = (data->capacity + capacity)/2;
	data->capacity = capacity;
	power_supply_changed(&battery_data->battery);
    return 0;
}

static struct platform_driver sprd_battery_device = {
	.probe		= sprd_battery_probe,
	.remove		= sprd_battery_remove,
    .resume     = sprd_battery_resume,
	.driver = {
		.name = "sprd-battery"
	}
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
