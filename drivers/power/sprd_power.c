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

//#define CHG_DEBUG
#define BATTERY_USE_WAKE_LOCK
#define BATTERY_WAKE_LOCK_LENGTH (10*HZ)
#ifdef CHG_DEBUG
#define DEBUG(_fmt...) printk(_fmt)
#else
#define DEBUG(_fmt...) 
#endif

struct sprd_battery_data {
	uint32_t reg_base;
	int irq;
	spinlock_t lock;

    struct timer_list battery_timer;
    struct timer_list charge_voltage_timer;

    uint32_t capacity;
    uint32_t voltage;
    uint32_t charging;
    uint32_t ac_online;
    uint32_t usb_online;

    uint32_t precharge_start;
    uint32_t precharge_end;
    uint32_t over_voltage;
    uint32_t over_current;
    uint32_t hw_switch_point;
    uint32_t charge_stop_point;

	struct power_supply battery;
	struct power_supply ac;
	struct power_supply usb;
#ifdef BATTERY_USE_WAKE_LOCK
    struct wake_lock charge_wake_lock;
    struct wake_lock update_wake_lock;
#endif
};

/* temporary variable used between sprd_battery_probe() and sprd_battery_open() */
static struct sprd_battery_data *battery_data;
static int timer_freq;
static int timer_freq_flag;
static int timer_freq_cnt;


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
    SPRD_CALIBERATE_ATTR(precharge_start),
    SPRD_CALIBERATE_ATTR(precharge_end),
    SPRD_CALIBERATE_ATTR(over_voltage),
    SPRD_CALIBERATE_ATTR(over_current),
    SPRD_CALIBERATE_ATTR(hw_switch_point),
    SPRD_CALIBERATE_ATTR(charge_stop_point),
    SPRD_CALIBERATE_ATTR(capacity_0),
    SPRD_CALIBERATE_ATTR(capacity_10),
    SPRD_CALIBERATE_ATTR(capacity_20),
    SPRD_CALIBERATE_ATTR(capacity_40),
    SPRD_CALIBERATE_ATTR(capacity_60),
    SPRD_CALIBERATE_ATTR(capacity_80),
    SPRD_CALIBERATE_ATTR(capacity_100),
    SPRD_CALIBERATE_ATTR_RO(real_time_voltage),
    SPRD_CALIBERATE_ATTR_WO(stop_charge),
};
static enum {
    PRECHARGE_START = 0,
    PRECHARGE_END,
    OVER_VOLTAGE,
    OVER_CURRENT,
    HW_SWITCH_POINT,
    CHARGE_STOP_POINT,
    CAPACITY_0,
    CAPACITY_10,
    CAPACITY_20,
    CAPACITY_40,
    CAPACITY_60,
    CAPACITY_80,
    CAPACITY_100,
    BATTERY_VOLTAGE,
    STOP_CHARGE,
};
extern uint16_t voltage_capacity_table[16];
static ssize_t sprd_set_caliberate(struct device *dev,
                                     struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    unsigned long flag;
    unsigned long set_value;
    
    const ptrdiff_t off = attr - sprd_caliberate;
    set_value = simple_strtoul(buf, NULL, 10);
    if(set_value > 1000){
        pr_err("set value %lu too big\n", set_value);
        return 0;
    }
    DEBUG("set value %d\n", set_value);

//use the global battery_data
    spin_lock_irqsave(&battery_data->lock, flag);
    switch(off){
    case PRECHARGE_START: 
        battery_data->precharge_start = set_value;
        break;
    case PRECHARGE_END:
        battery_data->precharge_end = set_value;
        break;
    case OVER_VOLTAGE:
        battery_data->over_voltage = set_value;
        break;
    case OVER_CURRENT:
        battery_data->over_current = set_value;
        break;
    case HW_SWITCH_POINT:
        battery_data->hw_switch_point = set_value;
        break;
    case CHARGE_STOP_POINT:
        battery_data->charge_stop_point = set_value;
        break;
    case CAPACITY_0:
        voltage_capacity_table[0] = set_value;
        break;
    case CAPACITY_10:
        voltage_capacity_table[2] = set_value;
        break;
    case CAPACITY_20:
        voltage_capacity_table[4] = set_value;
        break;
    case CAPACITY_40:
        voltage_capacity_table[6] = set_value;
        break;
    case CAPACITY_60:
        voltage_capacity_table[8] = set_value;
        break;
    case CAPACITY_80:
        voltage_capacity_table[10] = set_value;
        break;
    case CAPACITY_100:
        voltage_capacity_table[12] = set_value;
        break;
    case STOP_CHARGE:
        battery_data->usb_online = 0;
        battery_data->ac_online = 0;
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
//use the global battery_data
    spin_lock_irqsave(&battery_data->lock, flag);
    switch(off){
    case PRECHARGE_START:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->precharge_start);
        break;
    case PRECHARGE_END:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->precharge_end);
        break;
    case OVER_VOLTAGE:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->over_voltage);
        break;
    case OVER_CURRENT:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->over_current);
        break;
    case HW_SWITCH_POINT:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->hw_switch_point);
        break;
    case CHARGE_STOP_POINT:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    battery_data->charge_stop_point);
        break;
    case CAPACITY_0:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[0]);
        break;
    case CAPACITY_10:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[2]);
        break;
    case CAPACITY_20:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[4]);
        break;
    case CAPACITY_40:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[6]);
        break;
    case CAPACITY_60:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[8]);
        break;
    case CAPACITY_80:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[10]);
        break;
    case CAPACITY_100:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[12]);
        break;
    case BATTERY_VOLTAGE:
        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);        
        if(adc_value < 0)
          voltage = 0;
        else
          voltage = CHGMNG_AdcvalueToVoltage(adc_value);
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage);
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
    return gpio_get_value(CHARGER_DETECT_GPIO)&BIT_2? 1:0;
}
extern int charger_is_adapter(void);
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
        timer_freq = HZ;
        timer_freq_flag = 1;
        mod_timer(&data->battery_timer, jiffies + timer_freq);
        mod_timer(&data->charge_voltage_timer, jiffies + HZ/10);
    }else{
        data->ac_online = 0;
        data->usb_online = 0;
        timer_freq = 10*HZ;
    }
#ifdef BATTERY_USE_WAKE_LOCK
        wake_lock_timeout(&(data->update_wake_lock), BATTERY_WAKE_LOCK_LENGTH);
#endif
    set_irq_type(irq, charger_status? IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH);
	spin_unlock_irqrestore(&data->lock, irq_flags);
	return IRQ_HANDLED;
}

#define _BUF_SIZE 10
uint32_t vchg_buf[_BUF_SIZE];
uint32_t vprog_buf[_BUF_SIZE];
uint32_t vbat_buf[_BUF_SIZE];
void put_vchg_value(uint32_t vchg)
{
    int i;
    for(i=0;i<_BUF_SIZE -1;i++){
        vchg_buf[i] = vchg_buf[i+1];
    }

    vchg_buf[_BUF_SIZE-1] = vchg;
}

uint32_t get_vchg_value(void)
{
    unsigned long sum=0;
    int i;
    for(i=0; i < _BUF_SIZE; i++)
      sum += vchg_buf[i];

    return sum/_BUF_SIZE;
}
void put_vprog_value(uint32_t vprog)
{
    int i;
    for(i=0;i<_BUF_SIZE -1;i++){
        vprog_buf[i] = vprog_buf[i+1];
    }

    vprog_buf[_BUF_SIZE-1] = vprog;
}

uint32_t get_vprog_value(void)
{
    unsigned long sum=0;
    int i;
    for(i=0; i < _BUF_SIZE; i++)
      sum += vprog_buf[i];

    return sum/_BUF_SIZE;
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

static CHG_SWITPOINT_E now_hw_switch_point;
static bool charge_pluse = true;

static void battery_handler(unsigned long data)
{
    uint32_t voltage;
    uint32_t capacity;
    int32_t adc_value;
    int32_t vprog_value;
    int32_t vchg_value;
    int usb_online= 0;
    static int pre_usb_online = 0;
    int ac_online = 0;
    static int pre_ac_online = 0;
    int ac_notify = 0;
    int usb_notify = 0;
    int battery_notify = 0;
    unsigned long flag;
    static int pluse_charging = 0;
    static int pluse_charge_cnt = CHGMNG_PLUST_TIMES;
    static int hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
    
    
    struct sprd_battery_data * battery_data = (struct sprd_battery_data *)data;
    usb_online = battery_data->usb_online;
    ac_online = battery_data->ac_online;
    if(pluse_charging && charge_pluse){
        CHG_ShutDown();
        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
        CHG_TurnOn();
    }else{
        adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
    }
    if(adc_value < 0)
      return;
    put_vbat_value(adc_value);
    adc_value = get_vbat_value();
    DEBUG("vbat %d\n", adc_value);

    voltage = CHGMNG_AdcvalueToVoltage(adc_value);
    voltage = (voltage /100)*100;

    vprog_value = ADC_GetValue(ADC_CHANNEL_PROG, false);
    if(vprog_value < 0)
      return;
    put_vprog_value(vprog_value);
    vprog_value = get_vprog_value();
    DEBUG("vprog %d\n", vprog_value);

    //update capity;
    //notify user space
    spin_lock_irqsave(&battery_data->lock, flag);
    capacity = CHGMNG_VoltageToPercentum(adc_value);
    DEBUG("capacity %d\n", capacity);
    DEBUG("now_hw_switch_point %d\n", now_hw_switch_point);

    if(pre_ac_online != ac_online){
        pre_ac_online = ac_online;
        ac_notify = 1;
    }

    if(pre_usb_online != usb_online){
        pre_usb_online = usb_online;
        usb_notify = 1;
    }

    if(battery_data->capacity != capacity){
        battery_data->capacity = capacity;
        battery_notify = 1;
    }

    if(battery_data->voltage != voltage){
        battery_data->voltage = voltage;
        battery_notify = 1;
    }

    if(usb_online && (adc_value < battery_data->precharge_start) && !battery_data->charging){
    //if(usb_online && (adc_value < PREVRECHARGE) && !battery_data->charging){
        pluse_charge_cnt = CHGMNG_PLUST_TIMES;
        hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;

        battery_data->charging = 1;
        CHG_SetAdapterMode(CHG_USB_ADAPTER);
        CHG_SetUSBChargeCurrent(CHG_USB_300MA);
        CHG_SetSwitchoverPoint (CHGMNG_DEFAULT_SWITPOINT);
        CHG_TurnOn();
        CHG_SetRecharge();
        battery_notify = 1;
    }

    if(ac_online && (adc_value < battery_data->precharge_start) && !battery_data->charging){
    //if(ac_online && (adc_value < PREVRECHARGE) && !battery_data->charging){
        pluse_charge_cnt = CHGMNG_PLUST_TIMES;
        hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;

        battery_data->charging = 1;
        CHG_SetAdapterMode(CHG_NORMAL_ADAPTER);
        CHG_SetNormalChargeCurrent(CHG_NOR_800MA);
        CHG_SetSwitchoverPoint (CHGMNG_DEFAULT_SWITPOINT);
        CHG_TurnOn();
        CHG_SetRecharge();
        battery_notify = 1;
    }

    if(battery_data->charging){
        if(ac_online || usb_online){
            if(!pluse_charging ){
                if(adc_value < battery_data->precharge_end) {
                    hw_switch_update_cnt --;
                    if(hw_switch_update_cnt <= 0){
                        if(vprog_value < battery_data->hw_switch_point){
                            now_hw_switch_point = CHG_UpdateSwitchoverPoint(true);
                        }
                        hw_switch_update_cnt = CHARGE_VBAT_STATISTIC_BUFFERSIZE;
                    }
                }else{
                    pluse_charging = 1;
                }
            }else {
                if(charge_pluse){
                    pluse_charge_cnt --;
                    if(pluse_charge_cnt == 0){
                        charge_pluse = false;
                        CHG_ShutDown();
                        CHG_StopRecharge();
                        if(vprog_value < battery_data->charge_stop_point){
                            charge_pluse = true;
                            pluse_charging = 0;
                            battery_data->charging = 0;
                            battery_notify = 1;
                        }
                        pluse_charge_cnt = CHGMNG_PLUST_TIMES;
                    }
                }else{
                    charge_pluse = true;
                    CHG_TurnOn();
                    CHG_SetRecharge();
                }
            }
        }else{
            CHG_ShutDown();
            CHG_StopRecharge();
            battery_data->charging = 0;
            pluse_charging = 0;
            charge_pluse = true;
            battery_notify = 1;
            
        }
    }

    if(timer_freq_flag == 0){
        timer_freq_cnt --;
        if(timer_freq_cnt < 0){
            timer_freq_flag = 1;
            timer_freq = 10*HZ;
        }
    }

    spin_unlock_irqrestore(&battery_data->lock, flag);
    DEBUG("usb online %d, ac online %d\n", usb_online, ac_online);
    DEBUG("capacity %d\n", capacity);
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

static void charge_voltage_handler(unsigned long data)
{
    int32_t vprog_value;
    int32_t vchg_value;
    struct sprd_battery_data * battery_data = (struct sprd_battery_data *)data;

    vprog_value = ADC_GetValue(ADC_CHANNEL_PROG, false);
    if(vprog_value < 0)
      return;
    DEBUG("vprog %d\n", vprog_value);

    vchg_value = ADC_GetValue(ADC_CHANNEL_VCHG, false);
    if(vchg_value < 0)
      return;
    DEBUG("vchg %d\n", vchg_value);

    if(vchg_value > battery_data->over_voltage && vprog_value > battery_data->over_current){
        CHG_ShutDown();
        CHG_StopRecharge();
        printk("charger voltage too high\n");
        battery_data->ac_online = 0;
        battery_data->usb_online = 0;
    }
    if(battery_data->ac_online == 0 && battery_data->usb_online == 0)
        del_timer(&battery_data->charge_voltage_timer);
    else
        mod_timer(&battery_data->charge_voltage_timer, jiffies + HZ/10);
}

/* used to detect battery capacity status
 * return 1: need update
 *        0: don't need
 */
int battery_updata(void){
    int32_t adc_value;
    uint32_t capacity;
    static uint32_t pre_capacity = 0xffffffff;
    adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
    if(adc_value < 0)
      return;
    capacity = CHGMNG_VoltageToPercentum(adc_value);
    DEBUG("battery_update: capacity %d\n", capacity);
    if(pre_capacity == 0xffffffff){
        adc_value = get_vbat_value();
        pre_capacity = CHGMNG_VoltageToPercentum(adc_value);
    }

    if(pre_capacity != capacity){
        pre_capacity = capacity;
        update_vbat_value(adc_value);
        return 1;
    }else{
        return 0;
    }
}
static char * supply_list[]={
    "battery",
};

static int sprd_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct sprd_battery_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

    data->capacity = 100;
    data->charging = 0;

    data->over_voltage = CHARGE_OVER_VOLTAGE;
    data->over_current= CHARGE_OVER_CURRENT;
    data->precharge_start = PREVRECHARGE;
    data->precharge_end = PREVCHGEND;
    data->hw_switch_point = CHGMNG_SWITCH_CV_VPROG;
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

    init_timer(&data->charge_voltage_timer);
    data->charge_voltage_timer.function = charge_voltage_handler;
    data->charge_voltage_timer.data = (unsigned long)data;

    timer_freq = HZ/4;
    timer_freq_cnt = 20;
    if(usb_connected()){
        if(charger_is_adapter()){
            data->ac_online = 1;
            data->usb_online = 0;
        }
        else{
            data->usb_online = 1;
            data->ac_online = 0;
        }
        timer_freq_flag = 1;
        mod_timer(&data->charge_voltage_timer, HZ/10);
    }else{
        data->usb_online = 0;
        data->ac_online = 0;
        timer_freq_flag = 0;
    }

#if 0
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		DEBUG(KERN_ERR "%s: platform_get_resource failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_io_base;
	}
	data->reg_base = IO_ADDRESS(r->start - IO_START);

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		DEBUG(KERN_ERR "%s: platform_get_irq failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_irq;
	}
#endif
#ifdef BATTERY_USE_WAKE_LOCK
    wake_lock_init(&(data->charge_wake_lock), WAKE_LOCK_SUSPEND, "charge_wake_lock");
    wake_lock_init(&(data->update_wake_lock), WAKE_LOCK_SUSPEND, "update_wake_lock");
#endif
    ret = gpio_to_irq(CHARGER_DETECT_GPIO);
    data->irq = ret;      

	ret = request_irq(data->irq, sprd_battery_interrupt, IRQF_SHARED |
                        IRQF_TRIGGER_HIGH, pdev->name, data);
	if (ret)
		goto err_request_irq_failed;

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
    CHG_SetSwitchoverPoint (CHGMNG_DEFAULT_SWITPOINT);
    ADC_Init();

    sprd_creat_caliberate_attr(data->battery.dev);

    mod_timer(&data->battery_timer, jiffies + timer_freq);

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
    del_timer_sync(&data->charge_voltage_timer);
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
    int32_t adc_value;
    uint32_t capacity;

    adc_value = get_vbat_value();
    if(adc_value < 0)
      return 0;
    capacity = CHGMNG_VoltageToPercentum(adc_value);
    if(data->capacity != capacity){
        data->capacity = capacity;
        power_supply_changed(&battery_data->battery);
    }
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
