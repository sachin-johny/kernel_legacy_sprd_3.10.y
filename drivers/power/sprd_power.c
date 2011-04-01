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

//#define CHG_DEBUG
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

    uint32_t capacity;
    uint32_t charging;
    uint32_t ac_online;
    uint32_t usb_online;

    uint32_t precharge_start;
    uint32_t precharge_end;
    uint32_t over_voltage;
    uint32_t hw_switch_point;
    uint32_t charge_stop_point;

	struct power_supply battery;
	struct power_supply ac;
	struct power_supply usb;
};

/* temporary variable used between sprd_battery_probe() and sprd_battery_open() */
static struct sprd_battery_data *battery_data;


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
static struct device_attribute sprd_caliberate[]={
    SPRD_CALIBERATE_ATTR(precharge_start),
    SPRD_CALIBERATE_ATTR(precharge_end),
    SPRD_CALIBERATE_ATTR(over_voltage),
    SPRD_CALIBERATE_ATTR(hw_switch_point),
    SPRD_CALIBERATE_ATTR(charge_stop_point),
    SPRD_CALIBERATE_ATTR(capacity_0),
    SPRD_CALIBERATE_ATTR(capacity_20),
    SPRD_CALIBERATE_ATTR(capacity_40),
    SPRD_CALIBERATE_ATTR(capacity_60),
    SPRD_CALIBERATE_ATTR(capacity_80),
    SPRD_CALIBERATE_ATTR(capacity_100),
};
static enum {
    PRECHARGE_START = 0,
    PRECHARGE_END,
    OVER_VOLTAGE,
    HW_SWITCH_POINT,
    CHARGE_STOP_POINT,
    CAPACITY_0,
    CAPACITY_20,
    CAPACITY_40,
    CAPACITY_60,
    CAPACITY_80,
    CAPACITY_100,
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
        pr_err("set value %d too big\n", set_value);
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
    case HW_SWITCH_POINT:
        battery_data->hw_switch_point = set_value;
        break;
    case CHARGE_STOP_POINT:
        battery_data->charge_stop_point = set_value;
        break;
    case CAPACITY_0:
        voltage_capacity_table[0] = set_value;
        break;
    case CAPACITY_20:
        voltage_capacity_table[2] = set_value;
        break;
    case CAPACITY_40:
        voltage_capacity_table[4] = set_value;
        break;
    case CAPACITY_60:
        voltage_capacity_table[6] = set_value;
        break;
    case CAPACITY_80:
        voltage_capacity_table[8] = set_value;
        break;
    case CAPACITY_100:
        voltage_capacity_table[10] = set_value;
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
    case CAPACITY_20:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[2]);
        break;
    case CAPACITY_40:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[4]);
        break;
    case CAPACITY_60:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[6]);
        break;
    case CAPACITY_80:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[8]);
        break;
    case CAPACITY_100:
        i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                    voltage_capacity_table[10]);
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

static unsigned long chg_gpio_cfg =
    MFP_ANA_CFG_X(CHIP_RSTN, AF0, DS1, F_PULL_UP,S_PULL_UP, IO_IE);

#define USB_CONNECT_GPIO 162
static inline int usb_connected(void)
{
    return gpio_get_value(USB_CONNECT_GPIO)&BIT_2? 1:0;
}
static irqreturn_t sprd_battery_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct sprd_battery_data *data = dev_id;
	uint32_t status;
    uint32_t charger_status;
    DEBUG("charger plug interrupt happen\n");

	spin_lock_irqsave(&data->lock, irq_flags);

    charger_status = usb_connected();
    data->usb_online = charger_status;
    set_irq_type(irq, charger_status? IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH);
	spin_unlock_irqrestore(&data->lock, irq_flags);
	return IRQ_HANDLED;
}

static inline int ac_connected(void)
{
    return 0;
}

static CHG_SWITPOINT_E now_hw_switch_point;
static bool charge_pluse = false;

static void battery_handler(unsigned long data)
{
    uint32_t battery;
    uint32_t voltage;
    uint32_t capacity;
    uint32_t adc_value;
    uint32_t vprog_value;
    uint32_t vchg_value;
    int usb_online= 0;
    static int pre_usb_online = 0;
    int ac_online = 0;
    static int pre_ac_online = 0;
    int ac_notify = 0;
    int usb_notify = 0;
    int battery_notify = 0;
    unsigned long flag;
    
    
    struct sprd_battery_data * battery_data = (struct sprd_battery_data *)data;
    usb_online = battery_data->usb_online;
    ac_online = ac_connected();
    adc_value = ADC_GetValue(ADC_CHANNEL_VBAT, false);
    DEBUG("vbat %d\n", adc_value);
    vprog_value = ADC_GetValue(ADC_CHANNEL_PROG, false);
    DEBUG("vprog %d\n", vprog_value);
    vchg_value = ADC_GetValue(ADC_CHANNEL_VCHG, false);
    DEBUG("vchg %d\n", vchg_value);
    voltage = CHGMNG_AdcvalueToVoltage(adc_value);
    DEBUG("voltage %d\n", voltage);
    //update capity;
    //notify user space
    spin_lock_irqsave(&battery_data->lock, flag);
    capacity = CHGMNG_VoltageToPercentum(adc_value);
    DEBUG("capacity %d\n", capacity);
    DEBUG("now_hw_switch_point %d\n", now_hw_switch_point);
    if(vchg_value > battery_data->over_voltage){
    //if(vchg_value > CHARGE_OVER_VOLTAGE){
        printk("charger voltage too high\n");
        ac_online = 0;
        usb_online = 0;
    }

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

    if(usb_online && (adc_value < battery_data->precharge_start) && !battery_data->charging){
    //if(usb_online && (adc_value < PREVRECHARGE) && !battery_data->charging){
        battery_data->charging = 1;
        CHG_SetAdapterMode(CHG_USB_ADAPTER);
        CHG_SetUSBChargeCurrent(CHG_USB_400MA);
        CHG_TurnOn();
        CHG_SetRecharge();
        battery_notify = 1;
    }

    if(ac_online && (adc_value < battery_data->precharge_start) && !battery_data->charging){
    //if(ac_online && (adc_value < PREVRECHARGE) && !battery_data->charging){
        battery_data->charging = 1;
        CHG_SetAdapterMode(CHG_NORMAL_ADAPTER);
        CHG_SetNormalChargeCurrent(CHG_NOR_800MA);
        CHG_TurnOn();
        CHG_SetRecharge();
        battery_notify = 1;
    }

    if(battery_data->charging){
        if(ac_online || usb_online){
            if(adc_value < battery_data->precharge_end) {
            //if(adc_value < PREVCHGEND) {
                if(vprog_value < battery_data->hw_switch_point){
                //if(vprog_value < CHGMNG_SWITCH_CV_VPROG){
                    now_hw_switch_point = CHG_UpdateSwitchoverPoint(true);
                }
            }else {
                if(vprog_value >= battery_data->charge_stop_point){
                //if(vprog_value >= CHGMNG_STOP_VPROG){
                    if(charge_pluse){
                      CHG_SetRecharge();
                      charge_pluse = false;
                    }else{
                        CHG_ShutDown();
                        charge_pluse = true;
                    }
                }else{
                    CHG_ShutDown();
                    CHG_StopRecharge();
                    battery_data->charging = 0;
                    battery_notify = 1;
                }
            }
        }else{
            CHG_ShutDown();
            CHG_StopRecharge();
            battery_data->charging = 0;
            battery_notify = 1;
            
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
    mod_timer(&battery_data->battery_timer, jiffies + HZ);

}
static char * supply_list[]={
    "battery",
};

static int sprd_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct sprd_battery_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

    data->capacity = 100;
    data->ac_online = 0;
    data->usb_online = 0;
    data->charging = 0;

    data->over_voltage = CHARGE_OVER_VOLTAGE;
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
    sprd_mfp_config(&chg_gpio_cfg, 1);
    ret = gpio_request(USB_CONNECT_GPIO, "charger_plug");
    if(ret)
      goto err_alloc_gpio;
    gpio_direction_input(USB_CONNECT_GPIO);
    ret = sprd_alloc_gpio_irq(USB_CONNECT_GPIO);
    if(ret < 0)
      goto err_alloc_gpio_irq;
    else
      data->irq = ret;
      
    set_irq_type(data->irq, IRQ_TYPE_LEVEL_LOW); //set usb plug irq lowlevel

	ret = request_irq(data->irq, sprd_battery_interrupt, IRQF_SHARED, pdev->name, data);
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

    mod_timer(&data->battery_timer, jiffies + HZ);

	return 0;

err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
    power_supply_unregister(&data->usb);
err_usb_failed:
	free_irq(data->irq, data);
err_request_irq_failed:
    sprd_free_gpio_irq(data->irq);
err_alloc_gpio_irq:
    gpio_free(USB_CONNECT_GPIO);
err_alloc_gpio:
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
    gpio_free(USB_CONNECT_GPIO);

    del_timer_sync(&data->battery_timer);
	free_irq(data->irq, data);
	kfree(data);
	battery_data = NULL;
	return 0;
}

static struct platform_driver sprd_battery_device = {
	.probe		= sprd_battery_probe,
	.remove		= sprd_battery_remove,
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
