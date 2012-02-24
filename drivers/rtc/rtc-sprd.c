/*
 * An RTC device/driver
 * Copyright (C) 2011 Spreadtrum Communication Inc 
 * Author: Mark Yang<markyang@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <mach/bits.h>
#include <mach/regs_rtc.h>
#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>
#include <mach/irqs.h>
#include <linux/delay.h>

#define CLEAR_RTC_INT(mask) \
	do{ ANA_REG_SET(ANA_RTC_INT_CLR, mask); \
		while(ANA_REG_GET(ANA_RTC_INT_RSTS) & mask); \
	}while(0)

static struct platform_device *sprd0 = NULL;
static unsigned long sec_2011_to_1970;

static inline unsigned get_sec(void)
{
    unsigned sec, sec_bak;
    sec = ANA_REG_GET(ANA_RTC_SEC_CNT) & RTC_SEC_MASK;
    do{
        sec_bak = ANA_REG_GET(ANA_RTC_SEC_CNT) & RTC_SEC_MASK;
        if(sec_bak == sec)
          break;
        else
          sec = sec_bak;
    }while(1);

    return sec;
}
static inline unsigned get_min(void)
{
    unsigned min, min_bak;
    min = ANA_REG_GET(ANA_RTC_MIN_CNT) & RTC_MIN_MASK;
    do{
        min_bak = ANA_REG_GET(ANA_RTC_MIN_CNT) & RTC_MIN_MASK;
        if(min_bak == min)
          break;
        else
          min = min_bak;
    }while(1);

    return min;
}
static inline unsigned get_hour(void)
{
    unsigned hour, hour_bak;
    hour = ANA_REG_GET(ANA_RTC_HOUR_CNT) & RTC_HOUR_MASK;
    do{
        hour_bak = ANA_REG_GET(ANA_RTC_HOUR_CNT) & RTC_HOUR_MASK;
        if(hour_bak == hour)
          break;
        else
          hour = hour_bak;
    }while(1);

    return hour;
}
static inline unsigned get_day(void)
{
    unsigned day, day_bak;
    day = ANA_REG_GET(ANA_RTC_DAY_CNT) & RTC_DAY_MASK;
    do{
        day_bak = ANA_REG_GET(ANA_RTC_DAY_CNT) & RTC_DAY_MASK;
        if(day == day_bak)
          break;
        else
          day = day_bak;
    }while(1);
    
    return day;
}

static inline unsigned long sprd_rtc_get_sec(void)
{
	unsigned sec, min, hour, day;
	unsigned first = 0, second = 0;

	do{
		sec = get_sec();
		min = get_min();
		hour = get_hour();
		day = get_day();

		second = ((((day*24) + hour)*60 + min)*60 + sec);
		if((second - first) == 0)
			break;
		first = second;
	}while(1);

	return first;
}
static inline void sprd_rtc_set_alarm_sec(unsigned long secs);
static inline void sprd_rtc_set_sec(unsigned long secs)
{
	unsigned sec, min, hour, day;
    unsigned set_mask = 0, int_rsts;
	unsigned long temp;

	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;


    ANA_REG_OR(ANA_RTC_INT_CLR, RTC_UPD_TIME_MASK);

    if(sec != get_sec()){
        ANA_REG_SET(ANA_RTC_SEC_UPDATE, sec);
        set_mask |= RTC_SEC_ACK_BIT;
    }
    if(min != get_min()){
        ANA_REG_SET(ANA_RTC_MIN_UPDATE, min);
        set_mask |= RTC_MIN_ACK_BIT;
    }
    if(hour != get_hour()){
        ANA_REG_SET(ANA_RTC_HOUR_UPDATE, hour);
        set_mask |= RTC_HOUR_ACK_BIT;
    }
    if(day != get_day()){
        ANA_REG_SET(ANA_RTC_DAY_UPDATE, day);
        set_mask |= RTC_DAY_ACK_BIT;
    }

    //wait till all update done

    do{
        int_rsts = ANA_REG_GET(ANA_RTC_INT_RSTS) & RTC_UPD_TIME_MASK;

        if(set_mask == int_rsts)
          break;
    }while(1);
    ANA_REG_OR(ANA_RTC_INT_CLR, RTC_UPD_TIME_MASK);

	return;
}

static inline unsigned long sprd_rtc_get_alarm_sec(void)
{
	unsigned sec, min, hour, day;
	day = ANA_REG_GET(ANA_RTC_DAY_ALM) & RTC_DAY_MASK;
	hour = ANA_REG_GET(ANA_RTC_HOUR_ALM) & RTC_HOUR_MASK;
	min = ANA_REG_GET(ANA_RTC_MIN_ALM) & RTC_MIN_MASK;
	sec = ANA_REG_GET(ANA_RTC_SEC_ALM) & RTC_SEC_MASK;

	return ((((day*24) + hour)*60 + min)*60 + sec);
}
static inline void sprd_rtc_set_alarm_sec(unsigned long secs)
{
	unsigned sec, min, hour, day;
	unsigned long temp;
	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;
	ANA_REG_SET(ANA_RTC_SEC_ALM, sec);
	ANA_REG_SET(ANA_RTC_MIN_ALM, min);
	ANA_REG_SET(ANA_RTC_HOUR_ALM, hour);
	ANA_REG_SET(ANA_RTC_DAY_ALM, day);

	return;
}
static int sprd_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	unsigned long secs = sprd_rtc_get_alarm_sec();
    secs = secs + sec_2011_to_1970;
	rtc_time_to_tm(secs, &alrm->time);

	alrm->enabled = !!(ANA_REG_GET(ANA_RTC_INT_EN) & RTC_ALARM_BIT);
	alrm->pending = !!(ANA_REG_GET(ANA_RTC_INT_RSTS) & RTC_ALARM_BIT);

	return 0;
}

static int sprd_rtc_set_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	unsigned long secs;
	unsigned temp;
	unsigned long read_secs;
	int i = 0;
	rtc_tm_to_time(&alrm->time, &secs);
    if(secs < sec_2011_to_1970)
      return -1;

	ANA_REG_SET(ANA_RTC_INT_CLR, RTC_ALARM_BIT);

	if(alrm->enabled){
		temp = ANA_REG_GET(ANA_RTC_INT_EN);
		temp |= RTC_ALARM_BIT;
		ANA_REG_SET(ANA_RTC_INT_EN, temp);

        secs = secs - sec_2011_to_1970;
        sprd_rtc_set_alarm_sec(secs);
		msleep(150);
		do {
			read_secs = sprd_rtc_get_alarm_sec();
			msleep(1);
			i++;
		}while(read_secs != secs && i < 100);
	}else{
        ANA_REG_AND(ANA_RTC_INT_EN, ~(RTC_ALARM_BIT));
		msleep(150);
    }

	return 0;
}

static int sprd_rtc_read_time(struct device *dev,
	struct rtc_time *tm)
{
	unsigned long secs = sprd_rtc_get_sec();
    if(secs > 0x7f000000){
        sprd_rtc_set_sec(0);
        secs = 0;
    }
    secs = secs + sec_2011_to_1970;
    if(secs > 0x7f000000){
        secs = sec_2011_to_1970;
        sprd_rtc_set_sec(0);
    }
	rtc_time_to_tm(secs, tm);
	return 0;
}

static int sprd_rtc_set_time(struct device *dev, 
		struct rtc_time *tm)
{
	unsigned long secs;
	rtc_tm_to_time(tm, &secs);
    if(secs < sec_2011_to_1970)
      return -1;
    secs = secs - sec_2011_to_1970;
	sprd_rtc_set_sec(secs);
	return 0;
}

static int sprd_rtc_set_mmss(struct device *dev, unsigned long secs)
{
    if(secs < sec_2011_to_1970)
      return -1;
    secs = secs - sec_2011_to_1970;
	sprd_rtc_set_sec(secs);
	return 0;
}

static int sprd_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct platform_device *plat_dev = to_platform_device(dev);

	seq_printf(seq, "sprd_rtc\t: yes\n");
	seq_printf(seq, "id\t\t: %d\n", plat_dev->id);

	return 0;
}
static irqreturn_t rtc_interrupt_handler(int irq, void *dev_id)
{
    struct rtc_device * rdev = dev_id;
    printk(" RTC ***** interrupt happen\n");
    rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);
    CLEAR_RTC_INT(RTC_INT_ALL_MSK);
    return IRQ_HANDLED;
}

static int sprd_rtc_open(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct rtc_device * rtc_dev = platform_get_drvdata(pdev);
    int ret;
    unsigned temp;


    //ret = request_irq(IRQ_ANA_RTC_INT, rtc_interrupt_handler, 0, "sprd_rtc", rtc_dev);
    //if(ret){
    //    printk("RTC regist irq error\n");
    //    return ret;
    //}
    /* enable rtc interrupt */
    temp = ANA_REG_GET(ANA_RTC_INT_EN);
    temp |= RTC_ALARM_BIT;
    ANA_REG_SET(ANA_RTC_INT_EN, temp);
    return ret;
}

static const struct rtc_class_ops sprd_rtc_ops = {
    .open = sprd_rtc_open,
	.proc = sprd_rtc_proc,
	.read_time = sprd_rtc_read_time,
	.read_alarm = sprd_rtc_read_alarm,
	.set_time = sprd_rtc_set_time,
	.set_alarm = sprd_rtc_set_alarm,
	.set_mmss = sprd_rtc_set_mmss,
//	.ioctl = sprd_rtc_ioctl,
};


static int sprd_rtc_probe(struct platform_device *plat_dev)
{
	int err;
    struct rtc_device * rtc = NULL;
	ANA_REG_AND(ANA_RTC_INT_EN, ~(RTC_INT_ALL_MSK)); // disable all interrupt
	ANA_REG_OR(ANA_AGEN, AGEN_RTC_EN | AGEN_RTC_RTC_EN); //enable rtc device
	CLEAR_RTC_INT(RTC_INT_ALL_MSK);
	rtc = rtc_device_register("sprd_rtc", &plat_dev->dev,
						&sprd_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		return err;
	}
    err = request_irq(IRQ_ANA_RTC_INT, rtc_interrupt_handler, 0, "sprd_rtc", rtc);
    if(err != 0){
        printk(" rtc irq request error:%d \n", err);
        rtc_device_unregister(rtc);
        return err;
    }


	platform_set_drvdata(plat_dev, rtc);

	return 0;
}

static int __devexit sprd_rtc_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);

	rtc_device_unregister(rtc);

	return 0;
}

static struct platform_driver sprd_rtc_driver = {
	.probe	= sprd_rtc_probe,
	.remove = __devexit_p(sprd_rtc_remove),
	.driver = {
		.name = "rtc-sprd",
		.owner = THIS_MODULE,
	},
};

static int __init sprd_rtc_init(void)
{
	int err;

	if ((err = platform_driver_register(&sprd_rtc_driver)))
		return err;

	if ((sprd0 = platform_device_alloc("rtc-sprd", 0)) == NULL) {
		err = -ENOMEM;
		goto exit_driver_unregister;
	}

	if ((err = platform_device_add(sprd0)))
		goto exit_free_sprd0;

    sec_2011_to_1970 = mktime(2011, 1, 1, 0, 0, 0);

	return 0;

exit_free_sprd0:
	platform_device_put(sprd0);

exit_driver_unregister:
	platform_driver_unregister(&sprd_rtc_driver);
	return err;
}

static void __exit sprd_rtc_exit(void)
{
	platform_device_unregister(sprd0);
	platform_driver_unregister(&sprd_rtc_driver);
}

MODULE_AUTHOR("Mark Yang <markyang@spreadtrum.com");
MODULE_DESCRIPTION("RTC SC8800G driver/device");
MODULE_LICENSE("GPL");

module_init(sprd_rtc_init);
module_exit(sprd_rtc_exit);
