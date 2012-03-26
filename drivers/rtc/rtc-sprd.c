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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <mach/adi.h>
#include <mach/irqs.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <asm/bitops.h>
#include <linux/clkdev.h>
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

/* RTC_BASE      0x82000080 */
#define RTC_BASE (SPRD_MISC_BASE + 0x80)
#define ANA_RTC_SEC_CNT                 (RTC_BASE + 0x00)
#define ANA_RTC_MIN_CNT                 (RTC_BASE + 0x04)
#define ANA_RTC_HOUR_CNT                (RTC_BASE + 0x08)
#define ANA_RTC_DAY_CNT                 (RTC_BASE + 0x0C)
#define ANA_RTC_SEC_UPDATE              (RTC_BASE + 0x10)
#define ANA_RTC_MIN_UPDATE              (RTC_BASE + 0x14)
#define ANA_RTC_HOUR_UPDATE             (RTC_BASE + 0x18)
#define ANA_RTC_DAY_UPDATE              (RTC_BASE + 0x1C)
#define ANA_RTC_SEC_ALM                 (RTC_BASE + 0x20)
#define ANA_RTC_MIN_ALM                 (RTC_BASE + 0x24)
#define ANA_RTC_HOUR_ALM                (RTC_BASE + 0x28)
#define ANA_RTC_DAY_ALM                 (RTC_BASE + 0x2C)
#define ANA_RTC_INT_EN                  (RTC_BASE + 0x30)
#define ANA_RTC_INT_RSTS                (RTC_BASE + 0x34)
#define ANA_RTC_INT_CLR                 (RTC_BASE + 0x38)
#define ANA_RTC_INT_MSK                 (RTC_BASE + 0x3C)

/* The corresponding bit of RTC_CTL register. */
#define RTC_SEC_BIT                 BIT(0)        /* Sec int enable */
#define RTC_MIN_BIT                 BIT(1)        /* Min int enable */
#define RTC_HOUR_BIT                BIT(2)        /* Hour int enable */
#define RTC_DAY_BIT                 BIT(3)        /* Day int enable */
#define RTC_ALARM_BIT               BIT(4)        /* Alarm int enable */
#define RTCCTL_HOUR_FMT_SEL         BIT(5)        /* Hour format select */

#define RTC_SEC_ACK_BIT             BIT(8)        /* Sec ack int enable */
#define RTC_MIN_ACK_BIT             BIT(9)        /* Min ack int enable */
#define RTC_HOUR_ACK_BIT            BIT(10)        /* Hour ack int enable */
#define RTC_DAY_ACK_BIT             BIT(11)        /* Day ack int enable */
#define RTC_SEC_ALM_ACK_BIT         BIT(12)        /* Sec alm ack int enable */
#define RTC_MIN_ALM_ACK_BIT         BIT(13)        /* Min alm ack int enable */
#define RTC_HOUR_ALM_ACK_BIT        BIT(14)        /* Hour alm ack int enable */
#define RTC_DAY_ALM_ACK_BIT         BIT(15)        /* Day alm ack int enable */

#define RTC_UPD_TIME_MASK (RTC_SEC_ACK_BIT | RTC_MIN_ACK_BIT | RTC_HOUR_ACK_BIT | RTC_DAY_ACK_BIT)
#define RTC_INT_ALL_MSK (0xFFFF&(~(BIT(5)|BIT(6)|BIT(7))))

#define RTC_SEC_MASK 0x3F
#define RTC_MIN_MASK 0x3F
#define RTC_HOUR_MASK 0x1F
#define RTC_DAY_MASK 0xFFFF

#define SPRD_RTC_GET_MAX 10
#define SPRD_RTC_SET_MAX 150

#define CLEAR_RTC_INT(mask) \
	do{ sci_adi_raw_write(ANA_RTC_INT_CLR, mask); \
		while(sci_adi_read(ANA_RTC_INT_RSTS) & mask); \
	}while(0)

/* FIXME */
#define	  SPRD_ANA_BASE 	   (SPRD_MISC_BASE + 0x600)
#define   ANA_REG_BASE         SPRD_ANA_BASE   /*  0x82000600 */
#define   ANA_AGEN              (ANA_REG_BASE + 0x00)
static unsigned long sec_2012_to_1970;
struct sprd_rtc_data{
	struct rtc_device *rtc;
	unsigned int irq_no;
	struct clk *clk;
	struct regulator *regulator;
};

static struct sprd_rtc_data *rtc_data;

static inline unsigned get_sec(void)
{
	return sci_adi_read(ANA_RTC_SEC_CNT) & RTC_SEC_MASK;
}
static inline unsigned get_min(void)
{
	return sci_adi_read(ANA_RTC_MIN_CNT) & RTC_MIN_MASK;
}
static inline unsigned get_hour(void)
{
	return sci_adi_read(ANA_RTC_HOUR_CNT) & RTC_HOUR_MASK;
}
static inline unsigned get_day(void)
{
	return sci_adi_read(ANA_RTC_DAY_CNT) & RTC_DAY_MASK;
}

static unsigned long sprd_rtc_get_sec(void)
{
	unsigned sec, min, hour, day;
	unsigned first = 0, second = 0;
	int i = 0;
	do{
		sec = get_sec();
		min = get_min();
		hour = get_hour();
		day = get_day();

		second = ((((day*24) + hour)*60 + min)*60 + sec);
		if((second - first) == 0)
			break;
		first = second;

		i++;
	}while(i < SPRD_RTC_GET_MAX);

	return first;
}
static void sprd_rtc_set_sec(unsigned long secs)
{
	unsigned sec, min, hour, day;
	unsigned set_mask = 0, int_rsts;
	unsigned long temp;
	int i = 0;

	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;


	sci_adi_set(ANA_RTC_INT_CLR, RTC_UPD_TIME_MASK);

	if(sec != get_sec()){
		sci_adi_raw_write(ANA_RTC_SEC_UPDATE, sec);
		set_mask |= RTC_SEC_ACK_BIT;
	}
	if(min != get_min()){
		sci_adi_raw_write(ANA_RTC_MIN_UPDATE, min);
		set_mask |= RTC_MIN_ACK_BIT;
	}
	if(hour != get_hour()){
		sci_adi_raw_write(ANA_RTC_HOUR_UPDATE, hour);
		set_mask |= RTC_HOUR_ACK_BIT;
	}
	if(day != get_day()){
		sci_adi_raw_write(ANA_RTC_DAY_UPDATE, day);
		set_mask |= RTC_DAY_ACK_BIT;
	}

	/*
	 * wait till all update done
	 */

	do{
		int_rsts = sci_adi_read(ANA_RTC_INT_RSTS) & RTC_UPD_TIME_MASK;

		if(set_mask == int_rsts)
			break;

		if(i < SPRD_RTC_SET_MAX){
			msleep(1);
			i++;
		}else{
			break;
		}
	}while(1);
	sci_adi_set(ANA_RTC_INT_CLR, RTC_UPD_TIME_MASK);

	return;
}

static inline unsigned long sprd_rtc_get_alarm_sec(void)
{
	unsigned sec, min, hour, day;
	day = sci_adi_read(ANA_RTC_DAY_ALM) & RTC_DAY_MASK;
	hour = sci_adi_read(ANA_RTC_HOUR_ALM) & RTC_HOUR_MASK;
	min = sci_adi_read(ANA_RTC_MIN_ALM) & RTC_MIN_MASK;
	sec = sci_adi_read(ANA_RTC_SEC_ALM) & RTC_SEC_MASK;

	return ((((day*24) + hour)*60 + min)*60 + sec);
}
static void sprd_rtc_set_alarm_sec(unsigned long secs)
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
	sci_adi_raw_write(ANA_RTC_SEC_ALM, sec);
	sci_adi_raw_write(ANA_RTC_MIN_ALM, min);
	sci_adi_raw_write(ANA_RTC_HOUR_ALM, hour);
	sci_adi_raw_write(ANA_RTC_DAY_ALM, day);

	return;
}
static int sprd_rtc_read_alarm(struct device *dev,
		struct rtc_wkalrm *alrm)
{
	unsigned long secs = sprd_rtc_get_alarm_sec();

	secs = secs + sec_2012_to_1970;
	rtc_time_to_tm(secs, &alrm->time);

	alrm->enabled = !!(sci_adi_read(ANA_RTC_INT_EN) & RTC_ALARM_BIT);
	alrm->pending = !!(sci_adi_read(ANA_RTC_INT_RSTS) & RTC_ALARM_BIT);

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
	if(secs < sec_2012_to_1970)
		return -1;

	sci_adi_raw_write(ANA_RTC_INT_CLR, RTC_ALARM_BIT);

	if(alrm->enabled){
		temp = sci_adi_read(ANA_RTC_INT_EN);
		temp |= RTC_ALARM_BIT;
		sci_adi_raw_write(ANA_RTC_INT_EN, temp);

		secs = secs - sec_2012_to_1970;
		sprd_rtc_set_alarm_sec(secs);
		do {
			read_secs = sprd_rtc_get_alarm_sec();
			msleep(1);
			i++;
		}while(read_secs != secs && i < SPRD_RTC_SET_MAX);
	}else{
		sci_adi_clr(ANA_RTC_INT_EN, RTC_ALARM_BIT);
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
	secs = secs + sec_2012_to_1970;
	if(secs > 0x7f000000){
		secs = sec_2012_to_1970;
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
	if(secs < sec_2012_to_1970)
		return -1;
	secs = secs - sec_2012_to_1970;
	sprd_rtc_set_sec(secs);
	return 0;
}

static int sprd_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	if(secs < sec_2012_to_1970)
		return -1;
	secs = secs - sec_2012_to_1970;
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
	struct rtc_device *rdev = dev_id;

	pr_debug(" RTC ***** interrupt happen\n");
	rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);
	CLEAR_RTC_INT(RTC_INT_ALL_MSK);
	return IRQ_HANDLED;
}

static int sprd_rtc_open(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sprd_rtc_data *rtc_data = platform_get_drvdata(pdev);
	int ret;
	unsigned temp;

	ret = request_irq(rtc_data->irq_no, rtc_interrupt_handler, 0, "sprd_rtc", rtc_data->rtc);
	if(ret){
		printk(KERN_ERR "RTC regist irq error\n");
		return ret;
	}
	/* enable rtc interrupt */
	temp = sci_adi_read(ANA_RTC_INT_EN);
	temp |= RTC_ALARM_BIT;
	sci_adi_raw_write(ANA_RTC_INT_EN, temp);
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
};


static int sprd_rtc_probe(struct platform_device *plat_dev)
{
	int err = -ENODEV;
	struct resource *irq;

	rtc_data = kzalloc(sizeof(*rtc_data), GFP_KERNEL);
	if(IS_ERR(rtc_data)){
		err = PTR_ERR(rtc_data);
		return err;
	};
	/* disable all interrupt */
	sci_adi_clr(ANA_RTC_INT_EN, RTC_INT_ALL_MSK);
	/* enable rtc device */
	rtc_data->clk = clk_get(&plat_dev->dev, "ext_32k");
	if (IS_ERR(rtc_data->clk)) {
		err = PTR_ERR(rtc_data->clk);
		goto kfree_data;
	}

	err = clk_enable(rtc_data->clk);
	if (err < 0)
		goto put_clk;

	CLEAR_RTC_INT(RTC_INT_ALL_MSK);
	rtc_data->rtc = rtc_device_register("sprd_rtc", &plat_dev->dev,
			&sprd_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_data->rtc)) {
		err = PTR_ERR(rtc_data->rtc);
		goto disable_clk;
	}

	irq = platform_get_resource(plat_dev, IORESOURCE_IRQ, 0);
	if(unlikely(!irq)) {
		dev_err(&plat_dev->dev, "no irq resource specified\n");
		goto unregister_rtc;
	}
	rtc_data->irq_no = irq->start;
	platform_set_drvdata(plat_dev, rtc_data);

	return 0;

unregister_rtc:
	rtc_device_unregister(rtc_data->rtc);
disable_clk:
	clk_disable(rtc_data->clk);
put_clk:
	clk_put(rtc_data->clk);
kfree_data:
	kfree(rtc_data);
	return err;
}

static int __devexit sprd_rtc_remove(struct platform_device *plat_dev)
{
	struct sprd_rtc_data *rtc_data = platform_get_drvdata(plat_dev);

	rtc_device_unregister(rtc_data->rtc);
	clk_disable(rtc_data->clk);
	clk_put(rtc_data->clk);
	kfree(rtc_data);

	return 0;
}

static struct platform_driver sprd_rtc_driver = {
	.probe	= sprd_rtc_probe,
	.remove = __devexit_p(sprd_rtc_remove),
	.driver = {
		.name = "sprd_rtc",
		.owner = THIS_MODULE,
	},
};

static int __init sprd_rtc_init(void)
{
	int err;

	if ((err = platform_driver_register(&sprd_rtc_driver)))
		return err;

	if(CONFIG_RTC_START_YEAR > 1970)
		sec_2012_to_1970 = mktime(CONFIG_RTC_START_YEAR, 1, 1, 0, 0, 0);
	else 
		sec_2012_to_1970 = mktime(1970, 1, 1, 0, 0, 0);

	return 0;
}

static void __exit sprd_rtc_exit(void)
{
	platform_driver_unregister(&sprd_rtc_driver);
}

MODULE_AUTHOR("Mark Yang <markyang@spreadtrum.com");
MODULE_DESCRIPTION("RTC driver/device");
MODULE_LICENSE("GPL");

module_init(sprd_rtc_init);
module_exit(sprd_rtc_exit);
