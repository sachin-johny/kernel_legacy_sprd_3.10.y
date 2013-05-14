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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/headset_sprd.h>
#include <mach/board.h>
#include <linux/input.h>

#include <mach/hardware.h>
#include <mach/adc.h>
#include <mach/adi.h>

#define ADC_FIFO_CNT	(5)
#define HEADSET_TYPE_ADC_DISPART_VALUE	(100)
#define HEADSET_TYPE_GND_VALUE		(100)

#define HEADMIC_DETECT_BASE	(SPRD_MISC_BASE	+ 0x700)
#define HEADMIC_DETECT_REG(X)   (HEADMIC_DETECT_BASE + (X))

#define HEADMIC_DETECT_GLB_REG(X)   (ANA_REGS_GLB_BASE + (X))

#define HEADMIC_DETECT_INSRT_VOL_SHIFT  (5)
#define HEADMIC_DETECT_INSRT_VOL_MSK    (0x3 << HEADMIC_DETECT_INSRT_VOL_SHIFT)

#define HEADMIC_DETECT_INSRT_2P1V   (3)
#define HEADMIC_DETECT_INSRT_2P3V   (2)
#define HEADMIC_DETECT_INSRT_2P5V   (1)
#define HEADMIC_DETECT_INSRT_2P7V   (0)


#define HEADMIC_ADC_SWITCH_BIT			(BIT(13))
#define HEADMIC_DETECT_CIRCUIT_BIT      (BIT(11))

#define ABS(x)	(((x) < (0)) ? (-x) : (x))

#define headset_reg_write(val, addr) \
	do {	\
		sci_adi_raw_write(addr, val)	\
	} while(0)

#define headset_reg_read(addr) \
	do {	\
		sci_adi_read(addr);	\
	} while(0)

#define headset_reg_msk_or(val, addr, msk)  \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = (temp & (~msk)) | val;   \
        sci_adi_raw_write(addr, temp);    \
    } while(0)

#define headset_reg_clr_bit(addr, bit)   \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = temp & (~bit);   \
        sci_adi_raw_write(addr, temp);  \
    } while(0)

#define headset_reg_set_bit(addr, bit)   \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = temp | bit;  \
        sci_adi_raw_write(addr, temp);  \
    } while(0)

typedef enum sprd_headset_type{
	HEADPHONE,
	HEADSET_NORMAL,
	HEADSET_NO_MIC,
	HEADSET_NORTH_AMERICA,
	HEADSET_APPLE,
	HEADSET_TYPE_MAX,
}SPRD_HEADSET_TYPE;

static struct sprd_headset headset = {
	.detect = {
		.sdev = {
			.name = "h2w",
		}
	},
};

static int irq_enable = 0;
extern int sprd_codec_headmic_bias_control(int on);
static void headset_button_work_func(struct work_struct *work)
{
	struct headset_button_data *ht_button = container_of(work, struct headset_button_data, work);
	int state;
	int plug_in;
	plug_in = gpio_get_value(257);
	if (!plug_in){
		printk("ignore the button when headset not plug in\n");
		return;
	}
	state = gpio_get_value(260);
	printk("headset_button_work_func %d\n",state);
	if(state) {
		input_event(ht_button->input, EV_KEY, KEY_MEDIA, 1);
	} else {
		input_event(ht_button->input, EV_KEY, KEY_MEDIA, 0);
	}
	input_sync(ht_button->input);
}

/*  on = 0: open headmic detect circuit */
static void headset_detect_circuit(unsigned on)
{
    if (on) {
        headset_reg_clr_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_CIRCUIT_BIT);
    } else {
        headset_reg_set_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_CIRCUIT_BIT);
    }
}

static void headset_detect_openclk(void)
{
    headset_reg_set_bit(HEADMIC_DETECT_GLB_REG(0x84), (BIT(14) | BIT(15)));
}

static void headset_detect_init(void)
{
    /* enable headset detect clk */
    headset_reg_set_bit(HEADMIC_DETECT_GLB_REG(0x84), (BIT(14) | BIT(15)));

    /* set headset detect voltage */
    headset_reg_msk_or(HEADMIC_DETECT_INSRT_2P1V, HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_INSRT_VOL_MSK);
}

/* is_set = 1, headset_mic to AUXADC */
static void set_adc_to_headmic(unsigned is_set)
{
	if (is_set) {
		headset_reg_set_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_ADC_SWITCH_BIT);
	} else {
		headset_reg_clr_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_ADC_SWITCH_BIT);
	}
}

static SPRD_HEADSET_TYPE detect_headset_type(void)
{
	/* distinguish headset type */
	int adc_mic, adc_l;
	int adc_val[ADC_FIFO_CNT] = {0};
	int count;

    headset_detect_init();
    headset_detect_circuit(1);

	set_adc_to_headmic(1);
	for (count = 0; count < ADC_FIFO_CNT; count++) {
		adc_val[count] = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
	}
	adc_mic = adc_val[ADC_FIFO_CNT/2];

	memset(adc_val, 0, sizeof(adc_val));

	set_adc_to_headmic(0);
	for (count = 0; count < ADC_FIFO_CNT; count++) {
		adc_val[count] = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
	}
	adc_l = adc_val[ADC_FIFO_CNT/2];

	if (ABS(adc_mic - adc_l) < HEADSET_TYPE_ADC_DISPART_VALUE) {
		if (ABS(adc_mic - 0) < HEADSET_TYPE_GND_VALUE) {
			return HEADSET_NO_MIC;
		} else {
			return HEADSET_NORTH_AMERICA;
		}
	}

	return HEADSET_NORMAL;
}

static int headset_plug_in(void)
{
	return 0;
}

static void headset_detect_work_func(struct work_struct *work)
{
	struct headset_detect_data *ht_detect = container_of(work, struct headset_detect_data, work);
	struct sprd_headset *ht = &headset;
	int state;
	SPRD_HEADSET_TYPE headset_type;

	state = headset_plug_in();

	if(state) {
		sprd_codec_headmic_bias_control(1);
		mdelay(20);

		headset_type = detect_headset_type();
		if(headset_type == HEADSET_NORTH_AMERICA)
			gpio_direction_output(144,1);

		if(headset_type == HEADSET_NORMAL)
			ht_detect->headphone = 1;
		else
			ht_detect->headphone = 0;

		if (ht_detect->headphone) {
			ht_detect->type = BIT_HEADSET_NO_MIC;
			switch_set_state(&ht_detect->sdev, ht_detect->type);
			pr_info("headphone plug in\n");
		} else {
			ht_detect->type = BIT_HEADSET_MIC;
			switch_set_state(&ht_detect->sdev, ht_detect->type);
			if(irq_enable == 0) {
				mod_timer(&ht->detect.irq_timer,
						jiffies + msecs_to_jiffies(500));
			}
		}
	} else {
		if (ht_detect->headphone) {
			pr_info("headphone plug out\n");
		}
		else {
			if(irq_enable == 1) {
				disable_irq(ht->button.irq);
				irq_enable = 0;
			}
			pr_info("headset plug out\n");
		}
		ht_detect->type = BIT_HEADSET_OUT;
		switch_set_state(&ht_detect->sdev, ht_detect->type);
	}
}

static irqreturn_t headset_button_irq_handler(int irq, void *dev)
{
	struct sprd_headset *ht = dev;

	if(ht->button.platform_data->active_low == 1) {
		irq_set_irq_type(ht->button.irq, IRQF_TRIGGER_LOW);
		ht->button.platform_data->active_low = 0;
	} else {
		irq_set_irq_type(ht->button.irq, IRQF_TRIGGER_HIGH);
		ht->button.platform_data->active_low = 1;
	}
	mod_timer(&ht->button.timer,
			jiffies + msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static irqreturn_t headset_detect_irq_handler(int irq, void *dev)
{
	struct sprd_headset *ht = dev;

	if (irq_enable == 1) {
		disable_irq(ht->button.irq);
		irq_enable = 0;
	}
	if(ht->detect.platform_data->active_low == 1) {
		irq_set_irq_type(ht->detect.irq, IRQF_TRIGGER_LOW);
		ht->detect.platform_data->active_low = 0;
	} else {
		irq_set_irq_type(ht->detect.irq, IRQF_TRIGGER_HIGH);
		ht->detect.platform_data->active_low = 1;
	}
	mod_timer(&ht->detect.timer,
			jiffies + msecs_to_jiffies(500));
	return IRQ_HANDLED;
}
static void headset_button_timer(unsigned long _data)
{
	struct sprd_headset *data = (struct prd_headset *)_data;

	schedule_work(&data->button.work);
}
static void headset_detect_timer(unsigned long _data)
{
	struct sprd_headset *data = (struct sprd_headset *)_data;

	schedule_work(&data->detect.work);
}

static void headset_detect_irq_timer(unsigned long _data)
{
	struct sprd_headset *data = (struct sprd_headset *)_data;
	enable_irq(data->button.irq);
	irq_enable = 1;
}

static __devinit int headset_detect_probe(struct platform_device *pdev)
{
	struct sprd_headset_detect_platform_data *pdata = pdev->dev.platform_data;
	//struct regulator *reg_vdd;
	struct sprd_headset *ht = &headset;
	int ret;
	printk("headset_detect_probe\n");
	ret = switch_dev_register(&ht->detect.sdev);
	if (ret < 0) {
		pr_err("switch_dev_register failed!\n");
		return ret;
	}
	ht->detect.platform_data = pdata;
	//gpio_request(pdata->gpio, "headset_switch");
	setup_timer(&ht->detect.timer, headset_detect_timer, (unsigned long)ht);
	setup_timer(&ht->detect.irq_timer, headset_detect_irq_timer, (unsigned long)ht);
	INIT_WORK(&ht->detect.work, headset_detect_work_func);
	gpio_request(257,"headset_detect");
	ht->detect.irq = gpio_to_irq(257);
	pdata->active_low = 1;
	ret = request_irq(ht->detect.irq, headset_detect_irq_handler,
					IRQF_TRIGGER_HIGH, "headset_detect", ht);
	return ret;
}

#if 1
static __devinit int headset_buttons_probe(struct platform_device *pdev)
{
	printk("headset_buttons_probe\n");
	struct sprd_headset_buttons_platform_data *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct sprd_headset *ht = &headset;
	int i;
	int ret;

	ht->button.input = input_allocate_device();
	if (!pdata || !ht->button.input) {
		dev_err(dev, "failed to allocate state\n");
		//error = -ENOMEM;
		//goto fail1;
	}
	ht->button.input->name = "headset-keyboard";
	ht->button.input->id.bustype = BUS_HOST;
	ht->button.platform_data = pdata;
	pdata->active_low = 1;
	setup_timer(&ht->button.timer, headset_button_timer, (unsigned long)ht);
	INIT_WORK(&ht->button.work, headset_button_work_func);
	for (i = 0; i < pdata->nbuttons; i++) {
		struct headset_button *button = &pdata->headset_button[i];
		unsigned int type = button->type ?: EV_KEY;
		input_set_capability(ht->button.input, type, button->code);
	}
	input_register_device(ht->button.input);
	gpio_request(260,"headset_button");
	ht->button.irq = gpio_to_irq(260);
	ret = request_irq(ht->button.irq, headset_button_irq_handler,
					IRQF_TRIGGER_HIGH, "headset_button", ht);
	disable_irq(ht->button.irq);
	return ret;
}
#endif
static struct platform_driver headset_detect_driver = {
	.driver = {
		.name = "headset-detect",
		.owner = THIS_MODULE,
	},
	.probe = headset_detect_probe,
};

static struct platform_driver headset_buttons_driver = {
	.driver = {
		.name = "headset-button",
		.owner = THIS_MODULE,
	},
	.probe = headset_buttons_probe,
};

static int __init headset_init(void)
{
	int ret, i;
	platform_driver_register(&headset_detect_driver);
	platform_driver_register(&headset_buttons_driver);
	return ret;
}
module_init(headset_init);

static void __exit headset_exit(void)
{
}
module_exit(headset_exit);

MODULE_DESCRIPTION("headset & button detect driver");
MODULE_LICENSE("GPL");
