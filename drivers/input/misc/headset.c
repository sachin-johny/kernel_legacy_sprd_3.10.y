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
#include <linux/switch.h>
#include <linux/input.h>
#include <mach/gpio.h>
#include <linux/headset.h>
#include <mach/board.h>

#ifndef HEADSET_DETECT_GPIO
#define HEADSET_DETECT_GPIO 165
#endif
#ifndef HEADSET_BUTTON_GPIO
#define HEADSET_BUTTON_GPIO 164
#endif

#ifndef HEADSET_DETECT_GPIO_ACTIVE_LOW
#define HEADSET_DETECT_GPIO_ACTIVE_LOW 1
#endif
#ifndef HEADSET_BUTTON_GPIO_ACTIVE_LOW
#define HEADSET_BUTTON_GPIO_ACTIVE_LOW 0
#endif

#ifndef HEADSET_DETECT_GPIO_DEBOUNCE_SW
#define HEADSET_DETECT_GPIO_DEBOUNCE_SW 1000
#endif
#ifndef HEADSET_BUTTON_GPIO_DEBOUNCE_SW
#define HEADSET_BUTTON_GPIO_DEBOUNCE_SW 100
#endif

static struct _headset headset = {
	.sdev = {
		.name = "h2w",
	},
	.detect = {
		.active_low = HEADSET_DETECT_GPIO_ACTIVE_LOW,
		.gpio = HEADSET_DETECT_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_DETECT_GPIO_DEBOUNCE_SW,
		.desc = "headset detect",
		.irq_enabled = 1,
	},
	.button = {
		.active_low = HEADSET_BUTTON_GPIO_ACTIVE_LOW,
		.gpio = HEADSET_BUTTON_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_BUTTON_GPIO_DEBOUNCE_SW,
		.desc = "headset button",
		.irq_enabled = 1,
		.timeout_ms = 800, /* 800ms for long button down */
	},
};

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp);
#define HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME	20
static enum hrtimer_restart report_headset_button_status(int active, struct _headset_gpio *hgp)
{
	enum hrtimer_restart restart;
	int code = -1;
	int report_status = 0;
	int actived_count = 0;
	static int pre_code = KEY_RESERVED;
	if (active) {
		if (active < 0) {
			hgp->holded = 0;
			hgp->actived = 0;
			hgp->actived_count = 0;
			pre_code = KEY_RESERVED;
			return HRTIMER_NORESTART;
		}
		hgp->actived_count++;
		restart = HRTIMER_RESTART;
		if ((++hgp->actived * hgp->debounce_sw) >= hgp->timeout_ms) {
			if (pre_code != KEY_END) {
				pre_code = code = KEY_END;
				report_status = 1;
				actived_count = hgp->actived_count;
			}
		} else
			pre_code = code = KEY_MEDIA;
	} else {
		restart = HRTIMER_NORESTART;
		if (pre_code == KEY_MEDIA) {
			report_status = 1;
			code = pre_code;
			actived_count = hgp->actived_count;
		}
		hgp->actived_count = 0;
		hgp->actived = 0;
		pre_code = KEY_RESERVED;
	}
	if (report_status) {
		input_event(hgp->parent->input, EV_KEY, code, 1);
		input_sync(hgp->parent->input);
		input_event(hgp->parent->input, EV_KEY, code, 0);
		input_sync(hgp->parent->input);
		pr_info("headset button-%d[%dms]\n", code, actived_count * hgp->debounce_sw);
	}
	return restart;
}

static enum hrtimer_restart report_headset_detect_status(int active, struct _headset_gpio *hgp)
{
	if (active) {
		hgp->parent->headphone = 0; /* hgp->parent->button.active_low ^ gpio_get_value(hgp->parent->button.gpio); */
		if (hgp->parent->headphone) {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_NO_MIC);
			pr_info("headphone plug in\n");
		} else {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_MIC);
			pr_info("headset plug in\n");
			irq_set_irq_type(hgp->irq, hgp->parent->button.irq_type_active);
			headset_gpio_irq_enable(1, &hgp->parent->button);
		}
	} else {
		headset_gpio_irq_enable(0, &hgp->parent->button);
		if (hgp->parent->headphone)
			pr_info("headphone plug out\n");
		else
			pr_info("headset plug out\n");
		switch_set_state(&hgp->parent->sdev, BIT_HEADSET_OUT);
	}
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart headset_gpio_timer_func(struct hrtimer *timer)
{
	enum hrtimer_restart restart;
	struct _headset_gpio *hgp =
		container_of(timer, struct _headset_gpio, timer);
	int value = gpio_get_value(hgp->gpio);
	int active = hgp->active_low ^ value;
	if (hgp->pstatus != value) {
		hgp->pstatus = value;
		hgp->holded = 0;
	}
	pr_debug("%s : [%d] %s %s\n", __func__, value, hgp->desc, active ? "active":"inactive");
	if ((++hgp->holded * HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME) >= hgp->debounce_sw ||
		(!active && &hgp->parent->detect == hgp)) {
		pr_debug("call headset gpio handler\n");
		if (&hgp->parent->button == hgp)
			restart = report_headset_button_status(active, hgp);
		else
			restart = report_headset_detect_status(active, hgp);
		hgp->holded = 0;
	} else restart = HRTIMER_RESTART;
	if (restart == HRTIMER_RESTART)
		hrtimer_forward_now(timer,
			ktime_set(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME / 1000,
					(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME % 1000) * 1000000)); /* repeat timer */
	return restart;
}

static irqreturn_t headset_gpio_irq_handler(int irq, void *dev)
{
	struct _headset_gpio *hgp = dev;
	int active = hgp->active_low ^ gpio_get_value(hgp->gpio);
	irq_set_irq_type(hgp->irq, active ? hgp->irq_type_inactive:hgp->irq_type_active);
	pr_debug("%s : %s %s\n", __func__, hgp->desc, active ? "active":"inactive");
	hrtimer_cancel(&hgp->timer);
	hgp->holded = 0;
	hgp->actived = 0;
	hrtimer_start(&hgp->timer,
			ktime_set(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME / 1000,
				(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME % 1000) * 1000000),
			HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp)
{
	int action = 0;
	if (enable) {
		if (!hgp->irq_enabled) {
			hgp->irq_enabled = 1;
			action = 1;
			hgp->holded = 0;
			hgp->actived = 0;
			hgp->actived_count = 0;
			enable_irq(hgp->irq);
		}
	} else {
		if (hgp->irq_enabled) {
			disable_irq(hgp->irq);
			action = 1;
			hgp->irq_enabled = 0;
			hgp->holded = 0;
			hgp->actived = 0;
			hgp->actived_count = 0;
			hrtimer_cancel(&hgp->timer);
			if (&hgp->parent->button == hgp)
				report_headset_button_status(-1, hgp);
		}
	}
	pr_info("%s [ irq=%d ] --- %saction %s\n", __func__, hgp->irq_enabled, action ? "do ":"no ", hgp->desc);
}

static int __init headset_init(void)
{
	int ret, i;
	struct _headset *ht = &headset;
	ret = switch_dev_register(&ht->sdev);
	if (ret < 0) {
		pr_err("switch_dev_register failed!\n");
		return ret;
	}
	ht->input = input_allocate_device();
	if (ht->input == NULL) {
		pr_err("switch_dev_register failed!\n");
		goto _switch_dev_register;
	}
	ht->input->name = "headset-keyboard";
	ht->input->id.bustype = BUS_HOST;
	ht->input->id.vendor = 0x0001;
	ht->input->id.product = 0x0001;
	ht->input->id.version = 0x0100;
	__set_bit(EV_KEY, ht->input->evbit);
	for (i = 0; i < KEY_CNT; i++)
		input_set_capability(ht->input, EV_KEY, i);
	if (input_register_device(ht->input))
		goto _switch_dev_register;

	gpio_request(ht->detect.gpio, ht->detect.desc);
	gpio_request(ht->button.gpio, ht->button.desc);
	gpio_direction_input(ht->detect.gpio);
	gpio_direction_input(ht->button.gpio);
	if (ht->detect.debounce)
		gpio_set_debounce(ht->detect.gpio, ht->detect.debounce * 1000);
	if (ht->button.debounce)
		gpio_set_debounce(ht->button.gpio, ht->button.debounce * 1000);

	hrtimer_init(&ht->button.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ht->button.timer.function = headset_gpio_timer_func;
	if (ht->button.debounce_sw < HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME)
		ht->button.debounce_sw = HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME;
	ht->button.parent = ht;
	ht->button.irq = gpio_to_irq(ht->button.gpio);
	ht->button.irq_type_active = ht->button.active_low ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	ht->button.irq_type_inactive = ht->button.active_low ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = request_irq(ht->button.irq, headset_gpio_irq_handler,
					ht->button.irq_type_active, ht->button.desc, &ht->button);
	if (ret) {
		pr_err("request_irq gpio %d's irq failed!\n", ht->button.gpio);
		goto _gpio_request;
	}
	headset_gpio_irq_enable(0, &ht->button);

	hrtimer_init(&ht->detect.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ht->detect.timer.function = headset_gpio_timer_func;
	if (ht->detect.debounce_sw < HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME)
		ht->detect.debounce_sw = HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_TIME;
	ht->detect.parent = ht;
	ht->detect.irq = gpio_to_irq(ht->detect.gpio);
	ht->detect.irq_type_active = ht->detect.active_low ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	ht->detect.irq_type_inactive = ht->detect.active_low ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = request_irq(ht->detect.irq, headset_gpio_irq_handler,
					ht->detect.irq_type_active, ht->detect.desc, &ht->detect);
	if (ret) {
		pr_err("request_irq gpio %d's irq failed!\n", ht->detect.gpio);
		goto _headset_button_gpio_irq_handler;
	}
	return 0;
_headset_button_gpio_irq_handler:
	free_irq(ht->button.irq, &ht->button);
_gpio_request:
	gpio_free(ht->detect.gpio);
	gpio_free(ht->button.gpio);
	input_free_device(ht->input);
_switch_dev_register:
	switch_dev_unregister(&ht->sdev);
	return ret;
}
module_init(headset_init);

static void __exit headset_exit(void)
{
	struct _headset *ht = &headset;
	headset_gpio_irq_enable(0, &ht->button);
	headset_gpio_irq_enable(0, &ht->detect);
	free_irq(ht->detect.irq, &ht->detect);
	free_irq(ht->button.irq, &ht->button);
	gpio_free(ht->detect.gpio);
	gpio_free(ht->button.gpio);
	input_free_device(ht->input);
	switch_dev_unregister(&ht->sdev);
}
module_exit(headset_exit);

MODULE_DESCRIPTION("headset & button detect driver");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
