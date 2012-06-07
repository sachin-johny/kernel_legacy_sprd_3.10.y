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

static enum hrtimer_restart report_headset_button_status(int active, struct _headset_gpio *hgp);
static enum hrtimer_restart report_headset_detect_status(int active, struct _headset_gpio *hgp);
static struct _headset headset = {
	.sdev = {
		.name = "h2w",
	},
	.detect = {
		.desc = "headset detect",
		.active_low = HEADSET_DETECT_GPIO_ACTIVE_LOW,
		.gpio = HEADSET_DETECT_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_DETECT_GPIO_DEBOUNCE_SW,
		.irq_enabled = 1,
		.callback = report_headset_detect_status,
	},
	.button = {
		.desc = "headset button",
		.active_low = HEADSET_BUTTON_GPIO_ACTIVE_LOW,
		.gpio = HEADSET_BUTTON_GPIO,
		.debounce = 0,
		.debounce_sw = HEADSET_BUTTON_GPIO_DEBOUNCE_SW,
		.irq_enabled = 1,
		.callback = report_headset_button_status,
		.timeout_ms = 800, /* 800ms for long button down */
	},
};

#ifndef headset_gpio_init
#define headset_gpio_init(gpio, desc) \
	do { \
		gpio_request(gpio, desc); \
		gpio_direction_input(gpio); \
	} while (0)
#endif

#ifndef headset_gpio_free
#define headset_gpio_free(gpio) \
	gpio_free(gpio)
#endif

#ifndef headset_gpio2irq_free
#define headset_gpio2irq_free(irq, args) { }
#endif

#ifndef headset_gpio2irq
#define headset_gpio2irq(gpio) \
	gpio_to_irq(gpio)
#endif

#ifndef headset_gpio_set_irq_type
#define headset_gpio_set_irq_type(irq, type) \
	irq_set_irq_type(irq, type)
#endif

#ifndef headset_gpio_get_value
#define headset_gpio_get_value(gpio) \
	gpio_get_value(gpio)
#endif

#ifndef headset_gpio_debounce
#define headset_gpio_debounce(gpio, ms) \
	gpio_set_debounce(gpio, ms)
#endif

#ifndef headset_hook_detect
#define headset_hook_detect(status) { }
#endif

#define HEADSET_DEBOUNCE_ROUND_UP(dw) \
	dw = (((dw ? dw : 1) + HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD - 1) / \
		HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD) * HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD;

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp);
#define HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD	50 /* 10 */
static enum hrtimer_restart report_headset_button_status(int active, struct _headset_gpio *hgp)
{
	enum hrtimer_restart restart;
	int code = -1;
	static int step = 0;
	if (active < 0) {
		step = 0;
		return HRTIMER_NORESTART;
	}
	if (active) {
		restart = HRTIMER_RESTART;
		if (++step > 3)
			step = 0;
		switch (step) {
		case 2:
			code = KEY_END;
			break;
		}
	} else {
		restart = HRTIMER_NORESTART;
		if (step == 1)
			code = KEY_MEDIA;
		step = 0;
	}
	if (code >= 0) {
		input_event(hgp->parent->input, EV_KEY, code, 1);
		input_sync(hgp->parent->input);
		input_event(hgp->parent->input, EV_KEY, code, 0);
		input_sync(hgp->parent->input);
		pr_info("headset button-%d[%dms]\n", code, hgp->holded);
	}
	return restart;
}

static enum hrtimer_restart report_headset_detect_status(int active, struct _headset_gpio *hgp)
{
	if (active) {
		headset_hook_detect(1);
		hgp->parent->headphone = 0;
		/* hgp->parent->headphone = hgp->parent->button.active_low ^ headset_gpio_get_value(hgp->parent->button.gpio); */
		if (hgp->parent->headphone) {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_NO_MIC);
			pr_info("headphone plug in\n");
		} else {
			switch_set_state(&hgp->parent->sdev, BIT_HEADSET_MIC);
			pr_info("headset plug in\n");
			headset_gpio_set_irq_type(hgp->irq, hgp->parent->button.irq_type_active);
			headset_gpio_irq_enable(1, &hgp->parent->button);
		}
	} else {
		headset_gpio_irq_enable(0, &hgp->parent->button);
		hgp->parent->button.callback(-1, &hgp->parent->button);
		headset_hook_detect(0);
		if (hgp->parent->headphone)
			pr_info("headphone plug out\n");
		else
			pr_info("headset plug out\n");
		switch_set_state(&hgp->parent->sdev, BIT_HEADSET_OUT);
	}
	/* use below code only when gpio irq misses state ? */
	/* headset_gpio_set_irq_type(hgp->irq, active ? hgp->irq_type_inactive : hgp->irq_type_active); */
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart headset_gpio_timer_func(struct hrtimer *timer)
{
	enum hrtimer_restart restart = HRTIMER_RESTART;
	struct _headset_gpio *hgp =
		container_of(timer, struct _headset_gpio, timer);
	int active = hgp->active_low ^ headset_gpio_get_value(hgp->gpio); /* hgp->active */
	int green_ch = (!active && &hgp->parent->detect == hgp);
	if (active != hgp->active) {
		pr_info("The value %s mismatch [%d:%d] at %dms!\n",
				hgp->desc, active, hgp->active, hgp->holded);
		hgp->holded = 0;
	}
	pr_debug("%s : %s %s green_ch[%d], holed=%d, debounce_sw=%d\n", __func__,
			hgp->desc, active ? "active" : "inactive", green_ch, hgp->holded, hgp->debounce_sw);
	hgp->holded += HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD;
	if (hgp->holded >= hgp->debounce_sw || green_ch) {
		if (hgp->holded == hgp->debounce_sw || \
			hgp->holded == hgp->timeout_ms || \
			green_ch) {
			pr_debug("call headset gpio handler\n");
			restart = hgp->callback(active, hgp);
		} else
			pr_debug("gpio <%d> has kept active for %d ms\n", hgp->gpio, hgp->holded);
	}
	if (restart == HRTIMER_RESTART)
		hrtimer_forward_now(timer,
			ktime_set(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD / 1000,
					(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD % 1000) * 1000000)); /* repeat timer */
	return restart;
}

static irqreturn_t headset_gpio_irq_handler(int irq, void *dev)
{
	struct _headset_gpio *hgp = dev;
	hrtimer_cancel(&hgp->timer);
	hgp->active = hgp->active_low ^ headset_gpio_get_value(hgp->gpio);
	headset_gpio_set_irq_type(hgp->irq, hgp->active ? hgp->irq_type_inactive : hgp->irq_type_active);
	pr_debug("%s : %s %s\n", __func__, hgp->desc, hgp->active ? "active" : "inactive");
	hgp->holded = 0;
	hrtimer_start(&hgp->timer,
			ktime_set(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD / 1000,
				(HEADSET_GPIO_DEBOUNCE_SW_SAMPLE_PERIOD % 1000) * 1000000),
			HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}

static void headset_gpio_irq_enable(int enable, struct _headset_gpio *hgp)
{
	int action = 0;
	if (enable) {
		if (!hgp->irq_enabled) {
			hrtimer_cancel(&hgp->timer);
			hgp->irq_enabled = 1;
			action = 1;
			hgp->holded = 0;
			enable_irq(hgp->irq);
		}
	} else {
		if (hgp->irq_enabled) {
			disable_irq(hgp->irq);
			hrtimer_cancel(&hgp->timer);
			hgp->irq_enabled = 0;
			action = 1;
			hgp->holded = 0;
		}
	}
	pr_info("%s [ irq=%d ] --- %saction %s\n", __func__, hgp->irq_enabled, action ? "do " : "no ", hgp->desc);
}

static int __init headset_init(void)
{
	int ret;
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
	input_set_capability(ht->input, EV_KEY, KEY_MEDIA);
	input_set_capability(ht->input, EV_KEY, KEY_END);

	if (input_register_device(ht->input))
		goto _switch_dev_register;

	headset_gpio_init(ht->detect.gpio, ht->detect.desc);
	headset_gpio_init(ht->button.gpio, ht->button.desc);

	headset_gpio_debounce(ht->detect.gpio, ht->detect.debounce * 1000);
	headset_gpio_debounce(ht->button.gpio, ht->button.debounce * 1000);

	hrtimer_init(&ht->button.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ht->button.timer.function = headset_gpio_timer_func;
	HEADSET_DEBOUNCE_ROUND_UP(ht->button.debounce_sw);
	HEADSET_DEBOUNCE_ROUND_UP(ht->button.timeout_ms);
	ht->button.parent = ht;
	ht->button.irq = headset_gpio2irq(ht->button.gpio);
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
	HEADSET_DEBOUNCE_ROUND_UP(ht->detect.debounce_sw);
	ht->detect.parent = ht;
	ht->detect.irq = headset_gpio2irq(ht->detect.gpio);
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
	headset_gpio2irq_free(ht->button.irq, &ht->button);
_gpio_request:
	headset_gpio_free(ht->detect.gpio);
	headset_gpio_free(ht->button.gpio);
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
	headset_gpio2irq_free(ht->detect.irq, &ht->detect);
	free_irq(ht->button.irq, &ht->button);
	headset_gpio2irq_free(ht->button.irq, &ht->button);
	headset_gpio_free(ht->detect.gpio);
	headset_gpio_free(ht->button.gpio);
	input_free_device(ht->input);
	switch_dev_unregister(&ht->sdev);
}
module_exit(headset_exit);

MODULE_DESCRIPTION("headset & button detect driver");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
