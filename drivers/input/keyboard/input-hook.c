/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/input-hook.h>

static enum hrtimer_restart trigger_watch_event_timer_cb(struct hrtimer *timer)
{
	struct trigger_watch_event *event = container_of(timer, struct trigger_watch_event, timer);
	enum hrtimer_restart restart = HRTIMER_NORESTART;
	pr_info("< %s > trigger-watch event timer timeout %dms\n", event->name, event->period);
	if (event->trigger_watch_event_cb)
		event->trigger_watch_event_cb(event->private);
	if (event->repeated) { /* repeat timer */
		hrtimer_forward_now(timer, ktime_set(event->period / 1000, (event->period % 1000) * 1000000));
		restart = HRTIMER_RESTART;
	}
	return restart;
}

int trigger_watch_event_stop(struct trigger_watch_event *event)
{
	if (!event->disable_timer) {
		hrtimer_cancel(&event->timer);
	}
	if (!event->disable_clk_source) {
		u64 delta = (local_clock() - event->last_ts);
		if (delta > ((u64)event->period) * 1000000) {
			pr_info("< %s > trigger-watch event clock timeout %d / %lld\n", event->name, event->period, delta);
			if (event->trigger_watch_event_cb)
				event->trigger_watch_event_cb(event->private);
		}
	}
	return 0;
}
EXPORT_SYMBOL(trigger_watch_event_stop);

void trigger_watch_event_start(struct trigger_watch_event *event, int ms)
{
	if (!event->initialized) {
		event->initialized = 1;
		if (!event->disable_timer) {
			hrtimer_init(&event->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			event->timer.function = trigger_watch_event_timer_cb;
		}
	}
	event->repeated = !!(ms & TRIGGER_WATCH_EVENT_REPEATED);
	event->period = (ms & ~TRIGGER_WATCH_EVENT_REPEATED);
	event->last_ts = local_clock();
	trigger_watch_event_stop(event);
	if (!event->disable_timer) {
		hrtimer_start(&event->timer, ktime_set(event->period / 1000, (event->period % 1000) * 1000000), HRTIMER_MODE_REL);
	}
}
EXPORT_SYMBOL(trigger_watch_event_start);

static bool hook_power = 1;
static int input_hook_power_write_proc(struct file *file, const char __user * buffer,
		unsigned long count, void *data)
{
	char enable[2];

	if (copy_from_user(enable, buffer, 1))
		return -EFAULT;
	enable[1] = 0;

	hook_power = !!simple_strtoul(enable, NULL, 0);

	return count;
}

static int input_hook_power_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	page[0] = '0' + hook_power;
	page[1] = '\n';
	return 2;
}

void input_hook_init(void)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry("input_power", 0, NULL);
	if (!entry) {
		pr_err("unable to create /proc/input_power entry\n");
		return;
	}

	entry->read_proc = input_hook_power_read_proc;
	entry->write_proc = input_hook_power_write_proc;
	entry->data = NULL;
}

void input_hook_exit(void)
{
	remove_proc_entry("input_power", NULL);
}

static void trigger_watch_powerkey(void *private)
{
	unsigned long flags;
	local_irq_save(flags);
#ifdef CONFIG_MAGIC_SYSRQ
	handle_sysrq('m');
	handle_sysrq('w');
#endif
	pr_warn("!!!! trigger_watch_powerkey !!!! do emergency_restart\n");
	emergency_restart();
	pr_err("%s should never reach here!\n", __func__);
}

void input_report_key_hook(struct input_dev *dev, unsigned int code, int value)
{
	if (hook_power && code == KEY_POWER) {
		static struct trigger_watch_event twe = {
			.name = "power key",
			.private = NULL, /* like dev */
			.trigger_watch_event_cb = trigger_watch_powerkey,
		};
		static int pre_value = -1;
		if (value != pre_value) {
			if (value)
				trigger_watch_event_start(&twe, 6 * 1000);
			else
				trigger_watch_event_stop(&twe);
		} else {
			pr_warn("Key %d%c is dithering.", code, value ? 'D' : 'U');
		}
		pre_value = value;
	}
}
