#include <modk.h>

extern struct dvfs_dev dvfsdev;

static int sprd_dvfs_callback(void)
{
    wake_up_interruptible(&dvfsdev.wait);
    return NOTIFY_OK;
}

static int modk_notifier_handlers(struct notifier_block *nb, unsigned long val, void *data)
{
	int ret = NOTIFY_OK;

	switch(val) {
		case CPUFREQ_TRANSITION_NOTIFIER:
			ret = sprd_dvfs_callback();
			break;
		default :
			break;
	}

	return ret;
}
static struct notifier_block modk_notifier_block = {
	.notifier_call = modk_notifier_handlers,
};

static int __init sprd_modk_init(void)
{
    int ret_transition;

    ret_transition = cpufreq_register_notifier(&modk_notifier_block,
                     CPUFREQ_TRANSITION_NOTIFIER);
    if(ret_transition)
        return ret_transition;

    return 0;
}

static void __exit sprd_modk_exit(void)
{
	cpufreq_unregister_notifier(&modk_notifier_block,
                            CPUFREQ_TRANSITION_NOTIFIER);
	return;
}


module_init(sprd_modk_init);
module_exit(sprd_modk_exit);

