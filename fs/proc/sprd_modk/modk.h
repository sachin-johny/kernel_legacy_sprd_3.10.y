#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/poll.h>


struct dvfs_dev {
    unsigned int cpufreq[NR_CPUS];
	wait_queue_head_t wait;
    struct semaphore sem;
};

extern unsigned int cpufreq_quick_get(unsigned int cpu);
extern int cpufreq_register_notifier(struct notifier_block *nb, unsigned int list);

