#include <modk.h>

/* dvfs device struct */
struct dvfs_dev dvfsdev;

static ssize_t sprd_dvfs_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
    int i;
    unsigned int cpu;
    loff_t pos = *ppos;
    size_t ret,buffersize;

    DECLARE_WAITQUEUE(wait_queue, current);

    down(&dvfsdev.sem);
    add_wait_queue(&dvfsdev.wait, &wait_queue);
    if(file->f_flags & O_NONBLOCK){
        up(&dvfsdev.sem);
        ret = -EAGAIN;
        return ret;
    }
    __set_current_state(TASK_INTERRUPTIBLE);
    up(&dvfsdev.sem);

    schedule();
    if(signal_pending(current)){
        remove_wait_queue(&dvfsdev.wait, &wait_queue);
        ret = -ERESTARTSYS;
        return ret;
    }

    down(&dvfsdev.sem);
    remove_wait_queue(&dvfsdev.wait, &wait_queue);
    up(&dvfsdev.sem);

    for_each_possible_cpu(cpu) {
        dvfsdev.cpufreq[cpu] = cpufreq_quick_get(cpu);
    }

    buffersize = sizeof(dvfsdev.cpufreq);

    if (pos < 0 || len < 0)
        return -EINVAL;

    if (!len)
        return 0;
    if (pos >= buffersize) {
        *ppos = 0;
        return 0;
    }
    if (len > buffersize - pos)
        len = buffersize - pos;

    ret = copy_to_user(buf, dvfsdev.cpufreq + pos, len);
    if(ret){
        printk("####SPRD copy_to_user failed\n");
        return -EFAULT;
    }

//used for debug
#if 0
    for(i=0; i < NR_CPUS; i++){
        printk("####SPRD cpu[%d]: freq = %u\n", i, dvfsdev.cpufreq[i]);
    }
#endif

    if (ret == len)
        return -EFAULT;

    len -= ret;
    *ppos = pos + len;

    return len;
}

struct file_operations sprd_dvfs_fops = {
	.read	= sprd_dvfs_read,
};

static void sprd_dvfs_dev_init(void)
{
    init_waitqueue_head(&dvfsdev.wait);
    sema_init(&dvfsdev.sem, 1);
    memset(dvfsdev.cpufreq, 0, sizeof(dvfsdev.cpufreq));
}
static int __init sprd_dvfs_init(void)
{
    sprd_dvfs_dev_init();
    proc_create("kcpufreq", S_IRUGO, NULL, &sprd_dvfs_fops);
	return 0;
}

module_init(sprd_dvfs_init);

