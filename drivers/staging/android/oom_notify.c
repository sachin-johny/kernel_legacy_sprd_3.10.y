#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/syslog.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>



DECLARE_WAIT_QUEUE_HEAD(report_wait);
static DEFINE_SPINLOCK(reportbuf_lock);
#define BUF_LEN 16
static int report_buf_len = BUF_LEN;
char mm_report_buf[BUF_LEN][TASK_COMM_LEN+1];
static int record_size = TASK_COMM_LEN ;
int oom_notify_enable = 1;

#define REPORT_BUF_MASK (report_buf_len-1)
#define REPORT_BUF(idx) (mm_report_buf[(idx) & REPORT_BUF_MASK])


static unsigned report_start=0;	/* Index into mm_report_buf: next char to be read*/
static unsigned report_end=0;	/* Index into mm_report_buf: most-recently-written-char + 1 */


int do_mm_report(char* comm)
{
	memset(&REPORT_BUF(report_end),0,TASK_COMM_LEN+1);
	memcpy(&REPORT_BUF(report_end), comm, TASK_COMM_LEN);
	printk("LOW_MEMORY_KILLER name: %s\n", REPORT_BUF(report_end));
	report_end++;
	wake_up_interruptible(&report_wait);

}

static int do_read_memory_report(char __user *buf, int len)
{
	int i =0;
    int error = -EINVAL;
	char record[TASK_COMM_LEN+1];
	int total = 0;
	
    if (!buf || len < 0)
		goto out;
    error = 0;
    if (!len)
		goto out;
    if (!access_ok(VERIFY_WRITE, buf, len)) {
		error = -EFAULT;
		goto out;
    }
    error = wait_event_interruptible(report_wait,
	(report_start - report_end));
    if (error)
        goto out;
	total =len;
	if (len >record_size)
		len = record_size;
	spin_lock_irq(&reportbuf_lock);
	while (!error && (report_start != report_end)) {
		memset(record, 0, TASK_COMM_LEN+1);
		memcpy(record,REPORT_BUF(report_start), TASK_COMM_LEN);
		printk("do_read_memory_report report_start %d, report_end %d\n", report_start, report_end);
		printk("do_read_memory_report %s, %d, %d\n",record, len, record_size);
		report_start++; 
		spin_unlock_irq(&reportbuf_lock);
		error = copy_to_user(buf,record,len);
		i++;
		if((i+1)*len < total)
		{
			buf += len;
		}
		else
		{
			break;
		}
		cond_resched();
		spin_lock_irq(&reportbuf_lock);
	}
	spin_unlock_irq(&reportbuf_lock);
	if (!error)
		error = i*len;
out:
	return error;
}



static int memory_report_open(struct inode * inode, struct file * file)
{
	//printk("enter memory_report_open\n");
	return 0;
}

static int memory_report_release(struct inode * inode, struct file * file)
{
	//printk("enter memory_report_release\n");
	return 0;
}

static ssize_t memory_report_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	//printk("enter memory_report_read\n");
	if(oom_notify_enable)
	{
		if ((file->f_flags & O_NONBLOCK) &&
			(report_start != report_end))
			return -EAGAIN;
		return do_read_memory_report(buf,count);
	}
	else
	{
		printk("oom notify disabled\n");
		return -EAGAIN;
	}
}

static inline int my_atoi(const char *name)
{
    int val = 0;

    for (;; name++) {
        switch (*name) {
            case '0' ... '9':
                val = 10*val+(*name-'0');
                break;
            default:
                return val;
        }
    }
}

static ssize_t memory_report_write( struct file *file,
			   const char __user *buffer,
			   size_t count,
			   loff_t *offset )
{
	//printk("enter enter memory_report_write\n");
	char enable[9]={0};
	if(count<1)
	{
		printk("memory_report_write, count err\n");
		return -EINVAL;
	}
	if(count >8)
	{
		count = 8;
	}
	if (copy_from_user(enable, buffer, count))
	{
		return -EFAULT;
	}
	if(!strcmp(enable,"\n"))
	{
		printk("write nothing\n");	
		return -EINVAL;
	}
	oom_notify_enable = my_atoi(enable);
	printk("enable %s, oom_notify_enable %d", enable, oom_notify_enable);
	return 1;
}

static unsigned int memory_report_poll(struct file *file, poll_table *wait)
{
	//printk("enter memory_report_poll\n");
	poll_wait(file, &report_wait, wait);
	if (report_start != report_end)
		return POLLIN | POLLRDNORM;
	return 0;
}

static const struct file_operations memory_report_fops = { 
    .open = memory_report_open, 
    .read = memory_report_read,
	.write = memory_report_write,
    .poll = memory_report_poll,
    .release = memory_report_release,	
    .llseek = generic_file_llseek,  
};
static int __init init_memory_report(void)
{
    struct proc_dir_entry *pe;

    pe = proc_create("oom_notify", S_IRUSR|S_IWUSR, NULL, &memory_report_fops);
    if (!pe)
		return -ENOMEM;
    return 0;
}

static void __exit exit_memory_report(void)
{

}
module_init(init_memory_report);
module_exit(exit_memory_report);


