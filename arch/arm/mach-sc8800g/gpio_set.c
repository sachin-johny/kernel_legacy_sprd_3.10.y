#include <linux/module.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

static int gpio_set_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int gpio_set_release(struct inode *inode, struct file *filp)
{
    return 0;
}

void gpio_strtok(char **start)
{
    while ((**start != ' ') && **start) (*start)++;
    *(*start)++ = 0;
}

ssize_t gpio_set_write(struct file *file, const char __user *buf,
        size_t count, loff_t *offset)
{
    char tmp[50];
    char *p, *p1;
    int gpio, pre_level, ms, stable_level;

    if (count > 50)
        count = 50;
    copy_from_user(tmp, buf, count);
    tmp[count] = 0;

    // gpio pre_level ms stable_level
    p1 = p = tmp;
    gpio_strtok(&p);
    gpio = simple_strtoul(p1, NULL, 0);

    p1 = p;
    gpio_strtok(&p);
    pre_level = simple_strtoul(p1, NULL, 0);

    p1 = p;
    gpio_strtok(&p);
    ms = simple_strtoul(p1, NULL, 0);

    p1 = p;
    gpio_strtok(&p);
    stable_level = simple_strtoul(p1, NULL, 0);

    snprintf(tmp, sizeof tmp, "[%d-%d-%d-%d]", gpio, pre_level, ms, stable_level);
    printk("%s\n", tmp);

    gpio_request(gpio, tmp);
    gpio_direction_output(gpio, pre_level);
    msleep(ms);
    gpio_direction_output(gpio, stable_level);

    return count;
}

static const struct file_operations gpio_set_fops = {
    .open    =  gpio_set_open,
    .write   =  gpio_set_write,
    .release =  gpio_set_release,
};

static struct miscdevice gpio_set_dev = {
    .minor =    MISC_DYNAMIC_MINOR,
    .name  =    "gpio",
    .fops  =    &gpio_set_fops
};

static int gpio_set_init(void)
{
    return misc_register(&gpio_set_dev);
}

static void gpio_set_exit(void)
{
    misc_deregister(&gpio_set_dev);
}

late_initcall(gpio_set_init);
module_exit(gpio_set_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_DESCRIPTION("Driver for gpio control");
