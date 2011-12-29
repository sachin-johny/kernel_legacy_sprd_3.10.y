#include <linux/module.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>

static int board_type = 0;

static int miscsysdev_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int miscsysdev_release(struct inode *inode, struct file *filp)
{
    return 0;
}


ssize_t miscsysdev_write(struct file *file, const char __user *buf,
        size_t count, loff_t *offset)
{
    return 0;
}

extern int sprd_board_type;


static ssize_t boardtype_sysfs_show(struct device *dev,
            struct device_attribute *attr,
            char *buf)
{
    int len;
    len = sprintf(buf, "%d\n", sprd_board_type);
    return len;
}

static DEVICE_ATTR(boardtype, 0444, boardtype_sysfs_show, NULL);

static const struct file_operations miscsysdev_fops = {
    .open    =  miscsysdev_open,
    .write   =  miscsysdev_write,
    .release =  miscsysdev_release,
    .owner   =  THIS_MODULE,
};

static struct miscdevice miscsysdev_dev = {
    .minor =    MISC_DYNAMIC_MINOR,
    .name  =    "miscsysdev",
    .fops  =    &miscsysdev_fops
};

static int __init miscsysdev_init(void)
{
    int ret;
    ret = misc_register(&miscsysdev_dev);
    if (!ret){
        if ( !device_create_file((&miscsysdev_dev)->this_device, &dev_attr_boardtype)){
        }
    }
    return ret;
}

static void __exit miscsysdev_exit(void)
{
    misc_deregister(&miscsysdev_dev);
    device_remove_file((&miscsysdev_dev)->this_device, &dev_attr_boardtype);
}

late_initcall(miscsysdev_init);
module_exit(miscsysdev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Allen Zhang <Allen.Zhang@spreadtrum.com>");
MODULE_DESCRIPTION("Driver for gpio control");
