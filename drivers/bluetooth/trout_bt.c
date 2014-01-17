#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
//#include <linux/poll.h>
#include <net/bluetooth/bluetooth.h>
#include "trout_bt.h"
static int trout_bt_major = 0;
static int trout_bt_minor = 0;
static dev_t dev;
//Three counts;
static int trout_bt_count = 1;
static const char *name = "trout_bt_dev";

static struct class *trout_bt_class = NULL;
static struct device *trout_bt_device = NULL;
static struct trout_bt_dev *trout_bt_struct = NULL;

static int trout_bt_open(struct inode *inode, struct file *filp);
static int trout_bt_release(struct inode *inode, struct file *filp);
static ssize_t trout_bt_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t trout_bt_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static long trout_bt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int trout_bt_construct(struct trout_bt_dev** trout_bt_struct);
static void trout_bt_destroy(struct trout_bt_dev* trout_bt_struct);

typedef enum
{
	TROUT_NONE_MODULE	= 0,
	TROUT_WIFI_MODULE	= 1,
	TROUT_FM_MODULE		= 2,
	TROUT_BT_MODULE		= 4,
	TROUT_ALL_MODULE		= 7,
}TROUT_MODULE_T;

typedef enum
{
	TROUT_SDIO	= 0,
	TROUT_SPI	= 1,
}TROUT_BUS;


extern bool Set_Trout_RF_Start(TROUT_MODULE_T module);
extern bool Set_Trout_RF_Stop(TROUT_MODULE_T module);
extern bool Set_Trout_Download_BT_Code(void);
extern bool Set_Trout_PowerOn( unsigned int  MODE_ID );
extern bool Set_Trout_PowerOff(unsigned int MODE_ID);

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);

//extern void Trout_Print_Sys_Common_Reg();

static struct file_operations trout_bt_fops = {
  .owner = THIS_MODULE,
	.open = trout_bt_open,
	.release = trout_bt_release,
	.read = trout_bt_read,
	.write = trout_bt_write,
	.unlocked_ioctl = trout_bt_ioctl,
};

static int trout_bt_open(struct inode *inode, struct file *filp) {
	struct trout_bt_dev *dev;

	dev = container_of(inode->i_cdev, struct trout_bt_dev, cdev);
	filp->private_data = (void *)dev;

	return 0;
}


static int trout_bt_release(struct inode *inode, struct file *filp){
	return 0;
}


static ssize_t trout_bt_read(struct file * filp, char __user * buf, size_t count, loff_t * f_pos) {
	int rev = 0;
	struct trout_bt_dev *trout_bt_struct = (struct trout_bt_dev *)filp->private_data;
	return rev;
}
#if 0
INLINE u32 convert_to_le(u32 val)
{
#ifdef BIG_ENDIAN
    return SWAP_BYTE_ORDER_WORD(val);
#endif /* BIG_ENDIAN */

#ifdef LITTLE_ENDIAN
    return val;
#endif /* LITTLE_ENDIAN */
}
#endif
static ssize_t trout_bt_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos) {

    BT_DBG("trout_bt_write\n");
    BT_DBG("buf:%s\n", buf);

    // open bt
    if('1' == buf[0]){
        if(!Set_Trout_PowerOn(4)){
            BT_DBG("set Trout Poweron fail !!!!\n");
            return -1;
        }

        //Set_Trout_Download_BT_Code();
        //Set_Trout_RF_Start(4);
    }
    else if('0' == buf[0]){  // close BT
        //Set_Trout_RF_Stop(4);
        Set_Trout_PowerOff(4);
    }
	return 1;
}

static long trout_bt_ioctl(struct file * filp, unsigned int cmd, unsigned long arg) {
	int rev = 0;
	return 0;
}

 static int trout_bt_construct(struct trout_bt_dev ** trout_bt_struct) {
	return 0;
 }

 static void trout_bt_destroy(struct trout_bt_dev * trout_bt_struct) {
	kfree(trout_bt_struct);
	return;
 }

static int __init trout_bt_init(void) {
	int rev;

	BT_DBG("trout_bt_init\n");

	if(trout_bt_major) {
		dev = MKDEV(trout_bt_major, trout_bt_minor);
		rev = register_chrdev_region(dev, trout_bt_count, name);
	} else {
		rev = alloc_chrdev_region(&dev, 0, trout_bt_count, name);
		trout_bt_major = MAJOR(dev);
	}
	if(rev < 0) {
		BT_DBG("Register dev_t error.\n");
		goto out;
	}

	trout_bt_struct = (struct  trout_bt_dev*)kmalloc(sizeof(struct  trout_bt_dev), GFP_KERNEL);
	cdev_init(&trout_bt_struct->cdev, &trout_bt_fops);
	trout_bt_struct->cdev.owner = THIS_MODULE;
	trout_bt_struct->cdev.ops = &trout_bt_fops;
	rev = cdev_add(&trout_bt_struct->cdev, dev, 1);
	if (rev) {
		BT_DBG("cdev_add error:%d\n", rev);
		goto register_cdev_fail;
	}

	rev = trout_bt_construct(&trout_bt_struct);
	if (rev) {
		BT_DBG("trout_bt_construct error:  %d\n", rev);
		goto register_trout_bt_fail;
	}

	trout_bt_class = class_create(THIS_MODULE, "trout_bt_dev");
	if (IS_ERR(trout_bt_class)) {
		BT_DBG("trout_bt_class create failed.\n");
		goto register_class_fail;
	}

	trout_bt_device = device_create(trout_bt_class, NULL, dev, NULL, "trout_bt_dev");
	if(IS_ERR(trout_bt_device)) {
		BT_DBG("device_create failed\n");
		goto register_device_fail;
	}

	goto out;
register_device_fail:
	class_destroy(trout_bt_class);

register_class_fail:
	trout_bt_destroy(trout_bt_struct);

register_trout_bt_fail:
	cdev_del(&trout_bt_struct->cdev);

register_cdev_fail:
	unregister_chrdev_region(dev, trout_bt_count);

out:
	return rev;
}


static void __exit trout_bt_exit(void) {
	device_del(trout_bt_device);
	class_destroy(trout_bt_class);
	trout_bt_destroy(trout_bt_struct);
	//cdev is del in destroy;
	unregister_chrdev_region(dev, trout_bt_count);
}

module_init(trout_bt_init);
module_exit(trout_bt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("longting.zhao");




