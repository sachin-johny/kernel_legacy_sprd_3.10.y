/************************************************************************************************************	*/
/*****Copyright:	2014 Spreatrum. All Right Reserved									*/
/*****File: 		mdbg.c														*/
/*****Description: 	Marlin Debug System main file. Module,device & driver related defination.	*/
/*****Developer:	fan.kou@spreadtrum.com											*/
/*****Date:		06/01/2014													*/
/************************************************************************************************************	*/

#include "mdbg.h"
#include "mdbg_sdio.h"
#include <linux/mutex.h>

#define MDBG_INIT mdbg_sdio_init()
#define MDBG_REMOVE mdbg_sdio_remove()
#define MDBG_MAX_BUFFER_LEN (PAGE_SIZE-1)

bool read_flag = 0;
struct mdbg_devvice_t{
	int 			open_count;
	struct mutex 	mdbg_lock;
	char			*read_buf;
	char			*write_buf;
};

static struct mdbg_devvice_t *mdbg_dev=NULL;

LOCAL ssize_t mdbg_read(struct file *filp,char __user *buf,size_t count,loff_t *pos)
{
	MDBG_SIZE_T read_size;;

	mutex_lock(&mdbg_dev->mdbg_lock);

	if(count > MDBG_MAX_BUFFER_LEN)
		count = MDBG_MAX_BUFFER_LEN;

	if(!read_flag){
		mutex_unlock(&mdbg_dev->mdbg_lock);
		//MDBG_ERR("data no ready");
		return 0;
	}

	read_size = mdbg_receive(mdbg_dev->read_buf , MDBG_MAX_BUFFER_LEN);
	printk_ratelimited(KERN_INFO "%s read_size: %d\n",__func__,read_size);

	if((read_size > 0) && (read_size <= MDBG_MAX_BUFFER_LEN)){
		MDBG_LOG("Show %d bytes data.",read_size);

		if(copy_to_user((void  __user *)buf,mdbg_dev->read_buf ,read_size)){
			mutex_unlock(&mdbg_dev->mdbg_lock);
			MDBG_ERR("copy from user fail!");
			return -EFAULT;
		}
		read_flag = 0;
		mutex_unlock(&mdbg_dev->mdbg_lock);
		return read_size;
	}else{
		mutex_unlock(&mdbg_dev->mdbg_lock);
		MDBG_LOG("Show no data");
		return (0);
	}
}

LOCAL ssize_t mdbg_write(struct file *filp, const char __user *buf,size_t count,loff_t *pos)
{
	MDBG_SIZE_T sent_size = 0;

	if(NULL == buf || 0 == count){
		MDBG_ERR("Param Error!");
		return count;
	}
	mutex_lock(&mdbg_dev->mdbg_lock);

	if (count > MDBG_MAX_BUFFER_LEN ){
		mutex_unlock(&mdbg_dev->mdbg_lock);
		MDBG_ERR("write too long!");
		return -EINVAL;
	}

	if (copy_from_user(mdbg_dev->write_buf ,(void  __user *)buf,count )){
		mutex_unlock(&mdbg_dev->mdbg_lock);
		MDBG_ERR("to user fail!");
		return -EFAULT;
	}

	sent_size = mdbg_send(mdbg_dev->write_buf , count);
	mutex_unlock(&mdbg_dev->mdbg_lock);

	MDBG_LOG("sent_size = %d",sent_size);
	return count;
}

static int mdbg_open(struct inode *inode,struct file *filp)
{
	printk(KERN_INFO "MDBG:%s entry\n",__func__);

	if (mdbg_dev->open_count!=0) {
		MDBG_ERR( "mdbg_open %d \n",mdbg_dev->open_count);
		return -EBUSY;
	}

	mdbg_dev->read_buf = kzalloc(MDBG_MAX_BUFFER_LEN, GFP_KERNEL);
	if (mdbg_dev->read_buf == NULL) {
		MDBG_ERR("faile (NO MEM) Error!");
		return -ENOMEM;
	}
	mdbg_dev->write_buf = kzalloc(MDBG_MAX_BUFFER_LEN, GFP_KERNEL);
	if (mdbg_dev->write_buf == NULL) {
		kfree(mdbg_dev->read_buf);
		mdbg_dev->write_buf = NULL;
		MDBG_ERR("faile (NO MEM) Error!");
		return -ENOMEM;
	}

	mdbg_dev->open_count++;

	return 0;
}

static int mdbg_release(struct inode *inode,struct file *filp)
{
	printk(KERN_INFO "MDBG:%s entry\n",__func__);

	kfree(mdbg_dev->read_buf);
	kfree(mdbg_dev->write_buf);
	mdbg_dev->read_buf = NULL;
	mdbg_dev->write_buf = NULL;
	mdbg_dev->open_count--;
	return 0;
}

static struct file_operations mdbg_fops = {
	.owner = THIS_MODULE,
	.read  = mdbg_read,
	.write = mdbg_write,
	.open  = mdbg_open,
	.release = mdbg_release,
};
static struct miscdevice mdbg_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mdbg",
	.fops = &mdbg_fops,
};

LOCAL int __init mdbg_module_init(void)
{
	int err;

	MDBG_FUNC_ENTERY;

	mdbg_dev = kzalloc(sizeof(*mdbg_dev), GFP_KERNEL);
	if (!mdbg_dev)
		return -ENOMEM;

	mdbg_dev->open_count = 0;
	mutex_init(&mdbg_dev->mdbg_lock);
	err = MDBG_INIT;
	if(err != MDBG_SUCCESS){
		MDBG_ERR("mdbg thread init failed!error code:%d", err);
		return err;
	}

	return misc_register(&mdbg_device);
}

LOCAL void __exit mdbg_module_exit(void)
{
	MDBG_FUNC_ENTERY;
	MDBG_REMOVE;
	mutex_destroy(&mdbg_dev->mdbg_lock);
	misc_deregister(&mdbg_device);
}

module_init(mdbg_module_init);
module_exit(mdbg_module_exit);

MODULE_AUTHOR("Fan Kou<fan.kou@spreadtrum.com>");
MODULE_DESCRIPTION("MARLIN DEBUG SYSTEM.");
MODULE_LICENSE("GPL");

