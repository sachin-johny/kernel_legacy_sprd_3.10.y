#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/suspend.h>

#include "trout_fm_ctrl.h"
#include "trout_rf_common.h"
#include "trout_fm_audio.h"
#include "trout_interface.h"

#define TROUT_FM_VERSION	"v0.12"
#define POWER_ON_FM		(1<<1)
trout_interface_t   *p_trout_interface = NULL;
extern suspend_state_t get_suspend_state(void);

extern bool Set_Trout_PowerOn( unsigned int  MODE_ID );
extern bool Set_Trout_PowerOff(unsigned int MODE_ID);

atomic_t	is_fm_open;

int trout_fm_set_volume(u8 iarg)
{
	TROUT_PRINT("FM set volume : %i.", iarg);
	return 0;
}

int trout_fm_get_volume(void)
{
	TROUT_PRINT("FM get volume.");
	return 0;
}

int trout_fm_open(struct inode *inode, struct file *filep)
{
	int ret = -EINVAL;
   	int status;

	TROUT_PRINT("start open fm module...");

	if (atomic_read(&is_fm_open)) {
		TROUT_PRINT("trout_fm has been opened!");
		return -1;
	}

	if (get_suspend_state() != PM_SUSPEND_ON)
	{
		TROUT_PRINT("The system is suspending!");
		return -2;
	}

	ret = nonseekable_open(inode, filep);
	if (ret < 0)
	{
		TROUT_PRINT("open misc device failed.");
		return ret;
	}

	Set_Trout_PowerOn(2/*FM_MODE*/);

	ret = trout_fm_init();
	if(ret < 0)
	{
		TROUT_PRINT("trout_fm_init failed!");
		return ret;
	}
	
	ret = trout_fm_get_status(&status);
	if(ret < 0)
	{
		TROUT_PRINT("trout_read_fm_en failed!");
		return ret;
	}
	if(status)
	{
		TROUT_PRINT("trout fm have been opened.");
	}
	else
	{
		ret = trout_fm_en();
		if(ret < 0)
		{
			TROUT_PRINT("trout_fm_en failed!");
			return ret;
		}
	}

	/* set trout wifi goto sleep */
	#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	wifimac_sleep();
	#endif

	atomic_cmpxchg(&is_fm_open, 0, 1);

	TROUT_PRINT("Open fm module success.");

	return 0;
}

int trout_fm_ioctl(struct file *filep,
		unsigned int cmd, unsigned long arg)
{
	void __user  *argp = (void __user *)arg;
	int  ret = 0;
	int  iarg = 0;  
	int  buf[4] = {0};
	trout_reg_cfg_t reg;
 
	//clear interrupt
	//WRITE_REG(FM_REG_INT_CLR, 1);
	TROUT_PRINT("FM IOCTL: %i.", cmd);

	switch (cmd) {
		case READ_FM_REG:
			if (copy_from_user(&reg, argp, sizeof(reg))) {
				ret =  -EFAULT;
			}
			READ_REG(reg.addr, &reg.data);

			if (copy_to_user(argp, &reg, sizeof(reg))) {
				ret = -EFAULT;
			}
			break;
		case WRITE_FM_REG:
			if (copy_from_user(&reg, argp, sizeof(reg))) {
				ret =  -EFAULT;
			}

			WRITE_REG(reg.addr, reg.data);
			
			break;
		case READ_FM_RF_REG:
			if (copy_from_user(&reg, argp, sizeof(reg))) {
				ret =  -EFAULT;
			}
			READ_RF_REG(reg.addr, &reg.data);

			if (copy_to_user(argp, &reg, sizeof(reg))) {
				ret = -EFAULT;
			}
			break;
		case WRITE_FM_RF_REG:
			if (copy_from_user(&reg, argp, sizeof(reg))) {
				ret =  -EFAULT;
			}

			WRITE_RF_REG(reg.addr, reg.data);
			break;
		case Trout_FM_IOCTL_ENABLE:
			if (copy_from_user(&iarg, argp, sizeof(iarg)) || iarg > 1) {
				ret =  -EFAULT;
			}
			if (iarg ==1) {
				ret = trout_fm_en();
			}
			else {
				ret = trout_fm_dis();
			}
			break;

		case Trout_FM_IOCTL_GET_ENABLE:
			ret = trout_fm_get_status(&iarg);
			if (copy_to_user(argp, &iarg, sizeof(iarg))) {
				ret = -EFAULT;
			}
			break;

		case Trout_FM_IOCTL_SET_TUNE:
			if (copy_from_user(&iarg, argp, sizeof(iarg))) {
				ret = -EFAULT;
			}
			//ret = trout_fm_set_freq(iarg);
			ret = trout_fm_set_tune(iarg);
		
		#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
			wifimac_sleep();//hugh:add for power 20130425
		#endif
			break;

		case Trout_FM_IOCTL_GET_FREQ:
			ret = trout_fm_get_frequency((u16 *)&iarg);
			if (copy_to_user(argp, &iarg, sizeof(iarg))) {
				ret = -EFAULT;
			}
			break;

		case Trout_FM_IOCTL_SEARCH:
			if (copy_from_user(buf, argp, sizeof(buf))) {
				ret = -EFAULT;
			}
			ret = trout_fm_seek( buf[0], /* start frequency */
					buf[1], /* seek direction*/
					buf[2], /* time out */
					(u16*)&buf[3]);/* frequency found will be stored to */

			if(copy_to_user(argp, buf, sizeof(buf))){
				ret = -EFAULT;
			}
			break;

		case Trout_FM_IOCTL_STOP_SEARCH:
			ret = trout_fm_stop_seek();
			break;

		case Trout_FM_IOCTL_MUTE:
			break;

		case Trout_FM_IOCTL_SET_VOLUME:
			if (copy_from_user(&iarg, argp, sizeof(iarg))) {
				ret = -EFAULT;
			}
			ret = trout_fm_set_volume((u8)iarg);
			break;            

		case Trout_FM_IOCTL_GET_VOLUME:
			iarg = trout_fm_get_volume();
			if (copy_to_user(argp, &iarg, sizeof(iarg))) {
				ret = -EFAULT;
			}
			break;            
		//chenq add a ioctl cmd for deal i2s work,in songkun mail,2013-01-17
		case Trout_FM_IOCTL_CONFIG:
			if (copy_from_user(&iarg, argp, sizeof(iarg))) {
				ret = -EFAULT;
			}

			TROUT_PRINT("\n\nFM FM_IOCTL_CONFIG set %d\n\n",iarg);

			if (iarg == 0) {
                trout_fm_pcm_pin_cfg();
			}
			else if(iarg == 1) {
                trout_fm_iis_pin_cfg();
			}
			break;			
		//chenq add end

		default:
			TROUT_PRINT("Unknown FM IOCTL!");
			return -EINVAL;
	}

	//clear interrupt
	//WRITE_REG(FM_REG_INT_CLR, 1);

	return ret;
}

int trout_fm_release(struct inode *inode, struct file *filep)
{
	TROUT_PRINT("trout_fm_misc_release");

	trout_fm_deinit();

	Set_Trout_PowerOff(2 /*FM_MODE*/);

	atomic_cmpxchg(&is_fm_open, 1, 0);

	return 0;
}

const struct file_operations trout_fm_misc_fops = {
	.owner = THIS_MODULE,
	.open  = trout_fm_open,
	.unlocked_ioctl = trout_fm_ioctl,
	.release = trout_fm_release,
};

struct miscdevice trout_fm_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = TROUT_FM_DEV_NAME,
	.fops  = &trout_fm_misc_fops,
};

int __init init_fm_driver(void)
{
	int ret = -EINVAL;
	char *ver_str = TROUT_FM_VERSION;

    p_trout_interface = NULL;

	TROUT_PRINT("**********************************************");
    TROUT_PRINT(" Trout FM driver ");
	TROUT_PRINT(" Version: %s", ver_str);
	TROUT_PRINT(" Build date: %s %s", __DATE__, __TIME__);
	TROUT_PRINT("**********************************************");
    
#ifdef USE_INTERFACE_SDIO
    trout_sdio_init(&p_trout_interface);
#endif

#ifdef USE_INTERFACE_SPI
    trout_spi_init(&p_trout_interface);
#endif

#ifdef USE_INTERFACE_SHARED
    trout_shared_init(&p_trout_interface);
#endif

    if(p_trout_interface == NULL)
    {
        TROUT_PRINT("none interface used!");
        return ret;
    }

    TROUT_PRINT("use %s interface.", p_trout_interface->name);

    ret = p_trout_interface->init();
    if(ret < 0)
    {
        TROUT_PRINT("interface init failed!");
        return ret;
    }

	ret = misc_register(&trout_fm_misc_device);
	if(ret < 0)
	{
		TROUT_PRINT("misc_register failed!");
		return ret;
	}

	TROUT_PRINT("trout_fm_init success.\n");

	return 0;
}

void __exit exit_fm_driver(void)
{
	TROUT_PRINT("exit_fm_driver!\n");
	misc_deregister(&trout_fm_misc_device);

    if(p_trout_interface)
    {
        p_trout_interface->exit();
        p_trout_interface = NULL;
    }
}


module_init(init_fm_driver);
module_exit(exit_fm_driver);

MODULE_DESCRIPTION("TROUT FM radio driver");
MODULE_AUTHOR("Spreadtrum Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(TROUT_FM_VERSION);
