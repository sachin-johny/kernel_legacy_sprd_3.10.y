#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "trout_fm_ctrl.h"
#include "trout_interface.h"

struct sdio_trout_data{
	struct sdio_func * func;
	struct mutex sdio_mutex;
};

struct sdio_trout_data  g_sdio_data;

int sdio_trout_init(struct sdio_trout_data *sdiodata)
{
	mutex_init(&sdiodata->sdio_mutex);
	return 0;
}

//Host通过SDIO读取Trout内部寄存器
unsigned int sdio_read(u32 reg_addr, u32 *reg_data)
{
	struct sdio_func * func;
	unsigned int ret = 0;
	unsigned int i;
   // int err = 0;

	func = g_sdio_data.func;
	 //convert register offset from 4 to 1. byte ->word
	 //reg_addr  = reg_addr >> 2;
     
	mutex_lock(&g_sdio_data.sdio_mutex);

	for(i = 0; i < 9; i++)
	{
		sdio_claim_host(func);
		sdio_memcpy_fromio(func, &ret, reg_addr,4);
		sdio_release_host(func);
	}
	mutex_unlock(&g_sdio_data.sdio_mutex);

	*reg_data = ret;
      
     return ret;
}


// Host通过SDIO写Trout内部寄存器
unsigned int sdio_write (unsigned int reg_addr, unsigned int val)
{
	struct sdio_func * func;
	int err = 0;

	func = g_sdio_data.func;
	
	 //convert register offset from 4 to 1.
	//reg_addr  = reg_addr >> 2;

	mutex_lock(&g_sdio_data.sdio_mutex);
	sdio_claim_host(func);
	//sdio_writel(func,val,reg_addr,&err);
	err = sdio_memcpy_toio(func,reg_addr,&val,4);
	sdio_release_host(func);
	mutex_unlock(&g_sdio_data.sdio_mutex);

	return err;

}
static int sdio_trout_probe(struct sdio_func *func,
			   const struct sdio_device_id *id)
{
	int ret = 0;
	g_sdio_data.func = func;

	TROUT_PRINT("\nsdio_trout_probe start...\n\n");
	TROUT_PRINT("sdid dev id: %x\n", *((u32*)id));

	sdio_claim_host(func);

	sdio_enable_func(func);
	sdio_set_block_size(func,512);

	sdio_release_host(func);

	sdio_trout_init(&g_sdio_data);

	TROUT_PRINT("sdio_trout_probe end.\n");

	return 0;
}

static void __devexit sdio_trout_remove(struct sdio_func *func)
{
	TROUT_PRINT("trout: sdio_trout_exit!\n");
	
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
}
static int sdio_trout_suspend(struct device *dev)
{
	TROUT_PRINT("---sdio_trout_suspend\n");
	return 0;
}

static int sdio_trout_resume(struct device *dev)
{
	TROUT_PRINT("---sdio_trout_resume\n");
	return 0;
}


#define TROUT_VENDOR_ID 0x00
#define TROUT_DEVICE_ID  0x2260

static const struct sdio_device_id sdio_trout_ids[] = {
	{SDIO_DEVICE(TROUT_VENDOR_ID,TROUT_DEVICE_ID)},
	{},
};

static const struct dev_pm_ops sdio_pm = {
	.suspend = sdio_trout_suspend,
	.resume = sdio_trout_resume,
};

static struct sdio_driver sdio_trout_driver ={
	.probe	= sdio_trout_probe,
	.remove	= sdio_trout_remove,
	.name	= "sdio_trout_fm",
	.id_table = sdio_trout_ids,
	.drv = {
		.name	= "sdio_trout_fm",
		.pm 	= &sdio_pm,
	},
};

unsigned int sdio_init(void)
{
    int ret;
    
    ret = sdio_register_driver(&sdio_trout_driver);
	TROUT_PRINT("trout init: ret=%d\n", ret);
	if (ret < 0)
		return ret;
    return 0;
}

unsigned int sdio_exit(void)
{
    sdio_unregister_driver(&sdio_trout_driver);
    return 0;
}

static trout_interface_t sdio_interface = 
{
    .name = "sdio",
    .init = sdio_init,
    .exit = sdio_exit,
    .read_reg = sdio_read,
    .write_reg = sdio_write,
};

int trout_sdio_init(trout_interface_t **p)
{
    *p = &sdio_interface;
    
    return 0;
}

