/************************************************************************************************************	*/
/*****Copyright:	2014 Spreatrum. All Right Reserved									*/
/*****File: 		mdbg_sdio.c													*/
/*****Description: 	Marlin Debug System Sdio related interface functions.			*/
/*****Developer:	fan.kou@spreadtrum.com											*/
/*****Date:		06/09/2014													*/
/************************************************************************************************************	*/

/*******************************************************/
/*********************INCLUDING********************/
/*******************************************************/
#include "mdbg_sdio.h"
#include "mdbg_ring.h"
#include <linux/mutex.h>

/*******************************************************/
/******************Macor Definitions****************/
/*******************************************************/
#define MDBG_CHANNEL_READ			(14)
#define MDBG_CHANNEL_WRITE 		(15)
#define MDBG_MAX_RETRY 			(3)
#define MDBG_RX_BUFF_SIZE 			(4096)

/*******************************************************/
/******************Global Variables*****************/
/*******************************************************/
MDBG_RING_T* rx_ring;
struct mutex mdbg_read_mutex;
extern bool read_flag;

/*******************************************************/
/******************Local Variables*******************/
/*******************************************************/
LOCAL uint8_t* mdbg_rx_buff;
LOCAL struct work_struct rx_workq;
MDBG_SIZE_T sdio_read_len;
static struct wake_lock  mdbg_wake_lock;
/*******************************************************/
/***********Local Functions Declaration************/
/*******************************************************/
LOCAL void mdbg_sdio_read(void);
LOCAL MDBG_SIZE_T mdbg_sdio_write(char* buff, MDBG_SIZE_T len);

void mdbg_rx_handler(void )
{
	sdio_read_len = mdbg_ring_write(rx_ring,mdbg_rx_buff, sdio_read_len);
	read_flag = 1;
	//printk("\001" "0" "[%s]\n",__func__);
	return;
}

/*******************************************************/
/************MDBG SDIO Life Cycle****************/
/*******************************************************/
PUBLIC int mdbg_sdio_init(void)
{
	int retry_cnt = 0;
	int err = 0;
	do{
		//Step 1: Wait for sdio hal being ready.
		while(1 != get_sdiohal_status()){//check if sdio hal is ready; 
			if(retry_cnt++ > MDBG_MAX_RETRY){
				MDBG_ERR("SDIO hal not ready for a long time, init failed.");
				return (MDBG_ERR_TIMEOUT);//init failed,return immediately.
			}
			MDBG_ERR("SDIO hal not ready, wait 500ms and try again!");
			msleep(500);
		}
		MDBG_LOG("SDIO hal ready !");

		//Step 2: Init a ring buffer for sdio rx.
		wake_lock_init(&mdbg_wake_lock, WAKE_LOCK_SUSPEND, "mdbg_wake_lock");
		INIT_WORK(&rx_workq, mdbg_rx_handler);
		rx_ring = mdbg_ring(MDBG_RX_RING_SIZE);
		if(!rx_ring){
			MDBG_ERR("Ring malloc error.");
			err = (MDBG_ERR_MALLOC_FAIL);
			break;//goto init failed
		}
		MDBG_LOG("Ring alloc succeed !");
		
		//Step 3: Init mdbg_rx_buff.
		mdbg_rx_buff = kmalloc(MDBG_RX_BUFF_SIZE, GFP_KERNEL);
		if(!mdbg_rx_buff){
			MDBG_ERR("mdbg_rx_buff malloc error.");
			err = (MDBG_ERR_MALLOC_FAIL);
			break;//goto init failed
		}
		MDBG_LOG("mdbg_rx_buff alloc succeed !");
		
		//Step 4: Init sdio dev read channel.
		err = sdiodev_readchn_init(MDBG_CHANNEL_READ, mdbg_sdio_read,0);
		if(err != 0){
			MDBG_ERR("Sdio dev read channel init failed!");
			err = (MDBG_ERR_SDIO_ERR);
			break;//goto init failed
		}		
		mutex_init(&mdbg_read_mutex);
		MDBG_LOG("Sdio dev read channel init succeed !");
		//probe succeed;
		return (MDBG_SUCCESS);
	}while(0);

	//Marlin debug system probe failed.
	//clear everything.
	mdbg_sdio_remove();
	MDBG_ERR("init failed,return val:%d", err);
	return (err);
}

PUBLIC void mdbg_sdio_remove(void)
{
	MDBG_FUNC_ENTERY;
	cancel_work_sync(&rx_workq);
	wake_lock_destroy(&mdbg_wake_lock);
	sdiodev_readchn_uninit(MDBG_CHANNEL_READ);
	mdbg_ring_destroy(rx_ring);
	if(mdbg_rx_buff){
		kfree(mdbg_rx_buff);
	}
}

/*******************************************************/
/***********MDBG SDIO IO Functions**************/
/*******************************************************/
LOCAL MDBG_SIZE_T mdbg_sdio_write(char* buff, MDBG_SIZE_T len)
{
	MDBG_LOG("buff=%p,len=%d,[%s]",buff,len,buff);
	set_marlin_wakeup(MDBG_CHANNEL_WRITE,0x1);
	sdio_dev_write(MDBG_CHANNEL_WRITE, buff, len);
	return len;
}

PUBLIC void mdbg_sdio_read(void)
{
	wake_lock(&mdbg_wake_lock);
	mutex_lock(&mdbg_read_mutex);
	set_marlin_wakeup(MDBG_CHANNEL_READ,0x1);
	sdio_read_len = sdio_dev_get_chn_datalen(MDBG_CHANNEL_READ);
	if(sdio_read_len <= 0)
	{
		printk("\001" "0" "[%s][%d]\n",__func__, sdio_read_len);
		wake_unlock(&mdbg_wake_lock);
		mutex_unlock(&mdbg_read_mutex);
		return;
	}
	sdio_dev_read(MDBG_CHANNEL_READ,mdbg_rx_buff,&sdio_read_len);
	schedule_work(&rx_workq);
	//printk("\001" "0" "[%s][%d]\n",__func__, sdio_read_len);
	wake_unlock(&mdbg_wake_lock);
	mutex_unlock(&mdbg_read_mutex);
	return;
}
EXPORT_SYMBOL_GPL(mdbg_sdio_read);

/*******************************************************/
/**************MDBG IO Functions******************/
/*******************************************************/
PUBLIC MDBG_SIZE_T mdbg_send(char* buff, MDBG_SIZE_T len)
{
	MDBG_SIZE_T sent_size = 0;

	MDBG_LOG("BYTE MODE");
	wake_lock(&mdbg_wake_lock);
	sent_size = mdbg_sdio_write(buff, len);
	wake_unlock(&mdbg_wake_lock);
	return len;
}

PUBLIC MDBG_SIZE_T mdbg_receive(char* buff, MDBG_SIZE_T len)
{
	return (mdbg_ring_read(rx_ring, buff, len));
}


