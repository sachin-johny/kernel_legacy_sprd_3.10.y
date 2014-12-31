/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Filename : sdio_dev.c
 * Abstract : This file is a implementation for itm sipc command/event function
 *
 * Authors	: gaole.zhang
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "sdio_dev.h"

#if defined(CONFIG_SDIODEV_TEST)

#include "sdio_dev_test.h"

#endif

static int marlin_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id);
static void marlin_sdio_remove(struct sdio_func *func);
static int marlin_sdio_suspend(struct device *dev);
static int marlin_sdio_resume(struct device *dev);

extern void set_bt_pm_func(void *wakeup,void *sleep);

struct sdio_func *sprd_sdio_func[SDIODEV_MAX_FUNCS];
struct sdio_func sdio_func_0;
SDIO_CHN_HANDLER sdio_tran_handle[17] = {0};


static int sdio_irq_num;
static int marlinwake_irq_num;

static int marlin_sdio_ready_irq_num;


static struct wake_lock marlin_wakelock;
static struct wake_lock marlinup_wakelock;
static struct wake_lock BT_AP_wakelock;

static struct wake_lock marlinpub_wakelock;

static struct wake_lock marlin_sdio_ready_wakelock;

static struct work_struct marlin_wq;
static struct work_struct marlinwake_wq;

static bool ap_sdio_ready = 0;
static bool have_card = 0;
static MARLIN_SDIO_READY_T marlin_sdio_ready = {0};



static const struct sdio_device_id marlin_sdio_ids[] = {
	{SDIO_DEVICE(MARLIN_VENDOR_ID, MARLIN_DEVICE_ID)},
	{},
};

static const struct dev_pm_ops marlin_sdio_pm_ops = {
	.suspend = marlin_sdio_suspend,
	.resume	= marlin_sdio_resume,
};

static struct sdio_driver marlin_sdio_driver = {
	.probe = marlin_sdio_probe,
	.remove = marlin_sdio_remove,
	.name = "marlin_sdio",
	.id_table = marlin_sdio_ids,
	.drv = {
		.pm = &marlin_sdio_pm_ops,
	},
};

static struct mmc_host *sdio_dev_host = NULL;
char *sync_data_ptr = NULL;
static int gpio_marlin_req_tag = 0;
static int gpio_marlinwake_tag = 0;

static bool sdio_w_flag = 0;
static bool bt_wake_flag = 0;
static bool marlin_bt_wake_flag = 0;


static struct completion marlin_ack = {0};

static SLEEP_POLICY_T sleep_para = {0};


volatile bool marlin_mmc_suspend = 0;
MARLIN_PM_RESUME_WAIT_INIT(marlin_sdio_wait);



static int get_sdio_dev_func(struct sdio_func *func)
{	
	if(SDIODEV_MAX_FUNCS <= func->num)
	{
		SDIOTRAN_ERR("func num err!!! func num is %d!!!",func->num);
		return -1;
	}
	SDIOTRAN_DEBUG("func num is %d!!!",func->num);

	if (func->num == 1) 
	{
		sdio_func_0.num = 0;
		sdio_func_0.card = func->card;
		sprd_sdio_func[0] = &sdio_func_0;
	}	

	sprd_sdio_func[func->num]  = func;

	return 0;
}

static int free_sdio_dev_func(struct sdio_func *func)
{
	if(SDIODEV_MAX_FUNCS <= func->num)
	{
		SDIOTRAN_ERR("func num err!!! func num is %d!!!",func->num);
		return -1;
	}
	
	sprd_sdio_func[0]  = NULL;
	sprd_sdio_func[func->num] = NULL;
	sdio_func_0.num = 0;
	sdio_func_0.card = NULL;

	return 0;

}

static int wakeup_slave_pin_init(void)
{
	int ret;
	ret = gpio_request(GPIO_AP_TO_MARLIN,"marlin_wakeup");
	if (ret)
	{
		SDIOTRAN_ERR("req gpio GPIO_MARLIN_TO_CP = %d fail!!!",\
			GPIO_AP_TO_MARLIN);
		return ret;
	}
	
	ret = gpio_direction_output(GPIO_AP_TO_MARLIN,0);
	if (ret)
	{
		SDIOTRAN_ERR("GPIO_MARLIN_TO_CP = %d input set fail!!!",\
			GPIO_AP_TO_MARLIN);
		return ret;
	}
	
	SDIOTRAN_ERR("req GPIO_MARLIN_TO_CP = %d succ!!!",\
		GPIO_AP_TO_MARLIN);
	
	return ret;
}
void gpio_timer_handler(unsigned long data)
{
	if(gpio_get_value(GPIO_MARLIN_WAKE))
	{
		sleep_para.gpio_opt_tag = 1;
		mod_timer(&(sleep_para.gpio_timer),\
			jiffies + msecs_to_jiffies(300));//   250<300<2000-1500
		SDIOTRAN_ERR("ack high");
	}
	else
	{
		sleep_para.gpio_opt_tag = 0;

	}
}

int set_marlin_wakeup(uint32 chn,uint32 user_id)
{
	//user_id: wifi=0x1;bt=0x2
#if !defined(CONFIG_MARLIN_NO_SLEEP)

	int ret;
	SDIOTRAN_DEBUG("entry");

	if(0 != sleep_para.gpio_opt_tag)		
	{
		sleep_para.gpioreq_need_pulldown = 0;
	}	
	else
	{		
		if(user_id == 1)
		{
			sdio_w_flag = 1;
		}
		
		if(user_id == 0x2)
		{
			bt_wake_flag = 1;
		}
		sleep_para.gpioreq_need_pulldown = 1;
		gpio_direction_output(GPIO_AP_TO_MARLIN,1);		
		SDIOTRAN_ERR("pull up gpio %d",GPIO_AP_TO_MARLIN);
		//sleep_para.gpioreq_up_time = jiffies;
		
		if(user_id == 1)
		{	
			ret = wait_for_completion_timeout( &marlin_ack, msecs_to_jiffies(100) );
			if (ret == 0){
			
				SDIOTRAN_ERR("marlin chn %d ack timeout!!!",chn);
				return -ETIMEDOUT;//timeout
			}
		}
	}

#endif
	return 0;
}
EXPORT_SYMBOL_GPL(set_marlin_wakeup);


int set_marlin_sleep(uint32 chn,uint32 user_id)
{
	//user_id: wifi=0x1;bt=0x2
#if !defined(CONFIG_MARLIN_NO_SLEEP)
	SDIOTRAN_ERR("entry");	
	if(sleep_para.gpioreq_need_pulldown)
	{
		gpio_direction_output(GPIO_AP_TO_MARLIN,0);
		SDIOTRAN_ERR("pull down gpio %d",GPIO_AP_TO_MARLIN);
		if(user_id == 1)
		{	
			sdio_w_flag = 0;
		}
		if(user_id == 0x2)
		{
			bt_wake_flag = 0;
		}
	}
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(set_marlin_sleep);


/*only idle chn can be used for ap write*/
int  sdio_dev_chn_idle(uint32 chn)
{
	uint32 active_status;
	uint8  status0,status1;	
	int err_ret;
	
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if(NULL == sprd_sdio_func[SDIODEV_FUNC_0])
	{
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	
	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x840,&err_ret);

	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x841,&err_ret);
     
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	
	active_status = (uint32)(status0) + ((uint32)(status1) << 8);

	if( BIT_0 & ( active_status >> chn )){
		SDIOTRAN_DEBUG("SDIO channel num = %x is active!!",chn);
		return 0;	//active
	}
	else{
		SDIOTRAN_DEBUG("SDIO channel num = %x is idle!!",chn);
		return 1;  //idle
	}
}
EXPORT_SYMBOL_GPL(sdio_dev_chn_idle);

#if defined(CONFIG_SDIODEV_TEST)

/*only active chn can be readed*/
int  sdio_dev_get_read_chn(void)
{
	uint8  rw_flag0,rw_flag1;
	uint8  status0,status1;
	uint32 chn_status;
	uint32 chn_rw_flag;
	uint32 tmp;
	int err_ret,read_chn;
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x840,&err_ret);
	
	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x841,&err_ret);
	
	rw_flag0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x842,&err_ret);
	
	rw_flag1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x843,&err_ret);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	chn_status = (uint32)(status0) + ((uint32)(status1) << 8);
	chn_rw_flag = (uint32)(rw_flag0) + ((uint32)(rw_flag1) << 8);

	SDIOTRAN_DEBUG("ap recv: chn_status is 0x%x!!!",chn_status);
	SDIOTRAN_DEBUG("ap read: chn_rw_flag is 0x%x!!!",chn_rw_flag);

	tmp = (chn_status&(~chn_rw_flag))&(0x0000ffff);

	for(read_chn=0;read_chn<16;read_chn++)
	{
		if(BIT_0 & (tmp>>read_chn))
		{
		 	return read_chn;
		}
	}
	return -1;

}



#else

int  sdio_dev_get_read_chn(void)
{
	uint8  chn_status;
	int err_ret;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	chn_status = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x841,&err_ret);

	while( chn_status == 0 ){
		chn_status = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x841,&err_ret);		
	}
	
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	SDIOTRAN_DEBUG("ap read: chn_status is 0x%x!!!",chn_status);

	if(chn_status == 0xff)
	{
		SDIOTRAN_ERR("Sdio Err!!!chn status 0x%x!!!",chn_status);
		return -1;
	}
	else if(chn_status & SDIO_CHN_8)
		return 8;
	else if(chn_status & SDIO_CHN_9)
		return 9;
	else if(chn_status & SDIO_CHN_11)
		return 11;
	else if(chn_status & SDIO_CHN_12)
		return 12;
	else if(chn_status & SDIO_CHN_14)
		return 14;
	else if(chn_status & SDIO_CHN_15)
		return 15;
	else
	{		
		SDIOTRAN_ERR("Invalid sdio read chn!!!chn status 0x%x!!!",chn_status);
		return -1;
	}
}

#endif
EXPORT_SYMBOL_GPL(sdio_dev_get_read_chn);


/*ONLY WIFI WILL USE THIS FUNC*/
int  sdio_chn_status(unsigned short chn, unsigned short *status)
{
	unsigned char status0 = 0;
	unsigned char status1 = 0;	
	int err_ret;

	SDIOTRAN_DEBUG("ENTRY");
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if(NULL == sprd_sdio_func[SDIODEV_FUNC_0])
	{
		return -1;
	}
	
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	
	if(0x00FF & chn)
		status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x840,&err_ret);

	if(0xFF00 & chn)
		status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],0x841,&err_ret);   

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	
	*status = ( (status0+(status1 << 8)) & chn );
	
	return 0;	
}
EXPORT_SYMBOL_GPL(sdio_chn_status);

int  sdio_dev_get_chn_datalen(uint32 chn)
{
	uint32 usedata;
	uint8  status0,status1,status2;	
	int err_ret;
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if(NULL == sprd_sdio_func[SDIODEV_FUNC_0])
	{
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],(0x800+ 4*chn),&err_ret);
	
	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],(0x801+ 4*chn),&err_ret);
	
	status2 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],(0x802+ 4*chn),&err_ret);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]); 
	
	usedata = ((uint32)(status0) ) + ((uint32)(status1) << 8) + ((uint32)(status2)  << 16);

	if(status0 == 0xff || status1 == 0xff || status2 == 0xff)
	{
		SDIOTRAN_ERR("read err!!!");
		return -1;
	}

	return usedata;	
	
}
EXPORT_SYMBOL_GPL(sdio_dev_get_chn_datalen);


int sdio_dev_write(uint32 chn,void* data_buf,uint32 count)
{
	int ret,data_len;
	
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	if(NULL == sprd_sdio_func[SDIODEV_FUNC_1])
	{
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	if(chn != 0 && chn != 1)
	{
		data_len = sdio_dev_get_chn_datalen(chn);
		if(data_len < count || data_len <= 0)
		{
			SDIOTRAN_ERR("chn %d, len %d, cnt %d, err!!!",chn,data_len,count);
			return -1;
		}
	}
	
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);	

	ret = sdio_memcpy_toio(sprd_sdio_func[SDIODEV_FUNC_1],chn,data_buf,count);
	SDIOTRAN_DEBUG("chn %d send ok!!!",chn);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);	

	return ret;

}
EXPORT_SYMBOL_GPL(sdio_dev_write);


int  sdio_dev_read(uint32 chn,void* read_buf,uint32 *count)
{
	int err_ret,data_len;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	if(NULL == sprd_sdio_func[SDIODEV_FUNC_1])
	{
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	data_len = sdio_dev_get_chn_datalen(chn);
	if(data_len <= 0)
	{
		SDIOTRAN_ERR("chn %d,datelen %d err!!!",chn,data_len);
		return -1;
	}

	SDIOTRAN_DEBUG("ap recv chn %d: read_datalen is 0x%x!!!",chn,data_len);
	
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);

	err_ret = sdio_memcpy_fromio(sprd_sdio_func[SDIODEV_FUNC_1],read_buf,chn,data_len);
	
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);

	if(NULL != count)
		*count = data_len;
	


	return err_ret;

}

EXPORT_SYMBOL_GPL(sdio_dev_read);

int  sdio_read_wlan(uint32 chn,void* read_buf,uint32 *count)
{
	int err_ret,data_len;
	data_len = 16384;
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if(NULL == sprd_sdio_func[SDIODEV_FUNC_1])
	{
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	err_ret = sdio_memcpy_fromio(sprd_sdio_func[SDIODEV_FUNC_1],read_buf,chn,data_len);
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	if(NULL != count)
		*count = data_len;
	return err_ret;

}

EXPORT_SYMBOL_GPL(sdio_read_wlan);

int sdiodev_readchn_init(uint32 chn,void* callback,bool with_para)
{
	SDIOTRAN_DEBUG("Param, chn = %d, callback = %p",chn,callback);
	if(chn >= INVALID_SDIO_CHN)
	{
		SDIOTRAN_ERR("err input chn %d",chn);
		return -1;
	}
	sdio_tran_handle[chn].chn = chn;
	
	if(with_para == 1)
	{
		sdio_tran_handle[chn].tran_callback_para = callback;		
		SDIOTRAN_DEBUG("sdio_tran_handle, chn = %d, callback = %p",sdio_tran_handle[chn].chn,sdio_tran_handle[chn].tran_callback_para);
	}
	else
	{
		sdio_tran_handle[chn].tran_callback = callback;		
		SDIOTRAN_DEBUG("sdio_tran_handle, chn = %d, callback = %p",sdio_tran_handle[chn].chn,sdio_tran_handle[chn].tran_callback);
	}	
	
	return 0;
}
EXPORT_SYMBOL_GPL(sdiodev_readchn_init);

int sdiodev_readchn_uninit(uint32 chn)
{
	if(chn >= INVALID_SDIO_CHN)
	{
		SDIOTRAN_ERR("err input chn %d",chn);
		return -1;
	}

	sdio_tran_handle[chn].chn = INVALID_SDIO_CHN;
	sdio_tran_handle[chn].tran_callback = NULL;
	sdio_tran_handle[chn].tran_callback_para = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(sdiodev_readchn_uninit);

void invalid_recv_flush(uint32 chn)
{
	int len ;
	char* pbuff = NULL;
	
	if(chn < 0 || chn > 15)
	{
		SDIOTRAN_ERR("err chn");
		return;
	}

	len = sdio_dev_get_chn_datalen(chn);	
	if(len <= 0){
		SDIOTRAN_ERR("chn %d, len err %d!!",chn,len);
		return;
	}
	SDIOTRAN_ERR("NO VALID DATA! CHN %d FLUSH! DATALEN %d!",chn,len);	
	pbuff = kmalloc(len,GFP_KERNEL);
	if(pbuff){
		SDIOTRAN_DEBUG("Read to Flush,chn=%d,pbuff=%p,len=%d",chn, pbuff, len);
		sdio_dev_read(chn, pbuff, NULL);
		kfree(pbuff);
	}else{
		SDIOTRAN_ERR("Kmalloc %d failed!",len);
	}
}
EXPORT_SYMBOL_GPL(invalid_recv_flush);

int sdiolog_handler(void)
{
	if(NULL != sdio_tran_handle[SDIOLOG_CHN].tran_callback)
	{
		SDIOTRAN_DEBUG("SDIOLOG_CHN tran_callback=%p",sdio_tran_handle[SDIOLOG_CHN].tran_callback);
		sdio_tran_handle[SDIOLOG_CHN].tran_callback();
		return 0;
	}
	else
	{		
		return -1;
	}
}

int sdio_download_handler(void)
{
	if(NULL != sdio_tran_handle[DOWNLOAD_CHANNEL_READ].tran_callback)
	{
		SDIOTRAN_DEBUG("DOWNLOAD_CHN tran_callback=%p",sdio_tran_handle[DOWNLOAD_CHANNEL_READ].tran_callback);
		sdio_tran_handle[DOWNLOAD_CHANNEL_READ].tran_callback();
		return 0;
	}
	else
	{
		return -1;
	}
}

int sdio_pseudo_atc_handler(void)
{
	SDIOTRAN_ERR("ENTRY");

	if(NULL != sdio_tran_handle[PSEUDO_ATC_CHANNEL_READ].tran_callback)
	{
		SDIOTRAN_ERR("tran_callback=%p",sdio_tran_handle[PSEUDO_ATC_CHANNEL_READ].tran_callback);
		sdio_tran_handle[PSEUDO_ATC_CHANNEL_READ].tran_callback();
		return 0;
	}
	else
	{
		return -1;
	}
}

int sdio_pseudo_loopcheck_handler(void)
{
	SDIOTRAN_ERR("ENTRY");

	if(NULL != sdio_tran_handle[PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback)
	{
		SDIOTRAN_ERR("tran_callback=%p",sdio_tran_handle[PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback);
		sdio_tran_handle[PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback();
		return 0;
	}
	else
	{
		return -1;
	}
}


int sdio_wifi_handler(uint32 chn)
{
	if(NULL != sdio_tran_handle[chn].tran_callback_para)
	{
		SDIOTRAN_DEBUG("wifi_handle_chn %d  tran_callback=%p",chn,sdio_tran_handle[chn].tran_callback_para);
		sdio_tran_handle[chn].tran_callback_para(chn);
		return 0;
	}
	else
	{
		return -1;
	}
}

static void marlin_workq(void)
{
	int read_chn;
	int ret = 0;

	SDIOTRAN_DEBUG("ENTRY");

#if !defined(CONFIG_SDIODEV_TEST)
		
	read_chn = sdio_dev_get_read_chn();
	switch(read_chn)
	{
		case SDIOLOG_CHN:
			ret = sdiolog_handler();
			break;
		case DOWNLOAD_CHANNEL_READ:
			ret = sdio_download_handler();
			break;
		case PSEUDO_ATC_CHANNEL_READ:
			ret = sdio_pseudo_atc_handler();
			break;
		case PSEUDO_ATC_CHANNEL_LOOPCHECK:
			ret = sdio_pseudo_loopcheck_handler();
			break;
		case WIFI_CHN_8:
		case WIFI_CHN_9:
			ret = sdio_wifi_handler(read_chn);
			break;
		default:
			SDIOTRAN_ERR("no handler for this chn",read_chn);
			invalid_recv_flush(read_chn);
	}

	if(-1 == ret)
		invalid_recv_flush(read_chn);

#else

#if defined(SDIO_TEST_CASE4)
	case4_workq_handler();
#endif

#if defined(SDIO_TEST_CASE5)
	case5_workq_handler();
#endif

#if defined(SDIO_TEST_CASE6)
	case6_workq_handler();
#endif

#if defined(SDIO_TEST_CASE9)
	case9_workq_handler();
#endif

#endif//CONFIG_SDIODEV_TEST

	wake_unlock(&marlin_wakelock);

}

int fwdownload_fin = 0;

int get_sprd_download_fin(void)
{
	return fwdownload_fin;
}
EXPORT_SYMBOL_GPL(get_sprd_download_fin);

void set_sprd_download_fin(int dl_tag)
{
	return fwdownload_fin = dl_tag;
}
EXPORT_SYMBOL_GPL(set_sprd_download_fin);

static irqreturn_t marlin_irq_handler(int irq, void * para)
{
	disable_irq_nosync(irq);
	
	wake_lock(&marlin_wakelock);
	irq_set_irq_type(irq,IRQF_TRIGGER_RISING);


	if(NULL != sdio_tran_handle[WIFI_CHN_8].tran_callback_para)
	{
		sdio_tran_handle[WIFI_CHN_8].tran_callback_para(WIFI_CHN_8);
		wake_unlock(&marlin_wakelock);
	}
	else
		schedule_work(&marlin_wq);

	enable_irq(irq);
	
	return IRQ_HANDLED;



}

static int sdio_dev_intr_init(void)
{
	int ret;
	

	wake_lock_init(&marlin_wakelock, WAKE_LOCK_SUSPEND, "marlin_wakelock");
	
	INIT_WORK(&marlin_wq, marlin_workq);	

	gpio_request(GPIO_MARLIN_TO_AP,"marlin_irq");
	if (ret)
	{
		SDIOTRAN_ERR("req gpio GPIO_MARLIN_TO_AP = %d fail!!!",\
			GPIO_MARLIN_TO_AP);
		return ret;
	}
	
	gpio_direction_input(GPIO_MARLIN_TO_AP);
	if (ret)
	{
		SDIOTRAN_ERR("GPIO_MARLIN_TO_AP = %d input set fail!!!",\
			GPIO_MARLIN_TO_AP);
		return ret;

	}

	sdio_irq_num = gpio_to_irq(GPIO_MARLIN_TO_AP);
	
	ret = request_irq(sdio_irq_num, marlin_irq_handler,IRQF_TRIGGER_RISING |IRQF_NO_SUSPEND, "sdio_dev_intr", NULL );
	//ret = request_irq(sdio_irq_num, marlin_irq_handler,IRQF_TRIGGER_HIGH | IRQF_ONESHOT |IRQF_NO_SUSPEND, "sdio_dev_intr", NULL );
	if (ret != 0) {
		SDIOTRAN_ERR("request irq err!!!gpio is %d!!!",GPIO_MARLIN_TO_AP);
		return ret;
	}
	
	SDIOTRAN_ERR("ok!!!irq is %d!!!",sdio_irq_num);

	return 0;
}

static void sdio_dev_intr_uninit(void)
{
	disable_irq_nosync(sdio_irq_num);
	free_irq(sdio_irq_num,NULL);
	gpio_free(GPIO_MARLIN_TO_AP);
	gpio_free(GPIO_AP_TO_MARLIN);

	cancel_work_sync(&marlin_wq);
	//flush_workqueue(&marlin_wq);
	//destroy_workqueue(&marlin_wq);

	wake_lock_destroy(&marlin_wakelock);
	
	SDIOTRAN_ERR("ok!!!");

}
static void set_apsdiohal_ready(void)
{
	ap_sdio_ready = 1;
}

static void set_apsdiohal_unready(void)
{
	ap_sdio_ready = 0;
}

//return 1 means ap sdiohal ready
bool get_apsdiohal_status(void)
{
	return ap_sdio_ready;
}
EXPORT_SYMBOL_GPL(get_apsdiohal_status);

//return 1 means marlin sdiohal ready
bool get_sdiohal_status(void)
{
	return marlin_sdio_ready.marlin_sdio_init_end_tag;
}
EXPORT_SYMBOL_GPL(get_sdiohal_status);

static void clear_sdiohal_status(void)
{
	marlin_sdio_ready.marlin_sdio_init_start_tag = 0;
	marlin_sdio_ready.marlin_sdio_init_end_tag = 0;
}

static irqreturn_t marlinsdio_ready_irq_handler(int irq, void * para)
{
	disable_irq_nosync(irq);	
	wake_lock(&marlin_sdio_ready_wakelock);
	
	SDIOTRAN_ERR("entry");
	
	if(!marlin_sdio_ready.marlin_sdio_init_start_tag){
		SDIOTRAN_ERR("start");
		irq_set_irq_type(irq,IRQF_TRIGGER_LOW);
		marlin_sdio_ready.marlin_sdio_init_start_tag = 1;}
	else{
		SDIOTRAN_ERR("end");
		irq_set_irq_type(irq,IRQF_TRIGGER_HIGH);
		marlin_sdio_ready.marlin_sdio_init_end_tag = 1;}
	
	if(!marlin_sdio_ready.marlin_sdio_init_end_tag)
		enable_irq(irq);
	
	wake_unlock(&marlin_sdio_ready_wakelock);
	return IRQ_HANDLED;

}

static int marlin_sdio_sync_init(void)
{
	int ret;	

	SDIOTRAN_ERR("entry");

	sci_glb_clr(SPRD_PIN_BASE + 0x27c,(BIT(3)|BIT(7)|0));
	sci_glb_set(SPRD_PIN_BASE + 0x27c,(BIT(4)|BIT(5)|BIT(2)|BIT(6)));

	wake_lock_init(&marlin_sdio_ready_wakelock, WAKE_LOCK_SUSPEND, "marlin_sdio_ready_wakelock");

	gpio_request(GPIO_MARLIN_SDIO_READY,"marlin_sdio_ready_irq");
	if (ret)
	{
		SDIOTRAN_ERR("req gpio GPIO_MARLIN_SDIO_READY = %d fail!!!",\
			GPIO_MARLIN_SDIO_READY);
		return ret;
	}
	
	gpio_direction_input(GPIO_MARLIN_SDIO_READY);
	if (ret)
	{
		SDIOTRAN_ERR("GPIO_MARLIN_SDIO_READY = %d input set fail!!!",\
			GPIO_MARLIN_SDIO_READY);
		return ret;

	}

	marlin_sdio_ready_irq_num = gpio_to_irq(GPIO_MARLIN_SDIO_READY);
	SDIOTRAN_ERR("req irq");
	ret = request_irq(marlin_sdio_ready_irq_num, marlinsdio_ready_irq_handler,IRQF_TRIGGER_RISING |IRQF_NO_SUSPEND, "m_sdio_ready_intr", NULL );
	if (ret != 0) {
		SDIOTRAN_ERR("request irq err!!!gpio is %d!!!",GPIO_MARLIN_SDIO_READY);
		return ret;
	}
	
	SDIOTRAN_ERR("ok!!!irq is %d!!!",marlin_sdio_ready_irq_num);

	return 0;
}

void marlin_sdio_sync_uninit(void)
{
	free_irq(marlin_sdio_ready_irq_num,NULL);
	gpio_free(GPIO_MARLIN_SDIO_READY);
	wake_lock_destroy(&marlin_sdio_ready_wakelock);	
	sci_glb_clr(SPRD_PIN_BASE + 0x27c,(BIT(4)|BIT(5)|0));
	SDIOTRAN_ERR("ok!!!");
}
EXPORT_SYMBOL_GPL(marlin_sdio_sync_uninit);


static void marlinack_workq(void)
{
	int read_chn;
	int ret = 0;

	SDIOTRAN_ERR("ENTRY");
	if(sdio_w_flag == 1)
		complete(&marlin_ack);
	wake_unlock(&marlinup_wakelock);

}

static irqreturn_t marlinwake_irq_handler(int irq, void * para)
{
	uint32 gpio_wake_status = 0;
	disable_irq_nosync(irq);
	
	wake_lock(&marlinup_wakelock);	
	
	SDIOTRAN_DEBUG("ENTRY!!!");

	irq_set_irq_type(irq,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
	gpio_wake_status = gpio_get_value(GPIO_MARLIN_WAKE);

	if(gpio_wake_status)
	{
		sleep_para.gpio_up_time = jiffies;		
		sleep_para.gpio_opt_tag = 1;		
		mod_timer(&(sleep_para.gpio_timer),  jiffies + msecs_to_jiffies(sleep_para.marlin_waketime) );
	}
	else
	{
		sleep_para.gpio_down_time = jiffies;
	}
	
	if((!gpio_wake_status) && time_after(sleep_para.gpio_down_time,sleep_para.gpio_up_time))
	{
		if(jiffies_to_msecs(sleep_para.gpio_down_time -\
			sleep_para.gpio_up_time)>200)
		{
			SDIOTRAN_DEBUG("ENTRY bt irq!!!");
			marlin_bt_wake_flag = 1;
			wake_lock_timeout(&BT_AP_wakelock, HZ*2);    //wsh
		}

	}
	

	
	//schedule_work(&marlinack_wq);
	if(gpio_wake_status)
	{
		wake_lock_timeout(&marlinpub_wakelock, HZ*1); 
		if(sleep_para.gpioreq_need_pulldown)
		{
			SDIOTRAN_DEBUG("ENTRY sdio irq!!!");
			if(sdio_w_flag == 1){
				complete(&marlin_ack);
				set_marlin_sleep(0xff,0x1);
			}
			else{
				set_marlin_sleep(0xff,0x2);
			}
		}

	}
	


	
	/*
	else if(marlin_bt_wake_flag == 1 && )
	{
		SDIOTRAN_ERR("ENTRY bt irq!!!");
   		wake_lock_timeout(&BT_AP_wakelock, HZ*10);    //wsh
	}*/

	enable_irq(irq);
	wake_unlock(&marlinup_wakelock);

	return IRQ_HANDLED;
}


static int marlin_wake_intr_init(void)
{
	int ret;
	sdio_w_flag = 0;

	INIT_WORK(&marlinwake_wq, marlinack_workq);	

	init_completion (&marlin_ack);

	wake_lock_init(&marlinup_wakelock, WAKE_LOCK_SUSPEND, "marlinup_wakelock");
	wake_lock_init(&BT_AP_wakelock, WAKE_LOCK_SUSPEND, "marlinbtup_wakelock");	
	
	wake_lock_init(&marlinpub_wakelock, WAKE_LOCK_SUSPEND, "marlinpub_wakelock");	
	
	gpio_request(GPIO_MARLIN_WAKE,"marlinwake_irq");
	if (ret)
	{
		SDIOTRAN_ERR("req gpio GPIO_MARLIN_WAKE = %d fail!!!",\
			GPIO_MARLIN_WAKE);
		return ret;
	}
	
	gpio_direction_input(GPIO_MARLIN_WAKE);
	if (ret)
	{
		SDIOTRAN_ERR("GPIO_MARLIN_WAKE = %d input set fail!!!",\
			GPIO_MARLIN_WAKE);
		return ret;

	}

	marlinwake_irq_num = gpio_to_irq(GPIO_MARLIN_WAKE);
	
	ret = request_irq(marlinwake_irq_num, marlinwake_irq_handler,IRQF_TRIGGER_RISING |IRQF_NO_SUSPEND, "marlin_wake_intr", NULL );
	//ret = request_irq(marlinwake_irq_num, marlinwake_irq_handler,IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND, "marlin_wake_intr", NULL );
	if (ret != 0) {
		SDIOTRAN_ERR("request irq err!!!gpio is %d!!!",GPIO_MARLIN_WAKE);
		return ret;
	}
	
	SDIOTRAN_ERR("ok!!!irq is %d!!!",marlinwake_irq_num);

	return 0;
}

static void marlin_wake_intr_uninit(void)
{
	disable_irq_nosync(marlinwake_irq_num);
	free_irq(marlinwake_irq_num,NULL);
	gpio_free(GPIO_MARLIN_WAKE);

	
	cancel_work_sync(&marlinwake_wq);
	//flush_workqueue(&marlinwake_wq);
	//destroy_workqueue(&marlinwake_wq);

	wake_lock_destroy(&marlinup_wakelock);
	wake_lock_destroy(&BT_AP_wakelock);
	wake_lock_destroy(&marlinpub_wakelock);

	SDIOTRAN_ERR("ok!!!");

}



static void sdio_tran_sync(void)
{

	int datalen,i,j;

	SDIOTRAN_ERR("entry");

	set_blklen(4);

	while(!sdio_dev_chn_idle(SDIO_SYNC_CHN))
	{
		SDIOTRAN_ERR("sync chn is active!!!");
		//return;
	}

	while(1)
	{
		datalen = sdio_dev_get_chn_datalen(SDIO_SYNC_CHN);
		if(datalen > 0)
		{
			SDIOTRAN_ERR("channel %d len is =%d!!!",SDIO_SYNC_CHN,datalen);
			break;
		}
		else
			SDIOTRAN_ERR("channel %d len is =%d!!!",SDIO_SYNC_CHN,datalen);

	}

	sync_data_ptr = kmalloc(4,GFP_KERNEL);
	if (NULL == sync_data_ptr)
	{
		SDIOTRAN_ERR("kmalloc sync buf err!!!");
		return -1;
	}

	SDIOTRAN_ERR("kmalloc sync buf ok!!!");

	for(i=0;i<4;i++)
		*(sync_data_ptr+i) = i;

	sdio_dev_write(SDIO_SYNC_CHN,sync_data_ptr,\
		((datalen > 4) ? 4: datalen));

	kfree(sync_data_ptr);

	set_blklen(512);
}


void set_blklen(int blklen)
{
	sdio_claim_host(sprd_sdio_func[1]);
	sdio_set_block_size(sprd_sdio_func[1], blklen);
	sdio_release_host(sprd_sdio_func[1]);

	SDIOTRAN_ERR("set blklen = %d!!!",blklen);

}

static void sdio_init_timer(void)
{
	init_timer(&sleep_para.gpio_timer);
	sleep_para.gpio_timer.function = gpio_timer_handler;
	sleep_para.marlin_waketime = 1500;
	sleep_para.gpio_opt_tag = 0;
	sleep_para.gpioreq_need_pulldown = 1;
}


static void sdio_uninit_timer(void)
{
	del_timer(&sleep_para.gpio_timer);
	sleep_para.marlin_waketime = 0;
	sleep_para.gpio_opt_tag = 0;
	sleep_para.gpioreq_need_pulldown = 0;
}

static int marlin_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	int ret;
	
	SDIOTRAN_ERR("sdio drv and dev match!!!");

	ret = get_sdio_dev_func(func);
	if(ret < 0)
	{
		SDIOTRAN_ERR("get func err!!!\n");
		return ret;
	}
	SDIOTRAN_ERR("get func ok!!!\n");

	/* Enable Function 1 */
	sdio_claim_host(sprd_sdio_func[1]);
	ret = sdio_enable_func(sprd_sdio_func[1]);
	sdio_set_block_size(sprd_sdio_func[1], 512);
	sdio_release_host(sprd_sdio_func[1]);
	if (ret < 0) {
		SDIOTRAN_ERR("enable func1 err!!! ret is %d",ret);
		return ret;
	}
	SDIOTRAN_ERR("enable func1 ok!!!");

	wakeup_slave_pin_init();
	marlin_wake_intr_init();
	marlin_sdio_sync_init();
//case6
#if defined(CONFIG_SDIODEV_TEST)
		gaole_creat_test();
#endif

	sdio_dev_intr_init();

#if defined(CONFIG_SDIODEV_TEST)
	sdio_tran_sync();
#endif

/*case1
#if defined(CONFIG_SDIODEV_TEST)
	gaole_creat_test();
#endif
*/
	set_bt_pm_func(set_marlin_wakeup,set_marlin_sleep);
	sdio_init_timer();

	set_apsdiohal_ready();
	have_card = 1;
	return 0;
}

static void marlin_sdio_remove(struct sdio_func *func)
{
	SDIOTRAN_DEBUG("entry");
	sdio_uninit_timer();
	free_sdio_dev_func(func);
	sdio_dev_intr_uninit();
	marlin_wake_intr_uninit();
	//marlin_sdio_sync_uninit();
	set_apsdiohal_unready();
	clear_sdiohal_status();

	SDIOTRAN_DEBUG("ok");

}

static int marlin_sdio_suspend(struct device *dev)
{
	SDIOTRAN_DEBUG("entry");

	marlin_mmc_suspend = 1;

	gpio_marlin_req_tag = gpio_get_value(GPIO_MARLIN_TO_AP);

	if(gpio_marlin_req_tag)
		SDIOTRAN_ERR("err marlin_req!!!");
	else
		irq_set_irq_type(sdio_irq_num,IRQF_TRIGGER_HIGH);

	gpio_marlinwake_tag = gpio_get_value(GPIO_MARLIN_WAKE);
	
	if(gpio_marlinwake_tag)
		SDIOTRAN_ERR("err marlinwake!!!");
	else
		irq_set_irq_type(marlinwake_irq_num,IRQF_TRIGGER_HIGH);
	

	
	smp_mb();
	
	return 0;
}
static int marlin_sdio_resume(struct device *dev)
{
	SDIOTRAN_DEBUG("entry");
	
	marlin_mmc_suspend = 0;

	gpio_marlin_req_tag = gpio_get_value(GPIO_MARLIN_TO_AP);
	if(!gpio_marlin_req_tag)
		irq_set_irq_type(sdio_irq_num,IRQF_TRIGGER_RISING);

	gpio_marlinwake_tag = gpio_get_value(GPIO_MARLIN_WAKE);
	if(!gpio_marlinwake_tag)
		irq_set_irq_type(marlinwake_irq_num,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);	

	smp_mb();
	
	return 0;
}

static void* sdio_dev_get_host(void) 
{
	struct device *dev;

	struct sdhci_host *host;

	struct platform_device *pdev;

	dev = bus_find_device_by_name(&platform_bus_type, NULL, "sprd-sdhci.1");
	if(dev == NULL)
	{
		SDIOTRAN_ERR("sdio find dev by name failed!!!");
		return NULL;
	}

	pdev  = to_platform_device(dev);
	if(pdev == NULL)
	{
		SDIOTRAN_ERR("sdio dev get platform device failed!!!");
		return NULL;
	}

	host = platform_get_drvdata(pdev);	
	return container_of(host, struct mmc_host, private);

		
}

void  marlin_sdio_uninit(void)
{
	sdio_unregister_driver(&marlin_sdio_driver);
	sdio_dev_host = NULL;
	have_card = 0;

	SDIOTRAN_ERR("ok");
}

int marlin_sdio_init(void)
{
	int ret;
	SDIOTRAN_ERR("entry");
	if(have_card == 1){
		marlin_sdio_uninit();
	}

	sdio_dev_host = sdio_dev_get_host();
	if(NULL == sdio_dev_host)
	{
		SDIOTRAN_ERR("get host failed!!!");
		return -1;
	}
	SDIOTRAN_ERR("sdio get host ok!!!");

	ret = sdio_register_driver(&marlin_sdio_driver);
	if(0 != ret) 
	{
		SDIOTRAN_ERR("sdio register drv err!!!ret is %d!!!",ret);
		return -1;
	}
	
	SDIOTRAN_ERR("sdio register drv succ!!!");

	flush_delayed_work(&sdio_dev_host->detect);
	mmc_detect_change(sdio_dev_host, 0);
	
	return ret;
}
EXPORT_SYMBOL_GPL(marlin_sdio_init);


