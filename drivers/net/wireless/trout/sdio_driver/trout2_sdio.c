/*
 * SDIO interface for SpreadTrum Trout
 *
 * Copyright (C) 2012 SpreadTrum Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kallsyms.h>
//xiong add for trout_sdio.ko debug
#include <linux/mmc/sdio.h>
#include "trout2_interface.h"

#define TROUT_SDIO_VERSION	"0.27"
#define TROUT_SDIO_BLOCK_SIZE	512	/* rx fifo max size in bytes */

//#define POW_CTRL_VALUE		0x700 /*for Trout1*/ 
#define POW_CTRL_VALUE		0x500 /*for Trout2*/ 

#define TROUT_SHARE_MEM_BASE_ADDR 0x10000	/* word address */
#define TROUT_BT_RAM_BASE_ADDR    0x5000	/* word address */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
/*zhou huiquan add for protect read/write register*/
#define TROUT_AWAKE                      1
#define TROUT_DOZE                 	 0
/*leon liu added TROUT_SLEEP state*/
#define TROUT_SLEEP			 2	
#define LOCK_TURE                        1
#define LOCK_FALSE                       0
#define SYS_ADDR_MAX                     (0x1FF<<2)
#define SYS_ADDR_MIN                     0
#define FM_ADDR_MIN                      (0x3000<<2)
#define FM_ADDR_MAX                      (0x3FFF<<2)
#define UWORD32                          unsigned int

typedef void (*sdio_trout_awake)(bool flag);
sdio_trout_awake trout_awake_fn = NULL;

extern void tempr_compensated(void);//by lihua
extern int update_cfg_front_nv(void);
extern void  MxdRfSetClkReqLow(void);
extern unsigned int check_tpc_switch_from_nv(void);
DEFINE_MUTEX(rw_reg_mutex);

unsigned char g_trout_state = TROUT_AWAKE;
unsigned int g_done_wifi_suspend = 0;

typedef void (*wakeup_from_low_power_mode)(bool flag);
wakeup_from_low_power_mode wake_low_power_fn = NULL;

WIFI_POWER_MODE g_wifi_power_mode = WIFI_NORMAL_POWER_MODE;
EXPORT_SYMBOL(wake_low_power_fn);
EXPORT_SYMBOL(g_wifi_power_mode);

//xuanyang, 2013-11-1, prevent fm that is in the selecting channel state 
//and wifi-sleep from colliding
DEFINE_MUTEX(g_wifi_sleep_mutex);

EXPORT_SYMBOL(g_trout_state);
EXPORT_SYMBOL(rw_reg_mutex);
EXPORT_SYMBOL(trout_awake_fn);
EXPORT_SYMBOL(g_done_wifi_suspend);
EXPORT_SYMBOL(tempr_compensated);
EXPORT_SYMBOL(update_cfg_front_nv);
EXPORT_SYMBOL(MxdRfSetClkReqLow);
EXPORT_SYMBOL(check_tpc_switch_from_nv);
#endif
struct sdio_func *trout_sdio_func = NULL;
struct sdio_func *cur_sdio_func = NULL;/* Current need  */
static unsigned int sdio_can_sleep = 1;

#define TROUT_MODULE_FAIL                0
#define TROUT_MODULE_LOADING             1
#define TROUT_MODULE_LOADED              2
#define TROUT_MODULE_UNLOADING           3
#define TROUT_MODULE_UNLOADED            4
#define TROUT_MODULE_IDLE                5

//xuan.yang, 2013-10-16, save trout module state
static unsigned int g_trout_module_state = TROUT_MODULE_IDLE;
	
void set_trout_module_state(unsigned int st)
{
      g_trout_module_state = st;
}
EXPORT_SYMBOL(set_trout_module_state);
      
unsigned int get_trout_module_state(void)
{
       return g_trout_module_state;
}
EXPORT_SYMBOL(get_trout_module_state);
	
unsigned int check_trout_module_state(unsigned int st)
{
       if(g_trout_module_state == st) {
           return 1;
       }      
	    return 0;
}
EXPORT_SYMBOL(check_trout_module_state);

#if 0
#define TROUT_PRINT_STACK  if(1){ PRINT_TROUT_STACK_FN(__func__, __LINE__);}
inline void PRINT_TROUT_STACK_FN(const char *func_name, UWORD32 line)
{
    printk("%s-%d: ", func_name, line);    
#ifdef DV_SIM
    print_symbol("at %s", (unsigned long)__builtin_return_address(0));
    print_symbol(" <= %s", (unsigned long)__builtin_return_address(1));
    print_symbol(" <= %s\n", (unsigned long)__builtin_return_address(2));
#else
    print_symbol("at %s\n", (unsigned long)__builtin_return_address(0));
#endif
}
#endif
extern unsigned int g_trout_power_mode; // add for sdio suspend and resume!
extern int  MxdRfSwtchLnaCfg(UWORD32);
extern void mmc_set_clock(struct mmc_host*, unsigned int);
extern void  MxdRfGetRfMode(unsigned int * pFlagFmBit2BtBit1WfBit0 );
bool is_wifi_in_sleep(void);

//Export for Trout2 Interface (12Mhz->48Mhz)
void trout_sdio_set_clock(unsigned int Hz)
{
	printk("%s: trout set sdio clock to [%d]HZ\n", __func__, Hz);

	if(!cur_sdio_func)
	{
		printk("%s: cur_sdio_func is empty!\n", __func__);
		return;
	}

	mmc_set_clock(cur_sdio_func->card->host, Hz);
}

#if 0
void trout_sdio_set_clock_old(unsigned int Hz)
{
    int retries = 0;
    printk("trtr:%s Hz[%d]\n",__func__,Hz);
retry:
    if(NULL==cur_sdio_func)
    {
        //if(retries<20)
        if(retries < 40)	//modify by chengwg 2013-05-14.
        {
            printk("trtr:%s retries[%d]\n",__func__,retries);
            msleep(50);
            retries++;
            goto retry;
        }
        else
        {
            printk("trout set clock retry fail %d", retries);
        }
    }
    else
    {
	mmc_set_clock(cur_sdio_func->card->host,Hz);// NOTICED!!! -->cur_sdio_func
    }
}
#endif


EXPORT_SYMBOL(trout_sdio_set_clock);

/* Export for Wifi/BT/FM  */
EXPORT_SYMBOL(Set_Trout_PowerOn);
EXPORT_SYMBOL(Set_Trout_PowerOff);
//EXPORT_SYMBOL(Set_Trout_Download_BT_Code);
EXPORT_SYMBOL(Set_Trout_RF_Start);
EXPORT_SYMBOL(Set_Trout_RF_Stop);
EXPORT_SYMBOL(MxdRfSetFreqWifiCh);//just for wifi
EXPORT_SYMBOL(MxdRfSetFreqHKhzFm);//just for FM
EXPORT_SYMBOL(MxdRfPulse); // just for cmcc aci test
EXPORT_SYMBOL(MxdRfSwtchLnaCfg);
EXPORT_SYMBOL(Set_trout_pcm_iis_mode);
EXPORT_SYMBOL(Get_trout_comm_int_stat);
EXPORT_SYMBOL(Set_trout_comm_int_clear);
EXPORT_SYMBOL(Set_trout_comm_int_mask);
EXPORT_SYMBOL(get_phy_mutex);
EXPORT_SYMBOL(put_phy_mutex);
EXPORT_SYMBOL(GetTxPowerLevel);
EXPORT_SYMBOL(get_tx_pwr_from_nv);
EXPORT_SYMBOL (MxdRfGetRfMode);


/* alloc buffer for wifi by zhao */
unsigned char *wifi_buf = NULL;
unsigned int wifi_buf_size = 0;

#define TROUT_WIFI_BUF_SIZE	0x87000  //540K for RX/TX(trout share ram 144kb)
#define TROUT_WIFI_MIN_BUF	0x10000	 //min 64K
/*for internal use only*/ 
unsigned int root_host_read_trout_reg(unsigned int reg_addr)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int ret = 0;

	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
        //xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
		if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
		      printk("%s: the trout module is unloading ,return fail\n",__func__);
		}
		return ret;	
	}

	/* convert register offset from 4 to 1. byte ->word */
	reg_addr = reg_addr >> 2;

	sdio_claim_host(func);

	sdio_memcpy_fromio(func, &ret, reg_addr, 4);
	sdio_release_host(func);

	return ret;
}
EXPORT_SYMBOL(root_host_read_trout_reg);

/*for internal use only*/ 
unsigned int root_host_write_trout_reg(unsigned int val, unsigned int reg_addr)
{
	struct sdio_func *func = cur_sdio_func;
	int err = 0;

	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
		//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
		if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
		      printk("%s: the trout module is unloading ,return fail\n",__func__);
		}
		return err;	
	}
	
	reg_addr = reg_addr >> 2;

	sdio_claim_host(func);
	//sdio_writel(func,val,reg_addr,&err);
	err = sdio_memcpy_toio(func, reg_addr, &val, 4);
	sdio_release_host(func);

	return err;
}
EXPORT_SYMBOL(root_host_write_trout_reg);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
#define rSYSREG_POWER_CTRL             ((0x0045<<2))

#define rSYSREG_HOST2ARM_INFO0         ((0x005A<<2))
#define rSYSREG_HOST2ARM_INFO1         ((0x005B<<2))
#define rSYSREG_HOST2ARM_INFO2         ((0x005C<<2))
#define rSYSREG_HOST2ARM_INFO3         ((0x005D<<2))
#define rSYSREG_INFO0_FROM_ARM         ((0x005E<<2))
#define rSYSREG_INFO1_FROM_ARM         ((0x005F<<2))
#define rSYSREG_INFO2_FROM_ARM         ((0x0060<<2))
#define rSYSREG_INFO3_FROM_ARM         ((0x0061<<2))

#define rSYSREG_GEN_ISR_2_ARM7         ((0x01FF<<2))

#define PS_MSG_HOST_HANDLE_TBTT_MAGIC   0xf542 // host want to handle TBTT INTR himself

#ifndef rMAC_PA_CON
#define rMAC_PA_CON                  0x00008004
#endif 

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);
bool is_wifi_in_sleep(void)
{
#if 0
	unsigned int value = root_host_read_trout_reg((unsigned int)rSYSREG_POWER_CTRL);

	if((value != 0x800500) && (value != 0x1700500))
		printk("warning: get wifi status failed(0x%x)\n", value);
	
	if((value == 0) || (value & (1<<23)) ||g_done_wifi_suspend)	//bit23, value equal to 0 mean's sdio error.
	{
		if(!( (value == 0) || (value & (1<<23)) )) printk("status isn't available\n");
		return true;
	}
	else
		return false;
#else
	return (g_done_wifi_suspend == 1);
#endif
}

void wifimac_state_set_awake(void)
{
	g_trout_state = TROUT_AWAKE;
}

EXPORT_SYMBOL(wifimac_state_set_awake);

void wifimac_state_set_sleep(void)
{
	g_trout_state = TROUT_SLEEP;
}

EXPORT_SYMBOL(wifimac_state_set_sleep);


//xuanyang, 2013-11-1, add
void wifimac_sleep_mutex_lock(unsigned int mode)
{
//	mode;
	mutex_lock(&g_wifi_sleep_mutex);
	printk("%s[%d] lock mode:%d\n", __FUNCTION__, __LINE__, mode);
}
EXPORT_SYMBOL(wifimac_sleep_mutex_lock);

void wifimac_sleep_mutex_unlock(void)
{
	printk("%s[%d] unlock\n", __FUNCTION__, __LINE__);
	mutex_unlock(&g_wifi_sleep_mutex);
}
EXPORT_SYMBOL(wifimac_sleep_mutex_unlock);


void wifimac_sleep(void)
{
	if(!(current_rf_mode & WIFI_MODE))
	{
		//xuanyang, 2013-11-1, add 
		wifimac_sleep_mutex_lock(current_rf_mode);
		host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
		msleep(20);
		host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
		mdelay(20);

		g_trout_state = TROUT_SLEEP;
		pr_info("wifimac_sleep\n");
		wifimac_sleep_mutex_unlock();
	}
}
EXPORT_SYMBOL(wifimac_sleep);

static void root_wifimac_wakeup(void)
{
    UWORD32 i = 5;
    UWORD32 count = root_host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) + 1; 

    printk("wakeup: rSYSREG_POWER_CTRL: %x\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
    /*pr_info("command %x\n", msg);*/

    root_host_write_trout_reg((UWORD32)PS_MSG_HOST_HANDLE_TBTT_MAGIC, (UWORD32)rSYSREG_HOST2ARM_INFO1);	/*load message*/
    root_host_write_trout_reg((UWORD32)0x1, (UWORD32)rSYSREG_GEN_ISR_2_ARM7);	/*interrupt CP*/
    /*pr_info("command done!\n");*/

    /*wait for CP*/
    do { 
        msleep(10);
        printk("rSYSREG_POWER_CTRL: %x\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
    } while((root_host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) != count) && i--);
	
	printk("after, rSYSREG_POWER_CTRL: %x\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
    root_host_write_trout_reg(0x0, (UWORD32)rSYSREG_HOST2ARM_INFO1);		/*clear message*/
	
	g_trout_state = TROUT_AWAKE;
	g_done_wifi_suspend = 0;
}

#endif

/*Host read trout reg interface*/ 
unsigned int host_read_trout_reg(unsigned int reg_addr)
{
	unsigned int ret = 0;

#if 0
	if(!cur_sdio_func)
		TROUT_PRINT_STACK;
#endif

	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	ret = root_host_read_trout_reg(reg_addr);
	#else
	if((reg_addr >= SYS_ADDR_MIN && reg_addr <= SYS_ADDR_MAX)
		|| (reg_addr >= FM_ADDR_MIN && reg_addr <= FM_ADDR_MAX)){//system register
		//call root funtion
		ret = root_host_read_trout_reg(reg_addr);
	}
	else
	{
		mutex_lock(&rw_reg_mutex);
		
		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
		if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
		{
			if(wake_low_power_fn != NULL)
			{
				printk("Warning: RDREG(0x%x) in low power mode, wakeup frist!\n", reg_addr);
				dump_stack();	//debug.
				mutex_unlock(&rw_reg_mutex);
				wake_low_power_fn(1);
				mutex_lock(&rw_reg_mutex);
			}
			else
			{
				printk("Err: RDREG(0x%x) in low power mode, couldn't wakeup!\n", reg_addr);
				BUG();
			}
		}
		else if(g_trout_state == TROUT_AWAKE)
		{ //STA_AWAKE or SOFT_AP  state
			// call root function
			if(is_wifi_in_sleep())  //add by chengwg, 2013.7.9
			{	
				printk("Warning: RDREG(0x%x) in wifi suspend time, so wake up wifi first!\n", reg_addr);
				dump_stack();
				
                root_wifimac_wakeup();
			}
		}
		else
		{
			if(trout_awake_fn != NULL)
			{
				trout_awake_fn(LOCK_FALSE);
			}
			else  //Hugh: add for power save when not loading wifi driver. 2014-04-25
			{
				root_wifimac_wakeup();
			}
			
			g_trout_state = TROUT_AWAKE;
		}
			ret = root_host_read_trout_reg(reg_addr);
		mutex_unlock(&rw_reg_mutex);
	}
	#endif
	
	return ret;

}
EXPORT_SYMBOL(host_read_trout_reg);

/*Host write trout reg interface*/ 
unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr)
{
	int err = 0;

#if 0
	if(!cur_sdio_func)
		TROUT_PRINT_STACK;
#endif
	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	err = root_host_write_trout_reg(val,reg_addr);
	#else
	if(reg_addr >= SYS_ADDR_MIN && reg_addr <= SYS_ADDR_MAX){//system register
		//call root funtion
		err = root_host_write_trout_reg(val,reg_addr);
	}
	else
	{
		mutex_lock(&rw_reg_mutex);
		
		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
		if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
		{
			if(wake_low_power_fn != NULL)
			{
				printk("Warning: WTREG(0x%x) in low power mode, wakeup frist!\n", reg_addr);
				dump_stack();	//debug.
				mutex_unlock(&rw_reg_mutex);
				wake_low_power_fn(1);
				mutex_lock(&rw_reg_mutex);
			}
			else
			{
				printk("Err: WTREG(0x%x) in low power mode, couldn't wakeup!\n", reg_addr);
				BUG();
			}
		}
		else if(g_trout_state == TROUT_AWAKE)
		{ //STA_AWAKE or SOFT_AP  state
			// call root function
			if(is_wifi_in_sleep())  //add by chengwg, 2013.7.9
			{
				printk("Warning: WTREG(0x%x) in wifi suspend time, so wake up wifi first!\n", reg_addr);
				dump_stack();
                root_wifimac_wakeup();
			}
		}
		else
		{
			if(trout_awake_fn != NULL)
			{
				trout_awake_fn(LOCK_FALSE);
			}
			else  //Hugh: add for power save. 2014-04-25
			{
				root_wifimac_wakeup();
			}
			
			g_trout_state = TROUT_AWAKE;
		}
		 	err = root_host_write_trout_reg(val,reg_addr);
		mutex_unlock(&rw_reg_mutex);
	}
	#endif	
	return err;
}
EXPORT_SYMBOL(host_write_trout_reg);

/*for internal use only*/ 
unsigned int root_host_read_trout_ram(void *host_addr, void *trout_addr,
				 unsigned int length)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int ret, word_len, rxsize;
	unsigned int ram_addr = (unsigned int)trout_addr;
	unsigned int remainder, max_blocks, blocks;
	unsigned int *buffer = (unsigned int *)host_addr;
	
	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
		//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
        if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
            printk("%s: the trout module is unloading ,return fail\n",__func__);
        }
		return 0;	
	}
	
	word_len = (length + 3) >> 2;
	ram_addr = (ram_addr >> 2) & 0xffff;
	ram_addr += TROUT_SHARE_MEM_BASE_ADDR;

	if (ram_addr > 0x1FEFF) {
		pr_debug("the addr out of range\n");
	}
        /*
	if(is_wifi_in_sleep())
	{
			printk("%s:wifi in sleep addr:%x\n",__FUNCTION__,trout_addr);
			//BUG_ON(1);
	}
        */

	max_blocks = 511;	//16;
	remainder = word_len * 4;	

//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);

	while (remainder > 512) {
		blocks = remainder / 512;

		if (blocks > max_blocks)
			blocks = max_blocks;

		rxsize = blocks * 512;

		ret = sdio_memcpy_fromio(func, buffer, ram_addr, rxsize);
		if (ret)
			goto out;
		remainder -= rxsize;

		buffer += rxsize / 4;	
		ram_addr += rxsize / 4;
	}

	if (remainder > 0) {
		ret = sdio_memcpy_fromio(func, buffer, ram_addr, remainder);
		if (ret)
			goto out;
	}

out:
	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);

	return ret;
}
EXPORT_SYMBOL(root_host_read_trout_ram);

/*Host read trout ram interface*/ 
unsigned int host_read_trout_ram(void *host_addr, void *trout_addr,
				 unsigned int length)
{
	unsigned int ret ;
	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	ret = root_host_read_trout_ram(host_addr,trout_addr,length);
	#else
	mutex_lock(&rw_reg_mutex);

		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
 	if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
	{
		if(wake_low_power_fn != NULL)
		{
			printk("Warning: RDRAM(0x%x) in low power mode, wakeup frist!\n", trout_addr);
			dump_stack();	//debug.
			mutex_unlock(&rw_reg_mutex);
			wake_low_power_fn(1);
			mutex_lock(&rw_reg_mutex);
		}
		else
		{
			printk("Err: RDRAM(0x%x) in low power mode, couldn't wakeup!\n", trout_addr);
			BUG();
		}
	}
	else if(g_trout_state == TROUT_AWAKE)
 	{ //STA_AWAKE or SOFT_AP  state
		// call root function
		if(is_wifi_in_sleep())  //add by chengwg, 2013.7.9  
		{
			printk("Warning: RDRAM(0x%x) in wifi suspend time, so wake up wifi first!\n", trout_addr);
			dump_stack();
            root_wifimac_wakeup();
		}
	}
	else
	{
 		if(trout_awake_fn != NULL)
		{
			trout_awake_fn(LOCK_FALSE);
		}
		else  //Hugh: add for power save. 2014-04-25
		{
			root_wifimac_wakeup();
		}
		
		g_trout_state = TROUT_AWAKE;
	}
		ret = root_host_read_trout_ram(host_addr,trout_addr,length);
	mutex_unlock(&rw_reg_mutex);
	#endif	
	return ret;
}
EXPORT_SYMBOL(host_read_trout_ram);

// Host通过SDIO写Trout内部RAM
unsigned int root_host_write_trout_ram(void *trout_addr, void *host_addr,
				  unsigned int length)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int ret, word_len, txsize;
	unsigned int ram_addr = (unsigned int)trout_addr;
	unsigned int remainder, max_blocks, blocks;
	unsigned int *buffer = (unsigned int *)host_addr;
	
	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
		//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
        if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
            printk("%s: the trout module is unloading ,return fail\n",__func__);
        }
		return 0;	
	}
	
	word_len = (length + 3) >> 2;
	ram_addr = (ram_addr >> 2) & 0xffff;
	ram_addr += TROUT_SHARE_MEM_BASE_ADDR;

	if (ram_addr > 0x1FEFF) {
		pr_debug("the addr out of range\n");
	}
        /*
	if(is_wifi_in_sleep())
	{
		printk("%s:wifi in sleep addr:%x\n",__FUNCTION__,trout_addr);
		//BUG_ON(1);
	}
        */

	max_blocks = 511;	//16;
	remainder = word_len * 4;	

//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);

	while (remainder > 512) {
		blocks = remainder / 512;

		if (blocks > max_blocks)
			blocks = max_blocks;

		txsize = blocks * 512;
		ret = sdio_memcpy_toio(func, ram_addr, buffer, txsize);
		if (ret)
			goto out;

		remainder -= txsize;

		buffer += txsize / 4;	
		ram_addr += txsize / 4;
	}

	if (remainder > 0) {
		ret = sdio_memcpy_toio(func, ram_addr, buffer, remainder);
		if (ret)
			goto out;
	}

out:
	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);

	return ret;
}
EXPORT_SYMBOL(root_host_write_trout_ram);

/*Host write trout ram interface*/ 
unsigned int host_write_trout_ram(void *trout_addr, void *host_addr,
				  unsigned int length)
{
	unsigned int ret;
	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	ret = root_host_write_trout_ram(trout_addr, host_addr, length);
	#else
	mutex_lock(&rw_reg_mutex);

		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
	if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
	{
		if(wake_low_power_fn != NULL)
		{
			printk("Warning: WTRAM(0x%x) in low power mode, wakeup frist!\n", trout_addr);
			dump_stack();	//debug.
			mutex_unlock(&rw_reg_mutex);
			wake_low_power_fn(1);
			mutex_lock(&rw_reg_mutex);
		}
		else
		{
			printk("Err: WTRAM(0x%x) in low power mode, couldn't wakeup!\n", trout_addr);
			BUG();
		}
	}
	else if(g_trout_state == TROUT_AWAKE)
	{ //STA_AWAKE or SOFT_AP  state
		// call root function
		if(is_wifi_in_sleep())  //add by chengwg, 2013.7.9
		{
			printk("Warning: WTRAM(0x%x) in wifi suspend time, so wake up wifi first!\n", trout_addr);
			dump_stack();
            root_wifimac_wakeup();
		}
	}
	else
	{
 		if(trout_awake_fn != NULL)
 		{
			trout_awake_fn(LOCK_FALSE);
		}
		else  //Hugh: add for power save. 2014-04-25
		{
			root_wifimac_wakeup();
		}

		g_trout_state = TROUT_AWAKE;
	}
		ret = root_host_write_trout_ram(trout_addr, host_addr, length);
	mutex_unlock(&rw_reg_mutex);
	#endif
	return ret;
}
EXPORT_SYMBOL(host_write_trout_ram);

unsigned int host_read_bt_ram(void *host_addr, void *bt_addr,
				 unsigned int length)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int ret, word_len, rxsize;
	unsigned int ram_addr = (unsigned int)bt_addr;
	unsigned int remainder, max_blocks, blocks;
	unsigned int *buffer = (unsigned int *)host_addr;

	word_len = (length + 3) >> 2;
	ram_addr = (ram_addr >> 2) & 0xffff;
	ram_addr += TROUT_BT_RAM_BASE_ADDR;

	if (ram_addr > 0x1FEFF) {
		pr_debug("the addr out of range\n");
	}

	max_blocks = 511;	//16;
	remainder = word_len * 4;	

//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);

	while (remainder > 512) {
		blocks = remainder / 512;

		if (blocks > max_blocks)
			blocks = max_blocks;

		rxsize = blocks * 512;

		ret = sdio_memcpy_fromio(func, buffer, ram_addr, rxsize);
		if (ret)
			goto out;
		remainder -= rxsize;

		buffer += rxsize / 4;	//转换为word地址
		ram_addr += rxsize / 4;
	}

	if (remainder > 0) {
		ret = sdio_memcpy_fromio(func, buffer, ram_addr, remainder);
		if (ret)
			goto out;
	}

out:
	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);

	return ret;
}
EXPORT_SYMBOL(host_read_bt_ram);

unsigned int host_write_bt_ram(void *bt_addr, void *host_addr,
				  unsigned int length)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int ret, word_len, txsize;
	unsigned int ram_addr = (unsigned int)bt_addr;
	unsigned int remainder, max_blocks, blocks;
	unsigned int *buffer = (unsigned int *)host_addr;

	word_len = (length + 3) >> 2;
	ram_addr = (ram_addr >> 2) & 0xffff;
	ram_addr += TROUT_BT_RAM_BASE_ADDR;

	if (ram_addr > 0x1FEFF) {
		pr_debug("the addr out of range\n");
	}

	max_blocks = 511;	//16;
	remainder = word_len * 4;	

//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);

	while (remainder > 512) {
		blocks = remainder / 512;

		if (blocks > max_blocks)
			blocks = max_blocks;

		txsize = blocks * 512;
		ret = sdio_memcpy_toio(func, ram_addr, buffer, txsize);
		if (ret)
			goto out;

		remainder -= txsize;

		buffer += txsize / 4;	
		ram_addr += txsize / 4;
	}

	if (remainder > 0) {
		ret = sdio_memcpy_toio(func, ram_addr, buffer, remainder);
		if (ret)
			goto out;
	}

out:
	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);

	return ret;
}
EXPORT_SYMBOL(host_write_bt_ram);

/*for internal use only*/ 
unsigned int root_host_block_read_trout_ram(unsigned int host_addr,
				       unsigned int trout_addr[],
				       unsigned int block_len[],
				       unsigned int block_num)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int *buffer, *save;
	int i, blocks = 0, total_len = 0;
	unsigned char *copydata;
	
	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
		//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
        if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
            printk("%s: the trout module is unloading ,return fail\n",__func__);
        }
		return 0;	
	}
	
	if (block_num > 16) {
		printk("the block num out of range\n");
		return 0;
	}
/*
	if( is_wifi_in_sleep())
	{
			printk("%s:wifi in sleep addr:%x\n",__FUNCTION__,trout_addr[0]);
			//BUG_ON(1);
	}
        */
	save = buffer =
	    (unsigned int *)kmalloc((block_num + 1) * 4, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	*buffer++ = block_num - 1;	
	for (i = 0; i < block_num; i++) {
		total_len += block_len[i];
		*buffer = TROUT_SHARE_MEM_BASE_ADDR;	
		*buffer += (trout_addr[i] >> 2) & 0xffff;

		if (*buffer > 0x1FEFF) {
			printk("the addr out of range\n");
		}

		*buffer += ((block_len[i] >> 2) - 1) << 17;
		buffer++;
	}

	if ((total_len > 512) && (total_len % 512 != 0)) {	//multi block

		blocks = total_len / 512 + 1;
		copydata = (unsigned char *)kmalloc(blocks * 512, GFP_KERNEL);
		if (!copydata)
		{
			kfree(save);
			return -ENOMEM;
		}
		memset(copydata, 0, blocks * 512);
	}
//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);
	//op code is 1,bruins
	sdio_memcpy_toio(func, 0x1FF00, save, (block_num + 1) * 4);
	if (blocks == 0) {
		sdio_memcpy_fromio(func, (unsigned int *)host_addr, 0x1FFFF,
				   total_len);
	} else {
		sdio_memcpy_fromio(func, copydata, 0x1FFFF, blocks * 512);	
		memcpy((unsigned char *)host_addr, copydata, total_len);
		kfree(copydata);
	}

	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);

	if(save != NULL)
	{
		kfree(save);
	}
	return 0;
}
EXPORT_SYMBOL(root_host_block_read_trout_ram);

/*Host block read trout ram interface*/ 
unsigned int host_block_read_trout_ram(unsigned int host_addr,
				       unsigned int trout_addr[],
				       unsigned int block_len[],
				       unsigned int block_num)
{
	unsigned int ret;
	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	ret = root_host_block_read_trout_ram(host_addr,trout_addr, block_len,block_num);
	#else	
	mutex_lock(&rw_reg_mutex);

		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
		if(g_trout_state == TROUT_AWAKE){ //STA_AWAKE or SOFT_AP  state
		// call root function
		ret = root_host_block_read_trout_ram(host_addr,trout_addr, block_len,block_num);
	}
	else{
		printk("++%s,trout_addr = %p,host_addr = %x\n",__FUNCTION__,trout_addr,host_addr);
		if(trout_awake_fn != NULL){
			trout_awake_fn(LOCK_FALSE);
		}
		else  //Hugh: add for power save. 2014-04-25
		{
			root_wifimac_wakeup();
		}
		
		g_trout_state = TROUT_AWAKE;
		//call root function
		ret = root_host_block_read_trout_ram(host_addr,trout_addr, block_len,block_num);
	}
	mutex_unlock(&rw_reg_mutex);
	#endif
	return ret;
}
EXPORT_SYMBOL(host_block_read_trout_ram);

/*for internal use only*/ 
unsigned int root_host_block_write_trout_ram(unsigned int trout_addr[],
					unsigned int host_addr,
					unsigned int block_len[],
					unsigned int block_num)
{
	struct sdio_func *func = cur_sdio_func;
	unsigned int *buffer, *save;
	int i, blocks = 0, total_len = 0;
	unsigned char *copydata;
	
	if(!func)
	{
		pr_info("%s: struct sdio_func is null!\n", __func__);
		//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
        if(check_trout_module_state(TROUT_MODULE_UNLOADING))  {
            printk("%s: the trout module is unloading ,return fail\n",__func__);
        }
		return 0;	
	}
	
	if (block_num > 16) {
		printk("the block num out of range\n");
		return 0;
	}
/*
		if( is_wifi_in_sleep())
	{
			printk("%s:wifi in sleep addr:%x\n",__FUNCTION__,trout_addr[0]);
			//BUG_ON(1);
	}
        */
	save = buffer =
	    (unsigned int *)kmalloc((block_num + 1) * 4, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	memset(buffer, 0, (block_num + 1) * 4);

	*buffer++ = block_num - 1;
	for (i = 0; i < block_num; i++) {
		total_len += block_len[i];
		*buffer = TROUT_SHARE_MEM_BASE_ADDR;	
		*buffer += (trout_addr[i] >> 2) & 0xffff;
		if (*buffer > 0x1FEFF) {
			printk("the addr out of range\n");
		}

		*buffer += ((block_len[i] >> 2) - 1) << 17;
		buffer++;
	}

	if ((total_len > 512) && (total_len % 512 != 0)) {	//multi block

		blocks = total_len / 512 + 1;
		copydata = (unsigned char *)kmalloc(blocks * 512, GFP_KERNEL);
		if (!copydata)
		{
			kfree(save);	//add by chengwg.
			return -ENOMEM;
		}
		memset(copydata, 0, blocks * 512);
		memcpy(copydata, (unsigned char *)host_addr, total_len);
		total_len = blocks * 512;
	}
//      mutex_lock(&private_data->sdio_mutex);
	sdio_claim_host(func);
	//op code is 1,bruins
	sdio_memcpy_toio(func, 0x1FF00, save, (block_num + 1) * 4);
	if (blocks == 0) {
		sdio_memcpy_toio(func, 0x1FFFF, (unsigned int *)host_addr,
				 total_len);
	} else {
		sdio_memcpy_toio(func, 0x1FFFF, copydata, total_len);	
		kfree(copydata);
	}

	sdio_release_host(func);
//      mutex_unlock(&private_data->sdio_mutex);
	if(save != NULL)
	{
		kfree(save);
	}
	return 0;

}
EXPORT_SYMBOL(root_host_block_write_trout_ram);

/*Host block write trout ram interface*/ 
unsigned int host_block_write_trout_ram(unsigned int trout_addr[],
					unsigned int host_addr,
					unsigned int block_len[],
					unsigned int block_num)
{
	unsigned int ret;
	#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE 
	ret = root_host_block_write_trout_ram(trout_addr,host_addr,block_len,block_num);
	#else
	mutex_lock(&rw_reg_mutex);

		//leon liu modified on 2013-4-3, since TROUT_SLEEP is added, we should
		//check if g_trout_state is TROUT_AWAKE
		//if(g_trout_state != TROUT_DOZE){ //STA_AWAKE or SOFT_AP  state
		if(g_trout_state == TROUT_AWAKE){ //STA_AWAKE or SOFT_AP  state
		// call root function
		ret = root_host_block_write_trout_ram(trout_addr,host_addr,block_len,block_num);
	}
	else{
		printk("++%s, trout_addr = %p,host_addr = %x\n",__FUNCTION__,trout_addr,host_addr);
		if(trout_awake_fn != NULL){
			trout_awake_fn(LOCK_FALSE);
		}
		else  //Hugh: add for power save. 2014-04-25
		{
			root_wifimac_wakeup();
		}
		g_trout_state = TROUT_AWAKE;
		//call root function
		ret = root_host_block_write_trout_ram(trout_addr,host_addr,block_len,block_num);
	}
	mutex_unlock(&rw_reg_mutex);
	#endif
	return ret;
}
EXPORT_SYMBOL(host_block_write_trout_ram);

/* interface for wifi driver to get alloced buffer by zhao*/
void get_wifi_buf(unsigned long *start, unsigned long *size)
{
	*start = (unsigned long)wifi_buf;
	*size = wifi_buf_size;
}
EXPORT_SYMBOL(get_wifi_buf);

static int trout_sdio_enable(struct sdio_func *func)
{
	int error;

	sdio_claim_host(func);
	error = sdio_set_block_size(func, TROUT_SDIO_BLOCK_SIZE);
	if (error) {
		dev_err(&func->dev, "failed to set block size to %u bytes,"
			" error %d\n", TROUT_SDIO_BLOCK_SIZE, error);
		goto out;
	}
	error = sdio_enable_func(func);
	if (error) {
		dev_err(&func->dev, "failed to enable func, error %d\n", error);
	}
out:
	sdio_release_host(func);
	return error;
}

static int trout_sdio_disable(struct sdio_func *func)
{
	int error;
	
	//exclusively claim the bus
	sdio_claim_host(func);
	error = sdio_disable_func(func);
	if (error) {
		dev_err(&func->dev, "failed to disable func, error %d\n", error);
	}
	//release bus
	sdio_release_host(func);
	
	return error;
}

#ifdef CONFIG_PM
static int trout_sdio_suspend(struct device *dev)
{
	int error;

	pr_info("NOTICE:##:sdio_suspend start********\n");
	
	if( POWER_OFF == g_trout_power_mode ) //Trout is in power down mode
	{
		printk("NOTICE:##:Trout2 power down:sdio_suspend start********\n");
		return 0;
	}

	if(!cur_sdio_func)
	{
		printk("%s: cur_sdio_func is null!\n", __func__);
		return 0;
	}

	//xuan.yang, 2013-10-16, if the trout module is unloading, the sdio can not suspend
    if(check_trout_module_state(TROUT_MODULE_UNLOADING)) 
    {
	      printk("%s: the trout module is unloading ,return fail\n",__func__);
	      return 1;
	}

	error = trout_sdio_disable(cur_sdio_func);
	cur_sdio_func = NULL;
	
	return error;
}
static int trout_sdio_resume(struct device *dev)
{
	int error;
	
	printk("NOTICE:##:sdio_resume start********\n");

	if( POWER_OFF == g_trout_power_mode ) //Trout is in power down mode
	{
		printk("NOTICE:##:Trout2 power down:sdio_suspend start********\n");
		return 0;
	}

	if(!trout_sdio_func)	//add by chengwg.
	{
		printk("%s: cur_sdio_func is null!\n", __func__);
		return 0;
	}
	/*if(sdio_can_sleep) {
		[>pr_info("real resume!\n");<]
		trout_sdio_can_sleep(0);
	}*/
	cur_sdio_func = trout_sdio_func;
	error = trout_sdio_enable(cur_sdio_func);

	return error;
}
#else
#define trout_sdio_suspend	NULL
#define trout_sdio_resume	NULL
#endif

/*leon liu added function trout_get_sdio_func to retrieve sdio_func*/
struct sdio_func *trout_get_sdio_func()
{
		return trout_sdio_func;
}
EXPORT_SYMBOL(trout_get_sdio_func);

static void read_ID(int time, char *msg)
{
	printk("%s:==== %s =====\n", __func__, msg);
	while(time--)
	{
	printk("ID: 0x%x\n", host_read_trout_reg(0x8038));
	}

}

static int trout_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	int error;


	trout_sdio_func = func;
	cur_sdio_func = trout_sdio_func;

	/*leon liu added, set clock to 12M before any operations later on*/
	mmc_set_clock(func->card->host, 12000000);

	pr_info("==============SDIO Device ID: vendor - 0x%x device - 0x%x\n", id->vendor, id->device);

	error = trout_sdio_enable(cur_sdio_func);
	if(!error)
	{
		/*host_write_trout_reg(0x700, (0x22<<2));*/
		host_write_trout_reg((unsigned int)POW_CTRL_VALUE, (unsigned int)(M_POW_CTRL<<2)); // ??  should move to public interface! 
		pr_info("enable trout auto control!\n");
	}
	else
	{	
		dev_err(&func->dev, "failed to enable func, error %d\n", error);
	}

	cur_sdio_func = NULL;
	return error;
}

static void trout_sdio_remove(struct sdio_func *func)
{
	trout_sdio_disable(func);
	cur_sdio_func = NULL;
	trout_sdio_func = NULL;
}

#define TROUT_VENDOR_ID 0x00
#define TROUT_DEVICE_ID  0x2260
static const struct sdio_device_id trout_sdio_ids[] = {
	{SDIO_DEVICE(TROUT_VENDOR_ID, TROUT_DEVICE_ID)},
	{},
};
//zhou add start
static const struct dev_pm_ops trout_sdio_pm_ops = {
	.suspend = trout_sdio_suspend,
	.resume	= trout_sdio_resume,
};

static struct sdio_driver trout_sdio_driver = {
	.probe = trout_sdio_probe,
	.remove = trout_sdio_remove,
	.name = "trout_sdio",
	.id_table = trout_sdio_ids,
	.drv = {
		.pm = &trout_sdio_pm_ops,
	},
};



#ifdef NPI_NV_CALIBRATION_ENABLE

void hex_dump(unsigned char *info, unsigned char *str, unsigned int len)
{
	unsigned int  i = 0;

    if(str == NULL || len == 0)
        return;

	printk("dump %s, len: %d; data:\n",info,len);
	for(i = 0; i<len; i++)
	{
		if(((unsigned char *)str+i)==NULL)
			break;
		printk("%02x ",*((unsigned char *)str+i));
		if((i+1)%16 == 0)
			printk("\n");
	}
	printk("\n");
}

//#define NPI_CAL_RF_CMD_PATH "/sys/kernel/trout_nv_cal"

extern int get_npi_nv_cal_struct(unsigned char *buf);
extern int set_npi_nv_cal_struct(unsigned char *buf, unsigned int buf_len);
extern int get_npi_nv_update_struct(unsigned char *buf);

/* calibration mode */
static ssize_t trout_nvcal_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int ret = 0;

	printk("%s finish.\n",__FUNCTION__);

    //ret = sprintf(buf, "trout npi nv calibration function is working.\n");
    ret = get_npi_nv_cal_struct(buf);
    
	return ret;
}

static ssize_t trout_nvcal_store(struct kobject *kobj, struct kobj_attribute *attr,
									const char *buf, size_t count)
{
    int ret = 0;
    
	printk("%s start buf count = %d\n", __FUNCTION__, count);
	hex_dump("nv struct ", buf, count);

    ret = set_npi_nv_cal_struct(buf, count);
    if(ret == -1)
    {
	    printk("%s finish. error occur!\n",__FUNCTION__);
	    return 0;
    }
    
	printk("%s finish.\n",__FUNCTION__);
	return count;
}


static ssize_t trout_nvupdate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int ret = 0;

    //ret = sprintf(buf, "trout npi nv calibration function is working.\n");
    ret = get_npi_nv_update_struct(buf);

	return ret;
}

static ssize_t trout_nvupdate_store(struct kobject *kobj, struct kobj_attribute *attr,
									const char *buf, size_t count)
{
    int ret = 0;
    
	printk("%s start, buf count = %d\n", __FUNCTION__, count);
	//hex_dump("nv struct ", buf, count);

    if(ret == -1)
    {
	    printk("%s finish. error occur!\n",__FUNCTION__);
	    return 0;
    }
    
	printk("%s do nothing.\n",__FUNCTION__);
	return count;
}


static struct kobj_attribute trout_nvcal_attribute =
	__ATTR(trout_nvcal_cmd, 0644, trout_nvcal_show, trout_nvcal_store);

static struct kobj_attribute trout_nvupdated_attribute =
	__ATTR(trout_nvupdate_cmd, 0644, trout_nvupdate_show, trout_nvupdate_store);


static struct attribute *attrs[] = {
	&trout_nvcal_attribute.attr,
	&trout_nvupdated_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *trout_nv_kobj;

int npi_nv_cal_init(void)
{
    int retval;
	
    /*
    * Create a npi kobject with the name of "trout_nv_cal",
    * located under /sys/kernel/
    */
    trout_nv_kobj = kobject_create_and_add("trout_nv_cal", kernel_kobj);
    if (!trout_nv_kobj)
        return -ENOMEM;

    /* Create the files associated with this kobject */
    retval = sysfs_create_group(trout_nv_kobj, &attr_group);
    if (retval)
        kobject_put(trout_nv_kobj);

    return retval;
}

void npi_nv_cal_cleanup(void)
{
	kobject_put(trout_nv_kobj);
}

#endif /* NPI_NV_CALIBRATION_ENABLE */


static int __init trout_sdio_init(void)
{
	int ret;
	unsigned int size = TROUT_WIFI_BUF_SIZE;
	char *ver_str = TROUT_SDIO_VERSION;

	pr_info("**********************************************\n");
	pr_info(" Trout SDIO driver \n");
	pr_info(" Version: %s\n", ver_str);
	pr_info(" Build date: %s %s\n", __DATE__, __TIME__);
	pr_info("**********************************************\n");

	interface_init();  //zhuyg modify 04-24
	
	ret = sdio_register_driver(&trout_sdio_driver);
	printk("%s: sdio register driver ret=%d\n", __func__, ret);

	/*leon liu added,check if sdio_register_driver is ok*/
	if (ret < 0){
		pr_err("Error: cannot register sdio driver for trout!\n");
		return ret;
	}

	Set_Trout2_Open();	//debug.
	

#ifdef  NPI_NV_CALIBRATION_ENABLE
    npi_nv_cal_init();
#endif

	/* alloc buffer for wifi driver  by zhao */
retry:
	wifi_buf = kmalloc(size, GFP_KERNEL);
	if(NULL == wifi_buf){
		if(size > TROUT_WIFI_MIN_BUF){
			size >>= 1;
			goto retry;
		}
	}
	if(wifi_buf)
		wifi_buf_size = size;
	else
		printk("ALTER: alloc %x buf for wifi failed\n", size);
	printk("SB BUF:%X, SIZE:%X\n", wifi_buf, wifi_buf_size);
	return ret;
}

static void __exit trout_sdio_exit(void)
{
	/* if alloced buffer for wifi, then free it by zhao */
	if(wifi_buf)
		kfree(wifi_buf);
	wifi_buf_size = 0;
	sdio_unregister_driver(&trout_sdio_driver);
    

#ifdef  NPI_NV_CALIBRATION_ENABLE
    npi_nv_cal_cleanup();
#endif
}

module_init(trout_sdio_init);
module_exit(trout_sdio_exit);

MODULE_DESCRIPTION("TROUT sdio driver");
MODULE_AUTHOR("Spreadtrum Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(TROUT_SDIO_VERSION);
