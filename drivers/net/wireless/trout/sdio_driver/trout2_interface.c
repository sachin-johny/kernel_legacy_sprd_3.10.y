/*
 * Trout2 public interface for SpreadTrum
 *
 * Copyright (C) 2013 SpreadTrum Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#if 1 /*For kernel 3.x*/
#include <mach/hardware.h>
#include <mach/adi.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#else
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>

#include <mach/mfp.h>   
#include <linux/io.h>
#include <mach/ldo.h>
#endif /*KERNEL 3.X*/
#include <mach/regulator.h>

#include <linux/spinlock.h>
#include <linux/mutex.h> // For power_mutex_lock
#include <linux/string.h>

#include "trout2_interface.h"
extern void MxdRfSetClkReqLow();
extern struct sdio_func *cur_sdio_func;
extern struct sdio_func *trout_sdio_func;

/*extern int sprd_3rdparty_gpio_wifi_pwd;
extern int sprd_3rdparty_gpio_wifi_reset;
extern int sprd_3rdparty_gpio_wifi_irq;*/
#if 1
extern int sdhci_device_attach(int on);
unsigned int sdhci_wifi_detect_isbusy(void);
#endif

//Globe variable for trout power state
unsigned int g_trout_power_mode = POWER_OFF; //when reset ,should 0
static bool g_is_BT_Code_Download = false;/* BT code */
unsigned int current_rf_mode = NONE_MODE;
struct mutex trout_power_mutex_lock;

//set trout chip driver  supply the 48M 
static void trout_sdio_pin_cfg(void);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
extern void wifimac_state_set_awake(void);
extern void wifimac_sleep(void);

#endif

extern void sprd_mfp_config(unsigned long *mfp_cfgs, int num);
extern void sdhci_bus_scan(void); //zhouxw,bus rescan
extern void trout_sdio_set_clock(unsigned long Hz);
unsigned int host_read_trout_reg(unsigned int reg_addr);
unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);

 /* move phy_mutex to here, can be used by other driver  by zhao */
static struct mutex phy_mutex;

//zhouxw
extern void write_dot11_phy_reg(unsigned char ra, unsigned int rd);
extern void read_dot11_phy_reg(unsigned char ra, unsigned int *rd);
//extern LDO_ID_E get_trout_2v8_ldoid(void);

/* export to other drivers by zhao */
void get_phy_mutex(void)
{
	mutex_lock(&phy_mutex);
}


/* export to other drivers by zhao */
void put_phy_mutex(void)
{
	mutex_unlock(&phy_mutex);
}

//Debug interfaces
static void read_ID(int time, char *msg)
{
		printk("%s:==== %s =====\n", __func__, msg);
		while(time--)
		{
				printk("ID: 0x%x\n", host_read_trout_reg(0x8038));
		}

}

/* :::::::::::::::::::::::::::START INTERFACE::::::::::::::::::::::::::::::::: */
void interface_init(void)
{
	mutex_init(&trout_power_mutex_lock);
	mutex_init(&phy_mutex);
	current_rf_mode=NONE_MODE;
	g_trout_power_mode = POWER_OFF;
}

//Power mode control interface 
static void Set_Power_Mode( unsigned int Power_Mode )
{
	printk("Power_Mode_ID: %d \n",Power_Mode);
	g_trout_power_mode |= Power_Mode;
}

static void  Clear_Power_Mode(unsigned int Power_Mode )
{
	printk("Power_Mode_ID: %d \n",Power_Mode);
	g_trout_power_mode &= ~Power_Mode;
}

//static unsigned int  Get_Power_Mode(void)
unsigned int Get_Power_Mode(void)	//modify by chwg, 2014.01.02.
{
	printk("Get Power Mode: %d \n",g_trout_power_mode);
	return g_trout_power_mode;
}
EXPORT_SYMBOL(Get_Power_Mode);

/***********************************************************************/
#if 1
static void Set_Function_Start(unsigned int mode)
{
	unsigned int reg_data;
	printk(" Set_Function_Start: %d \n",mode);
	if(mode== WIFI_MODE)
	{
		host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
	  	printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
//		host_write_trout_reg(WIFI_OPEN_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? msleep lihuai & mxd - need commet
	  	printk(" Set TROUT2_SYS_REG_HOST2ARM_INFO3: 1");
		/* Fixed ,AD/DA settings */
		host_write_trout_reg(0x34, TROUT2_SYS_REG_WIFIAD_CFG);  // WIFI ADC
		host_write_trout_reg((BW_DAC_FORCE_EN|WIFI_ADC_FORCE_EN), TROUT2_SYS_REG_ADDA_FORCE);  //0x6  DAC power 	//dumy 0416 //?? liyuan: provide default value
	}
	if(mode== FM_MODE)
	{
	  reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
	  /* Clear trout fm sleep flag to prevent trout get into sleep mode */
//		reg_data |= TROUT_FM_SLEEP;  
		reg_data &= ~TROUT_FM_SLEEP;  
	  host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
	  printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
	}
	if(mode== BT_MODE)
	{
	  reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
	  reg_data |= BT_CLK_DS_4;
	  reg_data &= ~BT_CLK_DS_7;
	  host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
	  printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
		host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
	}
}
static bool Set_Trout_Common_Start(unsigned int mode)
{
	bool bRet = true;
	switch(g_trout_power_mode)	
	{
		case NONE_MODE: //it the first open one mode 
		{
			Set_Function_Start(mode);
			break;
		}
		case WIFI_MODE:
		{
			if(WIFI_MODE == mode)
			{
				bRet = false;
			}
			else if(FM_MODE == mode)
			{
				Set_Function_Start(mode);
				printk("set WIFI and FM  open !!!\n");
			}
			else//BT_MODE
			{
				Set_Function_Start(mode);
				printk("set WIFI and BT  open !!!\n");
			}
			break;
		}
		case FM_MODE:
		{
			if(FM_MODE == mode)
			{
				bRet = false;
			}
			else if(WIFI_MODE == mode)
			{
				Set_Function_Start(mode);
				printk("set FM  and BT  open !!!\n");
			}
			else//BT_MODE 
			{
				Set_Function_Start(mode);
				printk("set FM and BT  open !!!\n");
			}
			break;
		}
		case BT_MODE:
		{
			if(BT_MODE == mode)
			{
				bRet = false;
			}
			else if(WIFI_MODE == mode)
			{
				Set_Function_Start(mode);
				printk("set BT  and WIFI  open !!!\n");
			}
			else//FM_MODE 
			{
				Set_Function_Start(mode);
				printk("set BT and FM  open !!!\n");
			}
			break;
		}
		case (WIFI_MODE |FM_MODE):
		{
			if(BT_MODE == mode )
			{
				Set_Function_Start(mode);
				printk("set WIFI and FM  open ,and then open BT !!!\n");
			}
			else
			{
				bRet  = false;
			}
			break;
		}
		case (WIFI_MODE |BT_MODE):
		{
			if(FM_MODE == mode )
			{
				Set_Function_Start(mode);
				printk("set WIFI and BT  open ,and then open FM !!!\n");
			}
			else
			{
				bRet  = false;
			}	
			break;
		}
		case (FM_MODE |BT_MODE ):
		{
			if(WIFI_MODE == mode )
			{
				Set_Function_Start(mode);
				printk("set FM and BT  open ,and then open WIFI !!!\n");
			}
			else
			{
				bRet  = false;
			}
			break;
		}
		default:
		{
			bRet = false;
			printk("set mode error !!!\n");
		}
	}
	return bRet;
}
static void Set_Function_Stop(unsigned int mode)
{
	unsigned int reg_data;
	printk(" Set_Function_Stop: %d \n",mode);
	if(mode == WIFI_MODE)//close WIFI PA and PHY 
	{
	  //host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
	  //msleep(50);
	  //host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
	  //mdelay(20);
//	  host_write_trout_reg(WIFI_CLOSE_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? 0?
	  printk(" TROUT2_SYS_REG_HOST2ARM_INFO3 set 0 \n");
	}
	
	if(mode == FM_MODE)
	{
	  reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
//		reg_data &= ~ TROUT_FM_SLEEP;
		reg_data |= TROUT_FM_SLEEP;  
	  host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
	  printk(" Set TROUT2_SYS_REG_CLK_CTRL1:%d (0x%x)\n",
	  	TROUT2_SYS_REG_CLK_CTRL1,reg_data);
	}
	if(mode == BT_MODE) //close BT clock 
	{
	  reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
	#if 0
	  reg_data &= ~BT_CLK_DS_4;
	  reg_data |= BT_CLK_DS_7;
	  host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
	#endif
	  printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
	  		 reg_data);
	}
}
static bool Set_Trout_Common_Stop(unsigned int mode)
{
	bool bRet = true;
	printk(" Set_Trout_Common_Stop: %d \n",mode);
	switch(g_trout_power_mode)
{
		case (WIFI_MODE | FM_MODE | BT_MODE):
		{
			Set_Function_Stop(mode);
			if(WIFI_MODE == mode)
			{
				printk(" all function opened and close the WIFI \n");
			}
			else if(FM_MODE == mode)
			{
				printk(" all function opened and close the FM \n");
			}
			else // BT_MODE
			{
				printk(" all function opened and close the BT \n");
			}
			break;
		}
		case (WIFI_MODE | FM_MODE):
		{
			if(WIFI_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the WIFI and FM opened ,and close the WIFI \n");
			}
			else if(FM_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the WIFI and FM opened ,and close the FM \n");
			}
			else //BT_MODE
			{
				bRet = false;
				printk(" the WIFI and FM opened ,but it stop BT ,error!!! \n");
			}
			break;
		}
		case (WIFI_MODE |BT_MODE):
		{
			if(WIFI_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the WIFI and BT opened ,and close the WIFI \n");
			}
			else if(BT_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the WIFI and BT opened ,and close the BT \n");
			}
			else //BT_MODE
			{
				bRet = false;
				printk(" the WIFI and BT opened ,but it stop FM ,error!!! \n");
			}
			break;
		}
		case (FM_MODE | BT_MODE):
		{
			if(FM_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the FM and BT opened ,and close the FM \n");
			}
			else if(BT_MODE == mode)
			{
				Set_Function_Stop(mode);
				printk(" the FM and BT opened ,and close the BT \n");
			}
			else //BT_MODE
			{
				bRet = false;
				printk(" the FM and BT opened ,but it stop WIFI ,error!!! \n");
			}
			break;
		}
		case WIFI_MODE:
		case FM_MODE:
		case BT_MODE:
		{
			Set_Function_Stop(mode);
			break;
		}
		default: //NONE_MODE
		{
			bRet = false;
			printk(" all close, error!!! \n");
		}

	}
	return bRet;
 }

#else
static bool Set_Trout_Common_Start(unsigned int mode)
{
	unsigned int reg_data;

	printk("Current mode: %d, Set_Trout_Common_Start: %d \n",Get_Power_Mode(),mode);
		
	switch( Get_Power_Mode() )
	{

		case NONE_MODE:
		{
			if(WIFI_MODE == mode)	
			{
				//host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				// add system set 0x174  :1 enable   0 disable
				host_write_trout_reg(WIFI_OPEN_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? msleep lihuai & mxd - need commet
				printk(" Set TROUT2_SYS_REG_HOST2ARM_INFO3: 1");
				/* Fixed ,AD/DA settings */
				//host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
				//host_write_trout_reg(0x34, TROUT2_SYS_REG_WIFIAD_CFG);  // WIFI ADC
				host_write_trout_reg((BW_DAC_FORCE_EN|WIFI_ADC_FORCE_EN), TROUT2_SYS_REG_ADDA_FORCE);  //0x6  DAC power 	//dumy 0416 //?? liyuan: provide default value
			}	
			else if(FM_MODE== mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				//reg_data |= TROUT_FM_CLK_8; //?? liyuan: should be 0, bit10 need set 
				reg_data |= TROUT_FM_SLEEP;  
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
			}
			else if (BT_MODE == mode)	
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data |= BT_CLK_DS_4;
				reg_data &= ~BT_CLK_DS_7;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				//BT log enable
				host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
			}
			else 
			{
				return false;
			}
		}
		break;
		
		case WIFI_MODE:
		{
			if(FM_MODE == mode)	
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				//reg_data |= TROUT_FM_CLK_8; //?? liyuan: should be 0, bit10 need set 
				reg_data |= TROUT_FM_SLEEP;  
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
			}
			else if (BT_MODE == mode)	
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data |= BT_CLK_DS_4;
				reg_data &= ~BT_CLK_DS_7;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				//BT log enable
				host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
			}
			else 
			{
				return false;
			}
		}
		break;
		
		case FM_MODE:
		{
			if(WIFI_MODE == mode)
			{
				//host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				// add system set 0x174  :1 enable   0 disable
				host_write_trout_reg(WIFI_OPEN_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? msleep lihuai & mxd - need commet
				printk(" Set TROUT2_SYS_REG_HOST2ARM_INFO3: 1");
				/* Fixed ,AD/DA settings */
				//host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
				//host_write_trout_reg(0x34, TROUT2_SYS_REG_WIFIAD_CFG);  // WIFI ADC
				host_write_trout_reg((BW_DAC_FORCE_EN|WIFI_ADC_FORCE_EN), TROUT2_SYS_REG_ADDA_FORCE);  //0x6  DAC power 	//dumy 0416 //?? liyuan: provide default value
			}	
			else if (BT_MODE == mode)	
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data |= BT_CLK_DS_4;
				reg_data &= ~BT_CLK_DS_7;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				//BT log enable
				host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
			}
			else
			{
				return false;
			}
		}
		break;
		
		case BT_MODE:
		{
			if(WIFI_MODE == mode)
			{
				//host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				// add system set 0x174  :1 enable   0 disable
				host_write_trout_reg(WIFI_OPEN_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? msleep lihuai & mxd - need commet
				printk(" Set TROUT2_SYS_REG_HOST2ARM_INFO3: 1");
				/* Fixed ,AD/DA settings */
				//host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
				//host_write_trout_reg(0x34, TROUT2_SYS_REG_WIFIAD_CFG);  // WIFI ADC
				host_write_trout_reg((BW_DAC_FORCE_EN|WIFI_ADC_FORCE_EN), TROUT2_SYS_REG_ADDA_FORCE);  //0x6  DAC power 	//dumy 0416 //?? liyuan: provide default value

			}	
			else if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				//reg_data |= TROUT_FM_CLK_8; //?? liyuan: should be 0, bit10 need set 
				reg_data |= TROUT_FM_SLEEP;  
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
			}
			else
			{
				return false;
			}
		}
		break;
		
		case (WIFI_MODE|FM_MODE):
		{
			if(BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data |= BT_CLK_DS_4;
				reg_data &= ~BT_CLK_DS_7;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				//BT log enable
				host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
			}	
			else 
			{
				return false;
			}

		}
		break;
		
		case (WIFI_MODE|BT_MODE):
		{
			if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data |= BT_CLK_DS_4;
				reg_data &= ~BT_CLK_DS_7;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);

				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				//BT log enable
				host_write_trout_reg((unsigned int)ENABLE_DSP_LOG, TROUT2_SYS_REG_GPIO);
			}	
			else 
			{
				return false;
			}

		}
		break;
		
		case (FM_MODE|BT_MODE):
		{
			if(WIFI_MODE == mode)
			{
				//host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d \n",TROUT2_SYS_REG_CLK_CTRL1);
				// add system set 0x174  :1 enable   0 disable
				host_write_trout_reg(WIFI_OPEN_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? msleep lihuai & mxd - need commet
				printk(" Set TROUT2_SYS_REG_HOST2ARM_INFO3: 1");
				/* Fixed ,AD/DA settings */
				//host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
				//host_write_trout_reg(0x34, TROUT2_SYS_REG_WIFIAD_CFG);  // WIFI ADC
				host_write_trout_reg((BW_DAC_FORCE_EN|WIFI_ADC_FORCE_EN), TROUT2_SYS_REG_ADDA_FORCE);  //0x6  DAC power 	//dumy 0416 //?? liyuan: provide default value
			}	
			else
			{
				return false;
			}
		}
		break;
		
		default: return false;
	}
	
	return true;
}

static bool Set_Trout_Common_Stop(unsigned int mode)
{
	unsigned int reg_data;
	
	printk("Current mode: %d, Set_Trout_Common_Start: %d \n",Get_Power_Mode(),mode);

	switch( Get_Power_Mode() )
	{
		case (WIFI_MODE|BT_MODE|FM_MODE):
		{
			if(WIFI_MODE == mode)
			{
			  //host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
			  //msleep(50);
			  //host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
			  //mdelay(20);
			  //notiy arm7 wifi stop
			  host_write_trout_reg(WIFI_CLOSE_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? 0?
			  printk(" TROUT2_SYS_REG_HOST2ARM_INFO3 set 0 \n");
			}	
			else if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~ TROUT_FM_SLEEP;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
					
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1:%d (0x%x)\n",
				TROUT2_SYS_REG_CLK_CTRL1,reg_data);
			}
			else if (BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~BT_CLK_DS_4;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
				
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
						 reg_data);
			}
			else 
			{
				return false;
			}
		}
		break;
		
		case WIFI_MODE:
		{
			if(WIFI_MODE == mode)
			{
			  //host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
			  //msleep(50);
			  //host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
			  //mdelay(20);
			  //notiy arm7 wifi stop
			  host_write_trout_reg(WIFI_CLOSE_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? 0?
			  printk(" TROUT2_SYS_REG_HOST2ARM_INFO3 set 0 \n");
			}
			else
			{
				return false;
			}
		}
		break;
		
		case FM_MODE:
		{
			if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~ TROUT_FM_SLEEP;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
					
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1:%d (0x%x)\n",
				TROUT2_SYS_REG_CLK_CTRL1,reg_data);
			}	
			else 
			{
				return false;
			}
		}
		break;
		
		case BT_MODE:
		{
			if(BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~BT_CLK_DS_4;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
				
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
						 reg_data);
			}	
			else 
			{
				return false;
			}
		}
		break;
		
		case (WIFI_MODE|FM_MODE):
		{
			if(BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~BT_CLK_DS_4;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
				
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
						 reg_data);
			}
			else if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~ TROUT_FM_SLEEP;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
					
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1:%d (0x%x)\n",
				TROUT2_SYS_REG_CLK_CTRL1,reg_data);
			}
			else 
			{
				return false;
			}
		}
		break;
		
		case (WIFI_MODE|BT_MODE):
		{
			if(WIFI_MODE == mode)
			{
			  //host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
			  //msleep(50);
			  //host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
			  //mdelay(20);
			  //notiy arm7 wifi stop
			  host_write_trout_reg(WIFI_CLOSE_FLAG,TROUT2_SYS_REG_HOST2ARM_INFO3); //dumy 0416 //?? 0?
			  printk(" TROUT2_SYS_REG_HOST2ARM_INFO3 set 0 \n");
			}
			else if(BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~BT_CLK_DS_4;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
				
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
						 reg_data);
			}
			else 
			{
				return false;
			}
		}
		break;
		
		case (FM_MODE|BT_MODE):
		{
			if(FM_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~ TROUT_FM_SLEEP;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
					
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1:%d (0x%x)\n",
				TROUT2_SYS_REG_CLK_CTRL1,reg_data);
			}
			else if(BT_MODE == mode)
			{
				reg_data = host_read_trout_reg(TROUT2_SYS_REG_CLK_CTRL1);
				reg_data &= ~BT_CLK_DS_4;
				host_write_trout_reg(reg_data,TROUT2_SYS_REG_CLK_CTRL1);
				
				printk(" Set TROUT2_SYS_REG_CLK_CTRL1: %d(0x%x) \n",TROUT2_SYS_REG_CLK_CTRL1,
						 reg_data);
			}
			else 
			{
				return false;
			}
		}	
		break;
			
		default: return false;
	}
	
	return true;
}
#endif
/***********************************************************************/
extern void castorLcPllctrl(int on);
bool Set_Trout_RF_Start(unsigned int mode)
{
	//mutex_lock(&g_TroutDevices.rf_mutex_lock);
	//mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
	switch(mode) {
		case FM_MODE:
			if(!((current_rf_mode & BT_MODE) ||(current_rf_mode & WIFI_MODE) ))
					{
					trout_sdio_set_clock(12000000);
					castorLcPllctrl(0);
					printk("liwk-start fm,and bw not run, slow sdio and close pll\n");
					}
			if(current_rf_mode & FM_MODE) {
				printk("INIT_MODE_FM have been already set\n");
			} else {
				printk("Set the mode INIT_MODE_FM to RF\n");
				MxdRfSetFmMode(RF_ENABLE);     //real set
				current_rf_mode |= FM_MODE;
			}
			break;
		case BT_MODE:
			if((current_rf_mode & FM_MODE) &&(0==(current_rf_mode & WIFI_MODE) ))
					{
					castorLcPllctrl(1);
					#ifdef CONFIG_ARCH_SC8825
					trout_sdio_set_clock(45000000);
					#else
					trout_sdio_set_clock(48000000);
					#endif
					printk("liwk-start bt,and wifi not run,fm run,increase sdio and open pll\n");
					}			
			if(current_rf_mode & BT_MODE) {
				printk("INIT_MODE_BT have been already set\n");
			} else {
				
				printk("Set the mode INIT_MODE_BT to RF\n");
				MxdRfSetBtMode(RF_ENABLE);     //real set

				if((current_rf_mode == FM_MODE)||(current_rf_mode == NONE_MODE))
				{
				#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
					wifimac_sleep();//hugh:add for power 20130425
				#endif
				}
				current_rf_mode |= BT_MODE;
			}
			break;
		case WIFI_MODE:
			if((current_rf_mode & FM_MODE) &&(0==(current_rf_mode & BT_MODE) ))
			{				
				castorLcPllctrl(1);
				#ifdef CONFIG_ARCH_SC8825
				trout_sdio_set_clock(45000000);
				#else
				trout_sdio_set_clock(48000000);
				#endif
				printk("liwk-start wifi,and bt not run,fm run,increase sdio and open pll\n");
			}	
			if(current_rf_mode & WIFI_MODE) {
				printk("INIT_MODE_WIFI have been already set\n");
			} else {
				printk("Set the mode INIT_MODE_WIFI to RF\n");
				MxdRfSetWifiMode(RF_ENABLE);    //real set
				current_rf_mode |= WIFI_MODE;
			}
			break;
		case NONE_MODE:
			current_rf_mode &= ~FM_MODE;
			current_rf_mode &= ~BT_MODE;
			current_rf_mode &= ~WIFI_MODE;
			printk("Clear the mode with RF");
			break;
		default:
			printk("Wrong control type\n");
			//mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
			return false;
	}
	//mutex_unlock(&g_TroutDevices.rf_mutex_lock);
	//mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
	return true;
}

/*
 * Stop the RF with FM/BT/WIFI
 * @func: Set_Trout_RF_Stop
 * @control: TROUT_CONTRL_T(NONE_MODE/BT_MODE/WIFI_MODE/FM_MODE)
 * @return: true for start sucessfully
 */
 
bool Set_Trout_RF_Stop(unsigned int mode)
{
	//mutex_lock(&g_TroutDevices.rf_mutex_lock);
	//mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
	switch(mode) {
		case FM_MODE:
			if(!(current_rf_mode & FM_MODE)) {
				printk("INIT_MODE_FM have been already stop\n");
			} else {
			
				printk("Set the mode INIT_MODE_FM to RF\n");
				MxdRfSetFmMode(0);    //real set
				
				if(!(current_rf_mode & WIFI_MODE))
				{
				#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
					wifimac_sleep();//hugh:add for power 20130425
				#endif
				}
				current_rf_mode &= ~FM_MODE;
			}
			break;
		case BT_MODE:
			if(!(current_rf_mode & BT_MODE)) {
				printk("INIT_MODE_BT have been already stop\n");
			} else {
			
				printk("Set the mode INIT_MODE_BT to RF\n");
				MxdRfSetBtMode(0);     //real set
				if(!(current_rf_mode & WIFI_MODE))
				{
				#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
					wifimac_sleep();//hugh:add for power 20130425
				#endif
				}
				current_rf_mode &= ~BT_MODE;

			}
			if((current_rf_mode & FM_MODE) &&(!(current_rf_mode & WIFI_MODE) ))
					{
					trout_sdio_set_clock(12000000);
					castorLcPllctrl(0);
					printk("liwk-stop bt,wifi not run but fm run, slow sdio and close pll\n");
					}			
			break;
		case WIFI_MODE:
			if(!(current_rf_mode & WIFI_MODE)) {
				printk("INIT_MODE_WIFI have been already stop\n");
			} else {
				current_rf_mode &= ~WIFI_MODE;
				printk("Set the mode INIT_MODE_WIFI to RF\n");
				MxdRfSetWifiMode(0);   //real set
			}
			if((current_rf_mode & FM_MODE) &&(!(current_rf_mode & BT_MODE) ))
			{
				trout_sdio_set_clock(12000000);
				castorLcPllctrl(0);
				printk("liwk-stop wifi,bt not run but fm run, slow sdio and close pll\n");
			}
			break;
		case NONE_MODE:
			current_rf_mode &= ~FM_MODE;
			current_rf_mode &= ~BT_MODE;
			current_rf_mode &= ~WIFI_MODE;
			printk("Clear the mode with RF");
			MxdRfSetFmMode(0); 
			MxdRfSetBtMode(0);
			MxdRfSetWifiMode(0);
			break;
		default:
			printk("Wrong control type\n");
			//mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
			return false;
	}
	//mutex_unlock(&g_TroutDevices.rf_mutex_lock);
	//mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
	return true;
}

//set trout chip driver  supply the 48M 
static void trout_sdio_pin_cfg(void)
{
	unsigned int  reg_data;	

	reg_data = host_read_trout_reg(TROUT2_SYS_REG_SD_CMD);
	reg_data &= ~(TROUT_PIN_DS_MSK);
	reg_data |= TROUT_PIN_DS_3;
	host_write_trout_reg(reg_data,TROUT2_SYS_REG_SD_CMD);

	reg_data = host_read_trout_reg(TROUT2_SYS_REG_SD_D0);
	reg_data &= ~(TROUT_PIN_DS_MSK);
	reg_data |= TROUT_PIN_DS_3;
	host_write_trout_reg( reg_data,TROUT2_SYS_REG_SD_D0);

	reg_data = host_read_trout_reg(TROUT2_SYS_REG_SD_D1);
	reg_data &= ~(TROUT_PIN_DS_MSK);
	reg_data |= TROUT_PIN_DS_3;
	host_write_trout_reg( reg_data,TROUT2_SYS_REG_SD_D1);

	reg_data = host_read_trout_reg(TROUT2_SYS_REG_SD_D2);
	reg_data &= ~(TROUT_PIN_DS_MSK);
	reg_data |= TROUT_PIN_DS_3;
	host_write_trout_reg(reg_data,TROUT2_SYS_REG_SD_D2);

	reg_data = host_read_trout_reg(TROUT2_SYS_REG_SD_D3);
	reg_data &= ~(TROUT_PIN_DS_MSK);
	reg_data |= TROUT_PIN_DS_3;
	host_write_trout_reg(reg_data,TROUT2_SYS_REG_SD_D3);

	reg_data = 0xA0;
	host_write_trout_reg(reg_data,TROUT2_SYS_U0CTS_REG);
	reg_data = 0x120;
	host_write_trout_reg(reg_data,TROUT2_SYS_U0RTS_REG);

}

//set sdio mode 
static  void trout_set_sdiomode(bool mode)
{
#if 1
		SPRD_GPIO_REG_WRITEL((BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_WPU | BIT_PIN_SLP_WPU | BIT_PIN_SLP_OE), REG_PIN_SD1_D3);
		gpio_request(PIN_SD1_D3, "hello");
		gpio_direction_output(PIN_SD1_D3, 1); ///24 
		if(mode==SDIO_MODE)
				__gpio_set_value(PIN_SD1_D3,1);
		else if(mode==SPI_MODE)
				__gpio_set_value(PIN_SD1_D3,0);	

#else
		REG32(PIN_SD2_D3_REG)=(1<<8)|(0<<7)|(0<<6)|(3<<4)|(0<<2)|(0<<0); //zhouxw
		gpio_request(PIN_SD2_D3, "hello");
		gpio_direction_output(PIN_SD2_D3, 1); ///24 
		if(mode==SDIO_MODE)
				__gpio_set_value(PIN_SD2_D3,1);
		else if(mode==SPI_MODE)
				__gpio_set_value(PIN_SD2_D3,0);	

#endif /*For android 4.x*/
	printk("trout_sd3_SET!\n");
	//mdelay(10);	  
}

#if 1
/*TODO: consider ARCH_SC8825*/
//lone liu added for trout_chip_reset
#define WIFI_D3_PIN			PIN_SD1_D3
#define WIFI_RESET_PIN		TROUT2_RESET
#define WIFI_RESET_PIN_REG	REG_PIN_CLK_REQ2
//trout_chip_reset is used to RESET trout and initialize trout's interface as SDIO mode
static void trout_chip_reset(void)
{
	int ret;
	unsigned long value_gpio_reset = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_SLP_OE);
	unsigned long value_gpio_wifi_d3 = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_WPU | BIT_PIN_SLP_WPU | BIT_PIN_SLP_OE);
	unsigned long value_gpio_sd3 = (BITS_PIN_DS(1) | BITS_PIN_AF(0) | BIT_PIN_WPU);
	

	printk("%s: Trout's PINs: SD_D3: %d RESET: %d\n", __func__, WIFI_D3_PIN, WIFI_RESET_PIN);
	//Config WIFI_D3_PIN as gpio
	SPRD_GPIO_REG_WRITEL(value_gpio_wifi_d3, REG_PIN_SD1_D3);

	ret = gpio_request(WIFI_D3_PIN, "hello");

	if (ret)
	{
			pr_err("Requesting for GPIO function of PIN SD1_D3 failed!!!!");
	}

	gpio_direction_output(WIFI_D3_PIN, 1);
	mdelay(1);

	printk("WIFI_D3 val: %d\n", gpio_get_value(WIFI_D3_PIN));
	/*Config pin function to gpio for Trout Reset Pin*/
	SPRD_GPIO_REG_WRITEL(value_gpio_reset, WIFI_RESET_PIN_REG);
	ret = gpio_request(WIFI_RESET_PIN, "trout_reset");

	if (ret)
	{
			pr_err("Requesting for GPIO83(SD1_D3)failed!!!!");
	}
	gpio_direction_output(WIFI_RESET_PIN, 1);
	gpio_set_value(WIFI_RESET_PIN, 0);
	mdelay(100);
	gpio_set_value(WIFI_RESET_PIN, 1);
	

	gpio_free(WIFI_D3_PIN);
	gpio_free(WIFI_RESET_PIN);
	/*Config pin function to SD1_D3*/
	SPRD_GPIO_REG_WRITEL(value_gpio_sd3, REG_PIN_SD1_D3);
	mdelay(1);	
	
}
#endif /*CONFIG_ARCH_SC7710*/

//RSTN-----SC6820 GPIO140
static void trout_reset_low(void)
{
		mdelay(10);
		gpio_request(TROUT2_RESET, "trout_reset");
		gpio_direction_output(TROUT2_RESET, 0);
		printk("trout_gpio_reset_low!\n");
		gpio_free(TROUT2_RESET);   
}

#ifdef NPI_NV_CALIBRATION_ENABLE
static unsigned int g_rf_pwr_on_flag = 0;
#endif

#if 1
// Provide the 2.8v voltage.
#define TROUT_WIFI_XTL_EN_PIN_REG	REG_PIN_IIS0MCK
#define TROUT_WIFI_PIN_CLKREQ		214
static  void trout_rf_power_on(void)
{

#ifdef CONFIG_ARCH_SC8825
	pr_info("%s: Configuring pin BT/WIFI_CLKREQ to function normally\n", __func__);
	/*Config pin function as BT/WIFI_CLKREQ*/
	SPRD_GPIO_REG_WRITEL(CEQ_open_cfg, TROUT_WIFI_XTL_EN_PIN_REG);
#else
		struct regulator *rf_regulator = NULL;

		SPRD_GPIO_REG_WRITEL(CEQ_open_cfg, TROUT_WIFI_XTL_EN_PIN_REG);

		rf_regulator = regulator_get(NULL, "VDDRF1");

		if (rf_regulator == NULL)
		{
				printk("Cannot get regulator for rf\n");
				return ;
		}

		regulator_set_voltage(rf_regulator, 2850000, 2850000);

		regulator_set_mode(rf_regulator, REGULATOR_MODE_STANDBY);
		regulator_enable(rf_regulator);

		regulator_put(rf_regulator);
#endif

#ifdef NPI_NV_CALIBRATION_ENABLE
	g_rf_pwr_on_flag = 1;
#endif

}


// disable  the 2.8v voltage.
static  void trout_rf_power_off(void)
{	
#ifdef CONFIG_ARCH_SC8825
	pr_info("%s: Configuring pin BT/WIFI_CLKREQ as GPIO pin\n", __func__);
	SPRD_GPIO_REG_WRITEL(CEQ_close_cfg, TROUT_WIFI_XTL_EN_PIN_REG);
	gpio_set_value(TROUT_WIFI_PIN_CLKREQ, 0);
#else
		struct regulator *rf_regulator = NULL;

		SPRD_GPIO_REG_WRITEL(CEQ_close_cfg, TROUT_WIFI_XTL_EN_PIN_REG);

		rf_regulator = regulator_get(NULL, "VDDRF1");

		if (rf_regulator == NULL)
		{
				printk("Cannot get regulator for rf\n");
				return ;
		}

		regulator_disable(rf_regulator);

		regulator_put(rf_regulator);
#endif

#ifdef NPI_NV_CALIBRATION_ENABLE
	g_rf_pwr_on_flag = 0;
#endif
}

#else  /*ifdef CONFIG_ARCH_SC7710*/
// Provide the 2.8v voltage.
static  void trout_rf_power_on(void)
{
    unsigned int vctl0;
    
#ifdef TROUT_BOARD_EVB
	sprd_mfp_config(CEQ_open_cfg, ARRAY_SIZE(CEQ_open_cfg)); 

    vctl0 = ANA_REG_GET(ANA_LDO_VCTL4);
    vctl0 &= ~(0xf<<4);
    vctl0 |= (0xA<<4);  //b1b0 = 0 2.85V
    ANA_REG_SET(ANA_LDO_VCTL4, vctl0);

    ANA_REG_AND(ANA_LDO_PD_CTL1, (~(1<<4)));
    ANA_REG_OR(ANA_LDO_PD_CTL1, (1<<5));  //power on wifi1

	ANA_REG_OR(ANA_LDO_SLP0, (1<<0));
	ANA_REG_OR(ANA_LDO_SLP0, (1<<1));    //open xlten control
#elif defined TROUT_BOARD_MOBILE           //for mobile power on/off,use wifi0
	sprd_mfp_config(CEQ_open_cfg, ARRAY_SIZE(CEQ_open_cfg)); 
    LDO_SetVoltLevel(get_trout_2v8_ldoid(), LDO_VOLT_LEVEL_FAULT_MAX);
    LDO_TurnOnLDO(get_trout_2v8_ldoid());
#endif

#ifdef NPI_NV_CALIBRATION_ENABLE
	g_rf_pwr_on_flag = 1;
#endif
}

// disable  the 2.8v voltage.
static  void trout_rf_power_off(void)
{
#ifdef TROUT_BOARD_EVB
    ANA_REG_OR(ANA_LDO_PD_CTL1, (1<<4));
    ANA_REG_AND(ANA_LDO_PD_CTL1, (~(1<<5)));  //power off WIFI1
    //ANA_REG_AND(ANA_LDO_SLP0,(~(1<<0)));
	//ANA_REG_AND(ANA_LDO_SLP0,(~(1<<1)));     //close xlen control
	sprd_mfp_config(CEQ_close_cfg, ARRAY_SIZE(CEQ_close_cfg)); 
#elif defined TROUT_BOARD_MOBILE
	sprd_mfp_config(CEQ_close_cfg, ARRAY_SIZE(CEQ_close_cfg)); 
	//ANA_REG_AND(ANA_LDO_SLP0,(~(1<<0)));
	//ANA_REG_AND(ANA_LDO_SLP0,(~(1<<1)));     //close xlen control
    LDO_TurnOffLDO(get_trout_2v8_ldoid());
#endif

#ifdef NPI_NV_CALIBRATION_ENABLE
	g_rf_pwr_on_flag = 0;
#endif
}

#endif 
//PDN -----SC6820 GPIO137
static void trout_power_on(void)
{
		/* Power on trout */
		gpio_free(TROUT2_PDN);
		gpio_request(TROUT2_PDN, "trout_power"); 
		gpio_direction_output(TROUT2_PDN, 1); 
		printk("trout_gpio_power_on!\n");
		gpio_free(TROUT2_PDN);  
#if 1
#ifdef CONFIG_WLAN_SDIO
		sdhci_device_attach(1);
#endif
#endif
}

//RST  SC6820 GPIO140
static void trout_power_off(void)
{
		/* Power on trout */
		gpio_free(TROUT2_PDN);
		gpio_request(TROUT2_PDN, "trout_power");  //zhouxw //dumy 0416
		__gpio_set_value(TROUT2_PDN,0); //PDN  H-->L //dumy 0416
		//__gpio_set_value(TROUT2_RESET,0); //RST  H-->L //dumy 0416
		gpio_free(TROUT2_PDN); //dumy 0416
		//gpio_free(TROUT2_RESET); //dumy 0416
		printk("trout_gpio_power_off!\n");
#if 1
#ifdef CONFIG_WLAN_SDIO
		sdhci_device_attach(0);
#endif
#endif

}

static int wlan_ldo_enable(void)
{
		int err;
		struct regulator *wlan_regulator_18;

		wlan_regulator_18 = regulator_get(NULL, REGU_NAME_SDHOST1);

		if (IS_ERR(wlan_regulator_18)) {
				pr_err("can't get wlan 1.8V regulator\n");
				return -1;
		}

		err = regulator_set_voltage(wlan_regulator_18,1800000,1800000);
		if (err){
				pr_err("can't set wlan to 1.8V.\n");
				return -1;
		}
		
		regulator_set_mode(wlan_regulator_18, REGULATOR_MODE_STANDBY);
		regulator_enable(wlan_regulator_18);
		regulator_put(wlan_regulator_18);
}

static void trout_reset(void)
{
    int ret;
    unsigned long value_gpio_wifi_d3 = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_WPU | BIT_PIN_SLP_WPU | BIT_PIN_SLP_OE);
    unsigned long value_gpio_sd3 = (BITS_PIN_DS(1) | BITS_PIN_AF(0) | BIT_PIN_WPU);

		wlan_ldo_enable();
		
    //Config WIFI_D3_PIN as gpio
    SPRD_GPIO_REG_WRITEL(value_gpio_wifi_d3, REG_PIN_SD2_D3);

    ret = gpio_request(WIFI_D3_PIN, "hello");

    if (ret)
    {
        pr_err("Requesting for GPIO function of PIN SD1_D3 failed!!!!");
    }

    gpio_direction_output(WIFI_D3_PIN, 1);
    mdelay(1);

    printk("WIFI_D3 val: %d\n", gpio_get_value(WIFI_D3_PIN));

		gpio_request(TROUT2_RESET, "trout_reset");
		gpio_direction_output(TROUT2_RESET, 1);
		gpio_set_value(TROUT2_RESET,0);
		mdelay(100);
		gpio_set_value(TROUT2_RESET,1);
		printk("trout_gpio_reset!\n");

    SPRD_GPIO_REG_WRITEL(value_gpio_sd3, REG_PIN_SD2_D3);

    mdelay(1);
}

//enable 32KHz time clock.
#if 1
/*leon liu modified trout_rtc_on/off for sc7710*/
static  void  trout_rtc_on(void)
{
		struct clk *wlan_clk;

		wlan_clk	=	clk_get(NULL, "clk_aux0");
		if (IS_ERR(wlan_clk)){
				printk("clock: fail to get clk_aux0\n");
		}
		clk_set_rate(wlan_clk, 32000);
		clk_enable(wlan_clk);
		clk_put(wlan_clk);

}

static  void  trout_rtc_off(void)
{
		struct clk *wlan_clk;

		wlan_clk	=	clk_get(NULL, "clk_aux0");
		if (IS_ERR(wlan_clk)){
				printk("clock: fail to get clk_aux0\n");
		}
		clk_disable(wlan_clk);
		clk_put(wlan_clk);
}

#else /*!CONFIG_ARCH_SC7710*/
static  void  trout_rtc_on(void)
{
	unsigned int temp;

	temp = REG32(GR_PLL_SRC);
	temp &= ~(3 << 10);
	temp |= 2 << 10;
	REG32(GR_PLL_SRC)	= temp;
	temp = REG32(GR_GEN1);
	temp |=  GEN1_CLK_AUX0_EN;
	temp &= ~0xff;
	REG32(GR_GEN1) = temp;
}

//32KHz time clock close.
static  void  trout_rtc_off(void)
{
//	REG32(GR_GEN1) &= ~GEN1_CLK_AUX0_EN;
}
#endif /*CONFIG_ARCH_SC7710*/

/* ::::::::::::::::::::::::::::::::::::::::Add function */

unsigned int Get_trout_comm_int_stat(void)
{
	unsigned int_stat =0;
	
	mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
	int_stat=host_read_trout_reg(TROUT2_COM_REG_INT_STAT);
	mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
	return int_stat;
}
void Set_trout_comm_int_mask(unsigned int bit)
{
        unsigned int_mask=0;
	
		mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
        int_mask = host_read_trout_reg(TROUT2_COM_REG_INT_MASK);
        int_mask=int_mask|bit;
        host_write_trout_reg(int_mask,TROUT2_COM_REG_INT_MASK);
		mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
}
void Set_trout_comm_int_clear(unsigned int bit)
{
    unsigned int_clear =0;

	mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
    int_clear=host_read_trout_reg(TROUT2_COM_REG_INT_CLEAR);
    int_clear=int_clear|bit;
    host_write_trout_reg(int_clear,TROUT2_COM_REG_INT_CLEAR);
	mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
}

void Set_trout_pcm_iis_mode(bool mode)
{
   unsigned int  reg_data;	
   mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
   if(mode==PCM_MODE) //PCM  mode  
   {
		printk("zhouxw:set PCM \n");
	       reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISDO);
		reg_data &= ~(TROUT_IIS_DS_0|TROUT_IIS_DS_1|TROUT_IIS_DS_2);	
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISDO);
		reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISCLK);
		reg_data &= ~(TROUT_IIS_DS_0|TROUT_IIS_DS_1|TROUT_IIS_DS_2);
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISCLK);
		reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISLRCK);
		reg_data &= ~(TROUT_IIS_DS_0|TROUT_IIS_DS_1|TROUT_IIS_DS_2);
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISLRCK);
    }
  
  	else if(mode==IIS_MODE)  
    {
        printk("zhouxw:set iis \n");
		reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISDO);
		reg_data |= TROUT_IIS_DS_0;
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISDO);
		reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISCLK);
		reg_data |= TROUT_IIS_DS_0;
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISCLK);
		reg_data = host_read_trout_reg(TROUT2_SYS_REG_IISLRCK);
		reg_data |= TROUT_IIS_DS_0;
		host_write_trout_reg( reg_data,TROUT2_SYS_REG_IISLRCK);
    }
}

// It close all clocks and WIFI PA  and PHY
static void trout2_com_reg_init(void)
{
	host_write_trout_reg(TROUT_RF_ADI_EN, TROUT2_COM_REG_RF_SPI_CTRL); //dumy 0416 ??????
	printk(" Set TROUT2_COM_REG_RF_SPI_CTRL: 0x%x(0x13) \n",TROUT2_COM_REG_RF_SPI_CTRL);
        host_write_trout_reg((unsigned int)TROUT_CONFIG_CLK_CTRL0,TROUT2_SYS_REG_CLK_CTRL0); 
	host_write_trout_reg((unsigned int)TROUT_CLK_CONFG_DEFAUT,TROUT2_SYS_REG_CLK_CTRL1);   
	host_write_trout_reg((PMU_PD_INT_AUTO|LDO_ANA_PD_AUTO), TROUT2_SYS_REG_POW_CTRL); 
#if 0
	host_write_trout_reg(host_read_trout_reg((unsigned int)rMAC_PA_CON ) | (PA_DS_0 | PA_DS_25),(unsigned int)rMAC_PA_CON );
	msleep(50);
	host_write_trout_reg(host_read_trout_reg( (unsigned int)rMAC_PA_CON ) & (~(PA_DS_0 | PA_DS_25)),(unsigned int)rMAC_PA_CON );
	mdelay(10);
#endif
	//host_write_trout_reg(0x250, TROUT2_SYS_REG_ANT_SEL_CFG);  //zhouxw 
	host_write_trout_reg((unsigned int)TROUT_WIFI_RF_SEL, TROUT2_SYS_REG_RF_SEL_REG); 
}

static void set_phy_reg(void)
{
	unsigned int rd=0;
	read_dot11_phy_reg(0x1D, &rd);
	printk("zhouxw,before read 0x1D is %x\n",rd);

	write_dot11_phy_reg(0xff,  0x0);
	write_dot11_phy_reg(0x1D, 0x01);
	write_dot11_phy_reg(0x1E, 0x01);
	//write_dot11_phy_reg(0x08, 0);
	write_dot11_phy_reg(0x5e, 0x31);

	read_dot11_phy_reg(0x1D, &rd);
	printk("zhouxw,after read 0x1D is %x\n",rd);
	read_dot11_phy_reg(0x5E, &rd);
	printk("zhouxw,after read 0x5E is %x\n",rd);
}


#ifdef NPI_NV_CALIBRATION_ENABLE
typedef enum
{
    CAL_REGS_NOT_CONFIG = 0,
    CAL_REGS_NORMAL,
    CAL_REGS_NPI_CAL, // PC calibration mode
    CAL_REGS_UNKNOWN
}CAL_REGS_TYPE_T;

CAL_REGS_TYPE_T cal_regs_type = CAL_REGS_NOT_CONFIG;

#define NV_CAL_FINGERMARK 0xFF01
#define NV_VERSION 4

#define NOT_UPDATED 0x5A		// need to equal to the same definition in eng_rf_nv_config.h
#define ALREADY_UPDATED 0xA5   	// need to equal to the same definition in eng_rf_nv_config.h

ADJUST_REG_T cal_regs = {0};
ADJUST_REG_T *p_cal_regs = NULL;

// 0 no, 1 need self-cal, 2 cal done
typedef enum
{
	NOT_READY = 0x29,
	SELFCAL_DEFAULT = 0x30,
	SELFCAL_WAITING = 0x31,
	SELFCAL_DONE    = 0x32,
}SELFCAL_TYPE;
SELFCAL_TYPE g_self_calibration_flag = NOT_READY;

typedef unsigned int uint32;

/* nv struct for driver */
typedef struct nv_adjust_reg_t
{
	uint32 nv_ver;
	uint32 cal_flag;
	uint32 updated_nv_flag; // use for rf self calibration
	uint32 antenna_switch;
	uint32 pa_id;
	uint32 lna_id;
    uint32 wifi_imb[2];
	uint32 wifi_LOFT[4];
	uint32 DAC_scale_chnnl[14];
	uint32 PA_table[32];
	uint32 tx_gain[2];
	uint32 PAD_table[32];
	uint32 BT_LOFT[4];
	uint32 Pout_self_check[18];
	uint32 reserved[8];
	// use for rf self calibration
	uint32 btImb[2];
	uint32 btPaTbl[6];
	uint32 btPadTbl[6];
	uint32 wifiDcoc[64];
	uint32 btDcoc[32];
	uint32 reserved2[50]; //0806 add
} NV_ADJUST_REG_T;

static NV_ADJUST_REG_T nv_aj_info = {0};

unsigned int antenna_switch = 0;
int set_trout_antenna_switch(void)
{
    	if(antenna_switch == 0)
   	{
      		printk("%s: error! antenna_switch = 0, not right config!\n", __FUNCTION__);
      	 	return -1;
   	}
    
    	printk("%s: set sysreg 0x58 from %#x to %#x\n", __FUNCTION__, host_read_trout_reg((unsigned int)TROUT2_SYS_REG_ANT_SEL_CFG), antenna_switch);
    	host_write_trout_reg((unsigned int)antenna_switch, (unsigned int)TROUT2_SYS_REG_ANT_SEL_CFG); 
    	printk("%s: reread sysreg 0x58 = %#x\n",__FUNCTION__, host_read_trout_reg((unsigned int)TROUT2_SYS_REG_ANT_SEL_CFG));	
	return 0;
}

int get_npi_nv_cal_struct(unsigned char *buf)
{
	unsigned char *tmp_buf = buf;
	unsigned int asize = 0;
    int len = 0;
	int i = 0;
    
    len += sprintf(tmp_buf, "-----------npi cal struct-----------\n");
    tmp_buf = buf + len;
	
    len += sprintf(tmp_buf, "updatedNvFlag = %#x\n", cal_regs.updatedNvFlag);
    tmp_buf = buf + len;

    len += sprintf(tmp_buf, "len = %d\n", cal_regs.len);
    tmp_buf = buf + len;
    len += sprintf(tmp_buf, "nvVer = %#x\n", cal_regs.nvVer);
    tmp_buf = buf + len;
    len += sprintf(tmp_buf, "paId = %#x\n", cal_regs.paId);
    tmp_buf = buf + len;
    len += sprintf(tmp_buf, "lnaId = %#x\n", cal_regs.lnaId);
    tmp_buf = buf + len;
    len += sprintf(tmp_buf, "wifiImb = {%#x,%#x}\n", 
                cal_regs.wifiImb[0], cal_regs.wifiImb[1]);
    tmp_buf = buf + len;
	
    len += sprintf(tmp_buf, "wifiLoft = {%#x,%#x,%#x,%#x}\n", 
                cal_regs.wifiLoft[0], cal_regs.wifiLoft[1],
                cal_regs.wifiLoft[2], cal_regs.wifiLoft[3]);
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "dacScaleCh = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.dacScaleCh)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.dacScaleCh[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

    len += sprintf(tmp_buf, "wifiPaTbl = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.wifiPaTbl)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.wifiPaTbl[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "wifiTxGain = {%#x,%#x}\n", 
                cal_regs.wifiTxGain[0], cal_regs.wifiTxGain[1]);
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "wifiPadTbl = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.wifiPadTbl)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.wifiPadTbl[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "btLoft = {%#x,%#x,%#x,%#x}\n", 
            cal_regs.btLoft[0], cal_regs.btLoft[1],
            cal_regs.btLoft[2], cal_regs.btLoft[3]);
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "poutSelfChk = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.poutSelfChk)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.poutSelfChk[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;
    
    len += sprintf(tmp_buf, "reserved = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.reserved)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.reserved[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

	/* zhenlin.liu new add, 0709 */
    len += sprintf(tmp_buf, "btImb = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.btImb)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.btImb[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

    len += sprintf(tmp_buf, "btPaTbl = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.btPaTbl)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.btPaTbl[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;
	
    len += sprintf(tmp_buf, "btPadTbl = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.btPadTbl)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.btPadTbl[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;
	
    len += sprintf(tmp_buf, "wifiDcoc = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.wifiDcoc)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.wifiDcoc[i] & 0xFFFF);
        tmp_buf = buf + len;
	
        len += sprintf(tmp_buf, "%#x, ", (cal_regs.wifiDcoc[i] >> 16) & 0xFFFF);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

    len += sprintf(tmp_buf, "btDcoc = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.btDcoc)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.btDcoc[i] & 0xFFFF);
        tmp_buf = buf + len;
	
        len += sprintf(tmp_buf, "%#x, ", (cal_regs.btDcoc[i] >> 16) & 0xFFFF);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

    len += sprintf(tmp_buf, "reserved2 = { ");
    tmp_buf = buf + len;
	asize = sizeof(cal_regs.reserved2)/sizeof(unsigned int);
    for(i=0; i<asize; ++i)
    {
        len += sprintf(tmp_buf, "%#x, ", cal_regs.reserved2[i]);
        tmp_buf = buf + len;
    }
    len += sprintf(tmp_buf, "}\n");
    tmp_buf = buf + len;

    printk("%s: the npi cal struct info(len=%d): \n[%s]\n", __FUNCTION__, 
		len, buf);
	
    printk("\n%s: antenna_switch = %#x\n", __FUNCTION__, antenna_switch);
	
    return len;
}

int set_npi_nv_cal_struct(unsigned char *buf, unsigned int buf_len)
{
    NV_ADJUST_REG_T *p_nv_regs = &nv_aj_info;

    if(buf == NULL)
    {
        printk("%s: buff null, error! \n", __FUNCTION__);
        return -1;
    }

    memset((void *)&cal_regs, 0, sizeof(cal_regs));	
    memcpy(p_nv_regs, buf, sizeof(NV_ADJUST_REG_T));

    if(p_nv_regs->nv_ver == NV_VERSION)
    {
        if(sizeof(NV_ADJUST_REG_T) != buf_len)
        {
            printk("%s: buflen error! buf_len=%d, sizeof(NV_ADJUST_REG_T)=%d\n", 
                __FUNCTION__, buf_len, sizeof(NV_ADJUST_REG_T));
            return -1;
        }

	antenna_switch = p_nv_regs->antenna_switch;
	cal_regs.updatedNvFlag = p_nv_regs->updated_nv_flag;
	g_self_calibration_flag = (ALREADY_UPDATED == cal_regs.updatedNvFlag) ? SELFCAL_DEFAULT : SELFCAL_WAITING;
		
        cal_regs.len = sizeof(ADJUST_REG_T);
        cal_regs.nvVer = p_nv_regs->nv_ver;

        /* copy remaining items, skip 3 members: flag, len, nv_ver */
        memcpy((unsigned char *)(&(cal_regs.paId)), 
                (unsigned char *)(&(p_nv_regs->pa_id)), 
                sizeof(ADJUST_REG_T)-3*sizeof(unsigned int));
        
        p_cal_regs = &cal_regs;
    }
    else
    {
        printk("%s: version=%d, error! Current only support version %d!\n", 
            __FUNCTION__,p_nv_regs->nv_ver,NV_VERSION);
         return -1;
    }
    return 0;
}

//get the 802.11 a/b/g/n tx power form nv itmes
int GetTxPowerLevel(unsigned int* gnTxPower,
						unsigned int* bTxPower)
{
	unsigned int rd=0;
	int nRet = 0; 

#ifdef NPI_NV_CALIBRATION_ENABLE  
	/* config tx gain */
	if(p_cal_regs)
	{
		if(p_cal_regs->wifiTxGain[0]<= 0x3F)
		{
			*gnTxPower = p_cal_regs->wifiTxGain[0];
		}
		else
		{
			nRet = -1; // 0x70 nv invalid 
		}

		if(p_cal_regs->wifiTxGain[1] <= 0x3F)
		{
			*bTxPower = p_cal_regs->wifiTxGain[1];
		}
		else
		{
			nRet = -2; // 0x71 nv invalid 
		}
	}
	else
	{
		nRet = -3; // it don't get the value from the nv item
	}
#else
	nRet = -3;

#endif

	return nRet;

}

extern void hex_dump(unsigned char *info, unsigned char *str, unsigned int len);
int get_npi_nv_update_struct(unsigned char *buf)
{
    NV_ADJUST_REG_T *p_nv_regs = &nv_aj_info;

	unsigned char *tmp_buf = buf;
	unsigned int asize = 0;
	int len = 0;
	int i = 0;

	*tmp_buf = g_self_calibration_flag;
	tmp_buf++;
	len++;
	
	if(SELFCAL_DONE == g_self_calibration_flag)
	{
		p_nv_regs->updated_nv_flag = cal_regs.updatedNvFlag;

		//asize = sizeof(NV_ADJUST_REG_T)-5*sizeof(unsigned int);
		asize = ((unsigned char *)(&(p_nv_regs->wifi_imb))-(unsigned char *)p_nv_regs);
		asize = sizeof(NV_ADJUST_REG_T) - asize;
		
        /* copy remaining items, skip 5 members: flag, len, nv_ver, pa_id, lna_id */
        memcpy((unsigned char *)(&(p_nv_regs->wifi_imb)), 
                (unsigned char *)(&(cal_regs.wifiImb)), 
                asize);

		memcpy(tmp_buf, (void *)p_nv_regs, sizeof(NV_ADJUST_REG_T));
		len += sizeof(NV_ADJUST_REG_T);

		//hex_dump("output buf", buf, len);
		
    	//printk("%s: len=%d\n",__FUNCTION__,len);
	}
	
	
    return len;
}

#endif

#define NV_TX_PWR_11B_START 0
#define NV_TX_PWR_11G_START 4
#define NV_TX_PWR_11N_START 8
//get the 802.11 a/b/g/n tx power form nv itmes
int get_tx_pwr_from_nv(unsigned int *b_tx_pwr, unsigned int *g_tx_pwr, unsigned int *n_tx_pwr, unsigned int* delta_0x57){
    if(NULL != p_cal_regs){       
            	b_tx_pwr[0] = p_cal_regs->reserved2[NV_TX_PWR_11B_START];    // get 802.11b tx power
            	b_tx_pwr[1] = p_cal_regs->reserved2[NV_TX_PWR_11B_START+1];  
		b_tx_pwr[2] = p_cal_regs->reserved2[NV_TX_PWR_11B_START+2];
		b_tx_pwr[3] = p_cal_regs->reserved2[NV_TX_PWR_11B_START+3];

	  	g_tx_pwr[0] = p_cal_regs->reserved2[NV_TX_PWR_11G_START];    // get 802.11g tx power
            	g_tx_pwr[1] = p_cal_regs->reserved2[NV_TX_PWR_11G_START+1];  
		g_tx_pwr[2] = p_cal_regs->reserved2[NV_TX_PWR_11G_START+2];
		g_tx_pwr[3] = p_cal_regs->reserved2[NV_TX_PWR_11G_START+3];
		
           	n_tx_pwr[0] = p_cal_regs->reserved2[NV_TX_PWR_11N_START];    // get 802.11n tx power
            	n_tx_pwr[1] = p_cal_regs->reserved2[NV_TX_PWR_11N_START+1];  
		n_tx_pwr[2] = p_cal_regs->reserved2[NV_TX_PWR_11N_START+2];
		n_tx_pwr[3] = p_cal_regs->reserved2[NV_TX_PWR_11N_START+3];
		*delta_0x57 = p_cal_regs->reserved[4];
    }
    else{
        printk("Error: [%s] couldn't get values from nv\n", __FUNCTION__);
        return -1;  // couldn't get values from nv
    }
    return 0;
}
#define TIMEOUT_CNT 3
static int trout_common_init(void)
{
	int i = 0;
	//interface_init();  //zhuy modify
	trout_sdio_set_clock(12000000);
	trout_sdio_pin_cfg();
	castorLcPllInit(); //wait PLL stable and set highe speed clock	
	#ifdef CONFIG_ARCH_SC8825
	trout_sdio_set_clock(45000000);
	#else
	trout_sdio_set_clock(48000000);
	#endif    

    	set_phy_reg();    //zhouxw
   	mdelay(20);
	trout2_com_reg_init();	
	return 0;
}

 /***************************************************
*   RSTN-----SC6820 GPIO140
*   PDN -----SC6820 GPIO137
*
*   32K_IN 1.8V -----always on
*   
*   26M_IN 2.8V------ off when sleep 
*   CLK_REQ--SC6820 BTXLEN
****************************************************/
static int Set_Power_Control(bool on)
{
	static int scan_times = 0;
	static int find_trout_times = 0;
	int retry_count = 0;
	int i = 0;

	if ( on == true ) //power on 
	{
retry:
		trout_rf_power_on();	//Provide the 2.8v voltage.
		trout_power_on();	//Set PND pin high voltage.
		trout_reset();       	//tReset trout2
		trout_rtc_on();		//Provide 32KHz time clock.

		trout_sdio_func = NULL;
		cur_sdio_func = NULL;
		printk("%s: start sdhci bus scan(%d: %d)\n", __func__, scan_times++, find_trout_times);

		sdhci_bus_scan();		//detect mmc
		for (i = 0; i <= 200; i++) {
			if(!sdhci_wifi_detect_isbusy())
				break;
			msleep(100);
		}
		//yangke, 2013-08-27: retry five times, merge from 6820 baseline
		if(retry_count++ <5)
		{
			if(i > 200)
			{
				printk("trout scan failed, retry %d...\n", retry_count);
				goto retry;
			}
		}
		else
			goto err;

		if(!trout_sdio_func)
			goto err;
									
		find_trout_times++;
		cur_sdio_func = trout_sdio_func;	//modify by chengwg.
		if(-1 == trout_common_init()){
			goto err;
		}
		//read_ID(3, "After trout_common_init()\n");
	}
	else if( on == false ) //power off
	{
#ifdef TROUT_PDN_ENABLE
		#ifndef CONFIG_ARCH_SC8825
		//Hugh,2013-08-20: delete PDN for sc8825 external ldo control.
			trout_power_off(); //Set PND pin low voltage.
		#endif
		trout_rtc_off();// disable 32KHz time clock.
		trout_rf_power_off();//disable the 2.8v voltage.
		#ifndef CONFIG_ARCH_SC8825
		//Hugh,2013-08-20: delete RSTN for sc8825 external ldo control.
			trout_reset_low();//Set RSTN pin low voltage.
		#endif
		/*leon liu added, close SDIO clock for trout 2013-10-17*/
		//trout_sdio_set_clock(0);
		cur_sdio_func = NULL;	//add by chengwg.
#endif
		//sdhci_bus_scan();   
	}
	
	return 0;

err:
#ifdef TROUT_PDN_ENABLE
	printk("Failed to set power to trout...\n");
	trout_power_off(); //Set PND pin low voltage.		
	trout_rtc_off();// disable 32KHz time clock.
	trout_rf_power_off();//disable the 2.8v voltage.
	trout_reset_low();//Set RSTN pin low voltage.
	cur_sdio_func = NULL;	//add by chengwg.
#endif
	return -1;
}

void Set_Trout2_Open(void)
{
#ifdef TROUT_PDN_ENABLE
	printk("Set_Trout2_Open...\n");
	//call Power On function;
	if(Set_Power_Control(true) != 0)
		return;
	/* Down bt code for power sleep function */
	/*leon liu masked power sleep enable macro for downloading BT code*/
       /* #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE*/
    	Set_Trout_Download_BT_Code();
       /* #endif*/

	//Hugh: make wifi mac to sleep.2013-04-24
	#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
	wifimac_sleep();
	#endif
	
	MxdRfSetClkReqLow();
	Set_Power_Control(false);
	g_is_BT_Code_Download = false;
#else
	//call Power On function;
	Set_Power_Control(true);

	/* Down bt code for power sleep function */
	/*leon liu masked power sleep enable macro for downloading BT code*/
       /* #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE*/
	printk("Set_Trout_Download_BT_Code down...\n");
    	Set_Trout_Download_BT_Code();
	/*#endif*/
	
	//Hugh: make wifi mac to sleep.2013-04-24
	#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
	wifimac_sleep();
	#endif
#endif
}

//Power on wireless connection module
bool Set_Trout_PowerOn( unsigned int  MODE_ID )
{
	int i = 0;
	if((MODE_ID != WIFI_MODE) && (MODE_ID != FM_MODE) && (MODE_ID != BT_MODE))
	{
		printk("ATTENTION!!! Bad Mode ID.\n");
		return false;
	}
	
	mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex
	#ifdef TROUT_PDN_ENABLE
	printk("\n===========Power On==========\n");
	printk("zhuyg g_trout_power_mode is: %u\n", g_trout_power_mode);
	if( POWER_OFF == g_trout_power_mode )
	{		
		printk("Set Trout Power on for %d \n",MODE_ID);
		
		//Hugh: set wifimac state to be awake. 2013-04-25
		#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
		wifimac_state_set_awake();
		#endif
		
		//call Power On function;
		if(Set_Power_Control(true) != 0)
		{
			mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
			return false;
		}

		    
#ifdef NPI_NV_CALIBRATION_ENABLE  
		while(1){
			if(0 == set_trout_antenna_switch()){		
				MxdRfInit(&cal_regs);
				if(g_self_calibration_flag == SELFCAL_WAITING)
				{
					printk("%s: need self calibration!\n", __func__);
		
					if(cal_regs.updatedNvFlag == ALREADY_UPDATED)
					{
						printk("%s: need update nv data!\n", __func__);
						
						// self calibration driver changed nv
						cal_regs.reserved[0] = NV_CAL_FINGERMARK; 
						
						// save to nv
						// convert it and wait be got in sys cmd file
						g_self_calibration_flag = SELFCAL_DONE;
					}
				}
				break;
			}
			else{
				i++;
				if(TIMEOUT_CNT == i){	// 300ms*3 
					printk("[%s] Fatal error: sdio can't get nv information with eng_rf_nv_config, mobile need to be rebooted\n", __FUNCTION__);
			        mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
					return false;
				}
				msleep(300);
			}
		}
#endif

		/* Down bt code for power sleep function */
		/*leon liu masked power sleep enable macro for downloading BT code*/
       		/* #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE*/
	    	Set_Trout_Download_BT_Code();
		/*#endif*/
	}
	else
	{
	    /*if the pskey is the defalut file, need to download the pskey again */
	    Set_Trout_Download_BT_Code();
		printk("Trout is already Power on \n");
		//do nothing;
	}
	#endif

	if(cur_sdio_func == NULL)	//add by chengwg.
	{
		printk("%s: Fatal Err, trout not detected!\n");
		mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
		return false;
	}
	
	Set_Trout_Common_Start(MODE_ID);
	//updata Power mode 
	Set_Power_Mode( MODE_ID);	
	
	Set_Trout_RF_Start(MODE_ID);
	
	mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex

	printk("=============Power On exit==============\n\n");
	return true;
}


//Power off wireless connection module
bool Set_Trout_PowerOff(unsigned int MODE_ID)
{
	printk("\n==========Power Off==========\n");
	
	if( (MODE_ID !=WIFI_MODE) && (MODE_ID !=FM_MODE) && (MODE_ID !=BT_MODE) )
	{
		printk("ATTENTION!!! Bad Mode ID.\n");
		return false;
	}
	mutex_lock(&trout_power_mutex_lock);//Lock Mode_mutex

	Set_Trout_RF_Stop(MODE_ID);
	
	Set_Trout_Common_Stop(MODE_ID);  //zhouxw
	//updata Power mode
	Clear_Power_Mode(MODE_ID);
	
	//Hugh: make wifi mac to sleep.2013-04-24
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
	if(MODE_ID == WIFI_MODE){
	  /* do it before wifimac_sleep, or will never wakeup in some case by zhao */
		mdelay(10);	
		set_phy_reg();   //make sure phy can be write
		wifimac_sleep();
	}
#endif

#ifdef TROUT_PDN_ENABLE
	if( (g_trout_power_mode & (WIFI_MODE |FM_MODE|BT_MODE) ) == POWER_OFF)
	{
		printk("Set Trout Power off for %d \n",MODE_ID);
		MxdRfSetClkReqLow();
		//call Power Off function;	
		Set_Power_Control(false);
		g_is_BT_Code_Download = false;
	}
	else
	{
		printk("Other wireless module is Power on \n");
		//do nothing;
	}
#endif

	mutex_unlock(&trout_power_mutex_lock);//unLock Mode_mutex
	printk("=============================\n\n");
	return true;
}


/* Download BT Code Interface  */
bool Set_Trout_Download_BT_Code(void)
{
	int ret;
	
	if(g_is_BT_Code_Download)
	{
		pr_info("BT code have been downloaded.\n");
		return true;
	}

	pr_info("download_bt_code begin.\n");    
	ret = download_bt_code();	
	pr_info("download_bt_code end, ret=%d \n", ret);


	if(ret < 0)
	{
		pr_info("download_bt_code failed.\n");
#if 1
		g_is_BT_Code_Download = false;//Attention!!!poweroff set false
#endif
		return false;
	}
	
	g_is_BT_Code_Download = true;//Attention!!!poweroff set false

	return true;
}
void Trout_Print_Sys_Common_Reg(void)
{
	int i;
	unsigned int reg_data;
	unsigned int base_addr;
	printk("\n======Trout_Print SYSTEM REG begin ==========\n");
	for(i = 0 ; i < 0x13; i++)
	{
		reg_data = host_read_trout_reg((i << 2));
		printk("0x%x : 0x%x  ;",i, reg_data);
		if(i % 4 == 0)
		{
			printk("\r\n");
		}
	}
	printk("\r\n");
	for(i = 0x40; i < 0x8c; i++)
	{
		reg_data = host_read_trout_reg((i << 2));
		printk("0x%x : 0x%x  ;",i, reg_data);
		if(i % 4 == 0)
		{
			printk("\r\n");
		}
	}
	printk("\r\n");
	reg_data = host_read_trout_reg((0x1FF << 2));
	printk("0x1FF : 0x%x ", reg_data);
	printk("\r\n");
	printk("\n======Trout_Print SYSTEM REG end ==========\n");
	printk("\n======Trout_Print COMMON REG begin ==========\n");
	base_addr = 0x4000;
	for(i = 0 ; i < 0x74; i++)
	{
        unsigned int addr = base_addr + i;
		reg_data = host_read_trout_reg((addr << 2));
		printk("0x%x : 0x%x  ;",i, reg_data);
		if(i % 4 == 0)
		{
			printk("\r\n");
		}
	}
	printk("\r\n");
	reg_data = host_read_trout_reg(((0x40FF)<< 2));
	printk("0x40FF : 0x%x  ",reg_data);
	printk("\n======Trout_Print COMMON REG end ==========\n");
}

/***  :::::::::::::::::::::::::::END INTERFACE:::::::::::::::::::::::::::::::::  ****/

