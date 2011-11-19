/******************************************************************************
 ** File Name:      sensor_drv.c                                                  *
 ** Author:         Liangwen.Zhen                                             *
 ** DATE:           04/19/2006                                                *
 ** Copyright:      2006 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    This file defines the basic operation interfaces of sensor*
 **                                                                           *
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 04/19/2006     Liangwen.Zhen    Create.                                   *
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/ 


/**---------------------------------------------------------------------------*
 **                         Debugging Flag                                    *
 **---------------------------------------------------------------------------*/

#define DEBUG_SENSOR_DRV
#ifdef DEBUG_SENSOR_DRV
#define SENSOR_PRINT   printk
#else
#define SENSOR_PRINT(...)  
#endif
#define SENSOR_PRINT_ERR printk
#define SENSOR_PRINT_HIGH printk

#include <linux/delay.h>
#include "sensor_drv.h"
#include "sensor_cfg.h"
#include "dcam_reg_sc8800g2.h"
#include "dcam_power_sc8800g2.h"
#include <mach/adi_hal_internal.h>
#include <linux/dcam_sensor.h>
#include <mach/clock_common.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/ldo.h>

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif
/**---------------------------------------------------------------------------*
 **                         Macro Definition                                   *
 **---------------------------------------------------------------------------*/
#define SENSOR_ONE_I2C	1
#define SENSOR_ZERO_I2C	0
#define SENSOR_16_BITS_I2C	2
#define SENSOR_I2C_FREQ      (100*1000)
#define SENSOR_I2C_PORT_0		0
#define SENSOR_I2C_ACK_TRUE		1
#define SENSOR_I2C_ACK_FALSE		0
#define SENSOR_I2C_STOP    1
#define SENSOR_I2C_NOSTOP    0
#define SENSOR_I2C_NULL_HANDLE  -1
#define SENSOR_ADDR_BITS_8   1
#define SENSOR_ADDR_BITS_16   2
#define SENSOR_CMD_BITS_8   1
#define SENSOR_CMD_BITS_16   2
#define SENSOR_LOW_SEVEN_BIT     0x7f
#define SENSOR_LOW_EIGHT_BIT     0xff
#define SENSOR_HIGN_SIXTEEN_BIT  0xffff0000
#define SENSOR_LOW_SIXTEEN_BIT  0xffff
#define SENSOR_I2C_OP_TRY_NUM   4
#define SCI_TRUE 1 
#define SCI_FALSE 0
#define SENSOR_CCIRRST 77
#define SENSOR_POWER1 78
#define SENSOR_POWER0 79
#define SENSOR_CCIRRST_STR "ccirrst"
#define SENSOR_POWER1_STR "ccirpd1"
#define SENSOR_POWER0_STR "ccirpd0"

/**---------------------------------------------------------------------------*
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL SENSOR_INFO_T* s_sensor_list_ptr[SENSOR_ID_MAX];//={0x00}; 
LOCAL SENSOR_INFO_T* s_sensor_info_ptr=PNULL;
LOCAL SENSOR_EXP_INFO_T s_sensor_exp_info;//={0x00};
LOCAL uint32_t s_sensor_mclk=0;
//LOCAL uint8_t s_sensor_probe_index=0;
LOCAL BOOLEAN s_sensor_init=SENSOR_FALSE;  
//LOCAL BOOLEAN s_sensor_open=SENSOR_FALSE;
//LOCAL BOOLEAN s_atv_init=SENSOR_FALSE;
//LOCAL BOOLEAN s_atv_open=SENSOR_FALSE;
LOCAL SENSOR_TYPE_E s_sensor_type=SENSOR_TYPE_NONE;
LOCAL SENSOR_MODE_E s_sensor_mode[SENSOR_ID_MAX]={SENSOR_MODE_MAX,SENSOR_MODE_MAX,SENSOR_MODE_MAX};
LOCAL SENSOR_MUTEX_PTR	s_imgsensor_mutex_ptr=PNULL;
LOCAL SENSOR_REGISTER_INFO_T s_sensor_register_info={0x00};
LOCAL SENSOR_REGISTER_INFO_T_PTR s_sensor_register_info_ptr=&s_sensor_register_info;
struct clk *s_ccir_clk = NULL;//for power manager
struct clk *s_ccir_enable_clk = NULL;//for power manager

//wxz20110208: define for I2C driver.
static struct i2c_client *this_client = NULL;
static int g_is_main_sensor = 0;
static int g_is_register_sensor = 0;
#define SENSOR_DEV_NAME	SENSOR_MAIN_I2C_NAME //"sensor"

static const struct i2c_device_id sensor_main_id[] = {
	{ SENSOR_MAIN_I2C_NAME, 0 },
	{ }
};
static const struct i2c_device_id sensor_sub_id[] = {
	{ SENSOR_SUB_I2C_NAME, 0 },
	{ }
};
//static unsigned short sensor_main_force[] = {2, SENSOR_MAIN_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static unsigned short sensor_main_force[] = {2, SENSOR_MAIN_I2C_ADDR_CFG, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const sensor_main_forces[] = { sensor_main_force, NULL };
static struct i2c_client_address_data sensor_main_addr_data = { .forces = sensor_main_forces,};
//static unsigned short sensor_sub_force[] = {2, SENSOR_MAIN_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static unsigned short sensor_sub_force[] = {2, SENSOR_SUB_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static unsigned short sensor_sub_force[] = {2, SENSOR_SUB_I2C_ADDR_CFG, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const sensor_sub_forces[] = { sensor_sub_force, NULL };
static struct i2c_client_address_data sensor_sub_addr_data = { .forces = sensor_sub_forces,};

/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                     Local Function Prototypes                             *
 **---------------------------------------------------------------------------*/
 #define SENSOR_INHERIT 0
 #define SENSOR_WAIT_FOREVER 0

 static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	SENSOR_PRINT("SENSOR:sensor_probe E.\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_PRINT("SENSOR: %s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
	}	
	this_client = client;	
	if(SENSOR_MAIN == Sensor_GetCurId()){
		if(SENSOR_MAIN_I2C_ADDR_CFG != (this_client->addr & (~0xFF))) {
			this_client->addr = (this_client->addr & (~0xFF)) | (sensor_main_force[1]&0xFF); 
		}
	}
	else{ //for SENSOR_SUB	
		if(SENSOR_SUB_I2C_ADDR_CFG != (this_client->addr & (~0xFF))) {
			this_client->addr = (this_client->addr & (~0xFF)) | (sensor_sub_force[1]&0xFF); 
		}
	}
	SENSOR_PRINT("sensor_probe,this_client->addr =0x%x\n",this_client->addr );
	mdelay(20);
	
	return 0;
out:
	return res;
}
static int sensor_remove(struct i2c_client *client)
{
	//do nothing.
	return 0;
}
static int sensor_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	//strcpy(info->type, SENSOR_DEV_NAME);
	strcpy(info->type, client->name);
	
	return 0;
}

static struct i2c_driver sensor_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE, 
        //.name  = SENSOR_DEV_NAME,
    },
	.probe      = sensor_probe,
	.remove     = sensor_remove,
	.detect     = sensor_detect,
	//.id_table = sensor_main_id,
	//.address_data = &sensor_main_addr_data,
};

SENSOR_MUTEX_PTR SENSOR_CreateMutex(const char *name_ptr, uint32_t priority_inherit)
{
	return (SENSOR_MUTEX_PTR)1;
}
uint32_t SENSOR_DeleteMutex(SENSOR_MUTEX_PTR mutex_ptr)
{
	return SENSOR_SUCCESS;
}
uint32_t SENSOR_GetMutex(SENSOR_MUTEX_PTR mutex_ptr, uint32_t wait_option)
{
	return SENSOR_SUCCESS;
}
uint32_t SENSOR_PutMutex(SENSOR_MUTEX_PTR mutex_ptr)
{
	return SENSOR_SUCCESS;
}
/*****************************************************************************/
//  Description: Create Mutex
//	Global resource dependence:
//  Author: Tim.zhu
//	Note:
//		input:
//			none
//		output:
//			none
//		return:
//			Mutex
/*****************************************************************************/
PUBLIC void ImgSensor_CreateMutex(void)
{	
	s_imgsensor_mutex_ptr = SENSOR_CreateMutex("IMG SENSOR SYNC MUTEX", SENSOR_INHERIT);
	SENSOR_PASSERT((s_imgsensor_mutex_ptr!=PNULL),("IMG SENSOR Great MUTEX fail!"));	
	
}

/*****************************************************************************/
//  Description: Delete Mutex
//	Global resource dependence:
//  Author: Tim.zhu
//	Note:
//		input:
//			sm    -  Mutex
//		output:
//			none
//		return:
//			none
/*****************************************************************************/
PUBLIC void ImgSensor_DeleteMutex(void)
{
	uint32_t ret;
	
	if(DCAM_NULL==s_imgsensor_mutex_ptr)
	{
        return ;
    }
    
	ret=SENSOR_DeleteMutex(s_imgsensor_mutex_ptr);	
    SENSOR_ASSERT(ret==SENSOR_SUCCESS );	
    s_imgsensor_mutex_ptr=DCAM_NULL;	
}

/*****************************************************************************/
//  Description: Get Mutex
//	Global resource dependence:
//  Author: Tim.zhu
//	Note:
//		input:
//			sm    -  mutex
//		output:
//			none
//		return:
//			none
/*****************************************************************************/
PUBLIC void ImgSensor_GetMutex(void)
{
    uint32_t ret;
    if(PNULL==s_imgsensor_mutex_ptr)
    {
        ImgSensor_CreateMutex();
    }

    ret = SENSOR_GetMutex(s_imgsensor_mutex_ptr, SENSOR_WAIT_FOREVER);
    SENSOR_ASSERT( ret == SENSOR_SUCCESS );
}
/*****************************************************************************/
//  Description: Put mutex
//	Global resource dependence:
//  Author: Tim.zhu
//	Note:
//		input:
//			sm    -  mutex
//		output:
//			none
//		return:
//			none
/*****************************************************************************/
PUBLIC void ImgSensor_PutMutex(void)
{
    uint32_t ret;    
    if(DCAM_NULL==s_imgsensor_mutex_ptr)
    {
        return;
    }
    
    ret = SENSOR_PutMutex(s_imgsensor_mutex_ptr);	
    SENSOR_ASSERT( ret == SENSOR_SUCCESS );
}


/*****************************************************************************/
//  Description:    This function is used to get sensor type    
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_TYPE_E _Sensor_GetSensorType(void) 
{
	return s_sensor_type;
}

/*****************************************************************************/
//  Description:    This function is used to reset sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC void Sensor_Reset(uint32_t level)
{
	gpio_request(SENSOR_CCIRRST, SENSOR_CCIRRST_STR);

	gpio_direction_output(SENSOR_CCIRRST,!level);	
	
	gpio_set_value(SENSOR_CCIRRST, !level);
	msleep(20);
	gpio_set_value(SENSOR_CCIRRST, level);
	msleep(100);
	gpio_set_value(SENSOR_CCIRRST, !level);	

	gpio_free(SENSOR_CCIRRST);	
}

#if 0
PUBLIC void Sensor_Reset(void)
{
#define SENSOR_RST_CTL 77
#define SENSOR_POWER1_CTL 78
#define SENSOR_POWER0_CTL 79

SENSOR_PRINT("SENSOR: Sensor_Reset E \n");
	gpio_request(SENSOR_POWER1_CTL,"ccir_pd1");
	gpio_request(SENSOR_POWER0_CTL,"ccir_pd0");
	gpio_request(SENSOR_RST_CTL,"ccir_rst");
	
	gpio_direction_output(SENSOR_POWER1_CTL,0);
	gpio_direction_output(SENSOR_POWER0_CTL,0);
	gpio_direction_output(SENSOR_RST_CTL,1);
    	SENSOR_Sleep(10);
	gpio_direction_output(SENSOR_RST_CTL,0);
    	SENSOR_Sleep(20);
	gpio_direction_output(SENSOR_RST_CTL,1);
	SENSOR_Sleep(100);
	//SENSOR_Sleep(10);
	SENSOR_PRINT("SENSOR: Sensor_Reset X \n");	
	/*BOOLEAN 				reset_pulse_level;
	uint32_t					reset_pulse_width;
	SENSOR_IOCTL_FUNC_PTR 	reset_func;

	reset_pulse_level = (BOOLEAN)s_sensor_info_ptr->reset_pulse_level;
	reset_pulse_width = s_sensor_info_ptr->reset_pulse_width;
	reset_func        = s_sensor_info_ptr->ioctl_func_tab_ptr->reset;	

	SENSOR_PRINT("SENSOR: Sensor_Reset -> reset_pulse_level = %d\n", reset_pulse_level);
	// HW Reset sensor
	if(PNULL != reset_func)
	{
		reset_func(0);
	}
	else
	{
		//if(NULL == reset_pulse_width)
		if(0 == reset_pulse_width)
		{
			reset_pulse_width = SENSOR_RESET_PULSE_WIDTH_DEFAULT;
		}
		else if(SENSOR_RESET_PULSE_WIDTH_MAX < reset_pulse_width)
		{
			reset_pulse_width = SENSOR_RESET_PULSE_WIDTH_MAX;
		}
		
		//GPIO_ResetSensor(reset_pulse_level, reset_pulse_width); //wxz:???
	}*/

}
#endif

/*****************************************************************************/
//  Description:    This function is used to power on sensor and select xclk    
//  Author:         Liangwen.Zhen
//  Note:           1.Unit: MHz 2. if mclk equal 0, close main clock to sensor
/*****************************************************************************/
#if 1 //for Power manager
int Sensor_SetMCLK(uint32_t mclk)
{
    uint32_t divd = 0;
    char *name_parent = NULL;
    struct clk *clk_parent = NULL;
    int ret;    
	
    SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %dMHz, clk = %dMHz\n", s_sensor_mclk, mclk);

    if((0 != mclk) && (s_sensor_mclk != mclk))
    {
    	if(s_ccir_clk){
		clk_disable(s_ccir_clk);		
		SENSOR_PRINT("###sensor s_ccir_clk clk_disable ok.\n");
    	}
	else{
	    	s_ccir_clk = clk_get(NULL, "ccir_mclk");
		if(IS_ERR(s_ccir_clk)){
			SENSOR_PRINT_ERR("###: Failed: Can't get clock [ccir_mclk]!\n");
			SENSOR_PRINT_ERR("###: s_sensor_clk = %p.\n", s_ccir_clk);
		}
		else{
			SENSOR_PRINT("###sensor s_ccir_clk clk_get ok.\n");
		}	
	}
        if(mclk > SENSOR_MAX_MCLK)
        {
            mclk = SENSOR_MAX_MCLK;
        }
	name_parent = "clk_48m";	
	clk_parent = clk_get(NULL, name_parent);
	if(!clk_parent){
		SENSOR_PRINT_ERR("###:clock[%s]: failed to get parent [%s] by clk_get()!\n", s_ccir_clk->name, name_parent);
		return -EINVAL;
	}

	ret = clk_set_parent(s_ccir_clk, clk_parent);
	if(ret){
		SENSOR_PRINT_ERR("###:clock[%s]: clk_set_parent() failed!parent: %s, usecount: %d.\n", s_ccir_clk->name, clk_parent->name, s_ccir_clk->usecount);
		return -EINVAL;
	}
        divd = SENSOR_MAX_MCLK / mclk;      
	ret = clk_set_divisor(s_ccir_clk, divd);
	if(ret){
		SENSOR_PRINT_ERR("###:clock[%s]: clk_set_divisor failed!\n", s_ccir_clk->name);
		return -EINVAL;
	}
	ret = clk_enable(s_ccir_clk);
	if(ret){
		SENSOR_PRINT_ERR("###:clock[%s]: clk_enable() failed!\n", s_ccir_clk->name);
	}
	else{
		SENSOR_PRINT("###sensor s_ccir_clk clk_enable ok.\n");
	}
     
       // CCIR CLK Enable
    	if(NULL == s_ccir_enable_clk){
	    	s_ccir_enable_clk = clk_get(NULL, "clk_ccir");
		if(IS_ERR(s_ccir_enable_clk)){
			SENSOR_PRINT_ERR("###: Failed: Can't get clock [clk_ccir]!\n");
			SENSOR_PRINT_ERR("###: s_ccir_enable_clk = %p.\n", s_ccir_enable_clk);
			return -EINVAL;
		} 
		else{
			SENSOR_PRINT("###sensor s_ccir_enable_clk clk_get ok.\n");
		}	
		ret = clk_enable(s_ccir_enable_clk);
		if(ret){
			SENSOR_PRINT_ERR("###:clock[%s]: clk_enable() failed!\n", s_ccir_enable_clk->name);
		}
		else{
			SENSOR_PRINT("###sensor s_ccir_enable_clk clk_enable ok.\n");
		}
	}	
        
        s_sensor_mclk = mclk;
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %d Hz, divd = %d\n", s_sensor_mclk, divd);
    }
    else if(0 == mclk)
    { 
	if(s_ccir_clk){
		clk_disable(s_ccir_clk);
		SENSOR_PRINT("###sensor s_ccir_clk clk_disable ok.\n");
		clk_put(s_ccir_clk);		
		SENSOR_PRINT("###sensor s_ccir_clk clk_put ok.\n");
		s_ccir_clk = NULL;
	}
	// CCIR CLK disable
	if(s_ccir_enable_clk){
		clk_disable(s_ccir_enable_clk);
		SENSOR_PRINT("###sensor s_ccir_enable_clk clk_disable ok.\n");
		clk_put(s_ccir_enable_clk);		
		SENSOR_PRINT("###sensor s_ccir_enable_clk clk_put ok.\n");
		s_ccir_enable_clk = NULL;
	}	

        s_sensor_mclk = 0;
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> Disable MCLK !!!");
    }
    else
    {
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> Do nothing !! ");
    }
    SENSOR_PRINT("SENSOR: Sensor_SetMCLK X\n");

    return 0;
}
#else
PUBLIC void Sensor_SetMCLK(uint32_t mclk)
{
    uint32_t divd = 0;
    //uint32_t pll_clk = 0;	
	
    SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %dMHz, clk = %dMHz\n", s_sensor_mclk, mclk);

    if((0 != mclk) && (s_sensor_mclk != mclk))
    {
        if(mclk > SENSOR_MAX_MCLK)
        {
            mclk = SENSOR_MAX_MCLK;
        }

       // *(volatile uint32_t *)GR_GEN0 &= ~BIT_14; //first disable MCLK
       _paad(ARM_GLOBAL_REG_GEN0, ~BIT_14);

        //*(volatile uint32_t*)GR_PLL_SCR &= ~(BIT_19 | BIT_18); // bit19, bit18 ,  00 48M,01  76.8M, 1x, 26M
        _paad(ARM_GLOBAL_PLL_SCR, ~(BIT_18 | BIT_19));

        //*(volatile uint32_t *)GR_GEN3 &= ~(BIT_24 | BIT_25); // CCIR divide factor        
        _paad(ARM_GLOBAL_REG_GEN3, ~(BIT_24 | BIT_25));
		
        divd = SENSOR_MAX_MCLK / mclk - 1;

        if(divd > (BIT_0 | BIT_1))
        {
           divd = (BIT_0 | BIT_1); 
        }
        
        divd <<= 24;

        //*(volatile uint32_t *)GR_GEN3 |= divd; // CCIR divide factor
        _paod(ARM_GLOBAL_REG_GEN3, divd);
        
       // *(volatile uint32_t *)GR_GEN0 |= BIT_14; // CCIR CLK Enable
        _paod(ARM_GLOBAL_REG_GEN0, BIT_14);
        
        s_sensor_mclk = mclk;
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %d Hz, divd = %d\n", s_sensor_mclk, divd);
    }
    else if(0 == mclk)
    {
        //*(volatile uint32_t *)GR_GEN0 &= ~BIT_14;
	_paad(ARM_GLOBAL_REG_GEN0, ~BIT_14);

        s_sensor_mclk = 0;
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> Disable MCLK !!!");
    }
    else
    {
        SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> Do nothing !! ");
    }
    SENSOR_PRINT("SENSOR: Sensor_SetMCLK X\n");
}
#endif

PUBLIC void Sensor_SetVoltage(SENSOR_AVDD_VAL_E camd0_val, SENSOR_AVDD_VAL_E camd1_val, SENSOR_AVDD_VAL_E cama_val)
{
	uint32_t ldo_camd0_level = LDO_VOLT_LEVEL0;
	uint32_t ldo_camd1_level = LDO_VOLT_LEVEL0;
	uint32_t ldo_cama_level = LDO_VOLT_LEVEL0;

	SENSOR_PRINT("Sensor_SetVoltage: %d, %d, %d.\n", camd0_val, camd1_val,  cama_val);

	switch(camd1_val)
	{            
		case SENSOR_AVDD_2800MV:
			ldo_camd1_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_3300MV:
			ldo_camd1_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_1800MV:
			ldo_camd1_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1200MV:
			ldo_camd1_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_camd1_level = LDO_VOLT_LEVEL_MAX;   
			break;
	} 
	switch(cama_val)
	{            
		case SENSOR_AVDD_2800MV:
			ldo_cama_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_3000MV:
			ldo_cama_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_2500MV:
			ldo_cama_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1800MV:
			ldo_cama_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_cama_level = LDO_VOLT_LEVEL_MAX;   
			break;
	} 
   
	if((LDO_VOLT_LEVEL_MAX == ldo_cama_level) || (LDO_VOLT_LEVEL_MAX == ldo_camd1_level))
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off camd1, cama\n"); 
		LDO_TurnOffLDO(LDO_LDO_CAMA);        
		LDO_TurnOffLDO(LDO_LDO_CAMD1);            
	}
	else
	{	
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on camd1 and cama, cama VoltLevel %d, camd1 VoltLevel %d\n",ldo_cama_level, ldo_camd1_level); 

		LDO_SetVoltLevel(LDO_LDO_CAMA, ldo_cama_level);      
		LDO_TurnOnLDO(LDO_LDO_CAMA); 
		LDO_SetVoltLevel(LDO_LDO_CAMD1, ldo_camd1_level);
		LDO_TurnOnLDO(LDO_LDO_CAMD1);  
	}

	switch(camd0_val)
	{
		case SENSOR_AVDD_1800MV:
			ldo_camd0_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_2800MV:
			ldo_camd0_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_1500MV:
			ldo_camd0_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1300MV:
			ldo_camd0_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_camd0_level = LDO_VOLT_LEVEL_MAX;           
			break;
	} 
    
	if(LDO_VOLT_LEVEL_MAX == ldo_camd0_level)
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off camd0, sensor_id %d\n", Sensor_GetCurId()); 
		LDO_TurnOffLDO(LDO_LDO_CAMD0);
	}
	else
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on camd0, sensor_id %d, camd0 VoltLevel %d\n",Sensor_GetCurId(),ldo_camd0_level); 
		LDO_TurnOnLDO(LDO_LDO_CAMD0);    
		LDO_SetVoltLevel(LDO_LDO_CAMD0, ldo_camd0_level);      
	}  

	return ;
}

/*****************************************************************************/
//  Description:    This function is used to power on/off sensor     
//  Author:         Liangwen.Zhen
//  Note:           SENSOR_TRUE: POWER ON; SENSOR_FALSE: POWER OFF
/*****************************************************************************/
LOCAL void Sensor_PowerOn(BOOLEAN power_on)
{
	BOOLEAN 				power_down;		
	SENSOR_AVDD_VAL_E		dvdd_val;
	SENSOR_AVDD_VAL_E		avdd_val;
	SENSOR_AVDD_VAL_E		iovdd_val;    
	SENSOR_IOCTL_FUNC_PTR	power_func;
	uint32_t                                     rst_lvl = s_sensor_info_ptr->reset_pulse_level;

	power_down = (BOOLEAN)s_sensor_info_ptr->power_down_level;
	dvdd_val   = s_sensor_info_ptr->dvdd_val;
	avdd_val   = s_sensor_info_ptr->avdd_val;
	iovdd_val   = s_sensor_info_ptr->iovdd_val;
	power_func = s_sensor_info_ptr->ioctl_func_tab_ptr->power;

	SENSOR_PRINT("SENSOR: Sensor_PowerOn -> power_on = %d, power_down_level = %d, avdd_val = %d\n",power_on,power_down,avdd_val);   

	if(power_on)
	{				
		// Open power
		Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val); 
		// Reset sensor		
		Sensor_Reset(rst_lvl);		
		Sensor_PowerDown(!power_down);		
		msleep(10);
		Sensor_SetMCLK(SENSOR_DEFALUT_MCLK); 
	}
	else
	{
		// Power down sensor and maybe close DVDD, DOVDD
		Sensor_PowerDown(power_down);
		msleep(20);
		Sensor_SetMCLK(SENSOR_DISABLE_MCLK);			 
		Sensor_SetVoltage(SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED);	
	}   			
       
}

/*****************************************************************************/
//  Description:    This function is used to power down sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
#ifdef PLATFORM_6810
PUBLIC BOOLEAN Sensor_PowerDown(BOOLEAN power_level)
{
	SENSOR_PRINT("SENSOR: Sensor_PowerDown -> main: power_down %d\n", power_level);          
	SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD1-> 0x8C000344 0x%x\n", _pard(PIN_CTL_CCIRPD1)); 
	SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD0-> 0x8C000348 0x%x\n", _pard(PIN_CTL_CCIRPD0));

	switch(Sensor_GetCurId())
	{
		case SENSOR_MAIN:
		{
			_paod(PIN_CTL_CCIRPD1,  (BIT_4|BIT_5));
			gpio_request(SENSOR_POWER1,SENSOR_POWER1_STR);
			if(0 == power_level)
			{
				gpio_direction_output(SENSOR_POWER1,0);
			}
			else
			{
				gpio_direction_output(SENSOR_POWER1,1);
			}      
			gpio_free(SENSOR_POWER1);
			break;
		}
		case SENSOR_SUB:
		{
			_paod(PIN_CTL_CCIRPD0,  (BIT_4|BIT_5));
			gpio_request(SENSOR_POWER0,SENSOR_POWER0_STR);
			if(0 == power_level)
			{
				gpio_direction_output(SENSOR_POWER0,0);
			}
			else
			{
				gpio_direction_output(SENSOR_POWER0,1);
			}
			gpio_free(SENSOR_POWER0);
			break;
		}
		default :
		break;            
	}

	return SENSOR_SUCCESS;
}
#else 
PUBLIC BOOLEAN Sensor_PowerDown(BOOLEAN power_down)
{
	switch(Sensor_GetCurId())
	{
		case SENSOR_MAIN:
		{
			SENSOR_PRINT("SENSOR: Sensor_PowerDown -> main: power_down %d\n", power_down);          
			SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD1-> 0x8C000344 0x%x\n", _pard(PIN_CTL_CCIRPD1)); 
			SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD0-> 0x8C000348 0x%x\n", _pard(PIN_CTL_CCIRPD0));

			_paad(PIN_CTL_CCIRPD1,  ~(BIT_4|BIT_5));
			_paod(AHB_GLOBAL_REG_CTL0,  BIT_1|BIT_2);
			if(power_down == 0)
			{
				_paad(CAP_CNTRL,  ~(BIT_11|BIT_12));			
				mdelay(10);			
				_paod(CAP_CNTRL,  BIT_11);
			}
			else
			{			
				_paod(CAP_CNTRL,  BIT_12);
			}			
			_paad(AHB_GLOBAL_REG_CTL0,  ~(BIT_1|BIT_2));            
			break;
		}
		case SENSOR_SUB:
		{
			SENSOR_PRINT("SENSOR: Sensor_PowerDown -> sub: power_down %d\n", power_down);     
			SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD1-> 0x8C000344 0x%x\n", _pard(PIN_CTL_CCIRPD1));       
			SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD0-> 0x8C000348 0x%x\n", _pard(PIN_CTL_CCIRPD0));

			_paod(AHB_GLOBAL_REG_CTL0,  BIT_1|BIT_2);
			if(power_down == 0)
			{
				_paad(CAP_CNTRL,  ~(BIT_11|BIT_13));		
				mdelay(10);
				_paod(CAP_CNTRL,  BIT_11); 
				SENSOR_PRINT("sub power down. 0: CAP_CNTRL: 0x%x\n", _pard(CAP_CNTRL));
			}
			else
			{			
				_paod(CAP_CNTRL,  BIT_13);
				SENSOR_PRINT("sub power down. 1 : CAP_CNTRL: 0x%x\n", _pard(CAP_CNTRL));
			}
			_paad(AHB_GLOBAL_REG_CTL0,  ~(BIT_1|BIT_2));
			break;
		}
		case SENSOR_ATV:
		{
			SENSOR_PRINT("SENSOR: Sensor_PowerDown -> atv");
			break;
		}
		default :
			break;            
	}

	return SENSOR_SUCCESS;
}

#endif
/*****************************************************************************/
//  Description:    This function is used to reset img sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_SetResetLevel(BOOLEAN level)
{
        int err = 0xff;
        err = gpio_request(SENSOR_CCIRRST, SENSOR_CCIRRST_STR);
        if (err) {
                printk("Sensor_Reset failed requesting err=%d\n", err);
                return SENSOR_FAIL;
        }
        gpio_direction_output(SENSOR_CCIRRST, level);
        gpio_set_value(SENSOR_CCIRRST, level);
	gpio_free(SENSOR_CCIRRST);

    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to power on/off sensor     
//  Author:         Liangwen.Zhen
//  Note:           SENSOR_TRUE: POWER ON; SENSOR_FALSE: POWER OFF
/*****************************************************************************/
LOCAL void Sensor_SetExportInfo(SENSOR_EXP_INFO_T * exp_info_ptr)
{
	SENSOR_REG_TAB_INFO_T* resolution_info_ptr = PNULL;
	SENSOR_TRIM_T_PTR resolution_trim_ptr = PNULL;
	SENSOR_INFO_T* sensor_info_ptr = s_sensor_info_ptr;
	uint32_t i = 0;

	SENSOR_PRINT("SENSOR: Sensor_SetExportInfo.\n");	

	SENSOR_MEMSET(exp_info_ptr, 0x00, sizeof(SENSOR_EXP_INFO_T));
	exp_info_ptr->image_format = sensor_info_ptr->image_format;
	exp_info_ptr->image_pattern = sensor_info_ptr->image_pattern;	

	exp_info_ptr->pclk_polarity = (sensor_info_ptr->hw_signal_polarity & 0x01) ;  //the high 3bit will be the phase(delay sel)
	exp_info_ptr->vsync_polarity = ((sensor_info_ptr->hw_signal_polarity >> 2) & 0x1);
	exp_info_ptr->hsync_polarity = ((sensor_info_ptr->hw_signal_polarity >> 4) & 0x1);
	exp_info_ptr->pclk_delay = ((sensor_info_ptr->hw_signal_polarity >> 5) & 0x07);

	exp_info_ptr->source_width_max = sensor_info_ptr->source_width_max;
	exp_info_ptr->source_height_max = sensor_info_ptr->source_height_max;	

	exp_info_ptr->environment_mode = sensor_info_ptr->environment_mode;
	exp_info_ptr->image_effect = sensor_info_ptr->image_effect;	
	exp_info_ptr->wb_mode = sensor_info_ptr->wb_mode;
	exp_info_ptr->step_count = sensor_info_ptr->step_count;

	exp_info_ptr->ext_info_ptr = sensor_info_ptr->ext_info_ptr;

	exp_info_ptr->preview_skip_num = sensor_info_ptr->preview_skip_num;
	exp_info_ptr->capture_skip_num = sensor_info_ptr->capture_skip_num;    
	exp_info_ptr->preview_deci_num = sensor_info_ptr->preview_deci_num;  
	exp_info_ptr->video_preview_deci_num = sensor_info_ptr->video_preview_deci_num; 

	exp_info_ptr->threshold_eb = sensor_info_ptr->threshold_eb;
	exp_info_ptr->threshold_mode = sensor_info_ptr->threshold_mode;    
	exp_info_ptr->threshold_start = sensor_info_ptr->threshold_start;  
	exp_info_ptr->threshold_end = sensor_info_ptr->threshold_end; 

	exp_info_ptr->ioctl_func_ptr=sensor_info_ptr->ioctl_func_tab_ptr;
	if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->get_trim)
	{
		resolution_trim_ptr=(SENSOR_TRIM_T_PTR)sensor_info_ptr->ioctl_func_tab_ptr->get_trim(0x00);
	}
	for(i=SENSOR_MODE_COMMON_INIT; i<SENSOR_MODE_MAX; i++)
	{
		resolution_info_ptr = &(sensor_info_ptr->resolution_tab_info_ptr[i]);
		if((PNULL!= resolution_info_ptr->sensor_reg_tab_ptr)||((0x00!=resolution_info_ptr->width)&&(0x00!=resolution_info_ptr->width)))
		{
			exp_info_ptr->sensor_mode_info[i].mode=i;
			exp_info_ptr->sensor_mode_info[i].width=resolution_info_ptr->width;
			exp_info_ptr->sensor_mode_info[i].height=resolution_info_ptr->height;
			if((PNULL!=resolution_trim_ptr)
			&&(0x00!=resolution_trim_ptr[i].trim_width)
			&&(0x00!=resolution_trim_ptr[i].trim_height))
			{
				exp_info_ptr->sensor_mode_info[i].trim_start_x=resolution_trim_ptr[i].trim_start_x;
				exp_info_ptr->sensor_mode_info[i].trim_start_y=resolution_trim_ptr[i].trim_start_y;
				exp_info_ptr->sensor_mode_info[i].trim_width=resolution_trim_ptr[i].trim_width;
				exp_info_ptr->sensor_mode_info[i].trim_height=resolution_trim_ptr[i].trim_height;
				exp_info_ptr->sensor_mode_info[i].line_time=resolution_trim_ptr[i].line_time;
			}
			else
			{
				exp_info_ptr->sensor_mode_info[i].trim_start_x=0x00;
				exp_info_ptr->sensor_mode_info[i].trim_start_y=0x00;
				exp_info_ptr->sensor_mode_info[i].trim_width=resolution_info_ptr->width;
				exp_info_ptr->sensor_mode_info[i].trim_height=resolution_info_ptr->height;		
			}
			//exp_info_ptr->sensor_mode_info[i].line_time=resolution_trim_ptr[i].line_time;
			if(SENSOR_IMAGE_FORMAT_MAX != sensor_info_ptr->image_format)
			{
			exp_info_ptr->sensor_mode_info[i].image_format = sensor_info_ptr->image_format;
			}
			else
			{
			exp_info_ptr->sensor_mode_info[i].image_format = resolution_info_ptr->image_format;
			}
			SENSOR_PRINT("SENSOR: SENSOR mode Info > mode = %d, width = %d, height = %d, format = %d.\n",\
			i, resolution_info_ptr->width, resolution_info_ptr->height, exp_info_ptr->sensor_mode_info[i].image_format);
		}
		else
		{
			exp_info_ptr->sensor_mode_info[i].mode = SENSOR_MODE_MAX;
		}
	}
}

/**---------------------------------------------------------------------------*
 **                         Function Definitions                              *
 **---------------------------------------------------------------------------*/

int Sensor_WriteReg(uint16_t subaddr, uint16_t data)
{
	uint8_t cmd[4] = {0};
	uint32_t index=0, i=0;
	uint32_t cmd_num = 0;
	struct i2c_msg msg_w;
	int32_t ret = -1;
	SENSOR_IOCTL_FUNC_PTR 	write_reg_func;

	write_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->write_reg;

	if(PNULL != write_reg_func)
	{
		if(SENSOR_OP_SUCCESS != write_reg_func((subaddr << BIT_4) + data))
		{
			SENSOR_PRINT("SENSOR: IIC write : reg:0x%04x, val:0x%04x error\n", subaddr, data);
		}
	}
	else
	{
		if(SENSOR_I2C_REG_16BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_REG_16BIT) )
		{
			cmd[cmd_num++] = (uint8_t)((subaddr >> BIT_3)&SENSOR_LOW_EIGHT_BIT);
			index++;
			cmd[cmd_num++] = (uint8_t)(subaddr & SENSOR_LOW_EIGHT_BIT);		    	
			index++;
		}
		else
		{
			cmd[cmd_num++] = (uint8_t)subaddr;	
			index++;
		}

		if(SENSOR_I2C_VAL_16BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_VAL_16BIT))
		{
			cmd[cmd_num++] = (uint8_t)((data >> BIT_3)&SENSOR_LOW_EIGHT_BIT);
			index++;		
			cmd[cmd_num++] = (uint8_t)(data & SENSOR_LOW_EIGHT_BIT);	    		    	
			index++;		
		}
		else
		{
			cmd[cmd_num++] = (uint8_t)data;
			index++;		
		}

		if(SENSOR_WRITE_DELAY != subaddr)
		{	
			for(i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++)   
			{
				msg_w.addr = this_client->addr; 
				msg_w.flags = 0;
				msg_w.buf = cmd;
				msg_w.len = index;
				ret = i2c_transfer(this_client->adapter, &msg_w, 1);
				if(ret!=1)
				{
					SENSOR_PRINT_ERR("SENSOR: write sensor reg fai, ret : %d, I2C w addr: 0x%x, \n", ret, this_client->addr);
					return -1;
				}
				else
				{
					SENSOR_PRINT("SENSOR: addr: 0x%04x, val:0x%04x.\n", subaddr, data);
					break;
				}
			}           
		}
		else
		{
			msleep(data);
			SENSOR_PRINT("SENSOR: IIC write Delay %d ms.\n", data);	    	
		}
	}
	return 0;
}
uint16_t Sensor_ReadReg(uint16_t reg_addr)
{
	uint32_t  i=0;	
	uint8_t cmd[2] = {0};    
	uint16_t ret_val;
	uint16_t w_cmd_num = 0;
	uint16_t r_cmd_num = 0;
	uint8_t buf_r[2] = {0};
	int32_t ret = -1;
	struct i2c_msg msg_r[2];
	SENSOR_IOCTL_FUNC_PTR 	read_reg_func;

	SENSOR_PRINT("Read:this_client->addr=0x%x\n",this_client->addr);

	read_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->read_reg;

	if(PNULL != read_reg_func)
	{
		ret_val = (uint16_t)read_reg_func((uint32_t)(reg_addr & SENSOR_LOW_SIXTEEN_BIT));
	}
	else
	{
		if(SENSOR_I2C_REG_16BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_REG_16BIT))
		{
			cmd[w_cmd_num++] = (uint8_t)((reg_addr >>BIT_3)&SENSOR_LOW_EIGHT_BIT);				
			cmd[w_cmd_num++] = (uint8_t)(reg_addr & SENSOR_LOW_EIGHT_BIT);			
		}
		else
		{
			cmd[w_cmd_num++] = (uint8_t)reg_addr;		
		}

		if(SENSOR_I2C_VAL_16BIT==(s_sensor_info_ptr->reg_addr_value_bits & SENSOR_I2C_VAL_16BIT) )
		{
			r_cmd_num = SENSOR_CMD_BITS_16;
		}
		else
		{
			r_cmd_num = SENSOR_CMD_BITS_8; 	
		}  

		for(i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++)   
		{
			msg_r[0].addr = this_client->addr; 
			msg_r[0].flags = 0;
			msg_r[0].buf = cmd;
			msg_r[0].len = w_cmd_num;
			msg_r[1].addr = this_client->addr; 
			msg_r[1].flags = I2C_M_RD;
			msg_r[1].buf = buf_r;
			msg_r[1].len = r_cmd_num;
			ret = i2c_transfer(this_client->adapter, msg_r, 2);
			if(ret!=2)
			{
				SENSOR_PRINT_ERR("SENSOR: read sensor reg fai, ret : %d, I2C w addr: 0x%x, \n", ret, this_client->addr);
				msleep(20);
				ret_val = 0xFFFF;
			}
			else
			{			
				ret_val = (r_cmd_num == 1)?(uint16_t)buf_r[0]:(uint16_t)((buf_r[0] << 8) + buf_r[1]);  
				break;
			}
		}
	}
	return  ret_val;
}

int32_t Sensor_WriteReg_8bits(uint16_t reg_addr, uint8_t value)
{
	uint8_t buf_w[2];
	int32_t ret = -1;
	struct i2c_msg msg_w;

	if(0xFFFF == reg_addr) //for delay some msecond
	{		
		mdelay(value);
		SENSOR_PRINT("Sensor_WriteReg_8bits wait %d ms.\n", value);
		return 0;
	}
	
	buf_w[0]= (uint8_t)reg_addr;
	buf_w[1]= value;
	msg_w.addr = this_client->addr; 
	msg_w.flags = 0;
	msg_w.buf = buf_w;
	msg_w.len = 2;
        ret = i2c_transfer(this_client->adapter, &msg_w, 1);
	if(ret!=1)
        {
            SENSOR_PRINT_ERR("#DCAM: write sensor reg fai, ret : %d, I2C w addr: 0x%x, \n", ret, this_client->addr);
            return -1;
        }
	return 0;
}
int32_t Sensor_ReadReg_8bits(uint8_t reg_addr, uint8_t *reg_val)
{
	uint8_t buf_w[1];
	uint8_t buf_r;
	int32_t ret = -1;	
	struct i2c_msg msg_r[2];

	buf_w[0]= reg_addr;
	msg_r[0].addr = this_client->addr; 
	msg_r[0].flags = 0;
	msg_r[0].buf = buf_w;
	msg_r[0].len = 1;
	msg_r[1].addr = this_client->addr; 
	msg_r[1].flags = I2C_M_RD;
	msg_r[1].buf = &buf_r;
	msg_r[1].len = 1;
        ret = i2c_transfer(this_client->adapter, msg_r, 2);
	if(ret!=2)
        {
            SENSOR_PRINT_ERR("#sensor: read sensor reg fail, ret: %d, I2C r addr: 0x%x \n", ret, this_client->addr);
            return -1;
        }
	*reg_val = buf_r;
	return ret;
}

/*****************************************************************************/
//  Description:    This function is used to send a table of register to sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SendRegTabToSensor(SENSOR_REG_TAB_INFO_T * sensor_reg_tab_info_ptr	)
{
	uint32_t i;
	struct timeval  time1, time2;
	SENSOR_PRINT("SENSOR: Sensor_SendRegTabToSensor E.\n");

	do_gettimeofday(&time1);

	for(i = 0; i < sensor_reg_tab_info_ptr->reg_count; i++)
	{
		//ImgSensor_GetMutex();
		Sensor_WriteReg(sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr, sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_value);
	}	
	do_gettimeofday(&time2);
	SENSOR_PRINT("SENSOR: Sensor_SendRegValueToSensor -> reg_count = %d, g_is_main_sensor: %d.\n",sensor_reg_tab_info_ptr->reg_count, g_is_main_sensor);
	SENSOR_PRINT("SENSOR use new time sec: %ld, usec: %ld.\n", time1.tv_sec, time1.tv_usec);
	SENSOR_PRINT("SENSOR use old time sec: %ld, usec: %ld.\n", time2.tv_sec, time2.tv_usec);
    
	SENSOR_PRINT("SENSOR: Sensor_SendRegTabToSensor X.\n");

	return SENSOR_SUCCESS;
}

//------ To Digital Camera Module

/*****************************************************************************/
//  Description:    This function is used to reset sensor    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL void _Sensor_CleanInformation(void)
{
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

	s_sensor_info_ptr=PNULL;
	s_sensor_init=SENSOR_FALSE;

	SENSOR_MEMSET(&s_sensor_exp_info, 0x00, sizeof(s_sensor_exp_info));

	sensor_register_info_ptr->cur_id=SENSOR_ID_MAX; 

	return ;
}

/*****************************************************************************/
//  Description:    This function is used to set currect sensor id    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL int _Sensor_SetId(SENSOR_ID_E sensor_id)
{
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

	sensor_register_info_ptr->cur_id=sensor_id;
	
	SENSOR_PRINT_HIGH("_Sensor_SetId:sensor_id=%d,g_is_register_sensor=%d,g_is_main_sensor=%d \n",
		                                     sensor_id,g_is_register_sensor,g_is_main_sensor);
	
	if(1 == g_is_register_sensor)
	{
		if((SENSOR_MAIN == sensor_id) && (1 == g_is_main_sensor))
			return SENSOR_SUCCESS;
		if((SENSOR_SUB == sensor_id) && (0 == g_is_main_sensor))
			return SENSOR_SUCCESS;
	}
	if((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id))
	{
		if(SENSOR_SUB == sensor_id)
		{
			/*sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_main_id;
			sensor_i2c_driver.address_data = &sensor_main_addr_data;*/
			if((1== g_is_register_sensor) && (1 == g_is_main_sensor))
			{
				sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_main_id;
				sensor_i2c_driver.address_data = &sensor_main_addr_data;
				i2c_del_driver(&sensor_i2c_driver);
			}
			g_is_main_sensor = 0;		
			sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_data = &sensor_sub_addr_data;
	    	}
	    	else  if(SENSOR_MAIN == sensor_id)
	    	{
	    		/*sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_data = &sensor_sub_addr_data;   */  
			if((1== g_is_register_sensor) && (0 == g_is_main_sensor))
			{
		    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_sub_id;
				sensor_i2c_driver.address_data = &sensor_sub_addr_data; 
				i2c_del_driver(&sensor_i2c_driver);
			}
			g_is_main_sensor = 1;	
			sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_main_id;
			sensor_i2c_driver.address_data = &sensor_main_addr_data;
			
	    	}
		
	 	if(i2c_add_driver(&sensor_i2c_driver))
		{
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver error\n");
			msleep(20);
			return SENSOR_FAIL;
		}  
		else
		{
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver OK.\n");
			g_is_register_sensor = 1;
		}
	}
	msleep(20);
	return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to get currect sensor id
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_ID_E Sensor_GetCurId(void)
{
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

     SENSOR_PRINT("Sensor_GetCurId,sensor_id =%d",sensor_register_info_ptr->cur_id);	

    return (SENSOR_ID_E)sensor_register_info_ptr->cur_id;
}

/*****************************************************************************/
//  Description:    This function is used to set currect sensor id and set sensor
//                  information
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetCurId(SENSOR_ID_E sensor_id){	
	SENSOR_PRINT("Sensor_SetCurId : %d.\n", sensor_id);
	if(sensor_id >= SENSOR_ID_MAX){
        	_Sensor_CleanInformation();
	        return SENSOR_FAIL;
	}	
	if(SENSOR_SUCCESS != _Sensor_SetId(sensor_id)){
		SENSOR_PRINT("SENSOR: Fail to Sensor_SetCurId.\n");
		return SENSOR_FAIL;
	}
	return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to get info of register sensor     
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_REGISTER_INFO_T_PTR Sensor_GetRegisterInfo(void)
{
    return s_sensor_register_info_ptr;
}
/*****************************************************************************/
//  Description:    This function is used to set currect sensor id    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL int  _Sensor_I2CInit(SENSOR_ID_E sensor_id)
{	
          SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;
	sensor_register_info_ptr->cur_id=sensor_id;	

	if(0 == g_is_register_sensor)
	{
		if((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id))
		{		
			if(SENSOR_MAIN == sensor_id)
			{			
				SENSOR_PRINT("_Sensor_I2CInit,sensor_main_force[1] =%d \n",sensor_main_force[1] );
				sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_main_id;
				sensor_i2c_driver.address_data = &sensor_main_addr_data;
					
		    	}
		    	else  if(SENSOR_SUB == sensor_id)
		    	{		    
				SENSOR_PRINT("_Sensor_I2CInit,sensor_sub_force[1] =%d \n",sensor_sub_force[1] );
		    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_sub_id;
				sensor_i2c_driver.address_data = &sensor_sub_addr_data;     				
		    	}

		 	if(i2c_add_driver(&sensor_i2c_driver))
			{
				SENSOR_PRINT_ERR("SENSOR: add I2C driver error\n");
				return SENSOR_FAIL;
			}  
			else
			{
				SENSOR_PRINT_ERR("SENSOR: add I2C driver OK.\n");	
				g_is_register_sensor = 1;	
			}
		}
	}
	else
	{
		SENSOR_PRINT_ERR("Sensor: Init I2c  %d fail!\n",sensor_id);
	}
	SENSOR_PRINT_ERR("_Sensor_I2CInit,sensor_id=%d,g_is_register_sensor=%d\n",sensor_id,g_is_register_sensor);
	return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to set currect sensor id    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL int _Sensor_I2CDeInit(SENSOR_ID_E sensor_id)
{	
	if(1 == g_is_register_sensor)
	{
		if((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id))
		{
			if(SENSOR_MAIN == sensor_id)
			{
				sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_main_id;
				sensor_i2c_driver.address_data = &sensor_main_addr_data;

				i2c_del_driver(&sensor_i2c_driver);		
				g_is_register_sensor = 0;
				SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n",sensor_id);
		    	}
		    	else  if(SENSOR_SUB == sensor_id)
		    	{
		    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_sub_id;
				sensor_i2c_driver.address_data = &sensor_sub_addr_data;     		
				i2c_del_driver(&sensor_i2c_driver);	
				g_is_register_sensor = 0;
				SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n",sensor_id);
		    	}	 	
		}
	}
	else
	{
		SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n",SENSOR_ID_MAX);
	}
	return SENSOR_SUCCESS;
}

/*****************************************************************************
//  Description:    This function is handle the power up issue of appointed sensor    
//  Author:         xi.zhang
//  Global;         SENSOR_INFO_T* s_sensor_info_ptr
                    SENSOR_INFO_T* s_sensor_list_ptr
                    SENSOR_REGISTER_INFO_T_PTR s_sensor_register_info_ptr
//  Note:           
*****************************************************************************/
LOCAL int _Sensor_Identify(SENSOR_CONFIG_T *sensor_config)
{
	uint32_t sensor_index = 0;
	SENSOR_INFO_T** sensor_info_tab_ptr = PNULL;
	uint32_t valid_tab_index_max = 0x00;
	SENSOR_INFO_T* sensor_info_ptr=PNULL;

	SENSOR_PRINT_HIGH("SENSOR: sensor identifing %d, set: %d, sensor_num: %d.\n", sensor_config->sensor_id, sensor_config->set_sensor_num, sensor_config->sensor_num);

	if(1 == sensor_config->set_sensor_num)
	{
		uint32_t max_num = Sensor_GetInforTabLenght(sensor_config->sensor_id)-SENSOR_ONE_I2C;
                if(sensor_config->sensor_num > max_num){
                        SENSOR_PRINT_HIGH("SENSOR: fail to HAL set sensor number. sensor_num: %d, max_num: %d.\n", sensor_config->sensor_num, max_num);
                        return SENSOR_FAIL;
                }
		sensor_info_tab_ptr = (SENSOR_INFO_T**)Sensor_GetInforTab(sensor_config->sensor_id);
		if((NULL != sensor_info_tab_ptr) && (NULL != sensor_info_tab_ptr[sensor_config->sensor_num]))
		{	
			s_sensor_list_ptr[sensor_config->sensor_id] = sensor_info_tab_ptr[sensor_config->sensor_num];
			s_sensor_info_ptr = sensor_info_tab_ptr[sensor_config->sensor_num];	
			s_sensor_register_info_ptr->is_register[sensor_config->sensor_id] = SCI_TRUE;
			//s_sensor_register_info_ptr->img_sensor_num = 1;//sensor_config->sensor_num;
			//this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
			SENSOR_PRINT("SENSOR: HAL set sensor number.\n");
		}
		else
		{
			SENSOR_PRINT_HIGH("SENSOR: Fail : sensor_info_tab_ptr or sensor_info_tab_ptr[sensor_config->sensor_num] is null.\n");
			return SENSOR_FAIL;
		}
		return SENSOR_SUCCESS;
	}
	//SENSOR_PRINT_HIGH("SENSOR: sensor identifing for test.\n");
	//return;//for test

	//if already identified
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_config->sensor_id])
	{
		SENSOR_PRINT("SENSOR: sensor identified.\n");
		return SENSOR_SUCCESS;
	}	    

	sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(sensor_config->sensor_id);
	valid_tab_index_max=Sensor_GetInforTabLenght(sensor_config->sensor_id)-SENSOR_ONE_I2C;
	_Sensor_I2CInit(sensor_config->sensor_id);
		  
          //search the sensor in the table
	for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
	{  
		sensor_info_ptr = sensor_info_tab_ptr[sensor_index];

		if(NULL==sensor_info_ptr)
		{
			SENSOR_PRINT_ERR("SENSOR: %d info of Sensor_Init table %d is null.\n", sensor_index, (uint)sensor_config->sensor_id);
			continue ;
		}
		s_sensor_info_ptr = sensor_info_ptr;		
		Sensor_PowerOn(SCI_TRUE);

		if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
		{
			ImgSensor_GetMutex();		
			
			this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
			
			if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
			{			         
				s_sensor_list_ptr[sensor_config->sensor_id]=sensor_info_ptr; 
				s_sensor_register_info_ptr->is_register[sensor_config->sensor_id]=SCI_TRUE;
				s_sensor_register_info_ptr->img_sensor_num++;
				ImgSensor_PutMutex();
				Sensor_PowerOn(SCI_FALSE);			
				SENSOR_PRINT_HIGH("_Sensor_Identify:sensor_id :%d,img_sensor_num=%d, sensor_index: %d.\n",
					                                     sensor_config->sensor_id,s_sensor_register_info_ptr->img_sensor_num, sensor_index);
				break ;
			}
			ImgSensor_PutMutex();
		}
		Sensor_PowerOn(SCI_FALSE);	
	}
        _Sensor_I2CDeInit(sensor_config->sensor_id);
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_config->sensor_id])
	{
		if(2 == sensor_config->set_sensor_num){
			sensor_config->sensor_num = sensor_index--;//generate the new sensor number.
			printk("new sensor_num: %d, img_sensor_num: %d.\n", sensor_config->sensor_num, s_sensor_register_info_ptr->img_sensor_num);
		}
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify OK.\n",(uint32_t)sensor_config->sensor_id);
	}
	else
	{
		if(2 == sensor_config->set_sensor_num){
			sensor_config->sensor_num = 0xEF;
			//sensor_config->set_sensor_num = 2;//no generate the new sensor number. all sensors are not ok.
		}
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify FAILURE.\n",(uint32_t)sensor_config->sensor_id);
	}
	
	return SENSOR_SUCCESS;
}

LOCAL void _Sensor_SetStatus(SENSOR_ID_E sensor_id)
{
	uint32_t i = 0;
	//uint32_t ret_val=SENSOR_FAIL;   
    	//SENSOR_INFO_T* sensor_info_ptr=PNULL;
    	//SENSOR_INFO_T** sensor_info_tab_ptr=PNULL;   
    	//uint8_t sensor_index = 0x0;
    	//uint32_t valid_tab_index_max=0x00;
    	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

	for( i=0 ; i<=SENSOR_SUB ; i++)
	{
		if(i == sensor_id)
		{
			continue;
		}
		if(SENSOR_TRUE == sensor_register_info_ptr->is_register[i])
		{
	   		_Sensor_SetId(i);
			
			 s_sensor_info_ptr=s_sensor_list_ptr[i];

			this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 

			Sensor_PowerOn(SENSOR_TRUE);

			Sensor_SetExportInfo(&s_sensor_exp_info);

			Sensor_PowerDown((BOOLEAN)s_sensor_info_ptr->power_down_level);

			SENSOR_PRINT_HIGH("SENSOR: Sensor_sleep of id %d",i);
		}
	}
		
}


/*****************************************************************************/
//  Description:    This function is used to initialize Sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_Init(SENSOR_CONFIG_T *sensor_config)
{    	
    	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

    	SENSOR_PRINT("SENSOR: Sensor_Init, sensor_id: %d, set: %d, num: %d.\n", sensor_config->sensor_id, sensor_config->set_sensor_num, sensor_config->sensor_num);

   	if(Sensor_IsInit())
    	{
       	 	SENSOR_PRINT("SENSOR: Sensor_Init is done\n");        
       		return SENSOR_SUCCESS;
    	}

    	_Sensor_CleanInformation();
	
	if(0 != sensor_config->set_sensor_num){//wxz20111111: 1: set id; 2: get new id.
		if(SENSOR_SUCCESS != _Sensor_Identify(sensor_config)){
			SENSOR_PRINT_HIGH("Fail to _Sensor_Identify. set_sensor_num: %d.\n", sensor_config->set_sensor_num);
			return SENSOR_FAIL;
		}
		SENSOR_PRINT("SENSOR: Sensor_Init Identify, HAL set sensor num.\n");
	}
	else{
		SENSOR_CONFIG_T sensor_config_tmp;
		sensor_config_tmp.sensor_id = SENSOR_MAIN;
		sensor_config_tmp.set_sensor_num = 0;
		if(SENSOR_SUCCESS != _Sensor_Identify(&sensor_config_tmp)){
			SENSOR_PRINT_HIGH("Fail to _Sensor_Identify for main sensor. set_sensor_num: %d.\n", sensor_config->set_sensor_num);
			return SENSOR_FAIL;
		}
		msleep(20);		
		sensor_config_tmp.sensor_id = SENSOR_SUB;
		sensor_config_tmp.set_sensor_num = 0;
		if(SENSOR_SUCCESS != _Sensor_Identify(&sensor_config_tmp)){
			SENSOR_PRINT_HIGH("Fail to _Sensor_Identify for sub sensor. set_sensor_num: %d.\n", sensor_config->set_sensor_num);
			return SENSOR_FAIL;
		}
		SENSOR_PRINT("SENSOR: Sensor_Init Identify \n");
	}		
	SENSOR_PRINT("SENSOR: sensor_id: %d, set: %d, num: %d.\n", sensor_config->sensor_id, sensor_config->set_sensor_num, sensor_config->sensor_num);
	if(SENSOR_TRUE == sensor_register_info_ptr->is_register[sensor_config->sensor_id])
	{
		if(0 == sensor_config->set_sensor_num){
			msleep(20);
			_Sensor_SetStatus(sensor_config->sensor_id);		
		}
        	_Sensor_SetId(sensor_config->sensor_id);
		s_sensor_info_ptr = s_sensor_list_ptr[sensor_config->sensor_id];
		Sensor_SetExportInfo(&s_sensor_exp_info);
		s_sensor_init = SENSOR_TRUE;
		Sensor_PowerOn(SENSOR_TRUE);
		this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
		SENSOR_PRINT("Sensor_Init:sensor_id :%d,addr=0x%x\n",sensor_config->sensor_id,this_client->addr);		
		if(1 == sensor_config->set_sensor_num){
			if(PNULL != s_sensor_info_ptr->ioctl_func_tab_ptr->identify){
	            		if(SENSOR_SUCCESS != s_sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C)){
					sensor_register_info_ptr->is_register[sensor_config->sensor_id] = SENSOR_FALSE;
					Sensor_PowerOn(SCI_FALSE);
					s_sensor_init = SENSOR_FALSE;
					SENSOR_PRINT("Sensor_Init:fail to identity, sensor_id :%d,addr=0x%x\n",sensor_config->sensor_id,this_client->addr);
					return SENSOR_FAIL;
				}
			}
		}	
		if(SENSOR_SUCCESS != Sensor_SetMode(SENSOR_MODE_COMMON_INIT))
		{
			SENSOR_PRINT_ERR("Sensor set init mode error!\n");
			return SENSOR_FAIL;
		}	
		s_sensor_init = SENSOR_TRUE;
		SENSOR_PRINT("SENSOR: Sensor_Init  Success \n");					
	}
	else
	{
		SENSOR_PRINT_ERR("Sensor identify fail,sensor_id = %d",sensor_config->sensor_id);
	}      
	
	return SENSOR_SUCCESS;	
}	

/*****************************************************************************/
//  Description:    This function is used to check if sensor has been init    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_IsInit(void)
{
	return s_sensor_init;
}

/*****************************************************************************/
//  Description:    This function is used to set sensor work-mode    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SetMode(SENSOR_MODE_E mode)
{
    	uint32_t mclk;

        SENSOR_PRINT("SENSOR: Sensor_SetMode -> mode = %d.\n", mode);        
        if(SENSOR_FALSE == Sensor_IsInit()){	          
		SENSOR_PRINT("SENSOR: Sensor_SetResolution -> sensor has not init");
            	return SENSOR_OP_STATUS_ERR;
        }	
        
        if(s_sensor_mode[Sensor_GetCurId()] == mode){
            	SENSOR_PRINT("SENSOR: The sensor mode as before");	 
            	return SENSOR_SUCCESS;
        }

        if(PNULL != s_sensor_info_ptr->resolution_tab_info_ptr[mode].sensor_reg_tab_ptr){		
            // set mclk
            mclk = s_sensor_info_ptr->resolution_tab_info_ptr[mode].xclk_to_sensor;		
            Sensor_SetMCLK(mclk);

            // set image format
            s_sensor_exp_info.image_format = s_sensor_exp_info.sensor_mode_info[mode].image_format;

            // send register value to sensor
            Sensor_SendRegTabToSensor(&s_sensor_info_ptr->resolution_tab_info_ptr[mode]);

            s_sensor_mode[Sensor_GetCurId()]=mode;
        }
        else{
            SENSOR_PRINT("SENSOR: Sensor_SetResolution -> No this resolution information !!!");
        }

	return SENSOR_SUCCESS;
}
/*****************************************************************************/
//  Description:    This function is used to control sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_Ioctl(uint32_t cmd, uint32_t arg)
{
    SENSOR_IOCTL_FUNC_PTR func_ptr;	
    SENSOR_IOCTL_FUNC_TAB_T* func_tab_ptr;	
    uint32_t temp;
    uint32_t ret_value = SENSOR_SUCCESS;

    SENSOR_PRINT("SENSOR: Sensor_Ioctl -> cmd = %d, arg = %d.\n", cmd, arg);

  //  SENSOR_ASSERT (cmd <= SENSOR_IOCTL_MAX);

    if(!Sensor_IsInit())
    {
        SENSOR_PRINT("SENSOR: Sensor_Ioctl -> sensor has not init.\n");
        return SENSOR_OP_STATUS_ERR;
    }

    if(SENSOR_IOCTL_CUS_FUNC_1 > cmd) 
    {
        SENSOR_PRINT("SENSOR: Sensor_Ioctl - > can't access internal command !\n");
        return SENSOR_SUCCESS;	
    }
    func_tab_ptr = s_sensor_info_ptr->ioctl_func_tab_ptr;

    temp = *(uint32_t*)((uint32_t)func_tab_ptr + cmd * BIT_2);

    func_ptr = (SENSOR_IOCTL_FUNC_PTR)temp;

    if(PNULL!= func_ptr)
    {
	ImgSensor_GetMutex();
        ret_value = func_ptr(arg);
        ImgSensor_PutMutex();
    }
    else
    {
        SENSOR_PRINT("SENSOR: Sensor_Ioctl -> the ioctl function has not register err!\n");
    }

    return ret_value;	
    
}

/*****************************************************************************/
//  Description:    This function is used to Get sensor information    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_EXP_INFO_T* Sensor_GetInfo( void )
{
    if(!Sensor_IsInit())
    {
        SENSOR_PRINT("SENSOR: Sensor_GetInfo -> sensor has not init");
        return PNULL;
    }

    return &s_sensor_exp_info;
}

/*****************************************************************************/
//  Description:    This function is used to Close sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_Close(void) 
{
	SENSOR_PRINT("SENSOR: Sensor_close");

	if(1== g_is_register_sensor) 
	{
		if (1 == g_is_main_sensor)
		{
			//sensor_i2c_driver.id_table = sensor_main_id;
			sensor_i2c_driver.address_data = &sensor_main_addr_data;		
		}
		else 
		{
			//sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_data = &sensor_sub_addr_data;    
		}   	
			i2c_del_driver(&sensor_i2c_driver);
			g_is_register_sensor = 0;
			g_is_main_sensor = 0;
	}

	if(SENSOR_TRUE == Sensor_IsInit())
	{
		Sensor_PowerOn(SENSOR_FALSE);       
	}

	s_sensor_init = SENSOR_FALSE;   
	s_sensor_mode[SENSOR_MAIN]=SENSOR_MODE_MAX;	
	s_sensor_mode[SENSOR_SUB]=SENSOR_MODE_MAX;	
	return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to set sensor type (img sensor or atv)   
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetSensorType(SENSOR_TYPE_E sensor_type)
{
	s_sensor_type=sensor_type;

	return SENSOR_SUCCESS;
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
    
#endif  // End of sensor_drv.c
