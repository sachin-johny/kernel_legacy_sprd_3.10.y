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
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/sensor_drv.h>
#include <mach/sensor_cfg.h>
#include <mach/adi_hal_internal.h>
#include <linux/dcam_sensor.h>
#include <mach/clock_common.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/ldo.h>
#include <mach/sensor_drv.h>

#include "dcam_common.h"

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
#define SENSOR_MCLK_SRC_NUM   4
#define SENSOR_MCLK_DIV_MAX     4
#define NUMBER_MAX                         0x7FFFFFF
#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef struct SN_MCLK
{
    int     clock;
    char *src_name;
}SN_MCLK;

const SN_MCLK sensor_mclk_tab[SENSOR_MCLK_SRC_NUM] = 
{
    {96, "clk_96m"},
    {77,"clk_76m800k"},
    {48, "clk_48m"},
    {26, "ext_26m"}    
};


/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL SENSOR_INFO_T* s_sensor_list_ptr[SENSOR_ID_MAX];
LOCAL SENSOR_INFO_T* s_sensor_info_ptr=PNULL;
LOCAL SENSOR_EXP_INFO_T s_sensor_exp_info;
LOCAL uint32_t s_sensor_mclk=0;
LOCAL BOOLEAN s_sensor_init=SENSOR_FALSE;  
LOCAL SENSOR_TYPE_E s_sensor_type=SENSOR_TYPE_NONE;
LOCAL SENSOR_MODE_E s_sensor_mode[SENSOR_ID_MAX]={SENSOR_MODE_MAX,SENSOR_MODE_MAX,SENSOR_MODE_MAX};
LOCAL SENSOR_MUTEX_PTR	s_imgsensor_mutex_ptr=PNULL;
LOCAL SENSOR_REGISTER_INFO_T s_sensor_register_info={0x00};
LOCAL SENSOR_REGISTER_INFO_T_PTR s_sensor_register_info_ptr=&s_sensor_register_info;
struct clk *s_ccir_clk = NULL;//for power manager
struct clk *s_ccir_enable_clk = NULL;//for power manager
LOCAL uint32_t s_flash_mode = 0xff;

static struct i2c_client *this_client = NULL;
static int g_is_main_sensor = 0;
static int g_is_register_sensor = 0;
#define SENSOR_DEV_NAME	SENSOR_MAIN_I2C_NAME 

static const struct i2c_device_id sensor_main_id[] = {
	{ SENSOR_MAIN_I2C_NAME, 0 },
	{ }
};
static const struct i2c_device_id sensor_sub_id[] = {
	{ SENSOR_SUB_I2C_NAME, 0 },
	{ }
};

static unsigned short sensor_main_force[] = {2, SENSOR_MAIN_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const sensor_main_forces[] = { sensor_main_force, NULL };
static unsigned short sensor_main_default_addr_list[] ={ SENSOR_MAIN_I2C_ADDR,I2C_CLIENT_END};
static unsigned short sensor_sub_force[] = {2, SENSOR_SUB_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const sensor_sub_forces[] = { sensor_sub_force, NULL };
static unsigned short sensor_sub_default_addr_list[] ={ SENSOR_SUB_I2C_ADDR,I2C_CLIENT_END};

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
	printk( KERN_INFO "SENSOR:sensor_probe E.\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO  "SENSOR: %s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
	}	
	this_client = client;	
	if(SENSOR_MAIN == Sensor_GetCurId()){
		if(SENSOR_MAIN_I2C_ADDR != (this_client->addr & (~0xFF))) {
			this_client->addr = (this_client->addr & (~0xFF)) | (sensor_main_force[1]&0xFF); 
		}
	}
	else{ //for SENSOR_SUB	
		if(SENSOR_SUB_I2C_ADDR != (this_client->addr & (~0xFF))) {
			this_client->addr = (this_client->addr & (~0xFF)) | (sensor_sub_force[1]&0xFF); 
		}
	}
	printk( KERN_INFO "sensor_probe,this_client->addr =0x%x\n",this_client->addr );
	mdelay(20);
	
	return 0;
out:
	return res;
}
static int sensor_remove(struct i2c_client *client)
{	
	return 0;
}
static int sensor_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    	
         printk("SENSOR_DRV: detect!");
	strcpy(info->type, client->name);	
	return 0;
}

static struct i2c_driver sensor_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,   
    },
	.probe      = sensor_probe,
	.remove     = sensor_remove,
	.detect     = sensor_detect,	
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
	
	if(SENSOR_NULL==s_imgsensor_mutex_ptr)
	{
		return ;
	}
    
	ret=SENSOR_DeleteMutex(s_imgsensor_mutex_ptr);	
	SENSOR_ASSERT(ret==SENSOR_SUCCESS );	
	s_imgsensor_mutex_ptr=SENSOR_NULL;	
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
	if(SENSOR_NULL==s_imgsensor_mutex_ptr)
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
	int err = 0xff;
	SENSOR_IOCTL_FUNC_PTR	reset_func;
	printk("Sensor_Reset");

	reset_func = s_sensor_info_ptr->ioctl_func_tab_ptr->reset;

	if(PNULL != reset_func)
	{
		reset_func(0);
	}
	else
	{		
		err = gpio_request(72,"ccirrst");
		if (err) {
			printk("Sensor_Reset failed requesting err=%d\n", err);
			return ;
		}

		gpio_direction_output(72,level);	
		
		//gpio_set_value(72, !level);
		//msleep(20);
		gpio_set_value(72,level);
		msleep(40);
		gpio_set_value(72,!level);		
	         msleep(20);
		gpio_free(72);
	}
}

LOCAL int select_sensor_mclk(uint8_t clk_set, char** clk_src_name,uint8_t* clk_div)
{
	uint8_t   i,j,mark_src = 0,mark_div = 0,mark_src_tmp = 0;
	int           clk_tmp,src_delta,src_delta_min = NUMBER_MAX;
	int           div_delta,div_delta_min = NUMBER_MAX;

	printk("SENSOR:select_sensor_mclk,clk_set=%d.\n",clk_set);
	if(clk_set > 96 || !clk_src_name || !clk_div)
	{
		return SENSOR_FAIL;
	}

	for(i = 0; i < SENSOR_MCLK_DIV_MAX; i++ )
	{
		clk_tmp =(int)(clk_set * (i + 1));
		src_delta_min = NUMBER_MAX;
		for(j = 0; j < SENSOR_MCLK_SRC_NUM; j++ )
		{
			src_delta =    ABS(sensor_mclk_tab[j].clock - clk_tmp);
			if(src_delta < src_delta_min)
			{
				src_delta_min = src_delta;
				mark_src_tmp = j;
			}
		}

		if(src_delta_min <  div_delta_min)
		{
			div_delta_min = src_delta_min;
			mark_src = mark_src_tmp;
			mark_div = i;
		}
	}

	printk("SENSOR:select_sensor_mclk,clk_src=%d,clk_div=%d .\n",mark_src,mark_div);

	*clk_src_name = sensor_mclk_tab[mark_src].src_name;
	*clk_div = mark_div+1;
	return SENSOR_SUCCESS;
}
/*****************************************************************************/
//  Description:    This function is used to power on sensor and select xclk    
//  Author:         Liangwen.Zhen
//  Note:           1.Unit: MHz 2. if mclk equal 0, close main clock to sensor
/*****************************************************************************/
int Sensor_SetMCLK(uint32_t mclk)
{	
	struct clk *clk_parent = NULL;
	int ret;    
	uint8_t clk_div;
	char *clk_src_name=NULL;

	SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %d MHz, clk = %d MHz\n", s_sensor_mclk, mclk);

	if((0 != mclk) && (s_sensor_mclk != mclk))
	{
		if(s_ccir_clk)
		{
			clk_disable(s_ccir_clk);		
			SENSOR_PRINT("###sensor s_ccir_clk clk_disable ok.\n");
		}
		else
		{
		    	s_ccir_clk = clk_get(NULL, "ccir_mclk");
			if(IS_ERR(s_ccir_clk))
			{
				SENSOR_PRINT_ERR("###: Failed: Can't get clock [ccir_mclk]!\n");
				SENSOR_PRINT_ERR("###: s_sensor_clk = %p.\n", s_ccir_clk);
			}
			else
			{
				SENSOR_PRINT("###sensor s_ccir_clk clk_get ok.\n");
			}	
		}
		if(mclk > SENSOR_MAX_MCLK)
		{
			mclk = SENSOR_MAX_MCLK;
		}
		if(SENSOR_SUCCESS != select_sensor_mclk((uint8_t)mclk,&clk_src_name,&clk_div))
		{
			printk("SENSOR:Sensor_SetMCLK select clock source fail.\n");
			return -EINVAL;
		}
		
		clk_parent = clk_get(NULL, clk_src_name);
		if(!clk_parent)
		{
			SENSOR_PRINT_ERR("###:clock[%s]: failed to get parent [%s] by clk_get()!\n", s_ccir_clk->name, clk_src_name);
			return -EINVAL;
		}

		ret = clk_set_parent(s_ccir_clk, clk_parent);
		if(ret)
		{
			SENSOR_PRINT_ERR("###:clock[%s]: clk_set_parent() failed!parent: %s, usecount: %d.\n", s_ccir_clk->name, clk_parent->name, s_ccir_clk->usecount);
			return -EINVAL;
		}
		
		ret = clk_set_divisor(s_ccir_clk, clk_div);
		if(ret)
		{
			SENSOR_PRINT_ERR("###:clock[%s]: clk_set_divisor failed!\n", s_ccir_clk->name);
			return -EINVAL;
		}
		ret = clk_enable(s_ccir_clk);
		if(ret)
		{
			SENSOR_PRINT_ERR("###:clock[%s]: clk_enable() failed!\n", s_ccir_clk->name);
		}
		else
		{
			SENSOR_PRINT("###sensor s_ccir_clk clk_enable ok.\n");
		}
	 
	         // CCIR CLK Enable
		if(NULL == s_ccir_enable_clk)
		{
		    	s_ccir_enable_clk = clk_get(NULL, "clk_ccir");
			if(IS_ERR(s_ccir_enable_clk))
			{
				SENSOR_PRINT_ERR("###: Failed: Can't get clock [clk_ccir]!\n");
				SENSOR_PRINT_ERR("###: s_ccir_enable_clk = %p.\n", s_ccir_enable_clk);
				return -EINVAL;
			} 
			else
			{
				SENSOR_PRINT("###sensor s_ccir_enable_clk clk_get ok.\n");
			}	
			ret = clk_enable(s_ccir_enable_clk);
			if(ret)
			{
				SENSOR_PRINT_ERR("###:clock[%s]: clk_enable() failed!\n", s_ccir_enable_clk->name);
			}
			else
			{
				SENSOR_PRINT("###sensor s_ccir_enable_clk clk_enable ok.\n");
			}
		}	
	    
		s_sensor_mclk = mclk;
		SENSOR_PRINT("SENSOR: Sensor_SetMCLK -> s_sensor_mclk = %d Hz, divd = %d\n", s_sensor_mclk, clk_div);
	}
	else if(0 == mclk)
	{ 
		if(s_ccir_clk)
		{
			clk_disable(s_ccir_clk);
			SENSOR_PRINT("###sensor s_ccir_clk clk_disable ok.\n");
			clk_put(s_ccir_clk);		
			SENSOR_PRINT("###sensor s_ccir_clk clk_put ok.\n");
			s_ccir_clk = NULL;
		}
		// CCIR CLK disable
		if(s_ccir_enable_clk)
		{
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

/*****************************************************************************/
//  Description:    This function is used to set AVDD
//  Author:         Liangwen.Zhen
//  Note:           Open AVDD on one special voltage or Close it
/*****************************************************************************/
PUBLIC void Sensor_SetVoltage(SENSOR_AVDD_VAL_E dvdd_val, SENSOR_AVDD_VAL_E avdd_val, SENSOR_AVDD_VAL_E iodd_val)
{
	uint32_t ldo_volt_level = LDO_VOLT_LEVEL0;

	switch(iodd_val)
	{            
		case SENSOR_AVDD_2800MV:
			ldo_volt_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_3800MV: 
			ldo_volt_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_1800MV:
			ldo_volt_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1200MV:
			ldo_volt_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_volt_level = SENSOR_AVDD_CLOSED;   
			break;
	} 
	if(SENSOR_AVDD_CLOSED == ldo_volt_level)
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off camd1.\n");        
		LDO_TurnOffLDO(LDO_LDO_CAMD1);            
	}
	else
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on avdd, VoltLevel %d\n",ldo_volt_level); 
		LDO_SetVoltLevel(LDO_LDO_CAMD1, ldo_volt_level);      
		LDO_TurnOnLDO(LDO_LDO_CAMD1);  
	}

	switch(avdd_val)
	{            
		case SENSOR_AVDD_2800MV:
			ldo_volt_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_3000MV:
			ldo_volt_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_2500MV:
			ldo_volt_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1800MV:
			ldo_volt_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_volt_level = SENSOR_AVDD_CLOSED;   
			break;
	} 
   
	if(SENSOR_AVDD_CLOSED == ldo_volt_level)
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off avdd\n"); 
		LDO_TurnOffLDO(LDO_LDO_CAMA);        
        
	}
	else
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on avdd, VoltLevel %d\n",ldo_volt_level); 
		LDO_SetVoltLevel(LDO_LDO_CAMA, ldo_volt_level);      
		LDO_TurnOnLDO(LDO_LDO_CAMA);  	
	}

	switch(dvdd_val)
	{
		case SENSOR_AVDD_1800MV:
			ldo_volt_level = LDO_VOLT_LEVEL0;    
			break;

		case SENSOR_AVDD_2800MV:
			ldo_volt_level = LDO_VOLT_LEVEL1;    
			break;

		case SENSOR_AVDD_1500MV:
			ldo_volt_level = LDO_VOLT_LEVEL2;    
			break;

		case SENSOR_AVDD_1300MV:
			ldo_volt_level = LDO_VOLT_LEVEL3;    
			break;  

		case SENSOR_AVDD_CLOSED:
		case SENSOR_AVDD_UNUSED:
		default:
			ldo_volt_level = SENSOR_AVDD_CLOSED;           
			break;
	} 
    
	if(SENSOR_AVDD_CLOSED == ldo_volt_level)
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off dvdd, sensor_id %d\n",Sensor_GetCurId()); 
		LDO_TurnOffLDO(LDO_LDO_CAMD0);
	}
	else
	{
		SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on dvdd, sensor_id %d, VoltLevel %d\n",
		Sensor_GetCurId(),ldo_volt_level); 
		LDO_TurnOnLDO(LDO_LDO_CAMD0);    
		LDO_SetVoltLevel(LDO_LDO_CAMD0, ldo_volt_level);      
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
	    // call user func
	if(PNULL != power_func)
	{
		power_func(power_on);
	}  
	else
	{
		if(power_on)
		{			
			Sensor_PowerDown(power_down);	
			// Open power
			Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val);    
			msleep(10);
			Sensor_SetMCLK(SENSOR_DEFALUT_MCLK); 
			msleep(5);
			Sensor_PowerDown(!power_down);
			// Reset sensor		
			Sensor_Reset(rst_lvl);						
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
}


/*****************************************************************************/
//  Description:    This function is used to power down sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_PowerDown(BOOLEAN power_level)
{
	SENSOR_IOCTL_FUNC_PTR	entersleep_func=s_sensor_info_ptr->ioctl_func_tab_ptr->enter_sleep;
	
	SENSOR_PRINT("SENSOR: Sensor_PowerDown -> main: power_down %d\n", power_level);          
	SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD1-> 0x8C000344 0x%x\n", _pard(PIN_CTL_CCIRPD1)); 
	SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD0-> 0x8C000348 0x%x\n", _pard(PIN_CTL_CCIRPD0));

	if(entersleep_func)
	{
		entersleep_func(power_level);
		 return SENSOR_SUCCESS;
	}

extern int sprd_3rdparty_gpio_main_camera_pwd;
extern int sprd_3rdparty_gpio_sub_camera_pwd;
	
	switch(Sensor_GetCurId())
	{
		case SENSOR_MAIN:
		{
			gpio_request(sprd_3rdparty_gpio_main_camera_pwd,"main camera");
			if(0 == power_level)
			{
				gpio_direction_output(sprd_3rdparty_gpio_main_camera_pwd,0);
				
			}
			else
			{
				gpio_direction_output(sprd_3rdparty_gpio_main_camera_pwd,1);
			}    
			gpio_free(sprd_3rdparty_gpio_main_camera_pwd);
			break;
		}
		case SENSOR_SUB:
		{
			gpio_request(sprd_3rdparty_gpio_sub_camera_pwd,"sub camera");
			if(0 == power_level)
			{
				gpio_direction_output(sprd_3rdparty_gpio_sub_camera_pwd,0);
			}
			else
			{
				gpio_direction_output(sprd_3rdparty_gpio_sub_camera_pwd,1);
			}
			gpio_free(sprd_3rdparty_gpio_sub_camera_pwd);
			break;
		}
		default :
			break;            
	}

    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to reset img sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_SetResetLevel(BOOLEAN plus_level)
{
	int err = 0xff;
	err = gpio_request(72,"ccirrst");
	if (err) {
		printk("Sensor_Reset failed requesting err=%d\n", err);
		return SENSOR_FAIL;
	}
	gpio_direction_output(72,plus_level);		

	gpio_set_value(72,plus_level);		
         msleep(100);
	gpio_free(72);
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

	exp_info_ptr->sensor_interface = sensor_info_ptr->sensor_interface;
		
}

/**---------------------------------------------------------------------------*
 **                         Function Definitions                              *
 **---------------------------------------------------------------------------*/

 
/*****************************************************************************/
//  Description:    This function is used to write value to sensor register    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
int32_t Sensor_WriteReg(uint16_t subaddr, uint16_t data)
{
	uint8_t cmd[4] = {0};
	uint32_t index=0, i=0;
	uint32_t cmd_num = 0;
	struct i2c_msg msg_w;
	int32_t ret = -1;
	SENSOR_IOCTL_FUNC_PTR 	write_reg_func;

//	SENSOR_PRINT("this_client->addr=0x%x\n",this_client->addr);
//	SENSOR_PRINT_ERR("Sensor_WriteReg:addr=0x%x,data=0x%x .\n",subaddr,data);

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
					 continue;////return -1;
				}
				else
				{
					//printk("SENSOR: IIC write reg OK! 0x%04x, val:0x%04x ", subaddr, data);
					ret = 0;
					break;
				}
			}           
		}
		else
		{
			msleep(data);
			//SENSOR_PRINT("SENSOR: IIC write Delay %d ms", data);	    	
		}
	}
	return 0;
}
/*****************************************************************************/
//  Description:    This function is used to read value to sensor register    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
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

	s_sensor_info_ptr = PNULL;
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
			sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_main_id;
			sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];
			if((1== g_is_register_sensor) && (1 == g_is_main_sensor))
			{
				i2c_del_driver(&sensor_i2c_driver);
			}
			g_is_main_sensor = 0;		
			sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0];
	        	}
	    	else  if(SENSOR_MAIN == sensor_id)
	    	{
	    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0];
			if((1== g_is_register_sensor) && (0 == g_is_main_sensor))
			{
				i2c_del_driver(&sensor_i2c_driver);
			}
			g_is_main_sensor = 1;	
			sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
			sensor_i2c_driver.id_table = sensor_main_id;
			sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];
			
	    	}
		
	 	if(i2c_add_driver(&sensor_i2c_driver))
		{
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver error\n");
		//	msleep(20);
			return SENSOR_FAIL;
		}  
		else
		{
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver OK.\n");
			g_is_register_sensor = 1;
		}
	}
//	msleep(20);
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
     printk("Sensor_GetCurId,sensor_id =%d",sensor_register_info_ptr->cur_id);	

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
LOCAL void  _Sensor_I2CInit(SENSOR_ID_E sensor_id)
{	
          SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;
	sensor_register_info_ptr->cur_id=sensor_id;	

	if(0 == g_is_register_sensor)
	{
		if((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id))
		{		
			if(SENSOR_MAIN == sensor_id)
			{			
				printk("_Sensor_I2CInit,sensor_main_force[1] =%d \n",sensor_main_force[1] );
				sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_main_id;
				sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];
					
		    	}
		    	else  if(SENSOR_SUB == sensor_id)
		    	{		    
				printk("_Sensor_I2CInit,sensor_sub_force[1] =%d \n",sensor_sub_force[1] );
		    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_sub_id;
				sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0];    				
		    	}

		 	if(i2c_add_driver(&sensor_i2c_driver))
			{
				SENSOR_PRINT_ERR("SENSOR: add I2C driver error\n");
				return;
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
				sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];

				i2c_del_driver(&sensor_i2c_driver);		
				g_is_register_sensor = 0;
				SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n",sensor_id);
		    	}
		    	else  if(SENSOR_SUB == sensor_id)
		    	{
		    		sensor_i2c_driver.driver.name = SENSOR_SUB_I2C_NAME;
				sensor_i2c_driver.id_table = sensor_sub_id;
				sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0];    		
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
LOCAL void _Sensor_Identify(SENSOR_ID_E sensor_id)
{
	uint32_t sensor_index = 0;
	SENSOR_INFO_T** sensor_info_tab_ptr = PNULL;
	uint32_t valid_tab_index_max = 0x00;
	SENSOR_INFO_T* sensor_info_ptr=PNULL;

	SENSOR_PRINT_HIGH("SENSOR: sensor identifing %d", sensor_id);

	//if already identified
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_id])
	{
		SENSOR_PRINT("SENSOR: sensor identified");
		return;
	}	    

	sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(sensor_id);
	valid_tab_index_max=Sensor_GetInforTabLenght(sensor_id)-SENSOR_ONE_I2C;
	_Sensor_I2CInit(sensor_id);

//	msleep(20);
		  
          //search the sensor in the table
	for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
	{  
		sensor_info_ptr = sensor_info_tab_ptr[sensor_index];

		if(NULL==sensor_info_ptr)
		{
			SENSOR_PRINT_ERR("SENSOR: %d info of Sensor_Init table %d is null", sensor_index, (uint)sensor_id);
			continue ;
		}
		s_sensor_info_ptr = sensor_info_ptr;		
		Sensor_PowerOn(SCI_TRUE);
		SENSOR_PRINT_ERR("SENSOR: Sensor_PowerOn done,this_client=0x%x\n",(uint32_t)this_client);
		SENSOR_PRINT_ERR("SENSOR: identify ptr =0x%x\n",(uint32_t)sensor_info_ptr->ioctl_func_tab_ptr->identify);
	//	msleep(20);
		if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
		{
			//ImgSensor_GetMutex();		
			printk("SENSOR:identify  Sensor 00:this_client=0x%x,this_client->addr=0x%x,0x%x\n",(uint32_t)this_client,(uint32_t)&this_client->addr,this_client->addr);
	//		msleep(100);
			if(5 != Sensor_GetCurId())//test by wang bonnie
				this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
			SENSOR_PRINT_ERR("SENSOR:identify  Sensor 01\n");
	//		msleep(100);
			if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
			{			         
				s_sensor_list_ptr[sensor_id]=sensor_info_ptr; 
				s_sensor_register_info_ptr->is_register[sensor_id]=SCI_TRUE;
				s_sensor_register_info_ptr->img_sensor_num++;
				ImgSensor_PutMutex();
				Sensor_PowerOn(SCI_FALSE);			
				SENSOR_PRINT_HIGH("_Sensor_Identify:sensor_id :%d,img_sensor_num=%d\n",
					                                     sensor_id,s_sensor_register_info_ptr->img_sensor_num);
				break ;
			}
		//	ImgSensor_PutMutex();
		}
		Sensor_PowerOn(SCI_FALSE);	
	}
         _Sensor_I2CDeInit(sensor_id);
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_id])
	{
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify OK",(uint32_t)sensor_id);
	}
	else
	{
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify FAILURE",(uint32_t)sensor_id);
	}

}

LOCAL void _Sensor_SetStatus(SENSOR_ID_E sensor_id)
{
	uint32_t i = 0;	  
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
         		if(5 != Sensor_GetCurId())//bonnie
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
PUBLIC uint32_t Sensor_Init(uint32_t sensor_id)
{
    	uint32_t ret_val=SENSOR_FAIL;    	
    	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

    	SENSOR_PRINT("SENSOR: Sensor_Init, sensor_id: %d.\n", sensor_id);

   	 if(Sensor_IsInit())
    	{
       	 	SENSOR_PRINT("SENSOR: Sensor_Init is done\n");        
       		 return SENSOR_SUCCESS;
    	}

    	_Sensor_CleanInformation();

	_Sensor_Identify(SENSOR_MAIN);

//	msleep(20);

	_Sensor_Identify(SENSOR_SUB);
	if(5 == sensor_id){
		msleep(20);  //Jed add them 20111110 bonnie
		_Sensor_Identify(SENSOR_ATV);
	}
	
	SENSOR_PRINT("SENSOR: Sensor_Init Identify \n");

	if(SENSOR_TRUE == sensor_register_info_ptr->is_register[sensor_id])
	{
//		msleep(20);
		_Sensor_SetStatus(sensor_id);		
        		_Sensor_SetId(sensor_id);
		s_sensor_info_ptr=s_sensor_list_ptr[sensor_id];
		Sensor_SetExportInfo(&s_sensor_exp_info);
		s_sensor_init = SENSOR_TRUE;
		Sensor_PowerOn(SENSOR_TRUE);

		if(5 != Sensor_GetCurId())//bonnie
			this_client->addr = (this_client->addr & (~0xFF)) | (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
		printk("Sensor_Init:sensor_id :%d,addr=0x%x\n",sensor_id,this_client->addr);
		ret_val=SENSOR_SUCCESS;		
		if(SENSOR_SUCCESS != Sensor_SetMode(SENSOR_MODE_COMMON_INIT))
		{
			SENSOR_PRINT_ERR("Sensor set init mode error!\n");
			ret_val=SENSOR_FAIL;
		}	
		s_sensor_init = SENSOR_TRUE;
		SENSOR_PRINT("SENSOR: Sensor_Init  Success \n");					
	}
	else
	{
		SENSOR_PRINT_ERR("Sensor identify fail,sensor_id = %d",sensor_id);
	}      
	
	return ret_val;	
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

	if(PNULL != s_sensor_info_ptr->resolution_tab_info_ptr[mode].sensor_reg_tab_ptr)
	{		
		// set mclk
		mclk = s_sensor_info_ptr->resolution_tab_info_ptr[mode].xclk_to_sensor;		
		Sensor_SetMCLK(mclk);

		// set image format
		s_sensor_exp_info.image_format = s_sensor_exp_info.sensor_mode_info[mode].image_format;

		// send register value to sensor
		Sensor_SendRegTabToSensor(&s_sensor_info_ptr->resolution_tab_info_ptr[mode]);

		s_sensor_mode[Sensor_GetCurId()]=mode;
	}
	else
	{
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
			sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];
		}
		else 
		{
			//sensor_i2c_driver.id_table = sensor_sub_id;
			sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0]; 
		}   	
		i2c_del_driver(&sensor_i2c_driver);
		g_is_register_sensor = 0;
		g_is_main_sensor = 0;
	}
	
	if(SENSOR_TRUE == Sensor_IsInit())
	{
		Sensor_PowerOn(SENSOR_FALSE);     
		if(SENSOR_MAIN == Sensor_GetCurId())
		{			
			SENSOR_PRINT_HIGH("SENSOR: Sensor_close 0.\n");
			 if(SCI_TRUE==s_sensor_register_info_ptr->is_register[SENSOR_SUB])
			 {
			 	SENSOR_PRINT_HIGH("SENSOR: Sensor_close 1.\n");
	        			_Sensor_SetId(SENSOR_SUB);
				s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
				Sensor_SetExportInfo(&s_sensor_exp_info);			
				Sensor_PowerOn(SENSOR_FALSE);	
				if(1== g_is_register_sensor) 
				{				
					SENSOR_PRINT_HIGH("SENSOR: Sensor_close 2.\n");
					sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0]; 						
					i2c_del_driver(&sensor_i2c_driver);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			 }
		}
		else if(SENSOR_SUB == Sensor_GetCurId()) {
			SENSOR_PRINT_HIGH("SENSOR: Sensor_close 3.\n");
			 if(SCI_TRUE==s_sensor_register_info_ptr->is_register[SENSOR_MAIN])
			 {
			 	SENSOR_PRINT_HIGH("SENSOR: Sensor_close 4.\n");
				_Sensor_SetId(SENSOR_MAIN);
				s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
				Sensor_SetExportInfo(&s_sensor_exp_info);			
				Sensor_PowerOn(SENSOR_FALSE);	
				if(1== g_is_register_sensor) 
				{					
					SENSOR_PRINT_HIGH("SENSOR: Sensor_close 5.\n");
					sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];					
					i2c_del_driver(&sensor_i2c_driver);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			 }
		}
		else if(SENSOR_ATV == Sensor_GetCurId()) {
			 if(SCI_TRUE==s_sensor_register_info_ptr->is_register[SENSOR_MAIN])
			 {
			 	SENSOR_PRINT_HIGH("SENSOR: Sensor_close 4.\n");
				_Sensor_SetId(SENSOR_MAIN);
				s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
				Sensor_SetExportInfo(&s_sensor_exp_info);			
				Sensor_PowerOn(SENSOR_FALSE);	
				if(1== g_is_register_sensor) 
				{					
					SENSOR_PRINT_HIGH("SENSOR: Sensor_close 5.\n");
					sensor_i2c_driver.address_list= &sensor_main_default_addr_list[0];					
					i2c_del_driver(&sensor_i2c_driver);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			 }
			 if(SCI_TRUE==s_sensor_register_info_ptr->is_register[SENSOR_SUB])
			 {
			 	SENSOR_PRINT_HIGH("SENSOR: Sensor_close 1.\n");
	        		_Sensor_SetId(SENSOR_SUB);
				s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
				Sensor_SetExportInfo(&s_sensor_exp_info);			
				Sensor_PowerOn(SENSOR_FALSE);	
				if(1== g_is_register_sensor) 
				{				
					SENSOR_PRINT_HIGH("SENSOR: Sensor_close 2.\n");
					sensor_i2c_driver.address_list= &sensor_sub_default_addr_list[0]; 						
					i2c_del_driver(&sensor_i2c_driver);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			 }			
		}

	}
          SENSOR_PRINT_HIGH("SENSOR: Sensor_close 6.\n");
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
PUBLIC ERR_SENSOR_E Sensor_SetTiming(SENSOR_MODE_E mode)
{
	uint32_t mclk;
	uint32_t cur_id = s_sensor_register_info_ptr->cur_id;
	
	printk("SENSOR: Sensor_SetTiming -> mode = %d,sensor_id=%d.\n", mode,cur_id);        
       
	//if(0!=cur_id)
	//	return 0;
	
	if(PNULL != s_sensor_info_ptr->resolution_tab_info_ptr[mode].sensor_reg_tab_ptr)
	{
	          // send register value to sensor
		Sensor_SendRegTabToSensor(&s_sensor_info_ptr->resolution_tab_info_ptr[mode]);

		s_sensor_mode[Sensor_GetCurId()]=mode;
	}
	else
	{
		SENSOR_PRINT("SENSOR: Sensor_SetResolution -> No this resolution information !!!");
	}       
	return SENSOR_SUCCESS;
}

PUBLIC int Sensor_CheckTiming(SENSOR_MODE_E mode)
{
	SENSOR_REG_TAB_INFO_T * sensor_reg_tab_info_ptr = &s_sensor_info_ptr->resolution_tab_info_ptr[mode];
	uint32_t i = 0;
	uint16_t data = 0;
	uint32_t cur_id = s_sensor_register_info_ptr->cur_id;
	int ret = SENSOR_SUCCESS;	
	
	printk("SENSOR: Sensor_CheckTiming -> mode = %d,sensor_id=%d.\n", mode,cur_id);      

	if(0!=cur_id)
		return 0;
		
	for(i = 0; i < sensor_reg_tab_info_ptr->reg_count; i++)
	{		
		if((0x4202 == sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr)||(SENSOR_WRITE_DELAY == sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr))
			continue;
		data = Sensor_ReadReg(sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr);
		if(data != sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_value)
		{
			ret = -1;
			printk("SENSOR: Sensor_CheckTiming report error!.\n");      
			break;
		}
	}		
	printk("SENSOR: Sensor_CheckTiming return = %d.\n", ret);  
	return ret;
}

/*****************************************************************************/
//  Description:    This function is used to control flash
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetFlash(uint32_t flash_mode)
{
	uint32_t reg_val = 0;

	if(s_flash_mode == flash_mode)
		return;

	s_flash_mode = flash_mode;
	
	printk("Sensor_SetFlash:flash_mode=0x%x .\n",flash_mode);
	//printk("Sensor_SetFlash:PIN_CTL_GPIO135->0x%x,PIN_CTL_GPIO144->0x%x .\n",_pard(PIN_CTL_GPIO135),_pard(PIN_CTL_GPIO144));

	switch (flash_mode)
	{
		case 1:  // flash on
		case 2:  // for torch
			// low light
			gpio_request(135,"gpio135");
			gpio_direction_output(135,1);
			gpio_set_value(135,1);
			gpio_request(144,"gpio144");
			gpio_direction_output(144,0);
			gpio_set_value(144,0); 
			break;
			
		case 0x11:
			// high light
			gpio_request(135,"gpio135");
			gpio_direction_output(135,1);
			gpio_set_value(135,1);

			gpio_request(144,"gpio144");
			gpio_direction_output(144,1);
			gpio_set_value(144,1); 
			break;

		case 0x10: // close flash
		case 0x0:
			// close the light 
			gpio_request(135,"gpio135");
			gpio_direction_output(135,0);
			gpio_set_value(135,0);
			gpio_request(144,"gpio144");
			gpio_direction_output(144,0);
			gpio_set_value(144,0);
			break;

		default:
			printk("Sensor_SetFlash unknow mode:flash_mode=%d .\n",flash_mode);
			break;
	}

	return 0;
}
/*****************************************************************************/
//  Description:    This function is used to get i2c client
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC struct i2c_client *Sensor_GetI2CClien(void)
{
	return this_client;
}

/*****************************************************************************/
//  Description:    This function is used to set sensor exif info    
//  Author:         
//  Note:           
/*****************************************************************************/
PUBLIC uint32 Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_E cmd ,uint32 param)
{
    SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
    EXIF_SPEC_PIC_TAKING_COND_T* sensor_exif_info_ptr=PNULL;

    if(PNULL!=sensor_info_ptr->ioctl_func_ptr->get_exif)/*lint !e613*/
    {
        sensor_exif_info_ptr=(EXIF_SPEC_PIC_TAKING_COND_T*)sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);/*lint !e613*/
    }
    else
    {
        SENSOR_PRINT("SENSOR: Sensor_SetSensorExifInfo the get_exif fun is null error \n");
        return SENSOR_FAIL;
    }

    switch(cmd)
    {
        case SENSOR_EXIF_CTRL_EXPOSURETIME:
        {
            SENSOR_MODE_E img_sensor_mode=s_sensor_mode[Sensor_GetCurId()];
            uint32 exposureline_time=sensor_info_ptr->sensor_mode_info[img_sensor_mode].line_time;/*lint !e613*/
            uint32 exposureline_num=param;
            uint32 exposure_time=0x00;

            exposure_time=exposureline_time*exposureline_num;

            sensor_exif_info_ptr->valid.ExposureTime=1;

            if(0x00==exposure_time)
            {
                sensor_exif_info_ptr->valid.ExposureTime=0;
            }
            else if(1000000>=exposure_time)
            {
                sensor_exif_info_ptr->ExposureTime.numerator=0x01;
                sensor_exif_info_ptr->ExposureTime.denominator=1000000/exposure_time;
            }
            else
            {
                uint32 second=0x00;
                do
                {
                    second++;
                    exposure_time-=1000000;
                    if(1000000>=exposure_time)
                    {
                        break;
                    }
                }while(1);/*lint !e506*/

                sensor_exif_info_ptr->ExposureTime.denominator=1000000/exposure_time;
                sensor_exif_info_ptr->ExposureTime.numerator=sensor_exif_info_ptr->ExposureTime.denominator*second;
            }
            break;
        }
        case SENSOR_EXIF_CTRL_FNUMBER:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_EXPOSUREPROGRAM:
        {
            break;
        }	
        case SENSOR_EXIF_CTRL_SPECTRALSENSITIVITY:
        {
            break;
        }	
        case SENSOR_EXIF_CTRL_ISOSPEEDRATINGS:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_OECF:
        {
            break;
        }	
        case SENSOR_EXIF_CTRL_SHUTTERSPEEDVALUE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_APERTUREVALUE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_BRIGHTNESSVALUE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_EXPOSUREBIASVALUE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_MAXAPERTUREVALUE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SUBJECTDISTANCE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_METERINGMODE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_LIGHTSOURCE:
        {
            sensor_exif_info_ptr->valid.LightSource=1;
            switch(param)
            {
                case 0:
                {
                    sensor_exif_info_ptr->LightSource=0x00;
                    break;
                }
                case 1:
                {
                    sensor_exif_info_ptr->LightSource=0x03;
                    break;
                }
                case 2:
                {
                    sensor_exif_info_ptr->LightSource=0x0f;
                    break;
                }
                case 3:
                {
                    sensor_exif_info_ptr->LightSource=0x0e;
                    break;
                }
                case 4:
                {
                    sensor_exif_info_ptr->LightSource=0x03;
                    break;
                }
                case 5:
                {
                    sensor_exif_info_ptr->LightSource=0x01;
                    break;
                }
                case 6:
                {
                    sensor_exif_info_ptr->LightSource=0x0a;
                    break;
                }
                default :
                {
                    sensor_exif_info_ptr->LightSource=0xff;
                    break;
                }
            }
            break;
        }	
        case SENSOR_EXIF_CTRL_FLASH:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FOCALLENGTH:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SUBJECTAREA:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FLASHENERGY:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SPATIALFREQUENCYRESPONSE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FOCALPLANEXRESOLUTION:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FOCALPLANEYRESOLUTION:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FOCALPLANERESOLUTIONUNIT:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SUBJECTLOCATION:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_EXPOSUREINDEX:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SENSINGMETHOD:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FILESOURCE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SCENETYPE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_CFAPATTERN:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_CUSTOMRENDERED:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_EXPOSUREMODE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_WHITEBALANCE:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_DIGITALZOOMRATIO:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_FOCALLENGTHIN35MMFILM:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SCENECAPTURETYPE:
        {
            sensor_exif_info_ptr->valid.SceneCaptureType=1;
            switch(param)
            {
                case 0:
                {
                    sensor_exif_info_ptr->SceneCaptureType=0x00;
                    break;
                }
                case 1:
                {
                    sensor_exif_info_ptr->SceneCaptureType=0x03;
                    break;
                }
                default :
                {
                    sensor_exif_info_ptr->LightSource=0xff;
                    break;
                }
            }
            break;
        }	
        case SENSOR_EXIF_CTRL_GAINCONTROL:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_CONTRAST:
        {
            sensor_exif_info_ptr->valid.Contrast=1;
            switch(param)
            {
                case 0:
                case 1:
                case 2:
                {
                    sensor_exif_info_ptr->Contrast=0x01;
                    break;
                }
                case 3:
                {
                    sensor_exif_info_ptr->Contrast=0x00;
                    break;
                }
                case 4:
                case 5:
                case 6:
                {
                    sensor_exif_info_ptr->Contrast=0x02;
                    break;
                }
                default :
                {
                    sensor_exif_info_ptr->Contrast=0xff;
                    break;
                }
            }
            break;
        }
        case SENSOR_EXIF_CTRL_SATURATION:
        {
            sensor_exif_info_ptr->valid.Saturation=1;
            switch(param)
            {
                case 0:
                case 1:
                case 2:
                {
                    sensor_exif_info_ptr->Saturation=0x01;
                    break;
                }
                case 3:
                {
                    sensor_exif_info_ptr->Saturation=0x00;
                    break;
                }
                case 4:
                case 5:
                case 6:
                {
                    sensor_exif_info_ptr->Saturation=0x02;
                    break;
                }
                default :
                {
                    sensor_exif_info_ptr->Saturation=0xff;
                    break;
                }
            }
            break;
        }
        case SENSOR_EXIF_CTRL_SHARPNESS:
        {
            sensor_exif_info_ptr->valid.Sharpness=1;
            switch(param)
            {
                case 0:
                case 1:
                case 2:
                {
                    sensor_exif_info_ptr->Sharpness=0x01;
                    break;
                }
                case 3:
                {
                    sensor_exif_info_ptr->Sharpness=0x00;
                    break;
                }
                case 4:
                case 5:
                case 6:
                {
                    sensor_exif_info_ptr->Sharpness=0x02;
                    break;
                }
                default :
                {
                    sensor_exif_info_ptr->Sharpness=0xff;
                    break;
                }
            }
            break;
        }
        case SENSOR_EXIF_CTRL_DEVICESETTINGDESCRIPTION:
        {
            break;
        }
        case SENSOR_EXIF_CTRL_SUBJECTDISTANCERANGE:
        {
            break;
        }
        default :
            break;
    }

    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor exif info    
//  Author:         
//  Note:           
/*****************************************************************************/
PUBLIC EXIF_SPEC_PIC_TAKING_COND_T* Sensor_GetSensorExifInfo(void)
{
    SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
    EXIF_SPEC_PIC_TAKING_COND_T* sensor_exif_info_ptr=PNULL;

    if(PNULL!=sensor_info_ptr->ioctl_func_ptr->get_exif)/*lint !e613*/
    {
        sensor_exif_info_ptr=(EXIF_SPEC_PIC_TAKING_COND_T*)sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);/*lint !e613*/
    }

    return sensor_exif_info_ptr;
}
/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
    
#endif  // End of sensor_drv.c
