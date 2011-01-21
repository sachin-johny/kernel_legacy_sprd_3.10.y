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

//#define DEBUG_SENSOR_DRV

#ifdef DEBUG_SENSOR_DRV
#define SENSOR_PRINT   printk
#else
#define SENSOR_PRINT(...)  
#endif

#include "sensor_drv.h"
#include "sensor_cfg.h"
#include "dcam_reg_sc8800g2.h"
#include <mach/adi_hal_internal.h>

/*#include "sc_reg.h"
#include "i2c_api.h"
#include "os_api.h"
#include "chip.h"
#include "ref_outport.h"
#include "ldo_drv.h"
#include "gpio_prod_api.h" */

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
/**---------------------------------------------------------------------------*
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/
 //save value for register
//#define LOCAL_VAR_DEF uint32_t reg_val;
#define REG_SETBIT(_reg_addr, _bit_mask, _bit_set) ANA_REG_MSK_OR(_reg_addr, _bit_set, _bit_mask);
#define SET_LEVEL(_reg_addr, _bit0_mask, _bit1_mask, _set_bit0,\
                  _rst_bit0, _set_bit1, _rst_bit1) \
                  do{ \
                      REG_SETBIT( \
                     (_reg_addr), \
                     ((_set_bit0)|(_rst_bit0) | (_set_bit1)|(_rst_bit1)),  \
                     (((_set_bit0)&(_bit0_mask)) | ((_rst_bit0)&(~_bit0_mask))| \
                     ((_set_bit1)&(_bit1_mask)) | ((_rst_bit1)&(~_bit1_mask)))  \
                    )   \
                  }while(0)
#define REG_SETCLRBIT(_reg_addr, _set_bit, _clr_bit)    \
                      do  \
                      {   \
                         uint32_t reg_val;\
                         reg_val = ANA_REG_GET(_reg_addr);   \
                         reg_val |= (_set_bit);  \
                         reg_val &= ~(_clr_bit); \
                         ANA_REG_SET(_reg_addr,reg_val);   \
                      }while(0)


					  
//set or clear the bit of register in reg_addr address
/*#define REG_SETCLRBIT(_reg_addr, _set_bit, _clr_bit) \
	do \
	{ \
		reg_val = _pard((_reg_addr)); \
		reg_val |= (_set_bit); \
		reg_val &= ~(_clr_bit); \
		_pawd((_reg_addr), reg_val); \
	}while(0);
//set the value of register bits corresponding with bit_mask
#define REG_SETBIT(_reg_addr, _bit_mask, _bit_set) \
	do \
	{ \
		reg_val = _pard(_reg_addr); \
		reg_val &= ~(_bit_mask); \
		reg_val |= ((_bit_set) & (_bit_mask)); \
		_pawd((_reg_addr), reg_val); \
	}while(0);
*/
//macro used to set voltage level according to bit field
#define SET_LEVELBIT(_reg_addr, _bit_mask, _set_bit, _rst_bit) \
{ \
	REG_SETBIT( \
		_reg_addr, \
		(_set_bit) | (_rst_bit), \
		((_set_bit) & (_bit_mask)) | ((_rst_bit) & (~_bit_mask)) \
		) \
}
//macro used to set voltage level according to two bits
/*#define SET_LEVEL(_reg_addr, _bit0_mask, _bit1_mask, _set_bit0, \
	_rst_bit0, _set_bit1, _rst_bit1) \
{ \
	REG_SETBIT( \
		(_reg_addr), \
		((_set_bit0) |(_rst_bit0) | (_set_bit1) |(_rst_bit1)), \
		(((_set_bit0) & (_bit0_mask)) | ((_rst_bit0) & (~_bit0_mask)) | \
		((_set_bit1) & (_bit1_mask)) | ((_rst_bit1) & (~_bit1_mask))) \
		) \
}*/
//#define CHIP_REG_OR(reg_addr, value) (*(volatile uint32_t *)(reg_addr) |= (uint32_t)(value))
#define LDO_USB_PD BIT_9
typedef enum
{
	LDO_VOLT_LEVEL0 = 0,
	LDO_VOLT_LEVEL1,
	LDO_VOLT_LEVEL2,
	LDO_VOLT_LEVEL3,
	LDO_VOLT_LEVEL_MAX	
}LDO_VOLT_LEVEL_E;
typedef enum
{
	LDO_LDO_CAMA = 28,
	LDO_LDO_CAMD0,
	LDO_LDO_CAMD1,
	LDO_LDO_VDD18,
	LDO_LDO_VDD25,
	LDO_LDO_VDD28,
	LDO_LDO_RF0,
	LDO_LDO_RF1,
	LDO_LDO_USBD,
	LDO_LDO_MAX	
}LDO_ID_E;
typedef struct
{
    LDO_ID_E id;
    uint32_t bp_reg;
    uint32_t bp;
    uint32_t bp_rst;
    uint32_t level_reg_b0;
    uint32_t b0;
    uint32_t b0_rst;
    uint32_t level_reg_b1;
    uint32_t b1;
    uint32_t b1_rst;
    uint32_t valid_time;
    uint32_t init_level;
    uint32_t ref;           
}LDO_CTL_T,* LDO_CTL_PTR;

static LDO_CTL_T g_ldo_ctl_tab[] = 
{
	{
		LDO_LDO_USBD, DCAM_NULL, DCAM_NULL, DCAM_NULL, DCAM_NULL,DCAM_NULL,DCAM_NULL,
		DCAM_NULL, DCAM_NULL, DCAM_NULL,DCAM_NULL, LDO_VOLT_LEVEL_MAX, DCAM_NULL
	},
	{
		LDO_LDO_CAMA, ANA_LDO_PD_CTL, BIT_12, BIT_13, ANA_LDO_VCTL2,BIT_8,BIT_9,
		ANA_LDO_VCTL2, BIT_10, BIT_11,DCAM_NULL, LDO_VOLT_LEVEL_MAX, DCAM_NULL
	},
	{
		LDO_LDO_CAMD1, ANA_LDO_PD_CTL, BIT_10, BIT_11, ANA_LDO_VCTL2,BIT_4,BIT_5,
		ANA_LDO_VCTL2, BIT_6, BIT_7,DCAM_NULL, LDO_VOLT_LEVEL_MAX, DCAM_NULL
	},
	{
		LDO_LDO_CAMD0, ANA_LDO_PD_CTL, BIT_8, BIT_9, ANA_LDO_VCTL2,BIT_0,BIT_1,
		ANA_LDO_VCTL2, BIT_2, BIT_3,DCAM_NULL, LDO_VOLT_LEVEL_MAX, DCAM_NULL
	},
	{
		LDO_LDO_MAX, DCAM_NULL, DCAM_NULL, DCAM_NULL, DCAM_NULL,DCAM_NULL,DCAM_NULL,
		DCAM_NULL, DCAM_NULL, DCAM_NULL,DCAM_NULL, LDO_VOLT_LEVEL_MAX, DCAM_NULL
	}
};

/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL SENSOR_INFO_T* s_sensor_list_ptr[SENSOR_ID_MAX];//={0x00}; 
LOCAL SENSOR_INFO_T* s_sensor_info_ptr=PNULL;
LOCAL SENSOR_EXP_INFO_T s_sensor_exp_info;//={0x00};
LOCAL uint32_t s_sensor_mclk=0;
//LOCAL uint8_t s_sensor_probe_index=0;
LOCAL BOOLEAN s_sensor_init=SENSOR_FALSE;  
LOCAL BOOLEAN s_sensor_open=SENSOR_FALSE;
LOCAL BOOLEAN s_atv_init=SENSOR_FALSE;
LOCAL BOOLEAN s_atv_open=SENSOR_FALSE;
LOCAL SENSOR_TYPE_E s_sensor_type=SENSOR_TYPE_NONE;
LOCAL SENSOR_MODE_E s_sensor_mode[SENSOR_ID_MAX]={SENSOR_MODE_MAX,SENSOR_MODE_MAX,SENSOR_MODE_MAX};
LOCAL SENSOR_MUTEX_PTR	s_imgsensor_mutex_ptr=PNULL;
LOCAL SENSOR_REGISTER_INFO_T s_sensor_register_info={0x00};
LOCAL SENSOR_REGISTER_INFO_T_PTR s_sensor_register_info_ptr=&s_sensor_register_info;


/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                     Local Function Prototypes                             *
 **---------------------------------------------------------------------------*/
 #define SENSOR_INHERIT 0
 #define SENSOR_WAIT_FOREVER 0
 
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
PUBLIC int32_t _Sensor_IicHandlerInit(void) 
{
    int32_t dev_handler=0;
   /* I2C_DEV  dev={0x00};

    if(!((SENSOR_I2C_NULL_HANDLE==s_sensor_info_ptr->i2c_dev_handler)
        ||(NULL==s_sensor_info_ptr->i2c_dev_handler)))
    {
        return s_sensor_info_ptr->i2c_dev_handler;
    }
    
    dev.id=SENSOR_I2C_PORT_0;

    if(SENSOR_I2C_FREQ_20==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_FREQ_20))
    {
        dev.freq=(20*1000);
    }
    else if(SENSOR_I2C_FREQ_50==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_FREQ_50))
    {
        dev.freq=(50*1000);
    }   
    else if(SENSOR_I2C_FREQ_200==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_FREQ_200))
    {
        dev.freq=(200*1000);
    } 
    else
    {
        dev.freq=SENSOR_I2C_FREQ;
    }
    
    dev.slave_addr=s_sensor_info_ptr->salve_i2c_addr_w;

    if(SENSOR_I2C_CUSTOM==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_CUSTOM))
    {
        dev.reg_addr_num=SENSOR_ZERO_I2C;
    }
    else if(SENSOR_I2C_REG_16BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_REG_16BIT))
    {
        dev.reg_addr_num=SENSOR_ADDR_BITS_16;
    }        
    else
    {
        dev.reg_addr_num=SENSOR_ADDR_BITS_8;
    }

    if(SENSOR_I2C_NOACK_BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_NOACK_BIT))
    {
        dev.check_ack=SENSOR_I2C_ACK_FALSE;
    }      
    else
    {
        dev.check_ack=SENSOR_I2C_ACK_TRUE;
    }

    if(SENSOR_I2C_STOP_BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_STOP_BIT))
    {
        dev.no_stop=SENSOR_I2C_STOP;
    }
    else
    {
        dev.no_stop=SENSOR_I2C_NOSTOP;
    }

    dev_handler=I2C_HAL_Open(&dev);

    if(dev_handler==SENSOR_I2C_NULL_HANDLE)
    {
        SENSOR_PRINT("SENSOR_handler creat err first");
    }

    s_sensor_info_ptr->i2c_dev_handler=dev_handler;

    */
    return dev_handler;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor type    
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC void _Sensor_IicHandlerRelease(void) 
{
   /* if(SENSOR_ZERO_I2C!=I2C_HAL_Close(s_sensor_info_ptr->i2c_dev_handler))	
    {
        SENSOR_PRINT ("SENSOR: I2C_no_close");
    }
    else
    {
        s_sensor_info_ptr->i2c_dev_handler=SENSOR_I2C_NULL_HANDLE;
        SENSOR_PRINT ("SENSOR: I2C_close s_sensor_info_ptr->i2c_dev_handler=%d ",s_sensor_info_ptr->i2c_dev_handler);
    }*/
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
PUBLIC void Sensor_Reset(void)
{
#define SENSOR_RST_CTL 77
#define SENSOR_POWER1_CTL 78
#define SENSOR_POWER0_CTL 79
	

/*static unsigned long cam_gpio_cfg[] __initdata = {
		MFP_CFG_X(CCIRPD1,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
		MFP_CFG_X(CCIRPD0,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
		MFP_CFG_X(CCIRRST,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
	};	*/
/*static unsigned long cam_gpio_cfg[] __initdata = {
        MFP_CFG_X(CCIRMCLK, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRCK, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRHS, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRVS, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD0, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD1, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD2, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD3, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD4, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD5, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD6, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRD7, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRRST, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRPD1, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
        MFP_CFG_X(CCIRPD0, AF3, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
};
	sprd_mfp_config(cam_gpio_cfg,15);*/
    //sprd_mfp_config(cam_gpio_cfg,3);
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

/*****************************************************************************/
//  Description:    This function is used to power on sensor and select xclk    
//  Author:         Liangwen.Zhen
//  Note:           1.Unit: MHz 2. if mclk equal 0, close main clock to sensor
/*****************************************************************************/
/*static void sensor_setmclk(void)
{
    uint32_t divd;
    __raw_bits_and(~(1<<14),(SPRD_GREG_BASE+ 0x08));//first disable MCLK
    
    __raw_bits_and(~(1<<18 | 1<<19),(SPRD_GREG_BASE+ 0x70));// bit19, bit18 ,  00 48M,01  76.8M, 1x, 26M
    
    __raw_bits_and(~(1<<24 | 1<<25),(SPRD_GREG_BASE+ 0x1c));// CCIR divide factor  

    divd=48/12-1;

    divd = divd <<24;

    __raw_bits_or(divd,(SPRD_GREG_BASE + 0x1c));// CCIR divide factor
    
    __raw_bits_or(1<<14,(SPRD_GREG_BASE + 0x08)); // CCIR CLK Enable
                                                                                                                       
}*/
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

/*****************************************************************************/
//  Description:    This function is used to set AVDD
//  Author:         Liangwen.Zhen
//  Note:           Open AVDD on one special voltage or Close it
/*****************************************************************************/
LOCAL LDO_CTL_PTR LDO_GetLdoCtl(LDO_ID_E ldo_id)
{
	uint32_t i;
	LDO_CTL_PTR ctl = DCAM_NULL;

	for(i = 0; g_ldo_ctl_tab[i].id != LDO_LDO_MAX; i++)
	{
		if(ldo_id == g_ldo_ctl_tab[i].id)
		{
			ctl = &g_ldo_ctl_tab[i];
			break;
		}
	}

	return ctl;
}

LOCAL uint32_t LDO_TurnOnLDO(LDO_ID_E ldo_id)
{
	//LOCAL_VAR_DEF
	LDO_CTL_PTR ctl = DCAM_NULL;

	ctl = LDO_GetLdoCtl(ldo_id);
	if(DCAM_NULL == ctl->bp_reg)
	{
		if(LDO_LDO_USBD == ldo_id)
		{
			//CHIP_REG_OR( GR_CLK_GEN5, (~LDO_USB_PD));
			_paod( GR_CLK_GEN5, (~LDO_USB_PD));
		}
		return SENSOR_SUCCESS;
	}
	if(0 == ctl->ref)
	{
		REG_SETCLRBIT(ctl->bp_reg, ctl->bp_rst, ctl->bp);
	}
	ctl->ref++;

	return SENSOR_SUCCESS;	
}
LOCAL uint32_t LDO_TurnOffLDO(LDO_ID_E ldo_id)
{
	//LOCAL_VAR_DEF
	LDO_CTL_PTR ctl = DCAM_NULL;	

	ctl = LDO_GetLdoCtl(ldo_id);
	if(DCAM_NULL == ctl->bp_reg)
	{
		if(LDO_LDO_USBD == ldo_id)
		{
			//CHIP_REG_OR(GR_CLK_GEN5, LDO_USB_PD);
			_paod(GR_CLK_GEN5, LDO_USB_PD);
		}
		return SENSOR_SUCCESS;
	}
	if(ctl->ref > 0)
		ctl->ref--;
	if(0 == ctl->ref)
	{
		REG_SETCLRBIT(ctl->bp_reg, ctl->bp, ctl->bp_rst);
	}

	return SENSOR_SUCCESS;	
}

LOCAL uint32_t LDO_SetVoltLevel(LDO_ID_E ldo_id, LDO_VOLT_LEVEL_E volt_level)
{
	//LOCAL_VAR_DEF
	uint32_t b0_mask, b1_mask;
	LDO_CTL_PTR ctl = DCAM_NULL;

	b0_mask = (volt_level & BIT_0) ? ~0 : 0;
	b1_mask = (volt_level & BIT_1) ? ~0 : 0;	

	ctl = LDO_GetLdoCtl(ldo_id);
	if(DCAM_NULL == ctl->level_reg_b0)
		return SENSOR_SUCCESS;

	if(ctl->level_reg_b0 == ctl->level_reg_b1)
	{
		SET_LEVEL(ctl->level_reg_b0, b0_mask, b1_mask, ctl->b0, ctl->b0_rst, ctl->b1, ctl->b1_rst);
	}
	else
	{
		SET_LEVELBIT(ctl->level_reg_b0, b0_mask, ctl->b0, ctl->b0_rst);
		SET_LEVELBIT(ctl->level_reg_b1, b1_mask, ctl->b1, ctl->b1_rst);		
	}

	return SENSOR_SUCCESS;		
}
PUBLIC void Sensor_SetVoltage(SENSOR_AVDD_VAL_E dvdd_val, SENSOR_AVDD_VAL_E avdd_val, SENSOR_AVDD_VAL_E iodd_val)
{
    uint32_t ldo_volt_level = LDO_VOLT_LEVEL0;

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
            ldo_volt_level = LDO_VOLT_LEVEL_MAX;   
            break;
    } 
   
    if(LDO_VOLT_LEVEL_MAX == ldo_volt_level)
    {
        SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off avdd\n"); 
    
        LDO_TurnOffLDO(LDO_LDO_CAMA);        
        LDO_TurnOffLDO(LDO_LDO_CAMD1);            

        //*(volatile uint32_t*)AHB_GLOBAL_REG_CTL0 |=  (BIT_1|BIT_2);//ccir and dcam enable	
        //*(volatile uint32_t*)CAP_CNTRL &= ~(BIT_13 | BIT_12); //set two sensor all in PowerDown mode
       // *(volatile uint32_t*)AHB_GLOBAL_REG_CTL0 &= ~(BIT_1|BIT_2);//ccir and dcam disable
        _paod( AHB_GLOBAL_REG_CTL0, BIT_1|BIT_2);//ccir and dcam enable
	_paad(CAP_CNTRL, ~(BIT_13 | BIT_12)); //set two sensor all in PowerDown mode
	_paad(AHB_GLOBAL_REG_CTL0, ~(BIT_1|BIT_2));//ccir and dcam disable        
    }
    else
    {
        SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn on avdd, VoltLevel %d\n",ldo_volt_level); 
    
        LDO_SetVoltLevel(LDO_LDO_CAMA, ldo_volt_level);      
        LDO_TurnOnLDO(LDO_LDO_CAMA);  
        LDO_SetVoltLevel(LDO_LDO_CAMD1, 0);      
        LDO_TurnOnLDO(LDO_LDO_CAMD1);  

       // *(volatile uint32_t*)AHB_GLOBAL_REG_CTL0 |=  (BIT_1|BIT_2);
        //*(volatile uint32_t*)CAP_CNTRL |= BIT_13 | BIT_12;
        //*(volatile uint32_t*)AHB_GLOBAL_REG_CTL0 &= ~(BIT_1|BIT_2);
        _paod(AHB_GLOBAL_REG_CTL0, BIT_1|BIT_2);//ccir and dcam enable
	_paod(CAP_CNTRL, BIT_13 | BIT_12); //set two sensor all in PowerDown mode
	_paad(AHB_GLOBAL_REG_CTL0, ~(BIT_1|BIT_2));//ccir and dcam disable
    }

    switch(dvdd_val)
    {
        case SENSOR_AVDD_1800MV:
            ldo_volt_level = LDO_VOLT_LEVEL0;    
            break;
            
        case SENSOR_AVDD_2800MV:
            ldo_volt_level = LDO_VOLT_LEVEL1;    
            break;
            
        case SENSOR_AVDD_3000MV:
            ldo_volt_level = LDO_VOLT_LEVEL2;    
            break;
            
        case SENSOR_AVDD_1300MV:
            ldo_volt_level = LDO_VOLT_LEVEL3;    
            break;  
            
        case SENSOR_AVDD_CLOSED:
        case SENSOR_AVDD_UNUSED:
        default:
            ldo_volt_level = LDO_VOLT_LEVEL_MAX;           
            break;
    } 
    
    if(LDO_VOLT_LEVEL_MAX == ldo_volt_level)
    {
        SENSOR_PRINT("SENSOR: Sensor_SetVoltage.... turn off dvdd, sensor_id %d\n",
                     Sensor_GetCurId()); 

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
#if 0
#define AHB_CTL0        (SPRD_AHB_BASE + 0x200)
#define DCAM_CAP_CTL    (SPRD_ISP_BASE + 0x100)
static void sensor_power_down(void)
{
    __raw_bits_and(~(1<<4 | 1<<5),(SPRD_CPC_BASE+0x344));//ccir and dcam enable
    __raw_bits_or((1<<1 | 1<<2),AHB_CTL0);//ccir and dcam enable
    
    __raw_bits_and(~(1<<11 | 1<<12),DCAM_CAP_CTL);
    msleep(10);
    __raw_bits_or(1<<11, DCAM_CAP_CTL);

    __raw_bits_and(~(1<<1 | 1<<2),AHB_CTL0);//ccir and dcam disable

}


static int ldo_set_volt_level(LDO_CTL_PTR ctl)
{
   //LOCAL_VAR_DEF
   SET_LEVEL(ctl->level_reg_b0, 0 , 0, ctl->b0, ctl->b0_rst, ctl->b1, ctl->b1_rst); 
    
   return 0;   
}
static int ldo_turn_on_ldo(LDO_CTL_PTR ctl)
{
   //LOCAL_VAR_DEF
   REG_SETCLRBIT (ctl->bp_reg, ctl->bp_rst, ctl->bp); 
   return 0;   
}

static void  sensor_set_voltage(void)
{
    LDO_CTL_T ctl;

    //CAMA
    ctl.level_reg_b0= (SPRD_MISC_BASE+0x0480+0x1c);
    ctl.b0= (1<<8);
    ctl.b0_rst= (1<<9);
    ctl.b1= (1<<10);
    ctl.b1_rst= (1<<11);
    ctl.bp_reg= (SPRD_MISC_BASE+0x0480+0x10);
    ctl.bp_rst= (1<<13);
    ctl.bp= (1<<12);
    ldo_set_volt_level(&ctl);
    ldo_turn_on_ldo(&ctl);
    //CAMD1
    ctl.level_reg_b0= (SPRD_MISC_BASE+0x0480+0x1c);
    ctl.b0= (1<<4);
    ctl.b0_rst= (1<<5);
    ctl.b1= (1<<6);
    ctl.b1_rst= (1<<7);
    ctl.bp_reg= (SPRD_MISC_BASE+0x0480+0x10);
    ctl.bp_rst= (1<<11);
    ctl.bp= (1<<10);
    ldo_set_volt_level(&ctl);
    ldo_turn_on_ldo(&ctl);
   
    __raw_bits_or((1<<1)|(1<<2),AHB_CTL0); //ccir and dcam enable
    __raw_bits_or((1<<13)|(1<<12),DCAM_CAP_CTL);//set two sensor all in PowerDown mode
    __raw_bits_and(~(1<<1|1<<2),AHB_CTL0); //ccir and dcam disable
    
    //CAMD0
    ctl.level_reg_b0= (SPRD_MISC_BASE+0x0480+0x1c);
    ctl.b0= (1<<0);
    ctl.b0_rst= (1<<1);
    ctl.b1= (1<<2);
    ctl.b1_rst= (1<<3);
    ctl.bp_reg= (SPRD_MISC_BASE+0x0480+0x10);
    ctl.bp_rst= (1<<9);
    ctl.bp= (1<<8);
    ldo_set_volt_level(&ctl);
    ldo_turn_on_ldo(&ctl);


} 

static int scan_i2c_init(void)
{
#define I2C_ADAPTER_MAX_NUM 10
    int i, j, ret, n;
	static struct i2c_adapter * ap[I2C_ADAPTER_MAX_NUM];
static int i2c_adapter_num;

    sensor_set_voltage();

    sensor_power_down();
    
    sensor_setmclk();

	static unsigned long cam_gpio_cfg[] __initdata = {
		MFP_CFG_X(CCIRPD1,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
		MFP_CFG_X(CCIRPD0,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
		MFP_CFG_X(CCIRRST,AF3,DS1,F_PULL_NONE,S_PULL_NONE,IO_OE),
	};	
	
    sprd_mfp_config(cam_gpio_cfg,3);
 
	gpio_request(78,"ccir_pd1");
	gpio_request(79,"ccir_pd0");
	gpio_request(77,"ccir_rst");
	
	gpio_direction_output(78,0);
	gpio_direction_output(79,0);
	gpio_direction_output(77,1);
    msleep(10);
	gpio_direction_output(77,0);
    msleep(20);
	gpio_direction_output(77,1);
	msleep(100);
	

    i2c_adapter_num = 0;

    for (i = 0; i < I2C_ADAPTER_MAX_NUM; i++) {
        ap[i] = i2c_get_adapter(i);
        if (ap[i] == NULL)
            break;
        i2c_adapter_num++;
        printk("Probeing adapter %d...\n", i);
	for(j=0;j<0xff;j++){
	unsigned char buf_w[3];
	buf_w[0]=0x30;
	buf_w[1]=12;
	buf_w[2]=0x80;
	struct i2c_msg msg_w={
			     .addr=0x30,
			     .flags=0,
			     .buf=buf_w,
			     .len=3 };
        ret = i2c_transfer(ap[i],&msg_w,1);
	if(ret!=1)
	{
	    printk("write failed\n");
	    continue;
	}
	unsigned char buf_r;
	struct i2c_msg msg_r[2]={
			     {.addr=0x30,
			     .flags=0,
			     .buf=buf_w,
			     .len=2 },

			     {.addr=0x30,
			     .flags=I2C_M_RD,
			     .buf=&buf_r,
			     .len=1 }
			};
	
        ret = i2c_transfer(ap[i],msg_r,2);
        if (ret!=2){
		printk("read failed\n");
             	continue;
	}
	printk("buf_r=0x%x\n",buf_r);
	ndelay(10);
   }
  }

    return -EISNAM;
}

#endif //0
LOCAL void Sensor_PowerOn(BOOLEAN power_on)
{
//scan_i2c_init();
    BOOLEAN 				power_down;		
    SENSOR_AVDD_VAL_E		dvdd_val;
    SENSOR_AVDD_VAL_E		avdd_val;
    SENSOR_AVDD_VAL_E		iovdd_val;    
    SENSOR_IOCTL_FUNC_PTR	power_func;

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
            // Open power
            Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val); 
            // NOT in power down mode(maybe also open DVDD and DOVDD)
            Sensor_PowerDown(!power_down);           
            // Open Mclk in default frequency
            Sensor_SetMCLK(SENSOR_DEFALUT_MCLK); 
            // Reset sensor
            Sensor_Reset();	
        }
        else
        {
            // Power down sensor and maybe close DVDD, DOVDD
            Sensor_PowerDown(power_down);

            Sensor_SetVoltage(SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED);
            // Close Mclk
            Sensor_SetMCLK(SENSOR_DISABLE_MCLK);		
        }
    }    
}

/*****************************************************************************/
//  Description:    This function is used to power down sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_PowerDown(BOOLEAN power_down)
{
    switch(Sensor_GetCurId())
    {
        case SENSOR_MAIN:
        {
            SENSOR_PRINT("SENSOR: Sensor_PowerDown -> main: power_down %d\n", power_down);          
            SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD1-> 0x8C000344 0x%x\n", _pard(PIN_CTL_CCIRPD1)); // *(volatile uint32_t*)0x8C000344 );          
            SENSOR_PRINT("SENSOR: Sensor_PowerDown PIN_CTL_CCIRPD0-> 0x8C000348 0x%x\n", _pard(PIN_CTL_CCIRPD0));//*(volatile uint32_t*)0x8C000348 );    
            
            //*(volatile uint32_t*)0x8C000344 &=  ~(BIT_4|BIT_5);//ccir and dcam enable
            _paad(PIN_CTL_CCIRPD1,  ~(BIT_4|BIT_5));
           // *(volatile uint32_t*)AHB_REG_BASE |=  (BIT_1|BIT_2);//ccir and dcam enable
            _paod(AHB_GLOBAL_REG_CTL0,  BIT_1|BIT_2);
            if(power_down == 0)
            {
                //*(volatile uint32_t*)(DCAM_BASE + 0x0100) &= ~(BIT_11|BIT_12);
		_paad(CAP_CNTRL,  ~(BIT_11|BIT_12));
              	SENSOR_Sleep(10);
                //*(volatile uint32_t*)(DCAM_BASE + 0x0100) |= BIT_11;
                _paod(CAP_CNTRL,  BIT_11);
               
            }
            else
            {
                //*(volatile uint32_t*)(DCAM_BASE + 0x0100) |= BIT_12;
		_paod(CAP_CNTRL,  BIT_12);
            }

            //*(volatile uint32_t*)AHB_REG_BASE &= ~(BIT_1|BIT_2);//ccir and dcam disable
            _paad(AHB_GLOBAL_REG_CTL0,  ~(BIT_1|BIT_2));            
            break;
        }
        case SENSOR_SUB:
        {
            SENSOR_PRINT("SENSOR: Sensor_PowerDown -> sub\n");           
            //*(volatile uint32_t*)AHB_REG_BASE |=  (BIT_1|BIT_2);//ccir and dcam enable
            _paod(AHB_GLOBAL_REG_CTL0,  BIT_1|BIT_2);
            if(power_down == 0)
            {
               // *(volatile uint32_t*)(DCAM_BASE + 0x0100) &= ~(BIT_11|BIT_13);
		_paad(CAP_CNTRL,  ~(BIT_11|BIT_13));		
              	SENSOR_Sleep(10);
                //*(volatile uint32_t*)(DCAM_BASE + 0x0100) |= BIT_11;
		_paod(CAP_CNTRL,  BIT_11);	
               
            }
            else
            {
               // *(volatile uint32_t*)(DCAM_BASE + 0x0100) |= BIT_13;
		_paod(CAP_CNTRL,  BIT_13);
            }

            //*(volatile uint32_t*)AHB_REG_BASE &= ~(BIT_1|BIT_2);//ccir and dcam disable
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
/*****************************************************************************/
//  Description:    This function is used to reset img sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_SetResetLevel(BOOLEAN plus_level)
{
//wxz:???
/*
    if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
    {
	    GPIO_SetSensorResetLevel(plus_level);
    }
    else if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
    {
	    GPIO_SetAnalogTVResetLevel(plus_level);
    }*/

    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to check sensor parameter      
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
/*LOCAL BOOLEAN Sensor_CheckSensorInfo(SENSOR_INFO_T * info_ptr)
{
	if(info_ptr->name)
	{
		SENSOR_PRINT("SENSOR: Sensor_CheckSensorInfo -> sensor name = %s", info_ptr->name);		
	}
	
	return SENSOR_TRUE;
}*/

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

//------ To Sensor Module
 
/*****************************************************************************/
//  Description:    This function is used to write value to sensor register    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
int Sensor_WriteReg(uint16_t reg_addr, uint16_t value)
{
	struct i2c_adapter *adpter = DCAM_NULL;
	uint8_t buf_w[3];
	uint32_t ret = -1;
	struct i2c_msg msg_w;

	adpter = i2c_get_adapter(0);
        if (DCAM_NULL == adpter)
        {
            printk("#DCAM: get i2c adapter NULL\n");
            return -1;
        }
		
	buf_w[0]= reg_addr >> 8;
	buf_w[1]= reg_addr & 0xFF;
	buf_w[2]= (uint8_t)value;
	msg_w.addr = s_sensor_info_ptr->salve_i2c_addr_w; 
	msg_w.flags = 0;
	msg_w.buf = buf_w;
	msg_w.len = 3;
        ret = i2c_transfer(adpter, &msg_w, 1);
	if(ret!=1)
        {
            printk("#DCAM: write sensor reg fai, ret: %x \n", ret);
            return -1;
        }

	i2c_put_adapter(adpter);

	adpter = DCAM_NULL;

	return 0;
}
uint16_t Sensor_ReadReg(uint16_t reg_addr)
{
	static struct i2c_adapter *adpter = DCAM_NULL;
	uint8_t buf_w[2];
	uint8_t buf_r;
	uint32_t ret = -1;
	uint16_t value = 0;
	struct i2c_msg msg_r[2];

	adpter = i2c_get_adapter(0);
        if (DCAM_NULL == adpter)
        {
            printk("#DCAM: get i2c adapter NULL\n");
            return -1;
        }
		
	buf_w[0]= reg_addr >> 8;
	buf_w[1]= reg_addr & 0xFF;
	msg_r[0].addr = s_sensor_info_ptr->salve_i2c_addr_w; //OV2655_I2C_ADDR_W;
	msg_r[0].flags = 0;
	msg_r[0].buf = buf_w;
	msg_r[0].len = 2;
	msg_r[1].addr = s_sensor_info_ptr->salve_i2c_addr_r; //OV2655_I2C_ADDR_R;
	msg_r[1].flags = I2C_M_RD;
	msg_r[1].buf = &buf_r;
	msg_r[1].len = 1;
        ret = i2c_transfer(adpter, msg_r, 2);
	if(ret!=2)
        {
            printk("#DCAM: read sensor reg fail, ret: %x \n", ret);
            return -1;
        }

	i2c_put_adapter(adpter);

	value = buf_r;
	adpter = DCAM_NULL;

	return value;
}
/*PUBLIC void Sensor_WriteReg( uint16_t  subaddr, uint16_t data )
{
    int32_t i2c_handle_sensor;
    I2C_DEV dev;
    uint8_t  cmd[4] = {0};
    uint8_t  cmd_add[4] = {0};
    uint32_t index=0;
    uint8_t  bytes=0;
    uint8_t  addr_w;
    uint8_t  addr_r;
    uint32_t cmd_num = 0;
    SENSOR_IOCTL_FUNC_PTR 	write_reg_func;

    write_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->write_reg;

    if(PNULL != write_reg_func)
    {
        if(SENSOR_SUCCESS != write_reg_func((subaddr << BIT_4) + data))
        {
            SENSOR_PRINT("SENSOR: Sensor_WriteReg reg/value(%x,%x) Error !!", subaddr, data);
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
            cmd[cmd_num++] = (uint8_t)(data & SENSOR_LOW_EIGHT_BIT);	    		    	
        }
        else
        {
            cmd[cmd_num++] = (uint8_t)data;
        }

        if(SENSOR_WRITE_DELAY != subaddr)
        {
            i2c_handle_sensor=RE_I2C_HANDLER();
            I2C_HAL_Write(i2c_handle_sensor, cmd, &cmd[index], cmd_num-index);
            SENSOR_PRINT("write reg: %04x, val: %04x,", subaddr, data);
        }
        else
        {
            if(data > 0x80)
            {
                SENSOR_Sleep(data);
            }
            else
            {
                OS_TickDelay(data);
            }
            SENSOR_PRINT("SENSOR: Delay %d ms", data);	    	
        }
    }

}
*/

/*****************************************************************************/
//  Description:    This function is used to read value from sensor register     
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
/*PUBLIC uint16_t Sensor_ReadReg(uint16_t subaddr)
{
    int32_t i2c_handle_sensor;
    I2C_DEV dev;
    uint8_t  cmd[2] = {0};    
    uint8_t  addr_w;
    uint8_t  addr_r;
    uint16_t ret_val;
    uint16_t w_cmd_num = 0;
    uint16_t r_cmd_num = 0;
    SENSOR_IOCTL_FUNC_PTR 	read_reg_func;

    read_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->read_reg;

    if(PNULL != read_reg_func)
    {
        ret_val = (uint16)read_reg_func((uint32_t)(subaddr & SENSOR_LOW_SIXTEEN_BIT));
    }
    else
    {
        if(SENSOR_I2C_REG_16BIT==(s_sensor_info_ptr->reg_addr_value_bits&SENSOR_I2C_REG_16BIT))
        {
            cmd[w_cmd_num++] = (uint8_t)((subaddr >>BIT_3)&SENSOR_LOW_EIGHT_BIT);
            cmd[w_cmd_num++] = (uint8_t)(subaddr & SENSOR_LOW_EIGHT_BIT);
        }
        else
        {
            cmd[w_cmd_num++] = (uint8_t)subaddr;
        }

        dev.reg_addr_num = w_cmd_num;

        if(SENSOR_I2C_VAL_16BIT==(s_sensor_info_ptr->reg_addr_value_bits & SENSOR_I2C_VAL_16BIT) )
        {
            r_cmd_num = SENSOR_CMD_BITS_16;
        }
        else
        {
            r_cmd_num = SENSOR_CMD_BITS_8; 	
        }  

        i2c_handle_sensor=RE_I2C_HANDLER();
        SENSOR_TRACE("lh:Sensor_ReadReg: handle=%d", i2c_handle_sensor);
        I2C_HAL_Read(i2c_handle_sensor, cmd, &cmd[0], r_cmd_num);

        ret_val = (r_cmd_num == 1)?(uint16)cmd[0]:(uint16)((cmd[0] << 8) + cmd[1]);  

        SENSOR_TRACE("read reg %04x, val %04x", subaddr, ret_val);
    }

    return  ret_val;
}
*/

/*****************************************************************************/
//  Description:    This function is used to send a table of register to sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SendRegTabToSensor(SENSOR_REG_TAB_INFO_T * sensor_reg_tab_info_ptr	)
{
    uint32_t i;

/*    SENSOR_PRINT("SENSOR: Sensor_SendRegValueToSensor -> reg_count = %d start time = %d",\
//                    sensor_reg_tab_info_ptr->reg_count,\
    SCI_GetTickCount());

    SENSOR_ASSERT(PNULL != (void*)sensor_reg_tab_info_ptr);
    SENSOR_ASSERT(PNULL != (void *)sensor_reg_tab_info_ptr->sensor_reg_tab_ptr);
*/
    for(i = 0; i < sensor_reg_tab_info_ptr->reg_count; i++)
    {
        ImgSensor_GetMutex();
        Sensor_WriteReg(sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr, \
                        sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_value);
        ImgSensor_PutMutex();
    }	

//    SENSOR_PRINT("SENSOR: Sensor_SendRegValueToSensor -> end time = %d", SCI_GetTickCount());

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

    //s_sensor_info_ptr=PNULL;
    s_sensor_info_ptr = SENSOR_MALLOC(SENSOR_ID_MAX * sizeof(s_sensor_list_ptr[0]), GFP_KERNEL);

    s_sensor_init=SENSOR_FALSE;
    s_sensor_open=SENSOR_FALSE; 

    s_atv_init=SENSOR_FALSE;
    s_atv_open=SENSOR_FALSE;
	

    SENSOR_MEMSET(s_sensor_list_ptr, 0x00, SENSOR_ID_MAX * sizeof(s_sensor_list_ptr[0]));    
    SENSOR_MEMSET(&s_sensor_exp_info, 0x00, sizeof(s_sensor_exp_info));
    SENSOR_MEMSET(sensor_register_info_ptr, 0x00, sizeof(s_sensor_register_info_ptr));  
    sensor_register_info_ptr->cur_id=SENSOR_ID_MAX; 

    return ;
}

/*****************************************************************************/
//  Description:    This function is used to set currect sensor id    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL void _Sensor_SetId(SENSOR_ID_E sensor_id)
{
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;
    
    sensor_register_info_ptr->cur_id=sensor_id;
    
    return ;
}

/*****************************************************************************/
//  Description:    This function is used to get currect sensor id
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_ID_E Sensor_GetCurId(void)
{
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

    return (SENSOR_ID_E)sensor_register_info_ptr->cur_id;
}

/*****************************************************************************/
//  Description:    This function is used to set all sensor power down
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
LOCAL uint32_t _Sensor_SetAllPowerDown(void)
{
    uint8_t cur_sensor_id=0x00;
    uint8_t sensor_id=0x00;

    cur_sensor_id=Sensor_GetCurId();

    for(sensor_id=0x00; sensor_id<SENSOR_ID_MAX; sensor_id++)
    {
        if(DCAM_NULL == s_sensor_list_ptr[sensor_id])
        {
            continue;
        }
        
        _Sensor_SetId((SENSOR_ID_E)sensor_id);

        Sensor_PowerDown(SENSOR_TRUE);
    }

    _Sensor_SetId((SENSOR_ID_E)cur_sensor_id);

    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to set currect sensor id and set sensor
//                  information
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetCurId(SENSOR_ID_E sensor_id)
{
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

    SENSOR_TRACE("Sensor_SetCurId : %d", sensor_id);	
    if(sensor_id >= SENSOR_ID_MAX)
    {
        _Sensor_CleanInformation();
        return SENSOR_OP_PARAM_ERR;
    }
    if(SENSOR_FALSE==sensor_register_info_ptr->is_register[sensor_id])
    {
        return SENSOR_OP_PARAM_ERR;
    }
    if(sensor_register_info_ptr->cur_id != sensor_id)
    {
        if(Sensor_IsOpen())
        {
            _Sensor_SetAllPowerDown();
            _Sensor_IicHandlerRelease();
            s_sensor_info_ptr=s_sensor_list_ptr[sensor_id];
            Sensor_SetExportInfo(&s_sensor_exp_info);
            _Sensor_SetId(sensor_id);
            _Sensor_IicHandlerInit();
            Sensor_PowerDown(SENSOR_FALSE);
            SENSOR_Sleep(20);
        }
        else
        {
            s_sensor_info_ptr=s_sensor_list_ptr[sensor_id];
            Sensor_SetExportInfo(&s_sensor_exp_info);
            _Sensor_SetId(sensor_id);
        }
       
    }
    SENSOR_PRINT("SENSOR:Sensor_SetCurId id:%d, num:%d, ptr: 0x%x, ptr: 0x%x", sensor_id, sensor_register_info_ptr->img_sensor_num,(uint32_t)s_sensor_list_ptr, (uint32_t)s_sensor_info_ptr);            
    
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
//  Description:    This function is used to initialize Sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_Init(void)
{
//scan_i2c_init();

    BOOLEAN ret_val=SENSOR_FALSE;   
    SENSOR_INFO_T* sensor_info_ptr=PNULL;
    SENSOR_INFO_T** sensor_info_tab_ptr=PNULL;
    //I2C_DEV  dev={0x00};
    uint8_t sensor_index = 0x0;
    uint32_t valid_tab_index_max=0x00;
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=s_sensor_register_info_ptr;

    SENSOR_PRINT("SENSOR: Sensor_Init \n");

    if(Sensor_IsInit())
    {
        SENSOR_PRINT("SENSOR: Sensor_Init is done\n");        
        return SENSOR_TRUE;
    }
    //Clean the information of img sensor
    //nsor_SetCurId(SENSOR_ID_MAX); 
    _Sensor_CleanInformation();
    if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
    {
        // main img sensor
        SENSOR_PRINT("SENSOR: Sensor_Init Main Identify \n");
        _Sensor_SetId(SENSOR_MAIN);
        sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(SENSOR_MAIN);
        valid_tab_index_max=Sensor_GetInforTabLenght(SENSOR_MAIN)-SENSOR_ONE_I2C;

        for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
        {  
            sensor_info_ptr = sensor_info_tab_ptr[sensor_index];

            if(DCAM_NULL==sensor_info_ptr)
            {
                SENSOR_PRINT("SENSOR: Sensor_Init main %d info is null", sensor_index);
                continue ;
            }

            s_sensor_info_ptr = sensor_info_ptr;

            _Sensor_IicHandlerInit();

            Sensor_PowerOn(SENSOR_TRUE);

            if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
            {
                ImgSensor_GetMutex();
                if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
                {
                    s_sensor_list_ptr[SENSOR_MAIN]=s_sensor_info_ptr; 
                    sensor_register_info_ptr->is_register[SENSOR_MAIN]=SENSOR_TRUE;
                    sensor_register_info_ptr->img_sensor_num++;
                    ImgSensor_PutMutex();
                    Sensor_PowerOn(SENSOR_FALSE);
                    _Sensor_IicHandlerRelease();
		    SENSOR_PRINT("SENSOR: _Sensor_IicHandlerRelease SENSOR_MAIN break. \n");
                    break ;
                }
                ImgSensor_PutMutex();
            }

            _Sensor_IicHandlerRelease();

            Sensor_PowerOn(SENSOR_FALSE);
        }

        // sub img sensor
        SENSOR_PRINT("SENSOR: Sensor_Init Sub Identify \n");

        _Sensor_SetId(SENSOR_SUB);

        sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(SENSOR_SUB);
        valid_tab_index_max=Sensor_GetInforTabLenght(SENSOR_SUB)-SENSOR_ONE_I2C;

        for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
        {  
            sensor_info_ptr = sensor_info_tab_ptr[sensor_index];

            if(DCAM_NULL==sensor_info_ptr)
            {
                SENSOR_PRINT("SENSOR: Sensor_Init sub %d info is null", sensor_index);
                continue ;
            }

            s_sensor_info_ptr=sensor_info_ptr;

            _Sensor_IicHandlerInit();

            Sensor_PowerOn(SENSOR_TRUE);
            
            if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
            {
                ImgSensor_GetMutex();
                if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
                {
                    s_sensor_list_ptr[SENSOR_SUB]=s_sensor_info_ptr; 
                    sensor_register_info_ptr->is_register[SENSOR_SUB]=SENSOR_TRUE;
                    sensor_register_info_ptr->img_sensor_num++;
                    ImgSensor_PutMutex();
                    Sensor_PowerOn(SENSOR_FALSE);
                    _Sensor_IicHandlerRelease();
                   SENSOR_PRINT("SENSOR: _Sensor_IicHandlerRelease SENSOR_SUB break. \n");
                    break ;
                }
                ImgSensor_PutMutex();
            }

            _Sensor_IicHandlerRelease();

            Sensor_PowerOn(SENSOR_FALSE);

        }

        if(PNULL!=s_sensor_list_ptr[SENSOR_MAIN])
        {
            s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
            Sensor_SetExportInfo(&s_sensor_exp_info);
            _Sensor_SetId(SENSOR_MAIN);

            s_sensor_init = SENSOR_TRUE;
            ret_val=SENSOR_TRUE;
            SENSOR_PRINT("SENSOR: Sensor_Init Main Success \n");
        }
        else if(PNULL!=s_sensor_list_ptr[SENSOR_SUB])
        {
            s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
            Sensor_SetExportInfo(&s_sensor_exp_info);
            _Sensor_SetId(SENSOR_SUB);
            
            s_sensor_init = SENSOR_TRUE;
            ret_val=SENSOR_TRUE;
            SENSOR_PRINT("SENSOR: Sensor_Init Sub Success \n");
        }    
        else
        {
            _Sensor_SetId(SENSOR_ID_MAX);
            s_sensor_init = SENSOR_FALSE;
            SENSOR_PRINT("SENSOR: Sensor_Init Fail No Sensor err \n");
        }
    }
    else if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
    {
        sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(SENSOR_ATV);
        valid_tab_index_max=Sensor_GetInforTabLenght(SENSOR_ATV)-0x01;

        for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
        {  
            sensor_info_ptr=sensor_info_tab_ptr[sensor_index];

            if(DCAM_NULL==sensor_info_ptr)
            {
                SENSOR_PRINT("SENSOR: Sensor_Init atv %d info is null", sensor_index);            
                continue ;
            }

            s_sensor_info_ptr=sensor_info_ptr;

            _Sensor_IicHandlerInit();

            Sensor_PowerOn(SENSOR_TRUE);
            
            if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
            {
                ImgSensor_GetMutex();
                if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                {
                    s_sensor_list_ptr[SENSOR_ATV]=s_sensor_info_ptr; 
                    sensor_register_info_ptr->is_register[SENSOR_ATV]=SENSOR_TRUE;
                    ImgSensor_PutMutex();
                    Sensor_PowerOn(SENSOR_FALSE);
                    _Sensor_IicHandlerRelease();
                    break ;
                }
                ImgSensor_PutMutex();
            }

            _Sensor_IicHandlerRelease();

            Sensor_PowerOn(SENSOR_FALSE);

        }

        if(SENSOR_TRUE==sensor_register_info_ptr->is_register[SENSOR_ATV])
        {
            s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_ATV];
            Sensor_SetExportInfo(&s_sensor_exp_info);
            _Sensor_SetId(SENSOR_ATV);
            s_atv_init=SENSOR_TRUE;
            s_atv_open=SENSOR_FALSE;
            ret_val=SENSOR_TRUE;
            SENSOR_PRINT("SENSOR: Sensor_Init ATV Success \n");
        }
        else
        {
            _Sensor_SetId(SENSOR_ID_MAX);
            s_atv_init=SENSOR_FALSE;
            s_atv_open=SENSOR_FALSE;
            SENSOR_PRINT("SENSOR: Sensor_Init Fail not ATV err\n");
        }

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
    if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
    {
	    return s_sensor_init;
    }
    else if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
    {
	    return s_atv_init;
    }

    return SENSOR_FALSE;
}

/*****************************************************************************/
//  Description:    This function is used to Open sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_Open(void)
{
    //I2C_DEV  dev={0x00};
    SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr=Sensor_GetRegisterInfo();
    SENSOR_ID_E cur_sensor_id=SENSOR_ID_MAX;
    

	SENSOR_PRINT("SENSOR: Sensor_Open sensor type: %d id: %d.\n", _Sensor_GetSensorType(), Sensor_GetCurId());

    if(!Sensor_IsInit())
    {
        SENSOR_PRINT("SENSOR: Sensor_Open -> sensor has not init");
        s_atv_open=SENSOR_FALSE;
        s_sensor_open=SENSOR_FALSE;
        s_sensor_mode[SENSOR_MAIN]=SENSOR_MODE_MAX;
        s_sensor_mode[SENSOR_SUB]=SENSOR_MODE_MAX;
        return SENSOR_OP_STATUS_ERR;
    }
  
    if(Sensor_IsOpen())
    {
    	SENSOR_PRINT("SENSOR: Sensor_Open -> sensor has open");
    }
    else
    {       
        if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
        {
            cur_sensor_id=Sensor_GetCurId();

            if(SENSOR_MAIN==cur_sensor_id && 
               SENSOR_TRUE==sensor_register_info_ptr->is_register[SENSOR_MAIN])
            {
                s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
                Sensor_SetExportInfo(&s_sensor_exp_info);
                _Sensor_SetId(SENSOR_MAIN);
                _Sensor_IicHandlerInit();  
                Sensor_PowerOn(SENSOR_TRUE);

                if(PNULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify)
                {
                    if(SENSOR_SUCCESS!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                    {
                        Sensor_PowerDown(SENSOR_TRUE);
                        _Sensor_IicHandlerRelease();
                        return SENSOR_OP_ERR;
                    }
                }

                if(SENSOR_TRUE==sensor_register_info_ptr->is_register[SENSOR_SUB])
                {
                    /* Sub sensor exist*/
                    
                    /* First, put the main sensor into power down,
                       to avoid any disturber from main sensor while initilize the sub sensor */
                       
                    Sensor_PowerDown(SENSOR_TRUE);
                    _Sensor_IicHandlerRelease();
                    
                    /*then initilize the sub sensor,and put it into power down*/
                    s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
                    Sensor_SetExportInfo(&s_sensor_exp_info);            
                    _Sensor_SetId(SENSOR_SUB);
                    _Sensor_IicHandlerInit();  
                    Sensor_PowerOn(SENSOR_TRUE);

                    if(PNULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify)
                    {
                        if(SENSOR_SUCCESS==s_sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                        {
                            Sensor_SetMode(SENSOR_MODE_COMMON_INIT);
                        }
                    }

                    Sensor_PowerDown(SENSOR_TRUE);                    
                    _Sensor_IicHandlerRelease();

                    /*the end ,recorver the main sensor as current sensor*/
                    s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
                    Sensor_SetExportInfo(&s_sensor_exp_info);            
                    _Sensor_SetId(SENSOR_MAIN);
                    Sensor_PowerDown(SENSOR_FALSE);
                    _Sensor_IicHandlerInit();  

                }

                Sensor_SetMode(SENSOR_MODE_COMMON_INIT);
            }
            else if(SENSOR_SUB==cur_sensor_id && 
                    SENSOR_TRUE==sensor_register_info_ptr->is_register[SENSOR_SUB])
            {
                s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
                Sensor_SetExportInfo(&s_sensor_exp_info);            
                _Sensor_SetId(SENSOR_SUB);
                _Sensor_IicHandlerInit();  
                Sensor_PowerOn(SENSOR_TRUE);

                if(PNULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify)
                {
                    if(SENSOR_SUCCESS!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                    {
                        Sensor_PowerDown(SENSOR_TRUE);
                        _Sensor_IicHandlerRelease();
                        return SENSOR_OP_ERR;
                    }
                }
            
                if(SENSOR_TRUE==sensor_register_info_ptr->is_register[SENSOR_MAIN])
                {
                    /* Main sensor exist*/
                    
                    /* First, put the sub sensor into power down,
                       to avoid any disturber from sub sensor while initilize the main sensor */
                
                    Sensor_PowerDown(SENSOR_TRUE);
                    _Sensor_IicHandlerRelease();

                    /*then initilize the main sensor,and put it into power down*/
                    s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_MAIN];
                    Sensor_SetExportInfo(&s_sensor_exp_info);
                    _Sensor_SetId(SENSOR_MAIN);
                    _Sensor_IicHandlerInit();  
                    Sensor_PowerOn(SENSOR_TRUE);

                    if(PNULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify)
                    {
                        if(SENSOR_SUCCESS==s_sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                        {
                            Sensor_SetMode(SENSOR_MODE_COMMON_INIT);
                        }
                    }

                    Sensor_PowerDown(SENSOR_TRUE);
                    _Sensor_IicHandlerRelease();

                    /*the end ,recorver the sub sensor as current sensor*/
                    s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_SUB];
                    Sensor_SetExportInfo(&s_sensor_exp_info);
                    _Sensor_SetId(SENSOR_SUB);
                    Sensor_PowerDown(SENSOR_FALSE);
                    _Sensor_IicHandlerInit();                       
                }

                Sensor_SetMode(SENSOR_MODE_COMMON_INIT);
            }

            s_sensor_open=SENSOR_TRUE;

        }
        else if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
        {
            if(SENSOR_ATV!=Sensor_GetCurId())
            {
                s_sensor_info_ptr=s_sensor_list_ptr[SENSOR_ATV];
                Sensor_SetExportInfo(&s_sensor_exp_info);
                _Sensor_SetId(SENSOR_ATV);
            }

            _Sensor_IicHandlerInit();
            
            //open atv
            if(DCAM_NULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->cus_func_1)
            {
                uint32_t param=0x00;     

                Sensor_PowerOn(SENSOR_TRUE);

                if(PNULL!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify)
                {
                    if(SENSOR_SUCCESS!=s_sensor_info_ptr->ioctl_func_tab_ptr->identify(0x00))
                    {
                        Sensor_PowerDown(SENSOR_TRUE);
                        _Sensor_IicHandlerRelease();
                        return SENSOR_OP_ERR;
                    }
                }

                param=(ATV_CMD_CHIP_INIT<<BIT_4)&SENSOR_HIGN_SIXTEEN_BIT;  
                if(SENSOR_SUCCESS!=s_sensor_info_ptr->ioctl_func_tab_ptr->cus_func_1(param))
                {
                    s_atv_open=SENSOR_FALSE;
                    Sensor_PowerOn(SENSOR_FALSE);
                    return SENSOR_OP_ERR;
                }
            }

            s_atv_open=SENSOR_TRUE;
            
       }        
  }
    
return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to set sensor work-mode    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SetMode(SENSOR_MODE_E mode)
{
    uint32_t mclk;

    if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
    {
        SENSOR_PRINT("SENSOR: Sensor_SetMode -> mode = %d.\n", mode);
        
        if(!Sensor_IsInit())
        {
            SENSOR_PRINT("SENSOR: Sensor_SetResolution -> sensor has not init");
            return SENSOR_OP_STATUS_ERR;
        }	
        
        if(s_sensor_mode[Sensor_GetCurId()] == mode)
        {
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
        SENSOR_PRINT("SENSOR: Sensor_Ioctl -> sensor has not init");
        return SENSOR_OP_STATUS_ERR;
    }

    if(SENSOR_IOCTL_CUS_FUNC_1 > cmd) 
    {
        SENSOR_PRINT("SENSOR: Sensor_Ioctl - > can't access internal command !");
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
        SENSOR_PRINT("SENSOR: Sensor_Ioctl -> the ioctl function has not register err!");
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

    if(Sensor_IsInit())
    {
        if(Sensor_IsOpen())
        {
            if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
            {
                uint32_t param=0x00;
                param=(ATV_CMD_CLOSE<<BIT_4)&SENSOR_HIGN_SIXTEEN_BIT;
                s_sensor_info_ptr->ioctl_func_tab_ptr->cus_func_1(param);
            }
            Sensor_PowerOn(SENSOR_FALSE);
            _Sensor_IicHandlerRelease();
        }
    }
    s_sensor_init = SENSOR_FALSE;//wxz:???
    s_atv_open=SENSOR_FALSE;
    s_sensor_open=SENSOR_FALSE;
    s_sensor_mode[SENSOR_MAIN]=SENSOR_MODE_MAX;	
    s_sensor_mode[SENSOR_SUB]=SENSOR_MODE_MAX;	
    return SENSOR_SUCCESS;
}

/*****************************************************************************/
//  Description:    This function is used to Close sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN  Sensor_IsOpen(void) 
{
    if(SENSOR_TYPE_IMG_SENSOR==_Sensor_GetSensorType())
    {
	    return s_sensor_open;
    }
    else if(SENSOR_TYPE_ATV==_Sensor_GetSensorType())
    {
	    return s_atv_open;
    }

    return SENSOR_FALSE;
        
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
PUBLIC uint32_t RE_I2C_HANDLER(void)
{
    return s_sensor_info_ptr->i2c_dev_handler;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor exif info    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
/*PUBLIC uint32_t Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_E cmd ,uint32_t param)
{
    SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
    EXIF_SPEC_PIC_TAKING_COND_T* sensor_exif_info_ptr=PNULL;

    if(PNULL!=sensor_info_ptr->ioctl_func_ptr->get_exif)
    {
        sensor_exif_info_ptr=(EXIF_SPEC_PIC_TAKING_COND_T*)sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);
    }
    else
    {
        return SCI_ERROR;
    }

    switch(cmd)
    {
        case SENSOR_EXIF_CTRL_EXPOSURETIME:
        {
            SENSOR_MODE_E img_sensor_mode=s_sensor_mode[Sensor_GetCurId()];
            uint32_t exposureline_time=sensor_info_ptr->sensor_mode_info[img_sensor_mode].line_time;
            uint32_t exposureline_num=param;
            uint32_t exposure_time=0x00;

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
                uint32_t second=0x00;
                do
                {
                    second++;
                    exposure_time-=1000000;
                    if(1000000>=exposure_time)
                    {
                        break;
                    }
                }while(1);//lint !e506

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
*/

/*****************************************************************************/
//  Description:    This function is used to get sensor exif info    
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
/*PUBLIC EXIF_SPEC_PIC_TAKING_COND_T* Sensor_GetSensorExifInfo(void)
{
    SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
    EXIF_SPEC_PIC_TAKING_COND_T* sensor_exif_info_ptr=PNULL;

    if(PNULL!=sensor_info_ptr->ioctl_func_ptr->get_exif)
    {
        sensor_exif_info_ptr=(EXIF_SPEC_PIC_TAKING_COND_T*)sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);
    }

    return sensor_exif_info_ptr;
}
*/

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
    
#endif  // End of sensor_drv.c
