/******************************************************************************
 ** Copyright (c) 
 ** File Name:		sensor_GC0309.c 										  *
 ** Author: 													  *
 ** DATE:															  *
 ** Description:   This file contains driver for sensor GC0309. 
 ** 														 
 ******************************************************************************

 ******************************************************************************
 ** 					   Edit History 									  *
 ** ------------------------------------------------------------------------- *
 ** DATE		   NAME 			DESCRIPTION 							  *
 ** 	  
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 ** 						Dependencies									  *
 **---------------------------------------------------------------------------*/
#include "sensor_cfg.h"
#include "sensor_drv.h"
//#include "i2c_drv.h"
//#include "os_api.h"
//#include "chip.h"
//#include "dal_dcamera.h"
#include <linux/delay.h>

/**---------------------------------------------------------------------------*
 ** 						Compiler Flag									  *
 **---------------------------------------------------------------------------*/
#ifdef	 __cplusplus
	extern	 "C" 
	{
#endif
/**---------------------------------------------------------------------------*
 ** 					Extern Function Declaration 						  *
 **---------------------------------------------------------------------------*/
//extern uint32_t OS_TickDelay(uint32_t ticks);
/*
extern void Sensor_SetMCLK(uint32_t mclk);
*/
//extern ERR_I2C_E I2C_WriteCmdArrNoStop(uint8_t addr, uint8_t *pCmd, uint32_t len, BOOLEAN ack_en);

/**---------------------------------------------------------------------------*
 ** 						Const variables 								  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 ** 						   Macro Define
 **---------------------------------------------------------------------------*/
#define GC0309_I2C_ADDR_W	0x21 // 0x42 --> 0x42 / 2
#define GC0309_I2C_ADDR_R		0x21 // 0x43 --> 0x43 / 2

#define SENSOR_GAIN_SCALE		16

 
/**---------------------------------------------------------------------------*
 ** 					Local Function Prototypes							  *
 **---------------------------------------------------------------------------*/
LOCAL uint32_t set_gc0309_ae_enable(uint32_t enable);
LOCAL uint32_t set_hmirror_enable(uint32_t enable);
LOCAL uint32_t set_vmirror_enable(uint32_t enable);
LOCAL uint32_t set_preview_mode(uint32_t preview_mode);
LOCAL uint32_t GC0309_Identify(uint32_t param);
/*
LOCAL uint32_t GC0309_BeforeSnapshot(uint32_t param);
LOCAL uint32_t GC0309_After_Snapshot(uint32_t param);
*/
LOCAL uint32_t set_brightness(uint32_t level);
LOCAL uint32_t set_contrast(uint32_t level);
LOCAL uint32_t set_sharpness(uint32_t level);
LOCAL uint32_t set_saturation(uint32_t level);
LOCAL uint32_t set_image_effect(uint32_t effect_type);

LOCAL uint32_t read_ev_value(uint32_t value);
LOCAL uint32_t write_ev_value(uint32_t exposure_value);
LOCAL uint32_t read_gain_value(uint32_t value);
LOCAL uint32_t write_gain_value(uint32_t gain_value);
LOCAL uint32_t read_gain_scale(uint32_t value);
LOCAL uint32_t set_frame_rate(uint32_t param);
LOCAL uint32_t GC0309_set_work_mode(uint32_t mode);

LOCAL uint32_t set_gc0309_ev(uint32_t level);
LOCAL uint32_t set_gc0309_awb(uint32_t mode);
LOCAL uint32_t set_gc0309_anti_flicker(uint32_t mode);
LOCAL uint32_t set_gc0309_video_mode(uint32_t mode);

//LOCAL BOOLEAN gc_enter_effect = SENSOR_FALSE;

/**---------------------------------------------------------------------------*
 ** 						Local Variables 								 *
 **---------------------------------------------------------------------------*/
 typedef enum
{
	FLICKER_50HZ = 0,
	FLICKER_60HZ,
	FLICKER_MAX
}FLICKER_E;
//__align(4) const SENSOR_REG_T gc0309_YUV_640X480[]=
SENSOR_REG_T gc0309_YUV_640X480[]=
{

	{0xfe,0x80},   	
		
	{0xfe,0x00},       // set page0
	
	{0x1a,0x26},   	
	{0xd2,0x10},   // close AEC
	{0x22,0x55},   //close AWB	

	{0x5a,0x56}, 
	{0x5b,0x40},
	{0x5c,0x4a},		
	
	{0x22,0x57}, 
		
	{0x01,0x6a}, 
	{0x02,0x70}, 
	{0x0f,0x00},

	{0x03,0x01},   
	{0x04,0x2c},  

	{0xe2,0x00}, 
	{0xe3,0x96}, 
	/*{0xe4,0x03}, //16 fps
	{0xe5,0x84}, 
	{0xe6,0x03}, 
	{0xe7,0x84}, 
	{0xe8,0x03}, 
	{0xe9,0x84},*/ 
	{0xe4,0x05}, //10 fps
	{0xe5,0xdc}, 
	{0xe6,0x05}, 
	{0xe7,0xdc}, 
	{0xe8,0x05}, 
	{0xe9,0xdc}, 
	{0xea,0x05}, 
	{0xeb,0xdc}, 
#if 0	
	{0xe4,0x02}, 
	{0xe5,0xbc}, 
	{0xe6,0x02}, 
	{0xe7,0xbc}, 
	{0xe8,0x02}, 
	{0xe9,0xbc}, 
	{0xea,0x09}, 
	{0xeb,0xc4}, 
#endif
	{0x05,0x00},
	{0x06,0x00},
	{0x07,0x00}, 
	{0x08,0x00}, 
	{0x09,0x01}, 
	{0x0a,0xe8}, 
	{0x0b,0x02}, 
	{0x0c,0x88}, 
	{0x0d,0x00}, 
	{0x0e,0x00}, 
	{0x10,0x26}, 
	{0x11,0x0d}, 
	{0x12,0x2a}, 
	{0x13,0x00}, 
	{0x15,0x0a}, 
	{0x16,0x05}, 
	{0x17,0x01}, 

	{0x1b,0x03}, 
	{0x1c,0x49}, 
	{0x1d,0x98}, 
	{0x1e,0x20}, 
	//{0x1f,0x1e}, //wxz20110627: change the pclk driver
	{0x1f,0x1f}, 

	{0x20,0xff}, 
	{0x21,0xf8}, 
	{0x24,0xa2}, 
	{0x25,0x0f},
	//output sync_mode
	{0x26,0x03}, 
	{0x2f,0x01}, 
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// grab_t ////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x30,0xf7}, 
	{0x31,0x60},
	{0x32,0x00}, 
	{0x39,0x04},  
	{0x3a,0x20}, 
	{0x3b,0x20},
	{0x3c,0x02}, 
	{0x3d,0x02}, 
	{0x3e,0x02},
	{0x3f,0x02}, 
	
	//gain
	{0x50,0x22}, //0x24
	
	{0x53,0x82}, 
	{0x54,0x80}, 
	{0x55,0x80}, 
	{0x56,0x82}, 

	{0x57,0x80}, 
	{0x58,0x82},
	{0x59,0x82},		


	/////////////////////////////////////////////////////////////////////
	/////////////////////////// LSC_t  ////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x8b,0x7a}, //20 tony 2010.03.09
	{0x8c,0x7a}, //20 tony 2010.03.09
	{0x8d,0x7a}, //20 tony 2010.03.09
	{0x8e,0x4a}, //10 tony 2010.03.09
	{0x8f,0x4a}, //10 tony 2010.03.09
	{0x90,0x4a}, //10 tony 2010.03.09
	{0x91,0x42},   //2d  3f
	{0x92,0x48},  // 41
	{0x5d,0x12},   //0x12
	{0x5e,0x1a},  // 0x1a
	{0x5f,0x24}, // 0x24

	/////////////////////////////////////////////////////////////////////
	/////////////////////////// DNDD_t  ///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x60,0x07}, 
	{0x61,0x0e}, //0x0e
	{0x62,0x12}, //0c  tony 2010.03.09
	{0x64,0x02}, //03  tony 2010.03.09
	{0x66,0xe8}, 
	{0x67,0x86}, 
	{0x68,0xa2}, 
	
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// asde_t ///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x69,0x20}, 
	{0x6a,0x0f}, 
	{0x6b,0x00}, 
	{0x6c,0x53}, 
	{0x6d,0x83}, 
	{0x6e,0xac}, 
	{0x6f,0xac}, 
	{0x70,0x15}, 
	{0x71,0x33}, 
	/////////////////////////////////////////////////////////////////////
	/////////////////////////// eeintp_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x72,0xdc},  
	{0x73,0x80},  
	//for high resolution in light scene
	{0x74,0x02}, 
	{0x75,0x3f}, 
	{0x76,0x02}, 
	{0x77,0x78}, //0x54
	{0x78,0x88}, 
	{0x79,0x81}, 
	{0x7a,0x81}, 
	{0x7b,0x22}, 
	{0x7c,0xff},
	
	
	/////////////////////////////////////////////////////////////////////
	///////////////////////////CC_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0x93,0x40}, 
	{0x94,0xfb},//00 
	{0x95,0xfb}, 
	{0x96,0x00}, 
	{0x97,0x45}, 
	{0x98,0xf0}, 
	{0x9c,0x00}, 
	{0x9d,0x03}, 
	{0x9e,0x00}, 
	
	
	/////////////////////////////////////////////////////////////////////
	///////////////////////////YCP_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0xb1,0x52},  //  56 40  tony 2010.03.09 
	{0xb2,0x50},  //56  40  tony 2010.03.09
	{0xb8,0x20}, 
	{0xbe,0x36}, 
	{0xbf,0x00}, 
	/////////////////////////////////////////////////////////////////////
	///////////////////////////AEC_t///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	{0xd0,0xcb},  //0xc9
	{0xd1,0x10},  

	{0xd3,0x58},  //0x80
	{0xd5,0xf2}, 
	{0xd6,0x16},    
	{0xdb,0x92}, 
	{0xdc,0xa5},  
	{0xdf,0x23},   
	{0xd9,0x00},  
	{0xda,0x00},  
	{0xe0,0x09},

	{0xec,0x20},  
	{0xed,0x04},  
	{0xee,0xa0},  
	{0xef,0x40},  
	///////////////////////////////////////////////////////////////////
	///////////////////////////GAMMA//////////////////////////////////
	///////////////////////////////////////////////////////////////////


	{0x9F,0x0e},           // gamma curve lvl3
	{0xA0,0x1c},  
	{0xA1,0x34},
	{0xA2,0x48},
	{0xA3,0x5a},
	{0xA4,0x6b},
	{0xA5,0x7b},
	{0xA6,0x95},
	{0xA7,0xab},
	{0xA8,0xbf},
	{0xA9,0xce},
	{0xAA,0xd9},
	{0xAB,0xe4},
	{0xAC,0xec},
	{0xAD,0xf7},
	{0xAE,0xfd},
	{0xAF,0xff},

	//Y_gamma
	{0xc0,0x00},
	{0xc1,0x0B},
	{0xc2,0x15},
	{0xc3,0x27},
	{0xc4,0x39},
	{0xc5,0x49},
	{0xc6,0x5A},
	{0xc7,0x6A},
	{0xc8,0x89},
	{0xc9,0xA8},
	{0xca,0xC6},
	{0xcb,0xE3},
	{0xcc,0xFF},

	/////////////////////////////////////////////////////////////////
	/////////////////////////// ABS_t ///////////////////////////////
	/////////////////////////////////////////////////////////////////
	{0xf0,0x02},
	{0xf1,0x01},
	{0xf2,0x00},  //00  tony 2010.03.09
	{0xf3,0x30}, 
	
	/////////////////////////////////////////////////////////////////
	/////////////////////////// Measure Window ///////////////////////
	/////////////////////////////////////////////////////////////////
	{0xf7,0x04}, 
	{0xf8,0x02}, 
	{0xf9,0x9f},
	{0xfa,0x78},

	//---------------------------------------------------------------
	{0xfe,0x01},
	
	/////////////////////////////////////////////////////////////////
	///////////////////////////AWB_p/////////////////////////////////
	/////////////////////////////////////////////////////////////////
	{0x00,0xf5}, 
//	{0x01,0x0a},  
	{0x02,0x20}, //1a

	{0x04,0x10},
	{0x05,0x14},
	{0x06,0x20},
	{0x08,0x0a},
	
	{0x0a,0x90}, //a0
	{0x0b,0x60}, 
	{0x0c,0x08},
	{0x0e,0x4c}, 
	{0x0f,0x39}, 
	{0x11,0x3f}, 
	{0x13,0x13}, //72
	{0x14,0x42},  
	{0x15,0x43}, 
	{0x16,0xc2}, 
	{0x17,0xa8}, 
	{0x18,0x18},  
	{0x19,0x40},  
	{0x1a,0xd0}, 
	{0x1b,0xf5},  

	{0x70,0x40}, 
	{0x71,0x58},  
	{0x72,0x30},  
	{0x73,0x48},  
	{0x74,0x20},  
	{0x75,0x60},  
	
	{0xfe,0x00},

	{0xd2,0x90},  // Open AEC
	{0x23,0x00},   
	{0x2d,0x0a}, 
	{0x20,0xff}, 
	{0x73,0x00}, 
	{0x77,0x78},//0x38
	{0xb3,0x3c}, //0x40
	{0xb4,0x80}, 
	{0xb5,0x00},	
	{0xba,0x00}, 
	{0xbb,0x00}, 

	{0x14,0x11},  // Mirror UpsideDown 

    {SENSOR_WRITE_DELAY, 200},//delay 20ms

    { 0xff,0xff},

};


LOCAL SENSOR_REG_TAB_INFO_T s_GC0309_resolution_Tab_YUV[]=
{
	// COMMON INIT
	{ADDR_AND_LEN_OF_ARRAY(gc0309_YUV_640X480), 640, 480, 24, SENSOR_IMAGE_FORMAT_YUV422},
	
	// YUV422 PREVIEW 1	
	{PNULL, 0, 640, 480,24, SENSOR_IMAGE_FORMAT_YUV422},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	
	// YUV422 PREVIEW 2 
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0}

};

LOCAL SENSOR_IOCTL_FUNC_TAB_T s_GC0309_ioctl_func_tab = 
{
    // Internal 
    PNULL,
    PNULL,
    PNULL,
    GC0309_Identify,

    PNULL,			// write register
    PNULL,			// read  register	
    PNULL,
    PNULL,

    // External
    set_gc0309_ae_enable,
    set_hmirror_enable,
    set_vmirror_enable,

    set_brightness,
    set_contrast,
    set_sharpness,
    set_saturation,

    set_preview_mode,	
    set_image_effect,

	PNULL,	//	GC0309_BeforeSnapshot,
    PNULL,		//GC0309_After_Snapshot,

    PNULL,

    read_ev_value,
    write_ev_value,
    read_gain_value,
    write_gain_value,
    read_gain_scale,
    set_frame_rate,	
    PNULL,
    PNULL,
    set_gc0309_awb,
    PNULL,
    PNULL,
    set_gc0309_ev,
    PNULL,
    PNULL,
    PNULL,
    PNULL,
    set_gc0309_anti_flicker,
    set_gc0309_video_mode,
};

/**---------------------------------------------------------------------------*
 ** 						Global Variables								  *
 **---------------------------------------------------------------------------*/
 PUBLIC SENSOR_INFO_T g_GC0309_yuv_info =
{
	GC0309_I2C_ADDR_W,				// salve i2c write address
	GC0309_I2C_ADDR_R, 				// salve i2c read address
	
	0,								// bit0: 0: i2c register value is 8 bit, 1: i2c register value is 16 bit
									// bit2: 0: i2c register addr  is 8 bit, 1: i2c register addr  is 16 bit
									// other bit: reseved
	SENSOR_HW_SIGNAL_PCLK_N|\
	SENSOR_HW_SIGNAL_VSYNC_P|\
	SENSOR_HW_SIGNAL_HSYNC_P,		// bit0: 0:negative; 1:positive -> polarily of pixel clock
									// bit2: 0:negative; 1:positive -> polarily of horizontal synchronization signal
									// bit4: 0:negative; 1:positive -> polarily of vertical synchronization signal
									// other bit: reseved											
											
	// preview mode
	SENSOR_ENVIROMENT_NORMAL|\
	SENSOR_ENVIROMENT_NIGHT|\
	SENSOR_ENVIROMENT_SUNNY,		
	
	// image effect
	SENSOR_IMAGE_EFFECT_NORMAL|\
	SENSOR_IMAGE_EFFECT_BLACKWHITE|\
	SENSOR_IMAGE_EFFECT_RED|\
	SENSOR_IMAGE_EFFECT_GREEN|\
	SENSOR_IMAGE_EFFECT_BLUE|\
	SENSOR_IMAGE_EFFECT_YELLOW|\
	SENSOR_IMAGE_EFFECT_NEGATIVE|\
	SENSOR_IMAGE_EFFECT_CANVAS,
	
	// while balance mode
	0,
		
	7,								// bit[0:7]: count of step in brightness, contrast, sharpness, saturation
									// bit[8:31] reseved
	
	SENSOR_LOW_PULSE_RESET,			// reset pulse level
	100,								// reset pulse width(ms)
	
	SENSOR_HIGH_LEVEL_PWDN,			// 1: high level valid; 0: low level valid	
		
	2,								// count of identify code
	{{0x00, 0xa0},						// supply two code to identify sensor.
	{0x00, 0xa0}},						// for Example: index = 0-> Device id, index = 1 -> version id	
									
	SENSOR_AVDD_1800MV,				// voltage of avdd	

	640,							// max width of source image
	480,							// max height of source image
	"GC0309",						// name of sensor												

	SENSOR_IMAGE_FORMAT_YUV422,		// define in SENSOR_IMAGE_FORMAT_E enum,
									// if set to SENSOR_IMAGE_FORMAT_MAX here, image format depent on SENSOR_REG_TAB_INFO_T
	SENSOR_IMAGE_PATTERN_YUV422_YUYV,	// pattern of input image form sensor;			

	s_GC0309_resolution_Tab_YUV,	// point to resolution table information structure
	&s_GC0309_ioctl_func_tab,		// point to ioctl function table
			
	PNULL,							// information and table about Rawrgb sensor
	PNULL,							// extend information about sensor	
    SENSOR_AVDD_2800MV,                     // iovdd
	SENSOR_AVDD_1500MV,                      // dvdd
	3,
	0,
	0,
	2
	
};
/**---------------------------------------------------------------------------*
 ** 							Function  Definitions
 **---------------------------------------------------------------------------*/
LOCAL void GC0309_WriteReg( uint8_t  subaddr, uint8_t data )
{
	
	#ifndef	_USE_DSP_I2C_
		//uint8_t cmd[2];
		//cmd[0]	=	subaddr;
		//cmd[1]	=	data;		
		//I2C_WriteCmdArr(GC0309_I2C_ADDR_W, cmd, 2, SENSOR_TRUE);
		Sensor_WriteReg_8bits(subaddr, data);
	#else
		DSENSOR_IICWrite((uint16_t)subaddr, (uint16_t)data);
	#endif

	SENSOR_TRACE("SENSOR: GC0309_WriteReg reg/value(%x,%x) !!\n", subaddr, data);

}

LOCAL uint8_t GC0309_ReadReg( uint8_t  subaddr)
{
	uint8_t value = 0;
	
	#ifndef	_USE_DSP_I2C_
	//I2C_WriteCmdArrNoStop(GC0309_I2C_ADDR_W, &subaddr, 1,SENSOR_TRUE);
	//I2C_ReadCmd(GC0309_I2C_ADDR_R, &value, SENSOR_TRUE);
	//value =Sensor_ReadReg_8bits( subaddr);
	value = Sensor_ReadReg( subaddr);
	#else
		value = (uint16_t)DSENSOR_IICRead((uint16_t)subaddr);
	#endif

    SENSOR_TRACE("SENSOR: GC0309_ReadReg reg/value(%x,%x) !!\n", subaddr, value);
    
	return value;
}


LOCAL uint32_t GC0309_Identify(uint32_t param)
{
#define GC0309_PID_VALUE	0xa0	
#define GC0309_PID_ADDR		0x00
#define GC0309_VER_VALUE	0xa0	
#define GC0309_VER_ADDR		0x00	

	uint32_t i;
	uint32_t nLoop;
	uint8_t ret;
	uint32_t err_cnt = 0;
	uint8_t reg[2] 	= {0x00, 0x00};
	uint8_t value[2] 	= {0xa0, 0xa0};

	SENSOR_TRACE("GC0309_Identify");
	for(i = 0; i<2; )
	{
		nLoop = 1000;
		ret = GC0309_ReadReg(reg[i]);
		if( ret != value[i])
		{
			err_cnt++;
			if(err_cnt>3)			
			{
				SENSOR_TRACE("It is not GC0309\n");
				return SENSOR_FAIL;
			}
			else
			{
				//Masked by frank.yang,SENSOR_Sleep() will cause a  Assert when called in boot precedure
				//SENSOR_Sleep(10);
				while(nLoop--);
				continue;
			}
		}
        	err_cnt = 0;
		i++;
	}

	SENSOR_TRACE("GC0309_Identify: it is GC0309\n");
	
	return (uint32_t)SENSOR_SUCCESS;
}

LOCAL uint32_t set_gc0309_ae_enable(uint32_t enable)
{
	SENSOR_TRACE("set_gc0309_ae_enable: enable = %d\n", enable);
	return 0;
}


LOCAL uint32_t set_hmirror_enable(uint32_t enable)
{
 	uint8_t value = 0;	
	value = GC0309_ReadReg(0x14);
	value = (value & 0xFE) | (enable == 1 ? 0 : 1); //landscape
	SENSOR_TRACE("set_hmirror_enable: enable = %d, 0x14: 0x%x.\n", enable, value);
	GC0309_WriteReg(0x14, value);
	
	return 0;
}


LOCAL uint32_t set_vmirror_enable(uint32_t enable)
{
	uint8_t value = 0;	
	value = GC0309_ReadReg(0x14);
	value = (value & 0xFD) | ((enable & 0x1) << 1); //portrait
	SENSOR_TRACE("set_vmirror_enable: enable = %d, 0x14: 0x%x.\n", enable, value);
	GC0309_WriteReg(0x14, value);
	
	return 0;
}
/******************************************************************************/
// Description: set brightness 
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/
//__align(4) const SENSOR_REG_T gc0309_brightness_tab[][2]=
SENSOR_REG_T gc0309_brightness_tab[][2]=
{
		{		
			{0xb5, 0xd0},	{0xff,0xff},
		},
	
		{
			{0xb5, 0xe0},	{0xff,0xff},
		},
	
		{
			{0xb5, 0xf0},	{0xff,0xff},
		},
	
		{
			{0xb5, 0x00},	{0xff,0xff},
		},
	
		{
			{0xb5, 0x20},	{0xff,0xff},
		},
	
		{
			{0xb5, 0x30},	{0xff,0xff},
		},
	
		{
			{0xb5, 0x40},	{0xff,0xff},
		},
};


LOCAL uint32_t set_brightness(uint32_t level)
{
	uint16_t i;
	SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)gc0309_brightness_tab[level];

	SENSOR_ASSERT(level < 7);
	SENSOR_ASSERT(PNULL != sensor_reg_ptr);
	
	for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) && (0xFF != sensor_reg_ptr[i].reg_value); i++)
	{
		GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	}
	msleep(100); 
	SENSOR_TRACE("set_brightness: level = %d\n", level);
	
	return 0;
}

//__align(4) const SENSOR_REG_T GC0309_ev_tab[][3]=
SENSOR_REG_T GC0309_ev_tab[][3]=
{   
    {{0xd3, 0x48}, {0xb5, 0xd0},{0xff, 0xff}},
    {{0xd3, 0x50}, {0xb5, 0xe0},{0xff, 0xff}},
    {{0xd3, 0x58}, {0xb5, 0xf0},{0xff, 0xff}},
    {{0xd3, 0x60}, {0xb5, 0x10},{0xff, 0xff}},
    {{0xd3, 0x68}, {0xb5, 0x20},{0xff, 0xff}},
    {{0xd3, 0x70}, {0xb5, 0x30},{0xff, 0xff}},
    {{0xd3, 0x78}, {0xb5, 0x40},{0xff, 0xff}},
    
};


LOCAL uint32_t set_gc0309_ev(uint32_t level)
{
    uint16_t i; 
    SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0309_ev_tab[level];

    SENSOR_ASSERT(PNULL != sensor_reg_ptr);
    SENSOR_ASSERT(level < 7);
 
    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) ||(0xFF != sensor_reg_ptr[i].reg_value) ; i++)
    {
        GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }

    
    SENSOR_TRACE("SENSOR: set_ev: level = %d\n", level);

    return 0;
}

/******************************************************************************/
// Description: anti 50/60 hz banding flicker
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/
LOCAL uint32_t set_gc0309_anti_flicker(uint32_t param )
{
    switch (param)
    {
        case FLICKER_50HZ:
            GC0309_WriteReg(0x01, 0x6A); 	
            GC0309_WriteReg(0x02, 0x06); 
            GC0309_WriteReg(0x0f, 0x01);

            //write_cmos_sensor(0x03  ,0x01); 	
            //write_cmos_sensor(0x04  ,0x2c); 	

            GC0309_WriteReg(0xe2, 0x00); 	//anti-flicker step [11:8]
            GC0309_WriteReg(0xe3, 0x96);   //anti-flicker step [7:0]

            GC0309_WriteReg(0xe4, 0x02);   //exp level 1  16.67fps
            GC0309_WriteReg(0xe5, 0xEE); 
            GC0309_WriteReg(0xe6, 0x02);   //exp level 2  12.5fps
            GC0309_WriteReg(0xe7, 0xEE); 
            GC0309_WriteReg(0xe8, 0x02);   //exp level 3  8.33fps
            GC0309_WriteReg(0xe9, 0xEE); 
            GC0309_WriteReg(0xea, 0x09);   //exp level 4  4.00fps
            GC0309_WriteReg(0xeb, 0x60); 

            break;

        case FLICKER_60HZ:
            GC0309_WriteReg(0x01, 0x2c); 	
            GC0309_WriteReg(0x02, 0x98); 
            GC0309_WriteReg(0x0f, 0x02);

            //GC0309_WriteReg(0x03  ,0x01); 	
            //GC0309_WriteReg(0x04  ,0x40); 	

            GC0309_WriteReg(0xe2, 0x00); 	//anti-flicker step [11:8]
            GC0309_WriteReg(0xe3, 0x50);   //anti-flicker step [7:0]

            GC0309_WriteReg(0xe4, 0x02);   //exp level 1  15.00fps
            GC0309_WriteReg(0xe5, 0x80); 
            GC0309_WriteReg(0xe6, 0x03);   //exp level 2  10.00fps
            GC0309_WriteReg(0xe7, 0xc0); 
            GC0309_WriteReg(0xe8, 0x05);   //exp level 3  7.50fps
            GC0309_WriteReg(0xe9, 0x00); 
            GC0309_WriteReg(0xea, 0x09);   //exp level 4  4.00fps
            GC0309_WriteReg(0xeb, 0x60); 

            break;

        default:
            break;
    }

    return 0;
}

/******************************************************************************/
// Description: set video mode
// Global resource dependence: 
// Author:
// Note:
//		 
/******************************************************************************/
//__align(4) const SENSOR_REG_T GC0309_video_mode_nor_tab[][15]=
SENSOR_REG_T GC0309_video_mode_nor_tab[][15]=
{
    // normal mode      14.3 fps
    {
	{0x01,0x6a},{0x02,0x70},{0x0f,0x00},{0xe2,0x00},{0xe3,0x96},{0xe4,0x02},{0xe5,0x58},
	{0xe6,0x02},{0xe7,0x58},{0xe8,0x02},{0xe9,0x58},{0xea,0x09}, {0xeb,0xc4}, {0xec,0x20},{0xff,0xff} 
	},    
    //vodeo mode     
    {
	{0x01,0xfa},{0x02,0x9c},{0x0f,0x11},{0xe2,0x00},{0xe3,0x64},{0xe4,0x03},{0xe5,0x84},
	{0xe6,0x03},{0xe7,0x84},{0xe8,0x03},{0xe9,0x84},{0xea,0x09}, {0xeb,0xc4}, {0xec,0x20},{0xff,0xff} 
    }

};    

LOCAL uint32_t set_gc0309_video_mode(uint32_t mode)
{
    //uint8_t data=0x00;
    uint16_t i;
    SENSOR_REG_T* sensor_reg_ptr = PNULL;
    uint8_t tempregval = 0;

    SENSOR_ASSERT(mode <= 1);

     //SENSOR_TRACE("set_GC0309_video_mode-CHIP_DetectMemType()=%d",CHIP_DetectMemType());
   
    //if(!CHIP_DetectMemType())
        //#ifdef _WQVGA_LCD
            sensor_reg_ptr = (SENSOR_REG_T*)GC0309_video_mode_nor_tab[mode];
        //#else
            //sensor_reg_ptr = (SENSOR_REG_T*)GC0309_video_mode_nor_tab[mode];
        //#endif
    //else
        //#ifdef _WQVGA_LCD
            //SENSOR_PASSERT(0, ("Not support NOR wqvga yet"));
        //#else
            //sensor_reg_ptr = (SENSOR_REG_T*)GC0309_video_mode_nor_tab[mode];
        //#endif

    SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value); i++)
    {
    	GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
	tempregval = GC0309_ReadReg(0xd2);
    SENSOR_TRACE("SENSOR: GC0309_ReadReg(0xd2) = %x\n", tempregval);
    SENSOR_TRACE("SENSOR: set_video_mode: mode = %d\n", mode);
    return 0;
}


/******************************************************************************/
// Description: set wb mode 
// Global resource dependence: 
// Author:
// Note:
//		
/******************************************************************************/
//__align(4) const SENSOR_REG_T gc0309_awb_tab[][6]=
/*SENSOR_REG_T gc0309_awb_tab[][6]=
{

     //AUTO
    {
        {0x41, 0x3d},
        {0xff, 0xff}
    },    
    //INCANDESCENCE:
    {
        {0x41, 0x39},
        {0xca, 0x60},
        {0xcb, 0x40},
        {0xcc, 0x80},
        {0xff, 0xff}         
    },
    //U30 ?
    {
       {0x41, 0x39},
        {0xca, 0x60},
        {0xcb, 0x40},
        {0xcc, 0x50},
        {0xff, 0xff}      
    },  
    //CWF ?
    {
        {0x41, 0x39},
        {0xca, 0x60},
        {0xcb, 0x40},
        {0xcc, 0x50},
        {0xff, 0xff}            
    },    
    //FLUORESCENT:
    {
        {0x41, 0x39},
        {0xca, 0x50},
        {0xcb, 0x40},
        {0xcc, 0x70},
        {0xff, 0xff}          
    },
    //SUN:
    {
        {0x41, 0x39},
        {0xca, 0x5a},
        {0xcb, 0x40},
        {0xcc, 0x58},
        {0xff, 0xff}           
    },
    //CLOUD:
    {
        {0x41, 0x39},
        {0xca, 0x68},
        {0xcb, 0x40},
        {0xcc, 0x50},
        {0xff, 0xff}            
    },
};
*/
/*
// enum: preview while balance mode
typedef enum
{
	DCAMERA_WB_MODE_AUTO = 0x00,			//自动
	DCAMERA_WB_MODE_INCANDESCENCE,		//白炽灯
	DCAMERA_WB_MODE_U30,				//商用光源
	DCAMERA_WB_MODE_CWF,				//冷荧光
	DCAMERA_WB_MODE_FLUORESCENT,		//日光灯
	DCAMERA_WB_MODE_SUN,				//晴天
	DCAMERA_WB_MODE_CLOUD,				//阴天
	DCAMERA_WB_MODE_MAX
}DCAMERA_PARAM_WB_MODE_E;
*/
//__align(4) const SENSOR_REG_T GC0309_awb_tab[][5]=
SENSOR_REG_T GC0309_awb_tab[][5]=
	{
			//AUTO
			{
				{0x5a, 0x4c}, {0x5b, 0x40}, {0x5c, 0x4a},
					  {0x22, 0x57},    // the reg value is not written here, rewrite in set_GC0309_awb();
					  {0xff, 0xff}
			},	  
			//INCANDESCENCE:
			{
				{0x22, 0x55},	 // Disable AWB 
				{0x5a, 0x48},{0x5b, 0x40},{0x5c, 0x5c},
					  {0xff, 0xff} 
			},
    //U30 ?
    {
       {0x41, 0x39},
        {0xca, 0x60},
        {0xcb, 0x40},
        {0xcc, 0x50},
        {0xff, 0xff}      
    },  
    //CWF ?
    {
        {0x41, 0x39},
        {0xca, 0x60},
        {0xcb, 0x40},
        {0xcc, 0x50},
        {0xff, 0xff}            
    },    
			//FLUORESCENT:
			{
				{0x22, 0x55},	// Disable AWB 
					  {0x5a, 0x40},{0x5b, 0x42}, {0x5c, 0x50},
				{0xff, 0xff} 
			},
			//SUN:
			{
				{0x22, 0x55},	// Disable AWB 
				//{0x5a, 0x50},{0x5b, 0x45},{0x5c, 0x40},
				//{0x5a, 0x74},{0x5b, 0x52},{0x5c, 0x40},
				{0x5a, 0x45},{0x5b, 0x3a},{0x5c, 0x40},
				{0xff, 0xff} 
			},
					//CLOUD:
				  {
					  {0x22, 0x55},   // Disable AWB 
				//{0x5a, 0x5a}, {0x5b, 0x42},{0x5c, 0x40},
				//{0x5a, 0x8c}, {0x5b, 0x50},{0x5c, 0x40},
				{0x5a, 0x4a}, {0x5b, 0x32},{0x5c, 0x40},
				{0xff, 0xff} 
			},
	};
	
	LOCAL uint32_t set_gc0309_awb(uint32_t mode)
	{
		uint8_t awb_en_value;
		uint16_t i;
		
		SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0309_awb_tab[mode];
	
		awb_en_value = GC0309_ReadReg(0x22);
	
		SENSOR_ASSERT(mode < 7);
		SENSOR_ASSERT(PNULL != sensor_reg_ptr);
		
		for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value); i++)
		{
			   if(0x22 == sensor_reg_ptr[i].reg_addr)
			{
				if(mode == 0)
					GC0309_WriteReg(0x22, awb_en_value |0x02 );
				else
							GC0309_WriteReg(0x22, awb_en_value &0xfd );
			}
				  else
			   {
					   GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
			}
		}
		msleep(100); 
		SENSOR_TRACE("SENSOR: set_awb_mode: mode = %d\n", mode);
		
		return 0;
}

//__align(4) const SENSOR_REG_T gc0309_contrast_tab[][2]=
SENSOR_REG_T gc0309_contrast_tab[][2]=
{
		{
			{0xb3,0x34},	{0xff,0xff},
		},
	
		{
			{0xb3,0x38},	{0xff,0xff}, 
		},
	
		{
			{0xb3,0x3d},	{0xff,0xff}, 
		},
	
		{
			{0xb3,0x3c},	{0xff,0xff},
		},
	
		{
			{0xb3,0x44},	{0xff,0xff}, 
		},
	
		{
			{0xb3,0x48},	{0xff,0xff}, 
		},
	
		{
			{0xb3,0x50},	{0xff,0xff},
		},					 
};


LOCAL uint32_t set_contrast(uint32_t level)
{
    uint16_t i;
    SENSOR_REG_T* sensor_reg_ptr;

    sensor_reg_ptr = (SENSOR_REG_T*)gc0309_contrast_tab[level];

    SENSOR_ASSERT(level < 7 );
    SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) && (0xFF != sensor_reg_ptr[i].reg_value); i++)
    {
        GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
    msleep(20);
    SENSOR_TRACE("set_contrast: level = %d\n", level);
    return 0;
}


LOCAL uint32_t set_sharpness(uint32_t level)
{
	
	return 0;
}


LOCAL uint32_t set_saturation(uint32_t level)
{

	
	return 0;
}

/******************************************************************************/
// Description: set brightness 
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/

LOCAL uint32_t set_preview_mode(uint32_t preview_mode)
{
	SENSOR_TRACE("set_preview_mode: preview_mode = %d\n", preview_mode);
	
	
	switch (preview_mode)
	{
		case DCAMERA_ENVIRONMENT_NORMAL: 
		{
			GC0309_set_work_mode(0);
			break;
		}
		case DCAMERA_ENVIRONMENT_NIGHT:
		{
			GC0309_set_work_mode(1);
			break;
		}
		case DCAMERA_ENVIRONMENT_SUNNY:
		{
			GC0309_set_work_mode(0);
			break;
		}
		default:
		{
			break;
		}
			
	}
	
	SENSOR_Sleep(250);
	
	return 0;
}
/*
typedef enum
{
	DCAMERA_EFFECT_NORMAL = 0x00,
	DCAMERA_EFFECT_BLACKWHITE,			// 黑白
	DCAMERA_EFFECT_RED,					// 偏红
	DCAMERA_EFFECT_GREEN,				// 偏绿
	DCAMERA_EFFECT_BLUE,				// 偏蓝
	DCAMERA_EFFECT_YELLOW,				// 偏黄
	DCAMERA_EFFECT_NEGATIVE,			// 底片
	DCAMERA_EFFECT_CANVAS,				// 帆布
	DCAMERA_EFFECT_RELIEVOS,			// 浮雕	
	DCAMERA_EFFECT_MAX
}DCAMERA_PARAM_EFFECT_E;
*/

//__align(4) const SENSOR_REG_T GC0309_image_effect_tab[][11]=	
SENSOR_REG_T GC0309_image_effect_tab[][11]=	
{
    // effect normal
    {
		{0x23,0x00}, {0x2d,0x0a}, {0x20,0x7f}, {0xd2,0x90}, {0x73,0x00}, {0x77,0x78},
		{0xb3,0x42}, {0xb4,0x80}, {0xba,0x00}, {0xbb,0x00}, {0xff,0xff}
    },
    //effect BLACKWHITE
    {
		{0x23,0x02}, {0x2d,0x0a}, {0x20,0x7f}, {0xd2,0x90}, {0x73,0x00},  
		{0xb3,0x40},	{0xb4,0x80}, {0xba,0x00}, {0xbb,0x00}, {0xff,0xff}
    },
    // effect RED pink
    {
		//TODO: later work
 		{0x23,0x02},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x77,0x88},
         	{0xb3,0x40},{0xb4,0x80},{0xba,0x10},{0xbb,0x50},{0xff, 0xff}
    },
    // effect GREEN
    {
 		{0x23,0x02},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x77,0x88},
         	{0xb3,0x40},{0xb4,0x80},{0xba,0xc0},{0xbb,0xc0},{0xff, 0xff}
    },
    // effect  BLUE
    {
   		{0x23,0x02},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x73,0x00},
   		{0xb3,0x40},{0xb4,0x80},{0xba,0x50},{0xbb,0xe0},{0xff, 0xff}
    },
    // effect  YELLOW
    {
		//TODO:later work
 		{0x23,0x02},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x77,0x88},
         	{0xb3,0x40},{0xb4,0x80},{0xba,0x80},{0xbb,0x20},{0xff, 0xff}

    },  
    // effect NEGATIVE
    {	     
   		{0x23,0x01},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x73,0x00},
   		{0xb3,0x40},{0xb4,0x80},{0xba,0x00},{0xbb,0x00},{0xff, 0xff}
    },    
    //effect ANTIQUE
    {
   		{0x23,0x02},{0x2d,0x0a},{0x20,0x7f},{0xd2,0x90},{0x73,0x00},
   		{0xb3,0x40},{0xb4,0x80},{0xba,0xd0},{0xbb,0x28},{0xff, 0xff}
    },
};
LOCAL uint32_t set_image_effect(uint32_t effect_type)
{
    uint16_t i;
    
    SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0309_image_effect_tab[effect_type];

    SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value) ; i++)
    {
        Sensor_WriteReg_8bits(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
    SENSOR_TRACE("-----------set_image_effect: effect_type = %d------------\n", effect_type);
    
    return 0;
}

/*
LOCAL uint32_t GC0309_After_Snapshot(uint32_t param)
{

	Sensor_SetMCLK(24);
	
	GC0309_WriteReg(0x41,GC0309_ReadReg(0x41) | 0xf7);
	SENSOR_Sleep(200);
	return 0;
    
}

LOCAL uint32_t GC0309_BeforeSnapshot(uint32_t param)
{

    uint16_t shutter = 0x00;
    uint16_t temp_reg = 0x00;
    uint16_t temp_r =0x00;
    uint16_t temp_g =0x00;
    uint16_t temp_b =0x00;    
    BOOLEAN b_AEC_on;
    

    SENSOR_TRACE("GC0309_BeforeSnapshot ");   
    	if(GC0309_ReadReg(0X41)  & 0x08 == 0x08)  //AEC on
    		b_AEC_on = SENSOR_TRUE;
    	else
    		b_AEC_on = SENSOR_FALSE;

	temp_reg = GC0309_ReadReg(0xdb);
	temp_r = GC0309_ReadReg(0xcd);
	temp_g = GC0309_ReadReg(0xce);
	temp_b = GC0309_ReadReg(0xcf);

	shutter = (GC0309_ReadReg(0x03)<<8)  | (GC0309_ReadReg(0x04)&0x00ff) ;
	shutter = shutter /2;

	if(b_AEC_on)
		GC0309_WriteReg(0x41,GC0309_ReadReg(0x41) & 0xc5); //0x01);
	SENSOR_Sleep(300); 

///12m
	Sensor_SetMCLK(12);
	
	GC0309_WriteReg(0x03,shutter/256);
	GC0309_WriteReg(0x04,shutter & 0x00ff);	
   	//SENSOR_TRACE("GC0309_BeforeSnapshot, temp_r=%x,temp_reg=%x, final = %x ",temp_r,temp_reg, temp_r*temp_reg/ 0x80);    

	temp_r = (temp_r*temp_reg) / 0x80;
	temp_g = (temp_g*temp_reg) / 0x80;
	temp_b = (temp_b*temp_reg) / 0x80;
	if(b_AEC_on)
	{
		GC0309_WriteReg(0xcd, temp_r);
		GC0309_WriteReg(0xce, temp_g);
		GC0309_WriteReg(0xcf , temp_b);
	}
   	//SENSOR_TRACE("GC0309_BeforeSnapshot, temp_r=%x,temp_g=%x, temp_b = %x ",temp_r,temp_g,temp_b);    

	SENSOR_Sleep(300); 
    	return 0;
    
}
*/
LOCAL uint32_t read_ev_value(uint32_t value)
{
	return 0;
}

LOCAL uint32_t write_ev_value(uint32_t exposure_value)
{
	
	return 0;	
}

LOCAL uint32_t read_gain_value(uint32_t value)
{

	
	return 0;
}

LOCAL uint32_t write_gain_value(uint32_t gain_value)
{

	
	return 0;
}

LOCAL uint32_t read_gain_scale(uint32_t value)
{
	return SENSOR_GAIN_SCALE;
	
}


LOCAL uint32_t set_frame_rate(uint32_t param)
    
{
	//GC0309_WriteReg( 0xd8, uint8_t data );
	return 0;
}

/******************************************************************************/
// Description:
// Global resource dependence: 
// Author:
// Note:
//		mode 0:normal;	 1:night 
/******************************************************************************/
//__align(4) const SENSOR_REG_T gc0309_mode_tab[][8]=
SENSOR_REG_T gc0309_mode_tab[][8]=
{
	//LCD的GAMMA值需要细调，不然会有一圈圈的光晕
	//Fps 12.5 YUV open auto frame function, 展讯的jpeg编码不行太大和太快，因此将帧率限制在12.5fps
	{/*{0xa0, 0x50},*/{0xec, 0x20},{0xFF, 0xFF},},
	//Fps 12.5->3.125 YUV open auto frame function
	{/*{0xa0, 0x40},*/{0xd8, 0x30},{0xFF, 0xFF},},
};

LOCAL uint32_t GC0309_set_work_mode(uint32_t mode)
{
	uint16_t i;
	SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)gc0309_mode_tab[mode];

	SENSOR_ASSERT(mode <= 1);
	SENSOR_ASSERT(PNULL != sensor_reg_ptr);
	
	for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value); i++)
	{
		GC0309_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	}

	SENSOR_TRACE("set_work_mode: mode = %d\n", mode);
	return 0;
}
