/******************************************************************************
 ** File Name:      sensor_drv.h                                                  *
 ** Author:         Liangwen.Zhen                                             *
 ** DATE:           04/20/2006                                                *
 ** Copyright:      2006 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    This file defines the basic operation interfaces of sensor*
 **                				                                              *
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 04/20/2006     Liangwen.Zhen     Create.                                  *
 ******************************************************************************/

#ifndef _SENSOR_DRV_H_
#define _SENSOR_DRV_H_

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/

#ifdef __cplusplus
    extern   "C"
    {
#endif

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/mfp.h>
#include <mach/hardware.h>
#include <asm/io.h>

#include <mach/jpeg_exif_header_k.h>

//for sensor
#define SENSOR_SUCCESS 0
#define SENSOR_FAIL 1
#define SENSOR_FALSE 0 
#define SENSOR_TRUE 1
#define SENSOR_ASSERT(a) do{}while(!(a));
#define SENSOR_Sleep(m) msleep(m)
#define SENSOR_MEMSET memset
#define SENSOR_MALLOC kmalloc
typedef void *SENSOR_MUTEX_PTR;
#define SENSOR_PASSERT(m, n) if(!m) printk n 

#define BOOLEAN char
#define PNULL  ((void *)0)
#define PUBLIC 
#define LOCAL static
#define SENSOR_NULL 0

/**---------------------------------------------------------------------------*
 **                         Debugging Flag                                    *
 **---------------------------------------------------------------------------*/
#define DEBUG_SENSOR_DRV 1
#ifdef DEBUG_SENSOR_DRV
#define SENSOR_PRINT   printk
#else
#define SENSOR_PRINT(...)  
#endif

#define SENSOR_PRINT_ERR   printk
#define SENSOR_PRINT_HIGH printk

/**---------------------------------------------------------------------------*
**                                                   Global Register Micro Define                                                                     **
**----------------------------------------------------------------------------*/
#define GLOBAL_BASE SPRD_GREG_BASE //0xE0002E00UL <--> 0x8b000000
#define ARM_GLOBAL_REG_GEN0 GLOBAL_BASE + 0x008UL
#define ARM_GLOBAL_REG_GEN3 GLOBAL_BASE + 0x01CUL
#define ARM_GLOBAL_PLL_SCR GLOBAL_BASE + 0x070UL
#define GR_CLK_GEN5 GLOBAL_BASE + 0x07CUL

#define AHB_BASE SPRD_AHB_BASE //0xE000A000 <--> 0x20900000UL
#define AHB_GLOBAL_REG_CTL0 AHB_BASE + 0x200UL
#define AHB_GLOBAL_REG_SOFTRST AHB_BASE + 0x210UL

#define PIN_CTL_BASE SPRD_CPC_BASE//0xE002F000<-->0x8C000000UL
#define PIN_CTL_CCIRPD1 PIN_CTL_BASE + 0x344UL
#define PIN_CTL_CCIRPD0 PIN_CTL_BASE + 0x348UL

#define MISC_BASE SPRD_MISC_BASE //0xE0033000<-->0x82000000
#ifdef CONFIG_ARCH_SC8810
#define ANA_REG_BASE MISC_BASE + 0x600
#define ANA_LDO_PD_CTL ANA_REG_BASE + 0x10
#define ANA_LDO_VCTL2 ANA_REG_BASE + 0x20
#else
#define ANA_REG_BASE MISC_BASE + 0x480
#define ANA_LDO_PD_CTL ANA_REG_BASE + 0x10
#define ANA_LDO_VCTL2 ANA_REG_BASE + 0x1C  
#endif

/**---------------------------------------------------------------------------*
 **                         MACRO Definations                                 *
 **---------------------------------------------------------------------------*/
#define NUMBER_OF_ARRAY(a)    				(sizeof(a)/sizeof(a[0]))
#define ADDR_AND_LEN_OF_ARRAY(a) 			(SENSOR_REG_T*)a, NUMBER_OF_ARRAY(a)


#define SENSOR_DISABLE_MCLK					0		// MHZ
#define SENSOR_DEFALUT_MCLK					24		// MHZ
#define SENSOR_MAX_MCLK						96		// MHZ

#define SENSOR_LOW_PULSE_RESET				0x00
#define SENSOR_HIGH_PULSE_RESET				0x01

#define SENSOR_RESET_PULSE_WIDTH_DEFAULT	20  		// ms
#define SENSOR_RESET_PULSE_WIDTH_MAX		200  	// ms

#define SENSOR_LOW_LEVEL_PWDN				0x00
#define SENSOR_HIGH_LEVEL_PWDN				0x01

#define SENSOR_IDENTIFY_CODE_COUNT			0x02

// Image effect
#define SENSOR_IMAGE_EFFECT_NORMAL			(0x01 << 0)
#define SENSOR_IMAGE_EFFECT_BLACKWHITE		(0x01 << 1)		
#define SENSOR_IMAGE_EFFECT_RED				(0x01 << 2)		
#define SENSOR_IMAGE_EFFECT_GREEN			(0x01 << 3)		
#define SENSOR_IMAGE_EFFECT_BLUE				(0x01 << 4)		
#define SENSOR_IMAGE_EFFECT_YELLOW			(0x01 << 5)	
#define SENSOR_IMAGE_EFFECT_NEGATIVE			(0x01 << 6)		
#define SENSOR_IMAGE_EFFECT_CANVAS			(0x01 << 7)	
#define SENSOR_IMAGE_EFFECT_RELIEVOS			(0x01 << 8)		

// While balance mode
#define SENSOR_WB_MODE_AUTO 				(0x01 << 0)	
#define SENSOR_WB_MODE_INCANDESCENCE		(0x01 << 1)	
#define SENSOR_WB_MODE_U30					(0x01 << 2)	
#define SENSOR_WB_MODE_CWF					(0x01 << 3)	
#define SENSOR_WB_MODE_FLUORESCENT		(0x01 << 4)
#define SENSOR_WB_MODE_SUN					(0x01 << 5)
#define SENSOR_WB_MODE_CLOUD				(0x01 << 6)

// Preview mode
#define SENSOR_ENVIROMENT_NORMAL			(0x01 << 0)		
#define SENSOR_ENVIROMENT_NIGHT			(0x01 << 1)	
#define SENSOR_ENVIROMENT_SUNNY			(0x01 << 2)	
#define SENSOR_ENVIROMENT_SPORTS			(0x01 << 3)		
#define SENSOR_ENVIROMENT_LANDSCAPE		(0x01 << 4)	
#define SENSOR_ENVIROMENT_PORTRAIT			(0x01 << 5)		
#define SENSOR_ENVIROMENT_PORTRAIT_NIGHT	(0x01 << 6)		
#define SENSOR_ENVIROMENT_BACKLIGHT		(0x01 << 7)	
#define SENSOR_ENVIROMENT_MARCO			(0x01 << 8)	

#define SENSOR_ENVIROMENT_MANUAL			(0x01 << 30)	
#define SENSOR_ENVIROMENT_AUTO				(0x01 << 31)	

// YUV PATTERN
#define SENSOR_IMAGE_PATTERN_YUV422_YUYV	0x00
#define SENSOR_IMAGE_PATTERN_YUV422_YVYU	0x01
#define SENSOR_IMAGE_PATTERN_YUV422_UYVY	0x02
#define SENSOR_IMAGE_PATTERN_YUV422_VYUY	0x03
// RAW RGB BAYER
#define SENSOR_IMAGE_PATTERN_RAWRGB_GR	0x00
#define SENSOR_IMAGE_PATTERN_RAWRGB_R		0x01
#define SENSOR_IMAGE_PATTERN_RAWRGB_B		0x02
#define SENSOR_IMAGE_PATTERN_RAWRGB_GB	0x03

// I2C REG/VAL BIT count
#define SENSOR_I2C_VAL_8BIT			0x00
#define SENSOR_I2C_VAL_16BIT			0x01
#define SENSOR_I2C_REG_8BIT			(0x00 << 1)
#define SENSOR_I2C_REG_16BIT			(0x01 << 1)
#define SENSOR_I2C_CUSTOM 			(0x01 << 2)

//I2C ACK/STOP BIT count
#define SNESOR_I2C_ACK_BIT (0x00 << 3)
#define SNESOR_I2C_NOACK_BIT (0x00 << 3)
#define SNESOR_I2C_STOP_BIT (0x00 << 3)
#define SNESOR_I2C_NOSTOP_BIT (0x00 << 3)

//I2C FEEQ BIT count
#define SENSOR_I2C_FREQ_20 (0x01 << 6)
#define SENSOR_I2C_FREQ_50 (0x02 << 6)
#define SENSOR_I2C_FREQ_100 (0x00 << 6)
#define SENSOR_I2C_FREQ_200 (0x03 << 6)
#define SENSOR_I2C_FREQ_400      (0x04 << 5)

// Hardward signal polarity
#define SENSOR_HW_SIGNAL_PCLK_N				0x00
#define SENSOR_HW_SIGNAL_PCLK_P				0x01
#define SENSOR_HW_SIGNAL_VSYNC_N			(0x00 << 2)
#define SENSOR_HW_SIGNAL_VSYNC_P			(0x01 << 2)
#define SENSOR_HW_SIGNAL_HSYNC_N			(0x00 << 4)
#define SENSOR_HW_SIGNAL_HSYNC_P			(0x01 << 4)

#define SENSOR_WRITE_DELAY					0xffff

#define SENSOR_IOCTL_FUNC_NOT_REGISTER		0xffffffff

// sensor focus mode
#define SENSOR_FOCUS_TRIG 0x01
#define SENSOR_FOCUS_ZONE (0x01<<1)

// sensor exposure mode
#define SENSOR_EXPOSURE_AUTO 0x01
#define SENSOR_EXPOSURE_ZONE (0x01<<1)

#define FOCUS_ZONE_CNT_MAX   6
/**---------------------------------------------------------------------------*
 **                         Data Structures                                   *
 **---------------------------------------------------------------------------*/
 // enum: return value about sensor operation
 typedef enum
{
	SENSOR_OP_SUCCESS = SENSOR_SUCCESS,
	SENSOR_OP_PARAM_ERR,
	SENSOR_OP_STATUS_ERR,
	SENSOR_OP_ERR,  
	SENSOR_OP_MAX = 0xFFFF
}ERR_SENSOR_E;

// enum: AVDD value about BB LDO output
typedef enum
{
	SENSOR_MAIN = 0,
	SENSOR_SUB,
	SENSOR_ATV=5,//bonnie
	SENSOR_ID_MAX
}SENSOR_ID_E;

//enum: sensor type img sensor or analog tv
typedef enum
{
	SENSOR_TYPE_NONE = 0x00,
	SENSOR_TYPE_IMG_SENSOR,
	SENSOR_TYPE_ATV,
	SENSOR_TYPE_MAX	
}SENSOR_TYPE_E;

// enum: AVDD value about BB LDO output
typedef enum
{
	SENSOR_AVDD_3800MV = 0,
	SENSOR_AVDD_3000MV,
	SENSOR_AVDD_2800MV,
	SENSOR_AVDD_2500MV,
	SENSOR_AVDD_1800MV,
	SENSOR_AVDD_1500MV,
	SENSOR_AVDD_1300MV,	
	SENSOR_AVDD_1200MV,	
	SENSOR_AVDD_CLOSED,
	SENSOR_AVDD_UNUSED
}SENSOR_AVDD_VAL_E;

// enum: AVDD value about BB LDO output
typedef enum
{
	SENSOR_MCLK_12M=12,
	SENSOR_MCLK_13M=13,
	SENSOR_MCLK_24M=24,
	SENSOR_MCLK_26M=26,
	SENSOR_MCLK_MAX
}SENSOR_M_CLK_E;

typedef enum
{
	SENSOR_INTERFACE_CCIR601_8BITS = 0,
	SENSOR_INTERFACE_CCIR601_4BITS,
	SENSOR_INTERFACE_CCIR601_2BITS,
	SENSOR_INTERFACE_CCIR601_1BITS,
	SENSOR_INTERFACE_CCIR656_8BITS,
	SENSOR_INTERFACE_CCIR656_4BITS,
	SENSOR_INTERFACE_CCIR656_2BITS,
	SENSOR_INTERFACE_CCIR656_1BITS,
	SENSOR_INTERFACE_SPI_8BITS,
	SENSOR_INTERFACE_SPI_4BITS,
	SENSOR_INTERFACE_SPI_4BITS_BE,
	SENSOR_INTERFACE_SPI_2BITS,
	SENSOR_INTERFACE_SPI_2BITS_BE,
	SENSOR_INTERFACE_SPI_1BITS,
	SENSOR_INTERFACE_SPI_1BITS_BE,
	SENSOR_INTERFACE_MAX
}SENSOR_INTERFACE_E;


// preview one match some range resolution snapshot
// preview two match other range resolution snapshot
// so the smallest resolution of snapshot one overlap with the biggest resolution of two,
// or the biggest resolution of snapshot one overlap with the smallest resolution of two.
typedef enum
{		
	// COMMON INIT
	SENSOR_MODE_COMMON_INIT = 0,	

	// PREVIEW ONE
	SENSOR_MODE_PREVIEW_ONE,		
	SENSOR_MODE_SNAPSHOT_ONE_FIRST,
	SENSOR_MODE_SNAPSHOT_ONE_SECOND,
	SENSOR_MODE_SNAPSHOT_ONE_THIRD,

	// PREVIEW TWO	
	SENSOR_MODE_PREVIEW_TWO,
	SENSOR_MODE_SNAPSHOT_TWO_FIRST,
	SENSOR_MODE_SNAPSHOT_TWO_SECOND,
	SENSOR_MODE_SNAPSHOT_TWO_THIRD,


	SENSOR_MODE_MAX
} SENSOR_MODE_E;

// enum: image format about sensor output
typedef enum
{
	SENSOR_IMAGE_FORMAT_YUV422 = 0,
	SENSOR_IMAGE_FORMAT_YUV420,
	SENSOR_IMAGE_FORMAT_RAW,
	SENSOR_IMAGE_FORMAT_RGB565,
	SENSOR_IMAGE_FORMAT_RGB666,
	SENSOR_IMAGE_FORMAT_RGB888,
	SENSOR_IMAGE_FORMAT_CCIR656,
	SENSOR_IMAGE_FORMAT_JPEG,
	
	SENSOR_IMAGE_FORMAT_MAX
}SENSOR_IMAGE_FORMAT;

// enum: Sensor IOCTL command
typedef enum
{
    // Internal Command (count = 8)
    SENSOR_IOCTL_RESET = 0,			// use to reset sensor
    SENSOR_IOCTL_POWER,			// Power on/off sensor selected by input parameter(0:off,1:on)
    SENSOR_IOCTL_ENTER_SLEEP,		// enter sleep
    SENSOR_IOCTL_IDENTIFY,			// identify
    SENSOR_IOCTL_WRITE_REG,		// write register value
    SENSOR_IOCTL_READ_REG,			// read register value
    SENSOR_IOCTL_CUS_FUNC_1,		// set function 1 for custumer to configure
    SENSOR_IOCTL_CUS_FUNC_2,		// set function 2 for custumer to configure  	 

    // External Command (count = 18)
    SENSOR_IOCTL_AE_ENABLE,		// enable auto exposure
    SENSOR_IOCTL_HMIRROR_ENABLE,	// enable horizontal mirror
    SENSOR_IOCTL_VMIRROR_ENABLE,	// enable vertical mirror

    SENSOR_IOCTL_BRIGHTNESS,		// set brightness
    SENSOR_IOCTL_CONTRAST,			// set contrast
    SENSOR_IOCTL_SHARPNESS,		// set sharpness
    SENSOR_IOCTL_SATURATION,		// set saturation
    SENSOR_IOCTL_PREVIEWMODE,		// set preview mode

    SENSOR_IOCTL_IMAGE_EFFECT,		// set image effect
    SENSOR_IOCTL_BEFORE_SNAPSHOT,	// do something before do snapshort
    SENSOR_IOCTL_AFTER_SNAPSHOT,	// do something after do snapshort	 

    SENSOR_IOCTL_FLASH,			// control to open / close flash

    SENSOR_IOCTL_READ_EV,			// read AE value from sensor register
    SENSOR_IOCTL_WRITE_EV,			// write AE value to sensor register
    SENSOR_IOCTL_READ_GAIN,		// read GAIN value from sensor register
    SENSOR_IOCTL_WRITE_GAIN,		// write GAIN value to sensor register
    SENSOR_IOCTL_READ_GAIN_SCALE,  // read GAIN scale (sensor dependable, refer to sensor spec)
    SENSOR_IOCTL_SET_FRAME_RATE,	// set sensor frame rate based on current clock

    SENSOR_IOCTL_AF_ENABLE,		// enable auto focus function
    SENSOR_IOCTL_AF_GET_STATUS,	// get auto focus status

    SENSOR_IOCTL_SET_WB_MODE,		// set while balance mode

    SENSOR_IOCTL_GET_SKIP_FRAME,	// get snapshot skip frame num from customer

    SENSOR_IOCTL_ISO,				// set ISO mode
    SENSOR_IOCTL_EXPOSURE_COMPENSATION, // Set exposure compensation

    SENSOR_IOCTL_CHECK_IMAGE_FORMAT_SUPPORT, // check whether image format is support
    SENSOR_IOCTL_CHANGE_IMAGE_FORMAT, //change sensor image format
    SENSOR_IOCTL_ZOOM,					//change sensor output window and size

    SENSOR_IOCTL_CUS_FUNC_3,		// set function 3 for custumer to configure
    SENSOR_IOCTL_FOCUS,//SENSOR_IOCTL_CUS_FUNC_4,		// set function 4 for custumer to configure

    SENSOR_IOCTL_ANTI_BANDING_FLICKER, // Set anti banding flicker mode
    SENSOR_IOCTL_VIDEO_MODE, // Set video mode

    SENSOR_IOCTL_PICK_JPEG_STREAM,
	 
    SENSOR_IOCTL_MAX
}SENSOR_IOCTL_CMD_E;

// enum: Sensor EXTEND focus command
typedef enum
{
	SENSOR_EXT_FOCUS_NONE=0x00,
	SENSOR_EXT_FOCUS_TRIG,
	SENSOR_EXT_FOCUS_ZONE,
	SENSOR_EXT_FOCUS_MULTI_ZONE,
	SENSOR_EXT_FOCUS_MACRO,
	SENSOR_EXT_FOCUS_MAX
}SENSOR_EXT_FOCUS_CMD_E;

// enum: Sensor EXTEND exposure command
typedef enum
{
	SENSOR_EXT_EXPOSURE_NONE=0x00,
	SENSOR_EXT_EXPOSURE_AUTO,    
	SENSOR_EXT_EXPOSURE_ZONE,
	SENSOR_EXT_EXPOSURE_MAX
}SENSOR_EXT_EXPOSURE_CMD_E;

// enum: Sensor EXTEND function command
typedef enum
{
	SENSOR_EXT_FUNC_NONE=0x00,
	SENSOR_EXT_FUNC_INIT,
	SENSOR_EXT_FOCUS_START,
	SENSOR_EXT_EXPOSURE_START,
	SENSOR_EXT_FUNC_MAX
}SENSOR_EXT_FUNC_CMD_E;

// enum: Sensor exif command
typedef enum
{
    SENSOR_EXIF_CTRL_EXPOSURETIME=0x00,
    SENSOR_EXIF_CTRL_FNUMBER,
    SENSOR_EXIF_CTRL_EXPOSUREPROGRAM,	
    SENSOR_EXIF_CTRL_SPECTRALSENSITIVITY,	
    SENSOR_EXIF_CTRL_ISOSPEEDRATINGS,
    SENSOR_EXIF_CTRL_OECF,	
    SENSOR_EXIF_CTRL_SHUTTERSPEEDVALUE,
    SENSOR_EXIF_CTRL_APERTUREVALUE,
    SENSOR_EXIF_CTRL_BRIGHTNESSVALUE,
    SENSOR_EXIF_CTRL_EXPOSUREBIASVALUE,
    SENSOR_EXIF_CTRL_MAXAPERTUREVALUE,
    SENSOR_EXIF_CTRL_SUBJECTDISTANCE,
    SENSOR_EXIF_CTRL_METERINGMODE,
    SENSOR_EXIF_CTRL_LIGHTSOURCE,	
    SENSOR_EXIF_CTRL_FLASH,
    SENSOR_EXIF_CTRL_FOCALLENGTH,
    SENSOR_EXIF_CTRL_SUBJECTAREA,
    SENSOR_EXIF_CTRL_FLASHENERGY,
    SENSOR_EXIF_CTRL_SPATIALFREQUENCYRESPONSE,
    SENSOR_EXIF_CTRL_FOCALPLANEXRESOLUTION,
    SENSOR_EXIF_CTRL_FOCALPLANEYRESOLUTION,
    SENSOR_EXIF_CTRL_FOCALPLANERESOLUTIONUNIT,
    SENSOR_EXIF_CTRL_SUBJECTLOCATION,
    SENSOR_EXIF_CTRL_EXPOSUREINDEX,
    SENSOR_EXIF_CTRL_SENSINGMETHOD,
    SENSOR_EXIF_CTRL_FILESOURCE,
    SENSOR_EXIF_CTRL_SCENETYPE,
    SENSOR_EXIF_CTRL_CFAPATTERN,
    SENSOR_EXIF_CTRL_CUSTOMRENDERED,
    SENSOR_EXIF_CTRL_EXPOSUREMODE,
    SENSOR_EXIF_CTRL_WHITEBALANCE,
    SENSOR_EXIF_CTRL_DIGITALZOOMRATIO,
    SENSOR_EXIF_CTRL_FOCALLENGTHIN35MMFILM,
    SENSOR_EXIF_CTRL_SCENECAPTURETYPE,	
    SENSOR_EXIF_CTRL_GAINCONTROL,
    SENSOR_EXIF_CTRL_CONTRAST,
    SENSOR_EXIF_CTRL_SATURATION,
    SENSOR_EXIF_CTRL_SHARPNESS,
    SENSOR_EXIF_CTRL_DEVICESETTINGDESCRIPTION,
    SENSOR_EXIF_CTRL_SUBJECTDISTANCERANGE,
    SENSOR_EXIF_CTRL_MAX,
}SENSOR_EXIF_CTRL_E;

//enum: preview enviroment type
typedef enum
{
	DCAMERA_ENVIRONMENT_NORMAL = 0x00,
	DCAMERA_ENVIRONMENT_NIGHT,
	DCAMERA_ENVIRONMENT_SUNNY,
	DCAMERA_ENVIRONMENT_SPORTS,
	DCAMERA_ENVIRONMENT_LANDSCAPE,
	DCAMERA_ENVIRONMENT_PORTRAIT,
	DCAMERA_ENVIRONMENT_PORTRAIT_NIGHT,
	DCAMERA_ENVIRONMENT_BACKLIGHT,
	DCAMERA_ENVIRONMENT_MACRO,

	DCAMERA_ENVIRONMENT_MANUAL = 30,
	DCAMERA_ENVIRONMENT_AUTO = 31,

	DCAMERA_ENVIRONMENT_MAX	
}DCAMERA_PARAM_ENVIRONMENT_E;

typedef enum
{
	DCAMERA_WB_MODE_AUTO = 0x00, 
	DCAMERA_WB_MODE_INCANDESCENCE,
	DCAMERA_WB_MODE_U30,
	DCAMERA_WB_MODE_CWF,
	DCAMERA_WB_MODE_FLUORESCENT,
	DCAMERA_WB_MODE_SUN,
	DCAMERA_WB_MODE_CLOUD,
	DCAMERA_WB_MODE_MAX	
}DCAMERA_PARAM_WB_MODE_E;

//enum: preview effect type
typedef enum
{
	DCAMERA_EFFECT_NORMAL = 0x00,
	DCAMERA_EFFECT_BLACKWHITE,
	DCAMERA_EFFECT_RED,
	DCAMERA_EFFECT_GREEN,
	DCAMERA_EFFECT_BLUE,
	DCAMERA_EFFECT_YELLOW,
	DCAMERA_EFFECT_NEGATIVE,
	DCAMERA_EFFECT_CANVAS,
	DCAMERA_EFFECT_RELIEVOS,
	DCAMERA_EFFECT_MAX	
}DCAMERA_PARAM_EFFECT_E;

typedef uint32_t (*SENSOR_IOCTL_FUNC_PTR)(uint32_t param);
// struct: Sensor IOCTL function
typedef struct sensor_ioctl_func_tab_tag
{
    // 1: Internal IOCTL function
    uint32_t (*reset)			  (uint32_t param);	// use to reset sensor                                      
    uint32_t (*power)		  	  (uint32_t param);	// Power on/off sensor selected by input parameter(0:off,1:on)  
    uint32_t (*enter_sleep)  	  (uint32_t param);   // enter sleep  
    uint32_t (*identify)		  (uint32_t param); 	// identify sensor: 0 -> successful ; others -> fail
    uint32_t (*write_reg)		  (uint32_t param);	// [31:16] register address; [15:0] register value
    uint32_t (*read_reg)		  (uint32_t param);	// input value[15:0]: register address; return value[15:0]: register value
    // Custom function
    uint32_t (*cus_func_1)	  (uint32_t param);  	// function 1 for custumer to configure                      
//    uint32_t (*cus_func_2)	  (uint32_t param);  	// function 2 for custumer to configure
    uint32_t (*get_trim)	  (uint32_t param);  	// function 2 for custumer to configure

    // 2: External IOCTL function	
    uint32_t (*ae_enable)		  (uint32_t param);   // enable auto exposure                                         
    uint32_t (*hmirror_enable)  (uint32_t param);   // enable horizontal mirror                                     
    uint32_t (*vmirror_enable)  (uint32_t param);   // enable vertical mirror                                       
                                                                                            
    uint32_t (*set_brightness)  (uint32_t param);	// set brightness 	0: auto; other: the appointed level                                                
    uint32_t (*set_contrast)    (uint32_t param);   // set contrast     0: auto; other: the appointed level                                             
    uint32_t (*set_sharpness)   (uint32_t param);   // set sharpness    0: auto; other: the appointed level                                             
    uint32_t (*set_saturation)  (uint32_t param);   // set saturation   0: auto; other: the appointed level                                             
    uint32_t (*set_preview_mode)(uint32_t param);   // set preview mode : 0: normal mode; 1: night mode; 2: sunny mode                                               
                                                                                            
    uint32_t (*set_image_effect)(uint32_t param);   // set image effect                                             
    uint32_t (*before_snapshort)(uint32_t param);   // do something before do snapshort                             
    uint32_t (*after_snapshort) (uint32_t param);   // do something after do snapshort                              	

    uint32_t (*flash)     	  (uint32_t param);	// 1: open flash; 0: close falsh
                                                                                                
    uint32_t (*read_ae_value)	  (uint32_t param);	// return AE value
    uint32_t (*write_ae_value)  (uint32_t param);	// input AE value
    uint32_t (*read_gain_value) (uint32_t param);	// return GAIN value
    uint32_t (*write_gain_value)(uint32_t param);   // input GAIN value
    uint32_t (*read_gain_scale) (uint32_t param);   // return GAIN scale (for ov9650, 16)
    uint32_t (*set_frame_rate)  (uint32_t param);   // set sensor frame rate based on current clock

    uint32_t (*af_enable)		  (uint32_t param);	// input 1: enable; input 0: disable
    uint32_t (*af_get_status)	  (uint32_t param);	// return value: return 0 -> focus ok, other value -> lose focus

    uint32_t (*set_wb_mode)	  (uint32_t param);	// set while balance mode

    uint32_t (*get_skip_frame)  (uint32_t param);	// get snapshot skip frame num from customer, input SENSOR_MODE_E paramter

    uint32_t (* set_iso)		  (uint32_t param);	// set ISO level					 0: auto; other: the appointed level
    uint32_t (*set_exposure_compensation)(uint32_t param); // Set exposure compensation	 0: auto; other: the appointed level

    uint32_t (*check_image_format_support)(uint32_t param); // check whether image format is support
    uint32_t (*change_image_format)(uint32_t param); //change sensor image format according to param
    uint32_t (*set_zoom)(uint32_t param); //change sensor image format according to param

    // CUSTOMER FUNCTION	                      
//    uint32_t (*cus_func_3)	  (uint32_t param);  	// function 3 for custumer to configure                      
    uint32_t (*get_exif)	  (uint32_t param);  	// function 3 for custumer to configure                      
    uint32_t (*set_focus)	  (uint32_t param);	// function 4 for custumer to configure 	
    uint32_t (*set_anti_banding_flicker)(uint32_t param); // Set anti banding flicker	 0: 50hz;1: 60	
    uint32_t (*set_video_mode)(uint32_t param); // set video mode

    uint32_t (*pick_jpeg_stream)(uint32_t param);   // pick out the jpeg stream from given buffer
    uint32_t (*set_meter_mode)(uint32_t param);   // set meter mode
}SENSOR_IOCTL_FUNC_TAB_T, *SENSOR_IOCTL_FUNC_TAB_T_PTR;

// struct: Information about sensor register
typedef struct sensor_reg_tag
{
	uint16_t reg_addr;						// address of sensor register
	uint16_t reg_value;						// value of sensor register
	
}SENSOR_REG_T, *SENSOR_REG_T_PTR;

typedef struct sensor_reg_bits_tag
{
	uint16_t reg_addr;						// address of sensor register
	uint16_t reg_value;						// value of sensor register
	uint32_t reg_bits;                                                   // indicate invalid bits of register	
}SENSOR_REG_BITS_T, *SENSOR_REG_BITS_T_PTR;
typedef struct sensor_trim_tag
{
	uint16_t trim_start_x;
	uint16_t trim_start_y;	
	uint16_t trim_width;
	uint16_t trim_height;
	uint32_t line_time;	
	uint32_t pclk;
}SENSOR_TRIM_T, *SENSOR_TRIM_T_PTR;

typedef struct _sensor_rect_tag 
{
	uint16_t  x;
	uint16_t  y;
	uint16_t  w;
	uint16_t  h;
}SENSOR_RECT_T, *SENSOR_RECT_T_PTR;

typedef struct _sensor_ext_fun_tag 
{
	uint32_t cmd;
	uint32_t param;
	SENSOR_RECT_T zone;
}SENSOR_EXT_FUN_T, *SENSOR_EXT_FUN_T_PTR;
typedef struct _sensor_ext_fun_param_tag 
{
	uint8_t cmd;
	uint8_t param;
	uint16_t zone_cnt;
	SENSOR_RECT_T zone[FOCUS_ZONE_CNT_MAX];
}SENSOR_EXT_FUN_PARAM_T, *SENSOR_EXT_FUN_PARAM_T_PTR;

// struct: Information about sensor register table 
typedef struct sensor_reg_tab_info_tag
{
	SENSOR_REG_T_PTR	sensor_reg_tab_ptr;	// poiter of sensor register table
	uint32_t         		reg_count;			// count of registers in table
	uint16_t		   		width;				// width of resolution
	uint16_t        		height;				// height of resolution
	uint32_t		   		xclk_to_sensor;  	// unit : MHz ->0: default value
	SENSOR_IMAGE_FORMAT	image_format;		// image format is valid when set to SENSOR_IMAGE_FORMAT_MAX in SENSOR_INFO_T
											// or else image format is invalid
	
}SENSOR_REG_TAB_INFO_T, *SENSOR_REG_TAB_INFO_T_PTR;

// struct: Information about Sensor operation mode 
typedef struct sensor_mode_info_tag
{
	SENSOR_MODE_E 		mode;
	uint16_t				width;
	uint16_t				height;
	uint16_t trim_start_x;
	uint16_t trim_start_y;	
	uint16_t trim_width;	
	uint16_t trim_height;	
	uint32_t line_time;	
	SENSOR_IMAGE_FORMAT	image_format;
	
}SENSOR_MODE_INFO_T, *SENSOR_MODE_INFO_T_PTR;

// struct: Information about rawrgb
typedef struct sensor_raw_info_tag
{
    uint32_t              res;
}SENSOR_RAW_INFO_T, *SENSOR_RAW_INFO_T_PTR;

// struct: Information about sensor extend
typedef struct sensor_extend_info_tag
{
	uint32_t focus_mode;
	uint32_t exposure_mode;
}SENSOR_EXTEND_INFO_T, *SENSOR_EXTEND_INFO_T_PTR;

// struct: Information about currect register img sensor
typedef struct sensor_register_tag
{
    //uint32_t num;
    uint32_t img_sensor_num;
    uint8_t cur_id;
    uint8_t is_register[SENSOR_ID_MAX];
}SENSOR_REGISTER_INFO_T, *SENSOR_REGISTER_INFO_T_PTR;

// struct: Information about sensor export to other module
typedef struct sensor_exp_info_tag
{
	SENSOR_IMAGE_FORMAT	image_format;			// define in SENSOR_IMAGE_FORMAT_E enum
	uint32_t 				image_pattern;			// pattern of input image form sensor	

	uint8_t				pclk_polarity;			// 0:negative; 1:positive -> polarily of pixel clock	
	uint8_t				vsync_polarity;			// 0:negative; 1:positive -> polarily of vertical synchronization signal
	uint8_t				hsync_polarity;			// 0:negative; 1:positive -> polarily of horizontal synchronization signal
	uint8_t				pclk_delay;
										
	uint16_t  			source_width_max;		// max width of source image
	uint16_t  			source_height_max;		// max height of source image

	uint32_t				environment_mode;		// environment mode type that sensor can support
	uint32_t 				image_effect;			// image effect type that sensor can support
	uint32_t				wb_mode;				// while balance type that sensor can support		
	uint32_t   			step_count;				// bit[0:7]: count of step in brightness, contrast, sharpness, saturation
												// bit[8:15] count of step in ISO
												// bit[16:23] count of step in exposure compensation
												// bit[24:31] reseved

	SENSOR_MODE_INFO_T	sensor_mode_info[SENSOR_MODE_MAX];	
	SENSOR_IOCTL_FUNC_TAB_T_PTR ioctl_func_ptr;//point to ioctl function table												

	SENSOR_RAW_INFO_T_PTR raw_info_ptr;
	SENSOR_EXTEND_INFO_T_PTR ext_info_ptr;
	uint32_t 	                    preview_skip_num;	// skip frame num before preview;
	uint32_t                         capture_skip_num;	// skip frame num before capture;	    
	uint32_t                         preview_deci_num;	// deci frame num during preview;	
	uint32_t                         video_preview_deci_num;	// deci frame num during preview;
 //       uint16_t                         atv_threshold_eb;	// threshold enable(only analog TV)		
 //       uint16_t                         atv_threshold_mode;// threshold mode 0 fix mode 1 auto mode	
 //       uint16_t                         atv_threshold_start;	// threshold start postion	
//        uint16_t                         atv_threshold_end;	// threshold end postion      

	uint16_t                         threshold_eb;	// threshold enable(only analog TV)		
	uint16_t                         threshold_mode;// threshold mode 0 fix mode 1 auto mode	
	uint16_t                         threshold_start;	// threshold start postion	
	uint16_t                         threshold_end;	// threshold end postion  
	SENSOR_INTERFACE_E  sensor_interface;
               
}SENSOR_EXP_INFO_T, *SENSOR_EXP_INFO_T_PTR;

// struct: Information about sensor from customer

typedef struct sensor_info_tag
{
	// addr
	uint8_t 			salve_i2c_addr_w;		// salve i2c write address
	uint8_t 			salve_i2c_addr_r;		// salve i2c read address
	
	uint8_t			reg_addr_value_bits;	// bit0: 0: i2c register value is 8 bit, 1: i2c register value is 16 bit
											// bit2: 0: i2c register addr  is 8 bit, 1: i2c register addr  is 16 bit
											// other bit: reseved
											
	uint8_t			hw_signal_polarity;		// bit0: 0:negative; 1:positive -> polarily of pixel clock
											// bit2: 0:negative; 1:positive -> polarily of vertical synchronization signal
											// bit4: 0:negative; 1:positive -> polarily of horizontal synchronization signal
											// other bit: reseved
	
	uint32_t			environment_mode;		// environment mode type that sensor can support
	uint32_t 			image_effect;			// image effect type that sensor can support
	uint32_t			wb_mode;				// while balance type that sensor can support		
	uint32_t   		step_count;				// bit[0:7]: count of step in brightness, contrast, sharpness, saturation
											// bit[8:15] count of step in ISO
											// bit[16:23] count of step in exposure compensation
											// bit[24:31] reseved
	
	uint16_t			reset_pulse_level;		// 1: high level valid; 0: low level valid
	uint16_t			reset_pulse_width;		// Unit: ms. Less than 200ms
	
	uint32_t			power_down_level;		// 1: high level valid; 0: low level valid
	
	uint32_t			identify_count;			// count of identify code
	SENSOR_REG_T  	identify_code[SENSOR_IDENTIFY_CODE_COUNT];		
											// supply two code to identify sensor. 
											// for Example: index = 0-> Device id, index = 1 -> version id
											
	SENSOR_AVDD_VAL_E avdd_val;				// voltage of avdd	
												
	uint16_t  		source_width_max;		// max width of source image
	uint16_t  		source_height_max;		// max height of source image
	const char* 	name;					// name of sensor

	SENSOR_IMAGE_FORMAT		image_format;	// define in SENSOR_IMAGE_FORMAT_E enum,
											// if set to SENSOR_IMAGE_FORMAT_MAX here, image format depent on SENSOR_REG_TAB_INFO_T
	uint32_t 					image_pattern;	// pattern of input image from sensor;			

	SENSOR_REG_TAB_INFO_T_PTR 	resolution_tab_info_ptr;// point to resolution table information structure	
	SENSOR_IOCTL_FUNC_TAB_T_PTR	ioctl_func_tab_ptr;		// point to ioctl function table		
	SENSOR_RAW_INFO_T_PTR  		raw_info_ptr;			// information and table about Rawrgb sensor
	SENSOR_EXTEND_INFO_T_PTR 	ext_info_ptr;			// extend information about sensor	
	SENSOR_AVDD_VAL_E iovdd_val;				// voltage of iovdd	
	SENSOR_AVDD_VAL_E dvdd_val;				// voltage of dvdd	
	uint32_t 	                    preview_skip_num;	// skip frame num before preview;
	uint32_t                         capture_skip_num;	// skip frame num before capture;	
	uint32_t                         preview_deci_num;	// deci frame num during preview;		
	uint32_t                         video_preview_deci_num;	// deci frame num during video preview;
//	uint16_t                         atv_threshold_eb;	// threshold enable(only analog TV)		
//     uint16_t                         atv_threshold_mode;  // threshold mode 0 fix mode 1 auto mode	
//	uint16_t                         atv_threshold_start;	// threshold start postion	
//	uint16_t                         atv_threshold_end;	// threshold end postion      

	uint16_t                         threshold_eb;	// threshold enable(only analog TV)		
	uint16_t                         threshold_mode;// threshold mode 0 fix mode 1 auto mode	
	uint16_t                         threshold_start;	// threshold start postion	
	uint16_t                         threshold_end;	// threshold end postion 
	int32_t                           i2c_dev_handler;
	SENSOR_INTERFACE_E  sensor_interface;		
}SENSOR_INFO_T;

//enum: preview while balance mode
typedef enum
{
         SENSOR_PARAM_WB_MODE_AUTO = 0x00, 
	SENSOR_PARAM_WB_MODE_INCANDESCENCE,
	SENSOR_PARAM_WB_MODE_U30,
	SENSOR_PARAM_WB_MODE_CWF,
	SENSOR_PARAM_WB_MODE_FLUORESCENT,
	SENSOR_PARAM_WB_MODE_SUN,  
	SENSOR_PARAM_WB_MODE_CLOUD,
	SENSOR_PARAM_WB_MODE_MAX	
}SENSOR_PARAM_WB_MODE_E;

//enum: preview enviroment type
typedef enum
{
	SENSOR_PARAM_ENVIRONMENT_NORMAL = 0x00,
	SENSOR_PARAM_ENVIRONMENT_NIGHT,
	SENSOR_PARAM_ENVIRONMENT_SUNNY,
	SENSOR_PARAM_ENVIRONMENT_SPORTS,
	SENSOR_PARAM_ENVIRONMENT_LANDSCAPE,
	SENSOR_PARAM_ENVIRONMENT_PORTRAIT,
	SENSOR_PARAM_ENVIRONMENT_PORTRAIT_NIGHT,
	SENSOR_PARAM_ENVIRONMENT_BACKLIGHT,
	SENSOR_PARAM_ENVIRONMENT_MACRO,

	SENSOR_PARAM_ENVIRONMENT_MANUAL = 30,
	SENSOR_PARAM_ENVIRONMENT_AUTO = 31,

	SENSOR_PARAM_ENVIRONMENT_MAX	
}SENSOR_PARAM_ENVIRONMENT_E;
//enum: preview effect type
typedef enum
{
	SENSOR_PARAM_EFFECT_NORMAL = 0x00,
	SENSOR_PARAM_EFFECT_BLACKWHITE,
	SENSOR_PARAM_EFFECT_RED,
	SENSOR_PARAM_EFFECT_GREEN,
	SENSOR_PARAM_EFFECT_BLUE,
	SENSOR_PARAM_EFFECT_YELLOW,
	SENSOR_PARAM_EFFECT_NEGATIVE,
	SENSOR_PARAM_EFFECT_CANVAS,
	SENSOR_PARAM_EFFECT_RELIEVOS,
	SENSOR_PARAM_EFFECT_MAX	
}SENSOR_PARAM_EFFECT_E;
/**---------------------------------------------------------------------------*
 **                         Function Definitions                              *
 **---------------------------------------------------------------------------*/
 
//------ To Sensor Module
 
/*****************************************************************************/
//  Description:    This function is used to write value to sensor register    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC int32_t Sensor_WriteReg( uint16_t  subaddr, uint16_t data );

/*****************************************************************************/
//  Description:    This function is used to read value from sensor register     
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC uint16_t Sensor_ReadReg(uint16_t subaddr);
PUBLIC int32_t Sensor_WriteReg_8bits(uint16_t reg_addr, uint8_t value);
PUBLIC int32_t Sensor_ReadReg_8bits(uint8_t reg_addr, uint8_t *reg_val);

#if 0
PUBLIC void Sensor_WriteReg2( uint8_t  subaddr, uint8_t data );
PUBLIC uint8_t  Sensor_ReadReg2(uint8_t  subaddr);
#endif

/*****************************************************************************/
//  Description:    This function is used to send a table of register to sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SendRegTabToSensor(SENSOR_REG_TAB_INFO_T * sensor_reg_tab_info_ptr	);

//------ To Digital Camera Module

/*****************************************************************************/
//  Description:    This function is used to initialize Sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
//PUBLIC BOOLEAN Sensor_Init(void);
PUBLIC uint32_t Sensor_Init(uint32_t sensor_id);

/*****************************************************************************/
//  Description:    This function is used to check if sensor has been init    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_IsInit(void);

/*****************************************************************************/
//  Description:    This function is used to Open sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
//PUBLIC ERR_SENSOR_E Sensor_Open(void);

/*****************************************************************************/
//  Description:    This function is used to set work-mode    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_SetMode(SENSOR_MODE_E mode);

/*****************************************************************************/
//  Description:    This function is used to control sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_Ioctl(uint32_t cmd, uint32_t arg);

/*****************************************************************************/
//  Description:    This function is used to Get sensor information    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_EXP_INFO_T* Sensor_GetInfo( void );

/*****************************************************************************/
//  Description:    This function is used to Close sensor function    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC ERR_SENSOR_E Sensor_Close(void) ;
/*****************************************************************************/
//  Description:    This function is used to reset sensor    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC void Sensor_Reset(uint32_t level);

/*****************************************************************************/
//  Description:    This function is used to power down sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_PowerDown(BOOLEAN power_down);
/*****************************************************************************/
//  Description:    This function is used to set AVDD
//  Author:         Liangwen.Zhen
//  Note:           Open AVDD on one special voltage or Close it
/*****************************************************************************/
PUBLIC void Sensor_SetVoltage(SENSOR_AVDD_VAL_E dvdd_val, SENSOR_AVDD_VAL_E avdd_val, SENSOR_AVDD_VAL_E iodd_val);

/*****************************************************************************/
//  Description:    This function is used to power down front sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_PowerDownFront(BOOLEAN power_down);

/*****************************************************************************/
//  Description:    This function is used to power down sensor     
//  Author:         Tim.zhu
//  Note:           
/*****************************************************************************/
PUBLIC BOOLEAN Sensor_SetResetLevel(BOOLEAN plus_level);

/*****************************************************************************/
//  Description:    This function is used to power on sensor and select xclk    
//  Author:         Liangwen.Zhen
//  Note:           1.Unit: MHz 2. if mclk equal 0, close main clock to sensor
/*****************************************************************************/
//PUBLIC void Sensor_SetMCLK(uint32_t mclk);
PUBLIC int Sensor_SetMCLK(uint32_t mclk);

/*****************************************************************************/
//  Description:    This function is used to power on sensor and select xclk    
//  Author:         Liangwen.Zhen
//  Note:           1.Unit: MHz 2. if mclk equal 0, close main clock to sensor
/*****************************************************************************/
PUBLIC BOOLEAN  Sensor_IsOpen(void) ;

/*****************************************************************************/
//  Description:    This function is used to set sensor id
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetCurId(SENSOR_ID_E sensor_id);

/*****************************************************************************/
//  Description:    This function is used to get sensor id
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_ID_E Sensor_GetCurId(void);

/*****************************************************************************/
//  Description:    This function is Get the num of register img senosr
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_REGISTER_INFO_T_PTR Sensor_GetRegisterInfo(void);

PUBLIC uint32_t Sensor_SetSensorType(SENSOR_TYPE_E sensor_type);

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
PUBLIC void ImgSensor_DeleteMutex(void);

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
PUBLIC void ImgSensor_GetMutex(void);
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
PUBLIC void ImgSensor_GetMutex(void);

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
PUBLIC void ImgSensor_PutMutex(void);

/*****************************************************************************/
//  Description:    This function is used to control flash
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_SetFlash(uint32_t is_open);

/*****************************************************************************/
//  Description:    This function is used to get i2c client
//  Author:         Tim.Zhu
//  Note:           
/*****************************************************************************/
PUBLIC struct i2c_client *Sensor_GetI2CClien(void);

/*****************************************************************************/
//  Description:    This function is used to set sensor exif info    
//  Author:         
//  Note:           
/*****************************************************************************/
PUBLIC uint32 Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_E cmd ,uint32 param);

/*****************************************************************************/
//  Description:    This function is used to Get sensor exif info    
//  Author:         
//  Note:           
/*****************************************************************************/
PUBLIC EXIF_SPEC_PIC_TAKING_COND_T* Sensor_GetSensorExifInfo( void );

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/    
#ifdef __cplusplus
}

#endif

#endif  // _SENSOR_DRV_H_

// End of sensor_drv.h
