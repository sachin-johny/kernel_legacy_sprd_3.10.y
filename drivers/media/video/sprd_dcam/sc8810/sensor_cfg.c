/******************************************************************************
 ** File Name:      sensor_cfg.c                                                  *
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/mfp.h>
#include <mach/hardware.h>
#include <asm/io.h>

#include "sensor_drv.h"
#include "sensor_cfg.h"

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
#define SENSOR_IMAGE_FORMAT_DEFAULT			SENSOR_IMAGE_FORMAT_MAX


/**---------------------------------------------------------------------------*
 **                         extend Variables and function                     *
 **---------------------------------------------------------------------------*/
extern SENSOR_INFO_T g_OV7675_yuv_info;
extern SENSOR_INFO_T g_OV7670_yuv_info;
extern SENSOR_INFO_T g_OV9655_yuv_info;
extern SENSOR_INFO_T g_OV2640_yuv_info;
extern SENSOR_INFO_T g_OV2655_yuv_info;
extern SENSOR_INFO_T g_GC0306_yuv_info;
extern SENSOR_INFO_T g_SIV100A_yuv_info;
extern SENSOR_INFO_T g_SIV100B_yuv_info;
extern SENSOR_INFO_T g_OV3640_yuv_info;
extern SENSOR_INFO_T g_mt9m112_yuv_info;
extern SENSOR_INFO_T g_OV9660_yuv_info;
extern SENSOR_INFO_T g_OV7690_yuv_info;
extern SENSOR_INFO_T g_OV7675_yuv_info;
extern SENSOR_INFO_T g_GT2005_yuv_info;
extern SENSOR_INFO_T g_GC0309_yuv_info;
extern SENSOR_INFO_T g_ov5640_yuv_info;
extern SENSOR_INFO_T g_OV7660_yuv_info;
/**---------------------------------------------------------------------------*
 **                         analog tv                                         *
 **---------------------------------------------------------------------------*/
//extern SENSOR_INFO_T g_tlg1120_yuv_info; //wxz:???

/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL SENSOR_IMAGE_FORMAT s_sensor_image_format = SENSOR_IMAGE_FORMAT_DEFAULT;

/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/
const SENSOR_INFO_T* main_sensor_infor_tab[]=
{ 
	&g_ov5640_yuv_info,
	&g_OV2655_yuv_info,
	&g_OV7675_yuv_info,
	&g_OV2640_yuv_info,
	PNULL
};

const SENSOR_INFO_T* sub_sensor_infor_tab[]=
{
	&g_GC0309_yuv_info,//g_OV7690_yuv_info,
	PNULL
};


const SENSOR_INFO_T* atv_infor_tab[]=
{
	PNULL, 
	PNULL
};

/**---------------------------------------------------------------------------*
 **                     Local Function Prototypes                             *
 **---------------------------------------------------------------------------*/

/*****************************************************************************/
//  Description:    This function is used to select sensor data format    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC void Sensor_SelectSensorFormat(SENSOR_IMAGE_FORMAT image_format)
{
	if( SENSOR_IMAGE_FORMAT_MAX != image_format)
	{
		s_sensor_image_format = image_format;   
	}
	else
	{
		s_sensor_image_format = SENSOR_IMAGE_FORMAT_DEFAULT;
	}

	SENSOR_PRINT("Sensor_SelectSensorFormat: image_format %d, s_sensor_image_format = %d", 
		                        image_format, s_sensor_image_format);
}

/*****************************************************************************/
//  Description:    This function is used to get sensor sensor data format    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_IMAGE_FORMAT Sensor_GetSensorFormat(void)
{
	return s_sensor_image_format;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor information table    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC SENSOR_INFO_T ** Sensor_GetInforTab(SENSOR_ID_E sensor_id)
{
	SENSOR_INFO_T * sensor_infor_tab_ptr=NULL;

	switch(sensor_id)
	{
		case SENSOR_MAIN:
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&main_sensor_infor_tab;
			break;
		}
		case SENSOR_SUB:
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&sub_sensor_infor_tab;
			break;
		}
		case SENSOR_ATV: 
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&atv_infor_tab;
			break;
		}
		default:
			break;
	}

	return (SENSOR_INFO_T **)sensor_infor_tab_ptr;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor information table    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
PUBLIC uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id)
{
	uint32_t tab_lenght = 0;

	switch(sensor_id)
	{
		case SENSOR_MAIN:
		{
			tab_lenght=(sizeof(main_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		case SENSOR_SUB:
		{
			tab_lenght=(sizeof(sub_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		case SENSOR_ATV: 
		{
			tab_lenght=(sizeof(atv_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		default:
			break;
	}

	return tab_lenght;
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif  // End of sensor_drv.c


