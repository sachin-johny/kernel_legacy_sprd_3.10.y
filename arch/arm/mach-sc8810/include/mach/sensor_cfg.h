#ifndef _SENSOR_CFG_H_
#define _SENSOR_CFG_H_

#include "sensor_drv.h"

#ifdef CONFIG_MACH_SP6810A
#define PLATFORM_6810 1 //if define, it is for 6810. 
#endif

#ifdef PLATFORM_6810
#define SENSOR_MAIN_I2C_ADDR_CFG 0x3C // 0x3C--GT2005
#define SENSOR_SUB_I2C_ADDR_CFG 0x21 //GC0309
#else
#define SENSOR_MAIN_I2C_ADDR_CFG 0x30 // 0x30 //0x30--OV2655
#define SENSOR_SUB_I2C_ADDR_CFG 0x21 //OV7690
#endif

PUBLIC void Sensor_SelectSensorFormat(SENSOR_IMAGE_FORMAT data_format);
PUBLIC SENSOR_IMAGE_FORMAT Sensor_GetSensorFormat(void);
PUBLIC SENSOR_INFO_T **Sensor_GetInforTab(SENSOR_ID_E sensor_id);
PUBLIC uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id);

#endif //_SENSOR_CFG_H_
