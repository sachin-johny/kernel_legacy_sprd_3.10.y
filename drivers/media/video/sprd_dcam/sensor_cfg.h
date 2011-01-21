#ifndef _SENSOR_CFG_H_
#define _SENSOR_CFG_H_

#include "sensor_drv.h"

PUBLIC void Sensor_SelectSensorFormat(SENSOR_IMAGE_FORMAT data_format);
PUBLIC SENSOR_IMAGE_FORMAT Sensor_GetSensorFormat(void);
PUBLIC SENSOR_INFO_T **Sensor_GetInforTab(SENSOR_ID_E sensor_id);
PUBLIC uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id);

#endif //_SENSOR_CFG_H_
