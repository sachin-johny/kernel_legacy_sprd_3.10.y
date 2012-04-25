/******************************************************************************
 ** File Name:      chg_drvapi.h                                                *
 ** Author:         Benjamin.Wang                                             *
 ** DATE:           24/11/2004                                                *
 ** Copyright:      2003 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    This file defines the basic operation interfaces of       *
 **                 charger.                                              *
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 24/11/2004     Benjamin.Wang     Create.                                  *
 ******************************************************************************/

#ifndef _CHG_DRVAPI_H_
#define _CHG_DRVAPI_H_

#include <linux/types.h>
/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern   "C"
{
#endif

/**---------------------------------------------------------------------------*
 **                         MACRO Definations                                     *
 **---------------------------------------------------------------------------*/
// the interface that access the state of charge/battery

/**---------------------------------------------------------------------------*
 **                         MACRO Definations                                     *
 **---------------------------------------------------------------------------*/

#define PREVRECHARGE        4160//795   // 4.1V. When the battery volume is lower than this value and the charger is still plugged in, we will
// restart the charge process.
#define CHGMNG_OVER_CHARGE (4300)
#define PREVCHGEND      (4200)//816       // 4.22V. When the battery voltage is higher than this value, we will stop charging.

#define CHGMNG_DEFAULT_SWITPOINT CHG_SWITPOINT_20          // power up default point

#define CHGMNG_STOP_VPROG 80//0x70              // Isense stop point

#define CHARGE_VBAT_STATISTIC_BUFFERSIZE 16

#define CHGMNG_PLUST_TIMES  3
#define CHARGE_BEFORE_STOP 600
#define CHARGE_OVER_TIME 21600 /* set for charge over time, 6 hours */

#define VPROG_RESULT_NUM     10       //ADC sampling number

#define VBAT_RESULT_DELAY   10      //time delay between AD convert

#define CHARGE_OVER_CURRENT 200

#define VOL_TO_CUR_PARAM (576)
#define VOL_DIV_P1 (266)
#define VOL_DIV_P2 1000

#define VBAT_VOL_DIV_P1			266			///voltage divider 0.268,268/1000
#define VBAT_VOL_DIV_P2			1000		///voltage divider 0.268,268/1000  

#define VCHG_DIV_P1             75         ///voltage divider 0.0755,7555/10000
#define VCHG_DIV_P2             1000


#define CV_STOP_CURRENT 130
#define CC_CV_SWITCH_POINT 120

#define OVP_OVER_VOL 6500   //6.5V
#define OVP_RECV_VOL 5800   //5.8V

/**---------------------------------------------------------------------------*
 **                         Data Structures                                   *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Type Definations                                   *
 **---------------------------------------------------------------------------*/
 
typedef struct CHGMNG_VBATTable_struct
{
    uint16_t vbat_2_8;
    uint16_t vbat_2_9;
    uint16_t vbat_3_0;
    uint16_t vbat_3_1;
    uint16_t vbat_3_2;
    uint16_t vbat_3_3;
    uint16_t vbat_3_4;
    uint16_t vbat_3_5;
    uint16_t vbat_3_6;
    uint16_t vbat_3_7;
    uint16_t vbat_3_8;
    uint16_t vbat_3_9;
    uint16_t vbat_4_0;
    uint16_t vbat_4_1;
    uint16_t vbat_4_2;
    uint16_t reserve;
} CHGMNG_VBATTABLE_T;

/*This enum defines the lowest switchover point between constant-current and
constant-volatage.*/
typedef enum CHG_CCtoCVSwitchoverPoint_enum
{
    CHG_SWITPOINT_LOWEST = 0x0F,
    CHG_SWITPOINT_1 = 0x0E,
    CHG_SWITPOINT_2 = 0x0D,
    CHG_SWITPOINT_3 = 0x0C,
    CHG_SWITPOINT_4 = 0x0B,
    CHG_SWITPOINT_5 = 0x0A,
    CHG_SWITPOINT_6 = 0x09,
    CHG_SWITPOINT_7 = 0x08,
    CHG_SWITPOINT_8 = 0x07,
    CHG_SWITPOINT_9 = 0x06,
    CHG_SWITPOINT_10 = 0x05,
    CHG_SWITPOINT_11 = 0x04,
    CHG_SWITPOINT_12 = 0x03,
    CHG_SWITPOINT_13 = 0x02,
    CHG_SWITPOINT_14 = 0x01,
    CHG_SWITPOINT_15 = 0x00,
    CHG_SWITPOINT_16 = 0x10,
    CHG_SWITPOINT_17 = 0x11,
    CHG_SWITPOINT_18 = 0x12,
    CHG_SWITPOINT_19 = 0x13,
    CHG_SWITPOINT_20 = 0x14,
    CHG_SWITPOINT_21 = 0x15,
    CHG_SWITPOINT_22 = 0x16,
    CHG_SWITPOINT_23 = 0x17,
    CHG_SWITPOINT_24 = 0x18,
    CHG_SWITPOINT_25 = 0x19,
    CHG_SWITPOINT_26 = 0x1A,
    CHG_SWITPOINT_27 = 0x1B,
    CHG_SWITPOINT_28 = 0x1C,
    CHG_SWITPOINT_29 = 0x1D,
    CHG_SWITPOINT_30 = 0x1E,
    CHG_SWITPOINT_HIGHEST = 0x1F
}
CHG_SWITPOINT_E;

typedef enum
{
    CHG_DEFAULT_MODE = 0,
    CHG_NORMAL_ADAPTER,
    CHG_USB_ADAPTER
}
CHG_ADAPTER_MODE_E;

typedef enum
{
    CHG_USB_300MA = 0,
    CHG_USB_400MA = 0x1,
    CHG_USB_500MA = 0x3
}
CHG_USB_CHARGE_CURRENT_E;

typedef enum
{
    CHG_NOR_300MA = 0,
    CHG_NOR_400MA = 0x1,
    CHG_NOR_500MA = 0x2,
    CHG_NOR_600MA = 0x3,
    CHG_NOR_800MA = 0x4,
    CHG_NOR_1000MA = 0x5
}
CHG_NOR_CHARGE_CURRENT_E;

/**---------------------------------------------------------------------------*
 **                         Function Prototypes                               *
 **---------------------------------------------------------------------------*/

uint32_t CHGMNG_VoltageToPercentum (uint32_t voltage, int is_charging, int update, int is_usb);

uint32_t CHG_UpdateSwitchoverPoint (bool up_or_down);

/*****************************************************************************/
//  Description:    This function is used to convert voltage value to ADC value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_VoltageToAdcvalue (uint16_t votage);

/*****************************************************************************/
//  Description:    This function is used to convert ADC value to voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_AdcvalueToVoltage (uint16_t adcvalue);
 uint16_t CHGMNG_ChargerAdcvalueToVoltage (uint16_t adcvalue);

/*****************************************************************************/
//  Description:    This function initialize Charger function.
//  Author:         Benjamin.Wang
//  Note:           This module should be initialized after GPIO_Init.
/*****************************************************************************/
 void CHG_Init (void);

/*****************************************************************************/
//  Description:    This function is used to shut down the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_ShutDown (void);

/*****************************************************************************/
//  Description:    This function is used to turn on the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_TurnOn (void);

/*****************************************************************************/
//  Description:    This function restart the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetRecharge (void);

/*****************************************************************************/
//  Description:    This function stops the recharge mode.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_StopRecharge (void);

/*****************************************************************************/
//  Description:    This function sets the lowest switchover point between constant-current
//                      and constant-voltage modes.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetSwitchoverPoint (CHG_SWITPOINT_E eswitchpoint);
 uint32_t CHG_GetSwitchoverPoint (void);

/*****************************************************************************/
//  Description:    This function is used to update one level of the lowest switchover point
//                      between constant-current and constant-voltage modes.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint32_t CHG_UpdateSwitchoverPoint (bool);

/*****************************************************************************/
//  Description:    This function sets the lowest switchover point between constant-current
//                      and constant-voltage modes.
//                  Return: SCI_TRUE - Charger is present.
//                              SCI_FALSE - Charger had unplugged.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_IsChargerPresent (void);

/*****************************************************************************/
//  Description:    This function is used to set the adapter mode.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetAdapterMode (CHG_ADAPTER_MODE_E mode);

/*****************************************************************************/
//  Description:    This function is used to set USB charge current.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetUSBChargeCurrent (CHG_USB_CHARGE_CURRENT_E current);

/*****************************************************************************/
//  Description:    This function is used to set adapter charge current.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
void CHG_SetNormalChargeCurrent (CHG_NOR_CHARGE_CURRENT_E current);
int CHGMNG_AdcvalueToTemp(uint16_t adcvalue);
uint32_t CHGMNG_AdcvalueToCurrent(uint16_t vprog_adc, uint16_t cur_type);
int charger_is_adapter(void);
int32_t CHG_GetVirtualVprog (void);


#ifdef __cplusplus
}
#endif

#endif  // _CHG_DRVAPI_H_

