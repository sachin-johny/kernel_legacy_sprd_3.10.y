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

#define CHG_VOLTAGE_4V2 936
#define CHG_VOLTAGE_4V1 914
#define CHG_VOLTAGE_TEMP_4V1 914
#define CHG_VOLTAGE_4V0 892
#define CHG_VOLTAGE_3V9 870
#define CHG_VOLTAGE_3V8 848
#define CHG_VOLTAGE_3V7 826
#define CHG_VOLTAGE_3V6 804
#define CHG_VOLTAGE_3V5 782
#define CHG_VOLTAGE_3V4 760
#define CHG_VOLTAGE_3V3 738
#define CHG_VOLTAGE_3V2 716
#define CHG_VOLTAGE_3V1 694
#define CHG_VOLTAGE_3V0 672
#define CHG_VOLTAGE_2V9 650
#define CHG_VOLTAGE_2V8 628
#define CHG_VOLTAGE_2V7 606

// When lower than it ,Should shutdown the phone anyway
#define CHGMNG_VOLTAGE_DEADLINE    CHG_VOLTAGE_3V1//625
// When lower than it, Should shutdown the phone.
#define CHGMNG_VOLTAGE_SHUTDOWN    CHG_VOLTAGE_3V2//701    // 3.6V
// When lower than it, Should notify Client that need charge.
#define CHGMNG_VOLTAGE_WARNING     CHG_VOLTAGE_3V4//710        // 3.65V
// When higher than this, the capacity that report to client is 100%
#define CHGMNG_VOLTAGE_100         CHG_VOLTAGE_4V0//775        // 4.0V
// When higher than this, the capacity that report to client is 80%
#define CHGMNG_VOLTAGE_80          CHG_VOLTAGE_3V9//757        // 3.9V
// When higher than this, the capacity that report to client is 60%
#define CHGMNG_VOLTAGE_60          CHG_VOLTAGE_3V8//735        // 3.8V
// When higher than this, the capacity that report to client is 40%
#define CHGMNG_VOLTAGE_40          CHG_VOLTAGE_3V6//718        // 3.7V
// When higher than this, the capacity that report to client is 20%
// and notify client that need charge.
#define CHGMNG_VOLTAGE_20          CHG_VOLTAGE_3V4
#define CHGMNG_VOLTAGE_10          CHG_VOLTAGE_3V2
// When higher than this, the capacity that report to client is 0%
// and notify client that need power off.
#define CHGMNG_VOLTAGE_0           CHG_VOLTAGE_3V1

#define CHARGER_NORMAL_TYPE    0
#define CHARGER_NK_TYPE        1

#define CHGMNG_MAX_VCHG     0x3FF

#define PREVRECHARGE        4160//795   // 4.1V. When the battery volume is lower than this value and the charger is still plugged in, we will
// restart the charge process.
#define CHGMNG_OVER_CHARGE (4220)
#define PREVCHGEND      (4200)//816       // 4.22V. When the battery voltage is higher than this value, we will stop charging.
#define CHGMNG_SAFTY_CUTOFF_POINT 960//837   // 4.33V. When the battery voltage is higher than this value, we will stop charging forcibly.

#define BUSYSTATE       1   //when the phone is staying in busy state(for example, talking, play games or play music, etc.),we will stop
//the state timer until it is not busy.

#define POWER_WARNING_INTERVAL    5        //the interval between two warnings
#define CHR_VOLTAGE_WARNING     CHGMNG_VOLTAGE_WARNING       // 3.65V warning voltage
#define CHR_VOLTAGE_SHUTDOWN    CHGMNG_VOLTAGE_SHUTDOWN       // 3.6V shutdown voltage

#define VBAT_STATISTIC_PERIOD 2000
#define VBAT_STATISTIC_BUFFERSIZE 150

#define CHGMNG_DEFAULT_SWITPOINT CHG_SWITPOINT_20          // power up default point
#define CHARGING_DETECT_INTERVAL 1000
#define CHARGING_TOUT 18000
#define CHGMNG_SHUTDOWN_VPROG 100
#define CHGMNG_STOP_VPROG 80//0x70              // Isense stop point
#define CHGMNG_SWITCH_CV_VPROG 300//100//0x70              // Isense stop point
#define CHARGE_VBAT_STATISTIC_BUFFERSIZE 16
#define CHARGE_VPROG_STATISTIC_BUFFERSIZE 5
#define CHGMNG_PLUST_TIMES  3
#define CHARGE_BEFORE_STOP 1200
#define CHARGE_OVER_TIME 21600 /* set for charge over time, 6 hours */

#define ADC_VPROG_LIMIT    500
#define VBAT_RESULT_NUM     10       //ADC sampling number
#define VPROG_RESULT_NUM     10       //ADC sampling number
#define VBAT_RESULT_TIMEOUT 5       //resampling timeout number
#define VBAT_RESULT_DELAY   10      //time delay between AD convert
#define VBAT_RESULT_THRESHOLD   3   //VBAT result threshold

//
// battery status define
//
#define CHR_BATTERY_NONE_S          0x1    // No battery
#define CHR_BATTERY_NORMAL_S        0x2    // battery is in normal model( not in charge )
#define CHR_BATTERY_CHARGING_S      0x3    // is charging.

#define CHGMNG_STARTSUCCESS     0x0
#define CHGMNG_STARTINCHARGING  0x1

#define CHARGER_CURRENT_0       0
#define CHARGER_CURRENT_1       1
#define CHARGER_CURRENT_2       2
#define CHARGER_CURRENT_3       3

#define CHARGE_OVER_VOLTAGE 6500 
#define CHARGE_OVER_CURRENT 200

//#define VOL_TO_CUR_PARAM (576 * 3)
//#define VOL_DIV_P1 (268 * 4)
#define VOL_TO_CUR_PARAM (576)
#define VOL_DIV_P1 (268)
#define VOL_DIV_P2 1000

#define CC_CV_VOLTAGE 4200
#define CV_REF_CURRENT 4
#define CV_STOP_CURRENT 120
#define CC_CV_SWITCH_POINT 200
#define PLUSE_CURRENT 4

#define OVP_ADC_VALUE 0x198
#define OVP_ADC_RECV_VALUE 0x170

/**---------------------------------------------------------------------------*
 **                         Data Structures                                   *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Type Definations                                   *
 **---------------------------------------------------------------------------*/
 
typedef enum CHGMNG_ExceptionCode_enum
{
    CHGMNG_NOEXCEPTION = 0,
    CHGMNG_VCHGSAMPLEFAIL,
    CHGMNG_VBATSAMPLEFAIL,
    CHGMNG_TEMPSAMPLEFAIL,
    CHGMNG_BATSHORT,
    CHGMNG_VCHGOVER,
    CHGMNG_VBATOVER,
    CHGMNG_TEMPOVER
} CHGMNG_EXCEPTIONCODE_E;

typedef enum CHGMNG_StateCode_enum
{
    CHGMNG_IDLE = 0,
    CHGMNG_STARTING,
    CHGMNG_CHARGING,
    CHGMNG_PULSECHARGING,
    CHGMNG_STOPPING
} CHGMNG_STATE_E;

typedef enum CHGMNG_StopReason_enum
{
    CHGMNG_INVALIDREASON = 0,
    CHGMNG_CHARGERUNPLUG = 1,
    CHGMNG_TIMEOUT,
    CHGMNG_VBATEND,
    CHGMNG_PULSEEND,
    CHGMNG_CHARGEDONE,
    CHGMNG_OVERVOLTAGE  //add by paul for charge:over voltage
} CHGMNG_STOPREASON_E;

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

//
//the param of the callback.
//   typedef void (* REG_CALLBACK) (uint32_t id, uint32_t argc, void *argv);

//   id : the id that provided when register.
//   argc : the event; one of CHR_SVR_MSG_SERVICE_E;
//   argv : capacity. you can convert to uint32_t,
//

// The message notify to MMI
typedef enum
{
    // Charge message.
    CHR_CAP_IND = 0x1,      // Notify the battery's capacity

    CHR_CHARGE_START_IND,   // start the charge process.
    CHR_CHARGE_END_IND,     // the charge ended.

    CHR_WARNING_IND,        // the capacity is low, should charge.
    CHR_SHUTDOWN_IND,       // the capacity is very low and must shutdown.

    CHR_CHARGE_FINISH,      // the charge has been completed.
    CHR_CHARGE_DISCONNECT,  // the charge be disconnect
    CHR_CHARGE_FAULT,       // the charge fault, maybe the voltage of charge is too low.

    CHR_MSG_MAX_NUM
} CHR_SVR_MSG_SERVICE_E;

#define MAX_CHR_MSG_NUM  CHR_MSG_MAX_NUM

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
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Function Prototypes                               *
 **---------------------------------------------------------------------------*/
/*****************************************************************************/
// Check the battery status
// Return:
//  return the above battery status.
/*****************************************************************************/
uint32_t CHR_CheckBatteryStatus (void);

/*****************************************************************************/
// get the battery capacity
// Return :
//      the capacity( 0--100 ).
//          0:   no battery.
//          1 -- 99 : capacity
//          >=100   : battery full
/*****************************************************************************/
uint32_t CHR_GetBatCapacity (void);

/*****************************************************************************/
// Init the charge server.
/*****************************************************************************/
void    CHR_Init (void);

/*****************************************************************************/
// Report the capacity of battery.
/*****************************************************************************/
void  Charge_ReportCapacity (void);

/******************************************************************************
    AT Command Interface.

    关于充电管理模块需要实现下列功能：
1：stop the battery charge.
2：Start the battery charge.
3：Get the current battery voltage.
4：Set the battery charge parameters.
The below paramters:
1：level max for the battery voltage.When it is reached, the battery is considered as charged.
     (give the allowed range and the default value)
2：level min for the battery voltage.When it is reached , the battery is considered as discharged, and the module is turn off.
     (give the allowed range and the default value)
3：Time between pulses for the pulsed charge.
    (give the allowed range and the default value)
4：time pulse out charge.
     (give the allowed range and the default value)
5：battery internal resistor.
      (give the allowed range and the default value)
 *****************************************************************************/

typedef enum
{
    CHR_BATT_MIN_VOLTAGE,
    CHR_BATT_MAX_VOLTAGE,
    CHR_BATT_IN_CHARGE,
    CHR_BATT_OUT_OF_CHARGE
} CHR_BATT_STATE_E;

/*****************************************************************************/
//    return the battery state.( defined above )
/*****************************************************************************/
CHR_BATT_STATE_E   CHR_GetBattState (void);

/*****************************************************************************/
//    Start/Stop battery charge.
/*****************************************************************************/
void CHR_StopCharge (void);
void CHR_StartCharge (void);

/*****************************************************************************/
//    Get the current battery voltage( m-Voltage ).
/*****************************************************************************/
uint32_t CHR_GetBattVoltage (void);

/*@jim.zhang CR:MS8131 2004-03-25 */

/*****************************************************************************/
//    Get the battery voltage.
/*****************************************************************************/
uint32_t CHR_GetVoltage (void);

/*****************************************************************************/
//    Get the current battery voltage.
/*****************************************************************************/
uint32_t CHR_GetCurVoltage (void);

/* end CR:MS8131 2004-03-25 */

/*****************************************************************************/
//    Get battery internal resistor Param.
//    low, high : the allowed range
//    deft      : the defualt value
/*****************************************************************************/
void CHR_GetBattIntResParam (uint32_t *low, uint32_t *high, uint32_t *deft);

/*****************************************************************************/
//    Set battery internal resistor Param.
//    if the time is not in range, set failed and return 0. otherwise return 1;
/*****************************************************************************/
uint32_t CHR_SetBattIntRes (uint32_t level);

/*****************************************************************************/
//    Get the param : time pulse out charge.
//    low, high : the allowed range
//    deft      : the defualt value
/*****************************************************************************/
void CHR_GetTPulseOutChargeParam (uint32_t *low, uint32_t *high, uint32_t *deft);

/*****************************************************************************/
//    set the time
//    if the time is not in range, set failed and return 0. otherwise return 1;
/*****************************************************************************/
uint32_t CHR_SetTPulseOutCharge (uint32_t level);

/*****************************************************************************/
//    Get the param : Time between pulses for the pulsed charge.
//    low, high : the allowed range
//    deft      : the defualt value
/*****************************************************************************/
void CHR_GetTPulseInChargeParam (uint32_t *low, uint32_t *high, uint32_t *deft);

/*****************************************************************************/
//    set the time
//    if the time is not in range, set failed and return 0. otherwise return 1;
/*****************************************************************************/
uint32_t CHR_SetTPulseInCharge (uint32_t level);

/*****************************************************************************/
//    Get the level max for battery voltage.
//    low, high : the allowed range
//    deft      : the defualt value
/*****************************************************************************/
void CHR_GetBattLevelMaxParam (uint32_t *low, uint32_t *high, uint32_t *deft);

/*****************************************************************************/
//    set the level max.
//    if the level is not in range, set failed and return 0. otherwise return 1;
/*****************************************************************************/
uint32_t CHR_SetBattLevelMax (uint32_t level);

/*****************************************************************************/
//    Get the level min for battery voltage.
//    low, high : the allowed range
//    deft      : the defualt value
/*****************************************************************************/
void CHR_GetBattLevelMinParam (uint32_t *low, uint32_t *high, uint32_t *deft);

/*****************************************************************************/
//    set the level max.
//    if the level is not in range, set failed and return 0. otherwise return 1;
/*****************************************************************************/
uint32_t CHR_SetBattLevelMin (uint32_t level);

/*****************************************************************************/
// Check the charge is connect (return TRUE if the charge is connected)
//
// This function maybe use ADC(but not check ADC access conflit) and need a little time.
// And not disable the IRQ/FIQ in this function.
// So, this function should be called in init process(before thread schedule and needn't disable irq).
// And it is not recomment to use this function in other process.
// If you need, please care.
//
// Author: lin.liu
/*****************************************************************************/
bool CHR_IsChargeConnect (void);

/*****************************************************************************/
//This function provide for compatibility with prev-version.
/*****************************************************************************/
uint32_t CHR_CheckBatteryStaus (void);

/*****************************************************************************/
//  Description:    This function initialize the Charge manager. Before starting charge
//                      manager, you'd better set the essential parameters according to your need.
//                      Otherwise, it will use the default parameters.
//                      For example,
//                      {
//                          CHR_PARAM_T* chr_param = REFPARAM_GetChargeParam();
//
//                          CHGMNG_SetSwitchVoltage(chr_param->switch_voltage);
//                          CHGMNG_SetRechargeVoltage(chr_param->recharge_voltage);
//                          CHGMNG_SetChargeEndVoltage(chr_param->charge_end_voltage);
//
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_HCHARGING, chr_param->hcharging_endtime);
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_LCHARGING, chr_param->lcharging_endtime);
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_RECHARGING, chr_param->rcharging_endtime);
//
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_HCHARGING, chr_param->hcharging_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_LCHARGING, chr_param->lcharging_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_RECHARGING, chr_param->recharge_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_STOPPING, chr_param->scharging_detect_interval);
//
//                          CHGMNG_SetHWSwitchPoint((CHG_SWITPOINT_E)chr_param->hw_switch_point);
//
//                          CHGMNG_SetWarningVoltage(chr_param->voltage_warning);
//                          CHGMNG_SetShutdownVoltage(chr_param->voltage_shutdown);
//                          CHGMNG_SetWarningCount(chr_param->warning_count);
//
//                          CHGMNG_SetVBatTable((CHGMNG_VBATTABLE_T*)chr_param->adc_voltage_table);
//
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_PULSECHARGING, chr_param->pcharging_endtime);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_PULSECHARGING, chr_param->pcharge_detect_interval);
//                          CHGMNG_SetPulseVoltage(chr_param->pulse_charge_voltage);
//                          CHGMNG_SetPulseWide(chr_param->pulse_wide);
//                          CHGMNG_SetPulseWaitingTime(chr_param->pulse_waiting_time);
//
//                          CHGMNG_SetTestFlag(chr_param->charge_test_flag);
//                  }
//  Return:         CHGMNG_STARTSUCCESS - normal
//                      CHGMNG_STARTINCHARGING - t is charging when CHGMNG_Start
//  Author:         Benjamin.Wang
//  Note:           This module should be initialized after GPIO_Init.
/*****************************************************************************/
 uint32_t  CHGMNG_Start (void);

/*****************************************************************************/
//  Description:    This function is used to close charge manager.
//  Author:         Benjamin.Wang
//  Note:           It can't be called in int handler!
/*****************************************************************************/
 void CHGMNG_Close (void);

/*****************************************************************************/
//  Description:    This function is used to get battery capacity. If success, it will send a message
//                      to regiseted client.
//  Retrun:         SCI_SUCCESS - send bat capacity message successfully.
//  Author:         Benjamin.Wang
//  Note:           It can't be called in int handler!
/*****************************************************************************/
 uint32_t CHGMNG_ReportBatCapacityToAll (void);

/*****************************************************************************/
//  Description:    This function gets the restart charging voltage. After charging finished and
//                      when the battery voltage is lower than this value, we will restart the charging
//                      procedure.
//  Retrun:         recharge_voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetRechargeVoltage (void);

/*****************************************************************************/
//  Description:    This function sets the restart charging voltage.
//  Retrun:
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetRechargeVoltage (uint16_t value);

/*****************************************************************************/
//  Description:    This function gets the charge end voltage.
//  Retrun:         charge_end_voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetChargeEndVoltage (void);

/*****************************************************************************/
//  Description:    This function sets the charge end voltage.
//  Retrun:         charge_end_voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargeEndVoltage (uint16_t value);
uint32_t CHGMNG_VoltageToPercentum (uint32_t voltage, int is_charging, int update, int is_usb);

/*****************************************************************************/
//  Description:    This function gets the charge safty voltage.
//  Retrun:         g_charge_safty_voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetChargeSaftyVoltage (void);

/*****************************************************************************/
//  Description:    This function sets the charge safty voltage.
//  Retrun:         g_charge_safty_voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargeSaftyVoltage (uint16_t value);

/*****************************************************************************/
//  Description:    This function gets the charge end time. Every state has different end timer.
//  Retrun:         uint16_t value - the corresponding end time value of the given state.
//                      0 - undefined state.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetChargeEndTime (CHGMNG_STATE_E state);

/*****************************************************************************/
//  Description:    This function sets the charge end time. Every state has different end timer.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargeEndTime (CHGMNG_STATE_E state, uint16_t value);

/*****************************************************************************/
//  Description:    Return the detect interval of different states.
//  Retrun:         uint16_t value - the corresponding interval value of the given state.
//                      0 - undefined state.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetTimerInterval (CHGMNG_STATE_E state);

/*****************************************************************************/
//  Description:    Sets the detect interval of different states.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetTimerInterval (CHGMNG_STATE_E state, uint16_t value);

/*****************************************************************************/
//  Description:    Get the hardware switch point which is from cc to cv.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetHWSwitchPoint (void);

/*****************************************************************************/
//  Description:    Set the hardware switch point which is from cc to cv.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetHWSwitchPoint (CHG_SWITPOINT_E value);

/*****************************************************************************/
//  Description:    Get the battery warning voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetWarningVoltage (void);

/*****************************************************************************/
//  Description:    Set the battery warning voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetWarningVoltage (uint16_t value);

/*****************************************************************************/
//  Description:    Get the phone shutdown voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetShutdownVoltage (void);

/*****************************************************************************/
//  Description:    Set the phone shutdown voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetShutdownVoltage (uint16_t value);

/*****************************************************************************/
//  Description:    Get the phone deadline voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetDeadlineVoltage (void);

/*****************************************************************************/
//  Description:    Set the phone deadline voltage.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetDeadlineVoltage (uint16_t value);
/*****************************************************************************/
//  Description:    Get the warning waiting counts.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetWarningCount (void);

/*****************************************************************************/
//  Description:    Set the warning waiting counts.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetWarningCount (uint16_t value);

/*****************************************************************************/
//  Description:    Set the vbat calibration table.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetVBatTable (CHGMNG_VBATTABLE_T *ptable);

/*****************************************************************************/
//  Description:    Set the voltage capacity table.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetVBatCapTable (uint16_t *ptable);

/*****************************************************************************/
//  Description:    This function get the charger type, normal or Nokia type charger
//  Retrun:
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetChargerType (void);
uint32_t CHG_UpdateSwitchoverPoint (bool up_or_down);

/*****************************************************************************/
//  Description:    This function set the charger type, normal or Nokia type charger
//  Retrun:
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargerType (uint16_t type);

/*****************************************************************************/
//  Description:    This function set the charger current type
//  Retrun:
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargerCurrentType (uint16_t type);

/*****************************************************************************/
//  Description:    This function get the charger current type
//  Retrun:
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_GetChargerCurrentType (void);

/*****************************************************************************/
//  Description:    This function is used to set the adapter mode, adapter or usb.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_SetChargeAdapter (CHG_ADAPTER_MODE_E mode);
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
//  Description:    This function is used to get VBAT ADC value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint32_t CHGMNG_GetVBATADCValue (void);

/*****************************************************************************/
//  Description:    When charger pulgin/unplug interrupt happened, this function is called.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHGMNG_ChargerPlugInHandler (uint32_t gpio_id, uint32_t gpio_state);

/*****************************************************************************
    The following function is used to get or set charge ADC value.


*****************************************************************************/
 void    CHR_SetVchargeOverAdc (uint32_t over_adc_value);
 void    CHR_SetVchargeLowAdc (uint32_t low_adc_value);
 void    CHR_SetVchargeCheckFlag (bool flag);

 bool CHR_IsCheckVcharge (void);
 uint32_t CHG_GetVBATADCResult (void);

/**---------------------------------------------------------------------------*
 **                         Function Prototypes                               *
 **---------------------------------------------------------------------------*/

/*****************************************************************************/
//  Description:    This function initialize Charger function.
//  Author:         Benjamin.Wang
//  Note:           This module should be initialized after GPIO_Init.
/*****************************************************************************/
 void CHG_Init (void);

/*****************************************************************************/
//  Description:    This function indicates that the charger input Vchg is above 3V.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_GetCHGIntStatus (void);

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
//  Description:    This function indicates the charger status.If it is "1", the charge is done.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_GetCHGDoneStatus (void);

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
uint32_t CHGMNG_AdcvalueToCurrent(uint16_t voltage, uint16_t cur_type);
int charger_is_adapter(void);
int32_t CHG_GetVirtualVprog (void);


#ifdef __cplusplus
}
#endif

#endif  // _CHG_DRVAPI_H_

