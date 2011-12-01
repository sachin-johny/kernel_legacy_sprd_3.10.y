/******************************************************************************
 ** File Name:      chg_phy_v3.c                                                 *
 ** Author:         Benjamin.Wang                                                   *
 ** DATE:           24/11/2004                                                *
 ** Copyright:      2003 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    This file defines the basic operation interfaces of       *
 **                 charger.                                               *
*******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE                NAME                     DESCRIPTION                               *
 ** 24/11/2004      Benjamin.Wang       Create.                                   *
 ** 01/12/2009      Yi.Qiu              for SC6600L
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/
//#include "chip_plf_export.h"
//#include "..\..\analog\v3\analog_reg_v3.h"
//#include "adi_hal_internal.h"
#include <mach/chg_drvapi.h>
#include <linux/gpio.h>
#include <mach/mfp.h>
#include <mach/hardware.h>
#include <mach/regs_adi.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/regs_gpio.h>
#include <mach/regs_ana.h>
#define SCI_ASSERT 
#define SCI_TRUE true
#define SCI_FALSE false

/**---------------------------------------------------------------------------*
 **                         Debugging Flag                                    *
 **---------------------------------------------------------------------------*/
#define DEBUG_CHG

#ifdef DEBUG_CHG
#define CHG_PRINT( _format_string )   printk _format_string
#else
#define CHG_PRINT( _format_string )
#endif

/**---------------------------------------------------------------------------*
 **                         MACRO Definations                                     *
 **---------------------------------------------------------------------------*/

#define CHG_CTL             (ANA_GPIN_PG0_BASE + 0x0000)

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
extern   "C"
{
#endif

/**---------------------------------------------------------------------------*
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                     Local Function Prototypes                             *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Function Definitions                              *
 **---------------------------------------------------------------------------*/

/*****************************************************************************/
//  Description:    This function initialize charger function. It enables the control registers for
//                  GPIO48~GPIO58, sets the mask bits and sets the direction of GPIO55~58 to output.
//  Author:         Benjamin.Wang
//  Note:           This module should be initialized after GPIO_Init.
/*****************************************************************************/
 void  CHG_Init (void)
{
    /* GPIO for charge int */
//    GPIO_Enable (162);
}

/*****************************************************************************/
//  Description:    This function indicates that the charger input Vchg is above 3V.
//                  Return: SCI_TRUE - Charge INT has happened.
//                              SCI_FALSE - Charge INT hasn't happened.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_GetCHGIntStatus (void)
{
    return SCI_FALSE;
}

/*****************************************************************************/
//  Description:    This function is used to shut down the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_ShutDown (void)
{
	ANA_REG_OR (ANA_CHGR_CTRL0,CHGR_PD_BIT);
	CHG_PRINT ( ("CHGMNG:CHG_ShutDown"));
}

/*****************************************************************************/
//  Description:    This function is used to turn on the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_TurnOn (void)
{
	ANA_REG_AND (ANA_CHGR_CTRL0,~CHGR_PD_BIT);
	CHG_PRINT ( ("CHGMNG:CHG_TurnOn"));
}

/*****************************************************************************/
//  Description:    This function indicates the charger status.If it is "1", the charge is done.
//                  Return: SCI_TRUE - charge had done.
//                              SCI_FALSE - charge hadn't finished.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_GetCHGDoneStatus (void)
{
    return SCI_FALSE;       ///sc8800g don't have this function.
}

/*****************************************************************************/
//  Description:    This function restarts the charger.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetRecharge (void)
{
	ANA_REG_OR (ANA_CHGR_CTRL0,CHGR_RECHG_BIT);
    //CHG_PRINT ( ("CHGMNG:CHG_SetRecharge"));
}

/*****************************************************************************/
//  Description:    This function stops the recharge mode.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_StopRecharge (void)
{
    ANA_REG_AND (ANA_CHGR_CTRL0,~CHGR_RECHG_BIT);
    CHG_PRINT ( ("CHGMNG:CHG_StopRecharge"));
}

/*****************************************************************************/
//  Description:    This function sets the lowest switchover point between constant-current
//                      and constant-voltage modes.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetSwitchoverPoint (
    CHG_SWITPOINT_E eswitchpoint         // the input range is 0~31. shifup/down 1bit,
    // CC-CV switchover voltage shift amount, MSB, 2bits
    // CC-CV switchover voltage shift amount, LSB, 2bits
)
{
    SCI_ASSERT (eswitchpoint <=31);
    ANA_REG_MSK_OR (ANA_CHGR_CTRL1, (eswitchpoint<<CHAR_SW_POINT_SHIFT), CHAR_SW_POINT_MSK);
}

/*****************************************************************************/
//  Description:    This function is used to update one level of the lowest switchover point
//                      between constant-current and constant-voltage modes.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint32_t CHG_UpdateSwitchoverPoint (bool up_or_down)
{
    uint8_t current_switchpoint;
    uint8_t shift_bit;
    uint8_t chg_switchpoint;

    chg_switchpoint = (ANA_REG_GET (ANA_CHGR_CTRL1) & CHAR_SW_POINT_MSK) >> CHAR_SW_POINT_SHIFT;
    shift_bit = chg_switchpoint >> 4;
    current_switchpoint = chg_switchpoint&0x0F;

    if (up_or_down)
    {
        if (shift_bit > 0)
        {
            if (current_switchpoint < 0xF)
            {
                current_switchpoint += 1;
            }
        }
        else
        {
            if (current_switchpoint > 0)
            {
                current_switchpoint -= 1;
            }
            else
            {
                shift_bit = 1;
            }
        }
    }
    else
    {
        if (shift_bit > 0)
        {
            if (current_switchpoint > 0)
            {
                current_switchpoint -= 1;
            }
            else
            {
                shift_bit = 0;
            }
        }
        else
        {
            if (current_switchpoint < 0xF)
            {
                current_switchpoint += 1;
            }
        }
    }

    chg_switchpoint = (shift_bit << 4) | current_switchpoint;
    ANA_REG_MSK_OR (ANA_CHGR_CTRL1, (chg_switchpoint<<CHAR_SW_POINT_SHIFT), CHAR_SW_POINT_MSK);

    return chg_switchpoint;
}

/*****************************************************************************/
//  Description:    This function sets the lowest switchover point between constant-current
//                      and constant-voltage modes.
//                  Return: SCI_TRUE - Charger is present.
//                              SCI_FALSE - Charger had unplugged.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 bool CHG_IsChargerPresent (void)
{
    return SCI_FALSE;
}

/*****************************************************************************/
//  Description:    This function is used to set the adapter mode.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetAdapterMode (CHG_ADAPTER_MODE_E mode)
{
    if (mode == CHG_DEFAULT_MODE)
    {
        // BIT_5 reset USB_500ma_en, BIT_3 reset adapter_en
        ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (CHGR_ADATPER_EN_RST_BIT|CHGR_USB_500MA_EN_RST_BIT), CHAR_ADAPTER_MODE_MSK);
    }
    else if (mode == CHG_NORMAL_ADAPTER)
    {
        // BIT_23 reset USB_500ma_en, BIT_20 set adapter_en
        ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (CHGR_ADATPER_EN_BIT|CHGR_USB_500MA_EN_RST_BIT), CHAR_ADAPTER_MODE_MSK);
    }
    else if (mode == CHG_USB_ADAPTER)
    {
        //BIT_22 set USB_500ma_en, BIT_21 reset adapter_en
        ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (CHGR_ADATPER_EN_RST_BIT|CHGR_USB_500MA_EN_BIT), CHAR_ADAPTER_MODE_MSK);
    }
}

/*****************************************************************************/
//  Description:    This function is used to set usb charge current.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetUSBChargeCurrent (CHG_USB_CHARGE_CURRENT_E current)
{
    ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (current << CHGR_USB_CHG_SHIFT), CHGR_USB_CHG_MSK);
    CHG_PRINT ( ("CHGMNG:CHG_SetUSBChargeCurrent=%d", current));
}

/*****************************************************************************/
//  Description:    This function is used to set adapter charge current.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 void CHG_SetNormalChargeCurrent (CHG_NOR_CHARGE_CURRENT_E current)
{
    switch (current)
    {
        case CHG_NOR_300MA:
            /* only in this mode, charge current would be 300mA */
            CHG_SetAdapterMode (CHG_DEFAULT_MODE);
            return;
        case CHG_NOR_400MA:
            CHG_SetAdapterMode (CHG_NORMAL_ADAPTER);
            ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (0 << CHGR_ADAPTER_CHG_SHIFT), CHGR_ADAPTER_CHG_MSK);
            break;
        case CHG_NOR_600MA:
            CHG_SetAdapterMode (CHG_NORMAL_ADAPTER);
            ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (1 << CHGR_ADAPTER_CHG_SHIFT), CHGR_ADAPTER_CHG_MSK);
            break;
        case CHG_NOR_800MA:
            CHG_SetAdapterMode (CHG_NORMAL_ADAPTER);
            ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (2 << CHGR_ADAPTER_CHG_SHIFT), CHGR_ADAPTER_CHG_MSK);
            break;
        case CHG_NOR_1000MA:
            CHG_SetAdapterMode (CHG_NORMAL_ADAPTER);
            ANA_REG_MSK_OR (ANA_CHGR_CTRL0, (3 << CHGR_ADAPTER_CHG_SHIFT), CHGR_ADAPTER_CHG_MSK);
            break;
        default:
            SCI_ASSERT (0);
    }
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}

#endif  // End of chg_drv.c

