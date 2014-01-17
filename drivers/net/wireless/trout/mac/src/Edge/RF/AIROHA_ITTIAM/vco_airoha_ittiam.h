/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : vco_airoha_ittiam.h                                  */
/*                                                                           */
/*  Description       : This file contains the definitions and function      */
/*                      prototypes required to program the Airoha RF card    */
/*                      using Ittiam PHY.                                    */
/*                                                                           */
/*  List of Functions : set_channel                                          */
/*                      perform_poweron_calibration                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef VCO_AIROHA_ITTIAM_H
#define VCO_AIROHA_ITTIAM_H

#ifdef RF_AIROHA_ITTIAM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "phy_ittiam.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/


/* Number of AL2236 Registers */
#define NUM_AIROHA_REGS            16



#ifdef AL2236

/*****************************************************************************/
/* AL2236 initialization definitions                                         */
/*****************************************************************************/

/* AL2236 Register Addresses */
#define RFAIROHA_INTDIV                          0x00
#define RFAIROHA_FRACDIV                         0x01
#define RFAIROHA_CONFIG1                         0x02
#define RFAIROHA_CONFIG2                         0x03
#define RFAIROHA_CONFIG3                         0x04
#define RFAIROHA_CONFIG4                         0x05
#define RFAIROHA_CONFIG5                         0x06
#define RFAIROHA_CONFIG6                         0x07
#define RFAIROHA_CONFIG7                         0x08
#define RFAIROHA_CONFIG8                         0x09
#define RFAIROHA_CONFIG9                         0x0A
#define RFAIROHA_CONFIG10                        0x0B
#define RFAIROHA_CONFIG11                        0x0C
#define RFAIROHA_CONFIG12                        0x0D
#define RFAIROHA_RESERVED                        0x0E
#define RFAIROHA_K_CONTROL                       0x0F


/* AL2236 Register Initialization Values */
#define RFAIROHA_INTDIV_VAL                      0x03F79
#define RFAIROHA_FRACDIV_VAL                     0x03333
#define RFAIROHA_CONFIG1_VAL                     0x00B80
#define RFAIROHA_CONFIG2_VAL                     0x00E7F
#define RFAIROHA_CONFIG3_VAL                     0x0905A
#define RFAIROHA_CONFIG4_VAL                     0x0F4DC
#define RFAIROHA_CONFIG5_VAL                     0x0805B
#define RFAIROHA_CONFIG6_VAL                     0x0116C
#define RFAIROHA_CONFIG7_VAL                     0x05B00 /* 0x05B68 - Made Gain bits 0*/
#define RFAIROHA_CONFIG8_VAL                     0x05400 /* 0x0543B - Made Gain bits 0*/
#define RFAIROHA_CONFIG9_VAL                     0x001BB
#define RFAIROHA_CONFIG10_VAL                    0x000F9
#define RFAIROHA_CONFIG11_VAL                    0x038D8
#define RFAIROHA_CONFIG12_VAL                    0x08000
#define RFAIROHA_RESERVED_VAL                    0x00000
#define RFAIROHA_K_CONTROL_VAL                   0x00587

#define RFAIROHA_K_CONTROL_DCOC_CAL              0x00D87
#define RFAIROHA_K_CONTROL_RCK_CAL               0x00787

/*****************************************************************************/
/* AL2236 2.4 GHz channel setting definitions                                */
/*****************************************************************************/

    /* Channel 1 */
    #define INTDIV_VAL_CH1      0x03F79
    #define FRACDIV_VAL_CH1     0x03333

    /* Channel 2 */
    #define INTDIV_VAL_CH2      0x03F79
    #define FRACDIV_VAL_CH2     0x0B333

    /* Channel 3 */
    #define INTDIV_VAL_CH3      0x03E79
    #define FRACDIV_VAL_CH3     0x03333

    /* Channel 4 */
    #define INTDIV_VAL_CH4      0x03E79
    #define FRACDIV_VAL_CH4     0x0B333

    /* Channel 5 */
    #define INTDIV_VAL_CH5      0x03F7A
    #define FRACDIV_VAL_CH5     0x03333

    /* Channel 6 */
    #define INTDIV_VAL_CH6      0x03F7A
    #define FRACDIV_VAL_CH6     0x0B333

    /* Channel 7 */
    #define INTDIV_VAL_CH7      0x03E7A
    #define FRACDIV_VAL_CH7     0x03333

    /* Channel 8 */
    #define INTDIV_VAL_CH8      0x03E7A
    #define FRACDIV_VAL_CH8     0x0B333

    /* Channel 9 */
    #define INTDIV_VAL_CH9      0x03F7B
    #define FRACDIV_VAL_CH9     0x03333

    /* Channel 10 */
    #define INTDIV_VAL_CH10     0x03F7B
    #define FRACDIV_VAL_CH10    0x0B333

    /* Channel 11 */
    #define INTDIV_VAL_CH11     0x03E7B
    #define FRACDIV_VAL_CH11    0x03333

    /* Channel 12 */
    #define INTDIV_VAL_CH12     0x03E7B
    #define FRACDIV_VAL_CH12    0x0B333

    /* Channel 13 */
    #define INTDIV_VAL_CH13     0x03F7C
    #define FRACDIV_VAL_CH13    0x03333

    /* Channel 14 */
    #define INTDIV_VAL_CH14     0x03E7C
    #define FRACDIV_VAL_CH14    0x06666

#elif AL7230

/*****************************************************************************/
/* AL7230 initialization definitions                                         */
/*****************************************************************************/

/* AL7230 Register Addresses */
#define RFAIROHA_INTDIV                         0x00
#define RFAIROHA_FRACDIV                        0x01
#define RFAIROHA_CONFIG1                        0x02
#define RFAIROHA_CONFIG2                        0x03
#define RFAIROHA_CONFIG3                        0x04
#define RFAIROHA_CONFIG4                        0x05
#define RFAIROHA_CONFIG5                        0x06
#define RFAIROHA_CONFIG6                        0x07
#define RFAIROHA_CONFIG7                        0x08
#define RFAIROHA_FILTER_BW                      0x09
#define RFAIROHA_RX_GAIN                        0x0A
#define RFAIROHA_TX_GAIN                        0x0B
#define RFAIROHA_PA_CURRENT                     0x0C
#define RFAIROHA_CONFIG8                        0x0D
#define RFAIROHA_CONFIG9                        0x0E
#define RFAIROHA_K_CONTROL                      0x0F

/* Unused definition */
#define RFAIROHA_RESERVED                       0xFF

#ifdef FIVE_GHZ_BAND
/*****************************************************************************/
/* AL7230 5 GHz channel setting definitions                                  */
/*****************************************************************************/

#define RFAIROHA_INTDIV_VAL                     0x0FF56
#define RFAIROHA_FRACDIV_VAL                    0x0AAAA
#define RFAIROHA_CONFIG1_VAL                    0x451FE
#define RFAIROHA_CONFIG2_VAL                    0x5FDFA
#define RFAIROHA_CONFIG3_VAL                    0x67F78
#define RFAIROHA_CONFIG4_VAL                    0x853FF
#define RFAIROHA_CONFIG5_VAL                    0x56AF3
#define RFAIROHA_CONFIG6_VAL                    0xCE020
#define RFAIROHA_CONFIG7_VAL                    0x6E8C0
#define RFAIROHA_FILTER_BW_VAL                  0x221BB
#define RFAIROHA_RX_GAIN_VAL                    0xE0600
#define RFAIROHA_TX_GAIN_VAL                    0x08000
#define RFAIROHA_PA_CURRENT_VAL                 0x00143
#define RFAIROHA_CONFIG8_VAL                    0xFFFFF
#define RFAIROHA_CONFIG9_VAL                    0x00000
#define RFAIROHA_K_CONTROL_VAL                  0x12BAC

#define K_CONTROL_DCOC_CAL                      0x9ABA8
#define K_CONTROL_RCK_CAL                       0x3ABA8


    /* Channel 36 */
#define INTDIV_VAL_CH36     0x0FF56
#define FRACDIV_VAL_CH36    0x0AAAA

    /* Channel 40 */
#define INTDIV_VAL_CH40     0x0FF57
#define FRACDIV_VAL_CH40    0x15555

    /* Channel 44 */
#define INTDIV_VAL_CH44     0x0FF57
#define FRACDIV_VAL_CH44    0x00000

    /* Channel 48 */
#define INTDIV_VAL_CH48     0x0FF57
#define FRACDIV_VAL_CH48    0x0AAAA

    /* Channel 52 */
#define INTDIV_VAL_CH52     0x0FF58
#define FRACDIV_VAL_CH52    0x15555

    /* Channel 56 */
#define INTDIV_VAL_CH56     0x0FF58
#define FRACDIV_VAL_CH56    0x00000

    /* Channel 60 */
#define INTDIV_VAL_CH60     0x0FF58
#define FRACDIV_VAL_CH60    0x0AAAA

    /* Channel 64 */
#define INTDIV_VAL_CH64     0x0FF59
#define FRACDIV_VAL_CH64    0x15555

    /* Values below are not yet programmed properly */
    /* Channel 149 */
#define INTDIV_VAL_CH149    0x0FF60
#define FRACDIV_VAL_CH149   0x18000

    /* Channel 153 */
#define INTDIV_VAL_CH153    0x0FF60
#define FRACDIV_VAL_CH153   0x02AAA

    /* Channel 157 */
#define INTDIV_VAL_CH157    0x0FF60
#define FRACDIV_VAL_CH157   0x0D555

    /* Channel 161 */
#define INTDIV_VAL_CH161    0x0FF61
#define FRACDIV_VAL_CH161   0x18000


#else /* FIVE_GHZ_BAND */

/*****************************************************************************/
/* AL7230 2.4 GHz setting definitions                                        */
/*****************************************************************************/

/* AL7230 Register Initialization Values */
#define RFAIROHA_INTDIV_VAL                      0x00379
#define RFAIROHA_FRACDIV_VAL                     0x13333
#define RFAIROHA_CONFIG1_VAL                     0x841FF
#define RFAIROHA_CONFIG2_VAL                     0x3FDFA
#define RFAIROHA_CONFIG3_VAL                     0x7FD78
#define RFAIROHA_CONFIG4_VAL                     0x802BF
#define RFAIROHA_CONFIG5_VAL                     0x56AF3
#define RFAIROHA_CONFIG6_VAL                     0xCE020
#define RFAIROHA_CONFIG7_VAL                     0x6EBC0
#define RFAIROHA_FILTER_BW_VAL                   0x221BB
#define RFAIROHA_RX_GAIN_VAL                     0xE0000
#define RFAIROHA_TX_GAIN_VAL                     0x08000
#define RFAIROHA_PA_CURRENT_VAL                  0x000A3
#define RFAIROHA_CONFIG8_VAL                     0xFFFFF
#define RFAIROHA_CONFIG9_VAL                     0x00000
#define RFAIROHA_K_CONTROL_VAL                   0x1ABA8

#define RFAIROHA_K_CONTROL_DCOC_CAL              0x9ABA8
#define RFAIROHA_K_CONTROL_RCK_CAL               0x3ABA8

/* Channel Dependent Values */
/* Channel 1 */
#define INTDIV_VAL_CH1      0x00379
#define FRACDIV_VAL_CH1     0x13333

/* Channel 2 */
#define INTDIV_VAL_CH2      0x00379
#define FRACDIV_VAL_CH2     0x1B333

/* Channel 3 */
#define INTDIV_VAL_CH3      0x00379
#define FRACDIV_VAL_CH3     0x03333

/* Channel 4 */
#define INTDIV_VAL_CH4      0x00379
#define FRACDIV_VAL_CH4     0x0B333

/* Channel 5 */
#define INTDIV_VAL_CH5      0x0037A
#define FRACDIV_VAL_CH5     0x13333

/* Channel 6 */
#define INTDIV_VAL_CH6      0x0037A
#define FRACDIV_VAL_CH6     0x1B333

/* Channel 7 */
#define INTDIV_VAL_CH7      0x0037A
#define FRACDIV_VAL_CH7     0x03333

/* Channel 8 */
#define INTDIV_VAL_CH8      0x0037A
#define FRACDIV_VAL_CH8     0x0B333

/* Channel 9 */
#define INTDIV_VAL_CH9      0x0037B
#define FRACDIV_VAL_CH9     0x13333

/* Channel 10 */
#define INTDIV_VAL_CH10     0x0037B
#define FRACDIV_VAL_CH10    0x1B333

/* Channel 11 */
#define INTDIV_VAL_CH11     0x0037B
#define FRACDIV_VAL_CH11    0x03333

/* Channel 12 */
#define INTDIV_VAL_CH12     0x0037B
#define FRACDIV_VAL_CH12    0x0B333

/* Channel 13 */
#define INTDIV_VAL_CH13     0x0037C
#define FRACDIV_VAL_CH13    0x13333

/* Channel 14 */
#define INTDIV_VAL_CH14     0x0037C
#define FRACDIV_VAL_CH14    0x06666

#endif /* FIVE_GHZ_BAND */

#endif /* AL2236 */


/*****************************************************************************/
/* AL2236 miscellaneous definitions                                          */
/*****************************************************************************/

#define DEFAULT_TX_POWER      0

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD32 g_pll2_chnl_table[];
extern UWORD32 g_pll3_chnl_table[];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_airoha_ittiam_rf(UWORD8 channel, UWORD8 tx_power, BOOL_T is_40mhz_enabled);
extern void write_dot11_phy_reg(UWORD8 ra, UWORD32 rd);
extern void read_dot11_phy_reg(UWORD8 ra, UWORD32 *rd);
extern void perform_poweron_calibration(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function programs the RF registers to switch to the desired channel */
INLINE void set_channel(UWORD8 channel,BOOL_T is_40mhz_enabled)
{
#ifndef ITTIAM_PHY_EXT_PROG

#ifdef FIVE_GHZ_BAND
        write_RF_reg_ittiam(RFAIROHA_INTDIV, g_pll2_chnl_table[channel]);
        write_RF_reg_ittiam(RFAIROHA_FRACDIV, g_pll3_chnl_table[channel]);
#else /* FIVE_GHZ_BAND */
        write_RF_reg_ittiam(RFAIROHA_INTDIV, g_pll2_chnl_table[channel]);
        write_RF_reg_ittiam(RFAIROHA_FRACDIV, g_pll3_chnl_table[channel]);
#endif /* FIVE_GHZ_BAND */
#endif /* ITTIAM_PHY_EXT_PROG */
}


/* This function sets the gain of the Tx-VGA for all RF-Channels. For the */
/* gain setting to take effect, TX Gain Control Enable Bit (BIT0) of      */
/* RF Control Register-1 should be set.                                   */
INLINE void set_tx_power_rf(UWORD8 tx_power)
{
    /*  To be checked */
}

#endif /* RF_AIROHA_ITTIAM */

#endif /* VCO_AIROHA_ITTIAM_H */
