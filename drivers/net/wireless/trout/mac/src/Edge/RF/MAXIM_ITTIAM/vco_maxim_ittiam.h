/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
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
/*  File Name         : vco_maxim_ittiam.h                                   */
/*                                                                           */
/*  Description       : This file contains the definitions and function      */
/*                      prototypes required to program the MAXIM RF card     */
/*                      using Ittiam PHY.                                    */
/*                                                                           */
/*  List of Functions : set_tx_power_rf                                      */
/*                      initialize_rf_2040                                   */
/*                      update_maxim_ittiam_rf_start_freq_5                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef VCO_MAXIM_ITTIAM_H
#define VCO_MAXIM_ITTIAM_H

#ifdef RF_MAXIM_ITTIAM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "phy_ittiam.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* RF Related Constants */
/* MAX2829 Registers */
#define NUM_MAXIM_REGS              13

/* MAX2829 Register Address */
#define RFMAX_REG0                  0x00
#define RFMAX_REG1                  0x01
#define RFMAX_STBY                  0x02
#define RFMAX_INT_DIV               0x03
#define RFMAX_FRAC_DIV              0x04
#define RFMAX_BSPLL                 0x05
#define RFMAX_CAL                   0x06
#define RFMAX_LPF                   0x07
#define RFMAX_RXCRSSI               0x08
#define RFMAX_TXLIN                 0x09
#define RFMAX_PABIAS                0x0A
#define RFMAX_RXGAIN                0x0B
#define RFMAX_TXGAIN                0x0C

/* MAX2829 Operating Mode */
#define RFMODE_TRANSITION           0
#define RFMODE_SPI_RESET            1
#define RFMODE_SHUTDOWN             2
#define RFMODE_STANDBY              3
#define RFMODE_RX                   4
#define RFMODE_TX                   5
#define RFMODE_TX_CALIB             6
#define RFMODE_RX_CALIB             7

/* Maxim RF Related Configurable Parameters */
#define ITTIAM_RF_CHAIN_SELECT      0xF

/* MAX2829 Register Default Values */
#define RFMAX_REG0_VAL               0x1140                  // 0x1140   0x1140
#define RFMAX_REG1_VAL               0x00CA                  // 0x00CA   0x00CA
#define RFMAX_STBY_VAL               0x1007                  // 0x3007   0x1007
#define RFMAX_INT_DIV_VAL            0x30A2                  // 0x30A2
#define RFMAX_FRAC_DIV_VAL           0x1DDD                  // 0x1DDD

#define RFMAX_BSPLL_VAL_START_FREQ_5 0x18A5                  // 0x3804   0x1964
#define RFMAX_BSPLL_VAL_START_FREQ_2 0x1964                  // 0x3804   0x1964

#define RFMAX_CAL_VAL                0x1C00                  // 0x1C00   0x1C00

#define RFMAX_LPF_VAL                0x002A
#define RFMAX_RXCRSSI_VAL            0x0C21                  // 0x0025   0x0C21

#define RFMAX_TXLIN_VAL              0x0200                  // 0x03CC   0x0200
#define RFMAX_PABIAS_VAL             0x027F                  // 0x03C0   0x027F
#define RFMAX_RXGAIN_VAL             0x007F                  // 0x0000   0x007F
#define RFMAX_TXGAIN_VAL             0x0030                  // 0x0000   0x0030

#define DEFAULT_TX_POWER             0

/*****************************************************************************/
/* 5 GHz Band, 20 MHz : MAX2829 Channel Dependent Values                     */
/*****************************************************************************/

/* Channel 36 */
#define INTDIV_VAL_CH36     0x30CF
#define FRACDIV_VAL_CH36    0x0CCC

/* Channel 40 */
#define INTDIV_VAL_CH40     0x00D0
#define FRACDIV_VAL_CH40    0x0000

/* Channel 44 */
#define INTDIV_VAL_CH44     0x00D0
#define FRACDIV_VAL_CH44    0x3333

/* Channel 48 */
#define INTDIV_VAL_CH48     0x10D1
#define FRACDIV_VAL_CH48    0x2666

/* Channel 52 */
#define INTDIV_VAL_CH52     0x20D2
#define FRACDIV_VAL_CH52    0x1999

/* Channel 56 */
#define INTDIV_VAL_CH56     0x30D3
#define FRACDIV_VAL_CH56    0x0CCC

/* Channel 60 */
#define INTDIV_VAL_CH60     0x00D4
#define FRACDIV_VAL_CH60    0x0000

/* Channel 64 */
#define INTDIV_VAL_CH64     0x00D4
#define FRACDIV_VAL_CH64    0x3333

/* Channel 100 */
#define INTDIV_VAL_CH100    0x00DC
#define FRACDIV_VAL_CH100   0x0000

/* Channel 104 */
#define INTDIV_VAL_CH104    0x00DC
#define FRACDIV_VAL_CH104   0x3333

/* Channel 108 */
#define INTDIV_VAL_CH108    0x10DD
#define FRACDIV_VAL_CH108   0x2666

/* Channel 112 */
#define INTDIV_VAL_CH112    0x20DE
#define FRACDIV_VAL_CH112   0x1999

/* Channel 116 */
#define INTDIV_VAL_CH116    0x30DF
#define FRACDIV_VAL_CH116   0x0CCC

/* Channel 120 */
#define INTDIV_VAL_CH120    0x00E0
#define FRACDIV_VAL_CH120   0x0000

/* Channel 124 */
#define INTDIV_VAL_CH124    0x00E0
#define FRACDIV_VAL_CH124   0x3333

/* Channel 128 */
#define INTDIV_VAL_CH128    0x10E1
#define FRACDIV_VAL_CH128   0x2666

/* Channel 132 */
#define INTDIV_VAL_CH132    0x20E2
#define FRACDIV_VAL_CH132   0x1999

/* Channel 136 */
#define INTDIV_VAL_CH136    0x30E3
#define FRACDIV_VAL_CH136   0x0CCC

/* Channel 140 */
#define INTDIV_VAL_CH140    0x00E4
#define FRACDIV_VAL_CH140   0x0000

/* Channel 149 */
#define INTDIV_VAL_CH149    0x00E5
#define FRACDIV_VAL_CH149   0x3333

/* Channel 153 */
#define INTDIV_VAL_CH153    0x10E6
#define FRACDIV_VAL_CH153   0x2666

/* Channel 157 */
#define INTDIV_VAL_CH157    0x20E7
#define FRACDIV_VAL_CH157   0x1999

/* Channel 161 */
#define INTDIV_VAL_CH161    0x30E8
#define FRACDIV_VAL_CH161   0x0CCC

/* Channel 165 */
#define INTDIV_VAL_CH165    0x00E9
#define FRACDIV_VAL_CH165   0x0000

/*****************************************************************************/
/* 5 GHz Band, 40 MHz : MAX2829 Channel Dependent Values                     */
/*****************************************************************************/



/*****************************************************************************/
/* 2.4 GHz Band : MAX2829 Channel Dependent Values                           */
/*****************************************************************************/

/* Channel 1 */
#define INTDIV_VAL_CH1      0x00A0
#define FRACDIV_VAL_CH1     0x3333

/* Channel 2 */
#define INTDIV_VAL_CH2      0x20A1
#define FRACDIV_VAL_CH2     0x0888

/* Channel 3 */
#define INTDIV_VAL_CH3      0x30A1
#define FRACDIV_VAL_CH3     0x1DDD

/* Channel 4 */
#define INTDIV_VAL_CH4      0x00A1
#define FRACDIV_VAL_CH4     0x3333

/* Channel 5 */
#define INTDIV_VAL_CH5      0x20A2
#define FRACDIV_VAL_CH5     0x0888

/* Channel 6 */
#define INTDIV_VAL_CH6      0x30A2
#define FRACDIV_VAL_CH6     0x1DDD

/* Channel 7 */
#define INTDIV_VAL_CH7      0x00A2
#define FRACDIV_VAL_CH7     0x3333

/* Channel 8 */
#define INTDIV_VAL_CH8      0x20A3
#define FRACDIV_VAL_CH8     0x0888

/* Channel 9 */
#define INTDIV_VAL_CH9      0x30A3
#define FRACDIV_VAL_CH9     0x1DDD

/* Channel 10 */
#define INTDIV_VAL_CH10     0x00A3
#define FRACDIV_VAL_CH10    0x3333

/* Channel 11 */
#define INTDIV_VAL_CH11     0x20A4
#define FRACDIV_VAL_CH11    0x0888

/* Channel 12 */
#define INTDIV_VAL_CH12     0x30A4
#define FRACDIV_VAL_CH12    0x1DDD

/* Channel 13 */
#define INTDIV_VAL_CH13     0x00A4
#define FRACDIV_VAL_CH13    0x3333

/* Channel 14 */
#define INTDIV_VAL_CH14     0x10A5
#define FRACDIV_VAL_CH14    0x2666

#define INVALID_VCO_VAL     0x0000

/* Maximum number of Transmit Power Levels supported */
#ifdef TX_POWER_SETTING_KLUDGE
#define MAX_TX_POWER_LEVELS 18
#else /* TX_POWER_SETTING_KLUDGE */
#define MAX_TX_POWER_LEVELS 1
#endif /* TX_POWER_SETTING_KLUDGE */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* VCO programming information */
typedef struct
{
    UWORD32 int_div;
    UWORD32 frac_div;
} vco_prog_info_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern vco_prog_info_t g_vco_info_start_freq_5_20[];
extern vco_prog_info_t g_vco_info_start_freq_5_40[];
extern vco_prog_info_t g_vco_info_start_freq_2_5[];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_maxim_ittiam_rf(UWORD8 freq, UWORD8 ch, UWORD8 sec_ch_offset,
                                 UWORD8 tx_power);
extern void set_channel(UWORD8 freq, UWORD8 channel, UWORD8 sec_ch_offset);

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_EVB
/* dumy add for Trout IC RF 20121026 */
extern void trout_ic_set_channel(UWORD8 freq, UWORD8 channel, UWORD8 sec_ch_offset);
#endif

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the gain of the Tx-VGA for all RF-Channels. For the */
/* gain setting to take effect, TX Gain Control Enable Bit (BIT0) of      */
/* RF Control Register-1 should be set.                                   */
INLINE void set_tx_power_rf(UWORD8 tx_power)
{
    /*  To be checked */
}

/* This function performs the necessary RF related initializations */
/* depending upon the operating bandwidth                          */
INLINE void update_maxim_ittiam_bw(UWORD8 bw)
{
}

/* This function updates the MAXIM RF registers specific to 5GHz frequency band */
INLINE void update_maxim_ittiam_rf_start_freq_5(void)
{
	write_RF_reg_ittiam(RFMAX_BSPLL, RFMAX_BSPLL_VAL_START_FREQ_5);
}

#endif /* RF_MAXIM_ITTIAM */

#endif /* VCO_MAXIM_ITTIAM_H */
