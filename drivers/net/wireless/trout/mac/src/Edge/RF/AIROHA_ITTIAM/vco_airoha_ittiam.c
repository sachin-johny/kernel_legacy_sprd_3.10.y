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
/*  File Name         : vco_airoha_ittiam.c                                  */
/*                                                                           */
/*  Description       : This file contains the functions required to program */
/*                      the Airoha RF card using Ittiam PHY.                 */
/*                                                                           */
/*  List of Functions : init_maxim_ittiam_rf                                 */
/*                      reg_ad9863_init_spi                                  */
/*                      reg_maxim_init_spi                                   */
/*                      reset_signal                                         */
/*                      enable_signal                                        */
/*                      band_select                                          */
/*                      reg_ad9863_write_spi                                 */
/*                      reg_maxim_write_spi                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef RF_AIROHA_ITTIAM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "vco_airoha_ittiam.h"
#include "rf_if.h"
#include "phy_ittiam.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD32 g_airoha_reg[NUM_AIROHA_REGS] = {
#ifdef AL2236
     /* 0x00 */         RFAIROHA_INTDIV_VAL,
     /* 0x01 */         RFAIROHA_FRACDIV_VAL,
     /* 0x02 */         RFAIROHA_CONFIG1_VAL,
     /* 0x03 */         RFAIROHA_CONFIG2_VAL,
     /* 0x04 */         RFAIROHA_CONFIG3_VAL,
     /* 0x05 */         RFAIROHA_CONFIG4_VAL,
     /* 0x06 */         RFAIROHA_CONFIG5_VAL,
     /* 0x07 */         RFAIROHA_CONFIG6_VAL,
     /* 0x08 */         RFAIROHA_CONFIG7_VAL,
     /* 0x09 */         RFAIROHA_CONFIG8_VAL,
     /* 0x0A */         RFAIROHA_CONFIG9_VAL,
     /* 0x0B */         RFAIROHA_CONFIG10_VAL,
     /* 0x0C */         RFAIROHA_CONFIG11_VAL,
     /* 0x0D */         RFAIROHA_CONFIG12_VAL,
     /* 0x0E */         RFAIROHA_RESERVED_VAL,
     /* 0x0F */         RFAIROHA_K_CONTROL_VAL
#else /* AL2236 */
     /* 0x00 */         RFAIROHA_INTDIV_VAL,
     /* 0x01 */         RFAIROHA_FRACDIV_VAL,
     /* 0x02 */         RFAIROHA_CONFIG1_VAL,
     /* 0x03 */         RFAIROHA_CONFIG2_VAL,
     /* 0x04 */         RFAIROHA_CONFIG3_VAL,
     /* 0x05 */         RFAIROHA_CONFIG4_VAL,
     /* 0x06 */         RFAIROHA_CONFIG5_VAL,
     /* 0x07 */         RFAIROHA_CONFIG6_VAL,
     /* 0x08 */         RFAIROHA_CONFIG7_VAL,
     /* 0x09 */         RFAIROHA_FILTER_BW_VAL,
     /* 0x0A */         RFAIROHA_RX_GAIN_VAL,
     /* 0x0B */         RFAIROHA_TX_GAIN_VAL,
     /* 0x0C */         RFAIROHA_PA_CURRENT_VAL,
     /* 0x0D */         RFAIROHA_CONFIG8_VAL,
     /* 0x0E */         RFAIROHA_CONFIG9_VAL,
     /* 0x0F */         RFAIROHA_K_CONTROL_VAL
#endif /* AL2236 */
};

UWORD8 g_ad9863_reg[NUM_AD9863_REGS] = {
     /* 0x0  */         MFE_GEN_VAL,
     /* 0x1  */         MFE_CM_VAL,
     /* 0x2  */         MFE_PD_VAL,
     /* 0x3  */         MFE_RAPD_VAL,
     /* 0x4  */         MFE_RBPD_VAL,
     /* 0x5  */         MFE_RPD_VAL,
     /* 0x6  */         MFE_RX1_VAL,
     /* 0x7  */         MFE_RX2_VAL,
     /* 0x8  */         MFE_RX3_VAL,
     /* 0x9  */         MFE_RX4_VAL,
     /* 0xA  */         MFE_RX5_VAL,
     /* 0xB  */         MFE_TX1_VAL,
     /* 0xC  */         MFE_TX2_VAL,
     /* 0xD  */         MFE_TX3_VAL,
     /* 0xE  */         MFE_TX4_VAL,
     /* 0xF  */         MFE_TX5_VAL,
     /* 0x10 */         MFE_TX6_VAL,
     /* 0x11 */         MFE_TX7_VAL,
     /* 0x12 */         MFE_TX8_VAL,
     /* 0x13 */         MFE_IOC1_VAL,
     /* 0x14 */         MFE_IOC2_VAL,
     /* 0x15 */         MFE_CLK1_VAL,
     /* 0x16 */         MFE_CLK2_VAL
};

UWORD32 g_pll2_chnl_table[NUM_CHANNELS] = {0};
UWORD32 g_pll3_chnl_table[NUM_CHANNELS] = {0};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_airoha_ittiam_rf                                    */
/*                                                                           */
/*  Description   : This function initializes the RF card in use.            */
/*                                                                           */
/*  Inputs        : 1) Channel                                               */
/*                  2) Transmit power                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the registers to their initial value  */
/*                  and sets channel and transmit power to default values.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_airoha_ittiam_rf(UWORD8 channel, UWORD8 tx_power, BOOL_T is_40mhz_enabled)
{
    UWORD8 idx = 0;

    /* Initialize AD9863 Registers */
    for(idx = 0; idx < NUM_AD9863_REGS; idx++)
        write_mxfe_reg(idx, g_ad9863_reg[idx]);

    /* put PHY in shutdown state for pulling down PLLON */
    /* This is done for initialising the RF registers */
    write_dot11_phy_reg(rPHYPOWDNCNTRL, PHYPOWDNCNTRL &(~(BIT0+BIT1)));


    /* Initialize Airoha Registers */
    for(idx = 0; idx < NUM_AIROHA_REGS; idx++)
{
        if(idx != RFAIROHA_RESERVED)
            write_RF_reg(idx, g_airoha_reg[idx]);
}

    /* Bring PHY out of Shutdown to PULL up PLLON */
    /* TXON, RXON & PAON should be low at this time */
    write_dot11_phy_reg(rPHYPOWDNCNTRL,              PHYPOWDNCNTRL);

    /* Call function to perform power on calibration for Airoha RF */
    perform_poweron_calibration();

    /* Program the RF and Mixed Signal Chips */
    /* Switch to the desired channel */
    set_channel(channel,is_40mhz_enabled);

    /* Set the Tx-Power level */
    set_tx_power_rf(tx_power);

    PRINTK("performing power on calibration\n");
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : perform_poweron_calibration                              */
/*                                                                           */
/*  Description   : This function performs power on calibration for the RF   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function performs power on calibration for al2236   */
/*                  by setting appropriate registers                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void perform_poweron_calibration(void)
{

    add_calib_delay(20); /* Tune it to more than 150uS */
    write_RF_reg(RFAIROHA_K_CONTROL,  RFAIROHA_K_CONTROL_DCOC_CAL     );
    add_calib_delay(6); /* Tune it to more than 50uS  */
    write_RF_reg(RFAIROHA_K_CONTROL,  RFAIROHA_K_CONTROL_RCK_CAL  );
    add_calib_delay(6); /* Tune it to more than 50uS  */
    write_RF_reg(RFAIROHA_K_CONTROL,  RFAIROHA_K_CONTROL_VAL  );
    add_calib_delay(6); /* Tune it to more than 50uS  */
}


#endif /* RF_AIROHA_ITTIAM */
