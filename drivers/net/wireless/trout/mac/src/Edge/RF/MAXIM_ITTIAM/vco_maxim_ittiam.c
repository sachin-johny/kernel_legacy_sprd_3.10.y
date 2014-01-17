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
/*  File Name         : vco_maxim_ittiam.c                                   */
/*                                                                           */
/*  Description       : This file contains the functions required to program */
/*                      the MAXIM RF card using Ittiam PHY.                  */
/*                                                                           */
/*  List of Functions : init_maxim_ittiam_rf                                 */
/*                      set_channel                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef RF_MAXIM_ITTIAM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "vco_maxim_ittiam.h"
#include "mh.h"

#ifdef TROUT2_WIFI_IC /* Export from Trout_sdio.ko :: zhangym 0403 */
extern int  MxdRfSetFreqWifiCh(unsigned int chNo);
extern int update_cfg_front_nv(void);//by lihua
#endif

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD32 g_max2829_reg[NUM_MAXIM_REGS] = {
     /* 0x00 */         RFMAX_REG0_VAL,
     /* 0x01 */         RFMAX_REG1_VAL,
     /* 0x02 */         RFMAX_STBY_VAL,
     /* 0x03 */         RFMAX_INT_DIV_VAL,
     /* 0x04 */         RFMAX_FRAC_DIV_VAL,
     /* 0x05 */         RFMAX_BSPLL_VAL_START_FREQ_2,
     /* 0x06 */         RFMAX_CAL_VAL,
     /* 0x07 */         RFMAX_LPF_VAL,
     /* 0x08 */         RFMAX_RXCRSSI_VAL,
     /* 0x09 */         RFMAX_TXLIN_VAL,
     /* 0x0A */         RFMAX_PABIAS_VAL,
     /* 0x0B */         RFMAX_RXGAIN_VAL,
     /* 0x0C */         RFMAX_TXGAIN_VAL
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

/* Note that this list must be in increasing order of channels as per the    */
/* channel index definition (CH_NUM_FREQ_5_T) so that the index in this      */
/* table can be derived from the channel index correctly.                    */
vco_prog_info_t g_vco_info_start_freq_5_20[NUM_SUPP_CHANNEL_FREQ_5] = {
    {INTDIV_VAL_CH36,  FRACDIV_VAL_CH36},
    {INTDIV_VAL_CH40,  FRACDIV_VAL_CH40},
    {INTDIV_VAL_CH44,  FRACDIV_VAL_CH44},
    {INTDIV_VAL_CH48,  FRACDIV_VAL_CH48},
    {INTDIV_VAL_CH52,  FRACDIV_VAL_CH52},
    {INTDIV_VAL_CH56,  FRACDIV_VAL_CH56},
    {INTDIV_VAL_CH60,  FRACDIV_VAL_CH60},
    {INTDIV_VAL_CH64,  FRACDIV_VAL_CH64},
    {INTDIV_VAL_CH100, FRACDIV_VAL_CH100},
    {INTDIV_VAL_CH104, FRACDIV_VAL_CH104},
    {INTDIV_VAL_CH108, FRACDIV_VAL_CH108},
    {INTDIV_VAL_CH112, FRACDIV_VAL_CH112},
    {INTDIV_VAL_CH116, FRACDIV_VAL_CH116},
    {INTDIV_VAL_CH120, FRACDIV_VAL_CH120},
    {INTDIV_VAL_CH124, FRACDIV_VAL_CH124},
    {INTDIV_VAL_CH128, FRACDIV_VAL_CH128},
    {INTDIV_VAL_CH132, FRACDIV_VAL_CH132},
    {INTDIV_VAL_CH136, FRACDIV_VAL_CH136},
    {INTDIV_VAL_CH140, FRACDIV_VAL_CH140},
    {INTDIV_VAL_CH149, FRACDIV_VAL_CH149},
    {INTDIV_VAL_CH153, FRACDIV_VAL_CH153},
    {INTDIV_VAL_CH157, FRACDIV_VAL_CH157},
    {INTDIV_VAL_CH161, FRACDIV_VAL_CH161},
    {INTDIV_VAL_CH165, FRACDIV_VAL_CH165}
};


/* Note that this list must be in increasing order of channels as per the    */
/* channel index definition (CH_NUM_FREQ_2_T) so that the index in this      */
/* table can be derived from the channel index correctly.                    */
vco_prog_info_t g_vco_info_start_freq_2_5[NUM_SUPP_CHANNEL_FREQ_2] = {
    {INTDIV_VAL_CH1,  FRACDIV_VAL_CH1},
    {INTDIV_VAL_CH2,  FRACDIV_VAL_CH2},
    {INTDIV_VAL_CH3,  FRACDIV_VAL_CH3},
    {INTDIV_VAL_CH4,  FRACDIV_VAL_CH4},
    {INTDIV_VAL_CH5,  FRACDIV_VAL_CH5},
    {INTDIV_VAL_CH6,  FRACDIV_VAL_CH6},
    {INTDIV_VAL_CH7,  FRACDIV_VAL_CH7},
    {INTDIV_VAL_CH8,  FRACDIV_VAL_CH8},
    {INTDIV_VAL_CH9,  FRACDIV_VAL_CH9},
    {INTDIV_VAL_CH10, FRACDIV_VAL_CH10},
    {INTDIV_VAL_CH11, FRACDIV_VAL_CH11},
    {INTDIV_VAL_CH12, FRACDIV_VAL_CH12},
    {INTDIV_VAL_CH13, FRACDIV_VAL_CH13},
    {INTDIV_VAL_CH14, FRACDIV_VAL_CH14}
};

/* NOTE - The power levels (dbm) must be in descending order strictly */
#ifdef TX_POWER_SETTING_KLUDGE
WORD8 g_tx_power_levels_in_dbm[MAX_TX_POWER_LEVELS] = {
    17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
};
#else /* TX_POWER_SETTING_KLUDGE */
WORD8 g_tx_power_levels_in_dbm[MAX_TX_POWER_LEVELS] = {
    12
};
#endif /* TX_POWER_SETTING_KLUDGE */

#ifdef TX_POWER_MAINTAIN_PER_CHANNEL

UWORD8 g_gaincode_freq_5[NUM_SUPP_CHANNEL_FREQ_5][MAX_TX_POWER_LEVELS] = {
    {52, }, /* TX power levels 1 - Max: Channel 36  */
    {52, }, /* TX power levels 1 - Max: Channel 40  */
    {52, }, /* TX power levels 1 - Max: Channel 44  */
    {52, }, /* TX power levels 1 - Max: Channel 48  */
    {52, }, /* TX power levels 1 - Max: Channel 52  */
    {52, }, /* TX power levels 1 - Max: Channel 56  */
    {52, }, /* TX power levels 1 - Max: Channel 60  */
    {52, }, /* TX power levels 1 - Max: Channel 64  */
    {52, }, /* TX power levels 1 - Max: Channel 100 */
    {52, }, /* TX power levels 1 - Max: Channel 104 */
    {52, }, /* TX power levels 1 - Max: Channel 108 */
    {52, }, /* TX power levels 1 - Max: Channel 112 */
    {52, }, /* TX power levels 1 - Max: Channel 116 */
    {52, }, /* TX power levels 1 - Max: Channel 120 */
    {52, }, /* TX power levels 1 - Max: Channel 124 */
    {52, }, /* TX power levels 1 - Max: Channel 128 */
    {52, }, /* TX power levels 1 - Max: Channel 132 */
    {52, }, /* TX power levels 1 - Max: Channel 136 */
    {52, }, /* TX power levels 1 - Max: Channel 140 */
    {52, }, /* TX power levels 1 - Max: Channel 149 */
    {52, }, /* TX power levels 1 - Max: Channel 153 */
    {52, }, /* TX power levels 1 - Max: Channel 157 */
    {52, }, /* TX power levels 1 - Max: Channel 161 */
    {52, }, /* TX power levels 1 - Max: Channel 165 */
};

UWORD8 g_gaincode_freq_2[NUM_SUPP_CHANNEL_FREQ_2][MAX_TX_POWER_LEVELS] = {
    {52, }, /* TX power levels 1 - Max: Channel  1 */
    {52, }, /* TX power levels 1 - Max: Channel  2 */
    {52, }, /* TX power levels 1 - Max: Channel  3 */
    {52, }, /* TX power levels 1 - Max: Channel  4 */
    {52, }, /* TX power levels 1 - Max: Channel  5 */
    {52, }, /* TX power levels 1 - Max: Channel  6 */
    {52, }, /* TX power levels 1 - Max: Channel  7 */
    {52, }, /* TX power levels 1 - Max: Channel  8 */
    {52, }, /* TX power levels 1 - Max: Channel  9 */
    {52, }, /* TX power levels 1 - Max: Channel 10 */
    {52, }, /* TX power levels 1 - Max: Channel 11 */
    {52, }, /* TX power levels 1 - Max: Channel 12 */
    {52, }, /* TX power levels 1 - Max: Channel 13 */
    {52, }, /* TX power levels 1 - Max: Channel 14 */
};

#ifdef TX_POWER_LOOPBACK_ENABLED
UWORD8 g_tssi_freq_5[NUM_SUPP_CHANNEL_FREQ_5][MAX_TX_POWER_LEVELS] = {
    {0, }, /* TX power levels 1 - Max: Channel 36  */
    {0, }, /* TX power levels 1 - Max: Channel 40  */
    {0, }, /* TX power levels 1 - Max: Channel 44  */
    {0, }, /* TX power levels 1 - Max: Channel 48  */
    {0, }, /* TX power levels 1 - Max: Channel 52  */
    {0, }, /* TX power levels 1 - Max: Channel 56  */
    {0, }, /* TX power levels 1 - Max: Channel 60  */
    {0, }, /* TX power levels 1 - Max: Channel 64  */
    {0, }, /* TX power levels 1 - Max: Channel 100 */
    {0, }, /* TX power levels 1 - Max: Channel 104 */
    {0, }, /* TX power levels 1 - Max: Channel 108 */
    {0, }, /* TX power levels 1 - Max: Channel 112 */
    {0, }, /* TX power levels 1 - Max: Channel 116 */
    {0, }, /* TX power levels 1 - Max: Channel 120 */
    {0, }, /* TX power levels 1 - Max: Channel 124 */
    {0, }, /* TX power levels 1 - Max: Channel 128 */
    {0, }, /* TX power levels 1 - Max: Channel 132 */
    {0, }, /* TX power levels 1 - Max: Channel 136 */
    {0, }, /* TX power levels 1 - Max: Channel 140 */
    {0, }, /* TX power levels 1 - Max: Channel 149 */
    {0, }, /* TX power levels 1 - Max: Channel 153 */
    {0, }, /* TX power levels 1 - Max: Channel 157 */
    {0, }, /* TX power levels 1 - Max: Channel 161 */
    {0, }, /* TX power levels 1 - Max: Channel 165 */
};

UWORD8 g_tssi_freq_2[NUM_SUPP_CHANNEL_FREQ_2][MAX_TX_POWER_LEVELS] = {
    {0, }, /* TX power levels 1 - Max: Channel  1 */
    {0, }, /* TX power levels 1 - Max: Channel  2 */
    {0, }, /* TX power levels 1 - Max: Channel  3 */
    {0, }, /* TX power levels 1 - Max: Channel  4 */
    {0, }, /* TX power levels 1 - Max: Channel  5 */
    {0, }, /* TX power levels 1 - Max: Channel  6 */
    {0, }, /* TX power levels 1 - Max: Channel  7 */
    {0, }, /* TX power levels 1 - Max: Channel  8 */
    {0, }, /* TX power levels 1 - Max: Channel  9 */
    {0, }, /* TX power levels 1 - Max: Channel 10 */
    {0, }, /* TX power levels 1 - Max: Channel 11 */
    {0, }, /* TX power levels 1 - Max: Channel 12 */
    {0, }, /* TX power levels 1 - Max: Channel 13 */
    {0, }, /* TX power levels 1 - Max: Channel 14 */
};
#endif /* TX_POWER_LOOPBACK_ENABLED */

#else /* TX_POWER_MAINTAIN_PER_CHANNEL */

#ifdef TX_POWER_SETTING_KLUDGE
/* TX power levels 1 - Max: All Channels */
UWORD8 g_gaincode_freq_5[1][MAX_TX_POWER_LEVELS] = {
    {52, 49,  46,  43,  40,  37,  34,  31,  28,  25,  22,  19,  16,  13,  9,  6,   3,  0}
};

UWORD8 g_gaincode_freq_2[1][MAX_TX_POWER_LEVELS] = {
    {52, 49,  46,  43,  40,  37,  34,  31,  28,  25,  22,  19,  16,  13,  9,  6,   3,  0}
};

#else /* TX_POWER_SETTING_KLUDGE */
UWORD8 g_gaincode_freq_5[1][MAX_TX_POWER_LEVELS] = {
    {52, } /* TX power levels 1 - Max: All Channels */
};

UWORD8 g_gaincode_freq_2[1][MAX_TX_POWER_LEVELS] = {
    {52, } /* TX power levels 1 - Max: All Channels */
};
#endif /* TX_POWER_SETTING_KLUDGE */

#ifdef TX_POWER_LOOPBACK_ENABLED

UWORD8 g_tssi_freq_5[1][MAX_TX_POWER_LEVELS] = {
    {0, } /* TX power levels 1 - Max: All Channels */
};

UWORD8 g_tssi_freq_2[1][MAX_TX_POWER_LEVELS] = {
    {0, } /* TX power levels 1 - Max: All Channels */
};

#endif /* TX_POWER_LOOPBACK_ENABLED */

#endif /* TX_POWER_MAINTAIN_PER_CHANNEL */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_maxim_ittiam_rf                                     */
/*                                                                           */
/*  Description   : This function initializes the Maxim RF with Ittiam PHY.  */
/*                                                                           */
/*  Inputs        : 1) Frequency band                                        */
/*                  2) Channel index                                         */
/*                  3) Secondary channel offset                              */
/*                  4) Transmit power                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes all the registers and sets the */
/*                  the channel and transmit power to given values.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
static int run_flag = 0;//dumy add 0130 for BT debug
void init_maxim_ittiam_rf(UWORD8 freq, UWORD8 channel, UWORD8 sec_ch_offset,
                          UWORD8 tx_power)
{
	TROUT_FUNC_ENTER;
    write_dot11_phy_reg(0xFF, 0x00);

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA //dumy mark 20120618
    UWORD8 idx = 0;
    /* Initialize AD9863 Registers */
    for(idx = 0; idx < NUM_AD9863_REGS; idx++)
        write_mxfe_reg(idx, g_ad9863_reg[idx]);

    /* Initialize MAXIM Registers */
    for(idx = 0; idx < NUM_MAXIM_REGS; idx++)
        write_RF_reg(idx, g_max2829_reg[idx]);
#endif

#ifdef TROUT2_WIFI_FPGA //dumy mark 20120618

#ifdef TROUT2_WIFI_FPGA_RF55

	for(idx = 0; idx < NUM_AD9863_REGS; idx++)
				write_mxfe_reg(idx, g_ad9863_reg[idx]);

	if(run_flag == 0)//dumy add 0130 for BT debug
	{
		run_flag = 1;
		//for(idx = 0; idx < NUM_AD9863_REGS; idx++)
		//	write_mxfe_reg(idx, g_ad9863_reg[idx]);

		printk("start adi and mxd init!\n");
		trout2_fpga_rf55_adi_init();
		mxd_wifi_rf_initial();
	}
#endif

#ifdef TROUT2_WIFI_FPGA_RF2829
    /* Initialize AD9863 Registers */
    for(idx = 0; idx < NUM_AD9863_REGS; idx++)
        write_mxfe_reg(idx, g_ad9863_reg[idx]);

    /* Initialize MAXIM Registers */
    for(idx = 0; idx < NUM_MAXIM_REGS; idx++)
        write_RF_reg(idx, g_max2829_reg[idx]);
#endif

#endif

#ifdef TROUT2_WIFI_IC

	if(run_flag == 0)//dumy add 0130 for BT debug
	{
		run_flag = 1;

		//TROUT_DBG4("&&&& Trout2_ic_rf55_adi Config AD/DA. \n");
		//Trout2_ADC_DAC_Init();  //removed by zhangym(0415),move to public interface 
		
		//TROUT_DBG4("&&&& MxdRf start \n");
		//MxdRfStart();// 0328 added by zhangym
#if 1
 		/* system reg :0x4D --> bit 1 --0 ,not bypass--qinning*/
		TROUT_DBG4("0x4d(bit1->0) not Bypass. \n");
		host_write_trout_reg(host_read_trout_reg(((UWORD32)0x4D)<<2)&( ~BIT1),\
					(((UWORD32)0x4D)<<2));
#else		   
		//set 0x54, 55, 56 value 0xFF  phy reg bank1 
		write_dot11_phy_reg(0x54, 0xFF);
		write_dot11_phy_reg(0x55, 0xFF);
		write_dot11_phy_reg(0x56, 0xFF);
#endif
	}
	
#endif

#ifdef TROUT_WIFI_EVB //dumy mark 20120618

#ifdef TROUT_WIFI_RVB_RF55

	for(idx = 0; idx < NUM_AD9863_REGS; idx++)
				write_mxfe_reg(idx, g_ad9863_reg[idx]);

	if(run_flag == 0)//dumy add 0130 for BT debug
	{
		run_flag = 1;
		//for(idx = 0; idx < NUM_AD9863_REGS; idx++)
		//	write_mxfe_reg(idx, g_ad9863_reg[idx]);

		printk("start adi and mxd init!\n");
		trout2_fpga_rf55_adi_init();
		mxd_wifi_rf_initial();
	}
#else

	#ifdef TROUT_RF_SPI_MODE
	    trout_ic_rf_spi_init();
	#else
	    trout_ic_rf_adi_init();//ADI RF
	#endif

	//chenq mod 2012-12-17
    sc2330_initial();
	
	//chenq mask 2012-12-17
    //trout_ic_rf_spi_ctrl();

    sc2330_rf_tx_filter_cfg();

    sc2330_rf_rx_filter_cfg();

    sc2330_rf_IQ_swap_config(); //in wuranqing, sc2330_rf_initial()

	//chenq mod 2012-12-17
    //sc2330_initial();

	//chenq mask 2012-12-17
    trout_rf_switch(1,1);

    //chenq mask 2013-01-21 for test
	//trout_rf_switch(3,1);
    //trout_rf_switch(4,1);

	
    //sc2330_rf_AGC_config(0x12); // ranqing mask 1103
#endif

#endif
    
    /* Update the MAXIM RF registers for frequency band 5GHz if the current  */
    /* band configured is 5GHz.                                              */
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        update_maxim_ittiam_rf_start_freq_5();
    }

    TROUT_DBG4("Wrote MxFE and RF registers\n");

    /* Switch to the desired channel */
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
    set_channel(freq, channel, sec_ch_offset);
#endif

#ifdef TROUT2_WIFI_FPGA
	#ifdef TROUT2_WIFI_FPGA_RF55
	trout2_mxd_rf_channel_config(channel+1); //caisf temp for debug 20130116
	#endif

	#ifdef TROUT2_WIFI_FPGA_RF2829
    set_channel(freq, channel, sec_ch_offset);
	#endif
#endif

#ifdef TROUT2_WIFI_IC
	MxdRfSetFreqWifiCh(channel+1);
       update_cfg_front_nv();//by lihua
#endif

#ifdef TROUT_WIFI_EVB

#ifdef TROUT_WIFI_EVB_MF_RF55
	trout2_mxd_rf_channel_config(channel+1); //caisf temp for debug 20130116
#else
    trout_ic_set_channel(freq, channel, sec_ch_offset);
#endif

#endif
	// caisf add for sync current channel index, 20130505
    set_rf_channel_idx(channel);

    /* Set the TX power level */
    set_tx_power_rf(tx_power);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_channel                                              */
/*                                                                           */
/*  Description   : This function initializes the Maxim RF with Ittiam PHY.  */
/*                                                                           */
/*  Inputs        : 1) Frequency band                                        */
/*                  2) Channel index                                         */
/*                  3) Secondary channel offset                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function programs the VCO to set the given channel. */
/*                  It finds the required table index corresponding to the   */
/*                  given channel and secondary offset. Using this it gets   */
/*                  the value from the appropriate table based on the        */
/*                  frequency band and secondary offset. It then updates     */
/*                  the required RF registers.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_channel(UWORD8 freq, UWORD8 channel, UWORD8 sec_ch_offset)
{
#ifndef ITTIAM_PHY_EXT_PROG
    vco_prog_info_t *vco_prog_info = 0;
    UWORD8 tbl_idx = 0;

    /* Get the channel table index for the given channel */
    tbl_idx = get_tbl_idx_from_ch_idx(freq, channel);

    if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
    {
        /* If the given channel table index is not valid it is an exception. */
        /* Do nothing and return.                                            */
        return;
    }


    write_dot11_phy_reg(0xFF, 0x00);

    if(freq == RC_START_FREQ_5)
    {

        /* Table for 5 GHz Band: 20 MHz */
        vco_prog_info = g_vco_info_start_freq_5_20;
    }
    else if(freq == RC_START_FREQ_2)
    {
        /* Table for 2.4 GHz Band: 5 MHz */
        vco_prog_info = g_vco_info_start_freq_2_5;
    }

    write_RF_reg(RFMAX_INT_DIV, vco_prog_info[tbl_idx].int_div);
    write_RF_reg(RFMAX_FRAC_DIV, vco_prog_info[tbl_idx].frac_div);

    if(freq == RC_START_FREQ_5)
    {
// 20120709 caisf masked the "ifndef", merged ittiam mac v1.2 code
//#ifndef MAX2830_32
        if(channel > 7)
        {
            write_RF_reg(RFMAX_BSPLL, (RFMAX_BSPLL_VAL_START_FREQ_5 & 0x3F7F));
            write_RF_reg(RFMAX_BSPLL, (RFMAX_BSPLL_VAL_START_FREQ_5 | 0x00C0));
        }
        else
        {
            write_RF_reg(RFMAX_BSPLL, (RFMAX_BSPLL_VAL_START_FREQ_5 & 0x3F7F));
            write_RF_reg(RFMAX_BSPLL, (RFMAX_BSPLL_VAL_START_FREQ_5 | 0x0080));
        }
//#endif /* MAX2830_32 */
    }

	update_phy_ittiam_on_select_channel(freq, channel);
#endif /* ITTIAM_PHY_EXT_PROG */
}

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_EVB
/* dumy add for Trout IC RF 20121026 */
void trout_ic_set_channel(UWORD8 freq, UWORD8 channel, UWORD8 sec_ch_offset)
{
#ifndef ITTIAM_PHY_EXT_PROG
    UWORD8 tbl_idx = 0;

	//TRACE_FUNC_ENTER;
    /* Get the channel table index for the given channel */
    tbl_idx = get_tbl_idx_from_ch_idx(freq, channel);

    if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
    {
        /* If the given channel table index is not valid it is an exception. */
        /* Do nothing and return.                                            */
        return;
    }

    //write_dot11_phy_reg(0xFF, 0x00); //dumy mark 1107

    sc2330_rf_channel_config(channel);

    //update_phy_ittiam_on_select_channel(freq, channel);//dumy mark 1107
#endif /* ITTIAM_PHY_EXT_PROG */
    //TRACE_FUNC_EXIT;
}
#endif
#endif /* RF_MAXIM_ITTIAM */
