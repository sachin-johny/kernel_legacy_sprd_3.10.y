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
/*  File Name         : phy_ittiam.c                                         */
/*                                                                           */
/*  Description       : This file contains functions and parameters for the  */
/*                      Ittiam PHY.                                          */
/*                                                                           */
/*  List of Functions : initialize_phy_ittiam                                */
/*                      set_default_tx_power_levels_ittiam                   */
/*                      set_phy_continuous_tx_mode_ittiam                    */
/*                      read_phy_hw_version_ittiam                           */
/*                      set_phy_tx_power_level_11a_ittiam                    */
/*                      set_phy_tx_power_level_11b_ittiam                    */
/*                      reset_phy_ittiam                                     */
/*                      power_up_phy_ittiam									 */
/*                      power_down_phy_ittiam								 */
/*                      is_signal_weaker_phy_ittiam      					 */
/*                      set_max_phy_sensitivity_ittiam   					 */
/*                      set_normal_phy_sensitivity_ittiam					 */
/*                      set_phy_rx_sensitivity_ittiam    					 */
/*                      get_tssi_11gn_ittiam                                 */
/*                      get_tssi_11b_ittiam              					 */
/*                      update_supp_mcs_bitmap_ittiam						 */
/*                      initialize_phy_2040_ittiam							 */
/*                      post_radar_detect_ittiam                 			 */
/*                      enable_radar_detection_ittiam            			 */
/*                      disable_radar_detection_ittiam           			 */
/*                      reset_ofdmpre_unidentified_bits_ittiam   			 */
/*                      is_ofdm_preamble_detected_ittiam         			 */
/*                      is_unidentified_bit_detected_ittiam      			 */
/*                      update_phy_ittiam_start_freq_5						 */
/*                      get_lock_conv_fact_lsb								 */
/*                      get_curr_agc_val                 					 */
/*                      compute_agc_val                  					 */
/*                      set_phy_agc_val                  					 */
/*                      program_AGC_LUT										 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef ITTIAM_PHY
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "phy_ittiam.h"
#include "rf_if.h"
#include "mh.h"
#include "iconfig.h"
#include "ch_info.h"
#include "spi_interface.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* 802.11h Radar Detection Algorithm Initialization Value                    */
/*****************************************************************************/
#define RADAR_ENABLE_VALUE 0xB7

/*****************************************************************************/
/*                                                                           */
/*                   PHY - Table of register values                          */
/*                                                                           */
/*  This table contains the values to be written in PHY registers IN THE     */
/*  SAME ORDER. Adresses that are not used are initialized to zero           */
/*                                                                           */
/*  NOTE : To add a new phy register                                         */
/*  1. Add the register address definition in phy_ittiam.h prefix with a 'r' */
/*     This list is maintained so that every register can be acessed         */
/*     individually when required                                            */
/*  2. Add value to be written in the register in all the phy_<rf>_ittiam.h  */
/*     header files.                                                         */
/*  3. Add the macro in "phy_reg_val_table" in phy_ittiam.c . NOTE that the  */
/*     MACRO HAS TO BE PLACED AT THE EXACT LOCATION ONLY in this table.      */
/*                                                                           */
/*****************************************************************************/

//#ifdef TROUT2_WIFI_FPGA_RF55
#if  defined(TROUT2_WIFI_FPGA_RF55) || defined(TROUT_WIFI_EVB_MF_RF55) || defined(TROUT2_WIFI_IC)
//for agc
UWORD8 gain_table_BT[28]=
{//... // LNA & TIA & PGA
    0x95,//'10010101', '29C'...
    0x95,//'10010101', '298'...
    0x95,//'10010101', '294'...
    0x95,//'10010101', '290'...
    0x95,//'10010101', '28C'...
    0x95,//'10010101', '288'...
    0x94,//'10010100', '284'...
    0x93, //'10010011', '280'...
    0x93, //'10010011', '27C'...
    0x92, //'10010010', '278'...
    0x92, //'10010010', '274'...
    0x91, //'10010001', '270'...
    0x91, //'10010001', '26C'...
    0x91, //'10001001', '268'...
    0x90, //'10001000', '264'...
    0x90, //'10010000', '260'...
    0x50, //'01010000', '25C'...
    0x50, //'01010000', '258'...
    0x48, //'01001000', '254'...
    0x40, //'01000000', '250'...
    0x30, //'00110000', '24C'...
    0x28, //'00101000', '248'...
    0x12, //'00010010', '244'...
    0x20, //'00100000', '240'...
    0x09,//'00001001', '23C'...
    0x02,//'00000010', '238'...
    0x01,//'00000001', '234'...
    0x01//'00000001', '230'...
};


//for wifi agc
UWORD16 gain_table_WIFI[32] ={// LNA & TIA & PGA

    0x012a,//'100101010','3aC'...
    0x012a,//'100101010','3a8'...
    0x012a,//'100101010','3a4'...
    0x012a,//'100101010','3a0'...
    0x012a, //'100101010','39C'...
    0x012a, //'100101010','398'...
    0x012a, //'100101010','394'...
    0x012a, //'100101010','390'...
    0x012a, //'100101010','38C'...
    0x0129, //'100101001','388'...
    0x0127, //'100100111','384'...
    0x0126, //'100100110','380'...
    0x0125, //'100100101','37C'...
    0x0124, //'100100100','378'...
    0x0123, //'100100011','374'...
    0x0122, //'100100010','370'...
    0x0121, //'100100001','36C'...
    0x0120, //'100100000','368'...
    0x0111, //'100010001','364'...
    0x0110, //'100010000','360'...
    0x00a0, //'010100000','35C'...
    0x0091, //'010010001','358'...
    0x0090, //'010010000','354'...
    0x0081, //'010000001','350'...
    0x0051, //'001010001','34C'...
    0x0050, //'001010000','348'...
    0x0041, //'001000001','344'...
    0x0022, //'000100010','340'...
    0x0021, //'000100001','33C'...
    0x0020, //'000100000','338'...
    0x0003, //'000000011','334'...
    0x0010 //'000010000','330'...
    };

UWORD32 bt_freq[] = {
				2400,0x171,0x3b3b, 0x5c5,
				2401,0x171,0x6262, 0x5c6,
				2402,0x171,0x8989, 0x5c6,
				2403,0x171,0xb1b1, 0x5c7,
				2404,0x171,0xd8d8, 0x5c7,
				2405,0x172,0x0, 0x5c8,
				2406,0x172,0x2727, 0x5c9,
				2407,0x172,0x4e4e, 0x5c9,
				2408,0x172,0x7676, 0x5ca,
				2409,0x172,0x9d9d, 0x5ca,
				2410,0x172,0xc4c4, 0x5cb,
				2411,0x172,0xecec, 0x5cc,
				2412,0x173,0x1313, 0x5cc,
				2413,0x173,0x3b3b, 0x5cd,
				2414,0x173,0x6262, 0x5ce,
				2415,0x173,0x8989, 0x5ce,
				2416,0x173,0xb1b1, 0x5cf,
				2417,0x173,0xd8d8, 0x5cf,
				2418,0x174,0x0, 0x5d0,
				2419,0x174,0x2727, 0x5d1,
				2420,0x174,0x4e4e, 0x5d1,
				2421,0x174,0x7676, 0x5d2,
				2422,0x174,0x9d9d, 0x5d2,
				2423,0x174,0xc4c4, 0x5d3,
				2424,0x174,0xecec, 0x5d4,
				2425,0x175,0x1313, 0x5d4,
				2426,0x175,0x3b3b, 0x5d5,
				2427,0x175,0x6262, 0x5d6,
				2428,0x175,0x8989, 0x5d6,
				2429,0x175,0xb1b1, 0x5d7,
				2430,0x175,0xd8d8, 0x5d7,
				2431,0x176,0x0, 0x5d8,
				2432,0x176,0x2727, 0x5d9,
				2433,0x176,0x4e4e, 0x5d9,
				2434,0x176,0x7676, 0x5da,
				2435,0x176,0x9d9d, 0x5da,
				2436,0x176,0xc4c4, 0x5db,
				2437,0x176,0xecec, 0x5dc,
				2438,0x177,0x1313, 0x5dc,
				2439,0x177,0x3b3b, 0x5dd,
				2440,0x177,0x6262, 0x5de,
				2441,0x177,0x8989, 0x5de,
				2442,0x177,0xb1b1, 0x5df,
				2443,0x177,0xd8d8, 0x5df,
				2444,0x178,0x0, 0x5e0,
				2445,0x178,0x2727, 0x5e1,
				2446,0x178,0x4e4e, 0x5e1,
				2447,0x178,0x7676, 0x5e2,
				2448,0x178,0x9d9d, 0x5e2,
				2449,0x178,0xc4c4, 0x5e3,
				2450,0x178,0xecec, 0x5e4,
				2451,0x179,0x1313, 0x5e4,
				2452,0x179,0x3b3b, 0x5e5,
				2453,0x179,0x6262, 0x5e6,
				2454,0x179,0x8989, 0x5e6,
				2455,0x179,0xb1b1, 0x5e7,
				2456,0x179,0xd8d8, 0x5e7,
				2457,0x17a,0x0, 0x5e8,
				2458,0x17a,0x2727, 0x5e9,
				2459,0x17a,0x4e4e, 0x5e9,
				2460,0x17a,0x7676, 0x5ea,
				2461,0x17a,0x9d9d, 0x5ea,
				2462,0x17a,0xc4c4, 0x5eb,
				2463,0x17a,0xecec, 0x5ec,
				2464,0x17b,0x1313, 0x5ec,
				2465,0x17b,0x3b3b, 0x5ed,
				2466,0x17b,0x6262, 0x5ee,
				2467,0x17b,0x8989, 0x5ee,
				2468,0x17b,0xb1b1, 0x5ef,
				2469,0x17b,0xd8d8, 0x5ef,
				2470,0x17c,0x0, 0x5f0,
				2471,0x17c,0x2727, 0x5f1,
				2472,0x17c,0x4e4e, 0x5f1,
				2473,0x17c,0x7676, 0x5f2,
				2474,0x17c,0x9d9d, 0x5f2,
				2475,0x17c,0xc4c4, 0x5f3,
				2476,0x17c,0xecec, 0x5f4,
				2477,0x17d,0x1313, 0x5f4,
				2478,0x17d,0x3b3b, 0x5f5,
				2479,0x17d,0x6262, 0x5f6,
				2480,0x17d,0x8989, 0x5f6,
				2481,0x17d,0xb1b1, 0x5f7,
				2482,0x17d,0xd8d8, 0x5f7,
				2483,0x17e,0x0, 0x5f8
			};  

UWORD32 gFreqBtAfcBypass[] = {
				2400,0x171,0x3b3b, 0x5c5,
				2401,0x171,0x6262, 0x5c6,
				2402,0x171,0x8989, 0x5c6,
				2403,0x171,0xb1b1, 0x5c7,
				2404,0x171,0xd8d8, 0x5c7,
				2405,0x172,0x0, 0x5c8,
				2406,0x172,0x2727, 0x5c9,
				2407,0x172,0x4e4e, 0x5c9,
				2408,0x172,0x7676, 0x5ca,
				2409,0x172,0x9d9d, 0x5ca,
				2410,0x172,0xc4c4, 0x5cb,
				2411,0x172,0xecec, 0x5cc,
				2412,0x173,0x1313, 0x5cc,
				2413,0x173,0x3b3b, 0x5cd,
				2414,0x173,0x6262, 0x5ce,
				2415,0x173,0x8989, 0x5ce,
				2416,0x173,0xb1b1, 0x5cf,
				2417,0x173,0xd8d8, 0x5cf,
				2418,0x174,0x0, 0x5d0,
				2419,0x174,0x2727, 0x5d1,
				2420,0x174,0x4e4e, 0x5d1,
				2421,0x174,0x7676, 0x5d2,
				2422,0x174,0x9d9d, 0x5d2,
				2423,0x174,0xc4c4, 0x5d3,
				2424,0x174,0xecec, 0x5d4,
				2425,0x175,0x1313, 0x5d4,
				2426,0x175,0x3b3b, 0x5d5,
				2427,0x175,0x6262, 0x5d6,
				2428,0x175,0x8989, 0x5d6,
				2429,0x175,0xb1b1, 0x5d7,
				2430,0x175,0xd8d8, 0x5d7,
				2431,0x176,0x0, 0x5d8,
				2432,0x176,0x2727, 0x5d9,
				2433,0x176,0x4e4e, 0x5d9,
				2434,0x176,0x7676, 0x5da,
				2435,0x176,0x9d9d, 0x5da,
				2436,0x176,0xc4c4, 0x5db,
				2437,0x176,0xecec, 0x5dc,
				2438,0x177,0x1313, 0x5dc,
				2439,0x177,0x3b3b, 0x5dd,
				2440,0x177,0x6262, 0x5de,
				2441,0x177,0x8989, 0x5de,
				2442,0x177,0xb1b1, 0x5df,
				2443,0x177,0xd8d8, 0x5df,
				2444,0x178,0x0, 0x5e0,
				2445,0x178,0x2727, 0x5e1,
				2446,0x178,0x4e4e, 0x5e1,
				2447,0x178,0x7676, 0x5e2,
				2448,0x178,0x9d9d, 0x5e2,
				2449,0x178,0xc4c4, 0x5e3,
				2450,0x178,0xecec, 0x5e4,
				2451,0x179,0x1313, 0x5e4,
				2452,0x179,0x3b3b, 0x5e5,
				2453,0x179,0x6262, 0x5e6,
				2454,0x179,0x8989, 0x5e6,
				2455,0x179,0xb1b1, 0x5e7,
				2456,0x179,0xd8d8, 0x5e7,
				2457,0x17a,0x0, 0x5e8,
				2458,0x17a,0x2727, 0x5e9,
				2459,0x17a,0x4e4e, 0x5e9,
				2460,0x17a,0x7676, 0x5ea,
				2461,0x17a,0x9d9d, 0x5ea,
				2462,0x17a,0xc4c4, 0x5eb,
				2463,0x17a,0xecec, 0x5ec,
				2464,0x17b,0x1313, 0x5ec,
				2465,0x17b,0x3b3b, 0x5ed,
				2466,0x17b,0x6262, 0x5ee,
				2467,0x17b,0x8989, 0x5ee,
				2468,0x17b,0xb1b1, 0x5ef,
				2469,0x17b,0xd8d8, 0x5ef,
				2470,0x17c,0x0, 0x5f0,
				2471,0x17c,0x2727, 0x5f1,
				2472,0x17c,0x4e4e, 0x5f1,
				2473,0x17c,0x7676, 0x5f2,
				2474,0x17c,0x9d9d, 0x5f2,
				2475,0x17c,0xc4c4, 0x5f3,
				2476,0x17c,0xecec, 0x5f4,
				2477,0x17d,0x1313, 0x5f4,
				2478,0x17d,0x3b3b, 0x5f5,
				2479,0x17d,0x6262, 0x5f6,
				2480,0x17d,0x8989, 0x5f6,
				2481,0x17d,0xb1b1, 0x5f7,
				2482,0x17d,0xd8d8, 0x5f7,
				2483,0x17e,0x0, 0x5f8
			};  


void show_rf55_reg_val(void) //dumy add 0130 for RF55 debug
{
	UWORD32 temp_reg_data = 0;
	#ifdef TROUT2_WIFI_IC
	UWORD32 rf_reg_base_addr =  0x1000;
	#else
	UWORD32 rf_reg_base_addr = 0;
	#endif
	UWORD32 cnt;

	for(cnt=0;cnt<0x200;cnt++)
	{
		#ifdef TROUT2_WIFI_IC
		temp_reg_data = host_read_trout_reg((rf_reg_base_addr+cnt)<<2);
		#else
		rf_reg_read((rf_reg_base_addr+cnt),&temp_reg_data);
		#endif
		printk("reg_addr=%x,reg_data=%x.\n",(rf_reg_base_addr+cnt),temp_reg_data);
	}
}
#endif


UWORD32 phy_reg_val_table[MAX_PHY_ADDR + 1] = {
    /* PHY ADDRESS  INITIALIZATION VALUE */
    /* 0x00 */      PHYIMPLEMENTATIONREG,
    /* 0x01 */      PHYMSBVERNUMREG,
    /* 0x02 */      PHYLSBVERNUMREG,
    /* 0x03 */      PHY_ADDR_NOT_USED,
    /* 0x04 */      PHYCNTRLREG,
    /* 0x05 */      PHY_ADDR_NOT_USED,
    /* 0x06 */      PHY_ADDR_NOT_USED,
    /* 0x07 */      PHY_ADDR_NOT_USED,
#ifdef TROUT2_WIFI_IC
	/* 0x08 */      0x02, // wuranqing
#else
    /* 0x08 */      0x12,// wuranqing //PHYRFCNTRL_START_FREQ_2,
#endif	
    /* 0x09 */      PHYTXPWUPTIME1,
    /* 0x0A */      PHYTXPWUPTIME2,
    /* 0x0B */      PHYTXPWUPTIME3,
    /* 0x0C */      PHYTXPWUPTIME4,
    /* 0x0D */      PHYTXPWUPTIME5,
    /* 0x0E */      PHYRXPWUPTIME1,
    /* 0x0F */      PHYRXPWUPTIME2,
    /* 0x10 */      PHYRXPWUPTIME3,
    /* 0x11 */      PHYRXPWUPTIME4,
    /* 0x12 */      PHYRXPWUPTIME5,
    /* 0x13 */      PHYTXPWDNTIME1,
    /* 0x14 */      PHYTXPWDNTIME2,
    /* 0x15 */      PHYTXPWDNTIME3,
    /* 0x16 */      PHYTXPWDNTIME4,
    /* 0x17 */      PHYTXPWDNTIME5,
    /* 0x18 */      PHYRXPWDNTIME1,
    /* 0x19 */      PHYRXPWDNTIME2,
    /* 0x1A */      PHYRXPWDNTIME3,
    /* 0x1B */      PHYRXPWDNTIME4,
    /* 0x1C */      PHYRXPWDNTIME5,
    #if 0
    /* 0x1D */      0x0D,//wuranqing //PHYTXPOLCNTRL,
    #else
    /* 0x1D */      PHYTXPOLCNTRL,
    #endif
    /* 0x1E */      PHYRXPOLCNTRL,
    /* 0x1F */      PHYAGCTIMEOUT,
    /* 0x20 */      PHYAGCCNTRL,
    /* 0x21 */      PHYAGCADDTNGAIN,
    /* 0x22 */      PHYAGCINITWAITPKTSETTIME,
    /* 0x23 */      PHYAGCMAXVGAGAIN_START_FREQ_2,
    /* 0x24 */      PHYAGCLNAGAIN,
    /* 0x25 */      PHYAGCPKTDETCNTRL1,
    /* 0x26 */      PHYAGCPKTDETCNTRL2,
    /* 0x27 */      PHYAGCVGAGAINAFTERPKTDET,
    /* 0x28 */      PHYAGCLNAHITOLOVGACOMP_START_FREQ_2,
    /* 0x29 */      PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2,
    /* 0x2A */      PHYAGCCORS1UPWINCTRL,
    /* 0x2B */      PHYAGCCORS1LOWINCTRL,
    /* 0x2C */      PHYAGCCORS1LNAHITOLOCTRL,
    /* 0x2D */      PHYAGCCORS1LNAHITOMIDCTRL,
    /* 0x2E */      PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2,
    /* 0x2F */      PHYAGCCORS1LOWINVGAGAIN,
    /* 0x30 */      PHYAGCCORS1SETTIMECTRL1,
    /* 0x31 */      PHYAGCCORS1SETTIMECTRL2,
    /* 0x32 */      PHYAGCCORS1NXTSTATECTRL,
    /* 0x33 */      PHYAGCCORSNXTSTATECTRL,
    /* 0x34 */      PHYAGCCORSLNAGAINCTRL,
    /* 0x35 */      PHYAGCCORS2UPWINCTRL_START_FREQ_2,
    /* 0x36 */      PHYAGCCORS2LOWINCTRL,
    /* 0x37 */      PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2,
    /* 0x38 */      PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2,
    /* 0x39 */      PHYAGCCORS2UPWINVGAGAIN_START_FREQ_2,
    /* 0x3A */      PHYAGCCORS2LOWINVGAGAIN,
    /* 0x3B */      PHYAGCCORS2SETTIMECTRL1,
    /* 0x3C */      PHYAGCCORS2SETTIMECTRL2,
    /* 0x3D */      PHYAGCCORS2NXTSTATECTRL,
    /* 0x3E */      PHYAGCCORSFINEAVGWINCTRL1,
    /* 0x3F */      PHYAGCCORSFINEAVGWINCTRL2,
    /* 0x40 */      PHYAGCFINEUPWINCTRL,
    /* 0x41 */      PHYAGCFINE1CTRL,
    /* 0x42 */      PHYAGCFINE2CTRL,
    /* 0x43 */      PHYAGCFINE3CTRL,
    /* 0x44 */      PHYAGCFINALVGAGAINTH_START_FREQ_2,
    /* 0x45 */      PHYAGCULCTRL1,
    /* 0x46 */      PHYAGCULCTRL2,
    /* 0x47 */      PHYAGCULUPTH,
    /* 0x48 */      PHYAGCULLOTH,
    /* 0x49 */      PHYAGCULSETTIME,
    /* 0x4A */      PHYAGCMAXANDFINETOCORSTIME,
    /* 0x4B */      PHYAGCVGAGAINBACKOFF,
    /* 0x4C */      PHYAGCABSULUPTH,
    /* 0x4D */      PHYAGCABSULLOTH,
    /* 0x4E */      PHYTMAGCGAIN,
    /* 0x4F */      PHYTMAGCGAINSETTIME,
    /* 0x50 */      PHYCCACONTROL,
    /* 0x51 */      PHYCCACONSTVGAMC1,
    /* 0x52 */      PHYCCACONSTVGAMC2,
    /* 0x53 */      PHYCCACONSTVGAMC3,
    /* 0x54 */      PHYCCAMACTH1_START_FREQ_2,
    /* 0x55 */      PHYCCAMACTH2_START_FREQ_2,
    /* 0x56 */      PHYCCAPHYTH,
    /* 0x57 */      PHYCCAINSERTIONVAL_START_FREQ_2,
    /* 0x58 */      PHYCCAERMARGIN1,
    /* 0x59 */      PHYCCAERMARGIN2,
    /* 0x5A */      PHYCCASTATUS,
    /* 0x5B */      PHYUNLOCKCKECKCNTR,
    /* 0x5C */      PHYRXSCALEFACTMSB,
    /* 0x5D */      PHYRXSCALEFACTLSB,
    /* 0x5E */      PHYPOWDNCNTRL,
    /* 0x5F */      AGCSPRAMLSB,
    /* 0x60 */      AGCSPRAMMSB,
    /* 0x61 */      ERSSI_EN,
    /* 0x62 */      ERSSI_LTH,
    /* 0x63 */      ERSSI_HTH,
    /* 0x64 */      ERSSI_AGC_EN,
    /* 0x65 */      ERSSI_PKT_DET_TIM_CTRL1,
    /* 0x66 */      ERSSI_SAT_NXT_ST_LNA,
    /* 0x67 */      ERSSI_SAT_TH,
    /* 0x68 */      ERSSI_PKT_DET_TIM_CTRL2,
    /* 0x69 */      ERSSI_MC1,
    /* 0x6A */      ERSSI_MC2,
    /* 0x6B */      ERSSI_MC3,
    /* 0x6C */      ERSSI_OFSET_ML1,
    /* 0x6D */      ERSSI_OFSET_ML2,
    /* 0x6E */      ERSSI_OFSET_ML3,
    /* 0x6F */      ERSSI_PKT_DET_ITR_2_CTRL,
    /* 0x70 */      TXPOWER11A,
    /* 0x71 */      TXPOWER11B,
    /* 0x72 */      TXDCOI,
    /* 0x73 */      TXDCOQ,
    /* 0x74 */      TXPOSTFILTCOEFFLSB,
    /* 0x75 */      TXPOSTFILTCOEFFMSB,
    /* 0x76 */      RXHPSWITCHDLY,
    /* 0x77 */      RXDIGHPFCNTRL,
    /* 0x78 */      RSSIMSB,
    /* 0x79 */      RSSILSB,
    #if 0
    /* 0x7A */      0x01,//wuranqing //TXPOWCALCNTRL,
    #else
	/* 0x7A */      TXPOWCALCNTRL,
	#endif
    /* 0x7B */      TXPOWCALDLY,
    /* 0x7C */      ADCDACTSTMODE,
    /* 0x7D */      POWDET11A,
    /* 0x7E */      POWDET11B,
    /* 0x7F */      TXPOWERCALAVGWIN,
    /* 0x80 */      PHYBTXCNTRL,
    /* 0x81 */      PHYBTXLENH,
    /* 0x82 */      PHYBTXLENL,
    /* 0x83 */      PHYBRXCNTRL,
    /* 0x84 */      PHYBRXSTA,
    /* 0x85 */      PHYBRXLENH,
    /* 0x86 */      PHYBRXLENL,
    /* 0x87 */      PHYBACQTHRESH,
    /* 0x88 */      PHYBACQTIMEOUT,
    /* 0x89 */      PHYBSNRUPTHRESHRSW,
    /* 0x8A */      PHYBSNRLOTHRESHRSW,
    /* 0x8B */      PHYBMPRTHRESHRSW,
    /* 0x8C */      PHYBTSTMODCTRL1,
    /* 0x8D */      PHYBTSTMODCTRL2,
    /* 0x8E */      PHYBRXPLCPDLY,
    /* 0x8F */      PHYCSBARKERTH1REG1,
    /* 0x90 */      PHYCSBARKERTH1REG2,
    /* 0x91 */      PHYCSBARKERTH1REG3,
    /* 0x92 */      PHYCSBARKERTH1REG4,
    /* 0x93 */      PHYCSBARKERTH1REG5,
    /* 0x94 */      PHYCSBARKERTH1REG6,
    /* 0x95 */      PHYCSBARKERTH2REG1,
    /* 0x96 */      PHYCSBARKERTH2REG2,
    /* 0x97 */      PHYCSBARKERTH2REG3,
    /* 0x98 */      PHYCSBARKERTH2REG4,
    /* 0x99 */      PHYCSBARKERTH2REG5,
    /* 0x9A */      PHYCSBARKERTH2REG6,
    /* 0x9B */      PHYCSBARKERTH2REG7,
    /* 0x9C */      PHYRAKEESTDELAY,
    /* 0x9D */      PHYBHDRPLCPDLY,
    /* 0x9E */      PHYBANTIPHASEHANGUP,
    /* 0x9F */      PHYBDIGHPFCOEFFLSB,
    /* 0xA0 */      PHYBTXSCALEFACT,
    /* 0xA1 */      DSSSTXPREFILTCOEFFLSB,
    /* 0xA2 */      DSSSTXPREFILTCOEFFMSB,
    /* 0xA3 */      ANTDIVERSTYCTRL,
    /* 0xA4 */      ANTSWITCHDUR,
    /* 0xA5 */      ANTIDLESTATEWAIT,
    /* 0xA6 */      PHY11BMODECTRL,
    /* 0xA7 */      ADRSSICOMPFACTOR,
    /* 0xA8 */      RAKESWTMULTIPATHCNT,
    /* 0xA9 */      PHY_ADDR_NOT_USED,
    /* 0xAA */      PHY_ADDR_NOT_USED,
    /* 0xAB */      PHY_ADDR_NOT_USED,
    /* 0xAC */      PHY_ADDR_NOT_USED,
    /* 0xAD */      PHY_ADDR_NOT_USED,
    /* 0xAE */      PHY_ADDR_NOT_USED,
    /* 0xAF */      PHY_ADDR_NOT_USED,
    /* 0xB0 */      PHYRFCNTRL3,
    /* 0xB1 */      AGCTRIGSTATLSB,
    /* 0xB2 */      AGCTRIGSTATMSB,
    /* 0xB3 */      AGCSETTLEDVGAGAIN,
    /* 0xB4 */      AGCSETTLEDLNAGAIN,
    /* 0xB5 */      RXDCOI,
    /* 0xB6 */      RXDCOQ,
    /* 0xB7 */      TXIQIMBREG1,
    /* 0xB8 */      TXIQIMBREG2,
    /* 0xB9 */      RXIQIMBREG1,
    /* 0xBA */      RXIQIMBREG2,
    /* 0xBB */      RXIQIMBREG3,
    /* 0xBC */      SPIDATA,
    /* 0xBD */      SPICTRL,
    /* 0xBE */      MACIFCTRL,
    /* 0xBF */      PHY_ADDR_NOT_USED,
    /* 0xC0 */      PHYATXCON,
    /* 0xC1 */      PHYATXLENH,
    /* 0xC2 */      PHYATXLENL,
    /* 0xC3 */      PHYATXSERFLD,
    /* 0xC4 */      PHYATXSAMPLESCALEFACTOR,
    /* 0xC5 */      PHYARXCTL,
    /* 0xC6 */      PHYARXSTS,
    /* 0xC7 */      PHYARXSERFLD,
    /* 0xC8 */      PHYARXLENH,
    /* 0xC9 */      PHYARXLENL,
    /* 0xCA */      PHYAPKTDETTHRESH,
    /* 0xCB */      PHYAPKTDETCNT1,
    /* 0xCC */      PHYAPKTDETCNT2,
    /* 0xCD */      PHYALPDETTH1,
    /* 0xCE */      PHYALPDETTH2,
    /* 0xCF */      PHYALPDETCNT,
    /* 0xD0 */      PHYASNRTH1,
    /* 0xD1 */      PHYASNRTH2,
    /* 0xD2 */      PHYAUNLOCKTIMECORRCTRL,
    /* 0xD3 */      PHYATSTMODCTRL1,
    /* 0xD4 */      PHYATSTMODCTRL2,
    /* 0xD5 */      PHYATSTMODCTRL3,
    /* 0xD6 */      PHYARXPLCPDLY,
    /* 0xD7 */      PHYALOCKCONVFACTLSB,
    /* 0xD8 */      PHYALOCKCONVFACTMSB,
    /* 0xD9 */      PHYAMAXRXLENGTHMSB,
    /* 0xDA */      PHYAMAXRXLENGTHLSB,
    /* 0xDB */      UNLOCKSAMPREG1,
    /* 0xDC */      UNLOCKSAMPREG2,
    /* 0xDD */      UNLOCKSAMPREG3,
    /* 0xDE */      PHY_ADDR_NOT_USED,
    /* 0xDF */      PHY_ADDR_NOT_USED,
    /* 0xE0 */      CFEBIASPOS,
    /* 0xE1 */      CFEBIASNEG,
    /* 0xE2 */      FASTCSWIN,
    /* 0xE3 */      PHYATXSCALEFACT,
    /* 0xE4 */      PHYADIGHPFCOEFFLSB,
    /* 0xE5 */      PHYADIGHPFCOEFFMSB,
    /* 0xE6 */      PHYRADARCNTRLREG1,
    /* 0xE7 */      PHYRADARCNTRLREG2,
    /* 0xE8 */      PHYRADARCNTRLREG3,
    /* 0xE9 */      PHYRADARCNTRLREG4,
    /* 0xEA */      PHYRADARCNTRLREG5,
    /* 0xEB */      PHYRADARCNTRLREG6,
    /* 0xEC */      PHYRADARCNTRLREG7,
    /* 0xED */      PHYRADARCNTRLREG8,
    /* 0xEE */      PHYRADARCNTRLREG9,
    /* 0xEF */      PHYRADARCNTRLREG10,
    /* 0xF0 */      RXCNTRLREG11N
};

UWORD32 phy_reg_val_table_2[MAX_PHY_ADDR2+1] = {
/*  PHY ADDR        VALUE to be initialised */
    /* 0x00 */      AGCLNAMIDLOWCTRL1,
    /* 0x01 */      AGCLNAMIDLOWCTRL2,
    /* 0x02 */      AGCLNAMIDLOWCTRL3,
    /* 0x03 */      AGCLNAMIDLOWAVGWINCTRL1,
    /* 0x04 */      AGCLNAMIDLOWAVGWINCTRL2,
    /* 0x05 */      AGCMIDCOARSE1UPPERWINCTRL,
    /* 0x06 */      AGCMIDCOARSE1LOWERWINCTRL,
    /* 0x07 */      AGCMIDCOARSE1LNAMIDTOLOWCTRL,
    /* 0x08 */      AGCMIDCOARSE1UPPERWINVGAGAIN,
    /* 0x09 */      AGCMIDCOARSE1LOWERWINVGAGAIN,
    /* 0x0A */      AGCMIDCOARSELNAGAINCTRL,
    /* 0x0B */      AGCMIDCOARSE1SETTLETIMECTRL1,
    /* 0x0C */      AGCMIDCOARSE1SETTLETIMECTRL2,
    /* 0x0D */      AGCMIDCOARSE1NXTSTATECTRL,
    /* 0x0E */      AGCMIDCOARSENXTSTATECTRL,
    /* 0x0F */      AGCMIDCOARSE2UPPERWINCTRL,
    /* 0x10 */      AGCMIDCOARSE2LOWERWINCTRL,
    /* 0x11 */      AGCMIDCOARSE2LNAMIDTOLOWCTRL,
    /* 0x12 */      AGCMIDCOARSE2UPPERWINVGAGAIN,
    /* 0x13 */      AGCMIDCOARSE2LOWERWINVGAGAIN,
    /* 0x14 */      AGCMIDCOARSE2SETTLETIMECTRL1,
    /* 0x15 */      AGCMIDCOARSE2SETTLETIMECTRL2,
    /* 0x16 */      AGCMIDCOARSE2NXTSTATECTRL,
    /* 0x17 */      AGCLOWFINE1CTRL,
    /* 0x18 */      AGCLOWFINE2CTRL,
    /* 0x19 */      AGCLOWFINE3CTRL,
    /* 0x1A */      AGCRELOCKUPMARGIN,
    /* 0x1B */      AGCRELOCKLOWMARGIN,
    /* 0x1C */      AGCRELOCKCHECKTIME,
    /* 0x1D */      TXREADYDELAY,
    /* 0x1E */      RXREADY11ADELAY,
    /* 0x1F */      RXREADY11BDELAY,
    /* 0x20 */      SWRESETCTRL,
    //#ifdef TROUT2_WIFI_FPGA_RF55
	#if  defined(TROUT2_WIFI_FPGA_RF55) || defined(TROUT_WIFI_EVB_MF_RF55) || defined(TROUT2_WIFI_IC)
    /* 0x21 */      TX_UPFILTER_SCALER,
    #else
	/* 0x21 */		PHY_ADDR_NOT_USED,
	#endif
    /* 0x22 */      PHY_ADDR_NOT_USED,
    /* 0x23 */      PHY_ADDR_NOT_USED,
    /* 0x24 */      PHY_ADDR_NOT_USED,
    /* 0x25 */      PHY_ADDR_NOT_USED,
    /* 0x26 */      PHY_ADDR_NOT_USED,
    /* 0x27 */      PHY_ADDR_NOT_USED,
    /* 0x28 */      PHY_ADDR_NOT_USED,
    /* 0x29 */      PHY_ADDR_NOT_USED,
    /* 0x2A */      PHY_ADDR_NOT_USED,
    /* 0x2B */      PHY_ADDR_NOT_USED,
    /* 0x2C */      PHY_ADDR_NOT_USED,
    /* 0x2D */      PHY_ADDR_NOT_USED,
    /* 0x2E */      PHY_ADDR_NOT_USED,
    /* 0x2F */      PHY_ADDR_NOT_USED,
    /* 0x30 */      PHY_ADDR_NOT_USED,
    /* 0x31 */      PHY_ADDR_NOT_USED,
    /* 0x32 */      PHY_ADDR_NOT_USED,
    /* 0x33 */      PHY_ADDR_NOT_USED,
#ifdef TROUT2_WIFI_FPGA
		/* 0x34 */		VGA_GAIN_SHIFT_CTRL_LSB_1,
		/* 0x35 */		VGA_GAIN_SHIFT_CTRL_LSB_2,
		/* 0x36 */		VGA_GAIN_SHIFT_CTRL_LSB_3,
		/* 0x37 */		VGA_GAIN_SHIFT_CTRL_LSB_4,
		/* 0x38 */		VGA_GAIN_SHIFT_CTRL_LSB_5,
		/* 0x39 */		VGA_GAIN_SHIFT_CTRL_LSB_6,
		/* 0x3A */		VGA_GAIN_SHIFT_CTRL_LSB_7,
		/* 0x3B */		VGA_GAIN_SHIFT_CTRL_MSB,
#endif

#ifdef TROUT2_WIFI_IC
		/* 0x34 */		VGA_GAIN_SHIFT_CTRL_LSB_1,
		/* 0x35 */		VGA_GAIN_SHIFT_CTRL_LSB_2,
		/* 0x36 */		VGA_GAIN_SHIFT_CTRL_LSB_3,
		/* 0x37 */		VGA_GAIN_SHIFT_CTRL_LSB_4,
		/* 0x38 */		VGA_GAIN_SHIFT_CTRL_LSB_5,
		/* 0x39 */		VGA_GAIN_SHIFT_CTRL_LSB_6,
		/* 0x3A */		VGA_GAIN_SHIFT_CTRL_LSB_7,
		/* 0x3B */		VGA_GAIN_SHIFT_CTRL_MSB,
#else
		/* 0x34 */		PHY_ADDR_NOT_USED,
		/* 0x35 */		PHY_ADDR_NOT_USED,
		/* 0x36 */		PHY_ADDR_NOT_USED,
		/* 0x37 */		PHY_ADDR_NOT_USED,
		/* 0x38 */		PHY_ADDR_NOT_USED,
		/* 0x39 */		PHY_ADDR_NOT_USED,
		/* 0x3A */		PHY_ADDR_NOT_USED,
		/* 0x3B */		PHY_ADDR_NOT_USED,
#endif

    /* 0x3C */      PHY_ADDR_NOT_USED,
    /* 0x3D */      PHY_ADDR_NOT_USED,
    /* 0x3E */      PHY_ADDR_NOT_USED,
    /* 0x3F */      PHY_ADDR_NOT_USED,
    /* 0x40 */      PREFFTDELAY,
    /* 0x41 */      FFTDELAY
};

UWORD32 phy_reg_addr_start_freq_5_table[NUM_PHY_ADDR_START_FREQ_5] = {
    /* PHY ADDRESS */
    /* 0x08 */ rPHYRFCNTRL,
    /* 0x23 */ rPHYAGCMAXVGAGAIN,
    /* 0x28 */ rPHYAGCLNAHITOLOVGACOMP,
    /* 0x29 */ rPHYAGCLNAHITOMIDVGACOMP,
    /* 0x2E */ rPHYAGCCORS1UPWINVGAGAIN,
    /* 0x35 */ rPHYAGCCORS2UPWINCTRL,
    /* 0x37 */ rPHYAGCCORS2LNAHITOLOCTRL,
    /* 0x38 */ rPHYAGCCORS2LNAHITOMIDCTRL,
    /* 0x39 */ rPHYAGCCORS2UPWINVGAGAIN,
    /* 0x44 */ rPHYAGCFINALVGAGAINTH,
    /* 0x54 */ rPHYCCAMACTH1,
    /* 0x55 */ rPHYCCAMACTH2,
    /* 0x57 */ rPHYCCAINSERTIONVAL
};

UWORD32 phy_reg_val_start_freq_5_table[NUM_PHY_ADDR_START_FREQ_5] = {
    /* INITIALIZATION VALUE */
    /* 0x08 */ PHYRFCNTRL_START_FREQ_5,
    /* 0x23 */ PHYAGCMAXVGAGAIN_START_FREQ_5,
    /* 0x28 */ PHYAGCLNAHITOLOVGACOMP_START_FREQ_5,
    /* 0x29 */ PHYAGCLNAHITOMIDVGACOMP_START_FREQ_5,
    /* 0x2E */ PHYAGCCORS1UPWINVGAGAIN_START_FREQ_5,
    /* 0x35 */ PHYAGCCORS2UPWINCTRL_START_FREQ_5,
    /* 0x37 */ PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_5,
    /* 0x38 */ PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_5,
    /* 0x39 */ PHYAGCCORS2UPWINVGAGAIN_START_FREQ_5,
    /* 0x44 */ PHYAGCFINALVGAGAINTH_START_FREQ_5,
    /* 0x54 */ PHYCCAMACTH1_START_FREQ_5,
    /* 0x55 */ PHYCCAMACTH2_START_FREQ_5,
    /* 0x57 */ PHYCCAINSERTIONVAL_START_FREQ_5,
};

/*****************************************************************************/
/*                                                                           */
/*                        LOOK UP TABLE for PHY AGC                          */
/*                                                                           */
/* Digital RSSI LUT (LSB 8-bits) remains same across RFs and platforms       */
/* Analog RSSI LUT (MSB 8-bits) changes with RFs and platforms               */
/*                                                                           */
/*****************************************************************************/

#ifdef MAX2829

#ifdef MXFE_22MHZ

// 13dB back-off
UWORD16 agc_lut[256] = {

0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0200, 0x0200, 0x0200, 0x0200, 0x0200,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};
#elif defined(MXFE_44MHZ)  //modified by Hugh

UWORD16 agc_lut[256] = {

0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0200, 0x0200, 0x0200, 0x0200, 0x0200,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};

#else /* MXFE_44MHZ */

UWORD16 agc_lut[256] = {

//chenq add & mod 2012-12-20
#ifdef TROUT_WIFI_FPGA
0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#endif

#ifdef TROUT2_WIFI_FPGA
0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#endif

#ifdef TROUT2_WIFI_IC
#if 0
0x1E04, 0x1E02, 0x1E01, 0x1E01, 0x1E00, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFE, 0x1BFE, 0x1BFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x19FE,
0x19FE, 0x19FE, 0x19FD, 0x19FD, 0x19FD, 0x18FD, 0x18FD, 0x18FD, 0x18FD, 0x18FD,
0x18FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x16FD, 0x16FD, 0x16FD,
0x16FD, 0x16FD, 0x16FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x14FD,
0x14FD, 0x14FD, 0x14FD, 0x14FD, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#endif 
0x0004, 0x0002, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FD,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#if 0
0x0003, 0x0001, 0x0000, 0x0000, 0x0000, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#endif 
#endif

#ifdef TROUT_WIFI_EVB
0x1E04, 0x1E02, 0x1E01, 0x1E01, 0x1E00, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFE, 0x1BFE, 0x1BFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x1AFE, 0x19FE,
0x19FE, 0x19FE, 0x19FD, 0x19FD, 0x19FD, 0x18FD, 0x18FD, 0x18FD, 0x18FD, 0x18FD,
0x18FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x16FD, 0x16FD, 0x16FD,
0x16FD, 0x16FD, 0x16FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x15FD, 0x14FD,
0x14FD, 0x14FD, 0x14FD, 0x14FD, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
#endif
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0200, 0x0200, 0x0200, 0x0200, 0x0200,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};

#endif /* MXFE_44MHZ */
#endif /* MAX2829 */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAX2830_32

#ifdef MXFE_22MHZ
UWORD16 agc_lut[256] = {

0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};


#elif MXFE_44MHZ

UWORD16 agc_lut[256] = {

0x1608, 0x1606, 0x1605, 0x1604, 0x1604, 0x1603, 0x1603, 0x1502, 0x1502, 0x1502,
0x1502, 0x1501, 0x1501, 0x1501, 0x1401, 0x1401, 0x1401, 0x1401, 0x1400, 0x1400,
0x1400, 0x1300, 0x1300, 0x1300, 0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x12FF,
0x12FF, 0x12FF, 0x12FF, 0x12FF, 0x12FF, 0x12FF, 0x11FF, 0x11FF, 0x11FF, 0x11FF,
0x11FF, 0x11FF, 0x11FF, 0x10FF, 0x10FF, 0x10FE, 0x10FE, 0x10FE, 0x10FE, 0x10FE,
0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0EFE, 0x0EFE,
0x0EFE, 0x0EFE, 0x0EFE, 0x0EFE, 0x0E00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x1000, 0x1000, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0E00, 0x0E00, 0x0E00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x1000, 0x1000, 0x1000, 0x1000,
0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0D00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900,
0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0700,
0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0500, 0x0500, 0x0500,
0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
0x0400, 0x0300, 0x0300, 0x0300, 0x0300, 0x0300

};

#else /* MXFE_22MHZ */

UWORD16 agc_lut[256] = {

0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};

#endif /* MXFE_22MHZ */
#endif /* MAX2830_32 */

#ifdef AL2236

#ifdef MXFE_22MHZ

// 7.5dB back-off
UWORD16 agc_lut[256] = {

0x0008, 0x0006, 0x0005, 0x0004, 0x0004, 0x0003, 0x0003, 0x0002, 0x0002, 0x0002,
0x0002, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000

};

#elif MXFE_44MHZ

UWORD16 agc_lut[256] = {

0x0008, 0x0006, 0x0005, 0x0004, 0x0004, 0x0003, 0x0003, 0x0002, 0x0002, 0x0002,
0x0002, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000

};

#else /* MXFE_44MHZ */

UWORD16 agc_lut[256] = {

0x0005, 0x0003, 0x0002, 0x0001, 0x0001, 0x0000, 0x0000, 0x0000, 0x00FF, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB,
0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000

};

#endif /* MXFE_44MHZ */
#endif /* AL2236 */

#ifdef AL7230

#ifdef MXFE_22MHZ

// 13dB back-off
UWORD16 agc_lut[256] = {

0x1E05, 0x1E03, 0x1E02, 0x1E01, 0x1E01, 0x1D00, 0x1D00, 0x1D00, 0x1DFF, 0x1DFF,
0x1DFF, 0x1CFF, 0x1CFF, 0x1CFE, 0x1CFE, 0x1CFE, 0x1CFE, 0x1BFE, 0x1BFE, 0x1BFE,
0x1BFD, 0x1BFD, 0x1BFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x1AFD, 0x19FD,
0x19FD, 0x19FD, 0x19FC, 0x19FC, 0x19FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC, 0x18FC,
0x18FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x17FC, 0x16FC, 0x16FC, 0x16FC,
0x16FB, 0x16FB, 0x16FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x15FB, 0x14FB,
0x14FB, 0x14FB, 0x14FB, 0x14FB, 0x1400, 0x1400, 0x1400, 0x1300, 0x1300, 0x1300,
0x1300, 0x1300, 0x1300, 0x1300, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200, 0x1200,
0x1200, 0x1200, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100, 0x1100,
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00, 0x0E00,
0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0D00, 0x0C00, 0x0C00,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0B00,
0x0B00, 0x0B00, 0x0B00, 0x0B00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0A00,
0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0800, 0x0800,
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0400, 0x0300, 0x0300, 0x0300,
0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0200, 0x0200, 0x0200, 0x0200, 0x0200,
0x0A00, 0x0A00, 0x0A00, 0x0A00, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900, 0x0900,
0x0900, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0700, 0x0700, 0x0700,
0x0700, 0x0700, 0x0700, 0x0700, 0x0700, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
0x0600, 0x0600, 0x0500, 0x0500, 0x0500, 0x0500, 0x0500, 0x0400, 0x0400, 0x0400,
0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0C00, 0x0B00, 0x0B00, 0x0400, 0x0400,
0x0400, 0x0400, 0x0400, 0x0400, 0x0000, 0x0000

};

#elif MXFE_44MHZ

UWORD16 agc_lut[256] = {

0x0008, 0x0006, 0x0005, 0x0004, 0x0004, 0x0003, 0x0003, 0x0002, 0x0002, 0x0002,
0x0002, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000

};

#else /* MXFE_44MHZ */

UWORD16 agc_lut[256] = {

0x0005, 0x0003, 0x0002, 0x0001, 0x0001, 0x0000, 0x0000, 0x0000, 0x00FF, 0x00FF,
0x00FF, 0x00FF, 0x00FF, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE, 0x00FE,
0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD, 0x00FD,
0x00FD, 0x00FD, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x00FB,
0x00FB, 0x00FB, 0x00FB, 0x00FB, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000

};

#endif /* MXFE_22MHZ */
#endif /* AL7230 */
#endif

UWORD16 tx_filter_coeff_11g[] =
{
    0x015, 0x00B, 0x3CF, 0x3D4, 0x07F, 0x149, 0x09C, 0x0E4
};

#if 0 //change by wlh  improve the TX Power  on 11N mode
UWORD16 tx_filter_coeff_11n[] =
{
    0x3FA, 0x3EE, 0x007, 0x01D, 0x3F9, 0x3CB, 0x008, 0x0A8, 0x101
};
#else
UWORD16 tx_filter_coeff_11n[] =
{
    0x3F8, 0x3E7, 0x009, 0x029, 0x3F6, 0x3B4, 0x00B, 0x0EE, 0x16C
};

#endif 
UWORD16 dsss_pre_filt_coeff[] =
{
    0x13D7, 0x143D, 0x18f3
};

/* Note that this list must be in increasing order of channels as per the    */
/* channel index definition (CH_NUM_FREQ_5_T) so that the index in this      */
/* table can be derived from the channel index correctly.                    */

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
UWORD16 g_lock_conv_fact_start_freq_5[NUM_SUPP_CHANNEL_FREQ_5] = {
#ifdef AL7230
    0x0142,   /* Channel  36  */
    0x0141,   /* Channel  40  */
    0x0140,   /* Channel  44  */
    0x013E,   /* Channel  48  */
    0x013D,   /* Channel  52  */
    0x013C,   /* Channel  56  */
    0x013B,   /* Channel  60  */
    0x013A,   /* Channel  64  */
    0x012F,   /* Channel  100 */
    0x012E,   /* Channel  104 */
    0x012D,   /* Channel  108 */
    0x012C,   /* Channel  112 */
    0x012B,   /* Channel  116 */
    0x012A,   /* Channel  120 */
    0x0129,   /* Channel  124 */
    0x0128,   /* Channel  128 */
    0x0127,   /* Channel  132 */
    0x0126,   /* Channel  136 */
    0x0125,   /* Channel  140 */
    0x0122,   /* Channel  149 */
    0x0121,   /* Channel  153 */
    0x0120,   /* Channel  157 */
    0x011F,   /* Channel  161 */
    0x011E    /* Channel  165 */
#else /* AL7230 */
    0x0284,   /* Channel  36  */
    0x0282,   /* Channel  40  */
    0x027F,   /* Channel  44  */
    0x027D,   /* Channel  48  */
    0x027B,   /* Channel  52  */
    0x0278,   /* Channel  56  */
    0x0276,   /* Channel  60  */
    0x0273,   /* Channel  64  */
    0x025F,   /* Channel  100 */
    0x025D,   /* Channel  104 */
    0x025A,   /* Channel  108 */
    0x0258,   /* Channel  112 */
    0x0256,   /* Channel  116 */
    0x0254,   /* Channel  120 */
    0x0252,   /* Channel  124 */
    0x0250,   /* Channel  128 */
    0x024E,   /* Channel  132 */
    0x024C,   /* Channel  136 */
    0x024A,   /* Channel  140 */
    0x0245,   /* Channel  149 */
    0x0243,   /* Channel  153 */
    0x0241,   /* Channel  157 */
    0x023F,   /* Channel  161 */
    0x023D    /* Channel  165 */
#endif /* AL7230 */
};
#else
UWORD16 g_lock_conv_fact_start_freq_5[NUM_SUPP_CHANNEL_FREQ_5] = {
    0x0284,   /* Channel  36  */
    0x0282,   /* Channel  40  */
    0x027F,   /* Channel  44  */
    0x027D,   /* Channel  48  */
    0x027B,   /* Channel  52  */
    0x0278,   /* Channel  56  */
    0x0276,   /* Channel  60  */
    0x0273,   /* Channel  64  */
    0x025F,   /* Channel  100 */
    0x025D,   /* Channel  104 */
    0x025A,   /* Channel  108 */
    0x0258,   /* Channel  112 */
    0x0256,   /* Channel  116 */
    0x0254,   /* Channel  120 */
    0x0252,   /* Channel  124 */
    0x0250,   /* Channel  128 */
    0x024E,   /* Channel  132 */
    0x024C,   /* Channel  136 */
    0x024A,   /* Channel  140 */
    0x0245,   /* Channel  149 */
    0x0243,   /* Channel  153 */
    0x0241,   /* Channel  157 */
    0x023F,   /* Channel  161 */
    0x023D    /* Channel  165 */
};
#endif

/* Note that this list must be in increasing order of channels as per the    */
/* channel index definition (CH_NUM_FREQ_2_T) so that the index in this      */
/* table can be derived from the channel index correctly.                    */
UWORD16 g_lock_conv_fact_start_freq_2[NUM_SUPP_CHANNEL_FREQ_2] = {
    0x02B4,  /* Channel  1  */
    0x02B2,  /* Channel  2  */
    0x02B1,  /* Channel  3  */
    0x02B0,  /* Channel  4  */
    0x02AE,  /* Channel  5  */
    0x02AD,  /* Channel  6  */
    0x02AB,  /* Channel  7  */
    0x02AA,  /* Channel  8  */
    0x02A9,  /* Channel  9  */
    0x02A7,  /* Channel  10 */
    0x02A6,  /* Channel  11 */
    0x02A4,  /* Channel  12 */
    0x02A3,  /* Channel  13 */
    0x02A0   /* Channel  14 */
};

/*****************************************************************************/
/* Static function declarations                                              */
/*****************************************************************************/

static void program_AGC_LUT(void);
static void update_phy_ittiam_start_freq_5(void);


/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_phy_ittiam_start_freq_5                           */
/*                                                                           */
/*  Description   : This function updates the Ittiam PHY registers for 5GHz  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : phy_reg_addr_start_freq_5_table                          */
/*                  phy_reg_val_start_freq_5_table                           */
/*                                                                           */
/*  Processing    : This function updates the required Ittiam PHY registers  */
/*                  with values for 5GHz band. By default the registers are  */
/*                  initialized with 2.4GHz values.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_phy_ittiam_start_freq_5(void)
{
    UWORD8 i = 0;

    for (i = 0; i < NUM_PHY_ADDR_START_FREQ_5; i++)
    {
        write_dot11_phy_reg(phy_reg_addr_start_freq_5_table[i],
                            phy_reg_val_start_freq_5_table[i]);
    }

    /* Enable 11a only i.e. 11b is disabled */
    write_dot11_phy_reg(rPHYCNTRLREG, PHYCNTRLREG | BIT2);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_lock_conv_fact                                       */
/*                                                                           */
/*  Description   : This function returns the fs/fc value for a channel      */
/*                  based on the current starting frequency                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_lock_conv_fact_start_freq_*                            */
/*                                                                           */
/*  Processing    : This function gets the channel table index for the       */
/*                  given channel. If it is valid it gets the fs/fc value    */
/*                  from the appropriate LUT based on the current start      */
/*                  frequency.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8; fs/fc value for given channel and frequency      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_lock_conv_fact(UWORD8 start_freq, UWORD8 ch_idx)
{
    UWORD16 ret_val = 0;
    UWORD8  tbl_idx = 0;

    /* Get the channel table index for the given channel */
    tbl_idx = get_tbl_idx_from_ch_idx(start_freq, ch_idx);

    if(is_ch_tbl_idx_valid(start_freq, tbl_idx) == BFALSE)
    {
        /* If the given channel table index is not valid it is an exception. */
        /* Do nothing and return.                                            */
    return 0;
    }

    if(start_freq == RC_START_FREQ_5)
    {
        ret_val = g_lock_conv_fact_start_freq_5[tbl_idx];
    }
    else if(start_freq == RC_START_FREQ_2)
    {
        ret_val = g_lock_conv_fact_start_freq_2[tbl_idx];
    }

    return ret_val;
}

static void Trout2_SetTxPower(int result, 
	unsigned int gnTx,unsigned int bTx)
{
    switch(result)
	{
		case 0: //get the datas are right 
		{
			write_dot11_phy_reg(rTXPOWER11A,gnTx);
			write_dot11_phy_reg(rTXPOWER11B,bTx);
			break;
		}

		case -1:// 0x70 nv invalid 
		{
			write_dot11_phy_reg(rTXPOWER11B,bTx);
			break;
		}
		case -2:// 0x71 nv invalid 
		{
			write_dot11_phy_reg(rTXPOWER11A,gnTx);
			break;
		}
		
		default: //all the nv wrong 
			
		break;
		
	}
}

static  void trout2_evb_linear_agc_update(void)
{
	unsigned int i,gnTxPower,bTxPower;
	int nRet =0;
	
	unsigned int RegMap []=
	{	
			//define common regs 
			0x23, 0x1F,
			0x25, 0x01,
			0x27, 0x0F,
			0x28, 0x0b,
			0x29, 0x07,
			0x2E, 0x09,
			0x2F, 0x1b, 
			0x34, 0x00,
			0x41, 0x3a,
			0x42, 0x4a, 
			0x44, 0x1F, 
			0x51, 0x18,
			0X54, 0xA3,
			0X55, 0xA3,
			0X56, 0xA3,
			0x57, 0x14,
			0xF0, 0x01, // 11n rx 
#ifdef TROUT2_CHIP_VER_V2 
			0X54, 0xB4,
			0X55, 0xB4,
			0X56, 0xA1,
			0x57, 0x11,
			0x70, 0x01,//0x30, 11b mask
			0x71, 0x00,
#endif 

#ifdef TROUT2_CHIP_VER_V3
        0x22, 0x5A,
		0x23, 0x1D,
		0x25, 0x02,
		0x27, 0x0A,
		0x2A, 0x10,
		0x2B, 0x88,
		0x2E, 0x07,
		0x2F, 0x1A, 
		0x31, 0xA5,
		0x3E, 0x55,
		0x44, 0x1D, 
		0x4C, 0x60,
		0x4D, 0x14,
		0X54, 0xB4,
		0X55, 0xB4,
		0X56, 0xA1,
		0x57, 0x0B,
		0x5B, 0x28,
		0x6E, 0x08,
//zl@@0729		0x70, 0x3b,
//zl@@0729		0x71, 0x1b,
		0x74, 0X00,
#endif 

	};
	write_dot11_phy_reg(0xff, 0);
	for(i=0;i < sizeof(RegMap)/sizeof(RegMap[0])/2;i++)  {
		write_dot11_phy_reg(RegMap[2*i],RegMap[2*i + 1]);
	}
	//update the Tx Power value from the trout_sdio(get it from the NV item)
	nRet =  GetTxPowerLevel( &gnTxPower,&bTxPower);
	//set the Tx Power 
	Trout2_SetTxPower(nRet,gnTxPower,bTxPower);
	
}

static void SetTxPowerLevel_from_nv(void)
{
    unsigned int gnTxPower,bTxPower;
    int nRet =0;

    //update the Tx Power value from the trout_sdio(get it from the NV item)
    nRet =  GetTxPowerLevel( &gnTxPower,&bTxPower);
    if(0 == nRet)
    {
        phy_reg_val_table[rTXPOWER11A] = gnTxPower;
        phy_reg_val_table[rTXPOWER11B] = bTxPower;
    }
    else if(-1 == nRet){
        phy_reg_val_table[rTXPOWER11B] = bTxPower;
    }
    else if(-2 == nRet){
        phy_reg_val_table[rTXPOWER11A] = gnTxPower;
    }
    printk("%s: default tx power registers TXPOWER11A=0x%x, TXPOWER11B=0x%x\n", 
              __func__, phy_reg_val_table[rTXPOWER11A], phy_reg_val_table[rTXPOWER11B]);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_phy_ittiam                                    */
/*                                                                           */
/*  Description   : This function initializes the Ittiam PHY registers.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_cmcc_set_aci;
extern UWORD32 g_pwr_tpc_switch;
#endif
void initialize_phy_ittiam(void)
{
    /* 8 bits from softPhyReg1 and the 4LSBs of softPhyReg2 for the 12 bit   */
    /* Max length specification. If 1000 is specified as the max length then */
    /* packet length of > 1000 will cause Signal field extractor to produce  */
    /* parity error.                                                         */

    /* MAC Style PHY register writing */

    UWORD8 i = 0;

	TROUT_FUNC_ENTER;
    //yangke, 2013-11-08, set default tx power register from NV
    SetTxPowerLevel_from_nv();
	/* make sure to change to bank1 by zhao */
    write_dot11_phy_reg(0xFF, 0x00);
    for (i = 0; i <= MAX_PHY_ADDR; i++)
    {
        write_dot11_phy_reg(i, phy_reg_val_table[i]);
    }

	#ifdef IBSS_BSS_STATION_MODE
	//if(1 == g_cmcc_set_aci){		
        //    write_dot11_phy_reg(0xF0, 0x65);
	//}
	#endif
    write_dot11_phy_reg(0xFF, 0x01);
    for (i = 0; i <= MAX_PHY_ADDR2; i++)
    {
        write_dot11_phy_reg(i, phy_reg_val_table_2[i]);
    }
    write_dot11_phy_reg(0xFF, 0x00);

    /* Update the PHY registers for frequency band 5GHz if the current band  */
    /* configured is 5GHz.                                                   */
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        update_phy_ittiam_start_freq_5();
    }

    program_AGC_LUT();
    trout2_evb_linear_agc_update(); //add by wlh agc table 0420

#ifdef IBSS_BSS_STATION_MODE
    //yangke, 2013-10-10, for cmcc 11n signalling test, when set max power
    if(!g_pwr_tpc_switch){
        write_dot11_phy_reg(0xFF, 0x00);
        write_dot11_phy_reg(0xF0, 0x65);

        //yangke, 2013-10-24, switch off Q_empty check
        disable_rx_buff_based_ack();
    }
#endif
    TROUT_FUNC_EXIT;
}

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_default_tx_power_levels_ittiam                       */
/*                                                                           */
/*  Description   : This function sets the Tx power levels for Ittiam PHY    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_default_tx_power_levels_ittiam(void)
{
    WORD8  dbm       = 0;
    UWORD8 gain      = 0;
    UWORD8 freq      = get_current_start_freq();
    UWORD8 ch_idx    = mget_CurrentChannel();
    UWORD8 reg_class = get_reg_class_from_ch_idx(freq, ch_idx);
    reg_class_info_t *rc_info = get_reg_class_info(freq, reg_class);

	if(rc_info == NULL)
	{
		PRINTD("Warning: NULL Reg Class Info while setting Default Power levels");
		return;
	}

#ifdef PHY_802_11n
    dbm  = get_rc_max_rf_tx_pow_a(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_curr_tx_power.pow_a.dbm      = dbm;
    g_curr_tx_power.pow_a.gaincode = gain;

    dbm  = get_rc_max_rf_tx_pow_b(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_curr_tx_power.pow_b.dbm      = dbm;
    g_curr_tx_power.pow_b.gaincode = gain;

    dbm  = get_rc_max_rf_tx_pow_n(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_curr_tx_power.pow_n.dbm      = dbm;
    g_curr_tx_power.pow_n.gaincode = gain;

    if(g_reset_mac_in_progress == BFALSE)
    {
        write_dot11_phy_reg(rTXPOWER11A, g_curr_tx_power.pow_a.gaincode);
        set_tx_power_rf(g_curr_tx_power.pow_a.gaincode);


        write_dot11_phy_reg(rTXPOWER11B, g_curr_tx_power.pow_b.gaincode);
        set_tx_power_rf(g_curr_tx_power.pow_b.gaincode);
    }
#endif /* PHY_802_11n */
}

#else

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_default_tx_power_levels_ittiam                       */
/*                                                                           */
/*  Description   : This function sets the Tx power levels for Ittiam PHY    */
/*                  Function get_tx_pow returns minimum of user/reg tx power */
/*                  and default tx. So this function initializes power levels*/
/*                  of g_reg_max_tx_power and g_user_tx_power to             */
/*                  g_default_tx_power.                                      */
/*                                                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifdef TROUT_WIFI_NPI
extern UWORD32 pwr_level_b_val;
extern UWORD32 pwr_level_a_val;
#endif

void set_default_tx_power_levels_ittiam(void)
{
    WORD8  dbm       	  = 0;
    UWORD8 gain      	  = 0;
    UWORD8 freq      	  = get_current_start_freq();
    UWORD8 ch_idx    	  = mget_CurrentChannel();
    UWORD8 reg_max_tx_pow = 0;
    UWORD8 reg_class	  = get_reg_class_from_ch_idx(freq, ch_idx);
    reg_class_info_t *rc_info = get_reg_class_info(freq, reg_class,
                                                   get_current_reg_domain());

	if(rc_info == NULL)
	{
		PRINTD("Warning: NULL Reg Class Info while setting Default Power levels");
		return;
	}




#ifdef PHY_802_11n
    dbm  = get_rc_max_rf_tx_pow_a(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_default_tx_power.pow_a.dbm      = dbm;
    g_default_tx_power.pow_a.gaincode = gain;

    dbm  = get_rc_max_rf_tx_pow_b(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_default_tx_power.pow_b.dbm      = dbm;
    g_default_tx_power.pow_b.gaincode = gain;

    dbm  = get_rc_max_rf_tx_pow_n(rc_info);
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    g_default_tx_power.pow_n.dbm      = dbm;
    g_default_tx_power.pow_n.gaincode = gain;

    if(g_reset_mac_in_progress == BFALSE)
    {
	    //chenq add a macro 2012-12-20
        #ifdef TROUT_WIFI_FPGA //dumy add for Trout EVB  TX Power max 1108
        write_dot11_phy_reg(rTXPOWER11A, g_default_tx_power.pow_a.gaincode);
        #endif
		
        #ifdef TROUT2_WIFI_FPGA //dumy add for Trout EVB  TX Power max 1108
        write_dot11_phy_reg(rTXPOWER11A, g_default_tx_power.pow_a.gaincode);
        #endif	

		#ifdef TROUT2_WIFI_IC //dumy add for Trout EVB  TX Power max 1108
		#ifndef TROUT_WIFI_NPI
		/* lihuai.wang mask this line */
        //write_dot11_phy_reg(rTXPOWER11A, g_default_tx_power.pow_a.gaincode);
		#else // caisf add for npi power control, 20130619
		if(pwr_level_a_val)
			write_dot11_phy_reg(rTXPOWER11A, pwr_level_a_val);
		//else
        //	write_dot11_phy_reg(rTXPOWER11A, g_default_tx_power.pow_a.gaincode);
		
		printk("npi: set_default_tx_power_levels pwr A = %#x.\n",pwr_level_a_val);
		#endif
        #endif
		
		#ifdef TROUT_WIFI_EVB
		#ifdef TROUT_WIFI_EVB_MF //dumy add 0123
		write_dot11_phy_reg(rTXPOWER11A, g_default_tx_power.pow_a.gaincode);
        #else
		write_dot11_phy_reg(rTXPOWER11A, 0x7F);
        #endif
        #endif
        set_tx_power_rf(g_default_tx_power.pow_a.gaincode);
		//chenq add a macro 2012-12-20
        #ifdef TROUT_WIFI_FPGA //dumy add for Trout EVB  TX Power max 1108
        write_dot11_phy_reg(rTXPOWER11B, g_default_tx_power.pow_b.gaincode);
        #endif
		
		#ifdef TROUT2_WIFI_FPGA
		write_dot11_phy_reg(rTXPOWER11B, g_default_tx_power.pow_b.gaincode);
		#endif

		#ifdef TROUT2_WIFI_IC
		#ifndef TROUT_WIFI_NPI
		/* lihuai.wang mask this line */
		//write_dot11_phy_reg(rTXPOWER11B, g_default_tx_power.pow_b.gaincode); 
		#else // caisf add for npi power control, 20130619
		if(pwr_level_b_val)
			write_dot11_phy_reg(rTXPOWER11B, pwr_level_b_val);
		//else
        //	write_dot11_phy_reg(rTXPOWER11B, g_default_tx_power.pow_b.gaincode);
		
		printk("npi: set_default_tx_power_levels pwr B = %#x.\n",pwr_level_b_val);
		#endif
		#endif

		#ifdef TROUT_WIFI_EVB
		#ifdef TROUT_WIFI_EVB_MF //dumy add 0123
		write_dot11_phy_reg(rTXPOWER11B, g_default_tx_power.pow_b.gaincode);
		#else
		write_dot11_phy_reg(rTXPOWER11B, 0x7F);
		#endif       
        #endif
        set_tx_power_rf(g_default_tx_power.pow_b.gaincode);
    }
#endif /* PHY_802_11n */

	/* Get Regulatory Max power for the current Regulatory Class */
	reg_max_tx_pow = get_rc_max_reg_tx_pow(rc_info);

    dbm = MIN(reg_max_tx_pow, g_default_tx_power.pow_a.dbm);

    /* Convert the TX power dBm to appropriate Gain code value */
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_a.dbm      = dbm;
    g_reg_max_tx_power.pow_a.gaincode = gain;

    dbm = MIN(reg_max_tx_pow, g_default_tx_power.pow_b.dbm);

     /* Convert the TX power dBm to appropriate Gain code value */
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_b.dbm      = dbm;
    g_reg_max_tx_power.pow_b.gaincode = gain;

    dbm = MIN(reg_max_tx_pow, g_default_tx_power.pow_n.dbm);

     /* Convert the TX power dBm to appropriate Gain code value */
    gain = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_n.dbm      = dbm;
    g_reg_max_tx_power.pow_n.gaincode = gain;

    g_user_tx_power.pow_a.dbm         = g_default_tx_power.pow_a.dbm;
    g_user_tx_power.pow_a.gaincode    = g_default_tx_power.pow_a.gaincode;

    g_user_tx_power.pow_b.dbm         = g_default_tx_power.pow_b.dbm;
    g_user_tx_power.pow_b.gaincode    = g_default_tx_power.pow_b.gaincode;

    g_user_tx_power.pow_n.dbm         = g_default_tx_power.pow_n.dbm;
    g_user_tx_power.pow_n.gaincode    = g_default_tx_power.pow_n.gaincode;
}
#endif

#ifdef PHY_CONTINUOUS_TX_MODE

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ittiam_phy_continuous_tx_mode_ittiam                 */
/*                                                                           */
/*  Description   : This function sets Ittiam PHY in continuous Tx mode.     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the test mode control registers to    */
/*                  the required initial values for continuous transmission  */
/*                  mode. The data rate is set in the required register bits */
/*                  from the current transmit rate value that is set.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_phy_continuous_tx_mode_ittiam(void)
{
    UWORD8 ur    = 0;
    UWORD8 pr    = 0;
    UWORD8 bval1 = 0;
    UWORD8 aval1 = 0, aval2 = 0;

    UWORD32 temp1, temp2, temp3;

	TROUT_FUNC_ENTER;
    /* The power up time between RF enable(ON) and PHY Tx enable need to be
    extended for the RF PLL to settle before the actual packet transmission
    starts. Without this the header always fails in test mode. The PA is
    turned on normally, just when the Tx IQ samples come so as to mask the
    PA statup pulse */

    write_dot11_phy_reg( rPHYTXPWUPTIME1, 0x00 );
    write_dot11_phy_reg( rPHYTXPWUPTIME2, 0xCC );
    write_dot11_phy_reg( rPHYTXPWUPTIME3, 0xCC );
    write_dot11_phy_reg( rPHYTXPWUPTIME4, 0xFF );

    ur = get_hut_tx_rate();

    if(IS_RATE_MCS(ur) == BFALSE)
    {
        pr = get_phy_rate(ur);

        TROUT_DBG4("Legacy rate = %d\n", pr);

        if(IS_RATE_11B(pr) == BTRUE)
        {
            /* Set the Bits 0 and 1 to the data rate value for 11b rate */
            bval1  = PHYBTSTMODCTRL1_TEST_VAL;
            pr    &= 0x03;
            bval1 |= pr;

            TROUT_DBG4("11b = %x\n", pr);
            /* Note that the rPHYBTSTMODCTRL2 register must be written last in   */
            /* this sequence                                                     */
            write_dot11_phy_reg(rPHYBTSTMODCTRL1, bval1);
            write_dot11_phy_reg(rPHYBTSTMODCTRL2, PHYBTSTMODCTRL2_TEST_VAL);
        }
        else
        {
            /* Set the Bits 0, 1 and 2 to the data rate value for 11a rate */
            aval2  = PHYATSTMODCTRL2_TEST_VAL;
            pr    &= 0x07;
            aval2 |= pr;

            TROUT_DBG4("11a = %x\n", pr);
            /* Note that the rPHYATSTMODCTRL1 register must be written last in   */
            /* this sequence                                                     */
            write_dot11_phy_reg(rPHYATSTMODCTRL2, aval2);
            write_dot11_phy_reg(rPHYATSTMODCTRL3, PHYATSTMODCTRL3_TEST_VAL);
            write_dot11_phy_reg(rPHYATSTMODCTRL1, PHYATSTMODCTRL1_TEST_VAL);
        }
    }
    else
    {
        /* Set Bit 4 for enabling 11n mode BIT 0 for short GI */
        aval1   = PHYATSTMODCTRL1_TEST_VAL | BIT4 | (g_test_params.shortgi & BIT0) ;

        /* Set the Bits 0-2 to the mcs value for 11n rate */
        ur      &= 0x07;
        aval2  = PHYATSTMODCTRL2_TEST_VAL | ur;

        TROUT_DBG4("Continuous Tx MCS = %d sGI = %d\n", ur, aval1 & BIT0);

        /* Note that the rPHYATSTMODCTRL1 register must be written last in   */
        /* this sequence                                                     */
        write_dot11_phy_reg(rPHYATSTMODCTRL2, aval2);
        write_dot11_phy_reg(rPHYATSTMODCTRL3, PHYATSTMODCTRL3_TEST_VAL);
        write_dot11_phy_reg(rPHYATSTMODCTRL1, aval1);

        read_dot11_phy_reg(rPHYATSTMODCTRL1, &temp1);
        read_dot11_phy_reg(rPHYATSTMODCTRL2, &temp2);
        read_dot11_phy_reg(rPHYATSTMODCTRL3, &temp3);

        TROUT_DBG4("11a Test mode reg %02x %02x %02x\n", temp1, temp2, temp3);

    }
    TROUT_FUNC_EXIT;
}

#endif /* PHY_CONTINUOUS_TX_MODE */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : read_phy_hw_version_ittiam                               */
/*                                                                           */
/*  Description   : This function reads the Ittiam PHY version.              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_phy_ver_num                                            */
/*                  g_reg_val_diff                                           */
/*                  g_phy_ver                                                */
/*                                                                           */
/*  Processing    : This function reads the required register and updates    */
/*                  the global variable with the correct PHY version.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void read_phy_hw_version_ittiam(void)
{
    UWORD8   index = 1;
    UWORD32  temp1 = 0;
    UWORD32  temp2 = 0;

    read_dot11_phy_reg(rPHYMSBVERNUMREG, &temp1);
    read_dot11_phy_reg(rPHYLSBVERNUMREG, &temp2);

    g_phy_ver_num  = ((temp1 << 8) + temp2);

    /* Format version as 'x.x.x.x' */
    temp1 = (g_phy_ver_num >> 12) & 0x0f;
    g_phy_ver[index++] = HEX2ASCII(temp1);
    g_phy_ver[index++] = '.';

    temp1 = (g_phy_ver_num >> 8) & 0x0f;
    g_phy_ver[index++] = HEX2ASCII(temp1);
    g_phy_ver[index++] = '.';

    temp1 = (g_phy_ver_num >> 4) & 0x0f;
    g_phy_ver[index++] = HEX2ASCII(temp1);
    g_phy_ver[index++] = '.';

    temp1 = g_phy_ver_num  & 0x0f;
    g_phy_ver[index++] = HEX2ASCII(temp1);

    g_phy_ver[0] = index - 1; /* String length is saved in the first byte */
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_phy_tx_power_level_11a_ittiam                        */
/*                                                                           */
/*  Description   : This function sets the 11a transmit power level for      */
/*                  Ittiam PHY.                                              */
/*                                                                           */
/*  Inputs        : 1) Value of transmit power gaincode                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the required PHY and RF registers.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_phy_tx_power_level_11a_ittiam(UWORD8 val)
{
    phy_reg_val_table[rTXPOWER11A] = val;

    write_dot11_phy_reg(rTXPOWER11A, val);

    set_tx_power_rf(val);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_phy_tx_power_level_11b_ittiam                        */
/*                                                                           */
/*  Description   : This function sets the 11b transmit power level for      */
/*                  Ittiam PHY.                                              */
/*                                                                           */
/*  Inputs        : 1) Value of transmit power gaincode                      */
/*                                                                           */
/*  Globals       : None                                                      */
/*                                                                           */
/*  Processing    : This function sets the required PHY and RF registers.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_phy_tx_power_level_11b_ittiam(UWORD8 val)
{
    phy_reg_val_table[rTXPOWER11B] = val;

    write_dot11_phy_reg(rTXPOWER11B, val);

    set_tx_power_rf(val);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : program_AGC_LUT                                          */
/*                                                                           */
/*  Description   : This programs the AGC LUT                                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : AGC LUT array                                            */
/*                                                                           */
/*  Processing    : This function reads the Look-Up-Table (LUT) values from  */
/*                  the agc LUT array and writes to the PHY register. The    */
/*                  LUT has 256 words of 16 bits. The LSB's and MSB's go     */
/*                  into different registers                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void program_AGC_LUT(void)
{
    UWORD16 i                = 0;
    UWORD32 lsb_byte, msb_byte;


    /* Reset the AGC LUT ram in PHY*/
    write_dot11_phy_reg(rERSSI_EN, ERSSI_EN | BIT5);

    /* Bring the AGC LUT ram out of reset */
    write_dot11_phy_reg(rERSSI_EN, ERSSI_EN & (~BIT5));

    for(i = 0; i < 256; i++)
    {
        write_dot11_phy_reg(rAGCSPRAMLSB, agc_lut[i] & 0x00FF);
        write_dot11_phy_reg(rAGCSPRAMMSB, (agc_lut[i] >> 8) & 0x00FF);
    }


        /* Program the Tx filter coefficient for OFDM Tx filter */
        /* and DSSS post filter                                 */

    for(i = 0; i < 8; i++)
        {
        /* Pack the coeff as required by PHY */
        lsb_byte = tx_filter_coeff_11g[i] & 0xFF;
        msb_byte = 0x40 | (i << 2) | ((tx_filter_coeff_11g[i] >> 8) & 0x03);

        write_dot11_phy_reg(rTXPOSTFILTCOEFFLSB, lsb_byte);
        write_dot11_phy_reg(rTXPOSTFILTCOEFFMSB, msb_byte);

    }

    for(i = 0; i < 9; i++)
    {
        /* Pack the coeff as required by PHY */
        lsb_byte = tx_filter_coeff_11n[i] & 0xFF;
        msb_byte = 0xC0 | (i << 2) | ((tx_filter_coeff_11n[i] >> 8) & 0x03);

        write_dot11_phy_reg(rTXPOSTFILTCOEFFLSB, lsb_byte);
        write_dot11_phy_reg(rTXPOSTFILTCOEFFMSB, msb_byte);
    }

    for(i = 0; i < 3; i++)
    {
        write_dot11_phy_reg(rDSSSTXPREFILTCOEFFLSB, dsss_pre_filt_coeff[i] & 0x00FF);
        write_dot11_phy_reg(rDSSSTXPREFILTCOEFFMSB, (dsss_pre_filt_coeff[i] >> 8) & 0x00FF);
    }

#ifdef PHY_TEST_CODE
    UWORD32 phy_impl;

    for(i = 0; i <= MAX_PHY_ADDR; i++)
    {
        read_dot11_phy_reg(i, &phy_impl);

        PRINTK("0x%02x 0x%02x",i, phy_impl);

        if(phy_impl != phy_reg_val_table[i])
            PRINTK("  Register mismatch written %02x", phy_reg_val_table[i]);

        PRINTK("\n");

    }

    read_dot11_phy_reg(rPHYIMPLEMENTATIONREG, &phy_impl);

#ifdef OS_LINUX_CSL_TYPE
    PRINTK("\nMAC HW version: %s\n", get_hardwareProductVersion());
    PRINTK("PHY version: %04x\nImplementation:%x \n", g_phy_ver_num, phy_impl);
    PRINTK("Programming the AGC LUT \n");
#endif/* OS_LINUX_CSL_TYPE */

#endif /* PHY_TEST_CODE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_phy_ittiam                                         */
/*                                                                           */
/*  Description   : This function resets the Ittiam PHY.                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function resets the Ittiam PHY by resetting the     */
/*                  required MAC H/w register bit.                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_phy_ittiam(void)
{
	TROUT_FUNC_ENTER;
    //chenq mod
    //rMACPHYRESCNRTL &= ~REGBIT1;
    host_write_trout_reg(host_read_trout_reg((UWORD32)TROUT_SYS_REG_WIFI_RST) & (~REGBIT0) ,
        					(UWORD32)TROUT_SYS_REG_WIFI_RST);

    /* Keep PHY in reset mode for some time */
    add_calib_delay(10);

    //chenq mod
    //rMACPHYRESCNRTL |= REGBIT1;
    host_write_trout_reg(host_read_trout_reg((UWORD32)TROUT_SYS_REG_WIFI_RST) | (REGBIT0) ,
        					(UWORD32)TROUT_SYS_REG_WIFI_RST);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : power_up_phy_ittiam                                      */
/*                                                                           */
/*  Description   : This function powers up the Ittiam PHY.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function powers up the Ittiam PHY by setting the    */
/*                  required PHY H/w register bit.                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void power_up_phy_ittiam(void)
{
    write_dot11_phy_reg(rPHYPOWDNCNTRL, (PHYPOWDNCNTRL & ~(BIT0|BIT1)) | PHY_ACTIVE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : power_down_phy_ittiam                                    */
/*                                                                           */
/*  Description   : This function powers up the Ittiam PHY.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function powers down the Ittiam PHY by resetting the*/
/*                  required PHY H/w register bit.                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/


/* This function powers down the Ittiam PHY */
void power_down_phy_ittiam(void)
{
    /* Shut down PHY */
    write_dot11_phy_reg(rPHYPOWDNCNTRL,
                  (PHYPOWDNCNTRL & ~(BIT0 | BIT1)) |  PHY_SHUTDOWN);
}
/*zhq add for powersave*/
void root_power_down_phy_ittiam(void)
{
    /* Shut down PHY */
    root_write_dot11_phy_reg(rPHYPOWDNCNTRL,
                  (PHYPOWDNCNTRL & ~(BIT0 | BIT1)) |  PHY_SHUTDOWN);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_tssi_11b_ittiam                                      */
/*                                                                           */
/*  Description   : This function gets the TSSI for 11b.                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the TSSI for 11b.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, TSSI value                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_tssi_11b_ittiam(void)
{
    UWORD32 temp = 0;

    /* Reset and initialize PHY */
    reset_phy();
    initialize_phy();

    set_default_tx_power_levels();

    /* set each registers for Tx-calibration */
    write_dot11_phy_reg(rTXPOWCALDLY,      0x10); /* set delay of Tx-enable   */
    write_dot11_phy_reg(rTXPOWERCALAVGWIN, 0x08); /* set average sample count */
    write_dot11_phy_reg(rTXPOWCALCNTRL,    0x01); /* turn on Tx-calibration   */
    write_dot11_phy_reg(rPHYBTSTMODCTRL1,  0x58); /* Test mode setting        */
    write_dot11_phy_reg(rPHYBTSTMODCTRL2,  0x1D); /* set cont preamble mode   */

    /* Wait until average tssi is calculated */
    while(1)
    {
        read_dot11_phy_reg(rTXPOWCALCNTRL, &temp);
        if(temp & BIT1)
        {
            break;
        }
    }

    /* read rPOWDET11B value */
    read_dot11_phy_reg(rPOWDET11B, &temp);

    initialize_phy();

    return (UWORD8)temp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_tssi_11gn_ittiam                                     */
/*                                                                           */
/*  Description   : This function gets the TSSI for 11g/n.                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the TSSI for 11g/n.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, TSSI value                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_tssi_11gn_ittiam(void)
{
    UWORD32 temp = 0;

    /* reset and initialize PHY */
    reset_phy();
    initialize_phy();

    set_default_tx_power_levels();

    /* Set each registers for Tx-calibration */
    write_dot11_phy_reg(rTXPOWCALDLY,      0x10); /* set delay of Tx-enable */
    write_dot11_phy_reg(rTXPOWERCALAVGWIN, 0x02); /* set average sample cnt */
    write_dot11_phy_reg(rTXPOWCALCNTRL,    0x01); /* turn on Tx-calibration */
    write_dot11_phy_reg(rPHYATSTMODCTRL2,  0x44); /* Test-Mode enable       */
    write_dot11_phy_reg(rPHYATSTMODCTRL3,  0x00); /* mode, 54Mbps, Random   */
    write_dot11_phy_reg(rPHYATSTMODCTRL1,  0xC2); /* start cont preamble    */

    /* wait until average tssi is calculated */
    while(1)
    {
        read_dot11_phy_reg(rTXPOWCALCNTRL, &temp);
        if(temp & BIT1)
        {
            break;
        }
    }

    /* Read rPOWDET11A value */
    read_dot11_phy_reg(rPOWDET11A, &temp);

    initialize_phy();

    return (UWORD8)temp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_supp_mcs_bitmap_ittiam                            */
/*                                                                           */
/*  Description   : This function updates the bitmap of MCS supported by the */
/*                  Ittiam PHY. If MCS i is supported then bit i in the      */
/*                  bitmap is set to 1. Bit0 is the msb of the first byte in */
/*                  the Bitmap character array.                              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : The number of MCSs supported.                            */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 update_supp_mcs_bitmap_ittiam(UWORD8 *mcs_bmp)
{
    UWORD8 mcs = 0;
    UWORD8 idx = 0;

    /* All MCS between 0 to MAX_MCS_SUPPORTED are supported */
    for(mcs = 0; mcs <= MAX_MCS_SUPPORTED; mcs++)
    {
        idx = mcs >> 3;
        mcs_bmp[idx] |= (1 << (mcs & 7));
    }

    return MAX_MCS_SUPPORTED + 1;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : enable_radar_detection_ittiam                            */
/*                                                                           */
/*  Description   : Enables the radar detection algorithm in Ittiam PHY      */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function is used set the appropriate register val   */
/*                  in the appropriate PHY register in order to enable the   */
/*                  radar detection in Ittiam PHY.                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void enable_radar_detection_ittiam(void)
{
	if(phy_reg_val_table[rPHYRADARCNTRLREG1] != RADAR_ENABLE_VALUE)
	{
		phy_reg_val_table[rPHYRADARCNTRLREG1] = RADAR_ENABLE_VALUE;
		write_dot11_phy_reg(rPHYRADARCNTRLREG1, RADAR_ENABLE_VALUE);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : disable_radar_detection_ittiam                           */
/*                                                                           */
/*  Description   : Disables the radar detection algorithm in Ittiam PHY     */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function is used set the appropriate register val   */
/*                  in the appropriate PHY register in order to disable the  */
/*                  radar detection in Ittiam PHY.                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void disable_radar_detection_ittiam(void)
{
	if(phy_reg_val_table[rPHYRADARCNTRLREG1] != 0x00)
	{
		phy_reg_val_table[rPHYRADARCNTRLREG1] = 0;
		write_dot11_phy_reg(rPHYRADARCNTRLREG1, 0x00);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : post_radar_detect_ittiam                                 */
/*                                                                           */
/*  Description   : Handles the post radar detection situation with PHY      */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function resets the Radar detected bit in the       */
/*                  appropriate register in the Ittiam PHY                   */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void post_radar_detect_ittiam(void)
{
    UWORD32 radar_det_phy, radar_det_phy_reset;

	read_dot11_phy_reg(rPHYRADARCNTRLREG6, &radar_det_phy);

	/* Clear the radar status bit */
	radar_det_phy_reset = (radar_det_phy & 0x7F);

	write_dot11_phy_reg(rPHYRADARCNTRLREG6, radar_det_phy_reset);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_ofdmpre_unidentified_bits_ittiam                   */
/*                                                                           */
/*  Description   : Resets OFDM and unidentified bits in PHY reg.            */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function is used to reset the Detection             */
/*                  of OFDM preamble bit and unidentified signal bit in      */
/*                  the Phy register.                                        */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void reset_ofdmpre_unidentified_bits_ittiam(void)
{
	UWORD8 phy_reg_reset = 0x1F;
	write_dot11_phy_reg(rPHYRADARCNTRLREG6, phy_reg_reset);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_unidentified_bit_detected_ittiam                      */
/*                                                                           */
/*  Description   : Read un-identified bit from PHY reg                      */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function is used to read the Detection              */
/*                  of un-identified bits in the Phy register.               */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_unidentified_bit_detected_ittiam(void)
{
    UWORD32 read_reg;
    read_dot11_phy_reg(rPHYRADARCNTRLREG6, &read_reg);

    if(read_reg & BIT5)
        return BTRUE;
    else
        return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_ofdm_preamble_detected_ittiam                         */
/*                                                                           */
/*  Description   : Read OFDM bit from PHY reg                               */
/*                                                                           */
/*  Inputs        : None.                                                    */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    : This function is used to read the Detection              */
/*                  of OFDM preamble bit in the Phy register.                */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         24 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_ofdm_preamble_detected_ittiam(void)
{
    UWORD32 read_reg;
	read_dot11_phy_reg(rPHYRADARCNTRLREG6, &read_reg);

    if(read_reg & BIT6)
        return BTRUE;
    else
        return BFALSE;
}

#endif /* ITTIAM_PHY */

