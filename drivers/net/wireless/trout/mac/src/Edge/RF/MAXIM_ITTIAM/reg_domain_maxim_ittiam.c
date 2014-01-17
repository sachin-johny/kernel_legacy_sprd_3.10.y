/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : reg_domain_maxim_ittiam.c                            */
/*                                                                           */
/*  Description       : This file contains the supported Regulatory Domain   */
/*                      configuration for Maxim RF/Ittiam PHY.               */
/*                                                                           */
/*  List of Functions : init_reg_domain_config                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef RF_MAXIM_ITTIAM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "reg_domains.h"
#include "ch_info.h"
#include "reg_domain_maxim_ittiam.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Supported Channel Bitmap in each frequency band                           */
/*****************************************************************************/

UWORD32 g_ch_bmap_freq_2 = (GET_CH_BIT(CHANNEL1)  |
                            GET_CH_BIT(CHANNEL2)  |
                            GET_CH_BIT(CHANNEL3)  |
                            GET_CH_BIT(CHANNEL4)  |
                            GET_CH_BIT(CHANNEL5)  |
                            GET_CH_BIT(CHANNEL6)  |
                            GET_CH_BIT(CHANNEL7)  |
                            GET_CH_BIT(CHANNEL8)  |
                            GET_CH_BIT(CHANNEL9)  |
                            GET_CH_BIT(CHANNEL10) |
                            GET_CH_BIT(CHANNEL11) |
                            GET_CH_BIT(CHANNEL12) |
                            GET_CH_BIT(CHANNEL13) |
                            GET_CH_BIT(CHANNEL14));

UWORD32 g_ch_bmap_freq_5 = (GET_CH_BIT(CHANNEL36)  |
                            GET_CH_BIT(CHANNEL40)  |
                            GET_CH_BIT(CHANNEL44)  |
                            GET_CH_BIT(CHANNEL48)  |
                            GET_CH_BIT(CHANNEL52)  |
                            GET_CH_BIT(CHANNEL56)  |
                            GET_CH_BIT(CHANNEL60)  |
                            GET_CH_BIT(CHANNEL64)  |
                            GET_CH_BIT(CHANNEL100) |
                            GET_CH_BIT(CHANNEL104) |
                            GET_CH_BIT(CHANNEL108) |
                            GET_CH_BIT(CHANNEL112) |
                            GET_CH_BIT(CHANNEL116) |
                            GET_CH_BIT(CHANNEL120) |
                            GET_CH_BIT(CHANNEL124) |
                            GET_CH_BIT(CHANNEL128) |
                            GET_CH_BIT(CHANNEL132) |
                            GET_CH_BIT(CHANNEL136) |
                            GET_CH_BIT(CHANNEL140) |
                            GET_CH_BIT(CHANNEL149) |
                            GET_CH_BIT(CHANNEL153) |
                            GET_CH_BIT(CHANNEL157) |
                            GET_CH_BIT(CHANNEL161) |
                            GET_CH_BIT(CHANNEL165));

/*****************************************************************************/
/* Supported Regulatory Domain List                                          */
/*****************************************************************************/
// 20120830 caisf add, merged ittiam mac v1.3 code
UWORD8 *g_sup_reg_dom_info= {"0 => No Regulatory Domain,1 => US"};
/*****************************************************************************/
/* NOTE: Order of domain list elements should match REG_DOMAINS_T enum       */
/*****************************************************************************/

/*****************************************************************************/
/* Regulatory Domain and Regulatory Class Tables                             */
/* NOTE - To add new Regulatory Domains, additional tables for Regulatory    */
/*        classes for that Regulatory Domain must be added here.             */
/*****************************************************************************/

reg_domain_info_t g_rd_info[NUM_REG_DOMAINS];
reg_class_info_t  g_rc_info_rd0[NUM_REG_CLASS_RD0];
reg_class_info_t  g_rc_info_rd1[NUM_REG_CLASS_RD1];

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_reg_domain_maxim_ittiam                             */
/*                                                                           */
/*  Description   : This function initializes the Regulatory Domain          */
/*                  configuration supported for Maxim RF/Ittiam PHY.         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_rd_info                                                */
/*                  g_rc_info_rd0                                            */
/*                                                                           */
/*  Processing    : The information for all Regulatory Domains and the       */
/*                  included Regulatory Classes is initialized.              */
/*                  NOTE - To modify the supported Regulatory Domain info    */
/*                  this function must be updated appropriately and the      */
/*                  relevant PHY/RF dependant constants must be defined      */
/*                  in the appropriate PHY/RF files.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_reg_domain_maxim_ittiam(void)
{
    reg_class_info_t *rc    = NULL;
    UWORD8           rc_idx = 0;
    UWORD8           i      = 0;
	TROUT_FUNC_ENTER;
    /*************************************************************************/
    /* Regulatory Domain #0 (NO_REG_DOMAIN)                                  */
    /*************************************************************************/

    /* --------------------------------------------------------------------- */
    /* Default Table: To be used when Multi-Domain is not enabled            */
    /* --------------------------------------------------------------------- */
    /* |RC  | RC | Freq  | Spac  | Channel            | TX pow | Behavior  | */
    /* |IDX |    | (GHz) | (MHz) | Set                | limit  | Limit Set | */
    /* --------------------------------------------------------------------- */
    /* | 0  | 1  |  5    | 20    | 36, 40, 44, 48,    | MD-TBD | MD-TBD    | */
    /* |    |    |       |       | 52, 56, 60, 64,    |        |           | */
    /* |    |    |       |       | 100, 104, 108, 112 |        |           | */
    /* |    |    |       |       | 116, 120, 124, 128 |        |           | */
    /* |    |    |       |       | 132, 136, 140      |        |           | */
    /* |    |    |       |       | 149, 153, 157, 161 |        |           | */
    /* |    |    |       |       | 165                |        |           | */
    /* --------------------------------------------------------------------- */
    /* | 1  | 2  | 2.407 | 25    | 1, 2, 3, 4, 5, 6,  | MD-TBD | MD-TBD    | */
    /* |    |    |       |       | 7, 8, 9, 10, 11    |        |           | */
    /* |    |    |       |       | 12, 13, 14         |        |           | */
    /* --------------------------------------------------------------------- */

    strcpy(g_rd_info[RD0].country, "XXX");

    g_rd_info[RD0].reg_class_bmap[0] = (BIT1 | BIT2);
    g_rd_info[RD0].reg_class_bmap[1] = 0x00000000;

    for(i = 0; i < NUM_REG_CLASS_RD0; i++)
        g_rd_info[RD0].reg_class_info[i] = &g_rc_info_rd0[i];

    /*************************************************************************/
    /* Regulatory Domain #0: Regulatory Class 1                              */
    /*************************************************************************/
    rc = &g_rc_info_rd0[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL36)  |
                               GET_CH_BIT(CHANNEL40)  |
                               GET_CH_BIT(CHANNEL44)  |
                               GET_CH_BIT(CHANNEL48)  |
                               GET_CH_BIT(CHANNEL52)  |
                               GET_CH_BIT(CHANNEL56)  |
                               GET_CH_BIT(CHANNEL60)  |
                               GET_CH_BIT(CHANNEL64)  |
                               GET_CH_BIT(CHANNEL100) |
                               GET_CH_BIT(CHANNEL104) |
                               GET_CH_BIT(CHANNEL108) |
                               GET_CH_BIT(CHANNEL112) |
                               GET_CH_BIT(CHANNEL116) |
                               GET_CH_BIT(CHANNEL120) |
                               GET_CH_BIT(CHANNEL124) |
                               GET_CH_BIT(CHANNEL128) |
                               GET_CH_BIT(CHANNEL132) |
                               GET_CH_BIT(CHANNEL136) |
                               GET_CH_BIT(CHANNEL140) |
                               GET_CH_BIT(CHANNEL149) |
                               GET_CH_BIT(CHANNEL153) |
                               GET_CH_BIT(CHANNEL157) |
                               GET_CH_BIT(CHANNEL161) |
                               GET_CH_BIT(CHANNEL165));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD0_RC01_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD0_RC01_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD0_RC01_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD0_RC01_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD0_RC01_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #0: Regulatory Class 2                              */
    /*************************************************************************/
    rc = &g_rc_info_rd0[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_2);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_25MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL1)  |
                               GET_CH_BIT(CHANNEL2)  |
                               GET_CH_BIT(CHANNEL3)  |
                               GET_CH_BIT(CHANNEL4)  |
                               GET_CH_BIT(CHANNEL5)  |
                               GET_CH_BIT(CHANNEL6)  |
                               GET_CH_BIT(CHANNEL7)  |
                               GET_CH_BIT(CHANNEL8)  |
                               GET_CH_BIT(CHANNEL9)  |
                               GET_CH_BIT(CHANNEL10) |
                               GET_CH_BIT(CHANNEL11) |
                               GET_CH_BIT(CHANNEL12) |
                               GET_CH_BIT(CHANNEL13) |
                               GET_CH_BIT(CHANNEL14));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD0_RC02_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD0_RC02_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD0_RC02_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD0_RC02_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD0_RC02_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1                                                  */
    /*************************************************************************/
// 20120830 caisf mod, merged ittiam mac v1.3 code
    /* --------------------------------------------------------------------- */
    /* Table J.1: Regulatory classes in the USA                              */
    /* --------------------------------------------------------------------- */
    /* |RC  | RC | Freq  | Spac  | Channel            | TX pow | Behavior  | */
    /* |IDX |    | (GHz) | (MHz) | Set                | limit  | Limit Set | */
    /* --------------------------------------------------------------------- */
    /* | 0  | 1  |  5    | 20    | 36, 40, 44, 48     | 40     | 1, 4      | */
    /* --------------------------------------------------------------------- */
    /* | 1  | 2  |  5    | 20    | 52, 56, 60, 64     | 200    | 1,16      | */
    /* --------------------------------------------------------------------- */
    /* | 2  | 3  |  5    | 20    | 149, 153, 157, 161 | 800    | 1         | */
    /* --------------------------------------------------------------------- */
    /* | 3  | 4  |  5    | 20    | 100, 104, 108, 112 | 200    | 1, 4, 16  | */
    /* |    |    |       |       | 116, 120, 124, 128 |        |           | */
    /* |    |    |       |       | 132, 136, 140      |        |           | */
    /* --------------------------------------------------------------------- */
    /* | 4  | 5  |  5    | 20    | 165                | 1000   | 1         | */
    /* --------------------------------------------------------------------- */
    /* | 5  | 12 | 2.407 | 25    | 1, 2, 3, 4, 5, 6,  | 1000   | 10        | */
    /* |    |    |       |       | 7, 8, 9, 10, 11    |        |           | */
    /* --------------------------------------------------------------------- */

    strcpy(g_rd_info[RD1].country, "US ");

    g_rd_info[RD1].reg_class_bmap[0] = (BIT1 | BIT2 | BIT3 | BIT4 |
                                        BIT5 | BIT12);
    g_rd_info[RD1].reg_class_bmap[1] = 0x00000000;

    for(i = 0; i < NUM_REG_CLASS_RD1; i++)
        g_rd_info[RD1].reg_class_info[i] = &g_rc_info_rd1[i];

    rc_idx = 0; /* Re-initialize Regulatory Class table index */

    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 1                              */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL36) |
                               GET_CH_BIT(CHANNEL40) |
                               GET_CH_BIT(CHANNEL44) |
                               GET_CH_BIT(CHANNEL48));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC01_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC01_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC01_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC01_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC01_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 2                              */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

	// 20120830 caisf mod, merged ittiam mac v1.3 code
	#if 0
    set_rc_behaviour_bmap (rc, RC_DFS_BIT);
	#else
    set_rc_behaviour_bmap (rc, RC_DFS_BIT |
                               RC_DFS_50_100_BIT);
	#endif

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL52) |
                               GET_CH_BIT(CHANNEL56) |
                               GET_CH_BIT(CHANNEL60) |
                               GET_CH_BIT(CHANNEL64));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC02_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC02_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC02_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC02_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC02_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 3                              */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL149) |
                               GET_CH_BIT(CHANNEL153) |
                               GET_CH_BIT(CHANNEL157) |
                               GET_CH_BIT(CHANNEL161));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC03_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC03_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC03_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC03_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC03_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 4                              */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

	// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    set_rc_behaviour_bmap (rc, RC_DFS_BIT);
#else
    set_rc_behaviour_bmap (rc, RC_DFS_BIT |
                               RC_DFS_50_100_BIT);
#endif

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL100) |
                               GET_CH_BIT(CHANNEL104) |
                               GET_CH_BIT(CHANNEL108) |
                               GET_CH_BIT(CHANNEL112) |
                               GET_CH_BIT(CHANNEL116) |
                               GET_CH_BIT(CHANNEL120) |
                               GET_CH_BIT(CHANNEL124) |
                               GET_CH_BIT(CHANNEL128) |
                               GET_CH_BIT(CHANNEL132) |
                               GET_CH_BIT(CHANNEL136) |
                               GET_CH_BIT(CHANNEL140));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC04_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC04_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC04_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC04_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC04_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 5                              */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_5);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_20MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL165));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC05_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC05_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC05_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC05_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC05_MAX_TX_POW_N);


    /*************************************************************************/
    /* Regulatory Domain #1: Regulatory Class 12                             */
    /*************************************************************************/
    rc = &g_rc_info_rd1[rc_idx++];

    set_rc_start_freq     (rc, RC_START_FREQ_2);
    set_rc_ch_spacing     (rc, RC_CH_SPACING_25MHZ);

    set_rc_behaviour_bmap (rc, 0);

    set_rc_channel_bmap   (rc, GET_CH_BIT(CHANNEL1)  |
                               GET_CH_BIT(CHANNEL2)  |
                               GET_CH_BIT(CHANNEL3)  |
                               GET_CH_BIT(CHANNEL4)  |
                               GET_CH_BIT(CHANNEL5)  |
                               GET_CH_BIT(CHANNEL6)  |
                               GET_CH_BIT(CHANNEL7)  |
                               GET_CH_BIT(CHANNEL8)  |
                               GET_CH_BIT(CHANNEL9)  |
                               GET_CH_BIT(CHANNEL10) |
                               GET_CH_BIT(CHANNEL11));

    set_rc_coverage_class (rc, 0);
    set_rc_max_reg_tx_pow (rc, RD1_RC12_REG_MAX_TX_POW);

    set_rc_min_rf_tx_pow  (rc, RD1_RC12_MIN_TX_POW);
    set_rc_max_rf_tx_pow_a(rc, RD1_RC12_MAX_TX_POW_A);
    set_rc_max_rf_tx_pow_b(rc, RD1_RC12_MAX_TX_POW_B);
    set_rc_max_rf_tx_pow_n(rc, RD1_RC12_MAX_TX_POW_N);

	TROUT_FUNC_EXIT;
}

#endif /* RF_MAXIM_ITTIAM */
