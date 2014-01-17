/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : mib_802_11n.c                                        */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to 802.11n PHY MIB.                          */
/*                      Only 802.11g related MIB are initialized properly.   */
/*                      Rest of the MIBs are set to its default values.      */
/*                                                                           */
/*  List of Functions : init_phy_802_11n_mib                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef PHY_802_11n

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "mib_802_11n.h"
#include "phy_802_11n.h"
#include "phy_hw_if.h"
#include "rf_if.h"
#include "ch_info.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/* PHY Management Information Base */
mibp_t MIBP  = {0};
p_mibp_t P_MIBP = {TV_FALSE};
/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static UWORD8 g_tx_power_levels_11g[NUM_TX_POWER_LEVELS] = {0x06, /* 14dbm */
                                                            0x16, /* 10dbm */
                                                            0x25, /*  6dbm */
                                                            0x31, /*  3dbm */
                                                            0x34, /*  2dbm */
                                                            0x38, /*  1dbm */
                                                            0x3C, /*  0dbm */
                                                            0x40  /* -1dbm */};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_phy_802_11n_mib                                     */
/*                                                                           */
/*  Description   : This function initializes PHY 802.11n MIB entries.       */
/*                  Only 802.11g specific MIB entries are initialize properly*/
/*                  802.11n specific MIB entries are set to their standard   */
/*                  defined default values.                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : MIBP                                                     */
/*                                                                           */
/*  Processing    : All the MIB entries are initialized to their default     */
/*                  values.                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_phy_802_11n_mib(void)
{
    UWORD32 i = 0;
	TROUT_FUNC_ENTER;
    /*************************************************************************/
    /*                     Phy Operation Table Initializations               */
    /*************************************************************************/

    MIBP.dot11PHYType          = 7;    /* HT */
    MIBP.dot11CurrentRegDomain = 0x10; /* FCC(USA) */
    MIBP.dot11TempType         = 1;    /* Type 1, 0 to 40 degrees C */

    /*************************************************************************/
    /*                     Phy Antenna Table Initializations                 */
    /*************************************************************************/

    MIBP.dot11CurrentTxAntenna                                     = 1;
    MIBP.dot11DiversitySupport                                     = 2;
    MIBP.dot11CurrentRxAntenna                                     = 1;
    MIBP.dot11AntennaSelectionOptionImplemented                    = TV_FALSE;
    MIBP.dot11TransmitExplicitCSIFeedbackASOptionImplemented       = TV_FALSE;;
    MIBP.dot11TransmitIndicesFeedbackASOptionImplemented           = TV_FALSE;;
    MIBP.dot11ExplicitCSIFeedbackASOptionImplemented               = TV_FALSE;;
    MIBP.dot11TransmitIndicesComputationFeedbackASOptionImplemented= TV_FALSE;;
    MIBP.dot11ReceiveAntennaSelectionOptionImplemented             = TV_FALSE;;
    MIBP.dot11TransmitSoundingPPDUOptionImplemented                = TV_FALSE;;

    /*************************************************************************/
    /*                     Phy Tx Power Table Initializations                */
    /*************************************************************************/

    MIBP.dot11NumberSupportedPowerLevels = NUM_TX_POWER_LEVELS;

    for(i = 0; i < NUM_TX_POWER_LEVELS; i++)
        MIBP.dot11TxPowerLevel[i] = g_tx_power_levels_11g[i];

    MIBP.dot11CurrentTxPowerLevel        = 0;

    /*************************************************************************/
    /*                     Phy FHSS Table Initializations                    */
    /*************************************************************************/

    /* NOTE: Currently unused and hence no access functions are provided     */

    /*************************************************************************/
    /*                     Phy DSSS Table Initializations                    */
    /*************************************************************************/
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
		MIBP.dot11CurrentChannel = CHANNEL1;
	}
	else if(get_current_start_freq() == RC_START_FREQ_5)
	{
		MIBP.dot11CurrentChannel = CHANNEL36;
	}
    MIBP.dot11CCAModeSupported = 7;        /* ED, CS, ED + CS */
    MIBP.dot11CurrentCCAMode   = 1;        /* ED ONLY         */
    MIBP.dot11EDThreshold      = 0;

    /*************************************************************************/
    /*                     Phy IR Table Initializations                      */
    /*************************************************************************/

    /* NOTE: Currently unused and hence no access functions are provided     */

    /*************************************************************************/
    /*           Reg Domains Supported Table Initializations                 */
    /*************************************************************************/

    MIBP.dot11RegDomainsSupportValue = 0x10; /* FCC (USA) */

    /*************************************************************************/
    /*                Antenna List Table Initializations                     */
    /*************************************************************************/

    for(i = 0; i < NUM_ANTENNASLISTTABLE; i++)
    {
        MIBP.dot11AntennasListTable[i].dot11SupportedTxAntenna   = TV_FALSE;
        MIBP.dot11AntennasListTable[i].dot11SupportedRxAntenna   = TV_FALSE;
        MIBP.dot11AntennasListTable[i].dot11DiversitySelectionRx = TV_FALSE;
    }

    /*************************************************************************/
    /*            Supported Data rate Tx Table Initializations               */
    /* The values here needs to be verified before integration               */
    /*************************************************************************/

    for(i = 0; i < NUM_DR_PHY_802_11G; i++)
        MIBP.dot11SupportedDataRatesTxValue.rates[i] = 0;

    MIBP.dot11SupportedDataRatesTxValue.num_rates = 0;

    /*************************************************************************/
    /*            Supported Data rate Rx Table Initializations               */
    /* The values here needs to be verified before integration               */
    /*************************************************************************/

    for(i = 0; i < NUM_DR_PHY_802_11G; i++)
        MIBP.dot11SupportedDataRatesRxValue.rates[i] = 0;

    MIBP.dot11SupportedDataRatesRxValue.num_rates = 0;

    /*************************************************************************/
    /*                      OFDM Table Initializations                       */
    /*************************************************************************/

    MIBP.dot11CurrentFrequency              = 0;
    MIBP.dot11TIThreshold                   = 0;
    MIBP.dot11FrequencyBandsSupported       = 0;
    MIBP.dot11ChannelStartingFactor         = 0;
    MIBP.dot11FiveMHzOperationImplemented   = TV_FALSE;
    MIBP.dot11TenMHzOperationImplemented    = TV_FALSE;
    MIBP.dot11TwentyMHzOperationImplemented = TV_FALSE;
    MIBP.dot11PhyOFDMChannelWidth           = 0;

    /*************************************************************************/
    /*                     HR-DSS Table Initializations                      */
    /*************************************************************************/

    MIBP.dot11ShortPreambleOptionImplemented = TV_TRUE;
    MIBP.dot11PBCCOptionImplemented          = TV_FALSE;
    MIBP.dot11ChannelAgilityPresent          = TV_FALSE;
    MIBP.dot11ChannelAgilityEnabled          = TV_FALSE;
    MIBP.dot11HRCCAModeSupported             = 0;

    /*************************************************************************/
    /*                       ERP Table Initializations                       */
    /*************************************************************************/

    MIBP.dot11ERPBCCOptionImplemented        = TV_FALSE;
    MIBP.dot11ERPBCCOptionEnabled            = TV_FALSE;
    MIBP.dot11DSSSOFDMOptionImplemented      = TV_FALSE;
    MIBP.dot11DSSSOFDMOptionEnabled          = TV_FALSE;
    MIBP.dot11ShortSlotTimeOptionImplemented = TV_TRUE;
    MIBP.dot11ShortSlotOptionEnabled         = TV_TRUE;

    /*************************************************************************/
    /*                         HT Table Initializations                      */
    /*************************************************************************/

    MIBP.dot11FortyMHzOperationImplemented           = TV_FALSE;
    MIBP.dot11FortyMHzOperationEnabled               = TV_FALSE;
    MIBP.dot11CurrentPrimaryChannel                  = 0;
    MIBP.dot11CurrentSecondaryChannel                = 0;
    MIBP.dot11NumberOfSpatialStreamsImplemented      = 2;
    MIBP.dot11NumberOfSpatialStreamsEnabled          = 2;
    MIBP.dot11GreenfieldOptionImplemented            = TV_FALSE;
    MIBP.dot11GreenfieldOptionEnabled                = TV_FALSE;
    MIBP.dot11ShortGIOptionInTwentyImplemented       = TV_TRUE;
    MIBP.dot11ShortGIOptionInTwentyEnabled           = TV_FALSE;
    MIBP.dot11ShortGIOptionInFortyImplemented        = TV_FALSE;
    MIBP.dot11FortyMHzOperationImplemented           = TV_FALSE;
    MIBP.dot11FortyMHzOperationEnabled               = TV_FALSE;
    MIBP.dot11ShortGIOptionInFortyEnabled            = TV_FALSE;
    MIBP.dot11LDPCCodingOptionImplemented            = TV_FALSE;
    MIBP.dot11LDPCCodingOptionEnabled                = TV_FALSE;
    MIBP.dot11TxSTBCOptionImplemented                = TV_FALSE;
    MIBP.dot11TxSTBCOptionEnabled                    = TV_FALSE;
    MIBP.dot11RxSTBCOptionImplemented                = TV_FALSE;
    MIBP.dot11RxSTBCOptionEnabled                    = TV_FALSE;
    MIBP.dot11BeamFormingOptionImplemented           = TV_FALSE;
    MIBP.dot11BeamFormingOptionEnabled               = TV_FALSE;
    MIBP.dot11HighestSupportedDataRate               = 0;
    MIBP.dot11TxMCSSetDefined                        = TV_FALSE;
    MIBP.dot11TxRxMCSSetNotEqual                     = TV_FALSE;
    MIBP.dot11TxMaximumNumberSpatialStreamsSupported = 0;
    MIBP.dot11TxUnequalModulationSupported           = TV_FALSE;

    /*************************************************************************/
    /* Supported MCS Tx Table                                                */
    /*************************************************************************/

    mem_set(MIBP.dot11SupportedMCSTxValue.mcs_bmp, 0, MCS_BITMASK_LEN);
    MIBP.dot11SupportedMCSTxValue.num_mcs = update_supp_mcs_bitmap(
                                        MIBP.dot11SupportedMCSTxValue.mcs_bmp);

    /*************************************************************************/
    /* Supported MCS Rx Table                                                */
    /*************************************************************************/

    mem_set(MIBP.dot11SupportedMCSRxValue.mcs_bmp, 0, MCS_BITMASK_LEN);
    MIBP.dot11SupportedMCSRxValue.num_mcs = update_supp_mcs_bitmap(
                                        MIBP.dot11SupportedMCSRxValue.mcs_bmp);

    /*************************************************************************/
    /* TX Beamforming Table                                                 */
    /*************************************************************************/

    MIBP.dot11ReceiveStaggerSoundingOptionImplemented                 = TV_FALSE;
    MIBP.dot11TransmitStaggerSoundingOptionImplemented                = TV_FALSE;
    MIBP.dot11ReceiveNDPOptionImplemented                             = TV_FALSE;
    MIBP.dot11TransmitNDPOptionImplemented                            = TV_FALSE;
    MIBP.dot11ImplicitTxBFOptionImplemented                           = TV_FALSE;
    MIBP.dot11CalibrationOptionImplemented                            = 0;
    MIBP.dot11ExplicitCSITxBFOptionImplemented                        = TV_FALSE;
    MIBP.dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented = TV_FALSE;
    MIBP.dot11ExplicitBFCSIFeedbackOptionImplemented                          = 0;
    MIBP.dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented = 0;
    MIBP.dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented    = 0;
    MIBP.dot11NumberBeamFormingCSISupportAntenna                              = 0;
    MIBP.dot11NumberNonCompressedbeamformingMatrixSupportAntenna              = 0;
    MIBP.dot11NumberCompressedbeamformingMatrixSupportAntenna                 = 0;
    MIBP.dot11MaxCSIFeedbackDelay                                             = 0;

    /*************************************************************************/
    /* Initialize elements from Private PHY-MIB Structure                    */
    /*************************************************************************/
    P_MIBP.dsss_cck_40mhz_mode                                      = TV_TRUE;
    TROUT_FUNC_EXIT;
}

void mib_802_11n_print(void)
{
    UWORD32 i = 0;
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* mib 802.11n                                                           */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("\n");
	
    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Phy Operation Table - Dot11PhyOperationEntry                          */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	
	PRINT_MIB("MIBP.dot11PHYType          = %u\n",MIBP.dot11PHYType);/* HT */
    PRINT_MIB("MIBP.dot11CurrentRegDomain = 0x%08x\n",MIBP.dot11CurrentRegDomain);/* FCC(USA) */ 
    PRINT_MIB("MIBP.dot11TempType         = %u\n",MIBP.dot11TempType);/* Type 1, 0 to 40 degrees C */

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Phy Antenna Table - Dot11PhyAntennaEntry                              */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	
	PRINT_MIB("MIBP.dot11CurrentTxAntenna                                      = %u\n",MIBP.dot11CurrentTxAntenna);
    PRINT_MIB("MIBP.dot11DiversitySupport                                      = %u\n",MIBP.dot11DiversitySupport);
	PRINT_MIB("MIBP.dot11CurrentRxAntenna                                      = %u\n",MIBP.dot11CurrentRxAntenna);

	PRINT_MIB("MIBP.dot11AntennaSelectionOptionImplemented                     = %s\n",
		(MIBP.dot11AntennaSelectionOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitExplicitCSIFeedbackASOptionImplemented        = %s\n",
		(MIBP.dot11TransmitExplicitCSIFeedbackASOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitIndicesFeedbackASOptionImplemented            = %s\n",
		(MIBP.dot11TransmitIndicesFeedbackASOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ExplicitCSIFeedbackASOptionImplemented                = %s\n",
		(MIBP.dot11ExplicitCSIFeedbackASOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitIndicesComputationFeedbackASOptionImplemented = %s\n",
		(MIBP.dot11TransmitIndicesComputationFeedbackASOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	
	PRINT_MIB("MIBP.dot11ReceiveAntennaSelectionOptionImplemented              = %s\n",
		(MIBP.dot11ReceiveAntennaSelectionOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitSoundingPPDUOptionImplemented                 = %s\n",
		(MIBP.dot11TransmitSoundingPPDUOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Phy Tx Power Table - Dot11PhyTxPowerEntry                             */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIBP.dot11NumberSupportedPowerLevels = %u\n",MIBP.dot11NumberSupportedPowerLevels);

    for(i = 0; i < NUM_TX_POWER_LEVELS; i++)
    	PRINT_MIB("MIBP.dot11TxPowerLevel[%1d]           = %u\n",i,MIBP.dot11TxPowerLevel[i]);

    PRINT_MIB("MIBP.dot11CurrentTxPowerLevel        = %u\n",MIBP.dot11CurrentTxPowerLevel);
	
    /*************************************************************************/
    /*                     Phy FHSS Table Initializations                    */
    /*************************************************************************/

    /* NOTE: Currently unused and hence no access functions are provided     */

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Phy DHSS Table - Dot11PhyDSSSEntry                                    */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
    	PRINT_MIB("MIBP.dot11CurrentChannel             = CHANNEL%d\n",MIBP.dot11CurrentChannel+1);
	}
	else if(get_current_start_freq() == RC_START_FREQ_5)
	{
		if(MIBP.dot11CurrentChannel <= CHANNEL64)
		{
			PRINT_MIB("MIBP.dot11CurrentChannel             = CHANNEL%d\n",36 + (CHANNEL64 - CHANNEL36) * 4 );
		}
		else if(MIBP.dot11CurrentChannel <= CHANNEL140)
		{
			PRINT_MIB("MIBP.dot11CurrentChannel             = CHANNEL%d\n",100 + (CHANNEL140 - CHANNEL100) * 4 );
		}
		else if(MIBP.dot11CurrentChannel <= CHANNEL165)
		{
			PRINT_MIB("MIBP.dot11CurrentChannel             = CHANNEL%d\n",149 + (CHANNEL165 - CHANNEL149) * 4 );
		}
	}
    PRINT_MIB("MIBP.dot11CCAModeSupported           = %u\n",MIBP.dot11CCAModeSupported);        /* ED, CS, ED + CS */
    PRINT_MIB("MIBP.dot11CurrentCCAMode             = %u\n",MIBP.dot11CurrentCCAMode);        /* ED ONLY         */
    PRINT_MIB("MIBP.dot11EDThreshold                = %u\n",MIBP.dot11EDThreshold);

    /*************************************************************************/
    /*                     Phy IR Table Initializations                      */
    /*************************************************************************/

    /* NOTE: Currently unused and hence no access functions are provided     */

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Reg Domains Supported Table - Dot11RegDomainsSupportEntry             */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    PRINT_MIB("MIBP.dot11RegDomainsSupportValue = 0x%04x\n",MIBP.dot11RegDomainsSupportValue); /* FCC (USA) */

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Antenna List Table - Dot11AntennasListEntry                           */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    for(i = 0; i < NUM_ANTENNASLISTTABLE; i++)
    {
        PRINT_MIB("MIBP.dot11AntennasListTable[%d].dot11SupportedTxAntenna   = %s\n",i,
			(MIBP.dot11AntennasListTable[i].dot11SupportedTxAntenna == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

		PRINT_MIB("MIBP.dot11AntennasListTable[%d].dot11SupportedRxAntenna   = %s\n",i,
			(MIBP.dot11AntennasListTable[i].dot11SupportedRxAntenna == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

		PRINT_MIB("MIBP.dot11AntennasListTable[%d].dot11DiversitySelectionRx = %s\n",i,
			(MIBP.dot11AntennasListTable[i].dot11DiversitySelectionRx == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	}

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Supported Data rate Tx Table - Dot11SupportedDataRatesTxEntry         */\n");
    PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIBP.dot11SupportedDataRatesTxValue.num_rates = %u\n",
		MIBP.dot11SupportedDataRatesTxValue.num_rates);
	
    for(i = 0; i < NUM_DR_PHY_802_11G; i++)
	{
		PRINT_MIB("MIBP.dot11SupportedDataRatesTxValue.rates[%d] = %u\n",i,
			MIBP.dot11SupportedDataRatesTxValue.rates[i]);
    }

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Supported Data rate Rx Table - Dot11SupportedDataRatesRxEntry         */\n");
    PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIBP.dot11SupportedDataRatesRxValue.num_rates = %u\n",
		MIBP.dot11SupportedDataRatesRxValue.num_rates);
	
    for(i = 0; i < NUM_DR_PHY_802_11G; i++)
	{
		PRINT_MIB("MIBP.dot11SupportedDataRatesRxValue.rates[%2d] = %u\n",i,
			MIBP.dot11SupportedDataRatesRxValue.rates[i]);
    }
    
    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* OFDM Table (11a Attributes)                                           */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    PRINT_MIB("MIBP.dot11CurrentFrequency              = %u\n",MIBP.dot11CurrentFrequency);
    PRINT_MIB("MIBP.dot11TIThreshold                   = %u\n",MIBP.dot11TIThreshold);
    PRINT_MIB("MIBP.dot11FrequencyBandsSupported       = %u\n",MIBP.dot11FrequencyBandsSupported);
    PRINT_MIB("MIBP.dot11ChannelStartingFactor         = %u\n",MIBP.dot11ChannelStartingFactor);
    PRINT_MIB("MIBP.dot11FiveMHzOperationImplemented   = %s\n",
			(MIBP.dot11FiveMHzOperationImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11TenMHzOperationImplemented    = %s\n",
			(MIBP.dot11TenMHzOperationImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11TwentyMHzOperationImplemented = %s\n",
			(MIBP.dot11TwentyMHzOperationImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11PhyOFDMChannelWidth           = %u\n",MIBP.dot11PhyOFDMChannelWidth);

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* HR-DSS Table (11b Attributes)                                         */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    PRINT_MIB("MIBP.dot11ShortPreambleOptionImplemented = %s\n",
			(MIBP.dot11ShortPreambleOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11PBCCOptionImplemented          = %s\n",
			(MIBP.dot11PBCCOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11ChannelAgilityPresent          = %s\n",
			(MIBP.dot11ChannelAgilityPresent == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11ChannelAgilityEnabled          = %s\n",
			(MIBP.dot11ChannelAgilityEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    PRINT_MIB("MIBP.dot11HRCCAModeSupported             = %u\n",MIBP.dot11HRCCAModeSupported);

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* ERP Table (11g Attributes)                                            */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIBP.dot11ERPBCCOptionImplemented        = %s\n",
		(MIBP.dot11ERPBCCOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIBP.dot11ERPBCCOptionEnabled            = %s\n",
		(MIBP.dot11ERPBCCOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIBP.dot11DSSSOFDMOptionImplemented      = %s\n",
		(MIBP.dot11DSSSOFDMOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11DSSSOFDMOptionEnabled          = %s\n",
		(MIBP.dot11DSSSOFDMOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortSlotTimeOptionImplemented = %s\n",
		(MIBP.dot11ShortSlotTimeOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortSlotOptionEnabled         = %s\n",
		(MIBP.dot11ShortSlotOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ERPCCAModeSupported            = %u\n",MIBP.dot11ERPCCAModeSupported);

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* HT Table (11n Attributes)                                             */\n");
    PRINT_MIB("/*************************************************************************/\n");
		
	PRINT_MIB("MIBP.dot11FortyMHzOperationImplemented           = %s\n",
		(MIBP.dot11FortyMHzOperationImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11FortyMHzOperationEnabled               = %s\n",
		(MIBP.dot11FortyMHzOperationEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    PRINT_MIB("MIBP.dot11CurrentPrimaryChannel                  = %u\n",MIBP.dot11CurrentPrimaryChannel);
    PRINT_MIB("MIBP.dot11CurrentSecondaryChannel                = %u\n",MIBP.dot11CurrentSecondaryChannel);
    PRINT_MIB("MIBP.dot11NumberOfSpatialStreamsImplemented      = %u\n",MIBP.dot11NumberOfSpatialStreamsImplemented);
    PRINT_MIB("MIBP.dot11NumberOfSpatialStreamsEnabled          = %u\n",MIBP.dot11NumberOfSpatialStreamsEnabled);

	PRINT_MIB("MIBP.dot11GreenfieldOptionImplemented            = %s\n",
		(MIBP.dot11GreenfieldOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11GreenfieldOptionEnabled                = %s\n",
		(MIBP.dot11GreenfieldOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortGIOptionInTwentyImplemented       = %s\n",
		(MIBP.dot11ShortGIOptionInTwentyImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortGIOptionInTwentyEnabled           = %s\n",
		(MIBP.dot11ShortGIOptionInTwentyEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortGIOptionInFortyImplemented        = %s\n",
		(MIBP.dot11ShortGIOptionInFortyImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ShortGIOptionInFortyEnabled            = %s\n",
		(MIBP.dot11ShortGIOptionInFortyEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11LDPCCodingOptionImplemented            = %s\n",
		(MIBP.dot11LDPCCodingOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11LDPCCodingOptionEnabled                = %s\n",
		(MIBP.dot11LDPCCodingOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TxSTBCOptionImplemented                = %s\n",
		(MIBP.dot11TxSTBCOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TxSTBCOptionEnabled                    = %s\n",
		(MIBP.dot11TxSTBCOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11RxSTBCOptionImplemented                = %s\n",
		(MIBP.dot11RxSTBCOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11RxSTBCOptionEnabled                    = %s\n",
		(MIBP.dot11RxSTBCOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11BeamFormingOptionImplemented           = %s\n",
		(MIBP.dot11BeamFormingOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11BeamFormingOptionEnabled = %s\n",
		(MIBP.dot11BeamFormingOptionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIBP.dot11HighestSupportedDataRate               = %u\n",
		 MIBP.dot11HighestSupportedDataRate);

	PRINT_MIB("MIBP.dot11TxMCSSetDefined                        = %s\n",
		(MIBP.dot11TxMCSSetDefined == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TxRxMCSSetNotEqual                     = %s\n",
		(MIBP.dot11TxRxMCSSetNotEqual == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIBP.dot11TxMaximumNumberSpatialStreamsSupported = %d\n",
		 MIBP.dot11TxMaximumNumberSpatialStreamsSupported);

	PRINT_MIB("MIBP.dot11TxUnequalModulationSupported           = %s\n",
		(MIBP.dot11TxUnequalModulationSupported == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Supported MCS Tx Table                                                */\n");
    PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIBP.dot11SupportedMCSTxValue.num_mcs         = %d\n",
		MIBP.dot11SupportedMCSTxValue.num_mcs);
	for(i = 0; i < MCS_BITMASK_LEN ; i++)
	{
		PRINT_MIB("MIBP.dot11SupportedMCSTxValue.mcs_bmp[%d] = 0x%02x\n",i,
			MIBP.dot11SupportedMCSTxValue.mcs_bmp[i]);
	}
    
    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Supported MCS Rx Table                                                */\n");
    PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIBP.dot11SupportedMCSRxValue.num_mcs         = %u\n",
		MIBP.dot11SupportedMCSRxValue.num_mcs);
	for(i = 0; i < MCS_BITMASK_LEN ; i++)
	{
		PRINT_MIB("MIBP.dot11SupportedMCSRxValue.mcs_bmp[%d] = 0x%02x\n",i,
			MIBP.dot11SupportedMCSRxValue.mcs_bmp[i]);
	}

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* TX Beamforming Table                                                  */\n");
    PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIBP.dot11ReceiveStaggerSoundingOptionImplemented                         = %s\n",
		(MIBP.dot11ReceiveStaggerSoundingOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitStaggerSoundingOptionImplemented                        = %s\n",
		(MIBP.dot11TransmitStaggerSoundingOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ReceiveNDPOptionImplemented                                     = %s\n",
		(MIBP.dot11ReceiveNDPOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11TransmitNDPOptionImplemented                                    = %s\n",
		(MIBP.dot11TransmitNDPOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ImplicitTxBFOptionImplemented                                   = %s\n",
		(MIBP.dot11ImplicitTxBFOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIBP.dot11CalibrationOptionImplemented                                    = 0x%02x\n",
		 MIBP.dot11CalibrationOptionImplemented);
	
	PRINT_MIB("MIBP.dot11ExplicitCSITxBFOptionImplemented                                = %s\n",
		(MIBP.dot11ExplicitCSITxBFOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented         = %s\n",
		(MIBP.dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIBP.dot11ExplicitBFCSIFeedbackOptionImplemented                          = 0x%02x\n",
		MIBP.dot11ExplicitBFCSIFeedbackOptionImplemented);
	
	PRINT_MIB("MIBP.dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented = 0x%02x\n",
		MIBP.dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented);

	PRINT_MIB("MIBP.dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented    = 0x%02x\n",
		MIBP.dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented);

	PRINT_MIB("MIBP.dot11NumberBeamFormingCSISupportAntenna                              = 0x%02x\n",
		MIBP.dot11NumberBeamFormingCSISupportAntenna);

	PRINT_MIB("MIBP.dot11NumberNonCompressedbeamformingMatrixSupportAntenna              = 0x%02x\n",
		MIBP.dot11NumberNonCompressedbeamformingMatrixSupportAntenna);

	PRINT_MIB("MIBP.dot11NumberCompressedbeamformingMatrixSupportAntenna                 = 0x%02x\n",
		MIBP.dot11NumberCompressedbeamformingMatrixSupportAntenna);

	PRINT_MIB("MIBP.dot11MaxCSIFeedbackDelay                                             = %u\n",
		MIBP.dot11MaxCSIFeedbackDelay);

	PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* Private PHY-MIB Structure                                             */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("P_MIBP.dsss_cck_40mhz_mode                                                = %s\n",
		(P_MIBP.dsss_cck_40mhz_mode == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("\n");
}


#endif /* PHY_802_11n */
