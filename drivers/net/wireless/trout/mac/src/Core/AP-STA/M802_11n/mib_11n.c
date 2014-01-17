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
/*  File Name         : mib_11n.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions related to MIB as   */
/*                      given in Annex-D of 802.11n standard.                */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_11n                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "mib_11n.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

mib_11n_t MIB_11N;

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_11n                                */
/*                                                                           */
/*  Description      : This function initializes all MIB entries for 802.11n */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIB_11N                                               */
/*                                                                           */
/*  Processing       : All the MIB entries are initialized to their default  */
/*                     values.                                               */
/*                                                                           */
/*  Outputs          : The various entries of the global MIB structure are   */
/*                     set to default values.                                */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_mac_mib_11n(void)
{
	 TROUT_FUNC_ENTER;
     mem_set(&MIB_11N, 0,  sizeof(mib_11n_t));

     /************************************************************************/
     /* All mib values are the default values as mentioned in the standards  */
     /************************************************************************/

     MIB_11N.dot11HighThroughputOptionImplemented = TV_TRUE;

     /************************************************************************/
     /* HT Station Configuration Table - members of dot11StationConfigEntry  */
     /************************************************************************/

     MIB_11N.dot11MIMOPowerSave                       = 3;
     MIB_11N.dot11NDelayedBlockAckOptionImplemented   = TV_FALSE;
     MIB_11N.dot11MaxAMSDULength                      = 3839;
     MIB_11N.dot11PSMPOptionImplemented               = TV_FALSE;
     MIB_11N.dot11STBCControlFrameOptionImplemented   = TV_FALSE;
     MIB_11N.dot11LsigTxopProtectionOptionImplemented = TV_FALSE;

     /* current max RX AMPDU is 32KB, so the value is 2, ittiam original default is 3*/
     MIB_11N.dot11MaxRxAMPDUFactor                    = 2;	//modify by chengwg.
     
     MIB_11N.dot11MinimumMPDUStartSpacing             = 7; /* Changed from 5 to solve AMPDU+BA+WPA2-AES+TCPTx Issue */
     MIB_11N.dot11PCOOptionImplemented                = TV_FALSE;
     MIB_11N.dot11TransitionTime                      = 3;
     MIB_11N.dot11MCSFeedbackOptionImplemented        = 0;
     MIB_11N.dot11HTControlFieldSupported             = TV_FALSE;
     MIB_11N.dot11RDResponderOptionImplemented        = TV_FALSE;

    /************************************************************************/
    /* Operation Table - members of Dot11OperationEntry                     */
    /************************************************************************/

    MIB_11N.dot11HTOperatingMode               = 0;
    MIB_11N.dot11RIFSMode                      = TV_TRUE;
    MIB_11N.dot11PSMPControlledAccess          = TV_FALSE;
    MIB_11N.dot11ServiceIntervalGranularity    = 0;
    MIB_11N.dot11DualCTSProtection             = TV_FALSE;
    MIB_11N.dot11LSigTxopFullProtectionEnabled = TV_FALSE;
    MIB_11N.dot11NonGFEntitiesPresent          = TV_FALSE;
    MIB_11N.dot11PCOActivated                  = TV_FALSE;
    MIB_11N.dot11PCO40MaxDuration              = 60;
    MIB_11N.dot11PCO20MaxDuration              = 60;
    MIB_11N.dot11PCO40MinDuration              = 40;
    MIB_11N.dot11PCO20MinDuration              = 40;
    MIB_11N.dot11FortyMHzIntolerant            = TV_FALSE;
    /************************************************************************/
    /* Counters Table - members of Dot11CountersEntry                       */
    /************************************************************************/

    MIB_11N.dot11TransmittedAMSDUCount                  = 0;
    MIB_11N.dot11FailedAMSDUCount                       = 0;
    MIB_11N.dot11RetryAMSDUCount                        = 0;
    MIB_11N.dot11MultipleRetryAMSDUCount                = 0;
    MIB_11N.dot11AMSDUAckFailureCount                   = 0;
    MIB_11N.dot11ReceivedAMSDUCount                     = 0;
    MIB_11N.dot11TransmittedAMPDUCount                  = 0;
    MIB_11N.dot11TransmittedMPDUsInAMPDUCount           = 0;
    MIB_11N.dot11AMPDUReceivedCount                     = 0;
    MIB_11N.dot11MPDUInReceivedAMPDUCount               = 0;
    MIB_11N.dot11AMPDUDelimiterCRCErrorCount            = 0;
    MIB_11N.dot11ImplicitBARFailureCount                = 0;
    MIB_11N.dot11ExplicitBARFailureCount                = 0;
    MIB_11N.dot11ChannelWidthSwitchCount                = 0;
    MIB_11N.dot11TwentyMHzTransmittedFrameCount         = 0;
    MIB_11N.dot11FortyMHzTransmittedFrameCount          = 0;
    MIB_11N.dot11TwentyMHzReceivedFrameCount            = 0;
    MIB_11N.dot11FortyMHzReceivedFrameCount             = 0;
    MIB_11N.dot11PSMPSuccessCount                       = 0;
    MIB_11N.dot11PSMPFailureCount                       = 0;
    MIB_11N.dot11GrantedRDGUsedCount                    = 0;
    MIB_11N.dot11GrantedRDGUnusedCount                  = 0;
    MIB_11N.dot11TransmittedFramesInGrantedRDGCount     = 0;
    MIB_11N.dot11BeamformingFrameCount                  = 0;
    MIB_11N.dot11DualCTSSuccessCount                    = 0;
    MIB_11N.dot11DualCTSFailureCount                    = 0;
    MIB_11N.dot11STBCCTSSuccessCount                    = 0;
    MIB_11N.dot11STBCCTSFailureCount                    = 0;
    MIB_11N.dot11nonSTBCCTSFailureCount                 = 0;
    MIB_11N.dot11RTSLSIGSuccessCount                    = 0;
    MIB_11N.dot11RTSLSIGFailureCount                    = 0;
    MIB_11N.dot11TransmittedOctetsInAMSDU.lo_count      = 0;
    MIB_11N.dot11TransmittedOctetsInAMSDU.hi_count      = 0;
    MIB_11N.dot11ReceivedOctesInAMSDUCount.lo_count     = 0;
    MIB_11N.dot11ReceivedOctesInAMSDUCount.hi_count     = 0;
    MIB_11N.dot11TransmittedOctetsInAMPDUCount.lo_count = 0;
    MIB_11N.dot11TransmittedOctetsInAMPDUCount.hi_count = 0;
    MIB_11N.dot11ReceivedOctetsInAMPDUCount.lo_count    = 0;
    MIB_11N.dot11ReceivedOctetsInAMPDUCount.hi_count    = 0;
    MIB_11N.dot11TransmittedOctetsInGrantedRDG.lo_count = 0;
    MIB_11N.dot11TransmittedOctetsInGrantedRDG.hi_count = 0;
    TROUT_FUNC_EXIT;
}

void mib_11n_print(void)
{
	int i = 0;

     PRINT_MIB("/*************************************************************************/\n");
	 PRINT_MIB("/* mib 11n                                                               */\n");
	 PRINT_MIB("/*************************************************************************/\n");
	 PRINT_MIB("\n");

     /************************************************************************/
     /* All mib values are the default values as mentioned in the standards  */
     /************************************************************************/

	PRINT_MIB("MIB_11N.dot11HighThroughputOptionImplemented     = %s\n",
		(MIB_11N.dot11HighThroughputOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/************************************************************************/\n");
	PRINT_MIB("/* HT Station Configuration Table - members of dot11StationConfigEntry  */\n");
	PRINT_MIB("/************************************************************************/\n");

	PRINT_MIB("MIB_11N.dot11HTOperationalMCSSet.num_mcs         = %u\n",
		MIB_11N.dot11HTOperationalMCSSet.num_mcs);
	for(i = 0 ; i < MCS_BITMASK_LEN;i++)
	{
		PRINT_MIB("MIB_11N.dot11HTOperationalMCSSet.mcs_bmp[%2d]    = %02x\n",i,
			MIB_11N.dot11HTOperationalMCSSet.mcs_bmp[i]);
	}

	PRINT_MIB("MIB_11N.dot11MIMOPowerSave                       = %u\n",MIB_11N.dot11MIMOPowerSave);
	
	PRINT_MIB("MIB_11N.dot11NDelayedBlockAckOptionImplemented   = %s\n",
		(MIB_11N.dot11NDelayedBlockAckOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	
	PRINT_MIB("MIB_11N.dot11MaxAMSDULength                      = %u\n",MIB_11N.dot11MaxAMSDULength);

	PRINT_MIB("MIB_11N.dot11PSMPOptionImplemented               = %s\n",
		(MIB_11N.dot11PSMPOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11STBCControlFrameOptionImplemented   = %s\n",
		(MIB_11N.dot11STBCControlFrameOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11LsigTxopProtectionOptionImplemented = %s\n",
		(MIB_11N.dot11LsigTxopProtectionOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	/* current max RX AMPDU is 32KB, so the value is 2, ittiam original default is 3*/

	PRINT_MIB("MIB_11N.dot11MaxRxAMPDUFactor                    = %u\n",MIB_11N.dot11MaxRxAMPDUFactor);

	PRINT_MIB("MIB_11N.dot11MinimumMPDUStartSpacing             = %u\n",MIB_11N.dot11MinimumMPDUStartSpacing);

	PRINT_MIB("MIB_11N.dot11PCOOptionImplemented                = %s\n",
		(MIB_11N.dot11PCOOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11TransitionTime                      = %u\n",MIB_11N.dot11TransitionTime);

	PRINT_MIB("MIB_11N.dot11MCSFeedbackOptionImplemented        = %u\n",MIB_11N.dot11MCSFeedbackOptionImplemented);

	PRINT_MIB("MIB_11N.dot11HTControlFieldSupported             = %s\n",
		(MIB_11N.dot11HTControlFieldSupported == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11RDResponderOptionImplemented        = %s\n",
		(MIB_11N.dot11RDResponderOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/************************************************************************/\n");
	PRINT_MIB("/* Operation Table - members of Dot11OperationEntry                     */\n");
	PRINT_MIB("/************************************************************************/\n");

	PRINT_MIB("MIB_11N.dot11HTOperatingMode               = %u\n",MIB_11N.dot11HTOperatingMode);

	PRINT_MIB("MIB_11N.dot11RIFSMode                      = %s\n",
		(MIB_11N.dot11RIFSMode == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11PSMPControlledAccess          = %s\n",
		(MIB_11N.dot11PSMPControlledAccess == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11ServiceIntervalGranularity    = %u\n",MIB_11N.dot11ServiceIntervalGranularity);

	PRINT_MIB("MIB_11N.dot11DualCTSProtection             = %s\n",
		(MIB_11N.dot11DualCTSProtection == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11LSigTxopFullProtectionEnabled = %s\n",
		(MIB_11N.dot11LSigTxopFullProtectionEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11NonGFEntitiesPresent          = %s\n",
		(MIB_11N.dot11NonGFEntitiesPresent == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11PCOActivated                  = %s\n",
		(MIB_11N.dot11PCOActivated == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11N.dot11PCO40MaxDuration              = %u\n",MIB_11N.dot11PCO40MaxDuration);

	PRINT_MIB("MIB_11N.dot11PCO20MaxDuration              = %u\n",MIB_11N.dot11PCO20MaxDuration);

	PRINT_MIB("MIB_11N.dot11PCO40MinDuration              = %u\n",MIB_11N.dot11PCO40MinDuration);

	PRINT_MIB("MIB_11N.dot11PCO20MinDuration              = %u\n",MIB_11N.dot11PCO20MinDuration);

	PRINT_MIB("MIB_11N.dot11FortyMHzIntolerant            = %s\n",
		(MIB_11N.dot11FortyMHzIntolerant == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	PRINT_MIB("/************************************************************************/\n");
	PRINT_MIB("/* Counters Table - members of Dot11CountersEntry                       */\n");
	PRINT_MIB("/************************************************************************/\n");
	
	PRINT_MIB("MIB_11N.dot11TransmittedAMSDUCount                  = %u\n",MIB_11N.dot11TransmittedAMSDUCount);

	PRINT_MIB("MIB_11N.dot11FailedAMSDUCount                       = %u\n",MIB_11N.dot11FailedAMSDUCount);

	PRINT_MIB("MIB_11N.dot11RetryAMSDUCount                        = %u\n",MIB_11N.dot11RetryAMSDUCount);

	PRINT_MIB("MIB_11N.dot11MultipleRetryAMSDUCount                = %u\n",MIB_11N.dot11MultipleRetryAMSDUCount);

	PRINT_MIB("MIB_11N.dot11AMSDUAckFailureCount                   = %u\n",MIB_11N.dot11AMSDUAckFailureCount);

	PRINT_MIB("MIB_11N.dot11ReceivedAMSDUCount                     = %u\n",MIB_11N.dot11ReceivedAMSDUCount);

	PRINT_MIB("MIB_11N.dot11TransmittedAMPDUCount                  = %u\n",MIB_11N.dot11TransmittedAMPDUCount);

	PRINT_MIB("MIB_11N.dot11TransmittedMPDUsInAMPDUCount           = %u\n",MIB_11N.dot11TransmittedMPDUsInAMPDUCount);

	PRINT_MIB("MIB_11N.dot11AMPDUReceivedCount                     = %u\n",MIB_11N.dot11AMPDUReceivedCount);

	PRINT_MIB("MIB_11N.dot11MPDUInReceivedAMPDUCount               = %u\n",MIB_11N.dot11MPDUInReceivedAMPDUCount);

	PRINT_MIB("MIB_11N.dot11AMPDUDelimiterCRCErrorCount            = %u\n",MIB_11N.dot11AMPDUDelimiterCRCErrorCount);

	PRINT_MIB("MIB_11N.dot11ImplicitBARFailureCount                = %u\n",MIB_11N.dot11ImplicitBARFailureCount);

	PRINT_MIB("MIB_11N.dot11ExplicitBARFailureCount                = %u\n",MIB_11N.dot11ExplicitBARFailureCount);

	PRINT_MIB("MIB_11N.dot11ChannelWidthSwitchCount                = %u\n",MIB_11N.dot11ChannelWidthSwitchCount);

	PRINT_MIB("MIB_11N.dot11TwentyMHzTransmittedFrameCount         = %u\n",MIB_11N.dot11TwentyMHzTransmittedFrameCount);

	PRINT_MIB("MIB_11N.dot11FortyMHzTransmittedFrameCount          = %u\n",MIB_11N.dot11FortyMHzTransmittedFrameCount);

	PRINT_MIB("MIB_11N.dot11TwentyMHzReceivedFrameCount            = %u\n",MIB_11N.dot11TwentyMHzReceivedFrameCount);

	PRINT_MIB("MIB_11N.dot11FortyMHzReceivedFrameCount             = %u\n",MIB_11N.dot11FortyMHzReceivedFrameCount);

	PRINT_MIB("MIB_11N.dot11PSMPSuccessCount                       = %u\n",MIB_11N.dot11PSMPSuccessCount);

	PRINT_MIB("MIB_11N.dot11PSMPFailureCount                       = %u\n",MIB_11N.dot11PSMPFailureCount);

	PRINT_MIB("MIB_11N.dot11GrantedRDGUsedCount                    = %u\n",MIB_11N.dot11GrantedRDGUsedCount);

	PRINT_MIB("MIB_11N.dot11GrantedRDGUnusedCount                  = %u\n",MIB_11N.dot11GrantedRDGUnusedCount);

	PRINT_MIB("MIB_11N.dot11TransmittedFramesInGrantedRDGCount     = %u\n",MIB_11N.dot11TransmittedFramesInGrantedRDGCount);

	PRINT_MIB("MIB_11N.dot11BeamformingFrameCount                  = %u\n",MIB_11N.dot11BeamformingFrameCount);

	PRINT_MIB("MIB_11N.dot11DualCTSSuccessCount                    = %u\n",MIB_11N.dot11DualCTSSuccessCount);

	PRINT_MIB("MIB_11N.dot11DualCTSFailureCount                    = %u\n",MIB_11N.dot11DualCTSFailureCount);

	PRINT_MIB("MIB_11N.dot11STBCCTSSuccessCount                    = %u\n",MIB_11N.dot11STBCCTSSuccessCount);

	PRINT_MIB("MIB_11N.dot11STBCCTSFailureCount                    = %u\n",MIB_11N.dot11STBCCTSFailureCount);

	PRINT_MIB("MIB_11N.dot11nonSTBCCTSFailureCount                 = %u\n",MIB_11N.dot11nonSTBCCTSFailureCount);

	PRINT_MIB("MIB_11N.dot11RTSLSIGSuccessCount                    = %u\n",MIB_11N.dot11RTSLSIGSuccessCount);

	PRINT_MIB("MIB_11N.dot11RTSLSIGFailureCount                    = %u\n",MIB_11N.dot11RTSLSIGFailureCount);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInAMSDU.lo_count      = %u\n",MIB_11N.dot11TransmittedOctetsInAMSDU.lo_count);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInAMSDU.hi_count      = %u\n",MIB_11N.dot11TransmittedOctetsInAMSDU.hi_count);

	PRINT_MIB("MIB_11N.dot11ReceivedOctesInAMSDUCount.lo_count     = %u\n",MIB_11N.dot11ReceivedOctesInAMSDUCount.lo_count);

	PRINT_MIB("MIB_11N.dot11ReceivedOctesInAMSDUCount.hi_count     = %u\n",MIB_11N.dot11ReceivedOctesInAMSDUCount.hi_count);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInAMPDUCount.lo_count = %u\n",MIB_11N.dot11TransmittedOctetsInAMPDUCount.lo_count);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInAMPDUCount.hi_count = %u\n",MIB_11N.dot11TransmittedOctetsInAMPDUCount.hi_count);

	PRINT_MIB("MIB_11N.dot11ReceivedOctetsInAMPDUCount.lo_count    = %u\n",MIB_11N.dot11ReceivedOctetsInAMPDUCount.lo_count);

	PRINT_MIB("MIB_11N.dot11ReceivedOctetsInAMPDUCount.hi_count    = %u\n",MIB_11N.dot11ReceivedOctetsInAMPDUCount.hi_count);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInGrantedRDG.lo_count = %u\n",MIB_11N.dot11TransmittedOctetsInGrantedRDG.lo_count);

	PRINT_MIB("MIB_11N.dot11TransmittedOctetsInGrantedRDG.hi_count = %u\n",MIB_11N.dot11TransmittedOctetsInGrantedRDG.hi_count);
}

#endif /* MAC_802_11N */
