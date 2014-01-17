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
/*  File Name         : mib_11e.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions related to MIB as   */
/*                      given in Annex-D of 802.11e standard.                */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_11e                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "mib_11e.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

mib_11e_t   MIB_11E;
p_mib_wmm_t P_MIB_WMM;

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_11e                                */
/*                                                                           */
/*  Description      : This function initializes all MIB entries for 802.11e */
/*                     including the private MIB.                            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIB_11E                                               */
/*                     P_MIB_WMM                                             */
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

void initialize_mac_mib_11e(void)
{
    UWORD8 i = 0;

	TROUT_FUNC_ENTER;
    mem_set(&MIB_11E, 0,  sizeof(mib_11e_t));
    mem_set(&P_MIB_WMM, 0,  sizeof(p_mib_wmm_t));

	//chenq mod MIB_11E.dot11QoSOptionImplemented TV_FALSE =>TV_TRUE
    //MIB_11E.dot11QoSOptionImplemented                      = TV_FALSE;
    MIB_11E.dot11QoSOptionImplemented                      = TV_TRUE;
	
    MIB_11E.dot11ImmediateBlockAckOptionImplemented        = TV_FALSE;
    MIB_11E.dot11DelayedBlockAckOptionImplemented          = TV_FALSE;
    MIB_11E.dot11DirectOptionImplemented                   = TV_FALSE;
    MIB_11E.dot11APSDOptionImplemented                     = TV_FALSE;
    MIB_11E.dot11QAckOptionImplemented                     = TV_TRUE;
    MIB_11E.dot11QBSSLoadOptionImplemented                 = TV_FALSE;
    MIB_11E.dot11QueueRequestOptionImplented               = TV_FALSE;
    MIB_11E.dot11TXOPRequestOptionImplemented              = TV_FALSE;
    MIB_11E.dot11MoreDataAckOptionImplemented              = TV_FALSE;
    MIB_11E.dot11AssociateInNQBSS                          = TV_FALSE;
    MIB_11E.dot11DLSAllowedInQBSS                          = TV_FALSE;
    MIB_11E.dot11DLSAllowed                                = TV_FALSE;
    MIB_11E.dot11CAPLimit                                  = 50;
    MIB_11E.dot11HCCWmin                                   = 0;
    MIB_11E.dot11HCCWmax                                   = 0;
    MIB_11E.dot11HCCAIFSN                                  = 1;
    MIB_11E.dot11ADDBAResponseTimeout                      = 1;
    MIB_11E.dot11ADDTSResponseTimeout                      = 1;
    MIB_11E.dot11ChannelUtilzationBeaconInterval           = 50;
    MIB_11E.dot11ScheduleTimeout                           = 10;
    MIB_11E.dot11DLSResponseTimeout                        = 10;
    MIB_11E.dot11QAPMissingAckRetryLimit                   = 1;
    MIB_11E.dot11EDCAAveragingPeriod                       = 5;
    MIB_11E.dot11QoSDiscardedFragmentCount                 = 0;
    MIB_11E.dot11AssociatedStationCount                    = 0;
    MIB_11E.dot11QoSCFPollsReceivedCount                   = 0;
    MIB_11E.dot11QoSCFPollsUnusableCount                   = 0;
    MIB_11E.dot11QoSCFPollsUnusedCount                     = 0;

    /* AC_BK index = 1 according to std */
    MIB_11E.EDCATable[0].dot11EDCATableIndex               = 1;
    MIB_11E.EDCATable[0].dot11EDCATableAIFSN               = 7;
    MIB_11E.EDCATable[0].dot11EDCATableCWmax               = get_cwmax();
    MIB_11E.EDCATable[0].dot11EDCATableCWmin               = get_cwmin();
    MIB_11E.EDCATable[0].dot11EDCATableMSDULifetime        = 500;
    MIB_11E.EDCATable[0].dot11EDCATableTXOPLimit           = 0;
    MIB_11E.EDCATable[0].dot11EDCATableMandatory           = TV_FALSE;

    /* AC_BE index = 0 according to std */
    MIB_11E.EDCATable[1].dot11EDCATableIndex               = 0;
    MIB_11E.EDCATable[1].dot11EDCATableAIFSN               = 3;
    MIB_11E.EDCATable[1].dot11EDCATableCWmax               = get_cwmax();
    MIB_11E.EDCATable[1].dot11EDCATableCWmin               = get_cwmin();
    MIB_11E.EDCATable[1].dot11EDCATableMSDULifetime        = 500;
    MIB_11E.EDCATable[1].dot11EDCATableTXOPLimit           = 0;
    MIB_11E.EDCATable[1].dot11EDCATableMandatory           = TV_FALSE;

    /* AC_VI index = 2 according to std */
    MIB_11E.EDCATable[2].dot11EDCATableIndex               = 2;
    MIB_11E.EDCATable[2].dot11EDCATableAIFSN               = 2;
    MIB_11E.EDCATable[2].dot11EDCATableCWmax               = get_cwmin();
    MIB_11E.EDCATable[2].dot11EDCATableCWmin               = get_cwmin() - 1;
    MIB_11E.EDCATable[2].dot11EDCATableMSDULifetime        = 500;
    MIB_11E.EDCATable[2].dot11EDCATableMandatory           = TV_FALSE;

    if(get_phy_type() == PHY_B_DSSS)
        MIB_11E.EDCATable[2].dot11EDCATableTXOPLimit       = 6016;
    else
        MIB_11E.EDCATable[2].dot11EDCATableTXOPLimit       = 3008;

    /* AC_VO index = 3 according to std */
    MIB_11E.EDCATable[3].dot11EDCATableIndex               = 3;
    MIB_11E.EDCATable[3].dot11EDCATableAIFSN               = 2;
    MIB_11E.EDCATable[3].dot11EDCATableCWmax               = get_cwmin() - 1;
    MIB_11E.EDCATable[3].dot11EDCATableCWmin               = get_cwmin() - 2;
    MIB_11E.EDCATable[3].dot11EDCATableMSDULifetime        = 500;
    MIB_11E.EDCATable[3].dot11EDCATableMandatory           = TV_FALSE;

    if(get_phy_type() == PHY_B_DSSS)
        MIB_11E.EDCATable[3].dot11EDCATableTXOPLimit       = 3264;
    else
        MIB_11E.EDCATable[3].dot11EDCATableTXOPLimit       = 1504;

    /* AC_BK = 1 */
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableIndex         = 1;
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableAIFSN         = 7;
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmax         = get_cwmax();
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmin         = get_cwmin();
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableTXOPLimit     = 0;
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMandatory     = TV_FALSE;
    MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMSDULifetime  = 500;

    /* AC_BE = 0 */
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableIndex         = 0;
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableAIFSN         = 3;
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmax         = get_cwmin() + 2;
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmin         = get_cwmin();
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableTXOPLimit     = 0;
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMandatory     = TV_FALSE;
    MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMSDULifetime  = 500;

    /* AC_VI = 2 */
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableIndex         = 2;
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableAIFSN         = 1;
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmax         = get_cwmin();
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmin         = get_cwmin() - 1;
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMandatory     = TV_FALSE;
    MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMSDULifetime  = 500;

    if(get_phy_type() == PHY_B_DSSS)
    {
        MIB_11E.QAPEDCATable[2].dot11QAPEDCATableTXOPLimit = 6016;
    }
    else
    {
        MIB_11E.QAPEDCATable[2].dot11QAPEDCATableTXOPLimit = 3008;
    }

    /* AC_VO = 3 */
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableIndex         = 3;
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableAIFSN         = 1;
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmax         = get_cwmin() - 1;
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmin         = get_cwmin() - 2;
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMandatory     = TV_FALSE;
    MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMSDULifetime  = 500;

    if(get_phy_type() == PHY_B_DSSS)
    {
        MIB_11E.QAPEDCATable[3].dot11QAPEDCATableTXOPLimit = 3264;
    }
    else
    {
        MIB_11E.QAPEDCATable[3].dot11QAPEDCATableTXOPLimit = 1504;
    }

    /* QoS counters */
    for(i = 0; i < NUM_TIDS; i++)
    {
        MIB_11E.QoSCounterTable[i].dot11QoSCountersIndex            = (i + 1);
        MIB_11E.QoSCounterTable[i].dot11QoSTransmittedFragmentCount = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSFailedCount              = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSRetryCount               = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSMultipleRetryCount       = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSFrameDuplicateCount      = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSRTSSuccessCount          = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSRTSFailureCount          = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSAckFailureCount          = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSReceivedFragmentCount    = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSTransmittedFrameCount    = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSDiscardedFrameCount      = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSMPDUsReceivedCount       = 0;
        MIB_11E.QoSCounterTable[i].dot11QoSRetriesReceivedCount     = 0;
    }

    /* Initialize the private WMM MIB */
    P_MIB_WMM.UAPSD_ap = BFALSE;
    P_MIB_WMM.ac_parameter_set_count = 0;
    TROUT_FUNC_EXIT;
}

void mib_11e_print(void)
{
    UWORD8 i = 0;

    PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* mib 11e                                                               */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("\n");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* MIB used to indicate whether QoS is implemented or not                */\n");
	PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11E.dot11QoSOptionImplemented                                = %s\n",
		(MIB_11E.dot11QoSOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Block Ack Option                                                      */\n");
	PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11E.dot11ImmediateBlockAckOptionImplemented                  = %s\n",
		(MIB_11E.dot11ImmediateBlockAckOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.dot11DelayedBlockAckOptionImplemented                    = %s\n",
		(MIB_11E.dot11DelayedBlockAckOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* DLS Option                                                            */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11DirectOptionImplemented                             = %s\n",
		(MIB_11E.dot11DirectOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Advanced Power Management Option                                      */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11APSDOptionImplemented                               = %s\n",
		(MIB_11E.dot11APSDOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Q Ack Option                                                          */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11QAckOptionImplemented                               = %s\n",
		(MIB_11E.dot11QAckOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Q BSS Load option                                                     */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11QBSSLoadOptionImplemented                           = %s\n",
		(MIB_11E.dot11QBSSLoadOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Queue request option                                                  */\n");
	PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11E.dot11QueueRequestOptionImplented                         = %s\n",
		(MIB_11E.dot11QueueRequestOptionImplented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* TXOP Request option                                                   */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11TXOPRequestOptionImplemented                        = %s\n",
		(MIB_11E.dot11TXOPRequestOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* More Data Ack Option                                                  */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11MoreDataAckOptionImplemented                        = %s\n",
		(MIB_11E.dot11MoreDataAckOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* MIB used to tell STA whether to join Non-Q BSS                        */\n");
	PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11E.dot11AssociateInNQBSS                                    = %s\n",
		(MIB_11E.dot11AssociateInNQBSS == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* DLS related MIBs                                                      */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("MIB_11E.dot11DLSAllowedInQBSS                                    = %s\n",
		(MIB_11E.dot11DLSAllowedInQBSS == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.dot11DLSAllowed                                          = %s\n",
		(MIB_11E.dot11DLSAllowed == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Addition to Operation Table                                           */\n");
	PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11E.dot11CAPLimit                                            = %u\n",
		MIB_11E.dot11CAPLimit);

	PRINT_MIB("MIB_11E.dot11HCCWmin                                             = %u\n",
		MIB_11E.dot11HCCWmin);

	PRINT_MIB("MIB_11E.dot11HCCWmax                                             = %u\n",
		MIB_11E.dot11HCCWmax);

	PRINT_MIB("MIB_11E.dot11HCCAIFSN                                            = %u\n",
		MIB_11E.dot11HCCAIFSN);

	PRINT_MIB("MIB_11E.dot11ADDBAResponseTimeout                                = %u\n",
		MIB_11E.dot11ADDBAResponseTimeout);

	PRINT_MIB("MIB_11E.dot11ADDTSResponseTimeout                                = %u\n",
		MIB_11E.dot11ADDTSResponseTimeout);

	PRINT_MIB("MIB_11E.dot11ChannelUtilzationBeaconInterval                     = %u\n",
		MIB_11E.dot11ChannelUtilzationBeaconInterval);

	PRINT_MIB("MIB_11E.dot11ScheduleTimeout                                     = %u\n",
		MIB_11E.dot11ScheduleTimeout);

	PRINT_MIB("MIB_11E.dot11DLSResponseTimeout                                  = %u\n",
		MIB_11E.dot11DLSResponseTimeout);

	PRINT_MIB("MIB_11E.dot11QAPMissingAckRetryLimit                             = %u\n",
		MIB_11E.dot11QAPMissingAckRetryLimit);

	PRINT_MIB("MIB_11E.dot11EDCAAveragingPeriod                                 = %u\n",
		MIB_11E.dot11EDCAAveragingPeriod);

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* Additions to Counters Table                                           */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11QoSDiscardedFragmentCount                           = %u\n",
		MIB_11E.dot11QoSDiscardedFragmentCount);

	PRINT_MIB("MIB_11E.dot11AssociatedStationCount                              = %u\n",
		MIB_11E.dot11AssociatedStationCount);

	PRINT_MIB("MIB_11E.dot11QoSCFPollsReceivedCount                             = %u\n",
		MIB_11E.dot11QoSCFPollsReceivedCount);

	PRINT_MIB("MIB_11E.dot11QoSCFPollsUnusedCount                               = %u\n",
		MIB_11E.dot11QoSCFPollsUnusedCount);

	PRINT_MIB("MIB_11E.dot11QoSCFPollsUnusableCount                             = %u\n",
		MIB_11E.dot11QoSCFPollsUnusableCount);

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* EDCA Config Table                                                     */\n");
	PRINT_MIB("/*************************************************************************/\n");

	PRINT_MIB("MIB_11E.dot11QoSEDCAParameterSetUpdateCount                      = %u\n",
		MIB_11E.dot11QoSEDCAParameterSetUpdateCount);

    /* AC_BK index = 1 according to std */
	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableIndex                         = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableIndex);

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableAIFSN                         = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableAIFSN);

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableCWmax                         = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableCWmax);

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableCWmin                         = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableCWmin);

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableMSDULifetime                  = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableMandatory                     = %s\n",
		(MIB_11E.EDCATable[0].dot11EDCATableMandatory == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11E.EDCATable[0].dot11EDCATableTXOPLimit                     = %u\n",
		MIB_11E.EDCATable[0].dot11EDCATableTXOPLimit);

    /* AC_BE index = 0 according to std */
	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableIndex                         = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableIndex);

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableAIFSN                         = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableAIFSN);

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableCWmax                         = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableCWmax);

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableCWmin                         = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableCWmin);

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableMSDULifetime                  = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableMandatory                     = %s\n",
		(MIB_11E.EDCATable[1].dot11EDCATableMandatory == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11E.EDCATable[1].dot11EDCATableTXOPLimit                     = %u\n",
		MIB_11E.EDCATable[1].dot11EDCATableTXOPLimit);

    /* AC_VI index = 2 according to std */
	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableIndex                         = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableIndex);

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableAIFSN                         = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableAIFSN);

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableCWmax                         = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableCWmax);

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableCWmin                         = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableCWmin);

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableMSDULifetime                  = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableMandatory                     = %s\n",
		(MIB_11E.EDCATable[2].dot11EDCATableMandatory == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11E.EDCATable[2].dot11EDCATableTXOPLimit                     = %u\n",
		MIB_11E.EDCATable[2].dot11EDCATableTXOPLimit);

    /* AC_VO index = 3 according to std */

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableIndex                         = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableIndex);

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableAIFSN                         = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableAIFSN);

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableCWmax                         = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableCWmax);

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableCWmin                         = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableCWmin);

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableMSDULifetime                  = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableMandatory                     = %s\n",
		(MIB_11E.EDCATable[3].dot11EDCATableMandatory == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11E.EDCATable[3].dot11EDCATableTXOPLimit                     = %u\n",
		MIB_11E.EDCATable[3].dot11EDCATableTXOPLimit);

    /* AC_BK = 1 */
	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableIndex                   = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableIndex);

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableAIFSN                   = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableAIFSN);

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmax                   = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmax);

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmin                   = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableCWmin);

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMandatory               = %s\n",
		(MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMandatory == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMSDULifetime            = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.QAPEDCATable[0].dot11QAPEDCATableTXOPLimit               = %u\n",
		MIB_11E.QAPEDCATable[0].dot11QAPEDCATableTXOPLimit);

    /* AC_BE = 0 */
	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableIndex                   = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableIndex);

	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableAIFSN                   = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableAIFSN);

	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmax                   = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmax);

	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmin                   = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableCWmin);

	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMandatory               = %s\n",
		(MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMandatory == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMSDULifetime            = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableTXOPLimit               = %u\n",
		MIB_11E.QAPEDCATable[1].dot11QAPEDCATableTXOPLimit);

    /* AC_VI = 2 */
	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableIndex                   = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableIndex);

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableAIFSN                   = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableAIFSN);

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmax                   = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmax);

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmin                   = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableCWmin);

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMandatory               = %s\n",
		(MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMandatory == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMSDULifetime            = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.QAPEDCATable[2].dot11QAPEDCATableTXOPLimit               = %u\n",
		MIB_11E.QAPEDCATable[2].dot11QAPEDCATableTXOPLimit);

    /* AC_VO = 3 */

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableIndex                   = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableIndex);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableAIFSN                   = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableAIFSN);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmax                   = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmax);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmin                   = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableCWmin);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMandatory               = %s\n",
		(MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMandatory == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMSDULifetime            = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableMSDULifetime);

	PRINT_MIB("MIB_11E.QAPEDCATable[3].dot11QAPEDCATableTXOPLimit               = %u\n",
		MIB_11E.QAPEDCATable[3].dot11QAPEDCATableTXOPLimit);

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* QoS counters                                                          */\n");
	PRINT_MIB("/*************************************************************************/\n");
    /* QoS counters */
    for(i = 0; i < NUM_TIDS; i++)
    {
		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSCountersIndex            = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSCountersIndex);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSTransmittedFragmentCount = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSTransmittedFragmentCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSFailedCount              = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSFailedCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSRetryCount               = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSRetryCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSMultipleRetryCount       = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSMultipleRetryCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSFrameDuplicateCount      = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSFrameDuplicateCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSRTSSuccessCount          = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSRTSSuccessCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSRTSFailureCount          = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSRTSFailureCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSAckFailureCount          = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSAckFailureCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSReceivedFragmentCount    = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSReceivedFragmentCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSTransmittedFrameCount    = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSTransmittedFrameCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSDiscardedFrameCount      = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSDiscardedFrameCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSMPDUsReceivedCount       = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSMPDUsReceivedCount);

		PRINT_MIB("MIB_11E.QoSCounterTable[%d].dot11QoSRetriesReceivedCount     = %u\n",i,
			MIB_11E.QoSCounterTable[i].dot11QoSRetriesReceivedCount);
    }

	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* the private WMM MIB                                                   */\n");
	PRINT_MIB("/*************************************************************************/\n");
    /* Initialize the private WMM MIB */

	PRINT_MIB("P_MIB_WMM.UAPSD_ap                                               = %s\n",
		(P_MIB_WMM.UAPSD_ap == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("P_MIB_WMM.ac_parameter_set_count                                 = %u\n",
		P_MIB_WMM.ac_parameter_set_count);
}





#endif /* MAC_WMM */

