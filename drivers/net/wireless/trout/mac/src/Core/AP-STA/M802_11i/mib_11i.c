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
/*  File Name         : mib_11i.c                                            */
/*                                                                           */
/*  Description       : This file contains the function definitions          */
/*                      related to intialization of various MIBs.            */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_11i                               */
/*                      check_auth_policy                                    */
/*                      check_pcip_policy                                    */
/*                      set_rsna_policy                                      */
/*                      mget_RSNAStatsIndex                                  */
/*                      get_pairwise_cipher_suite                            */
/*                      mnew_RSNAStats                                       */
/*                      mset_RSNAConfigGroupCipher                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "itypes.h"
#include "management_11i.h"
#include "mib_11i.h"
#include "mh.h"
#include "rsna_km.h"
#include "prot_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8    g_mode_802_11i             = 0;
mib_11i_t MIB_11I;

/*****************************************************************************/
/* Extern Variables                                                          */
/*****************************************************************************/

extern CIPHER_T g_pcip_policy_802_11i;

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_11i                                */
/*                                                                           */
/*  Description      : This function initializes all MIB entries.            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIBM                                                  */
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

void initialize_mac_mib_11i(void)
{
    UWORD8 i = 0;
    
	TROUT_FUNC_ENTER;
    mem_set(&MIB_11I, 0,  sizeof(mib_11i_t));

    MIB_11I.dot11RSNAOptionImplemented             = TV_TRUE;
    MIB_11I.dot11RSNAPreauthenticationImplemented  = TV_FALSE;

    /* Added from 802.11i */
    MIB_11I.dot11RSNAEnabled                       = TV_FALSE;
    MIB_11I.dot11RSNAPreauthenticationEnabled      = TV_FALSE;

    MIBM.dot11PrivacyOptionImplemented             = TV_TRUE;
    MIBM.dot11PrivacyInvoked                       = TV_FALSE;

    /* The value of the RSNA OUI as per the 802.11i standard */
    MIB_11I.dot11RSNAOUI[0] = 0x00;
    MIB_11I.dot11RSNAOUI[1] = 0x0F;
    MIB_11I.dot11RSNAOUI[2] = 0xAC;
    MIB_11I.dot11WPAOUI[0]  = 0x00;
    MIB_11I.dot11WPAOUI[1]  = 0x50;
    MIB_11I.dot11WPAOUI[2]  = 0xF2;

    /*************************************************************************/
    /* RSNA Config Table Related Initializations                             */
    /*************************************************************************/
    MIB_11I.dot11RSNAConfigVersion                 = 1;
    MIB_11I.dot11RSNAConfigPairwiseKeysSupported   = 2;

    /* Gourp Cipher Suite : OUI (3 octets (00-0F-AC) and the cipher suite    */
    /* The current deafult value of the gourp cipher suite is CCMP           */
    if(g_mode_802_11i & RSNA_802_11I)
    {
        MIB_11I.dot11RSNAConfigGroupCipher         = 0x04;
    }
    else if(g_mode_802_11i & WPA_802_11I)
    {
        MIB_11I.dot11RSNAConfigGroupCipher         = 0x02;
    }

    /* Group Rekey Method Takes the following values                         */
    /* Disabled    timeBased     packetBased   timePacketBased               */
    /*    1            2               3             4                       */
    /* Default is timeBased once per day                                     */
    MIB_11I.dot11RSNAConfigGroupRekeyMethod        = 2;

    /* The time is seconds after which the RSNA GTK should be refreshed      */
    /* This timer is to be used moment AFTER GTK is used via MLME-SETKEYS    */
    MIB_11I.dot11RSNAConfigGroupRekeyTime          = 86400;

    /* The number of packets(in 1000s) after which GTK should be refreshed   */
    /* The packet counter starts moment AFTER GTK is used via MLME-SETKEYS   */
    MIB_11I.dot11RSNAConfigGroupRekeyPackets       = 30;

    /* This object signals that the GTK shall be refreshed whenever a STA    */
    /* leaves the BSS that possesses the GTK                                 */
    MIB_11I.dot11RSNAConfigGroupRekeyStrict        = TV_FALSE;

    //qin.chen, 2013-1016, changed
	//  mset_RSNAConfigPSKPassPhrase("12345678", 0);
	mset_RSNAConfigPSKPassPhrase("\0", 0);

    /* This value is set in the RSNA Initialize code as this needs to be     */
    /* updated only after the SSID for the AP/STA has been obtained & set    */
    mem_set(MIB_11I.dot11RSNAConfigPSKValue, 0, 32);

    /* The number of times Message 1 in the RSNA Group Key Handshake will    */
    /* be retried per GTK Handshake attempt                                  */
    MIB_11I.dot11RSNAConfigGroupUpdateCount        = 3;

    /* The number of times Message 1 and Message 3 in RSNA 4-Way Handshake   */
    /* will be retried per 4-Way Handshake attempt                           */
    MIB_11I.dot11RSNAConfigPairwiseUpdateCount     = 3;

    /* Length of the group cipher key in bits                                */
    if(MIB_11I.dot11RSNAConfigGroupCipher      == 0x02)
    {
        /* The number of bits in the GTK for TKIP is 256 */
        MIB_11I.dot11RSNAConfigGroupCipherSize     = 256;
    }
    else if(MIB_11I.dot11RSNAConfigGroupCipher == 0x04)
    {
        /* The number of bits in the GTK for CCMP is 128 */
        MIB_11I.dot11RSNAConfigGroupCipherSize     = 128;
    }

    /* Maximum life time of the PMK in PMK Cache (in seconds)                */
    MIB_11I.dot11RSNAConfigPMKLifetime             = 43200;

    /* The percentage of the PMK lifetime that should expire before an       */
    /* IEEE 802.1X reauthentication occurs.                                  */
    MIB_11I.dot11RSNAConfigPMKReauthThreshold      = 70;

    /* Specifies the number of PTKSA replay counters per association         */
    /* 1 counter       2 counters         4 counters          16 counters    */
    /*    0                 1                  2                   3         */
    set_RSNAConfigNumberOfPTKSAReplayCounters_prot();

    /* The maximum time a security association shall take to set up (in secs)*/
    MIB_11I.dot11RSNAConfigSATimeout               = 60;

    /* The last AKM Suite selected                                           */
    /* Currently Configured for PSK                                          */
    MIB_11I.dot11RSNAAuthenticationSuiteSelected   = 0x02;

    /* The last Pairwise Cipher Suite selected                               */
    MIB_11I.dot11RSNAPairwiseCipherSelected        = 0x04;

    /* The last Group Cipher Suite selected                                  */
    MIB_11I.dot11RSNAGroupCipherSelected = MIB_11I.dot11RSNAConfigGroupCipher;


    mem_set(MIB_11I.dot11RSNAPMKIDUsed, 0, 16);
    MIB_11I.dot11RSNAAuthenticationSuiteRequested  = 0;
    MIB_11I.dot11RSNAPairwiseCipherRequested       = 0;
    MIB_11I.dot11RSNAGroupCipherRequested          = 0;

    /* Set TKIP Counter Measures Invoked to 0 */
    MIB_11I.dot11RSNATKIPCounterMeasuresInvoked    = 0;

    /* Counts the number of 4-Way Handshake failures                         */
    MIB_11I.dot11RSNA4WayHandshakeFailures         = 0;

    /* Specifies the number of GTKSA replay counters per association         */
    /* 1 counter  2 counters   4 replay counters 16 replay counters          */
    /*     0           1               2                  3                  */
    set_RSNAConfigNumberOfGTKSAReplayCounters_prot();

    /*************************************************************************/
    /* RSNAConfigPairwiseCipherConfigTable Related Initializations           */
    /*************************************************************************/
    /* Table Entry for CCMP & TKIP */
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].
        dot11RSNAConfigPairwiseCipherIndex = 1;
	MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].
		dot11RSNAConfigPairwiseCipher = 0x04;
	MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].
		dot11RSNAConfigPairwiseCipherSize = 128;
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].
        dot11RSNAConfigPairwiseCipherEnabled = TV_TRUE;


    /* Second Pairwise key Entry */
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].
        dot11RSNAConfigPairwiseCipherIndex = 2;
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].
		dot11RSNAConfigPairwiseCipher = 0x02;
	MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].
		dot11RSNAConfigPairwiseCipherSize = 256;
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].
        dot11RSNAConfigPairwiseCipherEnabled = TV_TRUE;

    /*************************************************************************/
    /* dot11RSNAConfigAuthenticationSuites TABLE Related Initialization      */
    /*************************************************************************/
    MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].
        dot11RSNAConfigAuthenticationSuiteIndex = 1;

    MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].
        dot11RSNAConfigAuthenticationSuite = 0x02;

    MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].
        dot11RSNAConfigAuthenticationSuiteEnabled  = TV_TRUE;


    /*************************************************************************/
    /* dot11RSNAStats TABLE                                                  */
    /*************************************************************************/
    /* Setting the entry for the Broadcast Multicast packets    */
    /* Using Key index zero for the Broadcast Multicast packets */
    mnew_RSNAStats(0);
    mem_set(MIB_11I.dot11RSNAStatsEntry[0].dot11RSNAStatsSTAAddress, 0xFF, 6);
    mset_RSNAStatsSelectedPairwiseCipher(mget_RSNAConfigGroupCipher(), 0);

    /* Reset the RSNA STAT handles */
    for(i = 1; i < (MAX_STA_SUPPORTED + 1); i++)
    {
        mnew_RSNAStats(i);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_auth_policy                                     */
/*                                                                           */
/*  Description      : This function checks for a given auth policy          */
/*                                                                           */
/*  Inputs           : Policy to be checked                                  */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Returns TRUE if the AKM policy is implemented and     */
/*                     currently enabled, else returns FALSE                 */
/*                                                                           */
/*  Outputs          : TRUE or FALSE                                         */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
BOOL_T check_auth_policy(UWORD8 policy)
{
    UWORD8 i = 0;
    for(i = 0; i < AUTHENTICATION_SUITES; i++)
    {
        /* If the Entry in the MIB is enabled, increment the counter */
        if(MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[i].
            dot11RSNAConfigAuthenticationSuiteEnabled == TV_TRUE)
        {
            if(MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[i].
                dot11RSNAConfigAuthenticationSuite == policy)
            {
                return BTRUE;
            }
        }
    }
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_pcip_policy                                     */
/*                                                                           */
/*  Description      : This function checks for a given pairwise policy      */
/*                                                                           */
/*  Inputs           : Policy to be checked                                  */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Returns TRUE if the Pairwise Cipher policy is         */
/*                     implemented & currently enabled, else returns FALSE   */
/*                                                                           */
/*  Outputs          : TRUE or FALSE                                         */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
BOOL_T check_pcip_policy(UWORD8 policy)
{
    UWORD8 i = 0;
    for(i = 0; i < PAIRWISE_CIPHER_SUITES; i++)
    {
        /* If the Entry in the MIB is enabled, increment the counter */
        if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
            dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE)
        {
            if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
                dot11RSNAConfigPairwiseCipher == policy)
            {
                return BTRUE;
            }
        }
    }
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : set_rsna_policy                                       */
/*                                                                           */
/*  Description      : This function sets enables/disables a given policy    */
/*                                                                           */
/*  Inputs           : 1) Policy to be set                                   */
/*                     2) The value to be set to TRUE/FALSE                  */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Checks for the given policy in the MIB array and      */
/*                     sets to TRUE or FALSE                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void set_rsna_policy(UWORD8 policy, TRUTH_VALUE_T val)
{
    UWORD8 i = 0;
    for(i = 0; i < PAIRWISE_CIPHER_SUITES; i++)
    {
        if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
                dot11RSNAConfigPairwiseCipher == policy)
        {
            /* If the Entry in the MIB is enabled, increment the counter */
            MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
                dot11RSNAConfigPairwiseCipherEnabled = val;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mget_RSNAStatsIndex                                   */
/*                                                                           */
/*  Description      : This function gets the RSNA Station Index for a STA   */
/*                                                                           */
/*  Inputs           : The address of the STA for which STA-index is needed  */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Checks in the entries and returns the station index   */
/*                     if it is found, else returns a max value 0xFF         */
/*                                                                           */
/*  Outputs          : Station Index                                         */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
UWORD8 mget_RSNAStatsIndex(UWORD8 *addr)
{
    UWORD8 i = 0;

    /* Search for the given Address in the MIB table */
    for(i = 0; i < (MAX_STA_SUPPORTED + 1); i++)
    {
        /* Return the index of the first matching entry */
        if(memcmp(MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress,
                  addr, 6) == 0)
        {
            break;
        }
    }
    if(i == (MAX_STA_SUPPORTED + 1))
    {
        i = 0xFF;
    }
    return i;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_pairwise_cipher_suite                             */
/*                                                                           */
/*  Description      : This function gets the current enabled P-Cipher Type  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Checks in the MIB Array for enabled cipher suites and */
/*                     returns the first one that it found                   */
/*                                                                           */
/*  Outputs          : The cypher suite value                                */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
UWORD8 get_pairwise_cipher_suite(void)
{
    UWORD8 i, ret;

    ret = 0;
    for(i = 0; i < PAIRWISE_CIPHER_SUITES; i++)
    {
        /* If the Entry in the MIB is enabled, increment the counter */
        if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[i].
            dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE)
        {
            ret ++;
        }
    }

    return ret;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mnew_RSNAStats                                        */
/*                                                                           */
/*  Description      : This function creates an RSNA entry for a new STA     */
/*                                                                           */
/*  Inputs           : Station Index for which RSNA entry is needed          */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Checks if an entry already exists, if not allocs mem  */
/*                     for the entry. Resets the memory and returns          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void mnew_RSNAStats(UWORD8 index)
{
    /* Reset all the counters and RNSA information */
    mem_set(&(MIB_11I.dot11RSNAStatsEntry[index]), 0,
            sizeof(dot11RSNAStatsEntry_t));

    /* Set the index of the Entry and the version number for the RSNA */
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsIndex   = index;
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsVersion = MIB_11I.
                                                        dot11RSNAConfigVersion;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mset_RSNAConfigGroupCipher                            */
/*                                                                           */
/*  Description      : This function sets RSNA group cypher type             */
/*                                                                           */
/*  Inputs           : Group Cypher Policy to be set to                      */
/*                                                                           */
/*  Globals          : MIB_11I                                               */
/*                                                                           */
/*  Processing       : Sets the Groups Cypher type and the Cypher size for   */
/*                     for this perticular cypher type.                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void mset_RSNAConfigGroupCipher(UWORD8 inp)
{
    MIB_11I.dot11RSNAConfigGroupCipher = inp;
    if(inp == 0x02)
    {
        /* The number of bits in the GTK for TKIP is 256 */
        MIB_11I.dot11RSNAConfigGroupCipherSize     = 256;
    }
    else if(inp == 0x04)
    {
        /* The number of bits in the GTK for CCMP is 128 */
        MIB_11I.dot11RSNAConfigGroupCipherSize     = 128;
    }
}

void mib_11i_print(void)
{
    UWORD8 i = 0;
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* mib 11i                                                               */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("\n");
	
	PRINT_MIB("MIB_11I.dot11RSNAOptionImplemented                 = %s\n",
		(MIB_11I.dot11RSNAOptionImplemented == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11I.dot11RSNAPreauthenticationImplemented      = %s\n",
		(MIB_11I.dot11RSNAPreauthenticationImplemented == BTRUE) ? "BTRUE" : "BFALSE");

    /* Added from 802.11i */
	PRINT_MIB("MIB_11I.dot11RSNAEnabled                           = %s\n",
		(MIB_11I.dot11RSNAEnabled == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11I.dot11RSNAPreauthenticationEnabled          = %s\n",
		(MIB_11I.dot11RSNAPreauthenticationEnabled == BTRUE) ? "BTRUE" : "BFALSE");

    /* The value of the RSNA OUI as per the 802.11i standard */
    PRINT_MIB("MIB_11I.dot11RSNAOUI                           = %02x-%02x-%02x\n"
																,MIB_11I.dot11RSNAOUI[0]
																,MIB_11I.dot11RSNAOUI[1]
																,MIB_11I.dot11RSNAOUI[2]);

    PRINT_MIB("MIB_11I.dot11WPAOUI                            = %02x-%02x-%02x\n"
																,MIB_11I.dot11WPAOUI[0]
																,MIB_11I.dot11WPAOUI[1]
																,MIB_11I.dot11WPAOUI[2]);


    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* RSNA Config Table - Dot11RSNAConfigTable (RSNA and TSN)               */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
	PRINT_MIB("MIB_11I.dot11RSNAConfigVersion                     = %u\n",
		MIB_11I.dot11RSNAConfigVersion);

	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseKeysSupported       = %u\n",
		MIB_11I.dot11RSNAConfigPairwiseKeysSupported);

    /* Gourp Cipher Suite : OUI (3 octets (00-0F-AC) and the cipher suite    */
    /* The current deafult value of the gourp cipher suite is CCMP           */
	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupCipher                 = 0x%02x\n",
		MIB_11I.dot11RSNAConfigGroupCipher);

    /* Group Rekey Method Takes the following values                         */
    /* Disabled    timeBased     packetBased   timePacketBased               */
    /*    1            2               3             4                       */
    /* Default is timeBased once per day                                     */

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupRekeyMethod            = %u\n",
		MIB_11I.dot11RSNAConfigGroupRekeyMethod);

    /* The time is seconds after which the RSNA GTK should be refreshed      */
    /* This timer is to be used moment AFTER GTK is used via MLME-SETKEYS    */

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupRekeyTime              = %u\n",
		MIB_11I.dot11RSNAConfigGroupRekeyTime);

    /* The number of packets(in 1000s) after which GTK should be refreshed   */
    /* The packet counter starts moment AFTER GTK is used via MLME-SETKEYS   */

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupRekeyPackets           = %u\n",
		MIB_11I.dot11RSNAConfigGroupRekeyPackets);
	
    /* This object signals that the GTK shall be refreshed whenever a STA    */
    /* leaves the BSS that possesses the GTK                                 */

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupRekeyStrict            = %s\n",
		(MIB_11I.dot11RSNAConfigGroupRekeyStrict == BTRUE) ? "BTRUE" : "BFALSE");

	PRINT_MIB("MIB_11I.dot11RSNAConfigPSKPassPhrase len           = %u\n",
		MIB_11I.dot11RSNAConfigPSKPassPhrase[0]);
    PRINT_MIB("MIB_11I.dot11RSNAConfigPSKPassPhrase               = \"%s\" \n",
		(char *)(&MIB_11I.dot11RSNAConfigPSKPassPhrase[1]));

    /* This value is set in the RSNA Initialize code as this needs to be     */
    /* updated only after the SSID for the AP/STA has been obtained & set    */
	PRINT_MIB("MIB_11I.dot11RSNAConfigPSKValue                    = ");
	for(i = 0 ; i < 32 ; i++)
	{
		PRINT_MIB("%02x ",MIB_11I.dot11RSNAConfigPSKValue[i]);
	}
	PRINT_MIB("\n");
	
    /* The number of times Message 1 in the RSNA Group Key Handshake will    */
    /* be retried per GTK Handshake attempt                                  */

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupUpdateCount            = %u\n",
		MIB_11I.dot11RSNAConfigGroupUpdateCount);

    /* The number of times Message 1 and Message 3 in RSNA 4-Way Handshake   */
    /* will be retried per 4-Way Handshake attempt                           */

	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseUpdateCount         = %u\n",
		MIB_11I.dot11RSNAConfigPairwiseUpdateCount);

	PRINT_MIB("MIB_11I.dot11RSNAConfigGroupCipherSize             = %u\n",
		MIB_11I.dot11RSNAConfigGroupCipherSize);

    /* Maximum life time of the PMK in PMK Cache (in seconds)                */
	
	PRINT_MIB("MIB_11I.dot11RSNAConfigPMKLifetime                 = %u\n",
		MIB_11I.dot11RSNAConfigPMKLifetime);

    /* The percentage of the PMK lifetime that should expire before an       */
    /* IEEE 802.1X reauthentication occurs.                                  */

	PRINT_MIB("MIB_11I.dot11RSNAConfigPMKReauthThreshold          = %u\n",
		MIB_11I.dot11RSNAConfigPMKReauthThreshold);

    /* Specifies the number of PTKSA replay counters per association         */
    /* 1 counter       2 counters         4 counters          16 counters    */
    /*    0                 1                  2                   3         */

	PRINT_MIB("MIB_11I.dot11RSNAConfigNumberOfPTKSAReplayCounters = %u\n",
		MIB_11I.dot11RSNAConfigNumberOfPTKSAReplayCounters);	

    /* The maximum time a security association shall take to set up (in secs)*/

	PRINT_MIB("MIB_11I.dot11RSNAConfigSATimeout                   = %u\n",
		MIB_11I.dot11RSNAConfigSATimeout);

    /* The last AKM Suite selected                                           */
    /* Currently Configured for PSK                                          */

	PRINT_MIB("MIB_11I.dot11RSNAAuthenticationSuiteSelected       = 0x%02x\n",
		MIB_11I.dot11RSNAAuthenticationSuiteSelected);

    /* The last Pairwise Cipher Suite selected                               */

	PRINT_MIB("MIB_11I.dot11RSNAPairwiseCipherSelected            = 0x%02x\n",
		MIB_11I.dot11RSNAPairwiseCipherSelected);

    /* The last Group Cipher Suite selected                                  */
	PRINT_MIB("MIB_11I.dot11RSNAGroupCipherSelected               = 0x%02x\n",
		MIB_11I.dot11RSNAGroupCipherSelected);

	PRINT_MIB("MIB_11I.dot11RSNAPMKIDUsed                         = ");
	for(i = 0 ; i < 16 ; i++)
	{
		PRINT_MIB("%02x ",MIB_11I.dot11RSNAPMKIDUsed[i]);
	}
	PRINT_MIB("\n");
	
	PRINT_MIB("MIB_11I.dot11RSNAAuthenticationSuiteRequested      = 0x%02x\n",
		MIB_11I.dot11RSNAAuthenticationSuiteRequested);
	PRINT_MIB("MIB_11I.dot11RSNAPairwiseCipherRequested           = 0x%02x\n",
		MIB_11I.dot11RSNAPairwiseCipherRequested);
	PRINT_MIB("MIB_11I.dot11RSNAGroupCipherRequested              = 0x%02x\n",
		MIB_11I.dot11RSNAGroupCipherRequested);

	PRINT_MIB("MIB_11I.dot11RSNAModeRequested                     = %u\n",
		MIB_11I.dot11RSNAModeRequested);

    /* Set TKIP Counter Measures Invoked to 0 */
	PRINT_MIB("MIB_11I.dot11RSNATKIPCounterMeasuresInvoked        = %u\n",
		MIB_11I.dot11RSNATKIPCounterMeasuresInvoked);

    /* Counts the number of 4-Way Handshake failures                         */
	PRINT_MIB("MIB_11I.dot11RSNA4WayHandshakeFailures             = %u\n",
		MIB_11I.dot11RSNA4WayHandshakeFailures);

    /* Specifies the number of GTKSA replay counters per association         */
    /* 1 counter  2 counters   4 replay counters 16 replay counters          */
    /*     0           1               2                  3                  */

	PRINT_MIB("MIB_11I.dot11RSNAConfigNumberOfGTKSAReplayCounters = %u\n",
		MIB_11I.dot11RSNAConfigNumberOfGTKSAReplayCounters);

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* RSNA Config Pair wise cipher Table - dot11RSNAConfigPairwiseCiphers   */\n");
    PRINT_MIB("/*************************************************************************/\n");
    /* Table Entry for CCMP & TKIP */
    PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherIndex             = %u\n",
    	MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherIndex);
	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipher                  = 0x%02x\n",
		MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipher);
	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherSize              = %u\n",
		MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherSize);
    PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherEnabled           = %s\n",
		(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[0].dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	PRINT_MIB("\n");
	
    /* Second Pairwise key Entry */
    PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherIndex             = %u\n",
    	MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherIndex);
	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipher                  = 0x%02x\n",
		MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipher);
	PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherSize              = %u\n",
		MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherSize);
    PRINT_MIB("MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherEnabled           = %s\n",
		(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[1].dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* dot11RSNAConfigAuthenticationSuites TABLE                             */\n");
    PRINT_MIB("/*************************************************************************/\n");
	
    PRINT_MIB("MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuiteIndex   = %u\n",
    	MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuiteIndex);

    PRINT_MIB("MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuite        = 0x%02x\n",
		MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuite);

	PRINT_MIB("MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuiteEnabled = %s\n",
		(MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[0].dot11RSNAConfigAuthenticationSuiteEnabled == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    PRINT_MIB("/*************************************************************************/\n");
    PRINT_MIB("/* dot11RSNAStats TABLE                                                  */\n");
    PRINT_MIB("/*************************************************************************/\n");
    /* Setting the entry for the Broadcast Multicast packets    */
    /* Using Key index zero for the Broadcast Multicast packets */
	for(i = 0;i < MAX_STA_SUPPORTED + 1;i++)
	{
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsIndex                  = %u\n",
			i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsIndex);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsSTAAddress             = %02x:%02x:%02x:%02x:%02x:%02x\n",i
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[0]
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[1]
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[2]
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[3]
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[4]
																						,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSTAAddress[5]);
		PRINT_MIB("dot11RSNAStatsEntry[%d].dot11RSNAStatsVersion = %d\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsVersion);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsSelectedPairwiseCipher = 0x%02x\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsSelectedPairwiseCipher);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsTKIPICVErrors          = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsTKIPICVErrors);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsTKIPLocalMICFailures   = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsTKIPLocalMICFailures);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsTKIPRemoteMICFailures  = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsTKIPRemoteMICFailures);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsCCMPReplays            = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsCCMPReplays);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsCCMPDecryptErrors      = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsCCMPDecryptErrors);
		PRINT_MIB("MIB_11I.dot11RSNAStatsEntry[%d].dot11RSNAStatsTKIPReplays            = %u\n",i,MIB_11I.dot11RSNAStatsEntry[i].dot11RSNAStatsTKIPReplays);
		PRINT_MIB("\n");
	}
}	


#endif  /* MAC_802_11I */
