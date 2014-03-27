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
/*  File Name         : mib.c                                                */
/*                                                                           */
/*  Description       : This file contains the function definitions          */
/*                      related to intialization of various MIBs.            */
/*                                                                           */
/*  List of Functions : initialize_mac_mib                                   */
/*                      initialize_private_mib                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management.h"
#include "mib.h"
#include "mh.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

mib_t   MIBM;   /* Management Information Base */
p_mib_t P_MIB;   /* Private MIB                */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
p_wapi_mib_t P_WAPI_MIB;
#endif

int strmac2hex(unsigned char hexmac[], char *strmac)
{
    unsigned char i, temp8, *ptr8;
    
    ptr8 = (unsigned char *)strmac;
    for(i=0; i<6; i++)
    {
        temp8 = *ptr8++;
        if(temp8 >= '0' && temp8 <= '9')
        {
            hexmac[i] = (temp8 - '0') << 4;
        }
        else if((temp8 >= 'a' && temp8 <= 'f') || (temp8 >= 'A' && temp8 <= 'F'))
        {
            hexmac[i] = (((temp8 - 'a') + 10) & 0x0F) << 4;
        }
        else
        {
            return -1;
        }

		
        //low 4bits of char.
        temp8 = *ptr8++;
        if(temp8 >= '0' && temp8 <= '9')
        {
            hexmac[i] |= (temp8 - '0') ;
        }
        else if((temp8 >= 'a' && temp8 <= 'f') || (temp8 >= 'A' && temp8 <= 'F'))
        {
            hexmac[i] |= ((temp8 - 'a') + 10) & 0x0F;
        }
        else
        {
            return -1;
        }
    
        //skip separator.
        ptr8++;  
    }

    return 0;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib                                    */
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
extern char *usrmac;

void initialize_mac_mib(void)
{
    UWORD32 i = 0;
    // set different mac address between SDIO INTERFACE and SPI INTERFACE
#ifndef TROUT_SDIO_INTERFACE
#ifdef BSS_ACCESS_POINT_MODE
    static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x16 };
#else /* BSS_ACCESS_POINT_MODE */
    static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x17 };
#endif /* BSS_ACCESS_POINT_MODE */
#else
#ifdef BSS_ACCESS_POINT_MODE
    //static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x58 };	//test
    static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x82 };
#else /* BSS_ACCESS_POINT_MODE */
    //static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x5A };	//test
    static UWORD8  station_id[6] = { 0x02, 0x50, 0xC2, 0x5E, 0x10, 0x83 };
#endif /* BSS_ACCESS_POINT_MODE */
#endif
    //Hugh: add for debug.
    char machex[6] = {0};

	TROUT_FUNC_ENTER;

    if(usrmac != NULL)
    {
        if((strmac2hex(machex, usrmac) == 0) && (machex[0] & 1) == 0)
        {
            TROUT_DBG4("user set mac: %02x:%02x:%02x:%02x:%02x:%02x\n", machex[0],
                                                                    machex[1],
                                                                    machex[2],
                                                                    machex[3],
                                                                    machex[4],
                                                                    machex[5]
                                                                    );
            memcpy(station_id, machex, 6);
        }
    }
    //----------------------

    /*************************************************************************/
    /*          SMT Station Configuration Table Initializations              */
    /*************************************************************************/
#ifdef MAC_ADDRESS_FROM_FLASH
    static UWORD8 init_flag = 1;
    UWORD8  mac_address[8] = {0};

    /* Read the MAC address from EEPROM */
    if  ( init_flag == 1 )
    {
        /* Copy the address from Flash */
        memcpy(mac_address, (UWORD8 *)(g_virt_flash_base + FLASH_MAC_ADDRESS_OFFSET), 8);

        /* Validate the sanity of the read address */
        if ((mac_address[0] == FLASH_MAC_ADDR_ID1) &&
            (mac_address[1] == FLASH_MAC_ADDR_ID2))
        {
            /* Replace station_id with read address */
            memcpy(station_id, mac_address+2, 6);

            /* Do not read from SDRAM */
            g_addr_sdram = 0;
        }

    }
    init_flag = 0;

    TROUT_DBG4("MAC-Address=%x:%x:%x:%x:%x:%x\n",station_id[0],
    station_id[1] ,station_id[2],station_id[3],station_id[4],station_id[5]);

#endif /* MAC_ADDRESS_FROM_FLASH */

    /* Zero Initialize the MIB structure */
    mem_set(&MIBM, 0, sizeof(mib_t));

    /* Read the default value or from SDRAM depending on the setting */
    if(g_addr_sdram != 1)
    {
        memcpy(MIBM.dot11StationID, station_id, 6);
    }
    else
    {
        printk("FixMe: dot11StationID NOT set.\n");
        //memcpy(MIBM.dot11StationID, (UWORD8*)MAC_ADDRESS_LOCATION, 6); //modified by Hugh.
    }

    MIBM.dot11MediumOccupancyLimit          = 100;        /* 100ms           */
    MIBM.dot11CFPollable                    = TV_FALSE;
    MIBM.dot11CFPPeriod                     = 0;
    MIBM.dot11CFPMaxDuration                = 0;
    MIBM.dot11AuthenticationResponseTimeOut = 163; //wzl fix 163ms for coex retransmit// chenq fix 512; //dumy fix 1000 //512;        /* 512ms           */
    MIBM.dot11PrivacyOptionImplemented      = TV_FALSE;   /* Shared Key Auth */
#ifdef IBSS_BSS_STATION_MODE
    MIBM.dot11PowerManagementMode           = MIB_ACTIVE; /* Active Mode     */
#endif /* IBSS_BSS_STATION_MODE */

//chenq mod 0727
    memset(MIBM.dot11DesiredSSID,0x00,MAX_SSID_LEN);
#if 0
    MIBM.dot11DesiredSSID[0]                = 'w';        /* SSID: wifi      */
    MIBM.dot11DesiredSSID[1]                = 'i';
    MIBM.dot11DesiredSSID[2]                = 'f';
    MIBM.dot11DesiredSSID[3]                = 'i';
    MIBM.dot11DesiredSSID[4]                = '\0';
#endif
    MIBM.dot11DesiredBSSType                = INFRASTRUCTURE;

    MIBM.dot11BeaconPeriod                  = 100;        /* 100ms           */
    MIBM.dot11DTIMPeriod                    = 3;
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
    /* For Wi-Fi certification of P2P feature the DTIM period must be set to */
    /* 1. This is for Test Case 6.1.12 which checks that the TIM MC/BC bit   */
    /* is set in the immediate next beacon.                                  */
    MIBM.dot11DTIMPeriod = 1;
#endif /* MAC_P2P */
    MIBM.dot11AssociationResponseTimeOut    = 163; //wzl fix 163ms for coex retransmit// chenq fix 512; //dumy fix 1000  //512;        /* 512ms           */
    MIBM.dot11DisassociateReason            = 0;
    mem_set(MIBM.dot11DisassociateStation, 0, 6);
    MIBM.dot11DeauthenticateReason          = 0;
    mem_set(MIBM.dot11DeauthenticateStation, 0, 6);
    MIBM.dot11AuthenticateFailStatus        = 0;
    mem_set(MIBM.dot11AuthenticateFailStation, 0, 6);

    /*************************************************************************/
    /*          Authentication Algorithms Table Initializations              */
    /*************************************************************************/

    MIBM.dot11AuthenticationAlgorithmsTable[0].
                            dot11AuthenticationAlgorithm = OPEN_SYSTEM;
    MIBM.dot11AuthenticationAlgorithmsTable[0].
                            dot11AuthenticationAlgorithmsEnable = TV_FALSE;

    MIBM.dot11AuthenticationAlgorithmsTable[1].
                            dot11AuthenticationAlgorithm = SHARED_KEY;
    MIBM.dot11AuthenticationAlgorithmsTable[1].
                            dot11AuthenticationAlgorithmsEnable = TV_FALSE;

    for(i = 2; i < NUM_DOT11AUTHENTICATIONALGORITHMS; i++)
    {
        MIBM.dot11AuthenticationAlgorithmsTable[i].
                                dot11AuthenticationAlgorithm = OPEN_SYSTEM;
        MIBM.dot11AuthenticationAlgorithmsTable[i].
                                dot11AuthenticationAlgorithmsEnable = TV_FALSE;
    }

    /*************************************************************************/
    /*             WEP Default Keys Table Initializations                    */
    /*************************************************************************/

    /* Initialize WEP keys */
    for(i = 0; i < NUM_DOT11WEPDEFAULTKEYVALUE; i++)
    {
        /* Initialize the whole Keys to all zeros  */
        mem_set(MIBM.dot11WEPDefaultKeyValue[i], 0, MAX_WEP_STR_SIZE);

        /* Initialize the Sizes to WEP-40  */
        MIBM.dot11WEPDefaultKeyValue[i][WEP_SIZE_OFFSET] = 40;
    }

    /*************************************************************************/
    /*             WEP Kep Mappings Table Initializations                    */
    /*************************************************************************/

    for(i = 0; i < NUM_DOT11WEPKEYMAPPINGSTABLE; i++)
    {
        mem_set(MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingValue,
               0, MAX_WEP_STR_SIZE);

        mem_set(MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingAddress, 0, 6);

        MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingWEPOn = TV_FALSE;
    }

    /*************************************************************************/
    /*                 Privacy Table Initializations                         */
    /*************************************************************************/

    MIBM.dot11PrivacyInvoked                 = TV_FALSE;
    MIBM.dot11WEPDefaultKeyID                = 0;
    MIBM.dot11WEPKeyMappingLength            = 10;
    MIBM.dot11ExcludeUnencrypted             = TV_TRUE;
    MIBM.dot11WEPICVErrorCount               = 0;
    MIBM.dot11WEPExcludedCount               = 0;

    /* MAC_802_11_WEP */
    MIBM.dot11PrivacyOptionImplemented       = TV_TRUE;
    MIBM.dot11PrivacyInvoked                 = TV_FALSE;
    /* MAC_802_11_WEP */


    /*************************************************************************/
    /*                 SMT Notification Initializations                      */
    /*************************************************************************/

    MIBM.dot11Disassociate.reason            = 0;
    mem_set(MIBM.dot11Disassociate.station, 0, 6);

    MIBM.dot11Deauthenticate.reason          = 0;
    mem_set(MIBM.dot11Deauthenticate.station, 0, 6);

    MIBM.dot11AuthenticateFail.reason        = 0;
    mem_set(MIBM.dot11AuthenticateFail.station, 0, 6);

    /*************************************************************************/
    /*                 Operation Table Initializations                       */
    /*************************************************************************/

    if(g_addr_sdram != 1)
    {
        memcpy(MIBM.dot11MACAddress, station_id, 6);
    }
    else
    {
        printk("FixMe: dot11MACAddress NOT set.\n");
        //memcpy(MIBM.dot11MACAddress, (UWORD8*)MAC_ADDRESS_LOCATION, 6); //modified by Hugh.
    }
    MIBM.dot11RTSThreshold                   = 3000;//dumy fix 3000 //2347;

    MIBM.dot11ShortRetryLimit                = 7;

    MIBM.dot11LongRetryLimit                 = 4;
    MIBM.dot11FragmentationThreshold         = 2346;
    MIBM.dot11MaxTransmitMSDULifetime        = 512;
    MIBM.dot11MaxReceiveLifetime             = 512;
    MIBM.dot11ManufacturerID[0]              = '\0';
    MIBM.dot11ProductID[0]                   = '\0';

    /*************************************************************************/
    /*                  Counters Table Initializations                       */
    /*************************************************************************/

    MIBM.dot11TransmittedFragmentCount       = 0;
    MIBM.dot11MulticastTransmittedFrameCount = 0;
    MIBM.dot11FailedCount                    = 0;
    MIBM.dot11RetryCount                     = 0;
    MIBM.dot11MultipleRetryCount             = 0;
    MIBM.dot11FrameDuplicateCount            = 0;
    MIBM.dot11RTSSuccessCount                = 0;
    MIBM.dot11RTSFailureCount                = 0;
    MIBM.dot11ACKFailureCount                = 0;
    MIBM.dot11ReceivedFragmentCount          = 0;
    MIBM.dot11MulticastReceivedFrameCount    = 0;
    MIBM.dot11FCSErrorCount                  = 0;
    MIBM.dot11TransmittedFrameCount          = 0;
    MIBM.dot11WEPUndecryptableCount          = 0;

    /*************************************************************************/
    /*             Group Addresses Table Initializations                     */
    /*************************************************************************/

    mem_set(MIBM.dot11GroupAddressTable, 0, NUM_DOT11GRPADDRESS * 6 + 1);

    /*************************************************************************/
    /*        Resource Type Attribute Templates Initializations              */
    /*************************************************************************/

    /* NOTE: currently unused and hence no initializations are provided for  */
    /* the dot11ResourceTypeIDName                                           */

    /*************************************************************************/
    /*                     Resource info Table Initializations               */
    /*************************************************************************/

    MIBM.dot11manufacturerOUI = VENDOR_ID;

    for(i = 0; i < NUM_DOT11MANUFACTURERNAME; i++)
        MIBM.dot11manufacturerName[i] = 0;

    MIBM.dot11manufacturerProductName = PRODUCT_ID;

    strcpy((WORD8*)MIBM.dot11manufacturerProductVersion,
            (const WORD8*)(MAC_FW_VERSION));

    /*************************************************************************/
    /*                  Spectrum Management Initializations                  */
    /*************************************************************************/

    MIBM.dot11SpectrumManagementImplemented = TV_FALSE;
    MIBM.dot11SpectrumManagementRequired    = TV_FALSE;

#ifdef MAC_802_11H
	if(get_current_start_freq() == RC_START_FREQ_5)
	{
		MIBM.dot11SpectrumManagementImplemented = TV_TRUE;
		MIBM.dot11SpectrumManagementRequired    = TV_TRUE;
	}
#endif /* MAC_802_11H */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_private_mib                                */
/*                                                                           */
/*  Description      : This function initializes all the private MIB entries */
/*                     used by MAC.                                          */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB                                                 */
/*                                                                           */
/*  Processing       : All the private MIB entries are initialized to their  */
/*                     default values.                                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_private_mib(void)
{
	TROUT_FUNC_ENTER;
    mem_set(&P_MIB, 0, sizeof(p_mib_t));

    mem_set(P_MIB.bssid, 0, 6);
    mem_set(P_MIB.bcst_addr, 0xFF, 6);

    P_MIB.auth_type        = (UWORD8)OPEN_SYSTEM;
    P_MIB.scan_type        = ACTIVE_SCAN;
    P_MIB.listen_interval  = 3;
    P_MIB.chan_mgmt_enable = TV_FALSE;
    P_MIB.chan_switch_cnt  = MAX(2 * mget_DTIMPeriod(), WIFI_MIN_CH_SWITCH_CNT);
    P_MIB.chan_switch_mode = 1;

#ifdef MAC_802_11H
	if(get_current_start_freq() == RC_START_FREQ_5)
    	P_MIB.chan_mgmt_enable = TV_TRUE;
#endif /* MAC_802_11H */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

	memset(&P_WAPI_MIB,0x00,sizeof(p_wapi_mib_t));

	P_WAPI_MIB.wapi_enable = TV_FALSE;
	P_WAPI_MIB.wapi_key_ok = TV_FALSE;
#endif


}

void mib_mac_and_private_print(void)
{
	int i = 0;
	int j = 0;
	TROUT_DBG4("/*************************************************************************/\n");
	TROUT_DBG4("/* mib mac and private                                                   */\n");
	TROUT_DBG4("/*************************************************************************/\n");

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* SMT Station Configuration Table - members of dot11StationConfigEntry  */\n");
    TROUT_DBG4("/*************************************************************************/\n");

	TROUT_DBG4("MIBM.dot11StationID                     = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",MIBM.dot11StationID[i]);
	}
	TROUT_DBG4("\n");

	TROUT_DBG4("MIBM.dot11MediumOccupancyLimit          = %u\n",MIBM.dot11MediumOccupancyLimit);

	TROUT_DBG4("MIBM.dot11CFPollable                    = %s\n",(MIBM.dot11CFPollable == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	TROUT_DBG4("MIBM.dot11CFPPeriod                     = %u\n",MIBM.dot11CFPPeriod);

	TROUT_DBG4("MIBM.dot11CFPMaxDuration                = %u\n",MIBM.dot11CFPMaxDuration);

	TROUT_DBG4("MIBM.dot11AuthenticationResponseTimeOut = %u\n",MIBM.dot11AuthenticationResponseTimeOut);

	TROUT_DBG4("MIBM.dot11PrivacyOptionImplemented      = %s\n",(MIBM.dot11PrivacyOptionImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	TROUT_DBG4("MIBM.dot11PowerManagementMode           = %u\n",MIBM.dot11PowerManagementMode);

	TROUT_DBG4("MIBM.dot11DesiredSSID                   = \"%s\"\n",MIBM.dot11DesiredSSID);
    TROUT_DBG4("MIBM.dot11DesiredBSSType                = %s\n",(MIBM.dot11DesiredBSSType == (UWORD8)INFRASTRUCTURE) ? "INFRASTRUCTURE" :
												 (MIBM.dot11DesiredBSSType == (UWORD8)INDEPENDENT) ? "INDEPENDENT" : "ANY_BSS" );
	TROUT_DBG4("MIBM.dot11OperationalRateSet.num_rates  = %u\n",MIBM.dot11OperationalRateSet.num_rates);
			   
	for(i = 0 ; i < MAX_RATES_SUPPORTED; i++ )
	{
		TROUT_DBG4("MIBM.dot11OperationalRateSet.rates[%2d]  = %u\n",i,MIBM.dot11OperationalRateSet.rates[i]);
	}

	TROUT_DBG4("MIBM.dot11BeaconPeriod                  = %ums\n",MIBM.dot11BeaconPeriod);

	TROUT_DBG4("MIBM.dot11DTIMPeriod                    = %u\n",MIBM.dot11DTIMPeriod);

	TROUT_DBG4("MIBM.dot11AssociationResponseTimeOut    = %ums\n",MIBM.dot11AssociationResponseTimeOut);

	TROUT_DBG4("MIBM.dot11DisassociateReason            = %u\n",MIBM.dot11DisassociateReason);

	TROUT_DBG4("MIBM.dot11DisassociateStation           = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",MIBM.dot11DisassociateStation[i]);
	}
	TROUT_DBG4("\n");

	TROUT_DBG4("MIBM.dot11DeauthenticateReason          = %u\n",MIBM.dot11DeauthenticateReason);

	TROUT_DBG4("MIBM.dot11DeauthenticateStation         = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",MIBM.dot11DeauthenticateStation[i]);
	}
	TROUT_DBG4("\n");

	TROUT_DBG4("MIBM.dot11AuthenticateFailStatus        = %u\n",MIBM.dot11AuthenticateFailStatus);

	TROUT_DBG4("MIBM.dot11AuthenticateFailStation       = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",MIBM.dot11AuthenticateFailStation[i]);
	}
	TROUT_DBG4("\n");

	TROUT_DBG4("/*************************************************************************/\n");
	TROUT_DBG4("/* Spectrum management                                                   */\n");
	TROUT_DBG4("/*************************************************************************/\n");

	TROUT_DBG4("MIBM.dot11SpectrumManagementRequired    = %s\n",(MIBM.dot11SpectrumManagementRequired == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
	TROUT_DBG4("MIBM.dot11SpectrumManagementImplemented = %s\n",(MIBM.dot11SpectrumManagementImplemented == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Authentication Algorithms Table - mem of dot11AuthenticationAlgorithm */\n");
    TROUT_DBG4("/*************************************************************************/\n");

	for(i = 0 ; i < NUM_DOT11AUTHENTICATIONALGORITHMS ; i++ )
	{
		TROUT_DBG4("MIBM.dot11AuthenticationAlgorithmsTable[%2d].dot11AuthenticationAlgorithm        = %u ",i,MIBM.dot11AuthenticationAlgorithmsTable[i].dot11AuthenticationAlgorithm);
		TROUT_DBG4("MIBM.dot11AuthenticationAlgorithmsTable[%2d].dot11AuthenticationAlgorithmsEnable = %s\n",i,
			(MIBM.dot11AuthenticationAlgorithmsTable[i].dot11AuthenticationAlgorithmsEnable == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
		TROUT_DBG4("\n");
	}

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* WEP Default Keys Table - members of Dot11WEPDefaultKeysEntry          */\n");
    TROUT_DBG4("/*************************************************************************/\n");

	for(i = 0 ; i< NUM_DOT11WEPDEFAULTKEYVALUE ; i++)
	{
		TROUT_DBG4("MIBM.dot11WEPDefaultKeyValue[%2d] = \n",i);
		for(j = 0 ; j < ((MIBM.dot11WEPDefaultKeyValue[i][0] == 104) ? 13 : 5) ; j++)
			TROUT_DBG4("%02x ",MIBM.dot11WEPDefaultKeyValue[i][j+1]);

		TROUT_DBG4("\n");
	}


    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* WEP Kep Mappings Table - members of Dot11WEPKeyMappingsEntry          */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* Conceptual table for WEP Key Mappings. The MIB supports the ability   */
    /* to share a separate WEP key for each RA/TA pair. The Key Mappings     */
    /* Table contains zero or one entry for each MAC address and contains    */
    /* two fields for each entry: WEPOn and the corresponding WEP key. The   */
    /* WEP key mappings are logically WRITE-ONLY. Attempts to read the       */
    /* entries in this table shall return unsuccessful status and values of  */
    /* null or zero. The default value for all WEPOn fields is false.        */
    /* REFERENCE "ISO/IEC 8802-11:1999, 8.3.2"                               */
	for(i = 0 ; i< NUM_DOT11WEPKEYMAPPINGSTABLE ; i++)
	{
		TROUT_DBG4("MIBM.dot11WEPKeyMappingsTable[%2d].dot11WEPKeyMappingAddress = ",i);
		for(j = 0 ; j < 6 ; j++)
			TROUT_DBG4("%02x ",MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingAddress[j]);

		TROUT_DBG4("\n");

		TROUT_DBG4("MIBM.dot11WEPKeyMappingsTable[%2d].dot11WEPKeyMappingWEPOn   = %s\n",
			i,(MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingWEPOn == TV_TRUE) 
				? "TV_TRUE" : "TV_FALSE");

		TROUT_DBG4("MIBM.dot11WEPKeyMappingsTable[%2d].dot11WEPKeyMappingValue   = \n",i);
		for(j = 0 ; j < ((MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingValue[0] == 104) ? 13 : 5) ; j++)
			TROUT_DBG4("%02x ",MIBM.dot11WEPKeyMappingsTable[i].dot11WEPKeyMappingValue[j+1]);

		TROUT_DBG4("\n");
	}

	TROUT_DBG4("/*************************************************************************/\n");
	TROUT_DBG4("/* Privacy Table - members of dot11PrivacyEntry                          */\n");
	TROUT_DBG4("/*************************************************************************/\n");
    /* Group containing attributes concerned with IEEE 802.11 Privacy.       */
    /* Created as a table to allow multiple instantiations on an agent.      */
    /* Currently, table is not provided as a single instantiation exists.    */

	TROUT_DBG4("MIBM.dot11PrivacyInvoked      = %s\n",(MIBM.dot11PrivacyInvoked == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	TROUT_DBG4("MIBM.dot11WEPDefaultKeyID     = %u\n",MIBM.dot11WEPDefaultKeyID);

	TROUT_DBG4("MIBM.dot11WEPKeyMappingLength = %u\n",MIBM.dot11WEPKeyMappingLength);

	TROUT_DBG4("MIBM.dot11ExcludeUnencrypted  = %s\n",(MIBM.dot11ExcludeUnencrypted == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

	TROUT_DBG4("MIBM.dot11WEPICVErrorCount    = %u\n",MIBM.dot11WEPICVErrorCount);

	TROUT_DBG4("MIBM.dot11WEPExcludedCount    = %u\n",MIBM.dot11WEPExcludedCount);

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* SMT Notication                                                        */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* The value of the notification shall include the MAC address of the    */
    /* MAC to which the Disassoc/Deauth/auth frame was sent and the reason   */
    /* for the same.                                                         */
	TROUT_DBG4("MIBM.dot11Disassociate.reason      = %u\n",MIBM.dot11Disassociate.reason);
	TROUT_DBG4("MIBM.dot11Disassociate.station     = %02x:%02x:%02x:%02x:%02x:%02x\n"
												,MIBM.dot11Disassociate.station[0]
												,MIBM.dot11Disassociate.station[1]
												,MIBM.dot11Disassociate.station[2]
												,MIBM.dot11Disassociate.station[3]
												,MIBM.dot11Disassociate.station[4]
												,MIBM.dot11Disassociate.station[5]);

	TROUT_DBG4("MIBM.dot11Disassociate.reason      = %u\n",MIBM.dot11Disassociate.reason);
	TROUT_DBG4("MIBM.dot11Disassociate.station     = %02x:%02x:%02x:%02x:%02x:%02x\n"
												,MIBM.dot11Disassociate.station[0]
												,MIBM.dot11Disassociate.station[1]
												,MIBM.dot11Disassociate.station[2]
												,MIBM.dot11Disassociate.station[3]
												,MIBM.dot11Disassociate.station[4]
												,MIBM.dot11Disassociate.station[5]);
	TROUT_DBG4("MIBM.dot11Deauthenticate.reason    = %u\n",MIBM.dot11Deauthenticate.reason);
	TROUT_DBG4("MIBM.dot11Deauthenticate.station   = %02x:%02x:%02x:%02x:%02x:%02x\n"
												,MIBM.dot11Deauthenticate.station[0]
												,MIBM.dot11Deauthenticate.station[1]
												,MIBM.dot11Deauthenticate.station[2]
												,MIBM.dot11Deauthenticate.station[3]
												,MIBM.dot11Deauthenticate.station[4]
												,MIBM.dot11Deauthenticate.station[5]);
	TROUT_DBG4("MIBM.dot11AuthenticateFail.reason  = %u\n",MIBM.dot11AuthenticateFail.reason);
	TROUT_DBG4("MIBM.dot11AuthenticateFail.station = %02x:%02x:%02x:%02x:%02x:%02x\n"
												,MIBM.dot11AuthenticateFail.station[0]
												,MIBM.dot11AuthenticateFail.station[1]
												,MIBM.dot11AuthenticateFail.station[2]
												,MIBM.dot11AuthenticateFail.station[3]
												,MIBM.dot11AuthenticateFail.station[4]
												,MIBM.dot11AuthenticateFail.station[5]);

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Operation Table - members of Dot11OperationEntry                      */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* Group contains MAC attributes pertaining to the operation of the MAC. */
    /* This has been implemented as a table in order to allow for multiple   */
    /* instantiations on an agent.                                           */
    /* Currently, table is not provided as a single instantiation exists.    */
	TROUT_DBG4("MIBM.dot11MACAddress              = %02x:%02x:%02x:%02x:%02x:%02x\n"
												   ,MIBM.dot11MACAddress[0]
												   ,MIBM.dot11MACAddress[1]
												   ,MIBM.dot11MACAddress[2]
												   ,MIBM.dot11MACAddress[3]
												   ,MIBM.dot11MACAddress[4]
												   ,MIBM.dot11MACAddress[5]);

	TROUT_DBG4("MIBM.dot11RTSThreshold            = %u\n",MIBM.dot11RTSThreshold);

	TROUT_DBG4("MIBM.dot11ShortRetryLimit         = %u\n",MIBM.dot11ShortRetryLimit);

	TROUT_DBG4("MIBM.dot11LongRetryLimit          = %u\n",MIBM.dot11LongRetryLimit);

	TROUT_DBG4("MIBM.dot11FragmentationThreshold  = %u\n",MIBM.dot11FragmentationThreshold);

	TROUT_DBG4("MIBM.dot11MaxTransmitMSDULifetime = %u\n",MIBM.dot11MaxTransmitMSDULifetime);

	TROUT_DBG4("MIBM.dot11MaxReceiveLifetime      = %u\n",MIBM.dot11MaxReceiveLifetime);

	TROUT_DBG4("MIBM.dot11ManufacturerID          = %s\n",MIBM.dot11ManufacturerID);

	TROUT_DBG4("MIBM.dot11ProductID               = %s\n",MIBM.dot11ProductID);

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Counters Table - members of Dot11CountersEntry                        */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* Group containing attributes that are MAC counters. Implemented as a   */
    /* table to allow for multiple instantiations on an agent.               */
    /* Currently, table is not provided as a single instantiation exists.    */

	TROUT_DBG4("MIBM.dot11TransmittedFragmentCount       = %u\n",MIBM.dot11TransmittedFragmentCount);

	TROUT_DBG4("MIBM.dot11MulticastTransmittedFrameCount = %u\n",MIBM.dot11MulticastTransmittedFrameCount);

	TROUT_DBG4("MIBM.dot11FailedCount                    = %u\n",MIBM.dot11FailedCount);

	TROUT_DBG4("MIBM.dot11RetryCount                     = %u\n",MIBM.dot11RetryCount);

	TROUT_DBG4("MIBM.dot11MultipleRetryCount             = %u\n",MIBM.dot11MultipleRetryCount);

	TROUT_DBG4("MIBM.dot11FrameDuplicateCount            = %u\n",MIBM.dot11FrameDuplicateCount);

	TROUT_DBG4("MIBM.dot11RTSSuccessCount                = %u\n",MIBM.dot11RTSSuccessCount);

	TROUT_DBG4("MIBM.dot11RTSFailureCount                = %u\n",MIBM.dot11RTSFailureCount);

	TROUT_DBG4("MIBM.dot11ACKFailureCount                = %u\n",MIBM.dot11ACKFailureCount);

	TROUT_DBG4("MIBM.dot11ReceivedFragmentCount          = %u\n",MIBM.dot11ReceivedFragmentCount);

	TROUT_DBG4("MIBM.dot11MulticastReceivedFrameCount    = %u\n",MIBM.dot11MulticastReceivedFrameCount);

	TROUT_DBG4("MIBM.dot11FCSErrorCount 	                = %u\n",MIBM.dot11FCSErrorCount);

	TROUT_DBG4("MIBM.dot11TransmittedFrameCount          = %u\n",MIBM.dot11TransmittedFrameCount);

	TROUT_DBG4("MIBM.dot11WEPUndecryptableCount          = %u\n",MIBM.dot11WEPUndecryptableCount);

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Group Addresses Table - members of Dot11GroupAddressesEntry           */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* A conceptual table containing a set of MAC addresses identifying the  */
    /* multicast addresses for which this STA will receive frames. Default   */
    /* value of this attribute shall be null. The total number of bytes that */
    /* are allocated for this is equal to the number of addresses supported  */
    /* (each of 6 bytes) and 1 byte for storing the size (number of          */
    /* addresses stored).                                                    */

	TROUT_DBG4("MIBM.dot11GroupAddressTable nums    = %u \n",MIBM.dot11GroupAddressTable[0]);
	for( i = 0 ; i < MIBM.dot11GroupAddressTable[0] ; i++)
	{
		TROUT_DBG4("MIBM.dot11GroupAddressTable sub[%2d] = %02x:%02x:%02x:%02x:%02x:%02x\n",i
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 0]
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 1]
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 2]
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 3]
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 4]
														 ,MIBM.dot11GroupAddressTable[1 + i*6 + 5]);
	}

    //TROUT_DBG4("/*************************************************************************/\n");
    //TROUT_DBG4("/* Resource Type Attribute Templates                                     */\n");
    //TROUT_DBG4("/*************************************************************************/\n");
    /* Contains the name of the Resource Type ID managed object. Attribute   */
    /* is read-only and always contains the value RTID. This attribute value */
    /* shall not be used as a naming attribute for any other managed object  */
    /* class. REFERENCE "IEEE Std 802.1F-1993, A.7"                          */
    /* This is currently unused and hence commented out...                   */
    /* UWORD8 dot11ResourceTypeIDName[4];                                    */

    TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Resource info Table - Dot11ResourceInfoEntry                          */\n");
    TROUT_DBG4("/*************************************************************************/\n");
    /* Provides a means of indicating, in data readable from a managed       */
    /* object information that identifies the source of the implementation.  */
    /* REFERENCE "IEEE Std 802.1F-1993, A.7"                                 */

	TROUT_DBG4("MIBM.dot11manufacturerOUI            = %04x\n",MIBM.dot11manufacturerOUI);

	TROUT_DBG4("MIBM.dot11manufacturerName           = %s\n",MIBM.dot11manufacturerName);
	
	TROUT_DBG4("MIBM.dot11manufacturerProductName    = %04x\n",MIBM.dot11manufacturerProductName);

	TROUT_DBG4("MIBM.dot11manufacturerProductVersion = %s\n",MIBM.dot11manufacturerProductVersion);

	TROUT_DBG4("/*************************************************************************/\n");
	TROUT_DBG4("/* private mib                                                           */\n");
	TROUT_DBG4("/*************************************************************************/\n");

	TROUT_DBG4("P_MIB.bssid                = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",P_MIB.bssid[i]);
	}
	TROUT_DBG4("\n");
	
	TROUT_DBG4("P_MIB.bcst_addr            = ");
	for(i = 0 ; i < 6 ; i++ )
	{
		TROUT_DBG4("%02x ",P_MIB.bcst_addr[i]);
	}
	TROUT_DBG4("\n");

	TROUT_DBG4("P_MIB.auth_type            = %s\n",(P_MIB.auth_type == (UWORD8)OPEN_SYSTEM) ? "OPEN_SYSTEM" :
										      (P_MIB.auth_type == (UWORD8)SHARED_KEY)  ? "SHARED_KEY"  : "ANY" );

    TROUT_DBG4("P_MIB.scan_type            = %s\n",(P_MIB.scan_type == (UWORD8)ACTIVE_SCAN) ? "ACTIVE_SCAN" : "PASSIVE_SCAN");

    TROUT_DBG4("P_MIB.listen_interval      = %u\n",P_MIB.listen_interval);

	TROUT_DBG4("P_MIB.packet_success_count = %u\n",P_MIB.packet_success_count);

	TROUT_DBG4("/*************************************************************************/\n");
    TROUT_DBG4("/* Channel Management                                                    */\n");
    TROUT_DBG4("/*************************************************************************/\n");

	TROUT_DBG4("P_MIB.chan_mgmt_enable     = %s\n",(P_MIB.chan_mgmt_enable == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");

    TROUT_DBG4("P_MIB.chan_switch_cnt      = %u\n",P_MIB.chan_switch_cnt);

	TROUT_DBG4("P_MIB.chan_switch_mode     = %u\n",P_MIB.chan_switch_mode);


}

