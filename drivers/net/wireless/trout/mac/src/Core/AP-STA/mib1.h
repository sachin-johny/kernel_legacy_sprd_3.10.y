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
/*  File Name         : mib1.h                                               */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB as given in Annex-D of 802.11/b/a/g   */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all Private MIB parameters.     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MIB1_H
#define MIB1_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "management.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_DOT11AUTHENTICATIONALGORITHMS 6
#define NUM_DOT11WEPDEFAULTKEYVALUE       4
#define NUM_DOT11WEPKEYMAPPINGSTABLE      1
#define NUM_DOT11GRPADDRESS               6
#define GROUP_ADDRESS_SIZE_OFFSET         0
#define GROUP_ADDRESS_VALUE_OFFSET        1
#define MAX_ID_LEN                        16
#define MAX_WEP_STR_SIZE                  27 /* 5 for WEP 40; 13 for WEP 104 */
#define WEP_SIZE_OFFSET                   0  /* 5 for WEP 40; 13 for WEP 104 */
#define WEP_KEY_VALUE_OFFSET              1
#define NUM_DOT11MANUFACTURERNAME         128
#define NUM_DOT11MANUFACTURERPRODUCTVER   128


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* 'TruthValue' definition (according to RFC) */
typedef enum {TV_TRUE  = 1,
              TV_FALSE = 2
} TRUTH_VALUE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Basic rate set. This contains the set of basic rates supported by any     */
/* IBSS.                                                                     */
typedef struct
{
    UWORD8 rates[MAX_RATES_SUPPORTED];
    UWORD8 num_rates;
} rate_t;

/* Authentication Algorithms Table - Contains the authentication algorithm   */
/* type and whether it is enabled or not. It is instantiated as an array in  */
/* the  MIB structure.                                                       */
typedef struct
{
    UWORD8        dot11AuthenticationAlgorithm;
    TRUTH_VALUE_T dot11AuthenticationAlgorithmsEnable;
} dot11AuthenticationAlgorithmsEntry_t;

/* WEP Kep Mappings Table structure enables the use of different WEP keys    */
/* for different stations. It is instantiated as an array in MIB structure.  */
typedef struct
{
    UWORD8        dot11WEPKeyMappingAddress[6];
    TRUTH_VALUE_T dot11WEPKeyMappingWEPOn;
    UWORD8        dot11WEPKeyMappingValue[MAX_WEP_STR_SIZE];
} dot11WEPKeyMappingsEntry_t;

/* SMT Notication structure shall include the MAC address of the MAC to      */
/* which the Disassoc/Deauth/auth frame was sent and the reason for the same */
typedef struct
{
    UWORD16 reason;
    UWORD8  station[6];
} notification_t;



/* Generic MIB structure */
typedef struct
{
    /*************************************************************************/
    /* SMT Station Configuration Table - members of dot11StationConfigEntry  */
    /*************************************************************************/
    /* An entry in the dot11StationConfigTable. It is possible for there to  */
    /* be multiple IEEE 802.11 interfaces on one agent, each with its unique */
    /* MAC address. The relationship between an IEEE 802.11 interface and an */
    /* interface in the context of the Internet-standard MIB is one-to-one.  */
    /* As such, the value of an ifIndex object instance can be directly used */
    /* to identify corresponding instances of the objects defined herein.    */
    /* ifIndex - Each 802.11 interface is represented by an ifEntry.         */
    /* Interface tables in this MIB module are indexed by ifIndex.           */
    UWORD8        dot11StationID[6];
    UWORD16       dot11MediumOccupancyLimit;
    TRUTH_VALUE_T dot11CFPollable;
    UWORD8        dot11CFPPeriod;
    UWORD16       dot11CFPMaxDuration;
    UWORD32       dot11AuthenticationResponseTimeOut;
    TRUTH_VALUE_T dot11PrivacyOptionImplemented;
    UWORD8        dot11PowerManagementMode;
    WORD8         dot11DesiredSSID[MAX_SSID_LEN];
    UWORD8        dot11DesiredBSSType;
    rate_t        dot11OperationalRateSet;
    UWORD16       dot11BeaconPeriod;
    UWORD8        dot11DTIMPeriod;
    UWORD32       dot11AssociationResponseTimeOut;
    UWORD16       dot11DisassociateReason;
    UWORD8        dot11DisassociateStation[6];
    UWORD16       dot11DeauthenticateReason;
    UWORD8        dot11DeauthenticateStation[6];
    UWORD16       dot11AuthenticateFailStatus;
    UWORD8        dot11AuthenticateFailStation[6];

    /****************  Spectrum management ***********************************/
    TRUTH_VALUE_T dot11SpectrumManagementRequired;
    TRUTH_VALUE_T dot11SpectrumManagementImplemented;


    /*************************************************************************/
    /* Authentication Algorithms Table - mem of dot11AuthenticationAlgorithm */
    /*************************************************************************/
    /* This (conceptual) table of attributes shall be a set of all the       */
    /* authentication algorithms supported by the stations. The following    */
    /* are the default values and the associated algorithm:                  */
    /* Value = 1: Open System                                                */
    /* Value = 2: Shared Key"                                                */
    /* REFERENCE "ISO/IEC 8802-11:1999, 7.3.1.1"                             */
    dot11AuthenticationAlgorithmsEntry_t
        dot11AuthenticationAlgorithmsTable[NUM_DOT11AUTHENTICATIONALGORITHMS];

    /*************************************************************************/
    /* WEP Default Keys Table - members of Dot11WEPDefaultKeysEntry          */
    /*************************************************************************/
    /* Conceptual table for WEP default keys. This table shall contain the   */
    /* four WEP default secret key values corresponding to the four possible */
    /* KeyID values. The WEP default secret keys are logically WRITE-ONLY.   */
    /* Attempts to read the entries in this table shall return unsuccessful  */
    /* status and values of null or zero. The default value of each WEP      */
    /* default key shall be null. REFERENCE "ISO/IEC 8802-11:1999, 8.3.2"    */
    UWORD8 dot11WEPDefaultKeyValue[NUM_DOT11WEPDEFAULTKEYVALUE]
                                  [MAX_WEP_STR_SIZE];

    /*************************************************************************/
    /* WEP Kep Mappings Table - members of Dot11WEPKeyMappingsEntry          */
    /*************************************************************************/
    /* Conceptual table for WEP Key Mappings. The MIB supports the ability   */
    /* to share a separate WEP key for each RA/TA pair. The Key Mappings     */
    /* Table contains zero or one entry for each MAC address and contains    */
    /* two fields for each entry: WEPOn and the corresponding WEP key. The   */
    /* WEP key mappings are logically WRITE-ONLY. Attempts to read the       */
    /* entries in this table shall return unsuccessful status and values of  */
    /* null or zero. The default value for all WEPOn fields is false.        */
    /* REFERENCE "ISO/IEC 8802-11:1999, 8.3.2"                               */
    dot11WEPKeyMappingsEntry_t
        dot11WEPKeyMappingsTable[NUM_DOT11WEPKEYMAPPINGSTABLE];

    /*************************************************************************/
    /* Privacy Table - members of dot11PrivacyEntry                          */
    /*************************************************************************/
    /* Group containing attributes concerned with IEEE 802.11 Privacy.       */
    /* Created as a table to allow multiple instantiations on an agent.      */
    /* Currently, table is not provided as a single instantiation exists.    */
    TRUTH_VALUE_T dot11PrivacyInvoked;
    UWORD8        dot11WEPDefaultKeyID;
    UWORD32       dot11WEPKeyMappingLength;
    TRUTH_VALUE_T dot11ExcludeUnencrypted;
    UWORD32       dot11WEPICVErrorCount;
    UWORD32       dot11WEPExcludedCount;

    /*************************************************************************/
    /* SMT Notication                                                        */
    /*************************************************************************/
    /* The value of the notification shall include the MAC address of the    */
    /* MAC to which the Disassoc/Deauth/auth frame was sent and the reason   */
    /* for the same.                                                         */
    notification_t dot11Disassociate;
    notification_t dot11Deauthenticate;
    notification_t dot11AuthenticateFail;

    /*************************************************************************/
    /* Operation Table - members of Dot11OperationEntry                      */
    /*************************************************************************/
    /* Group contains MAC attributes pertaining to the operation of the MAC. */
    /* This has been implemented as a table in order to allow for multiple   */
    /* instantiations on an agent.                                           */
    /* Currently, table is not provided as a single instantiation exists.    */
    UWORD8  dot11MACAddress[6];
    UWORD16 dot11RTSThreshold;
    UWORD8  dot11ShortRetryLimit;
    UWORD8  dot11LongRetryLimit;
    UWORD16 dot11FragmentationThreshold;
    UWORD32 dot11MaxTransmitMSDULifetime;
    UWORD32 dot11MaxReceiveLifetime;
    WORD8   dot11ManufacturerID[MAX_ID_LEN];
    WORD8   dot11ProductID[MAX_ID_LEN];

    /*************************************************************************/
    /* Counters Table - members of Dot11CountersEntry                        */
    /*************************************************************************/
    /* Group containing attributes that are MAC counters. Implemented as a   */
    /* table to allow for multiple instantiations on an agent.               */
    /* Currently, table is not provided as a single instantiation exists.    */
    UWORD32 dot11TransmittedFragmentCount;
    UWORD32 dot11MulticastTransmittedFrameCount;
    UWORD32 dot11FailedCount;
    UWORD32 dot11RetryCount;
    UWORD32 dot11MultipleRetryCount;
    UWORD32 dot11FrameDuplicateCount;
    UWORD32 dot11RTSSuccessCount;
    UWORD32 dot11RTSFailureCount;
    UWORD32 dot11ACKFailureCount;
    UWORD32 dot11ReceivedFragmentCount;
    UWORD32 dot11MulticastReceivedFrameCount;
    UWORD32 dot11FCSErrorCount;
    UWORD32 dot11TransmittedFrameCount;
    UWORD32 dot11WEPUndecryptableCount;

    /*************************************************************************/
    /* Group Addresses Table - members of Dot11GroupAddressesEntry           */
    /*************************************************************************/
    /* A conceptual table containing a set of MAC addresses identifying the  */
    /* multicast addresses for which this STA will receive frames. Default   */
    /* value of this attribute shall be null. The total number of bytes that */
    /* are allocated for this is equal to the number of addresses supported  */
    /* (each of 6 bytes) and 1 byte for storing the size (number of          */
    /* addresses stored).                                                    */
    UWORD8 dot11GroupAddressTable[NUM_DOT11GRPADDRESS * 6 + 1];

    /*************************************************************************/
    /* Resource Type Attribute Templates                                     */
    /*************************************************************************/
    /* Contains the name of the Resource Type ID managed object. Attribute   */
    /* is read-only and always contains the value RTID. This attribute value */
    /* shall not be used as a naming attribute for any other managed object  */
    /* class. REFERENCE "IEEE Std 802.1F-1993, A.7"                          */
    /* This is currently unused and hence commented out...                   */
    /* UWORD8 dot11ResourceTypeIDName[4];                                    */

    /*************************************************************************/
    /* Resource info Table - Dot11ResourceInfoEntry                          */
    /*************************************************************************/
    /* Provides a means of indicating, in data readable from a managed       */
    /* object information that identifies the source of the implementation.  */
    /* REFERENCE "IEEE Std 802.1F-1993, A.7"                                 */
    UWORD16 dot11manufacturerOUI;
    UWORD8  dot11manufacturerName[NUM_DOT11MANUFACTURERNAME];
    UWORD16 dot11manufacturerProductName;
    UWORD8  dot11manufacturerProductVersion[NUM_DOT11MANUFACTURERPRODUCTVER];

} mib_t;

/* Structure for private MIB */
typedef struct
{
    UWORD8  bssid[6];                          /* BSSID                      */
    UWORD8  bcst_addr[6];                      /* Broadcast address          */
    UWORD8  auth_type;                         /* Authentication type of STA */
    UWORD8  scan_type;                         /* Scan type of STA           */
    UWORD8  listen_interval;                   /* Listen interval of STA     */
    UWORD32 packet_success_count;              /* Packet Success count       */

    /*************************************************************************/
    /* Channel Management                                                    */
    /*************************************************************************/
    TRUTH_VALUE_T chan_mgmt_enable;            /* Enables channel managemnet */
    UWORD8        chan_switch_cnt;             /* Channel Switch Count       */
    UWORD8        chan_switch_mode;            /* Channel Switch Mode        */

} p_mib_t;

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
typedef struct
{
	TRUTH_VALUE_T wapi_enable;
	TRUTH_VALUE_T wapi_key_ok;
	UWORD8        wapi_version[2];
	UWORD8        wapi_akm_cnt;
	UWORD8        wapi_akm[3][4];
	UWORD8        wapi_pcip_cnt;
	UWORD8        wapi_pcip_policy[3][4];
	UWORD8        wapi_grp_policy[4];

	UWORD8        wapi_ap_address[6];
	UWORD8        wapi_pairwise_key_index;
	UWORD8        wapi_pairwise_key_txrsc[16];
	UWORD8        wapi_pairwise_key_rxrsc[16];
	UWORD8        wapi_pairwise_pkt_key[3][16];
	UWORD8        wapi_pairwise_mic_key[3][16];
	UWORD8        wapi_group_key_index;
	UWORD8        wapi_group_key_rsc[16];
	UWORD8        wapi_group_pkt_key[3][16];
	UWORD8        wapi_group_mic_key[3][16];

	UWORD8        wapi_cap[2];
} p_wapi_mib_t;
#endif

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern mib_t   MIBM;
extern p_mib_t P_MIB;

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
extern p_wapi_mib_t P_WAPI_MIB;
#endif

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void initialize_mac_mib(void);
extern void initialize_private_mib(void);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* PRIVATE MIB Access Functions */

/* BSSID is a 46bit random number generated by the station when started or   */
/* is adopted from the beacon at start or is the value adopted from the      */
/* beacon with same SSID but different BSSID than the station BSSID.         */
INLINE UWORD8* mget_bssid(void)
{
    return P_MIB.bssid;
}

INLINE void mset_bssid(UWORD8* ma)
{
    memcpy(P_MIB.bssid, ma, 6);
}

/* Broadcast address is a 48 bit string of 1s. */
INLINE UWORD8* mget_bcst_addr(void)
{
    return P_MIB.bcst_addr;
}

INLINE void mset_bcst_addr(UWORD8* ba)
{
    memcpy(P_MIB.bcst_addr, ba, 6);
}

/* Authentication algorithm that is currently in use. 0 -> OPEN SYSTEM       */
/* 1 -> SHARED KEY                                                           */
INLINE UWORD8 mget_auth_type(void)
{
    return P_MIB.auth_type;
}

INLINE void mset_auth_type(UWORD8 at)
{
    P_MIB.auth_type = at;
}

/* Scan Type that is to be used. 0 -> Passive Scan, 1 -> Active Scan         */
INLINE UWORD8 mget_scan_type(void)
{
    return P_MIB.scan_type;
}

INLINE void mset_scan_type(UWORD8 st)
{
    if(st < NUM_SCANTYPE)
        P_MIB.scan_type = st;
}

INLINE UWORD8 mget_listen_interval(void)
{
    return P_MIB.listen_interval;
}

INLINE void mset_listen_interval(UWORD8 li)
{
    if(li != 0)
    {
        P_MIB.listen_interval = li;
    }
}

INLINE UWORD8 mget_chan_switch_cnt(void)
{
    return P_MIB.chan_switch_cnt;
}

INLINE void mset_chan_switch_cnt(UWORD8 val)
{
    P_MIB.chan_switch_cnt = val;
}

INLINE UWORD8 mget_chan_switch_mode(void)
{
    return P_MIB.chan_switch_mode;
}

INLINE void mset_chan_switch_mode(UWORD8 val)
{
    P_MIB.chan_switch_mode = val;
}

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
INLINE TRUTH_VALUE_T mget_wapi_key_ok(void)
{
	//printk("mget_wapi_key_ok %s\n",(P_WAPI_MIB.wapi_key_ok == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    return P_WAPI_MIB.wapi_key_ok;
}

INLINE void mset_wapi_key_ok(TRUTH_VALUE_T val)
{
	//printk("mset_wapi_key_ok %s\n",(val == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    P_WAPI_MIB.wapi_key_ok = val;
}

INLINE TRUTH_VALUE_T mget_wapi_enable(void)
{
	//printk("mget_wapi_enable %s\n",(P_WAPI_MIB.wapi_enable == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    return P_WAPI_MIB.wapi_enable;
}

INLINE void mset_wapi_enable(TRUTH_VALUE_T val)
{
	//printk("mset_wapi_enable %s\n",(val == TV_TRUE) ? "TV_TRUE" : "TV_FALSE");
    P_WAPI_MIB.wapi_enable = val;
}

INLINE WORD8* mget_wapi_version(void)
{
    return P_WAPI_MIB.wapi_version;
}

INLINE void mset_wapi_version(WORD8* val)
{
	memcpy(P_WAPI_MIB.wapi_version,val,2);
}

INLINE UWORD8 mget_wapi_akm_cnt(void)
{
    return P_WAPI_MIB.wapi_akm_cnt;
}

INLINE void mset_wapi_akm_cnt(UWORD8 val)
{
    P_WAPI_MIB.wapi_akm_cnt = val;
}

INLINE UWORD8* mget_wapi_akm(int index)
{
    return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_akm[index]; 
}

INLINE void mset_wapi_akm(int index,UWORD8* val)
{
    if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_akm[index]),val,4);
	}
}

INLINE UWORD8 mget_wapi_pcip_cnt(void)
{
    return P_WAPI_MIB.wapi_pcip_cnt;
}

INLINE void mset_wapi_pcip_cnt(UWORD8 val)
{
    P_WAPI_MIB.wapi_pcip_cnt = val;
}

INLINE UWORD8* mget_wapi_pcip_policy(int index)
{
	return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_pcip_policy[index]; 
}

INLINE void mset_wapi_pcip_policy(int index,UWORD8* val)
{
	if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_pcip_policy[index]),val,4);
	}
}

INLINE UWORD8* mget_wapi_grp_policy(void)
{
    return P_WAPI_MIB.wapi_grp_policy;
}

INLINE void mset_wapi_grp_policy(UWORD8* val)
{
	memcpy(P_WAPI_MIB.wapi_grp_policy,val,4);
}

INLINE UWORD8* mget_wapi_cap(void)
{
    return P_WAPI_MIB.wapi_cap;
}

INLINE void mset_wapi_cap(UWORD8* val)
{
	memcpy(P_WAPI_MIB.wapi_cap,val,2);
}

INLINE UWORD8* mget_wapi_address(void)
{
	return P_WAPI_MIB.wapi_ap_address; 
}

INLINE void mset_wapi_address(UWORD8* val)
{
	memcpy(&(P_WAPI_MIB.wapi_ap_address),val,6);
}

INLINE UWORD8 mget_wapi_pairwise_key_index(void)
{
	return P_WAPI_MIB.wapi_pairwise_key_index; 
}

INLINE void mset_wapi_pairwise_key_index(int index)
{
	if(index <= 3 )
	{
		P_WAPI_MIB.wapi_pairwise_key_index = index;
	}
}

INLINE UWORD8* mget_wapi_pairwise_key_txrsc(void)
{
	return P_WAPI_MIB.wapi_pairwise_key_txrsc; 
}

INLINE void mset_wapi_pairwise_key_txrsc(UWORD8* val)
{
	//int i = 0 ;
	//printk("mset_wapi_pairwise_key_txrsc:\n");
	//for( i = 0 ; i < 16 ; i++)
	//{
	//	printk("%02x ",val[i]);
	//}
	//printk("\n");
	memcpy(&(P_WAPI_MIB.wapi_pairwise_key_txrsc),val,16);
}

INLINE UWORD8 * inc_wapi_pairwise_key_txrsc(void)
{
	int i;
	
	P_WAPI_MIB.wapi_pairwise_key_txrsc[15] += 2;
	
	if( P_WAPI_MIB.wapi_pairwise_key_txrsc[15] == 0x00 )
	{
		for(i = 14 ; i >= 0 ; i--)
		{
			if( (P_WAPI_MIB.wapi_pairwise_key_txrsc[i] += 1) != 0x00 )
			{
				break;
			}
		}
	}

	return P_WAPI_MIB.wapi_pairwise_key_txrsc;
}

INLINE UWORD8* mget_wapi_pairwise_key_rxrsc(void)
{
	return P_WAPI_MIB.wapi_pairwise_key_rxrsc; 
}

INLINE void mset_wapi_pairwise_key_rxrsc(UWORD8* val)
{
	//int i = 0 ;
	//printk("mset_wapi_pairwise_key_rxrsc:\n");
	//for( i = 0 ; i < 16 ; i++)
	//{
	//	printk("%02x ",val[i]);
	//}
	//printk("\n");
	
	memcpy(&(P_WAPI_MIB.wapi_pairwise_key_rxrsc),val,16);
}

INLINE UWORD8 * inc_wapi_pairwise_key_rxrsc(void)
{
	int i;
	
	P_WAPI_MIB.wapi_pairwise_key_rxrsc[15] += 2;
	
	if( P_WAPI_MIB.wapi_pairwise_key_rxrsc[15] == 0x01 )
	{
		for(i = 14 ; i >= 0 ; i--)
		{
			if( (P_WAPI_MIB.wapi_pairwise_key_rxrsc[i] += 1) != 0x00 );
			{
				break;
			}
		}
	}

	return P_WAPI_MIB.wapi_pairwise_key_rxrsc;
}

INLINE UWORD8* mget_wapi_pairwise_pkt_key(int index)
{
	return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_pairwise_pkt_key[index]; 
}

INLINE void mset_wapi_pairwise_pkt_key(int index,UWORD8* val)
{
	if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_pairwise_pkt_key[index]),val,16);
	}
}	

INLINE UWORD8* mget_wapi_pairwise_mic_key(int index)
{
	return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_pairwise_mic_key[index]; 
}

INLINE void mset_wapi_pairwise_mic_key(int index,UWORD8* val)
{
	if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_pairwise_mic_key[index]),val,16);
	}
}	

INLINE UWORD8 mget_wapi_group_key_index(void)
{
	return P_WAPI_MIB.wapi_group_key_index; 
}

INLINE void mset_wapi_group_key_index(UWORD8 val)
{
	P_WAPI_MIB.wapi_group_key_index = val;
}

INLINE UWORD8* mget_wapi_group_key_rsc(void)
{
	return P_WAPI_MIB.wapi_group_key_rsc; 
}

INLINE void mset_wapi_group_key_rsc(UWORD8* val)
{
	//int i = 0 ;
	//printk("mset_wapi_group_key_rsc:\n");
	//for( i = 0 ; i < 16 ; i++)
	//{
	//	printk("%02x ",val[i]);
	//}
	//printk("\n");

	memcpy(&(P_WAPI_MIB.wapi_group_key_rsc),val,16);
}

INLINE UWORD8 * inc_wapi_group_key_rxrsc(void)
{
	int i;
	
	P_WAPI_MIB.wapi_group_key_rsc[15] += 2;
	
	if( P_WAPI_MIB.wapi_group_key_rsc[15] == 0x00 )
	{
		for(i = 14 ; i >= 0 ; i--)
		{
			if( (P_WAPI_MIB.wapi_group_key_rsc[i] += 1) != 0x00 );
			{
				break;
			}
		}
	}

	return P_WAPI_MIB.wapi_group_key_rsc;
}

INLINE UWORD8* mget_wapi_group_pkt_key(int index)
{
	return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_group_pkt_key[index]; 
}

INLINE void mset_wapi_group_pkt_key(int index,UWORD8* val)
{
	if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_group_pkt_key[index]),val,16);
	}
}	

INLINE UWORD8* mget_wapi_group_mic_key(int index)
{
	return ( index >= 3 ) ? NULL : P_WAPI_MIB.wapi_group_mic_key[index]; 
}

INLINE void mset_wapi_group_mic_key(int index,UWORD8* val)
{
	if(index < 3 )
	{
		memcpy(&(P_WAPI_MIB.wapi_group_mic_key[index]),val,16);
	}
}	

#endif

#endif /* MIB1_H */
