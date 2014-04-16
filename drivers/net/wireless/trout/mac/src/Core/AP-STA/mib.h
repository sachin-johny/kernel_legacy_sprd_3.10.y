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
/*  File Name         : mib.h                                                */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB as given in Annex-D of 802.11/b/a/g   */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all MIB parameters.             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MIB_H
#define MIB_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib1.h"

extern int g_mac_svn_ver;
#ifdef IBSS_BSS_STATION_MODE
extern BOOL_T g_wifi_bt_coex;
#endif
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#ifdef TROUT_SDIO_INTERFACE
#define TROUT_IF_TYPE "SDIO_"
#ifdef IBSS_BSS_STATION_MODE
#define TROUT_FW_VERSION_NUM  "v13.16.01"
#else   /* IBSS_BSS_STATION_MODE */
#define TROUT_FW_VERSION_NUM  "v13.16.01"
#endif	/* IBSS_BSS_STATION_MODE */
#else	/* SPI_INTERFACE */
#define TROUT_IF_TYPE "SPI_"
#ifdef IBSS_BSS_STATION_MODE
#define TROUT_FW_VERSION_NUM  "v13.16.01"
#else   /* IBSS_BSS_STATION_MODE */
#define TROUT_FW_VERSION_NUM  "v13.16.01"
#endif	/* IBSS_BSS_STATION_MODE */
#endif	/* TROUT_SDIO_INTERFACE */


#ifdef IBSS_BSS_STATION_MODE
#define MAC_FW_VERSION          "SPRD_STA_" TROUT_IF_TYPE "" TROUT_FW_VERSION_NUM
#else
#define MAC_FW_VERSION          "SPRD_AP_" TROUT_IF_TYPE "" TROUT_FW_VERSION_NUM
#endif

#define VENDOR_ID               0x4953 /* ASCII encoded IS */
#define PRODUCT_ID              0x3161 /* ASCII encoded 1a */
#define WIFI_MIN_CH_SWITCH_CNT       8

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* MAC MIB Access Functions */

/* SMT Station Configuration Table Access Functions */

/* The purpose of dot11StationID is to allow a manager to identify a station */
/* for its own purposes. This attribute provides for that eventuality while  */
/* keeping the true MAC address independent. The default value is the STA's  */
/* assigned, unique MAC address.                                             */
INLINE UWORD8* mget_StationID(void)
{
    return(MIBM.dot11StationID);
}

INLINE void mset_StationID(UWORD8* inp)
{
    memcpy(MIBM.dot11StationID, inp, 6);
}

/* This attribute shall indicate the maximum amount of time, in TU, that a   */
/* point coordinator may control the usage of the wireless medium without    */
/* relinquishing control for long enough to allow at least one instance of   */
/* DCF access to the medium. The default value of this attribute shall be 100*/
/* and the maximum value shall be 1000.                                      */
INLINE UWORD16 mget_MediumOccupancyLimit(void)
{
    return(MIBM.dot11MediumOccupancyLimit);
}

INLINE void mset_MediumOccupancyLimit(UWORD16 inp)
{
    MIBM.dot11MediumOccupancyLimit = inp;
}

/* When this attribute is true, it shall indicate that the STA is able to    */
/* respond to a CF-Poll with a data frame within a SIFS time. This attribute */
/* shall be false if the STA is not able to respond to a CF-Poll with a data */
/* frame within a SIFS time.                                                 */
INLINE TRUTH_VALUE_T mget_CFPollable(void)
{
    return(MIBM.dot11CFPollable);
}

INLINE void mset_CFPollable(TRUTH_VALUE_T inp)
{
    MIBM.dot11CFPollable = inp;
}

/* The attribute shall describe the number of DTIM intervals between the     */
/* start of CFPs. It is modified by MLME-START.request primitive.            */
INLINE UWORD8 mget_CFPPeriod(void)
{
    return(MIBM.dot11CFPPeriod);
}

INLINE void mset_CFPPeriod(UWORD8 inp)
{
    MIBM.dot11CFPPeriod = inp;
}

/* The attribute shall describe the maximum duration of the CFP in TU that   */
/* may be generated by the PCF. Modified by MLME-START.request primitive.    */
INLINE UWORD16 mget_CFPMaxDuration(void)
{
    return(MIBM.dot11CFPMaxDuration);
}

INLINE void mset_CFPMaxDuration(UWORD16 inp)
{
    MIBM.dot11CFPMaxDuration = inp;
}

/* This attribute shall specify the number of TUs that responding STA should */
/* wait for the next frame in the authentication sequence.                   */
INLINE UWORD32 mget_AuthenticationResponseTimeOut(void)
{
#ifdef IBSS_BSS_STATION_MODE
    // Modify by Yiming.Li at 2014-03-10 for fix bug283595: connect difficultly
    if(g_wifi_bt_coex)
		MIBM.dot11AuthenticationResponseTimeOut = 163;  //wzl fix 163ms for coex retransmit        
    else
#endif /*IBSS_BSS_STATION_MODE*/
		MIBM.dot11AuthenticationResponseTimeOut = 512;  /* 512ms  in default*/

    return(MIBM.dot11AuthenticationResponseTimeOut);
}

INLINE void mset_AuthenticationResponseTimeOut(UWORD32 inp)
{
    MIBM.dot11AuthenticationResponseTimeOut = inp;
}

/* This attribute, when true, shall indicate that the IEEE 802.11 WEP option */
/* is implemented. The default value of this attribute shall be false.       */
INLINE TRUTH_VALUE_T mget_PrivacyOptionImplemented(void)
{
    return(MIBM.dot11PrivacyOptionImplemented);
}

INLINE void mset_PrivacyOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBM.dot11PrivacyOptionImplemented = inp;
}

/* This attribute shall specify the power management mode of the STA. When   */
/* set to active, it shall indicate that the station is not in power-save    */
/*  mode. When set to powersave, it shall indicate that the station is in    */
/* power-save mode. The power management mode is transmitted in all frames   */
/* according to the rules in 7.1.3.1.7.                                      */
/* active(1), powersave(2)                                                   */
INLINE UWORD8 mget_PowerManagementMode(void)
{
    return(MIBM.dot11PowerManagementMode);
}

//caisf add 2013-02-19
#define PM_MODE_INT2STR(inp) (inp == 2 ? "MIB_POWERSAVE" : \
                       (inp == 1 ? "MIB_ACTIVE" : "UNKNOWN PM STAT"))

INLINE void mset_PowerManagementMode(UWORD8 inp)
{
    TROUT_DBG5("PS: mset_PowerManagementMode from %s to %s\n", 
        PM_MODE_INT2STR(MIBM.dot11PowerManagementMode),PM_MODE_INT2STR(inp));

    MIBM.dot11PowerManagementMode = inp;
}

/* This attribute reflects the Service Set ID used in DesiredSSID parameter  */
/* of the most recent MLME_Scan.request. This value may be modified by an    */
/* external management entity and used by the local SME to make decisions    */
/* about the Scanning process.                                               */
INLINE WORD8* mget_DesiredSSID(void)
{
    return(MIBM.dot11DesiredSSID);
}

INLINE void mset_DesiredSSID(const WORD8* inp)
{
    UWORD8 ssid_len = strnlen(inp, MAX_SSID_LEN);

    #if 0 //chenq add for clear ssid
    memcpy(MIBM.dot11DesiredSSID,inp,ssid_len);
    MIBM.dot11DesiredSSID[ssid_len] = '\0';
    #else
    if( ssid_len == 0 )
    {
        memset(MIBM.dot11DesiredSSID,0x00,MAX_SSID_LEN);
    }
    else
    {
        memcpy(MIBM.dot11DesiredSSID,inp,ssid_len);
        MIBM.dot11DesiredSSID[ssid_len] = '\0';
    }
    TROUT_DBG4("already set essid:%s\n",MIBM.dot11DesiredSSID);
    #endif
}

/* This attribute shall specify the type of BSS the station shall use when   */
/* scanning for a BSS with which to synchronize. This value is used to filter*/
/* Probe Response frames and Beacons. When set to infrastructure, the STA    */
/* shall only synchronize with a BSS whose Capability Information field has  */
/* ESS subfield set to 1. When set to independent, the station shall only    */
/* synchronize with a BSS whose Capability Information field has the IBSS    */
/* subfield set to 1. When set to any, the station may synchronize to either */
/* type of BSS. infrastructure(1), independent(2), any(3)                    */
INLINE UWORD8 mget_DesiredBSSType(void)
{
    return(MIBM.dot11DesiredBSSType);
}

INLINE void mset_DesiredBSSType(UWORD8 inp)
{
    MIBM.dot11DesiredBSSType = inp;
}

/* This attribute shall specify the set of data rates at which the station   */
/* may transmit data. Each octet contains a value representing a rate. Each  */
/* rate shall be within the range from 2 to 127, corresponding to data rates */
/* in increments of 500 kb/s from 1 Mbit/s to 63.5 Mbit/s, and shall be      */
/* supported (as indicated in the supported rates table) for receiving data. */
/* This value is reported in transmitted Beacon, Probe Request, Probe Rsp,   */
/* Association Request, Association Response,Reassociation Request, and      */
/* Reassociation Response frames, and is used to determine whether a BSS with*/
/* which the station desires to synchronize is suitable. It is also used when*/
/*  starting a BSS, as specified in 10.3.                                    */
INLINE rate_t mget_OperationalRateSet(void)
{
    return(MIBM.dot11OperationalRateSet);
}

INLINE void mset_OperationalRateSet(rate_t inp)
{
    MIBM.dot11OperationalRateSet = inp;
}

/* This attribute shall specify the number of TUs that a station shall use   */
/* for scheduling Beacon transmissions. This value is transmitted in Beacon  */
/* and Probe Response frames.                                                */
INLINE UWORD16 mget_BeaconPeriod(void)
{
    return(MIBM.dot11BeaconPeriod);
}

/* This attribute shall specify the number of beacon intervals that shall    */
/* elapse between transmission of Beacons frames containing a TIM element    */
/* whose DTIM Count field is 0. This value is transmitted in the DTIM Period */
/* field of Beacon frames.                                                   */
INLINE UWORD8 mget_DTIMPeriod(void)
{
    return(MIBM.dot11DTIMPeriod);
}

INLINE void mset_DTIMPeriod(UWORD8 inp)
{
    if(inp != 0)
    {
        MIBM.dot11DTIMPeriod = inp;
    }
}

/* This attribute shall specify the number of TUs that requesting STA should */
/* wait for a response to a transmitted association-request MMPDU.           */
INLINE UWORD32 mget_AssociationResponseTimeOut(void)
{
#ifdef IBSS_BSS_STATION_MODE
    // Modify by Yiming.Li at 2014-03-10 for fix bug283595: connect difficultly
    if(g_wifi_bt_coex)
		MIBM.dot11AssociationResponseTimeOut = 163;  //wzl fix 163ms for coex retransmit        
    else
#endif  /*IBSS_BSS_STATION_MODE*/
		MIBM.dot11AssociationResponseTimeOut = 512;  /* 512ms  in default*/
	
    return(MIBM.dot11AssociationResponseTimeOut);
}

INLINE void mset_AssociationResponseTimeOut(UWORD32 inp)
{
    MIBM.dot11AssociationResponseTimeOut = inp;
}

/* This attribute holds the most recently transmitted Reason Code in Disasoc */
/* frame. If no Disassociation frame has been transmitted, the value of this */
/* attribute shall be 0.                                                     */
INLINE UWORD16 mget_DisassociateReason(void)
{
    return(MIBM.dot11DisassociateReason);
}

INLINE void mset_DisassociateReason(UWORD16 inp)
{
    MIBM.dot11DisassociateReason = inp;
}

/* This attribute holds the MAC address from the Address 1 field of the most */
/* recently transmitted Disassociation frame. If no Disassociation frame has */
/* been transmitted, the value of this attribute shall be 0.                 */
INLINE UWORD8* mget_DisassociateStation(void)
{
    return(MIBM.dot11DisassociateStation);
}

INLINE void mset_DisassociateStation(UWORD8* inp)
{
    memcpy(MIBM.dot11DisassociateStation, inp, 6);
}

/* This attribute holds the most recently transmitted Reason Code in a Deauth*/
/* frame. If no Deauthentication frame has been transmitted, value of this   */
/* attribute shall be 0.                                                     */
INLINE UWORD16 mget_DeauthenticateReason(void)
{
    return(MIBM.dot11DeauthenticateReason);
}

INLINE void mset_DeauthenticateReason(UWORD16 inp)
{
    MIBM.dot11DeauthenticateReason = inp;
}

/* This attribute holds the MAC address from the Address 1 field of the most */
/* recently transmitted Deauthentication frame. If no Deauthentication frame */
/* has been transmitted, the value of this attribute shall be 0.             */
INLINE UWORD8* mget_DeauthenticateStation(void)
{
    return(MIBM.dot11DeauthenticateStation);
}

INLINE void mset_DeauthenticateStation(UWORD8* inp)
{
    memcpy(MIBM.dot11DeauthenticateStation, inp, 6);
}

/* This attribute holds the most recently transmitted Status Code in a failed*/
/* Authentication frame. If no failed Auth frame has been transmitted, the   */
/* value of this attribute shall be 0.                                       */
INLINE UWORD16 mget_AuthenticateFailStatus(void)
{
    return(MIBM.dot11AuthenticateFailStatus);
}

INLINE void mset_AuthenticateFailStatus(UWORD16 inp)
{
    MIBM.dot11AuthenticateFailStatus = inp;
}

/* This attribute holds the MAC address from the Address 1 field of the most */
/* recently transmitted failed Authentication frame. If no failed Auth frame */
/* has been transmitted, the value of this attribute shall be 0.             */
INLINE UWORD8* mget_AuthenticateFailStation(void)
{
    return(MIBM.dot11AuthenticateFailStation);
}

INLINE void mset_AuthenticateFailStation(UWORD8* inp)
{
    memcpy(MIBM.dot11AuthenticateFailStation, inp, 6);
}

/* This attribute shall specify whether spectrum management is
presently implemented  */
INLINE void mset_SpectrumManagementRequired(TRUTH_VALUE_T inp)
{
    MIBM.dot11SpectrumManagementRequired  =   inp;
}

/* Function to get spectrum management required bit */
INLINE TRUTH_VALUE_T mget_SpectrumManagementRequired(void)
{
    return(MIBM.dot11SpectrumManagementRequired);
}

/* Function to set spectrum management imlmented */
INLINE void mset_SpectrumManagementImplemented(TRUTH_VALUE_T inp)
{
    MIBM.dot11SpectrumManagementImplemented  =   inp;
}

/* Function to get spectrum management implemented bit */
INLINE TRUTH_VALUE_T mget_SpectrumManagementImplemented(void)
{
    return(MIBM.dot11SpectrumManagementImplemented);
}

/* Authentication Algorithms Table Access Functions */

/* This attribute shall be a set of all authentication algorithms supported  */
/* by the STAs. The following are default values and the associated algorithm*/
/* Value = 1: Open System, Value = 2: Shared Key                             */
INLINE UWORD8 mget_AuthenticationAlgorithm(UWORD8 alg_num)
{
    return(MIBM.dot11AuthenticationAlgorithmsTable[alg_num].
                                    dot11AuthenticationAlgorithm);
}

INLINE void mset_AuthenticationAlgorithm(UWORD8 inp, UWORD8 alg_num)
{
    MIBM.dot11AuthenticationAlgorithmsTable[alg_num].
                                    dot11AuthenticationAlgorithm = inp;
}

/* This attribute, when true at a station, shall enable acceptance of auth   */
/* algorithm described in the corresponding table entry in authentication    */
/* frames received by the station that have odd authentication sequence nums */
/* The default value of this attribute shall be 1 for the Open System table  */
/* entry and 2 for all other table entries.                                  */
INLINE TRUTH_VALUE_T mget_AuthenticationAlgorithmsEnable(UWORD8 an)
{
    return(MIBM.dot11AuthenticationAlgorithmsTable[an].
                                    dot11AuthenticationAlgorithmsEnable);
}

INLINE void mset_AuthenticationAlgorithmsEnable(TRUTH_VALUE_T inp,
                                                           UWORD8 alg_num)
{
    MIBM.dot11AuthenticationAlgorithmsTable[alg_num].
                                    dot11AuthenticationAlgorithmsEnable = inp;
}

/* WEP Default Keys Table Access Functions */

/* This attribute shall indicate use of the first, second, third, or fourth  */
/* element of the WEPDefaultKeys array when set to values of zero, one, two, */
/* or three. The default value of this attribute shall be 0.                 */
/* REFERENCE "ISO/IEC 8802-11:1999, 8.3.2"                                   */
INLINE UWORD8 mget_WEPDefaultKeyID(void)
{
    return(MIBM.dot11WEPDefaultKeyID);
}

/* Function to set WEP default key ID */
INLINE void mset_WEPDefaultKeyID(UWORD8 inp)
{
    if(inp < NUM_DOT11WEPDEFAULTKEYVALUE)
        MIBM.dot11WEPDefaultKeyID = inp;
}

/* A WEP default secret key value. Its initial value is not specified. WEP   */
/* Key is stored in an array of bytes, with the first byte storing the size  */
/* of the key (in number of bytes).                                          */
INLINE UWORD8* mget_WEPKeyValue(UWORD8 idx)
{
    return(&MIBM.dot11WEPDefaultKeyValue[idx][WEP_KEY_VALUE_OFFSET]);
}


INLINE UWORD8 mget_WEPKeySize(UWORD8 idx)
{
    return(MIBM.dot11WEPDefaultKeyValue[idx][WEP_SIZE_OFFSET]);
}


INLINE void mset_WEPKeyValue(UWORD8 idx, UWORD8* inp, UWORD8 size)
{
    MIBM.dot11WEPDefaultKeyValue[idx][WEP_SIZE_OFFSET] = size;
    switch(size)
    {
        case 40:
            size = 5;
            break;
        case 104:
            size = 13;
            break;
        default:
            size = 5;
            break;
    }
    memcpy(&MIBM.dot11WEPDefaultKeyValue[idx][WEP_KEY_VALUE_OFFSET], inp,
           size);
}

/* This function gets the current WEP key in use */
INLINE UWORD8* mget_WEPDefaultKeyValue(void)
{
    return(&MIBM.dot11WEPDefaultKeyValue[mget_WEPDefaultKeyID()]
                                        [WEP_KEY_VALUE_OFFSET]);
}

INLINE UWORD8 mget_WEPDefaultKeySize(void)
{
    return(MIBM.dot11WEPDefaultKeyValue[mget_WEPDefaultKeyID()]
                                       [WEP_SIZE_OFFSET]);
}


/* WEP Kep Mappings Table Access Functions */

/* The MAC address of the STA for which values from this key mapping entry   */
/* are to be used.                                                           */
INLINE UWORD8* mget_WEPKeyMappingAddress(UWORD8 idx)
{
    return(MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingAddress);
}

INLINE void mset_WEPKeyMappingAddress(UWORD8* inp, UWORD8 idx)
{
    memcpy(MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingAddress,
           inp, 6);
}

/* Boolean as to whether WEP is to be used when communicating with the       */
/* dot11WEPKeyMappingAddress STA.                                            */
INLINE TRUTH_VALUE_T mget_WEPKeyMappingWEPOn(UWORD8 idx)
{
    return(MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingWEPOn);
}

INLINE void mset_WEPKeyMappingWEPOn(TRUTH_VALUE_T inp, UWORD8 idx)
{
    MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingWEPOn = inp;
}

/* A WEP secret key value. */
INLINE UWORD8* mget_WEPKeyMappingValue(UWORD8 idx)
{
    return(MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingValue);
}

INLINE void mset_WEPKeyMappingValue(UWORD8* inp, UWORD8 idx)
{
    memcpy(MIBM.dot11WEPKeyMappingsTable[idx].dot11WEPKeyMappingValue,
           inp, inp[WEP_SIZE_OFFSET] + 1);
}

/* Privacy Table Access Functions */

/* When this attribute is true, it shall indicate that the IEEE 802.11 WEP   */
/* mechanism is used for transmitting frames of type Data. The default value */
/* of this attribute shall be false.                                         */
INLINE TRUTH_VALUE_T mget_PrivacyInvoked(void)
{
    return(MIBM.dot11PrivacyInvoked);
}

INLINE void mset_PrivacyInvoked(TRUTH_VALUE_T inp)
{
    MIBM.dot11PrivacyInvoked = inp;
}

/* The maximum number of tuples that dot11WEPKeyMappings can hold.           */
/* REFERENCE "ISO/IEC 8802-11:1999, 8.3.2"                                   */
INLINE UWORD32 mget_WEPKeyMappingLength(void)
{
    return(MIBM.dot11WEPKeyMappingLength);
}

INLINE void mset_WEPKeyMappingLength(UWORD32 inp)
{
    MIBM.dot11WEPKeyMappingLength = inp;
}

/* When this attribute is true, the STA shall not indicate at the MAC service*/
/* interface received MSDUs that have the WEP subfield of the Frame Control  */
/* field equal to zero. When this attribute is false, STA may accept MSDUs   */
/* that have the WEP subfield of the Frame Control field equal to zero. The  */
/* default value of this attribute shall be false.                           */
INLINE TRUTH_VALUE_T mget_ExcludeUnencrypted(void)
{
    return(MIBM.dot11ExcludeUnencrypted);
}

INLINE void mset_ExcludeUnencrypted(TRUTH_VALUE_T inp)
{
    MIBM.dot11ExcludeUnencrypted = inp;
}

/* This counter shall increment when a frame is received with WEP subfield   */
/* of the Frame Control field set to one and the value of the ICV as received*/
/* in the frame does not match the ICV value that is calculated for contents */
/* of the received frame.                                                    */
INLINE UWORD32 mget_WEPICVErrorCount(void)
{
    return(MIBM.dot11WEPICVErrorCount);
}

INLINE void mset_WEPICVErrorCount(UWORD32 inp)
{
    MIBM.dot11WEPICVErrorCount = inp;
}

INLINE void mincr_WEPICVErrorCount(void)
{
    MIBM.dot11WEPICVErrorCount++;
}

/* This counter shall increment when a frame is received with WEP subfield of*/
/* the Frame Control field set to zero and value of dot11ExcludeUnencrypted  */
/* causes that frame to be discarded.                                        */
INLINE UWORD32 mget_WEPExcludedCount(void)
{
    return(MIBM.dot11WEPExcludedCount);
}

INLINE void mset_WEPExcludedCount(UWORD32 inp)
{
    MIBM.dot11WEPExcludedCount = inp;
}

INLINE void mincr_WEPExcludedCount(void)
{
    MIBM.dot11WEPExcludedCount++;
}


/* SMT Notication Access Functions */

/* The disassociate notification shall be sent when the STA sends a Disasoc  */
/* frame. The value of the notification shall include the MAC address of the */
/* MAC to which the Disassociation frame was sent and the reason for the     */
/* disassociation.                                                           */
INLINE notification_t mget_Disassociate(void)
{
    MIBM.dot11Disassociate.reason  = mget_DisassociateReason();
    memcpy(MIBM.dot11Disassociate.station, mget_DisassociateStation(), 6);

    return(MIBM.dot11Disassociate);
}

INLINE void mset_Disassociate(notification_t inp)
{
    MIBM.dot11Disassociate.reason  = inp.reason;
    memcpy(MIBM.dot11Disassociate.station, inp.station, 6);
}

/* The deauthenticate notification shall be sent when the STA sends a Deauth */
/* frame. The value of the notification shall include the MAC address of the */
/* MAC to which the Deauthentication frame was sent and the reason for the   */
/* deauthentication.                                                         */
INLINE notification_t mget_Deauthenticate(void)
{
    return(MIBM.dot11Deauthenticate);
}

INLINE void mset_Deauthenticate(notification_t inp)
{
    MIBM.dot11Deauthenticate.reason  = inp.reason;
    memcpy(MIBM.dot11Deauthenticate.station, inp.station, 6);
}

/* The authenticate failure notification shall be sent when the STA sends an */
/* Authentication frame with a status code other than Successful.The value*/
/* of the notification shall include the MAC address of the MAC to which the */
/* Authentication frame was sent and the reason for authentication failure.  */
INLINE notification_t mget_AuthenticateFail(void)
{
    return(MIBM.dot11AuthenticateFail);
}

INLINE void mset_AuthenticateFail(notification_t inp)
{
    MIBM.dot11AuthenticateFail.reason  = inp.reason;
    memcpy(MIBM.dot11AuthenticateFail.station, inp.station, 6);
}

/* Operation Table Access Functions */

/* Unique MAC Address assigned to the STA. */
INLINE UWORD8* mget_MACAddress(void)
{
    return(MIBM.dot11MACAddress);
}

INLINE void mset_MACAddress(UWORD8* inp)
{
    memcpy(MIBM.dot11MACAddress, inp, 6);
}

/* This attribute shall indicate the number of octets in an MPDU, below which*/
/* an RTS/CTS handshake shall not be performed. An RTS/CTS handshake shall be*/
/* performed at the beginning of any frame exchange sequence where the MPDU  */
/* is of type Data or Management, the MPDU has an individual address in the  */
/* Address1 field, and the length of the MPDU is greater than this threshold.*/
/* (For additional details, refer to Table 21 in 9.7.) Setting this attribute*/
/* to be larger than the maximum MSDU size shall have the effect of turning  */
/* off RTS/CTS handshake for frames of Data or Management type transmitted   */
/* by this STA. Setting this attribute to zero shall have the effect of      */
/* turning on the RTS/CTS handshake for all frames of Data or Management     */
/* type transmitted by this STA. The default value of this attribute shall   */
/* be 2347.                                                                  */
INLINE UWORD16 mget_RTSThreshold(void)
{
    return(MIBM.dot11RTSThreshold);
}

/* This attribute shall indicate the maximum number of transmission attempts */
/* of a frame, the length of which is less than or equal to dot11RTSThreshold*/
/* that shall be made before a failure condition is indicated. The default   */
/* value of this attribute shall be 7.                                       */
INLINE UWORD8 mget_ShortRetryLimit(void)
{
    return(MIBM.dot11ShortRetryLimit);
}

/* This attribute shall indicate the maximum number of transmission attempts */
/* of a frame, the length of which is greater than dot11RTSThreshold, that   */
/* shall be made before a failure condition is indicated. The default value  */
/* of this attribute shall be 4.                                             */
INLINE UWORD8 mget_LongRetryLimit(void)
{
    return(MIBM.dot11LongRetryLimit);
}

/* This attribute shall specify the current maximum size, in octets, of the  */
/* MPDU that may be delivered to PHY. An MSDU shall be broken into fragments */
/* if its size exceeds the value of this attribute after adding MAC headers  */
/* and trailers. An MSDU or MMPDU shall be fragmented when resulting frame   */
/* has an individual address in the Address1 field, and the length of the    */
/* frame is larger than this threshold. The default value for this attribute */
/* shall be the lesser of 2346 or the aMPDUMaxLength of the attached PHY and */
/* shall never exceed the lesser of 2346 or aMPDUMaxLength of the attached   */
/* PHY. The value of this attribute shall never be less than 256.            */
INLINE UWORD16 mget_FragmentationThreshold(void)
{
    return(MIBM.dot11FragmentationThreshold);
}

INLINE void mset_FragmentationThreshold(UWORD16 inp)
{
    MIBM.dot11FragmentationThreshold = inp;
}

/* The MaxTransmitMSDULifetime shall be the elapsed time in TU, after initial*/
/* transmission of MSDU, after which further attempts to transmit the MSDU   */
/* shall be terminated. The default value of this attribute shall be 512.    */
INLINE UWORD32 mget_MaxTransmitMSDULifetime(void)
{
    return(MIBM.dot11MaxTransmitMSDULifetime);
}

INLINE void mset_MaxTransmitMSDULifetime(UWORD32 inp)
{
    MIBM.dot11MaxTransmitMSDULifetime = inp;
}

/* The MaxReceiveLifetime shall be the elapsed time in TU, after the initial */
/* reception of a fragmented MMPDU or MSDU, after which further attempts to  */
/* reassemble the MMPDU or MSDU shall be terminated. The default value shall */
/* be 512.                                                                   */
INLINE UWORD32 mget_MaxReceiveLifetime(void)
{
    return(MIBM.dot11MaxReceiveLifetime);
}

INLINE void mset_MaxReceiveLifetime(UWORD32 inp)
{
    MIBM.dot11MaxReceiveLifetime = inp;
}

/* The ManufacturerID shall include, at a minimum, the name of manufacturer. */
/* It may include additional information at the manufacturer's discretion.   */
/* The default value of this attribute shall be null.                        */
INLINE WORD8* mget_ManufacturerID(void)
{
    return(MIBM.dot11ManufacturerID);
}

INLINE void mset_ManufacturerID(WORD8* inp)
{
    memcpy(MIBM.dot11ManufacturerID, inp, MAX_ID_LEN);
}

/* The ProductID shall include, at a minimum, an identifier that is unique to*/
/* the manufacturer. It may include additional information at manufacturer's */
/* discretion. The default value of this attribute shall be null.            */
INLINE WORD8* mget_ProductID(void)
{
    return(MIBM.dot11ProductID);
}

INLINE void mset_ProductID(UWORD8* inp)
{
    memcpy(MIBM.dot11ProductID, inp, MAX_ID_LEN);
}

/* Counters Table Access Functions */

/* This counter shall be incremented for an acknowledged MPDU with an        */
/* individual address in the address 1 field or an MPDU with a multicast     */
/* address in the address 1 field of type Data or Management.                */
INLINE UWORD32 mget_TransmittedFragmentCount(void)
{
    return(MIBM.dot11TransmittedFragmentCount);
}

INLINE void mset_TransmittedFragmentCount(UWORD32 inp)
{
    MIBM.dot11TransmittedFragmentCount = inp;
}

/* This counter shall increment only when the multicast bit is set in dest   */
/* MAC address of a successfully transmitted MSDU. When operating as a STA   */
/* in an ESS, where these frames are directed to the AP, this implies having */
/* received an acknowledgment to all associated MPDUs.                       */
INLINE UWORD32 mget_MulticastTransmittedFrameCount(void)
{
    return(MIBM.dot11MulticastTransmittedFrameCount);
}

INLINE void mset_MulticastTransmittedFrameCount(UWORD32 inp)
{
    MIBM.dot11MulticastTransmittedFrameCount = inp;
}

/* This counter shall increment when an MSDU is not transmitted successfully */
/* due to number of transmit attempts exceeding either dot11ShortRetryLimit  */
/* or dot11LongRetryLimit.                                                   */
INLINE UWORD32 mget_FailedCount(void)
{
    return(MIBM.dot11FailedCount);
}

INLINE void mset_FailedCount(UWORD32 inp)
{
    MIBM.dot11FailedCount = inp;
}

/* This counter shall increment when an MSDU is successfully transmitted     */
/* after one or more retransmissions.                                        */
INLINE UWORD32 mget_RetryCount(void)
{
    return(MIBM.dot11RetryCount);
}

INLINE void mset_RetryCount(UWORD32 inp)
{
    MIBM.dot11RetryCount = inp;
}

/* This counter shall increment when an MSDU is successfully transmitted     */
/* after more than one retransmission.                                       */
INLINE UWORD32 mget_MultipleRetryCount(void)
{
    return(MIBM.dot11MultipleRetryCount);
}

INLINE void mset_MultipleRetryCount(UWORD32 inp)
{
    MIBM.dot11MultipleRetryCount = inp;
}

/* This counter shall increment when a frame is received that the Sequence   */
/* Control field indicates is a duplicate.                                   */
INLINE UWORD32 mget_FrameDuplicateCount(void)
{
    return(MIBM.dot11FrameDuplicateCount);
}

INLINE void mset_FrameDuplicateCount(UWORD32 inp)
{
    MIBM.dot11FrameDuplicateCount = inp;
}

/* This counter shall increment when a CTS is received in response to an RTS */
INLINE UWORD32 mget_RTSSuccessCount(void)
{
    return(MIBM.dot11RTSSuccessCount);
}

INLINE void mset_RTSSuccessCount(UWORD32 inp)
{
    MIBM.dot11RTSSuccessCount = inp;
}

/* This counter shall increment when a CTS is not received in response to an */
/* RTS.                                                                      */
INLINE UWORD32 mget_RTSFailureCount(void)
{
    return(MIBM.dot11RTSFailureCount);
}

INLINE void mset_RTSFailureCount(UWORD32 inp)
{
    MIBM.dot11RTSFailureCount = inp;
}

/* This counter shall increment when an ACK is not received when expected.   */
INLINE UWORD32 mget_ACKFailureCount(void)
{
    return(MIBM.dot11ACKFailureCount);
}

INLINE void mset_ACKFailureCount(UWORD32 inp)
{
    MIBM.dot11ACKFailureCount = inp;
}

/* This counter shall be incremented for each successfully received MPDU of  */
/* type Data or Management.                                                  */
INLINE UWORD32 mget_ReceivedFragmentCount(void)
{
    return(MIBM.dot11ReceivedFragmentCount);
}

INLINE void mset_ReceivedFragmentCount(UWORD32 inp)
{
    MIBM.dot11ReceivedFragmentCount = inp;
}

INLINE void mincr_ReceivedFragmentCount(void)
{
    MIBM.dot11ReceivedFragmentCount++;
}

/* This counter shall increment when a MSDU is received with the multicast   */
/* bit set in the destination MAC address.                                   */
INLINE UWORD32 mget_MulticastReceivedFrameCount(void)
{
    return(MIBM.dot11MulticastReceivedFrameCount);
}

INLINE void mset_MulticastReceivedFrameCount(UWORD32 inp)
{
    MIBM.dot11MulticastReceivedFrameCount = inp;
}

INLINE void mincr_MulticastReceivedFrameCount(void)
{
    MIBM.dot11MulticastReceivedFrameCount++;
}

/* This counter shall increment when an FCS error is detected in a received  */
/* MPDU.                                                                     */
INLINE UWORD32 mget_FCSErrorCount(void)
{
    return(MIBM.dot11FCSErrorCount);
}

INLINE void mset_FCSErrorCount(UWORD32 inp)
{
    MIBM.dot11FCSErrorCount = inp;
}

/* This counter shall increment for each successfully transmitted MSDU.      */
INLINE UWORD32 mget_TransmittedFrameCount(void)
{
    return(MIBM.dot11TransmittedFrameCount);
}

INLINE void mset_TransmittedFrameCount(UWORD32 inp)
{
    MIBM.dot11TransmittedFrameCount = inp;
}

/* This counter shall increment when a frame is received with WEP subfield of*/
/* the Frame Control field set to one and the WEPOn value for the key mapped */
/* to the TA's MAC address indicates that the frame should not have been     */
/* encrypted or that frame is discarded due to receiving STA not implementing*/
/* the privacy option.                                                       */
INLINE UWORD32 mget_WEPUndecryptableCount(void)
{
    return(MIBM.dot11WEPUndecryptableCount);
}

INLINE void mset_WEPUndecryptableCount(UWORD32 inp)
{
    MIBM.dot11WEPUndecryptableCount = inp;
}

/* Group Addresses Table Access Functions */
/* MAC address identifying a multicast addresses from which this STA will    */
/* receive frames.                                                           */
INLINE UWORD8* mget_GroupAddress(void)
{
    return(MIBM.dot11GroupAddressTable);
}

INLINE void mset_GroupAddress(UWORD8* inp)
{
    memcpy(MIBM.dot11GroupAddressTable, inp,
           inp[GROUP_ADDRESS_SIZE_OFFSET]*6 + GROUP_ADDRESS_VALUE_OFFSET);
}

/* Resource Type Attribute Templates Access Functions */

/* NOTE: currently unused and hence no access fucntions are provided for the */
/* dot11ResourceTypeIDName                                                   */

/* Resource info Table Access Functions */

/* Takes the value of an organizationally unique identifier. */
INLINE UWORD16 mget_manufacturerOUI(void)
{
    return(MIBM.dot11manufacturerOUI);
}

INLINE void mset_manufacturerOUI(UWORD16 inp)
{
    MIBM.dot11manufacturerOUI = inp;
}

/* A printable string used to identify the manufacturer of the resource.     */
/* Maximum string length is 128 octets.                                      */
INLINE UWORD8* mget_manufacturerName(void)
{
    return(MIBM.dot11manufacturerName);
}

INLINE void mset_manufacturerName(UWORD8* inp)
{
    memcpy(MIBM.dot11manufacturerName, inp, NUM_DOT11MANUFACTURERNAME);
}

/* A printable string used to identify the manufacturer's product name of the*/
/* resource. Maximum string length is 128 octets.                            */
INLINE UWORD16 mget_manufacturerProductName(void)
{
    return(MIBM.dot11manufacturerProductName);
}

INLINE void mset_manufacturerProductName(UWORD16 inp)
{
    MIBM.dot11manufacturerProductName = inp;
}

/* Printable string used to identify the manufacturer's product version of   */
/* resource. Maximum string length is 128 octets.                            */
INLINE UWORD8* mget_manufacturerProductVersion(void)
{
    return(MIBM.dot11manufacturerProductVersion);
}

INLINE void mset_manufacturerProductVersion(UWORD8* inp)
{
    strcpy((char*)MIBM.dot11manufacturerProductVersion, (char*)inp);
}


/* This attribute shall indicate the maximum number of transmission attempts */
/* of a frame, the length of which is less than or equal to dot11RTSThreshold*/
/* that shall be made before a failure condition is indicated. The default   */
/* value of this attribute shall be 7.                                       */
INLINE void mset_ShortRetryLimit(UWORD8 inp)
{
    MIBM.dot11ShortRetryLimit = inp;
}

/* This attribute shall indicate the maximum number of transmission attempts */
/* of a frame, the length of which is greater than dot11RTSThreshold, that   */
/* shall be made before a failure condition is indicated. The default value  */
/* of this attribute shall be 4.                                             */
INLINE void mset_LongRetryLimit(UWORD8 inp)
{
    MIBM.dot11LongRetryLimit = inp;
}

/* This attribute shall indicate the number of octets in an MPDU, below which*/
/* an RTS/CTS handshake shall not be performed. An RTS/CTS handshake shall be*/
/* performed at the beginning of any frame exchange sequence where the MPDU  */
/* is of type Data or Management, the MPDU has an individual address in the  */
/* Address1 field, and the length of the MPDU is greater than this threshold.*/
/* (For additional details, refer to Table 21 in 9.7.) Setting this attribute*/
/* to be larger than the maximum MSDU size shall have the effect of turning  */
/* off RTS/CTS handshake for frames of Data or Management type transmitted   */
/* by this STA. Setting this attribute to zero shall have the effect of      */
/* turning on the RTS/CTS handshake for all frames of Data or Management     */
/* type transmitted by this STA. The default value of this attribute shall   */
/* be 2347.                                                                  */
INLINE void mset_RTSThreshold(UWORD16 inp)
{
    MIBM.dot11RTSThreshold = inp;
}

/* This attribute shall specify the number of TUs that a station shall use   */
/* for scheduling Beacon transmissions. This value is transmitted in Beacon  */
/* and Probe Response frames.                                                */
INLINE void mset_BeaconPeriod(UWORD16 inp)
{
    if(inp > 0)
        MIBM.dot11BeaconPeriod = inp;
}
#endif /* MIB_H */
