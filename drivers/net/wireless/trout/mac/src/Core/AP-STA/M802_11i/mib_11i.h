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
/*  File Name         : mib_11i.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB as given in Annex-D of 802.11 i       */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all 802.11I related MIB         */
/*                      parameters.                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef MIB_11I_H
#define MIB_11I_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "itypes.h"
#include "mib1.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* Number of Cipher Suites Implemented */
#define PAIRWISE_CIPHER_SUITES            2
#define AUTHENTICATION_SUITES             1

#define MAX_PSK_PASS_PHRASE_LEN           64
/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Additions as per the 802.11i Standard */
typedef struct
{
    UWORD32 dot11RSNAConfigPairwiseCipherIndex;
    UWORD8  dot11RSNAConfigPairwiseCipher;
    TRUTH_VALUE_T dot11RSNAConfigPairwiseCipherEnabled;
    UWORD32 dot11RSNAConfigPairwiseCipherSize;
} dot11RSNAConfigPairwiseCiphersEntry_t;

typedef struct
{
    UWORD32 dot11RSNAConfigAuthenticationSuiteIndex;
    UWORD8 dot11RSNAConfigAuthenticationSuite;
    TRUTH_VALUE_T dot11RSNAConfigAuthenticationSuiteEnabled;
} dot11RSNAConfigAuthenticationSuitesEntry_t;

typedef struct
{
    UWORD32 dot11RSNAStatsIndex;
    UWORD8  dot11RSNAStatsSTAAddress[6];
    UWORD32 dot11RSNAStatsVersion;
    UWORD8  dot11RSNAStatsSelectedPairwiseCipher;
    UWORD32 dot11RSNAStatsTKIPICVErrors;
    UWORD32 dot11RSNAStatsTKIPLocalMICFailures;
    UWORD32 dot11RSNAStatsTKIPRemoteMICFailures;
    UWORD32 dot11RSNAStatsCCMPReplays;
    UWORD32 dot11RSNAStatsCCMPDecryptErrors;
    UWORD32 dot11RSNAStatsTKIPReplays;
} dot11RSNAStatsEntry_t;


typedef struct
{
    TRUTH_VALUE_T  dot11RSNAOptionImplemented;
    TRUTH_VALUE_T  dot11RSNAPreauthenticationImplemented;
    TRUTH_VALUE_T  dot11RSNAEnabled;
    TRUTH_VALUE_T  dot11RSNAPreauthenticationEnabled;

    /* OUI for the RSNA Field 00 - 0F - AC as per the 802.11I standard */
    UWORD8         dot11RSNAOUI[3];
    UWORD8         dot11WPAOUI[3];

    /* 802.11i Standard required addition of 4 new sections within the MIB   */

    /*************************************************************************/
    /* RSNA Config Table - Dot11RSNAConfigTable (RSNA and TSN)               */
    /*************************************************************************/
    /* The table containing RSNA configuration objects.                      */


    /* RSNA Config Entry - dot11RSNAConfigEntry                              */
    /* An Entry in the RSNA Config Table dot11RSNAConfigTable                */

    UWORD32        dot11RSNAConfigVersion;
    UWORD32        dot11RSNAConfigPairwiseKeysSupported;
    UWORD8         dot11RSNAConfigGroupCipher;
    UWORD32        dot11RSNAConfigGroupRekeyMethod;
    UWORD32        dot11RSNAConfigGroupRekeyTime;
    UWORD32        dot11RSNAConfigGroupRekeyPackets;
    TRUTH_VALUE_T  dot11RSNAConfigGroupRekeyStrict;
    UWORD8         dot11RSNAConfigPSKValue[40];
    UWORD8         dot11RSNAConfigPSKPassPhrase[MAX_PSK_PASS_PHRASE_LEN + 1];
    UWORD32        dot11RSNAConfigGroupUpdateCount;
    UWORD32        dot11RSNAConfigPairwiseUpdateCount;
    UWORD32        dot11RSNAConfigGroupCipherSize;
    UWORD32        dot11RSNAConfigPMKLifetime;
    UWORD32        dot11RSNAConfigPMKReauthThreshold;
    UWORD32        dot11RSNAConfigNumberOfPTKSAReplayCounters;
    UWORD32        dot11RSNAConfigSATimeout;
    UWORD8         dot11RSNAAuthenticationSuiteSelected;
    UWORD8         dot11RSNAPairwiseCipherSelected;
    UWORD8         dot11RSNAGroupCipherSelected;
    UWORD8         dot11RSNAPMKIDUsed[16];
    UWORD8         dot11RSNAAuthenticationSuiteRequested;
    UWORD8         dot11RSNAPairwiseCipherRequested;
    UWORD8         dot11RSNAGroupCipherRequested;
    UWORD8         dot11RSNAModeRequested;
    UWORD32        dot11RSNATKIPCounterMeasuresInvoked;
    UWORD32        dot11RSNA4WayHandshakeFailures;
    UWORD32        dot11RSNAConfigNumberOfGTKSAReplayCounters;


    /*************************************************************************/
    /* RSNA Config Pair wise cipher Table - dot11RSNAConfigPairwiseCiphers   */
    /*************************************************************************/
    /* This table lists the pairwise ciphers supported by this entity. It    */
    /* allows enabling and disabling of each pairwise cipher by network      */
    /* management.                                                           */


    /* RSNA Config Pairwise cipher Entry - dot11RSNAConfigPairWiseCipherEntry*/
    /* An Entry in the RSNA Config Pairwise cipher Table                     */
    dot11RSNAConfigPairwiseCiphersEntry_t
        dot11RSNAConfigPairwiseCiphersEntry[PAIRWISE_CIPHER_SUITES];


    /*************************************************************************/
    /* dot11RSNAConfigAuthenticationSuites TABLE                             */
    /*************************************************************************/
    /* This table lists the AKM suites supported by this entity. Each AKM    */
    /* suite can be individually enabled and disabled.                       */
    dot11RSNAConfigAuthenticationSuitesEntry_t
        dot11RSNAConfigAuthenticationSuitesEntry[AUTHENTICATION_SUITES];

    /*************************************************************************/
    /* dot11RSNAStats TABLE                                                  */
    /*************************************************************************/
    /* This table maintains per-STA statistics in an RSN. The entry with     */
    /* dot11RSNAStatsSTAAddress set to FF-FF-FF-FF-FF-FF shall contain       */
    /* statistics for broadcast/multicast traffic                            */
    dot11RSNAStatsEntry_t dot11RSNAStatsEntry[MAX_STA_SUPPORTED + 1];

} mib_11i_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern mib_11i_t MIB_11I;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void initialize_mac_mib_11i(void);
extern BOOL_T check_auth_policy(UWORD8 policy);
extern BOOL_T check_pcip_policy(UWORD8 policy);
extern void set_rsna_policy(UWORD8 policy, TRUTH_VALUE_T val);
extern UWORD8 mget_RSNAStatsIndex(UWORD8 *addr);
extern UWORD8 get_pairwise_cipher_suite(void);
extern void mnew_RSNAStats(UWORD8 index);
extern void mset_RSNAConfigGroupCipher(UWORD8 inp);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/


/* This attribute indicates if the RSNA options is implemented in the STA */
INLINE UWORD8 mget_RSNAOptionImplemented(void)
{
    return MIB_11I.dot11RSNAOptionImplemented;
}

INLINE void mset_RSNAOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11I.dot11RSNAOptionImplemented = inp;
}


/* This attribute indicates if the PRERSNA Authentication option */
/*  is implemented in the STA                                    */
INLINE UWORD8 mget_RSNAPreauthenticationImplemented(void)
{
    return MIB_11I.dot11RSNAPreauthenticationImplemented;
}

INLINE void mset_RSNAPreauthenticationImplemented(TRUTH_VALUE_T inp)
{
    MIB_11I.dot11RSNAPreauthenticationImplemented = inp;
}


/* This attribute indicates if the RSNA options is enabled in the STA */
INLINE UWORD8 mget_RSNAEnabled(void)
{
    return MIB_11I.dot11RSNAEnabled;
}

INLINE void mset_RSNAEnabled(TRUTH_VALUE_T inp)
{
    MIB_11I.dot11RSNAEnabled = inp;
}

/* This attribute indicates if the PRERSNA Authentication options is enabled */
/* in the STA                                                                */
INLINE UWORD8 mget_RSNAPreauthenticationEnabled(void)
{
    return MIB_11I.dot11RSNAPreauthenticationEnabled;
}

INLINE void mset_RSNAPreauthenticationEnabled(TRUTH_VALUE_T inp)
{
    MIB_11I.dot11RSNAPreauthenticationEnabled = inp;
}

/* This attribute indicates RSNA OUI as per the IEEE standard */
INLINE UWORD8* mget_RSNAOUI(void)
{
    return MIB_11I.dot11RSNAOUI;
}

INLINE void mset_RSNAOUI(UWORD8* inp)
{
    memcpy(MIB_11I.dot11RSNAOUI, inp, 3);
}

INLINE UWORD8* mget_WPAOUI(void)
{
    return MIB_11I.dot11WPAOUI;
}

INLINE void mset_WPAOUI(UWORD8* inp)
{
    memcpy(MIB_11I.dot11WPAOUI, inp, 3);
}


/* This attribute indicates if the RSNA version implemented */
/* in the STA                                               */
INLINE UWORD32 mget_RSNAConfigVersion(void)
{
    return MIB_11I.dot11RSNAConfigVersion;
}

INLINE void mset_RSNAConfigVersion(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigVersion = inp;
}

/* This object indicates how many pairwise keys the entity supports for RSNA */
INLINE UWORD32 mget_RSNAConfigPairwiseKeysSupported(void)
{
    return MIB_11I.dot11RSNAConfigPairwiseKeysSupported;
}

INLINE void mset_RSNAConfigPairwiseKeysSupported(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigPairwiseKeysSupported = inp;
}

/* This object indicates the group cipher suite selector the entity    */
/* must use. The group cipher suite in the RSN Information Element     */
/* shall take its value from this variable. It consists of an OUI (the */
/* first 3 octets) and a cipher suite identifier (the last octet)      */
INLINE UWORD8 mget_RSNAConfigGroupCipher(void)
{
    return (MIB_11I.dot11RSNAConfigGroupCipher);
}

/* "This object selects a mechanism for rekeying the RSNA GTK. The     */
/* default is time-based, once per day. Rekeying the GTK is only       */
/* applicable to an entity acting in the Authenticator role (an AP in  */
/* an ESS).                                                            */
INLINE UWORD32 mget_RSNAConfigGroupRekeyMethod(void)
{
    return MIB_11I.dot11RSNAConfigGroupRekeyMethod;
}


INLINE void mset_RSNAConfigGroupRekeyMethod(UWORD32 inp)
{
    /* Input Sanity Check */
    if(inp <= 4)
        MIB_11I.dot11RSNAConfigGroupRekeyMethod = inp;
}

/* The time in seconds after which the RSNA GTK shall be refreshed.          */
/* The timer shall start at the moment the GTK was set using the MLMESETKEYS.*/
/* request primitive                                                         */
INLINE UWORD32 mget_RSNAConfigGroupRekeyTime(void)
{
    return MIB_11I.dot11RSNAConfigGroupRekeyTime;
}

INLINE void mset_RSNAConfigGroupRekeyTime(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigGroupRekeyTime = inp;
}

/* "A packet count (in 1000s of packets) after which the RSNA GTK   */
/* shall be refreshed. The packet counter shall start at the moment */
/* the GTK was set using the MLME-SETKEYS.request primitive and it  */
/* shall count all packets encrypted using the current GTK          */
INLINE UWORD32 mget_RSNAConfigGroupRekeyPackets(void)
{
    return MIB_11I.dot11RSNAConfigGroupRekeyPackets;
}

INLINE void mset_RSNAConfigGroupRekeyPackets(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigGroupRekeyPackets = inp;
}

/* The PSK for when RSNA in PSK mode is the selected AKM suite. In   */
/* that case, the PMK will obtain its value from this object.        */
/* This object is logically write-only. Reading this variable shall  */
/* return unsuccessful status or null or zero                        */
INLINE UWORD8 mget_RSNAConfigGroupRekeyStrict(void)
{
    return MIB_11I.dot11RSNAConfigGroupRekeyStrict;
}

INLINE void mset_RSNAConfigGroupRekeyStrict(TRUTH_VALUE_T inp)
{
    MIB_11I.dot11RSNAConfigGroupRekeyStrict = inp;
}

/* The PSK, for when RSNA in PSK mode is the selected AKM suite, is     */
/* configured by dot11RSNAConfigPSKValue.                               */
/* An alternative manner of setting the PSK uses the password-to-key    */
/* algorithm defined in H.4. This variable provides a means to enter a  */
/* pass-phrase. When this object is written, the RSNA entity shall use  */
/* the password-to-key algorithm specified in H.4 to derive a preshared */
/* and populate dot11RSNAConfigPSKValue with this key.                  */
/* This object is logically write-only. Reading this variable shall     */
/* return unsuccessful status or null or zero                           */
INLINE UWORD8* mget_RSNAConfigPSKValue(void)
{
    return MIB_11I.dot11RSNAConfigPSKValue;
}

INLINE void mset_RSNAConfigPSKValue(UWORD8 *inp)
{
    memcpy(MIB_11I.dot11RSNAConfigPSKValue, inp, 32);
}

/* The PSK, for when RSNA in PSK mode is the selected AKM suite, is     */
/* configured by dot11RSNAConfigPSKValue.                               */
/* An alternative manner of setting the PSK uses the password-to-key    */
/* algorithm defined in H.4. This variable provides a means to enter a  */
/* pass-phrase. When this object is written, the RSNA entity shall use  */
/* the password-to-key algorithm specified in H.4 to derive a preshared */
/* and populate dot11RSNAConfigPSKValue with this key.                  */
/* This object is logically write-only. Reading this variable shall     */
/* return unsuccessful status or null or zero                           */
INLINE UWORD8* mget_RSNAConfigPSKPassPhrase(void)
{
    return MIB_11I.dot11RSNAConfigPSKPassPhrase;
}

INLINE UWORD8* mget_RSNAConfigPSKPassPhraseValue(void)
{
    return &(MIB_11I.dot11RSNAConfigPSKPassPhrase[1]);
}

INLINE UWORD8 mget_RSNAConfigPSKPassPhraseLength(void)
{
    return (MIB_11I.dot11RSNAConfigPSKPassPhrase[0]);
}

INLINE void mset_RSNAConfigPSKPassPhrase(UWORD8 *inp, UWORD8 len)
{
    /* Restrict the length to 64 and copy the parameters */
    len = (len > MAX_PSK_PASS_PHRASE_LEN) ? MAX_PSK_PASS_PHRASE_LEN:len;
    MIB_11I.dot11RSNAConfigPSKPassPhrase[0] = len;
    strncpy((char *)(&MIB_11I.dot11RSNAConfigPSKPassPhrase[1]),
            (const char*)inp, len);
}

/* The number of times Message 1 in the RSNA Group Key Handshake will */
/* be retried per GTK Handshake attempt                               */
INLINE UWORD32 mget_RSNAConfigGroupUpdateCount(void)
{
    return MIB_11I.dot11RSNAConfigGroupUpdateCount;
}


INLINE void mset_RSNAConfigGroupUpdateCount(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigGroupUpdateCount = inp;
}

/* The number of times Message 1 and Message 3 in the RSNA 4-Way Handshake */
/* will be retried per 4-Way Handshake attempt                             */
INLINE UWORD32 mget_RSNAConfigPairwiseUpdateCount(void)
{
    return MIB_11I.dot11RSNAConfigPairwiseUpdateCount;
}


INLINE void mset_RSNAConfigPairwiseUpdateCount(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigPairwiseUpdateCount = inp;
}

/* This object indicates the length in bits of the group cipher key */
INLINE UWORD32 mget_RSNAConfigGroupCipherSize(void)
{
    return MIB_11I.dot11RSNAConfigGroupCipherSize;
}


INLINE void mset_RSNAConfigGroupCipherSize(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigGroupCipherSize = inp;
}

/* The maximum lifetime of a PMK in the PMK cache */
INLINE UWORD32 mget_RSNAConfigPMKLifetime(void)
{
    return MIB_11I.dot11RSNAConfigPMKLifetime;
}


INLINE void mset_RSNAConfigPMKLifetime(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigPMKLifetime = inp;
}

/* The percentage of the PMK lifetime that should expire before an  */
/* IEEE 802.1X reauthentication occurs                              */
INLINE UWORD32 mget_RSNAConfigPMKReauthThreshold(void)
{
    return MIB_11I.dot11RSNAConfigPMKReauthThreshold;
}


INLINE void mset_RSNAConfigPMKReauthThreshold(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigPMKReauthThreshold = inp;
}

/* Specifies the number of PTKSA replay counters per association */
/* 1 counter  2 counters  4 replay counters  16 replay counters  */
/*      0          1            2                    3           */
INLINE UWORD32 mget_RSNAConfigNumberOfPTKSAReplayCounters(void)
{
    return MIB_11I.dot11RSNAConfigNumberOfPTKSAReplayCounters;
}


INLINE void mset_RSNAConfigNumberOfPTKSAReplayCounters(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigNumberOfPTKSAReplayCounters = inp;
}

/* The maximum time a security association shall take to set up.*/
INLINE UWORD32 mget_RSNAConfigSATimeout(void)
{
    return MIB_11I.dot11RSNAConfigSATimeout;
}


INLINE void mset_RSNAConfigSATimeout(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigSATimeout = inp;
}

/* The selector of the last AKM suite negotiated */
INLINE UWORD8 mget_RSNAAuthenticationSuiteSelected(void)
{
    return MIB_11I.dot11RSNAAuthenticationSuiteSelected;
}


INLINE void mset_RSNAAuthenticationSuiteSelected(UWORD8 inp)
{
    MIB_11I.dot11RSNAAuthenticationSuiteSelected = inp;
}

/* The selector of the last pairwise cipher negotiated */
INLINE UWORD8 mget_RSNAPairwiseCipherSelected(void)
{
    return MIB_11I.dot11RSNAPairwiseCipherSelected;
}


INLINE void mset_RSNAPairwiseCipherSelected(UWORD8 inp)
{
    MIB_11I.dot11RSNAPairwiseCipherSelected = inp;
}

/* The selector of the last group cipher negotiated. */
INLINE UWORD8 mget_RSNAGroupCipherSelected(void)
{
    return MIB_11I.dot11RSNAGroupCipherSelected;
}


INLINE void mset_RSNAGroupCipherSelected(UWORD8 inp)
{
    MIB_11I.dot11RSNAGroupCipherSelected = inp;
}

/* The selector of the last PMKID used in the last 4-Way Handshake */
INLINE UWORD8* mget_RSNAPMKIDUsed(void)
{
    return MIB_11I.dot11RSNAPMKIDUsed;
}


INLINE void mset_RSNAPMKIDUsed(UWORD8 *inp)
{
    memcpy(MIB_11I.dot11RSNAPMKIDUsed, inp, 16);
}

/* The selector of the last AKM suite requested */
INLINE UWORD8 mget_RSNAAuthenticationSuiteRequested(void)
{
    return MIB_11I.dot11RSNAAuthenticationSuiteRequested;
}


INLINE void mset_RSNAAuthenticationSuiteRequested(UWORD8 inp)
{
    MIB_11I.dot11RSNAAuthenticationSuiteRequested = inp;
}

/* The selector of the last pairwise cipher requested */
INLINE UWORD8 mget_RSNAPairwiseCipherRequested(void)
{
    return MIB_11I.dot11RSNAPairwiseCipherRequested;
}


INLINE void mset_RSNAPairwiseCipherRequested(UWORD8 inp)
{
    MIB_11I.dot11RSNAPairwiseCipherRequested = inp;
}

/* The selector of the last group cipher requested */
INLINE UWORD8 mget_RSNAGroupCipherRequested(void)
{
    return MIB_11I.dot11RSNAGroupCipherRequested;
}


INLINE void mset_RSNAGroupCipherRequested(UWORD8 inp)
{
    MIB_11I.dot11RSNAGroupCipherRequested = inp;
}

/* The selector of the last group cipher requested */
INLINE UWORD8 mget_RSNAModeRequested(void)
{
    return MIB_11I.dot11RSNAModeRequested;
}


INLINE void mset_RSNAModeRequested(UWORD8 inp)
{
    MIB_11I.dot11RSNAModeRequested = inp;
}

/* Counts the number of times that a TKIP MIC failure occurred two     */
/* times within 60 s and TKIP countermeasures were invoked. This       */
/* attribute counts both local and remote MIC failure events reported  */
/* to this STA. It increments every time TKIP countermeasures are      */
/* invoked                                                             */
INLINE UWORD32 mget_RSNATKIPCounterMeasuresInvoked(void)
{
    return MIB_11I.dot11RSNATKIPCounterMeasuresInvoked;
}


INLINE void mset_RSNATKIPCounterMeasuresInvoked(UWORD32 inp)
{
    MIB_11I.dot11RSNATKIPCounterMeasuresInvoked = inp;
}

/* Counts the number of 4-Way Handshake failures */
INLINE UWORD32 mget_RSNA4WayHandshakeFailures(void)
{
    return MIB_11I.dot11RSNA4WayHandshakeFailures;
}


INLINE void mset_RSNA4WayHandshakeFailures(UWORD32 inp)
{
    MIB_11I.dot11RSNA4WayHandshakeFailures = inp;
}

INLINE void mincr_RSNA4WayHandshakeFailures(void)
{
    MIB_11I.dot11RSNA4WayHandshakeFailures++;
}

/* Specifies the number of GTKSA replay counters per association */
/* 1 counter  2 counters  4 replay counters  16 replay counters  */
/*      0          1            2                    3           */
INLINE UWORD32 mget_RSNAConfigNumberOfGTKSAReplayCounters(void)
{
    return MIB_11I.dot11RSNAConfigNumberOfGTKSAReplayCounters;
}


INLINE void mset_RSNAConfigNumberOfGTKSAReplayCounters(UWORD32 inp)
{
    MIB_11I.dot11RSNAConfigNumberOfGTKSAReplayCounters = inp;
}

/* The selector of a supported pairwise cipher. It consists of an OUI */
/* (the first 3 octets) and a cipher suite identifier (the last       */
/* octet)                                                             */
INLINE UWORD8 mget_RSNAConfigPairwiseCipher(UWORD8 index)
{
    return MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipher;
}


INLINE void mset_RSNAConfigPairwiseCipher(UWORD8 inp, UWORD8 index)
{
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipher = inp;
}

/* "This object enables or disables the pairwise cipher. */
INLINE UWORD8 mget_RSNAConfigPairwiseCipherEnabled(UWORD8 index)
{
    return MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipherEnabled;
}


INLINE void mset_RSNAConfigPairwiseCipherEnabled(TRUTH_VALUE_T inp, UWORD8 index)
{
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipherEnabled = inp;
}

/* This object indicates the length in bits of the pairwise cipher */
/* key. This should be 256 for TKIP and 128 for CCMP               */
INLINE UWORD32 mget_RSNAConfigPairwiseCipherSize(UWORD8 index)
{
    return MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipherSize;
}


INLINE void mset_RSNAConfigPairwiseCipherSize(UWORD32 inp, UWORD8 index)
{
    MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[index].
        dot11RSNAConfigPairwiseCipherSize = inp;
}


/* The selector of an AKM suite. It consists of an OUI (the first 3 */
/* octets) and a cipher suite identifier (the last octet).          */

INLINE UWORD8 mget_RSNAConfigAuthenticationSuite(UWORD8 index)
{
    return MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[index].
        dot11RSNAConfigAuthenticationSuite;
}


INLINE void mset_RSNAConfigAuthenticationSuite(UWORD8 inp, UWORD8 index)
{
    MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[index].
        dot11RSNAConfigAuthenticationSuite = inp;
}

/* This variable indicates whether the corresponding AKM suite is */
/* enabled/disabled                                               */
INLINE UWORD8 mget_RSNAConfigAuthenticationSuiteEnabled(UWORD8 index)
{
    return MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[index].
        dot11RSNAConfigAuthenticationSuiteEnabled;
}


INLINE void mset_RSNAConfigAuthenticationSuiteEnabled(TRUTH_VALUE_T inp, UWORD8 index)
{
    MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[index].
        dot11RSNAConfigAuthenticationSuiteEnabled = inp;
}

INLINE void mfree_RSNAStats(UWORD8 index)
{
    mem_set(&(MIB_11I.dot11RSNAStatsEntry[index]), 0,
            sizeof(dot11RSNAStatsEntry_t));
}

INLINE UWORD8* mget_RSNAStatsSTAAddress(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsSTAAddress;
}


INLINE void mset_RSNAStatsSTAAddress(UWORD8 *inp, UWORD8 index)
{
    memcpy(MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsSTAAddress,
           inp, 6);
}

/* The RSNA version with which the STA associated */
INLINE UWORD32 mget_RSNAStatsVersion(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsVersion;
}


INLINE void mset_RSNAStatsVersion(UWORD32 inp, UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsVersion = inp;
}

/* The pairwise cipher suite Selector (as defined in 7.3.29.1) used */
/* during association, in transmission order.                       */
INLINE UWORD8 mget_RSNAStatsSelectedPairwiseCipher(UWORD8 index)
{
    return MIB_11I.
              dot11RSNAStatsEntry[index].dot11RSNAStatsSelectedPairwiseCipher;
}


INLINE void mset_RSNAStatsSelectedPairwiseCipher(UWORD8 inp, UWORD8 index)
{
        MIB_11I.dot11RSNAStatsEntry[index].
                dot11RSNAStatsSelectedPairwiseCipher = inp;
}

/* Counts the number of TKIP ICV errors encountered when decrypting */
/* packets for the STA                                              */
INLINE UWORD32 mget_RSNAStatsTKIPICVErrors(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPICVErrors;
}


INLINE void mincr_RSNAStatsTKIPICVErrors(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPICVErrors++;
}

/* Counts the number of MIC failures encountered when checking the */
/* integrity of packets received from the STA at this entity       */
INLINE UWORD32 mget_RSNAStatsTKIPLocalMICFailures(UWORD8 index)
{
    return MIB_11I.
                dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPLocalMICFailures;
}


INLINE void mincr_RSNAStatsTKIPLocalMICFailures(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPLocalMICFailures++;
}

/* Counts the number of MIC failures encountered by the STA identified */
/* by dot11StatsSTAAddress and reported back to this entity            */
INLINE UWORD32 mget_RSNAStatsTKIPRemoteMICFailures(UWORD8 index)
{
    return MIB_11I.
            dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPRemoteMICFailures;
}


INLINE void mincr_RSNAStatsTKIPRemoteMICFailures(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPRemoteMICFailures++;
}

/* The number of received CCMP MPDUs discarded by the replay */
/* mechanism                                                 */
INLINE UWORD32 mget_RSNAStatsCCMPReplays(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsCCMPReplays;
}


INLINE void mincr_RSNAStatsCCMPReplays(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsCCMPReplays++;
}

/* The number of received MPDUs discarded by the CCMP decryption */
/* algorithm                                                     */
INLINE UWORD32 mget_RSNAStatsCCMPDecryptErrors(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsCCMPDecryptErrors;
}


INLINE void mincr_RSNAStatsCCMPDecryptErrors(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsCCMPDecryptErrors++;
}

/* Counts the number of TKIP replay errors detected */
INLINE UWORD32 mget_RSNAStatsTKIPReplays(UWORD8 index)
{
    return MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPReplays;
}


INLINE void mincr_RSNAStatsTKIPReplays(UWORD8 index)
{
    MIB_11I.dot11RSNAStatsEntry[index].dot11RSNAStatsTKIPReplays++;
}

/* Returns the number of Pairwise Cipher Suites Selected */
INLINE UWORD8 get_authentication_suite(void)
{
    UWORD8 i, ret;

    ret = 0;
    for(i = 0; i < AUTHENTICATION_SUITES; i++)
    {
        /* If the Entry in the MIB is enabled, increment the counter */
        if(MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[i].
            dot11RSNAConfigAuthenticationSuiteEnabled == TV_TRUE)
        {
            ret ++;
        }
    }

    return ret;
}
/* Enable AES PW-CIP policy */
INLINE void enable_rsna_aes(void)
{
    set_rsna_policy(0x04, TV_TRUE);
}

/* Disable AES PW-CIP policy */
INLINE void disable_rsna_aes(void)
{
    set_rsna_policy(0x04, TV_FALSE);
}

/* Enables TKIP PW-CIP policy */
INLINE void enable_rsna_tkip(void)
{
    set_rsna_policy(0x02, TV_TRUE);
}

/* Disable TKIP PW-CIP policy */
INLINE void disable_rsna_tkip(void)
{
    set_rsna_policy(0x02, TV_FALSE);
}

#endif /* MIB_11I_H */

#endif /* MAC_802_11I */
