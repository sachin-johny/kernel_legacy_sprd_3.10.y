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
/*  File Name         : rsna_auth_km.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      RSNA Authenticator Key Management State Machine.     */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I

#ifndef RSNA_AUTH_KM_H
#define RSNA_AUTH_KM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"

#ifdef AUTH_11I

#include "ieee_1x.h"
#include "management.h"
#include "management_11i.h"
#include "rsna_km.h"
#include "ieee_auth_1x.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Authenticator Per-STA 4-Way Handshake state machine states */
typedef enum {A4_AUTHENTICATION      = 0,
              A4_AUTHENTICATION2     = 1,
              A4_DISCONNECT          = 2,
              A4_DISCONNECTED        = 3,
              A4_INITIALIZE          = 4,
              A4_INITPMK             = 5,
              A4_INITPSK             = 6,
              A4_PTKCALCNEGOTIATING  = 7,
              A4_PTKCALCNEGOTIATING2 = 8,
              A4_PTKINITNEGOTIATING  = 9,
              A4_PTKINITDONE         = 10,
              A4_PTKSTART            = 11
} RSNA_AUTH_4WAY_STATE_T;

/* Authenticator Per-STA Group Key Handshake state machine states */
typedef enum {AGK_IDLE             = 0x0,
AGK_KEYERROR         = 0x1,
AGK_REKEYESTABLISHED = 0x2,
AGK_REKEYNEGOTIATING = 0x3,
AGK_INVALID_STATE    = 0xF
} RSNA_AUTH_GRPKEY_STATE_T;

/* Authenticator Global Group Key Handshake state machine states */
typedef enum {AGLK_GTK_INIT    = 0,
              AGLK_SETKEYS     = 1,
              AGLK_SETKEYSDONE = 2
} RSNA_AUTH_GLOBAL_GRPKEY_STATE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Global variables used by the Authenticator state machines                 */
typedef struct
{
    RSNA_AUTH_GLOBAL_GRPKEY_STATE_T rsna_auth_glk_state;

    /*************************************************************************/
    /* Variables - Section 8.5.7.2                                           */
    /*************************************************************************/

    /* This variable is used to initialize the Group Key state machine. This */
    /* is a group variable.                                                  */
    BOOL_T GInit;

    /* This is TRUE if the Authenticator is on an AP or it is the designated */
    /* Authenticator for an IBSS.                                            */
    BOOL_T GTKAuthenticator;

    /* Count of number of STAs left to have their GTK updated. This is a     */
    /* global variable.                                                      */
    UWORD32 GKeyDoneStations;

    /* This variable is set TRUE when a GTK Handshake is required. This is a */
    /* global variable.                                                      */
    BOOL_T GTKReKey;

    /* This variable is set TRUE if last GTK Handshake has timed out         */
    BOOL_T GTKReKeyTimeOut;

    /* This variable counts the number of Supplicants so it is known how     */
    /* many Supplicants need to be sent the GTK. This is a global variable.  */
    UWORD32 GNoStations;

    /* This variable is the global STA Key Counter.                          */
    UWORD8 Counter[NONCE_SIZE];

    /* This variable is the global STA Key Counter. (used for GNonce)        */
    UWORD8 GCounter[NONCE_SIZE];

    /* This variable is the Nonce used for GTK Generation                    */
    UWORD8 GNonce[NONCE_SIZE];

    /* These are the current key indices for GTKs.                           */
    UWORD8 GN;
    UWORD8 GM;

    /* This variable is the current Group transient keys for each GTK index. */
    UWORD8 GTK[NUM_GTK][GTK_SIZE];

    /* Alarm handle: This is created to manage Rekeying */
    ALARM_HANDLE_T *alarm_handle;

    /* Alarm handle: This is created to manage Counter Measures */
    ALARM_HANDLE_T *cntr_msr_alarm_handle;

    /* Flag to check if the counter measure is in progress */
    BOOL_T cntr_msr_in_progress;

} rsna_auth_global_t;

/* Variables used by the Authenticator state machines          */
typedef struct
{
    RSNA_AUTH_GRPKEY_STATE_T rsna_auth_grpkey_state;
    RSNA_AUTH_4WAY_STATE_T   rsna_auth_4way_state;

    /*************************************************************************/
    /* Variables - Section 8.5.7.2                                           */
    /*************************************************************************/

    /* This variable is set to TRUE when the STA should initiate a           */
    /* deauthentication.                                                     */
    BOOL_T Disconnect;

    /* This variable is set TRUE if the IEEE802.11 Management Entity wants   */
    /* an association to be authenticated. This can be set when the STA      */
    /* associates or at other times.                                         */
    BOOL_T AuthenticationRequest;

    /* This variable is set TRUE if the IEEE 802.1X Authenticator received   */
    /* an eap-Start or 802.1X::reAuthenticate is set.                        */
    BOOL_T ReAuthenticationRequest;

    /* This variable is set TRUE if a Disassociation or Deauthentication     */
    /* message is received.                                                  */
    BOOL_T DeauthenticationRequest;

    /* This variable is set TRUE when an EAPOL-Key frame is received.        */
    BOOL_T EAPOLKeyReceived;

    /* This variable is used to initialize per STA state machine.            */
    BOOL_T InitA4;
    BOOL_T InitGK;

    /* This variable is set TRUE if the EAPOL-Key frame sent out fails to    */
    /* obtain a response from the Supplicant. The variable may be set by     */
    /* management action, or by the operation of a timeout while in the      */
    /* PTKSTART and REKEYNEGOTIATING states.                                 */
    BOOL_T TimeoutEvt;

    /* This variable maintains the count of EAPOL-Key receive timeouts. It   */
    /* is incremented each time a timeout occurs on EAPOLKeyReceived event   */
    /* and is initialized to 0.                                              */
    UWORD32 TimeoutCtr;

    /* This variable is set TRUE if the MIC on the received EAPOL-Key frame  */
    /* is verified and is correct. Any EAPOL-Key frames with an invalid MIC  */
    /* will be dropped and ignored.                                          */
    BOOL_T MICVerified;

    /* This variable holds the current Nonce to be used if the STA is an     */
    /* Authenticator.                                                        */
    UWORD8 ANonce[NONCE_SIZE];

    /* This variable is the current Pairwise transient key.                  */
    UWORD8 PTK[PTK_SIZE];

    /* PMK is the buffer holding the current Pairwise Master key.            */
    UWORD8 PMK[PMK_SIZE];

    /* PMK ID is used to store the PMKID of the PMKSA                        */
    UWORD8 pmkid[PMKID_LEN];

    /* This corresponding 802.1X variable.                                   */
    auth_t auth_1x_buff;
    auth_t *sta_1x;

    /* This variable is the global STA Key Counter.                          */
    UWORD8 ReplayCount[REPLAY_CNT_SIZE]; /* Not specified */

    /* This variable is set to TRUE when a new GTK is available to be sent   */
    /* to Supplicants.                                                       */
    BOOL_T GUpdateStationKeys;

    /* This variable maintains the count of EAPOL-Key receive timeouts for   */
    /* the Group Key Handshake.                                              */
    UWORD32 GTimeoutCtr;

    /* This variableis set to TRUE if last Group key update was timed out    */
    BOOL_T GTimeout;

    /* This variable is set to TRUE when the Group key update state machine  */
    /* is required                                                           */
    BOOL_T GInitAKeys;

    /* This variable is set to TRUE when the Authenticator is ready to send a*/
    /* Group key to its Supplicant after initialization.                     */
    BOOL_T PInitAKeys;

    /* This variable is used in IBSS mode and is used to decide when all the */
    /* keys have been delivered and an IBSS link is secure.                  */
    UWORD32 keycount;

    /*************************************************************************/
    /* Miscellaneous information                                             */
    /*************************************************************************/

    /* MAC address of the station corresponding to the AID. This is required */
    /* to access the AP's association table.                                 */
    UWORD8 addr[6];

    /* The EAPOL Key frame received from the supplicant for key handshakes.  */
    UWORD8 *eapol_key_frame;

    /* The length of the EAPOL Key frame received from the supplicant        */
    UWORD16 eapol_frame_len;

    /* AID - This specifies the AID of the STA for which the handle is       */
    /* intended                                                              */
    UWORD16 aid;

    /* Listen Interval - This specifies the interval used for the 11i message*/
    /* timeouts                                                              */
    UWORD16 listen_interval;

    /* Pairwise Cipher Policy: This is used to identify the Pairwise Cipher  */
    /* scheme used by the remote STA                                         */
    UWORD8 pcip_policy;

    /* Key version is the field which determines type of the encryption used */
    /* this is set based on the encryption type in WPA and is independently  */
    /* used in RSNA                                                          */
    UWORD8 key_version;

    /* Auth Policy: This is used to identify the Authentication suite policy */
    /* used by the remote STA                                                */
    UWORD8 auth_policy;

    /* Key Length: This is used to identify the length of the PTK            */
    /* 128 for CCMP and 256 for TKIP                                         */
    UWORD16 key_length;

    /* Key Index: This is used to index the TK in the MAC H/W                */
    UWORD16 key_index;

    /* Alarm handle: This is created per station to manage timers per station*/
    ALARM_HANDLE_T *alarm_handle;

    /* send_deauth: This flag is used to determine is the STA should be      */
    /* by sending the Deauthentication Frames                                */
    BOOL_T  send_deauth;
    UWORD16 deauth_reason;

    /* The mode used by the remote STA: RSNA WPA2 or WPA                     */
    UWORD8  mode_802_11i;

#ifdef MAC_WMM
    /* Packet Value for the receive path; used for Replay detection          */
    UWORD8 rx_pn_val[8][6];

    /* PN Value for the last fragment of the last TKIP-MIC passed frame      */
    UWORD8 valid_pn_val[8][6];
#else /* MAC_WMM */
    /* Packet Value for the receive path; used for Replay detection          */
    UWORD8 rx_pn_val[6];

    /* PN Value for the last fragment of the last TKIP-MIC passed frame      */
    UWORD8 valid_pn_val[6];
#endif /* MAC_WMM */
} rsna_auth_persta_t;

typedef struct
{
    rsna_auth_global_t global;
    rsna_auth_persta_t *rsna_ptr[NUM_SUPP_STA];
} rsna_auth_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    rsna_auth_4way_fsm  (rsna_auth_persta_t *rsna_per_sta_ptr);
extern void    rsna_auth_grpkey_fsm(rsna_auth_persta_t *rsna_per_sta_ptr);
extern void    rsna_auth_glk_fsm   (void);

extern void    initialize_auth_rsna(void);
extern BOOL_T  initialize_auth_rsna_hdl(rsna_auth_persta_t **rsna_per_sta_ptr,
                        UWORD16 aid, UWORD16 listen_interval,
                        UWORD8 pcip_policy, UWORD8 auth_policy, UWORD8* sa,
                        UWORD8 *msa, UWORD16 rx_len,
                        rsna_auth_persta_t *rsna_per_sta_ptr_buff);
extern void    stop_auth_rsna     (void);
extern void    stop_auth_rsna_fsm (rsna_auth_persta_t *rsna_per_sta_ptr);
extern void    free_auth_rsna_km  (rsna_auth_persta_t **rsna_per_sta_ptr);
extern void    auth_rsn_fsm_run   (rsna_auth_persta_t *rsna_ptr);
extern void    start_auth_rsna_fsm(rsna_auth_persta_t *rsna_per_sta_ptr);

extern void    auth_rsn_eapol  (UWORD8 *rx_ptr, UWORD16 rx_len,
                    rsna_auth_persta_t* rsna_ptr);

extern BOOL_T  check_eapol_pkt_auth(rsna_auth_persta_t *rsna_ptr);
extern BOOL_T  stop_11i_auth_timer (rsna_auth_persta_t *rsna_ptr);
extern BOOL_T  start_11i_auth_timer(rsna_auth_persta_t *rsna_ptr, UWORD32 time);
extern void    rsna_auth_disconnected(rsna_auth_persta_t *rsna_ptr);
extern void rsna_auth_grpkey_keyerror(rsna_auth_persta_t *rsna_ptr);
extern void handle_rsna_timeout(rsna_auth_persta_t *rsna_ptr);
/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern rsna_auth_t g_rsna_auth;
extern BOOL_T  g_use_pairwisekey;
extern UWORD8  g_gmk[GMK_SIZE];


INLINE BOOL_T enough_attempts_4way(rsna_auth_persta_t *rsna_ptr)
{
    if(BTRUE == rsna_ptr->TimeoutEvt)
    {
        if(rsna_ptr->TimeoutCtr > mget_RSNAConfigPairwiseUpdateCount())
        {
            return BTRUE;
        }
    }

    return BFALSE;
}


INLINE BOOL_T enough_attempts_grpkey(rsna_auth_persta_t *rsna_ptr)
{
    if(BTRUE == rsna_ptr->TimeoutEvt)
    {
        if(rsna_ptr->GTimeoutCtr > mget_RSNAConfigGroupUpdateCount())
        {
            return BTRUE;
        }
    }

    return BFALSE;
}
#endif /* RSNA_AUTH_KM_H */
#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
