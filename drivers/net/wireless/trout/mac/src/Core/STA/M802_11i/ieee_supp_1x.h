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
/*  File Name         : ieee_supp_1x.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      IEEE 802.1x-REV/D9 scaled down the STA mode          */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

#ifndef IEEE_SUPP_1X_H
#define IEEE_SUPP_1X_H

/*****************************************************************************/
/* Include Files                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "ieee_1x.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_SUPP_USERNAME_LEN   20
#define MAX_SUPP_PASSWORD_LEN   63

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Supplicant PAE states */
typedef enum {SS_INITIALIZE     =0,
              SS_LOGOFF         =1,
              SS_DISCONNECTED   =2,
              SS_HELD           =3,
              SSS_CONNECTING    =4,
              SS_AUTHENTICATED  =5,
              SS_ACQUIRED       =6,
              SS_AUTHENTICATING =7
} SUPP_STATE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

#ifdef IBSS_11I
/* Timers and global variables used in the definitions of the state machines */
typedef struct
{
    /*************************************************************************/
    /* Global variables used by one / more state machines  - Section 8.2.2.2 */
    /*************************************************************************/

    /* A value of TRUE set by an external entity when there is a new key     */
    /* available that can be used by the key machines to start a new key     */
    /* exchange. FALSE implies, key transmit state machines have transmitted */
    /* the key value.                                                        */
    BOOL_T keyAvailable;

    /* A value of TRUE is set by the key machine when it is in a state that  */
    /* portValid can be tested.                                              */
    BOOL_T keyDone;

    /* A value of TRUE is set by EAPOL machine when transmit key machine     */
    /* should run. FALSE is set by a PAE to indicate PAE state machine is    */
    /* reset and key machine should abort.                                   */
    BOOL_T keyRun;

    /* The value is derived from AuthControlledPortControl and               */
    /* SystemAuthControl parameter for the port. If SystemAuthControl is     */
    /* enabled the value is directly the AuthControlledPortControl. Else if  */
    /* it is disabled, value is FORCE_AUTHORIZED.                            */
    X_PORT_STATE_T portControl;

    /* A value of TRUE is set (externally controlled) if MAC is operable.    */
    BOOL_T portEnabled;


    /* Value is TRUE if:                                                     */
    /* 1) Port technology provides required level of security (LAN)          */
    /* 2) Keys have been exchanged between 802.11 STA and AP to allow the    */
    /*    traffic on that association to be encrypted.                       */
    /* It is used in conjunction with keyDone.                               */
    /* 1) portValid AND keyDone are TRUE: port is in Valid state.            */
    /* 2) portValid=TRUE AND keyDone=FALSE. port is in unknown state         */
    /*                                      as key m/c haven't run.          */
    /* 3) portValid=FALSE AND keyDone=TRUE. port is not in valid state       */
    /*                                      as key protocol failed.          */
    /* 4) portValid AND keyDone are FALSE: port is in unknown state          */
    /*                                      as key m/c haven't run.          */
    BOOL_T portValid;
} x_auth_global_t;

/* Authenticator state machines structure. This contains all the variables   */
/* whose names are similar to that present in the standard.                  */
typedef struct
{
    /*************************************************************************/
    /* Timers and global variables - Section 8.2.2.1 and 8.2.2.2             */
    /*************************************************************************/
    x_auth_global_t global;

    /* The RX_KEY Received from the 1x Authentication                        */
    UWORD8 rx_key[AAA_KEY_MAX_LEN];

} auth_t;
#endif /* IBSS_11I */

/* Timers and global variables used in the definitions of the state machines */
typedef struct
{
    /*************************************************************************/
    /* Timer variable definitions of the state machines  - Section 8.2.2.1   */
    /*************************************************************************/

    /* Timer used by Supplicant PAE to determine how long to wait for a      */
    /* request from Authenticator before timeout.                            */
    ALARM_HANDLE_T *authWhile_alarm_handle;
    BOOL_T authWhileTimeout;

    /* Timer used by Supplicant PAE to define periods of time during which   */
    /* it will not attempt to acquire an Authenticator.                      */
    ALARM_HANDLE_T *heldWhile_alarm_handle;
    BOOL_T heldWhileTimeout;

    /* A timer used by the Supplicant PAE state machine to determine when an */
    /* EAPOL-StartPDU is to be transmitted. The initial value of this timer  */
    /* is startPeriod.                                                       */
    ALARM_HANDLE_T *startWhen_alarm_handle;
    BOOL_T startWhenTimeout;

    /*************************************************************************/
    /* Global variables used by one / more state machines  - Section 8.2.2.2 */
    /*************************************************************************/

    /* The EAP packet encapsulated within the EAPOL packet as sent by the    */
    /* authenticator consists of a request ID. This is used to match the     */
    /* responses with the requests                                           */
    UWORD8 currentId;

    /* This variable is externally controlled. When asserted, it forces all  */
    /* EAPOL state machines to their initial state. The EAPOL state machines */
    /* are held in their initial state until initialize is deasserted.       */
    BOOL_T initialize;

    /* A value of TRUE set by an external entity when there is a new key     */
    /* available that can be used by the key machines to start a new key     */
    /* exchange. FALSE implies, key transmit state machines have transmitted */
    /* the key value.                                                        */
    BOOL_T keyAvailable;

    /* A value of TRUE is set by the key machine when it is in a state that  */
    /* portValid can be tested.                                              */
    BOOL_T keyDone;

    /* A value of TRUE is set by EAPOL machine when transmit key machine     */
    /* should run. FALSE is set by a PAE to indicate PAE state machine is    */
    /* reset and key machine should abort.                                   */
    BOOL_T keyRun;

    /* A value of TRUE allows key information to be transmitted once the     */
    /* keyAvailable and keyRun are set to TRUE.                              */
    BOOL_T keyTxEnabled;

    /* A value of TRUE is set (externally controlled) if MAC is operable.    */
    BOOL_T portEnabled;

    /* Value is TRUE if:                                                     */
    /* 1) Port technology provides required level of security (LAN)          */
    /* 2) Keys have been exchanged between 802.11 STA and AP to allow the    */
    /*    traffic on that association to be encrypted.                       */
    /* It is used in conjunction with keyDone.                               */
    /* 1) portValid AND keyDone are TRUE: port is in Valid state.            */
    /* 2) portValid=TRUE AND keyDone=FALSE. port is in unknown state         */
    /*                                      as key m/c haven't run.          */
    /* 3) portValid=FALSE AND keyDone=TRUE. port is not in valid state       */
    /*                                      as key protocol failed.          */
    /* 4) portValid AND keyDone are FALSE: port is in unknown state          */
    /*                                      as key m/c haven't run.          */
    BOOL_T portValid;

    /*  The value of the Identifier field carried in the EAP Request frame   */
    /* most recently received by the Supplicant, in the range 0-255.         */
    UWORD8 receivedId;

    /* suppStatus. This variable?s value is controlled by the Supplicant PAE */
    /* state machine; it indicates Authorized or Unauthorized in accordance  */
    /* with the outcome of the most recent authentication exchange that has  */
    /* taken place with the Authenticator. Its value is initialized to the   */
    /* Unauthorized state. This variable is used to control the operation of */
    /* the Supplicant Key Transmit state machine.                            */

    /* Current authorization state of Supplicant PAE state machine. It is    */
    /* set to AUTHORIZED if Supplicant state machine is not implemented.     */
    X_AUTH_STATUS_T suppPortStatus;
} x_supp_global_t;

/* Variables and constants used by the supplicant state machines             */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    SUPP_STATE_T state;

    /*************************************************************************/
    /* Variables                                                             */
    /*************************************************************************/

    /*  This variable is controlled externally to the state machine          */
    /*  Its value is set FALSE if the Supplicant System considers that its   */
    /*  user is logged on; its value is set TRUE if the Supplicant System    */
    /*  considers that its user is logged off.                               */
    BOOL_T userLogoff;

    /*  Indicates whether an EAPOL Logoff message has been sent from within  */
    /*  the LOGOFF state, thereby preventing repeated re-entry to the state  */
    /*  and consequent multiple transmission of logoff messages.             */
    BOOL_T logoffSent;

    /*  This variable is set TRUE if an EAPOL PDU carrying a Packet Type of  */
    /*  EAP-Packet,containing an EAP Request/Identity packet, is received    */
    /*  from the Authenticator.                                              */
    BOOL_T reqId;

    /*  This variable is set TRUE if an EAPOL PDU carrying a Packet Type of  */
    /*  EAP-Packet,containing an EAP Request packet other than a             */
    /*  Request/Identity packet, is received from the Authenticator.         */
    BOOL_T reqAuth;

    /*  This variable is set TRUE if an EAPOL PDU carrying a Packet Type of  */
    /*  EAP-Packet, containing an EAP Success packet, is received from the   */
    /*  Authenticator.                                                       */
    BOOL_T eapSuccess;

    /*  This variable is set TRUE if an EAPOL PDU carrying a Packet Type of  */
    /*  EAP-Packet,containing an EAP Failure packet, is received from the    */
    /*  Authenticator.                                                       */
    BOOL_T eapFail;

    /*  This variable is used to count the number of EAPOL-Start messages    */
    /*  that have been sent without receiving a response from the            */
    /*  Authenticator                                                        */
    UWORD32 startCount;

    /*  receivedId is copied into previousId after a response has been sent  */
    /*  to the most recently received EAP Request frame. Comparing previousId*/
    /*  with receivedId allows the Supplicant to detect repeated requests    */
    /*  and to resend the same response as for the initial request.          */
    UWORD16 previousId;

    /*************************************************************************/
    /* Constants                                                             */
    /*************************************************************************/

    /*  The initialization value used for the authWhile timer.               */
    /*  Its default value is 30 s.                                           */
    UWORD32 authPeriod;

    /*  The initialization value used for the heldWhile timer.               */
    /*  Its default value is 60 s.                                           */
    UWORD32 heldPeriod;

    /*  The initialization value used for the startWhen timer.               */
    /*  Its default value is 30 s.                                           */
    UWORD32 startPeriod;

    /*  The maximum number of successive EAPOL-Start messages that will be   */
    /*  sent before the Supplicant assumes that there is no Authenticator    */
    /*  present. Its default value is 3.                                     */
    UWORD16 maxStart;

} supp_pae_t;

/* Variables used by the Key Receive state machine                           */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    KEY_RX_STATE_T state;

    /*************************************************************************/
    /* Variables                                                             */
    /*************************************************************************/

    /* This variable  is  set  TRUE  if  an EAPOL-Key  message  is  received */
    /* by  the Supplicant or Authenticator. It is set FALSE when */
    /* the Key Receive state machine has transmitted the key value.          */
    BOOL_T rxKey;
} key_rx_t;


/* Supplicant state machines structure. This contains all the variables      */
/* whose names are similar to that present in the standard.                  */
typedef struct
{
    /*************************************************************************/
    /* Timers and global variables                                           */
    /*************************************************************************/
    x_supp_global_t global;

    /*************************************************************************/
    /* Supplicant PAE state machine                                          */
    /*************************************************************************/
    supp_pae_t supp_pae;

    /*************************************************************************/
    /* Key Receive state machine                                             */
    /*************************************************************************/
    key_rx_t key_rx;

    /*************************************************************************/
    /* Miscellaneous information                                             */
    /*************************************************************************/

    /* MAC address of the station corresponding to the AID. This is required */
    /* to access the AP's association table.                                 */
    UWORD8 addr[6];

    /* Pointer to the RSNA Handle */
    UWORD8 *rsna_ptr;

    /* Association ID of the Supplicant Station. This is required for NAS    */
    /* port identification                                                   */
    UWORD8 aid;

    /* The EAP packet encapsulated within the EAPOL packet as sent by the    */
    /* higher layers. This buffer does not contain the EAPOL header but has  */
    /* the buffer allocated to the required fields.                          */
    UWORD8 *eap;

    /* The EAP packet encapsulated within the EAPOL packet as sent by the    */
    /* higher layers. This buffer does not contain the EAPOL header but has  */
    /* the buffer allocated to the required fields.                          */
    UWORD16 eap_frame_len;

    /* The EAP packet encapsulated within the EAPOL packet as sent by the    */
    /* supplicant. This is sent to the upper layers by authentication        */
    /* backend state machine.                                                */
    UWORD8 *eapolEap;

    /* The EAP packet encapsulated within the EAPOL packet as sent by the    */
    /* supplicant. This is sent to the upper layers by authentication        */
    /* backend state machine.                                                */
    UWORD16 eapolEap_frame_len;

    /* The RX_KEY Received from the 1x Authentication                        */
    UWORD8 rx_key[AAA_KEY_MAX_LEN];
    
    struct work_struct work;     //add by Hugh for timer.
} supp_t;


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_supp_username[MAX_SUPP_USERNAME_LEN + 1];
extern UWORD8 g_supp_password[MAX_SUPP_PASSWORD_LEN + 1];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_sta_1x(void);
extern void stop_sta_1x(void);
extern void supplicant_fsm(supp_t* supp_1x);
extern void supp_onex_fsm_run(supp_t *supp_1x);
extern void free_supp_node_1x(supp_t **supp_1x);
extern BOOL_T init_supp_node_1x(supp_t **supp_1x, UWORD8* addr, UWORD16 aid,
                                supp_t *supp_1x_buff);
extern void start_fsm_sta_1x(supp_t *supp_1x);
//extern void txCannedFail(auth_t* auth_1x);
//extern void txCannedSuccess(auth_t* auth_1x);
extern void supp_onex_eapol(UWORD8* data_ptr, UWORD16 len, supp_t* onex_ptr);

#endif /* IEEE_SUPP_1X_H */

#endif /* MAC_802_11I */

#endif /* IBSS_BSS_STATION_MODE */
