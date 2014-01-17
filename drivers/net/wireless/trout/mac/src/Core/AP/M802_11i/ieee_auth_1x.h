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
/*  File Name         : ieee_auth_1x.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      IEEE 802.1x-REV/D9.                                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifdef MAC_802_11I

#ifndef IEEE_AUTH_1X_H
#define IEEE_AUTH_1X_H

/*****************************************************************************/
/* Include Files                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "ieee_1x.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define RAD_KEY_MAX_LEN 64

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Timers and global variables used in the definitions of the state machines */
typedef struct
{
    /*************************************************************************/
    /* Timer variable definitions of the state machines  - Section 8.2.2.1   */
    /*************************************************************************/

    /* Timer used by backend authentication state machine in order to        */
    /* determine timeout between Authenticator and EAP exchanges.            */
    ALARM_HANDLE_T *aWhile_alarm_handle;
    BOOL_T aWhileTimeout;

    /* Timer used by Authenticator state machine to define periods of time   */
    /* during which it will not attempt to acquire a supplicant.             */
    ALARM_HANDLE_T *quietWhile_alarm_handle;
    BOOL_T quietWhileTimeout;

    /* Timer used by Reauthentication timer state machine to determine when  */
    /* reauthentication of supplicant will happen.                           */
    ALARM_HANDLE_T *reAuthWhen_alarm_handle;
    BOOL_T reAuthWhenTimeout;

    /* A timer used by the Authenticator PAE state machine to determine when */
    /* an EAPOL PDU is to be transmitted. The initial value of this timer is */
    /* txPeriod.                                                             */
    ALARM_HANDLE_T *txWhen_alarm_handle;
    BOOL_T txWhenTimeout;

    /*************************************************************************/
    /* Global variables used by one / more state machines  - Section 8.2.2.2 */
    /*************************************************************************/

    /* A value fo TRUE is set by Authenticator PAE state machine to signal   */
    /* Backend authentication state machine to abort its authentication      */
    /* procedure. It is set to FALSE after procedure is aborted.             */
    BOOL_T authAbort;

    /* Authentication backend state machine sets it to TRUE if the           */
    /* authentication process fails. It is set to FALSE by Authenticator     */
    /* PAE prior to initiating authentication.                               */
    BOOL_T authFail;

    /* Indicates the current authorization state of the Authenticator PAE    */
    X_AUTH_STATUS_T authPortStatus;

    /* A value of TRUE is set by Authenticator PAE state machine indicates   */
    /* to Authentication backend state machine to start its authentication   */
    /* procedure. It is set to FALSE after the procedure is started.         */
    BOOL_T authStart;

    /* A value of TRUE is set if the Authentication backend state machine    */
    /* fails to obtain response from supplicant. It is set to FALSE by the   */
    /* operation of Authenticator PAE state machine.                         */
    BOOL_T authTimeout;

    /* Authentication backend state machine sets it to TRUE if the           */
    /* authentication process succeeds. It is set to FALSE by Authenticator  */
    /* PAE prior to initiating authentication.                               */
    BOOL_T authSuccess;

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

    /* The value is derived from AuthControlledPortControl and               */
    /* SystemAuthControl parameter for the port. If SystemAuthControl is     */
    /* enabled the value is directly the AuthControlledPortControl. Else if  */
    /* it is disabled, value is FORCE_AUTHORIZED.                            */
    X_PORT_STATE_T portControl;

    /* A value of TRUE is set (externally controlled) if MAC is operable.    */
    BOOL_T portEnabled;

    /* The current authorization state of the controlled port. This value    */
    /* directly determines the value of AuthControlledPortStatus. Its value  */
    /* is determined by the values of authPortStatus(APS), suppPortStatus    */
    /* (SPS) and Supplicant access control with Authenticator administrative */
    /* control (SAC-AAC) parameter.                                          */
    /* 1) If supplicant PAE and authenticator PAE are implemented for the    */
    /*    port AND SAC-AAC is inactive, then portStatus = authPortStatus.    */
    /* 2) If value of APS or SPS is UNAUTHORIZED, then portStatus =          */
    /*    UNAUTHORIZED.                                                      */
    /* 3) If value of APS or SPS is AUTHORIZED, then portStatus =            */
    /*    AUTHORIZED.                                                        */
    X_AUTH_STATUS_T portStatus;

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

    /* A value of TRUE is set by the Reauthentication Timer state machine on */
    /* expiry of the reAuthWhen timer. It is set FALSE by the operation of   */
    /* the Authenticator PAE state machine. NOTE: Only one pending reauth    */
    /* will be tracked.                                                      */
    BOOL_T reAuthenticate;

    /*  The value of the Identifier field carried in the EAP Request frame   */
    /* most recently received by the Supplicant, in the range 0-255.         */
    UWORD8 receivedId;
} x_auth_global_t;


/* Variables and constants used by the Authenticator state machines along    */
/* with the counters maintained.                                             */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    AUTH_STATE_T state;

    /*************************************************************************/
    /* Variables - Section 8.2.4.1.1                                         */
    /*************************************************************************/

    /* This variable is set TRUE if an EAPOL PDU carrying a Packet Type of   */
    /* EAPOL-Logoff is received. It is set FALSE by the operation of the     */
    /* Authenticator PAE state machine.                                      */
    BOOL_T eapolLogoff;

    /* This variable is set TRUE if an EAPOL PDU carrying a Packet Type of   */
    /* EAPOL-Start is received. It is set FALSE by the operation of the      */
    /* Authenticator PAE state machine.                                      */
    BOOL_T eapolStart;

    /* This variable is used in conjunction with portControl to switch       */
    /* between the Auto and non-Auto modes of operation of the Authenticator */
    /* PAE state machine.                                                    */
    X_PORT_STATE_T portMode;

    /* This variable counts the number of times the CONNECTING state is      */
    /* re-entered. If the count exceeds reAuthMax, it forces the Port to     */
    /* become Unauthorized before further attempts to authenticate can be    */
    /* made.                                                                 */
    UWORD32 reAuthCount;

    /* This flag is used to determine if the received ID matches with the    */
    /* last sent request ID                                                  */
    BOOL_T rxRespId;

    /*************************************************************************/
    /* Constants - Section 8.2.4.1.2                                         */
    /*************************************************************************/

    /* The initialization value used for the quietWhile timer. Its default   */
    /* value is 60 s; it can be set by management to any value in the range  */
    /* from 0 to 65535 s.                                                    */
    UWORD32 quietPeriod;

    /* The number of reauthentication attempts that are permitted before the */
    /* Port becomes Unauthorized. The default value of this constant is 2.   */
    UWORD32 reAuthMax;

    /*The initialization value used for the txWhen timer. Its default value  */
    /* is 30 s; it can be set by management to any value in the range from 1 */
    /* to 65 535 s.                                                          */
    UWORD32 txPeriod;

    /*************************************************************************/
    /* Counters - Section 8.2.4.2                                            */
    /*************************************************************************/

    /* Counts the number of times that the state machine transitions to the  */
    /* CONNECTING state from any other state.                                */
    UWORD32 authEntersConnecting;

    /* Counts the number of times that the state machine transitions from    */
    /* CONNECTING to DISCONNECTED as a result of receiving an EAPOL-Logoff   */
    /* message.                                                              */
    UWORD32 authEapLogoffsWhileConnecting;

    /* Counts the number of times that the state machine transitions from    */
    /* CONNECTING to AUTHENTICATING, as a result of an EAP-Response/Identity */
    /* message being received from the Supplicant.                           */
    UWORD32 authEntersAuthenticating;

    /* Counts  the  number  of  times that  the  state  machine  transitions */
    /* from  AUTHENTICATING  to AUTHENTICATED,  as  a  result  of  the       */
    /* Backend  Authentication  state  machine  indicating  successful       */
    /* authentication of the Supplicant (authSuccess = TRUE).                */
    UWORD32 authAuthSuccessesWhileAuthenticating;

    /* Counts the number of times that the state machine transitions from    */
    /* AUTHENTICATING to ABORTING, as a result of the Backend Authentication */
    /* state  machine  indicating  authentication  timeout  (authTimeout  =  */
    /* TRUE).                                                                */
    UWORD32 authAuthTimeoutsWhileAuthenticating;

    /* Counts  the  number of  times  that  the  state machine  transitions  */
    /* from AUTHENTICATING to HELD, as a result of Backend Authentication    */
    /* state machine indicating authentication failure (authFail = TRUE).    */
    UWORD32 authAuthFailWhileAuthenticating;

    /* Counts the number of times that the state machine transitions from    */
    /* AUTHENTICATING to ABORTING, as a result of an EAPOL-Start message     */
    /* being received from the Supplicant.                                   */
    UWORD32 authAuthEapStartsWhileAuthenticating;

    /* Counts the number of times that the state machine transitions from    */
    /* AUTHENTICATING to ABORTING, as a result of an EAPOL-Logoff message    */
    /* being received from the Supplicant.                                   */
    UWORD32 authAuthEapLogoffWhileAuthenticating;

    /* Counts the number of times that the state machine transitions from    */
    /* AUTHENTICATED to RESTART, as a result of a reauthentication request   */
    /* (reAuthenticate = TRUE).                                              */
    UWORD32 authAuthReauthsWhileAuthenticated;

    /* Counts the number of times that the state machine transitions from    */
    /* AUTHENTICATING to CONNECTING, as a result of an EAPOL-Start message   */
    /* being received from the Supplicant.                                   */
    UWORD32 authAuthEapStartsWhileAuthenticated;

    /* Counts  the  number  of  times  that  the  state  machine transitions */
    /* from  AUTHENTICATED  to DISCONNECTED, as a result of an EAPOL-Logoff  */
    /* message being received from the Supplicant.                           */
    UWORD32 authAuthEapLogoffWhileAuthenticated;
} auth_pae_t;

/* Variables used by the Key Receive state machine                           */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    KEY_RX_STATE_T state;

    /*************************************************************************/
    /* Variables - Section 8.2.7.1.1                                         */
    /*************************************************************************/

    /* This variable  is  set  TRUE  if  an EAPOL-Key  message  is  received */
    /* by  the Supplicant or Authenticator (see 8.1.9). It is set FALSE when */
    /* the Key Receive state machine has transmitted the key value.          */
    BOOL_T rxKey;
} key_rx_t;

/* Constants used by the Reauthentication Timer state machine                */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    REAUTH_STATE_T state;

    /*************************************************************************/
    /* Constants - Section 8.2.8.1                                           */
    /*************************************************************************/

    /* A  constant  that  defines  a  nonzero  number  of  seconds  between  */
    /* periodic reauthentication of the Supplicant. The default value is     */
    /* 3600 s.                                                               */
    UWORD32 reAuthPeriod;

    /* A  constant  that  defines whether  regular  reauthentication  will   */
    /* take  place  on  this Port. A value of TRUE enables reauthentication; */
    /* FALSE disables reauthentication.                                      */
    BOOL_T reAuthEnabled;
} reauth_t;

/* Variables and constants used by Background Authenticator state machines   */
/* along with the counters maintained.                                       */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    AUTH_BKND_STATE_T state;

    /* This variable is used to determine how many frames have been sent to  */
    /* the supplicant without receiving a response                           */
    UWORD32 reqCount;

    /* This variable is set to TRUE if an EAPOL PDU carrying a packet type of*/
    /* EAP Packet containing an EAP request is received from supplicant      */
    /* the supplicant without receiving a response                           */
    BOOL_T rxResp;

    /* This variable is set to TRUE if an EAP Accept frame is received from  */
    /* Authenticator server                                                  */
    BOOL_T aSuccess;

    /* This variable is set to TRUE if an EAP Reject frame is received from  */
    /* Authenticator server                                                  */
    BOOL_T aFail;

    /* This variable is set to TRUE if an EAP frame is received from the     */
    /* Authenticator server                                                  */
    BOOL_T aReq;

    /* This variable is the value of the Identifier field in the most recent */
    /* EAP frame received from the Authentication Server                     */
    UWORD8 idFromServer;

    /*************************************************************************/
    /* Constants - Section 8.2.9.1.2                                         */
    /*************************************************************************/

    /* The initialization value used for the aWhile timer when timing out    */
    /* the Supplicant. Its default value is 30 s; however, if the type of    */
    /* challenge involved in the current exchange demands a different value  */
    /* of timeout (for example, if the challenge requires an action on the   */
    /* part of the user), then the timeout value is adjusted accordingly. It */
    /* can be set by management to any value in the range from 1 to X s,     */
    /* where X is an implementation-dependent value.                         */
    UWORD32 suppTimeout;

    /* The initialization value used for the aWhile  timer  when  timing out */
    /* the higher layer. The timer should be longer than the longest time    */
    /* that the higher layer would take to exhaust all of its retries to the */
    /* Authentication Server with its current timeout period. 'aWhile' timer */
    /* default value is 30 s; however, the timeout value may be adjusted to  */
    /* take into account EAP/AAA settings. It can be set by management to    */
    /* any value in the range from 1 to X s, where X is an implementation    */
    /* dependent value.                                                      */
    UWORD32 serverTimeout;

    /* The maximum number of times that the state machine will retransmit an */
    /* EAP Request packet to the Supplicant before it times out the          */
    /* authentication session. Its default value is 2; it can be set by      */
    /* management to any value in the range from 1 to 10.                    */
    UWORD8 maxReq;

    /*************************************************************************/
    /* Counters                                                              */
    /*************************************************************************/

    /* Counts  the  number  of  times  that  the  state  machine  sends  a   */
    /* supplicant's first response packet to the AAA client (ie, executes    */
    /* sendRespToServer on entry to the RESPONSE state). Indicates that the  */
    /* Authenticator attempted communication with the Authentication Server  */
    /* via the AAA client.                                                   */
    UWORD32 backendResponses;

    /* Counts  the  number  of  times  that  the  state  machine  receives   */
    /* the first request from the AAA client following the first response    */
    /* from the supplicant (ie, eapReq becomes TRUE, causing exit from the   */
    /* RESPONSE state). Indicates that the Authentication Server has         */
    /* communication with the Authenticator via the AAA client.              */
    UWORD32 backendAccessChallenges;

    /* Counts the number of times that the state machine sends an EAP-Request*/
    /* packet following the first to the Supplicant (ie, executes txReq on   */
    /* entry to the REQUEST state). Indicates that the Authenticator chose   */
    /* an EAP-method.                                                        */
    UWORD32 backendOtherRequestsToSupplicant;

    /* Counts  the  number  of  times  that  the  state  machine  receives a */
    /* success indication from the AAA client (ie, eapSuccess becomes TRUE,  */
    /* causing a transition from RESPONSE to SUCCESS). Indicates that the    */
    /* Supplicant has successfully authenticated to Authentication Server.   */
    UWORD32 backendAuthSuccesses;

    /* Counts the number of times that the state machine receives a failure  */
    /* message from the AAA client (i.e., eapFail becomes TRUE, causing a    */
    /* transition from RESPONSE to FAIL). Indicates that the Supplicant has  */
    /* not authenticated to the Authentication Server.                       */
    UWORD32 backendAuthFails;
} auth_backend_t;

/* Variables used by Controlled Directions state machines                    */
typedef struct
{
    /*************************************************************************/
    /* Current State information                                             */
    /*************************************************************************/
    CONTROL_DIR_STATE_T state;

    /*************************************************************************/
    /* Variables - Section 8.2.10.1                                          */
    /*************************************************************************/

    /* The value of the AdminControlledDirections parameter (see 6.5); it    */
    /* can take the values BOTH or IN. This parameter is used, but not       */
    /* modified, by this state machine; its value may be changed only by     */
    /* management means.                                                     */
    X_CONTROL_DIR_T adminControlledDirections;

    /* The value of the OperControlledDirections parameter (see 6.5); it can */
    /* take the values Both or In. The value of this parameter is determined */
    /* by the operation of the state machine.                                */
    X_CONTROL_DIR_T operControlledDirections;

    /* The value of the operEdge variable maintained by a Bridge Port        */
    /* (Clause 17 of IEEE P802.1D-REV). If this Port is not a Bridge Port,   */
    /* then the value of this variable is TRUE.                              */
    BOOL_T operEdge;
} control_dir_t;

/* Authenticator state machines structure. This contains all the variables   */
/* whose names are similar to that present in the standard.                  */
typedef struct
{
    /*************************************************************************/
    /* Timers and global variables - Section 8.2.2.1 and 8.2.2.2             */
    /*************************************************************************/
    x_auth_global_t global;

    /*************************************************************************/
    /* Authenticator PAE state machine - Section 8.2.4.1 and 8.2.4.2         */
    /*************************************************************************/
    auth_pae_t auth_pae;

    /*************************************************************************/
    /* Key Receive state machine - Section 8.2.7.1.1                         */
    /*************************************************************************/
    key_rx_t key_rx;

    /*************************************************************************/
    /* Reauthentication Timer state machine - Section 8.2.8.1                */
    /*************************************************************************/
    reauth_t reauth;

    /*************************************************************************/
    /* Backend Authentication state machine - Section 8.2.9.1 and 8.2.9.2    */
    /*************************************************************************/
    auth_backend_t auth_bak;

    /*************************************************************************/
    /* Controlled Directions state machine - Section 8.2.10.1                */
    /*************************************************************************/
    control_dir_t control_dir;

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
    UWORD8 *eap_buffer;

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
} auth_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T (*send_msg_to_serv)(auth_t *auth_1x);
extern void   (*update_identity)(auth_t *auth_1x, UWORD8* usr_name, UWORD8 len);
extern UWORD8 g_radius_key[RAD_KEY_MAX_LEN + 1];
extern UWORD8 g_radius_serv_addr[6];
extern UWORD8 g_src_local_eth_addr[6];
extern UWORD8 g_1x_server_eth_addr[6];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_ap_1x(void);
extern void stop_ap_1x(void);
extern void authenticator_fsm(auth_t* auth_1x);
extern void authenticator_backend_fsm(auth_t* auth_1x);
extern void reauthenticator_fsm(auth_t* auth_1x);
extern void auth_onex_fsm_run(auth_t *auth_1x);
extern void free_auth_node_1x(auth_t **auth_1x);
extern BOOL_T init_auth_node_1x(auth_t **auth_1x, UWORD8* addr, UWORD16 aid,
                                auth_t *auth_1x_buff);
extern void start_fsm_ap_1x(auth_t *auth_1x);
extern void txCannedFail(auth_t* auth_1x);
extern void txCannedSuccess(auth_t* auth_1x);
extern void auth_onex_eapol(UWORD8* data_ptr, UWORD16 len, auth_t* onex_ptr);

#endif /* IEEE_AUTH_1X_H */

#endif /* MAC_802_11I */

#endif /* BSS_ACCESS_POINT_MODE */
