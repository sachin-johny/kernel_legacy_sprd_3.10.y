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
/*  File Name         : rsna_supp_km.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      RSNA supplicant Key Management State Machine.        */
/*                                                                           */
/*  List of Functions : set_grp_key                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef RSNA_SUPP_KM_H
#define RSNA_SUPP_KM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "ce_lut.h"
#include "ieee_supp_1x.h"
#include "management.h"
#include "management_11i.h"
#include "mib_11i.h"
#include "rsna_km.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define UNUSED_GTK_ID  0xFF

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Supplicant's Per-STA 4-Way Handshake state machine states */
typedef enum {S4_DISCONNECTED   = 0,
              S4_INITIALIZE     = 1,
              S4_AUTHENTICATION = 2,
              S4_STAKEYSTART    = 3
} RSNA_SUPP_4WAY_STATE_T;

/* Supplicant's STAProcessEAPOL Key states */
typedef enum {UNKNOWN = 0,
              MICOK   = 1,
              FAILED  = 2
} EAPOL_PROCESS_STATE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Global variables used by the Supplicant state machines                 */
typedef struct
{
    /* This variable is the global STA Key Counter.                          */
    UWORD8 Counter[NONCE_SIZE];

    /* Alarm handle: This is created to manage Counter Measures */
    ALARM_HANDLE_T *cntr_msr_alarm_handle;

    /* Flag to check if the counter measure is in progress */
    BOOL_T cntr_msr_in_progress;

} rsna_supp_global_t;

/* Variables used by the Suuplicant state machines          */
typedef struct
{
    RSNA_SUPP_4WAY_STATE_T   rsna_supp_4way_state;

    /*************************************************************************/
    /* Variables - Section 8.5.6.2                                           */
    /*************************************************************************/

    /* This variable is used to initialize per STA state machine.            */
    BOOL_T Init;

    /* This variable is used to log any timeout events                       */
    BOOL_T TimeoutEvt;

    /* This variable is set TRUE if a Disassociation or Deauthentication     */
    /* message is received.                                                  */
    BOOL_T DeauthenticationRequest;

    /* This variable is set TRUE if the IEEE802.11 Management Entity wants   */
    /* an association to be authenticated. This can be set when the STA      */
    /* associates or at other times.                                         */
    BOOL_T AuthenticationRequest;

    /* This variable is set TRUE if the IEEE802.1X Authentication has failed */
    /* The STA dissociates if this is set to TRUE                            */
    BOOL_T AuthenticationFailed;

    /* This variable is set TRUE when an EAPOL-Key frame is received.        */
    BOOL_T EAPOLKeyReceived;

    /* This variable is set TRUE if the STA reports that a Fatal Data        */
    /* Integrity Error has occiured.                                         */
    BOOL_T IntegrityFailed;

    /* This variable is set TRUE if the MIC on the received EAPOL-Key frame  */
    /* is verified and is correct. Any EAPOL-Key frames with an invalid MIC  */
    /* will be dropped and ignored.                                          */
    BOOL_T MICVerified;

    /* This variable is set Failed if the MIC in the received EAPOL-Key frame*/
    /* fails and MICOK if the MIC Passess                                    */
    /* The default value of the state is Unknown                             */
    EAPOL_PROCESS_STATE_T State;

    /* This variable holds the current Nonce to be used if the STA is an     */
    /* Supplicant                                                            */
    UWORD8 SNonce[NONCE_SIZE];

    /* This variable holds the current ANonce to be used if the STA is an    */
    /* Supplicant                                                            */
    UWORD8 PreANonce[NONCE_SIZE];

    /* This variable is the current Pairwise transient key.                  */
    UWORD8 PTK[PTK_SIZE];

    /* This variable is the current Group transient keys for each GTK index. */
    UWORD8 GTK[NUM_GTK][GTK_SIZE];

    /* The current index in use                                              */
    UWORD8 GN;

    /* The previous index in use                                             */
    UWORD8 last_key_num;

    /* PMK is the buffer holding the current Pairwise Master key.            */
    UWORD8 PMK[PMK_SIZE];

    /* PMK ID is used to store the PMKID of the PMKSA                        */
    UWORD8 pmkid[PMKID_LEN];

    /* 802.1x Variables                                                      */
    supp_t supp_1x_buff; /* Buffer for the 802.1X handle */
    supp_t *sta_1x;      /* 802.1X handle                */

    /* This variable is used in IBSS mode and is used to decide when all the */
    /* keys have been delivered and an IBSS link is secure.                  */
    UWORD32 keycount;

    /*************************************************************************/
    /* Miscellaneous information                                             */
    /*************************************************************************/

    /* AckReq: This flag is used to determine if the Ack field is set in the */
    /* received EAPOL Key Frame                                              */
    BOOL_T AckReq;

    /* MAC address of the station corresponding to the RSNA Association.     */
    UWORD8 addr[6];

    /* The EAPOL Key frame received from the supplicant for key handshakes.  */
    UWORD8 *eapol_key_frame;

    /* The length of the EAPOL Key frame received from the supplicant        */
    UWORD16 eapol_frame_len;

    /* Pairwise Cipher Policy: This is used to identify the Pairwise Cipher  */
    /* scheme used by the remote STA                                         */
    UWORD8 pcip_policy;

    /* Group Cipher Policy: This is used to identify the Pairwise Cipher     */
    /* scheme used by the remote STA                                         */
    UWORD8 grp_policy;

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

    /* Grp Key Length: This is used to identify the length of the GTK        */
    /* 128 for CCMP and 256 for TKIP                                         */
    UWORD16 gkey_length;

    /* Key Index: This is used to index the TK in the MAC H/W                */
    UWORD16 key_index;

    /* Alarm handle: This is created to manage Authentication timeout        */
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

    /* Packet Value for the receive path; used for Replay detection          */
    UWORD8 bcst_rx_pn_val[6];

    /* PN Value for the last fragmnt of the last TKIP-MIC passed bcast frame */
    UWORD8 bcst_valid_pn_val[6];

    /* Packet Value for the EAPOL packets; used for Replay detection         */
    UWORD8 ReplayCount[REPLAY_CNT_SIZE];

    UWORD8 tbtt_count;
    BOOL_T fsm_start_pend;

    struct work_struct work;        //add by Hugh.
}rsna_supp_persta_t;

typedef struct
{
    rsna_supp_global_t global;
    rsna_supp_persta_t *rsna_ptr[NUM_SUPP_STA];
} rsna_supp_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    rsna_supp_4way_fsm  (rsna_supp_persta_t *rsna_ptr);
extern void    initialize_supp_rsna(void);
extern void    stop_supp_rsna      (void);
extern void    free_supp_rsna_km   (rsna_supp_persta_t **rsna_ptr);
extern void    supp_rsn_fsm_run    (rsna_supp_persta_t *rsna_ptr);
extern void    stop_supp_rsna_fsm  (rsna_supp_persta_t *rsna_ptr);
extern void    start_supp_rsna_fsm (rsna_supp_persta_t *rsna_ptr);
extern void    supp_rsn_eapol      (UWORD8 *rx_ptr, UWORD16 rx_len,
                                    rsna_supp_persta_t *rsna_ptr);

extern BOOL_T check_eapol_pkt_supp(rsna_supp_persta_t *rsna_ptr);
extern void   update_join_req_params_11i(void *ptr);
extern BOOL_T check_rsn_capabilities_sta(void *bss_dscr);

//chenq add for check link ap info change 2013-06-08
extern BOOL_T check_rsn_capabilities_sta_change(void *bss_dscr);
//chenq add end

extern BOOL_T stop_11i_supp_timer(rsna_supp_persta_t *rsna_ptr);
extern BOOL_T start_11i_supp_timer(rsna_supp_persta_t *rsna_ptr, UWORD32 time);
extern BOOL_T initialize_supp_rsna_hdl(rsna_supp_persta_t **rsna_per_sta_ptr,
                          UWORD8 pcip_policy, UWORD8 grp_policy,
                          UWORD8 auth_policy, UWORD8 key_index, UWORD8* da,
                          rsna_supp_persta_t *rsna_per_sta_ptr_buff);

extern void update_scan_response_11i(UWORD8 *msa, UWORD16 len, UWORD16 offset,
                                     UWORD8 dscr_set_index);
extern void handle_tbtt_11i_event(void);
extern void set_grp_key(rsna_supp_persta_t *rsna_ptr);
extern void rsna_supp_disconnected(rsna_supp_persta_t *rsna_ptr);

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern rsna_supp_t g_rsna_supp;

#endif /* RSNA_SUPP_KM_H */

#endif /* MAC_802_11I */


