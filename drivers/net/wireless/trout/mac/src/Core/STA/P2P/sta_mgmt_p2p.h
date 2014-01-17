/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : sta_mgmt_p2p.h                                       */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to P2P STA management.                       */
/*                                                                           */
/*  List of Functions : is_p2p_grp_form_in_prog                              */
/*                      set_p2p_grp_form_in_prog                             */
/*                      p2p_wait_dev_disc                                    */
/*                      p2p_wait_inv_rsp                                     */
/*                      p2p_wait_go_neg_rsp                                  */
/*                      p2p_wait_go_neg_cnf                                  */
/*                      p2p_join_complete                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

#ifndef P2P_MGMT_STA_H
#define P2P_MGMT_STA_H

// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "event_manager.h"
#include "management.h"
#include "management_sta.h"
#include "mgmt_p2p.h"
#include "frame_p2p.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define PSK_PASS_PHRASE_LEN            8
#define RSP_TIMEOUT                    100
#define P2P_OBSS_FIND_TO               25
#define P2P_WAIT_WPS_START_TO          120000 /* 2 minutes */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
typedef enum { NORMAL_SCAN           = 0,
               LISTEN_STATE          = 1,
               SEARCH_STATE          = 2
}P2P_SCAN_STATE;

typedef enum{ P2P_WAIT_GO_NEG_RSP    = 0,
              P2P_WAIT_GO_NEG_CNF    = 1,
              P2P_WAIT_DEV_DISC_RSP  = 2,
              P2P_WAIT_INV_RSP       = 3,
              P2P_WAIT_PROV_DISC_RSP = 4,
              P2P_WAIT_WPS_START     = 5
}P2P_JOIN_STATE;

typedef enum{ P2P_SCAN_MATCH_FAIL    = 0,
              P2P_JOIN_GROUP         = 1,
              P2P_INVITE_PERSIST     = 2,
              P2P_GO_NEG             = 3,
              P2P_DDR_INVITE_PERSIST = 4,
              P2P_DDR_GO_NEG         = 5,
              P2P_UNAVAILABLE        = 6
} P2P_JOIN_METHOD_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* P2P MLME Join request structure */
typedef struct
{
    p2p_dev_dscr_t    dev_dscr;
    P2P_JOIN_METHOD_T join_method;
} p2p_join_req_t;


/* Structure to save p2p user settings */
typedef struct
{
    UWORD8    auth_type;
    UWORD8    sec_mode;
    UWORD8    psk[MAX_PSK_PASS_PHRASE_LEN];
    UWORD8    ssid[MAX_SSID_LEN];
    UWORD8    bssid[6];
    UWORD8    p2p_op_chan;
} p2p_user_settings_t;


/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern BOOL_T g_p2p_grp_form_in_prog;
extern UWORD8 g_p2p_scan_state;
extern UWORD8 g_p2p_find_phase_itr_cnt;
extern p2p_join_req_t g_p2p_join_req;
extern UWORD8 g_p2p_join_state;
extern UWORD8 g_p2p_go_disc_req_rx;
extern p2p_user_settings_t g_p2p_user_settings;
extern BOOL_T g_waiting_for_go_neg_req;
extern WORD8 g_p2p_rand_ssid_pre[];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern BOOL_T p2p_get_dev(UWORD8* msa, UWORD16 index, UWORD16 rx_len);
extern void p2p_send_go_neg_rsp(UWORD8 *msa, UWORD8 status);
extern void p2p_send_go_neg_req(void);
extern void p2p_send_dev_disc_req(void);
extern void p2p_send_go_neg_cnf(UWORD8 status);
extern UWORD16 add_asoc_req_p2p_ie(UWORD8 *data, UWORD16 index);
extern void gen_p2p_ssid(void);
extern void gen_random_pass_phrase(void);
extern SWORD8 handle_scan_itr_to_p2p(mac_struct_t *mac);
extern void misc_p2p_event_sta(UWORD8 *msg);
extern void p2p_join_complete(mac_struct_t *mac, UWORD8 result);
extern void p2p_end_grp_form(mac_struct_t *mac);
extern void update_join_req_params_p2p(join_req_t *join_req);
extern void init_p2p_sta_globals(void);
extern void store_user_settings_p2p(void);
extern void restore_user_settings_p2p(void);
extern void p2p_sta_wait_scan_handle_prb_req(UWORD8 *msa, UWORD16 rx_len);
extern void p2p_send_prov_disc_req(void);
extern STATUS_T p2p_config_persist_cred(void);
extern UWORD8* p2p_get_go_dev_address(UWORD8 *grp_bssid);
extern void p2p_handle_invit_rsp_sta(UWORD8 *msa, UWORD16 rx_len,
                                     mac_struct_t *mac);
#ifdef DEBUG_MODE
extern void print_dev_dscr(UWORD8 index);
#endif /* DEBUG_MODE */


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the P2P Group Formation in progress flag */
INLINE void set_p2p_grp_form_in_prog(BOOL_T val)
{
    g_p2p_grp_form_in_prog = val;
}

INLINE UWORD8 get_p2p_scan_state(void)
{
    return g_p2p_scan_state;
}

INLINE void set_p2p_scan_state(P2P_SCAN_STATE inp)
{
    g_p2p_scan_state = inp;
}

INLINE UWORD8 get_p2p_join_state(void)
{
    return g_p2p_scan_state;
}

INLINE void set_p2p_join_state(P2P_JOIN_STATE inp)
{
    TROUT_DBG4("set p2p join state from %d to %d",g_p2p_scan_state,inp);
    g_p2p_scan_state = inp;
}


/* Functions for P2P WAIT JOIN State Machine */

/* This function sends a Device Discoverability Request and changes the P2P  */
/* WAIT JOIN sub state to P2P_WAIT_DEV_DISC_RSP                              */
INLINE void p2p_wait_dev_disc_rsp(void)
{
    /* Send a P2P Device Discoverability Request */
    p2p_send_dev_disc_req();

    /* Restart the timeout timer for getting the response */
    start_mgmt_timeout_timer(RSP_TIMEOUT);

    /* Set P2P Join state to wait for a P2P Device Discoverability Response */
    set_p2p_join_state(P2P_WAIT_DEV_DISC_RSP);
}

/* This function sends a Invitation Request to reinvoke a persistent group   */
/* and changes the P2P WAIT JOIN sub state to P2P_WAIT_INV_RSP               */
INLINE void p2p_wait_inv_rsp(void)
{

    /* Send a P2P Invitation Request */
    if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
    {
        p2p_send_inv_req(1, get_config_timeout(), 0, g_persist_list.ssid,
                         mget_p2p_dev_addr(), mget_p2p_oper_chan());
    }
    else
    {
        p2p_send_inv_req(1, 0, get_config_timeout(),
                         g_persist_list.ssid, g_persist_list.cl_list[0],
                         mget_p2p_oper_chan());
    }

    /* Restart the timeout timer for getting the response */
    start_mgmt_timeout_timer(RSP_TIMEOUT);

    /* Set the P2P Join state to wait for a P2P Invitation Response */
    set_p2p_join_state(P2P_WAIT_INV_RSP);
}

/* This function starts a 2 minute timer and changes the P2P WAIT JOIN sub-  */
/* state to P2P_WAIT_WPS_START state.                                        */
INLINE void p2p_wait_wps_start(void)
{
    /* Restart the timeout timer for getting the response */
    start_mgmt_timeout_timer(P2P_WAIT_WPS_START_TO);

    /* Set the P2P Join state to wait for WPS to be started by the user */
    set_p2p_join_state(P2P_WAIT_WPS_START);
}

/* This function sends a GO Negotiation Request and changes the P2P WAIT     */
/* JOIN sub state to P2P_WAIT_GO_NEG_RSP.                                    */
INLINE void p2p_wait_go_neg_rsp(void)
{
	/* Reset the flag to wait for a GO Negotiation Request from the target   */
	/* device whenever a GO Negotitation Request is being sent. This shall   */
	/* be set later if required on receiving a GO Negotiation Response.      */
	g_waiting_for_go_neg_req = BFALSE;

    /* Send a GO Negotiation Request */
    p2p_send_go_neg_req();

    /* Restart the timeout timer for getting the response */
    start_mgmt_timeout_timer(RSP_TIMEOUT);

    /* Set the P2P Join state to wait for a GO Negotiation Response */
    set_p2p_join_state(P2P_WAIT_GO_NEG_RSP);
}

/* This function changes the P2P WAIT JOIN sub state to P2P_WAIT_GO_NEG_CNF */
/* and starts the management timer                                          */
INLINE void p2p_wait_go_neg_cnf(void)
{
    /* Restart the timeout timer for getting the response */
     start_mgmt_timeout_timer(RSP_TIMEOUT);

    /* Set the P2P Join state to wait for a GO Negotiation Confirm */
    set_p2p_join_state(P2P_WAIT_GO_NEG_CNF);
}

/* This function sends a Provision Discovery Request and changes the P2P WAIT*/
/* JOIN sub state to P2P_WAIT_PROV_DISC_RSP                                  */
INLINE void p2p_wait_prov_disc_rsp(void)
{
    /* Send a Provision Discovery Request */
    p2p_send_prov_disc_req();

    /* Restart the timeout timer for getting the response */
    start_mgmt_timeout_timer(RSP_TIMEOUT);

    /* Set the P2P Join state to wait for a Provision Discovery Response */
    g_p2p_join_state = P2P_WAIT_PROV_DISC_RSP;
}

#endif /* P2P_MGMT_STA_H */

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
