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
/*  File Name         : ap_mgmt_p2p.h                                        */
/*                                                                           */
/*  Description       : This file contains the functions required for        */
/*                      management of AP                                     */
/*                                                                           */
/*  List of Functions : add_p2p_pending_dd_rsp_elem                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_P2P

#ifndef P2P_MGMT_AP_H
#define P2P_MGMT_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "frame.h"
#include "p2p_ps.h"
#include "mgmt_p2p.h"


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define P2P_PRIM_DEV_TYPE_LEN       8


/* Invitation related constants */
#define  P2P_INV_OP_CHAN_TO         50
#define  P2P_INV_NON_OP_CHAN_TO     5
#define  P2P_INV_SOCIAL_CHAN_TO     8
#define  P2P_WAIT_INV_RSP_TO        10

#define MAX_NUM_P2P_INVI_TRY        30
#define WAIT_HIGH_PRI_Q_EMPTY_CNT   20

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* This is a structure to store the descriptor of the associated clients in a*/
/* P2P group                                                                 */
typedef struct
{
    UWORD8  dev_cap_bitmap;
    UWORD8  p2p_dev_addr[MAC_ADDRESS_LEN];
    UWORD16 config_method;
    UWORD8  prim_dev_typ[P2P_PRIM_DEV_TYPE_LEN];
    UWORD8  num_sec_dev;
#ifdef P2P_SEC_DEV_SUPPORT
    UWORD8  sec_dev_type_list[P2P_MAX_SEC_DEV * P2P_PRIM_DEV_TYPE_LEN];
#endif /* P2P_SEC_DEV_SUPPORT */
    UWORD8  dev_name[MAX_DEVICE_NAME_LEN + 1];
} p2p_client_info_dscr_t;

/*****************************************************************************/
/* Extern Global Variables                                                   */
/*****************************************************************************/
extern BOOL_T  g_target_p2p_wps_done;
extern UWORD8  g_p2p_switch_chan;
extern UWORD8  g_invi_scan_chan_idx ;
extern UWORD8  g_num_invi_try;

extern UWORD8  g_p2p_switch_chan;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void handle_p2p_go_disc_req_com_ap(UWORD8 *tx_dscr);
extern void update_p2p_asoc_client_info(void *asoc_entry,
                                        UWORD8 *attr_start_ptr,
                                        UWORD16 ie_len);
extern void p2p_send_invit_ap(UWORD8 *msa, UWORD16 rx_len, UWORD8 *sa,
                              UWORD8 index);
extern void p2p_update_persist_cl_list(UWORD8 *if_addr);
extern BOOL_T p2p_probe_req_dev_id_cmp(UWORD8 *msa, UWORD16 rx_len);
extern BOOL_T p2p_probe_req_dev_type_cmp(UWORD8 *msa, UWORD16 rx_len);
extern void set_p2p_noa_sched_ap(UWORD8 *val);
extern void handle_p2p_prb_req_com_ap(UWORD8 *tx_dscr);
extern void handle_p2p_inv_req_com_ap(UWORD8 *tx_dscr);
extern void handle_p2p_invitation_to(void);
extern void p2p_handle_probe_rsp(UWORD8 *msa, UWORD16 rx_len);
extern void init_invi_scan_chan_list(void);
extern void p2p_end_ivitation_process(void);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function stores the details required to send the pending DD response */
/* frame at the end of the GO DISC REQ frame buffer                          */
INLINE void add_p2p_pending_dd_rsp_elem(UWORD8 dia_tok, UWORD8* sa_addr,
                                   UWORD8* frame_ptr, UWORD16 index)
{
    /* Copy the address of the sender of the Device Discoverability frame */
    mac_addr_cpy((frame_ptr + index), sa_addr);
    index += MAC_ADDRESS_LEN;

    /* Copy the dialog token */
    frame_ptr[index] = dia_tok;
}

#endif /* P2P_MGMT_AP_H */

#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */

