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
/*  File Name         : p2p_wlan_rx_mgmt_sta.h                               */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to SME functions specific to P2P STA mode.   */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

#ifndef P2P_WLAN_RX_MGMT_STA_H
#define P2P_WLAN_RX_MGMT_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "sta_mgmt_p2p.h"

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern UWORD8  g_p2p_go_dev_id[];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern P2P_STATUS_CODE_T process_p2p_go_neg_req(UWORD8 *msa, UWORD16 rx_len);
extern SWORD8 process_p2p_go_neg_rsp(UWORD8 *msa, UWORD16 rx_len);
extern STATUS_T process_p2p_go_neg_cnf(UWORD8 *msa, UWORD16 rx_len);
extern void p2p_wait_scan_handle_action_rx(mac_struct_t *mac, UWORD8 *msa,
                                           UWORD16 rx_len);
extern void p2p_wait_join_handle_action_rx(mac_struct_t *mac, UWORD8 *msa,
                                           UWORD16 rx_len);
extern void p2p_wait_scan_process_go_neg_req(UWORD8 *msa, UWORD16 rx_len);
extern void p2p_wait_scan_process_inv_req(mac_struct_t *mac, UWORD8 *msa, 
                                          UWORD16 rx_len);
extern void p2p_wait_join_process_go_neg_req(mac_struct_t *mac, UWORD8 *msa,
                                             UWORD16 rx_len);
extern void p2p_wait_join_process_go_neg_rsp(mac_struct_t *mac, UWORD8 *msa,
                                      UWORD16 rx_len);
extern void p2p_wait_join_process_go_neg_cnf(mac_struct_t *mac, UWORD8 *msa,
                                      UWORD16 rx_len);
extern void p2p_wait_join_process_dev_disc_rsp(UWORD8 *msa, UWORD16 rx_len);
extern void p2p_wait_join_process_inv_rsp(mac_struct_t *mac, UWORD8 *msa,
                                   UWORD16 rx_len);
extern void sta_enabled_rx_p2p_gen_action(UWORD8* msa, UWORD16 rx_len);
extern void sta_enabled_rx_p2p_action(UWORD8 *msg);
extern void p2p_sta_wait_join(mac_struct_t *mac, UWORD8 *msg);
extern void p2p_handle_noa(UWORD8 *msa, UWORD16 rx_len);
extern UWORD16 add_p2p_ie_probe_req(UWORD8 *data, UWORD16 index);
extern void p2p_wait_join_process_prov_disc_rsp(mac_struct_t *mac, UWORD8 *msa,
                                         UWORD16 rx_len);


#endif /* P2P_WLAN_RX_MGMT_STA_H */

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */

