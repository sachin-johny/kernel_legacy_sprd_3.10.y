/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : ap_frame_p2p.h                                       */
/*                                                                           */
/*  Description       : This file contains the extern declaration of         */
/*                      functions required to handle P2P frame in GO mode    */
/*                                                                           */
/*  List of Functions :                                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_P2P

#ifndef AP_FRAME_P2P_H
#define AP_FRAME_P2P_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "frame_p2p.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern UWORD16 add_asoc_resp_p2p_ie(UWORD8* data, UWORD16 index,
                                    UWORD16 status);
extern void send_p2p_dd_resp(UWORD8 *da, UWORD8 dia_token,
                             P2P_STATUS_CODE_T status);
extern void send_p2p_presence_resp(UWORD8 *frame, UWORD8 *sa);
extern void *find_p2p_client_entry(UWORD8* trgt_dev_addr, UWORD8* if_addr);
extern void send_p2p_go_disc_req(UWORD8 *req_dev_addr, UWORD8 dia_token,
                                 UWORD8 *da);
extern UWORD16 add_p2p_beacon_ie(UWORD8* data, UWORD16 index);


#endif /* AP_FRAME_P2P_H */
#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */
