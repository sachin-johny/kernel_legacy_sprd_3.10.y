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
/*  File Name         : frame_ap.h                                           */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      preparation of the various MAC frames specific to    */
/*                      AP mode of operation.                                */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef FRAME_AP_H
#define FRAME_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "frame.h"
#include "mib.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define CHTXT_SIZE              128
#define DEFAULT_TIM_LEN         4
#define ASOC_ID_LEN             2

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD16 prepare_auth_rsp(UWORD8* data, UWORD8* auth_req);
extern UWORD16 prepare_asoc_rsp(UWORD8* data,    UWORD8* asoc_req,
                                UWORD16 asoc_id, UWORD16 status,
                                TYPESUBTYPE_T  type, UWORD16 asoc_len,
                                UWORD8 ht_enabled_sta, BOOL_T is_p2p);
extern UWORD16 prepare_disasoc(UWORD8* data, UWORD8* da, UWORD16 reason_code);
extern UWORD16 prepare_beacon_ap(UWORD8* data);
extern UWORD16 prepare_probe_rsp_ap(UWORD8* data, UWORD8* probe_req,
                                    BOOL_T is_p2p);
extern BOOL_T  is_challenge_txt_equal(UWORD8* data, UWORD8 *tx_ch_text);
extern void    set_cap_info_field_ap(UWORD8* data, UWORD16 index);
// 20120709 caisf add, merged ittiam mac v1.2 code
extern UWORD16 prepare_probe_request_ap(UWORD8 *data);

#endif /* FRAME_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
