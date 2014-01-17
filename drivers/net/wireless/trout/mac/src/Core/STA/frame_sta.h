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
/*  File Name         : frame_sta.h                                          */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      preparation of the various MAC frames specific to    */
/*                      Station mode of operation.                           */
/*                                                                           */
/*  List of Functions : get_beacon_period                                    */
/*                      get_asoc_id                                          */
/*                      get_dtim_count                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef FRAME_STA_H
#define FRAME_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "transmit.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD16 prepare_probe_req(UWORD8* data);
extern UWORD16 prepare_beacon_sta(UWORD8* data);
extern UWORD16 prepare_auth_req(UWORD8* data);
extern UWORD16 prepare_auth_req_seq3(UWORD8* data, UWORD8* msa);
extern UWORD16 prepare_asoc_req(UWORD8* data);
extern UWORD16 prepare_probe_rsp_sta(UWORD8* data, UWORD8* probe_req,
                                     BOOL_T is_p2p);
extern UWORD8* get_tim_elm(UWORD8* data, UWORD16 len, UWORD16 offset);
extern void    set_cap_info_field_sta(UWORD8* data, UWORD16 index);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function extracts the beacon period field from the beacon or probe   */
/* response frame.                                                           */
INLINE UWORD16 get_beacon_period(UWORD8* data)
{
    UWORD16 bcn_per = 0;

    bcn_per  = data[0];
    bcn_per |= (data[1] << 8) ;

    return bcn_per;
}

/* This function extracts association ID from the incoming association       */
/* response frame and returns the same.                                      */
INLINE UWORD16 get_asoc_id(UWORD8* data)
{
    UWORD16 asoc_id = 0;

    asoc_id  = data[MAC_HDR_LEN + 4];
    asoc_id |= (data[MAC_HDR_LEN + 5] << 8) ;

    return asoc_id;
}

/* This function extracts DTIM count from the incoming TIM element and       */
/* returns the same.                                                         */
INLINE UWORD8 get_dtim_count(UWORD8* data)
{
    return data[2];
}

INLINE UWORD32 get_beacon_timestamp_lo(UWORD8* data)
{
    UWORD32 time_stamp = 0;
    UWORD32 index    = MAC_HDR_LEN;

    time_stamp |= data[index++];
    time_stamp |= (data[index++] << 8);
    time_stamp |= (data[index++] << 16);
    time_stamp |= (data[index]   << 24);

    return time_stamp;
}

INLINE UWORD32 get_beacon_timestamp_hi(UWORD8* data)
{
    UWORD32 time_stamp = 0;
    UWORD32 index    = (MAC_HDR_LEN + 4);

    time_stamp |= data[index++];
    time_stamp |= (data[index++] << 8);
    time_stamp |= (data[index++] << 16);
    time_stamp |= (data[index]   << 24);

    return time_stamp;
}

#endif /* FRAME_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
