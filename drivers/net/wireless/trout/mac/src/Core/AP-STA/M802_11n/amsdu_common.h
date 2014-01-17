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
/*  File Name         : amsdu_common.h                                       */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions which is common between AMSDU aggregation  */
/*                      and deaggregation                                    */
/*                                                                           */
/*  List of Functions : set_submsdu_sa                                       */
/*                      set_submsdu_da                                       */
/*                      set_submsdu_len                                      */
/*                      get_submsdu_sa                                       */
/*                      get_submsdu_da                                       */
/*                      get_submsdu_len                                      */
/*                      get_submsdu_pad_len                                  */
/*                      get_submsdu_frame_len                                */
/*                      set_submsdu_padding                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

#ifndef AMSDU_COMMON_H
#define AMSDU_COMMON_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "buff_desc.h"

/*****************************************************************************/
/* Constants Definitions                                                     */
/*****************************************************************************/

#define SUB_MSDU_DA_OFFSET          0
#define SUB_MSDU_SA_OFFSET          6
#define SUB_MSDU_LENGTH_OFFSET      12

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the Source Address in the sub-MSDU header */
INLINE void set_submsdu_sa(UWORD8 *submsdu_hdr, UWORD8 *addr)
{
    memcpy(submsdu_hdr + SUB_MSDU_SA_OFFSET, addr, 6);
}

/* This function sets the Destination Address in the sub-MSDU header */
INLINE void set_submsdu_da(UWORD8 *submsdu_hdr, UWORD8 *addr)
{
    memcpy(submsdu_hdr + SUB_MSDU_DA_OFFSET, addr, 6);
}

/* This function parses the sub-msdu header to extract the sub-msdu length */
INLINE void set_submsdu_len(UWORD8 *submsdu_hdr, UWORD16 sub_msdu_len)
{

    *(submsdu_hdr + SUB_MSDU_LENGTH_OFFSET + 1) = sub_msdu_len & 0x00FF;
    *(submsdu_hdr + SUB_MSDU_LENGTH_OFFSET)     = (sub_msdu_len & 0xFF00) >> 8;

}

/* This function extracts the Source Address from the sub-MSDU header */
INLINE void get_submsdu_sa(UWORD8 *submsdu_hdr, UWORD8 *addr)
{
    memcpy(addr, submsdu_hdr + SUB_MSDU_SA_OFFSET, 6);
}

/* This function extracts the Destination Address from the sub-MSDU header */
INLINE void get_submsdu_da(UWORD8 *submsdu_hdr, UWORD8 *addr)
{
    memcpy(addr, submsdu_hdr + SUB_MSDU_DA_OFFSET, 6);
}

/* This function parses the sub-msdu header to extract the sub-msdu length */
INLINE UWORD16 get_submsdu_len(UWORD8 *submsdu_hdr)
{
    UWORD16 sub_msdu_len = 0;

    sub_msdu_len = *(submsdu_hdr + SUB_MSDU_LENGTH_OFFSET);

    sub_msdu_len = (sub_msdu_len << 8) +
                   *(submsdu_hdr + SUB_MSDU_LENGTH_OFFSET + 1);

    return sub_msdu_len;
}

/* This function computes the length of the padding required to be applied   */
/* to a sub-msdu. Padding is applied so that the Subframe Header, plus MSDU  */
/* plus Padding has a length that is a multiple of 4.                        */
INLINE UWORD8 get_submsdu_pad_len(UWORD16 sub_msdu_len)
{
    UWORD8 length_off = sub_msdu_len & 0x3;

    if(length_off)
        return (4 - length_off);

    return 0;
}

/* This function returns the length of the sub-msdu including the header     */
/* msdu and the padding.                                                     */
INLINE UWORD16 get_submsdu_frame_len(UWORD8 *submsdu_hdr)
{
    UWORD16 sub_msdu_frame_len = SUB_MSDU_HEADER_LENGTH;

    /* Parse the Sub-MSDU header to get the sub-MSDU length */
    sub_msdu_frame_len += get_submsdu_len(submsdu_hdr);

    /* The padding length is computed */
    sub_msdu_frame_len += get_submsdu_pad_len(sub_msdu_frame_len);

    return sub_msdu_frame_len;
}

/* This function sets the header fields of the sub-MSDU */
INLINE void set_submsdu_header_field(UWORD8 *sub_msdu_hdr, UWORD8 *sa,
                                     UWORD8 *da, UWORD16 data_len )
{
    /* Set the sub-MSDU Source Address */
    set_submsdu_sa(sub_msdu_hdr, sa);

    /* Set the sub-MSDU Destination Address Field */
    set_submsdu_da(sub_msdu_hdr, da);

    /* Set the sub-MSDU Length Field */
    set_submsdu_len(sub_msdu_hdr, data_len);
}

/* This function pads the sub-MSDU to make its length a multiple of 4 */
/* The number of bytes padded is returned.                            */
INLINE UWORD8 set_submsdu_padding_bytes(UWORD8 *submsdu_tail,
                                         UWORD16 submsdu_len)
{
    UWORD8 pad_len = get_submsdu_pad_len(submsdu_len);
    UWORD8 indx    = 0;

    for(indx =0; indx < pad_len; indx++)
        submsdu_tail[indx] = 0;

    return pad_len;
}

#endif /* AMSDU_COMMON_H */
#endif /* MAC_802_11N */
