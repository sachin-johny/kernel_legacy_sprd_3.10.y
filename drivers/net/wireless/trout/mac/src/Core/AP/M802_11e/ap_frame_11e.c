/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : ap_frame_11e.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions for the         */
/*                      preparation of the various MAC frames specific to    */
/*                      AP mode of operation in 802.11e.                     */
/*                                                                           */
/*  List of Functions : prepare_wmm_addts_rsp                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ap_frame_11e.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_wmm_addts_rsp                                    */
/*                                                                           */
/*  Description   : This function prepares the ADDTS response frame.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the ADDTS response frame                   */
/*                  2) Pointer to the incoming ADDTS request frame           */
/*                  3) Length of the ADDTS request frame                     */
/*                  4) MAC adress of non-AP QSTA sending the request         */
/*                  5) Status code                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the ADDTS response frame are set   */
/*                  as specified in the standard using the incoming request  */
/*                  parameters and the given status.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16, ADDTS response frame length                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_wmm_addts_rsp(UWORD8 *frame, UWORD8 *msa, UWORD16 len,
                              UWORD8 *sa, UWORD8 status)
{
    UWORD16 index = 0;
    UWORD16 i     = 0;
    UWORD8  *tspec = 0;

    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(frame, (UWORD16)ACTION);

    /* Set the address fields */
    set_address1(frame, sa);
    set_address2(frame, mget_bssid());
    set_address3(frame, mget_StationID());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                WMM ADDTS Response Frame - Frame Body                  */
    /* --------------------------------------------------------------------- */
    /* |Category     |Action      |Token      |Status      |TSPEC          | */
    /* --------------------------------------------------------------------- */
    /* |1            |1           |1          |1           | 61            | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Set the Category (QoS), Action (ADDTS Response) and the dialog token  */
    /* (same as received request dialog token).                              */
    frame[index++] = WMM_CATEGORY;
    frame[index++] = ADDTS_RSP_TYPE;
    frame[index++] = get_dialog_token(msa);

    /* Set the status as required */
    frame[index++] = (UWORD8)(status & 0x00FF);

    /* Copy TSPEC elements from the incoming ADDTS request frame */
    tspec = get_addts_req_tspec(msa);

    for(i = 0; i < TSPEC_ELEMENT_LEN ; i++)
        frame[index++] = tspec[i];

    /* Return the length of the ADDTS response frame */
    return index + FCS_LEN;
}

#endif /* MAC_WMM */
#endif /* BSS_ACCESS_POINT_MODE */
