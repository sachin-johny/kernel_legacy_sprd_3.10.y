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
/*  File Name         : sta_frame_11n.c                                      */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to Station mode   */
/*                      of operation.                                        */
/*                                                                           */
/*  List of Functions : prepare_smps_frame                                   */
/*                      set_ht_opern_11n_sta                                 */
/*                      prepare_coex_mgmt_frame                              */
/*                                                                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "frame_11n.h"
#include "sta_frame_11n.h"
#include "sta_management_11n.h"
#include "phy_hw_if.h"

#if 0 // need not use
// 20120830 caisf add, merged ittiam mac v1.3 code
static UWORD8 set_2040_into_ch_report(UWORD8 *msa, UWORD8 index,
 									  UWORD8 num_chan, UWORD32 chan_report,
                                      UWORD8 rc_val);
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_smps_frame                                       */
/*                                                                           */
/*  Description   : This function prepares the SMPS action frame.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the SMPS frame                             */
/*                                                                           */
/*  Globals       : g_smps_mode                                              */
/*                                                                           */
/*  Processing    : The various fields of the SMPS action frame are set as   */
/*                  specified in the standard.                               */
/*                                                                           */
/*  Outputs       : The contents of the given SMPS action frame are set.     */
/*                                                                           */
/*  Returns       : UWORD16, SMPS action frame length                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_smps_frame(UWORD8 *data)
{
    UWORD16 index = 0;

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
    set_frame_control(data, (UWORD16)ACTION);

    /* DA is address of the AP (BSSID) */
    set_address1(data, mget_bssid());

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                SM Power Save Frame - Frame Body                       */
    /* --------------------------------------------------------------------- */
    /* | Category | Action | SM Power Save Enabled | SM Mode     | Reserved| */
    /* --------------------------------------------------------------------- */
    /* | 1        | 1      | B0                    | B1          | B2 - B7 | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Set the Category and Action fields */
    data[index++] = HT_CATEGORY;
    data[index++] = SM_POWER_SAVE_TYPE;

// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Clear the SMPS byte */
    data[index] = 0;

    /* SM Power Save Enabled bit is set if SMPS is enabled */
    if(g_smps_mode != MIMO_MODE)
        data[index] |= BIT0;

    /* SM Mode bit is set if Dynamic SMPS mode is enabled */
    if(g_smps_mode == DYNAMIC_MODE)
        data[index] |= BIT1;

    return SMPS_ACTION_FRAME_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ht_opern_11n_sta                                     */
/*                                                                           */
/*  Description   : This function sets the HT Information Field. This is     */
/*                  also known as the HT Operation Information Element in    */
/*                  the final 11n standard.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of the HT Information field                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The fields of the HT Information element are set in the  */
/*                  given frame, at the given index. The only fields valid   */
/*                   in IBSS mode are                                        */
/*                  1. Primary Channel    2. SCO                             */
/*                  3. STA Channel Width  4. Basic MCS Set                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the HT Information Field               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_ht_opern_11n_sta(UWORD8 *data, UWORD16 index)
{
    UWORD16 htinfo          = 0;
    UWORD32 primary_channel = 0;

    /* The HT Information field is set only if HT Option is implemented */
    if(mget_HighThroughputOptionImplemented() == TV_FALSE)
        return 0;

    data[index++] = IHTOPERATION;
    data[index++] = IHTOPERATION_LEN;

    /* HT Information Field                                                  */
    /* |-------------------------------------------------------------------| */
    /* | Primary | Seconday  | STA Ch | RIFS |           Reserved          | */
    /* | Channel | Ch Offset | Width  | Mode |                             | */
    /* |-------------------------------------------------------------------| */
    /* |    1    | B0     B1 |   B2   |  B3  |  B4                      B7 | */
    /* |-------------------------------------------------------------------| */
    /* | Operating  | Non-GF STAs  |  Reserved  | OBSS Non-HT   | Reserved | */
    /* |    Mode    |   Present    |            | STAs Present  |          | */
    /* |-------------------------------------------------------------------| */
    /* | B0      B1 |     B2       |    B3      |      B4       | B5   B15 | */
    /* |-------------------------------------------------------------------| */
    /* | Reserved   |  Dual   |  Dual CTS  | Secondary |  LSIG TXOP Protn  | */
    /* |            | Beacon  | Protection |  Beacon   |  Full Support     | */
    /* |-------------------------------------------------------------------| */
    /* | B0      B5 |    B6   |     B7     |     B8    |        B9         | */
    /* |-------------------------------------------------------------------| */
    /* |    PCO Active    |    PCO Phase   |  Reserved  |   Basic MCS Set  | */
    /* |-------------------------------------------------------------------| */
    /* |         B10      |        B11     | B12    B15 |       16         | */
    /* |-------------------------------------------------------------------| */

    /* Indicates the channel number of the primary channel */
    primary_channel = mget_CurrentPrimaryChannel();
    data[index]     = (UWORD8)primary_channel;

    /* Indicates offset of secondary channel relative to the primary channel */
    if(mget_FortyMHzOperationImplemented() == TV_TRUE)
    {
    }

    if(mget_FortyMHzOperationEnabled() == TV_TRUE)
    {
        htinfo |= BIT2;
    }

    data[index + 1] = (UWORD8)htinfo;

    /* These fields are reseverved in IBSS mode */
    data[index + 2] = 0;
    data[index + 3] = 0;
    data[index + 4] = 0;
    data[index + 5] = 0;

    /* 11N TBD - Frame preparation - Implementation Pending */
    /* Indicates the MCS values that are supported by all HT STAs in the BSS */
    mem_set(data+index+6, 0, 16);

    return (IE_HDR_LEN + IHTOPERATION_LEN);
}

#endif /* MAC_802_11N */
#endif /* IBSS_BSS_STATION_MODE */
