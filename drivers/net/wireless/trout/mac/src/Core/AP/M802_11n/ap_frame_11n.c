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
/*  File Name         : ap_frame_11n.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to 802.11n Access */
/*                      Point mode of operation.                             */
/*                                                                           */
/*  List of Functions : set_ht_opern_11n_ap                                  */
/*                      set_obss_scan_params_ap                              */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "frame_11n.h"
#include "ap_frame_11n.h"
#include "ap_management_11n.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ht_opern_11n_ap                                      */
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
/*                  given frame, at the given index.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the HT Information Field               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_ht_opern_11n_ap(UWORD8 *data, UWORD16 index)
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

    if(mget_FortyMHzOperationEnabled() == TV_TRUE)
    {
        htinfo |= BIT2;

    }

    /* Indicates whether use of RIFS is permitted within the BSS */
    if(mget_RIFSMode() == TV_TRUE)
    {
        htinfo |= BIT3;
    }

    data[index + 1] = (UWORD8)htinfo;

    htinfo = 0;

    /* Indicates operating mode of the BSS from which protection requirement */
    /* of HT transmissions may be determined.                                */
    htinfo |= mget_HTOperatingMode();

    /* Indicates if any HT STA that is not Greenfield capable has associated */
    /* Determines when a non-AP STA should use greenfield protection.        */
    if(mget_NonGFEntitiesPresent() == TV_TRUE)
    {
        htinfo |= BIT2;
    }

    /* Set the OBSS Non-HT STAs Present bit if there are any non-HT STAs     */
    /* associated with the AP or if any non-HT STAs were detected in OBSS    */
    /* and the 11n detection flag is set to DETECT_PROTECT_REPORT            */
    if(get_obss_non_htsta_present_bit() == 1)
    {
        htinfo |= BIT4;
    }

    data[index + 2] = htinfo & 0x00FF;
    data[index + 3] = (htinfo & 0xFF00) >> 8;

    htinfo = 0;

    /* 11N TBD - Frame preparation - Implementation Pending */
    /* Indicates whether the AP transmits a a secondary beacon               */

    /* Dual CTS Protection is used by the AP to set a NAV at STAs that do    */
    /* not support STBC and at STAs that can associate solely through the    */
    /* secondary (STBC frame) beacon.                                        */
    if(mget_DualCTSProtection() == TV_TRUE)
    {
        htinfo |= BIT7;
    }

    /* 11N TBD - Frame preparation - Implementation Pending */
    /* Indicates whether the beacon containing this element is a primary or  */
    /* a secondary beacon. The secondary beacon has half a beacon period     */
    /* shift relative to the primary beacon.                                 */


    /* Indicates whether all HT STA in the BSS support L-SIG TXOP Protection */
    if(mget_LSigTxopFullProtectionEnabled() == TV_TRUE)
    {
        htinfo |= BIT9;
    }

    /* Indicates whether PCO is active in the BSS */
    if(mget_PCOActivated() == TV_TRUE)
    {
        htinfo |= BIT10;
    }

    /* 11N TBD - Frame preparation - Implementation Pending */
    /* Indicates the PCO phase of operation.                                */
    /* Defined only in a Beacon and Probe Response frames when PCO Active   */
    /* is 1. Otherwise reserved.                                            */

    data[index + 4] = htinfo & 0x00FF;
    data[index + 5] = (htinfo & 0xFF00) >> 8;

    /* 11N TBD - Frame preparation - Implementation Pending */
    /* Indicates the MCS values that are supported by all HT STAs in the BSS */
    mem_set(data+index+6, 0, 16);

    return (IE_HDR_LEN + IHTOPERATION_LEN);
}
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */
