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
/*  File Name         : frame_11n.c                                          */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to 802.11n        */
/*                      operation.                                           */
/*                                                                           */
/*  List of Functions : set_ht_cap_11n                                       */
/*                      set_ht_opern_11n                                     */
/*                      set_sec_choff_11n                                    */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "frame_11n.h"
#include "core_mode_if.h"
#include "blockack.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ht_cap_11n                                           */
/*                                                                           */
/*  Description   : This function sets the HT Capabilities field.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of the HT Capabilities field                    */
/*                  3) Frame Type/Subtype                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The fields of the HT Capabilities element are set in the */
/*                  given frame, at the given index.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the HT Capabilities Field              */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_ht_cap_11n(UWORD8 *data, UWORD16 index, TYPESUBTYPE_T frame_type)
{
    /* The HT Capability field is set only if HT Option is implemented */
    if(mget_HighThroughputOptionImplemented() == TV_FALSE)
        return 0;

    /* HT Capability Element                                               */
    /* ------------------------------------------------------------------- */
    /* | Elem | Len | HT Cap | A-MPDU | Supported | HT Ext | TxBF | ASEL | */
    /* |  ID  |     |  Info  | Params |  MCS Set  |  Cap.  | Cap. | Cap. | */
    /* ------------------------------------------------------------------- */
    /* |  1   |  1  |    2   |    1   |     16    |   2    |   4   |  1  | */
    /* ------------------------------------------------------------------- */

    data[index++] = IHTCAP;
    data[index++] = IHTCAP_LEN;

    /* Set HT capabilities information field */
    set_ht_capinfo(&data[index], frame_type);
    index += HT_CAPINFO_FIELD_LEN;

    /* Set A-MPDU parameters field */
    set_ampdu_params(&data[index]);
    index += AMPDU_PARAM_LEN;

    /* Set supported MCS set field */
    set_supp_mcs_set(&data[index]);
    index += MCS_SET_FIELD_LEN;

    /* Set HT extended capabilities information field */
    set_ht_extcap(&data[index]);
    index += HT_EXTCAP_FIELD_LEN;

    /* Set Tx beamforming field */
    set_txbf_cap(&data[index]);
    index += TX_BF_FIELD_LEN;

    /* Set ASEL capabilities field */
    set_asel_cap(&data[index]);
    index += ASEL_FIELD_LEN;

    return (IHTCAP_LEN + IE_HDR_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ht_opern_11n                                         */
/*                                                                           */
/*  Description   : This function sets the HT Operation Information Element  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of the HT Information field                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The fields of the HT Operation element are set in the    */
/*                  given frame, at the given index.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the HT Operation IE                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_ht_opern_11n(UWORD8 *data, UWORD16 index)
{
    /* The HT Information field is set only if HT Option is implemented */
    if(mget_HighThroughputOptionImplemented() == TV_FALSE)
        return 0;

    data[index++] = IHTOPERATION;
    data[index++] = IHTOPERATION_LEN;

    /* Set the HT information field */
    set_ht_opern(&data[index]);

    return (IE_HDR_LEN + IHTOPERATION_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_sec_channel_offset                                   */
/*                                                                           */
/*  Description   : This function sets the Secondary Channel Offset          */
/*                  information element.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of the Secondary Channel Offset element         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The fields of the Secondary Channel element are set in   */
/*                  the given frame, at the given index.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the Secondary Channel element          */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_sec_choff_11n(UWORD8 *data, UWORD16 index, TYPESUBTYPE_T frame_type)
{
    /* The Secondary Channel Offset is set only if HT Option is Implemented */
    if(mget_HighThroughputOptionImplemented() == TV_FALSE)
        return 0;

    /* For Beacon and Probe Response frames, this field is set only if the  */
    /* if SpectrumManagementRequired and FortyMHzOperationImplemented is    */
    /* true.                                                                */
    if((frame_type == BEACON) || (frame_type == PROBE_RSP))
    {
        if(mget_FortyMHzOperationImplemented() == TV_FALSE)
            return 0;
    }

    data[index++] = ISECCHOFF;
    data[index++] = ISECCHOFF_LEN;

    /* Set the secondary channel offset field */
    set_sec_choff_field(&data[index]);

    return (IE_HDR_LEN + ISECCHOFF_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_addba_req                                        */
/*                                                                           */
/*  Description   :                                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the addba frame                            */
/*                  2) Pointer to ADDBA request structure.                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       :                                                          */
/*  Returns       :                                                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_addba_req(UWORD8 *data, ba_tx_struct_t *ba, UWORD8 tid)
{
    UWORD16 index     = 0;
    UWORD16 ba_param  = 0;
    void    *ht_entry = NULL;

	TROUT_FUNC_ENTER;
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

    /* DA is address of STA requesting association */
    set_address1(data, ba->dst_addr);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Category */
    data[index++] = BA_CATEGORY;

    /* Action */
    data[index++] = ADDBA_REQ_TYPE;

    /* Dialog Token */
    data[index++] = ba->dialog_token;

    /* Block Ack Parameter set */
    /* B0 - AMSDU Allowed, B1- Immediate or Delayed block ack */
    /* B2-B5 : TID, B6-B15: Buffer size */
    ba_param  = (ba->ba_policy << 1);   /* BIT1 */
    ba_param |= (tid << 2);             /* BIT2 */
    ba_param |= (ba->buff_size << 6);   /* BIT6 */

    /* Update HT specific entries in the Block-Ack Parameter set.*/
    ht_entry = get_ht_handle_entry(ba->ba_data.entry);
    update_addba_req_param_ht(ht_entry, ba, &ba_param);

    data[index++] = (UWORD8)(ba_param & 0xFF);
    data[index++] = (UWORD8)((ba_param >> 8) & 0xFF);

    /* BlockAck timeout value */
    data[index++] = (UWORD8)(ba->timeout & 0xFF);
    data[index++] = (UWORD8)((ba->timeout >> 8) & 0xFF);

    /* Block ack starting sequence number is inserted by H/W */
    /* b0-b3 fragmentnumber, b4-b15: sequence number         */
    index += 2;

	TROUT_FUNC_EXIT;
    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_addba_rsp                                        */
/*                                                                           */
/*  Description   :                                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the addba frame                            */
/*                  2) Pointer to ADDBA request structure.                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       :                                                          */
/*  Returns       :                                                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_addba_rsp(UWORD8 *data, ba_rx_struct_t *addba_rsp, UWORD8 tid,
                          UWORD8 status)
{
    UWORD16 index     = 0;
    UWORD16 ba_param  = 0;

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

    /* DA is address of STA requesting association */
    set_address1(data, addba_rsp->dst_addr);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/
    /*************************************************************************/
    /*             ADDBA Response Frame - Frame Body                         */
    /*    ---------------------------------------------------------------    */
    /*    | Category | Action | Dialog | Status  | Parameters | Timeout |    */
    /*    ---------------------------------------------------------------    */
    /*    | 1        | 1      | 1      | 2       | 2          | 2       |    */
    /*    ---------------------------------------------------------------    */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Category */
    data[index++] = BA_CATEGORY;

    /* Action */
    data[index++] = ADDBA_RSP_TYPE;

    /* Dialog Token */
    /* Copy the dialog token from the request */
    data[index++] = addba_rsp->dialog_token;

    /* Status */
    data[index++] = status;
    data[index++]  = 0;

    /* Block Ack Parameter set */
    /* B0 - AMSDU Support, B1- Immediate or Delayed block ack */
    /* B2-B5 : TID, B6-B15: Buffer size */
#if 0    //force it support amsdu.
    printk("prepare_addba_rsp: amsdu_supp = %d\n", addba_rsp->amsdu_supp);
    if(addba_rsp->amsdu_supp == 0)
    	ba_param = 1;
#else
	ba_param  = addba_rsp->amsdu_supp;         /* BIT0 */
#endif    	
	printk("prepare_addba_rsp: amsdu_supp = %d\n", addba_rsp->amsdu_supp);
    ba_param |= (addba_rsp->ba_policy << 1);   /* BIT1 */
    ba_param |= (tid << 2);                    /* BIT2 */
    ba_param |= (addba_rsp->buff_size << 6);   /* BIT6 */

    data[index++] = (UWORD8)(ba_param & 0xFF);
    data[index++] = (UWORD8)((ba_param >> 8) & 0xFF);

    /* BlockAck timeout value */
    data[index++] = (UWORD8)(addba_rsp->timeout & 0xFF);
    data[index++] = (UWORD8)((addba_rsp->timeout >> 8) & 0xFF);

    return index + FCS_LEN;

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_delba_req                                        */
/*                                                                           */
/*  Description   :                                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the addba frame                            */
/*                  2) Pointer to ADDBA request structure.                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       :                                                          */
/*  Returns       :                                                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_delba_req(UWORD8 *data, UWORD8 *addr, UWORD8 tid,
                          UWORD8 initiator, UWORD8 reason)
{
    UWORD16 index = 0;

	TROUT_FUNC_ENTER;
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

    /* DA is address of STA requesting association */
    set_address1(data, addr);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Category */
    data[index++] = BA_CATEGORY;

    /* Action */
    data[index++] = DELBA_TYPE;

    /* DELBA parameter set */
    /* B0 - B10 -reserved */
    /* B11 - initiator */
    /* B12-B15 - TID */
    data[index++]  = 0;
    data[index]    = initiator << 3;
    data[index++] |= (tid << 4);

    /* Reason field */
    /* Reason can be either of END_BA, QSTA_LEAVING, UNKNOWN_BA */
    data[index++] = reason;
    data[index++] = 0;

	TROUT_FUNC_EXIT;
    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_blockack_req                                     */
/*                                                                           */
/*  Description   :         TBD                                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the addba frame                            */
/*                  2) Pointer to ADDBA request structure.                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       :                                                          */
/*  Returns       :                                                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 prepare_blockack_req(UWORD8 *data, ba_tx_struct_t *ba, UWORD8 tid)
{
    UWORD16 bar_ctl =0;

    /*************************************************************************/
    /*                     BlockAck Request Frame Format                     */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BAR Control|BlockAck Starting    |FCS|  */
    /* |             |        |  |  |           |Sequence number      |   |  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |2          |2                    |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)BLOCKACK_REQ);

    /* Set DA to the address of the STA requesting authentication */
    set_address1(data, ba->dst_addr);

    /* Set SA to the dot11MacAddress */
    set_address2(data, mget_StationID());

    /* BAR Control field */
    bar_ctl = (tid << 12);

    /* BAR-Ack Policy is set to Normal Ack */
    bar_ctl &= ~BIT0;

    if(ba->back_var == COMPRESSED_BACK)
    {
        bar_ctl |= BIT2;
	}
    else if(ba->back_var == MULTI_TID_BACK)
	{
        bar_ctl |= BIT2;
        bar_ctl |= BIT1;
    }

    data[16] = bar_ctl & 0xFF;
    data[17] = (bar_ctl >> 8) & 0xFF;

    /* Sequence number */
    data[18] = (ba->win_start << 4);
    data[19] = ((ba->win_start >> 4) & 0xFF);

    return 20 + FCS_LEN;
}

#endif /* MAC_802_11N */

