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
/*  File Name         : frame_p2p.c                                          */
/*                                                                           */
/*  Description       : This file conatins the functions to create P2P frames*/
/*                                                                           */
/*  List of Functions : is_p2p_frame                                         */
/*                      get_p2p_dev_cap                                      */
/*                      get_p2p_grp_cap                                      */
/*                      add_p2p_mgmt_frame_hdr                               */
/*                      add_p2p_pub_act_hdr                                  */
/*                      add_p2p_gen_act_hdr                                  */
/*                      add_p2p_device_info_attr                             */
/*                      add_p2p_ie_probe_rsp                                 */
/*                      p2p_send_inv_rsp                                     */
/*                      p2p_send_inv_req                                     */
/*                      send_prov_disc_resp                                  */
/*                      p2p_int_chan_list                                    */
/*                      p2p_get_operating_class                              */
/*                      add_p2p_ie_probe_req                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_P2P
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "frame.h"
#include "frame_p2p.h"
#include "mib_p2p.h"
#include "core_mode_if.h"
#include "iconfig.h"
#include "mgmt_p2p.h"
#ifdef IBSS_BSS_STATION_MODE
#include "sta_mgmt_p2p.h"
#endif /* IBSS_BSS_STATION_MODE */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
#ifdef P2P_NO_COUNTRY
/* MD-TBD : To be removed when using Regulatory domains */
UWORD8 g_p2p_chan_entry_list[MAX_CHANNEL_FREQ_2 + 2] = {0,};
#endif /* P2P_NO_COUNTRY */

UWORD8 g_p2p_len_chan_entry_list = 0;


#ifndef MAC_HW_UNIT_TEST_MODE
/*****************************************************************************/
/*                                                                           */
/*  Function Name :  is_p2p_frame                                            */
/*                                                                           */
/*  Description   :  This function checks if the incoming frame is P2P frame */
/*                                                                           */
/*  Inputs        :  1) Pointer to the start of the incoming frame           */
/*                   2) Index to the start of the Tagged Parameters          */
/*                   3) Length of the received frame                         */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    : This function checks if the incoming frame is a P2P frame*/
/*                  by searching for the presence of P2P IE in it            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16; Index to the start of the P2P IE                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 is_p2p_frame(UWORD8 *msa, UWORD16 index, UWORD16 rx_len)
{
    /* Loop and search for P2P IE */
    while(index < (rx_len - FCS_LEN))
    {
        if(BTRUE == is_p2p_ie(msa + index))
        {
            /* P2P IE found */
            /* Return the index to the start of the IE. */
            return index;

        }
        else
        {
            /* If the current IE is not a P2P IE, move to the next IE. */
            index += (msa[index + 1] + IE_HDR_LEN);
        }
    }
    return index;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_p2p_dev_cap                                          */
/*                                                                           */
/*  Description   : This function generates the dev capability bitmap        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The function updates the dev capabilty bitmap based on   */
/*                  MIB parameters                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD8 dev capability bitmap                             */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 get_p2p_dev_cap(void)
{
    UWORD8 p2p_dev_cap = 0;

    /* Device capability bit map */
    /* |-------------------------------------------------------------------| */
    /* | Service     | P2P Client     | Concurrent  | P2P Infrastructure   | */
    /* | Discovery   | Discovery      | Operation   | Managed              | */
    /* |-------------------------------------------------------------------| */
    /* |    B0       |     B1         |     B2      |         B3           | */
    /* |-------------------------------------------------------------------| */
    /* | P2P Device  | P2P Invitation |            Reserved                | */
    /* | Limit       | Procedure      |                                    | */
    /* |-------------------------------------------------------------------| */
    /* |     B4      |     B5         |             B6-B7                  | */
    /* |-------------------------------------------------------------------| */

    if(BTRUE == mget_serv_discovery())
    {
        p2p_dev_cap |= BIT0;
    }

    if(BTRUE == mget_p2p_discoverable())
    {
        p2p_dev_cap |= BIT1;
    }

    if(BTRUE == mget_conc_op())
    {
        p2p_dev_cap |= BIT2;
    }

    if(BTRUE == mget_p2p_infra_managed())
    {
        p2p_dev_cap |= BIT3;
    }

    if(BTRUE == mget_p2p_dev_lmt())
    {
        p2p_dev_cap |= BIT4;
    }

    if(BTRUE == mget_p2p_invit_proc())
    {
        p2p_dev_cap |= BIT5;
    }

    return p2p_dev_cap;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_p2p_grp_cap                                          */
/*                                                                           */
/*  Description   : This function generates the group capability bitmap      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The function updates the group capabilty bitmap based on */
/*                  MIB parameters                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD8 group capability bitmap                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 get_p2p_grp_cap(void)
{
    UWORD8 p2p_grp_cap = 0;

    /* Group  capability bit map */
    /* |-------------------------------------------------------------------| */
    /* |  P2P Group   |  Persistent     |  P2P Group   |  Intra-BSS        | */
    /* |  Owner       |  P2P Group      |  Limit       |  Distribution     | */
    /* |-------------------------------------------------------------------| */
    /* |     B0       |      B1         |     B2       |       B3          | */
    /* |-------------------------------------------------------------------| */
    /* |  Cross       |  Persistent     |  Group       |  Reserved         | */
    /* |  Connection  |  Reconnect      |  Formation   |                   | */
    /* |-------------------------------------------------------------------| */
    /* |     B4       |      B5         |     B6       |       B7          | */
    /* |-------------------------------------------------------------------| */

    if(BTRUE == mget_p2p_GO())
    {
        p2p_grp_cap |= BIT0;
    }

    if(BTRUE == mget_p2p_persist_grp())
    {
        p2p_grp_cap |= BIT1;
    }

    if(1 == mget_p2p_grp_lmt())
    {
        p2p_grp_cap |= BIT2;
    }

    if(BTRUE == mget_p2p_intra_bss())
    {
        p2p_grp_cap |= BIT3;
    }

    if(BTRUE == mget_crs_connect())
    {
        p2p_grp_cap |= BIT4;
    }

    if(BTRUE == mget_presistReconn())
    {
        p2p_grp_cap |= BIT5;
    }

    if(BTRUE == mget_grp_formation())
    {
        p2p_grp_cap |= BIT6;
    }

    return p2p_grp_cap;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_mgmt_frame_hdr                                   */
/*                                                                           */
/*  Description   : This function adds the header for the management frame   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the frame                     */
/*                  2) Pointer to the destination address of the frame       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the header for the management frame   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void add_p2p_mgmt_frame_hdr(UWORD8* frm_ptr, UWORD8* p2p_dev_id)
{

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
	TROUT_FUNC_ENTER;
    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(frm_ptr, (UWORD16)ACTION);

    /* DA is the device id of the P2P device */
    set_address1(frm_ptr, p2p_dev_id);

    /* SA is the P2P device ID */
    set_address2(frm_ptr, mget_p2p_dev_addr());

    /*!!!!!!!!!!!!!!!!!!!!!! Need to verify !!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    /* The bssid is set to the broadcast address */
    set_address3(frm_ptr, mget_bcst_addr());
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_pub_act_hdr                                      */
/*                                                                           */
/*  Description   : This funtion adds the action frame header for the p2p    */
/*                  public action frame                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the frame                     */
/*                  2) Dialog token                                          */
/*                  3) OUI subtype                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This funtion adds the action frame header for the p2p    */
/*                  public action frame and also calls the function to add   */
/*                  the IE header for the first IE and returns the index to  */
/*                  the start of the first attribute                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Index to the start of the first P2P attribute   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_p2p_pub_act_hdr(UWORD8* frm_ptr, UWORD8 dlog_tkn,
                            P2P_PUB_ACTION_FRM_TYPE oui_subtype)
{
    /*                Set the contents of the frame body                  */
    /**********************************************************************/
    /*                Vendor specific PUBLIC action frame                 */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /* |Category | Action | OUI | OUI type | OUI Subtype | Dialog Token | */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /* |    1    |   1    |  3  |     1    |      1      |       1      | */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /**********************************************************************/

    /* Initialize index and the frame data pointer */
    UWORD8 index = MAC_HDR_LEN;

    frm_ptr[index++]   = PUBLIC_CATEGORY;   /* Category     */
    frm_ptr[index++]   = VENDOR_PUBLIC_ACT; /* Action Field */
    frm_ptr[index++]   = WFA_OUI_BYTE1;     /* OUI          */
    frm_ptr[index++]   = WFA_OUI_BYTE2;     /* OUI          */
    frm_ptr[index++]   = WFA_OUI_BYTE3;     /* OUI          */
    frm_ptr[index++]   = WFA_P2P_v1_0;      /* OUI type     */
    frm_ptr[index++]   = oui_subtype;       /* OUI Subtype  */
    frm_ptr[index++]   = dlog_tkn;          /* Dialog Token */

    /* Add the P2P IE */
    if((P2P_PROV_DISC_RSP != oui_subtype))
        index += add_p2p_ie_hdr(frm_ptr + index);

    return index;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_gen_act_hdr                                      */
/*                                                                           */
/*  Description   : This funtion adds the action frame header for the p2p    */
/*                  general action frame                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the frame                     */
/*                  2) Dialog token                                          */
/*                  3) OUI subtype                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This funtion adds the action frame header for the p2p    */
/*                  general action frame and also calls the function to add  */
/*                  the IE header for the first IE and returns the index to  */
/*                  the start of the first attribute                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Index to the start of the first P2P attribute   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_p2p_gen_act_hdr(UWORD8* frm_ptr, UWORD8 dlog_tkn,
                            P2P_GEN_ACTION_FRM_TYPE oui_subtype)
{
    /* Initialize index and the frame data pointer */
    UWORD16 index = MAC_HDR_LEN;

    /*                Set the contents of the frame body                  */
    /**********************************************************************/
    /*                Vendor specific PUBLIC action frame                 */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /* |Category | Action | OUI | OUI type | OUI Subtype | Dialog Token | */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /* |    1    |   1    |  3  |     1    |      1      |       1      | */
    /* +---------+--------+-----+----------+-------------+--------------+ */
    /**********************************************************************/

    frm_ptr[index++]   = VENDOR_CATEGORY;   /* Category     */
    frm_ptr[index++]   = WFA_OUI_BYTE1;     /* OUI          */
    frm_ptr[index++]   = WFA_OUI_BYTE2;     /* OUI          */
    frm_ptr[index++]   = WFA_OUI_BYTE3;     /* OUI          */
    frm_ptr[index++]   = WFA_P2P_v1_0;      /* OUI type     */
    frm_ptr[index++]   = oui_subtype;       /* OUI Subtype  */
    frm_ptr[index++]   = dlog_tkn;          /* Dialog Token */

    /* Add the P2P IE if the oui subtype is not GO Discoverability request*/
    if(GO_DISC_REQ != oui_subtype)
        index += add_p2p_ie_hdr(frm_ptr + index);

    return index;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_device_info_attr                                 */
/*                                                                           */
/*  Description   : This function adds the device info attribute to the frame*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the frame                     */
/*                  2) Index to the start of the P2P device info attribute   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the device info attribute to the frame*/
/*                  and returns the total length of the attribute            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16; Total length of the attribute                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_p2p_device_info_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD8* dev_name          = 0;
    UWORD16 attr_len          = 0;
    UWORD16 wps_config        = 0;
    UWORD16 attr_start_offset = index;

	TROUT_FUNC_ENTER;
    ie_ptr[index] = P2P_DEVICE_INFO;       /* Attribute ID     */
    index += P2P_ATTR_HDR_LEN;

    /* Copy the device address  */
    mac_addr_cpy((ie_ptr + index), mget_StationID());
    index += 6;

    /* Copy the config method */
    wps_config  = get_wps_config_method();
    PUT_U16_BE((ie_ptr + index), wps_config);
    index += 2;


    /* Copy the primary device type  */
    /* Note : The get_prim_dev_type() returns the pointer to the start of the*/
    /* memory location which stores the length of the primary device type in */
    /* the first 2 bytes and the attribute in the next 8 bytes               */
    memcpy((ie_ptr + index), get_prim_dev_type() + 2, 8);
    index += 8;

    /* Number of secondary device type */
    ie_ptr[index++] = 0; /* or a funtion get_num_secondary_device() */

    /* assuming no seconday device */
    /* Copy the device name        */
    PUT_U16_BE((ie_ptr + index), WPS_ATTR_DEV_NAME_P2P);
    index += 2;
    dev_name  = get_dev_name();
    PUT_U16_BE((ie_ptr + index), dev_name[0]);
    index += 2;
    memcpy((ie_ptr + index), &dev_name[1], dev_name[0]);
    index += dev_name[0];

    /* Update the length field of the attribute */
    attr_len = index - (attr_start_offset + P2P_ATTR_HDR_LEN);
    ie_ptr[attr_start_offset + 1] = attr_len & 0x00FF;
    ie_ptr[attr_start_offset + 2] = attr_len >> 8;

	TROUT_FUNC_EXIT;
    return(attr_len + P2P_ATTR_HDR_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_ie_probe_rsp                                     */
/*                                                                           */
/*  Description   : This function adds the P2P IE to the probe response frame*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the probe response frame      */
/*                  2) Index to the start of the P2P IE                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the P2P IE along with the various     */
/*                  attributes required for the probe response frame and     */
/*                  returns the total length of the P2P IE added             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Total lenght of the P2P IE added                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 add_p2p_ie_probe_rsp(UWORD8 *data, UWORD16 index)
{
    UWORD16 added_attr_len    = 0;
    UWORD16 start_index       = 0;
    UWORD16 grp_info_len      = 0;
    UWORD16 curr_grp_info_len = 0;
    UWORD16 scr_index         = 0;
    UWORD16 total_len         = 0;
    UWORD8 *scrch_ptr         = 0;

    /* This processing may require scratch memory,so save the current scratch*/
    /* memory index for restoring later                                      */
    UWORD16 scratch_mem_idx = get_scratch_mem_idx();

    start_index = index;

    /* Add P2P IE header */
    index += add_p2p_ie_hdr(&data[index]);

    /* Add the P2P capability attribute */
    index += add_p2p_capability_attr(data, index, get_p2p_dev_cap(),
                                     get_p2p_grp_cap());

    /* Add the P2P device info attribute */
    index += add_p2p_device_info_attr(data, index);

#ifdef ENABLE_P2P_GO_TEST_SUPPORT
    /* Add P2P NOA attribute */
    index += add_NOA_attr(data, index, BTRUE);
    /* Should be present only in AP SW (GO) but since it is only for test    */
    /* support mode-ification is not done. The flag should anyway be defined */
    /* only for AP mode build.                                               */
#endif /* ENABLE_P2P_GO_TEST_SUPPORT */

    /* Get the length of the attributes added till now */
    added_attr_len = index - (start_index + P2P_IE_HDR_LEN);

    /* Allocate a scratch memory to copy the group info attribute */
    scrch_ptr = (UWORD8 *)scratch_mem_alloc(MAX_COMB_P2P_ATTR_LEN);

    if(NULL != scrch_ptr)
    {
        /* Copy the group info attribute to the scratch memory */
        grp_info_len = add_p2p_grp_info_attr_mode(scrch_ptr, 0);


        /* Check if the length of the group info attribute is such that it   */
        /* cannot be fit in the current IE and hence copy it to required     */
        /* number of IEs                                                     */
        if(grp_info_len > 0)
        {
            if((grp_info_len + added_attr_len) > P2P_MAX_IE_BODY_SIZE)
            {
                curr_grp_info_len = P2P_MAX_IE_BODY_SIZE - added_attr_len;
                do
                {
                    memcpy(&data[index], &scrch_ptr[scr_index],
                           curr_grp_info_len);
                    index += curr_grp_info_len;
                    scr_index += curr_grp_info_len;
                    grp_info_len -= curr_grp_info_len;
                    curr_grp_info_len = MIN(P2P_MAX_IE_BODY_SIZE,grp_info_len);

                    /* Update the P2P IE length field */
                    data[index + 1] = P2P_OUI_LEN + curr_grp_info_len;

                    /* Add the P2P IE header */
                    index += add_p2p_ie_hdr(&data[index]);
                }while(grp_info_len > P2P_MAX_IE_BODY_SIZE);
            }
            memcpy(&data[index], &scrch_ptr[scr_index], grp_info_len);
            index += grp_info_len;
        }
    }
    else
    {
        /* Raise system error */
    }

    total_len = index - start_index;

    /* Add the length element of the first IE */
    data[start_index + 1] = P2P_OUI_LEN + MIN(P2P_MAX_IE_BODY_SIZE,
                                              (total_len - P2P_IE_HDR_LEN));

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(scratch_mem_idx);

    return total_len;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_inv_rsp                                         */
/*                                                                           */
/*  Description   : This function prepares the invitation response frame     */
/*                  and calls the function to transmit it                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the invitation request frame  */
/*                  2) Status code to be added                               */
/*                  3) Pointer to the persistent credential list             */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the invitation response frame,adds*/
/*                  the header and the required P2P attributes and calls the */
/*                  function to transmit the frame                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_inv_rsp(UWORD8 *msa, P2P_STATUS_CODE_T status,
                      p2p_persist_list_t *persist_list)
{
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;
    UWORD8  dia_token     = 0;
    UWORD8  *frm_ptr      = 0;
    UWORD8  go_config_to  = 0;
    UWORD8  cl_config_to  = get_config_timeout();

	TROUT_FUNC_ENTER;
    TROUT_DBG4("P2P: send a P2P frame...");
    
    /* Allocate memory to prepare the invitation resp frame */
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
    	TROUT_FUNC_EXIT;
        return;
    }

    /* Call the function to add the management frame MAC header */
    add_p2p_mgmt_frame_hdr(frm_ptr, get_SA_ptr(msa));

    /* Extract the dialog token from the invitation request frame */
    dia_token = msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF] ;

    /* Add the  action frame header */
    index = add_p2p_pub_act_hdr(frm_ptr, dia_token, P2P_INV_RSP);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /* Add P2P IE */
    index += add_status_attr(frm_ptr, index, (P2P_STATUS_CODE_T)status);

    if(P2P_STAT_SUCCESS == status)
    {
        /* Check if the persistent credentials are available */
        if(NULL != persist_list)
        {
            /* Check if our device was the GO in the persistent grp */
            if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
            {
                go_config_to = cl_config_to;
                cl_config_to = 0;
                index += add_p2p_oper_chan_attr(frm_ptr, index,
                                                mget_p2p_oper_chan());
                index += add_grp_bssid_attr(frm_ptr, index,
                                            get_p2p_if_address());
            }
        }
        index += add_config_timout_attr(frm_ptr, index, go_config_to,
                                        cl_config_to);
        index += add_p2p_chan_list_attr(frm_ptr, index);
    }

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_inv_req                                         */
/*                                                                           */
/*  Description   : This function prepares the invitation request frame      */
/*                  and calls the function to transmit it                    */
/*                                                                           */
/*  Inputs        : 1) Is the invitation for a persistent grp formation      */
/*                  2) GO config timeout                                     */
/*                  3) client config timeout                                 */
/*                  4) SSID of the group the device wants to be a part of    */
/*                  5) Device id of the GO                                   */
/*                  6) Operating channel                                     */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                  g_p2p_dialog_token                                       */
/*                                                                           */
/*  Processing    : This function prepares the invitation request frame, adds*/
/*                  the header and the required P2P attributes and calls the */
/*                  function to transmit the frame                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_inv_req(UWORD8 is_persit, UWORD8 go_config, UWORD8 cl_config,
                      UWORD8 *ssid, UWORD8 *go_dev_addr, UWORD8 oper_chan)
{
    UWORD8  *frm_ptr      = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;

	TROUT_FUNC_ENTER;
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    TROUT_DBG4("P2P: send a P2P frame...");
    
    add_p2p_mgmt_frame_hdr(frm_ptr, mget_p2p_trgt_dev_id());
    g_p2p_dialog_token = get_random_byte();   /* rand dialog token */
    index = add_p2p_pub_act_hdr(frm_ptr, g_p2p_dialog_token, P2P_INV_REQ);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following P2P attributes are added as per the P2P v1.1 spec       */
    /* Table 54—P2P attributes in the GO Negotiation Request frame           */
    /* - Configuration Timeout          shall be present                     */
    /* - Invitation Flags               shall be present                     */
    /* - Operating Channel              shall be present                     */
    /* - P2P Group BSSID                Shall be present if not a persistent */
    /*                                  invitation request                   */
    /* - Channel List                   shall be present                     */
    /* - P2P Group ID                   Shall be present                     */
    /* - P2P Device Info                shall be present                     */
    /*************************************************************************/

    index += add_config_timout_attr(frm_ptr, index, go_config, cl_config);
    index += add_p2p_invit_flag_attr(frm_ptr, index, is_persit);
    index += add_p2p_oper_chan_attr(frm_ptr, index, oper_chan);
    if(1 == is_persit)
    {
        if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
        {
		    index += add_grp_bssid_attr(frm_ptr, index, get_p2p_if_address());
		}
    }
    else
    {
        index += add_grp_bssid_attr(frm_ptr, index, mget_bssid());
    }

    index += add_p2p_chan_list_attr(frm_ptr, index);
    index += add_p2p_grp_id_attr(frm_ptr, index, go_dev_addr, ssid);
    index += add_p2p_device_info_attr(frm_ptr, index);

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_prov_disc_resp                                      */
/*                                                                           */
/*  Description   : This function prepares the provision discovery response  */
/*                  and calls the function to transmit it                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address                      */
/*                  2) Config method to be added in WSC IE                   */
/*                  3) Address of the device that sent the provision         */
/*                     discovery request frame                               */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the provision discovery response  */
/*                  and adds the various IE. It then calls the function to   */
/*                  transmit it                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_prov_disc_resp(UWORD8 *msa, UWORD16 config_method, UWORD8 *sa)
{
    UWORD8  dia_tok    = 0;
    UWORD8  index      = 0;
    UWORD8  *frm_ptr   = 0;

	TROUT_FUNC_ENTER;
    /* Allocate memory for provision disc response frame */
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                 MANAGEMENT_FRAME_LEN);

    /* If allocation fails then return */
    if(frm_ptr == NULL)
    {
        return;
    }

    TROUT_DBG4("P2P: send a P2P frame...");

    /* Call the function to add the managemnet frame MAC header */
    add_p2p_mgmt_frame_hdr(frm_ptr, sa);

    /* Extract the dialog token from the frame */
    dia_tok = msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF];

    index = add_p2p_pub_act_hdr(frm_ptr, dia_tok, P2P_PROV_DISC_RSP);

    /* Add the WSC IE */
    index += wps_add_config_method_ie(frm_ptr, index, config_method);

    /* Transmit the presence response frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_get_operating_class                                  */
/*                                                                           */
/*  Description   : This function returns the Operating class for the channel*/
/*                                                                           */
/*  Inputs        : 1) Channel number                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the channel index for the channel     */
/*                  number and returns the corresponding Operating class     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Operating Class                                 */
/*  Issues        : None                                                     */
/*****************************************************************************/
UWORD16 p2p_get_operating_class(UWORD8 ch_num)
{
	/* MD-TBD: To be updated correctly using the Regulatory Domain Info */
#ifdef P2P_NO_COUNTRY
    return  P2P_OPERATING_CLASS1;
#endif /* P2P_NO_COUNTRY */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_ie_probe_req                                     */
/*                                                                           */
/*  Description   : This function adds the P2P IE to the probe request  frame*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the probe response frame      */
/*                  2) Index to the start of the P2P IE                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the P2P IE along with the various     */
/*                  attributes required for the probe request frame and      */
/*                  returns the total length of the P2P IE added             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Total lenght of the P2P IE added                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 add_p2p_ie_probe_req(UWORD8 *data, UWORD16 index)
{
    UWORD16 start_index       = 0;
    UWORD16 total_len         = 0;

    start_index = index;

    /* Add P2P IE header */
    index += add_p2p_ie_hdr(&data[index]);

    /* Add the P2P capability attribute                    */
    /* the Group Capability Bitmap field shall be Reserved */
    index += add_p2p_capability_attr(data, index, get_p2p_dev_cap(), 0);

    /* Add P2P device ID attribute */
    /* The device ID shall be of the target device */
    if(BFALSE == is_mac_addr_null(mget_p2p_trgt_dev_id()))
    {
        index += add_p2p_device_id_attr(data, index, mget_p2p_trgt_dev_id());
    }

    /* Add listen channel attribute */
    index += add_listen_chan_attr(data, index);

    /* Add operating channel attribute */
    index += add_p2p_oper_chan_attr(data, index, mget_p2p_oper_chan());

    total_len = index - start_index;

    /* Add the length element of the first IE */
    data[start_index + 1] = P2P_OUI_LEN + (total_len - P2P_IE_HDR_LEN);

    return total_len;
}
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_P2P */
