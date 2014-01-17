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
/*  File Name         : p2p_wlan_rx_mgmt_ap.c                                */
/*                                                                           */
/*  Description       : This file contains the functions to handle the       */
/*                      received WLAN management frames                      */
/*                                                                           */
/*  List of Functions : p2p_process_dd_req                                   */
/*                      ap_enabled_rx_p2p_gen_action                         */
/*                      ap_enabled_rx_p2p_pub_action                         */
/*                      ap_enabled_rx_p2p_action                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_P2P

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "frame.h"
#include "frame_p2p.h"
#include "ap_mgmt_p2p.h"
#include "ap_frame_p2p.h"
#include "management_ap.h"
#include "qmu_if.h"


/*****************************************************************************/
/*                                                                           */
/*  Function Name :  p2p_process_dd_req                                      */
/*                                                                           */
/*  Description   :  This function processes the DD request frame            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming DD request frame              */
/*                  2) Length of the received frame                          */
/*                  3) Pointer to the interface address of the requested P2P */
/*                     device                                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the address of the target device  */
/*                  from the DD request frame. It then checks if the device  */
/*                  is associated with the GO and supports device discovery. */
/*                  It returns the appropriate status code based on the      */
/*                  checks                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : P2P_STATUS_CODE_T;                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
P2P_STATUS_CODE_T p2p_process_dd_req(UWORD8 *msa, UWORD16 rx_len,
                                     UWORD8* if_addr)
{

    UWORD8   trgt_p2p_dev_id[6]  = {0};
    UWORD8*  attr_start_ptr      = 0;
    UWORD8*  attr_ptr            = 0;
    UWORD16  ie_len              = 0;
    UWORD16  attr_len            = 0;
    asoc_entry_t     *ae         = 0;
    P2P_STATUS_CODE_T status     = P2P_STAT_INVAL_PARAM;


    /* Get the pointer to the start of the attribute list */
    /* Note: a scratch memory need not be allocated because the frame is     */
    /* expected to have only one P2P IE                                      */
    attr_start_ptr = get_p2p_attributes(msa, (P2P_PUB_ACT_TAG_PARAM_OFF +
                                           MAC_HDR_LEN), rx_len, &ie_len);

    if(attr_start_ptr != NULL)
    {
        /* Get the pointer to the P2P Device ID attribute */
        attr_ptr = p2p_get_attr(P2P_DEVICE_ID, attr_start_ptr, ie_len,
                                 &attr_len);

        if(attr_ptr != NULL)
        {
            /* Extract the P2P Device Address field from the P2P Device ID   */
            /* attribute                                                     */
            mac_addr_cpy(trgt_p2p_dev_id, attr_ptr);
            ae = (asoc_entry_t*)find_p2p_client_entry(trgt_p2p_dev_id,
                                                      if_addr);
            if((NULL != ae) && (ASOC == ae->state))
            {
                /* Check if the associated client supports P2P Client        */
                /* discoverability                                           */
                if(CHECK_BIT(ae->p2p_client_dscr.dev_cap_bitmap,
                             P2P_CLIENT_DISC))
                {
                    status = P2P_STAT_SUCCESS;
                }
                else
                {
                    status = P2P_STAT_UNABLE_ACCO_REQ;
                }
            }
            else
            {
                status = P2P_STAT_INCOMP_PARAM;
            }
        }

        /* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, attr_start_ptr);
    }

    return status;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_p2p_gen_action                             */
/*                                                                           */
/*  Description   : This function checks the subtype of the incoming message */
/*                  and does appropriate processing                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message frame                 */
/*                  2) Length of the received message                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the incoming P2P general action  */
/*                  frame is sent by an associated client and based on the   */
/*                  subtype of the incoming P2P it calls the appropriate func*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void ap_enabled_rx_p2p_gen_action(UWORD8* msa, UWORD16 rx_len)
{
    UWORD8 sub_type    = 0;
    UWORD8 *data       = msa + MAC_HDR_LEN;
    UWORD8 sa[6]       = {0};
    asoc_entry_t *ae   = 0;

    /* Get the source address of the action frame */
    get_SA(msa,sa);

    /* Check if the STA is associated with the GO */
    ae   = (asoc_entry_t*)find_entry(sa);

    /* If the client is associated then process the frame according to the   */
    /* subtype                                                               */
    if(ae != NULL)
    {
        sub_type = data[P2P_GEN_ACT_OUI_SUBTYPE_OFF];

        switch(sub_type)
        {
            case P2P_PRESENCE_REQ:
            {
                send_p2p_presence_resp(msa, sa);
            }
            break;

            default :
            {
                /* do nothing */
            }

        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_p2p_pub_action                             */
/*                                                                           */
/*  Description   : This function checks the subtype of the incoming message */
/*                  and does appropriate processing                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message frame                 */
/*                  2) Length of the message received                        */
/*                  3) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the subtype of the incoming P2P     */
/*                  public action frame and calls the appropriate functions  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void ap_enabled_rx_p2p_pub_action(UWORD8* msa, UWORD16 rx_len,
                                  mac_struct_t *mac)
{
    UWORD8  sub_type = 0;
    UWORD8  sa[6]    = {0};
    UWORD8* data     = msa + MAC_HDR_LEN;

    sub_type = data[P2P_PUB_ACT_OUI_SUBTYPE_OFF];
    /* Get the source address and dialog token of the DD req frame */
    get_SA(msa, sa);

    switch(sub_type)
    {
        case P2P_INV_REQ:
        {
            p2p_send_inv_rsp(msa, P2P_STAT_INFO_UNAVAIL, NULL);
            send_host_p2p_req(msa, rx_len);
        }
        break;

        case P2P_DEV_DISC_REQ:
        {
            UWORD8   dialog_token    = 0;
            UWORD8   if_addr[6]      = {0};
            P2P_STATUS_CODE_T status = P2P_STAT_UNABLE_ACCO_REQ;

            dialog_token = msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF];

            /* Process the DD request frame */
                status = p2p_process_dd_req(msa, rx_len, if_addr);

            if(P2P_STAT_SUCCESS == status)
            {
                send_p2p_go_disc_req(sa, dialog_token, if_addr);
            }
            else
            {
                /* Send the device discoverability response with appropriate */
                /* FAILURE status code                                       */
                send_p2p_dd_resp(sa, dialog_token, status);
            }
        }
        break;
        case P2P_INV_RSP:
        {
            if(WAIT_INVITE_RSP == g_p2p_invit)
			{
				/* Check if the source address of the invitation request frame */
				/* matches with the target device address                      */
				if(BTRUE == mac_addr_cmp(get_TA_ptr(msa), mget_p2p_trgt_dev_id()))
				{
					/* Cancel the management timer */
					cancel_mgmt_timeout_timer();

					/* Cal the function to end the invitation prpcess */
					p2p_end_ivitation_process();

					send_host_p2p_req(msa, rx_len);
				}
				else
				{
					/* Nothing to do */
				}

	        }
        }
        break;
        case P2P_PROV_DISC_REQ:
        {
            p2p_handle_prov_disc_req(msa, sa, rx_len);
        }
        break;
        default :
        {
            /* Do nothing */
        }

    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_p2p_action                                 */
/*                                                                           */
/*  Description   : This function handles the incoming action frame as       */
/*                  appropriate in the ENABLED state.                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
/*                  2) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The incoming frame type is checked and appropriate       */
/*                  processing is done.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_enabled_rx_p2p_action(UWORD8 *msg, mac_struct_t *mac)
{
    UWORD8       *msa     = 0;
    UWORD8       *data    = 0;
    UWORD16      rx_len   = 0;
    wlan_rx_t    *wlan_rx = (wlan_rx_t*)msg;


    /* Pointer to the start of the frame */
    msa  = wlan_rx->msa;

    /* Pointer to the start of the data portion in the frame */
    data = msa + MAC_HDR_LEN;

    rx_len = wlan_rx->rx_len;

    /* Check if it is P2P public action frame or P2P general action frame */
    if(BTRUE == is_p2p_pub_action_frame(data))
    {
        ap_enabled_rx_p2p_pub_action(msa, rx_len, mac);
    }
    else if(BTRUE == is_p2p_gen_action_frame(data))
    {
        ap_enabled_rx_p2p_gen_action(msa, rx_len);
    }
}

#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */
