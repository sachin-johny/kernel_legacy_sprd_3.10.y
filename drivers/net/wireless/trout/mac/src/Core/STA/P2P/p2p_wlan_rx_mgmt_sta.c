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
/*  File Name         : p2p_wlan_rx_mgmt_sta.c                               */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to SME functions specific to P2P STA mode.   */
/*                                                                           */
/*  List of Functions : process_p2p_go_neg_req                               */
/*                      process_p2p_go_neg_rsp                               */
/*                      process_p2p_go_neg_cnf                               */
/*                      process_p2p_dev_disc_rsp                             */
/*                      process_p2p_inv_rsp                                  */
/*                      process_p2p_prov_disc_rsp                            */
/*                      p2p_wait_scan_handle_action_rx                       */
/*                      p2p_wait_join_handle_action_rx                       */
/*                      p2p_wait_scan_process_go_neg_req                     */
/*                      p2p_wait_scan_process_pers_inv_req                   */
/*                      p2p_wait_scan_process_inv_req                        */
/*                      p2p_wait_join_process_go_neg_req                     */
/*                      p2p_wait_join_process_go_neg_rsp                     */
/*                      p2p_wait_join_process_go_neg_cnf                     */
/*                      p2p_wait_join_process_dev_disc_rsp                   */
/*                      p2p_wait_join_process_inv_rsp                        */
/*                      p2p_wait_join_process_prov_disc_rsp                  */
/*                      p2p_wait_join_process_inv_req                        */
/*                      sta_enabled_rx_p2p_pub_action                        */
/*                      sta_enabled_rx_p2p_gen_action                        */
/*                      sta_enabled_rx_p2p_action                            */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P
// 20120709 caisf mod, merged ittiam mac v1.2 code

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "iconfig.h"
#include "frame_p2p.h"
#include "p2p_wlan_rx_mgmt_sta.h"
#include "mgmt_p2p.h"
#include "mac_init.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
UWORD8 g_p2p_go_dev_id[MAC_ADDRESS_LEN]         = {0};
BOOL_T g_waiting_for_go_neg_req = BFALSE;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void p2p_send_go_neg_cnf(UWORD8 status);

/*****************************************************************************/
/* Function Declarations                                                     */
/*****************************************************************************/
void p2p_wait_join_process_inv_req(UWORD8 *msa, UWORD16 rx_len);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_go_neg_req                                   */
/*                                                                           */
/*  Description   : This function processes the GO NEG REQ frame             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_GO_role                                            */
/*                                                                           */
/*  Processing    : This function processes the GO NEG REQ frame. It does the*/
/*                  various checks and set the status to be sent in the GO   */
/*                  NEG RESP frame accordingly                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
P2P_STATUS_CODE_T process_p2p_go_neg_req(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 ie_len           = 0;    /* combined Length of all attributes    */
    UWORD16 attr_len         = 0;    /* scratch variable for attribute length*/
    UWORD8  *attr_start_ptr  = NULL; /* Start of combined attributes         */
    UWORD8  *attr_ptr        = NULL; /* scratch variable for the current     */
                                     /* attribute                            */
    P2P_STATUS_CODE_T status = P2P_STAT_SUCCESS;

    /* Store the dialog token for future use */
    g_p2p_dialog_token = msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF];

    /* Get the pointer to the combined P2P attributes */
    attr_start_ptr     = get_p2p_attributes(msa, MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF, rx_len,
                                            &ie_len);
    if(NULL == attr_start_ptr)
        status = P2P_STAT_INVAL_PARAM;

    /*************************************************************************/
    /* The following P2P attributes shall be present as per the P2P v1.1 spec*/
    /* Table 49—P2P attributes in the GO Negotiation Request frame           */
    /* - P2P Capability                 shall be present                     */
    /* - Group Owner Intent             shall be present                     */
    /* - Configuration Timeout          shall be present                     */
    /* - Listen Channel                 shall be present                     */
    /* - Extended Listen Timing         may   be present                     */
    /* - Intended P2P Interface Address shall be present                     */
    /* - Channel List                   shall be present                     */
    /* - P2P Device Info                shall be present                     */
    /* - Operating Channel              shall be present                     */
    /*************************************************************************/

    if(P2P_STAT_SUCCESS == status)
    {   /* Process GROUP_OWNER_INTENT */
        attr_ptr = p2p_get_attr(GROUP_OWNER_INTENT, attr_start_ptr, ie_len,
                                &attr_len);
        if(NULL != attr_ptr)
        {
            UWORD8 go_intent1 = mget_p2p_GO_intent_val();
            UWORD8 go_intent2 = (*attr_ptr) >> 1;
            if(go_intent1 == go_intent2)
            {
                if(go_intent1 < 15)
                {
                    /* Check the tie breaker bit                             */
                    if(CHECK_BIT((*attr_ptr), BIT0))
                    {
                        g_p2p_GO_role = BFALSE;
                    }
                    else
                    {
                        g_p2p_GO_role = BTRUE;
                    }
                }
                else
                {
                    status = P2P_STAT_GO_INTENT_15;
                }
            }
            else
            {
                if(go_intent1 > go_intent2)
                    g_p2p_GO_role = BTRUE;
                else
                    g_p2p_GO_role = BFALSE;
            }

            /* As per P2P spec 1.1 p40-ln4                                   */
            /* "The Tie breaker bit in a GO Negotiation Response frame shall */
            /* be toggled from the corresponding GO Negotiation Request      */
            /* frame "                                                       */
            /* NOTE: This is currently done for GO neg requests that are     */
            /* processed not for declined GO neg req                         */
            g_GO_tie_breaker = (!CHECK_BIT((*attr_ptr), BIT0));

        }
        else
        {
            /* If no attribute is found then set the status to */
            /* "Fail; invalid parameters"                      */
            status = P2P_STAT_INVAL_PARAM;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {   /* Process P2P_CAPABILITY */
        attr_ptr = p2p_get_attr(P2P_CAPABILITY, attr_start_ptr, ie_len,
                                &attr_len);

        if(NULL != attr_ptr)
        {
            /* Check the capability of the device */
            if((BFALSE == g_p2p_GO_role) && (CHECK_BIT(attr_ptr[1], BIT1) &&
               (BFALSE == mget_p2p_persist_grp())))
            {
                status = P2P_STAT_INCOMP_PARAM;
            }
        }
        else
        {
            /* If no attribute is found then set the status to */
            /* "Fail; invalid parameters"                      */
            status = P2P_STAT_INVAL_PARAM;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {   /* Process P2P_CHANNEL_LIST                                 */
        /* Verify the channel list for match                        */
        /* NOTE: This should be done at the end of all other checks */
        attr_ptr = p2p_get_attr(P2P_CHANNEL_LIST, attr_start_ptr, ie_len,
                                &attr_len);

        if(NULL != attr_ptr)
        {
            /* MD-TBD: Check the channel list  */
            if(0)
            {
                status = P2P_STAT_INCOMP_PARAM;
            }
        }
        else
        {
            /* If no attribute is found then set the status to */
            /* "Fail; invalid parameters"                      */
            status = P2P_STAT_INVAL_PARAM;
        }
    }

	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);
    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_go_neg_rsp                                   */
/*                                                                           */
/*  Description   : This function processes the GO NEG RESP frame            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       :  g_p2p_join_req                                          */
/*                   g_waiting_for_go_neg_req                                */
/*                   g_p2p_GO_role                                           */
/*                                                                           */
/*  Processing    : This function processes the GO NEG RESP frame. The       */
/*                  various checks are done and it is detemined if the GO NEG*/
/*                  CONF frame has to be sent.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : SWORD8; the status                                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
SWORD8 process_p2p_go_neg_rsp(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 ie_len          = 0;    /* combined Length of all attributes     */
    UWORD16 attr_len        = 0;    /* scratch variable for attribute length */
    UWORD8 *attr_start_ptr  = NULL; /* Start of combined attributes          */
    UWORD8 *attr_ptr        = NULL; /* scratch variable for the current
                                       attribute                             */
    SWORD8 status            = P2P_STAT_SUCCESS;
    UWORD8 freq             = 0;

	TROUT_FUNC_ENTER;
    /* Match the source addr and dialog token with saved data */
    if((BFALSE ==
            mac_addr_cmp(g_p2p_join_req.dev_dscr.dev_addr, get_SA_ptr(msa))) ||
        (g_p2p_dialog_token !=
            msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF]))
    {
        status = P2P_STAT_INVAL_PARAM;
    }

    /* Get the pointer to the combined P2P attributes */
    if(P2P_STAT_SUCCESS == status)
    {
        /* Note: a scratch memory need not be allocated because the */
        /* frame is expected to have only one P2P IE  */
        attr_start_ptr = get_p2p_attributes(msa, (MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF),
                                            rx_len, &ie_len);
        if(NULL == attr_start_ptr)
            status = P2P_STAT_INVAL_PARAM;
    }

    /*************************************************************************/
    /* The following attributes would be present as per v1.1 fo P2P Spec     */
    /* Table 51—P2P attributes in the GO Negotiation Response frame          */
    /* -Status                         shall be present                      */
    /* -P2P Capability                 shall be present                      */
    /* -Group Owner Intent             shall be present                      */
    /* -Configuration Timeout          shall be present                      */
    /* -Operating Channel              may   be present                      */
    /* -Intended P2P Interface Address shall be present                      */
    /* -Channel List                   shall be present                      */
    /* -P2P Device Info                shall be present                      */
    /* -P2P Group ID                   shall be present                      */
    /*                                 if the P2P Device sending the GO      */
    /*                                 Negotiation Response frame will become*/
    /*                                 P2P Group Owner following Group Owner */
    /*                                 Negotiation                           */
    /*************************************************************************/

    if(P2P_STAT_SUCCESS == status)
    {
        attr_ptr = p2p_get_attr(P2P_STATUS, attr_start_ptr, ie_len, &attr_len);

        if(NULL == attr_ptr)
        {
            status = P2P_STAT_INVAL_PARAM;
        }
        else if(P2P_STAT_SUCCESS != *attr_ptr)
        {
			if((P2P_STAT_INFO_UNAVAIL == *attr_ptr) &&
			   (BFALSE == g_waiting_for_go_neg_req))
			{
				/* Start timeout timer for the Responder to send a request */
				start_mgmt_timeout_timer(P2P_WAIT_WPS_START_TO);

				/* Set a flag to indicate that the device is expecting a GO  */
				/* Negotiation Request from the responder.                   */
				g_waiting_for_go_neg_req = BTRUE;

				TROUT_DBG4("Received GO Negotiation Response: INFO UNAVAIL\n\r");
				TROUT_DBG4("Waiting for GO Negotiation Request...\n\r");
			}

            status = -1;
        }
    }

    if(P2P_STAT_SUCCESS == status )
    {
        /* Process GROUP_OWNER_INTENT */
        attr_ptr = p2p_get_attr(GROUP_OWNER_INTENT, attr_start_ptr, ie_len,
                            &attr_len);
        if(NULL != attr_ptr)
        {
            UWORD8 go_intent1 = mget_p2p_GO_intent_val();
            UWORD8 go_intent2 = (*attr_ptr) >> 1;
            if(go_intent1 == go_intent2)
            {
                if(go_intent1 < 15)
                {
                    /* Check the tie breaker bit                             */
                    if(CHECK_BIT((*attr_ptr), BIT0))
            		{
                        g_p2p_GO_role = BFALSE;
            		}
            		else
            		{
                        g_p2p_GO_role = BTRUE;
            		}
		        }
		        else
            	{
                    status = P2P_STAT_GO_INTENT_15;
                }
            }
            else
            {
                if(go_intent1 > go_intent2)
                    g_p2p_GO_role = BTRUE;
                else
                    g_p2p_GO_role = BFALSE;
            }
        }
        else
        {
            /* If no attribute is found then set the status to */
            /* "Fail; invalid parameters"                      */
            status = P2P_STAT_INVAL_PARAM;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {
        if(BTRUE == g_p2p_GO_role)
        {
            attr_ptr = p2p_get_attr(P2P_CHANNEL_LIST, attr_start_ptr, ie_len,
                                    &attr_len);
            if(NULL == attr_ptr)
            {
                status = P2P_STAT_INVAL_PARAM;
            }
            else
            {
                /* MD-TBD: Create a common minimum channel list */
            }
        }
        else
        {

            /* Adapt the channel list sent in the GO neg resp, the matching  */
            /* is supposed to be done by the other device                    */
            /* "The channels indicated in the Channel List shall only include*/
            /* channels from the Channel List attribute in the GO Negotiation*/
            /* Request frame"                                                */
            attr_ptr = p2p_get_attr(P2P_CHANNEL_LIST, attr_start_ptr, ie_len,
                                    &attr_len);

            if(NULL == attr_ptr)
            {
                status = P2P_STAT_INVAL_PARAM;
            }
            else
            {
                /* MD-TBD : Update the channel list */
            }

            if(P2P_STAT_SUCCESS == status)
            {
                /* Adapt the operating channel */
                attr_ptr = p2p_get_attr(P2P_OPERATING_CHANNEL, attr_start_ptr,
                                        ie_len, &attr_len);

                if(NULL == attr_ptr)
                {
                    status = P2P_STAT_INVAL_PARAM;
                }
                else
                {
                    freq = get_current_start_freq();
                    /* Extract the operating channle from the attribute and  */
                    /* update the operating channel MIB for the same         */
                    mset_p2p_oper_chan(get_ch_idx_from_num(freq, attr_ptr[4]));
                }
            }

            if(P2P_STAT_SUCCESS == status )
            {
                /* Adapt the INTENDED_P2P_IF_ADDR as the dessired BSSID */
                attr_ptr = p2p_get_attr(INTENDED_P2P_IF_ADDR, attr_start_ptr,
                                        ie_len, &attr_len);
                if(NULL == attr_ptr)
                {
                    status = P2P_STAT_INVAL_PARAM;
                }
            }
        }
    }

	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);

	TROUT_FUNC_EXIT;
    return status;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_go_neg_cnf                                   */
/*                                                                           */
/*  Description   : This function processes the GO NEG CONF frame            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                  g_p2p_GO_role                                            */
/*                                                                           */
/*  Processing    : This function processes the GO NEG CONF frame and based  */
/*                  on the status and role the channel and SSID and adopted  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : STATUS_T; FAILURE                                        */
/*                            SUCCESS                                        */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
STATUS_T process_p2p_go_neg_cnf(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 ie_len          = 0;    /* combined Length of all attributes     */
    UWORD16 attr_len        = 0;    /* scratch variable for attribute length */
    UWORD8 *attr_start_ptr  = NULL; /* Start of combined attributes          */
    UWORD8 *attr_ptr        = NULL; /* scratch variable for the current
                                       attribute                             */
    STATUS_T status         = SUCCESS;

	TROUT_FUNC_ENTER;
    /* Match the source addr and dialog token with saved data */
    if((BFALSE ==
            mac_addr_cmp(g_p2p_join_req.dev_dscr.dev_addr, get_SA_ptr(msa))) ||
        (g_p2p_dialog_token !=
            msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF]))
    {
        status = FAILURE;
    }

    /* Get the pointer to the combined P2P attributes */
    if(SUCCESS == status)
    {
        attr_start_ptr = get_p2p_attributes(msa, MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF, rx_len,
                                            &ie_len);
        if(NULL == attr_start_ptr)
            status = FAILURE;
    }

    /*************************************************************************/
    /* The following attributes would be present as per v1.1 fo P2P Spec     */
    /* Table 53—P2P attributes in the GO Negotiation Confirmation frame      */
    /* -Status                         shall be present                      */
    /* -P2P Capability                 shall be present                      */
    /* -Operating Channel              shall be present                      */
    /* -Channel List                   shall be present                      */
    /* -P2P Group ID                   shall be present                      */
    /*                                 if the P2P Device sending the GO      */
    /*                                 Negotiation Confirmation frame will   */
    /*                                 become P2P Group Owner following Group*/
    /*                                 Owner Negotiation.                    */
    /*************************************************************************/

    if(SUCCESS == status)
    {
        attr_ptr = p2p_get_attr(P2P_STATUS, attr_start_ptr, ie_len, &attr_len);

        if(NULL == attr_ptr)
        {
            status = FAILURE;
        }
        else if(0 != *attr_ptr)
        {
            status = FAILURE;
        }
    }

    if(SUCCESS == status )
    {   /* A succesfull GO neg conf is received. Check if we need to become  */
        /* the Group Owner. If a P2P_GROUP_ID attribute is not sent by the   */
        /* other device then we need to become the GO                        */
        attr_ptr = p2p_get_attr(P2P_GROUP_ID, attr_start_ptr, ie_len,
                            &attr_len);

        if(NULL == attr_ptr)
        {   /* We need to become the GO */
            g_p2p_GO_role = BTRUE;
            attr_ptr = p2p_get_attr(P2P_CHANNEL_LIST, attr_start_ptr, ie_len,
                                    &attr_len);
            if(NULL == attr_ptr)
            {
                status = FAILURE;
            }
            else
            {
                /* MD-TBD: Create a common minimum channel list */
            }
        }
        else
        {   /* The other device becomes the GO */
            g_p2p_GO_role     = BFALSE;

            /* Adapt the channel list sent in the GO neg resp, the matching  */
            /* is supposed to be done by the other device                    */
            /* "The channels indicated in the Channel List shall only include*/
            /* channels from the Channel List attribute in the GO Negotiation*/
            /* Request frame"                                                */
            attr_ptr = p2p_get_attr(P2P_CHANNEL_LIST, attr_start_ptr, ie_len,
                                    &attr_len);

            if(NULL == attr_ptr)
            {
                status = FAILURE;
            }
            else
            {
                /* MD-TBD : Update the channel list */
            }

            if(SUCCESS == status )
            {
                /* Adapt the operating channel */
                attr_ptr = p2p_get_attr(P2P_OPERATING_CHANNEL, attr_start_ptr,
                                        ie_len, &attr_len);

                if(NULL == attr_ptr)
                {
                    status = FAILURE;
                }
                else
                {
                    /* Extract the operating channle from the attribute and  */
                    /* update the operating channle MIB for the same         */
                    mset_p2p_oper_chan(attr_ptr[4]);
                }
            }

            if(SUCCESS == status )
            {   /* Adapt the P2P_GROUP_ID as the dessired SSID */
                WORD8 p2p_ssid[MAX_SSID_LEN] = {0,};
                attr_ptr = p2p_get_attr(P2P_GROUP_ID, attr_start_ptr,
                                        ie_len, &attr_len);
                if(NULL == attr_ptr)
                {
                    status = FAILURE;
                }
                else
                {
                    memcpy(p2p_ssid, attr_ptr+6, attr_len-6);
                    set_DesiredSSID(p2p_ssid);
                }
            }
        }
    }

	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);

	TROUT_FUNC_EXIT;
    return status;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_dev_disc_rsp                                 */
/*                                                                           */
/*  Description   : This function processes the DD response frame            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       :  g_p2p_join_req                                          */
/*                                                                           */
/*  Processing    : This function processes the DD request frame and returns */
/*                  the appropriate status                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : STATUS_T; SUCCESS                                        */
/*                            FAILURE                                        */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
STATUS_T process_p2p_dev_disc_rsp(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 ie_len          = 0;    /* combined Length of all attributes     */
    UWORD16 attr_len        = 0;    /* scratch variable for attribute length */
    UWORD8 *attr_start_ptr  = NULL; /* Start of combined attributes          */
    UWORD8 *attr_ptr        = NULL; /* scratch variable for the current
                                       attribute                             */
    STATUS_T status         = SUCCESS;
    UWORD8 *grp_bssid       = g_p2p_join_req.dev_dscr.grp_bssid;

	TROUT_FUNC_ENTER;
	
    /* Match the source addr and dialog token with saved data */
    if (g_p2p_dialog_token !=
         msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF])
    {
        status = FAILURE;
    }

    if(SUCCESS == status)
    {
         if(BTRUE == mac_addr_cmp(grp_bssid, get_SA_ptr(msa)))
        {
            /* Source address matched - no status change */
        }
        else
        {
            /* Try to check with the corresponding GO device address */
            UWORD8 *go_dev_addr = p2p_get_go_dev_address(grp_bssid);

            if((NULL == go_dev_addr) || /* No GO address found */
               (BFALSE == mac_addr_cmp(go_dev_addr, get_SA_ptr(msa))))
		    {
		        status = FAILURE;
		    }
        }
    }

    /* Get the pointer to the combined P2P attributes */
    if(SUCCESS == status)
    {
        /* Note: a scratch memory need not be allocated because the */
        /* frame is expected to have only one P2P IE  */
        attr_start_ptr = get_p2p_attributes(msa, MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF, rx_len,
                                            &ie_len);
        if(NULL == attr_start_ptr)
            status = FAILURE;
    }

    if(SUCCESS == status)
    {
        attr_ptr = p2p_get_attr(P2P_STATUS, attr_start_ptr, ie_len, &attr_len);

        if(NULL == attr_ptr)
        {
            status = FAILURE;
        }
        else if(P2P_STAT_SUCCESS != *attr_ptr)
        {
            status = FAILURE;
        }
    }

	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);

	TROUT_FUNC_EXIT;
    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_inv_rsp                                      */
/*                                                                           */
/*  Description   : This function processes the P2P invitation response      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_dialog_token                                       */
/*                                                                           */
/*  Processing    : This function processes the P2P invitation response frame*/
/*                  and returns the status sent in the frame                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : P2P_STATUS_CODE_T; p2p status code                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
P2P_STATUS_CODE_T process_p2p_inv_rsp(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 ie_len          = 0;
    UWORD16 attr_len        = 0;
    UWORD8 * attr_start_ptr = 0;
    UWORD8 * attr_ptr       = 0;
    P2P_STATUS_CODE_T status  = P2P_STAT_INVAL_PARAM;

    /* Match the source addr and dialog token with saved data */
    if((BTRUE ==
            mac_addr_cmp(g_p2p_join_req.dev_dscr.dev_addr, get_SA_ptr(msa))) &&
        (g_p2p_dialog_token ==
            msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF]))
    {
        /* Note: the P2P IE will not need scratch memory */
        /* Get the pointer to the combined IE  */
        attr_start_ptr = get_p2p_attributes(msa, MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF,
                                            rx_len, &ie_len);
        if(NULL != attr_start_ptr)
        {
            /* Get the start pointer to p2P status attribute */
            attr_ptr = p2p_get_attr(P2P_STATUS, attr_start_ptr, ie_len,
                                    &attr_len);

            if(NULL != attr_ptr)
            {
                status = (P2P_STATUS_CODE_T)(*attr_ptr);
            }
        }
    }
    /* Free the local memory buffer allocated for P2P attributes */
	if(NULL != attr_start_ptr)
        mem_free(g_local_mem_handle, attr_start_ptr);
    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_p2p_prov_disc_rsp                                */
/*                                                                           */
/*  Description   : This function processes the provision discovery response */
/*                  frame                                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_dialog_token                                       */
/*                                                                           */
/*  Processing    : This function processes the provision discovery response */
/*                  frame and returns the appropriate status                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : STATUS_T; SUCCESS                                        */
/*                            FAILURE                                        */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
STATUS_T process_p2p_prov_disc_rsp(UWORD8 *msa, UWORD16 rx_len)
{
    STATUS_T status = SUCCESS;

    /* Match the source addr and dialog token with saved data */
    if((BFALSE ==
            mac_addr_cmp(g_p2p_join_req.dev_dscr.dev_addr, get_SA_ptr(msa))) ||
        (g_p2p_dialog_token !=
            msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF]) ||
        (0 == wps_get_config_method(msa, rx_len)))
    {
        status = FAILURE;
    }
    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_scan_handle_action_rx                           */
/*                                                                           */
/*  Description   : This function handles received P2P Action frames in the  */
/*                  WAIT SCAN state                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function handles received P2P Action frames in the  */
/*                  WAIT SCAN state. It checks the frame subtype and calls   */
/*                  the appropriate functions                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_scan_handle_action_rx(mac_struct_t *mac, UWORD8 *msa,
                                    UWORD16 rx_len)
{
    UWORD8 sa[6] = {0};
    UWORD8 *data = msa + MAC_HDR_LEN;

	TROUT_FUNC_ENTER;
    if(BFALSE == is_p2p_pub_action_frame(data))
    {
		TROUT_FUNC_EXIT;
        return;
	}
    switch(data[P2P_PUB_ACT_OUI_SUBTYPE_OFF])
    {
    case P2P_GO_NEG_REQ:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_scan_process_go_neg_req(msa, rx_len);
    }
    break;
    case P2P_INV_REQ:
    {
        p2p_wait_scan_process_inv_req(mac, msa, rx_len);
    }
    break;
    case P2P_INV_RSP:
    {
         p2p_handle_invit_rsp_sta(msa, rx_len, mac);

    }
    break;
    case P2P_PROV_DISC_REQ:
    {
        get_SA(msa, sa);
        p2p_handle_prov_disc_req(msa, sa, rx_len);
    }
    break;
    default:
    {
        /* No other P2P action frames need to be processed in this state. */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_handle_action_rx                           */
/*                                                                           */
/*  Description   : This function handles received P2P Action frames in the  */
/*                  WAIT JOIN state                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function handles received P2P Action frames in the  */
/*                  WAIT JOIN state. It checks for the frame subtype and     */
/*                  calls the appropriate functions                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_wait_join_handle_action_rx(mac_struct_t *mac, UWORD8 *msa,
                                    UWORD16 rx_len)
{
    UWORD8 *data = msa + MAC_HDR_LEN;

	TROUT_FUNC_ENTER;
    if(BFALSE == is_p2p_pub_action_frame(data))
    {
		TROUT_FUNC_EXIT;
        return;
    }    
    //TROUT_PRINT("P2P: get P2P action frame:%d ",data[P2P_PUB_ACT_OUI_SUBTYPE_OFF]);
    switch(data[P2P_PUB_ACT_OUI_SUBTYPE_OFF])
    {
    case P2P_GO_NEG_REQ:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_join_process_go_neg_req(mac, msa, rx_len);
    }
    break;
    case P2P_GO_NEG_RSP:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_join_process_go_neg_rsp(mac, msa, rx_len);
    }
    break;
    case P2P_GO_NEG_CNF:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_join_process_go_neg_cnf(mac, msa, rx_len);
    }
    break;
    case P2P_INV_REQ:
    {
        /* Call the function to process the invitation request */
        p2p_wait_join_process_inv_req(msa, rx_len);
    }
    break;
    case P2P_INV_RSP:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_join_process_inv_rsp(mac, msa, rx_len);
    }
    break;
    case P2P_DEV_DISC_RSP:
    {
        /* This is processed based on the P2P WAIT JOIN sub-states */
        p2p_wait_join_process_dev_disc_rsp(msa, rx_len);
    }
    break;
    case P2P_PROV_DISC_RSP:
    {
        p2p_wait_join_process_prov_disc_rsp(mac, msa, rx_len);
    }
    break;
    default:
    {
        /* No other P2P action frames need to be processed in this state. */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/***************************** WAIT SCAN *************************************/
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_scan_process_go_neg_req                         */
/*                                                                           */
/*  Description   : This function processes the GO NEG REQ frame in the wait */
/*                  scan state                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes the GO NEG REQ frame in the wait */
/*                  scan state                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_scan_process_go_neg_req(UWORD8 *msa, UWORD16 rx_len)
{
    /* Get the P2P device info */
    p2p_get_dev(msa, MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF, rx_len);

    /* Send a GO neg resp with status set to
    "Fail; information is currently unavailable" */
    p2p_send_go_neg_rsp(msa, P2P_STAT_INFO_UNAVAIL);
    send_host_p2p_req(msa, rx_len);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_scan_process_pers_inv_req                       */
/*                                                                           */
/*  Description   : This function processes the incoming p2p invitation      */
/*                  request frame to invoke a persistent group               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                  3) Pointer to the P2P Group ID attribute                 */
/*                  4) Length of the P2P Group ID attribute                  */
/*                                                                           */
/*  Globals       : g_persist_list                                           */
/*                                                                           */
/*  Processing    : This function checks if the TA of the incoming invitation*/
/*                  request frame matches with our target device ID. If the  */
/*                  match is correct then it checks for the GO device id and */
/*                  the SSID sent in the P2P Group ID attribute. If all the  */
/*                  checks are successful it sends the invitation reponse    */
/*                  frame with status as P2P_STAT_SUCCESS and calls the      */
/*                  function to configure the device with the stored         */
/*                  persistent credentials else it returns the appropriate   */
/*                  failure status                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : P2P_STATUS_CODE_T; status                                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
P2P_STATUS_CODE_T p2p_wait_scan_process_pers_inv_req(mac_struct_t *mac,
                                                     UWORD8 *msa,
                                                     UWORD8 *grp_id_ptr,
                                                     UWORD16 grp_id_attr_len)
{
    UWORD16 ssid_len             = 0;
    UWORD8 *go_dev_id            = NULL;
    UWORD8 *trgt_dev_id          = NULL;
    UWORD8  ssid[MAX_SSID_LEN]   = {0};
    P2P_STATUS_CODE_T status     = P2P_STAT_SUCCESS;

	TROUT_FUNC_ENTER;
    go_dev_id   = grp_id_ptr;
    trgt_dev_id = mget_p2p_trgt_dev_id();

    /* Check if our target device id matches with the TA */
    if(BTRUE == mac_addr_cmp(get_TA_ptr(msa), trgt_dev_id))
    {
        /* Check if we were a GO in the persistent grp */
        if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
        {
            /* Check if the device id of the GO in the P2P_GROUP_ID attribute*/
            /* sent in the invitation req frame matches with the device id   */
            if(BFALSE == mac_addr_cmp(go_dev_id, mget_p2p_dev_addr()))
                status = P2P_STAT_UNKNW_P2P_GRP;
        }
        else
        {
            /* Check if the device id of the GO in the P2P_GROUP_ID attribute*/
            /* sent in the invitation request frame matches with go device id*/
            /* stored in the persistent credential list                      */
            if(BFALSE == mac_addr_cmp(go_dev_id, g_persist_list.cl_list[0]))
                status = P2P_STAT_UNKNW_P2P_GRP;
        }
    }
    else
    {
        status = P2P_STAT_INFO_UNAVAIL;
    }

    if(P2P_STAT_SUCCESS == status)
    {
        ssid_len = MIN(MAX_SSID_LEN, (grp_id_attr_len - MAC_ADDRESS_LEN));

        /* Check if the ssid in the P2P_GROUP_ID attribute sent in the       */
        /* invitation req frame matches with ssid stored in the persistent   */
        /* credential list                                                   */
        memcpy(ssid, (grp_id_ptr + MAC_ADDRESS_LEN), ssid_len);
        ssid[ssid_len] = '\0';
        if(0 != strcmp((WORD8 *)(g_persist_list.ssid), (WORD8 *)ssid))
            status = P2P_STAT_UNKNW_P2P_GRP;
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Store the user settings */
        store_user_settings_p2p();

        /* Call the function to configure the device with the stored         */
        /* configuration and check if the configuration was succesfull       */
        if(SUCCESS == p2p_config_persist_cred())
        {
            p2p_send_inv_rsp(msa, P2P_STAT_SUCCESS, &g_persist_list);

            /* Wait for the transmission of the resp frame */
            while(BFALSE == is_machw_q_null(HIGH_PRI_Q))
            {
                add_delay(0xFFF);
            }

            set_p2p_grp_form_in_prog(BFALSE);

            /* Call the function to switch module our device is supposed  */
            /* to be the GO                                               */
            if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
            {
                /* Call the function to switch module */
                initiate_mod_switch(mac);
            }
            else
            {
                /* Restart MAC */
                restart_mac(&g_mac, 0);
            }
        }
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

	TROUT_FUNC_EXIT;
    return status;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_scan_process_inv_req                            */
/*                                                                           */
/*  Description   : This function processes the invitation request frame     */
/*                  in the wait scan state                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes the invitation request frame     */
/*                  in the wait scan state. It checks if the invitation      */
/*                  request frame is for inviting the device to join an      */
/*                  existing group or to invoke a persistent group           */
/*                  If it is for joining an existing group then the function */
/*                  performs various checks to confirm if it can join that   */
/*                  group. If the request is to invoke a persistent group    */
/*                  then the function to process the frame is called and the */
/*                  an invitation response frame is sent with appropriate    */
/*                  status code                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_scan_process_inv_req(mac_struct_t *mac, UWORD8 *msa,
                                   UWORD16 rx_len)
{
    UWORD8 *attr_start_ptr    = NULL;
    UWORD8 *attr_ptr          = NULL;
    UWORD8 *go_dev_id         = NULL;
    UWORD8 *trgt_dev_id       = NULL;
    UWORD8  is_persist        = 0;
    UWORD16 ie_len            = 0;
    UWORD16 attr_len          = 0;
    P2P_STATUS_CODE_T status  = P2P_STAT_SUCCESS;

	TROUT_FUNC_ENTER;
    /* Get the start pointer of the combined P2P attribute present in the    */
    /* invitation request                                                    */
    attr_start_ptr = get_p2p_attributes(msa, (MAC_HDR_LEN +
                                            P2P_PUB_ACT_TAG_PARAM_OFF),
                                            rx_len, &ie_len);

    /* Check if P2P attribute is present */
    if(NULL == attr_start_ptr)
    {
        status = P2P_STAT_INFO_UNAVAIL;
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Extract the invitation flag attribute */
        attr_ptr = p2p_get_attr(INVITATION_FLAGS, attr_start_ptr, ie_len,
                                &attr_len);
        if(NULL != attr_ptr)
            is_persist = CHECK_BIT(*attr_ptr, BIT0);
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Extract the group id attribute */
        attr_ptr = p2p_get_attr(P2P_GROUP_ID, attr_start_ptr, ie_len,
                                &attr_len);
        if(NULL != attr_ptr)
        {
            if(0 == is_persist)
            {
                go_dev_id   = attr_ptr;
                trgt_dev_id = mget_p2p_trgt_dev_id();

                /* Check if the a) Go address or  */
                /*              b) TA,            */
                /* matches our target device      */
                if((BTRUE == mac_addr_cmp(get_TA_ptr(msa), trgt_dev_id)) ||
                   (BTRUE == mac_addr_cmp(go_dev_id, trgt_dev_id)) ||
                   (BTRUE == is_group(trgt_dev_id)))
                {
                    /* Set the target device id to the device id of the GO of*/
                    /* requesting gruop                                      */
                    mset_p2p_trgt_dev_id(go_dev_id);
                    p2p_send_inv_rsp(msa, P2P_STAT_SUCCESS, NULL);
                }
                else
                {
                    status = P2P_STAT_INFO_UNAVAIL;
                }
            }
            else
            {
                status = p2p_wait_scan_process_pers_inv_req(mac, msa,
                                                            attr_ptr,attr_len);
            }
        }
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

    if(P2P_STAT_SUCCESS != status)
    {
        p2p_send_inv_rsp(msa, status, NULL);
        send_host_p2p_req(msa, rx_len);
    }

    TROUT_DBG4("Invitation response sent with status code %d\n\r", status);

	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);

	TROUT_FUNC_EXIT;
    return;
}

/*****************************************************************************/
/***************************** WAIT JOIN *************************************/
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_go_neg_req                         */
/*                                                                           */
/*  Description   : This function processes the GO Negotiation Request based */
/*                  on the P2P WAIT JOIN sub-states                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_go_neg_req(mac_struct_t *mac, UWORD8 *msa,
                                      UWORD16 rx_len)
{
    SWORD8  status    = -1;
    UWORD8 *src_addr = get_SA_ptr(msa);

	TROUT_FUNC_ENTER;
	
    switch(get_p2p_join_state())
    {
    case P2P_WAIT_GO_NEG_RSP:
    {
        if(BTRUE == mac_addr_cmp(g_p2p_join_req.dev_dscr.dev_addr, src_addr))
        {
			if(BTRUE == g_waiting_for_go_neg_req)
			{
				/* Cancel the timeout timer */
				cancel_mgmt_timeout_timer();

				/* Reset the flag to indicate that the GO Negotiation Req    */
				/* being expected has been received from the target device   */
				g_waiting_for_go_neg_req = BFALSE;

                /* Process the GO Negotiation Req now (since the earlier     */
                /* response sent had status "Info unavailable".              */
                status = process_p2p_go_neg_req(msa, rx_len);

                if(P2P_STAT_SUCCESS == status)
                {
                    p2p_wait_go_neg_cnf();
                }
                else
                {
                    p2p_join_complete(mac, INVALID_MLMESTATUS);
                }
			}
            else if(0 < memcmp(mget_p2p_dev_addr(), src_addr, 6))
            {
                /* Our MAC ADDR is higher so we need to respond */
                status = process_p2p_go_neg_req(msa, rx_len);

                if(P2P_STAT_SUCCESS == status)
                {
                    p2p_wait_go_neg_cnf();
                }
                else
                {
                    p2p_join_complete(mac, INVALID_MLMESTATUS);
                }
            }
            else
            {
                /* Do nothing. Wait for the other initiator to respond. */
            }
        }
        else
        {
            /* GO Negotiation request from unknown device */
            /* response with status code "Unable to accomodate request." */
            status = P2P_STAT_UNABLE_ACCO_REQ;
        }

    }
    break;
    case P2P_WAIT_WPS_START:
    {
        /* If the P2P Device is in any other P2P WAIT_JOIN state, it cannot  */
        /* accomodate any GO Negotiation requests. Send a GO Negotiation     */
        /* response with status code "Unable to accomodate request."         */
        status = P2P_STAT_INFO_UNAVAIL;
    }
    break;
    default:
    {
        /* If the P2P Device is in any other P2P WAIT_JOIN state, it cannot  */
        /* accomodate any GO Negotiation requests. Send a GO Negotiation     */
        /* response with status code "Unable to accomodate request."         */
        status = P2P_STAT_UNABLE_ACCO_REQ;
    }
    break;
    }

    /* A P2P neg rsp is sent if required */
    if(status >= 0)
        p2p_send_go_neg_rsp(msa, status);
        
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_go_neg_rsp                         */
/*                                                                           */
/*  Description   : This function processes the GO Negotiation Response based*/
/*                  on the P2P WAIT JOIN sub-states                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    : This function processes the GO Negotiation Response based*/
/*                  on the P2P WAIT JOIN sub-states. It checks the join      */
/*                  method and sends the GO NEG CONF frame accordingly       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_go_neg_rsp(mac_struct_t *mac, UWORD8 *msa,
                                      UWORD16 rx_len)
{
    SWORD8 status = -1;

	TROUT_FUNC_ENTER;
    
    switch(get_p2p_join_state())
    {
    case P2P_WAIT_GO_NEG_RSP:
    {
        status = process_p2p_go_neg_rsp(msa, rx_len);
        if(status > -1)
        {
            if((P2P_DDR_GO_NEG == g_p2p_join_req.join_method) ||
               (P2P_GO_NEG     == g_p2p_join_req.join_method))
            {
                /* Send a GO Negotiation Confirm frame */
                p2p_send_go_neg_cnf(status);

                if(0 == status)
                    p2p_join_complete(mac, SUCCESS_MLMESTATUS);
                else
                    p2p_join_complete(mac, INVALID_MLMESTATUS);
            }
            else
            {
                /* Exception. Should not occur. */
            }
        }
        else
        {
            /* Processing failed. Do nothing. Wait for timeout. */
        }
    }
    break;
    default:
    {
        /* This need not be processed in any other P2P WAIT JOIN sub-states */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_go_neg_cnf                         */
/*                                                                           */
/*  Description   : This function processes the GO Negotiation Confirm based */
/*                  on the P2P WAIT JOIN sub-states                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    : This function processes theGO Negotiation Confirmation   */
/*                  based on the P2P WAIT JOIN sub-states. It checks if a    */
/*                  GO NEG RESPONSE frame was sent and if the status in the  */
/*                  GO NEG CONF frame is SUCCESS. It calls appropriate       */
/*                  functions based on the checks                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_go_neg_cnf(mac_struct_t *mac, UWORD8 *msa,
                                      UWORD16 rx_len)
{
	TROUT_FUNC_ENTER;
	
    switch(get_p2p_join_state())
    {
    case P2P_WAIT_GO_NEG_CNF:
    {
        if(SUCCESS == process_p2p_go_neg_cnf(msa, rx_len))
        {
            if((P2P_DDR_GO_NEG == g_p2p_join_req.join_method) ||
               (P2P_GO_NEG     == g_p2p_join_req.join_method))
            {
                p2p_join_complete(mac, SUCCESS_MLMESTATUS);
            }
            else
            {
                /* Exception. Should not occur. */
            }
        }
        else
        {
            /* Processing failed. Do nothing. Wait for timeout. */
        }
    }
    break;
    default:
    {
        /* This need not be processed in any other P2P WAIT JOIN sub-states */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_dev_disc_rsp                       */
/*                                                                           */
/*  Description   : This function processes the Device Discovery Response    */
/*                  based on the P2P WAIT JOIN sub-states                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    : This function processes the Device Discovery Response    */
/*                  based on the P2P WAIT JOIN sub-states. It checks if a    */
/*                  DD request frame was sent and if the status in the DD    */
/*                  response frame is SUCCESS. It calls appropriate functions*/
/*                  based on the checks                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_dev_disc_rsp(UWORD8 *msa, UWORD16 rx_len)
{
	TROUT_FUNC_ENTER;
	
    switch(get_p2p_join_state())
    {
    case P2P_WAIT_DEV_DISC_RSP:
    {
        if(SUCCESS == process_p2p_dev_disc_rsp(msa, rx_len))
        {
            if(P2P_DDR_INVITE_PERSIST == g_p2p_join_req.join_method)
            {
                p2p_wait_inv_rsp();
            }
            else if(P2P_DDR_GO_NEG == g_p2p_join_req.join_method)
            {
                try_p2p_wait_go_neg_rsp();
            }
            else
            {
                /* Exception. Should not occur. */
            }
        }
        else
        {
            /* Processing failed. Do nothing. Wait for timeout. */
        }
    }
    break;
    default:
    {
        /* This need not be processed in any other P2P WAIT JOIN sub-states */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_inv_rsp                            */
/*                                                                           */
/*  Description   : This function processes the Invitation Response frame    */
/*                  based on the P2P WAIT JOIN sub-states                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    : This function checks the status attribute sent in the    */
/*                  invitation response frame. If it is success then it calls*/
/*                  the function to configure the device with the stored     */
/*                  credentials for persistent group formation. If the device*/
/*                  is supposed to take the role of a GO then it calls for a */
/*                  mode switch                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_inv_rsp(mac_struct_t *mac, UWORD8 *msa,
                                   UWORD16 rx_len)
{
    P2P_STATUS_CODE_T status  = P2P_STAT_INVAL_PARAM;

    TROUT_FUNC_ENTER;
    
    switch(get_p2p_join_state())
    {
    case P2P_WAIT_INV_RSP:
    {
        status = process_p2p_inv_rsp(msa, rx_len);
        if(P2P_STAT_SUCCESS == status)
        {
            if((P2P_DDR_INVITE_PERSIST == g_p2p_join_req.join_method) ||
               (P2P_INVITE_PERSIST     == g_p2p_join_req.join_method))
            {
                /* Store the user settings */
                store_user_settings_p2p();

                /* Call the function to configure the device with the stored */
                /* configuration and check if the configuration was succesful*/
                if(SUCCESS == p2p_config_persist_cred())
                {
                    set_p2p_grp_form_in_prog(BFALSE);
                    /* Call the function to switch module is the device is   */
                    /* supposed to be the GO                                 */
                    if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
                    {
                        initiate_mod_switch(mac);
                    }
                    else
                    {
                        /* Restart MAC */
                        restart_mac(&g_mac, 0);
                    }
                }
            }
            else
            {
                /* Exception. Should not occur. */
            }
        }
        else if(P2P_STAT_UNKNW_P2P_GRP == status)
        {
            if((P2P_DDR_INVITE_PERSIST == g_p2p_join_req.join_method) ||
               (P2P_INVITE_PERSIST     == g_p2p_join_req.join_method))
            {
                mem_set(&g_persist_list, 0, sizeof(p2p_persist_list_t));
            }
        }
        else
        {
            /* Processing failed. Do nothing. Wait for timeout. */
        }
    }
    break;
    default:
    {
        /* This need not be processed in any other P2P WAIT JOIN sub-states */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_prov_disc_rsp                      */
/*                                                                           */
/*  Description   : This function processes the Provision Descovery Response */
/*                  frame based on the P2P WAIT JOIN sub-states              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the start of the incoming message          */
/*                  3) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                                                                           */
/*  Processing    : This function processes the Provision Descovery Response */
/*                  frame based on the P2P WAIT JOIN sub-states. It checks   */
/*                  if the status sent in the response frame is success. If  */
/*                  it is success then it calls p2p_join_complete function   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_prov_disc_rsp(mac_struct_t *mac, UWORD8 *msa,
                                         UWORD16 rx_len)
{
	TROUT_FUNC_ENTER;
	
    switch(g_p2p_join_state)
    {
    case P2P_WAIT_PROV_DISC_RSP:
    {
        if(SUCCESS == process_p2p_prov_disc_rsp(msa, rx_len))
        {
            if(P2P_JOIN_GROUP == g_p2p_join_req.join_method)
            {
                p2p_join_complete(mac, SUCCESS_MLMESTATUS);
            }
            else
            {
                /* Exception. Should not occur. */
            }
        }
        else
        {
            send_host_p2p_req(msa, rx_len);
        }
    }
    break;
    default:
    {
        /* This need not be processed in any other P2P WAIT JOIN sub-states */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_wait_join_process_inv_req                            */
/*                                                                           */
/*  Description   : This function process the invitation request frame       */
/*                  received in the wait_join state                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the incoming message          */
/*                  2) Length of the received message frame                  */
/*                                                                           */
/*  Globals       : g_curr_persist_list                                      */
/*                                                                           */
/*  Processing    : This function process the invitation request frame       */
/*                  received in the wait_join state. It checks if the frame  */
/*                  is sent to reinvoke a persistent group and does further  */
/*                  processing if it is true. It checks if the target device */
/*                  id matches with the TA of the frame and if an invitation */
/*                  request has been sent to the device to reinvoke a        */
/*                  group. If an invitation request has been sent then it    */
/*                  matches the credentials with credential sent in the      */
/*                  invitation request to the device. If all checks are      */
/*                  succesful then it sends and invitation response frame    */
/*                  with status as SUCCESS or with appropriate FAILURE status*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_wait_join_process_inv_req(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8 *attr_start_ptr               = NULL;
    UWORD8 *attr_ptr                     = NULL;
    UWORD8 *go_dev_id                    = NULL;
    UWORD8  ssid_len                     = 0;
    UWORD8  ssid[MAX_SSID_LEN]           = {0};
    UWORD16 ie_len                       = 0;
    UWORD16 attr_len                     = 0;
    P2P_STATUS_CODE_T status             = P2P_STAT_SUCCESS;

	TROUT_FUNC_ENTER;
	
    /* Check if our target device id matches with the TA and if an invitation*/
    /* request has been sent for reinvoking a persistent group               */
    if((BFALSE == mac_addr_cmp(get_TA_ptr(msa), mget_p2p_trgt_dev_id()) ||
	   (NULL == &g_persist_list)))
    //(NULL == g_curr_persist_list))
    {
        status =  P2P_STAT_UNABLE_ACCO_REQ;
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Get the start pointer of the combined P2P attribute present in the*/
        /* invitation request                                                */
        attr_start_ptr = get_p2p_attributes(msa, (MAC_HDR_LEN +
                                                  P2P_PUB_ACT_TAG_PARAM_OFF),
                                                  rx_len, &ie_len);
        /* Check if P2P attribute is present */
        if(NULL != attr_start_ptr)
        {
            /* Extract the invitation flag attribute */
            attr_ptr = p2p_get_attr(INVITATION_FLAGS, attr_start_ptr, ie_len,
                                    &attr_len);
            if(NULL != attr_ptr)
            {
                /* Check if the invitation is for reinvoking a persistent grp*/
                /* If it is not for persistent grp then change the status    */
                /* accordingly                                               */
                if(!CHECK_BIT(*attr_ptr, BIT0))
                {
                    status =  P2P_STAT_UNABLE_ACCO_REQ;
                }
            }
            else
            {
                status = P2P_STAT_INFO_UNAVAIL;
            }
        }
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Extract the group id attribute */
        attr_ptr = p2p_get_attr(P2P_GROUP_ID, attr_start_ptr, ie_len,
                                &attr_len);
        if(NULL != attr_ptr)
        {
            go_dev_id = attr_ptr;

            /* Check if we were a GO in the persistent grp */
            if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
           {
                /* Check if the device id of the GO in the P2P_GROUP_ID attr */
                /* sent in the invitation req frame matches with the dev id  */
                if(BFALSE == mac_addr_cmp(go_dev_id, mget_p2p_dev_addr()))
                    status = P2P_STAT_UNKNW_P2P_GRP;
            }
            else
            {
                /* Check if the device id of the GO in the P2P_GROUP_ID attr */
                /* sent in the invitation request frame matches with go dev  */
                /* stored in the persistent credential list                  */
                if(BFALSE == mac_addr_cmp(go_dev_id,
                                          g_persist_list.cl_list[0]))
                {
                    status = P2P_STAT_UNKNW_P2P_GRP;
                }
            }
        }
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

    if(P2P_STAT_SUCCESS == status)
    {
        ssid_len = MIN(MAX_SSID_LEN, (attr_len - MAC_ADDRESS_LEN));

        /* Check if the ssid in the P2P_GROUP_ID attribute sent in the       */
        /* invitation req frame matches with ssid stored in the persistent   */
        /* credential list                                                   */
        memcpy(ssid, (attr_ptr + MAC_ADDRESS_LEN), ssid_len);
        ssid[ssid_len] = '\0';
        if(0 != strcmp((WORD8 *)(g_persist_list.ssid), (WORD8 *)ssid))
            status = P2P_STAT_UNKNW_P2P_GRP;
    }

    if(P2P_STAT_SUCCESS == status)
    {
        /* Store the user settings */
        store_user_settings_p2p();

        /* Call the function to configure the device with the stored         */
        /* configuration and check if the configuration was succesfull       */
        if(SUCCESS == p2p_config_persist_cred())
        {
            p2p_send_inv_rsp(msa, P2P_STAT_SUCCESS, &g_persist_list);

            /* Wait for the transmission of the resp frame */
            while(BFALSE == is_machw_q_null(HIGH_PRI_Q))
            {
                add_delay(0xFFF);
            }

            set_p2p_grp_form_in_prog(BFALSE);

            /* Call the function to switch module our device is supposed  */
            /* to be the GO                                               */
            if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_GO))
            {
                /* Call the function to switch module */
                initiate_mod_switch(&g_mac);
            }
            else
            {
                /* Restart MAC */
                restart_mac(&g_mac, 0);
            }
        }
        else
        {
            status = P2P_STAT_INFO_UNAVAIL;
        }
    }

    /* Send the invitation response if the status is failure */
    if(P2P_STAT_SUCCESS != status)
    {
        p2p_send_inv_rsp(msa, status, &g_persist_list);
    }

   	/* Free the local memory buffer allocated for P2P attributes */
    if(NULL != attr_start_ptr)
		mem_free(g_local_mem_handle, attr_start_ptr);
		
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_p2p_pub_action                            */
/*                                                                           */
/*  Description   : This function does the necessary processing on the       */
/*                  reception of a P2P public  action frame                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the received frame            */
/*                  2) Length of the recieved frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the subtype of the received P2P   */
/*                  public action frame and does the required processing     */
/*                  based on the subtype                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void sta_enabled_rx_p2p_pub_action(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8 sub_type = 0;
    UWORD8 *data     = msa + MAC_HDR_LEN;

    sub_type = data[P2P_PUB_ACT_OUI_SUBTYPE_OFF];

    switch(sub_type)
    {
        case P2P_INV_REQ:
        {
            p2p_send_inv_rsp(msa, P2P_STAT_INFO_UNAVAIL, NULL);
            send_host_p2p_req(msa, rx_len);
        }
        break;

        case P2P_GO_NEG_REQ :
        {
            p2p_send_go_neg_rsp(msa, P2P_STAT_INFO_UNAVAIL);
            send_host_p2p_req(msa, rx_len);
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
/*  Function Name : sta_enabled_rx_p2p_gen_action                            */
/*                                                                           */
/*  Description   : This function does the necessary processing on the       */
/*                  reception of a P2P general action frame                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the received frame            */
/*                  2) Length of the recieved frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the subtype of the received P2P   */
/*                  general action frame and does the required processing    */
/*                  based on the subtype                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void sta_enabled_rx_p2p_gen_action(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8 sub_type    = 0;
    UWORD8 *data       = msa + MAC_HDR_LEN;

    /* Find the subtype of the general action frame */
    sub_type = data[P2P_GEN_ACT_OUI_SUBTYPE_OFF];
    switch(sub_type)
    {
        case GO_DISC_REQ:
        {
            /* Check if the client supports discoverability */
            if(BTRUE == mget_p2p_discoverable())
            {
                /* Get a count in terms of TBTT always greater than 100 TU's */
                /* And minimum of 2 TBTT                                     */
                g_p2p_go_disc_req_rx = MAX(2, 100/mget_BeaconPeriod() + 1);
            }
        }
        break;

        case P2P_NOA:
        {
           p2p_handle_noa(msa, rx_len);
        }
        break;

        default :
        {
            /* do nothing */
        }

    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_p2p_action                                */
/*                                                                           */
/*  Description   : This function handles the incoming action frame as       */
/*                  appropriate in the ENABLED state.                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
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

void sta_enabled_rx_p2p_action(UWORD8 *msg)
{
    UWORD8     *msa        = 0;
    UWORD8     *data       = 0;
    UWORD16    rx_len      = 0;
    wlan_rx_t  *wlan_rx    = (wlan_rx_t*)msg;

    /* Pointer to the start of the frame */
    msa  = wlan_rx->msa;

    /* Pointer to the start of the data portion in the frame */
    data = msa + MAC_HDR_LEN;

    rx_len = wlan_rx->rx_len;

    /* Check if it is P2P public action frame or P2P general action frame */
    if(BTRUE == is_p2p_pub_action_frame(data))
    {
        sta_enabled_rx_p2p_pub_action(msa, rx_len);
    }
    else if(BTRUE == is_p2p_gen_action_frame(data))
    {
        sta_enabled_rx_p2p_gen_action(msa, rx_len);
    }
}
#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
