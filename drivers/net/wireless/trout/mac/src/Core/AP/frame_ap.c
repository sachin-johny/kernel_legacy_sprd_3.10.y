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
/*  File Name         : frame_ap.c                                           */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to AP mode of     */
/*                      operation.                                           */
/*                                                                           */
/*  List of Functions : prepare_auth_rsp                                     */
/*                      prepare_asoc_rsp                                     */
/*                      prepare_disasoc                                      */
/*                      prepare_beacon_ap                                    */
/*                      prepare_probe_rsp_ap                                 */
/*                      prepare_null_frame_ap                                */
/*                      is_challenge_txt_equal                               */
/*                      set_cap_info_ap                                      */
/*                      check_erp_rates                                      */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "iconfig.h"
#include "index_util.h"
#include "prot_if.h"

/*****************************************************************************/
/* Extern Global Variables                                                   */
/*****************************************************************************/

extern UWORD8 g_tim_element_index;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_auth_rsp                                         */
/*                                                                           */
/*  Description   : This function prepares the authentication response       */
/*                  frames. (sequence 2 and 4)                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the authentication frame                   */
/*                  2) Pointer to the incoming authentication frame          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the authentication response        */
/*                  frame are set as specified in the standard, depending    */
/*                  on the authentication request received. Also the         */
/*                  association table elements are updated, as required,     */
/*                  to indicate the current state of the sending STA.        */
/*                                                                           */
/*  Outputs       : The contents of the given authentication frame have      */
/*                  been set.                                                */
/*                                                                           */
/*  Returns       : Authentication response frame length.                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_auth_rsp(UWORD8* data, UWORD8* auth_req)
{
    asoc_entry_t  *ae                 = 0;
    UWORD8        *data8              = 0;
    UWORD16       index               = 0;
    UWORD16       auth_rsp_len        = 0;
    UWORD16       auth_type           = 0;
    UWORD16       auth_seq            = 0;
    UWORD8        addr2[6]            = {0};
    UWORD8        *ch_text            = NULL;
    UWORD8        auth_type_cfg       = mget_auth_type();

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
    set_frame_control(data, (UWORD16)AUTH);

    /* Get the address of the STA requesting authentication */
    get_address2(auth_req, addr2);

    /* Set DA to the address of the STA requesting authentication */
    set_address1(data, addr2);

    /* Set SA to the dot11MacAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    if(get_wep(auth_req) == 1)
    {
        /* Recieved packed is encrypted */
        /* Since all the information is extracted already, */
        /* advance the pointer by 4 to skip the wep header */
        auth_req += WEP_HDR_LEN;
    }

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*              Authentication Frame - Frame Body                        */
    /* --------------------------------------------------------------------- */
    /* |Auth Algo Number|Auth Trans Seq Number|Status Code| Challenge Text | */
    /* --------------------------------------------------------------------- */
    /* | 2              |2                    |2          | 3 - 256        | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and 'frame data' pointer */
    index = MAC_HDR_LEN;
    data8 = (UWORD8*)(data + index);

    /* Initialize the authentication response length. The initialized value  */
    /* does not contain the challenge text length. It is included only if    */
    /* the response is sequence 2.                                           */
    auth_rsp_len = MAC_HDR_LEN + AUTH_ALG_LEN + AUTH_TRANS_SEQ_NUM_LEN +
                   STATUS_CODE_LEN + FCS_LEN;

    /* Extract Authentication type */
    auth_type = get_auth_algo_num(auth_req);

    /* Extract Authentication Sequence Number */
    auth_seq  = get_auth_seq_num(auth_req);

    /* Set Authentication algorithm number*/
    data8[0] = (auth_type & 0x00FF);
    data8[1] = (auth_type & 0xFF00) >> 8;

    /* Set Authentication Transaction Sequence Number to a value equal to    */
    /* the sequence number of the incoming authentication frame, incremented */
    /* by one.                                                               */
    data8[2] = ((auth_seq + 1) & 0x00FF);
    data8[3] = ((auth_seq + 1) & 0x00FF) >> 8;

    /* The incoming status code is reserved and is not checked for success   */


    /* Initialize the status code in the authentication response frame to    */
    /* 'Successful' if the status code is successful.                        */
    data8[4] = SUCCESSFUL_STATUSCODE;
    data8[5] = 0;

    /* If the Authentication algorithm specified is not supported, respond   */
    /* with 'unsupported alogrithm' status.                                  */
    if(((BTRUE == check_auth_type_prot())) && (auth_type_cfg != ANY) &&
       (((auth_type_cfg == OPEN_SYSTEM) && (auth_type != OPEN_SYSTEM)) ||
       ((auth_type_cfg == SHARED_KEY) && (auth_type != SHARED_KEY))))
    {
        data8[4] = UNSUPT_ALG;
        data8[5] = 0;
    }
    else
    {
        /* Update Association Table */

        /* Check if station already has an entry in the association table    */
        ae = (asoc_entry_t*)find_entry(addr2);

        /* No entry is found in the Association Table. This implies that the */
        /* Authentication transcation Sequence Number should be 1, and a new */
        /* association table entry must be made.                             */
        if(ae == 0)
        {
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
            /* Check if a new STA can be allowed to authenticate/associate */
		    if(is_new_sta_allowed(addr2) == BFALSE)
		    {
				data8[4] = UNSPEC_FAIL;
				data8[5] = 0;
				return auth_rsp_len;
			}
#endif
            /* Authentication Transaction Sequence 1 */
            if(auth_seq == 0x0001)
            {
                ae = (asoc_entry_t *)mem_alloc(g_local_mem_handle,
                                               sizeof(asoc_entry_t));
                if(ae == NULL)
                {
                    /* Key index table is full */
                    data8[4] = AP_FULL;
                    data8[5] = 0;

                    return auth_rsp_len;
                }

                /* Reset the AE Handle */
                mem_set(ae, 0, sizeof(asoc_entry_t));
                ae->cipher_type = NO_ENCRYP;


                if(auth_type == OPEN_SYSTEM)
                {
                    ae->state = AUTH_COMPLETE;

                    /* Add the entry to the table */
                    add_entry((void*)ae, addr2);
                }
                else
                {
                    /* If WEP is enabled, shared key authentication is       */
                    /* responded with challenge text                         */
                    if(is_wep_allowed() == BTRUE)
                    {
                        UWORD16 i = 0;

                        /* Add the entry to the table */
                        add_entry((void*)ae, addr2);

                        /* If WEP is implemented and supported. Obtain a new */
                        /* STA Index                                         */
                        ae->sta_index = get_new_sta_index(addr2);

                        if(ae->sta_index == INVALID_ID)
                        {
                            /* Key index table is full */
                            data8[4] = AP_FULL;
                            data8[5] = 0;

                            /* Free association entry memory */
                            delete_entry(addr2);
                            return auth_rsp_len;
                        }

                        /* Shared Key Authentication is used */
                        ae->state   = AUTH_KEY_SEQ1;

                        /* Challenge Text Element                  */
                        /* --------------------------------------- */
                        /* |Element ID | Length | Challenge Text | */
                        /* --------------------------------------- */
                        /* | 1         |1       |1 - 253         | */
                        /* --------------------------------------- */
                        data8[6] = ICTEXT;
                        data8[7] = CHTXT_SIZE;

                        /* Generate the challenge text using the uniform */
                        /* random number generator. Scale the output to  */
                        /* be in the range 0 to 0xFF.                    */
                        for(i = 0; i < CHTXT_SIZE; i++)
                        {
                            /* Update challenge text */
                            data8[8 + i] = get_random_byte();
                        }

                        /* Add the challenge text element length */
                        auth_rsp_len += (CHTXT_SIZE + IE_HDR_LEN);

                        /* Copy the challenge text */
                        memcpy(ae->ch_text, &data8[8], CHTXT_SIZE);

                        add_wep_entry(ae->sta_index, mget_WEPDefaultKeyID(),
                                  addr2);

                        /* A timer needs to be started for the shared key*/
                        /* message # 3                                   */
                        ae->msg_3_timer = create_msg_3_timer(msg_3_timeout,
                                                        (ADDRWORD_T)addr2, msg_3_timeout_work); //Hugh
                        /* Start Message 3 timer */
                        start_msg_3_timer(ae->msg_3_timer,
                                     mget_AuthenticationResponseTimeOut());

                        ae->cipher_type = get_wep_type(mget_WEPDefaultKeyID());
                    }
                    else
                    {
                        /* Wep is not supported so Shared Key Auth is        */
                        /* declined.                                         */
                        data8[4] = UNSUPT_ALG;
                        data8[5] = 0;
                    }
                }

                /* Add an association entry if status code is successful */
                if(data8[4] == SUCCESSFUL_STATUSCODE)
                {
                    /* Initialize asoc_id to 0, which is an invalid Id. When */
                    /* asoc request is received and if association table is  */
                    /* full this entry will be removed.                      */
                    ae->asoc_id = 0;

                    /* Initialize security entry table */
                    init_prot_handle_ap(ae);
                }
                else
                {
                    /* Free association entry memory */
                    mem_free(g_local_mem_handle, ae);
                }
            }
            /* Authentication Transaction Seq No is out of expected sequence.*/
            else
            {
                data8[4] = AUTH_SEQ_FAIL;
                data8[5] = 0;
            }
        }
        /* Entry exists in the association table */
        else
        {

            /* When STA authenticates again check the earlier PS state       */
            /* ASSUMPTION that auth is always sent in ACTIVE state           */
            check_ps_state(ae, ACTIVE_PS);

            /* Update auth_asoc state of the STA and status code in the      */
            /* response frame based on the entry in the table.               */
            if((ae->state == ASOC) && (auth_type == OPEN_SYSTEM))
            {
                /* The station has already been associated. Hence the state  */
                /* is not required to be changed.                            */
            }
            else
            {
                /* Open System Authentication */
                if(auth_type == OPEN_SYSTEM)
                {
                    ae->state = AUTH_COMPLETE;
                    ae->cipher_type = NO_ENCRYP;
                }
                else /* Shared Key Authentication */
                {
                    /* If WEP is enabled, shared key authentication is       */
                    /* responded with challenge text                         */
                    if(is_wep_allowed() == BTRUE)
                    {
                        ae->cipher_type = get_wep_type(mget_WEPDefaultKeyID());
                        /* If WEP is implemented and supported*/
                        /* Final transaction sequence */
                        if((ae->state == AUTH_KEY_SEQ1) ||
                           (ae->state == AUTH_COMPLETE))
                        {
                            /* Sequence expected is 0x03 */
                            if(auth_seq == 0x03)
                            {
                                ch_text   = get_auth_ch_text(auth_req);
                                if(is_challenge_txt_equal(ch_text,
                                                        ae->ch_text) == BTRUE)
                                {
                                    ae->state = AUTH_COMPLETE;

                                    /* Cancel Message 3 timer */
                                    cancel_msg_3_timer(ae->msg_3_timer);

                                }
                                else
                                {
                                    data8[4] = CHLNG_FAIL;
                                    data8[5] = 0;
                                }
                            }
                            else if(auth_seq == 0x01)
                            {
                                UWORD8 i = 0;

                                /* Re-transmission of the Authentication Req */
                                /* Form Authentication Seq-2 Packet again    */

                                /* Add the Challenge Packet */
                                data8[6] = ICTEXT;
                                data8[7] = CHTXT_SIZE;

                                /* Generate challenge text using   */
                                /* uniform random number generator */
                                for(i = 0; i < CHTXT_SIZE; i++)
                                {
                                    /* Update challenge text */
                                    ae->ch_text[i] = get_random_byte();
                                }

                                /* Copy the challenge text */
                                memcpy(&data8[8], ae->ch_text, CHTXT_SIZE);

                                /* Add the challenge text element length */
                                auth_rsp_len += (CHTXT_SIZE + IE_HDR_LEN);

                                /* Restart Message 3 timer */
                                cancel_msg_3_timer(ae->msg_3_timer);
                                start_msg_3_timer(ae->msg_3_timer,
                                     mget_AuthenticationResponseTimeOut());

                                /* Change the state to Auth Key Seq1 */
                                ae->state = AUTH_KEY_SEQ1;
                            }
                            else /* Out of expected sequence */
                            {
                                data8[4] = AUTH_SEQ_FAIL;
                                data8[5] = 0;
                            }
                        }
                        else if(ae->state == ASOC)
                        {
                            /* If STA leaves the network and tries to assoc */
                            /* before Assoc is removed because of aging     */

                            /* Respond with challenge text */
                            /* Get the Challenge text memory if required */
                            UWORD16 i = 0;

                            /* Challenge Text Element                  */
                            /* --------------------------------------- */
                            /* |Element ID | Length | Challenge Text | */
                            /* --------------------------------------- */
                            /* | 1         |1       |1 - 253         | */
                            /* --------------------------------------- */
                            data8[6] = ICTEXT;
                            data8[7] = CHTXT_SIZE;

                            /* Generate the challenge text using the   */
                            /* uniform random number generator. Scale  */
                            /* the output to be in the range 0 to 0xFF.*/
                            for(i = 0; i < CHTXT_SIZE; i++)
                            {
                                /* Update challenge text */
                                data8[8 + i] = get_random_byte();
                            }

                            /* Add the challenge text element length */
                            auth_rsp_len += (CHTXT_SIZE + IE_HDR_LEN);

                            /* Copy the challenge text */
                            memcpy(ae->ch_text, &data8[8], CHTXT_SIZE);

                            add_wep_entry(ae->sta_index,
                                          mget_WEPDefaultKeyID(), addr2);

                            /* A timer is needed for shared key msg-3 */
                        ae->msg_3_timer = create_msg_3_timer(
                                                msg_3_timeout,
                                                (ADDRWORD_T)addr2, msg_3_timeout_work);


                            /* Start Message 3 timer */
                            start_msg_3_timer(ae->msg_3_timer,
                                     mget_AuthenticationResponseTimeOut());

                            /* Change the state to Auth Key Seq1 */
                            ae->state = AUTH_KEY_SEQ1;
                        }
                    }
                    else
                    {
                        /* If wep is disabled */
                        data8[4] = UNSUPT_ALG;
                        data8[5] = 0;
                    }
                }
            }
        }
    }
    return auth_rsp_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_asoc_rsp                                         */
/*                                                                           */
/*  Description   : This function prepares the association response or       */
/*                  reassociation frame.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the (re)association response frame         */
/*                  2) Pointer to incoming (re)association request frame     */
/*                  3) Association Id                                        */
/*                  4) Status code to be set                                 */
/*                  5) Frame Subtype (ASSOC_RSP or REASSOC_RSP)              */
/*                  6) Association Request frame length                      */
/*                  7) Is P2P asoc resquest.                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the (re)association response       */
/*                  frame are set as specified in the standard.              */
/*                                                                           */
/*  Outputs       : The contents of the given (re)association response       */
/*                  frame have been set.                                     */
/*                                                                           */
/*  Returns       : Association response frame length.                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_asoc_rsp(UWORD8* data, UWORD8* asoc_req, UWORD16 asoc_id,
                         UWORD16 status, TYPESUBTYPE_T type, UWORD16 asoc_len,
                         UWORD8 ht_enabled_sta, BOOL_T is_p2p)
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
    set_frame_control(data, (UWORD16)type);

    /* DA is address of STA requesting association */
    set_address1(data, asoc_req + ADDR2_OFFSET);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*              Association Response Frame - Frame Body                  */
    /* --------------------------------------------------------------------- */
    /* | Capability Information |   Status Code   | AID | Supported  Rates | */
    /* --------------------------------------------------------------------- */
    /* |2                       |2                |2    |3-10              | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index = MAC_HDR_LEN;

    /* Set the capability information field */
    set_cap_info_field_ap(data, index);

    /* Set Status Code to the input Status Code */
    data[index + 2] = (status & 0xFF);
    data[index + 3] = ((status & 0xFF00) >> 8);

    /* Set Association ID element */
    data[index + 4] = (asoc_id & 0xFF);
    data[index + 5] = ((asoc_id & 0xFF00) >> 8);

    /* Update the index with the Association ID, Capability Information,     */
    /* Status Code element lengths.                                          */
    index += (ASOC_ID_LEN + CAP_INFO_LEN + STATUS_CODE_LEN);

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

    /* Add WMM Parameter element if required */
    index += set_wmm_asoc_rsp_params(data, index, asoc_req, asoc_len);

    if(ht_enabled_sta == 1)
    {
        /* Add HT-Capabilities Information element */
        index += set_ht_capabilities(data, index, type);

        /* Add HT-Operation Information element */
        index += set_ht_opern_element_ap(data, index);

        /* Add Overlapping BSS Scan Parameters Information element */
        index += set_obss_scan_params(data, index);

        /* Add Extended Capabilities Information element */
        index += set_ext_cap(data, index);
    } /* (ht_enabled_sta == 1) */

    /* Add WPS WSC Information Element */
    index += set_wps_wsc_info_element(data, index, ASSOC_RSP);

    /* Add P2P Information element. */
    index += set_asoc_resp_p2p_ie(data, index, status, is_p2p);

    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_disasoc                                          */
/*                                                                           */
/*  Description   : This function prepares the disassociation frame on       */
/*                  receiving a Class 3 frame from an unassociated           */
/*                  station.                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the disassociation frame                   */
/*                  2) Address of the destination STA.                       */
/*                  3) Reason Code                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the disassociation frame are       */
/*                  set as specified in the standard.                        */
/*                                                                           */
/*  Outputs       : The contents of  the given disassociation frame have     */
/*                  been set.                                                */
/*                                                                           */
/*  Returns       : Disassociation frame length.                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_disasoc(UWORD8* data, UWORD8* da, UWORD16 reason_code)
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

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)DISASOC);

    /* Set Address1 to the address of unassociated STA */
    set_address1(data, da);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                    Disassociation Frame - Frame Body                  */
    /* --------------------------------------------------------------------- */
    /* |                           Reason Code                             | */
    /* --------------------------------------------------------------------- */
    /* |2                                                                  | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set Reason Code to Reason sent as argument */
    data[MAC_HDR_LEN]     = (reason_code & 0x00FF);
    data[MAC_HDR_LEN + 1] = (reason_code & 0xFF00) >> 8;

    return MAC_HDR_LEN + REASON_CODE_LEN + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prepare_beacon_ap                                     */
/*                                                                           */
/*  Description      : This function prepares the fixed portion of the       */
/*                     beacon frame.                                         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the beacon frame                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The various fields of the beacon frame are set as     */
/*                     specified in the standard.                            */
/*                                                                           */
/*  Outputs          : The contents of the given beacon frame have been set. */
/*                                                                           */
/*  Returns          : Beacon frame length.                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_beacon_ap(UWORD8* data)
{
    UWORD16 index = 0;
    UWORD16 value = 0;


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

    /* The Type/Subtype, Duration, Sequence Number and Fragment Number fields*/
    /* in the Frame Control Field are set.                                   */
    set_frame_control(data, (UWORD16)BEACON);

    /* Set durationID to 0 */
    set_durationID(data, 0);

    /* Set fragment number to 0 */
    set_fragment_number(data, 0);

    /* DA is Broadcast Address */
    set_address1(data, mget_bcst_addr());

    /* SA is the dot11MacAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                       Beacon Frame - Frame Body                       */
    /* -------------------------------------------------------------------   */
    /* |Timestamp|BcnInt|CapInfo|SSID|SupRates|DSParamSet| CFP Set |TIM  |   */
    /* -------------------------------------------------------------------   */
    /* |8        |2     |2      |2-34|3-10    |3         | 8       |6-256|   */
    /* -------------------------------------------------------------------   */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Set Beacon interval */
    value = mget_BeaconPeriod();
    data[index + TIME_STAMP_LEN]   = value & 0xFF;
    data[index + TIME_STAMP_LEN+1] = (value >> 8) & 0xFF;

    /* Update the index with the lengths of the Timestamp, Beacon Interval */
    index += (TIME_STAMP_LEN + BEACON_INTERVAL_LEN);

    /* Set the capability information field and update the index with the    */
    /* length of the field.                                                  */
    set_cap_info_field_ap(data, index);
    index += CAP_INFO_LEN;

    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
    /* The ssid element is set only if the BCAST SSID option is reset        */
    /* Else the element ID is set with value zero                            */
    if(get_bcst_ssid() == 0)
    {
        index += set_ssid_element(data, index, mget_DesiredSSID());
    }
    else
    {
        data[index++] = ISSID;
        data[index++] = 0;
    }

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the PHY parameters depending on the PHY type */
    index += set_phy_params(data, index);

    /* Set the ERP Information element and update the index value with the   */
    /* ERP information element length.                                       */
    index += set_erp_info_field(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

    /* Set the TIM element field with default parameters and update the      */
    /* index value with the default length.                                  */
    data[index]     = ITIM;
    data[index + 1] = DEFAULT_TIM_LEN;
    data[index + 2] = 0;
    data[index + 3] = mget_DTIMPeriod();
    data[index + 4] = 0;
    data[index + 5] = 0;

    g_tim_element_index = index;
    index += (DEFAULT_TIM_LEN + 2);

	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Country Information element */
    index += set_country_info_field(data, index);

    /* Add the spectrum management parameters */
    index += set_spectrum_mgmt_params(data, index);
	
	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Supported operating classes element */
    index += set_sup_opclas_11d_elem(data, index);

    /* Set the WMM related parameters */
    index += set_wmm_beacon_params(data, index);

    /* Add the security information element */
    index += set_sec_info_element(data, index, 0);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, BEACON);

    /* Add HT-Operation Information element */
    index += set_ht_opern_element_ap(data, index);

    /* Add Secondary Channel Offset Information Element */
    index += set_sec_channel_offset(data, index, BEACON);

    /* Add Overlapping BSS Scan Parameters Information element */
    index += set_obss_scan_params(data, index);

    /* Add Extended Capabilities Information element */
    index += set_ext_cap(data, index);

    /* Add WPS WSC Information Element */
    index += set_wps_wsc_info_element(data, index, BEACON);

    /* Add P2P IE */
    index += set_p2p_beacon_ie(data, index);

    /* Update the TIM Trailer Count */
    g_tim_element_trailer_len = (index - g_tim_element_index -
                                  DEFAULT_TIM_LEN - 2);

    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_probe_rsp_ap                                     */
/*                                                                           */
/*  Description   : This function prepares the probe response frame.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe response frame                   */
/*                  2) Pointer to the probe request frame                    */
/*                  3) Is P2P IE present in the probe request frame          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the probe response frame are       */
/*                  set as specified in the standard.                        */
/*                                                                           */
/*  Outputs       : The contents of  the given probe response frame have     */
/*                  been set.                                                */
/*                                                                           */
/*  Returns       : Probe response frame length.                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_probe_rsp_ap(UWORD8* data, UWORD8* probe_req, BOOL_T is_p2p)
{
    UWORD16 value    = 0;
    UWORD16 index    = 0;

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
    set_frame_control(data, (UWORD16)PROBE_RSP);

    /* Set DA to the address of transmitting STA */
    set_address1(data, probe_req + ADDR2_OFFSET);

    /* Set SA to the dot11MacAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                       Probe Response Frame - Frame Body               */
    /* --------------------------------------------------------------------  */
    /* |Timestamp |BcnInt |CapInfo |SSID |SupRates |DSParamSet | CFP Set  |  */
    /* --------------------------------------------------------------------  */
    /* |8         |2      |2       |2-34 |3-10     |3          | 8        |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Set Beacon interval (2 bytes) */
    value = mget_BeaconPeriod();
    data[index + TIME_STAMP_LEN]   = value & 0xFF;
    data[index + TIME_STAMP_LEN+1] = (value >> 8) & 0xFF;

    /* Update the index with the lengths of the Timestamp, Beacon Interval. */
    index += (TIME_STAMP_LEN + BEACON_INTERVAL_LEN);

    /* Set the capability information field and update the index with the    */
    /* length of the field.                                                  */
    set_cap_info_field_ap(data, index);
    index += CAP_INFO_LEN;

    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
    index += set_ssid_element(data, index, mget_DesiredSSID());

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the PHY parameters depending on the PHY type */
    index += set_phy_params(data, index);
	
	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Country Information element */
    index += set_country_info_field(data, index);

    /* Set the ERP Information element and update the index value with the   */
    /* ERP information element length.                                       */
    index += set_erp_info_field(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

    /* Set the Security Information Element */
    index += set_sec_info_element(data, index, 0);

    /* Set the WMM parameters */
    index += set_wmm_probe_rsp_params(data, index);
	
	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Supported operating classes element */
    index += set_sup_opclas_11d_elem(data, index);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, PROBE_RSP);

    /* Add HT-Operation Information element */
    index += set_ht_opern_element_ap(data, index);

    /* Add Secondary Channel Offset Information Element */
    index += set_sec_channel_offset(data, index, PROBE_RSP);

    /* Add Overlapping BSS Scan Parameters Information element */
    index += set_obss_scan_params(data, index);

    /* Add Extended Capabilities Information element */
    index += set_ext_cap(data, index);

    /* Add WPS WSC Information Element */
    index += set_wps_wsc_info_element(data, index, PROBE_RSP);

    /* Add the P2P IE. NOTE: Should be the last IE */
    index += set_p2p_ie_probe_rsp(data, index,  is_p2p);

    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : is_challenge_txt_equal                                */
/*                                                                           */
/*  Description      : This function compares if the challenge text received */
/*                     is as expected.                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Challenge Text element in the       */
/*                        decrypted authentication frame.                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The various fields of the probe response frame are    */
/*                     set as specified in the standard.                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T. The result of the comparison.                 */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_challenge_txt_equal(UWORD8* data, UWORD8 *tx_ch_text)
{
    UWORD8  *ch_text    = 0;
    UWORD16 i           = 0;
    UWORD8  ch_text_len = 0;

    /* Challenge Text Element                  */
    /* --------------------------------------- */
    /* |Element ID | Length | Challenge Text | */
    /* --------------------------------------- */
    /* | 1         |1       |1 - 253         | */
    /* --------------------------------------- */
    ch_text_len = data[1];
    ch_text     = data + 2;

    for(i = 0; i < ch_text_len; i++)
    {
        /* Return false on mismatch */
        if(ch_text[i] != tx_ch_text[i])
            return BFALSE;
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name  : set_cap_info_field_ap                                   */
/*                                                                           */
/*  Description    : This function sets the Capability Information Field     */
/*                   in the given frame.                                     */
/*                                                                           */
/*  Inputs         : 1) Pointer to the frame.                                */
/*                   2) Index of the Capability Information Field            */
/*                                                                           */
/*  Globals        : None                                                    */
/*                                                                           */
/*  Processing     : The various bits of the Capability Information field    */
/*                   is set depending on the Privacy and Short Preamble      */
/*                   Capabilities of the station.                            */
/*                                                                           */
/*  Outputs        : The given data frame contents from the given offset     */
/*                   are set to the Capability Information Field.            */
/*                                                                           */
/*  Returns        : None                                                    */
/*  Issues         : None                                                    */
/*                                                                           */
/*****************************************************************************/

void set_cap_info_field_ap(UWORD8* data, UWORD16 index)
{
    /* Capability Information Field                                          */
    /* -------------------------------------------------------------------   */
    /* |ESS|IBSS|CFPollable|CFPReq|Privacy|Preamble|PBCC|Agility|SpecMgmt|   */
    /* -------------------------------------------------------------------   */
    /* |B0 |B1  |B2        |B3    |B4     |B5      |B6  |B7     |B8      |   */
    /* -------------------------------------------------------------------   */
    /* |QoS |ShortSlot|APSD|Reserved|DSSS-OFDM |Delayed BA |Immediate BA |   */
    /* -------------------------------------------------------------------   */
    /* |B9  |B10      |B11 |B12     |B13       |B14        |B15          |   */
    /* -------------------------------------------------------------------   */

    /* The ESS  bit is set  by  an AP in the  Association  Response,  Probe  */
    /* Response  and Beacon  Frames.                                         */
    data[index] = ESS;

    /* The Privacy bit is set if WEP is enabled */
    if(mget_PrivacyInvoked() == TV_TRUE)
            data[index] |= PRIVACY;

    /* The higher 8 bits of the Capability Information Field are reserved    */
    /* for 802.11a and 802.11b.                                              */
    data[index + 1] = 0;

    /* Set the PHY specific capability bits */
    set_capability_phy(data, index);

    /* Set protocol specific capability bits */
    set_capability_prot_ap(data, index);
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_probe_request_ap                                 */
/*                                                                           */
/*  Description   : This function send the probe request                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe request frame                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sends the probe request                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_probe_request_ap(UWORD8 *data)
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
    set_frame_control(data, (UWORD16)PROBE_REQ);

    /* Set fragment number of the outgoing packet to zero */
    set_fragment_number(data, 0);

    /* Set Address1 to broadcast address */
    set_address1(data, mget_bcst_addr());

    /* Set Address2 to dot11MacAddress */
    set_address2(data, mget_StationID());

    /* Set Address3 to Broadcast BSSID */
    set_address3(data, mget_bcst_addr());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                       Probe Request Frame - Frame Body                */
    /* --------------------------------------------------------------------- */
    /* |           SSID           |          Supported Rates               | */
    /* --------------------------------------------------------------------- */
    /* |2-34                      |3-10                                    | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index = MAC_HDR_LEN;

    /* Set ssid */
    index += set_ssid_element(data, index, get_probe_req_ssid_prot());

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, PROBE_REQ);

    /* Set the WPS IE if required */
    index += set_wps_wsc_info_element(data, index, PROBE_REQ);

    /* Set the P2P IE if required */
    index += set_p2p_ie_probe_req(data, index);

    return index + FCS_LEN;

}


#endif /* BSS_ACCESS_POINT_MODE */
