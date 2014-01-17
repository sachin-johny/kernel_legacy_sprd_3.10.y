/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wpse_parse.c                                         */
/*                                                                           */
/*  Description       : This file contains the parsing functions to related  */
/*                      the WPS protocol                                     */
/*                                                                           */
/*  List of Functions : wps_parse_attri                                      */
/*                      wps_parse_creden                                     */
/*                      wps_find_attr                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_prot.h"
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static UWORD8* wps_find_attr(UWORD8* st_ptr, UWORD8* end_ptr, UWORD16 attr_id);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_parse_attri                                       */
/*                                                                           */
/*  Description      : This function parses the Wi-Fi protected setup        */
/*                     attributes defined in TLV (Type, Length and Value)    */
/*                     format as per [WPSSPEC]                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Array of pointers to structures where attributes   */
/*                        for each message/Information element are stored.   */
/*                     3) length of the message/Information element to be    */
/*                        parsed                                             */
/*                     4) Maximum number of concatenated message/Information */
/*                        elements to be parsed.                             */
/*                     5) Indicate whether the input message is WSC_IE or not*/
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function send the NACK to the peer entity and    */
/*                     Error message to the host/user                        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_parse_attri(UWORD8 *msg_ptr, wps_rx_attri_t** parse_attr,
                       UWORD16 msg_len, UWORD8 max_element_cnt,
                       BOOL_T is_wsc_ie)
{
    wps_rx_attri_t *cur_attr_buff;
    UWORD8  *msg_cur_ptr, *msg_end_ptr;
    UWORD16 attr_type, attr_len;
    UWORD8  msg_cnt;
    BOOL_T  first_ver_rec = BFALSE;

    if(WPS_TLV_MIN_LEN > msg_len)
    {
        return 0;
    }
    cur_attr_buff = parse_attr[0];
    msg_cnt = 1;
    memset(cur_attr_buff, 0, sizeof(wps_rx_attri_t));
    msg_cur_ptr = msg_ptr;
    msg_end_ptr = msg_cur_ptr + msg_len;

    /*************************************************************************/
    /* Parse the received message                                            */
    /*************************************************************************/
    while (msg_cur_ptr < msg_end_ptr)
    {
        /*********************************************************************/
        /* If there is no scope for another Attribute then its an incorrect  */
        /* message. Also if length of the attribute is more than the         */
        /* remaining message length then also message is incorrect. Return 0 */
        /* to indicate Failure                                               */
        /*********************************************************************/
        if (msg_end_ptr - msg_cur_ptr < WPS_TLV_MIN_LEN)
        {
            return msg_cnt;
        }
        attr_type     = GET_U16_BE(msg_cur_ptr);
        msg_cur_ptr += 2;
        attr_len      = GET_U16_BE(msg_cur_ptr);
        msg_cur_ptr += 2;
        if(attr_len > msg_end_ptr - msg_cur_ptr)
        {
            return msg_cnt;
        }

        /*********************************************************************/
        /* Check the parse attribute, if its there in required list the check*/
        /* length field if it also passes the assign the pointer of the data */
        /* field to corresponding field in parsed structure. If field not    */
        /* there in required Attributes then ignore it, for attribute        */
        /* validity check failure return 0 to indicate Failure               */
        /*********************************************************************/
        switch (attr_type)
        {
            case WPS_ATTR_VERSION:
                if (1 != attr_len)
                {
                    return 0;
                }
                /*************************************************************/
                /* If version number is received then check if its the first */
                /* one if so then store it in the parsed buffer. if not then */
                /* in case message is WSC_IE, increment the message count and*/
                /* assign next parsed attribute buffer to current attribute  */
                /* buffer pointer. If already received maximum number of     */
                /* message then return. In case input message is not WSC_IE, */
                /* this is an error condition so return 0                    */
                /*************************************************************/
                if(BFALSE == first_ver_rec)
                {
                    first_ver_rec = BTRUE;
                }
                else
                {
                    if(BTRUE != is_wsc_ie)
                    {
                        return 0;
                    }
                    if(msg_cnt >= max_element_cnt)
                    {
                        return msg_cnt;
                    }
                    else
                    {
                        cur_attr_buff = parse_attr[msg_cnt];
                        memset(cur_attr_buff, 0, sizeof(wps_rx_attri_t));
                        msg_cnt++;
                    }
                }
                cur_attr_buff->version = msg_cur_ptr;
                break;
            case WPS_ATTR_MEG_TYPE:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->msg_type = msg_cur_ptr;
                break;
            case WPS_ATTR_ENR_NONCE:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->enr_nonce = msg_cur_ptr;
                break;
            case WPS_ATTR_REG_NONCE:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->reg_nonce = msg_cur_ptr;
                break;
            case WPS_ATTR_UUID_R:
            case WPS_ATTR_UUID_E:
                if (WPS_UUID_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->uuid_r = msg_cur_ptr;
                break;

            case WPS_ATTR_PUBLIC_KEY:
                if (WPS_PUBLIC_KEY_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->pkr = msg_cur_ptr;
                break;

            case WPS_ATTR_AUTH_TYPE_FLAGS:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->auth_type_flags = msg_cur_ptr;
                break;
            case WPS_ATTR_ENCRY_TYPE_FLAGS:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->encr_type_flags = msg_cur_ptr;
                break;
            case WPS_ATTR_CONN_TYPE_FLAGS:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->conn_type_flags = msg_cur_ptr;
                break;
            case WPS_ATTR_CONFIG_METHODS:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->config_meth = msg_cur_ptr;
                break;
            case WPS_ATTR_PRIM_DEV_TYPE:
                if (WPS_PRIM_DEV_TYPE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->prim_dev_type = msg_cur_ptr;
                break;
            case WPS_ATTR_RF_BANDS:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->rf_bands = msg_cur_ptr;
                break;
            case WPS_ATTR_ASSOC_STATE:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->assoc_state = msg_cur_ptr;
                break;
            case WPS_ATTR_CONFIG_ERROR:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->config_error = msg_cur_ptr;
                break;
            case WPS_ATTR_DEV_PASS_ID:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->dev_pass_id = msg_cur_ptr;
                break;
            case WPS_ATTR_OS_VERSION:
                if (WPS_OS_VERSION_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->os_version = msg_cur_ptr;
                break;
            case WPS_ATTR_AUTHENTICATOR:
                if (WPS_AUTHENTICATOR_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->authenticator = msg_cur_ptr;
                break;

            case WPS_ATTR_R_HASH1:
                if (WPS_HASH_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->r_hash1 = msg_cur_ptr;
                break;
            case WPS_ATTR_R_HASH2:
                if (WPS_HASH_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->r_hash2 = msg_cur_ptr;
                break;
            case WPS_ATTR_WPS_STATE:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->wps_state = msg_cur_ptr;
                break;
            case WPS_ATTR_SEL_REG:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->sel_reg = msg_cur_ptr;
                break;
            case WPS_ATTR_SEL_REG_CONFIG_METH:
                if (2 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->sel_reg_config_meth = msg_cur_ptr;
                break;
            case WPS_ATTR_RESPONSE_TYPE:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->res_type = msg_cur_ptr;
                break;
            case WPS_ATTR_R_SNONCE1:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->r_snonce1 = msg_cur_ptr;
                break;
            case WPS_ATTR_R_SNONCE2:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->r_snonce2 = msg_cur_ptr;
                break;
            case WPS_ATTR_KEY_WRAP_AUTH:
                if (WPS_KEY_WRAP_AUTH_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->key_wrap_authen = msg_cur_ptr;
                break;

            case WPS_ATTR_ENCRY_SETTINGS:
                if(WPS_AES_BYTE_BLOCK_SIZE > attr_len)
                {
                    return 0;
                }
                cur_attr_buff->encrypted_settings = msg_cur_ptr;
                cur_attr_buff->encrypted_settings_len = attr_len;
                break;
            case WPS_ATTR_MANUFACTURER:
                if(WPS_MAX_MANUFACT_LEN < attr_len)
                {
                    return 0;
                }
                cur_attr_buff->manufacturer = msg_cur_ptr;
                cur_attr_buff->manufacturer_len = (UWORD8) attr_len;
                break;
            case WPS_ATTR_MODEL_NAME:
                if(WPS_MAX_MODEL_NAME_LEN < attr_len)
                {
                    return 0;
                }
                cur_attr_buff->model_name = msg_cur_ptr;
                cur_attr_buff->model_name_len = (UWORD8) attr_len;
                break;
            case WPS_ATTR_MODEL_NUM:
                if(WPS_MAX_MODEL_NUM_LEN < attr_len)
                {
                    return 0;
                }
                cur_attr_buff->model_number = msg_cur_ptr;
                cur_attr_buff->model_number_len = (UWORD8) attr_len;
                break;
            case WPS_ATTR_SERIAL_NUM:
                if(WPS_MAX_SERIAL_NUM_LEN < attr_len)
                {
                    return 0;
                }
                cur_attr_buff->serial_number = msg_cur_ptr;
                cur_attr_buff->serial_number_len = (UWORD8) attr_len;
                break;
            case WPS_ATTR_DEV_NAME:
                if(WPS_MAX_DEVICE_NAME_LEN < attr_len)
                {
                    return 0;
                }
                cur_attr_buff->device_name = msg_cur_ptr;
                cur_attr_buff->device_name_len = (UWORD8) attr_len;
                break;
            case WPS_ATTR_CREDENTIAL:
                /*************************************************************/
                /* If already received maximum number of credential then     */
                /* ignore the current credential else store pointer to       */
                /* credential in the parsed attribute buffer credential array*/
                /*  and increment the received credential count              */
                /*************************************************************/
                if (WPS_MAX_CRED_CNT <= cur_attr_buff->cred_cnt)
                {
                    break;
                }
                cur_attr_buff->credential[cur_attr_buff->cred_cnt] =
                    msg_cur_ptr;
                cur_attr_buff->credential_len[cur_attr_buff->cred_cnt] =
                    attr_len;
                cur_attr_buff->cred_cnt++;
                break;
            case WPS_ATTR_VENDOR_EXTENSION:
                if(WPS_MIN_VEN_EXTN_LEN > attr_len)
                    return 0;

                if((msg_cur_ptr[0] != WFA_VEN_EXTN_ID_BYTE0) ||
                   (msg_cur_ptr[1] != WFA_VEN_EXTN_ID_BYTE1) ||
                   (msg_cur_ptr[2] != WFA_VEN_EXTN_ID_BYTE2))
                   break;

                cur_attr_buff->wfa_vendor = (msg_cur_ptr + 3);
                wps_parse_wsc_subelem_enr(cur_attr_buff, (attr_len - 3));
                break;
#ifdef MAC_P2P
            case WPS_ATTR_REQ_DEV_TYPE:
                if (WPS_REQ_DEV_TYPE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->req_dev_type = msg_cur_ptr;
                break;
#endif /* MAC_P2P */
            default:
                break;
            } /* switch (attr_type) */
            msg_cur_ptr += attr_len;
        } /* while (msg_cur_ptr < msg_end_ptr) */
        return msg_cnt;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_parse_creden                                      */
/*                                                                           */
/*  Description      : This function parses an array of credentials and      */
/*                     passes the parsed credentials one by one to the       */
/*                     system software through the function provided by the  */
/*                     system software                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Array of pointers to structures where attributes   */
/*                        for each message/Information element are stored.   */
/*                     3) length of the message/Information element to be    */
/*                        parsed                                             */
/*                     4) Maximum number of concatenated message/Information */
/*                        elements to be parsed.                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function parses each credential for the mandatory*/
/*                     attributes. If all the mandatory attributes are       */
/*                     present then send credential to system software.      */
/*                                                                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_parse_creden(wps_enrollee_t* wps_enrollee, UWORD8** cred,
                          UWORD16* cred_len, UWORD8 cred_cnt)
{
    UWORD8 *curr_cred_msg;
    UWORD8 *curr_cred_msg_end;
    UWORD16 curr_cred_len;
    wps_cred_t parse_cred;
    UWORD16 expected_attr;
    UWORD8  cnt;
    BOOL_T valid_cred[WPS_MAX_CRED_CNT] = {BFALSE};

    for(cnt =0; cnt < cred_cnt; cnt++)
        valid_cred[cnt] = BTRUE;

    for(cnt =0; cnt < cred_cnt; cnt++)
    {
        UWORD8 *attr_ptr;
        UWORD8  key_cnt = 0;
        BOOL_T  net_key_idx_present = BFALSE;

        curr_cred_msg = cred[cnt];
        curr_cred_len = cred_len[cnt];

        {
            UWORD16 i = 0;

            DBG_PRINT("WPS-Rx-Cred[%d]=%d:\n\r", cnt, curr_cred_len);

            for(i=0; i < curr_cred_len;i++)
            {
                DBG_PRINT("0x%.2x, ", curr_cred_msg[i]);
            }
            DBG_PRINT("\n\r");
        }

        if((WPS_TLV_MIN_LEN > curr_cred_len) || (NULL == curr_cred_msg))
        {
            valid_cred[cnt] = BFALSE;
            continue;
        }
        curr_cred_msg_end = curr_cred_msg + curr_cred_len;

        /*********************************************************************/
        /* Check for RF band, if present and has no band common with the STA */
        /* RF band then discard the current credential                       */
        /*********************************************************************/
        attr_ptr = wps_find_attr(curr_cred_msg, curr_cred_msg_end,
                                 WPS_ATTR_RF_BANDS);
        /*********************************************************************/
        /* Check if there is RF band attribute is correct, if not then       */
        /* continue with credential processing else process RF band attribute*/
        /*********************************************************************/
        if((curr_cred_msg_end - attr_ptr) >=  (WPS_TLV_DATA_OFF_SET + 1))
        {
            UWORD16 attr_len;
            UWORD8  rf_bands;
            attr_ptr += 2;
            attr_len      = GET_U16_BE(attr_ptr);
            if(1 != attr_len)
            {
                valid_cred[cnt] = BFALSE;
                continue;
            }
            attr_ptr += 2;
            rf_bands = *attr_ptr;
            /*********************************************************************/
            /* Check if there is any common RF band between RF bands present in  */
            /* credential and RF bands supported by STA if not then discard the  */
            /* current credential                                                */
            /*********************************************************************/
            if(0 == (rf_bands & wps_enrollee->config_ptr->rf_bands))
            {
                valid_cred[cnt] = BFALSE;
                continue;
            }
        }

        memset(&parse_cred, 0, sizeof(wps_cred_t));
        expected_attr = WPS_ATTR_NET_INDEX;

        /*********************************************************************/
        /* Parse the received message                                        */
        /*********************************************************************/
        while (curr_cred_msg < curr_cred_msg_end)
        {
            UWORD16 attr_type, attr_len;
            if(0 == expected_attr)
            {
                break;
            }
            /*****************************************************************/
            /* If there is no scope for another Attribute then its an        */
            /* incorrect credential. Also if length of the attribute is more */
            /* than the remaining credential length then also credential is  */
            /* incorrect. Proceed to processing of next credential attribute */
            /*****************************************************************/
            if (curr_cred_msg_end - curr_cred_msg < WPS_TLV_MIN_LEN)
            {
                valid_cred[cnt] = BFALSE;
                break;
            }
            attr_type     = GET_U16_BE(curr_cred_msg);
            curr_cred_msg += 2;
            attr_len      = GET_U16_BE(curr_cred_msg);
            curr_cred_msg += 2;
            if((attr_len > curr_cred_msg_end - curr_cred_msg) ||
               (expected_attr != attr_type))
            {
                valid_cred[cnt] = BFALSE;
                break;
            }


            /*****************************************************************/
            /* Attribute is same as expected attribute.Do the validity checks*/
            /* if check passes then copy the attribute to the corresponding  */
            /* field in parsed credential buffer. if validity check fails    */
            /* return FAILURE. If already received MAX_NW_KEY_PER_CRED number*/
            /* of keys then don't store the remaining keys                   */
            /*****************************************************************/
            switch (expected_attr)
            {
                case WPS_ATTR_NET_INDEX:
                    if (1 != attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    expected_attr = WPS_ATTR_SSID;
                    break;
                case WPS_ATTR_SSID:
                    if (MAX_ESSID_LEN < attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    memcpy(parse_cred.ssid, curr_cred_msg, attr_len);
                    parse_cred.ssid_len = (UWORD8) attr_len;
                    expected_attr = WPS_ATTR_AUTH_TYPE;
                    break;
                case WPS_ATTR_AUTH_TYPE:
                    if (2 != attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    parse_cred.auth_type = GET_U16_BE(curr_cred_msg);
                    expected_attr = WPS_ATTR_ENCRY_TYPE;
                    break;
                case WPS_ATTR_ENCRY_TYPE:
                    if (2 != attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    parse_cred.enc_type = GET_U16_BE(curr_cred_msg);
                    curr_cred_msg += attr_len;
                    attr_len = 0;
                    if (curr_cred_msg_end - curr_cred_msg < WPS_TLV_MIN_LEN)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    attr_type = GET_U16_BE(curr_cred_msg);
                    if((WPS_ATTR_NET_KEY_INDEX != attr_type) &&
                        (WPS_ATTR_NET_KEY != attr_type))
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    expected_attr = attr_type;
                    break;
                case WPS_ATTR_NET_KEY_INDEX:
                    if (1 != attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    net_key_idx_present = BTRUE;
                    if(MAX_NW_KEY_PER_CRED > key_cnt)
                    {
                        parse_cred.net_key_index[key_cnt] = *curr_cred_msg;
                    }
                    expected_attr = WPS_ATTR_NET_KEY;
                    break;

                case WPS_ATTR_NET_KEY:
                    if (MAX_NET_KEY_LEN < attr_len)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    if(MAX_NW_KEY_PER_CRED > key_cnt)
                    {
                        memcpy(parse_cred.net_key[key_cnt], curr_cred_msg,
                            attr_len);
                        parse_cred.net_key_len[key_cnt] = (UWORD8)attr_len;
                    }
                    expected_attr = WPS_ATTR_MAC_ADDR;
                    break;
                case WPS_ATTR_MAC_ADDR:
                    if (WPS_MAC_ADDR_LEN != attr_len)
                    {
                        return FAILURE;
                    }
                    if(MAX_NW_KEY_PER_CRED > key_cnt)
                    {
                        /*****************************************************/
                        /* For Version2 capable registrar the MAC address    */
                        /* should match                                      */
                        /*****************************************************/
                        if((BTRUE == wps_enrollee->ver2_cap_ap) &&
                           (0 != memcmp(wps_enrollee->config_ptr->mac_address,
                                        curr_cred_msg, WPS_MAC_ADDR_LEN)))
                        {
                            wps_enrollee->config_error =
                                ROGUE_ACTIVITY_SUSPECTED;
                            valid_cred[cnt] = BFALSE;
                            break;
                        }
                    }
                    curr_cred_msg += attr_len;
                    key_cnt++;
                    /*********************************************************/
                    /* All the required attributes have been received after  */
                    /* this attributes starting from network key index can   */
                    /* repeat. So check if Network Key index is present in   */
                    /* remaining message, if so then check if it was received*/
                    /* earlier. if it was received earlier also then process */
                    /* it if not then its an error.                          */
                    /*********************************************************/
                    curr_cred_msg = wps_find_attr(curr_cred_msg,
                                                  curr_cred_msg_end,
                                                  WPS_ATTR_NET_KEY_INDEX);
                    /*********************************************************/
                    /* If new current pointer is greater than end pointer    */
                    /* credential is incorrect.                              */
                    /*********************************************************/
                    if(curr_cred_msg > curr_cred_msg_end)
                    {
                        valid_cred[cnt] = BFALSE;
                        break;
                    }
                    attr_len      = 0;
                    expected_attr = WPS_ATTR_NET_KEY_INDEX;

                    /*********************************************************/
                    /* If Key index has not been parsed and is present after */
                    /* MAC address attribute, then parse the same            */
                    /*********************************************************/
                    if((BFALSE == net_key_idx_present) &&
                       (curr_cred_msg != curr_cred_msg_end))
                    {
                        break;
                    }
                default:
                    break;
            } /* switch (expected_attr) */

            if(BFALSE == valid_cred[cnt])
                break; /* while (curr_cred_msg < curr_cred_msg_end) */

            curr_cred_msg += attr_len;
        } /* while (curr_cred_msg < curr_cred_msg_end) */

        if(BFALSE == valid_cred[cnt])
            continue; /* for(cnt =0; cnt < cred_cnt; cnt++) */

        /*********************************************************************/
        /* if WPS_ATTR_NET_KEY_INDEX or WPS_ATTR_NET_KEY is not the expected */
        /* attribute then one mandatory attribute has not been received,     */
        /* return FAILURE.                                                   */
        /*********************************************************************/
        if((WPS_ATTR_NET_KEY_INDEX != expected_attr) &&
           (WPS_ATTR_NET_KEY != expected_attr))
        {
            return FAILURE;
        }
        /*********************************************************************/
        /* Credential is parsed correctly, send it to the System SW          */
        /*********************************************************************/
        valid_cred[cnt] = rec_wps_cred(&parse_cred, wps_enrollee->ver2_cap_ap);
    } /* for(cnt =0; cnt < cred_cnt; cnt++) */
    for(cnt =0; cnt < cred_cnt; cnt++)
        if(BTRUE == valid_cred[cnt])
            return SUCCESS;

    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_find_attr                                         */
/*                                                                           */
/*  Description      : This function checks the input message for a given    */
/*                     attribute                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to message location to start search        */
/*                     2) Pointer to message location where search should end*/
/*                     3) Attribute ID to be searched                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Checks the input message for a input attribute ID and */
/*                     if attribute is present then return the pointer to the*/
/*                     attribute, if attribute not present but message is    */
/*                     correct then return end_ptr else return end_ptr+1     */
/*                                                                           */
/*  Outputs          : Location of the attribute in the message              */
/*  Returns          : Pointer to the attribute ID found or end_ptr+1 if     */
/*                     attribute not found or end_ptr if message encoded     */
/*                     incorrectly                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         12 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD8* wps_find_attr(UWORD8* st_ptr, UWORD8* end_ptr, UWORD16 attr_id)
{

    while (st_ptr < end_ptr)
    {
        /*********************************************************************/
        /* If there is no scope for another Attribute then its an incorrect  */
        /* message. Also if length of the attribute is more than the         */
        /* remaining message length then also message is incorrect. Return   */
        /* end_ptr +1 to indicate failure.                                   */
        /*********************************************************************/
        if (end_ptr < (st_ptr + WPS_TLV_MIN_LEN))
        {
            return (end_ptr+1);
        }
        /*********************************************************************/
        /* If Attribute ID is same as the desired Attribute ID then return   */
        /* pointer to this attribute                                         */
        /*********************************************************************/
        if(GET_U16_BE(st_ptr) == attr_id)
        {
            return st_ptr;
        }
        /*********************************************************************/
        /* Read length of the attribute and increment the message pointer by */
        /* attribute ID field length (2) + attribute length field length (2)+*/
        /* attribute length. If Message pointer is greater than the end      */
        /* pointer then it indicate that the Attribute read was incorrect,   */
        /* return end_ptr + 1 to indicate failure                            */
        /*********************************************************************/
        st_ptr += 2;
        st_ptr += (UWORD16)GET_U16_BE(st_ptr);
        st_ptr += 2;
        if(st_ptr > end_ptr)
        {
            return (end_ptr+1);
        }
    }
    /*************************************************************************/
    /* Attribute not found hence return end_ptr                              */
    /*************************************************************************/
    return end_ptr;
}
#endif /* INT_WPS_ENR_SUPP */

