/*****************************************************************************/
/*                                                                           */
/*                           Ittiam WPS SOFTWARE                             */
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
/*  File Name         : wps_prot.c                                           */
/*                                                                           */
/*  Description       : This file contains the common WPS Protocol related   */
/*                      functions used in WPS Library                        */
/*                                                                           */
/*  List of Functions : wps_parse_attri                                      */
/*                      wps_verify_auth_attr                                 */
/*                      wps_decrypt_encrypted_setting                        */
/*                      wps_verify_key_wrap_auth                             */
/*                      wps_verify_pin_snonce                                */
/*                      wps_gen_secr_keys                                    */
/*                      wps_compute_auth_attr                                */
/*                      wps_gen_pin_hash                                     */
/*                      wps_put_key_wrap_auth                                */
/*                      wps_gen_encr_settings                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "aes_cbc.h"
#include "hmac_sha256.h"
#include "wps_key.h"
#include "wps_prot.h"
#include "wps_common.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

const UWORD8 g_eapol_snap_hdr[SNAP_HEADER_LEN] =
{
    0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E
};

const UWORD8 g_eap_wps_vid_vt[EAP_WPS_VID_VT_LEN] =
{
    0x00, 0x37, 0x2A, 0x00, 0x00, 0x00, 0x01
};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static UWORD16 wps_put_key_wrap_auth(UWORD8 *auth_key, UWORD8 *msg_ptr,
                                     UWORD16 msg_len);


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
/*                                                                           */
/*  Processing       : This function processes WPS TLV credentials from      */
/*                     multiple WPS entities (Registrar/Enrollee) and places */
/*                     attributes from each entuty in different user supplied*/
/*                     attribute structure. On finding Version Number        */
/*                     attribute again, the attribute structure is updated to*/
/*                     next one                                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
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

    /* Parse the received message */
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
            case WPS_ATTR_MSG_TYPE:
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
                cur_attr_buff->uuid = msg_cur_ptr;
                break;

            case WPS_ATTR_MAC_ADDR:
                if (MAC_ADDRESS_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->mac_addr = msg_cur_ptr;
                break;

            case WPS_ATTR_PUBLIC_KEY:
                if (WPS_PUBLIC_KEY_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->pub_key = msg_cur_ptr;
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
            case WPS_ATTR_REQUEST_TYPE:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->req_type = msg_cur_ptr;
                break;
            case WPS_ATTR_RESPONSE_TYPE:
                if (1 != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->rsp_type = msg_cur_ptr;
                break;
#ifdef INT_WPS_REG_SUPP
            case WPS_ATTR_E_HASH1:
                if (WPS_HASH_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->e_hash1 = msg_cur_ptr;
                break;
            case WPS_ATTR_E_HASH2:
                if (WPS_HASH_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->e_hash2 = msg_cur_ptr;
                break;
            case WPS_ATTR_E_SNONCE1:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->e_snonce1 = msg_cur_ptr;
                break;
            case WPS_ATTR_E_SNONCE2:
                if (WPS_NONCE_LEN != attr_len)
                {
                    return 0;
                }
                cur_attr_buff->e_snonce2 = msg_cur_ptr;
                break;
#else  /* INT_WPS_REG_SUPP */
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
#endif /* INT_WPS_REG_SUPP */
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
            case WPS_ATTR_VENDOR_EXTENSION:
                if(WPS_MIN_VEN_EXTN_LEN > attr_len)
                    return 0;

                if((msg_cur_ptr[0] != WFA_VEN_EXTN_ID_BYTE0) ||
                   (msg_cur_ptr[1] != WFA_VEN_EXTN_ID_BYTE1) ||
                   (msg_cur_ptr[2] != WFA_VEN_EXTN_ID_BYTE2))
                   break;

                cur_attr_buff->wfa_vendor = (msg_cur_ptr + 3);
                wps_parse_wsc_subelem_reg(cur_attr_buff, (attr_len - 3));
                break;
// 20120709 caisf add, merged ittiam mac v1.2 code
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
/*  Function Name    : wps_verify_auth_attr                                  */
/*                                                                           */
/*  Description      : This function verifies whether the received Mx message*/
/*                     has correct authenticator attribute or not.           */
/*                                                                           */
/*  Inputs           : 1) Pointer to received WSC Message                    */
/*                     2) Length of received WSC Message                     */
/*                     3) Pointer to transmit WSC Message                    */
/*                     4) Length of the transmit WSC Message                 */
/*                     5) Pointer to Authentication Key                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : Check if Key Wrap Authenticator is present at the end */
/*                     of the Mx Message. Checks if transmitted message is   */
/*                     present. Generates the Authenticator and compare it   */
/*                     with the value received. If any checks fails return   */
/*                     FAILURE else return SUCCESS                           */
/*                                                                           */
/*  Outputs          : Result of Authenticator check                         */
/*                                                                           */
/*  Returns          : SUCCESS/FAILURE                                       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifndef OS_LINUX_CSL_TYPE
#pragma Otime
#endif /* OS_LINUX_CSL_TYPE */

STATUS_T wps_verify_auth_attr(UWORD8 *rx_msg_ptr, UWORD16 rx_msg_len,
                              UWORD8 *tx_msg_ptr, UWORD16 tx_msg_len,
                              UWORD8 *auth_key)
{
    UWORD8  temp_buff[SHA_256_HASH_LEN];
    UWORD8* msg_ptr[2];
    UWORD16 msg_len[2];

    /*************************************************************************/
    /* Transmit message pointer is NULL or if the Transmit message len is    */
    /* Zero or if Receive Message pointer is NULL or if the Receive message  */
    /* length is less than WPS Authenticator Attribute length, then exit     */
    /*************************************************************************/
    if((NULL == rx_msg_ptr) || (0 == tx_msg_len) || (NULL == tx_msg_ptr) ||
       ((4 + WPS_AUTHENTICATOR_LEN) > rx_msg_len))
    {
        return FAILURE;
    }

    /* Last attribute in the received message should be authenticator        */
    /* attribute                                                             */
    if(WPS_ATTR_AUTHENTICATOR !=  GET_U16_BE((rx_msg_ptr + rx_msg_len -
                                              WPS_AUTHENTICATOR_LEN - 4)))
    {
        return FAILURE;
    }

    /* Initialize the message pointers and message length for Hashing        */
    msg_ptr[0] = tx_msg_ptr;
    msg_ptr[1] = rx_msg_ptr;
    msg_len[0] = tx_msg_len;
    msg_len[1] = rx_msg_len - 4 - WPS_AUTHENTICATOR_LEN;

    /* Hash the concatenation of transmitted and received message            */
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr, msg_len, 2, temp_buff);

    /*************************************************************************/
    /* The first 64 bits of the TX/RX Hash computed must equal to the        */
    /* authentictor attribute included                                       */
    /*************************************************************************/
    if(0 == memcmp((rx_msg_ptr + rx_msg_len - WPS_AUTHENTICATOR_LEN),
                   temp_buff, WPS_AUTHENTICATOR_LEN))
    {
        return SUCCESS;
    }

    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_decrypt_encrypted_setting                         */
/*                                                                           */
/*  Description      : This function decrypts the encryption setting         */
/*                     attribute and returns the decrypted data.             */
/*                                                                           */
/*  Inputs           : 1) Pointer to Key Wrap Key                            */
/*                     2) Array to the encrypted data                        */
/*                     3) Encrypted data length                              */
/*                     4) Pointer to buffer where decrypted data needs to be */
/*                        stored                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : Do the encrypted data length check if fails return    */
/*                     error code. Decrypt the data and remove pad. If       */
/*                     invalid pad or pad length received then return error  */
/*                     code else return decrypted data length without pad    */
/*                                                                           */
/*  Outputs          : Decrypted data along with length or error code        */
/*                                                                           */
/*  Returns          : Error code or decrypted data length                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

WORD16 wps_decrypt_encrypted_setting(UWORD8  *key_wrap_key,
                                     UWORD8  *encry_data,
                                     UWORD16 encry_data_len,
                                     UWORD8  *decry_data)
{
    UWORD8  *data_ptr;
    UWORD16 data_len;
    UWORD8 pad, cnt;

    if((2*WPS_AES_BYTE_BLOCK_SIZE > encry_data_len ) ||
       (0 != (encry_data_len % WPS_AES_BYTE_BLOCK_SIZE)))
    {
        return ERR_INVALID_ENCR_DATA_LEN;
    }

    /*************************************************************************/
    /* The Encrypted data len passed is len including IV, hence adjust this  */
    /* before doing decryption                                               */
    /*************************************************************************/
    data_len = encry_data_len - WPS_AES_BYTE_BLOCK_SIZE;

    /* Decrypt the Encrypted data */
    wps_aes_decrypt(key_wrap_key, encry_data, decry_data, data_len);

    /* Get the Pad value added before encryption */
    data_ptr = decry_data + data_len -1;
    pad = *data_ptr--;

    /*************************************************************************/
    /* Pad length should be less than WPS_AES_BYTE_BLOCK_SIZE. As per PKCS#5 */
    /* v2.0, Pad and pad length have same value. Check for pad length and    */
    /* byte of pad. If check fails return Pad error                          */
    /*************************************************************************/
    if(pad > WPS_AES_BYTE_BLOCK_SIZE)
    {
        return ERR_INVALID_PAD;
    }
    for (cnt = 1; cnt < pad; cnt++)
    {
        if (*data_ptr-- != pad)
        {
            return ERR_INVALID_PAD;
        }
    }
    return (data_len - pad);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_verify_key_wrap_auth                              */
/*                                                                           */
/*  Description      : This function verifies the whether the Key Wrap       */
/*                     Authenticator present in the received message is      */
/*                     correct or not                                        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS Authentication Key                  */
/*                     2) Pointer to the decrypted encrypted setting data    */
/*                     3) Length of decrypted encrypted setting data         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : Check if Key Wrap Authenticator is present at the end */
/*                     of the input data. If not then return FAILURE.        */
/*                     Generate Key Warp Authenticator and compare it with   */
/*                     the value received. If comparison fails return        */
/*                     FAILURE else return SUCCESS                           */
/*                                                                           */
/*  Outputs          : Result of Key Way Authenticator check                 */
/*                                                                           */
/*  Returns          : SUCCESS/FAILURE                                       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_verify_key_wrap_auth(UWORD8 *auth_key, UWORD8 *data,
                                  UWORD16 data_len)
{
    UWORD8 temp_buff[SHA_256_HASH_LEN];

    if(WPS_ATTR_KEY_WRAP_AUTH != GET_U16_BE((data + data_len -
        WPS_KEY_WRAP_AUTH_LEN-4)))
    {
        return FAILURE;
    }
    data_len -= (WPS_KEY_WRAP_AUTH_LEN + 4);

    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, &data, &data_len, 1,
                temp_buff);

    if(0 == memcmp(data + data_len + 4, temp_buff,
                   WPS_KEY_WRAP_AUTH_LEN))
    {
       return SUCCESS;
    }
    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_verify_pin_snonce                                 */
/*                                                                           */
/*  Description      : This function verifies the received E/R-SNonce1 or    */
/*                     E/R-SNonce2                                           */
/*                                                                           */
/*  Inputs           : 1) Pointer to Enrollee`s Public Key, PKe              */
/*                     2) Pointer to Registrar`s Public Key, PKr             */
/*                     3) Pointer to PIN Nonce, E-S1 or E-S2 or R-S1 or R-S2 */
/*                     4) Pointer to Rxed Hash, E/R-Hash1/2                  */
/*                     5) Pointer to the Computed to PSK1 or PSK2            */
/*                     6) Pointer to WPS Authentication Key                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) Hash the concatenated PIN Nonce, PSK, PKe and PKr  */
/*                        to generate PIN Hash                               */
/*                     2) If this computed PIN Hash is not same as the PIN   */
/*                        Hash passed, then return FAILURE                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS/FAILURE                                       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_verify_pin_snonce(UWORD8 *enr_pub_key, UWORD8 *reg_pub_key,
                               UWORD8 *s_nonce_ptr, UWORD8 *hash_ptr,
                               UWORD8 *psk_ptr, UWORD8 *auth_key)
{
    UWORD8     temp_buff[SHA_256_HASH_LEN] = {0};
    UWORD8     *msg_ptr[4];
    UWORD16    msg_len[4];

    /* If any of the input pointers are NULL then exit */
    if((NULL == s_nonce_ptr) || (NULL == hash_ptr) || (NULL == psk_ptr))
        return FAILURE;

    msg_ptr[0] = s_nonce_ptr;
    msg_ptr[1] = psk_ptr;
    msg_ptr[2] = enr_pub_key;
    msg_ptr[3] = reg_pub_key;
    msg_len[0] = WPS_NONCE_LEN;
    msg_len[1] = WPS_PSK_LEN;
    msg_len[2] = WPS_PUBLIC_KEY_LEN;
    msg_len[3] = WPS_PUBLIC_KEY_LEN;

    /* Compute HMAC-SHA256(E/R-S1/2 || PSK1/2 || PKe || PKr) */
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN,
                msg_ptr, msg_len, 4, temp_buff);

    /* Check if the computed hash matches with the hash received */
    if(0 == memcmp(hash_ptr, temp_buff, WPS_HASH_LEN))
    {
        return SUCCESS;
    }
    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_secr_keys                                     */
/*                                                                           */
/*  Description      : This function generates the secret Keys AuthKey and   */
/*                     KeyWrapKey                                            */
/*                                                                           */
/*  Inputs           : 1) Pointer to received Public Key (PKe or PKr)        */
/*                     2) Pointer to Public Key buffer                       */
/*                     3) Pointer to Enrollee`s Nonce, N1                    */
/*                     4) Pointer to Registrar`s Nonce, N2                   */
/*                     5) Pointer to Enrollee`s MAC address                  */
/*                     6) Pointer to WPS Authentication Key buffer           */
/*                     7) Pointer to WPS Key Wrap Key buffer                 */
/*                     8) Pointer to WPS Private Key                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates the secret Keys AuthKey and   */
/*                     KeyWrapKey using the received Public Key and DH Secret*/
/*                     Key (A or B)                                          */
/*                                                                           */
/*  Outputs          : Auth Key and Key Wrap Key                             */
/*                                                                           */
/*  Returns          : SUCCESS/FAILURE                                       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_gen_secr_keys(UWORD8  *rx_pub_key_ptr,
                           UWORD32 *pub_key_buf,
                           UWORD8  *enr_nonce,
                           UWORD8  *reg_nonce,
                           UWORD8  *enr_mac_addr,
                           UWORD8  *auth_key,
                           UWORD8  *key_wrap_key,
                           long_num_struct_t *priv_key_ptr)
{
    UWORD8 cnt;
    UWORD32* temp32_ptr1;

    /*************************************************************************/
    /* Copy the Received Public Key into the Pub Key buffer such that least  */
    /* significant word is copied at a lower memory location in big endian   */
    /* word format                                                           */
    /*************************************************************************/
    temp32_ptr1 = pub_key_buf + WPS_DH_PRIME_LEN_WORD32 - 1;
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        *temp32_ptr1-- = ((UWORD32)((UWORD32) rx_pub_key_ptr[0]) << 24) |
                         ((UWORD32)((UWORD32) rx_pub_key_ptr[1]) << 16) |
                         ((UWORD32)((UWORD32) rx_pub_key_ptr[2]) <<  8) |
                         ((UWORD32)((UWORD32) rx_pub_key_ptr[3]));
        rx_pub_key_ptr += 4;
    }
    rx_pub_key_ptr -= WPS_DH_PRIME_LEN_WORD32*4;

    if(FAILURE == wps_gen_keys((UWORD8 *)pub_key_buf, enr_nonce, reg_nonce,
                               enr_mac_addr, auth_key, key_wrap_key,
                               priv_key_ptr))
        return FAILURE;

    /*************************************************************************/
    /* Copy the received Public Key from rx_pub_key_ptr to Public Key Buffer */
    /* provided, with most significant byte at the first location. Source is */
    /* already in desired format.                                            */
    /*************************************************************************/
    memcpy((UWORD8 *)pub_key_buf, rx_pub_key_ptr, WPS_DH_PRIME_LEN_BYTE);

    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_compute_auth_attr                                 */
/*                                                                           */
/*  Description      : This function generates the Authenticator attribute   */
/*                                                                           */
/*  Inputs           : 1) Pointer to received WSC Message                    */
/*                     2) Length of received WSC Message                     */
/*                     3) Pointer to transmit WSC Message                    */
/*                     4) Length of the transmit WSC Message                 */
/*                     5) Pointer to WPS Authentication Key                  */
/*                     6) Pointer buffer where authenticator attribute is to */
/*                        be stored                                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates the Authenticator for a given */
/*                     message at the given input buffer                     */
/*                                                                           */
/*  Outputs          : Authenticator Attribute                               */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_compute_auth_attr(UWORD8 *rx_msg_ptr, UWORD16 rx_msg_len,
                           UWORD8 *tx_msg_ptr, UWORD16 tx_msg_len,
                           UWORD8 *auth_key,   UWORD8  *auth_attr)
{
    UWORD8* msg_ptr_arr[2];
    UWORD16 msg_len_arr[2];

    msg_ptr_arr[0] = rx_msg_ptr;
    msg_ptr_arr[1] = tx_msg_ptr;
    msg_len_arr[0] = rx_msg_len;
    msg_len_arr[1] = tx_msg_len;

    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr, msg_len_arr, 2,
                auth_attr);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_pin_hash                                      */
/*                                                                           */
/*  Description      : This function generates the PSK1, PSK2, E/R-Hash1 and */
/*                     E/R-Hash2                                             */
/*                                                                           */
/*  Inputs           : 1) WPS Protocol Type (PIN/PBC)                        */
/*                     2) Pointer to Device PIN                              */
/*                     3) Length of the Device PIN                           */
/*                     4) Pointer to location where E/R-hash1 and E/R-Hash2  */
/*                        attribute should be stored                         */
/*                     5) Attribute value for HASH1 (E-Hash1 or R-Hash1)     */
/*                     6) Attribute value for HASH2 (E-Hash2 or R-Hash2)     */
/*                     7) Pointer to WPS Authenication Key                   */
/*                     8) Pointer to PSK1                                    */
/*                     9) Pointer to PSK2                                    */
/*                     10) Pointer to 1st Half PIN Nonce, E/R-S1             */
/*                     11) Pointer to 2nd Half PIN Nonce, E/R-S2             */
/*                     12) Pointer to Registrar`s Public Key, PKr            */
/*                     13) Pointer to Enrolee`s Public Key, PKe              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates the PSK1, PSK2, E/R-Hash1 and */
/*                     E/R-Hash2 and puts E/R-Hash1 and E/R-Hash2 attributes */
/*                     in the input buffer.                                  */
/*                                                                           */
/*  Outputs          : PSK1, PSK2, E/R-Hash1, E/R-Hash2                      */
/*                                                                           */
/*  Returns          : Total length of E/R-Hash1 and E/R-Hash2 Attributes    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

UWORD16 wps_gen_pin_hash(WPS_PROT_TYPE_T prot_type, UWORD8 *dev_pin,
                         UWORD8 dev_pin_len, UWORD8 *buf_ptr,
                         WPS_TLV_ATTR_ID_T hash1, WPS_TLV_ATTR_ID_T hash2,
                         UWORD8 *auth_key, UWORD8 *psk1, UWORD8 *psk2,
                         UWORD8 *snonce1, UWORD8 *snonce2,
                         UWORD8 *reg_pub_key, UWORD8 *enr_pub_key)
{
    UWORD8  dev_pass_len = 0;
    UWORD16 index = 0;
    UWORD8  dev_pass[MAX_WPS_PASS_LEN];
    UWORD16 msg_len_arr[4];
    UWORD8  *msg_ptr_arr[4];

    if(PBC == prot_type)
    {
        memset(dev_pass, '0',WPS_PBC_PASS_LEN);
        dev_pass_len = WPS_PBC_PASS_LEN;
    }
    else
    {
        dev_pass_len = dev_pin_len;
        memcpy(dev_pass, dev_pin, dev_pass_len);
    }

    /*************************************************************************/
    /* PSK1=first 128 bits of HMACAuthKey(1st half of DevicePassword)        */
    /* Use input message buffer for temporary storing hash. it will have     */
    /* enough space for atleast 2 hash ad E-Hash1 and E-hash2 need to be     */
    /* stored there.                                                         */
    /*************************************************************************/
    msg_ptr_arr[0] = dev_pass;
    msg_len_arr[0] = (dev_pass_len + 1) >> 1;
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr, msg_len_arr, 1,
                buf_ptr);
    memcpy(psk1, buf_ptr, WPS_PSK_LEN);

    /*************************************************************************/
    /* PSK2=first 128 bits of HMACAuthKey(2nd half of DevicePassword)        */
    /* Use input message buffer for temporary storing hash. it will have     */
    /* enough space for atleast 2 hash ad E-Hash1 and E-hash2 need to be     */
    /* stored there. msg_len_arr[0] already have first half length of device */
    /* password.                                                             */
    /*************************************************************************/
    msg_ptr_arr[0] = dev_pass + msg_len_arr[0];
    msg_len_arr[0] = (dev_pass_len) >> 1;
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr, msg_len_arr, 1,
                buf_ptr);
    memcpy(psk2, buf_ptr, WPS_PSK_LEN);

    /* Generate Secret PIN Nonce E/R-SNonce1 and E/R-SNonce2                 */
    wps_get_rand_byte_array(snonce1, WPS_NONCE_LEN);
    wps_get_rand_byte_array(snonce2, WPS_NONCE_LEN);


    /*************************************************************************/
    /* Generate E/R-Hash1 and E/R-Hash2. Directly put the generated values in*/
    /* the corresponding attributes value fields                             */
    /*************************************************************************/
    PUT_U16_BE((buf_ptr + index), hash1);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_HASH_LEN);
    index += 2;


    /* Generate E/R-Hash1 = HMACAuthKey(E/R-S1 || PSK1 || PKE || PKR)        */
    msg_ptr_arr[0] = snonce1;
    msg_ptr_arr[1] = psk1;
    msg_ptr_arr[2] = enr_pub_key;
    msg_ptr_arr[3] = reg_pub_key;

    msg_len_arr[0] = WPS_NONCE_LEN;
    msg_len_arr[1] = WPS_PSK_LEN;
    msg_len_arr[2] = WPS_PUBLIC_KEY_LEN;
    msg_len_arr[3] = WPS_PUBLIC_KEY_LEN;
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 4, (buf_ptr + index));
    index += WPS_HASH_LEN;


    /*************************************************************************/
    /* Generate E/R-Hash2 = HMACAuthKey(E/R-S2 || PSK2 || PKE || PKR)        */
    /* Only S2 and PSK2 pointers needs to be assigned                        */
    /*************************************************************************/
    PUT_U16_BE((buf_ptr + index), hash2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_HASH_LEN);
    index += 2;
    msg_ptr_arr[0] = snonce2;
    msg_ptr_arr[1] = psk2;
    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr, msg_len_arr, 4,
                (buf_ptr + index));
    index += WPS_HASH_LEN;

    return index;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_put_key_wrap_auth                                 */
/*                                                                           */
/*  Description      : This function prepares the Key Wrap Authenticator     */
/*                                                                           */
/*  Inputs           : 1) Pointer to Authenticator Key                       */
/*                     2) Pointer to the input message.Key Wrap Authenticator*/
/*                        attribute is put at the end of data                */
/*                     3) Length of input message                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates the Key Wrap Authenticator for*/
/*                     a given message and puts the Key Wrap Authenticator   */
/*                     attribute at the end of the message                   */
/*                                                                           */
/*  Outputs          : Key Wrap Authenticator Attribute                      */
/*                                                                           */
/*  Returns          : Length of Key Wrap Authenticator Attribute            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static UWORD16 wps_put_key_wrap_auth(UWORD8 *auth_key, UWORD8 *msg_ptr,
                                     UWORD16 msg_len)
{
    UWORD16 index = 0;
    UWORD8 temp_buff[SHA_256_HASH_LEN];

    hmac_sha256(auth_key, WPS_AUTH_KEY_LEN, &msg_ptr, &msg_len,
                1, temp_buff);

    index += msg_len;
    PUT_U16_BE((msg_ptr + index), WPS_ATTR_KEY_WRAP_AUTH);
    index += 2;
    PUT_U16_BE((msg_ptr + index), WPS_KEY_WRAP_AUTH_LEN);
    index += 2;
    memcpy((msg_ptr + index), temp_buff, WPS_KEY_WRAP_AUTH_LEN);

    return(WPS_KEY_WRAP_AUTH_LEN + 4);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_encr_settings                                 */
/*                                                                           */
/*  Description      : This function prepares and puts the Encrypted Settings*/
/*                     Attribute for the given input attribute               */
/*                                                                           */
/*  Inputs           : 1) Pointer to the buffer where Encrypted Settings is  */
/*                        to be put                                          */
/*                     2) Pointer to the attribute value                     */
/*                     3) Attribute Type                                     */
/*                     4) Attribute Length                                   */
/*                     5) Flag to indicate if attribute is to be put         */
/*                     6) Pointer to WPS Authententicator Key                */
/*                     7) Pointer to WPS Key Wrap Key                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function prepares and puts the Encrypted Settings*/
/*                     Attribute for the given input attribute. Key Wrap     */
/*                     Authenticator is computed and added at the end of the */
/*                     Attribute and then AES Encrypted                      */
/*                                                                           */
/*  Outputs          : Encrypted Settings Attribute                          */
/*                                                                           */
/*  Returns          : Length of Encrypted Settings Attribute                */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

UWORD16 wps_gen_encr_settings(UWORD8  *buf_ptr,
                              UWORD8  *attr_val_ptr,
                              WPS_TLV_ATTR_ID_T attr_type,
                              UWORD16 attr_len,
                              BOOL_T  put_attr,
                              UWORD8  *auth_key,
                              UWORD8  *key_wrap_key)
{
    UWORD16 index   = 0;
    UWORD8  pad_len = 0;

    if((NULL == attr_val_ptr) || (NULL == buf_ptr) || (0 == attr_len))
        return 0;

    /*************************************************************************/
    /*    Encrypted Setting Structure                                        */
    /*************************************************************************/
    /*           0         |       1     |        2        |     3       |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |         Attribute ID          |            Length             |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |                     IV (1-4)                                  |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |                     IV (5-8)                                  |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |                     IV (9-12)                                 |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |                     IV (13-16)                                |   */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+   */
    /*   |                     Encrypted data...                             */
    /*   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                           */
    /*************************************************************************/

    /* Put Encrypted Settings ID and skip length field as it is unknown      */
    PUT_U16_BE((buf_ptr + index), WPS_ATTR_ENCRY_SETTINGS);

    /* Skip attribute ID and length fields */
    index += 4;

    /* Generate IV and store it at appropriate offset */
    wps_get_rand_byte_array((buf_ptr + index), WPS_AES_BYTE_BLOCK_SIZE);
    index += WPS_AES_BYTE_BLOCK_SIZE;

    /*************************************************************************/
    /* Put the passed input attribute in WPS TLV formate at given buffer    */
    /* offset if it is not put yet                                          */
    /*************************************************************************/
    if(BTRUE == put_attr)
    {
        PUT_U16_BE((buf_ptr + index), attr_type);
        index += 2;
        PUT_U16_BE((buf_ptr + index), attr_len);
        index += 2;
        memcpy((buf_ptr + index), attr_val_ptr, attr_len);
        index += attr_len;
    }
    else
        index += (attr_len);

    /*************************************************************************/
    /* Put Key Wrap Authenticator attribute at the end of Encrypted Settings */
    /* and calculate pad length. Add PKCS 5.0 pad                            */
    /*************************************************************************/
    index += wps_put_key_wrap_auth(auth_key,
                                   (buf_ptr + WPS_ENCR_SET_DATA_OFFSET),
                                   (index - WPS_ENCR_SET_DATA_OFFSET));
    pad_len = WPS_AES_BYTE_BLOCK_SIZE -
              ((index - WPS_ENCR_SET_DATA_OFFSET)% WPS_AES_BYTE_BLOCK_SIZE);
#ifndef MWLAN    
    memset((buf_ptr + index), pad_len, pad_len);
#else /* MWLAN */
    {
		UWORD16 i = 0;

		for(i = 0; i < pad_len; i++)
			*(buf_ptr + index + i) = pad_len;
	}
#endif /* MWLAN */
    index += pad_len;

    /*************************************************************************/
    /* Encrypt the Data portion of the Encrption Setting except the IV part  */
    /* Encrypted data will overwrite the original data                       */
    /*************************************************************************/
    wps_aes_encrypt(key_wrap_key, (buf_ptr + 4),
                    (index - WPS_ENCR_SET_DATA_OFFSET));

    /*************************************************************************/
    /* Length of Encrypted Settings attribute is now known, so put it in the */
    /* length field of the Encrypted Settings attribute                      */
    /*************************************************************************/
    PUT_U16_BE((buf_ptr + 2), (index - 4));

    return index;
}

#endif /* INT_WPS_REG_SUPP */
