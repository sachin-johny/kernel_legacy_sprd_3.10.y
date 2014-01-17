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
/*  File Name         : wpse_prot_tx.h                                       */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      the transmitter WPS registration protocol functions  */
/*                                                                           */
/*  List of Functions : wps_put_wsc_version                                  */
/*                      wps_put_wsc_msg_type                                 */
/*                      wps_put_enr_nonce                                    */
/*                      wps_put_reg_nonce                                    */
/*                      wps_put_config_err                                   */
/*                      wps_put_wsc_uuid_e                                   */
/*                      wps_put_wsc_mac_addr                                 */
/*                      wps_put_public_key                                   */
/*                      wps_put_auth_type_flag                               */
/*                      wps_put_encr_type_flag                               */
/*                      wps_put_conn_type_flag                               */
/*                      wps_put_cofig_meth                                   */
/*                      wps_put_wps_state                                    */
/*                      wps_put_manufacturer                                 */
/*                      wps_put_model_name                                   */
/*                      wps_put_model_num                                    */
/*                      wps_put_serial_num                                   */
/*                      wps_put_prim_dev_type                                */
/*                      wps_put_device_name                                  */
/*                      wps_put_rf_bands                                     */
/*                      wps_put_assoc_state                                  */
/*                      wps_put_dev_pass_id                                  */
/*                      wps_put_os_ver                                       */
/*                      wps_put_req_dev_type                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         08 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

#ifndef WPS_PROT_TX_H
#define WPS_PROT_TX_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_prot.h"
#include "wpse_prot_rx.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
#ifdef MAC_P2P
/* WPS Response Type */
typedef enum
{
    RSP_ENRO_INFO_ONLY          = 0x00,
    RSP_ENR_OPEN_802_1X         = 0x01,
    RSP_REGISTRAR               = 0x02,
    AP                          = 0x03
} WPS_RESPONSE_TYPE_T;
#endif /* MAC_P2P */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern STATUS_T wps_send_eapol_start(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_start_reg_msg_timer(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_frag_ack(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_eap_res_id(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_tx_eap_msg(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_nack_m2(wps_enrollee_t* wps_enrollee,
                             wps_rx_attri_t* rx_attr,
                             UWORD16 config_error);
extern STATUS_T wps_gen_secr_keys(wps_enrollee_t* wps_enrollee,
                                  UWORD8* pkr_ptr);
extern STATUS_T wps_send_done(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_m1(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_m3(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_m5_or_7(wps_enrollee_t* wps_enrollee,
                                 UWORD8 msg_type);
extern STATUS_T wps_send_wsc_ack(wps_enrollee_t* wps_enrollee);
extern STATUS_T wps_send_nack(wps_enrollee_t* wps_enrollee);

extern void wps_aes_encrypt(wps_enrollee_t* wps_enrollee,
                            UWORD8* data_ptr, UWORD16 data_len);
extern void wps_aes_decrypt(wps_enrollee_t* wps_enrollee,
                            UWORD8* data_inp_ptr, UWORD8* data_out_ptr,
                            UWORD16 data_len);
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* This function puts the WSC version number attribute at the location       */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD8* wps_put_wsc_version(UWORD8* buf_ptr)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_VERSION);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = WPS_VERSION;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the WSC request type attribute at the location         */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD8* wps_put_req_type(UWORD8* buf_ptr, WPS_REQUEST_TYPE_T req_type)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_REQUEST_TYPE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = req_type;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the WSC message type(value passed as input) attribute  */
/* at the location pointed by input pointer                                  */
/*****************************************************************************/
INLINE UWORD8* wps_put_wsc_msg_type(UWORD8* buf_ptr, UWORD8 msg_type)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_MEG_TYPE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = msg_type;
    return(buf_ptr);
}
/*****************************************************************************/
/* This function puts the WSC Enrollee nonce(value passed as input) attribute*/
/* at the location pointed by input pointer                                  */
/*****************************************************************************/
INLINE  UWORD8* wps_put_enr_nonce(UWORD8* buf_ptr, UWORD8* nonce_ptr)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_ENR_NONCE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_NONCE_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, nonce_ptr, WPS_NONCE_LEN);
    buf_ptr += WPS_NONCE_LEN;
    return(buf_ptr);
}
/*****************************************************************************/
/* This function puts the WSC Registrar nonce(value passed as input)         */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_reg_nonce(UWORD8* buf_ptr, UWORD8* nonce_ptr)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_REG_NONCE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_NONCE_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, nonce_ptr, WPS_NONCE_LEN);
    buf_ptr += WPS_NONCE_LEN;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the WSC Configuration error(value passed as input)     */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_config_err(UWORD8* buf_ptr, UWORD16 config_err)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_CONFIG_ERROR);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, config_err);
    buf_ptr += 2;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the UUID-E(value passed as input)  attribute at the    */
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_wsc_uuid_e(UWORD8* buf_ptr, UWORD8* uuid_e)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_UUID_E);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_UUID_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, uuid_e, WPS_UUID_LEN);
    buf_ptr += WPS_UUID_LEN;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the MAC-Address(value passed as input) attribute at the*/
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_wsc_mac_addr(UWORD8* buf_ptr, UWORD8* mac_addr)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_MAC_ADDR);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_MAC_ADDR_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, mac_addr, WPS_MAC_ADDR_LEN);
    buf_ptr += WPS_MAC_ADDR_LEN;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Public Key(value passed as input) attribute at the */
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_public_key(UWORD8* buf_ptr, UWORD8* pke)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_PUBLIC_KEY);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_PUBLIC_KEY_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, pke, WPS_PUBLIC_KEY_LEN);
    buf_ptr += WPS_PUBLIC_KEY_LEN;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Authentication Type flags(value passed as input)   */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_auth_type_flag(UWORD8* buf_ptr, UWORD16 auth_type_flag)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_AUTH_TYPE_FLAGS);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, auth_type_flag);
    buf_ptr += 2;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Encryption Type flags(value passed as input)       */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_encr_type_flag(UWORD8* buf_ptr, UWORD16 enc_type_flag)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_ENCRY_TYPE_FLAGS);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, enc_type_flag);
    buf_ptr += 2;
    return(buf_ptr);
}


/*****************************************************************************/
/* This function puts the Connection Type flags(value passed as input)       */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE  UWORD8* wps_put_conn_type_flag(UWORD8* buf_ptr, UWORD8 conn_type_flag)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_CONN_TYPE_FLAGS);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = conn_type_flag;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Config Methods(value passed as input) attribute at */
/* the location pointed by input pointer                                     */
/*****************************************************************************/
INLINE UWORD8* wps_put_cofig_meth(UWORD8* buf_ptr, UWORD16 config_meth)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_CONFIG_METHODS);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, config_meth);
    buf_ptr += 2;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Wi-Fi Protected Setup State(value passed as input) */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_wps_state(UWORD8* buf_ptr, UWORD8 wps_state)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_WPS_STATE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = wps_state;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Manufacturer attribute at the location pointed by  */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_manufacturer(UWORD8* buf_ptr,
                                     wps_enr_config_struct_t* config_ptr)
{
    UWORD16 len = (UWORD16)config_ptr->manufacturer_len;
    PUT_U16_BE(buf_ptr, WPS_ATTR_MANUFACTURER);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, len);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->manufacturer, len);
    buf_ptr += len;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Model name attribute at the location pointed by    */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_model_name(UWORD8* buf_ptr,
                                     wps_enr_config_struct_t* config_ptr)
{
    UWORD16 len = (UWORD16)config_ptr->model_name_len;
    PUT_U16_BE(buf_ptr, WPS_ATTR_MODEL_NAME);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, len);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->model_name, len);
    buf_ptr += len;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Model number attribute at the location pointed by  */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_model_num(UWORD8* buf_ptr,
                                     wps_enr_config_struct_t* config_ptr)
{
    UWORD16 len = (UWORD16)config_ptr->model_num_len;
    PUT_U16_BE(buf_ptr, WPS_ATTR_MODEL_NUM);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, len);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->model_num, len);
    buf_ptr += len;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Serial number attribute at the location pointed by */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_serial_num(UWORD8* buf_ptr,
                                     wps_enr_config_struct_t* config_ptr)
{
    UWORD16 len = (UWORD16)config_ptr->serial_num_len;
    PUT_U16_BE(buf_ptr, WPS_ATTR_SERIAL_NUM);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, len);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->serial_num, len);
    buf_ptr += len;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Primary Device type attribute at the location      */
/* pointed by the input pointer                                              */
/*****************************************************************************/
INLINE UWORD8* wps_put_prim_dev_type(UWORD8* buf_ptr,
                                      wps_enr_config_struct_t* config_ptr)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_PRIM_DEV_TYPE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_PRIM_DEV_TYPE_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->prim_dev_cat_id, WPS_PRIM_DEV_TYPE_LEN);
    buf_ptr += WPS_PRIM_DEV_TYPE_LEN;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Device name attribute at the location pointed by   */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD8* wps_put_device_name(UWORD8* buf_ptr,
                                     wps_enr_config_struct_t* config_ptr)
{
    UWORD16 len = (UWORD16)config_ptr->dev_name_len;
    PUT_U16_BE(buf_ptr, WPS_ATTR_DEV_NAME);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, len);
    buf_ptr += 2;
    memcpy(buf_ptr, config_ptr->dev_name, len);
    buf_ptr += len;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the RF-Bands attribute at the location pointed by the  */
/* input pointer                                                             */
/*****************************************************************************/
INLINE UWORD8* wps_put_rf_bands(UWORD8* buf_ptr, UWORD8 rf_bands)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_RF_BANDS);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = rf_bands;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Association State attribute at the location pointed*/
/* by the input pointer                                                      */
/*****************************************************************************/
INLINE UWORD8* wps_put_assoc_state(UWORD8* buf_ptr, UWORD16 assoc_state)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_ASSOC_STATE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, assoc_state);
    buf_ptr += 2;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the Device Password ID attribute at the location       */
/* pointed by the input pointer                                              */
/*****************************************************************************/
INLINE UWORD8* wps_put_dev_pass_id(UWORD8* buf_ptr, UWORD16 dev_pass_id)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_DEV_PASS_ID);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, dev_pass_id);
    buf_ptr += 2;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the OS Version attribute at the location pointed by the*/
/* input pointer                                                             */
/*****************************************************************************/
INLINE UWORD8* wps_put_os_ver(UWORD8* buf_ptr, UWORD32 os_version)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_OS_VERSION);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 4);
    buf_ptr += 2;
    PUT_U32_BE(buf_ptr, os_version);
    buf_ptr += 4;
    return(buf_ptr);
}

#ifdef MAC_P2P
/*****************************************************************************/
/* This function puts the Requested Device type attribute at the location    */
/* pointed by the input pointer                                              */
/*****************************************************************************/
INLINE UWORD8* wps_put_req_dev_type(UWORD8* buf_ptr,
                                      wps_enr_config_struct_t* config_ptr)
{
    if(NULL != config_ptr->req_dev_cat_id)
    {   /* Put the requested device ID attribute only if present */
        PUT_U16_BE(buf_ptr, WPS_ATTR_REQ_DEV_TYPE);
        buf_ptr += 2;
        PUT_U16_BE(buf_ptr, WPS_REQ_DEV_TYPE_LEN);
        buf_ptr += 2;
        memcpy(buf_ptr, config_ptr->req_dev_cat_id, WPS_REQ_DEV_TYPE_LEN);
        buf_ptr += WPS_REQ_DEV_TYPE_LEN;
    }
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the WSC Selected registrar type attribute at the       */
/* location  pointed by input pointer                                        */
/*****************************************************************************/
INLINE UWORD8* wps_put_sel_reg(UWORD8* buf_ptr, BOOL_T sel_reg)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_SEL_REG);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = sel_reg;
    return(buf_ptr);
}

/*****************************************************************************/
/* This function puts the WSC Response type attribute at the location        */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD8* wsc_put_rsp_type(UWORD8* buf_ptr, UWORD8 rsp_type)
{
    PUT_U16_BE(buf_ptr, WPS_ATTR_RESPONSE_TYPE);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, 1);
    buf_ptr += 2;
    *buf_ptr++ = rsp_type;
    return(buf_ptr);
}
#endif /* MAC_P2P */
/*****************************************************************************/
/* This function puts WFA Vendor Extension attribute with specified          */
/* subelements at specified location in the buffer                           */
/*****************************************************************************/
INLINE UWORD8* wps_put_wfa_vendor(UWORD8* buffer, wps_sub_elem_t **elem,
                                  UWORD8 elem_cnt)
{
    UWORD8  i = 0;
    UWORD8 *buf_start = buffer;

    /*************************************************************************/
    /* WFA Vendor Extension Attribute with Sub-Elements Format               */
    /*************************************************************************/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /* |  2-Bytes  | 2-Bytes |  3-Bytes |   1-Byte  |  1-Byte   |0-255 Bytes|*/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /* | Attribute |Attribute| Vendor-ID|Sub-Element|Sub-Element|Sub-Element|*/
    /* |ID (0x1049)|  Length |(0x00372A)|    ID-1   |   Length  |   value   |*/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /*************************************************************************/

    PUT_U16_BE(buffer, WPS_ATTR_VENDOR_EXTENSION);
    buffer += 4;

    *buffer++ = WFA_VEN_EXTN_ID_BYTE0;
    *buffer++ = WFA_VEN_EXTN_ID_BYTE1;
    *buffer++ = WFA_VEN_EXTN_ID_BYTE2;

    for(i = 0; i < elem_cnt; i++)
    {
        *buffer++ = elem[i]->id;
        *buffer++ = elem[i]->len;

        memcpy(buffer, elem[i]->val, elem[i]->len);
        buffer += elem[i]->len;
    }

    PUT_U16_BE((buf_start + 2), (buffer - buf_start - 4));

    return buffer;
}

#endif /* WPS_PROT_TX_H */

#endif /* INT_WPS_ENR_SUPP */
