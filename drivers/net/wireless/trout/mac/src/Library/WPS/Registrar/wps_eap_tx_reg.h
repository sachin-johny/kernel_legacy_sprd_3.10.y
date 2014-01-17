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
/*  File Name         : wps_eap_tx_reg.h                                     */
/*                                                                           */
/*  Description       : This file contains all the data type definitions and */
/*                      of INLINE functions of Registrar for EAP TX          */
/*                      handling of WPS Registrar Protocol.                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         20 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef WPS_EAP_TX_REG_H
#define WPS_EAP_TX_REG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "wps_prot.h"
#include "wpsr_reg_if.h"
#include "wps_registrar.h"
#include "wps_eap_rx_reg.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern STATUS_T wps_send_eap_req_id(wps_t *wps_ctxt_hdl,
                                    wps_eap_tx_t *eap_tx_hdl);
extern STATUS_T wps_send_eap_wsc_start(wps_t *wps_ctxt_hdl,
                                       wps_eap_tx_t *eap_tx_hdl);
extern STATUS_T wps_send_m2(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                            wps_eap_rx_t *eap_rx_hdl, UWORD8 *enr_pub_key);
extern STATUS_T wps_send_m2d(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl);
extern STATUS_T wps_send_m4(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                            wps_eap_rx_t *eap_rx_hdl);
extern STATUS_T wps_send_m6(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                            wps_eap_rx_t *eap_rx_hdl);
extern STATUS_T wps_send_m8(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                            wps_eap_rx_t *eap_rx_hdl);
extern STATUS_T wps_send_wsc_nack(wps_t *wps_ctxt_hdl,
                                  wps_eap_tx_t *eap_tx_hdl);
extern STATUS_T wps_send_eap_fail(wps_t *wps_ctxt_hdl,
                                  wps_eap_tx_t *eap_tx_hdl);
extern STATUS_T wps_handle_eap_retry(wps_t *wps_ctxt_hdl,
                                     wps_eap_tx_t *eap_tx_hdl, UWORD16 length);
extern STATUS_T wps_send_wsc_frag_ack(wps_t *wps_ctxt_hdl,
                                      wps_eap_tx_t *eap_tx_hdl);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function handles transmission of M2D message to Enrollee             */
INLINE STATUS_T wps_handle_m2d_tx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_tx_t *tx_hdl)
{
    STATUS_T status = SUCCESS;

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    status = wps_send_m2d(wps_ctxt_hdl, tx_hdl);

    if(FAILURE != status)
    {
        /* Set the Retry Count to Maximum and Start Activity timer. On       */
        /* Timeout the Protocol is stopped attomatically                     */
        tx_hdl->eap_msg_retry_cnt = MAX_EAP_MSG_RETRY_LIMIT;

        /* Update the Registrar State */
        reg_hdl->wpsr_state     = WPS_REG_PROT_ST;
        reg_hdl->reg_prot_state = REG_MSG_ST;
        reg_hdl->msg_state      = M2D_ST;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;

        send_wps_status(WPS_REG_M2D_TX, NULL, 0);
    }

    return status;

}

/*****************************************************************************/
/* This function puts the WSC Network Index  attribute at the location       */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_net_index(UWORD8 *buf_ptr, UWORD8 net_index)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_NET_INDEX);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = net_index;

    return index;
}

/*****************************************************************************/
/* This function puts the WSC SSID  attribute at the location                */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_ssid(UWORD8 *buf_ptr, UWORD8 *ssid, UWORD8 ssid_len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_SSID);
    index += 2;
    PUT_U16_BE((buf_ptr + index), ssid_len);
    index += 2;
    memcpy((buf_ptr + index), ssid, ssid_len);

    return (index + ssid_len);
}

/*****************************************************************************/
/* This function puts the WSC Authentication Type attribute at the location  */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_auth_type(UWORD8 *buf_ptr, UWORD16 auth_type)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_AUTH_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), auth_type);
    index += 2;

    return index;
}

/*****************************************************************************/
/* This function puts the WSC Encryption Type attribute at the location      */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_encr_type(UWORD8 *buf_ptr, UWORD16 encr_type)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_ENCRY_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), encr_type);
    index += 2;

    return index;
}

/*****************************************************************************/
/* This function puts the WSC Network Key Index  attribute at the location   */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_wep_keyid(UWORD8 *buf_ptr, UWORD8 key_id)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_WEPTRANSMITKEY);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = key_id;

    return index;
}

/*****************************************************************************/
/* This function puts the WSC Network Key Index  attribute at the location   */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_net_key_index(UWORD8 *buf_ptr, UWORD8 key_index)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_NET_KEY_INDEX);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = key_index;

    return index;
}

/*****************************************************************************/
/* This function puts the WSC Network Key attribute at the location          */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_net_key(UWORD8 *buf_ptr, UWORD8 *key, UWORD8 key_len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_NET_KEY);
    index += 2;
    PUT_U16_BE((buf_ptr + index), key_len);
    index += 2;
    memcpy((buf_ptr + index), key, key_len);

    return (index + key_len);
}

#endif /* WPS_EAP_TX_REG_H */
#endif /* INT_WPS_REG_SUPP */
