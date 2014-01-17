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
/*  File Name         : wps_registrar.h                                      */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS Registrar Protocol. It has function definitions  */
/*                      of APIs provided by WPS Registrar and APIs required  */
/*                      by WPS Registrar.                                    */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef WPS_REG_H
#define WPS_REG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_common.h"
#include "wpsr_reg_if.h"
#include "wps_prot.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#define MONITOR_TIME            120000000 /* PBC Monitor Time in uS          */
#define WPS_REG_PROT_TIMEOUT    1
#define WPS_EAP_MSG_TIMEOUT     2
// 20120709 caisf add, merged ittiam mac v1.2 code
#define WPS_DEAUTH_TIMEOUT        3
#define REG_PROT_TIMEOUT        120000    /* RP timeout value in mS          */
#define EAP_MSG_TIMEOUT         5000      /* EAP Message timeout value in mS */
// 20120709 caisf add, merged ittiam mac v1.2 code
#define WPS_DEAUTH_TIMEOUT_VALUE  20        /* Deauth timeout value in mS   */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern STATUS_T wps_start_eap_msg_timer(wps_t *wps_ctxt_hdl);
extern void wps_stop_eap_msg_timer(wps_t *wps_ctxt_hdl);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* This function checks if the TSF value passed in variable A is within the  */
/* monitor windows of value in B. This function returns BTRUE if (B +        */
/* Monitor Time) >= A                                                        */
/*****************************************************************************/
INLINE BOOL_T  in_mon_window(UWORD32 val_a_hi, UWORD32 val_a_lo,
                             UWORD32 val_b_hi, UWORD32 val_b_lo)
{
    UWORD32 temp = 0;
    BOOL_T retval = BFALSE;

    temp = val_b_lo + MONITOR_TIME;

    if(temp < val_b_lo)
        val_b_hi++;

    val_b_lo = temp;

    if(val_b_hi > val_a_hi)
        retval = BTRUE;
    else if(val_b_hi == val_a_hi)
    {
        if(val_b_lo >= val_a_lo)
        {
            retval = BTRUE;
        }
    }

    return retval;
}

/* This function swaps the 2 entries in PBC enrollee information array */
INLINE void swap_entries(wps_mon_pbc_t *pbc_info)
{
    wps_mon_pbc_t tem_pbc_info;

    memcpy(&tem_pbc_info, &pbc_info[0], sizeof(wps_mon_pbc_t));
    memcpy(&pbc_info[0], &pbc_info[1], sizeof(wps_mon_pbc_t));
    memcpy(&pbc_info[1], &tem_pbc_info, sizeof(wps_mon_pbc_t));

    return;
}

/*****************************************************************************/
/* This function checks if there has been PBC session overlap. If so then the*/
/* difference in the Current TSF timer value with timer values on which the  */
/* first PBC session enrollee was detected is computed. If this difference in*/
/* less than MONITOR_TIME then status FAILURE is returned indicate Session   */
/* Overlap.                                                                  */
/*****************************************************************************/
INLINE STATUS_T wps_handle_pbc_start_req(wps_t *wps_ctxt_hdl)
{
    UWORD32       cur_tsf_hi = 0;
    UWORD32       cur_tsf_lo = 0;
    wps_reg_t     *reg_hdl   = wps_ctxt_hdl->wps_reg_hdl;
    wps_mon_pbc_t *pbc_info  = reg_hdl->pbc_enr_info;

    /* Get the current TSF timer value */
    wps_get_tsf_timestamp(&cur_tsf_hi, &cur_tsf_lo);

    /* Check for PBC Session Overlap */
    if(2 == reg_hdl->pbc_enr_cnt)
    {
        /*********************************************************************/
        /* If the first PBC Enrollee was detected within 120 seconds prior   */
        /* to this start request then return failure. First PBC Enrollee     */
        /* detected will be at index 0 of PBC Enrollee information array     */
        /*********************************************************************/
        if(BTRUE == in_mon_window(cur_tsf_hi, cur_tsf_lo,
                                  pbc_info[0].tsf_timer_hi,
                                  pbc_info[0].tsf_timer_lo))
        {
            return FAILURE;
        }

        /*********************************************************************/
        /* If the entry at index 1 is also not with MONITOR window then reset*/
        /* the PBC Session Overlap indication flag and the pbc enrollee count*/
        /*********************************************************************/
        if(BFALSE ==  in_mon_window(cur_tsf_hi, cur_tsf_lo,
                                    pbc_info[1].tsf_timer_hi,
                                    pbc_info[1].tsf_timer_lo))
        {
            reg_hdl->pbc_enr_cnt  = 0;

            return SUCCESS;
        }

        /*********************************************************************/
        /* Since entry at index 0 is beyond the MONITOR window and that at   */
        /* index 1 is with MONTOR window, remove entry at index 0 and update */
        /* it with entry at index 1. Also update the count and SS olap flag  */
        /*********************************************************************/
        memcpy(&pbc_info[0], &pbc_info[1], sizeof(wps_mon_pbc_t));
        reg_hdl->pbc_enr_cnt  = 1;

        return SUCCESS;
    }

    return SUCCESS;
}

/*****************************************************************************/
/* This function updates the Enrollee PBC information structrue array at     */
/* given index                                                               */
/*****************************************************************************/
INLINE void update_pbc_enr_info(wps_mon_pbc_t *pbc_enr_info, UWORD8 *mac_addr,
                                UWORD8 *uuid_e, UWORD32 tsf_hi, UWORD32 tsf_lo,
                                UWORD8 index)
{
    if(1 < index)
     return;

    /* Update the MAC address */
    memcpy(pbc_enr_info[index].mac_addr, mac_addr, MAC_ADDRESS_LEN);
    memcpy(pbc_enr_info[index].uuid_e, uuid_e, WPS_UUID_LEN);

    /* Update current TSF timer value */
    pbc_enr_info[index].tsf_timer_hi = tsf_hi;
    pbc_enr_info[index].tsf_timer_lo = tsf_lo;
}

/* This function deletes all the timers in the system */
INLINE void wps_delete_timers(wps_t *wps_ctxt_hdl)
{
    /*************************************************************************/
    /* If Registration Protocol Timer is running and if the start request is */
    /* BFALSE then stop it and delete it                                     */
    /*************************************************************************/
    if((NULL != wps_ctxt_hdl->reg_prot_timer) &&
       (BFALSE == wps_ctxt_hdl->wps_reg_hdl->start_req))
    {
        wps_stop_alarm(wps_ctxt_hdl->reg_prot_timer);
        wps_delete_alarm(&wps_ctxt_hdl->reg_prot_timer);
    }

    /* If EAP TX Message Timer is running then stop it and delete it */
    if(NULL != wps_ctxt_hdl->eap_msg_timer)
    {
        wps_stop_alarm(wps_ctxt_hdl->eap_msg_timer);
        wps_delete_alarm(&wps_ctxt_hdl->eap_msg_timer);
    }

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* If EAP TX Message Timer is running then stop it and delete it */
	if(NULL != wps_ctxt_hdl->deauth_timer)
	{
		wps_stop_alarm(wps_ctxt_hdl->deauth_timer);
		wps_delete_alarm(&wps_ctxt_hdl->deauth_timer);
    }
}

/* Free all the memory that is allocated in this library */
INLINE void wps_free_all_memory(wps_t *wps_ctxt_hdl)
{
    wps_eap_tx_t *tx_hdl = wps_ctxt_hdl->wps_eap_tx_hdl;
    wps_eap_rx_t *rx_hdl = wps_ctxt_hdl->wps_eap_rx_hdl;

    while(NULL != tx_hdl->tx_wlan_frm_buff)
    {
        if (0 == wps_get_num_users(tx_hdl->tx_wlan_frm_buff))
            break;

        wps_pkt_mem_free(tx_hdl->tx_wlan_frm_buff);
    }

    tx_hdl->tx_wlan_frm_buff = NULL;
    tx_hdl->tx_m_msg_ptr = NULL;

    /* Only incase of support for EAP defragmentation the buffer is allocated*/
    /* locally also. Free such buffers                                       */
    if(BTRUE == rx_hdl->defrag_in_prog)
    {
        wps_pkt_mem_free(rx_hdl->rx_eap_msg);
    }
    rx_hdl->rx_eap_msg     = NULL;
    rx_hdl->rx_m_msg_ptr   = NULL;
    rx_hdl->defrag_in_prog = BFALSE;
}

/*****************************************************************************/
/* This function inserts all the mandatory and some optional WSC attributes  */
/* for Beacon/probe response/(re)association response frame passed           */
/*****************************************************************************/
INLINE UWORD16 insert_wsc_ie(wps_t *wps_ctxt_hdl, UWORD8 *buffer,
                             UWORD8 frm_type)
{
    UWORD8       sub_elem_cnt = 0;
    UWORD16      index       = 0;
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_sub_elem_t *sub_elem_ptr[MAX_SUB_ELEM_SUPP];

    /* Insert Version Attribute */
    index += put_wsc_version((buffer + index));

    if((ASSOC_RSP != frm_type) && (REASSOC_RSP != frm_type))
    {
        /* Insert Wi-Fi Protected Setup State Attribute */
        index += put_wsc_wps_state((buffer + index), config_hdl->wps_state);

        /* Insert AP Setup Locked Attribute */
        index += put_wsc_ap_setup_locked((buffer + index),
                                         config_hdl->ap_setup_locked);

        /*********************************************************************/
        /* Add device passowrd ID and slected registrar config method        */
        /* attributes in case the registrar is selected                      */
        /*********************************************************************/
        if(BTRUE == attr_hdl->sel_reg)
        {
            /* Insert Selected Registrar Attribute */
            index += put_wsc_sel_reg((buffer + index), attr_hdl->sel_reg);

           /* Insert the Device Password ID Attribute */
            index += put_wsc_dev_pass_id((buffer + index),
                                         config_hdl->dev_pass_id);

            /* Insert the Selected Registrar Config Methods Attribute */
            index += put_wsc_sel_reg_config_meth((buffer + index),
                                                 config_hdl->config_methods);
        }
    }

    /* Insert Response Type Attribute */
    if(BEACON != frm_type)
        index += put_wsc_rsp_type((buffer + index), config_hdl->rsp_type);

    /* Addtional Attributes are insteted in case of Probe Response           */
    if(PROBE_RSP == frm_type)
    {
        /* Insert UUID-E Attribute */
        index += put_wsc_uuid_e((buffer + index), attr_hdl->reg_uuid);

        /* Insert Manufacturer Attribute */
        index += put_wsc_manufacturer((buffer + index),
                                      config_hdl->manufacturer,
                                      config_hdl->manufacturer_len);

        /* Insert Model Name Attribute */
        index += put_wsc_model_name((buffer + index), config_hdl->model_name,
                                    config_hdl->model_name_len);

        /* Insert Model Number Attribute */
        index += put_wsc_model_num((buffer + index), config_hdl->model_num,
                                   config_hdl->model_num_len);

        /* Insert Serial Number Attribute */
        index += put_wsc_serial_num((buffer + index), config_hdl->serial_num,
                                    config_hdl->serial_num_len);

// 20120709 caisf mod, merged ittiam mac v1.2 code
#ifdef MAC_P2P
        /* Insert Configuration Methods Attribute */
        /* Note : The config method is set to zero since external registrar */
        /* is not supported in AP mode (Refer to P2P Test Case 4.2.2        */
        index += put_wsc_config_meth((buffer + index),0);
#else /* MAC_P2P */
        /* Insert Configuration Methods Attribute */
        index += put_wsc_config_meth((buffer + index),
                                     config_hdl->config_methods);
#endif /* MAC_P2P */
    }

    if((PROBE_RSP == frm_type) || (BEACON == frm_type))
    {
        /* Insert Primary Device Type Attribute */
        index += put_wsc_prim_dev_type((buffer + index),
                                       config_hdl->prim_dev_cat_id);

        /* Insert Device Name Attribute */
        index += put_wsc_device_name((buffer + index), config_hdl->device_name,
                                     config_hdl->device_name_len);
    }

    /* Version2 Subelement */
    sub_elem_ptr[sub_elem_cnt++] = &(config_hdl->version2);

    /*************************************************************************/
    /* If Selected Registrar is TRUE then set the Authorized Subelement  in  */
    /* Beacon and Probe Response frame                                       */
    /*************************************************************************/
    if((BTRUE == attr_hdl->sel_reg) && (ASSOC_RSP != frm_type) &&
       (REASSOC_RSP != frm_type))
    {
        /* Set the AuthorizedMACs subelement */
        sub_elem_ptr[sub_elem_cnt++] = &(config_hdl->auth_macs);
    }

    /* Insert WFA Vendor Extension Attribute with desired Sublements         */
    index += put_wsc_wfa_vendor((buffer + index), sub_elem_ptr, sub_elem_cnt);

    return index;
}

#endif /* WPS_REG_H */
#endif /* INT_WPS_REG_SUPP */
