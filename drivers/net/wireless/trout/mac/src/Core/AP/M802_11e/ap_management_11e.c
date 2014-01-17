/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : ap_management_11e.c                                  */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for 802.11e access point mode.           */
/*                                                                           */
/*  List of Functions : ap_enabled_wmm_rx_action                             */
/*                      update_beacon_wmm_params                             */
/*                      set_wmm_ap_ac_params_config                          */
/*                      get_wmm_ap_ac_params_config                          */
/*                      set_wmm_sta_ac_params_config                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "receive.h"
#include "management_ap.h"
#include "mib_11e.h"
#include "management_11e.h"
#include "frame_11e.h"
#include "ap_management_11e.h"
#include "ap_uapsd.h"
#include "ap_frame_11e.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void update_beacon_wmm_params(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_wmm_rx_action                                 */
/*                                                                           */
/*  Description   : This function handles the incoming action frame in the   */
/*                  ENABLED state in AP mode for WMM.                        */
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

void ap_enabled_wmm_rx_action(UWORD8 *msg)
{
    wlan_rx_t    *wlan_rx = (wlan_rx_t*)msg;
    UWORD8       *data    = wlan_rx->msa + MAC_HDR_LEN;
    asoc_entry_t *ae      = (asoc_entry_t*)(wlan_rx->sa_entry);

    /* Process the action frame based on the category */
    switch(data[CATEGORY_OFFSET])
    {
    case WMM_CATEGORY:
    {
        UWORD8 sa[6] = {0};

        /* Extract the source address from the frame */
        get_SA(wlan_rx->msa, sa);

        /* Process the WMM action frame based on the action type */
        switch(data[ACTION_OFFSET])
        {
        case ADDTS_REQ_TYPE:
        {
            ap_uapsd_handle_rx_addts_req(wlan_rx->msa, wlan_rx->rx_len, sa,
                                         ae->sta_index);
        }
        break;
        case DELTS_TYPE:
        {
            ap_uapsd_handle_rx_delts(wlan_rx->msa, sa, ae->sta_index);
        }
        break;
        default:
        {
            /* Do nothing. Received WMM action type is not recognized.*/
        }
        break;
        }
    }
    break;
    default:
    {
        /* Do nothing. Received action category is not recognized for WMM */
    }
    break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_beacon_wmm_params                                 */
/*                                                                           */
/*  Description   : This function updates the WMM parameters in the WMM      */
/*                  Parameter element in the current beacon frame.           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : g_beacon_frame                                           */
/*                  g_beacon_index                                           */
/*                  g_beacon_len                                             */
/*                                                                           */
/*  Processing    : This function parses the current beacon frame for the    */
/*                  WMM Parameter element. Once this is located the QoS Info */
/*                  and AC Parameter Record fields in the same are updated.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_beacon_wmm_params(void)
{
    UWORD16 index     = TAG_PARAM_OFFSET;
    UWORD8  *curr_bcn = g_beacon_frame[g_beacon_index];

    /* NULL check before accessing the beacon pointer */
    if((g_beacon_index > 1) ||
       (curr_bcn == NULL))
    	return;

    /* Traverse the beacon till WMM Parameter element */
    while(index < g_beacon_len)
    {
        if(is_wmm_param_elem(curr_bcn + index) == BTRUE)
        {
            /* Update the index to p[oint to the QoS Information Field */
            index += 8;

            /* QoS Information Field */
            set_qos_info_wmm_ap(&curr_bcn[index]);
            index += QOS_INFO_LEN;

            /* Reserved */
            curr_bcn[index++] = 0;

            /* Set the AC_BE, AC_BK, AC_VI, AC_VO Parameter Record fields */
            index += set_ac_param_record_wmm_ap(&curr_bcn[index], AC_BE);
            index += set_ac_param_record_wmm_ap(&curr_bcn[index], AC_BK);
            index += set_ac_param_record_wmm_ap(&curr_bcn[index], AC_VI);
            index += set_ac_param_record_wmm_ap(&curr_bcn[index], AC_VO);

            break;
        }

        index += (2 + curr_bcn[index + 1]);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wmm_ap_ac_params_config                              */
/*                                                                           */
/*  Description   : This function configures the AP AC parameters to be used */
/*                  in AP mode.                                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the configuration packet                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The configuration packet is parsed and for each AC the   */
/*                  parameter record is read and corresponding MIB updated.  */
/*                  Once all the MIB parameters have been updated for all AC */
/*                  the parameter set count is incremented. Also the MAC H/w */
/*                  registers are updated with the new AC parameters. The    */
/*                  WMM parameter element in the current beacon frame is     */
/*                  also updated with these new parameters.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_wmm_ap_ac_params_config(UWORD8 *val)
{
    UWORD8 index = 0;

    /* Set the index to point to the AC parameter configuration. The first 2 */
    /* bytes of the given configuration packet contains the length.          */
    index = 2;

    /* Format of AC Parameter configuration packet string                    */
    /* --------------------------------------------------------------------- */
    /* | Byte 0:5       | Byte 6:11      | Byte 12:17     | Byte 18:23     | */
    /* --------------------------------------------------------------------- */
    /* | AC_VO ParamRec | AC_VI ParamRec | AC_BE ParamRec | AC_BK ParamRec | */
    /* --------------------------------------------------------------------- */
    set_ap_ac_param_record(val + index, AC_VO);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_ap_ac_param_record(val + index, AC_VI);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_ap_ac_param_record(val + index, AC_BE);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_ap_ac_param_record(val + index, AC_BK);

    /* Each time the AC parameters are changed the Parameter set count is    */
    /* incremented. Also the MAC H/w registers and the beacon frame are      */
    /* updated.                                                              */
    mincr_ac_parameter_set_count();
    update_edca_machw();
    update_beacon_wmm_params();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_wmm_ap_ac_params_config                              */
/*                                                                           */
/*  Description   : This function gets the AP AC parameter configuration for */
/*                  the AP.                                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_cfg_val                                                */
/*                                                                           */
/*  Processing    : The AC parameter configuration is read from the MIB and  */
/*                  the global configuration string is updated with the      */
/*                  current configuration values.                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8* get_wmm_ap_ac_params_config(void)
{
    UWORD8 index = 0;

    /* Set the length in the first 2 bytes of the temporary configuration    */
    /* string global                                                         */
    g_cfg_val[0] = (UWORD8)(AC_PARAM_CONFIG_LEN & 0x00FF);
    g_cfg_val[1] = (UWORD8)((AC_PARAM_CONFIG_LEN & 0xFF00) >> 8);

    /* Set the index to point to the AC parameter configuration. The first 2 */
    /* bytes of the given configuration packet contains the length.          */
    index = 2;

    /* Format of AC Parameter configuration packet string                    */
    /* --------------------------------------------------------------------- */
    /* | Byte 0:5       | Byte 6:11      | Byte 12:17     | Byte 18:23     | */
    /* --------------------------------------------------------------------- */
    /* | AC_VO ParamRec | AC_VI ParamRec | AC_BE ParamRec | AC_BK ParamRec | */
    /* --------------------------------------------------------------------- */
    get_ap_ac_param_record(g_cfg_val + index, AC_VO);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_ap_ac_param_record(g_cfg_val + index, AC_VI);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_ap_ac_param_record(g_cfg_val + index, AC_BE);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_ap_ac_param_record(g_cfg_val + index, AC_BK);

    return &g_cfg_val[0];
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wmm_sta_ac_params_config                             */
/*                                                                           */
/*  Description   : This function configures the STA AC parameters to be     */
/*                  by the STA associating with the AP.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the configuration packet                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The configuration packet is parsed and for each AC the   */
/*                  parameter record is read and corresponding MIB updated.  */
/*                  These configuration values are used for displaying the   */
/*                  parameters to be used by the STA in the BSS.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_wmm_sta_ac_params_config(UWORD8 *val)
{
    UWORD8 index = 0;

    /* Set the index to point to the AC parameter configuration. The first 2 */
    /* bytes of the given configuration packet contains the length.          */
    index = 2;

    /* Format of AC Parameter configuration packet string                    */
    /* --------------------------------------------------------------------- */
    /* | Byte 0:5       | Byte 6:11      | Byte 12:17     | Byte 18:23     | */
    /* --------------------------------------------------------------------- */
    /* | AC_VO ParamRec | AC_VI ParamRec | AC_BE ParamRec | AC_BK ParamRec | */
    /* --------------------------------------------------------------------- */
    set_sta_ac_param_record(val + index, AC_VO);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_sta_ac_param_record(val + index, AC_VI);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_sta_ac_param_record(val + index, AC_BE);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    set_sta_ac_param_record(val + index, AC_BK);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_sta_info_wmm                                      */
/*                                                                           */
/*  Description   : This function updates the WMM specific station           */
/*                  information fields.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association entry of the station       */
/*                  2) Pointer buffer to update the information fields       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the WMM information about the     */
/*                  station from its association entry table and updates the */
/*                  fields with the same.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

/*************************************************************************/
/*                'Format of WMM Info Bytes'                             */
/*-----------------------------------------------------------------------*/
/* BYTE-0:                                                               */
/* b0    : 1 -> WMM Supported            0 -> Not supported              */
/* b1-b2 : Max Service Period                                            */
/* b3-b7 : Reserved                                                      */
/* BYTE-1: UAPSD Status                                                  */
/* b0    : 1 -> AC-VO Trigger Enabled    0 -> Not Enabled                */
/* b1    : 1 -> AC-VO Delivery Enabled   0 -> Not Enabled                */
/* b2    : 1 -> AC-VI Trigger Enabled    0 -> Not Enabled                */
/* b3    : 1 -> AC-VI Delivery Enabled   0 -> Not Enabled                */
/* b4    : 1 -> AC-BE Trigger Enabled    0 -> Not Enabled                */
/* b5    : 1 -> AC-BE Delivery Enabled   0 -> Not Enabled                */
/* b6    : 1 -> AC-BK Trigger Enabled    0 -> Not Enabled                */
/* b7    : 1 -> AC-BK Delivery Enabled   0 -> Not Enabled                */
/*************************************************************************/

void update_sta_info_wmm(void *asoc_entry, UWORD8 *info)
{
    asoc_entry_t *ae = (asoc_entry_t *)asoc_entry;

    info[0] = info[1] = 0;

    if(is_dst_wmm_capable(ae->sta_index) == BTRUE)
    {
        info[0] |= BIT0;
        /* max_sp_len is encoded as defined in the standard while updating */
        /* the Station Info Byte.                                          */
        /* max_sp_len     Code       Description                           */
        /*     0            0        All buffered frames                   */
        /*     2            1        Max 2 buffered frames                 */
        /*     4            2        Max 4 buffered frames                 */
        /*     6            3        Max 6 buffered frames                 */
        info[0] |= ((ae->uapsd_status.max_sp_len >> 1) << 1);

        info[1] = ae->uapsd_status.ac_asoc_uapsd_status;
    }
}

#endif /* MAC_WMM */
#endif /* BSS_ACCESS_POINT_MODE */
