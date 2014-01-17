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
/*  File Name         : management_11e.c                                     */
/*                                                                           */
/*  Description       : This file contains MAC management related functions  */
/*                      for 802.11e protocol.                                */
/*                                                                           */
/*  List of Functions : initialize_wmm                                       */
/*                      is_dst_wmm_capable                                   */
/*                      update_table_wmm                                     */
/*                      update_tx_mib_11e                                    */
/*                      classify_msdu                                        */
/*                      get_wmm_sta_ac_params_config                         */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "frame_11e.h"
#include "management_11e.h"
#include "mh.h"
#include "receive.h"
#include "prot_if.h"
#include "qmu_if.h"
#include "host_if.h"
#include "trout_wifi_rx.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

//chenq mod BFALSE=>BTRUE
//BOOL_T g_wmm_enabled = BFALSE;
BOOL_T g_wmm_enabled = BTRUE;

/* Table to maintain WMM info of stations, in order of STA index */
UWORD8 g_wmm_capability[MAX_STA_SUPPORTED] = {0};

/* Table of priority to queue number mapping for EDCA (WMM)                  */
/* For EDCA there is one queue per TID. The mapping of TIDs 0 - 7 to various */
/* access categories is as per standard. TIDs 8 - 15 are not valid for EDCA  */
/* and are all mapped by default to AC_BE_Q.                                 */
/* TID           : Access Category                                           */
/* 1, 2          : AC_BK_Q                                                   */
/* 3, 0          : AC_BE_Q                                                   */
/* 5, 4          : AC_VI_Q                                                   */
/* 7, 6          : AC_VO_Q                                                   */
/* 8 - 15        : AC_BE_Q (Invalid setting for EDCA)                        */
const UWORD8 g_txq_pri_to_num[NUM_TIDS + 1] = {
AC_BE_Q, AC_BK_Q, AC_BK_Q, AC_BE_Q, AC_VI_Q, AC_VI_Q, AC_VO_Q, AC_VO_Q,
AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q,
HIGH_PRI_Q};

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_wmm                                        */
/*                                                                           */
/*  Description      : This function initializes all WMM data structures.    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_wmm_capability                                      */
/*                     g_q_handle                                            */
/*                                                                           */
/*  Processing       : This function initializes the WMM information table,  */
/*                     the queue interface table and the queue manager.      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_wmm(void)
{
    mem_set(g_wmm_capability, 0, MAX_STA_SUPPORTED);
    init_qif_table();
    //destroy_mac_qmu(&g_q_handle);	//masked by chengwg, 2013-01-15.

	//init_mac_qmu();	//masked by chengwg, 2013-01-15.
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_dst_wmm_capable                                       */
/*                                                                           */
/*  Description   : This function checks if the destination station is WMM   */
/*                  capable.                                                 */
/*                                                                           */
/*  Inputs        : 1) Destination station index                             */
/*                                                                           */
/*  Globals       : g_wmm_capability                                         */
/*                                                                           */
/*  Processing    : The entry for the given station index is checked in the  */
/*                  global WMM enable table and the WMM status is returned.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T, BTRUE if the given STA is WMM capable            */
/*                          BFALSE otherwise                                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_dst_wmm_capable(UWORD8 sta_index)
{
    if((sta_index > 0) && (sta_index <= MAX_STA_SUPPORTED))
    {
        return (g_wmm_capability[sta_index - 1]? BTRUE:BFALSE);
    }

    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_table_wmm                                         */
/*                                                                           */
/*  Description   : This function updates the table with the WMM capability  */
/*                  information of the given station and the MAC H/w table   */
/*                  for WMM sequence number update.                          */
/*                                                                           */
/*  Inputs        : 1) Station index                                         */
/*                  2) Pointer to the received MAC frame                     */
/*                  3) Received frame length                                 */
/*                  4) Offset to the start of information elements           */
/*                                                                           */
/*  Globals       : g_wmm_capability                                         */
/*                                                                           */
/*  Processing    : The entry for the given STA index is updated to the WMM  */
/*                  capability of the STA. The corresponding MAC hardware    */
/*                  registers are also updated.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_table_wmm(UWORD8 sta_index, UWORD8 *msa, UWORD16 rx_len,
                      UWORD16 ie_offset)
{
    UWORD8 sa[6] = {0};
    BOOL_T wmm_cap  = BFALSE;

    get_TA(msa, sa);

    /* STA index starts from 1. Decrement by 1 to get the WMM table index */
    sta_index--;

    wmm_cap  = is_wmm_supported(msa, rx_len, ie_offset);

    /* Update the QoS Field for the entry */
    if((wmm_cap == BTRUE) &&
       (g_wmm_capability[sta_index] == BFALSE))
    {
        g_wmm_capability[sta_index] = BTRUE;

        /* Set the MAC H/w registers for WMM sequence number updation */
        set_machw_stamacaddr(sa);
        set_machw_seq_num_index_update(sta_index, (BOOL_T)g_wmm_capability[sta_index]);
    }
    else
        g_wmm_capability[sta_index] = wmm_cap;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_tx_mib_11e                                        */
/*                                                                           */
/*  Description   : This function updates the 11e MIB transmission count     */
/*                  parameters.                                              */
/*                                                                           */
/*  Inputs        : 1) TID of the packet                                     */
/*                  2) Retry count                                           */
/*                  3) CTS failure Count                                     */
/*                  4) RTS success Count                                     */
/*                  5) Number of fragments                                   */
/*                  6) Transmission status flag                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All the transmission related MIB prameters specific to   */
/*                  11e are updated based on the status of the transmission. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_tx_mib_11e(UWORD8 tid, UWORD8 retry_count, UWORD8 cts_fail,
                       UWORD8 rts_succ, UWORD8 frag_num, BOOL_T is_success)
{
    /* Retry Count */
    if(retry_count != 0)
    {
        mset_QoSRetryCount(mget_QoSRetryCount(tid) + 1, tid);
        if(retry_count > 1)
        {
            mset_QoSMultipleRetryCount(mget_QoSMultipleRetryCount(tid) + 1,
                                       tid);
        }
    }

    /* Ack Failures */
    mset_QoSAckFailureCount(mget_QoSAckFailureCount(tid) + retry_count, tid);

    /* RTS Success count */
    mset_QoSRTSSuccessCount(mget_QoSRTSSuccessCount(tid) + rts_succ, tid);

    /* RTS Failure count */
    mset_QoSRTSFailureCount(mget_QoSRTSFailureCount(tid) + cts_fail, tid);

    /* Fragment transmitted successfully */
    mset_QoSTransmittedFragmentCount(
                    mget_QoSTransmittedFragmentCount(tid) + frag_num, tid);

    if(is_success == BTRUE)
    {
        /* Frame transmitted */
        mset_QoSTransmittedFrameCount(mget_QoSTransmittedFrameCount(tid) + 1,
                                      tid);
    }
    else
    {
        /* Frame failed to transmit */
        mset_QoSFailedCount(mget_QoSFailedCount(tid) + 1, tid);
    }
}


#ifdef EDCA_DEMO_KLUDGE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : classify_msdu_port_number                                */
/*                                                                           */
/*  Description   : This function classifies an incoming MSDU to required    */
/*                  priority based on port number                            */
/*                                                                           */
/*  Inputs        : 1) Port number                                           */
/*                  2) Pointer to priority value                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The priority is decided based on the port number         */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if priority value is assigned                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T classify_msdu_port_number(UWORD16 prt, UWORD8 *ac)
{
    BOOL_T ac_assigned = BTRUE;
    /* Classify on port number */
        switch(prt)
        {
        case 2340:
            {
                *ac = PRIORITY_0; /* background */
            }
            break;

        case 2341:
            {
                *ac = PRIORITY_1; /* Best Effort */
            }
            break;

        case 2342:
            {
                *ac = PRIORITY_2; /* video */
            }
            break;

        case 2343:
            {
                *ac = PRIORITY_3; /* voice */
            }
            break;

        default:
            ac_assigned = BFALSE;
            break;
        }

    return ac_assigned;

}
#endif /* EDCA_DEMO_KLUDGE */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : classify_msdu                                            */
/*                                                                           */
/*  Description   : This function classifies an incoming MSDU to required    */
/*                  priority.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming frame                         */
/*                  2) Type of ethernet frame                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The user priority is found based on the VLAN tag or IP   */
/*                  TOS field. The priority is set to the user priority      */
/*                  extracted.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Prioirty value (0 - 7)                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 classify_msdu(UWORD8 *data, UWORD16 eth_type)
{
    UWORD8 ac = 0;

    if(eth_type == IP_TYPE)
    {
        /* Type of Service field in the IP header */
        UWORD8 priority = data[TOS_FIELD_OFFSET] & 0xE0;
        BOOL_T ac_assigned = BFALSE;

#ifdef EDCA_DEMO_KLUDGE
        UWORD16 dst_prt = data[UDP_DST_PORT_OFFSET + 1] | (data[UDP_DST_PORT_OFFSET] << 8);
        UWORD16 src_prt = data[UDP_SRC_PORT_OFFSET + 1] | (data[UDP_SRC_PORT_OFFSET] << 8);
        ac_assigned = BTRUE;

        /* Classify MSDU on destination port number */
        ac_assigned = classify_msdu_port_number(dst_prt, &ac);

        /* Classify MSDU on source port number */
        if(ac_assigned == BFALSE)
            ac_assigned = classify_msdu_port_number(src_prt, &ac);

#endif /* EDCA_DEMO_KLUDGE */
        if(ac_assigned == BFALSE)
        {
        switch (priority)
        {
        case 0x20:             /* IP-PL1 */
        case 0x40:             /* IP-PL2 */
            {
                ac = PRIORITY_0; /* background */
            }
            break;
        case 0x80:           /* IP-PL4 */
        case 0xA0:           /* IP-PL5 */
            {
                ac = PRIORITY_2; /* Video */
            }
            break;
        case 0xC0:           /* IP-PL6 */
        case 0xE0:           /* IP-PL7 */
            {
                ac = PRIORITY_3; /* Voice */
            }
            break;
        default:
            {
                ac = PRIORITY_1; /* Best Effort */
            }
            break;
            }
        }
    }
    else if(eth_type == VLAN_TYPE)
    {
        /* VLAN packets will have TID value 0-7 */
        ac = ((data[VLAN_TID_FIELD_OFFSET] >> 5) & 0x07);
    }
    else
    {
        ac = PRIORITY_1; /* Best Effort */
    }

    return ac;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_wmm_sta_ac_params_config                             */
/*                                                                           */
/*  Description   : This function gets the STA AC parameter configuration as */
/*                  set by the AP.                                           */
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

UWORD8* get_wmm_sta_ac_params_config(void)
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
    get_sta_ac_param_record(g_cfg_val + index, AC_VO);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_sta_ac_param_record(g_cfg_val + index, AC_VI);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_sta_ac_param_record(g_cfg_val + index, AC_BE);
    index += AC_PARAM_RECORD_CONFIG_LEN;
    get_sta_ac_param_record(g_cfg_val + index, AC_BK);

    return &g_cfg_val[0];
}


#endif /* MAC_WMM */
