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
/*  File Name         : transmit.c                                           */
/*                                                                           */
/*  Description       : This file contains the MAC transmit path related     */
/*                      functions independent of mode.                       */
/*                                                                           */
/*  List of Functions : create_default_tx_dscr                               */
/*                      set_tx_frame_details                                 */
/*                      tx_mgmt_frame                                        */
/*                      tx_control_frame                                     */
/*                      update_tx_mib                                        */
/*                      set_tx_params                                        */
/*                      free_tx_dscr                                         */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "core_mode_if.h"
#include "metrics.h"
#include "qmu_if.h"
#include "transmit.h"
#include "qmu_tx.h"

//zhuyg add for trout wifi npi in 2013-05-27
#ifdef TROUT_WIFI_NPI
#include "trout_wifi_npi.h"
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_tx_params                                            */
/*                                                                           */
/*  Description   : This function sets the transmit parameters in the        */
/*                  transmit descriptor.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                  2) Transmit rate                                         */
/*                  3) Preamble                                              */
/*                  4) ACK policy                                            */
/*                  5) PHY transmission mode                                 */
/*                  6) Retry rate set                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the transmit parameters (data rates,  */
/*                  ACK policy, preamble type, mode) as per defined format.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to transmit descriptor                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_pwr_tx_11b;
extern UWORD32 g_pwr_tx_11g;
extern UWORD32 g_pwr_tx_11n;
extern UWORD32 g_pwr_tx_rx_11abgn_reset;


extern UWORD32 g_nv_11b_tx_pwr[4];
extern UWORD32 g_nv_11g_tx_pwr[4];
extern UWORD32 g_nv_11n_tx_pwr[4];
extern UWORD32 g_cmcc_cfg_tx_rate;
extern UWORD32 g_cmcc_set_max_pwr;
#endif

void set_tx_params(UWORD8 *tx_dscr, UWORD8 tr, UWORD8 pr, UWORD8 ap,
                   UWORD32 ptm, UWORD32 *retry_rate_set)
{
   UWORD8 dr  = 0;
    UWORD8 pow = 0;
    /* --------------------------------------------------------------------- */
    /*  Data Rate encoding                                                   */
    /* --------------------------------------------------------------------- */
    /*  11b rates:                   11a/11g OFDM rates:   11n MCS:          */
    /*                                                                       */
    /*  0000 - Reserved              1011 - 6 Mbps         MCS Index with    */
    /*  0001 - Short 2 Mbps          1111 - 9 Mbps         Bit 7 set to 1    */
    /*  0010 - Short 5.5 Mbps        1010 - 12 Mbps                          */
    /*  0011 - Short 11 Mbps         1110 - 18 Mbps                          */
    /*  0100 - Long 1 Mbps           1001 - 24 Mbps                          */
    /*  0101 - Long 2 Mbps           1101 - 36 Mbps                          */
    /*  0110 - Long 5.5 Mbps         1000 - 48 Mbps                          */
    /*  0111 - Long 11 Mbps          1100 - 54 Mbps                          */
    /* --------------------------------------------------------------------- */
    dr  = get_phy_rate(tr);
    pow = get_tx_pow(dr, ptm);
    if(IS_RATE_11B(dr) == BTRUE)
    {
        dr |= (BIT2 & (pr << 2));
    }
    set_tx_dscr_data_rate_0((UWORD32 *)tx_dscr, dr);
    set_tx_dscr_ack_policy((UWORD32 *)tx_dscr, ap);
    set_tx_dscr_phy_tx_mode((UWORD32 *)tx_dscr, ptm);

#ifdef IBSS_BSS_STATION_MODE    
    if(g_wifi_bt_coex)
    {
        set_tx_dscr_retry_rate_set1((UWORD32 *)tx_dscr, (dr << 24) | (dr << 16) | (dr << 8) | dr);
    	set_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr, (dr << 16) | (dr << 8) | dr);
    }
	else
	{
    set_tx_dscr_retry_rate_set1((UWORD32 *)tx_dscr, retry_rate_set[0]);
    set_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr, retry_rate_set[1]);
	}
#else
    set_tx_dscr_retry_rate_set1((UWORD32 *)tx_dscr, retry_rate_set[0]);
    set_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr, retry_rate_set[1]);
#endif
    set_tx_dscr_tx_pow_level((UWORD32 *)tx_dscr, pow);
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_default_tx_dscr                                   */
/*                                                                           */
/*  Description   : This function creates the Transmit descriptor for the    */
/*                  transmit packet, and updates the fields with default     */
/*                  values.                                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory is allocated for the transmit descriptor in   */
/*                  shared memory, and the fields are cleared. Status and    */
/*                  TSF timestamp fields are set to default values.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to transmit descriptor                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *create_default_tx_dscr(UWORD8 is_qos, UWORD8 priority, UWORD8 is_amsdu)
{
    UWORD8  temp     = 0;
    UWORD8  *tx_dscr = 0;
    mem_handle_t *mem_handle = 0;

	TROUT_FUNC_ENTER;
#ifdef LOCALMEM_TX_DSCR
    mem_handle = g_local_dscr_mem_handle;
#else /* LOCALMEM_TX_DSCR */
    mem_handle = g_shared_dscr_mem_handle;
#endif /* LOCALMEM_TX_DSCR */

    /* Allocate buffer for transmit descriptor and reset the required fields */
    tx_dscr = allocate_tx_dscr(mem_handle, is_amsdu);
    if(NULL == tx_dscr)
    {
	printk("@@@TX DSCR alloc failed!\n");
        /* Exception */
        TROUT_FUNC_EXIT;
        return NULL;
    }

    /* Create the status field with packet status, QoS and TID details */
    temp = ((PENDING << 5) & 0xE0);
    temp |= ((is_qos == BTRUE)?BIT4:0);
    temp |= (priority & 0x0F);

    /* Set the status field */
    set_tx_dscr_status((UWORD32 *)tx_dscr, temp);

    /* Set the security parameter to No Encryption as the default value*/
    set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, NO_ENCRYP);

    /* Other than the first fragment, all other fragment's status should be  */
    /* set to 'not pending'.                                                 */
    set_tx_dscr_frag_status_summary((UWORD32 *)tx_dscr, DEFAULT_FRAG_STATUS);

    /* Update the value for latency test */
    latency_test_update(tx_dscr);

    //set_tx_dscr_op_flags(tx_dscr, 0);//dumy add 0816

    set_tx_dscr_tx_service_field((UWORD32 *)tx_dscr, 0);//dumy add 0816

	TROUT_FUNC_EXIT;
    return tx_dscr;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_tx_frame_details                                     */
/*                                                                           */
/*  Description   : This function sets the frame details in the given        */
/*                  transmit descriptor.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                  2) Buffer start address pointer                          */
/*                  3) MAC header start address pointer                      */
/*                  4) MAC header length                                     */
/*                  5) Frame length                                          */
/*                  6) Data offset                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various address, length and offset fields in the     */
/*                  transmit descriptor are set based on the given values.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_tx_frame_details(UWORD8 *tx_dscr, UWORD8 *bsa, UWORD8 *msa,
                          UWORD8 mhlen, UWORD16 frame_len, UWORD8 data_offset)
{
    UWORD8 mh_offset   = 0;
    UWORD8 frag0_offset = 0;

    /* +-------------------------------------------------------------------+ */
    /* | Buffer                      | MAC Header              | Data      | */
    /* +-------------------------------------------------------------------+ */
    /* | <--- MAC Header Offset ---> |                                     | */
    /* +-------------------------------------------------------------------+ */
    /* | <---------------- Fragment 0 Offset ----------------> |           | */
    /* +-------------------------------------------------------------------+ */

    mh_offset    = msa - bsa;
    frag0_offset = mh_offset + mhlen + data_offset;

    set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)bsa);
    set_tx_dscr_mh_offset((UWORD32 *)tx_dscr, mh_offset);
    set_tx_dscr_mh_len((UWORD32 *)tx_dscr, mhlen);
    set_tx_dscr_frame_len((UWORD32 *)tx_dscr, frame_len);
    set_tx_dscr_frag_offset((UWORD32 *)tx_dscr, frag0_offset, 0);
    set_tx_dscr_frag_len((UWORD32 *)tx_dscr, frame_len, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_mgmt_frame                                            */
/*                                                                           */
/*  Description   : This function creates the transmit descriptor for the    */
/*                  management packet to be transmitted and updates the      */
/*                  descriptor fields with required values.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to buffer address                             */
/*                  2) Length of the management frame content                */
/*                  3) The Queue number in which the packet must be added    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory is allocated for the transmit descriptor in   */
/*                  shared memory and the required fields are set to the     */
/*                  values appropriate for management frames.                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T tx_mgmt_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num, UWORD8 idx)
{
    UWORD8   tx_rate        = 0;
    UWORD8   pream          = 0;
    UWORD8   ack_pol        = 0;
    UWORD8   *tx_dscr       = 0;
    UWORD8   da[6]          = {0,};
    CIPHER_T ct             = NO_ENCRYP;
    UWORD16  frame_body_len = len - MAC_HDR_LEN - FCS_LEN;
    UWORD32  phy_tx_mode    = 0;
    UWORD32  retry_set[2]   = {0};

	TROUT_FUNC_ENTER;
	
#ifdef DEBUG_MODE
    g_mac_stats.tx_mgmt_frame_len += len;
    g_mac_stats.tx_mgmt_frame_count++;
#endif /* DEBUG_MODE */

    if(get_wep(buffer_addr) == 1)
    {
        /* If the frame to be Txed has WEP bit set, set the cipher policy */
        ct = (CIPHER_T) g_wep_type;
    }

    /* All management frames are sent at maximum basic rate and with long    */
    /* preamble                                                              */
    tx_rate     = get_min_basic_rate();
    pream       = 1;

    /* Update the retry set information for this frame */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);

    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        pkt_mem_free(buffer_addr);
        TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif        
        return BFALSE;
    }

    /* The destination address is checked and the appropriate ACK policy is  */
    /* set.                                                                  */
    get_DA(buffer_addr, da);
    if(is_group(da) == BTRUE)
        ack_pol = BCAST_NO_ACK;
    else
        /* BUG-ID:IWLANNPLFSW_1 */
        /* Management frames should always be sent at Normal ACK Policy */
        ack_pol = NORMAL_ACK;

    /* Set the required transmit descriptor contents */
    set_tx_params(tx_dscr, tx_rate, pream, ack_pol, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, buffer_addr, 0, MAC_HDR_LEN, frame_body_len);

    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

    /* For shared key authentication the security fields need to be set  */
    set_tx_security(tx_dscr, ct, 0, idx);

    /* Set the receiver address LUT index */
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);

    /* Update the TSF timestamp in the Tx-desc */
    //update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);	//chengwg mask, update before send to trout.

    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception. Do nothing. */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
        TROUT_FUNC_EXIT;
        return BFALSE;
    }
    
	TROUT_FUNC_EXIT;
    return BTRUE;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_uc_mgmt_frame                                         */
/*                                                                           */
/*  Description   : This function creates the transmit descriptor for the    */
/*                  management packet to be transmitted and updates the      */
/*                  descriptor fields with required values.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to buffer address                             */
/*                  2) Length of the management frame content                */
/*                  3) The Queue number in which the packet must be added    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory is allocated for the transmit descriptor in   */
/*                  shared memory and the required fields are set to the     */
/*                  values appropriate for management frames.                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         26 05 2005   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void tx_uc_mgmt_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num, UWORD8 idx)
{
    UWORD8   tx_rate        = 0;
    UWORD8   pream          = 0;
    UWORD8   *tx_dscr       = 0;
    UWORD8   da[6]          = {0,};
    CIPHER_T ct             = NO_ENCRYP;
    UWORD16  frame_body_len = len - MAC_HDR_LEN - FCS_LEN;
    UWORD32  phy_tx_mode    = 0;
    UWORD32  retry_set[2]   = {0};

	TROUT_FUNC_ENTER;
	
#ifdef DEBUG_MODE
    g_mac_stats.tx_mgmt_frame_count++;
    g_mac_stats.tx_mgmt_frame_len += len;
#endif /* DEBUG_MODE */

    if(get_wep(buffer_addr) == 1)
    {
        /* If the frame to be Txed has WEP bit set, set the cipher policy */
        ct = (CIPHER_T) g_wep_type;
    }

    /* All management frames are sent at maximum basic rate and with long    */
    /* preamble                                                              */
    tx_rate     = get_max_basic_rate();

    pream       = 1;

    /* Update the retry set information for this frame */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);

    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        pkt_mem_free(buffer_addr);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set the required transmit descriptor contents */
    set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, buffer_addr, 0, MAC_HDR_LEN, frame_body_len);

    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

    /* For shared key authentication the security fields need to be set  */
    set_tx_security(tx_dscr, ct, 0, idx);

    /* Set the receiver address LUT index */
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);

    /* Update the TSF timestamp in the Tx-desc */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    /* Get the DA */
    get_DA(buffer_addr, da);

    if(BFALSE == is_ps_buff_pkt((UWORD8 *)find_entry(da), da, tx_dscr))
    {
        if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
        {
            /* Exception. Do nothing. */
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
            free_tx_dscr((UWORD32 *)tx_dscr);
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_control_frame_dscr                               */
/*                                                                           */
/*  Description   : This function prepares the transmit descriptor for the   */
/*                  control packet.                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to buffer address                             */
/*                  2) Length of the control frame content                   */
/*                  3) The Queue number for the control frame                */
/*                  4) If QOS is enabled for the frame                       */
/*                  5) The Priority of the outgoing frame                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory is allocated for the transmit descriptor in   */
/*                  shared memory and the required fields are set to the     */
/*                  values appropriate for control frames.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *prepare_control_frame_dscr(UWORD8 *buffer_addr, UWORD16 len,
                                   void *entry, UWORD8 q_num, BOOL_T is_qos,
                                   UWORD8 priority)
{
    UWORD8 tx_rate         = 0;
    UWORD8 pream           = 0;
    UWORD8 ack_policy      = 0;
    UWORD8 *tx_dscr        = 0;
    UWORD16 frame_body_len = len - CONTROL_HDR_LEN - FCS_LEN;
    UWORD32 phy_tx_mode    = 0;
    UWORD32  retry_set[2]  = {0};

    /* All management frames are sent at maximum basic rate and with long    */
    /* preamble                                                              */
    tx_rate     = get_max_basic_rate();
    pream       = 1;

    /* Update the retry set information for this frame */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);

    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);

#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
    /* TX-Dscr taken from AMSDU Pool. Logic is AMSDU and Block-Ack may-not*/
   /* happen at same time.                                                */
    if(tx_dscr == NULL)
        tx_dscr = create_default_tx_dscr(is_qos, priority, 1);
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

    if(tx_dscr == NULL)
    {
        if(buffer_addr != NULL)
            pkt_mem_free(buffer_addr);

        return 0;
    }

    /* Get the ACK policy to be set for this Control frame */
    if(buffer_addr != NULL)
        ack_policy = get_ctrl_frame_ack_policy(buffer_addr);

    /* Set the required transmit descriptor contents */
    set_tx_params(tx_dscr, tx_rate, pream, ack_policy, phy_tx_mode, retry_set);

    if(buffer_addr != NULL)
    {
        set_tx_buffer_details(tx_dscr, buffer_addr, 0, CONTROL_HDR_LEN,
                          frame_body_len);
    }

    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

    /* Set the receiver address LUT index */
    set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry(entry), priority, tx_rate);

    /* Update the TSF timestamp in the Tx-desc */
    //update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    return tx_dscr;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_tx_mib                                            */
/*                                                                           */
/*  Description   : Update the MIB related to Tx.                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to transmit descriptor address                */
/*                  2) Number of descriptors for the current MPDU            */
/*                  3) Pointer to Station Entry Structure                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All the Tx related MIB are updated here                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_tx_mib(UWORD8 *tx_dscr, UWORD8 num_dscr, void *entry)
{
    UWORD32 status    = 0;
    UWORD16 frag_len  = 0;
    UWORD8  *da       = 0;
    UWORD8  lrl       = 0;
    UWORD8  srl       = 0;
    UWORD8  cts_fail  = 0;
    UWORD8  rts_succ  = 0;
    UWORD8  retry_cnt = 0;
    UWORD8  frag_num  = 0;
    UWORD8  *msa      = 0;
    UWORD16 num_bytes = 0;
    UWORD8  ackpol    = 0;
    UWORD8  cnt       = 0;

    lrl       = get_tx_dscr_num_long_retry((UWORD32*)tx_dscr);
    srl       = get_tx_dscr_num_short_retry((UWORD32*)tx_dscr);
    cts_fail  = get_tx_dscr_cts_failure((UWORD32 *)tx_dscr);
    rts_succ  = get_tx_dscr_rts_success((UWORD32 *)tx_dscr);
    num_bytes = get_tx_dscr_frame_len((UWORD32 *)tx_dscr);
    ackpol    = get_tx_dscr_ack_policy((UWORD32 *)tx_dscr);
    msa       = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);

    da        = get_address1_ptr(msa);
    retry_cnt = lrl + srl;

    /* RTS Success count for the MSDU */
    mset_RTSSuccessCount(mget_RTSSuccessCount() + rts_succ);

    /* CTS Failure count */
    mset_RTSFailureCount(mget_RTSFailureCount() + cts_fail);

    /* ACK failure count */
    mset_ACKFailureCount(mget_ACKFailureCount() + retry_cnt - cts_fail);

	#ifdef IBSS_BSS_STATION_MODE
	//chenq add for check mac state
	if(retry_cnt >= 1)
		g_tx_retries++;
	#endif	

    /* Retry count */
    if(retry_cnt >= 1)
        mset_RetryCount(mget_RetryCount() + 1);

    if(retry_cnt > 1)
        mset_MultipleRetryCount(mget_MultipleRetryCount() + 1);


    /* If Interrupt is raised because of Life time expiry don't update MIB */
    if(get_tx_dscr_status((UWORD32 *)tx_dscr) != TX_TIMEOUT)
    {
        if(num_dscr == 1)
        {
            frag_len = get_tx_dscr_frag_len((UWORD32 *)tx_dscr, frag_num);
            status   = get_tx_dscr_frag_status_summary((UWORD32 *)tx_dscr);

	        while(frag_len)
	        {
	            /* If you are reading wrong tx descriptor, don't go in infinite  */
	            /* loop                                                          */
	            if(frag_num >= 16)
	                break;

	            switch(status & MPDU_STATUS_MASK)
	            {
		            case NON_PENDING_MPDU:
		            {
		                mset_TransmittedFragmentCount
		                                (mget_TransmittedFragmentCount() + 1);
		            }
		            break;

		            case NORMAL_RETRY:
		            {
		                /* Failed to transmit the frame */
		                mset_FailedCount(mget_FailedCount() + 1);

		                /* Protocol dependent MIB updates */
		                update_tx_mib_prot(msa, retry_cnt, cts_fail, rts_succ,
		                                   frag_num, num_bytes, BFALSE);

		                return;
		            }

		            case PENDING_MPDU:
		            {
		                /* Exception. */
		            }
		            break;

		            case BLOCK_ACK_REQD:
		            {
		                /* Exception */
		            }
		            break;
	            }

	            frag_num++;
	            status >>= 2;
	            frag_len = get_tx_dscr_frag_len((UWORD32 *)tx_dscr, frag_num);
	        }
        }

        if(is_group(da) == BTRUE)
        {
            mset_MulticastTransmittedFrameCount(
                                mget_MulticastTransmittedFrameCount() + num_dscr);
        }

        mset_TransmittedFrameCount(mget_TransmittedFrameCount() + num_dscr);

        /* Update required MIB for the aggregation of frames transmitted */
        update_aggr_tx_mib(num_dscr);

        /* Protocol dependent MIB updates */
        while(1)
        {
            update_tx_mib_prot(msa, retry_cnt, cts_fail, rts_succ,
                               frag_num, num_bytes, BTRUE);

            if(++cnt >= num_dscr)
                break;

            tx_dscr   = (UWORD8 *)get_tx_dscr_next_addr((UWORD32 *)tx_dscr);
            if(tx_dscr == NULL)
                return;
            msa       = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);
            num_bytes = get_tx_dscr_frame_len((UWORD32 *)tx_dscr);
            retry_cnt = cts_fail = rts_succ = 0;
            frag_num  = 1;
        }

        /* Autorate stats are updated only for in ENABLED state and when ackp*/
        /* is Normal-ACK or No-ACK.                                          */
        if((get_mac_state() ==  ENABLED) &&
           (ackpol != COMP_BLOCK_ACK) && (ackpol != COMP_BAR))
        {
            ar_tx_success(entry, retry_cnt);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_tx_buffer_details                                    */
/*                                                                           */
/*  Description   : This function sets the buffer details for the msdu       */
/*                  header and the body when the frame to be txd has the     */
/*                  header & body contiguous.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx Descriptor                          */
/*                  2) Pointer to the buffer handle                          */
/*                  3) Offset to the start of the frame header.              */
/*                  4) Length of the frame header.                           */
/*                  5) Length of the frame body.                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_tx_buffer_details(UWORD8  *tx_dscr, UWORD8 *buffer_addr,
                           UWORD16 hdr_offset, UWORD16 hdr_len,
                           UWORD16 data_len)
{
    buffer_desc_t buff_desc = {0};

    /* Set the frame header buffer details */
    set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)buffer_addr);
    set_tx_dscr_mh_len((UWORD32 *)tx_dscr, hdr_len);
    set_tx_dscr_mh_offset((UWORD32 *)tx_dscr, hdr_offset);

    /* Set the frame body buffer details */
    buff_desc.buff_hdl    = (UWORD8 *)buffer_addr;
    buff_desc.data_length = data_len;
    buff_desc.data_offset = hdr_offset + hdr_len;

    set_tx_submsdu_info(tx_dscr, &buff_desc, 1, hdr_len);

#ifdef TX_MACHDR_IN_DSCR_MEM
    /* When Mac header for MSDUs are implemented as part of     */
    /* descriptor memory, the buffer  handle has two users: Frame  */
    /* header & Frame Body. The extra user is registered with the  */
    /* memory manager.                                             */
    mem_add_users(g_shared_pkt_mem_handle, buffer_addr, 1);

#endif /* TX_MACHDR_IN_DSCR_MEM */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_msdu_frame                                            */
/*                                                                           */
/*  Description   : This function adds the frame to the transmit queue.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Association entry.                     */
/*                  2) Destination address of the frame.                     */
/*                  3) Q Number                                              */
/*                  4) Tx Descriptor                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MSDU is added to the Tx-Q if permitted by the PS     */
/*                  mode of the receiver.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T, BTRUE if the packet has been added to the H/w    */
/*                          queue for transmission. BFALSE otherwise.        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T tx_msdu_frame(UWORD8 *entry, UWORD8 *da, UWORD8 priority, UWORD8 q_num,
                     UWORD8 *tx_dscr)
{
    BOOL_T ret_val = BFALSE;

	TROUT_FUNC_ENTER;
    /* Check if the packet can be transmitted or has to be queued */
    if(buffer_tx_packet(entry, da, priority, q_num, tx_dscr) == BFALSE)
    {
    	TX_PATH_DBG("%s: add tx pkt\n", __func__);
        if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
        {
            /* Exception. Do nothing. */
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
            print_log_debug_level_1("\n[DL1][INFO][Tx] {Packet not added to queue}");
			//caisf add 2013-02-15
            if(g_mac_net_stats)
                g_mac_net_stats->tx_dropped++;
				
            free_tx_dscr((UWORD32 *)tx_dscr);
        }
        else
        {
            ret_val = BTRUE;
            print_log_debug_level_1("\n[DL1][INFO][Tx] {Packet added to queue}");
        }
    }

	TROUT_FUNC_EXIT;
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : free_tx_dscr                                             */
/*                                                                           */
/*  Description   : This function frees all the buffers associated with a Tx */
/*                  Descriptor.                                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx descriptor.                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The buffers for MSDU header, sub-MSDUs and sub-MSDU      */
/*                  information table are freed. Finally, the tx descriptor  */
/*                  is also freed.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void free_tx_dscr(UWORD32 *tx_dscr)
{
    mem_handle_t *mem_handle = 0;

#ifndef MAC_HW_UNIT_TEST_MODE
    UWORD8  *buff_hdl = 0;
    UWORD32 indx = 0;
    UWORD32 num_sub_msdu = get_tx_dscr_num_submsdu((UWORD32 *)tx_dscr);

#ifdef TX_MACHDR_IN_DSCR_MEM
    /* Only when MAC header is part of desc */
    buff_hdl = (UWORD8 *)get_tx_dscr_buffer_addr(tx_dscr);
    if((NULL != buff_hdl) &&
       (BFALSE == is_buffer_in_tx_dscr((UWORD8 *)tx_dscr, buff_hdl)))
    {
        /* Get the memory pool to which the buffer belongs */
        mem_handle = get_mem_pool_hdl((void *)buff_hdl);
        mem_free(mem_handle, buff_hdl);
    }

#endif /* TX_MACHDR_IN_DSCR_MEM */

    /* Free all the sub-MSDU buffers */
    for(indx = 0; indx < num_sub_msdu; indx++)
    {
        buff_hdl = (UWORD8 *)get_tx_dscr_submsdu_buff_hdl(tx_dscr, indx);
        if(BFALSE == is_buffer_in_tx_dscr((UWORD8 *)tx_dscr, buff_hdl))
        {
            pkt_mem_free(buff_hdl);
        }
    }
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Get the memory pool to which the descriptor buffer belongs */
#ifdef LOCALMEM_TX_DSCR
    mem_handle = get_mem_pool_hdl((void *)tx_dscr);
#else /* LOCALMEM_TX_DSCR */
    mem_handle = g_shared_dscr_mem_handle;
#endif /* LOCALMEM_TX_DSCR */

    free_tx_dscr_buffer(mem_handle, (UWORD8*) tx_dscr);
}

#ifdef BURST_TX_MODE

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_burst_mode_frame                                      */
/*                                                                           */
/*  Description   : This function creates the transmit descriptor for the    */
/*                  burst mode packet to be transmitted and updates the      */
/*                  descriptor fields with required values.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to buffer address                             */
/*                  2) Length of the frame content                           */
/*                  3) The Queue number in which the packet must be added    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory is allocated for the transmit descriptor in   */
/*                  shared memory and the required fields are set to the     */
/*                  values appropriate for control frames.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void tx_burst_mode_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num, UWORD8 idx)
{
    UWORD8  tx_rate      = 0;
    UWORD8  pream        = 0;
    UWORD8  ack_pol      = 0;
    UWORD8  *tx_dscr[10] = {0};
    UWORD8  i            = 0;
    UWORD32 phy_tx_mode  = 0;
    UWORD32 retry_set[2] = {0};

    tx_rate = get_curr_tx_rate();
    pream   = get_preamble(get_curr_tx_rate());

    /* Update the retry set information for this frame */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);

    ack_pol = BCAST_NO_ACK;

    for(i = 0;i < 10;i++)
    {
        /* Create the transmit descriptor and set the contents */
        tx_dscr[i] = create_default_tx_dscr(0, 0, 0);
        if(tx_dscr[i] == NULL)
        {
            pkt_mem_free(buffer_addr);
            return;
        }

    }
    for(i = 0;i < 10;i++)
    {
        /* Set the required transmit descriptor contents */
        set_tx_params(tx_dscr[i], tx_rate, pream, ack_pol, phy_tx_mode,
                      retry_set);
        set_tx_frame_details(tx_dscr[i], buffer_addr,
                                            buffer_addr, MAC_HDR_LEN, len, 0);
        set_tx_dscr_q_num((UWORD32 *)tx_dscr[i], q_num);

        if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr[i]) != QMU_OK)
        {
            /* Exception. Do nothing. */
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        }
        set_tx_dscr_next_addr((UWORD32 *)tx_dscr[i], (UWORD32)tx_dscr[(i + 1) % 10]);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_burst_mode_frame                                 */
/*                                                                           */
/*  Description   : This function prepares the frame required for burst      */
/*                  transmission mode.                                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the burst mode frame                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the frame are set as required      */
/*                                                                           */
/*  Outputs       : The contents of the given burst mode frame have been set */
/*                                                                           */
/*  Returns       : frame length                                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_burst_mode_frame(UWORD8 *data, UWORD16 len)
{
    UWORD16 i = 0;
    /*************************************************************************/
    /*                        Burst Frame Format                             */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |996       |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)DATA);

    /* Set Address1 to broadcast address */
    set_address1(data, mget_bcst_addr());

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_StationID());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    for(i = 0; i <(BURST_TX_MODE_FRAME_LEN - MAC_HDR_LEN - FCS_LEN); i++)
    {
        data[MAC_HDR_LEN + i]     = (UWORD8)i;
    }

    return BURST_TX_MODE_FRAME_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initiate_burst_tx_mode                                   */
/*                                                                           */
/*  Description   : This function initiates the burst transmission mode      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : A frame is prepared and sent to the MAC HW for           */
/*                  transmission                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initiate_burst_tx_mode(mac_struct_t* mac)
{
    UWORD8  *burst_mode_frame    = 0;
    UWORD16 burst_mode_frame_len = 0;

    burst_mode_frame  = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    if(burst_mode_frame == NULL)
    {
        return;
    }

    burst_mode_frame_len = prepare_burst_mode_frame(burst_mode_frame,
                                  burst_mode_frame_len);

    /* Buffer this frame in the Memory Queue for transmission */
    tx_burst_mode_frame(burst_mode_frame, burst_mode_frame_len, HIGH_PRI_Q, 0);

    /* Initialize the MAC state to ENABLED */
    set_mac_state(ENABLED);

    /* Convey the current MAC status to Host */
	//chenq mask
    //send_mac_status(MAC_CONNECTED);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mac_fsm_bt                                          */
/*                                                                           */
/*  Description   : This function initiates the MAC FSM in case of burst     */
/*                  transmission mode                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Initialize all MAC FSM functions to null_fn so that      */
/*                  no operation happens in burst tx mode                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void init_mac_fsm_bt(mac_struct_t* mac)
{
    UWORD32 i=0,j=0;

    for(i = 0; i < MAX_MAC_FSM_STATES; i++)
    {
        for(j = 0; j < MAX_MAC_FSM_INPUTS; j++)
        {
            mac->fsm[i][j] = null_fn;
        }
    }

    /* Initialize the MAC state to DISABLED */
    set_mac_state(DISABLED);
}
#endif /* BURST_TX_MODE */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : allocate_tx_dscr                                         */
/*                                                                           */
/*  Description   : This function allocates the TX descriptor buffers from   */
/*                  the given memory pool, resets required fields of same    */
/*                  and links the buffers if more than 1 are allocated.      */
/*                                                                           */
/*  Inputs        : 1) Memory pool handle                                    */
/*                  2) Flag to indicate if the TX descriptor is for an AMSDU */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : For normal MSDUs a single TX descriptor is allocated.    */
/*                  For an AMSDU, a second TX descriptor buffer is allocated */
/*                  for the sub-MSDU information table. Note that the TX     */
/*                  descriptor buffer includes space for a single sub-MSDU   */
/*                  entry at an offset of TX_DSCR_LEN and is sufficient for  */
/*                  the non-AMSDU case.                                      */
/*                  The required fields of the allocated TX descriptor       */
/*                  buffer are cleared and the sub-MSDU information table    */
/*                  pointer field is updated:                                */
/*                  For non-AMSDU case, with the address of the offset in    */
/*                  the same TX descriptor buffer.                           */
/*                  For AMSDU case, with the address of the second TX        */
/*                  descriptor buffer allocated                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8 *, Pointer to transmit descriptor (with sub-MSDU  */
/*                            information table pointer updated to required  */
/*                            sub-MSDU information table buffer or location) */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 *allocate_tx_dscr(mem_handle_t *mem_handle, UWORD8 is_amsdu)
{
    UWORD8 *tx_dscr = NULL;

#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
    UWORD16 tx_dscr_size = 0;

	TROUT_FUNC_ENTER;
	
    if(is_amsdu == 0) /* Sub-MSDU table has only 1 entry */
        tx_dscr_size = TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE;
    else /* Sub-MSDU table has multiple entries */
        tx_dscr_size = TX_DSCR_LEN + MAX_SUBMSDU_TABLE_SIZE;

    tx_dscr = (UWORD8 *)mem_alloc(mem_handle, tx_dscr_size);
    if(NULL == tx_dscr)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return NULL;
    }

#ifdef MAC_HW_UNIT_TEST_MODE
    /* Set all the fields to zero (default values) and the status to pending */
    mem_set(tx_dscr, 0, tx_dscr_size);
#else /* MAC_HW_UNIT_TEST_MODE */
    mem_set(tx_dscr, 0, tx_dscr_size); // tanxy 0919
    reset_txdscr_fields((UWORD32 *)tx_dscr);
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Set the Pointer to the sub-MSDU information table in the tx descriptor */
    set_tx_dscr_submsdu_info((UWORD32 *)tx_dscr,
                             (UWORD32)(tx_dscr + TX_DSCR_LEN));

#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

    UWORD8 *sub_msdu_info = 0;
	TROUT_FUNC_ENTER;
    /* Allocate memory for the transmit descriptor */
    tx_dscr = (UWORD8 *)mem_alloc(mem_handle, TX_DSCR_BUFF_SZ);

	/* Exception */
    if(NULL == tx_dscr)
    {
		TROUT_FUNC_EXIT;
        return NULL;
    }

    /* For AMSDU case allocate a second transmit descriptor buffer. This     */
    /* shall be used as the sub-MSDU table for the AMSDU. For non-AMSDU case */
    /* the sub-MSDU table is included in the TX descriptor itself, at the    */
    /* end. The sub-MSDU table shall include space for 1 sub-MSDU entry in   */
    /* this case.                                                            */
    if(is_amsdu == 1)
    {
		sub_msdu_info = (UWORD8 *)mem_alloc(mem_handle, TX_DSCR_BUFF_SZ);
        if(NULL == sub_msdu_info)
        {
            mem_free(mem_handle, tx_dscr);
#ifdef DEBUG_MODE
			g_mac_stats.no_mem_count++;
#endif
			TROUT_FUNC_EXIT;
            return NULL;
        }
    }
    else
    {
		sub_msdu_info = (UWORD8 *)(tx_dscr + TX_DSCR_LEN);
	}

#if 0	//masked by chengwg.
#ifdef MAC_HW_UNIT_TEST_MODE
    /* Set all the fields to zero (default values) and the status to pending */
    mem_set(tx_dscr, 0, TX_DSCR_BUFF_SZ);
#else /* MAC_HW_UNIT_TEST_MODE */
	//*(UWORD32 *)tx_dscr = 0;  //Hugh: fix RTS bug.
    reset_txdscr_fields((UWORD32 *)tx_dscr);
#endif /* MAC_HW_UNIT_TEST_MODE */
#else
	mem_set(tx_dscr, 0, TX_DSCR_BUFF_SZ);	//modify by chengwg.
#endif

	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info((UWORD32 *)tx_dscr, (UWORD32)(sub_msdu_info));

#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

	TROUT_FUNC_EXIT;
    return tx_dscr;
}

#ifdef TROUT_B2B_TEST_MODE

#if 0
static UWORD8 tx_desc_sample[152] = {
    0x00, 0x0a, 0x03, 0x30, 0x00, 0x34, 0x00, 0x00, 0x42, 0x01, 0x01, 0x00, 0x7c, 0x38, 0x01, 0x31,
    0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xf6, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x1b, 0x01, 0x00, 0x2c, 0x1b, 0x01, 0x00,
    0x03, 0x01, 0x1a, 0x00, 0x03, 0x03, 0x0f, 0x0f, 0x0b, 0x0b, 0x0b, 0x00, 0x40, 0xe9, 0x00, 0xce,
    0x34, 0x1b, 0x01, 0x00, 0xdc, 0x05, 0x28, 0x00
};
#else

static UWORD8 tx_desc_sample[152] = {
#if 1
//rate: 24Mbps
    0x00, 0x09, 0x03, 0x30, 0x00, 0x34, 0x00, 0x00, 0x42, 0x01, 0x01, 0x00, 0x80, 0x15, 0x01, 0x31,
#else
//rate: 54Mbps
    0x00, 0x0c, 0x03, 0x30, 0x00, 0x34, 0x00, 0x00, 0x42, 0x01, 0x01, 0x00, 0x80, 0x15, 0x01, 0x31,
#endif
    0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0x02, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x6b, 0x17, 0xce, 0x28, 0xe9, 0x13, 0xce,
    0x03, 0x01, 0x1a, 0x00, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x98, 0xe8, 0x13, 0xce,
    0xf0, 0x6b, 0x17, 0xce, 0xe8, 0x05, 0x28, 0x00
};

static UWORD8 tx_pkt_sample[1552] = {
    0x88, 0x01, 0x17, 0xce, 0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0x06, 0x06, 0x06, 0x06, 0x06, 0x08,
    0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00,
    0x45, 0x00, 0x05, 0xdc, 0x00, 0x00, 0x40, 0x00, 0x40, 0x01, 0xb1, 0x61, 0xc0, 0xa8, 0x01, 0x6e,
    0xc0, 0xa8, 0x01, 0x01, 0x08, 0x00, 0x08, 0x57, 0x6b, 0x0a, 0x00, 0x01, 0x16, 0x91, 0x20, 0x4d,
    0xd0, 0x6c, 0x04, 0x00, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0x00, 0x00, 0x00, 0x00
};
#endif

extern UWORD8 trout_b2b_descr_pkt[];
extern int descpkt_cpy_to_trout(void);

#if 1

#if 0
int trout_b2b_pkt_from_host(const __user char *pkt, int pkt_len)
{
    descpkt_cpy_to_trout();
}

#else
int trout_b2b_pkt_from_host(const __user char *pkt, int pkt_len)
{
    UWORD8 *tx_descr;    
    UWORD8  *buffer;
    UWORD32 *tx_descr_32;
    UWORD8 q_num = 0;

    buffer = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer == NULL)
    {
        printk("alloc pkt buffer failed.\n");
        return -3;
    }

    tx_descr = create_default_tx_dscr(0, 0, 0);
    if(tx_descr == NULL)
    {
        printk("alloc tx descr failed.\n");
        pkt_mem_free(buffer);
        return -2;
    }

    //fill descr field.
    memcpy(tx_descr, tx_desc_sample, 152);
    tx_descr_32 = (UWORD32 *)tx_descr;
    tx_descr_32[4] = 0; //next is null.
    tx_descr_32[27] = (tx_descr_32[27] & 0xFFFF0000) | (pkt_len - 14);
    set_tx_dscr_buffer_addr(tx_descr_32, (UWORD32)buffer);
	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info(tx_descr_32, (UWORD32)(tx_descr + TX_DSCR_LEN));
    q_num = get_tx_dscr_q_num(tx_descr_32);
    //set_tx_dscr_q_num(tx_descr_32, q_num);
    
    //fill submsdu table
    tx_descr_32[36] = (UWORD32)buffer;
    tx_descr_32[37] = (tx_descr_32[37] & 0xFFFF0000) | ((pkt_len - ((tx_descr_32[37]>>16) & 0xFFFF)));
    
    //fill buffer with pkt data.
    //memcpy(buffer, tx_pkt_sample, pkt_len);
	if (copy_from_user(buffer, (char *)pkt, pkt_len))
	{
        printk("copy_from_user(pkt) failed: len=%d\n.", pkt_len);
        free_tx_dscr((UWORD32 *)tx_descr);
        return -EFAULT;
    }

    //printk("pkt_len=%u\n", pkt_len);
    //hex_dump("b2b mac header:", buffer, 40);

    //set source mac address
    //set_address2(buffer, mget_bssid());
    set_address2(buffer, mget_StationID());
    set_address3(buffer, mget_StationID());

    //add to queue and send out.
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_descr) != QMU_OK)
    {
        /* Exception. Do nothing. */
        //printk("[Tx] add to Q fail.\n");
        free_tx_dscr((UWORD32 *)tx_descr);
        return -1;
    }

    return pkt_len;
}
#endif
#else

int trout_b2b_pkt_from_host(const __user char *pkt, int pkt_len)
{
    UWORD8 *tx_descr;    
    UWORD8  *buffer;
    UWORD32 *tx_descr_32;
    UWORD8 q_num = 0;

    buffer = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer == NULL)
    {
        printk("alloc pkt buffer failed.\n");
        return -3;
    }

    tx_descr = create_default_tx_dscr(0, 0, 0);
    if(tx_descr == NULL)
    {
        printk("alloc tx descr failed.\n");
        pkt_mem_free(buffer);
        return -2;
    }

    //fill descr field.
    memcpy(tx_descr, tx_desc_sample, sizeof(tx_desc_sample));
    tx_descr_32 = (UWORD32 *)tx_descr;
    //tx_descr_32[4] = 0; //next is null.
    //tx_descr_32[27] = (tx_descr_32[27] & 0xFFFF0000) | (pkt_len - 14);
    set_tx_dscr_buffer_addr(tx_descr_32, (UWORD32)buffer);
	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info(tx_descr_32, (UWORD32)(tx_descr + TX_DSCR_LEN));
    q_num = get_tx_dscr_q_num(tx_descr_32);
    //set_tx_dscr_q_num(tx_descr_32, q_num);
    
    //fill submsdu table
    tx_descr_32[36] = (UWORD32)buffer;
//    tx_descr_32[37] = (tx_descr_32[37] & 0xFFFF) | ((pkt_len - ((tx_descr_32[37]>>16) & 0xFFFF)));
    
    //fill buffer with pkt data.
	if (copy_from_user(buffer, (char *)pkt, pkt_len))
	{
        printk("copy_from_user(pkt) failed: len=%d\n.", pkt_len);
        free_tx_dscr((UWORD32 *)tx_descr);
        return -EFAULT;
    }

    printk("pkt_len=%u\n", pkt_len);
//    hexdump(buffer, 40);

    //set source mac address
//    set_address2(buffer, mget_StationID());

    //add to queue and send out.
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_descr) != QMU_OK)
    {
        /* Exception. Do nothing. */
        printk("[Tx] add to Q fail.\n");
        free_tx_dscr((UWORD32 *)tx_descr);
        return -1;
    }

    return pkt_len;
}
#endif

EXPORT_SYMBOL(trout_b2b_pkt_from_host);

#endif


#ifdef TROUT_WIFI_NPI

static UWORD8 tx_desc_sample[152] = {
#if 1
//rate: 24Mbps
    0x00, 0x09, 0x03, 0x30, 0x00, 0x34, 0x00, 0x00, 0x42, 0x01, 0x01, 0x00, 0x80, 0x15, 0x01, 0x31,
#else
//rate: 54Mbps
    0x00, 0x0c, 0x03, 0x30, 0x00, 0x34, 0x00, 0x00, 0x42, 0x01, 0x01, 0x00, 0x80, 0x15, 0x01, 0x31,
#endif
    0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0x02, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x6b, 0x17, 0xce, 0x28, 0xe9, 0x13, 0xce,
    0x03, 0x01, 0x1a, 0x00, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x98, 0xe8, 0x13, 0xce,
    0xf0, 0x6b, 0x17, 0xce, 0xe8, 0x05, 0x28, 0x00
};

static UWORD8 tx_pkt_sample[2500] = {
#if 0
    0x88, 0x01, 0x17, 0xce, 0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0x06, 0x06, 0x06, 0x06, 0x06, 0x08,
#else
    0x88, 0x02, 0x17, 0xce, 0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0x06, 0x06, 0x06, 0x06, 0x06, 0x08,
#endif

    0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xd8, 0x5d, 0x4c, 0x5c, 0xdf, 0x0c, 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00,
    0x45, 0x00, 0x05, 0xdc, 0x00, 0x00, 0x40, 0x00, 0x40, 0x01, 0xb1, 0x61, 0xc0, 0xa8, 0x01, 0x6e,
    0xc0, 0xa8, 0x01, 0x01, 0x08, 0x00, 0x08, 0x57, 0x6b, 0x0a, 0x00, 0x01, 0x16, 0x91, 0x20, 0x4d,
    0xd0, 0x6c, 0x04, 0x00, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3,
    0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53,
    0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
    0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0x00, 0x00, 0x00, 0x00,0x00,
};
#if 0
static char g_rf_test_data_pkt_hdr[] = {
    0x08, 0x01, 0x2c, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x06, 0x06, 0x06, 0x06, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00
};
#endif
static char g_rf_test_qos_data_pkt_hdr[] = {
    0x88, 0x01, 0x2c, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x06, 0x06, 0x06, 0x06, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00
};

extern UWORD8 get_phy_rate(UWORD8 mbps);
extern UWORD8 get_curr_tx_rate(void);
extern void set_tx_dscr_tx_pow_level(UWORD32 *pkt_dscr_ptr, UWORD32 value);
//extern UWORD32 tx_power_level;
//extern UWORD8 g_mac_addr[6];
//extern UWORD8 g_tx_rate;
static UWORD32 npi_tx_count[2] = {0,0};

UWORD8 *  create_tx_packet(int pkt_len,UWORD8 * q_num)
{
	UWORD8 *tx_descr;    
    UWORD8  *buffer;
    UWORD32 *tx_descr_32;
	//int i = 0;
	UWORD8 tx_rate = 1;
    //UWORD8 phy_rate = 0;
    UWORD32  retry_set[2]   = {0};
    UWORD32  phy_tx_mode    = 0;
    UWORD8   pream          = 0;
    UWORD8   ack_pol        = 0;
    UWORD8 *sta_mac = NULL;
	*q_num = 0;
    buffer = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer == NULL)
    {
        printk("npi: alloc pkt buffer failed %u. \n", ++npi_tx_count[0]);
        return NULL;
    }

    tx_descr = create_default_tx_dscr(0, 0, 0);
    if(tx_descr == NULL)
    {
        printk("npi: alloc tx descr failed %u. list count = %d\n", ++npi_tx_count[1], g_q_handle.tx_handle.tx_list_count);
        pkt_mem_free(buffer);
        return NULL;
    }
	
	if(g_user_tx_rate)
		tx_rate = g_user_tx_rate;
	
	switch(tx_rate)
	{
		case 7: // 6.5M
			tx_rate = 0x80;
			break;
		case 13: // 13M
			tx_rate = 0x81;
			break;
		case 19: // 19.5M
			tx_rate = 0x82;
			break;
		case 26: // 26M
			tx_rate = 0x83;
			break;
		case 39: // 39M
			tx_rate = 0x84;
			break;
		case 52: // 52M
			tx_rate = 0x85;
			break;
		case 58: // 58.5M
			tx_rate = 0x86;
			break;
		case 65: // 65M
			tx_rate = 0x87;
			break;

		default:
			break;
	}
	if(0x80 <= tx_rate && tx_rate <= 0x87)
	{
		memcpy(tx_pkt_sample, g_rf_test_qos_data_pkt_hdr, 
					sizeof(g_rf_test_qos_data_pkt_hdr));
	}

	//fill descr field.
	memcpy(tx_descr, tx_desc_sample, 152);
#if 1
	update_retry_rate_set(0, tx_rate, 0, retry_set);
	pream = get_preamble(tx_rate);
	phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	// no ack
	ack_pol = BCAST_NO_ACK;

	/* Set the required transmit descriptor contents */
	set_tx_params(tx_descr, tx_rate, pream, ack_pol, phy_tx_mode, retry_set);
#endif



	/* Get the PHY transmit mode based on the transmit rate and preamble */
	// phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	tx_descr_32 = (UWORD32 *)tx_descr;
	tx_descr_32[4] = 0; //next is null.
	tx_descr_32[27] = (tx_descr_32[27] & 0xFFFF0000) | (pkt_len - 14);
	set_tx_dscr_buffer_addr(tx_descr_32, (UWORD32)buffer);
	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info(tx_descr_32, (UWORD32)(tx_descr + TX_DSCR_LEN));
	*q_num = get_tx_dscr_q_num(tx_descr_32);
	tx_descr_32[36] = (UWORD32)buffer;
	tx_descr_32[37] = (tx_descr_32[37] & 0xFFFF0000) | ((pkt_len - ((tx_descr_32[37]>>16) & 0xFFFF)));

	memcpy(buffer, tx_pkt_sample, pkt_len);

#if 0
	if (tx_power_level > 0)
	{
		printk("set_tx_power: %u\n", tx_power_level);
		//set_tx_dscr_tx_pow_level(buffer, tx_power_level);
		tx_descr[5] = tx_power_level;
		printk("after set, tx power is: %u\n", tx_descr[5]);
	}
#endif
	sta_mac = mget_StationID();
	//set_address2(buffer, g_mac_addr);
	//set_address3(buffer, g_mac_addr);
	set_address2(buffer, sta_mac);
	set_address3(buffer, sta_mac);
	return tx_descr;
	
}

UWORD8 qmu_add_tx_packet_no_send(qmu_tx_handle_t *tx_handle, UWORD8 q_num, UWORD8 *tx_dscr);

UWORD8 *  npi_update_pkt(UWORD8 * tx_descr)
{
	UWORD8 tx_rate = 1;
    //UWORD8 phy_rate = 0;
    UWORD32  retry_set[2]   = {0};
    UWORD32  phy_tx_mode    = 0;
    UWORD8   pream          = 0;


    if(tx_descr == NULL)
    {
        return NULL;
    }
	
	if(g_user_tx_rate)
		tx_rate = g_user_tx_rate;
	
	switch(tx_rate)
	{
		case 7: // 6.5M
			tx_rate = 0x80;
			break;
		case 13: // 13M
			tx_rate = 0x81;
			break;
		case 19: // 19.5M
			tx_rate = 0x82;
			break;
		case 26: // 26M
			tx_rate = 0x83;
			break;
		case 39: // 39M
			tx_rate = 0x84;
			break;
		case 52: // 52M
			tx_rate = 0x85;
			break;
		case 58: // 58.5M
			tx_rate = 0x86;
			break;
		case 65: // 65M
			tx_rate = 0x87;
			break;

		default:
			break;
	}

	update_retry_rate_set(0, tx_rate, 0, retry_set);
	pream = get_preamble(tx_rate);
	phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	/* Set the required transmit descriptor contents */
	set_tx_params(tx_descr, tx_rate, pream, BCAST_NO_ACK, phy_tx_mode, retry_set);
	return tx_descr;
}

UWORD8 * npi_create_pkt(int pkt_len, int q_num)
{
	 UWORD8 *tx_descr = NULL;    
    UWORD8  *buffer;
    UWORD32 *tx_descr_32;
    //UWORD8 q_num = 0;
	//int i = 0;
	UWORD8 tx_rate = 1;
    //UWORD8 phy_rate = 0;
    UWORD32  retry_set[2]   = {0};
    UWORD32  phy_tx_mode    = 0;
    UWORD8   pream          = 0;
    UWORD8   ack_pol        = 0;
    UWORD8 *sta_mac = NULL;
    UWORD8 mac_hdr_len = 24;
    //UWORD16 frame_len;
    UWORD8 mac_addr1[6]={0xf4,0xec,0x38,0x33,0xfd,0x3e};
    buffer_desc_t buff_desc = {0, };

    buffer = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer == NULL)
    {
        printk("npi: alloc pkt buffer failed %u. \n", ++npi_tx_count[0]);
        return NULL;
    }

    tx_descr = create_default_tx_dscr(0, 0, 0);
    if(tx_descr == NULL)
    {
        printk("npi: alloc tx descr failed %u. list count = %d\n", ++npi_tx_count[1], g_q_handle.tx_handle.tx_list_count);
        pkt_mem_free(buffer);
        return NULL;
    }
	
	if(g_user_tx_rate)
		tx_rate = g_user_tx_rate;
	
	switch(tx_rate)
	{
		case 7: // 6.5M
			tx_rate = 0x80;
			break;
		case 13: // 13M
			tx_rate = 0x81;
			break;
		case 19: // 19.5M
			tx_rate = 0x82;
			break;
		case 26: // 26M
			tx_rate = 0x83;
			break;
		case 39: // 39M
			tx_rate = 0x84;
			break;
		case 52: // 52M
			tx_rate = 0x85;
			break;
		case 58: // 58.5M
			tx_rate = 0x86;
			break;
		case 65: // 65M
			tx_rate = 0x87;
			break;

		default:
			break;
	}
	if(0x80 <= tx_rate && tx_rate <= 0x87)
	{
		memcpy(tx_pkt_sample, g_rf_test_qos_data_pkt_hdr, 
					sizeof(g_rf_test_qos_data_pkt_hdr));
	}

	//fill descr field.
	memcpy(tx_descr, tx_desc_sample, 152);
#if 1
	update_retry_rate_set(0, tx_rate, 0, retry_set);
	pream = get_preamble(tx_rate);
	phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	// no ack
	ack_pol = BCAST_NO_ACK;

	/* Set the required transmit descriptor contents */
	set_tx_params(tx_descr, tx_rate, pream, ack_pol, phy_tx_mode, retry_set);
#endif

	buff_desc.buff_hdl    = (UWORD8 *)buffer;
    buff_desc.data_length = pkt_len;
    buff_desc.data_offset = 0 + mac_hdr_len;

	set_tx_submsdu_info(tx_descr, &buff_desc, 1, mac_hdr_len);

	
	/* Get the PHY transmit mode based on the transmit rate and preamble */
	// phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	tx_descr_32 = (UWORD32 *)tx_descr;
	tx_descr_32[4] = 0; //next is null.
	//tx_descr_32[27] = (tx_descr_32[27] & 0xFFFF0000) | (pkt_len << 16);

#if 0
	set_tx_dscr_buffer_addr(tx_descr_32, (UWORD32)buffer);
	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info(tx_descr_32, (UWORD32)(tx_descr + TX_DSCR_LEN));
	q_num = get_tx_dscr_q_num(tx_descr_32);
	tx_descr_32[36] = (UWORD32)buffer;
	tx_descr_32[37] = (tx_descr_32[37] & 0xFFFF0000) | ((pkt_len - ((tx_descr_32[37]>>16) & 0xFFFF)));
#endif
	//memcpy(buffer, tx_pkt_sample, pkt_len);	
	memcpy(buffer, tx_pkt_sample, min(sizeof(tx_pkt_sample), pkt_len));
	set_tx_dscr_q_num(tx_descr_32, q_num);
       //printk("%s: q_num is %d=========================\n", __func__, q_num);
#if 0
	if (tx_power_level > 0)
	{
		printk("set_tx_power: %u\n", tx_power_level);
		//set_tx_dscr_tx_pow_level(buffer, tx_power_level);
		tx_descr[5] = tx_power_level;
		printk("after set, tx power is: %u\n", tx_descr[5]);
	}
#endif
	sta_mac = mget_StationID();
	//set_address2(buffer, g_mac_addr);
	set_address1(buffer, mac_addr1);
	set_address2(buffer, sta_mac);
	set_address3(buffer, sta_mac);
	return tx_descr;
}

int qmu_cpy_to_trout_new(UWORD32 q_num, int call_flag);

int trout_npi_send_pkt_from_host(int pkt_len,int num,int q_num)
{
	int i = 0 ;
	UWORD8 *tx_descr = NULL;    
       //UWORD8  *buffer;
	int all_send  = 0;
      UWORD32 *tx_descr_32;

	for(i=0;i<num;i++){
		tx_descr = npi_create_pkt(pkt_len, q_num);
		tx_descr_32  = (UWORD32 *)tx_descr;
		if(qmu_add_tx_packet_no_send(&g_q_handle.tx_handle, q_num, tx_descr) != QMU_OK){
			free_tx_dscr((UWORD32 *)tx_descr);
			continue;
		}
		all_send += pkt_len;
	}
       qmu_cpy_to_trout_new(q_num, TX_SEND_CALL);
	return all_send;
}

int trout_b2b_pkt_from_host(int pkt_len)
{
    UWORD8 *tx_descr;    
    UWORD8  *buffer;
    UWORD32 *tx_descr_32;
    UWORD8 q_num = 0;
	//int i = 0;
	UWORD8 tx_rate = 1;
    //UWORD8 phy_rate = 0;
    UWORD32  retry_set[2]   = {0};
    UWORD32  phy_tx_mode    = 0;
    UWORD8   pream          = 0;
    UWORD8   ack_pol        = 0;
    UWORD8 *sta_mac = NULL;
	UWORD8 mac_hdr_len = 24;
	//UWORD16 frame_len;
    buffer_desc_t buff_desc = {0, };

    buffer = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer == NULL)
    {
        printk("npi: alloc pkt buffer failed %u. \n", ++npi_tx_count[0]);
        return -3;
    }

    tx_descr = create_default_tx_dscr(0, 0, 0);
    if(tx_descr == NULL)
    {
        printk("npi: alloc tx descr failed %u. list count = %d\n", ++npi_tx_count[1], g_q_handle.tx_handle.tx_list_count);
        pkt_mem_free(buffer);
        return -2;
    }
	
	if(g_user_tx_rate)
		tx_rate = g_user_tx_rate;
	
	switch(tx_rate)
	{
		case 7: // 6.5M
			tx_rate = 0x80;
			break;
		case 13: // 13M
			tx_rate = 0x81;
			break;
		case 19: // 19.5M
			tx_rate = 0x82;
			break;
		case 26: // 26M
			tx_rate = 0x83;
			break;
		case 39: // 39M
			tx_rate = 0x84;
			break;
		case 52: // 52M
			tx_rate = 0x85;
			break;
		case 58: // 58.5M
			tx_rate = 0x86;
			break;
		case 65: // 65M
			tx_rate = 0x87;
			break;

		default:
			break;
	}
	if(0x80 <= tx_rate && tx_rate <= 0x87)
	{
		memcpy(tx_pkt_sample, g_rf_test_qos_data_pkt_hdr, 
					sizeof(g_rf_test_qos_data_pkt_hdr));
	}

	//fill descr field.
	memcpy(tx_descr, tx_desc_sample, 152);
#if 1
	update_retry_rate_set(0, tx_rate, 0, retry_set);
	pream = get_preamble(tx_rate);
	phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	// no ack
	ack_pol = BCAST_NO_ACK;

	/* Set the required transmit descriptor contents */
	set_tx_params(tx_descr, tx_rate, pream, ack_pol, phy_tx_mode, retry_set);
#endif

	buff_desc.buff_hdl    = (UWORD8 *)buffer;
    buff_desc.data_length = pkt_len;
    buff_desc.data_offset = 0 + mac_hdr_len;

	set_tx_submsdu_info(tx_descr, &buff_desc, 1, mac_hdr_len);

	
	/* Get the PHY transmit mode based on the transmit rate and preamble */
	// phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
	tx_descr_32 = (UWORD32 *)tx_descr;
	tx_descr_32[4] = 0; //next is null.
	//tx_descr_32[27] = (tx_descr_32[27] & 0xFFFF0000) | (pkt_len << 16);

#if 0
	set_tx_dscr_buffer_addr(tx_descr_32, (UWORD32)buffer);
	/* Set the Pointer to sub-MSDU information table in the TX descriptor */
	set_tx_dscr_submsdu_info(tx_descr_32, (UWORD32)(tx_descr + TX_DSCR_LEN));
	q_num = get_tx_dscr_q_num(tx_descr_32);
	tx_descr_32[36] = (UWORD32)buffer;
	tx_descr_32[37] = (tx_descr_32[37] & 0xFFFF0000) | ((pkt_len - ((tx_descr_32[37]>>16) & 0xFFFF)));
#endif
	memcpy(buffer, tx_pkt_sample, pkt_len);	

	q_num = get_tx_dscr_q_num(tx_descr_32);
	printk("%s: q_num=%d\n", __func__, q_num);

#if 0
	if (tx_power_level > 0)
	{
		printk("set_tx_power: %u\n", tx_power_level);
		//set_tx_dscr_tx_pow_level(buffer, tx_power_level);
		tx_descr[5] = tx_power_level;
		printk("after set, tx power is: %u\n", tx_descr[5]);
	}
#endif
	sta_mac = mget_StationID();
	//set_address2(buffer, g_mac_addr);
	//set_address3(buffer, g_mac_addr);
	set_address2(buffer, sta_mac);
	set_address3(buffer, sta_mac);

	//add to queue and send out.
	if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_descr) != QMU_OK)
	{
		/* Exception. Do nothing. */
		//printk("[Tx] add to Q fail.\n");
		free_tx_dscr((UWORD32 *)tx_descr);
		return -1;
	}

	return pkt_len;
}



int trout_rf_test_send_pkt(UWORD8 *pkt, UWORD16 len, UWORD8 tx_rate)
{
    UWORD8   pream          = 0;
    UWORD8   ack_pol        = 0;
    UWORD8   *tx_dscr       = 0;
    CIPHER_T ct             = NO_ENCRYP;
    UWORD16  frame_body_len = len - MAC_HDR_LEN - FCS_LEN;
    UWORD32  phy_tx_mode    = 0;
    UWORD32  retry_set[2]   = {0};

    UWORD8 *buffer_addr = 0;
    UWORD8 q_num  = NORMAL_PRI_Q;
    UWORD8 idx = 0;
    
    //TROUT_PRINT("try sending pkt..");
    buffer_addr = pkt_mem_alloc(MEM_PRI_TX);
    if(buffer_addr == NULL)
    {
        printk("%s pkt_mem_alloc error!\n",__FUNCTION__);
        return -1;
    }
    memcpy(buffer_addr, (char *)pkt, len);
    
    set_address2(buffer_addr, mget_StationID());

    {
        /* Get the transmit rate for the associated station based on the     */
        /* auto-rate, multi-rate or user-rate settings. The preamble must be */
        /* set accordingly.                                                  */
    	//tx_rate = 0x87;//dumy add for test TX_RATE 0704
    	//tx_rate = 0x81;
        //tx_rate = get_tx_rate_to_sta(se);
        //tx_rate     = get_max_basic_rate(); //for test
        //TROUT_PRINT("old rate=%d, new rate=%d",tx_rate, get_curr_tx_rate());
        //tx_rate = get_curr_tx_rate();
        pream = get_preamble(tx_rate);
        
     	//tx_rate = 0x87;//dumy add for test TX_RATE 0704
        /* Update the retry set information for this frame */
        //update_retry_rate_set(1, tx_rate, se, retry_set);
        update_retry_rate_set(0, tx_rate, 0, retry_set);
    }


    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, 0);
    //TROUT_PRINT("phy_tx_mode 0x%x, 0x%x",phy_tx_mode,pream);
    /* Create the transmit descriptor and set the contents */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        pkt_mem_free(buffer_addr);
        printk("%s create_default_tx_dscr error!\n",__FUNCTION__);
        return -1;
    }

    // no ack
    ack_pol = BCAST_NO_ACK;

    /* Set the required transmit descriptor contents */
    set_tx_params(tx_dscr, tx_rate, pream, ack_pol, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, buffer_addr, 0, MAC_HDR_LEN, frame_body_len);

    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

    /* For shared key authentication the security fields need to be set  */
    set_tx_security(tx_dscr, ct, 0, idx);

    /* Set the receiver address LUT index */
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);

    /* Update the TSF timestamp in the Tx-desc */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    //hex_dump("tx_dscr",tx_dscr,35*4+4*4);

    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        free_tx_dscr((UWORD32 *)tx_dscr);
        printk("%s qmu_add_tx_packet error!\n",__FUNCTION__);
        return -1;
    }

    return 0;
}
int trout_rf_test_send_pkt_hugh(UWORD8 *pkt, UWORD16 len, UWORD8 tx_rate){return 0;}

#endif


