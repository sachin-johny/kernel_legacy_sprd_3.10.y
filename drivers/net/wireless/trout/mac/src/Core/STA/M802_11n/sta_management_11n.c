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
/*  File Name         : sta_management_11n.c                                 */
/*                                                                           */
/*  Description       : This file contains the functions related to the MAC  */
/*                      management routines. The request updating and        */
/*                      mib updating will be done in these functions.        */
/*                                                                           */
/*  List of Functions : update_11n_params_sta                                */
/*                      update_scan_response_11n                             */
/*                      update_join_req_params_2040                          */
/*                      update_start_req_params_2040                         */
/*                      set_sm_power_save_sta                                */
/*                      reset_11n_entry_sta                                  */
/*                      sta_enabled_action_req_11n                           */
/*                      sta_enabled_rx_11n_control                           */
/*                      sta_enabled_rx_11n_action                            */
/*                      detect_2040_te_a_b                                   */
/*                      send_obss_scan_report_2040                           */
/*                      process_sec_chan_offset_2040                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/


#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "cglobals_sta.h"
#include "management_sta.h"
#include "frame_11n.h"
#include "sta_management_11n.h"
#include "sta_frame_11n.h"
#include "prot_if.h"
#include "receive.h"
#include "core_mode_if.h"
#include "qmu_if.h"
#include "ch_info.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_smps_mode = 0;
BOOL_T g_ap_ht_capable = BFALSE;


/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_11n_params_sta                                    */
/*                                                                           */
/*  Description   : This function parses the HT specific information         */
/*                  elements and updates the sta entry with it. Under        */
/*                  Infrastructure mode of operation, relevant MIB           */
/*                  parameters are also updated.                             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the management message                     */
/*                  2) Offset to the first information element.              */
/*                  3) Length of the message.                                */
/*                  4) Pointer to the STA entry                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The HT related information elements are extracted from   */
/*                  the management message and the relevant parameters in    */
/*                  the sta-entry are updated with them.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_11n_params_sta(UWORD8 *msa, UWORD16 offset, UWORD16 rx_len,
                           sta_entry_t *entry)
{
    UWORD16     index      = offset;
    UWORD16     info_elem  = 0;
    UWORD8      mcs_bmp_index;
    UWORD16     cap_info   = 0;
    ht_struct_t *ht_hdl    = &(entry->ht_hdl);
    ba_struct_t *ba_hdl    = &(entry->ba_hdl);
    UWORD8 is_ht_present   = 0;

	TROUT_FUNC_ENTER;
	
    /* Update the capability information */
    cap_info  = get_cap_info(msa);

    /* Update the Block ACK capability */
    update_ba_cap(ba_hdl, cap_info);

    while(index < (rx_len - FCS_LEN))
    {
        /* Search for HT Capabilities Element */
        if(msa[index] == IHTCAP)
        {
            UWORD16 ofst = index;

            is_ht_present = 1;
            /* Presence of HT Capability Element implies the station is HT */
            /* capable                                                     */
            ht_hdl->ht_capable = 1;

            ofst += IE_HDR_LEN;

            /* Parse HT Capabilities Info Field */
            info_elem = MAKE_WORD16(msa[ofst], msa[ofst + 1]);

            /* LDPC Coding Capability */
            ht_hdl->ldpc_cod_cap = (info_elem & BIT0);

            /* Channel Width Element is initialized by the channel width   */
            /* supported field. This will subsequently be overidden by the */
            /* STA-Channel Width field of the HT-Operation Element         */
            ht_hdl->chan_width = ((info_elem & BIT1) >> 1);

            /* Extract the SMPS mode B2 & B3 */
            {
                UWORD8 smps = (info_elem & 0x000C);

                if(smps == STATIC_SMPS)
                    ht_hdl->smps_mode = STATIC_MODE;
                else if(smps == DYNAMIC_SMPS)
                    ht_hdl->smps_mode = DYNAMIC_MODE;
                else if(smps == SM_ENABLED)
                    ht_hdl->smps_mode = MIMO_MODE;
            }

            /* Extract support for reception of Greenfield packets */
            ht_hdl->greenfield = ((info_elem & BIT4) >> 4);

            /* Extract Short-GI support for reception of 20MHz packets */
            ht_hdl->short_gi_20 = ((info_elem & BIT5) >> 5);

            /* Extract Short-GI support for reception of 40MHz packets */
            ht_hdl->short_gi_40 = ((info_elem & BIT6) >> 6);

            /* Extract STBC reception capability */
            ht_hdl->rx_stbc = ((info_elem & 0x30) >> 8);

            /* Extract AMSDU max-size field */
            if(0 == (info_elem & BIT11))
                ht_hdl->sta_amsdu_maxsize = 3839;
            else
                ht_hdl->sta_amsdu_maxsize = 7935;

            /* DSSS/CCK Mode in 40 MHz */
            ht_hdl->dsss_cck_40mhz = ((info_elem & BIT12) >> 12);

            /* Extract support for L-SIG TXOP Protection */
            ht_hdl->lsig_txop_prot = ((info_elem & BIT15) >> 15);

            ofst += HT_CAPINFO_FIELD_LEN;

            /* Parse A-MPDU Parameters Field */

            /* Extract the Maximum Rx A-MPDU factor (B1 - B0) */
            ht_hdl->max_rx_ampdu_factor = (msa[ofst] & 0x03);

            /* Extract the Minimum MPDU Start Spacing (B2 - B4) */
            ht_hdl->min_mpdu_start_spacing = (msa[ofst] >> 2) & 0x07;

            ofst += AMPDU_PARAM_LEN;

            /* Parse Supported MCS Set Field */

            /* AND the Rx MCS bitmask with supported Tx MCS bitmask          */
            /* and reset the last 3 bits in the last                         */
            /* byte since the bitmask is 77 bits                             */

            for(mcs_bmp_index = 0; mcs_bmp_index < MCS_BITMASK_LEN;
                mcs_bmp_index++)
            {
                ht_hdl->rx_mcs_bitmask[mcs_bmp_index] =
                    mget_SupportedMCSTxValue().mcs_bmp[mcs_bmp_index] &
                    (*(UWORD8 *)(msa + ofst + mcs_bmp_index));
            }

            ht_hdl->rx_mcs_bitmask[MCS_BITMASK_LEN - 1] &= 0x1F;

            ofst += MCS_SET_FIELD_LEN;

            /* Parse HT Extended Capabilities Info Field */
            info_elem = MAKE_WORD16(msa[ofst], msa[ofst + 1]);

            /* Extract HTC support Information */
            if((info_elem & BIT10) != 0)
                ht_hdl->htc_support = 1;

            ofst += HT_EXTCAP_FIELD_LEN;

            /* Parse Tx Beamforming Field */
            ofst += TX_BF_FIELD_LEN;

            /* Parse ASEL Capabilities Field */
            ofst += ASEL_FIELD_LEN;
        }

        /* Increment index by length information & tag header */
        index += msa[index + 1] + IE_HDR_LEN;
    }

    /* Set HT capable to 1 if HT capability element present or else 0 */
    ht_hdl->ht_capable = is_ht_present;

#ifdef DEBUG_MODE
    if(mget_DesiredBSSType() != INDEPENDENT)
    {
        if(is_ap_ht_cap() == BTRUE)
        {
            /* Print the STA's HT-Capabilities */
            TROUT_DBG4("Peer's HT-Capabilities:\n\r");
            TROUT_DBG4("ht_capable             = %x\n\r", ht_hdl->ht_capable);
            TROUT_DBG4("ldpc_cod_cap           = %x\n\r", ht_hdl->ldpc_cod_cap);
            TROUT_DBG4("smps_mode              = %x\n\r", ht_hdl->smps_mode);
            TROUT_DBG4("greenfield             = %x\n\r", ht_hdl->greenfield);
            TROUT_DBG4("short_gi_20            = %x\n\r", ht_hdl->short_gi_20);
            TROUT_DBG4("short_gi_40            = %x\n\r", ht_hdl->short_gi_40);
            TROUT_DBG4("rx_stbc                = %x\n\r", ht_hdl->rx_stbc);
            TROUT_DBG4("max_rx_ampdu_factor    = %x\n\r", ht_hdl->max_rx_ampdu_factor);
            TROUT_DBG4("min_mpdu_start_spacing = %x\n\r", ht_hdl->min_mpdu_start_spacing);
            TROUT_DBG4("htc_support            = %x\n\r", ht_hdl->htc_support);
            TROUT_DBG4("sta_amsdu_maxsize      = %x\n\r", ht_hdl->sta_amsdu_maxsize);
            TROUT_DBG4("chan_width             = %x\n\r", ht_hdl->chan_width);
            TROUT_DBG4("dsss_cck_40mhz         = %x\n\r", ht_hdl->dsss_cck_40mhz);
        }
    }
    TROUT_FUNC_EXIT;
#endif /* DEBUG_MODE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_scan_response_11n                                 */
/*                                                                           */
/*  Description   : This function updates the scan parameters according to   */
/*                  received beacon.                                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                                                                           */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_scan_response_11n(UWORD8 *msa, UWORD16 rx_len, UWORD16 offset,
                              UWORD8 dscr_set_index)
{
    UWORD16 index   = 0;

    g_bss_dscr_set[dscr_set_index].ht_capable     = 0;
    g_bss_dscr_set[dscr_set_index].supp_chwidth   = 0;
    g_bss_dscr_set[dscr_set_index].sta_chwidth    = 0;
    g_bss_dscr_set[dscr_set_index].sec_ch_offset  = SCN;
    g_bss_dscr_set[dscr_set_index].coex_mgmt_supp = 0;

    index = offset;
    while(index < (rx_len - FCS_LEN))
    {
        if(IHTCAP == msa[index])
        {
            /* Presence of HT Capability Element implies the station is HT */
            /* capable                                                     */
            g_bss_dscr_set[dscr_set_index].ht_capable = 1;

            /* Extract supported channel width */
            g_bss_dscr_set[dscr_set_index].supp_chwidth =
                                                ((msa[index + 2] & BIT1) >> 1);

        }
        else if(IHTOPERATION == msa[index])
        {
            /* Extract STA Channel Width and Secondary Channel Offset fields */
            /* from the HT-Operation Information Element.                    */
            g_bss_dscr_set[dscr_set_index].sec_ch_offset =
                                                (msa[index + 3] & 0x03);
            g_bss_dscr_set[dscr_set_index].sta_chwidth =
                                                (msa[index + 3] & BIT2) >> 2;
        }
        else if(IEXTCAP == msa[index])
        {
            /* Extract 20/40 BSS Coexistence Management Support */
            g_bss_dscr_set[dscr_set_index].coex_mgmt_supp =
                                                       (msa[index + 2] & BIT0);
        }

        index += (msa[index + 1] + IE_HDR_LEN);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_sm_power_save_sta                                    */
/*                                                                           */
/*  Description   : This function sets the SMPS mode for STA mode.           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_asoc_id, g_smps_mode                                   */
/*                                                                           */
/*  Processing    : SMPS mode is set for infrastructure STA. If the STA is   */
/*                  not associated, the SMPS mode is saved in MIB and once   */
/*                  association is successful, the required SMPS mode is set */
/*                  in H/w. If the STA is already associated a SMPS action   */
/*                  frame is prepared and sent to the AP to indicate the     */
/*                  change in SMPS mode. On successful transmission of the   */
/*                  same (checked in MISC_TX_COMP event processing) the      */
/*                  required SMPS mode is set in H/w.                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_sm_power_save_sta(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > NUM_MIMO_POWER_SAVE_MODE)
        return;

    if(mget_MIMOPowerSave() == val)
    {
        /* No change in value set by user and current setting */
        /* Only the temporary smps setting variable is updated since this */
        /* might be out of sync with the MIB setting during init.         */
        g_smps_mode = val;
        return;
    }

    /* The SMPS mode is currently set only for Infrastructure mode STA */
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        /* Save the user setting. MIB will be updated only after the mode is */
        /* sucessfully changed.                                              */
        g_smps_mode = val;

        if(g_asoc_id != 0)
        {
            UWORD16 smps_frame_len = 0;
            UWORD8  *smps_frame    = 0;

            /* If the STA is already associated, create and send an action   */
            /* request to change SMPS mode.                                  */

            /* Allocate memory for the SMPS Action frame */
            smps_frame = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,
                                             MANAGEMENT_FRAME_LEN);
            if(smps_frame == NULL)
            {
                return;
            }

            /* Prepare the SMPS Action frame */
            smps_frame_len = prepare_smps_frame(smps_frame);

            /* Transmit the management frame */
            tx_mgmt_frame(smps_frame, smps_frame_len, HIGH_PRI_Q, 0);

            /* Note that the required SMPS mode will be enabled in H/w only  */
            /* after this action frame is transmitted successfully. On the   */
            /* MISC_TX_COMP event processing the same is done after checking */
            /* for SMPS frame transmission.                                  */
        }
        else
        {
            /* If the STA is not yet associated, do nothing. The setting is  */
            /* saved in MIB. The required SMPS mode will be enabled in H/w   */
            /* once association is successful.                               */
            /* Update the MIB here. The h/w is updated when association completes */
            mset_MIMOPowerSave(g_smps_mode);
        }
    }
    else
    {
        /* Independent mode. Do nothing. */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_11n_entry_sta                                      */
/*                                                                           */
/*  Description   : This function resets the 11n related information in the  */
/*                  station entry.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function resets the state of all AMPDU and AMSDU    */
/*                  sessions with the given station.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_11n_entry_sta(sta_entry_t *se)
{
    UWORD8 tid         = 0;

    /* Reset all active AMSDU and AMPDU sessions setup for the STA */
    for(tid = 0; tid < 16; tid++)
    {
        reset_tx_amsdu_session(se, tid);
        reset_tx_ampdu_session(se, tid);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_action_req_11n                               */
/*                                                                           */
/*  Description   : This function calls a MAC core function to handle the    */
/*                  incoming action request.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core function is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. The state is not changed.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_enabled_action_req_11n(UWORD8 *msg)
{
    UWORD8 cat  = 0;
    UWORD8 type = 0;
    UWORD8 *req = 0;

    /*************************************************************************/
    /* Format of action request message                                      */
    /* --------------------------------------------------------------------- */
    /* | Message Length | Action Message                                   | */
    /* |                |- - - - - - - - - - - - - - - - - - - - - - - - - | */
    /* |                | Category | Action | Dialog Token | Frame body    | */
    /* --------------------------------------------------------------------- */
    /* | 1 (L bytes)    | 1        | 1      | 1            | (L - 3)       | */
    /* --------------------------------------------------------------------- */
    /*************************************************************************/

    cat  = msg[1];
    type = msg[2];
    req  = &msg[1];

    switch(cat)
    {
	    case BA_CATEGORY:
	    {
	        switch(type)
	        {
	            case MLME_ADDBA_REQ_TYPE:
	            {
	                handle_mlme_addba_req(req);
	            }
	            break;

	            case MLME_DELBA_REQ_TYPE:
	            {
	                handle_mlme_delba_req(req);
	            }
	            break;
	            default:
	            {
	                /* Do nothing */
	            }
	            break;
	        }
#ifdef NO_ACTION_RESET
	        save_action_req(req, type, BA_CATEGORY);
#endif /* NO_ACTION_RESET */
	    }
    	break;

	    default:
	    {
	        /* Process as a Private Action Request */
	        set_p_action_request(msg);
	    }
	    break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_11n_control                               */
/*                                                                           */
/*  Description   : This function handles the incoming control frame as      */
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

void sta_enabled_rx_11n_control(UWORD8 *msg)
{
    wlan_rx_t   *wlan_rx = (wlan_rx_t*)msg;
    sta_entry_t *se      = (sta_entry_t *)wlan_rx->sa_entry;
    UWORD8      *msa     = wlan_rx->msa;

	TROUT_FUNC_ENTER;
	
    if(se == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    switch(wlan_rx->sub_type)
    {
    case BLOCKACK_REQ:
    {
        UWORD8 tid          = (msa[17] & 0xF0) >> 4;
        ba_rx_struct_t **ba = &(se->ba_hdl.ba_rx_ptr[tid]);
        UWORD16 seq_num     = get_bar_start_seq_num(msa);

#ifdef DEBUG_MODE
        g_mac_stats.babarrxd++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
        g_delay_stats.numbarrxd++;
#endif /* MEASURE_PROCESSING_DELAY */

        if((*ba)== NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.babarrcvdigned++;
#endif /* DEBUG_MODE */
			TROUT_FUNC_EXIT;
            return;
        }

        /* Process the Block ACK request if Block ACK session is on and the  */
        /* Block ACK policy is not Immediate.                                */
        if((*ba)->is_ba == BTRUE)
        {
            /* Reorder the BA-Rx Buffer */
            reorder_ba_rx_buffer_bar((*ba), tid, seq_num);
        }
    }
    break;

    case BLOCKACK:
    {
#ifdef DEBUG_MODE
        if(is_expba_filter_on() == BTRUE)
            g_mac_stats.bafilunexp++;
#endif /* DEBUG_MODE */
    }
    break;
    default:
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_11n_action                                */
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

void sta_enabled_rx_11n_action(UWORD8 *msg)
{
    UWORD8    *data = NULL;
    wlan_rx_t *wlan_rx = (wlan_rx_t*)msg;
    sta_entry_t *se    = (sta_entry_t *)wlan_rx->sa_entry;

	TROUT_FUNC_ENTER;
    if(se == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Pointer to the start of the data portion in the frame. */
    data = wlan_rx->msa + wlan_rx->hdr_len;

    /* Process the action frame based on the category */
    switch(data[CATEGORY_OFFSET])
    {
    case BA_CATEGORY:
    {
        /* Process the QOS action frame based on the action type */
        switch(data[ACTION_OFFSET])
        {
        case ADDBA_REQ_TYPE:
        {
			TROUT_DBG4("handle Rx ADDBA Request!\n");
            handle_wlan_addba_req(wlan_rx->sa, data);
        }
        break;

        case ADDBA_RSP_TYPE:
        {
        	TROUT_DBG4("handle Tx ADDBA response!\n");
            handle_wlan_addba_rsp(wlan_rx->sa, data);
        }
        break;

        case DELBA_TYPE:
        {
            handle_wlan_delba(wlan_rx->sa, data);
        }
        break;

        default:
        {
            /* Do nothing. Received action type is either not recognized */
            /* or does not need to be handled in STA mode.               */
        }
        break;
        }
    }
    break;

    case HT_CATEGORY:
    {
        /* Process the QOS action frame based on the action type */
        switch(data[ACTION_OFFSET])
        {

        default:
        {
            /* Do nothing. Received action type is either not recognized */
            /* or does not need to be handled in STA mode.               */
        }
        break;
        }
    }
    break;

    default:
    {
        /* Do nothing. The received action category is not recognized. */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

#endif /* MAC_802_11N */
#endif /* IBSS_BSS_STATION_MODE */

