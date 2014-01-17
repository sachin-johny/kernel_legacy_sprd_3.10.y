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
/*  File Name         : management_ap.c                                      */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      functions, that are specific to AP mode of           */
/*                      operation.                                           */
/*                                                                           */
/*  List of Functions : init_asoc_table                                      */
/*                      update_start_req_params                              */
/*                      update_asoc_entry                                    */
/*                      update_asoc_entry_txci                               */
/*                      reset_asoc_entry                                     */
/*                      filter_host_rx_frame_ap                              */
/*                      filter_wlan_rx_frame_ap                              */
/*                      ap_aging_timeout_fn                                  */
/*                      ap_aging_fn                                          */
/*                      delete_asoc_entry                                    */
/*                      unjoin_all_stations                                  */
/*                      unjoin_station                                       */
/*                      get_sta_join_info_ap                                 */
/*                      get_all_sta_info_ap                                  */
/*                      process_obss_erp_info                                */
/*                      send_probe_request_ap                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_ap.h"
#include "index_util.h"
#include "management_ap.h"
#include "ap_prot_if.h"
#include "pm_ap.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variable                                                           */
/*****************************************************************************/

UWORD8         g_num_sta_no_short_slot  = 0;
UWORD8         g_num_sta_non_erp        = 0;
UWORD8         g_num_sta_no_short_pream = 0;
UWORD8         g_erp_aging_cnt          = ERP_AGING_THRESHOLD;
ALARM_HANDLE_T *g_aging_timer    = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_start_req_params                                  */
/*                                                                           */
/*  Description   : This function updates globals and MIB values according   */
/*                  to input start request structure.                        */
/*                                                                           */
/*  Inputs        : 1) Start request structure                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The global start structure and MIB values are updated    */
/*                  according to input start request structure.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_start_req_params(start_req_t *start_req)
{
    /* Set the channel with the configured channel. All other configurations */
    /* are not modified and need not be reinitialized.                       */
    select_channel_rf(mget_CurrentChannel(), get_bss_sec_chan_offset());
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_asoc_entry                                     */
/*                                                                           */
/*  Description      : This function processes the (re)association request   */
/*                     of a STA.                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to the association entry of the STA        */
/*                     2) Pointer to the (re)association request             */
/*                     3) Length of the (re)association frame                */
/*                     4) Pointer to is p2p asoc request or not.             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The supported capabilities of the STA requesting      */
/*                     (re)association request is checked and the status is  */
/*                     accordingly set. If the status is not 'Success', the  */
/*                     function returns. Otherwise the association entry is  */
/*                     updated with the relevant information required by the */
/*                     AP.                                                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Direct  : UWORD16, Status for (Re)Association Response*/
/*                     Indirect: is p2p asoc request or not.                 */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 update_asoc_entry(asoc_entry_t *ae, UWORD8 *msa, UWORD16 rx_len,
                          BOOL_T *is_p2p)
{
    UWORD8  i                   = 0;
    UWORD8  j                   = 0;
    UWORD8  num_of_erp_rates    = 0;
    UWORD8  num_basic_rates     = 0;
    UWORD8  num_rates           = 0;
    UWORD8  missed_IE           = 2;
    UWORD8  asoc_sta_num_rates  = 0;
    UWORD16 rate_index          = 0;
    UWORD16 ext_rate_index      = 0;
    UWORD16 cap_info            = 0;
    UWORD16 info_elem_offset    = 0;
    UWORD16 index               = MAC_HDR_LEN;

    /*************************************************************************/
    /*             Contents of the (Re)Association frame body                */
    /*************************************************************************/

    /*************************************************************************/
    /*              Association Request Frame - Frame Body                   */
    /* --------------------------------------------------------------------- */
    /* | Capability Information | Listen Interval | SSID | Supported Rates | */
    /* --------------------------------------------------------------------- */
    /* |2                       |2                |2-34  |3-10             | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*              Reassociation Request Frame - Frame Body                 */
    /* --------------------------------------------------------------------- */
    /* | Cap Info | Listen Interval | Current AP Address| SSID | Sup Rates | */
    /* --------------------------------------------------------------------- */
    /* |2         |2                |6                  |2-34  |3-10       | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Check if all the capabilities requested by the associating STA are    */
    /* supported by the AP.                                                  */
    /* Capability Information:                                               */
    /* |ESS|IBSS|CFPollable|CFPReq|Privacy|Preamble|PBCC|Agility|Reserved|   */
    cap_info  = msa[index];
    cap_info |= (msa[index + 1] << 8);

    if((check_bss_capability_info(cap_info) == BFALSE) ||
       (check_bss_capability_phy(cap_info) == BFALSE))
    {
        return UNSUP_CAP;
    }

    if(((cap_info & PRIVACY) == PRIVACY) && (ae->cipher_type == NO_ENCRYP))
    {
        ae->cipher_type = get_wep_type(mget_WEPDefaultKeyID());
    }
    else if(((cap_info & PRIVACY) != PRIVACY) &&
            (ae->cipher_type != NO_ENCRYP))
    {
        TROUT_DBG4("caisf return UNSPEC_FAIL\n");
        return UNSPEC_FAIL;
    }

    index += CAP_INFO_LEN;

    /* Set the Listen Interval of the associating STA */
    ae->listen_interval  = msa[index];
    ae->listen_interval |= (msa[index + 1] << 8);

    /* Check for boundary condition */
    if(ae->listen_interval > MAX_LISTEN_INT)
        ae->listen_interval = MAX_LISTEN_INT;

    index += LISTEN_INT_LEN;

    if(get_sub_type(msa) == REASSOC_REQ)
    {
        /* For reassociation request check that the current AP MAC address   */
        /* matches.                                                          */
        if(mac_addr_cmp(&msa[index], mget_StationID()) == BFALSE)
            return REASOC_NO_ASOC;

        /* Skip Current AP address  */
        index += MAC_ADDRESS_LEN;
    }

    /* If STA is already associated, store the number of supported rates for */
    /* future use in determining number of ERP stations                      */
    if(ae->state == ASOC)
    {
        asoc_sta_num_rates = ae->op_rates.num_rates;
    }

    info_elem_offset = index;

    /* Find the Extended support rate index */
    ext_rate_index = info_elem_offset;

    while((ext_rate_index < (rx_len - FCS_LEN)) && (msa[ext_rate_index] != IEXSUPRATES))
    {
      ext_rate_index += (msa[ext_rate_index + 1] + IE_HDR_LEN);
    }

    while(index < (rx_len - FCS_LEN))
    {
        /* Check for SSID element */
        if(msa[index] == ISSID)
        {
            WORD8 *ssid = 0;

            missed_IE--;

            /* Compare SSID of the association request */
            ssid = mget_DesiredSSID();
            if((msa[index+1] != strlen(ssid))||
               (memcmp(&msa[index + 2], ssid, msa[index + 1])!= 0))
            {
                TROUT_DBG4("caisf return UNSPEC_FAIL, SSID SET WRONG!");
                /* SSID is set wrongly in the association request frame */
                return UNSPEC_FAIL;
            }
        }

        /* Check for Supported rate element */
        if(msa[index] == ISUPRATES)
        {
            missed_IE--;
            num_rates = msa[index + 1];
            rate_index =index + IE_HDR_LEN;

            if(num_rates == 0)
                return UNSUP_RATE;

            /* Set the number of operational rates supported by the STA */
            ae->op_rates.num_rates = num_rates;

            for(i = 0; i < num_rates; i++)
                ae->op_rates.rates[i] = msa[rate_index + i] & 0x7F;

            rate_index += num_rates;

            /* Extract extended rates if applicable */
            if(ext_rate_index < rx_len)
                num_of_erp_rates = check_erp_rates((UWORD8 *)&(ae->op_rates),
                                   msa, ext_rate_index);
            else
                num_of_erp_rates = 0;

            num_rates += num_of_erp_rates;

            /* Update the number of operational rates supported by the STA */
            ae->op_rates.num_rates += num_of_erp_rates;

            /* Update the tx rate index */
            ae->tx_rate_index = ae->op_rates.num_rates - 1;

            /* Sort operational rate set of the station in ascending order */
            for(i = 0; i < num_rates; i++)
            {
                UWORD8 min = (ae->op_rates.rates[i] & 0x7F);
                UWORD8 mid = i;
                UWORD8 tmp = 0;

                for(j = i + 1; j < num_rates; j++)
                {
                    if(min > (ae->op_rates.rates[j] & 0x7F))
                    {
                        min = (ae->op_rates.rates[j] & 0x7F);
                        mid = j;
                    }
                }

                tmp                     = ae->op_rates.rates[i];
                ae->op_rates.rates[i]   = ae->op_rates.rates[mid];
                ae->op_rates.rates[mid] = tmp;
            }

            /* Sort the operational rate set of 11g station (operating in    */
            /* compatibility mode) in such a way that all 11b rates will be  */
            /* clubbed and 11a rates will come after the 11b rates           */
            if(ae->op_rates.num_rates == 12) /* 11g_compatibility mode */
            {
                if((ae->op_rates.rates[5] & 0x7F) == 0x16) /* 11Mbps */
                {
                    UWORD8 temp;
                    temp                  = ae->op_rates.rates[5];
                    ae->op_rates.rates[5] = ae->op_rates.rates[4];
                    ae->op_rates.rates[4] = ae->op_rates.rates[3];
                    ae->op_rates.rates[3] = temp;
                }
            }

            /* If the STA does not support all the basic rates respond with  */
            /* failure (unsupported rates) status.                           */
            num_basic_rates = get_num_basic_rates();
            for(i = 0; i < num_basic_rates; i++)
            {
                BOOL_T found  = BFALSE;
                UWORD8 mac_br = get_mac_basic_rate(i);

                for(j = 0; j < num_rates; j++)
                {
                    if((ae->op_rates.rates[j] & 0x7F) == (mac_br & 0x7F))
                    {
                        found = BTRUE;
                        break;
                    }
                }

                /* Return unsupported rates if the joining station does not  */
                /* support any of the basic rates.                           */
                if(found == BFALSE)
                    return UNSUP_RATE;
            }
        }

        /* Increment index by length information and header */
        index += msa[index + 1] + IE_HDR_LEN;
    }

    /* Check if either of SSID element or Supported rates element is missed */
    if(missed_IE != 0)
    {
        TROUT_DBG4("caisf return UNSPEC_FAIL, MISSED IE");
        return UNSPEC_FAIL;
    }

    /* Check for security parameters in the received Association request. In */
    /* case of mismatch, the error code is returned                          */
    {
        UWORD16 ret_val = check_sec_capabilities_ap(ae, msa, rx_len);

        if(ret_val != SUCCESSFUL_STATUSCODE){
            TROUT_DBG4("caisf check_sec_capabilities_ap failed, ret_val = %d",ret_val);
            return ret_val;
        }
    }

    /* Check for HT capability match and process further as required */
    {
        UWORD16 ret_val = 0;

        ret_val = check_ht_capabilities_ap(msa, info_elem_offset, rx_len, ae);

        if(ret_val != SUCCESSFUL_STATUSCODE){
            TROUT_DBG4("caisf ret_val = %d",ret_val);
            return ret_val;
        }
    }

    /* If the STA was already associated with this AP an entry will exist.   */
    /* In such a case no new association Id needed. Otherwise generate a new */
    /* association Id.                                                       */
    if(ae->asoc_id == 0)
    {
        if((ae->asoc_id = get_new_sta_index(msa + ADDR2_OFFSET)) == INVALID_ID)
        {
            /* The association table is full. */
            return AP_FULL;
        }

        /* Association ID is same as the STA index, used for 11i/WEP/11e */
        BUG_ON(ae->asoc_id > MAX_STA_SUPPORTED);	//add by chengwg, 2013-03-16.
        ae->sta_index = ae->asoc_id;
    }

    /* The 2 MSB bits of Association ID is set to 1 as required by the       */
    /* standard.                                                             */
    ae->asoc_id |= 0xC000;

    /* Assign the index to the maximum rate supported by the station */
    ae->tx_rate_mbps = get_user_rate((UWORD8)
                                (ae->op_rates.rates[num_rates - 1] & 0x7F));

    /* Slot time adoption is done only for 802.11g */
    if((get_phy_type() != PHY_B_DSSS) && (get_phy_type() != PHY_A_OFDM)&&
                                         (get_phy_type() != PHY_N_5GHZ))
    {
        UWORD8 erp_info_mod = 0;

        /* If the associated STA does not support Short Slot Time option,    */
        /* the AP needs to use Long Slot Time from the next beacon interval. */
        if((cap_info & SHORTSLOT) != SHORTSLOT)
        {
            /* If STA is already associated as a long slot STA, don't update */
            /* g_num_sta_no_short_slot                                       */
            if((ae->state != ASOC) || (ae->short_slot == 1))
            {
                /* Set the short slot supported field to 0. Also increment   */
                /* the global indicating number of stations not supporting   */
                /* ShortSlot                                                 */
                ae->short_slot = 0;
                g_num_sta_no_short_slot++;

                disable_short_slot();
                set_machw_long_slot_select();

                /* Update beacon in the free beacon buffer with ShortSlot    */
                /* disabled. Set the MAC H/w pointer to the address of the   */
                /* new beacon frame.                                         */
                set_cap_info_field_ap(g_beacon_frame[g_beacon_index],
                                      MAC_HDR_LEN + TIME_STAMP_LEN +
                                      BEACON_INTERVAL_LEN);
            }
        }
        else
        {
            /* If STA is already associated as a long slot STA, but trying   */
            /* to reassociate as a short slot STA, update the global         */
            /* g_num_sta_no_short_slot                                       */
            if((ae->state == ASOC) && (ae->short_slot == 0))
            {
                g_num_sta_no_short_slot--;
                if(g_num_sta_no_short_slot == 0)
                {
                    if(g_short_slot_allowed == BTRUE)
                    {
                        enable_short_slot();
                        set_machw_short_slot_select();

                        set_cap_info_field_ap(g_beacon_frame[g_beacon_index],
                           MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN);
                    }
                }

            }
            ae->short_slot = 1;
        }

        /* Check if the station is 11b station and enable the ERP protection */
        /* If the running mode is G_ONLY_MODE, it would have returned        */
        /* UNSUP_RATE and not allow the station to associate with AP.        */
        if(num_rates <= NUM_BR_PHY_802_11G_11B_1)
        {
            /* If STA is already associated as an non ERP STA, do not update */
            /* g_num_sta_non_erp                                             */
            if((ae->state != ASOC) ||
               (asoc_sta_num_rates > NUM_BR_PHY_802_11G_11B_1))
            {
                /* The associated STA is 11b (non-ERP) station */
                g_num_sta_non_erp++;

                /* Enable protection in case of auto protection */
                if(is_autoprot_enabled() == BTRUE)
                {
                    set_protection(ERP_PROT);
                    set_machw_prot_control();
                    erp_info_mod = 1;
                }
            }
        }
        else
        {
            /* If STA is already associated as a non ERP STA, but trying to */
            /* reassociate as an ERP STA, update g_num_sta_non_erp          */
            if((ae->state == ASOC) &&
               (asoc_sta_num_rates <= NUM_BR_PHY_802_11G_11B_1))
            {
                g_num_sta_non_erp--;

                if(g_num_sta_non_erp == 0)
                {
                    if(is_autoprot_enabled() == BTRUE)
                    {
                        set_protection(ERP_PROT);
                        set_machw_prot_control();
                        erp_info_mod = 1;
                    }
                }
            }
        }

        /* Check if the station supports only long preamble */
        if((cap_info & SHORTPREAMBLE) != SHORTPREAMBLE)
        {
            /* If STA is already associated as a long preamble STA, do not   */
            /* update g_num_sta_no_short_pream                               */
            if((ae->state != ASOC) || (ae->short_preamble == 1))
            {
                /* Set the short preamble supported field to 0 and increment */
                /* the global indicating number of stations not supporting   */
                /* short preamble.                                           */
                ae->short_preamble = 0;
                g_num_sta_no_short_pream++;
                g_short_preamble_enabled = BFALSE;
                set_machw_prot_pream(1);
                erp_info_mod = 1;
            }
        }
        else
        {
            /* If STA is already associated as a long preamble STA, but      */
            /* trying to reassociate as a short preamble STA, update         */
            /* g_num_sta_no_short_pream                                      */
            if((ae->state == ASOC) && (ae->short_preamble == 0))
            {
                g_num_sta_no_short_pream--;
                if(g_num_sta_no_short_pream == 0)
                {
                    g_short_preamble_enabled = BTRUE;
                    set_machw_prot_pream(0);
                    erp_info_mod = 1;
                }
            }
            ae->short_preamble = 1;
        }

        if(erp_info_mod == 1)
            update_beacon_erp_info();
    }

    /* Update the protocol capability in association entry for the STA */
    update_asoc_entry_prot(ae, msa, rx_len, cap_info, is_p2p);

    /* Update the QoS capability table for the associated station */
    update_qos_table_prot(ae->sta_index, msa, rx_len, info_elem_offset);

    /* Set the power save state to default ACTIVE. Will be checked/updated   */
    /* as required later.                                                    */
    ae->ps_state         = ACTIVE_PS;
    ae->ps_poll_rsp_qed  = BFALSE;
    ae->num_ps_pkt_lgcy  = 0;
    ae->num_qd_pkt       = 0;
    init_list_buffer(&(ae->ps_q_lgcy),
        TX_DSCR_NEXT_ADDR_WORD_OFFSET * sizeof(UWORD32));

    /* Update the retry rate set table for this station based on the current */
    /* transmit rate of this station                                         */
    update_entry_retry_rate_set((void *)ae, get_phy_rate(get_tx_rate_ap(ae)));

#ifdef AUTORATE_FEATURE
    /* Update the minimum and maximum rate index in the global auto  */
    /* rate table for this STA                                       */
    update_min_rate_idx_ap(ae);
    update_max_rate_idx_ap(ae);

    /* Update the current transmit rate index to minimum supported   */
    /* rate index in case of auto rate                               */
    init_tx_rate_idx_ap(ae);
    reinit_tx_rate_idx_ap(ae);

    ar_stats_init(&(ae->ar_stats));
#endif /* AUTORATE_FEATURE */

    return SUCCESSFUL_STATUSCODE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_asoc_entry_txci                                */
/*                                                                           */
/*  Description      : This function updates the association entry of a      */
/*                     station on getting the TX Complete Interrupt for      */
/*                     (re)association response frame sent to the sttaion.   */
/*                                                                           */
/*  Inputs           : 1) Pointer to TX-Dscr of TxC frame                    */
/*                     2) Pointer to the association entry of the STA        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : On successful transmission of a ASOC-Request frame,   */
/*                     the state of the station is updated and the security  */
/*                     procedures initiated.                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - Association with Station valid               */
/*                     BFALSE - Association with Station terminated          */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
//BOOL_T update_asoc_entry_txci(UWORD8 *tx_dscr, asoc_entry_t *ae)
BOOL_T update_asoc_entry_txci(UWORD8 *tx_dscr, asoc_entry_t *ae, UWORD8 *msa)
{
    UWORD8 assoc_rsp_code = UNSPEC_FAIL;
    UWORD8 da[6]          = {0};
    //UWORD8 *msa           = NULL;
    BOOL_T retval         = BTRUE;
    TYPESUBTYPE_T frm_st  = DATA;

    if(ae == NULL)
        return BFALSE;

    //msa    = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);// 20120709 caisf masked, merged ittiam mac v1.2 code
    frm_st = get_sub_type(msa);

    /* Handle Association response */
    if((frm_st == ASSOC_RSP) || (frm_st == REASSOC_RSP))
    {
        /* Get the Destination address */
        get_DA(msa, da);

        /* Read the code if the frame was sent successfully to the station  */
        if((get_tx_dscr_status((UWORD32 *)tx_dscr) != TX_TIMEOUT) &&
           (get_tx_dscr_frag_status_summary((UWORD32 *)tx_dscr) == 0xFFFFFFFF))
        {
            assoc_rsp_code = get_asoc_status(msa);
        }

        if(assoc_rsp_code == SUCCESSFUL_STATUSCODE)
        {
            /* Start the Security FSM, for the successful associations */
            start_sec_fsm_ap(ae);

            /* set the state to associated */
            ae->state = ASOC;

            /* Send the sta joining information to host */
            send_join_leave_info_to_host(ae->asoc_id, da, BTRUE);

#ifdef NO_ACTION_RESET
            /* Restore all the saved action requests with this STA */
            restore_all_saved_action_req_per_sta(da, ae->asoc_id);
#endif /* NO_ACTION_RESET */

            /* ITM_DEBUG */
            PRINTD("Status: Successfully Associated STA %x:%x:%x:%x:%x:%x\n\r",
                        da[0], da[1], da[2], da[3], da[4], da[5]);

            PRINTD("STA ID = %d\n", ae->sta_index);

            //config_amsdu_func(da, BTRUE);	//add by chengwg for ap mode of amsdu.
            config_802_11n_feature(da, BTRUE);
        }
        else
        {
            delete_entry(da);
            retval = BFALSE;
        }
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_asoc_entry                                      */
/*                                                                           */
/*  Description      : This function resets the fields of asoc entry.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the association entry of the STA        */
/*                                                                           */
/*  Globals          : g_num_sta_no_short_slot                               */
/*                     g_num_sta_non_erp                                     */
/*                     g_beacon_frame                                        */
/*                     g_beacon_index                                        */
/*                     g_num_sta_ps                                          */
/*                                                                           */
/*  Processing       : This function resets the association entry for a      */
/*                     station that disassociates.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void reset_asoc_entry(asoc_entry_t *ae)
{
    /* Reset the security LUTs and entries */
    reset_prot_entry_ap(ae);

    /* Remove the entry of the STA from the STA Index Table. Note that the   */
    /* sta index is reset and cannot be re-used                              */
    del_sta_index((void *)ae);

    /* Reset the TIM bit for this station in the bitmap */
    check_and_reset_tim_bit(ae->asoc_id);

    /* Reset the association ID of the station */
    ae->asoc_id = 0;

    /* Slot time adoption is done only for 802.11g/n-2G4 */
    if((get_phy_type() != PHY_B_DSSS) && (get_phy_type() != PHY_A_OFDM) &&
       (get_phy_type() != PHY_N_5GHZ))
    {
        UWORD8 erp_info_mod = 0;

        /* Now check which sta is leaving */
        if(ae->short_slot == 0)
        {
            g_num_sta_no_short_slot--;
        }

        if(ae->op_rates.num_rates <= NUM_BR_PHY_802_11G_11B_1)
        {
            g_num_sta_non_erp--;
        }

        if(ae->short_preamble == 0)
        {
            g_num_sta_no_short_pream--;
        }

        if(g_num_sta_no_short_slot == 0)
        {
            if(g_short_slot_allowed == BTRUE)
            {
                enable_short_slot();
                set_machw_short_slot_select();

                set_cap_info_field_ap(g_beacon_frame[g_beacon_index],
                       MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN);
            }
        }

        if(g_num_sta_no_short_pream == 0)
        {
            g_short_preamble_enabled = BTRUE;
            set_machw_prot_pream(0);
            erp_info_mod = 1;
        }

        if(g_num_sta_non_erp == 0)
        {
            if(is_autoprot_enabled() == BTRUE)
            {
                set_protection(ERP_PROT);
                set_machw_prot_control();
                erp_info_mod = 1;
            }
        }

        if(erp_info_mod == 1)
            update_beacon_erp_info();
    }

    /* Update number of PS stations */
    if(ae->ps_state == POWER_SAVE)
        g_num_sta_ps--;

    /* Flush the powersave queues */
    flush_ps_queues(ae);

    /* Reset all the variables in the Association entry */
    ae->ps_poll_rsp_qed  = BFALSE;;
    ae->ps_state         = ACTIVE_PS;
    ae->num_qd_pkt       = 0;

#ifdef ENABLE_PS_PKT_FLUSH
    /* If this STA was MAX PS PKT holding STA, then clear the variable */
    handle_max_ps_ae(ae);
#endif /* ENABLE_PS_PKT_FLUSH */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : filter_host_rx_frame_ap                               */
/*                                                                           */
/*  Description      : This function filters the incoming lan packet and     */
/*                     sends the results of the operation.                   */
/*                                                                           */
/*  Inputs           : 1) mac_addres_t. The destination address              */
/*                     2) UWORD8*. Pointer of the buffer to be released      */
/*                     3) Cipher Type*. Pointer to the cpiher type for STA   */
/*                     4) BOOL_T  Flag indicating if port Status needs to be */
/*                                checked                                    */
/*                     5) UWORD*. Pointer to the sta index                   */
/*                     6) UWORD*. Pointer to the key type                    */
/*                     7) UWORD** Pointer to any info required for Tx        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function returns 'BTRUE' only if the LAN packet  */
/*                     was discarded. Also the function updates the Cipher   */
/*                     type for the STA and the key index for the STA        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T. The result of lan frame filtering.            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T filter_host_rx_frame_ap(wlan_tx_req_t* tx_req,
                               BOOL_T ignore_port, CIPHER_T* ct,
                               UWORD8* sta_index, UWORD8* key_type,
                               UWORD8** info, asoc_entry_t **ae)
{
    asoc_entry_t  *asoc_entry = 0;
    UWORD8* start_ptr = tx_req->buffer_addr;
    UWORD8* da        = tx_req->da;

    /* Reset the parameters to default values */
    *ct        = NO_ENCRYP;
    *sta_index = 0;
    *key_type  = 0;
    *info      = NULL;
    *ae        = NULL;

    /* For unicast packets, check if the destination STA is associated. */
    if(is_group(da) == BFALSE)
    {
        /* Get association entry for association state */
        asoc_entry = (asoc_entry_t*)find_entry(da);

        *ae        = asoc_entry;

        /* If the destination STA is not associated free the data packet and */
        /* return.                                                           */
        if((asoc_entry == 0) || (asoc_entry->state != ASOC))
        {
            pkt_mem_free(start_ptr);

            return BTRUE;
        }

        /* Set key type as unicast key type */
        *key_type  = UCAST_KEY_TYPE;
        *sta_index = asoc_entry->sta_index;
    }
    else
    {
        /* Set key type as unicast key type */
        *key_type  = BCAST_KEY_TYPE;
    }

    /* Filter the frame on the basis of the security policies */
    {
        BOOL_T ret_val = BFALSE;

        ret_val = filter_host_rx_sec_ap(ct, info, *key_type, asoc_entry,
                        ignore_port);

        if(ret_val == BTRUE)
        {
            pkt_mem_free(start_ptr);
            return ret_val;
        }
    }

    update_serv_class_prot_ap(*ae, tx_req->priority, &(tx_req->service_class));
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : filter_wlan_rx_frame_ap                                  */
/*                                                                           */
/*  Description   : This function filters the incoming wlan rx packet and    */
/*                  sends the results of the operation.                      */
/*                                                                           */
/*  Inputs        : 1) MAC header of incoming packet.                        */
/*                  2) Pointer to rx descriptor                              */
/*                  3) Pointer to wlan_rx structure                          */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function returns 'BTRUE' only if the LAN packet     */
/*                  was discarded.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T. The result of lan frame filtering.               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T filter_wlan_rx_frame_ap(wlan_rx_t *wlan_rx)
{
    asoc_entry_t *ae = (asoc_entry_t *)wlan_rx->sa_entry;
    UWORD8 *msa      = wlan_rx->msa;

	TROUT_FUNC_ENTER;
    if(wlan_rx->type == DATA_BASICTYPE)
    {
        if(ae == 0)
        {
            UWORD16 error = 0;

            /* Get the error code based on the frame type */
            switch(get_frame_class(msa))
            {
                case CLASS2_FRAME_TYPE:
                default:
                {
                    error = (UWORD16)CLASS2_ERR;
                }
                break;
                case CLASS3_FRAME_TYPE:
                {
                    error = (UWORD16)CLASS3_ERR;
                }
            }
            /* Send the De-authentication Frame to the station */
            send_deauth_frame(wlan_rx->ta, error);

            /* Return TRUE for filter frame, this should free the rx frame */
			TROUT_FUNC_EXIT;
            return BTRUE;
        }
        else if (ae->state != ASOC)
        {
            UWORD16 error = 0;

            /* Get the error code based on the frame type */
            switch(get_frame_class(msa))
            {
                case CLASS2_FRAME_TYPE:
                default:
                {
                    error = (UWORD16)CLASS2_ERR;
                }
                break;
                case CLASS3_FRAME_TYPE:
                {
                    error = (UWORD16)CLASS3_ERR;
                }
            }

            /* Send the Disassociation Frame to the station */
            send_disasoc_frame(wlan_rx->ta, error);

            /* Return TRUE for filter frame, this should free the rx frame */
			TROUT_FUNC_EXIT;
            return BTRUE;
        }

        /* Filter the frame on the basis of the security policies */
        {
            BOOL_T ret_val = BFALSE;

            ret_val = filter_wlan_rx_sec_ap(wlan_rx);
            if(ret_val == BTRUE)
            {
				TROUT_FUNC_EXIT;
                return ret_val;
            }
        }

        /* This STA is active, set the count to 0 */
        ae->aging_cnt = 0;

        /* Filter the frame on the basis of the service class/priorities */
        {
            BOOL_T ret_val = BFALSE;

            ret_val = filter_wlan_serv_cls_ap(wlan_rx);
            if(ret_val == BTRUE)
            {
				TROUT_FUNC_EXIT;
                return ret_val;
            }
        }
    }
    else
    {
        if(ae != 0)
        {
            /* This STA is active, set the count to 0 */
            ae->aging_cnt = 0;
        }
    }
    TROUT_FUNC_EXIT;
    return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_aging_timeout_fn                                      */
/*                                                                           */
/*  Description   : This function is called on timeout of the Aging Timer.   */
/*                                                                           */
/*  Inputs        : Alarm Handle                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function posts an event in Misc queue to indicate   */
/*                  the Aging timer expiry event.                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void ap_aging_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void ap_aging_timeout_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc       = 0;

    /* Create a MISCELLANEOUS event with the pointer to the AMSDU ctxt and   */
    /* post it to the event queue.                                           */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
    if(misc == NULL)
    {
        /* Exception: This opportunity for performing age check is lost. */
        /* The Aging timer is restarted with the hope that next time the */
        /* Misc Event Memory will be available.                          */
        start_alarm(g_aging_timer, 1000 * AGING_TIMER_PERIOD_IN_SEC);
        return;
    }

    misc->data = NULL;
    misc->name = MISC_RUN_AGING_FN;

    /* Post the event */
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_aging_fn                                              */
/*                                                                           */
/*  Description   : This function checks the activity from each associated   */
/*                  stations and if its inactive for more time, sends deauth */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : It checks each association entry and checks its timer.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_aging_fn(void)
{
    UWORD8        i        = 0;
    asoc_entry_t  *ae      = 0;
    table_elmnt_t *tbl_elm = 0;
    table_elmnt_t *next_tbl_elm = 0;// 20120709 caisf add, merged ittiam mac v1.2 code

    /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            ae = tbl_elm->element;
            next_tbl_elm = tbl_elm->next_hash_elmnt;// 20120709 caisf add, merged ittiam mac v1.2 code

        /* the station in ps_powersave should not be deleted for inactive reason. by zhao 6-21 2013 */
            if(ae && ae->ps_state == ACTIVE_PS 
		&& (ae->aging_cnt * AGING_TIMER_PERIOD_IN_SEC) >= g_aging_thresh_in_sec)
            {
                /* This station is not active from long time, send           */
                /* deauthentication to it and delete the entry               */

                 /* Handle any Protocol related STA disconnection */
                handle_disconnect_req_prot_ap(ae, tbl_elm->key);

                /* Send the De-authentication Frame to the station */
                send_deauth_frame(tbl_elm->key, (UWORD16)INACTIVITY);

                /* Send the sta leaving information to host */
                send_join_leave_info_to_host(ae->asoc_id, tbl_elm->key, BFALSE);

				TROUT_DBG4("Inactivity entry detected, delete it(%d)\n", ae->aging_cnt);
                delete_entry(tbl_elm->key);
            }

            /* Increment the aging count for this STA */
            ae->aging_cnt++;

			// 20120709 caisf add, merged ittiam mac v1.2 code
            //tbl_elm = tbl_elm->next_hash_elmnt;
			tbl_elm = next_tbl_elm;
        }
    }

    if(g_erp_aging_cnt < ERP_AGING_THRESHOLD)
    {
        /* Increment the erp aging count                                    */
        g_erp_aging_cnt++;

        /* AP is not receving any ERP becons for a long time. so,change the */
        /* protection mechanism from Self-CTS protection to No protection,if*/
        /* the Auto protection is enabled                                   */
        if(g_erp_aging_cnt == ERP_AGING_THRESHOLD)
        {
            if(is_autoprot_enabled() == BTRUE)
            {
                if(get_protection() == ERP_PROT)
                {
                    set_protection(NO_PROT);
                    set_machw_prot_control();
                    update_beacon_erp_info();
                }
            }
        }
    }

    /* Start the checking timer again */
    start_alarm(g_aging_timer, 1000 * AGING_TIMER_PERIOD_IN_SEC);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_asoc_entry                                        */
/*                                                                           */
/*  Description   : This function clears the association table and deletes   */
/*                  the entry                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the asoc entry                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function clears the association table and deletes   */
/*                  the entry                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_asoc_entry(void *element)
{
    asoc_entry_t *ae = (asoc_entry_t *)element;

    /* Delete the STA index entry */
    del_sta_index(ae);

    /* Delete the security table/LUT and halt all the security related FSMs */
    delete_sec_entry_ap(ae);

    if(ae->state == ASOC)
    {
        /* Reset all the fields of asoc entry if the station was associated. */
        reset_asoc_entry(ae);
    }

    /* Free the association table entry */
    mem_free(g_local_mem_handle, ae);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : unjoin_all_stations                                      */
/*                                                                           */
/*  Description   : This function unjoins all the associated stations        */
/*                                                                           */
/*  Inputs        : Deauth reason code                                       */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : Deletes all sta entries and sends them de-auths          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void unjoin_all_stations(UWORD16 deauth_reason)
{
    UWORD8        i        = 0;
    table_elmnt_t *tbl_elm = 0;
    table_elmnt_t *next_tbl_elm = 0;

	TROUT_FUNC_ENTER;
    /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
        	next_tbl_elm = tbl_elm->next_hash_elmnt;// 20120709 caisf add, merged ittiam mac v1.2 code

            /* Handle any Protocol related STA disconnection */
            handle_disconnect_req_prot_ap((asoc_entry_t*)tbl_elm->element,
                                          tbl_elm->key);

            /* Send the De-authentication Frame to the station */
            send_deauth_frame(tbl_elm->key, (UWORD16)deauth_reason);

            /* Send the sta leaving information to host */
            send_join_leave_info_to_host(((asoc_entry_t*)(tbl_elm->element))->asoc_id,
                                         tbl_elm->key, BFALSE);

            delete_entry(tbl_elm->key);

			// 20120709 caisf mod, merged ittiam mac v1.2 code
            //tbl_elm = tbl_elm->next_hash_elmnt;
			tbl_elm = next_tbl_elm;
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : unjoin_station                                           */
/*                                                                           */
/*  Description   : This function unjoins the station with given aid         */
/*                                                                           */
/*  Inputs        : 1) Deauth reason code                                    */
/*                  2) Association ID                                        */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : Deletes a sta entry and sends a de-auth                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void unjoin_station(UWORD16 deauth_reason, UWORD8 assoc_id)
{
    UWORD8        i        = 0;
    asoc_entry_t  *ae      = 0;
    table_elmnt_t *tbl_elm = 0;
    table_elmnt_t *next_tbl_elm = 0;// 20120709 caisf add, merged ittiam mac v1.2 code

	TROUT_FUNC_ENTER;
    /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
        	next_tbl_elm = tbl_elm->next_hash_elmnt;// 20120709 caisf add, merged ittiam mac v1.2 code
            ae = tbl_elm->element;
            if((ae->asoc_id & ~0xC000) == (assoc_id &  ~0xC000))
            {
                /* Handle any Protocol related STA disconnection */
                handle_disconnect_req_prot_ap(ae, tbl_elm->key);

                /* Send the De-authentication Frame to the station */
                send_deauth_frame(tbl_elm->key, (UWORD16)deauth_reason);

                /* Send the sta leaving information to host */
                send_join_leave_info_to_host(ae->asoc_id, tbl_elm->key, BFALSE);

                /* Delete the association entry of the identified sta */
                delete_entry(tbl_elm->key);
				TROUT_FUNC_EXIT;
                return;
            }
			// 20120709 caisf mod, merged ittiam mac v1.2 code
            //tbl_elm = tbl_elm->next_hash_elmnt;
			tbl_elm = next_tbl_elm;
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_sta_join_info_ap                                     */
/*                                                                           */
/*  Description   : This function gives the information about the required   */
/*                  joining/leaving station                                  */
/*                                                                           */
/*  Inputs        : 1) Addres of the station joing/leaving                   */
/*                  2) Pointer to which information is copied to             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the inforamtion about the station */
/*                  from its association entry table and updates the pointer */
/*                  with the same.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_sta_join_info_ap(asoc_entry_t *ae, UWORD8* sta_addr, UWORD8* ptr)
{
    /* If station address is not given, return 0 */
    if(sta_addr == NULL)
    {
        return 0;
    }

    /* If assosiation entry is not given, get it from STA address */
    if(ae == NULL)
    {
        ae = (asoc_entry_t *)find_entry(sta_addr);
    }

     /* If entry is not present, then return here */
     if(ae == NULL)
     {
         return 0;
     }

    /*************************************************************************/
    /* Format of STA JOIN INFO message                                       */
    /* ----------------------------------------------------------------------*/
    /* |Last byte| MAC address  | Type of STA    |  11g info  | Security     */
    /* | of AID  | of STA       | 11 a/b/g/n     |  byte      | byte         */
    /* ----------------------------------------------------------------------*/
    /* | 1 byte  | 6 bytes      | 1 byte         |  1 byte    |  1 byte      */
    /* ----------------------------------------------------------------------*/
    /*                                                                       */
    /* ----------------------------------------------------------------------*/
    /* | WMM/QoS info     | 11n capability     |   Reserved                  */
    /* | bytes            | bytes              |   bytes                     */
    /* ----------------------------------------------------------------------*/
    /* | 2 bytes          | 2 bytes            |   2 bytes                   */
    /* ----------------------------------------------------------------------*/

    /*************************************************************************/
    /*                'Format of 11g Info Byte'                              */
    /*-----------------------------------------------------------------------*/
    /* BIT0: 1 -> Short Preamble supported, 0 -> Not supported               */
    /* BIT1: 1 -> Short Slot supported,     0 -> Not supported               */
    /* BIT2 - BIT7: Reserved.                                                */
    /*************************************************************************/

     /* memset the pointer */
     mem_set(ptr, 0, STA_JOIN_INFO_LEN);

     /* Copy the STA address */
     memcpy((ptr + 1), sta_addr, 6);

     /* Association ID */
     *(ptr) = (UWORD8) (ae->asoc_id & 0xFF);

     if(ae->state == ASOC)
     {
         /* Type of station */
         /* Update the protocol specific fields */
         update_sta_phy_type_info(ae, ptr+7);

         /* 11g information byte */
         if(ae->short_preamble)
         {
             *(ptr + 8) |= BIT0;
         }

         if(ae->short_slot)
         {
             *(ptr + 8) |= BIT1;
         }

         /* Update the protocol specific fields */
         update_sta_info_prot(ae, ptr);
    }

    return STA_JOIN_INFO_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_all_sta_info_ap                                      */
/*                                                                           */
/*  Description   : This function gives the information about all the stas   */
/*                  that are associated with the AP                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to which information is copied to             */
/*                  2) Maximum number of bytes that can be copied            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the inforamtion about the all stas*/
/*                  from its association entry tables and updates the pointer*/
/*                  with the same.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_all_sta_info_ap(UWORD8* ptr, UWORD16 max_len)
{
    UWORD8        i        = 0;
    table_elmnt_t *tbl_elm = 0;
    UWORD16  len_copied     = 0;

    /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            /* Copy the connected station information to the pointer */
            len_copied += get_sta_join_info_ap(tbl_elm->element,
                                               tbl_elm->key,
                                               (ptr + len_copied));

            /* If maximum length of the pointer is reached, stop here */
            if(len_copied >= max_len - STA_JOIN_INFO_LEN)
            {
                i = MAX_HASH_VALUES;
                break;
            }

            /* Go to the next element */
            tbl_elm = tbl_elm->next_hash_elmnt;
        }
    }

    return len_copied;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_obss_erp_info                                    */
/*                                                                           */
/*  Description   : This function processes for ERP STAs detected in the     */
/*                  overlapping BSS.                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame (Beacon)                */
/*                  2) Length of the received frame                          */
/*                  3) RSSI value of the beacon                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if ERP protection is already        */
/*                  enabled and if there are any detections currently. If    */
/*                  not the frame is parsed to check for ERP/11b  STA        */
/*                  presence. If found, based on the ERP/11b STA detection   */
/*                  flag, appropriate action is taken.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_obss_erp_info(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi)
{
	/* Check if ERP protection is not enabled currently */
	if(get_protection() != ERP_PROT)
	{
		/* Enable protection and set the ERP information field bit */
		if((BTRUE == is_olbc_present(msa,rx_len)) &&
		   (BTRUE == is_autoprot_enabled()))
		{
			set_protection(ERP_PROT);
			set_machw_prot_control();
			update_beacon_erp_info();
			g_erp_aging_cnt=0;
		}
	}
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_probe_request_ap                                    */
/*                                                                           */
/*  Description   : This function sends the probe request frame              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sends the probe request frame              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_probe_request_ap(void)
{

	UWORD16 probe_req_frame_len = 0;
	UWORD8  *probe_req_frame = 0;

    probe_req_frame = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                             MANAGEMENT_FRAME_LEN);

    /* Check if the mem alloc returned a NULL Pointer */
    if(probe_req_frame == NULL)
        return;

	probe_req_frame_len = prepare_probe_request_ap(probe_req_frame);

	/* Transmit the management frame */
	tx_mgmt_frame(probe_req_frame, probe_req_frame_len, HIGH_PRI_Q, 0);

}
#endif /* BSS_ACCESS_POINT_MODE */
