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
/*  File Name         : ap_management_11n.c                                  */
/*                                                                           */
/*  Description       : This file contains the functions related to the MAC  */
/*                      management with 802.11n in AP mode of operation.     */
/*                                                                           */
/*  List of Functions : check_11n_params_ap                                  */
/*                      update_beacon_ht_info                                */
/*                      reset_11n_entry_ap                                   */
/*                      process_obss_nonht_sta                               */
/*                      set_11n_ht_ps_params_ap                              */
/*                      ap_enabled_action_req_11n                            */
/*                      update_2040_chan_info                                */
/*                      get_2040_op_chan_list                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management_ap.h"
#include "frame_11n.h"
#include "ap_management_11n.h"
#include "ap_frame_11n.h"
#include "prot_if.h"
#include "receive.h"
#include "core_mode_if.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_num_sta_nonht_asoc      = 0;
UWORD8 g_num_sta_20mhz_only      = 0;
UWORD8 g_num_sta_ngf             = 0;
UWORD8 g_num_sta_no_lsig_txop    = 0;
UWORD8 g_num_sta_no_40dsss_cck   = 0;
UWORD8 g_obss_non_ht_sta_present = 0;
BOOL_T g_40MHz_intol_bit_recd    = BFALSE;


/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_11n_params_ap                                      */
/*                                                                           */
/*  Description   : This function parses the HT specific information         */
/*                  elements and updates the STA's association  entry with   */
/*                   it.                                                     */
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
/*                  the asoc-entry are updated with them.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Association status code                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 check_11n_params_ap(UWORD8 *msa, UWORD16 offset, UWORD16 rx_len,
                          asoc_entry_t *entry)
{
	UWORD16     saved_index  = 0;
    UWORD8      ht_info_mod  = 0;
    UWORD16     index        = offset;
    UWORD8      mcs_bmp_index = 0;
    UWORD16     info_elem    = 0;
    BOOL_T      prev_asoc_ht = BFALSE;
    ht_struct_t *ht_hdl      = &(entry->ht_hdl);

    /* Check if the STA is already associated as a HT capable STA. If so the */
    /* global counters should be incremented only after checking if this STA */
    /* has changed any HT capabilities. Also HT capability flag is reset.    */
    if((entry->state == ASOC) && (ht_hdl->ht_capable == 1))
    {
        ht_hdl->ht_capable = 0;
        prev_asoc_ht       = BTRUE;
    }

    /* Search for HT Capabilities Element */
    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IHTCAP)
        {
            /* Do not allow HT STA with TKIP */
            if(TKIP == entry->cipher_type)
            {
				PRINTD("Rejecting a HT STA because of its TKIP Pairwise Cipher....\n");
				return MISMATCH_HTCAP;
			}

			/* Save the index */
			saved_index = index;

            /* Presence of HT Capability Element implies STA is HT capable.  */
            ht_hdl->ht_capable = 1;

            index += IE_HDR_LEN;

            /* Parse HT Capabilities Info Field */
            info_elem = MAKE_WORD16(msa[index], msa[index + 1]);

            /* Extract reception capability of LDPC coded packets */
            ht_hdl->ldpc_cod_cap = (info_elem & BIT0);

            /* Check the channel width supported field */
            if(0 == (info_elem & BIT1)) /* 20MHz Only */
            {
                if((prev_asoc_ht == BFALSE) ||
                   ((prev_asoc_ht == BTRUE) && (ht_hdl->chan_width == 1)))
                {
                    /* Update operating mode and global counter only if STA  */
                    /* was not associated previously or was associated as a  */
                    /* 20/40MHz capable STA                                  */
                    if(get_11n_op_type() == NTYPE_HT_2040_ONLY)
                    {
                        if(mget_HTOperatingMode() == 0)
                        {
                            ht_info_mod = 1;
                            set_ht_operating_mode(2);
                        }
                    }

                    ht_hdl->chan_width = 0;
                    g_num_sta_20mhz_only++;
                }
                else
                {
                    /* STA already associated as 20MHz Only STA. Do nothing. */
                }
            }
            else
            {
                ht_hdl->chan_width = 1;
            }

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

            /* Check the Greenfield field */
            if(0 == (info_elem & BIT4)) /* GF not supported */
            {
                if((prev_asoc_ht == BFALSE) ||
                   ((prev_asoc_ht == BTRUE) && (ht_hdl->greenfield == 1)))
                {
                    /* Update NonGFPresent flag and global counter only if   */
                    /* STA was not associated previously or was associated   */
                    /* as a Greenfield capable STA                           */
                    if(mget_NonGFEntitiesPresent() != TV_TRUE)
                    {
                        ht_info_mod = 1;
                        mset_NonGFEntitiesPresent(TV_TRUE);
                    }
                    ht_hdl->greenfield = 0;
                    g_num_sta_ngf++;
                }
                else
                {
                    /* STA already associated as a Non-GF STA. Do nothing. */
                }
            }
            else
            {
                ht_hdl->greenfield = 1;
            }

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

            /* Extract DSSS/CCK Mode in 40 MHz */
            ht_hdl->dsss_cck_40mhz = ((info_elem & BIT12) >> 12);

            /*40 MHz STA that doesnt support DSSS-CCK in 40 MHz */
            if((ht_hdl->dsss_cck_40mhz == 0) &&
               (ht_hdl->chan_width == 1))
            {
				g_num_sta_no_40dsss_cck++;
			}

            /* Check for forty MHz Intolerant bit */
            if(0 != (info_elem & BIT14))
            {
				PRINTD("40MHz Intol STA joining....\n");
				if(TV_TRUE == mget_FortyMHzOperationEnabled())
				{
					initiate_switch_to_20MHz_ap();
				}
				else
				{
					g_40MHz_intol_bit_recd = BTRUE;
				}
            }

            /* Check the L-SIG TXOP Protection support field */
            if(0 == (info_elem & BIT15)) /* L-SIG TXOP prot not supported */
            {
                if((prev_asoc_ht == BFALSE) ||
                   ((prev_asoc_ht == BTRUE) && (ht_hdl->lsig_txop_prot == 1)))
                {
                    /* Update NonGFPresent flag and global counter only if   */
                    /* STA was not associated previously or was associated   */
                    /* as a Greenfield capable STA                           */
                    if(mget_LSigTxopFullProtectionEnabled() != TV_FALSE)
                    {
                        ht_info_mod = 1;
                        mset_LSigTxopFullProtectionEnabled(TV_FALSE);
                    }
                    ht_hdl->lsig_txop_prot = 0;
                    g_num_sta_no_lsig_txop++;
                }
                else
                {
                    /* STA already associated as NoLSigTxop STA. Do nothing. */
                }
            }
            else
            {
                ht_hdl->lsig_txop_prot = 1;
            }

            /* Enable GF protection if required */
            if((is_autoprot_enabled() == BTRUE) &&
               (get_protection() == NO_PROT)    &&
               (mget_NonGFEntitiesPresent() == TV_TRUE))
            {
                set_protection(GF_PROT);

                if(is_rifs_prot_enabled() == 0)
                    enable_machw_gf_rifs_prot();
            }

            index += HT_CAPINFO_FIELD_LEN;

            /* Parse A-MPDU Parameters Field */

            /* Extract the Maximum Rx A-MPDU factor (B1 - B0) */
            ht_hdl->max_rx_ampdu_factor = (msa[index] & 0x03);

            /* Extract the Minimum MPDU Start Spacing (B2 - B4) */
            ht_hdl->min_mpdu_start_spacing = (msa[index] >> 2)  & 0x07;

            index += AMPDU_PARAM_LEN;

            /* Parse Supported MCS Set Field */

            /* AND the Rx MCS bitmask with supported Tx MCS bitmask          */
            /* and reset the last 3 bits in the last                         */
            /* byte since the bitmask is 77 bits                             */

            for(mcs_bmp_index = 0; mcs_bmp_index < MCS_BITMASK_LEN;
                mcs_bmp_index++)
            {
                ht_hdl->rx_mcs_bitmask[mcs_bmp_index] =
                    mget_SupportedMCSTxValue().mcs_bmp[mcs_bmp_index] &
                    (*(UWORD8 *)(msa + index + mcs_bmp_index));
            }

            ht_hdl->rx_mcs_bitmask[MCS_BITMASK_LEN - 1] &= 0x1F;

            index += MCS_SET_FIELD_LEN;

            /* Parse HT Extended Capabilities Info Field */
            info_elem = MAKE_WORD16(msa[index], msa[index + 1]);

            /* Extract HTC support Information */
            if((info_elem & BIT10) != 0)
                ht_hdl->htc_support = 1;

            index += HT_EXTCAP_FIELD_LEN;

            /* Parse Tx Beamforming Field */
            index += TX_BF_FIELD_LEN;

            /* Parse ASEL Capabilities Field */
            index += ASEL_FIELD_LEN;

            /* Update the index back before searching for next elements */
            index = saved_index + msa[saved_index + 1] + IE_HDR_LEN;
        }
        else
        {
            /* Increment index by length information & tag header */
            index += msa[index + 1] + IE_HDR_LEN;
        }
    }

    /* If the STA is not HT capable disallow association if 11n operating    */
    /* type is not NTYPE_MIXED. If it is NTYPE_MIXED, update the HWOpMode to */
    /* 3. Also update the global number of non-HT STAs                       */
    if(ht_hdl->ht_capable == 0)
    {
        if(get_11n_op_type() == NTYPE_MIXED)
        {
            if(mget_HTOperatingMode() != 3)
            {
                ht_info_mod = 1;
                set_ht_operating_mode(3);
                disable_rifs();
            }

            g_num_sta_nonht_asoc++;

            /* If auto protection is set, and neither ERP nor HT protection  */
            /* is currently in use, enable HT protection.                    */
            if((is_autoprot_enabled() == BTRUE) &&
               (get_protection() != ERP_PROT)   &&
               (get_protection() != HT_PROT))
            {
                set_protection(HT_PROT);
            }
        }
        else
        {
            return MISMATCH_HTCAP;
        }
    }
    /* If the STA is not 2040 capable and 11n Op type is NTYPE_HT_2040_ONLY */
    /* Then the STA needs to be rejected if AP is a 20/40 capable one       */
    /* Note that Rejection need to be done even if currently 2040 isnt used */
    /* This is because we can switch to 40 MHz any time if its capable      */
    else if (ht_hdl->chan_width == 0)
    {
        if((get_11n_op_type() == NTYPE_HT_2040_ONLY) &&
           (mget_FortyMHzOperationImplemented() == TV_TRUE))
        {
            return MISMATCH_HTCAP;
		}
	}

    /* Update the HT Information element in the beacon if required */
    if(ht_info_mod == 1)
        update_beacon_ht_info();


    /* If any STA that doesnt support DSSS-CCK mode in 40 MHz */
    if(g_num_sta_no_40dsss_cck != 0)
	{
	   /* If any 11b STA is present in the vicinity */
       if((g_num_sta_non_erp != 0) ||
          (ERP_PROT == get_protection()))
	   {
		  /* Since DSSS-CCK-DISABLE in 40 MHz and 11b conflict */
		  /* switch to 20 MHz mode, if we are in 40 MHz mode now */
		  if(TV_TRUE == mget_FortyMHzOperationEnabled())
		  {
				PRINTD("11b STA and 40-MHz-non-DSS-STA both joined....\n");
          		initiate_switch_to_20MHz_ap();
	      }
	   }
	}


    /* Print the STA's HT-Capabilities */
    PRINTD("STA HT-Capabilities:\n\r");
    PRINTD("ht_capable             = %x\n\r", ht_hdl->ht_capable);
    PRINTD("Channel width          = %x\n\r", ht_hdl->chan_width);
    PRINTD("ldpc_cod_cap           = %x\n\r", ht_hdl->ldpc_cod_cap);
    PRINTD("smps_mode              = %x\n\r", ht_hdl->smps_mode);
    PRINTD("greenfield             = %x\n\r", ht_hdl->greenfield);
    PRINTD("short_gi_20            = %x\n\r", ht_hdl->short_gi_20);
    PRINTD("short_gi_40            = %x\n\r", ht_hdl->short_gi_40);
    PRINTD("rx_stbc                = %x\n\r", ht_hdl->rx_stbc);
    PRINTD("max_rx_ampdu_factor    = %x\n\r", ht_hdl->max_rx_ampdu_factor);
    PRINTD("min_mpdu_start_spacing = %x\n\r", ht_hdl->min_mpdu_start_spacing);
    PRINTD("htc_support            = %x\n\r", ht_hdl->htc_support);
    PRINTD("sta_amsdu_maxsize      = %x\n\r", ht_hdl->sta_amsdu_maxsize);
    PRINTD("chan_width             = %x\n\r", ht_hdl->chan_width);
    PRINTD("dsss_cck_40mhz         = %x\n\r", ht_hdl->dsss_cck_40mhz);
    PRINTD("cipher_type            = %x\n\r", entry->cipher_type);

    return SUCCESSFUL_STATUSCODE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_beacon_ht_info                                    */
/*                                                                           */
/*  Description   : This function updates the HT information field in the    */
/*                  beacon frame and sets the MAC H/w pointer.               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches for the HT inromation field and   */
/*                  updates the same inthe beacon frame. It also updates the */
/*                  MAC H/w beacon pointer.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_beacon_ht_info(void)
{
    UWORD16 idx = TAG_PARAM_OFFSET;

    /* NULL check before accessing the beacon pointer */
    if((g_beacon_index > 1) ||
       (g_beacon_frame[g_beacon_index] == NULL))
    	return;

    /* Traverse the beacon till HT Info element   */
    while(idx < g_beacon_len)
    {
        if(g_beacon_frame[g_beacon_index][idx] == IHTOPERATION)
        {
            /* Once the position of the HT in the beacon is found, set the   */
            /* value                                                         */
            set_ht_opern_11n_ap(g_beacon_frame[g_beacon_index], idx);
            break;
        }
        else
        {
            idx += (IE_HDR_LEN + g_beacon_frame[g_beacon_index][idx + 1]);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_11n_entry_ap                                       */
/*                                                                           */
/*  Description   : This function resets the 11n related information in the  */
/*                  association entry.                                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association entry                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the HT information of the entry and */
/*                  updates the required globals, MIB and beacon frame. It   */
/*                  also processes for auto protection and enables required  */
/*                  protection mechanism.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_11n_entry_ap(asoc_entry_t *ae)
{
    UWORD8 tid         = 0;
    UWORD8 ht_info_mod = 0;

    if(ae->ht_hdl.ht_capable == 0) /* Non-HT STA */
    {
        g_num_sta_nonht_asoc--;

        if(g_num_sta_nonht_asoc == 0) /* Only HT STAs remaining */
        {
            if(get_obss_non_htsta_present() == 1)
            {
                /* If there are OBSS non-HT STAs present check the detection */
                /* setting and continue with the same operating mode or      */
                /* change to an operating mode of 1.                         */
                if(get_11n_obss_detection() == DETECT_ONLY)
                {
                    set_ht_operating_mode(1);

                    /* For OM = 1, RIFS mode must be disabled if RIFS        */
                    /* protection is enabled at the AP.                      */
                    if(get_11n_rifs_prot_enable() == 1)
                        disable_rifs();

                    ht_info_mod = 1;
                }
                else /* DETECT_PROTECT || DETECT_PROTECT_REPORT */
                {
                    /* No change in operating mode (continues to be 3) or   */
                    /* protection mode.                                     */
                }
            }
            else
            {
                ht_info_mod = 1;

                /* There are no associated or overlapping non-HT STAs. Reset */
                /* the operating mode to 0 or 2 based on the 11n operating   */
                /* mode and value of the global g_num_sta_20mhz_only         */
                if((get_11n_op_type() == NTYPE_HT_2040_ONLY) &&
                   (g_num_sta_20mhz_only != 0))
                {
                    set_ht_operating_mode(2);
                }
                else
                {
                    set_ht_operating_mode(0);
                }

                /* Enable RIFS if operating mode is set to 0 or 2 */
                enable_rifs();

                /* If auto protection is set and there are nonGFSTAs present */
                /* enable GF protection. ERP/HT protection is not required.  */
                if((is_autoprot_enabled() == BTRUE) &&
                   (mget_NonGFEntitiesPresent() == TV_TRUE))
                {
                    set_protection(GF_PROT);

                    if(is_rifs_prot_enabled() == 0)
                        enable_machw_gf_rifs_prot();
                }
            }
        }
    }
    else /* HT STA */
    {
        /* STA supported only 20MHz */
        if(ae->ht_hdl.chan_width == 0)
        {
            g_num_sta_20mhz_only--;

            if(g_num_sta_20mhz_only == 0)
            {
                ht_info_mod = 1;
                set_ht_operating_mode(0);
            }
        }

        /* 40 MHz STA that doesnt support DSSS-CCK in 40 MHz */
        if((ae->ht_hdl.dsss_cck_40mhz == 0) &&
           (ae->ht_hdl.chan_width == 1))
        {
        	g_num_sta_no_40dsss_cck--;
		}

        /* STA did not support Greenfield */
        if(ae->ht_hdl.greenfield == 0)
        {
            g_num_sta_ngf--;

            if(g_num_sta_ngf == 0)
            {
                ht_info_mod = 1;
                mset_NonGFEntitiesPresent(TV_FALSE);
            }

            /* Disable GF protection if required */
            if((is_autoprot_enabled() == BTRUE) &&
               (get_protection() == GF_PROT)    &&
               (mget_NonGFEntitiesPresent() == TV_FALSE))
            {
                set_protection(NO_PROT);

                if(is_rifs_prot_enabled() == 0)
                    disable_machw_gf_rifs_prot();
            }
        }

        /* STA did not support L-SIG TXOP Protection */
        if(ae->ht_hdl.lsig_txop_prot == 0)
        {
            g_num_sta_no_lsig_txop--;

            if(g_num_sta_no_lsig_txop == 0)
            {
                ht_info_mod = 1;
                mset_LSigTxopFullProtectionEnabled(TV_TRUE);
            }
        }
    }

    if(ht_info_mod == 1)
        update_beacon_ht_info();

    /* Reset all active AMSDU and AMPDU sessions setup for the STA */
    for(tid = 0; tid < 16; tid++)
    {
        reset_tx_amsdu_session(ae, tid);
        reset_tx_ampdu_session(ae, tid);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_obss_nonht_sta                                   */
/*                                                                           */
/*  Description   : This function processes for non-HT STAs detected in the  */
/*                  overlapping BSS.                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame (Beacon)                */
/*                  2) Length of the received frame                          */
/*                  3) RSSI value of the beacon                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if OBSS non-HT STA detection is     */
/*                  enabled and if there are any detections currently. If    */
/*                  not the frame is parsed to check for OBSS non-HT STA     */
/*                  presence. If found, based on the OBSS non-HT detection   */
/*                  flag, appropriate action is taken.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_obss_nonht_sta(UWORD8 *msa, UWORD16 rx_len, WORD8 rssi)
{
    /* If OBSS non-HT STA detection is disabled or if there are already OBSS */
    /* non-HT STAs detected, no processing is required                       */
    if((get_11n_obss_detection() == NO_DETECT) ||
       (get_obss_non_htsta_present() == 1))
    {
        return;
    }

    /* Parse frame and check the OBSS non-HT STA present field in the HT     */
    /* Information element. This is also set if no HT field is present in    */
    /* the frame. If there are OBSS non-HT STAs present, appropriate action  */
    /* is taken based on the 11n OBSS detection setting.                     */
    if(detect_obss_nonht_sta(msa, rx_len) == 1)
    {

        set_obss_non_htsta_present(1);
        set_ht_operating_mode(1);

        if(get_11n_obss_detection() == DETECT_ONLY)
        {
            /* For OM = 1, RIFS mode must be disabled if RIFS protection is  */
            /* enabled at the AP.                                            */
            if(get_11n_rifs_prot_enable() == 1)
            {
                disable_rifs();
            }
        }
        else /* DETECT_PROTECT || DETECT_PROTECT_REPORT */
        {
            /* If auto protection is set, and neither ERP nor HT protection  */
            /* is currently in use, enable HT protection.                    */
            if((is_autoprot_enabled() == BTRUE) &&
               (get_protection() != ERP_PROT)   &&
               (get_protection() != HT_PROT))
            {
                set_protection(HT_PROT);
            }

            /* For OM = 3, RIFS mode must be disabled at AP */
            if(mget_RIFSMode() == TV_TRUE)
            {
                disable_rifs();
            }
        }

        /* Update the beacon with the new HT information */
        update_beacon_ht_info();
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_11n_ht_ps_params_ap                                  */
/*                                                                           */
/*  Description   : This function sets the SMPS parameters in the  transmit  */
/*                  descriptor.                                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                  2) Pointer to the station entry                          */
/*                  3) Transmit rate                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the SMPS mode for the station in the  */
/*                  descriptor as per the defined format. It also updates    */
/*                  the date rate field if required for Static SMPS mode.    */
/*                  Note that this function overwrites the data rate field   */
/*                  as required and hence should be called after the data    */
/*                  rate has been set.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to transmit descriptor                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_11n_ht_ps_params_ap(UWORD8 *tx_dscr, asoc_entry_t *ae, UWORD8 rate)
{
    UWORD8 smps_mode    = 0;
    UWORD8 max_mcs_rate = 0;

    if(ae == NULL)
        return;

    /* Get the SMPS mode for the station */
    smps_mode = ae->ht_hdl.smps_mode;

    /* Set the SMPS mode field in the Tx descriptor */
    set_tx_dscr_smps_mode((UWORD32 *)tx_dscr, smps_mode);

    /* If the SMPS mode is static check the data rate. If it is an HT rate,  */
    /* it must be a basic MCS. Ensure this and reset it if required.         */
    if(smps_mode == STATIC_MODE)
    {
        max_mcs_rate = get_phy_802_11n_max_br_htonly();

        if(IS_RATE_MCS(rate) == 1)
        {
            // 20120830 caisf mod, merged ittiam mac v1.3 code
            #if 0
            if(rate > max_mcs_rate)
                set_tx_dscr_data_rate_0((UWORD32 *)tx_dscr, max_mcs_rate);
            #else
            if(rate > max_mcs_rate)
                set_tx_dscr_data_rate_0((UWORD32 *)tx_dscr, EQUIVALENT_1X1_RATE(rate));
            #endif
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_action_req_11n                                */
/*                                                                           */
/*  Description   : This function calls a MAC core function to handle the    */
/*                  incoming action request.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
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

void ap_enabled_action_req_11n(UWORD8 *msg)
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
/*  Function Name : update_sta_info_11n                                      */
/*                                                                           */
/*  Description   : This function updates the 11n specific station           */
/*                  information fields.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association entry of the station       */
/*                  2) Pointer buffer to update the information fields       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the 11n information about the     */
/*                  station from its association entry table and updates the */
/*                  fields with the same.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

/*************************************************************************/
/*                'Format of 11N Info Bytes'                             */
/*-----------------------------------------------------------------------*/
/* BYTE-0:                                                               */
/* b0    : 1 -> HT Supported             0 -> Not supported              */
/* b1    : 1 -> Immediate BA Supported   0 -> Not supported              */
/* b2    : 1 -> HTC Supported            0 -> Not supported              */
/* b3    : 1 -> AMSDU Max Length=7935    0 -> AMSDU Max Length=3839      */
/* b4:b5 : Max AMPDU Rx Factor                                           */
/* b6:b7 : RX Antenna Config Supported 0->1X1  1->2X2 2->3X3 3->4X4      */
/* BYTE-1: UAPSD Status                                                  */
/* b0    : 1 -> 20/40 Supported          0 -> Not supported              */
/* b1    : 1 -> 20MHz SGI Supported      0 -> Not supported              */
/* b2    : 1 -> 40MHz SGI Supported      0 -> Not supported              */
/* b3    : 1 -> Greenfield Supported     0 -> Not supported              */
/* b4    : 1 -> LDPC coding Supported    0 -> Not supported              */
/* b5    : 1 -> LSIG TXOP Prot Supported 0 -> Not supported              */
/* b6:b7 : SM-PS Mode 0->Resvd 1->Static 2->Dynamic 3->MIMO              */
/*************************************************************************/

void update_sta_info_11n(asoc_entry_t *ae, UWORD8 *info)
{
    ht_struct_t  *ht =  &ae->ht_hdl;

    info[0] = info[1] = 0;

    if(ht->ht_capable == 1)
    {
        /* HT-Capability */
        info[0] |= BIT0;

        /* Immediate BA Capability */
        if(ae->ba_hdl.imm_ba_capable)
            info[0] |= BIT1;

        /* HTC Capability */
        if(ht->htc_support)
            info[0] |= BIT2;

        /* Max AMSDU frame length RX Capability */
        if(ht->sta_amsdu_maxsize == 7935)
            info[0] |= BIT3;

        /* Max AMPDU frame length RX Capability */
        info[0] |= (ht->max_rx_ampdu_factor << 4);

#ifdef AUTORATE_FEATURE //dumy add for test 0711
        /* Station is considered to be 4x4 if MCS24 is supported */
        if(is_mcs_supp(ae, 24) == BTRUE)
            info[0] |= (3 << 6);
        /* Station is considered to be 3x3 if MCS16 is supported */
        else if(is_mcs_supp(ae, 16) == BTRUE)
            info[0] |= (2 << 6);
        /* Station is considered to be 2x2 if MCS8 is supported */
        else if(is_mcs_supp(ae, 8) == BTRUE)
            info[0] |= (1 << 6);
        /* By default the Station is considered to be 1x1 */
        else
#endif /* AUTORATE_FEATURE */            
            info[0] |= (0 << 6);

        /* Channel Width Capability */
        if(ht->chan_width)
            info[1] |= BIT0;

        /* 20MHz SGI RX Capability */
        if(ht->short_gi_20)
            info[1] |= BIT1;

        /* 40MHz SGI RX Capability */
        if(ht->short_gi_40)
            info[1] |= BIT2;

        /* Greenfield frame RX Capability */
        if(ht->greenfield)
            info[1] |= BIT3;

        /* LDPC Coding Capability */
        if(ht->ldpc_cod_cap)
            info[1] |= BIT4;

        /* LSIG-TXOP protection Capability */
        if(ht->lsig_txop_prot)
            info[1] |= BIT5;

        /* SM-PS Mode Advertised */
        info[1] |= (ht->smps_mode << 6);
    }
}
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */
