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
/*  File Name         : ieee_11i_ap.c                                        */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the AP MAC FSM on receiving      */
/*                      802.11i MISC events.                                 */
/*                                                                           */
/*  List of Functions : check_rsn_capabilities_ap                            */
/*                      handle_11i_tx_comp_ap                                */
/*                      get_pmkid_asoc                                       */
/*                      update_11i_fail_stats_ap                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame_11i.h"
#include "auth_frame_11i.h"
#include "rsna_auth_km.h"
#include "rkmal_auth.h"
#include "management_ap.h"
#include "index_util.h"
#include "receive.h"
#include "prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_rsn_capabilities_ap                             */
/*                                                                           */
/*  Description      : This procedure verifies the RSNA Capabilites in Assoc */
/*                     and reassoc frames                                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Asoc/Reasoc Frame                   */
/*                     2) Length of the Received frame                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*                                                                           */
/*  Outputs          : Updates the AKM and PW Policies with the values in the*/
/*                     received frame                                        */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 check_rsn_capabilities_ap(asoc_entry_t *ae, UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 status = SUCCESSFUL_STATUSCODE;

    /* If RSNA is enabled, the RSN IE is a must in the ASOC/ReASOC Frames */
    if(mget_RSNAEnabled() == TV_TRUE)
    {
        UWORD16 index        = MAC_HDR_LEN;
        WORD32  rsn_found     = 0;
        UWORD8  auth_policy  = 0;
        UWORD8  pcip_policy  = 0;
        UWORD8  grp_policy   = 0;
        UWORD8  mode_802_11i = 0;
        UWORD8  temp         = 0;
        UWORD8  *oui         = 0;


        index += 2 /* Cap Info */
               + 2 /* Listen Interval */;

        if(get_sub_type(msa) == ASSOC_REQ)
        {
        }
        else
        {
            index += 6 /* AP Addr  */;
        }

        while(index < (rx_len - FCS_LEN))
        {
            /* Check if RSNA or WPA IE are present */
            if((g_mode_802_11i & RSNA_802_11I) && (msa[index] == IRSNELEMENT))
            {
                rsn_found    = 1;
                mode_802_11i = RSNA_802_11I;
                oui          = mget_RSNAOUI();
            }
            else if((g_mode_802_11i & WPA_802_11I) && (is_wpa_ie(msa + index) == BTRUE))
            {
                rsn_found    = 1;
                mode_802_11i = WPA_802_11I;
                oui          = mget_WPAOUI();
            }

            /* Process the RSN or WPA IE if found */
            if(rsn_found == 1)
            {
                /* Skip the tag number (1) and the length (1) */
                index += 2;

                 /* Skip the WPA OUI and OUI Type for WPA IE */
                if(mode_802_11i == WPA_802_11I)
                    index += 4;

                /* Verify the 802.11i version number */
                if(msa[index] != mget_RSNAConfigVersion())
                {
                    status    = UNSUP_RSN_INFO_VER;
                    rsn_found = -1;
                    break;
                }

                /* Skip the version number */
                index += 2;

                /* Check the OUI for the group cipher policy */
                if(check_oui(&msa[index], oui) == BFALSE)
                {
                    status = INVALID_INFO_ELMNT;
                    rsn_found = -1;
                    break;
                }

                /* Skip the OUI */
                index += 3;

                /* Extract Group, AKM, Pairwise Cipher policies */
                grp_policy = msa[index];
                index++;

                pcip_policy = get_pcip_policy_auth(&msa[index], &temp, oui);
                index      += temp;

                auth_policy = get_auth_policy_auth(&msa[index], &temp, oui);
                index      += temp;

                /* If pairwise policy is not supplied it is taken to be same */
                /* as the negotiated Group cipher policy                     */
                if(pcip_policy == 0)
                {
                    pcip_policy = grp_policy;
                }

                /* Verify the Group policy */
                if(mget_RSNAConfigGroupCipher() != grp_policy)
                {
                    status    = INVALID_GRP_CIPHER;
                    rsn_found = -1;
                    break;
                }

                /* Verify the AKM policy */
                if(check_auth_policy(auth_policy) == BFALSE)
                {
                    status    = INVALID_AKMP_CIPHER;
                    rsn_found = -1;
                    break;
                }

                /* Verify the Pairwise Cipher policy */
                if(check_pcip_policy(pcip_policy) == BFALSE)
                {
                    status    = INVALID_PW_CIPHER;
                    rsn_found = -1;
                    break;
                }

                /* Prohibit use of TKIP as Pairwise Cipher when CCMP is the  */
                /* Group Cipher                                              */
                if((grp_policy == 0x04) && (pcip_policy == 0x02))
                {
                    status    = CIPHER_REJ;
                    rsn_found = -1;
                    break;
                }

                /* For 802.11I RSNA, RSN Capabilities must match */
                if(mode_802_11i == RSNA_802_11I)
                {
                    /* RSN Capabilities Information                          */
                    /* ----------------------------------------------------- */
                    /* | B15 - B6    | B5 - B4          | B3 - B2          | */
                    /* ----------------------------------------------------- */
                    /* | Reserved    | GTSKA Replay Ctr | PTSKA Replay Ctr | */
                    /* ----------------------------------------------------- */
                    /* | B1             | B0                               | */
                    /* ----------------------------------------------------- */
                    /* | No Pairwise    | Pre-Authentication               | */
                    /* ----------------------------------------------------- */

                    /* Check Pre-Authentication capability */
                    if((mget_RSNAPreauthenticationImplemented() == TV_TRUE) &&
                       (mget_RSNAPreauthenticationEnabled() == TV_TRUE))
                    {
                        if(!(msa[index] & BIT0))
                        {
                            status    = INVALID_RSN_INFO_CAP;
                            rsn_found = -1;
                            break;
                        }
                    }
                    else if(msa[index] & BIT0)
                    {
                        status    = INVALID_RSN_INFO_CAP;
                        rsn_found = -1;
                        break;
                    }

                    /* Check Pairwise field in the Capability field        */
                    /* This shall be set only if the P/W policy is TKIP &  */
                    /* in a TSN                                            */
                    if((grp_policy != 0x01) && (grp_policy != 0x05))
                    {
                    }
                    else if((msa[index] & BIT1) && (pcip_policy == 0x02))
                    {
                        pcip_policy = grp_policy;
                    }

                    /* Check PTSKA Replay counters. This field is taken from */
                    /* the MIB's dot11RSNAConfigNumberOfPTSKAReplayCounters  */
                    if((msa[index] & (BIT2 | BIT3)) !=
                       ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2)
                         & 0x000C))
                    {
                        /* Replay counter if less than our value implies 11e */
                        /* mismatch, so the packets should not be sent in    */
                        /* other queues, to be taken care of by MAC S/w      */
                    }

                    /* Check GTSKA Replay counters. This field is taken from */
                    /* the MIB's dot11RSNAConfigNumberOfGTSKAReplayCounters  */
                    if((msa[index] & (BIT4 | BIT5)) !=
                        ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4)
                           & 0x0030))
                    {
                        /* Replay counter if less than our value implies 11e */
                        /* mismatch, so the packets should not be sent in    */
                        /* other queues, to be taken care of by MAC S/w      */
                    }
                }

                /* Update the MIB Parameters */
                mset_RSNAAuthenticationSuiteRequested(auth_policy);
                mset_RSNAPairwiseCipherRequested(pcip_policy);
                mset_RSNAGroupCipherRequested(grp_policy);
                mset_RSNAModeRequested(mode_802_11i);
                break;
            }
            else if(msa[index] == IRSNELEMENT)
            {
                /* On failure status code is set appropriately*/
                status    = INVALID_INFO_ELMNT;
                rsn_found = -1;
                break;
            }
            else
            {
                /* Increment the index by length information & tag header */
                index += msa[index + 1] + IE_HDR_LEN;
            }
        }

        if(rsn_found != 1)
        {
            if(rsn_found == 0)
            {
                UWORD8 grp_policy = mget_RSNAConfigGroupCipher();
                if((grp_policy != 0x01) && (grp_policy != 0x05))
                {
                    status = UNSPEC_FAIL;
                }
                else
                {
                    mset_RSNAPairwiseCipherRequested(grp_policy);
                }
            }
        }

		if(status == SUCCESSFUL_STATUSCODE)
		{
			ae->cipher_type = cipsuite_to_ctype(pcip_policy);
		}
    }

    return status;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_11i_tx_comp_ap                                    */
/*                                                                           */
/*  Description   : This function checks PN-val wrap around                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the packet descriptor                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks PN-val wrap around                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
void handle_11i_tx_comp_ap(UWORD8 *dscr, asoc_entry_t *ae)
{
    UWORD8       *msa     = (UWORD8*)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);
#else
void handle_11i_tx_comp_ap(UWORD8 *dscr, asoc_entry_t *ae, UWORD8 *msa)
{
#endif
    CIPHER_T      cp      = (CIPHER_T) get_tx_dscr_cipher_type((UWORD32 *)dscr);
    UWORD8       *da      = NULL;

    da = get_address1_ptr(msa);

    /* Continue anyfurther only if the cipher policy is AES or TKIP */
    if((cp == TKIP) || (cp == CCMP))
    {
        if(get_type(msa) == DATA_BASICTYPE)
        {
            if(is_group(da) == BTRUE)
            {
                /* Check the BCast PN Value           */
                /* If it overflows, schedule rekeying */
                UWORD32 temp[2] = {0};
                UWORD32 templ   = get_tx_dscr_iv32l((UWORD32 *)dscr);
                UWORD32 temph   = get_tx_dscr_iv32h((UWORD32 *)dscr);
                UWORD32 cfg_val = mget_RSNAConfigGroupRekeyPackets();
                get_iv_2_pn_val(templ, temph, (UWORD8)cp, temp);

                /* If the method is the packet based rekey schedule, check for */
                /* the PN Val limit, and schedule the rekeying.                */
                /* Else check for the wrap around condition                    */
                if(mget_RSNAConfigGroupRekeyMethod() > 2)
                {
                    if(cfg_val <= 4294967) /* (2^32 - 1)/1000 */
                    {
                        if(temp[0] > (cfg_val * 1000))
                        {
                            /* Reschedule Rekeying */
#ifndef OS_LINUX_CSL_TYPE
                            rekey_cb_fn(0, 0);
#else /* OS_LINUX_CSL_TYPE */
                            rekey_cb_fn(0);
#endif /* OS_LINUX_CSL_TYPE */
                        }
                    }
                    else
                    {
                        /* Computing the value of pn/1000 and comparing it */
                        /* with the configured value                       */
                        if(((temp[1] * 4294967) + (temp[0]/1000)) > cfg_val)
                        {
                            /* Reschedule Rekeying */
							post_rekey_event(DO_NEXT_REKEY);
                        }
                    }
                }
                else
                {
                    if((temp[0] == 0xFFFF) && (temp[1] == 0xFFF0))
                    {
                        /* Reschedule Rekeying */
					    post_rekey_event(DO_NEXT_REKEY);
                    }
                }
            }
            else
            {
                UWORD32 temp[2] = {0};
                UWORD32 templ   = 0;
                UWORD32 temph   = 0;

                if(ae == NULL)
                {
                    return;
                }

                /* Check the UCast PN Value for the TID */
                /* If it overflows, schedule deauth     */
                templ   = get_tx_dscr_iv32l((UWORD32 *)dscr);
                temph   = get_tx_dscr_iv32h((UWORD32 *)dscr);
                get_iv_2_pn_val(templ, temph, (UWORD8)cp, temp);

                if((temp[0] == 0xFFFF) && (temp[1] == 0xFFF0))
                {
                    /* Send Deauth */
                    rsna_send_deauth((UWORD8 *)ae->persta, 0);
                }
            }
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_pmkid_asoc                                           */
/*                                                                           */
/*  Description   : This function returns the PMKID pointer from the asoc req*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the association packet                     */
/*                  2) Size of the packet                                    */
/*                  3) Pointer to the buffered pmkid                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks PN-val wrap around                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *get_pmkid_asoc(UWORD8 *msa, UWORD16 rx_len, UWORD8 *pmkid)
{
    UWORD16 i = 0;

    /* If RSNA is disabled, return NULL */
    if(!(g_mode_802_11i & RSNA_802_11I))
    {
        return NULL;
    }

    /* Check if something is actually buffered */
    /* else return NULL                        */
    for(i = 0; i < PMKID_LEN; i++)
    {
        if(pmkid[i] != 0)
        {
            break;
        }
    }
    if(i == PMKID_LEN)
    {
        return NULL;
    }

    i = MAC_HDR_LEN + 2 /* Cap Info        */
                    + 2 /* Listen Interval */;

    if(get_sub_type(msa) == ASSOC_REQ)
    {
    }
    else
    {
        i += 6 /* AP Addr  */;
    }

    while(i < (rx_len - FCS_LEN))
    {
        /* Read if the Element is RSN Information */
        if(msa[i] == IRSNELEMENT)
        {
            msa += i;

            /* Skip the Tag number (1)  and the length (1) */
            i = 2;

            /* Skip the version number */
            i += 2;

            /* Get AKM, PW, and GW Cipher policies from the frame */
            {
                UWORD8 temp = 0;

                i += 4; /* Skip the OUI Length */

                get_pcip_policy_auth(&msa[i], &temp, mget_RSNAOUI());
                i += temp;

                get_auth_policy_auth(&msa[i], &temp, mget_RSNAOUI());
                i += temp;
            }

            /* For 802.11I RSNA, RSN Capability field must match */
            i += 2;

            if((i >= msa[1] + 1) || (msa[i] == 0))
            {
                return NULL;
            }
            else
            {
                UWORD8 j = 0;
                UWORD8 k = 0;

                /* Checking if atleast one of the PMKIDs match */
                for(k = 0; k < msa[i]; k++)
                {
                    for(j = 0; j < PMKID_LEN; j++)
                    {
                        if(pmkid[j] != msa[(i + 2) + (k * PMKID_LEN) + j])
                        {
                            continue;
                        }
                    }
                    if(j == PMKID_LEN)
                    {
                        return pmkid;
                    }
                }
                return NULL;
            }
        }
        else
        {
            /* Increment the index by length information & tag header */
            i += msa[i + 1] + IE_HDR_LEN;
        }
    }
    return NULL;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_11i_fail_stats_ap                                 */
/*                                                                           */
/*  Description   : This function updates the security failure statistics.   */
/*                                                                           */
/*  Inputs        : 1) Cipher type                                           */
/*                  2) Receive status                                        */
/*                  3) Source address                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the receive status and updates the  */
/*                  required statistics and triggers any counter measures    */
/*                  required.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_11i_fail_stats_ap(CIPHER_T ct, UWORD8 status, UWORD8 *sa)
{

    UWORD8       index = mget_RSNAStatsIndex(sa);
    asoc_entry_t *ae   = (asoc_entry_t *)find_entry(sa);

    if(ae == NULL)
        return;

    if(index != 0xFF)
    {
        if(status == MIC_FAILURE)
        {
            if(ct == CCMP)
            {
                mincr_RSNAStatsCCMPDecryptErrors(index);
                g_decr_fail_cnt[index]++;
            }
        }
        else if(status == ICV_FAILURE)
        {
            if(ct == TKIP)
            {
                 mincr_RSNAStatsTKIPICVErrors(index);
                 g_decr_fail_cnt[index]++;
            }
        }
        else if(status == TKIP_MIC_FAILURE)
        {
            if(ct == TKIP)
            {
                /* Increment the MIB MIC Failure Count */
                mincr_RSNAStatsTKIPLocalMICFailures(
                                            (UWORD8)ae->persta->key_index);

                /* MIC Failure: Counter Measures invoked */
                start_auth_cntr_msr_timer();
            }

        }
        else if(status == TKIP_REPLAY_FAILURE)
        {
            /* The PN Sequence as received is out of order */
            /* Replay error is logged                      */
            mincr_RSNAStatsTKIPReplays((UWORD8)ae->persta->key_index);
        }
        else if(status == CCMP_REPLAY_FAILURE)
        {
            /* The PN Sequence as received is out of order */
            /* Replay error is logged                      */
            mincr_RSNAStatsCCMPReplays((UWORD8)ae->persta->key_index);
        }
        else if(status == RX_SUCCESS)
        {
             g_decr_fail_cnt[index] = 0;
        }

        /* Invoke Ittiam Counter Measure if Continuous decryptn failures */
        /* are observed,to avoid any possible attacker attacking network */
        if(g_decr_fail_cnt[index] > DECR_FAIL_THR)
        {
             rsna_send_deauth((UWORD8 *)ae->persta, 0);
        }
    }
}

#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
