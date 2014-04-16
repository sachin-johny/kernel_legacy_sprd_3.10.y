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
/*  File Name         : ieee_11i_sta.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      802.11i MISC events.                                 */
/*                                                                           */
/*  List of Functions : misc_11i_event_sta                                   */
/*                      handle_11i_tx_comp_sta                               */
/*                      update_11i_fail_stats_sta                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "mac_init.h"
#include "maccontroller.h"
#include "management_sta.h"
#include "sta_prot_if.h"
#include "transmit.h"
#include "receive.h"
#include "ieee_11i_sta.h"
#include "qmu_if.h"
#include "tkip_sta.h"
#include "prot_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
pmkid_cache_t g_pmkid_cache = {0,};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_11i_event_sta                                       */
/*                                                                           */
/*  Description   : This function handles 802.11i STA events in ENABLED state*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle 802.11i events */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void misc_11i_event_sta(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
        case MISC_SUPP_SND_DEAUTH:
        {
            /* Load the RSNA Ptr to create the Send Auth message */
            rsna_supp_persta_t *rsna_ptr =
                                    (rsna_supp_persta_t *)misc_event_msg->data;

            /* If RSNA Entry is valid */
            if(rsna_ptr != NULL)
            {
                /* Deauth the RSNA Associated STA */
                if(mget_DesiredBSSType() == INFRASTRUCTURE)
                {
                    UWORD8      null_addr[6] = {0};
#ifdef DEBUG_MODE
                    g_reset_stats.rsnahsfailcnt++;
#endif /* DEBUG_MODE */
                    /* Send the de-authentication frame */
                    send_deauth_frame(mget_bssid(), rsna_ptr->deauth_reason);

                    /* Reset the STA Entry */
                    delete_entry(mget_bssid());

                    /* Reset the BSS ID so that no more deauth is sent */
                    mset_bssid(null_addr);

                    /* Restart MAC by raising a system error */
                    raise_system_error(RSNA_HS_FAIL);
                    return;
                }

                /* Remove the entry for the sta                             */
                /* This does the RSNA Cleanup along with that of Asoc table */
                delete_entry(rsna_ptr->addr);
            }
            else
            {
                /* Should not come here */
            }
        }
        break;

        /* This event is called in case of a security handshake time out */
        case MISC_TIMEOUT:
        {
            /* Return if the current MAC state does not match the state      */
            /* saved in the miscellaneous timeout event                      */
            if(get_mac_state() != misc_event_msg->info)
                return;

            if(mget_DesiredBSSType() == INFRASTRUCTURE)
            {
                UWORD8      null_addr[6] = {0};

#ifdef DEBUG_MODE
                g_reset_stats.rsnahstimeout++;
#endif /* DEBUG_MODE */

                /* Send De-authentication to AP indicating time out */
                send_deauth_frame(mget_bssid(), (UWORD16)HS_4W_TIMEOUT);

                /* Reset the STA Entry */
                delete_entry(mget_bssid());

                /* Reset the BSS ID so that no more deauth is sent */
                mset_bssid(null_addr);

               /* Restart the station by raising a system error */
                raise_system_error(RSNA_HS_FAIL);
            }
            else
            {
                /* TBD for IBSS Station */
            }
        }
        break;
        /* This Event is started to handle the TKIP Countermeasures */
        case MISC_SUPP_TKIP_CNT_MSR:
        {
            tkip_cntr_msr_supp(mac);
        }
        break;

        default:
        {
#ifdef IBSS_11I
            if(mget_DesiredBSSType() == INDEPENDENT)
            {
                misc_11i_event_auth(mac, msg);
            }
#endif /* IBSS_11I */
        }
        break;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_11i_tx_comp_sta                                   */
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
void handle_11i_tx_comp_sta(UWORD8 *dscr, sta_entry_t *se)
{
    UWORD8      *msa     = (UWORD8*)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);
#else
void handle_11i_tx_comp_sta(UWORD8 *dscr, sta_entry_t *se, UWORD8 *msa)
{
#endif
    CIPHER_T     cp      = (CIPHER_T) get_tx_dscr_cipher_type((UWORD32 *)dscr);

#ifdef IBSS_11I
    UWORD8      *da      = NULL;

    /* Get Receiver address */
    da = get_address1_ptr(msa);
#endif /* IBSS_11I */

    /* Continue anyfurther only if the cipher policy is AES or TKIP */
    if((cp == TKIP) || (cp == CCMP))
    {
        if(get_type(msa) == DATA_BASICTYPE)
        {
            /* Check if last sent Security Handshake packet was successfull */
            /* else take an appropriate action for this */
            if((BTRUE == is_this_sec_hs_pkt(msa,cp))  &&
               (get_tx_dscr_status((UWORD32 *)dscr) == TX_TIMEOUT))
            {
                if(se != NULL)
                {
#ifdef IBSS_11I
                    /* Terminate the connection with this station */
                    if(mget_DesiredBSSType() != INFRASTRUCTURE)
                    {
                        rsna_send_deauth((UWORD8 *)se->auth_persta, 0);
                    }
                    else
#endif /* IBSS_11I */
                    {
                        rsna_send_deauth((UWORD8 *)se->persta, 1);
                    }
                    return;
                }

            }
            /* This condition will occur only in a Independent network */
#ifdef IBSS_11I
            if((is_group(da) == BTRUE) &&
               (mget_DesiredBSSType() == INDEPENDENT))
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
                    if(cfg_val <= 65)
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
                        if(((temp[1] * 66) + (temp[0]/1000)) > cfg_val)
                        {
                            /* Reschedule Rekeying */
#ifndef OS_LINUX_CSL_TYPE
                            rekey_cb_fn(0, 0);
#else /* OS_LINUX_CSL_TYPE */
                            rekey_cb_fn(0);
#endif /* OS_LINUX_CSL_TYPE */
                        }
                    }
                }
                else
                {
                    if((temp[0] == 0xFFFF) && (temp[1] == 0xFFF0))
                    {
                        /* Reschedule Rekeying */
#ifndef OS_LINUX_CSL_TYPE
                        rekey_cb_fn(0, 0);
#else /* OS_LINUX_CSL_TYPE */
                        rekey_cb_fn(0);
#endif /* OS_LINUX_CSL_TYPE */
                    }
                }
            }
            else
#endif /* IBSS_11I */
            {
                UWORD32 temp[2] = {0};
                UWORD32 templ   = 0;
                UWORD32 temph   = 0;

                if(se == NULL)
                {
                    return;
                }

                /* Check the UCast PN Value for the TID */
                /* If it overflows, schedule deauth     */
                get_iv_2_pn_val(templ, temph, (UWORD8)cp, temp);

                if((temp[0] == 0xFFFF) && (temp[1] == 0xFFF0))
                {
                    /* Send Deauth */
#ifdef IBSS_11I
                    if(mget_DesiredBSSType() != INFRASTRUCTURE)
                    {
                        rsna_send_deauth((UWORD8 *)se->auth_persta, 0);
                    }
                    else
#endif /* IBSS_11I */
                    {
                        rsna_send_deauth((UWORD8 *)se->persta, 1);
                    }
                }
            }
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_11i_fail_stats_sta                                 */
/*                                                                           */
/*  Description   : This function updates the security failure statistics.   */
/*                                                                           */
/*  Inputs        : 1) Cipher type                                           */
/*                  2) Receive status                                        */
/*                  3) Source address                                        */
/*                  4) Destination address                                   */
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

void update_11i_fail_stats_sta(CIPHER_T ct, UWORD8 status, UWORD8 *sa,
                               UWORD8 *da)
{
    UWORD8 index = mget_RSNAStatsIndex(sa);
    sta_entry_t *se = (sta_entry_t*)find_entry(sa);

    if(index != 0xFF)
    {
        if(status == MIC_FAILURE)
        {
            if(ct == CCMP)
            {
                mincr_RSNAStatsCCMPDecryptErrors(index);
                g_decr_fail_cnt++;
            }
        }
        else if(status == ICV_FAILURE)
        {
            if(ct == TKIP)
            {
                mincr_RSNAStatsTKIPICVErrors(index);
                g_decr_fail_cnt++;
            }
        }
        else if(status == TKIP_MIC_FAILURE)
        {
            if(ct == TKIP)
            {
                BOOL_T is_broad_cast = is_group(da);

                if(NULL != se)
                {
                    /* Increment the MIB MIC Failure Count */
                    mincr_RSNAStatsTKIPLocalMICFailures(se->persta->key_index);

                    /* MIC Failure: Counter Measures invoked */
                    start_supp_cntr_msr_timer((void *)se->persta, is_broad_cast);
                }
            }
        }
        else if(status == TKIP_REPLAY_FAILURE)
        {
            if(NULL != se)
            {
                /* The PN Sequence as received is out of order */
                /* Replay error is logged                      */
                mincr_RSNAStatsTKIPReplays(se->persta->key_index);
            }
        }
        else if(status == CCMP_REPLAY_FAILURE)
        {
            if(NULL != se)
            {
                /* The PN Sequence as received is out of order */
                /* Replay error is logged                      */
                mincr_RSNAStatsCCMPReplays(se->persta->key_index);
            }
        }
        else if(status == RX_SUCCESS)
        {
            g_decr_fail_cnt = 0;
        }

        /* Invoke Ittiam Counter Measure if Continuous decryptn failures */
        /* are observed,to avoid any possible attacker attacking network */
        if(g_decr_fail_cnt > DECR_FAIL_THR)
        {
#ifdef DEBUG_MODE
           g_reset_stats.hackerexc++;
#endif /* DEBUG_MODE */
           /* Raise a system error which inturn restarts the MAC */
//-		   printk("update_11i_fail_stats_sta, status %d, ct %d\n", status, ct);
//-           raise_system_error(CUSTOM_CNTRMSR);
		   printk("[wjb]update_11i_fail_stats_sta, status %d, ct %d\n", status, ct);
           //raise_system_error(CUSTOM_CNTRMSR);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_pmkid_cache                                          */
/*                                                                           */
/*  Description   : This provides an interface to the configuration path to  */
/*                  update the PMKID cache.                                  */
/*                                                                           */
/*  Inputs        : 1) List of BSSID & PMKID pairs in the defined frame      */
/*                  format                                                   */
/*                                                                           */
/*  Globals       : g_pmkid_cache                                            */
/*                                                                           */
/*  Processing    : This function updates the PMKID cache with new entries   */
/*                  supplied from the configuration interface, The existing  */
/*                  entries in the PMKID cache is flushed when this          */
/*                  interface is accessed.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_pmkid_cache_sta(UWORD8 *val)
{
    UWORD8  i     = 0;
    UWORD16 index = 0;

    /*----------------------------------------------------------------------*/
    /*    NumEntries  | BSSID[1] |  PMKID[1]  | BSSID[2] |  PMKID[2]  |     */
    /*----------------------------------------------------------------------*/
    /*       1        |    6     |    16      |    6     |    16      |     */
    /*----------------------------------------------------------------------*/

    if(val == NULL)
    {
        return;
    }

    flush_pmkid_cache();

    g_pmkid_cache.num_elems = MIN(val[0], PMKID_CACHE_SIZE);
    index = 1;
    for(i = 0; i < g_pmkid_cache.num_elems; i++)
    {
        mac_addr_cpy(g_pmkid_cache.elem[i].addr, (val+index));
        index += MAC_ADDRESS_LEN;
        memcpy(g_pmkid_cache.elem[i].pmkid, (val+index), PMKID_LEN);
        index += PMKID_LEN;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_pmkid_cache                                          */
/*                                                                           */
/*  Description   : This provides an interface to the configuration path to  */
/*                  access the PMKID cache.                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_cfg_val                                                */
/*                  g_pmkid_cache                                            */
/*                                                                           */
/*  Processing    : This function reads the PMKID cache and returns the      */
/*                  existing entries in BSSID, PMKID pairs in the defined    */
/*                  frame format.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Pointer to array BSSID, PMKID pairs in the defined frame */
/*                  format                                                   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *get_pmkid_cache_sta(void)
{
    UWORD8  i     = 0;
    UWORD16 index = 0;

    /*----------------------------------------------------------------------*/
    /*    NumEntries  | BSSID[1] |  PMKID[1]  | BSSID[2] |  PMKID[2]  |     */
    /*----------------------------------------------------------------------*/
    /*       1        |    6     |    16      |    6     |    16      |     */
    /*----------------------------------------------------------------------*/

    g_cfg_val[1] = g_pmkid_cache.num_elems;
    index = 2;

    for(i = 0; i < g_pmkid_cache.num_elems; i++)
    {
        mac_addr_cpy((g_cfg_val+index), g_pmkid_cache.elem[i].addr);
        index += MAC_ADDRESS_LEN;
        memcpy((g_cfg_val+index), g_pmkid_cache.elem[i].pmkid, PMKID_LEN);
        index += PMKID_LEN;
    }

    /* Set length of the WID */
    g_cfg_val[0] = index - 1;

    return g_cfg_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : flush_pmkid_cache                                        */
/*                                                                           */
/*  Description   : This function flushes the existing PMKID Cache.          */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_pmkid_cache                                            */
/*                                                                           */
/*  Processing    : This function flushes the existing PMKID Cache.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void flush_pmkid_cache(void)
{
    g_pmkid_cache.num_elems = 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : search_pmkid_cache                                       */
/*                                                                           */
/*  Description   : This function searches the PMKID cache and returns the   */
/*                  PMKIDs which correspond to the passed BSSID.             */
/*                                                                           */
/*  Inputs        : 1) BSSID for which the PMKIDs are required               */
/*                  2) Buffer to return the PMKIDs                           */
/*                                                                           */
/*  Globals       : g_pmkid_cache                                            */
/*                                                                           */
/*  Processing    : The entire PMKID cache is searched all PMKIDs            */
/*                  corresponding to the passed BSSID are copied in the      */
/*                  buffer.                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Number of PMKID corresponding to the input BSSID         */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 search_pmkid_cache(UWORD8 *bssid, UWORD8 *pmkid)
{
    UWORD8 i = 0;
    UWORD8 index = 0;
    UWORD8 num_pmkid = 0;

    for(i = 0; i < g_pmkid_cache.num_elems; i++)
    {
        if(mac_addr_cmp(bssid, g_pmkid_cache.elem[i].addr) == BTRUE)
        {
            memcpy(pmkid + index, g_pmkid_cache.elem[i].pmkid, PMKID_LEN);
            index += PMKID_LEN;
            num_pmkid++;
        }
    }

    return num_pmkid;
}
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
