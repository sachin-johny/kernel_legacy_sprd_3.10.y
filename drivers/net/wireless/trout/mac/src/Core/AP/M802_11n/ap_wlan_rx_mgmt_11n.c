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
/*  File Name         : ap_wlan_rx_mgmt_11n.c                                */
/*                                                                           */
/*  Description       : This file contains the functions related to the MAC  */
/*                      management with 802.11n in AP mode of operation.     */
/*                                                                           */
/*  List of Functions : ap_enabled_rx_11n_action                             */
/*                      ap_enabled_rx_11n_control                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "receive.h"
#include "frame_11n.h"
#include "management_ap.h"
#include "prot_if.h"
#include "core_mode_if.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_11n_action                                 */
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

void ap_enabled_rx_11n_action(UWORD8 *msg)
{
    UWORD8       *data    = 0;
    wlan_rx_t    *wlan_rx = (wlan_rx_t*)msg;
    asoc_entry_t *ae      = (asoc_entry_t*)(wlan_rx->sa_entry);

    /* Pointer to the start of the data portion in the frame. */
    data = wlan_rx->msa + wlan_rx->hdr_len;

    if(PUBLIC_CATEGORY != data[CATEGORY_OFFSET])
    {
        /* An action frame is a Class 3 frame and can be processed only if */
        /* the station is associated except if it is a PUBLIC_CATEGORY     */

        /* If no association entry is found it indicates that the station is */
        /* not authenticated. In that case send a deauthentication frame to  */
        /* the station and return.                                           */
        if(ae == 0)
        {
            send_deauth_frame(wlan_rx->sa, (UWORD16)CLASS3_ERR);
            return;
        }

        /* If an association entry is found but the state is not associated  */
        /* send a disassociation frame to the station and return.            */
        if(ae->state != ASOC)
        {
            send_disasoc_frame(wlan_rx->sa, (UWORD16)CLASS3_ERR);
            return;
        }
    }
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
				TROUT_DBG4("%s: handle addba request!\n", __func__);
				handle_wlan_addba_req(wlan_rx->sa, data);
            }
            break;

            case ADDBA_RSP_TYPE:
            {
				TROUT_DBG4("%s: handle addba response!\n", __func__);
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
                /* or does not need to be handled in AP mode.                */
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
        case SM_POWER_SAVE_TYPE:
        {
            ht_struct_t *ht_hdl = &(ae->ht_hdl);

            /* Extract the SMPS mode from the SM Power Control field in the  */
            /* frame and update the association entry.                       */
            if(data[2] & BIT0)
            {
                if(data[2] & BIT1)
                {
                    ht_hdl->smps_mode = DYNAMIC_MODE;
                }
                else
                {
                    ht_hdl->smps_mode = STATIC_MODE;
                }
            }
            else
            {
                ht_hdl->smps_mode = MIMO_MODE;
            }
        }
        break;


        default:
        {
            /* Do nothing. The received action type is either not recognized */
            /* or does not need to be handled in AP mode.                    */
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
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_11n_control                                */
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

void ap_enabled_rx_11n_control(UWORD8 *msg)
{
    wlan_rx_t    *wlan_rx = (wlan_rx_t*)msg;
    asoc_entry_t *ae      = (asoc_entry_t *)wlan_rx->sa_entry;
    UWORD8       *msa     = wlan_rx->msa;

	TROUT_FUNC_ENTER;
    if(ae == NULL)
    {
    	TROUT_FUNC_EXIT;
        return;
    }

    switch(wlan_rx->sub_type)
    {
    case BLOCKACK_REQ:
    {
        UWORD8 tid          = (msa[17] & 0xF0) >> 4;
        ba_rx_struct_t **ba = &(ae->ba_hdl.ba_rx_ptr[tid]);
        UWORD16 seq_num     = get_bar_start_seq_num(msa);

#ifdef DEBUG_MODE
        g_mac_stats.babarrxd++;
#endif /* DEBUG_MODE */

        if((*ba) == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.babarrcvdigned++;
#endif /* DEBUG_MODE */
			TROUT_FUNC_EXIT;
            return;
        }

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
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */

