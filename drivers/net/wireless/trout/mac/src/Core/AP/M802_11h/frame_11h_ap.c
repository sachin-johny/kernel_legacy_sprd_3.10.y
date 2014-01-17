/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : frame_11h_ap.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to 802.11h        */
/*                      operation in AP mode.                                */
/*                                                                           */
/*  List of Functions : send_csa_frame                                       */
/*                      send_tpc_report_frame                                */
/*                      ap_enabled_11h_rx_action                             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "frame_11h_ap.h"
#include "dfs_ap.h"
#include "receive.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
UWORD8  g_country_ie_len                 = 8;
UWORD8  g_country_ie[MAX_COUNTRY_IE_LEN] = {ICOUNTRY, /* Country Info Elem */
                                            0x06, /* Length                */
                                            0x55, /* Country: USA          */
                                            0x53,
                                            0x20, /* Both Outdoor & Indoor */
                                            0x24, /* Starting Ch: 36       */
                                            0x0D, /* Total 13 Channels     */
                                            0x14};/* Max Power: 20 dBm     */
#endif

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void send_tpc_report_frame(UWORD8 *sa, UWORD8 dia_tok, UWORD32 *dscr);

// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : set_default_country_info_element                      */
/*                                                                           */
/*  Description      : This function sets a default country information      */
/*                     element at initialization.                            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_country_ie                                          */
/*                     g_country_ie_len                                      */
/*                                                                           */
/*  Processing       : This function prepares a default country information  */
/*                     element at the time of initialization. This can be    */
/*                     overwritten by setting the WID_COUNTRY_IE by the user */
/*                     based on the configuration desired.                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void set_default_country_info_element(void)
{
#if 0 /* MD-TBD: Set with the correct Country element */
    UWORD8 i = 0;

    /*  Country Information Element Format                                   */
    /* --------------------------------------------------------------------- */
    /* | Element ID | Length | Country String: Octet 1 | Octet 2 | Octet 3 | */
    /* --------------------------------------------------------------------- */
    /* | 1          | 1      | 1                       | 1       | 1       | */
    /* --------------------------------------------------------------------- */
    /* | First Channel Number| Number of Channels | Maximum Tx Power Level | */
    /* --------------------------------------------------------------------- */
    /* | 1                   | 1                  | 1                      | */
    /* --------------------------------------------------------------------- */
    /* | ............................................... | Pad (if needed) | */
    /* --------------------------------------------------------------------- */
    /* | ............................................... | 1               | */
    /* --------------------------------------------------------------------- */

    g_country_ie_len = 0;

    /* Element ID */
    g_country_ie[i++] = ICOUNTRY;

    /* Length */
    g_country_ie[i++] = g_country_ie_len - 2;

    /* Country string */
    g_country_ie[i++] = 0;
#endif
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_tpc_report_frame                                 */
/*                                                                           */
/*  Description      : This function sends a TPC Report Action frame.        */
/*                                                                           */
/*  Inputs           : 1) Source address of the TPC request sender           */
/*                     2) Dialog token of the TPC request frame              */
/*                     3) Pointer to receive descriptor of the TPC request   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function prepares a TPC Report Action frame for  */
/*                     the received request and transmits the same.          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void send_tpc_report_frame(UWORD8 *sa, UWORD8 dialog_token, UWORD32 *rx_dscr)
{
    WORD8   rx_pow      = 0;
    WORD8   link_margin = 0;
    UWORD16 index       = 0;
    UWORD8  *data       = NULL;

    data = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);

    if(data == NULL)
    {
        return;
    }

    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)ACTION);

    set_address1(data, sa);
    set_address2(data, mget_StationID());
    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /* TPC Report - Frame body                                               */
    /* --------------------------------------------------------------------- */
    /* | Category | Action | Dialog Token | TPC Report IE                  | */
    /* --------------------------------------------------------------------- */
    /* | 1        | 1      | 1            | 4                              | */
    /* --------------------------------------------------------------------- */

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Category field of action frame */
    data[index++] = SPECMGMT_CATEGORY;

    /* Action field of action frame */
    data[index++] = TPC_REPORT_TYPE;

    /* Dialog token of action frame */
    data[index++] = dialog_token;


    /* Extract the power from the received TPC request frame and calculate   */
    /* the link margin using this. This link margin value is included in the */
    /* TPC report                                                            */
    rx_pow      = get_rx_dscr_rssi(rx_dscr);
    link_margin = rx_pow - SENSITIVITY;

    /* Update the TPC Report element */
    set_tpc_report_info_field(data, index, link_margin);
    index += (IE_HDR_LEN + ITPCREPORT_LEN);

    /* Transmit the TPC Report Action frame */
    tx_mgmt_frame(data, index + FCS_LEN, HIGH_PRI_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : ap_enabled_11h_rx_action                              */
/*                                                                           */
/*  Description      : This function handles the incoming action frame as    */
/*                     appropriate in the ENABLED state for AP mode.         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the incoming message                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The incoming action category and type are checked and */
/*                     appropriate processing is done.                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void ap_enabled_11h_rx_action(UWORD8 *msg)
{
    wlan_rx_t    *wlan_rx = (wlan_rx_t*)msg;
    UWORD8       *msa     = wlan_rx->msa;
    UWORD8       *data    = wlan_rx->msa + MAC_HDR_LEN;
    asoc_entry_t *ae      = (asoc_entry_t*)(wlan_rx->sa_entry);

    /* An action frame is a Class 3 frame and can be processed only if the   */
    /* station is associated.                                                */

    /* If no association entry is found it indicates that the station is not */
    /* authenticated. In that case send a deauthentication frame to the      */
    /* station and return.                                                   */
    if(ae == 0)
    {
        UWORD8 sa[6]    = {0};
        get_SA(msa, sa);
        TROUT_DBG5("%s: rcved frame frome unknown sta!\n", __func__);
        send_deauth_frame(sa, (UWORD16)CLASS3_ERR);
        return;
    }

    /* If an association entry is found but the state is not associated send */
    /* a disassociation frame to the station and return.                     */
    if(ae->state != ASOC)
    {
        UWORD8 sa[6]    = {0};
        get_SA(msa, sa);
        send_disasoc_frame(sa, (UWORD16)CLASS3_ERR);
        return;
    }

    /* Process the action frame based on the category */
    switch(data[CATEGORY_OFFSET])
    {
    case SPECMGMT_CATEGORY:
    {
        UWORD8 dia_tok = 0;
        UWORD8 sa[6]   = {0};

        dia_tok = get_dialog_token(msa);
        get_SA(msa, sa);

        /* Process the QOS action frame based on the action type*/
        switch(data[ACTION_OFFSET])
        {
        case MEASUREMENT_REQUEST_TYPE:
        {
            /* 11H TBD: It is not very clear if AP can receive measurement   */
            /* request and in case it does, if it requires to honor the same */
            /* Currently the measurement request is dropped and not handled. */
        }
        break;
        case MEASUREMENT_REPORT_TYPE:
        {
            /* 11H TBD: Currently the AP does not initiate any measurement   */
            /* request and hence no processing is required for measurement   */
            /* report action frames.                                         */
			
			// 20120709 caisf add, merged ittiam mac v1.2 code
            /* Note : Autonomous report from the STA is not handled. This    */
            /* is done for following reasons                                 */
            /*  i. For security purpose, in order to prevent unnecessary     */
            /*     channel switches in DFS mode. Other vendors like Cisco,   */
            /*     buffalo also do not support this feature.                 */
            /* ii. This is not tested as part for 802.11H certification.     */
        }
        break;
        case TPC_REQUEST_TYPE:
        {
            /* 11H TBD: It is not very clear if AP can receive TPC request   */
            /* and in case it does, if it requires to honor the same.        */
            /* Currently it prepared the TPC report and sends it.            */
            UWORD32 *rx_dscr = (UWORD32 *)wlan_rx->base_dscr;

            PRINTD2("TPC Request\n");

            if(mget_enableTPC() == BTRUE)
            {
                send_tpc_report_frame(sa, dia_tok, rx_dscr);
            }
        }
        break;
        case TPC_REPORT_TYPE:
        {
            /* 11H TBD: Currently the AP does not initiate any TPC request   */
            /* and hence no processing is required for TPC report action     */
            /* frames.                                                       */
        }
        break;
        default:
        {
            /* Do nothing. The received action type is unknown or does not   */
            /* require any processing in AP mode.                            */
        }
        break;
        }
    }
    break;
    default:
    {
        /* Do nothing. The received action category is unknown. */
    }
    break;
    }
}

#endif /* MAC_802_11H */
#endif /* BSS_ACCESS_POINT_MODE */
