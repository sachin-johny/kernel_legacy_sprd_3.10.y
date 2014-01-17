/*****************************************************************************/
/*                                                                           */
/*              Ittiam 802.11 MAC HARDWARE UNIT TEST CODE                    */
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
/*  File Name         : mh_test_utils.c                                      */
/*                                                                           */
/*  Description       : This file contains all the utility functions for H/w */
/*                      unit test.                                           */
/*                                                                           */
/*  List of Functions : prepare_beacon                                       */
/*                      prepare_tx_frame                                     */
/*                      init_rx_buff                                         */
/*                      replenish_test_rx_queue                              */
/*                      compute_crc32                                        */
/*                      create_amsdu                                         */
/*                      search_and_update_tuple_cache                        */
/*                      clear_tuple_cache                                    */
/*                      clear_tuple_cache_entry                              */
/*                      check_hut_frame                                      */
/*                      get_hut_tx_rate                                      */
/*                      get_hut_phy_tx_mode                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mh_test.h"
#include "mh_test_utils.h"
#include "iconfig.h"
#include "receive.h"

#ifdef MAC_802_11I
#include "tkip.h"
#endif /* MAC_802_11I */

/*****************************************************************************/
/* Static Functions                                                          */
/*****************************************************************************/

static void clear_tuple_cache_entry(UWORD8 index);

/*****************************************************************************/
/* Static Global Varible Declarations                                        */
/*****************************************************************************/

static tuple_cache_t g_tuple_cache[TUPLECACHESIZE];

/* Signal quality related global variables */
static UWORD16 g_sig_qual_count = 0;
static UWORD32 g_sig_qual_sum   = 0;

/* RSSI related global variables */
static WORD32  g_rssi_ex_sum       = 0;
static WORD32  g_adj_rssi_ex_sum   = 0;
static UWORD16 g_rssi_ex_sum_count = 0;

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_txq_pri_to_num_hut[17] = {0};

/* CRC Computation table */
UWORD32 g_fcstab_32[256] = {
      0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
      0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
      0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
      0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
      0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
      0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
      0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
      0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
      0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
      0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
      0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
      0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
      0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
      0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
      0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
      0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
      0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
      0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
      0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
      0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
      0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
      0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
      0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
      0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
      0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
      0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
      0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
      0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
      0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
      0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
      0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
      0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
      0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
      0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
      0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
      0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
      0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
      0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
      0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
      0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
      0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
      0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
      0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
      0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
      0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
      0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
      0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
      0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
      0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
      0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
      0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
      0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
      0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
      0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
      0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
      0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
      0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
      0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
      0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
      0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
      0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
      0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
      0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
      0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_beacon                                           */
/*                                                                           */
/*  Description   : This function prepares a dummy beacon for test.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to beacon buffer                              */
/*                  2) Flag                                                  */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function prepares a dummy beacon of required length */
/*                  and predefined field values.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of the beacon frame.                              */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_beacon(UWORD8 *beacon, UWORD8 flag)
{
    UWORD32 i   = 0;
    UWORD32 idx = 0;

    beacon[0] = 0x80; /* TypeSubtype */
    beacon[1] = 0x00; /* Frame Cotrol */
    beacon[2] = 0x00; /* Duration ID */
    beacon[3] = 0x00;
    memcpy(&beacon[4],  mget_bcst_addr(), 6); /* Address 1, Destination */
    memcpy(&beacon[10], mget_StationID(), 6); /* Address 2, Source */
    memcpy(&beacon[16], mget_bssid(),     6); /* Address 3, BSSID */

    idx = 22;
    beacon[idx++] = 0x00; /* Sequence Control */
    beacon[idx++] = 0x00;
    beacon[idx++] = 0x12;
    beacon[idx++] = 0x13;
    beacon[idx++] = 0x14;
    beacon[idx++] = 0x15;
    beacon[idx++] = 0x16;
    beacon[idx++] = 0x17;
    beacon[idx++] = 0x18;
    beacon[idx++] = 0x19;
    beacon[idx++] = (mget_BeaconPeriod() & 0x00FF); /* Beacon Period */
    beacon[idx++] = ((mget_BeaconPeriod() & 0xFF00) >> 8);
    beacon[idx++] = 0x02;
    beacon[idx++] = 0x00;
    if(flag == 0)
    {
        beacon[idx++] = 0x00; /* SSID */
        beacon[idx++] = 0x09; /* SSID length */
        beacon[idx++] = 'I';  /* SSID start */
        beacon[idx++] = 'M';
        beacon[idx++] = 'W';
        beacon[idx++] = 'L';
        beacon[idx++] = 'A';
        beacon[idx++] = 'N';
        beacon[idx++] = 'M';
        beacon[idx++] = 'A';
        beacon[idx++] = 'C';  /* SSID end */
    }
    else
    {
        beacon[idx++] = 0x00; /* SSID */
        beacon[idx++] = 0x09; /* SSID length */
        beacon[idx++] = 'T';  /* SSID start */
        beacon[idx++] = 'E';
        beacon[idx++] = 'S';
        beacon[idx++] = 'T';
        beacon[idx++] = 'I';
        beacon[idx++] = 'N';
        beacon[idx++] = 'G';
        beacon[idx++] = '-';
        beacon[idx++] = '1';  /* SSID end */
    }
    beacon[idx++] = 0x01; /* Supported Rates */
    beacon[idx++] = 0x08; /* Supported Rates Length */
    beacon[idx++] = 0x82; /* 1Mbps   */
    beacon[idx++] = 0x84; /* 2Mbps   */
    beacon[idx++] = 0x8B; /* 5.5Mbps */
    beacon[idx++] = 0x96; /* 11Mbps  */
    beacon[idx++] = 0x0C;
    beacon[idx++] = 0x18;
    beacon[idx++] = 0x30;
    beacon[idx++] = 0x48;
    beacon[idx++] = 0x03;
    beacon[idx++] = 0x01;
    beacon[idx++] = (mget_CurrentChannel() + 1);

    beacon[idx++] = 0x06;
    beacon[idx++] = 0x02;
    beacon[idx++] = 0x00;
    beacon[idx++] = 0x00;
    beacon[idx++] = 0x32;
    beacon[idx++] = 0x04;
    beacon[idx++] = 0x12;
    beacon[idx++] = 0x24;
    beacon[idx++] = 0x60;
    beacon[idx++] = 0x6C;

    /* Fill up rest of the buffer */
    for(i = idx; i < g_test_config.tsf.bcn_len; i++)
        beacon[i] = i;

    return i;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_tx_frame                                         */
/*                                                                           */
/*  Description   : This function prepares a frame for transmitting.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to Packet buffer                              */
/*                  2) Queue number of the packet                            */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of the frame                                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_tx_frame(UWORD8 *mac_hdr, UWORD8 q_num, BOOL_T is_bcst)
{
    UWORD16  i             = 0;
    UWORD16  mac_hdr_len   = MAC_HDR_LEN;
    q_info_t *temp         = NULL;
    UWORD8   sec_trail_len = 0;
    UWORD8   tid           = 0;
    UWORD16  p_idx         = 0;

    set_frame_control(mac_hdr, g_test_config.tx.frame_type);

#ifdef MAC_WMM
    switch(q_num)
    {
    case HIGH_PRI_Q:
    {
        tid = PRIORITY_3;
        temp = &(g_test_config.tx.hp);
    }
    break;

    case AC_BK_Q:
    {
        tid = PRIORITY_0;
        temp = &(g_test_config.tx.ac_bk);
    }
    break;

    case AC_BE_Q:
    {
        tid = PRIORITY_1;
        temp = &(g_test_config.tx.ac_be);
    }
    break;

    case AC_VI_Q:
    {
        tid = PRIORITY_2;
        temp = &(g_test_config.tx.ac_vi);
    }
    break;

    case AC_VO_Q:
    {
        tid = PRIORITY_3;
        temp = &(g_test_config.tx.ac_vo);
    }
    break;

    default:
    break;
    }

    {
        UWORD8 ack_pol = NORMAL_ACK;

#ifdef MAC_802_11N
        switch(temp->ack_policy)
        {
            case NO_ACK:
            case BCAST_NO_ACK:
            {
                ack_pol = NO_ACK;
                break;
            }
            case BLOCK_ACK:
            case COMP_BLOCK_ACK:
            {
                ack_pol = BLOCK_ACK;
                break;
            }
            case NO_EXPLICIT_ACK:
            {
                ack_pol = BLOCK_ACK;
                break;
            }
            default:
                ack_pol = NORMAL_ACK;
        }
#endif /* MAC_802_11N */

        if(is_bcst == BTRUE)
            ack_pol = NO_ACK;

        mac_hdr_len = set_mac_hdr_11e(mac_hdr, tid, ack_pol);
    }
#else /* MAC_WMM */
    switch(q_num)
    {
    case HIGH_PRI_Q:
    {
        temp = &(g_test_config.tx.hp);
    }
    break;

    case NORMAL_PRI_Q:
    {
        temp = &(g_test_config.tx.nor);
    }
    break;

    case CF_PRI_Q:
    {
        temp = &(g_test_config.tx.cf);
    }
    break;

    default:
    break;
    }
#endif /* MAC_WMM */

#ifdef MAC_802_11I
    if(g_test_config.ce.ce_enable == 1)
    {
         set_wep(mac_hdr, 1);
    }
#endif /* MAC_802_11I */

    /* Set the Address fields for based on the mode of STA operation */
    if(mget_DesiredBSSType() == 2)
    {
        set_to_ds(mac_hdr, 0);

        /* Set Address1 field in the WLAN Header with destination address */
        if(is_bcst == BTRUE)
            set_address1(mac_hdr, mget_bcst_addr());
        else
            set_address1(mac_hdr, g_test_config.tx.da);

        /* Set Address2 field in the WLAN Header with the source address */
        set_address2(mac_hdr, mget_StationID());

        /* Set Address3 field in the WLAN Header with the BSSID */
        set_address3(mac_hdr, mget_bssid());
    }
    else if (mget_DesiredBSSType() == 1) /* INFRASTRUCTURE */
    {
        set_to_ds(mac_hdr, 1);

        /* Set Address1 field in the WLAN Header with the BSSID */
        set_address1(mac_hdr, mget_bssid());

        /* Set Address2 field in the WLAN Header with the source address */
        set_address2(mac_hdr, mget_StationID());

        /* Set Address3 field in the WLAN Header with destination address */
        set_address3(mac_hdr, g_test_config.tx.da);
    }
    else /* AP */
    {
        set_from_ds(mac_hdr, 1);

        /* Set Address1 field in the WLAN Header with the BSSID */
        set_address3(mac_hdr, mget_bssid());

        /* Set Address2 field in the WLAN Header with the source address */
        set_address2(mac_hdr, mget_StationID());

        /* Set Address3 field in the WLAN Header with destination address */
        if(is_bcst == BTRUE)
            set_address1(mac_hdr, mget_bcst_addr());
        else
            set_address1(mac_hdr, g_test_config.tx.da);
    }

#ifdef MAC_802_11I
    if(g_test_config.ce.ce_enable == 1 && g_test_config.ce.ct == TKIP)
        sec_trail_len = 8;
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    /* Set HT-Control Frame Header Field */
    if(g_test_config.ht.tx_htc_enable == 1)
        mac_hdr_len += set_ht_control(mac_hdr, mac_hdr_len);

    if(g_test_config.ht.amsdu_mode)
    {
        UWORD8 *da = NULL;

        da = (is_bcst == BTRUE) ? mget_bcst_addr() : g_test_config.tx.da;

        /* With AMSDU enabled, the frame length is upper bounded by the */
        /* max-AMSDU length                                             */
        if(temp->frame_len > g_test_config.ht.amsdu_maxlen)
            temp->frame_len = g_test_config.ht.amsdu_maxlen;

        /* Set the AMSDU present field in the MAC header */
        advt_amsdu_frame(mac_hdr);

        /* Prepare the AMSDU frame */
        temp->num_submsdu = create_amsdu(mac_hdr + mac_hdr_len,
              temp->frame_len - mac_hdr_len - sec_trail_len - FCS_LEN,
              g_test_config.ht.amsdu_maxnum, mget_StationID(), da);

        /* No Frame length modification is done for AMSDU */
        temp->min_frame_len = temp->frame_len;
        temp->cur_frame_len = temp->frame_len;
    }
    else
    {
        for(i = mac_hdr_len; i < (temp->frame_len - sec_trail_len - FCS_LEN); i++)
        {
            mac_hdr[i] = g_tx_pattern[p_idx++];

            if(p_idx >= g_tx_pattern_len)
                p_idx = 0;
        }

        temp->num_submsdu = 1;

        /* The smallest frame contains 1 data byte */
        temp->min_frame_len = mac_hdr_len + sec_trail_len + FCS_LEN + 1;
        temp->cur_frame_len = temp->frame_len;
    }
#else /* MAC_802_11N */
    for(i = mac_hdr_len; i < (temp->frame_len - sec_trail_len - FCS_LEN); i++)
    {
        mac_hdr[i] = g_tx_pattern[p_idx++];

        if(p_idx >= g_tx_pattern_len)
            p_idx = 0;
    }

    temp->num_submsdu = 1;

    /* The smallest frame contains 1 data byte */
    temp->min_frame_len = mac_hdr_len + sec_trail_len + FCS_LEN + 1;
    temp->cur_frame_len = temp->frame_len;

#endif /* MAC_802_11N */

    return temp->frame_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_rx_buff                                             */
/*                                                                           */
/*  Description   : This function initializes the receive buffer link list.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_q_handle, g_test_config                                */
/*                                                                           */
/*  Processing    : This function initializes the receive buffer link list.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_rx_buff(void)
{
    /* Reset the Q-Manager */
    qmu_close(&g_q_handle);

    /* Initialize the Q-manager */
    qmu_init(&g_q_handle, get_no_tx_queues());

    /* Add the required Number of buffers to the High Priority Rx-Queue */
    replenish_test_rx_queue(HIGH_PRI_RXQ);

    /* Add the required Number of buffers to the Normal Priority Rx-Queue */
    replenish_test_rx_queue(NORMAL_PRI_RXQ);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : replenish_test_rx_queue                                  */
/*                                                                           */
/*  Description   : This function adds buffers to the receive queue to       */
/*                  maintain the size to the desired level.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : g_q_handle, g_test_config                                */
/*                                                                           */
/*  Processing    : This function adds buffers to the receive queue to       */
/*                  maintain the size to the desired level.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void replenish_test_rx_queue(UWORD8 q_num)
{
    UWORD16 curr_num_bufs = g_q_handle.rx_handle.rx_header[q_num].element_cnt;

    /* If Rx Q replenish is disabled, return without doing anything */
    if(get_hut_disable_rxq_repl() == 1)
        return;

    while(curr_num_bufs++ < g_test_config.rx.num_rx_buffers[q_num])
    {
        if(qmu_add_rx_buffer(&(g_q_handle.rx_handle), q_num) != QMU_OK)
        {
            /* Free Buffers currently not available */
            return;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : compute_crc32                                            */
/*                                                                           */
/*  Description   : This function computes CRC32.                            */
/*                                                                           */
/*  Inputs        : 1) 32 bit FCS value                                      */
/*                  2) Pointer to data                                       */
/*                  3) Length of the data                                    */
/*                                                                           */
/*  Globals       : g_fcstab_32                                              */
/*                                                                           */
/*  Processing    : This function computes CRC32 using LUT.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : 32 bit FCS value.                                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 compute_crc32(UWORD32 fcs, UWORD8* data, UWORD16 len)
{
    while (len--)
        fcs = (((fcs) >> 8) ^ g_fcstab_32[((fcs) ^ (*data++)) & 0xff]);

       return (fcs);
}

#ifdef MAC_802_11N
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : create_amsdu                                          */
/*                                                                           */
/*  Description      : This function creates an A-MSDU for unit test.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to a buffer                                */
/*                     2) Maximum size of the AMSDU                          */
/*                     3) Maximum number of sub-MSDU                         */
/*                     4) Source address                                     */
/*                     5) Destination address                                */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The A-MSDU frame is created in required format.       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD16, Length of the A-MSDU frame                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 create_amsdu(UWORD8 *buff, UWORD16 max_size, UWORD16 max_num,
                     UWORD8 *sa, UWORD8 *da)
{
    UWORD16 num_sub_msdu      = 0;
    UWORD16 sub_msdu_len      = 0;
    UWORD16 offset            = 0;
    UWORD16 sub_msdu_datalen  = 0;
    UWORD16 last_sub_msdu_len = 0;
    UWORD16 indx              = 0;
    UWORD16 i                 = 0;
    UWORD16 p_idx             = 0;

    sub_msdu_len = max_size / max_num;

    /* The sub-MSDU length should be a multiple of 4 for all sub-MSDUs except */
    /* the last one.                                                          */
    sub_msdu_len -= sub_msdu_len & 3;

    if(sub_msdu_len < MIN_SUB_MSDU_SIZE)
        sub_msdu_len = MIN_SUB_MSDU_SIZE;

    /* Compute the length of the data portion within the sub-MSDU */
    sub_msdu_datalen = sub_msdu_len - SUB_MSDU_HEADER_LENGTH;

    /* Calculate the number of sub-MSDUs */
    num_sub_msdu = max_size / sub_msdu_len;

    /* Calculate the size of the last sub-MSDU */
    last_sub_msdu_len = max_size - (num_sub_msdu - 1) * sub_msdu_len;

    /* Prepare the sub-MSDUs */
    for(indx = 0; indx < num_sub_msdu - 1; indx++)
    {
        UWORD16 var_sub_msdu_datalen = sub_msdu_datalen;

        /* The length of the sub-MSDUs is varied around the nominal sub-MSDU  */
        /* length. However, when padding is applied, the length becomes equal */
        /* to the calculated sub-MSDU length.                                 */
        var_sub_msdu_datalen = sub_msdu_datalen - (indx & 0x3);

        /* Create the sub-MSDU header */
        set_submsdu_header(buff + offset, sa, da, var_sub_msdu_datalen);
        offset += SUB_MSDU_HEADER_LENGTH;
        p_idx   = 0;

        /* Create the body of the sub-MSDU */
        for(i = 0; i < var_sub_msdu_datalen; i++)
        {
            *(buff + offset + i) = g_tx_pattern[p_idx++];

            if(p_idx >= g_tx_pattern_len)
                p_idx = 0;
        }
        /* Set the sub-MSDU padding bytes. */
        set_submsdu_padding(buff + offset + var_sub_msdu_datalen,
                                  var_sub_msdu_datalen + SUB_MSDU_HEADER_LENGTH);
        offset += sub_msdu_datalen;

        if(g_test_config.ht.amsdu_mode == 2)
        {
#ifdef MAC_802_11I
            /* For TKIP the statergy of inserting Sub-MSDU spacing corrupts */
            /* MIC computation.                                             */
        if(g_test_config.ce.ct != TKIP)
#endif /* MAC_802_11I */
            offset += SUBMSDU_SPACING;
    }
    }

    /* Prepare the last sub-MSDU */
    set_submsdu_header(buff + offset, sa, da,
                       last_sub_msdu_len - SUB_MSDU_HEADER_LENGTH);
    offset += SUB_MSDU_HEADER_LENGTH;
    p_idx = 0;

    for(i = 0; i < (last_sub_msdu_len - SUB_MSDU_HEADER_LENGTH); i++)
    {
        *(buff + offset + i) = g_tx_pattern[p_idx++];

        if(p_idx >= g_tx_pattern_len)
            p_idx = 0;
    }

    return num_sub_msdu;
}

#endif /* MAC_802_11N */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : search_and_update_tuple_cache                         */
/*                                                                           */
/*  Description      : This function searches the tuple cache for duplicate  */
/*                     frame entries. In case no duplicate is detected it    */
/*                     updates the tuple cache entry with the details of the */
/*                     incoming frame.                                       */
/*                                                                           */
/*  Inputs           : 1) Source address                                     */
/*                     2) Sequence number                                    */
/*                     3) Fragment number                                    */
/*                     4) Retry Bit                                          */
/*                                                                           */
/*  Globals          : g_tuple_cache                                         */
/*                                                                           */
/*  Processing       : The entire tuple cache is searched for an entry       */
/*                     corresponding to the sending station. If it is not    */
/*                     found a new entry is added (provided that the tuple   */
/*                     cache is not full). Otherwise, if the incoming frame  */
/*                     is a retransmitted one the entry is checked to detect */
/*                     the frame as a duplicate. If a duplicate is not       */
/*                     detected, the entry is updated with the sequence      */
/*                     number and fragment number of the incoming frame.     */
/*                                                                           */
/*  Outputs          : Updates the tuple cache entry if found or adds a new  */
/*                     tuple cache entry.                                    */
/*                                                                           */
/*  Returns          : Boolean. BTRUE, if a duplicate entry is found.        */
/*                     BFALSE, otherwise.                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T search_and_update_tuple_cache(UWORD8* addr, UWORD16 seq_num,
                                     UWORD8 frag_num, UWORD8 retry)
{
    UWORD8 i       = 0;
    UWORD8 index   = 0;
    BOOL_T ret_val = BFALSE;
    BOOL_T found   = BFALSE;

    /* Search the entire tuple cache for an entry corresponding to the       */
    /* sending station.                                                      */
    for(i = 0; i < TUPLECACHESIZE; i++)
    {
        if(g_tuple_cache[i].in_use == BTRUE)
        {
            /* If the tuple cache entry is in use, check if the address in   */
            /* the entry matches the source address. If it matches break     */
            /* from the loop as a corresponding entry has been found.        */
            if(mac_addr_cmp(g_tuple_cache[i].addr, addr))
            {
                break;
            }
            else
            {
                /* If the address does not match, the entry is for another   */
                /* station. Do nothing                                       */
            }
        }
        else if(found == BFALSE)
        {
            /* If the tuple cache entry is not in use, the index for the     */
            /* cache entry is saved. This is used in case no matching entry  */
            /* is found. A flag 'found' is used to indicate that an empty    */
            /* entry has already been found. In that case the index is not   */
            /* updated. In the case that the tuple cache is full, this flag  */
            /* will have a value of BFALSE.                                  */
            index = i;
            found = BTRUE;
        }
    }

    /* If the counter 'i' reaches the TUPLECACHESIZE, it indicates that the  */
    /* entire tuple cache was searched and no entry was found.               */
    if(i == TUPLECACHESIZE)
    {
        if(found == BTRUE)
        {
            /* Add a new entry <address, seq num, frag num> for this station,*/
            /* only if the flag 'found' has a value of BTRUE (indicating the */
            /* tuple cache is not full yet.                                  */
            g_tuple_cache[index].in_use   = BTRUE;
            g_tuple_cache[index].seq_num  = seq_num;
            g_tuple_cache[index].frag_num = frag_num;
            memcpy(g_tuple_cache[index].addr, addr, 6);
        }
        else /* found = BFALSE */
        {
            /* The tuple cache is full and the incoming entry is ignored.    */
            /* Note that duplicate detection can be done for only            */
            /* TUPLECACHESIZE (32) stations at a time. Aging algorithm needs */
            /* to be implemented to update the cache by removing least       */
            /* recently used entry.                                          */
        }
    }
    /* The value of the counter 'i' gives the index of the required entry in */
    /* the tuple cache. The entry is updated if it is not a duplicate.       */
    else
    {
        /* Duplicate detection is done only for retransmitted frames. The    */
        /* frame is a duplicate if the sequence number and the fragment      */
        /* number match exactly.                                             */
        if((retry == 1) && (g_tuple_cache[i].seq_num == seq_num) &&
           (g_tuple_cache[i].frag_num == frag_num))
        {
            ret_val = BTRUE;
        }
        /* In the case that the frame is not a duplicate, the entry is       */
        /* updated with the sequence number and fragment number of the       */
        /* latest incoming frame.                                            */
        else
        {
            g_tuple_cache[i].seq_num  = seq_num;
            g_tuple_cache[i].frag_num = frag_num;
        }
    }

    /* The return value indicates if the incoming frame was a duplicate.     */
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : clear_tuple_cache                                     */
/*                                                                           */
/*  Description      : This function clears all the tuple cache entries.     */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : All the cache entries are cleared.                    */
/*                                                                           */
/*  Outputs          : The global tuple cache is cleared.                    */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void clear_tuple_cache(void)
{
    UWORD32 i = 0;

    for(i = 0; i < TUPLECACHESIZE; i++)
    {
        clear_tuple_cache_entry(i);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : clear_tuple_cache_entry                               */
/*                                                                           */
/*  Description      : This function clears the specified tuple cache entry. */
/*                                                                           */
/*  Inputs           : 1) Index of the tuple cache entry                     */
/*                                                                           */
/*  Globals          : g_tuple_cache                                         */
/*                                                                           */
/*  Processing       : The given cache entry is cleared.                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void clear_tuple_cache_entry(UWORD8 index)
{
    if(index < TUPLECACHESIZE)
    {
        mem_set(g_tuple_cache[index].addr, 0, 6);
        g_tuple_cache[index].seq_num  = 0;
        g_tuple_cache[index].frag_num = 0;
        g_tuple_cache[index].in_use   = BFALSE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_hut_frame                                       */
/*                                                                           */
/*  Description      : This function enqueues the frame received from the    */
/*                     host for transmission when the MHUT code is operating */
/*                     in  Test Support mode.                                */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Host Receive event entry.           */
/*                                                                           */
/*  Globals          : g_test_config                                         */
/*                                                                           */
/*  Processing       : If the operating mode is Test-Support and the frame   */
/*                     has the expected UDP Source and Destination Port      */
/*                     numbers, then the frame is enqueued for transmission. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : BTRUE  if Frame enqueued for transmission.            */
/*                     BFALSE otherwise                                      */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_hut_frame(UWORD8* event)
{
#ifdef ETHERNET_HOST
    UWORD8        idx           = 0;
    UWORD8        q_num         = 0;
    UWORD8        pream         = 0;
    UWORD8        tx_rate       = 0;
    UWORD8        *tx_dscr      = 0;
    UWORD8        *frm_header   = NULL;
    UWORD8        mac_hdr_len   = 0;
    UWORD8        *buffer_addr  = 0;
    UWORD8        dest_addr[6]  = {0};
    UWORD8        priority      = 0;
    UWORD8        ack_policy    = BCAST_NO_ACK;
    UWORD8        buffer_ofst   = 0;
    UWORD8        host_type     = 0;
    UWORD16       offset        = 0;
    UWORD32       buffer_len    = 0;
    UWORD32       phy_tx_mode   = 0;
    UWORD32       frame_attrib  = 0;
    buffer_desc_t buffer_desc   = {0};
    BASICTYPE_T   frame_type    = DATA_BASICTYPE;
    TYPESUBTYPE_T frame_subtype = DATA;
    UWORD32       retry_set[2] = {0};


    /* Check whether operating Mode is Frame-Injector */
    if(g_test_config.oper_mode != 1)
    {
        return BFALSE;
    }

    buffer_addr = ((host_rx_event_msg_t *)event)->buffer_addr;
    buffer_len  = ((host_rx_event_msg_t *)event)->rx_pkt_len;
    buffer_ofst = ((host_rx_event_msg_t *)event)->pkt_ofst;
    host_type   = ((host_rx_event_msg_t *)event)->host_if_type;

     /* MHSS : HUT MODE TBD */

    if(host_type != ETHERNET_HOST_TYPE)
    {
        return BFALSE;
    }

    offset = get_eth_hdr_offset(host_type);

    /* Check the Src & Dst UDP Port Numbers to determine whether this frame */
    /* should be transmitted on air.                                        */
#ifdef ETHERNET_HOST
    {
        UWORD8   *eth_hdr     = 0;
        UWORD8   ip_protocol  = 0;
        UWORD16  eth_type     = 0;
        UWORD16  udp_src_port = 0;
        UWORD16  udp_dst_port = 0;

        /* Initialize the pointer to the start of the ethernet header in the     */
        /* buffer received from ethernet.                                        */

        /* MHSS : HUT MODE TBD */
        eth_hdr = buffer_addr + buffer_ofst + ETH_ETHERNET_HDR_OFFSET;

        /* Extract the type of the ethernet packet. Extract the ethernet type    */
        /* and the IP protocol. If the packet is a UDP packet, check if the      */
        /* source and destination ports match the values as used by the          */
        /* configuration packets.                                                */
        eth_type    = ((eth_hdr[ETH_PKT_TYPE_OFFSET] << 8) |
                        eth_hdr[ETH_PKT_TYPE_OFFSET + 1]);

        ip_protocol = eth_hdr[IP_PROT_OFFSET];

        udp_src_port = ((eth_hdr[UDP_SRC_PORT_OFFSET] << 8) |
                         eth_hdr[UDP_SRC_PORT_OFFSET + 1]);

        udp_dst_port = ((eth_hdr[UDP_DST_PORT_OFFSET] << 8) |
                         eth_hdr[UDP_DST_PORT_OFFSET + 1]);

        if((eth_type != IP_TYPE) || (ip_protocol != UDP_TYPE) ||
           (udp_src_port != HUT_HOST_UDP_PORT) ||
           (udp_dst_port != HUT_WLAN_UDP_PORT))
        {
            return BFALSE;
        }
   }
#endif /* ETHERNET_HOST */

    /* Update the Buffer Variables to represent the data to be transmitted */

    /* MHSS : HUT MODE TBD */
    offset       = buffer_ofst + get_eth_hdr_offset(host_type) + UDP_DATA_OFFSET;
    buffer_len  -= UDP_DATA_OFFSET;

    /* Initialize the mac header */
    frm_header = buffer_addr + offset;

    /* Read the Per-Packet Attribute Field if present. The presence of the */
    /* Attribute field is indicated by setting BIT0 of the first byte.     */
    /* In WLAN frames this is the version field and is always set to 0.    */
    if(frm_header[0] & HUT_FRAME_ATTRIB_FLAG)
    {
        frame_attrib = frm_header[3];
        frame_attrib = (frame_attrib << 8) | frm_header[2];
        frame_attrib = (frame_attrib << 8) | frm_header[1];
        frame_attrib = (frame_attrib << 8) |
                                    (frm_header[0] & ~HUT_FRAME_ATTRIB_FLAG);
        /* Update frame length and header */
        offset     += 4;
        buffer_len -= 4;
        frm_header += 4;
    }

    frame_type    = get_type(frm_header);
    frame_subtype = get_sub_type(frm_header);

    if(frame_type == DATA_BASICTYPE)
    priority = get_priority_value(frm_header);
    else if(frame_subtype == BLOCKACK_REQ)
    {
        /* The TID is extracted from the BA-Ctl Field */
        priority = ((frm_header[17] >> 4) & 0x0F);
    }

    /* Create Tx-Descriptor */
    tx_dscr = create_default_tx_dscr(1, priority, 0);

    if(NULL == tx_dscr)
        return BFALSE;

    /* Update the parameters in the Tx-Descriptor */
    get_DA(frm_header, dest_addr);
    tx_rate     = get_hut_tx_rate();
    pream       = get_preamble(tx_rate);
    phy_tx_mode = get_hut_phy_tx_mode(tx_rate, pream);

    /* Set the rate set words with the same rate as current transmit rate */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

#ifdef MAC_802_11N
    if(frame_subtype == BLOCKACK_REQ)
        ack_policy = COMP_BAR;
    else
#endif /* MAC_802_11N */
        ack_policy  = get_ack_policy(frm_header, dest_addr);

    if(get_type(frm_header) == DATA_BASICTYPE)
        q_num = get_txq_num_hut(priority);
    else
        q_num = HIGH_PRI_Q;

    /* Update Security related parameters */
#ifdef MAC_802_11I
    if((frame_type == DATA_BASICTYPE) && (g_test_config.ce.ce_enable == 1))
    {
        set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, g_test_config.ce.ct);

        if(is_group(dest_addr) == BTRUE)
        {
            set_tx_dscr_key_type((UWORD32 *)tx_dscr, 0);
            set_tx_dscr_key_index((UWORD32 *)tx_dscr, 0);
        }
        else
        {
        set_tx_dscr_key_type((UWORD32 *)tx_dscr, 1);
            if(mac_addr_cmp(dest_addr, g_test_config.tx.da) == BTRUE)
        set_tx_dscr_key_index((UWORD32 *)tx_dscr, 1);
            else
                set_tx_dscr_key_index((UWORD32 *)tx_dscr, 2);
        }

        /* HACK: Should be done in the frame creator itself */
        set_wep(frm_header, 1);
    }
    else
    {
        set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, NO_ENCRYP);
    }
#else /* MAC_802_11I */
    set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, NO_ENCRYP);
#endif /* MAC_802_11I */

    set_tx_params((UWORD8 *)tx_dscr, tx_rate, pream, ack_policy, phy_tx_mode,
                  retry_set);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

#ifdef MAC_802_11N
    /* Set the SMPS mode field in the Tx descriptor */
    set_tx_dscr_smps_mode((UWORD32 *)tx_dscr, g_test_config.ht.da_smps_mode);
#else /* MAC_802_11N */
    /* Set the SMPS mode field in the Tx descriptor to 0 */
    set_tx_dscr_smps_mode((UWORD32 *)tx_dscr, 0);
#endif /* MAC_802_11N */

    /* Set the mac header related parameters in the Tx descriptor */
    if(frame_subtype == BLOCKACK_REQ)
        mac_hdr_len = 16;
    else
    mac_hdr_len = get_mac_hdr_len(frm_header);
    set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)buffer_addr);
    set_tx_dscr_mh_len((UWORD32 *)tx_dscr, mac_hdr_len);
    set_tx_dscr_mh_offset((UWORD32 *)tx_dscr, offset);

    /* Transmit the frame */
    buffer_desc.buff_hdl    = buffer_addr;
    buffer_desc.data_offset = mac_hdr_len + offset;
    buffer_desc.data_length = buffer_len - mac_hdr_len;

#ifndef DEBUG_KLUDGE
    /* This Kludge fixes the TX-Hang issue in MAC H/w */
    buffer_desc.data_length += 4;
#endif /* DEBUG_KLUDGE */

    /* Set the data buffer related information in the Tx-descriptor */
    set_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr,
                                  (UWORD32)(&buffer_desc),
                                  0);

    /* Set the frame length in the Tx Descriptor */
    set_tx_dscr_frame_len((UWORD32 *)tx_dscr, buffer_len + FCS_LEN);
    set_tx_dscr_num_submsdu((UWORD32 *)tx_dscr, 1);
    set_tx_dscr_frag_len((UWORD32 *)tx_dscr, buffer_len+ FCS_LEN, 0);

    /* Update the Tx-Dscr with the attributes of the frame */
    if(frame_attrib & HUT_RETX_FRAME_FLAG)
    {
        /* This frame should be marked as a Retransmitted frame. */
        /* Set Fragment Status to Block-Ack Retry Requested. */
        set_tx_dscr_frag_status_summary((UWORD32 *)tx_dscr, 0xFFFFFFFE);
        if(g_test_config.ce.ce_enable == 1)
        {
            UWORD32 temp_iv = 0x20000000 | get_sequence_number(frm_header);

            /* The IV-Field is set to the Sequence number of the incoming frame */
            set_tx_dscr_iv32((UWORD32 *)tx_dscr, temp_iv);
        }

    }
    else if(frame_attrib & HUT_CE_DISABLE_FLAG)
    {
        /* Disable encryption for this frame */
        set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, NO_ENCRYP);
        /* HACK: Should be done in the frame creator itself */
        set_wep(frm_header, 0);
    }

    /* Update the TSF timestamp */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

#ifdef MAC_802_11N
    /* Get the index of the A-MPDU session that this frame is part of. This  */
    /* is done by checking the test sessions for an entry with matching TID. */
    /* If no match is found the RA LUT index is set to 0. If match is found  */
    /* the index is incremented by 1.                                        */
    for(idx = 0; idx < g_test_config.ht.ampdu_num_sessions; idx++)
    {
        if(g_test_config.ht.ampdu_sess_params[idx].tid == priority)
            break;
    }

    if(idx == g_test_config.ht.ampdu_num_sessions)
        idx = 0;
    else
        idx++;
#else /* MAC_802_11N */
    /* For non-11N case always use a value of 0 for the RA LUT index */
    idx = 0;
#endif /* MAC_802_11N */

    /* Set the LUT index in the descriptor */
    set_tx_dscr_ra_lut_index((UWORD32 *)tx_dscr, idx);

    /* If RA LUT index is 0, indicating no A-MPDU session, queue it to the   */
    /* H/w queue and return.                                                 */
    if(idx == 0)
    {
        if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
        {
            mem_free(g_shared_pkt_mem_handle, buffer_addr);
            free_tx_dscr((UWORD32 *)tx_dscr);
        }

        return BTRUE;
    }

#ifdef MAC_802_11N

#ifdef ENABLE_HOST_AMPDU_TEST
    /* If RA LUT index is not 0, queue it in the AMPDU buffer queue if the   */
    /* maxnum is not yet reached.                                            */
    if(g_ampdu_buff_num[idx - 1] <
                              g_test_config.ht.ampdu_sess_params[idx - 1].numbuf)
    {
        if(q_insert_at_tail(&g_ampdu_buff_q[idx - 1], (void *)tx_dscr) ==
           BFALSE)
        {
            mem_free(g_shared_pkt_mem_handle, buffer_addr);
            free_tx_dscr((UWORD32 *)tx_dscr);
        }
        else
        {
            /* The counter is for Data frames only */
            if(frame_type == DATA_BASICTYPE)
            g_ampdu_buff_num[idx - 1]++;
        }

    }

    /* If RA LUT index is not 0 and the AMPDU buffer maxnum limit is reached */
    /* dequeue all the packets and add to the H/w queue.                     */
    if(g_ampdu_buff_num[idx-1] == g_test_config.ht.ampdu_sess_params[idx-1].numbuf)
    {
        UWORD32 *first_tx_dscr = NULL;
        UWORD32 *last_tx_dscr  = NULL;
        UWORD8  num_txdscr     = 0;
        UWORD8  q_num = 0;

        /* Create a List of Tx-Dscrs to queue */
        while(1)
        {
            tx_dscr = q_remove_from_head(&g_ampdu_buff_q[idx - 1]);

            if(tx_dscr == NULL)
                break;

            if(first_tx_dscr == NULL)
            {
                first_tx_dscr = (UWORD32 *)tx_dscr;
                last_tx_dscr  = first_tx_dscr;
            }
            else
            {
                set_tx_dscr_next_addr(last_tx_dscr, (UWORD32)tx_dscr);
                last_tx_dscr = (UWORD32 *)tx_dscr;
            }
            num_txdscr++;
        }

        g_ampdu_buff_num[idx - 1] = 0;

        q_num = get_tx_dscr_q_num(first_tx_dscr);

        /* Set the MAC H/w queue pointers based on configuration */
        if(qmu_add_tx_packet_list(&g_q_handle.tx_handle, q_num, (UWORD8 **)(&first_tx_dscr),
                    (UWORD8 **)(&last_tx_dscr), &num_txdscr) != QMU_OK)
        {
            tx_dscr = (UWORD8 *)first_tx_dscr;

            while(num_txdscr-- > 0)
            {
                buffer_addr = (UWORD8 *)get_tx_dscr_buffer_addr(
                                                            (UWORD32 *)tx_dscr);
                mem_free(g_shared_pkt_mem_handle, buffer_addr);
                free_tx_dscr((UWORD32 *)tx_dscr);
                tx_dscr = (UWORD8 *)get_tx_dscr_next_addr((UWORD32 *)tx_dscr);
            }
        }
    }
#endif /* ENABLE_HOST_AMPDU_TEST */

#endif /* MAC_802_11N */

    return BTRUE;
#else /* ETHERNET_HOST */
    /* Currently implemented for Ethernet Host Interface only */
    return BFALSE;
#endif /* ETHERNET_HOST */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_hut_tx_rate                                       */
/*                                                                           */
/*  Description      : This function returns the transmission rate for H/w   */
/*                     unit test mode.                                       */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_test_params                                         */
/*                                                                           */
/*  Processing       : The transmit packet format is checked and the         */
/*                     appropriate transmission rate/MCS is returned.        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD8, Data Rate (mbps) or MCS with BIT7 set         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_hut_tx_rate(void)
{
    UWORD8 curr_tx_mcs = 0;

    curr_tx_mcs = get_curr_tx_mcs();

    /* Tx Rate is returned if Transmit Packet Format is Non-HT (0) */
    if((g_test_params.tx_format == 0) || (curr_tx_mcs == INVALID_MCS_VALUE))
    {
        /* Get the rate from the list of configured values */
        if(g_test_config.phy.mode == PHY_TEST_PER_TX_MODE)
            return get_frm_stat_tx_rate(g_phy_test_idx);

        return get_curr_tx_rate();
    }

    /* If Transmit Packet Format is HT, Tx MCS is returned with Bit7 set */
    if(g_test_config.phy.mode == PHY_TEST_PER_TX_MODE)
        curr_tx_mcs  = get_frm_stat_tx_rate(g_phy_test_idx);

    curr_tx_mcs |= BIT7;

    return curr_tx_mcs;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_hut_phy_tx_mode                                   */
/*                                                                           */
/*  Description      : This function returns the PHY Tx mode for H/w unit    */
/*                     test mode.                                            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_test_params                                         */
/*                                                                           */
/*  Processing       : The function sets the PTM to the correct value for HT */
/*                     rates only. For non-HT rates, the PTM is set assuming */
/*                     HT-Mixed Frame format.                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD32, PHY Tx mode according to format specified    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 get_hut_phy_tx_mode(UWORD8 dr, UWORD8 pr)
{
    UWORD32 temp = 0;


    /* b2:b0   000: 11b Short Preamble                                       */
    /*         001: 11b Long Preamble                                        */
    /*         010: 11a                                                      */
    /*         011: Reserved                                                 */
    /*         100: Reserved                                                 */
    /*         101: Reserved                                                 */
    /*         110: HT Mixed Mode Frame                                      */
    /*         111: HT Green Field Frame                                     */
    if(g_test_params.tx_format == 1) /* HT-GF Mode */
        temp = 7;
    else /* HT-Mixed */
        temp = 6;

    /* b4:b3   00: HT_BW20-20 MHz                                            */
    /*         01: HT_BW40-40 MHz                                            */
    /*         10: HT_BW_20DL-Duplicate Legacy                               */
    /*         11: HT_BW_20DH-Duplicate HT                                   */
    temp |= ((UWORD32)(g_test_params.bandwidth) & 0x03) << 3;

    /* b6:b5   00: 40 MHz Band                                               */
    /*         01: Upper 20 MHz in 40 MHz Band                               */
    /*         10: Reserved                                                  */
    /*         11: Lower 20 MHz in 40 MHz Band                               */
    temp |= ((UWORD32)(g_test_params.op_band) & 0x03) << 5;

    /* b7      0: Smoothing is not recommended                               */
    /*         1: Smoothing is recommended                                   */
    if(g_test_params.smoothing_rec == 1)
        temp |= BIT7;

    /* b8      0: A sounding packet                                          */
    /*         1: Not a sounding packet                                      */
    if(g_test_params.sounding_pkt == 0)
        temp |= BIT8;

    /* b9      0: A packet without MPDU aggregation                          */
    /*         1: A packet with MPDU aggregation                             */

    /* b10     0: Binary Convolution code                                    */
    /*         1: Low Density Parity Check code                              */
    if(g_test_params.ht_coding == 1)
        temp |= BIT10;

    /* b11     0: Short GI is not used for this packet                       */
    /*         1: Short GI is used for this packet                           */
    if(g_test_params.shortgi == 1)
        temp |= BIT11;

    /* b13:b12 00: No STBC (NSS = NSTS)                                      */
    /*         01: Reserved                                                  */
    /*         10: Reserved                                                  */
    /*         11: Reserved                                                  */
    temp |= ((UWORD32)(g_test_params.stbc) & 0x03) << 12;

    /* b15:b14 Number of Extension spatial streams                           */
    temp |= ((UWORD32)(g_test_params.ess) & 0x03) << 14;

    /* b23:b16 Antenna set. This is an 8 bit set.                            */
    if(g_test_params.antset == 0)
        temp |= (0x0F << 16);
    else
    temp |= ((UWORD32)(g_test_params.antset) & 0xFF) << 16;

    /* b31:b24 Reserved. Set to zero                                         */

    return temp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : compute_sig_qual_avg                                  */
/*                                                                           */
/*  Description      : This function computes the average signal quality     */
/*                     over the required window.                             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_sig_qual_sum                                        */
/*                     g_sig_qual_count                                      */
/*                     g_sig_qual_avg                                        */
/*                     g_sig_qual_avg_cnt                                    */
/*                                                                           */
/*  Processing       : The function gets the signal quality from the receive */
/*                     descriptor, adds it to the accumulated signal quality */
/*                     sum and increments the count. On reaching required    */
/*                     averaging count, the average is computed and the      */
/*                     sum and count are reset to 0.                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void compute_sig_qual_avg(UWORD32* rx_dscr)
{
    UWORD16 cur_sig_qual = 0;

    /* Extract the current signal quality from the receive descriptor */
    cur_sig_qual = get_rx_dscr_signal_quality(rx_dscr);

    /* Check whether the current signal quality value is valid */
    if(cur_sig_qual == 0)
        return;

    /* Add the current signal quality to the accumulated signal quality sum */
    g_sig_qual_sum += cur_sig_qual;

    /* Increment the count of signal quality read if it is a non-zero value */
    g_sig_qual_count++;

    /* If the required averaging window is exceeded, compute the average and */
    /* reset the sum and counts.                                             */
    if((g_sig_qual_avg_cnt > 0) && (g_sig_qual_avg_cnt <= g_sig_qual_count))
    {
        g_sig_qual_avg   = g_sig_qual_sum/g_sig_qual_avg_cnt;
        g_sig_qual_count = 0;
        g_sig_qual_sum   = 0;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_rssi                                              */
/*                                                                           */
/*  Description   : This function calculates the RSSI Extra and the adjusted */
/*                  RSSI.                                                    */
/*                                                                           */
/*  Inputs        : 1) Receive Descriptor                                    */
/*                                                                           */
/*  Globals       : g_target_rssi_11g                                        */
/*                  g_target_rssi_11b                                        */
/*                  g_rssi_ex                                                */
/*                  g_adj_rssi_ex                                            */
/*                  g_rssi_ex_count                                          */
/*                                                                           */
/*  Processing    : This function caluclates the average ofRSSIs, got form   */
/*                  rx descr and the average calculated over g_rssi_ex_count,*/
/*                  to get g_rssi_ex. Similarly g_adj_rssi_is also calculated*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_rssi(UWORD32* rx_dscr)
{
    UWORD8 cur_rate = 0;
    WORD32 dscr_rssi = 0;
    WORD32 adj_rssi = 0;
    WORD8  cur_rssi = 0;

    /* Update RSSI */
    dscr_rssi = get_rx_dscr_rssi((UWORD32 *)rx_dscr);
    cur_rate = get_rx_dscr_data_rate((UWORD32 *)rx_dscr);

    if(BFALSE == compute_rssi_db(dscr_rssi, IS_OFDM_RATE(cur_rate), &cur_rssi))
        return;

    if(IS_OFDM_RATE(cur_rate) == BTRUE)
    {
        adj_rssi = g_target_rssi_11g;
    }
    else
    {
        adj_rssi = g_target_rssi_11b;
    }

    g_rssi_ex_sum     = g_rssi_ex_sum + cur_rssi;
    g_adj_rssi_ex_sum = g_adj_rssi_ex_sum + cur_rssi + adj_rssi;

    if(cur_rssi != 0)
    {
        g_rssi_ex_sum_count++;
    }

    if (g_rssi_ex_count <= g_rssi_ex_sum_count)
    {
        g_rssi_ex           = (WORD8)(g_rssi_ex_sum/g_rssi_ex_count);
        g_adj_rssi_ex       = (WORD8)(g_adj_rssi_ex_sum/g_rssi_ex_count);
        g_rssi_ex_sum       = 0;
        g_adj_rssi_ex_sum   = 0;
        g_rssi_ex_sum_count = 0;
    }
}

#endif /* MAC_HW_UNIT_TEST_MODE */
