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
/*  File Name         : receive.h                                            */
/*                                                                           */
/*  Description       : This file contains the MAC receive path related      */
/*                      functions independent of mode.                       */
/*                                                                           */
/*  List of Functions : Functions to set/get fields in transmit descriptor   */
/*                      and other miscellaneous functions that involve       */
/*                      descriptor processing.                               */
/*                      Other receive path functions,                        */
/*                      update_debug_rx_stats                                */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef RECEIVE_H
#define RECEIVE_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "fsm.h"
#include "buff_desc.h"
#include "phy_prot_if.h"
#include "phy_hw_if.h"
#include "trout_share_mem.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* The format of the receive packet descriptor is as following               */
/*                                                                           */
/*        +---------------------------------------------------------------+  */
/* WORD0  |Status   |Cipher  |Data Rate           |Frame Length           |  */
/*        +---------+--------+--------------------+-----------------------+  */
/*        |4 bits   |4 bits  |8 bits              |16 bits                |  */
/*        +---------------------------------------------------------------+  */
/* WORD1  |Rx Silence Lev |RSSI          |SNR           | RX Time Stamp   |  */
/*        +---------------------------------------------------------------+  */
/*        |8 bits         |8 bits        |8 bits        |8 bits           |  */
/*        +---------------------------------------------------------------+  */
/* WORD2  |PHY Rx mode                                                    |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */
/* WORD3  |Rx Service field              |Rx Legacy length                |  */
/*        +---------------------------------------------------------------+  */
/*        |16 bits                       |16 bits                         |  */
/*        +---------------------------------------------------------------+  */
/* ------------------------------------------------------------------------  */
/* | NOTE: The WORD4 is present in the new receive descriptor (post v2.2) |  */
/* | and is not present in previous versions. All other fields are same.  |  */
/* ------------------------------------------------------------------------  */
/*        +---------------------------------------------------------------+  */
/* WORD4  |Reserved                      |Rx Signal Quality               |  */
/*        +---------------------------------------------------------------+  */
/*        |16 bits                       |16 bits                         |  */
/*        +---------------------------------------------------------------+  */
/* WORD5  |RX int seq num | Num Buffers |Flags  |Reserved  | Num MSDUs    |  */
/*        +---------------------------------------------------------------+  */
/*        |8 bits         |8 bits       |4 bits |4 bits    |8 bits        |  */
/*        +---------------------------------------------------------------+  */
/* WORD6  |Pointer to next buffer in defragmentation                      |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */
/* WORD7  |Rx Buffer Pointer                                              |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */
/* WORD8  |Pointer to next element in RX Buffer linked list               |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */
/* WORD9  |Pointer to previous element in RX Buffer linked list           |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

/* Macros indexing various fields within the RX-Dscr */
#define RX_DSCR_FIELD_STATUS_OFFSET               0
#define RX_DSCR_FIELD_CIPHER_TYPE_OFFSET          4
#define RX_DSCR_FIELD_DATA_RATE_OFFSET            8
#define RX_DSCR_FIELD_PACKET_LEN_OFFSET           16
#define RX_DSCR_FIELD_RX_SILENCE_LEVEL_OFFSET     0
#define RX_DSCR_FIELD_RSSI_OFFSET                 8
#define RX_DSCR_FIELD_SNR_OFFSET                  16
#define RX_DSCR_FIELD_RX_TIME_STAMP_OFFSET        24
#define RX_DSCR_FIELD_PHY_RX_MODE_OFFSET          0
#define RX_DSCR_FIELD_RX_SERVICE_FIELD_OFFSET     0
#define RX_DSCR_FIELD_RX_LEGACY_LENGTH_OFFSET     16
#define RX_DSCR_FIELD_SIGNAL_QUALITY_OFFSET       16
#define RX_DSCR_FIELD_INT_SEQ_NUM_OFFSET          0
#define RX_DSCR_FIELD_NUM_BUFFERS_OFFSET          8
#define RX_DSCR_FIELD_FLAGS_OFFSET                16
#define RX_DSCR_FIELD_NUM_MSDUS_OFFSET            24
#define RX_DSCR_FIELD_NEXT_DEFRAG_BUFFER_OFFSET   0
#define RX_DSCR_FIELD_RX_BUFFER_OFFSET            0
#define RX_DSCR_FIELD_NEXT_DSCR_OFFSET            0
#define RX_DSCR_FIELD_PREV_DSCR_OFFSET            0

/* Size of the RX-Buffer in Bytes */
#ifndef PHY_TEST_MAX_PKT_RX
#define RX_BUFFER_SIZE                            SHARED_PKT_MEM_BUFFER_SIZE
                                                  /* Default Packet Size */
#else /* PHY_TEST_MAX_PKT_RX */
#define RX_BUFFER_SIZE                            4096 /* Test Max Pkt Rx */
#endif /* PHY_TEST_MAX_PKT_RX */

/* Maximum descriptor sequence number by 2 */
#define MAX_DSCR_SEQNO_BY_TWO                     128

/* Invalid RSSI value in decibels */
#define INVALID_RSSI_DB                           127

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {RX_NEW              = 0x0,
              RX_SUCCESS          = 0x1,
              DUP_DETECTED        = 0x2,
              FCS_ERROR           = 0x3,
              KEY_SEARCH_FAILURE  = 0x4,
              MIC_FAILURE         = 0x5,
              ICV_FAILURE         = 0x6,
              TKIP_REPLAY_FAILURE = 0x7,
              CCMP_REPLAY_FAILURE = 0x8,
              TKIP_MIC_FAILURE    = 0x9,
              DEFRAG_IN_PROGRESS  = 0xA,
              DEFRAG_ERROR        = 0xB,
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	      PS_SUCCESS	  = 0xC,
#endif
#endif
} RX_DSCR_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8   service_class; /* Service class extracted from header       */
    UWORD8   priority_val;  /* Priority value extracted from header      */
    UWORD8   hdr_len;       /* Length of header including offset         */
    UWORD8   num_dscr;      /* Number of Rx descriptors for this frame   */
    UWORD16  data_len;      /* Length of the frame excluding headers     */
    UWORD16  rx_len;        /* Length of the frame including the headers */
    UWORD8   *sa_entry;     /* Source Address station entry              */
    UWORD8   *msa;          /* MAC header start address                  */
    UWORD32  *base_dscr;    /* Base Rx descriptor pointer                */
    UWORD8   ct;            /* Cipher type of received frame             */
    UWORD8   addr1[6];      /* Address1 field extracted from header      */
    UWORD8   addr2[6];      /* Address2 field extracted from header      */
    UWORD8   addr3[6];      /* Address3 field extracted from header      */
    UWORD8   *sa;           /* Pointer to source address                 */
    UWORD8   *da;           /* Pointer to destination address            */
    UWORD8   *bssid;        /* Pointer to BSSID                          */
    UWORD8   *ta;           /* Pointer to Transmitter address            */
    BASICTYPE_T type;       /* Packet type extracted from the header     */
    UWORD8   sub_type;      /* Packet Sub type extracted from the header */
    UWORD8   frm_ds;        /* From DS field of MAC header               */
    UWORD8   to_ds;         /* To DS field of MAC header                 */
    BOOL_T   is_grp_addr;   /* Group address flag                        */

	//chenq add for wapi 2012-09-29
	#ifdef MAC_WAPI_SUPP
	UWORD8 wapi_mode;
	#endif
} wlan_rx_t;

typedef struct
{
    UWORD8      sa[6];
    UWORD8      da[6];
    UWORD8      priority;
    msdu_desc_t frame_desc;
} msdu_indicate_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void rx_frame_test(mac_struct_t *mac, UWORD32 *base_dscr, UWORD8 num_dscr);	//chengwg debug.
extern void process_wlan_rx(mac_struct_t *mac, UWORD8 *msg);
extern void process_rx_frame(mac_struct_t *mac, UWORD32 *base, UWORD8 num);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Functions to get and set various fields in the receive descriptor         */
/*****************************************************************************/

/*        +---------------------------------------------------------------+  */
/* WORD0  |Status   |Cipher  |Data Rate           |Frame Length           |  */
/*        +---------+--------+--------------------+-----------------------+  */
/*        |4 bits   |4 bits  |8 bits              |16 bits                |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_pkt_status(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_STATUS_OFFSET, FIELD_WIDTH_4,
                    (rx_dscr + WORD_OFFSET_0));
}

INLINE UWORD8 get_rx_dscr_cipher_type(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_CIPHER_TYPE_OFFSET, FIELD_WIDTH_4,
                    (rx_dscr + WORD_OFFSET_0));
}

INLINE UWORD32 get_rx_dscr_data_rate(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_DATA_RATE_OFFSET, FIELD_WIDTH_8,
                    (rx_dscr + WORD_OFFSET_0));
}

INLINE UWORD32 get_rx_dscr_frame_len(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_PACKET_LEN_OFFSET, FIELD_WIDTH_16,
                    (rx_dscr + WORD_OFFSET_0));
}

/*        +---------------------------------------------------------------+  */
/* WORD1  |Rx Silence Lev |RSSI          |SNR           | RX Time Stamp   |  */
/*        +---------------------------------------------------------------+  */
/*        |8 bits         |8 bits        |8 bits        |8 bits           |  */
/*        +---------------------------------------------------------------+  */

INLINE WORD8 get_rx_dscr_rssi(UWORD32 *rx_dscr)
{
    return (WORD8)get_dscr(RX_DSCR_FIELD_RSSI_OFFSET, FIELD_WIDTH_8,
                           (rx_dscr + WORD_OFFSET_1));
}

//chenq mod UWORD32 => WORD8
//INLINE UWORD32 get_rx_dscr_snr(UWORD32 *rx_dscr)
INLINE UWORD8 get_rx_dscr_snr(UWORD32 *rx_dscr)
{
	//printk("chenq_itm: get_rx_dscr_snr: %d\n",
	//	(WORD32)get_dscr(RX_DSCR_FIELD_SNR_OFFSET, FIELD_WIDTH_8,
    //                (rx_dscr + WORD_OFFSET_1)));

    //return get_dscr(RX_DSCR_FIELD_SNR_OFFSET, FIELD_WIDTH_8,
                    //(rx_dscr + WORD_OFFSET_1));
    return (UWORD8)get_dscr(RX_DSCR_FIELD_SNR_OFFSET, FIELD_WIDTH_8,
                    (rx_dscr + WORD_OFFSET_1));                
}

/*        +---------------------------------------------------------------+  */
/* WORD2  |PHY Rx mode                                                    |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_prm(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_PHY_RX_MODE_OFFSET, FIELD_WIDTH_32,
                    (rx_dscr + WORD_OFFSET_2));
}

/*        +---------------------------------------------------------------+  */
/* WORD4  |Reserved                      |Rx Signal Quality               |  */
/*        +---------------------------------------------------------------+  */
/*        |16 bits                       |16 bits                         |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_signal_quality(UWORD32 *rx_dscr)
{
#ifdef OLD_RX_DSCR_FORMAT_v2_2
    return 0; /* This word is not defined in the old receive descriptor */
#else /* OLD_RX_DSCR_FORMAT_v2_2 */
    return get_dscr(RX_DSCR_FIELD_SIGNAL_QUALITY_OFFSET, FIELD_WIDTH_16,
                    (rx_dscr + WORD_OFFSET_4));
#endif /* OLD_RX_DSCR_FORMAT_v2_2 */
}

/*        +---------------------------------------------------------------+  */
/* WORD5  |RX int seq num |Flags  |Reserved  | Num Buffers | Num MSDUs    |  */
/*        +---------------------------------------------------------------+  */
/*        |8 bits         |4 bits |4 bits    |8 bits       |8 bits        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD8 get_rx_dscr_int_seq_num(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_INT_SEQ_NUM_OFFSET, FIELD_WIDTH_8,
                    (rx_dscr + WORD_OFFSET_4 + RX_DSCR_EXTRA_WORD_OFFSET));
}

INLINE UWORD8 get_rx_dscr_flags(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_FLAGS_OFFSET, FIELD_WIDTH_4,
                    (rx_dscr + WORD_OFFSET_4 + RX_DSCR_EXTRA_WORD_OFFSET));
}

INLINE UWORD8 get_rx_dscr_num_buffers(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_NUM_BUFFERS_OFFSET, FIELD_WIDTH_8,
                    (rx_dscr + WORD_OFFSET_4 + RX_DSCR_EXTRA_WORD_OFFSET));
}

INLINE UWORD8 get_rx_dscr_num_msdus(UWORD32 *rx_dscr)
{
    return get_dscr(RX_DSCR_FIELD_NUM_MSDUS_OFFSET, FIELD_WIDTH_8,
                    (rx_dscr + WORD_OFFSET_4 + RX_DSCR_EXTRA_WORD_OFFSET));
}

/*        +---------------------------------------------------------------+  */
/* WORD6  |Pointer to next buffer in defragmentation                      |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_next_defrag_buffer(UWORD32 *rx_dscr)
{
    UWORD32 temp = 0;

    temp = get_dscr(RX_DSCR_FIELD_NEXT_DEFRAG_BUFFER_OFFSET, FIELD_WIDTH_32,
                    (rx_dscr + WORD_OFFSET_5 + RX_DSCR_EXTRA_WORD_OFFSET));

    return phy_to_virt_addr(temp);
}

/*        +---------------------------------------------------------------+  */
/* WORD7  |Rx Buffer Pointer                                              |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_buffer_ptr(UWORD32 *rx_dscr)
{
    UWORD32 temp = 0;

    temp = get_dscr(RX_DSCR_FIELD_RX_BUFFER_OFFSET, FIELD_WIDTH_32,
                    (rx_dscr + WORD_OFFSET_6 + RX_DSCR_EXTRA_WORD_OFFSET));

    return phy_to_virt_addr(temp);
}

INLINE void set_rx_dscr_buffer_ptr(UWORD32 *rx_dscr, UWORD32 value)
{
    set_dscr(RX_DSCR_FIELD_RX_BUFFER_OFFSET, FIELD_WIDTH_32,
             (rx_dscr + WORD_OFFSET_6 + RX_DSCR_EXTRA_WORD_OFFSET),
             virt_to_phy_addr(value));
}

/*        +---------------------------------------------------------------+  */
/* WORD8  |Pointer to next element in RX Buffer linked list               |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_next_dscr(UWORD32 *rx_dscr)
{
    UWORD32 temp = 0;

    temp = get_dscr(RX_DSCR_FIELD_NEXT_DSCR_OFFSET, FIELD_WIDTH_32,
                    (rx_dscr + WORD_OFFSET_7 + RX_DSCR_EXTRA_WORD_OFFSET));

    return phy_to_virt_addr(temp);
}

INLINE void set_rx_dscr_next_dscr(UWORD32 *rx_dscr, UWORD32 value)
{
    set_dscr(RX_DSCR_FIELD_NEXT_DSCR_OFFSET, FIELD_WIDTH_32,
             (rx_dscr + WORD_OFFSET_7 + RX_DSCR_EXTRA_WORD_OFFSET),
             virt_to_phy_addr(value));
}

/*        +---------------------------------------------------------------+  */
/* WORD9  |Pointer to previous element in RX Buffer linked list           |  */
/*        +---------------------------------------------------------------+  */
/*        |32 bits                                                        |  */
/*        +---------------------------------------------------------------+  */

INLINE UWORD32 get_rx_dscr_prev_dscr(UWORD32 *rx_dscr)
{
    UWORD32 temp = 0;

    temp = get_dscr(RX_DSCR_FIELD_PREV_DSCR_OFFSET, FIELD_WIDTH_32,
                    (rx_dscr + WORD_OFFSET_8 + RX_DSCR_EXTRA_WORD_OFFSET));

    return phy_to_virt_addr(temp);
}

INLINE void set_rx_dscr_prev_dscr(UWORD32 *rx_dscr, UWORD32 value)
{
    set_dscr(RX_DSCR_FIELD_PREV_DSCR_OFFSET, FIELD_WIDTH_32,
             (rx_dscr + WORD_OFFSET_8 + RX_DSCR_EXTRA_WORD_OFFSET),
             virt_to_phy_addr(value));
}

/*****************************************************************************/
/* Functions to check the flags set in the receive descriptor                */
/* ------------------------------------------------------------------------- */
/* b0 - MSDU aggregation status (0 - No aggregation, 1 - aggregation)        */
/* b1 - Buffer start flag                                                    */
/*        0 - Middle or last RX buffer corresponding to the current MPDU     */
/*        1 - Starting buffer of the current MPDU                            */
/* b2 - Fragmentation status                                                 */
/*         0 - the frame is not received as part of a fragmentation          */
/*         1 - Frame received through fragmentation mechanism                */
/* b3 - Reserved                                                             */
/*****************************************************************************/

/* This function returns BTRUE if the MSDU is an A-MSDU and BFALSE otherwise */
INLINE BOOL_T is_rx_frame_amsdu(UWORD32 *rx_dscr)
{
    UWORD8 rx_flags = get_rx_dscr_flags(rx_dscr);

    if(rx_flags & BIT0)
        return BTRUE;

    return BFALSE;
}

/* This function returns BTRUE if the buffer is the starting buffer of the */
/* current MPDU and BFALSE otherwise.                                      */
INLINE BOOL_T is_rx_frame_start_buf(UWORD32 *rx_dscr)
{
    UWORD8 rx_flags = get_rx_dscr_flags(rx_dscr);

    if(rx_flags & BIT1)
        return BTRUE;

    return BFALSE;
}

/* This function returns BTRUE if the MSDU was received through fragmentation*/
/* mechanism and BFALSE otherwise                                            */
INLINE BOOL_T is_rx_frame_frag(UWORD32 *rx_dscr)
{
    UWORD8 rx_flags = get_rx_dscr_flags(rx_dscr);

    if(rx_flags & BIT2)
        return BTRUE;

    return BFALSE;
}

/*****************************************************************************/
/* Miscellaneous functions for receive descriptor processing                 */
/*****************************************************************************/

INLINE void reset_rx_dscr(UWORD32 *rx_dscr)
{
    UWORD32 idx = 0;

    for (idx = 0; idx < RX_DSCR_NUM_WORDS; idx++)
        rx_dscr[idx] = 0;
}

/* This function frees the receive descriptor and all the packet buffers */
/* associated with it. No link updation is performed by this function.   */
INLINE void free_rx_dscr(UWORD32 *rx_dscr)
{
    UWORD32 *rx_buffer = (UWORD32 *)get_rx_dscr_buffer_ptr(rx_dscr);

    /* Free the receive frame buffer associated with the descriptor */
    if(NULL != rx_buffer)
        pkt_mem_free(rx_buffer);

    /* Free the receive descriptor */
    mem_free(g_shared_dscr_mem_handle, rx_dscr);
}

/* This function frees a list of receive descriptors and returns the next    */
/* descriptor in the list.                                                   */
INLINE UWORD32 *free_rx_dscr_list(UWORD32 *rx_dscr, UWORD8 count)
{
    UWORD32 *temp_dscr = 0;

    while(count--)
    {
        if(rx_dscr == NULL)
            return NULL;

        temp_dscr = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);
        free_rx_dscr(rx_dscr);
        rx_dscr = temp_dscr;
    }

    return rx_dscr;
}

/* This function adds the required number of additional users for the */
/* Rx-descriptor and the associated Rx-Frame buffer.                  */
INLINE void add_user_rx_dscr(UWORD32 *rx_dscr, UWORD8 count)
{
    UWORD32 *rx_buffer = (UWORD32 *)get_rx_dscr_buffer_ptr(rx_dscr);

    /* The required number of extra users is added to the Rx-Frame buffer */
    if(NULL != rx_buffer)
        mem_add_users(g_shared_pkt_mem_handle, rx_buffer, count);

    /* The required number of extra users is added to the Rx-Dscr buffer */
    mem_add_users(g_shared_dscr_mem_handle, rx_dscr, count);
}

/* This function adds the required number of additional users for the list */
/* of Rx-Descriptors.                                                      */
INLINE void add_user_rx_dscr_list(UWORD32 *rx_dscr, UWORD8 num_dscr,
                                  UWORD8 count)
{
    while(num_dscr--)
    {
        if(rx_dscr == NULL)
            return;

        add_user_rx_dscr(rx_dscr, count);

        rx_dscr = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);
    }
}

/* This function gets the last descriptor from a list of receive descriptors */
INLINE UWORD32 *get_last_rx_dscr(UWORD32 *rx_dscr, UWORD8 count)
{
    while(--count)
    {
        if(rx_dscr == NULL)
            return NULL;

        rx_dscr = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);
    }

    return rx_dscr;
}

/*****************************************************************************/
/* Function to process WLAN receive event and frame                          */
/*****************************************************************************/

extern UWORD32 g_unable2decode;
extern UWORD32 g_perform_mac_reassembly_err;
extern UWORD32 g_rx_misc;

/* This function updates debug statistics based on status of received frame */
INLINE void update_debug_rx_stats(UWORD8 status)
{
#ifdef IBSS_BSS_STATION_MODE
	//chenq add for check mac state
	switch(status)
    {
    case KEY_SEARCH_FAILURE:
    case MIC_FAILURE:
    case ICV_FAILURE:
    case TKIP_REPLAY_FAILURE:
    case CCMP_REPLAY_FAILURE:
    case TKIP_MIC_FAILURE:
		g_unable2decode++;
		break;
    case DEFRAG_IN_PROGRESS:
    case DEFRAG_ERROR:
		g_perform_mac_reassembly_err++;
		break;
    default:
        g_rx_misc++;
    break;
    }
#endif

#ifdef DEBUG_MODE
    switch(status)
    {
    case RX_NEW:
        g_mac_stats.pewrxnew++;
    break;
    case RX_SUCCESS:
        g_mac_stats.pewrx++;
    break;
    case DUP_DETECTED:
        g_mac_stats.pewrxdup++;
    break;
    case FCS_ERROR:
        g_mac_stats.pewrxfcs++;
    break;
    case KEY_SEARCH_FAILURE:
        g_mac_stats.pewrxkf++;
    break;
    case MIC_FAILURE:
        g_mac_stats.pewrxmf++;
    break;
    case ICV_FAILURE:
        g_mac_stats.pewrxicvf++;
    break;
    case TKIP_REPLAY_FAILURE:
        g_mac_stats.pewrxtkiprepf++;
    break;
    case CCMP_REPLAY_FAILURE:
        g_mac_stats.pewrxccmprepf++;
    break;
    case TKIP_MIC_FAILURE:
        g_mac_stats.pewrxtkipmicf++;
    break;
    case DEFRAG_IN_PROGRESS:
        g_mac_stats.pewrxdip++;
    break;
    case DEFRAG_ERROR:
        g_mac_stats.pewrxdeferr++;
    break;
    default:
        g_mac_stats.pewrxunknown++;
    break;
    }
#endif /* DEBUG_MODE */
}

/* This function checks if sequence number 1 is less than sequence number 2  */
/* as indicated in the descriptor.                                           */
INLINE BOOL_T dscr_seqno_lt(UWORD16 seq1, UWORD16 seq2)
{
    if(((seq1 < seq2) && ((seq2 - seq1) < MAX_DSCR_SEQNO_BY_TWO)) ||
       ((seq1 > seq2) && ((seq1 - seq2) > MAX_DSCR_SEQNO_BY_TWO)))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks whether the received frame is of OFDM or DSSS type */
/* and computes the RSSI (in db) from the value returned in the receive    */
/* descriptor accordingly.                                                 */
INLINE WORD8 get_rx_dscr_rssi_db(UWORD32 *rx_dscr)
{
    SWORD8 rssi_db = 0;
    WORD8  rssi    = 0;
    UWORD8 rate    = 0;

    rate = get_rx_dscr_data_rate(rx_dscr);
    rssi = get_rx_dscr_rssi(rx_dscr);

    if(compute_rssi_db(rssi, IS_OFDM_RATE(rate), &rssi_db) == BTRUE)
        return rssi_db;

    return INVALID_RSSI_DB;
}

/* This function extracts the updates the SA, DA & BSSID address pointers to */
/* addr1, addr2 & addr3 fields in the WLAN RX structure.                     */
INLINE void set_SA_DA_BSSID_ptr(wlan_rx_t *wlan_rx)
{
    UWORD8 frm_ds = wlan_rx->frm_ds;
    UWORD8 to_ds  = wlan_rx->to_ds;
    if((to_ds == 0) && (frm_ds == 0))
    {
        wlan_rx->sa    = wlan_rx->addr2;
        wlan_rx->da    = wlan_rx->addr1;
        wlan_rx->bssid = wlan_rx->addr3;
    }
    else if((to_ds == 0) && (frm_ds == 1))
    {
        wlan_rx->sa    = wlan_rx->addr3;
        wlan_rx->da    = wlan_rx->addr1;
        wlan_rx->bssid = wlan_rx->addr2;
    }
    else if((to_ds == 1) && (frm_ds == 0))
    {
        wlan_rx->sa    = wlan_rx->addr2;
        wlan_rx->da    = wlan_rx->addr3;
        wlan_rx->bssid = wlan_rx->addr1;
    }

    wlan_rx->ta    = wlan_rx->addr2;
}

/* This function is used to handle the Address4 field in the header.         */
/* This checks the from and to DS bits in the header and if both are set     */
/* returns BTRUE, BFALSE otherwise                                           */
INLINE BOOL_T check_from_to_ds(wlan_rx_t *wlan_rx)
{
    if(wlan_rx->frm_ds && wlan_rx->to_ds)
        return BTRUE;

    return BFALSE;
}

/* This function checks for BIT9 in the PHY RX mode. This is used to         */
/* determine AMPDU receptions                                                */
INLINE BOOL_T is_ampdu_rx_prm(UWORD32 prm)
{
    if(prm & BIT9)
        return BTRUE;

    return BFALSE;
}
#endif /* RECEIVE_H */
