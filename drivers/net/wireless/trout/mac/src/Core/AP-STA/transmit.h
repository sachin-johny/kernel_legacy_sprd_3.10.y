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
/*  File Name         : transmit.h                                           */
/*                                                                           */
/*  Description       : This file contains the MAC transmit path related     */
/*                      definitions.                                         */
/*                                                                           */
/*  List of Functions : Functions to set/get fields in transmit descriptor   */
/*                      and other miscellaneous functions that involve       */
/*                      descriptor processing.                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef TRANSMIT_H
#define TRANSMIT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management.h"
#include "mib.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "qmu.h"
#include "mh.h"
#include "buff_desc.h"
#include "trout_share_mem.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* The transmit packet descriptor is set for a packet added to the transmit  */
/* queue. It need not be contiguous to the transmit packet memory. The       */
/* pointer to the next packet descriptor is also available as a field in     */
/* this descriptor. However the updation of this field is handled by the     */
/* queue manager and not of concern to the core MAC. The format of this      */
/* descriptor is shown below.                                                */
/*                                                                           */
/*          +------------------------------------------------------------+   */
/* WORD0    |Status/QoS|ACKPol|OpFlags|SMPS  |DataRate0|H/w Runtime flags|   */
/*          +------------------------------------------------------------+   */
/*          |8 bits    |4 bits  |2 bits  |2 bits   |8 bits   |8 bits     |   */
/*          +------------------------------------------------------------+   */
/* WORD1    |Tx Service field        | Tx Power Level   | RA LUT Index   |   */
/*          +------------------------------------------------------------+   */
/*          |16 bits                 | 8 bits           | 8 bits         |   */
/*          +------------------------------------------------------------+   */
/* WORD2    |PHY Tx mode                                                 |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD3    |Cipher|Key type| TX STA Addr Index| TSF Time Stamp          |   */
/*          +------------------------------------------------------------+   */
/*          |4 bits|4 bits  |8 bits            | 16 bits                 |   */
/*          +------------------------------------------------------------+   */
/* WORD4    | Pointer to Next Packet Descriptor                          |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD5    | Fragment Status                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD6:21 |Frag status | Frag offset (1:16)  |Frag length (1:16)       |   */
/*          +------------------------------------------------------------+   */
/*          |2 bits      |14 bits              |16 bits                  |   */
/*          +------------------------------------------------------------+   */
/* WORD22:25| Frag K+3 idx | Frag K+2 idx  | Frag K+1 idx | Frag K index |   */
/*          +------------------------------------------------------------+   */
/*          |   8 bits     |     8 bits    |     8 bits   |    8 bits    |   */
/*          +------------------------------------------------------------+   */
/* WORD26   |  Long Cnt | Short Cnt  | Short Retries | Long Retries      |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits    | 8 bits     | 8 bits        | 8 bits            |   */
/*          +------------------------------------------------------------+   */
/* WORD27   |     Frame length       | CTS Failures  | RTS Success       |   */
/*          +------------------------------------------------------------+   */
/*          |         16 bits        | 8 bits        | 8 bits            |   */
/*          +------------------------------------------------------------+   */
/* WORD28   | IV MS Word (63-32)                                         |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD29   | IV MS Word (31-0)                                          |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD30   | Buffer Start Address                                       |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD31   | Pointer to sub-MSDU information table                      |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD32   | MAC Header Offset | MAC Header len | Num sub-MSDU |Q Number|   */
/*          +------------------------------------------------------------+   */
/*          |8 bits             |8 bits          |8 bits        | 8 bits |   */
/*          +------------------------------------------------------------+   */
/* WORD33   | Data Rate 4  | Data Rate 3  | Data Rate 2   | Data Rate 1  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */
/* WORD34   | TSSI-Value   | Data Rate 7  | Data Rate 6   | Data Rate 5  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */


/* The structure of the sub-MSDU information table is shown below.           */
/* It is used to describe multiple non-contiguous MSDUs which the h/w should */
/* pack into a single AMSDU frame                                            */
/*                                                                           */
/*          +------------------------------------------------------------+   */
/* WORD0    | Sub-MSDU 0 Buffer Address                                  |   */
/*          +------------------------------------------------------------+   */
/*          | 32 bits                                                    |   */
/*          +------------------------------------------------------------+   */
/* WORD1    | Sub-MSDU 0 Offset           | Sub-MSDU 0 Length            |   */
/*          +------------------------------------------------------------+   */
/*          | 16 bits                     | 16 bits                      |   */
/*          +------------------------------------------------------------+   */
/*          :                             :                              :   */
/*          :                             :                              :   */
/*          +------------------------------------------------------------+   */
/* WORD2N   | Sub-MSDU N Buffer Address                                  |   */
/*          +------------------------------------------------------------+   */
/*          | 32 bits                                                    |   */
/*          +------------------------------------------------------------+   */
/* WORD2N+1 | Sub-MSDU N Offset           | Sub-MSDU N Length            |   */
/*          +------------------------------------------------------------+   */
/*          | 16 bits                     | 16 bits                      |   */
/*          +------------------------------------------------------------+   */

//#define TX_DSCR_LEN                               140 /* 35 * 4 Bytes */
#define TX_DSCR_NUM_WORDS                         35
#define MAX_SUB_MSDU_TABLE_ENTRIES                16
#define SUB_MSDU_ENTRY_LEN                        8

/* The minimum sub-MSDU information table size in bytes (with 1 entry only) */
#define MIN_SUBMSDU_TABLE_SIZE (SUB_MSDU_ENTRY_LEN)

/* The maximum sub-MSDU information table size in bytes */
#define MAX_SUBMSDU_TABLE_SIZE (MAX_SUB_MSDU_TABLE_ENTRIES * SUB_MSDU_ENTRY_LEN)

#define TX_DSCR_BUFF_SZ (MAX((TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE), MAX_SUBMSDU_TABLE_SIZE))

/* Bit Offets for the fields within the Transmit Descriptor Structure */
/* Note that the offsets are defined from the end of each word (Bit 31) */
#define TX_DSCR_FIELD_STATUS_OFFSET               0
#define TX_DSCR_FIELD_PRIORITY_OFFSET             4
#define TX_DSCR_FIELD_ACK_POLICY_OFFSET           8
#define TX_DSCR_FIELD_OP_FLAGS_OFFSET             12
#define TX_DSCR_FIELD_SMPS_MODE_OFFSET            14
#define TX_DSCR_FIELD_DATA_RATE_0_OFFSET          16
#define TX_DSCR_FIELD_HW_RUNTIME_FLAGS_OFFSET     24

#define TX_DSCR_SERVICE_FIELD_OFFSET              0
#define TX_DSCR_TX_POWER_LEVEL_OFFSET             16
#define TX_DSCR_RA_LUT_INDEX_OFFSET               24

#define TX_DSCR_PHY_TX_MODE_OFFSET                0

#define TX_DSCR_FIELD_CIPHER_TYPE_OFFSET          0
#define TX_DSCR_FIELD_KEY_TYPE_OFFSET             4
#define TX_DSCR_FIELD_KEY_INDEX_OFFSET            8
#define TX_DSCR_FIELD_TSF_TIME_STAMP_OFFSET       16

#define TX_DSCR_NEXT_ADDR_OFFSET                  0

#define TX_DSCR_FIELD_FRAG_STATUS_SUMMARY_OFFSET  0

#define TX_DSCR_FIELD_FRAG_STATUS_OFFSET          0
#define TX_DSCR_FIELD_FRAG_OFFSET_OFFSET          4
#define TX_DSCR_FIELD_FRAG_LEN_OFFSET             16

#define TX_DSCR_FIELD_LONG_RETRY_COUNT_OFFSET     0
#define TX_DSCR_FIELD_SHORT_RETRY_COUNT_OFFSET    8
#define TX_DSCR_FIELD_NUM_SHORT_RETRIES_OFFSET    16
#define TX_DSCR_FIELD_NUM_LONG_RETRIES_OFFSET     24

#define TX_DSCR_FIELD_FRAME_LEN_OFFSET            0
#define TX_DSCR_FIELD_CTS_FAILURE_OFFSET          16
#define TX_DSCR_FIELD_RTS_SUCCESS_OFFSET          24

#define TX_DSCR_FIELD_IV_MS_WORD_OFFSET           0

#define TX_DSCR_FIELD_BUFFER_START_ADDR_OFFSET    0

#define TX_DSCR_SUBMSDU_INFO_OFFSET               0

#define TX_DSCR_FIELD_MAC_HEADER_OFFSET_OFFSET    0
#define TX_DSCR_FIELD_MAC_HEADER_LEN_OFFSET       8
#define TX_DSCR_FIELD_NUM_SUB_MSDU_OFFSET         16
#define TX_DSCR_FIELD_Q_NUM_OFFSET                24

#define TX_DSCR_FIELD_RETRY_RATE_SET1_OFFSET      0 /* Data Rate 1 - 4 */
#define TX_DSCR_FIELD_RETRY_RATE_SET2_OFFSET      8 /* Data Rate 5 - 8 */

#define TX_DSCR_FIELD_TSSI_VALUE_OFFSET           0

#define SUB_MSDU_INFO_BUFF_ADDRESS_OFFSET         0
#define SUB_MSDU_INFO_BUFF_OFFSET_OFFSET          0
#define SUB_MSDU_INFO_BUFF_LENGTH_OFFSET          16

/* Word Offsets for the fields within the Transmit Descriptor Structure */
#define TX_DSCR_NEXT_ADDR_WORD_OFFSET                WORD_OFFSET_4
#define BAR_TX_DSCR_FIELD_SW_INFO_BAR_WORD_OFFSET    WORD_OFFSET_12

/* Miscellaneous Constants */
#define MPDU_STATUS_MASK                          0x03

/* Key types for broad cast and multicast frames */
#define UCAST_KEY_TYPE                            1
#define BCAST_KEY_TYPE                            0

#ifdef BURST_TX_MODE
#define BURST_TX_MODE_FRAME_LEN                   1024
#endif /* BURST_TX_MODE */

#define DEFAULT_FRAG_STATUS                       0xFFFFFFFC

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct {
    UWORD8   sa[6];
    UWORD8   da[6];
    UWORD8   *data;
    UWORD8   *buffer_addr;
    UWORD8   priority;
    UWORD8   service_class;
    UWORD16  data_len;
    BOOL_T   ignore_port;
    BOOL_T   dont_aggr;
    BOOL_T   added_to_q;
    BOOL_T   min_basic_rate;
    UWORD8   ethernet_sa_addr[6];
    UWORD16  eth_type;
} wlan_tx_req_t;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Per fragment status in Tx descriptor */
typedef enum {PENDING_MPDU = 0,
              NORMAL_RETRY,
              BLOCK_ACK_REQD,
              NON_PENDING_MPDU
} TX_DSCR_FRAG_STATUS_T;

/* Type of Tx Descriptor status */
typedef enum {INVALID = 0,
              PENDING,
              NOT_PENDING,
              TX_TIMEOUT
} TX_DSCR_STATUS_T;

/* Different type of ACK policies. */
typedef enum {NORMAL_ACK = 0,
              NO_ACK,
              NO_EXPLICIT_ACK,
              BLOCK_ACK,
              BCAST_NO_ACK,
              COMP_BLOCK_ACK,
              COMP_BAR,
              NUM_ACK_POLICY
} TX_DSCR_ACK_POLICY_T;

/* Different type of Operational flags */
typedef enum {NULL_OP_FLAGS = 0,
              TXOP_PROT_ENABLE_OP_FLAG
} TX_DSCR_OP_FLAGS_T;

/* Priority given to IP packets after classifying. These are set to priority */
/* values currently unspecified in the standard. The mapping for this to the */
/* EDCA queues is shown below.                                               */
typedef enum {PRIORITY_0 = 1, /* Mapped to AC_BK_Q */
              PRIORITY_1 = 0, /* Mapped to AC_BE_Q */
              PRIORITY_2 = 4, /* Mapped to AC_VI_Q */
              PRIORITY_3 = 6  /* Mapped to AC_VO_Q */
} IP_PKT_PRIORITY_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   set_tx_params(UWORD8 *tx_dscr, UWORD8 tr, UWORD8 pr, UWORD8 ap,
                            UWORD32 ptm, UWORD32 *retry_rate_set);
extern UWORD8 *allocate_tx_dscr(mem_handle_t *mem_handle, UWORD8 is_amsdu);
extern UWORD8 *create_default_tx_dscr(UWORD8 is_qos, UWORD8 priority,
                                      UWORD8 is_amsdu);
extern void   set_tx_frame_details(UWORD8 *tx_dscr, UWORD8 *bsa, UWORD8 *msa,
                                   UWORD8 mhlen, UWORD16 frame_len,
                                   UWORD8 data_offset);
extern BOOL_T   tx_mgmt_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num,
                            UWORD8 index);
// 20120709 caisf add, merged ittiam mac v1.2 code
extern void tx_uc_mgmt_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num,
                             UWORD8 idx);

extern void update_tx_mib(UWORD8 *tx_dscr, UWORD8 num_dscr, void *entry);

extern UWORD8 *prepare_control_frame_dscr(UWORD8 *buffer_addr, UWORD16 len,
                                   void *entry, UWORD8 q_num, BOOL_T is_qos,
                                   UWORD8 priority);

extern void set_tx_buffer_details(UWORD8  *tx_dscr, UWORD8 *buffer_addr,
                           UWORD16 hdr_offset, UWORD16 hdr_len,
                           UWORD16 data_len);
extern void free_tx_dscr(UWORD32 *tx_dscr);
extern BOOL_T tx_msdu_frame(UWORD8 *entry, UWORD8 *da, UWORD8 priority,
                            UWORD8 q_num, UWORD8 *tx_dscr);

#ifdef BURST_TX_MODE
extern void tx_burst_mode_frame(UWORD8 *buffer_addr, UWORD16 len, UWORD8 q_num,
                                UWORD8 idx);
extern UWORD16 prepare_burst_mode_frame(UWORD8 *data, UWORD16 len);
extern void initiate_burst_tx_mode(mac_struct_t* mac);
extern void init_mac_fsm_bt(mac_struct_t*);
#endif /* BURST_TX_MODE */

#ifdef TROUT_WIFI_NPI
extern int user_tx_rate;

extern int trout_rf_test_send_pkt(UWORD8 *pkt, UWORD16 len, UWORD8 tx_rate);
extern int trout_rf_test_send_pkt_hugh(UWORD8 *pkt, UWORD16 len, UWORD8 tx_rate);
extern int trout_b2b_pkt_from_host(int pkt_len);
#define NPI_TX_PKT_NUM 5   //must smaller than 5

#endif


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Functions to get and set various fields in the transmit descriptor        */
/*****************************************************************************/

/*          +------------------------------------------------------------+   */
/* WORD0    |Status/QoS|ACKPol|OpFlags|SMPS  |DataRate0|H/w Runtime flags|   */
/*          +------------------------------------------------------------+   */
/*          |8 bits    |4 bits|2 bits |2 bits|8 bits   |8 bits           |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_status(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_STATUS_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}

INLINE UWORD8 get_tx_dscr_status_and_qos_details(UWORD32 *pkt_dscr_ptr)
{
    UWORD8 temp = get_dscr(TX_DSCR_FIELD_STATUS_OFFSET, FIELD_WIDTH_8,
                           pkt_dscr_ptr + WORD_OFFSET_0);

    return temp;
}

INLINE UWORD8 get_tx_dscr_priority(UWORD32 *pkt_dscr_ptr)
{
    UWORD8 temp = get_dscr(TX_DSCR_FIELD_PRIORITY_OFFSET, FIELD_WIDTH_4,
                           pkt_dscr_ptr + WORD_OFFSET_0);

    return temp;
}


INLINE UWORD32 get_tx_dscr_status(UWORD32 *pkt_dscr_ptr)
{
    UWORD8 temp = get_dscr(TX_DSCR_FIELD_STATUS_OFFSET, FIELD_WIDTH_8,
                           pkt_dscr_ptr + WORD_OFFSET_0);

    return ((temp & 0x60) >> 5);
}

INLINE void set_tx_dscr_ack_policy(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_ACK_POLICY_OFFSET, FIELD_WIDTH_4,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}

INLINE UWORD8 get_tx_dscr_ack_policy(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_ACK_POLICY_OFFSET, FIELD_WIDTH_4,
                    pkt_dscr_ptr + WORD_OFFSET_0);
}

INLINE void set_tx_dscr_op_flags(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_OP_FLAGS_OFFSET, FIELD_WIDTH_2,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}

INLINE UWORD8 get_tx_dscr_op_flags(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_OP_FLAGS_OFFSET, FIELD_WIDTH_2,
                    pkt_dscr_ptr + WORD_OFFSET_0);
}


INLINE void set_tx_dscr_smps_mode(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_SMPS_MODE_OFFSET, FIELD_WIDTH_2,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}

INLINE UWORD8 get_tx_dscr_smps_mode(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_SMPS_MODE_OFFSET, FIELD_WIDTH_2,
                    pkt_dscr_ptr + WORD_OFFSET_0);
}

INLINE void set_tx_dscr_data_rate_0(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_DATA_RATE_0_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}

INLINE UWORD8 get_tx_dscr_data_rate_0(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_DATA_RATE_0_OFFSET, FIELD_WIDTH_8,
                    pkt_dscr_ptr + WORD_OFFSET_0);
}

// 20120709 caisf masked, merged ittiam mac v1.2 code
/*
INLINE void set_tx_dscr_hwrt_flags(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_HW_RUNTIME_FLAGS_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_0), value);
}
*/

/*          +------------------------------------------------------------+   */
/* WORD1    |Tx Service field        | Tx Power Level   | RA LUT Index   |   */
/*          +------------------------------------------------------------+   */
/*          |16 bits                 | 8 bits           | 8 bits         |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_tx_service_field(UWORD32 *pkt_dscr_ptr, UWORD32 value) //dumy add 0816
{
    set_dscr(TX_DSCR_SERVICE_FIELD_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_1), value);
}

INLINE void set_tx_dscr_tx_pow_level(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_TX_POWER_LEVEL_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_1), value);
}

INLINE void set_tx_dscr_ra_lut_index(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_RA_LUT_INDEX_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_1), value);
}

/*          +------------------------------------------------------------+   */
/* WORD2    |PHY Tx mode                                                 |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_phy_tx_mode(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_PHY_TX_MODE_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_2), value);
}

INLINE UWORD32 get_tx_dscr_phy_tx_mode(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_PHY_TX_MODE_OFFSET, FIELD_WIDTH_32,
                           (pkt_dscr_ptr + WORD_OFFSET_2));
}

/* This function updates the Channel Bandwidth and Channel Mask in the */
/* specified TX-dscriptor                                              */
INLINE void update_tx_dscr_chan_bw(UWORD32 *tx_dscr, UWORD8 ch_mask,
                                   UWORD8 ch_band)
{
    UWORD32 ptm = get_tx_dscr_phy_tx_mode(tx_dscr);

    /* Update the Channel bandwidth and Channel Mask Fields */
    ptm = (ptm & ~0x78) | ((ch_band & 0x3) << 5) | ((ch_mask & 0x3) << 3);

    set_tx_dscr_phy_tx_mode(tx_dscr, ptm);
}

/*          +------------------------------------------------------------+   */
/* WORD3    |Cipher|Key type| TX STA Addr Index| TSF Time Stamp          |   */
/*          +------------------------------------------------------------+   */
/*          |4 bits|4 bits  |8 bits            | 16 bits                 |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_cipher_type(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_CIPHER_TYPE_OFFSET, FIELD_WIDTH_4,
             (pkt_dscr_ptr + WORD_OFFSET_3), value);
}

INLINE UWORD8 get_tx_dscr_cipher_type(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_CIPHER_TYPE_OFFSET, FIELD_WIDTH_4,
                    pkt_dscr_ptr + WORD_OFFSET_3);
}

INLINE void set_tx_dscr_key_type(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_KEY_TYPE_OFFSET, FIELD_WIDTH_4,
             (pkt_dscr_ptr + WORD_OFFSET_3), value);
}

INLINE void set_tx_dscr_key_index(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_KEY_INDEX_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_3), value);
}

INLINE void set_tx_dscr_tsf_ts(UWORD32 *pkt_dscr_ptr, UWORD16 value)
{
    set_dscr(TX_DSCR_FIELD_TSF_TIME_STAMP_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_3), value);
}

INLINE UWORD16 get_tx_dscr_tsf_ts(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_TSF_TIME_STAMP_OFFSET, FIELD_WIDTH_16,
                    (pkt_dscr_ptr + WORD_OFFSET_3));
}

/*          +------------------------------------------------------------+   */
/* WORD4    | Pointer to Next Packet Descriptor                          |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_next_addr(UWORD32 *dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_NEXT_ADDR_OFFSET, FIELD_WIDTH_32,
             (dscr_ptr + TX_DSCR_NEXT_ADDR_WORD_OFFSET), virt_to_phy_addr(value));
}

INLINE UWORD32 get_tx_dscr_next_addr(UWORD32 *dscr_ptr)
{
    UWORD32 temp = get_dscr(TX_DSCR_NEXT_ADDR_OFFSET, FIELD_WIDTH_32,
                            (dscr_ptr + TX_DSCR_NEXT_ADDR_WORD_OFFSET));

    return phy_to_virt_addr(temp);
}

//add by Hugh
INLINE void set_tx_dscr_host_dscr_addr(UWORD32 *dscr_ptr, UWORD32 value)
{
    set_dscr(0, FIELD_WIDTH_32, (dscr_ptr + WORD_OFFSET_35), value);
}

INLINE UWORD32 get_tx_dscr_host_dscr_addr(UWORD32 *dscr_ptr)
{
    return get_dscr(0, FIELD_WIDTH_32, (dscr_ptr + WORD_OFFSET_35));
}

/*          +------------------------------------------------------------+   */
/* WORD5    | Fragment Status                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_frag_status_summary(UWORD32 *dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_FRAG_STATUS_SUMMARY_OFFSET, FIELD_WIDTH_32,
             (dscr_ptr + WORD_OFFSET_5), value);
}

INLINE UWORD32 get_tx_dscr_frag_status_summary(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_FRAG_STATUS_SUMMARY_OFFSET, FIELD_WIDTH_32,
                    (pkt_dscr_ptr + WORD_OFFSET_5));
}

INLINE UWORD32 get_tx_dscr_frag_status(UWORD32 *pkt_dscr_ptr, UWORD8 frag_num)
{
    return get_dscr(30 - 2 * frag_num, FIELD_WIDTH_2,
                    (pkt_dscr_ptr + WORD_OFFSET_5));
}

/*          +------------------------------------------------------------+   */
/* WORD6:21 |Frag status | Frag offset (1:16)  |Frag length (1:16)       |   */
/*          +------------------------------------------------------------+   */
/*          |2 bits      |14 bits              |16 bits                  |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_frag_offset(UWORD32 *pkt_dscr_ptr, UWORD16 value,
                                    UWORD8 frag_num)
{
    set_dscr(TX_DSCR_FIELD_FRAG_OFFSET_OFFSET, FIELD_WIDTH_12,
             (pkt_dscr_ptr + WORD_OFFSET_6 + frag_num), value);
}

INLINE UWORD32 get_tx_dscr_frag_offset(UWORD32 *pkt_dscr_ptr, UWORD8 frag_num)
{
    return get_dscr(TX_DSCR_FIELD_FRAG_OFFSET_OFFSET, FIELD_WIDTH_12,
                    (pkt_dscr_ptr + WORD_OFFSET_6 + frag_num));
}

INLINE void set_tx_dscr_frag_len(UWORD32 *pkt_dscr_ptr, UWORD16 value,
                                 UWORD8 frag_num)
{
    set_dscr(TX_DSCR_FIELD_FRAG_LEN_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_6 + frag_num), value);
}

INLINE UWORD32 get_tx_dscr_frag_len(UWORD32 *pkt_dscr_ptr, UWORD8 frag_num)
{
    return get_dscr(TX_DSCR_FIELD_FRAG_LEN_OFFSET, FIELD_WIDTH_16,
                    (pkt_dscr_ptr + WORD_OFFSET_6 + frag_num));
}

/*          +------------------------------------------------------------+   */
/* WORD22:25| Frag K+3 idx | Frag K+2 idx  | Frag K+1 idx | Frag K index |   */
/*          +------------------------------------------------------------+   */
/*          |   8 bits     |     8 bits    |     8 bits   |    8 bits    |   */
/*          +------------------------------------------------------------+   */

INLINE UWORD8 get_sub_msdu_index(UWORD32 *tx_dscr, UWORD16 frag_num)
{
    UWORD16 field_offset = (3 - (frag_num & 0x3)) << 3;
    UWORD16 word_offset  = frag_num >> 2;

    return get_dscr(field_offset, FIELD_WIDTH_8,
                    (tx_dscr + WORD_OFFSET_22 + word_offset));
}

/*          +------------------------------------------------------------+   */
/* WORD26   |  Long Cnt | Short Cnt  | Short Retries | Long Retries      |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits    | 8 bits     | 8 bits        | 8 bits            |   */
/*          +------------------------------------------------------------+   */

INLINE UWORD32 get_tx_dscr_num_short_retry(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_NUM_SHORT_RETRIES_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_26));
}

INLINE UWORD32 get_tx_dscr_num_long_retry(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_NUM_LONG_RETRIES_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_26));
}

INLINE UWORD32 get_tx_dscr_short_retry_count(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_SHORT_RETRY_COUNT_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_26));
}

INLINE UWORD32 get_tx_dscr_long_retry_count(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_LONG_RETRY_COUNT_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_26));
}

/* This function resets all H/w statistics present in the TX-Dscr */
INLINE void reset_tx_dscr_stats(UWORD32 *pkt_dscr_ptr)
{
    set_dscr(0, FIELD_WIDTH_32, (pkt_dscr_ptr + WORD_OFFSET_26), 0);
    set_dscr(TX_DSCR_FIELD_CTS_FAILURE_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_27), 0);
}

/*          +------------------------------------------------------------+   */
/* WORD27   |     Frame length       | CTS Failures  | RTS Success       |   */
/*          +------------------------------------------------------------+   */
/*          |         16 bits        | 8 bits        | 8 bits            |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_frame_len(UWORD32 *pkt_dscr_ptr, UWORD16 value)
{
    set_dscr(TX_DSCR_FIELD_FRAME_LEN_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_27), value);
}

INLINE UWORD16 get_tx_dscr_frame_len(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_FRAME_LEN_OFFSET, FIELD_WIDTH_16,
                    (pkt_dscr_ptr + WORD_OFFSET_27));
}

INLINE UWORD32 get_tx_dscr_rts_success(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_RTS_SUCCESS_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_27));
}

INLINE UWORD32 get_tx_dscr_cts_failure(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_CTS_FAILURE_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_27));
}

/*          +------------------------------------------------------------+   */
/* WORD28   | IV MS Word (63-32)                                         |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE UWORD32 get_tx_dscr_iv32h(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_IV_MS_WORD_OFFSET, FIELD_WIDTH_32,
                    pkt_dscr_ptr + WORD_OFFSET_28);
}

/*          +------------------------------------------------------------+   */
/* WORD29   | IV MS Word (31-0)                                          |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_iv32(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_IV_MS_WORD_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_29), value);
}

INLINE UWORD32 get_tx_dscr_iv32l(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_IV_MS_WORD_OFFSET, FIELD_WIDTH_32,
                    pkt_dscr_ptr + WORD_OFFSET_29);
}

/*          +------------------------------------------------------------+   */
/* WORD30   | Buffer Start Address                                       |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_buffer_addr(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_BUFFER_START_ADDR_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_30), virt_to_phy_addr(value));
}

INLINE UWORD32 get_tx_dscr_buffer_addr(UWORD32 *pkt_dscr_ptr)
{
    UWORD32 temp = get_dscr(TX_DSCR_FIELD_BUFFER_START_ADDR_OFFSET,
                            FIELD_WIDTH_32, (pkt_dscr_ptr + WORD_OFFSET_30));

    return phy_to_virt_addr(temp);
}

/*          +------------------------------------------------------------+   */
/* WORD31   | Pointer to sub-MSDU information table                      |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_submsdu_info(UWORD32 *tx_dscr, UWORD32 value)
{
    set_dscr(TX_DSCR_SUBMSDU_INFO_OFFSET, FIELD_WIDTH_32,
             (tx_dscr + WORD_OFFSET_31), virt_to_phy_addr(value));
}

INLINE UWORD32 get_tx_dscr_submsdu_info(UWORD32 *tx_dscr)
{
    UWORD32 temp = get_dscr(TX_DSCR_SUBMSDU_INFO_OFFSET, FIELD_WIDTH_32,
                            (tx_dscr + WORD_OFFSET_31));

    return phy_to_virt_addr(temp);
}

/*          +------------------------------------------------------------+   */
/* WORD32   | MAC Header Offset | MAC Header len | Num sub-MSDU |Q Number|   */
/*          +------------------------------------------------------------+   */
/*          |8 bits             |8 bits          |8 bits        | 8 bits |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_mh_offset(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_MAC_HEADER_OFFSET_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_32), value);
}

INLINE UWORD8 get_tx_dscr_mh_offset(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_MAC_HEADER_OFFSET_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_32));
}

INLINE void set_tx_dscr_mh_len(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_MAC_HEADER_LEN_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_32), value);
}

INLINE UWORD8 get_tx_dscr_mh_len(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_MAC_HEADER_LEN_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_32));
}

INLINE void set_tx_dscr_num_submsdu(UWORD32 *tx_dscr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_NUM_SUB_MSDU_OFFSET, FIELD_WIDTH_8,
             (tx_dscr + WORD_OFFSET_32), value);

}

INLINE UWORD8 get_tx_dscr_num_submsdu(UWORD32 *tx_dscr)
{
    return get_dscr(TX_DSCR_FIELD_NUM_SUB_MSDU_OFFSET, FIELD_WIDTH_8,
                    (tx_dscr + WORD_OFFSET_32));
}

INLINE void set_tx_dscr_q_num(UWORD32 *pkt_dscr_ptr, UWORD8 value)
{
    set_dscr(TX_DSCR_FIELD_Q_NUM_OFFSET, FIELD_WIDTH_8,
             (pkt_dscr_ptr + WORD_OFFSET_32), value);
}

INLINE UWORD8 get_tx_dscr_q_num(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_Q_NUM_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_32));
}

/*          +------------------------------------------------------------+   */
/* WORD33   | Data Rate 4  | Data Rate 3  | Data Rate 2   | Data Rate 1  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_retry_rate_set1(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_RETRY_RATE_SET1_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_33), value);
}

/*          +------------------------------------------------------------+   */
/* WORD34   | TSSI value   | Data Rate 7  | Data Rate 6   | Data Rate 5  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */

INLINE void set_tx_dscr_retry_rate_set2(UWORD32 *pkt_dscr_ptr, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_RETRY_RATE_SET2_OFFSET, FIELD_WIDTH_24,
             (pkt_dscr_ptr + WORD_OFFSET_34), value);
}

/*          +------------------------------------------------------------+   */
/* WORD33   | Data Rate 4  | Data Rate 3  | Data Rate 2   | Data Rate 1  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */

INLINE UWORD32 get_tx_dscr_retry_rate_set1(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_RETRY_RATE_SET1_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_33));
}

/*          +------------------------------------------------------------+   */
/* WORD34   | TSSI value   | Data Rate 7  | Data Rate 6   | Data Rate 5  |   */
/*          +------------------------------------------------------------+   */
/*          | 8 bits       | 8 bits       | 8 bits        | 8 bits       |   */
/*          +------------------------------------------------------------+   */

INLINE UWORD32 get_tx_dscr_retry_rate_set2(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_RETRY_RATE_SET2_OFFSET, FIELD_WIDTH_24,
             (pkt_dscr_ptr + WORD_OFFSET_34));
}

INLINE UWORD8 get_tx_dscr_tssi_value(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(TX_DSCR_FIELD_TSSI_VALUE_OFFSET, FIELD_WIDTH_8,
                    (pkt_dscr_ptr + WORD_OFFSET_34));
}

INLINE void set_frag_status(UWORD32 *pkt_dscr_ptr, UWORD8 frag_num, UWORD32 value)
{
    set_dscr(TX_DSCR_FIELD_FRAG_STATUS_OFFSET, FIELD_WIDTH_32,
             (pkt_dscr_ptr + WORD_OFFSET_6 + frag_num), value);

}
// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE void set_tx_dscr_word_offset(UWORD32 *pkt_dscr_ptr, UWORD32 word_offset,
                                    UWORD32 value)
{
    set_dscr(0, FIELD_WIDTH_32, (pkt_dscr_ptr + word_offset), value);

}

/*****************************************************************************/
/* Miscellaneous functions involving descriptor processing                   */
/*****************************************************************************/

/* This function gets the fragment address of a given fragment  */
INLINE UWORD32 get_tx_dscr_frag_addr(UWORD32 *pkt_dscr_ptr, UWORD8 frag)
{
    return (get_tx_dscr_buffer_addr(pkt_dscr_ptr) +
            get_tx_dscr_frag_offset(pkt_dscr_ptr, frag));
}

/* This function gets the MAC header address  */
INLINE UWORD32 get_tx_dscr_mac_hdr_addr(UWORD32 *pkt_dscr_ptr)
{
    return (get_tx_dscr_buffer_addr(pkt_dscr_ptr) +
            get_tx_dscr_mh_offset(pkt_dscr_ptr));
}

/* This function gets the frame length  */
INLINE UWORD16 get_tx_frame_len(UWORD32 *dscr)
{
    UWORD16 frame_len   = get_tx_dscr_frame_len(dscr);
    UWORD32 data_offset = get_tx_dscr_frag_offset(dscr, 0);
    UWORD8  hdr_len     = get_tx_dscr_mh_len(dscr);

    return (frame_len + data_offset - hdr_len);
}

/* This function sets the security parameters */
INLINE void set_tx_security(UWORD8 *tx_dscr, UWORD8 ct, UWORD8 kt, UWORD8 ki)
{
    set_tx_dscr_cipher_type((UWORD32 *)tx_dscr, ct);
    set_tx_dscr_key_type((UWORD32 *)tx_dscr, kt);
    set_tx_dscr_key_index((UWORD32 *)tx_dscr, ki);
}

/* This function sets the sub-MSDU buffer description in the sub-MSDU */
/* information table for the specified sub-MSDU                       */
INLINE void set_tx_dscr_submsdu_buff_info(UWORD32 *tx_dscr, UWORD32 value,
                                     UWORD32 submsdu_idx)
{
    buffer_desc_t *buff_desc     = (buffer_desc_t *)value;
    UWORD32       word_offset    = submsdu_idx << 1;
    UWORD32       *sub_msdu_info = (UWORD32 *)get_tx_dscr_submsdu_info(tx_dscr);

    set_dscr(SUB_MSDU_INFO_BUFF_ADDRESS_OFFSET, FIELD_WIDTH_32,
             (sub_msdu_info + word_offset),
             virt_to_phy_addr((UWORD32)(buff_desc->buff_hdl)));

    set_dscr(SUB_MSDU_INFO_BUFF_OFFSET_OFFSET, FIELD_WIDTH_16,
             (sub_msdu_info + word_offset + 1),
             buff_desc->data_offset);

    set_dscr(SUB_MSDU_INFO_BUFF_LENGTH_OFFSET, FIELD_WIDTH_16,
             (sub_msdu_info + word_offset + 1),
             buff_desc->data_length);
}

/* This function fetches the sub-MSDU buffer handle */
INLINE UWORD32 get_tx_dscr_submsdu_buff_hdl(UWORD32 *tx_dscr, UWORD32 submsdu_idx)
{
    UWORD32 word_offset    = submsdu_idx << 1;
    UWORD32 *sub_msdu_info = (UWORD32 *)get_tx_dscr_submsdu_info(tx_dscr);
    UWORD32 temp           = 0;

    temp = get_dscr(SUB_MSDU_INFO_BUFF_ADDRESS_OFFSET, FIELD_WIDTH_32,
                    (sub_msdu_info + word_offset));

    return phy_to_virt_addr(temp);
}

/* This function sets the TSF timestamp in the Tx descriptor */
INLINE void update_tx_dscr_tsf_ts(UWORD32 *tx_dscr)
{
    UWORD32 tsf_lo   = 0;

    tsf_lo = get_machw_tsf_timer_lo();

    /* Set the current timestamp */
    set_tx_dscr_tsf_ts((UWORD32 *)tx_dscr, (UWORD16)((tsf_lo >> 10) & 0xFFFF));
}

//add by chengwg.
INLINE void simp_update_tx_dscr_tsf_ts(UWORD32 *tx_dscr, UWORD32 tsf_lo)
{
    /* Set the current timestamp */
    set_tx_dscr_tsf_ts((UWORD32 *)tx_dscr, (UWORD16)((tsf_lo >> 10) & 0xFFFF));
}


/* This function sets the sub-MSDU table pointers and updates the other */
/* relevant parameters in the Tx-descriptor.                            */
INLINE void set_tx_submsdu_info(UWORD8 *tx_dscr, buffer_desc_t *buff_list,
                                UWORD16 num_buff, UWORD16 mac_hdr_len)
{
    UWORD16 indx = 0;
    UWORD16 frame_len = 0;

    /* Set the length of the entire frame (including MAC header and FCS) */
    frame_len = buff_list[0].data_length + mac_hdr_len + FCS_LEN;

    /* Set the frame length in the Tx Descriptor */
    set_tx_dscr_frame_len((UWORD32 *)tx_dscr, frame_len);
    set_tx_dscr_num_submsdu((UWORD32 *)tx_dscr, num_buff);
    set_tx_dscr_frag_len((UWORD32 *)tx_dscr, frame_len, 0);
    set_tx_dscr_frag_offset((UWORD32 *)tx_dscr, 0, 0);

#ifdef DEBUG_KLUDGE
    /* Set the data buffer related information in the Tx-descriptor */
    for(indx = 0; indx < num_buff; indx++)
    {
        set_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr,
                                      (UWORD32)(&(buff_list[indx])),
                                      indx);
    }
#else  /* DEBUG_KLUDGE */
    /* Set the data buffer related information in the Tx-descriptor */
    /* This Kludge fixes the TX-Hang issue in MAC H/w */
    for(indx = 0; indx < num_buff; indx++)
    {
        buffer_desc_t buffer_desc = buff_list[indx];
        /* This Kludge fixes the TX-Hang issue in MAC H/w */
        if(indx == (num_buff - 1))
            buffer_desc.data_length += 4;

        set_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr,
                                      (UWORD32)(&buffer_desc),
                                      indx);
    }
#endif /* DEBUG_KLUDGE */
}

/* This function returns the buffer details of the required sub-MSDU */
INLINE void get_tx_dscr_submsdu_buff_info(UWORD32 *tx_dscr, UWORD32 value,
                                          UWORD32 submsdu_idx)
{
    buffer_desc_t *buff_desc     = (buffer_desc_t *)value;
    UWORD32       word_offset    = submsdu_idx << 1;
    UWORD32       *sub_msdu_info = (UWORD32 *)get_tx_dscr_submsdu_info(tx_dscr);
    UWORD32       temp           = 0;

    temp = get_dscr(SUB_MSDU_INFO_BUFF_ADDRESS_OFFSET, FIELD_WIDTH_32,
                                      (sub_msdu_info + word_offset));

    buff_desc->buff_hdl    = (UWORD8 *)phy_to_virt_addr(temp);

    buff_desc->data_offset = get_dscr(SUB_MSDU_INFO_BUFF_OFFSET_OFFSET,
                                      FIELD_WIDTH_16,
                                      (sub_msdu_info + word_offset + 1));

    buff_desc->data_length = get_dscr(SUB_MSDU_INFO_BUFF_LENGTH_OFFSET,
                                      FIELD_WIDTH_16,
                                      (sub_msdu_info + word_offset + 1));
}

/* This function updates the packet status in the Tx-Descriptor */
INLINE void update_tx_dscr_pkt_status(UWORD32 *tx_dscr,
                                      TX_DSCR_STATUS_T pkt_status)
{
    UWORD8 status = get_tx_dscr_status_and_qos_details(tx_dscr);

    status = (status & 0x9F) | ((pkt_status << 5) & 0x60);
    set_tx_dscr_status((UWORD32 *)tx_dscr, status);
}

/* This function checks for transmit failure. */
INLINE BOOL_T is_tx_failure(UWORD32 *tx_dscr)
{
    /* Tx failure can happen due to either timeout of TxLifetime-Timer or */
    /* retry counter reaching Short/Long Retry Limit.             */
    if((get_tx_dscr_status((UWORD32 *)tx_dscr) == TX_TIMEOUT) ||
       (get_tx_dscr_short_retry_count((UWORD32 *)tx_dscr) >=  mget_ShortRetryLimit()) ||
       (get_tx_dscr_long_retry_count((UWORD32 *)tx_dscr)  >= mget_LongRetryLimit()))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks for transmit failure. */
INLINE BOOL_T is_tx_success(UWORD32 *tx_dscr)
{
    /* Tx failure can happen due to either timeout of TxLifetime-Timer or */
    /* retry counter reaching Short/Long Retry Limit.             */
    if((get_tx_dscr_status(tx_dscr) == NOT_PENDING) &&
       (~(get_tx_dscr_frag_status_summary(tx_dscr)) == 0))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks whether the buffer start address passed points to    */
/* some location within the corresponding TX-Dscr. This situation arises when*/
/* the transmit frame is created within the TX-Dscr itself.                  */
INLINE BOOL_T is_buffer_in_tx_dscr(UWORD8 *tx_dscr, UWORD8 *buff_hdl)
{
    if(((UWORD32)buff_hdl - (UWORD32)tx_dscr) < TX_DSCR_LEN)
        return BTRUE;

    return BFALSE;
}

/* This function resets all fields in the TX-Dscr which are not updated by */
/* S/w and which is required to be set to Zero by the H/w.                 */
INLINE void reset_txdscr_fields(UWORD32 *tx_dscr)
{
    /* Fragment0 status (FL + Offset + Length) is cleared to Zero. This is   */
    /* fix the issue of more fragment bit being set for an MSDU that is less */
    /* than fragmentation threshold following a MSDU of length greater than  */
    /* the  fragmentation threshold. FL field of MSDU1 (>fragThres) was being*/
    /* used to set more fragment bit for MSDU2(<fragThres). This scenario is */
    /* is typically seen with pings of length greater than 1472 bytes.       */
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
    set_frag_status((UWORD32 *)tx_dscr, 0, 0);
    reset_tx_dscr_stats(tx_dscr);
    set_tx_dscr_hwrt_flags(tx_dscr, 0);
}
#else
    /* Reset Frag Status field word */
    set_tx_dscr_word_offset((UWORD32 *)tx_dscr, WORD_OFFSET_6, 0);
    /* Reset Tx Dscr H/w Statistics fields */
    set_tx_dscr_word_offset((UWORD32 *)tx_dscr, WORD_OFFSET_26, 0);
    set_tx_dscr_word_offset((UWORD32 *)tx_dscr, WORD_OFFSET_27, 0);
    /* Reset HW Runtime flags field in Tx Dscr */
    set_tx_dscr_word_offset((UWORD32 *)tx_dscr, WORD_OFFSET_0, 0);
    /* Reset Frag0MSDUIndex field in Tx Dscr */
    set_tx_dscr_word_offset((UWORD32 *)tx_dscr, WORD_OFFSET_22, 0);
#endif
}

/* This function frees the TX descriptor buffer/s from the given memory pool */
/* The sub-MSDU information table may be present in a second TX descriptor   */
/* buffer or in the same TX descriptor buffer at an offset of TX_DSCR_LEN.   */
/* The sub-MSDU information table pointer is extracted from the given TX     */
/* descriptor and compared with the descriptor address + TX_DSCR_LEN to find */
/* if a separate buffer is being used for the sub-MSDU information table. If */
/* so this buffer is also freed in addition to the TX descriptor buffer.     */
/* Note that it is assumed that the TX descriptor and  sub-MSDU information  */
/* table buffers must be allocated from the same memory pool.                */
INLINE void free_tx_dscr_buffer(mem_handle_t *mem_handle, UWORD8 *tx_dscr)
{
    int ret;
#ifndef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
	/* Get the sub-MSDU info table pointer from the TX descriptor */
    UWORD32 sub_msdu_info = (UWORD32)get_tx_dscr_submsdu_info((UWORD32*)tx_dscr);

    /* Free the sub-MSDU information table buffer if it is a separate buffer */
    if(sub_msdu_info != ((UWORD32)(tx_dscr + TX_DSCR_LEN)))
		mem_free(mem_handle, (void *)sub_msdu_info);
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

    /* Free the Tx descriptor buffer */
    ret = mem_free(mem_handle, (void *)tx_dscr);
}

/* This function adds a user to the the TX descriptor buffer/s from given    */
/* memory pool. The sub-MSDU information table may be present in a second TX */
/* descriptor buffer or in the same TX descriptor buffer at an offset of     */
/* TX_DSCR_LEN. The sub-MSDU information table pointer is extracted from the */
/* given TX descriptor and compared with descriptor address + TX_DSCR_LEN to */
/* find if a separate buffer is being used for the sub-MSDU information      */
/* table. If so a user is also added to this buffer in addition to the TX    */
/* descriptor buffer.                                                        */
/* Note that it is assumed that the TX descriptor and  sub-MSDU information  */
/* table buffers must be allocated from the same memory pool.                */
INLINE void add_user_tx_dscr_buffer(mem_handle_t *mem_handle, UWORD8 *tx_dscr)
{
#ifndef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
	/* Get the sub-MSDU info table pointer from the TX descriptor */
    UWORD32 sub_msdu_info = (UWORD32)get_tx_dscr_submsdu_info((UWORD32*)tx_dscr);

    /* Add a user to the sub-MSDU information table buffer if it is a        */
    /* separate buffer                                                       */
    if(sub_msdu_info != ((UWORD32)(tx_dscr + TX_DSCR_LEN)))
		mem_add_users(mem_handle, (void *)sub_msdu_info, 1);
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

    /* Add a user to the Tx descriptor buffer */
    mem_add_users(mem_handle, (void *)tx_dscr, 1);
}
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef LOCALMEM_TX_DSCR
/* This function updates the Buffer address of Block ACK-Request Tx          */
/* descriptor when Tx descriptors are created in local memory                */
INLINE void update_bar_buff_addr_txdscr(UWORD32 *tx_dscr, UWORD32 *sub_msdu_info)
{
    UWORD32 *buff_hdl = (tx_dscr + BAR_TX_DSCR_FIELD_SW_INFO_BAR_WORD_OFFSET);

    set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)buff_hdl);
    set_dscr(SUB_MSDU_INFO_BUFF_ADDRESS_OFFSET, FIELD_WIDTH_32,
             (sub_msdu_info),
             virt_to_phy_addr((UWORD32)(buff_hdl)));
}
#endif /* LOCALMEM_TX_DSCR */
#endif /* TRANSMIT_H */
