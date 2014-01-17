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
/*  File Name         : blockack.h                                           */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      definitions for 802.11e mode                         */
/*                                                                           */
/*  List of Functions : seqno_leq                                            */
/*                      seqno_lt                                             */
/*                      seqno_geq                                            */
/*                      seqno_gt                                             */
/*                      seqno_bound_chk                                      */
/*                      stop_ba_timer                                        */
/*                      del_ba_timer                                         */
/*                      start_addba_timer                                    */
/*                      restart_ba_timer                                     */
/*                      start_ba_pend_timer                                  */
/*                      get_ba_start_seq_num                                 */
/*                      get_bar_start_seq_num                                */
/*                      reset_ba_handle                                      */
/*                      initialize_ba                                        */
/*                      init_ba_handle                                       */
/*                      reorder_ba_rx_buffer_data                            */
/*                      reorder_ba_rx_buffer_bar                             */
/*                      filter_wlan_ba_serv_cls                              */
/*                      update_serv_class_ba                                 */
/*                      flush_ba_rx_buffer                                   */
/*                      transmit_frames_ba                                   */
/*                      restart_ba                                           */
/*                      update_halted_ba_ctxt                                */
/*                      get_ba_lut_index                                     */
/*                      del_ba_lut_index                                     */
/*                      is_bar_useful                                        */
/*                      is_ba_useful                                         */
/*                      update_addba_req_param_ht                            */
/*                      extract_addba_req_ht_params                          */
/*                      extract_addba_resp_ht_params                         */
/*                      update_ba_start_seq_num                              */
/*                      is_ba_missing_msdu                                   */
/*                      buffer_frame_in_pending_q                            */
/*                      BAR Tx Dscr Access Functions                         */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

#ifndef BLOCKACK_H
#define BLOCKACK_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mib_11e.h"
#include "transmit.h"
#include "receive.h"
#include "mib.h"
#include "mib_11n.h"
#include "index_util.h"
#include "management_11n.h"
#include "metrics.h"
#include "trout_wifi_rx.h"
/*****************************************************************************/
/* COMPILE-FLAG                                                              */
/*****************************************************************************/

/* Define this FLAG to disabled BABAR-SYNC */
//#define DISABLE_BABAR_SYNC

/* Flag to enable workaround for bu with Ralink AP */
//#define WIFI_SW_KLUDGE_RALINK

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/* The BA TX/RX Buffers Macros should be a power of 2. This fact is used for */
/* code optimization.                                                        */
#define MAX_BA_TX_BUFFERS           32  /* Maximum Num TX-Buffers/BA session */
#define MAX_BA_RX_BUFFERS           32  /* Maximum Num RX-Buffers/BA session */

#define MAX_NUM_BA_SESSIONS_SUPP    NUM_BA_SESSION_SUPPORTED
#define MAX_BAR_INTERVAL            16
#define MAX_BA_WINSIZE              64
#define MAX_BA_INACTIVE_TIMEOUT     200
#define MAX_ADDBA_INACTIVE_TIMEOUT  200
#define DEF_BA_GRP_BUFF_TO          100 /* Default grouping buffer time out */
#define MAX_SEQ_NUM_VAL             4096
#define BITMAP_LENGTH               256
#define BUFFER_LIMIT                20 /* Limit for buffering new packets */
#define MAX_SEQNO_BY_TWO            2048
#define SEQNO_MASK                  0x0FFF
#define LESSER_THAN_SEQLO           0
#define GREATER_THAN_SEQHI          1
#define BETWEEN_SEQLO_SEQHI         2
//#define AMSDU_BACK_SUPPORTED_FLAG   0  /* 1=>AMSDU supported with Block Ack */
#define AMSDU_BACK_SUPPORTED_FLAG   1  /*(modify by chengwg) 1=>AMSDU supported with Block Ack */
#define COMPR_BITMAP_LENGTH         16 /* Length of Compressed Bitmap       */
#define BA_LUT_IDX_BMAP_LEN         ((MAX_BA_LUT_SIZE >> 3) + 1)
#define INVALID_BA_LUT_INDEX        MAX_BA_LUT_SIZE

/* The maximum length of the data field within BA frame */
#define MAX_BA_DATA_LEN             12
#define BA_MAC_HDR_LEN              16
#define MAX_BAR_DATA_LEN            20

#define BLOCK_REQ_DATA_LEN			9	//add by chengwg.

#define BA_LOSS_RECOVERY_TIMEOUT    1000
#define BATX_WIN_STALL_THRESHOLD    6

/* BAR Tx Descriptor Format details: The transmit packet descriptor format   */
/* for a Block ACK Request frame is defined to be specific to include        */
/* relevant information for the Block ACK that is received in response to    */
/* the same. The format of this descriptor is shown below (note that only    */
/* the BA related fields are shown here, all other fields are same as the    */
/* existing transmit descriptor)                                             */
/*                                                                           */
/*          +------------------------------------------------------------+   */
/* WORD7    |BA H/w Flags                                                |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD8    |BA SSN        |BA SSN (FN)  |BA Control                     |   */
/*          +------------------------------------------------------------+   */
/*          |12 bits       |4 bits       |16 bits                        |   */
/*          +------------------------------------------------------------+   */
/* WORD9    |BA Bitmap Word 0                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD10   |BA Bitmap Word 1                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
/* WORD11   |S/W BA Information                                          |   */
/*          +------------------------------------------------------------+   */
/*          |BAR Last Sequence Number     |Reserved                      |   */
/*          +------------------------------------------------------------+   */
/*          |16 bits                      |16 bits                       |   */
/*          +------------------------------------------------------------+   */

/* Note that the offsets are defined from the end of each word (Bit 31) */
#define BAR_TX_DSCR_FIELD_BA_HW_FLAGS_OFFSET       0
#define BAR_TX_DSCR_FIELD_BA_SSN_OFFSET            0
#define BAR_TX_DSCR_FIELD_BA_CTRL_OFFSET           16
#define BAR_TX_DSCR_FIELD_BA_BITMAP_WORD0_OFFSET   0
#define BAR_TX_DSCR_FIELD_BA_BITMAP_WORD1_OFFSET   0
#define BAR_TX_DSCR_FIELD_BA_SW_INFO_BARLSN_OFFSET 0

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/

#define SEQNO_ADD(seq1, seq2) (((seq1) + (seq2)) & SEQNO_MASK)
#define SEQNO_SUB(seq1, seq2) (((seq1) - (seq2)) & SEQNO_MASK)

/* Function Macros to check the BA H/w Flags */
#define IS_BA_INFO_PRESENT(flags)  (flags & BIT0)    /* bit 0 is 1 */
#define IS_BA_AMPDU_RSP(flags)     (~(flags & BIT1)) /* bit 1 is 0 */
#define IS_BA_BAR_RSP(flags)       (flags & BIT1)    /* bit 1 is 1 */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum{BA_INIT        = 0,
             BA_INPROGRESS  = 1,
             BA_FAILED      = 2,
             BA_SUSPENDED   = 3,
             BA_HALTED      = 4,
}BA_CONN_STATUS_T;

typedef enum {RECIPIENT=0,
              INITIATOR
} INITIATOR_T;

typedef enum{BASIC_BACK      = 0,
             COMPRESSED_BACK = 2,
             MULTI_TID_BACK  = 3
}BACK_VARIANT_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8   in_use;
    UWORD16  seq_num;
    UWORD32 *dscr;
    UWORD8  num_dscr;
    UWORD16 rx_len;
    UWORD8   *msa;
    CIPHER_T ct;
} bfd_rx_pkt_t;

typedef struct
{
    UWORD8  in_use;
    UWORD16 seq_num;
    UWORD8 *tx_dscr;
} bfd_tx_pkt_t;

typedef struct
{
    UWORD8  *ba;
    UWORD8  tid;
    UWORD8  *entry;
    UWORD8  direction;
    struct work_struct work;  //Hugh: fix accessing trout by SPI/SDIO in timer context.
} ba_alarm_data_t;

typedef struct
{
    BACK_VARIANT_T    back_var;   /* Block-Ack variant */
    UWORD8            amsdu_supp; /* Block-Ack support for AMSDU */
    UWORD8            sta_index;  /* Transmitter Index */
    UWORD8            lut_index;  /* Session H/w LUT Index */
    UWORD8            dialog_token; /* ADDBA Dialog Token */
    UWORD8            ba_policy; /* Immediate=1 Delayed=0 */
    UWORD8            index; /* Delayed BA buffer index */
    UWORD16           buff_size; /* Block-Ack Buffer Size */
    UWORD16           timeout; /* Session Timeout interval */
    UWORD16           win_start;/* Seqno of the first un-Acked MSDU */
    UWORD16           win_end;  /* Seqno of the last MSDU that can be Rxd */
    UWORD16           win_tail; /* Highest Seqno currently present in the Q */
    UWORD16           buff_cnt; /* Num Frames currently in Reorder-Q        */
    UWORD8            *dst_addr; /* Transmitter Address */
    ALARM_HANDLE_T    *ba_alarm; /* Session Timer */
    BOOL_T            is_ba;     /* Session Valid Flag */
    BA_CONN_STATUS_T  conn_status;  /* Session Status Flag */
    ba_alarm_data_t   ba_data; /* Timeout Callback Function Data */
    bfd_rx_pkt_t      buff_list[MAX_BA_RX_BUFFERS]; /* Rx-Reorder Queue */
    UWORD8            rx_pn_val[6]; /* Last Received PN Number */
} ba_rx_struct_t;

typedef struct
{
    BACK_VARIANT_T  back_var; /* Block-Ack variant */
    UWORD8   amsdu_supp;   /* Block-Ack support for AMSDU */
    UWORD8   dialog_token; /* ADDBA Dialog Token */
    UWORD8   ba_policy;  /* Immediate=1 Delayed=0 */
    UWORD8   os_bar_cnt; /* Outstanding BAR count */
    UWORD16  bar_int;    /* Intervals at which BAR should be queued */
    UWORD16  cnt_from_last_bar; /* Number of packets queued from time last BAR was queued */
    UWORD16  buff_size;  /* Block-Ack Buffer Size */
    UWORD16  max_pend;   /* Max number of frames that can be pending when */
                         /* transmission is permitted by Block-Ack window */
    BOOL_T   is_ba;      /* Session Valid Flag */
    UWORD16  timeout;    /* Session Timeout interval */
    UWORD16  num_pend;    /* Number of buffers in Pending-Q */
    UWORD16  pend_limit;  /* Capacity of Pending-Q */
    UWORD16  num_pipe;   /* Number of buffers in the TX pipeline */
    UWORD16  last_seq_num; /* Seq-Num of the last frame txd */
    UWORD16  win_start; /* Seqno of the first un-Acked MSDU */
    UWORD16  win_end;   /* Seqno of the last MSDU that can be Txd */
    UWORD16  buff_timeout;   /* Timeout for the grouping buffer     */
    bfd_tx_pkt_t buff_list[MAX_BA_TX_BUFFERS];  /* Tx-Retry Queue */
    list_buff_t  pend_list; /* Tx-Pending Queue (a.k.a. Grouping Q) */
    BA_CONN_STATUS_T conn_status; /* Session Status Flag */
    ALARM_HANDLE_T *ba_alarm; /* Session Timer */
    ba_alarm_data_t ba_data; /* Timeout Callback Function Data */
    ALARM_HANDLE_T *ba_pend_alarm; /* Grouping Timer */
    UWORD8   pend_alarm_on; /* "Pending Timer is running" Flag */
    UWORD8   sta_index;    /* Receiver Index */
    UWORD8   *dst_addr;    /* Receiver Address */
    UWORD8   bar_frame[MAX_BAR_DATA_LEN]; /* BAR Frame buffer (Body only)*/
    UWORD8   batxwinstall; /* Parameter to detect and recover from window stall */
} ba_tx_struct_t;

typedef struct
{
    UWORD8         imm_ba_capable;
    UWORD8         sta_index;
    UWORD8         dst_addr[6];
    UWORD8         num_ba_tx;
    UWORD8         num_ba_rx;
    ba_rx_struct_t *ba_rx_ptr[NUM_TIDS];
    ba_tx_struct_t *ba_tx_ptr[NUM_TIDS];
} ba_struct_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_ba_lut_index_table[BA_LUT_IDX_BMAP_LEN];
extern UWORD8 g_num_ba_tx_sessions;
extern UWORD8 g_num_ba_rx_sessions;

extern struct mutex ba_rx_mutex;	//add by chengwg 2013-06-21.
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void send_delba(UWORD8 *addr, UWORD8 tid, UWORD8 initiator,
                       UWORD8 reason);
extern void update_addba_list(ba_tx_struct_t **ba, UWORD8 *data);
extern void send_addba_rsp(ba_rx_struct_t **ba, UWORD8 *data, UWORD8 *da);
extern void reset_tx_ba_handle(ba_tx_struct_t **ba, UWORD8 tid);
extern void reset_rx_ba_handle(ba_rx_struct_t **ba, UWORD8 tid);
extern void delba_recp_process(ba_rx_struct_t **ba, UWORD8 *data);
extern void delba_initiator_process(ba_tx_struct_t **ba, UWORD8 *data);
extern BOOL_T is_ba_buff_tx_pkt(ba_struct_t *ba_hdl, UWORD8 q_num, UWORD8 tid,
                                UWORD8 *tx_dscr);
extern UWORD8 dequeue_tx_ba_frame(void *entry, ba_tx_struct_t *ba,
                                UWORD8 num_dq, UWORD8 tid);
extern BOOL_T send_ba_req(ba_tx_struct_t **ba, UWORD8 tid, UWORD8 q_num);

extern UWORD16 send_frames_with_gap(ba_rx_struct_t *ba, UWORD8 tid,
                                    UWORD16 last_seqno);
extern UWORD16 send_frames_in_order(ba_rx_struct_t *ba, UWORD8 tid);
extern void ps_buff_ba_frames(void *entry, ba_tx_struct_t **ba, UWORD8 tid);
extern void handle_bar_tx_failure(void *entry, ba_tx_struct_t **ba,
                                  UWORD8 q_num, UWORD8 tid, UWORD16 lsn);
// 20120709 caisf mod, merged ittiam mac v1.2 code
//extern void handle_ba_tx_comp(UWORD8 *dscr, void *entry);
extern void handle_ba_tx_comp(UWORD8 *dscr, void *entry, UWORD8* msa);

#ifndef OS_LINUX_CSL_TYPE
extern void addba_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void ba_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
extern void ba_pend_tx_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void addba_timeout_fn(ADDRWORD_T data);
extern void addba_timeout_work(struct work_struct *work);    //add by Hugh
extern void ba_timeout_fn(ADDRWORD_T data);
extern void ba_timeout_work(struct work_struct *work);        //add by Hugh
extern void ba_pend_tx_timeout_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

extern void move_ba_window_ahead(ba_tx_struct_t *ba, UWORD16 lsn);
extern void handle_tx_ba_stall(ba_alarm_data_t *data);
extern void handle_rx_ba_stall(ba_alarm_data_t *data);
extern BOOL_T filter_wlan_ba_serv_cls(wlan_rx_t *wlan_rx, ba_rx_struct_t **ba);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/
INLINE void rx_frame_show(wlan_rx_t *wlan_rx)	//chwg test!
{
	if(wlan_rx == NULL)
		return;
	get_address1(wlan_rx->msa, wlan_rx->addr1);
    get_address2(wlan_rx->msa, wlan_rx->addr2);
    get_address3(wlan_rx->msa, wlan_rx->addr3);

    set_SA_DA_BSSID_ptr(wlan_rx);
    
    if(wlan_rx->sa == NULL)
    	printk("%s: sa is empty! frm_ds=%d, to_ds=%d\n", __func__, wlan_rx->frm_ds, wlan_rx->to_ds);
    if(wlan_rx->da == NULL)
    	printk("%s: da is empty! frm_ds=%d, to_ds=%d\n", __func__, wlan_rx->frm_ds, wlan_rx->to_ds);
}

/* This function updates the Block ACK capability of the STA. Note that only */
/* Immediate Block ACK option is supported and hence only this capability is */
/* checked (and not Delayed Block ACK)                                       */
INLINE void update_ba_cap(ba_struct_t *ba_hdl, UWORD16 cap)
{
    if((cap & IMMEDIATE_BA) == IMMEDIATE_BA)
        ba_hdl->imm_ba_capable = 1;
    else
        ba_hdl->imm_ba_capable = 0;
}

/* This function checks whether seq1 is less than or equal to seq2 */
INLINE BOOL_T seqno_leq(UWORD16 seq1, UWORD16 seq2)
{
    if(((seq1 <= seq2) && ((seq2 - seq1) < MAX_SEQNO_BY_TWO)) ||
       ((seq1 > seq2) && ((seq1 - seq2) > MAX_SEQNO_BY_TWO)))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks whether seq1 is less than seq2 */
INLINE BOOL_T seqno_lt(UWORD16 seq1, UWORD16 seq2)
{
    if(((seq1 < seq2) && ((seq2 - seq1) < MAX_SEQNO_BY_TWO)) ||
       ((seq1 > seq2) && ((seq1 - seq2) > MAX_SEQNO_BY_TWO)))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks whether seq1 is greater than or equal to seq2 */
INLINE BOOL_T seqno_geq(UWORD16 seq1, UWORD16 seq2)
{
    return seqno_leq(seq2, seq1);
}

/* This function checks whether seq1 is greater than seq2 */
INLINE BOOL_T seqno_gt(UWORD16 seq1, UWORD16 seq2)
{
    return seqno_lt(seq2, seq1);
}

/* This function compares the given sequence number with the specified */
/* upper and lower bounds and returns its position relative to them.   */
INLINE UWORD8 seqno_bound_chk(UWORD16 seq_lo, UWORD16 seq_hi, UWORD16 seq)
{
    BOOL_T lo_chk  = seqno_leq(seq_lo, seq);
    BOOL_T hi_chk  = seqno_leq(seq, seq_hi);
    UWORD8 chk_res = 0;

    if((BTRUE == lo_chk) && (BTRUE == hi_chk))
        chk_res = BETWEEN_SEQLO_SEQHI;
    else if(BTRUE == lo_chk)
        chk_res = GREATER_THAN_SEQHI;
    else
        chk_res = LESSER_THAN_SEQLO;

    return chk_res;
}

/*****************************************************************************/
/* Block ACK Request Transmit Descriptor Access Functions                    */
/*****************************************************************************/

/*          +------------------------------------------------------------+   */
/* WORD7    |BA H/w Flags                                                |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
INLINE UWORD32 get_bar_tx_dscr_ba_hw_flags(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_HW_FLAGS_OFFSET, FIELD_WIDTH_32,
                    (pkt_dscr_ptr + WORD_OFFSET_7));
}

/*          +------------------------------------------------------------+   */
/* WORD8    |BA SSN        |BA SSN (FN)  |BA Control                     |   */
/*          +------------------------------------------------------------+   */
/*          |12 bits       |4 bits       |16 bits                        |   */
/*          +------------------------------------------------------------+   */
INLINE UWORD16 get_bar_tx_dscr_ba_ssn(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_SSN_OFFSET, FIELD_WIDTH_12,
                    (pkt_dscr_ptr + WORD_OFFSET_8));
}

INLINE UWORD16 get_bar_tx_dscr_ba_ctrl(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_CTRL_OFFSET, FIELD_WIDTH_16,
                    (pkt_dscr_ptr + WORD_OFFSET_8));
}

/*          +------------------------------------------------------------+   */
/* WORD9    |BA Bitmap Word 0                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
INLINE UWORD32 get_bar_tx_dscr_ba_bmap_w0(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_BITMAP_WORD0_OFFSET, FIELD_WIDTH_32,
                    (pkt_dscr_ptr + WORD_OFFSET_9));
}

/*          +------------------------------------------------------------+   */
/* WORD10   |BA Bitmap Word 1                                            |   */
/*          +------------------------------------------------------------+   */
/*          |32 bits                                                     |   */
/*          +------------------------------------------------------------+   */
INLINE UWORD32 get_bar_tx_dscr_ba_bmap_w1(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_BITMAP_WORD1_OFFSET, FIELD_WIDTH_32,
                    (pkt_dscr_ptr + WORD_OFFSET_10));
}

/*          +------------------------------------------------------------+   */
/* WORD11   |S/W BA Information                                          |   */
/*          +------------------------------------------------------------+   */
/*          |BAR Last Sequence Number     |Reserved                      |   */
/*          +------------------------------------------------------------+   */
/*          |16 bits                      |16 bits                       |   */
/*          +------------------------------------------------------------+   */
INLINE void set_bar_tx_dscr_ba_sw_barlsn(UWORD32 *pkt_dscr_ptr, UWORD16 value)
{
    set_dscr(BAR_TX_DSCR_FIELD_BA_SW_INFO_BARLSN_OFFSET, FIELD_WIDTH_16,
             (pkt_dscr_ptr + WORD_OFFSET_11), value);
}

INLINE UWORD16 get_bar_tx_dscr_ba_sw_barlsn(UWORD32 *pkt_dscr_ptr)
{
    return get_dscr(BAR_TX_DSCR_FIELD_BA_SW_INFO_BARLSN_OFFSET, FIELD_WIDTH_16,
                    (pkt_dscr_ptr + WORD_OFFSET_11));
}

/*****************************************************************************/
/* Block ACK Alarm Related Functions                                         */
/*****************************************************************************/

/* Stop the BA timer */
INLINE void stop_ba_timer(ALARM_HANDLE_T **hdl)
{
    if(*hdl != NULL)
        stop_alarm(*hdl);
}

/* Delete the BA timer */
INLINE void del_ba_timer(ALARM_HANDLE_T **hdl)
{
    if(*hdl != NULL)
    {
        delete_alarm(hdl);
        *hdl = NULL;
    }
}

/* Start the ADDBA Timeout timer */
INLINE void start_addba_timer(ALARM_HANDLE_T **hdl, UWORD32 time, UWORD32 data)
{
    stop_ba_timer(hdl);
    del_ba_timer(hdl);

    *hdl = create_alarm(addba_timeout_fn, data, addba_timeout_work); //Hugh
    start_alarm(*hdl, time);
}

/* Restart the ba Timer */
INLINE void restart_ba_timer(ALARM_HANDLE_T **hdl, UWORD32 time, UWORD32 data)
{
    if(*hdl == NULL)
    {
        *hdl = create_alarm(ba_timeout_fn, data, ba_timeout_work);  //Hugh
    }

    stop_alarm(*hdl);
    start_alarm(*hdl, time);
}

/* Start the ADDBA Timeout timer */
INLINE void start_ba_pend_timer(ALARM_HANDLE_T **hdl, UWORD32 time,
                                UWORD32 data)
{
    stop_ba_timer(hdl);

    if(NULL == *hdl)
        *hdl = create_alarm(ba_pend_tx_timeout_fn, data, NULL);     //Hugh

    start_alarm(*hdl, time);
}

/* This function returns the starting sequence number contained in a */
/* Block-Ack Request or a Block-Ack frame.                           */
INLINE UWORD16 get_ba_start_seq_num(UWORD8 *msa)
{
    return ((msa[18] & 0xF0) >> 4) | (msa[19] << 4);
}

/* This function returns the starting sequence number contained in a */
/* Block-Ack Request or a Block-Ack frame.                           */
INLINE UWORD16 get_bar_start_seq_num(UWORD8 *msa)
{
    return ((msa[18] & 0xF0) >> 4) | (msa[19] << 4);
}

/* Reset the 11e BA handles */
INLINE void reset_ba_handle(ba_struct_t *ba)
{
    UWORD8 i = 0;

    for(i = 0; i < 16; i ++)
    {
        reset_tx_ba_handle(&ba->ba_tx_ptr[i], i);
        mutex_lock(&ba_rx_mutex);	//add by chengwg 2013-06-21.
        reset_rx_ba_handle(&ba->ba_rx_ptr[i], i);
        mutex_unlock(&ba_rx_mutex);
    }
}

/* This function performs the necessary initialization for Block-Ack */
INLINE void initialize_ba(void)
{
    if(mget_ImmediateBlockAckOptionImplemented() == TV_TRUE)
    {
        /* Initialize the BA LUT index table bitmap */
        init_lut(g_ba_lut_index_table, BA_LUT_IDX_BMAP_LEN);
        g_num_ba_tx_sessions = 0;
        g_num_ba_rx_sessions = 0;
    }

    mutex_init(&ba_rx_mutex);	//add by chengwg 2013-06-21.
}

/* Init the 11e BA handles */
INLINE void init_ba_handle(ba_struct_t *ba)
{
    reset_ba_handle(ba);
}

/* This function reorders the frames and sends them to the higher layer. */
/* It also updates the receiver buffer window.                           */
INLINE void reorder_ba_rx_buffer_data(ba_rx_struct_t *ba, UWORD8 tid,
                                          UWORD16 seq_num)
{
    UWORD8  seqno_pos     = 0;

    seqno_pos = seqno_bound_chk(ba->win_start, ba->win_end, seq_num);

    if(BETWEEN_SEQLO_SEQHI == seqno_pos)
    {
        ba->win_start = send_frames_in_order(ba, tid);
        ba->win_end   = SEQNO_ADD(ba->win_start, (ba->buff_size - 1));
    }
    else if(GREATER_THAN_SEQHI == seqno_pos)
    {
        UWORD16 temp_winend   = seq_num;
        UWORD16 temp_winstart = SEQNO_SUB(temp_winend, (ba->buff_size - 1));

        send_frames_with_gap(ba, tid, temp_winstart);
        ba->win_start = temp_winstart;
        ba->win_start = send_frames_in_order(ba, tid);
        ba->win_end   = SEQNO_ADD(ba->win_start, (ba->buff_size - 1));
    }
    else
    {
        /* Do Nothing */
    }
}

/* This function reorders the Reciver buffer and sends frames to the higher */
/* layer on reception of a Block-Ack-Request frame. It also updates the     */
/* receiver buffer window.                                                  */
INLINE void reorder_ba_rx_buffer_bar(ba_rx_struct_t *ba, UWORD8 tid,
                                  UWORD16 start_seq_num)
{
  if(BTRUE == seqno_gt(start_seq_num, ba->win_start))
  {
       send_frames_with_gap(ba, tid, start_seq_num);
       ba->win_start = start_seq_num;
       ba->win_start = send_frames_in_order(ba, tid);
       ba->win_end   = SEQNO_ADD(ba->win_start, (ba->buff_size - 1));
  }
}

/* This function checks whether the Receive Service Class for the specified */
/* entry is Block-Ack.                                                      */
INLINE BOOL_T is_ba_rx_session_setup(ba_rx_struct_t **ba)
    {
        if(((*ba) != NULL) && ((*ba)->is_ba == BTRUE))
        return BTRUE;

    return BFALSE;
}

/* This function sends DELBA if an AMPDU is received on a TID with no active */
/* BA session.                                                               */
INLINE void handle_non_ba_ampdu_rx(UWORD8 tid, UWORD8 *addr)
{
    send_delba(addr, tid, RECIPIENT, QSTA_SETUP_NOT_DONE);
}

/* This function updates the service class policy based on the Block-Ack */
/* status.                                                               */
INLINE void update_serv_class_ba(ba_struct_t *ba_hdl, UWORD8 tid,
                                  UWORD8 *serv_class)
{
    ba_tx_struct_t **ba = NULL;

    if((tid >= 16) || (ba_hdl->ba_tx_ptr[tid] == NULL))
    {
        return;
    }

    ba = &(ba_hdl->ba_tx_ptr[tid]);

    if(((*ba)->is_ba == BFALSE) || ((*ba)->conn_status == BA_FAILED) ||
       ((*ba)->conn_status == BA_HALTED))
    {
        return;
    }

    //*serv_class = BLOCK_ACK;
    *serv_class = NORMAL_ACK;	//chengwg test!
}

/* This function flushes the BA Receiver Buffer and passes all the pending */
/* frames to the higher layer.                                             */
INLINE void flush_ba_rx_buffer(ba_rx_struct_t *ba, UWORD8 tid)
{
    UWORD16 temp_winend = SEQNO_ADD(ba->win_end, 1);

    send_frames_with_gap(ba, tid, temp_winend);
    ba->win_start = temp_winend;
    ba->win_end   = SEQNO_ADD(ba->win_start, (ba->buff_size - 1));
}

INLINE UWORD8 get_pend_dequeue_thresh(ba_tx_struct_t **ba)
{
    UWORD8 deq_thresh = (*ba)->bar_int;

    if((*ba)->bar_int >= (*ba)->cnt_from_last_bar)
        deq_thresh -= (*ba)->cnt_from_last_bar;

    return deq_thresh;
}

INLINE UWORD8 get_curr_dequeue_limit(ba_tx_struct_t **ba)
{
    UWORD8 max_num_dq = 0;

    /* Compute the Maximum Number of frames allowed by BA-TX Window */
    max_num_dq = SEQNO_SUB((*ba)->win_end, (*ba)->last_seq_num);
	
    return max_num_dq;
}

/* This function checks the number of frames that can be transmitted as */
/* permitted by the BA-Tx window and enqueues them for transmission.    */
/* A BAReq frame is also generated.                                     */
INLINE void transmit_frames_ba(void *entry, ba_tx_struct_t **ba, UWORD8 q_num,
                               UWORD8 tid)
{
    UWORD8 num_dq     = 0;
    UWORD8 max_num_dq = 0;
    UWORD8 num_dqed   = 0;
    UWORD8 bar_int    = 0;
    UWORD8 cnt        = 0;

    if((*ba) == NULL)
        return;

    /* Check whether MSDUs can be buffered for transmission. */
    if(BTRUE == seqno_lt((*ba)->last_seq_num, (*ba)->win_end))
    {
        /* Compute the Maximum Number of frames allowed by BA-TX Window */
        max_num_dq = SEQNO_SUB((*ba)->win_end, (*ba)->last_seq_num);

		/* Move to Suspended state if the TX-BA Window is closed. */
        if(max_num_dq == 0)
        {
            (*ba)->conn_status = BA_SUSPENDED;
            TX_BA_DBG("%s: ba tx suspend!\n", __func__);
        }
        
        bar_int = (*ba)->bar_int;

        while(1)
        {
            /* Get the count of packets from the last BAR. Note that this    */
            /* count is updated to 0 whenever a BAR is transmitted and       */
            /* incremented whenever a pending packet is queued or a          */
            /* transmitted packet is retried.                                */
            cnt = (*ba)->cnt_from_last_bar;

            num_dq = MIN(max_num_dq, (bar_int - cnt));

            num_dqed = dequeue_tx_ba_frame(entry, *ba, num_dq, tid);
			TX_BA_DBG("bar_int=%d, cnt=%d, num_dq=%d, num_dqed=%d\n", bar_int, cnt, num_dq, num_dqed);
#ifdef DEBUG_MODE
            if(num_dqed > 0)
            {
                g_mac_stats.ba_num_dq   += num_dq;
                g_mac_stats.ba_num_dqed += num_dqed;
                g_mac_stats.batxfba++;
            }
#endif /* DEBUG_MODE */

            max_num_dq -= num_dqed;
            if((max_num_dq == 0) || (num_dqed < num_dq))
                break;
        }
    }
}

/* This function restarts the block-ack session */
INLINE void restart_ba(ba_tx_struct_t **ba)
{
    if(NULL != (*ba))
    {
        (*ba)->conn_status = BA_INPROGRESS;
        /* The LSN is tracked during PS for the number of frames that have   */
        /* been transmitted from the given TID. The window parameters are    */
        /* Initialised based on this. Reset of the counters are reset to 0   */
        (*ba)->os_bar_cnt = 0;
        (*ba)->cnt_from_last_bar = 0;
        (*ba)->num_pend = 0;
        (*ba)->num_pipe = 0;
        (*ba)->win_start    = SEQNO_ADD((*ba)->last_seq_num, 1);
        (*ba)->win_end      =
            SEQNO_ADD((*ba)->win_start, ((*ba)->buff_size - 1));
        (*ba)->pend_alarm_on = BFALSE;
        (*ba)->batxwinstall  = 0;
    }
}

/* This function updates the state parameters of a Halted Tx-Block Ack session */
/* on transmission of a frame.                                                 */
INLINE void update_halted_ba_ctxt(ba_tx_struct_t **ba)
{
    if(((*ba) != NULL) && ((*ba)->conn_status == BA_HALTED))
    {
        (*ba)->last_seq_num = SEQNO_ADD((*ba)->last_seq_num, 1);
        (*ba)->win_start    = SEQNO_ADD((*ba)->last_seq_num, 1);
        (*ba)->win_end      = SEQNO_ADD((*ba)->win_start, ((*ba)->buff_size - 1));
    }
}

/* This function returns an unused BA LUT index from the BA LUT index bitmap */
/* table. If full it returns an invalid index.                               */
INLINE UWORD8 get_ba_lut_index(void)
{
    return get_lut_index(g_ba_lut_index_table, BA_LUT_IDX_BMAP_LEN,
                         MAX_BA_LUT_SIZE);
}

/* This function deletes a BA LUT index from the BA LUT index bitmap table.  */
INLINE void del_ba_lut_index(UWORD8 idx)
{
    del_lut_index(g_ba_lut_index_table, idx);
}

/* This function checks whether the BAR is valid for the current window */
/* parameters.                                                          */
INLINE BOOL_T is_bar_useful(ba_tx_struct_t *ba, UWORD8 *msa)
{
   UWORD16 ssn = get_bar_start_seq_num(msa);

   if(ssn == ba->win_start)
       return BTRUE;

   return BFALSE;
}

/* This function checks whether the Transmitted Data frame is useful enough */
/* to be buffered.                                                          */
INLINE BOOL_T is_tx_data_useful(ba_tx_struct_t *ba, UWORD16 seq_num)
{
   if(BTRUE == seqno_geq(seq_num, ba->win_start))
       return BTRUE;

   return BFALSE;
}

/* This function checks whether the BA is valid for the current window */
/* parameters. The SSN of the BA should be in the neighbourhood of     */
/* the current WinStart to declare the BA valid.                       */
INLINE BOOL_T is_ba_useful(ba_tx_struct_t *ba, UWORD8 *msa)
{
   UWORD8  bat = (msa[16] & 0x06) >> 1;
   UWORD16 ssn = get_ba_start_seq_num(msa);
   UWORD16 wlb = SEQNO_SUB(ba->win_start, 63);
   UWORD16 wub = ba->last_seq_num;

   /* First check that the Block-Ack frame is of supported type */
   if(bat != COMPRESSED_BACK)
       return BFALSE;

   /* Second check is whether the Block-Ack frame is stale */
   if(seqno_bound_chk(wlb, wub, ssn) == BETWEEN_SEQLO_SEQHI)
       return BTRUE;

   return BFALSE;
}

/* This function updates the ADDBA-Request with 11n specific parameters */
INLINE void update_addba_req_param_ht(ht_struct_t *ht_hdl,
                                       ba_tx_struct_t *ba,
                                       UWORD16 *ba_param)
{
    if((NULL != ht_hdl) && (1 == ht_hdl->ht_capable))
    {
        ba->back_var   = COMPRESSED_BACK;
        ba->amsdu_supp = AMSDU_BACK_SUPPORTED_FLAG;
    }
    else
    {
        ba->back_var   = BASIC_BACK;
        ba->amsdu_supp = 0;
    }

    *ba_param = (*ba_param & ~BIT0) | ba->amsdu_supp;
}

/* This function extracts 11n specific parameters from ADDBA-Request frame */
INLINE void extract_addba_req_ht_params(ht_struct_t *ht_hdl,
                                        ba_rx_struct_t *ba, UWORD8 *req_data)
{
    if((NULL != ht_hdl) && (1 == ht_hdl->ht_capable))
    {
        ba->back_var   = COMPRESSED_BACK;
        ba->amsdu_supp = AMSDU_BACK_SUPPORTED_FLAG & (req_data[3] & BIT0);
    }
    else
    {
        ba->back_var   = BASIC_BACK;
        ba->amsdu_supp = 0;
    }
}

/* This function extracts 11n specific parameters from ADDBA-Response frame */
INLINE void extract_addba_resp_ht_params(ht_struct_t *ht_hdl,
                                         ba_tx_struct_t *ba, UWORD8 *req_data)
{
    if(1 == ba->amsdu_supp)
        ba->amsdu_supp = (ba->amsdu_supp) & (req_data[5] & BIT0);

    /* TBD: Disable AMSDU-Aggregation for the specified TID if the receiver */
    /* cannot support A-MSDU with Block-Ack.                                */

}

/* This function updates the start sequence number for the block-ack session */
INLINE void update_ba_start_seq_num(ba_struct_t *ba, UWORD8 tid, UWORD16 seq_num)
{
    if(ba->ba_tx_ptr[tid] == NULL)
    {
        return;
    }

    ba->ba_tx_ptr[tid]->win_start    = seq_num;
    ba->ba_tx_ptr[tid]->last_seq_num = SEQNO_SUB(seq_num, 1);
    ba->ba_tx_ptr[tid]->win_end      = SEQNO_ADD(ba->ba_tx_ptr[tid]->win_start,
                                                 (ba->ba_tx_ptr[tid]->buff_size - 1));
    TROUT_DBG4("%s: tid=%d, win_start=%d, win_end=%d, last_seq_num=%d\n",
    							__func__, tid, 
    							ba->ba_tx_ptr[tid]->win_start, 
    							ba->ba_tx_ptr[tid]->win_end, 
    							ba->ba_tx_ptr[tid]->last_seq_num);
}

/* This function checks whether the MSDU indexed by ofst parameter is */
/* reported missing within the bitmap.                                */
INLINE BOOL_T is_ba_missing_msdu(UWORD8 *bitmap, UWORD8 ofst)
{
    UWORD8 idx = ofst >> 3;

    if(ofst >= 64)
        return BTRUE;

	TX_BA_DBG("ofst=%d, bitmap[%d]=0x%x\n", ofst, idx, bitmap[idx]);
    if((bitmap[idx] & (1 << (ofst & 7))) != 0)
        return BFALSE;

    return BTRUE;
}

/* This function checks whether the current number of Block-Ack sessions */
/* exceed the maximum supported limit.                                   */
INLINE BOOL_T is_ba_limit_reached(void)
{
    if((g_num_ba_tx_sessions + g_num_ba_rx_sessions) >= MAX_NUM_BA_SESSIONS_SUPP)
        return BTRUE;

    return BFALSE;
}

/* This function returns the number of Block-Ack TX sessions setup with */
/* the specified station.                                               */
INLINE UWORD8 get_num_tx_ba_sessions_11n(ba_struct_t *ba_ctxt)
{
    UWORD8      retval   = 0;

    if(ba_ctxt != NULL)
        retval = ba_ctxt->num_ba_tx;

    return retval;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_tx_ba_sessions_11n(ba_struct_t *ba_ctxt)
{
    if(ba_ctxt != NULL)
        ba_ctxt->num_ba_tx++;
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_tx_ba_sessions_11n(ba_struct_t *ba_ctxt)
{
    if(ba_ctxt != NULL)
        ba_ctxt->num_ba_tx--;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_rx_ba_sessions_11n(ba_struct_t *ba_ctxt)
{
    if(ba_ctxt != NULL)
        ba_ctxt->num_ba_rx++;
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_rx_ba_sessions_11n(ba_struct_t *ba_ctxt)
{
    if(ba_ctxt != NULL)
        ba_ctxt->num_ba_rx--;
}

/* This function buffers the new TX descriptor to the pending queue. */
/* Since no other resource is required for this, it is assumed that  */
/* it will always succeed.                                           */
INLINE void buffer_frame_in_pending_q(ba_tx_struct_t *ba, UWORD8 *tx_dscr)
{
    add_list_element(&ba->pend_list, tx_dscr);
    ba->num_pend++;
}

/* This function reads out the TX-Dscr at the head of the Pending-Q */
INLINE UWORD8 *remove_frame_from_pending_q(ba_tx_struct_t *ba)
{
    UWORD8 *tx_dscr = NULL;

    tx_dscr = (UWORD8 *)remove_list_element_head(&ba->pend_list);

    if(tx_dscr != NULL)
        ba->num_pend--;

    return tx_dscr;
}

/* This function buffers the TX-Dscr to the Retry-Q. The Retry-Q is      */
/* maintained as a Ring-Buffer with the sequence number used as the key. */
INLINE void buffer_frame_in_retry_q(ba_tx_struct_t *ba, UWORD16 seq_num,
                                    UWORD8 *tx_dscr)
{
    UWORD16        idx = (seq_num & (MAX_BA_TX_BUFFERS - 1));
    bfd_tx_pkt_t *buff = &(ba->buff_list[idx]);

    if(buff->in_use == 1)
    {
        /* Check whether the buffer location is currently in use. This is an error */
        /* condition & should not occur under normal operation.                    */
#ifdef DEBUG_MODE
        /* Update debug counters */
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)buff->tx_dscr);
    }

    buff->seq_num = seq_num;
    buff->tx_dscr = tx_dscr;
    buff->in_use  = 1;
}

/* This function reads out the TX-Dscr indexed by the specified sequence number in */
/* the Retry-Q Ring-Buffer.                                                        */
INLINE void *remove_frame_from_retry_q(ba_tx_struct_t *ba, UWORD16 seq_num)
{
    UWORD16        idx = (seq_num & (MAX_BA_TX_BUFFERS - 1));
    bfd_tx_pkt_t *buff = &(ba->buff_list[idx]);

    if((buff->in_use == 0) || (buff->seq_num != seq_num))
        return NULL;

    buff->in_use  = 0;

    return buff->tx_dscr;
}

/* This function flushes out the Retry-Q by deleting all Tx-Dscrs  contained */
/* in it with sequence number less than or equal to the passed argument.     */
INLINE void cleanup_retry_q(ba_tx_struct_t *ba, UWORD16 lsn)
{
    UWORD16       idx  = 0;
    bfd_tx_pkt_t *buff = NULL;

    for(idx = 0; idx < MAX_BA_TX_BUFFERS; idx++)
    {
        buff = &(ba->buff_list[idx]);

        if((buff->in_use == 1) && (seqno_leq(buff->seq_num, lsn) == BTRUE))
        {
            free_tx_dscr((UWORD32 *)buff->tx_dscr);
            ba->num_pipe--;
            buff->in_use = 0;
        }
    }
}

/* This function flushes out the Retry-Q by deleting all Tx-Dscrs contained in it */
INLINE void flush_retry_q(ba_tx_struct_t *ba)
{
    UWORD16       idx  = 0;
    bfd_tx_pkt_t *buff = NULL;

    for(idx = 0; idx < MAX_BA_TX_BUFFERS; idx++)
    {
        buff = &(ba->buff_list[idx]);

        if(buff->in_use == 1)
        {
            free_tx_dscr((UWORD32 *)buff->tx_dscr);
            ba->num_pipe--;
            buff->in_use = 0;
        }
    }
}

/* This function buffers the RX-Dscr to the Reorder-Q. The Reorder-Q is    */
/* maintained as a Ring-Buffer with the sequence number used as the key.   */
/* This function does not update any fields of the reorder-Q element.      */
/* Instead it retruns the entire reorder queue element itself which is     */
/* expected to be updated by the calling function.                         */
INLINE bfd_rx_pkt_t *buffer_frame_in_reorder_q(ba_rx_struct_t *ba, UWORD16 seq_num)
{
    UWORD16        idx = (seq_num & (MAX_BA_RX_BUFFERS - 1));
    bfd_rx_pkt_t *buff = &(ba->buff_list[idx]);

    if(buff->in_use == 1)
    {
        /* Check whether the buffer location is currently in use. This is an */
        /* error condition & should not occur under normal operation.        */
#ifdef DEBUG_MODE
        /* Update debug counters */
#endif /* DEBUG_MODE */
        //free_rx_dscr_list(buff->dscr, buff->num_dscr);
//        printk("%s(%d): dscr=0x%p, num=%d\n", __func__, idx, buff->dscr, buff->num_dscr);
        free_host_rx_dscr_list(buff->dscr, buff->num_dscr);	//modify by chengwg.
    }

    buff->in_use = 1;

    return buff;
}

/* This function reads out the TX-Dscr indexed by the specified sequence number in */
/* the Retry-Q Ring-Buffer.                                                        */
INLINE bfd_rx_pkt_t *remove_frame_from_reorder_q(ba_rx_struct_t *ba, UWORD16 seq_num)
{
    UWORD16        idx = (seq_num & (MAX_BA_RX_BUFFERS - 1));
    bfd_rx_pkt_t *buff = &(ba->buff_list[idx]);

    if((buff->in_use == 0) || (buff->seq_num != seq_num))
        return NULL;

    buff->in_use = 0;

    return buff;
}

/* This function flushes out the Reorder-Q by deleting all Rx-Dscrs contained in it */
INLINE void flush_reorder_q(ba_rx_struct_t *ba)
{
    UWORD16       idx  = 0;
    bfd_rx_pkt_t *buff = NULL;

    for(idx = 0; idx < MAX_BA_RX_BUFFERS; idx++)
    {
        buff = &(ba->buff_list[idx]);

        if(buff->in_use == 1)
        {
#if 0
            free_rx_dscr_list(buff->dscr, buff->num_dscr);
#else            
            free_host_rx_dscr_list(buff->dscr, buff->num_dscr);		//modify by chenq, 2012-11-09
#endif            
            buff->in_use = 0;
        }
    }
}

/* This function updates the SSN field in the Block-Ack Request frame */
INLINE void update_blockack_req_ssn(ba_tx_struct_t *ba)
{
    ba->bar_frame[18] = (ba->win_start << 4);
    ba->bar_frame[19] = ((ba->win_start >> 4) & 0xFF);
}

/* This function sets the Tx-Dscr fields which are specific to BAR frames    */
/* BAR frames are different from other frames created by MAC S/w since these */
/* BAR frames are created within the TX-Dscr itself.                         */
INLINE void update_bar_tx_dscr_fields(ba_tx_struct_t *ba, UWORD32 *tx_dscr)
{
    buffer_desc_t buff_desc = {0};
    UWORD8 *bar_msa = (UWORD8 *)(tx_dscr + BAR_TX_DSCR_FIELD_SW_INFO_BAR_WORD_OFFSET);

    /* Copy the data portion of the BAR into the appropriate location in the TX-Dscr */
    memcpy(bar_msa, ba->bar_frame, MAX_BAR_DATA_LEN);	//bar_frame is prepared at update_addba_list()!
	
    /* The last sequence number being used is added to the TX-Dscr */
    set_bar_tx_dscr_ba_sw_barlsn(tx_dscr, ba->last_seq_num);

    /* Update relevant fields in the TX-Dscr */
    set_tx_dscr_ack_policy(tx_dscr, COMP_BAR);

    /* Set the frame header buffer details */
    set_tx_dscr_buffer_addr(tx_dscr, (UWORD32)bar_msa);
    set_tx_dscr_mh_len(tx_dscr, CONTROL_HDR_LEN);
    set_tx_dscr_mh_offset(tx_dscr, 0);

    /* Set the frame body buffer details */
    buff_desc.buff_hdl    = (UWORD8 *)bar_msa;
    buff_desc.data_length = MAX_BAR_DATA_LEN - CONTROL_HDR_LEN;
    buff_desc.data_offset = CONTROL_HDR_LEN;

    set_tx_submsdu_info((UWORD8 *)tx_dscr, &buff_desc, 1, CONTROL_HDR_LEN);
}

/* This function checks if the BA is a valid one */
INLINE BOOL_T is_valid_ba(UWORD16 bar_ctrl, UWORD8 tid)
{
    BACK_VARIANT_T back_var = BASIC_BACK;

    back_var = (BACK_VARIANT_T)((bar_ctrl & 0x06) >> 1);

    if(back_var != COMPRESSED_BACK)
    {
        /* Only Compressed Block-Ack currently supported */
        return BFALSE;
    }

     /* Check the TID */
    if(((bar_ctrl & 0xF000) >> 12) != tid)
        return BFALSE;

    return BTRUE;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/* This function checks if there are any frames in BA pending queue */
INLINE BOOL_T frames_in_ba_pnd_q(ba_struct_t *ba_hdl, UWORD8 tid)
{
    BOOL_T ret_val = BFALSE;
    ba_tx_struct_t **ba = NULL;

    if((tid >= 16) || (ba_hdl->ba_tx_ptr[tid] == NULL))
    {
        return ret_val;
    }

    ba = &(ba_hdl->ba_tx_ptr[tid]);

    if(((*ba)->is_ba == BFALSE) || ((*ba)->conn_status == BA_FAILED) ||
       ((*ba)->conn_status == BA_HALTED))
    {
        return ret_val;
    }

    if((*ba)->num_pend > 0)
        ret_val = BTRUE;

    return ret_val;
}

#endif  /* BLOCKACK_H */
#endif  /* MAC_802_11N */

