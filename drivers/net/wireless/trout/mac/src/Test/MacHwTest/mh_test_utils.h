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
/*  File Name         : mh_test_utils.h                                      */
/*                                                                           */
/*  Description       : This file contains all the utility definitions for   */
/*                      H/w unit test.                                       */
/*                                                                           */
/*  List of Functions : set_qos_control_field_ut                             */
/*                      init_hut_txq_pri_table                               */
/*                      get_txq_num_hut                                      */
/*                      process_txc_burst_tx_mode                            */
/*                      is_frag_frame                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

#ifndef MH_TEST_UTILS_H
#define MH_TEST_UTILS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

#ifdef MAC_WMM
#include "frame_11e.h"
#endif /* MAC_WMM */

#include "mh_test_config.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define TUPLECACHESIZE          8

/* Sub-MSDU length should be multiple of 4 & include atleast sub-MSDU header */
#define MIN_SUB_MSDU_SIZE       16
#define SUBMSDU_SPACING         2

/* Port definitions for the frames which should be injected in the WLAN */
#define HUT_HOST_UDP_PORT       40000
#define HUT_WLAN_UDP_PORT       40000

/* Per-Frame attribute definitions for the frames in Frame-Inject mode */
#define HUT_FRAME_ATTRIB_FLAG   BIT0
#define HUT_RETX_FRAME_FLAG     BIT1
#define HUT_CORRUPT_FRAME_FLAG  BIT2
#define HUT_CORRUPT_DLMTR_FLAG  BIT3
#define HUT_CE_DISABLE_FLAG     BIT4

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_txq_pri_to_num_hut[17];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    replenish_test_rx_queue(UWORD8 q_num);
extern void    update_packet_seq_num(UWORD32 *dscr);
extern void    clear_tuple_cache(void);
extern UWORD8  get_hut_tx_rate(void);
extern UWORD32 get_hut_phy_tx_mode(UWORD8 dr, UWORD8 pream);
extern BOOL_T  check_hut_frame(UWORD8* event);
extern BOOL_T  search_and_update_tuple_cache(UWORD8* addr, UWORD16 seq_num,
                                             UWORD8 frag_num, UWORD8 retry);

#ifdef MAC_802_11N
/* This function creates an AMSDU with the specified parameters */
extern UWORD16 create_amsdu(UWORD8 *buff, UWORD16 max_size, UWORD16 max_num,
                            UWORD8 *sa, UWORD8 *da);
#endif /* MAC_802_11N */

extern void compute_sig_qual_avg(UWORD32* rx_dscr);
extern void update_rssi(UWORD32* rx_dscr);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Set the QoS Control field in the MAC header for QoS Data frames with the  */
/* given TID and ACK Policy.                                                 */
INLINE void set_qos_control_field_ut(UWORD8 *header, UWORD8 tid, UWORD8 ap)
{
#ifdef MAC_WMM
    /* +---------+----+------------+----------+----------------------------+ */
    /* | B0 - B3 | B4 |   B5 - B6  |    B7    |  B8 - B15                  | */
    /* +---------+----+------------+----------+----------------------------+ */
    /* |   TID   | 0  | Ack Policy | Reserved | TXOP Duration Requested    | */
    /* +---------+----+------------+----------+----------------------------+ */
    /* |   TID   | 1  | Ack Policy | Reserved | Queue Size                 | */
    /* +---------+----+------------+----------+----------------------------+ */
    header[QOS_CTRL_FIELD_OFFSET]  = tid;
    header[QOS_CTRL_FIELD_OFFSET] |= ap << 5;

    header[QOS_CTRL_FIELD_OFFSET + 1] = 0;
#endif /* MAC_WMM */
}

/* In the station mode there is one queue per TID. The mapping of TIDs       */
/* 0 - 7 to access categories is as per standard.                            */
/* TID           : Access Category                                           */
/* 1, 2          : AC_BK_Q                                                   */
/* 3, 0          : AC_BE_Q                                                   */
/* 5, 4          : AC_VI_Q                                                   */
/* 7, 6          : AC_VO_Q                                                   */
/* 8 is mapped to High Priority queue                                        */
INLINE void init_hut_txq_pri_table(void)
{
#ifdef MAC_WMM
    g_txq_pri_to_num_hut[0] = AC_BE_Q;
    g_txq_pri_to_num_hut[1] = AC_BK_Q;
    g_txq_pri_to_num_hut[2] = AC_BK_Q;
    g_txq_pri_to_num_hut[3] = AC_BE_Q;
    g_txq_pri_to_num_hut[4] = AC_VI_Q;
    g_txq_pri_to_num_hut[5] = AC_VI_Q;
    g_txq_pri_to_num_hut[6] = AC_VO_Q;
    g_txq_pri_to_num_hut[7] = AC_VO_Q;
    g_txq_pri_to_num_hut[8] = HIGH_PRI_Q;
#else /* MAC_WMM */
    g_txq_pri_to_num_hut[0] = NORMAL_PRI_Q;
    g_txq_pri_to_num_hut[1] = CF_PRI_Q;
    g_txq_pri_to_num_hut[2] = HIGH_PRI_Q;
#endif /* MAC_WMM */
}

INLINE UWORD8 get_txq_num_hut(UWORD8 priority)
{
    UWORD8  retval = NORMAL_PRI_Q;

#ifdef MAC_WMM
    retval = g_txq_pri_to_num_hut[priority];
#endif /* MAC_WMM */

    return retval;
}

/* This function processes Tx complete interrupt if BURST_TX_MODE is enabled */
INLINE void process_txc_burst_tx_mode(void)
{
#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        update_burst_mode_tx_dscr((UWORD8 *)tx_dscr);
        return;
    }
#endif /* BURST_TX_MODE */
}

INLINE BOOL_T is_frag_frame(UWORD8 *msa)
{
    if((get_fragment_number(msa) > 0) || (get_more_frag(msa) == 1))
        return BTRUE;

    return BFALSE;
}

#endif /* MH_TEST_UTILS_H */

#endif /* MAC_HW_UNIT_TEST_MODE */
