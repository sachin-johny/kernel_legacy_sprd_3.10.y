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
/*  File Name         : amsdu_aggr.h                                         */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AMSDU-aggregation                                    */
/*                                                                           */
/*  List of Functions : stop_amsdu_timer                                     */
/*                      start_amsdu_timer                                    */
/*                      free_amsdu_ctxt                                      */
/*                      check_amsdu_ready_status                             */
/*                      check_amsdu_space_status                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

#ifndef AMSDU_AGGR_H
#define AMSDU_AGGR_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "amsdu_common.h"
#include "csl_if.h"
#include "host_if.h"
#include "transmit.h"
#include "frame_11n.h"

/*****************************************************************************/
/* Constants Definitions                                                     */
/*****************************************************************************/

#define MAX_NUM_SIMUL_AMSDU         4    /* Max simultaneous AMSDU sessions */
#define DEFAULT_AMSDU_LEN           3839
#define MIN_AMSDU_LEN               256  /* Min Size of an A-MSDU frame */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8         in_use_flag;     /* Indicates the usage status      */
    UWORD8         max_num;         /* Maximum number of sub-MSDUs     */
    UWORD8         num_msdu;        /* Number of sub-MSDUs accumulated */
    UWORD8         num_entries;     /* Number of Entries in Table      */
    UWORD8         priority_val;    /* TID of the ASDU                 */
    UWORD8         q_num;           /* Tx Q number                     */
    UWORD8         last_msdu_padlen;/* Pad-Length of the last sub-MSDU */
    UWORD16        amsdu_maxsize;   /* Max AMSDU Size supported        */
    UWORD16        amsdu_size;      /* Present size of the AMSDU       */
    void           *entry;          /* Association entry of the RA STA */
    void           *tx_dscr;        /* Tx descriptor of the AMSDU      */
    ALARM_HANDLE_T *to_alarm;       /* AMSDU timeout alarm             */
    UWORD8         addr[6];         /* RA of the AMSDU                 */
    UWORD32        invalid;	    /* the amsdu will be free due to timeout etc...*/
} amsdu_ctxt_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern amsdu_ctxt_t g_amsdu_ctxt[MAX_NUM_SIMUL_AMSDU];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
//extern void config_amsdu_func(UWORD8 *ra, BOOL_T B_ACK);	//add by chengwg.
extern void config_802_11n_feature(UWORD8 *ra, BOOL_T B_ACK);
extern BOOL_T amsdu_tx(amsdu_ctxt_t *amsdu_ctxt);
extern void amsdu_set_submsdu_info(UWORD8 *tx_dscr, buffer_desc_t *buff_list,
                                   UWORD16 num_buff, UWORD16 buff_len,
                                   UWORD16 mac_hdr_len, UWORD8 *sa, UWORD8 *da,
                                   void *amsdu_handle);
extern amsdu_ctxt_t *get_amsdu_ctxt(UWORD8 *rx_addr, UWORD8 tid, UWORD8 q_num,
                                    void *entry, UWORD16 msdu_len,
                                    UWORD8 num_frags, UWORD8 curr_tx_rate);
#ifndef OS_LINUX_CSL_TYPE
extern void amsdu_tx_timeout_handler(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void amsdu_tx_timeout_handler(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function stops the AMSDU Tx Timeout timer */
INLINE void stop_amsdu_timer(ALARM_HANDLE_T **hdl)
{
    if(*hdl != NULL)
        stop_alarm(*hdl);
}

/* This function starts the AMSDU Tx Timeout timer */
INLINE void start_amsdu_timer(ALARM_HANDLE_T **hdl, UWORD32 time, UWORD32 data)
{
    if(NULL == *hdl)
        *hdl = create_alarm(amsdu_tx_timeout_handler, data, NULL);      //Hugh
    start_alarm(*hdl, time);
}

INLINE void delete_amsdu_timer(ALARM_HANDLE_T **hdl)
{
    if(*hdl != NULL)
    {
        delete_alarm(hdl);
        *hdl = NULL;
    }
}
/* This function marks a AMSDU context structure as unused */
INLINE void free_amsdu_ctxt(amsdu_ctxt_t *amsdu_ctxt)
{
    if(NULL != amsdu_ctxt)
    {
        stop_amsdu_timer(&(amsdu_ctxt->to_alarm));
        amsdu_ctxt->in_use_flag = 0;
        amsdu_ctxt->invalid = 0;
	amsdu_ctxt->tx_dscr = NULL;
    }
}

/* This function check whether the AMSDU frame is ready for transmission */
INLINE BOOL_T check_amsdu_ready_status(amsdu_ctxt_t *ctxt)
{
    if((ctxt->num_msdu    == ctxt->max_num) ||
       (ctxt->num_entries == MAX_SUB_MSDU_TABLE_ENTRIES))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function checks whether the specified number of sub-MSDUs fit in the */
/* given A-MSDU.                                                             */
INLINE BOOL_T check_amsdu_space_status(amsdu_ctxt_t *ctxt, UWORD16 msdu_len,
                                       UWORD8 num_submsdu)
{
    UWORD16 sub_msdu_len = msdu_len + SUB_MSDU_HEADER_LENGTH;

    if((sub_msdu_len <= (ctxt->amsdu_maxsize - ctxt->amsdu_size)) &&
       (num_submsdu  <= (MAX_SUB_MSDU_TABLE_ENTRIES - ctxt->num_entries)))
        return BTRUE;

    return BFALSE;
}

/* This function adjusts the A-MSDU parameters before transmission */
INLINE void adjust_amsdu_before_txn(amsdu_ctxt_t *ctxt)
{
    buffer_desc_t buff_desc = {0};

    /* Remove the padding from the last sub-MSDU */
    if(ctxt->last_msdu_padlen)
    {
        get_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc),
                                      ctxt->num_entries - 1);

        buff_desc.data_length -= ctxt->last_msdu_padlen;
        ctxt->amsdu_size      -= ctxt->last_msdu_padlen;

        set_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc),
                                      ctxt->num_entries - 1);
    }

    /* Remove sub-MSDU header if only one sub-MSDU is present in the AMSDU */
    if(1 == ctxt->num_msdu)
    {
		UWORD32 sub_msdu_info = 0;
		UWORD32 *indep_sub_msdu_addr = NULL;
		
        get_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc),
                                      0);
        buff_desc.data_length -= SUB_MSDU_HEADER_LENGTH;
        buff_desc.data_offset += SUB_MSDU_HEADER_LENGTH;
        ctxt->amsdu_size      -= SUB_MSDU_HEADER_LENGTH;

		/* Move sub-MSDU table info from independent tx dscr buffer to current 
		 * tx dscr tail if only one sub-MSDU is present in the AMSDU --chengwg 
		 */
		indep_sub_msdu_addr = (UWORD32 *)get_tx_dscr_submsdu_info((UWORD32 *)ctxt->tx_dscr);
		sub_msdu_info = (UWORD32)(ctxt->tx_dscr + TX_DSCR_LEN);
		set_tx_dscr_submsdu_info((UWORD32 *)(ctxt->tx_dscr), sub_msdu_info);

        set_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc), 0);
                                      
        /* free useless sub msdu info table buffer */
		mem_free((mem_handle_t *)g_shared_dscr_mem_handle, (void *)indep_sub_msdu_addr);
        
        /* Reset AMSDU present bit in the MAC-Header */
        reset_amsdu_present_bit((UWORD8 *)get_tx_dscr_mac_hdr_addr(
                    (UWORD32 *)(ctxt->tx_dscr)));
        //printk("===num_msdu=%d, remove amsdu flag====\n", ctxt->num_msdu);
    }

#ifndef DEBUG_KLUDGE
    /* This Kludge fixes the TX-Hang issue in MAC H/w */
    {
        get_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc),
                                      ctxt->num_entries - 1);

        buff_desc.data_length += 4;

        set_tx_dscr_submsdu_buff_info((UWORD32 *)(ctxt->tx_dscr),
                                      (UWORD32)(&buff_desc),
                                      ctxt->num_entries - 1);
    }
#endif /* DEBUG_KLUDGE */

}

/* This function checks whether a AMSDU context already exists for the       */
/* specified RA/TID combination and returns the AMSDU context structure if   */
/* it does.                                                                  */
INLINE void *peek_amsdu_ctxt(void *rx_entry, UWORD8 tid)
{
    UWORD16 indx = 0;

    for(indx = 0; indx < MAX_NUM_SIMUL_AMSDU; indx++)
    {
        /* Check whether a AMSDU context structure exists for the specified */
        /* RA & TID combination                                             */
        if(1 == g_amsdu_ctxt[indx].in_use_flag)
        {
            if((g_amsdu_ctxt[indx].entry == rx_entry) &&
               (g_amsdu_ctxt[indx].priority_val == tid))
            {
                return (&g_amsdu_ctxt[indx]);
            }
        }
    }

    return NULL;
}

/* This function resets all AMSDU context structures */
INLINE void reset_amsdu_aggr(void)
{
    UWORD16 idx = 0;

    for(idx = 0; idx < MAX_NUM_SIMUL_AMSDU; idx++)
    {
        /* Free all A-MSDU related system resources and mark the context */
        /* as unused.                                                    */
        delete_amsdu_timer(&(g_amsdu_ctxt[idx].to_alarm));
        g_amsdu_ctxt[idx].in_use_flag = 0;
        /*jiangtao.yi add the following initiazation for bug242376*/
        g_amsdu_ctxt[idx].invalid = 0;
        g_amsdu_ctxt[idx].tx_dscr = 0;
    }
}
#endif /* AMSDU_AGGR_H */

#endif /* MAC_802_11N */
