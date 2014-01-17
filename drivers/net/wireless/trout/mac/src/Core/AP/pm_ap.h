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
/*  File Name         : pm_ap.h                                              */
/*                                                                           */
/*  Description       : This file contains all the power management related  */
/*                      structures, for AP mode of operation.                */
/*                                                                           */
/*  List of Functions : handle_ps_sta_change_to_active                       */
/*                      handle_ps_sta_change_to_powersave                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef PM_AP_H
#define PM_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "cglobals_ap.h"
#include "ap_prot_if.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    asoc_entry_t *ae;
    BOOL_T       q_legacy;
}pm_buff_element_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern BOOL_T is_ps_buff_pkt_ap(asoc_entry_t *ae, UWORD8 *da, void *data);
extern void update_ps_flags_ap(asoc_entry_t *ae, BOOL_T bc_mc_pkt,
                UWORD8 num_buff_added, BOOL_T ps_add_del_ac);
extern REQUEUE_STATUS_T requeue_ps_packet(asoc_entry_t *ae, list_buff_t *qh,
                                          BOOL_T ps_q_legacy, BOOL_T eosp);
extern void   handle_requeue_pending_packet(asoc_entry_t *ae, list_buff_t *qh,
                                            BOOL_T ps_q_legacy);
extern BOOL_T handle_ps_poll(UWORD8 *msg);
extern void   check_ps_state(asoc_entry_t* ae, STA_PS_STATE_T ps);
extern void   set_tim_bit(UWORD16 asoc_id);
extern void   check_and_reset_tim_bit(UWORD16 asoc_id);
// 20120709 caisf add, merged ittiam mac v1.2 code
//extern void handle_ps_tx_comp_ap(UWORD8 *tx_dscr, asoc_entry_t *ae);
extern void   handle_ps_tx_comp_ap(UWORD8 *tx_dscr, asoc_entry_t *ae, UWORD8 *msa);
extern void   tx_null_frame(UWORD8 *sa, asoc_entry_t *ae, BOOL_T is_qos,
                            UWORD8 priority, UWORD8 more_data);
extern asoc_entry_t *g_max_ps_ae;

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function handles the change of state of a station from power save to */
/* active state by requeing all buffered packets to the H/w queue.           */
/* It also reduces the global count of power save stations and requeues all  */
/* multicast/broadcast packets if this count becomes 0.                      */
INLINE void handle_ps_sta_change_to_active(asoc_entry_t *ae)
{
    REQUEUE_STATUS_T status = RE_Q_ERROR;

    handle_ba_active_ap_prot(ae);

    /* Continue to requeue buffered packets from the station's power save    */
    /* queue to the MAC H/w queue till the power save queue is empty.        */

    status = handle_ps_sta_change_to_active_prot(ae);

    if(status == PKT_NOT_REQUEUED)
    {
        return;
    }
    else
    {
        while((status=requeue_ps_packet(ae, &(ae->ps_q_lgcy), BTRUE, BFALSE))
            == PKT_REQUEUED);
#ifdef PS_DSCR_JIT
        /* This path is not valid currently                     */
        /* May be valid if descriptor are not buffered and are  */
        /* created just in time before transmission             */
        if(status == PKT_NOT_REQUEUED)
        {
            handle_requeue_pending_packet(ae, &g_ps_pending_q, BTRUE);
        }
#endif /* PS_DSCR_JIT */
    }

    /* To be safe flush all the PS queues. Though they should have been      */
    /* handled in the requeuing functions                                    */
    /* Reset other variables and TIM                                         */
    flush_ps_queues(ae);
    ae->ps_poll_rsp_qed = BFALSE;
    check_and_reset_tim_bit(ae->asoc_id);


    /* Decrement the global power save station count */
    g_num_sta_ps--;

    /* If there are no stations in Power save mode, requeue all multicast/   */
    /* broadcast packets.                                                    */
    if(g_num_sta_ps == 0)
    {
        while(requeue_ps_packet(NULL, &g_mc_q, BTRUE, BFALSE) == PKT_REQUEUED);
    }
}

/* This function handles the change of state of a station from active to     */
/* power save state by removing all packets from the H/w queue and buffering */
/* these in the S/w power save queue. Also ii increments the global count of */
/* power save stations. In case this is the only station in power save state */
/* the multicast/broadcast packets also need to be removed from the H/w      */
/* queues and buffered in the multicast queue.                               */
INLINE void handle_ps_sta_change_to_powersave(asoc_entry_t *ae)
{
    /* TBD. Remove packets from all H/w queue destined for this station and  */
    /* add to the ae queue.                                                  */

    if(g_num_sta_ps == 0)
    {
        /* TBD - no stations were in power save. mc/bc packets need to be    */
        /* removed and added to g_mc_q */
    }

    /* To be safe flush all the PS queues. Though they should have been      */
    /* handled in the requeuing functions                                    */
    /* Reset other variables and TIM                                         */
    flush_ps_queues(ae);
    ae->ps_poll_rsp_qed = BFALSE;

    /* Power Save buffering for 11n protocol related */
    handle_ps_sta_change_to_powersave_prot(ae);

    /* Increment the global power save station count */
    g_num_sta_ps++;
}

/* This function sets the AID0 bit or the Bit 7 in bit map control according */
/* to the type given as input.                                               */
INLINE void set_dtim_bit(UWORD8 type)
{
    if(type == AID0_BIT)
        g_vbmap[TIM_OFFSET]       |= 0x1;  /* AID = 0 */
    else
        g_vbmap[BMAP_CTRL_OFFSET] |= 0x1;  /* Set multicast bit */
}

/* This function resets the AID0 bit or Bit 7 in bit map control according   */
/* to the type given as input.                                               */
INLINE void reset_dtim_bit(UWORD8 type)
{
    if(type == AID0_BIT)
        g_vbmap[TIM_OFFSET]       &= 0xFE; /* AID = 0 */
    else
        g_vbmap[BMAP_CTRL_OFFSET] &= 0xFE; /* Reset multicast bit */
}

/* This function returns whether the Multicast bit is set in the given beacon*/
/* frame                                                                     */
INLINE BOOL_T get_mc_bit_bcn(UWORD8 bcn_idx)
{
    BOOL_T ret_value;
    ret_value = (BOOL_T)(g_beacon_frame[bcn_idx][g_tim_element_index + BMAP_CTRL_OFFSET] &
            0x01);
    return (ret_value);
}

/* This function returns whether the Multicast bit is set in the given beacon*/
/* frame                                                                     */
INLINE void reset_mc_bit_bcn(UWORD8 bcn_idx)
{
    g_beacon_frame[bcn_idx][g_tim_element_index + TIM_OFFSET] &= 0xFE;
    g_beacon_frame[bcn_idx][g_tim_element_index + BMAP_CTRL_OFFSET] &= 0xFE;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
/* This function sets the DTIM bit in the given beacon frame */
INLINE void set_mc_bit_bcn(UWORD8 bcn_idx)
{
    g_beacon_frame[bcn_idx][g_tim_element_index + TIM_OFFSET] 		|= 1;
    g_beacon_frame[bcn_idx][g_tim_element_index + BMAP_CTRL_OFFSET] |= 1;
}
/* This function sets the DTIM bit in the given beacon frame */

INLINE void set_tim_bit_bcn(UWORD8 bcn_idx, UWORD16 asoc_id)
{
    UWORD8 *tim_ie       = &g_beacon_frame[bcn_idx][g_tim_element_index];
    UWORD16 asoc_id_hi   = 0;
    UWORD16 asoc_id_lo   = 0;
    UWORD16 sta_byte_pos = 0;
    UWORD16 sta_bit_pos  = 0;

    asoc_id &= (~0xC000);

    /* Derive the range of Association IDs which are covered by the PV-Bitmap*/
    /* The lowest Asoc-ID is derived from the Bitmap-Offset field in TIM-IE. */
    /* The highest Asoc-ID is derived using the length of the  PV-Bitmap.    */
    asoc_id_lo = (tim_ie[BMAP_CTRL_OFFSET] & 0xFE) * 8;
    asoc_id_hi = asoc_id_lo + (tim_ie[LENGTH_OFFSET] - DEFAULT_TIM_LEN + 1) * 8;

    /* If the input Assoc-ID is not covered within the V-Bitmap, then return */
    /* This is done since only simple bit flips are allowed in the beacon    */
    /* which is currently with H/w.                                          */
    if((asoc_id >= asoc_id_hi) || (asoc_id < asoc_id_lo))
        return;

    sta_byte_pos = (asoc_id - asoc_id_lo) >> 3;
    sta_bit_pos  = (asoc_id & 0x7);

    tim_ie[TIM_OFFSET + sta_byte_pos] |= (1 << sta_bit_pos);
}

/* This function sets the DTIM bit in the given beacon frame */
INLINE void reset_tim_bit_bcn(UWORD8 bcn_idx, UWORD16 asoc_id)
{
    UWORD8 *tim_ie       = &g_beacon_frame[bcn_idx][g_tim_element_index];
    UWORD16 asoc_id_hi   = 0;
    UWORD16 asoc_id_lo   = 0;
    UWORD16 sta_byte_pos = 0;
    UWORD16 sta_bit_pos  = 0;

    asoc_id &= (~0xC000);

    /* Derive the range of Association IDs which are covered by the PV-Bitmap*/
    /* The lowest Asoc-ID is derived from the Bitmap-Offset field in TIM-IE. */
    /* The highest Asoc-ID is derived using the length of the  PV-Bitmap.    */
    asoc_id_lo = (tim_ie[BMAP_CTRL_OFFSET] & 0xFE) * 8;
    asoc_id_hi = asoc_id_lo + (tim_ie[LENGTH_OFFSET] - DEFAULT_TIM_LEN + 1) * 8;

    /* If the input Assoc-ID is not covered within the V-Bitmap, then return */
    /* This is done since only simple bit flips are allowed in the beacon    */
    /* which is currently with H/w.                                          */
    if((asoc_id >= asoc_id_hi) || (asoc_id < asoc_id_lo))
        return;

    sta_byte_pos = (asoc_id - asoc_id_lo) >> 3;
    sta_bit_pos  = (asoc_id & 0x7);

    tim_ie[TIM_OFFSET + sta_byte_pos] &= ~(1 << sta_bit_pos);
}


#ifdef ENABLE_PS_PKT_FLUSH
/* Note: This algorithm of finding the station with maximum PS packets  */
/* is sub-optimal, but is OK for the current number of PS pkt limit (5) */

/* Update tha maximim and second maximum PS STA entries */
/* when a PS-pkt count of a STA is changing */
INLINE void find_max_ps_ae(void)
{
    UWORD8        i        = 0;
    asoc_entry_t  *ae      = 0;
    table_elmnt_t *tbl_elm = 0;

   /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            ae = tbl_elm->element;

            /* Increment the aging count for this STA if it is not in PS */
            if(ae->ps_state != ACTIVE_PS)
            {
                if(g_max_ps_ae == NULL)
                {
                   g_max_ps_ae = ae;
                }
                else if(ae->num_ps_pkt_lgcy > g_max_ps_ae->num_ps_pkt_lgcy)
                {
                   g_max_ps_ae = ae;
                }
            }
            else if(g_max_ps_ae == ae)
            {
                g_max_ps_ae = NULL;
            }

            tbl_elm = tbl_elm->next_hash_elmnt;
        }
    }

}

/* Update tha maximim and second maximum PS STA entries */
/* when a STA is leaving */
INLINE void handle_max_ps_ae(asoc_entry_t *ae)
{
    if(g_max_ps_ae == ae)
    {
        g_max_ps_ae = NULL;
    }
}
#endif /* ENABLE_PS_PKT_FLUSH */

#endif /* PM_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
