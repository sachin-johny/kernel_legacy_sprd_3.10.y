/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : ap_uapsd.h                                           */
/*                                                                           */
/*  Description       :                                                      */
/*  List of Functions : get_ac_vo_trigger                                    */
/*                      get_ac_vo_delivery                                   */
/*                      get_ac_vi_trigger                                    */
/*                      get_ac_vi_delivery                                   */
/*                      get_ac_be_trigger                                    */
/*                      get_ac_be_delivery                                   */
/*                      get_ac_bk_trigger                                    */
/*                      get_ac_bk_delivery                                   */
/*                      set_ac_vo_trigger                                    */
/*                      set_ac_vo_delivery                                   */
/*                      set_ac_vi_trigger                                    */
/*                      set_ac_vi_delivery                                   */
/*                      set_ac_be_trigger                                    */
/*                      set_ac_be_delivery                                   */
/*                      set_ac_bk_trigger                                    */
/*                      set_ac_bk_delivery                                   */
/*                      reset_ac_bk_trigger                                  */
/*                      reset_ac_bk_delivery                                 */
/*                      reset_ac_be_trigger                                  */
/*                      reset_ac_be_delivery                                 */
/*                      reset_ac_vi_trigegr                                  */
/*                      reset_ac_vi_delivery                                 */
/*                      reset_ac_vo_trigger                                  */
/*                      reset_ac_vo_delivery                                 */
/*                      set_max_sp                                           */
/*                      get_ACMbit_config                                    */
/*                      revert_ac_uapsd_status                               */
/*                      revert_current_uapsd_status                          */
/*                      is_ap_uapsd_capable                                  */
/*                      is_all_ac_del                                        */
/*                      get_max_sp_length                                    */
/*                      incr_current_sp_length                               */
/*                      get_current_sp_length                                */
/*                      store_current_sp_length                              */
/*                      reset_current_sp_length                              */
/*                      is_USP_in_progress                                   */
/*                      is_end_of_USP                                        */
/*                      end_current_USP                                      */
/*                      uapsd_reset_tim                                      */
/*                      set_eosp                                             */
/*                      reset_uapsd_entry                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_WMM

#ifndef AP_UAPSD_H
#define AP_UAPSD_H

/* Bit values for UAPSD STATUS of each AC in a STA */

/****************************************************************************/
  /* Format of U-APSD Status bits                                           */
  /* ---------------------------------------------------------------------- */
  /* |     AC_BK      |       AC_BE     |      AC_VI     |      AC_VO     | */
  /* ---------------------------------------------------------------------- */
  /* |Delivery|Trigger|Delivery|Trigger |Delivery|Trigger|Delivery|Trigger| */
  /* ---------------------------------------------------------------------- */
  /* | B7     |  B6   |  B5    |   B4   |   B3   |   B2  |  B1    |  B0   | */
  /* ---------------------------------------------------------------------- */
/****************************************************************************/

#define AC_VO_TRIGGER                 BIT0
#define AC_VO_DELIVERY                BIT1
#define AC_VI_TRIGGER                 BIT2
#define AC_VI_DELIVERY                BIT3
#define AC_BE_TRIGGER                 BIT4
#define AC_BE_DELIVERY                BIT5
#define AC_BK_TRIGGER                 BIT6
#define AC_BK_DELIVERY                BIT7

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management_ap.h"
#include "common.h"
#include "frame_11e.h"
#include "queue.h"
#include "event_manager.h"

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void    update_asoc_uapsd_status(asoc_entry_t *ae, UWORD8 *msa,
                                        UWORD16 rx_len);
extern void    ap_enabled_wmm_rx_action(UWORD8 *msg);
extern void    uapsd_handle_pwr_mgt(asoc_entry_t *ae, UWORD8 priority,
                                   UWORD8 *msa);
extern void    reorganize_ps_q(asoc_entry_t *ae, UWORD8 ac, list_buff_t *src_q,
                               list_buff_t *dest_q, BOOL_T is_src_legacy_q);
extern BOOL_T  check_ac_is_del_en(asoc_entry_t *ae,UWORD8 priority);
extern BOOL_T  check_ac_is_trigger_enabled (asoc_entry_t *ae,UWORD8 priority);

extern REQUEUE_STATUS_T uapsd_handle_ps_sta_change_to_active(asoc_entry_t *ae);
extern void ap_uapsd_handle_rx_addts_req(UWORD8 *msa, UWORD16 len, UWORD8 *sa,
                                          UWORD8 idx);
extern void ap_uapsd_handle_rx_delts(UWORD8 *msa, UWORD8 *sa, UWORD8 idx);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* Functions to get UAPSD status of each AC for each STA */

INLINE BOOL_T get_ac_vo_trigger(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_VO_TRIGGER)
       == AC_VO_TRIGGER)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_vo_delivery(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_VO_DELIVERY)
       == AC_VO_DELIVERY)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_vi_trigger(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_VI_TRIGGER)
       == AC_VI_TRIGGER)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_vi_delivery(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_VI_DELIVERY)
       == AC_VI_DELIVERY)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_be_trigger(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_BE_TRIGGER)
       == AC_BE_TRIGGER)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_be_delivery(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_BE_DELIVERY)
       == AC_BE_DELIVERY)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

INLINE BOOL_T get_ac_bk_trigger(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_BK_TRIGGER)
       == AC_BK_TRIGGER)
   {
       return BTRUE;
   }
   else
        return BFALSE;
}

INLINE BOOL_T get_ac_bk_delivery(asoc_entry_t *ae)
{
   if((ae->uapsd_status.ac_current_uapsd_status & AC_BK_DELIVERY)
       == AC_BK_DELIVERY)
   {
       return BTRUE;
   }
   else
       return BFALSE;
}

/* Function to check if all AC s of the STA are delivery enabled or not */
INLINE BOOL_T is_all_ac_del(asoc_entry_t *ae)
{
    if((get_ac_vo_delivery(ae) && get_ac_vi_delivery(ae) &&
       get_ac_bk_delivery(ae) && get_ac_be_delivery(ae)) == BTRUE)
    {
        return BTRUE;
    }
    else
        return BFALSE;
}

/* Functions to set current UAPSD status of each AC for each STA */

INLINE void set_ac_vo_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status |= AC_VO_TRIGGER;
}

INLINE void set_ac_vo_delivery(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status |= AC_VO_DELIVERY;
}

INLINE void set_ac_vi_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status |= AC_VI_TRIGGER;
}

INLINE void set_ac_vi_delivery(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status |= AC_VI_DELIVERY;
}

INLINE void set_ac_be_trigger(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status |= AC_BE_TRIGGER;
}

INLINE void set_ac_be_delivery(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status |= AC_BE_DELIVERY;
}

INLINE void set_ac_bk_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status |= AC_BK_TRIGGER;
}

INLINE void set_ac_bk_delivery(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status |= AC_BK_DELIVERY;
}

/* Function to reset the current uapsd status of each AC for each STA */

INLINE void reset_ac_vo_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status &= (AC_VO_TRIGGER ^ 0xFF);
}

INLINE void reset_ac_vo_delivery(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status &= (AC_VO_DELIVERY^0xFF);
}

INLINE void reset_ac_vi_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status &= (AC_VI_TRIGGER^0xFF);
}

INLINE void reset_ac_vi_delivery(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status &= (AC_VI_DELIVERY^0xFF);
}

INLINE void reset_ac_be_trigger(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status &= (AC_BE_TRIGGER^0xFF);
}

INLINE void reset_ac_be_delivery(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status &= (AC_BE_DELIVERY^0xFF);
}

INLINE void reset_ac_bk_trigger(asoc_entry_t *ae)
{
   ae->uapsd_status.ac_current_uapsd_status &= (AC_BK_TRIGGER^0xFF);
}

INLINE void reset_ac_bk_delivery(asoc_entry_t *ae)
{
  ae->uapsd_status.ac_current_uapsd_status &= (AC_BK_DELIVERY^0xFF);
}

/* Function to revert the current uapsd status of each AC for each STA */
/* On receiving a delts frame */

INLINE void revert_ac_uapsd_status(asoc_entry_t *ae, UWORD8 ac, UWORD8 trigger,
                                   UWORD8 delivery)
{
    UWORD8 trigger_status  = 0;
    UWORD8 current_delivery_status = 0;
    UWORD8 updated_delivery_status = 0;

    /* Clear the current AC_** Trigger status bit */
    ae->uapsd_status.ac_current_uapsd_status &= (trigger ^ 0xFF);

    /* Get the AC_** Trigger bit from asoc entry status */
    trigger_status = ae->uapsd_status.ac_asoc_uapsd_status & trigger ;
    /* Update the current AC_** Trigger status bit */
    ae->uapsd_status.ac_current_uapsd_status |= trigger_status;

    /* Get the AC_** current delivery uapsd status */
    current_delivery_status =
                           ae->uapsd_status.ac_current_uapsd_status & delivery;

    /* Clear the current AC_** Delivery bit status */
    ae->uapsd_status.ac_current_uapsd_status &= (delivery ^ 0xFF);

    /* Get the AC_VO Delivery bit from asoc entry status */
    updated_delivery_status = ae->uapsd_status.ac_asoc_uapsd_status & delivery;
     /* Update the current AC_** Delivery status bit */
    ae->uapsd_status.ac_current_uapsd_status |= updated_delivery_status;

    /* Reorganize packets in the queue */
    if(current_delivery_status != updated_delivery_status)
    {
        if( current_delivery_status == delivery)
        {
            reorganize_ps_q(ae, ac, &ae->ps_q_del_ac, &ae->ps_q_lgcy, BFALSE);
        }
        else
        {
            reorganize_ps_q(ae, ac, &ae->ps_q_lgcy, &ae->ps_q_del_ac, BTRUE);
        }
    }
}


/* Functions to set max Sp length of the STA */
INLINE void set_max_sp(asoc_entry_t *ae, UWORD8 val)
{
    /* Max SP Length Usage */
    /*-------------------------------------*/
    /* |Bit6 |Bit 5|        Usage         |*/
    /*-------------------------------------*/
    /* |  0  |  0  | all buffered frames  |*/
    /* |  0  |  1  |max 2 buffered frames |*/
    /* |  1  |  0  |max 4 buffered frames |*/
    /* |  1  |  1  |max 6 buffered frames |*/
    /*-------------------------------------*/

    UWORD8 max_sp_length =((val & 0x60)>>5);
    switch(max_sp_length)
    {
        case 0:
        {
            ae->uapsd_status.max_sp_len = 0;
        }
        break;
        case 1:
        {
            ae->uapsd_status.max_sp_len = 2;
        }
        break;
        case 2:
        {
            ae->uapsd_status.max_sp_len = 4;
        }
        break;
        case 3:
        {
            ae->uapsd_status.max_sp_len = 6;
        }
        break;
        default:
        {
            ae->uapsd_status.max_sp_len = 0;
        }
        break;
    }

}


INLINE void revert_current_uapsd_status(asoc_entry_t *ae, UWORD8 priority)
{
    UWORD8 ac = 0;

    map_priority_to_edca_ac(priority, &ac);
    switch(ac)
    {
        case AC_VO:
        {
            revert_ac_uapsd_status(ae, AC_VO, AC_VO_TRIGGER, AC_VO_DELIVERY);
        }
        break;
        case AC_VI:
        {
            revert_ac_uapsd_status(ae, AC_VI, AC_VI_TRIGGER, AC_VI_DELIVERY);
        }
        case AC_BE:
        {
            revert_ac_uapsd_status(ae, AC_BE, AC_BE_TRIGGER, AC_BE_DELIVERY);
        }
        break;
        case AC_BK:
        {
            revert_ac_uapsd_status(ae, AC_BK, AC_BK_TRIGGER, AC_BK_DELIVERY);
        }
        break;
        default:
        {
            return ;
        }
    }
}

/* Function returns true if ap is UAPSD capable */
INLINE BOOL_T is_ap_uapsd_capable(void)
{
    return mget_UAPSD_ap();
}

/* Sets the End of service period  */
INLINE void set_eosp(UWORD8 *header)
{
    header[QOS_CTRL_FIELD_OFFSET] |= 0x10;
}

/* Reset the uapsd related association entry elements */
INLINE void reset_uapsd_entry(asoc_entry_t *ae)
{
    mem_set(&(ae->uapsd_status), 0, sizeof(uapsd_status_t));
    ae->USP_in_progress   = BFALSE;
    ae->num_ps_pkt_del_ac = 0;
    ae->num_USP_pkts_qed = 0;
}

#endif /* AP_UAPSD_H */

#endif /* MAC_WMM */

#endif /* ACCESS_MODE */
