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
/*  File Name         : ap_uapsd.c                                           */
/*                                                                           */
/*  Description       :                                                      */
/*  List of Functions : update_asoc_uapsd_status                             */
/*                      update_current_uapsd_status                          */
/*                      set_ac_asoc_uapsd_status                             */
/*                      check_ac_is_trigger_enabled                          */
/*                      check_ac_is_del_en                                   */
/*                      uapsd_requeue_ps_pkt                                 */
/*                      ap_enabled_wmm_rx_action                             */
/*                      ap_uapsd_handle_rx_addts_req                         */
/*                      update_ac_uapsd_status                               */
/*                      ap_uapsd_handle_rx_delts                             */
/*                      uapsd_handle_pwr_mgt                                 */
/*                      reorganize_ps_q                                      */
/*                      uapsd_handle_ps_sta_change_to_active                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ap_uapsd.h"
#include "pm_ap.h"
#include "ap_frame_11e.h"
#include "host_if.h"
#include "receive.h"
#include "iconfig.h"
#include "core_mode_if.h"


/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static void update_ac_uapsd_status(UWORD8 *tsinfo, asoc_entry_t *ae);
static void set_ac_asoc_uapsd_status(asoc_entry_t *ae, UWORD8 *msa,
                                     UWORD8 index);
static void update_current_uapsd_status(UWORD8 priority, TS_DIRECTION_T dir,
                                         UWORD8 PSB_bit, asoc_entry_t *ae);
/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ac_asoc_uapsd_status                                 */
/*                                                                           */
/*  Description   : This function sets the U-APSD related parameters in the  */
/*                  association entry table                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to association table                          */
/*                  2) Pointer to the rx msg                                 */
/*                  3) Index of Qos field                                    */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    : TBD                                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_ac_asoc_uapsd_status(asoc_entry_t *ae, UWORD8 *msa, UWORD8 index)
{
    /* Qos Info field sent from STA */
    /* -------------------------------------------------------------- */
    /* | AC_VO | AC_VI | AC_BK | AC_BE |         | Max SP|          | */
    /* | U-APSD| U-APSD| U-APSD| U-APSD| Reserved| Length| Reserved | */
    /* | Flag  | Flag  | Flag  | Flag  |         |       |          | */
    /* -------------------------------------------------------------- */
    /* | B0    | B1    | B2    | B3    | B4      | B5 B6 | B7       | */
    /* -------------------------------------------------------------- */
    ae->uapsd_status.ac_asoc_uapsd_status = 0;
    if((msa[index] & BIT0) == BIT0)
    {
        ae->uapsd_status.ac_asoc_uapsd_status |= AC_VO_TRIGGER;
        ae->uapsd_status.ac_asoc_uapsd_status |= AC_VO_DELIVERY;
    }
    if((msa[index] & BIT1) == BIT1)
    {
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_VI_TRIGGER;
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_VI_DELIVERY;
    }
    if((msa[index] & BIT2) == BIT2)
    {
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_BK_TRIGGER;
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_BK_DELIVERY;
    }
    if((msa[index] & BIT3) == BIT3)
    {
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_BE_TRIGGER;
          ae->uapsd_status.ac_asoc_uapsd_status |= AC_BE_DELIVERY;
    }

    /* Current status of UAPSD at Association is same as the status at       */
    /* Association                                                           */
    ae->uapsd_status.ac_current_uapsd_status =
                                         ae->uapsd_status.ac_asoc_uapsd_status;

    /* Set max SP length */
    set_max_sp(ae, msa[index]);

    /* Initialize USP_in_progress flag */
    ae->USP_in_progress = BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_asoc_uapsd_status                                 */
/*                                                                           */
/*  Description   : This function updates the U-APSD parameters in the       */
/*                  association entry table with the values obtained from    */
/*                  the association response frame                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to association table                          */
/*                  2) Pointer to the rx msg                                 */
/*                  3) Length of the msg received                            */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_asoc_uapsd_status(asoc_entry_t *ae, UWORD8 *msa, UWORD16 rx_len)
{
    UWORD32     index       = 0;

    /*The msg received is an association request msg*/
    index  = MAC_HDR_LEN + CAP_INFO_LEN + LISTEN_INT_LEN;

	// 20120709 caisf add, merged ittiam mac v1.2 code
	/* If the frame is Re-association request, add 6 bytes to the offset */
	/* since this frame includes an extra field (Current AP Address)     */
	if(get_sub_type(msa) == REASSOC_REQ)
	{
		index += MAC_ADDRESS_LEN;
	}

    /* Check if the WMM Information element is present in the frame */
    while(index < (rx_len - FCS_LEN))
    {
        if(is_wmm_info_param_elem(msa + index) == BTRUE)
        {
            set_ac_asoc_uapsd_status(ae, msa,(index+8));
            PRINTD("STA UAPSD status       = %02x\n",
                ae->uapsd_status.ac_asoc_uapsd_status);
            PRINTD("Service Period         = %d\n",
                ae->uapsd_status.max_sp_len);

            break;
        }
        index += (2 + msa[index + 1]);
    }

    /* Initialize number of pkts queued for delivery ac s of the STA */
    ae->num_ps_pkt_del_ac = 0;
    /* Initilaze the uapsd power save Q */
    init_list_buffer(&(ae->ps_q_del_ac),
        TX_DSCR_NEXT_ADDR_WORD_OFFSET * sizeof(UWORD32));
  }

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reorganize_ps_q                                          */
/*                                                                           */
/*  Description   : This function removes a packet buffered in the source    */
/*                  queue and adds it to the destination queue if required   */
/*                  else adds it back to the source queue                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to association entry of STA                   */
/*                  2) Priority                                              */
/*                  3) Pointer to legacy power save Q                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : A packet is removed from the source queue,               */
/*                  if the corresponding AC has changed its state from legacy*/
/*                  to delivery or vice versa. The packet is then buffered   */
/*                  in the corresponding queue                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void reorganize_ps_q(asoc_entry_t *ae, UWORD8 desired_ac, list_buff_t *src_q,
                     list_buff_t *dest_q, BOOL_T is_src_legacy_q)
{
    UWORD8      curr_el_ac          = 0;
    UWORD8      curr_el_priority    = 0;
    UWORD32     no_of_elements      = 0;
    void        *curr_el            = NULL;
    void        *prev_el            = NULL;

    /* If q empty return */
    if(src_q->head == 0)
    {
        return;
    }

    /*************************************************************************/
    /* Parse the entire queue and match the AC. If the AC is to be moved     */
    /* then move it to the destination queue                                 */
    /*************************************************************************/
    curr_el = peek_list(src_q);

    while( NULL != curr_el)
    {
        curr_el_priority = get_tx_dscr_priority(curr_el);

        map_priority_to_edca_ac(curr_el_priority, &curr_el_ac);

        if(curr_el_ac == desired_ac)
        {
            remove_list_element(src_q, prev_el, curr_el);
            add_list_element(dest_q, curr_el);
            no_of_elements++ ;
        }
        else
        {
            /*****************************************************************/
            /* The prev_el doesn't changes if the curr_el was removed        */
            /* Should be updated other wise                                  */
            /*****************************************************************/
            prev_el = curr_el;
        }

        curr_el = next_element_list(src_q, prev_el);
    }

    /*************************************************************************/
    /* Update the tim element based on the movement                          */
    /* Legacy queue -> Delivery enabled                                      */
    /*   - if after the movement legacy Q is empty and not all AC's are      */
    /*     delivery enabled reset the TIM bit                                */
    /* Delivery enabled -> Legacy queue                                      */
    /*   - Now atleast 1 Q is legacy. Set the TIM bit if non zero number     */
    /*     of packets are added. TIM may be already set (Don't care).        */
    /*************************************************************************/
    if(is_src_legacy_q == BTRUE)
    {
        ae->num_ps_pkt_lgcy -= no_of_elements;
        ae->num_ps_pkt_del_ac += no_of_elements;
        /* Check if all queues are delivery enabled */
        if((ae->num_ps_pkt_lgcy == 0) &&
           (BFALSE == check_legacy_flag_prot(ae)))
        {
            check_and_reset_tim_bit(ae->asoc_id);
        }
    }
    else
    {
        ae->num_ps_pkt_lgcy += no_of_elements;
        ae->num_ps_pkt_del_ac -= no_of_elements;

        if(no_of_elements > 0)
            set_tim_bit(ae->asoc_id);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_current_uapsd_status                              */
/*                                                                           */
/*  Description   : This function handles the wlan_rx request for U-APSD     */
/*                  operation                                                */
/*                                                                           */
/*  Inputs        : 1) Priority                                              */
/*                  2) Direction                                             */
/*                  3) Power save bit in ts-info field                       */
/*                  4) Pointer to association entry                          */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_current_uapsd_status(UWORD8 priority, TS_DIRECTION_T dir,
                                 UWORD8 PSB_bit, asoc_entry_t *ae)
{
    UWORD8 ac = 0;

    map_priority_to_edca_ac(priority, &ac);
    switch(ac)
    {
        case AC_VO:
        {

            if((dir == UPLINK) && (PSB_bit==1))
            {
                set_ac_vo_trigger(ae);

            }
            else if((dir == DOWNLINK) && (PSB_bit==1))
            {
                set_ac_vo_delivery(ae);
                reorganize_ps_q(ae, AC_VO, &ae->ps_q_lgcy, &ae->ps_q_del_ac,
                                BTRUE);
            }
            else if(dir == BIDIR)
            {
                if(PSB_bit == 1)
                {
                    set_ac_vo_trigger(ae);
                    set_ac_vo_delivery(ae);
                    reorganize_ps_q(ae, AC_VO, &ae->ps_q_lgcy,
                                    &ae->ps_q_del_ac, BTRUE);
                }
                else if(PSB_bit == 0)
                {
                    reset_ac_vo_trigger(ae);
                    reset_ac_vo_delivery(ae);
                    reorganize_ps_q(ae, AC_VO, &ae->ps_q_del_ac,
                                    &ae->ps_q_lgcy, BFALSE);
                }
            }
        }
        break;
        case AC_VI:
        {

            if((dir == UPLINK) && (PSB_bit==1))
            {
                set_ac_vi_trigger(ae);
            }
            else if((dir == DOWNLINK) && (PSB_bit==1))
            {
                set_ac_vi_delivery(ae);
                reorganize_ps_q(ae, AC_VI, &ae->ps_q_lgcy, &ae->ps_q_del_ac,
                                BTRUE);
            }
            else if(dir == BIDIR)
            {
                if(PSB_bit == 1)
                {
                    set_ac_vi_trigger(ae);
                    set_ac_vi_delivery(ae);
                    reorganize_ps_q(ae, AC_VI, &ae->ps_q_lgcy,
                                    &ae->ps_q_del_ac, BTRUE);
                }
                else if(PSB_bit == 0)
                {
                    reset_ac_vi_trigger(ae);
                    reset_ac_vi_delivery(ae);
                    reorganize_ps_q(ae, AC_VI, &ae->ps_q_del_ac,
                                    &ae->ps_q_lgcy, BFALSE);
                }
            }
        }
        break;
        case AC_BE:
        {

            if((dir == UPLINK) && (PSB_bit==1))
            {
                set_ac_be_trigger(ae);
            }
            else if((dir == DOWNLINK) && (PSB_bit==1))
            {
                set_ac_be_delivery(ae);
                reorganize_ps_q(ae, priority, &ae->ps_q_lgcy, &ae->ps_q_del_ac,
                                BTRUE);
            }
            else if(dir == BIDIR)
            {
                if(PSB_bit == 1)
                {
                    set_ac_be_trigger(ae);
                    set_ac_be_delivery(ae);
                    reorganize_ps_q(ae, priority, &ae->ps_q_lgcy,
                                    &ae->ps_q_del_ac, BTRUE);
                }
                else if(PSB_bit == 0)
                {
                    reset_ac_be_trigger(ae);
                    reset_ac_be_delivery(ae);
                    reorganize_ps_q(ae, priority, &ae->ps_q_del_ac,
                                    &ae->ps_q_lgcy, BFALSE);
                }
            }
        }
        break;
        case AC_BK:
        {

            if((dir == UPLINK) && (PSB_bit==1))
            {
                set_ac_be_trigger(ae);
            }
            else if((dir == DOWNLINK) && (PSB_bit==1))
            {
                set_ac_be_delivery(ae);
                reorganize_ps_q(ae, priority, &ae->ps_q_lgcy, &ae->ps_q_del_ac,
                                BTRUE);
            }
            else if(dir == BIDIR)
            {
                if(PSB_bit == 1)
                {
                    set_ac_bk_trigger(ae);
                    set_ac_bk_delivery(ae);
                    reorganize_ps_q(ae, priority, &ae->ps_q_lgcy,
                                    &ae->ps_q_del_ac, BTRUE);
                }
                else if(PSB_bit == 0)
                {
                    reset_ac_bk_trigger(ae);
                    reset_ac_bk_delivery(ae);
                    reorganize_ps_q(ae, priority, &ae->ps_q_del_ac,
                                    &ae->ps_q_lgcy, BFALSE);
                }
            }
        }
        break;
        default:
        {
            /* do nothing */
        }
     }
 }

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_ac_is_del_en                                       */
/*                                                                           */
/*  Description   : This function returns true if the ac is delivery enabled */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1)Pointer to the association entry of the sta            */
/*                  2)Priority                                               */
/*                                                                           */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : TRUE if Access category is delivery enabled              */
/*                  else returns FALSE                                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T check_ac_is_del_en (asoc_entry_t *ae, UWORD8 priority)
{
   UWORD8 ac=0;
   BOOL_T delivery_enabled = BFALSE;

   map_priority_to_edca_ac(priority, &ac);

   switch(ac)
   {
    case AC_BK:
    {
        delivery_enabled = get_ac_bk_delivery(ae);
    }
    break;
    case AC_BE:
    {
        delivery_enabled = get_ac_be_delivery(ae);
    }
    break;
    case AC_VI:
    {
        delivery_enabled = get_ac_vi_delivery(ae);
    }
    break;
    case AC_VO:
    {
        delivery_enabled = get_ac_vo_delivery(ae);
    }
    break;
    default:
    {
        delivery_enabled = BFALSE;
    }
   }

   return delivery_enabled;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_ac_is_trigger_enabled                              */
/*                                                                           */
/*  Description   : This function returns true if the ac is trigger enabled  */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1)Pointer to the association entry of the sta            */
/*                  2)Priority                                               */
/*                                                                           */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : TRUE if Access category is trigger enabled               */
/*                  else returns FALSE                                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T check_ac_is_trigger_enabled (asoc_entry_t *ae, UWORD8 priority)
{
   UWORD8 ac=0;
   BOOL_T trigger_enabled = BFALSE;

   map_priority_to_edca_ac(priority, &ac);

   switch(ac)
   {
    case AC_BK:
    {
        trigger_enabled = get_ac_bk_trigger(ae);
    }
    break;
    case AC_BE:
    {
        trigger_enabled = get_ac_be_trigger(ae);
    }
    break;
    case AC_VI:
    {
        trigger_enabled = get_ac_vi_trigger(ae);
    }
    break;
    case AC_VO:
    {
        trigger_enabled = get_ac_vo_trigger(ae);
    }
    break;
    default:
    {
        trigger_enabled = BFALSE;
    }
   }

   return trigger_enabled;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_uapsd_handle_rx_addts_req                             */
/*                                                                           */
/*  Description   : This function handles the incoming ADDTS request frame.  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
/*                  2) Length of the message                                 */
/*                  3) MAC adress of non-AP QSTA sending the request         */
/*                  4) Index of the non-AP QSTA                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The incoming frame dialog token is saved                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void ap_uapsd_handle_rx_addts_req(UWORD8 *msa, UWORD16 len, UWORD8 *sa,
                                  UWORD8 idx)
{
    UWORD32 status    = 0;
    UWORD8  *frame    = 0;
    UWORD8  *tspec    = 0;
    UWORD8  *tsinfo   = 0;
    UWORD16 frame_len = 0;
    asoc_entry_t *ae  = NULL;

    ae = (asoc_entry_t *)find_entry(sa);

    /* Note: The addts req frame is handled only for powersave */
    /* it is not handled for admission control */
    /* Since AP does not support Admission control as of now */
    /* Allocate memory for the ADDTS response frame */
    frame = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);

    if(frame == NULL)
    {
        return;
    }

    /******************************************************/
    /*        ADDTS Request Frame - Frame Body            */
    /* -------------------------------------------------- */
    /* | Category | Action | Dialog Token | TSPEC | FCS | */
    /* -------------------------------------------------- */
    /* | 1        | 1      | 1            | 61    |  4  | */
    /* -------------------------------------------------- */
    /*                                                    */
    /******************************************************/

    /* Extract the TSPEC element from the incoming ADDTS request frame */
    tspec = get_addts_req_tspec(msa);

    /* Extract the TS Info received */
    tsinfo = get_tsinfo(tspec);

    /* Depending on the TSInfo update the uapsd status  */
    /* of the corresponding AC */
    update_ac_uapsd_status(tsinfo, ae);

    /* Prepare the ADDTS Response frame with the status */
    frame_len = prepare_wmm_addts_rsp(frame, msa, len, sa, status);

    /* Transmit the management frame */
    tx_mgmt_frame(frame, frame_len, AC_VO_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_uapsd_handle_rx_delts                                 */
/*                                                                           */
/*  Description   : This function handles the incoming DELTS frame.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : UAPSD status of the corresponding  AC of the associated  */
/*                  STA will be reset                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void ap_uapsd_handle_rx_delts(UWORD8 *msa, UWORD8 *sa, UWORD8 idx)
{
    UWORD8 priority         = 0;
    asoc_entry_t *ae        = NULL;
    UWORD8  *tsinfo         = 0;

    ae=(asoc_entry_t *)find_entry(sa);

    /* Extract the TS Info field */
    tsinfo = get_delts_req_tsinfo(msa);

    /*Extract priority */
    priority=((tsinfo[1] & 0x38)>>3);


    /*No processing required except that the */
    /*uapsd status of the particular AC has to be reverted*/
    /*back to the static U-APSD settings during Association */

    revert_current_uapsd_status(ae, priority);

    /* RESET the USP_in_progress flag */
    ae->USP_in_progress = BFALSE;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_ac_uapsd_status                                   */
/*                                                                           */
/*  Description   : This function updates the uapsd status of the ac         */
/*                  from the TSinfo                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to tsinfo                                     */
/*                  2) Pointer to association entry of STA                   */
/*                                                                           */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_ac_uapsd_status(UWORD8 *tsinfo, asoc_entry_t *ae)
{
    UWORD8 priority     = 0;
    TS_DIRECTION_T dir  = UPLINK;
    UWORD8 PSB_bit      = 0;

     /********************************************************************/
     /*                    TSInfo Field                                  */
     /* ---------------------------------------------------------------- */
     /* | Reserved|Res |Reserved| UP    |PSB |Res| 0| 1|Dir |  TID |Res| */
     /* ---------------------------------------------------------------- */
     /* | B23-B17 |B16 |B15-B14 |B13-B11|B10 |B9 |B8|B7|B6-B5|B4-B1|B0 | */
     /* ---------------------------------------------------------------- */
     /*                                                                  */
     /********************************************************************/

    /*Extract priority */
    priority=((tsinfo[1] & 0x38)>>3);

    /*Extract direction*/
    dir = (TS_DIRECTION_T)((tsinfo[0] & 0x60)>>5);

    /*Extract PSB bit  */
    PSB_bit = ((tsinfo[1] & BIT2)>>2);

    /* Update the uapsd status of the corresponding AC */
    update_current_uapsd_status(priority, dir, PSB_bit, ae);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_requeue_ps_pkt                                     */
/*                                                                           */
/*  Description   :This function requeues the packets from uapsd power save  */
/*                 queue to the H/w queue.                                   */
/*                                                                           */
/*  Inputs        :                                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : If the data is received from a trigger enabled AC of the */
/*                  Source STA then the packets queued for the delivery      */
/*                  enabled AC s of the STA are requeued from the power save */
/*                  buffer to the h/w queue.The number of packets requeued   */
/*                  depends on the maximum Service Period length of the STA  */
/*                  After requeuing the packets,it resets the USP_in_progress*/
/*                  flag and sends a QOS null frame with EOSP bit set to 1   */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void uapsd_requeue_ps_pkt(asoc_entry_t *ae, UWORD8 priority, UWORD8 *msa)
{
    UWORD8 i                     = 0;
    UWORD8 max_sp_length         = ae->uapsd_status.max_sp_len;
    BOOL_T eosp                  = BFALSE;
    UWORD8 sa[6]                 = {0};

   /* Extract the source address from the frame */
    get_SA(msa, sa);

    ae->num_USP_pkts_qed = 0;

    /* If no packet in queue then send qos null frame */
    if(ae->ps_q_del_ac.head == 0)
    {
        /* Send a QOS NULL frame */
        tx_null_frame(sa, ae, BTRUE, priority, 0);
        ae->num_USP_pkts_qed = 1;
        return;
    }

   /* If max sp is 0 then all buffered packets have to be transmitted*/
    if(max_sp_length == 0)
    {
        while( PKT_REQUEUED == requeue_ps_packet(ae, &ae->ps_q_del_ac, BFALSE,
                                                 BFALSE))
        {
            ae->num_USP_pkts_qed++;
        }
    }
    else
    {

        for(i = 1; i <= max_sp_length; i++)
        {
            if(i == max_sp_length)
                eosp = BTRUE;

            if(PKT_REQUEUED != requeue_ps_packet(ae, &ae->ps_q_del_ac, BFALSE,
                                                 eosp))
                break;
            ae->num_USP_pkts_qed++;
        }

    }

#ifdef PS_DSCR_JIT
    /* If a packet not requeued due to lack of H/w buffer then stop USP */
    /* and send Qos Null frame */
    if(re_q_status == RE_Q_ERROR)
    {
        /*Send a Qos NULL frame with eosp bit set to 1 and more data set to 1*/
        more_data = 1;
        tx_null_frame(sa, ae, BTRUE, priority, more_data);
        return;
    }
#endif /* PS_DSCR_JIT */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_handle_pwr_mgt                                     */
/*                                                                           */
/*  Description   : This function handles the power management for uapsd     */
/*                  power save frames.                                       */
/*                                                                           */
/*  Inputs        :                                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : If the data is received from a trigger enabled AC of the */
/*                  Source STA then the packets queued for the delivery      */
/*                  enabled AC s of the STA are requeued from the power save */
/*                  buffer to the h/w queue.The number of packets requeued   */
/*                  depends on the maximum Service Period length of the STA  */
/*                  After requeuing the packets,it resets the USP_in_progress*/
/*                  flag and sends a QOS null frame with EOSP bit set to 1   */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void uapsd_handle_pwr_mgt(asoc_entry_t *ae, UWORD8 priority, UWORD8 *msa)
{
    /* If a USP already in progress , ignore the current trigger frame */
    if(ae->USP_in_progress == BTRUE)
    {
        return;
    }
	
	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Non-QoS frame cannot be a trigger frame */
    if(is_qos_bit_set(msa) == BFALSE)
    {
        return;
    }

    if(check_ac_is_trigger_enabled(ae,priority)==BTRUE)
    {
        /* Start a USP */
        ae->USP_in_progress = BTRUE;
        /* Requeue packets from delivery enabled queues */
        uapsd_requeue_ps_pkt(ae, priority, msa);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_handle_ps_sta_change_to_active                     */
/*                                                                           */
/*  Description   :This function handles the change of state of a station    */
/*                 from power save to active state by requeing all buffered  */
/*                 packets from delivery queue to the H/w queue.             */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to association entry of STA                   */
/*                  2) Priority                                              */
/*                  3) Pointer to legacy power save Q                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

REQUEUE_STATUS_T uapsd_handle_ps_sta_change_to_active(asoc_entry_t *ae)
{
    REQUEUE_STATUS_T status = RE_Q_ERROR;

	/* Move the packets from PS queue to HW queue */
    while((status = requeue_ps_packet(ae, &(ae->ps_q_del_ac), BFALSE, BFALSE))
           == PKT_REQUEUED);

#ifdef PS_DSCR_JIT
    /* This path is not valid currently                                      */
    /* May be valid if descriptors are not buffered and are created just in  */
    /* time before transmission                                              */
    if(status == PKT_NOT_REQUEUED)
    {
        handle_requeue_pending_packet(ae, &g_ps_pending_q, BFALSE);
    }
#endif /* PS_DSCR_JIT */
    return status;

}

#endif /* MAC_WMM */
#endif /* BSS_ACCESS_POINT_MODE */
