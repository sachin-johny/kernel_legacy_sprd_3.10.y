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
/*  File Name         : psm_sta.c                                            */
/*                                                                           */
/*  Description       : This file contains all BSS STA power management      */
/*                      related functions.                                   */
/*                                                                           */
/*  List of Functions : initialize_psm_globals                               */
/*                      psm_handle_rx_packet_sta                             */
/*                      psm_handle_tx_packet_sta                             */
/*                      handle_tx_complete_sta                               */
/*                      handle_tbtt_sta                                      */
/*                      psm_handle_tim_elm                                   */
/*                      is_tim_dtim_set                                      */
/*                      send_null_frame_to_AP                                */
/*                      sta_doze                                             */
/*                      sta_awake                                            */
/*                      start_activity_timer                                 */
/*                      psm_alarm_fn                                         */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include <linux/mutex.h>
#include "autorate_sta.h"
#include "cglobals_sta.h"
#include "pm_sta.h"
#include "prot_if.h"
#include "qmu_if.h"
#include "core_mode_if.h"
#include "host_if.h"
#include "iconfig.h"
#include "common.h"
//leon liu added for powersave timer header
#include "ps_timer.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

USER_PS_MODE_T g_user_ps_mode      = NO_POWERSAVE;
BOOL_T         g_receive_dtim       = (BOOL_T)BTRUE;
UWORD32        g_activity_timeout   = ACTIVITY_TIME_OUT; /* in ms */
ALARM_HANDLE_T *g_psm_alarm_handle  = 0;
PS_STATE_T     g_ps_state           = STA_ACTIVE;
BOOL_T         g_beacon_frame_wait  = BFALSE;
BOOL_T         g_active_null_wait   = BFALSE;
BOOL_T         g_doze_null_wait     = BFALSE;
BOOL_T         g_more_data_expected = BFALSE;
BOOL_T         g_send_active_null_frame_to_ap = BFALSE;
UWORD32        g_tx_rx_activity_cnt = 0;

// 20130216 caisf add for power management mode policy to take effect
USER_PS_MODE_T g_user_ps_mode_wait_apply            = NO_POWERSAVE;
BOOL_T         g_receive_dtim_wait_apply            = (BOOL_T)BFALSE;
UWORD8         dot11PowerManagementMode_wait_apply  = 0;
BOOL_T         g_powermanagement_wait_apply_flag    = (BOOL_T)BFALSE;
void show_tx_slots(void);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
unsigned int host2arm_irq_cnt = 0;
UWORD8 calc_data_rate(UWORD8 tr);
/*zhou huiquan add for protect while read/write trout register  */
//#define TROUT_AWAKE     1
//#define TROUT_DOZE      0
//#define TROUT_SLEEP	    2
#define LOCK_TURE       1
#define LOCK_FALSE      0
extern unsigned char g_trout_state;
extern struct mutex rw_reg_mutex;
#endif

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static const UWORD8  g_bmap[8] = {1, 2, 4, 8, 16, 32, 64, 128}; /* Bit map */

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static UWORD8 is_tim_dtim_set(UWORD8* tim_elm);


/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_psm_globals                                   */
/*                                                                           */
/*  Description   : This function initializes all global variables for power */
/*                  save in STA mode                                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_receive_dtim                                           */
/*                  g_beacon_frame_wait                                      */
/*                  g_active_null_wait                                       */
/*                  g_doze_null_wait                                         */
/*                  g_more_data_expected                                     */
/*                  g_activity_timeout                                       */
/*                                                                           */
/*  Processing    : This function initializes all power save related global  */
/*                  variables to the required values.                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_psm_globals(void)
{
    set_ps_state(STA_ACTIVE);

    g_beacon_frame_wait    = BFALSE;
    g_active_null_wait     = BFALSE;
    g_doze_null_wait       = BFALSE;
    g_more_data_expected   = BFALSE;
    g_activity_timeout     = ACTIVITY_TIME_OUT;
    g_tx_rx_activity_cnt   = 0;

    init_psm_prot_globals();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : psm_handle_rx_packet_sta                                 */
/*                                                                           */
/*  Description   : This function is called when a packet is received        */
/*                  in station mode for power save related processing.       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received packet                        */
/*                  2) Priority of the packet                                */
/*                                                                           */
/*  Globals       : g_more_data_expected                                     */
/*                                                                           */
/*  Processing    : This function does the following processing.             */
/*                  If the STA is not in Infrastructure mode, or not yet     */
/*                  associated nothing is done.                              */
/*                  For an infrastructure STA associated,                    */
/*                    i. ACTIVE State                                        */
/*                  The activity timer is restarted for legacy AC            */
/*                   ii. DOZE State                                          */
/*                  Not possible                                             */
/*                  iii. AWAKE State                                         */
/*                  If the packet is a broadcast packet and More Data bit is */
/*                  set then set the global flag g_more_data_expected to     */
/*                  TRUE. If not set then set the flag to FALSE. Note that   */
/*                  flag should have been set on DTIM reception but in case  */
/*                  the DTIM is missed the updating is done here.            */
/*                  In addition other protocol dependent processing are done */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void psm_handle_rx_packet_sta(UWORD8* msa, UWORD8 priority)
{
	TROUT_FUNC_ENTER;

    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();
    /* Handling is required only for Infrastructure associated STA */
    if((mget_DesiredBSSType() != INFRASTRUCTURE) ||
       (mget_PowerManagementMode() != MIB_POWERSAVE) ||
       (g_asoc_id == 0))
    {
        if((get_ps_state() != STA_ACTIVE))
        {
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
            set_ps_state(STA_ACTIVE);
        }
	 /* fix Bug 233922 */
        //incr_activity_cnt();
        TROUT_FUNC_EXIT;
        return;
    }

    /* Start the activity timer if the STA is in ACTIVE state */
    if(get_ps_state() == STA_ACTIVE)
    {
        if(is_legacy_ac(priority) == BTRUE)
        {
            start_activity_timer();
        }
    }
    else if(get_ps_state() == STA_AWAKE)
    {
        UWORD8 da[6] = {0};

        /* Extract the destination address from the frame */
        get_DA(msa, da);

        /* Handle Multicast/Broadcast frames and switch to DOZE state based  */
        /* on the More Data bit.                                             */
        if(is_group(da) == BTRUE)
        {
            if(get_more_data(msa) == 0)
            {
                g_more_data_expected = BFALSE;
                sta_doze();
            }
            else
            {
                g_more_data_expected = BTRUE;
            }
        }
        else
        {
            if(get_more_data(msa) == 0)
                g_more_data_expected = BFALSE;
            else
            {
                TROUT_DBG6("PS: GET more data!");

			// 20120709 caisf add, merged ittiam mac v1.2 code
                if(BTRUE == is_any_legacy_ac_present())
                {
					send_ps_poll_to_AP();
				}
                g_more_data_expected = BTRUE;
			}

            psm_handle_rx_packet_sta_prot(msa, priority);
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : psm_handle_tx_packet_sta                                 */
/*                                                                           */
/*  Description   : This function is called when a packet is ready for       */
/*                  transmission in station mode to check the power save     */
/*                  status and determine the power management bit to be set  */
/*                  in the frame.                                            */
/*                                                                           */
/*  Inputs        : 1) Priority of the packet                                */
/*                                                                           */
/*  Globals       : g_active_null_wait                                       */
/*                                                                           */
/*  Processing    : The current BSS PS algorithm causes the STA to switch to */
/*                  ACTIVE mode whenever there is a packet for transmission  */
/*                  since it expects further transmissions/receptions. This  */
/*                  is based on the Fast Power Save algorithm.               */
/*                  This function does the following processing.             */
/*                  If the STA is not in Infrastructure mode, or not yet     */
/*                  associated or not in Power Save mode nothing is done.    */
/*                  The power management bit is set to 0 (active).           */
/*                  For an infrastructure STA associated and in power save,  */
/*                  The power management bit is set to 1 (power save)        */
/*                    i. ACTIVE State                                        */
/*                  The activity timer is restarted for legacy AC            */
/*                   ii. DOZE State                                          */
/*                  Switch to AWAKE state and follow the steps in (iii)      */
/*                  iii. AWAKE State                                         */
/*                  Send a NULL frame to the AP to indicate the STA is       */
/*                  switching to ACTIVE state (if not done already - check   */
/*                  a flag for it)                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Power management bit to be set in the frame      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 psm_handle_tx_packet_sta(UWORD8 priority)
{
    UWORD8 pwr_mgmt_bit = 0;
    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();

    /* Handling is required only for Infrastructure associated STA in power  */
    /* save mode.                                                            */
    if((mget_DesiredBSSType() != INFRASTRUCTURE) ||
       (mget_PowerManagementMode() != MIB_POWERSAVE) ||
       (g_asoc_id == 0))
    {
        if((get_ps_state() != STA_ACTIVE))
        {
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
           set_ps_state(STA_ACTIVE);
        }

        /* Increment activity counter */
        incr_activity_cnt();
        return pwr_mgmt_bit;
    }

    if(get_ps_state() == STA_ACTIVE)
    {
        /* Start the activity timer for legacy AC */
        if(is_legacy_ac(priority) == BTRUE)
        {
            start_activity_timer();
        }
    }
    else
    {
        /* The power management bit is set whenever the STA is in power save */
        /* mode. In case a NULL frame is explicilty set to change the mode   */
        /* the power management bit will also be updated accordingly.        */
        pwr_mgmt_bit = 1;

        /* In case the STA is in Doze state switch to AWAKE state */
        if(get_ps_state() == STA_DOZE)
        {
#ifdef DEBUG_MODE
            /* Increment the counter to indicate STA AWAKE due to packet from*/
            /* the host                                                      */
            g_mac_stats.num_wakeup_on_hostrx++;
#endif /* DEBUG_MODE */
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
        }

        /* Check if the STA needs to switch to ACTIVE state. If so send a    */
        /* NULL frame to the AP indicating the same (after checking a global */
        /* flag to ensure this has not already been done)                    */
        if(psm_handle_tx_packet_sta_prot(priority) == STA_ACTIVE)
        {
            /* In case of FAST power save mode set the flag to indicate that */
            /* a ACTIVE NULL must be sent after TX-COMP of this current frame*/
            /* This is useful only in case of FAST-PS                        */
            if(is_fast_ps_enabled() == BTRUE)
                   g_send_active_null_frame_to_ap = BTRUE;
        }
    }

    return pwr_mgmt_bit;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_tx_complete_sta                                   */
/*                                                                           */
/*  Description   : This function handles the transmit complete interrupt in */
/*                  STA mode.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                                                                           */
/*  Globals       : g_active_null_wait                                       */
/*                  g_doze_null_wait                                         */
/*                                                                           */
/*  Processing    : This function does the following processing.             */
/*                  If the STA is not in Infrastructure mode, or not yet     */
/*                  associated do nothing.                                   */
/*                  If the STA is not in power save mode, restart the        */
/*                  activity timer                                           */
/*                  For an infrastructure STA associated and in power save,  */
/*                    i. ACTIVE State                                        */
/*                  Do nothing                                               */
/*                   ii. DOZE State                                          */
/*                  Not possible                                             */
/*                  iii. AWAKE State                                         */
/*                  If the packet transmitted was a NULL frame with power    */
/*                  management bit 0 (active) and the status is not TIMEOUT  */
/*                  then switch to ACTIVE and reset the global flag          */
/*                  g_active_null_wait. For TIMEOUT status retry the NULL    */
/*                  frame. If the packet transmitted was a NULL frame with   */
/*                  power management bit 1 (doze) and the status is not      */
/*                  TIMEOUT then switch to DOZE and reset the global flag    */
/*                  g_doze_null_wait. For TIMEOUT status retry the frame.    */
/*                  Call a function for protocol based processing.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_tx_complete_sta(UWORD32 *dscr)
{
    UWORD8      *msa = 0;
    sta_entry_t *se  = 0;

	TROUT_FUNC_ENTER;
    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();
    /* Handling is required only for Infrastructure associated STA in power  */
    /* save mode.                                                            */
    if((mget_DesiredBSSType() != INFRASTRUCTURE) ||
       (mget_PowerManagementMode() != MIB_POWERSAVE) ||
       (g_asoc_id == 0))
    {
        if((get_ps_state() != STA_ACTIVE))
        {
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
           set_ps_state(STA_ACTIVE);
        }
        TROUT_FUNC_EXIT;
        return;
    }

    /* Start power save only after complete authentication */
    se = find_entry(mget_bssid());

    if(BFALSE == is_sec_hs_complete(se))
    {
		TROUT_FUNC_EXIT;
        return;
	}
    /* Get the pointer to the MAC header */
    msa = (UWORD8*)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);

    if(mget_PowerManagementMode() == MIB_POWERSAVE)
    {
        UWORD8 tx_success = 0;

        /* Derive the transmission status from the transmit descriptor */
        if((get_tx_dscr_status(dscr) != TX_TIMEOUT) &&
           (get_tx_dscr_frag_status_summary(dscr) == 0xFFFFFFFF))
        {
            tx_success = 1;
        }

        /* Do legacy processing if the frame was a NULL frame */
        if(get_sub_type(msa) == NULL_FRAME)
        {
            /* If NULL frame was transmitted check if the globals waiting on */
            /* this are set and then process further.                        */
            if((g_active_null_wait == BTRUE) || (g_doze_null_wait == BTRUE))
            {
                UWORD8 pwr_mgt = get_pwr_mgt(msa);

                if(tx_success == 1)
                {
                    /* Change to AWAKE state if not AWAKE. This is an        */
                    /* exception case and should not occur.                  */
                    if(get_ps_state() != STA_AWAKE)
                    {
                        #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
                        sta_awake(LOCK_TURE);
                        #else
						sta_awake();
                        #endif
                    }

                    /* If  transmission status is success reset the global   */
                    /* flags and change to the required mode by checking the */
                    /* power management bit in the frame.                    */
                    if(pwr_mgt == STA_ACTIVE)
                    {
                        if(g_active_null_wait == BTRUE)
                        {
                            g_active_null_wait = BFALSE;
                            set_ps_state(STA_ACTIVE);

                            /* No need to track this flag in ACTIVE state */
                            g_more_data_expected = BFALSE;

                            /* Start Activity Timer on transition from AWAKE */
                            /* to ACTIVE state                               */
                            start_activity_timer();

                            /* In Active state all frames should go with Pwr */
                            /* bit set to zero                               */
                            reset_machw_ps_pm_tx_bit();

#ifdef DEBUG_MODE
                            /* Increment counter to count no of Awake to     */
                            /* Active transitions                            */
                            g_mac_stats.num_wake2active_trn++;
#endif /* DEBUG_MODE */

                        }
                    }
                    else
                    {
                        if(g_doze_null_wait == BTRUE)
                        {
                            g_doze_null_wait = BFALSE;
                            sta_doze();

#ifdef DEBUG_MODE
                            /* Increment counter to count no of Active to    */
                            /* Awake transitions                             */
                            g_mac_stats.num_active2wake_trn++;
#endif /* DEBUG_MODE */
                        }
                    }
                }
                else
                {
                    /* If  transmission status is timeout reset the global   */
                    /* flags and retransmit the NULL frame.                  */
                    if(pwr_mgt == STA_ACTIVE)
                    {
                        g_active_null_wait = BFALSE;
                    }
                    else
                    {
                        g_doze_null_wait = BFALSE;
                    }

                    send_null_frame_to_AP(pwr_mgt, BFALSE, 0);
                    TROUT_FUNC_EXIT;
                    return;
                }
            }
        }
        /* For non-NULL frame check if a NULL ACTIVE frame has to be sent to */
        /* the AP and then process the frame based on protocol.              */
        else
        {
            if((g_send_active_null_frame_to_ap == BTRUE) &&
               (g_active_null_wait == BFALSE))
            {
                g_send_active_null_frame_to_ap = BFALSE;
                send_null_frame_to_AP(STA_ACTIVE, BFALSE, 0);
            }

            handle_tx_complete_sta_prot(msa, tx_success);
        }
    }
    else
    {
        /* If the STA is not in Power save mode start  the activity timer    */
        /* for every non-NULL frame transmission to indicate its state to    */
        /* the AP in case of no traffic.                                     */
        if(get_sub_type(msa) != NULL_FRAME)
        {
            start_activity_timer();
        }
    }

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_tbtt_sta                                          */
/*                                                                           */
/*  Description   : This function handles TBTT event in STA mode.            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_beacon_frame_wait                                      */
/*                  g_more_data_expected                                     */
/*                                                                           */
/*  Processing    : This function does the following processing.             */
/*                  If the STA is not in Infrastructure mode, or not yet     */
/*                  associated or not in Power Save mode nothing is done.    */
/*                  For an infrastructure STA associated and in power save,  */
/*                    i. ACTIVE State                                        */
/*                  Do nothing                                               */
/*                   ii. DOZE/AWAKE State                                    */
/*                  Set the flag g_beacon_frame_wait to TRUE                 */
/*                  Set the flag g_more_data_expected to FALSE               */
/*                  iii. DOZE State                                          */
/*                  Switch to AWAKE state                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_tbtt_sta(void)
{
    /* Handling is required only for Infrastructure associated STA in power  */
    /* save mode.                                                            */

    TROUT_FUNC_ENTER;

    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();
    if((mget_DesiredBSSType() != INFRASTRUCTURE) ||
       (mget_PowerManagementMode() != MIB_POWERSAVE) ||
       (g_asoc_id == 0))
    {
        if((get_ps_state() != STA_ACTIVE))
        {
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
           set_ps_state(STA_ACTIVE);
        }
        TROUT_FUNC_EXIT;
        return;
    }

    /* For non ACTIVE state update the global flags. In case of DOZE state   */
    /* switch to AWAKE state in order to be able to receive TIM/DTIM beacons */
    if(get_ps_state() != STA_ACTIVE)
    {
        g_beacon_frame_wait  = BTRUE;

        if(get_ps_state() == STA_DOZE)
        {
            g_more_data_expected = BFALSE;
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif

#ifdef DEBUG_MODE
            /* No of wakeups due to TBBT interrupt */
            g_mac_stats.num_wakeup_on_tbtt++;
#endif /* DEBUG_MODE */
        }
        handle_tbtt_prot_sta();
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : psm_handle_tim_elm                                       */
/*                                                                           */
/*  Description   : This functions handles the TIM element for BSS STA in    */
/*                  power save mode.                                         */
/*                                                                           */
/*  Inputs        : 1) TIM element extracted from beacon received from AP    */
/*                                                                           */
/*  Globals       : g_beacon_frame_wait                                      */
/*                  g_more_data_expected                                     */
/*                                                                           */
/*  Processing    : This function does the following processing.             */
/*                  Reset the global flag g_beacon_frame_wait                */
/*                  Check if TIM/DTIM is set in the TIM element.             */
/*                    i. ACTIVE State                                        */
/*                  If no TIM/DTIM set global flag g_more_data_expected to   */
/*                  FALSE. If DTIM is set, set the global flag               */
/*                  g_more_data_expected to TRUE.                            */
/*                   ii. DOZE State                                          */
/*                  Not possible                                             */
/*                  iii. AWAKE State                                         */
/*                  If no TIM/DTIM set switch to DOZE. If TIM is set and     */
/*                  there is at least one legacy AC, send a NULL frame to    */
/*                  AP to indicate it is switching to ACTIVE mode (after     */
/*                  checking global flag to confirm this has not been done   */
/*                  already) and set the global flag g_more_data_expected    */
/*                  to TRUE. In case there are no  legacy ACs present call a */
/*                  function to do protocol dependent processing.            */
/*                  If DTIM is set, set the global flag g_more_data_expected */
/*                  to TRUE                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void psm_handle_tim_elm(UWORD8* tim_elm)
{
    UWORD8 result = 0;

    /* Reset the beacon frame wait flag */
    g_beacon_frame_wait = BFALSE;

    /* Check if TIM/DTIM is set */
    result = is_tim_dtim_set(tim_elm);

    /* In AWAKE state switch to DOZE mode if there are no packets buffered   */
    /* at the AP, to ACTIVE mode if there are unicast packets buffered and   */
    /* set the flag to wait for buffered broadcast/multicast frames if any.  */
    if(get_ps_state() == STA_AWAKE)
    {
        if(result == 0)
        {
            g_more_data_expected = BFALSE;

            if((g_active_null_wait == BFALSE) && (g_doze_null_wait == BFALSE))
            {
                /* No unicast / multicast packets stored by AP. Go back to   */
                /* doze mode. Note that for AP this STA is already in power  */
                /* save mode so no need to inform AP.                        */
                sta_doze();
            }
        }
        else if((result & TIM_IS_SET) == BTRUE)
        {
            if(is_any_legacy_ac_present() == BTRUE)
            {
                g_more_data_expected = BTRUE;

                /* Changes state to STA_ACTIVE and send ACTIVE NULL frame to */
                /* AP immeditately in case of FAST PS mode                   */
                init_ps_state_change(STA_ACTIVE);
            }
            else
            {
                psm_handle_tim_set_sta_prot();
            }

#ifdef DEBUG_MODE
            /* STA remains in AWAKE state due to TIM bit set */
            g_mac_stats.num_wakeup_on_tim++;
#endif /* DEBUG_MODE */
        }
        else
        {
            /* DTIM set, stay in AWAKE mode to recieve all broadcast frames */
            g_more_data_expected = BTRUE;

#ifdef DEBUG_MODE
            /* STA remains in AWAKE state due to DTIM count set to zero, to  */
            /* receive broadcast/multicast frames                            */
            g_mac_stats.num_wakeup_on_dtim++;
#endif /* DEBUG_MODE */
        }
    }
    else
    {
        /* Some APs (For Eg. Buffalo AP) sends TIM element set even after  */
        /* our NULL FRAME(ACTIVE) reception. If the TIM element says more  */
        /* data is on the way, then set the appropriate flag else reset it */
        if(result != 0)
        {
            g_more_data_expected = BTRUE;
        }
        else
        {
            g_more_data_expected = BFALSE;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_tim_dtim_set                                          */
/*                                                                           */
/*  Description   : This function parses the TIM element, and returns the    */
/*                  status indicating whether TIM or DTIM is set in it.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the TIM element of the beacon received     */
/*                     from AP.                                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks whether there are packets stored    */
/*                  for this STA by AP. It first computes the byte and       */
/*                  bit offset in partial virtual bitmap corresponding       */
/*                  to its association ID. In the tim_element, check         */
/*                  whether the bit in the computed offset is set or not.    */
/*                  If set, return TIM_IS_SET. If bit corresponding to       */
/*                  DTIM position is also set add DTIM_IS_SET to this.       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : TIM_IS_SET: If TIM element indicates packet buffered     */
/*                              by AP.                                       */
/*                  DTIM_IS_SET: If multicast packets are buffered by AP.    */
/*                  TIM_IS_SET|DTIM_IS_SET: If both are true.                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 is_tim_dtim_set(UWORD8* tim_elm)
{
    UWORD8 len         = 0;
    UWORD8 bmap_ctrl   = 0;
    UWORD8 bmap_offset = 0;
    UWORD8 byte_offset = 0;
    UWORD8 bit_offset  = 0;
    UWORD8 status      = 0;

    /* If the input to this function is not a pointer to the TIM element,    */
    /* '0' is returned indicating the absence of the TIM element.            */
    if(tim_elm[0] != ITIM)
        return status;

    /* Extract the TIM element fields. The length, DTIM Count, DTIM period   */
    /* and bit map control fields are all '1' byte in length.                */
    len           = tim_elm[1];
    bmap_ctrl     = tim_elm[4];

    /* Check if AP's DTIM period has changed */
    if(tim_elm[3] != mget_DTIMPeriod())
    {
        mset_DTIMPeriod(tim_elm[3]);
        set_machw_dtim_period(tim_elm[3]);
    }


    /* Compute the bit map offset, which is given by the MSB 7 bits in the   */
    /* bit map control field of the TIM element.                             */
    bmap_offset = (bmap_ctrl & 0xFE);

    /* A DTIM count of zero indicates that the current TIM is a DTIM. The    */
    /* BIT0 of the bit map control field is set (for TIMs with a value of 0  */
    /* in the DTIM count field) when one or more broadcast/multicast frames  */
    /* are buffered at the AP.                                               */
    if((bmap_ctrl & BIT0) == 1)
        status |= DTIM_IS_SET;

    /* Traffic Indication Virtual Bit Map within the AP, generates the TIM   */
    /* such that if a station has buffered packets, then the corresponding   */
    /* bit (which can be found from the association ID) is set. The byte     */
    /* offset is obtained by dividing the association ID by '8' and the bit  */
    /* offset is the remainder of the association ID when divided by '8'.    */
    byte_offset = g_asoc_id >> 3;
    bit_offset  = g_asoc_id & 0x07;

    /* Bit map offset should always be greater than the computed byte offset */
    /* and the byte offset should always be lesser than the 'maximum' number */
    /* of bytes in the Virtual Bitmap. If either of the above two conditions */
    /* are not satisfied, then the 'status' is returned as is.               */
    if(byte_offset < bmap_offset || byte_offset > bmap_offset + len - 4)
        return status;

    /* The station has buffered packets only if the corresponding bit is set */
    /* in the Virtual Bit Map field. Note: Virtual Bit Map field starts      */
    /* 5 bytes from the start of the TIM element.                            */
    if((tim_elm[5 + byte_offset - bmap_offset] & g_bmap[bit_offset]) != 0)
        status |= TIM_IS_SET;

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_ps_state_change                                     */
/*                                                                           */
/*  Description   : This function is initiates the PS state change depending */
/*                  on the PS type                                           */
/*                                                                           */
/*  Inputs        : New PS state                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This fucntion is called to handle PS state change for    */
/*                  FAST and PS-POLL based power save.                       */
/*                  Following are the different instances this function is   */
/*                  called and its handing in each of PS modes               */
/*                  FAST PS mode:                                            */
/*                   If TIM element in the beacon is set then transition to  */
/*                   STA_ACTIVE state is done by sending ACTIVE NULL to AP.  */
/*                   On maturity of ACTIVITY timer, if STA has to make       */
/*                   transition to STA_DOZE state then DOZE NULL frame is    */
/*                   sent to AP.                                             */
/*                  PS-POLL PS mode:                                         */
/*                   In PS POLL mode the STA remains either in STA-AWAKE or  */
/*                   STA_DOZE PS states.                                     */
/*                   When transition is to be made to DOZE from ACTIVE state */
/*                   DOZE NULL frame is sent to the AP.                      */
/*                   When transtion is to be done to AWAKE state PS-POLL     */
/*                   frame is sent to the AP if there are frames buffered for*/
/*                   the STA.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_ps_state_change(UWORD8 psm)
{
    // 20130216 caisf add for power management mode policy to take effect
    apply_PowerManagementMode_policy();
    /* Handling is required only for Infrastructure associated STA in power  */
    /* save mode.                                                            */
    if((mget_DesiredBSSType() != INFRASTRUCTURE) ||
       (mget_PowerManagementMode() != MIB_POWERSAVE) ||
       (g_asoc_id == 0))
    {
        if((get_ps_state() != STA_ACTIVE))
        {
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            sta_awake(LOCK_TURE);
            #else
            sta_awake();
            #endif
           set_ps_state(STA_ACTIVE);
        }
       return;
    }

    /* Handle PSPoll Power save method */
    if(is_fast_ps_enabled() == BFALSE)
    {
        if(psm == STA_DOZE)
        {
            /* First time send a null frame even in PS Poll method */
            if(get_ps_state() == STA_ACTIVE)
            {
                set_ps_state(STA_AWAKE);
                send_null_frame_to_AP(STA_DOZE, BFALSE,0);
            }
            /* Else directly go to power save */
            else
            {
                sta_doze();
            }
        }
        /* In PS POLL method, STA never goes to STA_ACTIVE state. Hence set */
        /* STA power save state to STA_AWAKE and send a PS_POLL if there are*/
        /* any packets pending for the STA.                                 */
        else /* STA_ACTIVE or STA_AWAKE */
        {
            set_ps_state(STA_AWAKE);
            if(g_more_data_expected == BTRUE)
            {
                send_ps_poll_to_AP();
            }
        }
    }
    /* Handle FAST power save mode */
    else
    {
        set_ps_state(STA_AWAKE);
        /* Do nothing */
        if(psm == STA_AWAKE)
        {
            return;
        }

        if((psm == STA_ACTIVE) && (g_active_null_wait == BFALSE))
        {
                send_null_frame_to_AP(STA_ACTIVE, BFALSE, 0);
        }
        else if((psm == STA_DOZE) && (g_doze_null_wait == BFALSE))
        {
            send_null_frame_to_AP(STA_DOZE, BFALSE,0);
        }
    }
}

void prepare_ps_poll_for_cp(void)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      mac_hdr_len  = MAC_HDR_LEN;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};
	unsigned char *pc;
	unsigned int ix = 0;
	unsigned char tmp[200];

	

	memset(tmp, 0, 200);

	TROUT_FUNC_ENTER;
    /* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());

    if(NULL == se)
    {
        /* This is an exception case and should never occur */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
	TROUT_FUNC_EXIT;
        return;
    }

    /* Set the Frame Control field of the PS-POLL frame.                     */
    set_frame_control(msa, (UWORD16)PS_POLL);
    q_num = HIGH_PRI_Q;

    set_pwr_mgt(msa, (UWORD8)STA_ACTIVE);
	//chenq mask 2012-12-06
    //set_to_ds(msa, 1);

    /* Set Association ID in the Duration ID field */
    set_durationID(msa, g_asoc_id);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());

	//set_address3(msa, mget_bssid());

    /* Create the transmit descriptor and set the contents                   */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        /* Free the memory allocated for the buffer                          */
        pkt_mem_free(msa);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set the transmit rate from the station entry */
    tx_rate = get_tx_rate_to_sta(se);
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, se, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, msa, 0, mac_hdr_len - 8, 0);//chenq add 2012-12-06
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);


	memcpy((void *)tmp, (void *)tx_dscr, 152);
	memcpy((void *)&tmp[152], (void *)msa, 16);
	host_write_trout_ram((void *)(BEACON_MEM_BEGIN + 200), (void *)tmp, 172);
#if 0
	printk("XXXYYYYZZZ:%d\n");
	pc = msa;
	for(ix = 0; ix < 20; ix++){
		if (ix % 16 == 0 )
			printk("\n");
		printk("%02X, ", *pc++);
	}
	printk("\n");
	pc = tx_dscr;
	for(ix = 0; ix < 152; ix++){
		if (ix % 16 == 0 )
			printk("\n");
		printk("%02X, ", *pc++);
	}
	printk("\n");
#endif

    

    free_tx_dscr((UWORD32 *)tx_dscr);
  
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_ps_poll_to_AP                                       */
/*                                                                           */
/*  Description   : This function prepares and sends a PS-Poll frame to AP   */
/*                                                                           */
/*  Inputs        : 1) Desired power save mode for the null frame            */
/*                  3) Priority of the null frame                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the PSPoll frame with the requrd  */
/*                  power save, QoS and priority as specified by the user.   */
/*                  The prepared frame is added to the appropriate MAC H/w   */
/*                  transmit queue for transmission.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_ps_poll_to_AP(void)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      mac_hdr_len  = MAC_HDR_LEN;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};

	TROUT_FUNC_ENTER;
    /* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());

    if(NULL == se)
    {
        /* This is an exception case and should never occur */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
	TROUT_FUNC_EXIT;
        return;
    }

    /* Set the Frame Control field of the PS-POLL frame.                     */
    set_frame_control(msa, (UWORD16)PS_POLL);
    q_num = HIGH_PRI_Q;

    set_pwr_mgt(msa, (UWORD8)STA_DOZE);
	//chenq mask 2012-12-06
    //set_to_ds(msa, 1);

    /* Set Association ID in the Duration ID field */
    set_durationID(msa, g_asoc_id);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());

	//set_address3(msa, mget_bssid());

    /* Create the transmit descriptor and set the contents                   */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        /* Free the memory allocated for the buffer                          */
        pkt_mem_free(msa);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set the transmit rate from the station entry */
    tx_rate = get_tx_rate_to_sta(se);
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, se, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, msa, 0, mac_hdr_len - 8, 0);//chenq add 2012-12-06
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Do the protocol specific processing */
    send_ps_poll_to_AP_prot();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_null_frame_to_AP                                    */
/*                                                                           */
/*  Description   : This function prepares and sends a NULL frame to AP      */
/*                                                                           */
/*  Inputs        : 1) Desired power save mode for the null frame            */
/*                  2) QoS requirement                                       */
/*                  3) Priority of the null frame                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the NULL frame with the required  */
/*                  power save, QoS and priority as specified by the user.   */
/*                  The prepared frame is added to the appropriate MAC H/w   */
/*                  transmit queue for transmission.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_null_frame_to_AP(UWORD8 psm, BOOL_T is_qos, UWORD8 priority)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};
    UWORD8      frame_len = MAC_HDR_LEN;
    UWORD8      serv_class = NORMAL_ACK;

	TROUT_FUNC_ENTER;
	
    printk("%s... clled from %pS\n", __func__, __builtin_return_address(0));
    /* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());
    if(NULL == se)
    {
        TROUT_DBG3("SE NULL, Cant Send NULL Frame\n\r");
        /* This is an exception case and should never occur */
        TROUT_FUNC_EXIT;
        return;
    }

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* For Qos NULL frame check if all protocol allow this transmission */
    if((is_qos == BTRUE) && (psm_allow_null_send_prot(se, priority) == BFALSE))
    {
		TROUT_DBG4("protocol not allow this transmission, Cant Send NULL Frame\n\r");
		TROUT_FUNC_EXIT;
        return;
    }

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        TROUT_DBG4("No Memory for NULL frame\n");
		TROUT_FUNC_EXIT;
        return;
    }

    /* Set the Frame Control field of the NULL frame. */
#ifdef MAC_WMM
    if(is_qos == BTRUE)
    {
        set_frame_control(msa, (UWORD16)QOS_NULL_FRAME);
        q_num       = get_txq_num(priority);
    }
    else
#endif /* MAC_WMM */
    {
        set_frame_control(msa, (UWORD16)NULL_FRAME);
        q_num       = HIGH_PRI_Q;
    }

    set_pwr_mgt(msa, (UWORD8)psm);
    set_to_ds(msa, 1);

    if(psm == 0)
        reset_machw_ps_pm_tx_bit();
    else
        set_machw_ps_pm_tx_bit();

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());

    if(is_qos == BTRUE)
    {
		// 20120709 caisf mod, merged ittiam mac v1.2 code
        //set_qos_control(msa, priority, NORMAL_ACK);
		set_qos_control(msa, priority, serv_class);
        frame_len += 2;
    }

    /* Create the transmit descriptor and set the contents */
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    //tx_dscr = create_default_tx_dscr(0, 0, 0);
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
    if(tx_dscr == NULL)
    {
		TROUT_DBG3("No Mem for NULL TX DSCR \n");
        /* Free the memory allocated for the buffer */
        pkt_mem_free(msa);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set the transmit rate from the station entry */
    tx_rate = get_tx_rate_to_sta(se);
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, se, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    //set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_params(tx_dscr, tx_rate, pream, serv_class, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* For Qos NULL frame check if any protocol related buffering is possible*/
    if((is_qos == BTRUE) &&
       (is_serv_cls_buff_pkt_sta(se, q_num, priority, tx_dscr) == BTRUE))
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Add the packet to the MAC H/w transmit queue */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception. Free the transmit descriptor. */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
    }
    else
    {
        /* Set the legacy global flags only if this is not a QoS NULL frame */
        if(is_qos == BFALSE)
        {
            if(psm == STA_ACTIVE)
            {
                g_active_null_wait = BTRUE;
                g_doze_null_wait   = BFALSE;
            }
            else
            {
                g_doze_null_wait   = BTRUE;
                g_active_null_wait = BFALSE;
            }
        }
    }

    TROUT_FUNC_EXIT;
}

UWORD8 calc_data_rate(UWORD8 tr)
{
	UWORD8 dr = 0;
	UWORD8 pr = 0;

	dr = get_phy_rate(tr);
	pr = get_preamble(tr);
	if(IS_RATE_11B(dr) == BTRUE)
		dr |= (BIT2 & (pr << 2));

	return dr;
}


void update_retry_dr_trick(UWORD8 *set)
{
	int i;
	UWORD8 *p = set;
	UWORD8 tr[7] = {54, 36, 24, 11, 11, 2, 1};
	UWORD8 *t = (UWORD8 *)&tr[0];

	for(i = 0; i < 7; i++ ){
		*p++ = calc_data_rate(*t++);
	}

}

int prepare_null_frame_for_cp(UWORD8 psm, BOOL_T is_qos, UWORD8 priority)
{
	UWORD8      tx_rate      = 0;
	UWORD8      pream        = 0;
	UWORD8      q_num        = 0;
	UWORD32     phy_tx_mode  = 0;
	UWORD8      *msa         = 0;
	UWORD8      *tx_dscr     = 0;
	sta_entry_t *se          = 0;
	UWORD32     retry_set[2] = {0};
	UWORD8      frame_len = MAC_HDR_LEN;
	UWORD8      serv_class = NORMAL_ACK;
	unsigned char tmp[200];
	unsigned char *pc, ix = 0;	


	memset(tmp, 0, 200);
	/* Get the station entry for the AP */
	se = (sta_entry_t *)find_entry(mget_bssid());
	if(NULL == se)
	{
		TROUT_DBG3("SE NULL, Cant Send NULL Frame\n\r");
		return -1;
	}

	/* For Qos NULL frame check if all protocol allow this transmission */
	if((is_qos == BTRUE) && (psm_allow_null_send_prot(se, priority) == BFALSE))
	{
		TROUT_DBG4("protocol not allow this transmission, Cant Send NULL Frame\n\r");
		return -1;
	}

	/* Allocate buffer for the NULL Data frame. This frame contains only the */
	/* MAC Header. The data payload of the same is '0'.                      */
	msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
	if(msa == NULL)
	{
		TROUT_DBG4("No Memory for NULL frame\n");
		return -1;
	}

	/* Set the Frame Control field of the NULL frame. */
#ifdef MAC_WMM
	if(is_qos == BTRUE)
	{
		set_frame_control(msa, (UWORD16)QOS_NULL_FRAME);
	}
	else
#endif /* MAC_WMM */
	{
		set_frame_control(msa, (UWORD16)NULL_FRAME);
	}    

	q_num = HIGH_PRI_Q;


	set_pwr_mgt(msa, (UWORD8)psm);
	set_to_ds(msa, 1);

	if(psm == 0)
		reset_machw_ps_pm_tx_bit();
	else
		set_machw_ps_pm_tx_bit();

	/* Set the address fields. For a station operating in the infrastructure */
	/* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
	/* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
	set_address1(msa, mget_bssid());
	set_address2(msa, mget_StationID());
	set_address3(msa, mget_bssid());

	if(is_qos == BTRUE)
	{
		//set_qos_control(msa, priority, NORMAL_ACK);
		set_qos_control(msa, priority, serv_class);
		frame_len += 2;
	}

	/* Create the transmit descriptor and set the contents */
	tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
	if(tx_dscr == NULL)
	{
		TROUT_DBG3("No Mem for NULL TX DSCR \n");
		/* Free the memory allocated for the buffer */
		pkt_mem_free(msa);
		return -1;
	}

	/* Set the transmit rate from the station entry */
	//tx_rate = 54;
	tx_rate = get_tx_rate_to_sta(se);
	pream   = get_preamble(tx_rate);

	/* Update the retry set information for this frame */
	update_retry_rate_set(1, tx_rate, se, retry_set);

	/* Get the PHY transmit mode based on the transmit rate and preamble */
	phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

	/* Set various transmit descriptor parameters */
	set_tx_params(tx_dscr, tx_rate, pream, serv_class, phy_tx_mode, retry_set);
#if 0
	/* we modified the retry data rate */
	update_retry_dr_trick((UWORD8 *)&retry_set[0]);
	retry_set[1] = get_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr);
	retry_set[1] &= 0xFF00FFFF;
	retry_set[1] |= (0x04 << 16);	//retry_rate7 set to 1Mbps
	set_tx_dscr_retry_rate_set2(tx_dscr, retry_set[1]);
#endif
	set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);
	set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
	set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
	set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
	set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
	update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

	memcpy((void *)&tmp[0], (void *)tx_dscr, TX_DSCR_LEN + 8);
	memcpy((void *)&tmp[TX_DSCR_LEN + 8], (void *)msa, frame_len);

#if 0	
	printk("frame_len:%d\n", frame_len);
	pc = tmp;
	for(ix = 0; ix < 200; ix++){
		if (ix % 16 == 0 )
			printk("\n");
		printk("%02X, ", *pc++);
	
	}
	printk("\n");
#endif
	host_write_trout_ram((void *)BEACON_MEM_BEGIN, (void *)tmp, TX_DSCR_LEN + 8 + frame_len);
	
        free_tx_dscr((UWORD32 *)tx_dscr);

}

int send_ps_poll_to_AP_trick(void)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      mac_hdr_len  = MAC_HDR_LEN;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};

        TROUT_FUNC_ENTER;

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
        null_frame_dscr = NULL; /*record NULL frame pointer of power save*/
#endif
#endif

    /* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());

    if(NULL == se)
    {
        /* This is an exception case and should never occur */
        return -1;
    }

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        return -1;
    }

    /* Set the Frame Control field of the PS-POLL frame.                     */
    set_frame_control(msa, (UWORD16)PS_POLL);
    q_num = HIGH_PRI_Q;

    set_pwr_mgt(msa, (UWORD8)STA_ACTIVE);
        //chenq mask 2012-12-06
    //set_to_ds(msa, 1);
    /* Set Association ID in the Duration ID field */
    set_durationID(msa, g_asoc_id);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());

        //set_address3(msa, mget_bssid());

    /* Create the transmit descriptor and set the contents                   */
    tx_dscr = create_default_tx_dscr(0, 0, 0);
    if(tx_dscr == NULL)
    {
        /* Free the memory allocated for the buffer                          */
        pkt_mem_free(msa);
        return -1;
    }

    /* Set the transmit rate from the station entry */
    tx_rate = get_tx_rate_to_sta(se);
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, se, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);

    /* we modified the retry data rate */
        update_retry_dr_trick((UWORD8 *)&retry_set[0]);
        retry_set[1] = get_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr);
        retry_set[1] &= 0xFF00FFFF;
        retry_set[1] |= (0x04 << 16);   //retry_rate7 set to 1Mbps
        set_tx_dscr_retry_rate_set2(tx_dscr, retry_set[1]);

    set_tx_buffer_details(tx_dscr, msa, 0, mac_hdr_len - 8, 0);//chenq add 2012-12-06
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
        return -1;
    }
    else
    {
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
            null_frame_dscr = tx_dscr; /*record NULL frame pointer of power save*/
#endif
#endif
            return 0;
    }

    /* Do the protocol specific processing */
    send_ps_poll_to_AP_prot();
    TROUT_FUNC_EXIT;
}


/* use speacial trick to make sure NULL frame send successfully by zhao*/
int send_null_frame_to_AP_trick(UWORD8 psm, BOOL_T is_qos, UWORD8 priority)
{
    UWORD8      tx_rate      = 0;
    UWORD8      pream        = 0;
    UWORD8      q_num        = 0;
    UWORD32     phy_tx_mode  = 0;
    UWORD8      *msa         = 0;
    UWORD8      *tx_dscr     = 0;
    sta_entry_t *se          = 0;
    UWORD32     retry_set[2] = {0};
    UWORD8      frame_len = MAC_HDR_LEN;
    UWORD8      serv_class = NORMAL_ACK;

	TROUT_FUNC_ENTER;

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	null_frame_dscr = NULL; /*record NULL frame pointer of power save*/
#endif
#endif
    /* Get the station entry for the AP */
    se = (sta_entry_t *)find_entry(mget_bssid());
    if(NULL == se)
    {
        TROUT_DBG3("SE NULL, Cant Send NULL Frame\n\r");
        /* This is an exception case and should never occur */
        TROUT_FUNC_EXIT;
        return -1;
    }

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* For Qos NULL frame check if all protocol allow this transmission */
    if((is_qos == BTRUE) && (psm_allow_null_send_prot(se, priority) == BFALSE))
    {
		TROUT_DBG4("protocol not allow this transmission, Cant Send NULL Frame\n\r");
        return -1;
    }

    /* Allocate buffer for the NULL Data frame. This frame contains only the */
    /* MAC Header. The data payload of the same is '0'.                      */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        TROUT_DBG4("No Memory for NULL frame\n");
	TROUT_FUNC_EXIT;
        return -1;
    }
	pr_info("NULL F@: %08X\n", msa);

    /* Set the Frame Control field of the NULL frame. */
#ifdef MAC_WMM
    if(is_qos == BTRUE)
    {
        set_frame_control(msa, (UWORD16)QOS_NULL_FRAME);
		if(get_wmm_enabled() == BTRUE)
	        q_num = AC_BK_Q;
		else
			q_num = NORMAL_PRI_Q;
    }
    else
#endif /* MAC_WMM */
    {
        set_frame_control(msa, (UWORD16)NULL_FRAME);
    	q_num =  NORMAL_PRI_Q; 
    }    

    if(psm == STA_ACTIVE)
    	q_num = HIGH_PRI_Q;

    printk("%s: psm=%d, q_num=%d\n", __func__, psm, q_num);

    set_pwr_mgt(msa, (UWORD8)psm);
    set_to_ds(msa, 1);

    if(psm == 0)
        reset_machw_ps_pm_tx_bit();
    else
        set_machw_ps_pm_tx_bit();

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());

    if(is_qos == BTRUE)
    {
		// 20120709 caisf mod, merged ittiam mac v1.2 code
        //set_qos_control(msa, priority, NORMAL_ACK);
		set_qos_control(msa, priority, serv_class);
        frame_len += 2;
    }

    /* Create the transmit descriptor and set the contents */
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    //tx_dscr = create_default_tx_dscr(0, 0, 0);
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
    if(tx_dscr == NULL)
    {
		TROUT_DBG3("No Mem for NULL TX DSCR \n");
        /* Free the memory allocated for the buffer */
        pkt_mem_free(msa);
        TROUT_FUNC_EXIT;
        return -1;
    }

    /* Set the transmit rate from the station entry */
    //tx_rate = 54;
    tx_rate = get_tx_rate_to_sta(se);
    pream   = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, se, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

    /* Set various transmit descriptor parameters */
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    //set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_params(tx_dscr, tx_rate, pream, serv_class, phy_tx_mode, retry_set);
	/* we modified the retry data rate */
	update_retry_dr_trick((UWORD8 *)&retry_set[0]);
	retry_set[1] = get_tx_dscr_retry_rate_set2((UWORD32 *)tx_dscr);
	retry_set[1] &= 0xFF00FFFF;
	retry_set[1] |= (0x04 << 16);	//retry_rate7 set to 1Mbps
	set_tx_dscr_retry_rate_set2(tx_dscr, retry_set[1]);
	
    set_tx_buffer_details(tx_dscr, msa, 0, frame_len, 0);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index);
    set_ht_ps_params(tx_dscr, (void *)se, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* For Qos NULL frame check if any protocol related buffering is possible*/
    if((is_qos == BTRUE) &&
       (is_serv_cls_buff_pkt_sta(se, q_num, priority, tx_dscr) == BTRUE))
    {
		TROUT_FUNC_EXIT;
        return -1;
    }

    /* Add the packet to the MAC H/w transmit queue */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception. Free the transmit descriptor. */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
		return -1;
    }
    else
    {
        /* Set the legacy global flags only if this is not a QoS NULL frame */
        if(is_qos == BFALSE)
        {
            if(psm == STA_ACTIVE)
            {
                g_active_null_wait = BTRUE;
                g_doze_null_wait   = BFALSE;
            }
            else
            {
                g_doze_null_wait   = BTRUE;
                g_active_null_wait = BFALSE;
            }
        }
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	null_frame_dscr = tx_dscr; /*record NULL frame pointer of power save*/
#endif
#endif
	return 0;
    }
}







#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_doze                                                 */
/*                                                                           */
/*  Description   : This function switches the STA to DOZE state during      */
/*                  Infrastructure power management.                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_beacon_frame_wait                                      */
/*                  g_more_data_expected                                     */
/*                  g_active_null_wait                                       */
/*                  g_doze_null_wait                                         */
/*                                                                           */
/*  Processing    : This function checks all global flags and if the flags   */
/*                  indicate that the STA is not waiting for any frame/event */
/*                  it switches to DOZE mode by switching off MAC H/w, PHY.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_doze(void)
{
    if((g_beacon_frame_wait == BFALSE) && (g_doze_null_wait == BFALSE) &&
       (g_more_data_expected == BFALSE) && (g_active_null_wait == BFALSE) &&
       (g_send_active_null_frame_to_ap == BFALSE) &&
       (is_csw_in_prog() == BTRUE) && (can_sta_doze_prot() == BTRUE))
    {
        /* Send all pending frames to the host */
        while(1)
        {
            UWORD32 hif_q_count = get_hif_cnfg_pkt_tx_q_cnt();

            hif_q_count += get_hif_data_pkt_tx_q_cnt();

            if(hif_q_count != 0)
            {
                send_pending_frames_to_host();
            }
            else
            {
                break;
            }
        }

        /* Change state to doze */
        set_ps_state(STA_DOZE);

        /* Suspend MAC H/w transmission before disabling it */
        set_machw_tx_suspend();

        /* Disable MAC H/w and PHY */
        disable_machw_phy_and_pa();

        /* Power down the PHY */
		/* avoid ARM7 to operate PHY regs, so doze need't down phy  by zhao */
        //power_down_phy();

        /* Indicate STA is dozing to MAC-HW */
        set_hw_ps_mode();

#ifdef DEBUG_MODE
        /* Increment the number of STA sleeps */
        g_mac_stats.num_sta_doze++;
#endif /* DEBUG_MODE */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_awake                                                */
/*                                                                           */
/*  Description   : This function switches the STA to AWAKE state during     */
/*                  Infrastructure power management.                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function swiches on PHY and MAC H/w                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_awake(void)
{
    if(get_ps_state() == STA_DOZE)
    {
        /* Power up the PHY */
		/* if doze does not power down phy, we needn't power it anymore by zhao */
        //power_up_phy();

        /* Enable MAC H/w and PHY */
        enable_machw_phy_and_pa();

        /* Check if STA can transmit now  */
        if((can_sta_transmit_now_prot() == BFALSE) ||
           (is_csw_in_prog() == BFALSE))
        {
            set_ps_state(STA_AWAKE);
            return;
        }

        /* Resume MAC H/w transmission after enabling it */
        set_machw_tx_resume();

		// 20120709 caisf masked, merged ittiam mac v1.2 code
        /* Indicate to MAC-HW STA is AWAKE */
        //set_machw_ps_active_mode();
    }

    set_ps_state(STA_AWAKE);
}

#else


/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_doze                                                 */
/*                                                                           */
/*  Description   : This function switches the STA to DOZE state during      */
/*                  Infrastructure power management.                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_beacon_frame_wait                                      */
/*                  g_more_data_expected                                     */
/*                  g_active_null_wait                                       */
/*                  g_doze_null_wait                                         */
/*                                                                           */
/*  Processing    : This function checks all global flags and if the flags   */
/*                  indicate that the STA is not waiting for any frame/event */
/*                  it switches to DOZE mode by switching off MAC H/w, PHY.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 ps_last_int_mask = 0;

void sta_doze(void)
{
    UWORD32 tmp_mask = 0;
    UWORD32 int_mask = 0;
    UWORD32 int_stat = 0;

	/*TROUT_PRINT_STACK;*/

    if((g_beacon_frame_wait == BFALSE) && (g_doze_null_wait == BFALSE) &&
       (g_more_data_expected == BFALSE) && (g_active_null_wait == BFALSE) &&
       (g_send_active_null_frame_to_ap == BFALSE) &&
       (is_csw_in_prog() == BTRUE) && (can_sta_doze_prot() == BTRUE))
	{
        /* Send all pending frames to the host */
        while(1)
        {
            UWORD32 hif_q_count = get_hif_cnfg_pkt_tx_q_cnt();

            hif_q_count += get_hif_data_pkt_tx_q_cnt();

            if(hif_q_count != 0)
            {
                send_pending_frames_to_host();
            }
            else
            {
                break;
            }
        }

	/*zhou huiquan add protect rw reg*/
	mutex_lock(&rw_reg_mutex);
        /* Change state to doze */
        set_ps_state(STA_DOZE);
	/*zhou huiquan add change protect state*/
	g_trout_state = TROUT_DOZE;

        /* Suspend MAC H/w transmission before disabling it */
        root_set_machw_tx_suspend();

        /* Disable MAC H/w and PHY */
        root_disable_machw_phy_and_pa();

        /* Power down the PHY */
		/* avoid ARM7 to operate PHY regs, so doze need't down phy  by zhao */
        //root_power_down_phy();

        /* Indicate STA is dozing to MAC-HW */
        root_set_hw_ps_mode();

	//Begin:delete by wulei 2791 for bug 155500 on 2013-04-30
	//root_mask_machw_tbtt_int();/*mask TBTT INTR from host side*/
	//End:delete by wulei 2791 for bug 155500 on 2013-04-30

	//  caisf add for unmask relate intterupt 0319
	ps_last_int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
	//tmp_mask = ((ps_last_int_mask & 0x3ffffc) & (~(BIT29 << 2)));
	tmp_mask = (~(BIT29 << 2));

	/*pr_info("sleep ps_last_int_mask = %#x; tmp_mask = %#x\n",ps_last_int_mask,tmp_mask);*/
	root_host_write_trout_reg(tmp_mask , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(tmp_mask);
	root_host_write_trout_reg(0xfffffffc, (UWORD32)rCOMM_INT_CLEAR);
	msleep(10);

	root_notify_cp_with_handshake(PS_MSG_ARM7_HANDLE_TBTT_MAGIC, 3);
	g_enable_machw_clock_flag = 0;

#ifdef POWERSAVE_DEBUG
	pr_info("%s: going to doze!\n", __func__);
#endif

#ifdef DEBUG_MODE
        /* Increment the number of STA sleeps */
        g_mac_stats.num_sta_doze++;
#endif /* DEBUG_MODE */

		/*zhou huiquan add*/
		mutex_unlock(&rw_reg_mutex);
    }
}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
void sta_doze_trick(DOZE_T type,BOOL_T reset_lock)
{
	UWORD32 tmp_mask = 0;
    UWORD32 pa_val = 0;
	struct trout_private *tp = netdev_priv(g_mac_dev);
	rx_queue_struct *hrc = &g_rx_handle->rx_q_info[HIGH_PRI_RXQ];
	rx_queue_struct *nrc = &g_rx_handle->rx_q_info[NORMAL_PRI_RXQ];

    mutex_lock(&tp->sm_mutex);
   /* Disable MAC H/w and PHY, no more INTS */
    root_disable_machw_phy_and_pa();
    pr_info("%s: XXdisable PA\n", __func__);
    
	/* record previous INT mask */
    ps_last_int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
	
    rx_complete_isr(HIGH_PRI_RXQ);
    rx_complete_isr(NORMAL_PRI_RXQ);
    /* it's safe to clear all INTs */
    root_host_write_trout_reg(0xffffffff, (UWORD32)rCOMM_INT_CLEAR);

    /* disable all INTS except bit31 */
    tmp_mask = (~(BIT29 << 2));
    root_host_write_trout_reg(tmp_mask , (UWORD32)rCOMM_INT_MASK);
    update_trout_int_mask(tmp_mask);

	/*ps_last_int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
	pr_info("%s: new clear\n", __func__);
	root_clear_machw_interrupts();*/
	//msleep(10);
	/* run RX-work, so we can make sure ARM7 has buffer to recving beacon */
	while(!list_empty(&hrc->rx_list)){
		schedule_work(&hrc->work);
		msleep(10);
		printk("@@@:HRX-work run\n");
	}
	while(!list_empty(&nrc->rx_list)){
		schedule_work(&nrc->work);
		msleep(10);
		printk("@@@:NRX-work run\n");
	}
	//cancel_work_sync(&(&g_rx_handle->rx_q_info[NORMAL_PRI_RXQ])->work); /*cancel rx works before suspending*/
	//cancel_work_sync(&(&g_rx_handle->rx_q_info[HIGH_PRI_RXQ])->work); /*cancel rx works before suspending*/
	printk("[sta_doze_trick]\t cancel_work_sync(event_work)====>>\n");
	cancel_work_sync(&tp->event_work);
	printk("[sta_doze_trick]\t cancel_work_sync(event_work)====<<\n");
	
#ifdef WAKE_LOW_POWER_POLICY
	if(type == LOW_POWER_DOZE)
	{
		g_wifi_power_mode = WIFI_LOW_POWER_MODE;
		printk("low power doze, set g_wifi_power_mode=%d\n", g_wifi_power_mode);
	}
#endif

	/* remember CP need to know AID */
	root_notify_cp_with_handshake((g_asoc_id << 16) | PS_MSG_ARM7_SBEA_KC_MAGIC, 3);

	//g_wifi_suspend_status = wifi_suspend_early_suspend;
	if(type == SUSPEND_DOZE){
	g_wifi_suspend_status = wifi_suspend_early_suspending;     /*when suspending, maybe enterring mac_isr_work */
	}
	pr_info("%s: switch to ARM7!, asoc_id = %x\n", __func__, g_asoc_id);
    mutex_unlock(&tp->sm_mutex);
    msleep(10);   
	if(type == SUSPEND_DOZE)
    	g_wifi_suspend_status = wifi_suspend_early_suspend;                      							/*delay 10ms , if have trout interrupt, deal mac_isr_work*/
}
#endif
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_awake                                                */
/*                                                                           */
/*  Description   : This function switches the STA to AWAKE state during     */
/*                  Infrastructure power management.                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function swiches on PHY and MAC H/w                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_awake(bool flag)
{
	if(flag){
		mutex_lock(&rw_reg_mutex);
	}
	/*TROUT_PRINT_STACK;*/
	//danny.deng comment here for bug 158430
	//bug 158430:download video data by pptv for long time, wifi can't be turn off.
	//add a condition judgement for doze
    if((get_ps_state() == STA_DOZE) || (g_trout_state == TROUT_DOZE))
    {
		UWORD32 sysval = 0;

		sysval = root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL);
		//if((sysval & 0x70000) != 0x70000) {
#ifdef POWERSAVE_DEBUG
		TROUT_DBG4("%s: wake up wifi, wait 10ms ...\n", __func__);
#endif
//		root_notify_cp_with_handshake(PS_MSG_HOST_WAKE_WIFI_MAGIC, 3); // for debug 0408
		root_notify_cp_with_handshake(PS_MSG_HOST_HANDLE_TBTT_MAGIC, 3);
		g_enable_machw_clock_flag = 1;

		root_reset_machw_tbtt_int();
		//Begin:delete by wulei 2791 for bug 155500 on 2013-4-30
		//root_unmask_machw_tbtt_int();								/*unmask TBTT INTR from host side*/
		//End:delete by wulei 2791 for bug 155500 on 2014-4-30
		//}
        /* Power up the PHY */
        /*power_up_phy();*/ /*by kelvin*/

        /* Enable MAC H/w and PHY */
        /*enable_machw_phy_and_pa();*/ /*by kelvin*/

       /* Test wifi wake up 2013.3.19 ranqing.wu*/
	root_host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(ps_last_int_mask);

        /* Check if STA can transmit now  */
        if((can_sta_transmit_now_prot() == BFALSE) ||
           (is_csw_in_prog() == BFALSE))
        {
		set_ps_state(STA_AWAKE);
		/*zhou huiquan add change protect state*/
		g_trout_state = TROUT_AWAKE;
		if(flag){
			mutex_unlock(&rw_reg_mutex);
		}
		return;
        }


        /* Resume MAC H/w transmission after enabling it */
        root_set_machw_tx_resume();

	// 20120709 caisf masked, merged ittiam mac v1.2 code
        /* Indicate to MAC-HW STA is AWAKE */
        //set_machw_ps_active_mode();
    }

    // goto active
    if(g_wifi_suspend_status == wifi_suspend_nosuspend)
    {
#ifdef POWERSAVE_DEBUG
		pr_info("%s: wake up, man!\n", __func__);
#endif
//		root_notify_cp_with_handshake(PS_MSG_HOST_HANDLE_TBTT_MAGIC, 3); // for debug 0408
//		g_enable_machw_clock_flag = 1;

		root_reset_hw_ps_mode();
    }

    set_ps_state(STA_AWAKE);
    /*zhou huiquan add change protect state*/
    g_trout_state = TROUT_AWAKE;

	if(flag){
		mutex_unlock(&rw_reg_mutex);
	}
}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
void sta_awake_trick(void)
{
	root_notify_cp_with_handshake(PS_MSG_ARM7_EBEA_KC_MAGIC, 10);
	/*root_notify_cp_with_handshake(PS_MSG_HOST_HANDLE_TBTT_MAGIC, 3);*/
        printk("@@@: AWK,KC1-CIMSK:%08X, CIS:%08X, PA:%08X\n", root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK),
                root_host_read_trout_reg((UWORD32)rCOMM_INT_STAT), root_host_read_trout_reg((UWORD32)rMAC_PA_CON)); 
	root_host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(ps_last_int_mask);
	g_wifi_suspend_status = wifi_suspend_nosuspend;
	pr_info("%s: back to Host!\n", __func__);
}
#endif
#endif

EXPORT_SYMBOL(sta_awake);
void sta_sleep(void)
{
	//leon liu added for stopping powersave timer on 2013-04-03
	pstimer_stop(&pstimer);
        /* Suspend MAC H/w transmission before disabling it */
        set_machw_tx_suspend();

	/* Disable TSF */
	/*host_write_trout_reg(0x0, (UWORD32)rMAC_TSF_CON);*/

        /* Disable MAC H/w and PHY */
        disable_machw_phy_and_pa();

        /* Power down the PHY */
        power_down_phy();

	notify_cp_with_handshake(PS_MSG_WIFI_SUSPEND_MAGIC, 3);
	g_enable_machw_clock_flag = 0;
	g_wifi_suspend_status = wifi_suspend_suspend;
}

void sta_wakeup(void)
{
	notify_cp_with_handshake(PS_MSG_WIFI_RESUME_MAGIC, 10);
	g_enable_machw_clock_flag = 1;

	g_wifi_suspend_status = wifi_suspend_nosuspend;
	printk("g_wifi_suspend_status = %d\n",g_wifi_suspend_status);
	/* do not use start_mac_and_phy, and power_up_phy, since ARM7 will never
	 * operate phy regs again by zhao
	 */
	restart_mac_plus(&g_mac, 0);
	power_up_phy(); /*by zhao*/
	//restart_mac(&g_mac, 0);
}

//leon liu added sta_sleep_disconnected 2013-4-3
//This function is used to let STA go into sleep stae when STA
//is not connected(NOTE than in this function, get_ps_state will always be STA_ACTIVE
//since the powersave timer(ps_timer.c)will use get_ps_state)
//A modified version of sta_doze(bool flag)
void sta_sleep_disconnected(void)
{
#ifdef POWERSAVE_DEBUG
	pr_info("%s\n", __func__);
#endif

	UWORD32 tmp_mask = 0;
	UWORD32 int_mask = 0;
	UWORD32 int_stat = 0;

	/*zhou huiquan add protect rw reg*/
	mutex_lock(&rw_reg_mutex);

	//leon liu added ,if already in SLEEP state, return
	if (g_trout_state == TROUT_SLEEP)
	{
		mutex_unlock(&rw_reg_mutex);
		return ;
	}
	/* Change state to doze */
	//leon liu masked set_ps_state,stay in STA_ACTIVE state
	//set_ps_state(STA_DOZE);
	g_trout_state = TROUT_SLEEP;

	/* Suspend MAC H/w transmission before disabling it */
	root_set_machw_tx_suspend();

	/* Disable MAC H/w and PHY */
	root_disable_machw_phy_and_pa();

	/* Power down the PHY */
	/* by zhao */
	//root_power_down_phy();

	/* Indicate STA is dozing to MAC-HW */
	root_set_hw_ps_mode();

	//  caisf add for unmask relate intterupt 0319
	/*pr_info("rMAC_PA_CON = %#x\n", root_host_read_trout_reg((UWORD32)rMAC_PA_CON));*/
	ps_last_int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
	//tmp_mask = ((ps_last_int_mask & 0x3ffffc) & (~(BIT29 << 2)));
	tmp_mask = (~(BIT29 << 2));

	/*pr_info("sleep ps_last_int_mask = %#x; tmp_mask = %#x\n",ps_last_int_mask,tmp_mask);*/
	root_host_write_trout_reg(tmp_mask , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(tmp_mask);
	root_host_write_trout_reg(0xfffffffc, (UWORD32)rCOMM_INT_CLEAR);
	msleep(10);

	root_notify_cp_with_handshake(PS_MSG_ARM7_HANDLE_TBTT_MAGIC, 3);
	g_enable_machw_clock_flag = 0;

	pr_info("%s: going to sleep!\n", __func__);
	/*zhou huiquan add*/
	mutex_unlock(&rw_reg_mutex);
}

//leon liu added sta_wakeup_disconnected
//This function will wake up STA from sleep when STA is not connected
void sta_wakeup_disconnected(void)
{
	UWORD32 sysval = 0;

#ifdef POWERSAVE_DEBUG
	pr_info("%s: entered\n", __func__);
	TROUT_DBG4("sta_awake read the sleep status = 0x%x\n",
			root_host_read_trout_reg((UWORD32)(0x22<<2)));
#endif

	sysval = root_host_read_trout_reg((UWORD32)(0x22<<2));
	//notify_cp_with_handshake(PS_MSG_HOST_WAKE_WIFI_MAGIC, 3);
	root_notify_cp_with_handshake(PS_MSG_HOST_HANDLE_TBTT_MAGIC, 3);
	/*pr_info("rMAC_PA_CON = %#x\n", root_host_read_trout_reg((UWORD32)rMAC_PA_CON));*/
	g_enable_machw_clock_flag = 1;

	/* Power up the PHY */
	/*power_up_phy();*/ /*by kelvin*/

	/* Enable MAC H/w and PHY */
	/*enable_machw_phy_and_pa();*/ /*by kelvin*/
	/*enable_machw_phy_and_pa(); [>by kelvin<]*/
	if(!(root_host_read_trout_reg((UWORD32)rMAC_PA_CON) & BIT0))
		root_enable_machw_phy_and_pa(); /*in case CP failed to enable MAC, by keguang*/

	/* Test wifi wake up 2013.3.19 ranqing.wu*/
	/*pr_info("resume ps_last_int_mask = %#x; \n",ps_last_int_mask);*/
	root_host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(ps_last_int_mask);

	/* Check if STA can transmit now  */


	/* Resume MAC H/w transmission after enabling it */
	if((root_host_read_trout_reg((UWORD32)rMAC_PA_CON) & BIT12))
		root_set_machw_tx_resume(); /*in case CP failed to resume TX, by keguang*/
	/*pr_info("rMAC_PA_CON = %#x\n", root_host_read_trout_reg((UWORD32)rMAC_PA_CON));*/

	// 20120709 caisf masked, merged ittiam mac v1.2 code
	/* Indicate to MAC-HW STA is AWAKE */
	//set_machw_ps_active_mode();

	// goto active
	if(g_wifi_suspend_status == wifi_suspend_nosuspend)
	{
		pr_info("%s: wake up, man!\n", __func__);
		//root_notify_cp_with_handshake(PS_MSG_HOST_HANDLE_TBTT_MAGIC, 3);
		g_enable_machw_clock_flag = 1;

		root_reset_hw_ps_mode();

		//root_reset_machw_tbtt_int();
		//root_unmask_machw_tbtt_int();
		/*unmask TBTT INTR from host side*/
	}

	/*zhou huiquan add change protect state*/
	g_trout_state = TROUT_AWAKE;

	return ;
}

//leon liu added sta_wakup_combo 2013-4-3
//This function is used to replace trout_awake_fn
//sta_wakup_combo mainly deals with TROUT_SLEEP state when STA
//is not connected and in sleep state
void sta_wakeup_combo(UWORD8 unused)
{
#ifdef POWERSAVE_DEBUG
	//pr_info("%s: g_trout_state = %d\n", __func__, g_trout_state);
#endif

	switch (g_trout_state)
	{
		case TROUT_SLEEP:
			sta_wakeup_disconnected();
			break;
		case TROUT_DOZE:
			sta_awake(LOCK_FALSE);
			break;
		default:
			break;
	}
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_activity_timer                                     */
/*                                                                           */
/*  Description   : This function starts the activity timer                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_psm_alarm_handle                                       */
/*                                                                           */
/*  Processing    : The activity timer is started for the STA in power save  */
/*                  mode whenever there is transmission or reception. This   */
/*                  is a key function of the power management design which   */
/*                  predicts the activity at the STA and if no activity is   */
/*                  detected in the time the STA switches to DOZE mode. In   */
/*                  active state this timer allows the STA to indicate its   */
/*                  presence to the AP at regular intervals so that the STA  */
/*                  may recover from any undetected deauth/disassoc event.   */
/*                                                                           */
/*  Outputs      : None                                                      */
/*  Returns      : None                                                      */
/*  Issues       : None                                                      */
/*                                                                           */
/*****************************************************************************/

void start_activity_timer(void)
{
	// libing fix the buger : can't delete the psm timer.
    //stop_alarm(g_psm_alarm_handle);

    if(mget_PowerManagementMode() == MIB_POWERSAVE)
    {
        start_alarm(g_psm_alarm_handle, MIN(MIN_PS_ACTIVITY_TO,
                                           (mget_BeaconPeriod() >> 1)));
    }
    else
    {
        g_activity_timeout = ACTIVITY_TIME_OUT;
        start_alarm(g_psm_alarm_handle, g_activity_timeout);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : stop_activity_timer                                      */
/*                                                                           */
/*  Description   : This function stops the activity timer                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_psm_alarm_handle                                       */
/*                                                                           */
/*  Processing    : The activity timer is stopped if enabled.                */
/*                                                                           */
/*  Outputs      : None                                                      */
/*  Returns      : None                                                      */
/*  Issues       : None                                                      */
/*                                                                           */
/*****************************************************************************/

void stop_activity_timer(void)
{
    if(g_psm_alarm_handle != NULL)
        stop_alarm(g_psm_alarm_handle);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : psm_alarm_fn                                             */
/*                                                                           */
/*  Description   : Activity timeout alarm handler                           */
/*                                                                           */
/*  Inputs        : 1) Alarm handle                                          */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : If all the H/w queues are not empty the activity timer   */
/*                  is restarted. If there are no pending transmissions then */
/*                  if the STA is in Power Save mode, it sends a NULL frame  */
/*                  to the AP to indicate switch to DOZE state and sets a    */
/*                  global flag to indicate this. The state is changed to    */
/*                  AWAKE. If the STA is not in Power Save mode it sends a   */
/*                  NULL frame to AP to indicate the STA is in ACTIVE state. */
/*                  The activity timer is then started with an increased     */
/*                  period limited by a maximum value.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void psm_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void psm_alarm_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
    UWORD8  i     = 0;
    UWORD16 num_q = get_no_tx_queues();
	//chenq add 2012-12-06 check ptk in soft q start
	qmu_tx_handle_t *tx_handle = &g_q_handle.tx_handle;
	struct trout_private *tp = netdev_priv(g_mac_dev);

      if(reset_mac_trylock() == 0)
	{
		return;
	}
	ALARM_WORK_ENTRY(work);
	//End:add by wulei 2791 for bug 160423 on 2013-05-04

#ifdef WAKE_LOW_POWER_POLICY
	if(g_wifi_power_mode){
		ALARM_WORK_EXIT(work);
  		reset_mac_unlock();     
		return;
	}
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
		pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d, g_tx_rx_activity_cnt %d\n", __func__, g_wifi_suspend_status, g_tx_rx_activity_cnt);
	   ALARM_WORK_EXIT(work);
	   reset_mac_unlock();     
		return;
	}
#endif
#endif

    /* If there has been activity in the previous acitivity interval, then */
    /* reset the activity counter and restart activity timer with doubling */
    /* the activity interval.                                              */
    if((mget_PowerManagementMode() != MIB_POWERSAVE) &&
       (is_activity_cnt_zero() == BFALSE))
    {
        g_activity_timeout = ACTIVITY_TIME_OUT;
        start_alarm(g_psm_alarm_handle, g_activity_timeout);
        reset_activity_cnt();
        ALARM_WORK_EXIT(work);
	 reset_mac_unlock();
        return;
    }

	//chenq add 2012-12-06 check ptk in soft q start
	/* Send the NULL frame only if there are no packets in the transmit side */
    for(i = 0; i < num_q; i++)
    {
        if(tx_handle->tx_header[i].element_to_load != NULL )
	{
		break;
	}
    }
	if(i<num_q)
	{
		start_activity_timer();
		ALARM_WORK_EXIT(work);
		reset_mac_unlock();
		return;
	}
	//chenq add 2012-12-06 check ptk in soft q end


    /* Send the NULL frame only if there are no packets in the transmit side */
    for(i = 0; i < num_q; i++)
    {
        if(is_machw_q_null(i) == BFALSE)
            break;
    }

    if(i == num_q)
    {
        /* If there are no pending transmissions and no data is expected,    */
        /* switch to DOZE state in power save mode. If data is expected then */
        /* restart the activity timer.                                       */
        if(mget_PowerManagementMode() == MIB_POWERSAVE)
        {
            if(g_more_data_expected == BFALSE)
            {
                init_ps_state_change(STA_DOZE);
            }
            else
            {
                start_activity_timer();
            }
        }
        else
        {
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
		if(mutex_trylock(&tp->cur_run_mutex)){
			/* Send NULL frame and Restart the activity timer with increased timeout */
			if(g_wifi_suspend_status == wifi_suspend_nosuspend){
			send_null_frame_to_AP(STA_ACTIVE, BFALSE, 0);
			}
			mutex_unlock(&tp->cur_run_mutex);
		}
#endif
#endif
            g_activity_timeout = ACTIVITY_TIME_OUT;
            start_alarm(g_psm_alarm_handle, g_activity_timeout);
        }
    }
    else
    {
        /* Restart activity timer if there are pending transmissions */
        start_activity_timer();
    }
	//danny deng add it for debuging bug 160423 on 2013-05-08
	/*printk("PSM alarm work exit\n");*/
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

//add by Hugh
#ifdef OS_LINUX_CSL_TYPE

void psm_alarm_fn(ADDRWORD_T data)
{
	//Begin:add by wulei 2791 for bug 160423 on 2013-05-04
	/*printk("PSM alarm scheduling work\n");*/

	if((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE)
	{
		/*printk("PSM alarm not schedule work\n");*/
		return;
	}
	//End:add by wulei 2791 for bug 160423 on 2013-05-04
#ifdef WAKE_LOW_POWER_POLICY
	if(g_wifi_power_mode)
		return;
#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
		pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		return;
	}
#endif
#endif
	alarm_fn_work_sched(data);
	//danny deng add it for debuging bug 160423 on 2013-05-08
	/*printk("PSM alarm scheduling work exit\n");*/
}

#endif

#endif /* IBSS_BSS_STATION_MODE */
