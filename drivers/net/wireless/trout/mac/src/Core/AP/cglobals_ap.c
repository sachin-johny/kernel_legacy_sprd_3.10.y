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
/*  File Name         : cglobals_ap.c                                        */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      the AP mode.                                         */
/*                                                                           */
/*  List of Functions : init_globals_ap                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_ap.h"
#include "itypes.h"
#include "ap_prot_if.h"
#include "pm_ap.h"

#ifdef MEM_STRUCT_SIZES_INIT
#include "radius_client.h"
#endif /* MEM_STRUCT_SIZES_INIT */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/


/* Power management related globals */
UWORD8       g_num_sta_ps              = 0; /* Num of STA in PS */
UWORD16      g_num_mc_bc_pkt           = 0; /* Num of BC/MC pkts in PSQ     */
UWORD32      g_num_ps_pkt              = 0; /* Num of UC/BC/MC pkts in PSQ  */
UWORD16      g_num_mc_bc_qd_pkt        = 0; /* Num of BC/MC pkts in Hw      */
// 20120709 caisf add, merged ittiam mac v1.2 code
UWORD8		 g_update_active_bcn	   = 0; /* Enable Active Beacon directly */
list_buff_t  g_mc_q;
UWORD8       g_vbmap[VBMAP_SIZE]       = {0,};
UWORD8       g_tim_element_index       = 0;
UWORD16      g_tim_element_trailer_len = 0;
q_head_t     g_ps_pending_q            = {0,};
UWORD16      g_aging_thresh_in_sec     = AGING_THR_IN_SEC;
asoc_entry_t *g_max_ps_ae = NULL;      /* Association entry of STA, with max num of PS pkts */

#ifdef PS_DSCR_JIT
/*****************************************************************************/
/* This queue is not used currently. May be used if descriptors are not      */
/* buffered and are created just in time before transmission                 */
/*****************************************************************************/
list_buff_t  g_ps_pending_q;
#endif /* PS_DSCR_JIT */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_globals_ap                                          */
/*                                                                           */
/*  Description   : This function initializes all AP MAC globals             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Initialize MAC Core global variables.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_globals_ap(void)
{
	TROUT_FUNC_EXIT;

    /* Initialize BSS State counters */
    g_num_sta_no_short_slot  = 0;
    g_num_sta_non_erp        = 0;
    g_num_sta_no_short_pream = 0;
    g_erp_aging_cnt          = ERP_AGING_THRESHOLD;

    /* Initialize the virtual bitmap */
    g_vbmap[TYPE_OFFSET]        = ITIM;                  /* Element ID       */
    g_vbmap[LENGTH_OFFSET]      = DEFAULT_TIM_LEN;       /* Element Length   */
    g_vbmap[DTIM_CNT_OFFSET]    = 0;                     /* Dtim Count       */
    g_vbmap[DTIM_PERIOD_OFFSET] = mget_DTIMPeriod();     /* Dtim Period      */
    g_vbmap[BMAP_CTRL_OFFSET]   = 0;                     /* Bitmap Control   */
    g_vbmap[TIM_OFFSET]         = 0;                     /* Copy TIM element */

    /* Length of all the elements after the TIM element in the beacon */
    g_tim_element_trailer_len   = 0;

    /* Initialise the power save related variables */
    g_num_sta_ps                = 0;
    g_num_mc_bc_pkt             = 0;
    g_num_ps_pkt                = 0;
	// 20120709 caisf add, merged ittiam mac v1.2 code
    g_num_mc_bc_qd_pkt          = 0;
    g_num_mc_bc_qd_pkt			= 0;
	g_update_active_bcn			= 0;

    init_list_buffer(&g_mc_q,
        TX_DSCR_NEXT_ADDR_WORD_OFFSET * sizeof(UWORD32));

    /* Initialize Globals controlling channel Management */
    init_chan_mgmt_ap_globals();

    /* Initialize Protocol dependent Globals */
    init_prot_globals_ap();

#ifdef MEM_STRUCT_SIZES_INIT

    strncpy(g_struct_sizes[g_struct_count].name, "asoc_entry_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(asoc_entry_t) ;

    strncpy(g_struct_sizes[g_struct_count].name, "pm_buff_element_t", MEM_MAX_STRUCT_NAME) ;
    g_struct_sizes[g_struct_count++].size = sizeof(pm_buff_element_t) ;

#ifdef MAC_802_11I

#ifdef MAC_802_1X
// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	strncpy(g_struct_sizes[g_struct_count].name, "resp_pending_t", MEM_MAX_STRUCT_NAME) ;
    	g_struct_sizes[g_struct_count++].size = sizeof(resp_pending_t) ;

    	strncpy(g_struct_sizes[g_struct_count].name, "radius_node_t", MEM_MAX_STRUCT_NAME) ;
    	g_struct_sizes[g_struct_count++].size = sizeof(radius_node_t) ;
	}
#endif /* MAC_802_1X */

    strncpy(g_struct_sizes[g_struct_count].name, "auth_t", MEM_MAX_STRUCT_NAME) ;

    g_struct_sizes[g_struct_count++].size = sizeof(auth_t) ;
#endif /*MAC_802_11I*/

#endif /*MEM_STRUCT_SIZES_INIT*/
	TROUT_FUNC_EXIT;
}

#endif /* BSS_ACCESS_POINT_MODE */
