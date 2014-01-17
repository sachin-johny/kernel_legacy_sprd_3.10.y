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
/*  File Name         : tpc_sta.c                                            */
/*                                                                           */
/*  Description       : All BSS STA Transmission power Control related       */
/*                      routines are placed in this file.                    */
/*                                                                           */
/*  List of Functions :update_power_constr                                   */
/*                     check_for_tx_power                                    */
/*                     set_power_cap_element                                 */
/*                     send_tpc_report                                       */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
// 20120830 caisf mod, merged ittiam mac v1.3 code

#ifdef MAC_802_11H

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_sta.h"
#include "mib1.h"
#include "frame.h"
#include "transmit.h"
#include "phy_prot_if.h"
#include "index_util.h"
#include "qmu_if.h"
#include "dfs_sta.h"
#include "tpc_sta.h"
#include "core_mode_if.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

void update_local_max_power(UWORD8 pw_cnstr_latest);

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static UWORD8 pw_cnstr        = 0;  /* in dB */
static UWORD8 prev_rc_max_pow = 0; /* in dBm */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_power_constr                                      */
/*                                                                           */
/*  Description   : this function is used to update power constraint value.  */
/*                                                                           */
/*  Inputs        : 1) Beacon Frame                                          */
/*                  2) Length of the frame                                   */
/*                  3) Index from where the search should start              */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : AP will send power constraint element in the beacon      */
/*                  STA will update this power constraint to                 */
/*                  calculate local maximum power for the current channel.   */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_power_constr (UWORD8 *msa,UWORD16 rx_len,UWORD16 tag_param_offset)
{
    UWORD16 index          = 0;
    UWORD16 index_pw_cnstr = 0;
    UWORD8 *elem_pw_cnstr  = 0;

    elem_pw_cnstr = 0;

  /*************************************************************************/
  /*                       Power Constraint Element                        */
  /*                -------------------------------------                  */
  /*                |Element ID|Length |Power Constraint|                  */
  /*                -------------------------------------                  */
  /*                |1         |1      |1               |                  */
  /*                -------------------------------------                  */
  /*                        Beacon Frame body                              */
  /*************************************************************************/

   /* Power constraint is in dB */
    index = tag_param_offset;

    /* Searching for power constraint element */
    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IPOWERCONSTRAINT)
        {
            elem_pw_cnstr     = &msa[index];
            index_pw_cnstr    = index;
            break;
        }
        else
        {
           index += (IE_HDR_LEN + msa[index + 1]);
        }
    }

    if(elem_pw_cnstr != 0)
    {
        /* Update local max power */
		update_local_max_power(msa[index_pw_cnstr+2]);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_local_max_power                                   */
/*                                                                           */
/*  Description   : this function is used to check Tx power.                 */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_tx_power_level_11a,reg_max_power,pw_cnstr              */
/*                                                                           */
/*  Processing    : Calculation of local maximum power by using regulatory   */
/*                  max power and power constraint                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         13 10 2008   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void update_local_max_power(UWORD8 pw_cnstr_latest)
{
    UWORD8 freq               = get_current_start_freq();
    UWORD8 ch_idx             = mget_CurrentChannel();
    UWORD8 reg_max_power      = 0;
    UWORD8 local_max_power    = 0;
    UWORD8 reg_class          = get_reg_class_from_ch_idx(freq, ch_idx);
    reg_class_info_t *rc_info = get_reg_class_info(freq, reg_class,
                                                   get_current_reg_domain());

    reg_max_power = get_rc_max_reg_tx_pow(rc_info);

	if(rc_info == NULL)
	{
		PRINTD("Warning: NULL Reg Class Info while updating local max pwr");
		return;
	}

	/* Update power constraint if there is any change from previous value */
    if((pw_cnstr_latest != pw_cnstr) || (prev_rc_max_pow != reg_max_power))
    {
		pw_cnstr = pw_cnstr_latest;

   	    /*in dBm*/        /*in dBm*/       /*in dB */
    	local_max_power = reg_max_power - pw_cnstr;

		prev_rc_max_pow = local_max_power;
		set_rc_max_reg_tx_pow(rc_info, local_max_power);
    	update_curr_rc_max_tx_pow();
	}

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_power_cap_element                                    */
/*                                                                           */
/*  Description   : This function sets the Min and Max Powers with which     */
/*                  a STA is capable of transmitting in the current          */
/*                  channel.                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the Power Capability element.                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Capable Tx power depends up on Rx                        */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the Power Capability element.                     */
/*                                                                           */
/*  Returns       : Power Capability element length.                         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 set_power_cap_element(UWORD8* data, UWORD16 index)
{
    WORD8  mintx       = 0;
    WORD8  maxtx       = 0;
    UWORD8 freq        = get_current_start_freq();
    UWORD8 ch_idx      = mget_CurrentChannel();
    UWORD8 reg_class   = get_reg_class_from_ch_idx(freq, ch_idx);

    reg_class_info_t *rc_info = get_reg_class_info(freq, reg_class,
                                                   get_current_reg_domain());

    /*************************************************************************/
    /*                    Power Capability Element Format                    */
    /* --------------------------------------------------------------------- */
    /* |Powercap   Ele ID | length  | Min Tx Power Cap| Max power Cap      | */
    /* --------------------------------------------------------------------- */
    /* | 1                | 1       |    1            |  1                 | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set the Power Capability element ID to the given frame at given       */
    /* index                                                                 */
    data[index] = IPOWERCAPABILITY;

    /* Length is fixed for Power capability element                          */
    data[index+1] = IPOWERCAPABILITY_LEN;

    /* Set Min power from the table  */
	if(rc_info != NULL)
    	mintx  = get_rc_min_rf_tx_pow(rc_info);

    /* Take  Max power Minimum of RF supported Max Tx power and Local Max Tx power*/
    maxtx  = MIN(get_rc_max_rf_tx_pow_a(rc_info), get_rc_max_reg_tx_pow(rc_info));

    data[index+2] = mintx;

    if(maxtx > mintx)
    {
        data[index+3] = maxtx;
    }
    else
    {
        data[index+3] = get_rc_max_rf_tx_pow_a(rc_info);
    }

    PRINTD("TPC Power Cap set: maxtx: %d\n", maxtx);

    return (IE_HDR_LEN + IPOWERCAPABILITY_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_tpc_power_limit                                    */
/*                                                                           */
/*  Description   : This function decided whether to join AP or not          */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Returns FALSE if Our RF's min tx power is greater than   */
/*                  Max power specified in AP's beacons                      */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE or BFALSE                                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         09 10 2008   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
BOOL_T check_tpc_power_limit(void)
{
#if 0 /* Attempt to join the AP always */
    UWORD8 freq        = get_current_start_freq();
    UWORD8 ch_idx      = mget_CurrentChannel();
    UWORD8 reg_class   = get_reg_class_from_ch_idx(freq, ch_idx);
    reg_class_info_t *rc_info = get_reg_class_info(freq, reg_class,
                                                   get_current_reg_domain());

    if(maxtx < get_rc_min_rf_tx_pow(rc_info))
    {
        PRINTD("TPCFail: MaxPwr of Network is < MinPwr of our RF");
        return BFALSE;
    }
    else
#endif
    {
        return BTRUE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_tpc_report                                          */
/*                                                                           */
/*  Description   : Prepare and send tpc report frame to AP.                 */
/*                                                                           */
/*  Inputs        : 1) rssi of TPC request frame                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the TPC report frame and          */
/*                  send to the AP.                                          */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_tpc_report(WORD8 rx_power_req,UWORD8 dia_token)
{
     UWORD8       q_num       = 0;
     UWORD16      index       = 0;
     UWORD8       *msa        = 0;

    /* Allocate buffer for the NULL Data frame. This frame contains only  */
    /* the MAC Header. The data payload of the same is '0'.               */
    msa  = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        return;
    }

    /* Set the Frame Control field of the NULL frame.                     */
    set_frame_control(msa, (UWORD16)ACTION);

    q_num = HIGH_PRI_Q;

    set_to_ds(msa, 1);

    /* Set the address fields. For a station operating in the infrastructure */
    /* mode, Address1 = BSSID, Address2 = Source Address (SA) and            */
    /* Address3 = Destination Address (DA) which is nothing but the BSSID.   */
    set_address1(msa, mget_bssid());
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_bssid());
    index = MAC_HDR_LEN;
    index = set_category(msa, SPECMGMT_CATEGORY,index);
    index = set_action(msa, TPCREPORT, index);
    index = set_dia_token(msa, dia_token,index);
    index = set_elem_id(msa,ITPCREPORT,index);
    index = set_length(msa,2,index);
    index = set_tx_power_field(msa,index);
    set_linkmargin_field(msa, rx_power_req,index);
    index +=1;

    if(BFALSE == tx_mgmt_frame(msa, index + FCS_LEN, q_num, 0))
    {
        PRINTD2("TPCErr: TPC Report Could not be sent\n");
    }
    else
    {
        PRINTD("TPC Report sent\n");
    }
}

#endif /* IBSS_BSS_STATION_MODE */

#endif /* MAC_802_11H */


