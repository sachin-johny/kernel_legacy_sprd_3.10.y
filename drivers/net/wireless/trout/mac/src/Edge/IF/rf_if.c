/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : rf_if.c                                              */
/*                                                                           */
/*  Description       : This file contains the functions required by the RF  */
/*                      interface.                                           */
/*                                                                           */
/*  List of Functions : initialize_rf                                        */
/*                      select_channel                                       */
/*                      get_current_tssi_11a                                 */
/*                      get_current_tssi_11b                                 */
/*                      get_rf_channel_num                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "rf_if.h"
#include "mh.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_dscr_tssi_11b  = 0; /* Current TSSI of 11b     */
UWORD8 g_dscr_tssi_11gn = 0; /* Current TSSI of 11g/11n */
UWORD8 g_rf_channel     = 0; /* Current channel index programmed in RF */

#ifdef TROUT2_WIFI_IC /* Export from Trout_sdio.ko  :: zhangym 0403 */
extern int  MxdRfSetFreqWifiCh(unsigned int chNo);
extern int update_cfg_front_nv(void); //by lihua
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_rf                                            */
/*                                                                           */
/*  Description   : This function initializes the RF card in use.            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the channel table and other RF */
/*                  registers. It is initialized to the default channel and  */
/*                  transmit power.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_rf(void)
{
#ifndef GENERIC_RF
    UWORD8 freq          = get_current_start_freq();
    UWORD8 ch_idx        = mget_CurrentChannel();
    UWORD8 sec_ch_offset = get_bss_sec_chan_offset();
    UWORD8 tx_power      = get_tx_power_level_11a();
#endif /* GENERIC_RF */

	TROUT_FUNC_ENTER;
#ifdef GENERIC_RF
    return;
#endif /* GENERIC_RF */

#ifdef IBSS_BSS_STATION_MODE
	//caisf add, 20130505
    /* Do not connect to the ap which channel is out of max channel number*/
    if((ch_idx+1) > get_scan_req_channel_no())
    {
		TROUT_DBG3("The channel %d is out of max channel number range!\n",
			(ch_idx+1));
		ch_idx = 0;
    }
#endif

#ifdef RF_MAXIM_ITTIAM
    init_maxim_ittiam_rf(freq, ch_idx, sec_ch_offset, tx_power);
#endif /* RF_MAXIM_ITTIAM */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef RF_AIROHA_ITTIAM
    init_airoha_ittiam_rf(freq, ch_idx, sec_ch_offset, tx_power);
#endif /* RF_AIROHA_ITTIAM */
#endif

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_rf_2040                                       */
/*                                                                           */
/*  Description   : This function initializes channel bandwidth of the RF    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates all the relevant registers when a  */
/*                  Channel bandwidth switch is performed. It does not       */
/*                  however program the channel center frequency             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_rf_2040(void)
{
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : select_channel                                           */
/*                                                                           */
/*  Description   : This function sets the channel for the RF in use.        */
/*                                                                           */
/*  Inputs        : 1) Primary Channel Index                                 */
/*                : 2) Secondary Channel Offset                              */
/*                     0 - No Secondary Channel                              */
/*                     1 - Secondary Channel Above Primary Channel           */
/*                     3 - Secondary Channel Below Primary Channel           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function programs the registers of the RF in use to */
/*                  the given channel.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void select_channel_rf(UWORD8 channel, UWORD8 sec_offset)
{
    BOOL_T is_tx_already_suspended = BFALSE;
    BOOL_T is_pa_already_enabled   = BFALSE;
    UWORD8 freq                    = get_current_start_freq();

    critical_section_start();

    is_pa_already_enabled   = is_machw_enabled();
    is_tx_already_suspended = is_machw_tx_suspended();

    /* Suspend the current transmission */
    if(is_tx_already_suspended == BFALSE)
        set_machw_tx_suspend();

    /* Disable MACHW & PHY before changing the channel */
    if(is_pa_already_enabled == BTRUE)
        disable_machw_phy_and_pa();

	/* Reset secondary channel offset if 40MHz is not enabled */
    if(mget_FortyMHzOperationEnabled() == TV_FALSE)
    {
#ifndef GENERIC_RF
		sec_offset = 0;
#endif /* GENERIC_RF */
	}
// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef TEST_2040_MACSW_KLUDGE
    PRINTD("SwDb: select_channel: PChan=%d SCO=%d\n\r",channel, sec_offset);
#endif /* TEST_2040_MACSW_KLUDGE */
#endif
    update_phy_on_select_channel(freq, channel);

#ifndef GENERIC_RF
    
/* Switch to the desired channel */
//chenq add for 2012-12-20
#ifdef TROUT_WIFI_FPGA 
    set_channel(freq, channel, sec_offset);
#endif//dumy add for Trout IC RF 20121026

#ifdef TROUT2_WIFI_FPGA
	#ifdef TROUT2_WIFI_FPGA_RF55
		trout2_mxd_rf_channel_config(channel+1); //caisf temp for debug 20130116
	#endif
	
	#ifdef TROUT2_WIFI_FPGA_RF2829
		set_channel(freq, channel, sec_offset);
	#endif
#endif

#ifdef TROUT2_WIFI_IC
	MxdRfSetFreqWifiCh(channel+1);
	update_cfg_front_nv(); //by lihua
#endif
#ifdef TROUT_WIFI_EVB

#ifdef TROUT_WIFI_EVB_MF_RF55
	trout2_mxd_rf_channel_config(channel+1); //caisf temp for debug 20130116
#else
    trout_ic_set_channel(freq, channel, sec_offset);
#endif

#endif

#endif /* GENERIC_RF */

    /* Enabled MAC HW & PHY after changing channel  */
    if(is_pa_already_enabled == BTRUE)
         enable_machw_phy_and_pa();

    /* Release the tranmit suspension */
    if(is_tx_already_suspended == BFALSE)
         set_machw_tx_resume();

    critical_section_end();

    g_rf_channel = channel;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_current_tssi_11a                                     */
/*                                                                           */
/*  Description   : This function gets the current TSSI 11a value from the Tx*/
/*                  descriptor.                                              */
/*                                                                           */
/*  Inputs        : 1) Transmit Descriptor                                   */
/*                                                                           */
/*                                                                           */
/*  Globals       : g_dscr_tssi_11gn                                         */
/*                                                                           */
/*  Processing    : This function extracts the TSSI value from the Tx        */
/*                  descriptor and updates the required global variable      */
/*                  based on the transmit rate.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Current TSSI value                               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_current_tssi_11a(UWORD8 *tx_dscr)
{
    UWORD8 cur_tssi = 0;

    /* Read current TSSI */
    cur_tssi = get_tx_dscr_tssi_value((UWORD32 *)tx_dscr);
    if(cur_tssi)
    {
        g_dscr_tssi_11gn = cur_tssi;
    }

    return cur_tssi;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_current_tssi_11b                                     */
/*                                                                           */
/*  Description   : This function gets the current 11b TSSI value from the Tx*/
/*                  descriptor.                                              */
/*                                                                           */
/*  Inputs        : 1) Transmit Descriptor                                   */
/*                                                                           */
/*                                                                           */
/*  Globals       : g_dscr_tssi_11b                                          */
/*                                                                           */
/*  Processing    : This function extracts the TSSI value from the Tx        */
/*                  descriptor and updates the required global variable      */
/*                  based on the transmit rate.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Current 11b TSSI value                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_current_tssi_11b(UWORD8 *tx_dscr)
{
    UWORD8 cur_tssi = 0;

    /* Read current TSSI */
    cur_tssi = get_tx_dscr_tssi_value((UWORD32 *)tx_dscr);
    if(cur_tssi)
    {
        g_dscr_tssi_11b  = cur_tssi;
    }

    return cur_tssi;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_rf_channel_idx                                       */
/*                                                                           */
/*  Description   : This function gets the channel index  in which RF is in  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_rf_channel                                             */
/*                                                                           */
/*  Processing    : This function gets the previously set RF channel         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Current Channel of RF                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_rf_channel_idx(void)
{
	return g_rf_channel;
}

// caisf add, 20130506
UWORD8 set_rf_channel_idx(UWORD8 ch_idx)
{
	g_rf_channel = ch_idx;
	return g_rf_channel;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_rf_channel_num                                       */
/*                                                                           */
/*  Description   : This function gets the channel number in which RF is in  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_rf_channel                                             */
/*                                                                           */
/*  Processing    : This function gets the previously set RF channel         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Current Channel of RF                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_rf_channel_num(void)
{
	return get_ch_num_from_idx(get_current_start_freq(), g_rf_channel);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : convert_dbm_to_regval                                    */
/*                                                                           */
/*  Description   : This function converts a given TX power value in dBm to  */
/*                  the corresponding gain code for Maxim RF with Ittiam PHY */
/*                                                                           */
/*  Inputs        : 1) Power Value in dBm                                    */
/*                  2) Frequency band                                        */
/*                  2) Channel index                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses the TX power information LUT to get   */
/*                  the gain code corresponding to the given power. Note     */
/*                  that the LUT is maintained for a set of discrete power   */
/*                  levels. Thus the gaincode for the maximum power level    */
/*                  that is less than the given power value is determined    */
/*                  and the gain code for the same is returned.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8: Power gain code                                  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 convert_dbm_to_regval(WORD8 dbm, UWORD8 freq, UWORD8 ch_idx)
{
    UWORD8 gaincode       = 0;
#ifndef GENERIC_RF
    UWORD8 i              = 0;
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    //UWORD8 **gaincode_arr = 0;
    UWORD8 (*gaincode_arr)[MAX_TX_POWER_LEVELS] = 0;

#ifndef TX_POWER_MAINTAIN_PER_CHANNEL
    ch_idx = 0;
#endif /* TX_POWER_MAINTAIN_PER_CHANNEL */

    switch(freq)
    {
    case RC_START_FREQ_2:
    {
        gaincode_arr = (UWORD8 **)g_gaincode_freq_2;
    }
    break;
    case RC_START_FREQ_5:
    {
        gaincode_arr = (UWORD8 **)g_gaincode_freq_5;
    }
    break;
    default:
    {
        /* Not valid */
    }
    break;
    }

    if(gaincode_arr == 0)
        return 0;

    for(i = 0; i < MAX_TX_POWER_LEVELS; i++)
    {
        if(dbm >= g_tx_power_levels_in_dbm[i])
        {
			// 20120709 caisf mod, merged ittiam mac v1.2 code
            //gaincode = *(UWORD8 *)(gaincode_arr + ch_idx * MAX_TX_POWER_LEVELS + i);
			gaincode = gaincode_arr[ch_idx][i];
            break;
        }
    }
#endif /* GENERIC_RF */

    return gaincode;
}

#ifdef TX_POWER_LOOPBACK_ENABLED
/* RAJENDRA TBD */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : correct_tx_gain_value                                    */
/*                                                                           */
/*  Description   : This function corrects the TX gaincode value based on    */
/*                  the expected and current TSSI values.                    */
/*                                                                           */
/*  Inputs        : 1) Frequency band                                        */
/*                  2) Channel index                                         */
/*                  3) Pointer to the TX descriptor                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : TBD                                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Outputs       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void correct_tx_gain_value(UWORD8 freq, UWORD8 ch_idx, UWORD32 *dscr)
{
#ifndef GENERIC_RF
	UWORD32 ptm      = 0;
    WORD8   dbm      = 0;
	UWORD8  dr       = 0;
    UWORD8  gaincode = 0;
    UWORD8  i        = 0;
    UWORD8  curr_tssi = 0;
    UWORD8  expected_tssi = 0;
	tx_power_t *curr_tx_pow = 0;
	UWORD8     *tssi_arr = 0;

	/* Get the TX rate and PHY TX Mode from the TX descriptor */
	dr  = get_tx_dscr_data_rate_0(dscr);
	ptm = get_tx_dscr_phy_tx_mode(dscr);

	/* Get the TSSI value from the TX descriptor */
	curr_tssi = get_tx_dscr_tssi_value(dscr);

	/* Get the current power information (dbm and gain value) for the given  */
	/* data rate, channel bandwidth                                          */
	curr_tx_pow = get_current_tx_pow(dr, ptm)

	dbm      = curr_tx_pow->dbm;
	gaincode = curr_tx_pow->gaincode;

	/* Get the expected TSSI from the power table using dbm and channel */
    switch(freq)
    {
    case RC_START_FREQ_2:
    {
        tssi_arr = g_tssi_freq_2;
    }
    break;
    case RC_START_FREQ_5:
    {
        tssi_arr = g_tssi_freq_5;
    }
    break;
    default:
    {
        /* Not valid */
    }
    break;
    }

    if(tssi_arr == 0)
        return 0;

    for(i = 0; i < MAX_TX_POWER_LEVELS; i++)
    {
        if(dbm >= g_tx_power_levels_in_dbm[i])
        {
            expected_tssi = tssi_arr[ch_idx][i];
            break;
        }
    }

	/* Adjust the gain code based on the expected TSSI and current TSSI */
	if(curr_tssi > (expected_tssi + ALLOWED_TSSI_BAND))
	{
		if(gaincode <= (MIN_GAINCODE_VAL + TX_GAINCODE_STEP_SIZE))
		    gaincode = MIN_GAINCODE_VAL;
		else
			gaincode -= TX_GAINCODE_STEP_SIZE;
	}
	else if(curr_tssi < (expected_tssi - ALLOWED_TSSI_BAND))
	{
		if(gaincode >= (MAX_GAINCODE_VAL - TX_GAINCODE_STEP_SIZE))
		    gaincode = MAX_GAINCODE_VAL;
		else
		    gaincode += TX_GAINCODE_STEP_SIZE;
	}

	curr_tx_pow->gaincode = gaincode;
#endif /* GENERIC_RF */
}

#endif /* TX_POWER_LOOPBACK_ENABLED */
