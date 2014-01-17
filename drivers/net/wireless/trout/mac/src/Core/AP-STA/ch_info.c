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
/*  File Name         : ch_info.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions for Channel         */
/*                      information.                                         */
/*                                                                           */
/*  List of Functions : init_supp_ch_list                                    */
/*                      update_ch_list_reg_domain                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ch_info.h"
#include "reg_domains.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_current_start_freq = RC_START_FREQ_2;

channel_info_t g_ch_info_freq_5[MAX_CHANNEL_FREQ_5] =
                {{ 36,  0}, { 40, 0}, { 44, 0}, { 48, 0}, { 52, 0}, { 56, 0},
                 { 60,  0}, { 64, 0}, {100, 0}, {104, 0}, {108, 0}, {112, 0},
                 {116,  0}, {120, 0}, {124, 0}, {128, 0}, {132, 0}, {136, 0},
                 {140,  0}, {149, 0}, {153, 0}, {157, 0}, {161, 0}, {165, 0}};

channel_info_t g_ch_info_freq_2[MAX_CHANNEL_FREQ_2] =
                {{1, 0}, {2, 0}, { 3, 0}, { 4, 0}, { 5, 0}, { 6, 0}, { 7,  0},
                 {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0}, {13,  0}, {14, 0}};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_supp_ch_list                                        */
/*                                                                           */
/*  Description   : This function initializes the channel list information   */
/*                  based on the supported information.                      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ch_info_freq_*                                         */
/*                                                                           */
/*  Processing    : This function initializes the channel list at the start  */
/*                  with supported information. For each starting frequency  */
/*                  the Regulatory Class value is set to "INVALID_REG_CLASS" */
/*                  if the channel is supported by the RF. This value shall  */
/*                  be updated later based on the Regulatory Domain in use.  */
/*                  The channel table index is set in incrementing order     */
/*                  starting from 0 for all these channels. These values     */
/*                  shall never be updated.                                  */
/*                  For channels not supported by RF the Regulatory Class    */
/*                  values are set to "NOT_SUPPORTED_BY_RF" and the channel  */
/*                  table index is set to "INVALID_CH_TBL_INDEX". These      */
/*                  values shall never be updated.                           */
/*                  Called along with "init_supp_reg_domain_rf" to configure */
/*                  the supported channel list information.                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_supp_ch_list(void)
{
	UWORD8 i          = 0;
	UWORD8 ch_tbl_idx = 0; /* Initialize starting channel table index to 0 */

	TROUT_FUNC_ENTER;
	for(i = 0; i < MAX_CHANNEL_FREQ_2; i++)
	{
		if(IS_CH_SUPP(g_ch_bmap_freq_2, i) == BTRUE)
		{
			g_ch_info_freq_2[i].reg_class = INVALID_REG_CLASS;
			g_ch_info_freq_2[i].ch_tbl_idx = ch_tbl_idx++;
		}
		else
		{
			g_ch_info_freq_2[i].reg_class = NOT_SUPPORTED_BY_RF;
			g_ch_info_freq_2[i].ch_tbl_idx = INVALID_CH_TBL_INDEX;
		}
	}

	ch_tbl_idx = 0; /* Reinitialize starting channel table index to 0 */

	for(i = 0; i < MAX_CHANNEL_FREQ_5; i++)
	{
		if(IS_CH_SUPP(g_ch_bmap_freq_5, i) == BTRUE)
		{
			g_ch_info_freq_5[i].reg_class = INVALID_REG_CLASS;
			g_ch_info_freq_5[i].ch_tbl_idx = ch_tbl_idx++;
		}
		else
		{
			g_ch_info_freq_5[i].reg_class = NOT_SUPPORTED_BY_RF;
			g_ch_info_freq_5[i].ch_tbl_idx = INVALID_CH_TBL_INDEX;
		}
	}
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_ch_list_reg_domain                                */
/*                                                                           */
/*  Description   : This function updates the Regulatory Class information   */
/*                  (corresponding to each channel) in the channel list for  */
/*                  each starting frequency as per the Regulatory Domain (as */
/*                  per information maintained internally).                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_ch_info_freq_*                                         */
/*                                                                           */
/*  Processing    : For each starting frequency the Regulatory Class value   */
/*                  for all channels is updated to the appropriate value if  */
/*                  it is not set to "NOT_SUPPORTED_BY_RF".                  */
/*                  The value is updated with the Regulatory Class Value     */
/*                  corresponding to each Channel and Starting Frequency as  */
/*                  per current Regulatory Domain information. In case there */
/*                  is more than one matching Regulatory Class for a channel */
/*                  any of the matched values can be used. For channels not  */
/*                  included in the Regulatory Domain, the Regulatory Class  */
/*                  value is reset to INVALID_REG_CLASS.                     */
/*                  Called from "set_reg_domain" to configure the Regulatory */
/*                  Domain to be used.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_ch_list_reg_domain(void)
{
	UWORD8 ch_idx = 0;
	UWORD8 rc_idx = 0;
	UWORD8 num_rc = 0;
	UWORD8 max_ch = 0;
	UWORD8 freq   = 0;
		
	// 20120830 caisf add, merged ittiam mac v1.3 code
    UWORD8 reg_domain = get_current_reg_domain(); 

	reg_domain_info_t *rd_info = 0;
    channel_info_t    *ch_info = 0;
	reg_class_info_t  *rc_info = 0;

	TROUT_FUNC_ENTER;
	// 20120830 caisf mod, merged ittiam mac v1.3 code
	#if 0
	/* Get current Regulatory Domain information and number of Regulatory    */
	/* Classes supported in the same.                                        */
    rd_info = get_current_reg_domain_info();
	#else
	/* Get current Regulatory Domain information and number of Regulatory    */
	/* Classes supported in the same.                                        */
    rd_info = get_reg_domain_info(reg_domain);

	if(rd_info == NULL)
	{
		TROUT_DBG3("MultiDomain Warning: Reg Domain Info is NULL\n\r");
		return;
	}
	#endif

    num_rc  = get_num_reg_class(rd_info);

    /* Update the channel list for Starting Frequency 2.4 GHz */
    freq    = RC_START_FREQ_2;
    ch_info = g_ch_info_freq_2;
    max_ch  = MAX_CHANNEL_FREQ_2;

    /* Traverse channel list to update Regulatory class for each channel */
	for(ch_idx = 0; ch_idx < max_ch; ch_idx++)
    {
		if(ch_info[ch_idx].reg_class != NOT_SUPPORTED_BY_RF)
		{
			/* Traverse Regulatory class information to find any Regulatory  */
			/* Class that supports this channel. If found, update the value  */
			/* in the channel information.                                   */
			for(rc_idx = 0; rc_idx < num_rc; rc_idx++)
			{
				rc_info = rd_info->reg_class_info[rc_idx];

				if(is_ch_supp_in_rc(rc_info, freq, ch_idx) == BTRUE)
                {
					// 20120830 caisf mod, merged ittiam mac v1.3 code
					//ch_info[ch_idx].reg_class = get_reg_class_val(rc_idx);
                    ch_info[ch_idx].reg_class = get_reg_class_val(rc_idx,
                                              						reg_domain);
					
					break;
                }
            }

			if(num_rc == rc_idx) /* Reached end of loop, No RC found */
			    ch_info[ch_idx].reg_class = INVALID_REG_CLASS;
		}
		else /* NOT_SUPPORTED_BY_RF */
        {
			/* If the channel is not supported by RF, no update is required */
        }
    }

    /* Update the channel list for Starting Frequency 5 GHz */
    freq    = RC_START_FREQ_5;
    ch_info = g_ch_info_freq_5;
    max_ch  = MAX_CHANNEL_FREQ_5;

    /* Traverse channel list to update Regulatory class for each channel */
	for(ch_idx = 0; ch_idx < max_ch; ch_idx++)
    {
		if(ch_info[ch_idx].reg_class != NOT_SUPPORTED_BY_RF)
		{
			/* Traverse Regulatory class information to find any Regulatory  */
			/* Class that supports this channel. If found, update the value  */
			/* in the channel information.                                   */
			for(rc_idx = 0; rc_idx < num_rc; rc_idx++)
			{
				rc_info = rd_info->reg_class_info[rc_idx];

				if(is_ch_supp_in_rc(rc_info, freq, ch_idx) == BTRUE)
                {
					// 20120830 caisf mod, merged ittiam mac v1.3 code
					//ch_info[ch_idx].reg_class = get_reg_class_val(rc_idx);
                    ch_info[ch_idx].reg_class = get_reg_class_val(rc_idx,
                                                                  reg_domain);
					break;
                }
            }

			if(num_rc == rc_idx) /* Reached end of loop, No RC found */
			    ch_info[ch_idx].reg_class = INVALID_REG_CLASS;
		}
		else /* NOT_SUPPORTED_BY_RF */
        {
			/* If the channel is not supported by RF, no update is required */
        }
    }

    /* MD-TBD : In case of BSS STA, need to update channel list based on the */
    /* regulatory domain information saved in the device and that received   */
    /* from the AP. This shall be a separate function which may or may not   */
    /* reuse this code.                                                      */
    TROUT_FUNC_EXIT;
}

// 20120830 caisf add, merged ittiam mac v1.3 code
/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_ch_list_reg_class                                 */
/*                                                                           */
/*  Description   : This function updates the Regulatory Class information   */
/*                  for the channels which are supported in the Regulatory   */
/*                  Class of RF table                                        */
/*                                                                           */
/*  Inputs        : Regulatory Class                                         */
/*                                                                           */
/*  Globals       : g_ch_info_freq_*                                         */
/*                                                                           */
/*  Processing    : This function updates Regulatory Class field of the      */
/*                  channel table. This function is called whenever          */
/*                  Regulatory Class is selected for the operation and       */
/*                  channel table is updated according to this               */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_ch_list_reg_class(UWORD8 reg_class)
{
    UWORD8 num_ch             = 0;
    UWORD8 ch_idx             = 0;
    UWORD8 freq               = get_current_start_freq();
    channel_info_t *ch_info   = NULL;
    reg_class_info_t *rc_info = NULL;

    ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(ch_info == NULL)
    {
        PRINTD("MultiDomain Warning: NULL ch_info\n\r");
        return;
    }

    rc_info = get_reg_class_info(freq, reg_class, get_current_reg_domain());

    if(rc_info == NULL)
    {
        PRINTD("MultiDomain Warning: NULL Reg Class Info\n\r");
        return;
    }

    for(ch_idx = 0; ch_idx < num_ch; ch_idx++)
    {
        /* Change Regulatory Class field if Regulatory class */
        /* supports the channel                              */
        if(is_ch_supp_in_rc(rc_info, freq, ch_idx) == BTRUE)
        {
            ch_info[ch_idx].reg_class = reg_class;
        }
    }

}
