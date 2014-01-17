/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : sta_management_mdom.c                                */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for Multi-domain operation.              */
/*                                                                           */
/*  List of Functions : update_scan_response_11d                             */
/*                      check_bss_reg_domain_cap_info                        */
/*                      update_11d_reg_max_tx_pow                            */
/*                      adjust_bss_width_in_new_rc                           */
/*                      adopt_new_rc                                         */
/*                     	get_sup_rc_of_chan_in_reg_dom                        */
/*                     	is_40mhz_op_to_be_enabled                            */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_MULTIDOMAIN

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "sta_management_mdom.h"
#include "sta_prot_if.h"
#include "common.h"
#include "frame.h"
#include "mib_mdom.h"

static BOOL_T is_40mhz_op_to_be_enabled(bss_dscr_t *bss_dscr);
static UWORD8 get_sup_rc_of_chan_in_reg_dom(UWORD8 ch_idx, UWORD8 reg_domain,
                                            UWORD8 sec_ch_offset,
                                            bss_dscr_t *bss_dscr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_scan_response_11d                                 */
/*                                                                           */
/*  Description   : This function updates the Regulatory Domain, Regulatory  */
/*                  Class and Regulatory Max powers of the BSS network.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame                         */
/*                  2) Received frame length                                 */
/*                  3) Index of the information elements                     */
/*                  4) Index to the BSS-Dscr Table entry                     */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                                                                           */
/*  Processing    : This function is called during scan period to find out   */
/*                  the Regulatory Domain in which BSS network is operating. */
/*                  And also updates Suitable Regulatory Class and           */
/*                  Max powers in the BSS descriptors.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_scan_response_11d(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                              UWORD8 dscr_set_index)
{
    UWORD8 cnt_str[4]            = {'\0', '\0','\0','\0'};
    UWORD8 len                   = 0;
    UWORD8 num_bands             = 0;
    UWORD8 cnt_bands             = 0;
    UWORD8 cnt_chan              = 0;
    UWORD8 ch_sp                 = 0;
    UWORD8 freq                  = get_current_start_freq();
    UWORD8 bss_dscr_rd           = NO_REG_DOMAIN;
    UWORD8 bss_dscr_rc           = UNKNOWN_REG_CLASS;
    UWORD8 max_reg_tx_pow        = 0xFF;
    BOOL_T max_rc_tx_pow_updated = BFALSE;
    reg_class_info_t *rc_info    = NULL;
    //UWORD8 sec_ch_offset         = g_bss_dscr_set[dscr_set_index].sec_ch_offset;

    /* Proceed if Multi-domain capability is enabled */
    if(mget_MultiDomainOperationImplemented() == TV_FALSE)
    {
        return;
    }

	/* Enable Multi-domain in the reception of every Beacon if user Enables  */
	/* Multi-domain implemented is TRUE, Multi-domain Capability will be     */
	/* disabled during join initiation If Country IE is not present in       */
	/* the Beacons */
    if(mget_MultiDomainCapabilityEnabled() == TV_FALSE)
    {
        mset_MultiDomainCapabilityEnabled(TV_TRUE);
    }

    /* Initialize with default values */
    g_bss_dscr_set[dscr_set_index].reg_domain   = bss_dscr_rd;
    g_bss_dscr_set[dscr_set_index].reg_class    = bss_dscr_rc;

    /* Update current regulatory class of the BSS if AP advertises       */
    /* Supported Operating Classes Element, If not update Regulatory     */
    /* with suitable value from the RF table.                            */
    update_bss_curr_oper_cls(msa, rx_len, index, dscr_set_index);

    /* Regulatory Class in BSS descriptor can be updated in the above    */
    /* above function, same is copied to bss_dscr_rc to further checks   */
    bss_dscr_rc = g_bss_dscr_set[dscr_set_index].reg_class;

    /* 2.4GHz Channels are defined as 1,2,3.. incremented value is '1'*/
    /* 5GHz Channels are defined as 36,40,44. incremented value is '4 */
    ch_sp    = get_ch_incr_val(freq);

   /*************************************************************************/
   /*                       Country Element                                 */
   /* --------------------------------------------------------------------- */
   /* |Element ID|Length |Country string|first ch |no.of chs|Max power  |   */
   /* --------------------------------------------------------------------- */
   /* |1         |1      |3             |1        |1        |1          |   */
   /* --------------------------------------------------------------------- */
   /* --------------------------------------------------------------------- */
   /* |first ch |no.of chs |Max power |.... |.... |....|first ch|no.of chs| */
   /* --------------------------------------------------------------------- */
   /* |    1    |     1    |    1     |.....|.....|....|  1     | 1       | */
   /* --------------------------------------------------------------------- */
   /*                                                                       */
   /*************************************************************************/

    /* Parse the frame for the Country information element */
    while(index < (rx_len - FCS_LEN))
    {
        // caisf mod 1225
        #ifdef MAC_MULTIDOMAIN
        if((ICOUNTRY == msa[index]) && (msa[index+4] != 'C' && msa[index+5] != 'N'))
        {
            ;//TROUT_PRINT("domain: if country is cn, disable multidomain.")
        }
        else
        #endif
        {
        if(ICOUNTRY == msa[index])
        {
            /* Length of the Country IE                                      */
            len        = msa[index+1];
            /* Copy Country Element */
            cnt_str[0] = msa[index+2];
            cnt_str[1] = msa[index+3];
            /* Space is saved in order to match saved format of country      */
            /* string in the RF table                                        */
            cnt_str[2] = ' ';
            cnt_str[3] = '\0';

            /* Update regulatory domain index in the BSS descriptor from     */
            /* the supported regulatory domains list by RF                   */
            bss_dscr_rd = get_reg_domain_idx(cnt_str);

            if((bss_dscr_rd == NO_REG_DOMAIN)||
               (bss_dscr_rd == INVALID_REG_DOMAIN))
            {
                /* Updation of Regulatory class in BSS descriptor is not     */
                /* required                                                  */
				g_bss_dscr_set[dscr_set_index].reg_domain = bss_dscr_rd;
                return;
            }

            /* This calculation will give us how many Regulatory triplets    */
            /* are present in country element, minus 1 is for subtracting    */
            /* country string                                                */
            num_bands = (len/3) - 1 ;

            for(cnt_bands = 0; cnt_bands < num_bands; cnt_bands++)
            {
                /* If Operation Extension ID is equal or greater than 201    */
                /* Country IE contains Regulatory triplets.                  */
                if(msa[index+5+3*cnt_bands] >= OPERATINGEXTNID)
                {
                    if((bss_dscr_rc == UNKNOWN_REG_CLASS)||
                       (bss_dscr_rc == INVALID_REG_CLASS))
                    {
                        /* Update Regulatory Class with suitable value from */
                        /* the RF table                                     */
                        if(BTRUE == is_ch_supp_in_rc_in_rd(
                                        g_bss_dscr_set[dscr_set_index].channel,
                                        bss_dscr_rd,
                                        msa[index+6+3*cnt_bands]))
                        {
                            rc_info = get_reg_class_info(freq,
                                                    msa[index+6+3*cnt_bands],
                                                    bss_dscr_rd);
                            bss_dscr_rc = msa[index+6+3*cnt_bands];

                        }
                        else
                        {
                            /* Invalid regulatory class if Operating Channel */
                            /* is not supported in any operating class       */
                            /* of RF table                                   */
                            bss_dscr_rc = INVALID_REG_CLASS;
                        }
                    }

                }
                else
                {
                    if(max_rc_tx_pow_updated == BFALSE)
                    {
                        /* 5+3*cnt_bands gives starting channel of the band      */
                        /* 6+3*cnt_bands gives number of channels in the band    */
                        /* 7+3*cnt_bands gives Regulatory max power of that band */
                        for(cnt_chan = 0;cnt_chan < msa[index+6+3*cnt_bands];cnt_chan++)
                        {
                            if(g_bss_dscr_set[dscr_set_index].channel ==
                              (msa[index+5+3*cnt_bands] + ch_sp*cnt_chan))
                            {
                                max_reg_tx_pow = msa[index+7+3*cnt_bands];

                                if((bss_dscr_rc != UNKNOWN_REG_CLASS) &&
                                   (bss_dscr_rc != INVALID_REG_CLASS))
                               {
                                   max_rc_tx_pow_updated = BTRUE;
                               }

                                break;
                            }
                        }
                    }
                }
            }

            g_bss_dscr_set[dscr_set_index].reg_domain     = bss_dscr_rd;
            g_bss_dscr_set[dscr_set_index].reg_class      = bss_dscr_rc;
            g_bss_dscr_set[dscr_set_index].max_reg_tx_pow = max_reg_tx_pow;
        }
        }

        index += (IE_HDR_LEN + msa[index + 1]);
    }

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_bss_reg_domain_cap_info                            */
/*                                                                           */
/*  Description   : This fuction checks BSS Capability and returns TRUE      */
/*                  if matches with STA's Capability                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the BSS Descriptor                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function is called during JOIN initiation to check  */
/*                  Multi-domain capability. Join request is failed if       */
/*                  Regulatory Domain and Regulatory Class are not           */
/*                  supported by STA. Disables Multi-domain capability if AP */
/*                  AP does not support Multi-domain capability.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : BTRUE: Check Pass                                        */
/*                  BFALSE: Check Fail                                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_bss_reg_domain_cap_info(bss_dscr_t *bss_dscr)
{
    UWORD8 reg_class          = UNKNOWN_REG_CLASS;
    UWORD8 bss_dscr_rd        = bss_dscr->reg_domain;
    UWORD8 bss_dscr_rc        = bss_dscr->reg_class;
    UWORD8 freq               = get_current_start_freq();
    UWORD8 ch_idx             = 0;
    UWORD8 sec_ch_offset      = bss_dscr->sec_ch_offset;
    reg_class_info_t *rc_info = NULL;
    reg_class_info_t *temp_rc_info = NULL;


    /* Check pass: if Multi-domain capability is not enabled by User */
    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return BTRUE;
    }

    /* Disable Multi-domain capability if AP does not support or does not    */
    /* include country information element                                   */
    if(bss_dscr_rd == NO_REG_DOMAIN)
    {
        mset_MultiDomainCapabilityEnabled(TV_FALSE);
        set_current_reg_domain(NO_REG_DOMAIN);
        set_current_reg_class(UNKNOWN_REG_CLASS);

        PRINTD("MultiDomain Capability is Disabled due to AP doesnot transmit Country IE\n\r");
        return BTRUE;
    }

    ch_idx = get_ch_idx_from_num(freq, bss_dscr->channel);

    if((is_reg_domain_valid(bss_dscr_rd)) &&
       (bss_dscr_rc != INVALID_REG_CLASS))
    {
        /* Set current regulatory domain before joining */
        set_current_reg_domain(bss_dscr_rd);

        /* If Regulatory Class is unknown, initiate join request with        */
        /* suitable Regulatory Class from RF table                           */
        if(bss_dscr_rc == UNKNOWN_REG_CLASS)
        {
            reg_class = get_sup_rc_of_chan_in_reg_dom(ch_idx, bss_dscr_rd,
            										  sec_ch_offset, bss_dscr);

            /* This check is useful to join Network if AP is advertising 40MHz */
            /* operation for the combination of Primary and Secondary Channel  */
            /* which is not allowed in the Current Regulatory Domain */
            if((sec_ch_offset != SCN) && (reg_class == UNKNOWN_REG_CLASS))
            {
            	reg_class = get_sup_rc_of_chan_in_reg_dom(ch_idx, bss_dscr_rd,
            										  		SCN, bss_dscr);
			}
        }
        else
        {
			temp_rc_info = get_reg_class_info(freq, bss_dscr_rc, bss_dscr_rd);

            if( temp_rc_info != NULL)
            {
				if(freq != get_rc_start_freq(temp_rc_info))
				{
					PRINTD("Warning:MultiDomain:SCAN COMPARE Failed due to unsupported regulatory class in the current frequency band\n\r");
					return BFALSE;
				}

				if(BFALSE == is_ch_supp_in_rc_in_rd(bss_dscr->channel, bss_dscr_rd, bss_dscr_rc))
				{
					PRINTD("Warning:MultiDomain:SCAN COMPARE Failed due to unsupported channel in advertised regulatory class\n\r");
					return BFALSE;
				}

                reg_class = bss_dscr_rc;
            }
            else
            {
                PRINTD("Warning:MultiDomain:SCAN COMPARE Failed due to unsupported regulatory class\n\r");
                return BFALSE;
            }

        }

        /* Update Regulatory Domain Max power                         */
        update_rc_max_tx_pow(reg_class, bss_dscr->max_reg_tx_pow);
        /* Set current Regulatory Class */
        set_current_reg_class(reg_class);
    }
    else
    {
        if(bss_dscr_rd == INVALID_REG_DOMAIN)
        {
            PRINTD("MultiDomain:SCAN COMPARE Failed due to Unsupported Regulatory Domain\n\r");
        }
        else if(bss_dscr_rc == INVALID_REG_CLASS)
        {
            PRINTD("MultiDomain:SCAN COMPARE Failed due to Invalid Regulatory Class\n\r");
        }
        else
        {
            PRINTD("MultiDomain:SCAN COMPARE Failed due to Bug in FW\n\r");
        }

        return BFALSE;
    }

    rc_info = get_reg_class_info(freq, get_current_reg_class(), bss_dscr_rd);

    if(rc_info == NULL)
    {
        PRINTD("MultiDomain:SCAN COMPARE Failed due to rc_info is NULL\n\r");
        return BFALSE;
    }

    /* Check fali if Regulatory max power is less than min RF supported power */
    if(bss_dscr->max_reg_tx_pow < get_rc_min_rf_tx_pow(rc_info))
    {
        PRINTD("MultiDomain:SCAN COMPARE Failed due to Tx Power Check Fail\n\r");
        return BFALSE;
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_11d_reg_max_tx_pow                                */
/*                                                                           */
/*  Description   : This function is called to update  Regulatory Max powers */
/*                  during joining process.                                  */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming message                       */
/*                  2) Index of the information elements                     */
/*                  3) Length of the frame                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :This function is called to update Regulatory Max powers   */
/*                 of the sub bands during the reception of Country Element  */
/*                 in the Beacon frames.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_11d_reg_max_tx_pow(UWORD8 *msa, UWORD16 rx_len, UWORD16 index)
{
    UWORD8 len                = 0;
    UWORD8 num_bands          = 0;
    UWORD8 cnt_bands          = 0;
    UWORD8 cnt_chan           = 0;
    UWORD8 cur_chan           = 0;
    UWORD8 ch_sp              = 0;
    UWORD8 *country_elem      = NULL;
    UWORD8 reg_class          = 0;
    UWORD8 coverage_class     = 0;
    UWORD8 rc_max_tx_pow      = 0;
    BOOL_T rc_triplet_present = BFALSE;
    UWORD8 freq = get_current_start_freq();

    cur_chan = get_ch_num_from_idx(freq, mget_CurrentChannel());
    ch_sp    = get_ch_incr_val(freq);


    /*************************************************************************/
    /*                       Country Element                                 */
    /* --------------------------------------------------------------------- */
    /* |Element ID|Length |Country string|first ch |no.of chs|Max power  |   */
    /* --------------------------------------------------------------------- */
    /* |1         |1      |3             |1        |1        |1          |   */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* |first ch |no.of chs |Max power |.... |.... |....|first ch|no.of chs| */
    /* --------------------------------------------------------------------- */
    /* |    1    |     1    |    1     |.....|.....|....|  1     | 1       | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Search country element */
    while(index < (rx_len - FCS_LEN))
    {
        if(ICOUNTRY == msa[index])
        {
            country_elem = &msa[index];
            break;
        }
        else
        {
            index += (IE_HDR_LEN + msa[index + 1]);
        }
    }

    if(country_elem != NULL)
    {
        /* msa[idex+1] contains length of the country element */
        len = msa[index+1];

        /* This calculation will give us how many Regulatory triplets    */
        /* are present in country element, minus 1 is for subtracting    */
        /* country string                                                */
        num_bands = (len/3) - 1 ;

        for(cnt_bands = 0; cnt_bands < num_bands; cnt_bands++)
        {

            if(msa[index+5+3*cnt_bands] >= OPERATINGEXTNID)
            {
                rc_triplet_present   = BTRUE;
                reg_class      = msa[index+6+3*cnt_bands];
                coverage_class = msa[index+7+3*cnt_bands];
                rc_max_tx_pow  = msa[index+10+3*cnt_bands];

                /* Update regulatory Class Max Tx power           */
                update_rc_max_tx_pow(reg_class, rc_max_tx_pow);
                /* Update Regulatory Class Coverage Class         */
                update_rc_coverage_class(reg_class, coverage_class);
            }

            if(rc_triplet_present == BFALSE)
            {
                /* 5+3*cnt_bands gives starting channel of the band      */
                /* 6+3*cnt_bands gives number of channels in the band    */
                /* 7+3*cnt_bands gives Regulatory max power of that band */
                for(cnt_chan = 0;cnt_chan < msa[index+6+3*cnt_bands];cnt_chan++)
                {
                    /* Check if Operating Channel is the part of sub band    */
                    /* and update Max power of the Operating Channel         */
                    if(cur_chan == (msa[index+5+3*cnt_bands] + ch_sp*cnt_chan))
                    {
                        update_rd_max_tx_pow_subbands(cur_chan,
                                                      msa[index+7+3*cnt_bands]);
                        break;
                    }
                }

                /* Update the Max powers of the sub bands */
                update_rd_max_tx_pow_subbands(msa[index+5+3*cnt_bands],
                                              msa[index+7+3*cnt_bands]);
            }

        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : adjust_bss_width_in_new_rc                               */
/*                                                                           */
/*  Description   : This function is called to adjust BSS width after channel*/
/*                  switching                                                */
/*                                                                           */
/*  Inputs        : 1) Regulatory Class                                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Adjust BSS channel width to SCN if the new channel in    */
/*                  current Regulatory Class does not allow 40MHz operation  */
/*                  (or particular SCA or SCB)                               */
/*                                                                           */
/*  Outputs       : Adjust Channel width to SCN if new channel does not allow*/
/*                  40MHz operation in the current Regulatory Class.         */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void adjust_bss_width_in_new_rc(UWORD8 reg_class)
{
    //UWORD8 bss_width          = 0;
    //reg_class_info_t *rc_info = NULL;
    //UWORD8 freq               = get_current_start_freq();

    /* Proceed if Multi-domain capability is enabled */
    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return;
    }


}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : adopt_new_rc                                             */
/*                                                                           */
/*  Description   : This function is called to adopt new Regulatory Class    */
/*                  If New Regulatory Class is INVALID then adopt suitable   */
/*                  Regulatory Class for the combination of Channel and      */
/*                  Secondary Channel Offset                                 */
/*                                                                           */
/*  Inputs        : 1) Regulatory Class                                      */
/*                  2) Channel Index                                         */
/*                  3) Secondary Channel Offset                              */
/*                                                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Adopt Regulatory class if it is valid. Otherwise select  */
/*                  suitable Regulatory Class for the given combination of   */
/*                  Channel and Secondary Channel Offset                     */
/*                                                                           */
/*  Outputs       : Regulatory Class change                                  */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void adopt_new_rc(UWORD8 reg_class, UWORD8 ch_idx, UWORD8 sec_ch_offset)
{
    reg_class_info_t *rc_info = NULL;
    UWORD8 freq = get_current_start_freq();
    UWORD8 reg_domain = get_current_reg_domain();

    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return;
    }

    if(reg_class == INVALID_REG_CLASS)
    {
		reg_class = get_sup_rc_of_chan_in_reg_dom(ch_idx, reg_domain,
		                                          sec_ch_offset, NULL);
	}

    rc_info   = get_reg_class_info(freq, reg_class, reg_domain);

    if(rc_info == NULL)
    {
        PRINTD("Warning:MultiDomain: NULL Reg Class Info while adopt new regulatory class\n\r");
        return;
    }

    set_current_reg_class(reg_class);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_40mhz_op_to_be_enabled                                */
/*                                                                           */
/*  Description   : This function is called to predict 40MHz operation is    */
/*                  enabled after join initiation. This function is required */
/*                  to predict 40MHz operation is enabled or not to select   */
/*                  suitable Regulatory Class                                */
/*                                                                           */
/*  Inputs        : 1) BSS Descriptor                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Returns TRUE if 40MHz is implemented and HT enabled on   */
/*                  both STA and AP, and Supported Channel Width is 1 in AP  */
/*                  Capabilities.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : BTRUE: If 2040 is to be enabled.                         */
/*                  BFALSE: If 2040 is not to be enabled.                    */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_40mhz_op_to_be_enabled(bss_dscr_t *bss_dscr)
{
	BOOL_T ret_val = BFALSE;

	if((mget_FortyMHzOperationImplemented() == TV_TRUE) &&
	   (get_ht_enable() == 1) &&
	   (bss_dscr->supp_chwidth == 1) &&
	   (bss_dscr->ht_capable == 1))
	{
		ret_val = BTRUE;
	}
	else
	{
		ret_val = BFALSE;
	}

	return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_sup_rc_of_chan_in_reg_dom                            */
/*                                                                           */
/*  Description   : This function gets the suitable Regulatory Class value   */
/*                  for the given channel and  Regulatory domain and also    */
/*                  checks for 40MHz operation (SCA and SCB conditions)      */
/*                                                                           */
/*  Inputs        : 1) Channel Index                                         */
/*                  2) Regulatory Domain                                     */
/*                  3) Secondary Channel Offset                              */
/*                  4) BSS Descriptor                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This Funtion returns suitable Regulatory Class for the   */
/*                  Combination of Channel index and Secondary Channel Offset*/
/*                  in the given Regulatory Domain							 */
/*                                                                           */
/*  Outputs       : None                                  					 */
/*  Returns       : Suitable Regulatory Class                                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_sup_rc_of_chan_in_reg_dom(UWORD8 ch_idx, UWORD8 reg_domain,
                                            UWORD8 sec_ch_offset,
                                            bss_dscr_t *bss_dscr)
{
    UWORD8 num_rc    = 0;
    UWORD8 rc_idx    = 0;
    UWORD8 reg_class = UNKNOWN_REG_CLASS;
    reg_class_info_t  *rc_info = NULL;
    reg_domain_info_t *rd_info = NULL;
    BOOL_T forty_Mhz_op_enabled = BFALSE;

    if(bss_dscr == NULL)
    {
		forty_Mhz_op_enabled = mget_FortyMHzOperationEnabled();
	}
	else
	{
		forty_Mhz_op_enabled = is_40mhz_op_to_be_enabled(bss_dscr);
	}

    rd_info  = get_reg_domain_info(reg_domain);
    num_rc   = get_num_reg_class(rd_info);

    for(rc_idx = 0; rc_idx < num_rc; rc_idx++)
    {
        rc_info = rd_info->reg_class_info[rc_idx];

        if(is_ch_supp_in_rc(rc_info, get_current_start_freq(), ch_idx) == BTRUE)
        {
            if((BTRUE == forty_Mhz_op_enabled) && (sec_ch_offset != SCN))
            {
                if(sec_ch_offset == SCA)
                {
                    if(BTRUE == is_rc_sca_req(rc_info))
                    {
                        reg_class = get_reg_class_val(rc_idx, reg_domain);
                        break;
                    }
                }
                else if(sec_ch_offset == SCB)
                {
                    if(BTRUE == is_rc_scb_req(rc_info))
                    {
                        reg_class = get_reg_class_val(rc_idx, reg_domain);
                        break;
                    }
                }
            }
            else
            {
                reg_class = get_reg_class_val(rc_idx, reg_domain);
                break;
            }
        }
    }

    return reg_class;
}

#endif /* MAC_MULTIDOMAIN */
#endif /* IBSS_BSS_STATION_MODE */

