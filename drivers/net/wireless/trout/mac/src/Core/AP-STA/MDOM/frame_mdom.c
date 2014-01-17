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
/*  File Name         : frame_mdom.c                                         */
/*                                                                           */
/*  Description       : This file contains the functions for setting and     */
/*                      preparing various MAC frames fields for Multi-domain */
/*                      operation.                                           */
/*                                                                           */
/*  List of Functions : set_sup_op_classes_ie                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_MULTIDOMAIN

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "frame.h"
#include "reg_domains.h"
#include "mib.h"
#include "mib_mdom.h"
#include "prot_if.h"

static void set_subband_triplet(UWORD8* data, UWORD16* index, UWORD8 first_ch_num,
                         UWORD8* num_subband_ch, UWORD8 max_reg_tx_pow);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_sup_op_classes_ie                                    */
/*                                                                           */
/*  Description   : This function is called to set the Supported Operating   */
/*                  Classes Information Element. This includes all the       */
/*                  supported operating classes that are valid in the current*/
/*                  Frequency band                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx frame                               */
/*                  2) Index of the information element                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function is called to set the Supported Operating   */
/*                  Classes Information Element.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Length                                                   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_sup_op_classes_ie(UWORD8* data, UWORD16 index)
{
    UWORD8 len         = 0;
    UWORD8 rc_idx      = 0;
    UWORD8 num_rc      = 0;
    UWORD8 index_len   = 0;
    UWORD8 num_rc_freq = 0;
    UWORD8 curr_freq   = 0;
    UWORD8 reg_class   = 0;
    UWORD8 reg_domain  = get_current_reg_domain();
    reg_domain_info_t  *rd_info = NULL;
    reg_class_info_t   *rc_info = NULL;

    if(mget_ExtendedChannelSwitchActivated() == TV_FALSE)
    {
        return len;
    }

    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return len;
    }

    rd_info = get_reg_domain_info(reg_domain);

    if((reg_domain == NO_REG_DOMAIN)||
       (reg_domain == INVALID_REG_DOMAIN))
    {
        return len;
    }

    curr_freq  = get_current_start_freq();
    num_rc     = get_num_reg_class(rd_info);
    reg_class  = get_reg_class_from_ch_idx(curr_freq, mget_CurrentChannel());

    /*************************************************************************/
    /*                  Supported Operating Classes Element                  */
    /* --------------------------------------------------------------------- */
    /* |Element ID|Length |Current Operating Class|Operating Classes       | */
    /* --------------------------------------------------------------------- */
    /* |1         |1      |1                      |Length - 1              | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set Supported Operating Classes element ID */
    data[index++] = ISUPOPCLASS;

    /* Store length index          */
    index_len     = index++;

    /* Set current Regulatory Class */
    data[index++] = reg_class;

    for(rc_idx = 0;rc_idx < num_rc; rc_idx++)
    {
        rc_info = rd_info->reg_class_info[rc_idx];

        if(curr_freq == get_rc_start_freq(rc_info))
        {
            num_rc_freq++;
            data[index++] =  get_reg_class_val(rc_idx, reg_domain);
        }
    }

    /* Set Length field of Supported Operating Classes IE */
    data[index_len] = num_rc_freq + 1;

    /* Total length of the IE */
    len = num_rc_freq + IE_HDR_LEN + 1;

    return len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_country_info_elem_frm_reg_tbl                        */
/*                                                                           */
/*  Description   : This function is called to add country information       */
/*                  element                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx frame                               */
/*                  2) Index of the information element                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds Country Information Element with sub  */
/*                  band triplets following Regulatory triplet if            */
/*                  OperatingClassesRequired is true. Regulatory triplets are*/
/*                  not added if OperatingClassesRequired is false           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_country_info_elem_frm_reg_tbl(UWORD8* data, UWORD16 index)
{
    UWORD8 reg_domain              = 0;
    UWORD8 prev_ch_rc              = 0;
    UWORD8 curr_ch_rc              = 0;
    UWORD8 curr_freq               = 0;
    UWORD8 num_ch                  = 0;
    UWORD8 num_rc                  = 0;
    UWORD8 ch_idx                  = 0;
    UWORD8 rc_idx                  = 0;
    UWORD8 index_len               = 0;
    UWORD8 num_subband_ch          = 0;
    UWORD8 ie_len                  = 0;
    UWORD8 ch_sp                   = 0;
    UWORD8 max_reg_tx_pow          = 0;
    UWORD8 curr_ch_num             = 0;
    UWORD8 first_ch_num            = 0;
    UWORD8 next_ch_num             = 0;
    BOOL_T start_flag              = BTRUE;
    channel_info_t    *ch_info     = NULL;
    reg_domain_info_t *rd_info     = NULL;
    reg_class_info_t  *rc_info     = NULL;

    if((mget_MultiDomainOperationImplemented() == TV_FALSE) ||
       (mget_MultiDomainCapabilityEnabled()    == TV_FALSE))
    {
        return 0;
    }

    curr_freq  = get_current_start_freq();
    ch_sp      = get_ch_incr_val(curr_freq);
    ch_info    = get_ch_info_from_freq(curr_freq, &num_ch);
    reg_domain = get_current_reg_domain();
    rd_info    = get_reg_domain_info(reg_domain);

    if(reg_domain == NO_REG_DOMAIN)
    {
        /* Disable MultiDomain */
        mset_MultiDomainCapabilityEnabled(TV_FALSE);
        return 0;
    }

    if(rd_info    == NULL)
    {
		return 0;
	}

    /*************************************************************************/
    /*                       Country Element                                 */
    /* --------------------------------------------------------------------- */
    /* |Element ID|Length |Country string|first ch/Operating Extension ID  | */
    /* --------------------------------------------------------------------- */
    /* |1         |1      |3             |1                                | */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* |no.of Channels/Operating Class |Max power/Coverage Class|first ch/.. */
    /* --------------------------------------------------------------------- */
    /* |    1                          |  1                     |   .....  | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Country Element ID */
    data[index++] = ICOUNTRY;

    /* Save index for length field */
    index_len = index++;

	data[index++] = rd_info->country[0];
	data[index++] = rd_info->country[1];
	data[index++] = rd_info->country[2];

    /* Add regulatory triplet if OperatingClassesRequired is true */
    if(mget_OperatingClassesRequired() == TV_TRUE)
    {
        num_rc = get_num_reg_class(rd_info);

        for(rc_idx = 0; rc_idx < num_rc; rc_idx++)
        {
            rc_info = rd_info->reg_class_info[rc_idx];

            if(curr_freq == get_rc_start_freq(rc_info))
            {
                data[index++]   = OPERATINGEXTNID;
                data[index++]   = get_reg_class_val(rc_idx, reg_domain);
                data[index++]   = rc_info->coverage_class;

                max_reg_tx_pow  = rc_info->max_reg_tx_pow;
                num_subband_ch  = 0;
                start_flag      = BTRUE;

                /* Update sub-band triplets */
                for(ch_idx = 0; ch_idx < num_ch; ch_idx++)
                {
                    curr_ch_num = get_ch_num_from_idx(curr_freq, ch_idx);

                    if(BTRUE == is_ch_supp_in_rc(rc_info, curr_freq, ch_idx))
                    {
                        if(start_flag == BTRUE)
                        {
                            first_ch_num = curr_ch_num;
                            next_ch_num  = curr_ch_num + ch_sp;
                            start_flag   = BFALSE;
                            num_subband_ch++;
                        }
                        else
                        {
                            if(next_ch_num != curr_ch_num)
                            {
                                set_subband_triplet(data, &index, first_ch_num,
                                                    &num_subband_ch, max_reg_tx_pow);

                                first_ch_num = curr_ch_num;
                                next_ch_num  =  curr_ch_num + ch_sp;
                                num_subband_ch++;
                            }
                            else
                            {
                                next_ch_num  =  curr_ch_num + ch_sp;
                                num_subband_ch++;
                            }

                        }

						if((ch_idx+1) == num_ch)
						{
							/* This check is required to handle if index */
							/* reaches end of the channel table          */
							set_subband_triplet(data, &index, first_ch_num,
												&num_subband_ch, max_reg_tx_pow);
						}

                    }
                    else
                    {
                        if(start_flag == BFALSE)
                        {
                            set_subband_triplet(data, &index, first_ch_num,
                                                &num_subband_ch, max_reg_tx_pow);
                            start_flag    = BTRUE;
                        }

                    }

                }

            }

        }

    }
    else
    {
        for(ch_idx = 0; ch_idx < num_ch; ch_idx++)
        {
            curr_ch_rc   = ch_info[ch_idx].reg_class;
            curr_ch_num  = get_ch_num_from_idx(curr_freq, ch_idx);

            if(start_flag == BTRUE)
            {
                /* Starting channel */
                if((curr_ch_rc != NOT_SUPPORTED_BY_RF) &&
                   (curr_ch_rc != INVALID_REG_CLASS))
                {
                    first_ch_num  = ch_info[ch_idx].ch_num;
                    start_flag    = BFALSE;
                    prev_ch_rc    = curr_ch_rc;
                    next_ch_num   = curr_ch_num + ch_sp;
                    num_subband_ch++;
                }
            }
            else
            {
                /* Check to handle gap in the supported channels to create */
                /* separate sub band */
                rc_info        = get_reg_class_info(curr_freq, prev_ch_rc, reg_domain);
                max_reg_tx_pow = rc_info->max_reg_tx_pow;

                if((curr_ch_rc == NOT_SUPPORTED_BY_RF) ||
                   (curr_ch_rc == INVALID_REG_CLASS))
                {

                    set_subband_triplet(data, &index, first_ch_num,
                                        &num_subband_ch, max_reg_tx_pow);

                    start_flag    = BTRUE;
                }
                else if(next_ch_num != curr_ch_num)
                {
                    set_subband_triplet(data, &index, first_ch_num,
                                        &num_subband_ch, max_reg_tx_pow);

                    first_ch_num = curr_ch_num;
                    prev_ch_rc   = curr_ch_rc;
                    next_ch_num  = curr_ch_num + ch_sp;
                    num_subband_ch++;
                }
                else
                {
                    if(prev_ch_rc == curr_ch_rc)
                    {
                        num_subband_ch++;
                    }
                    else
                    {
                        /* Check to handle Regulatory Class change in the  */
                        /* supported channels to create separate sub band  */
                        set_subband_triplet(data, &index, first_ch_num,
                                            &num_subband_ch, max_reg_tx_pow);

                        first_ch_num  = curr_ch_num;
                        num_subband_ch++;
                    }

					next_ch_num   = curr_ch_num + ch_sp;
                    prev_ch_rc    = curr_ch_rc;

                }
            }

			if(((ch_idx+1) == num_ch) && (start_flag == BFALSE))
			{
				/* This check is required to handle if index */
				/* reaches end of the channel table          */
				set_subband_triplet(data, &index, first_ch_num,
									&num_subband_ch, max_reg_tx_pow);
			}

        }

    }

    /* Update the length field */
    ie_len = (index - 1 - index_len);

    /* If IE length is not divisible by 2 then add 1byte zero Padding */
    if((ie_len % 2) == 1)
    {
        data[index] = 0;
        ie_len += 1;
    }

    data[index_len] = ie_len;
    return (IE_HDR_LEN + ie_len);

}

static void set_subband_triplet(UWORD8* data, UWORD16* index, UWORD8 first_ch_num,
                         UWORD8* num_subband_ch, UWORD8 max_reg_tx_pow)
{
    data[(*index)++] = first_ch_num;
    data[(*index)++] = *num_subband_ch;
    data[(*index)++] = max_reg_tx_pow;

    *num_subband_ch = 0;
}


#endif /* MAC_MULTIDOMAIN */
