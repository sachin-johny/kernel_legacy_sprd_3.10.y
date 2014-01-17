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
/*  File Name         : phy_802_11n.c                                        */
/*                                                                           */
/*  Description       : This file contains the functions for 802.11n PHY.    */
/*                                                                           */
/*  List of Functions : init_phy_chars                                       */
/*                      init_phy_802_11n_curr_rates                          */
/*                      is_rate_enabled                                      */
/*                      add_mac_rate                                         */
/*                      check_capability_phy_802_11n                         */
/*                      set_capability_phy_802_11n                           */
/*                      set_phy_params_802_11n                               */
/*                      set_erp_info_field_802_11n                           */
/*                      get_tx_pr_phy_802_11n                                */
/*                      get_current_channel_802_11n                          */
/*                      get_phy_tx_mode_802_11n                              */
/*                      init_ar_table_phy_802_11n                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef PHY_802_11n

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "frame.h"
#include "iconfig.h"
#include "management.h"
#include "mib.h"
#include "core_mode_if.h"
#include "cglobals.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_11n_op_type          = NTYPE_MIXED;
UWORD8 g_11g_op_mode          = G_MIXED_11B_1_MODE;
UWORD8 g_prot_mode            = NO_PROT;
UWORD8 g_erp_prot_type        = G_SELF_CTS_PROT;
UWORD8 g_ht_prot_type         = 0;
UWORD8 g_11n_obss_detection   = 0;
UWORD8 g_11n_rifs_prot_enable = 0;
UWORD8 g_11n_rifs_prot_type   = 0;
/*junbin.wang modify for cr 229369.set preamble to auto mode. 2013-11-13.*/
UWORD8 g_pr_phy_802_11n       = 2;//1;
UWORD8 g_pr_flag_phy_802_11n  = 0;
UWORD8 g_curr_rate_idx        = 0;
UWORD8 g_tx_mcs_phy_802_11n   = 0;
UWORD8 g_tx_bw_phy_802_11n    = CURR_TX_BW_AUTO;
// 20120830 caisf add, merged ittiam mac v1.3 code
UWORD32 g_tx_ant_set_phy_802_11n  = CURR_ANT_SET_INIT_VALUE;

BOOL_T g_autoprot_enable      = BTRUE;
UWORD8 g_ar_table_init_done   = 0;
phy_char_t PHY_C = {0};

/* All Rates                                           */
const data_rate_t g_rates_phy_802_11g[NUM_DR_PHY_802_11G] =
{
    {0x02, 0x04, 1},  /* 1 Mbps   */
    {0x04, 0x01, 2},  /* 2 Mbps   */
    {0x0B, 0x02, 5},  /* 5.5 Mbps */
    {0x0C, 0x0B, 6},  /* 6 Mbps   */
    {0x12, 0x0F, 9},  /* 9 Mbps   */
    {0x16, 0x03, 11}, /* 11 Mbps  */
    {0x18, 0x0A, 12}, /* 12 Mbps  */
    {0x24, 0x0E, 18}, /* 18 Mbps  */
    {0x30, 0x09, 24}, /* 24 Mbps  */
    {0x48, 0x0D, 36}, /* 36 Mbps  */
    {0x60, 0x08, 48}, /* 48 Mbps  */
    {0x6C, 0x0C, 54}  /* 54 Mbps  */
};

/* Current data rate structure */
curr_data_rate_t g_curr_rate_struct =
{
    NUM_DR_PHY_802_11G,       /* Num Rates           */
    NUM_BR_PHY_802_11G_11B_1, /* Num Basic Rates     */
    NUM_NBR_PHY_802_11G_11B_1,/* Num Non-Basic Rates */
    11,                       /* Max Basic Rates     */
    1,                        /* Min Basic Rates     */
    {
     {0x82, 0x04, 1},  /* 1 Mbps   */
     {0x84, 0x01, 2},  /* 2 Mbps   */
     {0x8B, 0x02, 5},  /* 5.5 Mbps */
     {0x0C, 0x0B, 6},  /* 6 Mbps   */
     {0x12, 0x0F, 9},  /* 9 Mbps   */
     {0x96, 0x03, 11}, /* 11 Mbps  */
     {0x18, 0x0A, 12}, /* 12 Mbps  */
     {0x24, 0x0E, 18}, /* 18 Mbps  */
     {0x30, 0x09, 24}, /* 24 Mbps  */
     {0x48, 0x0D, 36}, /* 36 Mbps  */
     {0x60, 0x08, 48}, /* 48 Mbps  */
     {0x6C, 0x0C, 54}  /* 54 Mbps  */
    }
};

/* Table containing the mapping of MCS to PHY-Rate in Mbps. Index i contains */
/* the PHY-rate in Mbps (without Short-GI) for MCS i.                        */
const UWORD8 g_mcs_user_rate[MAX_NUM_MCS_SUPPORTED] = {
    6,     /* MCS-0  */
    13,    /* MCS-1  */
    19,    /* MCS-2  */
    26,    /* MCS-3  */
    39,    /* MCS-4  */
    52,    /* MCS-5  */
    58,    /* MCS-6  */
    65,    /* MCS-7  */
    13,    /* MCS-8  */
    26,    /* MCS-9  */
    39,    /* MCS-10 */
    52,    /* MCS-11 */
    78,    /* MCS-12 */
    104,   /* MCS-13 */
    117,   /* MCS-14 */
    130    /* MCS-15 */
};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_phy_802_11n_curr_rates                              */
/*                                                                           */
/*  Description   : This function initializes the PHY rate table             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : g_curr_rate_struct                                       */
/*                                                                           */
/*  Processing    : Depending on Operating mode, Copies the relavent rates   */
/*                  from original constant strucuture to current struct      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_phy_802_11n_curr_rates(void)
{
    UWORD8 count                 = 0;
    UWORD8 num_copied            = 0;
    const data_rate_t *orig_rate = NULL;
    data_rate_t *curr_rate       = NULL;
    UWORD8 tx_rate               = 0xFF;

	TROUT_FUNC_ENTER;
    tx_rate = get_curr_tx_rate();

    /* 11g Only mode where OFDM Rates alone are switched on */
    if(get_running_mode() == G_ONLY_MODE)
    {
        /* Initialize num rate, num basic rates, num non basic rates */
        g_curr_rate_struct.num_rates = NUM_DR_PHY_802_11G_ONLY;
        g_curr_rate_struct.num_br    = NUM_BR_PHY_802_11G_GONLY;
        g_curr_rate_struct.num_nbr   = NUM_NBR_PHY_802_11G_GONLY;
        g_curr_rate_struct.min_br    = 6;
        g_curr_rate_struct.max_br    = 24;

        /* Copy the relavent rates from origninal struct to current one */
        for(count = 0; count < NUM_DR_PHY_802_11G; count++)
        {
            orig_rate = &g_rates_phy_802_11g[count];
            curr_rate = &g_curr_rate_struct.rates[num_copied];

            /* Basic Rates */
            if((orig_rate->mbps == 6)  ||
               (orig_rate->mbps == 12) ||
               (orig_rate->mbps == 24))
            {
                memcpy(curr_rate,orig_rate,sizeof(data_rate_t));
                curr_rate->mac_rate |= 0x80;
                num_copied++;
            }
            /* Non-basic rates */
            else if ((orig_rate->mbps == 9)  ||
                     (orig_rate->mbps == 18) ||
                     (orig_rate->mbps == 36) ||
                     (orig_rate->mbps == 48) ||
                     (orig_rate->mbps == 54))
            {
                memcpy(curr_rate,orig_rate,sizeof(data_rate_t));
                num_copied++;
            }
            else
            {
                continue;
            }

            if(num_copied == g_curr_rate_struct.num_rates)
            {
                break;
            }
        }
    }
    else if (get_current_start_freq() == RC_START_FREQ_2)
    {
         /* 11b Only Mode */
        if(get_running_mode() == B_ONLY_MODE)
        {
            /* Initialize num rate, num basic rates, num non basic rates */
            g_curr_rate_struct.num_rates = NUM_DR_PHY_802_11B;
            g_curr_rate_struct.num_br    = NUM_BR_PHY_802_11B;
            g_curr_rate_struct.num_nbr   = NUM_NBR_PHY_802_11B;
            g_curr_rate_struct.min_br    = 1;
            g_curr_rate_struct.max_br    = 2;

            /* Copy the relavent rates from origninal struct to current one */
            for(count = 0; count < NUM_DR_PHY_802_11G; count++)
            {
                orig_rate = &g_rates_phy_802_11g[count];
                curr_rate = &g_curr_rate_struct.rates[num_copied];

                /* Basic Rates */
                if((orig_rate->mbps == 1) ||
                   (orig_rate->mbps == 2))
                {
                    memcpy(curr_rate,orig_rate,sizeof(data_rate_t));
                    curr_rate->mac_rate |= 0x80;
                    num_copied++;
                }
                /* Non Basic Rates */
                else if((orig_rate->mbps == 5) ||
                        (orig_rate->mbps == 11))
                {
                    memcpy(curr_rate,orig_rate,sizeof(data_rate_t));
                    num_copied++;
                }
                else
                {
                    continue;
                }

                if(num_copied == g_curr_rate_struct.num_rates)
                {
                    break;
                }
            }
        }
        /* G-Mixed Mode1: All Rates On but 1,2,5.5 & 11 are basic rates */
        else if(get_running_mode() == G_MIXED_11B_1_MODE)
        {
            /* Initialize num rate, num basic rates, num non basic rates */
            g_curr_rate_struct.num_rates = NUM_DR_PHY_802_11G;
            g_curr_rate_struct.num_br    = NUM_BR_PHY_802_11G_11B_1;
            g_curr_rate_struct.num_nbr   = NUM_NBR_PHY_802_11G_11B_1;
            g_curr_rate_struct.min_br    = 1;
            g_curr_rate_struct.max_br    = 11;

            /* Copy the relavent rates from origninal struct to current one */
            for(count = 0; count < NUM_DR_PHY_802_11G; count++)
            {
                orig_rate = &g_rates_phy_802_11g[count];
                curr_rate = &g_curr_rate_struct.rates[count];

                memcpy(curr_rate,orig_rate,sizeof(data_rate_t));

                /* Basic Rates */
                if((orig_rate->mbps == 1) ||
                   (orig_rate->mbps == 2) ||
                   (orig_rate->mbps == 5) ||
                   (orig_rate->mbps == 11))
                {
                    curr_rate->mac_rate |= 0x80;
                }
            }
        }
        /* G-Mixed Mode2: All Rates On but 1,2,5.5,11,6,12 & 24 are basic rates */
        else if(get_running_mode() == G_MIXED_11B_2_MODE)
        {
            /* Initialize num rate, num basic rates, num non basic rates */
            g_curr_rate_struct.num_rates = NUM_DR_PHY_802_11G;
            g_curr_rate_struct.num_br    = NUM_BR_PHY_802_11G_11B_2;
            g_curr_rate_struct.num_nbr   = NUM_NBR_PHY_802_11G_11B_2;
            g_curr_rate_struct.min_br    = 1;
            g_curr_rate_struct.max_br    = 24;

            /* Copy the relavent rates from origninal struct to current one */
            for(count = 0; count < NUM_DR_PHY_802_11G; count++)
            {
                orig_rate = &g_rates_phy_802_11g[count];
                curr_rate = &g_curr_rate_struct.rates[count];

                memcpy(curr_rate,orig_rate,sizeof(data_rate_t));

                /* Basic Rates */
                if((orig_rate->mbps == 1)  ||
                   (orig_rate->mbps == 2)  ||
                   (orig_rate->mbps == 5)  ||
                   (orig_rate->mbps == 11) ||
                   (orig_rate->mbps == 6)  ||
                   (orig_rate->mbps == 12) ||
                   (orig_rate->mbps == 24))
                {
                    curr_rate->mac_rate |= 0x80;
                }
            }
        }
    }

    /* Restore the rate only after init is done */
    set_curr_tx_rate(tx_rate);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_rate_enabled                                          */
/*                                                                           */
/*  Description   : This function returns BTRUE if the given 11g rate is     */
/*                  enabled in the current running mode                      */
/*                                                                           */
/*  Inputs        : mac rate                                                 */
/*                                                                           */
/*  Globals       : g_curr_rate_struct                                       */
/*                                                                           */
/*  Processing    : Checks against the current running mode and sees if the  */
/*                  given rate can be used. It assumes the rate entered is a */
/*                  valid 11g rate                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_rate_enabled(UWORD8 mbps)
{
   if(get_running_mode() == B_ONLY_MODE)
   {
        /* Basic Rates */
       if((mbps == 1) ||
          (mbps == 2) ||
          (mbps == 5) ||
          (mbps == 11))
         {
            return BTRUE;
         }
         else
         {
             return BFALSE;
         }
    }
    else if(get_running_mode() == G_ONLY_MODE)
    {
        if((mbps == 6)  ||
           (mbps == 12) ||
           (mbps == 24) ||
           (mbps == 9)  ||
           (mbps == 18) ||
           (mbps == 36) ||
           (mbps == 48) ||
           (mbps == 54))
        {
             return BTRUE;
        }
        else
        {
             return BFALSE;
        }
    }
    else
    {
       return BTRUE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_mac_rate                                             */
/*                                                                           */
/*  Description   : This function adds a data rate to data rate structure    */
/*                                                                           */
/*  Inputs        : mac rate                                                 */
/*                                                                           */
/*  Globals       : g_curr_rate_struct                                       */
/*                                                                           */
/*  Processing    : Finds a data rate ptr correspoding to the current rate   */
/*                  and calls insert_data_rate function for further process  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void add_mac_rate(UWORD8 mac_rate)
{
   UWORD8 count = 0;
   const data_rate_t *orig_rate = NULL;

   while(count < NUM_DR_PHY_802_11G)
   {
       orig_rate = &g_rates_phy_802_11g[count];

       if((0x7F & mac_rate) == (0x7F & orig_rate->mac_rate))
       {
          if(BTRUE == is_rate_enabled(orig_rate->mbps))
          {
             insert_data_rate(mac_rate, orig_rate);
          }
          break;
       }
       count++;
   }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_phy_chars                                        */
/*                                                                           */
/*  Description      : This function initializes the PHY characteristic      */
/*                     elements.                                             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : PHY_C                                                 */
/*                                                                           */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void init_phy_chars(void)
{
	TROUT_FUNC_ENTER;
    PHY_C.aRIFSTime                  = 2;
    PHY_C.aSlotTime                  = 9;
    PHY_C.aSIFSTime                  = 16;
    PHY_C.aCCATime                   = 4;
    PHY_C.aRxStartDelay              = 56;
    PHY_C.aRxTxTurnaroundTime        = 2;
    PHY_C.aTxPLCPDelay               = 0;
    PHY_C.aRxPLCPDelay               = 0;
    PHY_C.aRxTxSwitchTime            = 0;
    PHY_C.aTxRampOnTime              = 0;
    PHY_C.aTxRampOffTime             = 0;
    PHY_C.aTxRFDelay                 = 0;
    PHY_C.aRxRFDelay                 = 0;
    PHY_C.aAirPropagationTime        = 0;
    PHY_C.aMACProcessingDelay        = 2;
    PHY_C.aPreambleLength            = 16;
    PHY_C.aSTFOneLength              = 8;
    PHY_C.aSTFTwoLength              = 4;
    PHY_C.aLTFOneLength              = 8;
    PHY_C.aLTFTwoLength              = 4;
    PHY_C.aPLCPHeaderLength          = 4;
    PHY_C.aPLCPSigTwoLength          = 8;
    PHY_C.aPSDUMaxLength             = 65535;
    PHY_C.aPPDUMaxTime               = 10;
    PHY_C.aIUStime                   = 8;
    PHY_C.aDTT2UTTTime               = 32;
    PHY_C.aCWmax                     = 10;
    PHY_C.aMaxCSIMatricesReportDelay = 250;
    PHY_C.aCWmin = 4;
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_capability_phy_802_11n                             */
/*                                                                           */
/*  Description   : This function checks if all the PHY capabilities in the  */
/*                  given capability information field are supported.        */
/*                                                                           */
/*  Inputs        : 1) Capability information field                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if all capabilities specific to PHY */
/*                  (Short Preamble, PBCC and Channel Agility) are           */
/*                  supported.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if all capabilities match                         */
/*                  BFALSE, otherwise.                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_capability_phy_802_11n(UWORD16 cap_info)
{
    if(get_current_start_freq() != RC_START_FREQ_2)
        return BTRUE;

    /* Short Preamble */
    if((mget_ShortPreambleOptionImplemented() == TV_FALSE) &&
       ((cap_info & SHORTPREAMBLE) == SHORTPREAMBLE))
        return BFALSE;

    /* PBCC */
    if((mget_PBCCOptionImplemented() == TV_FALSE) &&
       ((cap_info & PBCC) == PBCC))
        return BFALSE;

    /* Channel Agility */
    if((mget_ChannelAgilityPresent() == TV_FALSE) &&
       ((cap_info & CHANNELAGILITY) == CHANNELAGILITY))
        return BFALSE;

#if 0
    /* DSSS-OFDM Capabilities */
    if((mget_DSSSOFDMOptionEnabled() == TV_FALSE) &&
       ((cap_info & DSSS_OFDM) == DSSS_OFDM))
        return BFALSE;
#endif

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_capability_phy_802_11n                               */
/*                                                                           */
/*  Description   : This function sets the PHY capabilities in the given     */
/*                  frame.                                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of capability information field                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the bits in the capability            */
/*                  information field, specific to PHY (Short Preamble,      */
/*                  PBCC and Channel Agility).                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_capability_phy_802_11n(UWORD8* data, UWORD16 index)
{
    if(get_current_start_freq() != RC_START_FREQ_2)
        return;

    /* The Short Preamble Bit is set if Short Preamble type is set by user   */
    /* if the same is enabled in MIB in case of AP/STA mode                  */
    if((mget_ShortPreambleOptionImplemented() == TV_TRUE) &&
       ((get_user_pr_phy_802_11n() == 0) || (get_user_pr_phy_802_11n() == 2)))
            data[index] |= SHORTPREAMBLE;

    /* PBCC */
	//chenq add for wapi 2013-02-16
	#ifdef MAC_WAPI_SUPP
	if( mget_wapi_enable() == TV_FALSE)
	{
    	if(mget_PBCCOptionImplemented() == TV_TRUE)
        	data[index] |= PBCC;
	}
	#else
		if(mget_PBCCOptionImplemented() == TV_TRUE)
        	data[index] |= PBCC;
	#endif
	
    /* Channel Agility */
	//chenq add for wapi 2013-02-16
	#ifdef MAC_WAPI_SUPP
	if( mget_wapi_enable() == TV_FALSE)
	{
    	if(mget_ChannelAgilityPresent() == TV_TRUE)
       		data[index] |= CHANNELAGILITY;
	}
	#else
		if(mget_ChannelAgilityPresent() == TV_TRUE)
       		data[index] |= CHANNELAGILITY;
	#endif

    /* The Short Slot Time Bit is set to 1 if the Option is implemented and  */
    /* enabled. The option is disabled in case a station associates with the */
    /* option disabled or not implemented.                                   */
	//chenq add for wapi 2013-02-16
	#ifdef MAC_WAPI_SUPP
	//if( mget_wapi_enable() == TV_FALSE)
	//{
    	if((mget_ShortSlotOptionEnabled() == TV_TRUE) &&
       	(mget_ShortSlotTimeOptionImplemented() == TV_TRUE))
    	{
        	data[index + 1] |= (SHORTSLOT >> 8);
    	}
	//}
	#else
		if((mget_ShortSlotOptionEnabled() == TV_TRUE) &&
       	(mget_ShortSlotTimeOptionImplemented() == TV_TRUE))
    	{
        	data[index + 1] |= (SHORTSLOT >> 8);
    	}
	#endif
	
    /* DSSS-OFDM Capabilities */
	//chenq add for wapi 2013-02-16
	#ifdef MAC_WAPI_SUPP
	if( mget_wapi_enable() == TV_FALSE)
	{
    	if(mget_DSSSOFDMOptionEnabled() == TV_TRUE)
        	data[index + 1] |= (DSSS_OFDM >> 8);
	}
	#else
		if(mget_DSSSOFDMOptionEnabled() == TV_TRUE)
        	data[index + 1] |= (DSSS_OFDM >> 8);
	#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_phy_params_802_11n                                   */
/*                                                                           */
/*  Description   : This function sets the PHY parameter set field in the    */
/*                  given frame.                                             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index of PHY parameter set information element        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the PHY parameter set information     */
/*                  field in the given frame at the given index.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of the PHY parameter set field                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_phy_params_802_11n(UWORD8* data, UWORD16 index)
{
    UWORD8 freq = get_current_start_freq();

    if(freq != RC_START_FREQ_2)
        return 0;

    /* DS Parameter Set                                                      */
    /* ------------------------------                                        */
    /* |Element ID |Length |Channel |                                        */
    /* ------------------------------                                        */
    /* |1          |1      |1       |                                        */
    /* ------------------------------                                        */
    data[index]     = IDSPARMS;
    data[index + 1] = IDSPARMS_LEN;
    data[index + 2] = get_ch_num_from_idx(freq, mget_CurrentChannel());

    return (IE_HDR_LEN + IDSPARMS_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_erp_info_field_802_11n                               */
/*                                                                           */
/*  Description   : This function sets the ERP Information Field in the      */
/*                  frame.                                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the ERP Information Field                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The ERP Information field is set in the given frame at   */
/*                  the given index as required.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 set_erp_info_field_802_11n(UWORD8* data, UWORD8 index)
{
    if(get_current_start_freq() != RC_START_FREQ_2)
        return 0;

    /*************************************************************************/
    /* ERP Information Field                                                 */
    /* --------------------------------------------------------------------- */
    /* |EID   |Len   |NonERP Present |UseProtection |PreambleMode |Reserved| */
    /* --------------------------------------------------------------------- */
    /* |B0-B7 |B0-B7 |B0             |B1            |B2           |B3-B7   | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set ERP Information element ID */
    data[index] = IERPINFO;

    /* Set the element length to 1 */
    data[index + 1] = IERPINFO_LEN;

    /* Reset the ERP information field and set the bits as required */
    data[index + 2] = 0;

    /* if configured in MIXED MODE,bit 0 will be set when non-ERP station       */
    /* present, bit 1 will be set when protection is enabled, bit 2 will be set */
    /* when 'long-preamble support only' station present                        */
    if((get_running_mode() == G_MIXED_11B_1_MODE)||
       (get_running_mode() == G_MIXED_11B_2_MODE))
    {
        if(get_num_non_erp_sta() != 0)
        {
            data[index + 2] |= BIT0;
        }
        if(get_protection() == ERP_PROT)
        {
            data[index + 2] |= BIT1;
        }
        if(get_num_sta_no_short_pream() != 0)
        {
            data[index + 2] |= BIT2;
        }
    }

    /* Return the ERP Information element length */
    return (IE_HDR_LEN + IERPINFO_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_tx_pr_phy_802_11n                                    */
/*                                                                           */
/*  Description   : This function gets the transmit preamble type for 11n.   */
/*                                                                           */
/*  Inputs        : 1) Transmit rate                                         */
/*                                                                           */
/*  Globals       : g_pr_phy_802_11n                                         */
/*                                                                           */
/*  Processing    : If there are any STAs with only Long preamble support in */
/*                  the network only Long preamble must be used. Otherwise,  */
/*                  use the preamble type set by user based on transmit rate.*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8. 1 for long preamble, 0 for short preamble.       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
/* This function returns the current preamble used for frame transmission */
UWORD8 get_tx_pr_phy_802_11n(UWORD8 mbps)
{
    if(get_current_start_freq() != RC_START_FREQ_2)
        return 0;

    /* If there are any associated STAs supporting only Long preamble always */
    /* use long preamble.                                                    */
    if(get_num_sta_no_short_pream() != 0)
        return G_LONG_PREAMBLE;

    /* Rate is 1 Mbps */
    if(mbps == 1)
        return G_LONG_PREAMBLE;
	

    /* Other rates of 11b */
    if((mbps == 2) ||
       (mbps == 5) ||
       (mbps == 11))
    {
        if(g_pr_phy_802_11n == G_AUTO_PREAMBLE)
        {
            if(g_short_preamble_enabled == BTRUE)
                return G_SHORT_PREAMBLE;
            else
                return G_LONG_PREAMBLE;
        }
        else
        {
            return g_pr_phy_802_11n;
        }
    }

    return G_SHORT_PREAMBLE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_current_channel_802_11n                              */
/*                                                                           */
/*  Description   : This function gets the current channel information from  */
/*                  the 802.11n beacon/probe response frame                  */
/*                                                                           */
/*  Inputs        : 1) Beacon/Probe Response frame                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches for the DS parameter set IE and   */
/*                  returns the current channel value                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8. current channel value                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_current_channel_802_11n(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD8 freq     = get_current_start_freq();
    UWORD8 ch_idx   = INVALID_CHANNEL;
    UWORD16 index   = 0;
    UWORD8 sub_type = get_sub_type(msa);

    /* If not beacon or probe response then return the current channel       */
    /* programmed in th RF                                                   */
    if((sub_type != PROBE_RSP) && (sub_type != BEACON))
        return get_rf_channel_num();

#ifdef MAC_802_11N
    /* Set the Start Index for Parsing the HT Operations IE in Beacon or     */
    /* Probe Response frames                                                 */
    /* MAC Header(24), Timestamp(8), Beacon Interval(2) and Capability(2)    */
    index = TAG_PARAM_OFFSET;

    while(index < (rx_len - FCS_LEN))
    {
        if(IHTOPERATION == msa[index])
        {
            /* In case HT-Operation Info-Element is found, then the Primary */
            /* Channel can be directly extracted from it. This will be more */
            /* reliable than the input passed to this function              */
            ch_idx = get_ch_idx_from_num(freq, msa[index + 2]);

             if(BTRUE == is_ch_valid(ch_idx))
             {
                return msa[index + 2];
             }
        }

        index += (IE_HDR_LEN + msa[index + 1]);
    }
#endif /* MAC_802_11N */

    if(freq == RC_START_FREQ_2)
    {
        /* Set the Start Index for Parsing the IDSPARMS IE in Beacon or Probe*/
        /* Response frames                                                   */
        /* MAC Header(24), Timestamp(8), Beacon Interval(2) and Capability(2)*/
        index = TAG_PARAM_OFFSET;

        while(index < (rx_len - FCS_LEN))
        {
            if(msa[index] == IDSPARMS)
            {
                /* Get the Channel Index corresponding to the Channel */
                /* advertised in the DS Parameter Set.                */
                ch_idx = get_ch_idx_from_num(freq, msa[index + 2]);

                 if(BTRUE == is_ch_valid(ch_idx))
                 {
                     return msa[index + 2];
                 }
            }

            /* Increment index by length information and header */
            index += (IE_HDR_LEN + msa[index + 1]);
        }
    }

    /* Return current channel programmed, if beacon/probe response frame     */
    /* does not contain the DS parameter set IE                              */
    return get_rf_channel_num();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_phy_tx_mode_802_11n                               */
/*                                                                           */
/*  Description      : This function returns the PHY Tx mode.                */
/*                                                                           */
/*  Inputs           : 1) Data rate (in mbps or MCS)                         */
/*                     2) Preamble                                           */
/*                     3) Receiver STA entry                                 */
/*                     4) Flag to indicate if PTM used for reg/dscr setting  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The various fields of the PHY Tx mode is set based on */
/*                     transmit rate, preamble and 11n operating mode.       */
/*                     For register setting this field is set to the mode    */
/*                     corresponding to the relevant data rate. For          */
/*                     descriptor setting this field is set to the HT PHY    */
/*                     Tx mode whenever HT is enabled irrespective of the    */
/*                     data rate passed. MAC H/w automatically uses correct  */
/*                     PHY Tx mode values for non-HT rates. In case of Auto  */
/*                     rate feature, whenever the rate to be used becomes a  */
/*                     HT rate, the PHY Tx mode from the decsriptor is used. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD32, PHY Tx mode according to format specified    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 get_phy_tx_mode_802_11n(UWORD8 tx_rate, UWORD8 pr, void *entry,
                                UWORD8 reg_flag)
{
    UWORD8  dr   = 0;
    UWORD8  var  = 0;
    UWORD8  channel_wid = 0;
    UWORD32 temp = 0;

    dr = get_phy_rate(tx_rate);
#if 0
    if(((reg_flag == 1) && (IS_RATE_MCS(dr) == BTRUE)) ||
       ((reg_flag == 0) && (get_ht_enable() == 1)))
#else /* 0 */
    if(IS_RATE_MCS(dr) == BTRUE)
#endif /* 0 */
    {
        UWORD8      mcs         = 0;
        ht_struct_t *ht_hdl     = NULL;

        /* b2:b0   000: 11b Short Preamble                                   */
        /*         001: 11b Long Preamble                                    */
        /*         010: 11a                                                  */
        /*         011: Reserved                                             */
        /*         100: Reserved                                             */
        /*         101: Reserved                                             */
        /*         110: HT Mixed Mode Frame                                  */
        /*         111: HT Green Field Frame                                 */
        /* Default setting 110 (HT Mixed Mode Frame)                         */
        /*                                                                   */
        /* b4:b3   00: HT_BW20-20 MHz                                        */
        /*         01: HT_BW40-40 MHz                                        */
        /*         10: HT_BW_20DL-Duplicate Legacy                           */
        /*         11: HT_BW_20DH-Duplicate HT                               */
        /* Default setting 00 (20MHz)                                        */
        /*                                                                   */
        /* b6:b5   00: 40 MHz Band                                           */
        /*         01: Upper 20 MHz in 40 MHz Band                           */
        /*         10: None                                                  */
        /*         11: Lower 20 MHz in 40 MHz Band                           */
        /* Default setting 10 (20MHz)                                        */
        /*                                                                   */
        /* b7      0: Smoothing is not recommended                           */
        /*         1: Smoothing is recommended                               */
        /* Default setting 0 (Smoothing is not recommended)                  */
        /*                                                                   */
        /* b8      0: A sounding packet                                      */
        /*         1: Not a sounding packet                                  */
        /* Default setting 1 (Not a sounding packet)                         */
        /*                                                                   */
        /* b9      0: A packet without MPDU aggregation                      */
        /*         1: A packet with MPDU aggregation                         */
        /* Default setting 0 (A packet without MPDU aggregation)             */
        /*                                                                   */
        /* b10     0: Binary Convolution code                                */
        /*         1: Low Density Parity Check code                          */
        /* Default setting 0 (Binary Convolution code)                       */
        /*                                                                   */
        /* b11     0: Short GI is not used for this packet                   */
        /*         1: Short GI is used for this packet                       */
        /* Default setting 0 (Short GI is not used for this packet)          */
        /*                                                                   */
        /* b13:b12 00: No STBC (NSS = NSTS)                                  */
        /*         01: Reserved                                              */
        /*         10: Reserved                                              */
        /*         11: Reserved                                              */
        /* Set this based on the MCS value : MCS0-7 = 1, MCS8-15 = 2,        */
        /* MCS16-23 = 3, MCS24-31 = 4                                        */
        /*                                                                   */
        /* b15:b14 Number of Extension spatial streams                       */
        /* Default setting 0 (MCS0 to 7 is basic)                            */
        /*                                                                   */
        /* b23:b16 Antenna set. This is an 8 bit set.                        */
        /* Set this based on the MCS value : MCS0-7 = 0x01, MCS8-15 = 0x03,  */
        /* MCS16-23 = 0x07, MCS24-31 = 0x0F                                  */
        /* Default setting 0x01 (MCS0 to 7 is basic)                         */
        /*                                                                   */
        /* b31:b24 Reserved. Set to zero                                     */
        /*                                                                   */
        /* Default: 0000 0000 0000 0001 0000 0001 0100 0110 (0x00010146)     */
        /*                                                                   */

        /* Initialize the PHY Tx mode to the default HT PHY Tx mode. Note    */
        /* that the default mode is valid for MCS0-7 without any optional HT */
        /* features enabled.                                                 */
        temp = DEFAULT_HT_PHY_TX_MODE;

        /*********************************************************************/
        /* Update fields based on the MCS value selected for transmission    */
        /*********************************************************************/

        /* Update the fields based on the MCS value */
        mcs = GET_MCS(dr);

        /* Antenna set (b23:b16) */
		
// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
        /* This is set based on the device configuration. It is assumed that */
        /* all TX antennas shall be used for transmission independent of MCS */
        if(MAX_MCS_SUPPORTED <= 7)
            var = 0x01;
        else if(MAX_MCS_SUPPORTED <= 15)
            var = 0x03;
        else if(MAX_MCS_SUPPORTED <= 23)
            var = 0x07;
        else
            var = 0x0F;

        temp |= ((UWORD32)(var) & 0xFF) << 16;
#else
        /* No Setting from SW needed for Antenna Set as it will be  */
        /* over-written by MAC HW, with value from register 0x658   */
#endif

        /*********************************************************************/
        /* Update the fields based on the capability of the transmitter and  */
        /* receiver.                                                         */
        /*********************************************************************/

        /* Extract the HT structure handle */
        ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

        /* If no entry is given for the receiver default settings are used   */
        /* as the receiver capabilities cannot be checked.                   */
        if(ht_hdl != NULL)
        {
            /* Greenfield preamble (b2:b0) */
			
// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
            /* Receiver supports reception of PPDUs with HT Greenfield format*/
            /* and Greenfield transmission is enabled                        */
            if((ht_hdl->greenfield == 1) &&
               (mget_GreenfieldOptionEnabled() == TV_TRUE))
            {
                temp |= 0x07;
            }
#else
            /* Receiver supports reception of PPDUs with HT Greenfield format*/
            /* ,Greenfield transmission is enabled and also Current          */
            /*  Regulatory calss without Behaviour limit set 16              */
            if((ht_hdl->greenfield == 1) &&
               (mget_GreenfieldOptionEnabled() == TV_TRUE) &&
               (check_curr_rc_dfs_50_100us(mget_CurrentChannel())   == BFALSE))
            {
                temp |= 0x07;
            }
#endif


            /* Low Density Parity Check code (b10) */

            /* Receiver supports reception of LDPC coded packets and LDPC    */
            /* coding option is enabled for transmission                     */
            if((ht_hdl->ldpc_cod_cap == 1) &&
               (mget_LDPCCodingOptionEnabled() == TV_TRUE))
            {
               temp |= BIT10;
            }

            /* Short Guard Interval (b11) */

            /* Receiver supports Short GI for the reception of 20MHz packets */
            /* and channel width is 20MHz and Short GI in 20MHz is enabled   */
            /* for txn or the Receiver supports Short GI for the reception   */
            /* of 40MHz packets and channel width is 40MHz and Short GI in   */
            /* 40MHz is enabled for transmission                             */

            if((((ht_hdl->short_gi_20 == 1) &&
                 (channel_wid         == 0) &&
                 (mget_ShortGIOptionInTwentyEnabled() == TV_TRUE)) ||
                ((ht_hdl->short_gi_40 == 1) &&
                 (channel_wid         == 1) &&
                 (mget_ShortGIOptionInFortyEnabled() == TV_TRUE))))
            {
#ifdef AUTORATE_FEATURE
                /* If autorate is enabled, then Short-GI is used only when */
                /* enabled by the Autorate module.                         */
                if((ht_hdl->tx_sgi == BTRUE) || (is_autorate_enabled() == BFALSE))
#endif /* AUTORATE_FEATURE */
                {
                    temp |= BIT11;
                }
            }

            /* STBC (b13:b12) */

            /* Receiver supports reception of PPDUs using STBC */
            if(ht_hdl->rx_stbc)
            {
                /* STBC is enabled for transmission. This is set based on the*/
                /* MCS value:                                                */
                /* MCS0-7 = 1, MCS8-15 = 2, MCS16-23 = 3, MCS24-31 = 4       */
                if(0) /* 11N TBD - PHY supports Tx STBC: how to check this   */
                {
                    if(mcs <= 7)
                        var = 0;
                    else if(mcs <= 15)
                        var = 1;
                    else if(mcs <= 23)
                        var = 2;
                    else if(mcs <= 31)
                        var = 3;

                    if(var > ht_hdl->rx_stbc)
                        var = ht_hdl->rx_stbc;

                    temp |= ((UWORD32)(var) & 0x03) << 12;
                }
            }
        }
    }
    /* For Non-HT rates set the PHY Transmit mode based on rate and preamble */
    /* Non-HT Duplicate mode is presently not supported.                     */
    else
    {
        if(IS_RATE_11B(dr) == BTRUE)
        {
            if(pr == 0)
                temp = NONHT_11BS_PHY_TX_MODE;
            else
                temp = NONHT_11BL_PHY_TX_MODE;
        }
        else
        {
            temp = NONHT_11A_PHY_TX_MODE;
        }
    }


    return temp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : is_sec_channel_valid                                  */
/*                                                                           */
/*  Description      : This function checks the validity of the Secondary    */
/*                     Channel corresponding the specified Primary Channel   */
/*                     Index and Secondary Channel Offset.                   */
/*                                                                           */
/*  Inputs           : 1) Primary channel                                    */
/*                     2) Secondary channel offset                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function computes the Secondary Channel Index    */
/*                     using the Primary Channel Index and the Secondary     */
/*                     Channel Offset. The sanity check performed is         */
/*                     currently a trivial one with no consideration for the */
/*                     applicable Regulatory domain.                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : BTRUE  if Secondary Channel is Supported              */
/*                     BFALSE if Secondary Channel is not Supported          */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_sec_channel_valid_11n(UWORD8 pri_idx, N_SEC_CH_OFFSET_T sec_offset)
{
    UWORD8 sec_idx = INVALID_CHANNEL;
    BOOL_T retval  = BTRUE;

    sec_idx = get_sec_ch_idx_11n(pri_idx, sec_offset);

    retval = is_ch_valid(sec_idx);

    return retval;
}

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 1
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_tx_pow_11n                                        */
/*                                                                           */
/*  Description      : This function returns the appropriate TX power to be  */
/*                     used, given the data rate and PHY TX Mode.            */
/*                                                                           */
/*  Inputs           : 1) TX Data Rate (PHY Rate Format)                     */
/*                     2) PHY TX Mode                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The transmit power can be derived from the following  */
/*                     sources:                                              */
/*                     1) Default Power Settings                             */
/*                     2) User specified value depending upon TX-Format      */
/*                     1 and 2 are limited by Regulatory Max power.          */
/*                     The source of the TX-Power settings is also user      */
/*                     configurable. The value returned depends upon this    */
/*                     user configuration, the TX Data Rate and the Frame TX */
/*                     parameters (PHY-Tx-Mode)                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Tx Power                                              */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_tx_pow_11n(UWORD8 dr, UWORD8 ptm)
{
    tx_power_t *pow = 0;

    /* When user control is set to 0, power that will be returned is      */
    /* min of the default Tx Power and Reg Tx Power.                      */
    if(g_user_control_enabled == 0)
    {
        if(IS_OFDM_RATE(dr) == BFALSE)
        {
            pow = min_of_tx_pw(&g_default_tx_power.pow_b,
                               &g_reg_max_tx_power.pow_b);
        }
        else
        {
            if(IS_RATE_MCS(dr) == BTRUE)
            {
                pow = min_of_tx_pw(&g_default_tx_power.pow_n,
                                   &g_reg_max_tx_power.pow_n);

            }
            else
            {
                pow = min_of_tx_pw(&g_default_tx_power.pow_a,
                                   &g_reg_max_tx_power.pow_a);
            }
        }
    }
    /* When user control is set to 1, power that will be returned is       */
    /* min of the default Tx Power, Reg Tx Power and User configured Power */
    else
    {
        if(IS_OFDM_RATE(dr) == BFALSE)
        {
            pow = min_of_tx_pw(&g_user_tx_power.pow_b,
                               &g_reg_max_tx_power.pow_b);
        }
        else
        {
            if(IS_RATE_MCS(dr) == BTRUE)
            {
                pow = min_of_tx_pw(&g_user_tx_power.pow_n,
                                   &g_reg_max_tx_power.pow_n);

            }
            else
            {
                pow = min_of_tx_pw(&g_user_tx_power.pow_a,
                                   &g_reg_max_tx_power.pow_a);
            }
        }
    }

    return pow->gaincode;
}
#endif

#ifdef AUTORATE_FEATURE
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_ar_table_phy_802_11n                             */
/*                                                                           */
/*  Description      : This function initializes the global auto rate table. */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function initializes the global auto rate table  */
/*                     with default values for 11n mode. This table can be   */
/*                     overwritten by the user configuration.                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void init_ar_table_phy_802_11n(void)
{
    UWORD8 i = 0;

    if(g_ar_table_init_done == 1)
        return;
    g_ar_table_init_done = 1;

#ifdef NMAC_1X1_MODE
#if 0 // fix error according to curr_data_rate_t g_curr_rate_struct's phy rate, caisf 0723
    g_ar_table[i++] = 0x00; /* NON-HT :   1.0 Mbps */
#else
    g_ar_table[i++] = 0x04; /* NON-HT :   1.0 Mbps */
#endif
    g_ar_table[i++] = 0x01; /* NON-HT :   2.0 Mbps */
    g_ar_table[i++] = 0x02; /* NON-HT :   5.5 Mbps */
    g_ar_table[i++] = 0x0B; /* NON-HT :   6.0 Mbps */
    g_ar_table[i++] = 0x80; /* MCS-0  :   6.5 Mbps */
    g_ar_table[i++] = 0x0F; /* NON-HT :   9.0 Mbps */
    g_ar_table[i++] = 0x03; /* NON-HT :  11.0 Mbps */
    g_ar_table[i++] = 0x0A; /* NON-HT :  12.0 Mbps */
    g_ar_table[i++] = 0x81; /* MCS-1  :  13.0 Mbps */
    g_ar_table[i++] = 0x0E; /* NON-HT :  18.0 Mbps */
    g_ar_table[i++] = 0x82; /* MCS-2  :  19.5 Mbps */
    g_ar_table[i++] = 0x09; /* NON-HT :  24.0 Mbps */
    g_ar_table[i++] = 0x83; /* MCS-3  :  26.0 Mbps */
    g_ar_table[i++] = 0x0D; /* NON-HT :  36.0 Mbps */
    g_ar_table[i++] = 0x84; /* MCS-4  :  39.0 Mbps */
    g_ar_table[i++] = 0x08; /* NON-HT :  48.0 Mbps */
    g_ar_table[i++] = 0x85; /* MCS-5  :  52.0 Mbps */
    g_ar_table[i++] = 0x0C; /* NON-HT :  54.0 Mbps */
    g_ar_table[i++] = 0x86; /* MCS-6  :  58.5 Mbps */
    g_ar_table[i++] = 0x87; /* MCS-7  :  65.0 Mbps */
#else /* NMAC_1X1_MODE */
#if 0 // fix error according to curr_data_rate_t g_curr_rate_struct's phy rate, caisf 0723
    g_ar_table[i++] = 0x00; /* NON-HT :   1.0 Mbps */
#else
    g_ar_table[i++] = 0x04; /* NON-HT :   1.0 Mbps */
#endif
    g_ar_table[i++] = 0x01; /* NON-HT :   2.0 Mbps */
    g_ar_table[i++] = 0x02; /* NON-HT :   5.5 Mbps */
    g_ar_table[i++] = 0x0B; /* NON-HT :   6.0 Mbps */
    g_ar_table[i++] = 0x80; /* MCS-0  :   6.5 Mbps */
    g_ar_table[i++] = 0x0F; /* NON-HT :   9.0 Mbps */
    g_ar_table[i++] = 0x03; /* NON-HT :  11.0 Mbps */
    g_ar_table[i++] = 0x0A; /* NON-HT :  12.0 Mbps */
    g_ar_table[i++] = 0x81; /* MCS-1  :  13.0 Mbps */
    g_ar_table[i++] = 0x88; /* MCS-8  :  13.0 Mbps */
    g_ar_table[i++] = 0x0E; /* NON-HT :  18.0 Mbps */
    g_ar_table[i++] = 0x82; /* MCS-2  :  19.5 Mbps */
    g_ar_table[i++] = 0x09; /* NON-HT :  24.0 Mbps */
    g_ar_table[i++] = 0x83; /* MCS-3  :  26.0 Mbps */
    g_ar_table[i++] = 0x89; /* MCS-9  :  26.0 Mbps */
    g_ar_table[i++] = 0x0D; /* NON-HT :  36.0 Mbps */
    g_ar_table[i++] = 0x84; /* MCS-4  :  39.0 Mbps */
    g_ar_table[i++] = 0x8A; /* MCS-10 :  39.0 Mbps */
    g_ar_table[i++] = 0x08; /* NON-HT :  48.0 Mbps */
    g_ar_table[i++] = 0x85; /* MCS-5  :  52.0 Mbps */
    g_ar_table[i++] = 0x8B; /* MCS-11 :  52.0 Mbps */
    g_ar_table[i++] = 0x0C; /* NON-HT :  54.0 Mbps */
    g_ar_table[i++] = 0x86; /* MCS-6  :  58.5 Mbps */
    g_ar_table[i++] = 0x87; /* MCS-7  :  65.0 Mbps */
    g_ar_table[i++] = 0x8C; /* MCS-12 :  78.0 Mbps */
    g_ar_table[i++] = 0x8D; /* MCS-13 : 104.0 Mbps */
    g_ar_table[i++] = 0x8E; /* MCS-14 : 117.0 Mbps */
    g_ar_table[i++] = 0x8F; /* MCS-15 : 130.0 Mbps */
#endif /* NMAC_1X1_MODE */

    g_ar_table_size = i;
}

#endif /* AUTORATE_FEATURE */

#endif /* PHY_802_11n */
