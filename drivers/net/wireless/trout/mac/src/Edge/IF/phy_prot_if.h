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
/*  File Name         : phy_prot_if.h                                        */
/*                                                                           */
/*  Description       : This file contains the definitions and declarations  */
/*                      for the PHY protocol interface.                      */
/*                                                                           */
/*  List of Functions : Access functions for PHY protocol interface          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_PROT_IF_H
#define PHY_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/




#ifdef PHY_802_11n
#include "phy_802_11n.h"
#include "mib_802_11n.h"
#endif /* PHY_802_11n */

#include "ch_info.h"
#include "reg_domains.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* PHY Types */
typedef enum {PHY_B_DSSS = 0,
              PHY_A_OFDM = 1,
              PHY_G      = 2,
              PHY_N      = 3,
              PHY_N_5GHZ = 4
} PHYTYPE_T;

/*****************************************************************************/
/* Extern function declarations                                              */
/*****************************************************************************/

extern void init_phy_chars(void);
extern UWORD8 get_rf_channel_num(void);
extern UWORD8 get_rf_channel_idx(void);
extern UWORD8 set_rf_channel_idx(UWORD8 ch_idx);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function returns the CW min based on the PHY in use */
INLINE UWORD8 get_cwmin(void)
{
    return PHY_C.aCWmin;
}

/* This function returns the CW max based on the PHY in use */
INLINE UWORD8 get_cwmax(void)
{
    return PHY_C.aCWmax;
}

/* This function sets the running mode for 802.11g PHY */
INLINE void set_running_mode(UWORD8 mode)
{



#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        if(mode < NUM_G_OPERATING_MODE)
            g_11g_op_mode = mode;
    }
#endif /* PHY_802_11n */
}

/* This function returns the running mode for 802.11g PHY */
INLINE UWORD8 get_running_mode(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        ret_val = G_ONLY_MODE;
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        ret_val = g_11g_op_mode;
    }
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the operating mode for 802.11n PHY */
INLINE void set_11n_op_type(UWORD8 mode)
{



#ifdef PHY_802_11n
    if(mode < NUM_N_OPERATING_TYPE)
        g_11n_op_type = mode;
#endif /* PHY_802_11n */
}

/* This function returns the operating mode for 802.11n PHY */
INLINE UWORD8 get_11n_op_type(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = g_11n_op_type;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the ERP protection type for 802.11g/n PHY */
INLINE void set_erp_prot_type(UWORD8 type)
{



#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        if(type < NUM_G_PROTECTION_MODE)
            g_erp_prot_type = type;
    }
#endif /* PHY_802_11n */
}

/* This function returns the ERP protection type for 802.11g/n PHY */
INLINE UWORD8 get_erp_prot_type(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        ret_val = g_erp_prot_type;
    }
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the HT protection type for 802.11n PHY */
INLINE void set_11n_ht_prot_type(UWORD8 type)
{



#ifdef PHY_802_11n
    if(type < NUM_N_PROTECTION_MODE)
        g_ht_prot_type = type;
#endif /* PHY_802_11n */
}

/* This function returns the ERP protection type for 802.11g/n PHY */
INLINE UWORD8 get_11n_ht_prot_type(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = g_ht_prot_type;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the protection mode for 802.11g PHY */
INLINE void set_protection(UWORD8 mode)
{



#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        /* ERP protection is not required in 5GHZ operation */
		// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
        g_prot_mode = NO_PROT;
#else
        if(ERP_PROT != mode)
        {
	        g_prot_mode = mode;
		}
#endif
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        g_prot_mode = mode;
    }
#endif /* PHY_802_11n */
}

/* This function returns the protection mode for 802.11g PHY */
INLINE UWORD8 get_protection(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = g_prot_mode;
#endif /* PHY_802_11n */

    return ret_val;
}

INLINE BOOL_T is_autoprot_enabled(void)
{
	BOOL_T ret_val = BFALSE;




#ifdef PHY_802_11n
    ret_val = g_autoprot_enable;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function enables auto rate feature */
INLINE void enable_autoprot(void)
{



#ifdef PHY_802_11n
    g_autoprot_enable = BTRUE;
    set_protection(NO_PROT);
#endif /* PHY_802_11n */
}

/* This function disables auto rate feature */
INLINE void disable_autoprot(void)
{



#ifdef PHY_802_11n
    g_autoprot_enable = BFALSE;
#endif /* PHY_802_11n */
}

/* This function sets the OBSS non-HT STA detection for 802.11n PHY */
INLINE void set_11n_obss_detection(UWORD8 det)
{



#ifdef PHY_802_11n
    /* This Comparison should be for NUM_N_OBSS_DETECTION */
    if(det < 4)
        g_11n_obss_detection = det;
#endif /* PHY_802_11n */
}

/* This function returns the OBSS non-HT STA detection for 802.11n PHY */
INLINE UWORD8 get_11n_obss_detection(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = g_11n_obss_detection;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the RIFS protection enable for 802.11n PHY */
INLINE void set_11n_rifs_prot_enable(UWORD8 prot)
{



#ifdef PHY_802_11n
    /* Input Sanity Check */
    if(prot > 1)
        return;

    g_11n_rifs_prot_enable = prot;
#endif /* PHY_802_11n */
}

/* This function returns the RIFS protection enable for 802.11n PHY */
INLINE UWORD8 get_11n_rifs_prot_enable(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = g_11n_rifs_prot_enable;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function checks the PHY capabilities */
INLINE BOOL_T check_bss_capability_phy(UWORD16 cap_info)
{
	BOOL_T ret_val = BTRUE;




#ifdef PHY_802_11n
    ret_val = check_capability_phy_802_11n(cap_info);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function sets the PHY capabilities in the given frame */
INLINE void set_capability_phy(UWORD8* data, UWORD16 index)
{
#ifdef PHY_802_11n
    set_capability_phy_802_11n(data, index);
#endif /* PHY_802_11n */
}

/* This function sets the PHY parameters in the given frame */
INLINE UWORD8 set_phy_params(UWORD8* data, UWORD16 index)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = set_phy_params_802_11n(data, index);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function initializes the MIB specific to the PHY in use */
INLINE void init_phy_mib(void)
{
#ifdef PHY_802_11n
    init_phy_802_11n_mib();
#endif /* PHY_802_11n */
}

/* This function updates the MIB specific to the PHY in use after enabling   */
/* operation. This is required for certain MIB initialization dependent on   */
/* the mode of operation.                                                    */
INLINE void update_phy_mib(UWORD8 mode)
{
#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        /* For IBSS Stations Short Slot option is disabled */
        if(mode == INDEPENDENT)
        {
            mset_ShortSlotOptionEnabled(TV_FALSE);
        }
#ifdef BURST_TX_MODE
        /* Short slot must be enabled in Burst tx mode */
        else if (g_burst_tx_mode_enabled == BTRUE)
        {
            mset_ShortSlotOptionEnabled(TV_TRUE);
        }
#endif /* BURST_TX_MODE */
        else
        {
            if(g_short_slot_allowed == BTRUE)
                mset_ShortSlotOptionEnabled(TV_TRUE);
            else
                mset_ShortSlotOptionEnabled(TV_FALSE);
        }
    }
#endif /* PHY_802_11n */

}

/* This function disables Short Slot option for 11g PHY */
INLINE void disable_short_slot(void)
{



#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        /* Disable short slot option in 2.4GHZ operation */
        mset_ShortSlotOptionEnabled(TV_FALSE);
    }
#endif /* PHY_802_11n */

}

/* This function enables Short Slot option for 11g PHY */
INLINE void enable_short_slot(void)
{



#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        /* Set short slot option in 2.4GHZ operation */
        mset_ShortSlotOptionEnabled(TV_TRUE);
    }
#endif /* PHY_802_11n */
}

/* This function returns the Short Slot option state fpr PHY in use */
INLINE UWORD8 get_short_slot_enabled(void)
{
	UWORD8 ret_val = 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        ret_val = 1;
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        if(mget_ShortSlotOptionEnabled() == TV_TRUE)
            ret_val = 1;
    }
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the current preamble used to transmit the frame */
INLINE UWORD8 get_preamble(UWORD8 tx_rate)
{
	UWORD8 ret_val = 0;

#ifdef PHY_802_11n
    ret_val = get_tx_pr_phy_802_11n(tx_rate);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the maximum basic rate supported by PHY */
INLINE UWORD8 get_max_basic_rate(void)
{
    return g_curr_rate_struct.max_br;
}

/* This function returns the minimum basic rate supported by PHY */
INLINE UWORD8 get_min_basic_rate(void)
{
#if 1 // dumy debug 0815
    return g_curr_rate_struct.min_br;
#else
	return g_curr_rate_struct.max_br;
#endif
}

/* This function returns the number of basic rates supported by PHY */
INLINE UWORD8 get_num_basic_rates(void)
{
    return g_curr_rate_struct.num_br;
}

/* This function returns the basic rate as required by MAC corresponding to  */
/* the index given by 'idx'. Eg: If idx = 0, 1st basic rate; If idx = 1,     */
/* 2nd basic rate; etc                                                       */
INLINE UWORD8 get_mac_basic_rate(UWORD8 idx)
{
    UWORD8 i = 0;
    UWORD8 j = 0;

    /* Return on finding the basic rate corresponding 'idx'. To keep a track */
    /* of the number of basic rates the variable 'j' is used.                */
    for(i = 0; i < g_curr_rate_struct.num_rates; i++)
    {
        if((g_curr_rate_struct.rates[i].mac_rate & 0x80) != 0)
        {
            if(idx == j)
                return g_curr_rate_struct.rates[i].mac_rate;

            j++;
        }
    }

    /* If an invalid value of 'idx' is passed, return 0 */
    return 0;
}

/* This function returns the number of non-basic rates supported by PHY */
INLINE UWORD8 get_num_non_basic_rates(void)
{
    return g_curr_rate_struct.num_nbr;
}

/* This function returns the non-basic rate as required by MAC corresponding */
/* to the index given by 'idx'. Eg: If idx = 0, 1st non-basic rate;          */
/* If idx = 1, 2nd non-basic rate; etc                                       */
INLINE UWORD8 get_mac_non_basic_rate(UWORD8 idx)
{
    UWORD8 i = 0;
    UWORD8 j = 0;

    /* Return on finding the non-basic rate corresponding 'idx'. To keep a   */
    /* track of the number of non-basic rates the variable 'j' is used.      */
    for(i = 0; i < g_curr_rate_struct.num_rates; i++)
    {
        if((g_curr_rate_struct.rates[i].mac_rate & 0x80) == 0)
        {
            if(idx == j)
                return g_curr_rate_struct.rates[i].mac_rate;

            j++;
        }
    }

    /* If an invalid value of 'idx' is passed, return 0 */
    return 0;
}

/* This function returns the current transmission rate of PHY */
INLINE UWORD8 get_curr_tx_rate(void)
{
    /* Use the default rate if an invalid rate is found */
    if(g_curr_rate_idx >= g_curr_rate_struct.num_rates)
    {
        g_curr_rate_idx = (g_curr_rate_struct.num_rates - 1);
    }

    return g_curr_rate_struct.rates[g_curr_rate_idx].mbps;
}

/* This function sets the current transmission rate of PHY */
INLINE void set_curr_tx_rate(UWORD8 mbps)
{
    UWORD8 i = 0;

    /* Search for the rate mbps across the current rate array */
    for(i = 0; i < g_curr_rate_struct.num_rates; i++)
    {
        if(g_curr_rate_struct.rates[i].mbps == mbps)
        {
            g_curr_rate_idx = i;
            return;
        }
    }

    /* Use the default rate if it was not found */
    g_curr_rate_idx = (g_curr_rate_struct.num_rates - 1);

    return;
}

/* This function converts the PHY-Rate input to User-Rate (in Mbps) */
INLINE UWORD8 get_pr_to_ur(UWORD8 phy_rate)
{
    UWORD8 ur = 0;

#ifdef PHY_802_11n
    ur = get_phy_802_11n_pr_to_ur(phy_rate);
#endif /* PHY_802_11n */

    return ur;
}
/* This function returns the current transmission MCS of PHY */
INLINE UWORD8 get_curr_tx_mcs(void)
{
    UWORD8 curr_tx_mcs = 0;
    
#ifdef PHY_802_11n
    curr_tx_mcs = get_phy_802_11n_tx_mcs();
#endif /* PHY_802_11n */

    return curr_tx_mcs;
}

/* This function sets the current transmission MCS of PHY */
INLINE void set_curr_tx_mcs(UWORD8 mcs)
{



#ifdef PHY_802_11n
    set_phy_802_11n_tx_mcs(mcs);
#endif /* PHY_802_11n */
}

/* This function returns the current transmission Bandwidth of PHY */
INLINE UWORD8 get_curr_tx_bw(void)
{
    UWORD8 curr_tx_bw = 0;



#ifdef PHY_802_11n
    curr_tx_bw = get_phy_802_11n_tx_bw();
#endif /* PHY_802_11n */

    return curr_tx_bw;
}

/* This function sets the current transmission MCS of PHY */
INLINE void set_curr_tx_bw(UWORD8 bw)
{



#ifdef PHY_802_11n
    set_phy_802_11n_tx_bw(bw);
#endif /* PHY_802_11n */
}

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/* This function returns the current tx antenna set  */
INLINE UWORD32 get_curr_tx_ant_set_prot(void)
{
    UWORD32 curr_tx_ant_set = 0x1;




#ifdef PHY_802_11n
    curr_tx_ant_set = get_phy_802_11n_ant_set();
#endif /* PHY_802_11n */

    return curr_tx_ant_set;
}

/* This function sets the current tx antenna set */
INLINE void set_curr_tx_ant_set_prot(UWORD32 ant_set)
{



	
#ifdef PHY_802_11n
    set_phy_802_11n_ant_set(ant_set);
#endif /* PHY_802_11n */
}
#endif

/* This function returns the user rate given the MAC rate */
INLINE UWORD8 get_user_rate(UWORD8 mac_rate)
{
    UWORD8 i = 0;

    /* Return User rate (mbps) given the MAC rate. Mask the 7th bit of the   */
    /* MAC rate during comparison.                                           */
    for(i = 0; i < g_curr_rate_struct.num_rates; i++)
    {
        if((g_curr_rate_struct.rates[i].mac_rate & 0x7F) == (mac_rate & 0x7F))
        {
            return g_curr_rate_struct.rates[i].mbps;
        }
    }

    /* If an invalid value of 'mac_rate' is passed, return 0 */
    return 0;
}

/* This function returns the PHY rate corresponding to PHY rate in mbps */
INLINE UWORD8 get_phy_rate(UWORD8 mbps)
{
    UWORD8 i = 0;
    UWORD8 rate = 0;

#ifdef PHY_802_11n
    if(IS_RATE_MCS(mbps) != 0)
        return mbps;
#endif /* PHY_802_11n */

    /* Return PHY rate on finding the rate corresponding to 'mbps' */
    for(i = 0; i < g_curr_rate_struct.num_rates; i++)
    {
        if(g_curr_rate_struct.rates[i].mbps == mbps)
        {
            rate =  g_curr_rate_struct.rates[i].phy_rate;
        }
    }

	
    /* Set the long preamble bit if needed*/
	/*junbin.wang mask for cr 229369, because data rate is short preamble in g_ar_table.2013-11-13*/
	#if 0
    if(get_preamble(mbps) == G_LONG_PREAMBLE)
	{
		rate = rate | BIT2;
	}
	#endif

    /* If an invalid value of 'mbps' is passed, return 0 */
    return rate;
}


/* This function sets the preamble to be used for frame transmission */
INLINE void set_preamble(UWORD8 pr)
{
#ifdef PHY_802_11n
    set_pr_phy_802_11n(pr, get_curr_tx_rate());
#endif /* PHY_802_11n */

}

/* This function returns the current preamble used to transmit the frame */
INLINE UWORD16 set_erp_info_field(UWORD8 *data, UWORD16 index)
{
    UWORD16 ret_val= 0;




#ifdef PHY_802_11n
    ret_val = set_erp_info_field_802_11n(data, (UWORD8)index);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the short slot time based on the PHY in use */
INLINE UWORD8 get_short_slot_time(void)
{
    UWORD8 ret_val= 0;




#ifdef PHY_802_11n
    ret_val = SHORTSLOTTIME;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the long slot time based on the PHY in use */
INLINE UWORD8 get_long_slot_time(void)
{
    UWORD8 ret_val= 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        ret_val = LONGSLOTTIME;
    }
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the SIFS time based on the PHY in use */
INLINE UWORD8 get_sifs_time(void)
{
    UWORD8 ret_val= 0;




#ifdef PHY_802_11n
    ret_val = SIFSTIME;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the EIFS time based on the PHY in use */
INLINE UWORD16 get_eifs_time(void)
{
	UWORD16 ret_val = 0;




#ifdef PHY_802_11n
    if(get_running_mode() == G_ONLY_MODE)
        ret_val = (EIFSTIME_GONLY);
    else
        ret_val = (EIFSTIME_GMIXED);

    /* 11N TBD - check if any change required  */
#endif /* PHY_802_11n */

    //ret_val = SIFSTIME;//dumy add for debug 0726

    return ret_val;
}

/* This function returns the maximum PPDU time based on the PHY in use */
INLINE UWORD32 get_ppdu_max_time(void)
{
	UWORD32 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = PHY_C.aPPDUMaxTime;
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the PHY type with the running mode and protection   */
/* control in use.                                                           */
INLINE UWORD8 get_phy_type(void)
{
    UWORD8 phy_type = 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        phy_type = PHY_N_5GHZ;
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        phy_type = PHY_N;
    }
#endif /* PHY_802_11n */

    return phy_type;
}

/* This function returns the maximum operational rate supported by PHY */
INLINE UWORD8 get_max_op_rate(void)
{
    if(g_curr_rate_struct.num_rates)
    {
        return g_curr_rate_struct.rates[g_curr_rate_struct.num_rates - 1].mbps;
    }
        else
    {
        return 0;
    }
}

INLINE UWORD8 get_current_channel(UWORD8 *msa, UWORD16 rx_len)
{
	UWORD8 ret_val = 0;





#ifdef PHY_802_11n
    /* Extract current channel information from */
    /* the beacon/probe response frame          */
    ret_val = (get_current_channel_802_11n (msa, rx_len));
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the PHY Tx mode to be used for transmission to a    */
/* STA based on its HT capability. This is used for descriptor setting only  */
INLINE UWORD32 get_dscr_phy_tx_mode(UWORD8 tx_rate, UWORD8 pream, void *entry)
{
	UWORD32 ret_val = 0;

#ifdef PHY_802_11n
    ret_val = get_phy_tx_mode_802_11n(tx_rate, pream, entry, 0);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the PHY Tx mode to be used for transmission to a    */
/* STA based on its HT capability. This is used for register setting only.   */
INLINE UWORD32 get_reg_phy_tx_mode(UWORD8 tx_rate, UWORD8 pream)
{
	UWORD32 ret_val = 0;




#ifdef PHY_802_11n
    ret_val = get_phy_tx_mode_802_11n(tx_rate, pream, 0, 1);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function returns the appropriate TX power to be used, given the data */
/* rate and PHY TX Mode.                                                     */
INLINE UWORD8 get_tx_pow(UWORD8 dr, UWORD32 ptm)
{
    UWORD8 pow = 0;



#ifdef PHY_802_11n

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    if(IS_RATE_11B(dr) == BTRUE)
    {
        pow = g_curr_tx_power.pow_b.gaincode;
    }
    else
    {
        if(IS_RATE_MCS(dr) == BTRUE)
        {
            pow = g_curr_tx_power.pow_n.gaincode;

        }
        else
        {
            pow = g_curr_tx_power.pow_a.gaincode;
        }
    }
#else
	pow = get_tx_pow_11n(dr, ptm);
#endif
	
#endif /* PHY_802_11n */

    return pow;
}

// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
/* This function returns the appropriate TX power to be used, given the data */
/* rate and PHY TX Mode.                                                     */
INLINE tx_power_t *get_current_tx_pow(UWORD8 dr, UWORD32 ptm)
{
    tx_power_t *pow = 0;




#ifdef PHY_802_11n
    if(IS_RATE_11B(dr) == BTRUE)
    {
        pow = &(g_curr_tx_power.pow_b);
    }
    else
    {
        if(IS_RATE_MCS(dr) == BTRUE)
        {
            pow = &(g_curr_tx_power.pow_n);

        }
        else
        {
            pow = &(g_curr_tx_power.pow_a);
        }
    }
#endif /* PHY_802_11n */

    return pow;
}
#endif

/* This function initializes the global autorate table based on the PHY type */
INLINE void init_ar_table(void)
{
#ifdef AUTORATE_FEATURE
#ifdef PHY_802_11n
    init_ar_table_phy_802_11n();
#endif /* PHY_802_11n */
#endif /* AUTORATE_FEATURE */
}


/* This function returns the protection control register value based on the  */
/* PHY type.                                                                 */
INLINE UWORD8 get_phy_prot_con(void)
{
    UWORD8 temp = 0;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        temp = 0x01;
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        if(get_running_mode() == G_ONLY_MODE)
        {
            temp = 0x03; /* ERP Only, No protection */
        }
        else if(get_protection() == ERP_PROT)
        {
            if(get_erp_prot_type() == G_SELF_CTS_PROT)
                temp = 0x32; //* Mixed, Self-CTS protection 

            if(get_erp_prot_type() == G_RTS_CTS_PROT)
                temp = 0x12; //* Mixed, RTS-CTS protection 
        }
        else
        {
            temp = 0x02; /* Mixed, No protection */
        }
    }
#endif /* PHY_802_11n */

    return temp;
}

/* This function returns the protection data rate value based on PHY type */
INLINE UWORD8 get_prot_data_rate(void)
{
    UWORD8 dr = 0;




#ifdef PHY_802_11n
    /* The protection data rate is the maximum basic rate */
    dr = get_max_basic_rate();

    /* Currently only non-HT protection rate is supported. If the maximum    */
    /* basic rate is a HT rate, the protection data rate is set to 24Mbps.   */
    if(IS_RATE_MCS(dr) == BTRUE)
        dr = 24;
#endif /* PHY_802_11n */

    return dr;
}

/* This function returns the protection PHY Tx mode value based on PHY type */
INLINE UWORD32 get_phy_prot_tx_mode(UWORD8 dr, UWORD8 pr)
{
    UWORD32 temp = 0;




#ifdef PHY_802_11n
    temp = get_reg_phy_tx_mode(dr, pr);
#endif /* PHY_802_11n */
    return temp;
}

/* This function returns the rate at which the beacon should be transmitted */
INLINE UWORD8 get_beacon_tx_rate(void)
{
    UWORD8 dr = 0;




#ifdef PHY_802_11n
    dr = get_min_basic_rate();
#endif /* PHY_802_11n */

    return dr;
}
/* This function updates the maximum and minimum basic rate members of the */
/* current rate set.                                                       */
INLINE void update_max_min_basic_rates(void)
{
    UWORD8       max_br = 0;
    UWORD8       min_br = 0;
    UWORD8       count  = 0;
    data_rate_t *curr_rate = NULL;

    /* The current rate set is parsed to extract the Max & Min basic rates */
    for(count = 0; count < g_curr_rate_struct.num_rates; count++)
    {
        curr_rate = &g_curr_rate_struct.rates[count];

        if((curr_rate->mac_rate & 0x80) != 0)
        {
            max_br = curr_rate->mbps;

            if(min_br == 0)
                min_br = curr_rate->mbps;
        }
    }

    g_curr_rate_struct.max_br = max_br;
    g_curr_rate_struct.min_br = min_br;
}

/*  This function adds a new data rate to current data rate  */
/*  structure at the appropriate place (ascending order)     */
INLINE void insert_data_rate(UWORD8 mac_rate, const data_rate_t *data_rate)
{
   UWORD8 curr_mbps = 0;
   UWORD8 curr_mac_rate = 0;
   data_rate_t* curr_location = NULL;
   data_rate_t* new_location  = NULL;
   UWORD8 index = g_curr_rate_struct.num_rates;
   UWORD8 count = 0;
   int smart_type = 0;

   /* Find the right index where this data rate is to be inserted */
   while(count < g_curr_rate_struct.num_rates)
   {
      curr_mbps     = g_curr_rate_struct.rates[count].mbps;
      curr_mac_rate = g_curr_rate_struct.rates[count].mac_rate;

      /* This rate is already present in the table */
      if(curr_mbps == data_rate->mbps)
      {
         if(mac_rate != curr_mac_rate)
         {
            if(mac_rate & 0x80)
            {
               g_curr_rate_struct.num_br++;
               g_curr_rate_struct.num_nbr--;
            }
            else
            {
               g_curr_rate_struct.num_br--;
               g_curr_rate_struct.num_nbr++;
            }
            g_curr_rate_struct.rates[count].mac_rate = mac_rate;
         }
         return;
      }
      /* Already a higher rate found, so insert here */
      else if (curr_mbps > data_rate->mbps)
      {
         index = count;
         break;
      }
      /* Keep searching for a higher rate */
      else
      {
         count++;
      }

   } /* End of While Loop */

   smart_type = critical_section_smart_start(0,1);

   /* If any higher rates are already there, then shift them */
   if(index < g_curr_rate_struct.num_rates)
   {
       /* Re-arrange higher data rates to create space for current data rate */
       for(count = g_curr_rate_struct.num_rates - 1 ; count >= index; count--)
       {
          curr_location = &g_curr_rate_struct.rates[count];
          new_location  = &g_curr_rate_struct.rates[count + 1];

          memcpy(new_location,curr_location,sizeof(data_rate_t));
          if(count == 0)
              break;
       }
   }

    /* Copy the current data rate */
    memcpy(&g_curr_rate_struct.rates[index],data_rate,sizeof(data_rate_t));

    /* Copy the MAC rate */
    g_curr_rate_struct.rates[index].mac_rate = mac_rate;

    /* Increment the appropriate counts */
    if(mac_rate & 0x80)
    {
       g_curr_rate_struct.num_br++;
    }
    else
    {
       g_curr_rate_struct.num_nbr++;
    }

    g_curr_rate_struct.num_rates++;

    update_max_min_basic_rates();

    critical_section_smart_end(smart_type);
}

/* This function updates the phy rate table with current values */
INLINE void update_phy_rate_table(UWORD8 *supp_rates,UWORD8 *extn_rates)
{
    UWORD8 count = 0;
    /* Save a copy of current TX rate */
    UWORD8 tx_rate = get_curr_tx_rate();

    /* Reset the PHY Rate Structure */
    if((supp_rates != NULL) ||
       (extn_rates != NULL))
    {
        g_curr_rate_struct.num_rates = 0;
        g_curr_rate_struct.num_br    = 0;
        g_curr_rate_struct.num_nbr   = 0;
    }
    else
    {
        return;
    }

    /* Re-initialize PHY Rate Strcture with this supported rates */
    if(supp_rates != NULL)
    {
        UWORD8 num_supp_rates = *supp_rates;

        for(count = 0; count < num_supp_rates; count++)
        {
           add_mac_rate(supp_rates[count + 1]);
        }
    }

    /* Add Extended rates to PHY Rate Structure */
    if(extn_rates != NULL)
    {
        UWORD8 num_extn_rates = *extn_rates;

        for(count = 0; count < num_extn_rates; count++)
        {
           add_mac_rate(extn_rates[count + 1]);
        }
    }

    /* Restore the current TX rate */
    set_curr_tx_rate(tx_rate);
}

/* This function initializes the PHY current rate set */
INLINE void init_phy_curr_rates(void)
{
#ifdef PHY_802_11n
    init_phy_802_11n_curr_rates();
#endif /* PHY_802_11n */
}

/* This function gets the RF band based on the PHY type */
INLINE UWORD8 get_rf_band(void)
{
	UWORD8 ret_val = 0;

    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        ret_val = 0x02; /* 5 GHz */
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        ret_val = 0x01; /* 2.4 GHz */
    }

    return ret_val;
}

/* This function checks the validity of Secondary Channel corresponding to the */
/* specified Primary Channel Index and Secondary Channel offset.               */
INLINE BOOL_T is_sec_channel_valid(UWORD8 pri_idx, UWORD8 sec_offset)
{
	BOOL_T ret_val = BFALSE;



#ifdef PHY_802_11n
    ret_val = is_sec_channel_valid_11n(pri_idx, (N_SEC_CH_OFFSET_T)sec_offset);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function checks the validity of Secondary Channel corresponding to the */
/* specified Primary Channel Index and Secondary Channel offset.               */
INLINE UWORD8 get_sec_ch_idx(UWORD8 pri_idx, UWORD8 sec_offset)
{
	UWORD8 ret_val = pri_idx;




#ifdef PHY_802_11n
    ret_val = get_sec_ch_idx_11n(pri_idx, (N_SEC_CH_OFFSET_T)sec_offset);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function checks if the channel is in the radar band */
INLINE UWORD8 is_ch_in_radar_band(UWORD8 freq, UWORD8 ch_idx)
{
    BOOL_T ret_val = BFALSE;

#ifdef PHY_802_11n
    UWORD8           reg_class = get_reg_class_from_ch_idx(freq, ch_idx);
	
	// 20120830 caisf mod, merged ittiam mac v1.3 code
	#if 0
    reg_class_info_t *rc_info  = get_reg_class_info(freq, reg_class);
	#else
    reg_class_info_t *rc_info  = get_reg_class_info(freq, reg_class,
                                                get_current_reg_domain());
	#endif
	
    if(rc_info != NULL)
        ret_val = is_rc_dfs_req(rc_info);
#endif /* PHY_802_11n */

    return ret_val;
}

/* This function checks if the input PHY-Rate is a 11b rate or not */
INLINE BOOL_T is_11b_rate(UWORD8 phy_rate)
{
    BOOL_T retval = BFALSE;




#ifdef PHY_802_11n
    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        retval = BFALSE;
    }
    else if(get_current_start_freq() == RC_START_FREQ_2)
    {
        if((IS_RATE_MCS(phy_rate) == BFALSE) && ((phy_rate & BIT3) == 0))
        {
            retval = BTRUE;
        }
    }
#endif /* PHY_802_11n */

    return retval;
}
#endif /* PHY_PROT_IF_H */

