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
/*  File Name         : phy_hw_if.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions and declarations  */
/*                      for the PHY hardware interface.                      */
/*                                                                           */
/*  List of Functions : Interface functions for PHY hardware                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_HW_IF_H
#define PHY_HW_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/


#ifdef ITTIAM_PHY
#include "phy_ittiam.h"
#endif /* ITTIAM_PHY */


#include "ch_info.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#ifdef GENERIC_PHY

/* The maximum MCS value supported. This is valid only for 802.11n PHY */
#define MAX_MCS_SUPPORTED                       0

/* The default power level to be used for transmission */
#define DEFAULT_TX_POWER                        10

/* PHY dependent MAC Register initial value definition for Generic-PHY      */
#define MAC_TXPLCP_DELAY_INIT_VALUE          0x00000000
#define MAC_RXPLCP_DELAY_INIT_VALUE          0x00000000
#define MAC_RXTXTURNAROUND_TIME_INIT_VALUE   0x00000000
#define MAC_PHY_CCA_DELAY_INIT_VALUE         0x00000000
#define MAC_TXPLCP_ADJUST_INIT_VALUE         0x00000000
#define MAC_RXPLCP_DELAY2_INIT_VALUE         0x00000000

#endif /* GENERIC_PHY */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function initializes the PHY in use */
INLINE void initialize_phy(void)
{
	struct trout_private *tp = netdev_priv(g_mac_dev);
	
	/* initialize_phy should not be interruptted by zhao */
	mutex_lock(&tp->iphy_mutex); 
#ifdef ITTIAM_PHY
    initialize_phy_ittiam();
#endif /* ITTIAM_PHY */
	mutex_unlock(&tp->iphy_mutex); 
}

/* Read the Phy Register version */
INLINE void read_phy_version(void)
{

#ifdef ITTIAM_PHY
    read_phy_hw_version_ittiam();
#endif /* ITTIAM_PHY */

}

/* This function sets default transmit power levels */
INLINE void set_default_tx_power_levels(void)
{

#ifdef ITTIAM_PHY
    set_default_tx_power_levels_ittiam();
#endif /* ITTIAM_PHY */

}

/* This function sets in PHY the TX-Power Level for 11a Transmissions */
INLINE void set_phy_tx_power_level_11a(UWORD8 val)
{

#ifdef ITTIAM_PHY
    set_phy_tx_power_level_11a_ittiam(val);
#endif /* ITTIAM_PHY */

}

/* This function sets in PHY the TX-Power Level for 11n Transmissions */
INLINE void set_phy_tx_power_level_11n(UWORD8 val)
{

#ifdef ITTIAM_PHY
    /* No equivalent function available */
#endif /* ITTIAM_PHY */

}

/* This function sets in PHY the TX-Power Level for 11b Transmissions */
INLINE void set_phy_tx_power_level_11b(UWORD8 val)
{

#ifdef ITTIAM_PHY
    set_phy_tx_power_level_11b_ittiam(val);
#endif /* ITTIAM_PHY */

}

/* This function returns the current RX Sensitivity parameter from PHY */
INLINE UWORD16 get_phy_rx_sense(void)
{
    UWORD16 ret_val = 0;


#ifdef ITTIAM_PHY
    ret_val = get_phy_rx_sense_ittiam();
#endif /* ITTIAM_PHY */


    return ret_val;
}

/* This function sets the RX Sensitivity parameter in PHY */
INLINE void set_phy_rx_sense(UWORD16 val)
{

#ifdef ITTIAM_PHY
    set_phy_rx_sense_ittiam(val);
#endif /* ITTIAM_PHY */

}

/* This function returns the current CCA Threshold parameter from PHY */
INLINE UWORD16 get_phy_cca_threshold(void)
{
    UWORD16 ret_val = 0;


#ifdef ITTIAM_PHY
    ret_val = get_phy_cca_threshold_ittiam();
#endif /* ITTIAM_PHY */


    return ret_val;
}

/* This function sets the CCA Threshold parameter in PHY */
INLINE void set_phy_cca_threshold(UWORD16 input)
{

#ifdef ITTIAM_PHY
    set_phy_cca_threshold_ittiam(input);
#endif /* ITTIAM_PHY */

}

/* This function returns the data rate of the received frame from the */
/* PHY-Register                                                       */
INLINE UWORD8 get_phy_dr(void)
{
    UWORD8 ret_val = 0;


#ifdef ITTIAM_PHY
    ret_val = get_phy_dr_ittiam();
#endif /* ITTIAM_PHY */


    return ret_val;
}

#ifdef PHY_CONTINUOUS_TX_MODE
/* This function enables continuous Transmit mode in PHY */
INLINE void set_phy_continuous_tx_mode(void)
{

#ifdef ITTIAM_PHY
    set_phy_continuous_tx_mode_ittiam();
#endif /* ITTIAM_PHY */


}
#endif /* PHY_CONTINUOUS_TX_MODE */

/* This function triggers capture of RX samples in PHY */
INLINE void capture_samples(void)
{

#ifdef ITTIAM_PHY
    /* No equivalent function */
#endif /* ITTIAM_PHY */

}

/* This function waits for any PHY Level synchronization to complete before */
/* the PHY is brought out of Reset.                                         */
INLINE void wait_for_sync(void)
{

#ifdef ITTIAM_PHY
    /* No equivalent function */
#endif /* ITTIAM_PHY */

}

/* This function resets any PHY Level synchronization before PHY is put */
/* in disabled state.                                                   */
INLINE void reset_sync(void)
{

#ifdef ITTIAM_PHY
    /* No equivalent function */
#endif /* ITTIAM_PHY */

}

/* This function updates the MCS bitmap defined in the standards based on the*/
/* PHY capabilities. The bitmap is used to indicate the MCS values that the  */
/* PHY is able to receive. It returns the number of MCS values supported     */
/* by the PHY.                                                               */
INLINE UWORD8 update_supp_mcs_bitmap(UWORD8 *mcs_list)
{
    UWORD8 ret_val = 0;


#ifdef ITTIAM_PHY
    ret_val = update_supp_mcs_bitmap_ittiam(mcs_list);
#endif /* ITTIAM_PHY */


    return ret_val;
}

/* This function resets the PHY */
INLINE void reset_phy(void)
{

#ifdef ITTIAM_PHY
    reset_phy_ittiam();
#endif /* ITTIAM_PHY */

}

/* This function powers up PHY. This is used for BSS STA power management. */
INLINE void power_up_phy(void)
{

#ifdef ITTIAM_PHY
    power_up_phy_ittiam();
#endif /* ITTIAM_PHY */

}

/* This function powers down PHY. This is used for BSS STA power management. */
INLINE void power_down_phy(void)
{

#ifdef ITTIAM_PHY
    power_down_phy_ittiam();
#endif /* ITTIAM_PHY */

}
/*zhq add for powersave*/
INLINE void root_power_down_phy(void)
{

#ifdef ITTIAM_PHY
    root_power_down_phy_ittiam();
#endif /* ITTIAM_PHY */

}

/* This function updates a value in the PHY initialization table */
INLINE void update_phy_init_table(UWORD8 reg_idx, UWORD32 val)
{

#ifdef ITTIAM_PHY
    update_ittiam_reg_val_table(reg_idx, val);
#endif /* ITTIAM_PHY */

}

/* This function updates the secondary CCA Control register in PHY */
INLINE void update_sec_cca_control_phy(UWORD32 val)
{

#ifdef ITTIAM_PHY
    /* Do nothing */
#endif /* ITTIAM_PHY */

}

/* This function perfroms any calibration required by PHY */
INLINE void perform_phy_calibration(void)
{

#ifdef ITTIAM_PHY
    /* Do nothing */
#endif /* ITTIAM_PHY */

}

/* This function gets the TSSI value */
INLINE UWORD8 get_phy_tssi_11b(void)
{
    UWORD8 phy_tssi_11b = 0;

    if(get_current_start_freq() == RC_START_FREQ_2)
    {

#ifdef ITTIAM_PHY
    phy_tssi_11b = get_tssi_11b_ittiam();
#endif /* ITTIAM_PHY */

    }

    return phy_tssi_11b;
}

/* This function gets the TSSI value */
INLINE UWORD8 get_phy_tssi_11g(void)
{
    UWORD8 phy_tssi_11g = 0;


#ifdef ITTIAM_PHY
    phy_tssi_11g = get_tssi_11gn_ittiam(); /* For MIXED_MODE TSSI is same for g & n */
#endif /* ITTIAM_PHY */


    return phy_tssi_11g;
}

/* This function gets the TSSI value */
INLINE UWORD8 get_phy_tssi_11n(void)
{
    UWORD8 phy_tssi_11n = 0;


#ifdef ITTIAM_PHY
    phy_tssi_11n = get_tssi_11gn_ittiam(); /* For MIXED_MODE TSSI is same for g & n */
#endif /* ITTIAM_PHY */


    return phy_tssi_11n;
}

/* This function computes the RSSI value in db */
INLINE BOOL_T compute_rssi_db(WORD8 rssi, BOOL_T is_ofdm, SWORD8 *rssi_db)
{
    BOOL_T ret_val = BFALSE;


#ifdef ITTIAM_PHY
    ret_val = compute_rssi_db_ittiam(rssi, is_ofdm, rssi_db);
#endif /* ITTIAM_PHY */


    return ret_val;
}

/* This function checks whether the PHY Address is valid */
INLINE BOOL_T is_valid_phy_reg(UWORD8 regaddr)
{
    BOOL_T retval = BFALSE;


#ifdef ITTIAM_PHY
    retval = is_valid_phy_reg_ittiam(regaddr);
#endif /* ITTIAM_PHY */


    return retval;
}

/* This function clears the Radar Detect bit in PHY */
INLINE void clear_phy_radar_status_bit(void)
{

#ifdef ITTIAM_PHY
    clear_phy_radar_status_bit_ittiam();
#endif /* ITTIAM_PHY */

}



INLINE void post_radar_detect_phy(void)
{
#ifdef ITTIAM_PHY
    post_radar_detect_ittiam();
#endif /* ITTIAM_PHY */
}

/* This function is used to enable radar detection */
INLINE void enable_radar_detection(void)
{
#ifdef ITTIAM_PHY
    enable_radar_detection_ittiam();
#endif /* ITTIAM_PHY */
}

/* This function is used to disable radar detection */
INLINE void disable_radar_detection(void)
{
#ifdef ITTIAM_PHY
    disable_radar_detection_ittiam();
#endif /* ITTIAM_PHY */
}

INLINE void reset_ofdmpre_unidentified_bits_phy(void)
{
#ifdef ITTIAM_PHY
    reset_ofdmpre_unidentified_bits_ittiam();
#endif /* ITTIAM_PHY */
}

INLINE BOOL_T is_ofdm_preamble_detected(void)
{
#ifdef ITTIAM_PHY
    return is_ofdm_preamble_detected_ittiam();
#endif /* ITTIAM_PHY */

    return BFALSE;
}

INLINE BOOL_T is_unidentified_bit_detected(void)
{
#ifdef ITTIAM_PHY
    return is_unidentified_bit_detected_ittiam();
#endif /* ITTIAM_PHY */

    return BFALSE;
}

/* This function creates all alarms related to PHY H/w */
INLINE void create_phy_alarms(void)
{
}

/* This function deletes all alarms related to PHY H/w */
INLINE void delete_phy_alarms(void)
{
}

/* This function updates required PHY registers based on channel selection */
INLINE void update_phy_on_select_channel(UWORD8 freq, UWORD8 ch_idx)
{

#ifdef ITTIAM_PHY
    update_phy_ittiam_on_select_channel(freq, ch_idx);
#endif /* ITTIAM_PHY */

}

#endif /* PHY_HW_IF_H */
