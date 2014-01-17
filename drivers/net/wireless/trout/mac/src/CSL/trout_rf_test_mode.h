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
/*  File Name         : trout_rf_test_mode.h                                             */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the trout_rf_test_mode.                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef TROUT_RF_TEST_MODE_H_
#define TROUT_RF_TEST_MODE_H_

#ifdef TROUT_RF_TEST
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include <linux/netdevice.h>

#include "common.h"
#include "itypes.h"


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
struct rf_test_param
{
    UWORD16 magic;          // 0x9812
    UWORD16 type;           // 0 --> undefine; 1 --> tx; 2 --> rx;
    UWORD16 tx_pkt_size;    // tx pkt size
    UWORD16 tx_pkt_num;     // tx pkt num
    UWORD8  channel;        // range: 1-14;
    UWORD8  phy_mode;       // 0 undefine; 1 --> 802.11 bg only phy mode; 2 --> 802.11 bgn phy mode;
    UWORD8  rate;           // rate;
    UWORD8  power;          // tx power level, unit:dbm;
    UWORD8  freq_offset;    // frequency-offset
    volatile UWORD8  nonstop_work;   // tx/rx none stop, 0 undefine, 1 non stop, 2 stop
    UWORD8  dmac[6];        // destination mac address
};


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern int trout_rf_test_init(struct net_device *dev);
extern void trout_rf_test_release(struct net_device *dev);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

#endif /* #ifdef TROUT_WIFI_NPI */

#endif /* TROUT_RF_TEST_MODE_H_ */
