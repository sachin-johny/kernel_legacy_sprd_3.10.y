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
/*  File Name         : rf_if.h                                              */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the RF interface.              */
/*                                                                           */
/*  List of Functions : get_rf_con_width_val                                 */
/*                      write_rf_reg                                         */
/*                      convert_regval_to_dbm                                */
/*                      convert_dbm_to_regval                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef RF_IF_H
#define RF_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"

#ifdef RF_MAXIM_ITTIAM
#include "vco_maxim_ittiam.h"
#include "reg_domain_maxim_ittiam.h"
#endif /* RF_MAXIM_ITTIAM */

// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
#ifdef RF_AIROHA_ITTIAM
#include "vco_airoha_ittiam.h"
#endif /* RF_AIROHA_ITTIAM */
#endif


#ifdef GENERIC_RF
#include "reg_domain_generic.h"
#endif /* GENERIC_RF */

/*****************************************************************************/
/* Globals                                                                   */
/*****************************************************************************/

extern UWORD8 g_dscr_tssi_11b;
extern UWORD8 g_dscr_tssi_11gn;

#ifndef GENERIC_RF
extern WORD8 g_tx_power_levels_in_dbm[];

//modified by Hugh
#if 0
extern UWORD8 g_gaincode_freq_5[][];
extern UWORD8 g_gaincode_freq_2[][];
#else
extern UWORD8 g_gaincode_freq_5[][MAX_TX_POWER_LEVELS];
extern UWORD8 g_gaincode_freq_2[][MAX_TX_POWER_LEVELS];
#endif

#ifdef TX_POWER_LOOPBACK_ENABLED
extern UWORD8 g_tssi_freq_5[][];
extern UWORD8 g_tssi_freq_2[][];
#endif /* TX_POWER_LOOPBACK_ENABLED */
#endif /* GENERIC_RF */

/*****************************************************************************/
/* RF Interface Register                                                     */
/*****************************************************************************/

#define rMAC_RF_VCO_SPI_CON_WIDTH  (PA_BASE + 0x060C)
#define rMAC_RF_VCO_SPI_CON        (PA_BASE + 0x0610)
#define rMAC_RF_VCO_SPI_STAT       (PA_BASE + 0x0614)

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void initialize_rf(void);
extern void select_channel_rf(UWORD8 channel, UWORD8 sec_offset);
extern void add_rf_delay(UWORD32 count);
extern void initialize_rf_2040(void);
extern UWORD8 get_avg_tssi( void );
extern UWORD8 convert_dbm_to_regval(WORD8 dbm, UWORD8 freq, UWORD8 ch_idx);

#ifdef TX_POWER_LOOPBACK_ENABLED
extern void   correct_tx_gain_value(UWORD32 *dscr);
#endif /* TX_POWER_LOOPBACK_ENABLED */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function is used to write to the RF VCO register for programming the */
/* RF with the required value.                                               */
INLINE void write_RF_reg(UWORD8 address, UWORD32 data)
{
#ifdef RF_MAXIM_ITTIAM
    write_RF_reg_ittiam(address, data);
#endif /* RF_MAXIM_ITTIAM */

// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
#ifdef RF_AIROHA_ITTIAM
    write_RF_reg_ittiam(address, data);
#endif /* RF_AIROHA_ITTIAM */
#endif


#ifdef GENERIC_RF
    /* No Equivalent function defined */
#endif /* GENERIC_RF */
}

/* This function initializes the supported Regulatory Domain table based on  */
/* the RF/PHY defined.                                                       */
INLINE void init_supp_reg_domain_rf(void)
{
#ifdef RF_MAXIM_ITTIAM
    init_reg_domain_maxim_ittiam();
#endif /* RF_MAXIM_ITTIAM */

// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
#ifdef RF_AIROHA_ITTIAM
    init_reg_domain_airoha_ittiam();
#endif /* RF_AIROHA_ITTIAM */
#endif


#ifdef GENERIC_RF
    init_reg_domain_generic();
#endif /* GENERIC_RF */
}

/* This function gets the RF control width based on the RF in use */
INLINE UWORD8 get_rf_con_width_val(void)
{
    /* Deprecated function */
    return 0;
}

#endif /* RF_IF_H */
