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
/*  File Name         : tpc_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains all the functions declarations    */
/*                      that are used in tpc_sta.c                           */
/*                      Station mode of operation.                           */
/*                                                                           */
/*  List of Functions :  set_tx_power_field                                  */
/*                       set_linkmargin_field                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_802_11H

#ifdef IBSS_BSS_STATION_MODE

#ifndef TPC_STA_H
#define TPC_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "mh.h"
#include "csl_if.h"
#include "cglobals.h"
#include "phy_prot_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define TPCREPORT              3
#define SENSITIVITY   (WORD8) -70 /* Sensitivity in dBm */

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/
extern void update_power_constr (UWORD8 *msa,UWORD16 rx_len,
                                                     UWORD16 tag_param_offset);
// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
extern void update_reg_max_power(UWORD8 *msa,UWORD16 rx_len,
                                                     UWORD16 tag_param_offset);
#endif

extern void send_tpc_report(WORD8 rx_power_req,UWORD8 dia_token);
extern UWORD16 set_power_cap_element(UWORD8* data, UWORD16 index);
extern BOOL_T check_tpc_power_limit(void);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function is used to set Tx power of TPC report frame         */
INLINE UWORD16 set_tx_power_field(UWORD8 *msa,UWORD16 index)
{
	// 20120830 caisf mod, merged ittiam mac v1.3 code
    //msa[index++] = g_curr_tx_power.pow_a.dbm;
	msa[index++] = get_tpc_report_tx_pow(g_user_control_enabled);

    return index;
}

/* This function is used to set Link margin in TPC report frame        */
INLINE void set_linkmargin_field(UWORD8 *msa, WORD8 rx_power_req,UWORD16 index)
{
    WORD8 link_margin = 0;

    link_margin = rx_power_req - SENSITIVITY;
    msa[index]  = link_margin;
}

#endif /* TPC_STA_H */

#endif /* IBSS_BSS_STATION_MODE */

#endif /* MAC_802_11H */



