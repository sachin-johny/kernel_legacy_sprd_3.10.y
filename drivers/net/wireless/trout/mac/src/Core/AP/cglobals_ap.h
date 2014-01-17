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
/*  File Name         : cglobals_ap.h                                        */
/*                                                                           */
/*  Description       : This file contains all the globals used in the MAC   */
/*                      the AP mode.                                         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef CGLOBALS_AP_H
#define CGLOBALS_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management_ap.h"

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

/* Power management related globals */
extern UWORD8       g_num_sta_ps;
extern UWORD16      g_num_mc_bc_pkt;
extern list_buff_t  g_mc_q;
extern UWORD8       g_vbmap[VBMAP_SIZE];
extern UWORD8       g_tim_element_index;
extern UWORD16      g_tim_element_trailer_len;
// 20120709 caisf add, merged ittiam mac v1.2 code
extern UWORD8	    g_update_active_bcn;

#ifdef PS_DSCR_JIT
/*****************************************************************************/
/* This queue is not used currently. May be used if descriptors are not      */
/* buffered and are created just in time before transmission                 */
/*****************************************************************************/
extern list_buff_t  g_ps_pending_q;
#endif /* PS_DSCR_JIT */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_globals_ap(void);

#endif /* CGLOBALS_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
