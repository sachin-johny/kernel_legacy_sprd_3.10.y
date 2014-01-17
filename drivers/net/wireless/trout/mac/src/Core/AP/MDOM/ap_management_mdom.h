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
/*  File Name         : ap_management_mdom.h                                 */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      declarations for Multi-domain.                       */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_MULTIDOMAIN
#ifndef AP_MANAGEMENT_MDOM_H
#define AP_MANAGEMENT_MDOM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern UWORD8 get_rc_info_sec_chan_offset(UWORD8 ch_idx, UWORD8 sec_offset);
extern UWORD8 select_suitable_reg_class(UWORD8 ch_idx, UWORD8 sec_offset);

#endif /* AP_MANAGEMENT_11D_H */
#endif  /* MAC_MULTIDOMAIN */
#endif /* BSS_ACCESS_POINT_MODE */
