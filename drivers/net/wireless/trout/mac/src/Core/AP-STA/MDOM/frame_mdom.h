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
/*  File Name         : frame_mdom.h                                         */
/*                                                                           */
/*  Description       : This file contains all the declarations for the      */
/*                      preparation of the various MAC frames specific to    */
/*                      Multi-domain operation.                              */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_MULTIDOMAIN
#ifndef FRAME_MDOM_H
#define FRAME_MDOM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern UWORD8 set_sup_op_classes_ie(UWORD8* data, UWORD16 index);
extern UWORD8 set_country_info_elem_frm_reg_tbl(UWORD8* data, UWORD16 index);

#endif /* FRAME_MDOM_H */
#endif /* MAC_MULTIDOMAIN */
