/*****************************************************************************/
/*                                                                           */
/*                           Ittiam WPS SOFTWARE                             */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : wps_key.h                                            */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      generating Authentication and Key Wrap Key           */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         24 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef WPS_KEY_H
#define WPS_KEY_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpsr_reg_if.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern STATUS_T wps_gen_keys(UWORD8 *rx_pub_key, UWORD8 *enr_nonce,
                             UWORD8 *reg_nonce, UWORD8 *enr_mac_addr,
                             UWORD8 *auth_key, UWORD8 *key_wrap_key,
                             long_num_struct_t *priv_key_ptr);

#endif /* WPS_KEY_H */
#endif /* INT_WPS_REG_SUPP */
