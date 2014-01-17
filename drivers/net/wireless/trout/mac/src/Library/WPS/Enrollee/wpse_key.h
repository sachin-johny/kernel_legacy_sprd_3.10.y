/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wpse_key.h                                           */
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
#ifdef INT_WPS_ENR_SUPP

#ifndef WPS_KEY_H
#define WPS_KEY_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_hmac_sha256.h"
#include "wpse_dh_key.h"
/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern STATUS_T wps_gen_keys(wps_enrollee_t *wps_enrollee);
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

#endif /* WPS_KEY_H */
#endif /* INT_WPS_ENR_SUPP */
