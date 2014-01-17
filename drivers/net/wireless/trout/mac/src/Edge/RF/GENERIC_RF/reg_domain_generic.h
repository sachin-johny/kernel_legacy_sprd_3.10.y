/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
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
/*  File Name         : reg_domain_maxim_ittiam.h                            */
/*                                                                           */
/*  Description       : This file contains the definitions for Regulatory    */
/*                      Domain configuration for Maxim RF/Ittiam PHY         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef REG_DOMAIN_GENERIC_H
#define REG_DOMAIN_GENERIC_H

#ifdef GENERIC_RF

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Regulatory Domain Index - Note that RD0 is Mandatory and must be 0 since  */
/* it is used as the default information if no Regulatory Domain information */
/* is available                                                              */
typedef enum {RD0 = 0, /* Default - No Country : NOTE: DO NOT MODIFY */
              NUM_REG_DOMAINS
} REG_DOMAINS_T;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* Default Regulatory Domain configured */
#define DEFAULT_REG_DOMAIN NO_REG_DOMAIN

/*****************************************************************************/
/* Number of Channels supported by the RF in each frequency band             */
/*****************************************************************************/

#define NUM_SUPP_CHANNEL_FREQ_2 14
#define NUM_SUPP_CHANNEL_FREQ_5 24

/*****************************************************************************/
/* Number of Regulatory Classes Supported per Regulatory Domain              */
/* NOTE - To add new Regulatory Domains, additional constants for Regulatory */
/*        classes for that Regulatory Domain must be added here.             */
/*****************************************************************************/

#define NUM_REG_CLASS_RD0 2

/*****************************************************************************/
/* TX Power values in dBm for various Regulatory Domains/Regulatory Classes  */
/*                                                                           */
/* NOTE - Max supported power must be less than or equal to  Regulatory Max  */
/*        RDx_RCyy_MAX_TX_POW_* <= RDx_RCyy_REG_MAX_TX_POW                   */
/*                                                                           */
/* NOTE - Max supported power is per antenna in dBm. It is assumed that for  */
/*        a N TX antenna configuration N TX antennas shall always be used    */
/*        for transmission of all MCSes and maybe legacy rates. The values   */
/*        for Max supported power (RDx_RCyy_MAX_TX_POW_*) need to be         */
/*        appropriately set to account for this.                             */
/*                                                                           */
/*        RDx_RCyy_MAX_TX_POW_* + 3dB <= RDx_RCyy_REG_MAX_TX_POW (2-antenna) */
/*        RDx_RCyy_MAX_TX_POW_* + 5dB <= RDx_RCyy_REG_MAX_TX_POW (3-antenna) */
/*        RDx_RCyy_MAX_TX_POW_* + 6dB <= RDx_RCyy_REG_MAX_TX_POW (4-antenna) */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* Regulatory Domain #0 (NO_REG_DOMAIN)                                      */
/*****************************************************************************/

/* ---------------------------------------------------------------------     */
/* Default Table: To be used when Multi-Domain is not enabled                */
/* ---------------------------------------------------------------------     */
/* |RC  | RC | Freq  | Spac  | Channel            | TX pow | Behavior  |     */
/* |IDX |    | (GHz) | (MHz) | Set                | limit  | Limit Set |     */
/* ---------------------------------------------------------------------     */
/* | 0  | 1  |  5    | 20    | 36, 40, 44, 48,    | MD-TBD | MD-TBD    |     */
/* |    |    |       |       | 52, 56, 60, 64,    |        |           |     */
/* |    |    |       |       | 100, 104, 108, 112 |        |           |     */
/* |    |    |       |       | 116, 120, 124, 128 |        |           |     */
/* |    |    |       |       | 132, 136, 140      |        |           |     */
/* |    |    |       |       | 149, 153, 157, 161 |        |           |     */
/* |    |    |       |       | 165                |        |           |     */
/* ---------------------------------------------------------------------     */
/* | 1  | 2  | 2.407 | 25    | 1, 2, 3, 4, 5, 6,  | MD-TBD | MD-TBD    |     */
/* |    |    |       |       | 7, 8, 9, 10, 11    |        |           |     */
/* |    |    |       |       | 12, 13, 14         |        |           |     */
/* ---------------------------------------------------------------------     */

/* Regulatory Domain #0 - Regulatory Class #01 */
#define RD0_RC01_REG_MAX_TX_POW 20
#define RD0_RC01_MIN_TX_POW     0
#define RD0_RC01_MAX_TX_POW_A   12
#define RD0_RC01_MAX_TX_POW_B   12
#define RD0_RC01_MAX_TX_POW_N   12

/* Regulatory Domain #0 - Regulatory Class #02 */
#define RD0_RC02_REG_MAX_TX_POW 20
#define RD0_RC02_MIN_TX_POW     0
#define RD0_RC02_MAX_TX_POW_A   12
#define RD0_RC02_MAX_TX_POW_B   12
#define RD0_RC02_MAX_TX_POW_N   12

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define MAX_SUPP_CHANNEL MAX(NUM_SUPP_CHANNEL_FREQ_5, NUM_SUPP_CHANNEL_FREQ_2)

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_reg_domain_generic(void);

#endif /* GENERIC_RF */

#endif /* REG_DOMAIN_GENERIC_H */
