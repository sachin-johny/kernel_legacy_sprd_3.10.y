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
/*  File Name         : iparams.h                                            */
/*                                                                           */
/*  Description       : This file contains all the compile-time parameter    */
/*                      definitions for MAC.                                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef IPARAMS_H
#define IPARAMS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* Maximum number of events supported for each type */
#define MAX_NUM_HOST_RX_EVENTS  60
#define MAX_NUM_WLAN_RX_EVENTS  60
#define MAX_NUM_MISC_EVENTS     60
#define MAX_NUM_HOST_DTX_EVENTS 60
#define MAX_NUM_HOST_CTX_EVENTS 10

#define MAX_NUM_HOST_SUPPORTED  4

/* TBD: This number is currently set arbitrarily. Correct estimation */
/* of the number is pending                                          */
#define NUM_SW_TIMER_SUPPORTED  100


#ifdef IBSS_BSS_STATION_MODE //chenq add a ifdef for sta mode 20 ap info 2013-02-05

#ifdef CONFIG_NUM_AP_SCAN
    #define NUM_STA_SUPPORTED   CONFIG_NUM_AP_SCAN
#else /* CONFIG_NUM_AP_SCAN */
    #define NUM_STA_SUPPORTED   8
#endif /* CONFIG_NUM_AP_SCAN */

#else

#ifdef CONFIG_NUM_STA
    #define NUM_STA_SUPPORTED   CONFIG_NUM_STA
#else /* CONFIG_NUM_STA */
    #define NUM_STA_SUPPORTED   8
#endif /* CONFIG_NUM_STA */

#endif

#define NUM_BA_SESSION_SUPPORTED 2   /* Maximum number of BA TX+RX Session */
#define DOT11H_REQ_CHUNKS        16

#endif /* IPARAMS_H */
