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
/*  File Name         : ieee_1x.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      IEEE 802.1x-REV/D9.                                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef IEEE_1X_H
#define IEEE_1X_H

/*****************************************************************************/
/* Include Files                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define AAA_KEY_MAX_LEN 64

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Authenticator PAE states */
typedef enum {AS_INITIALIZE     = 0,
              AS_DISCONNECTED   = 1,
              AS_CONNECTING     = 2,
              AS_AUTHENTICATING = 3,
              AS_AUTHENTICATED  = 4,
              AS_ABORTING       = 5,
              AS_HELD           = 6,
              AS_FORCE_AUTH     = 7,
              AS_FORCE_UNAUTH   = 8
} AUTH_STATE_T;

/* Key Receive states */
typedef enum {KR_NO_KEY_RECEIVE = 0,
              KR_KEY_RECEIVE    = 1
} KEY_RX_STATE_T;

/* Reauthentication Timer states */
typedef enum {RA_INITIALIZE     = 0,
              RA_REAUTHENTICATE = 1
} REAUTH_STATE_T;

/* Backend Authentication states */
typedef enum {ABS_REQUEST       = 0,
              ABS_RESPONSE      = 1,
              ABS_SUCCESS       = 2,
              ABS_FAIL          = 3,
              ABS_TIMEOUT       = 4,
              ABS_IDLE          = 5,
              ABS_INITIALIZE    = 6,

} AUTH_BKND_STATE_T;

/* Controlled Directions states */
typedef enum {CD_FORCE_BOTH     = 0,
              CD_IN_OR_BOTH     = 1
} CONTROL_DIR_STATE_T;

/* Port status */
typedef enum {UNAUTHORIZED = 0,
              AUTHORIZED   = 1
} X_AUTH_STATUS_T;

/* Port control values */
typedef enum {FORCE_UNAUTHORIZED = 0,
              FORCE_AUTHORIZED   = 1,
              AUTO               = 2
} X_PORT_STATE_T;

/* Controlled directions values */
typedef enum {BOTH = 0,
              IN   = 1
} X_CONTROL_DIR_T;


#endif /* IEEE_1X_H */

#endif /* MAC_802_11I */
