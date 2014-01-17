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
/*  File Name         : ieee_1x_md5_supp.h                                   */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      IEEE 802.1x-REV/D9 scaled down the STA mode          */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I

#ifndef IEEE_1X_MD5_SUPP_H
#define IEEE_1X_MD5_SUPP_H

/*****************************************************************************/
/* Include Files                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_eapmd5(void);
extern BOOL_T eapmd5_decode_packet(UWORD8 *eap_ptr, UWORD8 *out_ptr,
                                   UWORD16 *out_size, UWORD16 receivedId);
extern BOOL_T eapmd5_auth_setup(supp_t *supp_1x);
extern void eapmd5_shutdown(void);

#endif /* IEEE_1X_MD5_SUPP_H */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
