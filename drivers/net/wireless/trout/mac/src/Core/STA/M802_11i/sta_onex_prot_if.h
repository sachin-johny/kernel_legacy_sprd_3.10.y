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
/*  File Name         : sta_onex_prot_if.h                                   */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      STA mode 802.1X                                      */
/*                                                                           */
/*  List of Functions : init_sec_auth_node_sta_1x                            */
/*                      free_sec_auth_node_sta_1x                            */
/*                      stop_sec_auth_node_sta_1x                            */
/*                      init_sec_supp_node_sta_1x                            */
/*                      free_sec_supp_node_sta_1x                            */
/*                      stop_sec_supp_node_sta_1x                            */
/*                      set_supp_username_sta_1x                             */
/*                      get_supp_username_sta_1x                             */
/*                      set_supp_password_sta_1x                             */
/*                      get_supp_password_sta_1x                             */
/*                      start_sec_supp_fsm_sta                               */
/*                      sec_supp_eapol_sta                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I

#ifndef STA_ONEX_PROT_IF_H
#define STA_ONEX_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_sta.h"
#include "common.h"
#include "prot_if.h"
#include "ieee_supp_1x.h"

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/


INLINE void init_sec_auth_sta_1x(void)
{
#ifdef MAC_802_1X
#endif /* MAC_802_1X */
}

INLINE void init_sec_supp_sta_1x(void)
{
#ifdef MAC_802_1X
    init_sta_1x();
#endif /* MAC_802_1X */
}

INLINE BOOL_T init_sec_auth_node_sta_1x(void **auth_1x, UWORD8 *rsna,
                                        UWORD16 aid, void *auth_1x_buff)
{
#ifdef MAC_802_1X
#endif /* MAC_802_1X */

    return BFALSE;
}


INLINE void free_sec_auth_node_sta_1x(void **sta_1x)
{
#ifdef MAC_802_1X
#endif /* MAC_802_1X */
}

/* Initialize 802.1x related security */
INLINE void stop_sec_auth_node_sta_1x(void)
{
#ifdef MAC_802_1X
#endif /* MAC_802_1X */
}

INLINE BOOL_T init_sec_supp_node_sta_1x(void **sta_1x, UWORD8 *rsna,
                                        UWORD16 aid, void *supp_1x_buff)
{
#ifdef MAC_802_1X
    return init_supp_node_1x((supp_t **)sta_1x, rsna, aid,
                             (supp_t *)supp_1x_buff);
#else /* MAC_802_1X */
    return BFALSE;
#endif /* MAC_802_1X */

}

INLINE void free_sec_supp_node_sta_1x(void **sta_1x)
{
#ifdef MAC_802_1X
    free_supp_node_1x((supp_t **)sta_1x);
#endif /* MAC_802_1X */
}

/* Initialize 802.1x related security */
INLINE void stop_sec_supp_node_sta_1x(void)
{
#ifdef MAC_802_1X
    if(check_auth_policy(0x01) == BTRUE)
    {
        /* If 8021X is enabled, intialize the 802.1x FSM, EAP-FSM */
        stop_sta_1x();
    }
#endif /* MAC_802_1X */
}

/* Set/Get Supplicant Username */
INLINE void set_supp_username_sta_1x(UWORD8 *val)
{
#ifdef MAC_802_1X
    UWORD8 len = strlen((const WORD8*)val);
    len = (len > MAX_SUPP_USERNAME_LEN) ? MAX_SUPP_USERNAME_LEN : len;
    memcpy(&g_supp_username[1], val, len);
    g_supp_username[0] = len;
#endif /* MAC_802_1X */
}

INLINE UWORD8 *get_supp_username_sta_1x(void)
{
#ifdef MAC_802_1X
    return g_supp_username;
#else /* MAC_802_1X */
    return NULL;
#endif /* MAC_802_1X */
}

/* Set/Get Supplicant Password */
INLINE void set_supp_password_sta_1x(UWORD8 *val)
{
#ifdef MAC_802_1X
    UWORD8 len = strlen((const WORD8 *)val);
    len = (len > MAX_SUPP_PASSWORD_LEN) ? MAX_SUPP_PASSWORD_LEN : len;
    memcpy(&g_supp_password[1], val, len);
    g_supp_password[0] = len;
#endif /* MAC_802_1X */
}

INLINE UWORD8 *get_supp_password_sta_1x(void)
{
#ifdef MAC_802_1X
    return g_supp_password;
#else /* MAC_802_1X */
    return NULL;
#endif /* MAC_802_1X */
}

/* Start the Security Authentication FSM */
INLINE void start_sec_supp_fsm_sta(supp_t *sta_1x)
{
#ifdef MAC_802_1X
    /* If PSK is disabled, start 802.1X state machine  */
    start_fsm_sta_1x(sta_1x);
#endif /* MAC_802_1X */
}

#ifdef SUPP_11I
/* Process the Security Authentication EAPOL Packet */
INLINE void sec_supp_eapol_sta(UWORD8 *rx_data, UWORD16 len, supp_t *sta_1x)
{
#ifdef MAC_802_1X
    supp_onex_eapol(rx_data, len, sta_1x);
#endif /* MAC_802_1X */
}
#endif /* SUPP_11I */

#endif /* STA_ONEX_PROT_IF_H */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
