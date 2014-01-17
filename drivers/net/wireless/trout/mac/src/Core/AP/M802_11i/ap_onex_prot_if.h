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
/*  File Name         : ap_onex_prot_if.h                                    */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AP mode 802.1X                                       */
/*                                                                           */
/*  List of Functions : check_sec_auth_pkt_ap_1x                             */
/*                      start_sec_auth_fsm_ap                                */
/*                      sec_auth_eapol_ap                                    */
/*                      init_sec_auth_node_ap_1x                             */
/*                      free_sec_auth_node_ap_1x                             */
/*                      init_sec_auth_ap_1x                                  */
/*                      stop_sec_auth_node_ap_1x                             */
/*                      stop_sec_daemon_ap_1x                                */
/*                      start_sec_daemon_ap_1x                               */
/*                      set_1x_radius_key_ap                                 */
/*                      get_1x_radius_key_ap                                 */
/*                      set_1x_radius_serv_addr_ap                           */
/*                      get_1x_radius_serv_addr_ap                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I

#ifndef AP_ONEX_PROT_IF_H
#define AP_ONEX_PROT_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "cglobals_ap.h"
#include "prot_if.h"
#include "rsna_auth_km.h"
#include "auth_frame_11i.h"
#include "mib_11i.h"
#include "ieee_auth_1x.h"

#ifdef MAC_802_1X
#include "radius_client.h"
#endif /* MAC_802_1X */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Handle the authentication/arp packets */
INLINE BOOL_T check_sec_auth_pkt_ap_1x(UWORD8 *buffer, UWORD32 len)
{
    BOOL_T ret_val = BFALSE;

#ifdef MAC_802_1X
	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Return from here if 1x is not enabled */
    if(check_auth_policy(0x01) == BFALSE)
    {
    	return ret_val;
	}

    if(is_arp_pkt(buffer) == BTRUE)
    {
        /* Process the incoming host configuration packet */
        send_arp_reply(buffer, len);
        ret_val = BTRUE;
    }
    else if(is_radius_pkt(buffer) == BTRUE)
    {
        /* Process the incoming host configuration packet */
        process_host_rx_radius(buffer, len);
        ret_val = BTRUE;
    }
#endif /* MAC_802_1X */

    return ret_val;
}

/* Start the Security Authentication FSM */
INLINE void start_sec_auth_fsm_ap(auth_t *sta_1x)
{
#ifdef MAC_802_1X
    /* If PSK is disabled, start 802.1X state machine  */
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	start_fsm_ap_1x(sta_1x);
	}
#endif /* MAC_802_1X */
}


/* Process the Security Authentication EAPOL Packet */
INLINE void sec_auth_eapol_ap(UWORD8 *rx_data, UWORD16 len, auth_t *sta_1x)
{
#ifdef MAC_802_1X
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	auth_onex_eapol(rx_data, len, sta_1x);
	}
#endif /* MAC_802_1X */
}

INLINE BOOL_T init_sec_auth_node_ap_1x(void **auth_1x, UWORD8 *rsna,
                                       UWORD16 aid, void *auth_1x_buff)
{
    BOOL_T retval = BFALSE;

#ifdef MAC_802_1X
    /* For 1x Authentication, 1X initialization is required */
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	retval = init_auth_node_1x((auth_t **)auth_1x, rsna, aid, (auth_t *)auth_1x_buff);
	}
#endif /* MAC_802_1X */

    return retval;
}


INLINE void free_sec_auth_node_ap_1x(void **auth_1x)
{
#ifdef MAC_802_1X
    /* Free the allocated memory from local pool for the 1X Handle */
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	free_auth_node_1x((auth_t **)auth_1x);
	}
#endif /* MAC_802_1X */
}

/* Initialize 802.1x related security */
INLINE void init_sec_auth_ap_1x(void)
{
#ifdef MAC_802_1X
    if(check_auth_policy(0x01) == BTRUE)
    {
        /* If 8021X is enabled, intialize the 802.1x FSM, EAP-FSM */
        init_ap_1x();
    }
#endif /* MAC_802_1X */
}

/* Initialize 802.1x related security */
INLINE void stop_sec_auth_node_ap_1x(void)
{
#ifdef MAC_802_1X
    if(check_auth_policy(0x01) == BTRUE)
    {
        /* If 8021X is enabled, intialize the 802.1x FSM, EAP-FSM */
        stop_ap_1x();
    }
#endif /* MAC_802_1X */
}

/* Stops 802.1x related daemon */
INLINE void stop_sec_daemon_ap_1x(void)
{
#ifdef MAC_802_1X
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	halt_radius_thread();
	}
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
#endif /* MAC_802_1X */
}

/* Starts 802.1x daemon */
INLINE void start_sec_daemon_ap_1x(void)
{
#ifdef MAC_802_1X
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(check_auth_policy(0x01) == BTRUE)
    {
    	start_radius_client();
	}
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
#endif /* MAC_802_1X */
}

/* This function sets the Radius Key used to communicate with radius server */
INLINE void set_1x_radius_key_ap(UWORD8* val)
{
#ifdef MAC_802_1X
    UWORD8 len = strlen((WORD8 *)val);
    len = (len > RAD_KEY_MAX_LEN) ? RAD_KEY_MAX_LEN : len;
    memcpy(&g_radius_key[1], val, len);
    g_radius_key[0] = len;
#endif /* MAC_802_1X */
}

/* This function gets the Radius Key used to communicate with radius server */
INLINE UWORD8* get_1x_radius_key_ap(void)
{
    UWORD8 *retval = NULL;
#ifdef MAC_802_1X
    retval = g_radius_key;
#endif /* MAC_802_1X */

    return retval;
}

/* This function sets the Radius Server Address */
INLINE void set_1x_radius_serv_addr_ap(UWORD32 val)
{
#ifdef MAC_802_1X
    g_radius_serv_addr[0] = (val & 0xFF000000) >> 24;
    g_radius_serv_addr[1] = (val & 0x00FF0000) >> 16;
    g_radius_serv_addr[2] = (val & 0x0000FF00) >> 8;
    g_radius_serv_addr[3] = val & 0x000000FF;
#endif /* MAC_802_1X */
}

/* This function sets the Radius Server Address */
INLINE UWORD32 get_1x_radius_serv_addr_ap(void)
{
    UWORD32 temp = 0;

#ifdef MAC_802_1X
    temp = g_radius_serv_addr[3] |
           ((g_radius_serv_addr[2] << 8) & 0x0000FF00) |
           ((g_radius_serv_addr[1] << 16) & 0x00FF0000) |
           ((g_radius_serv_addr[0] << 24) & 0xFF000000);
#endif /* MAC_802_1X */

    return temp;
}


#endif /* AP_ONEX_PROT_IF_H */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
