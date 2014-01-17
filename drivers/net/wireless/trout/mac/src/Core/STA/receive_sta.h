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
/*  File Name         : receive_sta.h                                        */
/*                                                                           */
/*  Description       : This file contains the MAC receive path related      */
/*                      functions for STA mode.                              */
/*                                                                           */
/*  List of Functions : check_source_address_sta                             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef RECEIVE_STA_H
#define RECEIVE_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "mib.h"
#include "receive.h"

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function checks if the received packet is sent and received by same  */
/* station, which is possible in case of broadcast packet transmission from  */
/* BSS STA.                                                                  */
INLINE BOOL_T check_source_address_sta(UWORD8 *sa)
{
    if(mac_addr_cmp(sa, mget_StationID()) == BTRUE)
    {
        return BTRUE;
    }
    return BFALSE;
}

/* This function updates the currently maintained RSSI value for the station */
INLINE void update_rssi(UWORD32 *rx_dscr)
{
    WORD8  rssi    = 0;

    rssi = get_rx_dscr_rssi_db(rx_dscr);

    if(rssi != INVALID_RSSI_DB)
        g_rssi = rssi;
}

/* This function updates the currently maintained snr value for the station */
INLINE void update_snr(UWORD32 *rx_dscr)
{
    g_snr = get_rx_dscr_snr(rx_dscr);
}


/* This function determines if the received non directed frame is useful  */
INLINE BOOL_T useful_non_directed_sta(mac_struct_t *mac, TYPESUBTYPE_T frame_st)
{
    BOOL_T retval = BFALSE;

    /* Allow non-directed probe response frames while in scan mode */
    /* This is to get maximum information regarding all networks   */
    if((get_mac_state() == WAIT_SCAN) && (frame_st == PROBE_RSP))
    {
        retval = BTRUE;
    }

    return retval;
}

#endif /* RECEIVE_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
