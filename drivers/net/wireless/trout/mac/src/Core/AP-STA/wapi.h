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
/*  File Name         : wapi.h                                                */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      WEP                                                  */
/*                                                                           */
/*  List of Functions : get_wep_type                                         */
/*                      char_2_hex                                           */
/*                      hex_2_char                                           */
/*                      str_2_hex                                            */
/*                      adjust_wep_headers                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef WAPI_H
#define WAPI_H

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"
#include "wapi_sms4.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
extern UWORD8 g_wapi_oui[3];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_wapi(void);

extern void update_join_req_params_wapi(void *ptr);
extern void update_scan_response_wapi(UWORD8 *msa, UWORD16 rx_len,
	                                          UWORD16 offset,UWORD8 dscr_set_index);
extern UWORD16 set_asoc_req_wapi_ie(UWORD8 *data, UWORD16 index);

extern UWORD16 wlan_rx_wapi_encryption(UWORD8 * header,
										     UWORD8 * data,UWORD16 data_len,
	                                         UWORD8 * ouput_buf);

extern UWORD16 wlan_tx_wapi_decryption(UWORD8 * input_ptk,
	                                         UWORD16 header_len,UWORD16 data_len,
	                                         UWORD8 * output_buf);

/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* Check whether the SNAP implies presence of WAPI header. */  
/* If yes, then merge the fragments into a single buffer.            */  
INLINE BOOL_T check_wapi_frame(UWORD8 *snap_hdr_ptr)  
{  
    /* Check SNAP header */  
    if((snap_hdr_ptr[6] == 0x88) && (snap_hdr_ptr[7] == 0xB4))  
        return BTRUE;  
   
    return BFALSE;  
}  


#endif

#endif /* WAPI_H */

