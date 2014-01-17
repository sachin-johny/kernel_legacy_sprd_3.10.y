/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : frame_11h_ap.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the MAC frames specific to 11h in AP mode.           */
/*                                                                           */
/*  List of Functions : set_power_constraint_info_field                      */
/*                      set_tpc_report_info_field                            */
/*                      set_country_info_field                               */
/*                      set_spectrum_mgmt_params_ap                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11H

#ifndef FRAME_11H_AP_H
#define FRAME_11H_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib_11h.h"
#include "management_11h.h"
#include "cglobals.h"
#include "cglobals_ap.h"
#include "phy_hw_if.h"
#include "rf_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_COUNTRY_IE_LEN 255

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

//extern UWORD8  g_country_ie_len; // 20120830 caisf masked, merged ittiam mac v1.3 code
extern UWORD16 g_csa_ie_index;
//extern UWORD8  g_country_ie[MAX_COUNTRY_IE_LEN]; // 20120830 caisf masked, merged ittiam mac v1.3 code

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

//extern void set_default_country_info_element(void); // 20120830 caisf masked, merged ittiam mac v1.3 code
extern void ap_enabled_11h_rx_action(UWORD8 *msg);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function updates the Power Constraint Information Element */
INLINE void set_power_constraint_info_field(UWORD8 *ie, UWORD8 index)
{
    /*  Power Constraint Information Element Format     */
    /* ------------------------------------------------ */
    /* | Element ID | Length | Local Power Constraint | */
    /* ------------------------------------------------ */
    /* | 1          | 1      | 1                      | */
    /* ------------------------------------------------ */

    /* Element ID */
    ie[index] = IPOWERCONSTRAINT;

    /* Length */
    ie[index + 1] = IPOWERCONSTRAINT_LEN;

    /* Local Power Constraint */
    /* Note that this field is always set to 0 currently. Ideally            */
    /* this field can be updated by having an algorithm to decide transmit   */
    /* power to be used in the BSS by the AP.                                */
    ie[index + 2] = 0;
}

/* This function updates the TPC Report Information Element */
INLINE void set_tpc_report_info_field(UWORD8 *ie, UWORD8 index,
                                      WORD8 link_margin)
{
    /*  TPC Report Information Element Format                   */
    /* -------------------------------------------------------- */
    /* | Element ID | Length | Transmit Power   | Link Margin | */
    /* -------------------------------------------------------- */
    /* | 1          | 1      | 1                | 1           | */
    /* -------------------------------------------------------- */

    /* Element ID */
    ie[index] = ITPCREPORT;

    /* Length */
    ie[index + 1] = ITPCREPORT_LEN;

    /* Transmit Power */
	// 20120830 caisf mod, merged ittiam mac v1.3 code
    //ie[index + 2] = g_curr_tx_power.pow_a.dbm;
	ie[index + 2] = get_tpc_report_tx_pow(g_user_control_enabled);

    /* Link Margin */
    ie[index + 3] = link_margin;
}

// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
/* This function updates the Country Information Element from the saved      */
/* information element.                                                      */
INLINE UWORD8 set_country_info_field(UWORD8 *ie, UWORD8 index)
{
    memcpy(ie + index, g_country_ie, g_country_ie_len);
    return g_country_ie_len;
}
#endif

/* This function sets the spectrum management parameters in beacon and probe */
/* response frames in AP mode.                                               */
INLINE UWORD16 set_spectrum_mgmt_params_ap(UWORD8 *data, UWORD16 index)
{
    UWORD16 len = 0;

    if(mget_enableTPC() == BTRUE)
    {
		// 20120830 caisf masked, merged ittiam mac v1.3 code
		#if 0
        /* Country Info element */
        len += set_country_info_field(data, index);
		#endif

        /* Power Constraint Info element */
        set_power_constraint_info_field(data, index + len);
        len += (IPOWERCONSTRAINT_LEN + IE_HDR_LEN);

        /* TPC Report Info element. Note that the Link Margin field shall be */
        /* set to 0 and shall be ignored when a TPC Report element is        */
        /* included in a Beacon or Probe Response frame.                     */
        set_tpc_report_info_field(data, index + len, 0);
        len += (IE_HDR_LEN + ITPCREPORT_LEN);
    }

    return len;
}

/* Set spectrum management bit in the capability information field */
INLINE void set_spectrum_mgmt_bit_ap(UWORD8* data, UWORD16 index)
{
    /* Set spectrum management bit in the capability info of association */
    /* request frame if APUT supports dot11h features */
    if(mget_SpectrumManagementRequired() == TV_TRUE)
    {
        data[index+1] |= (SPECTRUM_MGMT>>8);
    }
}
#endif /* FRAME_11H_AP_H */

#endif /* MAC_802_11H */
#endif /* BSS_ACCESS_POINT_MODE */
