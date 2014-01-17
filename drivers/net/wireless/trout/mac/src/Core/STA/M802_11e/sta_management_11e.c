/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : sta_management_11e.c                                 */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for 802.11e station mode.                */
/*                                                                           */
/*  List of Functions : update_scan_response_wmm                             */
/*                      update_params_wmm_sta                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "sta_management_11e.h"
#include "index_util.h"
#include "sta_frame_11e.h"
#include "qif.h"
#include "sta_prot_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/* WMM capability flag of AP */
BOOL_T g_ap_wmm_capable = BFALSE;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_scan_response_wmm                                 */
/*                                                                           */
/*  Description   : This function updates the BSS descriptor set with the    */
/*                  WMM parameters displayed in frames received during scan. */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame                         */
/*                  2) Received frame length                                 */
/*                  3) Index of the information elements                     */
/*                  4) Index to the BSS-Dscr Table entry                     */
/*                                                                           */
/*  Globals       : g_bss_dscr_set                                           */
/*                                                                           */
/*  Processing    : This function parses the received frame for WMM          */
/*                  information elements and updates the BSS descriptor set  */
/*                  with the WMM parameters.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_scan_response_wmm(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                              UWORD8 dscr_set_index)
{
    /* Reset the WMM capability and UAPSD capability flags */
    g_bss_dscr_set[dscr_set_index].wmm_cap   = BFALSE;
    g_bss_dscr_set[dscr_set_index].uapsd_cap = BFALSE;

    /* Parse the frame for the WMM information element */
    while(index < (rx_len - FCS_LEN))
    {
        if(is_wmm_info_param_elem(msa + index) == BTRUE)
        {
            /* Presence of WMM Info/Param element indicates WMM capability */
            g_bss_dscr_set[dscr_set_index].wmm_cap = BTRUE;

            /* Check if Bit 7 is set indicating U-APSD capability */
            if(msa[index + 8] & BIT7)
            {
                g_bss_dscr_set[dscr_set_index].uapsd_cap = BTRUE;
            }

            return;
        }

        index += (2 + msa[index + 1]);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_params_wmm_sta                                    */
/*                                                                           */
/*  Description   : This function updates the EDCA parameters from received  */
/*                  frame.                                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame                         */
/*                  2) Received frame length                                 */
/*                  3) Index to the start of information elements            */
/*                  4) Received frame sub type                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The received frame is checked for the WMM parameter      */
/*                  element. If present the MIB and MAC H/w registers are    */
/*                  updated with the EDCA parameters extracted from this. In */
/*                  case the received frame is a beacon frame the updation   */
/*                  is required only if the EDCA parameter set update count  */
/*                  saved by the STA does not match the one set in the WMM   */
/*                  parameter element.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_params_wmm_sta(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                           UWORD8 sub_type)
{
    /* Check if the WMM Parameter element is present in the frame */
    while(index < (rx_len - FCS_LEN))
    {
        /*                  WMM Parameter Element Format                     */
        /* ----------------------------------------------------------------- */
        /* |OUI |OUI Type |OUI Subtype |Version |QoS Info |Resd |AC Params | */
        /* ----------------------------------------------------------------- */
        /* |3   |1        |1           |1       |1        |1    |16        | */
        /* ----------------------------------------------------------------- */
        if(is_wmm_param_elem(msa + index) == BTRUE)
        {
            UWORD8 set_count = 0;
            UWORD8 i         = 0;

            /* Increment the index to point to the QoS Info field */
            index += 8;

            /* Read the EDCA parameter set count value from the frame */
            set_count = msa[index] & 0x0F;

            /* If the received frame is a beacon check the EDCA parameter    */
            /* set update count before updation. If the count saved at  STA  */
            /* matches the current set count sent by the AP, no updation is  */
            /* required. Do nothing and return.                              */
            if((sub_type == BEACON) &&
               (mget_EDCAParameterSetUpdateCount() == set_count))
                return;

            /* Increment the index to point to the AC Params field */
            index += 2;

            /* Update EDCA parameters for all the access categories */
            for(i = 0; i < NUM_AC; i++)
            {
                update_ac_param_record_sta(&msa[index]);
                index += AC_PARAM_REC_LEN;
            }

            /* Update the current EDCA parameter set update count to the new */
            /* value received.                                               */
            mset_EDCAParameterSetUpdateCount(set_count);

            /* Update the MAC H/w registers for the same */
            update_edca_machw();

#ifdef AUTORATE_FEATURE
            /* Re-initialize the transmit rate based on the updated EDCA */
            /* parameters.                                               */
            update_per_entry_rate_idx();
#endif /* AUTORATE_FEATURE */

            return;
        }

        index += (2 + msa[index + 1]);
    }

	/* Disable WMM if AP is not WMM capable */
    if(sub_type == ASSOC_RSP)
    {
        disable_machw_edca();
        set_wmm_enabled(BFALSE);

        /* Program default EDCA parameters for AC_VO queue when AP is not */
        /* WMM capable.                                                   */
        set_machw_aifsn();
        set_machw_cw(get_cwmax(), get_cwmin());
        set_machw_prng_seed_val(get_random_seed());
    }

}

#endif /* MAC_WMM */
#endif /* IBSS_BSS_STATION_MODE */
