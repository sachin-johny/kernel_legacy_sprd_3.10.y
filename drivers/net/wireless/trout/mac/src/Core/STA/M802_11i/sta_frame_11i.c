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
/*  File Name         : sta_frame_11i.c                                      */
/*                                                                           */
/*  Description       : This file contains all the functions for the         */
/*                      preparation of the various MAC frames specific to    */
/*                      station mode of operation in 802.11i.                */
/*                                                                           */
/*  List of Functions : set_rsn_element_sta                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management_11i.h"
#include "mib.h"
#include "mib_11i.h"
#include "rsna_km.h"
#include "sta_frame_11i.h"
#include "ieee_11i_sta.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_rsn_element_sta                                      */
/*                                                                           */
/*  Description   : This function sets the RSN Information element           */
/*                  in the given frame for the Station mode                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the RSN Information Element                  */
/*                  3) 802.11i Mode                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The RSN Information Elements are obtained from the MIB   */
/*                  and the fields are set in the given data frame, at the   */
/*                  given index.                                             */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the RSN Information Element element.              */
/*                                                                           */
/*  Returns       : RSN Information element length.                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_rsn_element_sta(UWORD8* data, UWORD16 index, UWORD8 mode_802_11i)
{
    UWORD16 index1                   = 0;
    UWORD8 num_pairwise_suites       = 0;
    UWORD8 num_authentication_suites = 0;
    UWORD16 rsn_capabilities         = 0;
    UWORD16 num_pmkid                = 0;
    UWORD8 *oui_ptr                  = NULL;

    /* If 802.11i is not enabled, donot append the element to the packet */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
        return 0;
    }

    /* Obtaining the number of the pairwise cipher suites implemented */
    num_pairwise_suites       = get_pairwise_cipher_suite();

    /* If no suites is implemented or none of the implemented suite is */
    /* enabled, the RSN Information Element is not added               */
    if(num_pairwise_suites == 0)
    {
        return 0;
    }

    /* Obtaining the number of the authentication suites implemented */
    num_authentication_suites = get_authentication_suite();

    /* If no suites is implemented or none of the implemented suite is */
    /* enabled, the RSN Information Element is not added               */
    if(num_authentication_suites == 0)
    {
        return 0;
    }

    /*************************************************************************/
    /*                  Extended Supported Rates Element Format              */
    /* --------------------------------------------------------------------- */
    /* |Element ID | Length | Version | Group Cipher Suite | Pairwise Cipher */
    /* --------------------------------------------------------------------- */
    /* | 1         | 1      | 2       |      4             |     2           */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* Suite | Pairwise Cipher Suite List | AKM Suite Count | AKM Suite List */
    /* --------------------------------------------------------------------- */
    /*       | 4-m                        |     2          | 4-n             */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* | RSN Capabilities | PMKID Count | PMKID List                       | */
    /* --------------------------------------------------------------------- */
    /* |    2             |    2        | 16 -s                            | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index1 = index + 1;


    /* Set RSN Information Element element ID */
    if(mode_802_11i == RSNA_802_11I)
    {
        /* Fetch the OUI PTR */
        oui_ptr = mget_RSNAOUI();

        data[index] = IRSNELEMENT;
        index += IE_HDR_LEN;
    }
    else if(mode_802_11i == WPA_802_11I)
    {
        /* Fetch the OUI PTR */
        oui_ptr = mget_WPAOUI();

        data[index] = IWPAELEMENT;
        index += IE_HDR_LEN;

        memcpy(&data[index], oui_ptr, 3);
        index += 3;

        data[index++] = OUITYPE_WPA;
    }

    /* Set the version of RSN Element to 1 */
    data[index++] = 1;
    data[index++] = 0;

    /* Set Group Cipher Suite from the MIB */
    /* The IEEE802.11I standard Annex D Specifies that this element shall be */
    /* taken from the dot11RSNAConfigGroupCipher parameter of the MIB        */
    memcpy(&data[index], oui_ptr, 3);
    index += 3;
    data[index++] = mget_RSNAConfigGroupCipher();

    /* Set Pairwise cipher Suite */
    data[index++] = 1;
    data[index++] = 0;

    /* Set Pairwise Cipher Suite List */
    memcpy(&data[index], oui_ptr, 3);
    index += 3;
    data[index++] = mget_RSNAPairwiseCipherRequested();

    /* Set Authentication Suite */
    data[index++] = 1;
    data[index++] = 0;

    /* Set Pairwise Cipher Suite List */
    memcpy(&data[index], oui_ptr, 3);
    index += 3;
    data[index++] = mget_RSNAAuthenticationSuiteRequested();

    if(mode_802_11i == RSNA_802_11I)
    {

        /* The RSN Capabilities, Replay counters (PTKSA and GTKSA) and PMKID */
        /* Lists are deprecated in WPA and are valid only in RSNA            */

    /* Set RSN Capabilities Information */
    /*************************************************************************/
    /* --------------------------------------------------------------------- */
    /* | B15 - B6  |  B5 - B4      | B3 - B2     |       B1    |     B0    | */
    /* --------------------------------------------------------------------- */
    /* | Reserved  |  GTSKA Replay | PTSKA Replay| No Pairwise | Pre - Auth| */
    /* |           |    Counter    |   Counter   |             |           | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/
        rsn_capabilities = 0;

        /* Set No Pairwise field in the Capability field */
        if(mget_DesiredBSSType() == INFRASTRUCTURE)
        {
            /* TBD - This is implemented after WEP is enabled */
            //rsn_capabilities |= BIT1;
        }

        /* Set PTSKA Replay counters. This field is taken from the MIB's */
        /* dot11RSNAConfigNumberOfPTSKAReplayCounters                    */
        rsn_capabilities |=
            ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2 ) & 0x000C );

        /* Set PTSKA Replay counters. This field is taken from the MIB's */
        /* dot11RSNAConfigNumberOfGTSKAReplayCounters                    */
        rsn_capabilities |=
            ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4 ) & 0x0030 );


        /* Set RSN Capabilities in the RSN Information Element */
        data[index++] = rsn_capabilities & 0x00FF;
        data[index++] = (rsn_capabilities & 0xFF00) >> 8;


        /* Set PMKID Field */
        /* This field is present only in (Re)Assoc request sent to the AP */
        if((mget_DesiredBSSType() == INFRASTRUCTURE) &&
            ((get_sub_type(data) == ASSOC_REQ) ||
            (get_sub_type(data) == REASSOC_REQ)))
        {
            num_pmkid = get_cached_pmkids(mget_bssid(), (data+index+2));

            if(num_pmkid > 0)
            {
                data[index++] = (num_pmkid & 0x00FF);
                data[index++] = (num_pmkid & 0xFF00) >> 8;
                index += num_pmkid * PMKID_LEN;
            }
        }

    }

    /* Set the length of the RSN Information Element */
    data[index1] = (index - index1 - 1);

    /* Return the Extended Supported Rates element length */
    return 1 + (index - index1);
}

#endif /* MAC_802_11I */

#endif /* IBSS_BSS_STATION_MODE */
