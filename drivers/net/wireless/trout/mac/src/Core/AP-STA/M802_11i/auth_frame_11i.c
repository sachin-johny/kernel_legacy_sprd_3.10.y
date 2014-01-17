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
/*  File Name         : auth_frame_802_11i.c                                 */
/*                                                                           */
/*  Description       : This file contains all the functions for the         */
/*                      preparation of the various MAC frames specific to    */
/*                      AP mode of operation in 802.11i.                     */
/*                                                                           */
/*  List of Functions : set_rsn_element_auth                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "auth_frame_11i.h"
#include "management_11i.h"
#include "mib_11i.h"
#include "rsna_auth_km.h"

#ifdef AUTH_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_rsn_element_auth                                     */
/*                                                                           */
/*  Description   : This function sets the RSN Information element           */
/*                  in the given frame for the Access point mode             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the RSN Information Element                  */
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

UWORD8 set_rsn_element_auth(UWORD8* data, UWORD16 index, UWORD8 mode_802_11i)
{
    UWORD8 index1              = 0;
    UWORD8 k                   = 0;
    UWORD8 num_pairwise_suites = 0;
    UWORD8 num_authentication_suites = 0;
    UWORD16 rsn_capabilities         = 0;
    UWORD8 *oui_ptr            = NULL;

    /* If 802.11i is not enabled, donot append the element to the packet */
    if(mget_RSNAEnabled() == TV_FALSE)
    {
            return 0;
    }

    /* Obtaining the number of the pairwise cipher suites implemented */
    num_pairwise_suites       = get_pairwise_cipher_suite();

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

    if(num_pairwise_suites == 0)
    {
        /* Set Pairwise cipher Suite */
        data[index++] = 1;
        data[index++] = 0;

        memcpy(&data[index], oui_ptr, 3);
        index += 3;
        data[index++] = 0;

    }
    else
    {
        /* Set Pairwise cipher Suite */
        data[index++] = num_pairwise_suites;
        data[index++] = 0;

        /* If TSN is used, Use Group Cipher as PW Cipher suite is advertised */
        /* Else all the enabled PW cihpers are advertised                    */
        if(num_pairwise_suites != 0)
        {
            UWORD8 pw_count = 0;
            UWORD8 pcip[3]  = {0};

            /* Set Pairwise Cipher Suite List */
            for(k = 0; k < PAIRWISE_CIPHER_SUITES; k++)
            {
                /* If the Entry in the MIB is enabled, Append entry to RSN */
                if(MIB_11I.dot11RSNAConfigPairwiseCiphersEntry[k].
                   dot11RSNAConfigPairwiseCipherEnabled == TV_TRUE)
                {
                    pcip[pw_count++] = mget_RSNAConfigPairwiseCipher(k);
                }
            }

            /* Sort the enabled PCIPs for the current mode in use */
            sort_pcip_4_mode(mode_802_11i, pcip, pw_count);

            for(k = 0; k < pw_count; k++)
            {
                memcpy(&data[index], oui_ptr, 3);
                index += 3;
                data[index++] = pcip[k];
            }
        }
    }

    /* Set Authentication Suite */
    data[index++] = num_authentication_suites;
    data[index++] = 0;

    /* Set Pairwise Cipher Suite List */
    for(k = 0; k < AUTHENTICATION_SUITES; k++)
    {
        /* If the Entry in the MIB is enabled, Append the entry to the RSN */
        if(MIB_11I.dot11RSNAConfigAuthenticationSuitesEntry[k].
            dot11RSNAConfigAuthenticationSuiteEnabled == TV_TRUE)
        {
            memcpy(&data[index], oui_ptr, 3);
            index += 3;
            data[index++] = mget_RSNAConfigAuthenticationSuite(k);
        }
    }


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

        /* Set Pre-Auth in AP Mode if implemented and enabled, 0 in all other*/
        /* modes                                                             */
        if((mget_RSNAPreauthenticationImplemented() == TV_TRUE) &&
            (mget_RSNAPreauthenticationEnabled() == TV_TRUE))
        {
            rsn_capabilities |= BIT0;
        }

        /* Set PTSKA Replay counters. This field is taken from the MIB's */
        /* dot11RSNAConfigNumberOfPTSKAReplayCounters                    */
        rsn_capabilities +=
            ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2 ) & 0x000C );

        /* Set PTSKA Replay counters. This field is taken from the MIB's */
        /* dot11RSNAConfigNumberOfGTSKAReplayCounters                    */
        rsn_capabilities +=
            ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4 ) & 0x0030 );


        /* Set RSN Capabilities in the RSN Information Element */
        data[index++] = rsn_capabilities & 0x00FF;
        data[index++] = (rsn_capabilities & 0xFF00) >> 8;
    }

    /* Set the length of the RSN Information Element */
    data[index1] = (index - index1 - 1);

    /* Return the Extended Supported Rates element length */
    return 1 + (index - index1);
}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
