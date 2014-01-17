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
/*  File Name         : frame.c                                              */
/*                                                                           */
/*  Description       : This file contains the functions for setting and     */
/*                      preparing various MAC frames fields.                 */
/*                                                                           */
/*  List of Functions : set_ssid_element                                     */
/*                      set_sup_rates_element                                */
/*                      set_exsup_rates_element                              */
/*                      get_frame_class                                      */
/*                      prepare_deauth                                       */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "iconfig.h"
#include "management.h"
#include "mib.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_ssid_element                                         */
/*                                                                           */
/*  Description   : This function sets the SSID element (element ID, length  */
/*                  SSID) in the given frame.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the SSID element.                            */
/*                  3) Pointer to the SSID                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The SSID is obtained from the MIB and the fields of the  */
/*                  SSID element are set in the given data frame, at the     */
/*                  given index.                                             */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the SSID element.                                 */
/*                                                                           */
/*  Returns       : SSID element length.                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_ssid_element(UWORD8* data, UWORD16 index, WORD8 *ssid)
{
    UWORD8 len   = 0;

    /*************************************************************************/
    /*                           SSID Element Format                         */
    /* --------------------------------------------------------------        */
    /* |SSID Element ID | SSID Length   | SSID Value                |        */
    /* --------------------------------------------------------------        */
    /* | 1              | 1             | SSID length               |        */
    /* --------------------------------------------------------------        */
    /*                                                                       */
    /*************************************************************************/

    len  = strlen(ssid);

    data[index] = ISSID;

    /* Send the SSID depending on Broadcast SSID support option */
    data[index + 1] = len;
    memcpy(&data[index + 2], (UWORD8 *)ssid, len);

    /* Return the SSID element length */
    return IE_HDR_LEN + len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_bcast_ssid                                           */
/*                                                                           */
/*  Description   : This function sets the SSID element (element ID, length  */
/*                  SSID) to broad cast SSID                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the SSID element.                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The SSID len is set to 0 for broadcast ssid              */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the SSID element.                                 */
/*                                                                           */
/*  Returns       : SSID element length.                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 set_bcast_ssid(UWORD8* data, UWORD16 index)
{
    /*************************************************************************/
    /*                           SSID Element Format                         */
    /* --------------------------------------------------------------        */
    /* |SSID Element ID | SSID Length   | SSID Value                |        */
    /* --------------------------------------------------------------        */
    /* | 1              | 1             | SSID length               |        */
    /* --------------------------------------------------------------        */
    /*                                                                       */
    /*************************************************************************/
    data[index] = ISSID;

    /* Send the SSID depending on Broadcast SSID support option */
    data[index + 1] = 0;

    /* Return the SSID element length */
    return IE_HDR_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_sup_rates_element                                    */
/*                                                                           */
/*  Description   : This function sets the Supported Rates element (element  */
/*                  ID, length, Rates) in the given frame.                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the Supported Rates element.                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The Supported Rates are obtained from the MIB and the    */
/*                  fields of the Supported Rates element are set in the     */
/*                  given data frame, at the given index.                    */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the Supported Rates element.                      */
/*                                                                           */
/*  Returns       : Supported Rates element length.                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_sup_rates_element(UWORD8* data, UWORD16 index)
{
    UWORD8 j                   = 0;
    UWORD8 k                   = 0;
    UWORD8 num_non_basic_rates = 0;
    UWORD8 num_basic_rates     = 0;

    /*************************************************************************/
    /*                    Supported Rates Element Format                     */
    /* --------------------------------------------------------------------- */
    /* |SupRates Element ID | Number of SupRates (Max 8) | SupRates        | */
    /* --------------------------------------------------------------------- */
    /* | 1                  | 1                          | NumSupRates     | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set the Supported Rates element ID to the given frame at given index  */
    data[index] = ISUPRATES;

    /* Extract the number of basic and non-basic rates and set the number of */
    /* supported rates field based on this. Maximum value allowed is 8. All  */
    /* other rates (if any) are set in the extended rates field.             */
    num_non_basic_rates = get_num_non_basic_rates();
    num_basic_rates     = get_num_basic_rates();

    k = num_non_basic_rates + num_basic_rates;

    if(k > MAX_SUPRATES)
        k = MAX_SUPRATES;

    /* Set the maximum number of supported rates in the element length */
    data[index + 1] = k;

    /* Set the supported rates */
    for(j = 0; j < k; j++)
    {
        if(j < num_basic_rates)
            data[index + 2 + j] = get_mac_basic_rate(j);
        else
            data[index + 2 + j] =
                        get_mac_non_basic_rate((UWORD8)(j - num_basic_rates));
    }

    /* Return the Supported Rates element length */
    return IE_HDR_LEN + k;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_exsup_rates_element                                  */
/*                                                                           */
/*  Description   : This function sets the Extended Supported Rates element  */
/*                  (element ID, length, Rates) in the given frame.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the Extended Supported Rates element.        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The Extended Supported Rates are obtained from the MIB   */
/*                  and the fields are set in the given data frame, at the   */
/*                  given index.                                             */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the Extended Supported Rates element. If the      */
/*                  number of operational rates does not exceed the maximum  */
/*                  number of rates that can be included in the Supported    */
/*                  Rates element, this function simply returns without      */
/*                  doing anything.                                          */
/*                                                                           */
/*  Returns       : Extended Supported Rates element length.                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_exsup_rates_element(UWORD8* data, UWORD16 index)
{
    UWORD8 j                   = 0;
    UWORD8 k                   = 0;
    UWORD8 num_non_basic_rates = 0;
    UWORD8 num_basic_rates     = 0;
    UWORD8 num_exop_rates      = 0;
    UWORD8 val                 = 0;

    /*************************************************************************/
    /*                  Extended Supported Rates Element Format              */
    /* --------------------------------------------------------------------- */
    /* |ExSupRates Element ID | Number of ExSupRates  | ExSupRates         | */
    /* --------------------------------------------------------------------- */
    /* | 1                    | 1                     | NumExSupRates      | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Extract the number of basic and non-basic rates. If the number of     */
    /* rates exceed the maximum number of rates set this element.            */
    num_non_basic_rates = get_num_non_basic_rates();
    num_basic_rates     = get_num_basic_rates();

    k = num_non_basic_rates + num_basic_rates;

    /* If the number of operational rates does not exceed the maximum number */
    /* of rates this field is not required and the function returns 0.       */
    if(k <= MAX_SUPRATES)
        return 0;

    /* Set Extended Supported Rates element ID */
    data[index] = IEXSUPRATES;

    num_exop_rates = k - MAX_SUPRATES;

    /* Set the Extended Supported Rates */
    for(j = 0, k = MAX_SUPRATES; j < num_exop_rates; j++, k++)
    {
        if(k < num_basic_rates)
            data[index + 2 + j] = get_mac_basic_rate(k);
        else
            data[index + 2 + j] =
                        get_mac_non_basic_rate((UWORD8)(k - num_basic_rates));
    }


    /* Get the BSS Membership selector value and set the same in the frame   */
    /* if non-zero                                                           */
    val = get_bss_mship_sel_val();
    if(val != 0)
    {
        data[index + 2 + j] = val;
        j++;
    }

    /* Set the number of extended supported rates in the element length */
    data[index + 1] = j;

    /* Return the Extended Supported Rates element length */
    return IE_HDR_LEN + j;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_frame_class                                          */
/*                                                                           */
/*  Description   : This function extracts the class of received frame       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the class of received frame as    */
/*                  defined in 802.11.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Returns the value of the class of the received frame     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

FRAMECLASS_T get_frame_class(UWORD8* header)
{
    FRAMECLASS_T ret_val = CLASS1_FRAME_TYPE;

    switch((TYPESUBTYPE_T)(header[0] & 0xFC))
    {
    case RTS:
    case CTS:
    case ACK:
    case CFEND_ACK:
    case CFEND:
    case PROBE_REQ:
    case PROBE_RSP:
    case BEACON:
    case ATIM:
    case AUTH:
    case DEAUTH:
    case ACTION:
    {
        ret_val = CLASS1_FRAME_TYPE;
    }
    break;
    case ASSOC_REQ:
    case ASSOC_RSP:
    case REASSOC_REQ:
    case REASSOC_RSP:
    case DISASOC:
    {
        ret_val = CLASS2_FRAME_TYPE;
    }
    break;
    case PS_POLL:
    case CFPOLL:
    case CFPOLL_ACK:
    {
        ret_val = CLASS3_FRAME_TYPE;
    }
    break;
    case DATA:
    case DATA_ACK:
    case DATA_POLL:
    case DATA_POLL_ACK:
    case NULL_FRAME:
    {
        if((get_to_ds(header) == 0) && (get_from_ds(header) == 0))
        {
            ret_val = CLASS1_FRAME_TYPE;
        }
        else
        {
            ret_val = CLASS3_FRAME_TYPE;
        }
    }
    break;
    default:
    {
        ret_val = (FRAMECLASS_T)get_prot_frame_class(header);
    }
    break;
    }
    return ret_val;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_erp_rates                                          */
/*                                                                           */
/*  Description   : Check ERP rates in the incoming request                  */
/*                                                                           */
/*  Inputs        : 1) The pointer to the rate table                         */
/*                  2) Pointer to ERP Information element                    */
/*                  3) Index into incoming frame                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Extract the ERP rates and update rate table              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Number of ERP rates specied in the incoming frame        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 check_erp_rates(UWORD8 *entry, UWORD8 *data, UWORD16 index)
{
    /* This function should check all these values only if its 11g PHY */
    if((get_phy_type() != PHY_B_DSSS) && (get_phy_type() != PHY_A_OFDM))
    {

        UWORD32 num_of_erp_rates = 0;
        UWORD32 i                = 0;
        rate_t  *op_rates = (rate_t *)entry;

        if(data[index] == IEXSUPRATES)
        {
            num_of_erp_rates = data[index + 1];
            index += IE_HDR_LEN;

            if(op_rates != 0)
            {
                for(i = 0; i < num_of_erp_rates; i++)
                {
                    op_rates->rates[op_rates->num_rates + i] =
                                data[index + i] & 0x7F;
                }
            }
            else
            {
                /* Do Nothing */
            }
        }

        return num_of_erp_rates;
    }
    return 0;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_deauth                                           */
/*                                                                           */
/*  Description   : This function prepares the deauthentication frame on     */
/*                  receiving a Class 2 frame from an unauthenticated        */
/*                  station.                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the deauthentication frame                 */
/*                  2) Pointer to the incoming Class 2 frame                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the deauthentication frame are     */
/*                  set as specified in the standard.                        */
/*                                                                           */
/*  Outputs       : The contents of  the given deauthentication frame        */
/*                  have been set.                                           */
/*                                                                           */
/*  Returns       : Deauthentication frame length.                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_deauth(UWORD8* data, UWORD8* da, UWORD16 reason_code)
{
    UWORD16 deauth_len = 0;
	TROUT_FUNC_ENTER;
    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)DEAUTH);

    /* Set DA to address of unauthenticated STA */
    set_address1(data, da);

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                  Deauthentication Frame - Frame Body                  */
    /* --------------------------------------------------------------------- */
    /* |                           Reason Code                             | */
    /* --------------------------------------------------------------------- */
    /* |2 Byte                                                             | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Set Reason Code to 'Class2 error' */
    data[MAC_HDR_LEN]     = (reason_code & 0x00FF);
    data[MAC_HDR_LEN + 1] = (reason_code & 0xFF00) >> 8;

    deauth_len = MAC_HDR_LEN + REASON_CODE_LEN + FCS_LEN;
	TROUT_FUNC_EXIT;
    return deauth_len;
}
