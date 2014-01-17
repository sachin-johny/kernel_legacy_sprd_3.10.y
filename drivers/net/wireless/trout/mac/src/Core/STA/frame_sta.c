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
/*  File Name         : frame_sta.c                                          */
/*                                                                           */
/*  Description       : This file contains the functions for the preparation */
/*                      of the various MAC frames specific to Station mode   */
/*                      of operation.                                        */
/*                                                                           */
/*  List of Functions : prepare_probe_req                                    */
/*                      prepare_beacon_sta                                   */
/*                      prepare_auth_req                                     */
/*                      prepare_auth_req_seq3                                */
/*                      prepare_asoc_req                                     */
/*                      prepare_probe_rsp_sta                                */
/*                      set_cap_info_field_sta                               */
/*                      get_tim_elm                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "prot_if.h"
#include "iconfig.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_probe_req                                        */
/*                                                                           */
/*  Description   : This function prepares the probe request frame.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe request frame                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the probe request frame are set    */
/*                  as specified in the standard.                            */
/*                                                                           */
/*  Outputs       : The contents of the given probe request frame have been  */
/*                  set.                                                     */
/*                                                                           */
/*  Returns       : Probe request frame length                               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_probe_req(UWORD8* data)
{
    UWORD16 index = 0;

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
    set_frame_control(data, (UWORD16)PROBE_REQ);

    /* Set fragment number of the outgoing packet to zero */
    set_fragment_number(data, 0);

    /* Set Address1 to broadcast address */
    set_address1(data, mget_bcst_addr());

    /* Set Address2 to dot11MacAddress */
    set_address2(data, mget_StationID());

    /* Set Address3 to Broadcast BSSID */
    set_address3(data, mget_bcst_addr());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                       Probe Request Frame - Frame Body                */
    /* --------------------------------------------------------------------- */
    /* |           SSID           |          Supported Rates               | */
    /* --------------------------------------------------------------------- */
    /* |2-34                      |3-10                                    | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index = MAC_HDR_LEN;

    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
    if((DEFAULT_SCAN == g_scan_source) &&
       (0 == get_bcst_ssid()) &&
       (BFALSE == is_p2p_grp_form_in_prog()))
    {
        index += set_ssid_element(data, index, mget_DesiredSSID());
    }
    else
    {
		// 20120709 caisf add, merged ittiam mac v1.2 code
        //index += set_bcast_ssid(data, index);
        index += set_ssid_probe_sta_prot(data, index);
    }

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Supported operating classes element */
    index += set_sup_opclas_11d_elem(data, index);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, PROBE_REQ);

    /* Add Extended Capabilities Information element */
    index += set_ext_cap(data, index);

    /* Set the WPS IE if required */
    index += set_wps_element(data, index, PROBE_REQ);

    /* Set the P2P IE if required */
    index += set_p2p_ie_probe_req(data, index);

    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_beacon_sta                                       */
/*                                                                           */
/*  Description   : This function prepares the beacon frame (fixed part)     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the beacon frame                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the beacon frame are set as        */
/*                  specified in the standard.                               */
/*                                                                           */
/*  Outputs       : The contents of the given beacon frame have been set     */
/*                                                                           */
/*  Returns       : Beacon frame length                                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_beacon_sta(UWORD8* data)
{
    UWORD16       value = 0;
    UWORD16       index = 0;

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

    /* The Type/Subtype, Duration, Sequence Number and Fragment Number fields*/
    /* in the Frame Control Field are set.                                   */
    set_frame_control(data, (UWORD16)BEACON);

    /* Set durationID to 0 */
    set_durationID(data, 0);

    /* Set fragment number to 0 */
    set_fragment_number(data, 0);

    /* DA is Broadcast Address */
    set_address1(data, mget_bcst_addr());

    /* SA is the dot11MacAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*                       Beacon Frame - Frame Body                       */
    /* --------------------------------------------------------------------- */
    /* |Timestamp |BeaconInt |CapInfo |SSID |SupRates |DSParSet |IBSSParSet| */
    /* --------------------------------------------------------------------- */
    /* |8         |2         |2       |2-34 |3-10     |3        |4         | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Set Beacon interval */
    value = mget_BeaconPeriod();
    data[index + TIME_STAMP_LEN]     = value & 0xFF;
    data[index + TIME_STAMP_LEN+1]   = (value >> 8) & 0xFF;

    /* Update the index with the lengths of the Timestamp, Beacon Interval   */
    index += (TIME_STAMP_LEN + BEACON_INTERVAL_LEN);

    /* Set the capability information field and update the index with the    */
    /* length of the field.                                                  */
    set_cap_info_field_sta(data, index);
    index += CAP_INFO_LEN;

    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
    index += set_ssid_element(data, index, mget_DesiredSSID());

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the PHY parameters depending on the PHY type */
    index += set_phy_params(data, index);

    /* IBSS Parameter Set                    */
    /* ------------------------------------- */
    /* | Element ID | Length | ATIM Window | */
    /* ------------------------------------- */
    /* |1           |1       |2            | */
    /* ------------------------------------- */

    data[index]     = IIBPARMS;
    data[index + 1] = IIBPARMS_LEN;
    data[index + 2] = 0;
    data[index + 3] = 0;

    index += (IE_HDR_LEN + IIBPARMS_LEN);

    /* Set the ERP Information element and update the index value with the   */
    /* ERP information element length.                                       */
    index += set_erp_info_field(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

	// 20120830 caisf add, merged ittiam mac v1.3 code
	/* Add Country Information element */
	index += set_country_info_field(data, index);

#ifdef IBSS_11I
    /* Add the security information element */
    index += set_sec_info_element(data, index, 0);
#endif /* IBSS_11I */

    /* Add the WMM information element */
    index += set_wmm_beacon_params(data, index);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, BEACON);

    /* Add HT-Operation Information element */
    index += set_ht_opern_element_sta(data, index);

    /* Add Secondary Channel Offset Information Element */
    index += set_sec_channel_offset(data, index, BEACON);

	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Supported operating classes element */
    index += set_sup_opclas_11d_elem(data, index);

    TROUT_FUNC_EXIT;
    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_auth_req                                         */
/*                                                                           */
/*  Description   : This function prepares the authentication request        */
/*                  (sequence 1) frame.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the authentication request frame           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the authentication request frame   */
/*                  are set as specified in the standard.                    */
/*                                                                           */
/*  Outputs       : The contents of the given authentication frame have been */
/*                  set.                                                     */
/*                                                                           */
/*  Returns       : Authentication request frame length                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_auth_req(UWORD8* data)
{
    UWORD16 auth_req_len = 0;
    sta_entry_t *se      = 0;

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
    set_frame_control(data, (UWORD16)AUTH);

    /* Authentication for STAs  in  IBSS is not handled since this  requires */
    /* the MAC address  of the STA with  which to  authenticate  as an input */
    /* from the user. Hence, authentication is performed only by STAs in BSS.*/
    /* STAs in BSS initiate authentication with the AP.                      */

    /* DA is address of the AP (BSSID) */
    set_address1(data, mget_bssid());

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*              Authentication Frame (Sequence 1) - Frame Body           */
    /* --------------------------------------------------------------------  */
    /* |Auth Algorithm Number|Auth Transaction Sequence Number|Status Code|  */
    /* --------------------------------------------------------------------  */
    /* | 2                   |2                               |2          |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    /* Set the Authentication Algorithm Number to Open System or Shared Key, */
    /* based on the authentication type that has been requested by the user. */
    /* This is given by 'mget_auth_type' (OPEN_SYSTEM = 0 and SHARED_KEY = 1)*/
    /* Shared Key authentication may be done only if the MIB attribute       */
    /* dot11PrivacyOptionImplemented has a value 'True'. If the value is     */
    /* 'False', authentication type used is OPEN_SYSTEM (0), irrespective of */
    /* the type requested by the user.                                       */
    if(is_wep_enabled() == BFALSE)
    {
        /* Open System = 0x0000 */
        data[MAC_HDR_LEN]     = 0x00;
        data[MAC_HDR_LEN + 1] = 0x00;
    }
    else
    {
        UWORD16 auth_type = mget_auth_type();

        if(auth_type & BIT0)
        {
            auth_type = SHARED_KEY;
        }
        else
        {
            auth_type = OPEN_SYSTEM;
        }

        data[MAC_HDR_LEN]     = (auth_type & 0xFF);
        data[MAC_HDR_LEN + 1] = ((auth_type & 0xFF00) >> 8);
    }

    /* Set Authentication Transaction Sequence Number to 1 */
    data[MAC_HDR_LEN + 2] = 0x01;
    data[MAC_HDR_LEN + 3] = 0x00;

    /* Set Status Code to zero. This field is reserved in this frame. */
    data[MAC_HDR_LEN + 4] = 0x00;
    data[MAC_HDR_LEN + 5] = 0x00;

    /* Set the authentication request frame length */
    auth_req_len = MAC_HDR_LEN + AUTH_ALG_LEN + AUTH_TRANS_SEQ_NUM_LEN +
                   STATUS_CODE_LEN + FCS_LEN;

    /* STATION table needs to be added/updated */
    /* Find the entry in the STA Table */
    se = find_entry(mget_bssid());

    /* If sta index is zero, an entry dsn't exist, new entry is added */
    /* else the value of the sta index is refreshed from the table    */
    if(se == NULL)
    {
        /* Malloc the memory for the new handle */
        se = (sta_entry_t *)mem_alloc(g_local_mem_handle, sizeof(sta_entry_t));
        if(se != NULL)
        {
            mem_set(se, 0, sizeof(sta_entry_t));

            /* Add the entry in the STA Table */
            add_entry(se, mget_bssid());

            se->sta_index = get_new_sta_index(mget_bssid());

            if(se->sta_index == INVALID_ID)
            {
                /* Should never come here */
                delete_entry(mget_bssid());
                se = NULL;
            }
            else
            {
                /* Initialize prot-table entry */
                init_prot_handle_sta(se);
            }
        }
        else
        {
            auth_req_len = 0;
        }
    }

	TROUT_FUNC_EXIT;
    return auth_req_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_auth_req_seq3                                    */
/*                                                                           */
/*  Description   : This function prepares the authentication request        */
/*                  (sequence 3) frame, for Shared Key Authentication.       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the authentication request frame           */
/*                  2) Pointer to the incoming authentication frame          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the authentication request frame   */
/*                  are set as specified in the standard.                    */
/*                                                                           */
/*  Outputs       : The contents of the given authentication frame have been */
/*                  set.                                                     */
/*                                                                           */
/*  Returns       : Authentication request frame length                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_auth_req_seq3(UWORD8* data, UWORD8* msa)
{
    UWORD8  *data8       = 0;
    UWORD16 index        = 0;
    UWORD16 auth_req_len = 0;

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
	TROUT_FUNC_ENTER;
    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field and the WEP bit is set.                            */
    set_frame_control(data, (UWORD16)AUTH);

    /* WEP bit is set only if the WEP is enabled */
    set_wep(data, 1);

    /* Authentication for STAs  in  IBSS is not handled since this  requires */
    /* the MAC address  of the STA with  which to  authenticate  as an input */
    /* from the user. Hence, authentication is performed only by STAs in BSS.*/
    /* STAs in BSS initiate authentication with the AP.                      */

    /* DA is address of the AP (BSSID) */
    set_address1(data, mget_bssid());

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*              Authentication Frame (Sequence 3) - Frame Body           */
    /* --------------------------------------------------------------------- */
    /* |Auth Algo Number|Auth Trans Seq Number|Status Code| Challenge Text | */
    /* --------------------------------------------------------------------- */
    /* | 2              |2                    |2          | 3 - 256        | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and frame data pointer */
    index = MAC_HDR_LEN;
    data8 = (UWORD8*)(data + index);

    /* Initialize the authentication request length.  The initialized  value */
    /* will not contain the challenge  text  length. It is  included only if */
    /* the incoming authentication frame is not in error.                    */
    auth_req_len = MAC_HDR_LEN + AUTH_ALG_LEN + AUTH_TRANS_SEQ_NUM_LEN +
                   STATUS_CODE_LEN + FCS_LEN;

    /* In case of no failure, the frame must be WEP encrypted. 4 bytes must  */
    /* be   left for the  IV  in  that  case. These   fields will  then  be  */
    /* reinitialized, using the correct index, with offset for IV field.     */
    data8[0] = SHARED_KEY;  /* Authentication Algorithm Number               */
    data8[1] = 0x00;

    data8[2] = 0x03;        /* Authentication Transaction Sequence Number    */
    data8[3] = 0x00;

    /* If WEP subfield in the  incoming  authentication frame is 1,  respond */
    /* with  'challenge text failure' status,  since the STA does not expect */
    /* an encrypted frame in this state.                                     */
    if(get_wep(msa) == 1)
    {
        data8[4] = CHLNG_FAIL;
        data8[5] = 0x00;
    }
    /* If the STA does not support WEP, respond with 'unsupported algo'      */
    /* status, since WEP is necessary for Shared Key Authentication.         */
    else if(is_wep_enabled() == BFALSE)
    {
        data8[4] = UNSUPT_ALG;
        data8[5] = 0x00;
    }
    /* If the default WEP key is NULL, respond with 'challenge text failure' */
    /* status, since a NULL key value cannot be used for WEP operations.     */
    else if(mget_WEPDefaultKeySize() == 0)
    {
        data8[4] = CHLNG_FAIL;
        data8[5] = 0x00;
    }

    /* If there is a mapping in dot11WEPKeyMappings matching the address of  */
    /* the AP, and the corresponding key is NULL respond with 'challenge     */
    /* text failure' status. This is currently not being used.               */

    /* No error condition detected */
    else
    {
        UWORD8  *ch_text     = 0;
        UWORD8  ch_text_len  = 0;

        /* Set Status Code to 'success' */
        data8[4] = SUCCESSFUL_STATUSCODE;
        data8[5] = 0x00;

        /* Extract 'Challenge Text' and its 'length' from the incoming       */
        /* authentication frame                                              */
        ch_text_len = msa[MAC_HDR_LEN + 7];
        ch_text     = (UWORD8*)(&msa[MAC_HDR_LEN + 8]);

        /* Challenge Text Element                  */
        /* --------------------------------------- */
        /* |Element ID | Length | Challenge Text | */
        /* --------------------------------------- */
        /* | 1         |1       |1 - 253         | */
        /* --------------------------------------- */

        data[index + 6]   = ICTEXT;
        data[index + 7]   = ch_text_len;
        memcpy(&data[index + 8], ch_text, ch_text_len);

        /* Add the challenge text element length to the authentication       */
        /* request frame length. The IV, ICV element lengths will be added   */
        /* after encryption.                                                 */
        auth_req_len += (ch_text_len + IE_HDR_LEN);
    }
    TROUT_FUNC_EXIT;
    return auth_req_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_asoc_req                                         */
/*                                                                           */
/*  Description   : This function prepares the  association request frame.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association request frame              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the association request frame are  */
/*                  set as specified in the standard.                        */
/*                                                                           */
/*  Outputs       : The contents of the given association request frame have */
/*                  been set.                                                */
/*                                                                           */
/*  Returns       : Association request frame length                         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_asoc_req(UWORD8* data)
{
    UWORD16 index = 0;

	//chenq add 2012-11-05
	UWORD16 element_index = 0;

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

	TROUT_FUNC_ENTER;
    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)ASSOC_REQ);

    /* DA is address of the AP (BSSID) */
    set_address1(data, mget_bssid());

    /* SA is the dot11MACAddress */
    set_address2(data, mget_StationID());

    set_address3(data, mget_bssid());

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/

    /*************************************************************************/
    /*              Association Request Frame - Frame Body                   */
    /* --------------------------------------------------------------------- */
    /* | Capability Information | Listen Interval | SSID | Supported Rates | */
    /* --------------------------------------------------------------------- */
    /* |2                       |2                |2-34  |3-10             | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index and the frame data pointer */
    index  = MAC_HDR_LEN;

    /* Set the capability information field */
    set_cap_info_field_sta(data, index);

    /* Set the Listen Interval (Listen Interval length is 1 byte) */
    data[MAC_HDR_LEN + 2] = mget_listen_interval();
    data[MAC_HDR_LEN + 3] = 0x00;

    /* Update the index value with the lengths of the Listen Interval and    */
    /* Capability Information Fields.                                        */
    index += (LISTEN_INT_LEN + CAP_INFO_LEN);

	//chenq add 2012-11-05
	element_index = index;
	
    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
    index += set_ssid_element(data, index, mget_DesiredSSID());

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set 802.11h Elements */
    index += set_11h_elements(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

    /* Add the security information element */
    index += set_sec_info_element(data, index, 1);

    /* WMM element */
    index += set_wmm_asoc_req_params(data, index);

	// 20120830 caisf add, merged ittiam mac v1.3 code
    /* Add Supported operating classes element */
    index += set_sup_opclas_11d_elem(data, index);

    /* Add HT-Capabilities Information element */
    index += set_ht_asoc_req_params(data, index);

    /* Set the WPS IE if required */
    index += set_wps_element(data, index, ASSOC_REQ);

    /* Set the P2P IE if required */
    index += set_asoc_req_p2p_ie(data, index);

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
	/* Set the wapi IE if required*/
	index += set_asoc_req_wapi_ie(data,index);
#endif
	
    /* Store the last association request information */
    /* This is to be sent to external supplicant(host)*/
    mem_set(g_assoc_req_info, 0xFF, MAX_STRING_LEN);

	//chenq add 2012-11-05
	//g_assoc_req_info[0] = MIN(MAX_STRING_LEN - 1,(index-MAC_HDR_LEN));
	//memcpy(g_assoc_req_info + 1, data + MAC_HDR_LEN, g_assoc_req_info[0]);
	
	g_assoc_req_info[0] = MIN(MAX_STRING_LEN - 1,(index-element_index));
	memcpy(g_assoc_req_info + 1, data + element_index, g_assoc_req_info[0]);
    

	TROUT_FUNC_EXIT;
    /* The value in 'index' contains the total number of bytes in the frame  */
    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_probe_rsp_sta                                    */
/*                                                                           */
/*  Description   : This function prepares the probe response frame.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe response frame                   */
/*                  2) Pointer to the probe request frame                    */
/*                  3) Is P2P IE present in the probe request frame          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various fields of the probe response frame are set   */
/*                  as specified in the standard.                            */
/*                                                                           */
/*  Outputs       : The contents of the given probe response frame have been */
/*                  set.                                                     */
/*                                                                           */
/*  Returns       : Probe response frame length                              */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_probe_rsp_sta(UWORD8* data, UWORD8* probe_req, BOOL_T is_p2p)
{
    UWORD16 value    = 0;
    UWORD16 index    = 0;
    UWORD8  addr2[6] = {0};

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
    set_frame_control(data, (UWORD16)PROBE_RSP);

    /* Get the address of the transmitting STA */
    get_address2(probe_req, addr2);

    /* Set DA to the address of transmitting STA */
    set_address1(data, addr2);

    /* Set SA to the dot11MacAddress */
    set_address2(data, mget_StationID());

    set_address3_probe_resp_prot(data);


    /* Set the contents of the frame body */
    /*************************************************************************/
    /*                       Probe Response Frame - Frame Body               */
    /* --------------------------------------------------------------------- */
    /* |Timestamp|BeaconInt|CapInfo| SSID |SupRates|DSParamSet|IBSSParamSet| */
    /* --------------------------------------------------------------------- */
    /* |8        |2        |2      |2-34  |3-10    |3         |4           | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    /* Initialize index */
    index = MAC_HDR_LEN;

    /* Set Beacon interval (2 bytes) */
    value = mget_BeaconPeriod();
    data[index + TIME_STAMP_LEN]     = value & 0xFF;
    data[index + TIME_STAMP_LEN+1]   = (value >> 8) & 0xFF;

    /* Update the index with the lengths of the Timestamp, Beacon Interval.  */
    index += (TIME_STAMP_LEN + BEACON_INTERVAL_LEN);

    /* Set the capability information field and update the index with the    */
    /* length of the field.                                                  */
    set_cap_info_field_sta(data, index);
    index += CAP_INFO_LEN;

    /* Set the SSID element and update the index value with the SSID element */
    /* length.                                                               */
	// 20120709 caisf add, merged ittiam mac v1.2 code
    //index += set_ssid_probe_rsp_sta_prot(data, index);
    index += set_ssid_probe_sta_prot(data, index);

    /* Set the Supported Rates element and update the index value with the   */
    /* Supported Rates element length.                                       */
    index += set_sup_rates_element(data, index);

    /* Set the PHY parameters depending on the PHY type */
    index += set_phy_params(data, index);

    /* IBSS Parameter Set                    */
    /* ------------------------------------- */
    /* | Element ID | Length | ATIM Window | */
    /* ------------------------------------- */
    /* |1           |1       |2            | */
    /* ------------------------------------- */

    data[index]     = IIBPARMS;
    data[index + 1] = IIBPARMS_LEN;
    data[index + 2] = 0;
    data[index + 3] = 0;

    index += (IE_HDR_LEN + IIBPARMS_LEN);

    /* Set the ERP Information element and update the index value with the   */
    /* ERP information element length.                                       */
    index += set_erp_info_field(data, index);

    /* Set the Extended Supported Rates element and update the index value   */
    /* with the Extended Supported Rates element length.                     */
    index += set_exsup_rates_element(data, index);

#ifdef IBSS_11I
    /* Add the security information element */
    index += set_sec_info_element(data, index, 0);
#endif /* IBSS_11I */

	// 20120830 caisf add, merged ittiam mac v1.3 code
	if(is_p2p == BFALSE)
	{
		/* Add Country Information element */
		index += set_country_info_field(data, index);
	}

    /* Add the WMM parameter element if required */
    index += set_wmm_probe_rsp_params(data, index, probe_req);

    /* Add HT-Capabilities Information element */
    index += set_ht_capabilities(data, index, PROBE_RSP);

    /* Add HT-Operation Information element */
    index += set_ht_opern_element_sta(data, index);

    /* Add Secondary Channel Offset Information Element */
    index += set_sec_channel_offset(data, index, PROBE_RSP);

    index += set_p2p_ie_probe_rsp(data, index, is_p2p);

    /* Add the WSC IE */
    index += add_wsc_ie((data + index), PROBE_RSP);

    return index + FCS_LEN;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_cap_info_field_sta                                   */
/*                                                                           */
/*  Description   : This function sets the Capability Information Field in   */
/*                  the given frame.                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame.                                 */
/*                  2) Index of the Capability Information Field             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The various bits of the Capability Information field is  */
/*                  set depending on the ESS/IBSS, Privacy and Short         */
/*                  Preamble Capabilities of the station.                    */
/*                                                                           */
/*  Outputs       : The given data frame contents from the given offset are  */
/*                  set to the Capability Information Field.                 */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_cap_info_field_sta(UWORD8* data, UWORD16 index)
{
    /* Capability Information Field                                          */
    /* -------------------------------------------------------------------   */
    /* |ESS|IBSS|CFPollable|CFPReq|Privacy|Preamble|PBCC|Agility|SpecMgmt|   */
    /* -------------------------------------------------------------------   */
    /* |B0 |B1  |B2        |B3    |B4     |B5      |B6  |B7     |B8      |   */
    /* -------------------------------------------------------------------   */
    /* |QoS |ShortSlot|APSD|Reserved|DSSS-OFDM |Delayed BA |Immediate BA |   */
    /* -------------------------------------------------------------------   */
    /* |B9  |B10      |B11 |B12     |B13       |B14        |B15          |   */
    /* -------------------------------------------------------------------   */

    data[index] = get_sta_bss_type_prot();

	#ifdef MAC_WAPI_SUPP
	/* The Privacy bit is set if WEP is enabled. */
    if( (mget_PrivacyInvoked() == TV_TRUE) || (mget_wapi_enable() == TV_TRUE) )
            data[index] |= PRIVACY;
	#else
	/* The Privacy bit is set if WEP is enabled. */
    if(mget_PrivacyInvoked() == TV_TRUE)
            data[index] |= PRIVACY;
	#endif

    data[index + 1] = 0;

    /* Set the PHY specific capability bits */
	//chenq add for wapi 2012-12-07
	#ifdef MAC_WAPI_SUPP
	//if( mget_wapi_enable() == TV_FALSE)
	//{
		set_capability_phy(data, index);
	//}
	#else
		set_capability_phy(data, index);
	#endif

    /* Set the spectrum management bit for CSA frames */
    set_spectrum_mgmt_bit_sta(data, index);

    /* Set protocol specific capability bits */
	//chenq add for wapi 2012-12-07
	#ifdef MAC_WAPI_SUPP
	if( mget_wapi_enable() == TV_FALSE)
	{
    	set_capability_prot_sta(data, index);
	}
	#else
		set_capability_prot_sta(data, index);
	#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_tim_elm                                              */
/*                                                                           */
/*  Description   : This function returns the TIM element in the received    */
/*                  beacon                                                   */
/*                                                                           */
/*  Inputs        : 1) Beacon/ProbeResponse Frame                            */
/*                  2) Length of the Frame                                   */
/*                  3) Index from where the search should start              */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to TIM element in beacon frame.                  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8* get_tim_elm(UWORD8* msa, UWORD16 rx_len, UWORD16 tag_param_offset)
{
    UWORD16 index = 0;

    /*************************************************************************/
    /*                       Beacon Frame - Frame Body                       */
    /* --------------------------------------------------------------------- */
    /* |Timestamp |BeaconInt |CapInfo |SSID |SupRates |DSParSet |TIM elm   | */
    /* --------------------------------------------------------------------- */
    /* |8         |2         |2       |2-34 |3-10     |3        |4-256     | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

    index = tag_param_offset;

    /* Search for the TIM Element Field and return if the element is found */
    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == ITIM)
        {
            return(&msa[index]);
        }
        else
        {
            index += (IE_HDR_LEN + msa[index + 1]);
        }
    }

    return(0);
}


#endif /* IBSS_BSS_STATION_MODE */
