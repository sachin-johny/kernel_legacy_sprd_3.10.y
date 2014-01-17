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
/*  File Name         : eapol_key.c                                          */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various functions related to EAPOL Key frames.       */
/*                                                                           */
/*  List of Functions : prepare_eapol_key_frame                              */
/*                      prepare_1x_hdr                                       */
/*                      prepare_key_info                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "eapol_key.h"


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prepare_eapol_key_frame                               */
/*                                                                           */
/*  Description      : This function prepares the EAPOL key frame with the   */
/*                     given parameters.                                     */
/*                                                                           */
 /*  Inputs           : 1) EAPOL Key frame buff                              */
/*                      2) Key Information value                             */
/*                      3) Key RSC value                                     */
/*                      4) Nonce                                             */
/*                      5) MIC                                               */
/*                      6) Key Data Length                                   */
/*                      7) Key Data                                          */
/*                      8) Replay count                                      */
/*                      9) EAPOL Key frame buffer                            */
/*                      10) 802.11i Mode type                                */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_eapol_key_frame(UWORD16 key_info, UWORD16 key_length,
                                UWORD8* key_rsc, UWORD8* nonce,
                                UWORD8 *key_iv,  UWORD16 key_data_len,
                                UWORD8* key_data, UWORD8* replay_cnt,
                                UWORD8* data, UWORD8 mode_802_11i)
{
    /* ----------------------------------------------------------------   */
    /* | Descriptor Type | Key Info | Key Length | Key Replay Counter |   */
    /* ----------------------------------------------------------------   */
    /* | 1 octet         | 2 octets | 2 octets   | 8 octets           |   */
    /* ----------------------------------------------------------------   */
    /* --------------------------------------------------------------     */
    /* | Key Nonce | EAPOL-Key IV | Key RSC  | Reserved | Key MIC   |     */
    /* --------------------------------------------------------------     */
    /* | 32 octets | 16 octets    | 8 octets | 8 octets | 16 octets |     */
    /* --------------------------------------------------------------     */
    /* ------------------------------                                     */
    /* | Key Data Length | Key Data |                                     */
    /* ------------------------------                                     */
    /* | 2 octets        | n octets |                                     */
    /* ------------------------------                                     */

    UWORD32 index = 0;

    if(mode_802_11i == RSNA_802_11I)
    {
        data[index] = IEE80211_KEY_DESCRIPTOR_TYPE;
    }
    else if(mode_802_11i == WPA_802_11I)
    {
        data[index] = WPA_KEY_DESCRIPTOR_TYPE;
    }
    index += KEY_DESCRIPTOR_SIZE;

    data[index]     = (UWORD8)((key_info & 0xFF00) >> 8);
    data[index + 1] = (UWORD8)(key_info & 0xFF);
    index += KEY_INFO_SIZE;

    data[index]     = (UWORD8)((key_length & 0xFF00) >> 8);
    data[index + 1] = (UWORD8)(key_length & 0xFF);
    index += KEY_LENGTH_SIZE;

    if(replay_cnt !=NULL)
    memcpy(&data[index], replay_cnt, REPLAY_COUNTER_SIZE);
    else
        mem_set(&data[index], 0, REPLAY_COUNTER_SIZE);

    index += REPLAY_COUNTER_SIZE;

    if(nonce != NULL)
    {
        memcpy(&data[index], nonce, NONCE_SIZE);
    }
    else
    {
        mem_set(&data[index], 0, NONCE_SIZE);
    }
    index += NONCE_SIZE;

    if(key_iv != NULL)
    {
        memcpy(&data[index], key_iv, EAPOL_KEY_IV_SIZE);
    }
    else
    {
        mem_set(&data[index], 0, EAPOL_KEY_IV_SIZE);
    }

    index += EAPOL_KEY_IV_SIZE;

    if(key_rsc != NULL)
    {
        memcpy(&data[index], key_rsc, RSC_SIZE);
    }
    else
    {
        mem_set(&data[index], 0, RSC_SIZE);
    }
    index += RSC_SIZE;

    mem_set(&data[index], 0, 8);
    index += 8; /* 8 octets reserved */

    /* The MIC field is set to zero. This is later filled up with the  */
    /* computed MIC value.                                             */
    mem_set(&data[index], 0, MIC_SIZE);
    index += MIC_SIZE;

    data[index] = (UWORD8)((key_data_len & 0xFF00) >> 8);
    data[index + 1] = key_data_len & 0xFF;
    index += 2;

    if(key_data_len != 0)
        memcpy(&data[index], key_data, key_data_len);
    index += key_data_len;

    return index;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prepare_1x_hdr                                        */
/*                                                                           */
/*  Description      : This function prepares the 1X Header as per the 802.1X*/
/*                     standard and sets it in the buffer.                   */
/*                                                                           */
/*  Inputs           : 1) Buffer                                             */
/*                     2) Body Type                                          */
/*                     3) Buffer Length                                      */
/*                     4) 802.11i Mode                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void prepare_1x_hdr(UWORD8* buffer, EAPOL_PKT_TYPE pkt_type, UWORD16 buff_len,
                    UWORD8 mode_802_11i)
{
    /* --------------------------------------------------------------------- */
    /* | SNAP    | Version | Type    | Body Length | Body                  | */
    /* --------------------------------------------------------------------- */
    /* | 8 octet | 1 octet | 1 octet | 2 octet     | (Length - 4) octets   | */
    /* --------------------------------------------------------------------- */

    buffer[0] = 0xAA;
    buffer[1] = 0xAA;
    buffer[2] = 0x03;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x00;
    buffer[6] = 0x88;
    buffer[7] = 0x8E;

    if(mode_802_11i == RSNA_802_11I)
    {
        buffer[8] = EAPOL_VERSION_02;
    }
    else if(mode_802_11i == WPA_802_11I)
    {
        buffer[8] = EAPOL_VERSION_01;
    }
    buffer[9] = pkt_type;

    buffer[10] = (UWORD8)((buff_len & 0xFF00) >> 8);
    buffer[11] = (UWORD8)(buff_len & 0x00FF);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prepare_key_info                                      */
/*                                                                           */
/*  Description      : This function prepares the key information field for  */
/*                     the EAPOL key frames with the given parameters.       */
/*                                                                           */
/*  Inputs           :  1) Descriptor version                                */
/*                      2) Security bit                                      */
/*                      3) MIC availability information                      */
/*                      4) Acknowledgement bit                               */
/*                      5) Install flag                                      */
/*                      6) Key type                                          */
/*                      7) Key Index                                         */
/*                      8) Enctrypted flag                                   */
/*                      9) WPA or WPA2 mode                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
UWORD16 prepare_key_info(UWORD8 dscr_ver, UWORD8 secure_bit,
                                UWORD8 mic_available, UWORD8 ack_bit,
                                UWORD8 install_flag, KEY_TYPE_T key_type,
                                UWORD8 key_index, UWORD8 encrypted,
                                UWORD8 mode_802_11i)
{
    UWORD16 key_info = 0;

    key_info = dscr_ver;
    key_info |= ((UWORD16)key_type << 3);

    if(mode_802_11i == RSNA_802_11I)
    {
        /* ----------------------------------------------------------------- */
        /* RNSA Key descriptor                                               */
        /* ----------------------------------------------------------------- */
        /* | Key Descriptor Version | Key Type | Rsvd | Install | Key Ack  | */
        /* ----------------------------------------- ----------------------- */
        /* | B0                  B2 | B3       | B4-B5| B6      | B7       | */
        /* ----------------------------------------------------------------- */
        /* | Key MIC | Secure | Error | Request | Encrypted Key Data| Rsvd | */
        /* ----------------------------------------------------------------- */
        /* | B8      | B9     | B10   | B11     | B12               |B13-15| */
        /* ----------------------------------------------------------------- */
        key_info |= ((UWORD16)encrypted << 12);

    }
    else if(mode_802_11i == WPA_802_11I)
    {
        /* ----------------------------------------------------------------- */
        /* WPA Key descriptor                                                */
        /* ----------------------------------------------------------------- */
        /* | Key Descriptor Version | Key Type | Key Index| Install|Key Ack| */
        /* ----------------------------------------------------------------- */
        /* | B0                  B2 | B3       | B4    B5 | B6     | B7    | */
        /* ----------------------------------------------------------------- */
        /* | Key MIC | Secure | Error | Request |      Reserved            | */
        /* ----------------------------------------------------------------- */
        /* | B8      | B9     | B10   | B11     | B12                  B15 | */
        /* ----------------------------------------------------------------- */

        key_info |= ((UWORD16)key_index << 4);
    }
    key_info |= ((UWORD16)install_flag << 6);
    key_info |= ((UWORD16)ack_bit << 7);
    key_info |= ((UWORD16)mic_available << 8);
    key_info |= ((UWORD16)secure_bit << 9);

    return key_info;
}

#endif /* MAC_802_11I */

