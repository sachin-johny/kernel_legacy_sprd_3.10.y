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
/*  File Name         : sta_frame_11e.h                                      */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      preparation of the various MAC frames specific to    */
/*                      station mode of operation in 802.11e.                */
/*                                                                           */
/*  List of Functions : set_qos_control_sta                                  */
/*                      set_qos_info_wmm_sta                                 */
/*                      set_ac_param_record_wmm_sta                          */
/*                      set_wmm_info_element_sta                             */
/*                      set_wmm_param_element_sta                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_WMM

#ifndef STA_FRAME_802_11E_H
#define STA_FRAME_802_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "host_if.h"
#include "frame_11e.h"
#include "management_11e.h"
#include "sta_uapsd.h"

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Set the QoS Control field in the MAC header for QoS Data frames with the  */
/* given TID and ACK Policy.                                                 */
INLINE void set_qos_control_sta(UWORD8 *header, UWORD8 tid, UWORD8 ap)
{
    /* QoS Control field                                                 */
    /* +---------------------------------------------------------------+ */
    /* | B0:B2   | B3           | B4       | B5:B6      | B7:B15       | */
    /* +---------------------------------------------------------------+ */
    /* | TID     | Reserved (0) | EOSP (0) | Ack Policy | Reserved (0) | */
    /* +---------------------------------------------------------------+ */
    header[QOS_CTRL_FIELD_OFFSET]  = (tid & 0x07);
    header[QOS_CTRL_FIELD_OFFSET] |= (ap & 0x03) << 5;

    header[QOS_CTRL_FIELD_OFFSET + 1] = 0;
}

/* Set the QoS info field in the WMM Information Element for STA */
INLINE void set_qos_info_wmm_sta(UWORD8 *data)
{
    UWORD8 qos_info = 0;

    /* QoS Information field                                          */
    /* -------------------------------------------------------------- */
    /* | B0    | B1    | B2    | B3    | B4      | B5:B6 | B7       | */
    /* -------------------------------------------------------------- */
    /* | AC_VO | AC_VI | AC_BK | AC_BE |         | Max SP|          | */
    /* | U-APSD| U-APSD| U-APSD| U-APSD| Reserved| Length| Reserved | */
    /* | Flag  | Flag  | Flag  | Flag  |         |       |          | */
    /* -------------------------------------------------------------- */

    /* Set the UAPSD configuration information in the QoS info field if the  */
    /* BSS type is Infrastructure and the AP supports UAPSD.                 */
    if((mget_DesiredBSSType() == INFRASTRUCTURE) &&
       (is_ap_uapsd_capable() == BTRUE))
    {
        UWORD8 max_sp_length  = get_uapsd_config_max_sp_len();

        qos_info |= (get_uapsd_config_ac(AC_VO) << 0);
        qos_info |= (get_uapsd_config_ac(AC_VI) << 1);
        qos_info |= (get_uapsd_config_ac(AC_BK) << 2);
        qos_info |= (get_uapsd_config_ac(AC_BE) << 3);

        if(max_sp_length <= 6)
        {
            UWORD8 max_sp_bits = max_sp_length >> 1;

            qos_info |= ((max_sp_bits & 0x03) << 5);
        }
    }

    data[0] = qos_info;
}

/* Set the AC parameter record field in the WMM Parameter Element for STA */
INLINE UWORD8 set_ac_param_record_wmm_sta(UWORD8 *data, UWORD8 ac)
{
    UWORD16 txop = 0;

    /* AC_** Parameter Record field               */
    /* ------------------------------------------ */
    /* | Byte 1    | Byte 2        | Byte 3:4   | */
    /* ------------------------------------------ */
    /* | ACI/AIFSN | ECWmin/ECWmax | TXOP Limit | */
    /* ------------------------------------------ */

    /* ACI/AIFSN Field                    */
    /* ---------------------------------- */
    /* | B0:B3 | B4  | B5:B6 | B7       | */
    /* ---------------------------------- */
    /* | AIFSN | ACM | ACI   | Reserved | */
    /* ---------------------------------- */
    data[0]  = (mget_EDCATableAIFSN(ac) & 0x0F);          /* AIFSN */
    data[0] |= (mget_EDCATableIndex(ac) & 0x03) << 5;     /* ACI   */
    if(mget_EDCATableMandatory(ac) == TV_TRUE)
        data[0] |= BIT4; /* ACM   */

    /* ECWmin/ECWmax Field */
    /* ------------------- */
    /* | B0:B3  | B4:B7  | */
    /* ------------------- */
    /* | ECWmin | ECWmax | */
    /* ------------------- */
    data[1]  = (mget_EDCATableCWmin(ac) & 0x0F);      /* ECWmin */
    data[1] |= (mget_EDCATableCWmax(ac) & 0x0F) << 4; /* ECWmax */

    /* TXOP Limit. The value saved in MIB is in usec while the value to be   */
    /* set in this element should be in multiple of 32us                     */
    txop = mget_EDCATableTXOPLimit(ac);
    txop = txop >> 5;

    data[2] = (txop & 0x00FF);
    data[3] = (txop & 0xFF00) >> 8;

    return AC_PARAM_REC_LEN;
}

/* Set the WMM Information Element for STA */
INLINE UWORD8 set_wmm_info_element_sta(UWORD8 *data, UWORD16 index)
{
    /* WMM Information Element Format                                */
    /* ------------------------------------------------------------- */
    /* | 3     | 1        | 1           | 1             | 1        | */
    /* ------------------------------------------------------------- */
    /* | OUI   | OUI Type | OUI Subtype | Version field | QoS Info | */
    /* ------------------------------------------------------------- */
    data[index++] = IWMM;
    data[index++] = IWMM_INFO_LEN;

    /* OUI */
    data[index++] = 0x00;
    data[index++] = 0x50;
    data[index++] = 0xF2;

    /* OUI Type */
    data[index++] = 2;

    /* OUI Subtype */
    data[index++] = 0;

    /* Version field */
    data[index++] = 1;

    /* QoS Info field */
    set_qos_info_wmm_sta(&data[index]);

    return (IE_HDR_LEN + IWMM_INFO_LEN);
}

/* Set the WMM Parameter Element for STA */
INLINE UWORD8 set_wmm_param_element_sta(UWORD8 *data, UWORD16 index)
{
    /* WMM Parameter Element Format                                          */
    /* --------------------------------------------------------------------- */
    /* | 3     | 1        | 1           | 1             | 1        | 1     | */
    /* --------------------------------------------------------------------- */
    /* | OUI   | OUI Type | OUI Subtype | Version field | QoS Info | Resvd | */
    /* --------------------------------------------------------------------- */
    /* | 4              | 4              | 4              | 4              | */
    /* --------------------------------------------------------------------- */
    /* | AC_BE ParamRec | AC_BK ParamRec | AC_VI ParamRec | AC_VO ParamRec | */
    /* --------------------------------------------------------------------- */
    data[index++] = IWMM;
    data[index++] = IWMM_PARAM_LEN;

    /* OUI */
    data[index++] = 0x00;
    data[index++] = 0x50;
    data[index++] = 0xF2;

    /* OUI Type */
    data[index++] = 2;

    /* OUI Subtype */
    data[index++] = 1;

    /* Version field */
    data[index++] = 1;

    /* QoS Info field */
    set_qos_info_wmm_sta(&data[index]);
    index += QOS_INFO_LEN;

    /* Reserved */
    data[index++] = 0;

    /* Set the AC_BE, AC_BK, AC_VI, AC_VO Parameter Record fields */
    index += set_ac_param_record_wmm_sta(&data[index], AC_BE);
    index += set_ac_param_record_wmm_sta(&data[index], AC_BK);
    index += set_ac_param_record_wmm_sta(&data[index], AC_VI);
    index += set_ac_param_record_wmm_sta(&data[index], AC_VO);

    return (IE_HDR_LEN + IWMM_PARAM_LEN);
}

#endif /* STA_FRAME_802_11E_H */

#endif /* MAC_WMM */
#endif /* IBSS_BSS_STATION_MODE */
