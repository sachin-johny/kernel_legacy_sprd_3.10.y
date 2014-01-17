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
/*  File Name         : frame.h                                              */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the MAC frames                                       */
/*                                                                           */
/*  List of Functions : get_type                                             */
/*                      get_sub_type                                         */
/*                      get_to_ds                                            */
/*                      get_from_ds                                          */
/*                      set_from_ds                                          */
/*                      get_wep                                              */
/*                      set_wep                                              */
/*                      set_frame_control                                    */
/*                      set_durationID                                       */
/*                      get_address1                                         */
/*                      set_address1                                         */
/*                      get_address2                                         */
/*                      set_address2                                         */
/*                      get_address3                                         */
/*                      set_address3                                         */
/*                      get_fragment_number                                  */
/*                      set_fragment_number                                  */
/*                      get_BSSID                                            */
/*                      get_auth_seq_num                                     */
/*                      get_auth_status                                      */
/*                      get_auth_ch_text                                     */
/*                      get_asoc_status                                      */
/*                      get_ssid                                             */
/*                      is_group                                             */
/*                      get_DA                                               */
/*                      get_SA                                               */
/*                      get_protocol_version                                 */
/*                      get_retry                                            */
/*                      get_sequence_number                                  */
/*                      get_more_frag                                        */
/*                      set_mac_hdr                                          */
/*                      get_auth_algo_num                                    */
/*                      get_cap_info                                         */
/*                      get_pwr_mgt                                          */
/*                      set_pwr_mgt                                          */
/*                      get_more_data                                        */
/*                      set_more_data                                        */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef FRAME_H
#define FRAME_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAC_HDR_LEN             24          /* No Address4 - non-ESS         */
#define MAX_MAC_HDR_LEN         26 //QOS_MAC_HDR_LEN
#define MAX_SSID_LEN            33
#define MAX_RATES_SUPPORTED     12
#define ADDR1_OFFSET            4
#define ADDR2_OFFSET            10

#define MAX_DEVICE_NAME_LEN     32

#define PROTOCOL_VERSION        0x00
/* Assuming Max A-MSDU length = 3839 Bytes */
#ifndef DISABLE_MACHW_DEAGGR
#define MAX_MSDU_LEN            3856
#else /* DISABLE_MACHW_DEAGGR */
#define MAX_MSDU_LEN            1596
#endif /* DISABLE_MACHW_DEAGGR */
#define SNAP_HDR_LEN            8
#define SNAP_HDR_ID_LEN         6

#define DEFRAG_SIZE             6
#define AUTH_ALG_LEN            2
#define AUTH_TRANS_SEQ_NUM_LEN  2
#define STATUS_CODE_LEN         2
#define REASON_CODE_LEN         2
#define CHTXT_ELE_LEN           138         /* Includes length of IV and ICV */
#define FCS_LEN                 4
#define TIME_STAMP_LEN          8
#define BEACON_INTERVAL_LEN     2
#define CAP_INFO_LEN            2
#define LISTEN_INT_LEN          2
#define AID_LEN                 2
#define IE_HDR_LEN              2

/*                                                                           */
/* Length of the Information Element fields.                                 */
/*                                                                           */
/* Note that this length does not include the length of the header of the    */
/* information element. The actual length of the information element will be */
/* IE_HDR_LEN bytes more than the length defined by these macros.     */
/* Macro with suffix of _MAX indicates Maximum length of Information element */
/* Macro with suffix of _MIN indicates Minimum length of Information element */
/* Macro with no suffix of indicates Fixed length of Information element     */
#define ISSID_LEN_MAX               32     /* Service Set Identifier         */
#define ISUPRATES_LEN_MAX            8     /* Supported Rates                */
#define IFHPARMS_LEN                 5     /* FH parameter set               */
#define IDSPARMS_LEN                 1     /* DS parameter set               */
#define ICFPARMS_LEN                 6     /* CF parameter set               */
#define ITIM_LEN_MAX               254     /* Traffic Information Map        */
#define IIBPARMS_LEN                 2     /* IBSS parameter set             */
#define ICOUNTRY_LEN                 6     /* Country element                */
#define IEDCAPARAMS_LEN             18     /* EDCA parameter set             */
#define ITSPEC_LEN                  55     /* Traffic Specification          */
#define ITCLAS_LEN_MAX             255     /* Traffic Classification         */
#define ISCHED_LEN                  12     /* Schedule                       */
#define IPOWERCONSTRAINT_LEN         1     /* Power Constraint               */
#define IPOWERCAPABILITY_LEN         2     /* Power Capability               */
#define ITPCREQUEST_LEN              0     /* TPC Request                    */
#define ITPCREPORT_LEN               2     /* TPC Report                     */
#define ICHSWANNOUNC_LEN             3     /* Channel Switch Announcement    */
#define IMEASUREMENTREQUEST_LEN_MIN  3     /* Measurement request            */
#define IMEASUREMENTREPORT_LEN_MIN   3     /* Measurement report             */
#define IQUIET_LEN                   6     /* Quiet element Info             */
#define IERPINFO_LEN                 1     /* ERP Information                */
#define ITSDELAY_LEN                 4     /* TS Delay                       */
#define ITCLASPROCESS_LEN            1     /* TCLAS Processing               */
#define IHTCAP_LEN                  26     /* HT Capabilities                */
#define IQOSCAP_LEN                  1     /* QoS Capability                 */
#define IRSNELEMENT_LEN_MIN         34     /* RSN Information Element        */
#define IEXSUPRATES_LEN_MIN          1     /* Extended Supported Rates       */
#define IEXCHSWANNOUNC_LEN           4     /* Extended Ch Switch Announcement*/
#define IHTOPERATION_LEN            22     /* HT Information                 */
#define ISECCHOFF_LEN                1     /* Secondary Channel Offeset      */
#define I2040COEX_LEN                1     /* 20/40 Coexistence IE           */
#define I2040INTOLCHREPORT_LEN_MIN   1     /* 20/40 Intolerant channel report*/
#define IOBSSSCAN_LEN               14     /* OBSS Scan parameters           */
#define IEXTCAP_LEN                  1     /* Extended capability            */
#define IWMM_INFO_LEN                7     /* WMM parameters                 */
#define IWMM_PARAM_LEN              24     /* WMM parameters                 */
#define IWPAELEMENT_LEN_MIN         34     /* WPA Information Element        */

#define BEACON_LEN              1024
#define PS_POLL_LEN             20

#define TAG_PARAM_OFFSET        MAC_HDR_LEN + TIME_STAMP_LEN + \
                                BEACON_INTERVAL_LEN + CAP_INFO_LEN

#define CONTROL_HDR_LEN         16          /* Other than ACK */

#define SUB_MSDU_HEADER_LENGTH  14
#define MAX_SEC_HEADER_LENGTH   16

// 20120830 caisf add, merged ittiam mac v1.3 code
#define OPERATINGEXTNID        201 /* Regulatory Extension Identifier*/


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Basic Frame Type Codes (2-bit) */
typedef enum {CONTROL               = 0x04,
              DATA_BASICTYPE        = 0x08,
              MANAGEMENT            = 0x00,
              RESERVED              = 0x0C
} BASICTYPE_T;

/* Frame Type and Subtype Codes (6-bit) */
typedef enum {ASSOC_REQ             = 0x00,
              ASSOC_RSP             = 0x10,
              REASSOC_REQ           = 0x20,
              REASSOC_RSP           = 0x30,
              PROBE_REQ             = 0x40,
              PROBE_RSP             = 0x50,
              BEACON                = 0x80,
              ATIM                  = 0x90,
              DISASOC               = 0xA0,
              AUTH                  = 0xB0,
              DEAUTH                = 0xC0,
              ACTION                = 0xD0,
              PS_POLL               = 0xA4,
              RTS                   = 0xB4,
              CTS                   = 0xC4,
              ACK                   = 0xD4,
              CFEND                 = 0xE4,
              CFEND_ACK             = 0xF4,
              DATA                  = 0x08,
              DATA_ACK              = 0x18,
              DATA_POLL             = 0x28,
              DATA_POLL_ACK         = 0x38,
              NULL_FRAME            = 0x48,
              CFACK                 = 0x58,
              CFPOLL                = 0x68,
              CFPOLL_ACK            = 0x78,
              QOS_DATA              = 0x88,
              QOS_DATA_ACK          = 0x98,
              QOS_DATA_POLL         = 0xA8,
              QOS_DATA_POLL_ACK     = 0xB8,
              QOS_NULL_FRAME        = 0xC8,
              QOS_CFPOLL            = 0xE8,
              QOS_CFPOLL_ACK        = 0xF8,
              BLOCKACK_REQ          = 0x84,
              BLOCKACK              = 0x94
} TYPESUBTYPE_T;

/* Basic Frame Classes */
typedef enum{CLASS1_FRAME_TYPE      = 0x00,
             CLASS2_FRAME_TYPE      = 0x01,
             CLASS3_FRAME_TYPE      = 0x02,
}FRAMECLASS_T;

/* Element ID  of various Information Elements */
typedef enum {ISSID               = 0,   /* Service Set Identifier         */
              ISUPRATES           = 1,   /* Supported Rates                */
              IFHPARMS            = 2,   /* FH parameter set               */
              IDSPARMS            = 3,   /* DS parameter set               */
              ICFPARMS            = 4,   /* CF parameter set               */
              ITIM                = 5,   /* Traffic Information Map        */
              IIBPARMS            = 6,   /* IBSS parameter set             */
              ICOUNTRY            = 7,   /* Country element                */
              IEDCAPARAMS         = 12,  /* EDCA parameter set             */
              ITSPEC              = 13,  /* Traffic Specification          */
              ITCLAS              = 14,  /* Traffic Classification         */
              ISCHED              = 15,  /* Schedule                       */
              ICTEXT              = 16,  /* Challenge Text                 */
              IPOWERCONSTRAINT    = 32,  /* Power Constraint               */
              IPOWERCAPABILITY    = 33,  /* Power Capability               */
              ITPCREQUEST         = 34,  /* TPC Request                    */
              ITPCREPORT          = 35,  /* TPC Report                     */
              ISUPCHANNEL         = 36,  /* Supported channel list         */
              ICHSWANNOUNC        = 37,  /* Channel Switch Announcement    */
              IMEASUREMENTREQUEST = 38,  /* Measurement request            */
              IMEASUREMENTREPORT  = 39,  /* Measurement report             */
              IQUIET              = 40,  /* Quiet element Info             */
              IIBSSDFS            = 41,  /* IBSS DFS                       */
              IERPINFO            = 42,  /* ERP Information                */
              ITSDELAY            = 43,  /* TS Delay                       */
              ITCLASPROCESS       = 44,  /* TCLAS Processing               */
              IHTCAP              = 45,  /* HT Capabilities                */
              IQOSCAP             = 46,  /* QoS Capability                 */
              IRSNELEMENT         = 48,  /* RSN Information Element        */
              IEXSUPRATES         = 50,  /* Extended Supported Rates       */
              ISUPOPCLASS         = 59,  /* Supported Operating Class      */ // 20120830 caisf add, merged ittiam mac v1.3 code
              IEXCHSWANNOUNC      = 60,  /* Extended Ch Switch Announcement*/
              IHTOPERATION        = 61,  /* HT Information                 */
              ISECCHOFF           = 62,  /* Secondary Channel Offeset      */
              
			  //chenq add wapi
			  IWAPIELEMENT        = 68,  /* WAPI Information Element       */

              I2040COEX           = 72,  /* 20/40 Coexistence IE           */
              I2040INTOLCHREPORT  = 73,  /* 20/40 Intolerant channel report*/
              IOBSSSCAN           = 74,  /* OBSS Scan parameters           */
              IEXTCAP             = 127, /* Extended capability            */
              /* Need to change all to Vendor */
              IWMM                = 221, /* WMM parameters                 */
              IWPAELEMENT         = 221, /* WPA Information Element        */
              IP2P                = 221  /* P2P Information Element        */
} ELEMENTID_T;

/* Action Frames: Category */
typedef enum{SPECMGMT_CATEGORY  = 0,
             QOS_CATEGORY       = 1,
             DLS_CATEGORY       = 2,
             BA_CATEGORY        = 3,
             PUBLIC_CATEGORY    = 4,
             HT_CATEGORY        = 7,
             WMM_CATEGORY       = 17,
             VENDOR_CATEGORY    = 127,
} CATEGORY_T;

/* Field offset values in ACTION frames */
typedef enum{CATEGORY_OFFSET     = 0,
             ACTION_OFFSET       = 1,
             DIALOG_TOKEN_OFFSET = 2,
             STATUS_CODE_OFFSET  = 3,
             ACTION_BODY_OFFSET  = 4
} ACTION_FRAME_OFFSET_T;

/* Action Frames: Action Value */

/* Action Values defined for Spectrum Management Category */
typedef enum {MEASUREMENT_REQUEST_TYPE         = 0,
              MEASUREMENT_REPORT_TYPE          = 1,
              TPC_REQUEST_TYPE                 = 2,
              TPC_REPORT_TYPE                  = 3,
              CHANNEL_SWITCH_ANNOUNCEMENT_TYPE = 4
} SPECMGMT_ACTION_TYPE_T;

/* Action Values defined for WMM Category */
typedef enum{ADDTS_REQ_TYPE = 0,
             ADDTS_RSP_TYPE = 1,
             DELTS_TYPE     = 2
} WMM_ACTION_TYPE_T;

/* Action Values defined for BlockAck Category */
typedef enum{ADDBA_REQ_TYPE = 0,
             ADDBA_RSP_TYPE = 1,
             DELBA_TYPE     = 2
} BA_ACTION_TYPE_T;

/* Action Values defined for Public Category */
typedef enum {COEX_MGMT               = 0,
              EX_CHAN_SWITCH_ANNOUNCE = 4,
              VENDOR_PUBLIC_ACT       = 9
} PUBLIC_ACTION_TYPE_T;

/* Action Values defined for HT Category */
typedef enum {NOTIFY_CHANNEL_WIDTH_TYPE   = 0,
              SM_POWER_SAVE_TYPE          = 1,
              PSMP_ACTION_TYPE            = 2,
              SET_PCO_PHASE_TYPE          = 3,
              MIMO_CSI_MATRICES_TYPE      = 4,
              MIMO_NON_COMPRESSED_BF_TYPE = 5,
              MIMO_COMPRESSED_BF_TYPE     = 6,
              ANT_SEL_INDICES_FB_TYPE     = 7,
              HT_INFO_EXCHANGE_TYPE       = 8
} HT_ACTION_TYPE_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 set_ssid_element(UWORD8* data, UWORD16 index, WORD8 *ssid);
extern UWORD8 set_bcast_ssid(UWORD8* data, UWORD16 index);
extern UWORD8 set_sup_rates_element(UWORD8* data, UWORD16 index);
extern UWORD8 set_exsup_rates_element(UWORD8* data, UWORD16 index);
extern FRAMECLASS_T get_frame_class(UWORD8* header);
extern UWORD8 check_erp_rates(UWORD8 *entry, UWORD8 *d, UWORD16 i);
extern UWORD16 prepare_deauth(UWORD8* data, UWORD8* da, UWORD16 reason_code);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function extracts the 'frame type' bits from the MAC header of the   */
/* input frame.                                                              */
/* Returns the value in the LSB of the returned value.                       */
INLINE BASICTYPE_T get_type(UWORD8* header)
{
    return ((BASICTYPE_T)(header[0] & 0x0C));
}

/* This function extracts the 'frame type and sub type' bits from the MAC    */
/* header of the input frame.                                                */
/* Returns the value in the LSB of the returned value.                       */
INLINE TYPESUBTYPE_T get_sub_type(UWORD8* header)
{
    return ((TYPESUBTYPE_T)(header[0] & 0xFC));
}

/* This function extracts the 'to ds' bit from the MAC header of the input   */
/* frame.                                                                    */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_to_ds(UWORD8* header)
{
    return (header[1] & 0x01);
}

/* This function sets the 'to ds' bit in the MAC header of the input frame   */
/* to the given value stored in the LSB bit.                                 */
/* The bit position of the 'to ds' in the 'frame control field' of the MAC   */
/* header is represented by the bit pattern 0x00000001                       */
INLINE void set_to_ds(UWORD8* header, UWORD8 to_ds)
{
    header[1] &= 0xFE;
    header[1] |= to_ds;
}

/* This function extracts the 'from ds' bit from the MAC header of the input */
/* frame.                                                                    */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_from_ds(UWORD8* header)
{
    return ((header[1] & 0x02) >> 1);
}

/* This function sets the 'from ds' bit in the MAC header of the input frame */
/* to the given value stored in the LSB bit.                                 */
/* The bit position of the 'from ds' in the 'frame control field' of the MAC */
/* header is represented by the bit pattern 0x00000010.                      */
INLINE void set_from_ds(UWORD8* header, UWORD8 from_ds)
{
    header[1] &= 0xFD;
    header[1] |= (from_ds << 1);
}


/* This function extracts the 'wep' bit from the MAC header of the input     */
/* frame.                                                                    */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_wep(UWORD8* header)
{
    return ((header[1] & 0x40) >> 6);
}

/* This function sets the 'wep' bit in the MAC header of the input frame to  */
/* the LSB of the given value.                                               */
/* The bit position of the 'wep' bit in the 'frame control field' of the MAC */
/* header is represented by the bit pattern 0x01000000.                      */
INLINE void set_wep(UWORD8* header, UWORD8 wep)
{
    header[1] &= 0xBF;
    header[1] |= (wep << 6);
}

/* This function sets the 'frame control' bits in the MAC header of the      */
/* input frame to the given 16-bit value.                                    */
INLINE void set_frame_control(UWORD8* header, UWORD16 fc)
{
   header[0] = (UWORD8)(fc & 0x00FF);
   header[1] = (UWORD8)(fc >> 8);
}

/* This function sets the 'duration id' value in the MAC header of the input */
/* frame.                                                                    */
INLINE void set_durationID(UWORD8* header, UWORD16 dur)
{
   header[2] = (UWORD8)(dur & 0x00FF);
   header[3] = (UWORD8)(dur >> 8);
}

/* This function extracts the MAC Address in 'address1' field of the MAC     */
/* header and updates the MAC Address in the allocated 'addr' variable.      */
INLINE void get_address1(UWORD8* msa, UWORD8* addr)
{
    memcpy(addr, msa + 4, 6);
}

/* This function returns the pointer to the MAC Address in 'Address-1" field */
/* of the MAC header.                                                        */
INLINE UWORD8 *get_address1_ptr(UWORD8* msa)
{
    return (msa + 4);
}

/* This function sets the 'address1' field in the MAC header of the input    */
/* frame to the input MAC Address 'addr'. The 16 LSB bits of 'addr' are      */
/* ignored.                                                                  */
INLINE void set_address1(UWORD8* msa, UWORD8* addr)
{
    memcpy(msa + 4, addr, 6);
}

/* This function extracts the MAC Address in 'address2' field of the MAC     */
/* header and updates the MAC Address in the allocated 'addr' variable.      */
INLINE void get_address2(UWORD8* msa, UWORD8* addr)
{
    memcpy(addr, msa + 10, 6);
}

/* This function returns the pointer to the MAC Address in 'Address-2" field */
/* of the MAC header.                                                        */
INLINE UWORD8 *get_address2_ptr(UWORD8* msa)
{
    return (msa + 10);
}

/* This function sets the 'address2' field in the MAC header of the input    */
/* frame to the input MAC Address 'addr'. The 16 LSB bits of 'addr' are      */
/* ignored.                                                                  */
INLINE void set_address2(UWORD8* msa, UWORD8* addr)
{
    memcpy(msa + 10, addr, 6);
}

/* This function extracts the MAC Address in 'address3' field of the MAC     */
/* header and updates the MAC Address in the allocated 'addr' variable.      */
INLINE void get_address3(UWORD8* msa, UWORD8* addr)
{
    memcpy(addr, msa + 16, 6);
}

/* This function returns the pointer to the MAC Address in 'Address-3" field */
/* of the MAC header.                                                        */
INLINE UWORD8 *get_address3_ptr(UWORD8* msa)
{
    return (msa + 16);
}

/* This function sets the 'address3' field in the MAC header of the input    */
/* frame to the input MAC Address 'addr'. The 16 LSB bits of 'addr' are      */
/* ignored.                                                                  */
INLINE void set_address3(UWORD8* msa, UWORD8* addr)
{
    memcpy(msa + 16, addr, 6);
}

/* This function extracts the 'fragment number' value from the MAC header of */
/* the input frame.                                                          */
/* Returns a 8 bit value with the lower 4 bits valid and containing the      */
/* fragment number.                                                          */
INLINE UWORD8 get_fragment_number(UWORD8* header)
{
    return (header[22] & 0x0F);
}

/* This function sets the 'fragment number' value in the MAC header of the   */
/* input frame to the lower 4 bits of the input fragment number.             */
INLINE void set_fragment_number(UWORD8* header, UWORD8 frag_num)
{
    header[22] &= 0xF0;
    header[22] |= (frag_num & 0x0F);
}

/* This function extracts the BSSID from the incoming WLAN packet based on   */
/* the 'from ds' bit, and updates the MAC Address in the allocated 'addr'    */
/* variable.                                                                 */
INLINE void get_BSSID(UWORD8* data, UWORD8* bssid)
{
    if(get_from_ds(data) == 1)
        get_address2(data, bssid);
    else if(get_to_ds(data) == 1)
        get_address1(data, bssid);
    else
        get_address3(data, bssid);
}

/* This function returns the pointer to BSSID from the incoming WLAN packet */
/* based on the 'from ds' bit.                                              */
INLINE UWORD8 *get_BSSID_ptr(UWORD8* data)
{
    if(get_from_ds(data) == 1)
        return get_address2_ptr(data);
    else if(get_to_ds(data) == 1)
        return get_address1_ptr(data);

    return get_address3_ptr(data);
}

/* This funcion extracts the authentication sequence number from the         */
/* incoming authentication frame.                                            */
/* Returns UWORD16, authentication sequence number                           */
INLINE UWORD16 get_auth_seq_num(UWORD8* data)
{
    UWORD16 auth_seq = 0;

    auth_seq = data[MAC_HDR_LEN + 3];
    auth_seq = (auth_seq << 8) | data[MAC_HDR_LEN + 2];

    return auth_seq;
}

/* This funcion extracts the authentication status code from the incoming    */
/* authentication frame.                                                     */
/* Returns UWORD16, authentication status code                               */
INLINE UWORD16 get_auth_status(UWORD8* data)
{
    UWORD16 auth_status = 0;

    auth_status = data[MAC_HDR_LEN + 5];
    auth_status = (auth_status << 8) | data[MAC_HDR_LEN + 4];

    return auth_status;
}


/* This funcion extracts the authentication algorithm number from the        */
/* incoming authentication frame.                                            */
INLINE UWORD8* get_auth_ch_text(UWORD8* data)
{
    return &(data[MAC_HDR_LEN + 6]);
}


/* This funcion extracts the association status code from the incoming       */
/* association response frame and returns association status code            */
INLINE UWORD16 get_asoc_status(UWORD8* data)
{
    UWORD16 asoc_status = 0;

    asoc_status = data[MAC_HDR_LEN + 3];
    asoc_status = (asoc_status << 8) | data[MAC_HDR_LEN + 2];

    return asoc_status;
}

/* This function extracts the SSID from a beacon/probe response frame        */
INLINE void get_ssid(UWORD8* data, UWORD8* ssid)
{
    UWORD8 len = 0;
    UWORD8 i   = 0;
    UWORD8 j   = 0;

    len = data[MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN +
                                                 CAP_INFO_LEN + 1];
    j   = MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN +
                                                  CAP_INFO_LEN + 2;

    /* If the SSID length field is set wrongly to a value greater than the   */
    /* allowed maximum SSID length limit, reset the length to 0              */
    if(len >= MAX_SSID_LEN)
       len = 0;

    for(i = 0; i < len; i++, j++)
        ssid[i] = data[j];

    ssid[len] = '\0';
}

/* This function compares the address with the (last bit on air) BIT24 to    */
/* determine if the address is a group address.                              */
/* Returns BTRUE if the input address has the group bit set.                 */
INLINE BOOL_T is_group(UWORD8* addr)
{
    if((addr[0] & BIT0) != 0)
        return BTRUE;

    return BFALSE;
}

/* This function extracts the destination MAC Address from the incoming WLAN */
/* packet based on the 'to ds' bit,and updates the MAC Address in the        */
/* allocated 'addr' variable.                                                */
INLINE void get_DA(UWORD8* data, UWORD8* da)
{
    if(get_to_ds(data) == 1)
        get_address3(data, da);
    else
        get_address1(data, da);
}

/* This function extracts the destination MAC Address from the incoming WLAN */
/* packet based on the 'to ds' bit and returns the pointer to the same.      */
INLINE UWORD8 *get_DA_ptr(UWORD8* data)
{
    if(get_to_ds(data) == 1)
        return get_address3_ptr(data);

    return get_address1_ptr(data);
}

/* This function extracts the source MAC Address from the incoming WLAN      */
/* packet based on the 'from ds' bit, and updates the MAC Address in the     */
/* allocated 'addr' variable.                                                */
INLINE void get_SA(UWORD8* data, UWORD8* sa)
{
    if(get_from_ds(data) == 1)
        get_address3(data, sa);
    else
        get_address2(data, sa);
}

/* This function extracts the source MAC Address from the incoming WLAN      */
/* packet based on the 'from ds' bit and returns the pointer to the same.    */
INLINE UWORD8 *get_SA_ptr(UWORD8* data)
{
    if(get_from_ds(data) == 1)
        return get_address3_ptr(data);

    return get_address2_ptr(data);
}

/* This function extracts the Transmitter MAC Address from the incoming WLAN */
/* packet                                                                    */
INLINE void get_TA(UWORD8* data, UWORD8* ta)
{
    get_address2(data, ta);
}

/* This function extracts the Transmitter MAC Address from the incoming WLAN */
/* packet and returns the pointer to the same.                               */
INLINE UWORD8 *get_TA_ptr(UWORD8* data)
{
    return get_address2_ptr(data);
}

/* This function extracts the 'protocol version' bits from the MAC header of */
/* the input frame.                                                          */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_protocol_version(UWORD8* header)
{
    return header[0] & 0x03;
}

/* This function extracts the 'retry' bit from the MAC header of the input   */
/* frame.                                                                    */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_retry(UWORD8* header)
{
    return ((header[1] & 0x08) >> 3);
}

/* This function extracts the 'sequence number' value from the MAC header of */
/* the input frame.                                                          */
/* Returns a 16 bit value with the lower 12 bits valid and containing the    */
/* sequence number.                                                          */
INLINE UWORD16 get_sequence_number(UWORD8* header)
{
    UWORD16 seq_num = 0;

    seq_num   = header[23];
    seq_num <<= 4;
    seq_num  |= (header[22] >> 4);

    return seq_num;
}

/* This function extracts the 'more frag' bit from the MAC header of the     */
/* input frame.                                                              */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_more_frag(UWORD8* header)
{
    return ((header[1] & 0x04) >> 2);
}

INLINE UWORD8 set_mac_hdr(UWORD8 *mac_hdr)
{
    set_frame_control(mac_hdr, DATA);

    return MAC_HDR_LEN;
}

/* This funcion extracts the authentication algorithm number from the        */
/* incoming authentication frame.                                            */
INLINE UWORD16 get_auth_algo_num(UWORD8* data)
{
    UWORD16 auth_algo = 0;

    auth_algo = data[MAC_HDR_LEN + 1];
    auth_algo = (auth_algo << 8) | data[MAC_HDR_LEN];

    return auth_algo;
}

/* This function extracts the capability info field from the beacon or probe */
/* response frame.                                                           */
INLINE UWORD16 get_cap_info(UWORD8* data)
{
    UWORD16 cap_info = 0;
    UWORD16 index    = MAC_HDR_LEN;
    TYPESUBTYPE_T st = BEACON;

    st = get_sub_type(data);

    /* Location of the Capability field is different for Beacon and */
    /* Association frames.                                          */
    if((st == BEACON) || (st == PROBE_RSP))
        index += TIME_STAMP_LEN + BEACON_INTERVAL_LEN;

    cap_info  = data[index];
    cap_info |= (data[index + 1] << 8);

    return cap_info;
}

/* This function extracts the 'power management' bit from the MAC header of  */
/* the input frame.                                                          */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_pwr_mgt(UWORD8* header)
{
    return ((header[1] & 0x10) >> 4);
}

/* This function sets the 'power management' bit in the MAC header of the    */
/* input frame to the LSB of the given value.                                */
/* The bit position of the 'power management' bit in the 'frame control      */
/* field' of the MAC header is represented by the bit pattern 0x00010000.    */
INLINE void set_pwr_mgt(UWORD8* header, UWORD8 pwr_mgt)
{
    header[1] &= 0xEF;
    header[1] |= (pwr_mgt << 4);
}

/* This function extracts the 'more data' bit from the MAC header of the     */
/* input frame.                                                              */
/* Returns the value in the LSB of the returned value.                       */
INLINE UWORD8 get_more_data(UWORD8* header)
{
    return ((header[1] & 0x20) >> 5);
}

/* This function sets the 'more data' bit in the MAC header of the input     */
/* frame to the LSB of the given value.                                      */
/* The bit position of the 'more data' bit in the 'frame control field' of   */
/* the MAC header is represented by the bit pattern 0x00100000.              */
INLINE void set_more_data(UWORD8* header, UWORD8 more_data)
{
    header[1] &= 0xDF;
    header[1] |= (more_data << 5);
}

/* This function extracts the 'order' bit from the frame control field  */
/* within the MAC header.                                               */
INLINE UWORD8 get_order_bit(UWORD8 *header)
{
    return ((header[1] & 0x80) >> 7);

}

/* This function sets the 'order' bit in the frame control field within the */
/* MAC header.                                                              */
INLINE void set_order_bit(UWORD8 *header, UWORD8 order)
{
    header[1] &= 0x7F;
    header[1] |= (order << 7);
}

/* This function sets the 'frame type and sub type' bits to the MAC header   */
INLINE void set_sub_type(UWORD8* header, UWORD8 val)
{
    header[0] = (val & 0xFC);
}

#endif /* FRAME_H */
