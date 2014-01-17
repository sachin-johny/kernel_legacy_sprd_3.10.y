/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : frame_p2p.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for checking the p2p frames and adding of  */
/*                      various attribute                                    */
/*                                                                           */
/*  List of Functions : is_p2p_pub_action_frame                              */
/*                      is_p2p_gen_action_frame                              */
/*                      is_p2p_ie                                            */
/*                      add_p2p_ie_hdr                                       */
/*                      add_status_attr                                      */
/*                      add_minorreasoncode_attr                             */
/*                      add_p2p_capability_attr                              */
/*                      add_p2p_device_id_attr                               */
/*                      add_GO_intent_attr                                   */
/*                      add_config_timout_attr                               */
/*                      add_listen_chan_attr                                 */
/*                      add_grp_bssid_attr                                   */
/*                      add_int_p2p_if_addr_attr                             */
/*                      add_p2p_chan_list_attr                               */
/*                      add_NOA_attr                                         */
/*                      add_p2p_grp_id_attr                                  */
/*                      add_p2p_oper_chan_attr                               */
/*                      add_p2p_invit_flag_attr                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_P2P

#ifndef FRAME_P2P_H
#define FRAME_P2P_H
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mib_p2p.h"
#include "mgmt_p2p.h"
#include "phy_hw_if.h"
/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/
#ifdef P2P_NO_COUNTRY
/* MD-TBD : To be removed when using Regulatory domains */
extern UWORD8 g_p2p_chan_entry_list[MAX_CHANNEL_FREQ_2 + 2];
#endif /* P2P_NO_COUNTRY */
extern UWORD8 g_p2p_len_chan_entry_list;

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/
#define WFA_OUI_BYTE1  0x50
#define WFA_OUI_BYTE2  0x6F
#define WFA_OUI_BYTE3  0x9A
#define WFA_P2P_v1_0   0x09

#define P2P_OUI_LEN         4
#define P2P_IE_HDR_LEN      6
#define P2P_ELEMENT_ID_SIZE 1
#define P2P_ATTR_ID_SIZE    1
#define P2P_ATTR_HDR_LEN    3

#define P2P_MAX_IE_BODY_SIZE 251
#define P2P_MAX_IE_LEN       (P2P_MAX_IE_BODY_SIZE + P2P_OUI_LEN)
#define MAX_COMB_P2P_ATTR_LEN 700

/* BIT fields for P2P Group Capability */
#define P2PCAPBIT_GO             BIT0
#define P2PCAPBIT_PERS_GRP       BIT1
#define P2PCAPBIT_GRP_LMT        BIT2
#define P2PCAPBIT_INTRA_BSS_DIST BIT3
#define P2PCAPBIT_PERS_RECNT     BIT5
#define P2PCAPBIT_GRP_FORM       BIT6

/* BIT fields for device capability */
#define P2P_CLIENT_DISC     BIT1
#define P2P_INVIT_PROC      BIT5

/* Channel List */
#ifdef P2P_NO_COUNTRY
#define P2P_COUNTRY_STRING_BYTE1   'X'
#define P2P_COUNTRY_STRING_BYTE2   'X'
#define P2P_COUNTRY_STRING_BYTE3   0x04
#define P2P_OPERATING_CLASS1       81
#endif /* P2P_NO_COUNTRY */

/* WSC Attribute ID */
#define WPS_ATTR_DEV_NAME_P2P      0x1011

/* The destination address and the dialog token to be added in the Device */
/* Discovery Resp frame is stored at the end of the GO Discovery Req frm  */
/* after FCS. P2P_DD_RSP_DST_ADR_OFFSET is the offset to the destination  */
/* address of the DD resp frame                                           */
#define P2P_DD_RSP_DST_ADR_OFFSET  (P2P_GEN_ACT_DIALOG_TOKEN_OFF + FCS_LEN + 1)

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/
#define GET_ATTR_LEN(buf_ptr)   (MAKE_WORD16((buf_ptr)[1], (buf_ptr)[2]))
#define P2P_IE_LEN(buf_ptr)     ((UWORD8)(*(buf_ptr + ELEMENT_ID_SIZE)))

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {
    P2P_STATUS             =  0,
    P2P_MINOR_REASON_CODE  =  1,
    P2P_CAPABILITY         =  2,
    P2P_DEVICE_ID          =  3,
    GROUP_OWNER_INTENT     =  4,
    CONFIG_TIMEOUT         =  5,
    LISTEN_CHANNEL         =  6,
    P2P_GROUP_BSSID        =  7,
    EXTENDED_LISTEN_TIMING =  8,
    INTENDED_P2P_IF_ADDR   =  9,
    P2P_MANAGEABILITY      =  10,
    P2P_CHANNEL_LIST       =  11,
    NOTICE_OF_ABSENCE      =  12,
    P2P_DEVICE_INFO        =  13,
    P2P_GROUP_INFO         =  14,
    P2P_GROUP_ID           =  15,
    P2P_INTERFACE          =  16,
    P2P_OPERATING_CHANNEL  =  17,
    INVITATION_FLAGS       =  18
} ATTRIBUTE_ID_T;

typedef enum {
    P2P_GO_NEG_REQ     = 0,
    P2P_GO_NEG_RSP     = 1,
    P2P_GO_NEG_CNF     = 2,
    P2P_INV_REQ        = 3,
    P2P_INV_RSP        = 4,
    P2P_DEV_DISC_REQ   = 5,
    P2P_DEV_DISC_RSP   = 6,
    P2P_PROV_DISC_REQ  = 7,
    P2P_PROV_DISC_RSP  = 8
} P2P_PUB_ACTION_FRM_TYPE;

typedef enum {
    P2P_NOA           = 0,
    P2P_PRESENCE_REQ  = 1,
    P2P_PRESENCE_RESP = 2,
    GO_DISC_REQ       = 3
} P2P_GEN_ACTION_FRM_TYPE;

typedef enum {
    P2P_PUB_ACT_OUI_OFF1         = 2,
    P2P_PUB_ACT_OUI_OFF2         = 3,
    P2P_PUB_ACT_OUI_OFF3         = 4,
    P2P_PUB_ACT_OUI_TYPE_OFF     = 5,
    P2P_PUB_ACT_OUI_SUBTYPE_OFF  = 6,
    P2P_PUB_ACT_DIALOG_TOKEN_OFF = 7,
    P2P_PUB_ACT_TAG_PARAM_OFF    = 8
} P2P_PUB_ACT_FRM_OFF;

typedef enum {
    P2P_GEN_ACT_OUI_OFF1         = 1,
    P2P_GEN_ACT_OUI_OFF2         = 2,
    P2P_GEN_ACT_OUI_OFF3         = 3,
    P2P_GEN_ACT_OUI_TYPE_OFF     = 4,
    P2P_GEN_ACT_OUI_SUBTYPE_OFF  = 5,
    P2P_GEN_ACT_DIALOG_TOKEN_OFF = 6,
    P2P_GEN_ACT_TAG_PARAM_OFF    = 7
} P2P_GEN_ACT_FRM_OFF;

typedef enum{P2P_STAT_SUCCESS           = 0,
             P2P_STAT_INFO_UNAVAIL      = 1,
             P2P_STAT_INCOMP_PARAM      = 2,
             P2P_STAT_LMT_REACHED       = 3,
             P2P_STAT_INVAL_PARAM       = 4,
             P2P_STAT_UNABLE_ACCO_REQ   = 5,
             P2P_STAT_PREV_PROT_ERROR   = 6,
             P2P_STAT_NO_COMMON_CHAN    = 7,
             P2P_STAT_UNKNW_P2P_GRP     = 8,
             P2P_STAT_GO_INTENT_15      = 9,
             P2P_STAT_INCOMP_PROV_ERROR = 10,
             P2P_STAT_USER_REJECTED     = 11
} P2P_STATUS_CODE_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern UWORD16 add_p2p_ie_probe_rsp(UWORD8 *data, UWORD16 index);
extern UWORD8  get_p2p_dev_cap(void);
extern UWORD8  get_p2p_grp_cap(void);
extern UWORD16 add_p2p_device_info_attr(UWORD8 *ie_ptr, UWORD16 index);
extern UWORD16 add_p2p_grp_info_attr(UWORD8 *ie_ptr, UWORD16 index);
extern UWORD16 is_p2p_frame(UWORD8 *msa, UWORD16 index, UWORD16 rx_len);
extern void    p2p_send_inv_rsp(UWORD8 *msa, P2P_STATUS_CODE_T status,
                                p2p_persist_list_t *persist_list);
extern void add_p2p_mgmt_frame_hdr(UWORD8* frm_ptr, UWORD8* p2p_dev_id);
extern UWORD16 add_p2p_gen_act_hdr(UWORD8* frm_ptr, UWORD8 dlog_tkn,
                            P2P_GEN_ACTION_FRM_TYPE oui_subtype);
extern UWORD16 add_p2p_pub_act_hdr(UWORD8* frm_ptr, UWORD8 dlog_tkn,
                            P2P_PUB_ACTION_FRM_TYPE oui_subtype);
extern UWORD16 p2p_get_operating_class(UWORD8 ch_num);
extern void p2p_send_inv_req(UWORD8 is_persit, UWORD8 go_config,
                             UWORD8 cl_config, UWORD8 *ssid,
                             UWORD8 *go_dev_addr, UWORD8 oper_chan);
extern void send_prov_disc_resp(UWORD8 *msa, UWORD16 config_method,
                                UWORD8 *sa);
extern UWORD16 add_p2p_ie_probe_req(UWORD8 *data, UWORD16 index);
/*****************************************************************************/
/* In-line Functions                                                         */
/*****************************************************************************/

/********************** Functions to check conditions ************************/

/* This function checks if the frame is a p2p public action frame */
INLINE BOOL_T is_p2p_pub_action_frame(UWORD8* data)
{
    if((PUBLIC_CATEGORY    == data[CATEGORY_OFFSET]) &&
       (VENDOR_PUBLIC_ACT  == data[ACTION_OFFSET]) &&
       (WFA_OUI_BYTE1      == data[P2P_PUB_ACT_OUI_OFF1]) &&
       (WFA_OUI_BYTE2      == data[P2P_PUB_ACT_OUI_OFF2]) &&
       (WFA_OUI_BYTE3      == data[P2P_PUB_ACT_OUI_OFF3]) &&
       (WFA_P2P_v1_0       == data[P2P_PUB_ACT_OUI_TYPE_OFF]))
        return BTRUE;
    else
        return BFALSE;
}

/* This function checks if the frame is a p2p general action frame */
INLINE BOOL_T is_p2p_gen_action_frame(UWORD8* data)
{
    if((VENDOR_CATEGORY    == data[CATEGORY_OFFSET]) &&
       (WFA_OUI_BYTE1      == data[P2P_GEN_ACT_OUI_OFF1]) &&
       (WFA_OUI_BYTE2      == data[P2P_GEN_ACT_OUI_OFF2]) &&
       (WFA_OUI_BYTE3      == data[P2P_GEN_ACT_OUI_OFF3]) &&
       (WFA_P2P_v1_0       == data[P2P_GEN_ACT_OUI_TYPE_OFF]))
        return BTRUE;
    else
        return BFALSE;
}

/* This funtion checks the various fields of the IE and returns if the IE is */
/* valid or not                                                              */
INLINE BOOL_T is_p2p_ie(UWORD8 *ie)
{
    if((IP2P          == ie[0]) &&
       (WFA_OUI_BYTE1 == ie[2]) &&
       (WFA_OUI_BYTE2 == ie[3]) &&
       (WFA_OUI_BYTE3 == ie[4]) &&
       (WFA_P2P_v1_0  == ie[5]))
        return BTRUE;
    else
        return BFALSE;
}

/********************** Functions to add frame data  *************************/

/* This function add the P2P IE header to the frame */
INLINE UWORD8 add_p2p_ie_hdr(UWORD8* data)
{
   data[0] = IP2P;
   data[2] = WFA_OUI_BYTE1;
   data[3] = WFA_OUI_BYTE2;
   data[4] = WFA_OUI_BYTE3;
   data[5] = WFA_P2P_v1_0;

   return P2P_IE_HDR_LEN;
}

/* This function adds the status attribute for the P2P IE and returns the    */
/* total length of the attribute including the header                        */
INLINE UWORD16 add_status_attr(UWORD8 *ie_ptr, UWORD16 index,
                                                P2P_STATUS_CODE_T status)
{
    UWORD16 attr_len  = 0;

    ie_ptr[index++] = P2P_STATUS; /* Attribute ID     */
    attr_len        = 1;          /* Attribute Length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    ie_ptr[index++] = status;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the minor reason code attribute for the P2P IE and     */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_minorreasoncode_attr(UWORD8 *ie_ptr, UWORD16 index,
                                                 UWORD8 reason_code)
{
    UWORD16 attr_len = 0;

    ie_ptr[index++] = P2P_MINOR_REASON_CODE; /* Attribute ID     */
    attr_len        = 1;                     /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    ie_ptr[index++] = reason_code;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the p2p capability attribute for the P2P IE and        */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_capability_attr(UWORD8 *ie_ptr, UWORD16 index,
                                       UWORD8 p2p_dev_cap, UWORD8  p2p_grp_cap)
{
    UWORD16 attr_len    = 0;


    ie_ptr[index++] = P2P_CAPABILITY;     /* Attribute ID     */
    attr_len        = 2;                  /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    ie_ptr[index++] = p2p_dev_cap;
    ie_ptr[index++] = p2p_grp_cap;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the P2P device ID attribute for the P2P IE and         */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_device_id_attr(UWORD8 *ie_ptr, UWORD16 index,
                                      UWORD8 *dev_addr)
{
    UWORD16 attr_len = 0;

    ie_ptr[index++] = P2P_DEVICE_ID;      /* Attribute ID     */
    attr_len        = MAC_ADDRESS_LEN;    /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    /* Copy the MAC address of the station */
    mac_addr_cpy((ie_ptr + index), dev_addr);

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the GO intent attribute for the P2P IE and returns the */
/* total length of the attribute including the header                        */
INLINE UWORD16 add_GO_intent_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD8  go_intent = 0;
    UWORD16 attr_len  = 0;

    ie_ptr[index++] = GROUP_OWNER_INTENT; /* Attribute ID     */
    attr_len        = 1;                  /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    if(1 == g_GO_tie_breaker)
    {
        go_intent |= BIT0;
    }

    /* Toggel the tie breaker */
    g_GO_tie_breaker ^= 1;

    go_intent |= mget_p2p_GO_intent_val() << 1;

    ie_ptr[index++] = go_intent;

    return (attr_len + P2P_ATTR_HDR_LEN);
}


/* This function adds the configuration timeout attribute for the P2P IE and */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_config_timout_attr(UWORD8 *ie_ptr, UWORD16 index,
                                      UWORD8 go_config, UWORD8 client_config)
{
    UWORD16 attr_len = 0;

    ie_ptr[index++] = CONFIG_TIMEOUT;     /* Attribute ID     */
    attr_len        = 2;                  /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    ie_ptr[index++] = go_config;
    ie_ptr[index++] = client_config;

    return (attr_len + P2P_ATTR_HDR_LEN);
}


/* This function adds the listen channel attribute for the P2P IE and        */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_listen_chan_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD8  listen_chan = 0;
    UWORD16 attr_len    = 0;

    ie_ptr[index++] = LISTEN_CHANNEL;     /* Attribute ID     */
    attr_len        = 5;                  /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    /* Add the country string */
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE1;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE2;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE3;

    /* Get the listen channel */
    listen_chan     =  get_ch_num_from_idx(get_current_start_freq(),
                                           mget_p2p_listen_chan());

    /* Get the operating class of the listen channel */
    ie_ptr[index++] =  p2p_get_operating_class(listen_chan);

    /* Add the listen channel */
    ie_ptr[index++] =  listen_chan;

    return (attr_len + P2P_ATTR_HDR_LEN);
}


/* This function adds the Group BSSID attribute for the P2P IE and           */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_grp_bssid_attr(UWORD8 *ie_ptr, UWORD16 index, UWORD8 *bssid)
{
    UWORD16 attr_len = 0;

    ie_ptr[index++] = P2P_GROUP_BSSID;    /* Attribute ID      */
    attr_len        = MAC_ADDRESS_LEN;    /* Attribute length  */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;


    mac_addr_cpy((ie_ptr + index), bssid);

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the P2P interface address attribute for the P2P IE and */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_int_p2p_if_addr_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD16 attr_len = 0;

    ie_ptr[index++] = INTENDED_P2P_IF_ADDR;    /* Attribute ID      */
    attr_len        = MAC_ADDRESS_LEN;         /* Attribute length  */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    mac_addr_cpy((ie_ptr + index), get_p2p_if_address());

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the channel list attribute for the P2P IE and          */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_chan_list_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD16 attr_len          = 0;
    UWORD16 attr_start_offset = index;

    ie_ptr[index] = P2P_CHANNEL_LIST; /* Attribute ID */
    index += P2P_ATTR_HDR_LEN;

    /* Add the country string */
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE1;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE2;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE3;

    /* Copy the channel entry list */
    memcpy((ie_ptr + index), &g_p2p_chan_entry_list,
                              g_p2p_len_chan_entry_list);

    index += g_p2p_len_chan_entry_list;

    /* Add the attribute len */
    attr_len = index - attr_start_offset - P2P_ATTR_HDR_LEN ;
    ie_ptr[attr_start_offset + 1] = attr_len & 0x00FF;
    ie_ptr[attr_start_offset + 2] = attr_len >> 8;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the Notice of Absence attribute for the P2P IE and     */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_NOA_attr(UWORD8 *ie_ptr, UWORD16 index, BOOL_T force_add)
{
    UWORD8 cnt       = 0;
    UWORD8 ctw_ops   = 0;
    UWORD16 attr_len = 0;

    /* Check if NOA attribute has to be added even if the NOA descriptor is  */
    /* NULL.                                                                 */
    /* Note : This check is required because in case of presence response NOA*/
    /* attribute should be present even id NOA descriptor is NULL            */
    if(force_add == BFALSE)
    {
    /* This attribute is not set if there Opportunistic Power Save is not    */
    /* enabled and there is no NOA schedules currently set up.               */
        if((get_opp_ps() == BFALSE) && (get_num_noa_sched() == 0))
    	{
        	return 0;
    	}
    }

    ie_ptr[index++] = NOTICE_OF_ABSENCE;              /* Attribute ID      */
    attr_len        = (get_num_noa_sched() * 13) + 2; /* Attribute length  */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    /* Update the NoA index */
    ie_ptr[index++] = get_noa_index();
    
    /* CTWindow and Opportunity parameters field */
    if(mget_p2p_CTW() != 0)
    {
        ctw_ops = mget_p2p_CTW();
        ctw_ops |= BIT7;
    }

    ie_ptr[index++] = ctw_ops;

    /* Add NOA descriptors (for NOA schedules currently set up) */
    for(cnt = 0; cnt < get_num_noa_sched(); cnt++)
    {
         ie_ptr[index++] = g_noa_dscr[cnt].cnt_typ;

         ie_ptr[index++] = g_noa_dscr[cnt].duration & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].duration >> 8) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].duration >> 16) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].duration >> 24);

         ie_ptr[index++] = g_noa_dscr[cnt].interval & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].interval >> 8) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].interval >> 16) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].interval >> 24);

         ie_ptr[index++] = g_noa_dscr[cnt].start_time & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].start_time >> 8) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].start_time >> 16) & 0x000000FF;
         ie_ptr[index++] = (g_noa_dscr[cnt].start_time >> 24);
    }

    return (attr_len + P2P_ATTR_HDR_LEN);
}


/* This function adds the P2P group id attribute for the P2P IE and          */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_grp_id_attr(UWORD8 *ie_ptr, UWORD16 index,
                                   UWORD8 *dev_addr, UWORD8 *grp_ssid)
{
    UWORD8  ssid_len          = 0;
    UWORD16 attr_len          = 0;
    UWORD16 attr_start_offset = index;

    ie_ptr[index] = P2P_GROUP_ID;        /* Attribute ID       */
    index += P2P_ATTR_HDR_LEN;

    mac_addr_cpy((ie_ptr + index), dev_addr);
    index += MAC_ADDRESS_LEN;
    ssid_len = strlen((WORD8 *)grp_ssid);
    memcpy((ie_ptr + index), grp_ssid, ssid_len);
    index += ssid_len;

    attr_len = index - attr_start_offset - P2P_ATTR_HDR_LEN ;
    ie_ptr[attr_start_offset + 1] = attr_len & 0x00FF;
    ie_ptr[attr_start_offset + 2] = attr_len >> 8;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the operating channel attribute for the P2P IE and     */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_oper_chan_attr(UWORD8 *ie_ptr, UWORD16 index,
                                      UWORD8 oper_chan)
{
    UWORD8  oper_chan_num = 0;
    UWORD16 attr_len      = 0;

    ie_ptr[index++] = P2P_OPERATING_CHANNEL; /* Attribute ID     */
    attr_len        = 5;                     /* Attribute length */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    /* Add the country string */
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE1;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE2;
    ie_ptr[index++] = P2P_COUNTRY_STRING_BYTE3;

    /* Get the operating channel */
    oper_chan_num = get_ch_num_from_idx(get_current_start_freq(), oper_chan);

    /* Add the operating class for the operating channel */
    ie_ptr[index++] = p2p_get_operating_class(oper_chan_num);

    /* Add the operating channel */
    ie_ptr[index++] = oper_chan_num;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

/* This function adds the invitation flag attribute for the P2P IE and       */
/* returns the total length of the attribute including the header            */
INLINE UWORD16 add_p2p_invit_flag_attr(UWORD8 *ie_ptr, UWORD16 index,
                                       UWORD8 invit_flag)
{
    UWORD16 attr_len      = 0;

    ie_ptr[index++] = INVITATION_FLAGS;    /* Attribute ID      */
    attr_len        = 1;                   /* Attribute length  */
    ie_ptr[index++] = attr_len & 0x00FF;
    ie_ptr[index++] = attr_len >> 8;

    ie_ptr[index] = invit_flag;

    return (attr_len + P2P_ATTR_HDR_LEN);
}

#endif /* FRAME_P2P_H */
#endif /* MAC_P2P */
