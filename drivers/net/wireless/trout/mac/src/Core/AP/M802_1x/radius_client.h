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
/*  File Name         : radius_client.h                                      */
/*                                                                           */
/*  Description       : This file contains the radius client related         */
/*                      definitions.                                         */
/*                                                                           */
/*  List of Functions : hex_2_chr                                            */
/*                      radius_add_int                                       */
/*                      radius_add_field                                     */
/*                      radius_add_hwaddr                                    */
/*                      radius_find_attr                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/


#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

#ifndef RADIUS_CLIENT_H
#define RADIUS_CLIENT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ieee_auth_1x.h"
#include "imem_if.h"
#include "buff_desc.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define ARP_PKT_LEN                 42

#define RAD_SERVICE_TYPE_FRAMED     2
#define RAD_NAS_PORT_TYPE_WIRELESS  19  /* IEEE 802.11 */
#define RAND_HASH_LEN               16
#define USR_NAME_MAX_LEN            32
#define RAD_STATE_MAX_LEN           32
#define RAD_CLASS_MAX_LEN           32
#define RAD_SECRET_MAX_LEN          64
#define RADIUS_EAP_MAX_LEN          1400
#define EAP_MAX_HEADER_LEN          215

#define RADIUS_ATTR_OFFSET          20

#define RAD_VENDOR_MICROSOFT        311
#define RAD_KEYTYPE_SEND            16  /* MPPE-Send-Key */
#define RAD_KEYTYPE_RECV            17  /* MPPE-Recv-Key */
#define RAD_SERVER_MSG_MAX_LEN      1788

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/


typedef enum {RAD_ACCESS_REQUEST    = 1,
              RAD_ACCESS_ACCEPT     = 2,
              RAD_ACCESS_REJECT     = 3,
              RAD_ACCESS_CHALLENGE  = 11,
}RAD_PACKET_TYPE_T;

typedef enum {RAD_ATTR_USER_NAME                = 1,
              RAD_ATTR_USER_PASSWD              = 2,
              RAD_ATTR_NAS_IP_ADDRESS           = 4,
              RAD_ATTR_NAS_PORT                 = 5,
              RAD_ATTR_SERVICE_TYPE             = 6,
              RAD_ATTR_FRAMED_MTU               = 12,
              RAD_ATTR_REPLY_MESSAGE            = 18,
              RAD_ATTR_STATE                    = 24,
              RAD_ATTR_CLASS                    = 25,
              RAD_ATTR_VENDOR_SPECIFIC          = 26,
              RAD_ATTR_SESSION_TIMEOUT          = 27,
              RAD_ATTR_CALLED_STATION_ID        = 30,
              RAD_ATTR_CALLING_STATION_ID       = 31,
              RAD_ATTR_NAS_IDENTIFIER           = 32,
              RAD_ATTR_NAS_PORT_TYPE            = 61,
              RAD_ATTR_CONNECT_INFO             = 77,
              RAD_ATTR_EAP_MESSAGE              = 79,
              RAD_ATTR_MESSAGE_AUTHENTICATOR    = 80,
              RAD_ATTR_NAS_PORT_ID              = 87,
}RAD_ATTR_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8 auth_hash[RAND_HASH_LEN];
    UWORD8 supp_usr_name[USR_NAME_MAX_LEN];
    UWORD8 supp_usr_name_len;
    UWORD8 supp_state[RAD_STATE_MAX_LEN];
    UWORD8 supp_state_len;
    UWORD8 supp_class[RAD_CLASS_MAX_LEN];
    UWORD8 supp_class_len;
    UWORD8 tx_count;
    UWORD8 *rad_msg;
    UWORD16 rad_msg_len;
    UWORD8 supp_curmode;
    auth_t *auth_1x;
    UWORD8 req_id;
    BOOL_T resp_expected;
    ALARM_HANDLE_T *serv_timer;
} radius_node_t;

typedef struct resp_pending
{
    UWORD8 req_id;
    radius_node_t *rad_node;
    struct resp_pending *next_ptr;
}resp_pending_t;

/*****************************************************************************/
/* Extern Variables                                                          */
/*****************************************************************************/

extern UWORD8 g_radius_secret_len;
extern UWORD8 g_local_ip_addr[4];
extern UWORD8 g_server_ip_addr[4];
extern UWORD16 g_local_port;
extern UWORD16 g_server_port;

/*****************************************************************************/
/* Extern Functions                                                          */
/*****************************************************************************/

extern void stop_radius_client(void);
extern void free_rad_auth_node(UWORD8 aid);
extern void init_radius_client(UWORD8 *auth_mac_addr, UWORD8* auth_eth_addr,
                        UWORD8 *secret, UWORD8 secret_len, UWORD8 *local_ip,
                        UWORD16 local_port, UWORD8 *server_ip,
                        UWORD16 server_port, UWORD8 *server_hw_addr,
                        BOOL_T (** send_msg_server)(auth_t *auth_1x),
                        void (** update_identity)(auth_t *, UWORD8*, UWORD8));
extern void update_supp_identity(auth_t *auth_1x,UWORD8 *usr_name, UWORD8 len);
extern BOOL_T send_msg_2_server(auth_t *auth_1x);
extern void process_host_rx_radius(UWORD8 *pkt, UWORD32 len);
extern void send_arp_reply(UWORD8 *buffer, UWORD32 len);
extern BOOL_T is_radius_pkt(UWORD8 *pkt);
extern BOOL_T is_arp_pkt(UWORD8 *pkt);
extern radius_node_t *get_pending_node(UWORD8 id);

#ifdef ENABLE_AP_1X_LINUX_PROCESSING
extern BOOL_T start_radius_client(void);
extern void   halt_radius_thread(void);
extern void   update_radius_param(void);
extern BOOL_T send_frame_to_rad_server(UWORD8 *buffer, UWORD32 len);
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Convert char to hex */
INLINE UWORD8 hex_2_chr(UWORD8 val)
{
    return (val <= 9)? (val +'0') : ('a' +  val - 10);
}

/* This function adds a field with given length to the message body */
INLINE UWORD8 radius_add_int(UWORD8 *msg, UWORD8 attr, UWORD32 val)
{
    msg[0] = attr;
    msg[1] = 6;
    msg[2] = (val & 0xFF000000) >> 24;
    msg[3] = (val & 0x00FF0000) >> 16;
    msg[4] = (val & 0x0000FF00) >> 8;
    msg[5] = (val & 0x000000FF);
    return 6;
}

/* This function adds a field with given length to the message body */
INLINE UWORD8 radius_add_field(UWORD8 *msg, UWORD8 attr,
                                                      UWORD8 *val, UWORD8 len)
{
    UWORD8 i = 0;

    msg[0] = attr;
    msg[1] = len + 2;

    for(i = 0; i < len; i++)
    {
        msg[2 + i] = val[i];
    }

    return (len + 2);
}

/* This function adds a hw address with given attribute to the message body */
INLINE UWORD8 radius_add_hwaddr(UWORD8 *msg, UWORD8 attr, UWORD8 *val)
{
    UWORD8 buf[12] = {0};
    UWORD8 i       = 0;

    for(i = 0; i < 6; i++)
    {
        buf[2 * i]       = hex_2_chr((UWORD8)(val[i] & 0x0F));
        buf[(2 * i) + 1] = hex_2_chr((UWORD8)((val[i] & 0xF0) >> 4));
    }

    return radius_add_field(msg, attr, buf, 12);
}

/* This function adds a field with given length to the message body */
INLINE UWORD16 radius_find_attr(UWORD8 *msg, UWORD16 len, UWORD8 attr)
{
    UWORD16 index = 0;
    while(index < len)
    {
        if(msg[index] != attr)
        {
            index += msg[index + 1];
        }
        else
        {
            return index;
        }
    }
    return 0xFFFF;
}

#endif /* RADIUS_CLIENT_H */

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
