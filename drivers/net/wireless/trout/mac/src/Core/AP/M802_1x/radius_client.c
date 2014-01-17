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
/*  File Name         : radius_client.c                                      */
/*                                                                           */
/*  Description       : This file contains the functions for the radius      */
/*                      client interface.                                    */
/*                                                                           */
/*  List of Functions : init_radius_client                                   */
/*                      update_supp_identity                                 */
/*                      send_msg_2_server                                    */
/*                      radius_make_resp                                     */
/*                      is_arp_pkt                                           */
/*                      send_arp_reply                                       */
/*                      form_arp_rply_pkt                                    */
/*                      is_radius_pkt                                        */
/*                      process_host_rx_radius                               */
/*                      add_node_2_pending                                   */
/*                      get_pending_node                                     */
/*                      radius_check_auth                                    */
/*                      radius_check_msg_auth                                */
/*                      prep_radius_reply                                    */
/*                      radius_extract_key                                   */
/*                      radius_extract_mppe                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "eapol_key.h"
#include "host_if.h"
#include "ieee_auth_1x.h"
#include "imem_if.h"
#include "maccontroller.h"
#include "md5.h"
#include "mh.h"
#include "prf.h"
#include "radius_client.h"
#include "radius_timers.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "iconfig.h"
#include "host_if.h"

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
#include "host_if.h"
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_radius_secret[RAD_SECRET_MAX_LEN] = {0};/* Radius Secret Key       */
UWORD8 g_radius_secret_len                 = 8;  /* Radius Secret Key Length*/
UWORD8 g_auth_eth_addr[6]                  = {0};/* Access point ETH address*/
UWORD8 g_auth_mac_addr[6]                  = {0};/* Access point MAC address*/
UWORD8 g_local_ip_addr[4]                  = {0};/* Radius client ip addr   */
UWORD8 g_server_ip_addr[4]                 = {0};/* Radius server ip addr   */
UWORD16 g_local_port                       = 0;  /* Radius client port      */
UWORD16 g_server_port                      = 0;  /* Radius server port      */
UWORD8 g_server_eth_addr[6]                = {0};/* Radius server ETH address*/
UWORD8 g_server_max_tx                     = 2;

radius_node_t  *g_rad_node[NUM_SUPP_STA]   = {0};
resp_pending_t *g_resp_pending                = NULL;

/*****************************************************************************/
/* Static Function Declaration                                               */
/*****************************************************************************/



static UWORD16 radius_make_resp(auth_t *auth_1x, radius_node_t *rad_node,
                                UWORD8* msg);
static void add_node_2_pending(radius_node_t *rad_node, UWORD8 id);



static BOOL_T radius_check_auth    (UWORD8 *msg, UWORD16 len,
                                    radius_node_t *rad_node);
static BOOL_T radius_check_msg_auth(UWORD8 *msg, UWORD16 len,
                                    radius_node_t *node);
static BOOL_T prep_radius_reply    (UWORD8 *msg, UWORD16 len,
                                    radius_node_t *rad_node);
static BOOL_T radius_extract_mppe  (UWORD8 *msg, UWORD8 len, UWORD8 *dkey,
                                    UWORD16 salt, UWORD8 *req_auth);
static BOOL_T radius_extract_key   (UWORD8 *msg, UWORD16 len,
                                    radius_node_t *rad_node);

static void send_1x_pkt_to_host    (void *start_buf_ptr, UWORD16 len,
                                    mem_handle_t *pool_hdl);

//#ifndef ENABLE_AP_1X_LINUX_PROCESSING
static void form_arp_rply_pkt(UWORD8 *arp_req, UWORD8 *msg);
//#endif /* ENABLE_AP_1X_LINUX_PROCESSING */


/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_radius_client                                       */
/*                                                                           */
/*  Description   : This function initializes the radius client with the help*/
/*                  of the given parameters                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Access point MAC address (h/w addr)    */
/*                  2) Pointer to the Access point ETH address               */
/*                  3) Pointer to the radius secret key                      */
/*                  4) The length of the secret key                          */
/*                  5) The local IP Address                                  */
/*                  6) The Radius client port number                         */
/*                  7) The Radisu server IP Address                          */
/*                  8) The Radius server port number                         */
/*                  9) The Radius server H/W Address                         */
/*                  10) Function pointer for sending Radius message to server*/
/*                  11) Function pointer for updating the radius node        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the raduis client with the     */
/*                  given parameters and starts the radius client            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_radius_client(UWORD8 *auth_mac_addr, UWORD8* auth_eth_addr,
                        UWORD8 *secret, UWORD8 secret_len, UWORD8 *local_ip,
                        UWORD16 local_port, UWORD8 *server_ip,
                        UWORD16 server_port, UWORD8 *server_eth_addr,
                        BOOL_T (** send_msg_server)(auth_t *auth_1x),
                        void (** update_identity)(auth_t *, UWORD8*, UWORD8))
{
    /* Copy the given parameters to the thread variables */
    if(auth_eth_addr != 0)
    {
        memcpy(g_auth_eth_addr,   auth_eth_addr,   6);
    }
    memcpy(g_auth_mac_addr,   auth_mac_addr,   6);
    memcpy(g_radius_secret,   secret,          secret_len);
    memcpy(g_local_ip_addr,   local_ip,        4);
    memcpy(g_server_ip_addr,  server_ip,       4);
    memcpy(g_server_eth_addr, server_eth_addr, 6);

    g_radius_secret_len = secret_len;
    g_local_port        = local_port;
    g_server_port       = server_port;

    /* Update the function pointers */
    *send_msg_server   = send_msg_2_server;
    *update_identity   = update_supp_identity;

#ifdef ENABLE_AP_1X_LINUX_PROCESSING
    update_radius_param();
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : stop_radius_client                                       */
/*                                                                           */
/*  Description   : This function stops the radius client                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function stops the radius client and frees all the  */
/*                  related nodes                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void stop_radius_client(void)
{
    UWORD8         i          = 0;
    resp_pending_t *temp_ptr1 = NULL;

    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        if(g_rad_node[i] != NULL)
        {
            free_rad_auth_node(i);
        }
    }

    /* Search for the mathcing request ID */
    temp_ptr1 = g_resp_pending;
    while(temp_ptr1 != NULL)
    {
        g_resp_pending = temp_ptr1->next_ptr;
        mem_free(g_local_mem_handle, temp_ptr1);
        temp_ptr1 = g_resp_pending;
    }

    g_resp_pending = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : free_rad_auth_node                                       */
/*                                                                           */
/*  Description   : This function stops the given radius node                */
/*                                                                           */
/*  Inputs        : 1) Association ID for the given radius node              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function stops the radius client and frees all the  */
/*                  related memory                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void free_rad_auth_node(UWORD8 id)
{
     if(g_rad_node[id] == NULL)
     {
          return;
     }

     /* Stop the radius timer */
     stop_radius_timer(g_rad_node[id]);

     /* Free the radius message */
     if(g_rad_node[id]->rad_msg != NULL)
     {
          pkt_mem_free(g_rad_node[id]->rad_msg);
          g_rad_node[id]->rad_msg = NULL;
     }

     /* Free the pending node */
     get_pending_node(g_rad_node[id]->req_id);

     g_rad_node[id]->auth_1x = NULL;

     /* Finally free the radius node itself */
     mem_set(g_rad_node[id], 0, sizeof(radius_node_t));
     mem_free(g_local_mem_handle, g_rad_node[id]);
     g_rad_node[id] = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_supp_identity                                     */
/*                                                                           */
/*  Description   : This function updates the radius client node with the    */
/*                  the given parameters                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the 802.1x Handle                          */
/*                  2) Pointer to the user name                              */
/*                  3) Length of the user name                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the radius node with the given     */
/*                  given parameters. If radius node is not available a new  */
/*                  node is obtained and intialized with the parameters      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_supp_identity(auth_t *auth_1x, UWORD8 *usr_name, UWORD8 len)
{
    radius_node_t * rad_node = g_rad_node[auth_1x->aid];

    /* If node is not preset, allocate one */
    if(rad_node == NULL)
    {
        rad_node = mem_alloc(g_local_mem_handle, sizeof(radius_node_t));
        if(rad_node == NULL)
        {
            return;
        }
        g_rad_node[auth_1x->aid] = rad_node;
        /* Reset all the parameters of the node */
        mem_set(rad_node, 0, sizeof(radius_node_t));
    }
    else
    {
        /* Stop the radius timer */
        stop_radius_timer(rad_node);

        if(rad_node->rad_msg != NULL)
        {
            pkt_mem_free(rad_node->rad_msg);
            rad_node->rad_msg = NULL;

        }

        get_pending_node(rad_node->req_id);

        /* Reset all the parameters of the node */
        mem_set(rad_node, 0, sizeof(radius_node_t));
    }

    /* Copy the user name and the length */
    memcpy(rad_node->supp_usr_name, usr_name, len);
    rad_node->supp_usr_name_len = len;
    rad_node->auth_1x           = auth_1x;
    rad_node->resp_expected     = BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_msg_2_server                                        */
/*                                                                           */
/*  Description   : This function sends the EAP message to the Authenticating*/
/*                  server                                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the 802.1x Handle                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sends the EAP message to the Authenticating*/
/*                  server. The function first creates the message and then  */
/*                  transmits the packet to the radius server                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_msg_2_server(auth_t *auth_1x)
{

    UWORD16 len     = 0;
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
    UWORD8 eth_hdr_offset =
        get_host_msg_hdr_len(get_host_data_if_type());
#else
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
    UWORD8 eth_hdr_offset =
        get_host_msg_hdr_len(get_host_data_if_type());
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
#endif

    BOOL_T  tx_flag = BTRUE;
    radius_node_t * rad_node = g_rad_node[auth_1x->aid];

    /* If node is not preset, allocate one */
    if(rad_node == NULL)
    {
        return BFALSE;
    }
    else
    {
        /* If packet has been successfully transmitted, create a backup of */
        /* the transmitted packet for retransmissions                      */
        /* A new buffer is required because buffer is freed after the      */
        /* end of transmission                                             */
        if(rad_node->tx_count <= g_server_max_tx)
        {
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
            /* Get new buffer for transmission */
            UWORD8 *msg =
            (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, RAD_SERVER_MSG_MAX_LEN);

            if(msg == NULL)
            {
                return BFALSE;
            }

            /* If the transmission is a fresh transmission, prepare packet */
            /* from the scratch, else use the msg cache and retransmit     */
            if(rad_node->tx_count == 0)
            {

                len = radius_make_resp(auth_1x, rad_node,
                                    msg + UDP_DATA_OFFSET +
                                    eth_hdr_offset);


                /* Prepare the ETH/IP/UDP Header */
                prepare_1x_config_pkt(msg + eth_hdr_offset, NULL, len,
                        g_server_eth_addr, g_auth_eth_addr, g_local_ip_addr,
                        g_server_ip_addr, g_local_port, g_server_port,
                        BFALSE, 0);

                len += UDP_DATA_OFFSET; /* Add the Eth/IP/UDP header length */
                rad_node->rad_msg_len = len;

                /* Copy the message to the radius node, to be used for */
                /* retransmissions                                     */
                rad_node->rad_msg =
                (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, RAD_SERVER_MSG_MAX_LEN);

                if(rad_node->rad_msg == NULL)
                {
                    pkt_mem_free(msg);
                    return BFALSE;
                }
                memcpy(rad_node->rad_msg, msg, len + eth_hdr_offset);
            }
            /* The copy is required only for the upto penultimate  */
            /* retransmissions                                     */
            else
            {
                if(rad_node->rad_msg == NULL)
                {
                    rad_node->tx_count = 0;
                    pkt_mem_free(msg);
                    return BFALSE;
                }
                len = rad_node->rad_msg_len;
                memcpy(msg, rad_node->rad_msg, len + eth_hdr_offset);
            }

            /* Transmit the packet and start the timer */
            if(len <= MAX_UDP_IP_PKT_LEN)
            {
                send_1x_pkt_to_host(msg, len, g_shared_pkt_mem_handle);
            }
            else
            {
                /* Get new buffer for transmission */
                UWORD8 *msg2 = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

                if(msg2 == NULL)
                {
                    pkt_mem_free(msg);
                    return BFALSE;
                }

                prepare_1x_config_pkt(msg + eth_hdr_offset,
                                   NULL,
                                   MAC_UDP_DATA_LEN, g_server_eth_addr,
                                   g_auth_eth_addr, g_local_ip_addr,
                                   g_server_ip_addr, g_local_port,
                                   g_server_port, BTRUE, 0);

                memcpy(msg2 + eth_hdr_offset + UDP_HDR_OFFSET,
                        msg + eth_hdr_offset + MAX_UDP_IP_PKT_LEN,
                       (len - MAX_UDP_IP_PKT_LEN));

                send_1x_pkt_to_host(msg, MAX_UDP_IP_PKT_LEN,
                                    g_shared_pkt_mem_handle);

                prepare_1x_config_pkt(msg2 + eth_hdr_offset, NULL,
                                   len - MAX_UDP_IP_PKT_LEN,
                                   g_server_eth_addr, g_auth_eth_addr,
                                   g_local_ip_addr, g_server_ip_addr,
                                   g_local_port, g_server_port, BFALSE, 0xb9);

                send_1x_pkt_to_host(msg2, len - MAX_UDP_IP_PKT_LEN
                                    + UDP_HDR_OFFSET, g_shared_pkt_mem_handle);

            }

            rad_node->tx_count++;
            start_radius_timer(rad_node,
                                rad_node->auth_1x->auth_bak.serverTimeout);


#else /* ENABLE_AP_1X_LINUX_PROCESSING */

            /* If the transmission is a fresh transmission, prepare packet */
            /* from the scratch, else use the msg cache and retransmit     */
            if(rad_node->tx_count == 0)
            {
                /* Get new buffer for transmission */
                UWORD8 *msg =
                (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, RAD_SERVER_MSG_MAX_LEN);

                if(msg == NULL)
                {
                    return BFALSE;
                }

                /* Linux Stack is used for Radius Client. UDP Header is not */
                /* formed. Message is directly formed and sent to the       */
                /* server                                                   */
                len = radius_make_resp(auth_1x, rad_node, msg);
                rad_node->rad_msg     = msg;
                rad_node->rad_msg_len = len;
            }
            else
            {
                if(rad_node->rad_msg == NULL)
                {
                    rad_node->tx_count = 0;
                    return BFALSE;
                }
            }

            /* Send the message                               */
            tx_flag = send_frame_to_rad_server(rad_node->rad_msg,
                                               rad_node->rad_msg_len);

            rad_node->tx_count++;
            start_radius_timer(rad_node,
                               rad_node->auth_1x->auth_bak.serverTimeout);

#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

            /* If packet has been successfully transmitted, add the radius   */
            /* node to the list of nodes for which the response is expected  */
            /* from the Authenticating server                                */
            if(tx_flag == BTRUE)
            {
                add_node_2_pending(rad_node, (UWORD8)(auth_1x->aid & 0xFF));
                rad_node->resp_expected = BTRUE;
            }
            else
            {
                pkt_mem_free(rad_node->rad_msg);
                rad_node->rad_msg = NULL;
            }
        }
        else
        {
            rad_node->tx_count = 0;
            if(rad_node->rad_msg != NULL)
            {
                pkt_mem_free(rad_node->rad_msg);
                rad_node->rad_msg = NULL;
            }
            auth_1x->global.aWhileTimeout = BTRUE;
            auth_onex_fsm_run(auth_1x);
        }
    }

    return BTRUE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : radius_make_resp                                         */
/*                                                                           */
/*  Description   : This function creates the radius message to be sent to   */
/*                  Authenticating Server using the 802.1x handle            */
/*                                                                           */
/*  Inputs        : 1) Pointer the 802.1x handle                             */
/*                  2) Pointer to the radius node handle                     */
/*                  3) Pointer to the message buffer                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function forms the radius header and forms the body */
/*                  of the radius message to be sent to the AS using the     */
/*                  pointer to the 802.1x handle                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : The length of the packet                                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 radius_make_resp(auth_t *auth_1x, radius_node_t *rad_node, UWORD8* msg)
{

    UWORD16 index                      = 0;
    UWORD8  i                          = 0;
    UWORD8  digest[HMAC_DIGEST_LENGTH] = {0};

    msg[index++] = RAD_ACCESS_REQUEST;
    msg[index++] = (UWORD8)(auth_1x->aid & 0xFF);
    rad_node->req_id = (UWORD8)(auth_1x->aid & 0xFF);

    /* Omit length field (2 Bytes). It is update after the packet is formed  */
    index += 2;

    /* Save authenticator hash (16 bytes) for use in validating replies */
    /* for extracting MPPE keys returned by the server                  */
    for(i = 0; i < RAND_HASH_LEN; i++)
    {
        rad_node->auth_hash[i] = get_random_byte();
        msg[index++] = rad_node->auth_hash[i];
    }

    /* Add the attribute-value pairs. */

    /* Add Service Type */
    index += radius_add_int(&msg[index], RAD_ATTR_SERVICE_TYPE,
                  (UWORD32)RAD_SERVICE_TYPE_FRAMED);

    /* Add User Name */
    index += radius_add_field(&msg[index], RAD_ATTR_USER_NAME,
                  rad_node->supp_usr_name, rad_node->supp_usr_name_len);

    /* NB: use Ethernet MTU rather than 802.11 MTU */
    index += radius_add_int(&msg[index], RAD_ATTR_FRAMED_MTU,
                  (UWORD32)RADIUS_EAP_MAX_LEN);


    /* If a response has been received with state, forward the same */
    if(rad_node->supp_state_len != 0)
    {
        index += radius_add_field(&msg[index], RAD_ATTR_STATE,
                      rad_node->supp_state, rad_node->supp_state_len);

    }


    /* If a response has been received with class, forward the same */
    if(rad_node->supp_class_len != 0)
    {
        index += radius_add_field(&msg[index], RAD_ATTR_CLASS,
                      rad_node->supp_class, rad_node->supp_class_len);

    }

    /* Add the called station address */
    index += radius_add_hwaddr(&msg[index], RAD_ATTR_CALLED_STATION_ID,
                               g_auth_mac_addr);

    /* Add the calling station address */
    index += radius_add_hwaddr(&msg[index], RAD_ATTR_CALLING_STATION_ID,
                               auth_1x->addr);

    /* Add the NAS Identifier */
    index += radius_add_hwaddr(&msg[index], RAD_ATTR_NAS_IDENTIFIER,
                               g_auth_mac_addr);

    /* Add the NAS port type */
    index += radius_add_int(&msg[index], RAD_ATTR_NAS_PORT_TYPE,
                            (UWORD32)RAD_NAS_PORT_TYPE_WIRELESS);

    /* Add connection info */
    {
        UWORD8 str[30] = {0};
        switch (rad_node->supp_curmode) {
        default:
        {
            strcpy((WORD8 *)str, "CONNECT 54Mbps 802.11g");
        }
        break;
        }
        /* index += radius_add_field(&msg[index], RAD_ATTR_CONNECT_INFO,
                      str, strlen(str)); */
    }

    /* Add NAS IP Address */
    index += radius_add_field(&msg[index], RAD_ATTR_NAS_IP_ADDRESS,
                  g_local_ip_addr, 4);

    /* Add NAS Port */
    index += radius_add_int(&msg[index], RAD_ATTR_NAS_PORT,
                  (UWORD32)auth_1x->aid);

    /* Add NAS Port ID */
    {
        UWORD8 str[15] = {0};
        strcpy((WORD8 *) str, "STA Port # ");
        str[11] = hex_2_chr((UWORD8)((auth_1x->aid & 0xF0) >> 4));
        str[12] = hex_2_chr((UWORD8)(auth_1x->aid & 0x0F));
        index  += radius_add_field(&msg[index], RAD_ATTR_NAS_PORT_ID,
                      str, 13);
    }

    /* Copy in the original EAP message. */
    {
        UWORD16 eap_len = auth_1x->eapolEap_frame_len;
        UWORD8 *eap_ptr = auth_1x->eapolEap;
        do
        {
            UWORD16 temp_len = (eap_len > 0xFD)?0xFD:eap_len;

            /* Update the message with EAP message */
            index += radius_add_field(&msg[index], RAD_ATTR_EAP_MESSAGE,
                                      eap_ptr, (UWORD8)temp_len);

            /* Update segment information */
            eap_len -= temp_len;
            eap_ptr += temp_len;
        }
        while(eap_len != 0);
    }


    /* Copy the hash value to the message */
    index += radius_add_field(&msg[index], RAD_ATTR_MESSAGE_AUTHENTICATOR,
                   digest, RAND_HASH_LEN);

    /* Patch up so that required the message be formulated and calculate */
    /* the message authenticator hash.                                   */

    /* Update the length field */
    msg[2] = (index & 0xFF00) >> 8;
    msg[3] =  index & 0x00FF;

    hmac_md5(msg, index, g_radius_secret, g_radius_secret_len, digest);
    memcpy(msg + index - RAND_HASH_LEN, digest, RAND_HASH_LEN);

    return index;

}


//#ifndef ENABLE_AP_1X_LINUX_PROCESSING


/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_arp_pkt                                               */
/*                                                                           */
/*  Description   : This function determines if the incoming packet is a     */
/*                  ARP request packet.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming host frame buffer             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function determines if the incoming packet is a     */
/*                  ARP request packet.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if the packet is a ARP Request message            */
/*                  BFALSE, otherwise                                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_arp_pkt(UWORD8 *buffer)
{

    UWORD8  *eth_hdr      = 0;
    UWORD8  eth_hdr_offset = get_eth_hdr_offset(get_host_data_if_type());
    UWORD8  *arp_dst_addr = 0;
    UWORD16 eth_type      = 0;

    /* Initialize the pointer to the start of the ethernet header in the     */
    /* buffer received from ethernet.                                        */
    eth_hdr = buffer + eth_hdr_offset;

    /* Extract the type of the ethernet packet. Extract the ethernet type    */
    /* and the IP protocol. If the packet is a UDP packet, check if the      */
    /* source and destination ports match the values as used by the          */
    /* radius packets.                                                       */
    eth_type     = ((eth_hdr[ETH_PKT_TYPE_OFFSET] << 8) |
                    eth_hdr[ETH_PKT_TYPE_OFFSET + 1]);

    arp_dst_addr = (UWORD8*)(&eth_hdr[38]);

    if(eth_type == ARP_TYPE)
    {
        if(memcmp(arp_dst_addr, g_local_ip_addr, 4) == 0)
        {
            return BTRUE;
        }
    }

    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_arp_reply                                           */
/*                                                                           */
/*  Description   : This function sends the ARP Response                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                                                                           */
/*  Globals       : g_auth_eth_addr                                          */
/*                  g_local_ip_addr                                          */
/*                                                                           */
/*  Processing    : This function sends the ARP Response                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_arp_reply(UWORD8 *buffer, UWORD32 len)
{
    UWORD8 *arp_req = buffer + get_eth_hdr_offset(get_host_data_if_type());
    UWORD8 *msg     = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);
    UWORD8 eth_hdr_offset = get_host_msg_hdr_len(get_host_data_if_type());

    if(msg == NULL)
    {
	printk("send arp_reply allock mem failed!\n");
        /* Free the received packet */
        pkt_mem_free(buffer);
        return;
    }

    /* If ECOS is used, ARP Reply is sent so that EAP protocol works fine */
    /* Get new buffer for transmission, form the packet and send it       */
    form_arp_rply_pkt(arp_req, msg + eth_hdr_offset);

    send_1x_pkt_to_host(msg, ARP_PKT_LEN + eth_hdr_offset,
        g_shared_pkt_mem_handle);

    /* Free the received packet */
    pkt_mem_free(buffer);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : form_arp_rply_pkt                                        */
/*                                                                           */
/*  Description   : This function creates the ARP response packet            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the ARP Request                            */
/*                  2) Pointer to the message buffer                         */
/*                                                                           */
/*  Globals       : g_auth_eth_addr                                          */
/*                  g_local_ip_addr                                          */
/*                                                                           */
/*  Processing    : This function creates the ARP response packet            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void form_arp_rply_pkt(UWORD8* arp_req, UWORD8 *msg)
{

    UWORD8 index = 0;
    /* Form the ethernet header */

    memcpy(msg, arp_req + 6, 6);
    index += 6;
    memcpy(msg + index, g_auth_eth_addr, 6);
    index += 6;
    /* Add the packet type as ARP */
    msg[index++] = 0x08;
    msg[index++] = 0x06;
    /* Add the Hardware type as ETH */
    msg[index++] = 0x00;
    msg[index++] = 0x01;
    /* Add the protocol type as IP */
    msg[index++] = 0x08;
    msg[index++] = 0x00;
    /* Add the hardware size as 6 */
    msg[index++] = 0x06;
    /* Add the protocol size as 4 IPv4 */
    msg[index++] = 0x04;
    /* Add the response opcode */
    msg[index++] = 0x00;
    msg[index++] = 0x02;
    /* Add the sender MAC Address */
    memcpy(msg + index, g_auth_eth_addr, 6);
    index += 6;
    /* Add the sender IP Address */
    memcpy(msg + index, g_local_ip_addr, 4);
    index += 4;
    /* Add the destination MAC Address */
    memcpy(msg + index, arp_req + 6, 6);
    index += 6;
    /* Add the target IP Address */
    memcpy(msg + index, arp_req + 28, 4);
    index += 4;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_radius_pkt                                            */
/*                                                                           */
/*  Description   : This function determines if the incoming packet is a     */
/*                  radius packet.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming host frame buffer             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : For the ethernet host, a UDP packet with source port and */
/*                  destination port as radius port is by protocol a         */
/*                  radius message.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if the packet is a radius message                 */
/*                  BFALSE, otherwise                                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_radius_pkt(UWORD8 *buffer)
{

    UWORD8  *eth_hdr     = 0;
    UWORD8  eth_hdr_offset = get_eth_hdr_offset(get_host_data_if_type());
    UWORD8  ip_protocol  = 0;
    UWORD16 eth_type     = 0;
    UWORD16 udp_src_port = 0;
    UWORD16 udp_dst_port = 0;

    /* Initialize the pointer to the start of the ethernet header in the     */
    /* buffer received from ethernet.                                        */
    eth_hdr = buffer + eth_hdr_offset;

    /* Extract the type of the ethernet packet. Extract the ethernet type    */
    /* and the IP protocol. If the packet is a UDP packet, check if the      */
    /* source and destination ports match the values as used by the          */
    /* radius packets.                                                       */
    eth_type    = ((eth_hdr[ETH_PKT_TYPE_OFFSET] << 8) |
                    eth_hdr[ETH_PKT_TYPE_OFFSET + 1]);

    ip_protocol = eth_hdr[IP_PROT_OFFSET];

    udp_src_port = ((eth_hdr[UDP_SRC_PORT_OFFSET] << 8) |
                     eth_hdr[UDP_SRC_PORT_OFFSET + 1]);

    udp_dst_port = ((eth_hdr[UDP_DST_PORT_OFFSET] << 8) |
                     eth_hdr[UDP_DST_PORT_OFFSET + 1]);

    if((eth_type == IP_TYPE) && (ip_protocol == UDP_TYPE))
    {
        if((udp_src_port == g_server_port) && (udp_dst_port == g_local_port))
        {
            return BTRUE;
        }
    }

    return BFALSE;
}

//#endif /* ENABLE_AP_1X_LINUX_PROCESSING */


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_rx_radius                                   */
/*                                                                           */
/*  Description   : This function processes the radius response from Server  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes the radius response from Server  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void process_host_rx_radius(UWORD8 *buffer, UWORD32 len)
{
    UWORD8        code      = 0;
    radius_node_t *rad_node = NULL;
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
    UWORD8        *rad_msg  = buffer +
                               get_eth_hdr_offset(get_host_data_if_type()) +
                               UDP_DATA_OFFSET;

    /* Update the len to the radius length */
    len -= UDP_DATA_OFFSET;
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
    UWORD8        *rad_msg  = buffer;
    if(len == 0)
    {
        return;
    }
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

    /* Get the radius node for the received packet */
    rad_node = get_pending_node(rad_msg[1]);

    if(rad_node != NULL)
    {
        /* Chech the authentication hash in the message */
        if(radius_check_auth(rad_msg, (UWORD16)len, rad_node) == BTRUE)
        {
            code = rad_msg[0];
            if(code != RAD_ACCESS_REJECT)
            {
                UWORD16 index = 0;
                if(radius_check_msg_auth(rad_msg, (UWORD16)len, rad_node) != BTRUE)
                {
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
                    /* Free the received packet */
                    pkt_mem_free(buffer);
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
                    return;
                }

                /* Copy the state */
                index = radius_find_attr(rad_msg + RADIUS_ATTR_OFFSET,
                                        (UWORD16)(len - RADIUS_ATTR_OFFSET),
                                        RAD_ATTR_STATE);
                if(index != 0xFFFF)
                {
                    index += RADIUS_ATTR_OFFSET;
                    rad_node->supp_state_len = rad_msg[index + 1] - 2;
                    if(rad_node->supp_state_len > RAD_STATE_MAX_LEN)
                    {
                        rad_node->supp_state_len = RAD_STATE_MAX_LEN;
                    }
                    memcpy(rad_node->supp_state, &rad_msg[index + 2],
                            rad_node->supp_state_len);
                }
                else
                {
                    rad_node->supp_state_len = 0;
                }

                /* Copy the CLASS */
                index = radius_find_attr(rad_msg + RADIUS_ATTR_OFFSET,
                                        (UWORD16)(len - RADIUS_ATTR_OFFSET),
                                        RAD_ATTR_CLASS);
                if(index != 0xFFFF)
                {
                    index += RADIUS_ATTR_OFFSET;
                    rad_node->supp_class_len = rad_msg[index + 1] - 2;
                    if(rad_node->supp_class_len > RAD_CLASS_MAX_LEN)
                    {
                        rad_node->supp_class_len = RAD_CLASS_MAX_LEN;
                    }
                    memcpy(rad_node->supp_class, &rad_msg[index + 2],
                            rad_node->supp_class_len);
                }
                else
                {
                    rad_node->supp_class_len = 0;
                }

                if(prep_radius_reply(rad_msg, (UWORD16)len, rad_node) != BTRUE)
                {
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
                    /* Free the received packet */
                    pkt_mem_free(buffer);
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
                    return;
                }

                /* Reset the retransmission count for the node */
                rad_node->tx_count = 0;
            }
            /* Stop the timer */
            stop_radius_timer(rad_node);
            /* Free the buffers stored for the retransmissions */
            pkt_mem_free(rad_node->rad_msg);
            rad_node->rad_msg = NULL;

            switch(code)
            {
            case RAD_ACCESS_ACCEPT:
                radius_extract_key(rad_msg, (UWORD16)len, rad_node);
                rad_node->auth_1x->auth_bak.aFail    = BFALSE;
                rad_node->auth_1x->auth_bak.aSuccess = BTRUE;
                auth_onex_fsm_run(rad_node->auth_1x);
                break;
            case RAD_ACCESS_REJECT:
                rad_node->auth_1x->auth_bak.aFail    = BTRUE;
                rad_node->auth_1x->auth_bak.aSuccess = BFALSE;
                auth_onex_fsm_run(rad_node->auth_1x);
                break;
            case RAD_ACCESS_CHALLENGE:
                rad_node->auth_1x->auth_bak.aReq = BTRUE;
                auth_onex_fsm_run(rad_node->auth_1x);
                break;
            default:
                break;
            }
        }
    }

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
    /* Free the received packet */
    pkt_mem_free(buffer);
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_node_2_pending                                       */
/*                                                                           */
/*  Description   : This function adds the node to the list of all the nodes */
/*                  for which the response is expected from the server       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the node to the list of all the nodes */
/*                  for which the response is expected from the server       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void add_node_2_pending(radius_node_t *rad_node, UWORD8 id)
{
    resp_pending_t *new_req  = NULL;
    resp_pending_t *temp_ptr = NULL;

    new_req = mem_alloc(g_local_mem_handle, sizeof(resp_pending_t));

    if(new_req != BFALSE)
    {
        new_req->req_id   = id;
        new_req->rad_node = rad_node;
        new_req->next_ptr = NULL;

        /* Search for the tail */
        temp_ptr = g_resp_pending;
        while((temp_ptr != NULL) && (temp_ptr->next_ptr != NULL))
        {
            temp_ptr = temp_ptr->next_ptr;
        }

        /* If head id NULL, initialize the head to the new member     */
        /* else the next of the tail is initialized to the new member */
        if(temp_ptr == NULL)
        {
            g_resp_pending = new_req;
        }
        else
        {
            temp_ptr->next_ptr = new_req;
        }
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_pending_node                                         */
/*                                                                           */
/*  Description   : This function fetches the node from the list of all nodes*/
/*                  for which the response is expected from the server       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function fetches the node from list of all nodes    */
/*                  for which the response is expected from the server       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : The node for which the search has matched                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

radius_node_t *get_pending_node(UWORD8 id)
{
    resp_pending_t *temp_ptr1 = NULL;
    resp_pending_t *temp_ptr2 = NULL;
    radius_node_t  *ret_val   = NULL;


    /* Search for the mathcing request ID */
    temp_ptr1 = g_resp_pending;
    temp_ptr2 = g_resp_pending;
    while(temp_ptr1 != NULL)
    {
        if((temp_ptr1->req_id == id)
            && (temp_ptr1->rad_node->resp_expected == BTRUE))
        {
            break;
        }
        temp_ptr2 = temp_ptr1;
        temp_ptr1 = temp_ptr1->next_ptr;
    }

    /* If search is not NULL, change the link list and free the node */
    if(temp_ptr1 != NULL)
    {
        /* Non NULL value implies a successful search */
        if(temp_ptr1 == g_resp_pending)
        {
            g_resp_pending = g_resp_pending->next_ptr;
        }
        else
        {
            temp_ptr2->next_ptr = temp_ptr1->next_ptr;
        }

        /* Set the return value and free the node */
        ret_val = temp_ptr1->rad_node;
        mem_free(g_local_mem_handle, temp_ptr1);
    }
    else
    {
        ret_val = NULL;
    }

    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : radius_check_auth                                        */
/*                                                                           */
/*  Description   : This function verifies the response authentication in the*/
/*                  received radius message                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                  3) Pointer to the radius node handle                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function verifies the response authentication in the*/
/*                  received radius message. The hash sent in the Access     */
/*                  request message is used to verify the authentication code*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if authentication test passes; else BFALSE         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T radius_check_auth(UWORD8 *msg, UWORD16 len, radius_node_t *rad_node)
{
    md5_state_t context = {{0},};
    UWORD8 digest[16] = {0};

    memcpy(digest, msg + 4, RAND_HASH_LEN);
    memcpy(msg + 4, rad_node->auth_hash, RAND_HASH_LEN);

    /* Inner MD5 */
    md5_init(&context);
    md5_append(&context, msg, len);
    md5_append(&context, g_radius_secret, g_radius_secret_len);
    md5_finish(&context, msg + 4);

    if(memcmp(msg + 4, digest, RAND_HASH_LEN) == 0)
    {
        return BTRUE;
    }
    return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : radius_check_msg_auth                                    */
/*                                                                           */
/*  Description   : This function verifies the message authentication in the */
/*                  received radius message                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                  3) Pointer to the radius node handle                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function verifies the message authentication in the */
/*                  received radius message                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if authentication test passes; else BFALSE         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T radius_check_msg_auth(UWORD8 *msg, UWORD16 len, radius_node_t *rad_node)
{
    UWORD16 index = 0;

    /* Find the attribute 'RAD_ATTR_MESSAGE_AUTHENTICATOR' */
    index = radius_find_attr(msg + RADIUS_ATTR_OFFSET,
                            (UWORD16)(len - RADIUS_ATTR_OFFSET),
                            RAD_ATTR_MESSAGE_AUTHENTICATOR);
    /* Compare the Message authenticator if found in the received message */
    if(index != 0xFFFF)
    {
        UWORD8  digest[RAND_HASH_LEN] = {0};
        UWORD8  auth[RAND_HASH_LEN]   = {0};

        index += RADIUS_ATTR_OFFSET;

        memcpy(auth, msg + 4, RAND_HASH_LEN);
        memcpy(msg + 4, rad_node->auth_hash, RAND_HASH_LEN);

        memcpy(digest, msg + index + 2, RAND_HASH_LEN);
        mem_set(msg + index + 2, 0, RAND_HASH_LEN);

        /* Verify the authenticity of the received packet */
        hmac_md5(msg, len, g_radius_secret,
                g_radius_secret_len, msg + index + 2);

        memcpy(msg + 4, auth, RAND_HASH_LEN);

        if(memcmp(msg + index + 2, digest, RAND_HASH_LEN) == 0)
        {
            return BTRUE;
        }
    }
    return BFALSE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : prep_radius_reply                                        */
/*                                                                           */
/*  Description   : This function prepares the message to be sent to the     */
/*                  supplicant from the received radius message              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                  3) Pointer to the radius node handle                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the message to be sent to the     */
/*                  supplicant from the received radius message              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if parsing radius message passes; else BFALSE      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T prep_radius_reply(UWORD8 *msg, UWORD16 len, radius_node_t *rad_node)
{
    UWORD8 *buffer  = 0;
    UWORD8 *eap_ptr = 0;
    UWORD16 index   = 0;
    UWORD16 p_index = RADIUS_ATTR_OFFSET;
    UWORD16 eap_len = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    if(buffer == NULL)
    {
        return BFALSE;
    }

    /* reset the buffer */
    mem_set(buffer, 0, RADIUS_EAP_MAX_LEN);

    /* Set the buffer pointer offseted with the mac and 1x header */
    eap_ptr = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

    /* Copy the EAP Message */
    do
    {
        index = radius_find_attr(msg + p_index,
                                (UWORD16)(len - p_index),
                                RAD_ATTR_EAP_MESSAGE);
        if(index != 0xFFFF)
        {
            /* Length of the EAP Segment */
            UWORD8 temp = 0;

            index += p_index;
            temp   = msg[index + 1] - 2;

            /* Copy the segment, and increment the length and data pointers */
            memcpy(eap_ptr, &msg[index + 2], temp);
            eap_len += temp;
            eap_ptr += temp;
            index   += temp + 2;
            p_index  = index;
        }
    }
    while(index != 0xFFFF);

    /* If atleast one EAP segment is present */
    /* update the eap pointer in 1x handle   */
    if(eap_len != 0)
    {
        /* Set the buffer pointer offseted with the mac and 1x header */
        eap_ptr = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

        /* Copy the frame, frame length and the id From server */
        if(rad_node->auth_1x->eap != NULL)
        {
            pkt_mem_free(rad_node->auth_1x->eap);
        }
        rad_node->auth_1x->eap                   = buffer;
        rad_node->auth_1x->eap_frame_len         = eap_len;
        rad_node->auth_1x->auth_bak.idFromServer = eap_ptr[1];
    }
    else
    {
        /* Copy the frame, frame length and the id From server */
        if(rad_node->auth_1x->eap != NULL)
        {
            pkt_mem_free(rad_node->auth_1x->eap);
        }
        rad_node->auth_1x->eap = NULL;

        pkt_mem_free(buffer);

        return BFALSE;
    }
    return BTRUE;
}




/*****************************************************************************/
/*                                                                           */
/*  Function Name : radius_extract_key                                       */
/*                                                                           */
/*  Description   : This function extracts the keys from the Accept message  */
/*                  received from the authenticating server                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the message buffer                         */
/*                  2) The length of the received packet                     */
/*                  3) Pointer to the radius node handle                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the keys from the Accept message  */
/*                  received from the authenticating server. The Tx and Rx   */
/*                  keys present in the radius messages are obtained via MD5 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if parsing radius message passes; else BFALSE      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T radius_extract_key(UWORD8 *msg, UWORD16 len, radius_node_t *rad_node)
{
    UWORD8  found    = 0;
    UWORD16 index    = 0;
    UWORD16 p_index  = RADIUS_ATTR_OFFSET;
    UWORD16 key_salt = 0;

    /* Search the keys of interest and decrypt them */
    do
    {
        index = radius_find_attr(msg + p_index,
                                (UWORD16)(len - p_index),
                                RAD_ATTR_VENDOR_SPECIFIC);
        if(index != 0xFFFF)
        {
            /* Length of the EAP Segment */
            UWORD8  key_type  = 0;
            UWORD8  key_len   = 0;
            UWORD32 vendor_id = 0;
            UWORD8 *key_ptr   = 0;

            index  += p_index;
            key_ptr = msg + index + 2;

            vendor_id = (key_ptr[0] << 24) | (key_ptr[1] << 16)
                      | (key_ptr[2] << 8 ) | (key_ptr[3]);

            key_type  = key_ptr[4];
            key_len   = key_ptr[5] - 4; /* Size of payload in size of salt */
                                        /* key type and key length         */
            key_salt  = (key_ptr[6] << 8) | key_ptr[7];

            if(vendor_id != RAD_VENDOR_MICROSOFT)
            {
                return BFALSE;
            }
            if((key_salt & 0x8000) == 0x0)
            {
                /* IntegrateTBD : commented for Tinypeap */
                //return BFALSE;
            }

            switch(key_type)
            {
                case RAD_KEYTYPE_SEND:
                {
                    found += RAD_KEYTYPE_SEND;
                    /* Since only Rx Key is used in deriving PMK */
                    /* ignore this field                         */
                }
                break;
                case RAD_KEYTYPE_RECV:
                {
                    found += RAD_KEYTYPE_RECV;
                    radius_extract_mppe(&key_ptr[8], key_len,
                                    rad_node->auth_1x->rx_key, key_salt,
                                    rad_node->auth_hash);
                }
                break;
            }

            if(found == (RAD_KEYTYPE_RECV + RAD_KEYTYPE_SEND))
            {
                break;
            }

            index   += msg[index + 1];
            p_index  = index;
        }
        else
        {
            return BFALSE;
        }
    }while(index != 0xFFFF);

    return (BOOL_T)(found == (RAD_KEYTYPE_RECV + RAD_KEYTYPE_SEND));
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : radius_extract_mppe                                      */
/*                                                                           */
/*  Description   : This function extracts the keys received in Accept msg   */
/*                  in MPPE format                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the key buffer                             */
/*                  2) The length of the received key                        */
/*                  3) Pointer to the decrypted key buffer                   */
/*                  4) The salt of the key received in accept message        */
/*                  5) Pointer to the Authenticating hash                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the keys received in Accept msg   */
/*                  in MPPE format. Necessary decryption is performed to     */
/*                  decrypt the received key                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if parsing radius message passes; else BFALSE      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T radius_extract_mppe(UWORD8 *msg, UWORD8 len, UWORD8 *dkey, UWORD16 salt,
                    UWORD8 *req_auth)
{
    UWORD16 curr_idx = 0;
    UWORD16 total    = 0;
    UWORD16 i        = 0;
    UWORD16 j        = 0;
    UWORD8  *key     = NULL;

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* If the key length is not a multiple of the md5 hash size(16) */
    /* then it is zero-padded (the rfc talks about an optional      */
    /* padding sub-field but we assume clients may not include      */
    /* it--this may be wrong if they include it and do not use      */
    /* zero for padding                                             */
    /* XXX is keylen == 0 ok? */
    total = ((total = (len % 16)) == 0)?(len):(((len/16) + 1) * 16);

    /* Allocate scratch memory for the output key */
#ifdef ENABLE_SCRATCH_MEM_ESTIMATION
    key = (UWORD8 *)scratch_mem_alloc(256);
#else /* ENABLE_SCRATCH_MEM_ESTIMATION */
    key = (UWORD8 *)scratch_mem_alloc(total);
#endif /* ENABLE_SCRATCH_MEM_ESTIMATION */

    if(key == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    /* Create zero-padded, copy                       */
    /* zero-pad key if length is not a multiple of 16 */
    memcpy(key, msg, len);
    if (len != total)
    {
        mem_set(key + len, 0, (total - len));
    }


    /* The first 16 bytes are xor'd with a hash constructed */
    /* from (shared secret | req-authenticator | salt).     */
    /* Subsequent 16-byte chunks of the key are xor'd with  */
    /* the hash of (shared secret | key).                   */
    for (i = 0; i < total; i += 16)
    {
        md5_state_t context    = {{0},};
        UWORD8      digest[16] = {0};

        md5_init(&context);
        md5_append(&context, g_radius_secret, g_radius_secret_len);
        if(i == 0)
        {
            UWORD8 temp[2]  = {0};

            temp[0] = (salt & 0xFF00) >> 8;
            temp[1] = (salt & 0x00FF);

            md5_append(&context, req_auth, RAND_HASH_LEN);
            md5_append(&context, temp, 2);
        }
        else
        {
            md5_append(&context, &(key[i - 16]), 16);
        }

        md5_finish(&context, digest);

        for (j = 0; j < 16; j++)
            dkey[i+j] = key[i+j] ^ digest[j];
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_1x_pkt_to_host                                      */
/*                                                                           */
/*  Description   : This function takes the frame sent by the 1x state       */
/*                  machine, puts  it into the frame descriptor format       */
/*                  supported by the host interface and sends it to the host */
/*                  interface.                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the data frame.                            */
/*                  2) Length of the data frame.                             */
/*                  3) Pointer to the Memory Pool.                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_1x_pkt_to_host(void *start_buf_ptr, UWORD16 len,
                         mem_handle_t *pool_hdl)
{
    msdu_desc_t frame_desc = {0};

    /* The contents of the frame descriptor are filled up with details of    */
    /* the 1x packet. Note that all 1x packets will have the header and the  */
    /* payload in the same buffer. Also the payload will be contained in a   */
    /* single buffer.                                                        */

    /* Set the buffer descriptor for the frame */
    frame_desc.buffer_addr     = start_buf_ptr;
    frame_desc.host_hdr_len    = ETHERNET_HDR_LEN;
    frame_desc.host_hdr_offset = 0;
    frame_desc.data_len        = len - ETHERNET_HDR_LEN;
    frame_desc.data_offset     = ETHERNET_HDR_LEN;

    set_host_msg_hdr(get_host_data_if_type(), start_buf_ptr, len);
    send_data_frame_to_host_if(&frame_desc, get_host_data_if_type());
}

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
