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
/*  File Name         : mgmt_p2p.h                                           */
/*                                                                           */
/*  Description       : This file is the header file for P2P protocol        */
/*                                                                           */
/*  List of Functions :                                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_P2P
#ifndef MGMT_P2P_H
#define MGMT_P2P_H
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "event_manager.h"
#include "frame.h"
#include "mib_11i.h"
#include "mib_p2p.h"
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#ifdef P2P_SEC_DEV_SUPPORT
#define P2P_MAX_SEC_DEV = 2
#endif /* P2P_SEC_DEV_SUPPORT */

// caisf add 1117
#define P2P_CONFIG_FILE_PATH    "/system/lib/modules/wifi_direct.conf"
#define P2P_WID_CONFIG_MEM_SIZE   (2U * 1024U) // 1660 // eq to MAX_QRSP_LEN + 64


#define P2P_WILDCARD_SSID       "DIRECT-"
#define P2P_WILDCARD_SSID_LEN   7
#define P2P_MAGIC_STR           "restore"
#define P2P_MAGIC_STR_LEN       7

#define P2P_NUM_SOCIAL_CHAN     3

/* Device limits */
#define MAX_DEV_FOR_SCAN        10
#define NUM_NOA_DSCR            2
#define MAX_NUM_CLIENT          10

#define RSP_TIMEOUT             100
#define MAX_PERSIST_NUM_CL      6
#define MAX_WPS_CRED_LEN        255

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8 dev_capability;
    UWORD8 grp_capability;
    UWORD8 dev_addr[MAC_ADDRESS_LEN];
    UWORD8 grp_bssid[MAC_ADDRESS_LEN];
    UWORD8 grp_ssid[MAX_SSID_LEN];
    SWORD8 dev_name[MAX_DEVICE_NAME_LEN + 1];
    UWORD16 config_method;
} p2p_dev_dscr_t;

/* P2P persistent credential list structure */
typedef struct
{
    UWORD8 grp_cap;
    UWORD8 num_cl;
    UWORD8 cl_list[MAC_ADDRESS_LEN][ MAX_PERSIST_NUM_CL];
    UWORD8 auth_type;
    UWORD8 sec_mode;
    UWORD8 key_len;
    UWORD8 psk[MAX_PSK_PASS_PHRASE_LEN];
    UWORD8 ssid[MAX_SSID_LEN];
} p2p_persist_list_t;

typedef struct
{
    UWORD8  cnt_typ;
    UWORD32 duration;
    UWORD32 interval;
    UWORD32 start_time;
} noa_dscr_t;

typedef enum { NO_INVITE           = 0,
               SEND_INVITE         = 1,
               WAIT_INVITE_RSP     = 2
}P2P_INVITE_STATE;



/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern const UWORD8 g_p2p_social_chan[];
extern BOOL_T g_mode_switch_in_prog;
extern p2p_dev_dscr_t g_p2p_dev_dscr_set[];
extern UWORD8 g_dev_dscr_set_index;
extern UWORD8 g_p2p_manageability;
extern UWORD8 g_GO_tie_breaker;
extern SWORD8 g_p2p_match_idx;
extern noa_dscr_t g_noa_dscr[];
extern BOOL_T g_p2p_GO_role;
extern UWORD8 g_p2p_find_to;
extern P2P_INVITE_STATE g_p2p_invit;
extern UWORD8 g_num_noa_sched;
extern p2p_persist_list_t g_persist_list;
extern p2p_persist_list_t *g_curr_persist_list;
extern UWORD8 g_p2p_dialog_token;
extern UWORD8 g_noa_index_go;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

// caisf add 1117
extern loff_t drv_read_file(char *file_path, char *str, size_t str_len, loff_t usr_pos);
extern loff_t drv_write_file(char *file_path, char *str, size_t str_len, loff_t usr_pos);

extern UWORD8* p2p_get_attr(UWORD8 attr_id, UWORD8 *data, UWORD16 ie_len,
                            UWORD16 *attr_len);
extern UWORD8* get_p2p_attributes(UWORD8* msa, UWORD16 index, UWORD16 rx_len,
                           UWORD16* ie_len);
extern void send_host_p2p_req(UWORD8 *msa, UWORD16 rx_len);
extern void init_p2p_globals(void);
extern void initiate_mod_switch(mac_struct_t *mac);
extern void handle_inv_req_to(mac_struct_t *mac);
extern void p2p_start_invit_scan_req(void);
extern void p2p_update_cred_list(void);
extern void p2p_handle_prov_disc_req(UWORD8 *msa, UWORD8 *sa, UWORD16 rx_len);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function returns the number of NOA schedule */
INLINE UWORD8 get_num_noa_sched(void)
{
    return g_num_noa_sched;
}

/* This function sets the number or NOA schedule */
INLINE void set_num_noa_sched(UWORD8 num)
{
    if(BFALSE == g_reset_mac_in_progress)
    {    
        g_num_noa_sched = num;
    }
}

INLINE UWORD8 get_noa_index(void)
{
    return g_noa_index_go;
}

/* This function sets the number or NOA schedule */
INLINE void set_noa_index(UWORD8 num)
{
    if(BFALSE == g_reset_mac_in_progress)
    {    
        g_noa_index_go = num;
    }    
}

/* This function resets the NOA descriptor */
INLINE void reset_noa_dscr(void)
{
    UWORD8 num = 0;

    g_num_noa_sched = 0;

    for(num = 0; num < NUM_NOA_DSCR; num++)
    {
        g_noa_dscr[num].cnt_typ    = 0;
        g_noa_dscr[num].duration   = 0;
        g_noa_dscr[num].interval   = 0;
        g_noa_dscr[num].start_time = 0;
    }
}

/* This function checks if the count of the schedule is zero i.e if it is a  */
/* null schedule                                                             */
INLINE BOOL_T is_noa_sched_null(UWORD8 idx)
{
    if(g_noa_dscr[idx].cnt_typ == 0)
        return BTRUE;

    return BFALSE;
}

/*****************************************************************************/
/* Configuration Functions for P2P GO and Client                             */
/*****************************************************************************/

INLINE UWORD8 *get_p2p_trgt_dev_id(void)
{
    g_cfg_val[0] = MAC_ADDRESS_LEN;
    mac_addr_cpy(g_cfg_val + 1, mget_p2p_trgt_dev_id());

    return g_cfg_val;
}

INLINE UWORD8 *get_p2p_invit_dev_id(void)
{
    g_cfg_val[0] = MAC_ADDRESS_LEN;
    mac_addr_cpy(g_cfg_val + 1, mget_p2p_invit_dev_id());

    return g_cfg_val;
}

/* This function returns the persistent credential list */
INLINE UWORD8 *get_p2p_persist_cred(void)
{
    UWORD8 *ret_val = NULL;

    g_cfg_val[0] = sizeof(p2p_persist_list_t);
    memcpy(g_cfg_val + 1, (UWORD8 *)(&g_persist_list), g_cfg_val[0]);
    ret_val = g_cfg_val;

    return ret_val;
}

/* This function stores the peristent credential list sent by the user */
INLINE void set_p2p_persist_cred(UWORD8 *val)
{
    memcpy(&g_persist_list, val, sizeof(p2p_persist_list_t));
}

INLINE void set_p2p_find_to(UWORD8 val)
{
    g_p2p_find_to = val;
}

INLINE UWORD8 get_p2p_find_to(void)
{
    return g_p2p_find_to;
}

#endif /* MGMT_P2P_H */
#endif /* MAC_P2P */
