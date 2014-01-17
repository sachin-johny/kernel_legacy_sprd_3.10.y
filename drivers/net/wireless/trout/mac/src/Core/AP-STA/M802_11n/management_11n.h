/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : management_11n.h                                     */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      definitions for 802.11n mode                         */
/*                                                                           */
/*  List of Functions : initialize_11n                                       */
/*                      get_ampdu_lut_index                                  */
/*                      del_ampdu_lut_index                                  */
/*                      set_ht_operating_mode                                */
/*                      is_rifs_prot_enabled                                 */
/*                      enable_rifs                                          */
/*                      disable_rifs                                         */
/*                      enable_rifs_prot                                     */
/*                      set_capability_11n                                   */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

#ifndef MANAGEMENT_11N_H
#define MANAGEMENT_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "maccontroller.h"
#include "index_util.h"
#include "mh.h"
#include "frame_11e.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_AMSDU_SIZE_WITH_AMPDU 4095
#define AMPDU_LUT_IDX_BMAP_LEN    ((MAX_AMPDU_LUT_SIZE >> 3) + 1)
#define INVALID_AMPDU_LUT_INDEX   (0)

#ifdef NO_ACTION_RESET
#define MAX_SAVE_ACTION_REQ_LEN     24
#define MAX_NUM_SAVED_ACTION_REQ    8
#endif /* NO_ACTION_RESET */

/* Power in dBm for ignoring 20/40 TE's */
#define RSSI_2040_IGNORE            (WORD8)(-101)
#define REG_DOMAIN                  32  /* This should be a MIB */

/* Action frame related constants */
#define N_P_ACTION_REQ              7
#define N_P_ACTION_QUERY_TID_LEN    32
#define N_P_ACT_Q_TID_INFO_OFFSET   10
#define N_P_ACTION_AMSDU_TX_SETUP   BIT3
#define N_P_ACTION_AMPDU_TX_SETUP   BIT2
#define N_P_ACTION_BA_TX_SETUP      BIT1
#define N_P_ACTION_BA_RX_SETUP      BIT0


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Miscellaneous event types for 802.11n */
typedef enum {MISC_AMSDU_TX          = 0x50,
              MISC_TX_PENDING_FRAMES = 0x51,
              MISC_BA_TX_STALL       = 0x52,
              MISC_BA_RX_STALL       = 0x53,
} EVENT_TYPESUBTYPE_11N_T;

/* The MLME Request types allowed in AP mode */
typedef enum {MLME_ADDBA_REQ_TYPE = ADDBA_REQ_TYPE,
              MLME_DELBA_REQ_TYPE = DELBA_TYPE
} AP_MLME_ACTION_TYPE_T;

/* Private action requests for 802.11n */
typedef enum {A_MSDU_START = 0,
              A_MSDU_END   = 1,
              A_MPDU_START = 2,
              A_MPDU_END   = 3,
              A_QUERY_TID  = 4,
              A_QUERY_ALL  = 5,
              A_ALL_CLEAR  = 0xFF
} P_11N_ACTION_TYPE_T;

/* Status Codes for Authentication and Association Frames in 802.11n*/
typedef enum {MISMATCH_HTCAP = 27,
              MISMATCH_PCO   = 29
} STATUS_CODE_11N_T;

#ifdef NO_ACTION_RESET
/* Action categories for saving/restoring */
typedef enum {INVALID_ACTION_CAT = 0,
              BA_ACTION_CAT      = 1,
              AMSDU_ACTION_CAT   = 2,
              AMPDU_ACTION_CAT   = 3
} SAVE_ACTION_CATEGORY_T;
#endif /* NO_ACTION_RESET */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* ADDBA request */
typedef struct
{
    UWORD8  dst_addr[6];
    UWORD8  tid;
    UWORD8  ba_policy;
    UWORD8  buff_size;
    UWORD16 timeout;
    UWORD16 addba_timeout;
} addba_req_t;

typedef struct
{
    UWORD8 dst_addr[6];
    UWORD8 tid;
    UWORD8 direction;
    UWORD8 reason;
} delba_req_t;

typedef struct
{
    UWORD8 req_type;
    union
    {
        addba_req_t addba;
        delba_req_t delba;
    } axn_req;
} action_req_t;

typedef struct
{
    UWORD8  amsdu_maxnum;   /* Maximum number of sub-MSDUs in an AMSDU */
    UWORD16 amsdu_maxsize;  /* Maximum size of an AMSDU                */
    UWORD16 amsdu_to_intvl; /* Timeout interval for AMSDU transmission */
    UWORD8  ampdu_maxnum;   /* Maximum number for aggregation in AMPDU */
    UWORD16 ampdu_to_intvl; /* Timeout interval for AMPDU transmission */
    UWORD8  ampdu_lut_idx;  /* Index in the H/w AMPDU LUT              */
    UWORD8  ampdu_ack_pol;  /* AMPDU Ack-Policy. 0=>Normal 1=>BlockAck */
} ht_tx_struct_t;

typedef struct
{
#ifdef AUTORATE_FEATURE
    UWORD8         tx_mcs_index;   /* Index of current tx mcs used      */
    BOOL_T         tx_sgi;         /* Flag to indicate to TX at SGI   */
#endif /* AUTORATE_FEATURE */
    UWORD8         ht_capable;             /* HT capable                    */
    UWORD8         ldpc_cod_cap;           /* LDPC coding capability        */
    UWORD8         chan_width;             /* Channel Width                 */
    UWORD8         smps_mode;              /* SM Power Save mode            */
    UWORD8         greenfield;             /* Greenfield supported          */
    UWORD8         short_gi_20;            /* Short GI for 20 MHz supported */
    UWORD8         short_gi_40;            /* Short GI for 40 MHz supported */
    UWORD8         rx_stbc;                /* Rx STBC supported             */
    UWORD8         lsig_txop_prot;         /* L-SIG TXOP Protection support */
    UWORD8         max_rx_ampdu_factor;    /* Max AMPDU Rx Factor           */
    UWORD8         min_mpdu_start_spacing; /* Min AMPDU Start Spacing       */
    UWORD8         htc_support;            /* HTC field is supported        */
    UWORD8         dsss_cck_40mhz;         /* DSSS/CCK Mode in 40 MHz       */
    UWORD16        sta_amsdu_maxsize;      /* Max AMSDU size supported      */
    UWORD8         rx_mcs_bitmask[MCS_BITMASK_LEN]; /* Rx MCS bitmask       */
    UWORD8         num_ampdu_tx_sess;      /* Number of TX-AMPDU Sessions   */
    ht_tx_struct_t ht_tx_buff[NUM_TIDS]; /* HT transmit structure buffers */
    ht_tx_struct_t *ht_tx_ptr[NUM_TIDS]; /* Pointers to HT transmit structures    */
} ht_struct_t;

#ifdef NO_ACTION_RESET
/* Structure for saving action requests */
typedef struct
{
    UWORD8 category;
    UWORD8 tid;
    UWORD8 req_len;
    UWORD8 da[MAC_ADDRESS_LEN];
    UWORD8 req[MAX_SAVE_ACTION_REQ_LEN];
} save_action_req_t;
#endif /* NO_ACTION_RESET */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_rifs_prot_enabled;
extern UWORD8 g_ampdu_lut_index_table[AMPDU_LUT_IDX_BMAP_LEN];

#ifdef NO_ACTION_RESET
extern save_action_req_t g_saved_action_req[MAX_NUM_SAVED_ACTION_REQ];
#endif /* NO_ACTION_RESET */

extern BOOL_T g_tx_ba_setup;

extern UWORD8 g_act_req_q_type;
extern UWORD8 g_act_req_q_tid;
extern UWORD8 g_act_req_q_ra[6];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void initialize_11n(void);
extern void misc_11n_event(mac_struct_t *mac, UWORD8 *msg);
extern void set_11n_p_action_req(UWORD8 *msg);
extern void update_tx_mib_11n(UWORD8 *msa, UWORD8 retry_count,
                              UWORD16 num_bytes, BOOL_T is_success);
extern void update_rx_mib_11n(UWORD8 *msa, UWORD16 num_bytes);
extern void handle_mlme_amsdu_start(UWORD8 *req);
extern void handle_mlme_amsdu_end(UWORD8 *req);
extern void handle_mlme_ampdu_start(UWORD8 *req);
extern void handle_mlme_ampdu_end(UWORD8 *req);
extern void handle_mlme_addba_req(UWORD8 *aba_req);
extern void handle_mlme_delba_req(UWORD8 *dba_req);
extern void reset_tx_amsdu_session(void *entry, UWORD8 tid);
extern void reset_tx_ampdu_session(void *entry, UWORD8 tid);
extern void handle_wlan_addba_req(UWORD8 *sa, UWORD8 *data);
extern void handle_wlan_addba_rsp(UWORD8 *sa, UWORD8 *data);
extern void handle_wlan_delba(UWORD8 *sa, UWORD8 *data);

#ifdef NO_ACTION_RESET
extern void save_action_req(UWORD8 *req, UWORD8 type, UWORD8 action_category);
#endif /* NO_ACTION_RESET */

extern UWORD16 get_txop_frag_min_len_11n(void *entry);
extern void delete_all_ba_handles(void);
extern UWORD8* handle_action_req_query_tid(void);
extern UWORD8* handle_action_req_query_all(void);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function returns an unused AMPDU LUT index from the AMPDU LUT index  */
/* bitmap table. Note that the AMPDU LUT index value maintained is 1 greater */
/* than the index in the AMPDU LUT index bitmap table used.  This is since   */
/* 0 is not a valid AMPDU LUT index value.                                   */
INLINE UWORD16 get_ampdu_lut_index(void)
{
    UWORD16 idx = 0;

    idx = get_lut_index(g_ampdu_lut_index_table, AMPDU_LUT_IDX_BMAP_LEN,
                        MAX_AMPDU_LUT_SIZE);

    /* Increment the index by 1 and return. For an invalid returned index    */
    /* (MAX_AMPDU_LUT_SIZE), this will become 0 which will be treated as an  */
    /* (INVALID_AMPDU_LUT_INDEX)                                             */
    return idx + 1;
}

/* This function deletes a AMPDU LUT index from the AMPDU LUT index bitmap   */
/* table.  Note that the AMPDU LUT index value maintained is 1 greater than  */
/* the index in the AMPDU LUT index bitmap table used.                       */
INLINE void del_ampdu_lut_index(UWORD16 idx)
{
    if(INVALID_AMPDU_LUT_INDEX == idx)
        return;

    del_lut_index(g_ampdu_lut_index_table, idx - 1);
}

/* This function sets the HT operating mode in MIB and MAC H/w */
INLINE void set_ht_operating_mode(UWORD8 val)
{
        mset_HTOperatingMode(val);

#ifndef HT_OP_MODE_3_PROT
        if(((val == 3) || (val == 1))&& (is_autoprot_enabled() == BFALSE) &&
           (get_protection() == NO_PROT))
        {
            /* If operating mode is being set to 3 but No protection is the  */
            /* user configuration, reset the value to be set to 0 so that no */
            /* protection is done by the MAC HW in such a case.              */
            val = 0;
        }
#endif /* HT_OP_MODE_3_PROT */

        set_machw_ht_op_mode(val);
}

/* This function checks if RIFS protection is enabled */
INLINE UWORD8 is_rifs_prot_enabled(void)
{
    return g_rifs_prot_enabled;
}

/* This function disables RIFS transmission in MIB and MAC H/w. It disables  */
/* H/w GF/RIFS protection if GF protection is not enabled.                   */
INLINE void disable_rifs(void)
{
    if(mget_RIFSMode() == TV_TRUE)
    {
        mset_RIFSMode(TV_FALSE);
        disable_machw_rifs();

        if(get_protection() != GF_PROT)
            disable_machw_gf_rifs_prot();
    }

    g_rifs_prot_enabled = 0;
}

/* This functions enables RIFS transmission in MIB and MAC H/w. It does not  */
/* set RIFS protection. This is checked separately as required.              */
INLINE void enable_rifs(void)
{
    if(mget_RIFSMode() == TV_FALSE)
    {
        mset_RIFSMode(TV_TRUE);
        /* Enable RIFS only if it is allowed by the user */
        if(g_user_allow_rifs_tx == 1)
            enable_machw_rifs();
    }
}

/* This functions enables RIFS protection. It enables H/w GF/RIFS protection */
/* if GF protection is not already enabled.                                  */
INLINE void enable_rifs_prot(void)
{
    if(get_11n_rifs_prot_enable() == 1)
    {
        g_rifs_prot_enabled = 1;

        if(get_protection() != GF_PROT)
            enable_machw_gf_rifs_prot();
    }
}

/* This function adds a new user to the Tx-descriptor & and all its */
/* associated buffers.                                              */
INLINE void add_user_tx_dscr(UWORD32 *tx_dscr)
{
    UWORD8  *buff_hdl = 0;
    UWORD32 indx = 0;
    UWORD32 num_sub_msdu = get_tx_dscr_num_submsdu((UWORD32 *)tx_dscr);

#ifdef TX_MACHDR_IN_DSCR_MEM
    /* Add user for the MSDU header */
    buff_hdl = (UWORD8 *)get_tx_dscr_buffer_addr(tx_dscr);
    if((NULL != buff_hdl) &&
       (BFALSE == is_buffer_in_tx_dscr((UWORD8 *)tx_dscr, buff_hdl)))
    {
        /* Get the memory pool to which the buffer belongs */
        mem_handle = get_mem_pool_hdl((void *)buff_hdl);
        mem_add_users(mem_handle, buff_hdl, 1);
    }
#endif /* TX_MACHDR_IN_DSCR_MEM */

    /* Add user for all the sub-MSDU buffers */
    for(indx = 0; indx < num_sub_msdu; indx++)
    {
        buff_hdl = (UWORD8 *)get_tx_dscr_submsdu_buff_hdl(tx_dscr, indx);
        mem_add_users(g_shared_pkt_mem_handle, buff_hdl, 1);
    }

    /* Add a user to the Tx descriptor buffer/s */
    add_user_tx_dscr_buffer(g_shared_dscr_mem_handle, (UWORD8 *)tx_dscr);
}

/* This function updates the Ack-Policy of the frames if they belong to a */
/* AMPDU session.                                                         */
INLINE void update_serv_class_11n(ht_struct_t *ht_entry, UWORD8 tid,
                           UWORD8 *serv_class)
{
    ht_tx_struct_t *ht_tx_hdl = NULL;

    if((tid >= 16) || (ht_entry->ht_tx_ptr[tid] == NULL))
        return;

    ht_tx_hdl = ht_entry->ht_tx_ptr[tid];

    /* The Service-Class is updated only when an AMPDU session is */
    /* setup with Ack-Policy set to No-Ack.                       */
    if((ht_tx_hdl->ampdu_maxnum > 0) &&
       (ht_tx_hdl->ampdu_ack_pol == NO_ACK))
    {
        *serv_class = NO_ACK;
    }
}

/* This function sets the capability for 11n protocol */
INLINE void set_capability_11n(UWORD8* data, UWORD16 index)
{
    /* Cap Info Field: Bit 15 - Immediate Block ACK */
    if(mget_ImmediateBlockAckOptionImplemented() == TV_TRUE)
        data[index + 1] |= BIT7;
}

#ifdef NO_ACTION_RESET
/* This function initializes the saved action request list */
INLINE void init_save_action_req_list_11n(void)
{
    UWORD8            i        = 0;
    save_action_req_t *axn_req = 0;

    /* Check the global saved action request list for a matching entry or a  */
    /* new entry.                                                            */
    for(i = 0; i < MAX_NUM_SAVED_ACTION_REQ; i++)
    {
        axn_req = &g_saved_action_req[i];

        axn_req->tid      = INVALID_TID;
        axn_req->category = INVALID_ACTION_CAT;
        axn_req->req_len  = 0;
        mem_set(axn_req->da,  0, MAC_ADDRESS_LEN);
        mem_set(axn_req->req, 0, MAX_SAVE_ACTION_REQ_LEN);
    }
}

/* This function finds the index to an entry in the global saved action      */
/* request list with matching DA, TID and Action Category. In case no        */
/* matching entry is found the index to the first new entry is returned.     */
INLINE UWORD8 get_saved_action_req_idx(UWORD8 *da, UWORD8 tid, UWORD8 cat)
{
    UWORD8            i        = 0;
    UWORD8            ret_idx  = MAX_SAVE_ACTION_REQ_LEN;
    save_action_req_t *axn_req = 0;
    BOOL_T            found    = BFALSE;

    /* Check the global saved action request list for a matching entry or a  */
    /* new entry.                                                            */
    for(i = 0; i < MAX_NUM_SAVED_ACTION_REQ; i++)
    {
        axn_req = &g_saved_action_req[i];

        if((axn_req->tid == tid) && (axn_req->category == cat) &&
           (mac_addr_cmp(axn_req->da, da) == BTRUE))
        {
            /* If DA, TID and Category match save this index and break */
            ret_idx = i;
            break;
        }

        if((found == BFALSE) && (axn_req->category == INVALID_ACTION_CAT))
        {
            /* If no match save the first index of the entry that is clear */
            ret_idx = i;
            found   = BTRUE;
        }
    }

    return ret_idx;
}

/* This function restores all the saved action requests */
INLINE void restore_all_saved_action_req(void)
{
    UWORD8            i        = 0;
    save_action_req_t *axn_req = 0;

    for(i = 0; i < MAX_NUM_SAVED_ACTION_REQ; i++)
    {
        axn_req = &g_saved_action_req[i];

        switch(axn_req->category)
        {
        case BA_ACTION_CAT:
        {
            handle_mlme_addba_req(axn_req->req);
        }
        break;

        case AMSDU_ACTION_CAT:
        {
            handle_mlme_amsdu_start(axn_req->req);
        }
        break;

        case AMPDU_ACTION_CAT:
        {
            handle_mlme_ampdu_start(axn_req->req);
        }
        default:
        {
            /* Do nothing. The saved action type is not valid. */
        }
        break;
        }
    }
}


/* This function restores all the saved action requests for a given peer address */
INLINE void restore_all_saved_action_req_per_sta(UWORD8 *peer_addr, UWORD8 asoc_id)
{
    UWORD8            i        = 0;
    save_action_req_t *axn_req = 0;

    for(i = 0; i < MAX_NUM_SAVED_ACTION_REQ; i++)
    {
        axn_req = &g_saved_action_req[i];

        if((mac_addr_cmp(axn_req->da, peer_addr) == BTRUE) ||
           ((axn_req->da[0] == 0xFF) && (axn_req->da[1] == asoc_id)))
        {
            /* Action request saved is with given peer STA. Restore it. */
            switch(axn_req->category)
            {
            case BA_ACTION_CAT:
            {
                handle_mlme_addba_req(axn_req->req);
            }
            break;

            case AMSDU_ACTION_CAT:
            {
                handle_mlme_amsdu_start(axn_req->req);
            }
            break;

            case AMPDU_ACTION_CAT:
            {
                handle_mlme_ampdu_start(axn_req->req);
            }
            default:
            {
                /* Do nothing. The saved action type is not valid. */
            }
            break;
            }
        }
        else
        {
            /* Action request saved is with different peer STA. Do nothing. */
        }
    }
}

/* Thsi function clears an action for the given entry */
INLINE void clear_action(UWORD8 idx)
{
    save_action_req_t *axn_req = 0;

    /* Get the pointer to the entry corresponding to the index found */
    axn_req = &g_saved_action_req[idx];

    /* Clear the entry only if a matching entry was found. If no entry was   */
    /* found nothing needs to be done.                                       */
    if(axn_req->category != INVALID_ACTION_CAT)
    {
        axn_req->tid      = INVALID_TID;
        axn_req->category = INVALID_ACTION_CAT;
        axn_req->req_len  = 0;

        mem_set(axn_req->da,  0, MAC_ADDRESS_LEN);
        mem_set(axn_req->req, 0, MAX_SAVE_ACTION_REQ_LEN);
    }
}
/* This function clears all the saved action requests */
INLINE void clear_all_action_req(void)
{
    UWORD8            i                  = 0;
    save_action_req_t *axn_req           = 0;
    UWORD8            pseudo_del_pkt[12] = {0};


    for(i = 0; i < MAX_NUM_SAVED_ACTION_REQ; i++)
    {
        axn_req = &g_saved_action_req[i];

        switch(axn_req->category)
        {
        case BA_ACTION_CAT:
        {
            /* Format of DELBA Body                                       */
            /* ---------------------------------------------------------- */
            /* | Ignored | Address   | TID    | Direction | Result Code | */
            /* ---------------------------------------------------------- */
            /* | 2       |   6       |     1  |     1     |     1       | */
            /* ---------------------------------------------------------- */
            mac_addr_cpy(pseudo_del_pkt + 2, axn_req->da);
            pseudo_del_pkt[8]  = axn_req->tid;
            pseudo_del_pkt[9]  = 1; /* INITIATOR */
            pseudo_del_pkt[10] = 0;

            handle_mlme_delba_req(pseudo_del_pkt); /* Initiator session */
        }
        break;

        case AMSDU_ACTION_CAT:
        {
            /* Format of A-MSDU End Request                       */
            /* -------------------------------------------------- */
            /* | Receiver MAC Address           | TID           | */
            /* -------------------------------------------------- */
            /* | 6                              | 1             | */
            /* -------------------------------------------------- */
            mac_addr_cpy(pseudo_del_pkt, axn_req->da);
            pseudo_del_pkt[6] = axn_req->tid;
            handle_mlme_amsdu_end(pseudo_del_pkt);
        }
        break;

        case AMPDU_ACTION_CAT:
        {
            /* Format of A-MPDU End Request                       */
            /* -------------------------------------------------- */
            /* | Receiver MAC Address           | TID           | */
            /* -------------------------------------------------- */
            /* | 6                              | 1             | */
            /* -------------------------------------------------- */
            mac_addr_cpy(pseudo_del_pkt, axn_req->da);
            pseudo_del_pkt[6] = axn_req->tid;
            handle_mlme_ampdu_end(pseudo_del_pkt);
        }
        default:
        {
            /* Do nothing. The saved action type is not valid. */
        }
        break;
        }

        /* Delete the saved action entry */
        clear_action(i);
    }
}

/* This function updates the given entry with the incoming action */
INLINE void update_action(UWORD8 idx, UWORD8 cat, UWORD8 len, UWORD8 tid,
                          UWORD8 *da, UWORD8 *req)
{
    save_action_req_t *axn_req = 0;

    /* Get the pointer to the entry corresponding to the index found */
    axn_req = &g_saved_action_req[idx];

    /* Update the Destination address, TID, Action Category and Action       */
    /* length if this is a new entry.                                        */
    if(axn_req->category == INVALID_ACTION_CAT)
    {
        axn_req->tid      = tid;
        axn_req->category = cat;
        axn_req->req_len  = len;
        mac_addr_cpy(axn_req->da, da);
    }

    /* Copy the incoming Action request */
    memcpy(axn_req->req, req, len);
}

#endif /* NO_ACTION_RESET */


/* This function returns the number of Block-Ack TX sessions setup with */
/* the specified station.                                               */
INLINE UWORD8 get_num_tx_ampdu_sessions_11n(ht_struct_t *ht_hdl)
{
    UWORD8      retval  = 0;

    if(ht_hdl != NULL)
        retval = ht_hdl->num_ampdu_tx_sess;

    return retval;
}

/* This function increments the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void incr_num_tx_ampdu_sessions_11n(ht_struct_t *ht_hdl)
{
    if(ht_hdl != NULL)
        ht_hdl->num_ampdu_tx_sess++;
}

/* This function decrements the number of Block-Ack TX sessions setup with */
/* the specified station.                                                  */
INLINE void decr_num_tx_ampdu_sessions_11n(ht_struct_t *ht_hdl)
{
    if(ht_hdl != NULL)
        ht_hdl->num_ampdu_tx_sess--;
}

/* This function handle config response for QUERY-TID and QUERY-ALL action */
/* requests                                                                */
INLINE UWORD8* handle_action_req_query(void)
{
    if(g_reset_mac_in_progress == BTRUE)
        return 0;

    if(g_act_req_q_type == A_QUERY_TID)
        return handle_action_req_query_tid();
    else
        return handle_action_req_query_all();
}

/* This function updates the AMSDU-TX paramters in given config response   */
/* buffer in desired format for QUERY-TID action request                   */
/* AMSDU Session Paramaters Format                   */
/* +--------------+----------------+---------------+ */
/* | MaxNum MSDUs | Max AMSDU Size | AMSDU Timeout | */
/* +--------------+----------------+---------------+ */
/* |      1       |        2       |       2       | */
/* +--------------+----------------+---------------+ */
INLINE UWORD8 set_amsdu_session_params(UWORD8  *buffer,
                                       UWORD8  amsdu_maxnum,
                                       UWORD16 amsdu_maxsize,
                                       UWORD16 amsdu_to_intvl)
{
    UWORD8 index = 0;

    buffer[index++] = amsdu_maxnum;
    buffer[index++] = (UWORD8)(amsdu_maxsize & 0x00FF);
    buffer[index++] = (UWORD8)(amsdu_maxsize >> 8);
    buffer[index++] = (UWORD8)(amsdu_to_intvl & 0x00FF);
    buffer[index++] = (UWORD8)(amsdu_to_intvl >> 8);

    return index;
}

/* This function updates the AMPDU-TX paramters in given config response   */
/* buffer in desired format for QUERY-TID action request                   */
/* AMPDU Session Paramaters Format                   */
/* +--------------+----------------+---------------+ */
/* | MaxNum MPDUs |  AMPDU Timeout | AMPDU ACK-Pol | */
/* +--------------+----------------+---------------+ */
/* |      1       |        2       |       1       | */
/* +--------------+----------------+---------------+ */
INLINE UWORD8 set_ampdu_session_params(UWORD8  *buffer,
                                       UWORD8  ampdu_maxnum,
                                       UWORD16 ampdu_to_intvl,
                                       UWORD8  ampdu_ack_pol)
{
    UWORD8 index = 0;

    buffer[index++] = ampdu_maxnum;
    buffer[index++] = (UWORD8)(ampdu_to_intvl & 0x00FF);
    buffer[index++] = (UWORD8)(ampdu_to_intvl >> 8);
    buffer[index++] = ampdu_ack_pol;

    return index;
}

/* This function updates the BA-TX paramters in given config response      */
/* buffer in desired format for QUERY-TID action request                   */
/* Block ACK Initiator Session Paramaters Format                   */
/* +--------+-----------+-------+----------+-----------+---------+ */
/* | BA-Pol | Buff-Size | BA-TO | ADDBA-TO | Max PendQ | PndQ-TO | */
/* +--------+-----------+-------+----------+-----------+---------+ */
/* |   1    |     1     |    2  |     2    |     1     |    2    | */
/* +--------+-----------+-------+----------+-----------+---------+ */
INLINE UWORD8 set_ba_initiator_params(UWORD8  *buffer,
                                      UWORD8  ba_policy,
                                      UWORD8  buff_size,
                                      UWORD16 ba_timeout,
                                      UWORD16 addba_timeout,
                                      UWORD8  max_pend,
                                      UWORD16 buff_timeout)
{
    UWORD8 index = 0;

    buffer[index++] = ba_policy;
    buffer[index++] = buff_size;
    buffer[index++] = (UWORD8)(ba_timeout & 0x00FF);
    buffer[index++] = (UWORD8)(ba_timeout >> 8);
    buffer[index++] = (UWORD8)(addba_timeout & 0x00FF);
    buffer[index++] = (UWORD8)(addba_timeout >> 8);
    buffer[index++] = max_pend;
    buffer[index++] = (UWORD8)(buff_timeout & 0x00FF);
    buffer[index++] = (UWORD8)(buff_timeout >> 8);

    return index;
}

/* This function updates the BA-RX paramters in given config response        */
/* buffer in desired format for QUERY-TID action request                     */
/* Blcok ACK Recepient Session Paramaters Format     */
/* +--------------+----------------+---------------+ */
/* |   BA-Policy  |   Buffer-Size  |   BA-Timeout  | */
/* +--------------+----------------+---------------+ */
/* |      1       |        1       |       2       | */
/* +--------------+----------------+---------------+ */
INLINE UWORD8 set_ba_recepient_params(UWORD8  *buffer,
                                      UWORD8  ba_policy,
                                      UWORD8  buff_size,
                                      UWORD16 ba_timeout)
{
    UWORD8 index = 0;

    buffer[index++] = ba_policy;
    buffer[index++] = buff_size;
    buffer[index++] = (UWORD8)(ba_timeout & 0x00FF);
    buffer[index++] = (UWORD8)(ba_timeout >> 8);

    return index;
}
#endif  /* MANAGEMENT_11N_H */
#endif /* MAC_802_11N */
