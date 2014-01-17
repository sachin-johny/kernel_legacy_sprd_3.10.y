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
/*  File Name         : phy_802_11n.h                                        */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions for 802.11n PHY.                           */
/*                                                                           */
/*  List of Functions : Access functions for PHY parameters                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef PHY_802_11n

#ifndef PHY_802_11n_H
#define PHY_802_11n_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "cglobals.h"
#include "mib_802_11n.h"
#include "ch_info.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define LONG_PREAMBLE_LEN_A 16
#define LONG_PREAMBLE_LEN_B 144
#define PLCP_HEADER_LEN_A   4
#define PLCP_HEADER_LEN_B   48
#define ACK_CTS_FRAME_LEN   14

#define SHORTSLOTTIME       9
#define LONGSLOTTIME        20

#define SIFSTIME            16
#define EIFSTIME_GONLY      (SIFSTIME + ACK_CTS_FRAME_LEN * 8 + \
                            PLCP_HEADER_LEN_A + LONG_PREAMBLE_LEN_A)
#define EIFSTIME_GMIXED     (SIFSTIME + ACK_CTS_FRAME_LEN * 8 + \
                            PLCP_HEADER_LEN_B + LONG_PREAMBLE_LEN_B)

#define MAX_PHY_OVERHEAD_A  (PLCP_HEADER_LEN_A + LONG_PREAMBLE_LEN_A)
#define MAX_PHY_OVERHEAD_B  (PLCP_HEADER_LEN_B + LONG_PREAMBLE_LEN_B)
#define MAX_PHY_OVERHEAD_N  40

#define NUM_DR_PHY_802_11G        12
#define NUM_DR_PHY_802_11G_ONLY    8
#define NUM_BR_PHY_802_11G_11B_1  4
#define NUM_BR_PHY_802_11G_11B_2  7
#define NUM_BR_PHY_802_11G_GONLY  3
#define NUM_NBR_PHY_802_11G_11B_1 8
#define NUM_NBR_PHY_802_11G_11B_2 5
#define NUM_NBR_PHY_802_11G_GONLY 5

#define NUM_DR_PHY_802_11A        8
#define NUM_BR_PHY_802_11A        3
#define NUM_NBR_PHY_802_11A       5

#define NUM_DR_PHY_802_11B        4
#define NUM_BR_PHY_802_11B        2
#define NUM_NBR_PHY_802_11B       2

#define MAX_BASIC_MCS_VAL         7
#define MIN_BASIC_MCS_VAL         0

#define NONHT_11BS_PHY_TX_MODE 0x00010140
#define NONHT_11BL_PHY_TX_MODE 0x00010141
#define NONHT_11A_PHY_TX_MODE  0x00010142
#define DEFAULT_HT_PHY_TX_MODE 0x00010146

#define MAX_NUM_RATES (NUM_DR_PHY_802_11G + MAX_NUM_MCS_SUPPORTED)

/* Offset of the Secondary Channel Index from the Primary Channel Index for  */
/* different frequency bands                                                 */
#define SEC_CHAN_INDEX_OFFSET_START_FREQ_5       1
#define AFFECTED_CHAN_OFFSET_START_FREQ_5        0
#define SEC_CHAN_INDEX_OFFSET_START_FREQ_2       4
#define AFFECTED_CHAN_OFFSET_START_FREQ_2        3

#define INVALID_MCS_VALUE         255
#define MCS32                      32

#define CURR_TX_BW_AUTO             0
#define CURR_TX_BW_20MHZ_ONLY       1

// 20120830 caisf add, merged ittiam mac v1.3 code
/* AntSet: Default 0x3 for both 1x1 & 2x2 rates, to keep the power constant */
#define CURR_ANT_SET_INIT_VALUE     0x0F070303

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define IS_RATE_MCS(A)      (((A) & BIT7)? BTRUE : BFALSE)
#define GET_MCS(A)          ((A) & ~BIT7)
#define IS_OFDM_RATE(A)     ((((A) & BIT7) | ((A) & BIT3))? BTRUE : BFALSE)
#define IS_RATE_11B(A)      ((((A) & BIT7) | ((A) & BIT3))? BFALSE : BTRUE)
// 20120830 caisf add, merged ittiam mac v1.3 code
#define EQUIVALENT_1X1_RATE(A) (BIT7 | (GET_MCS(A) % 8))


/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {B_ONLY_MODE        = 0,
              G_ONLY_MODE,
              G_MIXED_11B_1_MODE,
              G_MIXED_11B_2_MODE,
              NUM_G_OPERATING_MODE
} G_OPERATING_MODE_T;

typedef enum {NTYPE_MIXED      = 0,
              NTYPE_HT_ONLY,
              NTYPE_HT_2040_ONLY,
              NUM_N_OPERATING_TYPE
} N_OPERATING_TYPE_T;

/* Protection mode for MAC */
typedef enum {NO_PROT  = 1, /* Do not use any protection       */
              ERP_PROT, /* Protect all ERP frame exchanges */
              HT_PROT,  /* Protect all HT frame exchanges  */
              GF_PROT,  /* Protect all GF frame exchanges  */
              NUM_PROTECTION_MODE
} PROTECTION_MODE_T;

/* ERP Protection type */
typedef enum {G_SELF_CTS_PROT,
              G_RTS_CTS_PROT,
              NUM_G_PROTECTION_MODE
} G_PROTECTION_MODE_T;

/* Channel Bandwidth */
typedef enum {CH_BW_20MHZ  = 0, /* 20MHz Channel Bandwidth */
              CH_BW_40MHZ,      /* 40MHz Channel Bandwidth */
              CH_BW_40DL,       /* 40MHz Duplicate Legacy  */
              CH_BW_40DH        /* 40MHz Duplicate HT      */
} N_CH_BANDWIDTH_T;

/* Channel Offset */
typedef enum {CH_OFF_40   = 0, /* 40 MHz Band                 */
              CH_OFF_20U,      /* Upper 20 MHz in 40 MHz Band */
              CH_OFF_NONE,     /* No Offset Present           */
              CH_OFF_20L       /* Lower 20 MHz in 40 MHz Band */
} N_CH_OFFSET_T;

/* Secondary Channel Offset */
typedef enum {SCN = 0,  /* No Secondary Channel    */
              SCA = 1,  /* Secondary Channel Above */
              SCB = 3   /* Secondary Channel Below */
} N_SEC_CH_OFFSET_T;

/* Channel type in a scan */
typedef enum {CH_TYPE_NONE = 0,  /* No Network present Channel */
              CH_TYPE_PRIMARY = 1,  /* Primary channel of a Network */
              CH_TYPE_SECONDARY = 2,  /* Secondary Channel of a 20/40 Network */
}N_2040_CH_TYPE_T;

/* Channel Offset */
typedef enum {RTS_CTS_NONHT_PROT = 0, /* RTS-CTS at non-HT rate      */
              FIRST_FRAME_NONHT_PROT, /* First frame at non-HT rate  */
              LSIG_TXOP_PROT,         /* LSIG TXOP Protection        */
              FIRST_FRAME_MIXED_PROT, /* First frame at Mixed format */
              NUM_N_PROTECTION_MODE
} N_PROTECTION_MODE_T;

typedef enum {G_SHORT_PREAMBLE = 0, /* Short Preamble          */
              G_LONG_PREAMBLE  = 1, /* Long Preamble           */
              G_AUTO_PREAMBLE  = 2, /* Auto Preamble Selection */
              NUM_G_PREAMBLE
}G_PREAMBLE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD32 aRIFSTime;
    UWORD32 aSlotTime;
    UWORD32 aSIFSTime;
    UWORD32 aCCATime;
    UWORD32 aRxStartDelay;
    UWORD32 aRxTxTurnaroundTime;
    UWORD32 aTxPLCPDelay;
    UWORD32 aRxPLCPDelay;
    UWORD32 aRxTxSwitchTime;
    UWORD32 aTxRampOnTime;
    UWORD32 aTxRampOffTime;
    UWORD32 aTxRFDelay;
    UWORD32 aRxRFDelay;
    UWORD32 aAirPropagationTime;
    UWORD32 aMACProcessingDelay;
    UWORD32 aPreambleLength;
    UWORD32 aSTFOneLength;
    UWORD32 aSTFTwoLength;
    UWORD32 aLTFOneLength;
    UWORD32 aLTFTwoLength;
    UWORD32 aPLCPHeaderLength;
    UWORD32 aPLCPSigTwoLength;
    UWORD32 aPSDUMaxLength;
    UWORD32 aPPDUMaxTime;
    UWORD32 aIUStime;
    UWORD32 aDTT2UTTTime;
    UWORD32 aCWmin;
    UWORD32 aCWmax;
    UWORD32 aMaxCSIMatricesReportDelay;
} phy_char_t;

typedef struct
{
    UWORD8 mac_rate; /* Indicates rate as required by MAC */
    UWORD8 phy_rate; /* Indicates rate as required by PHY */
    UWORD8 mbps;     /* Corresponding rate for interfaces */
} data_rate_t;

typedef struct
{
    UWORD8 num_rates;
    UWORD8 num_br;
    UWORD8 num_nbr;
    UWORD8 max_br;
    UWORD8 min_br;
    data_rate_t rates[NUM_DR_PHY_802_11G];
} curr_data_rate_t;

/*****************************************************************************/
/* Extern Variable Declaration                                               */
/*****************************************************************************/

extern UWORD8      g_11n_op_type;
extern UWORD8      g_11g_op_mode;
extern UWORD8      g_prot_mode;
extern UWORD8      g_erp_prot_type;
extern UWORD8      g_ht_prot_type;
extern UWORD8      g_11n_obss_detection;
extern UWORD8      g_11n_rifs_prot_enable;
extern UWORD8      g_11n_rifs_prot_type;
extern UWORD8      g_pr_phy_802_11n;
extern UWORD8      g_pr_flag_phy_802_11n;
extern UWORD8      g_curr_rate_idx;
extern UWORD8      g_tx_mcs_phy_802_11n;
extern UWORD8      g_tx_bw_phy_802_11n;
extern UWORD32     g_tx_ant_set_phy_802_11n; // 20120830 caisf add, merged ittiam mac v1.3 code
extern BOOL_T      g_autoprot_enable;
extern const UWORD8 g_mcs_user_rate[MAX_NUM_MCS_SUPPORTED];
extern const data_rate_t g_rates_phy_802_11g[NUM_DR_PHY_802_11G];
extern curr_data_rate_t g_curr_rate_struct;
extern phy_char_t  PHY_C;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void    set_capability_phy_802_11n(UWORD8* data, UWORD16 index);
extern UWORD8  set_phy_params_802_11n(UWORD8* data, UWORD16 index);
extern UWORD8  get_tx_pr_phy_802_11n(UWORD8 rate);
extern UWORD8  get_current_channel_802_11n (UWORD8 *msa,UWORD16 rx_len);
extern UWORD16 set_erp_info_field_802_11n(UWORD8* data, UWORD8 index);
extern BOOL_T  check_capability_phy_802_11n(UWORD16 cap_info);
extern UWORD32 get_phy_tx_mode_802_11n(UWORD8 rate, UWORD8 pr, void *entry,
                                       UWORD8 reg_flag);
extern UWORD8 get_tx_pow_11n(UWORD8 dr, UWORD8 ptm); // 20120830 caisf add, merged ittiam mac v1.3 code
extern void init_phy_802_11n_curr_rates(void);
extern BOOL_T is_rate_enabled(UWORD8 mbps);
extern void add_mac_rate(UWORD8 mac_rate);
extern BOOL_T is_sec_channel_valid_11n(UWORD8 pri_idx,
                                       N_SEC_CH_OFFSET_T sec_offset);
#ifdef AUTORATE_FEATURE
extern void init_ar_table_phy_802_11n(void);
#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Rate support related                                                      */
/*****************************************************************************/

/* This function returns the maximum of basic rates supported by 11n PHY     */
/* operating in 802.11n HT only mode.                                        */
/* Returns rate in MCS (indicated by setting Bit 7)                          */
INLINE UWORD8 get_phy_802_11n_max_br_htonly(void)
{
    return (MAX_BASIC_MCS_VAL | BIT7);
}

/* This function returns the minimum of basic rates supported by 11n PHY     */
/* operating in 802.11n HT only mode.                                        */
/* Returns rate in MCS (indicated by setting Bit 7)                          */
INLINE UWORD8 get_phy_802_11n_min_br_htonly(void)
{
    return (MIN_BASIC_MCS_VAL | BIT7);
}

/* This function returns the number of basic rates supported by 11n PHY      */
/* operating in 802.11n HT only mode. In this mode the BSS basic rate set is */
/* empty. hence number of basic rates is 0.                                  */
INLINE UWORD8 get_phy_802_11n_num_br_htonly(void)
{
    return 0;
}

/* This function returns the current transmission rate of the STA/AP (MCS)   */
/* 11n PHY operating in 802.11n HT only mode. Bit 7 is set to indicate MCS.  */
INLINE UWORD8 get_phy_802_11n_tx_mcs(void)
{
    return g_tx_mcs_phy_802_11n;
}

/* This function sets the current transmission rate of the STA/AP            */
/* Input is in MCS. The same is saved in the global.                         */
INLINE void set_phy_802_11n_tx_mcs(UWORD8 mcs)
{
    g_tx_mcs_phy_802_11n = mcs;
}


/* This function returns the current transmission bandwidth of the STA/AP    */
/* 11n PHY operating in 802.11n HT only mode.                                */
INLINE UWORD8 get_phy_802_11n_tx_bw(void)
{
    return g_tx_bw_phy_802_11n;
}

/* This function sets the current transmission bandwidth of the STA/AP       */
/* Input is 0/1 for Auto/20 Only. The same is saved in the global.           */
INLINE void set_phy_802_11n_tx_bw(UWORD8 bw)
{
    g_tx_bw_phy_802_11n = bw;
}

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/* This function returns the current Antenna Set the STA/AP, 0 means Auto    */
INLINE UWORD32 get_phy_802_11n_ant_set(void)
{
    return g_tx_ant_set_phy_802_11n;
}

/* This function sets the current Antenna Setthe STA/AP, 0 means Auto        */
INLINE void set_phy_802_11n_ant_set(UWORD32 val)
{
	g_tx_ant_set_phy_802_11n = val;
}
#endif

/*****************************************************************************/
/* Preamble related                                                          */
/*****************************************************************************/

/* This function sets the preamble to be used for frame transmission */
INLINE void set_pr_phy_802_11n(UWORD8 pr, UWORD8 rate)
{
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        g_pr_phy_802_11n = pr;
    }
}

/* This function returns the user set preamble */
INLINE UWORD8 get_user_pr_phy_802_11n(void)
{
    return g_pr_phy_802_11n;
}

#endif /* PHY_802_11n_H */

/* This function returns the user rate in mbps given the PHY rate            */
/* for both 11g & 11n rates.                                                 */
INLINE UWORD8 get_phy_802_11n_pr_to_ur(UWORD8 phy_rate)
{
    UWORD8 i = 0;

    if(IS_RATE_MCS(phy_rate) != 0)
    {
        i = phy_rate & (~BIT7);

        if(i < MAX_NUM_MCS_SUPPORTED)
            return g_mcs_user_rate[i];
    }
    else
    {
        /* Return User rate (mbps) given the PHY rate. */
        for(i = 0; i < g_curr_rate_struct.num_rates; i++)
            if(g_curr_rate_struct.rates[i].phy_rate == phy_rate)
                return g_curr_rate_struct.rates[i].mbps;
    }

    /* If an invalid value of 'phy_rate' is passed, return 0 */
    return 0;
}

/* This function checks whether the frame fits in the given TXOP Limit. */
/* It returns 1 if it does and 0 if it fails.                           */
INLINE UWORD8 does_frame_fit_in_txop(UWORD16 txop_limit, UWORD16 frame_len,
                                     UWORD8 phy_rate)
{
    UWORD16 phy_hdr_time  = MAX_PHY_OVERHEAD_B;
    UWORD16 rate_mbps   = 0;
    UWORD16 frame_time  = 0;

    if(IS_OFDM_RATE(phy_rate) == BTRUE)
    {
        if(IS_RATE_MCS(phy_rate) == BTRUE)
            phy_hdr_time = MAX_PHY_OVERHEAD_N;
        else
            phy_hdr_time = MAX_PHY_OVERHEAD_A;
    }

    rate_mbps = get_phy_802_11n_pr_to_ur(phy_rate);

    if((rate_mbps != 0) && (txop_limit > phy_hdr_time))
    {
        frame_time = phy_hdr_time + (8 * frame_len)/rate_mbps;

        if(frame_time < txop_limit)
            return 1;
    }

    return 0;
}

/* This function returns the secondary channel index offset based on the     */
/* current frequency band in use                                             */
INLINE UWORD8 get_sec_ch_idx_offset(UWORD8 freq)
{
	UWORD8 val = 0;

    if(freq == RC_START_FREQ_5)
    {
		val = SEC_CHAN_INDEX_OFFSET_START_FREQ_5;
	}
    else if(freq == RC_START_FREQ_2)
    {
		val = SEC_CHAN_INDEX_OFFSET_START_FREQ_2;
	}

	return val;
}

/* This function returns the affected  channel index offset based on the     */
/* current frequency band in use                                             */
INLINE UWORD8 get_affected_ch_idx_offset(UWORD8 freq)
{
	UWORD8 val = 0;

    if(freq == RC_START_FREQ_5)
    {
		val = AFFECTED_CHAN_OFFSET_START_FREQ_5;
	}
    else if(freq == RC_START_FREQ_2)
    {
		val = AFFECTED_CHAN_OFFSET_START_FREQ_2;
	}

	return val;
}

/* This function returns the center frequency table index for the given      */
/* primary and secondary channel table index values.                         */
INLINE UWORD8 get_center_ch_tbl_idx(UWORD8 pri_tbl_idx, UWORD8 sec_tbl_idx)
{
    return ((pri_tbl_idx + sec_tbl_idx) >> 1);
}

/* This function checks if a secondary channel is possible for the given     */
/* Primary channel and Secondary channel offset (with respect to the minimum */
/* and maximum index values - 0 and maximum channels). If possible the       */
/* secondary channel index is calculated by adding or subtracting the        */
/* channel offset value based on the Secondary channel offset.               */
INLINE UWORD8 get_sec_ch_idx_11n(UWORD8 pri_idx, N_SEC_CH_OFFSET_T sec_offset)
{
    UWORD8 sec_idx           = INVALID_CHANNEL;
    UWORD8 freq = get_current_start_freq();
    UWORD8 sec_ch_idx_offset = get_sec_ch_idx_offset(freq);
	UWORD8 max_ch            = get_max_num_channel(freq);

    if(sec_offset == SCA)
    {
		if(pri_idx < (max_ch - sec_ch_idx_offset))
        {
            sec_idx = pri_idx + sec_ch_idx_offset;
        }
    }
    else if(sec_offset == SCB)
    {
		if(pri_idx >= sec_ch_idx_offset)
		{
            sec_idx = pri_idx - sec_ch_idx_offset;
        }
    }

    return sec_idx;
}


#endif /* PHY_802_11n */
