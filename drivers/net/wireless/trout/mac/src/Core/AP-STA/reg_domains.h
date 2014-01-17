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

// 20120830 caisf mod, merged ittiam mac v1.3 code

/*****************************************************************************/
/*                                                                           */
/*  File Name         : reg_domains.h                                        */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions for Regulatory Domain support.             */
/*                                                                           */
/*  List of Functions : Get/Set Regulatory Class information                 */
/*                      Get/Set Regulatory Domain/Class configurations       */
/*                      Get Regulatory Domain/Class information              */
/*                      Get/Set TX Power values                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef REG_DOMAINS_H
#define REG_DOMAINS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ch_info.h"
#include "rf_if.h"
#include "cglobals.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define CTRY_CODE_SZ 3

/* Number of 32-bit words required for the Regulatory Class bitmap */
/* Currently Regulatory Class values from 0 to 63 can be supported. This     */
/* needs to be updated in  future if more Regulatory classes are supported. */
#define RC_BMAP_NUM_WORDS 2
#define NUM_SUP_REG_CLASSES 64
/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Behaviour values for Regulatory classes. Note that the structure          */
/* reg_class_info_t assumes this field to be 8-bit. Therefore a maximum of   */
/* 8 values (each bit corresponds to 1 behaviour, multiple behaviours may be */
/* present) can be supported currently. In case this is exceeded the struct  */
/* reg_class_info_t: behaviour_bmap field must be appropriately updated.     */
typedef enum{RC_DFS_BIT          = BIT0,
             RC_TPC_BIT          = BIT1,
             RC_NO_IBSS_BIT      = BIT2,
             RC_SEC_CH_ABOVE_BIT = BIT3,
             RC_SEC_CH_BELOW_BIT = BIT4,
             RC_DFS_50_100_BIT   = BIT5
} RC_BEHAVIOUR_BMAP_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Information stored per Regulatory Class */
typedef struct
{
    UWORD8  freq_spacing;    /* Start frequency value and Channel Spacing   */
    UWORD8  behaviour_bmap;  /* Bitmap of required behavior/features        */
    UWORD8  coverage_class;  /* Coverage class value                        */
    UWORD8  max_reg_tx_pow;  /* Maximum TX power for Regulatory class       */
    UWORD8  min_rf_tx_pow;   /* Minimum RF TX power for 11a/n rates         */
    UWORD8  max_rf_tx_pow_a; /* Maximum RF TX power for 11a rates           */
    UWORD8  max_rf_tx_pow_b; /* Maximum RF TX power for 11b rates           */
    UWORD8  max_rf_tx_pow_n; /* Maximum RF TX power for 11n rates           */
    UWORD32 channel_bmap;    /* Bitmap of supported channels                */
} reg_class_info_t;

/* ------------------------------------------------------------------------- */
/* Format of freq_spacing                                                    */
/* ------------------------------------------------------------------------- */
/* BIT0 : BIT3 - Starting Frequency Value [CH_START_FREQ_T]                  */
/* BIT4 : BIT7 - Channel Spacing Value [CH_SPACING_T]                        */
/* ------------------------------------------------------------------------- */
/* Format of behaviour_bmap                                                  */
/* ------------------------------------------------------------------------- */
/* Each bit corresponds to a particular behaviour [RC_BEHAVIOUR_BMAP_T]      */
/* If the bit is set it indicates that behaviour is required for the         */
/* Regulatory class. Multiple bits may be set indicating all the relevant    */
/* behaviours are required.                                                  */
/* ------------------------------------------------------------------------- */
/* Format of channel_bmap                                                    */
/* ------------------------------------------------------------------------- */
/* Each bit corresponds to a particular channel in the supported channel     */
/* list. If the bit is set it indicates that the channel is included in the  */
/* Regulatory class. Multiple bits shall be set to specify the channel set.  */
/* ------------------------------------------------------------------------- */
#define MAX_NUM_REG_CLASS 20
/* Information stored per Regulatory Domain */
typedef struct
{
    WORD8           country[CTRY_CODE_SZ];    /* Country code                */
    UWORD32 reg_class_bmap[RC_BMAP_NUM_WORDS]; /* Supported regulatory class */
    reg_class_info_t *reg_class_info[MAX_NUM_REG_CLASS];        /* Regulatory class info list */
} reg_domain_info_t;

/* ------------------------------------------------------------------------- */
/* Format of reg_class_bmap                                                  */
/* ------------------------------------------------------------------------- */
/* Each bit corresponds to a particular Regulatory Class. If bit 'X' is set  */
/* it indicates that the Regulatory Class 'X' is supported. Multiple bits    */
/* shall be set to specify all the Regulatory Classes supported.             */
/* Note that the number of Regulatory Class information structures present   */
/* can be derived by extracting the number of 1's present in the Regulatory  */
/* Class bitmap. Also the index for a particular Regulatory Class can be     */
/* derived by getting the number of 1's preceding the corresponding bit in   */
/* the bitmap. For example the Regulatory Class 17 would be at Index 'n' in  */
/* the Regulatory Class info table, where 'n' is the number of 1's between   */
/* BIT0 to BIT16                                                             */
/* Regulatory Class Value             : .... 7   6   5   4   3   2   1   0   */
/* Regulatory Class Bitmap            : .... 1   1   0   1   1   1   0   1   */
/* Regulatory Class Information Index : .... 5   4   x   3   2   1   x   0   */
/* ------------------------------------------------------------------------- */

/* TX Power */
typedef struct
{
    UWORD8 dbm;
    UWORD8 gaincode;
} tx_power_t;

/* Current TX Power information */
typedef struct
{
    tx_power_t pow_a; /* TX power for 11a rates           */
    tx_power_t pow_b; /* TX power for 11b rates           */
    tx_power_t pow_n; /* TX power for 11n rates           */
} curr_tx_power_info_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD32 g_ch_bmap_freq_2;
extern UWORD8* g_sup_reg_dom_info;
extern UWORD32 g_ch_bmap_freq_5;
extern UWORD8 g_current_reg_domain;
extern UWORD8 g_current_reg_class;
//caisf add 2012-12-27
extern UWORD8 g_scan_req_channel_no;

extern UWORD8 g_new_reg_class;
extern reg_domain_info_t g_rd_info[];
extern curr_tx_power_info_t g_default_tx_power;
extern curr_tx_power_info_t g_user_tx_power;
extern curr_tx_power_info_t g_reg_max_tx_power;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_supp_reg_domain(void);
extern void update_curr_rc_max_tx_pow(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Functions to get/set Regulatory Class information                         */
/*****************************************************************************/

INLINE UWORD8 get_rc_start_freq(reg_class_info_t *rc_info)
{
    return ((rc_info->freq_spacing) & 0x0F); /* BIT0 : BIT3 */
}

INLINE UWORD8 get_rc_channel_spac(reg_class_info_t *rc_info)
{
    return (((rc_info->freq_spacing) & 0xF0) >> 4); /* BIT4 : BIT7 */
}

INLINE UWORD8 get_rc_max_reg_tx_pow(reg_class_info_t *rc_info)
{
    return rc_info->max_reg_tx_pow;
}

INLINE UWORD8 get_rc_min_rf_tx_pow(reg_class_info_t *rc_info)
{
    return rc_info->min_rf_tx_pow;
}

INLINE UWORD8 get_rc_max_rf_tx_pow_a(reg_class_info_t *rc_info)
{
    return rc_info->max_rf_tx_pow_a;
}

INLINE UWORD8 get_rc_max_rf_tx_pow_b(reg_class_info_t *rc_info)
{
    return rc_info->max_rf_tx_pow_b;
}

INLINE UWORD8 get_rc_max_rf_tx_pow_n(reg_class_info_t *rc_info)
{
    return rc_info->max_rf_tx_pow_n;
}


INLINE BOOL_T is_rc_dfs_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_DFS_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE BOOL_T is_rc_tpc_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_TPC_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE BOOL_T is_rc_no_ibss_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_NO_IBSS_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE BOOL_T is_rc_sca_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_SEC_CH_ABOVE_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE BOOL_T is_rc_scb_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_SEC_CH_BELOW_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE BOOL_T is_rc_dfs_50_100us_req(reg_class_info_t *rc_info)
{
    if(((rc_info->behaviour_bmap) & RC_DFS_50_100_BIT) == 0)
         return BFALSE;

    return BTRUE;
}

INLINE void set_rc_start_freq(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->freq_spacing &= ~(0x0F);
    rc_info->freq_spacing |= (val & 0x0F);
}

INLINE void set_rc_ch_spacing(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->freq_spacing &= ~(0xF0);
    rc_info->freq_spacing |= ((val << 4) & 0xF0);
}

INLINE void set_rc_behaviour_bmap(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->behaviour_bmap = val;
}

INLINE void set_rc_channel_bmap(reg_class_info_t *rc_info, UWORD32 val)
{
    rc_info->channel_bmap = val;
}

INLINE void set_rc_coverage_class(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->coverage_class = val;
}

INLINE void set_rc_max_reg_tx_pow(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->max_reg_tx_pow = val;
}

INLINE void set_rc_min_rf_tx_pow(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->min_rf_tx_pow = val;
}

INLINE void set_rc_max_rf_tx_pow_a(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->max_rf_tx_pow_a = val;
}

INLINE void set_rc_max_rf_tx_pow_b(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->max_rf_tx_pow_b = val;
}

INLINE void set_rc_max_rf_tx_pow_n(reg_class_info_t *rc_info, UWORD8 val)
{
    rc_info->max_rf_tx_pow_n = val;
}


/*****************************************************************************/
/* Functions to get/set current Regulatory Domain/Class configurations       */
/*****************************************************************************/

/* This function gets the list of supported Regulatory Domains by RF        */
INLINE UWORD8* get_sup_reg_dom_info(void)
{
    return g_sup_reg_dom_info;
}

/* This function sets the Regulatory Domain to use and the channel list is   */
/* updated as required.                                                      */
INLINE void set_current_reg_domain(UWORD8 reg_domain)
{
    g_current_reg_domain = reg_domain;

    update_ch_list_reg_domain();

    if(g_current_reg_domain == NO_REG_DOMAIN)
    {
		g_current_reg_class  = UNKNOWN_REG_CLASS;
	}
}

/* This function gets the current Regulatory Domain in use */
INLINE UWORD8 get_current_reg_domain(void)
{
    return g_current_reg_domain;
}

/* This function sets the Regulatory Class to use and the channel list is    */
/* updated as required                                                       */
INLINE void set_current_reg_class(UWORD8 reg_class)
{
    g_current_reg_class = reg_class;

    if((reg_class != UNKNOWN_REG_CLASS) &&
       (reg_class != INVALID_REG_CLASS))
    {
    	update_ch_list_reg_class(reg_class);
    	update_curr_rc_max_tx_pow();
	}
}

/* This function returns Regulatory Domain Index for the given Country String  */
/* If Country string is not found, it returns Invalid Regulatory Domain        */
INLINE UWORD8 get_reg_domain_idx(UWORD8* cnt_str)
{
    UWORD8 cnt  = 0;
    UWORD8 idx  = INVALID_REG_DOMAIN;
    UWORD8 *country_str = NULL;

    for(cnt = 0; cnt < NUM_REG_DOMAINS; cnt++)
    {
        country_str = g_rd_info[cnt].country;

        if(0 == strcmp(country_str, cnt_str))
        {
            idx = cnt;
            break;
        }
    }

    return idx;
}

/* This function gets the current Regulatory Class in use */
INLINE UWORD8 get_current_reg_class(void)
{
    return g_current_reg_class;
}

/* This function sets the scan request channel nubmer in use */
INLINE UWORD8 set_scan_req_channel_no(UWORD8 ch_no)
{
	TROUT_DBG4("domain: set scan channel NO. from %d to %d\n", g_scan_req_channel_no, ch_no);
    return g_scan_req_channel_no = ch_no;
}

/* This function gets the scan request channel nubmer in use */
INLINE UWORD8 get_scan_req_channel_no(void)
{
    return g_scan_req_channel_no;
}

/*****************************************************************************/
/* Functions to get Regulatory Domain and Regulatory Class information       */
/*****************************************************************************/

/* This function checks if the Regulatory Domain is valid */
INLINE BOOL_T is_reg_domain_valid(UWORD8 reg_domain)
{
    if(reg_domain < NUM_REG_DOMAINS)
        return BTRUE;

    return BFALSE;
}

/* This function returns the given Regulatory Domain information */
INLINE reg_domain_info_t *get_reg_domain_info(UWORD8 reg_domain)
{
    reg_domain_info_t *rd_info = NULL;

    if(is_reg_domain_valid(reg_domain) == BTRUE)
    {
        rd_info = &g_rd_info[reg_domain];
    }

    return rd_info;
}

/* This function returns the number of Regulatory Classes supported in given */
/* Regulatory Domain                                                         */
INLINE UWORD8 get_num_reg_class(reg_domain_info_t *rd_info)
{
    UWORD8 num_rc = 0;
    UWORD8 i      = 0;

    for(i = 0; i < RC_BMAP_NUM_WORDS; i++)
    {
        num_rc += get_num_bits_set(rd_info->reg_class_bmap[i]);
    }

    return num_rc;
}

/* This function gets the Regulatory Class info for the given Regulatory     */
/* Class in the given Regulatory Domain                                      */
INLINE reg_class_info_t *get_reg_class_info(UWORD8 freq, UWORD8 rc, UWORD8 reg_domain)
{
    UWORD8 rc_info_idx  = 0;
    UWORD8 rc_word      = 0;
    UWORD32 rc_bit      = 0;
    UWORD32 rc_bmap     = 0;
    reg_domain_info_t  *rd_info = NULL;

    rd_info = get_reg_domain_info(reg_domain);

    /* In the current implementation, 64 Regulatory Classes are supported */
    if(rc > NUM_SUP_REG_CLASSES)
    {
        return NULL;
    }

    /* Get the word offset and bit offset for the given Regulatory Class in  */
    /* the given Regulatory Domain reg_class_bmap                            */
    rc_word = (rc) >> 5;
    rc_bit  = (rc & 0x1F);
    rc_bmap = rd_info->reg_class_bmap[rc_word];

    /* If the Regulatory Class is not supported in the given Regulatory Domain */
    /* return NULL.                                                          */
    if((rc_bmap & (1 << rc_bit)) == 0)
    {
        return NULL;
    }

    /* Find the number of 1s preceding the Regulatory Class bit position to  */
    /* determine the index of the Regulatory Class information maintained in */
    /* the Regulatory Domain structure                                       */
    while(rc_word > 0)
    {
        rc_word--;
        rc_info_idx += get_num_bits_set(rd_info->reg_class_bmap[rc_word]);
    }

    rc_info_idx += get_num_preceding_bits_set(rc_bmap, rc_bit);

    return (rd_info->reg_class_info[rc_info_idx]);
}

/* This function updates regulatory max Tx power */
INLINE void update_rc_max_tx_pow(UWORD8 reg_class, UWORD8 max_tx_pow)
{
    reg_class_info_t *rc_info = NULL;

    rc_info = get_reg_class_info(get_current_start_freq(), reg_class,
                                 get_current_reg_domain());
    if(rc_info == NULL)
    {
        PRINTD("Warning:MultiDomain:Regulatory Class Info is not found\n");
        return;
    }

    set_rc_max_reg_tx_pow(rc_info, max_tx_pow);
}

/* This function updates Coverage class for the given regulatory class */
INLINE void update_rc_coverage_class(UWORD8 reg_class, UWORD8 coverage_class)
{
    reg_class_info_t *rc_info = NULL;

    rc_info = get_reg_class_info(get_current_start_freq(), reg_class,
                                 get_current_reg_domain());
    if(rc_info == NULL)
    {
        PRINTD("Warning:MultiDomain:Regulatory Class Info is not found\n");
        return;
    }

    set_rc_coverage_class(rc_info, coverage_class);
}

/* This function returns BTRUE if current Regulatory Class has behaviour     */
/* limit set 16, in this case HT Green field transmission will not be done   */
INLINE BOOL_T check_curr_rc_dfs_50_100us(UWORD8 curr_ch)
{
    UWORD8 reg_class = 0;
    UWORD8 freq      = get_current_start_freq();
    reg_class_info_t *rc_info = NULL;

    reg_class = get_reg_class_from_ch_idx(freq, curr_ch);
    rc_info   = get_reg_class_info(freq, reg_class, get_current_reg_domain());

    if(rc_info == NULL)
    {
        PRINTD("Warning:check_rc_dfs50us:Regulatory Class Info is not found\n");
        return BFALSE;
    }

    return is_rc_dfs_50_100us_req(rc_info);
}

/* This function checks if the given Channel Index is supported in the given */
/* Regulatory Class.                                                         */
INLINE BOOL_T is_ch_supp_in_rc(reg_class_info_t *rc_info, UWORD8 freq,
                               UWORD8 ch_idx)
{
    UWORD32 ch_bit  = 0;
    UWORD32 ch_bmap = 0;

    /* If the frequency band does not match, no check for channel support is */
    /* required to be done.                                                  */
    if(get_rc_start_freq(rc_info) != freq)
    return BFALSE;

    if((rc_info != NULL) && (ch_idx != INVALID_CHANNEL))
    {
        ch_bit  = GET_CH_BIT(ch_idx);
        ch_bmap = rc_info->channel_bmap;

        if((ch_bmap & ch_bit) != 0)
            return BTRUE;
    }

    return BFALSE;
}

/* This function checks if the given Channel Index is supported in the given */
/* Regulatory Class of the given Regulatory Domain                           */
INLINE BOOL_T is_ch_supp_in_rc_in_rd(UWORD8 ch_num, UWORD8 reg_domain,
                                     UWORD8 reg_class)
{
    UWORD8 freq    = get_current_start_freq();
    UWORD8 ch_idx  = get_ch_idx_from_num(freq, ch_num);
    BOOL_T ret_val = BFALSE;
    reg_class_info_t  *rc_info = NULL;

    if(BTRUE == is_reg_domain_valid(reg_domain))
    {
        rc_info  = get_reg_class_info(freq, reg_class, reg_domain);

        if(rc_info != NULL)
        {
            ret_val = is_ch_supp_in_rc(rc_info, freq, ch_idx);
        }
    }

    return ret_val;
}

/* This function gets the Regulatory Class value for the given Regulatory    */
/* Class information index in the given Regulatory Domain                    */
INLINE UWORD8 get_reg_class_val(UWORD8 rc_info_idx, UWORD8 reg_domain)
{
    UWORD8  rc_val       = 0;
    UWORD8 rc_word       = 0;
    UWORD8  num_bits_set = 0;
    UWORD32 rc_bit       = 0;
    UWORD32 rc_bmap      = 0;
    reg_domain_info_t *rd_info = NULL;

    rd_info = get_reg_domain_info(reg_domain);

    /* The Regulatory Class bitmap consists of multiple words. Find the word */
    /* to be checked for the given regulatory class information index.       */
    for(rc_word = 0; rc_word < RC_BMAP_NUM_WORDS; rc_word++)
    {
        rc_bmap = rd_info->reg_class_bmap[rc_word];
        num_bits_set = get_num_bits_set(rc_bmap);

        if(num_bits_set > rc_info_idx)
        {
            break;
        }

        rc_info_idx -= num_bits_set;
    }

    rc_bit = get_bit_position(rc_bmap, rc_info_idx);

    /* Exception case: RC Info Index not found. Should never occur. */
    if(is_bit_position_valid(rc_bit) == BFALSE)
        return INVALID_REG_CLASS;

    rc_val = (rc_word * MAX_BMAP_SZ) + rc_bit;

    return rc_val;
}

/*****************************************************************************/
/* Functions to get/set current TX power values                              */
/*****************************************************************************/

INLINE void set_curr_tx_power_dbm_11a(UWORD8 freq, UWORD8 ch_idx, UWORD8 dbm)
{
    UWORD8 gaincode  = 0;

    /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_user_tx_power.pow_a.dbm      = dbm;
    g_user_tx_power.pow_a.gaincode = gaincode;
}

INLINE UWORD8 get_curr_tx_power_dbm_11a(void)
{
    return (UWORD8)(g_user_tx_power.pow_a.dbm);
}

INLINE void set_curr_tx_power_dbm_11b(UWORD8 freq, UWORD8 ch_idx, UWORD8 dbm)
{
    UWORD8 gaincode  = 0;

    /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_user_tx_power.pow_b.dbm      = dbm;
    g_user_tx_power.pow_b.gaincode = gaincode;
}

INLINE UWORD8 get_curr_tx_power_dbm_11b(void)
{
    return (UWORD8)(g_user_tx_power.pow_b.dbm);
}

INLINE void set_curr_tx_power_dbm_11n(UWORD8 freq, UWORD8 ch_idx, UWORD8 dbm)
{
    UWORD8 gaincode  = 0;

    /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, ch_idx);

    /* Set the current TX power dBm and Gain code values */
    g_user_tx_power.pow_n.dbm      = dbm;
    g_user_tx_power.pow_n.gaincode = gaincode;
}

INLINE UWORD8 get_curr_tx_power_dbm_11n(void)
{
	    return (UWORD8)(g_user_tx_power.pow_n.dbm);
}


/* This function compares dbm power levels and returns the pointer */
/* which has lesser power level                                    */
INLINE tx_power_t* min_of_tx_pw (tx_power_t* usr, tx_power_t* reg)
{
    tx_power_t *min_pow = NULL;

    if(usr->dbm > reg->dbm)
        min_pow = reg;
    else
        min_pow = usr;

    return min_pow;
}

/* This function returns various power levels in a particular format.If user */
/* control is enabled, it returns minimum of g_user_tx_power and             */
/* g_reg_max_tx_power in the below mentioned format, if user control is      */
/* disabled it returns minimum of g_default_tx_power and g_reg_max_tx_power  */
/* in the below format                                                       */
/* -------------------------------------------------------------- */
/* |   pow_n40    |     pow_n     |    pow_b      |  pow_a      | */
/* -------------------------------------------------------------- */
/* |    Byte-3    |     Byte-2    |    Byte-1     |  Byte-0     | */
/* -------------------------------------------------------------- */

INLINE UWORD32 get_curr_tx_power_levels_dbm(UWORD8 usr_cntrl)
{
    UWORD32 pwr_levels   = 0;
    tx_power_t *pow      = NULL;

    if(usr_cntrl == 0)
    {

        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_default_tx_power.pow_n,
                           &g_reg_max_tx_power.pow_n);

        pwr_levels |= pow->dbm;
        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_default_tx_power.pow_b,
                           &g_reg_max_tx_power.pow_b);

        pwr_levels |= pow->dbm;
        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_default_tx_power.pow_a,
                           &g_reg_max_tx_power.pow_a);

        pwr_levels |= pow->dbm;
    }
    else
    {

        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_user_tx_power.pow_n,
                           &g_reg_max_tx_power.pow_n);

        pwr_levels |= pow->dbm;
        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_user_tx_power.pow_b,
                           &g_reg_max_tx_power.pow_b);

        pwr_levels |= pow->dbm;
        pwr_levels = pwr_levels << 8;

        pow = min_of_tx_pw(&g_user_tx_power.pow_a,
                           &g_reg_max_tx_power.pow_a);

        pwr_levels |= pow->dbm;
    }

    return pwr_levels;
}

/* This function is used to get current Tx power to update in TPC report */
INLINE UWORD8 get_tpc_report_tx_pow(UWORD8 usr_cntrl)
{
    tx_power_t *pow       = NULL;

    if(usr_cntrl == 0)
    {
        pow = min_of_tx_pw(&g_default_tx_power.pow_a,
                           &g_reg_max_tx_power.pow_a);
    }
    else
	{
        pow = min_of_tx_pw(&g_user_tx_power.pow_a,
                           &g_reg_max_tx_power.pow_a);
    }

    return pow->dbm;
}

#endif /* REG_DOMAINS_H */
