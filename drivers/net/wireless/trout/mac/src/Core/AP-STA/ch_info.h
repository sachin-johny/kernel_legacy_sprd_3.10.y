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
/*  File Name         : ch_info.h                                            */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions for Channel information.                   */
/*                                                                           */
/*  List of Functions : get_current_start_freq                               */
/*                      set_current_start_freq                               */
/*                      get_num_supp_channel                                 */
/*                      get_max_num_channel                                  */
/*                      get_ch_info_from_freq                                */
/*                      get_ch_num_from_idx                                  */
/*                      get_tbl_idx_from_ch_idx                              */
/*                      get_reg_class_from_ch_idx                            */
/*                      get_ch_idx_from_num                                  */
/*                      get_reg_class_from_ch_num                            */
/*                      get_ch_idx_from_tbl_idx                              */
/*                      is_ch_tbl_idx_valid                                  */
/*                      is_ch_valid                                          */
/*                      is_ch_idx_supported_by_rf                            */
/*                      is_ch_idx_supported                                  */
/*                      is_ch_idx_info_available                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CH_INFO_H
#define CH_INFO_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "rf_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NO_REG_DOMAIN     0   /* Regulatory Domain Index 0 is undefined      */
#define INVALID_REG_DOMAIN 255 /* Regulatory domain not supported by RF  */ // 20120830 caisf add, merged ittiam mac v1.3 code
#define INVALID_CHANNEL   255 /* Channel Number or Index 255 is undefined    */

/* Maximum Number of Supported Channels is the Invalid Channel table index */
#define INVALID_CH_TBL_INDEX MAX_SUPP_CHANNEL

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define GET_CH_BIT(val) (1 << val)
#define IS_CH_SUPP(bmap, val) ((GET_CH_BIT(val) & bmap) ? BTRUE : BFALSE)

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Special Regulatory Class values to indicate channel information.          */
/* NOT_SUPPORTED_BY_RF - Indicates the channel is not supported by device    */
/* INVALID_REG_CLASS   - Indicates the channel is supported by device but    */
/*                       not in the current Regulatory Domain                */
/* UNKNOWN_REG_CLASS   - Indicates the channel is supported by device but no */
/*                       Regulatory Class information is available for it    */
/* Note that current unused Regulatory Class values (0, 254, 255) are used   */
/* for special values. If these become valid this enum needs to be redefined */
typedef enum{NOT_SUPPORTED_BY_RF = 0,
             INVALID_REG_CLASS   = 254,
             UNKNOWN_REG_CLASS   = 255
} CH_REG_CLASS_T;

/* Starting Frequency values for Regulatory classes. Note that the structure */
/* reg_class_info_t assumes this field to be 4-bit. Therefore a maximum of   */
/* 16 values can be supported currently. In case this is exceeded the struct */
/* reg_class_info_t: freq_spacing field must be appropriately updated.       */
typedef enum{RC_START_FREQ_2 = 0, /* 2.407  */
             RC_START_FREQ_5,     /* 5      */
             RC_START_FREQ_5a,    /* 5.0025 */
             RC_START_FREQ_3,     /* 3      */
             RC_START_FREQ_3a,    /* 3.0025 */
             RC_START_FREQ_4a,    /* 4.85   */
             RC_START_FREQ_4b,    /* 4.89   */
             RC_START_FREQ_4c     /* 4.9375 */
} CH_START_FREQ_T;

/* Channel Spacing values for Regulatory classes. Note that the structure    */
/* reg_class_info_t assumes this field to be 4-bit. Therefore a maximum of   */
/* 16 values can be supported currently. In case this is exceeded the struct */
/* reg_class_info_t: freq_spacing field must be appropriately updated.       */
typedef enum{RC_CH_SPACING_5MHZ = 0,
             RC_CH_SPACING_10MHZ,
             RC_CH_SPACING_20MHZ,
             RC_CH_SPACING_25MHZ,
             RC_CH_SPACING_40MHZ
} CH_SPACING_T;

// 20120830 caisf add, merged ittiam mac v1.3 code
/* Channel increment values for frequency bands 2.4GHz and 5GHz      */
/* For 2.4GHz band channels 1,2,3,4.... Channel increment value is 1 */
/* For 5GHz band channels 36,40,44,48...Channel increment value is 4 */
typedef enum{CH_INCR_VAL_FREQ_2 = 1,
             CH_INCR_VAL_FREQ_5 = 4,
}CH_INCR_VAL_T;

/* Channel index values for 2.4GHz band (RC_START_FREQ_2) */
typedef enum{CHANNEL1 = 0,
             CHANNEL2,
             CHANNEL3,
             CHANNEL4,
             CHANNEL5,
             CHANNEL6,
             CHANNEL7,
             CHANNEL8,
             CHANNEL9,
             CHANNEL10,
             CHANNEL11,
             CHANNEL12,
             CHANNEL13,
             CHANNEL14,
             MAX_CHANNEL_FREQ_2
} CH_NUM_FREQ_2_T;

/* Channel index values for 5GHz band (RC_START_FREQ_5) */
typedef enum{CHANNEL36 = 0,
             CHANNEL40,
             CHANNEL44,
             CHANNEL48,
             CHANNEL52,
             CHANNEL56,
             CHANNEL60,
             CHANNEL64,
             CHANNEL100,
             CHANNEL104,
             CHANNEL108,
             CHANNEL112,
             CHANNEL116,
             CHANNEL120,
             CHANNEL124,
             CHANNEL128,
             CHANNEL132,
             CHANNEL136,
             CHANNEL140,
             CHANNEL149,
             CHANNEL153,
             CHANNEL157,
             CHANNEL161,
             CHANNEL165,
             MAX_CHANNEL_FREQ_5
} CH_NUM_FREQ_5_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Information stored per Channel */
typedef struct
{
    UWORD8  ch_num;      /* Channel number value               */
    UWORD8  ch_tbl_idx;  /* Index for supported channel tables */
    UWORD8  reg_class;   /* Corresponding Regulatory Class     */
} channel_info_t;

/* ------------------------------------------------------------------------- */
/* Format of reg_class                                                       */
/* ------------------------------------------------------------------------- */
/* UWORD8 with value denoting the particular Regulatory Class value.         */
/* The special values used are as per CH_REG_CLASS_T                         */
/* ------------------------------------------------------------------------- */
/* Format of ch_tbl_idx                                                      */
/* ------------------------------------------------------------------------- */
/* Separate lists for various functions shall be maintained, which would     */
/* include only the channels supported. This value contains the index into   */
/* the supported channel list from the master channel information table.     */
/* The supported channel list would be RF dependant while the master channel */
/* list would be constant. For channels not supported this would contain an  */
/* invalid value.                                                            */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_current_start_freq;

extern channel_info_t g_ch_info_freq_2[MAX_CHANNEL_FREQ_2];
extern channel_info_t g_ch_info_freq_5[MAX_CHANNEL_FREQ_5];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_supp_ch_list(void);
extern void update_ch_list_reg_domain(void);
// 20120830 caisf add, merged ittiam mac v1.3 code
extern void update_ch_list_reg_class(UWORD8 reg_class);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Functions to get/set current Starting Frequency configurations            */
/*****************************************************************************/

INLINE UWORD8 get_current_start_freq(void)
{
    return g_current_start_freq;
}

INLINE void set_current_start_freq(UWORD8 start_freq)
{
    g_current_start_freq = start_freq;
}

/*****************************************************************************/
/* Functions to get information based on the Starting Frequency              */
/*****************************************************************************/

/* This function returns the number of supported channels for given Starting */
/* Frequency                                                                 */
INLINE UWORD8 get_num_supp_channel(UWORD8 freq)
{
    UWORD8 num_ch = 0;

    switch(freq)
    {
    case RC_START_FREQ_2:
    {
        num_ch = NUM_SUPP_CHANNEL_FREQ_2;
    }
    break;
    case RC_START_FREQ_5:
    {
        num_ch = NUM_SUPP_CHANNEL_FREQ_5;
    }
    break;
    default:
    {
        num_ch = 0;
    }
    break;
    }

    return num_ch;
}

/* This function returns the maximum number of channels for a given Starting */
/* Frequency                                                                 */
INLINE UWORD8 get_max_num_channel(UWORD8 freq)
{
    UWORD8 num_ch = 0;

    switch(freq)
    {
    case RC_START_FREQ_2:
    {
        num_ch = MAX_CHANNEL_FREQ_2;
    }
    break;
    case RC_START_FREQ_5:
    {
        num_ch = MAX_CHANNEL_FREQ_5;
    }
    break;
    default:
    {
        num_ch = 0;
    }
    break;
    }

    return num_ch;
}

/* This function returns the Channel information for the given Starting      */
/* Frequency. It also updates the number of channels for the frequency in    */
/* the variable (num_ch) passed to it as an input.                           */
INLINE channel_info_t *get_ch_info_from_freq(UWORD8 freq, UWORD8 *num_ch)
{
    channel_info_t *ch_info = NULL;

    switch(freq)
    {
    case RC_START_FREQ_2:
    {
        ch_info = g_ch_info_freq_2;
        (*num_ch) = MAX_CHANNEL_FREQ_2;
    }
    break;
    case RC_START_FREQ_5:
    {
        ch_info = g_ch_info_freq_5;
        (*num_ch) = MAX_CHANNEL_FREQ_5;
    }
    break;
    default:
    {
        ch_info = NULL;
        (*num_ch) = 0;
    }
    break;
    }

    return ch_info;
}

/*****************************************************************************/
/* Functions to get Channel information parameters for the given Starting    */
/* Frequency and Channel Index value                                         */
/*****************************************************************************/

INLINE UWORD8 get_ch_num_from_idx(UWORD8 freq, UWORD8 ch_idx)
{
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: Invalid Ch Num for ch_idx: %d of freq:%d\n",ch_idx, freq);
        return INVALID_CHANNEL;
	}

    return (ch_info[ch_idx].ch_num);
}

INLINE UWORD8 get_tbl_idx_from_ch_idx(UWORD8 freq, UWORD8 ch_idx)
{
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: Invalid Tble Idx for ch_idx: %d of freq:%d\n",ch_idx, freq);
        return INVALID_CH_TBL_INDEX;
	}

    return (ch_info[ch_idx].ch_tbl_idx);
}

INLINE UWORD8 get_reg_class_from_ch_idx(UWORD8 freq, UWORD8 ch_idx)
{
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: Invalid Regulatory class for ch_idx: %d of freq:%d\n",ch_idx, freq);
        return INVALID_REG_CLASS;
	}

    return (ch_info[ch_idx].reg_class);
}

/*****************************************************************************/
/* Functions to get Channel information parameters for the given Starting    */
/* Frequency and Channel Number value                                         */
/*****************************************************************************/

INLINE UWORD8 get_ch_idx_from_num(UWORD8 freq, UWORD8 ch_num)
{
    UWORD8         i        = 0;
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: Invalid ch_idx for ch_num: %d of freq:%d\n",ch_num, freq);
        return INVALID_CHANNEL;
	}

    for(i = 0; i < num_ch; i++)
    {
        if(ch_info[i].ch_num == ch_num)
            return i;
    }

    return INVALID_CHANNEL;
}

INLINE UWORD8 get_reg_class_from_ch_num(UWORD8 freq, UWORD8 ch_num)
{
    UWORD8         i        = 0;
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: No Valid Reg class for ch_num: %d of freq:%d\n",ch_num, freq);
        return 0;
	}

    for(i = 0; i < num_ch; i++)
    {
        if(ch_info[i].ch_num == ch_num)
            return (ch_info[i].reg_class);
    }

    return 0;
}

/*****************************************************************************/
/* Functions to get Channel information parameters for the given Starting    */
/* Frequency and Channel Table Index value                                   */
/*****************************************************************************/

INLINE UWORD8 get_ch_idx_from_tbl_idx(UWORD8 freq, UWORD8 tbl_idx)
{
    UWORD8         i        = 0;
    UWORD8         num_ch   = 0;
    channel_info_t *ch_info = get_ch_info_from_freq(freq, &num_ch);

    if(NULL == ch_info)
    {
		PRINTD("Warning: Invalid ch_idx for tbl_idx: %d of freq:%d\n",tbl_idx, freq);
        return INVALID_CH_TBL_INDEX;
	}

    for(i = 0; i < num_ch; i++)
    {
        if(ch_info[i].ch_tbl_idx == tbl_idx)
            return i;
    }

    return INVALID_CH_TBL_INDEX;
}

/*****************************************************************************/
/* Functions for status and validity of channels                             */
/*****************************************************************************/

/* This function checks if a given Channel Table Index is valid */
INLINE BOOL_T is_ch_tbl_idx_valid(UWORD8 freq, UWORD8 tbl_idx)
{
    if(INVALID_CH_TBL_INDEX == tbl_idx)
        return BFALSE;

    return BTRUE;
}

/* This function checks if a given Channel Index or Number is valid. Note    */
/* that the same invalid value (INVALID_CHANNEL) is used for both Channel    */
/* Index and Channel Number parameters.                                      */
INLINE BOOL_T is_ch_valid(UWORD8 ch_idx_num)
{
    if(INVALID_CHANNEL == ch_idx_num)
        return BFALSE;

    return BTRUE;
}

/* This function checks if a given Channel Index is supported by the RF      */
/* (independent of any Regulatory Domain configuration).                     */
INLINE BOOL_T is_ch_idx_supported_by_rf(UWORD8 freq, UWORD8 ch_idx)
{
	UWORD8 reg_class = get_reg_class_from_ch_idx(freq, ch_idx);

    if(reg_class == NOT_SUPPORTED_BY_RF)
        return BFALSE;

    return BTRUE;
}

/* This function checks if a given Channel Index is supported as per the     */
/* current Regulatory Domain configured (it implies that the Channel index   */
/* is supported by the RF also)                                              */
INLINE BOOL_T is_ch_idx_supported(UWORD8 freq, UWORD8 ch_idx)
{
	UWORD8 reg_class = get_reg_class_from_ch_idx(freq, ch_idx);

    if((reg_class == INVALID_REG_CLASS) || (reg_class == NOT_SUPPORTED_BY_RF))
        return BFALSE;

    return BTRUE;
}

/* This function checks if required information regarding a given Channel    */
/* Index is available to allow transmission as per the current Regulatory    */
/* Domain configured                                                         */
INLINE BOOL_T is_ch_idx_info_available(UWORD8 freq, UWORD8 ch_idx)
{
    if(get_reg_class_from_ch_idx(freq, ch_idx) == UNKNOWN_REG_CLASS)
        return BFALSE;

    return BTRUE;
}

// 20120830 caisf add, merged ittiam mac v1.3 code
/* This function is used to get increment value to get next channel in the   */
/* specified frequency band                                                  */
INLINE UWORD8 get_ch_incr_val(UWORD8 freq)
{
    UWORD8 incr_val = 0;

    if(freq == RC_START_FREQ_2)
    {
        incr_val = CH_INCR_VAL_FREQ_2;
    }
    else if(freq == RC_START_FREQ_5)
    {
        incr_val = CH_INCR_VAL_FREQ_5;
    }
    else
    {
        PRINTD("WARNING:ch_info:Frequency is not 2.4GHz or 5GHz \n");
    }

    return incr_val;

}

#endif /* CH_INFO_H */
