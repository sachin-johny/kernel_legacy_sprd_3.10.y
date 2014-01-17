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
/*  File Name         : index_util.h                                         */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC CE LUT interface       */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef INDEX_UTIL_H
#define INDEX_UTIL_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "management.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* Maximum number of hash values possible for the Sta Entry Table */
#define MAX_HASH_VALUES      16
#define MAX_ELEMENTS         MAX_STA_SUPPORTED  /* Max no of table elements  */
#define INVALID_ID           (0)
#define ASOC_ID_BMAP_LEN     ((MAX_ELEMENTS >> 3) + 1)

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Structure used to search the Sta indicies */
typedef struct
{
    UWORD8 sta_index;
} table_entry_t;

typedef struct table_elmnt_t
{
    struct table_elmnt_t* next_sta_entry;  /* Pointer to the next sta entry  */
    UWORD8                key[6];          /* Key, i.e. the MAC address      */
    void*                 element;         /* Pointer to the table element   */
    struct table_elmnt_t* next_hash_elmnt; /* Pointer to next bucket element */
} table_elmnt_t;

/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

typedef table_elmnt_t* table_t[MAX_HASH_VALUES];

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern table_t g_sta_table;
extern list_buff_t g_sta_entry_list;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD32 hash(UWORD8* addr);
extern UWORD8  get_new_sta_index(UWORD8 *addr);
extern void    del_sta_index    (void * val);
extern void    init_sta_table(void);
extern void    *find_entry(UWORD8* key);
extern void    add_entry(void* entry, UWORD8* key);
extern void    delete_entry(UWORD8* key);
extern UWORD8  alloc_sta_index(void);

#ifdef TROUT_TRACE_DBG
extern void print_all_entry(void);
#endif	/* TROUT_TRACE_DBG */
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function initializes the given LUT bitmap table */
INLINE void init_lut(UWORD8 *lut_index_table, UWORD8 bmap_len)
{
    UWORD8 i = 0;

    for(i = 0; i < bmap_len; i++)
        lut_index_table[i] = 0;
}

/* This function returns an unused index from the required LUT index bitmap  */
/* table. If full it returns an invalid index.                               */
INLINE UWORD8 get_lut_index(UWORD8 *lut_index_table, UWORD8 bmap_len,
                            UWORD16 max_lut_size)
{
    UWORD8 byte = 0;
    UWORD8 bit  = 0;
    UWORD8 temp = 0;
    UWORD16 index = 0;

    for(byte = 0; byte < bmap_len; byte++)
    {
        temp = lut_index_table[byte];

        for(bit = 0; bit < 8; bit++)
        {
            if((temp & (1 << bit)) == 0x0)
            {
                index = (byte * 8 + bit);

                if(index < max_lut_size)
                {
                    lut_index_table[byte] |= (1 << bit);
                    return index;
                }
                else
                {
                    return max_lut_size;
                }
            }
        }
    }

    return max_lut_size;
}

/* This function deletes a LUT index by resetting the bit in the required    */
/* LUT index bitmap.                                                         */
INLINE void del_lut_index(UWORD8 *lut_index_table, UWORD8 idx)
{
    UWORD8 byte = idx/8;
    UWORD8 bit  = (1 << idx % 8);

    lut_index_table[byte] &= ~bit;
}

#endif /* INDEX_UTIL_H */
