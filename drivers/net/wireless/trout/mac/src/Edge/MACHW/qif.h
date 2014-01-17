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
/*  File Name         : qif.h                                                */
/*                                                                           */
/*  Description       : This file contains the definitions and function      */
/*                      prototypes for MAC hardware queue interface.         */
/*                                                                           */
/*  List of Functions : set_machw_q_ptr                                      */
/*                      is_machw_q_null                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef QIF_H
#define QIF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mh.h"
#include "spi_interface.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD32 addr;
    UWORD32 lock_bit;
} qif_struct_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern qif_struct_t g_qif_table[];

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the queue pointer for the given queue number with the  */
/* given transmit descriptor.                                                */
INLINE void set_machw_q_ptr(UWORD8 q_num, UWORD32 *ptr)
{
    //chenq mod
    //while(rMAC_PA_STAT & g_qif_table[q_num].lock_bit);
    while(host_read_trout_reg((UWORD32)rMAC_PA_STAT) & g_qif_table[q_num].lock_bit);
    
    //chenq mod
    //*((volatile UWORD32 *)(g_qif_table[q_num].addr)) =
                                  //convert_to_le(virt_to_phy_addr((UWORD32)ptr));
    //printk("%s: %p qnum:%d\n",__FUNCTION__,(void *)convert_to_le(virt_to_phy_addr((UWORD32)ptr)),q_num);
    host_write_trout_reg( convert_to_le(virt_to_phy_addr((UWORD32)ptr)), 
					      (UWORD32)(g_qif_table[q_num].addr));
}

/* This function checks if the queue pointer for the given queue number is   */
/* NULL.                                                                     */
INLINE BOOL_T is_machw_q_null(UWORD8 q_num)
{
    UWORD32 q_hdr_val = 0;

    //chenq mod
    //while(rMAC_PA_STAT & g_qif_table[q_num].lock_bit)
    while(host_read_trout_reg((UWORD32)rMAC_PA_STAT)
            & g_qif_table[q_num].lock_bit)
    {
        q_hdr_val = q_hdr_val;
    }

    //chenq mod
    //q_hdr_val = *((volatile UWORD32 *)(g_qif_table[q_num].addr)); 
    q_hdr_val = host_read_trout_reg( (UWORD32)(g_qif_table[q_num].addr));
    
    if(q_hdr_val == 0)
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function returns the current head queue pointer of the given queue */
INLINE UWORD32 get_machw_q_ptr(UWORD8 q_num)
{
    UWORD32 q_hdr_val = 0;

    //chenq mod
    //while(rMAC_PA_STAT & g_qif_table[q_num].lock_bit);
    while(host_read_trout_reg((UWORD32)rMAC_PA_STAT)
            & g_qif_table[q_num].lock_bit);

    //chenq mod
    //q_hdr_val = *((volatile UWORD32 *)(g_qif_table[q_num].addr));
    q_hdr_val = host_read_trout_reg( (UWORD32)(g_qif_table[q_num].addr));

    return convert_to_le(phy_to_virt_addr(q_hdr_val));
}

#endif /* QIF_H */
