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
/*  File Name         : p2p_ps.h                                             */
/*                                                                           */
/*  Description       : This file conatins the power save management         */
/*                      functions for P2P protocol                           */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_P2P

#ifndef P2P_PS_H
#define P2P_PS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mgmt_p2p.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/
#define P2P_NOA_SCHED_LEN 13

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum
{
    MISC_ABSENT_PERIOD_START = 0x80,
    MISC_ABSENT_PERIOD_END   = 0x81
} EVENT_TYPESUBTYPE_P2P_T ;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

#ifdef MAC_P2P_HW_SUPP
extern void p2p_absent_period_isr(EVENT_TYPESUBTYPE_P2P_T type);
#else /* MAC_P2P_HW_SUPP */
extern void post_absent_period_event(EVENT_TYPESUBTYPE_P2P_T type);
#endif /* MAC_P2P_HW_SUPP */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function gets the P2P NOA schedule in AP and STA mode */
INLINE UWORD8* get_p2p_noa_sched(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_P2P
    UWORD8  i   = 0;
    UWORD32 val = 0;
    /* --------------------------------------------------------------------- */
    /* Format of NoA schedule message                                        */
    /* --------------------------------------------------------------------- */
    /* | Length | NumNoA | NOA1-Cnt | NOA1-Dur | NOA1-Int | NOA1-StartTime | */
    /* --------------------------------------------------------------------- */
    /* | 1      | 1      | 1        | 4        | 4        | 4              | */
    /* --------------------------------------------------------------------- */
    /* | Present if      | NOA2-Cnt | NOA2-Dur | NOA2-Int | NOA2-StartTime | */
    /* | Num NoA sched   --------------------------------------------------- */
    /* | is 2            | 1        | 4        | 4        | 4              | */
    /* --------------------------------------------------------------------- */
    UWORD8 index = 0;
    UWORD8 len   = (get_num_noa_sched() * P2P_NOA_SCHED_LEN) + 1;

    g_cfg_val[index++] = len;
    g_cfg_val[index++] = get_num_noa_sched();

    for(i = 0; i < get_num_noa_sched(); i++)
    {
        g_cfg_val[index++] = g_noa_dscr[i].cnt_typ;

        val = g_noa_dscr[i].duration;
        g_cfg_val[index++] = (val & 0x000000FF);
        g_cfg_val[index++] = (val & 0x0000FF00) >> 8;
        g_cfg_val[index++] = (val & 0x00FF0000) >> 16;
        g_cfg_val[index++] = (val & 0xFF000000) >> 24;

        val = g_noa_dscr[i].interval;
        g_cfg_val[index++] = (val & 0x000000FF);
        g_cfg_val[index++] = (val & 0x0000FF00) >> 8;
        g_cfg_val[index++] = (val & 0x00FF0000) >> 16;
        g_cfg_val[index++] = (val & 0xFF000000) >> 24;

        val = g_noa_dscr[i].start_time;
        g_cfg_val[index++] = (val & 0x000000FF);
        g_cfg_val[index++] = (val & 0x0000FF00) >> 8;
        g_cfg_val[index++] = (val & 0x00FF0000) >> 16;
        g_cfg_val[index++] = (val & 0xFF000000) >> 24;
    }

    retval = &g_cfg_val[0];
#endif /* MAC_P2P */

    return retval;
}

#endif /* P2P_PS_H */

#endif /* MAC_P2P */
