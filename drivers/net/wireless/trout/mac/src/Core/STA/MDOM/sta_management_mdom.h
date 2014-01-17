/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : sta_management_mdom.h                                */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      definitions for Multi-domain operation.              */
/*                                                                           */
/*  List of Functions : update_rd_max_tx_pow_subbands                        */
/*                      update_bss_curr_oper_cls                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_MULTIDOMAIN
#ifndef STA_MANAGEMENT_MDOM_H
#define STA_MANAGEMENT_MDOM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "cglobals_sta.h"
#include "mib_mdom.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void update_scan_response_11d(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                     UWORD8 dscr_set_index);
extern BOOL_T check_bss_reg_domain_cap_info(bss_dscr_t *bss_dscr);
extern void update_11d_reg_max_tx_pow(UWORD8 *msa, UWORD16 rx_len,
                                      UWORD16 index);
extern void adjust_bss_width_in_new_rc(UWORD8 reg_class);
extern void adopt_new_rc(UWORD8 reg_class, UWORD8 ch_idx, UWORD8 sec_ch_offset);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/
/* This function is called to update Max Tx powers of the sub band frequencies */
INLINE void update_rd_max_tx_pow_subbands(UWORD8 chan, UWORD8 reg_max_tx_pow)
{
    UWORD8 ch_idx             = 0;
    UWORD8 reg_class          = 0;
    UWORD8 freq               = get_current_start_freq();
    reg_class_info_t *rc_info = NULL;

    ch_idx    = get_ch_idx_from_num(freq, chan);
    reg_class = get_reg_class_from_ch_idx(freq, ch_idx);
    rc_info   = get_reg_class_info(freq, reg_class, get_current_reg_domain());

    if(rc_info == NULL)
    {
        PRINTD("Warning:MultiDomain: NULL Reg Class Info\n\r");
        return;
    }

     if(get_rc_max_reg_tx_pow(rc_info) > reg_max_tx_pow)
    {
        set_rc_max_reg_tx_pow(rc_info, reg_max_tx_pow);
    }

}
/* Current operating class of the BSS network is extracted from the Supported */
/* Operating classes Information Element                                      */
INLINE void update_bss_curr_oper_cls(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                     UWORD8 dscr_set_index)
{
   /*************************************************************************/
   /*                  Supported Operating Classes Element                  */
   /* --------------------------------------------------------------------- */
   /* |Element ID|Length |Current Operating Class|Operating Classes       | */
   /* --------------------------------------------------------------------- */
   /* |1         |1      |1                      |1                       | */
   /* --------------------------------------------------------------------- */
   /*                                                                       */
   /*************************************************************************/

    while(index < (rx_len - FCS_LEN))
    {
        if(ISUPOPCLASS == msa[index])
        {
            /* Update Regulatory class field of the BSS Descriptor */
            g_bss_dscr_set[dscr_set_index].reg_class = msa[index+IE_HDR_LEN];
            return;
        }

        index += (IE_HDR_LEN + msa[index + 1]);
    }
}

#endif /* STA_MANAGEMENT_MDOM_H */
#endif /* MAC_MULTIDOMAIN */
#endif /* IBSS_BSS_STATION_MODE */
