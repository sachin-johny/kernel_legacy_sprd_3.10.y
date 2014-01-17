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
/*  File Name         : sta_management_11n.h                                 */
/*                                                                           */
/*  Description       : This file contains all the management related        */
/*                      structures, like mlme requests and responses, etc.   */
/*                                                                           */
/*  List of Functions : process_11n_ht_info_sta                              */
/*                      handle_11n_tx_comp_sta                               */
/*                      update_mimo_power_save_sta                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11N

#ifndef STA_MANAGEMENT_11N_H
#define STA_MANAGEMENT_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "management_sta.h"
#include "sta_frame_11n.h"

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern UWORD8 g_smps_mode;
extern BOOL_T g_ap_ht_capable;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void update_11n_params_sta(UWORD8 *msa, UWORD16 offset, UWORD16 rx_len,
                                  sta_entry_t *entry);
extern void update_scan_response_11n(UWORD8 *msa, UWORD16 rx_len,
                                     UWORD16 offset, UWORD8 dscr_set_index);
extern void set_sm_power_save_sta(UWORD8 val);
extern void reset_11n_entry_sta(sta_entry_t *se);
extern void sta_enabled_rx_11n_control(UWORD8 *msg);
extern void sta_enabled_rx_11n_action(UWORD8 *msg);
extern void sta_enabled_action_req_11n(UWORD8 *msg);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the HT capability of the AP */
INLINE void set_ap_ht_cap(BOOL_T val)
{
    g_ap_ht_capable = val;
}

/* This function checks if the AP is HT capable */
INLINE BOOL_T is_ap_ht_cap(void)
{
    return g_ap_ht_capable;
}


/* This function locates and processes the HT Information element. If auto   */
/* protection is enabled and ERP protection is not in use, check and enable  */
/* HT or GF protection. Also check and enable RIFS protection if required.   */
INLINE void process_11n_ht_info_sta(UWORD8 *msa, UWORD16 rx_len, sta_entry_t *se)
{
    UWORD16 index = TAG_PARAM_OFFSET;

    while(index < (rx_len - FCS_LEN))
    {
        if(msa[index] == IHTOPERATION)
        {
            UWORD8 opmode = msa[index + 4] & 0x03;

            /* Set the operating mode extracted to MIB and MAC H/w */
            set_ht_operating_mode(opmode);

            /* If auto protection is enabled and ERP protection is not used  */
            /* enable HT or GF protection as required based on the opertaing */
            /* mode and Non-Greenfield STAs present field values.            */
            if((is_autoprot_enabled() == BTRUE) &&
               (get_protection() != ERP_PROT))
            {
                UWORD8 ngfsta = (msa[index + 4] & BIT2) >> 2;

                if(opmode == 3) /* HT protection required */
                {
                    if(get_protection() != HT_PROT)
                    {
                        set_protection(HT_PROT);
                    }
                }
                else /* HT protection not required */
                {
                    if(ngfsta == 1) /* GF protection required */
                    {
                        if(get_protection() != GF_PROT)
                        {
                            mset_NonGFEntitiesPresent(TV_TRUE);
                            set_protection(GF_PROT);

                            if(is_rifs_prot_enabled() == 0)
                                enable_machw_gf_rifs_prot();
                        }
                    }
                    else /* GF protection not required */
                    {
                        if((get_protection() == GF_PROT) &&
                           (is_rifs_prot_enabled() == 0))
                           disable_machw_gf_rifs_prot();

                        set_protection(NO_PROT);
                    }
                }
            }

            /* Check the RIFS mode set by the AP. If it is 0, disable RIFS   */
            /* if it is currently being used. If it is 1, check if RIFS      */
            /* protection is enabled. If so, enable RIFS protection based on */
            /* the operating mode set by the AP.                             */
            {
                UWORD8 rifsmode = (msa[index + 3] & BIT3) >> 3;

                if(rifsmode == 1)
                {
                    enable_rifs();
                    if(opmode == 1)
                        enable_rifs_prot();
                }
                else
                {
                    disable_rifs();
                }
            }
        }

        /* Increment index by length information and tag header */
        index += msa[index + 1] + IE_HDR_LEN;
    }
}

/* This function updates the SMPS mode in MIB and MAC H/w */
INLINE void update_mimo_power_save_sta(void)
{
    /* Update the MIB with the SMPS mode */
    mset_MIMOPowerSave(g_smps_mode);

    /* Set the MAC H/w register with the Rx MIMO Power save mode */
    set_machw_smps_rx_mode(mget_MIMOPowerSave());
}

/* This function handles transmit complete event for 11n STA mode. It checks */
/* if a SMPS action frame has been transmitted successfully. If so, the      */
/* required SMPS mode in H/w is enabled.                                     */
INLINE void handle_11n_tx_comp_sta(UWORD8 *dscr)
{
#ifndef NMAC_1X1_MODE
    /* Successful transmission */
    if((get_tx_dscr_status((UWORD32 *)dscr) != TX_TIMEOUT) &&
       (get_tx_dscr_frag_status_summary((UWORD32 *)dscr) == 0xFFFFFFFF))
    {
        /* SMPS frame transmitted */
        if(is_smps_frame(dscr) == BTRUE)
        {
            /* SMPS mode updated */
            update_mimo_power_save_sta();
        }
    }
#endif /* NMAC_1X1_MODE */
}




#endif /* STA_MANAGEMENT_11N_H */
#endif /* MAC_802_11N */
#endif /* IBSS_BSS_STATION_MODE */
