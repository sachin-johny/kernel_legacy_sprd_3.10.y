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
/*  File Name         : rsna_auth_glk.c                                      */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of the Global Group Key   */
/*                      Handshake state machine.                             */
/*                                                                           */
/*  List of Functions : rsna_auth_glk_fsm                                    */
/*                      rsna_auth_glk_init                                   */
/*                      rsna_auth_glk_setkeysdone                            */
/*                      rsna_auth_glk_setkeys                                */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "rsna_auth_km.h"
#include "rkmal_auth.h"
#include "utils.h"
#include "mib_11i.h"
#include "mh.h"
#include "ce_lut.h"
#ifdef BSS_ACCESS_POINT_MODE
#include "management_ap.h"
#include "ieee_11i_ap.h"
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef AUTH_11I
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static UWORD16 get_gtk_hs_timeout(UWORD8);
static void rsna_auth_glk_init(void);
static void rsna_auth_glk_setkeysdone(void);
static void rsna_auth_glk_setkeys(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_gtk_hs_timeout                                    */
/*                                                                           */
/*  Description      : Returns the time out value for the whole GTK          */
/*                     handshake process                                     */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : Calculates the time out from num of stations          */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_gtk_hs_timeout(UWORD8 num_sta)
{
    return MAX(num_sta * GTKHS_PERSTA_TIMEOUT, GTKHS_MIN_TIMEOUT);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_glk_fsm                                     */
/*                                                                           */
/*  Description      : This function implements the Global Group Key         */
/*                     Handshake state machine. In each of the various       */
/*                     states, it checks for the appropriate inputs before   */
/*                     switching to next state. This state machine has only  */
/*                     one instance and is global to the authenticator.      */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_glk_fsm(void)
{
    /* If Init Condition is true, initialization is done */
    if(g_rsna_auth.global.GInit == BTRUE)
    {
        rsna_auth_glk_init();
    }

    if(g_rsna_auth.global.GTKReKeyTimeOut == BTRUE)
    {
        rsna_auth_glk_handle_timeout();
    }

    switch(g_rsna_auth.global.rsna_auth_glk_state)
    {
    case AGLK_GTK_INIT:
    {
        if(g_rsna_auth.global.GTKAuthenticator == BTRUE)
        {
            rsna_auth_glk_setkeysdone();
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case AGLK_GTK_INIT: */
    break;

    case AGLK_SETKEYS:
    {
        if(g_rsna_auth.global.GTKReKey == BTRUE)
        {
            rsna_auth_glk_setkeys();
        }
        else if(g_rsna_auth.global.GKeyDoneStations == 0)
        {
            rsna_auth_glk_setkeysdone();
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case AGLK_SETKEYS: */
    break;

    case AGLK_SETKEYSDONE:
    {
        if(g_rsna_auth.global.GTKReKey == BTRUE)
        {
            rsna_auth_glk_setkeys();
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case AGLK_SETKEYSDONE: */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(g_rsna_auth.global.rsna_auth_glk_state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_glk_init                                    */
/*                                                                           */
/*  Description      : This function implements the GTK_INIT state of the    */
/*                     global group key handshake state machine.             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_glk_init(void)
{
    UWORD32  i          = 0;
    UWORD32  j          = 0;
    UWORD8   *gnonce    = 0;
    UWORD16  gtk_size   = 0;
    UWORD8   grp_policy = mget_RSNAConfigGroupCipher();

    /* Reset the init condition */
    g_rsna_auth.global.GInit = BFALSE;

    /* Reset GTKs */
    for(i = 0; i < NUM_GTK; i++)
    {
        for(j = 0; j < GTK_SIZE; j++)
            g_rsna_auth.global.GTK[i][j] = 0;
    }

    /* Set the GTK Indices */
    if((grp_policy != 0x01) && (grp_policy != 0x05))
    {
        g_rsna_auth.global.GN = 1;
        g_rsna_auth.global.GM = 2;
    }
    else
    {
        g_rsna_auth.global.GN = 0;
        g_rsna_auth.global.GM = 0;
    }

    /* Reset the time out */
    g_rsna_auth.global.GTKReKeyTimeOut = BFALSE;

    /* Obtain GNonce from the global counter.*/
    /* Update the nonce with the GTK Counter */
    memcpy(g_rsna_auth.global.GNonce, g_rsna_auth.global.GCounter, NONCE_SIZE);

    /* Increment the Nonce Counter */
    incr_cnt(g_rsna_auth.global.GCounter, NONCE_SIZE);

    gnonce = g_rsna_auth.global.GNonce;

    /* Set the GTK Size as the number of bits in the group key */
    gtk_size = mget_RSNAConfigGroupCipherSize();

    if((grp_policy != 0x01) && (grp_policy != 0x05))
    {
        /* Obtain the GTK using the GMK and GNonce. GMK is saved in a global.*/
        /* Indices GN and GM are swapped and GTK[GN] is set on rekey.        */
        get_gtk(g_gmk, g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)], gtk_size,
                mget_bssid(), gnonce);
    }
    else
    {
        memcpy(g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)],
               mget_WEPDefaultKeyValue(), (mget_WEPDefaultKeySize() >> 3));
    }

    /* Change the state to AGLK_GTK_INIT */
    g_rsna_auth.global.rsna_auth_glk_state = AGLK_GTK_INIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_glk_setkeysdone                             */
/*                                                                           */
/*  Description      : This function implements the SETKEYSDONE state of the */
/*                     global group key handshake state machine.             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_glk_setkeysdone(void)
{
    UWORD8 grp_policy    = 0;
    UWORD8 key_id        = 0;
    UWORD8 key_index     = 0;
    CIPHER_T cipher_type = NO_ENCRYP;
    UWORD8 key[16]       = {0};
    UWORD8 mask          = {0};
    UWORD8 *mic_key      = NULL;

    /* Stop the timeout timer */
    stop_rekey_timer();

    /* Send an MLME-SetKeys.Request(GN, Tx/Rx, GTK[GN]) */
    /* Get Group policy */
    grp_policy = mget_RSNAConfigGroupCipher();

    if(grp_policy == 4)
    {
        UWORD16 i   = 0;
        cipher_type = (CIPHER_T) CCMP;
        key_id      = g_rsna_auth.global.GN;
        for(i = 0 ; i < 16; i ++)
        {
            key[i] = g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)][i];
        }
    }
    else if(grp_policy == 2)
    {
        UWORD16 i   = 0;
        cipher_type = (CIPHER_T) TKIP;
        key_id      = g_rsna_auth.global.GN;
        for(i = 0 ; i < 16; i ++)
        {
            key[i] = g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)][i];
        }
        mic_key = &g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)][TEMPORAL_KEY_LENGTH];
    }
    else if(grp_policy == 1)
    {
        cipher_type = WEP40;
        key_id      = mget_WEPDefaultKeyID();
        mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
        memcpy(key, mget_WEPDefaultKeyValue(), 5);
    }
    else if(grp_policy == 5)
    {
        cipher_type = WEP104;
        key_id      = mget_WEPDefaultKeyID();
        mask        = LUT_KEY_PRGM_MASK | LUT_ADDR_PRGM_MASK;
        memcpy(key, mget_WEPDefaultKeyValue(), 13);
    }

    key_index = 0;

    /* Set keys for the broad cast/multicast addresses */
    /* Key num = 0 for Group Key                       */
    machw_ce_add_key(key_id, TX_GTK_NUM, key_index, mask, cipher_type,
                     key, mget_bcst_addr(), AUTH_KEY, mic_key);

    /* For RSNA and Non TSN networks, start the group update          */
    /* Depending on the Rekey method the new ReKey alarm is set       */
    /* If the Rekey Method is time based, the rekey alarm is set here */
    if(((grp_policy == 2) || (grp_policy == 4))  &&
       ((mget_RSNAConfigGroupRekeyMethod() == 2) ||
       (mget_RSNAConfigGroupRekeyMethod() == 4)))
    {
        start_rekey_timer(mget_RSNAConfigGroupRekeyTime() * 1000, DO_NEXT_REKEY);
    }

    /* Change the state to AGLK_SETKEYSDONE */
    g_rsna_auth.global.rsna_auth_glk_state = AGLK_SETKEYSDONE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_glk_setkeys                                 */
/*                                                                           */
/*  Description      : This function implements the SETKEYS state of the     */
/*                     global group key handshake state machine.             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_glk_setkeys(void)
{
    UWORD32 i      = 0;
    UWORD8 *gnonce = 0;

    g_rsna_auth.global.GTKReKey = BFALSE;

    /* Swap GM, GN */
    {
        UWORD8 temp = 0;

        temp                  = g_rsna_auth.global.GM;
        g_rsna_auth.global.GM = g_rsna_auth.global.GN;
        g_rsna_auth.global.GN = temp;
    }

    /* Update the nonce with the GTK Counter */
    memcpy(g_rsna_auth.global.GNonce, g_rsna_auth.global.GCounter, NONCE_SIZE);

    /* Increment the Nonce Counter */
    incr_cnt(g_rsna_auth.global.GCounter, NONCE_SIZE);

    gnonce = g_rsna_auth.global.GNonce;

    /* Obtain the GTK using the GMK and GNonce. The GMK is saved in a global.*/
    /* The indices GN and GM are swapped and GTK[GN] is set on rekey.        */
    get_gtk(g_gmk, g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)],
        (UWORD16)(mget_RSNAConfigGroupCipherSize()), mget_bssid(), gnonce);

    g_rsna_auth.global.GKeyDoneStations = 0;

    for(i = 0; i < NUM_SUPP_STA; i++)
    {
        if((g_rsna_auth.rsna_ptr[i] != NULL))
        {
#ifdef BSS_ACCESS_POINT_MODE
            rsna_auth_persta_t *rsna_ptr = g_rsna_auth.rsna_ptr[i];
            asoc_entry_t *ae = (asoc_entry_t *)find_entry(rsna_ptr->addr);

            if(rsna_ptr->rsna_auth_grpkey_state != AGK_INVALID_STATE &&
                BTRUE == is_11i_hs_complete(ae))
            {
                rsna_ptr->GUpdateStationKeys = BTRUE;
	            g_rsna_auth.global.GKeyDoneStations++;
	        }

            else if(BFALSE == is_11i_hs_complete(ae))
            {
                if(ae != NULL)
                {
                    if(ae->persta != NULL)
                    {
                        if(ae->persta->sta_1x != NULL)
                        {
                            PRINTD("Warning: 11i HS flags during GTK update %x %x %d\n",
                                ((auth_t *)(ae->persta->sta_1x))->global.keyDone,
                                ((auth_t *)(ae->persta->sta_1x))->global.portValid,
                                rsna_ptr->rsna_auth_4way_state);
                        }
                    }
                }

            }
#endif /* BSS_ACCESS_POINT_MODE */
        }
    }

    /* Change the state to AGLK_SETKEYS */
    g_rsna_auth.global.rsna_auth_glk_state = AGLK_SETKEYS;

    /* Start the timer to handle the whole GTK time out process  */
    /* Re-using the existing GTK re-key timer */
    if(g_rsna_auth.global.GKeyDoneStations)
    {
        start_rekey_timer(
            get_gtk_hs_timeout(g_rsna_auth.global.GKeyDoneStations),
            REKEY_TIMEOUT);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_glk_handle_timeout                          */
/*                                                                           */
/*  Description      : This function implements the timeout process for the  */
/*                     global group key handshake state machine.             */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_glk_handle_timeout(void)
{
    UWORD32 i      = 0;

    g_rsna_auth.global.GTKReKeyTimeOut = BFALSE;

    if(g_rsna_auth.global.GKeyDoneStations)
    {
        for(i = 0; i < NUM_SUPP_STA; i++)
        {
            if((g_rsna_auth.rsna_ptr[i] != NULL))
            {
                if(g_rsna_auth.rsna_ptr[i]->GUpdateStationKeys == BTRUE)
                {
                    g_rsna_auth.rsna_ptr[i]->GTimeout = BTRUE;
                }
            }
        }
        g_rsna_auth.global.GKeyDoneStations = 0;
    }
}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
