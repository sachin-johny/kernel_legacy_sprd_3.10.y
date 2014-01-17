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
/*  File Name         : rsna_auth_4way.c                                     */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of the Per-STA 4-Way Key  */
/*                      Handshake state machine.                             */
/*                                                                           */
/*  List of Functions : rsna_auth_4way_fsm                                   */
/*                      rsna_auth_authentication                             */
/*                      rsna_auth_authentication2                            */
/*                      rsna_auth_disconnect                                 */
/*                      rsna_auth_disconnected                               */
/*                      rsna_auth_initialize                                 */
/*                      rsna_auth_initpmk                                    */
/*                      rsna_auth_initpsk                                    */
/*                      rsna_auth_ptkcalcnegotiating                         */
/*                      rsna_auth_ptkcalcnegotiating2                        */
/*                      rsna_auth_ptkinitnegotiating                         */
/*                      rsna_auth_ptkinitdone                                */
/*                      rsna_auth_ptkstart                                   */
/*                      STADisconnect                                        */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "itypes.h"
#include "rsna_auth_km.h"
#include "eapol_key.h"
#include "eapol_key_auth.h"
#include "prf.h"
#include "utils.h"
#include "rkmal_auth.h"
#include "aes.h"
#include "md5.h"
#include "mh.h"
#include "mib_11i.h"
#include "frame_11i.h"
#include "auth_frame_11i.h"
#include "ce_lut.h"
#include "rc4.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void rsna_auth_authentication(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_authentication2(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_disconnect(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_initialize(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_initpmk(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_initpsk(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_ptkcalcnegotiating(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_ptkcalcnegotiating2(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_ptkinitnegotiating(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_ptkinitdone(rsna_auth_persta_t *rsna_ptr);
static void rsna_auth_ptkstart(rsna_auth_persta_t *rsna_ptr);
static void STADisconnect(rsna_auth_persta_t *rsna_ptr);


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_4way_fsm                                    */
/*                                                                           */
/*  Description      : This function implements the Per-STA 4-Way Key        */
/*                     Handshake state machine. In each of the various       */
/*                     states, it checks for the appropriate inputs before   */
/*                     switching to next state. This state machine is        */
/*                     instantiated per supplicant.                          */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_4way_fsm(rsna_auth_persta_t *rsna_ptr)
{
    if(rsna_ptr->Disconnect == BTRUE)
    {
        rsna_auth_disconnect(rsna_ptr);
        return;
    }

    if(rsna_ptr->DeauthenticationRequest == BTRUE)
    {
        rsna_auth_disconnected(rsna_ptr);
        return;
    }

    if(rsna_ptr->InitA4 == BTRUE)
    {
        rsna_auth_initialize(rsna_ptr);
        return;
    }

    if(rsna_ptr->AuthenticationRequest == BTRUE)
    {
        rsna_auth_authentication(rsna_ptr);
        return;
    }

    if(rsna_ptr->ReAuthenticationRequest == BTRUE)
    {
        rsna_auth_authentication2(rsna_ptr);
        return;
    }


    switch(rsna_ptr->rsna_auth_4way_state)
    {
    case A4_AUTHENTICATION:
    {
        rsna_auth_authentication2(rsna_ptr);
    } /* End of case A4_AUTHENTICATION: */
    break;

    case A4_AUTHENTICATION2:
    {
        if(NULL != rsna_ptr->sta_1x) {
            if(((auth_t *)(rsna_ptr->sta_1x))->global.keyRun == BTRUE)
            {
                if((g_psk_available == BFALSE) && (rsna_ptr->auth_policy != 0x02))
                {
                    rsna_auth_initpmk(rsna_ptr);
                }
                else
                {
                    if(rsna_ptr->auth_policy == 0x02)
                        rsna_auth_initpsk(rsna_ptr);
                    else
                    {
                        /* Set the deauth reason and initiate a disconnect */
                        rsna_ptr->deauth_reason = INVALID_AKMP;
                        rsna_auth_disconnect(rsna_ptr);
                    }
                }
            }
            else
            {
                /* Do Nothing */
            }
		}
		else
		{
		     /* Set the deauth reason and initiate a disconnect */
            rsna_ptr->deauth_reason = INVALID_AKMP;
            rsna_auth_disconnect(rsna_ptr);
            printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
		}
    } /* End of case A4_AUTHENTICATION2: */
    break;

    case A4_DISCONNECT:
    {
        rsna_auth_disconnected(rsna_ptr);
    } /* End of case A4_DISCONNECT: */
    break;

    case A4_DISCONNECTED:
    {
        rsna_auth_initialize(rsna_ptr);
    } /* End of case A4_DISCONNECTED: */
    break;

    case A4_INITIALIZE:
    {
        /* Do Nothing */
    } /* End of case A4_INITIALIZE: */
    break;

    case A4_INITPMK:
    {
      //xuan.yang, 2013-10-17, check sta_1x
      if(NULL != rsna_ptr->sta_1x) {
        if(((auth_t *)(rsna_ptr->sta_1x))->global.keyAvailable)
        {
            rsna_auth_ptkstart(rsna_ptr);
        }
        else
        {
            /* Set the deauth reason and initiate a disconnect */
            rsna_ptr->deauth_reason = UNSPEC_REASON;
            rsna_auth_disconnect(rsna_ptr);
        }
	  }
	  else{
          /* Set the deauth reason and initiate a disconnect */
          rsna_ptr->deauth_reason = UNSPEC_REASON;
          rsna_auth_disconnect(rsna_ptr);
          printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	  }
    } /* End of case A4_INITPMK: */
    break;

    case A4_INITPSK:
    {
        //xuan.yang, 2013-10-17, check sta_1x
	    if(NULL != rsna_ptr->sta_1x) {
	       if(((auth_t *)(rsna_ptr->sta_1x))->global.keyAvailable)
	       {
	           rsna_auth_ptkstart(rsna_ptr);
	       }
	       else
	       {
	           /* Do Nothing */
	       }
		}
	    else {
			    /* Set the deauth reason and initiate a disconnect */
			   rsna_ptr->deauth_reason = INVALID_AKMP;
			   rsna_auth_disconnect(rsna_ptr);
			   printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	   }
	} /* End of case A4_INITPSK: */
    break;

    case A4_PTKCALCNEGOTIATING:
    {
        if(enough_attempts_4way(rsna_ptr) == BTRUE)
        {
            mincr_RSNA4WayHandshakeFailures();
            /* Set the Disconect reason */
            rsna_ptr->deauth_reason = HS_4W_TIMEOUT;
            /* Initiate a disconnect */
            rsna_auth_disconnect(rsna_ptr);
        }
        else if(rsna_ptr->TimeoutEvt == BTRUE)
        {
            rsna_auth_ptkstart(rsna_ptr);
        }
        else if(rsna_ptr->MICVerified == BTRUE)
        {
            rsna_auth_ptkcalcnegotiating2(rsna_ptr);

            /* Reset MIC Valid flag */
            rsna_ptr->MICVerified = BFALSE;
        }
        else if((rsna_ptr->EAPOLKeyReceived == BTRUE)     &&
                (!get_request(rsna_ptr->eapol_key_frame)) &&
                (get_k(rsna_ptr->eapol_key_frame) == P)   &&
                (check_eapol_pkt_auth(rsna_ptr)   == BTRUE))
        {
            rsna_auth_ptkcalcnegotiating(rsna_ptr);

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;
        }
        else if(rsna_ptr->EAPOLKeyReceived == BTRUE)
        {
            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Restart timer */
            start_11i_auth_timer(rsna_ptr,
                    get_11i_resp_timeout((UWORD8)rsna_ptr->TimeoutCtr,
                    rsna_ptr->listen_interval));
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case A4_PTKCALCNEGOTIATING: */
    break;

    case A4_PTKCALCNEGOTIATING2:
    {
        rsna_auth_ptkinitnegotiating(rsna_ptr);
    } /* End of case A4_PTKCALCNEGOTIATING2: */
    break;

    case A4_PTKINITNEGOTIATING:
    {
        if(enough_attempts_4way(rsna_ptr) == BTRUE)
        {
            mincr_RSNA4WayHandshakeFailures();
            /* Set the Disconect reason */
            rsna_ptr->deauth_reason = HS_4W_TIMEOUT;
            /* Initiate a disconnect */
            rsna_auth_disconnect(rsna_ptr);
        }
        else if(rsna_ptr->TimeoutEvt == BTRUE)
        {
            rsna_auth_ptkinitnegotiating(rsna_ptr);
        }
        else if((rsna_ptr->EAPOLKeyReceived == BTRUE)     &&
                (!get_request(rsna_ptr->eapol_key_frame)) &&
                (get_k(rsna_ptr->eapol_key_frame) == P)   &&
                (check_eapol_pkt_auth(rsna_ptr)   == BTRUE))
        {
            /* Calculate and verify EAPOL Mic field in the received packet */
            rsna_ptr->MICVerified = verifyMIC(rsna_ptr->eapol_key_frame,
                                              rsna_ptr->eapol_frame_len,
                                              rsna_ptr->PTK,
                                              rsna_ptr->key_version);

            /* Check the MIC for the received packet */
            if(rsna_ptr->MICVerified == BTRUE)
            {
                rsna_auth_ptkinitdone(rsna_ptr);
            }
            else
            {
                mincr_RSNA4WayHandshakeFailures();
                /* Set the Disconect reason */
                rsna_ptr->deauth_reason = MIC_FAIL;
                /* Initiate a disconnect */
                rsna_auth_disconnect(rsna_ptr);
            }

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Reset MIC Valid flag */
            rsna_ptr->MICVerified = BFALSE;
        }
        else if(rsna_ptr->EAPOLKeyReceived == BTRUE)
        {
            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Restart timer */
            start_11i_auth_timer(rsna_ptr,
                    get_11i_resp_timeout((UWORD8)rsna_ptr->TimeoutCtr,
                    rsna_ptr->listen_interval));
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case A4_PTKINITNEGOTIATING: */
    break;

    case A4_PTKINITDONE:
    {
        /* Do Nothing */
    } /* End of case A4_PTKINITDONE: */
    break;

    case A4_PTKSTART:
    {
        if(enough_attempts_4way(rsna_ptr) == BTRUE)
        {
            mincr_RSNA4WayHandshakeFailures();
            /* Set the Disconect reason */
                rsna_ptr->deauth_reason = AUTH_1X_FAIL;
            /* Initiate a disconnect */
            rsna_auth_disconnect(rsna_ptr);
        }
        else if(rsna_ptr->TimeoutEvt == BTRUE)
        {
            rsna_auth_ptkstart(rsna_ptr);
        }
        else if((rsna_ptr->EAPOLKeyReceived == BTRUE)     &&
                (!get_request(rsna_ptr->eapol_key_frame)) &&
                (get_k(rsna_ptr->eapol_key_frame) == P)   &&
                (check_eapol_pkt_auth(rsna_ptr)   == BTRUE))
        {
            rsna_auth_ptkcalcnegotiating(rsna_ptr);

            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;
        }
        else if(rsna_ptr->EAPOLKeyReceived == BTRUE)
        {
            /* Reset the Key Frame received flag */
            rsna_ptr->EAPOLKeyReceived = BFALSE;

            /* Restart timer */
            start_11i_auth_timer(rsna_ptr,
                    get_11i_resp_timeout((UWORD8)rsna_ptr->TimeoutCtr,
                    rsna_ptr->listen_interval));
        }
        else
        {
            /* Do Nothing */
        }
    } /* End of case A4_PTKSTART: */
    break;

    default:
    {
        /* Do nothing */
    }
    } /* End of switch(rsna_ptr->rsna_auth_4way_state) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_authentication                              */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATION state of  */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_authentication(rsna_auth_persta_t *rsna_ptr)
{
    {
        UWORD8 i = 0;

        for(i = 0; i < PTK_SIZE; i++)
            rsna_ptr->PTK[i] = 0;
    }

    //xuan.yang, 2013-10-17, check sta_1x
	if(NULL != rsna_ptr->sta_1x) {
        ((auth_t *)(rsna_ptr->sta_1x))->global.portControl = AUTO;
        ((auth_t *)(rsna_ptr->sta_1x))->global.portEnabled = BTRUE;
	}
	else{
			/* Set the deauth reason and initiate a disconnect */
			rsna_ptr->deauth_reason = INVALID_AKMP;
			rsna_auth_disconnect(rsna_ptr);
			printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
			return;
	}

    rsna_ptr->AuthenticationRequest = BFALSE;

    /* Set State to A4_AUTHENTICATION */
    rsna_ptr->rsna_auth_4way_state  = A4_AUTHENTICATION;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_authentication2                             */
/*                                                                           */
/*  Description      : This function implements the AUTHENTICATION2 state of */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_authentication2(rsna_auth_persta_t *rsna_ptr)
{

    memcpy(rsna_ptr->ANonce, &(g_rsna_auth.global.Counter),
           NONCE_SIZE);

    incr_cnt(g_rsna_auth.global.Counter, NONCE_SIZE);

    rsna_ptr->ReAuthenticationRequest = BFALSE;

    /* Set State to A4_AUTHENTICATION2 */
    rsna_ptr->rsna_auth_4way_state  = A4_AUTHENTICATION2;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_disconnect                                  */
/*                                                                           */
/*  Description      : This function implements the DISCONNECT state of      */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_disconnect(rsna_auth_persta_t *rsna_ptr)
{
    /* Initiate Disconnect */
    STADisconnect(rsna_ptr);
    rsna_ptr->Disconnect = BFALSE;

    /* Stop any running timer */
    stop_11i_auth_timer(rsna_ptr);

    /* Set State to A4_DISCONNECT */
    rsna_ptr->rsna_auth_4way_state  = A4_DISCONNECT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_disconnected                                */
/*                                                                           */
/*  Description      : This function implements the DISCONNECTED state of    */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_disconnected(rsna_auth_persta_t *rsna_ptr)
{
    rsna_ptr->DeauthenticationRequest = BFALSE;

    /* Set State to A4_DISCONNECTED */
    rsna_ptr->rsna_auth_4way_state  = A4_DISCONNECTED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_initialize                                  */
/*                                                                           */
/*  Description      : This function implements the INITIALIZE state of      */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_initialize(rsna_auth_persta_t *rsna_ptr)
{
    rsna_ptr->keycount = 0;

    if(rsna_ptr->GUpdateStationKeys == BTRUE)
    {
        g_rsna_auth.global.GKeyDoneStations--;
        rsna_ptr->GUpdateStationKeys = BFALSE;
    }

    //xuan.yang, 2013-10-17, check sta_1x
    if(NULL != rsna_ptr->sta_1x) {       
         /*If unicast cipher supported by Authenticator AND (ESS OR (IBSS or WDS)*/
         /* and Local AA> RemoteAA))), Pair = TRUE.                               */
         /* ETBD */
         ((auth_t *)(rsna_ptr->sta_1x))->global.portEnabled = BFALSE;
			
		 /* Send MLME-DeleteKeys.Request(PTK) */
		 /* MLMETBD */
		 ((auth_t *)(rsna_ptr->sta_1x))->global.portValid = BFALSE;
	  } else {
	      /* Set the deauth reason and initiate a disconnect */
	      rsna_ptr->deauth_reason = INVALID_AKMP;
	      rsna_ptr->send_deauth = BTRUE;
	      printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	   }

    rsna_ptr->TimeoutCtr = 0;

    /* Init Request is set to False */
    rsna_ptr->InitA4 = BFALSE;

    /* Set State to A4_INITIALIZE */
    rsna_ptr->rsna_auth_4way_state  = A4_INITIALIZE;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_initpmk                                     */
/*                                                                           */
/*  Description      : This function implements the INITPMK state of the     */
/*                     per-sta 4-way key handshake state machine.            */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_initpmk(rsna_auth_persta_t *rsna_ptr)
{
    //xuan.yang, 2013-10-17, check sta_1x
	if(NULL != rsna_ptr->sta_1x) {
	    /* Get PMK = L(AAA Key, 0, 256) */
	     get_pmk(((auth_t *)(rsna_ptr->sta_1x))->rx_key, rsna_ptr->PMK);
	} else {
	     /* Set the deauth reason and initiate a disconnect */
	      rsna_ptr->deauth_reason = INVALID_AKMP;
	      rsna_auth_disconnect(rsna_ptr);
	      printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	      return;
	}

    /* Set State to A4_INITPMK */
    rsna_ptr->rsna_auth_4way_state  = A4_INITPMK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_initpsk                                     */
/*                                                                           */
/*  Description      : This function implements the INITPSK state of the     */
/*                     per-sta 4-way key handshake state machine.            */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_initpsk(rsna_auth_persta_t *rsna_ptr)
{
    memcpy(rsna_ptr->PMK, mget_RSNAConfigPSKValue(), 32);

    /* Set State to A4_INITPSK */
    rsna_ptr->rsna_auth_4way_state  = A4_INITPSK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_ptkcalcnegotiating                          */
/*                                                                           */
/*  Description      : This function implements the PTKCALCNEGOTIATING state */
/*                     of the per-sta 4-way key handshake state machine.     */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_ptkcalcnegotiating(rsna_auth_persta_t *rsna_ptr)
{
    UWORD16 ptk_size      = 0;
    UWORD8 *pmk           = 0;
    UWORD8 *aa            = 0;
    UWORD8 *spa           = 0;
    UWORD8 *anonce        = 0;
    UWORD8 *snonce        = 0;
    UWORD8 *key_data      = 0;
    UWORD8 key_data_valid = 0xFF;
    UWORD8 grp_policy     = 0;
    UWORD8 pcip_policy    = 0;
    UWORD8 auth_policy    = 0;
    UWORD8 reason         = 0;
    UWORD8 index          = 0;
    UWORD8 *oui           = 0;

    key_data = rsna_ptr->eapol_key_frame + EAPOL_HDR_LEN + KEY_DATA_OFFSET;

    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
        if(key_data[index] != IRSNELEMENT)
        {
             reason         = INFO_ELMNT_FAIL;
             key_data_valid = 0x00;
        }

        oui = mget_RSNAOUI();

        /* Skip the Tag number (1)  and the length (1) */
        index += 2;
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        if(is_wpa_ie(key_data + index) == BFALSE)
        {
            reason         = INFO_ELMNT_FAIL;
            key_data_valid = 0x00;
        }

        oui = mget_WPAOUI();

        /* Skip the Tag number (1)  and the length (1) */
        index += 2;

        /* Skip the OUI and OUI Type */
        index += 4;
    }
    else /* Unknown mode_802_11i */
    {
        return;
    }

    /* Verify the verion number for the 802.11i at STA       */
    /* Check only if there had been no discrepancies earlier */
    if(key_data_valid == 0xFF)
    {
        if(key_data[index] != mget_RSNAConfigVersion())
        {
            /* On failure status code is set appropriately*/
            reason         = RSN_VER_FAIL;
            key_data_valid = 0x00;
        }
    }

    /* Skip the version number */
    index += 2;

    /* Get AKM, PW, and GW Cipher policies from the frame */
    /* Check only if there had been no discrepancies earlier */
    if(key_data_valid == 0xFF)
    {
        UWORD8 temp   = 0;

        /* Check the OUI for the group cipher policy */
        if(check_oui(&key_data[index], oui) == BFALSE)
        {
            reason         = INFO_ELMNT_FAIL;
            key_data_valid = 0x00;
        }

        index += 3; /* Skip the OUI Length */

        grp_policy  = key_data[index];
        index       ++;

        pcip_policy = get_pcip_policy_auth(&key_data[index], &temp, oui);
        index      += temp;

        auth_policy = get_auth_policy_auth(&key_data[index], &temp, oui);
        index      += temp;

        /* Verify the AKM policy for the 802.11i at STA */
        if((auth_policy == 0xFF) && (key_data_valid == 0xFF))
        {
            /* On failure status code is set appropriately*/
            reason         = INVALID_AKMP;
            key_data_valid = 0x00;
        }

        /* Verify the Pairwise Cipher policy for the 802.11i at STA */
        if((pcip_policy == 0xFF) && (key_data_valid == 0xFF))
        {
            /* On failure status code is set appropriately*/
            reason         = PW_CP_FAIL;
            key_data_valid = 0x00;
        }

        /* Prohibit use of TKIP as P/W Cipher when CCMP is Grp Cipher */
        if((grp_policy == 0x04) && (pcip_policy == 0x02))
        {
            /* On failure status code is set appropriately*/
            reason         = PW_CP_FAIL;
            key_data_valid = 0x00;
        }
    }

    /* Verify the Group policy for the 802.11i at STA */
    if((grp_policy != mget_RSNAConfigGroupCipher())
                      && (key_data_valid == 0xFF))
    {
        /* On failure key data valid is reset appropriately*/
        key_data_valid  = 0x00;
        reason          = GRP_CP_FAIL;
    }

    /* Verify the AKM policy for the 802.11i at STA */
    if((rsna_ptr->auth_policy != auth_policy) && (key_data_valid == 0xFF))
    {
        /* On failure key data valid is reset appropriately*/
        key_data_valid  = 0x00;
        reason          = DIFF_INFO_ELEM;
    }

    /* Verify the Pairwise Cipher policy for the 802.11i at STA */
    /* If pcip policy is zero pcip policy and grp policy must be same */
    if(((pcip_policy != 0 && rsna_ptr->pcip_policy != pcip_policy)
              || (pcip_policy == 0 && rsna_ptr->pcip_policy != 0))
              && (key_data_valid == 0xFF))
    {
        /* On failure key data valid is reset appropriately*/
        key_data_valid  = 0x00;
        reason          = DIFF_INFO_ELEM;
    }

    /* For 802.11I RSNA, RSN Capability field must match */
    if((rsna_ptr->mode_802_11i == RSNA_802_11I) && (key_data_valid == 0xFF))
    {

    /* RSN Capabilities Information                                          */
    /*************************************************************************/
    /* --------------------------------------------------------------------- */
    /* | B15 - B6  |  B5 - B4      | B3 - B2     |       B1    |     B0    | */
    /* --------------------------------------------------------------------- */
    /* | Reserved  |  GTSKA Replay | PTSKA Replay| No Pairwise | Pre - Auth| */
    /* |           |    Counter    |   Counter   |             |           | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/

        /* Check Pre-Auth status in the capability */
        if((mget_RSNAPreauthenticationImplemented() == TV_TRUE) &&
           (mget_RSNAPreauthenticationEnabled() == TV_TRUE))
        {
            if(!(key_data[index] & BIT0))
            {
                /* On failure key data valid is reset appropriately*/
                key_data_valid  = 0x00;
                reason          = RSN_CAP_FAIL;
            }
        }

        /* Check Pairwise field in the Capability field        */
        /* This shall be set only if the P/W policy is TKIP &  */
        /* in a TSN                                            */
        if((grp_policy != 0x01) && (grp_policy != 0x05))
        {
        }
        else if((key_data[index] & BIT1) && (pcip_policy == 0x02))
        {
            pcip_policy = grp_policy;
        }

        /* Check PTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfPTSKAReplayCounters  */
        if((key_data[index] & (BIT2 | BIT3)) !=
           ((mget_RSNAConfigNumberOfPTKSAReplayCounters() << 2)
               & 0x000C))
        {
            /* Replay counter if less than our value implies 11e */
            /* mismatch, so the packets should not be sent in    */
            /* other queues, to be taken care of by MAC S/w      */
        }

        /* Check GTSKA Replay counters. This field is taken from */
        /* the MIB's dot11RSNAConfigNumberOfGTSKAReplayCounters  */
        if((key_data[index] & (BIT4 | BIT5)) !=
            ((mget_RSNAConfigNumberOfGTKSAReplayCounters() << 4)
               & 0x0030))
        {
            /* Replay counter if less than our value implies 11e */
            /* mismatch, so the packets should not be sent in    */
            /* other queues, to be taken care of by MAC S/w      */
        }
    }

    if(key_data_valid == 0x00)
    {
        /* Log security threat */
        /* Disconnect */
        /* Set the reason for disconnection and disconnect */
        rsna_ptr->deauth_reason = reason;
        rsna_auth_disconnect(rsna_ptr);
    }
    else
    {

        /* The PMK, SPA and ANonce are saved in the per STA RSNA Handle */
        pmk      = rsna_ptr->PMK;
        spa      = rsna_ptr->addr;
        anonce   = rsna_ptr->ANonce;

        /* 'key_length' specifies the length of the TK to be used. */
        /* The PTK length is initialized according to the cipher   */
        /* suite in use.                                           */
        ptk_size = (((rsna_ptr->key_length) + 32) * 8); /* Length in bits    */

        /* The auth address is saved as a global. It may be derived from MIB.*/
        aa       = mget_StationID();

        /* The SNonce needs to be extracted from the EAPOL Key frame received*/
        snonce = get_nonce(rsna_ptr->eapol_key_frame);

        /* Get PTK = CalcPTK(ANonce, SNonce) */
        get_ptk(pmk, rsna_ptr->PTK, ptk_size, aa, spa, anonce,
                snonce);

        /* Calculate and verify the received EAPOL MIC Field */
        rsna_ptr->MICVerified = verifyMIC(rsna_ptr->eapol_key_frame,
                                    rsna_ptr->eapol_frame_len,
                                    rsna_ptr->PTK,
                                    rsna_ptr->key_version);


        if(rsna_ptr->MICVerified == BTRUE)
        {
            /* Set State to A4_PTKCALCNEGOTIATING */
            rsna_ptr->rsna_auth_4way_state  = A4_PTKCALCNEGOTIATING;

        }
        else
        {
            mincr_RSNA4WayHandshakeFailures();
            /* Set the Disconect reason */
            rsna_ptr->deauth_reason = MIC_FAIL;
            /* Initiate a disconnect */
            rsna_auth_disconnect(rsna_ptr);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_ptkcalcnegotiating2                         */
/*                                                                           */
/*  Description      : This function implements the PTKCALCNEGOTIATING2      */
/*                     state of the per-sta 4-way key handshake state        */
/*                     machine.                                              */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_ptkcalcnegotiating2(rsna_auth_persta_t *rsna_ptr)
{
    rsna_ptr->TimeoutCtr = 0;

    /* Set State to A4_PTKCALCNEGOTIATING2 */
    rsna_ptr->rsna_auth_4way_state  = A4_PTKCALCNEGOTIATING2;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_ptkinitnegotiating                          */
/*                                                                           */
/*  Description      : This function implements the PTKINITNEGOTIATING state */
/*                     of the per-sta 4-way key handshake state machine.     */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_ptkinitnegotiating(rsna_auth_persta_t *rsna_ptr)
{
    UWORD8  pad           = 0;
    UWORD8  i             = 0;
    UWORD8  gtk_len       = 0;
    UWORD16 key_info      = 0;
    UWORD16 key_data_len  = 0;
    UWORD16 index         = 0;
    UWORD8  *key_rsc      = 0;
    UWORD8  *replay_cnt   = 0;
    UWORD16 eapol_size    = 0;
    UWORD8  *buffer       = 0;
    UWORD8  *eapol_buffer = 0;
    UWORD8  *kde          = 0;
    //UWORD8  *kck          = 0;
    UWORD8  *kek          = 0;
    UWORD8  *key_data     = 0;
    UWORD16 eapol_length  = 0;
    UWORD8  *key_iv       = 0;
    BOOL_T timer_started  = BFALSE;
    UWORD16 curr_idx      = 0;

    /* For RNSA 802 11i Group Replay Counter is to be loaded into this */
    UWORD8 key_rsc_val[RSC_SIZE] = {0};

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Reset Timeout Event */
    rsna_ptr->TimeoutEvt = BFALSE;


    /* Send EAPOL(1, 1, 1, Pair, P, RSC, ANonce, MIC(PTK), RSNIE, GTK[GN]) */

    /* Allocate buffers for the EAPOL Key Frame, Key Data and MIC.         */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* If general buffers are not available then take from management buffers */
    if(buffer == NULL)
    {
        buffer = mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);
    }

    if(buffer == NULL)
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnect(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    key_data     = (UWORD8 *)scratch_mem_alloc(128);
    if(key_data == NULL)
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnect(rsna_ptr);
        pkt_mem_free(buffer);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }
    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

    eapol_buffer = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;


    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
        if(rsna_ptr->pcip_policy != 0x00)
        {
            /* Prepare Key Information field with the required parameters  */
            key_info = prepare_key_info(rsna_ptr->key_version, 1, 1, 1, 1, P,
                                        0, 1, rsna_ptr->mode_802_11i);
        }
        else
        {
            /* Prepare Key Information field with the required parameters  */
            key_info = prepare_key_info(rsna_ptr->key_version, 1, 1, 1, 0, P,
                                        0, 1, rsna_ptr->mode_802_11i);
        }

        if( g_rsna_auth.global.GN > GTK_MAX_INDEX )
        {
            /* Set the deauth reason and initiate a disconnect */
            rsna_ptr->deauth_reason = UNSPEC_REASON;
            rsna_auth_disconnect(rsna_ptr);
            pkt_mem_free(buffer);

            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);

            return;
        }

        /* Prepare the data for the EAPOL key frame. The EAPOL Key Data */
        /* consists of the RSNIE and GTK KDE. The RSNIE and RSNIE length*/
        /* are saved in global variables. The GTK KDE is prepared.      */
        index = 0;

        /* Set the RSNIE */
        index += set_rsn_element_auth(key_data, 0, rsna_ptr->mode_802_11i);

        key_data_len += index;

        /* Prepare the GTK KDE. This is set after RSNIE in the same buffer.  */
        /* ----------------------------------------------------------------- */
        /* | Key ID  | Tx    | Reserved (0) | Reserved (0) | GTK           | */
        /* ----------------------------------------------------------------- */
        /* | Bit 0-1 | Bit 2 | Bit 3-7      | 1 octet      | Len - 6 octets| */
        /* ----------------------------------------------------------------- */
        kde = key_data + key_data_len;

        /* dot11RSNAConfigGroupCipherSize specifies the length of GTK to be  */
        /* used. This is initialized according to the cipher suite in use.   */
        gtk_len = mget_RSNAConfigGroupCipherSize() >> 3; /* Group size / 8   */

        /* Prepare the KDE */
        /* Encapsulate the GTK KDE in the required format */
        prepare_kde(GTK_KDE, (UWORD8)(gtk_len + 2), kde);

        kde[KDE_OFFSET]     = g_rsna_auth.global.GN; /* Key ID. */

        /* The Tx/Rx bit is set according to the cipher suite in use. If a   */
        /* pairwise key is used for encryption/integrity, then the STA never */
        /* transmits with the GTK; otherwise, STA uses the GTK for transmit. */
        /* Currently this is maintained as a global.                         */
        if(g_use_pairwisekey == BFALSE)
            kde[KDE_OFFSET] |= BIT2;

        kde[KDE_OFFSET + 1] = 0;
        memcpy(kde + KDE_OFFSET + 2,
            g_rsna_auth.global.GTK[get_GTK_array_index(g_rsna_auth.global.GN)], gtk_len);

        key_data_len += (KDE_OFFSET + gtk_len + 2);

        /* Obtain the KEK from PTK. This will be used in key data encryption.*/
        kek = get_kek(rsna_ptr->PTK);

        if(rsna_ptr->key_version == 1)
        {
            /* RC4 - is used as Key encryption algorigthm*/
            UWORD8 *rc4_ek = 0;

            rc4_ek = (UWORD8 *)scratch_mem_alloc(32);
            if(rc4_ek == NULL)
            {
                pkt_mem_free(buffer);
                rsna_ptr->deauth_reason = UNSPEC_REASON;
                rsna_auth_disconnect(rsna_ptr);
                return;
            }
            /* IV Field is derived from the Global counter                */
            key_iv = g_rsna_auth.global.Counter + NONCE_SIZE - 16;

            /* RC4 Key Encryption uses IV field and KEK together          */
            /*                 RC4 KEY - 32 Bytes                         */
            /*  16 bytes from Key counter (LSB) | 16 bytes KEK from TK    */
            memcpy(rc4_ek, key_iv, 16);
            memcpy(rc4_ek + 16, kek, 16);

            /* RC4 is used to encrypt the Key Data */
            rc4_skip(rc4_ek, 32, 256, (UWORD8 *)(key_data), key_data_len);
        }
        else if(rsna_ptr->key_version == 2)
        {
            /* The key data field is encrypted using encryption algorithm */
            /* in use. The length of the encrypted data is returned. The  */
            /* encryption should be in place.                             */
            if(key_data_len < 8)
            {
                pad = 16 - key_data_len;
            }
            else if(key_data_len % 8 != 0)
            {
                pad = 8 - key_data_len % 8;
            }

            /* Pad for data less than 64-bits or non-multiple of 64-bits */
            key_data[key_data_len++] = 0xDD;

            for(i = 0; i < pad-1; i++)
                key_data[key_data_len++] = 0x00;

            /* Perform AES Key wrap */
            aes_key_wrap(key_data, kek, key_data_len/8);

            /* The length after wrapping increses by 64-bits */
            key_data_len += 8;
        }

        /* Read the key_rsc value from the MAC HW CE register */
        get_machw_ce_pn_val(key_rsc_val);

        /* The Read value is the last used PN Value */
        /* This is incremented by one to set it to the next value to be used */
        incr_byte_cnt(key_rsc_val, 6);
        key_rsc = key_rsc_val;
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        if(rsna_ptr->pcip_policy != 0x00)
        {
            /* Prepare Key Information field with the required parameters.   */
            key_info = prepare_key_info(rsna_ptr->key_version, 0, 1, 1, 1, P,
                                        0, 0, rsna_ptr->mode_802_11i);
        }
        else
        {
            /* Prepare Key Information field with the required parameters.   */
            key_info = prepare_key_info(rsna_ptr->key_version, 0, 1, 1, 0, P,
                                        0, 0, rsna_ptr->mode_802_11i);
        }

        /* Prepare the data for the EAPOL key frame. EAPOL Key Data consists */
        /* of the WPAIE                                                      */
        index = 0;

        /* Set the RSNIE */
        index = set_rsn_element_auth(key_data, 0, rsna_ptr->mode_802_11i);
        key_data_len = index;

        /* The key_rsc value will be zero for WPA as GTK is not sent in  */
        /* this message                                                  */
        key_rsc    = NULL;

        /* The key_iv is useful only when RC4 is used. This will be zero */
        /* as WPA this message in not encrypted                          */
        key_iv     = NULL;
    }


    /* Get the replay count for this station */
    replay_cnt = rsna_ptr->ReplayCount;


    /* Prepare the EAPOL Key frame with the Key MIC field being zeroed.      */
    eapol_length = prepare_eapol_key_frame(key_info, rsna_ptr->key_length,
                (UWORD8 *)key_rsc, rsna_ptr->ANonce, (UWORD8 *)key_iv,
                key_data_len, key_data, (UWORD8 *)replay_cnt, eapol_buffer,
                rsna_ptr->mode_802_11i);

    incr_rply_cnt(rsna_ptr->ReplayCount);

    /* Set the EAPOL Key frame size */
    eapol_size = KEY_DATA_OFFSET + key_data_len;

    /* Obtain the KCK from the PTK. This will be used in the MIC computation.*/
    //kck = get_kck(rsna_ptr->PTK);


    /* Add SNAP Header and the EAPOL Header */
    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_KEY, eapol_length,
                   rsna_ptr->mode_802_11i);
    eapol_length += EAPOL_1X_HDR_LEN;

    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
        /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC   */
        /* field zeroed. The MIC is computed using the KCK derived from PTK. */
        if(rsna_ptr->key_version == 1)
        {
            /* Key counter is used as the IV, therefore increment the Key */
            /* counter                                                    */
            incr_cnt(g_rsna_auth.global.Counter, NONCE_SIZE);
        }
    }

    /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC  */
    /* field zeroed. The MIC is computed using the KCK derived from PTK.*/
    add_eapol_mic(eapol_buffer, eapol_size, rsna_ptr->PTK,
            rsna_ptr->key_version);

    rsna_ptr->TimeoutCtr++;

    /* Set State to A4_PTKINITNEGOTIATING */
    rsna_ptr->rsna_auth_4way_state  = A4_PTKINITNEGOTIATING;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(rsna_ptr->addr, buffer, eapol_length, BFALSE);

    /* Set timer since the response is expected for this packet */
    timer_started = start_11i_auth_timer(rsna_ptr, get_11i_resp_timeout((UWORD8)rsna_ptr->TimeoutCtr,
                    rsna_ptr->listen_interval));

    if(timer_started == BFALSE)
    {
        PRINTD("Could not send start timer after Unsecured EAPOL2\n");
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnect(rsna_ptr);
        
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);
        
        return;
    }
    
    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_ptkinitdone                                 */
/*                                                                           */
/*  Description      : This function implements the PTKINITDONE state of     */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_ptkinitdone(rsna_auth_persta_t *rsna_ptr)
{
    /* if Pair == TRUE                                                       */
    /* Send MLME-SetKeys.Request(0, Tx/Rx, PTK)                              */
    /* Send MLME-SetProtection.Request(TA, Tx/Rx, PTK)                       */
    /* MLMETBD */
    UWORD8 key_id        = 0;
    CIPHER_T cipher_type = NO_ENCRYP;

    rsna_ptr->TimeoutCtr     = 0;
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        rsna_ptr->keycount++;
        if(get_supp_key_count((UWORD8)rsna_ptr->key_index) >= 1)
        {
            if(NULL != rsna_ptr->sta_1x) {
	            ((auth_t *)(rsna_ptr->sta_1x))->global.portValid = BTRUE;
			} else {
			    /* Set the deauth reason and initiate a disconnect */
			    rsna_ptr->deauth_reason = INVALID_AKMP;
			    rsna_auth_disconnect(rsna_ptr);
			    printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
			    return;
			}

            /* Depending on the ciphers policies set the key ids and policies */
            key_id      = 0;
            cipher_type = cipsuite_to_ctype(rsna_ptr->pcip_policy);

            if(rsna_ptr->pcip_policy != 0x00)
            {
                /* Set keys for this STA */
                if(memcmp(mget_StationID(), rsna_ptr->addr, 6) > 0)
                {
                    machw_ce_add_key(key_id,
                                     PTK_NUM,
                                     rsna_ptr->key_index,
                                     0,
                                     cipher_type,
                                     (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH +
                                     KEK_LENGTH),
                                     rsna_ptr->addr,
                                     AUTH_KEY,
                                     (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH +
                                     KEK_LENGTH + TEMPORAL_KEY_LENGTH));
                }
            }

            /* Set the flags indicating completion of the state machine */
            if(rsna_ptr->mode_802_11i == RSNA_802_11I)
            {
            }
            else if(rsna_ptr->mode_802_11i == WPA_802_11I)
            {
                rsna_ptr->GInitAKeys = BTRUE;
                rsna_ptr->PInitAKeys = BTRUE;
            }
        }
    }
    else
    {
        /* Set the PortValid value to True */
        if(NULL != rsna_ptr->sta_1x) {
             ((auth_t *)(rsna_ptr->sta_1x))->global.portValid = BTRUE;
	    } else {
	         /* Set the deauth reason and initiate a disconnect */
	       rsna_ptr->deauth_reason = INVALID_AKMP;
	       rsna_auth_disconnect(rsna_ptr);
	       printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	       return;
	    }

        /* Depending on the ciphers policies set the key ids and policies */
        key_id      = 0;
        cipher_type = cipsuite_to_ctype(rsna_ptr->pcip_policy);

        /* Set keys for this STA */
        machw_ce_add_key(key_id,
                         PTK_NUM,
                         rsna_ptr->key_index,
                         0,
                         cipher_type,
                         (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH + KEK_LENGTH),
                         rsna_ptr->addr,
                         AUTH_KEY,
                         (UWORD8 *)(rsna_ptr->PTK + KCK_LENGTH + KEK_LENGTH +
                         TEMPORAL_KEY_LENGTH));

        /* Set the flags indicating completion of the state machine */
        if(rsna_ptr->mode_802_11i == RSNA_802_11I)
        {
            /* Set keyDone to TRUE */
            if(NULL != rsna_ptr->sta_1x) {
	             ((auth_t *)(rsna_ptr->sta_1x))->global.keyDone = BTRUE;
		    } else {
		         /* Set the deauth reason and initiate a disconnect */
		       rsna_ptr->deauth_reason = INVALID_AKMP;
		       rsna_auth_disconnect(rsna_ptr);
		       printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
		       return;
		    }
        }
        else if(rsna_ptr->mode_802_11i == WPA_802_11I)
        {
            rsna_ptr->GInitAKeys = BTRUE;
            rsna_ptr->PInitAKeys = BTRUE;
        }
    }

    if(NULL != rsna_ptr->sta_1x) {
	   ((auth_t *)(rsna_ptr->sta_1x))->global.keyDone = BTRUE;
	} else {
	    /* Set the deauth reason and initiate a disconnect */
	    rsna_ptr->deauth_reason = INVALID_AKMP;
	    rsna_auth_disconnect(rsna_ptr);
	    printk("%s rsna_ptr[%p]->sta_1x is NULL\n", __FUNCTION__, rsna_ptr);
	    return;
	}
    /* Set State to A4_PTKINITDONE */
    rsna_ptr->rsna_auth_4way_state  = A4_PTKINITDONE;

    /* Initiate a new RSNA Stats MIB */
    mnew_RSNAStats((UWORD8)rsna_ptr->key_index);
    mset_RSNAStatsSTAAddress(rsna_ptr->addr, (UWORD8)rsna_ptr->key_index);

    /* Create and set the cipher element in the RSNA Stats */
    mset_RSNAStatsSelectedPairwiseCipher(rsna_ptr->pcip_policy,
                                         (UWORD8)rsna_ptr->key_index);

    /* Update the MIB with the last selected PCIP and ATUH policies */
    mset_RSNAPairwiseCipherSelected(rsna_ptr->pcip_policy);
    mset_RSNAAuthenticationSuiteSelected(rsna_ptr->auth_policy);
    update_persist_cred_list_mode(rsna_ptr->addr);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_auth_ptkstart                                    */
/*                                                                           */
/*  Description      : This function implements the PTK_START state of       */
/*                     the per-sta 4-way key handshake state machine.        */
/*                                                                           */
/*  Inputs           : 1) Association ID                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_auth_ptkstart(rsna_auth_persta_t *rsna_ptr)
{
    UWORD16 key_info      = 0;
    UWORD16 curr_idx      = 0;
    UWORD8  *replay_cnt   = 0;
    UWORD8  *eapol_buffer = 0;
    UWORD8  *buffer       = 0;
    UWORD8  *spa          = 0;
    UWORD8  *pmk          = 0;
    UWORD8  *kde          = 0;
    UWORD16 kde_length    = 0;
    UWORD16 eapol_length  = 0;
    /* Message - 1 contains Key RSC and KEY IV as Zero */
    UWORD8  *key_rsc      = NULL;
    UWORD8  *key_iv       = NULL;
    BOOL_T timer_started  = BFALSE;
    
    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Send EAPOL(0, 0, 1, 0, P, 0, ANonce, 0, 0, 0)                         */

    /* Allocate buffers for the EAPOL Key Frame and MIC.                     */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* If general buffers are not available then take from management buffers */
    if(buffer == NULL)
    {
        buffer = mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);
    }

    if(buffer == NULL)
    {
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_FAIL;
        rsna_auth_disconnect(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    /* reset the buffer */
    mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);


    eapol_buffer = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;


    /* Prepare the Key Information field with the required parameters.       */
    key_info     = prepare_key_info(rsna_ptr->key_version, 0, 0, 1, 0, P,
                                    0, 0, rsna_ptr->mode_802_11i);

    /* Extract the address and the PMK maintained for the supplicant.        */
    spa = rsna_ptr->addr;
    pmk = rsna_ptr->PMK;

    if(rsna_ptr->mode_802_11i == RSNA_802_11I)
    {
        /* MSG 1 of RSNA contains PMKID Key data                             */
        /* Prepare the data for the EAPOL Key frame. Obtain the PMKID and set*/
        /* it in the PMKID Key Data Encapsulation (PMK KDE) at the required  */
        /* offset                                                          . */
        kde = (UWORD8 *)scratch_mem_alloc(PMKID_KDE_SIZE);
        if(kde == NULL)
        {
            pkt_mem_free(buffer);

            /* Set the deauth reason and initiate a disconnect */
            rsna_ptr->deauth_reason = UNSPEC_FAIL;
            rsna_auth_disconnect(rsna_ptr);

            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);

            return;
        }
        get_pmkid(pmk, mget_bssid(), spa, rsna_ptr->pmkid);
        memcpy(kde + KDE_OFFSET, rsna_ptr->pmkid, PMKID_LEN);
        prepare_kde(PMKID_KDE, PMKID_LEN, kde);
        kde_length = PMKID_KDE_SIZE;
    }
    else if(rsna_ptr->mode_802_11i == WPA_802_11I)
    {
        /* No Key-data is present in the message 1 of WPA */
        kde_length = 0;
    }

    /* Set State to A4_PTKSTART */
    rsna_ptr->rsna_auth_4way_state  = A4_PTKSTART;

    /* Get the replay count for this station */
    replay_cnt = rsna_ptr->ReplayCount;

    /* Prepare the EAPOL Key frame */
    eapol_length = prepare_eapol_key_frame(key_info, rsna_ptr->key_length,
            key_rsc, rsna_ptr->ANonce, key_iv, kde_length, kde,
            (UWORD8*)replay_cnt, eapol_buffer, rsna_ptr->mode_802_11i);

    incr_rply_cnt(rsna_ptr->ReplayCount);

    /* Set State to A4_PTKSTART */
    rsna_ptr->rsna_auth_4way_state  = A4_PTKSTART;

    prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_KEY, eapol_length,
                   rsna_ptr->mode_802_11i);

    eapol_length += EAPOL_1X_HDR_LEN;

    /* Increment Timeout Ctr here */
    rsna_ptr->TimeoutCtr++;

    /* Schedule the EAPOL frame for transmission */
    send_eapol(rsna_ptr->addr, buffer, eapol_length, BFALSE);

    /* Set timer since the response is expected for this packet */
    timer_started = start_11i_auth_timer(rsna_ptr,
                         get_11i_resp_timeout((UWORD8)rsna_ptr->TimeoutCtr,
                         rsna_ptr->listen_interval));

    if(timer_started == BFALSE)
    {
        PRINTD("Could not send start timer after Unsecured EAPOL2\n");
        /* Set the deauth reason and initiate a disconnect */
        rsna_ptr->deauth_reason = UNSPEC_REASON;
        rsna_auth_disconnect(rsna_ptr);

        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);        

        return;
    }

    /* Reset Timeout Event */
    rsna_ptr->TimeoutEvt = BFALSE;

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : STADisconnect                                         */
/*                                                                           */
/*  Description      : This procedure deauthenticates the STA.               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void STADisconnect(rsna_auth_persta_t *rsna_ptr)
{
    /* Set the disconnect flag, which will send the de-auth frame after the */
    /* FSM Run is completed                                                 */
    rsna_ptr->send_deauth = BTRUE;
}
#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
