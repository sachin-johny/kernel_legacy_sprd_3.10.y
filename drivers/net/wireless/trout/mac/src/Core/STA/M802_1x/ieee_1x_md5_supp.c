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
/*  File Name         : ieee_1x_reauth.c                                     */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of Reauthenticator state  */
/*                      machine according to IEEE 802.1x-REV/D9.             */
/*                                                                           */
/*  List of Functions : reauthenticator_fsm                                  */
/*                      reauth_initialize                                    */
/*                      reauth_reauthenticate                                */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "imem_if.h"
#include "md5.h"
#include "ieee_supp_1x.h"
#include "ieee_1x_md5_supp.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_eapmd5                                           */
/*                                                                           */
/*  Description      : This function initializes MD5 parameters and memory   */
/*                     during the 1x initialization                          */
/*                                                                           */
/*  Inputs           : NONE                                                  */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void init_eapmd5(void)
{
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : eapmd5_shutdown                                       */
/*                                                                           */
/*  Description      : This function decodes a MD5 message from the AS       */
/*                                                                           */
/*  Inputs           : 1) Pointer to the incoming EAP Message                */
/*                     2) Pointer to the outgoing response                   */
/*                     3) Size of the outgoing EAP Message                   */
/*                     4) The received ID of the request                     */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*                                                                           */
/*  Processing       : When an MD5 challenge is sent in, the header, up to   */
/*                     the actual challenge string is stripped off before we */
/*                     get here.  Since the ID is needed to generate the     */
/*                     return md5 hash, we need to have it passed in too.    */
/*                     The authenticator sends us a random stream of bytes.  */
/*                     The first byte in this stream is the length of the    */
/*                     stream.  From this, we build our response by          */
/*                     building a string that contains ID + password +       */
/*                     challenge stream, and then MD5 it.                    */
/*                     The resulting value, with our username concatinated to*/
/*                     the end is returned in the response packet.           */
/*                                                                           */
/*  Outputs          : NONE                                                  */
/*                                                                           */
/*  Returns          : BTRUE on sucess; BFALSE Otherwise                     */
/*                                                                           */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T eapmd5_decode_packet(UWORD8 *eap_ptr, UWORD8 *out_ptr,
                            UWORD16 *out_size, UWORD16 receivedId)
{
    UWORD8  challen        = 0;
    UWORD16 hashlen        = 0;
    UWORD16 curr_idx       = 0;
    UWORD8  *chal          = NULL;
    UWORD8  md5_result[16] = {0};
    md5_state_t context    = {{0},};
    UWORD8 *username       = &g_supp_username[1];
    UWORD8 *password       = &g_supp_password[1];
    UWORD8 pwd_len         = g_supp_password[0];
    UWORD8 username_len    = g_supp_username[0];
    UWORD8 *in             = eap_ptr + 5;  // Strip EAPOL Header

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    challen = in[0];
#ifndef ENABLE_SCRATCH_MEM_ESTIMATION
    chal = (UWORD8 *)scratch_mem_alloc(challen);
#else /* ENABLE_SCRATCH_MEM_ESTIMATION */
    chal = (UWORD8 *)scratch_mem_alloc(256);
#endif /* ENABLE_SCRATCH_MEM_ESTIMATION */
    if (chal == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    memcpy(chal, &in[1], challen);   // We should have our hash info

    out_ptr[0] = receivedId;
    memcpy(&out_ptr[1], password, pwd_len);
    memcpy(&out_ptr[1 + pwd_len], chal, challen);
    hashlen = 1 + pwd_len + challen;

    md5_init(&context);                //Initialize MD5
    md5_append(&context, out_ptr, hashlen);
    md5_finish(&context, md5_result);

    chal = NULL;

    /* Probably shouldn't reuse the same variable.... */
    /* We need to malloc 1 for the length byte,       */
    /* 16 for the MD5 string,                         */
    /* strlen(username) for the username,             */
    /* and 1 for a NULL byte for good measure. 8-)    */
    out_ptr[0] = 0x10;
    memcpy(&out_ptr[1], &md5_result, 16);
    memcpy(&out_ptr[17], username, username_len);
    *out_size = 17 + username_len;

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : eapmd5_auth_setup                                     */
/*                                                                           */
/*  Description      : This function initializes the MD5 parameters for a    */
/*                     new MD5 connection                                    */
/*                                                                           */
/*  Inputs           : 1) Handle to the 1x Supplicant                        */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*                                                                           */
/*  Returns          : BTRUE                                                 */
/*                                                                           */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T eapmd5_auth_setup(supp_t *supp_1x)
{
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : eapmd5_shutdown                                       */
/*                                                                           */
/*  Description      : This function cleans up the MD5 parameters and memory */
/*                     during the shutdown                                   */
/*                                                                           */
/*  Inputs           : NONE                                                  */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void eapmd5_shutdown(void)
{
    return;
}

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */

