/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wps_key.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions to generate the     */
/*                      Authentication and Key Wrap Key                      */
/*                                                                           */
/*  List of Functions : wps_kdf                                              */
/*                      wps_gen_keys                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         24 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_key.h"
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static void wps_kdf(UWORD8* inp_key, UWORD8* pers_str, UWORD8 pers_str_len,
                 UWORD8* out_key, UWORD32 total_key_bytes);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_kdf                                               */
/*                                                                           */
/*  Description      : This function implements the Key Derivation function  */
/*                     as described in WPS standard version v.1.2.2          */
/*                                                                           */
/*  Inputs           : 1) Input Key used for key hashing                     */
/*                     2) Pointer to personalization string                  */
/*                     3) Length of personalization string                   */
/*                     4) Pointer to location where output key needs to be   */
/*                        stored. The output key buffer should be long       */
/*                        enough to store ((Total Output key bytes + SHA256  */
/*                        hash length -1)/ SHA256 hash length) of bytes i.e  */
/*                        this module output bytes in multiple of SHA256     */
/*                        hash length. Calling function should take care of  */
/*                        selecting appropriate number of bytes              */
/*                     5) Required length in bytes of the output key         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function implements the following algorithm      */
/*                     kdf(key, personalization_string, total_key_bits) :    */
/*                       result := ""                                        */
/*                       iterations = (total_key_bits+prf_digest_size-1)/    */
/*                                    prf_digest_size                        */
/*                       for i = 1 to iterations do                          */
/*                         result = result || prf(key, i ||                  */
/*                                  personalization_string || total_key_bits)*/
/*                       return result                                       */
/*                     The function prf is the keyed hash HMAC-SHA-256       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Key bits generating using KDF function                */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         24 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_kdf(UWORD8* inp_key, UWORD8* pers_str, UWORD8 pers_str_len,
                 UWORD8* out_key, UWORD32 total_key_bytes)
{
    UWORD8 *msg[3];
    UWORD8  iter_buff[4], total_key_bits_buff[4];
    UWORD16 msg_len[3];
    UWORD32 iter, max_iter;

    /*************************************************************************/
    /* use iter as temporary variable to store total number of bits          */
    /*************************************************************************/
    iter = total_key_bytes * 8;
    max_iter = (total_key_bytes + SHA_256_HASH_LEN - 1) / SHA_256_HASH_LEN;

    total_key_bits_buff[3] = (UWORD8)(iter);
    total_key_bits_buff[2] = (UWORD8)(iter >> 8);
    total_key_bits_buff[1] = (UWORD8)(iter >> 16);
    total_key_bits_buff[0] = (UWORD8)(iter >> 24);

    msg[0] = iter_buff;
    msg_len[0] = sizeof(iter_buff);
    msg[1] = pers_str;
    msg_len[1] = pers_str_len;
    msg[2] = total_key_bits_buff;
    msg_len[2] = sizeof(total_key_bits_buff);

    /*************************************************************************/
    /* This function will generates bytes in multiples of SHA_256_HASH_LEN   */
    /* calling function to ensure that the out_key has enough space          */
    /*************************************************************************/
    for (iter = 1; iter <= max_iter; ++iter)
    {
        iter_buff[3] = (UWORD8)(iter);
        iter_buff[2] = (UWORD8)(iter >> 8);
        iter_buff[1] = (UWORD8)(iter >> 16);
        iter_buff[0] = (UWORD8)(iter >> 24);
        hmac_sha256(inp_key, SHA_256_HASH_LEN, msg, msg_len,3, out_key);
        out_key += SHA_256_HASH_LEN;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_keys                                          */
/*                                                                           */
/*  Description      : This function generates Diffie-Hellman shared secret  */
/*                     key. This key needs to be hashed before it can be used*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure where DH remote Public Key*/
/*                        is stored                                          */
/*                     2) Pointer to the structure where DH Private Key  is  */
/*                        stored                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Using the remote public key and local private key this*/
/*                     functions generate the shared secret key  =           */
/*                     = (pubic key ^private Key) mod p) where g =2, p is    */
/*                     predefined prime. It then  converts the secret key in */
/*                     byte format. Public key, Private key, secret key and  */
/*                     Prime are all of  length  WPS_DH_PRIME_LEN_WORD32.    */
/*                                                                           */
/*  Outputs          : Authentication Key and Key Wrap key                   */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_gen_keys(wps_enrollee_t *wps_enrollee)
{
    UWORD8 *msg[3];
    UWORD8 *temp_ptr;
    UWORD16 msg_len[3];
    const UWORD8  pers_str[] = "Wi-Fi Easy and Secure Key Derivation";

    temp_ptr = (UWORD8 *) wps_enrollee->pub_key.pkr_u32;
    /*************************************************************************/
    /* Generate pre-hash DH Key = (pkr ^ A nonce) mod p                      */
    /* DH key will be stored in byte format in place of PKR                  */
    /*************************************************************************/
    if(FAILURE == wps_gen_ph_dh_key((UWORD32 *) temp_ptr,
        &(wps_enrollee->a_nonce)))
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Generate DH Key = sha256((pkr ^ A nonce) mod p)                       */
    /* Store DH key in place of pre-hash DH key                              */
    /*************************************************************************/
    msg[0] = temp_ptr;
    msg_len[0] = WPS_DH_PRIME_LEN_BYTE;
    sha256(msg,msg_len,1,temp_ptr);
    /*************************************************************************/
    /* Generate KDK = HMAC-SHA256(DHKey) (N1 || EnrolleeMAC || N2)           */
    /* Store KDK in place of DH key                                          */
    /*************************************************************************/
    msg[0] = wps_enrollee->enr_nonce;
    msg[1] = wps_enrollee->config_ptr->mac_address;
    msg[2] = wps_enrollee->reg_nonce;
    msg_len[0] = WPS_NONCE_LEN;
    msg_len[1] = WPS_MAC_ADDR_LEN;
    msg_len[2] = WPS_NONCE_LEN;
    hmac_sha256(temp_ptr, SHA_256_HASH_LEN, msg, msg_len,
        3, temp_ptr);
    /*************************************************************************/
    /* Use key derivation function to generate AuthKey and Key Wrap key using*/
    /* key derivation function key.                                          */
    /*************************************************************************/
    wps_kdf(temp_ptr, (UWORD8 *)pers_str,(UWORD8)(sizeof(pers_str) -1),
        temp_ptr + SHA_256_HASH_LEN,
        WPS_AUTH_KEY_LEN + WPS_KEY_WRAP_KEY_LEN + WPS_EMSK_LEN);
    /*************************************************************************/
    /* Copy Auth Key  and Key wrap key to the Enrollee persistent memory     */
    /*************************************************************************/
    memcpy(wps_enrollee->auth_key, temp_ptr + SHA_256_HASH_LEN,
        WPS_AUTH_KEY_LEN);

    memcpy(wps_enrollee->key_wrap_key,
        temp_ptr + SHA_256_HASH_LEN + WPS_AUTH_KEY_LEN,
        WPS_KEY_WRAP_KEY_LEN);
    return SUCCESS;
}
#endif /* INT_WPS_ENR_SUPP */
