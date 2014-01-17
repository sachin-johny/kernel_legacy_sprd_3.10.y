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
/*  File Name         : tkip.c                                               */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions                                            */
/*                                                                           */
/*  List of Functions : compute_b                                            */
/*                      compute_tkip_mic                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "tkip.h"
#include "maccontroller.h"
#include "iconfig.h"
#include "mac_init.h"
#include "controller_mode_if.h"
#include "rsna_auth_km.h"


#ifdef UTILS_11I

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  compute_tkip_mic                                        */
/*                                                                           */
/*  Description   :  This function computes Michael for TKIP encapsulation   */
/*                                                                           */
/*  Inputs        :  1) Pointer to the Destination address                   */
/*                   2) Pointer to the source address                        */
/*                   3) The priority of the frame (MSDU)                     */
/*                   4) Pointer to the data frame (MSDU)                     */
/*                   5) The size of the data frame (Pay load in MSDU)        */
/*                   6) Pointer to the MIC Computation key                   */
/*                   7) Pointer to the MIC Buffer                            */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    :  The MIC used for the TKIP security is computed is this  */
/*                   function. The function creates the MIC Computation unit */
/*                   from the DA, SA, priority and MSDU payload. The unit is */
/*                   zero padded with the appropriate number of zeroes and   */
/*                   the MIC is computed as specified in 802.11i sec 8.3.2.3 */
/*                                                                           */
/*  Outputs       :  The computed value of the MIC                           */
/*                                                                           */
/*  Returns       :  None                                                    */
/*                                                                           */
/*  Issues        :  None                                                    */
/*                                                                           */
/*****************************************************************************/

void compute_tkip_mic(UWORD8 *da, UWORD8* sa, UWORD8 priority,
                      buffer_desc_t  *frame_frags, UWORD8 num_frags,
                      UWORD16 frame_size, UWORD8 *mic_key, UWORD8 *mic)
{
    UWORD32 l,r,K0,K1,l1;
    UWORD16 lo = 0;
    UWORD16 hi = 0;
    UWORD8  pad = 4;
    UWORD8  i = 0;
    UWORD16 j = 0;
    UWORD8  *frag_ptr   = NULL;
    UWORD16 frag_len    = 0;
    UWORD8  prev_frag_process = 0;
    UWORD16 frag_len_word = 0;
    UWORD32 temp = 0;

    /* Input format for the MIC Computation                                  */
    /*-----------------------------------------------------------------------*/
    /*  DA(6)  |  SA (6) | Priority(1) | Zeroes (3) | Data (Padded)          */
    /*-----------------------------------------------------------------------*/
    /*                                                                       */

    /* Initialize K0 and K1 with the Mic Key */
    K0 = (mic_key[0]) | (mic_key[1] << 8) |
         (mic_key[2] << 16) | (mic_key[3] << 24);
    K1 = (mic_key[4]) | (mic_key[5] << 8) |
         (mic_key[6] << 16) | (mic_key[7] << 24);

    /* Compute the pointer to the end of the frame */
    i = num_frags - 1;
    frag_ptr = frame_frags[i].buff_hdl + frame_frags[i].data_offset +
                    frame_frags[i].data_length;

    /* Append data with 0x5A as required by 802.11i standard prior to MIC */
    /* Computation                                                        */
    *frag_ptr = 0x5A;
    frame_size += 1;
    frag_ptr++;

    /* Compute the Pad Length */
    if(frame_size % 4)
        pad += (4 - (frame_size % 4));

    /* Append the pad bytes */
    mem_set(frag_ptr, 0, pad);

    l = K0;
    r = K1;

    /* First Run the function 'b' on DA, SA, priority and zero pads */
    /* Run on DA */
    l1 =  (da[0]) | (da[1] << 8) | (da[2] << 16) | (da[3] << 24);
    l = l ^ l1;
    compute_b(&l, &r);
    /* Run on DA and SA */
    l1 =  (da[4]) | (da[5] << 8) | (sa[0] << 16) | (sa[1] << 24);
    l = l ^ l1;
    compute_b(&l, &r);
    /* Run on DA and SA */
    l1 =  (sa[2]) | (sa[3] << 8) | (sa[4] << 16) | (sa[5] << 24);
    l = l ^ l1;
    compute_b(&l, &r);
    /* Run on Priority and Zero Pads */
    l1 =  (priority & 0x0F);
    l = l ^ l1;
    compute_b(&l, &r);

    /* The value 'b' is computed on every Data word of the data     */
    /* The algorithm is as follows                                  */
    /*                                                              */
    /* l1 = (data[i]) | (data[i + 1] << 8) |                        */
    /*      (data[i + 2] << 16) | (data[i + 3] << 24);              */
    /* l = l ^ l1                                                   */
    /* compute_b(l , r)                                             */

    for(i = 0; i < num_frags; i++)
    {
        frag_ptr = frame_frags[i].buff_hdl + frame_frags[i].data_offset;
        frag_len = frame_frags[i].data_length;
        /* Adjust the length of the last fragment for the pad bytes */
        if(i == num_frags - 1)
            frag_len += pad + 1;

        /* Check for residual processing from the previous fragment */
        if(prev_frag_process)
        {
            hi = convert_to_le_w16(*(UWORD16 *)frag_ptr);
            l ^= ((hi << 16) | (lo));
            compute_b(&l, &r);

            prev_frag_process = 0;
            frag_ptr += sizeof(UWORD16);
            frag_len -= 2;
        }

        /* Check the alignment of the fragment pointer */
        if(0 == ((UWORD32)frag_ptr & 3))
        {
            /* 32-bit aligned processing */
            frag_len_word = frag_len >> 2;
            for(j = 0; j < frag_len_word; j++)
            {
                 temp  = *((UWORD32 *)(frag_ptr) + j);
                l    ^= convert_to_le(temp);
                compute_b(&l, &r);
            }
            j <<= 2;
        }
        else
        {
            /* 16-bit aligned processing */
            frag_len_word = (frag_len >> 1) & 0xFFFE;
            for(j = 0; j < frag_len_word; j += 2)
            {
                lo = convert_to_le_w16(*((UWORD16 *)(frag_ptr) + j));
                hi = convert_to_le_w16(*((UWORD16 *)(frag_ptr) + j + 1));
                l ^= ((hi << 16) | (lo));
                compute_b(&l, &r);
            }
            j <<= 1;
        }

        /* Store the extra bytes for processing along with the next fragment */
        if(j != frag_len)
        {
            lo = convert_to_le_w16(*(UWORD16 *)(frag_ptr + j));
            prev_frag_process = 1;
        }
    }

    mic[0] =  l        & 0xFF;
    mic[1] = (l >> 8)  & 0xFF;
    mic[2] = (l >> 16) & 0xFF;
    mic[3] = (l >> 24) & 0xFF;
    mic[4] =  r        & 0xFF;
    mic[5] = (r >> 8)  & 0xFF;
    mic[6] = (r >> 16) & 0xFF;
    mic[7] = (r >> 24) & 0xFF;
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */
