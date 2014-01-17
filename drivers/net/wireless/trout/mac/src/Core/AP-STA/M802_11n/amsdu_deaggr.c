/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : amsdu_deaggr.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions related to      */
/*                      deaggregation of AMSDU frames.                       */
/*                                                                           */
/*  List of Functions : deaggr_create_msdu_desc                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "buff_desc.h"
#include "imem_if.h"
#include "amsdu_deaggr.h"
#include "receive.h"
#include "host_if.h"
#include "prot_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : deaggr_create_msdu_desc                                  */
/*                                                                           */
/*  Description   : This function handles the deaggregation of the A-MSDU    */
/*                  frame and extracts the next sub-MSDU.                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WLAN frame RX structure.               */
/*                  2) Pointer to the MSDU Indicate strcuture.               */
/*                  3) Pointer to the MSDU Process State Stucture            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The A-MSDU frame is processed to extract the next sub    */
/*                  MSDU. The MSDU indicate structure is updated with the    */
/*                  details of the same. The current state of A-MSDU         */
/*                  processing is stored in the MSDU process state structure */
/*                  and should be initialized to 0 before this function is   */
/*                  called for the first time. It is then updated and        */
/*                  maintained based on the processing done in this function */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : MSDU_PROC_STATUS_T, Status of MSDU processing.           */
/*                  PROC_ERROR if there is a processing error                */
/*                  PROC_MORE_MSDU if there are more sub MSDUs to process    */
/*                  PROC_LAST_MSDU if A-MSDU processing is complete          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

MSDU_PROC_STATUS_T deaggr_create_msdu_desc(void *wlan_rx_struct,
                                           void *msdu_struct,
                                           void *msdu_state_struct)
{
    wlan_rx_t         *wlan_rx    = (wlan_rx_t *)wlan_rx_struct;
    msdu_indicate_t   *msdu       = (msdu_indicate_t *)msdu_struct;
    msdu_proc_state_t *msdu_state = (msdu_proc_state_t *)msdu_state_struct;

    msdu_desc_t *frame_desc    = 0;
    UWORD32     *dscr          = 0;
    UWORD32     *buffer_addr   = 0;
    UWORD16     offset         = 0;
    UWORD16     submsdu_len    = 0;
    UWORD8      *submsdu_hdr   = NULL;
    UWORD8      submsdu_padlen = 0;

    if(0 == is_amsdu_frame(wlan_rx->msa))
    {
        return create_msdu_desc(wlan_rx, msdu);
    }

    frame_desc = &(msdu->frame_desc);

    /* Initialize the MSDU processing state at the beginning for the first   */
    /* time this function is called                                          */

    /* If the total number of descriptors and the current number of MSDUs    */
    /* processed are both 0 it indicates that this is the first time the     */
    /* function has been called. Update the MSDU processing state structure  */
    /* with the required information in this case.                           */
    if((msdu_state->tot_num_dscr_proc == 0) &&
       (msdu_state->curr_num_msdu_proc == 0))
    {
        /* Get the base descriptor from the WLAN Rx structure */
        dscr = wlan_rx->base_dscr;

        /* Update MSDU processing state with information of this descriptor */
        msdu_state->curr_dscr     = dscr;
        msdu_state->curr_buffer   = (UWORD32 *)get_rx_dscr_buffer_ptr(dscr);
        msdu_state->curr_num_msdu = get_rx_dscr_num_msdus(dscr);
        msdu_state->curr_offset   = wlan_rx->hdr_len + HOST_HEADER_GAP;
    }

    /* Extract required information from the Sub MSDU as pointed by the      */
    /* current buffer and offset saved in the MSDU processing state          */

    /* Extract the Sub MSDU header pointer */
    buffer_addr = msdu_state->curr_buffer;
    offset      = msdu_state->curr_offset;
    submsdu_hdr = (UWORD8 *)buffer_addr + offset;

    /* Extract various fields from the Sub MSDU header and update the MSDU */
    submsdu_len = get_submsdu_len(submsdu_hdr);
    submsdu_padlen = get_submsdu_pad_len(SUB_MSDU_HEADER_LENGTH + submsdu_len);
    get_submsdu_sa(submsdu_hdr, msdu->sa);
    get_submsdu_da(submsdu_hdr, msdu->da);

    /* Update the frame descriptor with required information                 */

    /* Set the information related to the Sub MSDU payload buffers. */
    frame_desc->buffer_addr = (UWORD8 *)buffer_addr;
    frame_desc->data_len    = submsdu_len;
    frame_desc->data_offset = offset + SUB_MSDU_HEADER_LENGTH;

    /* Host header depends upon the specific host interface and is hence not */
    /* updated here.                                                         */
    frame_desc->host_hdr_len    = 0;
    frame_desc->host_hdr_offset = 0;

    /* Update the MSDU processing state with required information            */

    /* Increment current number of MSDUs processed */
    msdu_state->curr_num_msdu_proc++;

    /* If the current number of MSDUs processed is less than the number of   */
    /* MSDUs present in the buffer continue processing the same buffer. In   */
    /* this case update the current offset only.                             */
    if(msdu_state->curr_num_msdu_proc < msdu_state->curr_num_msdu)
    {
        msdu_state->curr_offset += submsdu_len + submsdu_padlen +
                                   SUB_MSDU_HEADER_LENGTH + HOST_HEADER_GAP;
    }
    /* If the current number of MSDUs processed is equal to the number of    */
    /* MSDUs present in the buffer increment the total number of descriptors */
    /* processed count. Compare this count with the number of descriptors    */
    /* indicated by the WLAN Rx structure and process based on this.         */
    else if(msdu_state->curr_num_msdu_proc == msdu_state->curr_num_msdu)
    {
        UWORD8 total_num_dscr = wlan_rx->num_dscr;

        msdu_state->tot_num_dscr_proc++;

        /* If the total number of buffers processed is less than the total   */
        /* number of descriptors continue processing on the next descriptor. */
        if(msdu_state->tot_num_dscr_proc < total_num_dscr)
        {
            /* Get the next descriptor from the current descriptor */
            dscr = (UWORD32 *)get_rx_dscr_next_dscr(msdu_state->curr_dscr);

            /* Update MSDU processing state with info of this descriptor */
            msdu_state->curr_dscr    = dscr;
            msdu_state->curr_buffer  = (UWORD32 *)get_rx_dscr_buffer_ptr(dscr);
            msdu_state->curr_num_msdu = get_rx_dscr_num_msdus(dscr);
            msdu_state->curr_offset   = HOST_HEADER_GAP;

            /* Reset the current number of MSDUs processed count */
            msdu_state->curr_num_msdu_proc = 0;
        }
        else if(msdu_state->tot_num_dscr_proc == total_num_dscr)
        {
            /* If total number of descriptors have been processed this is    */
            /* the last sub MSDU. Return appropriate status to indicate this */
            return PROC_LAST_MSDU;
        }
        else
        {
            /* Exception case: Total number of descriptors processed exceeds */
            /* the number of descriptors indicated by the WLAN Rx structure  */
            return PROC_ERROR;
        }
    }
    else
    {
        /* Exception case: Total number of MSDU processed in a single buffer */
        /* exceeds the number of MSDU as indicated in the Rx descriptor      */
        return PROC_ERROR;
    }

   return PROC_MORE_MSDU;
}

#endif /* MAC_802_11N */
