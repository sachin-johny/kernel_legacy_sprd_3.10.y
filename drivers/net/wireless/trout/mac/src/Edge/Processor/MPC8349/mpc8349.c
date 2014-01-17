/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                        COPYRIGHT(C) 2008                                  */
/*                                                                           */
/*  This program is proprietary to Ittiam Systems Pvt. Ltd. and is protected */
/*  under Indian Copyright Law as an unpublished work. Its use and           */
/*  disclosure is limited by the terms and conditions of a license agreement.*/
/*  It may not be copied or otherwise reproduced or disclosed to persons     */
/*  outside the licensee's organization except in accordance with the terms  */
/*  and conditions of such an agreement. All copies and reproductions shall  */
/*  be the property of Ittiam Systems India Pvt. Ltd. and must bear this     */
/*  notice in its entirety.                                                  */
/*                                                                           */
/*  File Name         : mpc8349.c                                            */
/*                                                                           */
/*  Description       : This file contains all functions specific to         */
/*                      Motorolla PowerPC MPC8349 processor.                 */
/*                                                                           */
/*  List of Functions :                                                      */
/*                                                                           */
/*  Issues            : NONE                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MWLAN

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mpc8349.h"
#include "spi_interface.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : do_chan0_dma                                          */
/*                                                                           */
/*  Description      : This function does a DMA transfer from the specified  */
/*                     source to destination. This function expects Physical */
/*                     address to be passed to it. Although there are no     */
/*                     buffer alignment requirements, for best performance,  */
/*                     the source and destination addresses should be aligned*/
/*                     to 32 byte boundary.                                  */
/*                                                                           */
/*  Inputs           : dstp - (Physical) Destination Address                 */
/*                     srcp - (Physical) Source Address                      */
/*                     len  - Length of the buffer                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : 0=>Success 1=>Failure                                 */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/
#if 1
//hugh
UWORD8 do_chan0_dma(void *dstp, void *srcp, UWORD32 len)
{
    return 0;
}
#else
UWORD8 do_chan0_dma(void *dstp, void *srcp, UWORD32 len)
{
    UWORD8 retval = 0;

    /* Wait till the DMA channel becomes free */

    //chenq mod
    //while(rDMA0_STATUS & SWAP_BYTE_ORDER_WORD(BIT2));
    while(host_read_trout_reg( 
            (UWORD32)rDMA0_STATUS) & SWAP_BYTE_ORDER_WORD(BIT2));

    /* Reset any previous DMA transfer error */

    //chenq mod
    //rDMA0_STATUS |= SWAP_BYTE_ORDER_WORD(BIT7);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rDMA0_STATUS ) 
            | (SWAP_BYTE_ORDER_WORD(BIT7)) ,
        (UWORD32)rDMA0_STATUS );

    /* The Direct Channel transfer mode is set. Snooping of direct mode DMA  */
    /* transactions is enabled to maintain cache coherency.                  */

    //chenq mod
    //rDMA0_MODE = SWAP_BYTE_ORDER_WORD(DMA_CFG_WORD);
    host_write_trout_reg( SWAP_BYTE_ORDER_WORD(DMA_CFG_WORD), 
                          (UWORD32)rDMA0_MODE );

    /* Set the source, destination and length registers. */

    //chenq mod
    //rDMA0_SADDR   = SWAP_BYTE_ORDER_WORD((UWORD32)srcp);
    //rDMA0_DADDR   = SWAP_BYTE_ORDER_WORD((UWORD32)dstp);
    //rDMA0_BYTECNT = SWAP_BYTE_ORDER_WORD(len);
    host_write_trout_reg( SWAP_BYTE_ORDER_WORD((UWORD32)srcp), 
					(UWORD32)rDMA0_SADDR );
    host_write_trout_reg( SWAP_BYTE_ORDER_WORD((UWORD32)dstp), 
					(UWORD32)rDMA0_DADDR );
    host_write_trout_reg( SWAP_BYTE_ORDER_WORD(len), 
					(UWORD32)rDMA0_BYTECNT );

    /* Trigger the DMA */
    //chenq mod
    //rDMA0_MODE &= ~SWAP_BYTE_ORDER_WORD(BIT0);
    //rDMA0_MODE |= SWAP_BYTE_ORDER_WORD(BIT0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rDMA0_MODE ) 
            & (~SWAP_BYTE_ORDER_WORD(BIT0)) ,
        (UWORD32)rDMA0_MODE );
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rDMA0_MODE ) 
            | (SWAP_BYTE_ORDER_WORD(BIT0)) ,
        (UWORD32)rDMA0_MODE );

    /* Wait for the transfer to complete */

    //chenq mod
    //while(rDMA0_STATUS & SWAP_BYTE_ORDER_WORD(BIT2));
    while(host_read_trout_reg( (UWORD32)rDMA0_STATUS)
            & SWAP_BYTE_ORDER_WORD(BIT2));

    /* Check for error condition */
    //chenq mod
    //if(rDMA0_STATUS & SWAP_BYTE_ORDER_WORD(BIT7))
    if(host_read_trout_reg( (UWORD32)rDMA0_STATUS) & SWAP_BYTE_ORDER_WORD(BIT7))
        retval = 1;

    return retval;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : set_led_pat                                           */
/*                                                                           */
/*  Description      : This function sets LEDs to display the binary         */
/*                     equivalent of the input number.                       */
/*                                                                           */
/*  Inputs           : Number (0-31)                                         */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void set_led_pat(UWORD8 num)
{
//masked by Hugh
//  UWORD32 all_pat = (PROC_LED_1 | PROC_LED_2 | PROC_LED_3 | PROC_LED_4 | PROC_LED_5);
    UWORD32 req_pat = 0;

    if(num & BIT0)
        req_pat |= PROC_LED_1;

    if(num & BIT1)
        req_pat |= PROC_LED_2;

    if(num & BIT2)
        req_pat |= PROC_LED_3;

    if(num & BIT3)
        req_pat |= PROC_LED_4;

    if(num & BIT4)
        req_pat |= PROC_LED_5;

    //chenq mod
    //rPROC_GPIO1_DAT = (rPROC_GPIO1_DAT & ~all_pat) | (~req_pat);

//masked by Hugh
/*
    host_write_trout_reg(  
        (host_read_trout_reg( (UWORD32)rPROC_GPIO1_DAT ) & ~all_pat) 
                                | (~req_pat) , 
        (UWORD32)rPROC_GPIO1_DAT );
   */
}
#endif /* MWLAN */
