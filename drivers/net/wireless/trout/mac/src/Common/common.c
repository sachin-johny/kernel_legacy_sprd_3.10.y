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
/*  File Name         : common.c                                             */
/*                                                                           */
/*  Description       : This file contains the functions used by both AP/STA */
/*                      modes in MAC.                                        */
/*                                                                           */
/*  List of Functions : set_dscr_fn                                          */
/*                      get_dscr_fn                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mh.h"
#include "csl_linux.h"
#include "src/Common/trout_share_mem.h"

#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD32 g_calib_cnt = DEFAULT_CALIB_COUNT;
extern UWORD32 g_done_wifi_suspend;
#ifdef DEBUG_MODE
mac_stats_t g_mac_stats     = {0};
reset_stats_t g_reset_stats = {0};
UWORD8 g_enable_debug_print = 1;
UWORD8 g_11n_print_stats    = 0;
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
mem_stats_t g_mem_stats = {0};
#endif /* MEM_DEBUG_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef DSCR_MACROS_NOT_DEFINED
/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_dscr_fn                                              */
/*                                                                           */
/*  Description   : This function modifies the packet descriptor with the    */
/*                  new specified value. The descriptor field to be modified */
/*                  is specified by the descriptor offset and width          */
/*                                                                           */
/*  Inputs        : 1) Offset of the descriptor field                        */
/*                  2) Width of the descriptor field                         */
/*                  3) Pointer to the packet descriptor to be modified       */
/*                  4) The new value for the descriptor field                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Modifies the specified descriptor with the supplied      */
/*                  value.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_dscr_fn(UWORD8 offset, UWORD16 width, UWORD32 *ptr, UWORD32 value)
{
    UWORD32 mask_inverse = 0;
    UWORD32 mask         = 0;
    UWORD32 temp         = 0;
    UWORD32 shift_offset = 32 - width - offset;

#ifdef DEBUG_MODE
    if((width + offset) > 32)
    {
        /* Signal Erroneous input */
    }

#endif /* DEBUG_MODE */

    /* Calculate the inverse of the Mask */
    if(width < 32)
        mask_inverse = ((1 << width) - 1) << shift_offset;
    else
        mask_inverse = 0xFFFFFFFF;

    /* Generate the mask */
    mask = ~mask_inverse;

    /* Read the descriptor word in little endian format */
    temp = convert_to_le(*ptr);

    /* Updating the value of the descriptor field with the help of masks */
    temp = ((value << shift_offset) & mask_inverse) | (temp & mask);

    /* Swap the byte order in the word if required for endian-ness change */
    *ptr = convert_to_le(temp);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_dscr_fn                                              */
/*                                                                           */
/*  Description   : This function reads a word32 location to extract a       */
/*                  specified descriptor field.of specified width.           */
/*                                                                           */
/*  Inputs        : 1) Offset of the descriptor field in the 32 bit boundary */
/*                  2) Width of the desired descriptor field                 */
/*                  3) Pointer to the word32 location of the field           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Reads the descriptor for the specific field and returns  */
/*                  the value.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Descriptor field value                                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 get_dscr_fn(UWORD8 offset, UWORD16 width, UWORD32 *ptr)
{
    UWORD32 mask         = 0;
    UWORD32 temp         = 0;
    UWORD32 value        = 0;
    UWORD32 shift_offset = 32 - width - offset;

#ifdef DEBUG_MODE
    if((width + offset) > 32)
    {
        /* Signal Erroneous input */
    }

#endif /* DEBUG_MODE */

    /* Calculate the Mask */
    if(width < 32)
        mask = ((1 << width) - 1) << shift_offset;
    else
        mask = 0xFFFFFFFF;

    /* Swap the byte order in the word if required for endian-ness change */
    temp = convert_to_le(*ptr);

    /* Obtain the value of the descriptor field with the help of masks */
    value = (temp & mask) >> shift_offset;

    return value;
}
#endif /* DSCR_MACROS_NOT_DEFINED */

#ifdef MWLAN
/*****************************************************************************/
/*                                                                           */
/*  Function Name : itm_memset                                               */
/*                                                                           */
/*  Description   : This function sets the specified number of bytes in the  */
/*                  buffer to the required value. The functionality is       */
/*                  similar to the standard memset function. The new         */
/*                  implementation was required due to the bug seen when     */
/*                  using memset function across the bridge on MWLAN.        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer.                                */
/*                  2) Value of character to set                             */
/*                  3) Number of characters to set.                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to the buffer                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void *itm_memset(void *buff, UWORD8 val, UWORD32 num)
{
    UWORD8  *cbuff = (UWORD8 *)buff;

    if(num < 20)
        while(num--)
            *cbuff++ = val;
    else
    {
        UWORD32 *wbuff = NULL;
        UWORD32 wval   = val;
        UWORD32 temp   = 0;

        temp = (UWORD32)cbuff & 0x3;

        /* Unaligned buffer */
        num -= temp;
        while(temp--)
            *cbuff++ = val;

        /* Word transfers */
        wval += (wval << 8) + (wval << 16) + (wval << 24);
        wbuff = (UWORD32 *)cbuff;
        while(num > 3)
        {
            *wbuff++ = wval;
            num -= 4;
        }

        /* Unaligned length */
        cbuff = (UWORD8 *)wbuff;
        while(num--)
            *cbuff++ = val;
    }

    return buff;
}

#endif /* MWLAN */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : calibrate_delay_loop                                     */
/*                                                                           */
/*  Description   : This function calibrates the delay loop counter using    */
/*                  MAC H/w TSF Timer .                                      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_calib_cnt                                              */
/*                                                                           */
/*  Processing    : MAC H/w version register is read a fixed number of times */
/*                  to introduce delay in S/w. This function calibrates this */
/*                  delay mechanism. It updates the global variable          */
/*                  (g_calib_cnt) which holds the number of times the        */
/*                  version register should be read to introduce a delay of  */
/*                  10us.                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void calibrate_delay_loop(void)
{
    UWORD32 i = 0;
    UWORD32 entry_time  = 0;
    UWORD32 exit_time   = 0;
    BOOL_T  pa_enabled  = BFALSE;
    BOOL_T  tbtt_mask   = BFALSE;
    UWORD32 tsf_ctrl_bkup = 0;
    UWORD32 calib_thresh  = 0;

	TROUT_FUNC_ENTER;
    /* Backup the registers which will be used for the calibration process */
    pa_enabled = is_machw_enabled();
    tbtt_mask  = is_machw_tbtt_int_masked();
    tsf_ctrl_bkup = get_machw_tsf_ctrl();

    critical_section_start();
	
    /* PA is disabled but TBTT Interrupts can still come. Mask it. */
    disable_machw_phy_and_pa();
    mask_machw_tbtt_int();

    set_machw_tsf_start();
    set_machw_tsf_beacon_tx_suspend_enable();

    /* Initialize Calibration Parameters */
    calib_thresh = 1000;
    entry_time   = get_machw_tsf_timer_lo();

    for(i = 0; i < calib_thresh; i++)
    {
         GET_TIME();	//modified by chengwg.
    }
    
    exit_time = get_machw_tsf_timer_lo();

    /* Restore the Backed-up registers */
    if(pa_enabled == BTRUE)
        enable_machw_phy_and_pa();

    if(tbtt_mask == BFALSE)
        unmask_machw_tbtt_int();

    set_machw_tsf_ctrl(tsf_ctrl_bkup);

    critical_section_end();

    /* The Delay Calibration Count is computed to provide a delay of 10us */
    if(exit_time > entry_time)
        g_calib_cnt = ((calib_thresh+2)*10)/(exit_time-entry_time) + 1;

    TROUT_DBG4("Delay Calibration: Cnt=%d Delay=%d Calib_Cnt=%d\n",calib_thresh+2,
           (exit_time-entry_time), g_calib_cnt);
    TROUT_FUNC_EXIT;
}

#ifdef COMBO_SCAN
void calibrate_delay_loop_plus(void)
{
    UWORD32 i = 0;
    UWORD32 entry_time  = 0;
    UWORD32 exit_time   = 0;
    BOOL_T  pa_enabled  = BFALSE;
    BOOL_T  tbtt_mask   = BFALSE;
    UWORD32 tsf_ctrl_bkup = 0;
    UWORD32 calib_thresh  = 0;

	TROUT_FUNC_ENTER;
    /* Backup the registers which will be used for the calibration process */
    pa_enabled = is_machw_enabled();
    tbtt_mask  = is_machw_tbtt_int_masked();
    tsf_ctrl_bkup = get_machw_tsf_ctrl();

    critical_section_start();
	
    /* PA is disabled but TBTT Interrupts can still come. Mask it. */
    disable_machw_phy_and_pa();
    mask_machw_tbtt_int();

    set_machw_tsf_start();
    set_machw_tsf_beacon_tx_suspend_enable();

    /* Initialize Calibration Parameters */
    calib_thresh = 1000;
    entry_time   = get_machw_tsf_timer_lo();

    for(i = 0; i < calib_thresh; i++)
    {
         GET_TIME();	//modified by chengwg.
    }
    
    exit_time = get_machw_tsf_timer_lo();

    /* Restore the Backed-up registers */
    if(pa_enabled == BTRUE)
        enable_machw_phy_and_pa();

    if(tbtt_mask == BFALSE)
        unmask_machw_tbtt_int();

    set_machw_tsf_ctrl(tsf_ctrl_bkup);

    critical_section_end();

    /* The Delay Calibration Count is computed to provide a delay of 10us */
    //if(exit_time > entry_time)
        //g_calib_cnt = ((calib_thresh+2)*10)/(exit_time-entry_time) + 1;

    //TROUT_DBG4("Delay Calibration: Cnt=%d Delay=%d Calib_Cnt=%d\n",calib_thresh+2,
           //(exit_time-entry_time), g_calib_cnt);
    TROUT_FUNC_EXIT;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_calib_delay                                          */
/*                                                                           */
/*  Description   : This function provides a minimum S/w delay of the        */
/*                  required time specified in units of 10us                 */
/*                                                                           */
/*  Inputs        : 1) The required delay in units of 10us. i.e. Input 10    */
/*                  will provide a delay of 100us                            */
/*                                                                           */
/*  Globals       : g_calib_cnt                                              */
/*                                                                           */
/*  Processing    : The MAC H/w version number is read continuously for a    */
/*                  precomputed number of times to provide the required      */
/*                  delay.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void add_calib_delay(UWORD32 delay)
{
    UWORD32 i = 0;
//    UWORD32 j = 0;
    UWORD32 delay_thresh = g_calib_cnt * delay;

    for(i = 0; i < delay_thresh; i++)
         //j += get_machw_pa_ver();
         //j += (*(volatile UWORD32 *)HOST_DELAY_FOR_TROUT_PHY);	//add by chengq.
         GET_TIME();
}

#ifdef DEBUG_MODE
void print_ba_debug_stats(void)
{
    UWORD32 idx = 0;

    PRINTK("BA Frames Rxd = %d\n\r",                                 g_mac_stats.babarxd);
    PRINTK("BAR Frames successfully Txd = %d\n\r",                   g_mac_stats.babartxd);
    PRINTK("BAR Frames Rxd = %d\n\r",                                g_mac_stats.babarrxd);
    PRINTK("Data Frames retransmitted = %d\n\r",                     g_mac_stats.badatretx);
    PRINTK("Times Window is moved = %d\n\r",                         g_mac_stats.bawinmove);
    PRINTK("BAR Tx-Failures = %d\n\r",                               g_mac_stats.babarfail);
    PRINTK("Data Tx-Failures = %d\n\r",                              g_mac_stats.badatfail);
    PRINTK("Missing Buffers = %d\n\r",                               g_mac_stats.babufmiss);
    PRINTK("Frames deleted during Buffer cleanup = %d\n\r",          g_mac_stats.badatclnup);
    PRINTK("Pending Frames discarded = %d\n\r",                      g_mac_stats.bapenddrop);
    PRINTK("Frames Txd from the Pending Q = %d\n\r",                 g_mac_stats.bapendingtxwlantxd);
    PRINTK("Stale BA frames received = %d\n\r",                      g_mac_stats.baoldbarxd);
    PRINTK("Stale BA frames received = %d\n\r",                      g_mac_stats.baoldbarrxd);
    PRINTK("Frames received out of window and hence droped = %d\n\r",g_mac_stats.barxdatoutwin);
    PRINTK("Re-queue failures = %d\n\r",                             g_mac_stats.bartrqfail);
    PRINTK("Number of blocks Qed = %d\n\r",                          g_mac_stats.banumblks);
    PRINTK("Number of Frames Qed = %d\n\r",                          g_mac_stats.banumqed);
    PRINTK("Number of times the pending Q was empty while enqueing = %d\n\r",g_mac_stats.baemptyQ);
    PRINTK("Num of times grp=%d\n\r",                                g_mac_stats.num_buffto);

    PRINTK("ba_num_dq=%d\n\r",                                      g_mac_stats.ba_num_dq);
    PRINTK("ba_num_dqed=%d\n\r",                                    g_mac_stats.ba_num_dqed);
    PRINTK("batxfba=%d\n\r",                                        g_mac_stats.batxfba);

    for(idx = 0; idx < 10; idx++)
            PRINTK("batemp[%d] = %d\n\r", idx,                      g_mac_stats.batemp[idx]);

}

UWORD8 print_mem_stats(void)
{
    UWORD8 print_flag = 0;

#ifdef MEM_DEBUG_MODE
    print_flag |= printe("nosizeallocexc", g_mem_stats.nosizeallocexc);
    print_flag |= printe("nofreeallocexc", g_mem_stats.nofreeallocexc);
    print_flag |= printe("reallocexc", g_mem_stats.reallocexc);
    print_flag |= printe("corruptallocexc", g_mem_stats.corruptallocexc);

    print_flag |= printe("nullfreeexc", g_mem_stats.nullfreeexc);
    print_flag |= printe("oobfreeexc", g_mem_stats.oobfreeexc);
    print_flag |= printe("refreeexc", g_mem_stats.refreeexc);
    print_flag |= printe("corruptfreeexc", g_mem_stats.corruptfreeexc);
    print_flag |= printe("invalidfreeexc", g_mem_stats.invalidfreeexc);

    print_flag |= printe("excessfreeexc", g_mem_stats.excessfreeexc);
    print_flag |= printe("nulladdexc", g_mem_stats.nulladdexc);
    print_flag |= printe("oobaddexc", g_mem_stats.oobaddexc);
    print_flag |= printe("freeaddexc", g_mem_stats.freeaddexc);
    print_flag |= printe("invalidaddexc", g_mem_stats.invalidaddexc);
    print_flag |= printe("excessaddexc", g_mem_stats.excessaddexc);

    print_flag |= printe("nofreeDscrallocexc[0]", g_mem_stats.nofreeDscrallocexc[0]);
    print_flag |= printe("nofreeDscrallocexc[1]", g_mem_stats.nofreeDscrallocexc[1]);

    print_flag |= printe("nofreePktallocexc[0]", g_mem_stats.nofreePktallocexc[0]);
    print_flag |= printe("nofreePktallocexc[1]", g_mem_stats.nofreePktallocexc[1]);
    print_flag |= printe("nofreePktallocexc[2]", g_mem_stats.nofreePktallocexc[2]);
    print_flag |= printe("nofreePktallocexc[3]", g_mem_stats.nofreePktallocexc[3]);
    print_flag |= printe("nofreePktallocexc[4]", g_mem_stats.nofreePktallocexc[4]);

    print_flag |= printe("nofreeLocalallocexc[0]", g_mem_stats.nofreeLocalallocexc[0]);
    print_flag |= printe("nofreeLocalallocexc[1]", g_mem_stats.nofreeLocalallocexc[1]);
    print_flag |= printe("nofreeLocalallocexc[2]", g_mem_stats.nofreeLocalallocexc[2]);
    print_flag |= printe("nofreeLocalallocexc[3]", g_mem_stats.nofreeLocalallocexc[3]);
    print_flag |= printe("nofreeLocalallocexc[4]", g_mem_stats.nofreeLocalallocexc[4]);
    print_flag |= printe("nofreeLocalallocexc[5]", g_mem_stats.nofreeLocalallocexc[5]);
    print_flag |= printe("nofreeLocalallocexc[6]", g_mem_stats.nofreeLocalallocexc[6]);
    print_flag |= printe("nofreeLocalallocexc[7]", g_mem_stats.nofreeLocalallocexc[7]);

    print_flag |= printe("nofreeEventallocexc", g_mem_stats.nofreeEventallocexc);

    /* Print the size of the maximum Shared memory used and reset it after that */
    PRINTK("Max Scratch Memory Utilized = %d", get_max_scratch_mem_usage());
    reset_scratch_mem_usage();
#endif /* MEM_DEBUG_MODE */

    return print_flag;
}

void print_debug_stats(void)
{
    UWORD8 i = 0;
#ifdef MEM_DEBUG_MODE
    PRINTK("Memory Statistics\n\r");
    PRINTK("sdalloc         = %d\n\r",g_mem_stats.sdalloc);
    PRINTK("sdfree          = %d\n\r",g_mem_stats.sdfree);
    PRINTK("sdtotalfree     = %d\n\r",g_mem_stats.sdtotalfree);

    PRINTK("spalloc         = %d\n\r",g_mem_stats.spalloc);
    PRINTK("spfree          = %d\n\r",g_mem_stats.spfree);
    PRINTK("sptotalfree     = %d\n\r",g_mem_stats.sptotalfree);

    PRINTK("lalloc          = %d\n\r",g_mem_stats.lalloc);
    PRINTK("lfree           = %d\n\r",g_mem_stats.lfree);
    PRINTK("ltotalfree      = %d\n\r",g_mem_stats.ltotalfree);

    PRINTK("ealloc          = %d\n\r",g_mem_stats.ealloc);
    PRINTK("efree           = %d\n\r",g_mem_stats.efree);
    PRINTK("etotalfree      = %d\n\r",g_mem_stats.etotalfree);

    PRINTK("nosizeallocexc  = %d\n\r",g_mem_stats.nosizeallocexc);
    PRINTK("nofreeallocexc  = %d\n\r",g_mem_stats.nofreeallocexc);
    PRINTK("reallocexc      = %d\n\r",g_mem_stats.reallocexc);
    PRINTK("corruptallocexc = %d\n\r",g_mem_stats.corruptallocexc);

    PRINTK("nullfreeexc     = %d\n\r",g_mem_stats.nullfreeexc);
    PRINTK("oobfreeexc      = %d\n\r",g_mem_stats.oobfreeexc);
    PRINTK("refreeexc       = %d\n\r",g_mem_stats.refreeexc);
    PRINTK("corruptfreeexc  = %d\n\r",g_mem_stats.corruptfreeexc);
    PRINTK("invalidfreeexc  = %d\n\r",g_mem_stats.invalidfreeexc);

    PRINTK("excessfreeexc   = %d\n\r",g_mem_stats.excessfreeexc);
    PRINTK("nulladdexc      = %d\n\r",g_mem_stats.nulladdexc);
    PRINTK("oobaddexc       = %d\n\r",g_mem_stats.oobaddexc);
    PRINTK("freeaddexc      = %d\n\r",g_mem_stats.freeaddexc);
    PRINTK("invalidaddexc   = %d\n\r",g_mem_stats.invalidaddexc);
    PRINTK("excessaddexc    = %d\n\r",g_mem_stats.excessaddexc);

    PRINTK("nofreeDscrallocexc[0]   = %d\n\r",g_mem_stats.nofreeDscrallocexc[0]);
    PRINTK("nofreeDscrallocexc[1]   = %d\n\r",g_mem_stats.nofreeDscrallocexc[1]);

    for(i = 0; i < 5; i++)
        PRINTK("nofreePktallocexc[%d]   = %d\n\r",i,
                        g_mem_stats.nofreePktallocexc[i]);

    for(i = 0; i < 8; i++)
        PRINTK("nofreeLocalallocexc[%d] = %d\n\r",i,
                        g_mem_stats.nofreeLocalallocexc[i]);

    PRINTK("nofreeEventallocexc     = %d\n\r",
                            g_mem_stats.nofreeEventallocexc);
#endif /* MEM_DEBUG_MODE */
    PRINTK("\nMAC Statistics\n\r");

#ifndef MAC_HW_UNIT_TEST_MODE
    PRINTK("itbtt   = %d\n\r",g_mac_stats.itbtt);
    PRINTK("itxc    = %d\n\r",g_mac_stats.itxc);
    PRINTK("irxc    = %d\n\r",g_mac_stats.irxc);
    PRINTK("ihprxc  = %d\n\r",g_mac_stats.ihprxc);
    PRINTK("ierr    = %d\n\r",g_mac_stats.ierr);
    PRINTK("ideauth = %d\n\r",g_mac_stats.ideauth);
    PRINTK("icapend = %d\n\r",g_mac_stats.icapend);
    PRINTK("enpmsdu = %d\n\r",g_mac_stats.enpmsdu);
    PRINTK("erxqemp = %d\n\r",g_mac_stats.erxqemp);
    PRINTK("etxsus1machang = %d\n\r",g_mac_stats.etxsus1machang);
    PRINTK("etxsus1phyhang = %d\n\r",g_mac_stats.etxsus1phyhang);
    PRINTK("etxsus3 = %d\n\r",g_mac_stats.etxsus3);
    PRINTK("ebus    = %d\n\r",g_mac_stats.ebus);
    PRINTK("ebwrsig = %d\n\r",g_mac_stats.ebwrsig);
    PRINTK("emsaddr = %d\n\r",g_mac_stats.emsaddr);
    PRINTK("etxfifo = %d\n\r",g_mac_stats.etxfifo);
    PRINTK("erxfifo = %d\n\r",g_mac_stats.erxfifo);
    PRINTK("ehprxfifo = %d\n\r",g_mac_stats.ehprxfifo);
    PRINTK("etxqempt = %d\n\r",g_mac_stats.etxqempt);
    PRINTK("edmanoerr = %d\n\r",g_mac_stats.edmanoerr);
    PRINTK("etxcenr = %d\n\r",g_mac_stats.etxcenr);
    PRINTK("erxcenr = %d\n\r",g_mac_stats.erxcenr);
    PRINTK("esgaf   = %d\n\r",g_mac_stats.esgaf);
    PRINTK("eother  = %d\n\r",g_mac_stats.eother);
    PRINTK("qatxp   = %d\n\r",g_mac_stats.qatxp);
    PRINTK("qdtxp   = %d\n\r",g_mac_stats.qdtxp);
#else /* MAC_HW_UNIT_TEST_MODE */
    PRINTK("rxci          = %d\n\r",g_test_stats.rxci);
    PRINTK("hprxci        = %d\n\r",g_test_stats.hprxci);
    PRINTK("txci          = %d\n\r",g_test_stats.txci);
    PRINTK("tbtti         = %d\n\r",g_test_stats.tbtti);
    PRINTK("erri          = %d\n\r",g_test_stats.erri);
    PRINTK("capei         = %d\n\r",g_test_stats.capei);
    PRINTK("uki           = %d\n\r",g_test_stats.uki);
    PRINTK("err.enpmsdu   = %d\n\r",g_test_stats.exp.enpmsdu);
    PRINTK("err.erxqemp   = %d\n\r",g_test_stats.exp.erxqemp);
    PRINTK("err.emsaddr   = %d\n\r",g_test_stats.exp.emsaddr);
    PRINTK("err.etxsus1machang = %d\n\r",g_test_stats.exp.etxsus1machang);
    PRINTK("err.etxsus1phyhang = %d\n\r",g_test_stats.exp.etxsus1phyhang);
    PRINTK("err.etxsus3   = %d\n\r",g_test_stats.exp.etxsus3);
    PRINTK("err.ebus      = %d\n\r",g_test_stats.exp.ebus);
    PRINTK("err.ebwrsig   = %d\n\r",g_test_stats.exp.ebwrsig);
    PRINTK("err.etxqempt  = %d\n\r",g_test_stats.exp.etxqempt);
    PRINTK("err.edmanoerr = %d\n\r",g_test_stats.exp.edmanoerr);
    PRINTK("err.etxcenr   = %d\n\r",g_test_stats.exp.etxcenr);
    PRINTK("err.erxcenr   = %d\n\r",g_test_stats.exp.erxcenr);
    PRINTK("err.esgaf     = %d\n\r",g_test_stats.exp.esgaf);
    PRINTK("err.etxfifo   = %d\n\r",g_test_stats.exp.etxfifo);
    PRINTK("err.erxfifo   = %d\n\r",g_test_stats.exp.erxfifo);
    PRINTK("err.eother    = %d\n\r",g_test_stats.exp.eother);
#endif /* MAC_HW_UNIT_TEST_MODE */
}

void print_build_flags(void)
{
#ifdef ETHERNET_HOST
    PRINTK("ETHERNET_HOST\n\r");
#endif /* ETHERNET_HOST */


#ifdef GENERIC_HOST
    PRINTK("GENERIC_HOST\n\r");
#endif /* GENERIC_HOST */

#ifdef PHY_802_11n
    PRINTK("PHY_802_11n\n\r");
#endif /* PHY_802_11n */

#ifdef GENERIC_PHY
    PRINTK("GENERIC_PHY\n\r");
#endif /* GENERIC_PHY */


#ifdef ITTIAM_PHY
    PRINTK("ITTIAM_PHY\n\r");
#endif /* ITTIAM_PHY */



#ifdef BSS_ACCESS_POINT_MODE
    PRINTK("BSS_ACCESS_POINT_MODE\n\r");
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    PRINTK("IBSS_BSS_STATION_MODE\n\r");
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    PRINTK("MAC_HW_UNIT_TEST_MODE \n\r");
#endif /* MAC_HW_UNIT_TEST_MODE  */

#ifdef MAC_802_11I
    PRINTK("MAC_802_11I \n\r");
#endif /* MAC_802_11I  */

#ifdef SUPP_11I
    PRINTK("SUPP_11I \n\r");
#endif /* SUPP_11I  */

#ifdef MAC_WMM
    PRINTK("MAC_WMM \n\r");
#endif /* MAC_WMM  */

#ifdef MAC_802_11N
    PRINTK("MAC_802_11N \n\r");
#endif /* MAC_802_11N  */


#ifdef MAC_802_1X
    PRINTK("MAC_802_1X \n\r");
#endif /* MAC_802_1X  */

#ifdef MAC_802_11H
    PRINTK("MAC_802_11H \n\r");
#endif /* MAC_802_11H */

#ifdef GENERIC_RF
    PRINTK("GENERIC_RF\n\r");
#endif /* GENERIC_RF */



#ifdef RF_MAXIM_ITTIAM
    PRINTK("RF_MAXIM_ITTIAM \n\r");
#endif /* RF_MAXIM_ITTIAM */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef RF_AIROHA_ITTIAM
    PRINTK("RF_AIROHA_ITTIAM \n\r");
#endif /* RF_AIROHA_ITTIAM */
#endif

#ifdef MAX2829
    PRINTK("MAX2829 \n\r");
#endif /* MAX2829 */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAX2830_32
    PRINTK("MAX2830_32 \n\r");
#endif /* MAX2830_32 */

#ifdef AL2236
    PRINTK("AL2236 \n\r");
#endif /* AL2236 */

#ifdef AL7230
    PRINTK("AL7230 \n\r");
#endif /* AL7230 */
#endif

#ifdef MWLAN
    PRINTK("MWLAN \n\r");
#endif /* MWLAN */

#ifdef OS_LINUX_CSL_TYPE
    PRINTK("OS_LINUX_CSL_TYPE \n\r");
#endif /* OS_LINUX_CSL_TYPE */

#ifdef DEBUG_MODE
    PRINTK("DEBUG_MODE \n\r");
#endif /* DEBUG_MODE */


#ifdef USE_PROCESSOR_DMA
    PRINTK("USE_PROCESSOR_DMA \n\r");
#endif /* USE_PROCESSOR_DMA */

#ifdef EDCA_DEMO_KLUDGE
    PRINTK("EDCA_DEMO_KLUDGE \n\r");
#endif /* EDCA_DEMO_KLUDGE */

#ifdef LOCALMEM_TX_DSCR
    PRINTK("LOCALMEM_TX_DSCR \n\r");
#endif /* LOCALMEM_TX_DSCR */

#ifdef AUTORATE_FEATURE
    PRINTK("AUTORATE_FEATURE \n\r");
#endif /* AUTORATE_FEATURE */

#ifdef DISABLE_MACHW_DEFRAG
    PRINTK("DISABLE_MACHW_DEFRAG \n\r");
#endif /* DISABLE_MACHW_DEFRAG */

#ifdef DISABLE_MACHW_DEAGGR
    PRINTK("DISABLE_MACHW_DEAGGR \n\r");
#endif /* DISABLE_MACHW_DEAGGR */

#ifdef PHY_TEST_MAX_PKT_RX
    PRINTK("PHY_TEST_MAX_PKT_RX \n\r");
#endif /* PHY_TEST_MAX_PKT_RX */

#ifdef DEFAULT_SME
    PRINTK("DEFAULT_SME \n\r");
#endif /* DEFAULT_SME */

#ifdef NO_ACTION_RESET
    PRINTK("NO_ACTION_RESET \n\r");
#endif /* NO_ACTION_RESET */

#ifdef LITTLE_ENDIAN
    PRINTK("LITTLE_ENDIAN \n\r");
#endif /* LITTLE_ENDIAN */

#ifdef DSCR_MACROS_NOT_DEFINED
    PRINTK("DSCR_MACROS_NOT_DEFINED \n\r");
#endif /* DSCR_MACROS_NOT_DEFINED */

#ifdef PHY_CONTINUOUS_TX_MODE
    PRINTK("PHY_CONTINUOUS_TX_MODE \n\r");
#endif /* PHY_CONTINUOUS_TX_MODE */

#ifdef HANDLE_ERROR_INTR
    PRINTK("HANDLE_ERROR_INTR \n\r");
#endif /* HANDLE_ERROR_INTR */

#ifdef MEM_DEBUG_MODE
    PRINTK("MEM_DEBUG_MODE \n\r");
#endif /* MEM_DEBUG_MODE */

#ifdef MEM_STRUCT_SIZES_INIT
    PRINTK("MEM_STRUCT_SIZES_INIT \n\r");
#endif /* MEM_STRUCT_SIZES_INIT */


#ifdef TX_ABORT_FEATURE
    PRINTK("TX_ABORT_FEATURE \n\r");
#endif /* TX_ABORT_FEATURE */

}
#endif /* DEBUG_MODE */

/*chenq add itm trace*/

/*flag of ShareMemInfo*/
int g_debug_print_tx_pkt_on = 0;
int g_debug_print_rx_ptk_on = 0;
int g_debug_print_tx_buf_on = 0;
int g_debug_print_rx_buf_on = 0;
int g_debug_buf_use_info_start = 0;
/*flag of MacTxRxStatistics*/
int g_debug_txrx_reg_info_start = 0;
int g_debug_txrx_frame_info_start = 0;
int g_debug_rx_size_info_start = 0;
int g_debug_isr_info_start = 0;
/*flag of SpiSdioDmaState*/
int g_debug_print_spisdio_bus_on = 0;
int g_debug_print_dma_do_on = 0;
int g_debug_spisdiodma_isr_info_start = 0;
/*flag of MacFsmMibState*/
int g_debug_print_fsm_on = 0;
int g_debug_print_assoc_on = 0;
int g_debug_print_Enc_auth_on = 0;
int g_debug_print_wps_on = 0; 
int g_debug_print_ps_on = 0;//PowerSave
int g_debug_print_wd_on = 0;//WiFi-Direct
int g_debug_print_txrx_path_on = 0;
/*flag of Host6820Info*/
//no add


void Reset_itm_trace_flag(void)
{
/*flag of ShareMemInfo*/	
    g_debug_print_tx_pkt_on = 0;
    g_debug_print_rx_ptk_on = 0;
    g_debug_print_tx_buf_on = 0;
    g_debug_print_rx_buf_on = 0;
    g_debug_buf_use_info_start = 0;
/*flag of MacTxRxStatistics*/
	g_debug_txrx_reg_info_start = 0;
	g_debug_txrx_frame_info_start = 0;
	g_debug_rx_size_info_start = 0;
	g_debug_isr_info_start = 0;
/*flag of SpiSdioDmaState*/
	g_debug_print_spisdio_bus_on = 0;
	g_debug_print_dma_do_on = 0;
	g_debug_spisdiodma_isr_info_start = 0;
/*flag of MacFsmMibState*/
	g_debug_print_fsm_on = 0;
	g_debug_print_assoc_on = 0;
	g_debug_print_Enc_auth_on = 0;
	g_debug_print_wps_on = 0; 
	g_debug_print_ps_on = 0;//PowerSave
	g_debug_print_wd_on = 0;//WiFi-Direct
	g_debug_print_txrx_path_on = 0;
/*flag of Host6820Info*/
    //no add
}

void ShareMemInfo(int type,int flag,int value,char * reserved2ext)
{
	if(type == itm_debug_plog_sharemem_tx_pkt)
	{
		g_debug_print_tx_pkt_on = value;
	}
	else if(type == itm_debug_plog_sharemem_rx_ptk)
	{
		g_debug_print_rx_ptk_on = value;
	}
	else if(type == itm_debug_plog_sharemem_tx_buf)
	{
		g_debug_print_tx_buf_on = value;
	}	
    else if(type == itm_debug_plog_sharemem_rx_buf)
	{
		g_debug_print_rx_buf_on = value;
    }
	else if(type == itm_debug_plog_sharemem_buf_use)
	{
		if(flag == counter_start)
    	{
    		g_debug_buf_use_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_buf_use_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_buf_use_info_start = counter_end;
		}
	}
}

void MacTxRxStatistics(int type,int flag,char * reserved2ext)
{
	if(type == itm_debug_plog_mactxrx_reg)
	{
		if(flag == counter_start)
    	{
    		g_debug_txrx_reg_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_txrx_reg_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_txrx_reg_info_start = counter_end;
		}
	}
	else if(type == itm_debug_plog_mactxrx_frame)
	{
		if(flag == counter_start)
    	{
    		g_debug_txrx_frame_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_txrx_frame_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_txrx_frame_info_start = counter_end;
		}
	}
	else if(type == itm_debug_plog_mactxrx_rx_size)
	{
		if(flag == counter_start)
    	{
    		g_debug_rx_size_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_rx_size_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_rx_size_info_start = counter_end;
		}
	}
	else if(type == itm_debug_plog_mactxrx_isr)
	{
		if(flag == counter_start)
    	{
    		g_debug_isr_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_isr_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_isr_info_start = counter_end;
		}
	}
}

void SpiSdioDmaState(int type,int flag,int value,char * reserved2ext)
{
	if(type == itm_debug_plog_spisdiodma_spisdio)
	{
		g_debug_print_spisdio_bus_on = value;
	}
	else if(type == itm_debug_plog_spisdiodma_dma)
	{
		g_debug_print_dma_do_on = value;
	}
	else if(type == itm_debug_plog_spisdiodma_isr)
	{
		if(flag == counter_start)
    	{
    		g_debug_spisdiodma_isr_info_start = counter_start;
		}
		else if( ( flag == counter_end ) && ( g_debug_spisdiodma_isr_info_start == counter_end ) )
		{
			/*printk("already in counter_end stat\n");*/
		}
		else if( flag == counter_end )
		{
			g_debug_spisdiodma_isr_info_start = counter_end;
		}
	}
}

void MacFsmMibState(int type,int value,char * reserved2ext)
{
	if(type == itm_debug_plog_macfsm_mib_fsm)
	{
		g_debug_print_fsm_on = value;
	}
	else if(type == itm_debug_plog_macfsm_mib_assoc)
	{
		g_debug_print_assoc_on = value;
	}
	else if(type == itm_debug_plog_macfsm_mib_Enc_auth)
	{
		g_debug_print_Enc_auth_on = value;
	}
	else if(type == itm_debug_plog_macfsm_mib_wps)
	{
		g_debug_print_wps_on = value; 
	}
	else if(type == itm_debug_plog_macfsm_mib_ps)//PowerSave
	{
		g_debug_print_ps_on = value;//PowerSave
	}
	else if(type == itm_debug_plog_macfsm_mib_wd)//WiFi-Direct
	{
		g_debug_print_wd_on = value;//WiFi-Direct
	}
	else if(type == itm_debug_plog_macfsm_mib_txrx_path)
	{
		g_debug_print_txrx_path_on = value;
	}
	else if(type == itm_debug_plog_macfsm_mib_mibapp)
	{
		/*print ...*/
	}
	else if(type == itm_debug_plog_macfsm_mib_mibprtcl)
	{
		/*print ...*/
	}
	else if(type == itm_debug_plog_macfsm_mib_mibmac)
	{
		/*print ...*/
	}
}

void Host6820Info(int type,char * reserved2ext)
{

}

/*chenq add end*/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
/* 
 * Notify co-processor to handle Power Management event
 * through interrupt.
 * Author: Keguang
 * Date: 20130321
 */
inline void notify_cp_with_handshake(uint msg, uint retry)
{
        uint i = retry;
        uint count = host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) + 1; 

//#ifdef POWERSAVE_DEBUG
        pr_info("rSYSREG_POWER_CTRL: %x\n", host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
//#endif

        host_write_trout_reg((UWORD32)msg, (UWORD32)rSYSREG_HOST2ARM_INFO1);	/*load message*/
        host_write_trout_reg((UWORD32)0x1, (UWORD32)rSYSREG_GEN_ISR_2_ARM7);	/*interrupt CP*/
        /*pr_info("command done!\n");*/

        /*wait for CP*/
        while((host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) != count) && i--) {
                msleep(10);
//#ifdef POWERSAVE_DEBUG
			pr_info("Done! rSYSREG_POWER_CTRL: %x\n", host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
//#endif
        }
	pr_info("!!! rSYSREG_POWER_CTRL: %x, retry %d, i %d\n", host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL), retry, i);
        host_write_trout_reg(0x0, (UWORD32)rSYSREG_HOST2ARM_INFO1);		/*clear message*/
		if(msg == PS_MSG_WIFI_SUSPEND_MAGIC) 
			g_done_wifi_suspend = 1;
		else if(msg == PS_MSG_WIFI_RESUME_MAGIC) 
			g_done_wifi_suspend = 0;
}
EXPORT_SYMBOL(notify_cp_with_handshake);

extern int prepare_null_frame_for_cp(UWORD8 psm, BOOL_T is_qos, UWORD8 priority);

void check_and_retransmit(void)
{
	uint which_frame = 0;
	uint sta = 0;
	uint vs;
	uint retry = 0;
	unsigned char tmp[200];
	uint *pw = (uint *)&tmp[0];


        which_frame = root_host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO1);
	if(which_frame)
		sta = BEACON_MEM_BEGIN;
	else
		sta = BEACON_MEM_BEGIN + 200;

	root_host_read_trout_ram((void *)tmp,  (void *)sta, TX_DSCR_LEN); 
	if(((tmp[3] >> 5) & 0x3)  == 0x3){
		goto retx;
	}
	if((tmp[20] & 0x3) != 0x3){
		printk("SF0-CASUED\n");
		goto retx;
	}

	/* arrive here, means the last frame ARM7 sent was success(AP acked) do nothing*/
	return;

retx:
	tmp[3] &= 0x9F;
	tmp[3] |= 0x20;

	tmp[20] &= 0xFC;
	vs = root_host_read_trout_reg((UWORD32)rMAC_TSF_TIMER_LO);
	vs = (vs >> 10) & 0xFFFF;
	pw[3] &= 0xFFFF0000;
	pw[3] |= vs;

	printk("RE-TX\n");
	root_host_write_trout_ram((void *)sta, (void *)tmp, TX_DSCR_LEN);
	root_host_write_trout_reg((UWORD32)sta, (UWORD32)rMAC_EDCA_PRI_HP_Q_PTR);
	msleep(20);
	return;
}
/*for internal use only*/
inline void root_notify_cp_with_handshake(uint msg, uint retry)
{
        uint i = retry;
        uint count = root_host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) + 1; 

#ifdef POWERSAVE_DEBUG
        pr_info("rSYSREG_POWER_CTRL: %x\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
        /*pr_info("command %x\n", msg);*/
#endif

        root_host_write_trout_reg((UWORD32)msg, (UWORD32)rSYSREG_HOST2ARM_INFO1);	/*load message*/
        root_host_write_trout_reg((UWORD32)0x1, (UWORD32)rSYSREG_GEN_ISR_2_ARM7);	/*interrupt CP*/
        /*pr_info("command done!\n");*/
	if((msg & 0xFFFF) == PS_MSG_ARM7_EBEA_KC_MAGIC){
		printk("EBEA.......\n");
		msleep(75);
		check_and_retransmit();	
	}

        /*wait for CP*/
        while((root_host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM) != count) && i--) {
                msleep(10);
#ifdef POWERSAVE_DEBUG
                pr_info("Done! rSYSREG_POWER_CTRL: %x\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
		/*pr_info("expected %x, SYSREG_INFO1_FROM_ARM = %x\n", count, root_host_read_trout_reg((UWORD32)rSYSREG_INFO1_FROM_ARM));*/
#endif
        }
        pr_info("@@@ rSYSREG_POWER_CTRL: %x, retry %d, i %d\n", root_host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL), retry, i);
        root_host_write_trout_reg(0x0, (UWORD32)rSYSREG_HOST2ARM_INFO1);		/*clear message*/
        if(msg == PS_MSG_WIFI_SUSPEND_MAGIC) 
			g_done_wifi_suspend = 1;
		else if(msg == PS_MSG_WIFI_RESUME_MAGIC) 
			g_done_wifi_suspend = 0;
}
#endif
