/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : MPC8349.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      device addresses in Motorolla PowerPC MPC8349        */
/*                      processor.                                           */
/*                                                                           */
/*  List of Functions : NONE                                                 */
/*  Issues            : NONE                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MWLAN

#ifndef MPC8349_H
#define MPC8349_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "spi_interface.h"

//#define ENABLE_MWLAN_WDT

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Processor Register Addresses                                              */
/*****************************************************************************/

/*****************************************************************************/
/* DMA Registers                                                             */
/*****************************************************************************/

#define MPC83xx_IRQ_EXT1 0	//add by Hugh
#if 0
//masked by Hugh
#define rDMA0_MODE            (IMMR_BASE + 0x8100)
#define rDMA0_STATUS          (IMMR_BASE + 0x8104)
#define rDMA0_SADDR           (IMMR_BASE + 0x8110)
#define rDMA0_DADDR           (IMMR_BASE + 0x8118)
#define rDMA0_BYTECNT         (IMMR_BASE + 0x8120)

/*****************************************************************************/
/* DMA Registers                                                             */
/*****************************************************************************/

#define rWDT_CONTROL          (IMMR_BASE + 0x0204)
#define rWDT_COUNT            (IMMR_BASE + 0x0208)
#define rWDT_SERVICE          (*(volatile UWORD16 *)(IMMR_BASE + 0x020E))

/*****************************************************************************/
/* GPIO Registers                                                            */
/*****************************************************************************/

#define rPROC_GPIO1_DIR       (IMMR_BASE + 0x0C00)
#define rPROC_GPIO1_DAT       (IMMR_BASE + 0x0C08)

#endif

/*****************************************************************************/
/* DMA Related Constants                                                     */
/*****************************************************************************/

#define DMA_CFG_WORD          0x00100004

/*****************************************************************************/
/* GPIO Related Constants                                                    */
/*****************************************************************************/

#define PROC_LED_1            (0x80000000 >> 17)
#define PROC_LED_2            (0x80000000 >> 18)
#define PROC_LED_3            (0x80000000 >> 12)
#define PROC_LED_4            (0x80000000 >> 14)
#define PROC_LED_5            (0x80000000 >> 20)

/*****************************************************************************/
/* Watchdog Timer Related Constants                                          */
/*****************************************************************************/

#define WDT_SERVICE_1         0x556C
#define WDT_SERVICE_2         0xAA39
#define WDT_ENABLE            BIT2
#define WDT_INIT_VAL          (BIT1 | BIT0)
#define WDT_1SEC_COUNT        4069
#define WDT_TIMEOUT_INTVL     10   /* Time in secs for timeout */

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/

/* These macros control the LED operation. */

//chenq mod
//#define TOGGLE_LED(n)   (rPROC_GPIO1_DAT ^= PROC_LED_##n)
//#define TURN_ON_LED(n)  (rPROC_GPIO1_DAT &= ~PROC_LED_##n)
//#define TURN_OFF_LED(n) (rPROC_GPIO1_DAT |= PROC_LED_##n)
//modified by Hugh
#define TOGGLE_LED(n)   //host_write_trout_reg((host_read_trout_reg((UWORD32)rPROC_GPIO1_DAT) ^ PROC_LED_##n), ((UWORD32)rPROC_GPIO1_DAT))
#define TURN_ON_LED(n)  //host_write_trout_reg((host_read_trout_reg((UWORD32)rPROC_GPIO1_DAT) & ~PROC_LED_##n), ((UWORD32)rPROC_GPIO1_DAT))
#define TURN_OFF_LED(n) //host_write_trout_reg((host_read_trout_reg( (UWORD32)rPROC_GPIO1_DAT) | PROC_LED_##n), ((UWORD32)rPROC_GPIO1_DAT))


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 do_chan0_dma(void *dstp, void *srcp, UWORD32 len);
extern void set_led_pat(UWORD8 num);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function configures the processor GPIO pins connected to the LEDs as */
/* output. This should be done before any of the LED operations are          */
/* performed.                                                                */
INLINE void init_led_control(void)
{

}

INLINE void turn_on_led(UWORD8 led_num)
{
    switch(led_num)
    {
        case 1:
            TURN_ON_LED(1);
        break;
        case 2:
            TURN_ON_LED(2);
        break;
        case 3:
            TURN_ON_LED(3);
        break;
        case 4:
            TURN_ON_LED(4);
        break;
        case 5:
            TURN_ON_LED(5);
        break;
    }
}

INLINE void turn_off_led(UWORD8 led_num)
{
    switch(led_num)
    {
        case 1:
            TURN_OFF_LED(1);
        break;
        case 2:
            TURN_OFF_LED(2);
        break;
        case 3:
            TURN_OFF_LED(3);
        break;
        case 4:
            TURN_OFF_LED(4);
        break;
        case 5:
            TURN_OFF_LED(5);
        break;
    }
}


INLINE void toggle_led(UWORD8 led_num)
{
    switch(led_num)
    {
        case 1:
            TOGGLE_LED(1);
        break;
        case 2:
            TOGGLE_LED(2);
        break;
        case 3:
            TOGGLE_LED(3);
        break;
        case 4:
            TOGGLE_LED(4);
        break;
        case 5:
            TOGGLE_LED(5);
        break;
    }
}
INLINE void init_mwlan_wdt(void)
{
#ifdef ENABLE_MWLAN_WDT
    UWORD32 count = WDT_1SEC_COUNT * WDT_TIMEOUT_INTVL;
    UWORD32 regval = 0;

    regval = (count << 16) | WDT_INIT_VAL | WDT_ENABLE;

    /* Start Watchdog Timer */

    //chenq mod
    //rWDT_CONTROL = regval;
    host_write_trout_reg( regval, (UWORD32)rWDT_CONTROL );
#endif /* ENABLE_MWLAN_WDT */
}

INLINE void service_mwlan_wdt(void)
{
#ifdef ENABLE_MWLAN_WDT
    //chenq mod
    rWDT_SERVICE = WDT_SERVICE_1;
    rWDT_SERVICE = WDT_SERVICE_2;
    //host_write_trout_reg( WDT_SERVICE_1, (UWORD32)rWDT_SERVICE );
    //host_write_trout_reg( WDT_SERVICE_2, (UWORD32)rWDT_SERVICE );
#endif /* ENABLE_MWLAN_WDT */
}
#endif /* MPC8349_H */
#endif /* MWLAN */

