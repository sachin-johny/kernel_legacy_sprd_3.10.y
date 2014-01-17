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
/*  File Name         : metrics.h                                            */
/*                                                                           */
/*  Description       : This file contains all the definitions and extern    */
/*                      variables required for getting the metrics.          */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef METRICS_H
#define METRICS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mh.h"
#include "transmit.h"
#include "spi_interface.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#ifdef MEASURE_PROCESSING_DELAY
#define NUM_PROC_DELAY_INDX_SUP           MAX_NUM_EVENT_QUEUES
#define NUM_ISR_DELAY_INDX_SUP            2 /* HostIF & MAC HW ISRs */
#define NUM_DELAY_INDEXES_SUPPORTED       (NUM_PROC_DELAY_INDX_SUP + \
                                           NUM_ISR_DELAY_INDX_SUP)
#endif /* MEASURE_PROCESSING_DELAY */

#ifdef LATENCY_TEST
#define NUM_LATENCY_PKT_COUNT 1000
#endif /* LATENCY_TEST */

#ifdef CPU_UTIL_TEST
#define CPU_UTIL_TEST_TIME_SLICE  50
#define CPU_UTIL_TEST_CALIB_COUNT 1000
#define CPU_UTIL_TEST_COUNT       2000

#define CPU_UTIL_CUT_OFF_VALUE    1000000
#define CPU_UTIL_NUM_BINS         30

#endif /* CPU_UTIL_TEST */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

#ifdef MEASURE_PROCESSING_DELAY

typedef enum { ISR_DELAY  = 0,  /* ISR Delay Type */
               PROC_DELAY = 1   /* PROCESSING Delay Type */
} DELAY_TYPE_T;

typedef enum { HOST_RX_DELAY_INDEX = 0,
               WLAN_RX_DELAY_INDEX = 1,
               MISC_DELAY_INDEX    = 2
} PROC_DELAY_INDEX_T;


typedef enum { MACHW_ISR_DELAY_INDEX = 0,
               HIF_ISR_DELAY_INDEX   = 1
} ISR_DELAY_INDEX_T;

#endif /* MEASURE_PROCESSING_DELAY */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

#ifdef MEASURE_PROCESSING_DELAY

typedef struct
{
    UWORD32 max_delay;
    UWORD32 min_delay;
    UWORD32 delay_sum;
    UWORD32 delay_cnt;
    UWORD32 timer_start;
    UWORD32 timer_stop;
} delay_struct_t;

typedef struct
{
    UWORD32 hifrxint;          /* No Of Host Rx Interrupts */
    UWORD32 hiftxint;          /* No Of Host Tx Interrupts */
    UWORD32 hostrxevent;       /* No Of Host Rx Events */
    UWORD32 submsdu2machwtxq;  /* No Of MSDUs  added to MACHW TxQ*/
    UWORD32 txdscr2machwtxq;   /* No Of TxDscr added to MACHW TxQ */
    UWORD32 txcmpint;          /* No Of Tx Complete Interrupts */
    UWORD32 misctxcompevent;   /* No Of Misc Tx Complete Events */
    UWORD32 numsubmsduontxcmp; /* No Of MSDUs on Tx complete */
    UWORD32 numtxdscrontxcmp;  /* No Of TxDscr on Tx Complete */
    UWORD32 rxcmpint;          /* No Of Rx Complete Interrupts */
    UWORD32 wlanrxevent;       /* No Of WLAN RX Events */
    UWORD32 numrxdscr;         /* No Of RxDscr on Rx Complete */
    UWORD32 nummsduhiftxq;     /* No Of MSDUs Added to HIF TxQ */
    UWORD32 nummsdudrpedhiftxq;/* No Of MSDUs Dropped at HIF */
    UWORD32 hifbusy;           /* No Of times HIF was busy */
    UWORD32 nummsduhiftxed;    /* No Of MSDUs Txed to the Host */
    UWORD32 bamsdupndq2mhwtxq; /* No Of MSDUs added to HwQ from PendingQ */
    UWORD32 bamsduretq2mhwtxq; /* No Of MSDUs added to HwQ form SwRetryQ */
    UWORD32 num_tsf_acesses;   /* No Of times TSF timer is acessed */
    UWORD32 numbarrxd;         /* No Of BAR Received   */
    UWORD32 numbartxd;         /* No Of BAR Transfered */
} delay_stats_t;

#endif /* MEASURE_PROCESSING_DELAY */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

#ifdef LATENCY_TEST
extern UWORD32 g_rx_lat_1[128];
extern UWORD32 g_rx_lat_2[128];
extern UWORD32 g_rx_lat_3[128];

extern UWORD32 g_rx_lat_index1;
extern UWORD32 g_rx_lat_index2;
extern UWORD32 g_rx_lat_index3;

extern UWORD32 g_min_tx_lat;
extern UWORD32 g_max_tx_lat;
extern UWORD32 g_lat_cnt;
extern UWORD32 g_tx_lat_sum;
extern UWORD32 g_avg_tx_lat;
extern UWORD32 g_last_lat_sum;
extern UWORD32 g_last_lat_avg;
#endif /* LATENCY_TEST */

#ifdef CPU_UTIL_TEST
extern ALARM_HANDLE_T *g_cpu_util_alarm_handle;

extern UWORD8  g_test_in_prog;
extern UWORD8  g_i1;
extern UWORD8  g_i2;
extern UWORD16 g_num_iterations;
extern UWORD32 g_cpu_util_cnt;
extern UWORD32 g_cpu_util_cnt_calib_sum;
extern UWORD32 g_cpu_util_cnt_calib_avg;
extern UWORD32 g_cpu_util_cnt_sum;
extern UWORD32 g_cpu_util_cnt_avg;
extern UWORD32 g_cpu_util_arr1[128];
extern UWORD32 g_cpu_util_arr2[128];
#endif /* CPU_UTIL_TEST */

#ifdef MEASURE_PROCESSING_DELAY
extern delay_struct_t g_proc_delay_stats[NUM_PROC_DELAY_INDX_SUP];
extern delay_struct_t g_isr_delay_stats[NUM_ISR_DELAY_INDX_SUP];
extern UWORD32        g_isr_overhead[NUM_PROC_DELAY_INDX_SUP];
extern delay_stats_t  g_delay_stats;
#endif /* MEASURE_PROCESSING_DELAY */

/*****************************************************************************/
/* External Function Declarations                                            */
/*****************************************************************************/

#ifdef CPU_UTIL_TEST
#ifndef OS_LINUX_CSL_TYPE
extern void cpu_util_alarm_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void cpu_util_alarm_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */
#endif /* CPU_UTIL_TEST */

#ifdef LATENCY_TEST
extern void latency_test_fn(UWORD8 *dscr);
#endif /* LATENCY_TEST */

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/
#ifdef MEASURE_PROCESSING_DELAY
INLINE UWORD32 read_tsf_lo_timer(void)
{
    g_delay_stats.num_tsf_acesses++;
    //chenq mod
    //return convert_to_le((UWORD32)rMAC_TSF_TIMER_LO);
    return convert_to_le((UWORD32)host_read_trout_reg( 
                                        (UWORD32)rMAC_TSF_TIMER_LO));
}

INLINE void start_delay_measurement(DELAY_TYPE_T delay_type,
                                    UWORD32      delay_index)
{
    critical_section_start();

    if(PROC_DELAY == delay_type)
    {
        delay_struct_t* delay_stats = &g_proc_delay_stats[delay_index];

        delay_stats->timer_start = read_tsf_lo_timer();

        /* Clear ISR overhead for the current measurement index */
        g_isr_overhead[delay_index] = 0;
    }
    else
    {
        delay_struct_t *isr_delay_stats =
        &g_isr_delay_stats[delay_index];

        isr_delay_stats->timer_start    = read_tsf_lo_timer();
    }

    critical_section_end();
}

/* This fucntion stops delay measurement and updates the corresponding */
/* index in the global processing/isr statistics.                      */
INLINE void stop_delay_measurement(DELAY_TYPE_T delay_type,
                                   UWORD32      delay_index)
{

    UWORD32 timer_stop       = 0;
    UWORD32 timer_start      = 0;
    UWORD32 processing_delay = 0;

    critical_section_start();

    timer_stop = read_tsf_lo_timer();

    if(PROC_DELAY == delay_type)
    {
        delay_struct_t* proc_delay_stats = &g_proc_delay_stats[delay_index];

        timer_start = proc_delay_stats->timer_start;
        if(timer_stop > timer_start)
        {
            UWORD32 delay_without_isr_overhead = 0;

            processing_delay = timer_stop - timer_start;
            delay_without_isr_overhead = (processing_delay -
                                         g_isr_overhead[delay_index]);
            proc_delay_stats->delay_sum += delay_without_isr_overhead;
            proc_delay_stats->delay_cnt++;

            if(delay_without_isr_overhead >= proc_delay_stats->max_delay)
                proc_delay_stats->max_delay = delay_without_isr_overhead;

            if((0 == proc_delay_stats->min_delay) ||
               (delay_without_isr_overhead < proc_delay_stats->min_delay))
                proc_delay_stats->min_delay = delay_without_isr_overhead;

        }
    }
    else
    {
        delay_struct_t *isr_delay_stats  = &g_isr_delay_stats[delay_index];
        UWORD32 i = 0;

        timer_start = isr_delay_stats->timer_start;
        if(timer_stop > timer_start)
        {
            processing_delay            = timer_stop - timer_start;
            isr_delay_stats->delay_sum += processing_delay;
            isr_delay_stats->delay_cnt++;

            /* Update the ISR Overhead for all the Processing delays */
            for(i = 0; i < NUM_PROC_DELAY_INDX_SUP; i++)
                g_isr_overhead[i] += processing_delay;

            if(processing_delay >= isr_delay_stats->max_delay)
                isr_delay_stats->max_delay = processing_delay;

            if((0 == isr_delay_stats->min_delay) ||
               (processing_delay < isr_delay_stats->min_delay))
                isr_delay_stats->min_delay = processing_delay;
        }
    }

    critical_section_end();
}

INLINE void print_delay_statistics(void)
{
    UWORD32 i = 0;

    PRINTK("====== Event Processing delay Statistics ======\n\r");
    PRINTK("Indx, \tMaxDelay, \tMinDelay, \tTotalDelay, \tDelayCnt\n\r");
    for(i = 0; i < NUM_PROC_DELAY_INDX_SUP; i++)
    {
        PRINTK("%d, \t%d, \t%d, \t%d, \t%d\n\r", i,
                                             g_proc_delay_stats[i].max_delay,
                                             g_proc_delay_stats[i].min_delay,
                                             g_proc_delay_stats[i].delay_sum,
                                             g_proc_delay_stats[i].delay_cnt);

    }

    PRINTK("====== ISR Processing delay Statistics ======\n\r");
    for(i = 0; i < NUM_ISR_DELAY_INDX_SUP; i++)
    {
        PRINTK("%d, \t%d, \t%d, \t%d, \t%d\n\r", i,
                                               g_isr_delay_stats[i].max_delay,
                                               g_isr_delay_stats[i].min_delay,
                                               g_isr_delay_stats[i].delay_sum,
                                               g_isr_delay_stats[i].delay_cnt);

    }

    /* These Statistics are used in the Excel Sheet for delay calculations */
    PRINTK("======= Overall Delay Statistics =======\n\r");
    PRINTK("NumHIFInterrupts        = %d\n\r",g_delay_stats.hifrxint);
    PRINTK("TotalTimeHIFRxISR       = %d\n\r",
           g_isr_delay_stats[HIF_ISR_DELAY_INDEX].delay_sum);
    PRINTK("NumHostRxEvents         = %d\n\r",g_delay_stats.hostrxevent);
    PRINTK("NumRxPktsDroppedHIF     = %d\n\r",(g_delay_stats.hifrxint -
                                             g_delay_stats.hostrxevent));
    PRINTK("NumTxDscr2MacHwTxQ      = %d\n\r",g_delay_stats.txdscr2machwtxq);
    PRINTK("NumMSDUs2MacHwTxQ       = %d\n\r",g_delay_stats.submsdu2machwtxq);
    PRINTK("NumMSDUsDroppedMacHwTxQ = %d\n\r",(g_delay_stats.hostrxevent -
                                             g_delay_stats.submsdu2machwtxq));
    PRINTK("TotalTimeforHostRxEvent = %d\n\r",
           g_proc_delay_stats[HOST_RX_DELAY_INDEX].delay_sum);
    PRINTK("NumTxCompInterrupts     = %d\n\r",g_delay_stats.txcmpint);
    PRINTK("TotalTimeforTxCompISR   = %d\n\r",
           g_isr_delay_stats[MACHW_ISR_DELAY_INDEX].delay_sum);
    PRINTK("NumMISCTxCompEvents     = %d\n\r",g_delay_stats.misctxcompevent);
    PRINTK("NumMISCEvents           = %d\n\r",
           g_proc_delay_stats[MISC_DELAY_INDEX].delay_cnt);
    PRINTK("NumTxDscrOnTxComp       = %d\n\r",g_delay_stats.numtxdscrontxcmp);
    PRINTK("NumMSDUsOnTxComp        = %d\n\r",g_delay_stats.numsubmsduontxcmp);
    PRINTK("NumTxDscrFrmRetQ2HwTxQ  = %d\n\r",g_delay_stats.bamsduretq2mhwtxq);
    PRINTK("TotalTimeforMISCEvent   = %d\n\r",
           g_proc_delay_stats[MISC_DELAY_INDEX].delay_sum);
    PRINTK("NumHwRxCompInterrupts   = %d\n\r",g_delay_stats.rxcmpint);
    PRINTK("TotalTimeforRxCompISR   = %d\n\r",
           g_isr_delay_stats[MACHW_ISR_DELAY_INDEX].delay_sum);
    PRINTK("NumRxDscrOnRxComp       = %d\n\r",g_delay_stats.numrxdscr);
    PRINTK("NumWlanRxEvents         = %d\n\r",g_delay_stats.wlanrxevent);
    PRINTK("TotalTimeforWlanRxEvent = %d\n\r",
           g_proc_delay_stats[WLAN_RX_DELAY_INDEX].delay_sum);
    PRINTK("NumMSDUsDroppedatHIF    = %d\n\r",g_delay_stats.nummsdudrpedhiftxq);
    PRINTK("NumMSDUsTxed2Host       = %d\n\r",g_delay_stats.nummsduhiftxed);
    PRINTK("NumHostTxCompInterrupts = %d\n\r",g_delay_stats.hiftxint);
    PRINTK("TotalTimeHIFTxACKISR    = %d\n\r",
           g_isr_delay_stats[HIF_ISR_DELAY_INDEX].delay_sum);
    PRINTK("NumOfBARRxed            = %d\n\r", g_delay_stats.numbarrxd);
    PRINTK("NumOfBARTxed            = %d\n\r", g_delay_stats.numbartxd);
    PRINTK("NumOfMSDUsFrmPndQ2HwQ   = %d\n\r", g_delay_stats.bamsdupndq2mhwtxq);
}

INLINE void clear_delay_statistics(void)
{
    UWORD8 i = 0;
    int smart_type = MODE_START;

    smart_type = critical_section_smart_start(0,1);

    for(i = 0;i < NUM_PROC_DELAY_INDX_SUP; i++)
    {
        g_proc_delay_stats[i].max_delay = 0;
        g_proc_delay_stats[i].min_delay = 0;
        g_proc_delay_stats[i].delay_sum = 0;
        g_proc_delay_stats[i].delay_cnt = 0;
    }

    for(i = 0;i < NUM_ISR_DELAY_INDX_SUP; i++)
    {
        g_isr_delay_stats[i].max_delay = 0;
        g_isr_delay_stats[i].min_delay = 0;
        g_isr_delay_stats[i].delay_sum = 0;
        g_isr_delay_stats[i].delay_cnt = 0;
    }

    mem_set((UWORD8*)&g_delay_stats,
           0,
           sizeof(delay_stats_t));

    critical_section_smart_end(smart_type);
}

#endif /* MEASURE_PROCESSING_DELAY */

INLINE void update_cpu_util_count(void)
{
#ifdef CPU_UTIL_TEST

        /* Increment the CPU utilization count */
        g_cpu_util_cnt++;

#endif /* CPU_UTIL_TEST */
}

INLINE void start_cpu_util_test(void)
{
#ifdef CPU_UTIL_TEST

        /* Create the CPU utilization alarm, if not created */
        if(g_cpu_util_alarm_handle == 0)
            g_cpu_util_alarm_handle = create_alarm(cpu_util_alarm_fn, 0, NULL);

        start_alarm(g_cpu_util_alarm_handle, CPU_UTIL_TEST_TIME_SLICE);

#endif /* CPU_UTIL_TEST */
}

/* This function calls latency test function if latency test flag is defined */
INLINE void latency_test_function(UWORD8 *dscr)
{
#ifdef LATENCY_TEST
    latency_test_fn(dscr);
#endif /* LATENCY_TEST */
}

/* This function updates the descriptor with TSF value to calculate latency */
INLINE void latency_test_update(UWORD8* dscr)
{
#ifdef LATENCY_TEST
    UWORD32 hi = 0;
    UWORD32 lo = 0;

    get_machw_tsf_timer(&hi, &lo);
    set_tx_dscr_iv32((UWORD32*)dscr, lo);
#endif /* LATENCY_TEST */
}

INLINE void rx_latency_test_update1(UWORD8 *header)
{
#ifdef LATENCY_TEST
    if(get_type(header) == DATA)
    {
        UWORD32 hi;
        UWORD32 lo;

        get_machw_tsf_timer(&hi, &lo);
        g_rx_lat_1[g_rx_lat_index1] = lo;

        g_rx_lat_index1 = (g_rx_lat_index1 + 1) % 128;
    }
#endif /* LATENCY_TEST */
}

INLINE void rx_latency_test_update2(void)
{
#ifdef LATENCY_TEST
    UWORD32 hi;
    UWORD32 lo;

    get_machw_tsf_timer(&hi, &lo);
    g_rx_lat_2[g_rx_lat_index2] = lo;

    g_rx_lat_index2 = (g_rx_lat_index2 + 1) % 128;
#endif /* LATENCY_TEST */
}

INLINE void rx_latency_test_update3(void)
{
#ifdef LATENCY_TEST
    UWORD32 hi;
    UWORD32 lo;

    get_machw_tsf_timer(&hi, &lo);
    g_rx_lat_3[g_rx_lat_index3] = lo;

    g_rx_lat_index3 = (g_rx_lat_index3 + 1) % 128;
#endif /* LATENCY_TEST */
}

#endif /* METRICS_H */
