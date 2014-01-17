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
/*  File Name         : metrics.c                                            */
/*                                                                           */
/*  Description       : This file contains all the globals and functions     */
/*                      used in getting the metrics like Latency and CPU     */
/*                      Utilizations.                                        */
/*                                                                           */
/*  List of Functions : cpu_util_break                                       */
/*                      cpu_util_break1                                      */
/*                      cpu_util_alarm_fn                                    */
/*                      latency_break                                        */
/*                      latency_test_fn                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "metrics.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

#ifdef CPU_UTIL_TEST
ALARM_HANDLE_T *g_cpu_util_alarm_handle = 0;

UWORD8  g_test_in_prog           = 0;
UWORD8  g_i1                     = 0;
UWORD8  g_i2                     = 0;
UWORD16 g_num_iterations         = 0;
UWORD32 g_cpu_util_cnt           = 0;
UWORD32 g_cpu_util_cnt_calib_sum = 0;
UWORD32 g_cpu_util_cnt_calib_avg = 0;
UWORD32 g_cpu_util_cnt_sum       = 0;
UWORD32 g_cpu_util_cnt_avg       = 0;

UWORD32 g_cpu_util_arr1[128]     = {0};
UWORD32 g_cpu_util_arr2[128]     = {0};

UWORD32 g_cpu_util_min1          = 0;
UWORD32 g_cpu_util_max1          = 0;
UWORD32 g_cpu_util_mean1         = 0;

UWORD32 g_cpu_util_min2          = 0;
UWORD32 g_cpu_util_max2          = 0;
UWORD32 g_cpu_util_mean2         = 0;

#endif /* CPU_UTIL_TEST */

#ifdef LATENCY_TEST

UWORD32 g_rx_lat_1[128] = {0};
UWORD32 g_rx_lat_2[128] = {0};
UWORD32 g_rx_lat_3[128] = {0};

UWORD32 g_rx_lat_index1 = 0;
UWORD32 g_rx_lat_index2 = 0;
UWORD32 g_rx_lat_index3 = 0;

UWORD32 g_min_tx_lat   = 0xFFFFFFFF;
UWORD32 g_max_tx_lat   = 0;
UWORD32 g_lat_cnt      = 0;
UWORD32 g_tx_lat_sum   = 0;
UWORD32 g_avg_tx_lat   = 0;
UWORD32 g_last_lat_sum = 0;
UWORD32 g_last_lat_avg = 0;
UWORD32 g_lat[128]     = {0};
UWORD16 gi             = 0;
#endif /* LATENCY_TEST */

#ifdef MEASURE_PROCESSING_DELAY
delay_struct_t g_proc_delay_stats[NUM_PROC_DELAY_INDX_SUP] = {{0},};
delay_struct_t g_isr_delay_stats[NUM_ISR_DELAY_INDX_SUP]   = {{0},};
UWORD32        g_isr_overhead[NUM_PROC_DELAY_INDX_SUP]     = {0};
delay_stats_t  g_delay_stats = {0};
#endif /* MEASURE_PROCESSING_DELAY */


#ifdef CPU_UTIL_TEST
/*****************************************************************************/
/*                                                                           */
/*  Function Name : cpu_util_calc                                            */
/*                                                                           */
/*  Description   : This function is used to calculate the min,max,mean of   */
/*                  the 128 values stored for CPU utilization counters       */
/*                                                                           */
/*  Inputs        : Pointer to the array of 128 values                       */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : The min,max,mean calculated values                       */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void cpu_util_calc(UWORD32 *cpu_util_arr,UWORD32 *cpu_util_min,
                   UWORD32 *cpu_util_max,UWORD32 *cpu_util_mean)
{
    UWORD8 i,j,k,accum_count;
    UWORD8 num_in_each_bin[CPU_UTIL_NUM_BINS];

    UWORD32 limit[CPU_UTIL_NUM_BINS];
    UWORD32 lower_limit,upper_limit;
    UWORD32 min_value,max_value;
    UWORD32 accum;

    /* Initialize the bin thresholds */
    for(i = 0; i < CPU_UTIL_NUM_BINS; i++)
    {
        limit[i] = (UWORD32) (((double)(i+1)/CPU_UTIL_NUM_BINS)* CPU_UTIL_CUT_OFF_VALUE);
        num_in_each_bin[i] = 0;
    }

    /* Fill the bins */
    for(i = 0; i < 128; i++)
    {
        j = 0;
        while(j < CPU_UTIL_NUM_BINS)
        {
            if(cpu_util_arr[i] < limit[j])
            {
                num_in_each_bin[j]++;
                break;
            }

            j++;
        }
    }

    k = 0;
    i = 0;

    /* Find the starting bin of a concentration/group */
    while(i < CPU_UTIL_NUM_BINS)
    {
        if(num_in_each_bin[i] > 5)
        {
            k = i;
            break;
        }
        else if(num_in_each_bin[i] > num_in_each_bin[k])
        {
            k = i;
        }

        i++;
    }

    /* Choose the range depending on the concentration*/
    /* Choose the lower limit*/
    if(k == 0)
    {
        lower_limit = 0;
    }
    else
    {
        lower_limit = limit[k-1];
    }

    j = k;

    /*Choose the upper limit*/
    while(j < CPU_UTIL_NUM_BINS-1)
    {
        if(num_in_each_bin[j+1] <= 5)
              break;
        else
              j++;
    }

    upper_limit = limit[j];


    /* Find the minimum, maximum and average of those in choosen range */
    min_value   = (UWORD32) CPU_UTIL_CUT_OFF_VALUE;
    max_value   = 0;
    accum       = 0;
    accum_count = 0;

    for(i = 0; i < 128; i++)
    {
        if((cpu_util_arr[i] < upper_limit) &&
           (cpu_util_arr[i] > lower_limit))
        {
            accum += cpu_util_arr[i];
            accum_count++;

            if(min_value > cpu_util_arr[i])
                min_value = cpu_util_arr[i];

            if(max_value < cpu_util_arr[i])
                max_value = cpu_util_arr[i];
        }
    }

    /* Mean value */
    *cpu_util_mean = (UWORD32) (accum/accum_count);
    /* Minimum value */
    *cpu_util_min = min_value;
    /* Maximum value */
    *cpu_util_max = max_value;
 }
/*****************************************************************************/
/*                                                                           */
/*  Function Name : cpu_util_break                                           */
/*                                                                           */
/*  Description   : This function is used as a break point function.         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void cpu_util_break(void)
{
    led_display(0xAA);

    /* CPU utilization calibration metric calculator */
    cpu_util_calc(g_cpu_util_arr1,&g_cpu_util_min1,
                  &g_cpu_util_max1,&g_cpu_util_mean1);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : cpu_util_break1                                          */
/*                                                                           */
/*  Description   : This function is used as a break point function.         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void cpu_util_break1(void)
{
    led_display(0xFF);

    /* CPU utilization metric calculator */
    cpu_util_calc(g_cpu_util_arr2,&g_cpu_util_min2,
                  &g_cpu_util_max2,&g_cpu_util_mean2);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : cpu_util_alarm_fn                                        */
/*                                                                           */
/*  Description   : This function is the CPU utilization test alarm function */
/*                  that saves the calibration counts.                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void cpu_util_alarm_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void cpu_util_alarm_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    /* Save the CPU utiltization count for this time slice */
    if(g_num_iterations < CPU_UTIL_TEST_CALIB_COUNT)
    {
        g_cpu_util_arr1[g_i1]  = g_cpu_util_cnt;
        g_i1 = (g_i1 + 1) % 128;
    }
    else if(g_num_iterations < CPU_UTIL_TEST_COUNT)
    {
        g_cpu_util_arr2[g_i2]  = g_cpu_util_cnt;
        g_i2 = (g_i2 + 1) % 128;
    }

    /* Reset the count */
    g_cpu_util_cnt = 0;

    /* Increment the number of iterations and glow LEDs if the number of     */
    /* iterations have reached the limit.                                    */
    g_num_iterations++;

    if(g_num_iterations == CPU_UTIL_TEST_CALIB_COUNT)
    {
        UWORD16 cnt = 0;
        for(cnt = 0; cnt < 128; cnt++)
        {
            g_cpu_util_cnt_calib_sum += g_cpu_util_arr1[cnt];
        }
        g_cpu_util_cnt_calib_avg = g_cpu_util_cnt_calib_sum / 128;
        cpu_util_break();
    }

    if(g_num_iterations == CPU_UTIL_TEST_COUNT)
    {
        UWORD16 cnt = 0;
        for(cnt = 0; cnt < 128; cnt++)
        {
            g_cpu_util_cnt_sum += g_cpu_util_arr2[cnt];
        }
        g_cpu_util_cnt_avg = g_cpu_util_cnt_sum / 128;
        cpu_util_break1();
    }
    else
    {
        start_alarm(g_cpu_util_alarm_handle, CPU_UTIL_TEST_TIME_SLICE);
    }
}
#endif /* CPU_UTIL_TEST */


#ifdef LATENCY_TEST
/*****************************************************************************/
/*                                                                           */
/*  Function Name : latency_break                                            */
/*                                                                           */
/*  Description   : This function is used as a break point function.         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void latency_break()
{
    /* Do nothing */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : latency_test_fn                                          */
/*                                                                           */
/*  Description   : This function calculates the latency and updates the     */
/*                  corresponding variables.                                 */
/*                                                                           */
/*  Inputs        : 1) Transmit descriptor                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void latency_test_fn(UWORD8 *dscr)
{
    UWORD32 hi  = 0;
    UWORD32 lo  = 0;
    UWORD32 ts  = 0;
    UWORD32 lat = 0;

    get_machw_tsf_timer(&hi, &lo);
    ts = get_tx_dscr_iv32l((UWORD32*)dscr);

    lat = lo - ts;

    g_lat[gi] = lat;
    gi = (gi + 1)%128;

    if(lat < g_min_tx_lat)
        g_min_tx_lat = lat;

    if(lat > g_max_tx_lat)
        g_max_tx_lat = lat;

    g_lat_cnt++;

    g_tx_lat_sum += lat;

    if(g_lat_cnt == NUM_LATENCY_PKT_COUNT)
    {
         UWORD8 li = 0;
         for(li = 0;li < 128; li++)
         {
             g_last_lat_sum += g_lat[li];
         }
         g_last_lat_avg = g_last_lat_sum / 128;
         g_avg_tx_lat = g_tx_lat_sum/g_lat_cnt;
         latency_break();
    }
}

#endif /* LATENCY_TEST */
