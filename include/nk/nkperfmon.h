/*
 ****************************************************************
 *
 * Component = Jaluna OSware Performance Monitoring
 *
 * Copyright (C) 2006, Jaluna SA. All Rights Reserved.
 *
 * Use of this product is contingent on the existence of an executed license agreement between
 * Jaluna or one of its sublicensee, and your organization, which specifies this software's
 * terms of use. This software falls under is here defined as Jaluna Intellectual Property
 * for the purposes of determining terms of use as defined within the license agreement.
 *
 * #ident "@(#)nkperfmon.h 1.3 06/07/25 Jaluna"
 *
 * Contributor(s):
 *    Chi Dat Truong <chidat.truong@jaluna.com>
 *
 ****************************************************************
 */

#ifndef OSWARE_PERFMON_H
#define OSWARE_PERFMON_H

//#include <osware/osware.h>
#include <nk/nkern.h>
#include <asm/nk/f_nk.h>
#include <asm/nkern.h>

#ifdef __c6x__
#define os_ctx nkctx
#endif

#define PMON_CPU_STATS_SIZE          32
#define PMON_MISC_STATS_SIZE         8

typedef struct NkPmonRecord {
    NkTime         stamp;    // time stamp
    NkPmonState    state;    // new state transition
    NkPhAddr       cookie;   // guest specific info
} NkPmonRecord;

typedef struct NkPmonCpuStats {
    NkTime  startstamp;                      // statistic start time stamp
    NkTime  laststamp;                       // statistic last time stamp
    NkTime  cpustats[PMON_CPU_STATS_SIZE];   // statistic array for 32 OSes
    NkTime  miscstats[PMON_MISC_STATS_SIZE]; // miscellaneous statistic array
} NkPmonCpuStats;

typedef struct NkPmonBuffer {    // circular buffer with header
    nku32_f        first;      // index of the start of the record buffer
    nku32_f        last;       // index of the end of the record buffer
    nku32_f        length;     // the length of the record array (number of elements)
    nku32_f        padding;    // for 64 bit alignment
    NkPmonRecord   data[0];     // array of NkPmonRecord
} NkPmonBuffer;

#define PMON_GET_RECORD_LENGTH(bufferSize) \
            ( ((bufferSize) - sizeof(NkPmonBuffer)) / sizeof(NkPmonRecord) )

/* User can define sub-state events for record for this OS */
//#define    PMON_EVENT_****		0x1

#endif
