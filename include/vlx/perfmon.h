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
 * #ident "@(#)perfmon.h 1.3 06/07/25 Jaluna"
 *
 * Contributor(s):
 *    Chi Dat Truong <chidat.truong@jaluna.com>
 *
 ****************************************************************
 */

#ifndef PERFMON_H
#define PERFMON_H

#define PMON_MAX_SUPPORTED_OS        32    // maximum of supported OS
#define PMON_MISC_STATS_SIZE          8    // number of miscellaneous statistics
#define PMON_SUPPORTED_SERVICE        2    // number of service supported

/*
 * CONTROL COMMANDS
 */
#define CMD_SWITCH          "PMON_CTRL_SWITCH"
#define CMD_ONESHOT_START   "PMON_CTRL_ONESHOT_START"
#define CMD_STATS_START     "PMON_CTRL_STATS_START"
#define CMD_START           "PMON_CTRL_START"
#define CMD_STOP            "PMON_CTRL_STOP"
#define CMD_IS_WORKING      "PMON_CTRL_IS_WORKING"

/*
 * User data interface
 */

typedef struct PmonTimerInfo {
    unsigned long       freq;
    unsigned long long  period;
} PmonTimerInfo;

typedef struct PmonCpuStats {
    unsigned long long    startstamp;    // statistic start time stamp
    unsigned long long    laststamp;     // statistic last time stamp
    unsigned long long    cpustats[PMON_MAX_SUPPORTED_OS];  // statistic array for 32 OSes
    unsigned long long    miscstats[PMON_MISC_STATS_SIZE];  // miscellaneous statistic array
} PmonCpuStats;

typedef struct PmonSysInfo {
    char             version[8];    // string of version number
    unsigned long    last_os_id;    // last os id number
    PmonTimerInfo    timer;         // timer information (frequency, ...)
    unsigned long    max_records;   // maximum of record elements
} PmonSysInfo;

typedef struct PmonRecordData {
    unsigned long long    stamp;    // time stamp
    unsigned long         state;    // new state transition
    unsigned long         cookie;   // guest specific info
} PmonRecordData;

#endif
