/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: plt_cfg.h
 * Abstract: 
 *     
 * Author:   Lucy
 * Version:  1.00
 * Revison Log:
 *     2012/03/18, Lucy: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _PLT_CFG_H
#define _PLT_CFG_H

#define CONFIG_FILE "testbenchCfg.txt"

/* typedef struct testbench_cfg_s
{
    char deDevType;
    char deDevName[MAX_CFG_VALUE_LEN];
    char deDevMod[MAX_CFG_VALUE_LEN];
    char dvDevName[MAX_CFG_VALUE_LEN];
    char dvDevMod[MAX_CFG_VALUE_LEN];
    char logName[MAX_CFG_VALUE_LEN];
    char caseDbName[MAX_CFG_VALUE_LEN];
    char resultDbname[MAX_CFG_VALUE_LEN];
}TESTBENCH_CFG_T; */
    
typedef struct plt_cfg_s
{
    char deDevType;
    char *deDevName;
    char *deDevMod;
    char *deDevFile;
    char *dvDevName;
    char *dvDevMod;
    char *dvDevFile;
    char *logName;
    char *caseDbName;
    char *resultDbname;
}PLT_CFG_T; 


PLT_CFG_T *pltCfgInfoGet(void);
int pltCfgInit(void);
void pltCfgExit(void);
#endif /* #ifndef _PLT_CFG_H */

/*
 *$Log$
 */

