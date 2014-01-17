/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: sim.h
 * Abstract: 
 *     
 * Author:   Lucy
 * Version:  1.00
 * Revison Log:
 *     2012/03/14, Lucy: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _SIM_H
#define _SIM_H

#include <linux/types.h>

#include "kern_comm.h"
#include "reg_def.h"
#include "debug.h"
#include "hw_sim.h"
#include "error.h"
#include "dv_ioctl.h"
#include "hw_dv.h"
#include "dv_log.h"



int dvLog(DV_LOG_CELL_T *logCell,u32 *data);  //get log message


#endif /* #ifndef _SIM_H */

/*
 *$Log$
 */

