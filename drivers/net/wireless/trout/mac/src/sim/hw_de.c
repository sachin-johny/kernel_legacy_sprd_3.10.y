/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_de.c
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/
#include <linux/types.h>
#include "sim.h"
#include "virt_irq.h"

int deIrqInfoRegister(unsigned int irq,
                          irq_handler_t handler,
                          unsigned long flags,
                          const char *name,
                          void *dev)
{
	return register_virt_irq(irq, handler, flags, name, dev);
}
EXPORT_SYMBOL(deIrqInfoRegister);

int hwDeDataHookCopyIn(char *pBuf, int bufLen, int kernMode)
{
	// TODO
	//return data_hook_in(pBuf, bufLen, kernMode);
	return 0;
}
EXPORT_SYMBOL(hwDeDataHookCopyIn);

void hwDeLogEndMsg(void)
{
   
	DV_LOG_CELL_T log; 
	log.valid = 1; 
	log.type = DV_LOG_TYPE_DONE; 
	log.dataLen = 0;
	dvLog(&log,NULL);
}
EXPORT_SYMBOL(hwDeLogEndMsg);

/*
 *$Log$
 */

