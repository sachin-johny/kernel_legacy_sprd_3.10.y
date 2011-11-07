/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VRPC driver interface
 *
 *  Copyright (C) 2010-2011, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Vladimir Grouzdev <vladimir.grouzdev@virtuallogix.com>
 *
 ****************************************************************
 */

#ifndef _VRPC_COMMON_H_
#define _VRPC_COMMON_H_

typedef struct vrpc_pmem_t {
    nku32_f req;	/* request counter */
    nku32_f ack;	/* acknowledge counter */
    nku32_f size;	/* size of RPC in/out data */
    nku32_f data[];	/* RPC data */
} vrpc_pmem_t;

#define	VRPC_PMEM_DEF_SIZE	1024

#endif
