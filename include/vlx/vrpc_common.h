/*
 ****************************************************************
 *
 * Copyright (C) 2010 VirtualLogix inc. All Rights Reserved.
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
