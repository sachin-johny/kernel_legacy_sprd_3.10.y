/*
 ****************************************************************
 *
 *  Component:	VirtualLogix virtual Android Physical Memory
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *   Christophe Lizzi (Christophe.Lizzi@virtuallogix.com)
 *
 ****************************************************************
 */

#ifndef _VPMEM_H_
#define _VPMEM_H_

// vpmem API for use by the other virtual drivers

typedef void* vpmem_handle_t;

vpmem_handle_t vpmem_lookup (char* name);
unsigned char* vpmem_map    (vpmem_handle_t handle);
void           vpmem_unmap  (vpmem_handle_t handle);
unsigned long  vpmem_phys   (vpmem_handle_t handle);
unsigned int   vpmem_size   (vpmem_handle_t handle);
unsigned int   vpmem_id     (vpmem_handle_t handle);

#endif // _VPMEM_H_
