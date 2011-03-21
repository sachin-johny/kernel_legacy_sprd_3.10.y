/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VRPC driver interface
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Vladimir Grouzdev <vladimir.grouzdev@virtuallogix.com>
 *
 ****************************************************************
 */

typedef unsigned int vrpc_size_t;

struct vrpc_t;    

   typedef void
(*vrpc_ready_t) (void* cookie);

   typedef vrpc_size_t
(*vrpc_call_t) (void* cookie, vrpc_size_t size);

    extern struct vrpc_t*
vrpc_server_lookup (const char* name, struct vrpc_t* last);

    extern struct vrpc_t*
vrpc_client_lookup (const char* name, struct vrpc_t* last);

    extern void
vrpc_release (struct vrpc_t* vrpc);

    extern NkOsId
vrpc_peer_id (struct vrpc_t* vrpc);

    extern void*
vrpc_data (struct vrpc_t* vrpc);

    extern vrpc_size_t
vrpc_maxsize (struct vrpc_t* vrpc);

    extern int
vrpc_server_open (struct vrpc_t* vrpc, vrpc_call_t call, void* cookie,
		  int direct);

    extern int
vrpc_client_open (struct vrpc_t* vrpc, vrpc_ready_t ready, void* cookie);

    extern int
vrpc_call (struct vrpc_t* vrpc, vrpc_size_t* size);

    extern void
vrpc_close (struct vrpc_t* vrpc);



