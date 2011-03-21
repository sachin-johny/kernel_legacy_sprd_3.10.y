/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VMQ driver interface
 *
 *  Copyright (C) 2009-2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski <adam.mirowski@virtuallogix.com>
 *
 ****************************************************************
 */

#ifndef VLX_VMQ_H
#define VLX_VMQ_H

#if LINUX_VERSION_CODE <= KERNEL_VERSION (2,6,18)
#define false	0
#define true	1
#endif

typedef struct vmq_link_t  vmq_link_t;
typedef struct vmq_links_t vmq_links_t;

typedef struct {
    void*	priv;		/* Must be first */
    NkOsId	local_osid;
    NkOsId	peer_osid;
    char*	rx_s_info;
    char*	rx_data_area;
    char*	tx_data_area;
    unsigned	data_max;
    unsigned	msg_max;
} vmq_link_public_t;

    /* Link control callbacks */

typedef struct {
    unsigned	msg_count;
    unsigned	msg_max;
    unsigned	data_count;
    unsigned	data_max;
} vmq_xx_config_t;

typedef struct {
    void (*link_on)		(vmq_link_t*);
    void (*link_off)		(vmq_link_t*);
    void (*link_off_completed)	(vmq_link_t*);
    void (*sysconf_notify)	(vmq_links_t*);
    void (*receive_notify)	(vmq_link_t*);
    void (*return_notify)	(vmq_link_t*);
    int  (*get_tx_config)	(vmq_links_t*, const char* s_info,
				 vmq_xx_config_t*);
} vmq_callbacks_t;

#ifndef __must_check
    /* Does not exist e.g. in 2.6.0 */
#define __must_check
#endif

    /* Communication functions */
static signed
	vmq_msg_allocate	(vmq_link_t*, unsigned data_len, void** msg,
				 unsigned* data_offset) __must_check;
signed	vmq_msg_allocate_ex	(vmq_link_t*, unsigned data_len, void** msg,
				 unsigned* data_offset, _Bool nonblocking)
				 __must_check;
void	vmq_msg_send		(vmq_link_t*, void* msg);
signed	vmq_msg_receive		(vmq_link_t*, void** msg) __must_check;
void	vmq_msg_free		(vmq_link_t*, void* msg);
void	vmq_msg_return		(vmq_link_t*, void* msg);
_Bool	vmq_data_offset_ok	(vmq_link_t*, unsigned data_offset);
void	vmq_data_free		(vmq_link_t*, unsigned data_offset);
signed	vmq_return_msg_receive	(vmq_link_t* link, void** msg)
				 __must_check;
void	vmq_return_msg_free	(vmq_link_t* link, void* msg);

    /* Link control functions */
signed	vmq_links_init		(vmq_links_t**, const char* vlink_name,
				 const vmq_callbacks_t*,
				 const vmq_xx_config_t* tx_config,
				 const vmq_xx_config_t* rx_config)
				 __must_check;
void	vmq_links_finish	(vmq_links_t*);
_Bool	vmq_links_iterate	(vmq_links_t*, _Bool (*func)(vmq_link_t*,
				 void*), void* cookie);
void	vmq_links_sysconf	(vmq_links_t*);
void	vmq_links_abort		(vmq_links_t*);

    static inline signed
vmq_msg_allocate (vmq_link_t* link, unsigned data_len, void** msg,
		  unsigned* data_offset)
{
    return vmq_msg_allocate_ex (link, data_len, msg, data_offset,
				0 /*!nonblocking*/);
}

    static inline NkOsId
vmq_peer_osid (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->peer_osid;
}

    static inline const char*
vmq_link_s_info (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->rx_s_info;
}

    static inline char*
vmq_rx_data_area (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->rx_data_area;
}

    static inline char*
vmq_tx_data_area (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->tx_data_area;
}

    static inline unsigned
vmq_data_max (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->data_max;
}

    static inline unsigned
vmq_msg_max (const vmq_link_t* link)
{
    return ((vmq_link_public_t*) link)->msg_max;
}

#endif

