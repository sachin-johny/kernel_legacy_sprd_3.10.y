/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VRPC Backend/Frontend Driver
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Vladimir Grouzdev (vladimir.grouzdev@vlx.com)
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,27)
#include <linux/semaphore.h>
#endif
#include <linux/mutex.h>
#include <linux/slab.h>

#include <nk/nkern.h>
#include <vlx/vrpc_common.h>

#include "vrpc.h"

#define TRACE(format, args...) 	printk("VRPC: " format, ## args)
#define ETRACE(format, args...)	printk("VRPC: [E] " format, ## args)

#if 0
#define DTRACE(format, args...) printk("VRPC: [D] " format, ## args)
#else
#define DTRACE(format, args...) 
#endif

typedef struct vrpc_end_t {
    NkOsId            id;	/* VM ID */
    NkXIrq            xirq;	/* XIRQ ID */
    volatile nku32_f* state;  	/* state */
    const char*       info;	/* info */
} vrpc_end_t;

    typedef void
(*vrpc_wrapper_t) (struct vrpc_t*);

typedef struct vrpc_t {
    NkPhAddr            plink;	/* server/client link physical address */
    NkDevVlink*         vlink;  /* server/client link */
    nku32_f             msize;	/* maximum data size */
    vrpc_pmem_t*        pmem;   /* persistent shared memory */
    vrpc_end_t          my;	/* my link end-point */
    vrpc_end_t          peer;	/* peer link end-point */
    NkXIrqId            xid; 	/* cross interrupt ID */
    int                 used;	/* VRPC channel is used */
    vrpc_ready_t        ready;	/* user ready handler (client) */
    vrpc_call_t         call;	/* user call handler (server) */
    void*               cookie;	/* user cookie */
    vrpc_wrapper_t      wrapper;/* RPC wrapper */
    struct task_struct* thread; /* thread completion structure */
    wait_queue_head_t   wait;   /* wait queue */
    int                 open;	/* the link is open */
    struct vrpc_t*      next;	/* next RPC descriptor */
} vrpc_t;

static NkXIrqId     _scid;      /* sysconf cross interrupt id */
static vrpc_t*      _vrpcs;	/* list of virtual RPCs */
static struct mutex _mutex;	/* exclusive lookup mutex */
static int          _aborted;	/* the module is aborted */

    static void
_vrpc_xirq_post (vrpc_t* vrpc)
{
    DTRACE("Sending xirq VM#%d (%d) -> VM#%d (%d) [vrpc=0x%p]\n",
            vrpc->my.id, *vrpc->my.state, vrpc->peer.id, *vrpc->peer.state,
            vrpc);
    nkops.nk_xirq_trigger(vrpc->peer.xirq, vrpc->peer.id);
}

    static void
_vrpc_sysconf_post (vrpc_t* vrpc)
{
    DTRACE("Sending sysconf VM#%d (%d) -> VM#%d (%d) [vrpc=0x%p]\n",
           vrpc->my.id, *vrpc->my.state, vrpc->peer.id, *vrpc->peer.state,
           vrpc);
    nkops.nk_xirq_trigger(NK_XIRQ_SYSCONF, vrpc->peer.id);
}

    static inline int
_vrpc_is_server (vrpc_t* vrpc)
{
    return (vrpc->my.id == vrpc->vlink->s_id);
}

    static inline int
_vrpc_is_client (vrpc_t* vrpc)
{
    return (vrpc->my.id == vrpc->vlink->c_id);
}

    static void
_vrpc_wakeup (vrpc_t* vrpc)
{
    wake_up_interruptible(&vrpc->wait);
}

    static void
_vrpc_pmem_reset (vrpc_t* vrpc)
{
}

    static void
_vrpc_pmem_init (vrpc_t* vrpc)
{
    vrpc_pmem_t* pmem = vrpc->pmem;
    if (_vrpc_is_server(vrpc)) {
	pmem->ack = 0;
    } else {
	pmem->req = 0;
    }    
}

    static int
_vrpc_link_ready (vrpc_t* vrpc)
{
    return (*vrpc->my.state   == NK_DEV_VLINK_ON) &&
           (*vrpc->peer.state == NK_DEV_VLINK_ON);
}

    static int
_vrpc_ready (vrpc_t* vrpc)
{
    return (!_aborted && _vrpc_link_ready(vrpc));
}

    static void
_vrpc_null_call (vrpc_t* vrpc)
{
    vrpc_pmem_t* pmem = vrpc->pmem;
    pmem->size = 0;
    pmem->ack  = pmem->req;
    _vrpc_xirq_post(vrpc);
}

    static void
_vrpc_direct_call (vrpc_t* vrpc)
{
    vrpc_call_t  call = vrpc->call;
    vrpc_pmem_t* pmem = vrpc->pmem;
    if (_vrpc_ready(vrpc) && (pmem->req != pmem->ack)) {
        if (call) {
            pmem->size = call(vrpc->cookie, pmem->size);
        } else {
            pmem->size = 0;
        }
        pmem->ack = pmem->req;
        _vrpc_xirq_post(vrpc);
    }
}

    static void
_vrpc_indirect_call (vrpc_t* vrpc)
{
    _vrpc_wakeup(vrpc);
}

    static int
_vrpc_handshake (vrpc_t* vrpc)
{
    volatile int* my_state;
    int           peer_state;

    my_state   = vrpc->my.state;
    peer_state = *vrpc->peer.state;

    DTRACE("handshake VM#%d (%d) -> VM#%d (%d) [vrpc=%p]\n",
            vrpc->peer.id, peer_state, vrpc->my.id, *my_state, vrpc);

    switch (*my_state) {
        case NK_DEV_VLINK_OFF:
            if (peer_state != NK_DEV_VLINK_ON) {
		_vrpc_pmem_reset(vrpc);
                *my_state = NK_DEV_VLINK_RESET;
                _vrpc_sysconf_post(vrpc);
            }
            break;
        case NK_DEV_VLINK_RESET:
            if (peer_state != NK_DEV_VLINK_OFF) {
                *my_state = NK_DEV_VLINK_ON;
		_vrpc_pmem_init(vrpc);
                _vrpc_sysconf_post(vrpc);
            }
            break;
        case NK_DEV_VLINK_ON:
            if (peer_state == NK_DEV_VLINK_OFF) {
                *my_state = NK_DEV_VLINK_RESET;
                _vrpc_sysconf_post(vrpc);
            }
            break;
    }

    return (*my_state  == NK_DEV_VLINK_ON) &&
           (peer_state == NK_DEV_VLINK_ON);
}

    static void
_vrpc_sysconf_handler (void* cookie, NkXIrq xirq)
{
    vrpc_t* vrpc = _vrpcs;
    while (vrpc) {
        if (vrpc->open) {
	    if (_vrpc_is_server(vrpc)) {
		_vrpc_handshake(vrpc);
	    } else {
		_vrpc_wakeup(vrpc);
	    }
	}
	vrpc = vrpc->next;
    }
}

    static void
_vrpc_xirq_handler (void* cookie, NkXIrq xirq)
{
    vrpc_t* vrpc = cookie;    
    vrpc->wrapper(vrpc);
}

    static int
_vrpc_my_xirq_id (vrpc_t* vrpc)
{
    return (_vrpc_is_server(vrpc) ? 0 : 1);
}

    static int
_vrpc_peer_xirq_id (vrpc_t* vrpc)
{
    return (_vrpc_is_server(vrpc) ? 1 : 0);
}

    static int __init
_vrpc_pxirq_alloc (vrpc_t* vrpc)
{
    NkPhAddr plink = vrpc->plink;
    int      mid   = _vrpc_my_xirq_id(vrpc);
    int      pid   = _vrpc_peer_xirq_id(vrpc);
    vrpc->my.xirq   = nkops.nk_pxirq_alloc(plink, mid, vrpc->my.id,   1);
    vrpc->peer.xirq = nkops.nk_pxirq_alloc(plink, pid, vrpc->peer.id, 1);
    return (vrpc->my.xirq && vrpc->peer.xirq);
}

    //
    // Convertion from character string to integer number
    //
    static const char* __init
_a2ui (const char* s, unsigned int* i)
{
    unsigned int xi = 0;    
    char         c  = *s;

    while (('0' <= c) && (c <= '9')) {
	xi = xi * 10 + (c - '0');
	c = *(++s);
    }

    if        ((*s == 'K') || (*s == 'k')) {
	xi *= 1024;
	s  += 1;
    } else if ((*s == 'M') || (*s == 'm')) {
	xi *= (1024*1024);
	s  += 1;
    }

    *i = xi;

    return s;
}

    static vrpc_size_t __init
_pmem_info_size (const char* info)
{
    vrpc_size_t size;
    if (!info) {
	return VRPC_PMEM_DEF_SIZE;
    }
    while (*info && (*info != ',')) info++;
    if (!*info) {
	return VRPC_PMEM_DEF_SIZE;
    }
    info = _a2ui(info+1, &size);
    if (*info) {
	return VRPC_PMEM_DEF_SIZE;
    }
    if (size < sizeof(vrpc_pmem_t)) {
	size = sizeof(vrpc_pmem_t);
    }
    return size;
}

    static vrpc_size_t __init
_vrpc_pmem_size (vrpc_t* vrpc)
{
    vrpc_size_t msize = _pmem_info_size(vrpc->my.info);
    vrpc_size_t psize = _pmem_info_size(vrpc->peer.info);
    return (msize > psize ? msize : psize);
}

    static int __init
_vrpc_pmem_alloc (vrpc_t* vrpc)
{
    vrpc_size_t size  = _vrpc_pmem_size(vrpc);
    NkPhAddr    paddr = nkops.nk_pmem_alloc(vrpc->plink, 0, size);
    if (!paddr) {
	return 0;
    }
    vrpc->pmem  = nkops.nk_mem_map(paddr, size);
    vrpc->msize = size - sizeof(vrpc_pmem_t) + 4;
    return 1;
}

    static int __init
_vrpc_link_init (NkDevVlink* vlink, NkPhAddr plink)
{
    vrpc_t* vrpc;

    vrpc = kzalloc(sizeof(vrpc_t), GFP_KERNEL);
    if (!vrpc) {
  	ETRACE("VLINK %d (%d -> %d) memory allocation failed\n",
	        vlink->link, vlink->c_id, vlink->s_id);
	return -ENOMEM;
    }

    vrpc->vlink   = vlink;
    vrpc->plink   = plink;
    vrpc->my.id   = nkops.nk_id_get();
    vrpc->wrapper = _vrpc_null_call;

    if (_vrpc_is_server(vrpc)) {
	vrpc->my.state   = &vlink->s_state;
        if (vlink->s_info) {
            vrpc->my.info = nkops.nk_ptov(vlink->s_info);
  	}
	vrpc->peer.id    = vlink->c_id;
	vrpc->peer.state = &vlink->c_state;
        if (vlink->c_info) {
            vrpc->peer.info = nkops.nk_ptov(vlink->c_info);
  	}
    } else {
	vrpc->my.state   = &vlink->c_state;
        if (vlink->c_info) {
            vrpc->my.info = nkops.nk_ptov(vlink->c_info);
  	}
	vrpc->peer.id    = vlink->s_id;
	vrpc->peer.state = &vlink->s_state;
        if (vlink->s_info) {
            vrpc->peer.info = nkops.nk_ptov(vlink->s_info);
  	}
    }

    if (!_vrpc_pmem_alloc(vrpc)) {
  	ETRACE("VLINK %d (%d -> %d) persistent memory allocation failed\n",
	        vlink->link, vlink->c_id, vlink->s_id);
	return -EINVAL;
    }    

    if (!_vrpc_pxirq_alloc(vrpc)) {
  	ETRACE("VLINK %d (%d -> %d) persistent IRQ allocation failed\n",
	        vlink->link, vlink->c_id, vlink->s_id);
	return -EINVAL;
    }    

    vrpc->xid = nkops.nk_xirq_attach(vrpc->my.xirq, _vrpc_xirq_handler, vrpc);
    if (!vrpc->xid) {
  	ETRACE("VLINK %d (%d -> %d) unable to attach XIRQ %d\n",
	        vlink->link, vlink->c_id, vlink->s_id, vrpc->my.xirq);
	return -ENOMEM;
    }

    DTRACE("VLINK %d (%d -> %d) found (info: %s)\n",
	   vlink->link, vlink->c_id, vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    vrpc->next = _vrpcs;
    _vrpcs     = vrpc;

    return 0;
}

    static int __init
_vrpc_init (void)
{
    int      res;
    NkOsId   myid  = nkops.nk_id_get();
    NkPhAddr plink = 0;

    mutex_init(&_mutex);

    while ((plink = nkops.nk_vlink_lookup("vrpc", plink))) {
	NkDevVlink* vlink = nkops.nk_ptov(plink);
	if ((vlink->s_id == myid) || (vlink->c_id == myid)) {
	    if ((res = _vrpc_link_init(vlink, plink))) {
	  	ETRACE("VLINK %d (%d -> %d) initialization failed\n",
		        vlink->link, vlink->c_id, vlink->s_id);
		return res;
	    }
	}
    }

    _scid = nkops.nk_xirq_attach(NK_XIRQ_SYSCONF, _vrpc_sysconf_handler, 0);
    if (!_scid) {
  	ETRACE("unable to attach a sysconf handler\n");
	return -ENOMEM;
    }

    TRACE("module loaded\n");	

    return 0;
}

    int
_vrpc_match (vrpc_t* vrpc, const char* name)
{
    if (name) {
	if (!vrpc->my.info) {
	    return 0;
	}
	return !strncmp(vrpc->my.info, name, strlen(name));
    }
    return (!vrpc->my.info || !vrpc->my.info[0] || (vrpc->my.info[0] == ','));
}

    vrpc_t*
_vrpc_lookup (const char* name, int server, vrpc_t* last)
{
    vrpc_t* vrpc;

    mutex_lock(&_mutex);

    vrpc = (last ? last->next : _vrpcs);
    while (vrpc) {
	if (!vrpc->used && (_vrpc_is_server(vrpc) == server) &&
	    _vrpc_match(vrpc, name)) {
	    vrpc->used = 1;
	    break;
	}
	vrpc = vrpc->next;
    }

    mutex_unlock(&_mutex);

    return vrpc;
}

    vrpc_t*
vrpc_server_lookup (const char* name, vrpc_t* last)
{
    return _vrpc_lookup(name, 1, last);
}

    vrpc_t*
vrpc_client_lookup (const char* name, vrpc_t* last)
{
    return _vrpc_lookup(name, 0, last);
}

    void
vrpc_release (vrpc_t* vrpc)
{
    mutex_lock(&_mutex);
    vrpc->used = 0;
    mutex_unlock(&_mutex);
}

    NkOsId
vrpc_peer_id (vrpc_t* vrpc)
{
    return vrpc->peer.id;
}

    void*
vrpc_data (vrpc_t* vrpc)
{
    return vrpc->pmem->data;
}

    vrpc_size_t
vrpc_maxsize (vrpc_t* vrpc)
{
    return vrpc->msize;
}

    static int
_vrpc_server_thread_wakeup (vrpc_t* vrpc)
{
    return _aborted || !vrpc->open ||
	    (_vrpc_ready(vrpc) && (vrpc->pmem->req != vrpc->pmem->ack));
}

    static int
_vrpc_server_thread (void* data)
{
    vrpc_t* vrpc = data;

    DTRACE("VLINK %d (%d -> %d) [%s] server thread started\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    for (;;) {
	if (wait_event_interruptible(vrpc->wait, _vrpc_server_thread_wakeup(vrpc))) {
	    break;
	}

	if (_aborted || !vrpc->open) {
	    break;
	}
	_vrpc_direct_call(vrpc);
    }

    DTRACE("VLINK %d (%d -> %d) [%s] server thread stopped\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    while (!kthread_should_stop()) {
	set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(1);
    }

    return 0;
}

    static int
_vrpc_wait_for_peer_wakeup (vrpc_t* vrpc)
{
    int wakeup = 0;

    if (_aborted) {
	wakeup = 1;
    }

    if (_vrpc_handshake(vrpc)) {
	wakeup = 1;
    }

    return wakeup;
}

    static int
_vrpc_wait_for_peer (vrpc_t* vrpc)
{
    while (!_aborted && !_vrpc_handshake(vrpc)) {
	wait_event_interruptible(vrpc->wait, _vrpc_wait_for_peer_wakeup(vrpc));
    }
    return !_aborted;
}

    static int
_vrpc_client_thread (void* data)
{
    vrpc_t* vrpc = data;

    DTRACE("VLINK %d (%d -> %d) [%s] client thread started\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    if (_vrpc_wait_for_peer(vrpc)) {
	vrpc->ready(vrpc->cookie);
    }

    DTRACE("VLINK %d (%d -> %d) [%s] client thread stopped\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    return 0;
}

    static void
_vrpc_thread_stop (vrpc_t* vrpc)
{
    if (vrpc->thread) {
        kthread_stop(vrpc->thread);
    }
}

    int
vrpc_server_open (vrpc_t* vrpc, vrpc_call_t call, void* cookie, int direct)
{
    vrpc->call    = call;
    vrpc->cookie  = cookie;

    init_waitqueue_head(&vrpc->wait);

    if (direct) {
	vrpc->wrapper = _vrpc_direct_call;
	vrpc->thread  = 0;
    } else {
        vrpc->wrapper = _vrpc_indirect_call;
        vrpc->thread  = kthread_run(_vrpc_server_thread, vrpc, "vrpc-server");
        if (!vrpc->thread) {
	    return -EFAULT;
        }
    }

    vrpc->open = 1;

    _vrpc_handshake(vrpc);

    DTRACE("VLINK %d (%d -> %d) [%s] server open\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    return 0;
}

    int
vrpc_client_open (vrpc_t* vrpc, vrpc_ready_t ready, void* cookie)
{
    vrpc->ready   = ready;
    vrpc->cookie  = cookie;
    vrpc->wrapper = _vrpc_wakeup;

    init_waitqueue_head(&vrpc->wait);

    if (ready) {
	if (!kthread_run(_vrpc_client_thread, vrpc, "vrpc-client")) {
	    return -EFAULT;
        }
    } else {
	if (!_vrpc_wait_for_peer(vrpc)) {
	    return -EFAULT;
	}
    }

    vrpc->open = 1;

    DTRACE("VLINK %d (%d -> %d) [%s] client open\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    return 0;
}

    static int
_vrpc_call_wakeup (vrpc_t* vrpc)
{
    vrpc_pmem_t* pmem   = vrpc->pmem;
    int          wakeup = 0;

    if (!_vrpc_ready(vrpc)) {
	wakeup = 1;
    }

    if (pmem->req == pmem->ack) {
	wakeup = 1;
    }

    return wakeup;
}

    int
vrpc_call (vrpc_t* vrpc, vrpc_size_t* size)
{
    vrpc_pmem_t* pmem = vrpc->pmem;

    DTRACE("VLINK %d (%d -> %d) [%s] call <- 0x%x (%d)\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""), *size, *size);

    pmem->size = *size;
    pmem->req++;

    _vrpc_xirq_post(vrpc);

    if (wait_event_interruptible(vrpc->wait, _vrpc_call_wakeup(vrpc))) {
	return -EINTR;
    }

    if (!_vrpc_ready(vrpc)) {
	*size = 0;
	DTRACE("VLINK %d (%d -> %d) [%s] call -> ERROR %d\n",
	       vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	       (vrpc->my.info ? vrpc->my.info : ""),
	       (_aborted ? -EFAULT : -EAGAIN));
	return (_aborted ? -EFAULT : -EAGAIN);
    }

    *size = pmem->size;

    DTRACE("VLINK %d (%d -> %d) [%s] call -> 0x%x (%d)\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""), *size, *size);

    return 0;
}

    void
vrpc_close (vrpc_t* vrpc)
{
    DTRACE("VLINK %d (%d -> %d) [%s] close\n",
	   vrpc->vlink->link, vrpc->vlink->c_id, vrpc->vlink->s_id,
	   (vrpc->my.info ? vrpc->my.info : ""));

    vrpc->open    = 0;
    vrpc->wrapper = _vrpc_null_call;
    _vrpc_wakeup(vrpc);
    _vrpc_thread_stop(vrpc);
    *vrpc->my.state = NK_DEV_VLINK_OFF;
    _vrpc_sysconf_post(vrpc);
}

    static void __exit
_vrpc_link_exit (vrpc_t* vrpc)
{
    if (vrpc->xid) {
	nkops.nk_xirq_detach(vrpc->xid);
    }
    vrpc_close(vrpc);
    kfree(vrpc);
}

    static void __exit
_vrpc_exit (void)
{
    _aborted = 1;
    if (_scid) {
	nkops.nk_xirq_detach(_scid);
    }
    while (_vrpcs) {
	vrpc_t* vrpc = _vrpcs;
	_vrpcs = vrpc->next;
	_vrpc_link_exit(vrpc);
    }
    TRACE("module unloaded\n");	
}

EXPORT_SYMBOL(vrpc_call);
EXPORT_SYMBOL(vrpc_client_lookup);
EXPORT_SYMBOL(vrpc_client_open);
EXPORT_SYMBOL(vrpc_close);
EXPORT_SYMBOL(vrpc_data);
EXPORT_SYMBOL(vrpc_maxsize);
EXPORT_SYMBOL(vrpc_peer_id);
EXPORT_SYMBOL(vrpc_release);
EXPORT_SYMBOL(vrpc_server_lookup);
EXPORT_SYMBOL(vrpc_server_open);

MODULE_DESCRIPTION("Virtual RPC driver on top of VLX");
MODULE_AUTHOR("Vladimir Grouzdev <vladimir.grouzdev@vlx.com> - VirtualLogix");
MODULE_LICENSE("Proprietary");

module_init(_vrpc_init);
module_exit(_vrpc_exit);

