/*****************************************************************************
 *                                                                           *
 *  Component: VLX VLink Wrapper Library.                                    *
 *                                                                           *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.              *
 *                                                                           *
 *  #ident  "%Z%%M% %I%     %E% Red Bend Software"                           *
 *                                                                           *
 *  Contributor(s):                                                          *
 *    Sebastien Laborie <sebastien.laborie@redbend.com>                      *
 *                                                                           *
 *****************************************************************************/

#ifndef _VLX_VLINK_LIB_H
#define _VLX_VLINK_LIB_H

#include <stddef.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/bug.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <asm/atomic.h>
#include <nk/nkern.h>
#include "vlink-lib-arch.h"

#define VLINK_PRINTK(v, ll, h, f, a...)			\
    do {						\
	printk(ll "%s [link#%d OS#%d %s OS#%d]" h f,	\
	       (v)->nk_vlink->name,			\
	       (v)->nk_vlink->link,			\
	       (v)->id,					\
	       (v)->server ? "<-" : "->",		\
	       (v)->id_peer,				\
	       ##a);					\
    } while (0)

#define VLINK_TRACE(v, m...)				\
    VLINK_PRINTK(v, KERN_INFO, ": ", m)

#define VLINK_ERROR(v, m...)				\
    VLINK_PRINTK(v, KERN_ERR, " ERROR: ", m)

#define VLINK_WARN(v, m...)				\
    VLINK_PRINTK(v, KERN_WARNING, " WARNING: ", m)

#define VLINK_ASSERT(e)					\
    do {						\
	if (!(e)) {					\
	    printk(KERN_CRIT "assert failed: line %d "	\
		   "file %s\n", __LINE__, __FILE__);	\
	    BUG();					\
	}						\
    } while (0)

#ifdef VLINK_DEBUG
#define VLINK_DTRACE(v, m...)				\
    VLINK_PRINTK(v, KERN_DEBUG, ": ", m)
#define VLINK_DASSERT(e)				\
    VLINK_ASSERT(e)
#else
#define VLINK_DTRACE(v, m...)
#define VLINK_DASSERT(e)
#endif

#define VLINK_DTRACE_ENTER(v, m...)			\
    VLINK_DTRACE(v, ">>> " m)

#define VLINK_DTRACE_LEAVE(v, m...)			\
    VLINK_DTRACE(v, "<<< " m)

#define VLINK_FUNC_ENTER(v)				\
    VLINK_DTRACE_ENTER(v, "%s\n", __FUNCTION__)

#define VLINK_FUNC_LEAVE(v)				\
    VLINK_DTRACE_LEAVE(v, "%s\n", __FUNCTION__)

#define VLINK_FUNC_MSG(v, f, a...)			\
    VLINK_DTRACE(v, "--- %s: " f, __FUNCTION__, ##a)

struct Vlink;
struct VlinkSession;
struct VlinkDrv;

#define VLINK_DRV_CLEAN			0
#define VLINK_DRV_PROBED		1
#define VLINK_DRV_STARTED		2
#define VLINK_DRV_STOPPED		3

#define VLINK_DRV_TYPE_CLIENT		(1 << 0)
#define VLINK_DRV_TYPE_SERVER		(1 << 1)
#define VLINK_DRV_TYPE_SYMMETRIC	(VLINK_DRV_TYPE_CLIENT | \
					 VLINK_DRV_TYPE_SERVER)

#define VLINK_DRV_IS_SYMMETRIC(d)			\
    (((d)->flags & VLINK_DRV_TYPE_SYMMETRIC) == VLINK_DRV_TYPE_SYMMETRIC)

typedef int  (*VlinkDrvInit)    (struct VlinkDrv*);
typedef void (*VlinkDrvCleanup) (struct VlinkDrv*);
typedef int  (*VlinkInit)       (struct Vlink*);

typedef struct VlinkDrv {
    /*
     * Public driver information - provided by the driver.
     */
    const char*      name;
    VlinkDrvInit     init;
    VlinkDrvCleanup  cleanup;
    VlinkInit        vlink_init;
    unsigned int     flags;
    /*
     * Public driver information - provided by the vlink library.
     */
    unsigned int     nr_units;
    /*
     * Private driver information - provided by the driver.
     */
    void*            private;
    /*
     * Internal vlink library data.
     */
    unsigned int     nr_clients;
    unsigned int     nr_servers;
    unsigned int     state;
    NkXIrqId         sysconf_id;
    struct list_head vlinks;
} VlinkDrv;

#define VLINK_OP_RESET			0
#define VLINK_OP_START			1
#define VLINK_OP_ABORT			2
#define VLINK_OP_STOP			3
#define VLINK_OP_CLEANUP		4
#define VLINK_OP_NR			5

typedef int (*VlinkOp)        (struct Vlink* vlink, void* cookie);
typedef int (*VlinkSessionOp) (struct VlinkSession* session);

typedef struct VlinkOpDesc {
    unsigned int     idx;
    VlinkOp          op;
} VlinkOpDesc;

typedef struct VlinkOpWrap {
    VlinkOp          op;
    void*            cookie;
    struct list_head link;
} VlinkOpWrap;

#define VLINK_CLEAN			0	/* VLINK_OFF   */
#define VLINK_STOPPED			1	/* VLINK_OFF   */
#define VLINK_RESET			2	/* VLINK_RESET */
#define VLINK_STARTED			3	/* VLINK_ON    */
#define VLINK_UP			4	/* VLINK_ON    */
#define VLINK_ABORTED			5	/* VLINK_ON    */

#define VLINK_HIT_STATE(v)				\
    ((v)->state == (v)->state_target)
#define VLINK_STARTUP(v)				\
    ((v)->state_target == VLINK_UP)
#define VLINK_TERMINATE(v)				\
    ((v)->state_target == VLINK_STOPPED)
#define VLINK_IS_USABLE(v)				\
    ((v)->state != VLINK_CLEAN)
#define VLINK_IS_UNUSABLE(v)				\
    ((v)->state == VLINK_CLEAN)
#define VLINK_IS_UP(v)					\
    ((v)->state == VLINK_UP)
#define VLINK_IS_STARTED(v)				\
    ((v)->state == VLINK_STARTED || VLINK_IS_UP(v))
#define VLINK_IS_TERMINATED(v)				\
    (VLINK_TERMINATE(v) && ((v)->state == VLINK_STOPPED))

#define VLINK_ADMIN_RECONF		(1 << 0)
#define VLINK_ADMIN_EXIT		(1 << 1)

typedef struct Vlink {
    /*
     * Public device information - provided by the vlink library.
     */
    VlinkDrv*              drv;
    NkDevVlink*            nk_vlink;
    struct Vlink*          sym_vlink;
    unsigned int           server;
    unsigned int           unit;
    NkOsId                 id;
    NkOsId                 id_peer;
    /*
     * Private device information - provided by the driver.
     */
    void*                  private;
    /*
     * Internal vlink library data.
     */
    volatile unsigned int* nk_state;
    volatile unsigned int* nk_state_peer;
    volatile unsigned int  state;
    volatile unsigned int  state_target;
    spinlock_t             state_lock;
    atomic_t               users;
    void*                  admin_thread;
    volatile unsigned int  admin_event;
    wait_queue_head_t      admin_event_wait;
    wait_queue_head_t      admin_comp_wait;
    struct semaphore       sessions_lock;
    struct semaphore       sessions_start_lock;
    wait_queue_head_t      sessions_end_wait;
    atomic_t               sessions_count;
    struct list_head       sessions;
    struct list_head       ops[VLINK_OP_NR];
    struct list_head       link;
} Vlink;

#define VLINK_SESSION_NEW		0
#define VLINK_SESSION_ALIVE		1
#define VLINK_SESSION_ABORTED		2

typedef struct VlinkSession {
    Vlink*                 vlink;
    volatile unsigned int  state;
    atomic_t               entered;
    atomic_t               refcount;
    void*                  private;
    VlinkSessionOp         op_abort;
    struct list_head       link;
} VlinkSession;

#define VLINK_SESSION_IS_ALIVE(s)	((s)->state == VLINK_SESSION_ALIVE)
#define VLINK_SESSION_IS_ABORTED(s)	((s)->state == VLINK_SESSION_ABORTED)

extern int  vlink_session_create  (Vlink*         vlink,
				   void*          private,
				   VlinkSessionOp op_abort,
				   VlinkSession** psession);
extern void vlink_session_release (VlinkSession*  session);

extern int  vlink_op_register  (Vlink*       vlink,
				unsigned int idx,
				VlinkOp      op,
				void*        cookie);
extern int  vlink_ops_register (Vlink*       vlink,
				VlinkOpDesc* ops,
				void*        cookie);

extern int  vlink_drv_probe    (VlinkDrv* drv);
extern int  vlink_drv_startup  (VlinkDrv* drv);
extern int  vlink_drv_shutdown (VlinkDrv* drv);
extern void vlink_drv_cleanup  (VlinkDrv* drv);

extern void vlink_dump (Vlink* vlink);

    /*
     *
     */
    static inline void
vlink_session_leave (VlinkSession* session)
{
#ifdef VLINK_DEBUG
    VLINK_ASSERT(atomic_read(&session->entered) > 0);
    vlink_atomic_dec(&session->entered);
#endif
    vlink_atomic_dec(&session->vlink->users);    
}

    /*
     *
     */
    static inline void
vlink_session_enter (VlinkSession* session)
{
#ifdef VLINK_DEBUG
    vlink_atomic_inc(&session->entered);
#endif
    vlink_atomic_inc(&session->vlink->users);
}

    /*
     *
     */
    static inline int
vlink_session_enter_and_test_alive (VlinkSession* session)
{
    vlink_session_enter(session);
    return (session->state - VLINK_SESSION_ALIVE);
}

    /*
     *
     */
    static inline void
vlink_session_destroy (VlinkSession* session)
{
    int last = atomic_dec_and_test(&session->refcount);
    VLINK_ASSERT(last);
#ifdef VLINK_DEBUG
    VLINK_ASSERT(atomic_read(&session->entered) == 0);
#endif
    vlink_session_release(session);
}

    /*
     *
     */
    static inline void
vlink_session_put (VlinkSession* session)
{
    if (atomic_dec_and_test(&session->refcount)) {
#ifdef VLINK_DEBUG
	VLINK_ASSERT(atomic_read(&session->entered) == 0);
#endif
	vlink_session_release(session);
    }
}

    /*
     *
     */
    static inline void
vlink_session_get (VlinkSession* session)
{
    atomic_inc(&session->refcount);
}

#endif /* _VLX_VLINK_LIB_H */
