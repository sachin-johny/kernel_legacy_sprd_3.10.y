/*****************************************************************************
 *                                                                           *
 *  Component: VLX Virtual Remote Procedure Queue (VRPQ).                    *
 *             VRPQ backend kernel driver implementation.                    *
 *                                                                           *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.              *
 *                                                                           *
 *  #ident  "%Z%%M% %I%     %E% Red Bend Software"                           *
 *                                                                           *
 *  Contributor(s):                                                          *
 *    Sebastien Laborie <sebastien.laborie@redbend.com>                      *
 *                                                                           *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/file.h>
#include "vrpq.h"

#define VLX_SERVICES_THREADS
#include <linux/version.h>
#include <linux/kthread.h>
#include "vlx-services.c"

MODULE_DESCRIPTION("VRPQ Back-End Driver");
MODULE_AUTHOR("Sebastien Laborie <sebastien.laborie@redbend.com>");
MODULE_LICENSE("GPL");

static noinline VrpqReqMsg* vrpq_adm_req_pop   (VrpqSrvSession*    session,
						VrpqSrvSessionAdm* adm);
static noinline void*       vrpq_adm_param_get (VrpqSrvSession*    session,
						VrpqSize           param_off,
						VrpqSize           param_sz);

#define VRPQ_ADM_PARAM_IN_GET(session, req, type)			\
    ((type* ) vrpq_adm_param_get(session,				\
				 (req)->procReq.inOffset,		\
				 sizeof(type)))

#define VRPQ_ADM_PARAM_OUT_GET(session, req, type)			\
    ((type* ) vrpq_adm_param_get(session,				\
				 (req)->procReq.outOffset,		\
				 sizeof(type)))

    /*
     *
     */
    static noinline unsigned int
vrpq_bitmap_alloc (VrpqBitmapAlloc* alloc, unsigned int limit)
{
    nku32_f      l1;
    unsigned int l1_idx;
    nku32_f      l2;
    unsigned int l2_idx;
    unsigned int idx;

    VLINK_DASSERT(limit <= (32 *32));

    spin_lock(&alloc->lock);

    l1 = alloc->l1;

    if (l1 == ~0UL) {
	idx = limit;
    } else {

	l1_idx = ffz(l1);
	VLINK_DASSERT(l1_idx <= 31);

	l2     = alloc->l2[l1_idx];
	l2_idx = ffz(l2);
	VLINK_DASSERT(l2 != ~0UL);
	VLINK_DASSERT(l2_idx <= 31);

	idx = l1_idx * 32 + l2_idx;

	if (idx < limit) {
	    l2 |= (1 << l2_idx);
	    if (l2 == ~0UL) {
		l1 |= (1 << l1_idx);
	    }
	    alloc->l2[l1_idx] = l2;
	    alloc->l1         = l1;
	} else {
	    idx = limit;
	}
    }

    spin_unlock(&alloc->lock);

    return idx;
}

    /*
     *
     */
     static noinline void
vrpq_bitmap_free (VrpqBitmapAlloc* alloc, unsigned int idx)
{
    unsigned int l1_idx;
    unsigned int l2_idx;

    VLINK_DASSERT(idx <= (32 *32));

    spin_lock(&alloc->lock);

    l1_idx = idx / 32;
    l2_idx = idx & 31;

    VLINK_DASSERT(alloc->l2[l1_idx] & (1 << l2_idx));
    if (alloc->l2[l1_idx] == ~0UL) {
	VLINK_DASSERT(alloc->l1 & (1 << l1_idx));
    } else {
	VLINK_DASSERT((alloc->l1 & (1 << l1_idx)) == 0);
    }

    alloc->l2[l1_idx] &= ~(1 << l2_idx);
    alloc->l1         &= ~(1 << l1_idx);

    spin_unlock(&alloc->lock);
}

    /*
     *
     */
    static noinline int
vrpq_crit_diag (int diag1, int diag2)
{
    if ((diag1 != 0) && (diag1 != -EAGAIN) && (diag1 != -ECHANBROKEN)) {
	return diag1;
    }
    if ((diag2 != 0) && (diag2 != -EAGAIN) && (diag2 != -ECHANBROKEN)) {
	return diag2;
    }
    if ((diag1 == -ECHANBROKEN) || (diag2 == -ECHANBROKEN)) {
	return -ECHANBROKEN;
    }
    if ((diag1 == -EAGAIN) || (diag2 == -EAGAIN)) {
	return -EAGAIN;
    }
    return diag1;
}

    /*
     *
     */
    static inline void
vrpq_peer_rsp_notify (VrpqSrvPeer* peer)
{
    nkops.nk_xirq_trigger(peer->xirq_rsp, peer->osid);
}

    /*
     *
     */
    static inline void
vrpq_peer_full_notify (VrpqSrvPeer* peer)
{
    nkops.nk_xirq_trigger(peer->xirq_full, peer->osid);
}

    /*
     *
     */
    static noinline int
vrpq_rsp_send (VrpqSrvSession* session, VrpqReqMsg* req, int ret_val)
{
    VrpqSrvDev*   dev   = session->dev;
    VrpqRspAlloc* alloc = &dev->rsp_alloc;
    VrpqRspRing*  ring  = &alloc->ring;
    VrpqRingIdx   head_idx;
    VrpqRingIdx   tail_idx;
    VrpqRingIdx   mod_mask;
    VrpqRspMsg*   rsp;

    spin_lock(&alloc->lock);

    head_idx = ring->headIdx;
    tail_idx = ring->global->tailIdx;
    mod_mask = ring->modMask;

    if (VRPQ_RSP_RING_IS_FULL(head_idx, tail_idx, mod_mask)) {
	return -EAGAIN;
    }

    rsp            = &ring->resps[head_idx & mod_mask];
    rsp->reqIdx    = req->msgId;
    rsp->retVal    = ret_val;
    req->flags.rsp = 1;

    ring->global->headIdx = ring->headIdx = ++head_idx;

    spin_unlock(&alloc->lock);

    vrpq_peer_rsp_notify(&dev->peer);

    return 0;
}

    /*
     *
     */
    static inline void
vrpq_reqs_wakeup (VrpqSrvChan* chan)
{
    wake_up_interruptible(&chan->chan_req_ring.wait);
}

    /*
     *
     */
    static noinline void
vrpq_reqs_do_dispatch (VrpqSrvDev* dev)
{
    VrpqReqRing*          req_ring;
    volatile VrpqRingIdx* gl_head_idx;
    VrpqRingIdx           tail_idx;
    VrpqRingIdx           head_idx;
    VrpqRingIdx           req_mod_mask;
    VrpqReqMsg*           reqs;
    VrpqReqMsg*           req;
    VrpqSrvChan**         chans;
    VrpqSrvChan*          chan;
    VrpqChanId            chan_id;
    VrpqChanReqRing*      chan_req_ring;
    VrpqRingIdx           chan_head_idx;

    VRPQ_PROF_INC(dev, notifs);

    req_ring     = &dev->req_ring;
    gl_head_idx  = &req_ring->global->headIdx;
    tail_idx     = req_ring->tailIdx;
    req_mod_mask = req_ring->modMask;
    reqs         = req_ring->reqs;
    chans        = dev->chans;
    req          = NULL;

    do {

	head_idx = *gl_head_idx;
	if (VRPQ_REQ_RING_IS_OVERFLOW(head_idx, tail_idx, req_mod_mask)) {
	    goto out;
	}

	do {
	    req = &reqs[tail_idx & req_mod_mask];

	    chan_id = req->chanId;
	    if (chan_id >= VRPQ_CHAN_MAX) {
		chan_id = 0;
	    }
	    chan = chans[chan_id];
	    if (chan->close.all) {
		chan = chans[0];
	    }

	    chan_req_ring = &chan->chan_req_ring;
	    chan_head_idx = chan_req_ring->head_idx;
	    if (VRPQ_CHAN_REQ_RING_IS_FULL(chan_head_idx,
					   chan_req_ring->tail_idx)) {
		goto out;
	    }
	    chan_req_ring->reqs[chan_head_idx & VRPQ_CHAN_REQS_MASK] = req;
	    chan_req_ring->head_idx = ++chan_head_idx;

	    /* XXX */
	    if (chan_req_ring->wait_flag) {
		chan_req_ring->wait_flag = 0;
		vrpq_reqs_wakeup(chan);
	    }

	    tail_idx++;

	} while (!VRPQ_CHAN_REQ_RING_IS_EMPTY(head_idx, tail_idx));

    } while (head_idx != *gl_head_idx);

out:
    req_ring->tailIdx = tail_idx;
}

    /*
     *
     */
    static inline void
vrpq_reqs_dispatch_locked (VrpqSrvDev* dev)
{
    VrpqReqRing* req_ring;

    req_ring = &dev->req_ring;
    if (!VRPQ_REQ_RING_IS_EMPTY(req_ring->global->headIdx,
				req_ring->tailIdx)) {
	vrpq_reqs_do_dispatch(dev);
    }
}

    /*
     *
     */
    static inline void
vrpq_reqs_dispatch_lock (VrpqSrvDev* dev)
{
    while (xchg(&dev->dispatch_excl, 1) != 0);
}

    /*
     *
     */
    static inline void
vrpq_reqs_dispatch_unlock (VrpqSrvDev* dev)
{
    dev->dispatch_excl = 0;
}

    /*
     *
     */
    static inline void
vrpq_reqs_dispatch (VrpqSrvDev* dev)
{
    if (xchg(&dev->dispatch_excl, 1) == 0) {
	vrpq_reqs_dispatch_locked(dev);
	dev->dispatch_excl = 0;
    }
}

    /*
     *
     */
    static void
vrpq_reqs_handler (void* cookie, NkXIrq xirq)
{
    VrpqSrvDev* dev = (VrpqSrvDev*) cookie;

    vrpq_reqs_dispatch(dev);
}

    /*
     *
     */
    static noinline int
vrpq_reqs_wait (VrpqSrvChan* chan, VrpqRingIdx old_head_idx)
{
    volatile VrpqRingIdx* head_idx = &chan->chan_req_ring.head_idx;
    int                   diag;

    chan->chan_req_ring.wait_flag = 1;

    diag = wait_event_interruptible(chan->chan_req_ring.wait,
				    (VLINK_SESSION_IS_ABORTED(chan->vls) ||
				     chan->close.clt ||
				     *head_idx != old_head_idx));
    if (diag) {
	return diag;
    }

    if (VLINK_SESSION_IS_ABORTED(chan->vls)) {
	return -ECHANBROKEN;
    }

    if (chan->close.clt && (*head_idx == old_head_idx)) {
	return -ECHANBROKEN;
    }

    return 0;
}

    /*
     *
     */
    static inline void
vrpq_peer_full_awake (VrpqSrvDev* dev)
{
    volatile nku32_f* full_flag = &dev->req_ring.global->fullFlag;

    if (*full_flag) {
	*full_flag = 0;
	vrpq_peer_full_notify(&dev->peer);
    }
}

    /*
     *
     */
    static int
vrpq_receive (struct VrpqSrvFile*  vrpq_file,
	      VrpqProcReq __user** ureqs,
	      int                  count)
{
    VrpqSrvChan*     chan = &vrpq_file->chan;
    VrpqChanReqRing* ring;
    VrpqRingIdx      head_idx;
    VrpqRingIdx      tail_idx;
    VrpqRingIdx      prev_tail_idx;
    VrpqReqMsg*      req;
    VrpqProcReq*     upreq;
    unsigned long    k2u_offset;
    int              avail;
    int              diag;

    if (vrpq_file->type != VRPQ_FILE_CHAN) {
	return -EINVAL;
    }

    if (vlink_session_enter_and_test_alive(chan->vls)) {
	diag = -ECHANBROKEN;
	goto out_session_leave;
    }

    if (xchg(&chan->recv_excl, 1) != 0) {
	diag = -EALREADY;
	goto out_session_leave;
    }

    vrpq_reqs_dispatch(chan->dev);

    ring          = &chan->chan_req_ring;
    tail_idx      = ring->tail_idx;
    prev_tail_idx = ring->prev_tail_idx;

    if (prev_tail_idx != tail_idx) {

	for (;;) {

	    req = ring->reqs[prev_tail_idx & VRPQ_CHAN_REQS_MASK];

	    if (++prev_tail_idx == tail_idx) {
		break;
	    }

	    req->flags.inuse = 0;
	}

	if (!req->flags.call) {
	    req->flags.inuse = 0;
	} else {
	    diag = vrpq_rsp_send(chan->session, req, chan->call_reason);
	    if (diag) {
		ring->prev_tail_idx = prev_tail_idx - 1;
		goto out_excl_leave;
	    }
	    chan->call_reason = 0;
	}

	ring->prev_tail_idx = prev_tail_idx;

	vrpq_peer_full_awake(chan->dev);
    }

    if (!count) {
	diag = 0;
	goto out_excl_leave;
    }

    head_idx = ring->head_idx;

    if (VRPQ_CHAN_REQ_RING_IS_EMPTY(head_idx, tail_idx)) {
	diag = vrpq_reqs_wait(chan, head_idx);
	if (diag) {
	    goto out_excl_leave;
	}
    }

    k2u_offset = chan->session->pmem_map.k2u_offset;
    avail      = 0;

    do {

	head_idx = ring->head_idx;

	do {

	    req   = ring->reqs[tail_idx & VRPQ_CHAN_REQS_MASK];
	    upreq = VRPQ_PMEM_K2U(&req->procReq, k2u_offset);

	    if (put_user((unsigned long) upreq,
			 (unsigned long __user *) &ureqs[avail])) {
		diag = -EFAULT;
		goto out_excl_leave;
	    }

	    tail_idx++;
	    avail++;

	    if (avail == count || req->flags.call) {
		goto out;
	    }

	} while (!VRPQ_CHAN_REQ_RING_IS_EMPTY(head_idx, tail_idx));

    } while (head_idx != ring->head_idx);

out:
    ring->tail_idx = tail_idx;
    diag = avail;

out_excl_leave:
    chan->recv_excl = 0;

out_session_leave:
    vlink_session_leave(chan->vls);
    return diag;
}

    /*
     *
     */
    static int
vrpq_call_set_reason (struct VrpqSrvFile* vrpq_file, int reason,
		      unsigned int sz)
{
    VrpqSrvChan* chan = &vrpq_file->chan;
    int          diag;

    if (vrpq_file->type != VRPQ_FILE_CHAN) {
	return -EINVAL;
    }

    if (vlink_session_enter_and_test_alive(chan->vls)) {
	diag = -ECHANBROKEN;
	goto out_session_leave;
    }

    if (xchg(&chan->recv_excl, 1) != 0) {
	diag = -EALREADY;
	goto out_session_leave;
    }

    chan->call_reason = reason;
    chan->recv_excl   = 0;
    diag              = 0;

out_session_leave:
    vlink_session_leave(chan->vls);
    return diag;
  
}

    /*
     *
     */
    static int
vrpq_srv_shm_map (struct VrpqSrvFile* vrpq_file, void __user** upaddr,
		  unsigned int sz)
{
    VrpqSrvSession* session = &vrpq_file->session;
    void*           ubase;

    if (vrpq_file->type != VRPQ_FILE_SESSION) {
	return -EINVAL;
    }

    ubase                        = session->pmem_map.kbase;
    session->pmem_map.ubase      = ubase;
    session->pmem_map.k2u_offset = ((nku8_f*)ubase -
				    (nku8_f*)session->pmem_map.kbase);

    if (put_user(ubase, upaddr)) {
	return -EFAULT;
    }
    
    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_client_id_get (struct VrpqSrvFile*  vrpq_file,
			VrpqClientId __user* upid,
			unsigned int         sz)
{
    VrpqSrvSession* session = &vrpq_file->session;
    VrpqClientId    id;

    if (vrpq_file->type != VRPQ_FILE_SESSION) {
	return -EINVAL;
    }

    id.vmId  = session->dev->peer.osid;
    id.appId = session->app_id;

    if (copy_to_user(upid, &id, sizeof(VrpqClientId))) {
	return -EFAULT;
    }

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_session_abort (VlinkSession* vls)
{
    VrpqSrvSession* session = vls->private;

    VLINK_DTRACE(vls->vlink, "vrpq_srv_session_abort called\n");

    wake_up_interruptible(&session->adm.wait);

    return 0;
}

    /*
     *
     */
    static inline void
vrpq_srv_session_id_free (VrpqSrvDev* dev, VrpqSessionId id)
{
    vrpq_bitmap_free(dev->sessions_alloc, id);
}

    /*
     *
     */
    static inline VrpqSessionId
vrpq_srv_session_id_alloc (VrpqSrvDev* dev)
{
    return (VrpqSessionId) vrpq_bitmap_alloc(dev->sessions_alloc,
					     VRPQ_SESSION_MAX);
}

    /*
     *
     */
    static noinline void
vrpq_srv_session_init (VrpqSrvSession* session, VrpqSessionId id,
		       VrpqSrvDev* dev)
{
    VrpqPmemMap* pmem_map = &session->pmem_map;

    memset(session, 0, sizeof(VrpqSrvSession));

    session->id         = id;
    session->dev        = dev;
    session->adm.req_nr = &session->dev->adm_req_nr;

    pmem_map->ubase      = NULL;
    pmem_map->kbase      = dev->gen_dev.pmem_layout.pmemVaddr;
    pmem_map->k2u_offset = 0;

    atomic_set(&session->refcount, 2);

    INIT_LIST_HEAD(&session->adm.reqs);
    spin_lock_init(&session->adm.lock);
    init_waitqueue_head(&session->adm.wait);
}

    /*
     *
     */
     static int
vrpq_srv_session_accept (VrpqSrvFile* vrpq_file, void* arg, unsigned int sz)
{
    Vlink*                vlink   = vrpq_file->dev->gen_dev.vlink;
    VrpqSrvDev*           dev     = vrpq_file->dev;
    VrpqSrvSession*       session = &vrpq_file->session;
    VrpqSessionId         session_id;
    AppId                 app_id;
    VrpqReqMsg*           req;
    VrpqSessionCreateIn*  in;
    VrpqSessionCreateOut* out;
    int                   ret_val;
    int                   diag;

    if (down_interruptible(&vrpq_file->lock)) {
	return -ERESTARTSYS;
    }

    if (vrpq_file->type != VRPQ_FILE_NONE) {
	up(&vrpq_file->lock);
	return -EINVAL;
    }

    session_id = 0;
    vrpq_srv_session_init(session, session_id, dev);

    do {

	do {

	    diag = vlink_session_create(vlink,
					session,
					vrpq_srv_session_abort,
					&session->vls);
	    if (diag) {
		break;
	    }

	    req = vrpq_adm_req_pop(session, &dev->sessions[0]->adm);
	    if (IS_ERR(req)) {
		diag = PTR_ERR(req);
		vlink_session_leave(session->vls);
		vlink_session_destroy(session->vls);
	    }

	} while (diag == -ECHANBROKEN);

	if (diag) {
	    break;
	}

	down(&dev->adm_lock);
	vrpq_reqs_dispatch_lock(dev);

	ret_val = 0;

	if (!session_id) {
	    session_id = vrpq_srv_session_id_alloc(dev);
	    if (session_id == VRPQ_SESSION_MAX) {
		session_id = 0;
		ret_val = -EAGAIN;
	    }
	}

	in  = VRPQ_ADM_PARAM_IN_GET(session,  req, VrpqSessionCreateIn);
	out = VRPQ_ADM_PARAM_OUT_GET(session, req, VrpqSessionCreateOut);

	app_id         = in->appId;
	out->sessionId = session_id;

	diag = vrpq_rsp_send(session, req, ret_val);
	diag = vrpq_crit_diag(diag, ret_val);

	if (VLINK_SESSION_IS_ABORTED(session->vls)) {
	    diag = vrpq_crit_diag(-ECHANBROKEN, diag);
	}

	if (!diag) {
	    session->id               = session_id;
	    session->app_id           = app_id;
	    dev->sessions[session_id] = session;
	}

	vrpq_reqs_dispatch_locked(dev);
	vrpq_reqs_dispatch_unlock(dev);
	up(&dev->adm_lock);

	vlink_session_leave(session->vls);

	if (diag) {
	    vlink_session_destroy(session->vls);	
	}

    } while ((diag == -EAGAIN) || (diag == -ECHANBROKEN));

    if (diag) {
	if (session_id) {
	    vrpq_srv_session_id_free(dev, session_id);
	}
    } else {
	vrpq_file->type = VRPQ_FILE_SESSION;

	VLINK_DTRACE(vlink,
		     "session #%d created on server side for client app #%d\n",
		     session->id, session->app_id);
    }

    up(&vrpq_file->lock);

    return diag;
}

    /*
     *
     */
     static noinline int
vrpq_srv_session_flush (VrpqSrvSession* session)
{
    VrpqSrvSessionAdm* adm  = &session->adm;
    VrpqReqMsg*        req;
    int                diag = 0;

    vlink_session_enter(session->vls);

    while ((req = vrpq_adm_req_pop(NULL, adm)) != NULL) {
	if (!VLINK_SESSION_IS_ALIVE(session->vls)) {
	    continue;
	}
	diag = vrpq_crit_diag(diag, vrpq_rsp_send(session, req, -ECHANBROKEN));
    }

    vlink_session_leave(session->vls);

    return diag;
}

    /*
     *
     */
     static noinline void
vrpq_srv_session_destroy (VrpqSrvSession* session)
{
    VrpqSrvDev*   dev = session->dev;
    VrpqSessionId id  = session->id;

    VLINK_ASSERT(id != 0);

    dev->sessions[id] = NULL;
    vrpq_srv_session_id_free(dev, id);

    vlink_session_destroy(session->vls);

    VLINK_DTRACE(session->dev->gen_dev.vlink,
		 "session #%d destroyed on server side\n",
		 session->id);

    kfree(VRPQ_SRV_SESSION_TO_FILE(session));
}

    /*
     *
     */
     static inline int
vrpq_srv_session_get (VrpqSrvSession* session)
{
    if (!atomic_inc_not_zero(&session->refcount)) {
	return 1;
    }
    return 0;
}

    /*
     *
     */
     static inline void
vrpq_srv_session_put (VrpqSrvSession* session)
{
    if (atomic_dec_and_test(&session->refcount)) {
	vrpq_srv_session_destroy(session);
    }
}

    /*
     *
     */
     static inline int
vrpq_srv_session_close (VrpqSrvSession* session)
{
    int diag;

    down(&session->dev->adm_lock);

    session->close.srv = 1;
    diag = vrpq_srv_session_flush(session);
    vrpq_srv_session_put(session);

    up(&session->dev->adm_lock);

    return diag;
}

    /*
     *
     */
    static inline void
vrpq_srv_chan_id_free (VrpqSrvDev* dev, VrpqChanId id)
{
    vrpq_bitmap_free(dev->chans_alloc, id);
}

    /*
     *
     */
    static inline VrpqChanId
vrpq_srv_chan_id_alloc (VrpqSrvDev* dev)
{
    return (VrpqChanId) vrpq_bitmap_alloc(dev->chans_alloc, VRPQ_CHAN_MAX);
}

    /*
     *
     */
    static noinline void
vrpq_srv_chan_init (VrpqSrvChan* chan, VrpqChanId id, VrpqSrvSession* session)
{
    memset(chan, 0, sizeof(VrpqSrvChan));

    chan->id      = id;
    chan->dev     = session->dev;
    chan->session = session;
    chan->vls     = session->vls;

    atomic_set(&chan->refcount, 2);

    init_waitqueue_head(&chan->chan_req_ring.wait);
}

    /*
     *
     */
     static int
vrpq_srv_chan_accept (VrpqSrvFile* vrpq_file, unsigned int session_fd,
		      unsigned int sz)
{
    int                fput_needed;
    struct file*       filp;
    unsigned int       minor;
    unsigned int       major;
    VrpqSrvDev*        dev;
    VrpqSrvFile*       session_vrpq_file;
    VrpqSrvSession*    session;
    VrpqReqMsg*        req;
    VrpqChanCreateOut* out;
    VrpqSrvChan*       chan    = &vrpq_file->chan;
    VrpqChanId         chan_id = 0;
    int                ret_val;
    int                diag;

    if (down_interruptible(&vrpq_file->lock)) {
	return -ERESTARTSYS;
    }

    if (vrpq_file->type != VRPQ_FILE_NONE) {
	diag = -EINVAL;
	goto out_unlock;
    }

    filp = fget_light(session_fd, &fput_needed);
    if (!filp) {
	diag = -EBADF;
	goto out_unlock;
    }

    major = imajor(filp->f_path.dentry->d_inode);
    minor = iminor(filp->f_path.dentry->d_inode);

    dev = (VrpqSrvDev*) vrpq_dev_find(major, minor);
    if (dev != vrpq_file->dev) {
	diag = -EINVAL;
	goto out_fput;
    }

    session_vrpq_file = (VrpqSrvFile*) filp->private_data;
    if (!session_vrpq_file) {
	diag = -EINVAL;
	goto out_fput;
    }

    if (session_vrpq_file->type != VRPQ_FILE_SESSION) {
	diag = -EINVAL;
	goto out_fput;
    }

    session = &session_vrpq_file->session;

    if (vrpq_srv_session_get(session)) {
	diag = -EINVAL;
	goto out_fput;
    }

    if (vlink_session_enter_and_test_alive(session->vls) ||
	session->close.clt) {
	diag = -ECHANBROKEN;
	goto out;
    }

    vrpq_srv_chan_init(chan, chan_id, session);

    do {

	req = vrpq_adm_req_pop(session, &session->adm);
	if (IS_ERR(req)) {
	    diag = PTR_ERR(req);
	    break;
	}

	down(&dev->adm_lock);
	vrpq_reqs_dispatch_lock(dev);

	ret_val = 0;

	if (!chan_id) {
	    chan_id = vrpq_srv_chan_id_alloc(dev);
	    if (chan_id == VRPQ_CHAN_MAX) {
		chan_id = 0;
		ret_val = -EAGAIN;
	    }
	}

	out = VRPQ_ADM_PARAM_OUT_GET(session, req, VrpqChanCreateOut);

	out->chanId = chan_id;

	diag = vrpq_rsp_send(session, req, ret_val);
	diag = vrpq_crit_diag(diag, ret_val);

	if (VLINK_SESSION_IS_ABORTED(session->vls) || session->close.clt) {
	    diag = vrpq_crit_diag(-ECHANBROKEN, diag);
	}

	if (!diag) {
	    chan->id            = chan_id;
	    dev->chans[chan_id] = chan;
	}

	vrpq_reqs_dispatch_locked(dev);
	vrpq_reqs_dispatch_unlock(dev);
	up(&dev->adm_lock);

    } while (diag == -EAGAIN);

out:
    vlink_session_leave(session->vls);

    if (diag) {
	if (chan_id) {
	    vrpq_srv_chan_id_free(dev, chan_id);
	}
	vrpq_srv_session_put(session);
    } else {
	vrpq_file->type = VRPQ_FILE_CHAN;

	VLINK_DTRACE(session->dev->gen_dev.vlink,
		     "channel #%d created on server side\n",
		     chan->id);
    }

out_fput:
    fput_light(filp, fput_needed);
out_unlock:
    up(&vrpq_file->lock);
    return diag;
}

    /*
     *
     */
     static noinline int
vrpq_srv_chan_flush (VrpqSrvChan* chan)
{
    VrpqSrvDev*      dev  = chan->dev;
    VrpqChanReqRing* ring = &chan->chan_req_ring;
    VrpqRingIdx      tail_idx;
    VrpqReqMsg*      req;
    int              ret_val;
    int              diag = 0;

    if (vlink_session_enter_and_test_alive(chan->vls)) {
	goto out_session_leave;
    }

    vrpq_reqs_dispatch_lock(dev);

    for (tail_idx = ring->prev_tail_idx;
	 tail_idx != ring->head_idx;
	 tail_idx++) {

	vrpq_reqs_dispatch_locked(dev);

	req = ring->reqs[tail_idx & VRPQ_CHAN_REQS_MASK];

	if (!req->flags.call) {
	    req->flags.inuse = 0;
	    continue;
	}

	ret_val = chan->call_reason;
	chan->call_reason = -ECHANBROKEN;

	diag = vrpq_crit_diag(diag,
			      vrpq_rsp_send(chan->session, req, ret_val));
    }

    ring->prev_tail_idx = ring->tail_idx = tail_idx;

    vrpq_peer_full_awake(chan->dev);

    vrpq_reqs_dispatch_unlock(dev);

out_session_leave:
    vlink_session_leave(chan->vls);
    return diag;
}

    /*
     *
     */
     static noinline void
vrpq_srv_chan_destroy (VrpqSrvChan* chan)
{
    VrpqSrvDev* dev = chan->dev;
    VrpqChanId  id  = chan->id;

    VLINK_ASSERT(id != 0);

    dev->chans[id] = dev->chans[0];
    vrpq_srv_chan_id_free(dev, id);

    vrpq_srv_session_put(chan->session);

    VLINK_DTRACE(chan->dev->gen_dev.vlink,
		 "channel #%d destroyed on server side\n",
		 chan->id);

    kfree(VRPQ_SRV_CHAN_TO_FILE(chan));
}

    /*
     *
     */
     static inline void
vrpq_srv_chan_put (VrpqSrvChan* chan)
{
    if (atomic_dec_and_test(&chan->refcount)) {
	vrpq_srv_chan_destroy(chan);
    }
}

    /*
     *
     */
     static inline int
vrpq_srv_chan_close (VrpqSrvChan* chan)
{
    int diag;

    down(&chan->dev->adm_lock);

    chan->close.srv = 1;
    diag = vrpq_srv_chan_flush(chan);
    vrpq_srv_chan_put(chan);

    up(&chan->dev->adm_lock);

    return diag;
}

    /*
     *
     */
    static inline void
vrpq_adm_req_wakeup (VrpqSrvSessionAdm* adm)
{
    wake_up_interruptible(&adm->wait);
}

    /*
     *
     */
    static noinline int
vrpq_adm_req_push (VrpqSrvSession* session, VrpqSrvSessionAdm* adm,
		   VrpqReqMsg* req)
{
    VrpqSrvAdmReq* adm_req;

    if (atomic_inc_return(adm->req_nr) > VRPQ_ADM_PENDING_REQS_MAX) {
	atomic_dec(adm->req_nr);
	return -EAGAIN;
    }

    adm_req = kzalloc(sizeof(VrpqSrvAdmReq), GFP_KERNEL);
    if (!adm_req) {
	return -ENOMEM;
    }

    vlink_session_get(session->vls);

    adm_req->req = req;
    adm_req->vls = session->vls;

    spin_lock(&adm->lock);

    list_add(&adm_req->link, &adm->reqs);

    spin_unlock(&adm->lock);

    vrpq_adm_req_wakeup(adm);

    return 0;
}

    /*
     *
     */
    static noinline VrpqReqMsg*
vrpq_adm_req_pop (VrpqSrvSession* session, VrpqSrvSessionAdm* adm)
{
    VrpqSrvAdmReq* adm_req;
    VrpqReqMsg*    req;
    int            diag;

    do {

	adm_req = NULL;
 	req     = NULL;
	diag    = 0;

	spin_lock(&adm->lock);

	if (!list_empty(&adm->reqs)) {
	    adm_req = list_first_entry(&adm->reqs, VrpqSrvAdmReq, link);
	    list_del(&adm_req->link);
	}

	spin_unlock(&adm->lock);

	if (adm_req) {
	    req = adm_req->req;
	    if (!VLINK_SESSION_IS_ALIVE(adm_req->vls)) {
		req = NULL;
	    }
	    vlink_session_put(adm_req->vls);
	    kfree(adm_req);
	    atomic_dec(adm->req_nr);
	    if (!req) {
		continue;
	    }
	    return req;
	}

	if (!session) {
	    return NULL;
	}

	diag = wait_event_interruptible(adm->wait,
			(VLINK_SESSION_IS_ABORTED(session->vls) ||
			 session->close.clt ||
			 !list_empty(&adm->reqs)));

	if (!diag && (VLINK_SESSION_IS_ABORTED(session->vls) ||
		      session->close.clt)) {
	    diag = -ECHANBROKEN;
	}

    } while (!diag);

    return ERR_PTR(diag);
}

    /*
     *
     */
    static inline void*
vrpq_adm_param_check (VrpqSrvSession* session, VrpqSize param_off,
		      VrpqSize param_sz, void* param_safe)
{
    VrpqPmemLayout* pmeml = &session->dev->gen_dev.pmem_layout;
    nku8_f*         start = (nku8_f*) pmeml->admParams;
    VrpqSize        size  = pmeml->admParamSize;
    nku8_f*         limit = start + size;
    nku8_f*         p     = ((nku8_f*) pmeml->pmemVaddr) + param_off;

    if (((p < start) || (p >= limit)) ||
	(p - start + param_sz > size) ||
	(param_off & (VRPQ_PARAM_ALIGN_SIZE - 1))) {
	return param_safe;
    }
    return p;
}

    /*
     *
     */
    static noinline void*
vrpq_adm_param_get (VrpqSrvSession* session, VrpqSize param_off,
		    VrpqSize param_sz)
{
    static nku8_f param_safe[VRPQ_ADM_PARAM_CHUNK_SIZE];

    return vrpq_adm_param_check(session, param_off, param_sz, param_safe);
}

    /*
     *
     */
    static noinline int
vrpq_adm_req_handle (VrpqSrvChan* chan, VrpqReqMsg* req)
{
    VrpqSrvDev*     dev     = chan->dev;
    VrpqSrvSession* session = chan->session;
    VrpqChanId      chan_id;
    VrpqProcId      proc_id;
    int             ret_val = 0;
    int             diag;

    if (!req->flags.call) {
	return 0;
    }

    down(&dev->adm_lock);

    chan_id = req->chanId;
    proc_id = req->procReq.procId;

    if (chan_id != 0) {
	if (chan_id >= VRPQ_CHAN_MAX) {
	    ret_val = -EINVAL;
	} else if (dev->chans[chan_id]->id == 0) {
	    ret_val = -EINVAL;
	} else {
	    ret_val = -ECHANBROKEN;
	}
    } else {
	if (VRPQ_PROC_ID_GRP_GET(proc_id) != VRPQ_GRP_CONTROL) {
	    ret_val = -EINVAL;
	}
    }

    if (ret_val) {
	diag = vrpq_rsp_send(session, req, ret_val);
	goto out_unlock;
    }

    switch (VRPQ_PROC_ID_FUNC_GET(proc_id)) {

    case VRPQ_CTRL_SESSION_CREATE:
	{
	    diag = vrpq_adm_req_push(session, &dev->sessions[0]->adm, req);
	    if (diag) {
		diag = vrpq_crit_diag(vrpq_rsp_send(session, req, diag), diag);
	    }
	}
	break;

    case VRPQ_CTRL_SESSION_DESTROY:
	{
	    VrpqSessionId         s_id;
	    VrpqSrvSession*       s;
	    VrpqSessionDestroyIn* in;

	    in = VRPQ_ADM_PARAM_IN_GET(session, req, VrpqSessionDestroyIn);

	    diag    = 0;
	    ret_val = -EINVAL;
	    s_id    = in->sessionId;

	    if (s_id < VRPQ_SESSION_MAX) {
		s = dev->sessions[s_id];
		if (s && !s->close.clt) {
		    s->close.clt = 1;
		    diag = vrpq_srv_session_flush(s);
		    vrpq_adm_req_wakeup(&s->adm);
		    vrpq_srv_session_put(s);
		    ret_val = diag;
		}
	    }

	    diag = vrpq_crit_diag(vrpq_rsp_send(session, req, ret_val), diag);
	}
	break;

    case VRPQ_CTRL_CHAN_CREATE :
	{
	    VrpqSessionId     s_id;
	    VrpqSrvSession*   s = NULL;
	    VrpqChanCreateIn* in;

	    in = VRPQ_ADM_PARAM_IN_GET(session, req, VrpqChanCreateIn);

	    diag    = 0;
	    ret_val = -EINVAL;
	    s_id    = in->sessionId;

	    if (s_id < VRPQ_SESSION_MAX) {
		s = dev->sessions[s_id];
		if (s && !s->close.clt) {
		    if (s->close.srv) {
			ret_val = -ECHANBROKEN;
		    } else {
			diag    = vrpq_adm_req_push(session, &s->adm, req);
			ret_val = diag;
		    }
		}
	    }

	    if (ret_val) {
		diag = vrpq_crit_diag(vrpq_rsp_send(session, req, ret_val),
				      diag);
	    }
	}
	break;

    case VRPQ_CTRL_CHAN_DESTROY :
	{
	    VrpqChanId         c_id;
	    VrpqSrvChan*       c;
	    VrpqChanDestroyIn* in;

	    in = VRPQ_ADM_PARAM_IN_GET(session, req, VrpqChanDestroyIn);

	    ret_val = -EINVAL;
	    c_id    = in->chanId;

	    if (c_id < VRPQ_CHAN_MAX) {
		c = dev->chans[c_id];
		if (c && c->id != 0 && !c->close.clt) {
		    c->close.clt = 1;
		    vrpq_reqs_wakeup(c);
		    vrpq_srv_chan_put(c);
		    ret_val = 0;
		}
	    }

	    diag = vrpq_rsp_send(session, req, ret_val);
	}
	break;

    default:
	diag = vrpq_rsp_send(session, req, -EINVAL);
	break;
    }

out_unlock:
    up(&dev->adm_lock);
    return diag;
}

    /*
     *
     */
    static int
vrpq_adm_thread (void* arg)
{
    VrpqSrvChan*     chan  = (VrpqSrvChan*) arg;
    Vlink*           vlink = chan->dev->gen_dev.vlink;
    VrpqChanReqRing* ring  = &chan->chan_req_ring;
    VrpqRingIdx      tail_idx;
    VrpqReqMsg*      req;
    int              diag;

    do {

	diag = vlink_session_create(vlink,
				    chan->session,
				    NULL,
				    &chan->session->vls);
	if (diag) {
	    break;
	}

	chan->vls = chan->session->vls;

	do {

	    for (tail_idx = ring->tail_idx;
		 tail_idx != ring->head_idx;
		 tail_idx++) {

		req  = ring->reqs[tail_idx & VRPQ_CHAN_REQS_MASK];
		diag = vrpq_adm_req_handle(chan, req);

		VLINK_DTRACE(vlink,
			     "admin request treated: procId=0x%x "
			     "inOffset=0x%x outOffset=0x%x diag=%d\n",
			     req->procReq.procId, req->procReq.inOffset,
			     req->procReq.outOffset, diag);

		if (diag) {
		    break;
		}
	    }

	    ring->tail_idx = tail_idx;

	    if (!diag) {
		diag = vrpq_reqs_wait(chan, tail_idx);
	    }

	} while (!diag);

	vlink_session_leave(chan->session->vls);
	vlink_session_put(chan->session->vls);

	if ((diag != -EAGAIN) && (diag != -ECHANBROKEN)) {
	    set_current_state(TASK_INTERRUPTIBLE);
	    schedule_timeout(HZ/10);
	}

    } while (1);

    return diag;
}

    /*
     *
     */
    static noinline void
vrpq_adm_cleanup (VrpqSrvDev* dev)
{
    if (dev->adm_thread) {
	vlx_thread_join(dev->adm_thread);
	kfree(dev->adm_thread);
	dev->adm_thread = NULL;
    }
    if (dev->chans[0]) {
	kfree(dev->chans[0]);
	dev->chans[0] = 0;
    }
    if (dev->sessions[0]) {
	kfree(dev->sessions[0]);
	dev->sessions[0] = 0;
    }
}

    /*
     *
     */
    static noinline int
vrpq_adm_init (VrpqSrvDev* dev)
{
    Vlink*          vlink = dev->gen_dev.vlink;
    VrpqSrvSession* session;
    VrpqSessionId   session_id;
    VrpqSrvChan*    chan;
    VrpqChanId      chan_id;
    vlx_thread_t*   thread;
    int             diag;
    unsigned int    len;
    char*           name;
    unsigned int    i;

    session = kzalloc(sizeof(VrpqSrvSession), GFP_KERNEL);
    if (!session) {
	VLINK_ERROR(vlink, "cannot allocate the admin session struct "
		    "(%d bytes)\n", sizeof(VrpqSrvSession));
	return -ENOMEM;
    }

    session_id = vrpq_srv_session_id_alloc(dev);
    VLINK_ASSERT(session_id == 0);
    vrpq_srv_session_init(session, session_id, dev);
    dev->sessions[session_id] = session;

    chan = kzalloc(sizeof(VrpqSrvChan), GFP_KERNEL);
    if (!chan) {
	VLINK_ERROR(vlink, "cannot allocate the admin channel struct "
		    "(%d bytes)\n", sizeof(VrpqSrvChan));
	return -ENOMEM;
    }

    chan_id = vrpq_srv_chan_id_alloc(dev);;
    VLINK_ASSERT(chan_id == 0);
    vrpq_srv_chan_init(chan, chan_id, session);

    for (i = 0; i < VRPQ_CHAN_MAX; i++) {
	dev->chans[i] = chan;
    }

    len = NK_DEV_VLINK_NAME_LIMIT + sizeof("-xxx") + 4;

    thread = (vlx_thread_t*) kzalloc(sizeof(vlx_thread_t) + len, GFP_KERNEL);
    if (!thread) {
	VLINK_ERROR(vlink,
		    "cannot allocate a vlx_thread_t struct (%d bytes)\n",
		    sizeof(vlx_thread_t) + len);
	return -ENOMEM;
    }

    name = ((char*) thread) + sizeof(vlx_thread_t);
    sprintf(name, "%s-%d", dev->gen_dev.vrpq_drv->name, vlink->nk_vlink->link);

    diag = vlx_thread_start(thread, vrpq_adm_thread, chan, name);
    if (diag) {
	VLINK_ERROR(vlink, "cannot create the VRPQ admin thread\n");
	kfree(thread);
	return diag;
    }

    dev->adm_thread = thread;

    sema_init(&dev->adm_lock, 1);

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_open (struct inode* inode, struct file* file)
{
    unsigned int minor = iminor(file->f_path.dentry->d_inode);
    unsigned int major = imajor(file->f_path.dentry->d_inode);
    VrpqDev*     vrpq_gendev;
    VrpqSrvDev*  vrpq_dev;
    VrpqSrvFile* vrpq_file;

    vrpq_gendev = vrpq_dev_find(major, minor);
    if (!vrpq_gendev) {
	return -ENXIO;
    }
    vrpq_dev = &vrpq_gendev->srv;

    vrpq_file = (VrpqSrvFile*) kzalloc(sizeof(VrpqSrvFile), GFP_KERNEL);
    if (!vrpq_file) {
	return -ENOMEM;
    }

    vrpq_file->type    = VRPQ_FILE_NONE;
    vrpq_file->dev     = vrpq_dev;
    sema_init(&vrpq_file->lock, 1);
    file->private_data = vrpq_file;

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_release (struct inode* inode, struct file* file)
{
    VrpqSrvFile* vrpq_file = file->private_data;
    int          diag      = 0;

    if (down_interruptible(&vrpq_file->lock)) {
	return -ERESTARTSYS;
    }

    if (vrpq_file->type == VRPQ_FILE_SESSION) {
	diag = vrpq_srv_session_close(&vrpq_file->session);
    } else if (vrpq_file->type == VRPQ_FILE_CHAN) {
	diag = vrpq_srv_chan_close(&vrpq_file->chan);
    } else {
	kfree(vrpq_file);
    }

    up(&vrpq_file->lock);

    return diag;
}

    /*
     *
     */
static int vrpq_bad_ioctl (struct VrpqSrvFile* vrpq_file, void* arg,
			   unsigned int sz)
{
    VLINK_ERROR(vrpq_file->dev->gen_dev.vlink, "invalid ioctl command\n");
    return -EINVAL;
}

    /*
     *
     */
static VrpqSrvIoctl vrpq_srv_ioctl_ops[VRPQ_IOCTL_CMD_MAX] = {
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  0 - SESSION_CREATE       */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  1 - CHAN_CREATE          */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  2 - NOTIFY               */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  3 - POST_MULTI           */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  4 - POST_MULTI_NOTIFY    */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  5 - POST_MULTI_CALL      */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  6 - Unused               */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/*  7 - Unused               */
    (VrpqSrvIoctl)vrpq_srv_session_accept,	/*  8 - SESSION_ACCEPT       */
    (VrpqSrvIoctl)vrpq_srv_chan_accept,		/*  9 - CHAN_ACCEPT          */
    (VrpqSrvIoctl)vrpq_srv_shm_map,		/* 10 - SHM_MAP              */
    (VrpqSrvIoctl)vrpq_srv_client_id_get,	/* 11 - CLIENT_ID_GET        */
    (VrpqSrvIoctl)vrpq_receive,			/* 12 - RECEIVE              */
    (VrpqSrvIoctl)vrpq_call_set_reason,		/* 13 - CALL_SET_REASON      */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/* 14 - Unused               */
    (VrpqSrvIoctl)vrpq_bad_ioctl,		/* 15 - Unused               */
};

    /*
     *
     */
    static int
vrpq_srv_ioctl (struct inode* inode, struct file* file,
		unsigned int cmd, unsigned long arg)
{
    VrpqSrvFile* vrpq_file = file->private_data;
    unsigned int nr;
    unsigned int sz;

    nr = _IOC_NR(cmd) & (VRPQ_IOCTL_CMD_MAX - 1);
    sz = _IOC_SIZE(cmd);

    return vrpq_srv_ioctl_ops[nr](vrpq_file, (void*)arg, sz);
}

    /*
     *
     */
    static ssize_t
vrpq_srv_read (struct file* file, char __user* buf, size_t count, loff_t* ppos)
{
    return -EINVAL;
}

    /*
     *
     */
    static ssize_t
vrpq_srv_write (struct file* file, const char __user* buf,
	    size_t count, loff_t* ppos)
{
    return -EINVAL;
}

    /*
     *
     */
    static unsigned int
vrpq_srv_poll(struct file* file, poll_table* wait)
{
    return -ENOSYS;
}

    /*
     *
     */
static struct file_operations vrpq_srv_fops = {
    .owner	= THIS_MODULE,
    .open	= vrpq_srv_open,
    .read	= vrpq_srv_read,
    .write	= vrpq_srv_write,
    .ioctl      = vrpq_srv_ioctl,
    .release	= vrpq_srv_release,
    .poll	= vrpq_srv_poll,
};

    /*
     *
     */
    static int
vrpq_srv_vlink_abort (Vlink* vlink, void* cookie)
{
    VrpqSrvDev*  dev = (VrpqSrvDev*) cookie;
    VrpqSrvChan* chan;
    unsigned int i;

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_abort called\n");

    wake_up_interruptible(&dev->chans[0]->chan_req_ring.wait);
    wake_up_interruptible(&dev->sessions[0]->adm.wait);

    for (i = 1; i < VRPQ_CHAN_MAX; i++) {
	chan = dev->chans[i];
	if (chan->id == 0 || chan->close.all) {
	    continue;
	}
	wake_up_interruptible(&chan->chan_req_ring.wait);
    }

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_vlink_reset (Vlink* vlink, void* cookie)
{
    VrpqSrvDev*  dev      = (VrpqSrvDev*) cookie;
    VrpqReqRing* req_ring = &dev->req_ring;
    VrpqRspRing* rsp_ring = &dev->rsp_alloc.ring;
    VrpqSrvChan* adm_chan = dev->chans[0];

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_reset called\n");

    req_ring->headIdx                     = 0;
    req_ring->tailIdx                     = 0;

    rsp_ring->headIdx                     = 0;
    rsp_ring->tailIdx                     = 0;

    rsp_ring->global->headIdx             = 0;

    adm_chan->chan_req_ring.head_idx      = 0;
    adm_chan->chan_req_ring.tail_idx      = 0;
    adm_chan->chan_req_ring.prev_tail_idx = 0;
    adm_chan->chan_req_ring.wait_flag     = 0;

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_vlink_start (Vlink* vlink, void* cookie)
{
    VrpqSrvDev* dev = (VrpqSrvDev*) cookie;

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_start called\n");

    nkops.nk_xirq_unmask(dev->gen_dev.sxirq);

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_vlink_stop (Vlink* vlink, void* cookie)
{
    VrpqSrvDev*     dev = (VrpqSrvDev*) cookie;
    VrpqSrvChan*    chan;
    VrpqSrvSession* session;
    int             i;

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_stop called\n");

    nkops.nk_xirq_mask(dev->gen_dev.sxirq);

    for (i = 1; i < VRPQ_CHAN_MAX; i++) {
	chan = dev->chans[i];
	if (chan->id == 0 || chan->close.clt) {
	    continue;
	}
	chan->close.clt = 1;
	vrpq_srv_chan_put(chan);
    }

    for (i = 1; i < VRPQ_SESSION_MAX; i++) {
	session = dev->sessions[i];
	if (session == NULL || session->close.clt) {
	    continue;
	}
	session->close.clt = 1;
	vrpq_srv_session_put(session);
    }

    return 0;
}

    /*
     *
     */
    static int
vrpq_srv_vlink_cleanup (Vlink* vlink, void* cookie)
{
    VrpqSrvDev* dev = (VrpqSrvDev*) cookie;

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_cleanup called\n");

    dev->gen_dev.enabled = 0;

    vrpq_adm_cleanup(dev);

    if (dev->chans_alloc) {
	kfree(dev->chans_alloc);
	dev->chans_alloc = NULL;
    }

    if (dev->sessions_alloc) {
	kfree(dev->sessions_alloc);
	dev->sessions_alloc = NULL;
    }

    if (dev->chans) {
	kfree(dev->chans);
	dev->chans = NULL;
    }

    if (dev->sessions) {
	kfree(dev->sessions);
	dev->sessions = NULL;
    }

    vrpq_gen_dev_cleanup(&dev->gen_dev);

    return 0;
}

    /*
     *
     */
static VlinkOpDesc vrpq_srv_vlink_ops[] = {
    { VLINK_OP_RESET,   vrpq_srv_vlink_reset   },
    { VLINK_OP_START,   vrpq_srv_vlink_start   },
    { VLINK_OP_ABORT,   vrpq_srv_vlink_abort   },
    { VLINK_OP_STOP,    vrpq_srv_vlink_stop    },
    { VLINK_OP_CLEANUP, vrpq_srv_vlink_cleanup },
    { 0,                NULL                   },
};

    /*
     *
     */
    int
vrpq_srv_vlink_init (VrpqDrv* vrpq_drv, Vlink* vlink)
{
    VrpqSrvDev*     dev       = &vrpq_drv->devs[vlink->unit].srv;
    VrpqReqRing*    req_ring  = &dev->req_ring;
    VrpqRspAlloc*   rsp_alloc = &dev->rsp_alloc;
    VrpqRspRing*    rsp_ring  = &rsp_alloc->ring;
    VrpqPmemLayout* pmem      = &dev->gen_dev.pmem_layout;
    unsigned int    sz;
    int             diag;

    VLINK_DTRACE(vlink, "vrpq_srv_vlink_init called\n");

    dev->gen_dev.vrpq_drv = vrpq_drv;
    dev->gen_dev.vlink    = vlink;

    if ((diag = vrpq_gen_dev_init(&dev->gen_dev)) != 0) {
	vrpq_srv_vlink_cleanup(vlink, dev);
	return diag;
    }

    dev->peer.xirq_rsp  = dev->gen_dev.cxirq;
    dev->peer.xirq_full = dev->gen_dev.cxirq + 1;
    dev->peer.osid      = dev->gen_dev.vlink->id_peer;

    req_ring->global    = pmem->reqRingGbl;
    req_ring->modMask   = pmem->reqCount - 1;
    req_ring->reqs      = pmem->reqs;

    rsp_ring->global    = pmem->rspRingGbl;
    rsp_ring->modMask   = pmem->rspCount - 1;
    rsp_ring->resps     = pmem->resps;

    sz = VRPQ_SESSION_MAX * sizeof(VrpqSrvSession*);
    dev->sessions = (VrpqSrvSession**) kzalloc(sz, GFP_KERNEL);
    if (!dev->sessions) {
	VLINK_ERROR(vlink, "cannot allocate sessions directory (%d bytes)\n",
		    sz);
	vrpq_srv_vlink_cleanup(vlink, dev);
	return -ENOMEM;
    }

    sz = VRPQ_CHAN_MAX * sizeof(VrpqSrvChan*);
    dev->chans = (VrpqSrvChan**) kzalloc(sz, GFP_KERNEL);
    if (!dev->chans) {
	VLINK_ERROR(vlink, "cannot allocate channels directory (%d bytes)\n",
		    sz);
	vrpq_srv_vlink_cleanup(vlink, dev);
	return -ENOMEM;
    }

    sz = sizeof(VrpqBitmapAlloc);

    dev->chans_alloc = (VrpqBitmapAlloc*) kzalloc(sz, GFP_KERNEL);
    if (!dev->chans_alloc) {
	VLINK_ERROR(vlink, "cannot allocate channels allocator (%d bytes)\n",
		    sz);
	vrpq_srv_vlink_cleanup(vlink, dev);
	return -ENOMEM;
    }

    dev->sessions_alloc = (VrpqBitmapAlloc*) kzalloc(sz, GFP_KERNEL);
    if (!dev->sessions_alloc) {
	VLINK_ERROR(vlink, "cannot allocate sessions allocator (%d bytes)\n",
		    sz);
	vrpq_srv_vlink_cleanup(vlink, dev);
	return -ENOMEM;
    }

    spin_lock_init(&rsp_alloc->lock);
    spin_lock_init(&dev->chans_alloc->lock);
    spin_lock_init(&dev->sessions_alloc->lock);

    diag = vrpq_adm_init(dev);
    if (diag) {
	vrpq_srv_vlink_cleanup(vlink, dev);
	return diag;
    }

    diag = vrpq_xirq_attach(&dev->gen_dev, dev->gen_dev.sxirq,
			    vrpq_reqs_handler);
    if (diag) {
	vrpq_srv_vlink_cleanup(vlink, dev);
	return diag;
    }

    dev->gen_dev.enabled = 1;

    return 0;
}

    /*
     *
     */
    int
vrpq_srv_drv_init (VlinkDrv* parent_drv, VrpqDrv* vrpq_drv)
{
    int diag;

    vrpq_drv->parent_drv   = parent_drv;
    vrpq_drv->vops         = vrpq_srv_vlink_ops;
    vrpq_drv->fops         = &vrpq_srv_fops;
    vrpq_drv->chrdev_major = 0;
    vrpq_drv->class        = NULL;
    vrpq_drv->devs         = NULL;

    if ((diag = vrpq_gen_drv_init(vrpq_drv)) != 0) {
	vrpq_srv_drv_cleanup(vrpq_drv);
	return diag;
    }

    return 0;
}

    /*
     *
     */
    void
vrpq_srv_drv_cleanup (VrpqDrv* vrpq_drv)
{
    vrpq_gen_drv_cleanup(vrpq_drv);
}

    /*
     *
     */
    static int
vrpq_be_module_init (void)
{
    return 0;
}

    /*
     *
     */
    static void
vrpq_be_module_exit (void)
{

}

module_init(vrpq_be_module_init);
module_exit(vrpq_be_module_exit);
