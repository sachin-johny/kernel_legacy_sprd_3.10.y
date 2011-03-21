/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VMQ driver implementation
 *
 *  Copyright (C) 2009-2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski <adam.mirowski@virtuallogix.com>
 *
 ****************************************************************
 */

/*----- System header files -----*/

#include <linux/version.h>
#include <linux/module.h>	/* __exit, __init */
#include <linux/sched.h>	/* TASK_INTERRUPTIBLE */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/proc_fs.h>
#include <nk/nkern.h>

/*----- Local configuration -----*/

#if 0
#define VMQ_DEBUG
#endif

#if 0
#undef VMQ_ODEBUG		/* Activates old/noisy debug traces */
#endif

/*----- Local header files -----*/

#include "vlx-vmq.h"

/*----- Tracing -----*/

#ifdef VMQ_DEBUG
#define DTRACE(x...)	do {printk ("(%d) %s: ", current->tgid, __func__);\
			    printk (x);} while (0)
#define VMQ_BUG_ON(x)	BUG_ON(x)
#else
#define DTRACE(x...)
#define VMQ_BUG_ON(x)
#endif

#ifdef VMQ_ODEBUG
#define OTRACE(x...)	DTRACE(x)
#else
#define OTRACE(x...)
#endif

#define WTRACE(x...)	printk (KERN_WARNING "VMQ: " x)
#define ETRACE(x...)	printk (KERN_ERR "VMQ: " x)

/*----- Locking -----*/

    /* Locking cannot be nested */

#define VMQ_LOCK(_spinlock, _flags) \
    do { \
	OTRACE ("lock\n"); \
	spin_lock_irqsave ((_spinlock), (_flags)); \
    } while (0)

#define VMQ_UNLOCK(_spinlock, _flags) \
    do { \
	OTRACE ("unlock\n"); \
	spin_unlock_irqrestore ((_spinlock), (_flags)); \
    } while (0)

/*----- Version compatibility functions -----*/

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,7)
    static inline void*
kzalloc (size_t size, unsigned flags)
{
    void* ptr = kmalloc (size, flags);
    if (ptr) {
	memset (ptr, 0, size);
    }
    return ptr;
}
#endif

#ifndef container_of
#define container_of(ptr, type, member) \
	((type*)((char*)(ptr)-(unsigned long)(&((type*) 0)->member)))
#endif

#ifndef list_for_each_entry
#define list_for_each_entry(pos, head, member)				\
	for (pos = list_entry((head)->next, typeof(*pos), member),	\
		     prefetch(pos->member.next);			\
	     &pos->member != (head);					\
	     pos = list_entry(pos->member.next, typeof(*pos), member),	\
		     prefetch(pos->member.next))
#endif

#define list_for_each_entry_type(pos, type, head, member) \
	list_for_each_entry(pos, head, member)

/*----- Shared data types -----*/

#define VMQ_ALIGN __attribute__ ((aligned (L1_CACHE_BYTES)))

    /* Header located at pmem start */

typedef struct {
    volatile nku32_f	unsafe_p_idx;		/* producer index */
    volatile nku32_f	unsafe_c_idx;		/* consumer index */
    volatile nku8_f	unsafe_stopped;
    volatile nku8_f	unused1 [3];
    volatile nku32_f	unused2;
} VMQ_ALIGN vmq_head;

typedef char vmq_check1_t [sizeof (vmq_head) % L1_CACHE_BYTES ? -1 : 1];

    /* One index slot */

typedef struct {
    volatile nku32_f	unsafe_ss_number;
} vmq_is;

#define VMQ_PMEM_ID	0
#define VMQ_RXIRQ_ID	1
#define VMQ_TXIRQ_ID	2

typedef struct {
    vmq_head	head;
    vmq_is	is [1] VMQ_ALIGN;
	/* Short slots */
	/* Long slots */
} vmq_pmem;

typedef char vmq_check2_t [sizeof (vmq_pmem) % L1_CACHE_BYTES ? -1 : 1];

#define VMQ_ROUNDUP(x,y)	((((x)+(y)-1) / (y)) * (y))
#define VMQ_SIZEOF_VMQ_PMEM \
	(sizeof (vmq_pmem) - VMQ_ROUNDUP (sizeof (vmq_is), L1_CACHE_BYTES))

typedef char vmq_check3_t [VMQ_SIZEOF_VMQ_PMEM % L1_CACHE_BYTES ? -1 : 1];

/*----- Helper macros -----*/

    /*
     *  Typecast allows to deal with 64 bit machines and situations
     *  where the producer has already overflown 32 bits and the
     *  consumer not yet.
     */

#define VMQ_UNSAFE_HEAD_SPACE(_head) \
    (nku32_f) ((_head)->unsafe_p_idx - (_head)->unsafe_c_idx)

#define VMQ_UNSAFE_HEAD_SPACE_OK(_xx) \
    (VMQ_UNSAFE_HEAD_SPACE (&(_xx)->pmem->head) <= (_xx)->config.msg_count)

#define VMQ_UNSAFE_HEAD_SPACE_ASSERT(_xx) \
    do {VMQ_BUG_ON (!VMQ_UNSAFE_HEAD_SPACE_OK(_xx));} while (0)

#define VMQ_SAFE_HEAD_SPACE(_xx) \
    (nku32_f) ((_xx)->safe_p_idx - (_xx)->safe_c_idx)

#define VMQ_SAFE_HEAD_SPACE_OK(_xx) \
    (VMQ_SAFE_HEAD_SPACE (_xx) <= (_xx)->config.msg_count)

#define VMQ_SAFE_XX_IS_FULL(_xx) \
    ((nku32_f) ((_xx)->safe_p_idx - (_xx)->last_x_idx) >= \
    (_xx)->config.msg_count)

#define VMQ_UNSAFE_C_PENDING(_xx) \
    (nku32_f) ((_xx)->pmem->head.unsafe_c_idx - (_xx)->last_x_idx)

#define VMQ_SAFE_C_PENDING(_xx) \
    (nku32_f) ((_xx)->safe_c_idx - (_xx)->last_x_idx)

#define VMQ_UNSAFE_P_PENDING(_xx) \
    (nku32_f) ((_xx)->pmem->head.unsafe_p_idx - (_xx)->last_x_idx)

#define VMQ_SAFE_P_PENDING(_xx) \
    (nku32_f) ((_xx)->safe_p_idx - (_xx)->last_x_idx)

    /* Macros for Index slots */

#define VMQ_TX_INDEX_SAFE_P_IDX(_xx) \
    ((_xx)->pmem->is + ((_xx)->safe_p_idx & (_xx)->ring_index_mask))

#define VMQ_RX_INDEX_SAFE_C_IDX(_xx) \
    ((_xx)->pmem->is + ((_xx)->safe_c_idx & (_xx)->ring_index_mask))

#define VMQ_XX_INDEX_LAST_X_IDX(_xx) \
    ((_xx)->pmem->is + ((_xx)->last_x_idx & (_xx)->ring_index_mask))

    /* Macros for Short slots */

#define VMQ_SHORT_SLOT(_xx, slot) \
	((_xx)->ss_area + (_xx)->config.msg_max * (slot))
#define VMQ_SHORT_SLOT_NUM(_xx,_ss) \
	(((char*)(_ss) - (_xx)->ss_area)) / (_xx)->config.msg_max

    /* Macros for Long slots */

#define VMQ_LONG_SLOT(_xx, slot) \
	((_xx)->ls_area + (_xx)->config.data_max * (slot))
#define VMQ_LONG_SLOT_NUM(_xx, _ls) \
	(((char*)(_ls) - (_xx)->ls_area)) / (_xx)->config.data_max

/*----- Generic functions -----*/

    static inline void
vmq_sysconf_trigger (NkOsId osid)
{
    DTRACE ("os %d\n", osid);
    nkops.nk_xirq_trigger (NK_XIRQ_SYSCONF, osid);
}

/*----- Generic channel management -----*/

typedef struct {
    NkXIrq		local_xirq;
    NkXIrqId		xid;		/* Handler id */
    NkXIrq		peer_xirq;
    NkDevVlink*		vlink;
    vmq_pmem*		pmem;
    NkPhAddr		paddr;
    _Bool		aborted;
    nku32_f		last_x_idx;	/* last producer/consumer index */
    nku32_f		safe_p_idx;
    nku32_f		safe_c_idx;
    vmq_xx_config_t	config;
    unsigned		ring_index_mask;
    char*		ss_area;	/* Inside pmem */
    char*		ls_area;	/* Inside pmem */
    unsigned		ss_checked_out;
    unsigned		ls_checked_out;
    spinlock_t*		spinlock;
} vmq_xx;

    /* VMQ_LOCK must be taken on entry */

    static void
vmq_xx_abort (vmq_xx* xx, _Bool server)
{
    xx->aborted = true;
    if (server) {
	xx->vlink->s_state = NK_DEV_VLINK_OFF;
	vmq_sysconf_trigger (xx->vlink->c_id);
    } else {
	xx->vlink->c_state = NK_DEV_VLINK_OFF;
	vmq_sysconf_trigger (xx->vlink->s_id);
    }
}

    /* Performed in sender. Only unsafe_c_idx can have changed */

    static signed
vmq_xx_return_msg_receive (vmq_xx* xx, void** msg,
			   const struct list_head* ss_heads)
{
    vmq_head*		head = &xx->pmem->head;
    unsigned long	flags;

    VMQ_LOCK (xx->spinlock, flags);
    OTRACE ("pending %d\n", VMQ_UNSAFE_C_PENDING (xx));
	/* Sample unsafe value before checking it */
    xx->safe_c_idx = head->unsafe_c_idx;
    if (!VMQ_SAFE_HEAD_SPACE_OK (xx)) {
	vmq_xx_abort (xx, false);
	VMQ_UNLOCK (xx->spinlock, flags);
	return -ESTALE;
    }
    if (VMQ_SAFE_C_PENDING (xx) > 0) {
	const unsigned ss_number =
	    VMQ_XX_INDEX_LAST_X_IDX (xx)->unsafe_ss_number;

	if (ss_number >= xx->config.msg_count ||
		!list_empty (ss_heads + ss_number)) {
	    vmq_xx_abort (xx, false);
	    VMQ_UNLOCK (xx->spinlock, flags);
	    return -ESTALE;
	}
	*msg = VMQ_SHORT_SLOT (xx, ss_number);
	OTRACE ("msg %p p_idx %d last_x_idx %d c_idx %d\n",
		*msg, head->unsafe_p_idx, xx->last_x_idx, head->unsafe_c_idx);
	++xx->last_x_idx;
	++xx->ss_checked_out;
	VMQ_UNLOCK (xx->spinlock, flags);
	return 0;
    }
    VMQ_UNLOCK (xx->spinlock, flags);
    return -EAGAIN;
}

    /* VMQ_LOCK must be taken */

    static inline _Bool
vmq_xx_ring_is_full (vmq_xx* xx)
{
    return VMQ_SAFE_XX_IS_FULL (xx);
}

    /* VMQ_LOCK must be taken */

    static inline _Bool
vmq_xx_all_returned (vmq_xx* xx)
{
    return !xx->ss_checked_out && !xx->ls_checked_out;
}

static void vmq_xx_sysconf_notify (vmq_xx* xx);

    /* Only called by vmq_msg_send() */

    static void
vmq_xx_msg_send (vmq_xx* xx, void* msg)
{
    vmq_pmem*		xx_pmem = xx->pmem;
    const unsigned	ss_number = VMQ_SHORT_SLOT_NUM (xx, msg);
    unsigned long	flags;

    VMQ_BUG_ON (ss_number >= xx->config.msg_count);
    VMQ_LOCK (xx->spinlock, flags);
    --xx->ss_checked_out;
    if (xx->aborted) {
	const _Bool sysconf_notify = vmq_xx_all_returned (xx);

	VMQ_UNLOCK (xx->spinlock, flags);
	    /* We do not wake-up anybody, this is done by (*link_off)() */
	if (sysconf_notify) {
	    vmq_xx_sysconf_notify (xx);
	}
	return;
    }
	/* Write ss_number at position safe_p_idx */
    VMQ_TX_INDEX_SAFE_P_IDX (xx)->unsafe_ss_number = ss_number;
    xx_pmem->head.unsafe_p_idx = ++xx->safe_p_idx;
    if (vmq_xx_ring_is_full (xx)) {
	xx_pmem->head.unsafe_stopped = 1;
    }
    VMQ_UNLOCK (xx->spinlock, flags);
    OTRACE ("xirq %d os %d\n", xx->peer_xirq, xx->vlink->s_id);
    nkops.nk_xirq_trigger (xx->peer_xirq, xx->vlink->s_id);
}

    /* Only called by vmq_msg_free() and vmq_msg_return() */

    static void
vmq_xx_msg_free (vmq_xx* xx, void* msg, _Bool signal)
{
    vmq_pmem*		xx_pmem = xx->pmem;
    vmq_head*		head	= &xx_pmem->head;
	/*
	 *  Convert msg pointer to a ss_number
	 *  and place it in index at slot head.c_idx.
	 */
    const unsigned	ss_number = VMQ_SHORT_SLOT_NUM (xx, msg);
    unsigned long	flags;

    VMQ_BUG_ON (ss_number >= xx->config.msg_count);
    OTRACE ("msg %p p_idx %d last_x_idx %d c_idx %d slot %d\n", msg,
	    head->unsafe_p_idx, xx->last_x_idx, head->unsafe_c_idx, ss_number);
    VMQ_LOCK (xx->spinlock, flags);
    --xx->ss_checked_out;
    if (xx->aborted) {
	const _Bool sysconf_notify = vmq_xx_all_returned (xx);

	VMQ_UNLOCK (xx->spinlock, flags);
	    /* We do not wake-up anybody, this is done by (*link_off)() */
	if (sysconf_notify) {
	    vmq_xx_sysconf_notify (xx);
	}
	return;
    }
    VMQ_RX_INDEX_SAFE_C_IDX (xx)->unsafe_ss_number = ss_number;
    head->unsafe_c_idx = ++xx->safe_c_idx;
    VMQ_UNLOCK (xx->spinlock, flags);

	/* Send tx xirq if producer ring was stopped (full) */
    if (head->unsafe_stopped || signal) {
	nkops.nk_xirq_trigger (xx->peer_xirq, xx->vlink->c_id);
    }
}

    /* Performed in receiver. Only unsafe_p_idx can have changed */

    static signed
vmq_xx_msg_receive (vmq_xx* xx, void** msg)
{
    vmq_head*		head = &xx->pmem->head;
    unsigned long	flags;

    VMQ_LOCK (xx->spinlock, flags);
    OTRACE ("pending %d\n", VMQ_UNSAFE_P_PENDING (xx));
	/* Sample unsafe value before checking it */
    xx->safe_p_idx = head->unsafe_p_idx;
    if (!VMQ_SAFE_HEAD_SPACE_OK (xx)) {
	vmq_xx_abort (xx, true);
	VMQ_UNLOCK (xx->spinlock, flags);
	return -ESTALE;
    }
    if (VMQ_SAFE_P_PENDING (xx) > 0) {
	const unsigned ss_number =
	    VMQ_XX_INDEX_LAST_X_IDX (xx)->unsafe_ss_number;

	if (ss_number >= xx->config.msg_count) {
	    vmq_xx_abort (xx, true);
	    VMQ_UNLOCK (xx->spinlock, flags);
	    return -ESTALE;
	}
	*msg = VMQ_SHORT_SLOT (xx, ss_number);
	OTRACE ("msg %p p_idx %d last_x_idx %d c_idx %d\n",
		*msg, head->unsafe_p_idx, xx->last_x_idx, head->unsafe_c_idx);
	++xx->last_x_idx;
	++xx->ss_checked_out;
	VMQ_UNLOCK (xx->spinlock, flags);
	return 0;
    }
    VMQ_UNLOCK (xx->spinlock, flags);

	/* Send tx xirq if producer ring was stopped (full) */
    if (head->unsafe_stopped) {
	nkops.nk_xirq_trigger (xx->peer_xirq, xx->vlink->c_id);
    }
    return -EAGAIN;
}

    static inline unsigned
vmq_xx_config_ls_total (const vmq_xx_config_t* config)
{
    return config->data_count * config->data_max;
}

    static inline size_t
vmq_xx_config_pmem_size (const vmq_xx_config_t* config)
{
    return VMQ_SIZEOF_VMQ_PMEM +
	config->msg_count * (sizeof (vmq_is) + config->msg_max) +
	vmq_xx_config_ls_total (config);
}

    static void
vmq_xx_finish (vmq_xx* xx)
{
    if (xx->xid) {
	nkops.nk_xirq_detach (xx->xid);
	xx->xid = 0;
    }
    if (xx->pmem) {
	nkops.nk_mem_unmap (xx->pmem, xx->paddr,
			    vmq_xx_config_pmem_size (&xx->config));
	xx->pmem = NULL;
    }
}

static void vmq_tx_hdl (void* cookie, NkXIrq xirq);
static void vmq_rx_hdl (void* cookie, NkXIrq xirq);

    static signed
vmq_xx_init (vmq_xx* xx, NkDevVlink* vlink, _Bool tx,
	     const vmq_xx_config_t* config, spinlock_t* spinlock)
{
    signed	diag;

    xx->config = *config;
    xx->config.msg_max  = VMQ_ROUNDUP (xx->config.msg_max,  L1_CACHE_BYTES);
    xx->config.data_max = VMQ_ROUNDUP (xx->config.data_max, L1_CACHE_BYTES);
    xx->spinlock = spinlock;

    if (config->msg_count <= 0 ||
	(config->msg_count & (config->msg_count-1))) return -EINVAL;

    xx->ring_index_mask = config->msg_count - 1;

    xx->vlink = vlink;
    xx->paddr = nkops.nk_pmem_alloc (nkops.nk_vtop (vlink), VMQ_PMEM_ID,
				     vmq_xx_config_pmem_size (&xx->config));
    if (!xx->paddr) {
	ETRACE ("OS %d->OS %d link %d %s pmem alloc failed.\n",
		vlink->c_id, vlink->s_id, vlink->link,
		tx ? "client" : "server");
	return -ENOMEM;
    }
    xx->pmem = (vmq_pmem*) nkops.nk_mem_map (xx->paddr,
					     vmq_xx_config_pmem_size
					     (&xx->config));
    if (!xx->pmem) {
	ETRACE ("Error while mapping\n");
	return -EAGAIN;
    }
    xx->ss_area = (char*) &xx->pmem->is [xx->config.msg_count];
    xx->ls_area = VMQ_SHORT_SLOT (xx, xx->config.msg_count);

    DTRACE ("%s: bytes %x phys %llx virt %p +%x +%x +%x +%x\n",
	    tx ? "tx" : "rx",
	    vmq_xx_config_pmem_size (&xx->config), (long long) xx->paddr,
	    xx->pmem, sizeof ((vmq_pmem*) 0)->head,
	    xx->config.msg_count * sizeof (vmq_is),
	    xx->config.msg_count * xx->config.msg_max,
	    vmq_xx_config_ls_total (&xx->config));

    xx->local_xirq = nkops.nk_pxirq_alloc (nkops.nk_vtop (vlink),
					   tx ? VMQ_TXIRQ_ID : VMQ_RXIRQ_ID,
					   tx ? vlink->c_id : vlink->s_id, 1);
    if (!xx->local_xirq) {
	ETRACE ("OS %d->OS %d link %d server pxirq alloc failed.\n",
		vlink->c_id, vlink->s_id, vlink->link);
	diag = -ENOMEM;
	goto error;
    }
    xx->xid = nkops.nk_xirq_attach (xx->local_xirq,
				    tx ? vmq_tx_hdl : vmq_rx_hdl, xx);
    if (!xx->xid) {
        ETRACE ("OS %d->OS %d link %d server cannot attach xirq handler.\n",
		vlink->c_id, vlink->s_id, vlink->link);
        diag = -ENOMEM;
	goto error;
    }
    xx->peer_xirq = nkops.nk_pxirq_alloc (nkops.nk_vtop (vlink),
					  tx ? VMQ_RXIRQ_ID : VMQ_TXIRQ_ID,
					  tx ? vlink->s_id : vlink->c_id, 1);
    if (!xx->peer_xirq) {
	ETRACE ("OS %d->OS %d link %d client pxirq alloc failed.\n",
		vlink->c_id, vlink->s_id, vlink->link);
	diag = -ENOMEM;
	goto error;
    }
    return 0;

error:
    vmq_xx_finish (xx);
    return diag;
}

    static inline void
vmq_xx_reset_rx (vmq_xx* xx)
{
    DTRACE ("\n");
    xx->last_x_idx		= 0;
    xx->safe_c_idx		= 0;
    xx->pmem->head.unsafe_c_idx	= 0;
}

    static inline void
vmq_xx_reset_tx (vmq_xx* xx)
{
    DTRACE ("\n");
    xx->last_x_idx		= 0;
    xx->safe_p_idx		= 0;
    xx->pmem->head.unsafe_p_idx	= 0;
    xx->pmem->head.unsafe_stopped	= 0;
}

    static inline _Bool
vmq_xx_vlink_on (vmq_xx* xx)
{
    return xx->vlink->c_state == NK_DEV_VLINK_ON &&
	   xx->vlink->s_state == NK_DEV_VLINK_ON;
}

/*----- Transmission channel: management -----*/

typedef struct {
    vmq_xx		xx;
    struct list_head	free_ss;
    struct list_head*	ss_heads;
    struct list_head	free_ls;
    struct list_head*	ls_heads;
    wait_queue_head_t	slots_wait_queue;
} vmq_tx;

    /* VMQ_LOCK should be taken */

    static inline void
vmq_tx_slots_freed (vmq_tx* tx)
{
    OTRACE ("\n");
    wake_up (&tx->slots_wait_queue);
}

    /* VMQ_LOCK must NOT be taken */

    static void
vmq_tx_slots_wait (vmq_tx* tx)
{
    DECLARE_WAITQUEUE (wait, current);

    DTRACE ("\n");
    set_current_state (TASK_INTERRUPTIBLE);
    add_wait_queue (&tx->slots_wait_queue, &wait);
    schedule();
    remove_wait_queue (&tx->slots_wait_queue, &wait);
}

    /*
     *  VMQ_LOCK must be taken.
     *  Only called by vmq_tx_return_msg_free() and vmq_tx_slots_init().
     */

    static inline void
vmq_tx_enqueue_ss (vmq_tx* tx, unsigned ss_number)
{
    VMQ_BUG_ON (ss_number >= tx->xx.config.msg_count);
    list_add (tx->ss_heads + ss_number, &tx->free_ss);
}

    /*
     *  VMQ_LOCK must be taken
     *  Only called by vmq_tx_msg_allocate().
     */

    static inline char*
vmq_tx_dequeue_ss (vmq_tx* tx)
{
    struct list_head* lh = tx->free_ss.next;

    list_del_init (lh);
	/* Now list_empty(lh) is true */
    ++tx->xx.ss_checked_out;
    return VMQ_SHORT_SLOT (&tx->xx, lh - tx->ss_heads);
}

    /* Only called by vmq_tx_notify() and vmq_return_msg_free() */

    static void
vmq_tx_return_msg_free (vmq_tx* tx, void* msg)
{
    _Bool		sysconf_notify;
    unsigned long	flags;

    VMQ_LOCK (tx->xx.spinlock, flags);
    vmq_tx_enqueue_ss (tx, VMQ_SHORT_SLOT_NUM (&tx->xx, msg));
    --tx->xx.ss_checked_out;
    sysconf_notify = tx->xx.aborted && vmq_xx_all_returned (&tx->xx);
    VMQ_UNLOCK (tx->xx.spinlock, flags);
    if (sysconf_notify) {
	vmq_xx_sysconf_notify (&tx->xx);
    }
}

    /* VMQ_LOCK must be taken */

    static inline void
vmq_tx_enqueue_ls (vmq_tx* tx, unsigned ls_number)
{
    VMQ_BUG_ON (ls_number >= tx->xx.config.data_count);
    list_add (tx->ls_heads + ls_number, &tx->free_ls);
}

    /* VMQ_LOCK must be taken */

    static inline nku32_f
vmq_tx_dequeue_ls (vmq_tx* tx)
{
    struct list_head* lh = tx->free_ls.next;

    list_del (lh);
    ++tx->xx.ls_checked_out;
    return VMQ_LONG_SLOT (&tx->xx, lh - tx->ls_heads) - tx->xx.ls_area;
}

static void vmq_tx_notify (vmq_tx* tx);

    static void
vmq_tx_hdl (void* cookie, NkXIrq xirq)
{
    vmq_tx*	tx = cookie;

    (void) xirq;
    OTRACE ("xirq %d pending %d\n", xirq, VMQ_UNSAFE_C_PENDING (&tx->xx));
    vmq_tx_notify (tx);
    if (tx->xx.pmem->head.unsafe_stopped) {
	unsigned long flags;

	VMQ_LOCK (tx->xx.spinlock, flags);
	if (!vmq_xx_ring_is_full (&tx->xx)) {
	    tx->xx.pmem->head.unsafe_stopped = 0;
		/* wake sending */
	    vmq_tx_slots_freed (tx);
	}
	VMQ_UNLOCK (tx->xx.spinlock, flags);
    }
}

    static inline signed
vmq_tx_msg_allocate (vmq_tx* tx, unsigned data_len, void** msg,
		     unsigned* data_offset, _Bool nonblocking)
{
    vmq_pmem*		tx_pmem = tx->xx.pmem;
    vmq_head*		head    = &tx_pmem->head;
    unsigned long	flags;

    if (!data_offset) data_len = 0;
    OTRACE ("data_len %d\n", data_len);
    if (data_len > tx->xx.config.data_max) {
	ETRACE ("Data size %u is too large for tx.\n", data_len);
	return -E2BIG;
    }
    while (true) {
	if (tx->xx.vlink->s_state != NK_DEV_VLINK_ON) {
		/*
		 *  This trace can be issued a great many times before
		 *  the handshake is finally performed to turn the
		 *  device to aborted state (this depends on the
		 *  priority of sysconf thread versus request threads)
		 *  and before link users notices the device is definitely
		 *  lost, so do not flood the console with warnings
		 *  during normal usage.
		 */
	    DTRACE ("Peer driver %d not ready\n", tx->xx.vlink->s_id);
	    return -EAGAIN;
	}
	VMQ_LOCK (tx->xx.spinlock, flags);
	OTRACE ("tx: p_idx %d c_idx %d last_x_idx %d\n",
		head->unsafe_p_idx, head->unsafe_c_idx, tx->xx.last_x_idx);
	if (tx->xx.aborted) {
	    VMQ_UNLOCK (tx->xx.spinlock, flags);
	    return -ECONNABORTED;
	}
	VMQ_UNSAFE_HEAD_SPACE_ASSERT (&tx->xx);
	if (!vmq_xx_ring_is_full (&tx->xx) &&
		(!data_len || !list_empty (&tx->free_ls)))
	    break;
	head->unsafe_stopped = 1;
	VMQ_UNLOCK (tx->xx.spinlock, flags);
	if (nonblocking) return -EAGAIN;
	DTRACE ("tx ring is full\n");
	vmq_tx_slots_wait (tx);
	if (signal_pending (current)) return -EINTR;
    }
    *msg = vmq_tx_dequeue_ss (tx);
    if (data_offset) {
	*data_offset = data_len ? vmq_tx_dequeue_ls (tx) : 0;
    }
    VMQ_UNLOCK (tx->xx.spinlock, flags);
    OTRACE ("msg %p\n", *msg);
    return 0;
}

    /*
     *  VMQ_LOCK must NOT be taken.
     *  Only called from vmq_data_free().
     */

    static inline void
vmq_tx_data_free (vmq_tx* tx, unsigned data_offset)
{
    char*		ls = tx->xx.ls_area + data_offset;
    const unsigned	ls_number = VMQ_LONG_SLOT_NUM (&tx->xx, ls);
    _Bool		sysconf_notify;
    unsigned long	flags;

    VMQ_LOCK (tx->xx.spinlock, flags);
    vmq_tx_enqueue_ls (tx, ls_number);
    --tx->xx.ls_checked_out;
    sysconf_notify = tx->xx.aborted && vmq_xx_all_returned (&tx->xx);
    vmq_tx_slots_freed (tx);
    VMQ_UNLOCK (tx->xx.spinlock, flags);
    if (sysconf_notify) {
	vmq_xx_sysconf_notify (&tx->xx);
    }
}

    /* Called by vmq_tx_init() */
    /* Called by vmq_tx_vlink_off_completed */

    static void
vmq_tx_slots_init (vmq_tx* tx)
{
    unsigned slot;

    DTRACE ("\n");
    INIT_LIST_HEAD (&tx->free_ss);
    INIT_LIST_HEAD (&tx->free_ls);
    for (slot = 0; slot < tx->xx.config.msg_count; ++slot) {
	vmq_tx_enqueue_ss (tx, slot);
    }
    for (slot = 0; slot < tx->xx.config.data_count; ++slot) {
	vmq_tx_enqueue_ls (tx, slot);
    }
    VMQ_BUG_ON (tx->xx.ss_checked_out);
    VMQ_BUG_ON (tx->xx.ls_checked_out);
}

    /* VMQ_LOCK must NOT be taken */

    static _Bool
vmq_tx_abort (vmq_tx* tx, void* cookie)
{
    unsigned long flags;

    (void) cookie;
    VMQ_LOCK (tx->xx.spinlock, flags);
    tx->xx.aborted = true;
    vmq_tx_slots_freed (tx);	/* Unblock waiters */
    VMQ_UNLOCK (tx->xx.spinlock, flags);
    return false;
}

    static _Bool
vmq_tx_handshake (vmq_tx* tx)
{
    volatile int*	my_state   = &tx->xx.vlink->c_state;
    volatile int*	peer_state = &tx->xx.vlink->s_state;
    _Bool		need_sysconf = false;

    DTRACE ("entry %d->%d\n", *my_state, *peer_state);

    switch (*my_state) {
    case NK_DEV_VLINK_OFF:
	if (*peer_state != NK_DEV_VLINK_ON) {
	    vmq_xx_reset_tx (&tx->xx);
	    *my_state = NK_DEV_VLINK_RESET;
	    need_sysconf = true;
	}
	break;
    case NK_DEV_VLINK_RESET:
	if (*peer_state != NK_DEV_VLINK_OFF) {
	    *my_state = NK_DEV_VLINK_ON;
	    need_sysconf = true;
	}
	break;
    case NK_DEV_VLINK_ON:
	if (*peer_state == NK_DEV_VLINK_OFF) {
	    vmq_tx_abort (tx, NULL);
	}
	break;
    }
    DTRACE ("exit %d->%d\n", *my_state, *peer_state);
    return need_sysconf;
}

    static void
vmq_tx_vlink_off_completed (vmq_tx* tx)
{
    if (tx->xx.vlink->s_state == NK_DEV_VLINK_OFF) {
	vmq_xx_reset_tx (&tx->xx);
	vmq_tx_slots_init (tx);
	tx->xx.vlink->c_state = NK_DEV_VLINK_RESET;
	tx->xx.aborted = false;
	DTRACE ("exit %d->%d\n", tx->xx.vlink->c_state, tx->xx.vlink->s_state);
    }
}

    static void
vmq_tx_finish (vmq_tx* tx)
{
    tx->xx.vlink->c_state = NK_DEV_VLINK_OFF;
    kfree (tx->ss_heads);
    kfree (tx->ls_heads);
    vmq_xx_finish (&tx->xx);
}

    static signed
vmq_tx_init (vmq_tx* tx, NkDevVlink* tx_vlink, const vmq_xx_config_t* config,
	     spinlock_t* spinlock)
{
    signed diag;

    DTRACE ("\n");
    diag = vmq_xx_init (&tx->xx, tx_vlink, true /*tx*/, config, spinlock);
    if (diag) return diag;	/* Error message already issued */

    tx->ss_heads = kzalloc (config->msg_count * sizeof (struct list_head),
			    GFP_KERNEL);
    if (!tx->ss_heads) {
	ETRACE ("Could not allocate memory for vmq tx.\n");
	vmq_tx_finish (tx);
	return -ENOMEM;
    }
    if (config->data_count) {
	tx->ls_heads = kzalloc (config->data_count * sizeof (struct list_head),
				GFP_KERNEL);
	if (!tx->ls_heads) {
	    ETRACE ("Could not allocate memory for vmq tx.\n");
	    vmq_tx_finish (tx);
	    return -ENOMEM;
	}
    }
    vmq_tx_slots_init (tx);
    init_waitqueue_head (&tx->slots_wait_queue);
    return 0;
}

/*----- Reception channel: management -----*/

typedef struct {
    vmq_xx		xx;
} vmq_rx;

static void vmq_rx_notify (vmq_rx* rx);

    static void
vmq_rx_hdl (void* cookie, NkXIrq xirq)
{
    vmq_rx* rx = cookie;

    (void) xirq;
    OTRACE ("xirq %d pending %d\n", xirq, VMQ_UNSAFE_P_PENDING (&rx->xx));
    vmq_rx_notify (rx);
}

    static _Bool
vmq_rx_abort (vmq_rx* rx, void* cookie)
{
    (void) cookie;
    rx->xx.aborted = true;
    return false;
}

    static _Bool
vmq_rx_handshake (vmq_rx* rx)
{
    volatile int*	my_state   = &rx->xx.vlink->s_state;
    volatile int*	peer_state = &rx->xx.vlink->c_state;
    _Bool		need_sysconf = false;

    DTRACE ("entry %d->%d\n", *peer_state, *my_state);

    switch (*my_state) {
    case NK_DEV_VLINK_OFF:
	if (*peer_state != NK_DEV_VLINK_ON) {
	    vmq_xx_reset_rx (&rx->xx);
	    *my_state = NK_DEV_VLINK_RESET;
	    need_sysconf = true;
	}
	break;
    case NK_DEV_VLINK_RESET:
	if (*peer_state != NK_DEV_VLINK_OFF) {
	    *my_state = NK_DEV_VLINK_ON;
	    need_sysconf = true;
	}
	break;
    case NK_DEV_VLINK_ON:
	if (*peer_state == NK_DEV_VLINK_OFF) {
	    vmq_rx_abort (rx, NULL);
	}
	break;
    }
    DTRACE ("exit %d->%d\n", *peer_state, *my_state);
    return need_sysconf;
}

    static void
vmq_rx_vlink_off_completed (vmq_rx* rx)
{
    if (rx->xx.vlink->c_state == NK_DEV_VLINK_OFF) {
	vmq_xx_reset_rx (&rx->xx);
	rx->xx.vlink->s_state = NK_DEV_VLINK_RESET;
	rx->xx.aborted = false;
	DTRACE ("exit %d->%d\n", rx->xx.vlink->c_state, rx->xx.vlink->s_state);
    }
}

    static void
vmq_rx_finish (vmq_rx* rx)
{
    rx->xx.vlink->s_state = NK_DEV_VLINK_OFF;
    vmq_xx_finish (&rx->xx);
}

    static signed
vmq_rx_init (vmq_rx* rx, NkDevVlink* rx_vlink, const vmq_xx_config_t* config,
	     spinlock_t* spinlock)
{
    return vmq_xx_init (&rx->xx, rx_vlink, false /*!tx*/, config, spinlock);
}

/*----- Transmission channel: API -----*/

struct vmq_link_t {
    vmq_link_public_t		public;		/* Must be first */
    struct list_head		link;
    vmq_tx			tx;
    vmq_rx			rx;
    struct vmq_links_t*		links;
    const vmq_callbacks_t*	callbacks;
};

    static void
vmq_tx_notify (vmq_tx* tx)
{
    vmq_link_t* link = container_of (tx, vmq_link_t, tx);

    if (link->callbacks->return_notify) {
	link->callbacks->return_notify (link);
    } else {
	void* msg;

	while (!vmq_xx_return_msg_receive (&tx->xx, &msg, tx->ss_heads)) {
	    vmq_tx_return_msg_free (tx, msg);
	}
    }
}

    signed
vmq_msg_allocate_ex (vmq_link_t* link, unsigned data_len, void** msg,
		     unsigned* data_offset, _Bool nonblocking)
{
    return vmq_tx_msg_allocate (&link->tx, data_len, msg, data_offset,
				nonblocking);
}

    void
vmq_msg_send (vmq_link_t* link, void* msg)
{
    vmq_xx_msg_send (&link->tx.xx, msg);
}

    void
vmq_data_free (vmq_link_t* link, unsigned data_offset)
{
    vmq_tx_data_free (&link->tx, data_offset);
}

    signed
vmq_return_msg_receive (vmq_link_t* link, void** msg)
{
    return vmq_xx_return_msg_receive (&link->tx.xx, msg, link->tx.ss_heads);
}

    void
vmq_return_msg_free (vmq_link_t* link, void* msg)
{
    vmq_tx_return_msg_free (&link->tx, msg);
}

/*----- Reception channel: API -----*/

    static void
vmq_rx_notify (vmq_rx* rx)
{
    vmq_link_t* link = container_of (rx, vmq_link_t, rx);

    if (link->callbacks->receive_notify) {
	link->callbacks->receive_notify (link);
    }
}

    signed
vmq_msg_receive (vmq_link_t* link, void** msg)
{
    return vmq_xx_msg_receive (&link->rx.xx, msg);
}

    void
vmq_msg_free (vmq_link_t* link, void* msg)
{
    vmq_xx_msg_free (&link->rx.xx, msg, false);
}

    void
vmq_msg_return (vmq_link_t* link, void* msg)
{
    vmq_xx_msg_free (&link->rx.xx, msg, true);
}

    _Bool
vmq_data_offset_ok (vmq_link_t* link, unsigned data_offset)
{
    return data_offset < vmq_xx_config_ls_total (&link->rx.xx.config);
}

/*----- Link status changes -----*/

    /* Called from client. VMQ_LOCK not taken */

    static void
vmq_link_off_completed (vmq_link_t* link)
{
	/*
	 *  This routine can be called following
	 *  init time changes in any channel. Do not
	 *  reset other side variables in this case.
	 */
    vmq_rx_vlink_off_completed (&link->rx);
    vmq_tx_vlink_off_completed (&link->tx);
    vmq_sysconf_trigger (link->tx.xx.vlink->s_id);
}

struct vmq_links_t {
    struct list_head		links;
    spinlock_t			spinlock;
    const vmq_callbacks_t*	callbacks;
    NkXIrqId			sysconf_id;
    char*			proc_name;
    struct proc_dir_entry*	proc;
};

    _Bool
vmq_links_iterate (vmq_links_t* links, _Bool (*func)(vmq_link_t*, void*),
		    void* cookie)
{
    vmq_link_t* link;

    list_for_each_entry_type (link, vmq_link_t, &links->links, link) {
	if (func (link, cookie)) return true;
    }
    return false;
}

    /* Only called from vmq_links_abort() */

    static _Bool
vmq_link_abort_tx (vmq_link_t* link, void* cookie)
{
    return vmq_tx_abort (&link->tx, cookie);
}

    void
vmq_links_abort (vmq_links_t* links)
{
    DTRACE ("\n");
    vmq_links_iterate (links, vmq_link_abort_tx, NULL);
}

    static _Bool
vmq_link_on (vmq_link_t* link)
{
    return vmq_xx_vlink_on (&link->rx.xx) &&
	   vmq_xx_vlink_on (&link->tx.xx);
}

    /* BASE: Only called from vmq_links_sysconf() */

    static _Bool
vmq_link_sysconf (vmq_link_t* link, void* cookie)
{
    _Bool		completed = false;
    unsigned long	flags;

    (void) cookie;
    VMQ_LOCK (link->tx.xx.spinlock, flags);
    if (link->tx.xx.aborted || link->rx.xx.aborted) {
	if (vmq_xx_all_returned (&link->tx.xx) &&
	    vmq_xx_all_returned (&link->rx.xx)) {
	    vmq_link_off_completed (link);
	    completed = true;
	}
    }
    VMQ_UNLOCK (link->tx.xx.spinlock, flags);
    if (completed) {
	if (link->callbacks->link_off_completed) {
	    link->callbacks->link_off_completed (link);
	}
    }
    if (vmq_link_on (link)) {
	if (link->callbacks->link_on) {
	    link->callbacks->link_on (link);
	}
    } else {
	if (link->callbacks->link_off) {
	    link->callbacks->link_off (link);
	}
    }
    return false;
}

    /* BASE: Called from client following sysconf_notify() */

    void
vmq_links_sysconf (vmq_links_t* links)
{
    DTRACE ("%p\n", links);
    vmq_links_iterate (links, vmq_link_sysconf, NULL);
}

    /* INTR: Only called from vmq_sysconf_hdl() */

    static _Bool
vmq_link_sysconf_hdl (vmq_link_t* link, void* cookie)
{
    unsigned changed = vmq_rx_handshake (&link->rx);

    (void) cookie;
    changed |= vmq_tx_handshake (&link->tx);
    if (changed) {
	vmq_sysconf_trigger (link->tx.xx.vlink->s_id);
    }
    return false;
}

    static void
vmq_xx_sysconf_notify (vmq_xx* xx)
{
    vmq_link_t* link = container_of (xx, vmq_link_t, tx.xx);

    link->callbacks->sysconf_notify (link->links);
}

    /* xirq handler */

    static void
vmq_sysconf_hdl (void* cookie, NkXIrq xirq)
{
    vmq_links_t* links = cookie;

    (void) xirq;
    DTRACE ("cookie %p xirq %d\n", cookie, xirq);
    vmq_links_iterate (links, vmq_link_sysconf_hdl, NULL);
    links->callbacks->sysconf_notify (links);
    DTRACE ("finished\n");
}

/*----- Support for /proc/vmq.<vlink_name> -----*/

    static int
vmq_proc_xx (char* buf, const char* name, const vmq_xx* xx)
{
    return sprintf (buf, "%s: %6x %4x %6x %4x %2d %2d %2d\n", name,
		    xx->config.msg_count,  xx->config.msg_max,
		    xx->config.data_count, xx->config.data_max,
		    xx->aborted, xx->ss_checked_out, xx->ls_checked_out);
}

    static int
vmq_read_proc (char* page, char** start, off_t off, int count, int* eof,
	       void* data)
{
    vmq_links_t* links = data;
    off_t	begin = 0;
    int		len = 0;
    vmq_link_t* link;

    list_for_each_entry_type (link, vmq_link_t, &links->links, link) {
	len += sprintf (page+len, "     Loc Rem DataMax MsgMax Info\n");
	len += sprintf (page+len, "Pub: %3d %3d %7x %6x %s\n",
			link->public.local_osid, link->public.peer_osid,
			link->public.data_max, link->public.msg_max,
			link->public.rx_s_info ? link->public.rx_s_info : "");
	len += sprintf (page+len, "    MCount MMax DCount DMax Ab MO DO\n");
	len += vmq_proc_xx (page+len, "TX", &link->tx.xx);
	len += vmq_proc_xx (page+len, "RX", &link->rx.xx);

	if (len + begin > off + count)
	    goto done;
	if (len + begin < off) {
	    begin += len;
	    len = 0;
	}
    }
    *eof = 1;

done:
    if (off >= len+begin) return 0;
    *start = page + off - begin;
    return (count < begin + len - off ? count : begin + len - off);
}

/*----- Initialization and shutdown code -----*/

    /* Only called by vmq_init_links() */

    static NkDevVlink*
vmq_find_pair_vlink (NkDevVlink* l, const char* vlink_name)
{
    NkPhAddr    plink = 0;
    NkDevVlink* vlink;

    DTRACE ("\n");
    while ((plink = nkops.nk_vlink_lookup (vlink_name, plink)) != 0) {
	vlink = nkops.nk_ptov (plink);

	if ((vlink != l) &&
	    (vlink->s_id == l->c_id) &&
	    (vlink->c_id == l->s_id) &&
	    (vlink->link == l->link)) {
	    return vlink;
	}
    }
    return NULL;
}

    /* Only called by vmq_vlink_in_use() */

    static _Bool
vmq_link_match (vmq_link_t* link, void* cookie)
{
    return link->rx.xx.vlink->link == *(int*) cookie;
}

    /* Only called by vmq_init_links() */

    static _Bool
vmq_vlink_in_use (vmq_links_t* links, NkDevVlink* vlink)
{
    DTRACE ("\n");
    return vmq_links_iterate (links, vmq_link_match, &vlink->link);
}

    /* Only called by vmq_init_links() */

    static signed
vmq_link_create (vmq_links_t* links, NkDevVlink* rx_vlink,
		 NkDevVlink* tx_vlink, const vmq_xx_config_t* tx_config,
		 const vmq_xx_config_t* rx_config)
{
    vmq_link_t*		link;
    vmq_xx_config_t	local_tx_config;
    signed		diag;

    DTRACE ("\n");
    link = kzalloc (sizeof *link, GFP_KERNEL);
    if (!link) {
	ETRACE ("Could not allocate memory for vmq link.\n");
	return -ENOMEM;
    }
    if (rx_vlink->s_info) {
	link->public.rx_s_info = (char*) nkops.nk_ptov (rx_vlink->s_info);
	DTRACE ("rx_s_info '%s'\n", link->public.rx_s_info);
    }
    if (!tx_config) {
	if (!links->callbacks->get_tx_config) return -EINVAL;
	diag = links->callbacks->get_tx_config (links, link->public.rx_s_info,
						&local_tx_config);
	if (diag) {
	    kfree (link);
	    return diag;
	}
	tx_config = &local_tx_config;
    }
    diag = vmq_tx_init (&link->tx, tx_vlink, tx_config, &links->spinlock);
    if (diag) {		/* Error message already issued */
	kfree (link);
	return diag;
    }
    diag = vmq_rx_init (&link->rx, rx_vlink, rx_config, &links->spinlock);
    if (diag) {		/* Error message already issued */
	vmq_tx_finish (&link->tx);
	kfree (link);
	return diag;
    }
	/*
	 * vdev=(...,linkid|....s_info...)
	 */
    link->public.local_osid   = link->rx.xx.vlink->s_id;
    link->public.peer_osid    = link->tx.xx.vlink->s_id;
    link->public.rx_data_area = link->rx.xx.ls_area;
    link->public.tx_data_area = link->tx.xx.ls_area;
    link->public.data_max     = link->tx.xx.config.data_max;
    link->public.msg_max      = link->tx.xx.config.msg_max;
    link->links               = links;
    link->callbacks           = links->callbacks;

    list_add (&link->link, &links->links);
    return 0;
}

    /* Only called externally */

    signed
vmq_links_init (vmq_links_t** result, const char* vlink_name,
		const vmq_callbacks_t* callbacks,
		const vmq_xx_config_t* tx_config,
		const vmq_xx_config_t* rx_config)
{
    NkOsId myid = nkops.nk_id_get();
    vmq_links_t* links;
    NkPhAddr plink = 0;
    signed diag;

    DTRACE ("\n");
    *result = NULL;	/* Make sure it is NULL on error */
    if (!callbacks->sysconf_notify) {
	DTRACE ("The sysconf notify callback is mandatory.\n");
	return -EINVAL;
    }
    links = kzalloc (sizeof *links, GFP_KERNEL);
    if (!links) {
	ETRACE ("Could not allocate vmtd_links descriptor.\n");
	diag = -ENOMEM;
	goto error;
    }
    INIT_LIST_HEAD (&links->links);
    links->spinlock = SPIN_LOCK_UNLOCKED;
    links->callbacks = callbacks;
    while ((plink = nkops.nk_vlink_lookup (vlink_name, plink)) != 0) {
	NkDevVlink* vlink = nkops.nk_ptov (plink);

	if (vlink->s_id == myid && !vmq_vlink_in_use (links, vlink)) {
	    NkDevVlink* rx_vlink;
	    NkDevVlink* tx_vlink;

	    rx_vlink = vlink;
	    tx_vlink = vmq_find_pair_vlink (rx_vlink, vlink_name);
	    if (tx_vlink) {
		diag = vmq_link_create (links, rx_vlink, tx_vlink,
					tx_config, rx_config);
		if (diag) {	/* Error message already issued */
		    goto error;
		}
	    }
	}
    }
	/*
	 *  Set result already now, in case a callback
	 *  is immediately called.
	 */
    *result = links;
    links->sysconf_id = nkops.nk_xirq_attach (NK_XIRQ_SYSCONF,
					      vmq_sysconf_hdl, links);
    if (!links->sysconf_id) {
	ETRACE ("Cannot attach sysconf handler\n");
	*result = NULL;
	diag = -EAGAIN;
	goto error;
    }
    {
	const size_t len = sizeof "vmq." + strlen (vlink_name);

	links->proc_name = kmalloc (len, GFP_KERNEL);
	if (links->proc_name) {
	    snprintf (links->proc_name, len, "vmq.%s", vlink_name);
	    links->proc = create_proc_read_entry (links->proc_name, 0, NULL,
						  vmq_read_proc, links);
	}
    }
	/*
	 *  Trigger a sysconf to ourselves rather than simply
	 *  calling "vmq_sysconf_hdl(links,0);" because this
	 *  way we get the proper environment in the handler
	 *  and avoid concurrency with the real handler.
	 */
    vmq_sysconf_trigger (myid);
    return 0;

error:
    vmq_links_finish (links);
    return diag;
}

    /* Only called by vmq_links_finish() */

    static _Bool
vmq_link_destroy (vmq_link_t* link, void* cookie)
{
    (void) cookie;
    vmq_rx_finish (&link->rx);
    vmq_tx_finish (&link->tx);
    vmq_sysconf_trigger (link->tx.xx.vlink->s_id);
    list_del (&link->link);
    kfree (link);
    return true;	/* Abort list scanning */
}

    /* Only called externally */

    void
vmq_links_finish (vmq_links_t* links)
{
    DTRACE ("\n");
    if (links) {
	if (links->proc && links->proc_name) {
	    remove_proc_entry (links->proc_name, NULL);
	}
	kfree (links->proc_name);
	if (links->sysconf_id) {
	    nkops.nk_xirq_detach (links->sysconf_id);
	}
	while (vmq_links_iterate (links, vmq_link_destroy, NULL));
	kfree (links);
    }
}

/*----- Module description -----*/

EXPORT_SYMBOL (vmq_data_free);
EXPORT_SYMBOL (vmq_data_offset_ok);
EXPORT_SYMBOL (vmq_links_abort);
EXPORT_SYMBOL (vmq_links_finish);
EXPORT_SYMBOL (vmq_links_init);
EXPORT_SYMBOL (vmq_links_iterate);
EXPORT_SYMBOL (vmq_links_sysconf);
EXPORT_SYMBOL (vmq_msg_allocate_ex);
EXPORT_SYMBOL (vmq_msg_free);
EXPORT_SYMBOL (vmq_msg_receive);
EXPORT_SYMBOL (vmq_msg_return);
EXPORT_SYMBOL (vmq_msg_send);
EXPORT_SYMBOL (vmq_return_msg_free);
EXPORT_SYMBOL (vmq_return_msg_receive);

#ifdef MODULE
MODULE_LICENSE ("GPL");
#else
MODULE_LICENSE ("Proprietary");
#endif
MODULE_AUTHOR ("Adam Mirowski <adam.mirowski@virtuallogix.com>");
MODULE_DESCRIPTION ("VLX Virtual VMQ communications driver");

/*----- End of file -----*/

