
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/kernel.h>

#include <nk/nkern.h>

#define XIRQB_TRACE(h...)	printk(h)
#if 0
#define XIRQB_DEBUG(h...)	printk(h)
#else
#define XIRQB_DEBUG(h...)
#endif
#define XIRQB_ERR(h...)		printk(h)

typedef struct xirq_sample {
	NkTime	start;		/* start timer */
	NkTime	t1;		/* intermediate time in the handler of backend*/
	NkTime	t2;		/* intermediate time in the handler of fronend*/
	NkTime	stop;		/* time a the arrival*/
} xirq_sample_t;


typedef struct xirq_bench_desc xirq_bench_desc_t;
struct xirq_bench_desc {
	NkDevVlink*      	vlink;		/* point to vlink          */
	xirq_sample_t* 	 	sample;		/* point to data 	   */
	int			count;		/* number of cross-irq sent*/
	NkXIrqId         	xirq_desc;	/* descriptor 		   */
	NkXIrqId         	sysconf_desc;	/* descriptor 		   */
	NkXIrq           	xirq;		/* target xirq 		   */
	NkOsId		 	osid;		/* target os 		   */
	struct semaphore 	sem;		/* semaphore used to sleep */
	xirq_sample_t    	min;	        /* min time sample         */
	xirq_sample_t    	max;            /* max time sample         */
	NkTime			avg1;           /* temporary value         */
	NkTime			avg2;
	NkTime			avg3;
	NkTime			avg4;
	xirq_bench_desc_t*	next;           /* all descritpro are linked*/
	struct proc_dir_entry*	proc_entry;
};

static xirq_bench_desc_t* head; /* head of the list of bench descriptor */

DECLARE_MUTEX(xirq_bench_lock); /* lock the driver */

static void xirq_fe_handler(void* cookie, NkXIrq xirq)
{
	xirq_bench_desc_t* d = (xirq_bench_desc_t*) cookie;
	d->sample->t2 = os_ctx->smp_time();
	up(&d->sem);
}

static void xirq_be_handler(void* cookie, NkXIrq xirq)
{
	xirq_bench_desc_t* d = (xirq_bench_desc_t*) cookie;
	d->sample->t1 = os_ctx->smp_time();
	nkops.nk_xirq_trigger(d->xirq, d->osid);
}

static void _sysconf_trigger(xirq_bench_desc_t* desc)
{
	nkops.nk_xirq_trigger(NK_XIRQ_SYSCONF, desc->osid);
}

static void _reset(xirq_bench_desc_t* desc)
{
	/*
	 * I should probably remove entry in /proc -- but this requires to
	 * do this within a thread
	 */
}

static void _init(xirq_bench_desc_t* desc)
{
	/*
	 * I should probably create entry in /proc -- but this requires to
	 * do this within a thread
	 */
}

static int xirq_bench_handshake(xirq_bench_desc_t* desc)
{
	volatile int* my_state;
	int           peer_state;

	if (desc->vlink->s_id == nkops.nk_id_get()) {
	    my_state   = &(desc->vlink->s_state);
	    peer_state =  desc->vlink->c_state;
	} else {
	    my_state   = &(desc->vlink->c_state);
	    peer_state =  desc->vlink->s_state;
	}

	XIRQB_DEBUG("%s: (link:%d, fe:%d) myself:%d peer:%d\n", __func__,
		    desc->vlink->link, desc->vlink->c_id,
		    *my_state, peer_state);

	switch (*my_state) {
		case NK_DEV_VLINK_OFF:
			if (peer_state != NK_DEV_VLINK_ON) {
				_reset(desc);
				*my_state = NK_DEV_VLINK_RESET;
				_sysconf_trigger(desc);
			}
			break;
		case NK_DEV_VLINK_RESET:
			if (peer_state != NK_DEV_VLINK_OFF) {
				*my_state = NK_DEV_VLINK_ON;
				_init(desc);
				_sysconf_trigger(desc);
			}
			break;
		case NK_DEV_VLINK_ON:
			if (peer_state == NK_DEV_VLINK_OFF) {
				*my_state = NK_DEV_VLINK_OFF;
				_sysconf_trigger(desc);
			}
			break;
	}

	return (*my_state  == NK_DEV_VLINK_ON) &&
		(peer_state == NK_DEV_VLINK_ON);
}

static void _sysconf_handler(void* cookie, NkXIrq xirq)
{
	xirq_bench_handshake((xirq_bench_desc_t*) cookie);
}

static inline unsigned long long xirqb_div64_32 (unsigned long long num,
						 unsigned long div)
{
    do_div (num, div);
    return num;
}

static int _proc_read(char *pages, char **start, off_t off,
		      int count, int *eof, void *data)
{
	int			len = 0;
	off_t			begin = 0;
	int			res = 0;
	const int		S = 1000000;
	int			hz;

	xirq_bench_desc_t*	d = (void*) data;
	xirq_sample_t*		t;

	/* if no benchmark has been done */
	if (d->count == 0) {
	    return 0;
	}

	hz = os_ctx->smp_time_hz();

	down(&xirq_bench_lock);

	len += sprintf(pages + len, "CROSS-IRQ BENCHMARK RESULTS:\n");
	len += sprintf(pages + len,
		"  Send %x cross-irq from %4d to %4d clock frequency %10d\n", 
		       d->count , d->vlink->c_id, 
		       d->vlink->s_id, hz);

	len += sprintf(pages + len, "TIME In tick: %-8s %-8s %-8s %-8s\n",
		       "FE->BE", "BE->FE-I", "I->Thrd", "ALL");

	/* display time for the best case */
	t = &d->min;
	len += sprintf(pages + len,
		       "  min       : %-8llu %-8llu %-8llu %-8llu\n",
		       t->t1   - t->start,
		       t->t2   - t->t1,
		       t->stop - t->t2,
		       t->stop - t->start);

	/* display time for the worst case */
	t = &d->max;
	len += sprintf(pages + len,
		       "  max       : %-8llu %-8llu %-8llu %-8llu\n",
		       t->t1   - t->start,
		       t->t2   - t->t1,
		       t->stop - t->t2,
		       t->stop - t->start);

	/* display average times  */
	len += sprintf(pages + len,
		       "  avg       : %-8llu %-8llu %-8llu %-8llu\n",
		       xirqb_div64_32 (d->avg1, d->count),
		       xirqb_div64_32 (d->avg2, d->count),
		       xirqb_div64_32 (d->avg3, d->count),
		       xirqb_div64_32 (d->avg4, d->count));

	len += sprintf(pages + len, "TIME In  us : %-8s %-8s %-8s %-8s\n",
		       "FE->BE", "BE->FE-I", "I->Thrd", "ALL");

#define TICKS_TO_US(x)	xirqb_div64_32 ((x) * S, hz)

	/* display time for the best case */
	t = &d->min;
	len += sprintf(pages + len,
		       "  min       : %-8llu %-8llu %-8llu %-8llu\n",
		       TICKS_TO_US (t->t1   - t->start),
		       TICKS_TO_US (t->t2   - t->t1),
		       TICKS_TO_US (t->stop - t->t2),
		       TICKS_TO_US (t->stop - t->start));

	/* display time for the worst case */
	t = &d->max;
	len += sprintf(pages + len,
		       "  max       : %-8llu %-8llu %-8llu %-8llu\n",
		       TICKS_TO_US (t->t1 -   t->start),
		       TICKS_TO_US (t->t2 -   t->t1),
		       TICKS_TO_US (t->stop - t->t2),
		       TICKS_TO_US (t->stop - t->start));

	/* display average times  */
	len += sprintf(pages + len,
		       "  avg       : %-8llu %-8llu %-8llu %-8llu\n",
		       TICKS_TO_US (xirqb_div64_32 (d->avg1, d->count)),
		       TICKS_TO_US (xirqb_div64_32 (d->avg2, d->count)),
		       TICKS_TO_US (xirqb_div64_32 (d->avg3, d->count)),
		       TICKS_TO_US (xirqb_div64_32 (d->avg4, d->count)));


	if (len+begin < off) {
		begin += len;
		len = 0;
	}

	*eof = 1;

	if (off >= len+begin) {
		res = 0;
		goto out;
	}

	*start = pages + (off-begin);
	res = ((count < begin+len-off) ? count : begin+len-off);

out:
	up(&xirq_bench_lock);

	return res;
}

#define MIN(A,B) ((A) < (B) ? (A) : (B))

static int _proc_write(struct file* file, const char __user *buf,
		       unsigned long count, void* data)
{
	xirq_bench_desc_t*	d = (void*) data;
	char			buffer[64];
	int			ret = -EFAULT;
	int			i;
	char*			last;

	down(&xirq_bench_lock);

	if (copy_from_user(buffer, buf, MIN(count, sizeof(buffer)))) 
		goto out;

	/* check remote hand is ready */
	if (d->vlink->s_state != NK_DEV_VLINK_ON  || 
	    d->vlink->c_state != NK_DEV_VLINK_ON )  {
		ret = -EINVAL;
		goto out;
	}


	d->min.start = 0;
	d->min.t1 = 0;
	d->min.t2 = 0;
	d->min.stop = -1;
	d->max.start = 0;
	d->max.t1 = 0;
	d->max.t2 = 0;
	d->max.stop = 0;
	d->avg1 = 0;
	d->avg2 = 0;
	d->avg3 = 0;
	d->avg4 = 0;
	d->count = 0;

	/* compute the number of iteration */
	count = simple_strtoul(buffer, &last, 0); 
	printk("Got count %lu iterations\n", count);

	/* initialize the number of iteration */
	d->count = count;

	/* send count xirq and take various measure */
	for (i = 0; i < count; i++) {
		NkTime	t;
		NkTime	t2;
		xirq_sample_t*	s = d->sample;

		s->t1 = 0;

		/* record start time */
		s->start = os_ctx->smp_time();

		/* send the cross-irq */
		nkops.nk_xirq_trigger(d->xirq, d->osid);

		/* wait to be awaken */
		down(&d->sem);

		/* get end of time */
		s->stop = os_ctx->smp_time();

		/* compute overall time */
		t = s->stop - s->start;

		/* if we get the minimal time -> record it*/
		t2 = d->min.stop - d->min.start;
		if (t2 > t) {
			d->min = *s; 
		}

		t2 = d->max.stop - d->max.start;
		if (t > t2) {
		    	d->max = *s;
		}

		/* overall time to make transition from this guest to the 
		 * remote guest */
		d->avg1 += s->t1 - s->start;

		/* overall time to make transition from remote to here */
		d->avg2 += s->t2 - s->t1;

		/* time to wakeup the thread */
		d->avg3 += s->stop - s->t2;

		/* overall time to make the whole transition */
		d->avg4 += s->stop - s->start;
	}
out:

	up(&xirq_bench_lock);
	return ret;
}



static int __init xirq_bench_init(void)
{
	NkXIrq      xirq_fe = 0;
	NkXIrq      xirq_be = 0;
	NkXIrqId    xirq_desc;
	NkXIrqId    sysconf_desc;
	NkOsId      osid_self = nkops.nk_id_get();
	NkOsId      osid_fe;
	NkOsId      osid_be;
	NkPhAddr    plink = 0;
	NkPhAddr    pa;
	NkDevVlink* vlink;
	xirq_bench_desc_t* desc;

	printk(" ++++++++======= BENCH INIT ======== +++++++++++++++++++\n");

	while ((plink = nkops.nk_vlink_lookup("xirqb", plink))) {

		vlink = nkops.nk_ptov(plink);
		if (vlink->s_id != osid_self && 
		    vlink->c_id != osid_self) {
			continue;
		}
		desc = (xirq_bench_desc_t*) 
			kzalloc(sizeof(xirq_bench_desc_t),  GFP_KERNEL);

		if (desc == 0) {
			XIRQB_ERR("Descriptor allocation failure\n");
			return -ENOMEM;
		}
		desc->vlink = vlink;

		pa = nkops.nk_pdev_alloc(plink, 0, sizeof(xirq_sample_t));
		if (!pa) {
			XIRQB_ERR("cannot allocate sample memory\n");
			return -ENOMEM;
		}

		desc->sample = (xirq_sample_t*) nkops.nk_ptov(pa);
		if (desc->sample == 0) {
			XIRQB_ERR("ptov failure\n");
			return -ENOMEM;
		}
		sema_init(&desc->sem, 0);

		osid_fe =  vlink->c_id;
		osid_be =  vlink->s_id;

		xirq_be = nkops.nk_pxirq_alloc(plink, 0, osid_be, 1);
		xirq_fe = nkops.nk_pxirq_alloc(plink, 1, osid_fe, 1);

		if (vlink->s_id == osid_self) {
			desc->osid = osid_fe;
			desc->xirq = xirq_fe; 
			xirq_desc = nkops.nk_xirq_attach(xirq_be,
							 xirq_be_handler,
							 desc);

		} else {
			desc->osid = osid_be;
			desc->xirq = xirq_be; 
			xirq_desc = nkops.nk_xirq_attach(xirq_fe,
							 xirq_fe_handler,
							 desc);
		}

		sysconf_desc = nkops.nk_xirq_attach(NK_XIRQ_SYSCONF,
						    _sysconf_handler,
						    desc);

		if (!xirq_desc) {
			XIRQB_ERR("xirqb_be_create: cannot attach BE xirq\n");
			return -ENOMEM;	
		}

		if (!sysconf_desc) {
			nkops.nk_xirq_detach(xirq_desc);
			XIRQB_ERR("xirqb_be_create: cannot attach SYSCONF "
				  "xirq\n");
			return -ENOMEM;	
		}

		desc->sysconf_desc = sysconf_desc;
		desc->xirq_desc    = xirq_desc;

		if (vlink->s_id != osid_self) {
			char	name[32];
			struct proc_dir_entry *res;
			struct proc_dir_entry *dir;

			dir = proc_mkdir("nk/bench", 0);
			if (dir == 0) {
			    XIRQB_TRACE("nk/bench already registered\n");
			} else {
			    printk("bench dir created\n");
			}

			snprintf(name, 31, "nk/bench/%d", vlink->s_id);
			desc->proc_entry = res = 
				create_proc_entry(name, S_IRWXU, 0);
			if (res) {
			    printk("%s created\n", name);
				res->read_proc  = _proc_read;
				res->write_proc = _proc_write;
				res->data	= desc;
			} else {
				XIRQB_ERR("Could not allocate proc "
					  "entry for front-end\n");
			}
		}

		desc->next = head;
		head = desc;
		xirq_bench_handshake(desc);
	}

	XIRQB_TRACE("xirqb devices created\n");
	return 0;
}


static void xirq_bench_exit(void)
{
	xirq_bench_desc_t*	desc;
	down(&xirq_bench_lock);
	while ( (desc = head) != NULL ) {
		head = desc->next;

		if (desc->xirq_desc) {
			nkops.nk_xirq_detach(desc->xirq_desc);
		}

		if (desc->sysconf_desc) {
			nkops.nk_xirq_detach(desc->sysconf_desc);
		}

		if (desc->vlink->s_id != nkops.nk_id_get()) {
			char	buffer[32];
			snprintf(buffer, 31, "nk/bench/%d", desc->vlink->s_id);
			remove_proc_entry(buffer, 0);
		}
		kfree(desc);
	}
	up(&xirq_bench_lock);

}

module_init(xirq_bench_init);
module_exit(xirq_bench_exit);
