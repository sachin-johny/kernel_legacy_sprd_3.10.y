/*
 ****************************************************************
 *
 *  Copyright (C) 2002-2009, VirtualLogix. All Rights Reserved.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Contributor(s):
 *	Vladimir Grouzdev <vladimir.grouzdev@vlx.com>
 *
 ****************************************************************
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/bug.h>
#include <linux/clockchips.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <nk/nkern.h>

#include <asm/mach-types.h>
#include <asm/mach/time.h>
#include <asm/div64.h>
#include <asm/nkern.h>

    extern void
secondary_startup (void);

    extern void
nk_ddi_cpu_init (void);

    irqreturn_t
nk_do_IPI (int xirq, void* dev_id);

extern void vector_und(void);
extern void vector_swi(void);
extern void vector_pabt(void);
extern void vector_dabt(void);
extern void vector_fiq(void);
extern void vector_irq(void);

static NkOsId           _myid;
static NkXIrq           _ipi_base;
static struct irqaction _ipi_irq[NR_CPUS];

#define	IPI_XIRQ(cpu)	(_ipi_base + (cpu)) // one IPI per vCPU

#define	NK_ETRACE(f...)	   do { printnk(KERN_ERR  f); } while (0)
#define	NK_DTRACE(f...)	   /* do { printnk(KERN_INFO f); } while (0) */

#ifdef CONFIG_LOCAL_TIMERS

    extern irqreturn_t
nk_do_local_timer (int irq, void* dev_id);

typedef struct lvtimer_t {	// local virtual timer
    void*            id;
    struct irqaction action;
} lvtimer_t;

static lvtimer_t _lvtimers[NR_CPUS];	 // one LVT per vCPU
static NkXIrq    _lvt_base;

#define	LVTIMER() 	(_lvtimers + VCPU()->vcpuid)
#define	LVT_XIRQ(cpu)	(_lvt_base + (cpu))

    /*
     * Calculate appropriate shift and mult for a clock event
     */
    static inline void
_clockevent_setup (struct clock_event_device* evt, unsigned int clock)
{
    u64 temp;
    u32 shift;

        /*
         * Find a shift value
         */
    for (shift = 32; shift > 0; shift--) {
	temp = (u64) clock << shift;
	do_div(temp, NSEC_PER_SEC);
	if ((temp >> 32) == 0) {
	    break;
	}
    }
    evt->shift = shift;
    evt->mult  = (u32) temp;
}

    /*
     * Handle the different clock event modes
     */
    static void
_local_vtimer_set_mode (enum clock_event_mode mode,
			struct clock_event_device* evt)
{
    lvtimer_t* timer = LVTIMER();

    switch(mode) {
    case CLOCK_EVT_MODE_PERIODIC:
	NK_DTRACE("LOCAL TIMER: PERIODIC mode\n");
	VCPU()->smp_timer_start_periodic(timer->id, HZ);
	break;
	
    case CLOCK_EVT_MODE_SHUTDOWN:
    case CLOCK_EVT_MODE_UNUSED:
	NK_DTRACE("LOCAL TIMER: SHUTDOWN mode\n");
	if (evt->mode == CLOCK_EVT_MODE_PERIODIC ||
	    evt->mode == CLOCK_EVT_MODE_ONESHOT) {
	    VCPU()->smp_timer_stop(timer->id);
	}
	break;
	
    case CLOCK_EVT_MODE_ONESHOT:
	NK_DTRACE("LOCAL TIMER: ONESHOT mode\n");
	if (evt->mode == CLOCK_EVT_MODE_PERIODIC) {
	    VCPU()->smp_timer_stop(timer->id);
	}
	break;
	
    case CLOCK_EVT_MODE_RESUME:
	    /*
             * Nothing to do here
             */
	break;
    }
}

    /*
     * Program the next event
     */
    static int
_local_vtimer_set_next_event (unsigned long delta,
	 	 	      struct clock_event_device* evt)
{
    lvtimer_t* timer = LVTIMER();
    VCPU()->smp_timer_start_oneshot(timer->id, delta);
    return 0;
}

    /*
     * Setup the local clock events for a CPU.
     */
    void __cpuinit
local_timer_setup (struct clock_event_device* evt)
{
    int         cpu   = VCPU()->vcpuid;
    lvtimer_t*  timer = LVTIMER();  
    NkXIrq      xirq  = LVT_XIRQ(cpu);
    void*       id    = timer->id;
    int         boot  = (id == 0); // boot or resume from memory
    NkTimerInfo info;

    if (boot) {
        id = VCPU()->smp_timer_alloc(xirq, NK_TIMER_FLAGS_LOCAL |
				           NK_TIMER_FLAGS_PERIODIC |
				           NK_TIMER_FLAGS_ONESHOT);
        if (!id) {
	    NK_ETRACE("VLX: Unable to allocate NK timer\n");
	    BUG();
        }
        timer->id = id;
    }

    VCPU()->smp_timer_info(id, nkops.nk_vtop(&info));

    evt->name	        = "local vtimer";
    evt->rating         = 350;
    evt->set_mode       = _local_vtimer_set_mode;
    evt->set_next_event = _local_vtimer_set_next_event;
    evt->irq            = xirq;

    evt->features = 0;
    if (info.flags & NK_TIMER_FLAGS_PERIODIC) {
       evt->features |=  CLOCK_EVT_FEAT_PERIODIC;
    }
    if (info.flags & NK_TIMER_FLAGS_ONESHOT) {
       evt->features |=  CLOCK_EVT_FEAT_ONESHOT;
    }

    _clockevent_setup(evt, info.freq_hz);

    NK_DTRACE("LOCAL TIMER: clockevent shift=%d mult=%lu irq=%d\n",
	      evt->shift, evt->mult, xirq);

    evt->max_delta_ns =	clockevent_delta2ns(info.max_delta, evt);
    evt->min_delta_ns =	clockevent_delta2ns(info.min_delta, evt);

    if (boot) {
        timer->action.name    = "local vtimer";
        timer->action.flags   = IRQF_DISABLED | IRQF_NOBALANCING | IRQF_TIMER;
        timer->action.handler = nk_do_local_timer;
        timer->action.dev_id  = evt;

        if (setup_irq(xirq, &timer->action)) {
	    NK_ETRACE("VLX: Unable to setup local timer IRQ %d\n", xirq);
	    BUG();
        }

        nkops.nk_xirq_affinity(xirq, (1 << cpu));
    }

    clockevents_register_device(evt);
}

    void
local_timer_stop (void)
{
    lvtimer_t* timer = LVTIMER();  
    NK_DTRACE("VLX: Stop local timer %d\n", VCPU()->vcpuid);
    VCPU()->smp_timer_stop(timer->id);
}

#endif

volatile int __cpuinitdata pen_release = -1;

    void __cpuinit
platform_secondary_init (unsigned int cpu)
{
    NkOsCtx* vcpu = VCPU();

    nk_ddi_cpu_init();

    vcpu->os_vectors[NK_UNDEF_INSTR_VECTOR/4]    = vector_und;
    vcpu->os_vectors[NK_SYSTEM_CALL_VECTOR/4]    = vector_swi;
    vcpu->os_vectors[NK_PREFETCH_ABORT_VECTOR/4] = vector_pabt;
    vcpu->os_vectors[NK_DATA_ABORT_VECTOR/4]     = vector_dabt;
    vcpu->os_vectors[NK_FIQ_VECTOR/4]            = vector_fiq;
    vcpu->os_vectors[NK_IIRQ_VECTOR/4]           = vector_irq;
    vcpu->os_vectors[NK_XIRQ_VECTOR/4]           = vector_irq;

    vcpu->ready(vcpu);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
    pen_release = -1;
}

    int __cpuinit
boot_secondary (unsigned int cpu, struct task_struct* idle)
{
    unsigned long timeout;

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
    pen_release = cpu;

    VCPU()->smp_cpu_start(cpu, virt_to_phys(secondary_startup));

    timeout = jiffies + (3 * HZ);
    while (time_before(jiffies, timeout)) {
	smp_rmb();
	if (pen_release == -1) {
		break;
	}
	udelay(10);
    }

    return (pen_release != -1 ? -ENOSYS : 0);
}

    void __init
smp_init_cpus (void)
{
    unsigned int cpu;
    unsigned int ncores = VCPU()->maxvcpus;

    if (!ncores) {
	ncores = 1;
    }

    for (cpu = 0; cpu < ncores; cpu++) {
	set_cpu_possible(cpu, true);
    }
}

    static void __init
_smp_setup_local_irqs (unsigned int maxcpus)
{
    unsigned int cpu;

    _myid = VCPU()->id;

    _ipi_base = VCPU()->smp_dxirq_alloc(maxcpus);
    NK_DTRACE("IPI IRQs base: %d\n", _ipi_base);
    BUG_ON(!_ipi_base);

#ifdef CONFIG_LOCAL_TIMERS
    _lvt_base = VCPU()->smp_dxirq_alloc(maxcpus);
    NK_DTRACE("LVT IRQs base: %d\n", _lvt_base);
    BUG_ON(!_lvt_base);
#endif

    for (cpu = 0; cpu < maxcpus; cpu++) {
	NkXIrq            xirq   = IPI_XIRQ(cpu);
	struct irqaction* action = _ipi_irq + cpu;

	action->name    = "IPI";
	action->flags   = IRQF_DISABLED | IRQF_NOBALANCING | IRQF_TIMER;
	action->handler = nk_do_IPI;

	if (setup_irq(xirq, action)) {
	    NK_ETRACE("VLX: Unable to setup IPI IRQ %d\n", xirq);
	    BUG();
	}

	nkops.nk_xirq_affinity(xirq, (1 << cpu));
    }
}

    void __init
smp_prepare_cpus (unsigned int maxcpus)
{
    unsigned int cpu;
    unsigned int ncores = VCPU()->maxvcpus;

    if (!ncores) {
	ncores = 1;
    }
    if (ncores > NR_CPUS) {
	ncores = NR_CPUS;
    }
    if (maxcpus > ncores) {
	maxcpus = ncores;
    }

    _smp_setup_local_irqs(maxcpus);

    smp_store_cpu_info(smp_processor_id());

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
    for (cpu = 0; cpu < maxcpus; cpu++) {
	set_cpu_present(cpu, true);
    }
}

    void
smp_cross_call (const struct cpumask* mask)
{
    unsigned int cpu;
    for_each_cpu(cpu, mask) {
	VCPU()->smp_irq_post(IPI_XIRQ(cpu), _myid);
    }
}

