/*
 ****************************************************************
 *
 *  Component:	VirtualLogix driver services
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski <adam.mirowski@virtuallogix.com>
 *
 ****************************************************************
 */

#ifdef VLX_SERVICES_THREADS

    /*
     *  On kernels starting with 2.6.7, we use the kthread_run() API
     *  instead of kernel_thread(), except if the thread is supposed
     *  to suicide, which cannot be implemented using kthreads.
     *  It would be simpler to just use kernel_thread() all the
     *  time...
     */

typedef struct {
    int			(*func) (void*);
    void*		arg;
    const char*		name;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    _Bool		is_kernel_thread;
#endif
    union {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
	struct {
	    struct task_struct*	desc;
	} kthread;
#endif
	struct {
	    pid_t		pid;
	    struct completion	completion;
	} kernel_thread;
    } u;
} vlx_thread_t;

    static int
vlx_thread_entry (void* arg)
{
    vlx_thread_t*	thread = arg;
    int			diag;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    if (thread->is_kernel_thread) {
	daemonize (thread->name);
    }
#else
    daemonize (thread->name);
#endif
#else
    daemonize();
    snprintf (current->comm, sizeof current->comm, thread->name);
#endif

    diag = thread->func (thread->arg);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
	/*
	 *  The thread function has finished before kthread_stop() has
	 *  been called. If we exited here immediately, struct task_struct
	 *  "desc" would be deallocated before vlx_thread_join() has used it
	 *  to call kthread_stop(). Therefore, we must await this call here.
	 */
    if (!thread->is_kernel_thread) {
	while (!kthread_should_stop()) {
	    set_current_state (TASK_INTERRUPTIBLE);
	    schedule_timeout (1);
	}
	return diag;
    }
#endif
    complete_and_exit (&thread->u.kernel_thread.completion, diag);
    /*NOTREACHED*/
    return diag;
}

    static void
vlx_thread_join (vlx_thread_t* thread)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    if (!thread->is_kernel_thread) {
	if (thread->u.kthread.desc) {
	    kthread_stop (thread->u.kthread.desc);
	}
	return;
    }
#endif
    if (thread->u.kernel_thread.pid > 0) {
	    /* On 2.4.20, it is a void function */
	wait_for_completion (&thread->u.kernel_thread.completion);
    }
}

    static int
vlx_thread_start_ex (vlx_thread_t* thread, int (*func) (void*), void* arg,
		     const char* name, _Bool will_suicide)
{
    thread->func = func;
    thread->arg  = arg;
    thread->name = name;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    thread->is_kernel_thread = will_suicide;
    if (!will_suicide) {
	thread->u.kthread.desc = kthread_run (vlx_thread_entry, thread, name);
	if (IS_ERR (thread->u.kthread.desc)) {
	    return PTR_ERR (thread->u.kthread.desc);
	}
	return 0;
    }
#endif
    init_completion (&thread->u.kernel_thread.completion);
    thread->u.kernel_thread.pid = kernel_thread (vlx_thread_entry, thread, 0);
    if (thread->u.kernel_thread.pid < 0) {
	return thread->u.kernel_thread.pid;
    }
    if (!thread->u.kernel_thread.pid) {
	return -EAGAIN;
    }
    return 0;
}

    static inline int
vlx_thread_start (vlx_thread_t* thread, int (*func) (void*), void* arg,
		  const char* name)
{
    return vlx_thread_start_ex (thread, func, arg, name, 0 /*!will_suicide*/);
}

#endif	/* VLX_SERVICES_THREADS */

#ifdef VLX_SERVICES_PROC_NK

    static int
vlx_proc_dir_match (struct proc_dir_entry* dir, const char* name)
{
    const unsigned namelen = strlen (name);

    if (!dir->low_ino) {
	return 0;
    }
    if (dir->namelen != namelen) {
	return 0;
    }
    return !memcmp (name, dir->name, namelen);
}

    static struct proc_dir_entry*
vlx_proc_dir_lookup (struct proc_dir_entry* dir, const char* name)
{
    while (dir && !vlx_proc_dir_match (dir, name)) {
	dir = dir->next;
    }
    return dir;
}

    /*
     *  Starting with kernel 2.6.27, proc_root is no more exported
     *  and no more present in proc_fs.h, but the VLX-specific kernel
     *  still offers it.
     */
extern struct proc_dir_entry proc_root;

    static struct proc_dir_entry*
vlx_proc_nk_lookup (void)
{
    return vlx_proc_dir_lookup (proc_root.subdir, "nk");
}

#endif	/* VLX_SERVICES_PROC_NK */

