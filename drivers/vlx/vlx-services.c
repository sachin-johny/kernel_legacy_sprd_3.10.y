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

typedef struct {
    int			(*func) (void*);
    void*		arg;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    struct task_struct*	desc;
#else
    const char*		name;
    pid_t		pid;
    struct completion	completion;
#endif
} vlx_thread_t;

    static int
vlx_thread_entry (void* arg)
{
    vlx_thread_t*	thread = arg;
    int			diag;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
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
    while (!kthread_should_stop()) {
	set_current_state (TASK_INTERRUPTIBLE);
	schedule_timeout (1);
    }
#else
    complete_and_exit (&thread->completion, diag);
    /*NOTREACHED*/
#endif
    return diag;
}

    static void
vlx_thread_join (vlx_thread_t* thread)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    if (thread->desc) {
	kthread_stop (thread->desc);
    }
#else
    if (thread->pid > 0) {
	    /* On 2.4.20, it is a void function */
	wait_for_completion (&thread->completion);
    }
#endif
}

    static int
vlx_thread_start (vlx_thread_t* thread, int (*func) (void*), void* arg,
		  const char* name)
{
    thread->func = func;
    thread->arg  = arg;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    thread->desc = kthread_run (vlx_thread_entry, thread, name);
    if (IS_ERR (thread->desc)) {
	return PTR_ERR (thread->desc);
    }
#else
    thread->name = name;
    thread->pid = kernel_thread (vlx_thread_entry, thread, 0);
    if (thread->pid <= 0) {
	return thread->pid ? thread->pid : -EAGAIN;
    }
#endif
    return 0;
}
#endif	/* VLX_SERVICES_THREADS */

