/*
 ****************************************************************
 *
 *  Component:	VLX User Mode virtual driver Proxy driver
 *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski <adam.mirowski@redbend.com>
 *
 ****************************************************************
 */

/*----- Header files -----*/

#include <linux/version.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/sched.h>	/* TASK_INTERRUPTIBLE */
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <nk/nkern.h>
#include <vlx/vlx-ump.h>

/*----- Local configuration -----*/

#if 1
#define TRACE_SYSCONF		/* enable to see handshake traces */
#endif

/*----- Traces -----*/

#define UMP_MSG		"UMP: "

#define TRACE(x...)	printk (KERN_NOTICE UMP_MSG x)
#define ETRACE(x...)	printk (KERN_ERR    UMP_MSG x)
#define DTRACE(x...)	printk (KERN_CRIT   UMP_MSG x)

#ifdef TRACE_SYSCONF
#define SYSCONF(x...)	printk (UMP_MSG x)
#else
#define SYSCONF(x...)
#endif

/*----- Implementation -----*/

#define UMP_NAME	"vlx-ump"

typedef struct {
    _Bool	enabled;	/* device has all resources allocated */
    struct mutex lock;		/* mutual exclusion lock for all ops */
    wait_queue_head_t wait;	/* waiting queue for all ops */
    int		count;		/* usage counter */
    NkPhAddr    data;           /* address of mmap-ed memory */
    size_t	size;		/* size of mmap-ed memory */
} ump_dev_t;

static int      ump_dev_count;	/* total count of minor */
static ump_dev_t*   ump_devs;	/* pointer to array of device descriptors */
static NkXIrqId ump_sysconf_id;	/* xirq id for sysconf handler */
static struct class* ump_class;	/* "ump" device class pointer */
static int	ump_major = -1;	/* major device of the driver */

    /*
     * NK_XIRQ_SYSCONFIG handler
     *
     * It scans all known communications devices
     * and wakes threads sleeping on corresponding
     * wait queues. Actual device state analyze
     * will be performed by awaken thread.
     */
    static void
ump_sysconf_hdl (void* cookie, NkXIrq xirq)
{
    ump_dev_t* ump_dev;
    int    i;

    SYSCONF ("ump_sysconf_hdl\n");
    for (i = 0, ump_dev = ump_devs; i < ump_dev_count; i++, ump_dev++) {
	if (ump_dev->enabled) {
	    SYSCONF ("Getting sysconf\n");
	    wake_up_interruptible (&ump_dev->wait);
	}
    }
}

    /*
     * cross interrupt handler.
     *
     * driver sends a cross interrupt to its counterpart
     * to say it has something to read or it has some
     * room to write, so it should wake up and do its work.
     * our job is simple - we need to wake reader or writer up.
     * we use a single waiting semaphore because only one
     * operation is allowed at time.
     *
     * the available room and device state will be retested
     * by awaken thread.
     */
    static void
ump_xirq_hdl (void* cookie, NkXIrq xirq)
{
    ump_dev_t*	ump_dev = (ump_dev_t*)cookie;

    wake_up_interruptible (&ump_dev->wait);
}

    /*
     * link clean up - it shuts link down
     * and free some resources (detach cross
     * interrupt handlers in our example)
     *
     * this function is called only from ump_open
     * and ump_release functions, so it is guaranteed
     * that there is no other activity on this link.
     */
    static void
ump_dev_cleanup (ump_dev_t* ump_dev)
{
}

    /*
     * ump_open implements open operation required
     * by linux semantic for character devices
     *
     * for the 1st open it attaches cross interrupt
     * handlers and perform "handshake" with peer
     * driver
     */
    static int
ump_open (struct inode* inode, struct file* file)
{
    const unsigned	minor   = iminor (inode);
    ump_dev_t*		ump_dev;

    if (minor >= ump_dev_count) {
	ETRACE ("invalid minor %d\n", minor);
	return -ENXIO;
    }
    ump_dev = &ump_devs [minor];
    if (!ump_dev->enabled) {
	ETRACE ("minor %d not enabled\n", minor);
	return -ENXIO;
    }
    if (mutex_lock_interruptible (&ump_dev->lock)) {
	ETRACE ("wait %d interrupted\n", minor);
	return -EINTR;
    }
    SYSCONF ("ump_open for minor=%d is OK\n", minor);
    mutex_unlock (&ump_dev->lock);
    return 0;
}

    /*
     * ump_release implements release (close) operation
     * required by linux semantic for character devices
     *
     * for the last close it shuts link down.
     *
     * Note that ioctl operations can detect
     * that something is wrong with peer driver
     * (it went to NK_DEV_OFF state). They will exit
     * with -EPIPE error code or they will return less bytes
     * the required in this case,  but they will not change
     * link state.
     *
     * After that the application may call open() to reopen
     * communication device and resume communication.
     */
    static int
ump_release (struct inode* inode, struct file* file)
{
    const unsigned	minor   = iminor (inode);
    ump_dev_t*		ump_dev = &ump_devs [minor];

	/*
         * ensure only one thread executes this operation
	 */
    if (mutex_lock_interruptible (&ump_dev->lock)) {
	ETRACE ("lock wait %d interrupted\n", minor);
	return -EINTR;
    }
    if (!--ump_dev->count) {
	ump_dev_cleanup (ump_dev);
	SYSCONF ("ump_release for minor=%d is OK\n", minor);
    }
    mutex_unlock (&ump_dev->lock);
    return 0;
}

    static int
ump_ioctl (struct inode* inode, struct file* file,
	   unsigned int cmd, unsigned long arg)
{
    const unsigned	minor   = iminor (inode);
    ump_dev_t*		ump_dev = &ump_devs [minor];
    int			diag;

    DTRACE ("ioctl cmd %x arg %lx\n", cmd, arg);
    if (mutex_lock_interruptible (&ump_dev->lock)) {
	ETRACE ("lock wait %d interrupted\n", minor);
	return -EINTR;
    }
    switch (cmd) {
    case UMPIOC_GET_VERSION:
	diag = sizeof *os_ctx;
	DTRACE ("get_version %d\n", diag);
	break;

    case UMPIOC_GET_NKOSCTX:
	diag = copy_to_user ((NkOsCtx __user*) arg, os_ctx, sizeof *os_ctx);
	DTRACE ("get_nkosctx %d\n", diag);
	if (diag) {
	    diag = -EFAULT;
	}
	break;

    case UMPIOC_READ_VIRT: {
	ump_ioctl_t	desc;

	diag = copy_from_user (&desc, (void __user*) arg, sizeof desc);
	DTRACE ("read_virt desc %d\n", diag);
	if (diag) {
	    diag = -EFAULT;
	    break;
	}
	DTRACE ("desc buf %p addr %lx size %x\n",
		desc.buf, desc.addr, desc.size);
	DTRACE ("at addr: %lx %lx\n", ((long*)desc.addr) [0],
		((long*)desc.addr) [1]);
	diag = copy_to_user ((void __user*) desc.buf, (void*) desc.addr,
			     desc.size);
	DTRACE ("read_virt copy %d\n", diag);
	if (diag) {
	    diag = -EFAULT;
	    break;
	}
	break;
    }
    default:
	ETRACE ("no ioctl %d\n", cmd);
	diag = -EINVAL;
    }
    mutex_unlock (&ump_dev->lock);
    return diag;
}

    /*
     * ump_mmap maps communication buffer to user space
     */
    static int
ump_mmap (struct file* file, struct vm_area_struct* vma)
{
    const unsigned	minor   = iminor (file->f_path.dentry->d_inode);
    ump_dev_t*		ump_dev = &ump_devs [minor];
    int			diag;
	/*
	 * Check mmap parameters
	 */
    if (((vma->vm_end - vma->vm_start) !=  ump_dev->size) || vma->vm_pgoff) {
	ETRACE ("invalid mmap %d parameters\n", minor);
	return -EINVAL;
    }
    vma->vm_flags    |= VM_IO;
    vma->vm_page_prot = pgprot_noncached (vma->vm_page_prot);

    diag = io_remap_pfn_range (vma, vma->vm_start, ump_dev->data >> PAGE_SHIFT,
			       ump_dev->size, vma->vm_page_prot);
    return diag;
}

    /*
     * ump_poll implements poll operation required
     * by linux semantic for character devices
     *
     * it checks if there are any characters to read and
     * if there is any room to write and reports to upper level.
     */
    static unsigned int
ump_poll (struct file* file, poll_table* wait)
{
    const unsigned	minor   = iminor (file->f_path.dentry->d_inode);
    ump_dev_t*		ump_dev = &ump_devs [minor];
    unsigned		res     = 0;

    poll_wait (file, &ump_dev->wait, wait);
    res |= (POLLIN | POLLRDNORM);
    res |= (POLLOUT | POLLWRNORM);
    return res;
}

    /*
     * this data structure will we passed as a parameter
     * to linux character device framework and inform it
     * we have implemented all 5 basic operations (open,
     * ioctl, mmap, close, poll)
     */
static const struct file_operations ump_fops = {
    .owner	= THIS_MODULE,
    .open	= ump_open,
    .ioctl	= ump_ioctl,
    .mmap	= ump_mmap,
    .release	= ump_release,
    .poll	= ump_poll,
    .llseek	= no_llseek,
};

/*----- Initialization and termination -----*/

    /*
     * ump_dev_init() function is called to initialize a device
     * instance during driver initialization phase
     * (see ump_module_init()).
     *
     * It allocates communication rings if they are not
     * already allocated by peer driver.
     *
     * It allocates cross interrupts if they are not already allocated
     * by previous driver "incarnation" (cross interrupts are "restart"
     * persistent).
     *
     * It creates "class device" so our communication device will
     * be "visible" by applications as /dev/vlx-ump*
     *
     * Finally it initialize mutual exclusion lock and
     * waiting queue.
     */

    static void
ump_dev_init (ump_dev_t* ump_dev, int minor)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION (2,6,23)
    struct device* cls_dev;
#else
    struct class_device* cls_dev;
#endif
	/*
	 * create a class device
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION (2,6,23)
    cls_dev = device_create (ump_class, NULL, MKDEV (ump_major, minor),
#else
    cls_dev = class_device_create (ump_class, NULL, MKDEV (ump_major, minor),
#endif
				   NULL, UMP_NAME "%d", minor);
    if (IS_ERR (cls_dev)) {
	ETRACE ("class device create failed (%ld)\n", PTR_ERR (cls_dev));
	return;
    }
    mutex_init          (&ump_dev->lock);
    init_waitqueue_head (&ump_dev->wait);
    ump_dev->enabled = 1;
}

    /*
     * ump_dev_destroy() function is called to free devices
     * resources and destroy a device instance during driver
     * exit phase (see ump_module_exit()).
     *
     * actually in our example it only destroys "class device"
     * because all other device resources (circular buffers
     * and cross interrupts) are persistent.
     */
    static void
ump_dev_destroy (ump_dev_t* ump_dev, int minor)
{
	/*
	 * Destroy class device
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION (2,6,23)
    device_destroy (ump_class, MKDEV (ump_major, minor));
#else
    class_device_destroy (ump_class, MKDEV (ump_major, minor));
#endif
}

    static void
ump_module_cleanup (void)
{
    if (ump_sysconf_id) {
        nkops.nk_xirq_detach (ump_sysconf_id);
    }
    if (ump_devs) {
	ump_dev_t*	ump_dev;
	int	i;

	for (i = 0, ump_dev = ump_devs; i < ump_dev_count; i++, ump_dev++) {
	    if (ump_dev->enabled) {
		ump_dev_destroy (ump_dev, i);
	    }
	}
	kfree (ump_devs);
    }
    if (ump_class) {
	class_destroy (ump_class);
    }
    if (ump_major >= 0) {
	unregister_chrdev (ump_major, UMP_NAME);
    }
}

    static int
ump_module_init (void)
{
    int	diag;

    if ((ump_major = register_chrdev (0, UMP_NAME, &ump_fops)) < 0) {
	ETRACE ("can't register chardev\n");
	return ump_major;
    }
    ump_class = class_create (THIS_MODULE, UMP_NAME);
    if (IS_ERR (ump_class)) {
	diag = PTR_ERR (ump_class);
	ump_class = NULL;
	ump_module_cleanup();
	ETRACE ("can't create class\n");
	return diag;
    }
    ump_devs = kzalloc (sizeof (*ump_devs), GFP_KERNEL);
    if (!ump_devs) {
	ump_module_cleanup();
	ETRACE ("can't alloc descriptor\n");
	return -ENOMEM;
    }
    ump_dev_init (ump_devs, 0 /*minor*/);
    ump_dev_count = 1;

    ump_sysconf_id = nkops.nk_xirq_attach (NK_XIRQ_SYSCONF, ump_sysconf_hdl, 0);
    if (!ump_sysconf_id) {
	ump_module_cleanup();
	ETRACE ("can't attach xirq\n");
	return -ENOMEM;
    }
    TRACE ("module loaded\n");
    return 0;
}

    static void
ump_module_exit (void)
{
    ump_module_cleanup();
    TRACE ("module unloaded\n");
}

/*----- Module glue -----*/

module_init (ump_module_init);
module_exit (ump_module_exit);

MODULE_DESCRIPTION ("VLX User Mode Virtual Driver proxy driver");
MODULE_AUTHOR      ("Adam Mirowski <adam.mirowski@redbend.com>");
MODULE_LICENSE     ("GPL");
