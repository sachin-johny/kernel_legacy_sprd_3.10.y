/*
 ****************************************************************
 *
 * Component = Linux console driver on top of the nanokernel
 *
 * Copyright (C) 2002-2005 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * #ident  "@(#)console.c 1.1     07/10/18 VirtualLogix"
 *
 * Contributor(s):
 *   Vladimir Grouzdev (grouzdev@jaluna.com) Jaluna SA
 *   Francois Armand (francois.armand@jaluna.com) Jaluna SA
 *   Guennadi Maslov (guennadi.maslov@jaluna.com) Jaluna SA
 *   Gilles Maigne (gilles.maigne@jaluna.com) Jaluna SA
 *   Chi Dat Truong <chidat.truong@jaluna.com>
 *
 ****************************************************************
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/nkern.h>
#include <nk/nkern.h>

//#define __TRACE__

#ifdef  __TRACE__
#define CON_TRACE printk
#else
#define CON_TRACE(...)
#endif

//#define CON_DEBUG

#ifdef  CON_DEBUG
#define CON_PRINT printk
#else
#define CON_PRINT(...)
#endif

#define CONFIG_TS0710_MUX_UART

#define SERIAL_NK_NAME	  "ttyNK"
#define SERIAL_NK_MAJOR	  220 /* GMv 254 */
#define SERIAL_NK_MINOR	  0

#define	NKLINE(tty)	((tty)->index)
#define	NKPORT(tty) ( (NkPort*)((tty)->driver_data))

#define	SERIAL_NK_TIMEOUT	(HZ/10)	/* ten times per second */
#define	SERIAL_NK_RXLIMIT	256	/* no more than 256 characters */
typedef struct NkPort NkPort;
#define MAX_BUF			32


#ifdef CONFIG_TS0710_MUX_UART
extern struct mux_ringbuffer rbuf;
struct tty_driver *serial_for_mux_driver = NULL;
static int serial_mux_guard = 0;
struct tty_struct *serial_for_mux_tty = NULL;
extern ssize_t mux_ringbuffer_write(struct mux_ringbuffer *rbuf, const u8 *buf, size_t len);
extern ssize_t mux_ringbuffer_free(struct mux_ringbuffer *rbuf);
extern int cmux_opened(void);
extern int is_cmux_mode(void);
void (*serial_mux_dispatcher)(struct tty_struct *tty) = NULL;
void (*serial_mux_sender)(void) = NULL;
#endif

struct NkPort {
	struct timer_list	timer;
	unsigned int		poss;    
	char			buf[MAX_BUF];
	volatile char		stoprx;    
	volatile char		stoptx;    
	unsigned short		count;
	unsigned short		sz;
	NkOsId			id;
	int			(*poll)(NkPort*, int*);
	spinlock_t		lock;
	struct tty_struct*	tty;
	int			wcount;
	int			ewcount;
	NkDevVlink*		vlink;
	NkXIrqId		xid;
};

#define MAX_PORT 4
struct NkPort serial_port[MAX_PORT];

static struct tty_driver     serial_driver;
static struct tty_operations serial_ops;
static struct tty_struct*    serial_table[MAX_PORT];
static struct ktermios*      serial_termios[MAX_PORT];
static struct ktermios*      serial_termios_locked[MAX_PORT];

static int use_only_console_output;

    static void
nk_serial_console_init(char *cmdline)
{
    char *options;
    options = strstr(cmdline, "console="SERIAL_NK_NAME);
    if (!options) {
	use_only_console_output = 1;
    } else {
	use_only_console_output = 0;
    }
}

    static int
nk_poll_null(NkPort* port, int* c)
{
    return 0;
}

    static int
nk_poll(NkPort* port, int* c)
{
    int res;
    char ch;

    res = os_ctx->cops.read(port->id, &ch, 1);

    *c = ch;

    return res;
}

    static int
nk_poll_hist(NkPort* port, int* pc)
{
    int c;
    unsigned long flags;

    for (;;) {
	hw_local_irq_save(flags);
    	c = os_ctx->hgetc(os_ctx, port->poss);
	hw_local_irq_restore(flags);
	if (c) {
	    break;
	}
	port->poss++;
    }
	    
    if (c > 0) {
	port->poss++;
	*pc = c;
    }

    return c;
}

    void
printnk (const char* fmt, ...)
{
    va_list args;
    int     size;
    char    buf[256];

    va_start(args, fmt);
    size = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (size >= sizeof(buf)) {
	size = sizeof(buf)-1;
    }

    os_ctx->cops.write_msg(os_ctx->id, buf, size);
}

#define ROOM(port)  (MAX_BUF - (port)->sz - 1)


    static int
serial_write_room (struct tty_struct* tty)
{
    return ROOM(NKPORT(tty));
}

	static inline int
_write(const u_char* buf, int size, int chan)
{
	int			res;

	res = os_ctx->cops.write(chan, buf , size);

	return res;
}

	static int
_flush_chars (struct tty_struct *tty)
{
	NkPort*		port = NKPORT(tty);
	if (port->sz) {
		int		sz;
		sz = _write(port->buf, port->sz, port->id);
		if (sz > 0) {
			port->sz -= sz;
			if (port->sz) {
				memcpy(port->buf, port->buf + sz, port->sz);
			}
		}
	}
	return port->sz;
}

	static void
_flush_input(NkPort* port)
{
	int		res;
	char		ch[16];
	/* flush input buffers */
	do {
		res = os_ctx->cops.read(port->id, ch, sizeof(ch));
	} while (res > 0);
}
	
#ifdef CONFIG_TS0710_MUX_UART
#define RX_NUM_MAX    2048
static char rev[RX_NUM_MAX*10];
#endif
	static void
serial_rx_intr (void* data, NkXIrq irq)
{
	NkPort*          	port = (NkPort*) data;
	unsigned int		size = SERIAL_NK_RXLIMIT;
	int			c;
#ifdef CONFIG_TS0710_MUX_UART
	int line,num;
    unsigned long flags;  
#endif
	
    if (port->count == 0) {
		_flush_input(port);
		return;
	}
	
#ifdef CONFIG_TS0710_MUX_UART
	line = NKLINE(port->tty);
	if ((3==line)&& cmux_opened()){
		num =0; 	
	    //printk("\nSR<");
        hw_local_irq_save(flags);
		//while (!port->stoprx && size && port->poll(port, &c) > 0 ) {
		while (!port->stoprx && (num=os_ctx->cops.read(port->id, rev, RX_NUM_MAX))){
            //printk("%x",c);
		    mux_ringbuffer_write(&rbuf, rev, num);
		}
        //printk("func[%s]:rev_num=%d\n",__FUNCTION__,num);
		//printk("\n%dSR>",num);
		if (serial_mux_dispatcher && is_cmux_mode())
			serial_mux_dispatcher(port->tty);
        hw_local_irq_restore(flags);
        CON_TRACE("func[%s]:rev_num=%d\n",__FUNCTION__,num);
		return;
	}
	else {
#endif
    CON_PRINT("func[%s]:rx<\n",__FUNCTION__);
	while (!port->stoprx && size && port->poll(port, &c) > 0 ) {
		tty_insert_flip_char(port->tty, c, TTY_NORMAL);
        CON_PRINT("%x",c);
		size--;
	}
    CON_PRINT("func[%s]:rx>\n",__FUNCTION__);

	if (size < SERIAL_NK_RXLIMIT) {
		tty_flip_buffer_push(port->tty);
	}
#ifdef CONFIG_TS0710_MUX_UART
	}
#endif
}

	static void
serial_tx_intr (void* data, NkXIrq irq)
{
	NkPort*          	port = (NkPort*) data;
	unsigned long		flags;
    int line;

	spin_lock_irqsave(&port->lock, flags);
	if (port->sz) {
        //printk("func[%s]:tx_buf=0x%x\n",__FUNCTION__,port->buf);
		if (_flush_chars(port->tty) == 0 )
		    tty_wakeup(port->tty);
	}
	spin_unlock_irqrestore(&port->lock, flags);

#ifdef CONFIG_TS0710_MUX_UART
	line = NKLINE(port->tty);
	if ((3==line)&& cmux_opened()){
        CON_PRINT("func[%s]:serial_mux_sender\n",__FUNCTION__);
		if(serial_mux_sender){
			serial_mux_sender();	
		}	
	}
#endif

}

	static void 
serial_intr(void* data, NkXIrq irq)
{
    
  CON_PRINT("func[%s]: irq=%d\n",__FUNCTION__,irq);
	serial_rx_intr(data, irq);
	serial_tx_intr(data, irq);
}

    static void
serial_send_xchar (struct tty_struct* tty, char c)
{
}

    static void
serial_throttle (struct tty_struct* tty)
{
    NKPORT(tty)->stoprx = 1;
}

    static void
serial_unthrottle (struct tty_struct* tty)
{
    NkPort*		port = NKPORT(tty);
    port->stoprx = 0;
    serial_rx_intr(port, 0);	
}


    static int
serial_write (struct tty_struct* tty,
	      const u_char*      buf,
	      int                count)
{
    NkPort*			port = NKPORT(tty);
    int			res = 0;
    unsigned long		flags;
    
    u_char* buffer;
    int num;
   
    buffer=buf;
    num=count;
  
    if(3 == NKLINE(tty)){
        CON_TRACE("func[%s]: count=%d\n",__FUNCTION__,count);
    }
   
    if(3 == NKLINE(tty)){
        CON_TRACE("func[%s]: write<0x",__FUNCTION__);
        while(num > 0){
            CON_TRACE("%x",*buffer);
            buffer++;
            num--;
        }
        CON_TRACE(">\n");
    }
    if (NKLINE(tty) == 1) {
	return count;
    }

    spin_lock_irqsave(&port->lock, flags);
    if (_flush_chars(tty) == 0) {
            if(3 == NKLINE(tty)){
                    CON_TRACE("func[%s]: gonna do _write()\n",__FUNCTION__);
            } 
            res = _write(buf, count, port->id);
    }
    if ( res < count ) {
	int		room = ROOM(port);

	if ( (count - res) < room  ) {
	    room = count - res;	
	}

	memcpy(port->buf + port->sz, buf + res, room); 
	port->sz += room;
	res += room;

    }
    spin_unlock_irqrestore(&port->lock, flags);
#if 0
#ifdef CONFIG_TS0710_MUX_UART
    if(3 == NKLINE(tty) && cmux_opened()){
        CON_PRINT("func[%s]:serial_mux_sender\n",__FUNCTION__);
		if(serial_mux_sender){
			serial_mux_sender();	
		}	
	}
#endif
#endif
    if(3 == NKLINE(tty)){
        CON_TRACE("func[%s]: res=%d\n",__FUNCTION__,res);
    }
    return res;
}


    static int
serial_chars_in_buffer (struct tty_struct* tty)
{
    return NKPORT(tty)->sz;
}

    static void
serial_flush_buffer (struct tty_struct* tty)
{
    unsigned long	flags;
    NkPort*		port = NKPORT(tty);
    spin_lock_irqsave(&port->lock, flags);
    NKPORT(tty)->sz = 0; 
    spin_unlock_irqrestore(&port->lock, flags);
    tty_wakeup(tty);
}

    static void
serial_set_termios (struct tty_struct* tty, struct ktermios* old)
{
}

    static void
serial_stop (struct tty_struct* tty)
{
    NKPORT(tty)->stoptx = 1;
}

    static void
serial_start(struct tty_struct* tty)
{
    NKPORT(tty)->stoptx = 0;
}


    static void
serial_timeout (unsigned long data)
{
    struct tty_struct*	tty  = (struct tty_struct*)data;
    unsigned int		size = SERIAL_NK_RXLIMIT;
    int			c;
    unsigned long		flags;

    NkPort*          port = NKPORT(tty);

    if(3 == NKLINE(tty))
        CON_PRINT("func[%s]\n",__FUNCTION__);
    while (!port->stoprx && size && port->poll(port, &c) > 0 ) {
	tty_insert_flip_char(tty, c, TTY_NORMAL);
	size--;
    }
    if (size < SERIAL_NK_RXLIMIT) {
	tty_flip_buffer_push(tty);
    }

    spin_lock_irqsave(&port->lock, flags);
    if (port->sz) {
	_flush_chars(port->tty);
	tty_wakeup(port->tty);
    }
    spin_unlock_irqrestore(&port->lock, flags);


    port->timer.expires = jiffies + SERIAL_NK_TIMEOUT;
    add_timer(&(port->timer));
}


    static int
serial_open (struct tty_struct* tty, struct file* filp)
{
    int			line;
    NkPort* 		port;
    NkPhAddr		plink = 0;
    NkDevVlink*		vlink;
    int			irq;

    line = NKLINE(tty);
    port = line + serial_port;
	
#ifdef CONFIG_TS0710_MUX_UART
	if(line == 3){

	        if( serial_mux_guard ) {
			serial_mux_guard++;
			printk("Fail to open ttyNK3, it's busy!\n");
			return -EBUSY;
		} else {
			serial_mux_guard++;
			serial_for_mux_tty = tty;
			printk("=========serial_for_mux_tty=%p========\n",serial_for_mux_tty);
		}
	}
#endif

    if (line >= MAX_PORT) {
	return -ENODEV;
    }

    port->count++;

    if (port->count > 1) {
	return 0;
    }

    port->id		= os_ctx->id;
    tty->driver_data	= port;
    port->stoptx		= 0;
    port->stoprx		= 0;
    port->sz		= 0;
    port->poss		= 0;
    port->tty		= tty;
    port->wcount		= 0;
    port->ewcount		= 0;
    port->vlink		= 0;
    port->xid		= 0;

    spin_lock_init(&port->lock);

    if (use_only_console_output) {
	port->poll = nk_poll_null;
	return 0;
    }

    port->poll		 = nk_poll;
    if (line == 1) {
	port->poll = nk_poll_hist;
    }
    port->timer.data = 0;


    while ((plink = nkops.nk_vlink_lookup("vcons", plink))) {
    CON_PRINT("func[%s]:plink was found\n",__FUNCTION__);
	vlink = (NkDevVlink*) nkops.nk_ptov(plink);

	if (vlink->s_id == nkops.nk_id_get()) {
	    
        int   minor = 0;
	    if (vlink->s_info) {
		char*	resinfo;
		char*   info;
		int	t;

		info = (char*) nkops.nk_ptov(vlink->s_info);

		t = simple_strtoul(info, &resinfo, 0);
		if (resinfo != info) {
		    minor = t;	
		}
	    }

	    if (vlink->link == nkops.nk_id_get() && 
		(tty->index != 0)) {
		    /* this should never occurs */
		continue;
	    }

	    if (minor == tty->index) {
		goto found_vlink;
	    }
	}
    }
    
    CON_PRINT("func[%s]:vlink was not found\n",__FUNCTION__);
	/*
	 * no vlink found so use timer to poll character and restart output
	 */
	init_timer(&port->timer);
	port->timer.data     = (unsigned long)tty;
	port->timer.function = serial_timeout;
	port->timer.expires  = jiffies + SERIAL_NK_TIMEOUT;
	add_timer(&port->timer);

	return 0;

found_vlink:
    
    CON_PRINT("func[%s]:vlink was found\n",__FUNCTION__);
	/* found a vlink for this tty device */
	port->vlink = vlink;

	/* 
	 * if link is null, we use os id as channel number
	 * otherwise one use the underlying link number as channel number
	 */
	if (vlink->link != 0) {
		port->id = vlink->link;
	}

	irq = nkops.nk_pxirq_alloc(plink, 1, vlink->s_id, 1);
	if (!irq) {
		return -ENOMEM;
	}

	os_ctx->cops.open(vlink->link);

	port->xid = nkops.nk_xirq_attach(irq, 
					 serial_intr, 
					 port);
	if (port->xid == 0) {
		return -ENOMEM;
	}

#ifdef DEBUG
	printk("Open ttyNK<%d> portid %x", line, port->id);
	printk("vlink %p vlink->link %x\n", vlink, vlink->link);
#endif
	return 0;
}

	static void
serial_close (struct tty_struct* tty, struct file* filp)
{ 
	NkPort* port = (NkPort*)tty->driver_data;

#ifdef CONFIG_TS0710_MUX_UART
	int line = NKLINE(tty);
	if (line == 3) {
		serial_mux_guard--;
	}
#endif
	
	port->count--;
	if (port->count == 0) {
		unsigned long	flags;

		os_ctx->cops.close(port->id);

		/* mask xirq */
		if (port->xid) {
		    nkops.nk_xirq_detach(port->xid);
		    port->xid = 0;
		}

		port->tty = 0;
		if (port->timer.data) {
			del_timer(&(port->timer));
		}

		spin_lock_irqsave(&port->lock, flags);
		_flush_input(port);
		_flush_chars(tty);
		spin_unlock_irqrestore(&port->lock, flags);

#ifdef DEBUG
		printk("Closing ttyNK<%d> 0x%x 0x%x \n", 
		       tty->index, port->ewcount, port->wcount);
#endif
	}
}

   static void
serial_wait_until_sent (struct tty_struct* tty, int timeout)
{
}

	static int __init
serial_init (void)
{
    serial_driver.owner           = THIS_MODULE;
    serial_driver.magic           = TTY_DRIVER_MAGIC;
    serial_driver.driver_name     = "nkconsole";
    serial_driver.name            = SERIAL_NK_NAME;
/* GMv    serial_driver.devfs_name      = SERIAL_NK_NAME; */
    serial_driver.major           = SERIAL_NK_MAJOR;
    serial_driver.minor_start     = SERIAL_NK_MINOR;
    serial_driver.num             = MAX_PORT;
    serial_driver.type            = TTY_DRIVER_TYPE_SERIAL;
    serial_driver.subtype         = SERIAL_TYPE_NORMAL;
    serial_driver.init_termios    = tty_std_termios;
    serial_driver.init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    serial_driver.flags           = TTY_DRIVER_REAL_RAW;
    kref_init(&serial_driver.kref);
    serial_driver.ttys            = serial_table;
    serial_driver.termios         = serial_termios;
    serial_driver.termios_locked  = serial_termios_locked;
    serial_driver.ops             = &serial_ops;

    serial_ops.open            = serial_open;
    serial_ops.close           = serial_close;
    serial_ops.write           = serial_write;
    serial_ops.write_room      = serial_write_room;
    serial_ops.chars_in_buffer = serial_chars_in_buffer;
    serial_ops.flush_buffer    = serial_flush_buffer;
    serial_ops.throttle        = serial_throttle;
    serial_ops.unthrottle      = serial_unthrottle;
    serial_ops.send_xchar      = serial_send_xchar;
    serial_ops.set_termios     = serial_set_termios;
    serial_ops.stop            = serial_stop;
    serial_ops.start           = serial_start;
    serial_ops.wait_until_sent = serial_wait_until_sent;

    if (tty_register_driver(&serial_driver)) {
    	printk(KERN_ERR "Couldn't register NK console driver\n");
    }
#ifdef CONFIG_TS0710_MUX_UART
	serial_for_mux_driver = &serial_driver;
	printk("=========serial_for_mux_driver=%p========\n",serial_for_mux_driver);
	//serial_for_mux_tty = &serial_tty;
	serial_mux_guard = 0;
#endif
	return 0;
}

    static void __exit
serial_fini(void)
{
    unsigned long flags;

    local_irq_save(flags);

    if (tty_unregister_driver(&serial_driver)) {
	printk(KERN_ERR "Unable to unregister NK console driver\n");
    }

    local_irq_restore(flags);
}

    static int
nk_console_setup (struct console* c, char* unused)
{
    return 1;
}

    static void
nk_console_write (struct console* c, const char* buf, unsigned int count)
{
    os_ctx->cops.write_msg(os_ctx->id, buf, count);
}

    static struct tty_driver*
nk_console_device (struct console* c, int* index)
{
    *index = c->index;
    return &serial_driver;
}

static struct console nkcons =
{
	name:	SERIAL_NK_NAME,
	write:	nk_console_write,
	device:	nk_console_device,
	setup:	nk_console_setup,
	flags:	CON_ENABLED,
	index:	-1,
};

    void __init
nk_console_init (void)
{
    nk_serial_console_init(boot_command_line);
    register_console(&nkcons);
}

module_init(serial_init);
module_exit(serial_fini);

