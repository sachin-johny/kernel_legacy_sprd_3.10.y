#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/termios.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#define PORT_SPX 1234 /* FIXME */

#define SPX_VCOM_SERIAL

#define SP_TTY_NAME	"ttyS"
#define SP_TTY_DEVFS_NAME	"tts/"
#define SP_TTY_MINOR_START	64
#define SP_TTY_MAJOR	TTY_MAJOR

#define ARM_UART_TXD	0x0000
#define ARM_UART_RXD	0x0004
#define ARM_UART_STS0	0x0008
#define ARM_UART_STS1	0x000C
#define ARM_UART_IEN	0x0010
#define ARM_UART_ICLR	0x0014
#define ARM_UART_CTL0	0x0018
#define ARM_UART_CTL1	0x001C
#define ARM_UART_CTL2	0x0020
#define ARM_UART_CLKD0	0x0024
#define ARM_UART_CLKD1	0x0028
#define ARM_UART_STS2	0x002C

#define UART1_TXD	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_TXD)))
#define UART1_RXD	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_RXD)))
#define UART1_STS0	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_STS0)))
#define UART1_STS1	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_STS1)))
#define UART1_STS2	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_STS2)))
#define UART1_IEN	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_IEN)))
#define UART1_ICLR	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_ICLR)))
#define UART1_CTL0	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_CTL0)))
#define UART1_CTL2	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_CTL2)))
#define UART1_CLKD0	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_CLKD0)))
#define UART1_CLKD1	(*((volatile unsigned int *)(SPRD_SERIAL1_BASE+ARM_UART_CLKD1)))

#define INT_IRQ_EN	(*((volatile unsigned int *)(SPRD_INTCV_BASE+0x0010)))
#define INT_IRQ_EN_CLR	(*((volatile unsigned int *)(SPRD_INTCV_BASE+0x0014)))
#define INT_IRQ_STS	(*((volatile unsigned int *)(SPRD_INTCV_BASE+0x0004)))

#define IRQ_UART1	24

#define SP_TX_FIFO	8
#define SP_RX_FIFO	8

#define BIT_0	(0x1<<0)
#define BIT_1	(0x1<<1)
#define BIT_7	(0x1<<7)

static void serialsc8800_stop_tx(struct uart_port *port)
{
	unsigned long flags;
	spin_lock_irqsave(&port->lock,flags);
	UART1_ICLR |=BIT_1;
	UART1_IEN &=~BIT_1;
	spin_unlock_irqrestore(&port->lock,flags);
}
/*static int vcom_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	int count,i=0;
	char vcom_buff[128];
	memset(vcom_buff,0,128);
	if(port->x_char){
		port->icount.tx++;
		port->x_char=0;
		goto out;
	}
	if(uart_circ_empty(xmit) || uart_tx_stopped(port)){
		goto out;
	}
	count=62;
	do{
		vcom_buff[i++] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail+1) & (UART_XMIT_SIZE-1);
		port->icount.tx++;
		if(uart_circ_empty(xmit))
			break;
	}while(--count>0);
	vcom_buff[i++] = 0;
	vcom_buff[i] = 0;

	printk("%s\n",&vcom_buff[0]);

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		uart_write_wakeup(port);
	}
	out:
		return 0;
}*/
static void serialsc8800_start_tx(struct uart_port *port)
{
	unsigned long flags;
	spin_lock_irqsave(&port->lock,flags);
	//vcom_tx_chars(port);
	//if(!(INT_IRQ_EN & 1<<IRQ_UART1))
		//INT_IRQ_EN =1<<IRQ_UART1;
	if(!(UART1_IEN & BIT_1))
		UART1_IEN |=BIT_1;
	spin_unlock_irqrestore(&port->lock,flags);
}	
static void serialsc8800_stop_rx(struct uart_port *port)
{
	unsigned long flags;
	spin_lock_irqsave(&port->lock,flags);
	UART1_IEN &=~(BIT_0 | BIT_7);
	UART1_ICLR |=BIT_0|BIT_7;
	spin_unlock_irqrestore(&port->lock,flags);
}
static void serialsc8800_enable_ms(struct uart_port *port)
{
}
static irqreturn_t serialsc8800_rx_chars(int irq,void *dev_id)
{
	struct uart_port *port=dev_id;
	struct tty_struct *tty=port->state->port.tty;
	unsigned int status,ch,flag,rxs,max_count=96;
	status = UART1_STS1;
	//printk("serialsc8800_rx_chars func: interrupt handler rx chars\n");
	while((status & 0x00ff) && max_count--){
		ch = UART1_RXD;
		flag = TTY_NORMAL;
		port->icount.rx++;
		rxs = UART1_STS1;
		//printk("rx_fifo_count=%d\n",rxs);
		uart_insert_char(port,0,0,ch,flag);
		status=UART1_STS1;
	}
	tty->low_latency = 1;
	tty_flip_buffer_push(tty);
	return IRQ_HANDLED;
}
static irqreturn_t serialsc8800_tx_chars(int irq,void *dev_id)
{
	struct uart_port *port=dev_id;
	struct circ_buf *xmit=&port->state->xmit;
	int count;
	//printk("serialsc8800_tx_chars func: interrupt handler tx chars\n");
	if(port->x_char){
		UART1_TXD = port->x_char;		
		port->icount.tx++;
		port->x_char=0;
		goto out;
	}
	if(uart_circ_empty(xmit) || uart_tx_stopped(port)){
		serialsc8800_stop_tx(port);		
		goto out;
	}
	count=SP_TX_FIFO;
	do{
		UART1_TXD = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail+1) & (UART_XMIT_SIZE-1);
		port->icount.tx++;
		if(uart_circ_empty(xmit))
			break;
	}while(--count>0);
	
	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		uart_write_wakeup(port);
	}
	if(uart_circ_empty(xmit)){
		serialsc8800_stop_tx(port);	
	}
	out:
		return IRQ_HANDLED;
}
static unsigned int serialsc8800_tx_empty(struct uart_port *port)
{
	if(UART1_STS1 & 0xff00)
		return 0;
	else
		return 1;
}
static unsigned int serialsc8800_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}
static void serialsc8800_set_mctrl(struct uart_port *port,unsigned int mctrl)
{
}
static void serialsc8800_break_ctl(struct uart_port *port,int break_state)
{
	//return 0;
}
static irqreturn_t serialsc8800_interrupt_chars(int irq,void *dev_id)
{
	int pass_counter=0;
	while(1){
		if(!(UART1_STS0 & UART1_IEN)){
			//printk("serialsc8800_interrupt_chars func:goto break\n");			
			break;
		}
		if(UART1_STS0 & (BIT_0 | BIT_7)){
			//printk("serialsc8800_interrupt_chars func:goto serialsc8800_rx_chars\n");
			serialsc8800_rx_chars(irq,dev_id);
		}
		if(UART1_STS0 & BIT_1){
			//printk("serialsc8800_interrupt_chars func:goto serialsc8800_tx_chars\n");
			serialsc8800_tx_chars(irq,dev_id);
		}
		UART1_ICLR = 0xffffffff;
		if(pass_counter++ > 50)
			break;
	}
	return IRQ_HANDLED;
}

static int serialsc8800_startup(struct uart_port *port)
{
	int ret=0;
	unsigned int temp;
	UART1_CTL2 =0x801;
	while(UART1_STS1 & 0x00ff){
		temp =UART1_RXD;
	}
	while(UART1_STS1 & 0xff00);
	UART1_IEN &=0x00;
	UART1_ICLR =0xff;
	//printk("begin enter request_irq\n");
	ret = request_irq(IRQ_UART1,serialsc8800_interrupt_chars,0,"serial",port);
	if(ret)
	{
 		printk("fail to request serial irq\n");
		free_irq(IRQ_UART1,port);
	}
	UART1_IEN |=BIT_0|BIT_1|BIT_7;
	INT_IRQ_EN |=1<<IRQ_UART1;
	return ret;
}
static void serialsc8800_shutdown(struct uart_port *port)
{
	INT_IRQ_EN_CLR |=(1<<IRQ_UART1);
	UART1_IEN=0;
	UART1_ICLR =0xff;
	free_irq(IRQ_UART1,port);
}
static void serialsc8800_set_termios(struct uart_port *port,struct ktermios *termios,struct ktermios *old)
{
}
static const char *serialsc8800_type(struct uart_port *port)
{
	return "SPX";
}
static void serialsc8800_release_port(struct uart_port *port)
{
}
static int serialsc8800_request_port(struct uart_port *port)
{
	return 0;
}
static void serialsc8800_config_port(struct uart_port *port,int flags)
{
	if(flags & UART_CONFIG_TYPE &&serialsc8800_request_port(port)==0)
		port->type =PORT_SPX;
}
static int serialsc8800_verify_port(struct uart_port *port,struct serial_struct *ser)
{
	int ret=0;
	if(ser->type !=PORT_SPX)
		ret=-EINVAL;
	if(ser->irq !=IRQ_UART1)
		ret=-EINVAL;
	return ret;
}
static struct uart_ops serialsc8800_ops = {
	.tx_empty =serialsc8800_tx_empty,
	.get_mctrl =serialsc8800_get_mctrl,
	.set_mctrl =serialsc8800_set_mctrl,
	.stop_tx =serialsc8800_stop_tx,
	.start_tx =serialsc8800_start_tx,
	.stop_rx =serialsc8800_stop_rx,
	.enable_ms =serialsc8800_enable_ms,
	.break_ctl =serialsc8800_break_ctl,
	.startup =serialsc8800_startup,
	.shutdown =serialsc8800_shutdown,
	.set_termios =serialsc8800_set_termios,
	.type =serialsc8800_type,
	.release_port =serialsc8800_release_port,
	.request_port =serialsc8800_request_port,
	.config_port=serialsc8800_config_port,
	.verify_port =serialsc8800_verify_port,
};
static struct uart_port serialsc8800_port = {
	.iotype =SERIAL_IO_PORT,
	.iobase =0x84000000,
	.irq =IRQ_UART1,
	.fifosize =8,
	.ops =&serialsc8800_ops,
	.flags =ASYNC_BOOT_AUTOCONF,
	.line =0,
};
static void serialsc8800_setup_ports(void)
{
	serialsc8800_port.uartclk=13000000;
}


static inline void wait_for_xmitr(void)
{
	unsigned int status,tmout=1000;
	do{
		status = UART1_STS0;
		if(--tmout == 0)
			break;
	}while((status & BIT_1)!=BIT_1);
}
static void serialsc8800_console_write(struct console *co,const char *s,unsigned int count)
{
	int i,ien;
	ien=UART1_IEN;
	UART1_IEN=0;
	for(i=0;i<count;i++){
		wait_for_xmitr();
		UART1_TXD=s[i];
		if(s[i] == '\n'){
			wait_for_xmitr();
			UART1_TXD='\r';
		}
	}
	wait_for_xmitr();
	UART1_IEN = ien;
}
static void __init serialsc8800_get_options(struct uart_port *port,int *baud,int *parity,int *bits)
{
	unsigned int tmp;
	tmp=UART1_CTL0;
	switch(tmp & 0x0c){
	case 0x00:
		*bits=5;
		break;
	case 0x01:
		*bits=6;
		break;
	case 0x02:
		*bits=7;
		break;
	case 0x03:
		*bits=8;
		break;
	}
	if(tmp & 0x1){
		*parity='o';
		if(tmp & 0x2)
			*parity='e';
	}
	tmp=UART1_CLKD0 | ((0x1f & (UART1_CLKD1))<<16);
	*baud = port->uartclk/(16*(tmp+1));
}
static int __init serialsc8800_console_setup(struct console *co,char *options)
{
	struct uart_port *port = &serialsc8800_port;
	int baud =115200;
	int bits =8;
	int parity ='n';
	int flow ='n';
	
 	//if(machine_is_personal_server())
		//baud = 57600;
	if(options)
		uart_parse_options(options,&baud,&parity,&bits,&flow);
	else
		serialsc8800_get_options(port,&baud,&parity,&bits);
	return uart_set_options(port,co,baud,parity,bits,flow);
}
static struct uart_driver serialsc8800_reg;
static struct console serialsc8800_console = {
	.name ="ttyS",
	.write =serialsc8800_console_write,
	.device =uart_console_device,
	.setup =serialsc8800_console_setup,
	.flags =CON_PRINTBUFFER,
	.index =-1,
	.data =&serialsc8800_reg,
};
static int __init rssc8800_console_init(void)
{
	serialsc8800_setup_ports();
	register_console(&serialsc8800_console);
	return 0;
}
console_initcall(rssc8800_console_init);
#define SERIAL_SC8800_CONSOLE &serialsc8800_console

#define UART_NR 1
static struct uart_driver serialsc8800_reg = {
	.owner = THIS_MODULE,
	.driver_name = "serial_sc8800",
	/*.devfs_name = SP_TTY_DEVFS_NAME,*/
	.dev_name = SP_TTY_NAME,
	.major = SP_TTY_MAJOR,
	.minor = SP_TTY_MINOR_START,
	.nr = UART_NR,
	.cons = SERIAL_SC8800_CONSOLE,
};
static int __init serialsc8800_init(void)
{
	int ret;
	printk(KERN_INFO"Serial:sc8800s driver $Revision:1.0 $\n");
	serialsc8800_setup_ports();
	ret = uart_register_driver(&serialsc8800_reg);
	if(ret ==0)
		printk("serialsc8800_init:enter uart_add_one_port\n");
		uart_add_one_port(&serialsc8800_reg,&serialsc8800_port);
	return ret;
}
static void __exit serialsc8800_exit(void)
{
	uart_remove_one_port(&serialsc8800_reg,&serialsc8800_port);
	uart_unregister_driver(&serialsc8800_reg);
}
module_init(serialsc8800_init);
module_exit(serialsc8800_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sp serial driver $Revision:1.0$");
