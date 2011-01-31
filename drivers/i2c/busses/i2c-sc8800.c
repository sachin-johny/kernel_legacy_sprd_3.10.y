/* linux/drivers/i2c/busses/i2c-sc8800.c
 *
 * Copyright (C) 2010 Spreadtrum
 *	
 *
 * SC8800 I2C Controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/bits.h>
#include <mach/io.h>
/*offset*/
#define I2C_CTL                 		0x0000
#define I2C_CMD                 		0x0004
#define I2C_CLKD0               		0x0008
#define I2C_CLKD1               		0x000C
#define I2C_RST                 		0x0010
#define I2C_CMD_BUF             		0x0014

//The corresponding bit of I2C_CTL register.
#define I2C_CTL_INT              		(1 << 0)        //I2c interrupt
#define I2C_CTL_ACK              		(1 << 1)        //I2c received ack value
#define I2C_CTL_BUSY             	(1 << 2)        //I2c data line value
#define I2C_CTL_IE               		(1 << 3)        //I2c interrupt enable
#define I2C_CTL_EN               		(1 << 4)        //I2c module enable
#define I2C_CTL_CMDBUF_EN        	(1 << 5)        //Enable the cmd buffer mode
#define I2C_CTL_CMDBUF_EXEC     (1 << 6)        //Start to exec the cmd in the cmd buffer
#define I2C_CTL_ST_CMDBUF        	(7 << 7)        //The state of  I2c cmd buffer state machine.
#define I2C_CTL_CMDBUF_WPTR    (7 << 7)        //I2c command buffer write pointer

//The corresponding bit of I2C_CMD register.
#define I2C_CMD_INT_ACK          		(1 << 0)        //I2c interrupt clear bit
#define I2C_CMD_TX_ACK           		(1 << 1)        //I2c transmit ack that need to be send
#define I2C_CMD_WRITE            		(1 << 2)        //I2c write command
#define I2C_CMD_READ             		(1 << 3)        //I2c read command
#define I2C_CMD_STOP             		(1 << 4)        //I2c stop command
#define I2C_CMD_START            		(1 << 5)        //I2c start command
#define I2C_CMD_ACK              		(1 << 6)        //I2c received ack  value
#define I2C_CMD_BUSY              		(1 << 7)        //I2c busy in exec commands
#define I2C_CMD_DATA             		0xFF00          //I2c data received or data need to be transmitted

/* i2c controller state */
enum sc8800_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};
struct sc8800_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;
	
	struct i2c_adapter	adap;

	enum sc8800_i2c_state	state;

	int irq;
	void __iomem		*membase;	
	unsigned int *memphys;
	unsigned int  *memsize;
};
struct sc8800_platform_i2c {
	unsigned int	normal_freq;	/* normal bus frequency */
	unsigned int	fast_freq;	/* fast frequency for the bus */
	//unsigned int	min_freq;	/* min frequency for the bus */
};
static struct sc8800_platform_i2c sc8800_i2c_default_platform = {
	.normal_freq	= 100*1000,
	.fast_freq	= 400*1000,
};
/* sc8800_i2c_wait_exec
 *
 * wait i2c cmd executable
*/
static inline int sc8800_i2c_wait_exec(struct sc8800_i2c *i2c)
{
	unsigned int cmd;
	int timeout = 1000;  //modify by kewang

	while (timeout-- > 0) {
		cmd = __raw_readl(i2c->membase + I2C_CMD);
		if (!(cmd & I2C_CMD_BUSY))
			return 0;
	}
	printk( "I2C:timeout,busy in exec commands !\n");

	return -ETIMEDOUT;
}
static inline void sc8800_i2c_disable_irq(struct sc8800_i2c *i2c)
{
	unsigned int ctl;
	ctl=__raw_readl(i2c->membase+I2C_CTL); 
	 //disable i2c interrupt
	__raw_writel(ctl &~I2C_CTL_IE,i2c->membase+I2C_CTL);	
}
static inline void sc8800_clr_irq(struct sc8800_i2c *i2c)
{
	unsigned int cmd;
	
	cmd=__raw_readl(i2c->membase+I2C_CMD); 
	//clear interrupt
	__raw_writel(cmd | I2C_CMD_INT_ACK,i2c->membase+I2C_CMD);
}
static inline void sc8800_i2c_enable_irq(struct sc8800_i2c *i2c)
{
	unsigned int ctl;
	
	ctl=__raw_readl(i2c->membase+I2C_CTL); 
	 //enable i2c interrupt
	__raw_writel(ctl | I2C_CTL_IE,i2c->membase+I2C_CTL);	
	
}
/* sc8800_i2c_message_start
 *
 * put the start bit & slave address of a message onto the bus 
*/
#define PIN_SIM_CONFIG  0
#define PIN_LCD_CONFIG  1
#define PIN_IIC_CONFIG1 2
#define PIN_IIC_CONFIG2 3

static void sc8800_i2c_message_start(struct sc8800_i2c *i2c, struct i2c_msg *msg)
{
    unsigned int cmd;
    uint16_t addr;

    addr = msg->addr;
    switch ((addr & 0xC000) >> 14){
        case PIN_SIM_CONFIG:
            //also need config ldo fix me!!!!
            __raw_bits_and(~(BIT_4 | BIT_3),SPRD_GREG_BASE+0x0028);
            break;
        case PIN_LCD_CONFIG:
            __raw_bits_or(BIT_3,SPRD_GREG_BASE+0x0028);
            __raw_bits_and(~BIT_4,SPRD_GREG_BASE+0x0028);
            break;
        case PIN_IIC_CONFIG1:
            __raw_bits_or(BIT_4,SPRD_GREG_BASE+0x0028);
            __raw_bits_and(~BIT_3,SPRD_GREG_BASE+0x0028);
            break;
        case PIN_IIC_CONFIG2:
            __raw_bits_or(BIT_3|BIT_4,SPRD_GREG_BASE+0x0028);
            break;
        default:
            printk("i2c pad switch error!!!");
            break;
    }
    cmd = __raw_readl(SPRD_GREG_BASE+0x0028);

    cmd = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD) 
		cmd |= 0x1;
    
	cmd=(cmd<<8) | I2C_CMD_START | I2C_CMD_WRITE;
	__raw_writel(cmd, i2c->membase + I2C_CMD);
	ndelay(50);
}
/* sc8800_i2c_doxfer
 *
 * this starts an i2c transfer
*/
static int sc8800_i2c_doxfer(struct sc8800_i2c *i2c, struct i2c_msg *msgs, int num)
{
	unsigned int timeout;
	//unsigned int cmd;
	unsigned long  flags;
	int ret;
#if 1
	ret = sc8800_i2c_wait_exec(i2c);
	if (ret != 0) {
		ret = -EAGAIN;
		goto out;
	}
#endif

#if 0
    cmd = __raw_readl(i2c->membase + I2C_CMD);
		
	while(cmd & I2C_CMD_BUSY)
        cmd = __raw_readl(i2c->membase + I2C_CMD);  //kewang
#endif
 
	spin_lock_irqsave(&i2c->lock,flags);

	i2c->msg = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->state = STATE_START;

	sc8800_clr_irq(i2c);
	sc8800_i2c_enable_irq(i2c);
	sc8800_i2c_message_start(i2c, msgs);
	
	spin_unlock_irqrestore(&i2c->lock,flags);
	//timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, 1);	//10mSec
	//if (i2c->msg_num != 0)
	timeout=wait_event_timeout(i2c->wait, i2c->msg_num == 0,HZ * 5);
	
	ret = i2c->msg_idx;
	/* having these next two as dev_err() makes life very 
	 * noisy when doing an i2cdetect */

	if (timeout == 0){
		printk("I2C:timeout\n");
		__raw_writel(0x1,i2c->membase+I2C_RST);  //reset i2c module
		ret = -ENXIO;
	}else if (ret != num){
		printk("incomplete xfer (%d)\n", ret);
		ret = -EAGAIN;
	}

	/* ensure the stop has been through the bus */

	msleep(5);

 out:    //kewang
	return ret;
}

static int sc8800_i2c_xfer(struct i2c_adapter *adap,struct i2c_msg *msgs, int num)
{
	struct sc8800_i2c *i2c = (struct sc8800_i2c *)adap->algo_data;
	int retry;
	int ret;

	for (retry = 0; retry < adap->retries; retry++) {

		ret = sc8800_i2c_doxfer(i2c, msgs, num);

		if (ret != -EAGAIN)
			return ret;

		printk("I2C:Retrying transmission (%d)\n", retry);

		udelay(100);
	}
	printk("I2C:transmission failed!\n");
	return -EREMOTEIO;
}
/* declare our i2c functionality */
static unsigned int sc8800_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL ;
}
/* i2c bus registration info */

static const struct i2c_algorithm sc8800_i2c_algorithm = {
	.master_xfer		= sc8800_i2c_xfer,
	.functionality		= sc8800_i2c_func,
};
static inline void sc8800_i2c_complete(struct sc8800_i2c *i2c, int ret)
{
//	printk("I2C:sc8800_i2c_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	wake_up(&i2c->wait);
}
#if 1
static inline void sc8800_i2c_stop(struct sc8800_i2c *i2c, int ret)
{
	unsigned int cmd;

	//printk("I2C:sc8800_i2c_stop!\n");

	/* stop the transfer */
	cmd= I2C_CMD_STOP | I2C_CMD_WRITE;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
	
	i2c->state = STATE_STOP;
	
	sc8800_i2c_complete(i2c, ret);
	sc8800_i2c_disable_irq(i2c);
}
#endif

/* is_lastmsg()
 *
 * returns TRUE if the current message is the last in the set 
*/

static inline int is_lastmsg(struct sc8800_i2c *i2c)
{
	return i2c->msg_idx >= (i2c->msg_num - 1);
}

/* is_msglast
 *
 * returns TRUE if we this is the last byte in the current message
*/

static inline int is_msglast(struct sc8800_i2c *i2c)
{
	return i2c->msg_ptr == i2c->msg->len-1;
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static inline int is_msgend(struct sc8800_i2c *i2c)
{
	return i2c->msg_ptr >= i2c->msg->len;
}
static inline void sc8800_i2c_read_tx_ack(struct sc8800_i2c *i2c)
{
	unsigned int cmd;
	
	cmd =I2C_CMD_READ | I2C_CMD_STOP | I2C_CMD_TX_ACK;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
}
static inline void sc8800_i2c_read(struct sc8800_i2c *i2c)
{
	unsigned int cmd;
	
	cmd =I2C_CMD_READ;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
}

static int sc8800_i2c_irq_nextbyte(struct sc8800_i2c *i2c,unsigned int cmd_reg)
{
	unsigned char byte;
	unsigned int cmd;
	unsigned long flags;
	int ret = 0;
	
	spin_lock_irqsave(&i2c->lock,flags);
	
	switch (i2c->state) {

	case STATE_IDLE:
		printk("sc8800_i2c_irq_nextbyte error: called in STATE_IDLE\n");
		goto out;
		break;

	case STATE_STOP:
		printk( "sc8800_i2c_irq_nextbyte error: called in STATE_STOP\n");		
		sc8800_i2c_disable_irq(i2c);		
		goto out_icr;

	case STATE_START:
		/* last thing we did was send a start condition on the
		 * bus, or started a new i2c message
		 */
		if (cmd_reg  & I2C_CMD_ACK && !(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
			/* ack was not received... */

			printk("I2C error:ack was not received\n");
		//	i2c->state = STATE_STOP;
		//	sc8800_i2c_complete(i2c,-EREMOTEIO);
		//	sc8800_i2c_disable_irq(i2c);
			sc8800_i2c_stop(i2c,-EREMOTEIO);
			goto out_icr;
		}

		if (i2c->msg->flags & I2C_M_RD)
			i2c->state = STATE_READ;
		else
			i2c->state = STATE_WRITE;

		/* terminate the transfer if there is nothing to do
		 * (used by the i2c probe to find devices */

		if (is_lastmsg(i2c) && i2c->msg->len == 0) {
			printk("detect address finish\n");
			//i2c->state =STATE_STOP;
			//sc8800_i2c_complete(i2c,0);
			//sc8800_i2c_disable_irq(i2c);
			sc8800_i2c_stop(i2c,0);
			goto out_icr;
		}

		if (i2c->state == STATE_READ)
			goto prepare_read;

		/* fall through to the write state, as we will need to 
		 * send a byte as well */

	case STATE_WRITE:
		/* we are writing data to the device... check for the
		 * end of the message, and if so, work out what to do
		 */
		if (!is_msgend(i2c)) {
			if(is_msglast(i2c) && is_lastmsg(i2c)){
				byte = i2c->msg->buf[i2c->msg_ptr++];
				cmd =(byte<<8) | I2C_CMD_WRITE | I2C_CMD_STOP;
				__raw_writel(cmd,i2c->membase+I2C_CMD);

			}
			else
			{
				byte = i2c->msg->buf[i2c->msg_ptr++];
				cmd =(byte<<8) | I2C_CMD_WRITE;
				//printk("w=0x%x\n",cmd);
				__raw_writel(cmd,i2c->membase+I2C_CMD);
			}
			/* delay after writing the byte to allow the
			 * data setup time on the bus, as writing the
			 * data to the register causes the first bit
			 * to appear on SDA, and SCL will change as
			 * soon as the interrupt is acknowledged */

			ndelay(50);

		} else if (!is_lastmsg(i2c)) {
			/* we need to go to the next i2c message */

			i2c->msg_ptr = 0;
			i2c->msg_idx ++;
			i2c->msg++;
			/* send the new start */
			sc8800_i2c_message_start(i2c, i2c->msg);
			i2c->state = STATE_START;
			
		} else {
			/* send stop */
			//printk("I2C write finished\n");   //comment by kewang 
		
			//i2c->state =STATE_STOP;
			//sc8800_i2c_complete(i2c,0);
			//sc8800_i2c_disable_irq(i2c);
			sc8800_i2c_stop(i2c,0);
		}
		break;

	case STATE_READ:
		/* we have a byte of data in the data register, do 
		 * something with it, and then work out wether we are
		 * going to do any more read/write
		 */

		if (!(i2c->msg->flags & I2C_M_IGNORE_NAK) &&
		    !(is_msglast(i2c) && is_lastmsg(i2c))) {

			if (cmd_reg  & I2C_CMD_ACK) {
				printk("I2C READ error: No Ack\n");
				//i2c->state = STATE_STOP;
				//sc8800_i2c_complete(i2c,-ECONNREFUSED);
				//sc8800_i2c_disable_irq(i2c);
				sc8800_i2c_stop(i2c,-ECONNREFUSED);
				goto out_icr;
			}
		}

		cmd = __raw_readl(i2c->membase+I2C_CMD);
		byte = (unsigned char)(cmd>>8);
		i2c->msg->buf[i2c->msg_ptr++] = byte;

	prepare_read:
		if (is_msglast(i2c)) {
			/* last byte of message*/
			if (is_lastmsg(i2c)){
				/*last message of set*/
				sc8800_i2c_read_tx_ack(i2c);
			}
		} else if (is_msgend(i2c)) {
			/* ok, we've read the entire buffer, see if there
			 * is anything else we need to do */

			if (is_lastmsg(i2c)) {
				/* last message, send stop and complete */
				//printk("Read finished\n");    //comment by kewang 
				i2c->state = STATE_STOP;
				sc8800_i2c_complete(i2c,0);
				sc8800_i2c_disable_irq(i2c);
				//sc8800_i2c_stop(i2c,0);
				
			} else {
				/* go to the next transfer */

				i2c->msg_ptr = 0;
				i2c->msg_idx++;
				i2c->msg++;
				sc8800_i2c_read(i2c);
			}
		}
		else{
			sc8800_i2c_read(i2c);
		}
		break;
	}

	/* acknowlegde the IRQ and get back on with the work */
 out_icr:
	cmd=__raw_readl(i2c->membase+I2C_CMD);
	cmd |=I2C_CMD_INT_ACK;   //clear interrupt
	__raw_writel(cmd,i2c->membase+I2C_CMD);
 out:	
	spin_unlock_irqrestore(&i2c->lock,flags);
	
	return ret;
}

/* sc8800_i2c_irq
 *
 * top level IRQ servicing routine
*/
static irqreturn_t sc8800_i2c_irq(unsigned int irq, void *dev_id)
{
	struct sc8800_i2c *i2c;
	unsigned int cmd;
	int ret;
	i2c = (struct sc8800_i2c *)dev_id;

	if (i2c->state == STATE_IDLE) {
		printk( "I2c irq: error i2c->state == IDLE\n");
		cmd=__raw_readl(i2c->membase+I2C_CMD);
		cmd |=I2C_CMD_INT_ACK;   //clear interrupt
		__raw_writel(cmd,i2c->membase+I2C_CMD);	
		
		goto out;
	}	
	
	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent */
#if 1	
    ret = sc8800_i2c_wait_exec(i2c);
	if (ret != 0) {
		printk("I2C busy on exec command!\n");
		goto out;
	}
#endif
#if 0
    uint32_t time=0,time1=0;
    time=__raw_readl(SPRD_TIMER_BASE +0x0024);
    time &= 0x7fffff;
    
    cmd = __raw_readl(i2c->membase + I2C_CMD);
		
	while(cmd & I2C_CMD_BUSY)
        cmd = __raw_readl(i2c->membase + I2C_CMD);  //kewang
    
    time1=__raw_readl(SPRD_TIMER_BASE +0x0024);
    time1 &= 0x7fffff;
    printk("<0x%x-0x%x=0x%x>\n",time1,time,time1-time);
#endif
    	
	cmd =__raw_readl(i2c->membase+I2C_CMD);
	ret=sc8800_i2c_irq_nextbyte(i2c,cmd);
	if(ret!=0)
		printk("I2C irq:error\n");
 out:
	return IRQ_HANDLED;
}

static inline struct sc8800_platform_i2c *sc8800_i2c_get_platformdata(struct device *dev)
{
	if (dev!=NULL && dev->platform_data != NULL)
		return (struct sc8800_platform_i2c *)dev->platform_data;

	return &sc8800_i2c_default_platform;
}
static void set_i2c_clk(struct sc8800_i2c *i2c,unsigned int freq)
{
	unsigned int apb_clk;
	unsigned int i2c_div;
	
	apb_clk=26000000;
	i2c_div=apb_clk/(4*freq)-1;
	
	__raw_writel(i2c_div&0xffff,i2c->membase+I2C_CLKD0);  //div0[0~15]
	__raw_writel(i2c_div>>16,i2c->membase+I2C_CLKD1);   //div1[16~25]
 
}
static void sc8800_i2c_init(struct sc8800_i2c *i2c)
{
	unsigned int tmp;
	struct sc8800_platform_i2c *pdata;
	
//     __raw_bits_or(BIT_3,SPRD_GREG_BASE+0x0028);
//     __raw_bits_and(~BIT_4,SPRD_GREG_BASE+0x0028);
             	
     __raw_bits_or(BIT_4,SPRD_CPC_BASE+0x0304);
     __raw_bits_and(~BIT_5,SPRD_CPC_BASE+0x0304);

     __raw_bits_or(BIT_4,SPRD_CPC_BASE+0x0308);
     __raw_bits_and(~BIT_5,SPRD_CPC_BASE+0x0308);

	tmp=__raw_readl(SPRD_GREG_BASE+0x0008); //global reg:i2c_en
	tmp|=(0x1<<4);
	__raw_writel(tmp,SPRD_GREG_BASE+0x0008);

	__raw_writel(0x1,i2c->membase+I2C_RST);  //reset i2c module
	
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_EN,i2c->membase+I2C_CTL); //disable i2c module then change clock
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_IE,i2c->membase+I2C_CTL);
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_CMDBUF_EN,i2c->membase+I2C_CTL);

	pdata=sc8800_i2c_get_platformdata(i2c->adap.dev.parent);
	set_i2c_clk(i2c,pdata->normal_freq);	//set i2c clock

	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp|I2C_CTL_EN,i2c->membase+I2C_CTL); //enable i2c module

	sc8800_clr_irq(i2c);  //clear i2c interrupt
	
}
//#define res_len(r)		((r)->end - (r)->start + 1)
/* sc8800_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/
static int sc8800_i2c_probe(struct platform_device *pdev)
{
	struct sc8800_i2c *i2c;
	int ret;
	//unsigned int tmp;

	i2c = kzalloc(sizeof(struct sc8800_i2c), GFP_KERNEL);
	if (!i2c){ 
		printk("I2C:kzalloc failed!\n");
		return  -ENOMEM;
	}		
#if 0	
/* map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0){
		printk("I2C:platform_get_resource&irq failed!\n");
		ret=-ENODEV;
		goto err_getirq;
	}
	if (!request_mem_region(res->start, res_len(res), res->name)){
		printk("I2C:request_mem_region failed!\n");
		ret=-ENOMEM;
		goto err_getirq;
	}

	i2c->membase= ioremap(res->start, res_len(res));
	if (!i2c->membase) {
		printk("I2C:ioremap failed!\n");
		ret=-EIO;
		goto err_remap;
	}
#endif
	i2c->membase = (unsigned int *)SPRD_I2C_BASE;
	i2c->memphys = (unsigned int *)SPRD_I2C_PHYS;
	i2c->memsize = (unsigned int *)SPRD_I2C_SIZE;
	i2c->irq = IRQ_I2C_INT;	

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	snprintf(i2c->adap.name,sizeof(i2c->adap.name),"%s","sc8800-i2c");
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.retries = 4;
	i2c->adap.algo = &sc8800_i2c_algorithm;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* initialize the i2c controller */

	sc8800_i2c_init(i2c);
	
	/*tmp=__raw_readl(SPRD_CPC_BASE+0x0098); 
	printk("0x98=0x%x\n",tmp);
	tmp=__raw_readl(SPRD_CPC_BASE+0x009c);  //chip pin control SDA_reg:select SDA
	printk("0x9c=0x%x\n",tmp);
	tmp=__raw_readl(SPRD_GREG_BASE+0x0008); //global reg:i2c_en
	printk("global=0x%x\n",tmp);
	
	printk("membase=0x%x,irq=%d\n",i2c->membase,i2c->irq);	
	tmp=__raw_readl(i2c->membase+I2C_CTL);
	printk("I2C_CTL=0x%x\n",tmp);
	tmp=__raw_readl(i2c->membase+I2C_CMD);
	printk("I2C_CMD=0x%x\n",tmp);
	tmp=__raw_readl(i2c->membase+I2C_CLKD0);
	printk("clkd0=0x%x\n",tmp);
	tmp=__raw_readl(i2c->membase+I2C_CLKD1);
	printk("clkd1=0x%x\n",tmp);
	*/
	ret = request_irq(i2c->irq, sc8800_i2c_irq, IRQF_DISABLED,pdev->name,i2c);
	if (ret) {
		printk("I2C:request_irq failed!\n");
		goto err_irq;
	}
    
    i2c->adap.nr =1;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0){ 
		printk("I2C:add_adapter failed!\n");
		goto err_adap;
	}
	platform_set_drvdata(pdev, i2c);

	return 0;

err_adap:
	free_irq(i2c->irq,i2c);
err_irq:
	kfree(i2c);

	return ret;
}

/* sc8800_i2c_remove
 *
 * called when device is removed from the bus
*/

static int sc8800_i2c_remove(struct platform_device *pdev)
{
	struct sc8800_i2c *i2c = platform_get_drvdata(pdev);
	
	platform_set_drvdata(pdev, NULL);
	
	i2c_del_adapter(&i2c->adap);
	
	free_irq(i2c->irq, i2c);
	
	kfree(i2c);

	return 0;
}

static struct platform_driver sc8800_i2c_driver = {
	.probe		= sc8800_i2c_probe,
	.remove		= sc8800_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sc8800-i2c",
	},
};
static int __init i2c_adap_sc8800_init(void)
{
	printk(KERN_INFO"I2c:sc8800 driver $Revision:1.0 $\n");
	
	return platform_driver_register(&sc8800_i2c_driver);	
}

static void __exit i2c_adap_sc8800_exit(void)
{
	platform_driver_unregister(&sc8800_i2c_driver);
}

module_init(i2c_adap_sc8800_init);
module_exit(i2c_adap_sc8800_exit);

MODULE_DESCRIPTION("SC8800 I2C Bus driver");
MODULE_AUTHOR("Ke Wang, <ke.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");

