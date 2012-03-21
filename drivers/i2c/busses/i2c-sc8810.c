/* linux/drivers/i2c/busses/i2c-sc8810.c
 *
 * Copyright (C) 2010 Spreadtrum
 *	
 *
 * SC8810 I2C Controller
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
#include <linux/slab.h>

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
#define I2C_CTL_BUSY		             	(1 << 2)        //I2c data line value
#define I2C_CTL_IE               		(1 << 3)        //I2c interrupt enable
#define I2C_CTL_EN               		(1 << 4)        //I2c module enable
#define I2C_CTL_CMDBUF_EN       	 	(1 << 5)        //Enable the cmd buffer mode
#define I2C_CTL_CMDBUF_EXEC    			(1 << 6)        //Start to exec the cmd in the cmd buffer
#define I2C_CTL_ST_CMDBUF	        	(7 << 7)        //The state of  I2c cmd buffer state machine.
#define I2C_CTL_CMDBUF_WPTR			(7 << 10)        //I2c command buffer write pointer

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
enum sc8810_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};
struct sc8810_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;
	
	struct i2c_adapter	adap;

	enum sc8810_i2c_state	state;

	int 			irq;
	void __iomem		*membase;	
	unsigned int 		*memphys;
	unsigned int  		*memsize;
};
struct sc8810_platform_i2c {
	unsigned int	normal_freq;	/* normal bus frequency */
	unsigned int	fast_freq;	/* fast frequency for the bus */
	unsigned int	min_freq;	/* min frequency for the bus */
};
static struct sc8810_platform_i2c sc8810_i2c_default_platform = {
	.normal_freq	= 100*1000,
	.fast_freq	= 400*1000,
	.min_freq	= 10*1000,
};
static struct sc8810_i2c *sc8810_i2c_ctl[4];
static void sc8810_i2c_ctl1_reset(struct sc8810_i2c *i2c);
static inline struct sc8810_platform_i2c *sc8810_i2c_get_platformdata(struct device *dev);
static void set_i2c_clk(struct sc8810_i2c *i2c,unsigned int freq);
static ssize_t i2c_reset(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len);

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, NULL, i2c_reset);

static ssize_t i2c_reset(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	printk("%s\n",__func__);
	sc8810_i2c_ctl1_reset(sc8810_i2c_ctl[1]);
	return len;
}

static int i2c_create_sysfs(struct platform_device *pdev)
{
	int err;
	struct device *dev = &(pdev->dev);
	
	err = device_create_file(dev, &dev_attr_reset);

	return err;
}



/* sc8810_i2c_wait_exec
 *
 * wait i2c cmd executable
*/
static inline int sc8810_i2c_wait_exec(struct sc8810_i2c *i2c)
{
	unsigned int cmd;
	int timeout = 2000;  //modify by kewang

	while (timeout-- > 0) {
		cmd = __raw_readl(i2c->membase + I2C_CMD);
		if (!(cmd & I2C_CMD_BUSY))
			return 0;
	}
	printk( "I2C:timeout,busy in exec commands !\n");

	return -ETIMEDOUT;
}
static inline void sc8810_i2c_disable_irq(struct sc8810_i2c *i2c)
{
	unsigned int ctl;
	ctl=__raw_readl(i2c->membase+I2C_CTL); 
	 //disable i2c interrupt
	__raw_writel(ctl &~I2C_CTL_IE,i2c->membase+I2C_CTL);	
}
static inline void sc8810_clr_irq(struct sc8810_i2c *i2c)
{
	unsigned int cmd;
	
	cmd=__raw_readl(i2c->membase+I2C_CMD); 
	//clear interrupt
	__raw_writel(cmd | I2C_CMD_INT_ACK,i2c->membase+I2C_CMD);
}
static inline void sc8810_i2c_enable_irq(struct sc8810_i2c *i2c)
{
	unsigned int ctl;
	
	ctl=__raw_readl(i2c->membase+I2C_CTL); 
	 //enable i2c interrupt
	__raw_writel(ctl | I2C_CTL_IE,i2c->membase+I2C_CTL);	
	
}
/* sc8810_i2c_message_start
 *
 * put the start bit & slave address of a message onto the bus 
*/
static void sc8810_i2c_message_start(struct sc8810_i2c *i2c, struct i2c_msg *msg)
{
	unsigned int cmd;

	cmd = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD) 
		cmd |= 0x1;
    
	cmd=(cmd<<8) | I2C_CMD_START | I2C_CMD_WRITE;
	__raw_writel(cmd, i2c->membase + I2C_CMD);
}

static void sc8810_i2c_ctl1_reset(struct sc8810_i2c *i2c)
{
	unsigned int tmp;
	struct sc8810_platform_i2c *pdata;

	tmp=__raw_readl(SPRD_GREG_BASE+0x004c); //global reg:i2c_en
	tmp|=(0x1<<3);
	__raw_writel(tmp,SPRD_GREG_BASE+0x004c);
	tmp&=~(0x1<<3);
	__raw_writel(tmp,SPRD_GREG_BASE+0x004c);

	
	__raw_writel(0x1,i2c->membase+I2C_RST);  //reset i2c module
	
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_EN,i2c->membase+I2C_CTL); //disable i2c module then change clock
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_IE,i2c->membase+I2C_CTL);
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_CMDBUF_EN,i2c->membase+I2C_CTL);

	pdata=sc8810_i2c_get_platformdata(i2c->adap.dev.parent);
	set_i2c_clk(i2c,pdata->normal_freq);	//set i2c clock
	printk("[I2C:sc8810_i2c_init:i2c clk=%d]",pdata->normal_freq);

	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp|I2C_CTL_EN,i2c->membase+I2C_CTL); //enable i2c module

	sc8810_clr_irq(i2c);  //clear i2c interrupt
	
}


/* sc8810_i2c_doxfer
 *
 * this starts an i2c transfer
*/
static int sc8810_i2c_doxfer(struct sc8810_i2c *i2c, struct i2c_msg *msgs, int num)
{
	unsigned int timeout;
	unsigned long  flags;
	int ret;
	
	ret = sc8810_i2c_wait_exec(i2c);
	if (ret != 0) {
		ret = -EAGAIN;
		goto out;
	}
 
	spin_lock_irqsave(&i2c->lock,flags);

	i2c->msg = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->state = STATE_START;

	sc8810_clr_irq(i2c);
	sc8810_i2c_enable_irq(i2c);
	sc8810_i2c_message_start(i2c, msgs);
	
	spin_unlock_irqrestore(&i2c->lock,flags);

	timeout=wait_event_timeout(i2c->wait, i2c->msg_num == 0, 5*HZ);
	
	ret = i2c->msg_idx;
	/* having these next two as dev_err() makes life very 
	 * noisy when doing an i2cdetect */
	if (timeout == 0){
#define INTCV_REG(off) (SPRD_INTCV_BASE + (off))		
#define INTCV_IRQ_STS     INTCV_REG(0x0000)
#define INTCV_INT_RAW     INTCV_REG(0x0004)
#define INTCV_INT_EN      INTCV_REG(0x0008)	/* 1: enable, 0: disable */		
		printk("I2C:timeout!!!!!!!!!!!!!!!!!!!!!!!!!!!!, i2c msg num = %d, i2c state =%d, debug_msg_num = %d \n", i2c->msg_num, i2c->state,num);       		
		printk("msg_ptr = %d, msg_idx = %d\n", i2c->msg_ptr, i2c->msg_idx);
		printk("i2c intraw =0x%x, intMak =0x%x, intenable =0x%x,i2c_trl =0x%x, i2c_cmd=0x%x\n", __raw_readl(INTCV_INT_RAW), __raw_readl(INTCV_IRQ_STS),__raw_readl(INTCV_IRQ_STS + 0x8), __raw_readl(i2c->membase+I2C_CTL), __raw_readl(i2c->membase+I2C_CMD));
		sc8810_i2c_disable_irq(i2c);
		sc8810_clr_irq(i2c);
		//__raw_writel(0x1,i2c->membase+I2C_RST);  //reset i2c module
		sc8810_i2c_ctl1_reset(i2c);
		__raw_writel(1 << 14 | (1 << 11), INTCV_INT_EN);
		
		ret = -ENXIO;
	}else if (ret != num){
		printk("incomplete xfer (%d)\n", ret);
		ret = -EAGAIN;
	}

	/* ensure the stop has been through the bus */
	//msleep(5);

 out:    
	return ret;
}

static int sc8810_i2c_xfer(struct i2c_adapter *adap,struct i2c_msg *msgs, int num)
{
	struct sc8810_i2c *i2c = (struct sc8810_i2c *)adap->algo_data;
	int retry;
	int ret;

//        printk("I2C: num=%d,addr=%x, len=%d,data=%s", num,msgs->addr, msgs->len, msgs->buf);
	for (retry = 0; retry < adap->retries; retry++) {
        if (retry > 0)
		    printk("I2C:Retrying transmission (%d)\n", retry);

		ret = sc8810_i2c_doxfer(i2c, msgs, num);

		if (ret != -EAGAIN)
			return ret;
		//sc8810_i2c_ctl1_reset(i2c);

		udelay(100);
	}
	printk("I2C:transmission failed!\n");
	return -EREMOTEIO;
}
/* declare our i2c functionality */
static unsigned int sc8810_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL ;
}
/* i2c bus registration info */

static const struct i2c_algorithm sc8810_i2c_algorithm = {
	.master_xfer		= sc8810_i2c_xfer,
	.functionality		= sc8810_i2c_func,
};
static inline void sc8810_i2c_complete(struct sc8810_i2c *i2c, int ret)
{
//	printk("I2C:sc8810_i2c_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	wake_up(&i2c->wait);
}
static inline void sc8810_i2c_stop_only(struct sc8810_i2c *i2c, int ret)
{
	i2c->state = STATE_STOP;

	sc8810_i2c_complete(i2c, ret);
	sc8810_i2c_disable_irq(i2c);
}
static inline void sc8810_i2c_stop(struct sc8810_i2c *i2c, int ret)
{
	unsigned int cmd;

	printk("I2C:sc8810_i2c_stop!\n");

	/* stop the transfer */
	cmd= I2C_CMD_STOP | I2C_CMD_WRITE;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
	
	i2c->state = STATE_STOP;
	
	sc8810_i2c_complete(i2c, ret);
	sc8810_i2c_disable_irq(i2c);
}

/* is_lastmsg()
 *
 * returns TRUE if the current message is the last in the set 
*/

static inline int is_lastmsg(struct sc8810_i2c *i2c)
{
	return i2c->msg_idx >= (i2c->msg_num - 1);
}

/* is_msglast
 *
 * returns TRUE if we this is the last byte in the current message
*/

static inline int is_msglast(struct sc8810_i2c *i2c)
{
	return i2c->msg_ptr == i2c->msg->len-1;
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static inline int is_msgend(struct sc8810_i2c *i2c)
{
	return i2c->msg_ptr >= i2c->msg->len;
}
static inline void sc8810_i2c_read_tx_ack(struct sc8810_i2c *i2c)
{
	unsigned int cmd;
	
	cmd =I2C_CMD_READ | I2C_CMD_STOP | I2C_CMD_TX_ACK;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
}
static inline void sc8810_i2c_read(struct sc8810_i2c *i2c)
{
	unsigned int cmd;
	
	cmd =I2C_CMD_READ;
	__raw_writel(cmd,i2c->membase+I2C_CMD);
}

static int sc8810_i2c_irq_nextbyte(struct sc8810_i2c *i2c,unsigned int cmd_reg)
{
	unsigned char byte;
	unsigned int cmd;
	unsigned long flags;
	int ret = 0;
	
	spin_lock_irqsave(&i2c->lock,flags);
	
	switch (i2c->state) {

	case STATE_IDLE:
		printk("sc8810_i2c_irq_nextbyte error: called in STATE_IDLE\n");
		goto out;
		break;

	case STATE_STOP:
		printk( "sc8810_i2c_irq_nextbyte error: called in STATE_STOP\n");		
		sc8810_i2c_disable_irq(i2c);		
		goto out_icr;

	case STATE_START:
		/* last thing we did was send a start condition on the
		 * bus, or started a new i2c message
		 */
		if (cmd_reg  & I2C_CMD_ACK && !(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
			/* ack was not received... */

			printk("I2C error:ack was not received\n");
		//	i2c->state = STATE_STOP;
		//	sc8810_i2c_complete(i2c,-EREMOTEIO);
		//	sc8810_i2c_disable_irq(i2c);
			sc8810_i2c_stop(i2c,-EREMOTEIO);
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
			//sc8810_i2c_complete(i2c,0);
			//sc8810_i2c_disable_irq(i2c);
			sc8810_i2c_stop(i2c,0);
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

		} else if (!is_lastmsg(i2c)) {
			/* we need to go to the next i2c message */

			i2c->msg_ptr = 0;
			i2c->msg_idx ++;
			i2c->msg++;
			/* send the new start */
			sc8810_i2c_message_start(i2c, i2c->msg);
			i2c->state = STATE_START;
			
		} else {
			/* send stop */
			//printk("I2C write finished\n");   //comment by kewang 
		
			//i2c->state =STATE_STOP;
			//sc8810_i2c_complete(i2c,0);
			//sc8810_i2c_disable_irq(i2c);
			sc8810_i2c_stop_only(i2c,0);
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
				//sc8810_i2c_complete(i2c,-ECONNREFUSED);
				//sc8810_i2c_disable_irq(i2c);
				sc8810_i2c_stop(i2c,-ECONNREFUSED);
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
				sc8810_i2c_read_tx_ack(i2c);
			}
		} else if (is_msgend(i2c)) {
			/* ok, we've read the entire buffer, see if there
			 * is anything else we need to do */

			if (is_lastmsg(i2c)) {
				/* last message, send stop and complete */
				//printk("Read finished\n");    //comment by kewang 
				i2c->state = STATE_STOP;
				sc8810_i2c_complete(i2c,0);
				sc8810_i2c_disable_irq(i2c);
				//sc8810_i2c_stop(i2c,0);
				
			} else {
				/* go to the next transfer */

				i2c->msg_ptr = 0;
				i2c->msg_idx++;
				i2c->msg++;
				sc8810_i2c_read(i2c);
			}
		}
		else{
			sc8810_i2c_read(i2c);
		}
		break;
	}

	/* acknowlegde the IRQ and get back on with the work */
 out_icr:
#if 0
	cmd=__raw_readl(i2c->membase+I2C_CMD);
	cmd |=I2C_CMD_INT_ACK;   //clear interrupt
	__raw_writel(cmd,i2c->membase+I2C_CMD);
#endif
 out:	
	spin_unlock_irqrestore(&i2c->lock,flags);
	
	return ret;
}

/* sc8810_i2c_irq
 *
 * top level IRQ servicing routine
*/
static irqreturn_t sc8810_i2c_irq(unsigned int irq, void *dev_id)
{
	struct sc8810_i2c *i2c;
	unsigned int cmd;
	unsigned int ctl;
	int ret;
	i2c = (struct sc8810_i2c *)dev_id;
		
	ctl = __raw_readl(i2c->membase+I2C_CTL);
	if (!(ctl & I2C_CTL_INT)) {
#if  0//only for debug
		unsigned int debug_irq_status = 0;
		debug_irq_status = __raw_readl(SPRD_I2C0_BASE+I2C_CTL);
		debug_irq_status |= __raw_readl(SPRD_I2C1_BASE+I2C_CTL);		
		debug_irq_status |= __raw_readl(SPRD_I2C2_BASE+I2C_CTL);
		debug_irq_status |= __raw_readl(SPRD_I2C3_BASE+I2C_CTL);
		if (!(debug_irq_status & I2C_CTL_INT)) {
				printk("I2C:sc8810_i2c_irq, NOT FOUND Or had handled!!!\n");
		}
#endif
		return IRQ_NONE;
	}
	sc8810_clr_irq(i2c);
 
	if (i2c->state == STATE_IDLE) {
		printk( "I2c irq: error i2c->state == IDLE\n");
		goto out;
	}	
	
	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent */
    	ret = sc8810_i2c_wait_exec(i2c);
	if (ret != 0) {
		printk("I2C busy on exec command!\n");
		goto out;
	}
    	
	cmd =__raw_readl(i2c->membase+I2C_CMD);
	ret=sc8810_i2c_irq_nextbyte(i2c,cmd);
	if(ret!=0)
		printk("I2C irq:error\n");
 out:
	return IRQ_HANDLED;
}

static inline struct sc8810_platform_i2c *sc8810_i2c_get_platformdata(struct device *dev)
{
	if (dev!=NULL && dev->platform_data != NULL)
		return (struct sc8810_platform_i2c *)dev->platform_data;

	return &sc8810_i2c_default_platform;
}
static void set_i2c_clk(struct sc8810_i2c *i2c,unsigned int freq)
{
	unsigned int apb_clk;
	unsigned int i2c_div;
	
	apb_clk=26000000;
	i2c_div=apb_clk/(4*freq)-1;
	
	__raw_writel(i2c_div&0xffff,i2c->membase+I2C_CLKD0);  //div0[0~15]
	__raw_writel(i2c_div>>16,i2c->membase+I2C_CLKD1);   //div1[16~25]
 
}
void sc8810_i2c_set_clk(unsigned int id_nr, unsigned int freq)
{
  //  set_i2c_clk(sc8810_i2c_ctl[id_nr], freq);
  	unsigned int tmp;
	
	tmp=__raw_readl(sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); 
	__raw_writel(tmp&(~I2C_CTL_EN),sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); //disable i2c module
	tmp=__raw_readl(sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); 

	printk("I2C:sc8810_i2c_set_clk: disable i2c%d, i2c ctl:%x\n", id_nr, tmp);
    	set_i2c_clk(sc8810_i2c_ctl[id_nr], freq);

	tmp=__raw_readl(sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); 
	__raw_writel(tmp|I2C_CTL_EN,sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); //enable i2c module	
	tmp=__raw_readl(sc8810_i2c_ctl[id_nr]->membase+I2C_CTL); 	
	printk("I2C:sc8810_i2c_set_clk: enable i2c%d, i2c ctl:%x\n", id_nr, tmp);
}
EXPORT_SYMBOL_GPL(sc8810_i2c_set_clk);

static void sc8810_i2c_init(struct sc8810_i2c *i2c)
{
	unsigned int tmp;
	struct sc8810_platform_i2c *pdata;
	
	tmp=__raw_readl(SPRD_GREG_BASE+0x0008); //global reg:i2c_en
	tmp|=((0x7<<29)|(0x1<<4));
	__raw_writel(tmp,SPRD_GREG_BASE+0x0008);

	tmp=__raw_readl(SPRD_GREG_BASE+0x004c); //global reg:i2c_en
	tmp|=((0x7<<2)|(0x1<<0));
	__raw_writel(tmp,SPRD_GREG_BASE+0x004c);
	tmp&=~((0x7<<2)|(0x1<<0));
	__raw_writel(tmp,SPRD_GREG_BASE+0x004c);

	__raw_writel(0x1,i2c->membase+I2C_RST);  //reset i2c module
	
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_EN,i2c->membase+I2C_CTL); //disable i2c module then change clock
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_IE,i2c->membase+I2C_CTL);
	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp &~ I2C_CTL_CMDBUF_EN,i2c->membase+I2C_CTL);

	pdata=sc8810_i2c_get_platformdata(i2c->adap.dev.parent);
	set_i2c_clk(i2c,pdata->normal_freq);	//set i2c clock
	printk("[I2C:sc8810_i2c_init:i2c clk=%d]",pdata->normal_freq);

	tmp=__raw_readl(i2c->membase+I2C_CTL); 
	__raw_writel(tmp|I2C_CTL_EN,i2c->membase+I2C_CTL); //enable i2c module

	sc8810_clr_irq(i2c);  //clear i2c interrupt
	
}
/* sc8810_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/
static int sc8810_i2c_probe(struct platform_device *pdev)
{
	struct sc8810_i2c *i2c;
	struct resource *res;
	int irq;
	int ret;
	unsigned int tmp;

	i2c = kzalloc(sizeof(struct sc8810_i2c), GFP_KERNEL);
	if (!i2c){ 
		printk("I2C:kzalloc failed!\n");
		return  -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	printk("I2C: res->start=%x\n", res->start);
	irq = platform_get_irq(pdev, 0);
	printk("I2C: irq=%d\n", irq);
	if (res == NULL || irq < 0){
		printk("I2C:platform_get_resource&irq failed!\n");
		ret=-ENODEV;
		goto err_irq;
	}
#if 0
	if (!request_mem_region(res->start, res_len(res), res->name)){
		printk("I2C:request_mem_region failed!\n");
		ret=-ENOMEM;
		goto err_irq;
	}
#endif

	i2c->membase= res->start;//ioremap(res->start, resource_size(res));
	if (!i2c->membase) {
		printk("I2C:ioremap failed!\n");
		ret=-EIO;
		goto err_irq;
	}

	i2c->irq = irq;	

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	snprintf(i2c->adap.name,sizeof(i2c->adap.name),"%s","sc8810-i2c");
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.retries = 4;
	i2c->adap.algo = &sc8810_i2c_algorithm;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* initialize the i2c controller */

	sc8810_i2c_init(i2c);
	
	ret = request_irq(i2c->irq, sc8810_i2c_irq, IRQF_SHARED,pdev->name,i2c);
	if (ret) {
		printk("I2C:request_irq failed!\n");
		goto err_irq;
	}
    
	i2c->adap.nr = pdev->id;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0){ 
		printk("I2C:add_adapter failed!\n");
		goto err_adap;
	}

    sc8810_i2c_ctl[pdev->id] = i2c;
    printk("I2C:sc8810_i2c_ctl[%d]=%x",pdev->id, sc8810_i2c_ctl[pdev->id]);
	platform_set_drvdata(pdev, i2c);

	i2c_create_sysfs(pdev);

	return 0;

err_adap:
	free_irq(i2c->irq,i2c);
err_irq:
	kfree(i2c);

	return ret;
}

/* sc8810_i2c_remove
 *
 * called when device is removed from the bus
*/

static int sc8810_i2c_remove(struct platform_device *pdev)
{
	struct sc8810_i2c *i2c = platform_get_drvdata(pdev);
	
	platform_set_drvdata(pdev, NULL);
	
	i2c_del_adapter(&i2c->adap);
	
	free_irq(i2c->irq, i2c);
	
	kfree(i2c);
	printk("[I2C:sc8810_i2c_remove]");

	return 0;
}

static struct platform_driver sc8810_i2c_driver = {
	.probe		= sc8810_i2c_probe,
	.remove		= sc8810_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sc8810-i2c",
	},
};
static int __init i2c_adap_sc8810_init(void)
{
	printk(KERN_INFO"I2c:sc8810 driver $Revision:1.0 $\n");
	
	return platform_driver_register(&sc8810_i2c_driver);	
}

static void __exit i2c_adap_sc8810_exit(void)
{
	platform_driver_unregister(&sc8810_i2c_driver);
}

module_init(i2c_adap_sc8810_init);
module_exit(i2c_adap_sc8810_exit);

MODULE_DESCRIPTION("SC8810 I2C Bus driver");
MODULE_AUTHOR("hao liu, <hao.liu@spreadtrum.com>");
MODULE_LICENSE("GPL");

