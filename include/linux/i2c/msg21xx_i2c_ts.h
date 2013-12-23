
/********************************************************************************
 *                                                                           								*
 *					Focaltech Systems (R)										*
 *                                                                           								*
 *                               All Rights Reserved                     							*
 *                                                                        								*
 *  THIS WORK CONTAINS TRADE SECRET AND PROPRIETARY INFORMATION WHICH IS      	*
 *  THE PROPERTY OF MENTOR GRAPHICS CORPORATION OR ITS LICENSORS AND IS       	*
 *  SUBJECT TO LICENSE TERMS.                               				    				*
 *                                                                           								*
 *******************************************************************************/
 

#ifndef __MSG2133_LINUX_H
#define __MSG2133_LINUX_H
#define TS_IRQ_PIN			"ts_irq_pin"
#define TS_RESET_PIN		"ts_rst_pin"

#define MSG21XX_TS_DEVICE	"ms-msg21xx"
#define MSG21XX_TS_ADDR		0x26

struct i2c_dev{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

struct msg_ts_platform_data {
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
};

struct point_node_t{
	unsigned char 	active ;
	unsigned char	finger_id;
	int	posx;
	int	posy;
};




#endif /*  __MSG2133_LINUX_H */


