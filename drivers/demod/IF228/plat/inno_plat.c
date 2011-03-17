 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
//#include <mach/mux.h>
//#include <mach/control.h>
#include <linux/delay.h>
#include "inno_comm.h"
#include "inno_spi_platform.h"
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/bits.h>


static struct spi_device *g_spi = NULL;
static int       demod_irq = 0xff;
static void *tx_dummy;

#define IF20x_RESET_GPIO     94
#define IF20x_POWER_GPIO   135
#define IF20x_INT_GPIO   93       

//#define IRQ_IF20x_GPIO_INT  __gpio_to_irq(IF20x_INT_GPIO)

#define IF20x_PWR_UP_DELAY         15 //10ms
#define IF20x_PWR_DN_DELAY         1 // 1ms
#define IF20x_RST_DELAY                 20 //10ms

#define SPI_TMOD_DEMOD  (2)

extern __must_check int sprd_alloc_gpio_irq(unsigned gpio);
extern void sprd_spi_tmod(struct spi_device *spi, u32 transfer_mod);


static void inno_platform_power(unsigned char on)
{
	pr_debug("%s\n", __func__);

	if (on)
	{
		__gpio_set_value(IF20x_POWER_GPIO, 1);    
		msleep(1);		

		ANA_REG_OR(ANA_LED_CTL,  0x00004000);

		__gpio_set_value(IF20x_RESET_GPIO, 0);	
		msleep(IF20x_RST_DELAY);
		__gpio_set_value(IF20x_RESET_GPIO, 1);	

		msleep(IF20x_PWR_UP_DELAY);
	}
	else
	{
		__gpio_set_value(IF20x_RESET_GPIO, 0);	
		msleep(IF20x_PWR_DN_DELAY);
		ANA_REG_AND(ANA_LED_CTL,  ~0x00004000);

	}
       
}


static int inno_platform_spi_transfer(struct spi_message *msg)
{
        struct spi_transfer             *t = NULL;
        int i, pos;
        char tmp[100];

        pr_debug("\n");
        pr_debug("%s:++++++++++++++++++++++++++++++++\n", __func__);

        list_for_each_entry(t, &msg->transfers, transfer_list) {
                if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
                        pr_err("%s: tx/rx == NULL, len == 0", __func__);
                        return -EINVAL;
                }
               
                if (t->tx_buf) {
                        pos = sprintf(tmp, "%s:TX:", __func__);
                        for (i = 0; i < t->len && i < 20 ; i++) {
                                pos += sprintf(&tmp[pos], "%02x ", ((unsigned char *)t->tx_buf)[i]);
                        }
                        if (t->len >= 20)
                                sprintf(&tmp[pos], "......");
                        pr_debug("%s\n", tmp);
                } else {
                        t->tx_buf = tx_dummy;
                }
                

                t->bits_per_word = 8;
        }
                

        if (g_spi) {
                spi_sync(g_spi, msg);
        }
        
        list_for_each_entry(t, &msg->transfers, transfer_list) {
                if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
                        pr_err("%s: tx/rx == NULL, len == 0", __func__);
                        return -EINVAL;
                }
                 
                if (t->rx_buf) {
                        pos = sprintf(tmp, "%s:RX:", __func__);
                        for (i = 0; i < t->len && i < 20 ; i++) {
                                pos += sprintf(&tmp[pos], "%02x ", ((unsigned char *)t->rx_buf)[i]);
                        }
                        if (t->len >= 20)
                                sprintf(&tmp[pos], "......");
                        pr_debug("%s\n", tmp);
                }
        }
        pr_debug("%s:--------------------------------\n\n", __func__);
                
        return 0;
}

static irqreturn_t (*irq_handler)(int irqnr, void *devid) = NULL;
static irqreturn_t plat_irq_handler(int irqnr, void *devid)
{
	printk("inno_plat : plat_irq_handler. 0x%x \n", irq_handler);
	
        if (irq_handler)
                return irq_handler(irqnr, devid);
        return IRQ_HANDLED;
}

int inno_platform_init(struct inno_platform *plat)
{       
        int ret = 0;
        pr_debug("%s\n", __func__);

        irq_handler = plat->irq_handler;
        plat->power = inno_platform_power;
        plat->spi_transfer = inno_platform_spi_transfer;
       
        return ret;
}
EXPORT_SYMBOL_GPL(inno_platform_init);

#if 0//ndef   CONFIG_ARCH_SC8800G

static struct pin_config spi_pins[] = {
        /* McSPI 3 */ 
        MUX_CFG_34XX("RESETGPIO", 0x190, OMAP34XX_MUX_MODE4 | OMAP34XX_PIN_OUTPUT)
};

static void spi_pin_config(void)
{       
        int i;  
        for (i = 0; i < ARRAY_SIZE(spi_pins); ++i) {
                omap_ctrl_writew(spi_pins[i].mux_val, spi_pins[i].mux_reg);
        }
}

static void plat_gpio_init(void)
{       
        int ret = 0;
        pr_debug("%s\n", __func__);

        /* config reset gpio */ 
        spi_pin_config();       
        ret = gpio_request(IF20x_RESET_GPIO, "IF20x_RESET_GPIO");       
        if (ret)               
                pr_err("%s: reset gpio request failed\n", __func__);
           

        ret = gpio_direction_output(IF20x_RESET_GPIO, 1);       
        if (ret)             
                pr_err("%s: reset gpio dir set failed\n", __func__);
             
        gpio_set_value(IF20x_RESET_GPIO, 1);

        /* config interrupt gpio */
        ret = gpio_request(IF20x_INT_GPIO, "IF20x_INT_GPIO");   
        if (ret)               
                pr_err("%s: interrupt gpio request failed\n", __func__);
               
        
        ret = gpio_direction_input(IF20x_INT_GPIO);     
        if (ret)               
                pr_err("%s: interrupt gpio dir set failed\n", __func__);
        
}


static void plat_gpio_free(void)
{       
        gpio_free(IF20x_RESET_GPIO);
        gpio_free(IF20x_INT_GPIO);
}

#endif


static int spi_probe(struct spi_device *spi)
{
        int ret = 0;
        pr_debug("%s %s\n",__FILE__, __func__);

        g_spi = spi;
		
	 g_spi->chip_select = 1;
	 g_spi->mode = 0;
	 g_spi->max_speed_hz = 8000000;

        sprd_spi_tmod(g_spi, SPI_TMOD_DEMOD);
    
//        plat_gpio_init();
        
//        irq_handler = NULL;
        demod_irq = sprd_alloc_gpio_irq(IF20x_INT_GPIO);
        
        ret = request_irq(demod_irq, plat_irq_handler, IRQF_TRIGGER_FALLING, "demod", NULL);
        if (ret < 0)
                pr_err("%s:request_irq failed\n", __func__); 

        tx_dummy = kmalloc(0x20000, GFP_KERNEL);
    return 0;
}

static int __devexit spi_remove(struct spi_device *spi)
{
//        plat_gpio_free();
        kfree(tx_dummy);
        pr_debug("%s\n",__func__);
        return 0;
}

static struct spi_driver spi_driver = {
        .probe = spi_probe,
        .remove = __devexit_p(spi_remove),
        .driver = {
                .name = "cmmb-dev",
        },
};

static int __init inno_plat_init(void)
{
        pr_debug("%s\n",__func__);
        spi_register_driver(&spi_driver);
        return 0;
}
static void __exit inno_plat_exit(void)
{
        free_irq(demod_irq, NULL);
        spi_unregister_driver(&spi_driver);
        pr_debug("%s\n",__func__);
}

module_init(inno_plat_init);
module_exit(inno_plat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean.zhao <zhaoguangyu@innofidei.com>");
MODULE_DESCRIPTION("innofidei cmmb platform");
