#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>

#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "trout_fm_ctrl.h"
#include "trout_interface.h"

#define	TROUT_FM_SPI_WRITE(addr, data)	((0UL<<31)|((addr&0x7f)<<24)|((data&0xffff)<<8))
#define	TROUT_FM_SPI_READ(addr)		((1UL<<31)|((addr&0x7f)<<24))

struct spidev_data
{
	struct spi_device *spi;
	struct mutex buf_lock;
	u8  *buffer;
};

struct spidev_data  g_spi_data;

int spitrout_init(struct spidev_data *spidata)
{

	spidata->buffer = kzalloc(128, GFP_KERNEL);
	if(!spidata->buffer)
	{
		return -ENOMEM;
	}

	mutex_init(&spidata->buf_lock);

	spidata->spi->max_speed_hz = SPI_MAX_SPEED;
	spidata->spi->bits_per_word = 32;
	spi_setup(spidata->spi);

	return 0;
}

int spitrout_exit(struct spidev_data *spidata)
{
	if(spidata != NULL)
	{
		if(spidata->buffer)
			kfree(spidata->buffer);
	}

	return 0;
}

int spi_trout_probe(struct spi_device *spi)
{
	int ret = 0;
	g_spi_data.spi = spi;

	//set spi speed and bitwidth.
	spitrout_init(&g_spi_data);
	dev_set_drvdata(&spi->dev, &g_spi_data);

	return ret;
}

int __devexit spi_trout_remove(struct spi_device *spi)
{
    spitrout_exit(&g_spi_data);

	return 0;
}

int spitrout_message (struct spi_device *spi, struct spi_transfer *k_xfers,
		unsigned int n_xfers,u8 trans_mode)
{
	struct spi_message	msg;
	struct spi_transfer	*k_tmp;
	int		status  =  -EFAULT;
	int     i;
	int     total =0;

	spi_message_init(&msg);

	/* Construct spi_message   */
	k_tmp = k_xfers;    

	for (i = 0 ; i < n_xfers ; i++)
	{
		spi_message_add_tail (k_tmp, &msg);
		k_tmp ++;
	}

	status  =  spi_sync(spi,  &msg);

	if (status < 0)
		goto done;

	k_tmp = k_xfers;    
	for (i = 0 ; i < n_xfers ; i++)
	{
		if (k_tmp ->rx_buf  != NULL) 
		{
			total += k_tmp->len;
		}
		k_tmp ++;
	}
	status = total;

done:
	return status;

}

unsigned int trout_spi_write (u32 reg_addr,  u32 val)
{
	u32 *buf, sysflag = 0;
	u32 ret;
	struct spi_device *spidev;
	struct spidev_data  *spidata = &g_spi_data;
	struct spi_transfer trout_xfers;

	spidev = spidata->spi;
	memset(&trout_xfers, 0, sizeof(struct spi_transfer));

	//convert register offset from 4 to 1.
	mutex_lock(&spidata->buf_lock);
	buf = (u32  *)spidata->buffer;
	*buf++ = SPI_WR_ERG_CMD(1,reg_addr, sysflag);
	*buf = val;
	trout_xfers.tx_buf  = (u8 *)spidata->buffer;
	trout_xfers.rx_buf  = NULL;
	trout_xfers.len = 8;

	ret = spitrout_message(spidev, &trout_xfers, 1, 0);
	mutex_unlock(&spidata->buf_lock);

	return ret;
}

unsigned int trout_spi_read (u32 reg_addr, u32 *reg_data)
{
	int ret;
	u32  *buf, sysflag = 0;
	struct spi_device   *spidev;
	struct spidev_data    *spidata = &g_spi_data;
	struct spi_transfer trout_xfers[2];
	struct spi_transfer *tmp_xfers;

	spidev = spidata->spi;
	memset(trout_xfers, 0, sizeof(struct spi_transfer) * 2);
	tmp_xfers = trout_xfers;
	mutex_lock( &spidata->buf_lock);

	buf = (u32 *)spidata->buffer;
	*buf = SPI_RD_ERG_CMD(1, reg_addr, sysflag);
	tmp_xfers->tx_buf  = buf;
	tmp_xfers->rx_buf  = NULL;
	tmp_xfers->len = 4;

	buf++;
	tmp_xfers++;
	tmp_xfers->tx_buf  =  NULL;
	tmp_xfers->rx_buf  = buf;
	tmp_xfers->len = 4;

	ret = spitrout_message(spidev, trout_xfers, 2, 0);
	if ( ret > 0)
	{
		*reg_data = *buf;
		TROUT_PRINT("regv: 0x%x\n", ret);
	}

	mutex_unlock(&spidata->buf_lock);

	return ret;
}

struct spi_driver sprd_trout_driver = {
	.driver = {
		.name   = TROUT_FM_DEV_NAME,
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
	.probe      = spi_trout_probe,
	.remove     = __devexit_p(spi_trout_remove),
};

unsigned int spi_init(void)
{
    int ret;
    
    ret = spi_register_driver(&sprd_trout_driver);
	TROUT_PRINT("trout init: ret=%d\n", ret);

    return ret;
}

unsigned int spi_exit(void)
{
    spi_unregister_driver(&sprd_trout_driver);
    
    return 0;
}

static trout_interface_t spi_interface = 
{
    .name = "spi",
    .init = spi_init,
    .exit = spi_exit,
    .read_reg = trout_spi_read,
    .write_reg = trout_spi_write,
};

int trout_spi_init(trout_interface_t **p)
{
    *p = &spi_interface;
    
    return 0;
}
