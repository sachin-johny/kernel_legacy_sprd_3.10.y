
/*
 * innofidei if2xx demod spi communication driver
 * 
 * Copyright (C) 2010 Innofidei Corporation
 * Author:      sean <zhaoguangyu@innofidei.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * 
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>
#include "inno_reg.h"
#include "inno_cmd.h"
#include "inno_core.h"
#include "inno_spi_platform.h"
#include "inno_comm.h"
#include "inno_power.h"
#include "inno_spi.h"


static struct inno_platform plat;



/**
 * inno_spi_sync - translate data need to send/receive to spi_message and send it
 * @data1:first data need to send/receive
 * @len1 :first data len
 * @dir1 :direction (send or receive)
 * @data2:second data need to send/receive
 * @len2 :second data len 
 * @dir2 :direction (send or receive)
 * @cs_change:is cs pin need change between data1 and data2?
 */
int inno_spi_sync(unsigned char *data1, int len1, int dir1,
                unsigned char *data2, int len2, int dir2,
                unsigned char cs_change)
{
        struct spi_message msg;
        struct spi_transfer t[2];
        int ret = 0;

        spi_message_init(&msg);
        memset(t, 0 ,2 * sizeof(struct spi_transfer));
        if (len1) {
                t[0].len = len1;
                t[0].cs_change = cs_change;
                if (dir1 == SPI_TX)
                        t[0].tx_buf = data1;
                else
                        t[0].rx_buf = data1;
                spi_message_add_tail(&t[0], &msg);
        }
        if (len2) {
                t[1].len = len2;
                if (dir2 == SPI_TX)
                        t[1].tx_buf = data2;
                else
                        t[1].rx_buf = data2;
                spi_message_add_tail(&t[1], &msg);
        }
       
        plat.spi_transfer(&msg); 

        return ret;
}

void inno_plat_power(unsigned char on)
{
        plat.power(on);
}

static struct {
        unsigned short fetch_len;
        unsigned short fetch_data;
}cmd_set[8] = {
        [0] = {
                .fetch_len = READ_LG0_LEN,
                .fetch_data = FETCH_LG0_DATA,
        }, 
        [1] = {
                .fetch_len = READ_LG1_LEN,
                .fetch_data = FETCH_LG1_DATA,
        }, 
        [2] = {
                .fetch_len = READ_LG2_LEN,
                .fetch_data = FETCH_LG2_DATA,
        }, 
        [3] = {
                .fetch_len = READ_LG3_LEN,
                .fetch_data = FETCH_LG3_DATA,
        }, 
        [4] = {
                .fetch_len = READ_LG4_LEN,
                .fetch_data = FETCH_LG4_DATA,
        }, 
        [5] = {
                .fetch_len = READ_LG5_LEN,
                .fetch_data = FETCH_LG5_DATA,
        }, 
        [6] = {
                .fetch_len = READ_LG6_LEN,
                .fetch_data = FETCH_LG6_DATA,
        }, 
        [7] = {
                .fetch_len = READ_LG7_LEN,
                .fetch_data = FETCH_LG7_DATA,
        }, 
};

/**
 * inno_lgx_fetch_data -fetch logic channel data
 * @lgx:lgx_device
 *
 * this func is used by inno_lgx.c, when a channel data is comming, 
 * inno_irq_req_handler will notify inno_lgx.c ,
 * then inno_lgx.c fetch data by call inno_lgx_fetch_data
 */
int inno_lgx_fetch_data(struct lgx_device *lgx)
{
        unsigned int    len;
        unsigned char cmd_fetch_len, cmd_fetch_data;
        union {
                unsigned int    len;
                unsigned char   data[4];
        }rsp = {0};

        pr_debug("%s\n", __func__);
        /* get data length need to fetch */
        cmd_fetch_len = cmd_set[lgx->id + 1].fetch_len;
        cmd_fetch_data  = cmd_set[lgx->id + 1].fetch_data;

        inno_spi_sync(&cmd_fetch_len, 1, SPI_TX, rsp.data, 3, SPI_RX, 1);
        len = rsp.len;
        
        pr_debug("%s:lg%d len = 0x%x\n", __func__, lgx->id, len);
        if (len > 0 && len <= lgx->buf_len) {
                inno_spi_sync(&cmd_fetch_data, 1, SPI_TX, lgx->buf, len, SPI_RX, 1);
                lgx->valid = len;
                return 0;
        } 
        return -EINVAL;
}
EXPORT_SYMBOL_GPL(inno_lgx_fetch_data);

#define UAM_BIT_CNT     0x1
#define UAM_BIT_SHIFT   0x8
#define LGX_BIT_CNT     UAM_BIT_CNT + UAM_BIT_SHIFT
int inno_get_intr_ch(unsigned long *ch_bit)
{
        unsigned char cmd[3];
        unsigned char rsp[3]; 
        int             i;

        pr_debug("%s\n", __func__);

        cmd[0] = READ_INT0_STATUS;
        cmd[1] = READ_INT1_STATUS;
        cmd[2] = READ_INT2_STATUS;

        for (i = 0 ; i < 3 ; i++) 
                inno_spi_sync(&cmd[i], 1, SPI_TX, &rsp[i], 1, SPI_RX, 1);

        *ch_bit = (rsp[0] >> 1) + ((rsp[2] & 0x80)? (0x1 << UAM_BIT_SHIFT) : 0);
        return 0;
}


static void packcmd(unsigned int addr,unsigned char cmd, unsigned char* buf)
{
    buf[0] = cmd;
    buf[1] = (addr>>24)&0xff;
    buf[2] = (addr>>16)&0xff;
    buf[3] = (addr>>8)&0xff;
    buf[4] = (addr)&0xff;
}

/**
 * inno_comm_send_unit - send data to addr
 * @addr:target address data will be sent
 * @buf :data
 * @len :len
 */
int inno_comm_send_unit(unsigned int addr, unsigned char *buf, int len)
{
        unsigned char cmd[5];
        packcmd(addr, WRITE_AHBM2, cmd);
        inno_spi_sync(cmd, 5, SPI_TX, buf, len, SPI_TX, 0);
        return 0;
}
EXPORT_SYMBOL_GPL(inno_comm_send_unit);

/**
 * inno_comm_get_unit - get data from addr
 * @addr:target address data will read from
 * @buf :data receive buf
 * @len :len need to read
 */
int inno_comm_get_unit(unsigned int addr, unsigned char *buf, int len)
{
        unsigned char *tmp;
        unsigned char cmd[5];
        int i;
        packcmd(addr, READ_AHBM2, cmd);
        tmp = kzalloc(len + 1, GFP_KERNEL | GFP_DMA);
        inno_spi_sync(cmd, 5, SPI_TX, tmp, len + 1, SPI_RX, 0);
        for(i = 0; i < len; i++) {
                buf[i] = tmp[i+1];
        }
        kfree(tmp);
        return 0;        

}
EXPORT_SYMBOL_GPL(inno_comm_get_unit);

int inno_comm_init(void)
{
        plat.irq_handler = (irq_handler_t)inno_demod_irq_handler;
        inno_platform_init(&plat); 
        return 0;
}

