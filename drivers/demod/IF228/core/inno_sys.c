
/*
 * innofidei if2xx demod sysfs driver
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
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include "inno_core.h"
#include "inno_ctl.h"
#include "inno_sys.h"
#include "inno_power.h"
#include "inno_comm.h"
#include "inno_demod.h"
#include "inno_io.h"
#include "inno_spi.h"


static ssize_t
freq_store(struct class *class, const char *buf, size_t count)
{
        unsigned int freq;
        struct inno_req req;
       
        if (count) {
                freq = simple_strtol(buf, NULL, 10);
                req.handler = set_frequency;
                req.context = &freq;
                inno_req_sync(&req);
        }
        return count;
}

static ssize_t
freq_show(struct class *class, char *buf)
{
        struct inno_req req;
        unsigned char freq_dot;        
        int ret = 0;
 
        req.handler = get_frequency;
        req.context = &freq_dot;
        ret = inno_req_sync(&req);
        if (ret < 0)
                return sprintf(buf, "Device transfer error, maybe power is lost\n");
        return sprintf(buf, "%d\n", freq_dot);

}

static ssize_t
fw_version_show(struct class *class, char *buf)
{
        struct inno_req req;
        unsigned int version;        
        int ret;
        
        req.handler = get_fw_version;
        req.context = &version;
        ret = inno_req_sync(&req);
        if (ret < 0)
                return sprintf(buf, "Device transfer error, maybe power is lost\n");
        return sprintf(buf, "%x\n", version);
}

static ssize_t
power_show(struct class *class, char *buf)
{
        if (POWER_OFF == inno_demod_power_status())
                return sprintf(buf, "off\n");
        else
                return sprintf(buf, "on\n");
}

static ssize_t
power_store(struct class *class, const char *buf, size_t count)
{
        unsigned char power;

        if (count) {
                power = simple_strtol(buf, NULL, 10);
                if (power)
                        inno_demod_inc_ref();
                else
                        inno_demod_dec_ref();
        }
        return count;
}

static ssize_t
ch_conf_store(struct class *class, const char *buf, size_t count)
{
        struct inno_req req;
        struct ch_config config;
        char *p = (char *)buf;
        char *p1;

        if (count) {
                while (*p == ' ')
                        p++;
                config.ch_id = simple_strtol(p, &p1, 10);
                if (p1 == p)
                        goto out;

                p = p1;
                while (*p == ' ')
                        p++;
                config.start_timeslot = simple_strtol(p, &p1, 10);
                if (p1 == p)
                        goto out;

                p = p1;
                while (*p == ' ')
                        p++;
                config.timeslot_count = simple_strtol(p, &p1, 10);
                if (p1 == p)
                        goto out;

                p = p1;
                while (*p == ' ')
                        p++;
                config.demod_config = simple_strtol(p, &p1, 16);
                if (p1 == p)
                        goto out;

                req.handler = set_ch_config;
                req.context = &config;
                inno_req_sync(&req);
        }
out:
        return count;
}

extern struct bus_type inno_lgx_bus_type;

struct ch_conf_result{
        char *buf;
        int len;
};

static int lgx_ch_conf_show(struct device *dev, void *data)
{
        struct inno_req req;
        struct ch_config config;
        char name[] = LG_PREFIX;
        struct lgx_device *lgx_dev;
        struct ch_conf_result *result = (struct ch_conf_result *)data;
        int ret = 0;

        lgx_dev = container_of(dev, struct lgx_device, dev);
        if (strcmp(name, lgx_dev->name) == 0)
        {
                req.handler = get_ch_config;
                config.ch_id = lgx_dev->id;
                req.context = &config;
                ret = inno_req_sync(&req);
                if (ret < 0)
                        result->len = sprintf(result->buf, "Device transfer error, maybe power is lost\n");
                else
                        result->len += sprintf(result->buf + result->len,
                                        "ch%d: start = %d count = %d demod = 0x%x\n",
                                        config.ch_id, 
                                        config.start_timeslot,
                                        config.timeslot_count,
                                        config.demod_config);
        }
        return ret;
}

static ssize_t
ch_conf_show(struct class *class, char *buf)
{
        struct ch_conf_result result;
        result.buf = buf;
        result.len = 0;
        bus_for_each_dev(&inno_lgx_bus_type, NULL, &result, lgx_ch_conf_show); 
        return result.len; 
}

static int lgx_ch_cnt_show(struct device *dev, void *data)
{
        char name[] = LG_PREFIX;
        struct lgx_device *lgx_dev;
        int *cnt = (int *)data;

        lgx_dev = container_of(dev, struct lgx_device, dev);
        if (strcmp(name, lgx_dev->name) == 0)
                (*cnt) += 1;
        return 0;
}

static ssize_t
ch_cnt_show(struct class *class, char *buf)
{
        int cnt = 0;       

        bus_for_each_dev(&inno_lgx_bus_type, NULL, &cnt, lgx_ch_cnt_show); 

        return sprintf(buf, "%d\n", cnt);
}

static ssize_t
sys_status_show(struct class *class, char *buf)
{
        struct inno_req req;
        struct sys_status status;
        int ret;
        req.handler = get_sys_status;
        req.context = &status;
        ret = inno_req_sync(&req);
        if (ret < 0)
                return sprintf(buf, "Device transfer error, maybe power is lost\n");
        return sprintf(buf, "sync=%d strength=%d quality=%d freq=%d ldpc=%d rs=%d\n",
                        status.sync,
                        status.signal_strength,
                        status.signal_quality,
                        status.cur_freq,
                        status.ldpc_err_percent,
                        status.rs_err_percent); 
}

static ssize_t
err_info_show(struct class *class, char *buf)
{
        struct inno_req req;
        struct err_info info;
        int ret;
        req.handler = get_err_info;
        req.context = &info;
        ret = inno_req_sync(&req);
        if (ret < 0)
                return sprintf(buf, "Device transfer error, maybe power is lost\n");
        return sprintf(buf, "ldpc_total=%d ldpc_err=%d rs_total=%d rs_err=%d BER=%d SNR=%d\n",
                        info.ldpc_total_count,
                        info.ldpc_error_count,
                        info.rs_total_count,
                        info.rs_error_count,
                        info.BER,
                        info.SNR); 
}

static ssize_t
chip_id_show(struct class *class, char *buf)
{
        struct inno_req req;
        unsigned int id;
        int ret;

        req.handler = get_chip_id;
        req.context = &id;

        ret = inno_req_sync(&req);
        if (ret < 0)
                return sprintf(buf, "Device transfer error, maybe power is lost\n");
        return sprintf(buf, "chip id = %x\n", id);
}

static ssize_t
test_show(struct class *class, char *buf)
{
	unsigned char tx_val, rx_val;
	unsigned int	cpu_ctr;
		
//	inno_plat_power(1);
	tx_val = 1;
	inno_spi_sync(&tx_val, 1, SPI_TX, &rx_val, 1, SPI_RX, 1);

	cpu_ctr = 0x00200000;
        inno_comm_send_unit(0x40004010, (unsigned char *)&cpu_ctr, 4);
	cpu_ctr = 0;
        inno_comm_get_unit(0x40004010, (unsigned char *)&cpu_ctr, 4);
	sprintf(buf, "0x%x", cpu_ctr);		
	
	return sprintf(buf, "spi:write(1), read(exp 2, act %d)\n"  \
                            "reg(0x40004010): write(0x00200000), read(exp 0x00200000, act 0x%x)\n", \
                             rx_val, cpu_ctr);
       
}

static struct class_attribute demod_attrs[] = {
        __ATTR(fw_version, S_IRUGO, fw_version_show, NULL),
        __ATTR(sys_status, S_IRUGO, sys_status_show, NULL),
        __ATTR(err_info, S_IRUGO, err_info_show, NULL),
        __ATTR(chip_id, S_IRUGO, chip_id_show, NULL),
        __ATTR(freq, 0666, freq_show, freq_store),
        __ATTR(power, 0666, power_show, power_store),
        __ATTR(ch_conf, 0666, ch_conf_show, ch_conf_store),
        __ATTR(ch_cnt, S_IRUGO, ch_cnt_show, NULL),
        __ATTR(test, S_IRUGO, test_show, NULL),
        { },
};

void inno_demod_sysfs_init(struct class *cls)
{
        cls->class_attrs = demod_attrs;
}
