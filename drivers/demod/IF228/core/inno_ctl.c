
/*
 * innofidei if2xx demod ctl driver
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
#include "inno_core.h"
#include "inno_reg.h"
#include "inno_cmd.h"
#include "inno_ctl.h"
#include "inno_irq.h"
#include "inno_demod.h"
#include "inno_comm.h"
#include "inno_io.h"
#include "inno_power.h"

struct {
        struct class    *cls;
        struct device   *parent;
        int             major;
        int             minor;
        dev_t           devt;
}inno_ctl;

int get_fw_version(void *data)
{
        struct inno_cmd_frame cmd_frame = {0};
        struct inno_rsp_frame rsp_frame = {0};
        unsigned int version = 0;
        int ret = 0;

        pr_debug("%s\n", __func__);
        if (data == NULL) {
                pr_err("%s:input param invalid\n", __func__);
                ret = -EINVAL;
                goto done;
        }

        cmd_frame.code = CMD_GET_FW_VER;
        if(inno_comm_cmd_idle()) {
                inno_comm_send_cmd(&cmd_frame);        
                ret = inno_comm_get_rsp(&rsp_frame);
                if (ret < 0) 
                        goto done;                
        
                version = (rsp_frame.data[0]<<8) | rsp_frame.data[1];
        } else {
                ret = -EBUSY;
                goto done;
        }

        *(unsigned int *)data = version;
done:
        return ret;
}

int set_frequency(void *data)
{
        struct inno_cmd_frame cmd_frame = {0};
        unsigned char freq_dot;

        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;

        freq_dot = *(unsigned char *)data;

        cmd_frame.code = CMD_SET_FREQUENCY;
        cmd_frame.data[0] = freq_dot;
        if(inno_comm_cmd_idle()) 
                inno_comm_send_cmd(&cmd_frame);        
        else
                return -EBUSY;

        return 0;

}

int get_frequency(void *data)
{
        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;
        inno_comm_get_unit(FETCH_PER_COMM17, data, 1);
        return 0;
}

int set_ch_config(void *data)
{
        struct inno_cmd_frame cmd_frame = {0};
        struct ch_config *config = (struct ch_config *)data;
    
        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;
        
        config->ch_id += 1;
        if (config->ch_id < 1)
                return -EINVAL;   
 
        cmd_frame.code = CMD_SET_CHANNEL_CONFIG;
        memcpy(cmd_frame.data, config, sizeof(*config));
        
        if(inno_comm_cmd_idle()) 
                inno_comm_send_cmd(&cmd_frame);        
        else
                return -EBUSY;

        return 0;
}

int get_ch_config(void *data)
{
        struct inno_cmd_frame cmd_frame = {0};
        struct inno_rsp_frame rsp_frame = {0};
        struct ch_config *config = (struct ch_config *)data;
        int ret = 0;        
 
        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;

        config->ch_id += 1;
        if (config->ch_id < 1) {
                ret = -EINVAL;
                goto done;
        }   

        cmd_frame.code  = CMD_GET_CHANNEL_CONFIG; 
        cmd_frame.data[0] = config->ch_id;
        
        if(inno_comm_cmd_idle()) {
                inno_comm_send_cmd(&cmd_frame);        
                ret = inno_comm_get_rsp(&rsp_frame);
                if (ret < 0)
                        goto done;                
                
                memcpy(config, rsp_frame.data, sizeof(*config));
                config->ch_id -= 1;
        } else {
                ret = -EBUSY;
        }

done:
        return ret;
}

//add by mahanghong 20110118
unsigned long ParseErrStatus(unsigned char status)
{
	unsigned long ret = 0;

	switch ( status )
	{
		case CAS_OK:
			ret = 0x9000;
			break;
		case NO_MATCHING_CAS:   //Can not find ECM data
			ret = NO_MATCHING_CAS;
			break;
		case CARD_OP_ERROR:    //Time out or err
			ret = CARD_OP_ERROR;
			break;
		case MAC_ERR:
			ret = 0x9862;
			break;
		case GSM_ERR:
			ret = 0x9864;
			break;
		case KEY_ERR:
			ret = 0x9865;
			break;
		case KS_NOT_FIND:
			ret = 0x6985;
			break;
		case KEY_NOT_FIND:
			ret = 0x6a88;
			break;
		case CMD_ERR:
			ret = 0x6f00;
			break;
		default:
			break;			
	}
	
 	return ret;
}

int get_sys_status(void *data)
{
        struct sys_status *sys_status = (struct sys_status *)data;

//add by mahanghong 20110118
	unsigned char status = 0;
        
        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;

        inno_comm_get_unit(OFDM_SYNC_STATE, &sys_status->sync, 1);
        if (sys_status->sync & 0x08)
                sys_status->sync = 1;
        else
                sys_status->sync = 0;
                
        inno_comm_get_unit(FETCH_PER_COMM16, &sys_status->signal_strength, 1);
        inno_comm_get_unit(REG_SIGNAL_QUALITY, &sys_status->signal_quality, 1); 
        inno_comm_get_unit(FETCH_PER_COMM17, &sys_status->cur_freq, 1);
        inno_comm_get_unit(FETCH_PER_COMM20, &sys_status->ldpc_err_percent, 1);
        inno_comm_get_unit(FETCH_PER_COMM21, &sys_status->rs_err_percent, 1);

//add by mahanghong 20110118
        inno_comm_get_unit(FETCH_PER_COMM29, &status, 1);
        sys_status->err_status = ParseErrStatus(status);
        
        return 0;
}

int get_err_info(void *data)
{
        struct err_info *err_info = (struct err_info *)data;
        
        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;
       
        inno_comm_get_unit(FETCH_LDPC_TOTAL, (unsigned char*)&err_info->ldpc_total_count, 4); 
        inno_comm_get_unit(FETCH_LDPC_ERR, (unsigned char*)&err_info->ldpc_error_count, 4); 
        inno_comm_get_unit(FETCH_RS_TOTAL, (unsigned char*)&err_info->rs_total_count, 4); 
        inno_comm_get_unit(FETCH_RS_ERR, (unsigned char*)&err_info->rs_error_count, 4); 
        inno_comm_get_unit(FETCH_PER_COMM22, (unsigned char*)&err_info->BER, 2); 
        inno_comm_get_unit(FETCH_PER_COMM18, (unsigned char*)&err_info->SNR, 2); 

        return 0;        
}

int get_chip_id(void *data)
{
        unsigned char data_high = 0, data_low = 0;
        unsigned int chip_id = 0;
	 unsigned char buf[8] = {0};

        pr_debug("%s\n", __func__);
        if (data == NULL)
                return -EINVAL;

        inno_comm_get_unit(M0_REG_PLL1_CTR, &data_high, 1);    
        inno_comm_get_unit(M0_REG_PLL_STATUS, &data_low, 1);   
	 inno_comm_get_unit(0, buf, 8);

	 pr_debug("buf : %x %x %x %x %x %x %x %x\n", buf[0], buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	 
        chip_id = (data_high<<8) + (data_low & 0x70);
        *(unsigned int *)data = chip_id;
        return 0;
}


int inno_cmd_proc(unsigned int cmd, unsigned long arg)
{
        struct inno_req req;
        int ret = 0;

        pr_debug("%s, cmd = %d\n", __func__, cmd);
        
        
        switch(cmd) {
                case INNO_IO_GET_FW_VERSION:
                {
                        unsigned int version;        
                        req.handler = get_fw_version;
                        req.context = &version;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        if (copy_to_user((void __user *)arg, &version, sizeof(version)))
                                return -EFAULT;
                        break;
                }
                case INNO_IO_SET_FREQUENCY:
                {
                        unsigned char freq_dot;
                        if (copy_from_user(&freq_dot, (void __user *)arg, sizeof(freq_dot)))
                                return -EFAULT; 
                        req.handler = set_frequency;
                        req.context = &freq_dot;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        break;
                }
                case INNO_IO_GET_FREQUENCY:
                {
                        unsigned char freq_dot;
                        req.handler = get_frequency;
                        req.context = &freq_dot;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        if (copy_to_user((void __user *)arg, &freq_dot, sizeof(freq_dot)))
                                return -EFAULT;
                        break;
                }
                case INNO_IO_SET_CH_CONFIG:
                {
                        struct ch_config config;
                                
                        if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
                                return -EFAULT; 

                        pr_debug("%s,set channel config\n", __func__);
                        
                        req.handler = set_ch_config;
                        req.context = &config;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        break;
                }
                case INNO_IO_GET_CH_CONFIG:
                {
                        struct ch_config config;
                        
                        if (!access_ok(VERIFY_READ, 
                                        (u8 __user *)(uintptr_t) arg,
                                        sizeof(config)))
                                return -EFAULT;

                        if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
                                return -EFAULT; 
                        
                        req.handler = get_ch_config;
                        req.context = &config;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;

                        if (copy_to_user((void __user *)arg, &config, sizeof(config)))
                                return -EFAULT; 
                        break;
                }
                case INNO_IO_GET_SYS_STATUS:
                {
                        struct sys_status sys_status;
                        req.handler = get_sys_status;
                        req.context = &sys_status;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;

                        if(copy_to_user((void __user *)arg, &sys_status, sizeof(sys_status)))
                                return -EFAULT;
                        break;
                }
                case INNO_IO_GET_ERR_INFO:
                {
                        struct err_info info;
                        
                        req.handler = get_err_info;
                        req.context = &info;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        if(copy_to_user((void __user *)arg, &info, sizeof(info)))
                                return -EFAULT;
                        break;
                }
                case INNO_IO_GET_CHIP_ID:
                {
                        unsigned int id;
                        req.handler = get_chip_id;
                        req.context = &id;
                        ret = inno_req_sync(&req);
                        if (ret < 0)
                                break;
                        if(copy_to_user((void __user *)arg, &id, sizeof(id)))
                                return -EFAULT;
                        break;
                }
                default:
                        return -EINVAL;
        }
        
        return ret;
}

static int innoctl_ioctl(struct inode* inode, struct file* filp, unsigned int cmd, unsigned long arg)
{
        void __user *argp = (void __user *)arg;
        unsigned char val;
        int     retval = 0;
        
        pr_debug("%s\n", __func__);

        switch(cmd){
                case INNO_IO_POWERENABLE:
                        if(copy_from_user(&val, argp, sizeof(val)))
                                return -EFAULT;
                        if (val)
                                retval = inno_demod_inc_ref();
                        else
                                inno_demod_dec_ref();
                        break;
                case INNO_IO_RESET:
                        if(copy_from_user(&val, argp, sizeof(val)))
                                return -EFAULT;
                        /* reset chip */ 
                        inno_plat_power(0);
                        mdelay(200);
                        inno_plat_power(1);
                        mdelay(200);
                        break;
                case INNO_IO_GET_FW_VERSION:
                case INNO_IO_SET_FREQUENCY:
                case INNO_IO_GET_FREQUENCY:
                case INNO_IO_SET_CH_CONFIG:
                case INNO_IO_GET_CH_CONFIG:
                case INNO_IO_GET_SYS_STATUS:
                case INNO_IO_GET_ERR_INFO:
                case INNO_IO_GET_CHIP_ID:
                        BUG_ON(arg == 0);
                       return inno_cmd_proc(cmd, arg); 
                default:
                        return -EINVAL;
        }
        return retval;
}

static int innoctl_open(struct inode* inode, struct file* file)
{
        pr_debug("%s\n", __func__);
        
        /* inc demod ref */
        return inno_demod_inc_ref();
}

static int innoctl_release(struct inode *inode, struct file *filep)
{
        pr_debug("%s\n", __func__);
        inno_demod_dec_ref();
        return 0;
}

static struct file_operations innoctl_fops = {
        .owner          = THIS_MODULE,
        .open           = innoctl_open,
        .ioctl          = innoctl_ioctl,
        .release        = innoctl_release,
};


int inno_ctl_init(struct inno_demod *inno_demod)
{
        pr_debug("%s\n", __func__);

        inno_ctl.major = register_chrdev(0, "innoctl", &innoctl_fops);
        if (inno_ctl.major < 0){ 
                pr_err("failed to register character device.\n");     
                return inno_ctl.major; 
        }
        pr_debug("%s:ctl major = %d\n", __func__, inno_ctl.major); 

        inno_ctl.cls = &inno_demod->cls;
        inno_ctl.parent = inno_demod->dev;
        inno_ctl.minor = 0;
        inno_ctl.devt = MKDEV(inno_ctl.major, 0);

        device_create(inno_ctl.cls, inno_ctl.parent, inno_ctl.devt,
                       NULL, "innoctl");
        
        return 0;
}

void inno_ctl_exit(void)
{
        pr_debug("%s\n", __func__);
        unregister_chrdev(inno_ctl.major, "innoctl");
        device_destroy(inno_ctl.cls, inno_ctl.devt);
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean.zhao <zhaoguangyu@innofidei.com>");
MODULE_DESCRIPTION("innofidei cmmb ctl");

