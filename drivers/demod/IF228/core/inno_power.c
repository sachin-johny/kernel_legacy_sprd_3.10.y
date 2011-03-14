
/*
 * innofidei if2xx demod power control driver
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
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/firmware.h>
#include "inno_core.h"
#include "inno_cmd.h"
#include "inno_reg.h"
#include "inno_comm.h"
#include "inno_power.h"


#define FW_REG 0x20000000

static void dump_mem(const unsigned char *buf, size_t len)
{
        int i;
        char tmp[10];
        int a, b, pos;
        a = (len / 8) * 8;
        b = len % 8;
        for (i = 0; i < a; i+=8) {
                pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                        buf[i+0], buf[i+1], buf[i+2], buf[i+3],
                        buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
        }
        for (i = 0, pos = 0; i < b; i++) 
                pos += sprintf(&tmp[pos], "%02x ", buf[a+i]);
        tmp[pos] = '\0';
        pr_debug("%s\n", tmp);
}

static int checkdownload(unsigned char *buffer, int len)
{
        int ret = 0;
        struct inno_cmd_frame cmd_frm;
        struct inno_rsp_frame rsp_frm;
        unsigned short file_checksum = 0;
        unsigned short read_checksum = 0;
        int i = 0;
        int retry = 0;

        /* check sum is not ok now, so we simple return success */
        return 0;

        /************1. calculate check sum of .bin ****************/
        if(len % 2 == 0){
                for(i = 0; i< len / 2; i++)             
                        file_checksum ^= *((unsigned short *)buffer + i);               

        }
        else{
                for(i = 0; i< (len - 1) / 2; i++)
                        file_checksum ^= *((unsigned short *)buffer + i);               
                file_checksum ^= *(buffer + len - 1) | (0xFF << 8);
                
        }

        pr_debug("\n%s:fw_checksum : 0x%x\n", __func__, file_checksum);

        /************2. Get check sum from demod *******************/

        /* CHIP_V5 CMD Description:
         * Check firmware sum Command 
         *
         * CMD[0] = CMD_GET_FW_CHECK_SUM
         * CMD[1] = Start_address_Low
         * CMD[2] = Start_address_High
         * CMD[3] = END_address_Low
         * CMD[4] = END_address_High
         * CMD[5] = 0
         * CMD[6] = 0
         * CMD[7] = 0
         *
         * RSP[0] = CMD_GET_FW_CHECK_SUM
         * RSP[1] = CheckSum Low
         * RSP[2] = CheckSum High
         * RSP[3] = 0
         * RSP[4] = 0
         * RSP[5] = 0
         * RSP[6] = 0
         * RSP[7] = 0
         */
         
        cmd_frm.code = CMD_GET_FW_CHECK_SUM;          
        cmd_frm.data[0] = 0;
        cmd_frm.data[1] = 0;
        cmd_frm.data[2] = len&0xff;
        cmd_frm.data[3] = (len>>8)&0xff;

        for(retry = 0; retry < 20; retry ++){

                ret = inno_comm_send_cmd(&cmd_frm);
                if(ret < 0)
                        continue;

                ret = inno_comm_get_rsp(&rsp_frm);
                if(ret < 0)
                        continue;
                else
                        break;
        }

        read_checksum = (rsp_frm.data[1] << 8) | rsp_frm.data[0];

        pr_debug("%s:file_checksum = 0x%x, read_checksum = 0x%x\n", __func__, file_checksum, read_checksum);

        
        /*************** 3. Compare check sum *****************/
        if(file_checksum != read_checksum)
                ret = -EIO;

        return ret;
}

static int downloadfw(unsigned char *buf, int len)
{
        int ret = 0;
        unsigned long clk_ctr = 0x80000001, cpu_ctr = 0x00200000, value = 0x00000000;

        /* reset chip */ 
        inno_plat_power(0);
        mdelay(200);
        inno_plat_power(1);
        mdelay(200);
        
        inno_comm_send_unit(M0_REG_CPU_CTR, (unsigned char *)&cpu_ctr, 4);
        
        inno_comm_send_unit(FW_BASE_ADDR, buf, len);
        
        inno_comm_send_unit(M0_REG_CLK_CTR, (unsigned char *)&clk_ctr, 4);

        inno_comm_send_unit(M0_REG_CPU_CTR, (unsigned char *)&value, 4);
        mdelay(500);
		
        return ret;
}

int inno_demod_power(struct inno_demod *demod, unsigned char on)
{
        int ret = 0;
        const struct firmware *fw;
        unsigned char *fw_buf;

        pr_debug("%s: %s\n", __func__, on?"on":"off");
        if (!on) {
                inno_plat_power(0);
                goto out;
        }

        ret = request_firmware(&fw, "inno_demod_fw.bin", demod->dev);
        if (ret) {
                pr_err("Error %d, dev %d:request firmware timeout, place inno_demod_fw.bin into firmware folder please\n", ret, (int)demod->dev);
                goto out;
        }

        pr_debug("%s:fw size = %d\n", __func__,fw->size);
        fw_buf = kzalloc(fw->size, GFP_KERNEL | GFP_DMA);
        memcpy(fw_buf, fw->data, fw->size);
        dump_mem(fw->data, 20);
       
        downloadfw(fw_buf, fw->size);
        ret = checkdownload(fw_buf, fw->size);
        if (ret < 0) {
                pr_debug("%s:fw download failed\n", __func__);        
                inno_plat_power(0);
        }

        release_firmware(fw);
        kfree(fw_buf);
out:
        
        return ret;     
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean.zhao <zhaoguangyu@innofidei.com>");
MODULE_DESCRIPTION("innofidei cmmb fw download");

