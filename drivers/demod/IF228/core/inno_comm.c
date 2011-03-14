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
#include "inno_core.h"
#include "inno_cmd.h"
#include "inno_reg.h"
#include "inno_comm.h"

/**
 * inno_comm_cmd_idle -check if previous cmd had been done
 */
int inno_comm_cmd_idle(void)
{
        unsigned char idle;
        int i;
        for(i = 0; i < 500; i++) {
                inno_comm_get_unit(FETCH_PER_COMM31, &idle, 1);
                if((idle & CMD_BUSY) == 0)   //bit7:1 cmd busy; 0 cmd over
                        return 1;
        }
        return 0;        
}
EXPORT_SYMBOL_GPL(inno_comm_cmd_idle);

/**
 * inno_comm_send_cmd -send cmd and cmd data to if20x
 */ 
int inno_comm_send_cmd(struct inno_cmd_frame *cmd_frame)
{
        /* send data to data reg*/
        inno_comm_send_unit(FETCH_PER_COMM1, cmd_frame->data, 7);
        /* send cmd to cmd reg*/
        inno_comm_send_unit(FETCH_PER_COMM0, &cmd_frame->code, 1);
        return 0;
}
EXPORT_SYMBOL_GPL(inno_comm_send_cmd);

/**
 * inno_comm_get_rsp -get rsp from if2xx
 */
int inno_comm_get_rsp(struct inno_rsp_frame *rsp_frame)
{
        int i;
        unsigned char status;
        for(i = 0; i < 500; i++) {
                inno_comm_get_unit(FETCH_PER_COMM31, &status, 1);

                /* bit6-0:0 success 1 error */
                if((status & RSP_DATA_VALID) == 0) {
                        /* RSP_DATA_OK: 0X00 RSP_DATA_ERR:0X01 */
                        if((status & 0x7F) == 0) {
                                /* get rsp data from rsp reg */
                                inno_comm_get_unit(FETCH_PER_COMM8, (unsigned char *)rsp_frame, 8);
                                return 0;
                        } else {
                                pr_err("\r\n rsp error = 0x%x\r\n", status&0x7f);
                                return -EIO;
                        }
                }
        }
        return -EBUSY;
}
EXPORT_SYMBOL_GPL(inno_comm_get_rsp);
