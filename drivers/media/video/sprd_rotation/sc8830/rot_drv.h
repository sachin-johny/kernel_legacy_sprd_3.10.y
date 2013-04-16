/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _ROT_REG_H_
#define _ROT_REG_H_

typedef void (*rot_isr_func)(void);
int rot_k_module_en(void);
int rot_k_module_dis(void);
void rot_k_enable(void);
void rot_k_disable(void);
int rot_k_isr_reg(rot_isr_func user_func);
void rot_k_isr_unreg(void);
int rot_k_is_end(void);
int rot_k_set_UV_param(void);
void rot_k_done(void);
int rot_k_io_cfg(ROT_CFG_T * param_ptr);
#endif
