/*
* drivers/media/video/sprd_dcam/dcam_power_sc8800g2.h
 * Dcam driver based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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
#ifndef _DCAM_POWER_SC8800G2_H_
#define _DCAM_POWER_SC8800G2_H_

extern void dcam_inc_user_count(void);
extern void dcam_dec_user_count(void);
extern uint32_t dcam_get_user_count(void);
#endif //_DCAM_SC8800G2_H_
