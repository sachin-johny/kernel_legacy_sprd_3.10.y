/*
* drivers/media/video/sprd_dcam/sc8810/isp_control.c
 * 
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
#include "dcam_common.h"
#include "isp_control.h"

static uint32_t s_dcam_user_count = 0;
static struct semaphore s_sem_cnt;
void dcam_inc_user_count(void)
{
	s_dcam_user_count++;
	DCAM_TRACE("DCAM:dcam_inc_user_count: count: %d.\n", s_dcam_user_count);
}

void dcam_dec_user_count(void)
{
	if(s_dcam_user_count >= 1)
		s_dcam_user_count--;
	else
		s_dcam_user_count = 0;
	DCAM_TRACE("DCAM:dcam_dec_user_count: count: %d.\n", s_dcam_user_count);
}
 
uint32_t dcam_get_user_count(void)
{
	DCAM_TRACE("DCAM:dcam_get_user_count: count: %d.\n", s_dcam_user_count);
	return s_dcam_user_count;
}
void isp_get_path2(void)
{
	down(&s_sem_cnt);
}
void isp_put_path2(void)
{
	up(&s_sem_cnt);
}
void isp_mutex_init(void)
{
	init_MUTEX(&s_sem_cnt);
}