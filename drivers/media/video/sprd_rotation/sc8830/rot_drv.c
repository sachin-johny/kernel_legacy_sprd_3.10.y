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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <video/sprd_rot_k.h>
#include "rot_drv.h"
#include "sc8830_reg_rot.h"
#include "../../sprd_dcam/sc8830/dcam_drv_sc8830.h"

#define RTT_PRINT pr_debug
//#define RTT_PRINT printk
//#define ROTATION_DEBUG 0
#define ALGIN_FOUR 0x03

#define REG_RD(a) __raw_readl(a)
#define REG_WR(a,v) __raw_writel(v,a)
#define REG_AWR(a,v) __raw_writel((__raw_readl(a) & (v)), (a))
#define REG_OWR(a,v) __raw_writel((__raw_readl(a) | (v)), (a))
#define REG_XWR(a,v) __raw_writel((__raw_readl(a) ^ (v)), (a))
#define REG_MWR(a,m,v) \
	do { \
		uint32_t _tmp = __raw_readl(a); \
		_tmp &= ~(m); \
		__raw_writel((_tmp | ((m) & (v))), (a)); \
	}while(0)

typedef struct _rot_param_tag {
	ROT_SIZE_T img_size;
	ROT_DATA_FORMAT_E format;
	ROT_ANGLE_E angle;
	ROT_ADDR_T src_addr;
	ROT_ADDR_T dst_addr;
	uint32_t s_addr;
	uint32_t d_addr;
	ROT_PIXEL_FORMAT_E pixel_format;
	ROT_UV_MODE_E uv_mode;
	int is_end;
	ROT_ENDIAN_E src_endian;
	ROT_ENDIAN_E dst_endian;
} ROT_PARAM_CFG_T;

static uint32_t g_rot_irq = 0x87654321;
static rot_isr_func user_rot_isr_func;
static ROT_PARAM_CFG_T s_rotation_cfg;
static DEFINE_SPINLOCK(rot_lock);

#define DECLARE_ROTATION_PARAM_ENTRY(s) ROT_PARAM_CFG_T *s = &s_rotation_cfg

int rot_k_module_en(void)
{
	int ret = 0;

	ret = dcam_module_en();

	if (ret) {
		printk("dcam_module_en, failed  %d \n", ret);
	}

	return ret;
}

int rot_k_module_dis(void)
{
	int ret = 0;

	ret = dcam_module_dis();

	if (ret) {
		printk("rot_k_module_dis, failed  %d \n", ret);
	}

	return ret;
}

static void rot_k_set_src_addr(uint32_t src_addr)
{
	REG_WR(REG_ROTATION_SRC_ADDR, src_addr);
}

static void rot_k_set_dst_addr(uint32_t dst_addr)
{
	REG_WR(REG_ROTATION_DST_ADDR, dst_addr);
}

static void rot_k_set_img_size(ROT_SIZE_T * size)
{
	REG_WR(REG_ROTATION_OFFSET_START, 0x00000000);
	REG_WR(REG_ROTATION_IMG_SIZE,
		((size->w & 0x1FFF) | ((size->h & 0x1FFF) << 13)));
	REG_WR(REG_ROTATION_ORIGWIDTH, (size->w & 0x1FFF));
}

static void rot_k_set_endian(ROT_ENDIAN_E src_end, ROT_ENDIAN_E dst_end)
{
	REG_AWR(REG_ROTATION_ENDIAN_SEL,
		(~(ROT_RD_ENDIAN_MASK |ROT_WR_ENDIAN_MASK)));

	REG_OWR(REG_ROTATION_ENDIAN_SEL,
		(ROT_AXI_RD_WORD_ENDIAN_BIT |
		ROT_AXI_WR_WORD_ENDIAN_BIT |
		(src_end << 16) |(dst_end << 14)));
}

static void rot_k_set_pixel_mode(ROT_PIXEL_FORMAT_E pixel_format)
{
	REG_AWR(REG_ROTATION_PATH_CFG, (~ROT_PIXEL_FORMAT_BIT));
	REG_OWR(REG_ROTATION_PATH_CFG, ((pixel_format & 0x1) << 1));
	return;
}

static void rot_k_set_UV_mode(ROT_UV_MODE_E uv_mode)
{
	REG_AWR(REG_ROTATION_PATH_CFG, (~ROT_UV_MODE_BIT));
	REG_OWR(REG_ROTATION_PATH_CFG, ((uv_mode & 0x1) << 4));
	return;
}

static void rot_k_set_dir(ROT_ANGLE_E angle)
{
	REG_AWR(REG_ROTATION_PATH_CFG, (~ROT_MODE_MASK));
	REG_OWR(REG_ROTATION_PATH_CFG, ((angle & 0x3) << 2));
}

static void rot_k_interrupt_en(void)
{
	REG_OWR(REG_ROTATION_INT_CLR, ROT_IRQ_BIT);
	REG_OWR(REG_ROTATION_INT_MASK, ROT_IRQ_BIT);
}

static void rot_k_interrupt_dis(void)
{
	REG_OWR(REG_ROTATION_INT_CLR, ROT_IRQ_BIT);
	REG_AWR(REG_ROTATION_INT_MASK, (~ROT_IRQ_BIT));
}

void rot_k_enable(void)
{
	REG_OWR(REG_ROTATION_PATH_CFG, ROT_EB_BIT);
	REG_OWR(REG_ROTATION_CTRL, ROT_START_BIT);
}

void rot_k_disable(void)
{
	REG_AWR(REG_ROTATION_PATH_CFG, (~ROT_EB_BIT));
}

static irqreturn_t rot_k_isr_root(int irq, void *dev_id)
{
	uint32_t status;
	uint32_t flag;

	(void)irq; (void)dev_id;
	status = REG_RD(REG_ROTATION_INT_STS);

	if (unlikely(0 == (status & ROT_IRQ_BIT))) {
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&rot_lock, flag);

	if (user_rot_isr_func) {
		user_rot_isr_func();
	}

	REG_OWR(REG_ROTATION_INT_CLR, ROT_IRQ_BIT);

	spin_unlock_irqrestore(&rot_lock, flag);

	return IRQ_HANDLED;
}

int rot_k_isr_reg(rot_isr_func user_func)
{
	int rtn = 0;
	uint32_t flag;

	spin_lock_irqsave(&rot_lock, flag);
	user_rot_isr_func = user_func;
	spin_unlock_irqrestore(&rot_lock, flag);

	rtn = request_irq(ROT_IRQ,
			rot_k_isr_root,
			IRQF_SHARED,
			"ROTATE",
			&g_rot_irq);
	if (rtn) {
		printk("request_irq error %d \n", rtn);
		rtn = -1;
	}else
		rot_k_interrupt_en();

	return rtn;
}

void rot_k_isr_unreg(void)
{
	rot_k_disable();
	rot_k_interrupt_dis();
	free_irq(ROT_IRQ, &g_rot_irq);
}

int rot_k_is_end(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	return s->is_end ;
}

static uint32_t rot_k_get_end_mode(void)
{
	uint32_t ret = 1;
	DECLARE_ROTATION_PARAM_ENTRY(s);

	switch (s->format) {
	case ROT_YUV422:
	case ROT_YUV420:
		ret = 0;
		break;
	case ROT_RGB565:
		ret = 1;
		break;
	default:
		ret = 1;
		break;
	}
	return ret;
}

static ROT_PIXEL_FORMAT_E rot_k_get_pixel_format(void)
{
	ROT_PIXEL_FORMAT_E ret = ROT_ONE_BYTE;
	DECLARE_ROTATION_PARAM_ENTRY(s);

	switch (s->format) {
	case ROT_YUV422:
	case ROT_YUV420:
		ret = ROT_ONE_BYTE;
		break;
	case ROT_RGB565:
		ret = ROT_TWO_BYTES;
		break;
	default:
		ret = ROT_ONE_BYTE;
		break;
	}

	return ret;
}

static int rot_k_set_y_param(ROT_CFG_T * param_ptr)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	memcpy((void *)&(s->img_size), (void *)&(param_ptr->img_size),
		sizeof(ROT_SIZE_T));
	memcpy((void *)&(s->src_addr), (void *)&(param_ptr->src_addr),
		sizeof(ROT_ADDR_T));
	memcpy((void *)&(s->dst_addr), (void *)&(param_ptr->dst_addr),
		sizeof(ROT_ADDR_T));

	s->s_addr = param_ptr->src_addr.y_addr;
	s->d_addr = param_ptr->dst_addr.y_addr;
	s->format = param_ptr->format;
	s->angle = param_ptr->angle;
	s->pixel_format = rot_k_get_pixel_format();
	s->is_end = rot_k_get_end_mode();
	s->uv_mode = ROT_NORMAL;
	s->src_endian = 1;/*param_ptr->src_addr;*/
	s->dst_endian = 1;/*param_ptr->dst_endian;*/
	return 0;
}

int rot_k_set_UV_param(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	s->s_addr = s->src_addr.u_addr;
	s->d_addr = s->dst_addr.u_addr;
	s->img_size.w >>= 0x01;
	s->pixel_format = ROT_TWO_BYTES;
	if (ROT_YUV422 == s->format){
		s->uv_mode = ROT_UV422;
	} else if (ROT_YUV420 == s->format) {
		s->img_size.h >>= 0x01;
	}
	return 0;
}

void rot_k_done(void)
{
	DECLARE_ROTATION_PARAM_ENTRY(s);

	rot_k_set_src_addr(s->s_addr);
	rot_k_set_dst_addr(s->d_addr);
	rot_k_set_img_size(&(s->img_size));
	rot_k_set_pixel_mode(s->pixel_format);
	rot_k_set_UV_mode(s->uv_mode);
	rot_k_set_dir(s->angle);
	rot_k_set_endian(s->src_endian, s->dst_endian);
	rot_k_enable();
	RTT_PRINT("ok to rotation_done.\n");
}

static int rot_k_check_param(ROT_CFG_T * param_ptr)
{
	if (NULL == param_ptr) {
		RTT_PRINT("Rotation: the param ptr is null.\n");
		return -1;
	}

	if ((param_ptr->src_addr.y_addr & ALGIN_FOUR)
	|| (param_ptr->src_addr.u_addr & ALGIN_FOUR)
	|| (param_ptr->src_addr.v_addr & ALGIN_FOUR)
	|| (param_ptr->dst_addr.y_addr & ALGIN_FOUR)
	|| (param_ptr->dst_addr.u_addr & ALGIN_FOUR)
	|| (param_ptr->dst_addr.v_addr & ALGIN_FOUR)) {
		RTT_PRINT("Rotation: the addr not algin.\n");
		return -1;
	}

	if (!(ROT_YUV422 == param_ptr->format || ROT_YUV420 == param_ptr->format
		||ROT_RGB565 == param_ptr->format)) {
		RTT_PRINT("Rotation: data for err : %d.\n", param_ptr->format);
		return -1;
	}

	if (ROT_MIRROR < param_ptr->angle) {
		RTT_PRINT("Rotation: data angle err : %d.\n", param_ptr->angle);
		return -1;
	}

	if (ROT_ENDIAN_MAX <= param_ptr->src_endian ||
		ROT_ENDIAN_MAX <= param_ptr->dst_endian ) {
		RTT_PRINT("Rotation: endian err : %d %d.\n", param_ptr->src_endian,
			param_ptr->dst_endian);
		return -1;
	}

	return 0;
}

int rot_k_io_cfg(ROT_CFG_T * param_ptr)
{
	int ret = 0;
	ROT_CFG_T *p = param_ptr;

	RTT_PRINT("rot_k_io_cfg start \n");
	RTT_PRINT("w=%d, h=%d \n", p->img_size.w, p->img_size.h);
	RTT_PRINT("format=%d, angle=%d \n", p->format, p->angle);
	RTT_PRINT("s.y=%x, s.u=%x, s.v=%x \n", p->src_addr.y_addr, p->src_addr.u_addr, p->src_addr.v_addr);
	RTT_PRINT("d.y=%x, d.u=%x, d.v=%x \n", p->dst_addr.y_addr, p->dst_addr.u_addr, p->dst_addr.v_addr);

	ret = rot_k_check_param(param_ptr);

	if(0 == ret)
		ret = rot_k_set_y_param(param_ptr);

	return ret;
}

