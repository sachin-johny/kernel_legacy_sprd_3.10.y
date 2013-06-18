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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/sci.h>

#include "dcam_drv_sc8830.h"
#include "gen_scale_coef.h"


//#define LOCAL    static
#define LOCAL

//#define DCAM_DRV_DEBUG
#define DCAM_LOWEST_ADDR                               0x800
#define DCAM_ADDR_INVALIDE(addr)                       ((addr) < DCAM_LOWEST_ADDR)
#define DCAM_YUV_ADDR_INVALIDE(y,u,v)                  \
	(DCAM_ADDR_INVALIDE(y) &&                      \
	DCAM_ADDR_INVALIDE(u) &&                       \
	DCAM_ADDR_INVALIDE(v))

#define DCAM_SC1_H_TAB_OFFSET                          0x400
#define DCAM_SC1_V_TAB_OFFSET                          0x4F0
#define DCAM_SC1_V_CHROMA_TAB_OFFSET                   0x8F0

#define DCAM_SC2_H_TAB_OFFSET                          0x1400
#define DCAM_SC2_V_TAB_OFFSET                          0x14F0
#define DCAM_SC2_V_CHROMA_TAB_OFFSET                   0x18F0

#define DCAM_SC_COEFF_BUF_SIZE                         (24 << 10)
#define DCAM_SC_COEFF_COEF_SIZE                        (1 << 10)
#define DCAM_SC_COEFF_TMP_SIZE                         (21 << 10)

#define DCAM_SC_H_COEF_SIZE                            (0xC0)
#define DCAM_SC_V_COEF_SIZE                            (0x210)
#define DCAM_SC_V_CHROM_COEF_SIZE                      (0x210)

#define DCAM_SC_COEFF_H_NUM                            (DCAM_SC_H_COEF_SIZE/4)
#define DCAM_SC_COEFF_V_NUM                            (DCAM_SC_V_COEF_SIZE/4)
#define DCAM_SC_COEFF_V_CHROMA_NUM                     (DCAM_SC_V_CHROM_COEF_SIZE/4)

#define DCAM_AXI_STOP_TIMEOUT                          100
#define DCAM_CLK_DOMAIN_AHB                            1
#define DCAM_CLK_DOMAIN_DCAM                           0

#define REG_RD(a)                                      __raw_readl(a)
#define REG_WR(a,v)                                    __raw_writel(v,a)
#define REG_AWR(a,v)                                   REG_WR(a, (REG_RD(a) & v))//__raw_writel((__raw_readl(a) & v), a)
#define REG_OWR(a,v)                                   REG_WR(a, (REG_RD(a) | v))//__raw_writel((__raw_readl(a) | v), a)
#define REG_XWR(a,v)                                   REG_WR(a, (REG_RD(a) ^ v)) //__raw_writel((__raw_readl(a) ^ v), a)
#define REG_MWR(a,m,v)                                 \
	do {                                           \
		uint32_t _tmp = REG_RD(a);        \
		_tmp &= ~(m);                          \
		REG_WR(a, _tmp | ((m) & (v))); \
	}while(0)

#define GLB_REG_RD(a)                                      sci_glb_read(a, 0xffffffff)
#define GLB_REG_WR(a,v)                                    sci_glb_write(a, v, 0xffffffff)
#define GLB_REG_AWR(a,v)                                   GLB_REG_WR(a, (GLB_REG_RD(a) & v))
#define GLB_REG_OWR(a,v)                                   GLB_REG_WR(a, (GLB_REG_RD(a) | v))
#define GLB_REG_XWR(a,v)                                   GLB_REG_WR(a, (GLB_REG_RD(a) ^ v))
#define GLB_REG_MWR(a,m,v)                                 \
	do {                                           \
		uint32_t _tmp = GLB_REG_RD(a);        \
		_tmp &= ~(m);                          \
		GLB_REG_WR(a, _tmp | ((m) & (v))); \
	}while(0)


#define DCAM_CHECK_PARAM_ZERO_POINTER(n)               \
	do {                                           \
		if (0 == (int)(n))              \
			return -DCAM_RTN_PARA_ERR;     \
	} while(0)

#define DCAM_CLEAR(a)                                  \
	do {                                           \
		memset((void *)(a), 0, sizeof(*(a)));  \
	} while(0)

#define DEBUG_STR                                      "Error L %d, %s: \n"
#define DEBUG_ARGS                                     __LINE__,__FUNCTION__
#define DCAM_RTN_IF_ERR          \
	do {                        \
		if(rtn) {                \
			printk(DEBUG_STR,DEBUG_ARGS);            \
			return -(rtn);       \
		}                        \
	} while(0)

#define DCAM_IRQ_LINE_MASK                             0x00077FFFUL  /* No ROT_DONE*/
#define DCAM_CLOCK_PARENT                              "clk_256m"

typedef void (*dcam_isr)(void);

enum {
	DCAM_FRM_UNLOCK = 0,
	DCAM_FRM_LOCK_WRITE = 0x10011001,
	DCAM_FRM_LOCK_READ  = 0x01100110
};

enum {
    DCAM_ST_STOP = 0,
    DCAM_ST_START,
};


enum {
	SN_SOF = 0,
	SN_EOF,
	CAP_SOF,
	CAP_EOF,
	PATH0_DONE,
	PATH0_OV,
	PATH1_DONE,
	PATH1_OV,
	PATH2_DONE,
	PATH2_OV,
	SN_LINE_ERR,
	SN_FRAME_ERR,
	JPEG_BUF_OV,
	ISP_OV,
	MIPI_OV,
	ROT_DONE,
	PATH1_SLICE_DONE,
	PATH2_SLICE_DONE,
	RAW_SLICE_DONE,
	IRQ_NUMBER
};

#define DCAM_IRQ_ERR_MASK										\
	((1 << PATH0_OV) | (1 << PATH1_OV) |  (1 << PATH2_OV) |		\
	(1 << SN_LINE_ERR) | (1 << SN_FRAME_ERR) |					\
	(1 << ISP_OV) | (1 << MIPI_OV))

#define DCAM_IRQ_JPEG_OV_MASK                          (1 << JPEG_BUF_OV)

struct dcam_cap_desc {
	uint32_t                   interface;
	uint32_t                   input_format;
	uint32_t                   frame_deci_factor;
	uint32_t                   img_x_deci_factor;
	uint32_t                   img_y_deci_factor;
};

struct dcam_path_valid{
	uint32_t input_size         :1;
	uint32_t input_rect         :1;
	uint32_t output_size        :1;
	uint32_t output_format      :1;
	uint32_t src_sel            :1;
	uint32_t data_endian        :1;
	uint32_t frame_deci         :1;
	uint32_t scale_tap          :1;
	uint32_t v_deci             :1;
};

struct dcam_path_desc {
	struct dcam_size           input_size;
	struct dcam_rect           input_rect;
	struct dcam_size           sc_input_size;
	struct dcam_size           output_size;
	struct dcam_frame          input_frame;
	struct dcam_frame          *output_frame_head;
	struct dcam_frame          *output_frame_cur;
	struct dcam_endian_sel     data_endian;
	struct dcam_sc_tap         scale_tap;
	struct dcam_deci           deci_val;
	struct dcam_path_valid     valid_param;
	uint32_t                   output_frame_count;
	uint32_t                   output_format;
	uint32_t                   src_sel;
	uint32_t                   rot_mode;
	uint32_t                   frame_deci;
	uint32_t                   valide;
	uint32_t                   status;
	uint32_t                   path_update;
	uint32_t                   path_update_wait;
	uint32_t                   path_done_cnt;
};

struct dcam_module {
	uint32_t                   dcam_mode;
	uint32_t                   module_addr;
	struct dcam_cap_desc       dcam_cap;
	struct dcam_path_desc      dcam_path0;
	struct dcam_path_desc      dcam_path1;
	struct dcam_path_desc      dcam_path2;
	dcam_isr_func              user_func[USER_IRQ_NUMBER];
	void                       *user_data[USER_IRQ_NUMBER];
};

#define DCAM_IRQ_DEV              0x5A0000A5

LOCAL struct dcam_frame           s_path0_frame[DCAM_PATH_0_FRM_CNT_MAX];
LOCAL struct dcam_frame           s_path1_frame[DCAM_PATH_1_FRM_CNT_MAX];
LOCAL struct dcam_frame           s_path2_frame[DCAM_PATH_2_FRM_CNT_MAX];
LOCAL atomic_t                    s_dcam_users = ATOMIC_INIT(0);
LOCAL atomic_t                    s_resize_flag = ATOMIC_INIT(0);
LOCAL struct semaphore            s_done_sema = __SEMAPHORE_INITIALIZER(s_done_sema, 0);
LOCAL struct semaphore            s_path1_update_sema = __SEMAPHORE_INITIALIZER(s_path1_update_sema, 0);
LOCAL struct semaphore            s_path_stop_sema = __SEMAPHORE_INITIALIZER(s_path_stop_sema, 0);
LOCAL uint32_t                    s_resize_wait = 0;
LOCAL uint32_t                    s_path1_wait = 0;
LOCAL uint32_t                    s_sof_cnt = 0;
LOCAL uint32_t                    s_sof_wait = 0;
LOCAL struct dcam_module          s_dcam_mod = {0};
LOCAL uint32_t                    g_dcam_irq = DCAM_IRQ_DEV;
LOCAL uint32_t                    g_dcam_irq_en = 0;
LOCAL struct clk                  *s_dcam_clk = NULL;
static struct clk                  *s_ccir_clk = NULL;
LOCAL struct clk                  *s_dcam_mipi_clk = NULL;
LOCAL uint32_t                    s_path1_done_cnt = 0;
LOCAL uint32_t                    s_path1_done_wait = 0;
LOCAL struct semaphore            s_path1_done_sema = __SEMAPHORE_INITIALIZER(s_path1_done_sema, 0);

LOCAL uint32_t                    s_path1_done_wait_dis_en = 0;
LOCAL struct semaphore            s_path1_done_dis_en_sema = __SEMAPHORE_INITIALIZER(s_path1_done_dis_en_sema, 0);


LOCAL DEFINE_MUTEX(dcam_sem);
LOCAL DEFINE_SPINLOCK(dcam_lock);

LOCAL void    _dcam_path0_set(void);
LOCAL void    _dcam_path1_set(void);
LOCAL void    _dcam_path2_set(void);
LOCAL void    _dcam_frm_clear(enum dcam_path_index path_index);
LOCAL void    _dcam_link_frm(uint32_t base_id);
LOCAL int32_t _dcam_path_set_next_frm(enum dcam_path_index path_index, uint32_t is_1st_frm);
LOCAL int32_t _dcam_path_trim(enum dcam_path_index path_index);
LOCAL int32_t _dcam_path_scaler(enum dcam_path_index path_index);
LOCAL int32_t _dcam_calc_sc_size(enum dcam_path_index path_index);
LOCAL int32_t _dcam_set_sc_coeff(enum dcam_path_index path_index);
LOCAL void    _dcam_force_copy_ext(enum dcam_path_index path_index, uint32_t path_copy, uint32_t coef_copy);
LOCAL void    _dcam_auto_copy_ext(enum dcam_path_index path_index, uint32_t path_copy, uint32_t coef_copy);
LOCAL void    _dcam_force_copy(enum dcam_path_index path_index);
LOCAL void    _dcam_auto_copy(enum dcam_path_index path_index);
LOCAL void    _dcam_reg_trace(void);
LOCAL void    _sensor_sof(void);
LOCAL void    _sensor_eof(void);
LOCAL void    _cap_sof(void);
LOCAL void    _cap_eof(void);
LOCAL void    _path0_done(void);
LOCAL void    _path0_overflow(void);
LOCAL void    _path1_done(void);
LOCAL void    _path1_overflow(void);
LOCAL void    _sensor_line_err(void);
LOCAL void    _sensor_frame_err(void);
LOCAL void    _jpeg_buf_ov(void);
LOCAL void    _path2_done(void);
LOCAL void    _path2_ov(void);
LOCAL void    _isp_ov(void);
LOCAL void    _mipi_ov(void);
LOCAL void    _path1_slice_done(void);
LOCAL void    _path2_slice_done(void);
LOCAL void    raw_slice_done(void);
LOCAL irqreturn_t dcam_isr_root(int irq, void *dev_id);
LOCAL void    _dcam_wait_for_stop(void);
LOCAL void    _dcam_stopped(void);
LOCAL int32_t _dcam_mipi_clk_en(void);
LOCAL int32_t _dcam_mipi_clk_dis(void);
LOCAL int32_t _dcam_ccir_clk_en(void);
LOCAL int32_t _dcam_ccir_clk_dis(void);
LOCAL int32_t _dcam_path_check_deci(enum dcam_path_index path_index, uint32_t *is_deci);
extern void   _dcam_isp_root(void);
LOCAL int32_t _dcam_path_check_deci(enum dcam_path_index path_index, uint32_t *is_deci);

LOCAL const dcam_isr isr_list[IRQ_NUMBER] = {
	_dcam_isp_root,
	_sensor_eof,
	_cap_sof,
	_cap_eof,
	_path0_done,
	_path0_overflow,
	_path1_done,
	_path1_overflow,
	_path2_done,
	_path2_ov,
	_sensor_line_err,
	_sensor_frame_err,
	_jpeg_buf_ov,
	_isp_ov,
	_mipi_ov,
	NULL, // rot
	_path1_slice_done,
	_path2_slice_done,
	raw_slice_done
};

void dcam_print_clock(void)
{
	uint32_t ahb_en, ahb_rst, gen_ckg_cfg;
	uint32_t clk_ahb, clk_sensor, clk_ccir, clk_dcam;

	ahb_en         = REG_RD(SPRD_MMAHB_BASE);
	ahb_rst        = REG_RD(SPRD_MMAHB_BASE + 0x4);
	gen_ckg_cfg    = REG_RD(SPRD_MMAHB_BASE + 0x8);

	clk_ahb        = REG_RD(SPRD_MMCKG_BASE + 0x20);
	clk_sensor     = REG_RD(SPRD_MMCKG_BASE + 0x24);
	clk_ccir       = REG_RD(SPRD_MMCKG_BASE + 0x28);
	clk_dcam       = REG_RD(SPRD_MMCKG_BASE + 0x2c);

	printk("dcam_print_clock: start \n");
	printk("ahb_en=0x%x, ahb_rst=0x%x, gen_ckg_cfg=0x%x \n", ahb_en, ahb_rst, gen_ckg_cfg);
	printk("clk_ahb=0x%x, clk_sensor=0x%x, clk_ccir=0x%x, clk_dcam=0x%x \n", clk_ahb, clk_sensor, clk_ccir, clk_dcam);
	printk("dcam_print_clock end \n");

}

int32_t dcam_module_init(enum dcam_cap_if_mode if_mode,
	              enum dcam_cap_sensor_mode sn_mode)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_cap_desc    *cap_desc = &s_dcam_mod.dcam_cap;
	int                     ret = 0;

	if (if_mode >= DCAM_CAP_IF_MODE_MAX) {
		rtn = -DCAM_RTN_CAP_IF_MODE_ERR;
	} else {
		if (sn_mode >= DCAM_CAP_MODE_MAX) {
			rtn = -DCAM_RTN_CAP_SENSOR_MODE_ERR;
		} else {
			DCAM_CLEAR(&s_dcam_mod);
			_dcam_link_frm(0); /* set default base frame index as 0 */
			cap_desc->interface = if_mode;
			cap_desc->input_format = sn_mode;
			/*REG_OWR(DCAM_EB, BIT_13);//MM_EB*/
			/*REG_OWR(DCAM_MATRIX_EB, BIT_10|BIT_5);*/
			if (DCAM_CAP_IF_CSI2 == if_mode) {
			/*	REG_OWR(CSI2_DPHY_EB, MIPI_EB_BIT);*/
				ret = _dcam_mipi_clk_en();
				REG_OWR(DCAM_CFG, BIT_9);
				REG_MWR(CAP_MIPI_CTRL, BIT_2 | BIT_1, sn_mode << 1);
			} else {
				/*REG_OWR(DCAM_EB, CCIR_IN_EB_BIT);
				REG_OWR(DCAM_EB, CCIR_EB_BIT);*/
				ret = _dcam_ccir_clk_en();
				REG_MWR(DCAM_CFG, BIT_9, 0 << 9);
				REG_MWR(CAP_CCIR_CTRL, BIT_2 | BIT_1, sn_mode << 1);
			}
			rtn = DCAM_RTN_SUCCESS;
		}
	}

	if(0 == g_dcam_irq_en){
		REG_WR(DCAM_INT_CLR,  DCAM_IRQ_LINE_MASK);
		REG_MWR(DCAM_INT_MASK, DCAM_IRQ_LINE_MASK, DCAM_IRQ_LINE_MASK);
		ret = request_irq(DCAM_IRQ,
				dcam_isr_root,
				IRQF_SHARED,
				"DCAM",
				&g_dcam_irq);
		if (ret) {
			DCAM_TRACE("dcam_start, error %d \n", ret);
			return -DCAM_RTN_MAX;
		}
		g_dcam_irq_en = 1;
		printk("g_dcam_irq=0x%x, g_dcam_irq_en = %d, ret=%d \n", g_dcam_irq, g_dcam_irq_en, ret);
	}

	dcam_print_clock();
/*MODULE_INIT_END:*/
	return -rtn;
}

int32_t dcam_module_deinit(enum dcam_cap_if_mode if_mode,
	              enum dcam_cap_sensor_mode sn_mode)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	if (DCAM_CAP_IF_CSI2 == if_mode) {
		/*REG_MWR(CSI2_DPHY_EB, MIPI_EB_BIT, 0 << 10);*/
		REG_MWR(DCAM_CFG, BIT_9, 0 << 9);
		_dcam_mipi_clk_dis();
	} else {
		/*REG_MWR(DCAM_EB, CCIR_IN_EB_BIT, 0 << 2);
		REG_MWR(DCAM_EB, CCIR_EB_BIT, 0 << 9);*/
		REG_MWR(DCAM_CFG, BIT_9, 0 << 9);
		_dcam_ccir_clk_dis();
	}

	printk("g_dcam_irq=0x%x, g_dcam_irq_en=%d \n", g_dcam_irq, g_dcam_irq_en);

	if (0 != g_dcam_irq_en) {
		free_irq(DCAM_IRQ, &g_dcam_irq);
		g_dcam_irq_en = 0;
		printk("free dcam irq \n");
	}

	dcam_print_clock();

	return -rtn;
}

int32_t dcam_module_en(void)
{
	int	ret = 0;

	DCAM_TRACE("DCAM DRV: dcam_module_en: %d \n", s_dcam_users.counter);
	if (atomic_inc_return(&s_dcam_users) == 1) {
		ret = dcam_set_clk(DCA_CLK_256M);
		/*REG_OWR(DCAM_EB, DCAM_EB_BIT);*/
		GLB_REG_OWR(DCAM_RST, DCAM_MOD_RST_BIT);
		GLB_REG_OWR(DCAM_RST, CCIR_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~DCAM_MOD_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~CCIR_RST_BIT);

		dcam_print_clock();
#if 0
		{
			// aiden fpga
			uint32_t bit_value;
			// 0x60d0_000
			printk("dcam_module_en: start enable module and set clock  \n");
			
			bit_value = BIT_0 | BIT_4 | BIT_6;
			REG_MWR(SPRD_MMAHB_BASE, bit_value, bit_value);  // CSI enable

			bit_value = BIT_0 | BIT_7 | BIT_8 | BIT_9;
			REG_MWR(SPRD_MMAHB_BASE+0x4, bit_value, bit_value); // reset
			REG_MWR(SPRD_MMAHB_BASE+0x4, bit_value, 0x0);

			bit_value = BIT_0 | BIT_1 | BIT_3 | BIT_7 | BIT_8;
			REG_MWR(SPRD_MMAHB_BASE+0x8, bit_value, bit_value); // ckg_cfg

			//REG_MWR(SPRD_MMCKG_BASE + 0x24, 0xfff, 0x101);  // sensor clock
			REG_MWR(SPRD_MMCKG_BASE + 0x2c, 0xf, 0x3);  // dcam clock: 76, 128, 192, 256

			REG_MWR(SPRD_MMCKG_BASE + 0x20, 0xf, 0x3);  // ahb clock: 76, 128, 192, 256
		}
#endif
		printk("dcam_module_en: end\n");
	}

/*MODULE_EN_END:*/
	return ret;
}

int32_t dcam_module_dis(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	DCAM_TRACE("DCAM DRV: dcam_module_dis: %d \n", s_dcam_users.counter);

	if (atomic_dec_return(&s_dcam_users) == 0) {
		GLB_REG_AWR(DCAM_EB, ~DCAM_EB_BIT);
		dcam_set_clk(DCAM_CLK_NONE);

		dcam_print_clock();
#if 0
		{
			// aiden fpga
			uint32_t bit_value;
			// 0x60d0_000
			printk("dcam_module_dis: start enable module and set clock  \n");


			bit_value = BIT_0 | BIT_1 | BIT_5 | BIT_7 | BIT_8 | BIT_9;
			REG_MWR(SPRD_MMAHB_BASE+0x4, bit_value, bit_value); // reset
			REG_MWR(SPRD_MMAHB_BASE+0x4, bit_value, 0x0);

			bit_value = BIT_0 | BIT_1 | BIT_2 | BIT_3;
			REG_MWR(SPRD_MMAHB_BASE+0x8, bit_value, 0); // ckg_cfg

			bit_value = BIT_0 | BIT_1| BIT_4;
			REG_MWR(SPRD_MMAHB_BASE, bit_value, 0);  // CSI enable

			//REG_MWR(SPRD_MMCKG_BASE + 0x24, 0xfff, 0x101);  // sensor clock
			//REG_MWR(SPRD_MMCKG_BASE + 0x2c, 0xf, 0x3);  // dcam clock: 76, 128, 192, 256

			printk("dcam_module_dis: end\n");
		}
#endif
	}


	printk("dcam_module_dis: end\n");
	return -rtn;
}

int32_t dcam_reset(enum dcam_rst_mode reset_mode)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	uint32_t                time_out = 0;

	//DCAM_TRACE("DCAM: dcam_reset: path=%x  start \n", reset_mode);

	if (DCAM_RST_ALL == reset_mode) {
		if (atomic_read(&s_dcam_users)) {
			/* firstly, stop AXI writing */
			REG_OWR(DCAM_AHBM_STS, BIT_6);
		}

		/* then wait for AHB busy cleared */
		while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
			if (0 == (REG_RD(DCAM_AHBM_STS) & BIT_0))
				break;
		}
		if (time_out >= DCAM_AXI_STOP_TIMEOUT)
			return DCAM_RTN_TIMEOUT;
	}

	/* do reset action */
	switch (reset_mode) {
	case DCAM_RST_PATH0:
		GLB_REG_OWR(DCAM_RST, PATH0_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~PATH0_RST_BIT);
		DCAM_TRACE("DCAM DRV: reset path0 \n");
		break;

	case DCAM_RST_PATH1:
		GLB_REG_OWR(DCAM_RST, PATH1_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~PATH1_RST_BIT);
		DCAM_TRACE("DCAM DRV: reset path1 \n");
		break;

	case DCAM_RST_PATH2:
		GLB_REG_OWR(DCAM_RST, PATH2_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~PATH2_RST_BIT);
		DCAM_TRACE("DCAM DRV: reset path2 \n");
		break;

	case DCAM_RST_ALL:
		GLB_REG_OWR(DCAM_RST, DCAM_MOD_RST_BIT);
		GLB_REG_OWR(DCAM_RST, CCIR_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~DCAM_MOD_RST_BIT);
		GLB_REG_AWR(DCAM_RST, ~CCIR_RST_BIT);
		DCAM_TRACE("DCAM DRV: reset all \n");
		break;
	default:
		rtn = DCAM_RTN_PARA_ERR;
		break;
	}

	if (DCAM_RST_ALL == reset_mode) {
		if (atomic_read(&s_dcam_users)) {
			/* the end, enable AXI writing */
			REG_AWR(DCAM_AHBM_STS, ~BIT_6);
		}
	}

	//DCAM_TRACE("DCAM: dcam_reset: path=%x  end \n", reset_mode);

	return -rtn;
}

#if 1
int32_t dcam_set_clk(enum dcam_clk_sel clk_sel)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct clk              *clk_parent;
	char                    *parent = DCAM_CLOCK_PARENT;
	int                     ret = 0;

	switch (clk_sel) {
	case DCA_CLK_256M:
		parent = DCAM_CLOCK_PARENT;
		break;
	case DCAM_CLK_128M:
		parent = "clk_128m";
		break;
	case DCAM_CLK_48M:
		parent = "clk_48m";
		break;
	case DCAM_CLK_76M8:
		parent = "clk_76p8m";
		break;

	case DCAM_CLK_NONE:
		if (s_dcam_clk) {
			clk_disable(s_dcam_clk);
			clk_put(s_dcam_clk);
		}
		printk("DCAM close CLK %d \n", (int)clk_get_rate(s_dcam_clk));
		return 0;
	default:
		parent = "clk_128m";
		break;
	}

	if (NULL == s_dcam_clk) {
		s_dcam_clk = clk_get(NULL, "clk_dcam");
		if (IS_ERR(s_dcam_clk)) {
			printk("DCAM DRV: clk_get fail, %d \n", (int)s_dcam_clk);
			return -1;
		} else {
			DCAM_TRACE("DCAM DRV: get clk_parent ok \n");
		}
	} else {
		clk_disable(s_dcam_clk);
	}

	clk_parent = clk_get(NULL, parent);
	if (IS_ERR(clk_parent)) {
		printk("DCAM DRV: dcam_set_clk fail, %d \n", (int)clk_parent);
		return -1;
	} else {
		DCAM_TRACE("DCAM DRV: get clk_parent ok \n");
	}

	ret = clk_set_parent(s_dcam_clk, clk_parent);
	if(ret){
		printk("DCAM DRV: clk_set_parent fail, %d \n", ret);
	}

	ret = clk_enable(s_dcam_clk);
	if (ret) {
		printk("enable dcam clk error.\n");
		return -1;
	}
	return rtn;
}

int32_t _dcam_mipi_clk_en(void)
{
	int                     ret = 0;
	if (NULL == s_dcam_mipi_clk) {
		s_dcam_mipi_clk = clk_get(NULL, "clk_dcam_mipi");
	}

	if (IS_ERR(s_dcam_mipi_clk)) {
		printk("DCAM DRV: get dcam mipi clk error \n");
		return -1;
	} else {
		ret = clk_enable(s_dcam_mipi_clk);
		if (ret) {
			printk("DCAM DRV: enable dcam mipi clk error %d \n", ret);
			return -1;
		}
	}

	return 0;
}

int32_t _dcam_mipi_clk_dis(void)
{
	if (s_dcam_mipi_clk) {
		clk_disable(s_dcam_mipi_clk);
		clk_put(s_dcam_mipi_clk);
	}
	return 0;
}

int32_t _dcam_ccir_clk_en(void)
{
	int                     ret = 0;

	if (NULL == s_ccir_clk) {
		s_ccir_clk = clk_get(NULL, "clk_ccir");
	}

	if (IS_ERR(s_ccir_clk)) {
		printk("DCAM DRV: get dcam ccir clk error \n");
		return -1;
	} else {
		ret = clk_enable(s_ccir_clk);
		if (ret) {
			printk("DCAM DRV: enable dcam ccir clk error %d \n", ret);
			return -1;
		}
	}
	return 0;
}

int32_t _dcam_ccir_clk_dis(void)
{
	if (s_ccir_clk) {
		clk_disable(s_ccir_clk);
		clk_put(s_ccir_clk);
	}
	return 0;
}
#else
int32_t dcam_set_clk(enum dcam_clk_sel clk_sel)
{
	return 0;
}

int32_t _dcam_mipi_clk_en(void)
{
	return 0;
}

int32_t _dcam_mipi_clk_dis(void)
{
	return 0;
}

int32_t _dcam_ccir_clk_en(void)
{
	return 0;
}

int32_t _dcam_ccir_clk_dis(void)
{
	return 0;
}

#endif
int32_t dcam_update_path(enum dcam_path_index path_index, struct dcam_size *in_size,
		struct dcam_rect *in_rect, struct dcam_size *out_size)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	unsigned long           flags;
	uint32_t                is_deci = 0;


	if ((DCAM_PATH_IDX_1 & path_index) && s_dcam_mod.dcam_path1.valide) {
		//local_irq_save(flags);
		//local_irq_restore(flags);

		rtn = dcam_path1_cfg(DCAM_PATH_INPUT_SIZE, in_size);
		DCAM_RTN_IF_ERR;
		rtn = dcam_path1_cfg(DCAM_PATH_INPUT_RECT, in_rect);
		DCAM_RTN_IF_ERR;
		rtn = dcam_path1_cfg(DCAM_PATH_OUTPUT_SIZE, out_size);
		DCAM_RTN_IF_ERR;

		rtn = _dcam_path_check_deci(DCAM_PATH_IDX_1, &is_deci);
		DCAM_RTN_IF_ERR;
		if ((s_dcam_mod.dcam_path1.deci_val.deci_x_en != is_deci) || is_deci) {
			DCAM_TRACE("DCAM DRV: dcam_update_path 1: deci cur=%d, will_set_to=%d \n",
				s_dcam_mod.dcam_path1.deci_val.deci_x_en, is_deci);
			dcam_stop_path(DCAM_PATH_IDX_1);
			s_dcam_mod.dcam_path1.output_frame_count = DCAM_PATH_1_FRM_CNT_MAX;
			s_dcam_mod.dcam_path1.valide = 1;
			dcam_start_path(DCAM_PATH_IDX_1);

			s_path1_done_cnt = 1;
			s_path1_done_wait = 1;
			rtn = down_timeout(&s_path1_done_sema, msecs_to_jiffies(500));
			if (rtn) {
				s_path1_done_wait = 0;
				printk("DCAM DRV: Wait path1 done: Failed wait s_path1_done_sema \n");
			} else {
				DCAM_TRACE("DCAM DRV: wait s_path1_done_sema done \n");
			}

		} else {

			rtn = _dcam_path_scaler(DCAM_PATH_IDX_1);
			DCAM_RTN_IF_ERR;

			//local_irq_save(flags);
			s_dcam_mod.dcam_path1.path_update_wait = 1;
			s_dcam_mod.dcam_path1.path_done_cnt = 0;
			s_dcam_mod.dcam_path1.path_update = 1;
			DCAM_TRACE("DCAM DRV: dcam_update_path 1:  waiting \n");
			rtn = down_timeout(&s_path1_update_sema, msecs_to_jiffies(500));
			if (rtn) {
				printk("DCAM DRV: failed wait s_path1_update_sema \n");
				return rtn;
			}
			DCAM_TRACE("DCAM DRV: dcam_update_path 1:  waiting done \n");
		}
		//local_irq_restore(flags);
	}

	if ((DCAM_PATH_IDX_2 & path_index) && s_dcam_mod.dcam_path2.valide) {
		local_irq_save(flags);
		if(s_dcam_mod.dcam_path2.path_update){
			local_irq_restore(flags);
			DCAM_TRACE("DCAM DRV: dcam_update_path 2:  updating return \n");
			return rtn;
		}
		local_irq_restore(flags);

		rtn = dcam_path2_cfg(DCAM_PATH_INPUT_SIZE, in_size);
		DCAM_RTN_IF_ERR;
		rtn = dcam_path2_cfg(DCAM_PATH_INPUT_RECT, in_rect);
		DCAM_RTN_IF_ERR;
		rtn = dcam_path2_cfg(DCAM_PATH_OUTPUT_SIZE, out_size);
		DCAM_RTN_IF_ERR;

		rtn = _dcam_path_scaler(DCAM_PATH_IDX_2);
		DCAM_RTN_IF_ERR;

		local_irq_save(flags);
		s_dcam_mod.dcam_path2.path_update = 1;
		s_dcam_mod.dcam_path2.path_done_cnt = 0;
		local_irq_restore(flags);
	}

	DCAM_TRACE("DCAM DRV: dcam_update_path: done \n");

	return -rtn;
}

int32_t dcam_start_path(enum dcam_path_index path_index)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	int                     ret = 0;
	uint32_t                cap_en = 0;
	uint32_t                is_deci = 0;

	DCAM_TRACE("DCAM DRV: dcam_start_path: path=%d, mode=%x \n", path_index, s_dcam_mod.dcam_mode);

#if 0//def DCAM_DEBUG
	REG_MWR(CAP_CCIR_FRM_CTRL, BIT_5 | BIT_4, 1 << 4);
	REG_MWR(CAP_MIPI_FRM_CTRL, BIT_5 | BIT_4, 1 << 4);
#endif

	REG_OWR(DCAM_AHBM_STS, BIT_8); // aiden add: write arbit mode
	
	cap_en = REG_RD(DCAM_CONTROL) & BIT_2;
	
	if ((DCAM_PATH_IDX_0 & path_index) && s_dcam_mod.dcam_path0.valide) {
		
		_dcam_path0_set();
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_0, true);
		DCAM_RTN_IF_ERR;
		if (cap_en) {
			/* if cap is already open, the sequence is: 
			   cap force copy -> path 0 enable -> cap auto copy */
			REG_MWR(DCAM_CONTROL, BIT_0, 1 << 0); /* Cap force copy */
			REG_OWR(DCAM_CFG, BIT_0);             /* Enable Path 0 */
			REG_MWR(DCAM_CONTROL, BIT_1, 1 << 1); /* Cap auto copy, trigger path 0 enable */
		} else {
			REG_OWR(DCAM_CFG, BIT_0);             /* Enable Path 0 */
		}
	}

	if ((DCAM_PATH_IDX_1 & path_index) && s_dcam_mod.dcam_path1.valide) {
		
		//rtn = _dcam_path_trim(DCAM_PATH_IDX_1); 
		//DCAM_RTN_IF_ERR;
		rtn = _dcam_path_scaler(DCAM_PATH_IDX_1);
		DCAM_RTN_IF_ERR;

		_dcam_path1_set();
		DCAM_TRACE("DCAM DRV: start path: path_control=%x \n", REG_RD(DCAM_CONTROL));

		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, true);
		DCAM_RTN_IF_ERR;
		_dcam_force_copy_ext(DCAM_PATH_IDX_1, true, true);
		DCAM_TRACE("DCAM:int= %x \n", REG_RD(DCAM_INT_STS));
		REG_OWR(DCAM_CFG, BIT_1);
	}

	if ((DCAM_PATH_IDX_2 & path_index) && s_dcam_mod.dcam_path2.valide) {
		rtn = _dcam_path_check_deci(DCAM_PATH_IDX_2, &is_deci);
		DCAM_RTN_IF_ERR;
		if (0 == is_deci) {
			if (s_dcam_mod.dcam_path2.input_rect.h >= DCAM_HEIGHT_MIN) {
				s_dcam_mod.dcam_path2.input_rect.h--;
			}
		}

		rtn = _dcam_path_scaler(DCAM_PATH_IDX_2);
		DCAM_RTN_IF_ERR;

		_dcam_path2_set();

		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, true);
		DCAM_RTN_IF_ERR;
		_dcam_force_copy_ext(DCAM_PATH_IDX_2, true, true);
		DCAM_TRACE("DCAM:int= %x \n", REG_RD(DCAM_INT_STS));
		REG_OWR(DCAM_INT_MASK, BIT_8);
		REG_OWR(DCAM_CFG, BIT_2);
		//REG_OWR(DCAM_BURST_GAP, BIT_20); // aiden todo
	}

	_dcam_reg_trace();

	printk("DCAM PATH S: %d \n", path_index);

	
	if(0 == cap_en){
		REG_MWR(DCAM_CONTROL, BIT_0, 1 << 0); /* Cap force copy */
		//REG_MWR(DCAM_CONTROL, BIT_1, 1 << 1); /* Cap auto  copy */
		REG_MWR(DCAM_CONTROL, BIT_2, 1 << 2); /* Cap Enable */
	}

	if ((DCAM_PATH_IDX_0 & path_index) && s_dcam_mod.dcam_path0.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_0, false);
		DCAM_RTN_IF_ERR;
		_dcam_auto_copy(DCAM_PATH_IDX_0);
		s_dcam_mod.dcam_path0.status = DCAM_ST_START;
	}

	if ((DCAM_PATH_IDX_1 & path_index) && s_dcam_mod.dcam_path1.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, false);
		DCAM_RTN_IF_ERR;
		_dcam_auto_copy(DCAM_PATH_IDX_1);
		s_dcam_mod.dcam_path1.status = DCAM_ST_START;
	}

	if ((DCAM_PATH_IDX_2 & path_index) && s_dcam_mod.dcam_path2.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, false);
		DCAM_RTN_IF_ERR;
		_dcam_auto_copy(DCAM_PATH_IDX_2);
		s_dcam_mod.dcam_path2.status = DCAM_ST_START;
	}

	return -rtn;
}

int32_t dcam_start(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	int                     ret = 0;

	DCAM_TRACE("DCAM DRV: dcam_start %x \n", s_dcam_mod.dcam_mode);

	ret = dcam_start_path(DCAM_PATH_IDX_ALL);
	//ret = dcam_start_path(DCAM_PATH_IDX_1);
	
	return -rtn;
}

int32_t dcam_stop_cap(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	DCAM_TRACE("DCAM DRV: dcam_stop_cap, dcam_mode: %d \n", s_dcam_mod.dcam_mode);	
	/* CAP_EB disable */
	//REG_AWR(DCAM_CONTROL, ~BIT_2); /* Cap Enable */
	REG_MWR(DCAM_CONTROL, BIT_2, 0); /* Cap Enable */
	//_dcam_wait_for_stop();
	DCAM_TRACE("DCAM DRV: dcam_stop_cap, s_resize_flag %d \n", atomic_read(&s_resize_flag));
	if (atomic_read(&s_resize_flag)) {
		s_resize_wait = 1;
		/* resize started , wait for it going to the end*/
		DCAM_TRACE("DCAM DRV: dcam_stop, wait: %d \n", s_done_sema.count);
		rtn = down_interruptible(&s_done_sema);
	}

	dcam_reset(DCAM_RST_ALL);

	DCAM_TRACE("DCAM DRV: dcam_stop_cap, exit from wait: %d \n", s_done_sema.count);

	return -rtn;
}

void _dcam_set_path1_stop()
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	s_path1_done_wait_dis_en = 1;
	{
		rtn = down_timeout(&s_path1_done_dis_en_sema, msecs_to_jiffies(500));
		if (rtn) {
			s_path1_done_wait_dis_en = 0;
			printk("DCAM DRV: Failed wait s_path1_done_dis_en_sema \n");
		} else {
			DCAM_TRACE("DCAM DRV: wait s_path1_done_dis_en_sema done \n");
		}
	}
}

void _dcam_wait_sof()
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	s_sof_cnt = 0;
	s_sof_wait = 1;
	rtn = down_timeout(&s_path_stop_sema, msecs_to_jiffies(500));
	if (rtn) {
		s_sof_wait = 0;
		printk("DCAM DRV: Failed wait s_path_stop_sema \n");
	} else {
		DCAM_TRACE("DCAM DRV: wait path stop done \n");
	}

}

int32_t dcam_stop_path(enum dcam_path_index path_index)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	uint32_t                stop_cap = 0;
	uint32_t                wait_path_stop = 0;
	uint32_t                time_out = 0;
	uint32_t                path_1_valid = 0;
	uint32_t                path_2_valid = 0;
	uint32_t                path_0_valid = 0;
	uint32_t                status;
	uint32_t                path_0_status;
	uint32_t                path_1_status;
	uint32_t                path_2_status;

	path_0_status = s_dcam_mod.dcam_path0.status;
	path_1_status = s_dcam_mod.dcam_path1.status;
	path_2_status = s_dcam_mod.dcam_path2.status;

	DCAM_TRACE("DCAM DRV: dcam_stop_path=0x%x \n", path_index);

	if(DCAM_PATH_IDX_0 & path_index){
		path_0_status = DCAM_ST_STOP;
	}
	if(DCAM_PATH_IDX_1 & path_index){
		path_1_status = DCAM_ST_STOP;
	}
	if(DCAM_PATH_IDX_2 & path_index){
		path_2_status = DCAM_ST_STOP;
	}

	if((DCAM_ST_STOP == path_0_status) &&
	(DCAM_ST_STOP == path_1_status) &&
	(DCAM_ST_STOP == path_2_status)){
		stop_cap = 1;
	}

#if 0
	if(stop_cap){
		/* CAP_EB disable */
		if (DCAM_CAPTURE_MODE_MULTIPLE == s_dcam_mod.dcam_mode) {
			REG_AWR(DCAM_CONTROL, ~BIT_2); /* Cap Enable */
			_dcam_wait_for_stop();
			if (atomic_read(&s_resize_flag)) {
				s_resize_wait = 1;
				/* resize started , wait for it going to the end*/
				DCAM_TRACE("DCAM DRV: dcam_stop, wait: %d \n", s_done_sema.count);
				rtn = down_interruptible(&s_done_sema);
			}
		}

		if(DCAM_IRQ_NONE != g_dcam_irq){
			free_irq(DCAM_IRQ, &g_dcam_irq);
			g_dcam_irq = DCAM_IRQ_NONE;
		}

		DCAM_TRACE("DCAM DRV: dcam_stop, exit from wait: %d \n", s_done_sema.count);
	}
#endif
	path_0_valid = s_dcam_mod.dcam_path0.valide;
	path_1_valid = s_dcam_mod.dcam_path1.valide;
	path_2_valid = s_dcam_mod.dcam_path2.valide;

	if ((DCAM_PATH_IDX_0 & path_index) && s_dcam_mod.dcam_path0.valide) {
		s_dcam_mod.dcam_path0.valide = 0;
		wait_path_stop = 1;
		
		/* PATH0_EB disable */
		REG_AWR(DCAM_CFG, ~BIT_0);
	}
	if ((DCAM_PATH_IDX_1 & path_index) && s_dcam_mod.dcam_path1.valide) {
		DCAM_TRACE("DCAM DRV: before diable path: path_control=%x \n", REG_RD(DCAM_CONTROL));
		s_dcam_mod.dcam_path1.valide = 0;
		wait_path_stop = 1;
		if (DCAM_CAPTURE_MODE_MULTIPLE == s_dcam_mod.dcam_mode) {
			_dcam_set_path1_stop();
		}
		DCAM_TRACE("DCAM DRV: after path1 done: path_control=%x \n", REG_RD(DCAM_CONTROL));
	}

	if ((DCAM_PATH_IDX_2 & path_index) && s_dcam_mod.dcam_path2.valide) {
		s_dcam_mod.dcam_path2.valide = 0;
		wait_path_stop = 1;

		/* PATH2_EB disable */
		REG_AWR(DCAM_CFG, ~BIT_2);
	}

	DCAM_TRACE("DCAM DRV: wait path stop, wait_path_stop=%d, dcam_mode=%d \n", wait_path_stop, s_dcam_mod.dcam_mode);

	if (wait_path_stop && (DCAM_CAPTURE_MODE_MULTIPLE == s_dcam_mod.dcam_mode)) {
		_dcam_wait_sof();
	}

	DCAM_TRACE("DCAM DRV: after diable path: path_control=%x \n", REG_RD(DCAM_CONTROL));

	if ((DCAM_PATH_IDX_0 & path_index) && path_0_valid) {
		dcam_reset(DCAM_RST_PATH0);
		_dcam_frm_clear(DCAM_PATH_IDX_0);
	}
	if ((DCAM_PATH_IDX_1 & path_index) && path_1_valid) {
		dcam_reset(DCAM_RST_PATH1);
		_dcam_frm_clear(DCAM_PATH_IDX_1);
	}

	if ((DCAM_PATH_IDX_2 & path_index) && path_2_valid) {
		dcam_reset(DCAM_RST_PATH2);
		_dcam_frm_clear(DCAM_PATH_IDX_2);
	}


#if 0
	if(stop_cap){
		DCAM_TRACE("DCAM DRV: stop_cap=0x%x, try to reset all \n", stop_cap);
		dcam_stop_cap();
		dcam_reset(DCAM_RST_ALL);
	}
#endif

	if(DCAM_PATH_IDX_0 & path_index){
		s_dcam_mod.dcam_path0.status = DCAM_ST_STOP;
	}
	if(DCAM_PATH_IDX_1 & path_index){
		s_dcam_mod.dcam_path1.status = DCAM_ST_STOP;
	}
	if(DCAM_PATH_IDX_2 & path_index){
		s_dcam_mod.dcam_path2.status = DCAM_ST_STOP;
	}

	printk("DCAM dcam_stop_path E: %d \n", path_index);

	return -rtn;
}
int32_t dcam_stop(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	rtn = dcam_stop_path(DCAM_PATH_IDX_ALL);

	printk("DCAM dcam_stop E \n");

	return -rtn;
}

int32_t dcam_resume(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	if (s_dcam_mod.dcam_path1.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, true);
		DCAM_RTN_IF_ERR;
		_dcam_force_copy(DCAM_PATH_IDX_1);
		_dcam_frm_clear(DCAM_PATH_IDX_1);
	}

	if (s_dcam_mod.dcam_path2.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, true);
		DCAM_RTN_IF_ERR;
		_dcam_force_copy(DCAM_PATH_IDX_2);
		_dcam_frm_clear(DCAM_PATH_IDX_2);
	}


	if (s_dcam_mod.dcam_path1.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, false);
		DCAM_RTN_IF_ERR;
		REG_OWR(DCAM_CFG, BIT_1);
		_dcam_auto_copy_ext(DCAM_PATH_IDX_1, true, true);
	}

	if (s_dcam_mod.dcam_path2.valide) {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, false);
		DCAM_RTN_IF_ERR;
		REG_OWR(DCAM_CFG, BIT_2);
		_dcam_auto_copy_ext(DCAM_PATH_IDX_2, true, true);
	}

	printk("DCAM R \n");

	REG_OWR(DCAM_CONTROL, BIT_2); /* Enable CAP */
	return -rtn;
}

int32_t dcam_pause(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;

	REG_AWR(DCAM_CONTROL, ~BIT_2); /* Disable CAP */
	printk("DCAM P \n");
	return -rtn;
}

int32_t dcam_reg_isr(enum dcam_irq_id id, dcam_isr_func user_func, void* user_data)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	unsigned long           flag;

	if(id >= USER_IRQ_NUMBER) {
		rtn = DCAM_RTN_ISR_ID_ERR;
	} else {
		spin_lock_irqsave(&dcam_lock, flag);
		s_dcam_mod.user_func[id] = user_func;
		s_dcam_mod.user_data[id] = user_data;
		spin_unlock_irqrestore(&dcam_lock, flag);
	}
	return -rtn;
}

int32_t dcam_cap_cfg(enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_cap_desc    *cap_desc = &s_dcam_mod.dcam_cap;

	switch (id) {
	case DCAM_CAP_SYNC_POL:
	{
		struct dcam_cap_sync_pol *sync_pol = (struct dcam_cap_sync_pol*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_CAP_IF_CCIR == cap_desc->interface) {

			if (sync_pol->vsync_pol > 1 ||
			    sync_pol->hsync_pol > 1 ||
			    sync_pol->pclk_pol > 1) {
				rtn = DCAM_RTN_CAP_SYNC_POL_ERR;
			} else {
				REG_MWR(CAP_CCIR_CTRL, BIT_3, sync_pol->hsync_pol << 3);
				REG_MWR(CAP_CCIR_CTRL, BIT_4, sync_pol->vsync_pol << 4);
				//REG_MWR(CLK_DLY_CTRL,  BIT_19, sync_pol->pclk_pol << 19); // aiden todo
			}
		} else {
			if (sync_pol->need_href) {
				REG_MWR(CAP_MIPI_CTRL, BIT_5, 1 << 5);
			} else {
				REG_MWR(CAP_MIPI_CTRL, BIT_5, 0 << 5);
			}
		}
		break;
	}

	case DCAM_CAP_DATA_BITS:
	{
		enum dcam_cap_data_bits bits = *(enum dcam_cap_data_bits*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_CAP_IF_CCIR == cap_desc->interface) {
			if (DCAM_CAP_8_BITS == bits || DCAM_CAP_10_BITS == bits) {
				REG_MWR(CAP_CCIR_CTRL,  BIT_10 | BIT_9, 0 << 9);
			} else if (DCAM_CAP_4_BITS == bits) {
				REG_MWR(CAP_CCIR_CTRL,  BIT_10 | BIT_9, 1 << 9);
			} else if (DCAM_CAP_2_BITS == bits) {
				REG_MWR(CAP_CCIR_CTRL,  BIT_10 | BIT_9, 2 << 9);
			} else if (DCAM_CAP_1_BITS == bits) {
				REG_MWR(CAP_CCIR_CTRL,  BIT_10 | BIT_9, 3 << 9);
			} else {
				rtn = DCAM_RTN_CAP_IN_BITS_ERR;
			}

		} else {
			if (DCAM_CAP_12_BITS == bits) {
				REG_MWR(CAP_MIPI_CTRL,  BIT_4 | BIT_3, 2 << 3);
			} else if (DCAM_CAP_10_BITS == bits) {
				REG_MWR(CAP_MIPI_CTRL,  BIT_4 | BIT_3, 1 << 3);
			} else if (DCAM_CAP_8_BITS == bits) {
				REG_MWR(CAP_MIPI_CTRL,  BIT_4 | BIT_3, 0 << 3);
			} else {
				rtn = DCAM_RTN_CAP_IN_BITS_ERR;
			}
		}
		break;
	}

	case DCAM_CAP_YUV_TYPE:
	{
		enum dcam_cap_pattern pat = *(enum dcam_cap_pattern*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (pat < DCAM_PATTERN_MAX) {
			if (DCAM_CAP_IF_CCIR == cap_desc->interface)
				REG_MWR(CAP_CCIR_CTRL, BIT_8 | BIT_7, pat << 7);
			else
				REG_MWR(CAP_MIPI_CTRL, BIT_8 | BIT_7, pat << 7);
		} else {
			rtn = DCAM_RTN_CAP_IN_YUV_ERR;
		}
		break;
	}

	case DCAM_CAP_PRE_SKIP_CNT:
	{
		uint32_t skip_num = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (skip_num > DCAM_CAP_SKIP_FRM_MAX) {
			rtn = DCAM_RTN_CAP_SKIP_FRAME_ERR;
		} else {
			if (DCAM_CAP_IF_CCIR == cap_desc->interface)
				REG_MWR(CAP_CCIR_FRM_CTRL, BIT_3 | BIT_2 | BIT_1 | BIT_0, skip_num);
			else
				REG_MWR(CAP_MIPI_FRM_CTRL, BIT_3 | BIT_2 | BIT_1 | BIT_0, skip_num);
		}
		break;
	}

	case DCAM_CAP_FRM_DECI:
	{
		uint32_t deci_factor = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (deci_factor < DCAM_FRM_DECI_FAC_MAX) {
			if (DCAM_CAP_IF_CCIR == cap_desc->interface)
				REG_MWR(CAP_CCIR_FRM_CTRL, BIT_5 | BIT_4, deci_factor << 4);
			else
				REG_MWR(CAP_MIPI_FRM_CTRL, BIT_5 | BIT_4, deci_factor << 4);
		} else {
			rtn = DCAM_RTN_CAP_FRAME_DECI_ERR;
		}
		break;
	}

	case DCAM_CAP_FRM_COUNT_CLR:
		if (DCAM_CAP_IF_CCIR == cap_desc->interface)
			REG_MWR(CAP_CCIR_FRM_CTRL, BIT_22, 1 << 22);
		else
			REG_MWR(CAP_MIPI_FRM_CTRL, BIT_22, 1 << 22);
		break;

	case DCAM_CAP_INPUT_RECT:
	{
		struct dcam_rect *rect = (struct dcam_rect*)param;
		uint32_t         tmp = 0;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);
		if (rect->x > DCAM_CAP_FRAME_WIDTH_MAX ||
		rect->y > DCAM_CAP_FRAME_HEIGHT_MAX ||
		rect->w > DCAM_CAP_FRAME_WIDTH_MAX ||
		rect->h > DCAM_CAP_FRAME_HEIGHT_MAX ) {
			rtn = DCAM_RTN_CAP_FRAME_SIZE_ERR;
			return -rtn;
		}

		if (DCAM_CAP_IF_CCIR == cap_desc->interface) {
			if (DCAM_CAP_MODE_RAWRGB == cap_desc->input_format) {
				tmp = rect->x | (rect->y << 16);
				REG_WR(CAP_CCIR_START, tmp);
				tmp = (rect->x + rect->w - 1);
				tmp |= (rect->y + rect->h - 1) << 16;
				REG_WR(CAP_CCIR_END, tmp);
			} else {
				tmp = (rect->x << 1) | (rect->y << 16);
				REG_WR(CAP_CCIR_START, tmp);
				tmp = ((rect->x + rect->w) << 1) - 1;
				tmp |= (rect->y + rect->h - 1) << 16;
				REG_WR(CAP_CCIR_END, tmp);
			}
		} else {
			tmp = rect->x | (rect->y << 16);
			REG_WR(CAP_MIPI_START, tmp);
			tmp = (rect->x + rect->w - 1);
			tmp |= (rect->y + rect->h - 1) << 16;
			REG_WR(CAP_MIPI_END, tmp);
		}
		break;
	}

	case DCAM_CAP_IMAGE_XY_DECI:
	{
		struct dcam_cap_dec *cap_dec = (struct dcam_cap_dec*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (cap_dec->x_factor > DCAM_CAP_X_DECI_FAC_MAX ||
		cap_dec->y_factor > DCAM_CAP_Y_DECI_FAC_MAX ) {
			rtn = DCAM_RTN_CAP_XY_DECI_ERR;
		} else {
			if (DCAM_CAP_MODE_RAWRGB == cap_desc->input_format) {
				if (cap_dec->x_factor > 1 ||
				    cap_dec->y_factor > 1) {
					rtn = DCAM_RTN_CAP_XY_DECI_ERR;
				}
			}
			if (DCAM_CAP_IF_CCIR == cap_desc->interface) {
				REG_MWR(CAP_CCIR_IMG_DECI, BIT_1 | BIT_0, cap_dec->x_factor);
				REG_MWR(CAP_CCIR_IMG_DECI, BIT_3 | BIT_2, cap_dec->y_factor << 2);
			} else {
				if (DCAM_CAP_MODE_RAWRGB == cap_desc->input_format) {
					// REG_MWR(CAP_MIPI_IMG_DECI, BIT_0, cap_dec->x_factor); // for camera path
					REG_MWR(CAP_MIPI_IMG_DECI, BIT_1, cap_dec->x_factor << 1);//for ISP
				} else {
					REG_MWR(CAP_MIPI_IMG_DECI, BIT_1 | BIT_0, cap_dec->x_factor);
					REG_MWR(CAP_MIPI_IMG_DECI, BIT_3 | BIT_2, cap_dec->y_factor << 2);
				}
			}
		}

		break;
	}

	case DCAM_CAP_JPEG_SET_BUF_LEN:
	{
		uint32_t jpg_buf_size = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);
		jpg_buf_size = jpg_buf_size/DCAM_JPG_BUF_UNIT;
		if (jpg_buf_size >= DCAM_JPG_UNITS) {
			rtn = DCAM_RTN_CAP_JPEG_BUF_LEN_ERR;
		} else {
			if (DCAM_CAP_IF_CCIR == cap_desc->interface)
				REG_WR(CAP_CCIR_JPG_CTRL,jpg_buf_size);
			else
				REG_WR(CAP_MIPI_JPG_CTRL,jpg_buf_size);
		}
		break;
	}

	case DCAM_CAP_TO_ISP:
	{
		uint32_t need_isp = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (need_isp) {
			REG_MWR(DCAM_CFG, BIT_7, 1 << 7);
		} else {
			REG_MWR(DCAM_CFG, BIT_7, 0 << 7);
		}
		break;
	}

	case DCAM_CAP_DATA_PACKET:
	{
		uint32_t is_loose = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_CAP_IF_CSI2 == cap_desc->interface &&
			DCAM_CAP_MODE_RAWRGB == cap_desc->input_format) {
			if (is_loose) {
				REG_MWR(CAP_MIPI_CTRL, BIT_0, 1);
			} else {
				REG_MWR(CAP_MIPI_CTRL, BIT_0, 0);
			}
		} else {
			rtn = DCAM_RTN_MODE_ERR;
		}

		break;
	}

	case DCAM_CAP_SAMPLE_MODE:
	{
		enum dcam_capture_mode samp_mode = *(enum dcam_capture_mode*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (samp_mode >= DCAM_CAPTURE_MODE_MAX) {
			rtn = DCAM_RTN_MODE_ERR;
		} else {
			if (DCAM_CAP_IF_CSI2 == cap_desc->interface) {
				REG_MWR(CAP_MIPI_CTRL, BIT_6, samp_mode << 6);
			} else {
				REG_MWR(CAP_CCIR_CTRL, BIT_6, samp_mode << 6);
			}
			s_dcam_mod.dcam_mode = samp_mode;
		}
		break;
	}
	default:
		rtn = DCAM_RTN_IO_ID_ERR;
		break;

	}

	return -rtn;
}

int32_t dcam_cap_get_info(enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_cap_desc    *cap_desc = &s_dcam_mod.dcam_cap;

	DCAM_CHECK_PARAM_ZERO_POINTER(param);

	switch(id) {
		case DCAM_CAP_FRM_COUNT_GET:
			if (DCAM_CAP_IF_CSI2 == cap_desc->interface) {
				*(uint32_t*)param = REG_RD(CAP_MIPI_FRM_CTRL) >> 16;
			} else {
				*(uint32_t*)param = REG_RD(CAP_CCIR_FRM_CTRL) >> 16;
			}
			break;

		case DCAM_CAP_JPEG_GET_LENGTH:
			if (DCAM_CAP_IF_CSI2 == cap_desc->interface) {
				*(uint32_t*)param = REG_RD(CAP_MIPI_FRM_SIZE);
			} else {
				*(uint32_t*)param = REG_RD(CAP_CCIR_FRM_SIZE);
			}
			break;
		default:
			rtn = DCAM_RTN_IO_ID_ERR;
			break;
	}
	return -rtn;
}

int32_t dcam_path0_cfg(enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path0;

	switch (id) {

	case DCAM_PATH_INPUT_SIZE:
	{
		struct dcam_size *size = (struct dcam_size*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH0_INPUT_SIZE {%d %d} \n", size->w, size->h);
		if (size->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		size->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_SRC_SIZE_ERR;
		} else {
			path->input_size.w = size->w;
			path->input_size.h = size->h;
			path->valid_param.input_size = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_FORMAT:
	{
		enum dcam_output_mode format = *(enum dcam_output_mode*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if((DCAM_OUTPUT_WORD != format) && 
		(DCAM_OUTPUT_HALF_WORD != format)){
			rtn = DCAM_RTN_PATH_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}else{
			path->output_format = format;
			path->valid_param.output_format = 1;
		}
		break;
	}

	case DCAM_PATH_FRAME_BASE_ID:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          base_id = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_BASE_ID 0x%x \n", base_id);
		for (i = 0; i < DCAM_PATH_0_FRM_CNT_MAX; i++) {
			frame->fid = base_id + i;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_FRAME_TYPE:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          frm_type = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_TYPE 0x%x \n", frm_type);
		for (i = 0; i < DCAM_PATH_0_FRM_CNT_MAX; i++) {
			frame->type = frm_type;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_ADDR:
	{
		struct dcam_addr *p_addr = (struct dcam_addr*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_YUV_ADDR_INVALIDE(p_addr->yaddr, p_addr->uaddr, p_addr->vaddr)) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			if (path->output_frame_count > DCAM_PATH_0_FRM_CNT_MAX - 1) {
				rtn = DCAM_RTN_PATH_FRAME_TOO_MANY;
			} else {
				path->output_frame_cur->yaddr = p_addr->yaddr;
				path->output_frame_cur->uaddr = p_addr->uaddr;
				path->output_frame_cur->vaddr = p_addr->vaddr;
				path->output_frame_cur = path->output_frame_cur->next;
				DCAM_TRACE("DCAM DRV: Path 0 DCAM_PATH_OUTPUT_ADDR, i=%d, y=0x%x, u=0x%x, v=0x%x \n",
					path->output_frame_count, p_addr->yaddr, p_addr->uaddr, p_addr->vaddr);

				path->output_frame_count ++;
			}
		}
		break;
	}

	case DCAM_PATH_FRM_DECI:
	{
		uint32_t deci_factor = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
			rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
		} else {
			path->frame_deci = deci_factor;
			path->valid_param.frame_deci = 1;
		}
		break;
	}

	case DCAM_PATH_ENABLE:
	{
		DCAM_CHECK_PARAM_ZERO_POINTER(param);
		path->valide = *(uint32_t*)param;
		break;
	}

	default:
		rtn = DCAM_RTN_IO_ID_ERR;
		break;
	}
	
	return -rtn;
}

int32_t dcam_path1_cfg(enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path1;

	switch (id) {

	case DCAM_PATH_INPUT_SIZE:
	{
		struct dcam_size *size = (struct dcam_size*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH1_INPUT_SIZE {%d %d} \n", size->w, size->h);
		if (size->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		size->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_SRC_SIZE_ERR;
		} else {
			path->input_size.w = size->w;
			path->input_size.h = size->h;
			path->valid_param.input_size = 1;
		}
		break;
	}

	case DCAM_PATH_INPUT_RECT:
	{
		struct dcam_rect *rect = (struct dcam_rect*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH1_INPUT_RECT  {%d %d %d %d} \n",
			rect->x,
			rect->y,
			rect->w,
			rect->h);

		if (rect->x > DCAM_PATH_FRAME_WIDTH_MAX ||
		rect->y > DCAM_PATH_FRAME_HEIGHT_MAX ||
		rect->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		rect->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			memcpy((void*)&path->input_rect,
				(void*)rect,
				sizeof(struct dcam_rect));
			//if (path->input_rect.h >= DCAM_HEIGHT_MIN) {
			//	path->input_rect.h--;
			//}
			path->valid_param.input_rect = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_SIZE:
	{
		struct dcam_size *size = (struct dcam_size*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH1_OUTPUT_SIZE {%d %d} \n", size->w, size->h);
		if (size->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		size->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_SRC_SIZE_ERR;
		} else {
			path->output_size.w = size->w;
			path->output_size.h = size->h;
			path->valid_param.output_size = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_FORMAT:
	{
		enum dcam_fmt format = *(enum dcam_fmt*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		path->output_format = format;
		
		if((DCAM_YUV422 != format) && 
		(DCAM_YUV420 != format) && 
		(DCAM_YUV420_3FRAME != format)){
			rtn = DCAM_RTN_PATH_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}else{
			path->valid_param.output_format = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_ADDR:
	{
		struct dcam_addr *p_addr = (struct dcam_addr*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_YUV_ADDR_INVALIDE(p_addr->yaddr, p_addr->uaddr, p_addr->vaddr)) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			if (path->output_frame_count > DCAM_PATH_1_FRM_CNT_MAX - 1) {
				rtn = DCAM_RTN_PATH_FRAME_TOO_MANY;
			} else {
				path->output_frame_cur->yaddr = p_addr->yaddr;
				path->output_frame_cur->uaddr = p_addr->uaddr;
				path->output_frame_cur->vaddr = p_addr->vaddr;
				path->output_frame_cur = path->output_frame_cur->next;
				DCAM_TRACE("DCAM DRV: Path 1 DCAM_PATH_OUTPUT_ADDR, i=%d, y=0x%x, u=0x%x, v=0x%x \n",
					path->output_frame_count, p_addr->yaddr, p_addr->uaddr, p_addr->vaddr);

				path->output_frame_count ++;
			}
		}
		break;
	}

	case DCAM_PATH_SRC_SEL:
	{
		uint32_t       src_sel = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (src_sel >= DCAM_PATH_FROM_NONE) {
			rtn = DCAM_RTN_PATH_SRC_ERR;
		} else {
			path->src_sel = src_sel;
			path->valid_param.src_sel = 1;
		}
		break;
	}

	case DCAM_PATH_FRAME_BASE_ID:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          base_id = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_BASE_ID 0x%x \n", base_id);
		for (i = 0; i < DCAM_PATH_1_FRM_CNT_MAX; i++) {
			frame->fid = base_id + i;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_DATA_ENDIAN:
	{
		struct dcam_endian_sel *endian = (struct dcam_endian_sel*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (endian->y_endian >= DCAM_ENDIAN_MAX ||
			endian->uv_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian  = endian->y_endian;
			path->data_endian.uv_endian = endian->uv_endian;
			path->valid_param.data_endian = 1;
		}
		break;
	}

	case DCAM_PATH_ENABLE:
	{
		DCAM_CHECK_PARAM_ZERO_POINTER(param);
		path->valide = *(uint32_t*)param;
		break;
	}

	case DCAM_PATH_FRAME_TYPE:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          frm_type = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_TYPE 0x%x \n", frm_type);
		for (i = 0; i < DCAM_PATH_1_FRM_CNT_MAX; i++) {
			frame->type = frm_type;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_FRM_DECI:
	{
		uint32_t deci_factor = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
			rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
		} else {
			path->frame_deci = deci_factor;
			path->valid_param.frame_deci = 1;
		}
		break;
	}

	default:
		break;
	}

	return -rtn;
}

int32_t dcam_path2_cfg(enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path2;

	switch (id) {

	case DCAM_PATH_INPUT_SIZE:
	{
		struct dcam_size *size = (struct dcam_size*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH2_INPUT_SIZE {%d %d} \n", size->w, size->h);
		if (size->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		size->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_SRC_SIZE_ERR;
		} else {
			path->input_size.w = size->w;
			path->input_size.h = size->h;
			path->valid_param.input_size = 1;

		}
		break;
	}

	case DCAM_PATH_INPUT_RECT:
	{
		struct dcam_rect *rect = (struct dcam_rect*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH2_INPUT_RECT {%d %d %d %d} \n",
			rect->x,
			rect->y,
			rect->w,
			rect->h);

		if (rect->x > DCAM_PATH_FRAME_WIDTH_MAX ||
		rect->y > DCAM_PATH_FRAME_HEIGHT_MAX ||
		rect->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		rect->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			memcpy((void*)&path->input_rect,
				(void*)rect,
				sizeof(struct dcam_rect));
#if 0
			if (path->input_rect.h >= DCAM_HEIGHT_MIN) {
				path->input_rect.h--;
			}
#endif
			path->valid_param.input_rect = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_SIZE:
	{
		struct dcam_size *size = (struct dcam_size*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH2_OUTPUT_SIZE {%d %d} \n", size->w, size->h);
		if (size->w > DCAM_PATH_FRAME_WIDTH_MAX ||
		size->h > DCAM_PATH_FRAME_HEIGHT_MAX) {
			rtn = DCAM_RTN_PATH_SRC_SIZE_ERR;
		} else {
			path->output_size.w = size->w;
			path->output_size.h = size->h;
			path->valid_param.output_size = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_FORMAT:
	{
		enum dcam_fmt format = *(enum dcam_fmt*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if((DCAM_YUV422 != format) && 
		(DCAM_YUV420 != format) && 
		(DCAM_YUV420_3FRAME != format)){
			rtn = DCAM_RTN_PATH_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}else{
			path->output_format = format;
			path->valid_param.output_format = 1;
		}
		break;
	}

	case DCAM_PATH_OUTPUT_ADDR:
	{
		struct dcam_addr *p_addr = (struct dcam_addr*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (DCAM_YUV_ADDR_INVALIDE(p_addr->yaddr, p_addr->uaddr, p_addr->vaddr)) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			if (path->output_frame_count > DCAM_PATH_2_FRM_CNT_MAX - 1) {
				DCAM_TRACE("DCAM DRV: Path 2 Error, DCAM_PATH_OUTPUT_ADDR, i=%d, y=0x%x, u=0x%x, v=0x%x \n",
					path->output_frame_count, p_addr->yaddr, p_addr->uaddr, p_addr->vaddr);
				rtn = DCAM_RTN_PATH_FRAME_TOO_MANY;
			} else {
				path->output_frame_cur->yaddr = p_addr->yaddr;
				path->output_frame_cur->uaddr = p_addr->uaddr;
				path->output_frame_cur->vaddr = p_addr->vaddr;
				path->output_frame_cur = path->output_frame_cur->next;
				DCAM_TRACE("DCAM DRV: Path 2 DCAM_PATH_OUTPUT_ADDR, i=%d, lock=%d, y=0x%x, u=0x%x, v=0x%x \n",
					path->output_frame_count, s_path2_frame[path->output_frame_count].lock,p_addr->yaddr, p_addr->uaddr, p_addr->vaddr);

				path->output_frame_count ++;
			}
		}
		break;
	}
	case DCAM_PATH_SRC_SEL:
	{
		uint32_t       src_sel = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (src_sel >= DCAM_PATH_FROM_NONE) {
			rtn = DCAM_RTN_PATH_SRC_ERR;
		} else {
			path->src_sel = src_sel;
			path->valid_param.src_sel = 1;

		}
		break;
	}

	case DCAM_PATH_FRAME_BASE_ID:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          base_id = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_BASE_ID 0x%x \n", base_id);
		for (i = 0; i < DCAM_PATH_2_FRM_CNT_MAX; i++) {
			frame->fid = base_id + i;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_DATA_ENDIAN:
	{
		struct dcam_endian_sel *endian = (struct dcam_endian_sel*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (endian->y_endian >= DCAM_ENDIAN_MAX ||
			endian->uv_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian  = endian->y_endian;
			path->data_endian.uv_endian = endian->uv_endian;
			path->valid_param.data_endian = 1;
		}
		break;
	}

	case DCAM_PATH_ENABLE:
	{
		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		path->valide = *(uint32_t*)param;

		break;
	}

	case DCAM_PATH_FRAME_TYPE:
	{
		struct dcam_frame *frame  = path->output_frame_head;
		uint32_t          frm_type = *(uint32_t*)param, i;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		DCAM_TRACE("DCAM DRV: DCAM_PATH_FRAME_TYPE 0x%x \n", frm_type);
		for (i = 0; i < DCAM_PATH_2_FRM_CNT_MAX; i++) {
			frame->type = frm_type;
			frame = frame->next;
		}
		break;
	}

	case DCAM_PATH_FRM_DECI:
	{
		uint32_t deci_factor = *(uint32_t*)param;

		DCAM_CHECK_PARAM_ZERO_POINTER(param);

		if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
			rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
		} else {
			path->frame_deci = deci_factor;
			path->valid_param.frame_deci = 1;
		}
		break;
	}

	default:
		break;
	}

	return -rtn;
}

int32_t    dcam_get_resizer(uint32_t wait_opt)
{
	int32_t                 rtn = 0;

	if( 0 == wait_opt) {
		rtn = mutex_trylock(&dcam_sem) ? 0 : 1;
		return rtn;
	} else if (DCAM_WAIT_FOREVER == wait_opt){
		mutex_lock(&dcam_sem);
		return rtn;
	} else {
		return 0;//mutex_timeout(&dcam_sem, wait_opt);
	}
}

int32_t    dcam_rel_resizer(void)
{
	 mutex_unlock(&dcam_sem);
	 return 0;
}

int32_t    dcam_resize_start(void)
{
	atomic_inc(&s_resize_flag);
	return 0;
}

int32_t    dcam_resize_end(void)
{
	atomic_dec(&s_resize_flag);
	if (s_resize_wait) {
		up(&s_done_sema);
		s_resize_wait = 0;
	}
	return 0;
}

void dcam_int_en(void)
{
	if (atomic_read(&s_dcam_users) == 1) {
		enable_irq(DCAM_IRQ);
	}
}

void dcam_int_dis(void)
{
	if (atomic_read(&s_dcam_users) == 1) {
		disable_irq(DCAM_IRQ);
	}
}

int32_t dcam_frame_is_locked(struct dcam_frame *frame)
{
	uint32_t                rtn = 0;
	unsigned long           flags;

	/*To disable irq*/
	local_irq_save(flags);
	if (frame)
		rtn = frame->lock == DCAM_FRM_LOCK_WRITE ? 1 : 0;
	local_irq_restore(flags);
	/*To enable irq*/

	return -rtn;
}

int32_t dcam_frame_lock(struct dcam_frame *frame)
{
	uint32_t                rtn = 0;
	unsigned long           flags;

	DCAM_TRACE_LOW("DCAM DRV: dcam_frame_lock 0x%x \n", (uint32_t)frame);

	/*To disable irq*/
	local_irq_save(flags);
	if (likely(frame))
		frame->lock = DCAM_FRM_LOCK_WRITE;
	else
		rtn = DCAM_RTN_PARA_ERR;
	local_irq_restore(flags);
	/*To enable irq*/

	return -rtn;
}

int32_t dcam_frame_unlock(struct dcam_frame *frame)
{
	uint32_t                rtn = 0;
	unsigned long           flags;

	DCAM_TRACE_LOW("DCAM DRV: dcam_frame_unlock 0x%x \n", (uint32_t)frame);

	/*To disable irq*/
	local_irq_save(flags);
	if (likely(frame))
		frame->lock = DCAM_FRM_UNLOCK;
	else
		rtn = DCAM_RTN_PARA_ERR;
	local_irq_restore(flags);
	/*To enable irq*/

	return -rtn;
}

int32_t    dcam_read_registers(uint32_t* reg_buf, uint32_t *buf_len)
{
	uint32_t               *reg_addr = (uint32_t*)DCAM_BASE;

	if (NULL == reg_buf || NULL == buf_len || 0 != (*buf_len % 4)) {
		return -1;
	}

	while (buf_len != 0 && (uint32_t)reg_addr < DCAM_END) {
		*reg_buf++ = REG_RD(reg_addr);
		reg_addr++;
		*buf_len -= 4;
	}

	*buf_len = (uint32_t)reg_addr - DCAM_BASE;
	return 0;
}

LOCAL irqreturn_t dcam_isr_root(int irq, void *dev_id)
{
	uint32_t                status, irq_line, err_flag = 0;
	unsigned long            flag;
	void                    *data;
	int32_t                 i;

	status = REG_RD(DCAM_INT_STS);
	if (unlikely(0 == status)) {
		return IRQ_NONE;
	}
	irq_line = status;
	if (unlikely(DCAM_IRQ_ERR_MASK & status)) {
		err_flag = 1;
		printk("DCAM DRV: error happened, 0x%x, %d \n", status, s_dcam_mod.dcam_path2.valide);
		_dcam_reg_trace();
		_dcam_stopped();
		if (s_dcam_mod.dcam_path2.valide) {
			/* both dcam paths have been working, it's safe to reset the whole dcam module*/
			dcam_reset(DCAM_RST_ALL);
		} else {
			dcam_reset(DCAM_RST_PATH1);
		}
	}

	if(status & ((1 << PATH2_DONE) | (1 << PATH2_OV))) {
		if ((0 == s_dcam_mod.dcam_path2.valide) && (DCAM_ST_STOP == s_dcam_mod.dcam_path2.status )) {
			printk("DCAM DRV: ignore path2, int=0x%x, valid=%d, status=%d \n",
				status, s_dcam_mod.dcam_path2.valide, s_dcam_mod.dcam_path2.status);
			status &= ~((1 << PATH2_DONE) | (1 << PATH2_OV));
			irq_line = status;
		}
	}

	spin_lock_irqsave(&dcam_lock,flag);

	if (err_flag && s_dcam_mod.user_func[DCAM_TX_ERR]) {
		data = s_dcam_mod.user_data[DCAM_TX_ERR];
		s_dcam_mod.user_func[DCAM_TX_ERR](NULL, data);
	} else if ((DCAM_IRQ_JPEG_OV_MASK & status) && s_dcam_mod.user_func[DCAM_NO_MEM]) {
		data = s_dcam_mod.user_data[DCAM_NO_MEM];
		s_dcam_mod.user_func[DCAM_NO_MEM](NULL, data);
	} else {
		for (i = IRQ_NUMBER - 1; i >= 0; i--) {
			if (irq_line & (1 << (uint32_t)i)) {
				isr_list[i]();
			}
			irq_line &= ~(uint32_t)(1 << (uint32_t)i); //clear the interrupt flag
			if(!irq_line) //no interrupt source left
				break;
		}
	}

	REG_WR(DCAM_INT_CLR, status);

	spin_unlock_irqrestore(&dcam_lock, flag);

	return IRQ_HANDLED;
}

LOCAL void _dcam_path0_set(void)
{
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path0;
	uint32_t         reg_val = 0;

	if(path->valid_param.input_size){
		reg_val = path->input_size.w | (path->input_size.h << 16);
		REG_WR(DCAM_PATH0_SRC_SIZE, reg_val);
	}
	
	if(path->valid_param.output_format){
		enum dcam_output_mode format = path->output_format;
		REG_MWR(DCAM_PATH0_CFG, BIT_2, format << 2);
		DCAM_TRACE("DCAM DRV: path 0: output_format=0x%x \n", format);
	}

	if(path->valid_param.frame_deci){
		REG_MWR(DCAM_PATH0_CFG, BIT_1 | BIT_0, path->frame_deci << 0);
	}

	REG_MWR(DCAM_ENDIAN_SEL, BIT_18, BIT_18); // axi write
	REG_MWR(DCAM_ENDIAN_SEL, BIT_19, BIT_19); // axi read


}
LOCAL void _dcam_path1_set(void)
{
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path1;
	uint32_t         reg_val = 0;
	
	if(path->valid_param.input_size){
		reg_val = path->input_size.w | (path->input_size.h << 16);
		REG_WR(DCAM_PATH1_SRC_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path1 set: src {%d %d} \n",path->input_size.w, path->input_size.h);
	}

	if(path->valid_param.input_rect){
		reg_val = path->input_rect.x | (path->input_rect.y << 16);
		REG_WR(DCAM_PATH1_TRIM_START, reg_val);
		reg_val = path->input_rect.w | (path->input_rect.h << 16);
		REG_WR(DCAM_PATH1_TRIM_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path1 set: rect {%d %d %d %d} \n",
			path->input_rect.x,
			path->input_rect.y,
			path->input_rect.w,
			path->input_rect.h);
	}

	if(path->valid_param.output_size){
		reg_val = path->output_size.w | (path->output_size.h << 16);
		REG_WR(DCAM_PATH1_DST_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path1 set: dst {%d %d} \n",path->output_size.w, path->output_size.h);
	}

	if(path->valid_param.output_format){
		enum dcam_fmt format = path->output_format;
		if (DCAM_YUV422 == format) {
			REG_MWR(DCAM_PATH1_CFG, BIT_7 | BIT_6, 0 << 6);
		} else if (DCAM_YUV420 == format) {
			REG_MWR(DCAM_PATH1_CFG, BIT_7 | BIT_6, 1 << 6);
		} else if (DCAM_YUV420_3FRAME == format) {
			REG_MWR(DCAM_PATH1_CFG, BIT_7 | BIT_6, 3 << 6);
		} else {
			DCAM_TRACE("DCAM DRV: invalid path 1 output format %d \n", format);
		}
		DCAM_TRACE("DCAM DRV: path 1: output_format=0x%x \n", format);
	}

	if(path->valid_param.src_sel){
		REG_MWR(DCAM_CFG, BIT_12 | BIT_11, path->src_sel << 11);
		DCAM_TRACE("DCAM DRV: path 1: src_sel=0x%x \n", path->src_sel);
	}

	if(path->valid_param.data_endian){
		REG_MWR(DCAM_ENDIAN_SEL, BIT_7 | BIT_6, path->data_endian.y_endian << 6);
		REG_MWR(DCAM_ENDIAN_SEL, BIT_9 | BIT_8, path->data_endian.uv_endian << 8);
		REG_MWR(DCAM_ENDIAN_SEL, BIT_18, BIT_18); // axi write
		REG_MWR(DCAM_ENDIAN_SEL, BIT_19, BIT_19); // axi read
		DCAM_TRACE("DCAM DRV: path 1: data_endian y=0x%x, uv=0x%x \n",
			path->data_endian.y_endian, path->data_endian.uv_endian);
	}

	if(path->valid_param.frame_deci){
		REG_MWR(DCAM_PATH0_CFG, BIT_24 | BIT_23, path->frame_deci << 23);
		DCAM_TRACE("DCAM DRV: path 1: frame_deci=0x%x \n", path->frame_deci);
	}

	if(path->valid_param.scale_tap){
		path->valid_param.scale_tap = 0;
		REG_MWR(DCAM_PATH1_CFG, BIT_19 | BIT_18 | BIT_17 | BIT_16, (path->scale_tap.y_tap & 0x0F) << 16);
		REG_MWR(DCAM_PATH1_CFG, BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11, (path->scale_tap.uv_tap & 0x1F) << 11);
		DCAM_TRACE("DCAM DRV: path 1: scale_tap, y=0x%x, uv=0x%x \n", path->scale_tap.y_tap, path->scale_tap.uv_tap);
	}

	if(path->valid_param.v_deci){
		path->valid_param.v_deci = 0;
		REG_MWR(DCAM_PATH1_CFG, BIT_2, path->deci_val.deci_x_en << 2);
		REG_MWR(DCAM_PATH1_CFG, BIT_1 | BIT_0, path->deci_val.deci_x);

		REG_MWR(DCAM_PATH1_CFG, BIT_5, path->deci_val.deci_y_en << 5);
		REG_MWR(DCAM_PATH1_CFG, BIT_4 | BIT_3, path->deci_val.deci_y << 3);
		DCAM_TRACE("DCAM DRV: path 1: deci, x_en=%d, x=%d, y_en=%d, y=%d \n",
			path->deci_val.deci_x_en, path->deci_val.deci_x, path->deci_val.deci_y_en, path->deci_val.deci_y);
	}
}

LOCAL void _dcam_path2_set(void)
{
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path2;
	uint32_t         reg_val = 0;
	
	if(path->valid_param.input_size){
		reg_val = path->input_size.w | (path->input_size.h << 16);
		REG_WR(DCAM_PATH2_SRC_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path2 set: src {%d %d} \n",path->input_size.w, path->input_size.h);
	}

	if(path->valid_param.input_rect){
		reg_val = path->input_rect.x | (path->input_rect.y << 16);
		REG_WR(DCAM_PATH2_TRIM_START, reg_val);
		reg_val = path->input_rect.w | (path->input_rect.h << 16);
		REG_WR(DCAM_PATH2_TRIM_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path2 set: rect {%d %d %d %d} \n",
			path->input_rect.x,
			path->input_rect.y,
			path->input_rect.w,
			path->input_rect.h);

	}

	if(path->valid_param.output_size){
		reg_val = path->output_size.w | (path->output_size.h << 16);
		REG_WR(DCAM_PATH2_DST_SIZE, reg_val);
		DCAM_TRACE("DCAM DRV: path2 set: dst {%d %d} \n",path->output_size.w, path->output_size.h);
	}

	if(path->valid_param.output_format){
		enum dcam_fmt format = path->output_format;
		
		// REG_MWR(DCAM_PATH2_CFG, BIT_8, 0 << 8); // aiden todo
		
		if (DCAM_YUV422 == format) {
			REG_MWR(DCAM_PATH2_CFG, BIT_7 | BIT_6, 0 << 6);
		} else if (DCAM_YUV420 == format) {
			REG_MWR(DCAM_PATH2_CFG, BIT_7 | BIT_6, 1 << 6);
		} else if (DCAM_YUV420_3FRAME == format) {
			REG_MWR(DCAM_PATH2_CFG, BIT_7 | BIT_6, 3 << 6);
		} else {
			DCAM_TRACE("DCAM DRV: invalid path 2 output format %d \n", format);
		}
		DCAM_TRACE("DCAM DRV: path 2: output_format=0x%x \n", format);
	}

	if(path->valid_param.src_sel){
		REG_MWR(DCAM_CFG, BIT_14 | BIT_13, path->src_sel << 13);
	}

	if(path->valid_param.data_endian){
		REG_MWR(DCAM_ENDIAN_SEL, BIT_11 | BIT_10, path->data_endian.y_endian << 10);
		REG_MWR(DCAM_ENDIAN_SEL, BIT_13 | BIT_12, path->data_endian.uv_endian << 12);
		REG_MWR(DCAM_ENDIAN_SEL, BIT_18, BIT_18); // axi write
		REG_MWR(DCAM_ENDIAN_SEL, BIT_19, BIT_19); // axi read
		DCAM_TRACE("DCAM DRV: path 2: data_endian y=0x%x, uv=0x%x \n",
			path->data_endian.y_endian, path->data_endian.uv_endian);
	}

	if(path->valid_param.frame_deci){
		REG_MWR(DCAM_PATH0_CFG, BIT_24 | BIT_23, path->frame_deci << 23);
		DCAM_TRACE("DCAM DRV: path 2: frame_deci=0x%x \n", path->frame_deci);
	}

	if(path->valid_param.scale_tap){
		path->valid_param.scale_tap = 0;
		REG_MWR(DCAM_PATH2_CFG, BIT_19 | BIT_18 | BIT_17 | BIT_16, (path->scale_tap.y_tap & 0x0F) << 16);
		REG_MWR(DCAM_PATH2_CFG, BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11, (path->scale_tap.uv_tap & 0x1F) << 11);
		DCAM_TRACE_LOW("DCAM DRV: path 2: scale_tap, y=0x%x, uv=0x%x \n", path->scale_tap.y_tap, path->scale_tap.uv_tap);
	}

	if(path->valid_param.v_deci){
		path->valid_param.v_deci = 0;
		REG_MWR(DCAM_PATH2_CFG, BIT_2, path->deci_val.deci_x_en << 2);
		REG_MWR(DCAM_PATH2_CFG, BIT_1 | BIT_0, path->deci_val.deci_x);

		REG_MWR(DCAM_PATH2_CFG, BIT_5, path->deci_val.deci_y_en << 5);
		REG_MWR(DCAM_PATH2_CFG, BIT_4 | BIT_3, path->deci_val.deci_y << 3);
		DCAM_TRACE("DCAM DRV: path 2: deci, x_en=%d, x=%d, y_en=%d, y=%d \n",
			path->deci_val.deci_x_en, path->deci_val.deci_x, path->deci_val.deci_y_en, path->deci_val.deci_y);
	}
}

LOCAL void _dcam_frm_clear(enum dcam_path_index path_index)
{
	uint32_t                i = 0;
	struct dcam_frame       *path0_frame = &s_path0_frame[0];
	struct dcam_frame       *path1_frame = &s_path1_frame[0];
	struct dcam_frame       *path2_frame = &s_path2_frame[0];

	if(DCAM_PATH_IDX_0 & path_index){
		for (i = 0; i < DCAM_PATH_0_FRM_CNT_MAX; i++) {
			(path0_frame+i)->lock = DCAM_FRM_UNLOCK;
		}
		s_dcam_mod.dcam_path0.output_frame_head = path0_frame;
		s_dcam_mod.dcam_path0.output_frame_cur  = path0_frame;
		s_dcam_mod.dcam_path0.output_frame_count  = 0;
	}

	if(DCAM_PATH_IDX_1 & path_index){
		for (i = 0; i < DCAM_PATH_1_FRM_CNT_MAX; i++) {
			(path1_frame+i)->lock = DCAM_FRM_UNLOCK;
		}
		s_dcam_mod.dcam_path1.output_frame_head = path1_frame;
		s_dcam_mod.dcam_path1.output_frame_cur  = path1_frame;
		s_dcam_mod.dcam_path1.output_frame_count  = 0;
	}

	if(DCAM_PATH_IDX_2 & path_index) {
		for (i = 0; i < DCAM_PATH_2_FRM_CNT_MAX; i++) {
			(path2_frame+i)->lock = DCAM_FRM_UNLOCK;
		}
		s_dcam_mod.dcam_path2.output_frame_head = path2_frame;
		s_dcam_mod.dcam_path2.output_frame_cur  = path2_frame;
		s_dcam_mod.dcam_path2.output_frame_count  = 0;
	}

	return;
}

LOCAL void _dcam_link_frm(uint32_t base_id)
{
	uint32_t                i = 0;
	struct dcam_frame       *path0_frame = &s_path0_frame[0];
	struct dcam_frame       *path1_frame = &s_path1_frame[0];
	struct dcam_frame       *path2_frame = &s_path2_frame[0];

	for (i = 0; i < DCAM_PATH_0_FRM_CNT_MAX; i++) {
		DCAM_CLEAR(path0_frame + i);
		(path0_frame+i)->next = path0_frame + (i + 1) % DCAM_PATH_0_FRM_CNT_MAX;
		(path0_frame+i)->prev = path0_frame + (i - 1 + DCAM_PATH_0_FRM_CNT_MAX) % DCAM_PATH_0_FRM_CNT_MAX;
		(path0_frame+i)->fid  = base_id + i;
	}

	for (i = 0; i < DCAM_PATH_1_FRM_CNT_MAX; i++) {
		DCAM_CLEAR(path1_frame + i);
		(path1_frame+i)->next = path1_frame + (i + 1) % DCAM_PATH_1_FRM_CNT_MAX;
		(path1_frame+i)->prev = path1_frame + (i - 1 + DCAM_PATH_1_FRM_CNT_MAX) % DCAM_PATH_1_FRM_CNT_MAX;
		(path1_frame+i)->fid  = base_id + i;
	}

	for (i = 0; i < DCAM_PATH_2_FRM_CNT_MAX; i++) {
		DCAM_CLEAR(path2_frame + i);
		(path2_frame+i)->next = path2_frame+(i + 1) % DCAM_PATH_2_FRM_CNT_MAX;
		(path2_frame+i)->prev = path2_frame+(i - 1 + DCAM_PATH_2_FRM_CNT_MAX) % DCAM_PATH_2_FRM_CNT_MAX;
		(path2_frame+i)->fid  = base_id + i;
	}

	s_dcam_mod.dcam_path0.output_frame_head = path0_frame;
	s_dcam_mod.dcam_path1.output_frame_head = path1_frame;
	s_dcam_mod.dcam_path2.output_frame_head = path2_frame;
	s_dcam_mod.dcam_path0.output_frame_cur = path0_frame;
	s_dcam_mod.dcam_path1.output_frame_cur = path1_frame;
	s_dcam_mod.dcam_path2.output_frame_cur = path2_frame;

	return;
}

LOCAL int32_t _dcam_path_set_next_frm(enum dcam_path_index path_index, uint32_t is_1st_frm)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_frame       *frame = NULL;
	struct dcam_path_desc   *path = NULL;
	uint32_t                yuv_reg[3] = {0};
	uint32_t                path_max_frm_cnt;

	if (DCAM_PATH_IDX_0 == path_index) {
		frame = &s_path0_frame[0];
		path = &s_dcam_mod.dcam_path0;
		yuv_reg[0] = DCAM_FRM_ADDR0;
		path_max_frm_cnt = DCAM_PATH_0_FRM_CNT_MAX;
	} else if (DCAM_PATH_IDX_1 == path_index) {
		frame = &s_path1_frame[0];
		path = &s_dcam_mod.dcam_path1;
		yuv_reg[0] = DCAM_FRM_ADDR1;
		yuv_reg[1] = DCAM_FRM_ADDR2;
		yuv_reg[2] = DCAM_FRM_ADDR3;
		path_max_frm_cnt = DCAM_PATH_1_FRM_CNT_MAX;
	} else /*if(DCAM_PATH_IDX_2 == path_index)*/ {
		frame = &s_path2_frame[0];
		path = &s_dcam_mod.dcam_path2;
		yuv_reg[0] = DCAM_FRM_ADDR4;
		yuv_reg[1] = DCAM_FRM_ADDR5;
		yuv_reg[2] = DCAM_FRM_ADDR6;
		path_max_frm_cnt = DCAM_PATH_2_FRM_CNT_MAX;
	}

	if (is_1st_frm) {
		if (path->output_frame_count < path_max_frm_cnt) {
			frame = path->output_frame_cur->prev;
			frame->next = path->output_frame_head;
			path->output_frame_head->prev = frame;
			path->output_frame_cur = path->output_frame_head;
		}
	}

	if (0 == dcam_frame_is_locked(path->output_frame_cur)) {
		REG_WR(yuv_reg[0], path->output_frame_cur->yaddr);
		if ((DCAM_YUV400 > path->output_format) && (DCAM_PATH_IDX_0 != path_index)) {
			REG_WR(yuv_reg[1], path->output_frame_cur->uaddr);
			if (DCAM_YUV420_3FRAME == path->output_format) {
				REG_WR(yuv_reg[2], path->output_frame_cur->vaddr);
			}
		}
		path->output_frame_cur = path->output_frame_cur->next;
	} else {
		rtn = DCAM_RTN_PATH_FRAME_LOCKED;
	}

	return -rtn;
}

LOCAL int32_t _dcam_path_trim(enum dcam_path_index path_index)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path;
	uint32_t                cfg_reg, ctrl_bit;

	if (DCAM_PATH_IDX_1 == path_index) {
		path = &s_dcam_mod.dcam_path1;
		cfg_reg = DCAM_PATH1_CFG;
		ctrl_bit = 8;
	} else if (DCAM_PATH_IDX_2 == path_index) {
		path = &s_dcam_mod.dcam_path2;
		cfg_reg = DCAM_PATH2_CFG;
		ctrl_bit = 1;
	} else {
		printk("DCAM DRV: _dcam_path_trim invalid path_index=%d \n", path_index);
		return -1;
	}

	if (path->input_size.w != path->input_rect.w ||
		path->input_size.h != path->input_rect.h) {
//		REG_OWR(cfg_reg, 1 << ctrl_bit);
	} else {
		//REG_MWR(cfg_reg, 1 << ctrl_bit, 0 << ctrl_bit);
	}

	return rtn;

}
LOCAL int32_t _dcam_path_scaler(enum dcam_path_index path_index)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = NULL;
	uint32_t                cfg_reg = 0;

	if (DCAM_PATH_IDX_1 == path_index) {
		path = &s_dcam_mod.dcam_path1;
		cfg_reg = DCAM_PATH1_CFG;
	} else if (DCAM_PATH_IDX_2 == path_index){
		path = &s_dcam_mod.dcam_path2;
		cfg_reg = DCAM_PATH2_CFG;
	}

	if (DCAM_RAWRGB == path->output_format ||
	DCAM_JPEG == path->output_format) {
		DCAM_TRACE("DCAM DRV: _dcam_path_scaler out format is %d, no need scaler \n", path->output_format);
		return DCAM_RTN_SUCCESS;
	}

	rtn = _dcam_calc_sc_size(path_index);
	if (DCAM_RTN_SUCCESS != rtn) {
		return rtn;
	}

//	if (path->sc_input_size.w != path->output_size.w ||
//	path->sc_input_size.h != path->output_size.h) {
		REG_MWR(cfg_reg, BIT_20, 0 << 20);
		rtn = _dcam_set_sc_coeff(path_index);
//	} else {
//		REG_MWR(cfg_reg, BIT_20, 1 << 20);
//	}

	return rtn;
}

LOCAL int32_t _dcam_path_check_deci(enum dcam_path_index path_index, uint32_t *is_deci)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = NULL;
	uint32_t				w_deci = 0;
	uint32_t				h_deci = 0;

	if (DCAM_PATH_IDX_1 == path_index) {
		path = &s_dcam_mod.dcam_path1;
	} else if (DCAM_PATH_IDX_2 == path_index){
		path = &s_dcam_mod.dcam_path2;
	} else {
		rtn = DCAM_RTN_PATH_SC_ERR;
		goto dcam_path_err;
	}

	if (path->input_rect.w > (path->output_size.w * DCAM_SC_COEFF_DOWN_MAX * (1<<DCAM_PATH_DECI_FAC_MAX)) ||
	path->input_rect.h > (path->output_size.h * DCAM_SC_COEFF_DOWN_MAX * (1<<DCAM_PATH_DECI_FAC_MAX)) ||
	path->input_rect.w * DCAM_SC_COEFF_UP_MAX < path->output_size.w ||
	path->input_rect.h * DCAM_SC_COEFF_UP_MAX < path->output_size.h) {
		rtn = DCAM_RTN_PATH_SC_ERR;
	} else {
		if (path->input_rect.w > path->output_size.w * DCAM_SC_COEFF_DOWN_MAX)
			w_deci = 1;
		if (path->input_rect.h > path->output_size.h * DCAM_SC_COEFF_DOWN_MAX)
			h_deci = 1;

		if(w_deci || h_deci)
			*is_deci = 1;
		else
			*is_deci = 0;
	}
dcam_path_err:
	DCAM_TRACE("DCAM DRV: _dcam_path_check_deci: path_index=%d, is_deci=%d, rtn=%d \n", path_index, *is_deci, rtn);

	return rtn;
}

LOCAL uint32_t _dcam_get_path_deci_factor(uint32_t src_size, uint32_t dst_size)
{
	uint32_t                 factor = 0;

	if (0 == src_size || 0 == dst_size) {
		return factor;
	}

	/* factor: 0 - 1/2, 1 - 1/4, 2 - 1/8, 3 - 1/16 */
	for (factor = 0; factor < DCAM_PATH_DECI_FAC_MAX; factor ++) {
		if (src_size < (uint32_t)(dst_size * (1 << (factor + 1)))) {
			break;
		}
	}

	return factor;
}

LOCAL int32_t _dcam_calc_sc_size(enum dcam_path_index path_index)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc   *path = NULL;
	uint32_t                cfg_reg = 0;
	uint32_t                tmp_dstsize = 0;
	uint32_t                align_size = 0;

	if (DCAM_PATH_IDX_1 == path_index) {
		path = &s_dcam_mod.dcam_path1;
		cfg_reg = DCAM_PATH1_CFG;
	} else if (DCAM_PATH_IDX_2 == path_index){
		path = &s_dcam_mod.dcam_path2;
		cfg_reg = DCAM_PATH2_CFG;
	}

	if (path->input_rect.w > (path->output_size.w * DCAM_SC_COEFF_DOWN_MAX * (1<<DCAM_PATH_DECI_FAC_MAX)) ||
	path->input_rect.h > (path->output_size.h * DCAM_SC_COEFF_DOWN_MAX * (1<<DCAM_PATH_DECI_FAC_MAX)) ||
	path->input_rect.w * DCAM_SC_COEFF_UP_MAX < path->output_size.w ||
	path->input_rect.h * DCAM_SC_COEFF_UP_MAX < path->output_size.h) {
		rtn = DCAM_RTN_PATH_SC_ERR;
	} else {
		path->sc_input_size.w = path->input_rect.w;
		path->sc_input_size.h = path->input_rect.h;
		if (path->input_rect.w > path->output_size.w * DCAM_SC_COEFF_DOWN_MAX) {
		//if(0) {
			//REG_MWR(cfg_reg, BIT_1 | BIT_0, 0);
			//REG_MWR(cfg_reg, BIT_2, BIT_2);
			tmp_dstsize = path->output_size.w * DCAM_SC_COEFF_DOWN_MAX;
			path->deci_val.deci_x = _dcam_get_path_deci_factor(path->input_rect.w, tmp_dstsize);
			path->deci_val.deci_x_en = 1;
			path->valid_param.v_deci = 1;
			align_size = (1 << (path->deci_val.deci_x+1))*DCAM_PIXEL_ALIGN_WIDTH;
			path->input_rect.w = (path->input_rect.w) & ~(align_size-1);
			path->input_rect.x = (path->input_rect.x) & ~(align_size-1);
			path->sc_input_size.w = path->input_rect.w >> (path->deci_val.deci_x+1);
		} else {
			path->deci_val.deci_x = 0;
			path->deci_val.deci_x_en = 0;
			path->valid_param.v_deci = 1;
		}

		if (path->input_rect.h > path->output_size.h * DCAM_SC_COEFF_DOWN_MAX) {
		//if (0) {
			//REG_MWR(cfg_reg, BIT_3 | BIT_4, 0);
			//REG_MWR(cfg_reg, BIT_5, BIT_5);
			tmp_dstsize = path->output_size.h * DCAM_SC_COEFF_DOWN_MAX;
			path->deci_val.deci_y = _dcam_get_path_deci_factor(path->input_rect.h, tmp_dstsize);
			path->deci_val.deci_y_en = 1;
			path->valid_param.v_deci = 1;
			align_size = (1 << (path->deci_val.deci_y+1))*DCAM_PIXEL_ALIGN_HEIGHT;
			path->input_rect.h = (path->input_rect.h) & ~(align_size-1);
			path->input_rect.y = (path->input_rect.y) & ~(align_size-1);
			path->sc_input_size.h = path->input_rect.h >> (path->deci_val.deci_y+1);
		} else {
			path->deci_val.deci_y = 0;
			path->deci_val.deci_y_en = 0;
			path->valid_param.v_deci = 1;
		}

	}

	DCAM_TRACE("DCAM: _dcam_calc_sc_size, path=%d, x_en=%d, deci_x=%d, y_en=%d, deci_y=%d \n",
		path_index, path->deci_val.deci_x_en, path->deci_val.deci_x, path->deci_val.deci_y_en, path->deci_val.deci_y);

	return -rtn;
}

LOCAL int32_t _dcam_set_sc_coeff(enum dcam_path_index path_index)
{
	struct dcam_path_desc   *path = NULL;
	uint32_t                i = 0;
	uint32_t                h_coeff_addr = DCAM_BASE;
	uint32_t                v_coeff_addr  = DCAM_BASE;
	uint32_t                v_chroma_coeff_addr  = DCAM_BASE;
	uint32_t                *tmp_buf = NULL;
	uint32_t                *h_coeff = NULL;
	uint32_t                *v_coeff = NULL;
	uint32_t                *v_chroma_coeff = NULL;
	uint32_t                clk_switch_bit = 0;
	uint32_t                clk_switch_shift_bit = 0;
	uint32_t                clk_status_bit = 0;
	uint32_t                ver_tap_reg = 0;
	uint32_t                scale2yuv420 = 0;
	uint8_t                 y_tap = 0;
	uint8_t                 uv_tap = 0;

	if (DCAM_PATH_IDX_1 != path_index && DCAM_PATH_IDX_2 != path_index)
		return -DCAM_RTN_PARA_ERR;

	if (DCAM_PATH_IDX_1 == path_index) {
		path = &s_dcam_mod.dcam_path1;
		h_coeff_addr += DCAM_SC1_H_TAB_OFFSET;
		v_coeff_addr += DCAM_SC1_V_TAB_OFFSET;
		v_chroma_coeff_addr += DCAM_SC1_V_CHROMA_TAB_OFFSET;
		clk_switch_bit = BIT_3;
		clk_switch_shift_bit = 3;
		clk_status_bit = BIT_5;
		ver_tap_reg = DCAM_PATH1_CFG;
	} else if (DCAM_PATH_IDX_2 == path_index){
		path = &s_dcam_mod.dcam_path2;
		h_coeff_addr += DCAM_SC2_H_TAB_OFFSET;
		v_coeff_addr += DCAM_SC2_V_TAB_OFFSET;
		v_chroma_coeff_addr += DCAM_SC2_V_CHROMA_TAB_OFFSET;
		clk_switch_bit = BIT_4;
		clk_switch_shift_bit = 4;
		clk_status_bit = BIT_6;
		ver_tap_reg = DCAM_PATH2_CFG;
	}

	if(DCAM_YUV420 == path->output_format){
	    scale2yuv420 = 1;
	}

	DCAM_TRACE("DCAM DRV: _dcam_set_sc_coeff {%d %d %d %d}, 420=%d \n",
		path->sc_input_size.w,
		path->sc_input_size.h,
		path->output_size.w,
		path->output_size.h, scale2yuv420);


	tmp_buf = (uint32_t *)kmalloc(DCAM_SC_COEFF_BUF_SIZE, GFP_KERNEL);

	if (NULL == tmp_buf) {
		return -DCAM_RTN_PATH_NO_MEM;
	}

	h_coeff = tmp_buf;
	v_coeff = tmp_buf + (DCAM_SC_COEFF_COEF_SIZE/4);
	v_chroma_coeff = v_coeff + (DCAM_SC_COEFF_COEF_SIZE/4);

	if (!(Dcam_GenScaleCoeff((int16_t)path->sc_input_size.w,
		(int16_t)path->sc_input_size.h,
		(int16_t)path->output_size.w,
		(int16_t)path->output_size.h,
		h_coeff,
		v_coeff,
		v_chroma_coeff,
		scale2yuv420,
		&y_tap,
		&uv_tap,
		tmp_buf + (DCAM_SC_COEFF_COEF_SIZE*3/4),
		DCAM_SC_COEFF_TMP_SIZE))) {
		kfree(tmp_buf);
		DCAM_TRACE("DCAM DRV: _dcam_set_sc_coeff Dcam_GenScaleCoeff error! \n");
		return -DCAM_RTN_PATH_GEN_COEFF_ERR;
	}
#if 0//ndef __SIMULATOR__
	do {
		REG_MWR(DCAM_CFG, clk_switch_bit, DCAM_CLK_DOMAIN_AHB << clk_switch_shift_bit);
	} while (clk_status_bit != (clk_status_bit & REG_RD(DCAM_CFG)));
#endif

	for (i = 0; i < DCAM_SC_COEFF_H_NUM; i++) {
		REG_WR(h_coeff_addr, *h_coeff);
		h_coeff_addr += 4;
		h_coeff++;
	}

	for (i = 0; i < DCAM_SC_COEFF_V_NUM; i++) {
		REG_WR(v_coeff_addr, *v_coeff);
		v_coeff_addr += 4;
		v_coeff++;
	}

	for (i = 0; i < DCAM_SC_COEFF_V_CHROMA_NUM; i++) {
		REG_WR(v_chroma_coeff_addr, *v_chroma_coeff);
		v_chroma_coeff_addr += 4;
		v_chroma_coeff++;
	}

	path->scale_tap.y_tap = y_tap;
	path->scale_tap.uv_tap = uv_tap;
	path->valid_param.scale_tap = 1;

	//REG_MWR(ver_tap_reg, BIT_19 | BIT_18 | BIT_17 | BIT_16, (y_tap & 0x0F) << 16);
	//REG_MWR(ver_tap_reg, BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11, (uv_tap & 0x1F) << 11);
	DCAM_TRACE("DCAM DRV: _dcam_set_sc_coeff y_tap=0x%x, uv_tap=0x%x \n", y_tap, uv_tap);
#if 0 //ndef __SIMULATOR__
	do {
		REG_MWR(DCAM_CFG, clk_switch_bit, 0 << clk_switch_shift_bit);
	} while (0 != (clk_status_bit & REG_RD(DCAM_CFG)));
#endif
	kfree(tmp_buf);

	return DCAM_RTN_SUCCESS;
}

LOCAL void _dcam_force_copy_ext(enum dcam_path_index path_index, uint32_t path_copy, uint32_t coef_copy)
{
	uint32_t         reg_val = 0;

	if(DCAM_PATH_IDX_1 == path_index){
		if(path_copy)
			reg_val |= BIT_10;
		if(coef_copy)
			reg_val |= BIT_14;

		REG_MWR(DCAM_CONTROL, reg_val, reg_val);
	}else if(DCAM_PATH_IDX_2 == path_index){
		if(path_copy)
			reg_val |= BIT_12;
		if(coef_copy)
			reg_val |= BIT_16;

		REG_MWR(DCAM_CONTROL, reg_val, reg_val);
	}else{
		DCAM_TRACE("DCAM DRV: _dcam_force_copy_ext invalid path index: %d \n", path_index);
	}
}

LOCAL void _dcam_auto_copy_ext(enum dcam_path_index path_index, uint32_t path_copy, uint32_t coef_copy)
{
	uint32_t         reg_val = 0;

	if(DCAM_PATH_IDX_1 == path_index){
		if(path_copy)
			reg_val |= BIT_11;
		if(coef_copy)
			reg_val |= BIT_15;

		REG_MWR(DCAM_CONTROL, reg_val, reg_val);
	}else if(DCAM_PATH_IDX_2 == path_index){
		if(path_copy)
			reg_val |= BIT_13;
		if(coef_copy)
			reg_val |= BIT_17;

		REG_MWR(DCAM_CONTROL, reg_val, reg_val);
	}else{
		DCAM_TRACE("DCAM DRV: _dcam_auto_copy_ext invalid path index: %d \n", path_index);
	}

	//DCAM_TRACE("DCAM DRV: _dcam_auto_copy_ext index: 0x%x, path_copy=%d, coef_copy=%d, reg_val=0x%x \n", 
	//	path_index, path_copy, coef_copy, reg_val);
}

LOCAL void _dcam_force_copy(enum dcam_path_index path_index)
{
	if(DCAM_PATH_IDX_1 == path_index){
		REG_MWR(DCAM_CONTROL, BIT_10, 1 << 10);
	}else if(DCAM_PATH_IDX_2 == path_index){
		REG_MWR(DCAM_CONTROL, BIT_12, 1 << 12);
	}else{
		DCAM_TRACE("DCAM DRV: _dcam_force_copy invalid path index: %d \n", path_index);
	}
}

LOCAL void _dcam_auto_copy(enum dcam_path_index path_index)
{
	if(DCAM_PATH_IDX_0 == path_index){
		REG_MWR(DCAM_CONTROL, BIT_9, 1 << 9);
	}else if(DCAM_PATH_IDX_1 == path_index){
		REG_MWR(DCAM_CONTROL, BIT_11, 1 << 11);
	}else if(DCAM_PATH_IDX_2 == path_index){
		REG_MWR(DCAM_CONTROL, BIT_13, 1 << 13);
	}else{
		DCAM_TRACE("DCAM DRV: _dcam_auto_copy invalid path index: %d \n", path_index);
	}
}

LOCAL void _dcam_reg_trace(void)
{
#ifdef DCAM_DRV_DEBUG
	uint32_t                addr = 0;

	printk("DCAM DRV: Register list");
	for (addr = DCAM_CFG; addr <= CAP_SENSOR_CTRL; addr += 16) {
		printk("\n 0x%x: 0x%x 0x%x 0x%x 0x%x",
			addr,
			REG_RD(addr),
			REG_RD(addr + 4),
			REG_RD(addr + 8),
			REG_RD(addr + 12));
	}

	printk("\n");
#endif	
}

LOCAL void    _sensor_sof(void)
{
	//DCAM_TRACE("DCAM DRV: _sensor_sof \n");

	return;
}

LOCAL void    _sensor_eof(void)
{
	//DCAM_TRACE("DCAM DRV: _sensor_eof \n");
	_dcam_stopped();
	return;
}

LOCAL void    _cap_sof(void)
{
	//DCAM_TRACE("DCAM DRV: _cap_sof, s_path_wait=%d, s_sof_cnt=%d \n", s_path_wait, s_sof_cnt);

	if (s_sof_wait) {
		s_sof_cnt++;
		DCAM_TRACE("DCAM: _cap_sof: %d \n", s_sof_cnt);
		if (s_sof_cnt >= 1) {
			s_sof_wait = 0;
			up(&s_path_stop_sema);
			DCAM_TRACE("DCAM: _cap_sof %d, send s_path_stop_sema\n", s_sof_cnt);	
		}
	}

	return;
}

LOCAL void    _cap_eof(void)
{
	//DCAM_TRACE("DCAM DRV: _cap_eof \n");
	_dcam_stopped();
	return;
}

LOCAL void    _path0_done(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func           user_func = s_dcam_mod.user_func[DCAM_TX_DONE];
	void                    *data = s_dcam_mod.user_data[DCAM_TX_DONE];
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path0;
	struct dcam_frame       *frame = path->output_frame_cur->prev->prev;

	if (0 == s_dcam_mod.dcam_path0.valide) {
		DCAM_TRACE("DCAM DRV: path0 is not valid \n");
		return;
	}

	DCAM_TRACE("DCAM 0\n");

	rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_0, false);
	if (rtn) {
		DCAM_TRACE("DCAM DRV: path 0 wait for frame unlocked \n");
		return;
	}
	_dcam_auto_copy(DCAM_PATH_IDX_0);

	frame->width = path->output_size.w;
	frame->height = path->output_size.h;
	

	if(user_func)
	{
		(*user_func)(frame, data);
	}
	return;
}

LOCAL void    _path0_overflow(void)
{
	printk("DCAM DRV: _path0_overflow \n");

	return;
}

LOCAL void    _path1_done(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func           user_func = s_dcam_mod.user_func[DCAM_TX_DONE];
	void                    *data = s_dcam_mod.user_data[DCAM_TX_DONE];
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path1;
	struct dcam_frame       *frame = path->output_frame_cur->prev->prev;
	uint32_t                coef_copy = 0;

	if (s_path1_done_wait_dis_en) {
		REG_AWR(DCAM_CFG, ~BIT_1);
		s_path1_done_wait_dis_en = 0;
		up(&s_path1_done_dis_en_sema);
	}

	if (s_path1_done_wait) {
		s_path1_done_cnt--;
		DCAM_TRACE("DCAM: _path1_done: %d \n", s_path1_done_cnt);
		if (0 == s_path1_done_cnt) {
			s_path1_done_wait = 0;
			up(&s_path1_done_sema);
			DCAM_TRACE("DCAM: _path1_done %d, send s_path1_done_sema\n", s_path1_done_cnt);	
		}
	}

	if (0 == s_dcam_mod.dcam_path1.valide) {
		DCAM_TRACE("DCAM DRV: path1 is not valid \n");
		return;
	}

	//printk("DCAM 1\n");

	DCAM_TRACE_LOW("DCAM 1: frame 0x%x, y uv, 0x%x 0x%x \n",
		(int)frame, frame->yaddr, frame->uaddr);

	if (s_dcam_mod.dcam_path1.path_update) {
		s_dcam_mod.dcam_path1.path_done_cnt++;
		DCAM_TRACE("DCAM: path 1 update, %d \n", s_dcam_mod.dcam_path1.path_done_cnt);
		if (1 == s_dcam_mod.dcam_path1.path_done_cnt) {
			coef_copy = 1;
			_dcam_path1_set();
		} else{
			coef_copy = 0;

			if(s_dcam_mod.dcam_path1.path_done_cnt >= 2) {
				s_dcam_mod.dcam_path1.path_update = 0;
				if(s_dcam_mod.dcam_path1.path_update_wait){
					s_dcam_mod.dcam_path1.path_update_wait = 0;
					up(&s_path1_update_sema);
					DCAM_TRACE("DCAM: path 1 done, send s_path1_update_sema \n");
				}
				DCAM_TRACE("path 1, update path done \n");
			}
		}
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, false);
		if (rtn) {
			DCAM_TRACE("DCAM DRV: path 1 update wait for frame unlocked\n");
			return;
		}
		_dcam_auto_copy_ext(DCAM_PATH_IDX_1, true, coef_copy);

	} else {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_1, false);
		if (rtn) {
			DCAM_TRACE("DCAM DRV: path 1 wait for frame unlocked\n");
			return;
		}
		_dcam_auto_copy(DCAM_PATH_IDX_1);
	}

	frame->width = path->output_size.w;
	frame->height = path->output_size.h;

	dcam_frame_lock(frame);

	if(user_func)
	{
		(*user_func)(frame, data);
	}
	return;
}

LOCAL void    _path1_overflow(void)
{
	printk("DCAM DRV: _path1_overflow \n");

	return;
}

LOCAL void    _sensor_line_err(void)
{
	printk("DCAM DRV: _sensor_line_err \n");

	return;
}

LOCAL void    _sensor_frame_err(void)
{
	printk("DCAM DRV: _sensor_eof \n");

	return;
}

LOCAL void    _jpeg_buf_ov(void)
{
	printk("DCAM DRV: _jpeg_buf_ov \n");

	return;
}

LOCAL void    _path2_done(void)
{
	enum dcam_drv_rtn       rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func           user_func = s_dcam_mod.user_func[DCAM_TX_DONE];
	void                    *data = s_dcam_mod.user_data[DCAM_TX_DONE];
	struct dcam_path_desc   *path = &s_dcam_mod.dcam_path2;
	struct dcam_frame       *frame = path->output_frame_cur->prev->prev;
	uint32_t                coef_copy = 0;

	if ((0 == s_dcam_mod.dcam_path2.valide) || (DCAM_ST_STOP == s_dcam_mod.dcam_path2.status)) {
		DCAM_TRACE("DCAM DRV: path 2 is not valid: 0x%x \n", REG_RD(DCAM_CFG));
		return;
	}

	//printk("DCAM 2\n");
	
	if (s_dcam_mod.dcam_path2.path_update) {
		s_dcam_mod.dcam_path2.path_done_cnt++;
		DCAM_TRACE("DCAM: path 2 update, %d \n", s_dcam_mod.dcam_path1.path_done_cnt);
		if (1 == s_dcam_mod.dcam_path2.path_done_cnt) {
			coef_copy = 1;
			_dcam_path2_set();
		} else {
			coef_copy = 0;
			if (s_dcam_mod.dcam_path2.path_done_cnt > 2) {
				s_dcam_mod.dcam_path2.path_update = 0;
				DCAM_TRACE("path 2, update path done \n");
			}
		}
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, false);
		if (rtn) {
			DCAM_TRACE("DCAM DRV: path 2 update wait for frame unlocked \n");
			return;
		}
		_dcam_auto_copy_ext(DCAM_PATH_IDX_2, true, coef_copy);

	} else {
		rtn = _dcam_path_set_next_frm(DCAM_PATH_IDX_2, false);
		if (rtn) {
			DCAM_TRACE("DCAM DRV: path 2 wait for frame unlocked \n");
			return;
		}
		_dcam_auto_copy(DCAM_PATH_IDX_2);
	}

	frame->width = path->output_size.w;
	frame->height = path->output_size.h;
	
	dcam_frame_lock(frame);
	if (user_func) {
		(*user_func)(frame, data);
	}
	return;
}

LOCAL void    _path2_ov(void)
{
	printk("DCAM DRV: _path2_ov \n");

	return;
}

LOCAL void    _isp_ov(void)
{
	printk("DCAM DRV: _isp_ov \n");

	return;
}

LOCAL void    _mipi_ov(void)
{
	printk("DCAM DRV: _mipi_ov \n");

	return;
}

LOCAL void    _path1_slice_done(void)
{
	printk("DCAM DRV: _path1_slice_done \n");

	return;
}

LOCAL void    _path2_slice_done(void)
{
	printk("DCAM DRV: _path2_slice_done \n");

	return;
}

LOCAL void    raw_slice_done(void)
{
	printk("DCAM DRV: raw_slice_done \n");

	return;
}

LOCAL void    _dcam_wait_for_stop(void)
{
	int                     rtn = -1;

	s_path1_wait = 1;
	rtn = down_interruptible(&s_done_sema);
	if (rtn) {
		printk("DCAM DRV: Failed down");
	}
	return;
}

LOCAL void    _dcam_stopped(void)
{
	if (s_path1_wait) {
		up(&s_done_sema);
		s_path1_wait = 0;
	}
	return;
}


