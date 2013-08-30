/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <linux/io.h>
#include <mach/adi.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>

#define REGS_FGU_BASE ANA_FPU_INT_BASE

/* 
 */
/* registers definitions for controller REGS_FGU */
#define REG_FGU_START                   SCI_ADDR(REGS_FGU_BASE, 0x0000)
#define REG_FGU_CONFIG                  SCI_ADDR(REGS_FGU_BASE, 0x0004)
#define REG_FGU_ADC_CONFIG              SCI_ADDR(REGS_FGU_BASE, 0x0008)
#define REG_FGU_STATUS                  SCI_ADDR(REGS_FGU_BASE, 0x000c)
#define REG_FGU_INT_EN                  SCI_ADDR(REGS_FGU_BASE, 0x0010)
#define REG_FGU_INT_CLR                 SCI_ADDR(REGS_FGU_BASE, 0x0014)
#define REG_FGU_INT_RAW                 SCI_ADDR(REGS_FGU_BASE, 0x0018)
#define REG_FGU_INT_STS                 SCI_ADDR(REGS_FGU_BASE, 0x001c)
#define REG_FGU_VOLT_VAL                SCI_ADDR(REGS_FGU_BASE, 0x0020)
#define REG_FGU_OCV_VAL                 SCI_ADDR(REGS_FGU_BASE, 0x0024)
#define REG_FGU_POCV_VAL                SCI_ADDR(REGS_FGU_BASE, 0x0028)
#define REG_FGU_CURT_VAL                SCI_ADDR(REGS_FGU_BASE, 0x002c)
#define REG_FGU_HIGH_OVER               SCI_ADDR(REGS_FGU_BASE, 0x0030)
#define REG_FGU_LOW_OVER                SCI_ADDR(REGS_FGU_BASE, 0x0034)
#define REG_FGU_VTHRE_HH                SCI_ADDR(REGS_FGU_BASE, 0x0038)
#define REG_FGU_VTHRE_HL                SCI_ADDR(REGS_FGU_BASE, 0x003c)
#define REG_FGU_VTHRE_LH                SCI_ADDR(REGS_FGU_BASE, 0x0040)
#define REG_FGU_VTHRE_LL                SCI_ADDR(REGS_FGU_BASE, 0x0044)
#define REG_FGU_OCV_LOCKLO              SCI_ADDR(REGS_FGU_BASE, 0x0048)
#define REG_FGU_OCV_LOCKHI              SCI_ADDR(REGS_FGU_BASE, 0x004c)
#define REG_FGU_CLBCNT_SETH             SCI_ADDR(REGS_FGU_BASE, 0x0050)
#define REG_FGU_CLBCNT_SETL             SCI_ADDR(REGS_FGU_BASE, 0x0054)
#define REG_FGU_CLBCNT_DELTH          SCI_ADDR(REGS_FGU_BASE, 0x0058)
#define REG_FGU_CLBCNT_DELTL           SCI_ADDR(REGS_FGU_BASE, 0x005c)
#define REG_FGU_CLBCNT_LASTOCVH         SCI_ADDR(REGS_FGU_BASE, 0x0060)
#define REG_FGU_CLBCNT_LASTOCVL         SCI_ADDR(REGS_FGU_BASE, 0x0064)
#define REG_FGU_CLBCNT_VALH             SCI_ADDR(REGS_FGU_BASE, 0x0068)
#define REG_FGU_CLBCNT_VALL             SCI_ADDR(REGS_FGU_BASE, 0x006c)
#define REG_FGU_CLBCNT_QMAXH            SCI_ADDR(REGS_FGU_BASE, 0x0070)
#define REG_FGU_CLBCNT_QMAXL            SCI_ADDR(REGS_FGU_BASE, 0x0074)
#define REG_FGU_QMAX_TOSET        SCI_ADDR(REGS_FGU_BASE, 0x0078)
#define REG_FGU_QMAX_TIMER          SCI_ADDR(REGS_FGU_BASE, 0x007c)
#define REG_FGU_RELAX_CURT_THRE         SCI_ADDR(REGS_FGU_BASE, 0x0080)
#define REG_FGU_RELAX_CNT_THRE          SCI_ADDR(REGS_FGU_BASE, 0x0084)
#define REG_FGU_RELAX_CNT               SCI_ADDR(REGS_FGU_BASE, 0x0088)
#define REG_FGU_OCV_LAST_CNT            SCI_ADDR(REGS_FGU_BASE, 0x008c)
#define REG_FGU_CURT_OFFSET             SCI_ADDR(REGS_FGU_BASE, 0x0090)

/* bits definitions for register REG_FGU_START */
#define BIT_QMAX_UPDATE_EN              ( BIT(2) )
#define BIT_FGU_RESET                   ( BIT(1) )
#define BIT_WRITE_SELCLB_EN             ( BIT(0) )

/* bits definitions for register REG_FGU_CONFIG */
#define BIT_VOLT_H_VALID                ( BIT(12) )
#define BIT_FGU_DISABLE_EN              ( BIT(11) )
#define BIT_CLBCNT_DELTA_MODE           ( BIT(10) )
#define BITS_ONEADC_DUTY(_x_)           ( (_x_) << 8 & (BIT(8)|BIT(9)) )
#define BIT_CURT_DUTY                   ( BIT(7) )
#define BITS_VOLT_DUTY(_x_)             ( (_x_) << 5 & (BIT(5)|BIT(6)) )
#define BIT_AD1_ENABLE                  ( BIT(4) )
#define BIT_SW_DIS_CURT                 ( BIT(3) )
#define BIT_FORCE_LOCK_EN               ( BIT(2) )
#define BIT_LOW_POWER_MODE              ( BIT(1) )
#define BIT_AUTO_LOW_POWER              ( BIT(0) )

/* bits definitions for register REG_FGU_ADC_CONFIG */
#define BIT_FORCE_AD1_VIN_EN            ( BIT(7) )
#define BIT_FORCE_AD0_VIN_EN            ( BIT(6) )
#define BIT_FORCE_AD0_IIN_EN            ( BIT(5) )
#define BIT_FORCE_AD_EN                 ( BIT(4) )
#define BIT_AD1_VOLT_REF                ( BIT(3) )
#define BIT_AD0_VOLT_REF                ( BIT(2) )
#define BIT_AD01_RESET                  ( BIT(1) )
#define BIT_AD01_PD                     ( BIT(0) )

/* bits definitions for register REG_FGU_STATUS */
#define BIT_POWER_LOW                   ( BIT(5) )
#define BIT_CURT_LOW                    ( BIT(4) )
#define BITS_OCV_LOCK_STS(_x_)          ( (_x_) << 2 & (BIT(2)|BIT(3)) )
#define BIT_QMAX_UPDATE_STS             ( BIT(1) )
#define BIT_WRITE_ACTIVE_STS            ( BIT(0) )

/* bits definitions for register REG_FGU_INT_EN */
#define BIT_CURT_RDEN_INT               ( BIT(7) )
#define BIT_VOLT_RDEN_INT               ( BIT(6) )
#define BIT_QMAX_UPD_TOUT               ( BIT(5) )
#define BIT_QMAX_UPD_DONE               ( BIT(4) )
#define BIT_RELX_CNT_INT                ( BIT(3) )
#define BIT_CLBCNT_DELTA_INT            ( BIT(2) )
#define BIT_VOLT_HIGH_INT               ( BIT(1) )
#define BIT_VOLT_LOW_INT                ( BIT(0) )

#define BITS_VOLT_VALUE(_x_)            ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

/* bits definitions for register REG_FGU_CURT_VAL */
#define BITS_CURT_VALUE(_x_)            ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

/* bits definitions for register REG_FGU_CLBCNT_SETH */
#define BITS_CLBCNT_SETH(_x_)           ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

/* bits definitions for register REG_FGU_CLBCNT_SETL */
#define BITS_CLBCNT_SETL(_x_)           ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

/* bits definitions for register REG_FGU_CLBCNT_DELTHAH */
#define BITS_CLBCNT_DELTHH(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

/* bits definitions for register REG_FGU_CLBCNT_DELTAL */
#define BITS_CLBCNT_DELTHL(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

/* bits definitions for register REG_FGU_RELAX_CURT_THRE */
#define BITS_RELAX_CUR_THRE(_x_)        ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

/* bits definitions for register REG_FGU_RELAX_CNT_THRE */
#define BITS_RELAX_CNT_THRE(_x_)        ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

/* bits definitions for register REG_FGU_RELAX_CNT */
#define BITS_RELAX_CNT_VAL(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

/* bits definitions for register REG_FGU_OCV_LAST_CNT */
#define BITS_OCV_LAST_CNT(_x_)          ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)) )

/* bits definitions for register REG_FGU_CURT_OFFSET */
#define BITS_CURT_OFFSET_VAL(_x_)       ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)) )

//#define CUR_1000MA_ADC   2872 //177//0xF9 //176   OK
//#define VOL_1000MV_ADC    678 //40
//#define VOL_OFFSET  -22
//#define CURRENT_OFFSET  87

static int cur_1000ma_adc;
static int vol_1000mv_adc;
static int vol_offset;
static int cur_offset;

#define SPRDFGU__DEBUG
#ifdef SPRDFGU__DEBUG
#define FGU_DEBUG(format, arg...) do{\
    printk("sprd fgu: " "@@*****@" format, ## arg);\
    }while(0)
#else
#define FGU_DEBUG(format, arg...)
#endif

#define VOL_0mv_ADC   4076
#define VOL_40mv_ADC    6790
#define CUR_0ma_ADC 7944
#define VOL_0mv_IDEA_ADC    4096
#define CUR_0ma_IDEA_ADC    8192
#define FGU_IMPEDANCE   212	//21.2moh
#define FGU_IMPEDANCE_IDEA  200	//200
static int sprdfgu_cal_init(void)
{
	uint32_t achip_id_low = sci_adi_read(ANA_REG_GLB_CHIP_ID_LOW);
	printk("sprdfgu_cal_init\n");
	vol_offset = VOL_0mv_IDEA_ADC - VOL_0mv_ADC;
	cur_offset = CUR_0ma_IDEA_ADC - CUR_0ma_ADC;
	cur_1000ma_adc =
	    ((VOL_40mv_ADC - VOL_0mv_ADC) * FGU_IMPEDANCE +
	     FGU_IMPEDANCE_IDEA / 2)
	    / FGU_IMPEDANCE_IDEA;
	vol_1000mv_adc = ((VOL_40mv_ADC - VOL_0mv_ADC) + 2) / 4;

	if (0xA000 == achip_id_low) {
		vol_offset = 327;
		cur_offset = 59;
		cur_1000ma_adc = 1760;
		vol_1000mv_adc = 500;
	}

	printk
	    ("cur_1000ma_adc = %d,vol_1000mv_adc = %d,vol_offset = %d,cur_offset = %d\n",
	     cur_1000ma_adc, vol_1000mv_adc, vol_offset, cur_offset);
}

static u32 fgu_adc2vol_mv(u32 adc)
{
	return ((adc + vol_offset) * 1000) / vol_1000mv_adc;
}

static u32 fgu_vol2adc_mv(u32 vol)
{
	return (vol * vol_1000mv_adc) / 1000 - vol_offset;
}

static int fgu_adc2cur_ma(int adc)
{
	return (adc * 1000) / cur_1000ma_adc;
}

static u32 fgu_cur2adc_ma(u32 cur)
{
	return (cur * cur_1000ma_adc) / 100;
}

static u32 pocv_raw, poweron_voltage;
static int init_clbcnt;
static u32 start_time;

static struct timer_list fgu_timer;
//static struct wake_lock fgu_lock;

#define REG_SYST_VALUE                  (SPRD_SYSCNT_BASE + 0x0004)
static u32 sci_syst_read(void)
{
	u32 t = __raw_readl(REG_SYST_VALUE);
	while (t != __raw_readl(REG_SYST_VALUE))
		t = __raw_readl(REG_SYST_VALUE);
	return t;
}

static inline int fgu_clbcnt_get(void)
{
	int cc1;

	cc1 = (sci_adi_read(REG_FGU_CLBCNT_VALL)) & 0xFFFF;
	cc1 |= (((sci_adi_read(REG_FGU_CLBCNT_VALH)) & 0xFFFF) << 16);
	return cc1;
}

int fgu_avg_current_query(void)
{
	int cur_cc, cc_delta, raw_avg, temp, curr_avg, capcity;
	u32 cur_time = sci_syst_read();
	u32 time_delta;

	cur_cc = fgu_clbcnt_get();
	time_delta = cur_time - start_time;
	cc_delta = cur_cc - init_clbcnt;
	temp = time_delta / 500;
	raw_avg = cc_delta / temp;

	FGU_DEBUG("start_time:%d,cur_time : %d,init_clbcnt: 0x%x,cur_cc:0x%x\n",
		  start_time, cur_time, init_clbcnt, cur_cc);
	FGU_DEBUG("time_delta : %d,cc_delta: %d,raw_avg = %d\n", time_delta,
		  cc_delta, raw_avg);
	curr_avg = fgu_adc2cur_ma(raw_avg);

	temp = time_delta / 3600;
	capcity = temp * curr_avg;
	FGU_DEBUG("the result you must capcity/1000 capcity = :  %dmah\n",
		  capcity);
	return curr_avg;
}

uint32_t sprdfgu_read_vbat_vol(void)
{
	u32 cur_vol_raw;
	uint32_t temp;
	cur_vol_raw = sci_adi_read(REG_FGU_VOLT_VAL);
	//FGU_DEBUG("cur_vol_raw = %x\n", cur_vol_raw);
	temp = fgu_adc2vol_mv(cur_vol_raw);
	//FGU_DEBUG("sprdfgu_read_vbat_vol : %d\n", temp);
	return temp;
}

u32 sprdfgu_read_soc(void)
{
	return 100;
}

static inline u32 fgu_ocv_vol_get(void)
{
	u32 ocv_vol_raw;
	ocv_vol_raw = sci_adi_read(REG_FGU_OCV_VAL);
	//FGU_DEBUG("ocv_vol_raw = %x\n", ocv_vol_raw);
	return fgu_adc2vol_mv(ocv_vol_raw);
}

static inline int fgu_cur_current_get(void)
{
	int current_raw;

	current_raw = sci_adi_read(REG_FGU_CURT_VAL);
	//FGU_DEBUG("current_raw: 0x%x\n", current_raw);

	return fgu_adc2cur_ma(current_raw - 0x2000);

}

int sprdfgu_read_batcurrent(void)
{
	int temp = fgu_cur_current_get();
	//FGU_DEBUG("sprdfgu_read_batcurrent : %d\n", temp);
	return temp;
}

#define BATTERY_INTERNAL_IMPEDANCE  250	//250moh
uint32_t sprdfgu_read_vbat_ocv(void)
{
	return sprdfgu_read_vbat_vol() -
	    (sprdfgu_read_batcurrent() * BATTERY_INTERNAL_IMPEDANCE) / 1000;
}

static void fgu_handler(unsigned long data)
{
	FGU_DEBUG("dump fgu message@@@@@@@@@@@@@@@@@@@@@@@@@@@start\n");
	FGU_DEBUG("fgu_avg_current_query avg current = %d\n",
		  fgu_avg_current_query());
	FGU_DEBUG("cur vol = %d\n", sprdfgu_read_vbat_vol());
	FGU_DEBUG("ocv vol = %d\n", fgu_ocv_vol_get());
	FGU_DEBUG("pocv_raw = 0x%x,pocv_voltage = %d\n", pocv_raw,
		  fgu_adc2vol_mv(pocv_raw));
	FGU_DEBUG("current current = %d\n", fgu_cur_current_get());
	FGU_DEBUG("dump fgu message@@@@@@@@@@@@@--!!!-@@@@@@@@@@@@@@end\n");
	mod_timer(&fgu_timer, jiffies + 60 * HZ);
}

static void fgu_hw_init(void)
{
	u32 cur_vol_raw, ocv_raw;
	int current_raw;
	FGU_DEBUG("FGU_Init\n");

	sci_adi_set(ANA_REG_GLB_MP_MISC_CTRL, (BIT(1)));
	sci_adi_write(ANA_REG_GLB_DCDC_CTRL2, (4 << 8), (7 << 8));

	sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_FGU_EN);
	sci_adi_set(ANA_REG_GLB_RTC_CLK_EN, BIT_RTC_FGU_EN | BIT_RTC_FGUA_EN);
	sci_adi_clr(REG_FGU_CONFIG, BIT_VOLT_H_VALID);
	//sci_adi_clr(REG_FGU_CONFIG, BIT_AD1_ENABLE);
	sci_adi_write(REG_FGU_CURT_OFFSET, cur_offset, ~0);
	sci_adi_write(REG_FGU_CONFIG, BITS_VOLT_DUTY(3), BITS_VOLT_DUTY(3));	//mingwei

	mdelay(3);

	pocv_raw = sci_adi_read(REG_FGU_POCV_VAL);
	cur_vol_raw = sci_adi_read(REG_FGU_VOLT_VAL);
	ocv_raw = sci_adi_read(REG_FGU_OCV_VAL);
	current_raw = sci_adi_read(REG_FGU_CURT_VAL);
	start_time = sci_syst_read();
	init_clbcnt = fgu_clbcnt_get();

	FGU_DEBUG("pocv_raw = 0x%x,pocv_voltage = %d\n", pocv_raw,
		  fgu_adc2vol_mv(pocv_raw));
	FGU_DEBUG("current voltage raw_data =  0x%x,cur voltage = %d\n",
		  cur_vol_raw, fgu_adc2vol_mv(cur_vol_raw));
	FGU_DEBUG("ocv_raw: 0x%x,ocv voltage = %d\n", ocv_raw,
		  fgu_adc2vol_mv(ocv_raw));
	FGU_DEBUG("current_raw: 0x%x,current = %d\n", current_raw,
		  fgu_adc2cur_ma(current_raw - 0x2000));
	FGU_DEBUG("init_clbcnt: 0x%x\n", init_clbcnt);

	//wake_lock_init(&fgu_lock, WAKE_LOCK_SUSPEND, "fgu_lock");
	//wake_lock(&fgu_lock);
	init_timer(&fgu_timer);
	fgu_timer.function = fgu_handler;
	mod_timer(&fgu_timer, jiffies + 45 * HZ);
}

int sprdfgu_init(void)
{
	sprdfgu_cal_init();
	fgu_hw_init();
}

uint16_t voltage_capacity_table[][2] = {
	{4180, 100},
	{4100, 95},
	{3980, 80},
	{3900, 70},
	{3840, 60},
	{3800, 50},
	{3760, 40},
	{3730, 30},
	{3700, 20},
	{3650, 15},
	{3600, 5},
	{3400, 0},
};

uint32_t sprdfgu_read_capacity(void)
{
	uint16_t percentum;
	int32_t temp;
	int32_t voltage;
	uint16_t table_size;
	int pos = 0;

	voltage = sprdfgu_read_vbat_ocv();

	FGU_DEBUG("sprdfgu_read_capacity voltage: %d\n", voltage);

	table_size = ARRAY_SIZE(voltage_capacity_table);
	for (pos = 0; pos < table_size - 1; pos++) {
		if (voltage > voltage_capacity_table[pos][0])
			break;
	}
	if (pos == 0) {
		percentum = 100;
	} else {
		temp =
		    voltage_capacity_table[pos][1] -
		    voltage_capacity_table[pos - 1][1];
		temp = temp * (voltage - voltage_capacity_table[pos][0]);
		temp =
		    temp / (voltage_capacity_table[pos][0] -
			    voltage_capacity_table[pos - 1][0]);
		temp = temp + voltage_capacity_table[pos][1];
		if (temp < 0)
			temp = 0;
		percentum = temp;
	}

	return percentum;
}
