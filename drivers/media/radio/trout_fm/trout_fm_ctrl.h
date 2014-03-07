#ifndef TROUT_FM_CTRL_H__
#define TROUT_FM_CTRL_H__

#include "trout_interface.h"

#define SPI_WR_ERG_CMD(count,addr, sys) ((1<<31) | (1<<30) | ((sys)<<29)| (((count-1)& 0x1ff) <<20) | (addr & 0xffff))
#define SPI_RD_ERG_CMD(count,addr, sys) ((0<<31) | (1<<30) | ((sys)<<29)| (((count-1)& 0x1ff) <<20) | (addr & 0xffff))
#define SPI_MAX_SPEED 24*1000*1000   //12M

//#define	TROUT_FM_DEV_NAME	"trout_fm"
#define	TROUT_FM_DEV_NAME	"radio0"

/* seek direction */
#define Trout_SEEK_DIR_UP          0
#define Trout_SEEK_DIR_DOWN        1

/** The following define the IOCTL command values via the ioctl macros */
#define	Trout_FM_IOCTL_BASE     'R'
#define	Trout_FM_IOCTL_ENABLE		 _IOW(Trout_FM_IOCTL_BASE, 0, int)
#define Trout_FM_IOCTL_GET_ENABLE  _IOW(Trout_FM_IOCTL_BASE, 1, int)
#define Trout_FM_IOCTL_SET_TUNE    _IOW(Trout_FM_IOCTL_BASE, 2, int)
#define Trout_FM_IOCTL_GET_FREQ    _IOW(Trout_FM_IOCTL_BASE, 3, int)
#define Trout_FM_IOCTL_SEARCH      _IOW(Trout_FM_IOCTL_BASE, 4, int[4])
#define Trout_FM_IOCTL_STOP_SEARCH _IOW(Trout_FM_IOCTL_BASE, 5, int)
#define Trout_FM_IOCTL_MUTE        _IOW(Trout_FM_IOCTL_BASE, 6, int)
#define Trout_FM_IOCTL_SET_VOLUME  _IOW(Trout_FM_IOCTL_BASE, 7, int)
#define Trout_FM_IOCTL_GET_VOLUME  _IOW(Trout_FM_IOCTL_BASE, 8, int)

//chenq add a ioctl cmd for deal i2s work,in songkun mail,2013-01-17
#define Trout_FM_IOCTL_CONFIG  _IOW(Trout_FM_IOCTL_BASE, 9, int)
//chenq add end

#define READ_FM_REG			9
#define WRITE_FM_REG		10
#define READ_FM_RF_REG			11
#define WRITE_FM_RF_REG		12

#define	FM_CTL_STI_MODE_NORMAL		0X0
#define	FM_CTL_STI_MODE_SEEK		0X1
#define	FM_CTL_STI_MODE_TUNE		0X2

#define	FM_REG_FM_CTRL			0X3000
#define	FM_REG_FM_EN			0X3001
#define	FM_REG_RF_INIT_GAIN		0X3002
#define	FM_REG_CHAN				0X3003
#define	FM_REG_AGC_TBL_CLK		0X3004
#define	FM_REG_SEEK_LOOP		0X3005
#define	FM_REG_FMCTL_STI		0X3006
#define	FM_REG_BAND_LMT			0X3007
#define	FM_REG_BAND_SPACE		0X3008
#define	FM_REG_RF_CTL			0X3009
#define	FM_REG_ADC_BITMAP		0X300A
#define	FM_REG_INT_EN			0X300C
#define	FM_REG_INT_CLR			0X300D
#define	FM_REG_INT_STS			0X300E
#define	FM_REG_SEEK_CH_TH		0X3010
#define	FM_REG_AGC_TBL_RFRSSI0	0X3011
#define	FM_REG_AGC_TBL_RFRSSI1	0X3012
#define	FM_REG_AGC_TBL_WBRSSI	0X3013
#define	FM_REG_AGC_IDX_TH		0X3014
#define	FM_REG_AGC_RSSI_TH		0X3015
#define	FM_REG_SEEK_ADPS		0X3016
#define	FM_REG_STER_PWR			0X3017
#define	FM_REG_AGC_CTRL			0X3018
#define	FM_REG_AGC_ITV_TH		0X3019
#define	FM_REG_AGC_ADPS0		0X301A
#define	FM_REG_AGC_ADPS1		0X301B
#define	FM_REG_PDP_TH			0X301C
#define	FM_REG_PDP_DEV			0X301D
#define	FM_REG_ADP_ST_CONF		0X301E
#define	FM_REG_ADP_LPF_CONF		0X301F
#define	FM_REG_DEPHA_SCAL		0X3020
#define	FM_REG_HW_MUTE			0X3021
#define	FM_REG_SW_MUTE			0X3022
#define	FM_REG_UPD_CTRL			0X3023
#define	FM_REG_AUD_BLD0			0X3024
#define	FM_REG_AUD_BLD1			0X3025
#define	FM_REG_AGC_HYS			0X3026
#define	FM_REG_MONO_PWR			0X3027
#define	FM_REG_RF_CFG_DLY				0X3028
#define	FM_REG_AGC_TBL_STS				0X3029
#define	FM_REG_ADP_BIT_SFT				0X302A
#define	FM_REG_SEEK_CNT					0X302B
#define	FM_REG_RSSI_STS					0X302C
#define	FM_REG_CHAN_FREQ_STS			0X302E
#define	FM_REG_FREQ_OFF_STS				0X302F
#define	FM_REG_INPWR_STS				0X3030
#define	FM_REG_RF_RSSI_STS				0X3031
#define	FM_REG_NO_LPF_STS				0X3032
#define	FM_REG_SMUTE_STS				0X3033
#define	FM_REG_WBRSSI_STS				0X3034
#define	FM_REG_AGC_BITS_TBL0			0X3040
#define	FM_REG_AGC_BITS_TBL1			0X3041
#define	FM_REG_AGC_BITS_TBL2			0X3042
#define	FM_REG_AGC_BITS_TBL3			0X3043
#define	FM_REG_AGC_BITS_TBL4			0X3044
#define	FM_REG_AGC_BITS_TBL5			0X3045
#define	FM_REG_AGC_BITS_TBL6			0X3046
#define	FM_REG_AGC_BITS_TBL7			0X3047
#define	FM_REG_AGC_BITS_TBL8			0X3048
#define	FM_REG_AGC_BITS_TBL9			0X3049
#define	FM_REG_AGC_BITS_TBL10			0X304A
#define	FM_REG_AGC_BITS_TBL11			0X304B
#define	FM_REG_AGC_BITS_TBL12			0X304C
#define	FM_REG_AGC_BITS_TBL13			0X304D
#define	FM_REG_AGC_BITS_TBL14			0X304E
#define	FM_REG_AGC_BITS_TBL15			0X304F
#define	FM_REG_AGC_BITS_TBL16			0X3050
#define	FM_REG_AGC_BITS_TBL17			0X3051
#define	FM_REG_AGC_VAL_TBL0				0X3080
#define	FM_REG_AGC_VAL_TBL1				0X3081
#define	FM_REG_AGC_VAL_TBL2				0X3082
#define	FM_REG_AGC_VAL_TBL3				0X3083
#define	FM_REG_AGC_VAL_TBL4				0X3084
#define	FM_REG_AGC_VAL_TBL5				0X3085
#define	FM_REG_AGC_VAL_TBL6				0X3086
#define	FM_REG_AGC_VAL_TBL7				0X3087
#define	FM_REG_AGC_VAL_TBL8				0X3088
#define	FM_REG_AGC_VAL_TBL9				0X3089
#define	FM_REG_AGC_VAL_TBL10			0X308A
#define	FM_REG_AGC_VAL_TBL11			0X308B
#define	FM_REG_AGC_VAL_TBL12			0X308C
#define	FM_REG_AGC_VAL_TBL13			0X308D
#define	FM_REG_AGC_VAL_TBL14			0X308E
#define	FM_REG_AGC_VAL_TBL15			0X308F
#define	FM_REG_AGC_VAL_TBL16			0X3090
#define	FM_REG_AGC_VAL_TBL17			0X3091
#define	FM_REG_AGC_BOND_TBL0			0X30C0
#define	FM_REG_AGC_BOND_TBL1			0X30C1
#define	FM_REG_AGC_BOND_TBL2			0X30C2
#define	FM_REG_AGC_BOND_TBL3			0X30C3
#define	FM_REG_AGC_BOND_TBL4			0X30C4
#define	FM_REG_AGC_BOND_TBL5			0X30C5
#define	FM_REG_AGC_BOND_TBL6			0X30C6
#define	FM_REG_AGC_BOND_TBL7			0X30C7
#define	FM_REG_AGC_BOND_TBL8			0X30C8
#define	FM_REG_AGC_BOND_TBL9			0X30C9
#define	FM_REG_AGC_DB_TBL_BEGIN			0X3140
#define	FM_REG_AGC_DB_TBL_END			0X3168
#define	FM_REG_AGC_DB_TBL_CNT			(FM_REG_AGC_DB_TBL_END+1-FM_REG_AGC_DB_TBL_BEGIN)

#define FM_REG_SPUR_FREQ0 0x30D0
#define FM_REG_SPUR_FREQ1 0x30D1


#define	FM_REG_SPI_CTL					0x3204
#define	FM_REG_SPI_WD0					0X3205
#define	FM_REG_SPI_WD1					0x3206
#define	FM_REG_SPI_RD					0x3207
#define	FM_REG_SPI_FIFO_STS				0x3208

#if 0
#define SYS_REG_PAD_IISDO				0x7
#define SYS_REG_PAD_IISCLK				0x8
#define SYS_REG_PAD_IISLRCK				0x9
#define	SYS_REG_CLK_CTRL1				0X21
#endif

#define RF_SPI_REG_START_ADDR			0x1000
#define RF_SPI_REG_END_ADDR				0x1FFF
#define RF_SPIREG_COUNT					(RF_SPI_REG_END_ADDR + 1 - RF_SPI_REG_START_ADDR)


#define COM_REG_RF_SPI_CTRL				0x4012

#define	BIT_0		0x00000001
#define	BIT_1		0x00000002
#define	BIT_2		0x00000003
#define	BIT_3		0x00000008
#define	BIT_4		0x00000010
#define	BIT_5		0x00000020
#define	BIT_6		0x00000040
#define	BIT_7		0x00000080
#define	BIT_8		0x00000100
#define	BIT_9		0x00000200
#define	BIT_10		0x00000400
#define	BIT_11		0x00000800
#define	BIT_12		0x00001000
#define	BIT_13		0x00002000
#define	BIT_14		0x00004000
#define	BIT_15		0x00008000
#define	BIT_16		0x00010000
#define	BIT_17		0x00020000
#define	BIT_18		0x00040000
#define	BIT_19		0x00080000
#define	BIT_20		0x00100000
#define	BIT_21		0x00200000
#define	BIT_22		0x00400000
#define	BIT_23		0x00800000
#define	BIT_24		0x01000000
#define	BIT_25		0x02000000
#define	BIT_26		0x04000000
#define	BIT_27		0x08000000
#define	BIT_28		0x10000000
#define	BIT_29		0x20000000
#define	BIT_30		0x40000000
#define	BIT_31		0x80000000

#define TROUT_PIN_Z_EN               0           		 // High-Z in sleep mode
#define TROUT_PIN_I_EN               BIT_7           	// Input enable in sleep mode
#define TROUT_PIN_O_EN              BIT_8           	// Output enable in sleep mode
#define TROUT_PIN_IO_MSK          (BIT_7|BIT_8)

#define TROUT_PIN_SPD_EN           BIT_9           // Pull down enable for sleep mode
#define TROUT_PIN_SPU_EN           BIT_10         // Pull up enable for sleep mode
#define TROUT_PIN_SPX_EN           0            	// Don't pull down or up
#define TROUT_PIN_SP_MSK	     (BIT_9|BIT_10)

#define TROUT_PIN_FUNC_DEF        0       //Function select,BIT4-5
#define TROUT_PIN_FUNC_1            BIT_0
#define TROUT_PIN_FUNC_2            BIT_1
#define TROUT_PIN_FUNC_3            (BIT_1|BIT_0)
#define TROUT_PIN_FUNC_MSK	      TROUT_PIN_FUNC_3

#define TROUT_PIN_DS_0                0  
#define TROUT_PIN_DS_1                (BIT_5)
#define TROUT_PIN_DS_2                (BIT_6)
#define TROUT_PIN_DS_3                (BIT_6|BIT_5)
#define TROUT_PIN_DS_MSK	      TROUT_PIN_DS_3

#define TROUT_PIN_FPD_EN            BIT_3           // Weak pull down for function mode
#define TROUT_PIN_FPU_EN            BIT_4           // Weak pull up for function mode
#define TROUT_PIN_FPX_EN            0            	// Don't pull down or up
#define TROUT_PIN_FP_MSK           (BIT_3|BIT_4)

typedef	struct
{
	u32	addr;
	u32	data;
}trout_reg_cfg_t;

#define TROUT_PRINT(format, arg...)  if(1){printk("trout_fm %s-%d -- "format"\n",__FUNCTION__,__LINE__,## arg);}

extern trout_interface_t   *p_trout_interface;

/* set trout wifi goto sleep */
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
extern void wifimac_sleep(void); //hugh:add for power 20130425
#endif

static inline unsigned int read_fm_reg(u32 reg_addr, u32 *reg_data)
{
    if(p_trout_interface)
    {
	    return p_trout_interface->read_reg(reg_addr, reg_data);
    }

    return -1;
}

static inline unsigned int write_fm_reg (u32 reg_addr,  u32 val)
{
    if(p_trout_interface)
    {
	    return p_trout_interface->write_reg(reg_addr, val);
    }

    return -1;
}

static inline unsigned int write_fm_regs (trout_reg_cfg_t *reg_cfg,  u32 cnt)
{
	u32 i = 0;
	for (i = 0; i < cnt; i++)
	{
		if (write_fm_reg(reg_cfg[i].addr, reg_cfg[i].data) < 0)
			return -1;
	}

	return 0;
}

#define WRITE_REG(addr, reg) \
{ \
	if(write_fm_reg(addr, reg) < 0) \
		return -1; \
}

#define READ_REG(addr, reg) \
{ \
	if(read_fm_reg(addr, reg) < 0) \
		return -1; \
}

#if 0
#define WRITE_RF_REG(addr, reg) \
{ \
	if(trout_fm_rf_write(addr, reg) < 0) \
		return -1; \
}
#else
#define WRITE_RF_REG(addr, reg) trout_fm_rf_write(addr, reg)
#endif

#define READ_RF_REG(addr, reg) \
{ \
	if(trout_fm_rf_read(addr, reg) < 0) \
		return -1; \
}

#endif
