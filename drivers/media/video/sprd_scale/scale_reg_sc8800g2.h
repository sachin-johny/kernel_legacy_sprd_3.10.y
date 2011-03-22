#ifndef _SCALE_REG_SC8800G2_H_
#define _SCALE_REG_SC8800G2_H_

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/io.h>
#include <mach/io.h>
#include <mach/bits.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define __pad(a) __raw_readl(a)
#define _pard(a) __raw_readl(a)
#define _pawd(a,v) __raw_writel(v,a)
#define _paad(a,v) __raw_bits_and(v,a)
#define _paod(a,v) __raw_bits_or(v,a)
#define _pacd(a,v) __raw_bits_xor(v,a)
#define _pamd(a,m,v) do{uint32 _tmp=__raw_readl(a); _tmp &= ~(m); __raw_writel(_tmp|((m)&(v)), (a));}while(0)

#define BOOLEAN char
#define PNULL  ((void *)0)
#ifndef PUBLIC
#define PUBLIC 
#endif
#ifndef LOCAL
#define LOCAL static
#endif
#ifndef NULL
#define NULL 0
#endif

#define SCALE_Sleep msleep


#define SCALE_SUCCESS 1
#define SCALE_FAIL 0
#define SCALE_FALSE 0
#define SCALE_TRUE 1
#define SCALE_NULL 0
#ifndef SCALE_ASSERT
#define SCALE_ASSERT(a) do{}while(!(a));
#endif
#ifndef SCALE_MEMCPY
#define SCALE_MEMCPY memcpy
#endif
#ifndef SCALE_TRACE
#define SCALE_TRACE printk
#endif
typedef void *DCAM_MUTEX_PTR;
typedef void *DCAM_SEMAPHORE_PTR;
typedef void *DCAM_TIMER_PTR;

//0xE0002000UL <-->0x20200000
#define DCAM_REG_BASE SPRD_ISP_BASE 
#define DCAM_CFG DCAM_REG_BASE + 0x00UL
#define DCAM_PATH_CFG DCAM_REG_BASE + 0x04UL
#define DCAM_SRC_SIZE DCAM_REG_BASE + 0x08UL
#define DCAM_DES_SIZE DCAM_REG_BASE + 0x0CUL
#define DCAM_TRIM_START DCAM_REG_BASE + 0x10UL
#define DCAM_TRIM_SIZE DCAM_REG_BASE + 0x14UL
#define REV_PATH_CFG DCAM_REG_BASE + 0x18UL
#define REV_SRC_SIZE DCAM_REG_BASE + 0x1CUL
#define REV_DES_SIZE DCAM_REG_BASE + 0x20UL
#define REV_TRIM_START DCAM_REG_BASE + 0x24UL
#define REV_TRIM_SIZE DCAM_REG_BASE + 0x28UL
#define SLICE_VER_CNT DCAM_REG_BASE + 0x2CUL
#define DCAM_INT_STS DCAM_REG_BASE + 0x30UL
#define DCAM_INT_MASK DCAM_REG_BASE + 0x34UL
#define DCAM_INT_CLR DCAM_REG_BASE + 0x38UL
#define DCAM_INT_RAW DCAM_REG_BASE + 0x3CUL
#define FRM_ADDR_0 DCAM_REG_BASE + 0x40UL
#define FRM_ADDR_1 DCAM_REG_BASE + 0x44UL
#define FRM_ADDR_2 DCAM_REG_BASE + 0x48UL
#define FRM_ADDR_3 DCAM_REG_BASE + 0x4CUL
#define FRM_ADDR_4 DCAM_REG_BASE + 0x50UL
#define FRM_ADDR_5 DCAM_REG_BASE + 0x54UL
#define FRM_ADDR_6 DCAM_REG_BASE + 0x58UL
#define FRM_ADDR_H DCAM_REG_BASE + 0x5CUL
#define SC1_TAB1_TEST_MODE DCAM_REG_BASE + 0x60UL
#define ENDIAN_SEL DCAM_REG_BASE + 0x64UL
#define SC1_TAB1_TEST_DOUT DCAM_REG_BASE + 0x68UL
#define DCAM_ADDR_7 DCAM_REG_BASE + 0x6CUL
#define DCAM_ADDR_8 DCAM_REG_BASE + 0x70UL
#define CAP_CNTRL DCAM_REG_BASE + 0x100UL
#define CAP_FRM_CNT DCAM_REG_BASE + 0x104UL
#define CAP_START DCAM_REG_BASE + 0x108UL
#define CAP_END DCAM_REG_BASE + 0x10CUL
#define CAP_IMAGE_DECI DCAM_REG_BASE + 0x110UL
#define CAP_TRANS DCAM_REG_BASE + 0x114UL
#define CAP_JPG_FRM_CTL DCAM_REG_BASE + 0x11CUL
#define CAP_JPG_FRM_SIZE DCAM_REG_BASE + 0x120UL
#define PREF_CONF DCAM_REG_BASE + 0x200UL
#define BLC_CONF DCAM_REG_BASE + 0x210UL
#define AWB_START DCAM_REG_BASE + 0x220UL
#define AWB_BLK_CONF DCAM_REG_BASE + 0x224UL
#define AWBC_CONF DCAM_REG_BASE + 0x230UL
#define NSR_WEIGHT DCAM_REG_BASE + 0x240UL
#define NSR_LIMIT DCAM_REG_BASE + 0x244UL
#define AF_SHIFT DCAM_REG_BASE + 0x250UL
#define AF_START DCAM_REG_BASE + 0x254UL
#define AF_END DCAM_REG_BASE + 0x258UL
#define AF_RESULT DCAM_REG_BASE + 0x25CUL
#define LNC_CENTER DCAM_REG_BASE + 0x260UL
#define LNC_PARA DCAM_REG_BASE + 0x264UL
#define LNC_CENTER_SQR DCAM_REG_BASE + 0x268UL
#define CFA_CONF DCAM_REG_BASE + 0x270UL
#define CCE_MATRIX_YR DCAM_REG_BASE + 0x280UL
#define CCE_MATRIX_YG DCAM_REG_BASE + 0x284UL
#define CCE_MATRIX_YB DCAM_REG_BASE + 0x288UL
#define CCE_MATRIX_UR DCAM_REG_BASE + 0x28CUL
#define CCE_MATRIX_UG DCAM_REG_BASE + 0x290UL
#define CCE_MATRIX_UB DCAM_REG_BASE + 0x294UL
#define CCE_MATRIX_VR DCAM_REG_BASE + 0x298UL
#define CCE_MATRIX_VG DCAM_REG_BASE + 0x29CUL
#define CCE_MATRIX_VB DCAM_REG_BASE + 0x2A0UL
#define CCE_Y_SHIFT_VALUE DCAM_REG_BASE + 0x2A4UL
#define CCE_U_SHIFT_VALUE DCAM_REG_BASE + 0x2A4UL
#define CCE_V_SHIFT_VALUE DCAM_REG_BASE + 0x2A4UL

//#define DCAM_SRC_SIZE DCAM_REG_BASE + 0x300UL
#define DCAM_SC_MODE DCAM_REG_BASE + 0x304UL
#define DCAM_SC2_SIZE DCAM_REG_BASE + 0x308UL
#define DCAM_SC2_START DCAM_REG_BASE + 0x30CUL
#define DCAM_DISP_SIZE DCAM_REG_BASE + 0x310UL
#define DCAM_DISP_START DCAM_REG_BASE + 0x314UL
#define DCAM_ENC_SIZE DCAM_REG_BASE + 0x318UL
#define DCAM_ENC_START DCAM_REG_BASE + 0x31CUL
#define SC1_CONF DCAM_REG_BASE + 0x320UL
#define SC2_CONF DCAM_REG_BASE + 0x324UL
#define DCAM_DBLK_THR DCAM_REG_BASE + 0x334UL
#define DCAM_SC_LINE_BLANK DCAM_REG_BASE + 0x338UL
#define DCAM_PORTA_CONF DCAM_REG_BASE + 0x340UL
#define DCAM_PORTA_STATE DCAM_REG_BASE + 0x344UL
#define DCAM_PORTA_FRM DCAM_REG_BASE + 0x348UL
#define DCAM_PORTA_CURR_YBA DCAM_REG_BASE + 0x34CUL
#define DCAM_PORTA_CURR_UBA DCAM_REG_BASE + 0x350UL
#define DCAM_PORTA_CURR_VBA DCAM_REG_BASE + 0x344UL
#define DCAM_PORTA_NEXT_YBA DCAM_REG_BASE + 0x358UL
#define DCAM_PORTA_NEXT_UBA DCAM_REG_BASE + 0x35CUL
#define DCAM_PORTA_NEXT_VBA DCAM_REG_BASE + 0x360UL
#define DCAM_PORTA_LAST_END DCAM_REG_BASE + 0x364UL
#define DCAM_PORTA_CURR_YADDR DCAM_REG_BASE + 0x368UL
#define DCAM_PORTA_SKIP DCAM_REG_BASE + 0x36CUL
#define DCAM_PORTA_TH DCAM_REG_BASE + 0x370UL
#define DCAM_PORTB_CONF DCAM_REG_BASE + 0x380UL
#define DCAM_PORTB_STATE DCAM_REG_BASE + 0x384UL
#define DCAM_PORTB_FRM DCAM_REG_BASE + 0x388UL
#define DCAM_PORTB_CURR_BA DCAM_REG_BASE + 0x38CUL
#define DCAM_PORTB_NEXT_BA DCAM_REG_BASE + 0x390UL
#define DCAM_PORTB_LAST_END DCAM_REG_BASE + 0x394UL
#define DCAM_PORTB_CURR_YADDR DCAM_REG_BASE + 0x398UL
#define DCAM_PORTB_SKIP DCAM_REG_BASE + 0x39CUL
#define DCAM_PORTB_TH DCAM_REG_BASE + 0x3A0UL
#define DCAM_PORTC_START DCAM_REG_BASE + 0x3C0UL
#define DCAM_PORTC_CONF DCAM_REG_BASE + 0x3C4UL
#define DCAM_PORTC_STATE DCAM_REG_BASE + 0x3C8UL
#define DCAM_PORTC_CURR_YBA DCAM_REG_BASE + 0x3CCUL
#define DCAM_PORTC_CURR_UBA DCAM_REG_BASE + 0x3D0UL
#define DCAM_PORTC_CURR_VBA DCAM_REG_BASE + 0x3D4UL
#define DCAM_PORTC_SKIP DCAM_REG_BASE + 0x3D8UL
#define DCAM_PORTC_CURR_YADDR DCAM_REG_BASE + 0x3DCUL
#define VP_REVIEW_SIZE DCAM_REG_BASE + 0x3E0UL
#define VP_REVIEW_SELF_RATE DCAM_REG_BASE + 0x3E4UL

#define DCAM_AWB_RESULT DCAM_REG_BASE + 0x400UL
#define DCAM_OBSERVED_ENABLE DCAM_REG_BASE + 0x3F8UL
#define DCAM_GAMMA_TABLE DCAM_REG_BASE + 0xB00UL
#define DCAM_OBSERVED_STATE DCAM_REG_BASE + 0x3F8UL

#define DCAM_SC1_CEOFX_S DCAM_REG_BASE + 0xC00
#define DCAM_SC1_CEOFX_E DCAM_REG_BASE + 0xD20
#define DCAM_SC1_CEOFY_S DCAM_REG_BASE + 0xE00
#define DCAM_SC1_CEOFY_E DCAM_REG_BASE + 0xF20
#define DCAM_SC2_CEOFX_S DCAM_REG_BASE + 0x1000 
#define DCAM_SC2_CEOFX_E DCAM_REG_BASE + 0x111C
#define DCAM_SC2_CEOFY_S DCAM_REG_BASE + 0x1200
#define DCAM_SC2_CEOFY_E DCAM_REG_BASE + 0x131C


#define GLOBAL_BASE SPRD_GREG_BASE //0xE0002E00UL <--> 0x8b000000
#define ARM_GLOBAL_REG_GEN0 GLOBAL_BASE + 0x008UL
#define ARM_GLOBAL_REG_GEN3 GLOBAL_BASE + 0x01CUL
#define ARM_GLOBAL_PLL_MN GLOBAL_BASE + 0x024UL
#define ARM_GLOBAL_VPLL_MN GLOBAL_BASE + 0x068UL
#define ARM_GLOBAL_PLL_SCR GLOBAL_BASE + 0x070UL
#define GR_CLK_DLY GLOBAL_BASE + 0x05CUL
#define GR_CLK_GEN5 GLOBAL_BASE + 0x07CUL

#define AHB_BASE SPRD_AHB_BASE //0xE000A000 <--> 0x20900000UL
#define AHB_GLOBAL_REG_CTL0 AHB_BASE + 0x200UL
#define AHB_GLOBAL_REG_CTL1 AHB_BASE + 0x204UL
#define AHB_GLOBAL_REG_SOFTRST AHB_BASE + 0x210UL

#define PIN_CTL_BASE SPRD_CPC_BASE//0xE002F000<-->0x8C000000UL
#define PIN_CTL_CCIRPD1 PIN_CTL_BASE + 0x344UL
#define PIN_CTL_CCIRPD0 PIN_CTL_BASE + 0x348UL

#define IRQ_BASE SPRD_ASHB_BASE //0xE0021000<-->0x80003000
#define SCL_INT_IRQ_EN IRQ_BASE + 0x008UL
#define SCL_INT_IRQ_DISABLE IRQ_BASE + 0x00CUL

#define MISC_BASE SPRD_MISC_BASE //0xE0033000<-->0x82000000
#define ANA_REG_BASE MISC_BASE + 0x480
#define ANA_LDO_PD_CTL ANA_REG_BASE + 0x10
#define ANA_LDO_VCTL0 ANA_REG_BASE + 0x14
#define ANA_LDO_VCTL1 ANA_REG_BASE + 0x18
#define ANA_LDO_VCTL2 ANA_REG_BASE + 0x1C

#define ISP_DEFAULT_CLK                   48
#define CAP_FIFO_FULL_SPEED               0
#define CAP_FIFO_HALF_SPEED               1
#define CAP_FIFO_RESERVED                 2
#define CAP_FIFO_DOUBLE_SPEED             3
#define CAP_INPUT_FORMAT_YUV              0
#define CAP_INPUT_FORMAT_JPEG             1
#define ISP_PATH1_FRAME_COUNT_MAX         4
#define ISP_PATH2_FRAME_COUNT_MAX         ISP_PATH1_FRAME_COUNT_MAX
#define ISP_PATH_SC_COEFF_MAX             4
#define ISP_CAP_DEC_FRAME_MAX             15
#define ISP_CAP_DEC_XY_MAX                15
#define ISP_SCALE_FRAME_MODE_WIDTH_TH     640

#define ISP_SCALE_COEFF_H_NUM             48
#define ISP_SCALE_COEFF_V_NUM             68

#define ISP_AHB_SLAVE_ADDR               DCAM_REG_BASE // 0x20200000


#define ISP_AHB_CTRL_MOD_EN_OFFSET        0
#define ISP_AHB_CTRL_MEM_SW_OFFSET        4
#define ISP_AHB_CTRL_SOFT_RESET_OFFSET    0x10


#define ISP_SCALE1_H_TAB_OFFSET                        0x200
#define ISP_SCALE1_V_TAB_OFFSET                        0x2F0
#define ISP_SCALE2_H_TAB_OFFSET                        0x400
#define ISP_SCALE2_V_TAB_OFFSET                        0x4F0

#define ISP_IRQ_SCL_LINE_MASK                              0x200UL //0x000003FFUL
#define ISP_IRQ_SENSOR_SOF_BIT                         BIT_0
#define ISP_IRQ_SENSOR_EOF_BIT                         BIT_1
#define ISP_IRQ_CAP_SOF_BIT                            BIT_2
#define ISP_IRQ_CAP_EOF_BIT                            BIT_3
#define ISP_IRQ_CMR_DONE_BIT                           BIT_4
#define ISP_IRQ_CAP_BUF_OV_BIT                         BIT_5
#define ISP_IRQ_SENSOR_LE_BIT                          BIT_6
#define ISP_IRQ_SENSOR_FE_BIT                          BIT_7
#define ISP_IRQ_JPG_BUF_OV_BIT                         BIT_8
#define ISP_IRQ_REVIEW_DONE_BIT                        BIT_9

#define ISP_CAP_FRAME_SKIP_NUM_MAX                     0x40
#define ISP_CAP_FRAME_DECI_FACTOR_MAX                  0x10
#define ISP_CAP_X_DECI_FACTOR_MAX                      0x10
#define ISP_CAP_Y_DECI_FACTOR_MAX                      0x10
#define ISP_CAP_FRAME_WIDTH_MAX                        4092
#define ISP_CAP_FRAME_HEIGHT_MAX                       4092
#define ISP_CAP_JPEG_DROP_NUM_MIN                      0x01 
#define ISP_CAP_JPEG_DROP_NUM_MAX                      0x10 
#define ISP_PATH_FRAME_HIGH_BITS                       6
#define ISP_PATH_FRAME_WIDTH_MAX                       4092
#define ISP_PATH_FRAME_HEIGHT_MAX                      4092
#define ISP_PATH_SUB_SAMPLE_MAX                        3  //0.....1/2, 1......1/4, 2......1/8, 3.......1/16
#define ISP_PATH_SUB_SAMPLE_FACTOR_BASE                1  // no subsample
#define ISP_PATH_SCALE_LEVEL                           64
#define ISP_PATH_SCALE_LEVEL_MAX                       256
#define ISP_PATH_SCALE_LEVEL_MIN                       16
#define ISP_PATH_SLICE_MASK                            0xFFF

typedef struct _isp_size_tag
{
    uint32_t               w;
    uint32_t               h;
} ISP_SIZE_T;

typedef struct _isp_rect_tag
{
    uint32_t               x;
    uint32_t               y;
    uint32_t               w;
    uint32_t               h;

} ISP_RECT_T;

typedef enum
{
    ISP_SCALE_NOEMAL = 0,
    ISP_SCALE_SLICE,
    ISP_SCALE__MODE_MAX
} ISP_SCALE_MODE_E;
typedef enum
{
    ISP_DATA_YUV422 = 0,
    ISP_DATA_YUV420,
    ISP_DATA_YUV400,
    ISP_DATA_YUV420_3FRAME,
    ISP_DATA_RGB565,
    ISP_DATA_RGB888,
    ISP_DATA_CCIR656,
    ISP_DATA_JPEG,
    ISP_DATA_MAX
} ISP_DATA_FORMAT_E;
enum
{
    ISP_PATH_DATA_FORMAT_YUV = 0,
    ISP_PATH_DATA_FORMAT_RGB 
};
typedef struct _isp_data_addr_tag
{
    uint32_t               yaddr;
    uint32_t               uaddr;
    uint32_t               vaddr;
} ISP_ADDRESS_T;
typedef struct _isp_frame_t
{
    uint32_t               type;
    uint32_t               lock;
    uint32_t               flags;
    uint32_t               fid;
    uint32_t               afval;
    uint32_t               width;
    uint32_t               height;
    uint32_t               yaddr;
    uint32_t               uaddr;
    uint32_t               vaddr;
    uint32_t               rgbaddr;
    struct _isp_frame_t  *prev;
    struct _isp_frame_t  *next;
} ISP_FRAME_T;
typedef struct _isp_path_desc_tag
{
    ISP_SIZE_T               input_size;
    ISP_RECT_T               input_rect;
    ISP_SIZE_T               sc_input_size;
    ISP_SIZE_T               output_size;
    ISP_FRAME_T              input_frame;
    uint32_t                   input_format;
    ISP_FRAME_T              *p_output_frame_head;
    ISP_FRAME_T              *p_output_frame_cur;
    uint32_t                   output_frame_count;    
    uint32_t                   output_format;
    uint32_t                   output_frame_flag;
    ISP_FRAME_T              swap_frame;
    ISP_FRAME_T              line_frame;	
    uint32_t                   scale_en;
    uint32_t                   sub_sample_en;
    uint32_t                   sub_sample_factor;
    uint32_t                   sub_sample_mode;	
    uint32_t                   slice_en;
    uint32_t                   h_scale_coeff;
    uint32_t                   v_scale_coeff;    
}ISP_PATH_DESCRIPTION_T;

typedef void (*ISP_ISR_FUNC_PTR) (void *);

typedef enum
{
    ISP_MODE_IDLE = 0,
    ISP_MODE_CAPTURE,
    ISP_MODE_PREVIEW,
    ISP_MODE_PREVIEW_EX,
    ISP_MODE_REVIEW,
    ISP_MODE_SCALE,
    ISP_MODE_MPEG,
    ISP_MODE_VT,
    ISP_MODE_VT_REVIEW,
    ISP_MODE_MAX
} ISP_MODE_E;
enum
{

    ISP_IRQ_SENSOR_SOF = 0,
    ISP_IRQ_SENSOR_EOF,		
    ISP_IRQ_CAP_SOF,
    ISP_IRQ_CAP_EOF,    
    ISP_IRQ_PATH1_DONE, 
    ISP_IRQ_CAP_FIFO_OF,    
    ISP_IRQ_SENSOR_LINE_ERR,		
    ISP_IRQ_SENSOR_FRAME_ERR,		
    ISP_IRQ_JPEG_BUF_OF,		
    ISP_IRQ_PATH2_DONE,    
    ISP_IRQ_NUMBER
};

typedef struct _isp_cap_desc_tag
{
    uint32_t                   input_format;
    uint32_t                   frame_deci_factor;
    uint32_t                   img_x_deci_factor;
    uint32_t                   img_y_deci_factor;
}ISP_CAP_DESCRIPTION_T;
typedef struct _isp_module_tagss
{
    ISP_MODE_E               isp_mode;
    uint32_t                   module_addr;
    ISP_CAP_DESCRIPTION_T    isp_cap;
    ISP_PATH_DESCRIPTION_T   isp_path1;
    ISP_PATH_DESCRIPTION_T   isp_path2;
    ISP_ISR_FUNC_PTR         user_func[ISP_IRQ_NUMBER];
  
}ISP_MODULE_T;
typedef enum
{
    ISP_IRQ_NOTICE_SENSOR_SOF = 0,
    ISP_IRQ_NOTICE_SENSOR_EOF,
    ISP_IRQ_NOTICE_CAP_SOF,
    ISP_IRQ_NOTICE_CAP_EOF,
    ISP_IRQ_NOTICE_PATH1_DONE,
    ISP_IRQ_NOTICE_CAP_FIFO_OF,
    ISP_IRQ_NOTICE_SENSOR_LINE_ERR,
    ISP_IRQ_NOTICE_SENSOR_FRAME_ERR,
    ISP_IRQ_NOTICE_JPEG_BUF_OF,
    ISP_IRQ_NOTICE_PATH2_DONE,
    ISP_IRQ_NOTICE_NUMBER,
} ISP_IRQ_NOTICE_ID_E;
enum
{
    ISP_FRAME_UNLOCK = 0,
    ISP_FRAME_LOCK_WRITE = 0x10011001,
    ISP_FRAME_LOCK_READ = 0x01100110
};
enum 
{
    ISP_MASTER_READ,
    ISP_MASTER_WRITE,
    ISP_MASTER_MAX    
};
enum 
{
    ISP_MASTER_ENDIANNESS_BIG,
    ISP_MASTER_ENDIANNESS_LITTLE,
    ISP_MASTER_ENDIANNESS_HALFBIG,
    ISP_MASTER_ENDIANNESS_MAX        
};
typedef enum
{
    ISP_DRV_RTN_SUCCESS = 0,
    ISP_DRV_RTN_PARA_ERR = 0x10,
    ISP_DRV_RTN_IO_ID_UNSUPPORTED,
    ISP_DRV_RTN_ISR_NOTICE_ID_ERR,
    ISP_DRV_RTN_MASTER_SEL_ERR,
    ISP_DRV_RTN_MODE_ERR,

    ISP_DRV_RTN_CAP_FRAME_SEL_ERR = 0x20,
    ISP_DRV_RTN_CAP_INPUT_FORMAT_ERR,
    ISP_DRV_RTN_CAP_INPUT_YUV_ERR,
    ISP_DRV_RTN_CAP_SYNC_POL_ERR,
    ISP_DRV_RTN_CAP_FIFO_DATA_RATE_ERR,
    ISP_DRV_RTN_CAP_SKIP_FRAME_TOO_MANY,
    ISP_DRV_RTN_CAP_FRAME_DECI_FACTOR_ERR,
    ISP_DRV_RTN_CAP_XY_DECI_FACTOR_ERR,
    ISP_DRV_RTN_CAP_FRAME_SIZE_ERR,
    ISP_DRV_RTN_CAP_JPEG_DROP_NUM_ERR,

    ISP_DRV_RTN_PATH_SRC_SIZE_ERR = 0x30,
    ISP_DRV_RTN_PATH_TRIM_SIZE_ERR,
    ISP_DRV_RTN_PATH_DES_SIZE_ERR,
    ISP_DRV_RTN_PATH_INPUT_FORMAT_ERR,
    ISP_DRV_RTN_PATH_OUTPUT_FORMAT_ERR,
    ISP_DRV_RTN_PATH_SC_COEFF_ERR,
    ISP_DRV_RTN_PATH_SUB_SAMPLE_ERR,
    ISP_DRV_RTN_PATH_ADDR_INVALIDE,
    ISP_DRV_RTN_PATH_FRAME_TOO_MANY,
    ISP_DRV_RTN_PATH_FRAME_LOCKED,    
    ISP_DRV_RTN_MAX
} ISP_DRV_RTN_E;

typedef enum
{   
    ISP_PATH_INPUT_FORMAT = 0,
    ISP_PATH_INPUT_SIZE,
    ISP_PATH_INPUT_RECT,
    ISP_PATH_INPUT_ADDR,
    ISP_PATH_OUTPUT_SIZE,
    ISP_PATH_OUTPUT_FORMAT,
    ISP_PATH_OUTPUT_ADDR,
    ISP_PATH_OUTPUT_FRAME_FLAG,
    ISP_PATH_SWAP_BUFF,
    ISP_PATH_LINE_BUFF,
    ISP_PATH_SUB_SAMPLE_EN,
    ISP_PATH_SUB_SAMPLE_MOD,
    ISP_PATH_SLICE_SCALE_EN,
    ISP_PATH_SLICE_SCALE_HEIGHT,
    ISP_PATH_DITHER_EN,
    ISP_PATH_IS_IN_SCALE_RANGE,
    ISP_PATH_IS_SCALE_EN,
    ISP_PATH_SLICE_OUT_HEIGHT,
    ISP_PATH_MODE,
    ISP_CFG_ID_E_MAX
} ISP_CFG_ID_E;

#define ISP_CLK_DOMAIN_AHB                             1
#define ISP_CLK_DOMAIN_DCAM                            0


#endif //_SCALE_REG_SC8800G2_H_
