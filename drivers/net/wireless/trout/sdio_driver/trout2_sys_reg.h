 /*****************************************************************************
 ** File Name:  	TroutII_Sys_Reg_config.h                                 *
 ** Author:         	Tiantian Yu                                          *
 ** DATE:           	07/04/2013                                           *
 ** Description:    	TroutII System register byte address define          *
 *****************************************************************************/

#ifndef TROUTII_SYS_REG_DEFS_H  
#define TROUTII_SYS_REG_DEFS_H  

#define   TROUT2_SYS_HOST_BASE             0x00000000             



#define   TROUT2_SYS_REG_IISDO                  (TROUT2_SYS_HOST_BASE+(0x00<<2))
#define   TROUT2_SYS_REG_IISCLK                 (TROUT2_SYS_HOST_BASE+(0x01<<2))
#define   TROUT2_SYS_REG_IISLRCK                (TROUT2_SYS_HOST_BASE+(0x02<<2))
#define   TROUT2_SYS_REG_IISDI                  (TROUT2_SYS_HOST_BASE+(0X13<<2))
                                         
#define   TROUT2_SYS_REG_PTEST                  (TROUT2_SYS_HOST_BASE+(0X03<<2))
#define   TROUT2_SYS_REG_GINT                   (TROUT2_SYS_HOST_BASE+(0X04<<2))
#define   TROUT2_SYS_REG_XTLEN                  (TROUT2_SYS_HOST_BASE+(0X05<<2))
#define   TROUT2_SYS_REG_RESETN                 (TROUT2_SYS_HOST_BASE+(0X06<<2))
#define   TROUT2_SYS_REG_GPIO                   (TROUT2_SYS_HOST_BASE+(0X07<<2))


#define   TROUT2_SYS_U0CTS_REG                  (TROUT2_SYS_HOST_BASE+(0X08<<2))
#define   TROUT2_SYS_U0RTS_REG                  (TROUT2_SYS_HOST_BASE+(0X09<<2))
#define   TROUT2_SYS_U0RXD_REG                  (TROUT2_SYS_HOST_BASE+(0X0A<<2))
#define   TROUT2_SYS_U0TXD_REG                  (TROUT2_SYS_HOST_BASE+(0X0B<<2))


#define  TROUT2_SYS_REG_CLK_32K                 (TROUT2_SYS_HOST_BASE+(0X0C<<2))


#define  TROUT2_SYS_REG_SD_CLK                  (TROUT2_SYS_HOST_BASE+(0X0D<<2))
#define  TROUT2_SYS_REG_SD_CMD                  (TROUT2_SYS_HOST_BASE+(0X0E<<2))
#define  TROUT2_SYS_REG_SD_D0                   (TROUT2_SYS_HOST_BASE+(0X0F<<2))
#define  TROUT2_SYS_REG_SD_D1                   (TROUT2_SYS_HOST_BASE+(0X10<<2))
#define  TROUT2_SYS_REG_SD_D2                   (TROUT2_SYS_HOST_BASE+(0X11<<2))
#define  TROUT2_SYS_REG_SD_D3                   (TROUT2_SYS_HOST_BASE+(0X12<<2))
                                         
                                         
#define  TROUT2_SYS_REG_FUNC_LOW_SPEED          (TROUT2_SYS_HOST_BASE+(0X40<<2))
#define  TROUT2_SYS_REG_CLK_CTRL0               (TROUT2_SYS_HOST_BASE+(0X43<<2))
#define  TROUT2_SYS_REG_CLK_CTRL1               (TROUT2_SYS_HOST_BASE+(0X44<<2))
#define  TROUT2_SYS_REG_POW_CTRL                (TROUT2_SYS_HOST_BASE+(0X45<<2))



#define  TROUT2_SYS_REG_GPIO_IN                 (TROUT2_SYS_HOST_BASE+(0X46<<2))
#define  TROUT2_SYS_REG_GPIO_OUT                (TROUT2_SYS_HOST_BASE+(0X47<<2))
#define  TROUT2_SYS_REG_GPIO_OE                 (TROUT2_SYS_HOST_BASE+(0X48<<2))
#define  TROUT2_SYS_REG_GPIO_INEN               (TROUT2_SYS_HOST_BASE+(0X49<<2))



#define  TROUT2_SYS_REG_TEST_SEL                (TROUT2_SYS_HOST_BASE+(0X4D<<2))
#define  TROUT2_SYS_REG_WIFI_CFG                (TROUT2_SYS_HOST_BASE+(0X4E<<2))
#define  TROUT2_SYS_REG_RF_SEL_REG              (TROUT2_SYS_HOST_BASE+(0X4F<<2))
#define  TROUT2_SYS_REG_ADDA_FORCE              (TROUT2_SYS_HOST_BASE+(0X50<<2))
#define  TROUT2_SYS_REG_WIFI_SOFT_RST           (TROUT2_SYS_HOST_BASE+(0X51<<2))
#define  TROUT2_SYS_REG_BIST_CFG                (TROUT2_SYS_HOST_BASE+(0X52<<2))
#define  TROUT2_SYS_REG_DACCONST_CFG            (TROUT2_SYS_HOST_BASE+(0X53<<2))
#define  TROUT2_SYS_REG_UPSAMPLE_CFG            (TROUT2_SYS_HOST_BASE+(0X54<<2))
#define  TROUT2_SYS_REG_IQ_SWAP                 (TROUT2_SYS_HOST_BASE+(0X55<<2))
#define  TROUT2_SYS_REG_ROM_BIST_RESULT0        (TROUT2_SYS_HOST_BASE+(0X56<<2))
#define  TROUT2_SYS_REG_ROM_BIST_RESULT1        (TROUT2_SYS_HOST_BASE+(0X57<<2))
#define  TROUT2_SYS_REG_ANT_SEL_CFG             (TROUT2_SYS_HOST_BASE+(0X58<<2))
#define  TROUT2_SYS_REG_HOST2ARM_INFO0          (TROUT2_SYS_HOST_BASE+(0X5A<<2))
#define  TROUT2_SYS_REG_HOST2ARM_INFO1          (TROUT2_SYS_HOST_BASE+(0X5B<<2))
#define  TROUT2_SYS_REG_HOST2ARM_INFO2          (TROUT2_SYS_HOST_BASE+(0X5C<<2))
#define  TROUT2_SYS_REG_HOST2ARM_INFO3          (TROUT2_SYS_HOST_BASE+(0X5D<<2))
#define  TROUT2_SYS_REG_INFO0_FROM_ARM          (TROUT2_SYS_HOST_BASE+(0X5E<<2))
#define  TROUT2_SYS_REG_INFO1_FROM_ARM          (TROUT2_SYS_HOST_BASE+(0X5F<<2))
#define  TROUT2_SYS_REG_INFO2_FROM_ARM          (TROUT2_SYS_HOST_BASE+(0X60<<2))
#define  TROUT2_SYS_REG_INFO3_FROM_ARM          (TROUT2_SYS_HOST_BASE+(0X61<<2))

#define  TROUT2_SYS_REG_WIFIAD_CFG              (TROUT2_SYS_HOST_BASE+(0X63<<2))
#define  TROUT2_SYS_REG_BTAD_CFG_0              (TROUT2_SYS_HOST_BASE+(0X65<<2))



#define  TROUT2_SYS_REG_TROUT_FAIL_H_1          (TROUT2_SYS_HOST_BASE+(0X67<<2))
#define  TROUT2_SYS_REG_TROUT_FAIL_H_2          (TROUT2_SYS_HOST_BASE+(0X68<<2))
#define  TROUT2_SYS_REG_TROUT_FAIL_H_3          (TROUT2_SYS_HOST_BASE+(0X69<<2))
#define  TROUT2_SYS_REG_RAMBIST_FAIL_H          (TROUT2_SYS_HOST_BASE+(0X6A<<2))



#define  TROUT2_SYS_REG_BWDAC_CFG_0              (TROUT2_SYS_HOST_BASE+(0X6E<<2))
#define  TROUT2_SYS_REG_BWDAC_CFG_1              (TROUT2_SYS_HOST_BASE+(0X6F<<2))
#define  TROUT2_SYS_REG_BWDAC_CFG_2              (TROUT2_SYS_HOST_BASE+(0X70<<2))
#define  TROUT2_SYS_REG_BWDAC_STS_0              (TROUT2_SYS_HOST_BASE+(0X71<<2))
#define  TROUT2_SYS_REG_BWDAC_STS_1              (TROUT2_SYS_HOST_BASE+(0X72<<2))


#define  TROUT2_SYS_REG_ADDA_TEST_CFG            (TROUT2_SYS_HOST_BASE+(0X0C7<<2))
#define  TROUT2_SYS_REG_FPGA_DEBUG_CFG           (TROUT2_SYS_HOST_BASE+(0X0C8<<2))
#define  TROUT2_SYS_REG_HOST_WAKE_UP             (TROUT2_SYS_HOST_BASE+(0X1FF<<2))

#endif //TROUTII_SYS_REG_DEFS_H


   
