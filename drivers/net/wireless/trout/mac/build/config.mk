###############################################################################
#                                                                             #
#                      Ittiam 802.11n MAC SOFTWARE                            #
#                                                                             #
#                   ITTIAM SYSTEMS PVT LTD, BANGALORE                         #
#                          COPYRIGHT(C) 2012                                  #
#                                                                             #
#   This program is proprietary to Ittiam Systems Pvt. Ltd. and is protected  #
#   under Indian Copyright Law as an unpublished work. Its use and            #
#   disclosure is limited by the terms and conditions of a license agreement. #
#   It may not be copied or otherwise reproduced or disclosed to persons      #
#   outside the licensee's organization except in accordance with the terms   #
#   and conditions of such an agreement. All copies and reproductions shall   #
#   be the property of Ittiam Systems India Pvt. Ltd. and must bear this      #
#   notice in its entirety.                                                   #
#                                                                             #
#   File Name         : config.mk                                             #
#                                                                             #
#   Description       : This file contains the definitions to be set by the   #
#                       user to configure the system for various parameters.  #
#                       This file is included in the makefile.                #
#                                                                             #
#   Issues            : None                                                  #
#                                                                             #
###############################################################################

# Set host type flag (Must Select One or More)                                #
# -DETHERNET_HOST       : Ethernet host driver                                #
HOST_FLAGS          = -DETHERNET_HOST

# Set the PHY Edge flags. Based on this the MAC source is compiled to         #
# operate with various PHY.                                                   #
# Set 1: PHY Hardware Flags (Must Select One)                                 #
# -DITTIAM_PHY	       : Compile MAC for Ittiam PHY hardware                  #
# Set 2: PHY Feature Flags  (May Select One or More)                          #

EDGE_FLAGS        = -DITTIAM_PHY 

# Set the MAC Mode flags. Based on this the MAC source is compiled to         #
# operate with various MAC Modes (AP/STA) (Must Select One)                   #
# -DBSS_ACCESS_POINT_MODE : Access Point operating mode                       #
# -DIBSS_BSS_STATION_MODE : Station (BSS/IBSS) operating mode                 #
# -DMAC_HW_UNIT_TEST_MODE : Hardware Unit Test mode                           #
# Hugh: MAC_MODE is defined when calling 'make' command.
ifneq (ap, $(MAC_MODE))
#$(warning  "sta mode!")
MAC_MODE_FLAGS      = -DIBSS_BSS_STATION_MODE
else
#$(warning  "ap mode!")
MAC_MODE_FLAGS      = -DBSS_ACCESS_POINT_MODE
endif

##zhuyg add for troutII npi test
ifeq (npi, $(MAC_MODE))
MAC_MODE_FLAGS      = -DIBSS_BSS_STATION_MODE
endif

# Set the MAC Edge flags. Based on this the MAC source is compiled to         #
# operate with various PHY and MAC H/W. (May Select One or More)              #
# -DSUPP_11I         : Enable Internal 802.11i Supplicant                     #
# -DINT_WPS_SUPP     : Enable Internal WPS Feature (For AP & STA Modes Only)  #
# -DMAC_P2P          : Enable Wi-Fi Direct Feature (For AP & STA Modes Only)  #
# caisf merge ittiam v1.2 code
MAC_EDGE_FLAGS = -DSUPP_11I
#MAC_EDGE_FLAGS = -DSUPP_11I -DINT_WPS_SUPP -DMAC_P2P 
#MAC_EDGE_FLAGS = -DSUPP_11I -DINT_WPS_SUPP		#modify by chengw to mask p2p.

# Set the RF flags. Based on this MAC source is compiled to operate with      #
# various RF provided by different vendors. The possible values are,          #
# Set 1: RF Vendor Flags (Must Select One)                                    #
# -DRF_MAXIM         : Compile for MAXIM RF                                   #
# -DRF_AIROHA        : Compile for AIROHA RF                                  #
# Set 2: RF Model Flags (Must Select One)                                     #
# -DMAX2829          : Compile for MAX2829 RF with Ittiam PHY                 #
RF_FLAGS             = -DRF_MAXIM -DMAX2829

# Set any Extra Compile time flags (May Select One or More)                   #
# -DDEBUG_MODE          : Enable Generation of Extra Debug information        #
# -DDEFAULT_SME         : Enable default SME configuration                    #
# -DCONFIG_NUM_STA      : Set the number of stations supported                #
# -DTROUT_SDIO_INTERFACE : use sdio interface(if not define,use spi interface)#
EXTRA_OPTION_FLAGS   = -DDEBUG_MODE
#EXTRA_OPTION_FLAGS  += -DDEFAULT_SME #masked by caisf
EXTRA_OPTION_FLAGS  += -DCONFIG_NUM_STA='8'

#chenq add, in sta mode,scan 20 ap info
EXTRA_OPTION_FLAGS  += -DCONFIG_NUM_AP_SCAN='20'

EXTRA_OPTION_FLAGS  += -DTROUT_SDIO_INTERFACE
#EXTRA_OPTION_FLAGS  += -DTX_PKT_USE_DMA			##chengwg: tx cpy to trout using dma or not switch.
#EXTRA_OPTION_FLAGS += -DTROUT_B2B_TEST_MODE       	##Hugh: For board to board TX/RX test.

#caisf add for power sleep feature 2013-01-04
# NOTE: POWER SLEEP FUNCTION CAN ONLY WORK IN BSS-STA MODE #
ifeq (y, $(CONFIG_TROUT_WIFI_POWER_SLEEP_ENABLE))
ifneq (ap, $(MAC_MODE))
#leon liu removed TROUT_WIFI_POWER_SLEEP_ENABLE for now - 2013-5-10
EXTRA_OPTION_FLAGS += -DTROUT_WIFI_POWER_SLEEP_ENABLE
EXTRA_OPTION_FLAGS += -DWIFI_SLEEP_POLICY
#EXTRA_OPTION_FLAGS += -DPOWERSAVE_DEBUG
ifneq (, $(findstring DWIFI_SLEEP_POLICY, $(EXTRA_OPTION_FLAGS)))	#chwg
	EXTRA_OPTION_FLAGS += -DWAKE_LOW_POWER_POLICY
endif
endif
endif
#POWER SAVE FUNCTION
# xy, add new auto rate
ifeq (sta, $(MAC_MODE))
EXTRA_OPTION_FLAGS += -DTROUT_WIFI_AUTO_RATE_NEW_EN
endif

#chenq add for wapi 20120919
ifneq (ap, $(MAC_MODE))

EXTRA_OPTION_FLAGS += -DMAC_WAPI_SUPP
EXTRA_OPTION_FLAGS += -DEXT_SUPP_11i
EXTRA_OPTION_FLAGS += -DCOMBO_SCAN
endif

#chenq add for gpio pin,enable: gpio 142,disable:gpio 136
EXTRA_OPTION_FLAGS += -DUSE_TROUT_PHONE

#chenq add for use external sdio spi if 2012-12-14
#EXTRA_OPTION_FLAGS += -DUSE_INTERNAL_SDIO_SPI_IF

#chenq add for evb|fpga 2012-12-20
#EXTRA_OPTION_FLAGS += -DTROUT_WIFI_FPGA
#EXTRA_OPTION_FLAGS += -DTROUT_WIFI_EVB
#EXTRA_OPTION_FLAGS += -DTROUT_WIFI_EVB -DTROUT_WIFI_EVB_MF
#EXTRA_OPTION_FLAGS += -DTROUT_WIFI_EVB -DTROUT_WIFI_EVB_MF_RF55

##dumy add for Trout2 FPGA RF55&MAX2829
#EXTRA_OPTION_FLAGS  += -DTROUT2_WIFI_FPGA -DTROUT2_WIFI_FPGA_RF55
#EXTRA_OPTION_FLAGS  += -DTROUT2_WIFI_FPGA -DTROUT2_WIFI_FPGA_RF2829

##chenq  add for  Trout2 IC RF55
EXTRA_OPTION_FLAGS  += -DTROUT2_WIFI_IC

##zhuyg add for troutII npi test
ifeq (npi, $(MAC_MODE))
EXTRA_OPTION_FLAGS += -DTROUT_WIFI_NPI
endif

##chenq add macro NMAC_1X1_MODE 2013-09-16
EXTRA_OPTION_FLAGS  += -DNMAC_1X1_MODE
EXTRA_OPTION_FLAGS += -DTROUT_DEBUG_RW_PROC
#EXTRA_OPTION_FLAGS  += -DTROUT_RUNMODE_PROC
#EXTRA_OPTION_FLAGS  += -DTROUT_METER
