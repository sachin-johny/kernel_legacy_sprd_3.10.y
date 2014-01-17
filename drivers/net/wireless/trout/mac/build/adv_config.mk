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
#   File Name         : adv_config.mk                                         #
#                                                                             #
#   Description       : This file contains the default definitions for the    #
#                       project that should not be modified by the user. In   #
#                       addition it provides definitions that can be set by   #
#                       the user to configure advanced system parameters.     #
#                       This file is included in the makefile. Note that this #
#                       must be included after config.mk. Any relevant        #
#                       dependencies of the compile-time flags are also       #
#                       resolved in this file.                                #
#                                                                             #
#   Issues            : None                                                  #
#                                                                             #
###############################################################################

# Set host type flag                                                          #
HOST_FLAGS          += 

# Set the PHY Edge flags. Based on this the MAC source is compiled to         #
# operate with various PHY.                                                   #
# SET 1: PHY Protocol Flags (Must Select One)                                 #
# -DPHY_802_11n        : Compile MAC for 802.11n                              #
# SET 2: PHY Hardware Flags (May Select One)                                  #
# SET 3: Miscellaneous Flags (May Select One or More)                         #
# -DLEFT_ANTENNA_SELECTED : Left Antenna is selected (For Ittiam PHY only)    #
# -DAFE_AD9861            : Compile AD9861 support for 20MHz Ittiam PHY       #
EDGE_FLAGS          += -DPHY_802_11n 				    # (Default)
#EDGE_FLAGS          += -DLEFT_ANTENNA_SELECTED    ##hugh #                     

# Set the MAC Mode flags. Based on this the MAC source is compiled to         #
# operate with various MAC Modes (AP/STA)                                     #
MAC_MODE_FLAGS      +=

# Set the MAC Edge flags. Based on this the MAC source is compiled to         #
# operate with various PHY and MAC H/W. (Must Select One or More)             #
# -DMAC_802_11I      : Enable 802.11i protocol                                #
# -DIBSS_11I         : Enable 802.11i for IBSS mode                           #
# -DMAC_WMM          : Enable WMM (802.11e) protocol                          #
# -DMAC_802_11N      : Enable 802.11n protocol                                #
# -DMAC_802_1X       : Enable 802.1x protocol                                 #
# -DMAC_802_11H      : Enable 802.11H protocol                                #
# -DMAC_MULTIDOMAIN  : Enable Multi-domain support                            #
MAC_EDGE_FLAGS += -DMAC_802_11I -DMAC_WMM -DMAC_802_11N             # (Default)

# 20120830 caisf mod, merged ittiam mac v1.3 code
# Begin:modified by wulei 2791 for bug 160423 on 2013-05-04
 MAC_EDGE_FLAGS += -DMAC_802_1X                        # (Default)
# MAC_EDGE_FLAGS += -DMAC_802_1X -DMAC_802_11H                        # (Default)
# End:modified by wulei 2791 for bug 160423 on 2013-05-04
#MAC_EDGE_FLAGS += -DMAC_802_1X -DMAC_802_11H -DMAC_MULTIDOMAIN      # (Default)

# Set the RF flags. Based on this MAC source is compiled to operate with      #
# various RF provided by different vendors.                                   #
RF_FLAGS       +=

# Set the Processor flags. Based on this MAC source is compiled to operate    #
# with various processors (Must Select One)                                   #
# -DMWLAN : MIMO WLAN (MWLAN) Platform                                        #
PROC_FLAGS          = -DMWLAN                                       # (Default)

# Set the Chip support library/OS Flags (Must Select One)                     #
# -DOS_LINUX_CSL_TYPE : Linux CSL/OS Type                                     #
CSL_FLAGS           = -DOS_LINUX_CSL_TYPE                           # (Default)

# Set any Extra Compile time flags (May Select One or More)                   #
# -DAUTORATE_FEATURE    : Enable S/w Auto rate feature                        #
# -DNO_ACTION_RESET     : Action Requests shall be restored after reset       #
# -DEDCA_DEMO_KLUDGE    : Enable kludge for EDCA demo based on port numbers   #
# -DIBSS_2040_2G4_KLUDGE: Enable 40Mhz operation in 2.4GHz band in IBSS mode  #
#                       : (standard incompliant feature)                      #
# -DTEST_2040_MACSW_KLUDGE: Enable support in MAC S/w to test 20/40 MAC S/w   #
#                           with 20MHz H/w                                    #
# -DTX_ABORT_FEATURE    : Enable Tx Abort Feature                             #

# Extra Compile Time flags related to Memory Management                       #
# -DMAX_SHARED_MEMORY_UTIL: Enables higher number of BA sessions              #
# -DPHY_TEST_MAX_PKT_RX : Enable reception of 4096B Pkts in HUT Mode          #
# -DLOCALMEM_TX_DSCR    : Enable manipulation of Tx-Descriptors in local      #
#                       : memory (Expt: S/w optimization)                     #

# Extra Compile Time flags for MAC H/w                                        #
# -DENABLE_MACHW_KLUDGE : Enable S/w Kludge for DS2 MAC H/w                   #
# -DDISABLE_MACHW_DEFRAG: Disable (H/w) Defragmentation feature               #
# -DDISABLE_MACHW_DEAGGR: Disable (H/w) AMSDU Deaggregation feature           #
# -DDISABLE_CE_CLKGATING: Disable MAC H/w CE Clock-Gating                     #

# Extra Compile Time flags for MWLAN                                          #
# -DUSE_PROCESSOR_DMA   : Enable use of Processor DMA for copying buffer      #
# -DMAC_ADDRESS_FROM_FLASH : Read STA MAC Address from Flash                  #
# TROUT_WIFI_NPI DO NOT USE AUTO RATE!
#ifeq (, $(findstring DTROUT_WIFI_NPI, $(EXTRA_OPTION_FLAGS)))
#	EXTRA_OPTION_FLAGS  += -DAUTORATE_FEATURE
#endif
##zhuyg add for npi in 2013-05-28
ifneq (npi, $(MAC_MODE))
EXTRA_OPTION_FLAGS  += -DAUTORATE_FEATURE
endif

EXTRA_OPTION_FLAGS  += -DNO_ACTION_RESET 
#EXTRA_OPTION_FLAGS  += -DENABLE_MACHW_KLUDGE -DUSE_PROCESSOR_DMA -DEDCA_DEMO_KLUDGE  #hugh
EXTRA_OPTION_FLAGS  += -DENABLE_MACHW_KLUDGE -DEDCA_DEMO_KLUDGE 

#EXTRA_OPTION_FLAGS  += -DMAC_ADDRESS_FROM_FLASH -DMAX_SHARED_MEMORY_UTIL #hugh
EXTRA_OPTION_FLAGS  += -DMAX_SHARED_MEMORY_UTIL

# it define the new version of CHIP 2E
#EXTRA_OPTION_FLAGS  += -DTROUT2_CHIP_VER_V2
EXTRA_OPTION_FLAGS  += -DTROUT2_CHIP_VER_V3
###############################################################################
# Resolve Dependencies                                                        #
###############################################################################

# Debug mode for trace switch(add by chengwg)
ifneq (, $(findstring DDEBUG_MODE, $(EXTRA_OPTION_FLAGS)))	#TROUT_DEBUG
	EXTRA_OPTION_FLAGS += -DTROUT_TRACE_DBG		#switch for trace func
endif

# Resolve dependencies for Ittiam PHY
ifneq (,$(findstring DITTIAM_PHY,$(EDGE_FLAGS)))            #ITTIAM_PHY
                                                            #ITTIAM_PHY
# 40MHz support in Ittiam PHY                               #ITTIAM_PHY
ifneq (,$(findstring DENABLE_PHY_40MHZ,$(EDGE_FLAGS)))      #ITTIAM_PHY
endif                                                       #ITTIAM_PHY
endif                                                       #ITTIAM_PHY


# Resolve dependencies for P2P feature                      #MAC_P2P
ifneq (,$(findstring DMAC_P2P,$(MAC_EDGE_FLAGS)))           #MAC_P2P
# caisf merge ittiam v1.2 code
#        MAC_EDGE_FLAGS += -DINT_WPS_SUPP -DP2P_US           #MAC_P2P
        MAC_EDGE_FLAGS += -DINT_WPS_SUPP  -DP2P_NO_COUNTRY  #MAC_P2P
        MAC_EDGE_FLAGS += -DMAC_P2P_HW_SUPP                 #MAC_P2P
endif                                                       #MAC_P2P

# Resolve dependencies for WPS feature                           #INT_WPS_SUPP
ifneq (,$(findstring DINT_WPS_SUPP,$(MAC_EDGE_FLAGS)))           #INT_WPS_SUPP
                                                                 #INT_WPS_SUPP
# Include Internal WPS Registrar Support in AP Mode              #INT_WPS_SUPP
ifneq (,$(findstring DBSS_ACCESS_POINT_MODE,$(MAC_MODE_FLAGS)))  #INT_WPS_SUPP
        MAC_EDGE_FLAGS += -DINT_WPS_REG_SUPP                     #INT_WPS_SUPP
endif                                                            #INT_WPS_SUPP
                                                                 #INT_WPS_SUPP
# Include Internal WPS Enrollee Support in STA Mode              #INT_WPS_SUPP
ifneq (,$(findstring DIBSS_BSS_STATION_MODE,$(MAC_MODE_FLAGS)))  #INT_WPS_SUPP
        MAC_EDGE_FLAGS += -DINT_WPS_ENR_SUPP                     #INT_WPS_SUPP
endif                                                            #INT_WPS_SUPP
endif                                                            #INT_WPS_SUPP
                                                               
# Resolve dependencies for Maxim RF
ifneq (,$(findstring DRF_MAXIM,$(RF_FLAGS)))

# Include Maxim RF support for Ittiam PHY                   #ITTIAM_PHY                   
ifneq (,$(findstring DITTIAM_PHY,$(EDGE_FLAGS)))            #ITTIAM_PHY
        RF_FLAGS += -DRF_MAXIM_ITTIAM                       #ITTIAM_PHY
endif                                                       #ITTIAM_PHY

endif

# Resolve dependencies for Airoha RF
ifneq (,$(findstring DRF_AIROHA,$(RF_FLAGS)))

# Include Airoha RF support for Ittiam PHY                  #ITTIAM_PHY
ifneq (,$(findstring DITTIAM_PHY,$(EDGE_FLAGS)))            #ITTIAM_PHY
# caisf merge ittiam v1.2 code
#        RF_FLAGS += -DRF_AIROHA_ITTIAM                     #ITTIAM_PHY
endif                                                       #ITTIAM_PHY

endif


# Include dependencies for HUT Mode                            #MAC_HW_UNIT_TEST_MODE
ifneq (,$(findstring MAC_HW_UNIT_TEST_MODE,$(MAC_MODE_FLAGS))) #MAC_HW_UNIT_TEST_MODE
        MAC_EDGE_FLAGS +=  -DSUPP_11I 			       #MAC_HW_UNIT_TEST_MODE
endif 							       #MAC_HW_UNIT_TEST_MODE

###############################################################################
# Set all the include directory paths as required by the MAC source files.    #
# The makefile uses these paths to inlcude the header files.                  #
# Set all the include directory paths as required by the MAC source files.    #
# The makefile uses these paths to inlcude the header files.                  #
###############################################################################
COMMON_INCS         = $(MAC_SRC_DIR)/Common
CON_INCS            = $(MAC_SRC_DIR)/Controller/AP-STA
CONSTA_INCS         = $(MAC_SRC_DIR)/Controller/STA
CONSTAPROT_INCS     = $(MAC_SRC_DIR)/Controller/STA/ProtIF
CONSTAP2P_INCS      = $(MAC_SRC_DIR)/Controller/STA/P2P
CONIF_INCS          = $(MAC_SRC_DIR)/Controller/ModeIF
CONAP_INCS          = $(MAC_SRC_DIR)/Controller/AP
CORE_INCS           = $(MAC_SRC_DIR)/Core/AP-STA
APSTAPROTIF_INCS    = $(MAC_SRC_DIR)/Core/AP-STA/ProtIF
APSTAM11E_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/M802_11e
APSTAM11I_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/M802_11i
APSTAM11H_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/M802_11h
APSTAM11N_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/M802_11n
APSTAMDOM_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/MDOM # 20120830 caisf mod, merged ittiam mac v1.3 code
APSTAMP2P_INCS      = $(MAC_SRC_DIR)/Core/AP-STA/P2P
CORESTA_INCS        = $(MAC_SRC_DIR)/Core/STA
STAPROTIF_INCS      = $(MAC_SRC_DIR)/Core/STA/ProtIF
STAM11E_INCS        = $(MAC_SRC_DIR)/Core/STA/M802_11e
STAM11I_INCS        = $(MAC_SRC_DIR)/Core/STA/M802_11i
STAM11H_INCS        = $(MAC_SRC_DIR)/Core/STA/M802_11h
STAM11N_INCS        = $(MAC_SRC_DIR)/Core/STA/M802_11n
STAMDOM_INCS        = $(MAC_SRC_DIR)/Core/STA/MDOM # 20120830 caisf mod, merged ittiam mac v1.3 code
STAMWPS_INCS        = $(MAC_SRC_DIR)/Core/STA/WPS
STAMP2P_INCS        = $(MAC_SRC_DIR)/Core/STA/P2P
COREAP_INCS         = $(MAC_SRC_DIR)/Core/AP
APPROTIF_INCS       = $(MAC_SRC_DIR)/Core/AP/ProtIF
APM11E_INCS         = $(MAC_SRC_DIR)/Core/AP/M802_11e
APM11I_INCS         = $(MAC_SRC_DIR)/Core/AP/M802_11i
APM11N_INCS         = $(MAC_SRC_DIR)/Core/AP/M802_11n
APM1X_INCS          = $(MAC_SRC_DIR)/Core/AP/M802_1x
APM11H_INCS         = $(MAC_SRC_DIR)/Core/AP/M802_11h
APMDOM_INCS         = $(MAC_SRC_DIR)/Core/AP/MDOM # 20120830 caisf mod, merged ittiam mac v1.3 code
APMWPS_INCS         = $(MAC_SRC_DIR)/Core/AP/WPS
APMP2P_INCS         = $(MAC_SRC_DIR)/Core/AP/P2P
MODEIF_INCS         = $(MAC_SRC_DIR)/Core/ModeIF
EDGEIF_INCS         = $(MAC_SRC_DIR)/Edge/IF
EDGEPROC_PPC_INCS   = $(MAC_SRC_DIR)/Edge/Processor/MPC8349
EDGEHOSTSDIO_INCS   = $(MAC_SRC_DIR)/Edge/Host/SDIO
EDGEHOSTUART_INCS   = $(MAC_SRC_DIR)/Edge/Host/UART
EDGEMACHW_INCS      = $(MAC_SRC_DIR)/Edge/MACHW
EDGERF_ITTMAX_INCS  = $(MAC_SRC_DIR)/Edge/RF/MAXIM_ITTIAM
EDGERF_ITTAIR_INCS  = $(MAC_SRC_DIR)/Edge/RF/AIROHA_ITTIAM
EDGERF_GENERIC_INCS = $(MAC_SRC_DIR)/Edge/RF/GENERIC_RF
EDGEPHY_INCS_ITTIAM = $(MAC_SRC_DIR)/Edge/PHY/PHYHW/ITTIAM
EDGEPHY_INCS_11g    = $(MAC_SRC_DIR)/Edge/PHY/PHYProt/P802_11g
EDGEPHY_INCS_11a    = $(MAC_SRC_DIR)/Edge/PHY/PHYProt/P802_11a
EDGEPHY_INCS_11b    = $(MAC_SRC_DIR)/Edge/PHY/PHYProt/P802_11b
EDGEPHY_INCS_11n    = $(MAC_SRC_DIR)/Edge/PHY/PHYProt/P802_11n
MM_INCS             = $(MAC_SRC_DIR)/Utils/MM
QM_INCS             = $(MAC_SRC_DIR)/Utils/QM
UTILS_INCS	    = $(MAC_SRC_DIR)/Utils/
WPSE_INCS           = $(MAC_SRC_DIR)/Library/WPS/Enrollee/
WPSR_COMMON_INCS    = $(MAC_SRC_DIR)/Library/WPS/Common
WPSR_PROTOCOL_INCS  = $(MAC_SRC_DIR)/Library/WPS/Protocols
WPSR_REGISTRAR_INCS = $(MAC_SRC_DIR)/Library/WPS/Registrar
WPSR_IF_INCS        = $(MAC_SRC_DIR)/Library/WPS/IF
CSL_INCS            = $(MAC_SRC_DIR)/CSL
MSTEST_INCS         = $(MAC_SRC_DIR)/Test/MacSwTest
MHTEST_INCS         = $(MAC_SRC_DIR)/Test/MacHwTest
TRW_INCS	    = $(MAC_SRC_DIR)/Test/trw
RUNMODE_INCS	    = $(MAC_SRC_DIR)/Test/runmode
METER_INCS	    = $(MAC_SRC_DIR)/Test/meter

OS_LIBRARY   = target.ld
###############################################################################
