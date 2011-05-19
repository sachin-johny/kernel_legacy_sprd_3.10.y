#!/bin/bash

#
#***************************************************************
#
# Component = Build & Configuration scripts
#             Trusted Linux
#
# Contributor(s):
#
#***************************************************************
#

#----------------------------------------------------------------------------
# Configure the Installation and Build Directories
#----------------------------------------------------------------------------

# Source Directory
INSTALL_DIR=`pwd`
BUILD_DIR=${INSTALL_DIR}

#----------------------------------------------------------------------------
# Location of external components
#----------------------------------------------------------------------------

ANDROID_DIR=${INSTALL_DIR}/..
ANDROID_ARM_TOOLCHAIN=${ANDROID_DIR}/prebuilt/linux-x86/toolchain/arm-eabi-4.3.1
LINUX_SRC=${INSTALL_DIR}
LINUX_DEF_CONFIG=${LINUX_SRC}/arch/arm/configs/sc8800g-trusted-abs-android_defconfig
CMDLINE_FILE=${ANDROID_DIR}/vendor/sprd/hsdroid/AndroidBoard.mk
MKBOOTIMG=${ANDROID_DIR}/out/host/linux-x86/bin/mkbootimg
BOOTIMG=${ANDROID_DIR}/out/target/product/hsdroid/boot.img
INITRD_DIR=${ANDROID_DIR}/out/target/product/hsdroid
INITRD_IMG=${INITRD_DIR}/ramdisk.img

if [ ! -f "${LINUX_DEF_CONFIG}" ]; then
    echo Invalid configuration
    exit 1
fi

if [ ! -f "${MKBOOTIMG}" ]; then
    echo "Missing host tool: ${MKBOOTIMG}"
    echo "Please build your android project first"
    exit 1
fi

if [ ! -d ${INITRD_DIR} ]
then
    echo "Missing component Linux initrd: ${INITRD_DIR}"
    exit -1
fi
if [ ! -f ${INITRD_IMG} ]
then
    echo "Missing component Linux initrd image: ${INITRD_IMG}"
    exit 1
fi
#----------------------------------------------------------------------------
# Build related variables
#----------------------------------------------------------------------------

BUILD_LINUX_DIR=${BUILD_DIR}
LINUX_CONFIG=${BUILD_LINUX_DIR}/.config
CMDLINE=`cat ${CMDLINE_FILE}  |sed -n '/BOARD_KERNEL_CMDLINE/p' |cut -c24-`
KERNELIMG=${BUILD_LINUX_DIR}/arch/arm/boot/Image

#----------------------------------------------------------------------------
# Linux kernel build
#----------------------------------------------------------------------------

echo
echo "Building Linux kernel"
echo

mkdir -p ${BUILD_LINUX_DIR}

cd ${LINUX_SRC}
if [ ! -f ${LINUX_CONFIG} -o ${LINUX_DEF_CONFIG} -nt ${LINUX_CONFIG} ]
then
    echo "  with default config:"
    echo "     ${LINUX_DEF_CONFIG}"
    echo
    cp ${LINUX_DEF_CONFIG} ${LINUX_CONFIG}
    make ARCH=arm CROSS_COMPILE=${ANDROID_ARM_TOOLCHAIN}/bin/arm-eabi- sc8800g-trusted-abs-android_defconfig
else
    echo "  with current config"
    echo
fi
#make -j2 -f Makefile ARCH=arm EXTRAVERSION=-sprd-redbend CROSS_COMPILE=${ANDROID_ARM_TOOLCHAIN}/bin/arm-eabi- CONFIG_DEBUG_SECTION_MISMATCH=y O=${BUILD_LINUX_DIR} Image
make -j2 -f Makefile ARCH=arm EXTRAVERSION=-sprd-redbend CROSS_COMPILE=${ANDROID_ARM_TOOLCHAIN}/bin/arm-eabi- CONFIG_DEBUG_SECTION_MISMATCH=y Image

if [ $? -ne 0 ]; then
    echo "*** LINUX KERNEL BUILD FAILED ***"
    exit 1
fi


#rm -f ${BUILD_DIR}/linux.bin

#cp ${BUILD_LINUX_DIR}/arch/arm/boot/Image ${BUILD_DIR}/linux.bin || exit 1
#dd conv=notrunc if=${INITRD_IMG} obs=32k seek=$(( (0x700000 / 0x8000) - 1 )) of=${BUILD_DIR}/linux.bin || exit 1

#echo "Linux image : ${BUILD_DIR}/linux.bin ready"
#INITRD_SIZE=`ls -l ${INITRD_IMG} | cut -d' ' -f5` 
#echo "RAMDISK Size: ${INITRD_SIZE}"
#exit 0;

echo
echo "Building boot.img"
echo
echo "  CMDLINE: ${CMDLINE}"
${MKBOOTIMG} --kernel ${BUILD_LINUX_DIR}/arch/arm/boot/Image --ramdisk ${INITRD_IMG} -o ${BOOTIMG} --cmdline "${CMDLINE}"

if [ $? -ne 0 ]; then
    echo "*** MKBOOTIMG FAILED ***"
    exit 1
fi
echo "  BOOTIMG: ${BOOTIMG} is ready"
