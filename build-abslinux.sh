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

if [ ! -f "${LINUX_DEF_CONFIG}" ]; then
    echo Invalid configuration
    exit 1
fi

#----------------------------------------------------------------------------
# Build related variables
#----------------------------------------------------------------------------

BUILD_LINUX_DIR=${BUILD_DIR}
LINUX_CONFIG=${BUILD_LINUX_DIR}/.config

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

export INITRD_DIR=${ANDROID_DIR}/out/target/product/hsdroid/
export INITRD_IMG=${INITRD_DIR}/ramdisk.img
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

rm -f ${BUILD_DIR}/linux.bin

cp ${BUILD_LINUX_DIR}/arch/arm/boot/Image ${BUILD_DIR}/linux.bin || exit 1
dd conv=notrunc if=${INITRD_IMG} obs=32k seek=$(( (0x700000 / 0x8000) - 1 )) of=${BUILD_DIR}/linux.bin || exit 1

echo "Linux image : ${BUILD_DIR}/linux.bin ready"
exit 0;

