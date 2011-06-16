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

# borrow something from envsetup.sh, namely _arrayoffset, gettop()
source ../build/envsetup.sh
#----------------------------------------------------------------------------
# Configure the Installation and Build Directories
#----------------------------------------------------------------------------
# Source Directory
INSTALL_DIR=`pwd`
BUILD_DIR=${INSTALL_DIR}

#----------------------------------------------------------------------------
# Location of external components
#----------------------------------------------------------------------------

ANDROID_DIR=$(gettop)
ANDROID_ARM_TOOLCHAIN=${ANDROID_DIR}/prebuilt/linux-x86/toolchain/arm-eabi-4.3.1
LINUX_SRC=${INSTALL_DIR}
CMDLINE_FILE=${ANDROID_DIR}/vendor/sprd/hsdroid/AndroidBoard.mk
MKBOOTIMG=${ANDROID_DIR}/out/host/linux-x86/bin/mkbootimg
BOOTIMG=${ANDROID_DIR}/out/target/product/hsdroid/boot.img
INITRD_DIR=${ANDROID_DIR}/out/target/product/hsdroid
INITRD_IMG=${INITRD_DIR}/ramdisk.img

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

function build_kernel()
{
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
        cp arch/arm/configs/${LINUX_DEF_CONFIG} ${LINUX_CONFIG}
        make ARCH=arm CROSS_COMPILE=${ANDROID_ARM_TOOLCHAIN}/bin/arm-eabi- ${LINUX_DEF_CONFIG}
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
}


function clean_kernel()
{
    echo
    echo "Cleaning Linux kernel"
    echo

    make -j2 -f Makefile ARCH=arm  CROSS_COMPILE=${ANDROID_ARM_TOOLCHAIN}/bin/arm-eabi- distclean
}
#rm -f ${BUILD_DIR}/linux.bin

#cp ${BUILD_LINUX_DIR}/arch/arm/boot/Image ${BUILD_DIR}/linux.bin || exit 1
#dd conv=notrunc if=${INITRD_IMG} obs=32k seek=$(( (0x700000 / 0x8000) - 1 )) of=${BUILD_DIR}/linux.bin || exit 1

#echo "Linux image : ${BUILD_DIR}/linux.bin ready"
#INITRD_SIZE=`ls -l ${INITRD_IMG} | cut -d' ' -f5` 
#echo "RAMDISK Size: ${INITRD_SIZE}"
#exit 0;

#----------------------------------------------------------------------------
# boot.img build
#----------------------------------------------------------------------------
function build_bootimg()
{
    echo
    echo "Building boot.img"
    echo

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

    echo "  CMDLINE: ${CMDLINE}"
    ${MKBOOTIMG} --kernel ${KERNELIMG} --ramdisk ${INITRD_IMG} -o ${BOOTIMG} --cmdline "${CMDLINE}"
    cp ${BOOTIMG} .

    if [ $? -ne 0 ]; then
        echo "*** MKBOOTIMG FAILED ***"
        exit 1
    fi
    echo "  BOOTIMG: ${BOOTIMG} is ready"
}

unset MACH_CFG_CHOICES
unset MACH_CHOICES
function get_mach_config()
{
    local f
    local m
    cd arch/arm/configs
    for f in `/bin/ls *8800*abs* 2> /dev/null`
    do
        m=$(echo $f| sed "s/sc8800g-//" |sed "s/-trusted-abs-android_defconfig//")
	MACH_CFG_CHOICES=(${MACH_CFG_CHOICES[@]} $f)
        MACH_CHOICES=(${MACH_CHOICES[@]} $m)
    done
    cd ../../..
}

function print_config_menu()
{
    echo "Pick a machine please:"

    local i=1
    local choice
    for choice in ${MACH_CHOICES[@]}
    do
        echo "     $i. $choice"
	i=$(($i+1))
    done

    echo
}

function get_cfg_from_name()
{
    local i=$((1-$_arrayoffset))
    local config
    for choice in ${MACH_CHOICES[@]}
    do
        if [ "$selection" = "${MACH_CHOICES[i]}" ]
        then
            config=${MACH_CFG_CHOICES[i]}
        fi
	i=$(($i+1))
    done

    echo $config
}

function print_help()
{
cat <<EOF
USAGE: ./build-abslinux.sh [TARGET]

  TARGET could be:
    - <machine name>
    - <machine number>
        build the kernel with corresponding config, and make the boot.img
    - clean.
        "make distclean" for the linux kernel
    - help
        this message

  Leave the TARGET empty means enterring interactive mode.

EXAMPLES:
    ./build-abslinux.sh openphone     build kernel for openphone and make the
                                      boot.img
    ./build-abslinux.sh 2             build kernel for the 2nd option in the 
                                      menu of interactive mode and make the
                                      boot.img
    ./build-abslinux.sh               enter interactive mode, to choose the 
                                      machine you want to build the kernel for,
				      and make the boot.img
EOF
}

# main process (mostly copied from envsetup.sh)
get_mach_config
if [ "$1" ] ; then
    answer=$1
else
    print_config_menu
    echo -n "Which would you like? [openphone] "
    read answer
fi

if [ -z "$answer" ]
then
    selection=openphone
elif [ "$answer" = "help" ]
then
    print_help
    exit 0
elif [ "$answer" = "clean" ]
then
    clean_kernel
    rm -f boot.img
    exit 0
elif (echo -n $answer | grep -q -e "^[0-9][0-9]*$")
then
    if [ $answer -le ${#MACH_CHOICES[@]} ]
    then
        selection=${MACH_CHOICES[$(($answer-$_arrayoffset))]}
    fi
else
    selection=$answer
fi

LINUX_DEF_CONFIG=$(get_cfg_from_name)
echo $LINUX_DEF_CONFIG
if [ -z $LINUX_DEF_CONFIG ]
then
    echo "unsupported machine name..."
    exit 1
fi

build_kernel
build_bootimg

