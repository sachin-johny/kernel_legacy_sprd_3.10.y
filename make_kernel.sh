#!/bin/bash
lcurdir=$(readlink -f .)
while [ "${lcurdir}" != "/" ]; do
    if [ -f ${lcurdir}/build/core/envsetup.mk ]; then
        ANDROID_TOP_BASE=${lcurdir}
        break;
    fi
    lcurdir=$(readlink -f ${lcurdir}/..)
done
export PATH="${ANDROID_TOP_BASE}/out/host/linux-x86/bin":$PATH
BASE_DIR="${ANDROID_TOP_BASE}/out/target/product/hsdroid"

cd ${ANDROID_TOP_BASE}

# make -f vendor/sprd/hsdroid/kernel.mk
make -f vendor/sprd/hsdroid/kernel.mk EXTRAVERSION=-sprd-redbend CONFIG_DEBUG_SECTION_MISMATCH=y \
    KERNEL_CONFIG=sc8800g-trusted-abs-android_defconfig

acp -fpt ${BASE_DIR}/obj/KERNEL/arch/arm/boot/Image ${BASE_DIR}/kernel

cmdline=$(echo $(sed '/BOARD_KERNEL_CMDLINE/!d' vendor/sprd/hsdroid/AndroidBoard.mk) | cut -d'=' -f 2-)

if [ "${cmdline}" != "" ] ; then
echo;echo;
echo CMDLINE=
echo ${cmdline}
mkbootimg --kernel ${BASE_DIR}/kernel --ramdisk ${BASE_DIR}/ramdisk.img --cmdline "${cmdline}" --base 0x0 --output ${BASE_DIR}/boot.img
echo;echo;
else
mkbootimg --kernel ${BASE_DIR}/kernel --ramdisk ${BASE_DIR}/ramdisk.img --base 0x0 --output ${BASE_DIR}/boot.img
fi

if [ -f ${BASE_DIR}/obj/KERNEL/arch/arm/boot/Image -a -f ${BASE_DIR}/ramdisk.img ]; then
cp ${BASE_DIR}/obj/KERNEL/arch/arm/boot/Image ${BASE_DIR}/linux.bin
dd conv=notrunc if=${BASE_DIR}/ramdisk.img obs=32k seek=$(( (0x700000 / 0x8000) - 1 )) of=${BASE_DIR}/linux.bin
echo 1>&2
echo 1>&2
echo "Createing ok -- ${BASE_DIR}/linux.bin" 1>&2
echo 1>&2
echo 1>&2
else
echo 1>&2
echo 1>&2
echo "Error: no ${BASE_DIR}/ramdisk.img is found!" 1>&2
echo "Error: no ${BASE_DIR}/obj/KERNEL/arch/arm/boot/Image is found!" 1>&2
echo 1>&2
echo 1>&2
fi

cp -f ${BASE_DIR}/boot.img /data/ 2>/dev/null
