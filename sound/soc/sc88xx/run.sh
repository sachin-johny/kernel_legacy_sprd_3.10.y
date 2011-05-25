adb shell stop media

adb shell rmmod sprdphone
adb shell rmmod sc88xx_vbc
adb shell rmmod vbc_codec
adb shell rmmod sc88xx_pcm

adb push ../../../../out/target/product/hsdroid/obj/KERNEL/sound/soc/sc88xx/sc88xx-vbc.ko /data
adb push ../../../../out/target/product/hsdroid/obj/KERNEL/sound/soc/sc88xx/sc88xx-pcm.ko /data
adb push ../../../../out/target/product/hsdroid/obj/KERNEL/sound/soc/sc88xx/vbc-codec.ko /data
adb push ../../../../out/target/product/hsdroid/obj/KERNEL/sound/soc/sc88xx/sprdphone.ko /data
adb shell sync

adb shell insmod /data/sc88xx-vbc.ko
adb shell insmod /data/sc88xx-pcm.ko
adb shell insmod /data/vbc-codec.ko
adb shell insmod /data/sprdphone.ko

#sleep 2

#adb shell start media
