#!/bin/bash
function get_pwd_abs()
{
lcurdir=$(readlink -f .)
while [ "${lcurdir}" != "/" ]; do
    if [ -f ${lcurdir}/$1 ]; then
        echo "${lcurdir}"
        break;
    fi
    lcurdir=$(readlink -f ${lcurdir}/..)
done
}
ANDROID_3RDPARTY_BASE=$(get_pwd_abs 3rdparty/common/build.3rdparty.common.sh)/3rdparty
[ -e ${ANDROID_3RDPARTY_BASE} ] && {
cd ${ANDROID_3RDPARTY_BASE}
selected_product="$1"
[ "$1" ] && shift 1
./build.products.sh "${selected_product}" bootimage $@
}
