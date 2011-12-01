make -C ~/works/sprdroid/kernel M=`pwd` modules $1
cp -r ../sc8810 ~/vobs/sc88xx_backup/`date +'%m%d%H%M%S'`
tar jcvf sprd-asoc-ko.tar.bz2 *.ko 
