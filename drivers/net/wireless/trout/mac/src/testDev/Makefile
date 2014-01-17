obj-m := dev_interf.o

KERNEL_DIR = /home/apuser/idh.code/kernel_2.3.5  
MODULEDIR := $(shell pwd)

modules:
	make -C $(KERNEL_DIR) M=$(MODULEDIR) modules
clean distclean:
	make -C $(KERNEL_DIR) M=$(MODULEDIR) clean
