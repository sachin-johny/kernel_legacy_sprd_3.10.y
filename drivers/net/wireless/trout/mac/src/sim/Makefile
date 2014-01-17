KERNEL_DIR = /lib/modules/$(shell uname -r)/build/  
MODULEDIR := $(shell pwd)

modules:
	make -C $(KERNEL_DIR) M=$(MODULEDIR) modules
clean distclean:
	make -C $(KERNEL_DIR) M=$(MODULEDIR) clean
