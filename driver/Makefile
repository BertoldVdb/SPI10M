ifneq ($(KERNELRELEASE),)
	obj-m += spi10m.o
else
	KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD INSTALL_MOD_DIR=kernel/net/802 modules_install
	depmod -a `uname -r`

clean:
	rm -f *.o *.ko Module.symvers spi10m.mod.c modules.order .*.cmd
	rm -rf .tmp_versions 
endif
