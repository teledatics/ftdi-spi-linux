KDIR ?= /lib/modules/$(shell uname -r)/build
BUILD_DIR:=$(shell pwd)
# KBUILD_OPTIONS += CROSS_COMPILE=arm-linux- ARCH=arm

# include $(KDIR)/.config
obj-m += spi-ft232h.o

all: modules

modules:
	$(MAKE) -C $(KDIR) M=$$PWD modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	@rm -f *.o
	@rm -f *.ko
	@rm -f .*.cmd
	@rm -f *.mod *.mod.c
	@rm -f modules.order
	@rm -f Module.symvers
	@rm -rf .tmp_versions
	@rm -rf html
	@rm -rf latex
	@rm -f doxyfile.inc
