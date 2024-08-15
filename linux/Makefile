obj-m += gpio-avr-usb.o
KERNELRELEASE ?= $(shell uname -r)
KSRC ?= /lib/modules/$(KERNELRELEASE)/build
MAKE_FLAGS := -C "$(KSRC)" M="$(CURDIR)" ccflags-y=-I"$(CURDIR)/../include"

modules:
	make $(MAKE_FLAGS) modules

modules_install:
	make $(MAKE_FLAGS) modules modules_install

clean:
	make $(MAKE_FLAGS) clean

.PHONY = modules modules_install clean

