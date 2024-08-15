obj-m += gpio-avr-usb.o
KERNELRELEASE ?= $(shell uname -r)
KSRC ?= /lib/modules/$(KERNELRELEASE)/build

modules:
	make -C "$(KSRC)" M="$(CURDIR)" modules

modules_install:
	make -C "$(KSRC)" M="$(CURDIR)" modules modules_install

clean:
	make -C "$(KSRC)" M="$(CURDIR)" clean

.PHONY = modules modules_install clean

