# Name: Makefile
# Project: hid-mouse example
# Author: Christian Starkjohann
# Creation Date: 2008-04-07
# Tabsize: 4
# Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
# This Revision: $Id: Makefile 692 2008-11-07 15:07:40Z cs $

DEVICE=atmega8
F_CPU = 16000000 # in Hz
AVRDUDE = avrdude -c usbasp -p $(DEVICE)

CFLAGS  = -Iv-usb/usbdrv -I. -DDEBUG_LEVEL=0
OBJECTS = v-usb/usbdrv/usbdrv.o v-usb/usbdrv/usbdrvasm.o v-usb/usbdrv/oddebug.o main.o

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ....... to build main.hex"
	@echo "make program ... to flash fuses and firmware"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash the firmware (use this on metaboard)"
	@echo "make clean ..... to delete objects and hex file"

hex: main.hex

program: fuse flash

# rule for programming fuse bits:
fuse: main.fuse.bin
# Slow rate in case target has one of the default slow clock configurations.
	$(AVRDUDE) -B200kHz $(shell \
	    i=0; \
	    for reg in lfuse hfuse efuse; do \
	        hex=$$(od -An -tx1 -j$$i -N1 $< | tr -d ' '); \
	        i=$$((i + 1)); \
	        [ "$$hex" ] || break; \
	        echo "-U$$reg:w:0x$$hex:m"; \
	    done)
main.fuse.bin: main.elf Makefile
	avr-objcopy -j .fuse -O binary $< $@

# rule for uploading firmware:
flash: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf *.bin *.o v-usb/usbdrv/*.o main.s v-usb/usbdrv/oddebug.s v-usb/usbdrv/usbdrv.s

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.hex

# debugging targets:

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
