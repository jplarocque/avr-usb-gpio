DEVICE := atmega8
F_CPU := 16000000 # in Hz

CFLAGS := -Wall -Os
CPPFLAGS := -mmcu=$(DEVICE) -I. -I../include -Iv-usb/usbdrv -DDEBUG_LEVEL=0 -DF_CPU=$(F_CPU)
AVRDUDE_FLAGS := -c usbasp

CC := avr-gcc
OBJCOPY := avr-objcopy
AVRDUDE := avrdude

all: main.bin main.fuse.bin
	@avr-size main.elf

program: fuse flash

fuse: main.fuse.bin
# Slow rate in case target has one of the default slow clock configurations.
	$(AVRDUDE) $(AVRDUDE_FLAGS) -p $(DEVICE) -B200kHz $(shell \
	    i=0; \
	    for reg in lfuse hfuse efuse; do \
	        hex=$$(od -An -tx1 -j$$i -N1 $< | tr -d ' '); \
	        i=$$((i + 1)); \
	        [ "$$hex" ] || break; \
	        echo "-U$$reg:w:0x$$hex:m"; \
	    done)

flash: main.bin
# 1.8 MHz chosen to be 90 % of the maximum serial programming clock of 2 MHz
# for MCUs running at the lowest supported CPU clock of 12 MHz.
	$(AVRDUDE) $(AVRDUDE_FLAGS) -p $(DEVICE) -B1.8MHz -U flash:w:$<:r

clean:
	rm -f *.o *.elf *.bin *.var v-usb/usbdrv/*.o

.PHONY: all program fuse flash clean phony

define DEPENDABLE_VAR
$1.var: phony
	@printf '%s\n' '$($1)' > $1.var.new
	@cmp -s $1.var $1.var.new && rm $1.var.new || mv $1.var.new $1.var
endef
$(eval $(call DEPENDABLE_VAR,CFLAGS))
$(eval $(call DEPENDABLE_VAR,CPPFLAGS))

main.bin: main.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
main.fuse.bin: main.elf
	$(OBJCOPY) -j .fuse -O binary $< $@

main.elf: main.o $(addprefix v-usb/usbdrv/,usbdrv.o usbdrvasm.o oddebug.o)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^

%.o: %.c CFLAGS.var CPPFLAGS.var usbconfig.h ../include/usb_ids.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

%.o: %.S CFLAGS.var CPPFLAGS.var usbconfig.h ../include/usb_ids.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# Cancel implicit rules, which can mask failures in the DEPENDABLE_VAR
# dependency logic.
%.o: %.c
%.o: %.S
%.o: %.s
