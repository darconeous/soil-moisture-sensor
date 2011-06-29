
DEVICE=attiny13a
AVRDUDE_DEVICE=t13

#DEVICE=attiny25
#AVRDUDE_DEVICE=t25

CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
AVRDUDE=avrdude

AVRDUDEFLAGS += -p $(AVRDUDE_DEVICE)
CFLAGS += -mmcu=$(DEVICE)
LDFLAGS += -mmcu=$(DEVICE)

CFLAGS += -Os
CFLAGS += -DF_CPU=9600000
CFLAGS += -std=c99
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -gdwarf-2

AVRDUDEFLAGS += -y
AVRDUDEFLAGS += -c dragon_dw
AVRDUDEFLAGS += -P usb

# The following two lines strip out unused functions and symbols, making the executable smaller.
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections

all: main.hex main.eep main.lss main.size

clean:
	$(RM) main.o main.elf main.hex main.eep owslave.o moist.o eeprom.o main.lss

hex: main.hex
hex-eeprom: main.eep
burn: main.burn

burn-eeprom:
	./gen-eeprom-hex.sh | $(AVRDUDE) $(AVRDUDEFLAGS) -U eeprom:w:-:i

%.elf: %.o
	$(CC) $(LDFLAGS) -o $@ $^

%.size: %.elf
	@avr-size $<
	@md5 $<

%.burn: %.hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -U $<

%.burn-eeprom: %.eep
	$(AVRDUDE) $(AVRDUDEFLAGS) -U eeprom:w:$<

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .signature $< $@

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

%.eep: %.elf
	$(OBJCOPY) -O ihex -j .eeprom $< $@

main.elf: main.o owslave.o moist.o eeprom.o
main.o: main.c moist.h owslave.h eeprom.h
owslave.o: owslave.c owslave.h
moist.o: moist.c moist.h
eeprom.o: eeprom.c eeprom.h owslave.h

