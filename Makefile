
#DEVICE=at90usb1287
#DEVICE=atmega128rfa1
DEVICE=attiny13a

CC=avr-gcc -mmcu=$(DEVICE)
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=9600000 -std=c99
AVRDUDE=avrdude -p t13 -y -c dragon_dw -P usb
DFUPROGRAMMER=dfu-programmer

# The following two lines strip out unused functions and symbols, making the executable smaller.
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections

all: main.hex main.size

clean:
	$(RM) main.o main.elf main.hex owslave.o moist.o

burn: main.burn

burn-eeprom: main.burn-eeprom

%.elf: %.o
	$(CC) -o $@ $^

%.size: %.elf
	avr-size $<

%.burn: %.hex
	$(AVRDUDE) -U flash:w:$<

%.burn-eeprom: %.hex-eeprom
	$(AVRDUDE) -U eeprom:w:$<

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .signature $< $@

%.hex-eeprom: %.elf
	$(OBJCOPY) -O ihex -j .eeprom $< $@

main.elf: owslave.o main.o moist.o
main.o: main.c moist.h owslave.h
owslave.o: owslave.c owslave.h
moist.o: moist.c moist.h


