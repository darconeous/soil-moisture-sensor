# This Makefile is public domain.

#DEVICE=attiny13a
DEVICE=attiny25

ifeq ($(DEVICE),attiny25)
AVRDUDE_DEVICE=t25
CFLAGS += -DF_CPU=8000000
endif

ifeq ($(DEVICE),attiny13a)
AVRDUDE_DEVICE=t13
CFLAGS += -DF_CPU=9600000
endif

CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
AVRDUDE=avrdude

AVRDUDEFLAGS += -p $(AVRDUDE_DEVICE)
CFLAGS += -mmcu=$(DEVICE)
LDFLAGS += -mmcu=$(DEVICE)

CFLAGS += -Os
CFLAGS += -std=c99
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -gdwarf-2
#CFLAGS += -mcall-prologues
#CFLAGS += -mint8

AVRDUDEFLAGS += -y
AVRDUDEFLAGS += -c dragon_dw
#AVRDUDEFLAGS += -c dragon_isp
AVRDUDEFLAGS += -P usb

# The following two lines strip out unused functions
# and symbols, making the executable smaller.
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections

all: main.hex main.eep main.lss main.size

clean:
	$(RM) main.o main.elf main.hex main.eep main.lss
	$(RM) eagle/soil-moisture-sensor.cmp
	$(RM) eagle/soil-moisture-sensor.drd
	$(RM) eagle/soil-moisture-sensor.dri
	$(RM) eagle/soil-moisture-sensor.gpi
	$(RM) eagle/soil-moisture-sensor.plc
	$(RM) eagle/soil-moisture-sensor.pls
	$(RM) eagle/soil-moisture-sensor.pro
	$(RM) eagle/soil-moisture-sensor.sol
	$(RM) eagle/soil-moisture-sensor.stc
	$(RM) eagle/soil-moisture-sensor.sts

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

main.elf: main.o
main.o: main.c owslave.h Makefile

