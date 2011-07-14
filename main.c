#include "owslave.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdint.h>

//#define OLD_SOIL_MOISTURE_SENSOR_BOARD  1

#define EXTRA_TEMP_RESOLUTION           3

#ifndef MOIST_DRIVE_PIN
#define MOIST_DRIVE_PIN             4
#endif

#ifndef OWSLAVE_IOPIN
#define OWSLAVE_IOPIN   (0)
#endif

#ifndef MOIST_COLLECTOR_PIN
#if OLD_SOIL_MOISTURE_SENSOR_BOARD
#define MOIST_COLLECTOR_PIN         2
#else
#define MOIST_COLLECTOR_PIN         3
#endif
#endif

#ifndef MOIST_FULLY_DRIVE_PULSES
#if OLD_SOIL_MOISTURE_SENSOR_BOARD
#define MOIST_FULLY_DRIVE_PULSES    0
#else
#define MOIST_FULLY_DRIVE_PULSES    1
#endif
#endif

#ifndef ENABLE_WATCHDOG
#define ENABLE_WATCHDOG         1
#endif

#ifndef DEVICE_IS_SPACE_CONSTRAINED
#define DEVICE_IS_SPACE_CONSTRAINED (FLASHEND <= 0x3FF)
#endif

#ifndef OWSLAVE_SUPPORTS_CONVERT_INDICATOR
#define OWSLAVE_SUPPORTS_CONVERT_INDICATOR 1
#endif

#ifndef SUPPORT_DEVICE_NAMING
#define SUPPORT_DEVICE_NAMING   0
#endif

#ifndef SUPPORT_VOLTAGE_READING
#define SUPPORT_VOLTAGE_READING !DEVICE_IS_SPACE_CONSTRAINED
#endif

#ifndef EMULATE_DS18B20
#define EMULATE_DS18B20         !DEVICE_IS_SPACE_CONSTRAINED
#endif

#ifndef DO_MEDIAN_FILTERING
#define DO_MEDIAN_FILTERING     !DEVICE_IS_SPACE_CONSTRAINED
#endif

#ifndef DO_CALIBRATION
#define DO_CALIBRATION          !DEVICE_IS_SPACE_CONSTRAINED
#endif

// ----------------------------------------------------------------------------

#define MOIST_MAX_VALUE \
        (const uint16_t)((1l << \
                (sizeof(uint16_t) * 8l)) - 1)

#define sbi(x, y)    x |= (uint8_t)(1 << y)
#define cbi(x, y)    x &= (uint8_t) ~(1 << y)

#define ATTR_NO_INIT   __attribute__ ((section(".noinit")))

#if !defined(TIMSK0) && defined(TIMSK)
#define TIMSK0 TIMSK
#endif

#include <util/crc16.h>
#define _crc_ibutton_update     _crc_ibutton_update

#define SAVE_POWER()        __asm__ __volatile__ ("sleep")

#ifndef WDTO_MAX
#if defined(__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__)
#define WDTO_MAX WDTO_2S
#else
#define WDTO_MAX WDTO_8S
#endif
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark Memory Page Layout

// Page 1 - Values
struct {
	uint16_t	moisture;
	uint16_t	raw;
	uint16_t	temp;
	uint16_t	voltage;
} value ATTR_NO_INIT;

// Page 2 - cfguration
struct cfg_t {
	uint16_t	moisture;
	uint16_t	raw;
	uint16_t	temp;
	uint16_t	voltage;
} cfg ATTR_NO_INIT;

// Page 3 - Alarms
struct alarm_t {
	uint8_t moisture_low;
	uint8_t moisture_high;

	uint8_t pad[2];

	uint8_t temp_low;
	uint8_t temp_high;

	uint8_t voltage_low;
	uint8_t voltage_high;
} alarm ATTR_NO_INIT;

// Page 4 - Calibration
struct calib_t {
	uint8_t range;
	uint8_t offset;
	uint8_t samplecount;

	uint8_t reserved;

	int8_t	temp_offset;
	uint8_t pad[3];
} calib ATTR_NO_INIT;

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
//uint8_t was_interrupted ATTR_NO_INIT
register uint8_t was_interrupted __asm__("r3");
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark EEPROM Layout

owslave_addr_t owslave_addr EEMEM = {
	.s.type		= OWSLAVE_TYPE_MOIST,
	.s.serial	= { 0x00,		   0x00,0x00, 0x00, 0x00, 0xff },
	.s.crc		= 0x4D
};

enum {
	ALARM_ENABLE_L = (1 << 10),
	ALARM_ENABLE_H = (1 << 11),
	ALARM_FLAG_L = (1 << 12),
	ALARM_FLAG_H = (1 << 13),
};

struct cfg_t cfg_eeprom EEMEM = {
	.moisture	= ALARM_ENABLE_L | ALARM_ENABLE_H,
	.raw		= 0,
	.temp		= ALARM_ENABLE_L | ALARM_ENABLE_H,
	.voltage	= 0,
};

struct alarm_t alarm_eeprom EEMEM = {
	0x00, 0xFF,
	0x00, 0xFF,
	0x00, 0xFF,
	0x00, 0xFF,
};

struct calib_t calib_eeprom EEMEM = {
	.range			= (95 - 8),
	.offset			= (8),
	.samplecount	= 128,
	.temp_offset	= 24 - 9,
};

#if SUPPORT_DEVICE_NAMING
char device_name[32] EEMEM = "";
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark Misc. Helper Functions

#if DO_MEDIAN_FILTERING
uint16_t median_uint16(
	uint16_t a, uint16_t b, uint16_t c
) {
	if(a < c) {
		if(b < a)
			return a;
		else if(c < b)
			return c;
	} else {
		if(a < b)
			return a;
		else if(b < c)
			return c;
	}
	return b;
}
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark OWSlave Functions

uint8_t
owslave_read_bit() {
	// Wait for the bus to go idle.
	if(bit_is_clear(PINB, OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB, OWSLAVE_IOPIN);

	// Wait for the slot to open.
	loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);

	// Wait until we should sample.
	_delay_us(30);

	// Return the value of the bit.
	return bit_is_set(PINB, OWSLAVE_IOPIN);
}

void
owslave_write_bit(uint8_t v) {
	// Wait for the bus to go idle.
	if(bit_is_clear(PINB, OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB, OWSLAVE_IOPIN);

	if(v == 0) {
		// Disable interrupts, so we can make sure we get the timing right.
		cli();

		// Wait for the slot to open.
		loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);

		// Assert our zero bit.
		sbi(DDRB, OWSLAVE_IOPIN);

		// Wait for the master to sample us.
		_delay_us(30);

		// Return the bus back to idle.
		cbi(DDRB, OWSLAVE_IOPIN);

		// Turn interrupts back on.
		sei();
	} else {
		// Wait for the slot to open.
		loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);
	}
}

uint8_t
owslave_read_byte() {
	// We really DON'T need to initialize this. Honest.
	uint8_t ret;

	for(uint8_t i = 0; i != 8; i++) {
		ret >>= 1;
		if(owslave_read_bit())
			sbi(ret, 7);
	}
	return ret;
}

inline uint16_t
owslave_read_word() {
	return owslave_read_byte() + (owslave_read_byte() << 8);
}

void
owslave_write_byte(uint8_t byte) {
	for(uint8_t i = 0; i != 8; i++) {
		owslave_write_bit(byte & 1);
		byte >>= 1;
	}
}

void
owslave_main() {
	uint8_t cmd;
	uint8_t flags;

	TCCR0B = 0; // Stop the timer.

	// Initialize the IOPin
	cbi(PORTB, OWSLAVE_IOPIN);
	cbi(DDRB, OWSLAVE_IOPIN);

	// Enable overflow interrupt, which is how we detect reset pulses.
	// The interrupt handler for the overflow interrupt isn't actually
	// defined, which means that __bad_interrupt gets called instead.
	// This causes a soft-reset of the device by jumping to address 0x0000.
	TIMSK0 = 0;
	sbi(TIMSK0, TOIE0);

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
	OCR0A = (uint8_t)(30l * F_CPU / 8l / 1000000l);
#endif

	// Allow the 1wire pin to generate interrupts.
	sbi(PCMSK, OWSLAVE_IOPIN);

	// Turn on the pin-change interrupt.
	sbi(GIMSK, PCIE);

	// Enable interrupts.
	sei();

	// Check to see if this was a hard or soft reset.
	if(MCUSR) {
		// Hard reset. No presence pulse.
		wdt_disable();
		owslave_cb_recall();
		MCUSR = 0;

		goto wait_for_reset;
	}

	// Reset the MCU status register.
	MCUSR = 0;

#if ENABLE_WATCHDOG
	// Turn on the watchdog with a maximum watchdog timeout period.
	wdt_enable(WDTO_MAX);
#endif

	// Wait for reset pulse to end.
	while(bit_is_clear(PINB, OWSLAVE_IOPIN)) SAVE_POWER();

#if ENABLE_WATCHDOG
	wdt_reset();
#endif

	// Let the bus idle for 20µSec
	_delay_us(20);

	// Send the 80µSec presence pulse
	sbi(DDRB, OWSLAVE_IOPIN);
	_delay_us(80);
	cbi(DDRB, OWSLAVE_IOPIN);

	// Read ROM command
	cmd = owslave_read_byte();
	flags = 0;

	// Interpret what the ROM command means.
	if(cmd == OWSLAVE_ROMCMD_MATCH)
		flags = _BV(2);
	else if(cmd == OWSLAVE_ROMCMD_READ)
		flags = _BV(0);
	else if((cmd == OWSLAVE_ROMCMD_SEARCH)
	    || ((cmd == OWSLAVE_ROMCMD_ALARM_SEARCH)
	        && owslave_cb_alarm_condition()
	    )
	)
		flags = _BV(0) | _BV(1) | _BV(2);
	else if(cmd == OWSLAVE_ROMCMD_SKIP)
		flags = 0;
	else goto wait_for_reset;

	// Perfom the ROM command.
	if(flags) {
		for(uint8_t i = 0; i != 8; i++) {
			uint8_t byte = eeprom_read_byte(&owslave_addr.d[i]);
			uint8_t j = 8;
			do {
				if(flags & _BV(0))
					owslave_write_bit(byte & 1);
				if(flags & _BV(1))
					owslave_write_bit((~byte) & 1);
				if(flags & _BV(2))
					if((byte & 1) ^ owslave_read_bit())
						goto wait_for_reset;
				byte >>= 1;
			} while(--j);
		}
	}

#if ENABLE_WATCHDOG
	wdt_reset();
#endif

	// Read function command
	cmd = owslave_read_byte();

#if SUPPORT_DEVICE_NAMING
	if(cmd == OWSLAVE_FUNCCMD_RD_NAME) {
		for(uint8_t i = 0; i < sizeof(device_name); i++) {
			owslave_write_byte(eeprom_read_byte(&device_name[i]));
		}
		owslave_write_byte(0x00);
	} else
#endif
	if((cmd == OWSLAVE_FUNCCMD_RD_MEM)
	    || (cmd == OWSLAVE_FUNCCMD_WR_MEM)
	) {
		uint16_t crc = _crc16_update(0, cmd);
		uint8_t i = owslave_read_byte() & 31;
		owslave_read_byte();
		crc = _crc16_update(crc, i);
		crc = _crc16_update(crc, 0);
		do {
			uint8_t byte;
			if(cmd == OWSLAVE_FUNCCMD_RD_MEM) {
				byte = ((uint8_t*)&value)[i++];
				owslave_write_byte(byte);
			} else {
				byte = owslave_read_byte();
				    ((uint8_t*)&value)[i++] = byte;
			}
			crc = _crc16_update(crc, byte);
			if((i & 7) == 0) {
				owslave_write_byte(crc);
				owslave_write_byte(crc >> 8);
				crc = 0;
			}
		} while(i < 32);
	} else if(cmd == OWSLAVE_FUNCCMD_CONVERT) {
		owslave_read_byte();    // Ignore input select mask
		owslave_read_byte();    // Ignore read-out control
		owslave_cb_convert();
	} else if(cmd == OWSLAVE_FUNCCMD_COMMIT_MEM) {
		owslave_cb_commit();
	} else if(cmd == OWSLAVE_FUNCCMD_RECALL_MEM) {
		owslave_cb_recall();
	} else if(cmd == OWSLAVE_FUNCCMD_CONVERT_T) {
		owslave_cb_convert();
	}
#if EMULATE_DS18B20
	else if(cmd == OWSLAVE_FUNCCMD_RD_SCRATCH) {
		uint8_t crc = 0;
		crc = _crc_ibutton_update(crc, ((uint8_t*)&value.temp)[0]);
		owslave_write_byte(((uint8_t*)&value.temp)[0]);
		crc = _crc_ibutton_update(crc, ((uint8_t*)&value.temp)[1]);
		owslave_write_byte(((uint8_t*)&value.temp)[1]);
		for(uint8_t i = 0; i < 6; i++) {
			crc = _crc_ibutton_update(crc, 0);
			owslave_write_byte(0);
		}
		owslave_write_byte(crc);
	}
#endif

wait_for_reset:
	for(;; ) {
		// Allow the 1wire pin to generate interrupts.
		sbi(PCMSK, OWSLAVE_IOPIN);

		// Turn on the pin-change interrupt.
		sbi(GIMSK, PCIE);

		// Enable interrupts.
		sei();

#if ENABLE_WATCHDOG
		wdt_reset();
#endif
	}
}

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
void
owslave_begin_busy() {
	sbi(TIMSK0, OCIE0A);
}

void
owslave_end_busy() {
	cbi(DDRB, OWSLAVE_IOPIN);
	cbi(TIMSK0, OCIE0A);
}
#else
#define owslave_begin_busy()    do {} while(0)
#define owslave_end_busy()  do {} while(0)
#endif

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
ISR(TIM0_COMPA_vect) {
	cbi(DDRB, OWSLAVE_IOPIN);
	was_interrupted++;
}
#endif

// Pin change interrupt
ISR(PCINT0_vect) {
	TCCR0B = 0; // Stop the timer.

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
	was_interrupted++;
#endif

	// Is this a high-to-low transition?
	if(bit_is_clear(PINB, OWSLAVE_IOPIN)) {
#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
		if(bit_is_set(TIMSK0, OCIE0A))
			sbi(DDRB, OWSLAVE_IOPIN);
#endif

		TCNT0 = 0; // Reset counter.

		// Start timer with prescaler of 1/8.
		// @9.6MHz: ~.8333µSecconds per tick, 214µSecond reset pulse
		// @8.0MHz: ~1µSecond per tick, 256µSecond reset pulse
		TCCR0B = (1 << 1);
	}
}

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark Other

uint8_t
owslave_cb_alarm_condition() {
	uint8_t ret = 0;

	for(int i = 0; i < 4; i++)
		ret |=
		    !(uint8_t) !(((uint16_t*)&cfg)[i] &
		        (ALARM_FLAG_H | ALARM_FLAG_L));

	return ret;
}

void
update_alarm_flags() {
	for(uint8_t i = 0; i < 8; i += 2) {
		uint8_t tmp = ((uint8_t*)&value)[i + 1];
		uint16_t cfg = ((uint16_t*)&cfg)[i / 2];

		cfg &= ~(ALARM_FLAG_L | ALARM_FLAG_H);

		if((cfg & ALARM_ENABLE_L) && (tmp <= ((uint8_t*)&alarm)[i]))
			cfg |= ALARM_FLAG_L;

		if((cfg & ALARM_ENABLE_H) && (tmp >= ((uint8_t*)&alarm)[i + 1]))
			cfg |= ALARM_FLAG_H;

		    ((uint16_t*)&cfg)[i / 2] = cfg;
	}
}

// This is the general capacitance-reading function.
uint16_t
moist_calc() {
	uint16_t v;

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
again:
#endif

	// Make sure pins are configured.
	cbi(PORTB, MOIST_DRIVE_PIN);
	cbi(PORTB, MOIST_COLLECTOR_PIN);
	sbi(DDRB, MOIST_DRIVE_PIN);
	sbi(DDRB, MOIST_COLLECTOR_PIN);

	// Wait long enough for the sensing capacitor to fully flush.
	_delay_us(10);

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
	was_interrupted = 0;
#else
	cli();
#endif

	cbi(DDRB, MOIST_DRIVE_PIN);
	cbi(DDRB, MOIST_COLLECTOR_PIN);

	for(v = 0;
	        (v != MOIST_MAX_VALUE) &&
	    bit_is_clear(PINB, MOIST_COLLECTOR_PIN);
	    v++
	) {
		sbi(PORTB, MOIST_DRIVE_PIN);
#if MOIST_FULLY_DRIVE_PULSES
		sbi(DDRB, MOIST_DRIVE_PIN);
		cbi(DDRB, MOIST_DRIVE_PIN);
#endif
		cbi(PORTB, MOIST_DRIVE_PIN);
#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
		if(was_interrupted)
			goto again;
		_NOP();
#else
		_delay_us(1);
#endif
	}

	// Turn interrupts back on.
#if !OWSLAVE_SUPPORTS_CONVERT_INDICATOR
	sei();
#endif

	// Pull both lines low to avoid floating inputs
	sbi(DDRB, MOIST_DRIVE_PIN);
	sbi(DDRB, MOIST_COLLECTOR_PIN);

	return v;
}

void
owslave_cb_convert() {
	owslave_begin_busy();

	// Set all values to OxFFFF
#if !DEVICE_IS_SPACE_CONSTRAINED
	memset(&value, 0xFF, 8);
#endif

#if SUPPORT_VOLTAGE_READING || EMULATE_DS18B20
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
#endif

#if SUPPORT_VOLTAGE_READING
#if defined(__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__)
	// Vref=Vcc, Input=PORTB2
	ADMUX = _BV(MUX0);
	cbi(DDRB, 2);
	sbi(PORTB, 2);
#else
	// Vref=Vcc, Input=Vbg
	ADMUX = _BV(MUX3) | _BV(MUX2);
#endif

	// Throw away the first reading.
	sbi(ADCSRA, ADSC);
	_delay_ms(1);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	// Now read the voltage for real.
	sbi(ADCSRA, ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	value.voltage = ADC;
#endif // SUPPORT_VOLTAGE_READING

#if EMULATE_DS18B20
	ADMUX = _BV(REFS1) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0);

	// Throw away the first reading.
	sbi(ADCSRA, ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	value.temp = 0;
	for(int i = 0; i < (1 << (4 + EXTRA_TEMP_RESOLUTION)); i++) {
		sbi(ADCSRA, ADSC);
		loop_until_bit_is_clear(ADCSRA, ADSC);
		value.temp += ADC - 270 + calib.temp_offset;
	}
	value.temp >>= EXTRA_TEMP_RESOLUTION;
#endif

	uint16_t value_a = 0;

	for(uint8_t i = 0; i < calib.samplecount; i++)
		value_a += moist_calc();

#if DO_MEDIAN_FILTERING
	{
		uint16_t value_b = 0;
		uint16_t value_c = 0;

		_delay_ms(1);

		for(uint8_t i = 0; i < calib.samplecount; i++)
			value_b += moist_calc();

		_delay_ms(1);

		for(uint8_t i = 0; i < calib.samplecount; i++)
			value_c += moist_calc();

		value_a = median_uint16(value_a, value_b, value_c);
	}
#endif

	value.raw = value_a;

#if DO_CALIBRATION
	// Apply calibration
	{
		uint16_t tmp = calib.offset * calib.samplecount;
		if(value_a >= tmp)
			value_a -= calib.offset * calib.samplecount;
		else
			value_a = 0;

		value_a =
		    ((uint32_t)value_a *
		    1024) / ((uint32_t)calib.range * calib.samplecount);

		if(value_a >= 1024)
			value_a = 0x3FF;

		value_a <<= 6;
	}
#endif

	value.moisture = value_a;

	update_alarm_flags();

	owslave_end_busy();
}

void
owslave_cb_recall() {
	eeprom_busy_wait();
	eeprom_read_block(&cfg, &cfg_eeprom, sizeof(cfg_eeprom) +
		sizeof(alarm_eeprom) + sizeof(calib_eeprom));
}

void
owslave_cb_commit() {
	eeprom_update_block(&cfg, &cfg_eeprom, sizeof(cfg_eeprom) +
		sizeof(alarm_eeprom) + sizeof(calib_eeprom));
	eeprom_busy_wait();
}

void
main(void) {
	// Main loop.
	owslave_main();
	wdt_enable(WDTO_15MS);
}
