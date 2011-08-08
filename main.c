#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdint.h>

#define OLD_SOIL_MOISTURE_SENSOR_BOARD  0
#define DO_MEDIAN_FILTERING				1

#define FIRMWARE_VERSION				0

#define CALIBRATED_BITS					10

#ifndef OWSLAVE_IOPIN
#define OWSLAVE_IOPIN					(0)
#endif

#ifndef MOIST_DRIVE_PIN
#define MOIST_DRIVE_PIN					4
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

#ifndef DEVICE_IS_SPACE_CONSTRAINED
#define DEVICE_IS_SPACE_CONSTRAINED (FLASHEND <= 0x3FF)
#endif

#ifndef OWSLAVE_SUPPORTS_CONVERT_INDICATOR
#define OWSLAVE_SUPPORTS_CONVERT_INDICATOR 1
#endif

#ifndef SUPPORT_DEVICE_NAMING
#define SUPPORT_DEVICE_NAMING   0
#endif

#ifndef ENABLE_WATCHDOG
#define ENABLE_WATCHDOG         !DEVICE_IS_SPACE_CONSTRAINED
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

#define TEMP_RESOLUTION_MAX                 (0x7)

#define TEMP_RESOLUTION_MASK                (0x7)
#define OVERSAMPLE_COUNT_EXPONENT_MASK      (0xF)

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark owslave types

typedef uint8_t bool;
#define true(bool) (1)
#define false(bool) (0)

#define OWSLAVE_T_X     (20)
#define OWSLAVE_T_PDH   (20)    // 15-60 uSec
#define OWSLAVE_T_PDL   (80)    // 60-240 uSec

enum {
	OWSLAVE_TYPE_CUSTOMFLAG = 0x80,

	OWSLAVE_TYPE_DS2401     = 0x01, // Serial-number only
	OWSLAVE_TYPE_DS1920     = 0x10,
	OWSLAVE_TYPE_DS18B20    = 0x28, // Temperature Sensor
	OWSLAVE_TYPE_DS2450     = 0x20, // 4-channel ADC

	OWSLAVE_TYPE_MOIST      = OWSLAVE_TYPE_DS2450|OWSLAVE_TYPE_CUSTOMFLAG
};

enum {
	OWSLAVE_ROMCMD_READ=0x33,           // 00110011b
	OWSLAVE_ROMCMD_MATCH=0x55,          // 01010101b
	OWSLAVE_ROMCMD_SKIP=0xCC,           // 11001100b
	OWSLAVE_ROMCMD_SEARCH=0xF0,         // 11110000b
	OWSLAVE_ROMCMD_ALARM_SEARCH=0xEC,   // 11101100b

	// DS2450, Quad ADC
	OWSLAVE_FUNCCMD_RD_MEM=0xAA,
	OWSLAVE_FUNCCMD_WR_MEM=0x55,
	OWSLAVE_FUNCCMD_CONVERT=0x3C,

	OWSLAVE_FUNCCMD_COMMIT_MEM=0x48,
	OWSLAVE_FUNCCMD_RECALL_MEM=0xB8,

	// DS18B20, 1-Wire Thermometer
	OWSLAVE_FUNCCMD_CONVERT_T=0x44,
	OWSLAVE_FUNCCMD_WR_SCRATCH=0x4E,
	OWSLAVE_FUNCCMD_RD_SCRATCH=0xBE,
	OWSLAVE_FUNCCMD_CP_SCRATCH=0x48,
	OWSLAVE_FUNCCMD_RECALL_E2=0xB8,

	// Custom
	OWSLAVE_FUNCCMD_RD_NAME=0xF1,
	OWSLAVE_FUNCCMD_WR_NAME=0xFE,
};

typedef union {
	struct {
		uint8_t type;
		uint8_t serial[6];
		uint8_t crc;
	} s;
	uint8_t d[8];
} owslave_addr_t;

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark Memory Page Layout

// Page 1 - Values
struct {
	uint16_t	moisture;
	uint16_t	raw;
	int16_t		temp;
	uint16_t	voltage;
} value ATTR_NO_INIT;

// Page 2 - configuration
struct cfg_t {
	uint8_t	alarm_low;
	uint8_t	alarm_high;

	uint8_t	flags;
	uint8_t	reserved[4];


	uint8_t firmware_version;
} cfg ATTR_NO_INIT;

// Page 3 - Calibration
struct calib_t {
	uint8_t range;
	uint8_t offset;
	uint8_t flags;
	int8_t	temp_offset;

	uint8_t reserved[4];
} calib ATTR_NO_INIT;

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
//uint8_t was_interrupted ATTR_NO_INIT;
register uint8_t was_interrupted __asm__("r3");
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark EEPROM Layout

owslave_addr_t owslave_addr EEMEM = {
	.s.type		= OWSLAVE_TYPE_MOIST,
	.s.serial	= { 0x00, 0x00, 0x00, 0x00, 0x00, 0xff },
	.s.crc		= 0x4D
};

struct cfg_t cfg_eeprom EEMEM = {
	.alarm_low		= 0x00,
	.alarm_high		= 0xFF,
	.flags			= 0x00 | (7&TEMP_RESOLUTION_MASK),
};

struct calib_t calib_eeprom EEMEM = {
	.range			= (0x69),
	.offset			= (0x11),
	.flags			= 0x6,
	.temp_offset	= 0,
};

#if SUPPORT_DEVICE_NAMING
char device_name[16] EEMEM = "";
#endif

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark Misc. Helper Functions

#if DO_MEDIAN_FILTERING
static uint16_t
median_uint16(
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
#pragma mark Other

#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
static void
owslave_begin_busy() {
	sbi(TIMSK0, OCIE0A);
}

static void
owslave_end_busy() {
	cbi(DDRB, OWSLAVE_IOPIN);
	cbi(TIMSK0, OCIE0A);
}
#else
#define owslave_begin_busy()    do {} while(0)
#define owslave_end_busy()  do {} while(0)
#endif

static uint8_t
owslave_cb_alarm_condition() {
	uint8_t moist_h = (value.moisture>>8);
	return (moist_h > cfg.alarm_high) || (moist_h < cfg.alarm_low);
}

// This is the general capacitance-reading function.
static uint16_t
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
	_delay_ms(2);

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

#if SUPPORT_VOLTAGE_READING
static void
convert_voltage() {
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
}
#endif // SUPPORT_VOLTAGE_READING

#if EMULATE_DS18B20
static void
convert_temp() {
	int32_t temp = 0;

	ADMUX = _BV(REFS1) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0);

	// Throw away the first reading.
	sbi(ADCSRA, ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	for(uint16_t i = (1 << (4 + (cfg.flags&TEMP_RESOLUTION_MASK))); i; --i) {
		sbi(ADCSRA, ADSC);
		loop_until_bit_is_clear(ADCSRA, ADSC);
		temp += ADC - 270;
	}
	temp >>= (cfg.flags&TEMP_RESOLUTION_MASK);
	value.temp = (uint32_t)7250*(uint32_t)16/value.voltage - 337 + temp + calib.temp_offset*2;
}
#endif

static uint16_t
read_moisture() {
	uint16_t ret = 0;

	for(int i = (1 << (calib.flags&OVERSAMPLE_COUNT_EXPONENT_MASK)); i; --i)
		ret += moist_calc();

	return ret;
}

static void
convert_moisture() {
	uint16_t value_a = 0;

	value_a = read_moisture();

#if DO_MEDIAN_FILTERING
	{
		uint16_t value_b;
		uint16_t value_c;

		_delay_ms(4);

		value_b = read_moisture();

		_delay_ms(4);

		value_c = read_moisture();

		value_a = median_uint16(value_a, value_b, value_c);
	}
#endif

	value.raw = value_a;

#if DO_CALIBRATION
	// Apply calibration
	{
		uint16_t tmp = (calib.offset << (calib.flags&OVERSAMPLE_COUNT_EXPONENT_MASK));

		if(value_a >= tmp)
			value_a -= tmp;
		else
			value_a = 0;

		value_a = ((uint32_t)value_a << CALIBRATED_BITS)
			/ (uint32_t)((uint32_t)calib.range << (calib.flags&OVERSAMPLE_COUNT_EXPONENT_MASK));

		if(value_a > (1<<CALIBRATED_BITS)-1)
			value_a = (1<<CALIBRATED_BITS)-1;

		value_a <<= 16-CALIBRATED_BITS;
	}
#endif

	value.moisture = value_a;
}

static void
owslave_cb_convert() {
	owslave_begin_busy();

#if !DEVICE_IS_SPACE_CONSTRAINED
	// Set all values to OxFFFF
	uint8_t i = 7;
	do {
		((uint8_t*)&value)[i]=0xFF;
	} while(i--);
#endif

#if SUPPORT_VOLTAGE_READING
	convert_voltage();
#endif

#if EMULATE_DS18B20
	convert_temp();
#endif

	convert_moisture();

	owslave_end_busy();
}

static void
owslave_cb_recall() {
	eeprom_busy_wait();
	eeprom_read_block(
		&cfg,
		&cfg_eeprom,
		sizeof(cfg_eeprom) + sizeof(calib_eeprom)
	);
	cfg.firmware_version = FIRMWARE_VERSION;
}

static void
owslave_cb_commit() {
#if ENABLE_WATCHDOG
	wdt_reset();
#endif
	eeprom_update_block(
		&cfg,
		&cfg_eeprom,
		sizeof(cfg_eeprom) + sizeof(calib_eeprom)
	);
	eeprom_busy_wait();
}

// ----------------------------------------------------------------------------
#pragma mark -
#pragma mark OWSlave Functions

static uint8_t
owslave_read_bit() {
	// Wait for the bus to go idle if it is already low.
	if(bit_is_clear(PINB, OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB, OWSLAVE_IOPIN);

	// Wait for the slot to open.
	loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);

	// Wait until we should sample.
	_delay_us(30);

	// Return the value of the bit.
	return bit_is_set(PINB, OWSLAVE_IOPIN);
}

static void
owslave_write_bit(uint8_t v) {
	// Wait for the bus to go idle.
	if(bit_is_clear(PINB, OWSLAVE_IOPIN))
		loop_until_bit_is_set(PINB, OWSLAVE_IOPIN);

	if(v == 0) {
#if OWSLAVE_SUPPORTS_CONVERT_INDICATOR
		owslave_begin_busy();
		loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);
		loop_until_bit_is_set(PINB, OWSLAVE_IOPIN);
		owslave_end_busy();
#else
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
#endif
	} else {
		// Wait for the slot to open.
		loop_until_bit_is_clear(PINB, OWSLAVE_IOPIN);
	}
}

static uint8_t
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

static void
owslave_write_byte(uint8_t byte) {
	for(uint8_t i = 0; i != 8; i++) {
		owslave_write_bit(byte & 1);
		byte >>= 1;
	}
}

static inline uint16_t
owslave_read_word() {
	return owslave_read_byte() + (owslave_read_byte() << 8);
}

static inline void
owslave_write_word(uint16_t x) {
	owslave_write_byte(x);
	owslave_write_byte(x >> 8);
}

// These next three lines help clean out some,
// but not all, of the C boilerplate cruft. This
// saves a few dozen bytes without affecting behavior.
extern void __do_clear_bss(void) __attribute__ ((naked));
extern void main (void) __attribute__ ((naked)) __attribute__ ((section (".init8")));
void __do_clear_bss() { }

void
main(void) {
	uint8_t cmd;
	uint8_t flags;

	// Stop the timer, if it happens to be running.
	TCCR0B = 0;

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

#if SUPPORT_VOLTAGE_READING || EMULATE_DS18B20
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
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

		// Always disable the watchdog, even if we don't use it.
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

	// Let the bus idle for 20µSec after the end of the reset pulse.
	_delay_us(20);

	// Send the 80µSec presence pulse.
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
	else if(cmd != OWSLAVE_ROMCMD_SKIP)
		goto wait_for_reset;

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
		for(uint8_t i = 0; i != (uint8_t)sizeof(device_name); i++) {
			owslave_write_byte(eeprom_read_byte(&device_name[i]));
		}
		owslave_write_byte(0x00);
	} else
#endif
	if((cmd == OWSLAVE_FUNCCMD_RD_MEM)
	    || (cmd == OWSLAVE_FUNCCMD_WR_MEM)
	) {
		// Initialize the CRC by shifting in the command.
		uint16_t crc = _crc16_update(0, cmd);

		// Read in the requested byte address and update the CRC.
		uint8_t i = owslave_read_byte();
		crc = _crc16_update(crc, i);
		owslave_read_byte();
		crc = _crc16_update(crc, 0);

		while(i < 23) {
			uint8_t byte;

			if(cmd == OWSLAVE_FUNCCMD_RD_MEM) {
				// Send the byte to the OW master.
				byte = ((uint8_t*)&value)[i++];
				owslave_write_byte(byte);
			} else {
				// receive the byte from the OW master.
				byte = owslave_read_byte();
			    ((uint8_t*)&value)[i++] = byte;
			}

			// Update the CRC.
			crc = _crc16_update(crc, byte);

			// Write out the CRC at every 8-byte page boundry.
			if((i & 7) == 0) {
				owslave_write_word(crc);
				crc = 0;
			}
		}
	} else if(cmd == OWSLAVE_FUNCCMD_CONVERT) {
		owslave_read_byte();    // Ignore input select mask
		owslave_read_byte();    // Ignore read-out control
		owslave_cb_convert();
	} else if(cmd == OWSLAVE_FUNCCMD_COMMIT_MEM) {
		owslave_begin_busy();
		owslave_cb_commit();
		owslave_end_busy();
	} else if(cmd == OWSLAVE_FUNCCMD_RECALL_MEM) {
		owslave_begin_busy();
		owslave_cb_recall();
		owslave_end_busy();
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

