#include "moist.h"
#include "owslave.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "eeprom.h"

#define sbi(x,y)	x|=(1<<y)
#define cbi(x,y)	x&=~(1<<y)

#define 	ATTR_NO_INIT   __attribute__ ((section (".noinit")))

#define DEVICE_IS_SPACE_CONSTRAINED	(defined (__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__))


struct {
	uint16_t value;
	uint8_t alarm_high;
	uint8_t alarm_low;
	uint8_t config;
	uint8_t calib_range;
	uint8_t calib_offset;
	
	// End DS18B20-compatible zone

	uint8_t voltage;	
} msensor_scratch ATTR_NO_INIT;


void
owslave_cb_convert() {
	uint8_t tmp;
	uint16_t value;

#if defined(__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__)
	// Vref=Vcc, Input=PORTB2
	ADMUX = _BV(ADLAR)|_BV(MUX0);
	cbi(DDRB,2);
	sbi(PORTB,2);
#else
	// Vref=Vcc, Input=Vbg
	ADMUX = _BV(ADLAR)|_BV(REFS1)|_BV(MUX3)|_BV(MUX2);
#endif
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADPS2)|_BV(ADPS1);
	
	value = moist_calc();

	loop_until_bit_is_set(ADCSRA,ADIF);
	sbi(ADCSRA,ADSC);
	loop_until_bit_is_set(ADCSRA,ADIF);
	ADCSRA = 0;
	
	if(!(msensor_scratch.config&CONFIG_FLAG_RAW_VALUE))
	{
		// Apply calibration
		value -= msensor_scratch.calib_offset;
		value *= 256;
		value /= msensor_scratch.calib_range;
		if(value >= 256)
			value = 0xFF;
	}
	
	msensor_scratch.value = value<<8;
	msensor_scratch.voltage = ADCH;
}

bool
owslave_cb_alarm_condition() {
	uint8_t tmp = ((uint8_t*)&msensor_scratch.value)[1];
	return (uint8_t)((uint8_t)(tmp>=msensor_scratch.alarm_high)
		|| (uint8_t)(tmp<=msensor_scratch.alarm_low));
}

uint8_t*
owslave_cb_get_scratch_ptr() {
	return ((uint8_t*)&msensor_scratch);
}

void
owslave_cb_recall() {
	eeprom_busy_wait();
	eeprom_read_block(&msensor_scratch.alarm_high,&msensor_e2.alarm_high,sizeof(msensor_e2));
}

void
owslave_cb_commit() {
	cli();
	eeprom_busy_wait();
	eeprom_write_block(&msensor_scratch.alarm_high,&msensor_e2.alarm_high,sizeof(msensor_e2));
	eeprom_busy_wait();
	sei();
}

void
main(void) {
	if(MCUSR)
	{
		memset(&msensor_scratch,0,sizeof(msensor_scratch));
		owslave_cb_recall();
	}
	MCUSR = 0;

	// Main loop.
	for(;;) owslave_main();
}
